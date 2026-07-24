/*
 * ESP32-C3 — 44.1 kHz ADC acquisition streamed to a PC over native USB
 * ---------------------------------------------------------------------------
 * Strategy for "never miss a sample":
 *   - The ADC runs in CONTINUOUS (DMA) mode. Sampling is driven by hardware at
 *     a fixed rate; the CPU is NOT in the sampling loop, so task/USB jitter
 *     cannot skew or drop samples as long as we drain the DMA pool in time.
 *   - A DMA "conversion frame" completes -> an ISR wakes a drain task.
 *   - The drain task copies samples out, wraps them in a small framed protocol
 *     (magic + sequence + overflow counter + count + payload), and writes them
 *     to the native USB CDC (USB Serial/JTAG). Acquisition and transmission are
 *     decoupled, and the DMA pool gives headroom against brief USB stalls.
 *   - A hardware pool-overflow event (host too slow) is counted in the ISR and
 *     reported in every frame header, so the PC can *prove* whether any sample
 *     was lost on the device side.
 *
 * Wiring / config notes are at the bottom of this file and in the README.
 *
 * Tested against the ESP-IDF v5.x `esp_adc/adc_continuous` API. Not run on
 * hardware here — flash, then verify with the accompanying pc_reader.py.
 */

#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"   /* ESP_RETURN_ON_ERROR */
#include "esp_attr.h"
#include "esp_adc/adc_continuous.h"
#include "driver/usb_serial_jtag.h"

/* ------------------------------ configuration ---------------------------- */
#define ADC_UNIT_SEL        ADC_UNIT_1          /* Use ADC1 (ADC2 conflicts with Wi-Fi). */
#define ADC_CHANNEL_SEL     +   -       /* GPIO3 on ESP32-C3. Avoid strapping pins GPIO2/8/9. */
#define ADC_ATTEN_SEL       ADC_ATTEN_DB_12     /* ~0..3.1 V full scale. */
#define ADC_BITWIDTH_SEL    ADC_BITWIDTH_12     /* ESP32-C3 SAR ADC is 12-bit. */

#define SAMPLE_RATE_HZ      48000               /* Target rate. C3 continuous range ~611..100000 Hz (IDF 5.x).
                                                   NOTE: the C3 ADC clock is APB(80 MHz)/integer_divider, so
                                                   only rates of the form 80e6/N are actually achievable. Asking
                                                   for 44100 lands on divider N=1792 => real rate 44642.857 Hz
                                                   (+1.23%). The PC reader displays the *actual* rate; treat that
                                                   as ground truth for any downstream FFT/timing. If you need it
                                                   closer to 44100, try SAMPLE_RATE_HZ=44050 (often maps to
                                                   N=1814 => 44101.4 Hz). */
#define SAMPLES_PER_FRAME   256                 /* DMA conversion-frame size, in samples (~5.8 ms @44.1k). */
#define POOL_FRAMES         8                   /* DMA pool depth in frames = headroom vs. USB stalls
                                                   (~46 ms of raw DMA output @44.1k). */

/* Framing */
#define FRAME_MAGIC0        0xA5
#define FRAME_MAGIC1        0x5A
#define FRAME_HEADER_LEN    10                  /* magic(2)+seq(4)+ovf(2)+count(2) */

/* ------------------------------ derived ---------------------------------- */
#define READ_LEN            (SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES)
#define POOL_SIZE           (READ_LEN * POOL_FRAMES)

/* Older IDF calls the top attenuation step DB_11; newer IDF renamed it to DB_12.
   Provide bidirectional aliasing so this compiles on both. */
#if !defined(ADC_ATTEN_DB_12) && defined(ADC_ATTEN_DB_11)
#define ADC_ATTEN_DB_12 ADC_ATTEN_DB_11
#elif !defined(ADC_ATTEN_DB_11) && defined(ADC_ATTEN_DB_12)
#define ADC_ATTEN_DB_11 ADC_ATTEN_DB_12
#endif

/* Result layout differs by chip: ESP32/S2 = TYPE1, everything else (incl. C3) = TYPE2. */
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE1
#define ADC_GET_CHANNEL(p)  ((p)->type1.channel)
#define ADC_GET_DATA(p)     ((p)->type1.data)
#else
#define ADC_OUTPUT_TYPE     ADC_DIGI_OUTPUT_FORMAT_TYPE2
#define ADC_GET_CHANNEL(p)  ((p)->type2.channel)
#define ADC_GET_DATA(p)     ((p)->type2.data)
#endif

static const char *TAG = "adc_stream";
static volatile uint32_t s_pool_ovf = 0;   /* incremented in ISR when DMA pool overflows */

/* ---- ISR: one DMA conversion frame is ready -> wake the drain task ------- */
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                   const adc_continuous_evt_data_t *edata,
                                   void *user_data)
{
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t)user_data, &must_yield);
    return must_yield == pdTRUE;
}

/* ---- ISR: DMA pool overflowed (we fell behind) -> count it --------------- */
static bool IRAM_ATTR on_pool_ovf(adc_continuous_handle_t handle,
                                  const adc_continuous_evt_data_t *edata,
                                  void *user_data)
{
    s_pool_ovf++;
    return false;
}

/* Create, configure, hook callbacks and start the continuous ADC. */
static esp_err_t adc_start(adc_continuous_handle_t *out_handle, TaskHandle_t notify_task)
{
    adc_continuous_handle_t handle = nullptr;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = POOL_SIZE,   /* if this pool fills, new results are dropped */
        .conv_frame_size    = READ_LEN,    /* ISR fires once per this many bytes */
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&handle_cfg, &handle), TAG, "new_handle");

    adc_digi_pattern_config_t pattern = {
        .atten     = ADC_ATTEN_SEL,
        .channel   = ADC_CHANNEL_SEL,
        .unit      = ADC_UNIT_SEL,
        .bit_width = ADC_BITWIDTH_SEL,
    };

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        .format         = ADC_OUTPUT_TYPE,
        .pattern_num    = 1,
        .adc_pattern    = &pattern,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_config(handle, &dig_cfg), TAG, "config");

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_conv_done,
        .on_pool_ovf  = on_pool_ovf,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_register_event_callbacks(handle, &cbs, notify_task),
                        TAG, "register_cb");

    ESP_RETURN_ON_ERROR(adc_continuous_start(handle), TAG, "start");
    *out_handle = handle;
    return ESP_OK;
}

/* Drain the DMA pool and stream framed sample blocks over native USB. */
static void stream_task(void *arg)
{
    /* 4-byte aligned: results are read back as 32-bit adc_digi_output_data_t.
       The C standard only guarantees 1-byte alignment for a uint8_t array, and
       an unaligned 32-bit load faults (LoadStoreAlignment) on the RV32 C3. */
    static __attribute__((aligned(4))) uint8_t raw[READ_LEN];  /* raw DMA results */
    static uint16_t samples[SAMPLES_PER_FRAME];                 /* extracted 12-bit values */
    static uint8_t  frame[FRAME_HEADER_LEN + sizeof(samples)];  /* header + payload */

    adc_continuous_handle_t adc = nullptr;
    ESP_ERROR_CHECK(adc_start(&adc, xTaskGetCurrentTaskHandle()));
    ESP_LOGI(TAG, "streaming %d Hz on ADC1 ch%d", SAMPLE_RATE_HZ, ADC_CHANNEL_SEL);

    uint32_t seq = 0;

    for (;;) {
        /* Wait for a "frame ready" nudge from the ISR. Timeout lets us loop
           even if the host stalls the stream (so overflow keeps being counted). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        uint32_t got = 0;
        /* Drain everything currently buffered (timeout 0 = return immediately). */
        while (adc_continuous_read(adc, raw, READ_LEN, &got, 0) == ESP_OK) {
            uint32_t n = got / SOC_ADC_DIGI_RESULT_BYTES;
            uint32_t k = 0;

            for (uint32_t i = 0; i < n; i++) {
                adc_digi_output_data_t *p =
                    (adc_digi_output_data_t *)&raw[i * SOC_ADC_DIGI_RESULT_BYTES];
                if (ADC_GET_CHANNEL(p) == ADC_CHANNEL_SEL) {
                    samples[k++] = (uint16_t)(ADC_GET_DATA(p) & 0x0FFF);
                }
            }
            if (k == 0) continue;

            uint32_t ovf = s_pool_ovf;

            frame[0] = FRAME_MAGIC0;
            frame[1] = FRAME_MAGIC1;
            frame[2] = (uint8_t)(seq);
            frame[3] = (uint8_t)(seq >> 8);
            frame[4] = (uint8_t)(seq >> 16);
            frame[5] = (uint8_t)(seq >> 24);
            frame[6] = (uint8_t)(ovf);
            frame[7] = (uint8_t)(ovf >> 8);
            frame[8] = (uint8_t)(k);
            frame[9] = (uint8_t)(k >> 8);
            memcpy(&frame[FRAME_HEADER_LEN], samples, k * sizeof(uint16_t));

            int wlen = FRAME_HEADER_LEN + (int)(k * sizeof(uint16_t));
            /* Native USB CDC. If the host is reading (it easily keeps up at
               ~20 kB/s over a 12 Mbps link) this returns instantly. On a hard
               host stall it may write short; the PC re-syncs on the magic word. */
            usb_serial_jtag_write_bytes(frame, wlen, pdMS_TO_TICKS(20));
            seq++;
        }
    }
}

void app_main()
{
    /* Native USB CDC for the data stream. Give TX plenty of room so a whole
       frame always fits without a partial write in the common case. */
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 4096,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    xTaskCreate(stream_task, "adc_stream", 4096, nullptr, 10, nullptr);
}

/* ---------------------------------------------------------------------------
 * BUILD / CONFIG (idf.py menuconfig):
 *   Component config -> ESP System Settings -> Channel for console output:
 *       set to "None" (cleanest) or "UART0". Do NOT leave it on
 *       "USB Serial/JTAG", or ESP-IDF log text will be injected into your
 *       binary data stream. (The PC reader re-syncs on the magic word, so the
 *       default is tolerable, but None/UART0 is the clean choice.)
 *
 * WIRE PROTOCOL (little-endian), one frame per DMA conversion block:
 *   byte 0..1  : magic 0xA5 0x5A
 *   byte 2..5  : seq   (uint32) frame counter; a gap => a frame lost in transit
 *   byte 6..7  : ovf   (uint16) cumulative DMA pool-overflow count; if this
 *                       ever increments, the DEVICE dropped samples
 *   byte 8..9  : count (uint16) number of samples in this frame
 *   byte 10..  : count * uint16 raw 12-bit ADC values (0..4095)
 * ------------------------------------------------------------------------- */