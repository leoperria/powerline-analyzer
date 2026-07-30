/*
 * ESP32-C3 -- mains-voltage acquisition streamed to a PC over native USB
 * ---------------------------------------------------------------------------
 * Strategy for "never miss a sample":
 *   - The ADC runs in CONTINUOUS (DMA) mode. Sampling is driven by hardware at
 *     a fixed rate; the CPU is NOT in the sampling loop, so task/USB jitter
 *     cannot skew or drop samples as long as we drain the DMA pool in time.
 *   - A DMA "conversion frame" completes -> an ISR wakes this drain task.
 *   - The drain task copies samples out, reconstructs the instantaneous mains
 *     voltage from each one (see mains_filter.h), wraps them in a small framed
 *     protocol (see wire_protocol.h), and writes them to the native USB CDC
 *     (USB Serial/JTAG). Acquisition and transmission are decoupled, and the
 *     DMA pool gives headroom against brief USB stalls.
 *   - A hardware pool-overflow event (host too slow) is counted by
 *     adc_setup.c and reported in every frame header, so the PC can *prove*
 *     whether any sample was lost on the device side.
 *
 * Module map:
 *   board_config.h  - ADC pin/rate/buffer knobs
 *   adc_setup.[ch]  - ADC continuous-mode setup, calibration, raw->mV parsing
 *   mains_filter.h  - ADC millivolts -> reconstructed instantaneous mains mV
 *   wire_protocol.h - USB frame format
 *
 * See docs/main_c_commentary.md for a full walkthrough and
 * docs/calibration_history.md for the empirical gain-calibration log.
 *
 * In `idf.py menuconfig`, set Component config -> ESP System Settings ->
 * Channel for console output to "None" or "UART0" (NOT "USB Serial/JTAG"),
 * or ESP-IDF log text will be injected into the binary data stream -- see
 * README.md for details.
 *
 * Tested against the ESP-IDF v5.x `esp_adc/adc_continuous` API.
 */

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/usb_serial_jtag.h"

#include "board_config.h"
#include "adc_setup.h"
#include "mains_filter.h"
#include "wire_protocol.h"

static const char *TAG = "main";

/* Drain the DMA pool and stream framed sample blocks over native USB. */
static void stream_task(void *arg) {
    /* 4-byte aligned: results are read back as 32-bit adc_digi_output_data_t.
       The C standard only guarantees 1-byte alignment for a uint8_t array, and
       an unaligned 32-bit load faults (LoadStoreAlignment) on the RV32 C3. */
    static __attribute__((aligned(4))) uint8_t raw[READ_LEN];      /* raw DMA results */
    static int adc_mv[SAMPLES_PER_FRAME];                          /* calibrated ADC millivolts */
    static int32_t samples[SAMPLES_PER_FRAME];                     /* reconstructed instantaneous mains millivolts */
    static uint8_t frame[FRAME_HEADER_LEN + sizeof(samples)];      /* header + payload */

    adc_continuous_handle_t adc = nullptr;
    adc_cali_handle_t cali = nullptr;
    ESP_ERROR_CHECK(adc_setup_start(xTaskGetCurrentTaskHandle(), &adc, &cali));

    mains_filter_t filt;
    mains_filter_init(&filt);

    ESP_LOGI(TAG, "streaming %d Hz on ADC1 ch%d", SAMPLE_RATE_HZ, ADC_CHANNEL_SEL);

    uint32_t seq = 0;

    for (;;) {
        /* Wait for a "frame ready" nudge from the ISR. Timeout lets us loop
           even if the host stalls the stream (so overflow keeps being counted). */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        uint32_t got = 0;

        /* Drain everything currently buffered (timeout 0 = return immediately). */
        while (adc_continuous_read(adc, raw, READ_LEN, &got, 0) == ESP_OK) {
            uint32_t k = adc_setup_extract_mv(cali, raw, got, adc_mv, SAMPLES_PER_FRAME);
            if (k == 0) continue;

            for (uint32_t i = 0; i < k; i++) {
                samples[i] = mains_filter_step(&filt, adc_mv[i]);
            }

            wire_pack_header(frame, seq, adc_setup_overflow_count(), (uint16_t) k);
            memcpy(&frame[FRAME_HEADER_LEN], samples, k * sizeof(samples[0]));

            int wlen = FRAME_HEADER_LEN + (int) (k * sizeof(samples[0]));
            /* Native USB CDC. If the host is reading (it easily keeps up at
               ~20 kB/s over a 12 Mbps link) this returns instantly. On a hard
               host stall it may write short; the PC re-syncs on the magic word. */
            usb_serial_jtag_write_bytes(frame, wlen, pdMS_TO_TICKS(20));
            seq++;
        }
    }
}

void app_main() {
    /* Native USB CDC for the data stream. Give TX plenty of room so a whole
       frame always fits without a partial write in the common case. */
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 4096,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    xTaskCreate(stream_task, "adc_stream", 4096, nullptr, 10, nullptr);
}
