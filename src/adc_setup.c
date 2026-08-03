#include "adc_setup.h"

#include "esp_check.h"   /* ESP_RETURN_ON_ERROR */
#include "esp_attr.h"    /* IRAM_ATTR */
#include "esp_adc/adc_cali_scheme.h"

/* Older IDF calls the top attenuation step DB_11; newer IDF renamed it to
   DB_12. Provide bidirectional aliasing so this compiles on both. */
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

static const char *TAG = "adc_setup";
static volatile uint32_t s_pool_ovf = 0; /* incremented in ISR when DMA pool overflows */

/* ---- ISR: one DMA conversion frame is ready -> wake the drain task ------- */
static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                   const adc_continuous_evt_data_t *edata,
                                   void *user_data) {
    BaseType_t must_yield = pdFALSE;
    vTaskNotifyGiveFromISR((TaskHandle_t) user_data, &must_yield);
    return must_yield == pdTRUE;
}

/* ---- ISR: DMA pool overflowed (we fell behind) -> count it --------------- */
static bool IRAM_ATTR on_pool_ovf(adc_continuous_handle_t handle,
                                  const adc_continuous_evt_data_t *edata,
                                  void *user_data) {
    s_pool_ovf++;
    return false;
}

uint32_t adc_setup_overflow_count(void) {
    return s_pool_ovf;
}

/* Create the ADC calibration handle used to convert raw codes -> millivolts.
 * The raw ADC transfer function is not perfectly linear (especially at
 * ADC_ATTEN_DB_12), so we use ESP-IDF's calibration scheme (curve fitting,
 * falling back to line fitting on chips that don't support curve fitting)
 * instead of a naive raw*Vref/4095 formula. This reads factory eFuse
 * calibration data baked into every chip. */
static esp_err_t adc_cali_start(adc_cali_handle_t *out_cali) {
    adc_cali_handle_t cali = nullptr;
    esp_err_t ret = ESP_FAIL;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve_cfg = {
        .unit_id = ADC_UNIT_SEL,
        .atten = ADC_ATTEN_SEL,
        .bitwidth = ADC_BITWIDTH_SEL,
#if SOC_ADC_CALIBRATION_V1_SUPPORTED
        .chan = ADC_CHANNEL_SEL, /* ESP32-C3 (and other V1-calibration SoCs) calibrate per channel. */
#endif
    };
    ret = adc_cali_create_scheme_curve_fitting(&curve_cfg, &cali);
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (ret != ESP_OK) {
        adc_cali_line_fitting_config_t line_cfg = {
            .unit_id = ADC_UNIT_SEL,
            .atten = ADC_ATTEN_SEL,
            .bitwidth = ADC_BITWIDTH_SEL,
        };
        ret = adc_cali_create_scheme_line_fitting(&line_cfg, &cali);
    }
#endif

    ESP_RETURN_ON_ERROR(ret, TAG, "adc_cali_create_scheme");
    *out_cali = cali;
    return ESP_OK;
}

esp_err_t adc_setup_start(TaskHandle_t notify_task,
                           adc_continuous_handle_t *out_adc,
                           adc_cali_handle_t *out_cali) {
    adc_continuous_handle_t handle = nullptr;

    adc_continuous_handle_cfg_t handle_cfg = {
        .max_store_buf_size = POOL_SIZE, /* if this pool fills, new results are dropped */
        .conv_frame_size = READ_LEN, /* ISR fires once per this many bytes */
    };
    ESP_RETURN_ON_ERROR(adc_continuous_new_handle(&handle_cfg, &handle), TAG, "new_handle");

    adc_digi_pattern_config_t pattern = {
        .atten = ADC_ATTEN_SEL,
        .channel = ADC_CHANNEL_SEL,
        .unit = ADC_UNIT_SEL,
        .bit_width = ADC_BITWIDTH_SEL,
    };

    adc_continuous_config_t dig_cfg = {
        .pattern_num = 1,
        .adc_pattern = &pattern,
        .sample_freq_hz = SAMPLE_RATE_HZ,
        .conv_mode = ADC_CONV_SINGLE_UNIT_1,
        .format = ADC_OUTPUT_TYPE,

    };
    ESP_RETURN_ON_ERROR(adc_continuous_config(handle, &dig_cfg), TAG, "config");

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_conv_done,
        .on_pool_ovf = on_pool_ovf,
    };
    ESP_RETURN_ON_ERROR(adc_continuous_register_event_callbacks(handle, &cbs, notify_task),
                        TAG, "register_cb");

    ESP_RETURN_ON_ERROR(adc_continuous_start(handle), TAG, "start");

    ESP_RETURN_ON_ERROR(adc_cali_start(out_cali), TAG, "cali_start");

    *out_adc = handle;
    return ESP_OK;
}

uint32_t adc_setup_extract_mv(adc_cali_handle_t cali, const uint8_t *raw, uint32_t raw_len,
                               int *out_mv, uint32_t out_cap) {
    uint32_t n = raw_len / SOC_ADC_DIGI_RESULT_BYTES;
    uint32_t k = 0;

    for (uint32_t i = 0; i < n && k < out_cap; i++) {
        const adc_digi_output_data_t *p =
                (const adc_digi_output_data_t *) &raw[i * SOC_ADC_DIGI_RESULT_BYTES];
        if (ADC_GET_CHANNEL(p) != ADC_CHANNEL_SEL) continue;

        int raw_code = (int) (ADC_GET_DATA(p) & 0x0FFF);
        int mv = 0;
        /* adc_cali_raw_to_voltage() only fails on an invalid raw value, which
           cannot happen here (masked to 12 bits); on any unexpected error,
           fall back to a straight-line approximation rather than silently
           sending stale data. */
        if (adc_cali_raw_to_voltage(cali, raw_code, &mv) != ESP_OK) {
            mv = (raw_code * 3100) / 4095;
        }
        out_mv[k++] = mv;
    }
    return k;
}
