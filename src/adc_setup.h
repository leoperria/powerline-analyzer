/*
 * ADC hardware setup: continuous (DMA) mode configuration, calibration
 * (raw code -> millivolts), and the ISR-driven pool-overflow counter.
 *
 * This module knows about ADC hardware quirks (chip-specific result layout,
 * attenuation naming differences across IDF versions, calibration scheme
 * availability) so the rest of the firmware doesn't have to.
 */
#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"

#include "board_config.h"

/* Create, configure and start the continuous ADC plus its calibration
 * handle. `notify_task` is woken (via task notification) once per completed
 * DMA conversion frame -- see on_conv_done() in adc_setup.c. */
esp_err_t adc_setup_start(TaskHandle_t notify_task,
                           adc_continuous_handle_t *out_adc,
                           adc_cali_handle_t *out_cali);

/* Cumulative count of hardware DMA pool overflows (host too slow to drain in
 * time) observed so far. Safe to call from any task. */
uint32_t adc_setup_overflow_count(void);

/* Parse a raw DMA read buffer (as filled by adc_continuous_read()) into
 * calibrated millivolt readings, keeping only entries for ADC_CHANNEL_SEL.
 * Writes at most out_cap values to out_mv and returns how many were written. */
uint32_t adc_setup_extract_mv(adc_cali_handle_t cali, const uint8_t *raw, uint32_t raw_len,
                               int *out_mv, uint32_t out_cap);
