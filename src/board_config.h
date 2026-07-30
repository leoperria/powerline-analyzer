/*
 * Board/acquisition configuration: the knobs to tune for your board and signal.
 */
#pragma once

#include "esp_adc/adc_continuous.h"

#define ADC_UNIT_SEL        ADC_UNIT_1          /* Use ADC1 (ADC2 conflicts with Wi-Fi). */
#define ADC_CHANNEL_SEL     ADC_CHANNEL_3       /* GPIO3 on ESP32-C3. Avoid strapping pins GPIO2/8/9. */
#define ADC_ATTEN_SEL       ADC_ATTEN_DB_12     /* ~0..3.1 V full scale. */
#define ADC_BITWIDTH_SEL    ADC_BITWIDTH_12     /* ESP32-C3 SAR ADC is 12-bit. */

#define SAMPLE_RATE_HZ      10000               /* Target rate. C3 continuous range ~611..100000 Hz (IDF 5.x).
                                                   NOTE: the C3 ADC clock is APB(80 MHz)/integer_divider, so
                                                   only rates of the form 80e6/N are actually achievable. Asking
                                                   for 44100 lands on divider N=1792 => real rate 44642.857 Hz
                                                   (+1.23%). The PC reader displays the *actual* rate; treat that
                                                   as ground truth for any downstream FFT/timing. If you need it
                                                   closer to 44100, try SAMPLE_RATE_HZ=44050 (often maps to
                                                   N=1814 => 44101.4 Hz). */
#define SAMPLES_PER_FRAME   256                 /* DMA conversion-frame size, in samples (~25.6 ms @10 kHz). */
#define POOL_FRAMES         8                   /* DMA pool depth in frames = headroom vs. USB stalls
                                                   (~205 ms of raw DMA output @10 kHz). */

/* ------------------------------ derived ---------------------------------- */
#define READ_LEN            (SAMPLES_PER_FRAME * SOC_ADC_DIGI_RESULT_BYTES)
#define POOL_SIZE           (READ_LEN * POOL_FRAMES)
