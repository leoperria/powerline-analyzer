/*
 * Mains-voltage reconstruction.
 *
 * The ADC does not see the mains directly: it sees the output of a sensing
 * front-end (step-down transformer -> 122k/22nF RC filter -> 3.3k/10uF/2.2k
 * bias network centered at ~half of the 3.3V rail, with +/-0.7V-past-rail
 * clamp diodes for overvoltage protection). At the 50 Hz mains frequency
 * this network is, to good approximation, a fixed linear gain plus a fixed
 * DC bias, so the instantaneous mains voltage can be recovered on-device
 * from a single (noise-filtered) ADC reading:
 *
 *     v_mains(t) = (v_adc_filtered(t) - dc_estimate(t)) * MAINS_INV_GAIN
 *
 * where dc_estimate(t) is a live-tracked estimate of the DC bias (see
 * "adaptive DC-bias tracking" below), not a fixed constant, and
 * v_adc_filtered(t) has already been through the anti-noise low-pass (see
 * "anti-noise smoothing filter" below).
 *
 * Full circuit derivation: docs/main_c_commentary.md
 * Full empirical calibration log: docs/calibration_history.md
 *
 *   - Transformer ratio                 117:880 -> 0.1329545
 *   - RC filter + bias-network gain |H| at 50 Hz -> 0.0270392
 *   - Combined gain  Vadc_ac / Vmains_ac = 0.1329545 * 0.0270392 = 0.00359498
 *   - DC bias point at the ADC node (no signal): ~1.5926 V = 1592.6 mV
 *
 * At 240 Vrms max mains input this reproduces the target output range of
 * approximately -339400 mV .. +339400 mV.
 *
 * CALIBRATION: the theoretical gain above is only as good as nominal
 * component values (resistor/cap tolerances on the divider/filter network
 * easily contribute a few percent of error). MAINS_CAL_FACTOR corrects that
 * residual error empirically, from a single reference point: with the current
 * factor flashed, compare the device's reported Vrms against a multimeter and
 * scale by (true Vrms / reported Vrms). The current value was set against a
 * multimeter reading of 205 V while the device reported 204 V, so it is exact
 * at ~205 V and drifts slightly at other grid voltages -- just repeat the
 * measurement to recalibrate for the range you care about. See
 * docs/calibration_history.md for the full trial-by-trial history.
 * Re-derive it for each physically distinct board/transformer, and again any
 * time MAINS_LPF_CUTOFF_HZ changes (a new cutoff attenuates the real
 * waveform's harmonics by a different amount, which is a new systematic
 * shift). */
#pragma once

#include <math.h>
#include <stdint.h>

#include "board_config.h"    /* SAMPLE_RATE_HZ */

#define MAINS_GAIN_VADC_PER_VIN   0.00359498f    /* Vadc_ac / Vmains_ac, dimensionless (theoretical, from component values) */
#define MAINS_CAL_FACTOR          1.006331f      /* empirical correction, see docs/calibration_history.md */
#define MAINS_INV_GAIN            ((1.0f / MAINS_GAIN_VADC_PER_VIN) * MAINS_CAL_FACTOR) /* mV mains per mV of ADC AC swing, calibration-corrected */
#define VADC_DC_BIAS_MV           1592.6f        /* ADC node's no-signal DC bias point, in mV -- theoretical value,
                                                     used only to SEED the adaptive tracker below; the actual bias
                                                     is measured on-device (see MAINS_DC_TRACK_ALPHA) because
                                                     resistor tolerances on the 2.2k/2.2k/3.3k bias divider shift
                                                     the true no-signal bias by a percent or more. */

/* ---- adaptive DC-bias tracking ---------------------------------------------
 * Rather than trust a fixed theoretical bias (wrong by however much the
 * 2.2k/2.2k/3.3k divider resistors deviate from nominal), track the ADC
 * node's actual DC operating point live with an exponential moving average
 * (a 1-pole low-pass) and subtract *that* before applying the mains gain.
 * This self-calibrates away resistor tolerance, ADC offset error, and slow
 * thermal drift, with no hard-coded assumption about the divider's exact
 * midpoint. It works because the 50 Hz AC signal riding on the bias
 * averages to ~0 over full mains cycles, so a low-pass filter with a cutoff
 * far below 50 Hz converges on the true DC bias regardless of whether mains
 * is present, absent, or its amplitude changes.
 *
 * MAINS_DC_TRACK_ALPHA sets the filter's cutoff: alpha = dt / tau, with
 * dt = 1/SAMPLE_RATE_HZ and a time constant tau chosen well above one mains
 * period (20 ms) so it doesn't distort the 50 Hz waveform, but short enough
 * to settle in a few seconds after power-up. tau = 1 s -> cutoff ~0.16 Hz,
 * about 300x below the 50 Hz mains frequency. */
#define MAINS_DC_TRACK_TAU_S      1.0f
#define MAINS_DC_TRACK_ALPHA      (1.0f / (SAMPLE_RATE_HZ * MAINS_DC_TRACK_TAU_S))

/* ---- anti-noise smoothing filter --------------------------------------------
 * Why bias tracking alone can't zero out a floating/disconnected input:
 * MAINS_INV_GAIN is ~280x (needed because the front-end attenuates a 240 Vrms
 * mains swing down to only ~2.4 Vpp at the ADC). That means every single
 * ADC code step (1 LSB = ~0.76 mV at ADC_ATTEN_DB_12/12-bit) is amplified to
 * ~210 mV in the reported mains value. A routine ADC code jitter of only
 * 6-7 LSBs rms -- from ADC quantization/thermal noise, USB/CPU switching
 * noise, or ambient 50 Hz EMI capacitively/inductively picked up by the
 * transformer secondary once it's floating (open primary) -- fully explains
 * a multi-volt residual with no mains connected. DC-bias tracking removes
 * only the *average* (DC) component; it cannot remove this AC-ish noise/
 * pickup, and note that ambient EMI pickup happens to land right at 50 Hz,
 * the same frequency as the real signal, so it is NOT separable from a true
 * reading by frequency-selective filtering either.
 *
 * What this DOES fix: broadband, higher-frequency noise (ADC quantization,
 * digital switching noise) outside the handful of harmonics that matter for
 * a mains waveform. A simple 1-pole low-pass ahead of the gain stage, cut
 * well above the 50 Hz fundamental (preserving several harmonics for
 * waveform fidelity) but well below the ADC's ~5 kHz Nyquist, reduces that
 * broadband contribution. It will NOT fully zero an open/floating input --
 * that is a real hardware/physics limit of this high-gain sensing front
 * end, not a firmware bug. Changing this cutoff shifts how much of the real
 * waveform's harmonics get attenuated too, so re-run the calibration in
 * docs/calibration_history.md whenever it changes. */
#define MAINS_LPF_CUTOFF_HZ       1000.0f
#define MAINS_LPF_ALPHA           (1.0f / (1.0f + (SAMPLE_RATE_HZ / (2.0f * (float) M_PI * MAINS_LPF_CUTOFF_HZ))))

/* Per-channel filter state: anti-noise low-pass feeding an adaptive DC-bias
 * tracker, both seeded at the theoretical bias so early samples aren't
 * wildly off before they converge. */
typedef struct {
    float lpf_estimate;
    float dc_estimate;
} mains_filter_t;

static inline void mains_filter_init(mains_filter_t *f) {
    f->lpf_estimate = VADC_DC_BIAS_MV;
    f->dc_estimate = VADC_DC_BIAS_MV;
}

/* Feed one calibrated ADC millivolt reading in; get the reconstructed
 * instantaneous mains millivolt reading out. Order of operations: smooth
 * out broadband noise first, track the (now-smoothed) DC bias, then undo
 * the sensing front-end's gain. */
static inline int32_t mains_filter_step(mains_filter_t *f, int adc_mv) {
    f->lpf_estimate += ((float) adc_mv - f->lpf_estimate) * MAINS_LPF_ALPHA;
    f->dc_estimate += (f->lpf_estimate - f->dc_estimate) * MAINS_DC_TRACK_ALPHA;
    float mains_mv = (f->lpf_estimate - f->dc_estimate) * MAINS_INV_GAIN;
    return (int32_t) lroundf(mains_mv);
}
