# Mains gain calibration history

`MAINS_CAL_FACTOR` (in `src/mains_filter.h`) corrects the theoretical sensing
front-end gain for real-world component tolerance. The theoretical gain is
only as good as nominal resistor/capacitor values, which easily contribute a
few percent of error. This constant is derived empirically: with the
firmware running, capture a CSV, compute the RMS of the reported mains
voltage, and compare it to a trusted multimeter reading of the same outlet.

**Method — average across trials, don't compound factors.** Naively
multiplying in `(true / measured)` from only the *latest* trial chases noise:
real mains voltage drifts by a volt or two between measurements (grid load
changes), and a single quick multimeter/CSV comparison is not more precise
than that drift.

For each trial, divide the measured value by the `MAINS_CAL_FACTOR` that was
active at the time to recover the underlying *raw* (uncalibrated) reading,
then compute that trial's implied correction ratio:

```
raw_i   = measured_i / factor_active_at_the_time
ratio_i = true_i / raw_i            <- the factor that would have been exactly right for trial i
```

The best estimate is then the **mean of the ratios** across all trials in the
current epoch:

```
MAINS_CAL_FACTOR = mean(ratio_i)
```

Averaging the *ratio* rather than the raw voltage matters as soon as trials
are taken at different times of day: the grid voltage itself moves (this
project has seen anything from 204 V to 221 V at the same outlet), so raw
readings from different sessions aren't directly comparable, while the
ratio — a property of the sensing hardware, not of the grid — is. On trials
taken at the *same* mains voltage the two approaches agree to ~0.001%, so
this is a strict generalization of the earlier "average the raw readings"
method.

Any time a filter parameter changes (e.g. `MAINS_LPF_CUTOFF_HZ`), start a
**fresh calibration epoch**: a different cutoff attenuates the real
waveform's harmonics by a different amount, which is a new systematic shift,
not measurement noise, so it shouldn't be averaged in with trials taken
under the old filter settings.

## Epoch 1 — before the anti-noise low-pass filter existed

| iter | factor in effect | measured Vrms | true Vrms | raw (measured / factor) |
|------|-------------------|----------------|-----------|--------------------------|
| 1 | 1.000000 | 218.00 | 220.00 | 218.00 |
| 2 | 1.009174 | 219.49 | 220.00 | 217.50 |
| 3 | 1.011519 | 222.15 | 220.00 | 219.62 — overshot iter 2, confirms it's noise, not a bug |

avg raw = (218.00 + 217.50 + 219.62) / 3 = **218.37**
`MAINS_CAL_FACTOR = 220.00 / 218.37 = 1.007457`

| iter | factor in effect | measured Vrms | true Vrms | raw |
|------|-------------------|----------------|-----------|-----|
| 4 | 1.007457 | ~222.44 (19 one-second reports averaged, ~1 min capture) | 220.00 | 220.77 |

A consistent 1.1% high reading across 19 samples over ~1 minute isn't
one-off noise, so it's folded into the running average as another raw trial:

avg raw = (218.00 + 217.50 + 219.62 + 222.44/1.007457) / 4 = **220.87**
`MAINS_CAL_FACTOR = 220.00 / 220.87 = 0.996415`

## Epoch 2 — after adding the anti-noise low-pass filter (`MAINS_LPF_CUTOFF_HZ`)

The filter attenuates a bit of the real mains waveform's higher harmonics (a
household outlet is not a pure 50 Hz sine — switching loads on the same
circuit put several percent of THD onto it), which lowers the reported RMS
slightly. That's a new, filter-dependent systematic shift, so calibration
restarts from a single fresh trial rather than reusing the epoch 1 raw
average.

| iter | factor in effect | measured Vrms | true Vrms | raw | ratio (true/raw) |
|------|-------------------|----------------|-----------|-----|-------------------|
| 5 | 1.004485 (carried over from epoch 1) | ~218.43 (10 one-second reports) | 220.20 | 219.22 | 1.004470 |
| 6 | 1.004485 | ~221.86 (9 one-second reports) | 220.20 | 220.87 | 0.996967 |
| 7 | 1.000703 | ~221.08 (9 one-second reports) | 220.20 | 220.92 | 0.996741 |
| 8 | 0.999379 | ~220.88 (7 one-second reports) | 220.20 | 221.01 | 0.996335 |
| 9 | 0.998611 | ~202.15 (39 one-second reports, ~40 s capture) | ~204.00 | 202.43 | 1.007756 |
| 10 | 1.000453 | ~203.82 (19 one-second reports, ~20 s capture) | ~205.00 | 203.72 | 1.006263 |

Iterations 5–8 all happened at essentially the same grid voltage
(220.20 Vrms), so at the time the factor was recomputed as
`true / mean(raw)`:

- iter 5: raw = 218.43 / 0.996415 = 219.22 &rarr; factor = 220.20 / 219.22 = **1.004485**
- iter 6: raw = 221.86 / 1.004485 = 220.87; avg(219.22, 220.87) = 220.05 &rarr; factor = 220.20 / 220.05 = **1.000703**
- iter 7: raw = 221.08 / 1.000703 = 220.92; avg(219.22, 220.87, 220.92) = 220.34 &rarr; factor = 220.20 / 220.34 = **0.999379**
- iter 8: raw = 220.88 / 0.999379 = 221.01; avg(219.22, 220.87, 220.92, 221.01) = 220.51 &rarr; factor = 220.20 / 220.51 = **0.998611**

Four trials in, the raw values (219.22..221.01 Vrms) spanned less than 2 Vrms
with no clear trend — normal grid voltage variation, not a residual
systematic error. A follow-up live session then read 220.72–221.82 Vrms
against a multimeter reading of 221.5 V (mains had drifted up), confirming
the calibration held within ordinary noise/drift once you account for the
grid not sitting still.

**Iteration 9 was taken on a low-voltage day (~204 V at the outlet), which is
what forced the switch to averaging ratios** rather than raw voltages — a raw
reading of 202.43 V isn't comparable to the ~220 V raw readings above, but
its ratio is. Iteration 10 followed on the same low-voltage day (~205 V).

mean ratio (all 6) = (1.004470 + 0.996967 + 0.996741 + 0.996335 + 1.007756 + 1.006263) / 6 = **1.001422**
`MAINS_CAL_FACTOR = 1.001422` (superseded — see iteration 11 below)

### Iteration 11 — reset to a single-point calibration at ~205 V

Averaging across trials was abandoned at the user's request. The device is
now calibrated directly against one operating point: multimeter **205 V** vs
device **204 V**, with `MAINS_CAL_FACTOR = 1.001422` active during that
capture.

    MAINS_CAL_FACTOR = 1.001422 x (205 / 204) = 1.006331

`MAINS_CAL_FACTOR = 1.006331` (current)

This is exact at ~205 V by construction. It sits ~0.77% above the mean ratio
of the four ~220 V trials (0.998628), so expect the device to read roughly
0.8% high if the grid returns to ~220 V. Recalibrate the same way against a
fresh multimeter reading if you care about accuracy at a different operating
point, or if `MAINS_LPF_CUTOFF_HZ` (or any other filter parameter) changes.

## Current values

```
MAINS_CAL_FACTOR = 1.006331f
```

See `src/mains_filter.h` for how this combines with the theoretical gain, and
`docs/main_c_commentary.md` for the full circuit derivation and the
adaptive DC-bias tracker / anti-noise filter that this calibration sits on
top of.
