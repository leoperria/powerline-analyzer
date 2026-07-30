# Mains gain calibration history

`MAINS_CAL_FACTOR` (in `src/mains_filter.h`) corrects the theoretical sensing
front-end gain for real-world component tolerance. The theoretical gain is
only as good as nominal resistor/capacitor values, which easily contribute a
few percent of error. This constant is derived empirically: with the
firmware running, capture a CSV, compute the RMS of the reported mains
voltage, and compare it to a trusted multimeter reading of the same outlet.

**Method — average raw trials, don't compound factors.** Naively multiplying
in `(true / measured)` from only the *latest* trial chases noise: real mains
voltage drifts by a volt or two between measurements (grid load changes), and
a single quick multimeter/CSV comparison is not more precise than that
drift. Instead, divide each trial's measured value by the `MAINS_CAL_FACTOR`
that was active at the time, to recover the underlying *raw* (uncalibrated)
reading for that trial. Average the raw readings across trials and derive a
single factor from the mean, rather than compounding factor-on-factor.

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

| iter | factor in effect | measured Vrms | true Vrms | raw |
|------|-------------------|----------------|-----------|-----|
| 5 | 1.004485 (carried over from epoch 1) | ~218.43 (10 one-second reports) | 220.20 | 219.22 |
| 6 | 1.004485 | ~221.86 (9 one-second reports) | 220.20 | 220.87 |
| 7 | 1.000703 | ~221.08 (9 one-second reports) | 220.20 | 220.92 |
| 8 | 0.999379 | ~220.88 (7 one-second reports) | 220.20 | 221.01 |

Each iteration recomputes the factor from the mean of *all* epoch-2 raw
trials so far:

- iter 5: raw = 218.43 / 0.996415 = 219.22 &rarr; factor = 220.20 / 219.22 = **1.004485**
- iter 6: raw = 221.86 / 1.004485 = 220.87; avg(219.22, 220.87) = 220.05 &rarr; factor = 220.20 / 220.05 = **1.000703**
- iter 7: raw = 221.08 / 1.000703 = 220.92; avg(219.22, 220.87, 220.92) = 220.34 &rarr; factor = 220.20 / 220.34 = **0.999379**
- iter 8: raw = 220.88 / 0.999379 = 221.01; avg(219.22, 220.87, 220.92, 221.01) = 220.51 &rarr; factor = 220.20 / 220.51 = **0.998611** (current)

Four trials in, the raw values (219.22..221.01 Vrms) span less than 2 Vrms
with no clear trend — normal grid voltage variation, not a residual
systematic error. A follow-up live session then read 220.72–221.82 Vrms
against a multimeter reading of 221.5 V (mains had drifted up), confirming
the calibration holds within ordinary noise/drift once you account for the
grid not sitting still.

**Practical noise floor: treat +/-0.5 Vrms (~0.25%) as the practical limit of
this single-session calibration method.** Further single-session tweaks
below that threshold are overfitting to momentary grid voltage rather than
correcting a real error. Prefer averaging several trials collected over
different times/days for tighter long-term accuracy, and only recalibrate
again if a fresh trial falls clearly outside that band, or if
`MAINS_LPF_CUTOFF_HZ` (or any other filter parameter) changes.

## Current values

```
MAINS_CAL_FACTOR = 0.998611f
```

See `src/mains_filter.h` for how this combines with the theoretical gain, and
`docs/main_c_commentary.md` for the full circuit derivation and the
adaptive DC-bias tracker / anti-noise filter that this calibration sits on
top of.
