# Firmware Commentary — `src/`

This document explains how the ESP32-C3 firmware works: an analog mains
sensing front-end feeds the ADC, the device samples it continuously via DMA,
reconstructs the instantaneous mains voltage on-device, and streams it to a
PC over native USB with a framed protocol that lets the host *prove* no
sample was ever dropped.

The firmware is split into a few focused files instead of one large one:

| File | Responsibility |
|---|---|
| `main.c` | Orchestration: `app_main()` + the drain/stream loop |
| `board_config.h` | ADC pin/rate/buffer-size knobs |
| `adc_setup.h` / `adc_setup.c` | ADC continuous-mode setup, calibration, raw→mV parsing, ISRs |
| `mains_filter.h` | ADC millivolts → reconstructed instantaneous mains millivolts |
| `wire_protocol.h` | USB frame format (header packing) |

This document covers the current design and feature set (mains-voltage
reconstruction, gain calibration, adaptive DC-bias tracking, anti-noise
filtering); for the empirical calibration trial log see
`docs/calibration_history.md`. Section references below point to functions
and files, not line numbers, since those move around as the code evolves.

---

## 1. The Big Picture

```
  Mains outlet (up to 240 Vrms)
        │
        ▼
  Sensing front-end (transformer + RC filter + bias network + clamp diodes)
        │  ~2.4 Vpp around a ~1.6 V DC bias, see mains_filter.h
        ▼
  Analog pin (GPIO3)
        │
        ▼
  SAR ADC ── sampled by HARDWARE at a fixed rate (~10 kHz target) ──┐
        │                                                           │
        ▼                                                           │
  DMA "conversion frames" (256 samples each)                        │  no CPU
        │                                                           │  in the
        ▼                                                           │  sampling
  DMA pool (8 frames deep = headroom against USB stalls) ───────────┘  loop
        │
        │  frame complete → ISR fires
        ▼
  adc_setup.c: on_conv_done() ISR → notifies the drain task
        │
        ▼
  main.c: stream_task():
        1. drain the DMA pool                  (adc_continuous_read)
        2. raw code → calibrated ADC mV         (adc_setup_extract_mv)
        3. ADC mV → reconstructed mains mV       (mains_filter_step)
        4. wrap in a frame (magic+seq+ovf+count) (wire_pack_header)
        ▼
  usb_serial_jtag_write_bytes()  → PC (host/pc_reader.py)
```

**Key idea:** the CPU is *not* in the sampling loop. Hardware + DMA sample at
an exact rate, so task scheduling jitter or USB stalls cannot skew or drop
samples — as long as the software drains the DMA pool fast enough. If it
ever *can't* keep up, the hardware raises a **pool overflow**, which is
counted and reported so the PC can *prove* whether data was lost.

---

## 2. `board_config.h` — the knobs

- **`ADC_UNIT_SEL = ADC_UNIT_1`** — ADC2 conflicts with Wi-Fi on ESP32 chips,
  so ADC1 is the safe choice.
- **`ADC_CHANNEL_SEL = ADC_CHANNEL_3`** — GPIO3 on the ESP32-C3. Avoid the
  strapping pins GPIO2/8/9 (they influence boot mode).
- **`ADC_ATTEN_SEL = ADC_ATTEN_DB_12`** — ~0–3.1 V full-scale input range.
- **`ADC_BITWIDTH_SEL = ADC_BITWIDTH_12`** — the C3's SAR ADC is 12-bit
  (values 0–4095).
- **`SAMPLE_RATE_HZ = 10000`** — target sample rate. The C3 ADC clock is
  `APB(80 MHz) / integer_divider`, so only rates of the form `80e6/N` are
  actually achievable; the real rate lands on the nearest divider. The PC
  reader reports the *actual* measured rate — treat that as ground truth for
  any downstream FFT/timing math.
- **`SAMPLES_PER_FRAME = 256`** — size of one DMA conversion frame, i.e. how
  many samples accumulate before the ISR fires.
- **`POOL_FRAMES = 8`** — DMA pool depth in frames; the headroom that absorbs
  brief USB/host stalls without losing samples.
- **`READ_LEN`, `POOL_SIZE`** — derived byte sizes (`READ_LEN = SAMPLES_PER_FRAME
  * SOC_ADC_DIGI_RESULT_BYTES`; `POOL_SIZE = READ_LEN * POOL_FRAMES`).

---

## 3. `adc_setup.h` / `adc_setup.c` — ADC hardware layer

This module owns every ADC hardware quirk so nothing else in the firmware
has to know about them: chip-specific result layout, attenuation naming
differences across IDF versions, and calibration-scheme availability.

### Compatibility shims

- Older IDF names the top attenuation step `ADC_ATTEN_DB_11`; newer IDF
  renamed it to `ADC_ATTEN_DB_12`. A preprocessor block aliases whichever one
  is missing to the one that exists, so the code compiles on both.
- The raw DMA result struct layout differs by chip: ESP32/S2 use `TYPE1`,
  everything else (including the C3) uses `TYPE2`. `ADC_GET_CHANNEL(p)` /
  `ADC_GET_DATA(p)` macros abstract over this so the extraction code stays
  chip-agnostic.

### ISRs

- **`on_conv_done()`** (`IRAM_ATTR`, so it runs even with flash cache
  disabled) fires when a DMA conversion frame completes. It does the minimum
  possible work: a FreeRTOS task notification (`vTaskNotifyGiveFromISR`) to
  wake the drain task, requesting a context switch on ISR exit if the drain
  task is higher priority than whatever was running. Task notifications are
  the fastest FreeRTOS signaling primitive, ideal for a simple "wake me up".
- **`on_pool_ovf()`** fires when the DMA pool filled before software drained
  it (the device fell behind). It just increments `s_pool_ovf`, a `static
  volatile` counter (volatile because it's written in an ISR and read in a
  task). `adc_setup_overflow_count()` exposes it read-only to the rest of the
  firmware; it's later embedded in every frame header so the PC can detect
  device-side sample loss.

### `adc_setup_start()`

Creates the continuous-ADC handle, configures the sampling pattern (channel/
attenuation/bit-width) and digital config (rate, format, callbacks), starts
hardware sampling, and creates the calibration handle — all in one call, in
that order, each step checked with `ESP_RETURN_ON_ERROR`. Internally the
calibration handle comes from `adc_cali_start()` (static/private to this
file), which prefers curve-fitting calibration and falls back to
line-fitting on chips that don't support it, reading each chip's factory
eFuse calibration data rather than assuming a naive `raw * Vref / 4095`
linear formula (the real transfer function is measurably non-linear at
`ADC_ATTEN_DB_12`).

### `adc_setup_extract_mv()`

Parses one raw DMA read buffer (as filled by `adc_continuous_read()`) into
calibrated millivolt readings:

1. Walk the buffer in `SOC_ADC_DIGI_RESULT_BYTES`-sized entries.
2. Keep only entries whose channel matches `ADC_CHANNEL_SEL` (defensive —
   only one channel is configured, but this guards against stray entries).
3. Mask to the low 12 bits (`& 0x0FFF`) to get the raw ADC code.
4. Convert raw code → millivolts via `adc_cali_raw_to_voltage()`. This can
   only fail on an invalid raw code, which can't happen here (already
   masked to 12 bits); on any unexpected error it falls back to a
   straight-line approximation (`raw * 3100 / 4095`) rather than silently
   sending stale data.

Returns the count of samples written (bounded by `out_cap`), which callers
use as the definitive "how many results were actually valid this pass".

---

## 4. `mains_filter.h` — ADC millivolts → mains millivolts

The ADC does not see the mains directly: it sees the output of a sensing
front-end (step-down transformer → 122k/22nF RC filter → 3.3k/10µF/2.2k bias
network centered at ~half the 3.3 V rail, with ±0.7V-past-rail clamp diodes
for overvoltage protection). At 50 Hz this network is, to good
approximation, a fixed linear gain plus a fixed DC bias, so the
instantaneous mains voltage can be recovered on-device from a single
(noise-filtered) ADC reading:

```
v_mains(t) = (v_adc_filtered(t) - dc_estimate(t)) * MAINS_INV_GAIN
```

Full transfer-function derivation (transformer ratio, RC/bias-network gain,
theoretical DC bias point) is in the header's top comment. The short version:
combined gain `Vadc_ac / Vmains_ac ≈ 0.0036`, so at 240 Vrms max mains input
this reproduces a target output range of roughly ±339400 mV.

### Gain calibration (`MAINS_CAL_FACTOR`)

The theoretical gain is only as good as nominal component values (a few
percent of error from resistor/cap tolerance is normal). `MAINS_CAL_FACTOR`
is an empirically-derived correction multiplied into the theoretical inverse
gain (`MAINS_INV_GAIN`). See `docs/calibration_history.md` for the full
trial-by-trial derivation of the current value, and the methodology used to
avoid chasing measurement noise (average several raw trials rather than
compounding the latest ratio).

### Adaptive DC-bias tracking (`mains_filter_t::dc_estimate`)

Rather than trust a fixed theoretical DC bias (wrong by however much the
2.2k/2.2k/3.3k divider resistors deviate from nominal), the filter tracks
the ADC node's actual DC operating point live with an exponential moving
average — a 1-pole low-pass with cutoff far below 50 Hz (`MAINS_DC_TRACK_ALPHA`,
time constant `MAINS_DC_TRACK_TAU_S = 1 s` → cutoff ~0.16 Hz, ~300x below
mains frequency) — and subtracts *that* before applying the gain. Since the
50 Hz AC signal averages to ~0 over full mains cycles, this converges on the
true DC bias regardless of whether mains is present, absent, or its
amplitude changes, self-calibrating away resistor tolerance, ADC offset
error, and slow thermal drift with no hard-coded assumption about the
divider's exact midpoint.

### Anti-noise smoothing (`mains_filter_t::lpf_estimate`)

Why DC-bias tracking alone can't zero out a floating/disconnected input:
`MAINS_INV_GAIN` is ~280x (the front-end attenuates a 240 Vrms mains swing
down to only ~2.4 Vpp at the ADC). That means every single ADC code step (1
LSB ≈ 0.76 mV) is amplified to ~210 mV in the reported mains value — a
routine ADC code jitter of only 6–7 LSBs rms (quantization/thermal noise,
USB/CPU switching noise, or ambient 50 Hz EMI picked up by the transformer
secondary once it's floating) fully explains a multi-volt residual with no
mains connected. DC-bias tracking only removes the *average* component; it
can't remove this AC-ish noise, and ambient EMI pickup happens to land right
at 50 Hz — the same frequency as the real signal — so it isn't separable
from a true reading by frequency-selective filtering either.

A 1-pole low-pass (`MAINS_LPF_CUTOFF_HZ = 1000 Hz`, well above the 50 Hz
fundamental to preserve waveform harmonics, well below the ADC's ~5 kHz
Nyquist) ahead of the DC-bias tracker reduces the broadband noise
contribution. It will **not** fully zero an open/floating input — that's a
real hardware/physics limit of this high-gain sensing front end, not a
firmware bug. Changing this cutoff also changes how much of the *real*
waveform's harmonics get attenuated, which is why `docs/calibration_history.md`
tracks calibration in separate "epochs" whenever it changes.

### `mains_filter_t` / `mains_filter_init()` / `mains_filter_step()`

A tiny struct holding the two running estimates (`lpf_estimate`,
`dc_estimate`), both seeded at the theoretical DC bias so early samples
aren't wildly off before they converge. `mains_filter_step()` runs one ADC
millivolt reading through: low-pass first (noise reduction), then DC-bias
tracking on the smoothed value, then gain to recover instantaneous mains
millivolts, rounded to the nearest integer.

---

## 5. `wire_protocol.h` — USB frame format

Defines the on-the-wire frame layout and `wire_pack_header()`, which writes
the 10-byte little-endian header (magic + seq + ovf + count) byte-by-byte
(rather than casting a packed struct) — this keeps the wire format explicit
and immune to struct-padding/endianness surprises regardless of what
compiler/platform reads it on the PC side.

| Bytes | Field | Type | Meaning |
|---|---|---|---|
| 0–1 | magic | `0xA5 0x5A` | Sync word; the PC scans for this to (re-)align to frame boundaries. |
| 2–5 | seq | `uint32` | Frame counter; a **gap** ⇒ a frame lost **in transit** (USB/host side). |
| 6–7 | ovf | `uint16` | Cumulative DMA pool-overflow count; if it **increments**, the **device** dropped samples. |
| 8–9 | count | `uint16` | Number of samples in this frame. |
| 10… | payload | `count × int32` | Reconstructed instantaneous **mains millivolts** (not raw ADC volts) — see `mains_filter.h`. Expected range ≈ -339400..+339400 mV at 240 Vrms max mains input. |

`seq` gaps and `ovf` increments are independent loss signals — transport vs.
device-side — so together they let the PC prove exactly where any missing
data went.

---

## 6. `main.c` — orchestration

### `stream_task()`

Runs as a FreeRTOS task; drains the DMA pool and emits framed sample blocks
over USB.

- **Buffers** (`raw`, `adc_mv`, `samples`, `frame`) are all `static` so they
  live in `.bss`, not on the task's stack — important for these
  kilobyte-sized buffers on a small stack. `raw` is explicitly 4-byte
  aligned because it's read back as 32-bit `adc_digi_output_data_t` entries,
  and the RV32 C3 faults (`LoadStoreAlignment`) on an unaligned 32-bit load;
  a plain `uint8_t[]` is only guaranteed 1-byte alignment by the C standard.
- **Startup:** `adc_setup_start()` (passing this task's own handle so the ISR
  can notify it) and `mains_filter_init()`. `ESP_ERROR_CHECK` aborts if ADC
  startup fails — appropriate here since there's no meaningful recovery.
- **Main loop:**
  1. `ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000))` blocks until the ISR
     notifies (or a 1 s timeout, which keeps the loop alive and overflow
     counting even if the host stalls indefinitely).
  2. `while (adc_continuous_read(..., /*timeout=*/0) == ESP_OK)` drains
     *everything* currently buffered, not just one frame — this is what
     keeps the device ahead of overflow.
  3. `adc_setup_extract_mv()` turns the raw DMA bytes into calibrated ADC
     millivolts; if none were valid this pass, skip straight to the next
     read.
  4. Each ADC millivolt reading is run through `mains_filter_step()` to get
     the reconstructed instantaneous mains millivolt value.
  5. `wire_pack_header()` + `memcpy()` build the outgoing frame; `seq`
     advances every iteration so the PC can detect transport-side gaps.
  6. `usb_serial_jtag_write_bytes()` sends it with a 20 ms timeout. At
     ~20 kB/s over a 12 Mbps link the host easily keeps up, so this usually
     returns instantly; on a hard host stall the write may be short
     (truncated), which is fine because the PC re-syncs on the magic word.

### `app_main()`

Installs the native USB CDC driver (4096-byte TX buffer so a whole frame
fits without a partial write in the common case; a small 256-byte RX buffer
since the device mostly only transmits), then creates `stream_task` with a
4096-byte stack at priority 10 (moderately high, so it drains promptly).
`app_main` then returns; the task runs the stream forever.

### Console output vs. USB CDC

In `idf.py menuconfig`, **Component config → ESP System Settings → Channel
for console output** must be set to **None** or **UART0** — not **USB
Serial/JTAG**, or ESP-IDF log text would be injected into the binary data
stream. The PC reader re-syncs on the magic word so it's tolerable even if
left on the default, but None/UART0 is the clean choice. See `README.md` for
the full build/flash instructions.

---

## 7. Summary of the "never miss a sample" guarantees

1. **Hardware-timed sampling** — DMA samples at a fixed rate; CPU jitter
   can't affect sample spacing.
2. **Deep DMA pool (8 frames)** — absorbs brief USB/host stalls without loss.
3. **Aggressive draining** — the inner `while` empties the whole pool every
   wake, staying ahead of overflow.
4. **Overflow counting** — any device-side loss is counted in the ISR and
   reported in every frame (`ovf`).
5. **Sequence numbering** — any transport-side loss is detectable via `seq`
   gaps.
6. **Magic-word framing** — the PC can always re-align after a short write or
   injected log text.

The result: the device streams continuously and, critically, gives the PC
the metadata to **prove** whether the stream is complete — on top of which
it also reconstructs and reports the actual instantaneous mains voltage, not
just a raw ADC pin reading.
