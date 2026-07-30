# powerline-analyzer

Lossless mains-voltage waveform acquisition on a **Seeed XIAO ESP32-C3**,
streamed to a PC over native USB with a framed protocol that lets the host
*prove* no sample was ever dropped.

An analog sensing front-end (step-down transformer + RC filter + bias
network) scales the mains outlet down into the ADC's safe input range. The
device samples it with the SAR ADC in continuous DMA mode at ~10 kHz
(12-bit), reconstructs the instantaneous mains voltage on-device (inverting
the front-end's gain/bias, with adaptive calibration — see
`docs/main_c_commentary.md` and `docs/calibration_history.md`), and pushes
signed millivolt samples over USB CDC. A Python reader on the PC parses,
verifies, reports live RMS voltage, and records the stream to CSV and/or WAV.

## How it works

```
  mains outlet ──▶ sensing front-end ──▶ ESP32-C3 SAR ADC (DMA, ~10 kHz, 12-bit)
                                              │  framed protocol (magic + seq + ovf + count + payload)
                                              ▼
                                        native USB CDC ──▶ pc_reader.py ──▶ CSV / WAV + Vrms + integrity report
```

### Firmware (`src/`, ESP-IDF v5.x)

- ADC runs in **continuous (DMA) mode**, so sampling is hardware-driven and CPU
  or USB jitter cannot skew or drop samples as long as the DMA pool is drained
  in time.
- A DMA conversion-frame completion fires an ISR that wakes a drain task. The
  task copies samples out, wraps them in a small framed protocol, and writes
  them to the native USB CDC (USB Serial/JTAG). Acquisition and transmission are
  decoupled, giving headroom against brief USB stalls.
- A hardware DMA pool-overflow event (host too slow) is counted in the ISR and
  reported in every frame header, so the PC can prove whether any sample was
  lost on the device side.
- Each raw 12-bit ADC code is converted to millivolts via ESP-IDF's ADC
  calibration API (curve fitting, with a line-fitting fallback), then converted
  again from ADC millivolts to **reconstructed instantaneous mains millivolts**
  by inverting the sensing front-end's gain and an adaptively-tracked DC bias,
  with an anti-noise low-pass and an empirically-tuned gain correction.
- Split into focused modules — see `docs/main_c_commentary.md` for a full
  walkthrough: `board_config.h` (knobs), `adc_setup.{h,c}` (ADC hardware
  layer), `mains_filter.h` (mains-voltage reconstruction math), `wire_protocol.h`
  (USB frame format), `main.c` (orchestration).

### PC reader (`pc_reader.py`, Python + pyserial)

- Re-syncs on the magic word, parses each frame, and continuously reports the
  effective sample rate, live RMS mains voltage, frame-sequence gaps, device
  pool overflows during capture, short frames, and disconnects.
- Survives USB disconnects: it reopens the port and resumes for up to
  `RECONNECT_TIMEOUT_S`, repeating a per-connect warmup so the reported rate is
  not polluted by post-reconnect buffer backlogs.
- Optionally records every sample to CSV (one millivolt value per line) and/or
  a 16-bit PCM mono WAV (centered at 0, scaled by the expected ±339400 mV peak
  range). The WAV sample rate is *learned* from the measured effective rate, so
  playback runs at real time even if the true rate differs from the target.

## Wire protocol (little-endian, one frame per DMA conversion block)

| Bytes  | Field   | Meaning                                                        |
| ------ | ------- | -------------------------------------------------------------- |
| 0..1   | magic   | `0xA5 0x5A`                                                    |
| 2..5   | seq     | uint32 frame counter; a gap means a frame was lost in transit  |
| 6..7   | ovf     | uint16 cumulative DMA pool-overflow count; if it ever increments, the device dropped samples |
| 8..9   | count   | uint16 number of samples in this frame                         |
| 10..   | payload | `count` × int32 reconstructed instantaneous mains voltage, in millivolts (~-339400..+339400 at 240 Vrms max mains input) |

## Hardware

- Board: **Seeed XIAO ESP32-C3** (4 MB flash).
- Analog input: **GPIO3** (ADC1 channel 3), ~0..3.1 V full scale (12 dB atten).
  Avoid strapping pins GPIO2/8/9. Do not exceed 3.3 V on the ADC pin.
- Expects a mains-sensing front-end (step-down transformer → RC filter → bias
  network → clamp diodes) ahead of the ADC pin, scaling a 240 Vrms-max mains
  input down to a safe ~0.4–2.8 V swing. See `docs/main_c_commentary.md` §4
  (`mains_filter.h`) for the transfer-function derivation and
  `docs/calibration_history.md` for how the gain-correction constant was
  derived. Adapting this to a different sensing circuit or signal just means
  re-deriving the constants in `mains_filter.h`.

> **Note on sample rate:** the C3 ADC clock is `APB(80 MHz) / integer_divider`,
> so only rates of the form `80e6/N` are achievable. The target in
> `board_config.h` (`SAMPLE_RATE_HZ`) lands on the nearest divider — e.g.
> requesting 44100 Hz gives a real rate ≈ 44642.857 Hz. Treat the rate reported
> by the PC reader as ground truth for any downstream FFT/timing.

## Build & flash (PlatformIO)

```bash
# build
pio run

# flash + open the serial monitor
pio run -t upload
pio device monitor
```

In `idf.py menuconfig` (or `sdkconfig`), set **Component config → ESP System
Settings → Channel for console output** to **None** or **UART0** — do not leave
it on **USB Serial/JTAG**, or ESP-IDF log text will be injected into the binary
data stream. (The PC reader re-syncs on the magic word, so the default is
tolerable, but None/UART0 is the clean choice.)

## Capture on the PC

```bash
pip install pyserial

# just verify the stream (prints live + overall RMS mains voltage)
python pc_reader.py /dev/ttyACM0

# record to CSV
python pc_reader.py /dev/ttyACM0 capture.csv

# record to WAV
python pc_reader.py /dev/ttyACM0 --wav capture.wav

# both
python pc_reader.py /dev/ttyACM0 capture.csv --wav capture.wav
```

On Windows the port looks like `COM7`; on Linux/macOS like `/dev/ttyACM0`.
Press Ctrl-C to stop; the reader prints a final loss summary and, for WAV output,
stamps the learned sample rate into the file header.

## Repository layout

```
src/main.c                    Orchestration: app_main() + the drain/stream loop
src/board_config.h            ADC pin/rate/buffer-size knobs
src/adc_setup.h, adc_setup.c  ADC continuous-mode setup, calibration, raw->mV parsing, ISRs
src/mains_filter.h            ADC millivolts -> reconstructed instantaneous mains millivolts
src/wire_protocol.h           USB frame format (header packing)
pc_reader.py                  PC-side reader / verifier / recorder / live Vrms
platformio.ini                PlatformIO project config (Seeed XIAO ESP32-C3, ESP-IDF)
docs/main_c_commentary.md     Full firmware walkthrough
docs/calibration_history.md   Empirical gain-calibration trial log
capture.csv                   Example capture (one millivolt sample per line)
output.wav                    Example decoded audio-rate capture
```
