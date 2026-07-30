#!/usr/bin/env python3
"""
PC-side reader/verifier for the ESP32-C3 ADC stream.

Usage:
    pip install pyserial
    python pc_reader.py <PORT> [out.csv] [--wav out.wav]

    # examples
    python pc_reader.py /dev/ttyACM0
    python pc_reader.py COM7 capture.csv
    python pc_reader.py /dev/ttyACM0 --wav capture.wav
    python pc_reader.py /dev/ttyACM0 capture.csv --wav capture.wav

It re-syncs on the magic word, parses each frame, and continuously reports:
  - effective sample rate (should read ~44100 Hz)
  - frame-sequence gaps  (frame lost in transit -> almost never on USB CDC)
  - device pool overflows DURING CAPTURE (device dropped samples because the
    host fell behind AFTER warmup). Overflows that happen during boot / USB
    enumeration, before the host attaches, are flagged separately as
    "pre-capture buffer overflow" -- those frames are inside the warmup
    window and get discarded anyway, so they are NOT counted as losses.
  - short frames         (device emitted a frame with fewer samples than expected)
  - disconnects          (USB cable pulled or device rebooted)

If the USB link disappears mid-stream, the reader closes the port, waits for
the device node to reappear, and resumes -- for up to RECONNECT_TIMEOUT_S
seconds. Cumulative counters (samples, losses, connected time) survive across
reconnects; the per-connect warmup is repeated each time so the reported rate
is not polluted by post-reconnect buffer backlogs.

Optionally writes every sample to a CSV (one millivolt value per line) and/or
a 16-bit PCM mono WAV file. The WAV sample rate is not hard-coded: it is
*learned* from the mean of the per-second effective-rate reports emitted
during the run (so a device actually running at, say, 43987 Hz will produce
a WAV that plays back at real-time speed).

Samples arrive already converted to millivolts of the *instantaneous mains
voltage* (not the raw ADC pin voltage). The device inverts its sensing
front-end's fixed gain and DC bias (transformer + RC filter + bias network)
to recover the mains waveform, streaming signed 32-bit millivolt values with
an expected range of roughly -339400..+339400 mV at a 240 Vrms max mains
input.

The once-per-second status line includes the RMS mains voltage (Vrms),
computed on the host from the actual sample values received since the
previous report (sqrt(mean(sample^2))). This is a live sanity check against
a trusted multimeter reading of the same outlet.
"""

import argparse
import math
import struct
import sys
import time
import wave

try:
    import serial  # pyserial
except ImportError:
    sys.exit("Install pyserial:  pip install pyserial")

MAGIC = b"\xA5\x5A"
HDR = struct.Struct("<IHH")      # seq(uint32), ovf(uint16), count(uint16)  -- after the 2 magic bytes
MAX_COUNT = 4096                 # sanity ceiling for a frame's sample count
EXPECTED_COUNT = 256             # firmware SAMPLES_PER_FRAME -- any deviation is a red flag

# Samples arrive as reconstructed instantaneous mains voltage, in millivolts
# (signed int32), not raw ADC codes and not raw ADC-pin millivolts.
MAINS_MV_PEAK = 339400           # expected peak magnitude at a 240 Vrms max mains input

# ---- reconnect / stall behaviour -------------------------------------------
RECONNECT_TIMEOUT_S = 60.0       # give up after this long trying to reopen the port
RECONNECT_INTERVAL_S = 0.5       # poll interval while waiting for device to come back
STALL_TIMEOUT_S = 5.0            # no bytes for this long => assume the link is dead
WARMUP_FRAMES = 40               # drop this many frames after each (re)connect


class Disconnected(Exception):
    """Raised when the USB CDC link errors out or stops delivering bytes."""


_PCM_SCALE = 32767.0 / MAINS_MV_PEAK


def mv_to_pcm16(s):
    """Convert a signed mains-millivolt sample (~-339400..+339400) to signed
    16-bit PCM.

    Already centered at 0 (it's an AC waveform), just scaled to fill
    [-32768..32767]. Clamped because transient spikes/noise can occasionally
    exceed the nominal peak.
    """
    v = int(round(s * _PCM_SCALE))
    return max(-32768, min(32767, v))


# ---- low-level I/O ---------------------------------------------------------

def read_exact(ser, n, stall_timeout=STALL_TIMEOUT_S):
    """Read exactly n bytes. Raise Disconnected if the link stalls or errors."""
    buf = bytearray()
    deadline = time.time() + stall_timeout
    while len(buf) < n:
        try:
            chunk = ser.read(n - len(buf))
        except (serial.SerialException, OSError) as e:
            raise Disconnected(f"read failed: {e}")
        if chunk:
            buf += chunk
            deadline = time.time() + stall_timeout
        elif time.time() > deadline:
            raise Disconnected(f"no data for {stall_timeout:.1f}s")
    return bytes(buf)


def resync(ser, stall_timeout=STALL_TIMEOUT_S):
    """Advance the stream until we've just consumed a magic word.

    Raises Disconnected on error / prolonged silence so the caller can
    trigger reconnect logic instead of hanging forever.
    """
    prev = b""
    deadline = time.time() + stall_timeout
    while True:
        try:
            b = ser.read(1)
        except (serial.SerialException, OSError) as e:
            raise Disconnected(f"read failed: {e}")
        if not b:
            if time.time() > deadline:
                raise Disconnected(f"no data for {stall_timeout:.1f}s")
            continue
        deadline = time.time() + stall_timeout
        if prev == MAGIC[0:1] and b == MAGIC[1:2]:
            return
        prev = b


def open_serial(port, retry_seconds=RECONNECT_TIMEOUT_S):
    """Open the serial port, retrying while the device node is missing or busy.

    Raises TimeoutError if the port does not become available within
    retry_seconds.
    """
    deadline = time.time() + retry_seconds
    last_err = None
    announced = False
    while True:
        try:
            return serial.Serial(port, baudrate=115200, timeout=0.1)
        except (serial.SerialException, OSError) as e:
            last_err = e
            if not announced:
                print(f"  ... waiting up to {retry_seconds:.0f}s for {port} to (re)appear ...")
                announced = True
            if time.time() >= deadline:
                raise TimeoutError(
                    f"could not open {port} within {retry_seconds:.0f}s: {last_err}")
            time.sleep(RECONNECT_INTERVAL_S)


# ---- main ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Read and verify the ESP32-C3 ADC stream over USB CDC.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("port", help="Serial port (e.g. /dev/ttyACM0 or COM7)")
    parser.add_argument(
        "csv",
        nargs="?",
        default=None,
        help="Optional CSV output path (one millivolt sample per line).",
    )
    parser.add_argument(
        "--wav",
        dest="wav",
        default=None,
        help="Optional 16-bit PCM mono WAV output path. The sample rate is "
             "learned from the mean of per-second effective-rate reports.",
    )
    args = parser.parse_args()

    port = args.port
    csv_path = args.csv
    wav_path = args.wav
    csv = open(csv_path, "w") if csv_path else None

    # For the WAV file we buffer the post-warmup samples in memory as raw
    # little-endian signed 16-bit PCM. Only after capture ends do we know the
    # actual (measured) sample rate to stamp into the WAV header.
    wav_pcm = bytearray() if wav_path else None
    reported_rates = []  # every value printed as "~<rate> Hz" during the run

    print(f"Reading {port} ... Ctrl-C to stop")
    try:
        ser = open_serial(port)
    except TimeoutError as e:
        sys.exit(str(e))

    # ---- cumulative counters (survive reconnects) ----
    frame_gaps = 0                 # in-transit losses AFTER warmup, whole run
    last_ovf = 0                   # highest device-side pool-overflow count seen (raw)
    capture_ovf = 0                # ovf events that happened AFTER warmup (real drops
                                   # during measurement, not the pre-connect boot burst)
    short_frames = 0
    disconnects = 0
    cum_samples = 0                # samples counted across all connected segments
    cum_time = 0.0                 # seconds we've spent actually connected & post-warmup
    cum_sum_sq = 0.0                # sum of sample^2 across the whole run (for overall Vrms)

    # ---- per-segment state (reset on each connect) ----
    segment_t0 = None
    segment_samples = 0

    def roll_segment_into_cum():
        """Fold the live segment (if any) into cum_* and clear it."""
        nonlocal cum_time, cum_samples, segment_t0, segment_samples
        if segment_t0 is not None:
            cum_time += time.time() - segment_t0
            cum_samples += segment_samples
        segment_t0 = None
        segment_samples = 0

    try:
        while True:  # outer loop: one iteration per (re)connection
            # per-session state (reset every connect)
            expected_seq = None
            warmup_left = WARMUP_FRAMES
            first_seq = None
            last_seq = None
            frames_counted = 0
            segment_samples = 0
            segment_t0 = None
            t_report = None
            rms_sum_sq = 0.0     # sum of sample^2 since the last per-second report
            rms_count = 0        # sample count since the last per-second report

            try:
                resync(ser)

                while True:
                    header = read_exact(ser, HDR.size)
                    seq, ovf, count = HDR.unpack(header)

                    if count == 0 or count > MAX_COUNT:
                        # Garbled framing (e.g. a torn frame during a stall). Re-sync.
                        resync(ser)
                        expected_seq = None
                        continue

                    payload = read_exact(ser, count * 4)
                    samples = struct.unpack(f"<{count}i", payload)

                    # The next 2 bytes must be the next magic word; if not, re-sync.
                    nxt = read_exact(ser, 2)
                    need_resync = (nxt != MAGIC)

                    # ---- integrity checks ----
                    if expected_seq is not None and seq != expected_seq:
                        # During warmup this is expected (see WARMUP_FRAMES comment);
                        # only count/print it once we're past warmup.
                        if warmup_left == 0:
                            frame_gaps += 1
                            print(f"  ! frame gap: expected seq {expected_seq}, got {seq} "
                                  f"(a frame was lost in transit)")
                    expected_seq = (seq + 1) & 0xFFFFFFFF

                    if ovf > last_ovf:
                        delta = ovf - last_ovf
                        if warmup_left > 0:
                            # DMA pool filled while USB host wasn't draining yet
                            # (typical during boot / USB enumeration). These frames
                            # are inside the warmup window and get discarded anyway.
                            print(f"  .. pre-capture buffer overflow: {last_ovf} -> {ovf} "
                                  f"(+{delta} events, host not yet draining; discarded as warmup)")
                        else:
                            capture_ovf += delta
                            print(f"  !! DEVICE DROPPED SAMPLES: pool overflow {last_ovf} -> {ovf} "
                                  f"(+{delta} events, {capture_ovf} total during capture)")
                        last_ovf = ovf

                    if count != EXPECTED_COUNT and warmup_left == 0:
                        # Firmware always emits SAMPLES_PER_FRAME=256. If we ever see
                        # a short frame here, the DMA drain path is behaving oddly.
                        short_frames += 1
                        print(f"  ! short frame: seq {seq} carried {count} samples "
                              f"(expected {EXPECTED_COUNT})")

                    # ---- warmup handling ----
                    if warmup_left > 0:
                        warmup_left -= 1
                        if warmup_left == 0:
                            first_seq = (seq + 1) & 0xFFFFFFFF   # start with NEXT frame
                            last_seq = seq
                            frames_counted = 0
                            segment_samples = 0
                            segment_t0 = time.time()
                            t_report = segment_t0
                            print(f"  -- warmup done, measurement starts at seq {first_seq} --")
                        if csv:
                            csv.write("\n".join(str(s) for s in samples))
                            csv.write("\n")
                        if need_resync:
                            resync(ser)
                            expected_seq = None
                        continue

                    segment_samples += count
                    frames_counted += 1
                    last_seq = seq
                    for s in samples:
                        sq = float(s) * float(s)
                        rms_sum_sq += sq
                        cum_sum_sq += sq
                    rms_count += count
                    if csv:
                        csv.write("\n".join(str(s) for s in samples))
                        csv.write("\n")
                    if wav_pcm is not None:
                        # Samples are mains millivolts (signed, ~-339400..+339400);
                        # convert to centered, scaled 16-bit signed PCM.
                        wav_pcm.extend(
                            struct.pack(
                                f"<{count}h",
                                *(mv_to_pcm16(s) for s in samples),
                            )
                        )

                    # ---- once-per-second status ----
                    now = time.time()
                    if now - t_report >= 1.0:
                        seg_dt = now - segment_t0
                        cum_dt = cum_time + seg_dt
                        cum_ns = cum_samples + segment_samples
                        rate = (cum_ns / cum_dt) if cum_dt > 0 else 0.0
                        if rate > 0:
                            reported_rates.append(rate)
                        expected_frames = ((last_seq - first_seq) & 0xFFFFFFFF) + 1
                        lost = expected_frames - frames_counted
                        rms_mv = math.sqrt(rms_sum_sq / rms_count) if rms_count > 0 else 0.0
                        rms_v = rms_mv / 1000.0
                        print(f"  {cum_ns} samples, ~{rate:8.1f} Hz, Vrms={rms_v:7.2f} V, "
                              f"frame_gaps={frame_gaps}, device_overflows={capture_ovf}, "
                              f"short_frames={short_frames}, unaccounted={lost}, "
                              f"disconnects={disconnects}")
                        t_report = now
                        rms_sum_sq = 0.0
                        rms_count = 0

                    if need_resync:
                        resync(ser)
                        expected_seq = None

            except Disconnected as e:
                roll_segment_into_cum()
                try:
                    ser.close()
                except Exception:
                    pass
                disconnects += 1
                print(f"  !! DISCONNECTED (connected time so far: {cum_time:.1f}s): {e}")
                try:
                    ser = open_serial(port)
                except TimeoutError as e2:
                    print(f"  !! could not reconnect: {e2}")
                    break
                print(f"  == RECONNECTED to {port}; restarting warmup ==")
                # fall through -> outer while restarts a fresh session

    except KeyboardInterrupt:
        pass
    finally:
        roll_segment_into_cum()
        try:
            ser.close()
        except Exception:
            pass

        rate = (cum_samples / cum_time) if cum_time > 0 else 0.0
        overall_rms_v = (math.sqrt(cum_sum_sq / cum_samples) / 1000.0) if cum_samples > 0 else 0.0
        print(f"\nStopped. {cum_samples} samples in {cum_time:.1f}s connected "
              f"(~{rate:.1f} Hz), overall Vrms={overall_rms_v:.2f} V")
        print(f"  disconnects={disconnects}, frame_gaps={frame_gaps}, "
              f"device_overflows={capture_ovf}, short_frames={short_frames}")
        # "no samples lost" only makes sense per-connected-window; a disconnect
        # is by definition data lost, so we call that out separately.
        clean_stream = (frame_gaps == 0 and capture_ovf == 0 and short_frames == 0)
        if clean_stream and disconnects == 0:
            print("  => NO SAMPLES LOST.")
        elif clean_stream:
            print(f"  => No in-stream losses, but {disconnects} disconnect(s) "
                  f"interrupted the capture.")
        else:
            print("  => LOSSES DETECTED (see counters above).")

        if csv:
            csv.close()
            print(f"Wrote {csv_path}")

        if wav_pcm is not None:
            # Learn the sample rate from the mean of what we *actually*
            # measured during the run. Fall back to cum_samples/cum_time if
            # no per-second report was ever emitted (very short capture),
            # and finally to 44100 Hz if we captured nothing at all.
            if reported_rates:
                learned_rate = sum(reported_rates) / len(reported_rates)
                rate_src = (f"mean of {len(reported_rates)} per-second "
                            f"rate report(s)")
            elif cum_time > 0 and cum_samples > 0:
                learned_rate = cum_samples / cum_time
                rate_src = "cum_samples / cum_time (no per-second reports)"
            else:
                learned_rate = 44100.0
                rate_src = "fallback default (no measurements available)"

            wav_rate = int(round(learned_rate))
            n_frames = len(wav_pcm) // 2
            if n_frames == 0:
                print(f"  !! no post-warmup samples captured; skipping {wav_path}")
            else:
                try:
                    with wave.open(wav_path, "wb") as wf:
                        wf.setnchannels(1)
                        wf.setsampwidth(2)         # 16-bit PCM
                        wf.setframerate(wav_rate)
                        wf.writeframes(bytes(wav_pcm))
                    duration = n_frames / wav_rate if wav_rate > 0 else 0.0
                    print(f"Wrote {wav_path}: {n_frames} samples, "
                          f"{wav_rate} Hz (learned from {rate_src}), "
                          f"~{duration:.2f}s of audio")
                except OSError as e:
                    print(f"  !! failed to write {wav_path}: {e}")


if __name__ == "__main__":
    main()

