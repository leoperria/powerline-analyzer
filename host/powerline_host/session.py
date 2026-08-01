"""The capture session: reads frames, verifies stream integrity, reports.

Once per second it prints effective sample rate, RMS mains voltage, mains
frequency, and the integrity counters:

  - frame-sequence gaps  (frame lost in transit -> almost never on USB CDC)
  - device pool overflows DURING CAPTURE (device dropped samples because the
    host fell behind AFTER warmup). Overflows that happen during boot / USB
    enumeration, before the host attaches, are flagged separately as
    "pre-capture buffer overflow" -- those frames are inside the warmup
    window and get discarded anyway, so they are NOT counted as losses.
  - short frames         (device emitted a frame with fewer samples than expected)
  - disconnects          (USB cable pulled or device rebooted)

Cumulative counters survive reconnects; the warmup is repeated on each
connect so the reported rate is not polluted by post-reconnect backlogs.
"""

import time

from .link import Disconnected, SerialLink
from .meters import FrequencyMeter, RmsMeter, SampleClock
from .protocol import EXPECTED_COUNT, GarbledFrame, read_frame

WARMUP_FRAMES = 40           # drop this many frames after each (re)connect
REPORT_INTERVAL_S = 1.0
SEQ_MASK = 0xFFFFFFFF


class CaptureSession:
    def __init__(self, port, csv=None, wav=None):
        self.link = SerialLink(port)
        self.csv = csv
        self.wav = wav

        # ---- whole-run state (survives reconnects) ----
        self.clock = SampleClock()
        self.total_rms = RmsMeter()
        self.frame_gaps = 0        # in-transit losses AFTER warmup
        self.device_overflows = 0  # overflow events AFTER warmup (real drops during
                                   # measurement, not the pre-connect boot burst)
        self.last_overflow = 0     # highest device-side overflow count seen (raw)
        self.short_frames = 0
        self.disconnects = 0
        self.reported_rates = []   # every value printed as "~<rate> Hz"
        self.reported_hz = []      # every mains frequency printed

        # ---- per-connection state (see _reset_connection_state) ----
        self.warmup_left = 0
        self.expected_seq = None
        self.first_seq = None
        self.last_seq = None
        self.frames_counted = 0
        self.t_report = None
        self.window_rms = RmsMeter()
        self.frequency = FrequencyMeter()

    # ---- top level ----

    def run(self):
        """Capture until Ctrl-C, a fatal reconnect failure, or EOF."""
        print(f"Reading {self.link.port} ... Ctrl-C to stop")
        self.link.open()
        try:
            while True:
                self._reset_connection_state()
                try:
                    self._stream_until_disconnect()
                except Disconnected as e:
                    if not self._reconnect(e):
                        break
        except KeyboardInterrupt:
            pass
        finally:
            self.clock.end_segment()
            self.link.close()
            self._print_summary()
            self._close_outputs()

    def _reset_connection_state(self):
        self.warmup_left = WARMUP_FRAMES
        self.expected_seq = None
        self.first_seq = None
        self.last_seq = None
        self.frames_counted = 0
        self.t_report = None
        self.window_rms = RmsMeter()
        self.frequency = FrequencyMeter()

    def _stream_until_disconnect(self):
        self.link.resync()
        while True:
            try:
                frame = read_frame(self.link)
            except GarbledFrame:
                # Torn frame (e.g. during a stall) -- find the next magic word.
                self._resync()
                continue

            self._handle_frame(frame)

            if not frame.trailing_magic:
                self._resync()

    def _resync(self):
        self.link.resync()
        self.expected_seq = None

    def _reconnect(self, reason):
        """Returns True if the link came back, False if we should give up."""
        self.clock.end_segment()
        self.link.close()
        self.disconnects += 1
        print(f"  !! DISCONNECTED (connected time so far: "
              f"{self.clock.total_time:.1f}s): {reason}")
        try:
            self.link.open()
        except TimeoutError as e:
            print(f"  !! could not reconnect: {e}")
            return False
        print(f"  == RECONNECTED to {self.link.port}; restarting warmup ==")
        return True

    # ---- per-frame handling ----

    def _handle_frame(self, frame):
        self._check_sequence(frame)
        self._check_overflows(frame)
        self._check_short_frame(frame)

        if self.warmup_left > 0:
            self._consume_warmup_frame(frame)
            return

        self._accumulate(frame)
        self._maybe_report()

    def _check_sequence(self, frame):
        if self.expected_seq is not None and frame.seq != self.expected_seq:
            # During warmup a gap is expected (the device has been streaming
            # into the void); only count it once we're measuring.
            if self.warmup_left == 0:
                self.frame_gaps += 1
                print(f"  ! frame gap: expected seq {self.expected_seq}, got "
                      f"{frame.seq} (a frame was lost in transit)")
        self.expected_seq = (frame.seq + 1) & SEQ_MASK

    def _check_overflows(self, frame):
        if frame.overflows <= self.last_overflow:
            return
        delta = frame.overflows - self.last_overflow
        if self.warmup_left > 0:
            # DMA pool filled while the USB host wasn't draining yet (typical
            # during boot / enumeration). These frames are inside the warmup
            # window and get discarded anyway.
            print(f"  .. pre-capture buffer overflow: {self.last_overflow} -> "
                  f"{frame.overflows} (+{delta} events, host not yet draining; "
                  f"discarded as warmup)")
        else:
            self.device_overflows += delta
            print(f"  !! DEVICE DROPPED SAMPLES: pool overflow "
                  f"{self.last_overflow} -> {frame.overflows} (+{delta} events, "
                  f"{self.device_overflows} total during capture)")
        self.last_overflow = frame.overflows

    def _check_short_frame(self, frame):
        # The firmware always emits SAMPLES_PER_FRAME. A short frame here means
        # the DMA drain path is behaving oddly.
        if len(frame.samples) != EXPECTED_COUNT and self.warmup_left == 0:
            self.short_frames += 1
            print(f"  ! short frame: seq {frame.seq} carried "
                  f"{len(frame.samples)} samples (expected {EXPECTED_COUNT})")

    def _consume_warmup_frame(self, frame):
        self.warmup_left -= 1
        if self.warmup_left == 0:
            self._begin_measurement(frame)
        if self.csv:  # the CSV gets every sample, warmup included
            self.csv.write(frame.samples)

    def _begin_measurement(self, frame):
        self.first_seq = (frame.seq + 1) & SEQ_MASK   # start with the NEXT frame
        self.last_seq = frame.seq
        self.frames_counted = 0
        self.clock.start_segment()
        self.t_report = time.time()
        print(f"  -- warmup done, measurement starts at seq {self.first_seq} --")

    def _accumulate(self, frame):
        self.clock.add(len(frame.samples))
        self.frames_counted += 1
        self.last_seq = frame.seq
        self.window_rms.add(frame.samples)
        self.total_rms.add(frame.samples)
        self.frequency.add(frame.samples)
        if self.csv:
            self.csv.write(frame.samples)
        if self.wav:
            self.wav.write(frame.samples)

    # ---- reporting ----

    def _maybe_report(self):
        now = time.time()
        if now - self.t_report < REPORT_INTERVAL_S:
            return

        rate = self.clock.rate
        if rate > 0:
            self.reported_rates.append(rate)
        mains_hz = self.frequency.hertz(rate)
        if mains_hz > 0:
            self.reported_hz.append(mains_hz)

        expected_frames = ((self.last_seq - self.first_seq) & SEQ_MASK) + 1
        unaccounted = expected_frames - self.frames_counted

        print(f"  {self.clock.samples} samples, ~{rate:8.1f} Hz, "
              f"Vrms={self.window_rms.volts:7.2f} V, mains={mains_hz:5.2f} Hz, "
              f"frame_gaps={self.frame_gaps}, "
              f"device_overflows={self.device_overflows}, "
              f"short_frames={self.short_frames}, unaccounted={unaccounted}, "
              f"disconnects={self.disconnects}")

        self.t_report = now
        self.window_rms.reset()
        self.frequency.reset_window()

    def _print_summary(self):
        mean_hz = (sum(self.reported_hz) / len(self.reported_hz)) if self.reported_hz else 0.0
        print(f"\nStopped. {self.clock.samples} samples in "
              f"{self.clock.seconds:.1f}s connected (~{self.clock.rate:.1f} Hz), "
              f"overall Vrms={self.total_rms.volts:.2f} V, mains={mean_hz:.2f} Hz")
        print(f"  disconnects={self.disconnects}, frame_gaps={self.frame_gaps}, "
              f"device_overflows={self.device_overflows}, "
              f"short_frames={self.short_frames}")

        # "no samples lost" only makes sense per-connected-window; a disconnect
        # is by definition data lost, so we call that out separately.
        clean = (self.frame_gaps == 0 and self.device_overflows == 0
                 and self.short_frames == 0)
        if clean and self.disconnects == 0:
            print("  => NO SAMPLES LOST.")
        elif clean:
            print(f"  => No in-stream losses, but {self.disconnects} disconnect(s) "
                  f"interrupted the capture.")
        else:
            print("  => LOSSES DETECTED (see counters above).")

    def _close_outputs(self):
        if self.csv:
            self.csv.close()
        if self.wav:
            self.wav.save(self.reported_rates, self.clock.rate)
