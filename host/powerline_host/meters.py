"""Measurements computed on the host from the received sample stream."""

import math
import time

# Zero-crossing hysteresis for the frequency measurement, in mains millivolts.
# The signal must dip below -HYSTERESIS_MV before another rising crossing is
# accepted, so noise wobbling around 0 can't fire several crossings per cycle.
# Small next to the ~290000 mV peak, huge next to the ~1000 mV noise floor.
HYSTERESIS_MV = 10000


class RmsMeter:
    """Running sqrt(mean(sample^2)) over whatever samples it is fed."""

    def __init__(self):
        self.sum_sq = 0.0
        self.count = 0

    def add(self, samples):
        for s in samples:
            self.sum_sq += float(s) * float(s)
        self.count += len(samples)

    def reset(self):
        self.sum_sq = 0.0
        self.count = 0

    @property
    def volts(self):
        if self.count == 0:
            return 0.0
        return math.sqrt(self.sum_sq / self.count) / 1000.0


class FrequencyMeter:
    """Mains frequency from rising zero crossings.

    Each crossing's position is linearly interpolated between the two samples
    that straddle it, so resolution is not limited to one sample period. The
    frequency is then (whole cycles spanned) / (time they took).

    `prev`/`index`/`armed` run continuously; `first`/`last`/`count` describe
    only the current reporting window and are cleared by reset_window().
    """

    def __init__(self, hysteresis_mv=HYSTERESIS_MV):
        self.hysteresis_mv = hysteresis_mv
        self.prev = 0        # previous sample value
        self.index = 0       # running sample index
        self.armed = False   # True once the signal has gone below -hysteresis
        self.first = 0.0     # fractional sample index of first crossing in window
        self.last = 0.0      # ... and of the most recent one
        self.count = 0

    def add(self, samples):
        for s in samples:
            if s < -self.hysteresis_mv:
                self.armed = True
            elif self.armed and s >= 0 > self.prev:
                # Interpolate where the line from prev to s crosses zero.
                pos = self.index - 1 + (-self.prev / (s - self.prev))
                if self.count == 0:
                    self.first = pos
                self.last = pos
                self.count += 1
                self.armed = False
            self.prev = s
            self.index += 1

    def reset_window(self):
        self.count = 0

    def hertz(self, sample_rate):
        """Frequency in Hz, or 0.0 if fewer than two crossings were seen.

        `sample_rate` converts sample indices to seconds; pass the *measured*
        rate so the result self-corrects for device clock error.
        """
        span = self.last - self.first
        if self.count < 2 or span <= 0:
            return 0.0
        return (self.count - 1) * sample_rate / span


class SampleClock:
    """Tracks samples and connected time across disconnect/reconnect cycles.

    Time only accrues while a segment is open, so the gap spent waiting for
    the device to reappear never pollutes the effective sample rate.
    """

    def __init__(self):
        self.total_time = 0.0       # completed segments only
        self.total_count = 0        # ... likewise
        self.segment_t0 = None
        self.segment_count = 0

    def start_segment(self):
        self.segment_t0 = time.time()
        self.segment_count = 0

    def add(self, n):
        self.segment_count += n

    def end_segment(self):
        """Fold the live segment (if any) into the totals and clear it."""
        if self.segment_t0 is not None:
            self.total_time += time.time() - self.segment_t0
            self.total_count += self.segment_count
        self.segment_t0 = None
        self.segment_count = 0

    @property
    def samples(self):
        return self.total_count + self.segment_count

    @property
    def seconds(self):
        if self.segment_t0 is None:
            return self.total_time
        return self.total_time + (time.time() - self.segment_t0)

    @property
    def rate(self):
        seconds = self.seconds
        return (self.samples / seconds) if seconds > 0 else 0.0
