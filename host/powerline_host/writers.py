"""Optional capture outputs: CSV of raw millivolt samples, and a WAV file."""

import struct
import wave

from .protocol import MAINS_MV_PEAK

WAV_FALLBACK_RATE = 44100.0     # only used if we never measured a rate at all

_PCM_SCALE = 32767.0 / MAINS_MV_PEAK


def mv_to_pcm16(sample):
    """Convert a signed mains-millivolt sample to signed 16-bit PCM.

    Already centered at 0 (it's an AC waveform), just scaled to fill
    [-32768..32767]. Clamped because transient spikes/noise can occasionally
    exceed the nominal peak.
    """
    v = int(round(sample * _PCM_SCALE))
    return max(-32768, min(32767, v))


def learn_sample_rate(reported_rates, fallback_rate):
    """Pick the sample rate to stamp into the WAV header.

    Preference order: the mean of the per-second rate reports (what we
    actually measured), then a whole-run average for captures too short to
    have produced any report, then a hard-coded default.

    Returns (rate_hz, human_readable_source).
    """
    if reported_rates:
        return (sum(reported_rates) / len(reported_rates),
                f"mean of {len(reported_rates)} per-second rate report(s)")
    if fallback_rate > 0:
        return fallback_rate, "whole-run average (no per-second reports)"
    return WAV_FALLBACK_RATE, "fallback default (no measurements available)"


class CsvWriter:
    """One millivolt sample per line."""

    def __init__(self, path):
        self.path = path
        self._fh = open(path, "w")

    def write(self, samples):
        self._fh.write("\n".join(str(s) for s in samples))
        self._fh.write("\n")

    def close(self):
        self._fh.close()
        print(f"Wrote {self.path}")


class WavRecorder:
    """Buffers samples as 16-bit PCM until the measured sample rate is known.

    The WAV rate is not hard-coded: it is *learned* from the run, so a device
    actually running at, say, 9998 Hz produces a file that plays back at
    real-time speed.
    """

    def __init__(self, path):
        self.path = path
        self._pcm = bytearray()

    def write(self, samples):
        self._pcm.extend(
            struct.pack(f"<{len(samples)}h", *(mv_to_pcm16(s) for s in samples))
        )

    def save(self, reported_rates, fallback_rate):
        n_frames = len(self._pcm) // 2
        if n_frames == 0:
            print(f"  !! no post-warmup samples captured; skipping {self.path}")
            return

        learned_rate, source = learn_sample_rate(reported_rates, fallback_rate)
        rate = int(round(learned_rate))
        try:
            with wave.open(self.path, "wb") as wf:
                wf.setnchannels(1)
                wf.setsampwidth(2)      # 16-bit PCM
                wf.setframerate(rate)
                wf.writeframes(bytes(self._pcm))
        except OSError as e:
            print(f"  !! failed to write {self.path}: {e}")
            return

        duration = n_frames / rate if rate > 0 else 0.0
        print(f"Wrote {self.path}: {n_frames} samples, {rate} Hz "
              f"(learned from {source}), ~{duration:.2f}s of audio")
