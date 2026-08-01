#!/usr/bin/env python3
"""PC-side reader/verifier for the ESP32-C3 mains-voltage stream.

Usage:
    pip install pyserial
    python host/pc_reader.py <PORT> [out.csv] [--wav out.wav]

    # examples
    python host/pc_reader.py /dev/ttyACM0
    python host/pc_reader.py COM7 capture.csv
    python host/pc_reader.py /dev/ttyACM0 --wav capture.wav
    python host/pc_reader.py /dev/ttyACM0 capture.csv --wav capture.wav

Samples arrive already converted to millivolts of the *instantaneous mains
voltage* (not the raw ADC pin voltage): the device inverts its sensing
front-end's fixed gain and DC bias (transformer + RC filter + bias network)
to recover the mains waveform, streaming signed 32-bit millivolt values with
an expected range of roughly -339400..+339400 mV at a 240 Vrms max mains
input.

The reader re-syncs on the magic word, parses each frame, survives USB
disconnects, and once per second reports the effective sample rate, the RMS
mains voltage, the mains frequency, and a set of stream-integrity counters.
Vrms and frequency are live sanity checks against a trusted multimeter on the
same outlet.

It can optionally write every sample to a CSV and/or a 16-bit PCM mono WAV.

The implementation lives in the powerline_host package next to this file;
see powerline_host/__init__.py for the module map.
"""

from powerline_host.cli import main

if __name__ == "__main__":
    main()
