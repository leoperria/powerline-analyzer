"""Command-line entry point for the PC-side reader."""

import argparse
import sys

from .session import CaptureSession
from .writers import CsvWriter, WavRecorder


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        prog="pc_reader.py",
        description="Read and verify the ESP32-C3 mains-voltage stream over USB CDC.",
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
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    session = CaptureSession(
        args.port,
        csv=CsvWriter(args.csv) if args.csv else None,
        wav=WavRecorder(args.wav) if args.wav else None,
    )
    try:
        session.run()
    except TimeoutError as e:
        sys.exit(str(e))
