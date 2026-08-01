"""Wire format spoken by the ESP32-C3 firmware over USB CDC.

Each frame on the wire looks like:

    A5 5A | seq:uint32 | overflows:uint16 | count:uint16 | count x int32

all little-endian. The payload samples are already-reconstructed
*instantaneous mains voltage in millivolts* (signed): the device inverts its
sensing front-end's fixed gain and DC bias (transformer + RC filter + bias
network) before streaming, so the host never sees raw ADC codes.

See src/wire_protocol.h for the firmware side of this contract.
"""

import struct
from typing import NamedTuple

MAGIC = b"\xA5\x5A"
HEADER = struct.Struct("<IHH")   # seq, overflows, count -- follows the magic word
SAMPLE_BYTES = 4                 # int32 per sample

MAX_COUNT = 4096                 # sanity ceiling for a frame's sample count
EXPECTED_COUNT = 256             # firmware SAMPLES_PER_FRAME -- any deviation is a red flag

# Expected peak magnitude at a 240 Vrms max mains input, in millivolts.
MAINS_MV_PEAK = 339400


class GarbledFrame(Exception):
    """Frame header failed its sanity check; the caller should re-sync."""


class Frame(NamedTuple):
    seq: int              # frame sequence number, wraps at 2^32
    overflows: int        # device-side DMA pool overflow counter (cumulative)
    samples: tuple        # signed mains millivolts
    trailing_magic: bool  # False if the next frame's magic word wasn't where we expected it


def read_frame(link):
    """Read one frame from `link`, assuming its magic word was just consumed.

    Raises GarbledFrame if the header is implausible, or link.Disconnected if
    the underlying link dies.
    """
    seq, overflows, count = HEADER.unpack(link.read_exact(HEADER.size))
    if count == 0 or count > MAX_COUNT:
        raise GarbledFrame(f"implausible sample count {count}")

    payload = link.read_exact(count * SAMPLE_BYTES)
    samples = struct.unpack(f"<{count}i", payload)

    # The frame is immediately followed by the next frame's magic word. If it
    # isn't, we've lost alignment and the caller must re-sync.
    trailing_magic = link.read_exact(len(MAGIC)) == MAGIC

    return Frame(seq, overflows, samples, trailing_magic)
