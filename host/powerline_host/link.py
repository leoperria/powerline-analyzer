"""Serial link to the device, with reconnect and stall detection.

If the USB link disappears mid-stream the reader closes the port, waits for
the device node to reappear, and resumes -- for up to RECONNECT_TIMEOUT_S
seconds.
"""

import time

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - environment problem, not logic
    raise SystemExit("Install pyserial:  pip install pyserial")

from .protocol import MAGIC

BAUDRATE = 115200                # ignored by USB CDC, but pyserial wants a value
RECONNECT_TIMEOUT_S = 60.0       # give up after this long trying to reopen the port
RECONNECT_INTERVAL_S = 0.5       # poll interval while waiting for device to come back
STALL_TIMEOUT_S = 5.0            # no bytes for this long => assume the link is dead


class Disconnected(Exception):
    """The USB CDC link errored out or stopped delivering bytes."""


class SerialLink:
    """A byte stream from the device that knows how to re-find frame boundaries."""

    def __init__(self, port, stall_timeout=STALL_TIMEOUT_S):
        self.port = port
        self.stall_timeout = stall_timeout
        self._ser = None

    # ---- connection management ----

    def open(self, retry_seconds=RECONNECT_TIMEOUT_S):
        """Open the port, retrying while the device node is missing or busy.

        Raises TimeoutError if it does not become available in time.
        """
        deadline = time.time() + retry_seconds
        announced = False
        while True:
            try:
                self._ser = serial.Serial(self.port, baudrate=BAUDRATE, timeout=0.1)
                return
            except (serial.SerialException, OSError) as e:
                if not announced:
                    print(f"  ... waiting up to {retry_seconds:.0f}s for "
                          f"{self.port} to (re)appear ...")
                    announced = True
                if time.time() >= deadline:
                    raise TimeoutError(
                        f"could not open {self.port} within {retry_seconds:.0f}s: {e}")
                time.sleep(RECONNECT_INTERVAL_S)

    def close(self):
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:
            pass
        self._ser = None

    # ---- reading ----

    def read_exact(self, n):
        """Read exactly n bytes. Raise Disconnected if the link stalls or errors."""
        buf = bytearray()
        deadline = time.time() + self.stall_timeout
        while len(buf) < n:
            chunk = self._read_some(n - len(buf))
            if chunk:
                buf += chunk
                deadline = time.time() + self.stall_timeout
            elif time.time() > deadline:
                raise Disconnected(f"no data for {self.stall_timeout:.1f}s")
        return bytes(buf)

    def resync(self):
        """Advance the stream until we've just consumed a magic word."""
        prev = b""
        deadline = time.time() + self.stall_timeout
        while True:
            b = self._read_some(1)
            if not b:
                if time.time() > deadline:
                    raise Disconnected(f"no data for {self.stall_timeout:.1f}s")
                continue
            deadline = time.time() + self.stall_timeout
            if prev == MAGIC[0:1] and b == MAGIC[1:2]:
                return
            prev = b

    def _read_some(self, n):
        try:
            return self._ser.read(n)
        except (serial.SerialException, OSError) as e:
            raise Disconnected(f"read failed: {e}")
