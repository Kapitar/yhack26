"""
Hiwonder miniAuto — Bluetooth serial driver.

After pairing the robot on Windows, it appears as a COM port (e.g. COM5).
Find it in Device Manager → Ports (COM & LPT).

Protocol: commands end with &, fields separated by |
  A|<dir>|&   move in a direction
  C|<spd>|&   set speed (0-100)
  A|0|&       stop

Direction codes (A command):
  0 = stop
  2 = forward
  1 = backward
  3 = left turn
  4 = right turn
"""

import serial
import serial.tools.list_ports
import time

BAUD_RATE = 9600

# ── Movement commands ─────────────────────────────────────────────────────────
STOP         = b"A|0|&"
FORWARD      = b"A|2|&"
TURN_LEFT    = b"A|3|&"
TURN_RIGHT   = b"A|4|&"

def speed_cmd(speed: int) -> bytes:
    """speed: 0-100"""
    s = max(0, min(100, speed))
    return f"C|{s}|&".encode()


class MiniAuto:
    def __init__(self, port: str = None, speed: int = 60):
        self._port   = port or _find_port()
        self._speed  = speed
        self._serial = None

    def connect(self):
        self._serial = serial.Serial(self._port, BAUD_RATE, timeout=1.0)
        time.sleep(1.0)
        self.set_speed(self._speed)
        print(f"[robot] connected on {self._port}")

    def disconnect(self):
        self.stop()
        if self._serial and self._serial.is_open:
            self._serial.close()
        print("[robot] disconnected")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    def forward(self):
        self._send(FORWARD)

    def turn_left(self):
        self._send(TURN_LEFT)

    def turn_right(self):
        self._send(TURN_RIGHT)

    def stop(self):
        self._send(STOP)

    def set_speed(self, speed: int):
        self._speed = speed
        self._send(speed_cmd(speed))

    def _send(self, cmd: bytes):
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(cmd)
            except serial.SerialException as e:
                print(f"[robot] write error: {e}")


def _find_port() -> str:
    """Auto-detect the paired miniAuto COM port."""
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        if any(kw in desc for kw in ("bluetooth", "btspp", "wireless", "hiwonder")):
            print(f"[robot] auto-detected {p.device} ({p.description})")
            return p.device
    # Fallback: let the user know what's available
    ports = [p.device for p in serial.tools.list_ports.comports()]
    raise RuntimeError(
        "Could not auto-detect miniAuto COM port.\n"
        f"Available ports: {ports}\n"
        "Pass --port COM5 (or whichever port the robot is on).\n"
        "Tip: check Device Manager → Ports (COM & LPT) after pairing."
    )
