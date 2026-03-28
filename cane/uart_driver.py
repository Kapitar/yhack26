"""
LeadMe — miniAuto UART Driver
Jetson Orin Nano → USB → Arduino (miniAuto)

Usage:
    driver = MiniAutoDriver()
    driver.connect()
    driver.move(angle=0, velocity=80)        # forward
    driver.move(angle=90, velocity=60)       # strafe left
    driver.rotate(rate=50)                   # spin CCW
    driver.move(angle=45, velocity=70, rot=-20)  # forward-left + slight CW rotation
    driver.stop()
    driver.disconnect()

Or as a context manager:
    with MiniAutoDriver() as driver:
        driver.move(angle=0, velocity=80)
"""

import serial
import serial.tools.list_ports
import time
import threading
import logging

logger = logging.getLogger(__name__)

# ── Constants ─────────────────────────────────────────────────────────────────

BAUD_RATE    = 9600
SEND_HZ      = 50        # command rate for the keep-alive loop
TIMEOUT_S    = 1.0       # serial read timeout
PING_TIMEOUT = 2.0       # seconds to wait for ping response


class MiniAutoDriver:
    """
    Serial driver for the Hiwonder miniAuto running LeadMe firmware.

    Thread-safe: move()/rotate()/stop() can be called from any thread.
    A background thread sends the last command at SEND_HZ to keep the
    Arduino watchdog alive and ensure smooth motor response.
    """

    def __init__(self, port: str = None, baud: int = BAUD_RATE):
        self._port   = port
        self._baud   = baud
        self._serial: serial.Serial = None

        # Current command state (angle, velocity, rot)
        self._angle    = 0
        self._velocity = 0
        self._rot      = 0
        self._lock     = threading.Lock()

        self._running  = False
        self._thread: threading.Thread = None

    # ── Connection ────────────────────────────────────────────────────────────

    def connect(self) -> bool:
        """
        Open serial connection to the Arduino.
        If port is None, auto-detects the first Arduino-compatible port.
        Returns True on success.
        """
        port = self._port or self._auto_detect_port()
        if port is None:
            raise RuntimeError(
                "No Arduino found. Specify port explicitly or check USB connection."
            )

        self._serial = serial.Serial(
            port=port,
            baudrate=self._baud,
            timeout=TIMEOUT_S
        )
        # Arduino resets on serial connect — wait for it to boot
        time.sleep(2.0)
        self._serial.reset_input_buffer()

        if not self._ping():
            raise RuntimeError(
                f"Arduino on {port} did not respond to ping. "
                "Check firmware is flashed correctly."
            )

        logger.info(f"Connected to miniAuto on {port} at {self._baud} baud")

        # Start keep-alive background thread
        self._running = True
        self._thread  = threading.Thread(target=self._send_loop, daemon=True)
        self._thread.start()
        return True

    def disconnect(self):
        """Stop motors and close connection."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            self._send_command(0, 0, 0)   # ensure stopped before closing
            self._serial.close()
        logger.info("Disconnected from miniAuto")

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()

    # ── Public control API ────────────────────────────────────────────────────

    def move(self, angle: int, velocity: int, rot: int = 0):
        """
        Move in a direction.

        angle    : 0–359 degrees. 0 = forward, 90 = strafe left,
                   180 = backward, 270 = strafe right.
        velocity : 0–100 speed magnitude.
        rot      : -100–100 simultaneous rotation.
                   Positive = CCW, negative = CW.
        """
        angle    = int(angle)    % 360
        velocity = max(0,   min(100, int(velocity)))
        rot      = max(-100, min(100, int(rot)))
        with self._lock:
            self._angle    = angle
            self._velocity = velocity
            self._rot      = rot

    def rotate(self, rate: int):
        """
        Rotate in place.
        rate: -100–100. Positive = CCW, negative = CW.
        """
        self.move(angle=0, velocity=0, rot=rate)

    def stop(self):
        """Stop all motors immediately."""
        with self._lock:
            self._angle    = 0
            self._velocity = 0
            self._rot      = 0
        if self._serial and self._serial.is_open:
            self._raw_write(b"S\n")

    # ── Directional helpers (convenience wrappers) ────────────────────────────

    def forward(self, speed: int = 60):
        self.move(angle=0, velocity=speed)

    def backward(self, speed: int = 60):
        self.move(angle=180, velocity=speed)

    def strafe_left(self, speed: int = 60):
        self.move(angle=90, velocity=speed)

    def strafe_right(self, speed: int = 60):
        self.move(angle=270, velocity=speed)

    def tug(self, heading_error_deg: float, base_speed: int = 60):
        """
        Apply a directional tug based on heading error from navigation PID.

        heading_error_deg: positive = user needs to go right,
                           negative = user needs to go left.
        Translates PID output directly to a lateral force angle.
        """
        # Clamp error to ±90° for safety
        error = max(-90, min(90, heading_error_deg))
        # Map error to movement angle:
        #   0 error  → pure forward (angle=0)
        #   +ve error → rightward component (angle < 0 → wraps to ~270–360)
        #   -ve error → leftward component (angle > 0 → ~0–90)
        move_angle = int(-error) % 360
        velocity   = min(100, base_speed + int(abs(error) * 0.3))
        self.move(angle=move_angle, velocity=velocity)

    # ── Internal ──────────────────────────────────────────────────────────────

    def _auto_detect_port(self) -> str:
        """Scan serial ports and return the first Arduino-compatible one."""
        for port_info in serial.tools.list_ports.comports():
            desc = (port_info.description or "").lower()
            mfr  = (port_info.manufacturer or "").lower()
            if any(kw in desc or kw in mfr for kw in
                   ("arduino", "ch340", "ch341", "ftdi", "usb serial")):
                logger.info(f"Auto-detected Arduino on {port_info.device}")
                return port_info.device
        # Fallback: first /dev/ttyUSB or /dev/ttyACM
        for port_info in serial.tools.list_ports.comports():
            if "ttyUSB" in port_info.device or "ttyACM" in port_info.device:
                return port_info.device
        return None

    def _ping(self) -> bool:
        """Send P command and wait for OK response."""
        self._raw_write(b"P\n")
        deadline = time.time() + PING_TIMEOUT
        while time.time() < deadline:
            if self._serial.in_waiting:
                line = self._serial.readline().decode("ascii", errors="ignore").strip()
                if line == "OK":
                    return True
        return False

    def _send_command(self, angle: int, velocity: int, rot: int):
        """Encode and write one movement command."""
        cmd = f"{angle}|{velocity}|{rot}\n".encode("ascii")
        self._raw_write(cmd)

    def _raw_write(self, data: bytes):
        try:
            if self._serial and self._serial.is_open:
                self._serial.write(data)
        except serial.SerialException as e:
            logger.error(f"Serial write failed: {e}")

    def _send_loop(self):
        """
        Background thread: sends the current command at SEND_HZ.
        This keeps the Arduino watchdog alive and ensures motor
        commands reach the Arduino even if the caller doesn't re-send.
        """
        interval = 1.0 / SEND_HZ
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                angle, velocity, rot = self._angle, self._velocity, self._rot
            self._send_command(angle, velocity, rot)
            elapsed = time.monotonic() - t0
            sleep_time = interval - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)


# ── CLI test harness ──────────────────────────────────────────────────────────

if __name__ == "__main__":
    import sys

    logging.basicConfig(level=logging.INFO)

    port = sys.argv[1] if len(sys.argv) > 1 else None

    print("Connecting to miniAuto...")
    with MiniAutoDriver(port=port) as driver:
        print("Connected. Running movement test sequence.")

        print("Forward 2s")
        driver.forward(speed=60)
        time.sleep(2)

        print("Strafe left 2s")
        driver.strafe_left(speed=60)
        time.sleep(2)

        print("Strafe right 2s")
        driver.strafe_right(speed=60)
        time.sleep(2)

        print("Rotate CCW 2s")
        driver.rotate(rate=50)
        time.sleep(2)

        print("Rotate CW 2s")
        driver.rotate(rate=-50)
        time.sleep(2)

        print("Stop")
        driver.stop()
        time.sleep(1)

    print("Done.")
