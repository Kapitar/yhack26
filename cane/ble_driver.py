"""
LeadMe — miniAuto BLE Driver (Windows)
Laptop ↔ HM-10 BLE module (FFE0/FFE1) ↔ Arduino UNO

The HM-10 module acts as a transparent UART bridge:
  - Writing to FFE1  →  Arduino Serial.read()
  - Arduino Serial.print()  →  FFE1 notify  →  our callback

Usage:
    async with MiniAutoBLE() as driver:
        await driver.move(angle=0, velocity=80)
        imu = driver.get_imu()  # latest IMU reading

Or with the sync wrapper:
    driver = MiniAutoBLESync()
    driver.connect()
    driver.move(angle=0, velocity=80)
    imu = driver.get_imu()
    driver.disconnect()
"""

import asyncio
import threading
import logging
import time
from dataclasses import dataclass
from typing import Optional, Callable

from bleak import BleakClient, BleakScanner

logger = logging.getLogger(__name__)

# ── HM-10 BLE UART UUIDs ─────────────────────────────────────────────────────
SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB"
CHAR_UUID    = "0000FFE1-0000-1000-8000-00805F9B34FB"

# Device name advertised by the HM-10 module (default: "HMSoft" or "BT05")
# Change this to match your module's advertised name.
DEVICE_NAME  = "Hiwonder"

ACCEL_SCALE  = 16384.0   # raw → g  (±2g range)
GYRO_SCALE   = 131.0     # raw → °/s (±250°/s range)


@dataclass
class ImuReading:
    ax: float = 0.0   # g
    ay: float = 0.0
    az: float = 0.0
    gx: float = 0.0   # °/s
    gy: float = 0.0
    gz: float = 0.0
    timestamp: float = 0.0


# ── Async BLE driver ──────────────────────────────────────────────────────────

class MiniAutoBLE:
    """
    Async BLE driver. Use with `async with MiniAutoBLE() as driver`.

    All motor commands are fire-and-forget (no ack from Arduino).
    IMU data arrives as BLE notifications and is stored in _latest_imu.
    """

    def __init__(
        self,
        device_name: str = DEVICE_NAME,
        imu_callback: Optional[Callable[[ImuReading], None]] = None,
    ):
        self._device_name    = device_name
        self._imu_callback   = imu_callback
        self._client: Optional[BleakClient] = None
        self._latest_imu     = ImuReading()
        self._rx_buf         = ""
        self._write_handle   = None   # characteristic handle for sending commands
        self._notify_handle  = None   # characteristic handle for receiving IMU data

    # ── Connection ────────────────────────────────────────────────────────────

    async def connect(self) -> None:
        logger.info(f"Scanning for BLE device '{self._device_name}'...")
        device = await BleakScanner.find_device_by_name(
            self._device_name, timeout=10.0
        )
        if device is None:
            raise RuntimeError(
                f"BLE device '{self._device_name}' not found. "
                "Make sure the miniAuto is powered on and in range.\n"
                "Run: python cane\\ble_driver.py scan  to list nearby devices."
            )

        self._client = BleakClient(device)
        await self._client.connect()
        logger.info(f"Connected to {device.name} ({device.address})")

        # The HM-10 exposes two characteristics with the same FFE1 UUID:
        # one supports Write/WriteWithoutResponse, the other supports Notify.
        # Select each by its properties using the handle to avoid UUID ambiguity.
        all_chars = [
            c
            for service in self._client.services
            for c in service.characteristics
            if c.uuid.lower() == CHAR_UUID.lower()
        ]
        logger.info(f"Found {len(all_chars)} FFE1 characteristic(s):")
        for c in all_chars:
            logger.info(f"  handle={c.handle}  properties={c.properties}")

        for c in all_chars:
            props = set(c.properties)
            if "notify" in props and self._notify_handle is None:
                self._notify_handle = c.handle
            if ("write-without-response" in props or "write" in props) \
                    and self._write_handle is None:
                self._write_handle = c.handle

        # If both roles share the same characteristic (single-char HM-10 variants)
        if self._notify_handle is None and self._write_handle is not None:
            self._notify_handle = self._write_handle
        if self._write_handle is None and self._notify_handle is not None:
            self._write_handle = self._notify_handle

        if self._notify_handle is None:
            raise RuntimeError(
                "Could not find a usable FFE1 characteristic. "
                "Check that the miniAuto firmware is flashed correctly."
            )

        await self._client.start_notify(self._notify_handle, self._on_notify)
        logger.info(
            f"Ready — write handle={self._write_handle}, "
            f"notify handle={self._notify_handle}"
        )

    async def disconnect(self) -> None:
        if self._client and self._client.is_connected:
            await self._send_raw(b"S\n")
            if self._notify_handle is not None:
                await self._client.stop_notify(self._notify_handle)
            await self._client.disconnect()
            logger.info("Disconnected from miniAuto")

    async def __aenter__(self):
        await self.connect()
        return self

    async def __aexit__(self, *_):
        await self.disconnect()

    # ── Motor control API ─────────────────────────────────────────────────────

    async def move(self, angle: int, velocity: int, rot: int = 0) -> None:
        """
        Move in a direction.
        angle    : 0–359°  (0=forward, 90=strafe-left, 180=back, 270=strafe-right)
        velocity : 0–100
        rot      : -100–100  (positive=CCW, negative=CW)
        """
        angle    = int(angle) % 360
        velocity = max(0,    min(100, int(velocity)))
        rot      = max(-100, min(100, int(rot)))
        cmd      = f"{angle}|{velocity}|{rot}\n".encode("ascii")
        await self._send_raw(cmd)

    async def stop(self) -> None:
        await self._send_raw(b"S\n")

    async def rotate(self, rate: int) -> None:
        await self.move(angle=0, velocity=0, rot=rate)

    async def forward(self, speed: int = 60) -> None:
        await self.move(angle=0, velocity=speed)

    async def backward(self, speed: int = 60) -> None:
        await self.move(angle=180, velocity=speed)

    async def strafe_left(self, speed: int = 60) -> None:
        await self.move(angle=90, velocity=speed)

    async def strafe_right(self, speed: int = 60) -> None:
        await self.move(angle=270, velocity=speed)

    async def tug(self, heading_error_deg: float, base_speed: int = 60) -> None:
        """
        Apply a lateral tug based on PID heading error output.
        heading_error_deg: positive = user needs to go right,
                           negative = user needs to go left.
        """
        error      = max(-90.0, min(90.0, heading_error_deg))
        move_angle = int(-error) % 360
        velocity   = min(100, base_speed + int(abs(error) * 0.3))
        await self.move(angle=move_angle, velocity=velocity)

    # ── IMU data ──────────────────────────────────────────────────────────────

    def get_imu(self) -> ImuReading:
        """Return the most recent IMU reading. Non-blocking."""
        return self._latest_imu

    # ── Internal ──────────────────────────────────────────────────────────────

    async def _send_raw(self, data: bytes) -> None:
        """Write bytes to the BLE characteristic (→ Arduino Serial)."""
        if self._client and self._client.is_connected and self._write_handle:
            await self._client.write_gatt_char(self._write_handle, data, response=False)

    def _on_notify(self, sender, data: bytearray) -> None:
        """BLE notification callback — called on bleak's internal thread."""
        try:
            self._rx_buf += data.decode("ascii", errors="ignore")
            while "\n" in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split("\n", 1)
                line = line.strip()
                if line:
                    self._parse_line(line)
        except Exception as e:
            logger.warning(f"Notification parse error: {e}")

    def _parse_line(self, line: str) -> None:
        """Parse one complete line from the Arduino."""
        if line.startswith("I:"):
            # Format: "I:<ax>,<ay>,<az>,<gx>,<gy>,<gz>"
            parts = line[2:].split(",")
            if len(parts) == 6:
                try:
                    imu = ImuReading(
                        ax=int(parts[0]) / ACCEL_SCALE,
                        ay=int(parts[1]) / ACCEL_SCALE,
                        az=int(parts[2]) / ACCEL_SCALE,
                        gx=int(parts[3]) / GYRO_SCALE,
                        gy=int(parts[4]) / GYRO_SCALE,
                        gz=int(parts[5]) / GYRO_SCALE,
                        timestamp=time.monotonic(),
                    )
                    self._latest_imu = imu
                    if self._imu_callback:
                        self._imu_callback(imu)
                except ValueError:
                    pass
        elif line == "OK":
            logger.debug("Arduino ping OK")
        else:
            logger.debug(f"Arduino: {line}")


# ── Sync wrapper ──────────────────────────────────────────────────────────────

class MiniAutoBLESync:
    """
    Synchronous wrapper around MiniAutoBLE for use in non-async code.
    Runs the asyncio event loop in a background thread.

    Usage:
        driver = MiniAutoBLESync()
        driver.connect()
        driver.move(angle=0, velocity=80)
        imu = driver.get_imu()
        driver.disconnect()
    """

    def __init__(self, device_name: str = DEVICE_NAME,
                 imu_callback: Optional[Callable[[ImuReading], None]] = None):
        self._async_driver = MiniAutoBLE(device_name, imu_callback)
        self._loop   = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._loop.run_forever, daemon=True)
        self._thread.start()

    def _run(self, coro):
        return asyncio.run_coroutine_threadsafe(coro, self._loop).result(timeout=5.0)

    def connect(self):
        self._run(self._async_driver.connect())

    def disconnect(self):
        self._run(self._async_driver.disconnect())
        self._loop.call_soon_threadsafe(self._loop.stop)

    def move(self, angle: int, velocity: int, rot: int = 0):
        self._run(self._async_driver.move(angle, velocity, rot))

    def stop(self):
        self._run(self._async_driver.stop())

    def rotate(self, rate: int):
        self._run(self._async_driver.rotate(rate))

    def forward(self, speed: int = 60):
        self._run(self._async_driver.forward(speed))

    def backward(self, speed: int = 60):
        self._run(self._async_driver.backward(speed))

    def strafe_left(self, speed: int = 60):
        self._run(self._async_driver.strafe_left(speed))

    def strafe_right(self, speed: int = 60):
        self._run(self._async_driver.strafe_right(speed))

    def tug(self, heading_error_deg: float, base_speed: int = 60):
        self._run(self._async_driver.tug(heading_error_deg, base_speed))

    def get_imu(self) -> ImuReading:
        return self._async_driver.get_imu()

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *_):
        self.disconnect()


# ── CLI utilities ─────────────────────────────────────────────────────────────

async def _scan():
    """List nearby BLE devices to find your HM-10 module's name/address."""
    print("Scanning for BLE devices (10 seconds)...\n")
    # return_adv=True gives (BLEDevice, AdvertisementData) pairs — RSSI lives in adv data
    results = await BleakScanner.discover(timeout=10.0, return_adv=True)
    if not results:
        print("No devices found.")
        return
    print(f"{'Name':<30} {'Address':<20} RSSI")
    print("-" * 60)
    entries = [
        (dev.name or "Unknown", dev.address, adv.rssi or -999)
        for dev, adv in results.values()
    ]
    for name, address, rssi in sorted(entries, key=lambda x: x[2], reverse=True):
        print(f"{name:<30} {address:<20} {rssi} dBm")
    print(f"\nSet DEVICE_NAME in ble_driver.py to match your module's name.")


async def _test(device_name: str):
    """Run a movement test sequence using the leadme_firmware protocol."""
    print(f"Connecting to '{device_name}'...")
    async with MiniAutoBLE(device_name) as driver:
        print("Connected. Running test sequence (leadme_firmware protocol).")
        for label, error, duration in [
            ("Straight (error=0)",   0.0,   2),
            ("Tug right (error=45)", 45.0,  2),
            ("Tug left (error=-45)", -45.0, 2),
            ("Hard right (error=90)",  90.0, 2),
            ("Hard left (error=-90)", -90.0, 2),
        ]:
            print(f"  {label}...")
            await driver._send_raw(f"E:{error:.1f}:0.00\n".encode("ascii"))
            await asyncio.sleep(duration)

        await driver.stop()
        print("Done.")


if __name__ == "__main__":
    import sys
    logging.basicConfig(level=logging.INFO)

    cmd = sys.argv[1] if len(sys.argv) > 1 else "test"

    if cmd == "scan":
        asyncio.run(_scan())
    elif cmd == "test":
        name = sys.argv[2] if len(sys.argv) > 2 else DEVICE_NAME
        asyncio.run(_test(name))
    else:
        print("Usage:")
        print("  python ble_driver.py scan           # list nearby BLE devices")
        print("  python ble_driver.py test [name]    # movement + IMU test")
