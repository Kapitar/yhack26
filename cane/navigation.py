"""
LeadMe — Navigation Loop
Runs on the laptop.

Flow:
  Phone (WebSocket) ──► navigation.py ──► BLE ──► Arduino ──► motors
                              ▲
                         IMU feedback
                       (gyro D-term)

WebSocket protocol (phone → laptop):
  {
    "type":           "nav",
    "target_bearing": 95.0,   // degrees to next waypoint  (0–360)
    "device_heading": 82.0,   // phone compass/GPS heading  (0–360, or null)
    "distance":       14.3,   // metres to next waypoint
    "lat":            40.748,
    "lng":           -73.985,
    "speed":          1.1     // m/s (optional)
  }

WebSocket protocol (laptop → phone):
  {
    "type":          "status",
    "heading_error": 13.0,    // degrees, for UI display
    "pid_output":    15.6,    // raw PID output
    "connected":     true
  }

Usage:
    python navigation.py
"""

import asyncio
import json
import logging
import time
from dataclasses import dataclass
from typing import Optional

import websockets

from ble_driver import MiniAutoBLE, DEVICE_NAME
from pid import HeadingPID

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(name)s  %(message)s",
)
logger = logging.getLogger("navigation")

# ── Configuration ─────────────────────────────────────────────────────────────

WS_HOST        = "0.0.0.0"
WS_PORT        = 8765

# PID gains — tune these once the hardware is running
PID_KP         = 1.2
PID_KI         = 0.04
PID_KD         = 0.25

# Minimum heading error (degrees) before applying any correction.
# Avoids constant micro-corrections when roughly on course.
DEAD_ZONE_DEG  = 5.0

# Speed sent to tug() as the base — scales with heading error automatically.
BASE_TUG_SPEED = 55


# ── Shared navigation state ───────────────────────────────────────────────────

@dataclass
class NavPacket:
    target_bearing: float
    device_heading: Optional[float]
    distance:       float
    lat:            float
    lng:            float
    received_at:    float = 0.0


_latest_nav:    Optional[NavPacket] = None
_phone_clients: set = set()


def _normalize_angle(deg: float) -> float:
    return ((deg + 180) % 360) - 180


# ── WebSocket server (phone → laptop) ─────────────────────────────────────────

async def _ws_handler(websocket):
    global _latest_nav
    _phone_clients.add(websocket)
    logger.info(f"Phone connected from {websocket.remote_address}")
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if data.get("type") != "nav":
                continue

            _latest_nav = NavPacket(
                target_bearing = float(data["target_bearing"]),
                device_heading = float(data["device_heading"])
                                 if data.get("device_heading") is not None else None,
                distance       = float(data.get("distance", 0)),
                lat            = float(data.get("lat", 0)),
                lng            = float(data.get("lng", 0)),
                received_at    = time.monotonic(),
            )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        _phone_clients.discard(websocket)
        logger.info("Phone disconnected")


async def _broadcast_status(payload: dict):
    if not _phone_clients:
        return
    msg = json.dumps(payload)
    await asyncio.gather(
        *[ws.send(msg) for ws in _phone_clients],
        return_exceptions=True,
    )


# ── PID control loop ──────────────────────────────────────────────────────────

async def _control_loop(ble: MiniAutoBLE):
    pid     = HeadingPID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
    nav_age = 2.0   # seconds before we consider nav data stale

    logger.info("Control loop started")

    while True:
        await asyncio.sleep(0.05)   # 20 Hz

        nav = _latest_nav

        # ── No nav data or arrived ────────────────────────────────────────────
        if nav is None or (time.monotonic() - nav.received_at) > nav_age:
            await ble.stop()
            pid.reset()
            continue

        if nav.distance < 2.0:
            # Close enough — stop and wait for next waypoint
            await ble.stop()
            pid.reset()
            continue

        # ── Heading error ─────────────────────────────────────────────────────
        if nav.device_heading is not None:
            heading_error = _normalize_angle(nav.target_bearing - nav.device_heading)
        else:
            # Phone heading unavailable (stationary or no compass).
            # Can't compute a meaningful error — hold still.
            await ble.stop()
            continue

        # ── IMU D-term ────────────────────────────────────────────────────────
        imu       = ble.get_imu()
        gyro_z    = imu.gz   # °/s yaw rate

        # ── PID ───────────────────────────────────────────────────────────────
        pid_output = pid.compute(heading_error, gyro_z)

        # ── Dead zone ─────────────────────────────────────────────────────────
        if abs(heading_error) < DEAD_ZONE_DEG:
            await ble.forward(speed=BASE_TUG_SPEED)
            pid.reset()
            await _broadcast_status({
                "type": "status",
                "heading_error": round(heading_error, 1),
                "pid_output":    0,
                "connected":     True,
            })
            continue

        # ── Apply correction ──────────────────────────────────────────────────
        # pid_output is signed degrees-equivalent:
        #   positive → tug right  (angle = 270°)
        #   negative → tug left   (angle = 90°)
        # We blend a forward component with the lateral correction so the cane
        # still moves forward rather than purely strafing.
        correction_magnitude = abs(pid_output)
        correction_magnitude = min(100.0, correction_magnitude)

        if pid_output >= 0:
            # Tug right: blend forward (0°) with strafe-right (270°)
            blend_angle = int(360 - min(90, correction_magnitude * 0.6)) % 360
        else:
            # Tug left: blend forward (0°) with strafe-left (90°)
            blend_angle = int(min(90, correction_magnitude * 0.6))

        tug_speed = int(BASE_TUG_SPEED + correction_magnitude * 0.25)
        tug_speed = min(100, tug_speed)

        await ble.move(angle=blend_angle, velocity=tug_speed)

        logger.debug(
            f"error={heading_error:+.1f}°  gyro_z={gyro_z:+.1f}°/s  "
            f"pid={pid_output:+.1f}  angle={blend_angle}°  speed={tug_speed}"
        )

        await _broadcast_status({
            "type":          "status",
            "heading_error": round(heading_error, 1),
            "pid_output":    round(pid_output, 1),
            "connected":     True,
        })


# ── Entry point ───────────────────────────────────────────────────────────────

async def main():
    logger.info(f"Connecting to miniAuto BLE ({DEVICE_NAME})...")
    ble = MiniAutoBLE()
    await ble.connect()
    logger.info("BLE connected")

    logger.info(f"Starting WebSocket server on ws://0.0.0.0:{WS_PORT}")
    logger.info("Point your phone to:  ws://<THIS-LAPTOP-IP>:{WS_PORT}")

    ws_server    = websockets.serve(_ws_handler, WS_HOST, WS_PORT)
    control_task = asyncio.create_task(_control_loop(ble))

    async with ws_server:
        try:
            await asyncio.Future()   # run forever
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            control_task.cancel()
            await ble.stop()
            await ble.disconnect()
            logger.info("Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
