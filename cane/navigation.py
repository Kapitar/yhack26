"""
LeadMe — Navigation Loop
Runs on the laptop.

Flow:
  Phone  (WebSocket :8765) ──► navigation.py ──► RPi (WebSocket :8766) ──► Arduino ──► motors
                                      ▲
                              IMU from RPi (RealSense D457 gyro)

WebSocket protocol — phone → laptop (:8765):
  {
    "type":           "nav",
    "target_bearing": 95.0,    // degrees to next waypoint  (0–360)
    "device_heading": 82.0,    // phone compass heading      (0–360, or null)
    "distance":       14.3,    // metres to next waypoint
    "lat":            40.748,
    "lng":           -73.985
  }

WebSocket protocol — laptop → phone (:8765):
  {
    "type":          "status",
    "heading_error": 13.0,
    "pid_output":    15.6,
    "connected":     true
  }

WebSocket protocol — RPi → laptop (:8766):
  {
    "type": "imu",
    "gx": 0.5, "gy": -0.1, "gz": 1.2,   // rad/s  (RealSense native units)
    "ax": 0.01, "ay": 0.0, "az": 9.81    // m/s²
  }

WebSocket protocol — laptop → RPi (:8766):
  { "type": "motor", "angle": 45, "velocity": 60, "rot": 0 }
  { "type": "stop" }

Usage:
    python navigation.py
"""

import asyncio
import json
import logging
import math
import time
from dataclasses import dataclass
from typing import Optional

import websockets

from pid import HeadingPID

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(name)s  %(message)s",
)
logger = logging.getLogger("navigation")

# ── Configuration ──────────────────────────────────────────────────────────────

PHONE_WS_HOST  = "0.0.0.0"
PHONE_WS_PORT  = 8765

RPI_WS_HOST    = "0.0.0.0"
RPI_WS_PORT    = 8766

PID_KP         = 1.2
PID_KI         = 0.04
PID_KD         = 0.25

DEAD_ZONE_DEG  = 5.0      # ignore errors smaller than this
BASE_TUG_SPEED = 55       # base motor speed (0–100)
NAV_AGE_S      = 2.0      # seconds before nav packet is considered stale

# RealSense gyro gives rad/s — convert to °/s for PID
_RAD_TO_DEG = 180.0 / math.pi


# ── Shared state ───────────────────────────────────────────────────────────────

@dataclass
class NavPacket:
    target_bearing: float
    device_heading: Optional[float]
    distance:       float
    lat:            float
    lng:            float
    received_at:    float = 0.0


@dataclass
class ImuReading:
    gx: float = 0.0   # rad/s (RealSense native)
    gy: float = 0.0
    gz: float = 0.0   # yaw rate — used for PID D-term
    ax: float = 0.0   # m/s²
    ay: float = 0.0
    az: float = 0.0
    timestamp: float = 0.0


_latest_nav:   Optional[NavPacket] = None
_latest_imu:   ImuReading          = ImuReading()
_phone_clients: set                 = set()
_rpi_ws:       Optional[websockets.WebSocketServerProtocol] = None


def _normalize_angle(deg: float) -> float:
    return ((deg + 180) % 360) - 180


# ── Phone WebSocket server (:8765) ────────────────────────────────────────────

async def _phone_handler(websocket):
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


async def _broadcast_phone(payload: dict) -> None:
    if not _phone_clients:
        return
    msg = json.dumps(payload)
    await asyncio.gather(
        *[ws.send(msg) for ws in _phone_clients],
        return_exceptions=True,
    )


# ── RPi WebSocket server (:8766) ──────────────────────────────────────────────

async def _rpi_handler(websocket):
    global _rpi_ws, _latest_imu
    _rpi_ws = websocket
    logger.info(f"RPi connected from {websocket.remote_address}")
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except json.JSONDecodeError:
                continue

            if data.get("type") == "imu":
                _latest_imu = ImuReading(
                    gx=float(data.get("gx", 0)),
                    gy=float(data.get("gy", 0)),
                    gz=float(data.get("gz", 0)),
                    ax=float(data.get("ax", 0)),
                    ay=float(data.get("ay", 0)),
                    az=float(data.get("az", 0)),
                    timestamp=time.monotonic(),
                )
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        if _rpi_ws is websocket:
            _rpi_ws = None
        logger.info("RPi disconnected — motors will stop")


async def _send_rpi(payload: dict) -> None:
    ws = _rpi_ws
    if ws is None:
        return
    try:
        await ws.send(json.dumps(payload))
    except websockets.exceptions.ConnectionClosed:
        pass


# ── PID control loop (20 Hz) ──────────────────────────────────────────────────

async def _control_loop() -> None:
    pid = HeadingPID(kp=PID_KP, ki=PID_KI, kd=PID_KD)
    logger.info("Control loop started")

    while True:
        await asyncio.sleep(0.05)   # 20 Hz

        nav = _latest_nav

        # ── No nav data or stale ──────────────────────────────────────────────
        if nav is None or (time.monotonic() - nav.received_at) > NAV_AGE_S:
            await _send_rpi({"type": "stop"})
            pid.reset()
            continue

        # ── Arrived at waypoint ───────────────────────────────────────────────
        if nav.distance < 2.0:
            await _send_rpi({"type": "stop"})
            pid.reset()
            continue

        # ── Need phone heading ────────────────────────────────────────────────
        if nav.device_heading is None:
            await _send_rpi({"type": "stop"})
            continue

        # ── Heading error ─────────────────────────────────────────────────────
        heading_error = _normalize_angle(nav.target_bearing - nav.device_heading)

        # RealSense gz is rad/s — convert to °/s for the PID D-term
        gyro_z_dps = _latest_imu.gz * _RAD_TO_DEG

        # ── PID ───────────────────────────────────────────────────────────────
        pid_output = pid.compute(heading_error, gyro_z_dps)

        # ── Dead zone — go straight ───────────────────────────────────────────
        if abs(heading_error) < DEAD_ZONE_DEG:
            await _send_rpi({"type": "motor", "angle": 0,
                             "velocity": BASE_TUG_SPEED, "rot": 0})
            pid.reset()
            await _broadcast_phone({
                "type": "status",
                "heading_error": round(heading_error, 1),
                "pid_output":    0,
                "connected":     True,
            })
            continue

        # ── Blend forward + lateral correction ───────────────────────────────
        # pid_output > 0  → user is left of target → tug right (angle toward 270°)
        # pid_output < 0  → user is right of target → tug left  (angle toward 90°)
        correction = min(100.0, abs(pid_output))

        if pid_output >= 0:
            blend_angle = int(360 - min(90, correction * 0.6)) % 360
        else:
            blend_angle = int(min(90, correction * 0.6))

        tug_speed = min(100, int(BASE_TUG_SPEED + correction * 0.25))

        await _send_rpi({"type": "motor", "angle": blend_angle,
                         "velocity": tug_speed, "rot": 0})

        logger.debug(
            f"error={heading_error:+.1f}°  gz={gyro_z_dps:+.1f}°/s  "
            f"pid={pid_output:+.1f}  angle={blend_angle}°  speed={tug_speed}"
        )

        await _broadcast_phone({
            "type":          "status",
            "heading_error": round(heading_error, 1),
            "pid_output":    round(pid_output, 1),
            "connected":     True,
        })


# ── Entry point ────────────────────────────────────────────────────────────────

async def main() -> None:
    logger.info(f"Phone  WebSocket → ws://0.0.0.0:{PHONE_WS_PORT}")
    logger.info(f"RPi    WebSocket → ws://0.0.0.0:{RPI_WS_PORT}")

    phone_server  = websockets.serve(_phone_handler, PHONE_WS_HOST, PHONE_WS_PORT)
    rpi_server    = websockets.serve(_rpi_handler,   RPI_WS_HOST,   RPI_WS_PORT)
    control_task  = asyncio.create_task(_control_loop())

    async with phone_server, rpi_server:
        logger.info("Waiting for phone and RPi connections…")
        try:
            await asyncio.Future()
        except (KeyboardInterrupt, asyncio.CancelledError):
            pass
        finally:
            control_task.cancel()
            await _send_rpi({"type": "stop"})
            logger.info("Shutdown complete")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
