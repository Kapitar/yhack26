"""
LeadMe — Laptop Main
Runs on the laptop. Replaces navigation.py + rpi_main.py.

Flow:
  Phone  (WebSocket :8765) ──► laptop_main.py ──► Arduino (USB Serial) ──► motors
  RealSense D457 (USB) ──────► gyro reader ──────► serial payload

Serial protocol — laptop → Arduino (USB):
  "E:<heading_error>:<gyro_z>\\n"
    heading_error : -180.0 to +180.0 degrees
    gyro_z        : yaw rate in °/s (positive = turning CCW)
  "S\\n"   → stop immediately, reset PID
  "P\\n"   → ping, Arduino responds "OK\\n"

WebSocket protocol — phone → laptop (:8765):
  {
    "type":           "nav",
    "target_bearing": 95.0,
    "device_heading": 82.0,
    "distance":       14.3,
    "lat":            40.748,
    "lng":           -73.985
  }

WebSocket protocol — laptop → phone (:8765):
  {
    "type":          "status",
    "heading_error": 13.0,
    "state":         "navigating" | "arrived" | "nav_stale" | "no_heading"
  }

Usage:
    python laptop_main.py [--port /dev/ttyUSB0]
"""

import argparse
import asyncio
import json
import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Optional

import pyrealsense2 as rs
import serial
import serial.tools.list_ports
import websockets

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(name)s  %(message)s",
)
logger = logging.getLogger("laptop_main")

# ── Configuration ──────────────────────────────────────────────────────────────

PHONE_WS_HOST = "0.0.0.0"
PHONE_WS_PORT = 8765

SERIAL_BAUD   = 115200
CONTROL_HZ    = 20

NAV_AGE_S     = 2.0    # seconds before nav packet considered stale
_RAD_TO_DEG   = 180.0 / math.pi


# ── Shared state ───────────────────────────────────────────────────────────────

@dataclass
class NavPacket:
    target_bearing: float
    device_heading: Optional[float]
    distance:       float
    lat:            float
    lng:            float
    received_at:    float = 0.0


_latest_nav: Optional[NavPacket] = None
_nav_lock = threading.Lock()

_gyro_z_dps: float = 0.0   # yaw rate in °/s — written by IMU thread
_gyro_lock = threading.Lock()

_phone_clients: set = set()


def _normalize_angle(deg: float) -> float:
    return ((deg + 180) % 360) - 180


# ── RealSense gyro reader (background thread) ──────────────────────────────────

def _start_realsense() -> rs.pipeline:
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, 200)
    pipeline.start(config)
    logger.info("RealSense gyro stream started at 200 Hz")
    return pipeline


def imu_reader(pipeline: rs.pipeline) -> None:
    """Reads gyro Z from RealSense D457 and updates _gyro_z_dps."""
    global _gyro_z_dps
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            gyro_frame = frames.first_or_default(rs.stream.gyro)
            if gyro_frame:
                g = gyro_frame.as_motion_frame().get_motion_data()
                with _gyro_lock:
                    _gyro_z_dps = g.z * _RAD_TO_DEG   # rad/s → °/s
        except Exception as exc:
            logger.warning(f"IMU read error: {exc}")


# ── Serial helpers ─────────────────────────────────────────────────────────────

def find_arduino_port() -> Optional[str]:
    keywords = {"arduino", "ch340", "ch341", "ftdi", "usbserial", "usbmodem", "acm"}
    for p in serial.tools.list_ports.comports():
        haystack = ((p.description or "") + (p.manufacturer or "") +
                    (p.device or "")).lower()
        if any(k in haystack for k in keywords):
            return p.device
    return None


def send_serial(ser: serial.Serial, cmd: str) -> None:
    try:
        ser.write(cmd.encode())
    except serial.SerialException as exc:
        logger.warning(f"Serial write failed: {exc}")


# ── Phone WebSocket server (:8765) ────────────────────────────────────────────

async def _phone_handler(websocket) -> None:
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
            packet = NavPacket(
                target_bearing=float(data["target_bearing"]),
                device_heading=(float(data["device_heading"])
                                if data.get("device_heading") is not None else None),
                distance=float(data.get("distance", 0)),
                lat=float(data.get("lat", 0)),
                lng=float(data.get("lng", 0)),
                received_at=time.monotonic(),
            )
            with _nav_lock:
                _latest_nav = packet
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


# ── Control loop (20 Hz) ──────────────────────────────────────────────────────

async def _control_loop(ser: serial.Serial) -> None:
    period = 1.0 / CONTROL_HZ
    logger.info(f"Control loop started at {CONTROL_HZ} Hz")

    while True:
        t0 = time.monotonic()

        with _nav_lock:
            nav = _latest_nav
        with _gyro_lock:
            gyro_z = _gyro_z_dps

        # ── Stale / missing nav ───────────────────────────────────────────────
        if nav is None or (time.monotonic() - nav.received_at) > NAV_AGE_S:
            send_serial(ser, "S\n")
            await _broadcast_phone({"type": "status", "state": "nav_stale",
                                    "heading_error": None})
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
            continue

        # ── Arrived ───────────────────────────────────────────────────────────
        if nav.distance is not None and nav.distance < 2.0:
            send_serial(ser, "S\n")
            await _broadcast_phone({"type": "status", "state": "arrived",
                                    "heading_error": 0.0})
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
            continue

        # ── No device heading yet ─────────────────────────────────────────────
        if nav.device_heading is None:
            send_serial(ser, "S\n")
            await _broadcast_phone({"type": "status", "state": "no_heading",
                                    "heading_error": None})
            await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))
            continue

        # ── Send heading error + gyro — Arduino runs PID ──────────────────────
        heading_error = _normalize_angle(nav.target_bearing - nav.device_heading)
        cmd = f"E:{heading_error:.1f}:{gyro_z:.2f}\n"
        send_serial(ser, cmd)

        logger.debug(
            f"target={nav.target_bearing:.1f}°  "
            f"heading={nav.device_heading:.1f}°  "
            f"error={heading_error:+.1f}°  "
            f"gyro_z={gyro_z:+.1f}°/s  dist={nav.distance:.1f}m"
        )

        await _broadcast_phone({
            "type":          "status",
            "state":         "navigating",
            "heading_error": round(heading_error, 1),
        })

        await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))


# ── Entry point ────────────────────────────────────────────────────────────────

async def _async_main(args: argparse.Namespace) -> None:
    # ── Serial ────────────────────────────────────────────────────────────────
    port = args.port or find_arduino_port()
    if not port:
        logger.error("Arduino not found. Plug it in or pass --port /dev/ttyXXX")
        return

    logger.info(f"Opening serial port {port} @ {SERIAL_BAUD} baud…")
    ser = serial.Serial(port, SERIAL_BAUD, timeout=1)
    time.sleep(2)
    send_serial(ser, "P\n")
    logger.info("Arduino connected")

    # ── RealSense gyro ────────────────────────────────────────────────────────
    pipeline = _start_realsense()
    imu_thread = threading.Thread(target=imu_reader, args=(pipeline,), daemon=True)
    imu_thread.start()

    # ── Phone WebSocket server ────────────────────────────────────────────────
    phone_server = await websockets.serve(_phone_handler, PHONE_WS_HOST, PHONE_WS_PORT)
    logger.info(f"Phone WebSocket → ws://0.0.0.0:{PHONE_WS_PORT}")

    control_task = asyncio.create_task(_control_loop(ser))

    logger.info("Ready. Waiting for phone connection…")
    try:
        await asyncio.Future()
    except (KeyboardInterrupt, asyncio.CancelledError):
        pass
    finally:
        control_task.cancel()
        send_serial(ser, "S\n")
        phone_server.close()
        pipeline.stop()
        ser.close()
        logger.info("Shutdown complete")


def main() -> None:
    parser = argparse.ArgumentParser(description="LeadMe laptop main process")
    parser.add_argument("--port", default=None,
                        help="Arduino serial port (auto-detected if omitted)")
    args = parser.parse_args()

    try:
        asyncio.run(_async_main(args))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
