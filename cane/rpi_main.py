"""
LeadMe — Raspberry Pi Bridge
Runs on the Raspberry Pi attached to the cane.

  RealSense D457 IMU ──► RPi ──► WebSocket ──► Laptop (navigation.py)
  Laptop (navigation.py) ──► WebSocket ──► RPi ──► USB Serial ──► Arduino ──► Motors

Usage:
    python rpi_main.py                        # auto-detect Arduino port
    python rpi_main.py --laptop 192.168.1.42  # specify laptop IP
    python rpi_main.py --port /dev/ttyACM0    # specify Arduino port
"""

import argparse
import asyncio
import json
import logging
import threading
import time

import pyrealsense2 as rs
import serial
import serial.tools.list_ports
import websockets

from realsense_obstacle import RealSenseObstacleMonitor

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s  %(levelname)-7s  %(name)s  %(message)s",
)
logger = logging.getLogger("rpi")

# ── Configuration ──────────────────────────────────────────────────────────────

LAPTOP_WS_PORT = 8766
BAUD_RATE      = 9600
IMU_HZ         = 50     # rate at which IMU data is sent to laptop
OBSTACLE_STOP_M = 2.0
DEPTH_STALE_S = 1.0

# ── Shared IMU state (written by reader thread, read by async loop) ────────────

_imu = {"gx": 0.0, "gy": 0.0, "gz": 0.0, "ax": 0.0, "ay": 0.0, "az": 0.0}
_imu_lock = threading.Lock()
_obstacle_monitor = RealSenseObstacleMonitor(stop_distance_m=OBSTACLE_STOP_M)


# ── RealSense IMU reader (background thread) ───────────────────────────────────

def _imu_reader(pipeline: rs.pipeline) -> None:
    """
    Reads gyro + accel + depth frames from the RealSense D457 continuously.
    Stores latest IMU values in the shared _imu dict and updates obstacle state.
    """
    while True:
        try:
            frames = pipeline.wait_for_frames(timeout_ms=1000)

            gyro_frame  = frames.first_or_default(rs.stream.gyro)
            accel_frame = frames.first_or_default(rs.stream.accel)
            depth_frame = frames.first_or_default(rs.stream.depth)

            with _imu_lock:
                if gyro_frame:
                    g = gyro_frame.as_motion_frame().get_motion_data()
                    _imu["gx"] = g.x   # rad/s — convert to °/s on laptop
                    _imu["gy"] = g.y
                    _imu["gz"] = g.z

                if accel_frame:
                    a = accel_frame.as_motion_frame().get_motion_data()
                    _imu["ax"] = a.x   # m/s²
                    _imu["ay"] = a.y
                    _imu["az"] = a.z

            if depth_frame:
                _obstacle_monitor.update_from_depth_frame(depth_frame)

        except Exception as exc:
            logger.warning(f"IMU read error: {exc}")


def _start_realsense() -> rs.pipeline:
    pipeline = rs.pipeline()
    config   = rs.config()
    config.enable_stream(rs.stream.gyro,  rs.format.motion_xyz32f, 200)
    config.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, 100)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)
    pipeline.start(config)
    logger.info("RealSense D457 IMU + depth streams started")
    return pipeline


# ── Arduino serial connection ──────────────────────────────────────────────────

def _find_arduino_port() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if any(kw in desc or kw in mfr
               for kw in ("arduino", "ch340", "ch341", "ftdi", "usb serial")):
            logger.info(f"Auto-detected Arduino on {p.device}")
            return p.device
    # Fallback: first ttyUSB / ttyACM
    for p in serial.tools.list_ports.comports():
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            logger.info(f"Fallback Arduino port: {p.device}")
            return p.device
    return None


def _open_arduino(port: str | None) -> serial.Serial:
    port = port or _find_arduino_port()
    if port is None:
        raise RuntimeError(
            "Arduino not found. Check USB connection or pass --port explicitly."
        )
    ser = serial.Serial(port=port, baudrate=BAUD_RATE, timeout=1.0)
    time.sleep(2.0)           # wait for Arduino bootloader reset
    ser.reset_input_buffer()
    logger.info(f"Arduino connected on {port} at {BAUD_RATE} baud")
    return ser


# ── WebSocket loop ─────────────────────────────────────────────────────────────

async def _send_imu(ws: websockets.WebSocketClientProtocol) -> None:
    """Continuously send IMU readings to the laptop."""
    interval = 1.0 / IMU_HZ
    while True:
        with _imu_lock:
            payload = dict(_imu)
        payload["type"] = "imu"
        await ws.send(json.dumps(payload))
        await asyncio.sleep(interval)


async def _recv_commands(
    ws:      websockets.WebSocketClientProtocol,
    arduino: serial.Serial,
) -> None:
    """Receive motor commands from laptop and forward to Arduino via Serial."""
    async for raw in ws:
        try:
            data = json.loads(raw)
        except json.JSONDecodeError:
            continue

        t = data.get("type")
        if t == "motor":
            angle    = int(data.get("angle",    0)) % 360
            velocity = max(0,    min(100, int(data.get("velocity", 0))))
            rot      = max(-100, min(100, int(data.get("rot",      0))))
            obstacle = _obstacle_monitor.snapshot()
            obstacle_stale = (not obstacle.is_fresh) or (
                (time.monotonic() - obstacle.timestamp) > DEPTH_STALE_S
            )
            if (obstacle_stale or obstacle.blocked) and (velocity > 0 or rot != 0):
                arduino.write(b"S\n")
                continue
            cmd      = f"{angle}|{velocity}|{rot}\n".encode("ascii")
            arduino.write(cmd)

        elif t == "stop":
            arduino.write(b"S\n")


async def _run(laptop_ip: str, arduino: serial.Serial) -> None:
    """Main loop: connect to laptop, run IMU sender + command receiver."""
    url = f"ws://{laptop_ip}:{LAPTOP_WS_PORT}"

    while True:
        try:
            async with websockets.connect(url, ping_interval=5, ping_timeout=10) as ws:
                logger.info(f"Connected to laptop at {url}")
                await asyncio.gather(
                    _send_imu(ws),
                    _recv_commands(ws, arduino),
                )

        except (websockets.exceptions.ConnectionClosed, OSError) as exc:
            logger.warning(f"Laptop WebSocket lost ({exc}). Safety stop. Retrying in 2 s…")
            arduino.write(b"S\n")
            await asyncio.sleep(2.0)


# ── Entry point ────────────────────────────────────────────────────────────────

async def _main(laptop_ip: str, arduino_port: str | None) -> None:
    # Start RealSense
    pipeline = _start_realsense()
    imu_thread = threading.Thread(
        target=_imu_reader, args=(pipeline,), daemon=True
    )
    imu_thread.start()

    # Open Arduino
    arduino = _open_arduino(arduino_port)

    logger.info(f"Connecting to laptop WebSocket at ws://{laptop_ip}:{LAPTOP_WS_PORT}")
    await _run(laptop_ip, arduino)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="LeadMe RPi bridge")
    parser.add_argument(
        "--laptop", default="192.168.1.100",
        help="Laptop IP address (default: 192.168.1.100)"
    )
    parser.add_argument(
        "--port", default=None,
        help="Arduino serial port (default: auto-detect)"
    )
    args = parser.parse_args()

    try:
        asyncio.run(_main(args.laptop, args.port))
    except KeyboardInterrupt:
        pass
