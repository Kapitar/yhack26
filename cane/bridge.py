"""
LeadMe — Simple Laptop Bridge
Phone (WebSocket :8765) → Laptop → BLE → Arduino → Motors

Usage:
    python bridge.py                       # scan for default BLE device
    python bridge.py --device "Hiwonder"  # specify BLE name

Requirements:
    pip install bleak websockets
"""

import asyncio
import json
import sys
import time
import socket

import websockets
from bleak import BleakClient, BleakScanner

WS_PORT     = 8765
CHAR_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"
DEVICE_NAME = "Hiwonder"
CONTROL_HZ  = 20
NAV_STALE_S = 2.0

_latest_nav = None
_nav_lock   = asyncio.Lock()


def normalize_angle(deg):
    return ((deg + 180) % 360) - 180


def heading_error_to_cmd(error_deg):
    """
    Map heading error → angle|velocity|rot for 2-wheel differential drive.
      error > 0: need to go right → forward + CW rotation  (rot < 0)
      error < 0: need to go left  → forward + CCW rotation (rot > 0)
      error ≈ 0: straight forward
    Angle is always 0 (forward) — no strafing on 2 wheels.
    """
    error    = max(-90.0, min(90.0, error_deg))
    velocity = min(100, 45 + int(abs(error) * 0.3))
    rot      = int(-error * 0.8)   # scale error → rotation, flip sign
    rot      = max(-80, min(80, rot))
    return f"0|{velocity}|{rot}"


async def find_write_handle(client: BleakClient) -> int:
    for service in client.services:
        for c in service.characteristics:
            if c.uuid.lower() == CHAR_UUID.lower():
                if "write-without-response" in c.properties or "write" in c.properties:
                    return c.handle
    raise RuntimeError("No writable FFE1 characteristic — check firmware is flashed.")


async def ble_send(client: BleakClient, handle: int, cmd: str) -> None:
    data = (b"S\n" if cmd == "S" else f"{cmd}\n".encode("ascii"))
    try:
        await client.write_gatt_char(handle, data, response=False)
    except Exception as e:
        print(f"[ble] write failed: {e}")


async def phone_handler(websocket):
    global _latest_nav
    print(f"[ws] phone connected from {websocket.remote_address}")
    try:
        async for raw in websocket:
            try:
                data = json.loads(raw)
            except Exception:
                continue
            if data.get("type") == "nav":
                async with _nav_lock:
                    _latest_nav = {**data, "ts": time.monotonic()}
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        print("[ws] phone disconnected")


async def control_loop(client: BleakClient, handle: int):
    period = 1.0 / CONTROL_HZ
    print(f"[ctrl] running at {CONTROL_HZ} Hz")

    while True:
        t0 = time.monotonic()

        async with _nav_lock:
            nav = _latest_nav

        if nav is None or (time.monotonic() - nav["ts"]) > NAV_STALE_S:
            await ble_send(client, handle, "S")

        elif nav.get("distance", 99) < 2.0:
            await ble_send(client, handle, "S")
            print("[ctrl] arrived")

        else:
            target  = nav.get("target_bearing")
            heading = nav.get("device_heading")

            if target is None:
                await ble_send(client, handle, "S")
            elif heading is None:
                await ble_send(client, handle, "0|55|0")
                print("[ctrl] no heading — going forward")
            else:
                error = normalize_angle(target - heading)
                cmd   = heading_error_to_cmd(error)
                await ble_send(client, handle, cmd)
                print(f"[ctrl] error={error:+.1f}° → {cmd}")

        await asyncio.sleep(max(0.0, period - (time.monotonic() - t0)))


async def run(device_name: str):
    # ── BLE connect ───────────────────────────────────────────────────────────
    print(f"[ble] scanning for '{device_name}'...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
    if device is None:
        print(f"[ble] ERROR: '{device_name}' not found.")
        print("      Power on the robot and make sure it's in range.")
        sys.exit(1)

    async with BleakClient(device) as client:
        handle = await find_write_handle(client)
        print(f"[ble] connected to {device.name} ({device.address})")

        # ── WebSocket server ──────────────────────────────────────────────────
        server   = await websockets.serve(phone_handler, "0.0.0.0", WS_PORT)
        local_ip = socket.gethostbyname(socket.gethostname())
        print(f"\n[ws] server on ws://0.0.0.0:{WS_PORT}")
        print(f"[ws] set phone WS URL to:  ws://{local_ip}:{WS_PORT}")
        print("Waiting for phone...\n")

        await control_loop(client, handle)


def main():
    import argparse
    parser = argparse.ArgumentParser(description="LeadMe laptop bridge")
    parser.add_argument("--device", default=DEVICE_NAME,
                        help=f"BLE device name (default: {DEVICE_NAME})")
    args = parser.parse_args()

    try:
        asyncio.run(run(args.device))
    except KeyboardInterrupt:
        print("\nShutdown.")


if __name__ == "__main__":
    main()
