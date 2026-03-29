"""
LeadMe — Obstacle Avoidance
Detects obstacles via camera and steers the cane via Bluetooth.

Usage:
    python -m obstacle.main                        # auto-scan for BLE device
    python -m obstacle.main --device "Hiwonder"   # specify BLE name
    python -m obstacle.main --cam                  # force webcam (skip RealSense)
    python -m obstacle.main --cam --show           # show camera preview

Requirements:
    pip install bleak opencv-python ultralytics
    RealSense SDK optional (pyrealsense2) — auto-falls back to webcam if missing.
"""

import argparse
import asyncio
import sys
import time

from bleak import BleakClient, BleakScanner

from .detector import make_detector
from .avoid    import zone_to_cmd

# HM-10 BLE UART characteristic
CHAR_UUID   = "0000FFE1-0000-1000-8000-00805F9B34FB"
DEVICE_NAME = "Hiwonder"
LOOP_HZ     = 15


async def find_write_handle(client: BleakClient) -> int:
    """Pick the writable FFE1 characteristic handle."""
    for service in client.services:
        for c in service.characteristics:
            if c.uuid.lower() == CHAR_UUID.lower():
                if "write-without-response" in c.properties or "write" in c.properties:
                    return c.handle
    raise RuntimeError("No writable FFE1 characteristic found — check firmware is flashed.")


async def ble_send(client: BleakClient, handle: int, cmd: str) -> None:
    data = (b"S\n" if cmd == "S" else f"{cmd}\n".encode("ascii"))
    try:
        await client.write_gatt_char(handle, data, response=False)
    except Exception as e:
        print(f"[ble] write failed: {e}")


async def run(device_name: str, prefer_realsense: bool, show: bool) -> None:
    import cv2

    # ── Connect BLE ───────────────────────────────────────────────────────────
    print(f"[ble] scanning for '{device_name}'...")
    device = await BleakScanner.find_device_by_name(device_name, timeout=10.0)
    if device is None:
        print(f"[ble] ERROR: '{device_name}' not found.")
        print("      Make sure the robot is powered on and in range.")
        print("      Run:  python -m obstacle.scan  to list nearby BLE devices.")
        sys.exit(1)

    async with BleakClient(device) as client:
        handle = await find_write_handle(client)
        print(f"[ble] connected to {device.name} ({device.address}), handle={handle}")

        # ── Start detector ────────────────────────────────────────────────────
        detector = make_detector(prefer_realsense=prefer_realsense)
        period   = 1.0 / LOOP_HZ
        print(f"[obstacle] running at {LOOP_HZ} Hz — press Ctrl+C to quit")
        if show:
            print("[obstacle] press Q in the preview window to quit")

        try:
            while True:
                t0 = time.monotonic()

                zones = detector.get()
                cmd   = zone_to_cmd(zones)

                await ble_send(client, handle, cmd)

                l = "OK" if zones.left_clear   else "!!"
                c = "OK" if zones.center_clear else "!!"
                r = "OK" if zones.right_clear  else "!!"
                print(f"  L={l}  C={c}  R={r}  →  {cmd}")

                if show and zones.frame is not None:
                    cv2.imshow("LeadMe Obstacle", zones.frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                elapsed = time.monotonic() - t0
                sleep   = period - elapsed
                if sleep > 0:
                    await asyncio.sleep(sleep)

        except KeyboardInterrupt:
            pass
        finally:
            await ble_send(client, handle, "S")
            detector.stop()
            if show:
                cv2.destroyAllWindows()
            print("\n[obstacle] stopped — motors off")


def main() -> None:
    parser = argparse.ArgumentParser(description="LeadMe obstacle avoidance")
    parser.add_argument("--device", default=DEVICE_NAME, help=f"BLE device name (default: {DEVICE_NAME})")
    parser.add_argument("--cam",  action="store_true", help="Force webcam (skip RealSense)")
    parser.add_argument("--show", action="store_true", help="Show camera preview window")
    args = parser.parse_args()

    try:
        asyncio.run(run(
            device_name      = args.device,
            prefer_realsense = not args.cam,
            show             = args.show,
        ))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
