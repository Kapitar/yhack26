"""
LeadMe — Obstacle Avoidance
Reads camera, detects obstacles, steers the miniAuto around them.

Usage:
    python -m obstacle_avoidance.main --port COM5
    python -m obstacle_avoidance.main --port COM5 --webcam --show

    --port   COM port the miniAuto is paired to (see Device Manager)
    --webcam force webcam instead of RealSense
    --show   open a live camera preview window
    --speed  drive speed 0-100 (default 60)

Requirements:
    pip install pyserial opencv-python ultralytics
    Optional: pyrealsense2 (for RealSense depth camera)

How to find your COM port (Windows):
    Pair the miniAuto in Bluetooth settings, then open
    Device Manager → Ports (COM & LPT) — look for "Bluetooth Serial Port"
"""

import argparse
import sys
import time

from .robot    import MiniAuto
from .detector import make_detector, Zones

LOOP_HZ = 15


def decide(zones: Zones) -> str:
    """
    Return an action string based on which zones are clear.
    All actions use forward + rotation (differential drive — no strafing).
    """
    if zones.center:
        return "forward"
    if zones.left and not zones.right:
        return "turn_left"
    if zones.right and not zones.left:
        return "turn_right"
    if zones.left and zones.right:
        return "turn_right"   # both open, pick a side
    return "stop"             # all blocked


def run(port: str, use_webcam: bool, show: bool, speed: int):
    import cv2

    detector = make_detector(use_webcam=use_webcam)
    period   = 1.0 / LOOP_HZ

    with MiniAuto(port=port, speed=speed) as robot:
        print(f"[main] running at {LOOP_HZ} Hz  —  Ctrl+C to quit")
        try:
            while True:
                t0 = time.monotonic()

                zones  = detector.read()
                action = decide(zones)

                if   action == "forward":    robot.forward()
                elif action == "turn_left":  robot.turn_left()
                elif action == "turn_right": robot.turn_right()
                else:                        robot.stop()

                l = "OK" if zones.left   else "!!"
                c = "OK" if zones.center else "!!"
                r = "OK" if zones.right  else "!!"
                print(f"  L={l} C={c} R={r}  → {action}")

                if show and zones.frame is not None:
                    cv2.imshow("LeadMe Obstacle Avoidance", zones.frame)
                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

                elapsed = time.monotonic() - t0
                remaining = period - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            pass
        finally:
            detector.close()
            if show:
                cv2.destroyAllWindows()
            print("[main] stopped")


def main():
    parser = argparse.ArgumentParser(description="LeadMe obstacle avoidance")
    parser.add_argument("--port",   default=None,  help="COM port (e.g. COM5)")
    parser.add_argument("--webcam", action="store_true", help="Force webcam (skip RealSense)")
    parser.add_argument("--show",   action="store_true", help="Show camera preview")
    parser.add_argument("--speed",  type=int, default=60, help="Drive speed 0-100 (default 60)")
    args = parser.parse_args()

    try:
        run(
            port       = args.port,
            use_webcam = args.webcam,
            show       = args.show,
            speed      = args.speed,
        )
    except RuntimeError as e:
        print(f"\nERROR: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
