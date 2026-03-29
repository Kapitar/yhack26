"""
LeadMe — Obstacle Avoidance (2-wheel differential drive)
Runs YOLO object detection on the RealSense camera.
When an obstacle is in the forward path, the robot arcs around it
(forward + rotation) so overall forward progress is maintained.

Motor layout (front two wheels only):
    [0 LEFT] ─── [1 RIGHT]

Serial protocol to main.cpp:
    "angle|velocity|rot\\n"
        angle    : 0 = forward, 180 = backward
        velocity : 0–100
        rot      : positive = arc left, negative = arc right

Usage:
    python avoid.py                      # auto-detect Arduino port
    python avoid.py --port COM3          # Windows
    python avoid.py --port /dev/ttyACM0  # Linux/RPi
    python avoid.py --no-display         # headless

Requires:
    pip install pyrealsense2 ultralytics opencv-python pyserial
"""

import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs
import serial
import serial.tools.list_ports
from ultralytics import YOLO

# ── Tuning ────────────────────────────────────────────────────────────────────

STOP_DISTANCE_M  = 2.0   # obstacle closer than this triggers avoidance
DRIVE_SPEED      = 70    # forward velocity (0–100)
ARC_SPEED        = 80    # velocity while arcing (higher to compensate for rot halving it)
ARC_ROT          = 70    # rotation magnitude while arcing (0–100); higher = tighter arc
BACK_SPEED       = 45    # reverse speed when fully blocked
BACK_DURATION_S  = 0.5   # seconds to reverse before re-evaluating

# Horizontal zones as fraction of frame width
LEFT_END    = 0.25
RIGHT_START = 0.75

CONF_THRESH      = 0.45   # minimum YOLO confidence
OBSTACLE_CLASSES = None   # None = all classes; e.g. {"person", "chair"}

BAUD = 9600


# ── Arduino serial ────────────────────────────────────────────────────────────

def find_arduino() -> str | None:
    for p in serial.tools.list_ports.comports():
        desc = (p.description or "").lower()
        mfr  = (p.manufacturer or "").lower()
        if any(k in desc or k in mfr for k in
               ("arduino", "ch340", "ch341", "ftdi", "usb serial")):
            return p.device
    for p in serial.tools.list_ports.comports():
        if "ttyUSB" in p.device or "ttyACM" in p.device:
            return p.device
    return None


class Robot:
    def __init__(self, port: str | None):
        port = port or find_arduino()
        if port is None:
            raise RuntimeError("Arduino not found. Connect USB or pass --port.")
        self._ser = serial.Serial(port=port, baudrate=BAUD, timeout=1.0)
        time.sleep(2.0)
        self._ser.reset_input_buffer()
        print(f"[robot] connected on {port}")

    def _send(self, angle: int, velocity: int, rot: int):
        line = f"{angle % 360}|{max(0, min(100, velocity))}|{max(-100, min(100, rot))}\n"
        self._ser.write(line.encode("ascii"))

    def forward(self):
        """Drive straight ahead."""
        self._send(0, DRIVE_SPEED, 0)

    def arc_left(self):
        """Drive forward while curving left — avoids obstacle on the right side."""
        self._send(0, ARC_SPEED, ARC_ROT)

    def arc_right(self):
        """Drive forward while curving right — avoids obstacle on the left side."""
        self._send(0, ARC_SPEED, -ARC_ROT)

    def backward(self):
        self._send(180, BACK_SPEED, 0)

    def stop(self):
        self._ser.write(b"S\n")

    def close(self):
        self.stop()
        self._ser.close()


# ── Depth zone helper ─────────────────────────────────────────────────────────

def zone_distance(depth_frame, x0r: float, x1r: float,
                  y0r: float = 0.25, y1r: float = 0.75) -> float:
    """10th-percentile distance (m) in a zone. Returns inf if no valid readings."""
    w = depth_frame.get_width()
    h = depth_frame.get_height()
    xs = np.linspace(int(w * x0r), int(w * x1r), 7, dtype=int)
    ys = np.linspace(int(h * y0r), int(h * y1r), 7, dtype=int)
    # clamp so coordinates are always strictly inside the frame
    xs = np.clip(xs, 0, w - 1)
    ys = np.clip(ys, 0, h - 1)
    dists = [
        depth_frame.get_distance(int(x), int(y))
        for y in ys for x in xs
        if 0.6 <= depth_frame.get_distance(int(x), int(y)) <= 8.0
    ]
    if not dists:
        return float("inf")
    dists.sort()
    return dists[max(0, int(len(dists) * 0.10))]


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port",       default=None)
    parser.add_argument("--no-display", action="store_true")
    parser.add_argument("--model",      default="yolov8n.pt")
    args = parser.parse_args()

    model  = YOLO(args.model)
    robot  = Robot(args.port)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
    cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  15)
    pipeline.start(cfg)
    align = rs.align(rs.stream.color)

    print("[avoid] running — press Q to quit")

    # Track backing manoeuvre so we don't interrupt it mid-reverse
    backing_until = 0.0

    try:
        while True:
            frames  = pipeline.wait_for_frames()
            aligned = align.process(frames)
            color_f = aligned.get_color_frame()
            depth_f = aligned.get_depth_frame()
            if not color_f or not depth_f:
                continue

            frame = np.asanyarray(color_f.get_data())
            h, w  = frame.shape[:2]

            # ── YOLO ─────────────────────────────────────────────────────────
            results        = model(frame, verbose=False)
            obstacle_ahead = False

            for box in results[0].boxes:
                conf = float(box.conf[0])
                if conf < CONF_THRESH:
                    continue
                label = model.names[int(box.cls[0])]
                if OBSTACLE_CLASSES and label not in OBSTACLE_CLASSES:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx, cy   = (x1 + x2) // 2, (y1 + y2) // 2
                cx_ratio = cx / w
                distance = depth_f.get_distance(cx, cy)
                in_center = LEFT_END <= cx_ratio <= RIGHT_START
                is_close  = 0.6 < distance < STOP_DISTANCE_M

                color = (0, 0, 255) if (in_center and is_close) else (0, 200, 0)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame,
                            f"{label} {conf:.2f} {distance:.2f}m",
                            (x1, max(y1 - 8, 0)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

                if in_center and is_close:
                    obstacle_ahead = True

            # ── Avoidance decision ────────────────────────────────────────────
            now    = time.monotonic()
            action = "FORWARD"

            if now < backing_until:
                # Finish reverse manoeuvre before doing anything else
                robot.backward()
                action = "BACK UP"

            elif obstacle_ahead:
                left_dist  = zone_distance(depth_f, 0.0,        LEFT_END)
                right_dist = zone_distance(depth_f, RIGHT_START, 1.0)

                if left_dist >= right_dist:
                    # More space on the left → arc left around the obstacle
                    robot.arc_left()
                    action = f"ARC LEFT  (L:{left_dist:.1f}m R:{right_dist:.1f}m)"
                elif right_dist > left_dist:
                    # More space on the right → arc right
                    robot.arc_right()
                    action = f"ARC RIGHT (L:{left_dist:.1f}m R:{right_dist:.1f}m)"
                else:
                    # Fully blocked — reverse briefly then try again
                    robot.backward()
                    backing_until = now + BACK_DURATION_S
                    action = "BACK UP"

            else:
                robot.forward()

            # ── Display ───────────────────────────────────────────────────────
            if not args.no_display:
                cv2.line(frame, (int(w * LEFT_END),    0),
                                (int(w * LEFT_END),    h), (200, 200, 0), 1)
                cv2.line(frame, (int(w * RIGHT_START), 0),
                                (int(w * RIGHT_START), h), (200, 200, 0), 1)

                color_map = {"FORWARD": (0, 255, 0)}
                txt_color = (0, 165, 255) if "ARC" in action else \
                            (0, 0, 255)   if "BACK" in action else \
                            (0, 255, 0)
                cv2.putText(frame, action, (10, h - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, txt_color, 2)

                cv2.imshow("LeadMe Avoid", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        pass
    finally:
        robot.close()
        pipeline.stop()
        cv2.destroyAllWindows()
        print("[avoid] stopped")


if __name__ == "__main__":
    main()
