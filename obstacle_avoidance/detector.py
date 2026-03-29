"""
Camera-based obstacle detector.

Splits the camera frame into 3 vertical zones (left / center / right)
and reports whether each zone is clear or blocked.

Two backends:
  - RealSense D457: uses depth data directly — most reliable.
  - Webcam + YOLOv8n: detects objects and checks their size as a
    rough distance proxy (large box = close = obstacle).
"""

import numpy as np
from dataclasses import dataclass
from typing import Optional

OBSTACLE_DIST_M   = 1.2    # RealSense: closer than this = obstacle
MIN_BOX_HEIGHT    = 0.18   # YOLO: bounding box height > 18% of frame = close enough to block


@dataclass
class Zones:
    left:   bool   # True = clear, False = blocked
    center: bool
    right:  bool
    frame:  Optional[np.ndarray] = None   # annotated BGR frame for preview


# ── RealSense depth backend ───────────────────────────────────────────────────

class RealSenseDetector:
    def __init__(self, threshold_m: float = OBSTACLE_DIST_M):
        import pyrealsense2 as rs
        self.threshold = threshold_m
        pipe = rs.pipeline()
        cfg  = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16,  30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        pipe.start(cfg)
        self._pipe  = pipe
        self._align = rs.align(rs.stream.color)

    def read(self) -> Zones:
        import cv2
        frames = self._pipe.wait_for_frames()
        frames = self._align.process(frames)
        d = frames.get_depth_frame()
        c = frames.get_color_frame()
        if not d:
            return Zones(True, True, True)

        depth = np.asanyarray(d.get_data()).astype(np.float32) * d.get_units()
        h, w  = depth.shape
        # Only look at the middle vertical band (ignore floor / ceiling)
        roi   = depth[h // 4 : 3 * h // 4, :]
        roi   = np.where(roi == 0, 99.0, roi)   # 0 = no reading → treat as far
        t     = w // 3
        left   = float(np.percentile(roi[:, :t],    10)) > self.threshold
        center = float(np.percentile(roi[:, t:2*t], 10)) > self.threshold
        right  = float(np.percentile(roi[:, 2*t:],  10)) > self.threshold

        frame = None
        if c:
            frame = np.asanyarray(c.get_data()).copy()
            _draw_zones(frame, left, center, right)
        return Zones(left, center, right, frame)

    def close(self):
        self._pipe.stop()


# ── Webcam + YOLOv8n backend ──────────────────────────────────────────────────

class WebcamDetector:
    def __init__(self, camera: int = 0):
        import cv2
        from ultralytics import YOLO
        import logging
        logging.getLogger("ultralytics").setLevel(logging.WARNING)
        self._cap   = cv2.VideoCapture(camera)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._model = YOLO("yolov8n.pt")   # ~6 MB, auto-downloads on first run

    def read(self) -> Zones:
        ret, frame = self._cap.read()
        if not ret:
            return Zones(True, True, True)

        h, w    = frame.shape[:2]
        t       = w // 3
        results = self._model(frame, verbose=False)[0]

        left_blocked = center_blocked = right_blocked = False

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx             = (x1 + x2) / 2
            box_h_frac     = (y2 - y1) / h
            if box_h_frac < MIN_BOX_HEIGHT:
                continue   # too small → too far away
            if   cx < t:     left_blocked   = True
            elif cx < 2 * t: center_blocked = True
            else:            right_blocked  = True

        annotated = results.plot()
        _draw_zones(annotated, not left_blocked, not center_blocked, not right_blocked)
        return Zones(not left_blocked, not center_blocked, not right_blocked, annotated)

    def close(self):
        self._cap.release()


# ── Zone annotation ───────────────────────────────────────────────────────────

def _draw_zones(frame: np.ndarray, left: bool, center: bool, right: bool):
    import cv2
    h, w = frame.shape[:2]
    t    = w // 3
    cv2.line(frame, (t,     0), (t,     h), (200, 200, 0), 1)
    cv2.line(frame, (2 * t, 0), (2 * t, h), (200, 200, 0), 1)
    for i, (label, clear) in enumerate(zip("LCR", [left, center, right])):
        color = (0, 210, 0) if clear else (0, 0, 210)
        cv2.putText(frame, label, (i * t + 10, 38),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 2)


# ── Factory ───────────────────────────────────────────────────────────────────

def make_detector(use_webcam: bool = False, camera: int = 0) -> RealSenseDetector | WebcamDetector:
    if not use_webcam:
        try:
            d = RealSenseDetector()
            print("[detector] RealSense depth")
            return d
        except Exception as e:
            print(f"[detector] RealSense unavailable ({e}), falling back to webcam + YOLOv8n")
    d = WebcamDetector(camera)
    print("[detector] webcam + YOLOv8n")
    return d
