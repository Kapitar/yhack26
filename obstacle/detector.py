"""
Obstacle detector — tries RealSense depth first, falls back to webcam + YOLOv8.

Returns a ZoneResult with left / center / right clearance booleans.
The frame is split into three equal vertical columns:
  left | center | right
Only the middle 50% of rows (vertically) are checked to avoid
picking up the floor or ceiling as obstacles.
"""

from typing import NamedTuple, Optional
import numpy as np

OBSTACLE_DIST_M  = 1.2   # closer than this → obstacle
FRAME_W, FRAME_H = 640, 480


class ZoneResult(NamedTuple):
    left_clear:   bool
    center_clear: bool
    right_clear:  bool
    frame: Optional[np.ndarray] = None   # annotated preview (BGR), or None


# ── RealSense depth detector ──────────────────────────────────────────────────

class RealSenseDetector:
    """Uses the D457 depth stream for reliable distance measurement."""

    def __init__(self, threshold_m: float = OBSTACLE_DIST_M):
        import pyrealsense2 as rs
        self.threshold = threshold_m
        self.pipeline  = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.pipeline.start(cfg)
        self._align = rs.align(rs.stream.color)

    def get(self) -> ZoneResult:
        import cv2
        frames        = self.pipeline.wait_for_frames()
        aligned       = self._align.process(frames)
        depth_frame   = aligned.get_depth_frame()
        color_frame   = aligned.get_color_frame()

        if not depth_frame:
            return ZoneResult(True, True, True)

        depth = np.asanyarray(depth_frame.get_data()).astype(float) \
                * depth_frame.get_units()   # → metres

        h, w  = depth.shape
        row_lo, row_hi = h // 4, 3 * h // 4   # ignore top/bottom 25%
        roi   = depth[row_lo:row_hi, :]
        roi   = np.where(roi == 0, 10.0, roi)  # 0 = no reading → treat as far

        third = w // 3
        # 10th-percentile of each zone = closest point in that zone
        left   = float(np.percentile(roi[:, :third],       10)) > self.threshold
        center = float(np.percentile(roi[:, third:2*third], 10)) > self.threshold
        right  = float(np.percentile(roi[:, 2*third:],     10)) > self.threshold

        frame = None
        if color_frame:
            frame = np.asanyarray(color_frame.get_data()).copy()
            _annotate(frame, left, center, right, third, h)

        return ZoneResult(left, center, right, frame)

    def stop(self):
        self.pipeline.stop()


# ── Webcam + YOLOv8 detector ──────────────────────────────────────────────────

class WebcamDetector:
    """
    Webcam with YOLOv8-nano object detection.
    Obstacles are objects whose bounding box is large enough to be close
    (height > 15% of frame height).
    """

    def __init__(self, threshold_m: float = OBSTACLE_DIST_M, camera_index: int = 0):
        import cv2
        from ultralytics import YOLO
        self.cap = cv2.VideoCapture(camera_index)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  FRAME_W)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_H)
        self.model = YOLO("yolov8n.pt")   # auto-downloads ~6 MB on first run
        # Suppress ultralytics console spam
        import logging
        logging.getLogger("ultralytics").setLevel(logging.WARNING)

    def get(self) -> ZoneResult:
        import cv2
        ret, frame = self.cap.read()
        if not ret:
            return ZoneResult(True, True, True)

        h, w  = frame.shape[:2]
        third = w // 3
        results = self.model(frame, verbose=False)[0]

        left_blocked = center_blocked = right_blocked = False

        for box in results.boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cx = (x1 + x2) / 2
            box_h_frac = (y2 - y1) / h
            if box_h_frac < 0.15:   # too small → too far away, skip
                continue
            if cx < third:
                left_blocked = True
            elif cx < 2 * third:
                center_blocked = True
            else:
                right_blocked = True

        annotated = results.plot()
        _annotate(annotated, not left_blocked, not center_blocked, not right_blocked, third, h)

        return ZoneResult(
            not left_blocked,
            not center_blocked,
            not right_blocked,
            annotated,
        )

    def stop(self):
        self.cap.release()


# ── Annotation helper ─────────────────────────────────────────────────────────

def _annotate(frame, left_clear, center_clear, right_clear, third, h):
    import cv2
    cv2.line(frame, (third,       0), (third,       h), (200, 200, 0), 1)
    cv2.line(frame, (2 * third,   0), (2 * third,   h), (200, 200, 0), 1)
    for i, (label, clear) in enumerate(
        zip(["L", "C", "R"], [left_clear, center_clear, right_clear])
    ):
        color = (0, 220, 0) if clear else (0, 0, 220)
        cv2.putText(frame, label, (i * third + 10, 36),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 2)


# ── Factory ───────────────────────────────────────────────────────────────────

def make_detector(prefer_realsense: bool = True, **kwargs):
    """Return the best available detector."""
    if prefer_realsense:
        try:
            d = RealSenseDetector(**kwargs)
            print("[detector] RealSense depth")
            return d
        except Exception as e:
            print(f"[detector] RealSense unavailable ({e}) — falling back to webcam+YOLO")
    d = WebcamDetector(**kwargs)
    print("[detector] webcam + YOLOv8n")
    return d
