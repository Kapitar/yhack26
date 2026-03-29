"""
RealSense depth-based obstacle detection helpers.

The runtime only needs a conservative "is something directly ahead within the
stop distance?" signal. This module samples a centered region of interest from
the depth frame and uses a low percentile distance instead of a raw minimum so
single noisy pixels do not trigger false stops.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional


@dataclass(frozen=True)
class ObstacleSnapshot:
    distance_m: Optional[float]
    blocked: bool
    timestamp: float

    @property
    def is_fresh(self) -> bool:
        return self.timestamp > 0.0


class RealSenseObstacleMonitor:
    """Tracks whether a forward obstacle is inside the stop distance."""

    def __init__(
        self,
        stop_distance_m: float = 2.0,
        min_valid_distance_m: float = 0.15,
        max_valid_distance_m: float = 6.0,
        roi_width_ratio: float = 0.35,
        roi_height_ratio: float = 0.45,
        grid_samples_x: int = 12,
        grid_samples_y: int = 10,
        percentile: float = 0.2,
    ) -> None:
        self._stop_distance_m = stop_distance_m
        self._min_valid_distance_m = min_valid_distance_m
        self._max_valid_distance_m = max_valid_distance_m
        self._roi_width_ratio = roi_width_ratio
        self._roi_height_ratio = roi_height_ratio
        self._grid_samples_x = max(2, grid_samples_x)
        self._grid_samples_y = max(2, grid_samples_y)
        self._percentile = min(max(percentile, 0.0), 1.0)
        self._lock = threading.Lock()
        self._snapshot = ObstacleSnapshot(distance_m=None, blocked=False, timestamp=0.0)

    def update_from_depth_frame(self, depth_frame) -> None:
        width = depth_frame.get_width()
        height = depth_frame.get_height()

        roi_half_w = max(1, int(width * self._roi_width_ratio * 0.5))
        roi_half_h = max(1, int(height * self._roi_height_ratio * 0.5))
        cx = width // 2
        cy = height // 2

        left = max(0, cx - roi_half_w)
        right = min(width - 1, cx + roi_half_w)
        top = max(0, cy - roi_half_h)
        bottom = min(height - 1, cy + roi_half_h)

        step_x = max(1, (right - left) // (self._grid_samples_x - 1))
        step_y = max(1, (bottom - top) // (self._grid_samples_y - 1))

        distances: list[float] = []
        for y in range(top, bottom + 1, step_y):
            for x in range(left, right + 1, step_x):
                distance_m = depth_frame.get_distance(x, y)
                if self._min_valid_distance_m <= distance_m <= self._max_valid_distance_m:
                    distances.append(distance_m)

        if distances:
            distances.sort()
            idx = min(len(distances) - 1, int(len(distances) * self._percentile))
            obstacle_distance_m = distances[idx]
            blocked = obstacle_distance_m <= self._stop_distance_m
        else:
            obstacle_distance_m = None
            blocked = False

        with self._lock:
            self._snapshot = ObstacleSnapshot(
                distance_m=obstacle_distance_m,
                blocked=blocked,
                timestamp=time.monotonic(),
            )

    def snapshot(self) -> ObstacleSnapshot:
        with self._lock:
            return self._snapshot
