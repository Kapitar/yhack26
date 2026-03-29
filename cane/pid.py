"""
PID controller for LeadMe heading correction.

Input:  heading_error (degrees, –180 to +180)
        gyro_z        (°/s from MPU6050, direct measurement of turn rate)
Output: correction    (–100 to +100, passed straight to driver.tug())

Using gyro_z directly for the D term instead of differentiating the error
gives much smoother behaviour — no derivative kick on sudden setpoint changes.
"""

import time


def _normalize_angle(deg: float) -> float:
    """Wrap angle to (–180, +180]."""
    return ((deg + 180) % 360) - 180


class HeadingPID:
    def __init__(
        self,
        kp: float = 1.2,
        ki: float = 0.04,
        kd: float = 0.25,
        output_limit: float = 100.0,
        integral_limit: float = 40.0,
    ):
        """
        kp             — proportional gain
        ki             — integral gain
        kd             — derivative gain (applied to gyro_z, not d(error)/dt)
        output_limit   — clamp output to ±this value
        integral_limit — anti-windup: clamp integral accumulator to ±this value
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit   = output_limit
        self.integral_limit = integral_limit

        self._integral   = 0.0
        self._last_time  = None

    def compute(self, heading_error_deg: float, gyro_z_dps: float) -> float:
        """
        heading_error_deg : target_bearing − device_heading, normalised to (–180, 180]
                            positive → user is pointing left of target (needs rightward tug)
                            negative → user is pointing right of target (needs leftward tug)
        gyro_z_dps        : yaw rate from MPU6050 in °/s
                            positive = turning CCW (leftward), negative = turning CW

        Returns correction in (–100, +100):
            positive → apply rightward tug
            negative → apply leftward tug
        """
        now = time.monotonic()
        dt  = (now - self._last_time) if self._last_time else 0.05
        self._last_time = now
        dt = max(0.001, min(dt, 0.5))   # clamp dt to sane range

        error = _normalize_angle(heading_error_deg)

        # Proportional
        p = self.kp * error

        # Integral with anti-windup
        self._integral += error * dt
        self._integral  = max(-self.integral_limit,
                              min(self.integral_limit, self._integral))
        i = self.ki * self._integral

        # Derivative — use gyro directly (negative because CCW gyro opposes error growth)
        d = -self.kd * gyro_z_dps

        output = p + i + d
        return max(-self.output_limit, min(self.output_limit, output))

    def reset(self) -> None:
        self._integral  = 0.0
        self._last_time = None
