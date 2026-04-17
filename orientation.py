"""Tip orientation estimator from an MPU-6500 (accel + gyro, no magnetometer).

- Pitch and roll are derived from gravity direction via the accelerometer.
- Yaw is integrated from the gyroscope; it drifts over time (no mag reference).
- `capture_zero()` snapshots the current orientation as the reference so that
  later reads are returned as *deltas* from the zero pose.
"""
from __future__ import annotations
import math
import time
from typing import Optional


class OrientationEstimator:
    def __init__(self) -> None:
        self.pitch: float = 0.0   # radians, around body Y
        self.roll: float = 0.0    # radians, around body X
        self.yaw: float = 0.0     # radians, around body Z (integrated gyro)
        self._last_gyro_t: Optional[float] = None
        # Zero offsets (subtracted when returning "from zero" values).
        self._zero_pitch = 0.0
        self._zero_roll = 0.0
        self._zero_yaw = 0.0

    def update_accel(self, ax: float, ay: float, az: float) -> None:
        """Update pitch and roll from a gravity-direction measurement (units: g)."""
        # Standard small-angle decomposition of gravity vector:
        self.pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        self.roll = math.atan2(ay, az) if az != 0.0 else 0.0

    def update_gyro(self, gx: float, gy: float, gz: float, now: Optional[float] = None) -> None:
        """Integrate gyro z-rate into yaw. gx/gy/gz are rad/s."""
        now = now if now is not None else time.monotonic()
        if self._last_gyro_t is None:
            self._last_gyro_t = now
            return
        dt = now - self._last_gyro_t
        self._last_gyro_t = now
        # Simple rectangular integration on yaw only.
        self.yaw += gz * dt

    def capture_zero(self) -> None:
        """Snapshot current orientation as the zero reference."""
        self._zero_pitch = self.pitch
        self._zero_roll = self.roll
        self._zero_yaw = self.yaw

    def from_zero(self) -> tuple[float, float, float]:
        """Return (pitch, roll, yaw) relative to the last capture_zero() call."""
        return (
            self.pitch - self._zero_pitch,
            self.roll - self._zero_roll,
            self.yaw - self._zero_yaw,
        )
