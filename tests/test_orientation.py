"""Unit tests for orientation.py (IMU-based tip orientation estimator)."""
import math
import pytest
from orientation import OrientationEstimator


def test_level_accel_gives_zero_pitch_roll():
    """With accelerometer reading (0, 0, 1) g — tip pointing straight up along
    its own body +Z and base +Z aligned — pitch and roll should be 0."""
    est = OrientationEstimator()
    est.update_accel(ax=0.0, ay=0.0, az=1.0)
    assert est.pitch == pytest.approx(0.0, abs=1e-6)
    assert est.roll == pytest.approx(0.0, abs=1e-6)


def test_tilted_forward_gives_nonzero_pitch():
    """Accelerometer reading (sin 30, 0, cos 30) means the tip is tilted
    30 deg around the Y axis (pitch)."""
    est = OrientationEstimator()
    est.update_accel(ax=math.sin(math.radians(30)), ay=0.0, az=math.cos(math.radians(30)))
    assert est.pitch == pytest.approx(math.radians(30), abs=1e-4)
    assert est.roll == pytest.approx(0.0, abs=1e-4)


def test_gyro_yaw_integrates():
    """Feeding gz=1 rad/s for 1s should yield yaw ~= 1 rad."""
    est = OrientationEstimator()
    # Seed the integrator with t=0.
    est.update_gyro(gx=0.0, gy=0.0, gz=1.0, now=0.0)
    # Step 1 second forward.
    est.update_gyro(gx=0.0, gy=0.0, gz=1.0, now=1.0)
    assert est.yaw == pytest.approx(1.0, abs=1e-6)


def test_capture_zero_offsets_subsequent_reads():
    est = OrientationEstimator()
    est.pitch = 0.1
    est.roll = 0.2
    est.yaw = 0.3
    est.capture_zero()
    # Still the same absolute values, but from_zero() returns deltas.
    assert est.from_zero() == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    est.pitch = 0.15
    assert est.from_zero() == pytest.approx((0.05, 0.0, 0.0), abs=1e-9)


def test_from_zero_without_capture_returns_absolute():
    est = OrientationEstimator()
    est.pitch = 0.5
    assert est.from_zero()[0] == pytest.approx(0.5, abs=1e-9)
