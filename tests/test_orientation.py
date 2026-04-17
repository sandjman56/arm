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
