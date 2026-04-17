"""Unit tests for kinematics.py (PCC continuum-arm model)."""
import math
import pytest
from kinematics import forward_kinematics


def test_straight_trunk_tip_is_at_height_L():
    """A trunk with zero tilt should have its tip at (0, 0, L)."""
    tip = forward_kinematics(L=100.0, theta=0.0, phi=0.0)
    assert tip == pytest.approx((0.0, 0.0, 100.0), abs=1e-9)


def test_zero_length_returns_origin():
    tip = forward_kinematics(L=0.0, theta=0.0, phi=0.0)
    assert tip == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)


def test_quarter_arc_in_xz_plane():
    """Arc of length L with theta=pi/2 in the XZ plane (phi=0).
    Radius R = L / (pi/2). Tip at (R, 0, R)."""
    L = math.pi / 2  # so R = 1
    tip = forward_kinematics(L=L, theta=math.pi / 2, phi=0.0)
    assert tip == pytest.approx((1.0, 0.0, 1.0), abs=1e-9)


def test_quarter_arc_in_yz_plane():
    """Same arc rotated 90 deg around Z (phi=pi/2): tip at (0, R, R)."""
    L = math.pi / 2
    tip = forward_kinematics(L=L, theta=math.pi / 2, phi=math.pi / 2)
    assert tip == pytest.approx((0.0, 1.0, 1.0), abs=1e-9)


def test_small_theta_is_continuous_with_straight():
    """As theta -> 0, tip should approach (0, 0, L) smoothly."""
    L = 100.0
    tip_tiny = forward_kinematics(L=L, theta=1e-6, phi=0.0)
    assert tip_tiny == pytest.approx((0.0, 0.0, L), abs=1e-3)


def test_negative_L_raises():
    with pytest.raises(ValueError):
        forward_kinematics(L=-1.0, theta=0.0, phi=0.0)
