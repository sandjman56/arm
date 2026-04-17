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
