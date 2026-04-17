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


from kinematics import inverse_kinematics, Unreachable


def test_inverse_straight_up():
    """Target directly above base recovers L = z, theta = 0."""
    L, theta, phi = inverse_kinematics((0.0, 0.0, 100.0))
    assert L == pytest.approx(100.0, abs=1e-6)
    assert theta == pytest.approx(0.0, abs=1e-6)


def test_inverse_round_trip_in_xz():
    """FK(IK(target)) == target for a reachable point."""
    target = (50.0, 0.0, 80.0)
    L, theta, phi = inverse_kinematics(target)
    tip = forward_kinematics(L, theta, phi)
    assert tip == pytest.approx(target, abs=1e-4)


def test_inverse_round_trip_arbitrary_azimuth():
    target = (30.0, 40.0, 70.0)
    L, theta, phi = inverse_kinematics(target)
    tip = forward_kinematics(L, theta, phi)
    assert tip == pytest.approx(target, abs=1e-4)


def test_inverse_rejects_point_below_base():
    """The arc can't reach z < 0 (modeled trunk only tilts up to 90 deg)."""
    with pytest.raises(Unreachable):
        inverse_kinematics((10.0, 0.0, -5.0))


from kinematics import is_reachable


def test_reachable_target_inside_workspace():
    # L_min=100, L_max=300, theta_max=60deg. Point straight up at z=150 is reachable.
    assert is_reachable((0.0, 0.0, 150.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_too_far():
    assert not is_reachable((0.0, 0.0, 1000.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_too_close():
    assert not is_reachable((0.0, 0.0, 50.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_theta_exceeded():
    # A target far out horizontally needs theta > theta_max.
    assert not is_reachable((200.0, 0.0, 10.0), L_min=100, L_max=300, theta_max=math.radians(30))


def test_unreachable_below_base():
    assert not is_reachable((10.0, 0.0, -5.0), L_min=100, L_max=300, theta_max=math.radians(60))
