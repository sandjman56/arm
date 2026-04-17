"""Piecewise-constant-curvature (PCC) kinematics for the continuum arm.

The trunk is modeled as a single circular arc of arclength L, tilting by
angle theta from the base's +Z axis in a plane at azimuth phi around Z.
"""
from __future__ import annotations

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def forward_kinematics(L: float, theta: float, phi: float) -> Vec3:
    """Return tip position (x, y, z) in the physics frame.

    L     : arclength (mm), >= 0
    theta : tilt magnitude (radians), 0 = straight up along +Z
    phi   : azimuth of the bend plane (radians around Z)
    """
    if L < 0:
        raise ValueError(f"L must be non-negative, got {L}")
    # Straight case — avoid division by zero.
    if abs(theta) < 1e-9:
        return (0.0, 0.0, L)
    # General PCC: radius R = L/theta, tip in bend plane at (R*(1-cos t), R*sin t).
    R = L / theta
    r_plane = R * (1.0 - math.cos(theta))
    z = R * math.sin(theta)
    x = r_plane * math.cos(phi)
    y = r_plane * math.sin(phi)
    return (x, y, z)


class Unreachable(ValueError):
    """Raised when a target cannot be reached by any valid (L, theta, phi)."""


def inverse_kinematics(target: Vec3) -> Tuple[float, float, float]:
    """Return (L, theta, phi) such that forward_kinematics(L, theta, phi) == target.

    Raises Unreachable if the target is outside the representable half-space.
    """
    x, y, z = target
    if z < 0:
        raise Unreachable(f"target {target} has z < 0; arc cannot reach below base")

    phi = math.atan2(y, x)
    r = math.hypot(x, y)  # horizontal distance from Z axis

    # Degenerate: target on Z axis => straight trunk
    if r < 1e-9:
        if z <= 0:
            raise Unreachable(f"target {target} on Z axis with z<=0")
        return (z, 0.0, 0.0)

    # Planar geometry: tip at (r, z). For a PCC arc rooted at origin with
    # initial tangent +Z and radius R, the tip lies at (R*(1-cos t), R*sin t).
    # So: r = R*(1-cos t), z = R*sin t => r^2 + z^2 = 2*R*r (from identity
    # (1-cos)^2 + sin^2 = 2(1-cos)), giving R = (r^2 + z^2) / (2r).
    R = (r * r + z * z) / (2.0 * r)
    # theta from z = R*sin(theta), clamp for numerical safety.
    sin_t = max(-1.0, min(1.0, z / R))
    theta = math.asin(sin_t)
    L = R * theta
    return (L, theta, phi)


def is_reachable(target: Vec3, L_min: float, L_max: float, theta_max: float) -> bool:
    """Check whether target is inside the PCC reachable workspace."""
    try:
        L, theta, _phi = inverse_kinematics(target)
    except Unreachable:
        return False
    if not (L_min <= L <= L_max):
        return False
    if not (0.0 <= theta <= theta_max):
        return False
    return True
