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
