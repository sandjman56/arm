"""Experiments-mode controller state machine + bend-phase P-controller."""
from __future__ import annotations
import enum
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

from experiment_backend import Backend
from length_calibration import LengthCalibration


class State(enum.Enum):
    IDLE = "IDLE"
    ZEROING = "ZEROING"
    WAITING_FOR_TARGET = "WAITING_FOR_TARGET"
    ELONGATING = "ELONGATING"
    BENDING = "BENDING"
    REACHED = "REACHED"
    TIMED_OUT = "TIMED_OUT"


@dataclass
class RunResult:
    final_pitch_err_rad: float
    final_position_err_mm: float
    elapsed_s: float
    timed_out: bool


class ExperimentController:
    ELONGATION_TIMEOUT_S = 10.0
    BEND_TIMEOUT_S = 15.0
    TOL_ANGLE_RAD = math.radians(3)
    TOL_POS_MM = 10.0
    KP_BEND = 4.0
    OMEGA_MAX = 1.0  # tendon rate saturation, in [-1, 1]

    def __init__(self, backend: Backend, calibration: LengthCalibration):
        self.backend = backend
        self.cal = calibration
        self.state = State.IDLE
        self._target: Optional[Tuple[float, float, float]] = None  # in physics frame
        self._phase_start: float = 0.0
        self._last_result: Optional[RunResult] = None

    # --- Transitions -------------------------------------------------------

    def start_zeroing(self) -> None:
        self.state = State.ZEROING

    def confirm_zero(self) -> None:
        if self.state != State.ZEROING:
            return
        self.backend.capture_zero()
        self.state = State.WAITING_FOR_TARGET

    def emergency_stop(self) -> None:
        self.backend.emergency_stop()
        self.state = State.IDLE
        self._target = None

    @property
    def last_result(self) -> Optional[RunResult]:
        return self._last_result

    # --- Reach -------------------------------------------------------------

    def reach(self, target: Tuple[float, float, float]) -> None:
        """Begin a reach attempt to `target` in the physics frame."""
        if self.state != State.WAITING_FOR_TARGET and self.state != State.REACHED:
            return
        from kinematics import is_reachable, inverse_kinematics
        theta_max = math.radians(60)
        if not is_reachable(target, L_min=self.cal.L_rest, L_max=self.cal.L_max, theta_max=theta_max):
            raise ValueError(f"target {target} outside reachable workspace")
        self._target = target
        L_target, theta_target, phi_target = inverse_kinematics(target)
        # Distribute total length across available (calibrated) modules.
        self._L_target = L_target
        self._theta_target = theta_target
        self._phi_target = phi_target
        self._phase_start = time.monotonic()
        # Command module pressures. For default calibration, split evenly.
        self._command_module_pressures_for_length(L_target)
        # In sim mode, also poke the backend with the total length directly
        # so its slewing engine has a target.
        if hasattr(self.backend, "set_total_length_target"):
            self.backend.set_total_length_target(L_target)
        self.state = State.ELONGATING

    def _command_module_pressures_for_length(self, L_total: float) -> None:
        """Solve per-module pressures that sum to L_total using calibration curves.

        Simple strategy: equal fractional extension across modules. If a module
        has no calibration (default), command a proportional pressure.
        """
        num = max(1, len(self.cal.modules) or 6)
        per_module = L_total / num
        for mid in (self.cal.modules.keys() if self.cal.modules else range(1, num + 1)):
            if self.cal.modules:
                m = self.cal.modules[mid]
                a, b, c = m["coeffs"]
                # Solve a + b*p + c*p^2 = per_module for p.
                psi = _solve_psi_for_length(a, b, c, per_module, m["max_psi"])
            else:
                # Default: linear assumption, 40mm rest + 5mm/psi.
                psi = max(0.0, (per_module - 40.0) / 5.0)
            self.backend.set_module_pressure(mid, psi)


def _solve_psi_for_length(a: float, b: float, c: float, target_len: float, max_psi: float) -> float:
    """Solve a + b*p + c*p^2 = target_len for p in [0, max_psi]. Falls back to
    clamped endpoints if no root exists in range."""
    if abs(c) < 1e-12:
        # Linear.
        if abs(b) < 1e-12:
            return 0.0
        return max(0.0, min(max_psi, (target_len - a) / b))
    # Quadratic: c*p^2 + b*p + (a - target_len) = 0
    disc = b * b - 4 * c * (a - target_len)
    if disc < 0:
        # Unreachable length; clamp.
        return max_psi if target_len > a else 0.0
    sqrt_d = math.sqrt(disc)
    p1 = (-b + sqrt_d) / (2 * c)
    p2 = (-b - sqrt_d) / (2 * c)
    # Pick the positive root in [0, max_psi] if any; else clamp.
    for p in sorted([p1, p2]):
        if 0.0 <= p <= max_psi:
            return p
    return max(0.0, min(max_psi, max(p1, p2)))
