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

    # --- Main tick ---------------------------------------------------------

    def tick(self, dt: float) -> None:
        """Advance the controller by dt seconds. Called ~20 Hz from the UI."""
        # Always let the backend advance first.
        self.backend.tick(dt)

        if self.state == State.ELONGATING:
            self._tick_elongating()
        elif self.state == State.BENDING:
            self._tick_bending()

    def _tick_elongating(self) -> None:
        s = self.backend.read_state()
        elapsed = time.monotonic() - self._phase_start
        # Convergence: SimBackend length near target, or pressures near setpoints.
        if abs(s.total_length_mm - self._L_target) < 5.0:
            # Move on to bending phase.
            self._phase_start = time.monotonic()
            # Command orientation target for sim; the real backend sees commands
            # via set_tendon_rate() during _tick_bending.
            if hasattr(self.backend, "set_orientation_target"):
                # Convert target direction into pitch/roll (roll=0, pitch=theta in bend plane).
                # For sim: project into pitch/roll according to phi.
                target_pitch = self._theta_target * math.cos(self._phi_target)
                target_roll = self._theta_target * math.sin(self._phi_target)
                self.backend.set_orientation_target(target_pitch, target_roll, 0.0)
            self.state = State.BENDING
            return
        if elapsed > self.ELONGATION_TIMEOUT_S:
            self._finish(timed_out=True)

    def _finish(self, timed_out: bool) -> None:
        elapsed = time.monotonic() - self._phase_start
        pitch, roll, _yaw = self.backend.read_orientation()
        # Angular error: angle between current tilt vector and target tilt vector.
        cur_pitch = pitch
        cur_roll = roll
        target_pitch = self._theta_target * math.cos(self._phi_target)
        target_roll = self._theta_target * math.sin(self._phi_target)
        # Simple 2D error norm:
        ang_err = math.hypot(cur_pitch - target_pitch, cur_roll - target_roll)
        # Position error: forward kinematics from current state.
        from kinematics import forward_kinematics
        s = self.backend.read_state()
        theta_now = math.hypot(cur_pitch, cur_roll)
        phi_now = math.atan2(cur_roll, cur_pitch) if theta_now > 1e-9 else 0.0
        tip = forward_kinematics(s.total_length_mm, theta_now, phi_now)
        tx, ty, tz = self._target or (0, 0, 0)
        pos_err = math.sqrt((tip[0] - tx) ** 2 + (tip[1] - ty) ** 2 + (tip[2] - tz) ** 2)
        self._last_result = RunResult(
            final_pitch_err_rad=ang_err,
            final_position_err_mm=pos_err,
            elapsed_s=elapsed,
            timed_out=timed_out,
        )
        self.state = State.TIMED_OUT if timed_out else State.REACHED

    def _tick_bending(self) -> None:
        s = self.backend.read_state()
        # Live-mode IMU-staleness watchdog (spec §8). SimBackend reports fresh.
        if not self.backend.is_sim and not s.imu_fresh:
            self.emergency_stop()
            return
        elapsed = time.monotonic() - self._phase_start
        pitch, roll, _yaw = self.backend.read_orientation()

        target_pitch = self._theta_target * math.cos(self._phi_target)
        target_roll = self._theta_target * math.sin(self._phi_target)

        err_pitch = target_pitch - pitch
        err_roll = target_roll - roll

        # Position error check (kinematic).
        from kinematics import forward_kinematics
        theta_now = math.hypot(pitch, roll)
        phi_now = math.atan2(roll, pitch) if theta_now > 1e-9 else 0.0
        tip = forward_kinematics(s.total_length_mm, theta_now, phi_now)
        tx, ty, tz = self._target
        pos_err = math.sqrt((tip[0] - tx) ** 2 + (tip[1] - ty) ** 2 + (tip[2] - tz) ** 2)
        ang_err = math.hypot(err_pitch, err_roll)

        if ang_err < self.TOL_ANGLE_RAD and pos_err < self.TOL_POS_MM:
            # Halt tendons and declare REACHED.
            for sid in (1, 2, 3, 4):
                self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=False)
            return

        if elapsed > self.BEND_TIMEOUT_S:
            for sid in (1, 2, 3, 4):
                self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=True)
            return

        # 4-tendon antagonistic mapping:
        # servo 1 at 0 deg  winds to pull +pitch
        # servo 3 at 180 deg winds to pull -pitch
        # servo 2 at 90 deg winds to pull +roll
        # servo 4 at 270 deg winds to pull -roll
        u_pitch = self.KP_BEND * err_pitch
        u_roll = self.KP_BEND * err_roll
        rate_1 = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, +u_pitch))
        rate_3 = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, -u_pitch))
        rate_2 = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, +u_roll))
        rate_4 = max(-self.OMEGA_MAX, min(self.OMEGA_MAX, -u_roll))
        self.backend.set_tendon_rate(1, rate_1)
        self.backend.set_tendon_rate(2, rate_2)
        self.backend.set_tendon_rate(3, rate_3)
        self.backend.set_tendon_rate(4, rate_4)

    # --- Re-zero guard -----------------------------------------------------

    NEAR_REST_PSI = 0.5  # psi threshold below which modules count as resting

    def can_rezero(self) -> bool:
        """True if safe to capture a new zero (all pressures near rest)."""
        s = self.backend.read_state()
        return all(abs(p) < self.NEAR_REST_PSI for p in s.module_pressures_psi.values())

    def rezero(self) -> bool:
        """Attempt to capture a new zero. Returns True if allowed."""
        if not self.can_rezero():
            return False
        self.backend.capture_zero()
        return True

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
