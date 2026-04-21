"""Experiments-mode controller state machine + bend-phase P-controller."""
from __future__ import annotations
import enum
import math
import time
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

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
    ELONGATION_TIMEOUT_S = 45.0
    MIN_ELONGATION_S = 2.0          # don't transition to BENDING sooner than this
    BEND_TIMEOUT_S = 30.0
    TOL_ANGLE_RAD = math.radians(3)
    TOL_POS_MM = 10.0
    TOL_PRESSURE_PSI = 0.3  # per-module setpoint convergence
    KP_BEND = 4.0                 # legacy rate gain (unused when servo_defaults is set)
    KP_BEND_ANGLE_DEG_PER_RAD = 30.0
    BEND_ANGLE_MAX_DEG = 45.0
    OMEGA_MAX = 1.0  # tendon rate saturation, in [-1, 1]
    SLACK_PER_PSI = 10.0 / 0.3    # degrees of tendon slack per psi of aggregate module delta
    SLACK_MAX_DEG = 90.0          # absolute clamp on magnitude of slack offset
    EXCLUDED_MODULES = frozenset({6})  # no pressure sensor wired; skip for baseline/slack/pressurize

    def __init__(self, backend: Backend, calibration: LengthCalibration):
        self.backend = backend
        self.cal = calibration
        self.state = State.IDLE
        self._target: Optional[Tuple[float, float, float]] = None  # in physics frame
        self._phase_start: float = 0.0
        self._last_result: Optional[RunResult] = None
        # Pressure-driven tendon-slack state. Baseline latched at confirm_zero;
        # servo defaults latched at reach(). Angle-control path is active only
        # while both are populated.
        self._psi_baseline: Optional[Dict[int, float]] = None
        self._servo_defaults: Optional[Dict[int, float]] = None
        self._pressure_ceiling_psi: Optional[float] = None
        # Last commanded per-module pressure setpoints (for convergence check).
        self._pressure_setpoints: Dict[int, float] = {}
        # Last commanded tendon-servo angles (for logging).
        self.last_tendon_angles: Dict[int, float] = {}

    # --- Transitions -------------------------------------------------------

    def start_zeroing(self) -> None:
        self.state = State.ZEROING

    def confirm_zero(self) -> None:
        if self.state != State.ZEROING:
            return
        self.backend.capture_zero()
        # Latch pressure baseline for tendon-slack compensation, excluding
        # modules without pressure sensors (see EXCLUDED_MODULES).
        pressures = self.backend.read_state().module_pressures_psi
        self._psi_baseline = {
            mid: p for mid, p in pressures.items() if mid not in self.EXCLUDED_MODULES
        }
        self.state = State.WAITING_FOR_TARGET

    def emergency_stop(self) -> None:
        self.backend.emergency_stop()
        self.state = State.IDLE
        self._target = None
        # Disengage the angle-control path. Baseline stays; operator must
        # re-Reach to re-arm tendon writes.
        self._servo_defaults = None

    @property
    def last_result(self) -> Optional[RunResult]:
        return self._last_result

    # --- Reach -------------------------------------------------------------

    def reach(
        self,
        target: Tuple[float, float, float],
        servo_defaults: Optional[Dict[int, float]] = None,
        pressure_ceiling_psi: Optional[float] = None,
    ) -> None:
        """Begin a reach attempt to `target` in the physics frame.

        `servo_defaults` is a {1..4: angle_deg} dict of the operator's current
        tendon-servo angles. When provided, the controller uses absolute-angle
        control with pressure-driven slack compensation; the bend P-controller
        composes on top (common-mode slack + differential-mode bend). When
        omitted, the legacy rate-based path is used (tests, simulation).
        """
        if self.state not in (State.WAITING_FOR_TARGET, State.REACHED, State.TIMED_OUT):
            raise ValueError(
                f"not ready to Reach (state={self.state.value}). "
                "Press Zero @ Rest → Confirm Zero first."
            )
        if servo_defaults is not None and self._psi_baseline is None:
            raise ValueError(
                "pressure baseline not captured — press Confirm Zero before Reach"
            )
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
        # Latch servo defaults (by value). None disables the angle-control path.
        self._servo_defaults = dict(servo_defaults) if servo_defaults is not None else None
        self._pressure_ceiling_psi = pressure_ceiling_psi
        # Command module pressures for the requested arc length, clamped to
        # the operator-supplied safety ceiling.
        self._command_module_pressures_for_length(L_target)
        # In sim mode, poke the backend with the total length directly so its
        # slewing engine has a target.
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

        # Angle-control path: whenever defaults + baseline are latched, write
        # composed tendon angles each tick so (a) slack tracks pressure smoothly
        # during elongation/deflation, and (b) the bend P-controller layers on
        # top during BENDING. Inactive in tests/sim that pass no defaults.
        if self._servo_defaults is not None and self._psi_baseline is not None:
            self._update_tendon_angles()

    def _tick_elongating(self) -> None:
        s = self.backend.read_state()
        elapsed = time.monotonic() - self._phase_start
        # Hold in ELONGATING for at least MIN_ELONGATION_S so inflation and
        # tendon slack pay-out are visible and physically complete.
        if elapsed < self.MIN_ELONGATION_S:
            return
        length_converged = abs(s.total_length_mm - self._L_target) < 5.0
        pressure_converged = bool(self._pressure_setpoints) and all(
            abs(s.module_pressures_psi.get(mid, 0.0) - sp) < self.TOL_PRESSURE_PSI
            for mid, sp in self._pressure_setpoints.items()
        )
        if length_converged or pressure_converged:
            self._phase_start = time.monotonic()
            if hasattr(self.backend, "set_orientation_target"):
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
            # Halt the bend P-controller; tendons hold at their last commanded
            # angle (firmware latches). Slack continues to track pressure via
            # _update_tendon_angles() in subsequent ticks.
            if self._servo_defaults is None:
                for sid in (1, 2, 3, 4):
                    self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=False)
            return

        if elapsed > self.BEND_TIMEOUT_S:
            if self._servo_defaults is None:
                for sid in (1, 2, 3, 4):
                    self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=True)
            return

        # Angle-control path handles bend writes in _update_tendon_angles().
        # Legacy rate-control path is only engaged when no servo_defaults were
        # provided (tests, headless sim).
        if self._servo_defaults is None:
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
        """Command every non-excluded module to the operator pressure ceiling.

        The controller treats pressure as stiffness (bending comes from tendons),
        so all active modules inflate to the same target during ELONGATING. If
        per-module calibration is present, solve its quadratic for the target
        arc length instead; otherwise default to the ceiling.
        """
        if self.cal.modules:
            active_ids = [mid for mid in self.cal.modules.keys() if mid not in self.EXCLUDED_MODULES]
        else:
            active_ids = [mid for mid in range(1, 7) if mid not in self.EXCLUDED_MODULES]
        per_module = L_total / max(1, len(active_ids))
        for mid in active_ids:
            baseline = (self._psi_baseline or {}).get(mid, 0.0)
            if self.cal.modules and mid in self.cal.modules:
                m = self.cal.modules[mid]
                a, b, c = m["coeffs"]
                psi = _solve_psi_for_length(a, b, c, per_module, m["max_psi"])
            elif self._pressure_ceiling_psi is not None:
                psi = self._pressure_ceiling_psi
            else:
                psi = baseline
            psi = max(psi, baseline)
            if self._pressure_ceiling_psi is not None:
                psi = min(psi, self._pressure_ceiling_psi)
            self._pressure_setpoints[mid] = psi
            self.backend.set_module_pressure(mid, psi)

    # --- Angle-control path -----------------------------------------------

    def _compute_slack_deg(self) -> float:
        """Common-mode tendon slack (negative = pay out) from aggregate Δpsi.

        Slack = −Σ max(0, psi_m − baseline_m) × (10 / 0.3) degrees, summed
        over non-excluded modules. Clamped to [−SLACK_MAX_DEG, 0]. Symmetric
        deflation returns to 0 naturally via the max(0, …) floor and the
        instantaneous pressure reading.
        """
        if self._psi_baseline is None:
            return 0.0
        pressures = self.backend.read_state().module_pressures_psi
        sum_delta = 0.0
        for mid, base in self._psi_baseline.items():
            cur = pressures.get(mid, base)
            sum_delta += max(0.0, cur - base)
        slack = -sum_delta * self.SLACK_PER_PSI
        # Never over-tension past defaults; never exceed safety clamp.
        return max(-self.SLACK_MAX_DEG, min(0.0, slack))

    def _compute_bend_offsets_deg(self) -> Dict[int, float]:
        """Differential-mode P-controller offsets (degrees) per tendon.

        Zero outside BENDING. 1↔3 antagonistic for pitch, 2↔4 for roll.
        """
        if self.state != State.BENDING:
            return {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0}
        pitch, roll, _yaw = self.backend.read_orientation()
        target_pitch = self._theta_target * math.cos(self._phi_target)
        target_roll = self._theta_target * math.sin(self._phi_target)
        err_pitch = target_pitch - pitch
        err_roll = target_roll - roll
        k = self.KP_BEND_ANGLE_DEG_PER_RAD
        lim = self.BEND_ANGLE_MAX_DEG
        u_pitch = max(-lim, min(lim, k * err_pitch))
        u_roll = max(-lim, min(lim, k * err_roll))
        return {
            1: +u_pitch,
            3: -u_pitch,
            2: +u_roll,
            4: -u_roll,
        }

    def _update_tendon_angles(self) -> None:
        """Write composed absolute angles to all 4 tendon servos."""
        slack = self._compute_slack_deg()
        bend = self._compute_bend_offsets_deg()
        for sid in (1, 2, 3, 4):
            default = self._servo_defaults.get(sid, 0.0) if self._servo_defaults else 0.0
            angle = default + slack + bend.get(sid, 0.0)
            self.last_tendon_angles[sid] = angle
            self.backend.set_tendon_angle(sid, angle)


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
