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
    RETRACTING = "RETRACTING"
    REACHED = "REACHED"
    TIMED_OUT = "TIMED_OUT"


class ExperimentMode(enum.Enum):
    COMPLEX = "COMPLEX"
    BASIC_ELONGATION = "BASIC_ELONGATION"


@dataclass
class RunResult:
    final_pitch_err_rad: float
    final_position_err_mm: float
    elapsed_s: float
    timed_out: bool


class ExperimentController:
    MIN_ELONGATION_S = 2.0          # don't transition to BENDING sooner than this
    TOL_ANGLE_RAD = math.radians(3)
    TOL_POS_MM = 10.0
    TOL_PRESSURE_PSI = 0.3  # per-module setpoint convergence
    KP_BEND = 4.0                 # legacy rate gain (unused when servo_defaults is set)
    KP_BEND_ANGLE_DEG_PER_RAD = 30.0
    BEND_ANGLE_MAX_DEG = 45.0
    OMEGA_MAX = 1.0  # tendon rate saturation, in [-1, 1]
    SLACK_MAX_DEG = 90.0          # common-mode tendon pay-out at full inflation
    # Tendon pulley geometry — used to convert commanded servo angle change
    # into arc-length change (ΔL = |Δθ| × r) during ELONGATING. Replaces the
    # IMU as the length source during that phase (IMU stays responsible for
    # orientation during BENDING only).
    PULLEY_RADIUS_MM = 25.0       # 2.5 cm radius (5 cm diameter)
    EXCLUDED_MODULES = frozenset()  # all 6 modules have pressure sensors wired
    # Synchronized-inflation ramp. All modules' commanded pressures walk from
    # baseline toward their finals via a single shared [0, 1] progress scalar
    # that rises at INFLATION_RATE_PER_S. Open-loop on purpose — the previous
    # actual-pressure gate stalled for ~20 s at startup when firmware PID
    # deadband ate the initial ramp deltas.
    INFLATION_RATE_PER_S = 0.2    # full ramp in ~5 s (matches 2× stepper rate)
    # Basic Elongation sub-mode tuning.
    BASIC_UNWIND_KP_DEG_PER_S_PER_PSI = 10.0   # unwind rate per psi of overshoot
    BASIC_UNWIND_MAX_DEG_PER_S = 30.0           # clamp on unwind rate
    # Retraction safety: soft-start ramp so the reverse motion eases in from
    # zero rather than slamming to full rate at the instant of target-reach.
    # Linear ramp from 0 to target rate over BASIC_RETRACT_RAMP_S seconds.
    BASIC_RETRACT_RAMP_S = 0.5
    # Hard absolute ceiling on per-tick servo step during retraction,
    # independent of speed_scale. Bounds worst-case motion if the UI ever
    # sends a pathological speed value.
    BASIC_RETRACT_MAX_STEP_DEG = 3.0
    # After the elongation target is hit in basic mode, linger in REACHED for
    # this many seconds before beginning RETRACTING. Purpose: make the target-
    # reach moment visible as distinct rows in the CSV phase column and as a
    # clear visual indicator in the UI. The firmware holds its last commanded
    # position during the hold (no servo/pressure rewrites).
    BASIC_REACHED_HOLD_S = 0.5

    def __init__(self, backend: Backend, calibration: LengthCalibration):
        self.backend = backend
        self.cal = calibration
        self.state = State.IDLE
        self.mode: ExperimentMode = ExperimentMode.COMPLEX
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
        # Synchronized-inflation ramp state. `_inflation_final` holds the final
        # per-module psi targets computed at reach(); `_inflation_progress` is
        # the shared [0, 1] ramp fraction that tick() advances during ELONGATING.
        self._inflation_final: Dict[int, float] = {}
        self._inflation_progress: float = 0.0
        # Last commanded tendon-servo angles (for logging).
        self.last_tendon_angles: Dict[int, float] = {}
        # Basic-mode state.
        self._basic_z_target_mm: float = 0.0
        self._basic_psi_threshold: float = 15.0
        self._basic_speed_scale: float = 1.0
        self._basic_slack_deg: float = 0.0
        self._basic_elongation_mm: float = 0.0
        self._basic_retract_elapsed_s: float = 0.0
        self._basic_reached_hold_elapsed_s: float = 0.0

    # --- Transitions -------------------------------------------------------

    def start_zeroing(self) -> None:
        self.state = State.ZEROING

    def confirm_zero(self, servo_defaults: Optional[Dict[int, float]] = None) -> None:
        if self.state != State.ZEROING:
            return
        self.backend.capture_zero()
        # Latch pressure baseline for tendon-slack compensation, excluding
        # modules without pressure sensors (see EXCLUDED_MODULES).
        pressures = self.backend.read_state().module_pressures_psi
        self._psi_baseline = {
            mid: p for mid, p in pressures.items() if mid not in self.EXCLUDED_MODULES
        }
        if servo_defaults is not None:
            self._servo_defaults = dict(servo_defaults)
        self.state = State.WAITING_FOR_TARGET

    def emergency_stop(self) -> None:
        self.backend.emergency_stop()
        self.state = State.IDLE
        self._target = None
        # Disengage the angle-control path. Baseline stays; operator must
        # re-Reach to re-arm tendon writes.
        self._servo_defaults = None
        # Drop the inflation ramp; next reach() rebuilds it from baseline.
        self._inflation_final = {}
        self._inflation_progress = 0.0

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
        # Precompute the final per-module psi targets and arm the synchronized
        # inflation ramp. Initial commanded pressures are each module's baseline
        # so the firmware PID latches on without an immediate burst; tick()
        # walks every module up together during ELONGATING.
        self._inflation_final = self._compute_module_pressure_targets(L_target)
        self._inflation_progress = 0.0
        self._pressure_setpoints = {}
        baseline = self._psi_baseline or {}
        for mid in self._inflation_final:
            base = baseline.get(mid, 0.0)
            self._pressure_setpoints[mid] = base
            self.backend.set_module_pressure(mid, base)
        # In sim mode, poke the backend with the total length directly so its
        # slewing engine has a target.
        if hasattr(self.backend, "set_total_length_target"):
            self.backend.set_total_length_target(L_target)
        self.state = State.ELONGATING

    def reach_basic(
        self,
        z_target_mm: float,
        psi_threshold: float,
        pressure_ceiling_psi: Optional[float] = None,
        speed_scale: float = 1.0,
    ) -> None:
        """Begin a Basic Elongation run.

        Steppers keep inflating; the 4 tendon servos share a common-mode
        unwind driven by a P-controller on max-balloon-psi vs. threshold.
        Elongation is integrated from slack angle x pulley radius. Reach
        declared when elongation >= z_target_mm.

        `speed_scale` multiplies both the unwind kp and the unwind max rate.
        Steppers benefit indirectly: faster unwind means balloons keep
        expanding, so the firmware PID has room to keep pumping.
        """
        if self.mode != ExperimentMode.BASIC_ELONGATION:
            raise ValueError("reach_basic requires BASIC_ELONGATION mode")
        if self.state not in (State.WAITING_FOR_TARGET, State.REACHED, State.TIMED_OUT):
            raise ValueError(
                f"not ready to Reach (state={self.state.value}). "
                "Press Zero @ Rest -> Confirm Zero first."
            )
        if self._servo_defaults is None:
            raise ValueError(
                "servo defaults not latched - press Confirm Zero with servo angles first"
            )
        if speed_scale <= 0:
            raise ValueError(f"speed_scale must be positive, got {speed_scale}")
        self._basic_z_target_mm = float(z_target_mm)
        self._basic_psi_threshold = float(psi_threshold)
        self._basic_speed_scale = float(speed_scale)
        self._basic_slack_deg = 0.0
        self._basic_elongation_mm = 0.0
        self._pressure_ceiling_psi = pressure_ceiling_psi
        # Flat per-module setpoint: operator ceiling wins; otherwise threshold+5.
        flat_setpoint = (
            pressure_ceiling_psi if pressure_ceiling_psi is not None
            else psi_threshold + 5.0
        )
        self._pressure_setpoints = {}
        for mid in range(1, 7):
            if mid in self.EXCLUDED_MODULES:
                continue
            self._pressure_setpoints[mid] = flat_setpoint
            self.backend.set_module_pressure(mid, flat_setpoint)
        self._phase_start = time.monotonic()
        self.state = State.ELONGATING

    # --- Main tick ---------------------------------------------------------

    _ACTIVE_STATES = frozenset({State.ELONGATING, State.BENDING, State.REACHED})
    # ELONGATING is intentionally excluded from the IMU watchdog: during
    # inflation the tip can swing visibly and the IMU is noisy enough that
    # `imu_fresh` false positives would E-stop healthy runs. Length comes
    # from the tendon-servo angles + pulley geometry in that phase, so the
    # IMU isn't load-bearing until BENDING.
    _IMU_WATCHDOG_STATES = frozenset({State.BENDING, State.REACHED})

    def tick(self, dt: float) -> None:
        """Advance the controller by dt seconds. Called ~20 Hz from the UI."""
        # Always let the backend advance first.
        self.backend.tick(dt)

        # Live-mode IMU-staleness watchdog (spec §8). Active during BENDING
        # and REACHED only — ELONGATING uses servo-angle-derived length and
        # does not need a valid orientation estimate.
        if self.state in self._IMU_WATCHDOG_STATES and not self.backend.is_sim:
            if not self.backend.read_state().imu_fresh:
                self.emergency_stop()
                return

        if self.state == State.ELONGATING:
            if self.mode == ExperimentMode.BASIC_ELONGATION:
                self._tick_basic_elongating(dt)
            else:
                self._advance_inflation_ramp(dt)
                self._tick_elongating()
        elif self.state == State.RETRACTING:
            self._tick_basic_retracting(dt)
        elif (
            self.state == State.REACHED
            and self.mode == ExperimentMode.BASIC_ELONGATION
            and self._basic_reached_hold_elapsed_s < self.BASIC_REACHED_HOLD_S
        ):
            self._tick_basic_reached_hold(dt)
        elif self.state == State.BENDING:
            self._tick_bending()

        # Complex-mode angle-control path. Basic mode writes its own angles in
        # _tick_basic_elongating() and must not double-write via
        # _update_tendon_angles(), which uses Complex-mode slack/bend formulas.
        # Also skipped while waiting/idle - nothing is moving, so there's no
        # reason to spam SERVO commands at the firmware (continuous-rotation
        # servos drift under repeated non-center commands).
        if (
            self.mode == ExperimentMode.COMPLEX
            and self.state in (State.ELONGATING, State.BENDING)
            and self._servo_defaults is not None
            and self._psi_baseline is not None
        ):
            self._update_tendon_angles()

    def _tick_elongating(self) -> None:
        s = self.backend.read_state()
        elapsed = time.monotonic() - self._phase_start
        # Hold in ELONGATING for at least MIN_ELONGATION_S so inflation and
        # tendon slack pay-out are visible and physically complete.
        if elapsed < self.MIN_ELONGATION_S:
            return
        # Length during ELONGATING is derived from tendon servo angles + pulley
        # radius; IMU is not consulted. On sim/tests where slack-comp isn't
        # armed, fall back to the backend's own reading.
        L_current = self._current_length_estimate_mm()
        if hasattr(self.backend, "set_total_length_mm"):
            self.backend.set_total_length_mm(L_current)
        # Don't transition to BENDING until the synchronized inflation ramp
        # has finished — pressure_converged otherwise trips against the
        # slowly-advancing ramp-point setpoint (not the final target), which
        # lets BENDING kick in with modules at ~20% inflation and swings the
        # tendon servos antagonistically instead of letting them unwind
        # together to match the already-inflating modules.
        if self._inflation_progress < 1.0:
            return
        length_converged = abs(L_current - self._L_target) < 5.0
        pressure_converged = bool(self._inflation_final) and all(
            abs(s.module_pressures_psi.get(mid, 0.0) - final) < self.TOL_PRESSURE_PSI
            for mid, final in self._inflation_final.items()
        )
        if length_converged or pressure_converged:
            self._phase_start = time.monotonic()
            if hasattr(self.backend, "set_orientation_target"):
                target_pitch = self._theta_target * math.cos(self._phi_target)
                target_roll = self._theta_target * math.sin(self._phi_target)
                self.backend.set_orientation_target(target_pitch, target_roll, 0.0)
            self.state = State.BENDING

    def _tick_basic_elongating(self, dt: float) -> None:
        """Basic-mode elongation tick: P-controller on max-psi -> tendon unwind."""
        pressures = self.backend.read_state().module_pressures_psi
        max_psi = max(pressures.values()) if pressures else 0.0
        err = max(0.0, max_psi - self._basic_psi_threshold)
        scale = self._basic_speed_scale
        rate = min(
            self.BASIC_UNWIND_KP_DEG_PER_S_PER_PSI * scale * err,
            self.BASIC_UNWIND_MAX_DEG_PER_S * scale,
        )
        self._basic_slack_deg -= rate * dt
        if self._servo_defaults is not None:
            for sid in (1, 2, 3, 4):
                default = self._servo_defaults.get(sid, 0.0)
                angle = default + self._basic_slack_deg
                self.last_tendon_angles[sid] = angle
                self.backend.set_tendon_angle(sid, angle)
        self._basic_elongation_mm = (
            math.radians(-self._basic_slack_deg) * self.PULLEY_RADIUS_MM
        )
        if self._basic_elongation_mm >= self._basic_z_target_mm:
            # Tag the target-reach moment with REACHED so it's visible in
            # CSV phase rows + UI before RETRACTING begins. Result is
            # latched here (elongation-phase elapsed, no pose error in
            # basic mode). The REACHED hold runs for BASIC_REACHED_HOLD_S;
            # RETRACTING takes over in _tick_basic_reached_hold.
            elapsed = time.monotonic() - self._phase_start
            self._last_result = RunResult(
                final_pitch_err_rad=0.0,
                final_position_err_mm=0.0,
                elapsed_s=elapsed,
                timed_out=False,
            )
            self._basic_reached_hold_elapsed_s = 0.0
            self.state = State.REACHED

    def basic_elongation_mm(self) -> float:
        """Current integrated elongation in mm (Basic mode only)."""
        return self._basic_elongation_mm

    def _tick_basic_reached_hold(self, dt: float) -> None:
        """Count down the brief REACHED hold after basic-mode target-reach,
        then begin RETRACTING. Pressures and servos are not rewritten here
        (REACHED carries the no-SERVO-writes convention); the firmware holds
        the last commanded position."""
        self._basic_reached_hold_elapsed_s += dt
        if self._basic_reached_hold_elapsed_s >= self.BASIC_REACHED_HOLD_S:
            self._phase_start = time.monotonic()
            self._basic_retract_elapsed_s = 0.0
            self.state = State.RETRACTING

    def _tick_basic_retracting(self, dt: float) -> None:
        """Reverse the basic-mode motion: wind servos back up at the set rate
        while modules deflate toward baseline. Terminal state is REACHED
        (baseline).

        Safety: rate eases in from zero over BASIC_RETRACT_RAMP_S so there's
        no sudden step at the ELONGATING->RETRACTING transition, and any
        single-tick slack change is clamped to BASIC_RETRACT_MAX_STEP_DEG
        regardless of speed_scale. Ramp uses accumulated sim dt, not wall
        clock, so it works correctly under tests and real-time alike."""
        self._basic_retract_elapsed_s += dt
        if self.BASIC_RETRACT_RAMP_S > 0:
            ease = min(1.0, self._basic_retract_elapsed_s / self.BASIC_RETRACT_RAMP_S)
        else:
            ease = 1.0
        target_rate = self.BASIC_UNWIND_MAX_DEG_PER_S * self._basic_speed_scale
        rate = target_rate * ease
        step = min(rate * dt, self.BASIC_RETRACT_MAX_STEP_DEG)
        self._basic_slack_deg = min(0.0, self._basic_slack_deg + step)
        # Write composed tendon angles.
        if self._servo_defaults is not None:
            for sid in (1, 2, 3, 4):
                default = self._servo_defaults.get(sid, 0.0)
                angle = default + self._basic_slack_deg
                self.last_tendon_angles[sid] = angle
                self.backend.set_tendon_angle(sid, angle)
        # Drive module pressures back toward their zero-capture baseline. The
        # firmware PID deflates at its natural rate to match.
        baseline = self._psi_baseline or {}
        for mid in range(1, 7):
            if mid in self.EXCLUDED_MODULES:
                continue
            target = baseline.get(mid, 0.0)
            self._pressure_setpoints[mid] = target
            self.backend.set_module_pressure(mid, target)
        # Elongation tracks the (shrinking) slack magnitude.
        self._basic_elongation_mm = (
            math.radians(-self._basic_slack_deg) * self.PULLEY_RADIUS_MM
        )
        if self._basic_slack_deg >= 0.0:
            elapsed = time.monotonic() - self._phase_start
            self._last_result = RunResult(
                final_pitch_err_rad=0.0,
                final_position_err_mm=0.0,
                elapsed_s=elapsed,
                timed_out=False,
            )
            self.state = State.REACHED

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
        # Freeze pressures at their current readings so the firmware PID
        # stops driving toward the length-phase setpoint (which would
        # otherwise keep the steppers pumping past the reached pose).
        self._hold_pressures_at_current()
        self.state = State.TIMED_OUT if timed_out else State.REACHED

    def _tick_bending(self) -> None:
        s = self.backend.read_state()
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

    def _compute_module_pressure_targets(self, L_total: float) -> Dict[int, float]:
        """Return the final {module_id: psi} targets for a reach of L_total mm.

        The controller treats pressure as stiffness (bending comes from tendons),
        so all active modules inflate to the same length during ELONGATING. If
        per-module calibration is present, solve its quadratic for the target
        arc length. Otherwise, linearly interpolate between baseline (at L_rest)
        and the operator's safety ceiling (at L_max) based on L_total — so a
        tiny reach commands a small pressure, not full ceiling inflation.
        """
        if self.cal.modules:
            active_ids = [mid for mid in self.cal.modules.keys() if mid not in self.EXCLUDED_MODULES]
        else:
            active_ids = [mid for mid in range(1, 7) if mid not in self.EXCLUDED_MODULES]
        per_module = L_total / max(1, len(active_ids))
        L_rest = self.cal.L_rest
        L_max = self.cal.L_max
        span = L_max - L_rest
        frac = max(0.0, min(1.0, (L_total - L_rest) / span)) if span > 0 else 0.0
        targets: Dict[int, float] = {}
        for mid in active_ids:
            baseline = (self._psi_baseline or {}).get(mid, 0.0)
            if self.cal.modules and mid in self.cal.modules:
                m = self.cal.modules[mid]
                a, b, c = m["coeffs"]
                psi = _solve_psi_for_length(a, b, c, per_module, m["max_psi"])
            elif self._pressure_ceiling_psi is not None:
                psi = baseline + frac * (self._pressure_ceiling_psi - baseline)
            else:
                psi = baseline
            psi = max(psi, baseline)
            if self._pressure_ceiling_psi is not None:
                psi = min(psi, self._pressure_ceiling_psi)
            targets[mid] = psi
        return targets

    def _advance_inflation_ramp(self, dt: float) -> None:
        """Walk all modules' commanded pressures toward their finals together.

        Each module's commanded psi = baseline + progress × (final − baseline),
        where `progress` is a single shared [0, 1] scalar that rises at
        INFLATION_RATE_PER_S. The ramp is open-loop on purpose — gating on
        actual pressure caused the frontier to stall for ~20 s at startup
        when firmware PID deadband filtered out the initial tiny ramp deltas.
        The firmware PID still hunts each module independently; the ramp
        just keeps their setpoints in lockstep and their slack in lockstep.
        """
        if not self._inflation_final or self._inflation_progress >= 1.0:
            return
        baseline = self._psi_baseline or {}
        self._inflation_progress = min(
            1.0, self._inflation_progress + self.INFLATION_RATE_PER_S * dt
        )
        for mid, final in self._inflation_final.items():
            base = baseline.get(mid, 0.0)
            commanded = base + self._inflation_progress * (final - base)
            self._pressure_setpoints[mid] = commanded
            self.backend.set_module_pressure(mid, commanded)

    def _hold_pressures_at_current(self) -> None:
        """Freeze each module's setpoint at its present reading so the firmware
        PID holds position instead of continuing to drive toward the last
        commanded (length-target) setpoint once the pose is reached."""
        pressures = self.backend.read_state().module_pressures_psi
        for mid, last_sp in list(self._pressure_setpoints.items()):
            cur = pressures.get(mid, last_sp)
            self._pressure_setpoints[mid] = cur
            self.backend.set_module_pressure(mid, cur)

    # --- Angle-control path -----------------------------------------------

    def _current_length_estimate_mm(self) -> float:
        """Return total arm length estimate in mm.

        Live path (servo_defaults latched): derive from commanded slack —
        ΔL_arm = −slack_deg × π/180 × PULLEY_RADIUS_MM. Drift-free and
        IMU-free.

        Fallback (no servo defaults, e.g. sim/tests): read whatever the
        backend reports in BackendState.total_length_mm.
        """
        if self._servo_defaults is None:
            return self.backend.read_state().total_length_mm
        slack_deg = self._compute_slack_deg()
        elongation_mm = -math.radians(slack_deg) * self.PULLEY_RADIUS_MM
        return self.cal.L_rest + elongation_mm

    def current_slack_deg(self) -> float:
        """Public read of the current common-mode slack (degrees)."""
        return self._compute_slack_deg()

    def pressure_sum_delta_psi(self) -> float:
        """Σ max(0, psi_m − baseline_m) across modules, in psi.

        Proxy for how much inflation energy is in the arm right now. Useful
        for correlating camera-measured physical z against sensor state
        when no per-module length calibration is available.
        """
        if self._psi_baseline is None:
            return 0.0
        pressures = self.backend.read_state().module_pressures_psi
        total = 0.0
        for mid, base in self._psi_baseline.items():
            cur = pressures.get(mid, base)
            total += max(0.0, cur - base)
        return total

    def _compute_slack_deg(self) -> float:
        """Common-mode tendon slack (negative = pay out), driven by the
        synchronized inflation ramp progress.

        Slack = −SLACK_MAX_DEG × progress. Monotonic by construction:
        progress only ever increases during ELONGATING, so slack only
        becomes more negative. This replaces the previous sensor-driven
        formulation, which oscillated (and occasionally wound the servos
        UP) as noisy per-module pressure readings moved the aggregate
        Δpsi sum around. All 4 tendon servos share this one scalar, so
        they unwind together at the same rate regardless of their per-
        servo default angles.
        """
        return -self.SLACK_MAX_DEG * self._inflation_progress

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
