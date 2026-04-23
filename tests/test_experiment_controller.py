"""Tests for experiment_controller.ExperimentController state machine."""
import math
import time
import pytest
from experiment_backend import SimBackend
from experiment_controller import ExperimentController, State
from length_calibration import LengthCalibration


@pytest.fixture
def cal_default():
    return LengthCalibration(is_default=True)


@pytest.fixture
def ctrl(cal_default):
    be = SimBackend(L_rest=cal_default.L_rest, num_modules=6, rng_seed=0)
    return ExperimentController(backend=be, calibration=cal_default)


def test_initial_state_is_IDLE(ctrl):
    assert ctrl.state == State.IDLE


def test_controller_defaults_to_complex_mode(ctrl):
    from experiment_controller import ExperimentMode
    assert ctrl.mode == ExperimentMode.COMPLEX


def test_controller_mode_can_be_set_to_basic(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    assert ctrl.mode == ExperimentMode.BASIC_ELONGATION


def test_confirm_zero_latches_servo_defaults(ctrl):
    ctrl.start_zeroing()
    defaults = {1: 30.0, 2: 40.0, 3: 50.0, 4: 60.0}
    ctrl.confirm_zero(servo_defaults=defaults)
    assert ctrl._servo_defaults == defaults


def test_confirm_zero_without_servo_defaults_leaves_them_none(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    assert ctrl._servo_defaults is None


def test_reach_basic_requires_waiting_state(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_requires_basic_mode(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_requires_servo_defaults(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_transitions_to_elongating(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)
    assert ctrl.state == State.ELONGATING
    assert ctrl._basic_z_target_mm == 30.0
    assert ctrl._basic_psi_threshold == 15.0
    assert ctrl._basic_slack_deg == 0.0


def test_basic_unwind_zero_when_under_threshold(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    # Low ceiling -> all modules start under threshold.
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0, pressure_ceiling_psi=10.0)
    for _ in range(10):
        ctrl.tick(dt=0.1)
    assert ctrl._basic_slack_deg == 0.0


def test_basic_unwind_proportional_to_overshoot(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0, pressure_ceiling_psi=10.0)
    ctrl.backend._pressures[1] = 16.0
    ctrl.tick(dt=0.1)
    # kp=10 deg/s/psi, err=1 psi, dt=0.1 -> slack -= 1.0 deg.
    assert ctrl._basic_slack_deg == pytest.approx(-1.0, abs=1e-6)


def test_basic_unwind_clamped_at_max_rate(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 25.0  # +10 psi -> kp*10=100, clamped to 30.
    ctrl.tick(dt=0.1)
    assert ctrl._basic_slack_deg == pytest.approx(-3.0, abs=1e-6)


def test_basic_unwind_gated_on_max_psi_across_all_balloons(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0)
    for mid in range(1, 7):
        ctrl.backend._pressures[mid] = 5.0
    ctrl.backend._pressures[5] = 17.0  # +2 overshoot
    ctrl.tick(dt=0.1)
    assert ctrl._basic_slack_deg == pytest.approx(-2.0, abs=1e-6)


def test_basic_elongation_derived_from_slack_and_pulley_radius(ctrl):
    from experiment_controller import ExperimentMode, ExperimentController
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0, pressure_ceiling_psi=10.0)
    ctrl.backend._pressures[1] = 16.0  # +1 psi overshoot -> 10 deg/s unwind
    for _ in range(5):
        ctrl.tick(dt=0.1)
    # 5 ticks * 0.1s * 10 deg/s = 5 deg slack
    expected_mm = math.radians(5.0) * ExperimentController.PULLEY_RADIUS_MM
    assert ctrl._basic_elongation_mm == pytest.approx(expected_mm, rel=1e-6)


def test_basic_elongation_mm_accessor(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0)
    assert ctrl.basic_elongation_mm() == 0.0


def test_basic_reaches_when_elongation_hits_target(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    z_target = 5.0
    ctrl.reach_basic(z_target_mm=z_target, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 25.0  # overshoot -> clamped 30 deg/s
    peak_elongation = 0.0
    for _ in range(200):
        ctrl.tick(dt=0.05)
        peak_elongation = max(peak_elongation, ctrl._basic_elongation_mm)
        if ctrl.state == State.REACHED:
            break
    # After retraction completes, state = REACHED; peak elongation hit the target.
    assert ctrl.state == State.REACHED
    assert peak_elongation >= z_target


def test_basic_no_bending_state_ever_reached(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=5.0, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 25.0
    for _ in range(200):
        ctrl.tick(dt=0.05)
        assert ctrl.state != State.BENDING
        if ctrl.state == State.REACHED:
            break


def test_basic_reach_commands_flat_pressure_setpoint(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(
        z_target_mm=30.0, psi_threshold=15.0, pressure_ceiling_psi=20.0
    )
    for mid in range(1, 7):
        assert ctrl.backend._pressures[mid] == 20.0


def test_basic_reach_falls_back_to_threshold_plus_5_if_no_ceiling(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)
    for mid in range(1, 7):
        assert ctrl.backend._pressures[mid] == 20.0


def test_basic_speed_scale_multiplies_unwind_rate(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(
        z_target_mm=1000.0, psi_threshold=15.0,
        pressure_ceiling_psi=10.0, speed_scale=2.5,
    )
    ctrl.backend._pressures[1] = 16.0  # +1 psi overshoot
    ctrl.tick(dt=0.1)
    # kp=10 * 2.5 = 25 deg/s/psi, err=1 psi, dt=0.1 -> slack -= 2.5 deg.
    assert ctrl._basic_slack_deg == pytest.approx(-2.5, abs=1e-6)


def test_basic_speed_scale_also_raises_max_rate_clamp(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(
        z_target_mm=1000.0, psi_threshold=15.0,
        pressure_ceiling_psi=10.0, speed_scale=2.0,
    )
    ctrl.backend._pressures[1] = 25.0  # +10 psi overshoot
    ctrl.tick(dt=0.1)
    # Unclamped: kp*scale*err = 10*2*10 = 200. Clamp: max*scale = 30*2 = 60.
    # slack -= 60 * 0.1 = 6.0
    assert ctrl._basic_slack_deg == pytest.approx(-6.0, abs=1e-6)


def test_basic_reach_rejects_nonpositive_speed_scale(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0, speed_scale=0.0)


def test_basic_target_reached_transitions_to_retracting(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=3.0, psi_threshold=15.0, pressure_ceiling_psi=20.0)
    for _ in range(100):
        ctrl.tick(dt=0.05)
        if ctrl.state == State.RETRACTING:
            break
    assert ctrl.state == State.RETRACTING
    assert ctrl._basic_elongation_mm >= 3.0


def test_basic_retracting_winds_slack_back_up_at_set_rate(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0, speed_scale=1.0)
    # Force into RETRACTING with a known starting slack.
    ctrl._basic_slack_deg = -20.0
    ctrl.state = State.RETRACTING
    ctrl.tick(dt=0.1)
    # rate = 30 deg/s * 1.0 * 0.1s = 3.0 deg
    assert ctrl._basic_slack_deg == pytest.approx(-17.0, abs=1e-6)


def test_basic_retracting_honors_speed_scale(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0, speed_scale=2.0)
    ctrl._basic_slack_deg = -20.0
    ctrl.state = State.RETRACTING
    ctrl.tick(dt=0.1)
    # rate = 30 * 2.0 * 0.1 = 6.0 deg
    assert ctrl._basic_slack_deg == pytest.approx(-14.0, abs=1e-6)


def test_basic_retracting_clamps_slack_at_zero(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0)
    ctrl._basic_slack_deg = -1.0
    ctrl.state = State.RETRACTING
    ctrl.tick(dt=1.0)  # plenty of time to overshoot
    assert ctrl._basic_slack_deg == 0.0
    assert ctrl.state == State.REACHED


def test_basic_retracting_commands_deflation_to_baseline(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    # Make the baseline nontrivial so we can verify it's used as the target.
    ctrl._psi_baseline = {mid: 0.5 for mid in range(1, 7)}
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0, pressure_ceiling_psi=20.0)
    ctrl._basic_slack_deg = -10.0
    ctrl.state = State.RETRACTING
    ctrl.tick(dt=0.1)
    for mid in range(1, 7):
        assert ctrl.backend._pressures[mid] == pytest.approx(0.5, abs=1e-6)


def test_basic_full_run_reaches_then_retracts_then_reached(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(
        z_target_mm=3.0, psi_threshold=15.0,
        pressure_ceiling_psi=20.0, speed_scale=2.0,
    )
    saw_retracting = False
    for _ in range(500):
        ctrl.tick(dt=0.05)
        if ctrl.state == State.RETRACTING:
            saw_retracting = True
        if ctrl.state == State.REACHED:
            break
    assert saw_retracting
    assert ctrl.state == State.REACHED
    assert ctrl._basic_slack_deg == 0.0


def test_start_zeroing_transitions_to_ZEROING(ctrl):
    ctrl.start_zeroing()
    assert ctrl.state == State.ZEROING


def test_confirm_zero_captures_and_transitions_to_WAITING(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    assert ctrl.state == State.WAITING_FOR_TARGET


def test_estop_from_any_state_returns_to_IDLE(ctrl):
    ctrl.start_zeroing()
    ctrl.emergency_stop()
    assert ctrl.state == State.IDLE


def test_reach_requires_WAITING_state(ctrl):
    # Still in IDLE — reach should be a no-op.
    ctrl.reach(target=(0.0, 0.0, 300.0))
    assert ctrl.state == State.IDLE


def test_reach_transitions_to_ELONGATING_with_valid_target(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))  # reachable straight-up
    assert ctrl.state == State.ELONGATING


def test_reach_rejects_unreachable_target(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    # Far-out target that needs theta > 60 deg.
    with pytest.raises(ValueError):
        ctrl.reach(target=(10000.0, 0.0, 10.0))
    assert ctrl.state == State.WAITING_FOR_TARGET


def test_elongating_transitions_to_bending_when_length_reached(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))
    # Advance enough ticks for SimBackend to reach the length.
    for _ in range(200):
        ctrl.tick(dt=0.05)
        if ctrl.state == State.BENDING:
            break
    assert ctrl.state == State.BENDING


def test_elongating_times_out(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))
    # Force a timeout by stubbing the clock.
    ctrl._phase_start = time.monotonic() - (ctrl.ELONGATION_TIMEOUT_S + 1)
    ctrl.tick(dt=0.05)
    assert ctrl.state == State.TIMED_OUT


def test_bending_reaches_on_tolerance(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))  # straight up — theta_target = 0
    # Straight-up target has zero orientation error; after elongation finishes,
    # one bending tick should declare REACHED.
    for _ in range(400):
        ctrl.tick(dt=0.05)
        if ctrl.state in (State.REACHED, State.TIMED_OUT):
            break
    assert ctrl.state == State.REACHED
    assert ctrl.last_result is not None
    assert ctrl.last_result.timed_out is False


def test_bending_commands_tendons_for_off_axis_target(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    # Target that needs pitch bending.
    ctrl.reach(target=(100.0, 0.0, 260.0))
    for _ in range(400):
        ctrl.tick(dt=0.05)
        if ctrl.state in (State.REACHED, State.TIMED_OUT):
            break
    # SimBackend slews to match; should converge within budget.
    assert ctrl.state == State.REACHED


def test_rezero_requires_near_rest_pressures(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    # Manually pump a module's pressure in the sim backend.
    ctrl.backend._pressures[1] = 5.0
    assert ctrl.can_rezero() is False


def test_rezero_allowed_when_at_rest(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    # Defaults: all pressures 0, backend at rest.
    assert ctrl.can_rezero() is True


def test_rezero_captures_new_reference(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    assert ctrl.rezero() is True  # allowed at rest


def test_bending_aborts_on_stale_IMU_in_live_mode():
    """If read_state().imu_fresh is False during BENDING, controller must halt."""
    from experiment_backend import BackendState
    from length_calibration import LengthCalibration

    class StaleBackend:
        is_sim = False

        def __init__(self):
            self._p = {i + 1: 0.0 for i in range(6)}
            self.sent_stops = 0

        def read_state(self):
            return BackendState(
                total_length_mm=300.0, module_pressures_psi=dict(self._p),
                pitch=0.0, roll=0.0, yaw=0.0, imu_fresh=False,
            )

        def read_orientation(self):
            return (0.0, 0.0, 0.0)

        def set_module_pressure(self, mid, psi):
            self._p[mid] = psi

        def set_tendon_rate(self, sid, rate):
            pass

        def capture_zero(self):
            pass

        def emergency_stop(self):
            self.sent_stops += 1

        def tick(self, dt):
            pass

        def set_total_length_target(self, L):
            pass

        def set_orientation_target(self, p, r, y):
            pass

    be = StaleBackend()
    cal = LengthCalibration(is_default=True)
    ctrl = ExperimentController(backend=be, calibration=cal)
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))
    # Force into BENDING so the staleness check applies there.
    ctrl._phase_start = time.monotonic()
    ctrl.state = State.BENDING
    ctrl._target = (0.0, 0.0, 300.0)
    ctrl.tick(dt=0.1)
    assert ctrl.state == State.IDLE
    assert be.sent_stops >= 1


def test_reached_aborts_on_stale_IMU_in_live_mode():
    """Stale IMU while holding pose in REACHED must also fire E-stop."""
    from experiment_backend import BackendState
    from length_calibration import LengthCalibration

    class StaleBackend:
        is_sim = False

        def __init__(self):
            self._p = {i + 1: 0.0 for i in range(6)}
            self.sent_stops = 0
            self._fresh = True

        def read_state(self):
            return BackendState(
                total_length_mm=300.0, module_pressures_psi=dict(self._p),
                pitch=0.0, roll=0.0, yaw=0.0, imu_fresh=self._fresh,
            )

        def read_orientation(self):
            return (0.0, 0.0, 0.0)

        def set_module_pressure(self, mid, psi):
            self._p[mid] = psi

        def set_tendon_rate(self, sid, rate):
            pass

        def capture_zero(self):
            pass

        def emergency_stop(self):
            self.sent_stops += 1

        def tick(self, dt):
            pass

        def set_total_length_target(self, L):
            pass

        def set_orientation_target(self, p, r, y):
            pass

    be = StaleBackend()
    cal = LengthCalibration(is_default=True)
    ctrl = ExperimentController(backend=be, calibration=cal)
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 100.0))
    # Force into REACHED (pose hold) and then drop IMU freshness.
    ctrl.state = State.REACHED
    ctrl._target = (0.0, 0.0, 100.0)
    be._fresh = False
    ctrl.tick(dt=0.1)
    assert ctrl.state == State.IDLE
    assert be.sent_stops >= 1
