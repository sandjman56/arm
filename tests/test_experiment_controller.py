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
