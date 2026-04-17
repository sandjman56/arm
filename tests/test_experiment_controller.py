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
