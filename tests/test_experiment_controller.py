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
