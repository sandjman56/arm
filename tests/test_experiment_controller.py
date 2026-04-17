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
