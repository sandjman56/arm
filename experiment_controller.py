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
