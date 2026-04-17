"""Backends for the Experiments-mode controller.

Two implementations:
- LiveBackend: talks to the real rig via serial, reads real IMU/pressure.
- SimBackend:  no serial I/O; advances a synthetic internal state each tick.

The controller is backend-agnostic: it calls `set_module_pressure()`,
`set_tendon_rate()`, reads `read_state()` / `read_orientation()`, etc.
"""
from __future__ import annotations
import math
import random
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Dict, Optional


@dataclass
class BackendState:
    total_length_mm: float
    module_pressures_psi: Dict[int, float]  # module_id -> current psi
    pitch: float   # radians from zero
    roll: float
    yaw: float
    imu_fresh: bool  # True if IMU telemetry has been received recently


class Backend(ABC):
    @abstractmethod
    def read_state(self) -> BackendState: ...

    @abstractmethod
    def read_orientation(self) -> tuple[float, float, float]:
        """(pitch, roll, yaw) in radians, all relative to the last zero capture."""

    @abstractmethod
    def set_module_pressure(self, module_id: int, target_psi: float) -> None: ...

    @abstractmethod
    def set_tendon_rate(self, servo_id: int, rate: float) -> None:
        """Command a continuous-rotation servo. rate in [-1, 1]."""

    @abstractmethod
    def capture_zero(self) -> None: ...

    @abstractmethod
    def emergency_stop(self) -> None: ...

    @abstractmethod
    def tick(self, dt: float) -> None:
        """Advance internal state (sim backend) or process serial I/O (live)."""

    @property
    @abstractmethod
    def is_sim(self) -> bool: ...


class SimBackend(Backend):
    """Synthetic backend for offline use.

    - Total length slews toward `set_total_length_target()` at LENGTH_RATE mm/s.
    - Orientation slews toward `set_orientation_target()` at ORIENT_RATE rad/s.
    - Per-module pressure is tracked (modeled trivially as instantaneous).
    - Small Gaussian noise is layered on so readings look realistic.
    """
    LENGTH_RATE = 80.0          # mm/s
    ORIENT_RATE = math.radians(20)  # rad/s
    NOISE_LEN_MM = 0.3
    NOISE_ORIENT_RAD = math.radians(0.1)

    def __init__(self, L_rest: float, num_modules: int, rng_seed: Optional[int] = None):
        self._L_rest = L_rest
        self._num_modules = num_modules
        self._length_mm = L_rest
        self._length_target = L_rest
        self._pressures: Dict[int, float] = {i + 1: 0.0 for i in range(num_modules)}
        # Orientation state (absolute, pre-zero-offset).
        self._pitch = 0.0
        self._roll = 0.0
        self._yaw = 0.0
        self._target_pitch = 0.0
        self._target_roll = 0.0
        self._target_yaw = 0.0
        # Zero offsets.
        self._zero_pitch = 0.0
        self._zero_roll = 0.0
        self._zero_yaw = 0.0
        self._halted = False
        self._rng = random.Random(rng_seed)

    @property
    def is_sim(self) -> bool:
        return True

    def set_total_length_target(self, L_mm: float) -> None:
        self._length_target = L_mm

    def set_orientation_target(self, target_pitch: float, target_roll: float, target_yaw: float) -> None:
        self._target_pitch = target_pitch
        self._target_roll = target_roll
        self._target_yaw = target_yaw

    def set_module_pressure(self, module_id: int, target_psi: float) -> None:
        # Pressure is instantaneous in sim. The controller drives length via
        # set_total_length_target() directly.
        self._pressures[module_id] = target_psi

    def set_tendon_rate(self, servo_id: int, rate: float) -> None:
        # Interpret tendon pulls as orientation intent: servo 1 (0 deg) pulls
        # pitch+ ; servo 3 (180) pulls pitch-; servo 2 (90) pulls roll+;
        # servo 4 (270) pulls roll-. Caller gives rate in [-1, 1].
        # This is just for the sim to look plausible; real mapping is elsewhere.
        scale = math.radians(15)
        if servo_id == 1:
            self._target_pitch += rate * scale * 0.05
        elif servo_id == 2:
            self._target_roll += rate * scale * 0.05
        elif servo_id == 3:
            self._target_pitch -= rate * scale * 0.05
        elif servo_id == 4:
            self._target_roll -= rate * scale * 0.05

    def capture_zero(self) -> None:
        self._zero_pitch = self._pitch
        self._zero_roll = self._roll
        self._zero_yaw = self._yaw

    def emergency_stop(self) -> None:
        self._halted = True
        self._length_target = self._length_mm
        self._target_pitch = self._pitch
        self._target_roll = self._roll
        self._target_yaw = self._yaw

    def tick(self, dt: float) -> None:
        if self._halted:
            return
        # Slew length.
        self._length_mm = _slew(self._length_mm, self._length_target, self.LENGTH_RATE * dt)
        # Slew orientation.
        self._pitch = _slew(self._pitch, self._target_pitch, self.ORIENT_RATE * dt)
        self._roll = _slew(self._roll, self._target_roll, self.ORIENT_RATE * dt)
        self._yaw = _slew(self._yaw, self._target_yaw, self.ORIENT_RATE * dt)

    def read_state(self) -> BackendState:
        return BackendState(
            total_length_mm=self._length_mm + self._rng.gauss(0, self.NOISE_LEN_MM),
            module_pressures_psi=dict(self._pressures),
            pitch=self._pitch - self._zero_pitch + self._rng.gauss(0, self.NOISE_ORIENT_RAD),
            roll=self._roll - self._zero_roll + self._rng.gauss(0, self.NOISE_ORIENT_RAD),
            yaw=self._yaw - self._zero_yaw + self._rng.gauss(0, self.NOISE_ORIENT_RAD),
            imu_fresh=True,
        )

    def read_orientation(self) -> tuple[float, float, float]:
        return (
            self._pitch - self._zero_pitch,
            self._roll - self._zero_roll,
            self._yaw - self._zero_yaw,
        )


def _slew(current: float, target: float, max_step: float) -> float:
    """Move `current` toward `target` by at most `max_step`."""
    diff = target - current
    if abs(diff) <= max_step:
        return target
    return current + math.copysign(max_step, diff)
