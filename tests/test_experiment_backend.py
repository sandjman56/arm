"""Tests for Backend implementations."""
import math
import pytest
from experiment_backend import SimBackend, BackendState


def test_sim_backend_starts_at_rest():
    # rng_seed makes the noise deterministic; internal state must be exactly at rest.
    be = SimBackend(L_rest=240.0, num_modules=6, rng_seed=0)
    assert be._length_mm == pytest.approx(240.0, abs=1e-9)
    p, r, y = be.read_orientation()  # noise-free accessor
    assert (p, r, y) == (0.0, 0.0, 0.0)
    # Noisy read stays within a few sigma of rest.
    s = be.read_state()
    assert s.total_length_mm == pytest.approx(240.0, abs=2.0)  # ~6 sigma of 0.3mm noise
    assert abs(s.pitch) < math.radians(1)
    assert abs(s.roll) < math.radians(1)


def test_sim_backend_advances_length_toward_setpoint():
    be = SimBackend(L_rest=240.0, num_modules=6, rng_seed=0)
    be.set_total_length_target(360.0)
    # Advance several ticks of 50ms; SimBackend slews at a known rate.
    for _ in range(40):
        be.tick(dt=0.05)
    assert be.read_state().total_length_mm == pytest.approx(360.0, abs=5.0)


def test_sim_backend_advances_orientation_toward_target():
    be = SimBackend(L_rest=240.0, num_modules=6, rng_seed=0)
    # Command 30 deg pitch in +X bend plane.
    be.set_orientation_target(target_pitch=math.radians(30), target_roll=0.0, target_yaw=0.0)
    for _ in range(60):
        be.tick(dt=0.05)
    p, r, y = be.read_orientation()
    assert p == pytest.approx(math.radians(30), abs=math.radians(2))


def test_sim_backend_estop_halts_advance():
    be = SimBackend(L_rest=240.0, num_modules=6, rng_seed=0)
    be.set_total_length_target(400.0)
    be.tick(dt=0.05)
    be.emergency_stop()
    # After e-stop, the underlying length should not change further.
    # (read_state() adds Gaussian noise, so compare the internal state directly.)
    before = be._length_mm
    for _ in range(20):
        be.tick(dt=0.05)
    assert be._length_mm == pytest.approx(before, abs=1e-9)
