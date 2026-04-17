# Experiments Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a fourth controller mode ("Experiments") to the Continuum Arm Controller that lets the operator zero the trunk at rest, pick a target in 3D, and drive the arm to it via a two-phase controller (inflate to length via existing PID, then closed-loop bend on IMU orientation via 4 antagonistic tendon servos). The panel is fully usable with no serial connection, via a simulation backend.

**Architecture:** Pure-math `kinematics.py` does PCC forward/inverse kinematics. `orientation.py` estimates tip orientation from IMU. `experiment_backend.py` defines a `Backend` interface with `LiveBackend` and `SimBackend` implementations. `experiment_controller.py` holds a backend-agnostic state machine (IDLE → ZEROING → WAITING_FOR_TARGET → ELONGATING → BENDING → REACHED/TIMED_OUT). `panels/experiment_panel.py` is the Tkinter UI (XY picker, XZ picker, 3D matplotlib preview, readouts) and wires up whichever backend matches the current serial state. `app.py` gets a fourth radio button and is generalized to 6 modules.

**Tech Stack:** Python 3, Tkinter, matplotlib (including `Axes3D`), numpy (already transitively installed via matplotlib), pyserial (existing), pytest (new).

**Spec:** [docs/superpowers/specs/2026-04-17-experiments-mode-design.md](../specs/2026-04-17-experiments-mode-design.md)

---

## File Structure

**New files:**

| Path | Responsibility |
|---|---|
| `kinematics.py` | PCC forward/inverse kinematics, reachable-workspace check. No I/O, no UI. |
| `orientation.py` | `OrientationEstimator` — complementary filter: accel → pitch/roll, gyro → yaw. Zero capture. |
| `length_calibration.py` | JSON load/save of per-module `length(psi)` curves. Default workspace when missing. |
| `experiment_backend.py` | `Backend` abstract base + `LiveBackend` (serial) + `SimBackend` (synthetic advance). |
| `experiment_controller.py` | `ExperimentController` state machine + bend-phase P-controller with tendon mapping. |
| `panels/experiment_panel.py` | Main Tk frame — buttons, readouts, wiring, owns the three canvas widgets. |
| `panels/experiment_pickers.py` | `XYPicker` and `XZPicker` matplotlib canvas widgets. |
| `panels/experiment_preview.py` | `TrunkPreview3D` matplotlib 3D canvas (view-only). |
| `panels/experiment_calibration.py` | `LengthCalibrationDialog` — modal for the per-module length calibration flow. |
| `length_calibration.json` | (Runtime) Per-module calibration data, created by the calibration flow. |
| `tests/__init__.py` | Empty marker. |
| `tests/conftest.py` | Shared pytest fixtures. |
| `tests/test_kinematics.py` | Unit tests for kinematics module. |
| `tests/test_orientation.py` | Unit tests for orientation estimator. |
| `tests/test_length_calibration.py` | Unit tests for calibration I/O. |
| `tests/test_experiment_backend.py` | Unit tests for SimBackend and LiveBackend (mocked). |
| `tests/test_experiment_controller.py` | State-machine and control-law tests. |
| `requirements-test.txt` | `pytest`, `pytest-mock`. |
| `pytest.ini` | Pytest config. |

**Modified files:**

| Path | What changes |
|---|---|
| `app.py` | Add "Experiments" radio; instantiate `ExperimentPanel`; extend `switch_mode`; generalize hardcoded 3-module setup (lines 76-78) to support 6; route telemetry + connection state into experiment controller. |
| `panels/__init__.py` | Export `ExperimentPanel`. |
| `arduino_interface.py` | Add `send_set_module(id, hpa)` and `send_tendon(servo_id, rate)` helpers (thin wrappers around `send()`). |

**Firmware (out of scope for this plan, tracked separately):** Arduino protocol extensions for `SET <id> <hPa>` and `TEND <id> <rate>`. The Python side will issue these commands; if the firmware rejects them, `LiveBackend` will log a warning and the user can work in sim mode.

---

## Task 1: Bootstrap test infrastructure

**Files:**
- Create: `requirements-test.txt`
- Create: `pytest.ini`
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`

- [ ] **Step 1: Create `requirements-test.txt`**

```
pytest>=7.0
pytest-mock>=3.10
```

- [ ] **Step 2: Create `pytest.ini`**

```ini
[pytest]
testpaths = tests
python_files = test_*.py
addopts = -v
```

- [ ] **Step 3: Create `tests/__init__.py`** (empty file)

- [ ] **Step 4: Create `tests/conftest.py`**

```python
"""Shared pytest fixtures for the Continuum Arm Controller test suite."""
import sys
from pathlib import Path

# Ensure the project root is on sys.path so tests can import modules directly.
PROJECT_ROOT = Path(__file__).resolve().parent.parent
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))
```

- [ ] **Step 5: Install deps and verify pytest runs**

Run: `pip install -r requirements-test.txt && pytest --collect-only`
Expected: `collected 0 items` (no tests yet, but pytest finds the config).

- [ ] **Step 6: Commit**

```bash
git add requirements-test.txt pytest.ini tests/__init__.py tests/conftest.py
git commit -m "test: bootstrap pytest infrastructure"
```

---

## Task 2: Kinematics — straight-trunk forward kinematics

**Files:**
- Create: `kinematics.py`
- Create: `tests/test_kinematics.py`

- [ ] **Step 1: Write the failing test**

In `tests/test_kinematics.py`:

```python
"""Unit tests for kinematics.py (PCC continuum-arm model)."""
import math
import pytest
from kinematics import forward_kinematics


def test_straight_trunk_tip_is_at_height_L():
    """A trunk with zero tilt should have its tip at (0, 0, L)."""
    tip = forward_kinematics(L=100.0, theta=0.0, phi=0.0)
    assert tip == pytest.approx((0.0, 0.0, 100.0), abs=1e-9)


def test_zero_length_returns_origin():
    tip = forward_kinematics(L=0.0, theta=0.0, phi=0.0)
    assert tip == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_kinematics.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'kinematics'`.

- [ ] **Step 3: Create minimal `kinematics.py`**

```python
"""Piecewise-constant-curvature (PCC) kinematics for the continuum arm.

The trunk is modeled as a single circular arc of arclength L, tilting by
angle theta from the base's +Z axis in a plane at azimuth phi around Z.
"""
from __future__ import annotations

import math
from typing import Tuple

Vec3 = Tuple[float, float, float]


def forward_kinematics(L: float, theta: float, phi: float) -> Vec3:
    """Return tip position (x, y, z) in the physics frame.

    L     : arclength (mm), >= 0
    theta : tilt magnitude (radians), 0 = straight up along +Z
    phi   : azimuth of the bend plane (radians around Z)
    """
    if L < 0:
        raise ValueError(f"L must be non-negative, got {L}")
    # Straight case — avoid division by zero.
    if abs(theta) < 1e-9:
        return (0.0, 0.0, L)
    # General PCC: radius R = L/theta, tip in bend plane at (R*(1-cos t), R*sin t).
    R = L / theta
    r_plane = R * (1.0 - math.cos(theta))
    z = R * math.sin(theta)
    x = r_plane * math.cos(phi)
    y = r_plane * math.sin(phi)
    return (x, y, z)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_kinematics.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add kinematics.py tests/test_kinematics.py
git commit -m "feat(kinematics): forward kinematics for straight trunk"
```

---

## Task 3: Kinematics — bent forward kinematics

**Files:**
- Modify: `tests/test_kinematics.py`

- [ ] **Step 1: Add failing tests for bent cases**

Append to `tests/test_kinematics.py`:

```python
def test_quarter_arc_in_xz_plane():
    """Arc of length L with theta=pi/2 in the XZ plane (phi=0).
    Radius R = L / (pi/2). Tip at (R, 0, R)."""
    L = math.pi / 2  # so R = 1
    tip = forward_kinematics(L=L, theta=math.pi / 2, phi=0.0)
    assert tip == pytest.approx((1.0, 0.0, 1.0), abs=1e-9)


def test_quarter_arc_in_yz_plane():
    """Same arc rotated 90 deg around Z (phi=pi/2): tip at (0, R, R)."""
    L = math.pi / 2
    tip = forward_kinematics(L=L, theta=math.pi / 2, phi=math.pi / 2)
    assert tip == pytest.approx((0.0, 1.0, 1.0), abs=1e-9)


def test_small_theta_is_continuous_with_straight():
    """As theta -> 0, tip should approach (0, 0, L) smoothly."""
    L = 100.0
    tip_tiny = forward_kinematics(L=L, theta=1e-6, phi=0.0)
    assert tip_tiny == pytest.approx((0.0, 0.0, L), abs=1e-3)


def test_negative_L_raises():
    with pytest.raises(ValueError):
        forward_kinematics(L=-1.0, theta=0.0, phi=0.0)
```

- [ ] **Step 2: Run tests to verify the new ones pass**

Run: `pytest tests/test_kinematics.py -v`
Expected: PASS (all 6). The existing implementation already covers these cases.

- [ ] **Step 3: Commit**

```bash
git add tests/test_kinematics.py
git commit -m "test(kinematics): cover bent arc cases + azimuth + small-theta continuity"
```

---

## Task 4: Kinematics — inverse kinematics

**Files:**
- Modify: `kinematics.py`
- Modify: `tests/test_kinematics.py`

- [ ] **Step 1: Write failing tests for inverse kinematics**

Append to `tests/test_kinematics.py`:

```python
from kinematics import inverse_kinematics, Unreachable


def test_inverse_straight_up():
    """Target directly above base recovers L = z, theta = 0."""
    L, theta, phi = inverse_kinematics((0.0, 0.0, 100.0))
    assert L == pytest.approx(100.0, abs=1e-6)
    assert theta == pytest.approx(0.0, abs=1e-6)


def test_inverse_round_trip_in_xz():
    """FK(IK(target)) == target for a reachable point."""
    target = (50.0, 0.0, 80.0)
    L, theta, phi = inverse_kinematics(target)
    tip = forward_kinematics(L, theta, phi)
    assert tip == pytest.approx(target, abs=1e-4)


def test_inverse_round_trip_arbitrary_azimuth():
    target = (30.0, 40.0, 70.0)
    L, theta, phi = inverse_kinematics(target)
    tip = forward_kinematics(L, theta, phi)
    assert tip == pytest.approx(target, abs=1e-4)


def test_inverse_rejects_point_below_base():
    """The arc can't reach z < 0 (modeled trunk only tilts up to 90 deg)."""
    with pytest.raises(Unreachable):
        inverse_kinematics((10.0, 0.0, -5.0))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_kinematics.py -v`
Expected: FAIL (`ImportError: cannot import name 'inverse_kinematics'`).

- [ ] **Step 3: Implement `inverse_kinematics`**

Append to `kinematics.py`:

```python
class Unreachable(ValueError):
    """Raised when a target cannot be reached by any valid (L, theta, phi)."""


def inverse_kinematics(target: Vec3) -> Tuple[float, float, float]:
    """Return (L, theta, phi) such that forward_kinematics(L, theta, phi) == target.

    Raises Unreachable if the target is outside the representable half-space.
    """
    x, y, z = target
    if z < 0:
        raise Unreachable(f"target {target} has z < 0; arc cannot reach below base")

    phi = math.atan2(y, x)
    r = math.hypot(x, y)  # horizontal distance from Z axis

    # Degenerate: target on Z axis => straight trunk
    if r < 1e-9:
        if z <= 0:
            raise Unreachable(f"target {target} on Z axis with z<=0")
        return (z, 0.0, 0.0)

    # Planar geometry: tip at (r, z). For a PCC arc rooted at origin with
    # initial tangent +Z and radius R, the tip lies at (R*(1-cos t), R*sin t).
    # So: r = R*(1-cos t), z = R*sin t => r^2 + z^2 = 2*R*r (from identity
    # (1-cos)^2 + sin^2 = 2(1-cos)), giving R = (r^2 + z^2) / (2r).
    R = (r * r + z * z) / (2.0 * r)
    # theta from z = R*sin(theta), clamp for numerical safety.
    sin_t = max(-1.0, min(1.0, z / R))
    theta = math.asin(sin_t)
    L = R * theta
    return (L, theta, phi)
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_kinematics.py -v`
Expected: PASS (10 tests).

- [ ] **Step 5: Commit**

```bash
git add kinematics.py tests/test_kinematics.py
git commit -m "feat(kinematics): closed-form inverse kinematics with Unreachable exception"
```

---

## Task 5: Kinematics — reachable-workspace check

**Files:**
- Modify: `kinematics.py`
- Modify: `tests/test_kinematics.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_kinematics.py`:

```python
from kinematics import is_reachable


def test_reachable_target_inside_workspace():
    # L_min=100, L_max=300, theta_max=60deg. Point straight up at z=150 is reachable.
    assert is_reachable((0.0, 0.0, 150.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_too_far():
    assert not is_reachable((0.0, 0.0, 1000.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_too_close():
    assert not is_reachable((0.0, 0.0, 50.0), L_min=100, L_max=300, theta_max=math.radians(60))


def test_unreachable_theta_exceeded():
    # A target far out horizontally needs theta > theta_max.
    assert not is_reachable((200.0, 0.0, 10.0), L_min=100, L_max=300, theta_max=math.radians(30))


def test_unreachable_below_base():
    assert not is_reachable((10.0, 0.0, -5.0), L_min=100, L_max=300, theta_max=math.radians(60))
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_kinematics.py -v`
Expected: FAIL (`ImportError: cannot import name 'is_reachable'`).

- [ ] **Step 3: Implement `is_reachable`**

Append to `kinematics.py`:

```python
def is_reachable(target: Vec3, L_min: float, L_max: float, theta_max: float) -> bool:
    """Check whether target is inside the PCC reachable workspace."""
    try:
        L, theta, _phi = inverse_kinematics(target)
    except Unreachable:
        return False
    if not (L_min <= L <= L_max):
        return False
    if not (0.0 <= theta <= theta_max):
        return False
    return True
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_kinematics.py -v`
Expected: PASS (15 tests).

- [ ] **Step 5: Commit**

```bash
git add kinematics.py tests/test_kinematics.py
git commit -m "feat(kinematics): reachable-workspace check against L/theta bounds"
```

---

## Task 6: Orientation estimator — pitch/roll from gravity

**Files:**
- Create: `orientation.py`
- Create: `tests/test_orientation.py`

- [ ] **Step 1: Write failing test**

In `tests/test_orientation.py`:

```python
"""Unit tests for orientation.py (IMU-based tip orientation estimator)."""
import math
import pytest
from orientation import OrientationEstimator


def test_level_accel_gives_zero_pitch_roll():
    """With accelerometer reading (0, 0, 1) g — tip pointing straight up along
    its own body +Z and base +Z aligned — pitch and roll should be 0."""
    est = OrientationEstimator()
    est.update_accel(ax=0.0, ay=0.0, az=1.0)
    assert est.pitch == pytest.approx(0.0, abs=1e-6)
    assert est.roll == pytest.approx(0.0, abs=1e-6)


def test_tilted_forward_gives_nonzero_pitch():
    """Accelerometer reading (sin 30, 0, cos 30) means the tip is tilted
    30 deg around the Y axis (pitch)."""
    est = OrientationEstimator()
    est.update_accel(ax=math.sin(math.radians(30)), ay=0.0, az=math.cos(math.radians(30)))
    assert est.pitch == pytest.approx(math.radians(30), abs=1e-4)
    assert est.roll == pytest.approx(0.0, abs=1e-4)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_orientation.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'orientation'`).

- [ ] **Step 3: Create `orientation.py`**

```python
"""Tip orientation estimator from an MPU-6500 (accel + gyro, no magnetometer).

- Pitch and roll are derived from gravity direction via the accelerometer.
- Yaw is integrated from the gyroscope; it drifts over time (no mag reference).
- `capture_zero()` snapshots the current orientation as the reference so that
  later reads are returned as *deltas* from the zero pose.
"""
from __future__ import annotations
import math
import time
from typing import Optional


class OrientationEstimator:
    def __init__(self) -> None:
        self.pitch: float = 0.0   # radians, around body Y
        self.roll: float = 0.0    # radians, around body X
        self.yaw: float = 0.0     # radians, around body Z (integrated gyro)
        self._last_gyro_t: Optional[float] = None
        # Zero offsets (subtracted when returning "from zero" values).
        self._zero_pitch = 0.0
        self._zero_roll = 0.0
        self._zero_yaw = 0.0

    def update_accel(self, ax: float, ay: float, az: float) -> None:
        """Update pitch and roll from a gravity-direction measurement (units: g)."""
        # Standard small-angle decomposition of gravity vector:
        self.pitch = math.atan2(ax, math.sqrt(ay * ay + az * az))
        self.roll = math.atan2(ay, az) if az != 0.0 else 0.0

    def update_gyro(self, gx: float, gy: float, gz: float, now: Optional[float] = None) -> None:
        """Integrate gyro z-rate into yaw. gx/gy/gz are rad/s."""
        now = now if now is not None else time.monotonic()
        if self._last_gyro_t is None:
            self._last_gyro_t = now
            return
        dt = now - self._last_gyro_t
        self._last_gyro_t = now
        # Simple rectangular integration on yaw only.
        self.yaw += gz * dt

    def capture_zero(self) -> None:
        """Snapshot current orientation as the zero reference."""
        self._zero_pitch = self.pitch
        self._zero_roll = self.roll
        self._zero_yaw = self.yaw

    def from_zero(self) -> tuple[float, float, float]:
        """Return (pitch, roll, yaw) relative to the last capture_zero() call."""
        return (
            self.pitch - self._zero_pitch,
            self.roll - self._zero_roll,
            self.yaw - self._zero_yaw,
        )
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_orientation.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add orientation.py tests/test_orientation.py
git commit -m "feat(orientation): pitch/roll estimator from gravity vector"
```

---

## Task 7: Orientation estimator — gyro-integrated yaw and zero capture

**Files:**
- Modify: `tests/test_orientation.py`

- [ ] **Step 1: Add failing tests**

Append to `tests/test_orientation.py`:

```python
def test_gyro_yaw_integrates():
    """Feeding gz=1 rad/s for 1s should yield yaw ~= 1 rad."""
    est = OrientationEstimator()
    # Seed the integrator with t=0.
    est.update_gyro(gx=0.0, gy=0.0, gz=1.0, now=0.0)
    # Step 1 second forward.
    est.update_gyro(gx=0.0, gy=0.0, gz=1.0, now=1.0)
    assert est.yaw == pytest.approx(1.0, abs=1e-6)


def test_capture_zero_offsets_subsequent_reads():
    est = OrientationEstimator()
    est.pitch = 0.1
    est.roll = 0.2
    est.yaw = 0.3
    est.capture_zero()
    # Still the same absolute values, but from_zero() returns deltas.
    assert est.from_zero() == pytest.approx((0.0, 0.0, 0.0), abs=1e-9)
    est.pitch = 0.15
    assert est.from_zero() == pytest.approx((0.05, 0.0, 0.0), abs=1e-9)


def test_from_zero_without_capture_returns_absolute():
    est = OrientationEstimator()
    est.pitch = 0.5
    assert est.from_zero()[0] == pytest.approx(0.5, abs=1e-9)
```

- [ ] **Step 2: Run tests**

Run: `pytest tests/test_orientation.py -v`
Expected: PASS (5 tests total — implementation already covers these).

- [ ] **Step 3: Commit**

```bash
git add tests/test_orientation.py
git commit -m "test(orientation): cover gyro integration and zero-capture offset"
```

---

## Task 8: Length calibration — file I/O skeleton

**Files:**
- Create: `length_calibration.py`
- Create: `tests/test_length_calibration.py`

- [ ] **Step 1: Write failing test**

In `tests/test_length_calibration.py`:

```python
"""Unit tests for length_calibration.py."""
import json
import pytest
from length_calibration import (
    LengthCalibration,
    load_or_default,
    DEFAULT_WORKSPACE,
)


def test_default_workspace_when_file_missing(tmp_path):
    path = tmp_path / "does_not_exist.json"
    cal = load_or_default(str(path))
    assert cal.is_default is True
    assert cal.L_rest == DEFAULT_WORKSPACE["L_rest_mm"]
    assert cal.L_max == DEFAULT_WORKSPACE["L_max_mm"]


def test_save_load_roundtrip(tmp_path):
    path = tmp_path / "cal.json"
    original = LengthCalibration(
        modules={
            1: {"rest_length_mm": 42.0, "coeffs": [42.0, 3.0, 0.1], "max_psi": 8.0},
            2: {"rest_length_mm": 41.5, "coeffs": [41.5, 3.1, 0.05], "max_psi": 8.0},
        },
        missing_modules=[5, 6],
    )
    original.save(str(path))
    loaded = load_or_default(str(path))
    assert loaded.is_default is False
    assert loaded.modules[1]["rest_length_mm"] == 42.0
    assert loaded.missing_modules == [5, 6]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_length_calibration.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'length_calibration'`).

- [ ] **Step 3: Create `length_calibration.py`**

```python
"""Per-module length-vs-pressure calibration I/O.

Calibration data lives in `length_calibration.json` at the project root.
Each module has a quadratic fit: length_mm(psi) = a + b*psi + c*psi^2.
"""
from __future__ import annotations
import json
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional


DEFAULT_WORKSPACE = {
    "L_rest_mm": 240.0,   # total rest length assuming 6 modules of 40mm
    "L_max_mm": 480.0,    # total max length assuming 6 modules of 80mm
    "theta_max_rad": 1.0472,  # 60 degrees
}


@dataclass
class LengthCalibration:
    modules: Dict[int, dict] = field(default_factory=dict)
    missing_modules: List[int] = field(default_factory=list)
    is_default: bool = False
    calibrated_at: Optional[str] = None

    @property
    def L_rest(self) -> float:
        """Sum of per-module rest lengths, or default if no calibration."""
        if self.is_default or not self.modules:
            return DEFAULT_WORKSPACE["L_rest_mm"]
        return sum(m["rest_length_mm"] for m in self.modules.values())

    @property
    def L_max(self) -> float:
        """Sum of per-module max lengths, or default if no calibration."""
        if self.is_default or not self.modules:
            return DEFAULT_WORKSPACE["L_max_mm"]
        total = 0.0
        for m in self.modules.values():
            a, b, c = m["coeffs"]
            psi = m["max_psi"]
            total += a + b * psi + c * psi * psi
        return total

    def length_at_pressure(self, module_id: int, psi: float) -> float:
        """Return module length in mm at the given pressure (psi)."""
        m = self.modules.get(module_id)
        if m is None:
            return DEFAULT_WORKSPACE["L_rest_mm"] / max(1, len(self.modules) or 6)
        a, b, c = m["coeffs"]
        return a + b * psi + c * psi * psi

    def save(self, path: str) -> None:
        payload = {
            "calibrated_at": self.calibrated_at or datetime.now(timezone.utc).isoformat(),
            # JSON object keys must be strings.
            "modules": {str(k): v for k, v in self.modules.items()},
            "missing_modules": self.missing_modules,
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)


def load_or_default(path: str) -> LengthCalibration:
    """Load calibration from disk, or return a default if the file is missing."""
    if not os.path.exists(path):
        return LengthCalibration(is_default=True)
    try:
        with open(path, "r") as f:
            data = json.load(f)
        return LengthCalibration(
            modules={int(k): v for k, v in data.get("modules", {}).items()},
            missing_modules=list(data.get("missing_modules", [])),
            is_default=False,
            calibrated_at=data.get("calibrated_at"),
        )
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        print(f"[WARN] Could not parse calibration at {path}: {e}; using default.")
        return LengthCalibration(is_default=True)
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_length_calibration.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add length_calibration.py tests/test_length_calibration.py
git commit -m "feat(calibration): length-vs-pressure JSON I/O with default workspace"
```

---

## Task 9: Length calibration — curve fitting from samples

**Files:**
- Modify: `length_calibration.py`
- Modify: `tests/test_length_calibration.py`

- [ ] **Step 1: Add failing test**

Append to `tests/test_length_calibration.py`:

```python
from length_calibration import fit_module_curve


def test_fit_module_curve_recovers_linear_samples():
    # Synthetic: length = 40 + 5*psi, no curvature
    samples = [(0.0, 40.0), (2.0, 50.0), (4.0, 60.0), (6.0, 70.0), (8.0, 80.0)]
    coeffs = fit_module_curve(samples)
    assert coeffs[0] == pytest.approx(40.0, abs=1e-6)
    assert coeffs[1] == pytest.approx(5.0, abs=1e-6)
    assert coeffs[2] == pytest.approx(0.0, abs=1e-6)


def test_fit_module_curve_recovers_quadratic_samples():
    # length = 40 + 3*psi + 0.1*psi^2
    samples = [(psi, 40 + 3 * psi + 0.1 * psi * psi) for psi in (0, 2, 4, 6, 8)]
    coeffs = fit_module_curve(samples)
    assert coeffs[0] == pytest.approx(40.0, abs=1e-6)
    assert coeffs[1] == pytest.approx(3.0, abs=1e-6)
    assert coeffs[2] == pytest.approx(0.1, abs=1e-6)


def test_fit_module_curve_requires_at_least_three_samples():
    with pytest.raises(ValueError):
        fit_module_curve([(0.0, 40.0), (8.0, 80.0)])
```

- [ ] **Step 2: Run tests**

Run: `pytest tests/test_length_calibration.py -v`
Expected: FAIL (`ImportError: cannot import name 'fit_module_curve'`).

- [ ] **Step 3: Implement `fit_module_curve`**

Append to `length_calibration.py`:

```python
def fit_module_curve(samples: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit length = a + b*psi + c*psi^2 by least squares.

    samples: list of (psi, length_mm) pairs. At least 3 required.
    Returns (a, b, c).
    """
    if len(samples) < 3:
        raise ValueError(f"need >= 3 samples for quadratic fit, got {len(samples)}")
    # Build the normal equations by hand to avoid depending on numpy here.
    # For y = a + b*x + c*x^2, minimizing sum (y_i - a - b*x_i - c*x_i^2)^2
    # yields a 3x3 linear system.
    n = len(samples)
    Sx = sum(x for x, _ in samples)
    Sx2 = sum(x * x for x, _ in samples)
    Sx3 = sum(x ** 3 for x, _ in samples)
    Sx4 = sum(x ** 4 for x, _ in samples)
    Sy = sum(y for _, y in samples)
    Sxy = sum(x * y for x, y in samples)
    Sx2y = sum(x * x * y for x, y in samples)

    # System:
    # [ n    Sx    Sx2  ] [a]   [Sy  ]
    # [ Sx   Sx2   Sx3  ] [b] = [Sxy ]
    # [ Sx2  Sx3   Sx4  ] [c]   [Sx2y]
    A = [
        [n, Sx, Sx2],
        [Sx, Sx2, Sx3],
        [Sx2, Sx3, Sx4],
    ]
    rhs = [Sy, Sxy, Sx2y]
    a, b, c = _solve_3x3(A, rhs)
    return (a, b, c)


def _solve_3x3(A: list[list[float]], rhs: list[float]) -> tuple[float, float, float]:
    """Solve a 3x3 linear system by Gaussian elimination."""
    # Copy to avoid mutating caller.
    M = [row[:] + [r] for row, r in zip(A, rhs)]
    n = 3
    for i in range(n):
        # Pivot.
        max_row = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[max_row][i]) < 1e-12:
            raise ValueError("singular matrix in _solve_3x3")
        M[i], M[max_row] = M[max_row], M[i]
        # Eliminate below.
        for j in range(i + 1, n):
            f = M[j][i] / M[i][i]
            for k in range(i, n + 1):
                M[j][k] -= f * M[i][k]
    # Back-substitute.
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][k] * x[k] for k in range(i + 1, n))) / M[i][i]
    return (x[0], x[1], x[2])
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_length_calibration.py -v`
Expected: PASS (5 tests total).

- [ ] **Step 5: Commit**

```bash
git add length_calibration.py tests/test_length_calibration.py
git commit -m "feat(calibration): quadratic least-squares curve fitting"
```

---

## Task 10: Backend interface + SimBackend

**Files:**
- Create: `experiment_backend.py`
- Create: `tests/test_experiment_backend.py`

- [ ] **Step 1: Write failing tests for the SimBackend**

In `tests/test_experiment_backend.py`:

```python
"""Tests for Backend implementations."""
import math
import pytest
from experiment_backend import SimBackend, BackendState


def test_sim_backend_starts_at_rest():
    be = SimBackend(L_rest=240.0, num_modules=6)
    s = be.read_state()
    assert s.total_length_mm == pytest.approx(240.0, abs=1e-6)
    assert s.pitch == 0.0
    assert s.roll == 0.0
    assert s.yaw == 0.0


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
    # After e-stop, ticks should not advance further.
    before = be.read_state().total_length_mm
    for _ in range(20):
        be.tick(dt=0.05)
    assert be.read_state().total_length_mm == pytest.approx(before, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_backend.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'experiment_backend'`).

- [ ] **Step 3: Create `experiment_backend.py`**

```python
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
        # Pressure is instantaneous in sim. Scale length proportionally to
        # aggregate pressure to make the elongation phase look coherent.
        self._pressures[module_id] = target_psi
        # Re-derive length target from current module pressures if the caller
        # is driving by per-module pressures rather than total length.
        # (Controller uses set_total_length_target; this is a no-op sanity path.)

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
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_backend.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add experiment_backend.py tests/test_experiment_backend.py
git commit -m "feat(backend): Backend ABC + SimBackend for offline operation"
```

---

## Task 11: LiveBackend — serial-connected implementation

**Files:**
- Modify: `experiment_backend.py`
- Modify: `tests/test_experiment_backend.py`
- Modify: `arduino_interface.py`

- [ ] **Step 1: Add tests using a mock arduino**

Append to `tests/test_experiment_backend.py`:

```python
from experiment_backend import LiveBackend


class FakeArduino:
    def __init__(self):
        self.sent = []
        self.ser = object()  # truthy placeholder
    def send(self, cmd):
        self.sent.append(cmd)
        return True


def test_live_backend_sends_set_module_pressure():
    fake = FakeArduino()
    be = LiveBackend(arduino=fake, num_modules=6)
    be.set_module_pressure(module_id=2, target_psi=4.5)
    # Expect a per-module SET command. hPa conversion: psi / 0.0145038.
    assert any(c.startswith("SET 2") for c in fake.sent)


def test_live_backend_sends_tendon_rate():
    fake = FakeArduino()
    be = LiveBackend(arduino=fake, num_modules=6)
    be.set_tendon_rate(servo_id=1, rate=0.5)
    assert any(c.startswith("TEND 1") for c in fake.sent)


def test_live_backend_emergency_stop_sends_STOP():
    fake = FakeArduino()
    be = LiveBackend(arduino=fake, num_modules=6)
    be.emergency_stop()
    assert "STOP" in fake.sent


def test_live_backend_ingests_imu_line():
    fake = FakeArduino()
    be = LiveBackend(arduino=fake, num_modules=6)
    be.ingest_serial_line("IMU,1000,0.0,0.0,1.0,0.0,0.0,0.0,25.0")
    assert be.read_state().imu_fresh is True
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_backend.py -v`
Expected: FAIL (`ImportError: cannot import name 'LiveBackend'`).

- [ ] **Step 3: Add command helpers to `arduino_interface.py`**

Append to `arduino_interface.py` (after the existing class — these are standalone format helpers, not methods):

```python
# --- Experiment-mode protocol helpers ---------------------------------------
# Format strings for commands used by experiment_backend.LiveBackend.
# Kept here next to the serial interface so protocol changes live together.

def fmt_set_module(module_id: int, hpa: float) -> str:
    return f"SET {module_id} {hpa:.1f}"

def fmt_tendon(servo_id: int, rate: float) -> str:
    # rate in [-1, 1]; scaled to a firmware-understood int 0-1000 with sign.
    scaled = int(round(max(-1.0, min(1.0, rate)) * 1000))
    return f"TEND {servo_id} {scaled}"
```

- [ ] **Step 4: Add `LiveBackend` to `experiment_backend.py`**

Append to `experiment_backend.py`:

```python
import time
from orientation import OrientationEstimator
from arduino_interface import fmt_set_module, fmt_tendon


class LiveBackend(Backend):
    """Backend that drives the real rig over serial.

    Ingests IMU,* and PM,* telemetry lines via `ingest_serial_line()` (the
    main app's reader routes these in). Outgoing commands go through the
    injected arduino-like object.
    """
    IMU_FRESH_WINDOW_S = 1.0

    def __init__(self, arduino, num_modules: int):
        self._arduino = arduino
        self._num_modules = num_modules
        self._pressures: Dict[int, float] = {i + 1: 0.0 for i in range(num_modules)}
        self._last_imu_t: float = 0.0
        self._orient = OrientationEstimator()
        self._length_mm = 0.0
        self._halted = False

    @property
    def is_sim(self) -> bool:
        return False

    def ingest_serial_line(self, line: str) -> None:
        """Process one already-decoded serial line from the arduino reader."""
        if line.startswith("IMU,"):
            parts = line.split(",")
            if len(parts) >= 9:
                try:
                    ax, ay, az = float(parts[2]), float(parts[3]), float(parts[4])
                    gx, gy, gz = float(parts[5]), float(parts[6]), float(parts[7])
                    self._orient.update_accel(ax, ay, az)
                    self._orient.update_gyro(gx, gy, gz)
                    self._last_imu_t = time.monotonic()
                except (ValueError, IndexError):
                    pass
        elif line.startswith("PM,"):
            parts = line.split(",")
            if len(parts) >= 5:
                try:
                    mod_id = int(parts[1])
                    psi = float(parts[4])
                    self._pressures[mod_id] = psi
                except (ValueError, IndexError):
                    pass

    def set_module_pressure(self, module_id: int, target_psi: float) -> None:
        hpa = target_psi / 0.0145038
        self._arduino.send(fmt_set_module(module_id, hpa))

    def set_tendon_rate(self, servo_id: int, rate: float) -> None:
        self._arduino.send(fmt_tendon(servo_id, rate))

    def capture_zero(self) -> None:
        self._orient.capture_zero()

    def emergency_stop(self) -> None:
        self._halted = True
        self._arduino.send("STOP")

    def tick(self, dt: float) -> None:
        # Live backend has no internal time-advance; it reacts to serial.
        pass

    def read_state(self) -> BackendState:
        pitch, roll, yaw = self._orient.from_zero()
        imu_fresh = (time.monotonic() - self._last_imu_t) < self.IMU_FRESH_WINDOW_S
        return BackendState(
            total_length_mm=self._length_mm,  # set externally via calibration + pressures
            module_pressures_psi=dict(self._pressures),
            pitch=pitch,
            roll=roll,
            yaw=yaw,
            imu_fresh=imu_fresh,
        )

    def read_orientation(self) -> tuple[float, float, float]:
        return self._orient.from_zero()

    def set_total_length_mm(self, L_mm: float) -> None:
        """Updated from outside (by the controller) based on calibration+pressures."""
        self._length_mm = L_mm
```

- [ ] **Step 5: Run tests**

Run: `pytest tests/test_experiment_backend.py -v`
Expected: PASS (8 tests total).

- [ ] **Step 6: Commit**

```bash
git add experiment_backend.py arduino_interface.py tests/test_experiment_backend.py
git commit -m "feat(backend): LiveBackend wrapping serial I/O and telemetry ingest"
```

---

## Task 12: Controller — state machine skeleton

**Files:**
- Create: `experiment_controller.py`
- Create: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

In `tests/test_experiment_controller.py`:

```python
"""Tests for experiment_controller.ExperimentController state machine."""
import math
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: FAIL (`ModuleNotFoundError: No module named 'experiment_controller'`).

- [ ] **Step 3: Create `experiment_controller.py`**

```python
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
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "feat(controller): state machine skeleton with IDLE/ZEROING/WAITING + estop"
```

---

## Task 13: Controller — Reach transition into ELONGATING

**Files:**
- Modify: `experiment_controller.py`
- Modify: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_experiment_controller.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: FAIL (`reach` not defined).

- [ ] **Step 3: Implement `reach()`**

Append to `experiment_controller.py` inside the class:

```python
    # --- Reach -------------------------------------------------------------

    def reach(self, target: Tuple[float, float, float]) -> None:
        """Begin a reach attempt to `target` in the physics frame."""
        if self.state != State.WAITING_FOR_TARGET and self.state != State.REACHED:
            return
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
        # Command module pressures. For default calibration, split evenly.
        self._command_module_pressures_for_length(L_target)
        # In sim mode, also poke the backend with the total length directly
        # so its slewing engine has a target.
        if hasattr(self.backend, "set_total_length_target"):
            self.backend.set_total_length_target(L_target)
        self.state = State.ELONGATING

    def _command_module_pressures_for_length(self, L_total: float) -> None:
        """Solve per-module pressures that sum to L_total using calibration curves.

        Simple strategy: equal fractional extension across modules. If a module
        has no calibration (default), command a proportional pressure.
        """
        num = max(1, len(self.cal.modules) or 6)
        per_module = L_total / num
        for mid in (self.cal.modules.keys() if self.cal.modules else range(1, num + 1)):
            if self.cal.modules:
                m = self.cal.modules[mid]
                a, b, c = m["coeffs"]
                # Solve a + b*p + c*p^2 = per_module for p.
                psi = _solve_psi_for_length(a, b, c, per_module, m["max_psi"])
            else:
                # Default: linear assumption, 40mm rest + 5mm/psi.
                psi = max(0.0, (per_module - 40.0) / 5.0)
            self.backend.set_module_pressure(mid, psi)
```

Also add the helper at module scope (outside the class):

```python
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
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (7 tests total).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "feat(controller): reach() transitions to ELONGATING with workspace check"
```

---

## Task 14: Controller — ELONGATING tick + transition to BENDING or TIMED_OUT

**Files:**
- Modify: `experiment_controller.py`
- Modify: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_experiment_controller.py`:

```python
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
    # Simulate 11 seconds of stalled progress by e-stopping the backend's
    # internal slew — here we force a timeout by stubbing the clock.
    ctrl._phase_start = time.monotonic() - (ctrl.ELONGATION_TIMEOUT_S + 1)
    ctrl.tick(dt=0.05)
    assert ctrl.state == State.TIMED_OUT
```

(Add `import time` to test file if not already.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: FAIL (`tick` not defined).

- [ ] **Step 3: Implement `tick()` for ELONGATING**

Append to `experiment_controller.py` inside the class:

```python
    # --- Main tick ---------------------------------------------------------

    def tick(self, dt: float) -> None:
        """Advance the controller by dt seconds. Called ~20 Hz from the UI."""
        # Always let the backend advance first.
        self.backend.tick(dt)

        if self.state == State.ELONGATING:
            self._tick_elongating()
        elif self.state == State.BENDING:
            self._tick_bending()

    def _tick_elongating(self) -> None:
        s = self.backend.read_state()
        elapsed = time.monotonic() - self._phase_start
        # Convergence: SimBackend length near target, or pressures near setpoints.
        if abs(s.total_length_mm - self._L_target) < 5.0:
            # Move on to bending phase.
            self._phase_start = time.monotonic()
            # Command orientation target for sim; the real backend sees commands
            # via set_tendon_rate() during _tick_bending.
            if hasattr(self.backend, "set_orientation_target"):
                # Convert target direction into pitch/roll (roll=0, pitch=theta in bend plane).
                # For sim: project into pitch/roll according to phi.
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
        # Placeholder — will be filled in Task 15.
        pass
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (9 tests total).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "feat(controller): ELONGATING tick with convergence and timeout"
```

---

## Task 15: Controller — BENDING phase P-controller + tendon mapping

**Files:**
- Modify: `experiment_controller.py`
- Modify: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_experiment_controller.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: FAIL (_tick_bending is a placeholder, orientation stays at zero).

- [ ] **Step 3: Replace `_tick_bending` with the full control law**

In `experiment_controller.py`, replace the placeholder `_tick_bending`:

```python
    def _tick_bending(self) -> None:
        elapsed = time.monotonic() - self._phase_start
        pitch, roll, _yaw = self.backend.read_orientation()

        target_pitch = self._theta_target * math.cos(self._phi_target)
        target_roll = self._theta_target * math.sin(self._phi_target)

        err_pitch = target_pitch - pitch
        err_roll = target_roll - roll

        # Position error check (kinematic).
        from kinematics import forward_kinematics
        s = self.backend.read_state()
        theta_now = math.hypot(pitch, roll)
        phi_now = math.atan2(roll, pitch) if theta_now > 1e-9 else 0.0
        tip = forward_kinematics(s.total_length_mm, theta_now, phi_now)
        tx, ty, tz = self._target
        pos_err = math.sqrt((tip[0] - tx) ** 2 + (tip[1] - ty) ** 2 + (tip[2] - tz) ** 2)
        ang_err = math.hypot(err_pitch, err_roll)

        if ang_err < self.TOL_ANGLE_RAD and pos_err < self.TOL_POS_MM:
            # Halt tendons and declare REACHED.
            for sid in (1, 2, 3, 4):
                self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=False)
            return

        if elapsed > self.BEND_TIMEOUT_S:
            for sid in (1, 2, 3, 4):
                self.backend.set_tendon_rate(sid, 0.0)
            self._finish(timed_out=True)
            return

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
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (11 tests total).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "feat(controller): BENDING P-control with 4-tendon antagonistic mapping"
```

---

## Task 16: Controller — Re-zero guard

**Files:**
- Modify: `experiment_controller.py`
- Modify: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_experiment_controller.py`:

```python
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
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: FAIL (`can_rezero` and `rezero` not defined).

- [ ] **Step 3: Add guard methods**

Append inside the `ExperimentController` class:

```python
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
```

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (14 tests total).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "feat(controller): re-zero guard — requires all modules near rest"
```

---

## Task 17: Panel — skeleton frame with status line and buttons

**Files:**
- Create: `panels/experiment_panel.py`
- Modify: `panels/__init__.py`

- [ ] **Step 1: Read existing panel pattern**

Run: `cat panels/calibration_panel.py | head -50`
Expected: see the class pattern (subclass of `tk.Frame`, receives callbacks in `__init__`).

- [ ] **Step 2: Create `panels/experiment_panel.py` with skeleton**

```python
"""Experiments-mode panel: target picking + two-phase reach controller.

This module owns the outer `ExperimentPanel` frame. Picker widgets and the
3D preview live in sibling modules so each stays focused.
"""
from __future__ import annotations
import tkinter as tk

from theme import (
    BG_PANEL, ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_GREEN, ACCENT_ORANGE, ACCENT_RED,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_DATA, FONT_LABEL,
)
from widgets import AccentButton, EmergencyStopButton


class ExperimentPanel(tk.Frame):
    """Main panel for Experiments mode. Does not assume an active connection."""

    def __init__(
        self,
        parent,
        on_start_zero,
        on_confirm_zero,
        on_rezero,
        on_recalibrate_length,
        on_reach,
        on_emergency_stop,
        **kwargs,
    ):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._on_start_zero = on_start_zero
        self._on_confirm_zero = on_confirm_zero
        self._on_rezero = on_rezero
        self._on_recalibrate_length = on_recalibrate_length
        self._on_reach = on_reach
        self._on_emergency_stop = on_emergency_stop

        # Readouts as StringVars so the controller's update loop can push values in.
        self.status_var = tk.StringVar(value="IDLE — click Zero @ Rest to begin")
        self.backend_badge_var = tk.StringVar(value="")  # "MODE: SIMULATED" when sim
        self.tip_from_zero_var = tk.StringVar(value="(0.0, 0.0, 0.0)")
        self.tip_from_base_var = tk.StringVar(value="(0.0, 0.0, 0.0)")
        self.orient_var = tk.StringVar(value="pitch 0.0° roll 0.0° yaw 0.0°")
        self.yaw_drift_var = tk.StringVar(value="Yaw drift: 0.0°/min")
        self.phase_var = tk.StringVar(value="Phase: —")
        self.error_var = tk.StringVar(value="")

        self.target_x = tk.StringVar(value="")
        self.target_y = tk.StringVar(value="")
        self.target_z = tk.StringVar(value="")

        self._build()

    def _build(self) -> None:
        # Header.
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="EXPERIMENTS", font=FONT_BODY_BOLD,
                 fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

        # Top button row.
        btn_row = tk.Frame(self, bg=BG_PANEL)
        btn_row.pack(fill="x", padx=10, pady=6)
        AccentButton(btn_row, text="Zero @ Rest", accent=ACCENT_GREEN,
                     command=self._on_start_zero).pack(side="left", padx=3)
        AccentButton(btn_row, text="Confirm Zero",
                     command=self._on_confirm_zero).pack(side="left", padx=3)
        AccentButton(btn_row, text="Re-zero",
                     command=self._on_rezero).pack(side="left", padx=3)
        AccentButton(btn_row, text="Recalibrate Length", accent=ACCENT_ORANGE,
                     command=self._on_recalibrate_length).pack(side="left", padx=3)
        tk.Label(btn_row, textvariable=self.backend_badge_var,
                 font=FONT_LABEL, fg=ACCENT_ORANGE, bg=BG_PANEL).pack(side="right", padx=10)
        tk.Label(btn_row, textvariable=self.status_var,
                 font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="right", padx=10)

        # Canvas row placeholder (Tasks 18-20 replace this).
        self.canvas_row = tk.Frame(self, bg=BG_PANEL, height=260)
        self.canvas_row.pack(fill="x", padx=10, pady=4)
        tk.Label(self.canvas_row, text="[XY picker]  [XZ picker]  [3D preview]",
                 font=FONT_BODY, fg=TEXT_SECONDARY, bg=BG_PANEL).pack(pady=40)

        # Target entry row.
        tgt_row = tk.Frame(self, bg=BG_PANEL)
        tgt_row.pack(fill="x", padx=10, pady=(4, 6))
        tk.Label(tgt_row, text="Target:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        for label, var in (("X", self.target_x), ("Y", self.target_y), ("Z", self.target_z)):
            tk.Label(tgt_row, text=label, font=FONT_BODY,
                     fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(10, 2))
            tk.Entry(tgt_row, textvariable=var, width=8,
                     font=FONT_DATA).pack(side="left")
        AccentButton(tgt_row, text="Reach", accent=ACCENT_GREEN,
                     command=self._handle_reach).pack(side="left", padx=15)

        # Readout block.
        readout = tk.Frame(self, bg=BG_PANEL)
        readout.pack(fill="x", padx=10, pady=4)
        for var in (self.tip_from_zero_var, self.tip_from_base_var,
                    self.orient_var, self.yaw_drift_var,
                    self.phase_var, self.error_var):
            tk.Label(readout, textvariable=var, font=FONT_BODY,
                     fg=TEXT_PRIMARY, bg=BG_PANEL).pack(anchor="w")

        # E-stop.
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(10, 10))

    def _handle_reach(self) -> None:
        try:
            x = float(self.target_x.get())
            y = float(self.target_y.get())
            z = float(self.target_z.get())
        except ValueError:
            self.status_var.set("Invalid target — X/Y/Z must be numbers")
            return
        self._on_reach((x, y, z))

    # --- Public setters (called by the controller/app) -------------------

    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def set_backend_badge(self, text: str) -> None:
        self.backend_badge_var.set(text)

    def set_readouts(self, tip_from_zero, tip_from_base, pitch, roll, yaw,
                     yaw_drift_deg_per_min, phase, error_text):
        self.tip_from_zero_var.set(f"Tip (from zero): ({tip_from_zero[0]:.1f}, {tip_from_zero[1]:.1f}, {tip_from_zero[2]:.1f})")
        self.tip_from_base_var.set(f"Tip (from base): ({tip_from_base[0]:.1f}, {tip_from_base[1]:.1f}, {tip_from_base[2]:.1f})")
        import math as _m
        self.orient_var.set(
            f"pitch {_m.degrees(pitch):.1f}°  roll {_m.degrees(roll):.1f}°  yaw {_m.degrees(yaw):.1f}°"
        )
        self.yaw_drift_var.set(f"Yaw drift: {yaw_drift_deg_per_min:.2f}°/min")
        self.phase_var.set(f"Phase: {phase}")
        self.error_var.set(error_text)
```

- [ ] **Step 3: Register in `panels/__init__.py`**

Check current contents (`cat panels/__init__.py`) and add:

```python
from .experiment_panel import ExperimentPanel  # noqa: F401
```

- [ ] **Step 4: Smoke-test the panel by launching the app**

Run: `python main.py`
Manually: the app still launches (Experiments panel is not yet wired into mode selection; just verify no import errors).
Expected: window opens normally.

- [ ] **Step 5: Commit**

```bash
git add panels/experiment_panel.py panels/__init__.py
git commit -m "feat(panel): experiment panel skeleton with buttons, readouts, target entry"
```

---

## Task 18: Panel — XY picker widget

**Files:**
- Create: `panels/experiment_pickers.py`
- Modify: `panels/experiment_panel.py`

- [ ] **Step 1: Create `panels/experiment_pickers.py`**

```python
"""2D matplotlib pickers for Experiments mode — XY top view and XZ side view.

Each picker emits a callback with (x, y) or (x, z) when the user clicks inside
the reachable region. Clicks outside briefly flash red and are ignored.
"""
from __future__ import annotations
import math
import tkinter as tk
from typing import Callable, Optional, Tuple

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class _BasePicker(tk.Frame):
    def __init__(self, parent, title: str, width_px: int = 260, height_px: int = 260, **kwargs):
        super().__init__(parent, **kwargs)
        self._title = title
        self._fig = Figure(figsize=(width_px / 100, height_px / 100), dpi=100)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_title(title, fontsize=9)
        self._ax.set_aspect("equal", adjustable="box")
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas.mpl_connect("button_press_event", self._on_click)
        self._on_pick: Optional[Callable[[float, float], None]] = None

    def bind_pick(self, cb: Callable[[float, float], None]) -> None:
        self._on_pick = cb

    def _on_click(self, event) -> None:
        if event.xdata is None or event.ydata is None:
            return
        if not self._is_inside(event.xdata, event.ydata):
            self._flash_invalid()
            return
        if self._on_pick is not None:
            self._on_pick(event.xdata, event.ydata)

    def _is_inside(self, u: float, v: float) -> bool:  # override
        return True

    def _flash_invalid(self) -> None:
        # Draw a momentary red cross at the click; clear on next redraw.
        # Minimal implementation: change the axis frame color briefly.
        for spine in self._ax.spines.values():
            spine.set_color("red")
        self._canvas.draw_idle()
        self.after(200, self._clear_flash)

    def _clear_flash(self) -> None:
        for spine in self._ax.spines.values():
            spine.set_color("black")
        self._canvas.draw_idle()


class XYPicker(_BasePicker):
    """Top-down XY picker. `r_max` is the reachable horizontal radius."""
    def __init__(self, parent, r_max: float = 400.0, **kwargs):
        super().__init__(parent, title="XY (top view)", **kwargs)
        self._r_max = r_max
        self._tip_xy: Tuple[float, float] = (0.0, 0.0)
        self._target_xy: Optional[Tuple[float, float]] = None
        self._redraw()

    def set_r_max(self, r_max: float) -> None:
        self._r_max = r_max
        self._redraw()

    def set_tip(self, x: float, y: float) -> None:
        self._tip_xy = (x, y)
        self._redraw()

    def set_target(self, x: Optional[float], y: Optional[float]) -> None:
        self._target_xy = (x, y) if x is not None and y is not None else None
        self._redraw()

    def _is_inside(self, x: float, y: float) -> bool:
        return math.hypot(x, y) <= self._r_max

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        ax.set_title(self._title, fontsize=9)
        lim = self._r_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal", adjustable="box")
        # Reachable circle.
        circle = plt_circle(0.0, 0.0, self._r_max)
        ax.plot(circle[0], circle[1], linestyle="--", color="#888")
        # Tip + target markers.
        ax.plot([self._tip_xy[0]], [self._tip_xy[1]], marker="o", color="#0aa")
        if self._target_xy is not None:
            ax.plot([self._target_xy[0]], [self._target_xy[1]],
                    marker="+", color="#f80", markersize=15)
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        self._canvas.draw_idle()


def plt_circle(cx: float, cy: float, r: float, n: int = 64) -> Tuple[list, list]:
    xs, ys = [], []
    for i in range(n + 1):
        t = 2 * math.pi * i / n
        xs.append(cx + r * math.cos(t))
        ys.append(cy + r * math.sin(t))
    return xs, ys
```

- [ ] **Step 2: Wire `XYPicker` into the panel**

Replace the `canvas_row` placeholder in `panels/experiment_panel.py` `_build`:

```python
        # Canvas row.
        from panels.experiment_pickers import XYPicker
        self.canvas_row = tk.Frame(self, bg=BG_PANEL)
        self.canvas_row.pack(fill="x", padx=10, pady=4)
        self.xy_picker = XYPicker(self.canvas_row, r_max=400.0)
        self.xy_picker.pack(side="left", padx=4)
        self.xy_picker.bind_pick(self._on_xy_click)
```

Add handler method:

```python
    def _on_xy_click(self, x: float, y: float) -> None:
        self.target_x.set(f"{x:.1f}")
        self.target_y.set(f"{y:.1f}")
```

- [ ] **Step 3: Smoke-test**

Run: `python main.py`
Manually: select "Experiments" later (not wired yet — skip this manual check and just verify import works).

Alternative verification — run the module in isolation:

```bash
python -c "from panels.experiment_pickers import XYPicker; print('ok')"
```

Expected: prints `ok`.

- [ ] **Step 4: Commit**

```bash
git add panels/experiment_pickers.py panels/experiment_panel.py
git commit -m "feat(panel): XY picker with reachable-circle bounds and click callback"
```

---

## Task 19: Panel — XZ picker widget with guide line

**Files:**
- Modify: `panels/experiment_pickers.py`
- Modify: `panels/experiment_panel.py`

- [ ] **Step 1: Add `XZPicker` class**

Append to `panels/experiment_pickers.py`:

```python
class XZPicker(_BasePicker):
    """Side view XZ picker with a vertical guide line at locked X."""
    def __init__(self, parent, L_rest: float = 240.0, L_max: float = 480.0,
                 theta_max_rad: float = math.radians(60), **kwargs):
        super().__init__(parent, title="XZ (side view)", **kwargs)
        self._L_rest = L_rest
        self._L_max = L_max
        self._theta_max = theta_max_rad
        self._tip_xz: Tuple[float, float] = (0.0, L_rest)
        self._locked_x: Optional[float] = None
        self._target_xz: Optional[Tuple[float, float]] = None
        self._redraw()

    def set_workspace(self, L_rest: float, L_max: float, theta_max_rad: float) -> None:
        self._L_rest = L_rest
        self._L_max = L_max
        self._theta_max = theta_max_rad
        self._redraw()

    def set_tip(self, x: float, z: float) -> None:
        self._tip_xz = (x, z)
        self._redraw()

    def set_locked_x(self, x: Optional[float]) -> None:
        self._locked_x = x
        self._redraw()

    def set_target(self, x: Optional[float], z: Optional[float]) -> None:
        self._target_xz = (x, z) if x is not None and z is not None else None
        self._redraw()

    def _is_inside(self, x: float, z: float) -> bool:
        # Must lie inside the workspace cross-section AND on the guide line if locked.
        if self._locked_x is not None and abs(x - self._locked_x) > 2.0:
            return False
        # Quick reachability check in XZ: approximate with a dome of radius L_max.
        if z < 0:
            return False
        if math.hypot(x, z) > self._L_max * 1.02:
            return False
        return True

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        ax.set_title(self._title, fontsize=9)
        lim = self._L_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-10, lim)
        ax.set_aspect("equal", adjustable="box")
        # Reachable dome cross-section: L_min inner arc, L_max outer arc.
        inner = _dome_arc(self._L_rest, self._theta_max)
        outer = _dome_arc(self._L_max, self._theta_max)
        ax.plot(inner[0], inner[1], linestyle=":", color="#888")
        ax.plot(outer[0], outer[1], linestyle="--", color="#888")
        # Tip + guide + target.
        ax.plot([self._tip_xz[0]], [self._tip_xz[1]], marker="o", color="#0aa")
        if self._locked_x is not None:
            ax.axvline(self._locked_x, color="#0aa", linewidth=0.8, linestyle="--")
        if self._target_xz is not None:
            ax.plot([self._target_xz[0]], [self._target_xz[1]],
                    marker="+", color="#f80", markersize=15)
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Z (mm)")
        self._canvas.draw_idle()


def _dome_arc(L: float, theta_max: float, n: int = 40) -> Tuple[list, list]:
    """Generate (x, z) points of a PCC arc swept from -theta_max to +theta_max."""
    xs, zs = [], []
    import math as _m
    for i in range(n + 1):
        t = -theta_max + (2 * theta_max) * i / n
        if abs(t) < 1e-9:
            xs.append(0.0)
            zs.append(L)
        else:
            R = L / abs(t)
            r = R * (1.0 - _m.cos(t))
            z = R * _m.sin(abs(t))
            xs.append(_m.copysign(r, t))
            zs.append(z)
    return xs, zs
```

- [ ] **Step 2: Wire `XZPicker` into the panel**

In `panels/experiment_panel.py`, extend the canvas_row build:

```python
        from panels.experiment_pickers import XYPicker, XZPicker
        self.xz_picker = XZPicker(self.canvas_row, L_rest=240.0, L_max=480.0)
        self.xz_picker.pack(side="left", padx=4)
        self.xz_picker.bind_pick(self._on_xz_click)
```

Add handler:

```python
    def _on_xz_click(self, x: float, z: float) -> None:
        self.target_x.set(f"{x:.1f}")
        self.target_z.set(f"{z:.1f}")

    def _on_xy_click(self, x: float, y: float) -> None:  # replace earlier
        self.target_x.set(f"{x:.1f}")
        self.target_y.set(f"{y:.1f}")
        # Lock X on the XZ picker so clicks there stay consistent.
        self.xz_picker.set_locked_x(x)
```

- [ ] **Step 3: Smoke-test import**

```bash
python -c "from panels.experiment_pickers import XZPicker; print('ok')"
```

Expected: `ok`.

- [ ] **Step 4: Commit**

```bash
git add panels/experiment_pickers.py panels/experiment_panel.py
git commit -m "feat(panel): XZ picker with guide line locked by XY click"
```

---

## Task 20: Panel — 3D trunk preview widget

**Files:**
- Create: `panels/experiment_preview.py`
- Modify: `panels/experiment_panel.py`

- [ ] **Step 1: Create `panels/experiment_preview.py`**

```python
"""Read-only 3D preview of the trunk for Experiments mode."""
from __future__ import annotations
import math
import tkinter as tk
from typing import Optional, Tuple

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers projection


class TrunkPreview3D(tk.Frame):
    """3D render of base plate + trunk arc + target crosshair + reachable dome."""

    def __init__(self, parent, L_max: float = 480.0, theta_max: float = math.radians(60), **kwargs):
        super().__init__(parent, **kwargs)
        self._L_max = L_max
        self._theta_max = theta_max
        self._fig = Figure(figsize=(2.8, 2.8), dpi=100)
        self._ax = self._fig.add_subplot(111, projection="3d")
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._tip: Tuple[float, float, float] = (0.0, 0.0, 240.0)
        self._arc_points = [(0.0, 0.0, z) for z in range(0, 241, 12)]
        self._target: Optional[Tuple[float, float, float]] = None
        self._redraw()

    def set_tip_and_arc(self, tip: Tuple[float, float, float],
                        arc_points: list[Tuple[float, float, float]]) -> None:
        self._tip = tip
        self._arc_points = arc_points
        self._redraw()

    def set_target(self, target: Optional[Tuple[float, float, float]]) -> None:
        self._target = target
        self._redraw()

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        lim = self._L_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(0, lim)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        # Base plate (small square at z=0).
        sq = 40.0
        ax.plot([-sq, sq, sq, -sq, -sq], [-sq, -sq, sq, sq, -sq], [0, 0, 0, 0, 0], color="#444")
        # Trunk arc.
        if self._arc_points:
            xs = [p[0] for p in self._arc_points]
            ys = [p[1] for p in self._arc_points]
            zs = [p[2] for p in self._arc_points]
            ax.plot(xs, ys, zs, color="#0aa", linewidth=2)
        # Tip marker.
        ax.scatter([self._tip[0]], [self._tip[1]], [self._tip[2]], color="#0aa", s=40)
        # Target marker.
        if self._target is not None:
            ax.scatter([self._target[0]], [self._target[1]], [self._target[2]],
                       color="#f80", s=60, marker="+")
        self._canvas.draw_idle()
```

- [ ] **Step 2: Add the preview to the panel's canvas row**

In `panels/experiment_panel.py`, in `_build` after `xz_picker`:

```python
        from panels.experiment_preview import TrunkPreview3D
        self.preview3d = TrunkPreview3D(self.canvas_row, L_max=480.0)
        self.preview3d.pack(side="left", padx=4, fill="both", expand=True)
```

- [ ] **Step 3: Smoke-test**

```bash
python -c "from panels.experiment_preview import TrunkPreview3D; print('ok')"
```

Expected: `ok`.

- [ ] **Step 4: Commit**

```bash
git add panels/experiment_preview.py panels/experiment_panel.py
git commit -m "feat(panel): 3D matplotlib preview widget for trunk + target"
```

---

## Task 21: Panel — target entry two-way binding + update_preview method

**Files:**
- Modify: `panels/experiment_panel.py`

- [ ] **Step 1: Add method to push tip + target into the three canvases**

In `ExperimentPanel`, add:

```python
    def set_tip_position(self, tip_display: Tuple[float, float, float],
                         arc_points_display: list) -> None:
        """Syncs all three canvases. Args are in DISPLAY frame (relative to zero).

        The pickers and the 3D preview all render in display frame so the user
        picks targets relative to the resting tip (spec §3). The caller
        (app.py update_loop) converts physics-frame telemetry to display frame
        before invoking this method.
        """
        x, y, z = tip_display
        self.xy_picker.set_tip(x, y)
        self.xz_picker.set_tip(x, z)
        self.preview3d.set_tip_and_arc(tip_display, arc_points_display)

    def set_target_marker(self, target_from_base: Optional[Tuple[float, float, float]]) -> None:
        if target_from_base is None:
            self.xy_picker.set_target(None, None)
            self.xz_picker.set_target(None, None)
            self.preview3d.set_target(None)
        else:
            tx, ty, tz = target_from_base
            self.xy_picker.set_target(tx, ty)
            self.xz_picker.set_target(tx, tz)
            self.preview3d.set_target(target_from_base)

    def set_workspace(self, r_max: float, L_rest: float, L_max: float, theta_max_rad: float) -> None:
        self.xy_picker.set_r_max(r_max)
        self.xz_picker.set_workspace(L_rest, L_max, theta_max_rad)
        self.preview3d._L_max = L_max
        self.preview3d._theta_max = theta_max_rad
        self.preview3d._redraw()
```

At the top of the file add:

```python
from typing import Optional, Tuple
```

- [ ] **Step 2: Smoke-test panel instantiation**

Create `tests/test_experiment_panel_smoke.py`:

```python
"""Smoke test: panel instantiates without a live controller or serial."""
import tkinter as tk
import pytest


@pytest.mark.skipif("DISPLAY" not in __import__("os").environ and __import__("sys").platform.startswith("linux"),
                    reason="no display")
def test_panel_instantiates_offline():
    root = tk.Tk()
    root.withdraw()
    from panels.experiment_panel import ExperimentPanel
    panel = ExperimentPanel(
        root,
        on_start_zero=lambda: None,
        on_confirm_zero=lambda: None,
        on_rezero=lambda: None,
        on_recalibrate_length=lambda: None,
        on_reach=lambda t: None,
        on_emergency_stop=lambda: None,
    )
    panel.pack()
    root.update()
    assert panel.status_var.get().startswith("IDLE")
    root.destroy()
```

Run: `pytest tests/test_experiment_panel_smoke.py -v`
Expected: PASS (panel builds without errors).

- [ ] **Step 3: Commit**

```bash
git add panels/experiment_panel.py tests/test_experiment_panel_smoke.py
git commit -m "feat(panel): tip/target/workspace setters; smoke test for offline instantiation"
```

---

## Task 22: Length calibration dialog

**Files:**
- Create: `panels/experiment_calibration.py`

- [ ] **Step 1: Create the dialog**

```python
"""Modal length-calibration dialog.

Walks the operator through each available module: command a pressure,
prompt for a measured length, collect samples, fit a quadratic, save.
"""
from __future__ import annotations
import tkinter as tk
from tkinter import simpledialog, messagebox
from typing import Callable, Dict, List

from length_calibration import LengthCalibration, fit_module_curve


def run_length_calibration_dialog(
    root: tk.Tk,
    available_modules: List[int],
    command_pressure: Callable[[int, float], None],
    wait_for_steady: Callable[[], None],
    max_psi: float = 8.0,
    num_points: int = 5,
) -> LengthCalibration | None:
    """Blocking walkthrough. Returns a `LengthCalibration` on success or None on cancel."""
    if not available_modules:
        messagebox.showwarning("Calibrate Length",
                               "No modules with live pressure telemetry. Connect the rig first.")
        return None

    modules: Dict[int, dict] = {}
    psi_setpoints = [max_psi * i / (num_points - 1) for i in range(num_points)]
    for mid in available_modules:
        samples: List[tuple[float, float]] = []
        for psi in psi_setpoints:
            command_pressure(mid, psi)
            wait_for_steady()
            answer = simpledialog.askfloat(
                f"Module {mid}",
                f"Commanded {psi:.2f} psi. Measure module length in mm and enter:",
                parent=root, minvalue=0.0, maxvalue=500.0,
            )
            if answer is None:
                return None
            samples.append((psi, answer))
        try:
            a, b, c = fit_module_curve(samples)
        except ValueError as e:
            messagebox.showerror("Calibrate Length", f"Module {mid} fit failed: {e}")
            return None
        modules[mid] = {
            "rest_length_mm": samples[0][1],
            "coeffs": [a, b, c],
            "max_psi": max_psi,
        }
        # Vent the module for the next iteration.
        command_pressure(mid, 0.0)

    # Modules not in available_modules are treated as missing.
    all_possible = set(range(1, 7))
    missing = sorted(all_possible - set(available_modules))
    cal = LengthCalibration(modules=modules, missing_modules=missing, is_default=False)
    return cal
```

- [ ] **Step 2: Add a unit test that bypasses the dialog by monkeypatching**

Append to `tests/test_length_calibration.py`:

```python
def test_dialog_flow_builds_calibration(monkeypatch, tmp_path):
    """Drive run_length_calibration_dialog with patched simpledialog to end-to-end test it."""
    import panels.experiment_calibration as calmod

    class FakeSD:
        seq = iter([40.0, 50.0, 60.0, 70.0, 80.0])  # linear length(psi)=40+5*psi
        @staticmethod
        def askfloat(*args, **kwargs):
            return next(FakeSD.seq)

    monkeypatch.setattr(calmod.simpledialog, "askfloat", FakeSD.askfloat)
    sent = []
    cal = calmod.run_length_calibration_dialog(
        root=None,
        available_modules=[1],
        command_pressure=lambda mid, psi: sent.append((mid, psi)),
        wait_for_steady=lambda: None,
        max_psi=8.0,
        num_points=5,
    )
    assert cal is not None
    assert 1 in cal.modules
    assert cal.modules[1]["coeffs"][0] == pytest.approx(40.0, abs=1e-4)
    assert cal.modules[1]["coeffs"][1] == pytest.approx(5.0, abs=1e-4)
```

- [ ] **Step 3: Run tests**

Run: `pytest tests/test_length_calibration.py -v`
Expected: PASS (6 tests).

- [ ] **Step 4: Commit**

```bash
git add panels/experiment_calibration.py tests/test_length_calibration.py
git commit -m "feat(calibration): modal length-calibration dialog with per-module walkthrough"
```

---

## Task 23: app.py — generalize to 6 modules

**Files:**
- Modify: `app.py` (lines 73-78)

- [ ] **Step 1: Read current module setup**

Run: `sed -n '73,80p' app.py`
Expected: see the three `_add_module_data(...)` calls.

- [ ] **Step 2: Replace the hardcoded 3-module block with a configurable list**

In `app.py`, replace lines 73-78:

```python
        # Module system — configurable list of modules. Pin mapping reflects
        # current wiring; modules beyond wired ones get None pins.
        self.modules = []
        self.next_module_id = 1
        MODULE_CONFIG = [
            ("Module 1", 3, 4),
            ("Module 2", 8, 7),
            ("Module 3", None, None),
            ("Module 4", None, None),
            ("Module 5", None, None),
            ("Module 6", None, None),
        ]
        for name, step_pin, dir_pin in MODULE_CONFIG:
            self._add_module_data(name, step_pin=step_pin, dir_pin=dir_pin)
```

- [ ] **Step 3: Smoke-test**

Run: `python main.py`
Expected: window opens; pressure display row shows 6 boxes. Close.

- [ ] **Step 4: Commit**

```bash
git add app.py
git commit -m "refactor(app): generalize hardcoded 3-module setup to 6-module config list"
```

---

## Task 24: app.py — add Experiments radio and instantiate panel

**Files:**
- Modify: `app.py`

- [ ] **Step 1: Add Experiments to the mode radio list**

Locate the radio-button loop in `app.py` (currently `app.py:228-232`):

```python
        for text, val in [("Burst Control", "burst"), ("PID Control", "pid"),
                          ("Calibration", "calibration")]:
```

Change to:

```python
        for text, val in [("Burst Control", "burst"), ("PID Control", "pid"),
                          ("Calibration", "calibration"), ("Experiments", "experiments")]:
```

- [ ] **Step 2: Add backend + controller + panel instantiation**

After the panel creations (around `app.py:250`), add:

```python
        # Experiments mode: backend + controller + panel.
        from length_calibration import load_or_default
        from experiment_backend import SimBackend, LiveBackend
        from experiment_controller import ExperimentController

        self.length_cal_path = os.path.join(os.path.dirname(__file__) or ".", "length_calibration.json")
        self.length_cal = load_or_default(self.length_cal_path)
        # Start with a SimBackend; app._refresh_experiment_backend() swaps to
        # LiveBackend whenever serial is connected.
        self.experiment_backend = SimBackend(
            L_rest=self.length_cal.L_rest, num_modules=len(self.modules),
        )
        self.experiment_controller = ExperimentController(
            backend=self.experiment_backend, calibration=self.length_cal,
        )
        self.experiment_panel = ExperimentPanel(
            self.panel_container,
            on_start_zero=self._exp_start_zero,
            on_confirm_zero=self._exp_confirm_zero,
            on_rezero=self._exp_rezero,
            on_recalibrate_length=self._exp_recalibrate_length,
            on_reach=self._exp_reach,
            on_emergency_stop=self._exp_emergency_stop,
        )
```

At the top of the file add the import:

```python
from panels import BurstPanel, PIDPanel, CalibrationPanel, ExperimentPanel
```

- [ ] **Step 3: Add the handler methods** (in the `ArmUI` class, near other handlers)

```python
    # --- Experiments handlers --------------------------------------------

    def _exp_start_zero(self):
        self.experiment_controller.start_zeroing()
        self.experiment_panel.set_status("Zeroing — confirm trunk is at rest, then Confirm Zero")

    def _exp_confirm_zero(self):
        self.experiment_controller.confirm_zero()
        self.experiment_panel.set_status("Zero captured — pick a target")

    def _exp_rezero(self):
        if self.experiment_controller.rezero():
            self.experiment_panel.set_status("Re-zeroed at current rest pose")
        else:
            self.experiment_panel.set_status("Re-zero blocked — pressures not near rest")

    def _exp_recalibrate_length(self):
        if not (self.arduino.ser and self.arduino.ser.is_open):
            self.experiment_panel.set_status("Recalibrate needs a serial connection")
            return
        from panels.experiment_calibration import run_length_calibration_dialog
        live_modules = [mid for mid, psi in self.experiment_backend.read_state().module_pressures_psi.items()
                        if isinstance(psi, (int, float))]
        cal = run_length_calibration_dialog(
            root=self.root,
            available_modules=live_modules or [m["id"] for m in self.modules],
            command_pressure=lambda mid, psi: self.experiment_backend.set_module_pressure(mid, psi),
            wait_for_steady=lambda: self.root.after(1500),
        )
        if cal is not None:
            cal.save(self.length_cal_path)
            self.length_cal = cal
            self.experiment_controller.cal = cal
            self.experiment_panel.set_workspace(
                r_max=cal.L_max, L_rest=cal.L_rest, L_max=cal.L_max,
                theta_max_rad=1.0472,
            )
            self.experiment_panel.set_status("Length calibration saved")

    def _exp_reach(self, target_display):
        # Translate target from display frame (zero-relative) to physics frame
        # (base-relative). For the simple model, display frame origin = (0,0,L_rest).
        tx, ty, tz = target_display
        target_physics = (tx, ty, tz + self.length_cal.L_rest)
        try:
            self.experiment_controller.reach(target_physics)
            self.experiment_panel.set_status(f"Reaching toward {target_display}")
            self.experiment_panel.set_target_marker(target_physics)
        except ValueError as e:
            self.experiment_panel.set_status(f"Rejected: {e}")

    def _exp_emergency_stop(self):
        self.experiment_controller.emergency_stop()
        self.experiment_panel.set_status("EMERGENCY STOP")
```

- [ ] **Step 4: Wire `experiments` into `switch_mode`**

In `switch_mode`, extend the forget list and add the new branch:

```python
        self.burst_panel.pack_forget()
        self.pid_panel.pack_forget()
        self.cal_panel.pack_forget()
        self.experiment_panel.pack_forget()
```

```python
        elif mode == "experiments":
            # Select backend based on serial state.
            self._refresh_experiment_backend()
            self.experiment_panel.set_workspace(
                r_max=self.length_cal.L_max,
                L_rest=self.length_cal.L_rest,
                L_max=self.length_cal.L_max,
                theta_max_rad=1.0472,
            )
            self.experiment_panel.pack(fill="both", expand=True)
```

- [ ] **Step 5: Add `_refresh_experiment_backend` method**

```python
    def _refresh_experiment_backend(self):
        """Swap between LiveBackend and SimBackend based on serial connection state."""
        from experiment_backend import LiveBackend, SimBackend
        connected = bool(self.arduino.ser and self.arduino.ser.is_open)
        want_live = connected and not self.experiment_backend.is_sim
        want_sim = (not connected) and self.experiment_backend.is_sim
        if want_live or want_sim:
            return  # already correct type
        if connected:
            self.experiment_backend = LiveBackend(arduino=self.arduino, num_modules=len(self.modules))
            self.experiment_panel.set_backend_badge("")
        else:
            self.experiment_backend = SimBackend(
                L_rest=self.length_cal.L_rest, num_modules=len(self.modules),
            )
            self.experiment_panel.set_backend_badge("MODE: SIMULATED")
        self.experiment_controller.backend = self.experiment_backend
```

- [ ] **Step 6: Smoke-test launching the app and switching modes**

Run: `python main.py`
Manually:
  1. Verify window opens, all 4 radio buttons present.
  2. Click Experiments — panel appears with `MODE: SIMULATED` badge (no serial connected).
  3. Click Zero @ Rest → Confirm Zero → type target `X=0 Y=0 Z=50` → Reach.
  4. Watch the 3D preview animate (SimBackend advances).
  5. Click Burst Control, then back to Experiments — state persists.
  6. Close.

Expected: all above work; no exceptions in terminal.

- [ ] **Step 7: Commit**

```bash
git add app.py
git commit -m "feat(app): wire Experiments radio, panel, controller, backend auto-swap"
```

---

## Task 25: app.py — route telemetry into LiveBackend + controller tick

**Files:**
- Modify: `app.py`

- [ ] **Step 1: In `update_loop`, route IMU and PM lines to the LiveBackend**

Inside the queue-draining block (after the existing IMU parsing), add:

```python
            # Route telemetry into the experiments backend when in live mode.
            if not self.experiment_backend.is_sim:
                try:
                    self.experiment_backend.ingest_serial_line(line)
                except Exception as e:
                    print(f"[WARN] experiment backend ingest failed: {e}")
```

(Place this *before* the `self.status.set(line)` fallback — it's the same shape as the existing P/PM/IMU branches.)

- [ ] **Step 2: Tick the experiment controller on every update_loop iteration**

Before `self.root.after(100, self.update_loop)`, add:

```python
        # Advance experiment controller ~10Hz (matches update_loop cadence).
        try:
            self.experiment_controller.tick(dt=0.1)
            # Push readouts to the panel — in DISPLAY frame for the pickers,
            # in both frames for the readout labels.
            s = self.experiment_backend.read_state()
            import math as _m
            from kinematics import forward_kinematics
            theta_now = _m.hypot(s.pitch, s.roll)
            phi_now = _m.atan2(s.roll, s.pitch) if theta_now > 1e-9 else 0.0
            tip_base = forward_kinematics(s.total_length_mm, theta_now, phi_now)
            tip_zero = (tip_base[0], tip_base[1], tip_base[2] - self.length_cal.L_rest)
            yaw_drift = self._yaw_drift_deg_per_min(s.yaw)
            self.experiment_panel.set_readouts(
                tip_from_zero=tip_zero,
                tip_from_base=tip_base,
                pitch=s.pitch, roll=s.roll, yaw=s.yaw,
                yaw_drift_deg_per_min=yaw_drift,
                phase=self.experiment_controller.state.value,
                error_text=(
                    f"err {_m.degrees(self.experiment_controller.last_result.final_pitch_err_rad):.1f}° / "
                    f"{self.experiment_controller.last_result.final_position_err_mm:.1f}mm  "
                    f"in {self.experiment_controller.last_result.elapsed_s:.1f}s"
                    if self.experiment_controller.last_result else ""
                ),
            )
            # Arc points in DISPLAY frame (subtract L_rest from Z) so pickers render correctly.
            arc_pts_base = _sample_arc_points(s.total_length_mm, theta_now, phi_now, n=20)
            arc_pts_disp = [(p[0], p[1], p[2] - self.length_cal.L_rest) for p in arc_pts_base]
            self.experiment_panel.set_tip_position(tip_zero, arc_pts_disp)
        except Exception as e:
            # Keep the main loop alive even if the experiment panel breaks.
            print(f"[WARN] experiment tick failed: {e}")
```

- [ ] **Step 3: Add the `_sample_arc_points` helper at module scope**

At the bottom of `app.py` (outside the class):

```python
def _sample_arc_points(L: float, theta: float, phi: float, n: int = 20):
    """Sample n+1 points along the PCC arc from base to tip."""
    from kinematics import forward_kinematics
    return [forward_kinematics(L * i / n, theta * i / n, phi) for i in range(n + 1)]
```

- [ ] **Step 4: Smoke-test**

Run: `python main.py`
Manually:
  1. Experiments mode → Zero @ Rest → Confirm → reach (0, 0, 50).
  2. Verify 3D preview animates toward the target, readouts update, and on completion the error line shows e.g. `err 0.1° / 0.3mm in 4.5s`.
  3. E-stop during a run → state returns to IDLE.

Expected: all works; terminal free of errors.

- [ ] **Step 5: Commit**

```bash
git add app.py
git commit -m "feat(app): route telemetry into LiveBackend; tick controller; update readouts"
```

---

## Task 25B: Yaw-drift rolling estimator + live IMU-staleness watchdog

**Files:**
- Modify: `app.py`
- Modify: `experiment_controller.py`
- Modify: `tests/test_experiment_controller.py`

- [ ] **Step 1: Add a failing test for IMU-staleness watchdog**

Append to `tests/test_experiment_controller.py`:

```python
def test_bending_aborts_on_stale_IMU_in_live_mode():
    """If read_state().imu_fresh is False during BENDING, controller must halt."""
    import math
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
        def read_orientation(self): return (0.0, 0.0, 0.0)
        def set_module_pressure(self, mid, psi): self._p[mid] = psi
        def set_tendon_rate(self, sid, rate): pass
        def capture_zero(self): pass
        def emergency_stop(self): self.sent_stops += 1
        def tick(self, dt): pass
        def set_total_length_target(self, L): pass
        def set_orientation_target(self, p, r, y): pass

    be = StaleBackend()
    cal = LengthCalibration(is_default=True)
    ctrl = ExperimentController(backend=be, calibration=cal)
    ctrl.start_zeroing(); ctrl.confirm_zero()
    ctrl.reach(target=(0.0, 0.0, 300.0))
    # Force into BENDING so the staleness check applies there.
    ctrl._phase_start = time.monotonic()
    ctrl.state = State.BENDING
    ctrl._target = (0.0, 0.0, 300.0)
    ctrl.tick(dt=0.1)
    assert ctrl.state == State.IDLE
    assert be.sent_stops >= 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_experiment_controller.py::test_bending_aborts_on_stale_IMU_in_live_mode -v`
Expected: FAIL (controller keeps running despite `imu_fresh=False`).

- [ ] **Step 3: Add the staleness check to `_tick_bending`**

In `experiment_controller.py`, at the top of `_tick_bending`, add:

```python
    def _tick_bending(self) -> None:
        s = self.backend.read_state()
        # Live-mode IMU-staleness watchdog (spec §8). SimBackend always reports fresh.
        if not self.backend.is_sim and not s.imu_fresh:
            self.emergency_stop()
            return
        elapsed = time.monotonic() - self._phase_start
        pitch, roll, _yaw = self.backend.read_orientation()
        # ... rest of the existing method unchanged ...
```

(Replace the existing first two lines `elapsed = ...` and `pitch, roll, ...` accordingly; re-use `s` where `_tick_bending` previously re-read it.)

- [ ] **Step 4: Run tests**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: PASS (all existing tests + the new one = 15 total).

- [ ] **Step 5: Add `_yaw_drift_deg_per_min` helper to `app.py`**

Near the top of the `ArmUI` class, initialize a rolling buffer:

```python
        from collections import deque
        self._yaw_history = deque(maxlen=600)  # ~60s at 10Hz
```

Add the method:

```python
    def _yaw_drift_deg_per_min(self, current_yaw_rad: float) -> float:
        """Estimate yaw drift over the last ~60s as degrees per minute."""
        import math as _m, time as _t
        now = _t.monotonic()
        self._yaw_history.append((now, current_yaw_rad))
        if len(self._yaw_history) < 2:
            return 0.0
        t0, y0 = self._yaw_history[0]
        dt = now - t0
        if dt < 5.0:
            return 0.0
        drift_rad_per_s = (current_yaw_rad - y0) / dt
        return _m.degrees(drift_rad_per_s) * 60.0
```

- [ ] **Step 6: Smoke-test the full loop**

Run: `python main.py`
Manually:
  1. Experiments mode → Zero @ Rest → Confirm.
  2. Let it sit idle for 10+ seconds — yaw drift readout should update.
  3. Reach toward a target; confirm tip follows in the 3D preview (display frame).

Expected: yaw drift number updates and is near zero in sim (noise-bounded); 3D preview tip position matches readout `Tip (from zero)`.

- [ ] **Step 7: Commit**

```bash
git add experiment_controller.py app.py tests/test_experiment_controller.py
git commit -m "feat: yaw-drift rolling estimator + live-mode IMU staleness watchdog"
```

---

## Task 26: README update + manual test checklist

**Files:**
- Modify: `README.md` (create if missing) — a short section on running the new mode.

- [ ] **Step 1: Append Experiments-mode section to `README.md`**

Append (or create with):

```markdown
## Experiments Mode

A fourth controller mode that lets you pick a 3D target and drive the trunk tip
toward it via two phases: (1) inflate all modules to the target arc length using
the existing PID pressure loops; (2) close the loop on IMU orientation by
winding four antagonistic tendon servos.

### Running the test suite

```
pip install -r requirements-test.txt
pytest
```

### Running the UI without the rig

Launch normally (`python main.py`) without connecting a serial port. In
Experiments mode you'll see a `MODE: SIMULATED` badge and can fully exercise the
target picker, 3D preview, reach sequence, and E-stop against a synthetic
backend. This is safe for UI development and demos.

### First-time setup with the rig

1. Connect serial.
2. Go to Experiments → click **Recalibrate Length**.
3. Follow the per-module prompts (inflate, measure with a ruler, enter the
   length, repeat).
4. Calibration saves to `length_calibration.json` and persists across sessions.

### Known limitations

- Yaw axis drifts (no magnetometer on the MPU-6500). Re-zero between runs for
  best accuracy.
- Firmware protocol extensions for `SET <module_id> <hPa>` and
  `TEND <servo_id> <rate>` must be present on the Arduino side. If they're
  missing, the rig will not move — use SIMULATED mode for UI work until firmware
  is updated.
```

- [ ] **Step 2: Run the full test suite one final time**

Run: `pytest -v`
Expected: all tests pass. Note the total count.

- [ ] **Step 3: Final manual checklist (on disconnected machine)**

- [ ] Launch the app. All four radio buttons visible.
- [ ] Experiments mode selectable; panel renders with `MODE: SIMULATED` badge.
- [ ] Zero @ Rest → Confirm Zero sequence works; status updates.
- [ ] Click inside XY picker → target X/Y populate; XZ picker shows guide line.
- [ ] Click inside XZ picker → target Z populates.
- [ ] Out-of-bounds click flashes red briefly.
- [ ] Reach runs: 3D preview animates, readouts update, REACHED with small error.
- [ ] E-stop at any time returns to IDLE and halts motion.
- [ ] Unreachable target shows rejection message.
- [ ] Switching to Burst/PID/Calibration and back preserves the zero.

- [ ] **Step 4: Commit**

```bash
git add README.md
git commit -m "docs: README section for Experiments mode (setup, sim use, limitations)"
```

---

## Appendix: Open items surfaced by the spec

These are not plan tasks — they are questions the implementing engineer should flag if encountered:

1. **Firmware protocol extensions** — `SET <module_id> <hPa>` and `TEND <servo_id> <rate>` must exist on the Arduino side. If they don't, LiveBackend commands will be silently ignored by the rig. Add a graceful detection path (no response within N seconds after a known command → warn).
2. **IMU sample rate** — target is ≥40 Hz for a stable 20 Hz control loop. If the current firmware emits IMU lines slower, consider reducing controller KP or raising telemetry cadence.
3. **θ_max tuning** — default 60° is illustrative. A rig-side value may be smaller; tune per hardware.
4. **Tendon winding bounds** — `winding_max` not yet enforced as a hard clamp on integrated rotations in this plan; if you see runaway during live testing, add an integrator clamp in `_tick_bending`.

