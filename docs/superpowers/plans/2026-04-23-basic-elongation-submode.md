# Basic Elongation Sub-Mode Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a Basic Elongation sub-mode to Experiments mode that drives the arm up the Z axis using a P-controller on max-balloon-PSI feeding a common-mode tendon-servo unwind, with no IMU, no bending, and no kinematics.

**Architecture:** A new `ExperimentMode` enum on `ExperimentController` selects between the existing `COMPLEX` two-phase reach and a new `BASIC_ELONGATION` path. Basic path uses `reach_basic(z, threshold)`, a new `_tick_basic_elongating` method, and servo defaults latched at `confirm_zero()`. The panel grows a sub-mode radio and a Basic body frame that replaces the XY/XZ pickers and 3D preview when Basic is selected.

**Tech Stack:** Python 3.11+, tkinter, pytest. No new third-party deps.

**Spec:** `docs/superpowers/specs/2026-04-23-basic-elongation-submode-design.md`

---

## File Touchpoints

| File | Change |
|---|---|
| `experiment_controller.py` | Add `ExperimentMode`, `mode` field, `_servo_defaults` latch at `confirm_zero`, `reach_basic()`, `_tick_basic_elongating()`, new constants, `tick()` dispatcher. |
| `panels/experiment_panel.py` | Add sub-mode radio, Basic body frame (Z entry, threshold entry, readouts), show/hide logic. |
| `app.py` | Pass servo_defaults into `confirm_zero`; route `_exp_reach` by sub-mode; add `_exp_reach_basic` handler. |
| `tests/test_experiment_controller.py` | 9 new tests for Basic-mode behavior. |

No changes to firmware, backends, IPC, schemas, or calibration files.

---

## Task 1: Add ExperimentMode enum and controller `mode` field

**Files:**
- Modify: `experiment_controller.py` (top of file)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_experiment_controller.py`:

```python
def test_controller_defaults_to_complex_mode(ctrl):
    from experiment_controller import ExperimentMode
    assert ctrl.mode == ExperimentMode.COMPLEX


def test_controller_mode_can_be_set_to_basic(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    assert ctrl.mode == ExperimentMode.BASIC_ELONGATION
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_experiment_controller.py::test_controller_defaults_to_complex_mode -v`
Expected: FAIL with `ImportError` or `AttributeError` (no `ExperimentMode`).

- [ ] **Step 3: Add the enum and field**

In `experiment_controller.py`, near the top (after the `State` enum, before `RunResult`):

```python
class ExperimentMode(enum.Enum):
    COMPLEX = "COMPLEX"
    BASIC_ELONGATION = "BASIC_ELONGATION"
```

In `ExperimentController.__init__`, after `self.state = State.IDLE`:

```python
        self.mode: ExperimentMode = ExperimentMode.COMPLEX
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS (including the new two).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: add ExperimentMode enum (COMPLEX default, BASIC_ELONGATION)"
```

---

## Task 2: Latch `servo_defaults` at `confirm_zero`

**Files:**
- Modify: `experiment_controller.py` (`confirm_zero` method around line 82)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_experiment_controller.py`:

```python
def test_confirm_zero_latches_servo_defaults(ctrl):
    ctrl.start_zeroing()
    defaults = {1: 30.0, 2: 40.0, 3: 50.0, 4: 60.0}
    ctrl.confirm_zero(servo_defaults=defaults)
    assert ctrl._servo_defaults == defaults


def test_confirm_zero_without_servo_defaults_leaves_them_none(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero()
    assert ctrl._servo_defaults is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_experiment_controller.py::test_confirm_zero_latches_servo_defaults -v`
Expected: FAIL — `confirm_zero()` doesn't take a `servo_defaults` kwarg yet.

- [ ] **Step 3: Modify `confirm_zero`**

In `experiment_controller.py`, change the `confirm_zero` signature and body:

```python
    def confirm_zero(self, servo_defaults: Optional[Dict[int, float]] = None) -> None:
        if self.state != State.ZEROING:
            return
        self.backend.capture_zero()
        pressures = self.backend.read_state().module_pressures_psi
        self._psi_baseline = {
            mid: p for mid, p in pressures.items() if mid not in self.EXCLUDED_MODULES
        }
        if servo_defaults is not None:
            self._servo_defaults = dict(servo_defaults)
        self.state = State.WAITING_FOR_TARGET
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: latch servo_defaults at confirm_zero (shared by both modes)"
```

---

## Task 3: `reach_basic` skeleton (state + guards, no tick logic yet)

**Files:**
- Modify: `experiment_controller.py`
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_experiment_controller.py`:

```python
def test_reach_basic_requires_waiting_state(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    # Still IDLE — reach_basic should reject.
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_requires_basic_mode(ctrl):
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    # Mode still COMPLEX — reach_basic should reject.
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_requires_servo_defaults(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero()  # no servo_defaults
    with pytest.raises(ValueError):
        ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)


def test_reach_basic_transitions_to_elongating(ctrl):
    from experiment_controller import ExperimentMode, State
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)
    assert ctrl.state == State.ELONGATING
    assert ctrl._basic_z_target_mm == 30.0
    assert ctrl._basic_psi_threshold == 15.0
    assert ctrl._basic_slack_deg == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v -k reach_basic`
Expected: FAIL — `reach_basic` doesn't exist.

- [ ] **Step 3: Add `reach_basic` and the basic-mode state fields**

In `experiment_controller.py`, add these fields at the end of `__init__`:

```python
        # Basic-mode state.
        self._basic_z_target_mm: float = 0.0
        self._basic_psi_threshold: float = 15.0
        self._basic_slack_deg: float = 0.0
        self._basic_elongation_mm: float = 0.0
```

Add `reach_basic` as a new method on `ExperimentController` (place it right after the existing `reach` method):

```python
    def reach_basic(self, z_target_mm: float, psi_threshold: float) -> None:
        """Begin a Basic Elongation run.

        Steppers keep inflating; the 4 tendon servos share a common-mode
        unwind driven by a P-controller on max-balloon-psi vs. threshold.
        Elongation is integrated from slack angle x pulley radius. Reach
        declared when elongation >= z_target_mm.
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
        self._basic_z_target_mm = float(z_target_mm)
        self._basic_psi_threshold = float(psi_threshold)
        self._basic_slack_deg = 0.0
        self._basic_elongation_mm = 0.0
        self._phase_start = time.monotonic()
        self.state = State.ELONGATING
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v -k reach_basic`
Expected: all 4 reach_basic tests PASS.

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: add reach_basic() with state/mode/defaults guards"
```

---

## Task 4: Basic unwind P-controller

**Files:**
- Modify: `experiment_controller.py` (add constants + `_tick_basic_elongating` + dispatch)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write failing tests**

Append to `tests/test_experiment_controller.py`:

```python
def test_basic_unwind_zero_when_under_threshold(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0)
    # All pressures default to 0 in sim -> well under 15 psi threshold.
    for _ in range(10):
        ctrl.tick(dt=0.1)
    assert ctrl._basic_slack_deg == 0.0


def test_basic_unwind_proportional_to_overshoot(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0)  # big Z so no early REACHED
    # Force one module 1 psi above threshold.
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
    # Force 10 psi overshoot -> kp*10=100 deg/s, but clamp is 30 deg/s.
    ctrl.backend._pressures[1] = 25.0
    ctrl.tick(dt=0.1)
    # clamped to 30 deg/s * 0.1 s = 3.0 deg.
    assert ctrl._basic_slack_deg == pytest.approx(-3.0, abs=1e-6)


def test_basic_unwind_gated_on_max_psi_across_all_balloons(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0)
    # Only balloon 5 is above threshold. Should still unwind.
    for mid in range(1, 7):
        ctrl.backend._pressures[mid] = 5.0
    ctrl.backend._pressures[5] = 17.0  # +2 overshoot
    ctrl.tick(dt=0.1)
    assert ctrl._basic_slack_deg == pytest.approx(-2.0, abs=1e-6)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v -k basic_unwind`
Expected: all 4 FAIL — `_basic_slack_deg` never changes (no tick logic yet).

- [ ] **Step 3: Add constants and `_tick_basic_elongating`**

In `experiment_controller.py`, add after `INFLATION_RATE_PER_S`:

```python
    # Basic Elongation sub-mode tuning.
    BASIC_UNWIND_KP_DEG_PER_S_PER_PSI = 10.0   # unwind rate per psi of overshoot
    BASIC_UNWIND_MAX_DEG_PER_S = 30.0           # clamp on unwind rate
```

Add a new method `_tick_basic_elongating` (place it right after `_tick_elongating`):

```python
    def _tick_basic_elongating(self, dt: float) -> None:
        """Basic-mode elongation tick: P-controller on max-psi -> tendon unwind."""
        pressures = self.backend.read_state().module_pressures_psi
        max_psi = max(pressures.values()) if pressures else 0.0
        err = max(0.0, max_psi - self._basic_psi_threshold)
        rate = min(
            self.BASIC_UNWIND_KP_DEG_PER_S_PER_PSI * err,
            self.BASIC_UNWIND_MAX_DEG_PER_S,
        )
        self._basic_slack_deg -= rate * dt
        # Write composed absolute angles to all 4 tendon servos.
        if self._servo_defaults is not None:
            for sid in (1, 2, 3, 4):
                default = self._servo_defaults.get(sid, 0.0)
                angle = default + self._basic_slack_deg
                self.last_tendon_angles[sid] = angle
                self.backend.set_tendon_angle(sid, angle)
```

Modify `tick` to dispatch by mode. Replace the existing `if self.state == State.ELONGATING:` block with:

```python
        if self.state == State.ELONGATING:
            if self.mode == ExperimentMode.BASIC_ELONGATION:
                self._tick_basic_elongating(dt)
            else:
                self._advance_inflation_ramp(dt)
                self._tick_elongating()
        elif self.state == State.BENDING:
            self._tick_bending()
```

Also gate the existing angle-control block at the bottom of `tick` so it doesn't double-write in Basic mode. Replace:

```python
        # Angle-control path: whenever defaults + baseline are latched, write
        # composed tendon angles each tick ...
        if self._servo_defaults is not None and self._psi_baseline is not None:
            self._update_tendon_angles()
```

with:

```python
        # Complex-mode angle-control path. Basic mode writes its own angles in
        # _tick_basic_elongating() and must not double-write via _update_tendon_angles,
        # which uses Complex-mode slack/bend formulas.
        if (
            self.mode == ExperimentMode.COMPLEX
            and self._servo_defaults is not None
            and self._psi_baseline is not None
        ):
            self._update_tendon_angles()
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS (old + new).

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: basic-mode P-unwind on max-psi vs threshold (kp=10, clamp=30)"
```

---

## Task 5: Elongation integrated from slack angle × pulley radius

**Files:**
- Modify: `experiment_controller.py` (`_tick_basic_elongating`)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_experiment_controller.py`:

```python
def test_basic_elongation_derived_from_slack_and_pulley_radius(ctrl):
    from experiment_controller import ExperimentMode, ExperimentController
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=1000.0, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 16.0  # +1 psi overshoot
    for _ in range(5):  # 5 * 0.1s * 10 deg/s = -5 deg total slack
        ctrl.tick(dt=0.1)
    expected_mm = math.radians(5.0) * ExperimentController.PULLEY_RADIUS_MM
    assert ctrl._basic_elongation_mm == pytest.approx(expected_mm, rel=1e-6)


def test_basic_elongation_mm_property_available(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=100.0, psi_threshold=15.0)
    assert ctrl.basic_elongation_mm() == 0.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v -k basic_elongation`
Expected: FAIL — `_basic_elongation_mm` never updated; `basic_elongation_mm()` missing.

- [ ] **Step 3: Update `_tick_basic_elongating` and add accessor**

In `experiment_controller.py`, at the end of `_tick_basic_elongating`, after the servo write loop, add:

```python
        # Integrate elongation from slack angle (radians) * pulley radius (mm).
        self._basic_elongation_mm = (
            math.radians(-self._basic_slack_deg) * self.PULLEY_RADIUS_MM
        )
```

Add a public accessor right below `_tick_basic_elongating`:

```python
    def basic_elongation_mm(self) -> float:
        """Current integrated elongation in mm (Basic mode only)."""
        return self._basic_elongation_mm
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: integrate basic-mode elongation from slack angle + pulley radius"
```

---

## Task 6: REACHED transition when elongation ≥ Z target

**Files:**
- Modify: `experiment_controller.py` (`_tick_basic_elongating`)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_experiment_controller.py`:

```python
def test_basic_reaches_when_elongation_hits_target(ctrl):
    from experiment_controller import ExperimentMode, State, ExperimentController
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    # Target = 5mm elongation -> slack_deg needed = 5 / r / (pi/180)
    z_target = 5.0
    ctrl.reach_basic(z_target_mm=z_target, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 25.0  # big overshoot -> clamped at 30 deg/s
    # 30 deg/s * pi/180 * 25 mm = ~13 mm/s -> ~0.4s to hit 5mm.
    for _ in range(40):
        ctrl.tick(dt=0.05)
        if ctrl.state == State.REACHED:
            break
    assert ctrl.state == State.REACHED
    assert ctrl._basic_elongation_mm >= z_target


def test_basic_no_bending_state_ever_reached(ctrl):
    from experiment_controller import ExperimentMode, State
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=5.0, psi_threshold=15.0)
    ctrl.backend._pressures[1] = 25.0
    for _ in range(200):
        ctrl.tick(dt=0.05)
        assert ctrl.state != State.BENDING  # must never enter BENDING
        if ctrl.state == State.REACHED:
            break
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v -k basic_reaches`
Expected: FAIL — state never transitions to REACHED.

- [ ] **Step 3: Add REACHED transition and pressure freeze**

In `experiment_controller.py`, at the end of `_tick_basic_elongating`, add:

```python
        if self._basic_elongation_mm >= self._basic_z_target_mm:
            self._hold_pressures_at_current()
            elapsed = time.monotonic() - self._phase_start
            self._last_result = RunResult(
                final_pitch_err_rad=0.0,
                final_position_err_mm=0.0,
                elapsed_s=elapsed,
                timed_out=False,
            )
            self.state = State.REACHED
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: basic-mode REACHED on elongation >= z_target (freeze pressures)"
```

---

## Task 7: Flat per-module pressure setpoint at reach_basic

**Files:**
- Modify: `experiment_controller.py` (`reach_basic`)
- Test: `tests/test_experiment_controller.py`

- [ ] **Step 1: Write the failing tests**

Append to `tests/test_experiment_controller.py`:

```python
def test_basic_reach_commands_flat_pressure_setpoint(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0, pressure_ceiling_psi=20.0)
    # Every (non-excluded) module should have been set to 20.0 psi.
    for mid in range(1, 7):
        assert ctrl.backend._pressures[mid] == 20.0


def test_basic_reach_falls_back_to_threshold_plus_5_if_no_ceiling(ctrl):
    from experiment_controller import ExperimentMode
    ctrl.mode = ExperimentMode.BASIC_ELONGATION
    ctrl.start_zeroing()
    ctrl.confirm_zero(servo_defaults={1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0})
    ctrl.reach_basic(z_target_mm=30.0, psi_threshold=15.0)  # no ceiling
    for mid in range(1, 7):
        assert ctrl.backend._pressures[mid] == 20.0  # threshold + 5
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `pytest tests/test_experiment_controller.py -v -k basic_reach_commands`
Expected: FAIL — `reach_basic` doesn't take `pressure_ceiling_psi`, doesn't command pressures.

- [ ] **Step 3: Extend `reach_basic`**

In `experiment_controller.py`, change the `reach_basic` signature and add pressure commands before the state transition. Replace the existing `reach_basic` body with:

```python
    def reach_basic(
        self,
        z_target_mm: float,
        psi_threshold: float,
        pressure_ceiling_psi: Optional[float] = None,
    ) -> None:
        """Begin a Basic Elongation run.

        Steppers keep inflating; the 4 tendon servos share a common-mode
        unwind driven by a P-controller on max-balloon-psi vs. threshold.
        Elongation is integrated from slack angle x pulley radius. Reach
        declared when elongation >= z_target_mm.
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
        self._basic_z_target_mm = float(z_target_mm)
        self._basic_psi_threshold = float(psi_threshold)
        self._basic_slack_deg = 0.0
        self._basic_elongation_mm = 0.0
        self._pressure_ceiling_psi = pressure_ceiling_psi
        # Flat per-module setpoint. Operator ceiling wins; otherwise threshold + 5.
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
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `pytest tests/test_experiment_controller.py -v`
Expected: all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add experiment_controller.py tests/test_experiment_controller.py
git commit -m "controller: basic reach commands flat psi setpoint (ceiling or threshold+5)"
```

---

## Task 8: Panel — sub-mode radio and Basic body frame

**Files:**
- Modify: `panels/experiment_panel.py`

UI is hard to TDD cleanly; this task is a carefully-written structural change with a smoke check at the end. No new unit tests.

- [ ] **Step 1: Add StringVar for sub-mode + `on_reach_basic` callback slot**

In `panels/experiment_panel.py`, modify `ExperimentPanel.__init__`:

Change the signature from:

```python
    def __init__(
        self,
        parent,
        on_start_zero,
        on_confirm_zero,
        on_rezero,
        on_reach,
        on_emergency_stop,
        **kwargs,
    ):
```

to:

```python
    def __init__(
        self,
        parent,
        on_start_zero,
        on_confirm_zero,
        on_rezero,
        on_reach,
        on_reach_basic,
        on_emergency_stop,
        **kwargs,
    ):
```

Store the new callback after the other `self._on_*` assignments:

```python
        self._on_reach_basic = on_reach_basic
```

Add these StringVars near the other StringVar declarations (right after `self.status_var = ...`):

```python
        self.submode_var = tk.StringVar(value="BASIC")
        self.basic_z_var = tk.StringVar(value="20.0")
        self.basic_threshold_var = tk.StringVar(value="15.0")
        self.basic_elongation_readout_var = tk.StringVar(value="Elongation: 0.0 / 0.0 mm")
        self.basic_slack_readout_var = tk.StringVar(value="Slack: 0.0 deg")
        self.basic_maxpsi_readout_var = tk.StringVar(value="Max psi: 0.0")
```

- [ ] **Step 2: Add the sub-mode radio row in `_build` before the Zero buttons**

In `panels/experiment_panel.py`, locate the `_build` method. After the `EXPERIMENTS` header label (`tk.Label(self, text="EXPERIMENTS", ...)` call), insert:

```python
        # Sub-mode selector: Basic (scalar Z, no IMU) vs Complex (XYZ + bending).
        mode_row = tk.Frame(self, bg=BG_PANEL)
        mode_row.pack(fill="x", padx=10, pady=(4, 0), side="top")
        tk.Label(mode_row, text="Sub-mode:", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Radiobutton(
            mode_row, text="Basic Elongation", variable=self.submode_var, value="BASIC",
            command=self._on_submode_change,
            font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
            selectcolor=BG_PANEL, activebackground=BG_PANEL,
        ).pack(side="left", padx=(8, 0))
        tk.Radiobutton(
            mode_row, text="Complex", variable=self.submode_var, value="COMPLEX",
            command=self._on_submode_change,
            font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
            selectcolor=BG_PANEL, activebackground=BG_PANEL,
        ).pack(side="left", padx=(8, 0))
```

- [ ] **Step 3: Build the Basic body frame**

In `_build`, after the existing target-entry row (`tgt_row`) and before the canvas row (`self.canvas_row = ...`), insert a new frame:

```python
        # Basic body frame (shown when sub-mode == BASIC; hides the canvas row).
        self.basic_body = tk.Frame(self, bg=BG_PANEL)
        self.basic_body.pack(fill="both", expand=True, padx=10, pady=4, side="top")

        basic_entry = tk.Frame(self.basic_body, bg=BG_PANEL)
        basic_entry.pack(fill="x", pady=6)
        tk.Label(basic_entry, text="Z (mm):", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(basic_entry, textvariable=self.basic_z_var, width=8,
                 font=FONT_DATA).pack(side="left", padx=(4, 12))
        tk.Label(basic_entry, text="PSI threshold:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(basic_entry, textvariable=self.basic_threshold_var, width=6,
                 font=FONT_DATA).pack(side="left", padx=(4, 12))
        AccentButton(basic_entry, text="Reach", accent=ACCENT_GREEN,
                     command=self._handle_reach_basic).pack(side="left", padx=12)

        basic_readouts = tk.Frame(self.basic_body, bg=BG_PANEL)
        basic_readouts.pack(fill="x", pady=(4, 2))
        for var in (self.basic_elongation_readout_var,
                    self.basic_slack_readout_var,
                    self.basic_maxpsi_readout_var):
            tk.Label(basic_readouts, textvariable=var, font=FONT_BODY,
                     fg=TEXT_PRIMARY, bg=BG_PANEL).pack(anchor="w")
```

- [ ] **Step 4: Add show/hide logic and the Reach-basic handler**

Still in `panels/experiment_panel.py`, add these methods on `ExperimentPanel`:

```python
    def _on_submode_change(self) -> None:
        """Show Basic body frame (hide pickers/preview) or vice versa."""
        if self.submode_var.get() == "BASIC":
            self.canvas_row.pack_forget()
            self.basic_body.pack(fill="both", expand=True, padx=10, pady=4, side="top")
        else:
            self.basic_body.pack_forget()
            self.canvas_row.pack(fill="both", expand=True, padx=10, pady=4, side="top")

    def _handle_reach_basic(self) -> None:
        try:
            z = float(self.basic_z_var.get())
            threshold = float(self.basic_threshold_var.get())
        except ValueError:
            self.status_var.set("Invalid input - Z and threshold must be numbers")
            return
        self._on_reach_basic(z, threshold)

    def set_basic_readouts(self, elongation_mm: float, z_target_mm: float,
                           slack_deg: float, max_psi: float) -> None:
        self.basic_elongation_readout_var.set(
            f"Elongation: {elongation_mm:.1f} / {z_target_mm:.1f} mm"
        )
        self.basic_slack_readout_var.set(f"Slack: {slack_deg:.1f} deg")
        self.basic_maxpsi_readout_var.set(f"Max psi: {max_psi:.1f}")

    def current_submode(self) -> str:
        return self.submode_var.get()
```

At the end of `_build`, after all widgets are packed, add a call so Basic is the default view:

```python
        # Default sub-mode is BASIC -> hide the canvas row on first render.
        self._on_submode_change()
```

- [ ] **Step 5: Verify the panel still imports and instantiates**

Run: `pytest tests/ -v`
Expected: the existing `test_panel_instantiates_offline` (or equivalent) still passes; the app imports cleanly.

If `test_panel_instantiates_offline` exists and was calling `ExperimentPanel(...)` without `on_reach_basic`, fix that call site too — it will error with a TypeError.

Run: `python -c "import panels.experiment_panel"` — expected: no output (clean import).

- [ ] **Step 6: Commit**

```bash
git add panels/experiment_panel.py tests/
git commit -m "panel: sub-mode radio + Basic body frame (Z + threshold + readouts)"
```

---

## Task 9: Wire Basic mode into `app.py`

**Files:**
- Modify: `app.py`

- [ ] **Step 1: Update `_exp_confirm_zero` to pass servo defaults in**

In `app.py`, find `_exp_confirm_zero` (near line 528). Replace its body so it reads the burst-panel servo slider values and passes them to the controller:

```python
    def _exp_confirm_zero(self):
        try:
            servo_defaults = {
                sid: int(self.burst_panel._servo_vars[sid - 1].get())
                for sid in (1, 2, 3, 4)
            }
        except (AttributeError, IndexError):
            servo_defaults = None
        self.experiment_controller.confirm_zero(servo_defaults=servo_defaults)
        self.experiment_panel.set_status("Zero captured - pick a target")
```

- [ ] **Step 2: Pass `on_reach_basic` into the panel constructor**

Locate the `ExperimentPanel(...)` instantiation (around line 344) and add the new callback:

```python
        self.experiment_panel = ExperimentPanel(
            self.panel_container,
            on_start_zero=self._exp_start_zero,
            on_confirm_zero=self._exp_confirm_zero,
            on_rezero=self._exp_rezero,
            on_reach=self._exp_reach,
            on_reach_basic=self._exp_reach_basic,
            on_emergency_stop=self._exp_emergency_stop,
        )
```

- [ ] **Step 3: Add `_exp_reach_basic` handler and sync mode at panel level**

In `app.py`, add a new method near `_exp_reach`:

```python
    def _exp_reach_basic(self, z_target_mm: float, psi_threshold: float):
        from experiment_controller import ExperimentMode
        self.experiment_controller.mode = ExperimentMode.BASIC_ELONGATION
        from tkinter import simpledialog
        ceiling = simpledialog.askfloat(
            "Reach - Pressure Ceiling",
            "Max per-module pressure (psi absolute):",
            parent=self.root, minvalue=14.0, maxvalue=25.0, initialvalue=18.0,
        )
        if ceiling is None:
            self.experiment_panel.set_status("Reach cancelled (no ceiling)")
            return
        try:
            self.experiment_controller.reach_basic(
                z_target_mm=z_target_mm,
                psi_threshold=psi_threshold,
                pressure_ceiling_psi=ceiling,
            )
            self.experiment_panel.set_status(
                f"Basic Reach: Z={z_target_mm:.1f}mm threshold={psi_threshold:.1f}psi (<={ceiling:.1f})"
            )
        except ValueError as e:
            self.experiment_panel.set_status(f"Rejected: {e}")
            return
        if not self.logger.is_active:
            self.start_log()
            self._exp_auto_logging = True
        self._exp_prev_state = self.experiment_controller.state
        self._exp_prev_connected = bool(self.arduino.ser and self.arduino.ser.is_open)
```

- [ ] **Step 4: Keep Complex mode mode-synced when `_exp_reach` is called**

In `_exp_reach` (around line 538), at the very top of the method body, add:

```python
        from experiment_controller import ExperimentMode
        self.experiment_controller.mode = ExperimentMode.COMPLEX
```

- [ ] **Step 5: Update the readout path to include Basic readouts**

Locate the update-loop section that calls `self.experiment_panel.set_readouts(...)` (around line 939). After that call, add a block that updates Basic readouts when the sub-mode is BASIC:

```python
            if self.experiment_panel.current_submode() == "BASIC":
                pressures = s.module_pressures_psi
                max_psi = max(pressures.values()) if pressures else 0.0
                self.experiment_panel.set_basic_readouts(
                    elongation_mm=self.experiment_controller.basic_elongation_mm(),
                    z_target_mm=self.experiment_controller._basic_z_target_mm,
                    slack_deg=self.experiment_controller._basic_slack_deg,
                    max_psi=max_psi,
                )
```

- [ ] **Step 6: Run the full test suite**

Run: `pytest tests/ -v`
Expected: all tests PASS.

- [ ] **Step 7: Commit**

```bash
git add app.py
git commit -m "app: wire Basic sub-mode (confirm_zero latch, reach_basic, readouts)"
```

---

## Task 10: Manual smoke test

**Files:** none

Validate the end-to-end flow in simulated mode.

- [ ] **Step 1: Launch the app**

Run: `python main.py`
Expected: GUI opens; "MODE: SIMULATED" badge visible (no serial connection).

- [ ] **Step 2: Navigate to Experiments tab**

Click the **Experiments** tab in the left nav.
Expected: panel shows the new "Sub-mode: [Basic Elongation] [Complex]" radio row. Basic is selected by default. Canvas row (XY / XZ / 3D) is hidden. Basic body frame with Z entry and PSI threshold entry is visible.

- [ ] **Step 3: Basic reach flow**

1. Click **Zero @ Rest**, then **Confirm Zero**. Status should read "Zero captured".
2. Enter Z = 10 mm, threshold = 15 psi (default).
3. Click **Reach**. A pressure-ceiling dialog appears; accept default (18).
4. Status should read "Basic Reach: Z=10.0mm threshold=15.0psi (<=18.0)".
5. Watch the readouts. The sim backend inflates; once any balloon exceeds 15 psi, Slack goes negative and Elongation climbs.
6. When Elongation >= 10mm, phase label turns green and shows "REACHED". Pressures freeze.

- [ ] **Step 4: Switch sub-modes**

Click the **Complex** radio. Canvas row (pickers + preview) returns; Basic body frame hides.
Click **Basic Elongation** again. Basic body returns; canvas hides.

- [ ] **Step 5: E-stop**

During a Basic run, click **EMERGENCY STOP**. State drops to IDLE; steppers stop.

- [ ] **Step 6: Commit any fixes**

If any step failed and you had to patch a bug, commit the fix separately. If smoke test passed cleanly, nothing to commit.

```bash
# Only if fixes were needed:
git add -p
git commit -m "fix: <describe the issue discovered during smoke test>"
```

- [ ] **Step 7: Update the graphify knowledge graph**

Per project CLAUDE.md (`After modifying code files in this session, run graphify update .`):

Run: `graphify update .`
Expected: updates `graphify-out/` with the new nodes/edges.

```bash
git add graphify-out/
git commit -m "graphify: update after basic-elongation sub-mode implementation"
```

---

## Self-Review Notes (already applied)

- All 9 spec tests appear in Tasks 1-7; additionally, Task 8/9 produce working UI.
- No placeholders; every code step shows the exact code.
- Type consistency: `servo_defaults: Dict[int, float]`, `z_target_mm: float`, `psi_threshold: float`, `pressure_ceiling_psi: Optional[float]` used uniformly.
- `_basic_slack_deg`, `_basic_elongation_mm`, `_basic_z_target_mm`, `_basic_psi_threshold` named consistently across tasks.
- `reach_basic` signature extended in Task 7 (adds `pressure_ceiling_psi`) — Task 3's skeleton test still works because it passes only the first two args; Task 9 passes all three.
- No references to `set_orientation_target` or kinematics in Basic path — confirmed via the dispatch guard in Task 4.
