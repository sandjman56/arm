# Basic Elongation Sub-Mode — Design

**Date:** 2026-04-23
**Status:** Draft (pre-implementation)

## Purpose

Split Experiments mode into two sub-modes so elongation and bending can be exercised independently:

1. **Basic Elongation Testing** — scalar Z target, no bending, no IMU. This spec.
2. **Complex** — today's two-phase (elongate → bend) behavior with the XYZ picker. Untouched.

Basic mode is for rig bring-up and elongation-only experiments where orientation control is off the table. It exercises the stepper-and-tendon interaction alone.

## Behavior

### User flow
1. Operator opens Experiments mode and selects the **Basic Elongation** sub-mode.
2. Operator presses **Zero @ Rest**, then **Confirm Zero**. At Confirm Zero, the controller latches both the per-module PSI baseline (as today) **and** the current tendon-servo angles as the new `_servo_defaults`.
3. Operator enters a scalar **Z (mm)** target and an optional **PSI threshold** (default 15.0).
4. Operator presses **Reach**. The controller transitions `WAITING_FOR_TARGET → ELONGATING`.
5. During ELONGATING:
   - Steppers continue their existing synchronized-inflation ramp until the final per-module pressures are reached (or they hold above the operator's pressure ceiling, unchanged from today).
   - A P-controller on the **maximum balloon pressure across all 6 modules** drives a common-mode tendon-servo unwind. All 4 tendon servos share a single slack scalar and unwind together.
   - Tendon pay-out is integrated into a scalar `elongation_mm` derived from slack angle × `PULLEY_RADIUS_MM`.
6. When `elongation_mm ≥ z_target`, the controller freezes pressures and servo angles and transitions to `REACHED`. UI shows the REACHED badge.
7. Emergency stop behaves exactly as in Complex mode.

### Control law (Basic ELONGATING tick)

```
max_psi   = max(backend.module_pressures_psi.values())
err_psi   = max(0.0, max_psi - psi_threshold)
unwind    = min(BASIC_UNWIND_KP_DEG_PER_S_PER_PSI * err_psi,
                BASIC_UNWIND_MAX_DEG_PER_S)         # deg/s, >= 0

slack_deg         -= unwind * dt                    # slack becomes more negative
elongation_mm      = math.radians(-slack_deg) * PULLEY_RADIUS_MM

for sid in (1..4):
    backend.set_tendon_angle(sid, servo_defaults[sid] + slack_deg)

if elongation_mm >= z_target:
    hold_pressures_at_current()
    state = REACHED
```

Constants:
- `BASIC_UNWIND_KP_DEG_PER_S_PER_PSI = 10.0` (1 psi overshoot ⇒ 10 °/s unwind)
- `BASIC_UNWIND_MAX_DEG_PER_S = 30.0` (clamp so no violent unwinds)
- `PULLEY_RADIUS_MM = 25.0` (existing constant, reused)

### Why P-control on max-psi

The user's stated intent: "unwind slowly so psi stays around 15" and "if any balloon is over 15 psi, lower briefly until it isn't." A proportional law on `max(0, max_psi − threshold)` satisfies both — rate scales with overshoot, hits zero at threshold, and bounds unwind speed at the clamp. Bang-bang was considered and rejected: it would oscillate around the threshold and require hand-tuning a single rate.

### Why max-across-balloons (not per-module)

There is only one common-mode slack scalar (all 4 tendon servos share it during elongation, matching today's `_compute_slack_deg`). So the gate must collapse 6 balloon pressures into a single scalar. "Max" is the only safe choice — if any balloon is above threshold, we need to relieve it.

## Architecture

```
ExperimentPanel
  ├── Sub-mode radio (Basic / Complex)  ─────── drives which body frame is shown
  ├── [Basic body frame]    ← shown when BASIC selected
  │     ├── Z entry, PSI threshold entry
  │     ├── Reach button
  │     └── Readouts: elongation mm, servo slack °, max psi, phase
  └── [Complex body frame]  ← shown when COMPLEX selected  (today's pickers + preview)

ExperimentController
  ├── mode: ExperimentMode  (BASIC_ELONGATION | COMPLEX, default COMPLEX)
  ├── reach(target_xyz, servo_defaults, pressure_ceiling)     ← COMPLEX only
  ├── reach_basic(z_target_mm, psi_threshold)                 ← BASIC only
  ├── confirm_zero()  → latches psi_baseline + servo_defaults (both modes)
  ├── _tick_elongating()         ← COMPLEX (today's logic, unchanged)
  └── _tick_basic_elongating()   ← BASIC (new)
```

### File touchpoints

- `experiment_controller.py` — add `ExperimentMode`, `reach_basic`, `_tick_basic_elongating`, constants; `tick()` dispatches to the right ELONGATING handler by mode; `confirm_zero()` reads servo angles from the backend.
- `experiment_backend.py` — expose a `read_tendon_angles() -> Dict[int, float]` on `Backend` (LiveBackend reads from the arduino interface's last known command; SimBackend returns its internal state). Both backends already track these.
- `panels/experiment_panel.py` — add sub-mode radio, build the Basic body frame, hide pickers/preview in Basic.
- `app.py` — route the Reach callback by sub-mode (basic path skips kinematics validation; existing path unchanged).
- `tests/test_experiment_controller.py` — new Basic-mode coverage (see Tests section).

No schema, IPC, or firmware changes.

## Data flow

1. **Confirm Zero** — controller reads `module_pressures_psi` (PSI baseline) and now also `read_tendon_angles()` (servo defaults). Both stored on the controller.
2. **Reach (Basic)** — panel emits `(z_target_mm, psi_threshold)`. App calls `controller.reach_basic(z, threshold)` iff sub-mode is Basic. Controller: validates state, stores targets, starts `_inflation_final` ramp (reuse), sets `self.state = ELONGATING`.
3. **tick loop** — UI tick (~20 Hz) calls `controller.tick(dt)`. In Basic mode, `_tick_basic_elongating` runs; servo commands and pressure commands both go out through the existing backend methods.
4. **Readouts** — app update loop asks controller for `current_slack_deg()` (existing), `elongation_mm` (new property), `max_psi` (compute from backend state), and writes them to the Basic panel's StringVars.

## Error handling

- `reach_basic` called from wrong state ⇒ `ValueError` (same pattern as `reach`).
- `reach_basic` called in Complex mode (or `reach` called in Basic mode) ⇒ `ValueError`.
- Servo defaults missing at Reach (user skipped Confirm Zero) ⇒ `ValueError`.
- E-stop from any Basic state ⇒ IDLE, same as today.

## Testing

New unit tests in `tests/test_experiment_controller.py`:

1. `test_basic_reach_requires_waiting_state` — reach_basic raises from IDLE.
2. `test_basic_reach_requires_servo_defaults` — reach_basic raises when baseline/defaults unset.
3. `test_basic_unwind_zero_when_under_threshold` — all balloons < threshold, slack stays at 0.
4. `test_basic_unwind_proportional_to_overshoot` — balloon at threshold+1 psi ⇒ 10°/s unwind; at threshold+10 psi ⇒ clamped to 30°/s.
5. `test_basic_elongation_derived_from_slack_and_pulley_radius` — after N ticks with known unwind, elongation_mm ≈ |Δθ| · π/180 · r.
6. `test_basic_reaches_when_elongation_hits_target` — elongation ≥ z_target ⇒ REACHED.
7. `test_basic_no_bending_phase` — never enters BENDING in BASIC mode.
8. `test_confirm_zero_latches_servo_defaults` — after confirm_zero, controller has servo_defaults populated.
9. `test_basic_estop_returns_to_idle` — same E-stop contract.

## YAGNI / non-goals

- No per-module PSI gating (use max-across-all).
- No user-configurable P gain or max unwind rate (hardcoded).
- No new picker UI — scalar Z entry only.
- No timeout tuning knob beyond the existing one.
- No logger column additions beyond what's already emitted (existing `last_tendon_angles`, pressures, phase).
- Complex mode UI and logic are untouched.

## Pressure setpoint strategy (Basic mode)

Basic mode does not compute per-module pressure targets from a total arm length (there's no kinematics). Each module's setpoint is flat at the operator's pressure ceiling for the whole run. The servo-unwind loop is the regulator — as long as it's running, max-psi is pinned near the threshold, and the stepper PID just pumps until the servo "valve" stops needing air.

If no pressure ceiling is entered, the ramp uses `psi_threshold + 5.0` as the per-module setpoint so the firmware PID has a target above the threshold but not far above it.

## Open assumptions (flag if wrong)

- `SimBackend` and `LiveBackend` both already track commanded tendon angles internally; exposing a `read_tendon_angles() -> Dict[int, float]` getter is mechanical. To verify in implementation.
