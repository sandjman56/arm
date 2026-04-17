# Experiments Mode — Design

**Date:** 2026-04-17
**Status:** Approved design, ready for implementation plan

---

## 1. Purpose

Add a fourth mode — **Experiments** — to the Continuum Arm Controller, next to Burst, PID, and Calibration. It lets the operator:

1. Set the resting tip position as the coordinate-system origin.
2. Pick a target point in 3D space relative to that origin.
3. Command the arm to reach the target via a two-phase controller:
   - **Phase 1 — Elongation**: inflate all available modules using the existing per-module PID pressure loops until the trunk reaches the required total length.
   - **Phase 2 — Bending**: close the loop on IMU orientation, winding/unwinding the 4 tendon servos antagonistically until the tip orientation matches the vector toward the target.

The feature is motivated by wanting a concrete, repeatable way to characterize the arm's reachable workspace and closed-loop behavior.

---

## 2. Hardware assumptions

These assumptions define the model. Anything that contradicts them will require design changes.

- **Trunk**: 6 stacked "spines" with a bladder in each spine. Inflating a bladder extends that module's length.
- **Actuation**:
  - **Pneumatic**: one bladder per module, one pressure sensor per module (target configuration). The design tolerates missing sensors on some modules.
  - **Tendons**: 4 strings routed through holes in every spine, anchored **only to the tip spine**, spaced 90° apart around the circumference (0°, 90°, 180°, 270°).
  - **Servos**: 4 continuous-rotation servos on the top plate, each with a pulley horn winding one tendon.
- **Sensing**: one MPU-6500 (accel + gyro, no magnetometer) mounted on the tip spine.
- **Existing commands** (over serial, already implemented): `INFLATE`, `DEFLATE`, `STOP`, `SET <hPa>`, `SET_LIMITS`, `RESET_POS`, plus telemetry lines `PM,module_id,t,hPa,psi[,pos]` and `IMU,t,ax,ay,az,gx,gy,gz,tempC`.

Two items listed in this section are **not yet guaranteed by the firmware** and must be verified / added:

- **Per-module pressure targets**: the existing `SET <hPa>` command may not accept a module selector. Experiments mode needs per-module setpoints. Implementation will extend to `SET <module_id> <hPa>` if not present.
- **Per-servo tendon commands**: the firmware today exposes stepper `step/dir` pins per module, not tendon winding commands. Implementation will add `TEND <servo_id> <delta_deg>` or equivalent. Exact protocol TBD during implementation, but this design assumes 4 addressable tendon servos.

These are flagged again in §11 (Open Items).

---

## 3. Coordinate frames

Three frames coexist. Separating them is the main reason the rest of the design stays simple.

| Frame | Origin | Used for |
|---|---|---|
| **Physics frame** | Base plate center, Z along trunk neutral axis, X at tendon-0° azimuth | All kinematics math; reachable workspace bounds |
| **Display frame** | Tip position at the moment the user zeroed (at rest) | UI rendering, target picking, readouts |
| **IMU orientation reference** | Orientation captured at zero-press | Orientation error computation |

The display frame is a pure translation offset from the physics frame. The controller always runs in the physics frame; the offset is applied only at UI boundaries.

**Zeroing procedure** (automatic on mode entry):

1. Prompt the user to confirm trunk is fully deflated and tendons slack.
2. On confirm, capture current IMU orientation → reference orientation.
3. Compute tip position at rest in the physics frame as `(0, 0, L_rest)` where `L_rest = sum(module_rest_lengths)` from the length-calibration file.
4. Store this as the display-frame origin offset.

**Re-zero button**: available, but *guarded* — enabled only when all module pressures are below a "near-rest" threshold **and** gyro magnitude has been below a stillness threshold for at least 500 ms. Clicking while bent shows an error tooltip instead of re-zeroing.

**Yaw drift disclosure**: because the IMU has no magnetometer, integrated yaw drifts over time. The UI displays a running estimate of yaw drift rate (°/min over last 60 s) and suggests re-zeroing when the rate exceeds a threshold.

---

## 4. Kinematic model

### 4.1 Forward kinematics (current tip position from sensors)

Single constant-curvature (PCC) arc approximation:

- **Total length**: `L = sum(L_i)` where `L_i` is module *i*'s current length, inferred from its pressure via its calibrated curve (see §5). Modules without a live pressure sensor contribute their **rest length** (fixed).
- **Tip orientation**: from the IMU.
  - `pitch`, `roll` derived from gravity via the accelerometer (a gentle low-pass filter).
  - `yaw` derived from integrated gyro, initialized to 0 at zero.
  - Combined into a unit direction vector `d̂ = rotate((0,0,1), pitch, roll, yaw)`.
- **Arc model**: assume the trunk forms a planar arc from the base, ending at the tip with tangent `d̂`. Let `θ` = tilt magnitude (angle between `d̂` and base's +Z) and `φ` = azimuth of the bend plane (angle around Z axis). Standard PCC tip position:
  - If `θ ≈ 0` (nearly straight): `tip = (0, 0, L)`.
  - Otherwise, radius of curvature `R = L / θ`, and in-plane coordinates `(r_plane, z_plane) = (R * (1 − cos θ), R * sin θ)`. Rotate by `φ` around Z: `tip = (r_plane * cos φ, r_plane * sin φ, z_plane)`.
  - Full derivation and unit tests live in `kinematics.py`.

### 4.2 Inverse kinematics (target → commanded length + orientation)

Given a target `T = (tx, ty, tz)` in the physics frame:

1. Compute azimuth `φ* = atan2(ty, tx)`.
2. Compute horizontal reach `r = sqrt(tx² + ty²)` and vertical `h = tz`.
3. For a PCC arc of arclength `L` reaching `(r, h)`, there is a closed-form solution for `(L, θ)` — standard result. Details in `kinematics.py`.
4. If `L` exceeds `L_max` (maximum calibrated total length) or `θ` exceeds `θ_max` (see §6), the target is **unreachable**; the UI rejects it at picking time.

### 4.3 Reachable workspace

A half-dome in the physics frame:

- `L_min ≤ L ≤ L_max` where `L_min = sum(rest_lengths)` and `L_max = sum(fully_inflated_lengths)` from calibration.
- `0 ≤ θ ≤ θ_max` — `θ_max` is taken from an empirical safety limit (default 60°, editable in-UI).
- Azimuth unconstrained (0–360°).

Rendered in the display frame as a translucent dome. Target picks outside the dome are rejected.

---

## 5. Length calibration sub-mode

First-run (or on-demand via "Recalibrate Length") walkthrough, stored separately from the existing `calibration.json`:

1. Detect which modules have live pressure telemetry (via recent `PM,` lines).
2. For each live module, step through N pressure setpoints (default: 0, 25%, 50%, 75%, 100% of max PSI). For each:
   - Command the PID loop to hold that pressure.
   - Prompt operator: "Measure module length in mm and enter."
3. Fit a quadratic `length_i(psi) = a_i + b_i*psi + c_i*psi²` per module.
4. Save to `length_calibration.json`:

   ```json
   {
     "calibrated_at": "2026-04-17T...",
     "modules": {
       "1": {"rest_length_mm": 42.0, "coeffs": [a, b, c], "max_psi": 8.0},
       "2": {...},
       ...
     },
     "missing_modules": [5]
   }
   ```

5. Modules with missing sensors are recorded in `missing_modules` and treated as fixed-length (`rest_length_mm`) everywhere.

---

## 6. Controller state machine

```
   ┌────────┐  start   ┌─────────┐  done   ┌────────────────────┐
   │  IDLE  │─────────▶│ ZEROING │────────▶│ WAITING_FOR_TARGET │
   └────────┘          └─────────┘         └──────────┬─────────┘
       ▲                                              │ Reach clicked
       │                                              ▼
       │                                       ┌─────────────┐
       │                                       │ ELONGATING  │
       │                                       └──┬──────┬───┘
       │                                   done   │      │ timeout
       │                                          ▼      │
       │                                    ┌─────────┐  │
       │                                    │ BENDING │  │
       │                                    └──┬───┬──┘  │
       │                             reached   │   │ timeout
       │                                       ▼   ▼     ▼
       │  reset/new-target  ┌─────────┐   ┌───────────┐
       └────────────────────┤ REACHED │   │ TIMED_OUT │
                            └─────────┘   └─────┬─────┘
                                                │
   E-STOP from any state ─────────────────────▶ IDLE
```

**States:**

- **IDLE** — panel entered but not zeroed. Start button disabled.
- **ZEROING** — waiting on user to confirm rest state; captures IMU reference.
- **WAITING_FOR_TARGET** — origin locked; target picker active.
- **ELONGATING** — per-module PID pressure setpoints driven to hit the target `L`. Reuses existing PID pressure code. Exits when all participating modules are within pressure tolerance OR elongation timeout (default 10 s).
- **BENDING** — closed loop on IMU orientation. Exits on success tolerance met OR bend timeout (default 15 s) OR E-stop.
- **REACHED** — hold position briefly, report final error in the status pane.
- **TIMED_OUT** — halt all motion, report how close we got.

E-stop from any state → IDLE, all tendons halted, all pressures vented via existing `STOP`.

### 6.1 Bend-phase control law

Simplest working form: proportional control on orientation error, mapped to antagonistic tendon pairs.

- Current tip orientation (in physics frame, relative to IMU reference): unit vector `d̂_now`.
- Target orientation: `d̂_target = normalize(T − tip_base_offset)` — i.e., the direction from where the trunk exits the base to the target point.
- Error as a rotation axis-angle: `(axis_e, θ_e) = axis_angle_from(d̂_now → d̂_target)`.
- Project `axis_e` onto the X axis and Y axis of the physics frame → `e_x`, `e_y`.
- Command deltas for antagonistic pairs (continuous-rotation servo signed rotation rate):
  - `servo_0°   ← +K_p * e_x` (wind)
  - `servo_180° ← −K_p * e_x` (unwind, mirror)
  - `servo_90°  ← +K_p * e_y`
  - `servo_270° ← −K_p * e_y`
- Saturate each command at `±ω_max` (default matches servo max speed).
- Integrate winding commands over time into a total winding displacement per tendon; clamp the integral at `±winding_max` (hard limit derived from tendon slack budget) to prevent runaway if the loop ever fails to converge.

`K_p` tunable in-UI. Stop condition: `θ_e < tol_angle` (default 3°) AND kinematic tip-position error `< tol_pos` (default 10 mm), OR bend timeout, OR E-stop.

### 6.2 Why a timeout

Documented at length in the brainstorming transcript; summary: real rigs have friction/backlash/leaks, P-only loops can oscillate without converging, and continuous-rotation servos under sustained pull can damage tendons or heat themselves. Timeout converts "stuck" into actionable feedback (final error + elapsed time are logged).

---

## 7. UI layout

A new radio button **"Experiments"** is added to the mode row in `app.py` alongside Burst / PID / Calibration. Selecting it shows a new `ExperimentPanel` in the panel container.

```
┌─ EXPERIMENTS ─────────────────────────────────────────────────────────┐
│                                                                       │
│  [Zero @ Rest]  [Re-zero]  [Recalibrate Length]   Status: WAITING...  │
│                                                                       │
│  ┌────── XY (top) ──────┐ ┌────── XZ (side) ─────┐ ┌── 3D Preview ──┐ │
│  │                      │ │                      │ │                │ │
│  │      ·  tip          │ │       ·   tip        │ │   (rotatable   │ │
│  │     · target         │ │      · target        │ │    view-only)  │ │
│  │                      │ │       │ guide        │ │                │ │
│  │   [reachable circle] │ │   [reachable arc]    │ │                │ │
│  └──────────────────────┘ └──────────────────────┘ └────────────────┘ │
│                                                                       │
│  Target:  X [____]  Y [____]  Z [____]   [Reach]                      │
│                                                                       │
│  Tip (from zero):  (0.0, 0.0, 0.0)   Tip (from base): (0.0, 0.0, 142.0)│
│  Tip orient:  pitch 0.0°  roll 0.0°  yaw 0.0°   Yaw drift: 0.2°/min    │
│  Phase: BENDING    Elapsed: 3.4s    Err: 5.2° / 14mm                  │
│                                                                       │
│  [ EMERGENCY STOP ]                                                   │
└───────────────────────────────────────────────────────────────────────┘
```

**Target-picking interaction**:

1. Click on **XY** canvas → locks `(X, Y)`. Text fields update.
2. A vertical guide line appears on the **XZ** canvas at the locked X.
3. Click on **XZ** along the guide → locks `Z`. Text fields update.
4. Text fields accept direct entry; clicking inside the reachable zone updates the corresponding canvas.
5. Out-of-bounds clicks are rejected with a momentary red flash on the canvas.
6. **Reach** button enabled only when a valid target is set and state is `WAITING_FOR_TARGET` or `REACHED`.

3D preview uses `matplotlib` 3D axes (already in the stack, via the pressure graph). Renders: base plate, current trunk arc (from forward kinematics), target crosshair, reachable dome translucent.

---

## 7a. Offline / disconnected operation

The panel must be **fully usable with no serial connection** so the UI can be developed, tested, and demoed independently of the rig. Specifically:

- **Panel builds and renders unconditionally** when the Experiments radio button is selected. No check for `self.arduino.ser` gates the build.
- **Target picking (XY/XZ canvases, text entry, 3D preview) works offline.** Reachable workspace is drawn from whatever calibration data is on disk; if `length_calibration.json` is missing, fall back to a **default synthetic workspace** (e.g., `L_rest = 240 mm`, `L_max = 480 mm`, `θ_max = 60°`) and surface a small inline note: "Using default workspace — no calibration found."
- **Zeroing works offline.** If no IMU telemetry has arrived, zero uses a synthetic identity orientation. The "near-rest" guard on Re-zero skips the gyro-stillness check when no IMU data is present.
- **3D preview shows a synthetic tip** when no telemetry is present. While idle (`WAITING_FOR_TARGET`), the tip stays rendered at the zero position. During a `Reach` run on the sim backend, the preview animates as the synthetic state advances (see below).
- **Simulation mode for Reach**: the state machine has two backends, selected automatically:
  - *Live backend*: issues serial commands, reads real IMU/pressure telemetry. Used when serial is connected.
  - *Sim backend*: no serial I/O. Advances a toy internal state each tick — total length ramps toward the target `L` at a fixed rate; tip orientation slews toward the target direction at a fixed angular rate; a small synthetic noise term is added so the closed-loop curve on the graph looks realistic. Used when serial is not connected.
- **Selection is transparent but visible**: a small `MODE: SIMULATED` badge appears in the status line whenever the sim backend is active, so it is never unclear whether a run was physical or not.
- **Recalibrate Length**: disabled offline (it requires real pressure feedback). Tooltip explains why.
- **Logging**: the CSV logger still writes rows in sim mode, tagged with a `sim=1` column so recordings are distinguishable from real runs.
- **E-stop button**: always visible and active in both backends; in sim mode it simply halts the synthetic advance and returns to `IDLE`.

This means day-to-day UI work — layout, target picker, preview, workspace rendering, state transitions, copy — can be done against a laptop without the rig, and a demo is possible without powering the arm.

---

## 8. Graceful fallback behavior

- **Missing pressure sensor on module k**: module k excluded from the elongating PID set. Its length is pinned to `rest_length_mm`. `L_max` and `L_min` are reduced accordingly. If the reachable workspace shrinks below a target, the target is rejected with "workspace reduced due to missing sensor(s)".
- **IMU silent for > 1 s** *(live backend only)*: controller forces E-stop and shows "IMU timeout". In sim backend, no IMU timeout applies.
- **Pressure leak during bend**: detected by module pressure dropping below setpoint by more than a threshold while not commanded to deflate → halts, flags fault.
- **Yaw drift accumulating fast**: UI warns and suggests re-zero; does not auto-stop.

---

## 9. File layout

New files:

- `panels/experiment_panel.py` — the Tkinter panel (pickers, text fields, buttons, readouts, 3D preview canvas).
- `experiment_controller.py` — state machine + bend control loop, split behind a `Backend` interface with two implementations: `LiveBackend` (serial I/O + real telemetry) and `SimBackend` (synthetic state advance). The state machine is backend-agnostic. Runs in the existing `update_loop` callback or a dedicated `after()`-driven timer.
- `kinematics.py` — pure functions: forward kinematics, inverse kinematics, workspace check. Importable and unit-testable in isolation.
- `length_calibration.json` — per-module calibration data.

Modified files:

- `app.py` — add `Experiments` radio button, instantiate `ExperimentPanel`, add to `switch_mode()`. Generalize the hardcoded 3-module setup to support up to 6. Route IMU + pressure telemetry into the experiment controller when in Experiments mode.
- `panels/__init__.py` — export `ExperimentPanel`.
- `arduino_interface.py` — helper for new `TEND` / per-module `SET` commands (exact names finalized during implementation).

Arduino firmware changes (separate but tracked):

- Add per-module `SET <id> <hPa>` if not present.
- Add tendon command(s) for the 4 servos.
- Verify `IMU,` telemetry cadence is high enough for a ~20 Hz bend loop (50 ms period).

---

## 10. Testing approach

- **`kinematics.py`**: unit-tested with synthetic inputs (known arc → known tip), including degenerate θ ≈ 0 case. No hardware needed.
- **Length calibration flow**: tested by mocking the pressure telemetry stream and checking that the saved JSON has the expected structure.
- **Controller state machine**: tested by driving synthetic IMU and pressure streams through the controller and asserting state transitions.
- **End-to-end on the rig**: manual, with the operator picking a small target near zero first, then expanding outward. Bend-phase timeouts and E-stop must be verified live.

---

## 11. Open items / assumptions to verify during implementation

1. **Per-module pressure command** — verify or extend the Arduino protocol to accept `SET <module_id> <hPa>`. Current firmware appears to use a single global `SET`.
2. **Per-tendon servo command** — verify or add `TEND <servo_id> <rate>` (or an equivalent rotation/pull command). Existing code has `step_pin`/`dir_pin` for steppers per module, which does not match the described 4-servo top-plate rig. Hardware/firmware state needs reconciliation before implementation starts.
3. **Module count generalization** — existing code hardcodes 3 modules in `app.py:76-78`; generalize to 6 (and treat module count as configurable).
4. **IMU sample rate** — for a stable 20 Hz control loop, need IMU telemetry at ≥40 Hz. Verify current rate.
5. **Tendon slack/winding bounds** — `winding_max` per servo is rig-specific; initial value picked conservatively during calibration, refined empirically.
6. **Reachable `θ_max`** — initial default 60°; may need rig-specific tuning.
