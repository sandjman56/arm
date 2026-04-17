# Continuum Arm Controller

Tkinter GUI and Arduino firmware for a **6-module pneumatic-tendon continuum
arm**. The arm consists of a stacked column of 6 "spines," each with an
internal bladder (pressure expands it along the trunk axis) and 4 tendons
routed through holes in every spine but anchored only at the tip. The tip
also carries an MPU-6500 IMU.

Four GUI modes: **Burst**, **PID**, **Calibration**, **Experiments**.

---

## Running

Python 3.10 with a working Tk (macOS system Python is typically OK; pyenv
Pythons may need to be built with `--enable-framework` and a Tcl/Tk install).

```
python main.py
```

Tests:

```
pip install -r requirements-test.txt
pytest
```

Tests run headless (matplotlib `Agg` backend, no Tk needed). The Tk smoke
test `tests/test_experiment_panel_smoke.py` is opt-in — set `RUN_UI_TESTS=1`
to run it where Tk works. Current baseline: **57 passed, 1 skipped**.

---

## Hardware overview

| Component | Model / notes | Pin / bus |
|---|---|---|
| Microcontroller | Arduino Uno | — |
| Stepper drivers | x2 (module 1, module 2) | M1: STEP=3 DIR=4; M2: STEP=8 DIR=7; EN=5 (shared) |
| Pressure sensors | Adafruit MPRLS, one per module (target 6) | I²C via PCA9548A mux (addr `0x70`); channels M1=1, M2=0, M3=3 |
| IMU | MPU-6500 (accel + gyro, **no magnetometer**) | I²C via mux channel 2, addr `0x68` (AD0→GND) |
| Tendon servos | FT6335M 360° continuous-rotation, x4 | PWM pins 9 (A, 0°), 10 (B, 90°), 11 (C, 180°), 12 (D, 270°) |

Modules 3–6 have pressure-sensor channels configurable in firmware but no
stepper drivers wired; an Uno doesn't have enough free pins for 4 more stepper
pairs. Those modules are treated as fixed-length in software.

### Tendon layout (important for control law)

Four tendons, anchored only at the tip spine, routed through aligned holes in
every spine at **0° / 90° / 180° / 270°** around the trunk circumference. Pairs:

- **Servo 1 (A, 0°) and Servo 3 (C, 180°)** — antagonists, bend tip in +X / −X
- **Servo 2 (B, 90°) and Servo 4 (D, 270°)** — antagonists, bend tip in +Y / −Y

Each servo has a continuous-rotation pulley horn. Winding one tendon pulls
the tip toward that side; its antagonist is unwound at the same rate to
maintain slack budget.

### Pressure-to-length: length calibration

Each module's `length(psi)` curve is empirical and fitted from user
measurement via the **Recalibrate Length** dialog. Stored as a quadratic
per module in `length_calibration.json`:

```json
{
  "modules": {
    "1": {"rest_length_mm": 42.0, "coeffs": [a, b, c], "max_psi": 8.0},
    ...
  },
  "missing_modules": [5]
}
```

`length_mm(psi) = a + b·psi + c·psi²`. Without calibration the UI falls back
to a default workspace (L_rest=240 mm, L_max=480 mm, θ_max=60°) and shows a
`MODE: SIMULATED` badge when no serial is connected.

---

## GUI overview

Main window is fully scrollable (vertical scrollbar on the right) so no
matter how small you make it, every section stays reachable.

### Mode: Burst Control
Per-module inflate / deflate buttons. Short burst of stepper steps per click.

### Mode: PID Control
Single-module pressure setpoint; Arduino runs a PID loop driving the stepper.
Now works per-module via the new `SET <mod_id> <hPa>` protocol.

### Mode: Calibration
Sets stepper position limits for module 1 (back wall / front wall /
persisted to `calibration.json`).

### Mode: Experiments (the new one)

Goal: pick a 3D target point and have the arm reach it via two control phases.

**Coordinate frames (all three coexist):**
- **Physics frame** — origin at base plate, Z along trunk neutral axis, X at
  tendon-0° azimuth. All kinematics and workspace math live here.
- **Display frame** — origin at "wherever you clicked Zero @ Rest." Purely a
  UX convenience; target picking is in this frame.
- **IMU reference** — orientation snapshot taken at zero; all pitch/roll/yaw
  readouts are relative to this.

**Flow:**
1. Trunk fully deflated, tendons slack.
2. **Zero @ Rest** → snapshots IMU; **Confirm Zero** locks the reference.
3. Pick a target:
   - **Click on XY (top view)** picker → red dot. This also drops a red
     dashed guide line onto the XZ picker at the locked X.
   - **Click or drag on XZ (side view)** picker → Z updates. X is snapped to
     the XY-locked value so the two views can never disagree.
   - Or type into X / Y / Z fields directly; pickers mirror.
   - 3D preview updates live from any of the above.
   - Clicking XY again invalidates Z (must re-set).
4. **Reach** runs the two-phase controller:
   - **ELONGATING** — PID-drives each module's pressure toward the setpoint
     that (per `length_calibration.json`) produces the required total arc
     length L. Timeout 10 s.
   - **BENDING** — closed-loop proportional control on IMU orientation using
     the 4 tendon servos antagonistically. Timeout 15 s.
5. Terminal states: **REACHED** (err < 3° ∧ err < 10 mm), **TIMED_OUT**, or
   aborted via Stop Reach / EMERGENCY STOP.

**Stop Reach vs EMERGENCY STOP:**
- **Stop Reach** — graceful halt. Tendons release, PID clears, state returns
  to `WAITING_FOR_TARGET`. No re-zero needed.
- **EMERGENCY STOP** — panic button. Same halt, but state returns to `IDLE`.

**Re-zero** — guarded. Only allowed when every module pressure is below
0.5 psi (so we don't capture a weird-pose zero silently).

**Auto-logging (live mode only):** hitting **Reach** while serial is
connected opens `logger/PointSelect_YYYYMMDD_HHMMSS.csv` and writes a row
every ~100 ms with:

| col | meaning |
|---|---|
| time, timestamp | elapsed-seconds and wall-clock |
| target_x/y/z | target in physics frame |
| tip_x/y/z | current tip in physics frame (from FK) |
| pitch_deg, roll_deg, yaw_deg | orientation relative to zero |
| total_length_mm | FK-derived arc length |
| phase | one of ELONGATING / BENDING / REACHED / TIMED_OUT / WAITING_FOR_TARGET / IDLE |
| psi_1..psi_6 | per-module pressure (blank if sensor missing) |
| error_text | latest pitch/pos error + timed_out flag once `last_result` is set |

Log closes and the filename appears in the status bar on REACHED,
TIMED_OUT, Stop Reach, or EMERGENCY STOP. Sim runs are **not** logged.

---

## Sim vs Live — automatic backend switch

There's no mode toggle. On **Connect**, the app swaps from `SimBackend` to
`LiveBackend` the next time you (re-)enter Experiments mode. The
`MODE: SIMULATED` badge disappears when live.

- **SimBackend** — synthetic state advance per tick. Length slews at 80 mm/s
  toward setpoint; orientation at ~20°/s. Gaussian noise layered on readouts
  (`NOISE_LEN_MM = 0.3 mm`, orient noise ≈ 0.1°) so sim demos look real.
  Intentional: demonstrates the controller receiving noisy feedback.
- **LiveBackend** — issues `SET`/`TEND`/`STOP` to Arduino, ingests `PM,` and
  `IMU,` telemetry. IMU-staleness watchdog halts an active reach if no IMU
  line arrives for > 1 s.

---

## Arduino firmware protocol

File: `arduino/edd_copy_20260401182348/edd_copy_20260401182348.ino`

Compiles clean on `arduino:avr:uno` (57% flash, 74% RAM).

### Commands (Python → Arduino)

| Command | Meaning |
|---|---|
| `SET <hPa>` | Legacy single-module PID setpoint (module 1) |
| `SET <mod_id> <hPa>` | Per-module PID setpoint (1-based) |
| `STOP` | Halt all per-module PIDs, detach all tendon servos, clear state |
| `INFLATE [mod_id]` / `DEFLATE [mod_id]` | Burst-steps the stepper (module_id 1-based, defaults to 1) |
| `BEND <int>` / `BEND_XZ <int>` / `BEND_ALL <int>` | Legacy bend commands used by Burst mode (positional 90°±60°) |
| `TEND <servo_id> <rate>` | Drive continuous-rotation tendon servo. servo_id 1..4 (A/B/C/D). rate ∈ [-1000, +1000]. Gentle bring-up default: mapped to `write(90 ± 30°)`. rate=0 detaches (releases output). |
| `RESET_POS` | Zero module 1 stepper position counter |
| `SET_MIN` / `SET_MAX` | Record current step position as back/front wall |
| `SET_LIMITS <min>,<max>` / `CLEAR_LIMITS` | Step-position limits |
| `GET_POS` | Dump module 1 step position |
| `KP <f>` / `KI <f>` / `KD <f>` | Tune PID gains (shared across modules) |

### Telemetry (Arduino → Python, ~20 Hz)

```
PM,<mod_id>,<t_ms>,<hPa>,<psi>,<step_pos>
IMU,<t_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>,<tempC>
```
Plus `READY`, `SENSOR_OK`, `SENSOR_FAIL`, `MUX_OK`, `STOPPED`,
`SETPOINT UPDATED Mi -> v`, `TEND i rate=r angle=a`, `POS_RESET n`,
`LIMITS_SET min,max`, `LIMITS_CLEARED`, `LIMIT_HIT i,pos`, `BEND ...`.

### Firmware tuning knobs

| Constant | Default | Purpose |
|---|---|---|
| `TENDON_MAX_OFFSET_DEG` | `30` | Tendon servo speed; ±deg from write(90). Raise for faster bend, risk overshoot. |
| `BURST_STEPS` | `40` | Steps per INFLATE/DEFLATE tap |
| `STEP_DELAY_US` | `2000` | Half-period between step pulses |
| `PID_INTERVAL_US` | `25000` | Per-module PID loop period (40 Hz) |
| `Kp / Ki / Kd` | `25 / 2 / 8` | PID gains (shared across modules) |

### Firmware tuning knobs (Python side)

| Constant | Default | Location | Purpose |
|---|---|---|---|
| `KP_BEND` | `4.0` | `experiment_controller.py` | Proportional gain on bend-phase orientation error |
| `OMEGA_MAX` | `1.0` | `experiment_controller.py` | Tendon rate saturation (mapped to ±1000 integer on the wire) |
| `TOL_ANGLE_RAD` | `3°` | `experiment_controller.py` | REACHED tolerance |
| `TOL_POS_MM` | `10` | `experiment_controller.py` | REACHED tolerance |
| `ELONGATION_TIMEOUT_S` | `10` | `experiment_controller.py` | |
| `BEND_TIMEOUT_S` | `15` | `experiment_controller.py` | |
| `NOISE_LEN_MM` / `NOISE_ORIENT_RAD` | `0.3` / `~0.1°` | `experiment_backend.py` | Sim-mode noise levels |
| Default workspace | `L_rest=240 / L_max=480 / θ_max=60°` | `length_calibration.py` | Used when no calibration on disk |

---

## Project structure

```
main.py                         entry point
app.py                          ArmUI orchestrator (4 modes, update loop, scrollable canvas,
                                experiment-reach auto-log)
arduino_interface.py            serial wrapper + experiment protocol helpers
                                (fmt_set_module, fmt_tendon)
graph.py                        matplotlib pressure graph
logger.py                       CSV logger (filename_prefix + header args)
theme.py                        colors / fonts
widgets.py                      reusable Tk widgets (AccentButton, StatusDot, ...)

kinematics.py                   PCC forward/inverse/workspace (pure math)
orientation.py                  MPU-6500 tip orientation estimator
length_calibration.py           per-module length(psi) curve I/O + quadratic fit
experiment_backend.py           Backend ABC + SimBackend + LiveBackend
experiment_controller.py        Experiments state machine + bend P-controller

panels/
  burst_panel.py
  calibration_panel.py
  pid_panel.py
  experiment_panel.py           Experiments frame — XY/XZ/3D, Reach, Stop Reach, E-stop
  experiment_pickers.py         XY + XZ matplotlib pickers (XY-first, drag-to-set-Z)
  experiment_preview.py         3D trunk preview
  experiment_calibration.py     length-calibration modal dialog

arduino/
  edd_copy_20260401182348/*.ino Firmware sketch

tests/                          pytest suite (57 tests)
logger/                         CSV output dir (runtime; gitignored)
docs/superpowers/
  specs/2026-04-17-experiments-mode-design.md
  plans/2026-04-17-experiments-mode.md
```

---

## Known limitations

- **Yaw drifts** (no magnetometer). UI shows running drift estimate. Re-zero
  between runs for best accuracy.
- Firmware and Python must both support the new `SET <mod_id> <hPa>` and
  `TEND <servo_id> <rate>` commands. Flash the included sketch before
  running Experiments mode on the rig.
- The Tk smoke test crashes the Python process on some pyenv Pythons built
  without a proper Tcl/Tk; opt-in via `RUN_UI_TESTS=1`.
- Only modules 1 and 2 have stepper drivers wired. Modules 3–6 accept PID
  setpoints but have no hardware to actuate until a Mega or I²C expansion is
  added.
