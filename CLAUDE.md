# CLAUDE.md — context for AI coding agents working in this repo

Start here if you are an AI agent (Claude Code, Cursor, Copilot, etc.) that
has been asked to make changes to this project. This file gives you the
project-specific conventions and context that aren't obvious from the code
alone. For the user-facing overview, see `README.md`.

## What this project is

A control system for a 6-module pneumatic-tendon **continuum robot arm**.
Two halves:

1. **Python/Tkinter GUI** (`app.py`, `panels/`, pure-logic modules)
2. **Arduino firmware** (`arduino/edd_copy_20260401182348/*.ino`) running on
   an Uno over USB serial at 115200 baud.

The GUI has four modes: **Burst**, **PID**, **Calibration**, **Experiments**.
The Experiments mode is the most complex — it is the closed-loop point-reach
feature that drives most of the design.

## Hardware constraints that shape the code

- **Arduino Uno**, not a Mega. Pin budget is tight. 4 tendon servos live on
  pins 9–12, stepper drivers for only 2 modules fit (M1=3,4; M2=8,7).
  Modules 3–6 are "software-present, hardware-absent" — the code supports
  them but commands directed at them don't move anything.
- **MPU-6500 IMU — accel + gyro only. No magnetometer.** Yaw drifts.
  Pitch/roll are gravity-anchored and stable. Design must tolerate yaw drift
  (Re-zero button is the escape hatch).
- **Tendon servos are FT6335M 360° continuous-rotation.** Controlled via
  standard `Servo.h` PWM. `write(90)` = stop, `write(90 ± 30°)` = slow CR
  turn. Full-speed is `write(0)` / `write(180)` (avoid for now).
- **I²C bus is multiplexed** through a PCA9548A at `0x70`. The IMU and each
  pressure sensor live behind the mux. Always `tcaselect(channel)` before
  reading a sensor.
- **Pressure sensors** (MPRLS, one per module target) are partial: current
  rig is wired for 3. Code must fall back gracefully — a module with no
  recent `PM,` telemetry is treated as fixed-length by the Python side.

## Key architectural decisions

### Three coordinate frames

Physics, display, and IMU-reference frames coexist. See `README.md` for the
full explanation. The rule: **the controller always runs in the physics
frame.** The display-frame offset (zero-at-rest) is applied only at UI
boundaries. Do not bake the zero offset into kinematics math.

### Backend abstraction (Sim vs Live)

`experiment_backend.py` defines a `Backend` ABC with two implementations:

- `SimBackend` — no serial I/O. Internal state slews toward setpoints each
  tick. Gaussian noise on readouts. Lets UI work without a rig.
- `LiveBackend` — wraps `ArduinoInterface`, ingests `PM,` and `IMU,`
  telemetry lines. Issues `SET`, `TEND`, `STOP` commands.

The controller is backend-agnostic. `app.py::_refresh_experiment_backend()`
swaps between them based on serial connection state on mode entry.

### State machine

`ExperimentController` (`experiment_controller.py`):

```
IDLE → ZEROING → WAITING_FOR_TARGET → ELONGATING → BENDING → REACHED
                                          │              │
                                          └─> TIMED_OUT <┘
```

Plus E-stop from anywhere → IDLE, Stop Reach from ELONGATING/BENDING →
WAITING_FOR_TARGET (graceful, no re-zero needed).

### Auto-logging

Hitting **Reach** in live mode auto-opens `logger/PointSelect_*.csv`. One
row per `update_loop` tick (~10 Hz). Closed on REACHED/TIMED_OUT/stop/
E-stop. Sim runs are not logged (explicit user decision).

## Conventions for making changes

### Tests first

- Most code is covered by `pytest`. 57 tests, 1 opt-in Tk smoke skipped.
- **When you change logic, add/update a test before changing code** (TDD).
- Run `pytest` after any change. It's fast (<1s).
- Opt-in Tk smoke test: `RUN_UI_TESTS=1 pytest`.

### Firmware changes

- Always compile-check with `arduino-cli compile --fqbn arduino:avr:uno
  arduino/edd_copy_20260401182348/` before committing.
- Keep the sketch under ~28 KB / 1800 B RAM (current: 18504 B / 1529 B).
- If you add a new serial command, mirror it in both:
  - `arduino_interface.py` (format helper)
  - `experiment_backend.py::LiveBackend` (sender)

### UI layout rules

- The main window is scrollable via a `tk.Canvas` wrapper. All children
  should be packed into `self.content_frame`, NOT `self.root`.
- In `ExperimentPanel`, always-visible rows (target entry, readouts, E-stop)
  are packed `side="bottom"` so they stay in frame when the window shrinks.
  The canvas row (XY/XZ/3D) is packed with `expand=True` and shrinks first.

### Git / branch hygiene

- Work on short-lived feature branches cut from `main`.
- Merge back with `--no-ff` and a descriptive merge message.
- Delete the branch after merge (`git branch -d`).
- Commit messages: conventional-ish — `feat:`, `fix:`, `refactor:`,
  `firmware:`, `ui:`, `test:`, `docs:`, `chore:`.

### Code style

- Don't rewrite whole files unless asked.
- Don't introduce new libraries casually. Current deps: matplotlib, numpy
  (transitive), pyserial, pytest, pytest-mock.
- Preserve backward compatibility on the Arduino protocol (e.g., `SET
  <hPa>` without a module id must still work).
- Pure logic (`kinematics.py`, `orientation.py`, `length_calibration.py`)
  has no Tk or serial dependencies — keep it that way so it stays
  unit-testable.

## Session notes / decisions that didn't make it into code comments

- **Sim noise is kept on purpose.** When readouts jiggle ±0.1° / ±0.3 mm
  in idle sim mode, that's intentional — shows the controller receiving
  realistic feedback. User explicitly asked to leave it.
- **Target in readouts vs. tip in readouts are different things.** The
  readout block shows *current state*, not the picked target. Users have
  asked about this; the answer is: the target is in the Target entry row,
  the tip is in the readout block, they're supposed to differ until REACHED.
- **Picker interaction is XY-first.** XZ is gated until an XY click has
  happened. Clicking XY again resets Z. This is to prevent inconsistent
  X values between the two views.
- **Tendon "gentle" rate was a deliberate choice.** `TENDON_MAX_OFFSET_DEG
  = 30` on the firmware is a safety default during bring-up. Raise with
  care once the loop is characterized on the rig.
- **`_REACH_LOG_HEADER` is defined at app.py module scope** so it can be
  unit-tested / referenced externally if needed. Don't move it into
  `ArmUI`.

## When in doubt

- Check `docs/superpowers/specs/2026-04-17-experiments-mode-design.md` for
  the full design.
- Check `docs/superpowers/plans/2026-04-17-experiments-mode.md` for the
  step-by-step implementation plan that produced the current code.
- Match the `brainstorming → writing-plans → executing-plans` workflow
  (superpowers skills) for new features.
- Ask the user before destructive actions on their rig (flashing unverified
  firmware, force-pushing, etc.).
