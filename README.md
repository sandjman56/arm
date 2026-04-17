# Continuum Arm Controller

Tkinter GUI for a 6-module pneumatic-tendon continuum arm. Four modes: Burst,
PID, Calibration, Experiments.

## Running

Requires Python 3.10 with a working Tk (macOS system Python is typically OK;
pyenv Pythons need to be built with `--enable-framework` and Tcl/Tk installed).

```
python main.py
```

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

Tests run headless (matplotlib `Agg` backend, no Tk needed). The Tk-dependent
smoke test in `tests/test_experiment_panel_smoke.py` is opt-in — set
`RUN_UI_TESTS=1` to run it in an environment where Tk works.

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
- The panel's Tk smoke test crashes the Python process on pyenv Pythons built
  without a matching Tcl/Tk framework (macOS). That's why the smoke test is
  opt-in via `RUN_UI_TESTS=1`.

## Project structure

```
main.py                         entry point
app.py                          ArmUI orchestrator (4 modes, update loop)
arduino_interface.py            serial wrapper + experiment protocol helpers
graph.py                        matplotlib pressure graph
logger.py                       CSV telemetry logger
theme.py                        colors / fonts
widgets.py                      reusable Tk widgets

kinematics.py                   PCC forward/inverse/workspace (pure math)
orientation.py                  MPU-6500 tip orientation estimator
length_calibration.py           per-module length(psi) curve I/O + fitting
experiment_backend.py           Backend ABC + SimBackend + LiveBackend
experiment_controller.py        Experiments state machine + bend P-controller

panels/
  burst_panel.py
  calibration_panel.py
  pid_panel.py
  experiment_panel.py           main Experiments frame
  experiment_pickers.py         XY + XZ matplotlib pickers
  experiment_preview.py         3D trunk preview
  experiment_calibration.py     length-calibration modal dialog

tests/                          pytest suite
docs/superpowers/
  specs/2026-04-17-experiments-mode-design.md
  plans/2026-04-17-experiments-mode.md
```
