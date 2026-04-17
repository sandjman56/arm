"""Modal length-calibration dialog.

Walks the operator through each available module: command a pressure,
prompt for a measured length, collect samples, fit a quadratic, save.
"""
from __future__ import annotations
import tkinter as tk
from tkinter import simpledialog, messagebox
from typing import Callable, Dict, List, Optional

from length_calibration import LengthCalibration, fit_module_curve


def run_length_calibration_dialog(
    root: Optional[tk.Tk],
    available_modules: List[int],
    command_pressure: Callable[[int, float], None],
    wait_for_steady: Callable[[], None],
    max_psi: float = 8.0,
    num_points: int = 5,
) -> Optional[LengthCalibration]:
    """Blocking walkthrough. Returns a `LengthCalibration` on success or None on cancel."""
    if not available_modules:
        messagebox.showwarning("Calibrate Length",
                               "No modules with live pressure telemetry. Connect the rig first.")
        return None

    modules: Dict[int, dict] = {}
    psi_setpoints = [max_psi * i / (num_points - 1) for i in range(num_points)]
    for mid in available_modules:
        samples: List[tuple] = []
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
