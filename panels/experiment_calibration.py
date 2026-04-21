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
    rest_psi: Optional[Dict[int, float]] = None,
    max_psi: float = 8.0,
    num_points: int = 5,
) -> Optional[LengthCalibration]:
    """Blocking walkthrough. Returns a `LengthCalibration` on success or None on cancel.

    `rest_psi` is a per-module {mid: psi} map of the at-rest pressure read
    from live sensors. The sweep commands [rest, rest + max_psi/(n-1), …,
    rest + max_psi] (absolute psi) and vents each module back to its rest
    value rather than 0 psi — commanding 0 psi tells the firmware to pull
    toward vacuum, far below atmospheric.
    """
    if not available_modules:
        messagebox.showwarning("Calibrate Length",
                               "No modules with live pressure telemetry. Connect the rig first.")
        return None

    if rest_psi is None:
        rest_psi = {}

    modules: Dict[int, dict] = {}
    for mid in available_modules:
        rest = float(rest_psi.get(mid, 14.0))
        psi_setpoints = [rest + max_psi * i / (num_points - 1) for i in range(num_points)]
        samples: List[tuple] = []
        for psi in psi_setpoints:
            command_pressure(mid, psi)
            wait_for_steady()
            answer = simpledialog.askfloat(
                f"Module {mid}",
                f"Commanded {psi:.2f} psi (absolute). Measure module length in mm and enter:",
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
            "max_psi": rest + max_psi,
        }
        # Vent the module back to rest for the next iteration — NOT 0 psi.
        command_pressure(mid, rest)

    # Modules not in available_modules are treated as missing.
    all_possible = set(range(1, 7))
    missing = sorted(all_possible - set(available_modules))
    cal = LengthCalibration(modules=modules, missing_modules=missing, is_default=False)
    return cal
