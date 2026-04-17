"""Experiments-mode panel: target picking + two-phase reach controller.

This module owns the outer `ExperimentPanel` frame. Picker widgets and the
3D preview live in sibling modules so each stays focused.
"""
from __future__ import annotations
import tkinter as tk
from typing import Optional, Tuple

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

        # Canvas row.
        from panels.experiment_pickers import XYPicker, XZPicker
        self.canvas_row = tk.Frame(self, bg=BG_PANEL)
        self.canvas_row.pack(fill="x", padx=10, pady=4)
        self.xy_picker = XYPicker(self.canvas_row, r_max=400.0)
        self.xy_picker.pack(side="left", padx=4)
        self.xy_picker.bind_pick(self._on_xy_click)
        self.xz_picker = XZPicker(self.canvas_row, L_rest=240.0, L_max=480.0)
        self.xz_picker.pack(side="left", padx=4)
        self.xz_picker.bind_pick(self._on_xz_click)
        from panels.experiment_preview import TrunkPreview3D
        self.preview3d = TrunkPreview3D(self.canvas_row, L_max=480.0)
        self.preview3d.pack(side="left", padx=4, fill="both", expand=True)

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

    def _on_xy_click(self, x: float, y: float) -> None:
        self.target_x.set(f"{x:.1f}")
        self.target_y.set(f"{y:.1f}")
        # Lock X on the XZ picker so clicks there stay consistent.
        self.xz_picker.set_locked_x(x)

    def _on_xz_click(self, x: float, z: float) -> None:
        self.target_x.set(f"{x:.1f}")
        self.target_z.set(f"{z:.1f}")

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
