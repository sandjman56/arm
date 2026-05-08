"""Experiments-mode panel: Basic Elongation and Bending sub-modes."""
from __future__ import annotations
import tkinter as tk
from typing import Optional

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
        on_reach_basic,
        on_emergency_stop,
        on_reach_bending=None,
        **kwargs,
    ):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._on_start_zero = on_start_zero
        self._on_confirm_zero = on_confirm_zero
        self._on_rezero = on_rezero
        self._on_reach_basic = on_reach_basic
        self._on_reach_bending = on_reach_bending
        self._on_emergency_stop = on_emergency_stop

        # StringVars so the update loop can push values in.
        self.status_var = tk.StringVar(value="IDLE — click Zero @ Rest to begin")
        self.backend_badge_var = tk.StringVar(value="")
        self.tip_from_zero_var = tk.StringVar(value="(0.0, 0.0, 0.0)")
        self.tip_from_base_var = tk.StringVar(value="(0.0, 0.0, 0.0)")
        self.orient_var = tk.StringVar(value="pitch 0.0° roll 0.0° yaw 0.0°")
        self.yaw_drift_var = tk.StringVar(value="Yaw drift: 0.0°/min")
        self.phase_var = tk.StringVar(value="Phase: —")
        self.error_var = tk.StringVar(value="")

        # Basic sub-mode state.
        self.submode_var = tk.StringVar(value="BASIC")
        self.basic_z_var = tk.StringVar(value="20.0")
        self.basic_threshold_var = tk.StringVar(value="15.0")
        self.basic_speed_var = tk.StringVar(value="2.0")
        self.basic_elongation_readout_var = tk.StringVar(value="Elongation: 0.0 / 0.0 mm")
        self.basic_slack_readout_var = tk.StringVar(value="Slack: 0.0 deg")
        self.basic_maxpsi_readout_var = tk.StringVar(value="Max psi: 0.0")

        # Bending sub-mode state.
        self.bend_theta_var = tk.StringVar(value="20.0")
        self.bend_dir_var = tk.StringVar(value="+X")
        self.bend_ramp_var = tk.StringVar(value="5.0")
        self.bend_hold_var = tk.StringVar(value="5.0")
        self.bend_loop_var = tk.StringVar(value="OPEN")  # OPEN / CLOSED
        self.bend_pre_psi_vars = {
            mid: tk.StringVar(value="0.0") for mid in (1, 2, 3, 4, 5, 6)
        }
        self.bend_theta_readout_var = tk.StringVar(value="IMU θ: 0.0°")
        self.bend_pull_readout_var = tk.StringVar(value="Pull: 0.0°")

        self._build()

    # ----------------------------------------------------------------------
    # Layout
    # ----------------------------------------------------------------------
    def _build(self) -> None:
        # --- 1. Header + top button row (pack from TOP first) --------------
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x", side="top")
        tk.Label(self, text="EXPERIMENTS", font=FONT_BODY_BOLD,
                 fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0),
                                                   side="top")

        # Sub-mode selector: Basic Elongation vs Bending.
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
            mode_row, text="Bending", variable=self.submode_var, value="BENDING",
            command=self._on_submode_change,
            font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
            selectcolor=BG_PANEL, activebackground=BG_PANEL,
        ).pack(side="left", padx=(8, 0))

        btn_row = tk.Frame(self, bg=BG_PANEL)
        btn_row.pack(fill="x", padx=10, pady=6, side="top")
        AccentButton(btn_row, text="Zero @ Rest", accent=ACCENT_GREEN,
                     command=self._on_start_zero).pack(side="left", padx=3)
        AccentButton(btn_row, text="Confirm Zero",
                     command=self._on_confirm_zero).pack(side="left", padx=3)
        AccentButton(btn_row, text="Re-zero",
                     command=self._on_rezero).pack(side="left", padx=3)
        tk.Label(btn_row, textvariable=self.backend_badge_var,
                 font=FONT_LABEL, fg=ACCENT_ORANGE, bg=BG_PANEL).pack(side="right", padx=10)
        tk.Label(btn_row, textvariable=self.status_var,
                 font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="right", padx=10)

        # --- 2. Always-visible bottom rows (pack from BOTTOM up) -----------
        # Order below is bottom-up: last thing packed here appears ABOVE the
        # items packed before it. So the visual stack (top -> bottom) ends up:
        #   readouts
        #   E-stop
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(6, 10), side="bottom")

        readout = tk.Frame(self, bg=BG_PANEL)
        readout.pack(fill="x", padx=10, pady=4, side="bottom")
        self._phase_label: Optional[tk.Label] = None
        for var in (self.tip_from_zero_var, self.tip_from_base_var,
                    self.orient_var, self.yaw_drift_var,
                    self.phase_var, self.error_var):
            lbl = tk.Label(readout, textvariable=var, font=FONT_BODY,
                           fg=TEXT_PRIMARY, bg=BG_PANEL)
            lbl.pack(anchor="w")
            if var is self.phase_var:
                self._phase_label = lbl

        # Stepper activity indicators: one dot per module, lit when the
        # firmware is currently stepping (position changing) — i.e. actively
        # driving pressure up or down.
        stepper_row = tk.Frame(self, bg=BG_PANEL)
        stepper_row.pack(fill="x", padx=10, pady=(2, 2), side="bottom")
        tk.Label(stepper_row, text="Steppers:", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        self._stepper_dots: dict = {}
        for mid in (1, 2, 3, 4, 5, 6):
            cell = tk.Frame(stepper_row, bg=BG_PANEL)
            cell.pack(side="left", padx=(8, 0))
            tk.Label(cell, text=f"M{mid}", font=FONT_LABEL,
                     fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
            dot = tk.Canvas(cell, width=12, height=12, bg=BG_PANEL,
                            highlightthickness=0)
            dot.create_oval(2, 2, 10, 10, fill=BG_PANEL,
                            outline=TEXT_SECONDARY, tags=("dot",))
            dot.pack(side="left", padx=(3, 0))
            self._stepper_dots[mid] = dot

        # --- 3a. Basic body frame (shown when sub-mode == BASIC) ----------
        self.basic_body = tk.Frame(self, bg=BG_PANEL)

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
        tk.Label(basic_entry, text="Speed:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(basic_entry, textvariable=self.basic_speed_var, width=5,
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

        # --- 3b. Bending body frame (shown when sub-mode == BENDING) ------
        self.bend_body = tk.Frame(self, bg=BG_PANEL)
        self._build_bend_body(self.bend_body)

        # Default to BASIC: pack basic_body, hide everything else.
        self._on_submode_change()

    def _build_bend_body(self, parent: tk.Frame) -> None:
        # Row 1: target angle + direction + mode + ramp/hold
        row1 = tk.Frame(parent, bg=BG_PANEL)
        row1.pack(fill="x", pady=(6, 2))
        tk.Label(row1, text="θ target (deg):", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(row1, textvariable=self.bend_theta_var, width=6,
                 font=FONT_DATA).pack(side="left", padx=(4, 12))
        tk.Label(row1, text="Direction:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        for d in ("+X", "-X", "+Y", "-Y"):
            tk.Radiobutton(
                row1, text=d, variable=self.bend_dir_var, value=d,
                font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
                selectcolor=BG_PANEL, activebackground=BG_PANEL,
            ).pack(side="left", padx=(4, 0))

        row2 = tk.Frame(parent, bg=BG_PANEL)
        row2.pack(fill="x", pady=2)
        tk.Label(row2, text="Ramp (s):", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(row2, textvariable=self.bend_ramp_var, width=5,
                 font=FONT_DATA).pack(side="left", padx=(4, 12))
        tk.Label(row2, text="Hold (s):", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Entry(row2, textvariable=self.bend_hold_var, width=5,
                 font=FONT_DATA).pack(side="left", padx=(4, 12))
        tk.Label(row2, text="Mode:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Radiobutton(
            row2, text="Open-loop", variable=self.bend_loop_var, value="OPEN",
            font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
            selectcolor=BG_PANEL, activebackground=BG_PANEL,
        ).pack(side="left", padx=(4, 0))
        tk.Radiobutton(
            row2, text="Closed-loop", variable=self.bend_loop_var, value="CLOSED",
            font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
            selectcolor=BG_PANEL, activebackground=BG_PANEL,
        ).pack(side="left", padx=(4, 0))

        # Row 3: per-module pre-pressures
        row3 = tk.Frame(parent, bg=BG_PANEL)
        row3.pack(fill="x", pady=(6, 2))
        tk.Label(row3, text="Pre-pressure (psi) per module:", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(anchor="w")
        row3b = tk.Frame(parent, bg=BG_PANEL)
        row3b.pack(fill="x", pady=(0, 4))
        for mid in (1, 2, 3, 4, 5, 6):
            cell = tk.Frame(row3b, bg=BG_PANEL)
            cell.pack(side="left", padx=(0, 8))
            tk.Label(cell, text=f"M{mid}", font=FONT_LABEL,
                     fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
            tk.Entry(cell, textvariable=self.bend_pre_psi_vars[mid], width=5,
                     font=FONT_DATA).pack(side="left", padx=(3, 0))

        # Run button + readouts
        row4 = tk.Frame(parent, bg=BG_PANEL)
        row4.pack(fill="x", pady=(6, 2))
        AccentButton(row4, text="Run Experiment", accent=ACCENT_GREEN,
                     command=self._handle_reach_bending).pack(side="left", padx=(0, 12))
        tk.Label(row4, textvariable=self.bend_theta_readout_var, font=FONT_BODY,
                 fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="left", padx=(0, 16))
        tk.Label(row4, textvariable=self.bend_pull_readout_var, font=FONT_BODY,
                 fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="left")

    # ----------------------------------------------------------------------
    # Sub-mode / Basic handlers
    # ----------------------------------------------------------------------
    def _on_submode_change(self) -> None:
        """Swap between Basic body and Bending body."""
        sub = self.submode_var.get()
        self.basic_body.pack_forget()
        self.bend_body.pack_forget()
        if sub == "BASIC":
            self.basic_body.pack(fill="both", expand=True, padx=10, pady=4, side="top")
        elif sub == "BENDING":
            self.bend_body.pack(fill="both", expand=True, padx=10, pady=4, side="top")

    def _handle_reach_bending(self) -> None:
        if self._on_reach_bending is None:
            self.status_var.set("Bending runs not wired up")
            return
        try:
            theta = float(self.bend_theta_var.get())
            ramp_s = float(self.bend_ramp_var.get())
            hold_s = float(self.bend_hold_var.get())
            pre_psi = {
                mid: float(self.bend_pre_psi_vars[mid].get())
                for mid in (1, 2, 3, 4, 5, 6)
            }
        except ValueError:
            self.status_var.set("Invalid bending input - check numeric fields")
            return
        direction = self.bend_dir_var.get()
        closed_loop = self.bend_loop_var.get() == "CLOSED"
        self._on_reach_bending(
            theta_deg=theta,
            direction=direction,
            pre_pressures_psi=pre_psi,
            ramp_s=ramp_s,
            hold_s=hold_s,
            closed_loop=closed_loop,
        )

    def set_bending_readouts(self, theta_imu_deg: float, pull_deg: float) -> None:
        self.bend_theta_readout_var.set(f"IMU θ: {theta_imu_deg:.1f}°")
        self.bend_pull_readout_var.set(f"Pull: {pull_deg:.1f}°")

    def _handle_reach_basic(self) -> None:
        try:
            z = float(self.basic_z_var.get())
            threshold = float(self.basic_threshold_var.get())
            speed = float(self.basic_speed_var.get())
        except ValueError:
            self.status_var.set("Invalid input - Z, threshold, speed must be numbers")
            return
        if speed <= 0:
            self.status_var.set("Speed must be positive")
            return
        self._on_reach_basic(z, threshold, speed)

    def set_basic_readouts(self, elongation_mm: float, z_target_mm: float,
                           slack_deg: float, max_psi: float) -> None:
        self.basic_elongation_readout_var.set(
            f"Elongation: {elongation_mm:.1f} / {z_target_mm:.1f} mm"
        )
        self.basic_slack_readout_var.set(f"Slack: {slack_deg:.1f} deg")
        self.basic_maxpsi_readout_var.set(f"Max psi: {max_psi:.1f}")

    def current_submode(self) -> str:
        return self.submode_var.get()

    # ----------------------------------------------------------------------
    # Public setters (called by the controller/app update loop)
    # ----------------------------------------------------------------------
    def set_status(self, text: str) -> None:
        self.status_var.set(text)

    def set_backend_badge(self, text: str) -> None:
        self.backend_badge_var.set(text)

    def set_readouts(self, tip_from_zero, tip_from_base, pitch, roll, yaw,
                     yaw_drift_deg_per_min, phase, error_text):
        self.tip_from_zero_var.set(
            f"Tip (from zero): ({tip_from_zero[0]:.1f}, {tip_from_zero[1]:.1f}, {tip_from_zero[2]:.1f})"
        )
        self.tip_from_base_var.set(
            f"Tip (from base): ({tip_from_base[0]:.1f}, {tip_from_base[1]:.1f}, {tip_from_base[2]:.1f})"
        )
        import math as _m
        self.orient_var.set(
            f"pitch {_m.degrees(pitch):.1f}°  roll {_m.degrees(roll):.1f}°  yaw {_m.degrees(yaw):.1f}°"
        )
        self.yaw_drift_var.set(f"Yaw drift: {yaw_drift_deg_per_min:.2f}°/min")
        reached = (phase == "REACHED")
        # Swap the phase line to a loud "TARGET REACHED" banner on reach so
        # it's impossible to miss; revert to the plain phase readout otherwise.
        self.phase_var.set("TARGET REACHED" if reached else f"Phase: {phase}")
        self.error_var.set(error_text)
        if self._phase_label is not None:
            if reached:
                self._phase_label.configure(
                    fg=BG_PANEL, bg=ACCENT_GREEN, font=FONT_BODY_BOLD,
                )
            else:
                self._phase_label.configure(
                    fg=TEXT_PRIMARY, bg=BG_PANEL, font=FONT_BODY,
                )

    def set_stepper_active(self, module_id: int, active: bool) -> None:
        """Light/dim the activity dot for one module's stepper."""
        dot = self._stepper_dots.get(module_id)
        if dot is None:
            return
        fill = ACCENT_GREEN if active else BG_PANEL
        outline = ACCENT_GREEN if active else TEXT_SECONDARY
        dot.itemconfigure("dot", fill=fill, outline=outline)

    def set_workspace(self, r_max: float, L_rest: float, L_max: float, theta_max_rad: float) -> None:
        pass
