"""Experiments-mode panel: target picking + two-phase reach controller.

Layout is arranged so that the target-entry row, readouts, and E-stop
button ALWAYS remain visible regardless of window height. The 3D preview
canvas is the only element that shrinks when the window does. This is
achieved by packing the always-visible rows from the bottom first, then
packing the canvas row with expand=True in the middle.

Picker interaction (spec §7 revision):
  1. Click on XY picker                -> locks (X, Y). Red dot appears.
                                          XZ picker gets a red dashed guide
                                          line at the locked X with a red
                                          draggable dot at a default Z.
  2. Click or drag on XZ picker        -> Z snaps to the click/drag Y value
                                          (X is snapped to locked X).
  3. Any of (XY click, XZ drag, typed
     text entry) refreshes the 3D
     preview's target marker live.
  4. Clicking XY again invalidates Z
     (the red dot on XZ jumps back to
     the default Z for the new X).
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
        on_reach,
        on_emergency_stop,
        **kwargs,
    ):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._on_start_zero = on_start_zero
        self._on_confirm_zero = on_confirm_zero
        self._on_rezero = on_rezero
        self._on_reach = on_reach
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

        self.target_x = tk.StringVar(value="")
        self.target_y = tk.StringVar(value="")
        self.target_z = tk.StringVar(value="")

        # Current workspace for coordinate-frame conversions when updating the
        # 3D preview (physics-frame). Kept in sync by set_workspace().
        self._L_rest = 240.0
        self._L_max = 480.0

        # Guard flag so programmatic updates to target_* don't re-trigger the
        # text-change trace (which itself updates the pickers and preview).
        self._suppress_trace = False

        self._build()

        # Text-field traces: user edits XYZ fields -> pickers and preview update.
        self.target_x.trace_add("write", lambda *_: self._on_text_change())
        self.target_y.trace_add("write", lambda *_: self._on_text_change())
        self.target_z.trace_add("write", lambda *_: self._on_text_change())

    # ----------------------------------------------------------------------
    # Layout
    # ----------------------------------------------------------------------
    def _build(self) -> None:
        # --- 1. Header + top button row (pack from TOP first) --------------
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x", side="top")
        tk.Label(self, text="EXPERIMENTS", font=FONT_BODY_BOLD,
                 fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0),
                                                   side="top")

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
        #   target entry
        #   readouts
        #   E-stop
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(6, 10), side="bottom")

        readout = tk.Frame(self, bg=BG_PANEL)
        readout.pack(fill="x", padx=10, pady=4, side="bottom")
        for var in (self.tip_from_zero_var, self.tip_from_base_var,
                    self.orient_var, self.yaw_drift_var,
                    self.phase_var, self.error_var):
            tk.Label(readout, textvariable=var, font=FONT_BODY,
                     fg=TEXT_PRIMARY, bg=BG_PANEL).pack(anchor="w")

        tgt_row = tk.Frame(self, bg=BG_PANEL)
        tgt_row.pack(fill="x", padx=10, pady=(4, 4), side="bottom")
        tk.Label(tgt_row, text="Target:", font=FONT_BODY,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        for label, var in (("X", self.target_x), ("Y", self.target_y), ("Z", self.target_z)):
            tk.Label(tgt_row, text=label, font=FONT_BODY,
                     fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(10, 2))
            tk.Entry(tgt_row, textvariable=var, width=8,
                     font=FONT_DATA).pack(side="left")
        AccentButton(tgt_row, text="Reach", accent=ACCENT_GREEN,
                     command=self._handle_reach).pack(side="left", padx=15)

        # --- 3. Canvas row fills whatever is left (shrinks first) ---------
        from panels.experiment_pickers import XYPicker, XZPicker
        from panels.experiment_preview import TrunkPreview3D

        self.canvas_row = tk.Frame(self, bg=BG_PANEL)
        self.canvas_row.pack(fill="both", expand=True, padx=10, pady=4, side="top")
        self.xy_picker = XYPicker(self.canvas_row, r_max=400.0)
        self.xy_picker.pack(side="left", padx=4, fill="both", expand=True)
        self.xy_picker.bind_pick(self._on_xy_pick)
        self.xz_picker = XZPicker(self.canvas_row, L_rest=240.0, L_max=480.0)
        self.xz_picker.pack(side="left", padx=4, fill="both", expand=True)
        self.xz_picker.bind_pick(self._on_xz_pick)
        self.preview3d = TrunkPreview3D(self.canvas_row, L_max=480.0)
        self.preview3d.pack(side="left", padx=4, fill="both", expand=True)

    # ----------------------------------------------------------------------
    # Picker callbacks
    # ----------------------------------------------------------------------
    def _on_xy_pick(self, x: float, y: float) -> None:
        """User clicked or dragged on XY. Locks (X, Y), invalidates Z."""
        self._suppress_trace = True
        try:
            self.target_x.set(f"{x:.1f}")
            self.target_y.set(f"{y:.1f}")
            self.target_z.set("")
        finally:
            self._suppress_trace = False

        self.xy_picker.set_target(x, y)
        self.xz_picker.set_locked_x(x)
        # Z is not set yet -> clear 3D preview target.
        self.preview3d.set_target(None)

    def _on_xz_pick(self, x: float, z: float) -> None:
        """User clicked or dragged on XZ (only fires when XZ is gated open)."""
        self._suppress_trace = True
        try:
            self.target_z.set(f"{z:.1f}")
        finally:
            self._suppress_trace = False
        # X was already locked; nothing to update for xy_picker.
        # Push the completed target into the 3D preview in physics frame.
        self._update_preview_target_from_fields()

    def _on_text_change(self) -> None:
        """User edited one of the XYZ entry fields."""
        if self._suppress_trace:
            return
        x = _parse_float(self.target_x.get())
        y = _parse_float(self.target_y.get())
        z = _parse_float(self.target_z.get())
        # Mirror into pickers where possible.
        if x is not None and y is not None:
            self.xy_picker.set_target(x, y)
            self.xz_picker.set_locked_x(x)
        if x is not None and z is not None:
            self.xz_picker.set_target(x, z)
        # 3D preview.
        self._update_preview_target_from_fields()

    def _update_preview_target_from_fields(self) -> None:
        x = _parse_float(self.target_x.get())
        y = _parse_float(self.target_y.get())
        z = _parse_float(self.target_z.get())
        if x is None or y is None or z is None:
            self.preview3d.set_target(None)
            return
        # 3D preview is rendered in physics frame — convert from display frame
        # (resting-tip origin) by adding L_rest to z.
        self.preview3d.set_target((x, y, z + self._L_rest))

    # ----------------------------------------------------------------------
    # Reach
    # ----------------------------------------------------------------------
    def _handle_reach(self) -> None:
        try:
            x = float(self.target_x.get())
            y = float(self.target_y.get())
            z = float(self.target_z.get())
        except ValueError:
            self.status_var.set("Invalid target — X/Y/Z must be numbers")
            return
        self._on_reach((x, y, z))

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
        self.phase_var.set(f"Phase: {phase}")
        self.error_var.set(error_text)

    def set_tip_position(self, tip_display: Tuple[float, float, float],
                         arc_points_physics: list) -> None:
        """Sync live-telemetry tip into the three canvases.

        Args:
            tip_display:        tip in DISPLAY frame (picker rendering)
            arc_points_physics: arc samples in PHYSICS frame (preview rendering)
        """
        x, y, z_disp = tip_display
        self.xy_picker.set_tip(x, y)
        self.xz_picker.set_tip(x, z_disp)
        # 3D preview is physics-frame: reconstruct tip_physics from arc points
        # (the caller already built them consistently).
        tip_physics = arc_points_physics[-1] if arc_points_physics else (x, y, z_disp + self._L_rest)
        self.preview3d.set_tip_and_arc(tip_physics, arc_points_physics)

    def set_target_marker(self, target_physics: Optional[Tuple[float, float, float]]) -> None:
        """Draw a target marker in all three canvases from a PHYSICS-frame target.

        Called from the app when `Reach` is pressed with a validated target.
        Picker markers are drawn in display frame (subtract L_rest from z).
        """
        if target_physics is None:
            self.xy_picker.set_target(None, None)
            self.xz_picker.set_target(None, None)
            self.preview3d.set_target(None)
            return
        tx, ty, tz = target_physics
        tz_disp = tz - self._L_rest
        self.xy_picker.set_target(tx, ty)
        self.xz_picker.set_target(tx, tz_disp)
        self.preview3d.set_target(target_physics)

    def set_workspace(self, r_max: float, L_rest: float, L_max: float, theta_max_rad: float) -> None:
        self._L_rest = L_rest
        self._L_max = L_max
        self.xy_picker.set_r_max(r_max)
        self.xz_picker.set_workspace(L_rest, L_max, theta_max_rad)
        self.preview3d._L_max = L_max
        self.preview3d._theta_max = theta_max_rad
        self.preview3d._redraw()


def _parse_float(s: str) -> Optional[float]:
    try:
        return float(s)
    except (TypeError, ValueError):
        return None
