# panels/calibration_panel.py -- Calibration mode panel
import tkinter as tk
from theme import (
    BG_PANEL, BG_INPUT, ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_ORANGE, ACCENT_GREEN, ACCENT_RED,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_BUTTON_LARGE, FONT_DATA, FONT_LABEL,
)
from widgets import AccentButton, EmergencyStopButton


class CalibrationPanel(tk.Frame):
    """Calibration panel for setting per-module stepper position limits."""

    def __init__(self, parent, modules, on_send, on_set_back, on_set_front,
                 on_save, on_clear, on_module_change, on_emergency_stop, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        # modules is a list of module dicts (id, name, color, ...) for wired
        # modules only. The panel renders one radio per module and routes all
        # commands/state updates via the currently-selected module.
        self._modules = modules
        self._on_send = on_send
        self._on_set_back = on_set_back
        self._on_set_front = on_set_front
        self._on_save = on_save
        self._on_clear = on_clear
        self._on_module_change = on_module_change
        self._on_emergency_stop = on_emergency_stop

        self.cal_pos_var = tk.StringVar(value="0")
        self.cal_back_var = tk.StringVar(value="NOT SET")
        self.cal_front_var = tk.StringVar(value="NOT SET")
        self.cal_range_var = tk.StringVar(value="---")

        default_id = modules[0]["id"] if modules else 1
        self.selected_module_id = tk.IntVar(value=default_id)

        self._build()

    def _build(self):
        # Header
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="CALIBRATION", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

        # Module selector
        sel_frame = tk.Frame(self, bg=BG_PANEL)
        sel_frame.pack(fill="x", padx=15, pady=(6, 0))
        tk.Label(sel_frame, text="MODULE:", font=FONT_LABEL,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        for mod in self._modules:
            tk.Radiobutton(
                sel_frame, text=mod["name"], variable=self.selected_module_id,
                value=mod["id"], command=self._handle_module_change,
                font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL,
                selectcolor=BG_PANEL, activebackground=BG_PANEL,
                activeforeground=mod["color"],
            ).pack(side="left", padx=(10, 0))

        # Current position display
        pos_frame = tk.Frame(self, bg=BG_PANEL)
        pos_frame.pack(fill="x", padx=15, pady=8)

        tk.Label(pos_frame, text="CURRENT POSITION:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")

        pos_border = tk.Frame(pos_frame, bg=ACCENT_GREEN, padx=1, pady=1)
        pos_border.pack(side="left", padx=10)
        tk.Label(pos_border, textvariable=self.cal_pos_var,
                font=FONT_DATA, fg=ACCENT_GREEN, bg=BG_PANEL,
                padx=10, pady=2).pack()

        tk.Label(pos_frame, text="steps", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=5)

        # Step 1: Back wall
        self._build_wall_section("STEP 1: SET BACK WALL",
                                 self._handle_set_back, self.cal_back_var, "back")

        # Step 2: Front wall
        self._build_wall_section("STEP 2: SET FRONT WALL",
                                 self._handle_set_front, self.cal_front_var, "front")

        # Results & save
        result_frame = tk.Frame(self, bg=BG_PANEL)
        result_frame.pack(fill="x", padx=15, pady=8)

        tk.Label(result_frame, text="RANGE:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
        tk.Label(result_frame, textvariable=self.cal_range_var,
                font=FONT_BODY_BOLD, fg=ACCENT_CYAN, bg=BG_PANEL).pack(
                    side="left", padx=10)

        self._save_btn = AccentButton(result_frame, text="Save Calibration",
                                       accent=ACCENT_GREEN, font=FONT_BUTTON_LARGE,
                                       command=self._on_save)
        self._save_btn.pack(side="left", padx=10)

        AccentButton(result_frame, text="Clear Limits",
                    accent=ACCENT_RED, font=FONT_BUTTON,
                    command=self._handle_clear).pack(side="left", padx=10)

        # Emergency stop
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(10, 10))

    def _build_wall_section(self, title, on_set, status_var, wall_type):
        """Build a back/front wall calibration section."""
        section = tk.Frame(self, bg=BG_PANEL)
        section.pack(fill="x", padx=15, pady=4)

        tk.Frame(section, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(section, text=title, font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        row = tk.Frame(section, bg=BG_PANEL)
        row.pack(fill="x", pady=5)

        tk.Label(row, text="Adjust:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=5)

        # Inflate/Deflate target the currently-selected module. The visual
        # "Inflate" vs "Deflate" labels are swapped intentionally against the
        # underlying serial command so the UI matches the physical rig: on
        # this rig, deflating physically extends, so "Inflate" -> DEFLATE.
        AccentButton(row, text="Inflate",
                    command=lambda: self._send_module("DEFLATE")).pack(side="left", padx=3)
        AccentButton(row, text="Deflate",
                    command=lambda: self._send_module("INFLATE")).pack(side="left", padx=3)

        AccentButton(row, text=f"Set {title.split(':')[1].strip()}",
                    accent=ACCENT_ORANGE, font=FONT_BUTTON_LARGE,
                    command=on_set).pack(side="left", padx=15)

        tk.Label(row, textvariable=status_var, font=FONT_BODY,
                fg=ACCENT_ORANGE, bg=BG_PANEL).pack(side="left", padx=10)

    def _send_module(self, verb):
        self._on_send(f"{verb},{self.selected_module_id.get()}")

    def _handle_set_back(self):
        self._on_set_back(self.selected_module_id.get())

    def _handle_set_front(self):
        self._on_set_front(self.selected_module_id.get())

    def _handle_clear(self):
        self._on_clear(self.selected_module_id.get())

    def _handle_module_change(self):
        self._on_module_change(self.selected_module_id.get())

    def set_position(self, module_id, steps):
        """Update position display if module_id is the currently selected one."""
        if module_id == self.selected_module_id.get():
            self.cal_pos_var.set(f"{steps}")

    def set_limits_display(self, module_id, back, front):
        """Restore saved calibration display for a specific module."""
        if module_id != self.selected_module_id.get():
            return
        self.cal_back_var.set(f"Back wall: {back} steps")
        self.cal_front_var.set(f"Front wall: {front} steps")
        rng = front - back
        self.cal_range_var.set(f"{rng} steps")
        self._save_btn.set_state(True)

    def clear_display(self, module_id=None):
        """Reset back/front/range text. If module_id is given, only clear if
        that module is the currently selected one."""
        if module_id is not None and module_id != self.selected_module_id.get():
            return
        self.cal_back_var.set("NOT SET")
        self.cal_front_var.set("NOT SET")
        self.cal_range_var.set("---")
        self._save_btn.set_state(False)

    def enable_save(self, enabled):
        self._save_btn.set_state(enabled)
