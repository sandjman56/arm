# panels/calibration_panel.py -- Calibration mode panel
import tkinter as tk
from theme import (
    BG_PANEL, BG_INPUT, ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_ORANGE, ACCENT_GREEN, ACCENT_RED,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_BUTTON_LARGE, FONT_DATA, FONT_LABEL,
)
from widgets import AccentButton, EmergencyStopButton


class CalibrationPanel(tk.Frame):
    """Calibration panel for setting position limits."""

    def __init__(self, parent, on_send, on_set_back, on_set_front,
                 on_save, on_clear, on_emergency_stop, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._on_send = on_send
        self._on_set_back = on_set_back
        self._on_set_front = on_set_front
        self._on_save = on_save
        self._on_clear = on_clear
        self._on_emergency_stop = on_emergency_stop

        self.cal_pos_var = tk.StringVar(value="0")
        self.cal_back_var = tk.StringVar(value="NOT SET")
        self.cal_front_var = tk.StringVar(value="NOT SET")
        self.cal_range_var = tk.StringVar(value="---")

        self._build()

    def _build(self):
        # Header
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="CALIBRATION", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

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
                                 self._on_set_back, self.cal_back_var, "back")

        # Step 2: Front wall
        self._build_wall_section("STEP 2: SET FRONT WALL",
                                 self._on_set_front, self.cal_front_var, "front")

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
                    command=self._on_clear).pack(side="left", padx=10)

        # Emergency stop
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(10, 10))

    def _build_wall_section(self, title, on_set, status_var, wall_type):
        """Build a back/front wall calibration section."""
        section = tk.Frame(self, bg=BG_PANEL)
        section.pack(fill="x", padx=15, pady=4)

        # Section header
        tk.Frame(section, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(section, text=title, font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        row = tk.Frame(section, bg=BG_PANEL)
        row.pack(fill="x", pady=5)

        tk.Label(row, text="Adjust:", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=5)

        # For calibration, inflate/deflate commands are swapped visually
        AccentButton(row, text="Inflate",
                    command=lambda: self._on_send("DEFLATE")).pack(side="left", padx=3)
        AccentButton(row, text="Deflate",
                    command=lambda: self._on_send("INFLATE")).pack(side="left", padx=3)

        AccentButton(row, text=f"Set {title.split(':')[1].strip()}",
                    accent=ACCENT_ORANGE, font=FONT_BUTTON_LARGE,
                    command=on_set).pack(side="left", padx=15)

        tk.Label(row, textvariable=status_var, font=FONT_BODY,
                fg=ACCENT_ORANGE, bg=BG_PANEL).pack(side="left", padx=10)

    def set_position(self, steps):
        self.cal_pos_var.set(f"{steps}")

    def set_limits_display(self, back, front):
        """Restore saved calibration display."""
        self.cal_back_var.set(f"Back wall: {back} steps")
        self.cal_front_var.set(f"Front wall: {front} steps")
        rng = front - back
        self.cal_range_var.set(f"{rng} steps")
        self._save_btn.set_state(True)

    def clear_display(self):
        self.cal_back_var.set("NOT SET")
        self.cal_front_var.set("NOT SET")
        self.cal_range_var.set("---")
        self._save_btn.set_state(False)

    def enable_save(self, enabled):
        self._save_btn.set_state(enabled)
