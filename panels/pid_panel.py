# panels/pid_panel.py -- PID pressure control panel
import tkinter as tk
from tkinter import ttk
from theme import (
    BG_PANEL, BG_INPUT, ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_RED,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_BUTTON_LARGE,
    FONT_DATA, FONT_LABEL, FONT_SLIDER,
)
from widgets import AccentButton, EmergencyStopButton


class PIDPanel(tk.Frame):
    """PID pressure control panel with goal entry, tuning sliders, and status."""

    def __init__(self, parent, on_send, on_start_pid, on_stop_pid,
                 on_emergency_stop, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._on_send = on_send
        self._on_start_pid = on_start_pid
        self._on_stop_pid = on_stop_pid
        self._on_emergency_stop = on_emergency_stop

        self.kp_var = tk.DoubleVar(value=25.0)
        self.ki_var = tk.DoubleVar(value=2.0)
        self.kd_var = tk.DoubleVar(value=8.0)
        self._sliders_ready = False

        self.pid_status_var = tk.StringVar(value="PID INACTIVE")

        self._build()
        self._sliders_ready = True

    def _build(self):
        # Header
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="PID PRESSURE CONTROL", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

        # Goal PSI input
        input_row = tk.Frame(self, bg=BG_PANEL)
        input_row.pack(pady=12, padx=10, fill="x")

        tk.Label(input_row, text="GOAL PRESSURE (PSI):", font=FONT_BODY,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")

        self.pid_entry = tk.Entry(input_row, width=10, font=FONT_DATA,
                                  bg=BG_INPUT, fg=ACCENT_CYAN,
                                  insertbackground=ACCENT_CYAN,
                                  relief="flat", bd=0)
        # Entry border frame
        entry_border = tk.Frame(input_row, bg=ACCENT_CYAN_DIM, padx=1, pady=1)
        entry_border.pack(side="left", padx=10)
        self.pid_entry = tk.Entry(entry_border, width=10, font=FONT_DATA,
                                  bg=BG_INPUT, fg=ACCENT_CYAN,
                                  insertbackground=ACCENT_CYAN,
                                  relief="flat", bd=0)
        self.pid_entry.pack()

        self._start_btn = AccentButton(input_row, text="Start PID",
                                        accent=ACCENT_CYAN, font=FONT_BUTTON_LARGE,
                                        command=self._on_start)
        self._start_btn.pack(side="left", padx=5)

        self._stop_btn = AccentButton(input_row, text="Stop PID",
                                       accent=TEXT_SECONDARY, font=FONT_BUTTON_LARGE,
                                       command=self._on_stop)
        self._stop_btn.pack(side="left", padx=5)

        # Status display
        status_frame = tk.Frame(self, bg=BG_PANEL)
        status_frame.pack(pady=5)
        self._status_label = tk.Label(status_frame, textvariable=self.pid_status_var,
                                       font=FONT_DATA, fg=TEXT_SECONDARY, bg=BG_PANEL)
        self._status_label.pack()

        # PID Tuning sliders
        tuning_header = tk.Frame(self, bg=BG_PANEL)
        tuning_header.pack(fill="x", padx=10)
        tk.Frame(tuning_header, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(tuning_header, text="PID TUNING", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        tuning = tk.Frame(self, bg=BG_PANEL)
        tuning.pack(fill="x", padx=15, pady=5)

        for label, var, from_, to_, res in [
            ("Kp", self.kp_var, 0, 100, 0.5),
            ("Ki", self.ki_var, 0, 20, 0.1),
            ("Kd", self.kd_var, 0, 50, 0.5),
        ]:
            row = tk.Frame(tuning, bg=BG_PANEL)
            row.pack(fill="x", pady=3)

            tk.Label(row, text=f"{label}:", width=3, anchor="e",
                    font=FONT_BODY_BOLD, fg=ACCENT_CYAN, bg=BG_PANEL).pack(side="left")

            slider = tk.Scale(row, variable=var, from_=from_, to=to_, resolution=res,
                             orient="horizontal", length=300,
                             bg=BG_PANEL, fg=TEXT_PRIMARY, troughcolor=BG_INPUT,
                             highlightthickness=0, bd=0,
                             activebackground=ACCENT_CYAN,
                             font=FONT_SLIDER,
                             command=lambda val, l=label, v=var: self._send_gain(l, v))
            slider.pack(side="left", fill="x", expand=True, padx=5)

            tk.Label(row, textvariable=var, width=6,
                    font=FONT_BODY, fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="left")

        # Emergency stop
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(10, 10))

    def _on_start(self):
        val = self.pid_entry.get().strip()
        self._on_start_pid(val)

    def _on_stop(self):
        self._on_stop_pid()

    def _send_gain(self, name, var):
        if not self._sliders_ready:
            return
        value = var.get()
        self._on_send(f"{name.upper()} {value:.2f}")

    def set_active(self, psi, hpa):
        """Update display to show active PID state."""
        self.pid_status_var.set(f"HOLDING {psi:.2f} PSI ({hpa:.1f} hPa)")
        self._status_label.configure(fg=ACCENT_CYAN)
        self._start_btn.set_state(False)
        self._stop_btn.set_state(True)

    def set_inactive(self, message="PID INACTIVE"):
        """Reset to inactive display."""
        self.pid_status_var.set(message)
        self._status_label.configure(fg=TEXT_SECONDARY)
        self._start_btn.set_state(True)
        self._stop_btn.set_state(False)

    def set_emergency(self):
        """Emergency stop display."""
        self.pid_status_var.set("EMERGENCY STOPPED")
        self._status_label.configure(fg=ACCENT_RED)
        self._start_btn.set_state(True)
        self._stop_btn.set_state(False)

    def get_entry_value(self):
        return self.pid_entry.get().strip()
