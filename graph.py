# graph.py -- Real-time pressure graph widget
import tkinter as tk
from theme import (
    BG_PRIMARY, BG_PANEL, BG_INPUT, ACCENT_CYAN, ACCENT_RED,
    TEXT_SECONDARY, FONT_BODY, FONT_LABEL,
)


class PressureGraph(tk.Canvas):
    """Real-time multi-module pressure graph with dark theme."""

    def __init__(self, parent, **kwargs):
        kwargs.setdefault("bg", BG_PRIMARY)
        kwargs.setdefault("height", 180)
        kwargs.setdefault("highlightthickness", 0)
        super().__init__(parent, **kwargs)

    def draw(self, modules, pid_active=False, pid_setpoint_psi=0.0):
        """Redraw the graph with current module data."""
        self.delete("all")

        w = self.winfo_width()
        h = self.winfo_height()
        if w < 10 or h < 10:
            return

        pad_l, pad_r, pad_t, pad_b = 55, 10, 10, 25
        plot_w = w - pad_l - pad_r
        plot_h = h - pad_t - pad_b

        # Collect all values for Y-axis scaling
        all_values = []
        for mod in modules:
            all_values.extend(mod["psi_history"])

        if len(all_values) < 2:
            self.create_text(w // 2, h // 2, text="WAITING FOR DATA...",
                             fill=TEXT_SECONDARY, font=FONT_BODY)
            return

        y_min = min(all_values)
        y_max = max(all_values)
        if y_max - y_min < 0.1:
            y_mid = (y_max + y_min) / 2
            y_min = y_mid - 0.5
            y_max = y_mid + 0.5

        margin = (y_max - y_min) * 0.1
        y_min -= margin
        y_max += margin

        # Grid lines
        num_ticks = 5
        for i in range(num_ticks + 1):
            frac = i / num_ticks
            y = pad_t + plot_h * (1 - frac)
            val = y_min + (y_max - y_min) * frac
            self.create_line(pad_l, y, w - pad_r, y, fill="#1a2332", width=1)
            self.create_text(pad_l - 5, y, text=f"{val:.2f}", anchor="e",
                             fill=TEXT_SECONDARY, font=("Courier", 8))

        # Plot border
        self.create_rectangle(pad_l, pad_t, w - pad_r, pad_t + plot_h,
                              outline="#1a2332", width=1)

        # Plot line per module
        for mi, mod in enumerate(modules):
            values = list(mod["psi_history"])
            if len(values) < 2:
                continue

            n = len(values)
            points = []
            for i, v in enumerate(values):
                x = pad_l + (i / (n - 1)) * plot_w
                y = pad_t + plot_h * (1 - (v - y_min) / (y_max - y_min))
                points.append(x)
                points.append(y)

            self.create_line(points, fill=mod["color"], width=2, smooth=True)

            # Latest value label
            label_y = pad_t + 5 + mi * 18
            self.create_text(w - pad_r - 5, label_y, anchor="ne",
                             text=f"{values[-1]:.3f} PSI", fill=mod["color"],
                             font=("Courier", 10, "bold"))

        # Setpoint line (PID active)
        if pid_active and pid_setpoint_psi > 0:
            sp = pid_setpoint_psi
            if y_min <= sp <= y_max:
                sp_y = pad_t + plot_h * (1 - (sp - y_min) / (y_max - y_min))
                self.create_line(pad_l, sp_y, w - pad_r, sp_y,
                                 fill=ACCENT_RED, width=1, dash=(6, 3))
                self.create_text(pad_l + 5, sp_y - 10, anchor="w",
                                 text=f"GOAL: {sp:.2f} PSI", fill=ACCENT_RED,
                                 font=("Courier", 9, "bold"))

        # X-axis label
        self.create_text(w // 2, h - 3, text="TIME \u2192", fill=TEXT_SECONDARY,
                         font=("Courier", 8))
