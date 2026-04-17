# widgets.py -- Custom styled widgets for the Continuum Arm Controller
import tkinter as tk
from theme import (
    BG_PRIMARY, BG_PANEL, BG_INPUT, BG_HOVER, BG_BUTTON,
    ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_RED, ACCENT_GREEN,
    TEXT_PRIMARY, TEXT_SECONDARY, TEXT_ACCENT,
    MODULE_COLORS,
    FONT_HEADING, FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_BUTTON_LARGE,
    FONT_DATA, FONT_DATA_LARGE, FONT_DATA_SMALL, FONT_ESTOP,
    FONT_STEPPER, FONT_STEPPER_LABEL, FONT_LABEL,
)


class AccentButton(tk.Frame):
    """Custom button with accent-colored border and hover effect."""

    def __init__(self, parent, text, command, accent=ACCENT_CYAN,
                 font=FONT_BUTTON, width=None, height=1, **kwargs):
        super().__init__(parent, bg=accent, padx=1, pady=1, **kwargs)

        self._accent = accent
        self._command = command

        self._inner = tk.Frame(self, bg=BG_BUTTON)
        self._inner.pack(fill="both", expand=True)

        lbl_kwargs = dict(
            text=text, font=font, fg=accent, bg=BG_BUTTON,
            cursor="hand2", pady=2, padx=8,
        )
        if width:
            lbl_kwargs["width"] = width
        self._label = tk.Label(self._inner, **lbl_kwargs)
        self._label.pack(fill="both", expand=True)

        for w in (self, self._inner, self._label):
            w.bind("<Button-1>", self._on_click)
            w.bind("<Enter>", self._on_enter)
            w.bind("<Leave>", self._on_leave)

    def _on_click(self, event):
        if self._command:
            self._command()

    def _on_enter(self, event):
        self._inner.configure(bg=BG_HOVER)
        self._label.configure(bg=BG_HOVER)

    def _on_leave(self, event):
        self._inner.configure(bg=BG_BUTTON)
        self._label.configure(bg=BG_BUTTON)

    def set_state(self, enabled):
        if enabled:
            self._label.configure(fg=self._accent, cursor="hand2")
            self._command_backup = None
        else:
            self._label.configure(fg=TEXT_SECONDARY, cursor="")


class EmergencyStopButton(tk.Frame):
    """Large, prominent emergency stop button with pulsing border."""

    def __init__(self, parent, command, **kwargs):
        super().__init__(parent, bg=ACCENT_RED, padx=2, pady=2, **kwargs)
        self._command = command
        self._pulse_state = False

        self._inner = tk.Frame(self, bg=BG_PRIMARY)
        self._inner.pack(fill="both", expand=True)

        self._label = tk.Label(
            self._inner, text="EMERGENCY STOP",
            font=FONT_ESTOP, fg=ACCENT_RED, bg=BG_PRIMARY,
            cursor="hand2", pady=10,
        )
        self._label.pack(fill="both", expand=True)

        for w in (self, self._inner, self._label):
            w.bind("<Button-1>", self._on_click)

        self._pulse()

    def _on_click(self, event):
        if self._command:
            self._command()

    def _pulse(self):
        self._pulse_state = not self._pulse_state
        color = "#ff6666" if self._pulse_state else ACCENT_RED
        self.configure(bg=color)
        self.after(800, self._pulse)


class GlowLabel(tk.Frame):
    """A value display with a subtle glow border effect."""

    def __init__(self, parent, text_var=None, text="", color=ACCENT_CYAN,
                 font=FONT_DATA, bg_color=BG_PANEL, **kwargs):
        super().__init__(parent, bg=color, padx=1, pady=1, **kwargs)

        self._inner = tk.Frame(self, bg=bg_color)
        self._inner.pack(fill="both", expand=True, padx=0, pady=0)

        lbl_kwargs = dict(font=font, fg=color, bg=bg_color)
        if text_var:
            lbl_kwargs["textvariable"] = text_var
        else:
            lbl_kwargs["text"] = text

        self._label = tk.Label(self._inner, **lbl_kwargs)
        self._label.pack(padx=6, pady=2)

    def set_color(self, color):
        self.configure(bg=color)
        self._label.configure(fg=color)


class StatusDot(tk.Frame):
    """Small colored status indicator dot."""

    def __init__(self, parent, color=TEXT_SECONDARY, size=10, **kwargs):
        super().__init__(parent, bg=BG_PRIMARY, **kwargs)
        self._canvas = tk.Canvas(self, width=size, height=size,
                                 bg=BG_PRIMARY, highlightthickness=0)
        self._canvas.pack()
        self._dot = self._canvas.create_oval(1, 1, size-1, size-1, fill=color, outline="")

    def set_color(self, color):
        self._canvas.itemconfigure(self._dot, fill=color)


class StepperPositionBar(tk.Frame):
    """Horizontal bar showing stepper position per module."""

    def __init__(self, parent, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._cells = {}  # module_id -> {frame, value_label, name_label}

        # Header
        header = tk.Label(self, text="STEPPER POSITIONS",
                         font=FONT_STEPPER_LABEL, fg=ACCENT_CYAN_DIM, bg=BG_PANEL)
        header.pack(side="left", padx=(10, 15), pady=4)

        self._container = tk.Frame(self, bg=BG_PANEL)
        self._container.pack(side="left", fill="x", expand=True, pady=4)

    def rebuild(self, modules):
        """Rebuild cells for current module list."""
        for w in self._container.winfo_children():
            w.destroy()
        self._cells.clear()

        for mod in modules:
            mid = mod["id"]
            color = mod["color"]

            cell = tk.Frame(self._container, bg=BG_PANEL)
            cell.pack(side="left", padx=(0, 20))

            # Top border accent line
            accent_line = tk.Frame(cell, bg=color, height=2)
            accent_line.pack(fill="x")

            name_lbl = tk.Label(cell, text=mod["name"],
                               font=FONT_STEPPER_LABEL, fg=TEXT_SECONDARY, bg=BG_PANEL)
            name_lbl.pack()

            val_lbl = tk.Label(cell, text="---",
                              font=FONT_STEPPER, fg=color, bg=BG_PANEL)
            val_lbl.pack()

            self._cells[mid] = {"frame": cell, "value_label": val_lbl, "name_label": name_lbl}

    def set_position(self, module_id, position):
        """Update the displayed position for a module."""
        if module_id in self._cells:
            self._cells[module_id]["value_label"].configure(text=f"{position}")


class StyledLabelFrame(tk.Frame):
    """Dark-themed label frame with accent border."""

    def __init__(self, parent, text="", accent=ACCENT_CYAN_DIM, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)

        # Top accent line
        tk.Frame(self, bg=accent, height=1).pack(fill="x")

        # Title
        if text:
            tk.Label(self, text=text, font=FONT_BODY_BOLD,
                    fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(4, 0))

        # Content container
        self.content = tk.Frame(self, bg=BG_PANEL)
        self.content.pack(fill="both", expand=True, padx=8, pady=5)


class PressureBox(tk.Frame):
    """Individual module pressure display box."""

    def __init__(self, parent, mod, **kwargs):
        color = mod["color"]
        super().__init__(parent, bg=color, padx=1, pady=1, **kwargs)

        inner = tk.Frame(self, bg=BG_PANEL)
        inner.pack(fill="both", expand=True)

        # Module name
        tk.Label(inner, text=mod["name"], font=FONT_LABEL,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(pady=(6, 0))

        # PSI value
        tk.Label(inner, textvariable=mod["pressure_psi"],
                font=FONT_DATA_LARGE, fg=color, bg=BG_PANEL).pack()

        tk.Label(inner, text="PSI", font=FONT_LABEL,
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack()

        # hPa value (smaller)
        tk.Label(inner, textvariable=mod["pressure_hpa"],
                font=FONT_DATA_SMALL, fg=TEXT_SECONDARY, bg=BG_PANEL).pack()

        tk.Label(inner, text="hPa", font=("Courier", 8),
                fg=TEXT_SECONDARY, bg=BG_PANEL).pack(pady=(0, 6))
