# panels/burst_panel.py -- Burst control mode panel
import tkinter as tk
from theme import (
    BG_PANEL, BG_PRIMARY, ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_GREEN,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_BUTTON, FONT_LABEL,
)
from widgets import AccentButton, EmergencyStopButton, StyledLabelFrame


class BurstPanel(tk.Frame):
    """Burst control panel with per-module inflate/deflate and bending controls."""

    def __init__(self, parent, modules, on_send,
                 on_emergency_stop, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._modules = modules
        self._on_send = on_send
        self._on_emergency_stop = on_emergency_stop

        # One IntVar per servo (A/B/C/D -> pins 9/10/11/24). Sliders are
        # UI-only; angles are not sent to the firmware until Save is pressed.
        self._servo_defaults = (-21, 22, 135, 208)
        self._servo_vars = [tk.IntVar(value=v) for v in self._servo_defaults]

        self._build()

    def _build(self):
        # Top accent line
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="BURST CONTROL", font=FONT_BODY_BOLD,
                fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

        main = tk.Frame(self, bg=BG_PANEL)
        main.pack(fill="both", expand=True, padx=5, pady=5)

        # Left: Inflation controls
        left = tk.Frame(main, bg=BG_PANEL)
        left.pack(side="left", fill="both", expand=True, padx=5)

        infl_header = tk.Frame(left, bg=BG_PANEL)
        infl_header.pack(fill="x")
        tk.Frame(infl_header, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(infl_header, text="INFLATION CONTROL", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        self._infl_frame = tk.Frame(left, bg=BG_PANEL)
        self._infl_frame.pack(fill="both", expand=True, padx=5, pady=5)
        self._rebuild_modules()

        # Right: Per-servo bend controls. Sliders set target angles; nothing
        # is sent to the firmware until Save is pressed, so the arm doesn't
        # move as the user drags.
        right = tk.Frame(main, bg=BG_PANEL)
        right.pack(side="right", fill="both", padx=5)

        bend_header = tk.Frame(right, bg=BG_PANEL)
        bend_header.pack(fill="x")
        tk.Frame(bend_header, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(bend_header, text="SERVOS", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        bend_box = tk.Frame(right, bg=BG_PANEL)
        bend_box.pack(pady=10, padx=10, fill="both")

        # One horizontal slider per servo, 0..180°, starting at 90 (center).
        # Each slider writes its angle live as the user drags so the physical
        # servo tracks the slider. Save resends all four as a safety/confirm.
        servo_labels = [
            ("Servo A (pin 9)",  1),
            ("Servo B (pin 10)", 2),
            ("Servo C (pin 11)", 3),
            ("Servo D (pin 24)", 4),
        ]
        for label, servo_id in servo_labels:
            header = tk.Frame(bend_box, bg=BG_PANEL)
            header.pack(fill="x")
            tk.Label(header, text=label, font=FONT_LABEL,
                     fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left")
            tk.Button(
                header, text="\u25B6", font=("Courier", 9),
                fg=ACCENT_CYAN, bg=BG_PANEL, activebackground=BG_PRIMARY,
                activeforeground=ACCENT_CYAN, bd=0, padx=4, pady=0,
                highlightthickness=0,
                command=lambda sid=servo_id: self._nudge_servo(sid, +5),
            ).pack(side="right")
            tk.Button(
                header, text="\u25C0", font=("Courier", 9),
                fg=ACCENT_CYAN, bg=BG_PANEL, activebackground=BG_PRIMARY,
                activeforeground=ACCENT_CYAN, bd=0, padx=4, pady=0,
                highlightthickness=0,
                command=lambda sid=servo_id: self._nudge_servo(sid, -5),
            ).pack(side="right")
            tk.Scale(
                bend_box, from_=-1000, to=1000, orient="horizontal",
                variable=self._servo_vars[servo_id - 1], showvalue=True,
                length=220,
                bg=BG_PANEL, fg=TEXT_PRIMARY, troughcolor=BG_PRIMARY,
                highlightthickness=0, activebackground=ACCENT_CYAN,
                command=lambda val, sid=servo_id:
                    self._on_send(f"SERVO,{sid},{int(float(val))}"),
            ).pack(pady=(0, 6), fill="x")

        AccentButton(bend_box, text="Save", accent=ACCENT_GREEN,
                     command=self._on_save_servos).pack(
                         fill="x", pady=(4, 0))

        # Emergency stop (built once, at panel construction)
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(5, 10))

    def _nudge_servo(self, servo_id, delta):
        var = self._servo_vars[servo_id - 1]
        new_val = max(-1000, min(1000, var.get() + delta))
        var.set(new_val)
        self._on_send(f"SERVO,{servo_id},{new_val}")

    def _on_save_servos(self):
        self.push_servos()

    def current_servo_angles(self):
        """Current UI slider values in servo-id order (A, B, C, D)."""
        return tuple(int(v.get()) for v in self._servo_vars)

    def push_servos(self):
        """Send the bulk SERVOS command with the current slider values."""
        a, b, c, d = self.current_servo_angles()
        self._on_send(f"SERVOS,{a},{b},{c},{d}")

    def _rebuild_modules(self):
        """Rebuild inflate/deflate rows for all modules."""
        for w in self._infl_frame.winfo_children():
            w.destroy()

        for i, mod in enumerate(self._modules):
            row = tk.Frame(self._infl_frame, bg=BG_PANEL)
            row.pack(fill="x", pady=3)

            # Color dot
            tk.Label(row, text="\u25CF", fg=mod["color"], font=("Courier", 14),
                    bg=BG_PANEL).pack(side="left", padx=(0, 5))

            tk.Label(row, text=mod["name"], font=FONT_BODY,
                    fg=TEXT_PRIMARY, bg=BG_PANEL).pack(side="left", padx=(0, 8))

            mid = mod["id"]
            AccentButton(row, text="Inflate", accent=ACCENT_GREEN,
                        command=lambda m=mid: self._on_send(f"INFLATE,{m}")).pack(
                            side="left", padx=2)
            AccentButton(row, text="Deflate", accent=ACCENT_CYAN,
                        command=lambda m=mid: self._on_send(f"DEFLATE,{m}")).pack(
                            side="left", padx=2)

            # Pin info
            tk.Label(row, text=f"STEP={mod['step_pin']} DIR={mod['dir_pin']}",
                    font=("Courier", 8), fg=TEXT_SECONDARY, bg=BG_PANEL).pack(
                        side="left", padx=10)


    def rebuild(self):
        """Public method to rebuild module controls after add/remove."""
        self._rebuild_modules()
