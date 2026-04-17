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

        # Right: Bending controls
        right = tk.Frame(main, bg=BG_PANEL)
        right.pack(side="right", fill="both", padx=5)

        bend_header = tk.Frame(right, bg=BG_PANEL)
        bend_header.pack(fill="x")
        tk.Frame(bend_header, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(bend_header, text="BENDING", font=FONT_LABEL,
                fg=ACCENT_CYAN_DIM, bg=BG_PANEL).pack(anchor="w", padx=5, pady=(3, 0))

        bend_box = tk.Frame(right, bg=BG_PANEL)
        bend_box.pack(pady=10, padx=10, fill="both")

        # Horizontal layout: vertical slider on left, horizontal sliders on right
        bend_row = tk.Frame(bend_box, bg=BG_PANEL)
        bend_row.pack(fill="both", expand=True)

        # --- Left: All-servos vertical slider ---
        all_col = tk.Frame(bend_row, bg=BG_PANEL)
        all_col.pack(side="left", padx=(0, 15), anchor="n")

        tk.Label(all_col, text="All Servos", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack()
        tk.Label(all_col, text="Up", font=("Courier", 8),
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack()

        self._bend_all_val = tk.IntVar(value=0)
        self._bend_all_slider = tk.Scale(
            all_col, from_=100, to=-100, orient="vertical",
            variable=self._bend_all_val, showvalue=True, length=180,
            bg=BG_PANEL, fg=TEXT_PRIMARY, troughcolor=BG_PRIMARY,
            highlightthickness=0, activebackground=ACCENT_CYAN,
            command=self._on_bend_all_slide,
        )
        self._bend_all_slider.pack(pady=2)
        self._bend_all_slider.bind("<ButtonRelease-1>", self._on_bend_all_release)

        tk.Label(all_col, text="Down", font=("Courier", 8),
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack()
        AccentButton(all_col, text="Center", width=8,
                     command=self._bend_all_center).pack(pady=(6, 0))

        # --- Right: XY and XZ horizontal sliders stacked ---
        hz_col = tk.Frame(bend_row, bg=BG_PANEL)
        hz_col.pack(side="left", fill="both", expand=True, anchor="n")

        tk.Label(hz_col, text="XY Plane", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack()

        self._bend_val = tk.IntVar(value=0)
        self._bend_slider = tk.Scale(
            hz_col, from_=-100, to=100, orient="horizontal",
            variable=self._bend_val, showvalue=True, length=180,
            bg=BG_PANEL, fg=TEXT_PRIMARY, troughcolor=BG_PRIMARY,
            highlightthickness=0, activebackground=ACCENT_CYAN,
            command=self._on_bend_slide,
        )
        self._bend_slider.pack(pady=5)
        self._bend_slider.bind("<ButtonRelease-1>", self._on_bend_release)

        row = tk.Frame(hz_col, bg=BG_PANEL)
        row.pack()
        tk.Label(row, text="L", font=FONT_LABEL, fg=TEXT_SECONDARY,
                 bg=BG_PANEL).pack(side="left", padx=(0, 60))
        tk.Label(row, text="R", font=FONT_LABEL, fg=TEXT_SECONDARY,
                 bg=BG_PANEL).pack(side="left")

        AccentButton(hz_col, text="Center", width=8,
                     command=self._bend_center).pack(pady=(6, 0))

        # XZ plane
        tk.Label(hz_col, text="XZ Plane", font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(pady=(10, 0))

        self._bend_xz_val = tk.IntVar(value=0)
        self._bend_xz_slider = tk.Scale(
            hz_col, from_=-100, to=100, orient="horizontal",
            variable=self._bend_xz_val, showvalue=True, length=180,
            bg=BG_PANEL, fg=TEXT_PRIMARY, troughcolor=BG_PRIMARY,
            highlightthickness=0, activebackground=ACCENT_CYAN,
            command=self._on_bend_xz_slide,
        )
        self._bend_xz_slider.pack(pady=5)
        self._bend_xz_slider.bind("<ButtonRelease-1>", self._on_bend_xz_release)

        row_xz = tk.Frame(hz_col, bg=BG_PANEL)
        row_xz.pack()
        tk.Label(row_xz, text="L", font=FONT_LABEL, fg=TEXT_SECONDARY,
                 bg=BG_PANEL).pack(side="left", padx=(0, 60))
        tk.Label(row_xz, text="R", font=FONT_LABEL, fg=TEXT_SECONDARY,
                 bg=BG_PANEL).pack(side="left")

        AccentButton(hz_col, text="Center", width=8,
                     command=self._bend_xz_center).pack(pady=(6, 0))

        # Emergency stop (built once, at panel construction)
        EmergencyStopButton(self, command=self._on_emergency_stop).pack(
            fill="x", padx=10, pady=(5, 10))

    def _on_bend_slide(self, value):
        # Live update while dragging
        self._on_send(f"BEND,{int(float(value))}")

    def _on_bend_release(self, _evt):
        self._on_send(f"BEND,{int(self._bend_val.get())}")

    def _bend_center(self):
        self._bend_val.set(0)
        self._on_send("BEND,0")

    def _on_bend_xz_slide(self, value):
        self._on_send(f"BEND_XZ,{int(float(value))}")

    def _on_bend_xz_release(self, _evt):
        self._on_send(f"BEND_XZ,{int(self._bend_xz_val.get())}")

    def _bend_xz_center(self):
        self._bend_xz_val.set(0)
        self._on_send("BEND_XZ,0")

    def _on_bend_all_slide(self, value):
        self._on_send(f"BEND_ALL,{int(float(value))}")

    def _on_bend_all_release(self, _evt):
        self._on_send(f"BEND_ALL,{int(self._bend_all_val.get())}")

    def _bend_all_center(self):
        self._bend_all_val.set(0)
        self._on_send("BEND_ALL,0")

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
