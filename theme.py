# theme.py -- Design system for Continuum Arm Controller
# Colors, fonts, and ttk theme configuration

from tkinter import ttk

# === Backgrounds ===
BG_PRIMARY = "#0a0e14"       # Main window
BG_PANEL = "#111822"         # Panel/card
BG_INPUT = "#1a2332"         # Entry fields, dropdowns
BG_HOVER = "#1e2d3d"         # Button hover
BG_BUTTON = "#162030"        # Button default

# === Accent Colors ===
ACCENT_CYAN = "#00f0ff"
ACCENT_CYAN_DIM = "#007a82"
ACCENT_RED = "#ff2244"
ACCENT_RED_DIM = "#991133"
ACCENT_ORANGE = "#ff9800"
ACCENT_GREEN = "#00ff88"
ACCENT_GREEN_DIM = "#00994d"

# === Text Colors ===
TEXT_PRIMARY = "#e0e6ed"
TEXT_SECONDARY = "#6b7b8d"
TEXT_ACCENT = "#00f0ff"
TEXT_DANGER = "#ff4466"

# === Module Colors (graph lines & pressure boxes) ===
MODULE_COLORS = [
    "#00ff88",  # green (Module 1)
    "#ff6644",  # orange-red
    "#44aaff",  # blue
    "#ffcc00",  # yellow
    "#cc44ff",  # purple
    "#ff44aa",  # pink
    "#44ffcc",  # teal
    "#ffaa44",  # amber
]

# === Fonts ===
FONT_TITLE = ("Courier", 22, "bold")
FONT_HEADING = ("Courier", 14, "bold")
FONT_BODY = ("Courier", 11)
FONT_BODY_BOLD = ("Courier", 11, "bold")
FONT_DATA_LARGE = ("Courier", 22, "bold")
FONT_DATA = ("Courier", 16, "bold")
FONT_DATA_SMALL = ("Courier", 11)
FONT_BUTTON = ("Courier", 11, "bold")
FONT_BUTTON_LARGE = ("Courier", 14, "bold")
FONT_ESTOP = ("Courier", 18, "bold")
FONT_STEPPER = ("Courier", 15, "bold")
FONT_STEPPER_LABEL = ("Courier", 9)
FONT_LABEL = ("Courier", 10)
FONT_SLIDER = ("Courier", 10)


def apply_theme(root):
    """Apply the dark futuristic theme to the root window and ttk styles."""
    root.configure(bg=BG_PRIMARY)

    style = ttk.Style()
    style.theme_use("clam")

    # General frame
    style.configure("TFrame", background=BG_PRIMARY)
    style.configure("Panel.TFrame", background=BG_PANEL)

    # Labels
    style.configure("TLabel",
                     background=BG_PRIMARY,
                     foreground=TEXT_PRIMARY,
                     font=FONT_BODY)
    style.configure("Heading.TLabel",
                     background=BG_PRIMARY,
                     foreground=ACCENT_CYAN,
                     font=FONT_HEADING)
    style.configure("Secondary.TLabel",
                     background=BG_PRIMARY,
                     foreground=TEXT_SECONDARY,
                     font=FONT_BODY)

    # Combobox
    style.configure("TCombobox",
                     fieldbackground=BG_INPUT,
                     background=BG_BUTTON,
                     foreground=TEXT_PRIMARY,
                     arrowcolor=ACCENT_CYAN,
                     font=FONT_BODY)
    style.map("TCombobox",
              fieldbackground=[("readonly", BG_INPUT)],
              foreground=[("readonly", TEXT_PRIMARY)])

    # Radiobutton
    style.configure("TRadiobutton",
                     background=BG_PRIMARY,
                     foreground=TEXT_PRIMARY,
                     font=FONT_BODY,
                     indicatorcolor=BG_INPUT,
                     focuscolor=BG_PRIMARY)
    style.map("TRadiobutton",
              background=[("active", BG_PRIMARY)],
              indicatorcolor=[("selected", ACCENT_CYAN)])

    # Scale (slider)
    style.configure("TScale",
                     background=BG_PRIMARY,
                     troughcolor=BG_INPUT,
                     sliderthickness=16)
    style.configure("Cyan.Horizontal.TScale",
                     background=BG_PRIMARY,
                     troughcolor=BG_INPUT)

    # LabelFrame
    style.configure("TLabelframe",
                     background=BG_PANEL,
                     foreground=ACCENT_CYAN,
                     font=FONT_BODY_BOLD)
    style.configure("TLabelframe.Label",
                     background=BG_PANEL,
                     foreground=ACCENT_CYAN,
                     font=FONT_BODY_BOLD)

    # Entry
    style.configure("TEntry",
                     fieldbackground=BG_INPUT,
                     foreground=TEXT_PRIMARY,
                     insertcolor=ACCENT_CYAN,
                     font=FONT_BODY)

    return style
