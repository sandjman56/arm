"""2D matplotlib pickers for Experiments mode — XY top view and XZ side view.

Interaction model (post-revision):
- XY picker: click (or click + drag) anywhere inside the reachable circle to
  set (X, Y). A red dot marks the chosen point. Clicks outside the circle
  briefly flash the frame red and are ignored.
- XZ picker: GATED on XY-first. Until the user clicks XY, the XZ picker
  refuses clicks. Once XY is locked, a red dashed guide line appears at the
  locked X with a red draggable dot at the current Z. Clicking anywhere in
  the picker snaps the dot's X to the locked X and sets Z to the click's Y.
  Press + move continuously drags Z while the button is held.

Both pickers fire `_on_pick(x, y)` on every click AND every drag-motion step.
"""
from __future__ import annotations
import math
import tkinter as tk
from typing import Callable, Optional, Tuple

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class _BasePicker(tk.Frame):
    """Shared behavior: figure + axes + press/drag/release wiring."""

    def __init__(self, parent, title: str, width_px: int = 240, height_px: int = 240, **kwargs):
        super().__init__(parent, **kwargs)
        self._title = title
        # Smaller figsize so the canvas can shrink with the window.
        self._fig = Figure(figsize=(width_px / 100, height_px / 100), dpi=100)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_title(title, fontsize=9)
        self._ax.set_aspect("equal", adjustable="box")
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas.mpl_connect("button_press_event", self._on_press)
        self._canvas.mpl_connect("motion_notify_event", self._on_motion)
        self._canvas.mpl_connect("button_release_event", self._on_release)
        self._on_pick: Optional[Callable[[float, float], None]] = None
        self._dragging = False

    def bind_pick(self, cb: Callable[[float, float], None]) -> None:
        self._on_pick = cb

    # --- Event handlers (subclasses override fire()) ---------------------

    def _on_press(self, event) -> None:
        if event.xdata is None or event.ydata is None:
            return
        if not self._is_inside(event.xdata, event.ydata):
            self._flash_invalid()
            return
        self._dragging = True
        self._fire(event.xdata, event.ydata)

    def _on_motion(self, event) -> None:
        if not self._dragging:
            return
        if event.xdata is None or event.ydata is None:
            return
        if not self._is_inside(event.xdata, event.ydata):
            return
        self._fire(event.xdata, event.ydata)

    def _on_release(self, event) -> None:
        self._dragging = False

    def _fire(self, x: float, y: float) -> None:
        if self._on_pick is not None:
            self._on_pick(x, y)

    def _is_inside(self, u: float, v: float) -> bool:  # override
        return True

    def _flash_invalid(self) -> None:
        for spine in self._ax.spines.values():
            spine.set_color("red")
        self._canvas.draw_idle()
        self.after(200, self._clear_flash)

    def _clear_flash(self) -> None:
        for spine in self._ax.spines.values():
            spine.set_color("black")
        self._canvas.draw_idle()


class XYPicker(_BasePicker):
    """Top-down XY picker. `r_max` is the reachable horizontal radius."""

    def __init__(self, parent, r_max: float = 400.0, **kwargs):
        super().__init__(parent, title="XY (top view)", **kwargs)
        self._r_max = r_max
        self._tip_xy: Tuple[float, float] = (0.0, 0.0)
        self._target_xy: Optional[Tuple[float, float]] = None
        self._reached = False
        self._redraw()

    def set_r_max(self, r_max: float) -> None:
        self._r_max = r_max
        self._redraw()

    def set_tip(self, x: float, y: float) -> None:
        self._tip_xy = (x, y)
        self._redraw()

    def set_target(self, x: Optional[float], y: Optional[float]) -> None:
        self._target_xy = (x, y) if x is not None and y is not None else None
        self._redraw()

    def set_reached(self, reached: bool) -> None:
        if self._reached == reached:
            return
        self._reached = reached
        self._redraw()

    def _is_inside(self, x: float, y: float) -> bool:
        return math.hypot(x, y) <= self._r_max

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        ax.set_title(self._title, fontsize=9)
        lim = self._r_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_aspect("equal", adjustable="box")
        # Reachable circle.
        circle = plt_circle(0.0, 0.0, self._r_max)
        ax.plot(circle[0], circle[1], linestyle="--", color="#888")
        # Axes.
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
        # Tip marker (teal).
        ax.plot([self._tip_xy[0]], [self._tip_xy[1]], marker="o", color="#0aa", markersize=7)
        # Target marker (red normally, green once reached).
        if self._target_xy is not None:
            tcolor = "#00d060" if self._reached else "red"
            ax.plot([self._target_xy[0]], [self._target_xy[1]],
                    marker="o", color=tcolor, markersize=9, markeredgecolor=tcolor,
                    markerfacecolor=tcolor)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Y (mm)")
        self._canvas.draw_idle()


def plt_circle(cx: float, cy: float, r: float, n: int = 64) -> Tuple[list, list]:
    xs, ys = [], []
    for i in range(n + 1):
        t = 2 * math.pi * i / n
        xs.append(cx + r * math.cos(t))
        ys.append(cy + r * math.sin(t))
    return xs, ys


class XZPicker(_BasePicker):
    """Side-view XZ picker.

    Gated: until `set_locked_x(x)` is called (by the panel after the user
    clicks the XY picker), the picker will not accept clicks or drags.
    Once locked, a red dashed guide line appears at x=locked_x and a red
    draggable dot appears on the line at the current Z. The picker snaps
    X to locked_x regardless of where the user clicks horizontally —
    only Z moves during drag.
    """

    def __init__(self, parent, L_rest: float = 240.0, L_max: float = 480.0,
                 theta_max_rad: float = math.radians(60), **kwargs):
        super().__init__(parent, title="XZ (side view)", **kwargs)
        self._L_rest = L_rest
        self._L_max = L_max
        self._theta_max = theta_max_rad
        self._tip_xz: Tuple[float, float] = (0.0, L_rest)
        self._locked_x: Optional[float] = None
        # Current Z of the draggable dot (None if no lock yet).
        self._dot_z: Optional[float] = None
        self._reached = False
        self._redraw()

    def set_reached(self, reached: bool) -> None:
        if self._reached == reached:
            return
        self._reached = reached
        self._redraw()

    def set_workspace(self, L_rest: float, L_max: float, theta_max_rad: float) -> None:
        self._L_rest = L_rest
        self._L_max = L_max
        self._theta_max = theta_max_rad
        self._redraw()

    def set_tip(self, x: float, z: float) -> None:
        self._tip_xz = (x, z)
        self._redraw()

    def set_locked_x(self, x: Optional[float]) -> None:
        """Lock X (from XY pick) and reset the draggable dot to a default Z.

        The default Z sits on the outer dome at the locked X so the dot is
        visually in the middle of the reachable height at that X.
        """
        self._locked_x = x
        if x is None:
            self._dot_z = None
        else:
            self._dot_z = self._default_z_for(x)
        self._redraw()

    def set_target(self, x: Optional[float], z: Optional[float]) -> None:
        """Programmatic target set (e.g., from text entry or Reach caller)."""
        if x is None or z is None:
            self._dot_z = None
        else:
            self._locked_x = x
            self._dot_z = self._clamp_z(x, z)
        self._redraw()

    def get_current_z(self) -> Optional[float]:
        return self._dot_z

    # --- Overrides --------------------------------------------------------

    def _on_press(self, event) -> None:
        if self._locked_x is None:
            # Gate: user must pick XY first.
            self._flash_invalid()
            return
        if event.xdata is None or event.ydata is None:
            return
        self._dragging = True
        self._update_dot_from_y(event.ydata)

    def _on_motion(self, event) -> None:
        if not self._dragging:
            return
        if self._locked_x is None or event.ydata is None:
            return
        self._update_dot_from_y(event.ydata)

    def _update_dot_from_y(self, z_raw: float) -> None:
        assert self._locked_x is not None
        z = self._clamp_z(self._locked_x, z_raw)
        self._dot_z = z
        self._redraw()
        self._fire(self._locked_x, z)

    # --- Geometry helpers --------------------------------------------------

    def _clamp_z(self, x: float, z: float) -> float:
        """Clamp z into the reachable half-dome at horizontal position x."""
        if z < 0:
            z = 0.0
        # Outer bound: sqrt(x^2 + z^2) <= L_max
        r = self._L_max
        if x * x + z * z > r * r:
            # Project onto the dome at this x.
            z = math.sqrt(max(0.0, r * r - x * x))
        # Inner bound: L_rest for interior, but clamp softly so it's pickable.
        # Don't enforce an inner wall here — it would make the dot jumpy.
        return z

    def _default_z_for(self, x: float) -> float:
        """Default Z for the red dot after an XY pick: midway of the dome."""
        # Midpoint between 0 and the outer dome at that x.
        outer = math.sqrt(max(0.0, self._L_max * self._L_max - x * x))
        return outer * 0.75  # slightly above midpoint so it's clearly inside

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        ax.set_title(self._title, fontsize=9)
        lim = self._L_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-10, lim)
        ax.set_aspect("equal", adjustable="box")
        # Reachable dome cross-section: L_rest inner arc, L_max outer arc.
        inner = _dome_arc(self._L_rest, self._theta_max)
        outer = _dome_arc(self._L_max, self._theta_max)
        ax.plot(inner[0], inner[1], linestyle=":", color="#888")
        ax.plot(outer[0], outer[1], linestyle="--", color="#888")
        # Axes.
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
        # Current tip (teal).
        ax.plot([self._tip_xz[0]], [self._tip_xz[1]], marker="o", color="#0aa", markersize=7)
        # Guide line + draggable target dot (red normally, green once reached).
        if self._locked_x is not None:
            tcolor = "#00d060" if self._reached else "red"
            ax.axvline(self._locked_x, color=tcolor, linewidth=1.2, linestyle="--")
            if self._dot_z is not None:
                ax.plot([self._locked_x], [self._dot_z],
                        marker="o", color=tcolor, markersize=10,
                        markeredgecolor=tcolor, markerfacecolor=tcolor)
        ax.set_xlabel("X (mm)")
        ax.set_ylabel("Z (mm)")
        self._canvas.draw_idle()


def _dome_arc(L: float, theta_max: float, n: int = 40) -> Tuple[list, list]:
    """Generate (x, z) points of a PCC arc swept from -theta_max to +theta_max."""
    xs, zs = [], []
    for i in range(n + 1):
        t = -theta_max + (2 * theta_max) * i / n
        if abs(t) < 1e-9:
            xs.append(0.0)
            zs.append(L)
        else:
            R = L / abs(t)
            r = R * (1.0 - math.cos(t))
            z = R * math.sin(abs(t))
            xs.append(math.copysign(r, t))
            zs.append(z)
    return xs, zs
