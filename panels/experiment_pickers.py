"""2D matplotlib pickers for Experiments mode — XY top view and XZ side view.

Each picker emits a callback with (x, y) or (x, z) when the user clicks inside
the reachable region. Clicks outside briefly flash red and are ignored.
"""
from __future__ import annotations
import math
import tkinter as tk
from typing import Callable, Optional, Tuple

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


class _BasePicker(tk.Frame):
    def __init__(self, parent, title: str, width_px: int = 260, height_px: int = 260, **kwargs):
        super().__init__(parent, **kwargs)
        self._title = title
        self._fig = Figure(figsize=(width_px / 100, height_px / 100), dpi=100)
        self._ax = self._fig.add_subplot(111)
        self._ax.set_title(title, fontsize=9)
        self._ax.set_aspect("equal", adjustable="box")
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._canvas.mpl_connect("button_press_event", self._on_click)
        self._on_pick: Optional[Callable[[float, float], None]] = None

    def bind_pick(self, cb: Callable[[float, float], None]) -> None:
        self._on_pick = cb

    def _on_click(self, event) -> None:
        if event.xdata is None or event.ydata is None:
            return
        if not self._is_inside(event.xdata, event.ydata):
            self._flash_invalid()
            return
        if self._on_pick is not None:
            self._on_pick(event.xdata, event.ydata)

    def _is_inside(self, u: float, v: float) -> bool:  # override
        return True

    def _flash_invalid(self) -> None:
        # Draw a momentary red cross at the click; clear on next redraw.
        # Minimal implementation: change the axis frame color briefly.
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
        # Tip + target markers.
        ax.plot([self._tip_xy[0]], [self._tip_xy[1]], marker="o", color="#0aa")
        if self._target_xy is not None:
            ax.plot([self._target_xy[0]], [self._target_xy[1]],
                    marker="+", color="#f80", markersize=15)
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
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
    """Side view XZ picker with a vertical guide line at locked X."""
    def __init__(self, parent, L_rest: float = 240.0, L_max: float = 480.0,
                 theta_max_rad: float = math.radians(60), **kwargs):
        super().__init__(parent, title="XZ (side view)", **kwargs)
        self._L_rest = L_rest
        self._L_max = L_max
        self._theta_max = theta_max_rad
        self._tip_xz: Tuple[float, float] = (0.0, L_rest)
        self._locked_x: Optional[float] = None
        self._target_xz: Optional[Tuple[float, float]] = None
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
        self._locked_x = x
        self._redraw()

    def set_target(self, x: Optional[float], z: Optional[float]) -> None:
        self._target_xz = (x, z) if x is not None and z is not None else None
        self._redraw()

    def _is_inside(self, x: float, z: float) -> bool:
        # Must lie inside the workspace cross-section AND on the guide line if locked.
        if self._locked_x is not None and abs(x - self._locked_x) > 2.0:
            return False
        # Quick reachability check in XZ: approximate with a dome of radius L_max.
        if z < 0:
            return False
        if math.hypot(x, z) > self._L_max * 1.02:
            return False
        return True

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        ax.set_title(self._title, fontsize=9)
        lim = self._L_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-10, lim)
        ax.set_aspect("equal", adjustable="box")
        # Reachable dome cross-section: L_min inner arc, L_max outer arc.
        inner = _dome_arc(self._L_rest, self._theta_max)
        outer = _dome_arc(self._L_max, self._theta_max)
        ax.plot(inner[0], inner[1], linestyle=":", color="#888")
        ax.plot(outer[0], outer[1], linestyle="--", color="#888")
        # Tip + guide + target.
        ax.plot([self._tip_xz[0]], [self._tip_xz[1]], marker="o", color="#0aa")
        if self._locked_x is not None:
            ax.axvline(self._locked_x, color="#0aa", linewidth=0.8, linestyle="--")
        if self._target_xz is not None:
            ax.plot([self._target_xz[0]], [self._target_xz[1]],
                    marker="+", color="#f80", markersize=15)
        ax.axhline(0, color="#444", linewidth=0.5)
        ax.axvline(0, color="#444", linewidth=0.5)
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
