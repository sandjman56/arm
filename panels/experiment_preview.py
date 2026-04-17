"""Read-only 3D preview of the trunk for Experiments mode."""
from __future__ import annotations
import math
import tkinter as tk
from typing import Optional, Tuple

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 - registers projection


class TrunkPreview3D(tk.Frame):
    """3D render of base plate + trunk arc + target crosshair + reachable dome."""

    def __init__(self, parent, L_max: float = 480.0, theta_max: float = math.radians(60), **kwargs):
        super().__init__(parent, **kwargs)
        self._L_max = L_max
        self._theta_max = theta_max
        self._fig = Figure(figsize=(2.8, 2.8), dpi=100)
        self._ax = self._fig.add_subplot(111, projection="3d")
        self._canvas = FigureCanvasTkAgg(self._fig, master=self)
        self._canvas.get_tk_widget().pack(fill="both", expand=True)
        self._tip: Tuple[float, float, float] = (0.0, 0.0, 240.0)
        self._arc_points = [(0.0, 0.0, z) for z in range(0, 241, 12)]
        self._target: Optional[Tuple[float, float, float]] = None
        self._redraw()

    def set_tip_and_arc(self, tip: Tuple[float, float, float],
                        arc_points: list) -> None:
        self._tip = tip
        self._arc_points = arc_points
        self._redraw()

    def set_target(self, target: Optional[Tuple[float, float, float]]) -> None:
        self._target = target
        self._redraw()

    def _redraw(self) -> None:
        ax = self._ax
        ax.clear()
        lim = self._L_max * 1.1
        ax.set_xlim(-lim, lim)
        ax.set_ylim(-lim, lim)
        ax.set_zlim(0, lim)
        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")
        # Base plate (small square at z=0).
        sq = 40.0
        ax.plot([-sq, sq, sq, -sq, -sq], [-sq, -sq, sq, sq, -sq], [0, 0, 0, 0, 0], color="#444")
        # Trunk arc.
        if self._arc_points:
            xs = [p[0] for p in self._arc_points]
            ys = [p[1] for p in self._arc_points]
            zs = [p[2] for p in self._arc_points]
            ax.plot(xs, ys, zs, color="#0aa", linewidth=2)
        # Tip marker.
        ax.scatter([self._tip[0]], [self._tip[1]], [self._tip[2]], color="#0aa", s=40)
        # Target marker.
        if self._target is not None:
            ax.scatter([self._target[0]], [self._target[1]], [self._target[2]],
                       color="#f80", s=60, marker="+")
        self._canvas.draw_idle()
