# panels/evaluation_panel.py -- Evaluation mode: plot pressure/elongation from logged CSVs
import csv
import math
import os
import re
import tkinter as tk
from tkinter import filedialog, messagebox

from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

from theme import (
    BG_PANEL, BG_PRIMARY, BG_INPUT,
    ACCENT_CYAN, ACCENT_CYAN_DIM, ACCENT_GREEN, ACCENT_ORANGE,
    TEXT_PRIMARY, TEXT_SECONDARY,
    FONT_BODY, FONT_BODY_BOLD, FONT_LABEL,
    MODULE_COLORS,
)
from widgets import AccentButton, StyledLabelFrame


# Must match experiment_controller.ExperimentController.PULLEY_RADIUS_MM.
PULLEY_RADIUS_MM = 25.0


def _csv_to_series(path):
    """Parse a logger CSV into time/pressure/elongation arrays.

    Elongation is derived from the mean delta of s1..s4 (tendon servo angles)
    relative to the first row — the servos unwind in lockstep, so their common
    displacement is the slack angle, and elongation_mm = -radians(slack) * R.
    The CSV's `slack_deg` column is only used as a fallback when servos
    aren't present or are all constant.

    Returns dict with keys: t, p1..p5, elong_mm, name. Rows with unparseable
    time are skipped so partial runs still plot.
    """
    t = []
    pressures = {k: [] for k in ("p1", "p2", "p3", "p4", "p5")}
    servos = {k: [] for k in ("s1", "s2", "s3", "s4")}
    slack_col = []
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                ti = float(row["time"])
            except (TypeError, ValueError, KeyError):
                continue
            t.append(ti)
            for k in pressures:
                try:
                    pressures[k].append(float(row[k]))
                except (TypeError, ValueError, KeyError):
                    pressures[k].append(math.nan)
            for k in servos:
                try:
                    servos[k].append(float(row[k]))
                except (TypeError, ValueError, KeyError):
                    servos[k].append(math.nan)
            try:
                slack_col.append(float(row["slack_deg"]))
            except (TypeError, ValueError, KeyError):
                slack_col.append(math.nan)

    elong = _elongation_mm_series(servos, slack_col)
    name = os.path.basename(path)
    target_mm = _parse_target_mm(name)
    stats = _elongation_stats(t, elong, target_mm)
    return {
        "name": name,
        "t": t,
        **pressures,
        "elong_mm": elong,
        "target_mm": target_mm,
        "stats": stats,
    }


_TARGET_RE = re.compile(r"(?:^|[_\-])(\d+(?:\.\d+)?)\s*mm(?:[_\-\.]|$)", re.IGNORECASE)


def _parse_target_mm(filename):
    """Extract the target elongation in mm from a filename like
    `Elong_20mm_20260423_133033.csv`. Returns None if no `<N>mm` token is
    present."""
    m = _TARGET_RE.search(filename)
    if not m:
        return None
    try:
        return float(m.group(1))
    except ValueError:
        return None


def _elongation_stats(t, elong_mm, target_mm):
    """Compute peak/final/error stats for an elongation trace.

    If `target_mm` is None, only peak/final are reported and error fields
    are left as None. `rmse_after_reach_mm` averages squared error from the
    first crossing of the target onward (steady-state tracking error) —
    skipped if the target is never reached.
    """
    real = [(ti, e) for ti, e in zip(t, elong_mm) if not math.isnan(e)]
    if not real:
        return {
            "target_mm": target_mm, "peak_mm": None, "final_mm": None,
            "peak_err_mm": None, "final_err_mm": None, "overshoot_mm": None,
            "t_reach_s": None, "rmse_after_reach_mm": None,
        }
    peak = max(e for _, e in real)
    final = real[-1][1]
    t_reach = None
    rmse_after = None
    peak_err = final_err = overshoot = None
    if target_mm is not None:
        peak_err = peak - target_mm
        final_err = final - target_mm
        overshoot = max(0.0, peak - target_mm)
        for ti, e in real:
            if e >= target_mm:
                t_reach = ti
                break
        if t_reach is not None:
            tail = [e for ti, e in real if ti >= t_reach]
            if tail:
                sq = sum((e - target_mm) ** 2 for e in tail)
                rmse_after = math.sqrt(sq / len(tail))
    return {
        "target_mm": target_mm,
        "peak_mm": peak,
        "final_mm": final,
        "peak_err_mm": peak_err,
        "final_err_mm": final_err,
        "overshoot_mm": overshoot,
        "t_reach_s": t_reach,
        "rmse_after_reach_mm": rmse_after,
    }


def _format_stats(stats):
    """Multi-line stats block for in-plot annotation. Omits target/error
    lines when no target was parsed from the filename."""
    lines = []
    if stats["peak_mm"] is None:
        return "no elongation data"
    if stats["target_mm"] is not None:
        lines.append(f"target:     {stats['target_mm']:.2f} mm")
    lines.append(f"peak:       {stats['peak_mm']:.2f} mm")
    lines.append(f"final:      {stats['final_mm']:.2f} mm")
    if stats["target_mm"] is not None:
        lines.append(f"peak err:   {stats['peak_err_mm']:+.2f} mm")
        lines.append(f"final err:  {stats['final_err_mm']:+.2f} mm")
        lines.append(f"overshoot:  {stats['overshoot_mm']:.2f} mm")
        if stats["t_reach_s"] is not None:
            lines.append(f"t_reach:    {stats['t_reach_s']:.2f} s")
        else:
            lines.append("t_reach:    (target not reached)")
        if stats["rmse_after_reach_mm"] is not None:
            lines.append(f"rmse post:  {stats['rmse_after_reach_mm']:.2f} mm")
    return "\n".join(lines)


def _elongation_mm_series(servos, slack_col):
    """Compute per-row elongation in mm from servo angle deltas, falling back
    to the logged `slack_deg` column if the servo columns are unusable."""
    cols = [servos[k] for k in ("s1", "s2", "s3", "s4")]
    # Find first row where every servo reads a finite number — that row
    # becomes the rest reference.
    n = len(cols[0]) if cols else 0
    rest = None
    for i in range(n):
        vals = [c[i] for c in cols]
        if all(not math.isnan(v) for v in vals):
            rest = vals
            break
    servos_usable = rest is not None and any(
        any(not math.isnan(v) and abs(v - rest[j]) > 1e-6 for v in cols[j])
        for j in range(4)
    )
    out = []
    if servos_usable:
        for i in range(n):
            vals = [cols[j][i] for j in range(4)]
            deltas = [v - rest[j] for j, v in enumerate(vals) if not math.isnan(v)]
            if not deltas:
                out.append(math.nan)
                continue
            slack = sum(deltas) / len(deltas)
            out.append(-math.radians(slack) * PULLEY_RADIUS_MM)
        return out
    # Fallback: use logged slack_deg column directly.
    for s in slack_col:
        if math.isnan(s):
            out.append(math.nan)
        else:
            out.append(-math.radians(s) * PULLEY_RADIUS_MM)
    return out


def _build_figure(series):
    """Build a 2-row figure: pressure vs time on top, elongation vs time below."""
    fig = Figure(figsize=(7.5, 4.6), dpi=100, facecolor=BG_PRIMARY)
    ax_p = fig.add_subplot(2, 1, 1)
    ax_e = fig.add_subplot(2, 1, 2, sharex=ax_p)

    for ax in (ax_p, ax_e):
        ax.set_facecolor(BG_PANEL)
        for spine in ax.spines.values():
            spine.set_color(ACCENT_CYAN_DIM)
        ax.tick_params(colors=TEXT_SECONDARY, labelsize=8)
        ax.grid(True, color="#1a2332", linewidth=0.5)

    t = series["t"]
    for i, key in enumerate(("p1", "p2", "p3", "p4", "p5")):
        vals = series[key]
        if any(not math.isnan(v) for v in vals):
            ax_p.plot(t, vals, color=MODULE_COLORS[i % len(MODULE_COLORS)],
                      linewidth=1.2, label=f"M{i+1}")
    ax_p.set_ylabel("Pressure (psi)", color=TEXT_PRIMARY, fontsize=9)
    ax_p.set_title(series["name"], color=ACCENT_CYAN, fontsize=10, loc="left")
    leg = ax_p.legend(loc="upper right", fontsize=7, facecolor=BG_INPUT,
                      edgecolor=ACCENT_CYAN_DIM, labelcolor=TEXT_PRIMARY)
    if leg:
        for text in leg.get_texts():
            text.set_color(TEXT_PRIMARY)

    ax_e.plot(t, series["elong_mm"], color=ACCENT_ORANGE, linewidth=1.4,
              label="measured")
    target = series.get("target_mm")
    if target is not None and t:
        ax_e.plot([t[0], t[-1]], [target, target],
                  color=TEXT_SECONDARY, linewidth=1.0, linestyle=(0, (3, 3)),
                  label=f"target {target:g} mm")
        leg_e = ax_e.legend(loc="lower right", fontsize=7, facecolor=BG_INPUT,
                            edgecolor=ACCENT_CYAN_DIM, labelcolor=TEXT_PRIMARY)
        if leg_e:
            for tx in leg_e.get_texts():
                tx.set_color(TEXT_PRIMARY)
    ax_e.set_ylabel("Elongation (mm)", color=TEXT_PRIMARY, fontsize=9)
    ax_e.set_xlabel("Time (s)", color=TEXT_PRIMARY, fontsize=9)

    stats_text = _format_stats(series.get("stats") or {})
    if stats_text:
        ax_e.text(
            0.01, 0.98, stats_text,
            transform=ax_e.transAxes, ha="left", va="top",
            family="monospace", fontsize=7, color=TEXT_PRIMARY,
            bbox=dict(facecolor=BG_INPUT, edgecolor=ACCENT_CYAN_DIM,
                      linewidth=0.5, boxstyle="round,pad=0.3"),
        )

    fig.tight_layout()
    return fig


class EvaluationPanel(tk.Frame):
    """Evaluation panel: load CSVs, render per-file pressure/elongation plots,
    and export all plots as PNGs into logger/graphs/."""

    def __init__(self, parent, logger_dir, **kwargs):
        super().__init__(parent, bg=BG_PANEL, **kwargs)
        self._logger_dir = logger_dir
        self._graphs_dir = os.path.join(logger_dir, "graphs")
        # Each entry: {"path": str, "series": dict, "figure": Figure,
        #              "canvas": FigureCanvasTkAgg, "frame": tk.Frame}
        self._loaded = []

        self._build()

    def _build(self):
        tk.Frame(self, bg=ACCENT_CYAN_DIM, height=1).pack(fill="x")
        tk.Label(self, text="EVALUATION", font=FONT_BODY_BOLD,
                 fg=ACCENT_CYAN, bg=BG_PANEL).pack(anchor="w", padx=10, pady=(6, 0))

        # Controls row
        ctl = tk.Frame(self, bg=BG_PANEL)
        ctl.pack(fill="x", padx=10, pady=(6, 2))

        AccentButton(ctl, text="Load CSV(s)...", accent=ACCENT_CYAN,
                     command=self._load_csvs).pack(side="left", padx=(0, 6))
        AccentButton(ctl, text="Clear", command=self._clear).pack(side="left", padx=6)
        AccentButton(ctl, text="Save All Graphs", accent=ACCENT_GREEN,
                     command=self._save_all).pack(side="left", padx=6)

        self._status_var = tk.StringVar(value="No files loaded.")
        tk.Label(ctl, textvariable=self._status_var, font=FONT_LABEL,
                 fg=TEXT_SECONDARY, bg=BG_PANEL).pack(side="left", padx=(12, 0))

        # Scrollable plot area. Each CSV gets its own figure stacked vertically.
        outer = tk.Frame(self, bg=BG_PANEL)
        outer.pack(fill="both", expand=True, padx=10, pady=6)

        self._canvas_host = tk.Canvas(outer, bg=BG_PRIMARY, highlightthickness=0)
        vbar = tk.Scrollbar(outer, orient="vertical", command=self._canvas_host.yview)
        self._canvas_host.configure(yscrollcommand=vbar.set)
        vbar.pack(side="right", fill="y")
        self._canvas_host.pack(side="left", fill="both", expand=True)

        self._plots_frame = tk.Frame(self._canvas_host, bg=BG_PRIMARY)
        self._plots_window = self._canvas_host.create_window(
            (0, 0), window=self._plots_frame, anchor="nw",
        )

        def _on_plots_resize(event):
            self._canvas_host.configure(scrollregion=self._canvas_host.bbox("all"))
        self._plots_frame.bind("<Configure>", _on_plots_resize)

        def _on_host_resize(event):
            self._canvas_host.itemconfigure(self._plots_window, width=event.width)
        self._canvas_host.bind("<Configure>", _on_host_resize)

    # ---- actions ----

    def _load_csvs(self):
        initial = self._logger_dir if os.path.isdir(self._logger_dir) else "."
        paths = filedialog.askopenfilenames(
            parent=self, title="Choose CSV log(s)",
            initialdir=initial,
            filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        )
        if not paths:
            return
        self._clear()
        errors = []
        for p in paths:
            try:
                series = _csv_to_series(p)
                if not series["t"]:
                    errors.append(f"{os.path.basename(p)}: no data rows")
                    continue
                self._add_plot(p, series)
            except Exception as e:
                errors.append(f"{os.path.basename(p)}: {e}")
        if errors:
            messagebox.showwarning(
                "Some files could not be loaded",
                "\n".join(errors),
                parent=self,
            )
        self._refresh_status()

    def _add_plot(self, path, series):
        frame = tk.Frame(self._plots_frame, bg=BG_PANEL, bd=1, relief="flat",
                         highlightthickness=1, highlightbackground=ACCENT_CYAN_DIM)
        frame.pack(fill="x", padx=2, pady=4)
        fig = _build_figure(series)
        canvas = FigureCanvasTkAgg(fig, master=frame)
        canvas.draw()
        canvas.get_tk_widget().pack(fill="both", expand=True)
        self._loaded.append({
            "path": path, "series": series,
            "figure": fig, "canvas": canvas, "frame": frame,
        })

    def _clear(self):
        for entry in self._loaded:
            try:
                entry["canvas"].get_tk_widget().destroy()
            except Exception:
                pass
            try:
                entry["frame"].destroy()
            except Exception:
                pass
        self._loaded = []
        self._refresh_status()

    def _save_all(self):
        if not self._loaded:
            messagebox.showinfo("Save Graphs", "Load at least one CSV first.",
                                parent=self)
            return
        try:
            os.makedirs(self._graphs_dir, exist_ok=True)
        except Exception as e:
            messagebox.showerror("Save Graphs",
                                 f"Could not create {self._graphs_dir}: {e}",
                                 parent=self)
            return
        saved = []
        errors = []
        for entry in self._loaded:
            base = os.path.splitext(os.path.basename(entry["path"]))[0]
            out = os.path.join(self._graphs_dir, f"{base}.png")
            try:
                entry["figure"].savefig(out, dpi=150, facecolor=BG_PRIMARY)
                saved.append(os.path.basename(out))
            except Exception as e:
                errors.append(f"{base}: {e}")
        msg = f"Saved {len(saved)} graph(s) to {self._graphs_dir}"
        if errors:
            msg += "\n\nErrors:\n" + "\n".join(errors)
        messagebox.showinfo("Save Graphs", msg, parent=self)
        self._status_var.set(f"Saved {len(saved)} graph(s) to logger/graphs/")

    def _refresh_status(self):
        n = len(self._loaded)
        if n == 0:
            self._status_var.set("No files loaded.")
        else:
            self._status_var.set(f"{n} file(s) loaded.")
