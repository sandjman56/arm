"""Smoke test: panel instantiates without a live controller or serial.

This test is opt-in because Tk crashes the Python process at the C level on some
macOS/pyenv combinations (no try/except can catch it). Set RUN_UI_TESTS=1 to
run it in an environment where Tk is known to work.
"""
import os
import pytest


@pytest.mark.skipif(
    os.environ.get("RUN_UI_TESTS") != "1",
    reason="set RUN_UI_TESTS=1 to run Tk smoke tests",
)
def test_panel_instantiates_offline():
    import tkinter as tk
    try:
        root = tk.Tk()
    except tk.TclError as e:
        pytest.skip(f"Tk not available in this environment: {e}")
    root.withdraw()
    from panels.experiment_panel import ExperimentPanel
    panel = ExperimentPanel(
        root,
        on_start_zero=lambda: None,
        on_confirm_zero=lambda: None,
        on_rezero=lambda: None,
        on_reach=lambda t: None,
        on_reach_basic=lambda z, t, s: None,
        on_emergency_stop=lambda: None,
    )
    panel.pack()
    root.update()
    assert panel.status_var.get().startswith("IDLE")
    root.destroy()
