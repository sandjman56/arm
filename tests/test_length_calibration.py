"""Unit tests for length_calibration.py."""
import json
import pytest
from length_calibration import (
    LengthCalibration,
    load_or_default,
    DEFAULT_WORKSPACE,
)


def test_default_workspace_when_file_missing(tmp_path):
    path = tmp_path / "does_not_exist.json"
    cal = load_or_default(str(path))
    assert cal.is_default is True
    assert cal.L_rest == DEFAULT_WORKSPACE["L_rest_mm"]
    assert cal.L_max == DEFAULT_WORKSPACE["L_max_mm"]


def test_save_load_roundtrip(tmp_path):
    path = tmp_path / "cal.json"
    original = LengthCalibration(
        modules={
            1: {"rest_length_mm": 42.0, "coeffs": [42.0, 3.0, 0.1], "max_psi": 8.0},
            2: {"rest_length_mm": 41.5, "coeffs": [41.5, 3.1, 0.05], "max_psi": 8.0},
        },
        missing_modules=[5, 6],
    )
    original.save(str(path))
    loaded = load_or_default(str(path))
    assert loaded.is_default is False
    assert loaded.modules[1]["rest_length_mm"] == 42.0
    assert loaded.missing_modules == [5, 6]


from length_calibration import fit_module_curve


def test_fit_module_curve_recovers_linear_samples():
    # Synthetic: length = 40 + 5*psi, no curvature
    samples = [(0.0, 40.0), (2.0, 50.0), (4.0, 60.0), (6.0, 70.0), (8.0, 80.0)]
    coeffs = fit_module_curve(samples)
    assert coeffs[0] == pytest.approx(40.0, abs=1e-6)
    assert coeffs[1] == pytest.approx(5.0, abs=1e-6)
    assert coeffs[2] == pytest.approx(0.0, abs=1e-6)


def test_fit_module_curve_recovers_quadratic_samples():
    # length = 40 + 3*psi + 0.1*psi^2
    samples = [(psi, 40 + 3 * psi + 0.1 * psi * psi) for psi in (0, 2, 4, 6, 8)]
    coeffs = fit_module_curve(samples)
    assert coeffs[0] == pytest.approx(40.0, abs=1e-6)
    assert coeffs[1] == pytest.approx(3.0, abs=1e-6)
    assert coeffs[2] == pytest.approx(0.1, abs=1e-6)


def test_fit_module_curve_requires_at_least_three_samples():
    with pytest.raises(ValueError):
        fit_module_curve([(0.0, 40.0), (8.0, 80.0)])


def test_dialog_flow_builds_calibration(monkeypatch, tmp_path):
    """Drive run_length_calibration_dialog with patched simpledialog."""
    import panels.experiment_calibration as calmod

    class FakeSD:
        seq = iter([40.0, 50.0, 60.0, 70.0, 80.0])  # linear length(psi)=40+5*psi
        @staticmethod
        def askfloat(*args, **kwargs):
            return next(FakeSD.seq)

    monkeypatch.setattr(calmod.simpledialog, "askfloat", FakeSD.askfloat)
    sent = []
    cal = calmod.run_length_calibration_dialog(
        root=None,
        available_modules=[1],
        command_pressure=lambda mid, psi: sent.append((mid, psi)),
        wait_for_steady=lambda: None,
        max_psi=8.0,
        num_points=5,
    )
    assert cal is not None
    assert 1 in cal.modules
    assert cal.modules[1]["coeffs"][0] == pytest.approx(40.0, abs=1e-4)
    assert cal.modules[1]["coeffs"][1] == pytest.approx(5.0, abs=1e-4)
