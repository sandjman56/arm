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
