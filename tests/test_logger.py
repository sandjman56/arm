"""Tests for CSVLogger prefix/header customization."""
import csv
import os
import pytest
from logger import CSVLogger


def test_default_prefix_is_log(tmp_path):
    lg = CSVLogger(output_dir=str(tmp_path))
    name = lg.start()
    assert name is not None
    assert name.startswith("log_")
    assert name.endswith(".csv")
    lg.stop()


def test_custom_prefix_and_header(tmp_path):
    lg = CSVLogger(output_dir=str(tmp_path))
    header = ["time", "timestamp", "target_x", "target_y", "target_z", "phase"]
    name = lg.start(filename_prefix="PointSelect", header=header)
    assert name is not None
    assert name.startswith("PointSelect_")
    lg.log([10.0, 20.0, 300.0, "ELONGATING"])
    path = lg.stop()
    assert path is not None
    assert os.path.exists(path)

    with open(path, "r", newline="") as f:
        rows = list(csv.reader(f))
    assert rows[0] == header
    # One data row + header.
    assert len(rows) == 2
    # Last 4 cells are the payload we passed.
    assert rows[1][-4:] == ["10.0", "20.0", "300.0", "ELONGATING"]


def test_stop_is_idempotent(tmp_path):
    lg = CSVLogger(output_dir=str(tmp_path))
    assert lg.stop() is None  # no-op when inactive
    lg.start()
    assert lg.stop() is not None
    assert lg.stop() is None  # already stopped


def test_double_start_is_noop(tmp_path):
    lg = CSVLogger(output_dir=str(tmp_path))
    lg.start()
    assert lg.start() is None
    lg.stop()


def test_log_while_inactive_is_noop(tmp_path):
    lg = CSVLogger(output_dir=str(tmp_path))
    lg.log([1, 2, 3])  # should not raise
