# logger.py -- CSV data logging
import csv
import os
import time
from datetime import datetime


class CSVLogger:
    def __init__(self, output_dir="logger"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self._active = False
        self._file = None
        self._writer = None
        self._start_time = None

    @property
    def is_active(self):
        return self._active

    HEADER = [
        "time", "timestamp",
        "p1", "p2", "p3", "p4", "p5",
        "ax", "ay", "az", "gx", "gy", "gz",
        "s1", "s2", "s3", "s4",
        "pitch_deg", "roll_deg", "yaw_deg",
        "phase",
        "slack_deg", "p_sum_delta_psi",
        # Extended fields appended (evaluation panel reads by column name,
        # so extra columns are ignored by older readers).
        "p6",
        "pos1", "pos2", "pos3", "pos4", "pos5", "pos6",
    ]

    def start(self, prefix="log"):
        """Start logging. Returns the filename. `prefix` is prepended to the
        timestamped filename so specialty runs (e.g. `length_test`) are easy
        to spot alongside normal `log_*` runs."""
        if self._active:
            return None
        name = datetime.now().strftime(f"{prefix}_%Y%m%d_%H%M%S.csv")
        path = os.path.join(self.output_dir, name)
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(self.HEADER)
        self._start_time = time.time()
        self._active = True
        return name

    def stop(self):
        """Stop logging."""
        if not self._active:
            return
        self._file.close()
        self._file = None
        self._writer = None
        self._active = False

    def log(self, vals):
        """Write a data row with elapsed time and timestamp."""
        if not self._active:
            return
        t = time.time() - self._start_time
        ts = datetime.now().strftime("%H:%M:%S.%f")
        self._writer.writerow([t, ts] + vals)
        self._file.flush()
