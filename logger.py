# logger.py -- CSV data logging
import csv
import os
import time
from datetime import datetime


class CSVLogger:
    """Simple CSV logger. Supports a custom filename prefix and header row.

    Default behavior (no args) matches the original logger used by the main
    pressure-graph logging flow:
        filename = log_YYYYMMDD_HHMMSS.csv
        columns  = time, timestamp, p1, p2, p3, p4

    Experiments-mode auto-log flow opts into a different prefix and header:
        filename = PointSelect_YYYYMMDD_HHMMSS.csv
        columns  = time, timestamp, target_x, target_y, target_z,
                   tip_x, tip_y, tip_z, pitch_deg, roll_deg, yaw_deg,
                   total_length_mm, phase, psi_1, ..., psi_6, error_text
    """

    def __init__(self, output_dir="logger"):
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self._active = False
        self._file = None
        self._writer = None
        self._start_time = None
        self._current_path = None

    @property
    def is_active(self):
        return self._active

    @property
    def current_path(self):
        return self._current_path

    def start(self, filename_prefix="log", header=None):
        """Start logging. Returns the filename (not the full path), or None if
        already active.

        filename_prefix: used in "<prefix>_YYYYMMDD_HHMMSS.csv".
        header:          list of column names; defaults to the legacy header
                         ["time", "timestamp", "p1", "p2", "p3", "p4"].
        """
        if self._active:
            return None
        if header is None:
            header = ["time", "timestamp", "p1", "p2", "p3", "p4"]
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        name = f"{filename_prefix}_{stamp}.csv"
        path = os.path.join(self.output_dir, name)
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(header)
        self._start_time = time.time()
        self._active = True
        self._current_path = path
        return name

    def stop(self):
        """Stop logging. Returns the full path of the saved file, or None."""
        if not self._active:
            return None
        path = self._current_path
        self._file.close()
        self._file = None
        self._writer = None
        self._active = False
        return path

    def log(self, vals):
        """Write a data row with elapsed time and timestamp prepended."""
        if not self._active:
            return
        t = time.time() - self._start_time
        ts = datetime.now().strftime("%H:%M:%S.%f")
        self._writer.writerow([t, ts] + list(vals))
        self._file.flush()
