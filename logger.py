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

    def start(self):
        """Start logging. Returns the filename."""
        if self._active:
            return None
        name = datetime.now().strftime("log_%Y%m%d_%H%M%S.csv")
        path = os.path.join(self.output_dir, name)
        self._file = open(path, "w", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(["time", "timestamp", "p1", "p2", "p3", "p4"])
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
