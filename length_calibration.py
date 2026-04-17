"""Per-module length-vs-pressure calibration I/O.

Calibration data lives in `length_calibration.json` at the project root.
Each module has a quadratic fit: length_mm(psi) = a + b*psi + c*psi^2.
"""
from __future__ import annotations
import json
import os
from dataclasses import dataclass, field
from datetime import datetime, timezone
from typing import Dict, List, Optional


DEFAULT_WORKSPACE = {
    "L_rest_mm": 240.0,   # total rest length assuming 6 modules of 40mm
    "L_max_mm": 480.0,    # total max length assuming 6 modules of 80mm
    "theta_max_rad": 1.0472,  # 60 degrees
}


@dataclass
class LengthCalibration:
    modules: Dict[int, dict] = field(default_factory=dict)
    missing_modules: List[int] = field(default_factory=list)
    is_default: bool = False
    calibrated_at: Optional[str] = None

    @property
    def L_rest(self) -> float:
        """Sum of per-module rest lengths, or default if no calibration."""
        if self.is_default or not self.modules:
            return DEFAULT_WORKSPACE["L_rest_mm"]
        return sum(m["rest_length_mm"] for m in self.modules.values())

    @property
    def L_max(self) -> float:
        """Sum of per-module max lengths, or default if no calibration."""
        if self.is_default or not self.modules:
            return DEFAULT_WORKSPACE["L_max_mm"]
        total = 0.0
        for m in self.modules.values():
            a, b, c = m["coeffs"]
            psi = m["max_psi"]
            total += a + b * psi + c * psi * psi
        return total

    def length_at_pressure(self, module_id: int, psi: float) -> float:
        """Return module length in mm at the given pressure (psi)."""
        m = self.modules.get(module_id)
        if m is None:
            return DEFAULT_WORKSPACE["L_rest_mm"] / max(1, len(self.modules) or 6)
        a, b, c = m["coeffs"]
        return a + b * psi + c * psi * psi

    def save(self, path: str) -> None:
        payload = {
            "calibrated_at": self.calibrated_at or datetime.now(timezone.utc).isoformat(),
            # JSON object keys must be strings.
            "modules": {str(k): v for k, v in self.modules.items()},
            "missing_modules": self.missing_modules,
        }
        with open(path, "w") as f:
            json.dump(payload, f, indent=2)


def load_or_default(path: str) -> LengthCalibration:
    """Load calibration from disk, or return a default if the file is missing."""
    if not os.path.exists(path):
        return LengthCalibration(is_default=True)
    try:
        with open(path, "r") as f:
            data = json.load(f)
        return LengthCalibration(
            modules={int(k): v for k, v in data.get("modules", {}).items()},
            missing_modules=list(data.get("missing_modules", [])),
            is_default=False,
            calibrated_at=data.get("calibrated_at"),
        )
    except (json.JSONDecodeError, KeyError, ValueError) as e:
        print(f"[WARN] Could not parse calibration at {path}: {e}; using default.")
        return LengthCalibration(is_default=True)
