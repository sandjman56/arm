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


def fit_module_curve(samples: list[tuple[float, float]]) -> tuple[float, float, float]:
    """Fit length = a + b*psi + c*psi^2 by least squares.

    samples: list of (psi, length_mm) pairs. At least 3 required.
    Returns (a, b, c).
    """
    if len(samples) < 3:
        raise ValueError(f"need >= 3 samples for quadratic fit, got {len(samples)}")
    # Build the normal equations by hand to avoid depending on numpy here.
    # For y = a + b*x + c*x^2, minimizing sum (y_i - a - b*x_i - c*x_i^2)^2
    # yields a 3x3 linear system.
    n = len(samples)
    Sx = sum(x for x, _ in samples)
    Sx2 = sum(x * x for x, _ in samples)
    Sx3 = sum(x ** 3 for x, _ in samples)
    Sx4 = sum(x ** 4 for x, _ in samples)
    Sy = sum(y for _, y in samples)
    Sxy = sum(x * y for x, y in samples)
    Sx2y = sum(x * x * y for x, y in samples)

    # System:
    # [ n    Sx    Sx2  ] [a]   [Sy  ]
    # [ Sx   Sx2   Sx3  ] [b] = [Sxy ]
    # [ Sx2  Sx3   Sx4  ] [c]   [Sx2y]
    A = [
        [n, Sx, Sx2],
        [Sx, Sx2, Sx3],
        [Sx2, Sx3, Sx4],
    ]
    rhs = [Sy, Sxy, Sx2y]
    a, b, c = _solve_3x3(A, rhs)
    return (a, b, c)


def _solve_3x3(A: list[list[float]], rhs: list[float]) -> tuple[float, float, float]:
    """Solve a 3x3 linear system by Gaussian elimination."""
    # Copy to avoid mutating caller.
    M = [row[:] + [r] for row, r in zip(A, rhs)]
    n = 3
    for i in range(n):
        # Pivot.
        max_row = max(range(i, n), key=lambda r: abs(M[r][i]))
        if abs(M[max_row][i]) < 1e-12:
            raise ValueError("singular matrix in _solve_3x3")
        M[i], M[max_row] = M[max_row], M[i]
        # Eliminate below.
        for j in range(i + 1, n):
            f = M[j][i] / M[i][i]
            for k in range(i, n + 1):
                M[j][k] -= f * M[i][k]
    # Back-substitute.
    x = [0.0] * n
    for i in range(n - 1, -1, -1):
        x[i] = (M[i][n] - sum(M[i][k] * x[k] for k in range(i + 1, n))) / M[i][i]
    return (x[0], x[1], x[2])
