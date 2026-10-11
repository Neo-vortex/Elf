"""Touchstone 1.1 two-port files (S-parameters, real/imaginary, 50 Ω)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

ORDER = ("11", "21", "12", "22")  # Touchstone 1.x two-port column order


def write_s2p(path: Path, freq_hz: np.ndarray, s: dict[str, np.ndarray], z0: float = 50.0,
              comments: tuple[str, ...] = ()) -> Path:
    missing = [k for k in ORDER if k not in s]
    if missing:
        raise ValueError(f"missing S-parameters {missing}")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"! {c}" for c in comments] + [f"# HZ S RI R {z0:g}"]
    for i, f in enumerate(freq_hz):
        values = " ".join(f"{s[k][i].real:.9e} {s[k][i].imag:.9e}" for k in ORDER)
        lines.append(f"{f:.9e} {values}")
    path.write_text("\n".join(lines) + "\n")
    return path


def read_s2p(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("!"):
            continue
        if line.startswith("#"):
            if line.upper().split() != ["#", "HZ", "S", "RI", "R", "50"]:
                raise ValueError(f"{path}: unsupported option line {line!r}")
            continue
        rows.append([float(x) for x in line.split()])
    data = np.array(rows)
    if data.ndim != 2 or data.shape[1] != 9:
        raise ValueError(f"{path}: expected 9 columns per row")
    s = {k: data[:, 1 + 2 * i] + 1j * data[:, 2 + 2 * i] for i, k in enumerate(ORDER)}
    return data[:, 0], s
