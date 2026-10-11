"""Minimal SigMF writer (cf32_le)."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def write_sigmf(base: Path, iq: np.ndarray, sample_rate: float, center_hz: float) -> Path:
    base = Path(base)
    base.parent.mkdir(parents=True, exist_ok=True)
    np.asarray(iq, dtype="<c8").tofile(base.with_suffix(".sigmf-data"))
    meta = base.with_suffix(".sigmf-meta")
    meta.write_text(json.dumps({
        "global": {"core:datatype": "cf32_le", "core:sample_rate": sample_rate, "core:version": "1.0.0",
                   "core:description": "NeoRadio digital twin capture"},
        "captures": [{"core:sample_start": 0, "core:frequency": center_hz}],
        "annotations": [],
    }, indent=2))
    return meta
