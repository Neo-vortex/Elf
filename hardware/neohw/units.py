"""SI value parsing and formatting."""
from __future__ import annotations

import math
import re

_PREFIX = {"f": 1e-15, "p": 1e-12, "n": 1e-9, "u": 1e-6, "µ": 1e-6, "m": 1e-3,
           "k": 1e3, "M": 1e6, "G": 1e9}
_RE = re.compile(r"^\s*([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?)\s*([fpnuµmkMG]?)\s*$")


def parse_si(text: str | float) -> float:
    if isinstance(text, (int, float)):
        return float(text)
    m = _RE.match(text)
    if not m:
        raise ValueError(f"not an SI value: {text!r}")
    return float(m.group(1)) * _PREFIX.get(m.group(2), 1.0)


def format_si(value: float, unit: str = "") -> str:
    if value == 0:
        return f"0{unit}"
    exp = int(math.floor(math.log10(abs(value)) / 3) * 3)
    exp = max(-15, min(9, exp))
    prefix = {-15: "f", -12: "p", -9: "n", -6: "u", -3: "m", 0: "", 3: "k", 6: "M", 9: "G"}[exp]
    mantissa = value / 10 ** exp
    return f"{mantissa:.4g}{prefix}{unit}"
