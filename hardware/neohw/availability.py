"""Iranian-market availability from the Torob survey."""
from __future__ import annotations

import json
import re
from pathlib import Path

from neohw.tools import REPO_ROOT

SPECS = REPO_ROOT / "docs" / "superpowers" / "specs"
DEFAULT_PATH = SPECS / "2026-10-12-parts-availability-torob.json"
SURVEYS = (DEFAULT_PATH, SPECS / "2026-10-12-parts-survey-phase3-torob.json")


def load(path: Path | None = None) -> dict:
    """All Torob surveys merged (later surveys win for a repeated key), or one file."""
    if path is not None:
        return json.loads(Path(path).read_text(encoding="utf-8"))
    data: dict = {}
    for p in SURVEYS:
        data.update(json.loads(p.read_text(encoding="utf-8")))
    return data


def _norm(text: str) -> str:
    return re.sub(r"[\s\-_./]", "", text.lower())


def _matches(key: str, hit_name: str) -> bool:
    """Torob search is fuzzy: a hit counts only if it names every part-number token of the key.

    Non-ASCII words in the key (e.g. Persian category words) are not required.
    """
    tokens = [t for t in key.split() if t.isascii()] or [key]
    name = _norm(hit_name)
    return all(_norm(t) in name for t in tokens)


_HENRY = re.compile(r"(\d+(?:\.\d+)?)\s*(nh|uh|µh|μh|mh|نانو\s*هانری|(?:میکرو|ميکرو)\s*هانری|میلی\s*هانری)",
                    re.IGNORECASE)
_HENRY_SCALE = {"n": 1e-9, "u": 1e-6, "µ": 1e-6, "μ": 1e-6, "m": 1e-3, "ن": 1e-9, "م": 1e-6}


def inductor_values(name: str) -> list[float]:
    """Inductances named in a shop title: '470 نانو هانری', '4.7µH', '10UH', '270NH', '22MH'."""
    out = []
    for number, unit in _HENRY.findall(name):
        unit = unit.lower()
        scale = 1e-3 if unit.startswith("میلی") else _HENRY_SCALE[unit[0]]
        out.append(float(number) * scale)
    return out


def inductor_hit(name: str, henries: float) -> bool:
    """A hit is this inductor if it names a value within 1 %."""
    return any(abs(v / henries - 1) <= 0.01 for v in inductor_values(name))


def lookup(key: str | None, data: dict, match=None) -> str:
    """`match(hit_name) -> bool` replaces the default part-number token match."""
    if not key or key not in data:
        return "unknown"
    match = match or (lambda name: _matches(key, name))
    hits = [h for h in data[key].get("hits") or [] if match(h.get("name") or "")]
    if not hits:
        return "unknown"
    return "available" if any(h.get("instock") for h in hits) else "out_of_stock"
