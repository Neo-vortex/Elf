"""Iranian-market availability from the Torob survey."""
from __future__ import annotations

import json
from pathlib import Path

from neohw.tools import REPO_ROOT

DEFAULT_PATH = REPO_ROOT / "docs" / "superpowers" / "specs" / "2026-10-12-parts-availability-torob.json"


def load(path: Path = DEFAULT_PATH) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def lookup(key: str | None, data: dict) -> str:
    if not key or key not in data:
        return "unknown"
    hits = data[key].get("hits") or []
    if not hits:
        return "unknown"
    return "available" if any(h.get("instock") for h in hits) else "out_of_stock"
