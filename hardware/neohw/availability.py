"""Iranian-market availability from the Torob survey."""
from __future__ import annotations

import json
import re
from pathlib import Path

from neohw.tools import REPO_ROOT

DEFAULT_PATH = REPO_ROOT / "docs" / "superpowers" / "specs" / "2026-10-12-parts-availability-torob.json"


def load(path: Path = DEFAULT_PATH) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def _norm(text: str) -> str:
    return re.sub(r"[\s\-_./]", "", text.lower())


def _matches(key: str, hit_name: str) -> bool:
    """Torob search is fuzzy: a hit counts only if it names every part-number token of the key.

    Non-ASCII words in the key (e.g. Persian category words) are not required.
    """
    tokens = [t for t in key.split() if t.isascii()] or [key]
    name = _norm(hit_name)
    return all(_norm(t) in name for t in tokens)


def lookup(key: str | None, data: dict) -> str:
    if not key or key not in data:
        return "unknown"
    hits = [h for h in data[key].get("hits") or [] if _matches(key, h.get("name") or "")]
    if not hits:
        return "unknown"
    return "available" if any(h.get("instock") for h in hits) else "out_of_stock"
