"""Two populations of one PCB (Rev A spec §0, §2): China (primary, the default) and Iran (fallback).

A part without `variants` is fitted in both with its own model. `set_variants` gives a part a
per-population value, generic identification (spec §2b), model and tolerance; a `None` variant
means "not fitted" in that population. `only` fits a part in one population."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

POPULATIONS = ("china", "iran")
DEFAULT = "china"


@dataclass(frozen=True)
class Variant:
    value: str
    ident: str                                  # manufacturer part number or full specification
    note: str = ""
    spice_lines: Callable | None = None        # None: the part's own model
    tol: float | None = None                    # None: the part's own tolerance
    torob: str | None = None                    # Iranian availability search key (never in a BOM)
    torob_match: Callable | None = None


def _check(pop: str) -> None:
    if pop not in POPULATIONS:
        raise ValueError(f"population must be one of {POPULATIONS}, got {pop!r}")


def set_variants(part, china: Variant | None, iran: Variant | None):
    part.variants = {"china": china, "iran": iran}
    return part


def only(part, pop: str):
    _check(pop)
    own = Variant(part.value, getattr(part, "mpn", None) or getattr(part, "spec", "") or part.value,
                  getattr(part, "bom_note", ""), None, None, getattr(part, "torob", None),
                  getattr(part, "torob_match", None))
    return set_variants(part, *(own if p == pop else None for p in POPULATIONS))


def variant(part, pop: str) -> Variant | None:
    _check(pop)
    v = getattr(part, "variants", None)
    return None if v is None else v[pop]


def fitted(part, pop: str) -> bool:
    _check(pop)
    v = getattr(part, "variants", None)
    return v is None or v[pop] is not None


def tol(part, pop: str) -> float:
    v = variant(part, pop)
    return v.tol if v is not None and v.tol is not None else getattr(part, "tol", 0.0)
