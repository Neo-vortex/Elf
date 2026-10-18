"""Generic part identification (Rev A spec §2b): a manufacturer part number for active parts,
relays, crystals, oscillators, connectors and protection parts, a full specification for
passives. No distributor catalogue number and no Torob key may reach a BOM."""
from __future__ import annotations

import re

from neohw import population as pop

CATALOGUE = re.compile(r"\bC\d{4,}\b|LCSC|JLC|torob", re.I)
PASSIVES = ("R", "C", "L")


def of(part, population: str = pop.DEFAULT) -> str:
    """The variant's identification, else the part's MPN, else its specification."""
    v = pop.variant(part, population)
    if v is not None and v.ident:
        return v.ident
    return getattr(part, "mpn", None) or getattr(part, "spec", None) or ""


def violations(circuit) -> list[str]:
    """Refs whose identification (in any population) or note is empty, names a catalogue, or whose
    passive specification lacks a tolerance or the package."""
    out = []
    for part in circuit.parts:
        for population in pop.POPULATIONS:
            if not pop.fitted(part, population):
                continue
            ident = of(part, population)
            v = pop.variant(part, population)
            note = v.note if v is not None else getattr(part, "bom_note", "")
            if not ident:
                out.append(f"{part.ref} ({population}): no identification")
            elif CATALOGUE.search(ident) or CATALOGUE.search(note or ""):
                out.append(f"{part.ref} ({population}): catalogue number in {ident!r} / {note!r}")
        spec = getattr(part, "spec", "")
        if part.ref.rstrip("0123456789") in PASSIVES and ("±" not in spec or getattr(part, "package", "") not in spec):
            out.append(f"{part.ref}: specification lacks tolerance or package: {spec!r}")
    return out
