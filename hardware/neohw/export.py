"""KiCad netlist, BOM and diagram exports for a SKiDL circuit."""
from __future__ import annotations

import contextlib
import csv
import html
import re
import subprocess
import tempfile
from pathlib import Path

from skidl import KICAD9

from neohw.availability import lookup
from neohw.tools import tool_path


def kicad_netlist(circuit, path: Path) -> Path:
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix="neohw-skidl-") as tmp, contextlib.chdir(tmp):
        circuit.generate_netlist(tool=KICAD9, file_=str(path))
    return path


def _natural(refs: str) -> list:
    """Sort key so R2 comes before R10."""
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", refs.split()[0])]


def bom_rows(circuit, data: dict) -> list[dict]:
    groups: dict[tuple, dict] = {}
    for part in circuit.parts:
        mpn = getattr(part, "mpn", None) or ""
        key = (part.value, part.footprint, mpn)
        availability = lookup(getattr(part, "torob", None), data, getattr(part, "torob_match", None))
        row = groups.setdefault(key, {"refs": [], "value": part.value, "footprint": part.footprint,
                                      "mpn": mpn, "qty": 0, "availability": availability,
                                      "note": getattr(part, "bom_note", "")})
        row["refs"].append(part.ref)
        row["qty"] += 1
    rows = list(groups.values())
    for row in rows:
        row["refs"] = " ".join(sorted(row["refs"]))
    return sorted(rows, key=lambda r: _natural(r["refs"]))


def write_bom(circuit, path: Path, data: dict) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    rows = bom_rows(circuit, data)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["refs", "value", "footprint", "mpn", "qty", "availability", "note"])
        writer.writeheader()
        writer.writerows(rows)
    return path


def svg_diagram(circuit, path: Path) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    out = ["graph G {", "  rankdir=LR; node [fontname=Helvetica fontsize=10];"]
    for part in circuit.parts:
        label = html.escape(f"{part.ref}\n{part.value}")
        out.append(f'  "{part.ref}" [shape=box label="{label}"];')
    for net in circuit.get_nets():
        out.append(f'  "net:{net.name}" [shape=point xlabel="{html.escape(net.name)}"];')
    for part in circuit.parts:
        for pin in part.pins:
            if pin.net is not None:
                out.append(f'  "{part.ref}" -- "net:{pin.net.name}" [taillabel="{html.escape(pin.name)}" fontsize=8];')
    out.append("}")
    dot = tool_path("dot")
    proc = subprocess.run([str(dot), "-Tsvg", "-o", str(path)], input="\n".join(out), text=True,
                          capture_output=True)
    if proc.returncode != 0:
        raise RuntimeError(f"dot failed: {proc.stderr.strip()}")
    return path
