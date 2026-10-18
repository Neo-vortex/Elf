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

from neohw import ident, population as pop
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


def _row_fields(part, population: str) -> tuple[str, str, str, str | None, object]:
    """(value, ident, note, torob key, torob matcher) of a fitted part in a population."""
    v = pop.variant(part, population)
    if v is None:
        return (part.value, ident.of(part, population), getattr(part, "bom_note", ""),
                getattr(part, "torob", None), getattr(part, "torob_match", None))
    return v.value, ident.of(part, population), v.note, v.torob, v.torob_match


def bom_rows(circuit, data: dict, population: str = pop.DEFAULT) -> list[dict]:
    """One row per (value, package, ident) of the parts fitted in `population`. The Iranian
    population also gets the Torob availability of each row."""
    groups: dict[tuple, dict] = {}
    for part in circuit.parts:
        if not pop.fitted(part, population):
            continue
        value, ident_, note, torob, match = _row_fields(part, population)
        package = getattr(part, "package", None) or part.footprint.split(":")[-1]
        row = groups.setdefault((value, package, ident_), {
            "refs": [], "qty": 0, "value": value, "package": package, "ident": ident_, "notes": []})
        if population == "iran" and "availability" not in row:
            row["availability"] = lookup(torob, data, match)
        row["refs"].append(part.ref)
        row["qty"] += 1
        if note and note not in row["notes"]:
            row["notes"].append(note)
    rows = list(groups.values())
    for row in rows:
        row["refs"] = " ".join(sorted(row["refs"]))
        row["note"] = "; ".join(row.pop("notes"))
    return sorted(rows, key=lambda r: _natural(r["refs"]))


def fitted_rows(circuit) -> list[dict]:
    """Per ref, what is fitted in each population (`—` where nothing is)."""
    rows = []
    for part in sorted(circuit.parts, key=lambda p: _natural(p.ref)):
        rows.append({"ref": part.ref, **{
            population: ident.of(part, population) if pop.fitted(part, population) else "—"
            for population in pop.POPULATIONS}})
    return rows


def _write_csv(path: Path, fields: list[str], rows: list[dict]) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_bom(circuit, path: Path, data: dict, population: str = pop.DEFAULT) -> Path:
    fields = ["refs", "qty", "value", "package", "ident", "note"] + (["availability"] if population == "iran" else [])
    return _write_csv(path, fields, bom_rows(circuit, data, population))


def write_fitted(circuit, path: Path) -> Path:
    return _write_csv(path, ["ref", *pop.POPULATIONS], fitted_rows(circuit))


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
