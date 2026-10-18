import csv
import os
import re

import pytest
from skidl import Circuit, Net

from neohw import export, parts
from neohw.tools import ToolNotFoundError, tool_path


def small():
    ckt = Circuit()
    with ckt:
        a, gnd = Net("a"), Net("GND")
        for _ in range(2):
            r = parts.resistor("61.9", torob="G6K-2F")
            r[1] += a
            r[2] += gnd
        c = parts.capacitor("10n")
        c[1] += a
        c[2] += gnd
    return ckt


def test_bom_groups_identical_parts():
    rows = export.bom_rows(small(), {"G6K-2F": {"ok": True, "hits": [{"name": "G6K-2F", "instock": True}]}}, "iran")
    by_value = {r["value"]: r for r in rows}
    assert by_value["61.9Ω"]["qty"] == 2
    assert by_value["61.9Ω"]["availability"] == "available"
    assert by_value["10nF"]["availability"] == "unknown"
    assert "availability" not in export.bom_rows(small(), {})[0]


def test_write_bom_csv(tmp_path):
    path = export.write_bom(small(), tmp_path / "bom.csv", {})
    rows = list(csv.DictReader(path.open()))
    assert {r["value"] for r in rows} == {"61.9Ω", "10nF"}


def test_kicad_netlist_leaves_no_clutter(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    out = export.kicad_netlist(small(), tmp_path / "out" / "block.net")
    text = out.read_text()
    assert "(export" in text and "Resistor_SMD:R_0805_2012Metric" in text
    assert sorted(os.listdir(tmp_path)) == ["out"]


@pytest.mark.integration
def test_svg_diagram(tmp_path):
    try:
        tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    svg = export.svg_diagram(small(), tmp_path / "d.svg").read_text()
    assert svg.lstrip().startswith("<?xml") and "61.9Ω" in svg


def test_kicad_netlist_tags_are_the_refs(tmp_path):
    # stable tags: no "Missing tag" warnings and a reproducible netlist
    text = export.kicad_netlist(small(), tmp_path / "block.net").read_text()
    comps = re.findall(r'\(ref "([^"]+)"\).*?\(name "SKiDL Tag"\) "([^"]+)"', text, re.S)
    assert len(comps) == 3 and all(ref == tag for ref, tag in comps)


def test_bom_is_per_population_and_unfitted_parts_are_left_out():
    from neohw import population as pop
    ckt = Circuit()
    with ckt:
        a, b = Net("a"), Net("b")
        r1, r2 = parts.resistor("1k"), parts.resistor("2k")
        for r in (r1, r2):
            r[1] += a
            r[2] += b
        pop.only(r2, "iran")
    china = export.bom_rows(ckt, {}, "china")
    iran = export.bom_rows(ckt, {}, "iran")
    assert [r["value"] for r in china] == ["1kΩ"] and {r["value"] for r in iran} == {"1kΩ", "2kΩ"}
    assert {"refs", "qty", "value", "package", "ident", "note"} <= set(china[0])
    assert [r["china"] for r in export.fitted_rows(ckt)] == [r1.spec, "—"]
