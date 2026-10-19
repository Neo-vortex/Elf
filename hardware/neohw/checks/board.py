"""Board check (phase 3 spec §1 item 2): the whole receiver as one netlist. No net with a single
pin, no required pin left open, every IC supply pin on its rail and every ground pin on GND, every
footprint in the KiCad 9 libraries or the project library. Writes the board KiCad netlist and BOM,
and board.json."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, charfile, export, ident
from neohw.blocks import board
from neohw.checks import BlockResult, CheckResult
from neohw.footprints import FOOTPRINT_LIB
from neohw.spice import _canonical

KICAD_FOOTPRINTS = Path("/usr/share/kicad/footprints")
# NeoRadio: footprints not drawn yet (layout phase); every other NeoRadio: footprint must be in the
# project library
LAYOUT_PENDING = ("NeoRadio:GDT_SMD_2pin",)
# (part value, pin) -> rail the pin must be on
SUPPLY_RAILS = {
    ("Si5351C", "VDD"): "+3V3_RF", ("Si5351C", "VDDOA"): "+3V3_RF", ("Si5351C", "VDDOB"): "+3V3_RF",
    ("Si5351C", "VDDOC"): "+3V3_RF", ("Si5351C", "VDDOD"): "+3V3_RF", ("74LVC74", "VCC"): "+3V3_RF",
    ("74LVC157", "VCC"): "+3V3_RF", ("LM75A", "VCC"): "+3V3_RF", ("FST3253", "VCC"): "+5V_A",
    ("LM4562", "V+"): "+5V_A", ("CS5361", "VA"): "+5V_A", ("CS5361", "VD"): "+3V3_D", ("CS5361", "VL"): "+3V3_D",
    ("ESP32-S3-WROOM-1", "3V3"): "+3V3_D", ("74HC595", "VCC"): "+3V3_D", ("ULN2803", "COM"): "+5V_MAIN",
    ("G6K-2F", "COIL+"): "+5V_MAIN", ("TPS7A20-5.0", "OUT"): "+5V_A", ("TPS7A20-3.3", "OUT"): "+3V3_RF",
    ("TLV75733", "OUT"): "+3V3_D", ("AMS1117-3.3", "OUT"): "+3V3_D", ("USBLC6-2SC6", "VBUS"): "VBUS",
}
GROUND_PINS = ("GND", "GND2", "EPAD", "PAD", "V-", "REFGND")


def _net(pin):
    return _canonical(pin.net).name if pin.net is not None else None


def _footprint_exists(footprint: str) -> bool:
    lib, name = footprint.split(":")
    if lib == "NeoRadio":
        return footprint in LAYOUT_PENDING or (FOOTPRINT_LIB / f"{name}.kicad_mod").exists()
    return (KICAD_FOOTPRINTS / f"{lib}.pretty" / f"{name}.kicad_mod").exists()


def findings(ckt) -> dict:
    pins_on: dict[int, list] = {}
    for p in ckt.parts:
        for pin in p.pins:
            if pin.net is not None and pin.is_connected():
                n = _canonical(pin.net)
                pins_on.setdefault(id(n), [n, 0])[1] += 1
    single = sorted(n.name for n, c in pins_on.values() if c < 2)
    open_pins = sorted(f"{p.ref}.{pin.name}" for p in ckt.parts for pin in p.pins
                       if (pin.net is None or not pin.is_connected()) and pin.name not in getattr(p, "nc_pins", ()))
    rails = []
    for p in ckt.parts:
        for pin in p.pins:
            want = SUPPLY_RAILS.get((p.value, pin.name))
            if want and _net(pin) != want:
                rails.append(f"{p.ref} {p.value}.{pin.name} on {_net(pin)}, expected {want}")
            if (pin.name in GROUND_PINS and p.value not in ("LM4562",) and _net(pin) != "GND"
                    and pin.name not in getattr(p, "nc_pins", ())):
                rails.append(f"{p.ref} {p.value}.{pin.name} on {_net(pin)}, expected GND")
            if p.value == "LM4562" and pin.name == "V-" and _net(pin) != "GND":
                rails.append(f"{p.ref} LM4562.V- on {_net(pin)}, expected GND (single supply)")
    missing = sorted({p.footprint for p in ckt.parts if not _footprint_exists(p.footprint)})
    return {"single_pin_nets": single, "open_pins": open_pins, "rail_errors": rails, "missing_footprints": missing}


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    b = board.build()
    f = findings(b.circuit)
    data = availability.load()
    rows = export.bom_rows(b.circuit, data)
    iran_rows = export.bom_rows(b.circuit, data, "iran")
    bad_ident = ident.violations(b.circuit)
    semis = [r for r in iran_rows if r["refs"].split()[0].rstrip("0123456789") in ("U", "Q", "D", "K", "X", "GDT")]
    unconfirmed = sorted(r["value"] for r in semis if r["availability"] != "available")
    checks = [
        CheckResult("parts", len(b.circuit.parts), "report", True, ""),
        CheckResult("single_pin_nets", len(f["single_pin_nets"]), "= 0", not f["single_pin_nets"], ""),
        CheckResult("open_required_pins", len(f["open_pins"]), "= 0", not f["open_pins"], ""),
        CheckResult("supply_rail_errors", len(f["rail_errors"]), "= 0", not f["rail_errors"], ""),
        CheckResult("missing_footprints", len(f["missing_footprints"]), "= 0", not f["missing_footprints"], ""),
        CheckResult("ident_violations", len(bad_ident), "= 0", not bad_ident, "; ".join(bad_ident[:5])),
        CheckResult("semiconductors_unconfirmed_on_torob", len(unconfirmed), "report (survey pending)", True, ""),
    ]
    result = BlockResult("board", checks)
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(b.circuit, out_dir / "board.net"),
        "bom": export.write_bom(b.circuit, out_dir / "board_bom_china.csv", data, "china"),
        "bom_iran": export.write_bom(b.circuit, out_dir / "board_bom_iran.csv", data, "iran"),
        "fitted": export.write_fitted(b.circuit, out_dir / "board_fitted.csv"),
        "characterisation": charfile.write(char_dir / "board.json", "board", "pass" if result.passed else "fail", {
            "parts_by_block": {k: len(v) for k, v in b.ports["parts_by_block"].items()},
            "bom_lines": len(rows), "bom_lines_iran": len(iran_rows), "ident_violations": bad_ident, "unconfirmed_semiconductors": unconfirmed, **f,
            "checks": [c.__dict__ for c in checks]}),
    }
    return result
