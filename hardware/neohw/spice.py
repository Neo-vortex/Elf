"""Turn a SKiDL circuit into an ngspice deck and simulate it."""
from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from neohw.ngspice import run_deck
from neohw.rawfile import Plot


class SpiceError(ValueError):
    """The circuit cannot be turned into a valid deck."""


@dataclass(frozen=True)
class Port:
    net: object
    number: int
    z0: float = 50.0


@dataclass(frozen=True)
class Ac:
    start_hz: float
    stop_hz: float
    points_per_decade: int = 20

    def command(self) -> str:
        return f"ac dec {self.points_per_decade} {self.start_hz:g} {self.stop_hz:g}"


@dataclass(frozen=True)
class Sp:
    start_hz: float
    stop_hz: float
    points: int = 201
    sweep: str = "lin"

    def command(self) -> str:
        return f"sp {self.sweep} {self.points} {self.start_hz:g} {self.stop_hz:g}"


@dataclass(frozen=True)
class Op:
    def command(self) -> str:
        return "op"


def node_name(net) -> str:
    if net.name == "GND":
        return "0"
    return re.sub(r"[^a-z0-9_]", "_", net.name.lower())


def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=()) -> tuple[str, list[str]]:
    lines = [title]
    for part in circuit.parts:
        if not hasattr(part, "spice_lines"):
            raise SpiceError(f"{part.ref} has no SPICE model")
        nodes = {}
        for pin in part.pins:
            if pin.net is None or not pin.is_connected():
                if pin.name.upper().startswith("COIL"):
                    continue
                raise SpiceError(f"{part.ref} pin {pin.name} is unconnected")
            nodes[pin.name] = node_name(pin.net)
        lines.extend(part.spice_lines(nodes))
    for port in ports:
        drive = 1 if port.number == 1 else 0
        lines.append(f"VPORT{port.number} {node_name(port.net)} 0 dc 0 ac {drive} "
                     f"portnum {port.number} z0 {port.z0:g}")
    lines.extend(extra_lines)
    raws = []
    lines += [".control", "set filetype=ascii"]
    for k, analysis in enumerate(analyses):
        raw = f"a{k}.raw"
        lines += [analysis.command(), f"write {raw} all"]
        raws.append(raw)
    lines += [".endc", ".end", ""]
    return "\n".join(lines), raws


def simulate(circuit, title: str, ports: list[Port], analyses: list, keep_dir: Path | None = None) -> list[Plot]:
    deck, raws = to_deck(circuit, title, ports, analyses)
    results = run_deck(deck, raws, keep_dir=keep_dir)
    return [results[r][0] for r in raws]
