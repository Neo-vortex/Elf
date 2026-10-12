"""Turn a SKiDL circuit into an ngspice deck and simulate it."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

from neohw.ngspice import run_deck
from neohw.rawfile import Plot


class SpiceError(ValueError):
    """The circuit cannot be turned into a valid deck."""


@dataclass(frozen=True)
class Port:
    """S-parameter port (ngspice `portnum`); its internal z0 is noiseless, so use Source/Load
    for noise."""
    net: object
    number: int
    z0: float = 50.0

    def lines(self) -> list[str]:
        drive = 1 if self.number == 1 else 0
        return [f"VPORT{self.number} {node_name(self.net)} 0 dc 0 ac {drive} "
                f"portnum {self.number} z0 {self.z0:g}"]


@dataclass(frozen=True)
class Source:
    """Signal source with a (noisy) series resistor. `tones` are (frequency Hz, available
    power dBm) sine generators for transient analyses; the first stage also carries `ac 1`."""
    net: object
    name: str = "in"
    z0: float = 50.0
    tones: tuple[tuple[float, float], ...] = ()

    @property
    def vname(self) -> str:
        return f"VS_{self.name}0"

    def lines(self) -> list[str]:
        out, prev = [], "0"
        for k, tone in enumerate(self.tones or ((None, None),)):
            nxt = f"s_{self.name}{k}"
            sine = ""
            if tone[0] is not None:
                amp = 2 * math.sqrt(2 * self.z0 * 1e-3 * 10 ** (tone[1] / 10))   # open-circuit peak
                sine = f" sin(0 {amp:.9g} {tone[0]:.9g})"
            out.append(f"VS_{self.name}{k} {nxt} {prev} dc 0{' ac 1' if k == 0 else ''}{sine}")
            prev = nxt
        out.append(f"RS_{self.name} {prev} {node_name(self.net)} {self.z0:g}")
        return out


@dataclass(frozen=True)
class Load:
    net: object
    name: str = "out"
    r: float = 50.0

    def lines(self) -> list[str]:
        return [f"RL_{self.name} {node_name(self.net)} 0 {self.r:g}"]


@dataclass(frozen=True)
class Supply:
    net: object
    volts: float
    name: str = "vcc"

    @property
    def vname(self) -> str:
        return f"VSUP_{self.name}"

    def lines(self) -> list[str]:
        return [f"{self.vname} {node_name(self.net)} 0 dc {self.volts:g}"]


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


@dataclass(frozen=True)
class Noise:
    """Noise analysis referred to `source` (a Source); results in the noise1 plot."""
    output_net: object
    source: Source
    start_hz: float
    stop_hz: float
    points_per_decade: int = 20

    def command(self) -> str:
        return (f"noise v({node_name(self.output_net)}) {self.source.vname} dec {self.points_per_decade} "
                f"{self.start_hz:g} {self.stop_hz:g}\nsetplot noise1")


@dataclass(frozen=True)
class Tran:
    """Transient on a uniform grid (`linearize`), saved from `start_s`."""
    step_s: float
    stop_s: float
    start_s: float = 0.0

    def command(self) -> str:
        return f"tran {self.step_s:g} {self.stop_s:g} {self.start_s:g} {self.step_s:g}\nlinearize"


def _canonical(net):
    """One net for a merged group (`a += b` leaves pins on both objects): GND if present,
    otherwise the alphabetically first name."""
    group = list(getattr(net, "nets", None) or [net])
    grounds = [n for n in group if n.name.upper() == "GND"]
    return grounds[0] if grounds else min(group, key=lambda n: n.name)


def node_name(net) -> str:
    net = _canonical(net)
    if net.name.upper() == "GND":
        return "0"
    return re.sub(r"[^a-z0-9_]", "_", net.name.lower())


def _check_nets(circuit, ports: list[Port]) -> None:
    """Reject what KiCad and SPICE would read differently, or a part cut out of the circuit."""
    port_nets = {id(_canonical(p.net)) for p in ports}
    pins_on: dict[int, int] = {}
    nets: dict[int, object] = {}
    for part in circuit.parts:
        for pin in part.pins:
            if pin.net is not None and not pin.name.upper().startswith("COIL"):
                net = _canonical(pin.net)
                pins_on[id(net)] = pins_on.get(id(net), 0) + 1
                nets[id(net)] = net
    by_node: dict[str, str] = {}
    for key, net in nets.items():
        node = node_name(net)
        if node in by_node and by_node[node] != net.name:
            raise SpiceError(f"nets {by_node[node]!r} and {net.name!r} map to the same SPICE node {node!r}")
        by_node[node] = net.name
        if pins_on[key] < 2 and key not in port_nets and node != "0":
            raise SpiceError(f"net {net.name!r} has a single connection (dangling part?)")


def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=(),
            variation: dict[str, float] | None = None) -> tuple[str, list[str]]:
    """`variation` maps a part ref to a value multiplier (Monte Carlo); missing refs use 1."""
    variation = variation or {}
    lines = [title]
    models: list[str] = []
    for part in circuit.parts:
        for model in getattr(part, "spice_models", ()):
            if model not in models:
                models.append(model)
    lines += models
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
        lines.extend(part.spice_lines(nodes, variation.get(part.ref, 1.0)))
    _check_nets(circuit, ports)
    for port in ports:
        lines += port.lines()
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
