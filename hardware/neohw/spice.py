"""Turn a SKiDL circuit into an ngspice deck and simulate it."""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path

from neohw import population as pops
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
class QuadratureLO:
    """Two square waves driving the mixer selects: S0 at f_hz, S1 a quarter period later (+ skew_s).
    `duty_err_s0/s1` lengthen (or, negative, shorten) each high time. The FST3253 decodes
    (S1, S0) = 00, 01, 11, 10 into four 25 % phases."""
    s0: object
    s1: object
    f_hz: float
    skew_s: float = 0.0
    edge_s: float = 1e-9
    high_v: float = 3.3
    duty_err_s0: float = 0.0
    duty_err_s1: float = 0.0

    @property
    def nets(self) -> tuple:
        return (self.s0, self.s1)

    def lines(self) -> list[str]:
        t = 1 / self.f_hz
        w = t / 2 - self.edge_s
        return [f"VLO_s0 {node_name(self.s0)} 0 dc 0 pulse(0 {self.high_v:g} 0 {self.edge_s:g} {self.edge_s:g} "
                f"{w + self.duty_err_s0:.12g} {t:.12g})",
                f"VLO_s1 {node_name(self.s1)} 0 dc 0 pulse(0 {self.high_v:g} {t / 4 + self.skew_s:.12g} "
                f"{self.edge_s:g} {self.edge_s:g} {w + self.duty_err_s1:.12g} {t:.12g})"]


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
    """Noise analysis referred to `source` (a Source); results in the noise1 plot. With
    `ref_net` the output is the differential voltage v(output_net, ref_net)."""
    output_net: object
    source: Source
    start_hz: float
    stop_hz: float
    points_per_decade: int = 20
    ref_net: object = None

    def command(self) -> str:
        out = node_name(self.output_net) + ("" if self.ref_net is None else f", {node_name(self.ref_net)}")
        return (f"noise v({out}) {self.source.vname} dec {self.points_per_decade} "
                f"{self.start_hz:g} {self.stop_hz:g}\nsetplot noise1")


@dataclass(frozen=True)
class Tran:
    """Transient saved from `start_s` on a uniform `step_s` grid (`linearize`); the solver's
    internal step is capped at `max_step_s` (default: `step_s`)."""
    step_s: float
    stop_s: float
    start_s: float = 0.0
    max_step_s: float | None = None

    def command(self) -> str:
        return (f"tran {self.step_s:g} {self.stop_s:g} {self.start_s:g} {self.max_step_s or self.step_s:g}"
                f"\nlinearize")


def _canonical(net):
    """One net for a merged group (`a += b` leaves pins on both objects): GND if present,
    otherwise the alphabetically first name."""
    group = list(getattr(net, "nets", None) or [net])
    grounds = [n for n in group if n.name.upper() == "GND"]
    return grounds[0] if grounds else min(group, key=lambda n: n.name)


def node_name(net) -> str:
    """SPICE node for a net; a string is already a node name (cascade decks)."""
    if isinstance(net, str):
        return net
    net = _canonical(net)
    if net.name.upper() == "GND":
        return "0"
    return re.sub(r"[^a-z0-9_]", "_", net.name.lower())


def _check_nets(circuit, ports: list[Port], exempt: frozenset[str] = frozenset(),
                population: str = pops.DEFAULT) -> None:
    """Reject what KiCad and SPICE would read differently, or a part cut out of the circuit.
    Only parts fitted in `population` count. Nets driven by a port, or whose node is in `exempt` (cascade interfaces), may have one pin."""
    port_nets = {id(_canonical(n)) for p in ports for n in (p.nets if hasattr(p, "nets") else (p.net,))
                 if not isinstance(n, str)}
    pins_on: dict[int, int] = {}
    nets: dict[int, object] = {}
    nc_on: dict[int, int] = {}
    for part in circuit.parts:
        if not pops.fitted(part, population):
            continue
        for pin in part.pins:
            if pin.net is not None and pin.name not in getattr(part, "nc_pins", ()):
                net = _canonical(pin.net)
                pins_on[id(net)] = pins_on.get(id(net), 0) + 1
                nets[id(net)] = net
            elif pin.net is not None and pin.is_connected():
                nc_on[id(_canonical(pin.net))] = nc_on.get(id(_canonical(pin.net)), 0) + 1
    for key in nets:       # a pin that is NC in one population and wired in another (the shared LDO pads) still ends a net
        pins_on[key] += nc_on.get(key, 0)
    by_node: dict[str, str] = {}
    for key, net in nets.items():
        node = node_name(net)
        if node in by_node and by_node[node] != net.name:
            raise SpiceError(f"nets {by_node[node]!r} and {net.name!r} map to the same SPICE node {node!r}")
        by_node[node] = net.name
        if pins_on[key] < 2 and key not in port_nets and node != "0" and node not in exempt:
            raise SpiceError(f"net {net.name!r} has a single connection (dangling part?)")


def _models(circuits) -> list[str]:
    models: list[str] = []
    for circuit in circuits:
        for part in circuit.parts:
            for model in getattr(part, "spice_models", ()):
                if model not in models:
                    models.append(model)
    return models


def _part_lines(circuit, variation: dict, overrides: dict, node=lambda n: n,
                population: str = pops.DEFAULT) -> list[str]:
    """Element lines of every part fitted in `population`; `node` maps each net's node name (cascade prefixes)."""
    lines = []
    for part in circuit.parts:
        if not pops.fitted(part, population):
            continue
        if not hasattr(part, "spice_lines"):
            raise SpiceError(f"{part.ref} has no SPICE model")
        nodes = {}
        for pin in part.pins:
            if pin.net is None or not pin.is_connected():
                if pin.name in getattr(part, "nc_pins", ()):   # may stay open: private node
                    nodes[pin.name] = f"{part.ref}_{pin.name.lower()}"
                    continue
                raise SpiceError(f"{part.ref} pin {pin.name} is unconnected")
            nodes[pin.name] = node(node_name(pin.net))
        if part.ref in overrides:
            lines.extend(overrides[part.ref](nodes))
        else:
            var = pops.variant(part, population)
            model = var.spice_lines if var is not None and var.spice_lines else part.spice_lines
            lines.extend(model(nodes, variation.get(part.ref, 1.0)))
    return lines


def _control(ports, analyses, extra_lines) -> tuple[list[str], list[str]]:
    lines = []
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
    return lines, raws


def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=(),
            variation: dict[str, float] | None = None, overrides: dict | None = None,
            population: str = pops.DEFAULT) -> tuple[str, list[str]]:
    """`variation` maps a part ref to a value multiplier (Monte Carlo); missing refs use 1.
    `overrides` maps a part ref to a function of its pin -> node map that returns the SPICE
    lines used instead of the part's own model (a linear stand-in for one analysis). `population` selects the parts fitted and their variants."""
    lines = [title, *_models([circuit]), *_part_lines(circuit, variation or {}, overrides or {}, population=population)]
    _check_nets(circuit, ports, population=population)
    control, raws = _control(ports, analyses, extra_lines)
    return "\n".join(lines + control), raws


@dataclass(frozen=True)
class Stage:
    """One block of a cascade deck. Its part refs get the suffix `_<tag>` and its nodes the
    prefix `<tag>_`, except ground and the nodes named in `joins`, which map to shared node
    names (the interfaces between blocks, supplies). Ports then use those names as strings."""
    block: object
    tag: str
    joins: dict = field(default_factory=dict)


def cascade_to_deck(stages: list[Stage], title: str, ports: list, analyses: list, extra_lines=(),
                    population: str = pops.DEFAULT) -> tuple[str, list[str]]:
    """Several blocks in one deck (a receive chain), each simulated with its own models."""
    tags = [s.tag for s in stages]
    if len(set(tags)) != len(tags):
        raise SpiceError(f"cascade stage tags must be unique, got {tags}")
    lines = [title, *_models([s.block.circuit for s in stages])]
    driven = {n for p in ports for n in (p.nets if hasattr(p, "nets") else (p.net,)) if isinstance(n, str)}
    for st in stages:
        circuit = st.block.circuit
        # a net may have one pin when it is an interface or a port drives its (mapped) node
        mapped = {node_name(n): st.joins.get(node_name(n), f"{st.tag}_{node_name(n)}") for n in circuit.get_nets()}
        _check_nets(circuit, [], exempt=frozenset(st.joins) | {n for n, m in mapped.items() if m in driven},
                    population=population)
        renamed = []
        try:
            for part in circuit.parts:
                renamed.append((part, part.ref))
                part.ref = f"{part.ref}_{st.tag}"
            node = lambda n, st=st: "0" if n == "0" else st.joins.get(n, f"{st.tag}_{n}")
            lines += _part_lines(circuit, {}, {}, node, population)
        finally:
            for part, ref in renamed:
                part.ref = ref
    control, raws = _control(ports, analyses, extra_lines)
    return "\n".join(lines + control), raws


def simulate(circuit, title: str, ports: list[Port], analyses: list, keep_dir: Path | None = None) -> list[Plot]:
    deck, raws = to_deck(circuit, title, ports, analyses)
    results = run_deck(deck, raws, keep_dir=keep_dir)
    return [results[r][0] for r in raws]
