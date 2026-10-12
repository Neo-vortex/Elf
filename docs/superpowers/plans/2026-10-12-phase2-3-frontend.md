# Phase 2.3: Front End — Bias-T, Limiter and Preamp (Implementation Plan)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `make hw` also builds and checks the rest of the front end — the antenna input with its 5 V bias-T, the biased BAV99 limiter and the BFR92A preamp with its bypass relay — and writes `hardware/characterized/preamp.json`, with netlists, BOMs and SVGs for each block.

**Architecture:** The SPICE writer gains terminations beside `Port`: `Source` (a signal source with a noisy series resistor and optional sine tones), `Load` and `Supply`, plus `Noise` and `Tran` analyses. Parts can carry SPICE model text (`spice_models`), which the writer emits once per deck; vendor models live in `hardware/models/`. A small `rfmath` module turns simulation vectors into noise figure, two-tone IMD/OIP3, Rollett stability and shunt impedance. Each block has a check module in the existing pattern.

**Tech Stack:** Python 3.11+, SKiDL 2.3.0, numpy, matplotlib, pytest, ngspice 45 (batch mode), Graphviz `dot`.

**Spec:** `docs/superpowers/specs/2026-10-12-phase2-frontend-mixer-design.md` (§4 Bias-T, Protection and Preamp rows as amended 2026-10-12, §6 `preamp.json`). Parent: `docs/superpowers/specs/2026-10-12-neoradio-hf-receiver-design.md` (§4.2–4.3, §4.6).

**Branch:** create `phase2-3-frontend` from `main` before Task 1. Do not work on `main`.

## Global Constraints

- Bias-T (spec §4): shunt impedance > 500 Ω and insertion loss < 0.3 dB over 0.1–54 MHz. Chain from the RF line: 4.7 µH, 47 µH ∥ 2.2 kΩ, 1 mH (SRF modelled with 2.2 / 2.4 / 6.3 pF); 1 µF DC block; feed decoupled (100 nF + 10 µF). GDT (1.5 pF model) and 1 MΩ bleed at the connector.
- Limiter (spec §4): IMD3 products at −10 dBm two-tone ≤ −100 dBm. BAV99 AC-coupled by 100 nF to a node held at 2.5 V by 10 kΩ; diode ends at 3.5 V and 1.5 V from a 1.5k/1k/1k/1.5k ladder on 5V_A; each ladder node decoupled with 100 nF.
- Preamp (spec §4): gain 14 ± 1 dB over 1–54 MHz at 4.75, 5.0 and 5.25 V; NF ≤ 4.5 dB; OIP3 ≥ +25 dBm (two tones 100 kHz apart at 14 and 50 MHz, −20 dBm each, worst taken); K > 1 and |Δ| < 1 from 0.1 to 500 MHz at all three supply voltages; input RL ≥ 12 dB. Output RL, bypass loss and supply current are reported, not gated.
- Preamp circuit: Rf 360 Ω (with 100 nF DC block), Re1 5.6 Ω, Re2 12 Ω ∥ 1 µF, divider 4.7k/2.7k, 10 Ω base stopper, 10 µH collector choke ∥ 1 kΩ, 100 nF coupling, 100 nF + 10 µF supply decoupling; G6K-2F bypass (COM1 = in, NO1 = amplifier in, NC1 = bypass; COM2 = out, NO2 = amplifier out, NC2 = bypass; energised = amplifier).
- Transistor model: Infineon BFR92P Gummel-Poon + SOT-23 package (`hardware/models/bfr92p.lib`, source in the file header). Diodes: Philips/NXP 1N4148 parameters (`hardware/models/bav99.lib`).
- Noise figure uses an explicit `Source` and `Load`: ngspice treats a `portnum` source's internal z0 as noiseless, which gave an impossible 2.2 dB.
- Transient two-tone runs: 0.25 ns step, 10 µs settle, 20 µs window (whole periods of 100 kHz), `.options reltol=1e-6 abstol=1e-15 vntol=1e-12`. The FFT floor with no diodes is about −197 dBm.
- `make test` stays green: 43 C# + 60 sim + hardware (129 before this plan, 153 after).

## Review Focus

1. **A noise figure computed with S-parameter ports** would look 2 dB too good, because the port resistance is noiseless; noise must use `Source`/`Load`. → Task 6 `test_preamp_meets_targets_and_writes_characterisation` (NF within the 4.5 dB limit but well above 2.2 dB in `preamp.json`).
2. **A two-tone record that is not a whole number of periods** smears the tones, and the IMD3 reading becomes meaningless. → Task 3 `test_off_grid_tone_rejected`, `test_two_tone_recovers_known_intercept`.
3. **A missing vendor model file** must name the file and where it belongs, not leave ngspice with an "unknown subcircuit". → Task 2 `test_missing_model_file_names_the_path`.
4. **Supply at the edges of the USB range (4.75 / 5.25 V)** must keep the preamp stable and in its gain window. → Task 6 (the check gates gain and K over all three voltages).
5. **Two parts that use the same model** must not emit the `.subckt` twice (ngspice rejects a duplicate definition). → Task 2 `test_part_models_are_emitted_once_before_elements`.

---

## File Structure

```
hardware/models/bfr92p.lib, bav99.lib     vendor model text (new)
hardware/neohw/spice.py                   + Port.lines, Source, Load, Supply, Noise, Tran, model emission
hardware/neohw/parts.py                   + inductor(c_par=), npn_bfr92a, diode_pair_bav99, gdt, MODELS_DIR
hardware/neohw/rfmath.py                  NF, two-tone, stability, shunt impedance (new)
hardware/neohw/blocks/bias_tee.py, protection.py, preamp.py      (new)
hardware/neohw/checks/bias_tee.py, protection.py, preamp.py      (new)
hardware/neohw/__main__.py                CHECKS in signal-chain order
hardware/tests/test_spice_terminations.py, test_parts_rf.py, test_rfmath.py,
               test_bias_tee.py, test_protection.py, test_preamp.py (new); test_cli.py (append)
hardware/characterized/preamp.json        (generated, committed)
```

---

### Task 1: Terminations and analyses for noise and transient runs

**Files:**
- Modify: `hardware/neohw/spice.py` (whole file below)
- Test: `hardware/tests/test_spice_terminations.py`

**Interfaces:**
- Produces:
  - `Port.lines() -> list[str]` (the 2.1 `VPORT…` line).
  - `Source(net, name="in", z0=50.0, tones=())`: `VS_<name>0 s_<name>0 0 dc 0 ac 1 [sin(0 A f)]`, further tones stacked as `VS_<name>k`, then `RS_<name> s_<name>K <node> z0`; A = 2·√(2·z0·P) for available power P. `vname` = `VS_<name>0`.
  - `Load(net, name="out", r=50.0)`: `RL_<name> <node> 0 r`. `Supply(net, volts, name="vcc")`: `VSUP_<name> <node> 0 dc V`; `vname`.
  - `Noise(output_net, source, start_hz, stop_hz, points_per_decade=20)` → `noise v(<out>) <source.vname> dec …` + `setplot noise1`.
  - `Tran(step_s, stop_s, start_s=0.0)` → `tran step stop start step` + `linearize`.
  - `to_deck` accepts any of these in `ports` (anything with `.net` and `.lines()`); terminated nets are exempt from the single-connection check; each distinct `part.spice_models` text is emitted once, right after the title.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_spice_terminations.py`:

```python
import math

from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Load, Noise, Op, Port, Source, Supply, Tran, to_deck


def amp_like():
    ckt = Circuit()
    with ckt:
        a, b, vcc, gnd = Net("in"), Net("out"), Net("+5V"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += b
        r2 = parts.resistor("1k"); r2[1] += vcc; r2[2] += b
        r3 = parts.resistor("1k"); r3[1] += b; r3[2] += gnd
    return ckt, a, b, vcc


def test_source_has_noisy_series_resistor_and_ac_drive():
    ckt, a, b, vcc = amp_like()
    src = Source(a, "rf")
    assert src.lines() == ["VS_rf0 s_rf0 0 dc 0 ac 1", "RS_rf s_rf0 in 50"]
    assert src.vname == "VS_rf0"


def test_two_tone_source_stacks_sines_at_available_power():
    src = Source(Net("x"), "rf", tones=((14e6, -10.0), (14.1e6, -10.0)))
    lines = src.lines()
    amp = 2 * math.sqrt(2 * 50 * 1e-3 * 0.1)     # open-circuit peak for -10 dBm available
    assert lines[0] == f"VS_rf0 s_rf0 0 dc 0 ac 1 sin(0 {amp:.9g} 14000000)"
    assert lines[1] == f"VS_rf1 s_rf1 s_rf0 dc 0 sin(0 {amp:.9g} 14100000)"
    assert lines[2] == "RS_rf s_rf1 x 50"


def test_load_supply_and_analyses_render():
    ckt, a, b, vcc = amp_like()
    src = Source(a, "rf")
    deck, raws = to_deck(ckt, "t", [src, Load(b), Supply(vcc, 5.0)],
                         [Op(), Noise(b, src, 1e6, 54e6, 10), Tran(1e-9, 2e-6, 1e-6)])
    assert "RL_out out 0 50" in deck and "VSUP_vcc _5v 0 dc 5" in deck
    assert "noise v(out) VS_rf0 dec 10 1e+06 5.4e+07\nsetplot noise1\nwrite a1.raw all" in deck
    assert "tran 1e-09 2e-06 1e-06 1e-09\nlinearize\nwrite a2.raw all" in deck
    assert raws == ["a0.raw", "a1.raw", "a2.raw"]


def test_terminated_nets_count_as_connected():
    ckt, a, b, vcc = amp_like()
    to_deck(ckt, "t", [Source(a), Load(b), Supply(vcc, 5.0)], [Op()])   # no dangling-net error
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_spice_terminations.py -q`
Expected: FAIL with `ImportError: cannot import name 'Load' from 'neohw.spice'`.

- [ ] **Step 3: Replace `hardware/neohw/spice.py` with**

```python
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
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `133 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/spice.py hardware/tests/test_spice_terminations.py
git commit -m "feat(hw): Source/Load/Supply terminations, Noise and Tran analyses, part model emission"
```

---

### Task 2: Model files and RF parts

**Files:**
- Create: `hardware/models/bfr92p.lib`, `hardware/models/bav99.lib`
- Modify: `hardware/neohw/parts.py` (whole file below)
- Test: `hardware/tests/test_parts_rf.py`

**Interfaces:**
- Consumes: Task 1 model emission.
- Produces:
  - `parts.MODELS_DIR` (= `hardware/models`); a missing model raises `FileNotFoundError` naming the path.
  - `parts.inductor(..., c_par=INDUCTOR_C_PAR)`.
  - `parts.npn_bfr92a(mpn="BFR92A", torob="BFR92")`: pins B=1, E=2, C=3, SOT-23; `X<ref> C B E BFR92P`.
  - `parts.diode_pair_bav99(mpn="BAV99", torob=None)`: pins A=1, K=2, COM=3, SOT-23; `D<ref>a A COM D1N4148`, `D<ref>b COM K D1N4148`.
  - `parts.gdt(capacitance=1.5e-12, torob="گازی برقگیر GDT")`: pins 1, 2; `C<ref>` + 1 GΩ; footprint `NeoRadio:GDT_SMD_2pin` (project-local, made in the layout phase).

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_parts_rf.py`:

```python
import pytest
from skidl import Circuit, Net

from neohw import parts
from neohw.spice import Op, Port, to_deck


def test_bfr92a_pins_and_subckt_call():
    ckt = Circuit()
    with ckt:
        q = parts.npn_bfr92a()
    assert {p.name: p.num for p in q.pins} == {"B": "1", "E": "2", "C": "3"}
    assert q.spice_lines({"C": "c", "B": "b", "E": "0"}) == [f"X{q.ref} c b 0 BFR92P"]
    assert "IMST" in q.spice_models[0] and ".ends BFR92P" in q.spice_models[0]


def test_bav99_is_two_series_diodes():
    ckt = Circuit()
    with ckt:
        d = parts.diode_pair_bav99()
    assert d.spice_lines({"A": "lo", "COM": "y", "K": "hi"}) == [
        f"D{d.ref}a lo y D1N4148", f"D{d.ref}b y hi D1N4148"]
    assert ".model D1N4148" in d.spice_models[0]


def test_gdt_is_small_capacitance():
    ckt = Circuit()
    with ckt:
        g = parts.gdt()
    assert g.spice_lines({"1": "ant", "2": "0"})[0] == f"C{g.ref} ant 0 1.5e-12"


def test_inductor_parallel_c_can_be_set():
    ckt = Circuit()
    with ckt:
        ind = parts.inductor("1m", 40, 0.8e6, c_par=6.3e-12)
    assert ind.spice_lines({"1": "a", "2": "b"})[2] == f"C{ind.ref}_p a b 6.3e-12"


def test_missing_model_file_names_the_path(monkeypatch, tmp_path):
    monkeypatch.setattr(parts, "MODELS_DIR", tmp_path)
    with pytest.raises(FileNotFoundError, match="bfr92p.lib"):
        parts.npn_bfr92a()


def test_part_models_are_emitted_once_before_elements():
    ckt = Circuit()
    with ckt:
        c, b, e = Net("c"), Net("b"), Net("GND")
        for _ in range(2):
            q = parts.npn_bfr92a(); q["C"] += c; q["B"] += b; q["E"] += e
    deck, _ = to_deck(ckt, "t", [Port(c, 1), Port(b, 2)], [Op()])
    assert deck.count(".subckt BFR92P") == 1
    assert deck.index(".subckt BFR92P") < deck.index("XQ1 ")
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_parts_rf.py -q`
Expected: 6 failures: `AttributeError: module 'neohw.parts' has no attribute 'npn_bfr92a'` (and `diode_pair_bav99`, `gdt`, `MODELS_DIR`), and `TypeError: inductor() got an unexpected keyword argument 'c_par'`.

- [ ] **Step 3: Implement**

`hardware/models/bfr92p.lib`:

```
* BFR92P (Infineon) Gummel-Poon chip model + SOT-23 package equivalent circuit.
* Source: Infineon BFR92P datasheet (Aug-03-2001), "SPICE Parameters (Gummel-Poon Model,
* Berkley-SPICE 2G.6 Syntax)", extracted by IMST; valid up to 6 GHz. The NXP BFR92A is the
* same 5 GHz SOT-23 wideband NPN class; NXP's own model page was not reachable.
* Node order: collector base emitter.
.model BFR92P_chip NPN(IS=0.1213f BF=94.733 NF=1.0947 VAF=30 IKF=0.46227 ISE=129.55f NE=1.9052
+ BR=10.729 NR=0.8983 VAR=14.599 IKR=0.01 ISC=0.75557f NC=1.371 RB=14.998 IRB=0.01652m
+ RBM=7.8145 RE=0.29088 RC=0.13793 CJE=10.416f VJE=0.70618 MJE=0.34686 TF=26.796p XTF=0.3817
+ VTF=0.32861 ITF=4.4601m PTF=0 CJC=946.47f VJC=0.84079 MJC=0.4085 XCJC=0.13464 TR=1.2744n
+ CJS=0 VJS=0.75 MJS=0 XTB=0 EG=1.11 XTI=3 FC=0.99545 TNOM=27)
.subckt BFR92P c b e
LBO b b1 0.51n
LBI b1 bi 0.85n
LEO e e1 0.61n
LEI e1 ei 0.69n
LCO c ci 0.49n
CBE b1 e1 73f
CCB b1 c 84f
CCE c e1 165f
Q1 ci bi ei BFR92P_chip
.ends BFR92P
```

`hardware/models/bav99.lib`:

```
* BAV99: two 1N4148-class switching diodes in series (SOT-23). Diode parameters are the
* Philips/NXP 1N4148 SPICE model widely published with their small-signal diode data.
.model D1N4148 D(IS=2.682n N=1.836 RS=0.5664 CJO=4p M=0.3333 VJ=0.5 TT=11.54n BV=100 IBV=100n)
```

Replace `hardware/neohw/parts.py` with:

```python
"""Part constructors: SKiDL parts that also know their SPICE model."""
from __future__ import annotations

import math
from pathlib import Path

from skidl import SKIDL, TEMPLATE, Part, Pin

from neohw.availability import inductor_hit
from neohw.units import format_si, parse_si


def _tagged(part: Part) -> Part:
    """A stable tag (the ref) instead of SKiDL's random one: quiet, reproducible netlists."""
    part.tag = part.ref
    return part


def _template(name: str, prefix: str, pins: list[str] | dict[str, str], footprint: str) -> Part:
    """`pins` is a list of names (pad number = name) or a name -> footprint pad number map."""
    numbered = pins if isinstance(pins, dict) else {p: p for p in pins}
    return Part(tool=SKIDL, name=name, ref_prefix=prefix, dest=TEMPLATE, footprint=footprint,
                pins=[Pin(num=num, name=name) for name, num in numbered.items()])


def _positive(value) -> float:
    v = parse_si(value)
    if v <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    return v


def _num(v: float) -> str:
    return f"{v:.6g}"


def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None, tol: float = 0.01) -> Part:
    ohms = _positive(value)
    p = _tagged(_template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω")))
    p.mpn, p.torob, p.tol = mpn, torob, tol
    p.spice_lines = lambda n, k=1.0: [f"R{p.ref} {n['1']} {n['2']} {_num(ohms * k)}"]
    return p


def capacitor(value, footprint: str = "Capacitor_SMD:C_0805_2012Metric", mpn: str | None = None,
              torob: str | None = None, esr: float = 0.0, tol: float = 0.02, note: str = "") -> Part:
    farads = _positive(value)
    p = _tagged(_template("C", "C", ["1", "2"], footprint)(value=format_si(farads, "F")))
    p.mpn, p.torob, p.tol, p.bom_note = mpn, torob, tol, note

    def lines(n, k=1.0):
        if esr > 0:
            mid = f"{p.ref}_esr"
            return [f"R{p.ref}_esr {n['1']} {mid} {_num(esr)}", f"C{p.ref} {mid} {n['2']} {_num(farads * k)}"]
        return [f"C{p.ref} {n['1']} {n['2']} {_num(farads * k)}"]

    p.spice_lines = lines
    return p


# G6K terminal arrangement (pin name -> footprint pad), as drawn in Omron's
# G6K-2F-RF-V datasheet (top view). The -RF-V variant adds ground pads between
# these terminals; its footprint differs from the plain G6K-2F.
RELAY_PINS = {"COIL+": "1", "NC1": "2", "COM1": "3", "NO1": "4",
              "NO2": "5", "COM2": "6", "NC2": "7", "COIL-": "8"}
G6K_INSULATION_OHMS = 1e9  # Omron G6K datasheet: insulation resistance 1,000 MΩ min (500 VDC)


# Not published for the plain G6K-2F. Omron gives >= 30 dB isolation between poles at 1 GHz
# for the G6K-2F-RF (about 0.05 pF in a 50-ohm system), which has grounded pins between the
# contacts; the plain part is assumed twice as bad. Rev 0 measures it.
G6K_INTER_POLE_C = 0.1e-12


def relay_dpdt(state: str, contact_r: float = 0.05, open_c: float = 0.5e-12,
               inter_pole_c: float = G6K_INTER_POLE_C,
               footprint: str = "Relay_SMD:Relay_DPDT_Omron_G6K-2F", mpn: str = "G6K-2F-5VDC",
               torob: str = "G6K-2F") -> Part:
    if state not in ("released", "energised"):
        raise ValueError(f"relay state must be 'released' or 'energised', got {state!r}")
    p = _tagged(_template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F"))
    p.mpn, p.torob, p.state, p.tol = mpn, torob, state, 0.0
    p.bom_note = f"inter-pole C {inter_pole_c * 1e12:g} pF assumed"

    def lines(n, k=1.0):
        out = []
        for pole in ("1", "2"):
            com = n[f"COM{pole}"]
            closed, open_ = ("NC", "NO") if state == "released" else ("NO", "NC")
            out.append(f"R{p.ref}_{closed}{pole} {com} {n[closed + pole]} {_num(contact_r)}")
            out.append(f"C{p.ref}_{open_}{pole} {com} {n[open_ + pole]} {_num(open_c)}")
            # the insulation resistance also gives every contact node a DC path
            out.append(f"R{p.ref}_{open_}{pole}_ins {com} {n[open_ + pole]} {_num(G6K_INSULATION_OHMS)}")
        for contact in ("COM", "NO", "NC"):   # stray C between the poles' matching contacts
            out.append(f"C{p.ref}_x{contact} {n[contact + '1']} {n[contact + '2']} {_num(inter_pole_c)}")
        return [line for line in out if line.split()[1] != line.split()[2]]  # drop node-to-itself

    p.spice_lines = lines
    return p


# Axial colour-ring (moulded) inductors, measured and selected with an LCR meter.
INDUCTOR_FOOTPRINT = "Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2"
INDUCTOR_C_PAR = 2.5e-12  # SRF: 1 µH ≈ 100 MHz, 10 µH ≈ 32 MHz, 100 µH ≈ 10 MHz (IM-2 class minimums)


def inductor(value, q: float, q_freq_hz: float, tol: float = 0.03, trim: float = 1.0,
             mpn: str | None = None, torob: str | None = None, note: str | None = None,
             c_par: float = INDUCTOR_C_PAR) -> Part:
    """L + series R (giving `q` at `q_freq_hz`) with a parallel C for the self-resonance.

    `value` is the part to buy. `trim` scales it to what the builder selects with an LCR
    meter (a pair is picked so its sum hits the design target); `tol` is the selection
    tolerance. Q is assumed; both are stated in the BOM note.
    """
    henries = _positive(value)
    if q <= 0 or q_freq_hz <= 0:
        raise ValueError(f"q and q_freq_hz must be positive, got {q!r}, {q_freq_hz!r}")
    if not 0.9 <= trim <= 1.1:
        raise ValueError(f"trim {trim:.3f} is outside the ±10 % a selected part can reach")
    henries_built = henries * trim
    r_s = 2 * math.pi * q_freq_hz * henries_built / q
    p = _tagged(_template("L", "L", ["1", "2"], INDUCTOR_FOOTPRINT)(value=format_si(henries, "H")))
    p.mpn, p.torob, p.tol = mpn, torob, tol
    p.torob_match = lambda name: inductor_hit(name, henries)
    p.bom_note = note or f"select to ±{tol * 100:g} % with LCR meter; Q={q:g} assumed"

    def lines(n, k=1.0):
        mid = f"{p.ref}_m"
        return [f"L{p.ref} {n['1']} {mid} {_num(henries_built * k)}",
                f"R{p.ref} {mid} {n['2']} {_num(r_s * k)}",
                f"C{p.ref}_p {n['1']} {n['2']} {_num(c_par)}"]

    p.spice_lines = lines
    return p


MODELS_DIR = Path(__file__).resolve().parents[1] / "models"


def _model(name: str) -> str:
    path = MODELS_DIR / name
    if not path.exists():
        raise FileNotFoundError(f"SPICE model {path} is missing (it is committed in hardware/models/)")
    return path.read_text().strip()


def npn_bfr92a(mpn: str = "BFR92A", torob: str | None = "BFR92") -> Part:
    """NXP BFR92A, SOT-23 (pin 1 base, 2 emitter, 3 collector); BFR92P model."""
    p = _tagged(_template("BFR92A", "Q", {"B": "1", "E": "2", "C": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BFR92A"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (_model("bfr92p.lib"),)
    p.spice_lines = lambda n, k=1.0: [f"X{p.ref} {n['C']} {n['B']} {n['E']} BFR92P"]
    return p


def diode_pair_bav99(mpn: str = "BAV99", torob: str | None = None) -> Part:
    """BAV99, SOT-23: pin 1 anode of D1, pin 3 = D1 cathode + D2 anode, pin 2 cathode of D2."""
    p = _tagged(_template("BAV99", "D", {"A": "1", "K": "2", "COM": "3"},
                          "Package_TO_SOT_SMD:SOT-23")(value="BAV99"))
    p.mpn, p.torob, p.tol = mpn, torob, 0.0
    p.spice_models = (_model("bav99.lib"),)
    p.spice_lines = lambda n, k=1.0: [f"D{p.ref}a {n['A']} {n['COM']} D1N4148",
                                      f"D{p.ref}b {n['COM']} {n['K']} D1N4148"]
    return p


def gdt(capacitance: float = 1.5e-12, torob: str | None = "گازی برقگیر GDT") -> Part:
    """Gas discharge tube at the connector: an open circuit with its capacitance below the
    breakdown voltage (only small-signal behaviour is simulated). Footprint is project-local,
    created in the layout phase."""
    p = _tagged(_template("GDT", "GDT", ["1", "2"], "NeoRadio:GDT_SMD_2pin")(value="GDT 90V"))
    p.mpn, p.torob, p.tol = None, torob, 0.0
    p.spice_lines = lambda n, k=1.0: [f"C{p.ref} {n['1']} {n['2']} {_num(capacitance)}",
                                      f"R{p.ref} {n['1']} {n['2']} {_num(G6K_INSULATION_OHMS)}"]
    return p
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `139 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/models hardware/neohw/parts.py hardware/tests/test_parts_rf.py
git commit -m "feat(hw): BFR92A, BAV99 and GDT parts with committed SPICE models"
```

---

### Task 3: RF figures from simulation vectors

**Files:**
- Create: `hardware/neohw/rfmath.py`
- Test: `hardware/tests/test_rfmath.py`

**Interfaces:**
- Produces: `K_B`, `T_SIM = 300.15`; `noise_figure_db(inoise, z0=50)`; `tone_powers_dbm(v, step_s, freqs_hz, r=50)` (`ValueError` "not on the FFT grid"); `two_tone(v, step_s, f1, f2, r=50) -> {"pout_dbm", "imd3_dbm", "oip3_dbm"}`; `stability(s11, s21, s12, s22) -> (K, |Δ|)`; `shunt_impedance_at_port1(s11, s21, s12, s22, z0=50)`.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_rfmath.py`:

```python
import math

import numpy as np
import pytest

from neohw import rfmath


def test_noise_figure_of_a_matched_resistor_source_is_0_db():
    e = math.sqrt(4 * rfmath.K_B * rfmath.T_SIM * 50)
    assert rfmath.noise_figure_db(np.array([e]))[0] == pytest.approx(0.0, abs=1e-9)
    assert rfmath.noise_figure_db(np.array([e * math.sqrt(2)]))[0] == pytest.approx(3.0103, abs=1e-3)


def test_two_tone_recovers_known_intercept():
    h, n = 1e-9, 20000                  # 20 µs: whole periods of 50 kHz multiples
    t = np.arange(n) * h
    a = math.sqrt(2 * 50 * 1e-3)        # 0 dBm per tone in 50 Ω
    a3 = math.sqrt(2 * 50 * 1e-3 * 10 ** (-60 / 10))
    v = a * (np.sin(2 * np.pi * 14.0e6 * t) + np.sin(2 * np.pi * 14.1e6 * t)) \
        + a3 * np.sin(2 * np.pi * 13.9e6 * t)
    r = rfmath.two_tone(v, h, 14.0e6, 14.1e6)
    assert r["pout_dbm"] == pytest.approx(0.0, abs=1e-6)
    assert r["imd3_dbm"] == pytest.approx(-60.0, abs=1e-6)
    assert r["oip3_dbm"] == pytest.approx(30.0, abs=1e-6)


def test_off_grid_tone_rejected():
    with pytest.raises(ValueError, match="FFT grid"):
        rfmath.tone_powers_dbm(np.zeros(1000), 1e-9, [1.234567e6])


def test_stability_of_a_matched_attenuator():
    s = np.array([0.0]), np.array([0.1]), np.array([0.1]), np.array([0.0])
    k, d = rfmath.stability(*s)
    assert k[0] == pytest.approx((1 + 0.01 ** 2) / 0.02) and d[0] == pytest.approx(0.01)


def _s_from_abcd(a, b, c, d, z0=50.0):
    den = a + b / z0 + c * z0 + d
    return ((a + b / z0 - c * z0 - d) / den, 2 / den, 2 * (a * d - b * c) / den,
            (-a + b / z0 - c * z0 + d) / den)


def test_shunt_impedance_of_a_pi_network():
    # shunt 500 ohm at port 1, then a 10 ohm series resistor (ABCD = shunt @ series)
    rp, rs = 500.0, 10.0
    a, b, c, d = 1.0, rs, 1 / rp, 1 + rs / rp
    s11, s21, s12, s22 = (np.array([x]) for x in _s_from_abcd(a, b, c, d))
    assert rfmath.shunt_impedance_at_port1(s11, s21, s12, s22)[0] == pytest.approx(rp, rel=1e-9)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_rfmath.py -q`
Expected: FAIL with `ImportError: cannot import name 'rfmath' from 'neohw'`.

- [ ] **Step 3: Implement `hardware/neohw/rfmath.py`**

```python
"""RF figures from simulation vectors: noise figure, two-tone IMD, stability, shunt impedance."""
from __future__ import annotations

import math

import numpy as np

K_B = 1.380649e-23
T_SIM = 300.15   # ngspice default temperature (27 °C)


def noise_figure_db(inoise_v_per_rthz: np.ndarray, z0: float = 50.0) -> np.ndarray:
    """NF from ngspice's input-referred noise density (referred to the source voltage) for a
    source of resistance z0 at the simulation temperature."""
    return 20 * np.log10(np.asarray(inoise_v_per_rthz) / math.sqrt(4 * K_B * T_SIM * z0))


def tone_powers_dbm(v: np.ndarray, step_s: float, freqs_hz, r: float = 50.0) -> list[float]:
    """Power in r of each frequency in `freqs_hz` from a uniformly sampled voltage record.
    The record must hold a whole number of periods of every frequency (rectangular window)."""
    n = len(v)
    spec = np.fft.rfft(np.asarray(v)) / n * 2
    bins = np.fft.rfftfreq(n, step_s)
    out = []
    for f in freqs_hz:
        k = int(np.argmin(np.abs(bins - f)))
        if abs(bins[k] - f) > 1e-6 * f:
            raise ValueError(f"{f} Hz is not on the FFT grid (record must hold whole periods)")
        out.append(10 * math.log10(abs(spec[k]) ** 2 / 2 / r / 1e-3 + 1e-30))
    return out


def two_tone(v: np.ndarray, step_s: float, f1: float, f2: float, r: float = 50.0) -> dict:
    """Fundamental and worst third-order product powers, and the output intercept."""
    p1, p2, l3, u3 = tone_powers_dbm(v, step_s, (f1, f2, 2 * f1 - f2, 2 * f2 - f1), r)
    pout, imd3 = (p1 + p2) / 2, max(l3, u3)
    return {"pout_dbm": pout, "imd3_dbm": imd3, "oip3_dbm": pout + (pout - imd3) / 2}


def stability(s11, s21, s12, s22) -> tuple[np.ndarray, np.ndarray]:
    """Rollett K and |Δ|; unconditionally stable where K > 1 and |Δ| < 1."""
    delta = s11 * s22 - s12 * s21
    k = (1 - np.abs(s11) ** 2 - np.abs(s22) ** 2 + np.abs(delta) ** 2) / (2 * np.abs(s12 * s21))
    return k, np.abs(delta)


def shunt_impedance_at_port1(s11, s21, s12, s22, z0: float = 50.0) -> np.ndarray:
    """|Z| of the shunt branch at port 1 of a two-port seen as a pi network (Y11 + Y12)."""
    d = (1 + s11) * (1 + s22) - s12 * s21
    y11 = ((1 - s11) * (1 + s22) + s12 * s21) / d / z0
    y12 = -2 * s12 / d / z0
    return np.abs(1 / (y11 + y12))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `144 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/rfmath.py hardware/tests/test_rfmath.py
git commit -m "feat(hw): noise figure, two-tone IMD, stability and shunt impedance helpers"
```

---

### Task 4: Antenna input and bias-T

**Files:**
- Create: `hardware/neohw/blocks/bias_tee.py`, `hardware/neohw/checks/bias_tee.py`
- Test: `hardware/tests/test_bias_tee.py`

**Interfaces:**
- Consumes: Task 2 `gdt`, `inductor(c_par=)`; Task 3 `shunt_impedance_at_port1`.
- Produces: `bias_tee.build() -> Block("bias_tee")` with ports `rf_in` (net `ant`), `rf_out`, `bias_feed`; `checks.bias_tee.run(out_dir, char_dir) -> BlockResult` with checks `shunt_impedance_min_ohm` (> 500 Ω) and `insertion_loss_max_db` (< 0.3 dB), plus netlist/BOM/SVG artefacts.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_bias_tee.py`:

```python
import pytest

from neohw.blocks import bias_tee
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import bias_tee as check
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def _deck(blk):
    sup = [Supply(blk.ports["vcc"], 5.0)] if "vcc" in blk.ports else []
    return to_deck(blk.circuit, blk.name, [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)] + sup,
                   [Op()])[0]


def test_bias_tee_has_gdt_bleed_block_and_damped_choke_chain():
    blk = bias_tee.build()
    values = sorted(p.value for p in blk.circuit.parts)
    assert values == sorted(["GDT 90V", "1MΩ", "1uF", "4.7uH", "47uH", "1mH", "2.2kΩ", "100nF", "10uF"])
    assert "bias_feed" in _deck(blk)


@pytest.mark.integration
def test_bias_tee_meets_targets(tools, tmp_path):
    res = check.run(tmp_path / "o", tmp_path / "c")
    assert res.passed, [(c.name, c.value) for c in res.checks]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_bias_tee.py -q`
Expected: FAIL with `ImportError: cannot import name 'bias_tee' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

`hardware/neohw/blocks/bias_tee.py`:

```python
"""Antenna input: GDT + 1 MΩ bleed at the connector, then the 5 V bias-T (spec §4).

The bias feed (switched by an AO3401 + PTC in phase 3) is decoupled to ground here. The
choke chain runs smallest-first from the RF line: 4.7 µH, 47 µH (damped by 2.2 kΩ: without
it the 47 µH and the capacitive 1 mH form a series resonance near 38 MHz) and 1 mH.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

# (inductance, parallel C for the SRF, Q, frequency the Q is quoted at): axial colour-ring
# chokes, SRF about 50 MHz / 15 MHz / 2 MHz.
CHOKES = ((4.7e-6, 2.2e-12, 40.0, 8e6), (47e-6, 2.4e-12, 40.0, 2.5e6), (1e-3, 6.3e-12, 40.0, 0.8e6))
DAMPING_OHMS = {1: "2.2k"}   # index into CHOKES -> parallel resistor
CHOKE_NOTE = "choke, rated >= 150 mA"


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd, feed = Net("ant"), Net("rf_out"), Net("GND"), Net("bias_feed")
        g = parts.gdt()
        g[1] += rf_in
        g[2] += gnd
        bleed = parts.resistor("1M")
        bleed[1] += rf_in
        bleed[2] += gnd
        block = parts.capacitor("1u", note="X7R DC block")
        block[1] += rf_in
        block[2] += rf_out
        node = rf_in
        for k, (henries, c_par, q, q_freq) in enumerate(CHOKES):
            nxt = feed if k == len(CHOKES) - 1 else Net(f"choke{k + 1}")
            ch = parts.inductor(henries, q, q_freq, c_par=c_par, note=CHOKE_NOTE)
            ch[1] += node
            ch[2] += nxt
            if k in DAMPING_OHMS:
                rd = parts.resistor(DAMPING_OHMS[k])
                rd[1] += node
                rd[2] += nxt
            node = nxt
        for value, note in (("100n", "X7R decoupling"), ("10u", "X5R decoupling")):
            c = parts.capacitor(value, note=note)
            c[1] += feed
            c[2] += gnd
    return Block("bias_tee", ckt, {"rf_in": rf_in, "rf_out": rf_out, "bias_feed": feed})
```

`hardware/neohw/checks/bias_tee.py`:

```python
"""Bias-T check (spec §4): shunt impedance > 500 Ω and insertion loss < 0.3 dB over 0.1–54 MHz."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, export
from neohw.blocks import bias_tee
from neohw.checks import BlockResult, CheckResult, db
from neohw.rfmath import shunt_impedance_at_port1
from neohw.spice import Port, Sp, simulate

SWEEP = Sp(0.1e6, 54e6, 100, "dec")
SHUNT_MIN_OHMS = 500.0
IL_MAX_DB = 0.3


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir = Path(out_dir)
    blk = bias_tee.build()
    (plot,) = simulate(blk.circuit, "bias-T", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [SWEEP])
    d = plot.data
    f = d["frequency"].real
    s = [d[f"s_{a}_{b}"] for a, b in ((1, 1), (2, 1), (1, 2), (2, 2))]
    z = shunt_impedance_at_port1(*s)
    il = -db(s[1])
    checks = [
        CheckResult("shunt_impedance_min_ohm", float(z.min()), f"> {SHUNT_MIN_OHMS:g}", bool(z.min() > SHUNT_MIN_OHMS), "Ω"),
        CheckResult("insertion_loss_max_db", float(il.max()), f"< {IL_MAX_DB:g}", bool(il.max() < IL_MAX_DB)),
    ]
    result = BlockResult("bias_tee", checks, plots={"shunt |Z| (dB re 1 Ω)": (f, db(z)), "S21 (dB)": (f, db(s[1]))},
                         logx=True)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "bias_tee.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "bias_tee_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "bias_tee.svg"),
    }
    return result
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `146 passed`. The check reports shunt impedance ≥ 679 Ω (at 0.1 MHz, set by the 1 mH choke) and insertion loss ≤ 0.18 dB.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks/bias_tee.py hardware/neohw/checks/bias_tee.py hardware/tests/test_bias_tee.py
git commit -m "feat(hw): antenna input and 5 V bias-T block with shunt-impedance check"
```

---

### Task 5: Biased limiter

**Files:**
- Create: `hardware/neohw/blocks/protection.py`, `hardware/neohw/checks/protection.py`
- Test: `hardware/tests/test_protection.py`

**Interfaces:**
- Consumes: Task 1 `Source(tones=)`, `Load`, `Supply`, `Tran`; Task 2 `diode_pair_bav99`; Task 3 `two_tone`.
- Produces: `protection.build() -> Block("protection")` with ports `rf_in` = `rf_out` = net `rf` and `vcc` (`+5V_A`); `checks.protection.imd3(blk=None) -> two_tone dict`; `checks.protection.run(...)` with checks `imd3_at_minus10dbm_dbm` (≤ −100 dBm) and `insertion_loss_max_db` (reported).

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_protection.py`:

```python
import pytest

from neohw.blocks import protection
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import protection as check
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def _deck(blk):
    sup = [Supply(blk.ports["vcc"], 5.0)] if "vcc" in blk.ports else []
    return to_deck(blk.circuit, blk.name, [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)] + sup,
                   [Op()])[0]


def test_limiter_diodes_sit_between_the_bias_rails():
    blk = protection.build()
    (d,) = [p for p in blk.circuit.parts if p.value == "BAV99"]
    assert (d["A"].net.name, d["COM"].net.name, d["K"].net.name) == ("lim_lo", "lim_y", "lim_hi")
    _deck(blk)


@pytest.mark.integration
def test_limiter_imd3_below_target(tools):
    r = check.imd3()
    assert r["pout_dbm"] == pytest.approx(-10.0, abs=0.1)
    assert r["imd3_dbm"] <= check.IMD3_MAX_DBM
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_protection.py -q`
Expected: FAIL with `ImportError: cannot import name 'protection' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

`hardware/neohw/blocks/protection.py`:

```python
"""Light limiter after the attenuator (parent §4.3): BAV99 anti-parallel pair, AC-coupled and
held about 1 V from conduction by a resistor ladder on 5V_A, so it clamps near ±1.6 V
(~+14 dBm) and stays out of conduction at normal levels (IMD3 target, spec §4)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

# 5V_A - 1.5k - hi (3.5 V) - 1k - mid (2.5 V) - 1k - lo (1.5 V) - 1.5k - GND
LADDER = ("1.5k", "1k", "1k", "1.5k")


def build() -> Block:
    ckt = Circuit()
    with ckt:
        rf, gnd, vcc = Net("rf"), Net("GND"), Net("+5V_A")
        hi, mid, lo, y = Net("lim_hi"), Net("lim_mid"), Net("lim_lo"), Net("lim_y")
        cc = parts.capacitor("100n", note="X7R coupling")
        cc[1] += rf
        cc[2] += y
        rby = parts.resistor("10k")
        rby[1] += mid
        rby[2] += y
        d = parts.diode_pair_bav99()
        d["A"] += lo
        d["COM"] += y
        d["K"] += hi
        nodes = [vcc, hi, mid, lo, gnd]
        for k, value in enumerate(LADDER):
            r = parts.resistor(value)
            r[1] += nodes[k]
            r[2] += nodes[k + 1]
        for node in (hi, mid, lo):
            c = parts.capacitor("100n", note="X7R decoupling")
            c[1] += node
            c[2] += gnd
    return Block("protection", ckt, {"rf_in": rf, "rf_out": rf, "vcc": vcc})
```

`hardware/neohw/checks/protection.py`:

```python
"""Limiter check (spec §4): IMD3 products at −10 dBm two-tone ≤ −100 dBm; small-signal loss."""
from __future__ import annotations

from pathlib import Path

from neohw import availability, export
from neohw.blocks import protection
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.rfmath import two_tone
from neohw.spice import Load, Port, Source, Sp, Supply, Tran, to_deck

TONE_DBM = -10.0
F1, DF = 14.0e6, 0.1e6
SETTLE_S, WINDOW_S, STEP_S = 10e-6, 20e-6, 0.25e-9
IMD3_MAX_DBM = -100.0
TRAN_OPTIONS = ".options reltol=1e-6 abstol=1e-15 vntol=1e-12"


def imd3(blk=None) -> dict:
    blk = blk or protection.build()
    src = Source(blk.ports["rf_in"], "rf", tones=((F1, TONE_DBM), (F1 + DF, TONE_DBM)))
    deck, raws = to_deck(blk.circuit, "limiter two-tone",
                         [src, Load(blk.ports["rf_out"]), Supply(blk.ports["vcc"], 5.0)],
                         [Tran(STEP_S, SETTLE_S + WINDOW_S, SETTLE_S)], extra_lines=[TRAN_OPTIONS])
    (res,) = run_decks([deck], raws)
    d = res[raws[0]][0].data
    t, v = d["time"].real, d["v(rf)"].real
    n = int(round(WINDOW_S / STEP_S))
    return two_tone(v[t >= SETTLE_S - STEP_S / 2][:n], STEP_S, F1, F1 + DF)


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir = Path(out_dir)
    blk = protection.build()
    r = imd3(blk)
    deck, raws = to_deck(blk.circuit, "limiter sp",
                         [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2), Supply(blk.ports["vcc"], 5.0)],
                         [Sp(0.1e6, 54e6, 50, "dec")])
    (res,) = run_decks([deck], raws)
    d = res[raws[0]][0].data
    il = float(-db(d["s_2_1"]).max())
    checks = [CheckResult("imd3_at_minus10dbm_dbm", r["imd3_dbm"], f"≤ {IMD3_MAX_DBM:g}", r["imd3_dbm"] <= IMD3_MAX_DBM, "dBm"),
              CheckResult("insertion_loss_max_db", il, "report", True)]
    result = BlockResult("protection", checks)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "protection.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "protection_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "protection.svg"),
    }
    return result
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `148 passed`. IMD3 ≈ −119 dBm at −10 dBm per tone; small-signal loss ≈ 0.02 dB.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks/protection.py hardware/neohw/checks/protection.py hardware/tests/test_protection.py
git commit -m "feat(hw): biased BAV99 limiter with two-tone IMD3 check"
```

---

### Task 6: BFR92A preamp with bypass relay

**Files:**
- Create: `hardware/neohw/blocks/preamp.py`, `hardware/neohw/checks/preamp.py`
- Test: `hardware/tests/test_preamp.py`

**Interfaces:**
- Consumes: Tasks 1–3; 2.2 `relay_dpdt` (with inter-pole C), `inductor`, `charfile`.
- Produces: `preamp.build(state="amp" | "bypass") -> Block("preamp")` with ports `rf_in`, `rf_out`, `vcc`; `checks.preamp.evaluate() -> dict` and `checks.preamp.run(...)` with checks `gain_min_db`, `gain_max_db`, `nf_max_db`, `oip3_min_dbm`, `stability_k_min`, `stability_delta_max`, `input_rl_min_db` (gated) and `output_rl_min_db`, `bypass_loss_max_db`, `supply_current_ma` (reported); writes `preamp.json` with `freq_hz`, `gain_db`, `nf_db` (1–54 MHz grid), `oip3_dbm`, `bypass_loss_db`, `checks`.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_preamp.py`:

```python
import json

import pytest

from neohw.blocks import preamp
from neohw.spice import Op, Port, Supply, to_deck
from neohw.checks import preamp as check
from neohw.tools import ToolNotFoundError, tool_path


@pytest.fixture
def tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def _deck(blk):
    sup = [Supply(blk.ports["vcc"], 5.0)] if "vcc" in blk.ports else []
    return to_deck(blk.circuit, blk.name, [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)] + sup,
                   [Op()])[0]


@pytest.mark.parametrize("state, relay", [("amp", "energised"), ("bypass", "released")])
def test_preamp_states_drive_the_bypass_relay(state, relay):
    blk = preamp.build(state)
    (k,) = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert k.state == relay
    assert "BFR92P" in _deck(blk)


def test_preamp_rejects_unknown_state():
    with pytest.raises(ValueError):
        preamp.build("off")


@pytest.mark.integration
def test_preamp_meets_targets_and_writes_characterisation(tools, tmp_path):
    res = check.run(tmp_path / "o", tmp_path / "c")
    assert res.passed, [(c.name, c.value) for c in res.checks]
    doc = json.loads((tmp_path / "c" / "preamp.json").read_text())
    assert doc["block"] == "preamp" and doc["status"] == "pass"
    assert len(doc["freq_hz"]) == len(doc["gain_db"]) == len(doc["nf_db"]) > 50
    assert doc["oip3_dbm"] >= check.OIP3_MIN_DBM and doc["bypass_loss_db"] < 0.3
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_preamp.py -q`
Expected: FAIL with `ImportError: cannot import name 'preamp' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

`hardware/neohw/blocks/preamp.py`:

```python
"""BFR92A resistive shunt-feedback preamp with a G6K-2F bypass relay (spec §4).

Gain ≈ Rf / Re1 for Rf·Re1 ≈ 50²; split emitter (5.6 Ω at RF, + 12 Ω bypassed for DC);
collector fed by a 10 µH choke damped by 1 kΩ; 10 Ω base stopper; base divider 4.7k / 2.7k.
State "amp": relay energised (COM-NO through the amplifier); "bypass": released.
"""
from __future__ import annotations

from skidl import Circuit, Net

from neohw import parts
from neohw.blocks import Block

VALUES = {"rf": "360", "re1": "5.6", "re2": "12", "r1": "4.7k", "r2": "2.7k", "rbs": "10",
          "rcl": "1k", "lc": 10e-6}
STATES = ("amp", "bypass")


def _r(value, a: Net, b: Net) -> None:
    r = parts.resistor(value)
    r[1] += a
    r[2] += b


def _c(value, a: Net, b: Net, note: str = "X7R") -> None:
    c = parts.capacitor(value, note=note)
    c[1] += a
    c[2] += b


def build(state: str = "amp") -> Block:
    if state not in STATES:
        raise ValueError(f"preamp state must be one of {STATES}, got {state!r}")
    v = VALUES
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd, vcc = Net("rf_in"), Net("rf_out"), Net("GND"), Net("+5V_A")
        amp_in, amp_out, bypass = Net("pre_in"), Net("pre_out"), Net("pre_bypass")
        b_in, b, c, e, e2, fb = (Net(f"pre_{n}") for n in ("bin", "b", "c", "e", "e2", "fb"))
        k = parts.relay_dpdt("energised" if state == "amp" else "released")
        k["COM1"] += rf_in
        k["NO1"] += amp_in
        k["NC1"] += bypass
        k["COM2"] += rf_out
        k["NO2"] += amp_out
        k["NC2"] += bypass
        _c("100n", amp_in, b_in)
        _r(v["rbs"], b_in, b)
        _r(v["r1"], vcc, b_in)
        _r(v["r2"], b_in, gnd)
        _r(v["rf"], c, fb)
        _c("100n", fb, b_in)
        q = parts.npn_bfr92a()
        q["C"] += c
        q["B"] += b
        q["E"] += e
        _r(v["re1"], e, e2)
        _r(v["re2"], e2, gnd)
        _c("1u", e2, gnd)
        lc = parts.inductor(v["lc"], 30.0, 14e6, note="collector choke, rated >= 50 mA")
        lc[1] += vcc
        lc[2] += c
        _r(v["rcl"], vcc, c)
        _c("100n", c, amp_out)
        _c("100n", vcc, gnd, "X7R decoupling")
        _c("10u", vcc, gnd, "X5R decoupling")
    return Block("preamp", ckt, {"rf_in": rf_in, "rf_out": rf_out, "vcc": vcc})
```

`hardware/neohw/checks/preamp.py`:

```python
"""Preamp check (spec §4, NF amended): gain 14 ± 1 dB over 1–54 MHz, NF ≤ 4.5 dB, OIP3 ≥ +25 dBm,
unconditionally stable 0.1–500 MHz (also at 4.75 and 5.25 V), input RL ≥ 12 dB; output RL and
bypass loss reported. Writes preamp.json (spec §6)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from neohw import availability, charfile, export
from neohw.blocks import preamp
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.rfmath import noise_figure_db, stability, two_tone
from neohw.spice import Load, Noise, Op, Port, Source, Sp, Supply, Tran, node_name, to_deck

BAND = (1e6, 54e6)
GAIN_DB, GAIN_TOL_DB = 14.0, 1.0
NF_MAX_DB = 4.5
OIP3_MIN_DBM = 25.0
RL_IN_MIN_DB = 12.0
SUPPLY_V = (4.75, 5.0, 5.25)
STAB_SWEEP = Sp(0.1e6, 500e6, 40, "dec")
TONES_HZ = (14.0e6, 50.0e6)
DF = 0.1e6
TONE_DBM = -20.0
SETTLE_S, WINDOW_S, STEP_S = 10e-6, 20e-6, 0.25e-9
TRAN_OPTIONS = ".options reltol=1e-6 abstol=1e-15 vntol=1e-12"


def _ports(blk, volts):
    return [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2), Supply(blk.ports["vcc"], volts, "5v")]


def _sparams(d):
    return [d[f"s_{a}_{b}"] for a, b in ((1, 1), (2, 1), (1, 2), (2, 2))]


def evaluate() -> dict:
    amp, byp = preamp.build("amp"), preamp.build("bypass")
    decks = [to_deck(amp.circuit, f"preamp {v} V", _ports(amp, v), [Op(), STAB_SWEEP]) for v in SUPPLY_V]
    decks.append(to_deck(byp.circuit, "preamp bypass", _ports(byp, 5.0), [Sp(*BAND, 50, "dec")]))
    src = Source(amp.ports["rf_in"], "rf")
    decks.append(to_deck(amp.circuit, "preamp noise",
                         [src, Load(amp.ports["rf_out"]), Supply(amp.ports["vcc"], 5.0, "5v")],
                         [Noise(amp.ports["rf_out"], src, BAND[0], BAND[1], 20)]))
    for f1 in TONES_HZ:
        tones = Source(amp.ports["rf_in"], "rf", tones=((f1, TONE_DBM), (f1 + DF, TONE_DBM)))
        decks.append(to_deck(amp.circuit, f"preamp two-tone {f1:g}",
                             [tones, Load(amp.ports["rf_out"]), Supply(amp.ports["vcc"], 5.0, "5v")],
                             [Tran(STEP_S, SETTLE_S + WINDOW_S, SETTLE_S)], extra_lines=[TRAN_OPTIONS]))
    out = run_decks([d for d, _ in decks[:3]], decks[0][1])
    corners = []
    for v, res in zip(SUPPLY_V, out):
        op, sp = res["a0.raw"][0].data, res["a1.raw"][0].data
        f = sp["frequency"].real
        s = _sparams(sp)
        k, delta = stability(*s)
        band = (f >= BAND[0]) & (f <= BAND[1])
        corners.append({"volts": v, "f": f, "s": s, "band": band, "k_min": float(k.min()),
                        "delta_max": float(delta.max()),
                        "current_ma": float(-op["i(vsup_5v)"][0].real * 1e3)})
    (byp_res,) = run_decks([decks[3][0]], decks[3][1])
    (nz_res,) = run_decks([decks[4][0]], decks[4][1])
    tt = run_decks([d for d, _ in decks[5:]], decks[5][1])
    bd = byp_res["a0.raw"][0].data
    nz = nz_res["a0.raw"][0].data
    oip3 = []
    for f1, res in zip(TONES_HZ, tt):
        d = res["a0.raw"][0].data
        t, vout = d["time"].real, d[f"v({node_name(amp.ports['rf_out'])})"].real
        n = int(round(WINDOW_S / STEP_S))
        oip3.append(two_tone(vout[t >= SETTLE_S - STEP_S / 2][:n], STEP_S, f1, f1 + DF)["oip3_dbm"])
    return {"corners": corners, "bypass_loss_db": float(-db(bd["s_2_1"]).min()),
            "bypass_loss_max_db": float(-db(bd["s_2_1"]).max()),
            "nf_f": nz["frequency"].real, "nf_db": noise_figure_db(nz["inoise_spectrum"].real),
            "oip3_dbm": dict(zip(TONES_HZ, oip3))}


def run(out_dir: Path, char_dir: Path) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    r = evaluate()
    nominal = next(c for c in r["corners"] if c["volts"] == 5.0)
    f, s, band = nominal["f"], nominal["s"], nominal["band"]
    gain = db(s[1])[band]
    rl_in, rl_out = -db(s[0])[band], -db(s[3])[band]
    nf_max = float(r["nf_db"].max())
    oip3_min = min(r["oip3_dbm"].values())
    k_min = min(c["k_min"] for c in r["corners"])
    d_max = max(c["delta_max"] for c in r["corners"])
    gains = [db(c["s"][1])[c["band"]] for c in r["corners"]]
    g_lo, g_hi = float(min(g.min() for g in gains)), float(max(g.max() for g in gains))
    checks = [
        CheckResult("gain_min_db", g_lo, f"≥ {GAIN_DB - GAIN_TOL_DB:g}", g_lo >= GAIN_DB - GAIN_TOL_DB),
        CheckResult("gain_max_db", g_hi, f"≤ {GAIN_DB + GAIN_TOL_DB:g}", g_hi <= GAIN_DB + GAIN_TOL_DB),
        CheckResult("nf_max_db", nf_max, f"≤ {NF_MAX_DB:g}", nf_max <= NF_MAX_DB),
        CheckResult("oip3_min_dbm", oip3_min, f"≥ {OIP3_MIN_DBM:g}", oip3_min >= OIP3_MIN_DBM, "dBm"),
        CheckResult("stability_k_min", k_min, "> 1", k_min > 1, ""),
        CheckResult("stability_delta_max", d_max, "< 1", d_max < 1, ""),
        CheckResult("input_rl_min_db", float(rl_in.min()), f"≥ {RL_IN_MIN_DB:g}", float(rl_in.min()) >= RL_IN_MIN_DB),
        CheckResult("output_rl_min_db", float(rl_out.min()), "report", True),
        CheckResult("bypass_loss_max_db", r["bypass_loss_max_db"], "report", True),
        CheckResult("supply_current_ma", nominal["current_ma"], "report", True, "mA"),
    ]
    fb = f[band]
    nf_on_grid = np.interp(fb, r["nf_f"], r["nf_db"])
    result = BlockResult("preamp", checks, plots={"gain (dB)": (fb, gain), "NF (dB)": (fb, nf_on_grid),
                                                   "input RL (dB)": (fb, rl_in)}, logx=True)
    blk = preamp.build("amp")
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(blk.circuit, out_dir / "preamp.net"),
        "bom": export.write_bom(blk.circuit, out_dir / "preamp_bom.csv", data),
        "svg": export.svg_diagram(blk.circuit, out_dir / "preamp.svg"),
        "characterisation": charfile.write(
            char_dir / "preamp.json", "preamp", "pass" if result.passed else "fail",
            {"freq_hz": fb.tolist(), "gain_db": gain.tolist(), "nf_db": nf_on_grid.tolist(),
             "oip3_dbm": oip3_min, "bypass_loss_db": r["bypass_loss_max_db"],
             "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `152 passed`. Reference values: gain 13.14–14.78 dB over the three supply voltages, NF 4.33 dB, OIP3 +26.1 dBm (worst of 14 and 50 MHz), K ≥ 1.107, |Δ| ≤ 0.63, input RL 14.3 dB, output RL 11.2 dB, bypass loss 0.007 dB, 24.1 mA.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks/preamp.py hardware/neohw/checks/preamp.py hardware/tests/test_preamp.py
git commit -m "feat(hw): BFR92A preamp with bypass relay; gain, NF, OIP3, stability checks and preamp.json"
```

---

### Task 7: Register the checks and characterise

**Files:**
- Modify: `hardware/neohw/__main__.py` (whole file below)
- Test: `hardware/tests/test_cli.py` (append)
- Create (generated, committed): `hardware/characterized/preamp.json`; regenerate the others

**Interfaces:**
- Produces: `__main__.CHECKS` keys in signal-chain order: `bias_tee`, `attenuator`, `protection`, `bpf_bank`, `preamp`.

- [ ] **Step 1: Write the failing test**

Append to `hardware/tests/test_cli.py`:

```python
def test_checks_run_in_signal_chain_order():
    assert list(cli.CHECKS) == ["bias_tee", "attenuator", "protection", "bpf_bank", "preamp"]
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd hardware && python3 -m pytest tests/test_cli.py -q`
Expected: 1 failure (`['attenuator', 'bpf_bank'] == [...]`).

- [ ] **Step 3: Replace `hardware/neohw/__main__.py` with**

```python
"""Command line: python -m neohw run"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from neohw.checks import attenuator as attenuator_check
from neohw.checks import bias_tee as bias_tee_check
from neohw.checks import bpf as bpf_check
from neohw.checks import preamp as preamp_check
from neohw.checks import protection as protection_check
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import NgspiceError
from neohw.report import write_report
from neohw.spice import SpiceError
from neohw.tools import ToolNotFoundError

# signal-chain order
CHECKS = {"bias_tee": bias_tee_check.run, "attenuator": attenuator_check.run,
          "protection": protection_check.run, "bpf_bank": bpf_check.run, "preamp": preamp_check.run}


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="neohw")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run block checks, write characterisation files and a report")
    run.add_argument("--out", type=Path, default=Path("build/hw"))
    run.add_argument("--characterized", type=Path, default=Path("characterized"))
    run.add_argument("--report", type=Path, default=Path("build/hw/report.html"))
    args = parser.parse_args(argv)

    results, errored = [], False
    for name, check in CHECKS.items():
        try:
            result = check(args.out, args.characterized)
        except (ToolNotFoundError, NgspiceError, SpiceError, RuntimeError) as e:
            print(f"ERROR {name}\nerror: {e}", file=sys.stderr)
            errored = True
            result = BlockResult(name, [CheckResult("ERROR", float("nan"), str(e).splitlines()[0], False, "")])
        else:
            print(f"{'PASS' if result.passed else 'FAIL'}  {name}")
        results.append(result)
    report = write_report(results, args.report)
    print(f"report: {report}")
    if errored:
        return 2
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
```

- [ ] **Step 4: Run the tests**

Run: `cd hardware && python3 -m pytest -q`
Expected: `153 passed`.

- [ ] **Step 5: Commit, characterise, commit again**

```bash
git add hardware/neohw/__main__.py hardware/tests/test_cli.py
git commit -m "feat(hw): run bias-T, limiter and preamp checks in make hw"
```

Run: `make hw`
Expected (about 100 s): `PASS  bias_tee`, `PASS  attenuator`, `PASS  protection`, `PASS  bpf_bank`, `PASS  preamp`; `hardware/characterized/preamp.json` appears and the other characterisation files change only in `generated_by`.

```bash
git add hardware/characterized
git commit -m "chore(hw): characterise the preamp and regenerate from $(git log -1 --format=%h -- hardware/neohw)"
```

Run: `make hw`, then `git status --short`
Expected: empty (identical regeneration).

Run: `make test`
Expected: .NET (43), sim (60) and hardware (153) suites all pass.
