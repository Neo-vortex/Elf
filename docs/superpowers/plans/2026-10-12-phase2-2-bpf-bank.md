# Phase 2.2: Band-Pass Filter Bank (Implementation Plan)

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `make hw` synthesises the 10-filter band-pass bank as SKiDL code, simulates each filter selected in the full bank (nominal and 200 Monte Carlo runs), checks it against the spec §4 targets, and writes `hardware/characterized/bpf_<n>.s2p` (0.05–200 MHz), `bpf_bank.json` (band edges, selection rule, results and an inductor build sheet), a KiCad 9 netlist, a BOM, an SVG and a report section.

**Architecture:** Chebyshev ladder synthesis (`neohw.synth.filters`) gives ideal element values. Each position is realised from 1–2 preferred-value parts (`neohw.eseries`): C0G capacitors in parallel and axial colour-ring inductors in series. The builder selects the inductors with an LCR meter so that each position reaches its ideal value within ±3 %, and the simulation models exactly that (a `trim` on the bought value plus ±3 % Monte Carlo). `blocks/bpf.py` builds one relay-switched filter; `blocks/bpf_bank.py` puts all ten on common buses. `checks/bpf.py` generates every Monte Carlo deck up front and runs them in parallel ngspice processes.

**Tech Stack:** Python 3.11+, SKiDL 2.3.0, numpy, matplotlib, pytest, ngspice 45 (batch mode), Graphviz `dot`.

**Spec:** `docs/superpowers/specs/2026-10-12-phase2-frontend-mixer-design.md` (§2 BPF amendments, §4 BPF row, §6 `bpf_<n>.s2p` / `bpf_bank.json`). Parent: `docs/superpowers/specs/2026-10-12-neoradio-hf-receiver-design.md` (§4.5 band table and safe state).

**Branch:** create `phase2-2-bpf-bank` from `main` before Task 1. Do not work on `main`.

## Global Constraints

- Bands (parent §4.5), filter n covers: 1 0.1–0.5, 2 0.5–1.7, 3 1.7–2.5, 4 2.5–4.5, 5 4.5–7.5, 6 7.5–12, 7 12–16, 8 16–22, 9 22–30, 10 30–54 MHz.
- Topology (spec §4): 5th-order HPF then LPF of order 9 (filters 1–6, 10) or 7 (filters 7–9), Chebyshev **0.02 dB** ripple, 50 Ω. HPF cutoff = lower edge / 1.03; LPF cutoff = upper edge × 1.03 (× 1.05 on filter 10).
- Inductors: 1–2 E12 axial colour-ring parts in series, none below 0.1 µH, selected with an LCR meter to **±3 %** of the ideal value; model = L + series R giving **Q = 30 at the band centre** (√(lo·hi)) + **2.5 pF** parallel C. Capacitors: 1–2 E24 C0G 0805 parts in parallel, none below 1 pF, **±2 %**.
- One G6K-2F per filter. COM = filter end, NO = bus, NC = GND: an unselected filter has both ends grounded and the buses see only open contacts. All relays released = muted (parent §4.5 safe state).
- Targets (spec §4): in-band IL ≤ 3 dB (filters 1–7) / ≤ 2 dB (8–10); in-band RL ≥ 14 dB; rejection ≥ 50 dB from 3× the lower to 3× the upper edge on filters 3–10 (none on 1–2: 3× their lower edge is inside the passband); Monte Carlo 200 runs, uniform L ±3 % / C ±2 %, ≥ 95 % of runs pass all three. Muted bank: S21 ≤ −40 dB over 0.1–54 MHz.
- Monte Carlo is reproducible: run r of filter n uses `numpy.random.default_rng([n, r])`.
- `bpf_<n>.s2p`: Touchstone 1.1, `# HZ S RI R 50`, 0.05 MHz to ≥ 200 MHz, 50 points/decade, nominal values, filter n selected in the full bank; comment lines carry the band and `generated_by`.
- Characterisation files keep the phase 2.1 contract (`schema_version` 1, `generated_by`, `block`, `status`).
- `make test` stays green: 43 C# + 60 sim + all hardware tests (66 before this plan, 123 after).

## Review Focus

1. **A block that merges two nets (`a += b`)** must give one SPICE node. SKiDL leaves the pins on two Net objects, and the 2.1 writer silently split the node. → Task 3 `test_merged_nets_share_one_spice_node`.
2. **An inductor pair that selection cannot bring to the target** (bought values more than 10 % away) must be rejected when the circuit is built, not simulated as if the builder could reach it. → Task 4 `test_inductor_trim_beyond_selection_range_rejected`, Task 6 `test_design_has_hpf_then_lpf_with_buyable_parts`.
3. **An LO frequency on a band edge (0.5 MHz, 54 MHz) or outside 0.1–54 MHz** must select the documented filter or raise, never pick a neighbour silently. → Task 6 `test_selection_rule`, `test_selection_outside_range_rejected`.
4. **Persian shop titles with mixed units** (`نانو هانری`, `میکرو هانری`, `µH`, `UH`, `NH`) must be parsed, and a title that names a different value must not count as this part. → Task 4 `test_inductor_values_parsed_from_shop_titles`, `test_value_match_replaces_token_match`.
5. **The same commit must regenerate identical characterisation files** (spec §8), so Monte Carlo must not depend on run order or thread scheduling. → Task 7 `test_monte_carlo_is_reproducible`.

---

## File Structure

```
hardware/neohw/
  eseries.py          E12/E24 values, snap, 1–2 part combinations (new)
  synth/__init__.py   (new)
  synth/filters.py    Chebyshev g-values, lowpass/highpass ladders (new)
  parts.py            + tol, variation multiplier k, stable tags, inductor(), capacitor note
  spice.py            + variation, merged-net canonicalisation
  availability.py     + inductor value parsing, lookup(match=)
  export.py           + BOM note column, natural ref order
  ngspice.py          + run_decks (parallel)
  touchstone.py       Touchstone 1.1 write/read (new)
  blocks/bpf.py       one switched filter: design, add_filter, add_switched, build (new)
  blocks/bpf_bank.py  10 filters on buses, select(f) (new)
  checks/__init__.py  + BlockResult.logx
  checks/bpf.py       IL/RL/rejection/MC/mute checks, s2p + bpf_bank.json (new)
  report.py           + log frequency axis
  __main__.py         + bpf_bank in CHECKS
hardware/tests/       test_eseries, test_synth, test_touchstone, test_bpf_blocks, test_bpf_check (new); others extended
hardware/characterized/  bpf_1..10.s2p, bpf_bank.json (generated, committed); attenuator.json regenerated
```

---

### Task 1: E-series values and part combinations

**Files:**
- Create: `hardware/neohw/eseries.py`
- Test: `hardware/tests/test_eseries.py`

**Interfaces:**
- Produces:
  - `eseries.E12`, `eseries.E24`: tuples of mantissas.
  - `eseries.values(series, lo=1e-13, hi=1e-2) -> list[float]`: sorted, exact decimals (`4.7e-06`, not `4.700000000000001e-06`).
  - `eseries.snap(value, series) -> float`: nearest on a log scale; `ValueError` for value ≤ 0.
  - `eseries.combo(value, series, tolerance=0.01, minimum=0.0) -> tuple[float, ...]`: one part if within `tolerance`, else the best pair (larger first) whose sum is closest; no part below `minimum`.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_eseries.py`:

```python
import pytest

from neohw.eseries import E12, E24, combo, snap, values


def test_values_are_exact_decimals():
    assert 4.7e-06 in values(E12)
    assert 3.3e-07 in values(E12)
    assert values(E24, 1e-12, 1e-11)[:3] == [1e-12, 1.1e-12, 1.2e-12]


@pytest.mark.parametrize("value, series, expected", [
    (4.6e-6, E12, 4.7e-6), (1.04e-9, E24, 1e-9), (9.6e-8, E12, 1e-7), (2.0e-10, E24, 2.0e-10),
])
def test_snap_to_nearest_on_log_scale(value, series, expected):
    assert snap(value, series) == expected


def test_snap_rejects_non_positive():
    with pytest.raises(ValueError):
        snap(0.0, E12)


def test_combo_uses_one_part_when_close_enough():
    assert combo(1.005e-9, E24) == (1e-9,)


def test_combo_pairs_two_parts_that_add_up():
    assert combo(3.62e-6, E12) == (3.3e-06, 3.3e-07)
    assert sum(combo(1.07e-9, E24)) == pytest.approx(1.07e-9, rel=0.01)


def test_combo_respects_minimum_part_value():
    parts = combo(4.99e-7, E12, minimum=1e-7)
    assert all(p >= 1e-7 for p in parts)
    assert sum(parts) == pytest.approx(4.99e-7, rel=0.05)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_eseries.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'neohw.eseries'`.

- [ ] **Step 3: Implement `hardware/neohw/eseries.py`**

```python
"""E-series preferred values and one- or two-part combinations."""
from __future__ import annotations

import math

E12 = (1.0, 1.2, 1.5, 1.8, 2.2, 2.7, 3.3, 3.9, 4.7, 5.6, 6.8, 8.2)
E24 = (1.0, 1.1, 1.2, 1.3, 1.5, 1.6, 1.8, 2.0, 2.2, 2.4, 2.7, 3.0,
       3.3, 3.6, 3.9, 4.3, 4.7, 5.1, 5.6, 6.2, 6.8, 7.5, 8.2, 9.1)


def values(series: tuple[float, ...], lo: float = 1e-13, hi: float = 1e-2) -> list[float]:
    """Every value of `series` from `lo` to `hi`, built from decimal text so 4.7e-06 is exact."""
    out = []
    for d in range(math.floor(math.log10(lo)), math.ceil(math.log10(hi)) + 1):
        out += [float(f"{s}e{d}") for s in series]
    return sorted(v for v in out if lo <= v <= hi)


def snap(value: float, series: tuple[float, ...]) -> float:
    """Nearest preferred value on a log scale."""
    if value <= 0:
        raise ValueError(f"value must be positive, got {value!r}")
    d = math.floor(math.log10(value))
    candidates = [float(f"{s}e{dd}") for dd in (d - 1, d, d + 1) for s in series]
    return min(candidates, key=lambda c: abs(math.log(c / value)))


def combo(value: float, series: tuple[float, ...], tolerance: float = 0.01,
          minimum: float = 0.0) -> tuple[float, ...]:
    """One or two preferred values whose sum is closest to `value`.

    Inductors in series and capacitors in parallel both add, so one rule serves both.
    A single part is used when it lands within `tolerance` (fractional). No part is
    smaller than `minimum` (the smallest value that can be bought).
    """
    single = max(snap(value, series), snap(minimum, series) if minimum else 0.0)
    best_err, best = abs(math.log(single / value)), (single,)
    if best_err <= tolerance:
        return best
    for a in values(series, max(value / 1000, minimum or 1e-13), value):
        if a >= value:
            break
        b = snap(value - a, series)
        if b < minimum:
            continue
        err = abs(math.log((a + b) / value))
        if err < best_err - 1e-12:
            best_err, best = err, tuple(sorted((a, b), reverse=True))
    return best
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_eseries.py -q`
Expected: `9 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/eseries.py hardware/tests/test_eseries.py
git commit -m "feat(hw): E12/E24 values and two-part combinations"
```

---

### Task 2: Chebyshev ladder synthesis

**Files:**
- Create: `hardware/neohw/synth/__init__.py`, `hardware/neohw/synth/filters.py`
- Test: `hardware/tests/test_synth.py`

**Interfaces:**
- Produces:
  - `filters.Element(kind: str, value: float)`, frozen; kind ∈ `C_shunt`, `L_series`, `C_series`, `L_shunt`.
  - `filters.chebyshev_g(order, ripple_db) -> list[float]`: odd orders only (equal terminations); `ValueError` for even/zero order or ripple ≤ 0.
  - `filters.lowpass(fc_hz, order, ripple_db, r0=50.0) -> list[Element]`: shunt-C first.
  - `filters.highpass(fc_hz, order, ripple_db, r0=50.0) -> list[Element]`: series-C first.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_synth.py`:

```python
import math

import pytest

from neohw.synth.filters import chebyshev_g, highpass, lowpass


def test_chebyshev_matches_matthaei_table_0p1db():
    # Matthaei, Young & Jones, Table 4.05-2(a), 0.1 dB ripple
    assert chebyshev_g(5, 0.1) == pytest.approx([1.1468, 1.3712, 1.9750, 1.3712, 1.1468], abs=1e-4)
    assert chebyshev_g(7, 0.1)[:4] == pytest.approx([1.1812, 1.4228, 2.0967, 1.5734], abs=1e-4)


def test_odd_orders_are_symmetric():
    g = chebyshev_g(9, 0.02)
    assert g == pytest.approx(g[::-1])


@pytest.mark.parametrize("order, ripple", [(4, 0.1), (0, 0.1), (5, 0.0)])
def test_invalid_prototype_rejected(order, ripple):
    with pytest.raises(ValueError):
        chebyshev_g(order, ripple)


def test_lowpass_at_unit_scale_equals_prototype():
    els = lowpass(1 / (2 * math.pi), 5, 0.1, r0=1.0)
    assert [e.kind for e in els] == ["C_shunt", "L_series", "C_shunt", "L_series", "C_shunt"]
    assert [e.value for e in els] == pytest.approx(chebyshev_g(5, 0.1))


def test_highpass_is_the_reciprocal_transform():
    els = highpass(1 / (2 * math.pi), 5, 0.1, r0=1.0)
    assert [e.kind for e in els] == ["C_series", "L_shunt", "C_series", "L_shunt", "C_series"]
    assert [e.value for e in els] == pytest.approx([1 / g for g in chebyshev_g(5, 0.1)])


def test_lowpass_scales_to_50_ohm_and_frequency():
    (c1, l2, *_) = lowpass(10e6, 5, 0.1)
    w = 2 * math.pi * 10e6
    assert c1.value == pytest.approx(1.1468 / (50 * w), rel=1e-4)
    assert l2.value == pytest.approx(1.3712 * 50 / w, rel=1e-4)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_synth.py -q`
Expected: FAIL with `ModuleNotFoundError: No module named 'neohw.synth'`.

- [ ] **Step 3: Implement**

`hardware/neohw/synth/__init__.py`:

```python
"""Filter synthesis."""
```

`hardware/neohw/synth/filters.py`:

```python
"""Chebyshev ladder synthesis (Matthaei, Young & Jones, 1964, §4.05)."""
from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Element:
    """One ladder position, from the source side. kind: C_shunt, L_series, C_series or L_shunt."""
    kind: str
    value: float


def chebyshev_g(order: int, ripple_db: float) -> list[float]:
    """Lowpass prototype values g1..gn for equal 1 Ω terminations (odd orders only)."""
    if order < 1 or order % 2 == 0:
        raise ValueError(f"order must be odd and positive, got {order}")
    if ripple_db <= 0:
        raise ValueError(f"ripple must be positive, got {ripple_db}")
    beta = math.log(1 / math.tanh(ripple_db / 17.37))
    gamma = math.sinh(beta / (2 * order))
    a = [math.sin((2 * k - 1) * math.pi / (2 * order)) for k in range(1, order + 1)]
    b = [gamma ** 2 + math.sin(k * math.pi / order) ** 2 for k in range(1, order + 1)]
    g = [2 * a[0] / gamma]
    for k in range(1, order):
        g.append(4 * a[k - 1] * a[k] / (b[k - 1] * g[-1]))
    return g


def lowpass(fc_hz: float, order: int, ripple_db: float, r0: float = 50.0) -> list[Element]:
    """Shunt-C-first lowpass: (order + 1) / 2 capacitors, (order - 1) / 2 inductors."""
    w = 2 * math.pi * fc_hz
    return [Element("C_shunt", g / (r0 * w)) if k % 2 == 0 else Element("L_series", g * r0 / w)
            for k, g in enumerate(chebyshev_g(order, ripple_db))]


def highpass(fc_hz: float, order: int, ripple_db: float, r0: float = 50.0) -> list[Element]:
    """Series-C-first highpass (lowpass-to-highpass transform of `lowpass`)."""
    w = 2 * math.pi * fc_hz
    return [Element("C_series", 1 / (g * r0 * w)) if k % 2 == 0 else Element("L_shunt", r0 / (g * w))
            for k, g in enumerate(chebyshev_g(order, ripple_db))]
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest tests/test_synth.py -q`
Expected: `8 passed`. The 0.1 dB values reproduce Matthaei's table to 4 decimals; that validates the formula for the 0.02 dB ripple the bank uses.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/synth hardware/tests/test_synth.py
git commit -m "feat(hw): Chebyshev lowpass/highpass ladder synthesis"
```

---

### Task 3: Part variation, stable tags and merged nets

**Files:**
- Modify: `hardware/neohw/parts.py`, `hardware/neohw/spice.py`
- Test: `hardware/tests/test_parts.py`, `hardware/tests/test_spice.py`, `hardware/tests/test_export.py` (append)

**Interfaces:**
- Consumes: phase 2.1 `parts.resistor/capacitor/relay_dpdt`, `spice.to_deck`.
- Produces:
  - Every part has `tol: float` (resistor 0.01, capacitor 0.02, relay 0.0) and `spice_lines(nodes, k=1.0)`, where `k` multiplies the part's value.
  - `parts.capacitor(..., tol=0.02, note="")`; `note` becomes `p.bom_note`.
  - Every part's SKiDL `tag` equals its ref (no "Missing tag" warnings; stable netlists).
  - `spice.to_deck(circuit, title, ports, analyses, extra_lines=(), variation: dict[str, float] | None = None)`: `variation` maps a ref to `k`.
  - `spice.node_name(net)` and the net checks use one canonical net per merged group: `GND` if present, else the alphabetically first name.

- [ ] **Step 1: Write the failing tests**

Append to `hardware/tests/test_parts.py`:

```python
def test_resistor_and_capacitor_follow_variation():
    ckt = Circuit()
    with ckt:
        r = parts.resistor("1k")
        c = parts.capacitor("100p")
    assert r.spice_lines({"1": "a", "2": "b"}, 1.1)[0].endswith(" 1100")
    assert c.spice_lines({"1": "a", "2": "b"}, 0.98)[0].endswith(" 9.8e-11")
    assert (r.tol, c.tol) == (0.01, 0.02)
```

Append to `hardware/tests/test_spice.py`:

```python
def test_variation_scales_named_parts_only():
    ckt, a, mid = divider()
    r1, r2 = sorted(ckt.parts, key=lambda p: p.ref)
    deck, _ = to_deck(ckt, "x", [Port(a, 1), Port(mid, 2)], [Op()], variation={r1.ref: 1.1})
    assert f"R{r1.ref} rf_in mid 1100" in deck and f"R{r2.ref} mid 0 2000" in deck


def test_merged_nets_share_one_spice_node():
    ckt = Circuit()
    with ckt:
        a, b, gnd = Net("a"), Net("b"), Net("GND")
        r1 = parts.resistor("1k"); r1[1] += a; r1[2] += gnd
        r2 = parts.resistor("1k"); r2[1] += b; r2[2] += gnd
        a += b
    deck, _ = to_deck(ckt, "x", [Port(a, 1)], [Op()])
    assert f"R{r1.ref} a 0 1000" in deck and f"R{r2.ref} a 0 1000" in deck


def test_sp_decade_sweep_command():
    assert Sp(0.05e6, 200e6, 50, "dec").command() == "sp dec 50 50000 2e+08"
```

Append to `hardware/tests/test_export.py`:

```python
def test_kicad_netlist_has_no_missing_tag_warnings(tmp_path, capfd):
    export.kicad_netlist(small(), tmp_path / "block.net")
    assert "Missing tag" not in capfd.readouterr().err
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_parts.py tests/test_spice.py tests/test_export.py -q`
Expected: 4 failures:
- `test_resistor_and_capacitor_follow_variation`: `TypeError` (`<lambda>() takes 1 positional argument but 2 were given`);
- `test_variation_scales_named_parts_only`: `TypeError: to_deck() got an unexpected keyword argument 'variation'`;
- `test_merged_nets_share_one_spice_node`: `SpiceError: net 'b' has a single connection (dangling part?)` (the split-node bug);
- `test_kicad_netlist_has_no_missing_tag_warnings`: the captured stderr contains `Missing tag`.

`test_sp_decade_sweep_command` already passes: `Sp` supported `dec` since 2.1. It pins the command Task 7 relies on.

- [ ] **Step 3: Implement**

In `hardware/neohw/parts.py`, add after the imports (before `_template`):

```python
def _tagged(part: Part) -> Part:
    """A stable tag (the ref) instead of SKiDL's random one: quiet, reproducible netlists."""
    part.tag = part.ref
    return part
```

Replace `resistor` with:

```python
def resistor(value, footprint: str = "Resistor_SMD:R_0805_2012Metric", mpn: str | None = None,
             torob: str | None = None, tol: float = 0.01) -> Part:
    ohms = _positive(value)
    p = _tagged(_template("R", "R", ["1", "2"], footprint)(value=format_si(ohms, "Ω")))
    p.mpn, p.torob, p.tol = mpn, torob, tol
    p.spice_lines = lambda n, k=1.0: [f"R{p.ref} {n['1']} {n['2']} {_num(ohms * k)}"]
    return p
```

Replace `capacitor` with:

```python
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
```

In `relay_dpdt`, replace

```python
    p = _template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F")
    p.mpn, p.torob, p.state = mpn, torob, state

    def lines(n):
```

with

```python
    p = _tagged(_template("G6K-2F", "K", RELAY_PINS, footprint)(value="G6K-2F"))
    p.mpn, p.torob, p.state, p.tol = mpn, torob, state, 0.0

    def lines(n, k=1.0):
```

In `hardware/neohw/spice.py`, replace `node_name` and `_check_nets` with:

```python
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
```

Replace the `to_deck` signature line

```python
def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=()) -> tuple[str, list[str]]:
    lines = [title]
```

with

```python
def to_deck(circuit, title: str, ports: list[Port], analyses: list, extra_lines=(),
            variation: dict[str, float] | None = None) -> tuple[str, list[str]]:
    """`variation` maps a part ref to a value multiplier (Monte Carlo); missing refs use 1."""
    variation = variation or {}
    lines = [title]
```

and, in its part loop, replace `lines.extend(part.spice_lines(nodes))` with
`lines.extend(part.spice_lines(nodes, variation.get(part.ref, 1.0)))`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `88 passed` (66 + 9 + 8 + 5).

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/parts.py hardware/neohw/spice.py hardware/tests/test_parts.py hardware/tests/test_spice.py hardware/tests/test_export.py
git commit -m "feat(hw): part value variation, stable SKiDL tags; fix split SPICE node on merged nets"
```

---

### Task 4: Inductor part, value-aware Torob lookup, BOM note

**Files:**
- Modify: `hardware/neohw/parts.py`, `hardware/neohw/availability.py`, `hardware/neohw/export.py`
- Test: `hardware/tests/test_parts.py`, `hardware/tests/test_availability.py` (append)

**Interfaces:**
- Consumes: Task 3 `_tagged`, `spice_lines(nodes, k)`.
- Produces:
  - `parts.INDUCTOR_FOOTPRINT = "Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2"`, `parts.INDUCTOR_C_PAR = 2.5e-12`.
  - `parts.inductor(value, q, q_freq_hz, tol=0.03, trim=1.0, mpn=None, torob=None) -> Part`. SPICE: `L<ref> n1 <ref>_m <L·trim·k>`, `R<ref> <ref>_m n2 <2π·q_freq·L·trim/q · k>`, `C<ref>_p n1 n2 2.5e-12`. `ValueError` if q or q_freq ≤ 0, or trim outside 0.9–1.1. Attributes: `tol`, `torob_match(name) -> bool`, `bom_note = "select to ±3 % with LCR meter; Q=30 assumed"` (with the actual numbers).
  - `availability.inductor_values(name) -> list[float]`, `availability.inductor_hit(name, henries) -> bool` (a named value within 1 %), `availability.lookup(key, data, match=None)`.
  - BOM rows gain `note`; rows sort naturally (R2 before R10); `write_bom` writes the `note` column.

- [ ] **Step 1: Write the failing tests**

Append to `hardware/tests/test_parts.py`:

```python
def test_inductor_model_has_q_resistor_and_srf_capacitor():
    ckt = Circuit()
    with ckt:
        ind = parts.inductor("1u", q=30, q_freq_hz=1e6)
    assert ind.value == "1uH"
    assert ind.footprint == parts.INDUCTOR_FOOTPRINT
    assert ind.spice_lines({"1": "a", "2": "b"}) == [
        f"L{ind.ref} a {ind.ref}_m 1e-06", f"R{ind.ref} {ind.ref}_m b 0.20944", f"C{ind.ref}_p a b 2.5e-12"]
    assert "LCR meter" in ind.bom_note and "Q=30" in ind.bom_note


def test_inductor_trim_and_variation_scale_l_and_r():
    ckt = Circuit()
    with ckt:
        ind = parts.inductor("1u", q=30, q_freq_hz=1e6, trim=1.05)
    lines = ind.spice_lines({"1": "a", "2": "b"}, 1.02)
    assert lines[0].endswith(f" {1e-6 * 1.05 * 1.02:.6g}")
    assert lines[1].endswith(f" {0.20944 * 1.05 * 1.02:.5g}")


def test_inductor_trim_beyond_selection_range_rejected():
    with pytest.raises(ValueError, match="trim"):
        parts.inductor("1u", q=30, q_freq_hz=1e6, trim=1.2)
```

Add `import pytest` as the first line of `hardware/tests/test_availability.py` (followed by a blank line), then append:

```python
def test_inductor_values_parsed_from_shop_titles():
    from neohw.availability import inductor_values
    assert inductor_values("سلف 470 نانو هانری سایز 1210") == [pytest.approx(470e-9)]
    assert inductor_values("سلف 4.7µH میکروهانری SMD") == [pytest.approx(4.7e-6)]
    assert inductor_values("سلف اس ام دی 10UH (1210) TDK") == [pytest.approx(10e-6)]
    assert inductor_values("سلف 270نانوهانری 270NH 0805") == [pytest.approx(270e-9)] * 2
    assert inductor_values("سلف 1.5 ميکرو هانری") == [pytest.approx(1.5e-6)]
    assert inductor_values("رله G6K-2F") == []


def test_value_match_replaces_token_match():
    from neohw.availability import inductor_hit
    data = {"سلف رنگی": {"ok": True, "hits": [{"name": "سلف رنگی 10 میکرو هانری", "instock": True},
                                              {"name": "سلف رنگی 4.7UH", "instock": False}]}}
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 10e-6)) == "available"
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 4.7e-6)) == "out_of_stock"
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 2.2e-6)) == "unknown"
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_parts.py tests/test_availability.py -q`
Expected: 5 failures: `AttributeError: module 'neohw.parts' has no attribute 'inductor'` (3) and `ImportError: cannot import name 'inductor_values'` / `'inductor_hit'` (2).

- [ ] **Step 3: Implement**

In `hardware/neohw/availability.py`, replace the `lookup` function's first lines

```python
def lookup(key: str | None, data: dict) -> str:
    if not key or key not in data:
        return "unknown"
    hits = [h for h in data[key].get("hits") or [] if _matches(key, h.get("name") or "")]
```

with

```python
_HENRY = re.compile(r"(\d+(?:\.\d+)?)\s*(nh|uh|µh|μh|mh|نانو\s*هانری|(?:میکرو|ميکرو)\s*هانری|میلی\s*هانری)",
                    re.IGNORECASE)
_HENRY_SCALE = {"n": 1e-9, "u": 1e-6, "µ": 1e-6, "μ": 1e-6, "m": 1e-3, "ن": 1e-9, "م": 1e-6}


def inductor_values(name: str) -> list[float]:
    """Inductances named in a shop title: '470 نانو هانری', '4.7µH', '10UH', '270NH', '22MH'."""
    out = []
    for number, unit in _HENRY.findall(name):
        unit = unit.lower()
        scale = 1e-3 if unit.startswith("میلی") else _HENRY_SCALE[unit[0]]
        out.append(float(number) * scale)
    return out


def inductor_hit(name: str, henries: float) -> bool:
    """A hit is this inductor if it names a value within 1 %."""
    return any(abs(v / henries - 1) <= 0.01 for v in inductor_values(name))


def lookup(key: str | None, data: dict, match=None) -> str:
    """`match(hit_name) -> bool` replaces the default part-number token match."""
    if not key or key not in data:
        return "unknown"
    match = match or (lambda name: _matches(key, name))
    hits = [h for h in data[key].get("hits") or [] if match(h.get("name") or "")]
```

In `hardware/neohw/parts.py`, add `import math` after `from __future__ import annotations` (followed by a blank line), add `from neohw.availability import inductor_hit` before `from neohw.units import …`, and append at the end of the file:

```python
# Axial colour-ring (moulded) inductors, measured and selected with an LCR meter.
INDUCTOR_FOOTPRINT = "Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm_Horizontal_Vishay_IM-2"
INDUCTOR_C_PAR = 2.5e-12  # SRF: 1 µH ≈ 100 MHz, 10 µH ≈ 32 MHz, 100 µH ≈ 10 MHz (IM-2 class minimums)


def inductor(value, q: float, q_freq_hz: float, tol: float = 0.03, trim: float = 1.0,
             mpn: str | None = None, torob: str | None = None) -> Part:
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
    p.bom_note = f"select to ±{tol * 100:g} % with LCR meter; Q={q:g} assumed"

    def lines(n, k=1.0):
        mid = f"{p.ref}_m"
        return [f"L{p.ref} {n['1']} {mid} {_num(henries_built * k)}",
                f"R{p.ref} {mid} {n['2']} {_num(r_s * k)}",
                f"C{p.ref}_p {n['1']} {n['2']} {_num(INDUCTOR_C_PAR)}"]

    p.spice_lines = lines
    return p
```

In `hardware/neohw/export.py`, add `import re` after `import html`, add before `bom_rows`:

```python
def _natural(refs: str) -> list:
    """Sort key so R2 comes before R10."""
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", refs.split()[0])]
```

In `bom_rows`, replace

```python
        row = groups.setdefault(key, {"refs": [], "value": part.value, "footprint": part.footprint,
                                      "mpn": mpn, "qty": 0,
                                      "availability": lookup(getattr(part, "torob", None), data)})
```

with

```python
        availability = lookup(getattr(part, "torob", None), data, getattr(part, "torob_match", None))
        row = groups.setdefault(key, {"refs": [], "value": part.value, "footprint": part.footprint,
                                      "mpn": mpn, "qty": 0, "availability": availability,
                                      "note": getattr(part, "bom_note", "")})
```

and `return sorted(rows, key=lambda r: r["refs"])` with `return sorted(rows, key=lambda r: _natural(r["refs"]))`.
In `write_bom`, add `"note"` as the last entry of `fieldnames`.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `93 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/parts.py hardware/neohw/availability.py hardware/neohw/export.py hardware/tests/test_parts.py hardware/tests/test_availability.py
git commit -m "feat(hw): axial inductor model with Q/SRF and selection trim; value-aware Torob lookup; BOM notes"
```

---

### Task 5: Parallel ngspice runs and Touchstone files

**Files:**
- Modify: `hardware/neohw/ngspice.py`
- Create: `hardware/neohw/touchstone.py`
- Test: `hardware/tests/test_ngspice.py` (append), `hardware/tests/test_touchstone.py`

**Interfaces:**
- Consumes: `ngspice.run_deck`.
- Produces:
  - `ngspice.run_decks(decks: list[str], rawfiles: list[str], workers: int | None = None) -> list[dict[str, list[Plot]]]`, results in input order; threads, each running its own ngspice process.
  - `touchstone.write_s2p(path, freq_hz, s: dict["11"|"21"|"12"|"22", ndarray], z0=50.0, comments=()) -> Path` (`ValueError` naming a missing parameter); `touchstone.read_s2p(path) -> (freq, s)` (`ValueError` on an option line other than `# HZ S RI R 50`).

- [ ] **Step 1: Write the failing tests**

Append to `hardware/tests/test_ngspice.py`:

```python
@pytest.mark.integration
def test_run_decks_returns_results_in_input_order():
    _need_ngspice()
    from neohw.ngspice import run_decks
    decks = [RC.replace("R1 in out 1k", f"R1 in out {r}") for r in ("1k", "2k", "4k")]
    results = run_decks(decks, ["ac.raw"], workers=3)
    corners = []
    for res in results:
        ac = res["ac.raw"][0]
        f, g = ac.data["frequency"].real, np.abs(ac.data["v(out)"])
        corners.append(f[np.argmin(np.abs(g - 2 ** -0.5))])
    assert corners[0] > corners[1] > corners[2]
```

`hardware/tests/test_touchstone.py`:

```python
import numpy as np
import pytest

from neohw.touchstone import read_s2p, write_s2p


def _s(n):
    k = np.arange(n)
    return {"11": 0.1 + 0.01j * k, "21": 0.9 - 0.02j * k, "12": 0.9 - 0.02j * k, "22": 0.2 + 0j * k}


def test_round_trip(tmp_path):
    f = np.array([1e6, 2e6, 3e6])
    p = write_s2p(tmp_path / "a.s2p", f, _s(3), comments=("hello",))
    text = p.read_text().splitlines()
    assert text[0] == "! hello" and text[1] == "# HZ S RI R 50"
    f2, s2 = read_s2p(p)
    np.testing.assert_allclose(f2, f)
    for k in ("11", "21", "12", "22"):
        np.testing.assert_allclose(s2[k], _s(3)[k], atol=1e-9)


def test_missing_parameter_rejected(tmp_path):
    s = _s(1)
    del s["22"]
    with pytest.raises(ValueError, match="22"):
        write_s2p(tmp_path / "a.s2p", np.array([1e6]), s)


def test_unsupported_option_line_rejected(tmp_path):
    p = tmp_path / "a.s2p"
    p.write_text("# MHZ S MA R 50\n1 0 0 0 0 0 0 0 0\n")
    with pytest.raises(ValueError, match="option line"):
        read_s2p(p)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_ngspice.py tests/test_touchstone.py -q`
Expected: FAIL: `ImportError: cannot import name 'run_decks'` and `ModuleNotFoundError: No module named 'neohw.touchstone'`.

- [ ] **Step 3: Implement**

In `hardware/neohw/ngspice.py`, add `import os` before `import shutil`, add `from concurrent.futures import ThreadPoolExecutor` before `from pathlib import Path`, and append:

```python
def run_decks(decks: list[str], rawfiles: list[str], workers: int | None = None) -> list[dict[str, list[Plot]]]:
    """Run independent decks in parallel (each is its own ngspice process); results in input order."""
    with ThreadPoolExecutor(max_workers=workers or os.cpu_count()) as pool:
        return list(pool.map(lambda deck: run_deck(deck, rawfiles), decks))
```

`hardware/neohw/touchstone.py`:

```python
"""Touchstone 1.1 two-port files (S-parameters, real/imaginary, 50 Ω)."""
from __future__ import annotations

from pathlib import Path

import numpy as np

ORDER = ("11", "21", "12", "22")  # Touchstone 1.x two-port column order


def write_s2p(path: Path, freq_hz: np.ndarray, s: dict[str, np.ndarray], z0: float = 50.0,
              comments: tuple[str, ...] = ()) -> Path:
    missing = [k for k in ORDER if k not in s]
    if missing:
        raise ValueError(f"missing S-parameters {missing}")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f"! {c}" for c in comments] + [f"# HZ S RI R {z0:g}"]
    for i, f in enumerate(freq_hz):
        values = " ".join(f"{s[k][i].real:.9e} {s[k][i].imag:.9e}" for k in ORDER)
        lines.append(f"{f:.9e} {values}")
    path.write_text("\n".join(lines) + "\n")
    return path


def read_s2p(path: Path) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    rows = []
    for line in Path(path).read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("!"):
            continue
        if line.startswith("#"):
            if line.upper().split() != ["#", "HZ", "S", "RI", "R", "50"]:
                raise ValueError(f"{path}: unsupported option line {line!r}")
            continue
        rows.append([float(x) for x in line.split()])
    data = np.array(rows)
    if data.ndim != 2 or data.shape[1] != 9:
        raise ValueError(f"{path}: expected 9 columns per row")
    s = {k: data[:, 1 + 2 * i] + 1j * data[:, 2 + 2 * i] for i, k in enumerate(ORDER)}
    return data[:, 0], s
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `97 passed`.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/ngspice.py hardware/neohw/touchstone.py hardware/tests/test_ngspice.py hardware/tests/test_touchstone.py
git commit -m "feat(hw): parallel ngspice runs and Touchstone two-port files"
```

---

### Task 6: Filter and bank blocks

**Files:**
- Create: `hardware/neohw/blocks/bpf.py`, `hardware/neohw/blocks/bpf_bank.py`
- Test: `hardware/tests/test_bpf_blocks.py`

**Interfaces:**
- Consumes: Task 1 `eseries.combo`, Task 2 `highpass/lowpass`, Task 3 `capacitor(note=)`, Task 4 `inductor(trim=)`, 2.1 `relay_dpdt`, `Block`.
- Produces:
  - `bpf.BANDS`, `bpf.LPF_ORDER = (9, 9, 9, 9, 9, 9, 7, 7, 7, 9)`, `bpf.HPF_ORDER = 5`, `bpf.RIPPLE_DB = 0.02`, `bpf.EDGE_MARGIN = 1.03`, `bpf.LPF_MARGIN = (1.03,) * 9 + (1.05,)`, `bpf.Q_ASSUMED = 30.0`, `bpf.L_MIN = 0.1e-6`, `bpf.C_MIN = 1e-12`.
  - `bpf.Position(kind, ideal, parts)`, `bpf.FilterDesign(index, lo_hz, hi_hz, positions)` with property `centre_hz`.
  - `bpf.design(index) -> FilterDesign` (`ValueError` outside 1..10); `bpf.add_filter(design, f_in, f_out, gnd)`; `bpf.add_switched(index, state, bus_in, bus_out, gnd)`; `bpf.build(index, state="energised") -> Block` (ports `rf_in`, `rf_out`).
  - `bpf_bank.build(selected: int | None) -> Block` (`ValueError` for an index outside 1..10); `bpf_bank.select(freq_hz) -> int`: `lo ≤ f < hi`, 54 MHz → 10, `ValueError` outside 0.1–54 MHz.
  - Net names: `f<n>_in`, `f<n>_out`, `f<n>_n<k>` (between positions), `f<n>_p<k>_s<j>` (inside an inductor pair). Nets are never merged (Review Focus 1).

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_bpf_blocks.py`:

```python
import pytest

from neohw.blocks import bpf, bpf_bank
from neohw.spice import Op, Port, to_deck


@pytest.mark.parametrize("index", range(1, 11))
def test_design_has_hpf_then_lpf_with_buyable_parts(index):
    d = bpf.design(index)
    kinds = [p.kind for p in d.positions]
    assert len(kinds) == bpf.HPF_ORDER + bpf.LPF_ORDER[index - 1]
    assert kinds[:2] == ["C_series", "L_shunt"] and kinds[5:7] == ["C_shunt", "L_series"]
    for pos in d.positions:
        assert 1 <= len(pos.parts) <= 2
        assert sum(pos.parts) == pytest.approx(pos.ideal, rel=0.10)  # a selected pair reaches it
        if pos.kind.startswith("L"):
            assert min(pos.parts) >= bpf.L_MIN


def test_design_rejects_bad_index():
    with pytest.raises(ValueError):
        bpf.design(11)


def test_bank_has_one_relay_per_filter_and_valid_deck():
    blk = bpf_bank.build(3)
    relays = [p for p in blk.circuit.parts if p.ref.startswith("K")]
    assert len(relays) == 10 and sum(p.state == "energised" for p in relays) == 1
    deck, _ = to_deck(blk.circuit, "t", [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)], [Op()])
    assert "f3_in" in deck and "f10_out" in deck


@pytest.mark.parametrize("freq, index", [(0.1e6, 1), (0.499e6, 1), (0.5e6, 2), (14.074e6, 7),
                                         (50.313e6, 10), (54e6, 10)])
def test_selection_rule(freq, index):
    assert bpf_bank.select(freq) == index


@pytest.mark.parametrize("freq", [0.05e6, 54.1e6])
def test_selection_outside_range_rejected(freq):
    with pytest.raises(ValueError):
        bpf_bank.select(freq)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_bpf_blocks.py -q`
Expected: FAIL with `ImportError: cannot import name 'bpf' from 'neohw.blocks'`.

- [ ] **Step 3: Implement**

`hardware/neohw/blocks/bpf.py`:

```python
"""One band-pass filter of the bank: Chebyshev HPF(5) + LPF(7/9), switched by a G6K-2F (spec §4)."""
from __future__ import annotations

import math
from dataclasses import dataclass

from skidl import Circuit, Net

from neohw import eseries, parts
from neohw.blocks import Block
from neohw.synth.filters import Element, highpass, lowpass

# Parent spec §4 table; filter n covers BANDS[n - 1] (Hz).
BANDS = ((0.1e6, 0.5e6), (0.5e6, 1.7e6), (1.7e6, 2.5e6), (2.5e6, 4.5e6), (4.5e6, 7.5e6),
         (7.5e6, 12e6), (12e6, 16e6), (16e6, 22e6), (22e6, 30e6), (30e6, 54e6))
LPF_ORDER = (9, 9, 9, 9, 9, 9, 7, 7, 7, 9)  # 7 where rejection at 3x has > 5 dB to spare
HPF_ORDER = 5
RIPPLE_DB = 0.02
EDGE_MARGIN = 1.03   # HPF cutoff 3 % below the band's lower edge
LPF_MARGIN = (1.03,) * 9 + (1.05,)   # LPF cutoff above the upper edge; filter 10 trades
                                     # 2 dB of its spare rejection for in-band loss
Q_ASSUMED = 30.0
L_MIN = 0.1e-6       # smallest axial colour-ring inductor commonly sold
C_MIN = 1e-12        # smallest E24 C0G 0805


@dataclass(frozen=True)
class Position:
    """One ladder position realised with 1–2 preferred-value parts."""
    kind: str
    ideal: float
    parts: tuple[float, ...]


@dataclass(frozen=True)
class FilterDesign:
    index: int
    lo_hz: float
    hi_hz: float
    positions: tuple[Position, ...]   # HPF positions then LPF positions, source to load

    @property
    def centre_hz(self) -> float:
        return math.sqrt(self.lo_hz * self.hi_hz)


def _realise(e: Element) -> Position:
    if e.kind.startswith("L"):
        return Position(e.kind, e.value, eseries.combo(e.value, eseries.E12, minimum=L_MIN))
    return Position(e.kind, e.value, eseries.combo(e.value, eseries.E24, minimum=C_MIN))


def design(index: int) -> FilterDesign:
    if not 1 <= index <= len(BANDS):
        raise ValueError(f"filter index must be 1..{len(BANDS)}, got {index}")
    lo, hi = BANDS[index - 1]
    elements = (highpass(lo / EDGE_MARGIN, HPF_ORDER, RIPPLE_DB)
                + lowpass(hi * LPF_MARGIN[index - 1], LPF_ORDER[index - 1], RIPPLE_DB))
    return FilterDesign(index, lo, hi, tuple(_realise(e) for e in elements))


def _inductors(pos: Position, a: Net, b: Net, prefix: str, d: FilterDesign) -> None:
    """Series chain of 1–2 inductors from a to b, selected so their sum is pos.ideal."""
    values = pos.parts
    nodes = [a] + [Net(f"{prefix}_s{k}") for k in range(1, len(values))] + [b]
    trim = pos.ideal / sum(values)
    for k, v in enumerate(values):
        ind = parts.inductor(v, Q_ASSUMED, d.centre_hz, trim=trim)
        ind[1] += nodes[k]
        ind[2] += nodes[k + 1]


def _capacitors(values, a: Net, b: Net) -> None:
    """1–2 C0G capacitors in parallel between a and b."""
    for v in values:
        c = parts.capacitor(v, note="C0G/NP0 ±2 %")
        c[1] += a
        c[2] += b


def add_filter(d: FilterDesign, f_in: Net, f_out: Net, gnd: Net) -> None:
    """Add the filter's parts to the current circuit between f_in and f_out.

    The last series position ends on f_out itself (nets are never merged: SKiDL keeps
    merged nets as separate objects, which would split the SPICE node).
    """
    series_at = [k for k, pos in enumerate(d.positions) if pos.kind in ("C_series", "L_series")]
    node = f_in
    for k, pos in enumerate(d.positions):
        prefix = f"f{d.index}_p{k + 1}"
        if k in series_at:
            nxt = f_out if k == series_at[-1] else Net(f"f{d.index}_n{k + 1}")
            if pos.kind == "C_series":
                _capacitors(pos.parts, node, nxt)
            else:
                _inductors(pos, node, nxt, prefix, d)
            node = nxt
        elif pos.kind == "C_shunt":
            _capacitors(pos.parts, node, gnd)
        else:
            _inductors(pos, node, gnd, prefix, d)


def add_switched(index: int, state: str, bus_in: Net, bus_out: Net, gnd: Net) -> None:
    """Filter plus its relay. Energised: COM-NO joins the filter ends to the buses.
    Released: COM-NC grounds the filter ends and the buses see only open contacts."""
    f_in, f_out = Net(f"f{index}_in"), Net(f"f{index}_out")
    k = parts.relay_dpdt(state)
    k["COM1"] += f_in
    k["NO1"] += bus_in
    k["NC1"] += gnd
    k["COM2"] += f_out
    k["NO2"] += bus_out
    k["NC2"] += gnd
    add_filter(design(index), f_in, f_out, gnd)


def build(index: int, state: str = "energised") -> Block:
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        add_switched(index, state, rf_in, rf_out, gnd)
    return Block(f"bpf{index}", ckt, {"rf_in": rf_in, "rf_out": rf_out})
```

`hardware/neohw/blocks/bpf_bank.py`:

```python
"""The 10-filter bank on common input/output buses (parent spec §4.5)."""
from __future__ import annotations

from skidl import Circuit, Net

from neohw.blocks import Block, bpf


def build(selected: int | None) -> Block:
    """`selected` is the one energised filter, or None for the muted safe state."""
    if selected is not None and not 1 <= selected <= len(bpf.BANDS):
        raise ValueError(f"selected must be 1..{len(bpf.BANDS)} or None, got {selected}")
    ckt = Circuit()
    with ckt:
        rf_in, rf_out, gnd = Net("rf_in"), Net("rf_out"), Net("GND")
        for index in range(1, len(bpf.BANDS) + 1):
            bpf.add_switched(index, "energised" if index == selected else "released", rf_in, rf_out, gnd)
    return Block("bpf_bank", ckt, {"rf_in": rf_in, "rf_out": rf_out})


def select(freq_hz: float) -> int:
    """Filter index for an LO frequency: lo <= f < hi, and 54 MHz itself -> filter 10."""
    for index, (lo, hi) in enumerate(bpf.BANDS, 1):
        if lo <= freq_hz < hi or (index == len(bpf.BANDS) and freq_hz == hi):
            return index
    raise ValueError(f"{freq_hz} Hz is outside the bank's 0.1–54 MHz range")
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd hardware && python3 -m pytest -q`
Expected: `117 passed`. The bank has 249 parts: 10 relays, 106 inductors and 133 capacitors.

- [ ] **Step 5: Commit**

```bash
git add hardware/neohw/blocks/bpf.py hardware/neohw/blocks/bpf_bank.py hardware/tests/test_bpf_blocks.py
git commit -m "feat(hw): band-pass filter and 10-filter bank blocks with selection rule"
```

---

### Task 7: Bank check, characterisation files, report axis and `make hw`

**Files:**
- Create: `hardware/neohw/checks/bpf.py`
- Modify: `hardware/neohw/checks/__init__.py`, `hardware/neohw/report.py`, `hardware/neohw/__main__.py`
- Create (generated, committed): `hardware/characterized/bpf_1.s2p` … `bpf_10.s2p`, `hardware/characterized/bpf_bank.json`; regenerate `hardware/characterized/attenuator.json`
- Test: `hardware/tests/test_bpf_check.py`, `hardware/tests/test_report.py` (append)

**Interfaces:**
- Consumes: Task 3 `to_deck(variation=)`, Task 5 `run_decks`, `write_s2p`, Task 6 blocks, 2.1 `charfile`, `export`, `availability`.
- Produces:
  - `checks.BlockResult.logx: bool = False`; the report draws a log frequency axis when set.
  - `checks.bpf`: `MC_RUNS = 200`, `MC_MIN_PCT = 95.0`, `RL_MIN_DB = 14.0`, `REJ_MIN_DB = 50.0`, `MUTE_MIN_DB = 40.0`, `S2P_SWEEP = Sp(0.05e6, 210e6, 50, "dec")`, `SELECTION_RULE`.
  - `il_limit_db(index)`, `has_rejection_target(index)` (true where 3·lo > hi).
  - `evaluate(index, mc_runs) -> {"nominal": (il, rl, rej), "mc_runs", "mc_passed", "s2p": plot data}`.
  - `mute_isolation_db() -> float`; `build_sheet(index) -> list[{"position", "kind", "target_h", "buy_h"}]`.
  - `run(out_dir, char_dir, mc_runs=MC_RUNS, filters=None) -> BlockResult("bpf_bank")`. Checks per filter n: `bpf<n>_il_db`, `bpf<n>_rl_db`, `bpf<n>_rej3f_db` (filters 3–10), `bpf<n>_mc_pass_pct` (unit `%`); then `bank_mute_isolation_db`.
  - `__main__.CHECKS = {"attenuator": …, "bpf_bank": bpf_check.run}`.

- [ ] **Step 1: Write the failing tests**

`hardware/tests/test_bpf_check.py`:

```python
import json

import pytest

from neohw.checks import bpf as check
from neohw.tools import ToolNotFoundError, tool_path
from neohw.touchstone import read_s2p


def _need_tools():
    try:
        tool_path("ngspice"), tool_path("dot")
    except ToolNotFoundError as e:
        pytest.skip(str(e))


def test_rejection_target_only_where_3f_is_out_of_band():
    assert [check.has_rejection_target(i) for i in range(1, 11)] == [False, False] + [True] * 8


@pytest.mark.integration
def test_filter_8_meets_targets_with_monte_carlo():
    _need_tools()
    r = check.evaluate(8, mc_runs=10)
    il, rl, rej = r["nominal"]
    assert il <= 2.0 and rl >= 14.0 and rej >= 50.0
    assert r["mc_passed"] == 10


@pytest.mark.integration
def test_muted_bank_isolates():
    _need_tools()
    assert check.mute_isolation_db() >= check.MUTE_MIN_DB


@pytest.mark.integration
def test_run_writes_s2p_and_bank_file(tmp_path):
    _need_tools()
    res = check.run(tmp_path / "out", tmp_path / "char", mc_runs=4, filters=[8])
    f, s = read_s2p(tmp_path / "char" / "bpf_8.s2p")
    assert f[0] == pytest.approx(0.05e6) and f[-1] >= 200e6
    doc = json.loads((tmp_path / "char" / "bpf_bank.json").read_text())
    (entry,) = doc["filters"]
    assert entry["index"] == 8 and entry["s2p"] == "bpf_8.s2p" and entry["inductors"]
    assert doc["selection_rule"] == check.SELECTION_RULE
    assert {"bpf8_il_db", "bpf8_rl_db", "bpf8_rej3f_db", "bpf8_mc_pass_pct",
            "bank_mute_isolation_db"} == {c.name for c in res.checks}
    for key in ("kicad_netlist", "bom", "svg"):
        assert res.artefacts[key].exists()


@pytest.mark.integration
def test_monte_carlo_is_reproducible():
    _need_tools()
    a, b = check.evaluate(9, mc_runs=5), check.evaluate(9, mc_runs=5)
    assert a["nominal"] == b["nominal"] and a["mc_passed"] == b["mc_passed"]
```

Append to `hardware/tests/test_report.py`:

```python
def test_report_with_log_axis(tmp_path):
    res = BlockResult("bpf_bank", [CheckResult("x", 1.0, "≤ 2", True)],
                      plots={"1": (np.array([1e5, 1e6, 1e7]), np.array([-60.0, -1.0, -60.0]))}, logx=True)
    assert "data:image/png;base64," in write_report([res], tmp_path / "r.html").read_text()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd hardware && python3 -m pytest tests/test_bpf_check.py tests/test_report.py -q`
Expected: FAIL: `ImportError: cannot import name 'bpf' from 'neohw.checks'` and `TypeError: BlockResult.__init__() got an unexpected keyword argument 'logx'`.

- [ ] **Step 3: Implement**

In `hardware/neohw/checks/__init__.py`, add the field `logx: bool = False` after `artefacts` in `BlockResult`.

In `hardware/neohw/report.py`, replace

```python
def _plot_png(plots: dict) -> str:
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for label, (x, y) in plots.items():
        ax.plot(x / 1e6, y, label=label)
    ax.set_xlabel("frequency (MHz)")
```

with

```python
def _plot_png(plots: dict, logx: bool = False) -> str:
    fig, ax = plt.subplots(figsize=(9, 3.2))
    for label, (x, y) in plots.items():
        ax.plot(x / 1e6, y, label=label)
    if logx:
        ax.set_xscale("log")
    ax.set_xlabel("frequency (MHz)")
```

and `_plot_png(r.plots)` with `_plot_png(r.plots, r.logx)`.

In `hardware/neohw/__main__.py`, add `from neohw.checks import bpf as bpf_check` after the attenuator import and set
`CHECKS = {"attenuator": attenuator_check.run, "bpf_bank": bpf_check.run}`.

`hardware/neohw/checks/bpf.py`:

```python
"""BPF bank check (spec §4): per-filter IL, RL, 3f rejection and Monte Carlo; muted state; s2p files."""
from __future__ import annotations

from pathlib import Path

import numpy as np

from neohw import availability, charfile, export
from neohw.blocks import bpf, bpf_bank
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.spice import Port, Sp, to_deck
from neohw.touchstone import write_s2p
from neohw.tools import git_describe

MC_RUNS = 200
MC_MIN_PCT = 95.0
RL_MIN_DB = 14.0
REJ_MIN_DB = 50.0
MUTE_MIN_DB = 40.0
S2P_SWEEP = Sp(0.05e6, 210e6, 50, "dec")  # decade grid: last point 208 MHz, spec asks for 200
SELECTION_RULE = "filter n with lo_n <= f < hi_n; f = 54 MHz -> 10"


def il_limit_db(index: int) -> float:
    """Spec §2 amendment: 3 dB on filters 1–7 (preamp bypassed), 2 dB on 8–10."""
    return 3.0 if index <= 7 else 2.0


def has_rejection_target(index: int) -> bool:
    """For filters 1–2, 3x the lower edge lies inside the passband."""
    lo, hi = bpf.BANDS[index - 1]
    return 3 * lo > hi


def _sweeps(index: int) -> list[Sp]:
    lo, hi = bpf.BANDS[index - 1]
    return [Sp(lo, hi, 101, "lin"), Sp(3 * lo, 3 * hi, 61, "lin")]


def _metrics(plots) -> tuple[float, float, float]:
    """(worst in-band IL, worst in-band RL, worst rejection over 3x band), all in dB."""
    band, third = plots[0].data, plots[1].data
    return (float(-db(band["s_2_1"]).min()), float(-db(band["s_1_1"]).max()),
            float(-db(third["s_2_1"]).max()))


def _passes(index: int, il: float, rl: float, rej: float) -> bool:
    return (il <= il_limit_db(index) and rl >= RL_MIN_DB
            and (not has_rejection_target(index) or rej >= REJ_MIN_DB))


def _variation(circuit, rng) -> dict[str, float]:
    return {p.ref: 1 + rng.uniform(-p.tol, p.tol) for p in sorted(circuit.parts, key=lambda p: p.ref)
            if getattr(p, "tol", 0) > 0}


def _ports(blk) -> list[Port]:
    return [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)]


def evaluate(index: int, mc_runs: int) -> dict:
    """Nominal metrics, Monte Carlo pass count and the 0.05–200 MHz S-parameters of one filter
    selected in the full bank."""
    blk = bpf_bank.build(index)
    sweeps = _sweeps(index)
    decks = [to_deck(blk.circuit, f"bpf{index} nominal", _ports(blk), sweeps)]
    for run in range(mc_runs):
        rng = np.random.default_rng([index, run])
        decks.append(to_deck(blk.circuit, f"bpf{index} mc{run}", _ports(blk), sweeps,
                             variation=_variation(blk.circuit, rng)))
    decks.append(to_deck(blk.circuit, f"bpf{index} s2p", _ports(blk), [S2P_SWEEP]))
    raws = decks[0][1]
    results = run_decks([d for d, _ in decks[:-1]], raws)
    (s2p_plot,) = run_decks([decks[-1][0]], decks[-1][1])[0]["a0.raw"]
    nominal = _metrics([results[0][r][0] for r in raws])
    mc = [_metrics([res[r][0] for r in raws]) for res in results[1:]]
    return {"nominal": nominal, "mc_runs": mc_runs,
            "mc_passed": sum(1 for m in mc if _passes(index, *m)), "s2p": s2p_plot.data}


def mute_isolation_db() -> float:
    """Worst S21 over 0.1–54 MHz with every relay released (the safe state)."""
    blk = bpf_bank.build(None)
    deck, raws = to_deck(blk.circuit, "bank muted", _ports(blk), [Sp(0.1e6, 54e6, 50, "dec")])
    (plot,) = run_decks([deck], raws)[0][raws[0]]
    return float(-db(plot.data["s_2_1"]).max())


def build_sheet(index: int) -> list[dict]:
    """What to buy and what to select, per inductor position (source to load)."""
    return [{"position": k, "kind": pos.kind, "target_h": pos.ideal, "buy_h": list(pos.parts)}
            for k, pos in enumerate(bpf.design(index).positions, 1) if pos.kind.startswith("L")]


def run(out_dir: Path, char_dir: Path, mc_runs: int = MC_RUNS, filters=None) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    indices = list(filters or range(1, len(bpf.BANDS) + 1))
    checks, plots, entries = [], {}, []
    revision = git_describe()
    for index in indices:
        lo, hi = bpf.BANDS[index - 1]
        r = evaluate(index, mc_runs)
        il, rl, rej = r["nominal"]
        pct = 100.0 * r["mc_passed"] / mc_runs if mc_runs else 0.0
        checks += [
            CheckResult(f"bpf{index}_il_db", il, f"≤ {il_limit_db(index):g}", il <= il_limit_db(index)),
            CheckResult(f"bpf{index}_rl_db", rl, f"≥ {RL_MIN_DB:g}", rl >= RL_MIN_DB),
        ]
        if has_rejection_target(index):
            checks.append(CheckResult(f"bpf{index}_rej3f_db", rej, f"≥ {REJ_MIN_DB:g}", rej >= REJ_MIN_DB))
        checks.append(CheckResult(f"bpf{index}_mc_pass_pct", pct, f"≥ {MC_MIN_PCT:g}", pct >= MC_MIN_PCT, "%"))
        s = r["s2p"]
        f = s["frequency"].real
        s2p = write_s2p(char_dir / f"bpf_{index}.s2p", f,
                        {k: s[f"s_{k[0]}_{k[1]}"] for k in ("11", "21", "12", "22")},
                        comments=(f"NeoRadio BPF {index}: {lo / 1e6:g}-{hi / 1e6:g} MHz, selected in the bank",
                                  f"generated_by {revision}"))
        plots[f"{index}: {lo / 1e6:g}–{hi / 1e6:g} MHz"] = (f, db(s["s_2_1"]))
        entries.append({"index": index, "lo_hz": lo, "hi_hz": hi, "hpf_order": bpf.HPF_ORDER,
                        "lpf_order": bpf.LPF_ORDER[index - 1], "il_db": il, "rl_db": rl,
                        "rej3f_db": rej if has_rejection_target(index) else None,
                        "mc_runs": mc_runs, "mc_passed": r["mc_passed"], "s2p": s2p.name,
                        "inductors": build_sheet(index)})
    mute = mute_isolation_db()
    checks.append(CheckResult("bank_mute_isolation_db", mute, f"≥ {MUTE_MIN_DB:g}", mute >= MUTE_MIN_DB))
    result = BlockResult("bpf_bank", checks, plots=plots, logx=True)
    bank = bpf_bank.build(None)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(bank.circuit, out_dir / "bpf_bank.net"),
        "bom": export.write_bom(bank.circuit, out_dir / "bpf_bank_bom.csv", data),
        "svg": export.svg_diagram(bank.circuit, out_dir / "bpf_bank.svg"),
        "characterisation": charfile.write(
            char_dir / "bpf_bank.json", "bpf_bank", "pass" if result.passed else "fail",
            {"ripple_db": bpf.RIPPLE_DB, "q_assumed": bpf.Q_ASSUMED, "selection_rule": SELECTION_RULE,
             "filters": entries, "checks": [c.__dict__ for c in checks]}),
    }
    return result
```

- [ ] **Step 4: Run the tests**

Run: `cd hardware && python3 -m pytest -q`
Expected: `123 passed`.

- [ ] **Step 5: Commit the code, then generate and commit the characterisation files**

The characterisation files name the last commit of `hardware/neohw`, so the code is committed first.

```bash
git add hardware/neohw/checks hardware/neohw/report.py hardware/neohw/__main__.py hardware/tests/test_bpf_check.py hardware/tests/test_report.py
git commit -m "feat(hw): BPF bank check with Monte Carlo, s2p and bpf_bank.json characterisation"
```

Run: `make hw`
Expected (about 95 s on 20 cores): prints `PASS  attenuator`, `PASS  bpf_bank` and `report: ../build/hw/report.html`. `hardware/characterized/` gains `bpf_1.s2p` … `bpf_10.s2p` and `bpf_bank.json` with `"status": "pass"`. Reference values from the prototype run (nominal IL / RL / rejection, MC passes of 200):

| Filter | IL dB | RL dB | Rej dB | MC |
|---|---|---|---|---|
| 1 | 1.85 | 18.5 | – | 200 |
| 2 | 1.73 | 20.3 | – | 200 |
| 3 | 1.91 | 18.1 | 73.7 | 200 |
| 4 | 1.70 | 20.4 | 54.9 | 200 |
| 5 | 1.79 | 17.8 | 62.9 | 200 |
| 6 | 1.88 | 17.6 | 67.7 | 200 |
| 7 | 1.51 | 17.2 | 60.7 | 199 |
| 8 | 1.51 | 17.4 | 58.9 | 199 |
| 9 | 1.51 | 16.7 | 59.8 | 200 |
| 10 | 1.71 | 17.7 | 57.6 | 200 |

`bank_mute_isolation_db` ≈ 188 dB. That is the ideal circuit; on the board, coupling will set the real figure, and Rev 0 measures it.

```bash
git add hardware/characterized
git commit -m "chore(hw): characterise the BPF bank and regenerate attenuator from $(git log -1 --format=%h -- hardware/neohw)"
```

Run: `make hw`, then `git status --short`
Expected: empty output (the same commit regenerates identical files, spec §8).

Run: `make test`
Expected: the .NET (43), sim (60) and hardware (123) suites all pass.
