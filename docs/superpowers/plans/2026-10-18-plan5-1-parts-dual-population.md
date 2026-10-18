# Plan 5.1: Parts and Dual Population Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the Torob workarounds with better parts (the China population) while keeping the Iranian parts as a fallback population on the same PCB, identify every part generically, and prove both populations meet every existing simulation target.

**Architecture:** A part may carry `variants` (one per population: China, Iran; `None` = not fitted in that population). SPICE decks, Monte Carlo, BOMs and the net check take a `population` argument (default `"china"`), so existing checks run the China population unchanged and the affected checks also run Iran. Parts that differ only in model share one footprint (one part, two variants); parts that differ in circuit are separate parts, each fitted in one population. The firmware learns which bias-sense circuit and which Si5351 reference input is fitted by measuring, not by configuration.

**Tech Stack:** Python 3 (SKiDL 2.3, ngspice 45, numpy, pytest), C11 firmware (CMake/Ninja, ctest), KiCad 9 footprint files, ffmpeg (GIF).

**Spec:** `docs/superpowers/specs/2026-10-18-rev-a-china-fab-design.md` (§0, §2, §2a, §2b, §6, §7a; §7b for the site pages)

## Global Constraints

- Populations are exactly `("china", "iran")`; China is the default everywhere and the one that feeds the characterisation files the twin and firmware read.
- **Generic identification (§2b):** active parts, relays, crystals, oscillators, connectors and protection parts carry a manufacturer part number; passives carry a full specification (capacitor: value, tolerance, package, dielectric, voltage; resistor: value, tolerance, package; inductor: value, tolerance, package, construction, minimum Q at a frequency, minimum SRF, current). No distributor catalogue number (`C` + digits, LCSC, JLCPCB, Torob key) may appear in a BOM; Torob keys stay as the Iranian availability check only.
- Block targets are unchanged and apply to **both** populations: BPF IL ≤ 3 dB (filters 1–7) / ≤ 2 dB (8–11), RL ≥ 14 dB, 3f rejection ≥ 50 dB where it applies, Monte Carlo ≥ 95 % of 200 runs, bank mute ≥ 40 dB; power rails in regulation at every scenario, junction ≤ 125 °C at 50 °C ambient; bias sense monotonic and ≤ 3.1 V at the ADC. A target a population cannot meet is reported with numbers and decided by the user, never loosened silently.
- The relay bit map, I2C map and GPIO functions do not change (the GPIO pads change in plan 5.2, not here).
- LCSC/JLC: not used (spec §2b). The `china-parts` branch is a source of part choices only.
- Per-plan verification: `make hw-fresh`, then `make hw` with identical outputs; `make test`; `make sim`; `make fw-test`.
- dotnet steps keep the Makefile's retry wrapper; heavy jobs check RAM first (`free -g`).
- Every task that changes a block updates its site page (`site/docs/blocks/<page>.md`, plan 5.0) in the same commit, if plan 5.0 has been executed; otherwise note the change in the plan's final task list for 5.0.

## Review Focus

1. **A net that only unfitted parts touch** (e.g. the mirror's internal nodes in the China population): the SPICE net check must ignore it instead of reporting a dangling net, and must still report a genuinely dangling net among fitted parts (Task 1 test).
2. **Monte Carlo tolerance per population:** a part's variation must use the fitted variant's tolerance (China inductor ±5 %, Iran ±3 % after selection), not the part's default (Task 1 test, used in Task 3).
3. **Bias-sense detection at the threshold:** a zero reading near 300 mV, or a board with neither circuit fitted (reading 0 V and never rising), must not select the wrong table silently: the firmware reports a sense fault when the zero is between 150 and 450 mV (Task 4 test).
4. **Si5351 reference detection with the TCXO slow to start:** the firmware must wait for the status register's CLKIN loss-of-signal flag to settle (poll up to 10 ms) before choosing the crystal path (Task 6 test with a delayed CLKIN in the virtual Si5351).
5. **An LDO variant swap that changes the regulator's dropout or θJA:** the power check must use the fitted variant's dropout, PSRR and θJA for each population, not the China values for both (Task 5 test).

---

### Task 1: The population model in SPICE

**Files:**
- Create: `hardware/neohw/population.py`
- Modify: `hardware/neohw/spice.py` (`_part_lines`, `_check_nets`, `to_deck`, the cascade helpers that call them), `hardware/neohw/checks/bpf.py` (`_variation`)
- Test: `hardware/tests/test_population.py`

**Interfaces:**
- Produces:
  - `population.POPULATIONS = ("china", "iran")`, `population.DEFAULT = "china"`
  - `@dataclass(frozen=True) class Variant: value: str; ident: str; note: str = ""; spice_lines: Callable | None = None; tol: float | None = None; torob: str | None = None; torob_match: Callable | None = None`. `ident` is the generic identification (MPN or specification text, §2b).
  - `population.set_variants(part, china: Variant | None, iran: Variant | None) -> Part`
  - `population.only(part, pop: str) -> Part`: the part is fitted only in `pop`, with its own value, `mpn`/spec and model.
  - `population.fitted(part, pop: str) -> bool`; `population.variant(part, pop: str) -> Variant | None` (None for parts without variants)
  - `population.tol(part, pop) -> float`: the variant's tolerance if it sets one, else `part.tol`
  - `spice.to_deck(..., population: str = "china")`; unfitted parts emit no lines; a fitted part with a variant `spice_lines` uses it.

- [ ] **Step 1: Write the failing tests**

```python
# hardware/tests/test_population.py
import pytest
from skidl import Circuit, Net

from neohw import parts, population as pop
from neohw.spice import Port, Sp, SpiceError, to_deck


def _divider(fit_lower_in: str | None):
    ckt = Circuit()
    with ckt:
        a, mid, gnd = Net("a"), Net("mid"), Net("GND")
        r1 = parts.resistor("1k")
        r1[1] += a
        r1[2] += mid
        r2 = parts.resistor("1k")
        r2[1] += mid
        r2[2] += gnd
        if fit_lower_in:
            pop.only(r2, fit_lower_in)
    return ckt, a, mid


def test_unfitted_part_emits_no_spice_and_its_nets_are_not_dangling():
    ckt, a, mid = _divider("iran")
    deck, _ = to_deck(ckt, "t", [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, 3, "lin")], population="china")
    assert "R" + [p for p in ckt.parts if p.value == "1kΩ"][1].ref not in deck.split()
    deck_iran, _ = to_deck(ckt, "t", [Port(a, 1), Port(mid, 2)], [Sp(1e6, 2e6, 3, "lin")], population="iran")
    assert deck_iran.count("\nR") >= 2


def test_a_really_dangling_net_is_still_reported():
    ckt = Circuit()
    with ckt:
        a, b = Net("a"), Net("b")
        r = parts.resistor("1k")
        r[1] += a
        r[2] += b
    with pytest.raises(SpiceError, match="single connection"):
        to_deck(ckt, "t", [Port(a, 1)], [Sp(1e6, 2e6, 3, "lin")])


def test_variant_model_and_tolerance_follow_the_population():
    with Circuit():
        c = parts.capacitor("100p", tol=0.02)
        pop.set_variants(c, pop.Variant("100 pF", "100 pF ±1 % C0G 0805 50 V", tol=0.01),
                         pop.Variant("100 pF", "100 pF ±2 % C0G 0805 50 V (LCR-selected)", tol=0.02))
        assert pop.tol(c, "china") == 0.01 and pop.tol(c, "iran") == 0.02
        assert pop.fitted(c, "china") and pop.fitted(c, "iran")


def test_unknown_population_is_rejected():
    with Circuit():
        r = parts.resistor("1k")
        with pytest.raises(ValueError, match="population"):
            pop.fitted(r, "germany")
```

- [ ] **Step 2: Run to see them fail**

Run: `cd hardware && python3 -m pytest -q tests/test_population.py`
Expected: FAIL (`neohw.population` missing).

- [ ] **Step 3: Implement**

```python
# hardware/neohw/population.py
"""Two populations of one PCB (Rev A spec §0, §2): China (primary, the default) and Iran (fallback).

A part without `variants` is fitted in both with its own model. `set_variants` gives a part a
per-population value, generic identification (spec §2b), model and tolerance; a `None` variant
means "not fitted" in that population. `only` fits a part in one population."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable

POPULATIONS = ("china", "iran")
DEFAULT = "china"


@dataclass(frozen=True)
class Variant:
    value: str
    ident: str                                  # manufacturer part number or full specification
    note: str = ""
    spice_lines: Callable | None = None        # None: the part's own model
    tol: float | None = None                    # None: the part's own tolerance
    torob: str | None = None                    # Iranian availability search key (never in a BOM)
    torob_match: Callable | None = None


def _check(pop: str) -> None:
    if pop not in POPULATIONS:
        raise ValueError(f"population must be one of {POPULATIONS}, got {pop!r}")


def set_variants(part, china: Variant | None, iran: Variant | None):
    part.variants = {"china": china, "iran": iran}
    return part


def only(part, pop: str):
    _check(pop)
    own = Variant(part.value, getattr(part, "mpn", None) or getattr(part, "spec", "") or part.value,
                  getattr(part, "bom_note", ""), None, None, getattr(part, "torob", None),
                  getattr(part, "torob_match", None))
    return set_variants(part, *(own if p == pop else None for p in POPULATIONS))


def variant(part, pop: str) -> Variant | None:
    _check(pop)
    v = getattr(part, "variants", None)
    return None if v is None else v[pop]


def fitted(part, pop: str) -> bool:
    _check(pop)
    v = getattr(part, "variants", None)
    return v is None or v[pop] is not None


def tol(part, pop: str) -> float:
    v = variant(part, pop)
    return v.tol if v is not None and v.tol is not None else getattr(part, "tol", 0.0)
```

In `spice.py`:
- `_part_lines(circuit, variation, overrides, node=..., population=DEFAULT)`: `if not population.fitted(part, pop): continue`; when `population.variant(part, pop)` has `spice_lines`, call it instead of `part.spice_lines`.
- `_check_nets(circuit, ports, population=DEFAULT)`: count pins only of fitted parts; skip nets whose count is 0 (only unfitted parts touch them).
- `to_deck(..., population: str = DEFAULT)` passes it to both. Find every other caller of `_part_lines` / `_check_nets` (`grep -n "_part_lines\|_check_nets" hardware/neohw/*.py`) and thread the same argument with the same default.

In `checks/bpf.py`, `_variation(circuit, rng, pop="china")` uses `population.tol(p, pop)` and only fitted parts, iterating in the same sorted-ref order (so existing seeds give existing draws for parts without variants).

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_population.py tests/test_spice.py tests/test_spice_cascade.py tests/test_bpf_check.py` — Expected: PASS (the existing tests are unchanged: no part has variants yet).

- [ ] **Step 5: Commit** `git commit -m "feat(hw): populations (China primary, Iran fallback): per-population SPICE, net check and tolerances"`

### Task 2: Generic identification and the per-population BOM

**Files:**
- Modify: `hardware/neohw/parts.py` (every factory sets `p.mpn` for active parts or `p.spec` for passives; `resistor`, `capacitor`, `inductor` build `spec` from their arguments), `hardware/neohw/export.py` (`bom_rows`, `write_bom` take `population`), `hardware/neohw/checks/board.py` (writes `board_bom_china.csv`, `board_bom_iran.csv`, `board_fitted.csv`)
- Create: `hardware/neohw/ident.py`
- Test: `hardware/tests/test_ident.py`, `hardware/tests/test_export.py`

**Interfaces:**
- Consumes: Task 1 `population`.
- Produces:
  - `ident.of(part, pop="china") -> str` (variant ident, else `part.mpn`, else `part.spec`); `ident.violations(circuit) -> list[str]` (refs whose ident is empty or matches `CATALOGUE = re.compile(r"\bC\d{4,}\b|LCSC|JLC|torob", re.I)`, or a passive whose spec lacks a tolerance or package).
  - `export.bom_rows(circuit, data, population="china") -> list[dict]` with keys `refs, qty, value, package, ident, note, availability` (availability only for `iran`, from Torob); unfitted parts excluded. `export.fitted_rows(circuit) -> list[dict]` (`ref, china, iran` with the ident or `—`).
  - Spec strings: resistor `f"{value} ±{tol*100:g} % {package}"`; capacitor `f"{value} ±{tol*100:g} % {dielectric} {package} {voltage}"` with new parameters `dielectric="C0G"`, `voltage="50 V"`; inductor `f"{value} ±{tol*100:g} % {package} {construction}, Q≥{q:g} at {f}, SRF≥{srf}, {current}"`. Package is the footprint's short name (e.g. `0805` from `C_0805_2012Metric`).

- [ ] **Step 1: Failing tests**

```python
# hardware/tests/test_ident.py
from skidl import Circuit, Net

from neohw import ident, parts, population as pop
from neohw.blocks import board


def test_passives_carry_a_full_specification():
    with Circuit():
        c = parts.capacitor("47p", tol=0.01, note="filter")
        r = parts.resistor("61.9")
        assert ident.of(c) == "47 pF ±1 % C0G 0805 50 V"
        assert ident.of(r) == "61.9 Ω ±1 % 0805"


def test_catalogue_numbers_are_rejected():
    ckt = Circuit()
    with ckt:
        u = parts.lm75a()
        u.mpn = "C21101"
        for pin in u.pins:
            pin += Net(f"n_{pin.name}")
    assert any("catalogue" in v for v in ident.violations(ckt))


def test_the_whole_board_is_identified_generically():
    assert ident.violations(board.build().circuit) == []
```

Append to `tests/test_export.py`:
```python
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
```

- [ ] **Step 2: Run to see them fail.** `cd hardware && python3 -m pytest -q tests/test_ident.py tests/test_export.py` — Expected: FAIL.

- [ ] **Step 3: Implement** `ident.py`, the `spec` strings in the three passive factories, `mpn` values in every IC/semiconductor factory that lacks one (open each with `grep -n "def \|mpn" hardware/neohw/parts.py`; use the manufacturer part numbers already in the docstrings, e.g. `LM75AD`, `SN74HC595DR`, `ULN2803ADWR`), and the BOM functions. In `checks/board.py::run`, write the three CSVs alongside the existing artefacts and add a check `ident_violations` (value = count, limit `= 0`).

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q` — Expected: PASS (fix every factory the board test names).

- [ ] **Step 5: Commit** `git commit -m "feat(hw): generic part identification (MPN or full specification) and per-population BOMs"`

### Task 3: Filter bank on surface-mount inductors, axial fallback, capacitor slots

**Files:**
- Create: `hardware/neohw/footprints.py`, `hardware/footprints/NeoRadio.pretty/L_1210_Axial_P10.16mm.kicad_mod` (generated, committed)
- Modify: `hardware/neohw/parts.py` (add `NLV32_TABLE`, `smd_inductor_variant`, combined footprint constant), `hardware/neohw/blocks/bpf.py`, `hardware/neohw/checks/bpf.py`, `hardware/neohw/checks/board.py` (project-local footprints must exist in `hardware/footprints/NeoRadio.pretty/`)
- Test: `hardware/tests/test_bpf_blocks.py`, `hardware/tests/test_bpf_check.py`, `hardware/tests/test_footprints.py`

**Interfaces:**
- Consumes: Task 1 (`set_variants`, `only`, `population=`), Task 2 (spec strings).
- Produces:
  - `parts.NLV32_TABLE` (copy it verbatim from `git show ed5be14:hardware/neohw/parts.py`: henries → (Q min, Q test Hz, SRF min Hz, DCR max Ω), TDK NLV32-EF datasheet rows 0.1–82 µH).
  - `parts.filter_inductor(value, q_freq_iran: float, trim_iran: float) -> Part`: footprint `NeoRadio:L_1210_Axial_P10.16mm` (pads `1`, `2` each with an SMD 1210 pad and a THT pad joined); China variant: `"{value} ±5 % 1210 wirewound, Q≥{q} at {f}, SRF≥{srf}, ≥ 100 mA (TDK NLV32 or equivalent)"`, SPICE from the NLV32 row (L + series R giving Q at the test frequency, C_par from SRF), tol 0.05; Iran variant: today's axial model (`Q_ASSUMED` at `q_freq_iran`, trim, tol 0.03), ident `"{value} axial moulded inductor, selected to ±3 % with an LCR meter"`, Torob key and matcher as today.
  - `bpf.CAP_TOL = {"china": 0.01, "iran": 0.02}`; `bpf.Position` gains `iran_parts: tuple[float, ...]` (the capacitor realisation from E12 values only for Iran, E24 for China; identical for inductors).
  - `checks/bpf.evaluate(index, mc_runs, population="china")`; `run()` adds per filter `bpf{n}_iran_il_db`, `_rl_db`, `_rej3f_db`, `_mc_pass_pct` and writes both populations' metrics into `bpf_bank.json` (`"populations": {"china": {...}, "iran": {...}}` per filter); the `.s2p` files stay China.

- [ ] **Step 1: Failing tests**

```python
# append to hardware/tests/test_bpf_blocks.py
from neohw import population as pop


def test_every_bank_inductor_has_an_smd_china_variant_and_an_axial_iran_variant():
    bank = bpf_bank.build(7)
    inductors = [p for p in bank.circuit.parts if p.ref.startswith("L")]
    assert inductors
    for p in inductors:
        assert p.footprint == "NeoRadio:L_1210_Axial_P10.16mm"
        china, iran = pop.variant(p, "china"), pop.variant(p, "iran")
        assert "1210 wirewound" in china.ident and china.tol == 0.05
        assert "axial" in iran.ident and iran.tol == 0.03


def test_iran_capacitors_use_e12_values_only_and_china_e24():
    from neohw import eseries
    e12 = {round(v, 15) for v in eseries.values(eseries.E12, 1e-13, 1e-6)}
    for index in range(1, 12):
        for pos in bpf.design(index).positions:
            if pos.kind.startswith("C"):
                assert all(round(v, 15) in e12 for v in pos.iran_parts), (index, pos)
```

```python
# append to hardware/tests/test_bpf_check.py
def test_filter_8_meets_targets_in_both_populations():
    for p in ("china", "iran"):
        r = check.evaluate(8, mc_runs=20, population=p)
        il, rl, rej = r["nominal"]
        assert il <= check.il_limit_db(8) and rl >= check.RL_MIN_DB, p
        assert r["mc_passed"] >= 19, p
```

```python
# append to hardware/tests/test_footprints.py
def test_combined_inductor_footprint_has_smd_and_tht_pads_on_both_terminals():
    from neohw import footprints
    text = footprints.combined_inductor().read_text()
    assert text.count('(pad "1"') == 2 and text.count('(pad "2"') == 2
    assert "smd" in text and "thru_hole" in text
```

- [ ] **Step 2: Run to see them fail.** `cd hardware && python3 -m pytest -q tests/test_bpf_blocks.py tests/test_bpf_check.py tests/test_footprints.py -k "variant or populations or e12 or combined"` — Expected: FAIL.

- [ ] **Step 3: Implement**

`footprints.py` writes KiCad 9 footprint text (s-expression) for `L_1210_Axial_P10.16mm`: the 1210 pads copied from `/usr/share/kicad/footprints/Inductor_SMD.pretty/L_1210_3225Metric.kicad_mod` (same size and position, pads 1 and 2), plus two THT pads 10.16 mm apart (drill 0.8 mm, pad 1.6 mm, positioned at x = ∓5.08 mm), silkscreen body outline 6.6 × 2.7 mm, courtyard around both. Function `combined_inductor(lib: Path = FOOTPRINT_LIB) -> Path` writes it and returns the path; `FOOTPRINT_LIB = hardware/footprints/NeoRadio.pretty`. Commit the generated file.

`bpf._realise` returns both realisations:
```python
def _realise(e: Element) -> Position:
    if e.kind.startswith("L"):
        parts_ = eseries.combo(e.value, eseries.E12, minimum=L_MIN)
        return Position(e.kind, e.value, parts_, parts_)
    return Position(e.kind, e.value, eseries.combo(e.value, eseries.E24, minimum=C_MIN),
                    eseries.combo(e.value, eseries.E12, minimum=C_MIN))
```

`_capacitors(pos, a, b)` places `max(len(pos.parts), len(pos.iran_parts))` slots; slot k gets `set_variants(c, china=Variant(value, spec at CAP_TOL["china"], tol=0.01) if k < len(pos.parts) else None, iran=... if k < len(pos.iran_parts) else None)`, each variant's `spice_lines` built with `parts.capacitor(v)`'s model for its own value (create a throwaway model function, not a second SKiDL part: factor `parts._capacitor_lines(farads, esr, esl, ref)` out of `capacitor()`).

`_inductors` uses `parts.filter_inductor(v, q_freq, trim)` for every series member; keep `select_group`/`select_target` (they describe the Iran build sheet).

`checks/bpf.py`: thread `population` through `evaluate`, `_variation`, `to_deck`; `run()` evaluates both populations; only China writes `.s2p`.

Run the bank check for one filter in both populations and read the numbers before going on: `cd hardware && python3 -c "from neohw.checks import bpf as c; [print(p, c.evaluate(i, 20, p)['nominal']) for i in (1, 2, 11) for p in ('china', 'iran')]"`. If a China filter misses IL (most likely filters 1–2, whose NLV32 Q is specified at 2.52 MHz), re-run the synthesis for that filter with the real Q (raise `RIPPLE_DB` or lower `LPF_ORDER` for that index only, as the phase 2 spec did) and record the change and the numbers in the spec's §2 table; if no setting meets the target, stop and report to the user.

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_bpf_blocks.py tests/test_bpf_check.py tests/test_footprints.py tests/test_board.py` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): filter bank on 1210 wirewound inductors (China) with axial fallback pads (Iran); E24/E12 capacitor slots; both populations checked"`

### Task 4: Bias-T current sense: INA180A1 (China) and the mirror (Iran)

**Files:**
- Modify: `hardware/neohw/parts.py` (`ina180a1()`), `hardware/neohw/blocks/bias_switch.py`, `hardware/neohw/checks/controller.py`, `firmware/tools/gen_tables.py`, `firmware/core/supervisor.{c,h}`, `firmware/host/hal_virtual.c` (bias ADC reading follows the fitted circuit), `firmware/tests/test_supervisor.c`
- Test: `hardware/tests/test_controller.py`, `firmware/tests/test_supervisor.c`

**Interfaces:**
- Produces:
  - `parts.ina180a1() -> Part`: pins `OUT 1, GND 2, IN+ 3, IN- 4, VS 5` (SOT-23-5 "DBV" pinout; **confirm against the INA180 datasheet's Pin Functions table before writing**), mpn `INA180A1IDBVR`; SPICE: `E` source `V(OUT) = clamp(20 · (V(IN+) − V(IN−)) + 0.15e-3·20, 0, V(VS) − 0.02)` through 10 Ω (behavioural; offset at its datasheet maximum, sign chosen to read high).
  - `controller.json` replaces `bias_sense_table_27c` with `"bias_sense_tables_27c": {"ina180": [...], "mirror": [...]}`, each a list of `{"i_a", "sense_v"}`; and `"bias_sense_zero_split_v": 0.3`.
  - Firmware: `TABLES_BIAS_SENSE_{INA180,MIRROR}_{MV,UA}[]`, `TABLES_BIAS_SENSE_{INA180,MIRROR}_POINTS`; `supervisor_bias_ua(mv)` uses the table chosen at the last zero measurement; new flag `SUP_FLAG_BIAS_SENSE_FAULT` when the zero is in 150–450 mV.

- [ ] **Step 1: Failing tests**

```python
# append to hardware/tests/test_controller.py
def test_both_bias_sense_circuits_are_on_the_board_one_per_population():
    from neohw import population as pop
    from neohw.blocks import bias_switch
    blk = bias_switch.build()
    ina = [p for p in blk.circuit.parts if p.value == "INA180A1"]
    mirror = [p for p in blk.circuit.parts if p.value == "BC857B"]
    assert len(ina) == 1 and len(mirror) == 2
    assert pop.fitted(ina[0], "china") and not pop.fitted(ina[0], "iran")
    assert all(pop.fitted(q, "iran") and not pop.fitted(q, "china") for q in mirror)


def test_ina180_sense_reads_3v_at_the_cutoff_and_never_exceeds_the_adc_range(ngspice):
    from neohw.checks import controller as c
    bs = c.bias_sense("china")
    tab = bs["table"][27.0]
    near150 = min(tab, key=lambda iv: abs(iv[0] - 0.15))
    assert 2.85 <= near150[1] <= 3.15
    assert max(v for t in bs["table"].values() for _, v in t) <= 3.3
    assert tab[0][1] < 0.05                         # zero reads ≈ 0 V: the firmware tells it from the mirror
```

```c
/* append to firmware/tests/test_supervisor.c, and RUN() it */
static void bias_sense_table_follows_the_zero_reading(void)
{
    /* mirror zero ≈ 630 mV picks the mirror table; INA180 zero ≈ 0 mV picks the INA180 table */
    supervisor_init();
    supervisor_set_bias_zero_mv(630);
    CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_MIRROR_MV[3]) == TABLES_BIAS_SENSE_MIRROR_UA[3]);
    supervisor_set_bias_zero_mv(2);
    CHECK(supervisor_bias_ua(TABLES_BIAS_SENSE_INA180_MV[3] + 2) == TABLES_BIAS_SENSE_INA180_UA[3]);
    supervisor_set_bias_zero_mv(300);
    CHECK(supervisor_poll()->flags & SUP_FLAG_BIAS_SENSE_FAULT);
}
```
(`supervisor_set_bias_zero_mv` is a test hook declared in `supervisor.h` under `#ifdef ELF_HOST_TEST`; the real zero comes from the existing measurement with the feed off. Check how the existing test sets the zero (`grep -n zero firmware/tests/test_supervisor.c`) and reuse that path if one exists instead of adding the hook.)

- [ ] **Step 2: Run to see them fail.** `cd hardware && python3 -m pytest -q tests/test_controller.py` and `make fw-test` — Expected: FAIL.

- [ ] **Step 3: Implement.** In `bias_switch.build`: keep the shunt and SY6280; mark the mirror's parts (2 × BC857B, its 100 Ω ×2, 4.7 kΩ, 680 Ω) with `population.only(p, "iran")`; add the INA180A1 (`IN+` on `+5V_BIAS`, `IN−` on `bias_shunt_lo`, `VS` on a new port `v3d` → `+3V3_D`, `GND`, `OUT` on `bias_isense_raw`) with a 100 nF X7R on VS, all `only(…, "china")`. The 10 kΩ + 100 nF ADC filter stays common. `checks/controller.bias_sense(population)` runs per population; `run()` writes both tables and checks `bias_sense_*` per population (`bias_sense_iran_*` for the mirror). Update the module docstring. `gen_tables.py` emits both tables; `supervisor.c` stores the chosen table pointer at zero measurement:

```c
static const uint32_t *tab_mv = TABLES_BIAS_SENSE_INA180_MV, *tab_ua = TABLES_BIAS_SENSE_INA180_UA;
static int tab_n = TABLES_BIAS_SENSE_INA180_POINTS;

static void choose_table(uint32_t zero)
{
    bool mirror = zero >= TABLES_BIAS_ZERO_SPLIT_MV;
    tab_mv = mirror ? TABLES_BIAS_SENSE_MIRROR_MV : TABLES_BIAS_SENSE_INA180_MV;
    tab_ua = mirror ? TABLES_BIAS_SENSE_MIRROR_UA : TABLES_BIAS_SENSE_INA180_UA;
    tab_n = mirror ? TABLES_BIAS_SENSE_MIRROR_POINTS : TABLES_BIAS_SENSE_INA180_POINTS;
    if (zero >= 150 && zero <= 450) st.flags |= SUP_FLAG_BIAS_SENSE_FAULT;
    else st.flags &= ~SUP_FLAG_BIAS_SENSE_FAULT;
}
```
`referred_mv` and `supervisor_bias_ua` use `tab_mv/tab_ua/tab_n`. A sense fault refuses `supervisor_bias_enable(true)`. `hal_virtual.c` produces the bias ADC reading from the INA180 table by default (China), from the mirror table when the virtual board's state file says `"population": "iran"`.

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_controller.py && cd .. && make fw-test` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "feat: INA180A1 bias-T sense (China) beside the BC857B mirror (Iran); firmware picks the table from the zero reading"`

### Task 5: Regulators: TPS7A20 / MIC5205 on shared pads, TLV75733 / AMS1117

**Files:**
- Modify: `hardware/neohw/parts.py` (`tps7a20(v)`, `tlv75733()`, LDO metadata `dropout_v`, `psrr`, `theta_ja` per part), `hardware/neohw/blocks/power.py`, `hardware/neohw/checks/power.py`
- Test: `hardware/tests/test_power.py`

**Interfaces:**
- Produces: `parts.tps7a20(v_out)` (SOT-23-5: `IN 1, GND 2, EN 3, NC 4, OUT 5`, mpn `TPS7A2050PDBVR` / `TPS7A2033PDBVR`; **confirm pin 4 is "NC, no internal connection" in the TPS7A20 datasheet**), `parts.tlv75733()` (`IN 1, GND 2, EN 3, NC 4, OUT 5`, mpn `TLV75733PDBVR`; confirm). Each LDO part carries `p.dropout_v`, `p.theta_ja`, `p.psrr_db(f)`. `checks/power.dc_scenarios(blk, population="china")`, `junction_temps(dc, blk, population)` and `ripple_spurs(blk, population)` read regulators from the fitted parts (`REGULATORS` becomes a function of the circuit and population).

- [ ] **Step 1: Failing tests**

```python
# append to hardware/tests/test_power.py
from neohw import population as pop


def test_rf_ldos_share_pads_and_swap_by_population():
    blk = power.build()
    rf = [p for p in blk.circuit.parts if pop.variant(p, "china") and "TPS7A20" in pop.variant(p, "china").ident]
    assert len(rf) == 2 and all(p.footprint.endswith("SOT-23-5") for p in rf)
    assert all("MIC5205" in pop.variant(p, "iran").ident for p in rf)
    byp = [p for p in blk.circuit.parts if p.value == "470pF"]
    assert len(byp) == 2 and all(pop.fitted(c, "iran") and not pop.fitted(c, "china") for c in byp)


def test_3v3_d_is_tlv75733_in_china_and_ams1117_in_iran():
    blk = power.build()
    china = {p.value for p in blk.circuit.parts if pop.fitted(p, "china") and p.value in ("TLV75733", "AMS1117-3.3")}
    iran = {p.value for p in blk.circuit.parts if pop.fitted(p, "iran") and p.value in ("TLV75733", "AMS1117-3.3")}
    assert china == {"TLV75733"} and iran == {"AMS1117-3.3"}


@pytest.mark.parametrize("population", ["china", "iran"])
def test_every_rail_in_regulation_and_every_junction_cool_enough(ngspice, population):
    blk = power.build()
    dc = check.dc_scenarios(blk, population)
    for name, s in dc.items():
        if "reversed" in name:
            continue
        for rail, v_min in check.RAIL_MIN_V.items():
            assert s[rail] >= v_min, (population, name, rail, s[rail])
    assert max(check.junction_temps(dc, blk, population).values()) <= check.TJ_MAX_C
```

- [ ] **Step 2: Run to see them fail.** `cd hardware && python3 -m pytest -q tests/test_power.py` — Expected: FAIL.

- [ ] **Step 3: Implement.** In `power.build`: each RF LDO is one part (`parts.tps7a20(v)` footprint `Package_TO_SOT_SMD:SOT-23-5`) with `set_variants(china=Variant("TPS7A20-5.0", "TPS7A2050PDBVR", spice_lines=<tps7a20 model>), iran=Variant("MIC5205-5.0", "MIC5205-5.0YM5", spice_lines=<mic5205 model>, torob="MIC5205YM5"))`; pin 4 net `ldo_a_byp` / `ldo_rf_byp` keeps its 470 pF, now `only(…, "iran")` (on the TPS7A20 pin 4 is NC, so the net is open in China: list `NC` in the part's `nc_pins` so the net check accepts it). 3V3_D: `parts.tlv75733()` with 1 µF X7R on IN and OUT, `only(…, "china")`; the existing AMS1117 and its 22 µF tantalum `only(…, "iran")`. PSRR models: MIC5205 stays as `checks/power.psrr_db`; TPS7A20 from its datasheet's PSRR plot at 300 mA (read three points: 1 kHz, 100 kHz, 1 MHz, and interpolate on log f; write the values and the figure number in the docstring). θJA: TPS7A20 DBV and TLV75733 DBV from their datasheets' thermal tables.

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_power.py tests/test_board.py` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): TPS7A20 RF LDOs and TLV75733 (China) with MIC5205 and AMS1117 fallbacks (Iran); power check per population"`

### Task 6: 26 MHz reference: TCXO on CLKIN (China), crystal on XA/XB (Iran)

**Files:**
- Modify: `hardware/neohw/lo_plan.py` (`XTAL_HZ` → `REF_HZ = 26_000_000`), `hardware/neohw/parts.py` (`tcxo_26m()`, `crystal_26mhz()` replacing `crystal_25mhz`), `hardware/neohw/blocks/clock.py`, `sim/neosim/si5351_decode.py`, `firmware/core/lo_plan.h` (`LO_REF_HZ 26000000LL`), `firmware/core/si5351.{c,h}` (reference source), `firmware/core/radio.c` (detect at init), `firmware/host/vhw.c` (status register model), `hardware/neohw/availability.py` (new Torob survey file)
- Create: `docs/superpowers/specs/2026-10-18-parts-survey-torob.json` (26 MHz 3225 crystal)
- Test: `hardware/tests/test_lo_plan.py`, `hardware/tests/test_clock.py`, `firmware/tests/test_si5351.c`, `firmware/tests/test_crosscheck.py`

**Interfaces:**
- Produces: `lo_plan.REF_HZ`; C `LO_REF_HZ`; `si5351_image(const lo_plan_t *p, si5351_ref_t ref, uint8_t reg[])` with `typedef enum { SI5351_REF_XTAL, SI5351_REF_CLKIN } si5351_ref_t;` (register 15: PLLA_SRC bit 2 and PLLB_SRC bit 3 set for CLKIN, CLKIN_DIV bits 7:6 = 00; register 183 crystal load only for XTAL); `si5351_ref_t si5351_detect_ref(void)`: polls register 0 up to 10 ms and returns CLKIN when its CLKIN loss-of-signal bit is clear.

- [ ] **Step 1: Read the datasheets and write down three facts in the clock block docstring** before any code: (1) the Si5351C CLKIN input's accepted signal (level, coupling) and whether a clipped-sine TCXO needs AC coupling and a bias network (Si5351 datasheet, CLKIN section; AN619); (2) register 0's bit positions (SYS_INIT, LOL_B, LOL_A, LOS_CLKIN, LOS_XTAL) from AN619; (3) the TCXO's output spec (clipped sine, ≥ 0.8 Vpp, load). Design the CLKIN network from (1).

- [ ] **Step 2: Failing tests**

```python
# append to hardware/tests/test_lo_plan.py
def test_reference_is_26_mhz_and_every_lo_is_exact():
    from fractions import Fraction
    from neohw import lo_plan
    assert lo_plan.REF_HZ == 26_000_000
    for khz in range(100, 54_001, 7):            # every 7th kHz here; the full grid runs in the check
        p = lo_plan.plan(khz * 1000)
        for name, o in p.outputs.items():
            if name in ("CLK0", "CLK1", "CLK2"):
                assert o.actual_hz == Fraction(khz * 1000 * (4 if name == "CLK2" else 1)), (khz, name)
```

```python
# append to hardware/tests/test_clock.py
def test_tcxo_on_clkin_for_china_and_crystal_on_xa_xb_for_iran():
    from neohw import population as pop
    blk = clock.build()
    tcxo = next(p for p in blk.circuit.parts if p.value == "26MHz TCXO")
    xtal = next(p for p in blk.circuit.parts if p.value == "26MHz")
    assert pop.fitted(tcxo, "china") and not pop.fitted(tcxo, "iran")
    assert pop.fitted(xtal, "iran") and not pop.fitted(xtal, "china")
```

```c
/* append to firmware/tests/test_si5351.c and RUN() it */
static void reference_source_sets_register_15(void)
{
    lo_plan_t p;
    uint8_t reg[SI5351_REG_COUNT];
    CHECK(lo_plan(7074000, &p) == 0);
    si5351_image(&p, SI5351_REF_CLKIN, reg);
    CHECK((reg[SI5351_REG_PLL_SRC] & 0x0C) == 0x0C && (reg[SI5351_REG_PLL_SRC] & 0xC0) == 0);
    si5351_image(&p, SI5351_REF_XTAL, reg);
    CHECK((reg[SI5351_REG_PLL_SRC] & 0x0C) == 0 && reg[SI5351_REG_XTAL_LOAD] == SI5351_XTAL_LOAD_8PF);
}
```
```c
/* host test (firmware/tests/test_radio.c or a new test against the virtual hardware): a TCXO that
   needs 5 ms to start is still detected, and a board without one falls back to the crystal */
static void reference_detection_waits_for_a_slow_tcxo(void)
{
    vhw_set_clkin_delay_ms(5);          /* LOS_CLKIN stays set for 5 ms after power-up */
    CHECK(si5351_detect_ref() == SI5351_REF_CLKIN);
    vhw_set_clkin_present(false);       /* Iran population: no TCXO fitted */
    CHECK(si5351_detect_ref() == SI5351_REF_XTAL);
}
```
(Match `lo_plan`'s real C signature: `grep -n "lo_plan(" firmware/core/lo_plan.h`. The `vhw_set_clkin_*` hooks are new test controls in `firmware/host/vhw.{c,h}`; `hal_delay_ms` must advance the virtual clock so the poll terminates.)

- [ ] **Step 3: Run to see them fail.** `cd hardware && python3 -m pytest -q tests/test_lo_plan.py tests/test_clock.py` and `make fw-test`. Expected: FAIL.

- [ ] **Step 4: Implement.** Rename `XTAL_HZ` to `REF_HZ` in Python (lo_plan and si5351_decode) and C (`LO_XTAL_HZ` → `LO_REF_HZ`), value 26 MHz; update docstrings. Clock block: TCXO (`parts.tcxo_26m()`: KiCad footprint `Oscillator:Oscillator_SMD_SiT_PQFN-4Pin_2.0x1.6mm`, ident `"26 MHz TCXO, 2016, ±1.5 ppm or better, 3.3 V, clipped sine (KDS 1XXD26000MAA or equivalent)"`, VCC on 3V3_RF with 100 nF) and the CLKIN network from Step 1, all `only(…, "china")`; the 10 kΩ CLKIN pull-down becomes `only(…, "iran")` (in China the network sets CLKIN's level); the crystal `parts.crystal_26mhz()` (3225, CL 8 pF; ident `"26 MHz crystal, 3225, CL 8 pF, ±20 ppm"`) `only(…, "iran")`. Torob: search `کریستال 26 مگاهرتز 3225` and `26MHz 3225` on torob.com (browser or `curl -s 'https://torob.com/search/?query=…'`; the site is Iranian and reachable here), record hits in the new survey file in the same shape as `2026-10-13-parts-survey-power-torob.json`, add it to `availability.SURVEYS`, and give `crystal_26mhz` a matcher like `crystal_25mhz_hit` for 26 MHz. Firmware: `si5351_detect_ref()` called once in `radio_init` before the first `si5351_image`; the chosen reference is reported in the CAT telemetry extension (`ZT`) as `REF=TCXO` or `REF=XTAL`. `vhw.c`: register 0 returns LOS_CLKIN clear when the virtual board's population is China (default), set for Iran; a test option delays clearing by 5 ms (Review Focus 4).

- [ ] **Step 5: Run** `cd hardware && python3 -m pytest -q && cd .. && make fw-test` (the crosscheck test compares the C and Python planners on the whole 1 kHz grid at 26 MHz) — Expected: PASS.

- [ ] **Step 6: Commit** `git commit -m "feat: 26 MHz reference: TCXO on CLKIN (China) or crystal (Iran), detected by the firmware; LO planner exact at 26 MHz"`

### Task 7: Relays and remaining factories

**Files:**
- Modify: `hardware/neohw/parts.py` (`relay_dpdt`: footprint `Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y`, mpn `G6K-2F-Y DC5`; docstring records G6KU-2F-Y DC5 as the successor and that Omron lists the G6K-2F-Y as not recommended for new designs)
- Test: `hardware/tests/test_parts.py`, `hardware/tests/test_footprints.py`

- [ ] **Step 1: Failing test**

```python
def test_relay_is_the_surface_mount_g6k_by_manufacturer_part_number():
    from skidl import Circuit
    with Circuit():
        k = parts.relay_dpdt("released")
    assert k.footprint == "Relay_SMD:Relay_DPDT_Omron_G6K-2F-Y" and k.mpn == "G6K-2F-Y DC5"
```

- [ ] **Step 2: Run to see it fail**, **Step 3: implement**, **Step 4: run** `cd hardware && python3 -m pytest -q` (the footprint-exists test covers the new footprint) — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): surface-mount G6K-2F-Y relays"`

### Task 8: Parts list tool, both BOMs, README and the branch note

**Files:**
- Modify: `tools/rfq_parts_list.py` (two sheets of main parts, China and Iran, from `export.bom_rows(…, population)`; a "Fitted" sheet from `export.fitted_rows`; notes rewritten in generic terms, Persian notes kept for the Iran sheet; no distributor numbers), `README.md` (the sourcing sentence: "Parts are identified by manufacturer part number or full specification; the board is assembled from a primary parts set, and every line has a fallback part buyable in the Iranian market that fits the same PCB"; the feature list's changed parts), `docs/superpowers/specs/2026-10-13-china-assembly-parts-design.md` on main (merge note at its top: "Superseded by 2026-10-18-rev-a-china-fab-design.md §2a")
- Test: `hardware/tests/test_export.py` (RFQ sheets have no catalogue numbers: run `rfq_parts_list.main` into `tmp_path` and scan every cell with `ident.CATALOGUE`)

- [ ] **Step 1: Failing test** (as described), **Step 2: run, fail**, **Step 3: implement**, **Step 4: run** `cd hardware && python3 -m pytest -q tests/test_export.py` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "tools+docs: parts list per population with generic identification; README sourcing"`

### Task 9: `filter-bank.gif`

**Files:**
- Create: `tools/media_filter_bank.py`, output `docs/media/rev-a/filter-bank.gif` (committed)
- Modify: `Makefile` (target `media` runs every `tools/media_*.py`; `make media` is listed in README "Building and running")
- Test: `hardware/tests/test_media.py`

**Interfaces:**
- Consumes: `checks.bpf.evaluate(index, 0, population)["s2p"]`-style sweeps; to avoid re-simulating, add an `s2p` return for Iran in `evaluate` (already computed per population in Task 3) and cache both in `build/media/filter_bank.npz` from the `make hw` run (`checks/bpf.run` writes it).

- [ ] **Step 1: Failing test**

```python
# hardware/tests/test_media.py
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]


def test_filter_bank_gif_exists_and_fits_the_size_limit():
    gif = ROOT / "docs/media/rev-a/filter-bank.gif"
    assert gif.exists() and gif.read_bytes()[:6] in (b"GIF89a", b"GIF87a")
    assert gif.stat().st_size <= 8 * 1024 * 1024
```

- [ ] **Step 2: Run, see it fail.**

- [ ] **Step 3: Implement** `tools/media_filter_bank.py`: one frame per filter (11 frames, then the whole bank): S21 in dB over 0.05–200 MHz on a log axis, the selected filter's band shaded, China solid, Iran dashed, title "Filter n: lo–hi MHz (band names)"; frames written as PNG with matplotlib into `build/media/filter-bank/`, then `ffmpeg -framerate 1 -i frame%02d.png -vf "fps=10,scale=900:-1:flags=lanczos,split[a][b];[a]palettegen[p];[b][p]paletteuse" -loop 0 docs/media/rev-a/filter-bank.gif`. Run it after `make hw`.

- [ ] **Step 4: Run** `make hw && make media && cd hardware && python3 -m pytest -q tests/test_media.py` — Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "media: filter-bank.gif (both populations through the bank)"`

### Task 10: Full verification and the characterisation

- [ ] **Step 1:** `free -g` (stop if under 4 GB free), then `make hw-fresh 2>&1 | tee build/hw-fresh.log`. Expected: every check PASS in `build/hw/report.html`, both populations. Any FAIL: stop, report numbers to the user.
- [ ] **Step 2:** `cp -r hardware/characterized /tmp/claude-char-before && make hw && diff -r /tmp/claude-char-before hardware/characterized` (use the session scratchpad, not `/tmp`, when one is listed). Expected: no difference (cache reproduces).
- [ ] **Step 3:** `make test`, `make sim`, `make fw-test`. Expected: all pass; the twin's scenarios use the regenerated `bpf_*.s2p`.
- [ ] **Step 4:** Update the spec §2 table with any synthesis change from Task 3 and the measured numbers; update README "Simulated performance" from `hardware/characterized/` (filters, both populations' worst IL/RL).
- [ ] **Step 5: Commit** `git commit -m "chore(hw): characterisation for Rev A parts, both populations"`

## Self-review notes

- Spec coverage: §2 rows: relays (T7), bias sense (T4), RF LDOs and 3V3_D (T5), TCXO/crystal (T6), filter capacitors and inductors (T3), everything else pinned generically (T2); §2a branch choices taken (T3, T5, T6, T7) and excluded (LCSC fields not carried over: T2's rule test); §2b (T2, T8); §6 part changes and dual population rows (T3–T6, T10), sourcing row (T2 test, T6 Torob); §7a `filter-bank.gif` (T9); §9 plan 5.1 bullet incl. README (T8).
- Not in this plan: STM32F411 (5.2), layout (5.3), fab package (5.4), site pages beyond updating existing ones (5.0).
