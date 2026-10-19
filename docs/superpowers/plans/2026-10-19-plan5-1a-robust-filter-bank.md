# Plan 5.1a: Robust Filter Bank Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Redesign the 11 band-pass filters so every filter meets every target in both populations with every inductor at the physical worst-case Q minus 25 %. The filters use only through-hole axial RF chokes with part numbers.

**Architecture:**
- A fast numpy ABCD ladder model (`synth/ladder.py`), validated against ngspice, drives a constrained optimiser (`synth/robust.py`). The optimiser finds low-loss elliptic or Chebyshev element values and snaps them to buyable parts. It then checks Monte Carlo yield in the fast model and raises its margins until the yield holds.
- The designs are committed as data (`data/filter_designs.json`). `blocks/bpf.py` builds the SKiDL circuit from that file.
- ngspice remains the authority: `make hw` checks the committed designs with the robust model.

**Tech Stack:** Python 3 (numpy, scipy 1.16 `optimize.minimize` SLSQP, SKiDL 2.3, ngspice 45, pytest), KiCad 9 `Inductor_THT` footprints.

**Spec:** `docs/superpowers/specs/2026-10-19-robust-filter-bank-design.md`. Its parent is `2026-10-18-rev-a-china-fab-design.md` §2 and §6.

## Global Constraints

**The robust inductor model** (spec §2) is the only model once Task 6 lands:
- R_t = 2π·f_t·L / (0.75·Q_min).
- Below f_t, R(f) = R_t.
- At and above f_t, R(f) = R_t·f/f_t.
- A parallel C gives the minimum SRF.
- Add lead inductance of 1 nH per mm of lead, where the lead length per choke is (pitch − body length) + 2 × 1.0 mm.

**Capacitors:** C0G, with a series loss of Q = 500 at the band centre.

**Filter inductors:** through-hole axial only, lying flat, the same part in both populations:
- ≥ 1 µH: TDK BC B78108S (body ≤ 4.0 × 9.2 mm, 12.7 mm pitch).
- < 1 µH: Bourns 78F (body ≤ 2.79 × 7.11 mm, 10.16 mm pitch).
- Tolerance: China fits them as bought, at the series grade (BC K ±10 % up to 39 µH, J ±5 % from 47 µH; 78F K ±10 % up to 2.7 µH). Iran LCR-selects to ±3 %.
- No SMD filter inductors.

**Capacitors as fitted:** China ±1 % E24, Iran ±2 % E12, C0G 0805, 1–2 per position.

**Targets, applying to both populations:**
- IL ≤ 3 dB on filters 1–7 and ≤ 2 dB on filters 8–11. Filter 1 instead has IL ≤ 8 dB plus a noise-floor rise ≤ 1 dB (system-limited).
- RL ≥ 14 dB in band.
- 3f rejection ≥ 50 dB over 3·lo..3·hi where 3·lo > hi (filters 3–11).
- Monte Carlo ≥ 95 % of 200 builds.
- Bank mute ≥ 40 dB.

**Optimiser margins:** RL ≥ 15.5 dB and rejection ≥ 53 dB at the start. They are raised if the fast-model yield is < 97 %.

**No silent loss of selectivity.** Attenuation at lo/2 (if ≥ 50 kHz) and at 2·hi must be ≥ min(today's value − 3 dB, today's value) and ≥ 20 dB. "Today's value" is the current Chebyshev design under the same robust model. Each design records both numbers.

**Unchanged:** band edges (`bpf.BANDS`), 11 filters, the relay chain and bit map. Never split a band without the user.

**Failures:** a target a filter cannot meet is reported with numbers (status BLOCKED) and is never loosened.

**Working rules:**
- The full hardware suite must pass at the end of every task: `cd hardware && python3 -m pytest -q`. Until Task 6 the old interpolated model and the old designs stay in place.
- Commit on `main` with the `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>` trailer, and only your own paths (`git add <paths>`).

## Review Focus

1. **A filter where the optimiser cannot meet the constraints for any topology.** The tool must say which constraint failed and by how much, write no design for that filter, and exit non-zero. It must not write the least-bad design (Task 5 test).
2. **A designed value outside the buyable range, or between two chokes no pair can reach within ±2 %.** Snapping must report it, not pick a distant value silently (Task 4 test).
3. **The fast model drifting from ngspice** (relay bus loading, lead inductance, capacitor ESR). Every committed design is re-checked in ngspice, and the ladder-vs-ngspice agreement is a test for every filter, not just the seed design (Task 6 test).
4. **China tolerance of ±10 % on most chokes.** Yield must be computed with each population's real tolerances, not the Iran ±3 %. A design that only passes at ±3 % must fail the China yield gate (Task 4 test).
5. **Filters 1–2 have no 3f target.** The optimiser must still hold the selectivity floor at lo/2 and 2·hi, so it cannot "win" IL by removing selectivity (Task 3 test).

---

### Task 1: Robust inductor model and choke data (additive)

**Files:**
- Modify: `hardware/neohw/inductors.py`
- Modify: `hardware/neohw/data/inductors.json`: add `body_mm` and `pitch_mm` to `tdk_bc` and `bourns_78f`, and add the BC rows 120–1500 µH
- Test: `hardware/tests/test_inductors.py`

**Interfaces:**
- Produces (new; the existing `r_series`, `q_at`, `c_par` and `row` stay unchanged until Task 6):
  - `ROBUST_DERATE = 0.75`, `LEAD_NH_PER_MM = 1.0`, `LEAD_EXTRA_MM = 1.0`
  - `FILTER_LARGE = "tdk_bc"`, `FILTER_SMALL = "bourns_78f"`, `filter_series(henries) -> str` (BC from 1 µH)
  - `robust_r(row: Row, f_hz: float | np.ndarray) -> float | np.ndarray`
  - `lead_l(series: str) -> float` (henries)
  - `available(series: str) -> list[float]` (henries, ascending)
  - `nearest_row(series: str, henries: float) -> Row`
  - `robust_z(henries: float, f_hz: np.ndarray) -> np.ndarray`: the complex impedance of one filter choke of any value in range. It uses the nearest row's Q_min, f_t and SRF; R scales with the actual henries; lead L is added; C_par comes from the row's SRF minimum.
  - `tolerance(row: Row, population: str) -> float`: China is the row's grade (`tol_pct`, or the series `tol_pct`); Iran is 0.03.

- [ ] **Step 1: Add the data.** In `inductors.json`:
  - Set `"body_mm": [4.0, 9.2], "pitch_mm": 12.7` on `tdk_bc`, and `"body_mm": [2.79, 7.11], "pitch_mm": 10.16` on `bourns_78f`.
  - Append these BC rows. They come from the TDK datasheet copied in the research notes (`.superpowers/sdd/2026-10-18-plan5-1-parts-dual-population/task-3-research.md` and the `bc.pdf` table). The columns are µH, Q min, f_t MHz, rated mA, DCR max Ω, SRF min MHz, all ±5 % J. The MPN is `B78108S1{2 significant digits}{exponent}J000`, with the value in nH (e.g. 120 µH = 120000 nH → `B78108S1124J000`).
  ```
  120 70 0.796 300 2.40 4.5
  150 70 0.796 280 2.80 4.2
  180 70 0.796 270 3.00 3.9
  220 70 0.796 250 3.30 3.7
  270 70 0.796 200 5.70 2.8
  330 70 0.796 190 6.40 2.7
  390 70 0.796 180 7.00 2.4
  470 70 0.796 170 7.90 2.2
  560 60 0.796 160 8.80 2.0
  680 55 0.796 150 10.0 1.9
  820 50 0.796 140 12.0 1.6
  1000 50 0.252 130 14.0 1.6
  1200 50 0.252 115 17.5 1.3
  1500 50 0.252 100 23.0 1.25
  ```
  Re-read each row against `bc.pdf` (`pdftotext -layout`) before committing. The copy is in the session scratchpad `ds/bc.pdf`; if it is gone, download `https://static.chipdip.ru/lib/393/DOC047393723.pdf`.

- [ ] **Step 2: Write the failing tests** (append to `tests/test_inductors.py`):

```python
import numpy as np


def test_robust_resistance_is_the_bound_with_25_percent_margin():
    r = inductors.row(inductors.FILTER_LARGE, 39e-6)        # B78108S1393K000: Q 50 at 2.52 MHz
    r_t = 2 * math.pi * r.f_test_hz * r.henries / (0.75 * r.q_min)
    assert inductors.robust_r(r, 0.3e6) == pytest.approx(r_t)               # flat below f_t
    assert inductors.robust_r(r, 2 * r.f_test_hz) == pytest.approx(2 * r_t)  # Q held above f_t
    f = np.array([0.1e6, r.f_test_hz, 10e6])
    assert np.allclose(inductors.robust_r(r, f), [r_t, r_t, r_t * 10e6 / r.f_test_hz])


def test_filter_series_and_lead_inductance():
    assert inductors.filter_series(1e-6) == "tdk_bc" and inductors.filter_series(0.82e-6) == "bourns_78f"
    assert inductors.lead_l("tdk_bc") == pytest.approx((12.7 - 9.2 + 2.0) * 1e-9)
    assert inductors.lead_l("bourns_78f") == pytest.approx((10.16 - 7.11 + 2.0) * 1e-9)


def test_robust_z_of_a_table_value_matches_its_row():
    h, f = 10e-6, np.array([1e6])
    r = inductors.row("tdk_bc", h)
    z = inductors.robust_z(h, f)
    w = 2 * math.pi * f
    z_series = inductors.robust_r(r, f) + 1j * w * (h + inductors.lead_l("tdk_bc"))
    want = z_series / (1 + 1j * w * inductors.c_par(r) * z_series)
    assert np.allclose(z, want)


def test_bc_rows_reach_1500_uh_and_tolerances_follow_population():
    vals = inductors.available("tdk_bc")
    assert vals[0] == pytest.approx(1e-6) and vals[-1] == pytest.approx(1500e-6)
    r = inductors.row("tdk_bc", 39e-6)
    assert inductors.tolerance(r, "china") == 0.10 and inductors.tolerance(r, "iran") == 0.03
    assert inductors.tolerance(inductors.row("tdk_bc", 47e-6), "china") == 0.05
```

- [ ] **Step 3: Run them to see them fail.** Run `cd hardware && python3 -m pytest -q tests/test_inductors.py`. Expected: FAIL (`robust_r` missing).

- [ ] **Step 4: Implement** (append to `inductors.py`; add `import numpy as np`; add `tol` to `Row` with a default of `None` so existing calls keep working):

```python
ROBUST_DERATE = 0.75            # spec 2026-10-19 §2: 25 % below the guaranteed Q
LEAD_NH_PER_MM = 1.0
LEAD_EXTRA_MM = 1.0             # per side: body end to the board, and through it
FILTER_LARGE, FILTER_SMALL = "tdk_bc", "bourns_78f"
IRAN_SELECT_TOL = 0.03


def filter_series(henries: float) -> str:
    return FILTER_LARGE if henries >= 1e-6 * (1 - 1e-9) else FILTER_SMALL


def robust_r(r: Row, f_hz):
    """Physical worst case (spec §2): R ≤ R(f_t) below f_t; Q held at the derated Q_min above."""
    r_t = 2 * math.pi * r.f_test_hz * r.henries / (ROBUST_DERATE * r.q_min)
    f = np.asarray(f_hz, dtype=float)
    out = np.where(f < r.f_test_hz, r_t, r_t * f / r.f_test_hz)
    return float(out) if out.ndim == 0 else out


def lead_l(series: str) -> float:
    info = series_info(series)
    return (info["pitch_mm"] - info["body_mm"][1] + 2 * LEAD_EXTRA_MM) * LEAD_NH_PER_MM * 1e-9


def available(series: str) -> list[float]:
    return sorted(r["uh"] * 1e-6 for r in _series()[series]["rows"])


def nearest_row(series: str, henries: float) -> Row:
    uh = min(available(series), key=lambda h: abs(math.log(h / henries)))
    return row(series, uh)


def robust_z(henries: float, f_hz):
    """One filter choke of `henries` (any value in its series' range): the nearest datasheet
    row's Q_min, f_t and SRF; R scales with the actual inductance; lead L added."""
    series = filter_series(henries)
    r = nearest_row(series, henries)
    scaled = Row(r.series, r.mpn, henries, r.q_min, r.f_test_hz, r.srf_min_hz, r.dcr_max, r.tol)
    w = 2 * math.pi * np.asarray(f_hz, dtype=float)
    z = robust_r(scaled, f_hz) + 1j * w * (henries + lead_l(series))
    return z / (1 + 1j * w * c_par(r) * z)


def tolerance(r: Row, population: str) -> float:
    if population == "iran":
        return IRAN_SELECT_TOL
    pct = r.tol if r.tol is not None else series_info(r.series)["tol_pct"]
    return pct / 100
```

In `row()`, fill `tol` from the row's `tol_pct` if present, else from the series' `tol_pct`.
- BC rows up to 39 µH are `K` (10 %) and from 47 µH `J` (5 %). Add a series-level `"tol_pct": 5` to `tdk_bc`, and give every row of 39 µH or less `"tol_pct": 10`.
- The 78F rows already carry `tol_pct`.

- [ ] **Step 5: Run** `cd hardware && python3 -m pytest -q`. Expected: PASS (all new and existing tests).

- [ ] **Step 6: Commit** `git commit -m "feat(hw): robust filter-choke model (worst-case bound, 25 % margin, lead L) and TDK BC rows to 1.5 mH"`

### Task 2: Fast ladder model, validated against ngspice

**Files:**
- Create: `hardware/neohw/synth/ladder.py`
- Test: `hardware/tests/test_ladder.py`

**Interfaces:**
- Consumes: Task 1 `inductors.robust_z`.
- Produces:
  - `Branch(place: "series"|"shunt", form: "L"|"C"|"LC_par"|"LC_ser", l: float = 0, c: float = 0)`
  - `Losses(inductor_z: Callable[[float, ndarray], ndarray], cap_q: float = 500.0, f_cap_q_hz: float = 1e6)`
  - `robust_losses(f_centre_hz) -> Losses`
  - `Bus(shunt_c: float = 0.0, series_r: float = 0.0)`, `bank_bus() -> Bus`
  - `s21_s11(branches, f, losses, bus=Bus(), z0=50.0) -> (ndarray, ndarray)`, `db(x)`
  - `metrics(branches, lo, hi, losses, bus) -> dict` with keys `il_db`, `rl_db`, `rej3f_db` (`None` when 3·lo ≤ hi), `att_half_lo_db` (`None` when lo/2 < 50 kHz), `att_2hi_db`
  - `from_positions(positions) -> list[Branch]` (today's `bpf.Position` list → branches)

- [ ] **Step 1: Write the failing tests**

```python
# hardware/tests/test_ladder.py
import math

import numpy as np
import pytest

from neohw.synth import ladder
from neohw.synth.filters import lowpass


def _lossless():
    return ladder.Losses(inductor_z=lambda h, f: 1j * 2 * math.pi * f * h, cap_q=1e12)


def test_lossless_chebyshev_ripple_and_cutoff():
    els = lowpass(10e6, 5, 0.1)
    br = [ladder.Branch("shunt", "C", c=e.value) if e.kind == "C_shunt" else ladder.Branch("series", "L", l=e.value)
          for e in els]
    f = np.linspace(0.1e6, 10e6, 400)
    s21, _ = ladder.s21_s11(br, f, _lossless())
    assert -ladder.db(s21).min() == pytest.approx(0.1, abs=0.01)     # equiripple 0.1 dB
    s21c, _ = ladder.s21_s11(br, np.array([10e6]), _lossless())
    assert -ladder.db(s21c)[0] == pytest.approx(0.1, abs=0.02)      # ripple bandwidth edge


def test_lossless_network_conserves_power():
    br = [ladder.Branch("series", "LC_par", l=1e-6, c=100e-12), ladder.Branch("shunt", "LC_ser", l=2e-6, c=50e-12),
          ladder.Branch("shunt", "C", c=200e-12)]
    f = np.linspace(1e6, 50e6, 200)
    s21, s11 = ladder.s21_s11(br, f, _lossless())
    assert np.allclose(np.abs(s21) ** 2 + np.abs(s11) ** 2, 1.0, atol=1e-9)


def test_branch_rejects_missing_values():
    with pytest.raises(ValueError):
        ladder.Branch("series", "LC_par", l=1e-6)
    with pytest.raises(ValueError):
        ladder.Branch("diagonal", "L", l=1e-6)


@pytest.mark.integration
def test_matches_ngspice_on_todays_filter_8_in_the_bank():
    """Today's Chebyshev filter 8 in the full bank, with the robust model on both sides: the
    fast model (with the relay bus loading) against ngspice."""
    from neohw.blocks import bpf
    from tests.robust_spice import spice_s21_db            # helper added in this task
    d = bpf.design(8)
    br = ladder.from_positions(d.positions)
    f = np.linspace(d.lo_hz, d.hi_hz, 41)
    f3 = np.linspace(3 * d.lo_hz, 3 * d.hi_hz, 41)
    losses = ladder.robust_losses(d.centre_hz)
    fast = ladder.db(ladder.s21_s11(br, np.concatenate([f, f3]), losses, ladder.bank_bus())[0])
    ref = spice_s21_db(8, np.concatenate([f, f3]))
    assert np.max(np.abs(fast[:41] - ref[:41])) <= 0.05
    assert np.max(np.abs(fast[41:] - ref[41:])[ref[41:] > -60]) <= 1.0
```

- [ ] **Step 2: Run** `cd hardware && python3 -m pytest -q tests/test_ladder.py`. Expected: FAIL (no module).

- [ ] **Step 3: Implement `ladder.py`:**

```python
"""Fast ABCD model of a filter ladder between equal terminations (plan 5.1a, spec 2026-10-19 §4).

Branches run from the source side; a series branch is an impedance in the through path, a shunt
branch an admittance to ground. Inductor impedance comes from a loss model (by default the robust
model of neohw.inductors); capacitors carry a series loss giving `cap_q` at `f_cap_q_hz`. The
relay bank loads each filter port (`Bus`). ngspice remains the authority for reported numbers."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from neohw import inductors

Z0 = 50.0
PLACES, FORMS = ("series", "shunt"), ("L", "C", "LC_par", "LC_ser")


@dataclass(frozen=True)
class Branch:
    place: str
    form: str
    l: float = 0.0
    c: float = 0.0

    def __post_init__(self):
        if self.place not in PLACES or self.form not in FORMS:
            raise ValueError(f"branch place/form must be in {PLACES}/{FORMS}, got {self.place!r}/{self.form!r}")
        if (self.form != "C" and self.l <= 0) or (self.form != "L" and self.c <= 0):
            raise ValueError(f"{self.form} branch needs positive values, got l={self.l!r} c={self.c!r}")


@dataclass(frozen=True)
class Losses:
    inductor_z: Callable[[float, np.ndarray], np.ndarray]
    cap_q: float = 500.0
    f_cap_q_hz: float = 1e6


def robust_losses(f_centre_hz: float) -> Losses:
    return Losses(inductors.robust_z, 500.0, f_centre_hz)


@dataclass(frozen=True)
class Bus:
    shunt_c: float = 0.0     # relay open-contact and inter-pole capacitance at each filter port
    series_r: float = 0.0    # closed contact of the selected filter's relay


def bank_bus() -> Bus:
    """Loading of one selected filter in the 11-filter bank, from the relay model in
    neohw.parts.relay_dpdt (read its defaults; do not copy numbers)."""
    from neohw import parts
    import inspect
    sig = inspect.signature(parts.relay_dpdt).parameters
    open_c, contact_r = sig["open_c"].default, sig["contact_r"].default
    n = 11
    return Bus(shunt_c=(n - 1) * (open_c + parts.G6K_INTER_POLE_C) + parts.G6K_INTER_POLE_C + open_c,
               series_r=contact_r)


def _cap_z(c, w, losses: Losses):
    esr = 1 / (2 * math.pi * losses.f_cap_q_hz * c * losses.cap_q)
    return esr + 1 / (1j * w * c)


def branch_z(b: Branch, f: np.ndarray, losses: Losses) -> np.ndarray:
    w = 2 * math.pi * f
    zl = losses.inductor_z(b.l, f) if b.form != "C" else None
    zc = _cap_z(b.c, w, losses) if b.form != "L" else None
    if b.form == "L":
        return zl
    if b.form == "C":
        return zc
    if b.form == "LC_ser":
        return zl + zc
    return zl * zc / (zl + zc)


def s21_s11(branches, f, losses: Losses, bus: Bus = Bus(), z0: float = Z0):
    f = np.asarray(f, dtype=float)
    one, zero = np.ones_like(f, dtype=complex), np.zeros_like(f, dtype=complex)
    a, b, c, d = one, zero, zero, one

    def series(z):
        nonlocal a, b, c, d
        b, d = a * z + b, c * z + d

    def shunt(y):
        nonlocal a, b, c, d
        a, c = a + b * y, c + d * y

    w = 2 * math.pi * f
    if bus.shunt_c:
        shunt(1j * w * bus.shunt_c)
    if bus.series_r:
        series(bus.series_r * one)
    for br in branches:
        z = branch_z(br, f, losses)
        series(z) if br.place == "series" else shunt(1 / z)
    if bus.series_r:
        series(bus.series_r * one)
    if bus.shunt_c:
        shunt(1j * w * bus.shunt_c)
    den = a + b / z0 + c * z0 + d
    return 2 / den, (a + b / z0 - c * z0 - d) / den


def db(x) -> np.ndarray:
    return 20 * np.log10(np.maximum(np.abs(x), 1e-30))


def metrics(branches, lo, hi, losses: Losses, bus: Bus = Bus(), n: int = 41) -> dict:
    band = np.linspace(lo, hi, n)
    s21, s11 = s21_s11(branches, band, losses, bus)
    out = {"il_db": float(-db(s21).min()), "rl_db": float(-db(s11).max()), "rej3f_db": None,
           "att_half_lo_db": None, "att_2hi_db": float(-db(s21_s11(branches, [2 * hi], losses, bus)[0])[0])}
    if 3 * lo > hi:
        out["rej3f_db"] = float(-db(s21_s11(branches, np.linspace(3 * lo, 3 * hi, n), losses, bus)[0]).max())
    if lo / 2 >= 50e3:
        out["att_half_lo_db"] = float(-db(s21_s11(branches, [lo / 2], losses, bus)[0])[0])
    return out


def from_positions(positions) -> list[Branch]:
    """Today's bpf.Position list (C_series, L_shunt, C_shunt, L_series) as branches, using each
    position's ideal value."""
    out = []
    for p in positions:
        place = "series" if p.kind.endswith("series") else "shunt"
        out.append(Branch(place, "L", l=p.ideal) if p.kind.startswith("L") else Branch(place, "C", c=p.ideal))
    return out
```

`a, b` before `series` uses the ABCD product [[a,b],[c,d]]·[[1,z],[0,1]] = [[a, a·z+b],[c, c·z+d]], and `shunt` uses [[a,b],[c,d]]·[[1,0],[y,1]] = [[a+b·y, b],[c+d·y, d]]. Check both by hand against the lossless conservation test.

**The ngspice helper `hardware/tests/robust_spice.py`.** It builds today's filter n in the bank (`bpf_bank.build(n)`). It replaces every filter inductor's SPICE lines with the robust model of the same value: L + lead L, R = `inductors.robust_r(nearest_row, f_centre)`, and C_par. Because R depends on frequency, it runs a narrow-band sweep per frequency segment:
- in band: R evaluated at the centre;
- the 3f band: R evaluated at 3·centre.

The fast model must use the same rule for this comparison. For the test only, pass `Losses(lambda h, f: <robust_z evaluated with R fixed at the segment's frequency>, …)` and compare segment by segment. Write that wrapper next to the helper, and state the reason in its docstring: ngspice AC has no frequency-dependent R, so both sides fix R per segment.

Capacitors carry ESR from Q = 500 at the centre on both sides (`parts._capacitor_lines(farads, esr, 0, part)`).

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_ladder.py`. Expected: PASS. If the ngspice comparison misses, fix `bank_bus()` from the relay model's lines (`parts.relay_dpdt` `lines`). Do not widen the tolerance.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): fast ABCD ladder model with relay-bus loading, validated against ngspice"`

### Task 3: Optimiser for low-loss elliptic or Chebyshev filters

**Files:**
- Create: `hardware/neohw/synth/robust.py`
- Test: `hardware/tests/test_robust_synth.py`

**Interfaces:**
- Consumes: Task 2 (`Branch`, `metrics`, `s21_s11`, `robust_losses`, `bank_bus`), `synth.filters.highpass/lowpass`, `blocks.bpf.BANDS`, `EDGE_MARGIN`, today's `bpf.design(index)` (the selectivity reference), and `checks.bpf.il_limit_db`.
- Produces:
  - `Targets(index, lo, hi, il_max, rl_min, rej_min: float | None, att_half_lo_min: float | None, att_2hi_min: float)`
  - `targets(index, rl_min=15.5, rej_min=53.0) -> Targets`. The selectivity floors are min(today − 3 dB, today), at least 20 dB, with today's design in the robust model.
  - `Topology(hpf_order: int, lpf: str)` with `lpf` in `("cheb7", "cheb9", "ell5", "ell7")`; `TOPOLOGIES`
  - `seed(top, lo, hi) -> list[Branch]`
  - `optimise(top, t: Targets) -> Result(branches, metrics, feasible: bool, violations: dict[str, float])`
  - `best(index, rl_min=15.5, rej_min=53.0) -> Result`: the lowest-IL feasible result over `TOPOLOGIES`, or the closest infeasible one with its violations.

- [ ] **Step 1: Write the failing tests**

```python
# hardware/tests/test_robust_synth.py
import pytest

from neohw.synth import ladder, robust


def test_elliptic_seed_has_tanks_with_zeros_in_the_3f_band():
    br = robust.seed(robust.Topology(3, "ell5"), 16e6, 22e6)
    tanks = [b for b in br if b.form == "LC_par"]
    assert len(tanks) == 2
    import math
    for b in tanks:
        fz = 1 / (2 * math.pi * math.sqrt(b.l * b.c))
        assert 3 * 16e6 * 0.9 <= fz <= 3 * 22e6 * 1.5


def test_targets_carry_selectivity_floors():
    t = robust.targets(1)
    assert t.rej_min is None and t.att_2hi_min >= 20 and t.il_max == 8.0
    assert robust.targets(8).rej_min == 53.0 and robust.targets(8).il_max == 2.0


@pytest.mark.integration
def test_optimised_filter_8_meets_its_constraints_with_margin():
    r = robust.best(8)
    assert r.feasible, r.violations
    m = r.metrics
    assert m["il_db"] <= 2.0 and m["rl_db"] >= 15.5 and m["rej3f_db"] >= 53.0
    t = robust.targets(8)
    assert m["att_2hi_db"] >= t.att_2hi_min


@pytest.mark.integration
def test_filter_1_cannot_buy_il_by_losing_selectivity():
    r = robust.best(1)
    t = robust.targets(1)
    assert r.metrics["att_2hi_db"] >= t.att_2hi_min - 1e-6
    if t.att_half_lo_min is not None:
        assert r.metrics["att_half_lo_db"] >= t.att_half_lo_min - 1e-6


def test_infeasible_targets_report_violations():
    t = robust.targets(8)
    impossible = robust.Targets(t.index, t.lo, t.hi, 0.01, 40.0, 120.0, t.att_half_lo_min, t.att_2hi_min)
    r = robust.optimise(robust.Topology(3, "ell5"), impossible)
    assert not r.feasible and r.violations and all(v > 0 for v in r.violations.values())
```

- [ ] **Step 2: Run** `cd hardware && python3 -m pytest -q tests/test_robust_synth.py`. Expected: FAIL.

- [ ] **Step 3: Implement `robust.py`:**

```python
"""Low-loss filter synthesis by optimisation against the robust model (spec 2026-10-19 §4).

Each topology is an HPF section (Chebyshev, series C / shunt L) followed by an LPF section:
Chebyshev (shunt C / series L) or elliptic (the series inductors become parallel-LC tanks whose
zeros start in the 3f band). The optimiser minimises the worst in-band IL t (epigraph form) under
RL, 3f rejection and selectivity-floor constraints, all in the fast ladder model with the relay
bus loading, over log element values within the buyable range."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from neohw import inductors
from neohw.synth import ladder
from neohw.synth.filters import highpass, lowpass

L_RANGE = (0.1e-6, 1500e-6)
C_RANGE = (1e-12, 100e-9)
N_BAND, N_REJ = 41, 41
SEED_RIPPLE_DB = 0.02


@dataclass(frozen=True)
class Targets:
    index: int
    lo: float
    hi: float
    il_max: float
    rl_min: float
    rej_min: float | None
    att_half_lo_min: float | None
    att_2hi_min: float


@dataclass(frozen=True)
class Topology:
    hpf_order: int
    lpf: str


TOPOLOGIES = tuple(Topology(h, l) for h in (3, 5) for l in ("ell5", "ell7", "cheb7", "cheb9"))


@dataclass(frozen=True)
class Result:
    topology: Topology
    branches: tuple
    metrics: dict
    feasible: bool
    violations: dict


def targets(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Targets:
    from neohw.blocks import bpf
    from neohw.checks import bpf as check
    lo, hi = bpf.BANDS[index - 1]
    d = bpf.design(index)                          # today's Chebyshev design: the selectivity reference
    m = ladder.metrics(ladder.from_positions(d.positions), lo, hi,
                       ladder.robust_losses(d.centre_hz), ladder.bank_bus())
    floor = lambda today: None if today is None else max(20.0, today - 3.0)
    return Targets(index, lo, hi, check.il_limit_db(index), rl_min, rej_min if 3 * lo > hi else None,
                   floor(m["att_half_lo_db"]), floor(m["att_2hi_db"]))


def _hpf(lo, order):
    from neohw.blocks.bpf import EDGE_MARGIN
    return [ladder.Branch("series", "C", c=e.value) if e.kind == "C_series" else ladder.Branch("shunt", "L", l=e.value)
            for e in highpass(lo / EDGE_MARGIN, order, SEED_RIPPLE_DB)]


def _lpf(lo, hi, kind):
    order = int(kind[-1])
    els = lowpass(hi * 1.03, order, SEED_RIPPLE_DB)
    out = [ladder.Branch("shunt", "C", c=e.value) if e.kind == "C_shunt" else ladder.Branch("series", "L", l=e.value)
           for e in els]
    if kind.startswith("ell"):
        series_l = [k for k, b in enumerate(out) if b.form == "L"]
        f_lo, f_hi = (3 * lo, 3 * hi) if 3 * lo > hi else (2 * hi, 4 * hi)
        zeros = np.geomspace(f_lo, f_hi * 1.2, len(series_l))
        for k, fz in zip(series_l, zeros):
            l = out[k].l
            out[k] = ladder.Branch("series", "LC_par", l=l, c=1 / ((2 * math.pi * fz) ** 2 * l))
    return out


def seed(top: Topology, lo: float, hi: float) -> list:
    return _hpf(lo, top.hpf_order) + _lpf(lo, hi, top.lpf)


def _pack(branches):
    x, slots = [], []
    for k, b in enumerate(branches):
        if b.form != "C":
            x.append(math.log(b.l)); slots.append((k, "l"))
        if b.form != "L":
            x.append(math.log(b.c)); slots.append((k, "c"))
    return np.array(x), slots


def _unpack(x, slots, template):
    vals = {k: {"l": b.l, "c": b.c} for k, b in enumerate(template)}
    for v, (k, which) in zip(x, slots):
        vals[k][which] = math.exp(v)
    return [ladder.Branch(b.place, b.form, l=vals[k]["l"], c=vals[k]["c"]) for k, b in enumerate(template)]


def optimise(top: Topology, t: Targets, max_iter: int = 300) -> Result:
    template = seed(top, t.lo, t.hi)
    x0, slots = _pack(template)
    losses = ladder.robust_losses(math.sqrt(t.lo * t.hi))
    bus = ladder.bank_bus()
    band = np.linspace(t.lo, t.hi, N_BAND)
    rej = np.linspace(3 * t.lo, 3 * t.hi, N_REJ) if t.rej_min is not None else None
    sel = [(t.hi * 2, t.att_2hi_min)] + ([(t.lo / 2, t.att_half_lo_min)] if t.att_half_lo_min is not None else [])
    bounds = [(math.log(L_RANGE[0]), math.log(L_RANGE[1])) if w == "l" else (math.log(C_RANGE[0]), math.log(C_RANGE[1]))
              for _, w in slots] + [(0.0, 60.0)]

    def response(x):
        br = _unpack(x[:-1], slots, template)
        s21, s11 = ladder.s21_s11(br, band, losses, bus)
        out = [-ladder.db(s21), -ladder.db(s11)]
        out.append(-ladder.db(ladder.s21_s11(br, rej, losses, bus)[0]) if rej is not None else np.array([]))
        out.append(np.array([-ladder.db(ladder.s21_s11(br, [f], losses, bus)[0])[0] for f, _ in sel]))
        return out

    def cons(x):
        il, rl, rj, at = response(x)
        parts_ = [x[-1] - il, rl - t.rl_min]
        if rej is not None:
            parts_.append(rj - t.rej_min)
        parts_.append(at - np.array([m for _, m in sel]))
        return np.concatenate(parts_)

    il0 = response(np.append(x0, 0.0))[0].max()
    res = minimize(lambda x: x[-1], np.append(x0, il0), method="SLSQP", bounds=bounds,
                   constraints=[{"type": "ineq", "fun": cons}], options={"maxiter": max_iter, "ftol": 1e-6})
    br = _unpack(res.x[:-1], slots, template)
    m = ladder.metrics(br, t.lo, t.hi, losses, bus)
    v = {}
    if m["il_db"] > t.il_max: v["il_db"] = m["il_db"] - t.il_max
    if m["rl_db"] < t.rl_min: v["rl_db"] = t.rl_min - m["rl_db"]
    if t.rej_min is not None and m["rej3f_db"] < t.rej_min: v["rej3f_db"] = t.rej_min - m["rej3f_db"]
    if m["att_2hi_db"] < t.att_2hi_min - 1e-6: v["att_2hi_db"] = t.att_2hi_min - m["att_2hi_db"]
    if t.att_half_lo_min is not None and m["att_half_lo_db"] < t.att_half_lo_min - 1e-6:
        v["att_half_lo_db"] = t.att_half_lo_min - m["att_half_lo_db"]
    return Result(top, tuple(br), m, not v, v)


def best(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Result:
    t = targets(index, rl_min, rej_min)
    results = [optimise(top, t) for top in TOPOLOGIES]
    feasible = [r for r in results if r.feasible]
    if feasible:
        return min(feasible, key=lambda r: r.metrics["il_db"])
    return min(results, key=lambda r: sum(r.violations.values()))
```

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q tests/test_robust_synth.py`. Expected: PASS. If `best(8)` is infeasible:
  - first try more starts: perturb the seed values by ×(1 ± 10 %), 5 starts, and keep the best;
  - then try `maxiter` 1000.

  Report the violations if it is still infeasible. Do not relax the targets.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): low-loss filter synthesis by optimisation against the robust model (elliptic and Chebyshev topologies)"`

### Task 4: Realisation on buyable parts and the yield loop

**Files:**
- Modify: `hardware/neohw/synth/robust.py`
- Test: `hardware/tests/test_robust_synth.py`

**Interfaces:**
- Consumes: Task 1 (`available`, `filter_series`, `row`, `tolerance`), Task 3 (`Result`, `optimise`, `targets`), `eseries.combo`, `eseries.E24`, `eseries.E12`, `blocks.bpf.C_MIN`.
- Produces:
  - `snap_l(target: float) -> tuple[float, ...]`: 1–2 chokes in series from `available(filter_series(.))`. One part if within 2 %, else the best pair, also within 2 %. Raises `ValueError("cannot realise …")` otherwise.
  - `Realised(branches_china: tuple, branches_iran: tuple, l_parts: dict[int, tuple], c_china: dict[int, tuple], c_iran: dict[int, tuple], topology, metrics: dict[str, dict], yield_pct: dict[str, float], margins: dict)`
  - `realise(result: Result, t: Targets) -> Realised`
  - `fast_yield(realised, t, population, runs=200, seed=0) -> float`
  - `design(index) -> Realised`: `best`, then `realise`, then `fast_yield` for both populations. If either yield is < 97 %, it raises `rl_min` by 1 dB and `rej_min` by 2 dB and repeats, at most 3 times. It raises `DesignError` with the last metrics if the yield still fails.

**How realisation works:**
1. Each inductor value is snapped with `snap_l` (the same parts in both populations).
2. China fits the bought sum; Iran selects to the target (nominal L = the design value).
3. With the inductors fixed, re-optimise the capacitors only (continuous), separately per population's inductor values: China uses the bought sums, Iran the targets.
4. Snap the capacitors with `eseries.combo(c, E24 for China / E12 for Iran, minimum=C_MIN)`.
5. Evaluate `metrics` for each population.

**How Monte Carlo works (`fast_yield`).** Draw each part uniformly within ±tolerance, the same distribution as `checks/bpf._variation` (read it and match it):
- inductors: `inductors.tolerance(row, population)`;
- capacitors: China 0.01, Iran 0.02.

A run passes when the IL, RL and rejection targets (the real targets: 3 / 2 / 8 dB, 14 dB, 50 dB) hold.

- [ ] **Step 1: Write the failing tests**

```python
def test_snap_uses_one_choke_when_close_and_reports_the_unreachable():
    assert robust.snap_l(39e-6) == (39e-6,)
    parts_ = robust.snap_l(50e-6)
    assert abs(sum(parts_) / 50e-6 - 1) <= 0.02
    with pytest.raises(ValueError, match="cannot realise"):
        robust.snap_l(5000e-6)                     # above the largest choke pair


def test_china_yield_uses_china_tolerances():
    """A design whose margins only survive ±3 % must fail the China yield (±10 % chokes)."""
    from neohw import inductors
    r = robust.best(8, rl_min=14.0, rej_min=50.0)      # no margin at all
    real = robust.realise(r, robust.targets(8, 14.0, 50.0))
    china, iran = (robust.fast_yield(real, robust.targets(8), p) for p in ("china", "iran"))
    assert china <= iran                       # ±10 % chokes never yield better than ±3 %
    assert china < 97                          # and a zero-margin design does not pass the China gate


@pytest.mark.integration
def test_design_of_filter_8_holds_yield_in_both_populations():
    real = robust.design(8)
    assert real.yield_pct["china"] >= 97 and real.yield_pct["iran"] >= 97
    for pop in ("china", "iran"):
        m = real.metrics[pop]
        assert m["il_db"] <= 2.0 and m["rl_db"] >= 14 and m["rej3f_db"] >= 50
```

- [ ] **Step 2: Run** `cd hardware && python3 -m pytest -q tests/test_robust_synth.py -k "snap or yield or design_of"`. Expected: FAIL.

- [ ] **Step 3: Implement** `snap_l`, `Realised`, `realise`, `fast_yield`, `DesignError` and `design` as specified above.
  - The capacitor-only re-optimisation is `optimise` restricted to the C slots: pass a mask of which slots are free.
  - Keep functions under about 40 lines each.

- [ ] **Step 4: Run** the tests. Expected: PASS.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): realise designs on buyable chokes and E24/E12 capacitors; fast Monte Carlo yield loop"`

### Task 5: Design all eleven filters and commit the designs

**Files:**
- Create: `tools/synth_filters.py`, `hardware/neohw/data/filter_designs.json` (generated, committed)
- Test: `hardware/tests/test_filter_designs.py`

**Interfaces:**
- Consumes: Task 4 `design(index)`.
- Produces `filter_designs.json`:

```json
{"_doc": "...", "settings": {"robust_derate": 0.75, "cap_q": 500, "rl_min_start": 15.5, "rej_min_start": 53.0, "topologies": ["3/ell5", "..."]},
 "filters": [{"index": 8, "lo_hz": 16e6, "hi_hz": 22e6, "topology": {"hpf_order": 3, "lpf": "ell5"},
   "margins": {"rl_min": 15.5, "rej_min": 53.0},
   "branches": [{"place": "series", "form": "C", "l": 0, "c": 1.2e-10, "l_parts": [], "c_china": [1.2e-10], "c_iran": [1.2e-10]}],
   "fast": {"china": {"il_db": 1.4, "rl_db": 16.0, "rej3f_db": 55.0, "att_half_lo_db": 30.1, "att_2hi_db": 41.0, "yield_pct": 99.5},
            "iran": {"...": "..."}},
   "selectivity_today": {"att_half_lo_db": 31.0, "att_2hi_db": 44.0}}]}
```

- [ ] **Step 1: Write the failing test**

```python
# hardware/tests/test_filter_designs.py
import json
from pathlib import Path

import pytest

DESIGNS = Path(__file__).resolve().parents[1] / "neohw/data/filter_designs.json"


def test_every_filter_has_a_design_on_buyable_parts():
    from neohw import eseries, inductors
    doc = json.loads(DESIGNS.read_text())
    assert [f["index"] for f in doc["filters"]] == list(range(1, 12))
    chokes = {round(h, 15) for s in ("tdk_bc", "bourns_78f") for h in inductors.available(s)}
    e24 = {round(v, 15) for v in eseries.values(eseries.E24, 1e-13, 1e-6)}
    e12 = {round(v, 15) for v in eseries.values(eseries.E12, 1e-13, 1e-6)}
    for f in doc["filters"]:
        for b in f["branches"]:
            assert all(round(h, 15) in chokes for h in b["l_parts"]), (f["index"], b)
            assert all(round(c, 15) in e24 for c in b["c_china"]) and all(round(c, 15) in e12 for c in b["c_iran"])
            assert (b["form"] == "C") == (not b["l_parts"]) and (b["form"] == "L") == (not b["c_china"])


def test_fast_model_yield_and_selectivity_recorded():
    doc = json.loads(DESIGNS.read_text())
    for f in doc["filters"]:
        for pop in ("china", "iran"):
            assert f["fast"][pop]["yield_pct"] >= 97, (f["index"], pop)
        assert f["fast"]["china"]["att_2hi_db"] >= min(f["selectivity_today"]["att_2hi_db"] - 3, 20) - 1e-6
```

Also in Step 1, add a test of the tool's failure path:
- Run `tools/synth_filters.py --only 8 --out <tmp> --force-targets il_max=0.01` through `subprocess`.
- Expect a non-zero exit and a stderr message naming `il_db` and the shortfall.
- Expect no JSON file written.

The tool takes `--force-targets` (test-only overrides of `Targets` fields) for this.

- [ ] **Step 2: Run** `cd hardware && python3 -m pytest -q tests/test_filter_designs.py`. Expected: FAIL (no file).

- [ ] **Step 3: Implement `tools/synth_filters.py`:**
  - argparse with `--only N` (repeatable), `--out PATH` (default `hardware/neohw/data/filter_designs.json`) and `--force-targets k=v`.
  - Insert `hardware/` on `sys.path` like `tools/rfq_parts_list.py`.
  - Call `robust.design(n)` for each filter, using processes in parallel (`concurrent.futures.ProcessPoolExecutor`, max 8 workers; check `free -g` first).
  - Print a table: index, topology, IL / RL / rejection / selectivity / yield for both populations.
  - On any `DesignError`, print it to stderr, write nothing, and exit 1.
  - The JSON numbers are rounded to 15 significant digits (`float(f"{x:.15g}")`) so the E-series membership test is exact.

- [ ] **Step 4: Run the design.** `cd /home/neo/projects/personal/NeoRadio && python3 tools/synth_filters.py`
  - Expected: all 11 filters designed, exit 0. Paste the table into the task report.
  - If a filter fails (`DesignError`), stop with status BLOCKED, the filter's numbers, and the closest topology's violations. Then propose, without implementing: raising one series-L to a two-choke pair, adding a section (HPF order 7 or LPF ell9), or splitting the band. Splitting needs the user's approval.

- [ ] **Step 5: Run** `cd hardware && python3 -m pytest -q tests/test_filter_designs.py`. Expected: PASS.

- [ ] **Step 6: Commit** `git add tools/synth_filters.py hardware/neohw/data/filter_designs.json hardware/tests/test_filter_designs.py && git commit -m "feat(hw): robust designs of all 11 filters on through-hole chokes (tools/synth_filters.py)"`

### Task 6: Build the bank from the designs; the robust model becomes the only model

**Files:**
- Modify:
  - `hardware/neohw/blocks/bpf.py`: designs come from `filter_designs.json`, and branches are built by form.
  - `hardware/neohw/parts.py`: `filter_choke`; remove `filter_inductor`, `_datasheet_ident` (if unused) and `SMD_INDUCTOR_TOL`.
  - `hardware/neohw/inductors.py`: remove the interpolated `r_series`/`q_at`. `robust_r` is the model, and `r_series` becomes an alias for `robust_r`, so callers keep working.
  - `hardware/neohw/checks/bpf.py`: docstrings; the characterisation records `filter_designs` and `robust_derate`.
  - `hardware/neohw/footprints.py`: remove the combined-inductor generator (keep `FOOTPRINT_LIB`).
  - Delete `hardware/footprints/NeoRadio.pretty/L_1210_Axial_P10.16mm.kicad_mod` and `L_1812_Axial_P10.16mm.kicad_mod`.
- Tests: `tests/test_bpf_blocks.py`, `tests/test_bpf_check.py`, `tests/test_footprints.py`, `tests/test_inductors.py`, `tests/test_ladder.py` (add the per-filter ngspice agreement).

**Interfaces:**
- Consumes: the Task 5 JSON, Task 1 (`robust_r`, `row`, `tolerance`, `series_info`, `lead_l`), and Task 2 `Branch`.
- Produces:
  - `bpf.Design(index, lo_hz, hi_hz, topology: dict, branches: tuple[dict, ...])` replaces `FilterDesign`/`Position`. `bpf.design(index) -> Design` reads the JSON (cached).
  - `bpf.add_filter(d: Design, f_in, f_out, gnd, q_ref="centre")`.
  - `parts.filter_choke(value, f_eval_hz, trim_iran=1.0, note=None) -> Part`.
    - Footprint: `Inductor_THT:L_Axial_L9.5mm_D4.0mm_P12.70mm_Horizontal_Fastron_SMCC` (BC) or `Inductor_THT:L_Axial_L7.0mm_D3.3mm_P10.16mm_Horizontal_Fastron_MICC` (78F).
    - Spec (both populations): `"{L} ±{tol} % axial ferrite-drum RF choke, body ≤ {d} × {len} mm, {pitch} mm lead pitch, Q≥{q} at {f_t}, SRF≥{srf}, DCR≤{dcr} Ω, I≥20 mA ({maker} {mpn} or equivalent)"`.
    - China variant: tol = the series grade, as bought, model at the nominal value.
    - Iran variant: tol 0.03, model × `trim_iran`, note "select … with an LCR meter".
    - SPICE: L + lead L, R = `robust_r(row, f_eval_hz)`, C_par.
  - The capacitor variants carry ESR = 1 / (2π·f_centre·C·500): `parts._capacitor_lines(farads, esr, 0.0, part)`.

**How `add_filter` builds each branch** (series branch: node to next node; shunt branch: node to gnd):
- `L`: a chain of `filter_choke`s (1–2, `select_group` and `select_target` as today).
- `C`: capacitor slots, as `_capacitors` today, from `c_china` / `c_iran`.
- `LC_par`: the choke chain and the capacitor slots between the same two nodes.
- `LC_ser`: the choke chain to a new mid node `f{n}_b{k}_m`, then the capacitor slots to the end node.

- [ ] **Step 1: Write the failing tests** (replace the old per-position assertions in `test_bpf_blocks.py`):

```python
def test_every_filter_inductor_is_a_through_hole_choke_in_both_populations():
    from neohw import population as pop
    from neohw.units import parse_si
    bank = bpf_bank.build(None)
    for p in (p for p in bank.circuit.parts if p.ref.startswith("L")):
        assert p.footprint.startswith("Inductor_THT:L_Axial_"), p.footprint
        h = parse_si(p.value[:-1])
        assert ("P12.70mm" in p.footprint) == (h >= 1e-6)
        china, iran = pop.variant(p, "china"), pop.variant(p, "iran")
        assert "±" in china.ident and "axial ferrite-drum RF choke" in china.ident
        assert "mm lead pitch" in china.ident and "L_" not in china.ident        # a body, not a footprint name
        assert iran.tol == 0.03 and china.tol in (0.05, 0.10)


def test_the_bank_is_built_from_the_committed_designs():
    import json
    doc = json.loads((bpf.DESIGNS).read_text())
    for f in doc["filters"]:
        d = bpf.design(f["index"])
        assert len(d.branches) == len(f["branches"]) and d.topology == f["topology"]
```

In `test_ladder.py`, add `test_fast_model_matches_ngspice_for_every_committed_design`. It is parametrised over 1..11 and marked `integration`. It uses the Step 3 helper of Task 2, applied to the bank built from the designs, with the same tolerances: 0.05 dB in band, 1 dB at > −60 dB.

In `test_bpf_check.py`, update `test_filter_8_meets_targets_with_monte_carlo` and `test_filter_8_meets_targets_in_both_populations` unchanged in intent (they now run on the new design). Also check that `test_worst_case_q_loss_is_reported_and_higher` still holds. If the worst-case run equals nominal for a filter wholly below its chokes' f_t, change that test to `>=` and say why in a comment.

- [ ] **Step 2: Run** `cd hardware && python3 -m pytest -q tests/test_bpf_blocks.py`. Expected: FAIL.

- [ ] **Step 3: Implement** as described:
  - `bpf.py`: `DESIGNS = Path(__file__).resolve().parents[1] / "data" / "filter_designs.json"`. Remove `RIPPLE_DB`, `LPF_ORDER`, `HPF_ORDER`, `LPF_MARGIN`, `_realise` and `Position` if nothing else uses them; check with `grep -rn` across `hardware/ tools/ sim/ site/`. `BANDS`, `EDGE_MARGIN` and `C_MIN` stay.
  - `build_sheet` in `checks/bpf.py` keeps working, since `select_group` / `select_target` are set by the choke chain.
  - Update the module docstrings to describe the robust design.

- [ ] **Step 4: Run** `cd hardware && python3 -m pytest -q`. Expected: PASS, the full suite. Then run a quick check of every filter in both populations:

```bash
cd hardware && python3 -c "
from neohw.checks import bpf as c
for i in range(1, 12):
    for p in ('china', 'iran'):
        r = c.evaluate(i, 20, p); il, rl, rej = r['nominal']
        print(i, p, f'{il:.2f} {rl:.1f} {rej:.1f} mc {r[\"mc_passed\"]}/20', 'ok' if c._passes(i, il, rl, rej) else 'FAIL')
"
```

Expected: every line ok, with mc ≥ 19/20. Paste it into the report. Any FAIL: stop with BLOCKED and the numbers.

- [ ] **Step 5: Commit** `git commit -m "feat(hw): filter bank built from the robust designs on through-hole chokes; robust inductor model is the only model"`

### Task 7: Full verification, characterisation and documents

**Files:**
- Modify: `hardware/characterized/bpf_bank.json`, `bpf_*.s2p` (regenerated), `docs/superpowers/specs/2026-10-18-rev-a-china-fab-design.md` (§2 filter rows), `docs/superpowers/specs/2026-10-19-robust-filter-bank-design.md` (Status: implemented, with results), `README.md` ("Simulated performance" filter row)

- [ ] **Step 1: Full Monte Carlo.**
  - Run `free -g` (stop if < 4 GB is free).
  - Then run `make hw`; only the changed blocks re-run.
  - Expected: every `bpf*` check PASS, both populations, MC ≥ 95 % of 200, `bank_mute_isolation_db` ≥ 40.
  - Any FAIL: stop, BLOCKED, with numbers.
- [ ] **Step 2: Reproducibility.**
  - `cp -r hardware/characterized <scratchpad>/char-before && make hw && diff -r <scratchpad>/char-before hardware/characterized`
  - Expected: no difference.
- [ ] **Step 3: The twin uses the new `.s2p` files.**
  - Run `make sim` and `make fw-test`.
  - Expected: pass. Record the MDS and blocker numbers before and after in the report.
- [ ] **Step 4: Documents.**
  - Spec §2 filter rows: the inductor row now says "through-hole axial chokes in both populations"; add a "Result (robust design)" note with a per-filter IL / RL / rejection / MC table for both populations, taken from `bpf_bank.json`.
  - Robust spec: Status "Implemented (plan 5.1a)", plus a short results section.
  - README: the filter row.
  - The ledger notes for plan 5.0 gain the filter-bank page changes: topologies, chokes, robust model, figures.
- [ ] **Step 5: Commit**
  - `git add hardware/characterized docs README.md`
  - `git commit -m "chore(hw): characterisation of the robust filter bank (both populations); spec and README results"`

## Self-review notes

- **Spec coverage:**
  - §2 robust model: Task 1 adds it, Task 6 makes it the only model.
  - §3 targets and margins: Tasks 3 and 4; Task 7 verifies.
  - §4 fast model: Task 2. Optimiser: Task 3. Snapping: Task 4. Committed designs and the tool: Task 5.
  - §5 parts, identification and footprints: Tasks 1 and 6. Assembly note: plan 5.4 fab notes.
  - §6 verification: Tasks 2, 6 and 7.
  - §7 out-of-scope items are respected (no band split without the user: Task 5 Step 4).
- **Plan 5.1 impact:**
  - Task 9 (`filter-bank.gif`) and Task 10 (full verification) run after this plan.
  - Plan 5.1's Task 3 code paths (`filter_inductor`, the combined footprints, the NL453232/NLV32 use in filters) are removed in Task 6. The data rows stay in `inductors.json` as records.
