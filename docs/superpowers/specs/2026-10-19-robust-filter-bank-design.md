# Robust band-pass filter bank (Design Spec)

- **Date:** 2026-10-19
- **Status:** Direction approved by the user (2026-10-19); written spec awaiting review
- **Parent spec:** `2026-10-18-rev-a-china-fab-design.md` §2 (part changes), §6 (verification). This spec replaces the filter-inductor row of §2 and its "Result (2026-10-19)" note, which relied on a model this spec shows to be optimistic.
- **Plan:** 5.1a (written after this spec is approved). It runs before plan 5.1 Tasks 9 (filter-bank GIF) and 10 (full verification). Plan 5.1 Tasks 4–8 do not touch the filters and proceed in parallel.

## 0. Why

Rev A is ordered once, and no samples can be bought beforehand to measure inductor Q (user, 2026-10-19). Plan 5.1 Task 3 put the filter bank on real inductors with guaranteed Q, but its model joined the guaranteed Q at the datasheet's test frequency f_t to the DC resistance below f_t. That join is an interpolation, not a guarantee. Two things showed it is not safe:
- TDK's typical NL453232 curve shows much lower Q at 0.1–0.5 MHz. That curve is itself inconsistent with TDK's own self-resonance table, so it cannot settle the question either way.
- With twice the loss below f_t, filter 1 China fails (IL 9.9 dB, RL 13.6 dB).

The user asked for robust filters "whatever it takes, even with a lot more cheap components".

## 1. Decisions (user, 2026-10-19)

| Topic | Decision |
|---|---|
| Robustness rule | Every filter meets every target in both populations with each inductor at the **physical worst case**, and **25 % below its guaranteed Q** (§2) |
| Rejected rules | "8× the interpolated loss" (and 6×) asks for more loss than any genuine part can have, and no standard part meets it (all filters 2–8 fail even with through-hole chokes). The bound of §2 is the strictest physically meaningful rule; the 25 % derating is the margin on top |
| Filter design | Low-loss redesign: **elliptic (Cauer) sections** with transmission zeros on the 3f band, element values **found by optimisation** against the robust model, then snapped to buyable values and confirmed in ngspice |
| Inductors | **No SMD filter inductors (user, 2026-10-19).** Every filter inductor is a through-hole axial RF choke with a part number, lying flat, the **same part in both populations**: TDK BC **B78108S** (4 × 9.2 mm, 12.7 mm pitch) from 1 µH, **Bourns 78F** (2.79 × 7.11 mm, 10.16 mm pitch) below 1 µH |
| More parts | Allowed: extra sections, extra parts per position, larger chokes |

## 2. The robust inductor model (design and verification)

For each inductor, the series loss resistance follows from its datasheet row (guaranteed Q_min at f_t, DCR max, SRF min):

- R_t = 2π·f_t·L / (0.75·Q_min), i.e. Q derated by 25 %.
- **Below f_t:** R(f) = R_t. Wire resistance, skin effect and core loss do not grow as frequency falls, so the loss of a part meeting its datasheet cannot exceed R(f_t) there. This gives Q(f) ≥ 0.75·Q_min·f/f_t. The DC resistance is not used to lower R.
- **At and above f_t:** R(f) = R_t·f/f_t, i.e. Q is held at 0.75·Q_min. A real part's Q rises above f_t up to its peak, so holding it is conservative.
- **Self-resonance:** a parallel C for the SRF minimum.

This is `inductors.r_series` with `ROBUST_DERATE = 0.75`. The interpolated model of plan 5.1 Task 3 is removed. One model serves design, nominal checks, worst case and Monte Carlo.

**Capacitors.** C0G capacitors get a series loss of Q = 500 at the band centre, which is conservative for C0G. Capacitors have negligible loss compared with the inductors, so this is a small conservative term, not a design driver.

## 3. Targets (unchanged in kind; the model is stricter)

- **Insertion loss:** IL ≤ 3 dB for filters 1–7 and ≤ 2 dB for filters 8–11.
- **Filter 1:** keeps the system-based limit of 2026-10-19, IL ≤ 8 dB plus a noise-floor rise ≤ 1 dB over an ideal receiver in P.372 residential noise. The noise check never binds at these frequencies; the 8 dB cap is the real limit (stated so nobody reads it as a second safeguard).
- **Return loss:** RL ≥ 14 dB in band.
- **Rejection:** ≥ 50 dB over the 3f band (3·lo to 3·hi) where 3·lo > hi (filters 3–11).
- **Monte Carlo:** ≥ 95 % of 200 builds pass every target. Tolerances:
  - China, inductors as bought: the series' grade, e.g. TDK BC ±10 % (K) up to 39 µH, ±5 % (J) above.
  - Iran, inductors LCR-selected: ±3 %.
  - Capacitors: China ±1 % E24, Iran ±2 % E12.
- **Bank mute:** ≥ 40 dB.
- **Margins inside the optimiser:** RL ≥ 15.5 dB and rejection ≥ 53 dB nominal, so Monte Carlo has room.
- **Unchanged constraints:** the band edges (parent spec band table) and the 11-filter relay chain and bit map stay as they are. Splitting a band would change the relay chain and firmware, so it needs the user's approval and is proposed only if a filter cannot pass otherwise.

## 4. Synthesis by optimisation

**New `hardware/neohw/synth/ladder.py`:** a fast ABCD-matrix model of a ladder (series and shunt branches of L, C, series-LC and parallel-LC with the §2 losses) giving S21 and S11 against frequency in numpy.
- It is validated against ngspice on today's Chebyshev designs: |ΔS21| ≤ 0.05 dB in band and ≤ 1 dB at −50 dB.
- ngspice stays the authority for every reported number.

**New `hardware/neohw/synth/robust.py`:**
- **Topologies per filter:**
  - an elliptic high-pass section, order 3–5;
  - an elliptic low-pass section, order 5–7, with its transmission zeros (series parallel-LC "traps" or shunt series-LC branches) seeded inside the 3f band;
  - the Chebyshev designs of today as fallback seeds.
- **Optimiser:** scipy `minimize` (SLSQP), multi-start from the seeds.
  - Objective: minimise the worst in-band IL under the §2 model, plus a yield penalty from 50 quick Monte Carlo draws.
  - Constraints: RL ≥ 15.5 dB, 3f rejection ≥ 53 dB.
  - Element bounds: the buyable range (C 1 pF–100 nF; L 0.1–100 µH, the chosen series' range).
- **Snapping:** each element is realised with one or two buyable values. For inductors: two chokes in series where needed, and in Iran a pair LCR-selected to the target. Then a local re-optimisation of the remaining elements.
- **Output:** a design table per filter (topology, element values, realisation), committed as `hardware/neohw/data/filter_designs.json` with the optimiser's settings and seed. `blocks/bpf.py` builds the SKiDL circuit from that table, not from the closed-form synthesis.
- Re-running the optimiser is a tool (`tools/synth_filters.py`), not part of `make hw`. `make hw` checks the committed designs, so builds stay reproducible.

## 5. Parts and footprints

**No SMD filter inductors (user, 2026-10-19).** Every filter inductor, in both populations, is a through-hole axial choke lying flat:

| Range | Part (both populations) | Body, pitch | Equivalents (must fit the same footprint) |
|---|---|---|---|
| ≥ 1 µH | TDK BC **B78108S1xxx{K,J}000** (axial, ammo version ends in 9) | 4 × 9.2 mm max, **12.7 mm** | Fastron or Bourns axial RF chokes of the same body, Q min at the same test frequency, DCR max and SRF min |
| < 1 µH | Bourns **78F** (78FRxx{K}-RC) | 2.79 × 7.11 mm max, **10.16 mm** | Taiyo Yuden LAL03, generic colour-code axial chokes (AL0307 class) with the same Q min, DCR and SRF |

- **One part per position.** Both populations fit the same part, so the circuit is identical in both. China fits it as bought (series tolerance); Iran fits it LCR-selected (±3 %), or as bought where the design passes with the bought tolerance.
- **Data file.** `hardware/neohw/data/inductors.json` keeps TDK BC (axial codes) and Bourns 78F as the filter series, every row traceable to its datasheet. NL453232 and NLV32 rows may remain as data, but no filter uses them.
- **Choice per position.** The optimiser's realisation step picks the value or values. Two chokes in series are allowed where one value cannot reach the target.
- **Identification (fixes the Task 3 review).** The BOM specification states the physical package: "axial ferrite-drum RF choke, body ≤ 4.0 × 9.2 mm, 12.7 mm lead pitch" (or 2.8 × 7.1 mm, 10.16 mm). It never states a footprint name. Equivalents are listed only if they fit.
- **Footprints.** Two plain axial through-hole footprints, generated by `neohw/footprints.py` or taken from KiCad's `Inductor_THT` library where one matches (e.g. `L_Axial_L7.0mm_D3.3mm_P10.16mm_Horizontal_Fastron_MICC`). There are no combined SMD/axial footprints for the filters; the combined 1210/1812 footprints of plan 5.1 Task 3 are removed if nothing else uses them.
- **Layout consequences (plan 5.3).**
  - Lying axial chokes are rotated 90° from their neighbours.
  - They stay low enough for shield cans.
  - Leads are kept short and flush: at 30–54 MHz, lead inductance (≈ 1 nH/mm) matters for the 100–270 nH parts of filters 9–11, and the optimiser models the lead inductance of the chosen pitch.
- **Assembly.** China fits roughly 80–100 through-hole chokes by hand or selective soldering; the fab notes (plan 5.4) state this.

## 6. Verification

- `ladder.py` agrees with ngspice (§4) on the current designs and on every new design.
- **Monte Carlo:** every filter, both populations, under the §2 model, 200 builds, ≥ 95 %. This is the `make hw` check, as today.
- **The robust model is the only model**, so there is no separate "nominal model" result to confuse with it. The worst-case run (loss evaluated at the upper band edge) stays as a report.
- **Tests:**
  - the model equals the bound (R below f_t equals R_t; Q above f_t equals 0.75·Q_min);
  - every committed design's elements are buyable and present in the data file;
  - every footprint exists and equals its generator;
  - every BOM ident passes §2b, with no footprint names as packages.
- The spec §2 table and README "Simulated performance" are updated from the new characterisation. The parent spec's "Result (2026-10-19)" note is marked superseded.

## 7. Out of scope

Changing band edges or the number of filters (only with the user's approval, §3); measuring parts (not possible before ordering); air-core or hand-wound toroid inductors (not catalogue parts with part numbers; kept as a fallback idea only).
