# Elf proving ground: pre-production sign-off in a real-world environment (Design Spec)

- **Date:** 2026-10-18
- **Status:** Design approved section by section by the user (2026-10-18); written spec awaiting review
- **Parent spec:** `2026-10-18-rev-a-china-fab-design.md` (Rev A). This spec adds a sign-off gate between layout (plan 5.3) and the fab package (plan 5.4), and absorbs that spec's §9a (physical-antenna end-to-end test), which moves from "after Rev A" to "before ordering".
- **Why:** there is one shot at the Rev A board (user, 2026-10-18). Block checks and the twin's idealised scenarios (white noise, one blocker, a perfect board) are not enough; the board must be shown to work in the environment it will actually be used in, with the board's own parasitics and emissions, before it is ordered.

## 0. Decisions (user, 2026-10-18)

| Topic | Decision |
|---|---|
| Environments | **City apartment** and **suburban house** (ITU-R P.372 "city" and "residential" man-made noise). Quiet rural and "next to a broadcast site" are out of scope |
| Antennas | **Active mini-whip on a balcony** (our own design, §9; fed by Elf's 5 V / 12 V bias-T) and **random wire / end-fed** from a window (9:1 unun) |
| Missions | **FT8/WSPR, CW, SSB voice, AM broadcast**: each an end-to-end pass/fail test scored on decoded output |
| Compute | **Overnight sign-off** (8–12 h, full campaign) plus a **10–20 min daily subset** in `make sim`; heavy EM results cached like characterisation files |
| Approach | **Synthetic, calibrated environments gate fab**; real KiwiSDR city recordings run as a **non-gating realism cross-check** |
| Levels | Worst-case levels of §3 approved as proposed |
| Targets | Mission targets of §6 approved as proposed. As everywhere in this project, a target is never loosened silently: a failure is reported with numbers and the user decides |
| LO phase noise | The "real LO phase-noise check" raised during plan 5.1 is not added to 5.1: it is PG-2 here (per population, against every strong station) |

## 1. Pipeline

```
5.1 parts → PG-1 environment, PG-2 receiver realism, PG-3 missions + gate (subset running)
         → 5.2 STM32F411 → 5.3 layout + PG-4 antennas (openEMS) + PG-5 post-layout
         → overnight sign-off (must pass) → 5.4 fab package
```

Each PG sub-project gets its own plan (written with the writing-plans skill, numbered 6.1–6.5 or folded into 5.x plans where noted below).

| # | Sub-project | Depends on | Plan |
|---|---|---|---|
| PG-1 | Environment generators | the twin | 6.1, after 5.1 |
| PG-2 | Receiver realism without layout | 5.1 parts | 6.2, after 5.1 (parallel with 5.2) |
| PG-3 | Missions, scorers, campaign runner, gate | PG-1, PG-2 | 6.3, after 5.1 |
| PG-4 | Antennas and surroundings in openEMS | openEMS Python bindings | 6.4, alongside 5.3 |
| PG-5 | Post-layout extraction and self-emission | 5.3 layout, PG-4 | 6.5, after 5.3, before 5.4 |

Code lives in `sim/neosim/proving/` (environment, missions, campaign, report) and `hardware/em/` (openEMS studies, shared with plan 5.3). Results: `hardware/characterized/` for cached EM and extraction results; `build/proving/` for campaign output.

## 2. Architecture

```
scenario YAML (seeded) ──► PG-1 environment: field at the antenna, per station + noise sources
                              │
                              ▼
               PG-4 antenna transfer (plane wave → 50 Ω input) + coax common-mode pickup
                              │
                              ▼
   PG-5 self-emission (board → antenna, conducted on shield) ──► sum at Elf's input
                              │
                              ▼
   twin hardware model (existing) + PG-2 realism (phase noise, corners, whole-band loading)
     + PG-5 extracted parasitics (re-run block checks feed the twin's characterisation)
                              │
                              ▼
   virtual board (existing; real firmware incl. attenuator/preamp policy) over USB/IP
                              │
                              ▼
   host DSP + decoders (ft8_lib, wsprd, CW, SSB/AM demod) ──► PG-3 scorers ──► signoff.json
                              ▲
   same scenario through an IDEAL receiver (perfect hardware, same antenna) ─┘  (reference)
```

Units and interfaces:
- `proving.environment.render(scenario, f_lo, n, fs, rng) -> Field`: complex baseband per source at the antenna reference plane (V/m-equivalent), each source tagged by kind.
- `proving.antenna.apply(field, antenna_id, choke: bool) -> np.ndarray`: volts into 50 Ω, from the stored PG-4 transfer functions.
- `proving.selfemission.render(antenna_id, f_lo, n, fs) -> np.ndarray` (PG-5).
- `proving.missions.<name>.score(decoded, truth) -> MissionScore` with `value`, `limit`, `passed`, `mechanism` (§7).
- `proving.campaign.run(level: "subset" | "full") -> SignoffDoc`, written to `build/proving/signoff.json`.
- An **ideal receiver** path reuses the twin with `Hardware.ideal()` (noise-free, linear, perfect I/Q) and the same antenna, so every target can be stated relative to it.

## 3. PG-1: environment (approved levels)

- **Background noise:** ITU-R P.372 noise factor Fa = c − d·log10(f / MHz). Man-made (P.372 Table 2): city c = 76.8, d = 27.7; residential c = 72.5, d = 27.7. Galactic noise (P.372) and atmospheric noise (P.372 maps, Tehran latitude, summer night: the worst case) are added as a power sum. The implementer reads the coefficients and maps from the Recommendation text, not from memory, and records the edition.
- **Impulsive and local interference** (each 10–20 dB above the P.372 city floor in its band, the bad end of urban surveys):
  - switch-mode supply combs: fundamentals 50–300 kHz, harmonics with drift and jitter;
  - power-line arcing: 100 Hz-gated bursts, Middleton Class A statistics;
  - PLC/VDSL leakage: OFDM-shaped broadband noise 2–30 MHz with notches in the amateur bands;
  - LED/plasma-display broadband buzz.
- **Band occupancy (city worst case):**
  - local MW AM station (100 kW, ≈ 5 km): **−5 dBm** at the end-fed's input; the mini-whip level follows from its PG-4 transfer;
  - SW broadcasters: up to **−20 dBm** each, 20 carriers across 41/31/25 m;
  - amateur bands: populations from S1 to S9+40 (−120 to −33 dBm).
- **Neighbour transmitter in the building:** 100 W at ≈ 30 m, **−10 dBm** at Elf on the same band; phase noise −130 dBc/Hz at 10 kHz (modern) and −110 dBc/Hz (old rig) variants; IMD3 splatter −30 dBc; hard-keyed CW clicks.
- **Propagation:** Watterson channels per ITU-R F.1487, good / moderate / poor (delay spread 0.5 / 1 / 2 ms, Doppler spread 0.1 / 0.5 / 1 Hz); the path model of the former §9a (ITU-R P.368 ground wave or a simple skywave model) sets wanted-station levels where a scenario gives a distance instead of a level.
- **Realism cross-check (non-gating):** 10 min recordings from 3–5 public KiwiSDRs in large cities on 40 m and 20 m (open-licence sources as in `2026-10-13-recorded-antenna-design.md`). Compare noise floor and amplitude probability distribution (APD) with the synthetic city; > 6 dB floor difference or a clearly different APD opens a recalibration task. Reported, never gating.
- Scenarios are seeded YAML in `sim/scenarios/proving/`, extending the twin's scenario format.

## 4. PG-2: receiver realism without layout

- **LO phase noise per population:** Si5351 measured curve (`lo_phase_noise.json`) combined with the reference's own noise (China: TCXO datasheet curve; Iran: crystal model), the larger governing at each offset; scaled 20·log10(f_LO / f_ref) and applied as reciprocal mixing against **every** strong station of the environment.
- **Corners:** 0 / 25 / 50 °C × supply min/max (USB 4.75–5.25 V; DC jack range ends) × tolerance Monte Carlo (50 builds per corner overnight, 5 in the subset), both populations. Per-build gains, filter shapes and I/Q imbalance come from the SPICE models into the twin.
- **Whole-band loading:** the sum of every station 0.1–54 MHz at once through limiter, preamp compression, mixer switch headroom and ADC full scale; intermodulation products computed and placed where they land in the passband.
- **Static and surge on the end-fed:** rain static and an IEC 61000-4-5 1 kV surge: GDT, limiter and BAV99 survive (SPICE, within ratings) and the receiver recovers within 1 s (virtual board).

## 5. PG-4 and PG-5: antennas, surroundings, the real board

**PG-4 (openEMS):**
- Mini-whip on a 1.5 m pole at a balcony railing, 0.5 m from a concrete wall (εr 6 with rebar mesh), 15 m coax.
- End-fed wire 10 m from a window, 9:1 unun, 5 m coax indoors.
- Each antenna: transfer function plane wave (azimuth, elevation, polarisation) → Elf's 50 Ω input over 0.1–54 MHz, and the **coax shield common-mode pickup** of the house noise field; each with and without a common-mode choke (the site recommends one from the result).
- Stored as characterisation files (`hardware/characterized/antenna_<id>.json` + data), regenerated only when the geometry changes.

**PG-5 (after the 5.3 layout):**
- Extraction (openEMS + KiCad geometry): every 50 Ω line's impedance and loss; LO-to-antenna-input and filter-to-filter coupling; stray C/L of traces where they matter. Inserted into the SPICE decks as small networks; **all `make hw` checks re-run on the extracted board**, both populations.
- Self-emission: buck converter (52 kHz and harmonics, switch node), USB full-speed, STM32 clock and HSE, I2S lines; each source's spectrum from its own circuit model; coupled radiated (board → antenna, in the PG-4 geometries) and conducted (coax shield) into the twin as interference at the antenna input.

## 6. PG-3: missions and targets (approved)

Every mission runs in city and residential environments, both populations, every corner. "Ideal" is the same scenario through the ideal receiver (§2).

| Mission | Scenario | Target |
|---|---|---|
| FT8 | 40 m and 20 m, crowded band, neighbour transmitting 2–5 kHz away | decode threshold within **1 dB** of ideal; ≥ **95 %** decoded at −18 dB SNR (2.5 kHz); **0** false decodes in 1 h of noise |
| WSPR | 30 m and 20 m | threshold within **1 dB** of ideal (`wsprd` added as a decoder) |
| CW | S3 station 1 kHz from an S9+40 station, impulsive noise on | weak-station CER ≤ **5 %** and ≤ ideal + 2 percentage points |
| SSB | S5 station, neighbour at ±3 kHz, "moderate" fading | STOI ≥ ideal − **0.05** and ≥ **0.75** |
| AM | SW broadcaster at S9+20 with the −5 dBm local MW station present | audio SNR ≥ **30 dB**; no image or IMD product above **−40 dBc** in the audio |
| Sensitivity | each band, residential noise | noise-floor rise over ideal ≤ **1 dB** up to 30 MHz, ≤ **3 dB** at 50 MHz |
| Overload | whole-band worst case | the firmware's attenuator/preamp policy on the virtual board keeps the ADC out of clipping; recovery after the surge ≤ **1 s** |

## 7. The gate and the report

- The overnight campaign writes `build/proving/signoff.json`: per mission × population × corner the value, limit, PASS/FAIL, and the **mechanism** of any failure, plus hashes of its inputs (SKiDL netlist, PCB file, characterisation files, firmware image, scenario files).
- `make fab` (plan 5.4) refuses to build the package when `signoff.json` is missing, has any FAIL, or is older than any hashed input.
- The HTML report (published on the site's Reports page) explains each failure with the analytic budget: which source, which path, which receiver stage (e.g. "reciprocal mixing from the neighbour at +10 kHz on 7 MHz", "buck harmonic 3 into the mini-whip at 14.2 MHz").
- `make sim` runs the subset (10–20 min) on every change; `make proving` runs the full campaign.
- Media (spec §7a): a GIF of the city band (waterfall through the environment, ideal vs Elf) is added to `docs/media/rev-a/`.

## 8. Testing the testers

No model is trusted before it passes its own check:
- noise generator PSD equals P.372 within 0.5 dB per band;
- impulsive generators match their target APD;
- fading matches the F.1487 Doppler spectrum;
- openEMS first reproduces a short monopole's analytic radiation resistance within 10 %;
- line extraction first reproduces a coplanar line's analytic impedance within 5 %;
- every scorer gives its best score on a clean known signal and its worst on noise alone;
- the ideal receiver meets each mission's absolute targets (otherwise the scenario, not Elf, is wrong).

## 9. Active antenna: our own mini-whip and a 5 V / 12 V bias-T (user, 2026-10-18)

The user will use a remote mini-whip. Its amplifier sits at the antenna (a coax run in between would load the whip's ≈ 10–20 pF source by ≈ 100 pF/m), so it is a **second board of our own design**, ordered with Elf; Elf feeds it over the coax.

**Elf side (Rev A circuit change, plan 5.1b, after 5.1 and before 5.2):**
- **Bias-T slide switch OFF / 5 V / 12 V** beside the BNC (silkscreen). The switch is the master: in OFF the feed is never powered. The firmware keeps its enable, current sense and short-circuit cut-off as a safety layer, reads the switch position (one GPIO/ADC line) and reports it in telemetry.
- **12 V:** a boost converter 5V_MAIN → 12 V, enabled only in the 12 V position (works on USB power too); fixed switching frequency, LC π output filter, layout keep-out. PG-5 must show its harmonics below the band noise at the antenna input.
- **12 V current limit:** the SY6280 is a 5.5 V part; the 12 V path gets its own current-limited switch rated ≥ 15 V (eFuse class). Current is sensed through the existing shunt and INA180 / mirror; the sense tables gain a 12 V calibration.
- **Budget:** a 12 V mini-whip at 60–80 mA costs ≈ 0.2–0.25 A at 5 V; on USB it needs a USB-C source offering 1.5 A, or the DC jack (as the README already says for active antennas).
- **DC input range 9–20 V** (user, 2026-10-18; was 9–30 V). The TVS is re-chosen for 20 V (≈ 22–24 V stand-off instead of SMBJ30A); done in plan 5.1 Task 5 with the rest of the power block.

**Mini-whip board (plan 6.6; PCB in the plan 5.3 layout flow; 2 layers):**
- Probe: copper plate ≈ 3 × 5 cm on the PCB, in a plastic tube on a 1.5 m pole.
- JFET high-impedance buffer (J310 / BF862 class) → push-pull emitter follower; designed for OIP3 ≥ +30 dBm at 12 V; also works at 5 V with less headroom (both voltages are simulated and reported).
- MW high-pass ≈ 1.8 MHz, selectable by a solder jumper; the proving ground decides the default (MW reception versus overload from the local MW station).
- GDT and diode clamp at the probe; ferrite common-mode choke at the coax feed point.
- Same flow as Elf: SKiDL circuit, SPICE checks (gain, noise, IP3, stability, both supply voltages, both populations where parts differ), openEMS probe model inside the PG-4 balcony scene, its own BOMs and fab files inside the fab package.

## 10. Out of scope

Quiet rural and broadcast-site environments; transmit; antennas other than the two of §0; measuring the real board (after fab); real-time operation of the campaign.
