# NeoRadio Phase 2: Front End, Mixer and Baseband as Code (Design Spec)

- **Date:** 2026-10-12
- **Status:** Draft for review
- **Parent spec:** `2026-10-12-neoradio-hf-receiver-design.md` (§4 front end, §5 LO/mixer/baseband/ADC, §9 L1/L2/L4)
- **Builds on:** Phase 1 (`neosim` digital twin, `NeoRadio.Dsp` + `neoradio-dsp`), merged to `main` 2026-10-12.
- **Scope:** 2a front end, 2b mixer and baseband, 2c digital twin with the real circuit. The Rev 0 coupon (layout, openEMS, Gerbers) is **phase 3**. Relay drive and control logic (PCF8574/ULN2003) are also phase 3.

## 1. Goal and success criteria

The receiver's RF and baseband circuits are written as Python code (SKiDL). From that code we generate SPICE and KiCad netlists, prove each block against its targets in ngspice, and feed the results into the digital twin so the twin measures the **real** circuit.

**Phase 2 succeeds when:**

1. Every block check in §4–§5 passes, including Monte Carlo (≥ 95 % of runs passing).
2. In the twin, with the hardware model, the **FT8 implementation loss is ≤ 1 dB** against the audio-only reference. The sweep uses the same method as phase 1: 1 dB steps, 20 trials, 95 % intervals shown.
3. **Blocker scenario:** an FT8 signal at −15 dB SNR on 20 m still decodes with a +60 dB (relative to the band noise in 2500 Hz) carrier at 15.4 MHz. That carrier lies inside the 12–16 MHz filter. The report shows the decode result and the measured degradation.
4. **MDS ≈ −130 dBm in 500 Hz** (parent §5), ±3 dB, measured in the twin with only thermal noise at the antenna (−174 dBm/Hz), preamp on, at 14.1 MHz and 50.1 MHz. MDS is the CW level that gives 3 dB (S+N)/N in a 500 Hz bandwidth.
5. A KiCad 9 netlist, a BOM with Iranian availability flags and an SVG diagram are generated for every block.

## 2. Decisions (agreed)

| Topic | Decision |
|---|---|
| Circuit capture | SKiDL 2.3 with our own part layer. One description produces both the KiCad netlist and the SPICE netlist. |
| Simulation | Our SPICE writer + ngspice batch mode (subprocess), not PySpice |
| Review artefacts | Code is the source of truth. Reviewers get an auto-generated SVG connection diagram (Graphviz), BOM and plots per block. No hand-drawn schematic. |
| Twin coupling | The twin reads committed characterisation files and never runs ngspice |
| 3f rejection target | **System ≥ 60 dB = filter ≥ 50 dB + Tayloe's inherent ≈ 9.5 dB** (the 3rd-harmonic response has amplitude 1/3). This amends parent §4. |

## 3. Code structure

```
hardware/
  pyproject.toml          deps: skidl==2.3.0, numpy, scipy, scikit-rf, pyyaml, matplotlib; dev: pytest
  neohw/
    parts.py              part constructors: R, C, L (Q + SRF model), D, NPN, Opamp, Fst3253, RelayContact, Port
                          each creates a SKiDL part (footprint, MPN, Torob key) carrying its SPICE model
    spice.py              SKiDL circuit → SPICE netlist text; analyses .ac/.noise/.tran/.sp; Monte Carlo variants
    ngspice.py            run ngspice -b in a temp dir, parse ASCII rawfile, raise NgspiceError with log on failure
    export.py             KiCad 9 netlist (skidl), BOM CSV (MPN, footprint, qty, Torob availability), Graphviz SVG
    availability.py       reads docs/superpowers/specs/2026-10-12-parts-availability-torob.json
    synth/filters.py      Chebyshev 0.1 dB HPF(5) + LPF(7) synthesis → snap to buyable values → value table
    blocks/               bias_tee, protection, attenuator, bpf (one band), bpf_bank, preamp,
                          lo_quadrature, qsd_mixer, baseband_amp; each a function returning named ports
    checks/               one module per block: analyses + targets → CheckResult list
    report.py             hardware HTML report (diagrams, BOM, plots, pass/fail)
    __main__.py           python -m neohw run [blocks…] --report … --characterized …
  models/                 vendor SPICE models committed to the repo (LM4562, BFR92A, Coilcraft/Murata inductors),
                          plus our datasheet-derived FST3253 and CS5361-input models
  characterized/          committed outputs for the twin (see §6)
  tests/                  pytest
```

**Make targets:** `make hw` (all checks → `build/hw/report.html` + refresh `hardware/characterized/`), `make sim` (twin, ideal + hardware), `make test` (everything).

**Setup:** the user installs ngspice with `sudo apt install ngspice` (Ubuntu candidate 45.2). Vendor model downloads may need the VPN; the files are committed once downloaded.

## 4. Front end (2a)

| Block | Design | Targets |
|---|---|---|
| **Bias-T (5 V)** | Choke chain 1 mH + 47 µH + 4.7 µH (each covers above the previous one's SRF), DC blocks, AO3401 switch + PTC (switching itself is phase 3) | Shunt impedance > 500 Ω across 0.1–54 MHz; insertion loss < 0.3 dB |
| **Protection** | GDT at the connector, 1 MΩ bleed, BAV99 anti-parallel limiter after the attenuator | Limiter IMD3 products at −10 dBm two-tone ≤ −100 dBm |
| **Attenuator** | 20 dB π-pad 61.9 Ω / 249 Ω (E96) on a G6K-2F DPDT; relay model = contact R + open-contact C from the datasheet | 20 ± 0.5 dB; return loss ≥ 18 dB; bypass loss ≤ 0.2 dB, 0.1–54 MHz |
| **BPF bank** | 10 filters (parent §4 table): 5th-order HPF + 7th-order LPF, Chebyshev 0.1 dB, 50 Ω; values snapped to E12 inductors (1008CS / LQW18 / 1210 wirewound) and E24 C0G capacitors; inductors modelled with Q and SRF; one G6K-2F per filter | In-band IL ≤ 2 dB; RL ≥ 14 dB; rejection at 3× the band's lower edge ≥ 50 dB; Monte Carlo 200 runs (L ±5 %, C ±2 %), ≥ 95 % pass |
| **Preamp** | BFR92A resistive-feedback stage, 5 V / ~25 mA, bypass relay | Gain 14 ± 1 dB over 1–54 MHz; NF ≤ 4 dB; OIP3 ≥ +25 dBm; unconditionally stable (K > 1, \|Δ\| < 1) from 0.1 to 500 MHz; RL ≥ 12 dB |

Filters 1–2 (LF/MW) use 1210 wirewound inductors with lower Q. The IL limit is relaxed to ≤ 3 dB there because band noise dominates.

## 5. Mixer and baseband (2b)

- **LO** (`lo_quadrature`): behavioural square waves with ~1 ns edges.
  - Direct-quadrature mode (Si5351, ≥ ~3.2 MHz).
  - ÷4 mode (74LVC74) with an edge-time mismatch from the datasheet.
  - Phase noise is not modelled in SPICE.
- **Mixer** (`qsd_mixer`): FST3253 model.
  - 4 voltage-controlled switches, R_on ≈ 4 Ω, C_off from the datasheet, 2-to-4 decode, switching times.
  - 4 sampling capacitors, nominally ~33 nF, final value chosen by the checks.
  - Single-ended 50 Ω RF input.
- **Baseband** (`baseband_amp`): per channel, one LM4562 as an instrumentation-style differential stage (I = C0 − C2, Q = C1 − C3) with a shared gain resistor.
  - Differential output → anti-alias RC → CS5361 differential input model.
  - Gain set so that a strong in-band signal sits ~6 dB below ADC full scale (≈ 20–30 dB).
- **Analyses:**
  - Transient, 1–2 ms windows, ~0.5 ns steps, parallel across cores.
  - NF = conversion loss (transient) combined with AC `.noise` of the baseband amplifier, driven by the mixer's real output impedance.

| Check | Target |
|---|---|
| Conversion gain | Chosen for the full-scale mapping; flatness ±0.5 dB across ±40 kHz |
| IIP3 (preamp bypassed) | ≥ +25 dBm |
| Raw image rejection (Monte Carlo + LO skew, p95) | ≥ 30 dB |
| 3f / 5f response | Within 1 dB of ideal 1/3 and 1/5 (−9.5 / −14 dB) |
| 2f response | ≤ −40 dBc |
| Test frequencies | One per band (centre of each filter), both LO paths; 1.9 MHz on ÷4 |

## 6. Characterisation files (the contract between `neohw` and `neosim`)

| File | Content |
|---|---|
| `bpf_<n>.s2p` | Touchstone S-parameters, 0.05–200 MHz, from ngspice `.sp` (nominal values) |
| `bpf_bank.json` | Band edges per filter, selection rule (LO → filter index) |
| `preamp.json` | Frequency grid with gain_db and nf_db; oip3_dbm; bypass_loss_db |
| `attenuator.json` | Attenuation and bypass loss |
| `mixer.json` | conv_gain_db; baseband response H(f) over ±60 kHz (complex samples); harmonic responses {1, 2, 3, 5} as complex gain including sign and sideband; iip3_dbm; image_rejection_db {median, p95} with the corresponding gain/phase error; dc_offset; adc_full_scale_dbm |
| `lo_phase_noise.json` | Si5351 phase-noise curve (dBc/Hz vs offset), hand-entered from the datasheet with the source noted |

Every file carries `schema_version`, the generating git commit and the block's check status. The twin refuses files whose `schema_version` it does not support.

## 7. Digital twin with hardware (2c)

**Scenario additions:**
- `receiver: {model: ideal | hardware, preamp: on|off, attenuator: 0|20}`. Default: `ideal`, which keeps phase 1 scenarios unchanged.
- Absolute levels: `noise: {type: awgn, dbm_per_hz: -150}` and stations given as `level_dbm`. SNR mode remains.

**Hardware path:** each station's contribution to the 96 kHz window is computed as follows.
- **In-window stations:** filter S21 at the station's RF frequency × attenuator × preamp gain × mixer conversion × H(f_bb). An image is added at the mirror frequency with the p95 imbalance.
- **Stations within the window of 3·LO or 5·LO:** filter S21 at the RF frequency × harmonic response (with sign/sideband) → folded into the window.
- **Strong out-of-window stations:**
  - IMD3 products from station pairs, using the preamp OIP3 and mixer IIP3, wherever they land in the window.
  - Reciprocal-mixing noise: station power + L(Δf).
  - They are attenuated by H(f_bb) and are otherwise not added.
- **Noise:**
  - Band noise is in absolute dBm/Hz.
  - Receiver noise = kT·(F − 1) at the antenna, from the cascaded NF (preamp, mixer, baseband).
- **ADC:** full-scale mapping from `adc_full_scale_dbm`; real clipping.
- The window rule and exclusion list from phase 1 still apply to everything not covered above.

**C# DSP addition:**
- `NeoRadio.Dsp.IqCorrector`: amplitude and phase correction, applied before demodulation.
- `neoradio-dsp` options: `--iq-gain-db`, `--iq-phase-deg`.
- The twin passes the known imbalance from `mixer.json`. Automatic estimation from the calibration tone belongs to the firmware phase.

**Report:**
- The phase 1 report plus an ideal-vs-hardware FT8 sweep.
- The blocker scenario.
- MDS measurement.
- A link to the hardware report.

## 8. Testing

- **Unit tests (pytest):**
  - Part-model netlist text.
  - SPICE writer on small known circuits (RC, LC).
  - Rawfile parser.
  - Filter synthesis against textbook prototype values.
  - E-series snapping.
  - Characterisation file schema.
  - Twin hardware path on synthetic characterisation data (e.g. a known 3f gain produces the expected folded tone).
- **Integration tests** (marked; need ngspice): each block's checks run and pass; `make hw` regenerates identical characterisation files from the same commit.
- **C#:** `IqCorrector` unit tests (a known imbalance is corrected to ≥ 60 dB image rejection); CLI option parsing and validation (finite values, ranges).
- Failure surfaces:
  - A missing ngspice or vendor model produces an error naming the file/tool and the fix.
  - An ngspice convergence failure includes the tail of the ngspice log.

## 9. Risks

| Risk | Mitigation |
|---|---|
| Long transient runs for the mixer | 1–2 ms windows, parallel workers, results cached by netlist hash |
| No vendor model for FST3253 | Datasheet-derived switch model, documented with sources; sensitivity check on R_on/C_off |
| Inductor Q/SRF data unavailable for a chosen value | Use the series' published curves; when unknown, use a conservative Q and flag it in the BOM |
| Filter 3f target missed with buyable values | Raise the LPF order to 9 for that band only, or narrow the band split; the check reports the margin |
| Behavioural twin misses an effect | Phase 3 Rev 0 measurements recalibrate the models (parent §9 L6) |

## 10. Out of scope

Rev 0 layout, openEMS, Gerbers (phase 3); relay drive/control logic (phase 3); Si5351 and firmware behaviour; automatic I/Q calibration; ITU-R P.372 noise models (absolute AWGN only in this phase).
