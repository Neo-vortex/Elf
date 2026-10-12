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
4. **MDS ≤ −130 dBm in 500 Hz** (parent §5), measured in the twin with only thermal noise at the antenna (−174 dBm/Hz), preamp on, at 14.1 MHz and 50.1 MHz, and within ±1 dB of the MDS predicted by the twin's noise budget. MDS is the CW level that gives 3 dB (S+N)/N in a 500 Hz bandwidth. (Amended 2026-10-12, plan 2.5 prototype: the earlier "≈ −130 ±3 dB" would fail a receiver that meets the 8 dB system-NF target, which gives about −139 dBm; the prototype's budget predicts −141 dBm.)
5. A KiCad 9 netlist, a BOM with Iranian availability flags and an SVG diagram are generated for every block.

## 2. Decisions (agreed)

| Topic | Decision |
|---|---|
| Circuit capture | SKiDL 2.3 with our own part layer. One description produces both the KiCad netlist and the SPICE netlist. |
| Simulation | Our SPICE writer + ngspice batch mode (subprocess), not PySpice |
| Review artefacts | Code is the source of truth. Reviewers get an auto-generated SVG connection diagram (Graphviz), BOM and plots per block. No hand-drawn schematic. |
| Twin coupling | The twin reads committed characterisation files and never runs ngspice |
| 3f rejection target | **System ≥ 60 dB = filter ≥ 50 dB + Tayloe's inherent ≈ 9.5 dB** (the 3rd-harmonic response has amplitude 1/3). This amends parent §4. |
| BPF design (amended 2026-10-12, plan 2.2 prototype) | Chebyshev **0.02 dB** ripple, not 0.1 dB: two 0.1 dB sections in cascade give only ~11 dB return loss even with ideal parts. Each inductor is 1–2 E12 parts in series and each capacitor 1–2 E24 C0G parts in parallel: single E12 inductors land up to ~10 % off and give 6–10 dB return loss. Relay stays the plain G6K-2F (the -RF variants cost ~30× more on Torob). |
| BPF insertion loss (amended 2026-10-12) | **≤ 3 dB on filters 1–7** (LF to 20 m, where the preamp is bypassed because band noise dominates), **≤ 2 dB on filters 8–10**. This amends parent §4's 2 dB for the lower bands. |

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
| **Bias-T (5 V)** | Choke chain 4.7 µH + 47 µH + 1 mH from the RF line (axial chokes rated ≥ 150 mA; the 47 µH is damped by 2.2 kΩ, which removes a series resonance near 38 MHz between it and the capacitive 1 mH), 1 µF DC block, bias feed decoupled; AO3401 switch + PTC (switching itself is phase 3). The connector GDT and 1 MΩ bleed are part of this block | Shunt impedance > 500 Ω across 0.1–54 MHz; insertion loss < 0.3 dB; DC drop ≤ 0.5 V at 100 mA antenna current (1 mH is a radial drum choke: an axial 1 mH has tens of ohms DCR) |
| **Protection** | Two BAV99 in anti-parallel from the RF line (after the attenuator) to ground, each a series string of two diodes (amended 2026-10-12: a single unbiased pair gives −96.6 dBm IMD3; a biased pair pumped its bias ladder above 5 V under overload) | Limiter IMD3 products at −10 dBm two-tone ≤ −100 dBm; output peak ≤ 2.5 V for a +27 dBm input |
| **Attenuator** | 20 dB π-pad 61.9 Ω / 249 Ω (E96) on a G6K-2F DPDT; relay model = contact R + open-contact C from the datasheet | 20 ± 0.5 dB; return loss ≥ 18 dB; bypass loss ≤ 0.2 dB, 0.1–54 MHz |
| **BPF bank** | 11 filters (parent §4 table; 30–54 MHz split at 40 MHz, final review of plan 2.4): 5th-order HPF + LPF of order 9 (filters 1–6) or 7 (filters 7–11), Chebyshev 0.02 dB (filters 1–8), 0.01 dB (9, 11) and 0.005 dB (10) — lower ripple on the top filters, where ten open relay contacts load each bus with ~6 pF — 50 Ω, cutoffs 3 % outside the band edges. Each L = 1–2 E12 **axial colour-ring (moulded) inductors** in series, through-hole, **measured and selected with an LCR meter to ±3 %** (unselected ±10 % parts fail: return loss ≈ 11 dB at the 5th percentile), modelled as L + series R giving **Q = 30 at the band centre** (below the Vishay IM-2 / Bourns 78F class minimum of ~40; assumed, flagged in the BOM) + 2.5 pF parallel C for the SRF. No ferrite or iron-powder HF cores were found on Torob (only power ferrite, Q ≈ 15 → 3–4 dB loss). Each C = 1–2 E24 C0G 0805 parts in parallel, selected with the LCR meter to ±2 % (±5 % stock gives ~90 % Monte Carlo yield). **Two G6K-2F per filter** (amended 2026-10-12, final review of plan 2.2): an input relay and an output relay, pole 1 switching (COM = filter end, NO = bus, NC = GND), pole 2 grounded. One relay carrying both buses couples them through its inter-pole capacitance (not published for the plain G6K-2F; 0.1 pF assumed, twice the G6K-2F-RF's 1 GHz figure) and ×10 relays fails 3f rejection and mute isolation on the upper bands. | In-band IL ≤ 3 dB (filters 1–7) / ≤ 2 dB (8–10); in-band RL ≥ 14 dB; rejection ≥ 50 dB from 3× the lower to 3× the upper band edge (filters 3–10; for filters 1–2, 3× the lower edge lies inside the passband); Monte Carlo 200 runs (L ±3 % after selection, C ±2 %, uniform), ≥ 95 % pass |
| **Preamp** | BFR92A resistive shunt-feedback stage (Rf 360 Ω, Re 5.6 Ω), 47 µH collector choke, **active bias** (BC857B sensing ~1 V across 47 Ω, feeding the base through 2.2 kΩ from an RF-decoupled node) at ~20 mA, G6K-2F bypass relay. Amended 2026-10-12 (final review of plan 2.3): the BFR92A's absolute maximum is 25 mA, and the first design (25.8 mA at 5.25 V, 32 mA at high hFE) exceeded it; a 2N3866 was tried and rejected (NF 5.6–6.5 dB) | Over every corner of supply 4.75 / 5.0 / 5.25 V × hFE 65 / 94.7 / 150 × 0 / 27 / 50 °C: gain 14 ± 1 dB over 1–54 MHz; **NF ≤ 5.0 dB** (IEEE, 290 K source; 4.3 dB nominal); **OIP3 ≥ +22 dBm** (+23.9 dBm nominal; two tones at 14 and 50 MHz); collector current < 25 mA; unconditionally stable (K > 1, \|Δ\| < 1) from 0.1 MHz to 6 GHz (bypass relay's inter-pole C included). **Input** RL ≥ 12 dB at the nominal corner (it terminates the filter bank; output RL is reported, since the preamp drives the Tayloe mixer, not 50 Ω). System NF on 6 m stays ≤ 8 dB (≈ 1.7 + 4.9 + < 1 dB worst case) |

Prototype result (2026-10-12, Q = 30, L ±3 %): every filter passes 200/200 Monte Carlo runs; in-band IL ≤ 2.02 dB and RL ≥ 15.6 dB at the 95th/5th percentile; 3f rejection ≥ 54.9 dB nominal. Rev 0 must measure real inductor Q (NanoVNA), the relay inter-pole capacitance and bus-to-bus isolation on the board. With Q = 30 held at the upper band edge instead of the centre (reported as `il_worst_q_db`), filters 1, 2 and 10 exceed their IL limits (≈ 4.0, 3.1, 2.3 dB): measure those first. Iranian availability of axial inductors is not yet surveyed (no Torob key).

## 5. Mixer and baseband (2b)

Amended 2026-10-12 from the plan 2.4 prototype.

- **LO** (behavioural `QuadratureLO` stimulus): S0 and S1, two 50 % square waves at f_LO, S1 a quarter period later, 1 ns edges, 3.3 V.
  - Direct-quadrature mode (Si5351, ≥ 3.2 MHz): skew between S0 and S1 uniform ±0.2 ns in Monte Carlo.
  - ÷4 mode (74LVC74 Johnson counter, below 3.2 MHz): skew uniform ±0.5 ns.
  - Phase noise is not modelled in SPICE.
- **Mixer**: FST3253 behavioural model (`hardware/models/fst3253.lib`).
  - **VCC 4.0–5.5 V (datasheet): the FST3253 runs from 5V_A, not 3V3_RF** (this corrects parent §5/§7). Select inputs are TTL, so the 3.3 V clock chips drive them directly.
  - Switches are NMOS fitted to R_on = 4 Ω at V_in = 0 (so R_on rises with the signal level), A port 13 pF, B ports 5 pF; the 2-to-4 decoder is ideal fast switches (its delay mismatch is covered by the LO skew).
  - RF node AC-coupled and biased at 2.5 V (10 kΩ from a 10k/10k divider on 5V_A).
  - **4 sampling capacitors 5.6 nF C0G**, selected to ±1 % of each other with the LCR meter. (33 nF puts the RC corner near 22 kHz: −6 dB at 40 kHz.)
  - Single-ended 50 Ω RF input. (S1, S0) = 00, 01, 11, 10 selects B0, B1, B3, B2, so I = B0 − B3 and Q = B1 − B2.
- **Baseband**: per channel, one LM4562 as the front half of an instrumentation amplifier (Rf 2.2 kΩ, Rg 240 Ω, gain 19.3 = 25.7 dB), passing the 2.5 V common mode to the CS5361 (VQ); anti-alias 100 Ω + 2.7 nF differential; CS5361 input 7.5 kΩ differential.
  - LM4562 model: datasheet macromodel (`hardware/models/lm4562.lib`; TI's PSpice model could not be fetched by script and is reported to have swapped inputs). **On 5 V the LM4562 is at its minimum supply (±2.5 V) and swings about ±1 V per output: it, not the ADC, sets full scale** (≈ −20 dBm at the mixer input, about 3 dB below ADC full scale). Phase 3 power must deliver ≥ 5.0 V at the op-amps. No rail-to-rail low-noise op-amp (AD8656, OPA2350, LT6231) was in stock on Torob.
- **Analyses:** transient, 1 ms window after a 30 µs settle, output on a 0.25 µs grid, solver step ≤ T_LO/280, **Gear integration** (trapezoidal integration rings on the switched capacitors and the run time explodes). Tones on a 3 kHz offset grid, Hann window.
- **Mixer NF** (amended 2026-10-12, plan 2.5 prototype). AC `.noise` cannot run through the switching mixer, so the noise run uses a linear stand-in: the FST3253 is replaced by four branches from the RF node (held at the 2.5 V bias) to the sampling caps, each 4·R_source (noiseless) in series with 4·R_on (noisy; a conductance on 25 % of the time has the noise of a 4·R_on resistor). R_on is measured from the FST3253 model at the 2.5 V bias: **7.6 Ω** (the model comment's 13 Ω was wrong). Op-amp voltage noise (the macromodel's 440 Ω), Rf, Rg, the anti-alias R and the CS5361 7.5 kΩ are noisy.
  - Definition (SSB, referred to the 50 Ω input; harmonic folding of the source noise excluded, because the filters and the twin handle 3f/5f separately): a tone of available power P gives |z|² = 2·R·G²·P in z = I − jQ (G = conversion gain, linear). The source's noise at the fundamental gives each channel 2·R·G²·k·T0 per Hz, and an I/Q receiver's noise factor is that of one channel: **F = 1 + (N_I + N_Q) / (4·R·G²·k·T0)**, with N_I, N_Q the added one-sided output noise densities averaged over 1–40 kHz. (A planning note had 2·R·G² in the denominator, which counts the excess noise twice.)
  - Consistency check: with the source made noisy, the stand-in's source noise is 1.35× the fundamental-only value. That is π²/8 = 1.23 (odd-harmonic folding of white noise in a sampling mixer) times the 0.47 dB by which the transient conversion gain falls below ideal sampling theory. The check gates this ratio within ±1 dB of π²/8.
  - Prototype result: **mixer NF 7.2 dB** (14.2 MHz conversion gain). The CS5361's own noise is not in this figure: the twin adds it from the datasheet (108 dB dynamic range in 40 kHz, double-speed mode, typical).
- **Receive-chain interfaces** (plan 2.5 prototype, full transients of filter bank → mixer at every band centre and preamp → mixer at 14.2 and 46.5 MHz). The mixer was characterised from a 50 Ω resistive source. The Tayloe switch also draws current at the LO harmonics, and a band-pass filter's output impedance there is reactive. With the preamp bypassed, the filter → mixer cascade therefore runs **1.1–1.75 dB below** the product of the filter's S21 and the mixer's conversion gain on bands 3–11 (0.6 dB on band 2, 0 on band 1), with band 1's in-window flatness rising from 0.44 to 1.29 dB. The preamp's broadband ~55 Ω output keeps preamp → mixer within ±0.35 dB of the product. Correcting the product for the preamp's output impedance did not help consistently, so it is not used. Consequences:
  - `make hw` runs the bank → mixer cascade at every band centre and writes the complex ratio of the cascade's response to the product model, per baseband offset, into `mixer.json` (`bypass_correction`). The twin applies it whenever the preamp is bypassed and adds it to the noise budget as a noiseless loss in front of the mixer.
  - Harmonic responses through the cascade are **not** usable from these transients. A 5f tone through filter 7 appeared at −67.6 dBc, but this is integration error at the switch edges on filter nodes carrying a large 5f voltage: it scales with the tone, and drops to −87 dBc at reltol 1e-6 and to −83 dBc with half the time step, while the filter alone gives the AC value (−111 dB) in transient. System 2f/3f rejection stays the product of the filter's S21 and the mixer's harmonic response, as in plan 2.4. Plan 2.6 re-checks the mixer IIP3 (IMD3 at −85 dBc, slope 1.81 dB/dB) at reltol 1e-6.
  - With the preamp on (bands 8–11), the system 2f/3f check still ignores the preamp: its gain falls by 0.02–1.6 dB from f to 2f/3f/5f (0.1–300 MHz grid), so leaving it out is conservative.
  - Cost: the 11 bank → mixer transients raise `make hw` from about 15 to about 40 minutes (20 cores).
  - The preamp + mixer transient converges only with `.options method=gear maxord=2 rshunt=1e9`. The bank + preamp + mixer transient still fails (time step too small in the BFR92A) at 25.7 and 46.5 MHz, so no chain with the preamp is in `make hw`.

| Check | Target |
|---|---|
| Conversion gain | Reported (≈ 36 dB per-channel differential at the ADC pins vs the matched source amplitude); flatness within ±0.5 dB of the mean across ±40 kHz at every test point |
| IIP3 (preamp bypassed) | ≥ +25 dBm, measured on the mixer core (sampling capacitors) with the IMD3 slope reported (2 dB/dB = third order); the op-amp model is linear |
| Raw image rejection (Monte Carlo of cap mismatch + LO skew, 40 runs per LO path, 5th percentile) | ≥ 30 dB |
| 3f / 5f response | Within 1 dB of ideal 1/3 and 1/5 (−9.5 / −14 dB) at test points ≥ 1.7 MHz. Below ~1 MHz the sampling RC (≈ 1.2 µs) is no longer much shorter than T_LO and the response departs from ideal (band 1: 3f −5.2 dB, 5f −8.6 dB); bands 1–2 are reported in `mixer.json` and have no 3f filter target |
| 2f response | Mixer alone reported (≤ −40 dBc up to 26 MHz; −37.9 dBc at 40 MHz, set by the 1 ns edges and the 1.5 V TTL threshold) |
| System 2f and 3f rejection | ≥ 60 dB on bands 3–10: filter attenuation at k·f_LO (from `bpf_<n>.s2p`) plus the mixer's k-th harmonic response. This is the parent §4 3f target made concrete, extended to 2f |
| Test frequencies | One per band (geometric centre of each filter): ÷4 below 3.2 MHz, direct above; plus 1.9 MHz on ÷4 |

## 6. Characterisation files (the contract between `neohw` and `neosim`)

| File | Content |
|---|---|
| `bpf_<n>.s2p` | Touchstone S-parameters, 0.05–200 MHz, from ngspice `.sp` (nominal values) |
| `bpf_bank.json` | Band edges per filter, selection rule (LO → filter index) |
| `preamp.json` | Frequency grid with gain_db and nf_db (nominal corner; 0.1–300 MHz since plan 2.5, so the twin can fold the preamp's output noise at 3f and 5f; the checks stay on 1–54 MHz); oip3_dbm; bypass_loss_db |
| `attenuator.json` | Attenuation and bypass loss |
| `mixer.json` | conv_gain_db; baseband response H(f) over ±60 kHz (complex samples); harmonic responses {1, 2, 3, 5} as complex gain including sign and sideband; iip3_dbm; image_rejection_db {median, p5} with the corresponding gain/phase error; dc_offset; adc_full_scale_dbm. Plan 2.5 adds: nf_db with its definition, ron_ohms and the stand-in noise densities; `bypass_correction` per band; adc_fs_peak_v (CS5361 full scale), clip_peak_v (op-amp swing, ±1 V per output) and adc_noise (dynamic range and its bandwidth) |
| `lo_phase_noise.json` | Si5351 phase-noise curve (dBc/Hz vs offset), hand-entered with the source noted. Amended 2026-10-12 (plan 2.6): the Si5351 datasheet (Rev. 1.3) gives only period and cycle-to-cycle jitter (70 ps pk-pk typical), no phase-noise curve. The file holds the RFzero measurement of an Si5351A at 9 MHz on a Rohde & Schwarz FSIQ 26 with the low-phase-noise option (−103, −128, −136, −138, −144 dBc/Hz at 100 Hz, 1 kHz, 10 kHz, 100 kHz, 1 MHz), cross-checked against the ARRL QST review of the QRP Labs QCX (Si5351A-driven, better than −135 dBc/Hz at 10 and 50 kHz). The twin scales it by 20·log10(f_LO / 9 MHz) and holds it flat outside 100 Hz–1 MHz; the ÷4 path scales the same way |

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

**Amended 2026-10-12 (plan 2.5 prototype):**
- Plan 2.5 builds the in-window path, LO-harmonic folding, the noise budget, the ADC and the I/Q imbalance. Plan 2.6 adds IMD3, reciprocal mixing, `lo_phase_noise.json`, the C# `IqCorrector`, the blocker and MDS scenarios and the ideal-vs-hardware sweep. Until then, out-of-window stations are excluded with a reason, as in phase 1.
- Signals are complex envelopes in volts (|x| = sqrt(2·50·P)), and the ADC works in volts: clip at the op-amp swing, then the CS5361's noise, full scale and 24-bit quantisation.
- Noise budget: Friis over attenuator (or its bypass), filter (passive at T0), preamp (or bypass loss), the bypass correction (preamp off) and mixer NF. With the preamp on, its output noise at k·LO is folded in through the mixer's k-th harmonic response. Prototype: NF 5.9 dB at 14.1 MHz with the preamp (0.4·kT0 of it folded in from 3f/5f) and 8.7 dB without it (before the bypass correction). The twin's output noise floor matched this budget within 0.1 dB at 14.1 and 50.1 MHz.
- The image uses the 5th-percentile Monte Carlo run's gain and phase error for the LO path in use (direct ≥ 3.2 MHz, ÷4 below), in neohw's convention: q = −Im(z·e^{jφ})/g.
- Stations near k·LO are folded with the product model; the filter data ends at 208 MHz, so a harmonic station beyond it is excluded with a reason.
- Scenarios: `noise.dbm_per_hz` makes a scenario absolute (stations then need `level_dbm`, the power while active). The hardware model requires absolute levels and rejects `adc` settings; the ideal model converts absolute levels to SNR in 2500 Hz.
- The twin refuses characterisation files that are missing, carry another schema_version, or have status `fail` (naming the failed checks).

**Amended 2026-10-12 (plan 2.6 prototype):**
- IMD3: every ordered station pair (i, j) whose product 2·f_i − f_j lands in the window adds c·u_i²·conj(u_j) at the product frequency, where u are the stations' unit-power envelopes. The mixer term uses its IIP3 at its input (mixer.json iip3_dbm) and the levels there; with the preamp on, a preamp term uses IIP3 = OIP3 − gain at its input. The two terms add with the phases of the front-end gains, which for a flat front end is in phase (the worst case). The mixer IIP3 was re-run at reltol 1e-6: +32.51 dBm, slope 1.817 dB/dB, against +32.48 dBm, 1.814 at 1e-4, so the check keeps its tolerance.
- Reciprocal mixing: each station whose frequency the filter data covers adds noise of density P_mixer·L(f_s − f_LO − f_bb) at baseband frequency f_bb, in-window stations included.
- A station whose power at the mixer input exceeds the full-scale level (adc_full_scale_dbm) is reported as an overload risk. The baseband response is characterised only to ±60 kHz, so clipping by out-of-window stations is not simulated.
- MDS is measured on the twin's I/Q: a −110 dBm carrier 10 kHz from the LO; the noise density 2–20 kHz from the tone, excluding the DC offset and the tone's image; MDS = level − 10·log10(S / N_500). Prototype: −140.5 dBm at 14.1 MHz and −140.6 dBm at 50.1 MHz (preamp on; budget −140.5 dBm, receiver NF 6.4 dB).
- Blocker scenario (`20m_blocker_hardware.yaml`, preamp off): the FT8 signal at −15 dB still decodes; the window's noise floor rises by 0.00 dB. The carrier's reciprocal mixing (about −201 dBm/Hz) is far below the −155 dBm/Hz band noise.
- FT8 sweep, hardware model (20 m, band noise −155 dBm/Hz, preamp off, the known imbalance corrected): implementation loss +0.5 dB against the audio-only reference (ideal receiver −0.1 dB), 1 dB steps, 20 trials.
- The twin passes the I/Q imbalance it applied (the 5th-percentile run for the LO path) to `neoradio-dsp --iq-gain-db/--iq-phase-deg`. Gain is limited to ±6 dB and phase to ±30°.
- `make sim` runs the sweep with the hardware curve and the hardware checks (MDS, blocker).

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
