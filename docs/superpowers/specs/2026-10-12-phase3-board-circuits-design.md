# NeoRadio Phase 3: Board Circuits as Code (Design Spec)

- **Date:** 2026-10-12
- **Status:** Draft, written for autonomous execution (user instruction: work phase by phase)
- **Parent spec:** `2026-10-12-neoradio-hf-receiver-design.md` (§5 LO, §6 controller and USB, §7 power, §10 parts)
- **Builds on:** phase 2 (`2026-10-12-phase2-frontend-mixer-design.md`): RF chain, mixer, baseband and digital twin, merged 2026-10-12.
- **Scope:** every remaining circuit of the receiver board, written in SKiDL like phase 2, with block checks in `neohw` and BOM availability. Layout (KiCad), firmware, host software and the virtual USB board are later phases.

## 1. Goal and success criteria

**Phase 3 succeeds when:**

1. Every block below exists as a SKiDL block with a KiCad netlist, a BOM whose ICs and semiconductors are all confirmed on Torob, and an SVG diagram, and its checks pass in `make hw`.
2. A board-level netlist connects all blocks (phase 2 RF chain included). Net-level checks pass: every IC supply pin is on its rail, every I2C address is unique, every ESP32-S3 GPIO is valid for its use, and no net has a single connection.
3. The power budget, computed from the parts' datasheet currents, fits USB-only operation (≤ 500 mA) and leaves margin.
4. The LO path delivers quadrature from 0.1 to 54 MHz. A frequency planner in Python (the reference the firmware mirrors) finds a valid Si5351C setting for every LO on a 1 kHz grid and picks the direct or ÷4 path by the crossover below.

## 2. Parts survey (Torob, 2026-10-12, VPN off; raw results in `2026-10-12-parts-survey-phase3-torob.json`)

**In stock and chosen:**

| Part | Role | Shops |
|---|---|---|
| Si5351C-B-GM1 | clock | 2 |
| 74LVC74 (SO-14) | ÷4 Johnson counter | 1 |
| 74LVC157 (SO-16) | 2:1 LO path mux (quad) | 2 |
| 25 MHz crystal (SMD) | Si5351C reference | 6+ |
| ESP32-S3-WROOM-1-N16R8 | controller | 4 |
| USB Type-C receptacle, 16-pin SMD | USB | 5 |
| USBLC6-2SC6 | USB ESD | 3 |
| AO3401 | power mux P-MOSFETs | 3 |
| SMBJ5.0A | input TVS | 4 |
| DC-005 5.5/2.1 jack | external 5 V | 18 |
| 0.5 A 1206 fuse | input protection | 3 |
| MT3608 (SOT23-6) | boost to 7 V | 4 |
| MIC5205-5.0 / -3.3 / ADJ | low-noise LDOs (150 mA) | 5 |
| TLV74033PDBVR (replaces AMS1117-3.3, plan 3.2 review) | 3V3_D | listed under the LDL1117S33 search |
| SY6280 | bias-T switch with current limit | 1 |
| 74HC595 (SO-16) | relay shift registers | 17 |
| ULN2803 | relay drivers | 26 |
| LM75A | temperature | 9 |
| CS5361 | ADC (re-checked) | 1 |

**Changes from the parent spec, and why:**

| Parent spec | Phase 3 | Reason |
|---|---|---|
| PCF8574 / MCP23017 I/O expander + ULN2003 | 3 × 74HC595 + 3 × ULN2803 | PCF8574T 350–675 k Toman each, MCP23017 1.35 M; 74HC595 23–60 k. The ESP32-S3 has GPIOs to spare. |
| INA219 bias-T and supply monitoring | PNP current mirror (BC857B, already in the BOM) into an ESP32-S3 ADC pin; supply voltages through dividers into ADC pins | No INA219/INA226/INA180/INA199/ZXCT1009 chip on Torob (module only); MAX4080 is 2.26 M Toman |
| PTC on the bias-T | SY6280 current-limited load switch | No resettable PTC on Torob; SY6280 adds current limit and thermal shutdown |
| LT3045 / ADP7118 / LP2985 low-noise LDOs | MIC5205 (5.0 V for 5V_A from the 7 V boost, 3.3 V for 3V3_RF) | LT3045 2.6 M Toman, TPS7A47 2.8 M, ADP7118 not found, LP2985 one shop |
| Direct quadrature from ~3.2 MHz | **from 4.8 MHz** | The CLKx_PHOFF register is 7 bits in units of ¼ VCO period (AN619 p. 10, as quoted by the Etherkit Si5351 library); 90° needs an even divider ≤ 126 with the VCO ≥ 600 MHz (Linux `clk-si5351.h`: VCO 600–900 MHz), so the lowest direct quadrature LO is 600 / 126 = 4.76 MHz |

## 3. Blocks

### 3.1 Clock and LO path (plan 3.1)

- Si5351C with a 25 MHz crystal (and an optional TCXO footprint on CLKIN). It runs on 3V3_RF and is controlled over I2C (address 0x60).
- **Direct path, LO ≥ 4.8 MHz:** CLK0 and CLK1 come from PLLA with the same even integer divider N, and CLK1 is set PHOFF = N (90°).
- **÷4 path, LO < 4.8 MHz:** CLK2 = 4·LO from PLLB drives a 74LVC74 Johnson counter, giving S0/S1 in quadrature.
- **Path mux:** a 74LVC157 selects the direct or ÷4 pair, under ESP32 GPIO control. The FST3253 select inputs are TTL, so 3.3 V logic drives them directly.
- **Calibration tone:** CLK3 (PLLB) feeds the front end through a −70 dB resistive pad and is disabled when not calibrating.
- **Checks:**
  - **Frequency planner** (Python reference for the firmware): a valid plan for every LO on 0.1–54 MHz in 1 kHz steps; VCO 600–900 MHz; integer dividers on the direct path; frequency error ≤ 1 Hz.
  - **÷4 and mux timing** from the 74LVC74 and 74LVC157 datasheet propagation delays and their spreads: S0/S1 skew and duty error. These feed the mixer Monte Carlo (spec phase 2 §5 assumed ±0.5 ns / ±0.3 ns).
  - **Cal-tone level and isolation:** the level at the antenna-side node in SPICE, and the leakage with CLK3 disabled.
- The mixer check's `DIRECT_MIN_HZ` moves from 3.2 to 4.8 MHz. Band 4's test point (3.35 MHz) moves to the ÷4 path, and `make hw` regenerates the characterisation.

**Amended 2026-10-12 (plan 3.1 prototype):**
- **Clock part:** the Torob part is the **Si5351C-B-GM1**, 16-QFN 3 × 3 mm with 4 outputs (datasheet Rev. 1.3 §10.6; the 8-output 20-QFN Si5351C-B-GM is not stocked). Four outputs are exactly enough: CLK0/1 direct I/Q, CLK2 for ÷4, CLK3 cal tone. Footprint `QFN-16-1EP_3x3mm_P0.5mm_EP1.75x1.75mm` (exposed pad 1.70–1.90 mm per the datasheet).
- **Planner:** 53,901 LO points on the 1 kHz grid, 0 failures, 0 Hz error (exact rational settings; on a 1 kHz grid every PLL ratio has a denominator ≤ 25,000).
- **Skew budget** (uniform ± per stage, summed in the mixer Monte Carlo):
  - Direct path: Si5351 ±0.2 ns (assumed, not specified) + 74LVC157 ±0.5 ns.
  - ÷4 path: 74LVC74A ±0.5 ns + 74LVC157A ±0.5 ns (tsk(o) ≤ 1.0 ns each, Nexperia datasheets).
  - With the 4.8 MHz crossover (band 4's test point is now ÷4), image rejection at the 5th percentile is 39.8 dB direct and 40.0 dB ÷4 (target ≥ 30).
- **Cal-tone pad:** 100 Ω / 51 Ω / **22 kΩ** (33 kΩ gave −74.3 dB once the Si5351's 50 Ω output impedance is included). Result: −70.8 dB at 1 MHz, −68.9 dB at 54 MHz (the 22 kΩ resistor's 0.1 pF body capacitance is modelled); it loads the RF line by 0.01 dB.
- **Footprint check:** every footprint must exist in the installed KiCad 9 libraries. It caught the bias-T's 1 mH choke footprint (`…D10.0mm_P5.00mm_Fastron_07HCP` does not exist), now `L_Radial_D10.0mm_P5.00mm_Fastron_07P` with a 13 × 13 mm keep-out (user: leave generous room).
- **Final review of plan 3.1:**
  - **Skew** is now ±tsk(o) per stage (tsk(o) bounds the whole output difference): direct ±0.2 + ±1.0 ns, ÷4 ±1.0 + ±1.0 ns.
  - **Duty error** is now in the budget per stage, with the mux on both paths: direct ±0.5 (Si5351 driver rise/fall asymmetry; an even-integer MultiSynth is exactly 50 %) + ±0.25 ns; ÷4 ±0.25 + ±0.25 ns. These values are assumed, because no datasheet gives opposite-edge mismatch. The Si5351 only guarantees 45–55 % duty below 160 MHz (±3.5 ns at 14 MHz); this is a Rev 0 measurement item, and the software I/Q calibration corrects any static imbalance.
  - **Crystal:** the BOM matches only a 25 MHz 3225 crystal, not DIP crystals or oscillators, and notes CL ≤ 10 pF (Si5351 internal load 6/8/10 pF).
  - **Firmware contract:** outside calibration, CLK3 is powered down (CLK3_PDN / MultiSynth 3 off), not just output-disabled. A running MultiSynth 3 beside the pad could couple a spur at LO + offset. That coupling cannot be simulated and is checked on Rev 0.
  - **Cache:** entries are gzipped, `make hw-fresh` prunes entries it did not produce, and a failure to store never fails a simulation.
- **`make hw` result cache** (user request), keyed by deck text + ngspice version + rawfiles, in `build/hw-cache`. `make hw-fresh` re-simulates everything and refreshes the cache. Per phase: one `make hw-fresh` plus one cached `make hw` that must give identical files (replaces two full runs).

### 3.2 Power (plan 3.2)

- **Inputs:**
  - USB VBUS through a 0.5 A fuse.
  - External 5 V on the barrel jack (user decision, 2026-10-12: keep the jack, no second USB port), protected by a fuse, an SMBJ5.0A TVS, reverse-polarity protection (P-MOSFET) and a ferrite + LC filter.
- **Power mux:** AO3401 pair. The external supply has priority, and the circuit never back-feeds the USB host.
- **Rails:**
  - **5V_A:** MT3608 boost to 7 V → two-stage LC → MIC5205-5.0.
  - **3V3_RF:** MIC5205-3.3 from 5 V main.
  - **3V3_D:** AMS1117-3.3 (replaced by a TLV74033, see the amendment).
  - **5 V main:** relays and the bias-T.
- **Checks (SPICE):**
  - **Mux:** the source switch-over, and no current into VBUS when external power is present.
  - **Boost ripple:** a behavioural switching model at 1.2 MHz. The ripple and its harmonics at 5V_A after the LC filter and the LDO's PSRR must sit below the level that would show up above the receiver noise floor through the op-amp supply rejection, reported per amateur band.
  - **Rail voltages at minimum input:** USB at 4.75 V, at full load.
  - **Power budget table**, from datasheet currents.

**Amended 2026-10-13 (plan 3.2 prototype):**
- **Input switch-over, changed after simulation:**
  - The USB path uses **two AO3401 back to back** (common source), gated by the external-supply node. The single MOSFET of the first draft let USB supply the board through its body diode whenever the external supply was a little lower.
  - The external input's reverse-polarity MOSFET comes **before** the SMBJ5.0A. With the TVS first, a reversed plug drew 19 A through it until the fuse blew.
  - Results:
    - USB draws 0.03 mA when external power is present, even with USB at 5.25 V and external at 4.75 V, and nothing flows back into the host with USB at 4.4 V and external at 5.25 V.
    - A reversed plug draws 0 mA, and the board keeps running on USB.
    - Fuses are 1 A, 1206 time-lag (see Transients).
- **Rails** at minimum input and maximum load all hold, over 10 DC scenarios: 5V_A ≥ 4.994 V; 3V3_RF ≥ 3.297 V; 3V3_D ≥ 3.292 V.
- **3V3_D uses a TLV74033 (changed after review):** the AMS1117's maximum dropout of 1.3 V left 3V3_D below the CS5361 VD minimum (3.1 V) with the external supply at 4.75 V (5V_MAIN 4.30 V). The TLV74033 (SOT-23-5, 300 mA, on Torob as TLV74033PDBVR) is modelled with 0.4 V dropout. Continuous 3V3_D maximum is 0.15 A from the datasheet sum (ESP32-S3 at 240 MHz with the radio off ≈ 107 mA, CS5361 VD+VL 17 mA, oscillator 10 mA, logic and LED 8 mA); 0.25 A peaks stay within the part's rating.
- **EXT_P pull-down is 1 kΩ (changed after review):** with 100 kΩ, the SS34's reverse leakage from 5V_MAIN (mA when hot) lifted EXT_P until the USB FETs turned off (simulated with 2 mA: 5V_MAIN fell to 2.7 V). With 1 kΩ, 5V_MAIN stays at 4.54 V. The cost is 25 mW from the jack.
- **Transients:**
  - Pulling the jack with USB present hands over without a dip (3V3_D minimum 3.29 V).
  - USB plug-in: rails up in 0.84 ms; inrush peak 5.6 A, I²t 0.0035 A²s. Both fuses are time-lag 1206 (melting I²t ≥ 0.05 A²s), and the check keeps the inrush below a fifth of that.
  - A soft start on the shared USB-FET gate would also slow the hand-over (simulated: 3V3_D dips to 2.93 V), so the switch stays hard. A dedicated USB load switch with soft start (a new IC to confirm on Torob) is a Rev A decision.
- **The external input's ferrite and LC filter are dropped:** every sensitive rail has its own LC and LDO, and the conducted check has about 90 dB of margin. Adapter noise is mostly common mode, which a differential LC does not remove; a clip-on ferrite on the adapter cable is the remedy.
- **Current budget** (datasheet currents):
  - USB-only typical **426 mA**, within USB 2.0's 500 mA.
  - Maximum 618 mA, with the active antenna's 100 mA. **An active antenna needs the barrel jack or a USB-C host offering 1.5 A** (read from CC in plan 3.3); the firmware enforces this before enabling the bias-T.
- **Junction temperatures at 50 °C ambient**, each regulator at its own input over every maximum-load case (USB and external at 4.75 and 5.25 V): MIC5205-5.0 96 °C, TLV74033 114 °C, MIC5205-3.3 70 °C (limit 125 °C).
- **Conducted boost ripple:**
  - Sources: the MT3608's diode-current pulses and inductor current, driven into the simulated filter network. Coupling paths: preamp supply (−84 dB at 14 MHz), mixer bias (−185 dB) and bias-T feed to the antenna line (−39 dB).
  - Worst spur over all bands is −253 dBm against a gate of −160.5 dBm (MDS − 20 dB). Without the bias-T feed LC it would be about −180 dBm.
  - Conducted ripple is therefore not a risk with this filtering. Radiated and layout coupling are not modelled and belong to the layout phase.
  - The bias-T feed LC is kept: it costs little and isolates the antenna feed from 5V_MAIN transients (relays, boost start-up).
  - Assumption: MIC5205 PSRR 75 dB at 100 Hz (datasheet), then −20 dB/decade from 1 kHz to a 15 dB floor.
- **Pending (user away 2026-10-13):** confirm the SS34 Schottky on Torob (a very common part).

### 3.3 Controller, USB, ADC interface, relay drive, monitoring (plan 3.3)

- **ESP32-S3-WROOM-1-N16R8** on 3V3_D. Native USB goes on GPIO19/20 to the USB-C receptacle, with 5.1 kΩ CC pull-downs and a USBLC6-2SC6.
- **GPIO rules:** the octal-PSRAM pins (GPIO33–37) are never used, and strapping pins are used only where their boot level is safe.
- **CS5361 digital side:**
  - The ESP32 I2S is master: MCLK, SCLK, LRCK.
  - SDOUT goes to the ESP32.
  - Mode pins are set for double-speed mode (96 kHz) and I2S format.
  - VL and VD are on 3V3_D.
- **Relay drive:** 3 × 74HC595 (chained on one SPI-like bus) → 3 × ULN2803 (internal flyback diodes) → 24 G6K-2F coils: 22 for the bank, 1 attenuator, 1 preamp.
  - At most 4 coils are energised at once: the selected filter's 2, plus the attenuator and the preamp.
  - The register outputs default off at power-up, through /OE held by a pull-up until the firmware enables them. The bank's safe state is then all filters released (mute).
- **Bias-T:** SY6280 on 5 V main, with the current limit set to about 150 mA by its ISET resistor. The current is sensed by a BC857B current mirror into an ADC pin, and the firmware cuts it off at a lower threshold.
- **Monitoring:** LM75A over I2C (address 0x48); dividers on 5 V main, VBUS and the external input into ADC pins.
- **Status LED.**
- **Checks:**
  - **Net checks:** I2C address uniqueness, GPIO validity, supply-pin rails, pin budget.
  - **Current-mirror transfer:** SPICE over 0–200 mA and 0–50 °C, accuracy reported.
  - **Relay coil current budget.**

## 4. Code structure

New blocks in `hardware/neohw/blocks/`: `clock.py`, `power.py`, `controller.py`, `relay_drive.py`, `monitor.py`, `board.py` (connects every block). New checks follow the phase 2 pattern. The frequency planner lives in `hardware/neohw/lo_plan.py` (pure Python, unit-tested; the firmware's C version will be tested against its output tables).

## 5. Layout phase deliverables (decided 2026-10-12, recorded here for the next phase)

- **Standards:**
  - 4-layer stackup (user-confirmed): signal / GND / power / signal.
  - 50 Ω grounded coplanar lines, sized to the fab's stackup (Aryamadar first, Maher PCB as fallback).
  - IPC-2221 / IPC-2152 / IPC-7351 rules.
  - Signal flow in one direction, with digital, USB and the boost converter in the far corner.
  - A via fence around each filter, inductors at right angles to their neighbours, optional shield-can footprints, short matched S0/S1 lines.
  - Board about external-hard-drive size, at most roughly 140 × 100 mm.
  - The 1 mH drum choke gets a 13 × 13 mm keep-out.
- **Educational silkscreen:**
  - A labelled outline per block, e.g. "FILTER 7 · 12–16 MHz (20 m)".
  - Signal-flow arrows, labelled test points, the band table and a legend.
- **Layout as code** through KiCad 9's `pcbnew` API (system Python).
- **Verification:**
  - DRC.
  - Netlist comparison against SKiDL.
  - openEMS on the filters and lines.
- **3D:**
  - Ray-traced renders (`kicad-cli pcb render`) in the hardware report.
  - GLB export (`kicad-cli pcb export glb`; needs the `kicad-packages3d` models), plus simple dimensioned models for the G6K-2F, the GDT and the axial inductors.
- **Project site on GitHub Pages:**
  - `make site` builds a static site: an interactive three.js board viewer (rotate, zoom, flip, silkscreen toggle, part info on hover), the renders, the hardware and simulation reports, and block explanations.
  - A GitHub Actions workflow publishes the committed site on every push to `main`.
  - The README links to it.
  - Enabling Pages is confirmed with the user first.

## 6. Out of scope

PCB layout and EM checks (KiCad needs `sudo apt install kicad` by the user), firmware, host software, the virtual USB board (L5), transmit.
