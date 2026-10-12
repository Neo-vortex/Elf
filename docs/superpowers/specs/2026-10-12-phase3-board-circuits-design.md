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
| AMS1117-3.3 | 3V3_D | 22 (parent §10) |
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

### 3.2 Power (plan 3.2)

- **Inputs:**
  - USB VBUS through a 0.5 A fuse.
  - External 5 V on the barrel jack (user decision, 2026-10-12: keep the jack, no second USB port), protected by a fuse, an SMBJ5.0A TVS, reverse-polarity protection (P-MOSFET) and a ferrite + LC filter.
- **Power mux:** AO3401 pair. The external supply has priority, and the circuit never back-feeds the USB host.
- **Rails:**
  - **5V_A:** MT3608 boost to 7 V → two-stage LC → MIC5205-5.0.
  - **3V3_RF:** MIC5205-3.3 from 5 V main.
  - **3V3_D:** AMS1117-3.3.
  - **5 V main:** relays and the bias-T.
- **Checks (SPICE):**
  - **Mux:** the source switch-over, and no current into VBUS when external power is present.
  - **Boost ripple:** a behavioural switching model at 1.2 MHz. The ripple and its harmonics at 5V_A after the LC filter and the LDO's PSRR must sit below the level that would show up above the receiver noise floor through the op-amp supply rejection, reported per amateur band.
  - **Rail voltages at minimum input:** USB at 4.75 V, at full load.
  - **Power budget table**, from datasheet currents.

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

## 5. Out of scope

PCB layout and EM checks (KiCad needs `sudo apt install kicad` by the user), firmware, host software, the virtual USB board (L5), transmit.
