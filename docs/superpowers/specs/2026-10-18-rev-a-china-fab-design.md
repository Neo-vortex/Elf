# Elf Rev A for a Chinese board maker (Design Spec)

- **Date:** 2026-10-18
- **Status:** Approved in conversation (sections 1–4, 2026-10-18); written spec awaiting user review
- **Parent specs:** `2026-10-12-neoradio-hf-receiver-design.md` (§2 requirements, §12 phases), `2026-10-12-phase3-board-circuits-design.md` (§5 layout deliverables), `2026-10-13-phase4-firmware-virtual-board-design.md` (firmware layout)
- **Supersedes:** the sourcing rule "all parts from Iranian shops", the ESP32-S3 controller, the Aryamadar/Maher PCB fab target, and the Rev 0 coupon stage. Everything else in the parent specs stands.

## 0. Read this first (decisions of 2026-10-18)

| Topic | Before | Now |
|---|---|---|
| Fab and assembly | Iranian fab (Aryamadar, Maher PCB), parts from Torob | **A Chinese board maker** (JLCPCB or PCBWay, not chosen yet) builds and assembles the board from its own parts library |
| Sourcing rule | Every BOM line on Torob | **Dual population:** every line has a China primary (MPN + LCSC number) and an Iranian fallback (Torob key). One PCB accepts either part on every line, and both populations must pass every simulation target |
| Scope of circuit changes | — | **Fix the Torob workarounds only.** The architecture, GPIO-level functions, relay chain, I2C map, LO plan and 96 kHz bandwidth stay |
| Controller | ESP32-S3-WROOM-1 N16R8 | **STM32F411CEU6** (no radio, better ADC, ROM USB DFU, buyable in China and Iran) |
| Firmware update | ESP32 USB download mode | **ROM USB DFU**, entered by a CAT command or by a **BOOT/RUN slide switch** (unbrickable); plus a **RESET** button |
| PCB | 4 layers (already decided 2026-10-12) | **4 layers, 1.6 mm, ENIG**, rules both fabs accept. 6 layers only if the EM coupling checks (§6) fail and layout cannot fix them |
| Hardware staging | Rev 0 coupon, then Rev A | **Straight to Rev A**, with test points and 0 Ω links so each block can be measured alone |
| Final deliverable | — | **Fab package:** Gerber X2, Excellon drill, IPC-D-356, CPL, both BOMs, fab notes, STEP, renders (`make fab`) |

## 1. Goal and success criteria

Rev A succeeds (before ordering) when:

1. Every block changed here passes its existing `make hw` targets, **with both populations** (China and Iran).
2. The board check passes with the STM32F411 controller and its pin rules.
3. `make fw-test` and the virtual-board CW test still pass, and the STM32F411 target builds.
4. The layout passes KiCad DRC with zero errors against the fab rule set, the PCB nets equal the SKiDL nets, and the openEMS line and coupling checks meet their targets.
5. `make fab` produces the package of §7 and its own checks pass.
6. Every BOM-China line is in stock at LCSC, and every BOM-Iran line has a Torob result or is marked as China-only.

## 2. Part changes (approved section 1)

| Block | Now (Torob workaround) | China primary | Iranian fallback | Same footprint? |
|---|---|---|---|---|
| Relays, CS5361, LM2576HV, Si5351C, FST3253, LM4562, logic, connectors | — | same part, MPN pinned + LCSC number | same part | **Yes** |
| Relays (detail) | G6K-2F 5 V | **G6K-2F-Y-TR DC5** (LCSC C47190). Omron marks it "not recommended for new designs"; **G6KU-2F-Y** is recorded as the successor | G6K-2F 5 V | Yes. Coil margin is already +0.42 V at 50 °C on the 5.5 V main rail (`controller.json`, `relay_coil_margin_v`), so no driver change |
| Bias-T current sense | BC857B current mirror (0.63 V zero, 1.3 % drift, table in firmware) | **INA180A2** (gain 50, SOT-23-5, LCSC C192764) across the existing 1 Ω shunt | the BC857B mirror (today's circuit) | **No:** both circuits on the board, one fitted. The firmware detects which: with the feed off the mirror reads ≈ 0.63 V, the INA180 ≈ 0 V |
| 5V_A, 3V3_RF LDOs | MIC5205 | **ADP7118** (11 µV rms, 200 mA, TSOT-5; e.g. ADP7118WAUJZ-5.0, LCSC C3745262); LT3045 only if stocked and the noise budget needs it | MIC5205 | **Pins 1, 2, 3, 5 match** (IN, GND, EN, OUT). Pin 4 is SENSE on the ADP7118 (to OUT) and BYP on the MIC5205 (470 pF to GND): two small pads on pin 4, one fitted. **Confirm the ADP7118 TSOT-5 pinout from the ADI datasheet before freeze** |
| Filter capacitors | values the designer chose; 18 E24 values missing on Torob | **exact E24 C0G, 1 %** (Murata GCM/GRM, Samsung CL), 0805 | nearest stocked value | Yes, plus a **spare 0805 pad pair beside each filter capacitor** (not fitted) so a missing value is built from two common values in parallel. The tooling computes the pair |
| Filter inductors | axial moulded parts, Q = 30 assumed, LCR-selected pairs (`Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm…`) | **SMD wirewound chips with datasheet Q(f)** (Murata LQW/LQH, Sunlord SDWL or similar, 1008/1210, 2 % where offered) | today's axial parts | **No:** a **combined footprint** per inductor position, chip pads inside the axial pad span, one fitted. Q is modelled per part over frequency, not one constant |
| Everything else | — | unchanged, MPN pinned + LCSC number | unchanged | Yes |

Any filter that misses its targets with the real chip values is re-synthesised on obtainable values (phase 2 synthesis), then both populations are re-run.

## 3. Controller: STM32F411CEU6 (approved section 2)

- **Removed:** ESP32-S3-WROOM-1 module and its antenna keep-out.
- **Added in `controller.py`:** STM32F411CEU6 (UFQFPN-48), its crystal, decoupling and VCAP capacitor, BOOT0 network, PB2 (BOOT1) pull-down, SWD header, RESET button on NRST. USB-C connector, USBLC6-2SC6 and 3V3_D stay.
- **BOOT/RUN slide switch (user request 2026-10-18):** BOOT0 to 3V3_D through a resistor when in BOOT, pull-down otherwise. Silkscreen `BOOT ▲ / RUN ▼`. In BOOT the chip starts in the ROM DFU loader at every reset or power-up, whatever is in flash.
- **Resources:**

| Function | STM32 resource |
|---|---|
| USB | OTG FS, PA11/PA12 (the ROM DFU port) |
| I2S from the CS5361 | SPI2/I2S2 slave receive (CS5361 stays clock master on its 24.576 MHz oscillator) |
| I2C (Si5351C 0x60, LM75A 0x48) | I2C1, pull-ups on 3V3_RF as now |
| Analog sense (bias current, rails, CC1/CC2) | ADC1 inputs (10 available, 6 needed) |
| Shift chain, LO path select, ADC RST/OVFL/M-S/I2S-LJ, bias EN, LED | GPIO |

- **To settle from datasheets in plan 5.2 (not from memory):**
  - The HSE crystal frequency: it must be one the ROM bootloader detects (ST AN2606, STM32F411 section). Run a spur check of its harmonics against 0.1–54 MHz like every other board clock.
  - USB FIFO budget (1.25 KB on the F411): UAC2 96 kHz stereo 24-bit packed (576 B/ms) plus CDC must fit; the descriptor uses 3-byte subslots.
  - Whether STM32F401CCU6 is a pin-compatible Iranian fallback (same package, ROM DFU).
- **Firmware update flow:** host sends a CAT extension command ("enter bootloader") → firmware jumps to system memory → `dfu-util` writes the image. Recovery: BOOT switch on, press RESET, flash, BOOT switch off, RESET.
- **Firmware:** new `firmware/target/stm32f411/` (HAL, clocks, I2S DMA into the audio ring, ADC, DFU jump). `core/`, `usb/`, host tests and the virtual board carry over unchanged. The ESP32 target in the phase 4 spec (`target/esp32s3/`) is dropped.
- **Board check:** the ESP32 rules (PSRAM, strapping pins) are replaced by STM32 rules: each function on a pin that has that alternate function, analog functions on ADC1 pins, no conflict with BOOT0, PB2, SWD or USB pins.

## 4. Stackup, rules and floor plan (approved section 3)

- **Stackup:** 4 layers, 1.6 mm, 1 oz outer copper, ENIG.

| Layer | Use |
|---|---|
| L1 | All components; all RF routing |
| L2 | Solid ground, never cut |
| L3 | Power pours (5V_MAIN, 5V_A, 3V3_RF, 3V3_D, 5V_BIAS) and slow digital |
| L4 | Slow digital and ground pour, stitched to L2 |

- **50 Ω lines:** grounded coplanar waveguide referenced to L2; width and gap computed for both JLC04161H-7628 and PCBWay's standard 4-layer stackup, checked in openEMS; ordered with controlled impedance.
- **Rules (both fabs):** 0.15 mm trace/space default, 0.127 mm minimum; vias 0.3 mm drill / 0.6 mm pad, none in pads; 0.5 mm copper to edge; board ≤ 140 × 100 mm; **all SMD parts on top** (single-sided assembly). Through-hole only for the BNC, DC jack, 1 mH choke (13 × 13 mm keep-out), axial-inductor fallback pads, SWD header and the BOOT switch if no SMD part fits.
- **Floor plan:** the phase 3 §5 rules: one signal-flow direction (BNC → protection → filter bank → preamp → mixer → baseband → ADC); clock between filter bank and mixer; STM32 and USB in the far corner; buck converter in its own corner; via fences around each filter; inductors at right angles to their neighbours; optional shield-can footprints; test points and 0 Ω links between blocks; educational silkscreen.
- **Layout as code:** a `pcbnew` script (system `/usr/bin/python3`; Anaconda's `python3` has no `pcbnew`) reads the SKiDL netlist, places blocks by rule, routes the RF path, LO lines and power pours. Slow digital nets go to Freerouting (v2.5.0 is installed; see §8) through Specctra DSN/SES.

## 5. Why not 6 layers (decided 2026-10-18)

The step that matters is 2 → 4 (a solid ground under all RF). Signals stop at 54 MHz on a 14 cm board, and filter-bank leakage is set by relay isolation and inductor coupling, which spacing, orientation, via fences and shield cans address. 6 layers (L1 sig / L2 GND / L3 sig / L4 power / L5 GND / L6 sig) is the fallback if a §6 coupling check fails and layout cannot fix it; only the stackup and layer assignment change in the layout code.

## 6. Verification (approved section 4)

| What | Check |
|---|---|
| Part changes | Re-simulated in `make hw`: INA180 sense (0–200 mA and a short, ≤ 3.1 V at the ADC); ADP7118 rails (dropout at the low corner, thermal, transients, noise); BPF Monte Carlo with per-part Q(f) |
| Dual population | Each affected block simulated with the China set and the Iran set; both pass the existing targets. Parallel-capacitor pairs for missing E24 values are computed and simulated |
| Controller | Board check with STM32 rules; crystal on the AN2606 list; USB FIFO budget |
| Firmware | `make fw-test` and the virtual-board CW test unchanged and passing; STM32F411 target builds with `arm-none-eabi-gcc` |
| Layout | DRC zero errors against the fab rule set; netlist comparison (every SKiDL net equals the PCB's); courtyards and keep-outs |
| EM (openEMS) | 50 Ω lines; coupling of LO/clock lines into the antenna input and filter bank; filter-to-filter leakage around an unselected filter; I2S, USB and shift chain into the baseband. Each path gets an isolation target from the receiver budget; a failure blocks the fab package |
| Fab package | Gerbers re-read with `gerbonara`: 4 copper layers, outline equals the board, drill count equals the PCB's holes, nothing outside the outline; every BOM line has a CPL entry and the reverse; both BOMs agree with the netlist |
| Sourcing | BOM-China against LCSC (web lookup: LCSC refuses scripted requests from this network); BOM-Iran with the existing Torob tooling |

## 7. Fab package (`make fab` → `build/fab/revA/`, one zip per fab)

- Gerber X2 for all layers (L1–L4 copper, mask, paste, silkscreen, edge cuts), Excellon drill (plated and unplated separate) and drill map
- IPC-D-356 netlist for the fab's bare-board electrical test
- Pick-and-place (CPL) in JLCPCB column format (PCBWay accepts it), with a rotation-offset table per footprint
- **BOM-China** (designator, value, footprint, MPN, LCSC number), **BOM-Iran** (Torob key, fallback parts, parallel-capacitor pairs), and a fitted / not-fitted list per population
- Fab notes PDF: stackup, impedance target, ENIG, thickness, which BOM to use
- STEP model and ray-traced renders

## 8. Tooling on this machine (2026-10-18)

| Tool | Status |
|---|---|
| KiCad 9.0.8, `kicad-cli`, 3D models | installed; `pcbnew` imports only in `/usr/bin/python3` |
| Freerouting v2.5.0 | installed per the user; its log is `~/.local/state/freerouting/logs/freerouting.log`. Executable path still to be recorded here |
| Java | installed |
| `gcc-arm-none-eabi`, `dfu-util` | in apt; need `sudo apt install` by the user |
| openEMS | no apt package on this Ubuntu release; built from source into `~/opt/openEMS` after the user installs the build dependencies with apt |
| gerbonara | pip, installed at the start of plan 5.4 |
| LCSC | blocks scripted requests from this network; stock is checked through web search |
| `sudo` | needs a password; the agent cannot run it. Give the user the exact commands |

## 9. Plans

1. **5.1 Parts and dual population:** MPN/LCSC fields in `parts.py` and `availability.py` (Torob stays as the fallback source); INA180, ADP7118, real inductors and capacitors; alternate and parallel pads; both populations in `make hw`; README sourcing statement updated.
2. **5.2 STM32F411 controller:** controller block, BOOT switch, RESET, pin map, board-check rules, firmware target and DFU jump; README and phase 4 spec updated.
3. **5.3 Layout:** openEMS install, placement, RF routing, pours, Freerouting, DRC, netlist comparison, EM checks.
4. **5.4 Fab package:** `make fab`, Gerber re-check, both BOMs, CPL, fab notes, renders.

## 10. Out of scope

Choosing between JLCPCB and PCBWay (the design suits both); 192 kHz / High-Speed USB; transmit; ordering.
