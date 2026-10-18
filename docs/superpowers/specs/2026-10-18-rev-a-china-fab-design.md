# Elf Rev A for a Chinese board maker (Design Spec)

- **Date:** 2026-10-18
- **Status:** Approved by the user (2026-10-18)
- **Parent specs:** `2026-10-12-neoradio-hf-receiver-design.md` (§2 requirements, §12 phases), `2026-10-12-phase3-board-circuits-design.md` (§5 layout deliverables), `2026-10-13-phase4-firmware-virtual-board-design.md` (firmware layout)
- **Earlier China work:** branch `china-parts` (spec `2026-10-13-china-assembly-parts-design.md`, plan `2026-10-13-china-assembly-parts.md`, LCSC survey `2026-10-13-parts-survey-lcsc.json`, `tools/jlc_survey.py`, commit ed5be14). It was written by another agent before this spec and never merged. §2a says what is taken from it and what is not; plan 5.1 starts from that branch.
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
| Buck converter | LM2576HV (plan 3.4) | **LM2576HV kept** (user, 2026-10-18): already simulated, sold in both countries. The branch's TPS54360 is not used |
| Reference clock | 25 MHz crystal | **26 MHz TCXO** (KDS 1XXD26000MAA, ±1.5 ppm, from the branch); Iranian fallback a 26 MHz crystal on shared pads, so the LO planner sees 26 MHz either way |
| Relay drivers | 74HC595 + ULN2803 | **Kept.** The branch's TPIC6C595 + AHCT125 is not used (coil margin is already +0.42 V; a 3.3 V controller would need the level shifter) |
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
| Relays (detail) | G6K-2F 5 V | **G6K-2F-Y 5 VDC** (LCSC C2982926 per the branch survey, or C47190 for the -TR reel). Omron marks it "not recommended for new designs"; **G6KU-2F-Y** is recorded as the successor | G6K-2F 5 V | Yes. Coil margin is already +0.42 V at 50 °C on the 5.5 V main rail (`controller.json`, `relay_coil_margin_v`), so no driver change |
| Bias-T current sense | BC857B current mirror (0.63 V zero, 1.3 % drift, table in firmware) | **INA180A2** (gain 50, SOT-23-5, LCSC C192764) across the existing 1 Ω shunt | the BC857B mirror (today's circuit) | **No:** both circuits on the board, one fitted. The firmware detects which: with the feed off the mirror reads ≈ 0.63 V, the INA180 ≈ 0 V |
| 5V_A, 3V3_RF LDOs | MIC5205 | **TPS7A2050 / TPS7A2033** (SOT-23-5, 300 mA, ultra-low noise; C2864504 / C2862740, from the branch survey). ADP7118 was the first candidate; TPS7A20 is cheaper and quieter | MIC5205 | Pins 1, 2, 3, 5 expected to match (IN, GND, EN, OUT); pin 4 differs (MIC5205 BYP, 470 pF to GND). Two small pads on pin 4, one fitted. **Confirm both pinouts from the datasheets before freeze** |
| 3V3_D LDO | TLV74033 | **TLV75733PDBV** (1 A, C485517, from the branch) | TLV74033 | Both SOT-23-5 TI LDOs; confirm the pinouts match |
| Si5351C reference | 25 MHz crystal | **26 MHz TCXO** KDS 1XXD26000MAA (C135223), clipped sine into CLKIN or XA as the Si5351C datasheet requires | 26 MHz crystal | **No:** shared pads (TCXO footprint with crystal pads, one fitted). Firmware: reference 26 MHz either way; input selection (CLKIN vs XTAL) is a firmware setting detected or configured at first boot (plan 5.1 decides) |
| Filter capacitors | values the designer chose; 18 E24 values missing on Torob | **exact E24 C0G, 1 %** (Murata GCM/GRM, Samsung CL), 0805 | nearest stocked value | Yes, plus a **spare 0805 pad pair beside each filter capacitor** (not fitted) so a missing value is built from two common values in parallel. The tooling computes the pair |
| Filter inductors | axial moulded parts, Q = 30 assumed, LCR-selected pairs (`Inductor_THT:L_Axial_L6.6mm_D2.7mm_P10.16mm…`) | **SMD wirewound chips with datasheet Q(f)**: the branch's choice, TDK NLV32 (1210, 5 %) for 22 values and named substitutes for 180 nH, 390 nH, 680 nH, 8.2 µH and 12 µH, with the branch's NLV32 model in `parts.py` | today's axial parts | **No:** a **combined footprint** per inductor position, chip pads inside the axial pad span, one fitted. Q is modelled per part over frequency, not one constant |
| Everything else | — | unchanged, MPN pinned + LCSC number | unchanged | Yes |

Any filter that misses its targets with the real chip values is re-synthesised on obtainable values (phase 2 synthesis), then both populations are re-run.

## 2a. What is taken from the `china-parts` branch

- **Taken:** the LCSC survey JSON and `tools/jlc_survey.py`; the SMD inductor model and values (NLV32 and substitutes); G6K-2F-Y; 26 MHz TCXO; TPS7A20 RF LDOs; TLV75733; the LCSC-number fields in `parts.py` and `test_parts_lcsc.py`.
- **Not taken:** TPS54360 (LM2576HV stays), TPIC6C595 + SN74AHCT125 (74HC595 + ULN2803 stay), "no Iranian fallback" (dual population instead), ESP32-S3 (STM32F411 instead).
- The branch spec gets a note pointing here when plan 5.1 merges it.

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
- **Layout as code:** a `pcbnew` script (system `/usr/bin/python3`; Anaconda's `python3` has no `pcbnew`) reads the SKiDL netlist, places blocks by rule, routes the RF path, LO lines and power pours. Slow digital nets go to Freerouting (§8) through Specctra DSN/SES, run in its GUI so the user can watch the routing live.

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

## 7a. Process animations (user request 2026-10-18)

Every stage of the layout and its verification is recorded as a GIF and **committed to the repo** in `docs/media/rev-a/`, with the README showing them. They are made by scripts (`make media`, regenerated when their inputs change), not by hand, so the next build makes the same set.

| GIF | Made from | Plan |
|---|---|---|
| `placement.gif` | The layout script saves a frame after each block is placed (`kicad-cli pcb export svg` of the board, rasterised) | 5.3 |
| `rf-routing.gif` | One frame per routed RF / LO / power step of the layout script | 5.3 |
| `freerouting.gif` | Screen recording of the Freerouting GUI window while it routes (the user watches the same window live): `xwininfo` finds the window, `ffmpeg -f x11grab` records it, then a palette-optimised GIF | 5.3 |
| `em-<study>.gif` (50 Ω line, LO coupling, filter leakage, digital into baseband) | openEMS time-domain E-field dumps on a plane, rendered frame by frame (matplotlib) | 5.3 |
| `board-3d.gif` | Turntable: `kicad-cli pcb render` at evenly spaced angles, top and bottom | 5.4 |
| `layers.gif` | The Gerber layers stacked one at a time, rendered from the fab package with `gerbonara` | 5.4 |
| `filter-bank.gif` | The 11 filters' simulated responses stepping through the bands, China and Iran populations overlaid | 5.1 |

Size: each GIF at most 8 MB (GitHub renders up to 10 MB), achieved with ffmpeg's palettegen/paletteuse and frame-rate or size reduction; a test fails if one is larger. Raw frames and recordings stay in `build/media/` (not committed).

## 8. Tooling on this machine (installed 2026-10-18)

Everything below survives reboots. Nothing project-related lives under `/tmp` (session scratchpads there are wiped; that is how the earlier Freerouting install and its layout files were lost).

| Tool | Where | How to run / notes |
|---|---|---|
| KiCad 9.0.8, `kicad-cli`, 3D models | apt (`/usr/bin`, `/usr/share/kicad`) | `pcbnew` imports only in `/usr/bin/python3`; the default `python3` (Anaconda, used by the Makefile) has no `pcbnew` |
| Freerouting 2.5.0 | `~/opt/freerouting/freerouting-2.5.0.jar` (64,599,107 bytes, sha256 `f6f51bb0…5de3c7`), launcher `~/.local/bin/freerouting` | **Run it with the GUI for the board (user, 2026-10-18: they watch the routing live):** `freerouting -de board.dsn -do board.ses --router.autorouter.max_passes=N`, never `--gui.enabled=false` for the real board (headless was only the install smoke test). Settings `~/.config/freerouting/freerouting.json`, log `~/.local/state/freerouting/logs/freerouting.log`. DSN export: `pcbnew.ExportSpecctraDSN(board, path)`; import: `pcbnew.ImportSpecctraSES(board, path)`. Smoke-tested on a KiCad demo board |
| Java | apt, OpenJDK 25 | runs Freerouting |
| `arm-none-eabi-gcc` 14.2, newlib | apt (`gcc-arm-none-eabi`, `libnewlib-arm-none-eabi`) | Cortex-M4F link verified (`-mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16`) |
| `dfu-util` 0.11 | apt | udev rule `/etc/udev/rules.d/50-stm32-dfu.rules` (0483:df11, uaccess) so flashing needs no root |
| openEMS + CSXCAD | built from source: sources `~/opt/src/openEMS-Project`, install prefix `~/opt/openEMS`, build log `~/opt/src/openEMS-build.log` | build dependencies from apt (libhdf5, VTK 9, Boost, CGAL, TinyXML, Qt5, Cython). Python bindings: see the note added when they are built |
| gerbonara | pip, installed in plan 5.4 | Gerber re-check |
| LCSC | — | refuses scripted requests from this network (Akamai "Access Denied"); the earlier agent's `tools/jlc_survey.py` on branch `china-parts` reached JLCPCB's library; otherwise use web search |
| `sudo` | — | needs a password: the agent cannot run it. Give the user exact commands to run with `!` |

## 9. Plans

1. **5.1 Parts and dual population:** start from branch `china-parts` (§2a), drop what §2a excludes; MPN/LCSC fields in `parts.py` and `availability.py` (Torob stays as the fallback source); INA180, ADP7118, real inductors and capacitors; alternate and parallel pads; both populations in `make hw`; README sourcing statement updated.
2. **5.2 STM32F411 controller:** controller block, BOOT switch, RESET, pin map, board-check rules, firmware target and DFU jump; README and phase 4 spec updated.
3. **5.3 Layout:** openEMS install, placement, RF routing, pours, Freerouting, DRC, netlist comparison, EM checks.
4. **5.4 Fab package:** `make fab`, Gerber re-check, both BOMs, CPL, fab notes, renders.

## 10. Out of scope

Choosing between JLCPCB and PCBWay (the design suits both); 192 kHz / High-Speed USB; transmit; ordering.
