# NeoRadio — HF SDR Receiver (Rev A) — Design Spec

- **Date:** 2026-10-12
- **Status:** Draft for review
- **Scope of this spec:** receive-only HF board v1, its ESP32-S3 firmware, the Raspberry Pi software, and the simulation framework that must validate all of it before the board is ordered.

---

## 1. Goal and success criteria

Build the best HF receiver we can, from parts buyable in the Iranian market, connected to a Raspberry Pi 4B over USB, and **prove it works in simulation before spending money on hardware**.

**Success means:**

1. The digital twin (§9, layer L4), running the *same* Pi DSP code that ships, shows an FT8 **implementation loss ≤ 1 dB** against an audio-only reference that feeds the same decoder directly — including with a +60 dB stronger broadcast station 1.3 MHz away. (Measured 2026-10-12: the open-source ft8_lib decoder's own limit is ≈ −18…−19 dB in 2500 Hz, so absolute −20 dB tests require WSJT-X `jt9`, added in a later phase.)
2. All block-level targets in §5–§7 pass in simulation, including Monte Carlo and supply/temperature corners.
3. Every BOM line is available from at least one Iranian shop (single-source lines are flagged and bought early).
4. After Rev A is built, raw I/Q recordings from the real board match the twin's predictions closely enough to recalibrate models (§9, L6).

## 2. Requirements (agreed with the user)

| Topic | Decision |
|---|---|
| Function | **Receive only** (v1). Transmit is a future add-on board; an expansion header is reserved. VHF/UHF (SA818 based) is deferred to a separate future board. |
| Coverage | General coverage ~0.1–54 MHz; all amateur bands 2200 m … 6 m |
| Modes | SSB, CW, FT8 (plus AM broadcast); demodulation happens on the Pi |
| Host | Raspberry Pi 4 Model B, USB-C (board) ↔ USB-A (Pi) |
| Controller | **ESP32-S3-WROOM-1 (N16R8)**, firmware in **C (ESP-IDF)**, Wi-Fi/BT disabled |
| Visible bandwidth | **96 kHz**, 24-bit I/Q (USB Full-Speed UAC2 limit) |
| Power | USB 5 V from the Pi; optional **external 5 V** (same PSU that powers the Pi) on the board |
| Active antenna | **5 V bias-T** usable on USB power alone |
| PCB | **2 layers, routed as single layer**: all components and traces on top, bottom = unbroken ground plane |
| Assembly | By a provider; any package (incl. QFN) allowed |
| Complexity | Keep the circuit as simple as the performance targets allow |
| Sourcing | All parts from Iranian shops; checked on Torob (raw data: `2026-10-12-parts-availability-torob.json`) |
| Pi software | **C# on .NET 10 (LTS)**, linux-arm64 |
| Schematic capture | **Circuits as code with SKiDL (Python)**: one source per block generates the KiCad netlist (layout in KiCad) and the ngspice netlist (simulation) |
| Verification | Heavy simulation before building (§9) |

**User test equipment:** multimeter, bench power supply, RTL-SDR (weak on HF), no oscilloscope. A NanoVNA purchase is recommended for Rev 0.

## 3. Architecture

```
 BNC ─► 5V bias-T ─► protection ─► 0/20 dB atten ─► 10× band-pass filters ─► preamp ─► FST3253 Tayloe ─► LM4562 diff amps ─► CS5361 ─I2S─► ESP32-S3 ─USB─► Pi 4B
                                                        ▲                     (bypass/   ▲                                                   (UAC2 I/Q +
                                                        │                      5V LNA)   │                                                    CDC control)
                                        cal tone (−70 dB pad) ◄── Si5351C (8 outputs) ───┘ quadrature LO (direct ≥ ~3.2 MHz, 74LVC74 ÷4 below)
 Power: USB 5V ─┬─► P-MOSFET power mux ─► 5V main ─► boost 7V ─► LC ─► LDO 5V_A (analog)
        Ext 5V ─┘   (ext has priority,              ├─► LDO 3V3_RF (clock, mixer)
                    no back-feed into Pi)           ├─► AMS1117 3V3_D (ESP32, logic)
                                                    └─► relays (ULN2003), bias-T (PTC + MOSFET)
 Monitoring: INA219 (V/I), LM75 (temp). Relay drive: I2C I/O expander + ULN2003.
 Expansion header (future TX board): I2C, I2S, PTT, RF in/out, 5V, GND.
```

**Board floor plan principle:** signal flows in one direction across the board; ESP32-S3 and USB at the far corner from the antenna input; boost converter in its own shielded corner; clock (Si5351C) between front end and mixer with short LO traces.

## 4. RF front end (antenna → preamp)

1. **Connector:** BNC PCB-mount.
2. **Bias-T (5 V):** P-MOSFET high-side switch + PTC fuse; INA219 detects over-current (>150 mA → firmware switches off and reports). RF choke chain (large + small inductor in series) flat across 0.1–54 MHz; DC-blocking capacitors. **Off at every power-up.**
3. **Protection:** gas discharge tube at the connector, 1 MΩ static bleed, light anti-parallel diode limiter after the attenuator, biased so it never conducts at normal signal levels.
4. **Attenuator:** one 20 dB resistive π-pad, switched in/out by one DPDT relay.
5. **Band-pass filter bank** — 10 sub-octave filters, each switched by one **Omron G6K-2F (DPDT)** relay (both sides with one relay). Exactly one filter relay is energised at a time; with all relays de-energised (controller not running) no filter is connected and the receiver is muted, which is the defined safe state.

   | # | Range (MHz) | Covers |
   |---|---|---|
   | 1 | 0.1–0.5 | 2200 m, 630 m, LF |
   | 2 | 0.5–1.7 | MW broadcast |
   | 3 | 1.7–2.5 | 160 m |
   | 4 | 2.5–4.5 | 80 m |
   | 5 | 4.5–7.5 | 60 m, 40 m |
   | 6 | 7.5–12 | 30 m |
   | 7 | 12–16 | 20 m |
   | 8 | 16–22 | 17 m, 15 m |
   | 9 | 22–30 | 12 m, 10 m |
   | 10 | 30–54 | 6 m + general |

   Built from **SMD inductors and capacitors only — no toroids** (no RF toroids were found in Iranian online shops).
6. **Preamp, 2 modes:** *Bypass* (default on 160–20 m, where band noise dominates) and *5 V LNA* (BFR92A or 2N3866 resistive-feedback stage, ~15 dB gain, ~25 mA).

**Targets (simulated):**
- Rejection of the 3f response ≥ 60 dB at system level = filter ≥ 50 dB at 3× the band's lower edge + the Tayloe mixer's inherent ≈ 9.5 dB (amended 2026-10-12, see phase 2 spec).
- Passband insertion loss ≤ 2 dB with realistic inductor Q.
- System noise figure on 6 m with LNA ≤ 8 dB.
- 2nd/3rd-order products from a +60 dB blocker below the noise floor in the L4 scenarios.

## 5. LO, mixer, baseband, ADC

- **Clock:** **Si5351C** (QFN-20, 8 outputs), 25 MHz crystal, optional TCXO footprint.
  - ≥ ~3.2 MHz: CLK0/CLK1 with 90° phase offset drive the mixer directly.
  - < ~3.2 MHz: CLK2 at 4f → **74LVC74** Johnson ÷4 → quadrature.
  - A 2:1 logic mux selects the path; firmware chooses by frequency. (The exact crossover frequency is to be confirmed from the datasheet's PLL/divider limits during L2.)
  - CLK3 on PLLB: calibration tone through a −70 dB resistive pad into the front end; output disabled when not calibrating.
- **Mixer:** **FST3253**, classic 4-phase Tayloe, single-ended RF input (no transformer), 4 sampling capacitors.
- **Baseband:** **LM4562** differential amplifiers form I = C0−C2 and Q = C1−C3; gain set so a strong-but-legal in-band signal sits ~6 dB below ADC full scale. **NE5532** is a pin-compatible fallback.
- **Anti-alias:** RC/active low-pass ahead of the ADC.
- **ADC:** **CS5361** (24-bit, 114 dB DR, differential input), 96 kHz double-speed mode, I2S slave, MCLK from the ESP32-S3. **Single-source — buy 2–3 spares at project start.** Fallback CS5343 requires a board revision.

**Targets (simulated):** MDS ≈ −130 dBm in 500 Hz; image rejection ≥ 60 dB after software calibration; residual I/Q imbalance from Monte Carlo within the calibration range; reciprocal mixing from Si5351 phase noise quantified in L4.

## 6. Controller, USB, firmware

**Hardware:** ESP32-S3-WROOM-1 N16R8; native USB (GPIO19/20) to a 16-pin USB-C receptacle with 5.1 kΩ CC pull-downs and USB ESD protection; status LED.

**USB composite device (no custom host driver):**
1. **UAC2 audio input:** stereo, 24-bit, 96 kHz, L = I, R = Q.
2. **CDC-ACM serial:** Kenwood TS-2000-compatible CAT subset (works with Hamlib) + extended commands (filter, preamp, attenuator, bias-T, calibration, telemetry, self-test).

**Firmware (C, ESP-IDF), one `.c/.h` pair per module, hardware access only through `hal`:**

| Module | Responsibility |
|---|---|
| `audio` | I2S DMA from CS5361 → UAC2 endpoint; buffer level monitoring, under/overrun counters |
| `cat` | Command parser (Kenwood subset + extensions) |
| `radio` | Frequency → Si5351 settings, LO path (direct/÷4), filter and preamp selection |
| `hal` | Drivers: Si5351C, I/O expander, INA219, LM75, GPIO |
| `supervisor` | Power source detection, bias-T over-current cut-off, USB under-voltage warning (<4.75 V), temperature, watchdog |
| `calib` | Frequency and I/Q calibration data in NVS |

**Startup:** self-test (I2C devices present, PLL lock, ADC clocks running) → result reported over CAT and on the LED.
**Firmware update:** from the Pi over the same USB using `esptool`, wrapped by a Pi script.
**Host build:** `radio`, `cat`, `calib`, `supervisor` compile on Linux against a fake `hal` for unit tests and for the virtual board (§9, L5).

## 7. Power

- **Inputs:** USB VBUS; external 5 V on a 5.5/2.1 mm barrel jack (reverse-polarity protection, TVS, ferrite + LC filter).
- **Power mux:** two P-MOSFETs; external has priority and USB is disconnected when it is present; no back-feed into the Pi. Firmware reads which source is active.
- **Rails:**

  | Rail | Source | Loads |
  |---|---|---|
  | 5V_A | 5 V → small boost to 7 V → LC → low-noise LDO 5.0 V | CS5361 VA, LM4562 |
  | 3V3_RF | separate LDO | Si5351C, FST3253, 74LVC74, mux |
  | 3V3_D | AMS1117-3.3 | ESP32-S3, I/O expander, CS5361 VL/VD |
  | 5V main | power mux output | relays, bias-T, boost input |

- **Budget on USB only (Pi 4 USB total ≈ 1.2 A):** board ≈ 350 mA + active antenna 50–100 mA ≈ **450 mA**.
- **Boost converter noise** is a known risk: shielded corner, input/output LC filtering, switching frequency chosen to keep harmonics off amateur bands where possible; verified in L2 (conducted) and L3 (radiated/coupled).

## 8. Raspberry Pi software

**Language:** **C# on .NET 10 (LTS)**, self-contained linux-arm64 deployment.

**Solution layout:**

| Project | Type | Content |
|---|---|---|
| `NeoRadio.Dsp` | class library | All DSP: FFT, demodulators, filters, AGC, noise reduction, I/Q calibration, S-meter. No I/O, no ASP.NET dependency. SIMD via `System.Numerics.Tensors`; FFTW via P/Invoke only if profiling requires it |
| `NeoRadio.Dsp.Cli` | console app (`neoradio-dsp`) | Runs `NeoRadio.Dsp` on a SigMF/WAV I/Q file or stdin stream and writes demodulated audio + JSON metrics. **This is how the simulation (L4) executes the shipping DSP.** Runs on the PC (linux-x64) and the Pi |
| `NeoRadio.Daemon` | ASP.NET Core service (`neoradiod`) | The components below |
| `NeoRadio.*.Tests` | xUnit | Unit tests per project |

**Audio I/O:** PipeWire via `pw-record` / `pw-play` child processes (raw PCM over stdin/stdout), behind an `IAudioSource` / `IAudioSink` interface so a native binding can replace it later. **Serial:** `System.IO.Ports`. **Firmware update:** `esptool` invoked as a child process.

**`neoradiod`** (systemd service), components:

| Component | Responsibility |
|---|---|
| `device` | Opens the ALSA card (via PipeWire) and the CDC port; udev rule gives stable names (`/dev/neoradio`, card `NeoRadio`) |
| `dsp` | FFT/waterfall, SSB/CW/AM/FM demodulation, filters, AGC, noise reduction, automatic I/Q calibration, S-meter |
| `control` | Talks to the firmware; owns the serial port |
| `rigctl` | Hamlib `rigctld`-compatible TCP server (port 4532) for WSJT-X, fldigi and SDR++ Rigctl Client |
| `audio_out` | Demodulated audio to the Pi speaker and to a virtual sink for WSJT-X |
| `web` | ASP.NET Core HTTP + WebSocket UI (static HTML/JS front end): spectrum, waterfall, click-to-tune, bands, filter width, preamp/attenuator/bias-T, S-meter, power/temperature |
| `diagnostics` | Raw view (below) |
| `recorder` | Raw I/Q recording (WAV and SigMF) and replay as a virtual source |
| `updater` | Firmware flashing via `esptool`, self-test report |

**Raw / diagnostics page:** unprocessed I/Q spectrum (no calibration, filtering or AGC), I/Q time-domain scope, I/Q constellation, ADC histogram and peak dBFS, telemetry (voltages, currents, bias-T current, temperature, PLL status, USB buffer counters).

**SDR++ compatibility (method 1):** SDR++ *Audio* source on the NeoRadio card at 96 kHz + SDR++ *Rigctl Client* pointed at `neoradiod`'s rigctl port. PipeWire shares the card between SDR++ and `neoradiod`. Module availability in the Pi build of SDR++ is verified during implementation.

**FT8:** WSJT-X on the virtual audio sink + Hamlib NET rigctl. An in-app ft8_lib decoder may come later.

## 9. Simulation and verification framework

**Tools:** SKiDL (schematics as Python; emits KiCad and SPICE netlists from the same source), KiCad (layout), ngspice, Qucs-S, openEMS, scikit-rf, Python (numpy/scipy), .NET 10 SDK (for `neoradio-dsp`), ft8_lib / jt9.

**Language split:** the simulation framework (models, scenario generation, RF/mixer/ADC models, orchestration, reports) is **Python**. The receiver DSP is **C#**; the twin calls the shipping `neoradio-dsp` CLI on generated I/Q files, so the code under simulation is the code that ships.

| Layer | Content | Output |
|---|---|---|
| **L1 Models** | Vendor SPICE models (LM4562, BFR92A …); inductors with Q and SRF; FST3253 R_on/C from datasheet; behavioural Si5351 (phase noise) and CS5361 (noise, FS, digital filter) | `sim/models/` |
| **L2 Blocks** | AC, noise, short transient, Monte Carlo, corners (temp, 4.75–5.25 V) for filters, preamp, mixer+baseband, bias-T, power | pass/fail vs §4, §5, §7 targets |
| **L3 EM** | openEMS on the real KiCad layout: filters in place; coupling boost→front end, ESP32→front end, Si5351→antenna; S-params fed back into L2 | post-layout results |
| **L4 Digital twin** | YAML scenarios (stations, ITU-R P.372 band noise, blockers, impulse noise, fading) → front-end (L2/L3 data) → mixer model (3f/5f response, phase noise, DC offset, I/Q imbalance) → ADC (quantisation, clipping) → I/Q file (SigMF) → **shipping C# DSP (`neoradio-dsp`)** → real decoders | FT8 decode vs SNR, CW CER, MDS, BDR, IMD3 DR, image rejection |
| **L5 Whole-board simulation over real USB** | The real firmware C code (including its TinyUSB stack) compiled for Linux, with a virtual-hardware backend fed by the L4 twin (I2S samples from the ADC model; virtual Si5351, relays, INA219). A USB/IP device-controller backend for TinyUSB attaches it through `vhci_hcd`, so the Linux kernel enumerates a real "NeoRadio" USB device (`snd-usb-audio` + `cdc_acm`). `neoradiod`, the web UI, SDR++ and WSJT-X run against it unchanged. Chosen 2026-10-12 over a Python USB emulator because it tests the actual firmware USB code. | end-to-end system test of firmware + USB + host software without hardware |
| **L6 Hardware validation** | Rev 0 coupon and Rev A board: Si5351 cal tone as generator, NanoVNA on filters, RTL-SDR for spurs >24 MHz, real antenna vs WebSDR; raw I/Q recordings fed back into L4 | calibrated models |

**Automation:** one command (`make sim`) runs everything and writes an HTML report with plots and pass/fail. Re-run on every design change.

**Example scenario:**
```yaml
band: 20m
noise: itu_p372_rural
stations:
  - {mode: ft8, freq: 14.0755, snr_db: -20, msg: "CQ EP2ABC LL35"}
  - {mode: ft8, freq: 14.0761, snr_db: -5,  msg: "EP2ABC DL1XYZ JO62"}
  - {mode: am,  freq: 15.400,  level_dbm: -20}
  - {mode: cw,  freq: 14.072,  snr_db: 3, text: "CQ TEST", wpm: 22}
expect: {ft8_decoded: ">= 2", cw_cer: "< 5%"}
```

**Staged hardware:** Rev 0 = small coupon (filter bank + preamp + mixer + baseband) measured against simulation; Rev A = full board, ordered only once the models match Rev 0.

## 10. Parts availability (Torob, 2026-10-11/12)

✅ = in stock in at least one shop. Shop counts are approximate.

| Part | Role | Status |
|---|---|---|
| Si5351C-B-GM1 | clock | ✅ 2 shops |
| FST3253 | mixer | ✅ 2+ |
| 74LVC74 | ÷4 divider | ✅ 1 (single source) |
| CS5361 | ADC | ✅ 1 (single source, ~1.25 M Toman) |
| LM4562 / NE5532 | baseband op-amp | ✅ 2 / 20+ |
| BFR92A / 2N3866 | LNA | ✅ 1 / 4 |
| G6K-2F | RF relay | ✅ 11 |
| ESP32-S3-WROOM-1 N16R8 | controller | ✅ 17 |
| INA219, LM75 | monitoring | ✅ |
| AMS1117-3.3 | 3V3_D | ✅ 22 |
| BAT54 | limiter | ✅ 26 |
| BNC PCB | connector | ✅ 15 |
| LT3045 | low-noise LDO candidate | ✅ 1 |

**To verify before schematic freeze (one batch):** AO3401, gas discharge tube, PTC fuse, BAV99, 16-pin USB-C receptacle, USB ESD array, 25 MHz crystal / TCXO, small boost IC (e.g. MT3608), alternative low-noise LDOs (LP2985, ADP7118, TPS7A47), I/O expander (PCF8574/MCP23017), ULN2003, 2:1 logic mux, 5.5/2.1 barrel jack, SMD inductor series with known Q in the needed values.

**Not available (drove design choices):** Amidon/Fair-Rite toroids, AD8307, RD16HHF1 (TX only — out of scope now), fast ADCs/FPGAs (rules out direct sampling).

## 11. Repository layout (planned)

```
hardware/      SKiDL circuit sources (blocks/ as Python modules) + generated netlists; KiCad layout projects rev0-coupon/, revA/
sim/           models/, blocks/ (L2), em/ (L3), twin/ (L4), scenarios/, report/
firmware/      ESP-IDF project; components/{audio,cat,radio,hal,supervisor,calib}; host/ (Linux build + tests)
pi/            NeoRadio.sln: NeoRadio.Dsp, NeoRadio.Dsp.Cli, NeoRadio.Daemon (device, control, rigctl, audio_out, web, diagnostics, recorder, updater), tests; udev/, systemd/
docs/          specs, plans, measurement logs
```

## 12. Delivery phases

1. **Simulation environment + L1 models + L4 twin skeleton** (stations, noise, decoders running end-to-end with ideal blocks).
2. **Front end + mixer + baseband design** with L2 targets met → **Rev 0 coupon** layout, L3, order, measure.
3. **Full Rev A schematic** (clock, ADC, controller, power), L2 for power and bias-T.
4. **Rev A layout** + L3 EM + post-layout L2/L4.
5. **Firmware** (host-tested modules, then on target).
6. **Pi software** (`neoradiod`, web UI, raw view, SDR++/WSJT-X integration).
7. **L5 whole-board simulation over real USB** (firmware + TinyUSB over USB/IP + host software) sign-off → order Rev A.
8. **L6 validation** on the real board; models recalibrated.

Each phase gets its own implementation plan.

## 13. Risks

| Risk | Mitigation |
|---|---|
| Single-source CS5361 / 74LVC74 / LT3045 | Buy spares at project start; documented fallbacks |
| Boost converter or ESP32 noise reaching the front end | Floor plan, shielding, filtering; L2 + L3 checks; raw view for measurement |
| SMD inductor Q too low for filter targets | Model real Q in L2; choose higher-Q series; widen filters if needed |
| Si5351 quadrature limits differ from assumption | Confirm crossover in L2; ÷4 path covers the low range regardless |
| Limited test equipment | Built-in cal tone, raw diagnostics, recommended NanoVNA, Rev 0 coupon |
| UAC2 full-speed timing on ESP32-S3 | Prototype `audio` early on an ESP32-S3 dev board with a CS5361 or I2S test source |

## 14. Out of scope (v1)

Transmit, PA, TX filters; VHF/UHF; 192 kHz / USB High-Speed; Wi-Fi features; SoapySDR / network servers for SDR++; in-app FT8 decoder (WSJT-X used instead).
