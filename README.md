# Elf

Elf is a general-coverage HF software-defined receiver that connects to a Linux computer over
a single USB-C cable. It covers 0.1 to 54 MHz with a quadrature sampling (Tayloe) detector, a
bank of sub-octave band-pass filters and a 24-bit audio-rate ADC. The board enumerates as a
standard USB audio and serial device, so it works with any Linux host, from a Raspberry Pi 4B
to a desktop PC, without a custom driver. The name comes from the elves' famously large ears.

The design is developed simulation-first: every circuit is written as code, verified in
SPICE against explicit targets, and fed into a digital twin that runs the shipping receiver
software and real decoders on synthetic band conditions before any hardware is built. All
parts are selected for availability in the Iranian electronics market.

## Features

### Receiver

- Continuous coverage from 0.1 to 54 MHz (LF, MW, all amateur bands from 160 m to 6 m,
  shortwave broadcast bands)
- 96 kHz of instantaneous bandwidth, 24-bit I/Q
- 11 relay-switched sub-octave band-pass filters (Chebyshev, 5th-order high-pass and 7th or
  9th-order low-pass), each isolated by an input and an output relay
- Switchable low-noise preamplifier (BFR92A, active bias) with relay bypass
- Switchable 20 dB attenuator
- 5 V bias-T for active antennas, gas discharge tube and diode limiter at the input
- Four-phase Tayloe detector (FST3253) clocked by an Si5351C: direct quadrature from 4.8 MHz
  (the limit of its phase register) and a 74LVC74 divide-by-four path below, selected by a
  74LVC157
- Built-in calibration tone (Si5351 output through a -70 dB pad) for software self-calibration
- Low-noise instrumentation baseband (LM4562) into a CS5361 audio ADC
- Muted safe state with all relays released

### Connectivity and software

- ESP32-S3 controller presenting a USB composite device: UAC2 stereo I/Q audio and a CDC
  serial port with a Kenwood TS-2000 compatible CAT subset
- No custom host driver: the board enumerates as a standard sound card and serial port on any
  Linux host (kernel `snd-usb-audio` and `cdc_acm`)
- Host software runs on desktop Linux (x86-64) and on the Raspberry Pi (arm64) alike; nothing
  depends on Pi-specific hardware
- Receiver daemon (C#, .NET 10) with spectrum and waterfall web interface, SSB, CW, AM and FM
  demodulation, raw I/Q recording and a Hamlib-compatible rigctl server
- Works with SDR++ (audio source plus rigctl) and WSJT-X (virtual audio sink plus rigctl)

### Design and verification toolchain

- Circuits written in Python with SKiDL; one source produces the SPICE netlist, the KiCad 9
  netlist, the bill of materials with Iranian market availability, and a connection diagram
- Block checks in ngspice: S-parameters, noise figure, two-tone intermodulation, stability,
  Monte Carlo over component tolerances, supply, gain and temperature corners
- Characterisation files (Touchstone and JSON) that form the contract between the circuit
  simulations and the digital twin
- Digital twin in Python: scenarios with imaginary stations and noise, rendered to I/Q, run
  through the shipping C# DSP and decoded by real FT8 and CW decoders
- Single-command reports: `make hw` for the circuits, `make sim` for the twin
- Simulation results cached by netlist content: an unchanged block is not re-simulated
  (`make hw-fresh` re-runs everything)
- Si5351 frequency planner (the reference for the firmware) with an exact plan for every LO
  from 0.1 to 54 MHz

## Simulated performance

Results from the current circuit simulations (nominal values unless stated).

| Block | Result |
|---|---|
| Band-pass filters | In-band loss 1.3 to 1.9 dB; return loss 16 to 20 dB; third-harmonic rejection 55 to 74 dB; 196 or more of 200 Monte Carlo builds within all targets for every filter |
| Preamplifier | Gain 14.1 to 14.9 dB over 1 to 54 MHz; noise figure 4.3 dB (4.9 dB worst corner); OIP3 +23.9 dBm; unconditionally stable to 6 GHz; collector current below 22.5 mA in all corners |
| Attenuator | 19.96 dB; return loss 43 dB; bypass loss 0.01 dB |
| Limiter | IMD3 -115 dBm at two -10 dBm tones; +27 dBm input clamped to 1.9 V peak |
| Bias-T | Shunt impedance above 660 ohm from 0.1 to 54 MHz; 0.37 V drop at 100 mA |
| Mixer and baseband | Conversion gain 36 dB; flatness within 0.45 dB over +/-40 kHz; IIP3 +32 dBm; raw image rejection 40 dB (5th percentile, before software correction); system 2nd and 3rd harmonic rejection above 62 dB on bands 3 to 11 |

## Repository layout

```
hardware/   Circuit sources (neohw), SPICE models, block checks, characterisation files
sim/        Digital twin (neosim): scenarios, channel and ADC models, decoders, reports
pi/         Receiver DSP library and command-line tool (C#, .NET 10)
docs/       Design specifications and implementation plans
third_party/ft8_lib  FT8 encoder and decoder used by the twin (git submodule)
```

## Building and running

Requirements: Python 3.11 or later, .NET 10 SDK, ngspice 45 or later, Graphviz, and the Python
packages `skidl==2.3.0`, `numpy`, `scipy`, `matplotlib`, `pyyaml`.

```
git clone --recurse-submodules https://github.com/Neo-vortex/Elf.git
cd Elf
make tools     # build ft8_lib and the C# DSP
make test      # unit and integration tests (C#, twin, hardware)
make hw        # circuit checks and characterisation; report in build/hw/report.html
make sim       # digital twin scenarios and FT8 sweep; report in build/sim/report.html
```

`make hw` runs several hundred SPICE simulations in parallel and takes about 15 minutes on a
20-core machine.

## Roadmap

### Simulation and DSP

- [x] Digital twin with an ideal receiver, FT8 and CW decoding, and SNR sweeps
- [x] Receiver DSP library and command-line tool
- [x] Circuit-as-code toolchain: SPICE, KiCad netlist, BOM with availability, diagrams
- [x] Mixer noise figure and receiver noise budget
- [x] Digital twin driven by the simulated hardware: filters, attenuator, preamplifier,
      mixer response, image, harmonic responses and ADC clipping
- [x] Intermodulation and reciprocal-mixing models; blocker and MDS scenarios
- [x] I/Q imbalance correction in the DSP

### Circuits

- [x] Antenna input and 5 V bias-T
- [x] 20 dB attenuator
- [x] Input limiter
- [x] 11-filter band-pass bank
- [x] Preamplifier with bypass
- [x] Tayloe mixer, baseband amplifier and ADC input
- [x] Clock generation: Si5351C, divide-by-four, path selection
- [ ] ADC digital interface (CS5361, I2S)
- [ ] Controller and USB: ESP32-S3, USB-C, ESD protection
- [ ] Power: source selection, boost converter, low-noise regulators
- [ ] Relay drivers, bias-T switching, current and temperature monitoring
- [ ] Calibration tone path
- [ ] Availability survey for the remaining parts

### Board

- [ ] Rev 0 test board layout: front end, mixer and baseband
- [ ] Electromagnetic simulation of the layout
- [ ] Rev A full schematic
- [ ] Rev A layout and post-layout verification

### Firmware

- [ ] ESP32-S3 firmware: USB audio, CAT, radio control, supervisor, calibration
- [ ] Host build of the firmware modules with unit tests

### Host software (desktop Linux and Raspberry Pi)

- [ ] Receiver daemon: device handling, control, rigctl, audio output, recorder
- [ ] Web interface: spectrum, waterfall, tuning, diagnostics
- [ ] AM, FM, SSB and CW demodulation
- [ ] SDR++ and WSJT-X integration
- [ ] Packages for desktop Linux (x86-64) and Raspberry Pi (arm64)

### System verification

- [ ] Whole-board simulation over virtual USB: firmware with TinyUSB attached through USB/IP
- [ ] End-to-end test: a Python application reads I/Q from the simulated board and decodes
      CW on several bands
- [ ] Hardware validation on Rev 0 and Rev A
