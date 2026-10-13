# Elf Phase 4: Firmware and the Virtual Board over USB/IP (Design Spec)

- **Date:** 2026-10-13
- **Status:** Draft, written for autonomous execution (user instruction 2026-10-13: "go until full circuit with tiny usb simulation")
- **Parent spec:** `2026-10-12-neoradio-hf-receiver-design.md` (§6 firmware, §9 L5, §12 phases 5 and 7)
- **Builds on:** phase 2 (twin with the hardware model) and phase 3 (board circuits: `controller.json`, `bpf_bank.json`, `clock.json`, `lo_plan.py`).
- **Order:** the parent spec puts layout (phase 4 there) before firmware. The user asked to reach the whole-board USB simulation first; layout follows this phase.

## 1. Goal and success criteria

**Phase 4 succeeds when:**

1. The firmware's hardware-independent modules build on Linux with CMake and pass their unit tests (`make fw-test`).
2. The firmware's tables agree with the hardware design by construction or by test: the C LO planner equals `lo_plan.py` on the whole 1 kHz grid; the filter table, relay bit order and bias-sense table are generated from the committed characterisation files.
3. The real firmware, with its TinyUSB stack, runs as a Linux process (`elf-virtual`) whose TinyUSB device controller driver is a USB/IP server. A user-space USB/IP client test (no root) enumerates it, reads its descriptors, talks CAT over CDC and receives isochronous UAC2 audio.
4. **Acceptance (user, 2026-10-12):** attached to this PC through `vhci_hcd`, the kernel enumerates the virtual board as a standard sound card and serial port. A Python application tunes it over CDC to CW signals on several bands, reads its I/Q from the sound card, and decodes the messages. The I/Q comes from the twin's hardware model, driven by what the firmware actually wrote to the virtual Si5351 and relays.

## 2. Firmware structure (`firmware/`)

C11, one `.c/.h` pair per module; no ESP-IDF dependency outside `target/`.

| Path | Content |
|---|---|
| `core/lo_plan.{c,h}` | C port of `lo_plan.py`: PLL and MultiSynth integers, R divider, direct or ÷4 path |
| `core/si5351.{c,h}` | Register image (AN619 encoding) from a plan; write sequence through `hal_i2c_write`; PLL reset; CLK3 cal tone enable |
| `core/radio.{c,h}` | Tune: plan, path GPIO, filter index, preamp and attenuator; relay word |
| `core/relays.{c,h}` | Relay word ↔ shift-register bits in the order of `controller.json`, bit-banged on SER/SRCLK/RCLK, /OE released only after an all-released word is latched |
| `core/cat.{c,h}` | Kenwood TS-2000 subset (`FA`, `IF`, `ID`, `MD`, `PA`, `RA`, `AI`, `PS`) and extensions (`ZB` bias-T, `ZT` telemetry, `ZC` calibration state) |
| `core/supervisor.{c,h}` | Source detection from the dividers, CC current advertisement, bias-T current from the sense table, cut-off at 150 mA, under-voltage warning, temperature |
| `core/audio.{c,h}` | I2S frames (24 bits in 32-bit slots, L = I, R = Q) → ring buffer → UAC2 packets (96 frames per 1 ms, 576 bytes); overrun and underrun counters |
| `core/hal.h` | The only hardware interface: I2C, GPIO, ADC read, I2S read, time |
| `tools/gen_tables.py` | Generates `tables_gen.{c,h}` in the build from `hardware/characterized/` (filter edges, relay order, GPIO pads, bias-sense table, I2C addresses) |
| `usb/` | TinyUSB configuration, descriptors and class callbacks: UAC2 stereo 24-bit 96 kHz input + CDC-ACM, full speed (the ESP32-S3's USB is full speed) |
| `host/` | Linux build: `vhw.c` (the parts modelled at pins and registers: Si5351C register file, 74HC595 chain, LM75A, ADC), `hal_virtual.c`, `dcd_usbip.c`, `twin_link.c`, `main.c` → `elf-virtual` |
| `target/esp32s3/` | ESP-IDF glue; later plan (ESP-IDF is not installed here) |
| `../third_party/tinyusb` | Git submodule, pinned to release 0.21.0 |

## 3. The virtual board (`elf-virtual`)

- **TinyUSB DCD over USB/IP** (`dcd_usbip.c`): a TCP server on 127.0.0.1:3240 implementing the USB/IP protocol (`OP_REQ_DEVLIST`, `OP_REQ_IMPORT`, `USBIP_CMD_SUBMIT`/`RET_SUBMIT`, `USBIP_CMD_UNLINK`/`RET_UNLINK`, isochronous packet descriptors). SETUP packets become `dcd_event_setup_received`, data stages and endpoint transfers map to `dcd_edpt_xfer` and `dcd_event_xfer_complete`. A 1 ms frame clock paces isochronous IN at the real sample rate.
- **Virtual hardware** (`hal_virtual.c`): an Si5351C register file behind the fake I2C bus, the shift-register chain, GPIO levels, ADC readings (rails, CC, bias sense) set from a state file, and I2S frames from the twin.
- **Twin link:** a Unix socket to `python -m neosim vboard <stations.yaml>`. The virtual board sends the raw Si5351 register image, the relay word and the path GPIO; the Python side decodes them **independently** (AN619 arithmetic written separately from the C encoder) into LO frequency, filter, preamp and attenuator, renders the stations through `hardware.receive`, and streams 24-bit I/Q. A wrong register, relay bit or path in the firmware therefore shows up as a wrong or silent band, not as a passing test.

## 4. Host side

- **User-space USB/IP client** (`sim/neosim/usbip_client.py`, test only, no root): enumeration, descriptors, SET_CONFIGURATION, CDC line coding, CAT round trips on bulk endpoints, UAC2 alternate setting and isochronous reads.
- **Kernel attach** (needs root once per run): `sudo modprobe vhci-hcd` and `sudo usbip attach -r 127.0.0.1 -b 1-1`, wrapped in `tools/vboard-attach.sh`. The user runs it; nothing in `make test` needs root.
- **CW application** (`tools/elf_cw.py`, Python standard library + numpy): finds the Elf ALSA card and `/dev/ttyACM*`, and for each band sends `FA`, waits for the relays, records I/Q with `arecord` (S24_3LE, 96 kHz, stereo), runs the shipping C# DSP CLI in CW mode and decodes with the twin's CW decoder. Reports text and CER per band.
- **Bands for acceptance:** 80 m (3.550 MHz, the ÷4 path), 40 m (7.030), 20 m (14.050), 15 m (21.050) and 6 m (50.090 MHz): both LO paths and five filters. The application tunes the LO 10 kHz below each station (clear of DC and the I/Q image) and the DSP demodulates at the station − 600 Hz.

## 5. Plans

1. **4.1 Firmware core:** `lo_plan`, `si5351`, `relays`, `radio`, `cat`, `supervisor`, `audio`, generated tables, the virtual hardware models, ctest unit tests, Python cross-checks (planner grid, Si5351 register images decoded by the independent decoder).
2. **4.2 The virtual board:** TinyUSB submodule, descriptors and class callbacks, `dcd_usbip.c`, `elf-virtual`, the twin link and `neosim vboard`, the user-space USB/IP client and its tests, the five-band acceptance through it, and the kernel-path tools (`elf_cw.py`, attach scripts). The kernel run itself needs root and is done with the user.

## 6. Prototype results (2026-10-13)

- The C planner equals `lo_plan.py` on all 53 901 LOs of the 1 kHz grid. Every register image decodes, through the independent AN619 decoder, to exactly the requested LO and calibration tone.
- Unit tests: 6 suites. Mutation checks: reversed relay bit order, inverted path select, a wrong audio byte, a doubled cut-off, no relay settling and a missing MS_INT bit are each caught.
- USB over USB/IP (user-space client): enumeration, descriptors, CAT round trip, UAC2 sample-rate controls, unsupported-request stall, URB unlink, recovery after a malformed URB. The isochronous stream is sample-continuous (phase residual < 10⁻⁵ rad) at one packet per 1 ms frame. The real `usbip list -r` tool lists the device and its four interfaces.
- Acceptance through the user-space client: CW decodes with CER 0 on all five bands, 80 m on the ÷4 path. A wrong firmware state (no single filter, a filter that does not cover the LO, a wrong path select) blocks the stations in the twin.
- Found by the prototype: the USB/IP `ep` field is the endpoint number only. A value above 15 now drops the client instead of indexing past the endpoint table.

## 7. Out of scope

ESP-IDF target build and flashing (needs ESP-IDF; later plan), Wi-Fi/BT, the C# receiver daemon and web UI (Pi software phase), real-time guarantees beyond the 1 ms frame pacing.
