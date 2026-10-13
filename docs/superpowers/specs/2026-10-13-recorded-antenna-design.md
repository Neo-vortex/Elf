# Elf Virtual Board: Recorded-Antenna Input (Design Spec)

- **Date:** 2026-10-13
- **Status:** Draft for the user's review (user, 2026-10-13: "spec it", after the spike).
- **Extends:** phase 4 spec `2026-10-13-phase4-firmware-virtual-board-design.md` (§3 the twin link).
- **Spike (throwaway, 2026-10-13):** 60 s of real I/Q from a public KiwiSDR in Taoyuan, Taiwan, at 11.905 MHz (a broadcast station at ≈ −61 dBm) went through the virtual board: the firmware tuned 11.895 MHz, the twin decoded the registers and relays, the carrier appeared at +10 kHz in the board's I/Q, and the shipping DSP's USB demodulation kept the programme 36.4 dB above the out-of-band level (40.0 dB for the recording demodulated without the board).

## 1. Goal and success criteria

The virtual board's twin can take **real recorded antenna signals** as its input, alongside or instead of imaginary stations, so the firmware, USB path and host software are exercised with real signals.

**Success criteria:**
1. A recording (I/Q at any sample rate, with its centre frequency and a level reference) placed on the antenna appears in the board's I/Q at the right offset from the LO the firmware set, at the right level through the hardware model (±1 dB for a test tone), and is absent when the firmware tunes elsewhere.
2. Recordings narrower than the board's window (KiwiSDR: 12 kHz) and wider ones (wideband SDR files) both work; parts of the window no recording covers carry the model's noise floor.
3. A recording loops without a click (cross-faded seam).
4. KiwiSDR I/Q WAV files from `kiwirecorder` and SigMF files are read directly; a small sidecar file supplies what a format lacks.
5. `make vboard REC=antenna.yaml` runs the virtual board on a recorded antenna; a test proves criteria 1–3 with synthetic recordings of known content.

## 2. Decisions (defaults chosen; open to the user's review)

1. **Formats:** SigMF (`cf32_le`, `ci16_le`; centre frequency and rate from its metadata) and stereo WAV I/Q (16-bit or float; I left, Q right), which is what `kiwirecorder -m iq` writes. A WAV needs a sidecar for the centre frequency.
2. **Level reference:** either `full_scale_dbm` (the antenna power of a full-scale sine in the file) or `reference: {hz, dbm}` (a carrier in the file and its measured antenna level, e.g. the KiwiSDR S-meter reading taken with the recording). Without one the twin refuses the recording: levels decide whether the board overloads, so they are never guessed.
3. **Antenna file (YAML):** lists one or more recordings and optional imaginary stations; each recording has `file`, `center_hz`, `sample_rate` (WAV only), the level reference and an optional `gain_db`. Noise for uncovered parts of the window comes from `noise.dbm_per_hz` (default thermal, −174 dBm/Hz).
4. **Signal path:** each recording is resampled to 96 kHz, shifted by (its centre − the firmware's LO), and added as an antenna signal before the hardware model (front-end gain per frequency, conversion gain, baseband, noise, ADC), exactly where imaginary stations enter. Parts of a recording beyond the board's ±43 kHz usable window are cut by the model as for any station; LO harmonic responses (3·LO, 5·LO) apply only when a recording covers those frequencies.
5. **Looping:** a recording longer than the render length is cut; a shorter one loops with a 20 ms raised-cosine cross-fade at the seam.
6. **Fetching from KiwiSDR:** `tools/kiwi_record.py` wraps `kiwirecorder` (from the user's own `kiwiclient` checkout, path in `KIWICLIENT`; not vendored): it records I/Q and the S-meter together and writes the WAV plus a ready sidecar (`center_hz`, `sample_rate`, `reference` = the carrier and its median S-meter reading). It honours the public receivers' rules: one connection, a user name, `--tlimit`, no automated receiver-list scraping (the user names the receiver).
7. **Listening:** AM demodulation belongs to the host-software phase; until then the twin's tests and the user's checks use the shipping DSP's USB mode zero-beat on a carrier (as in the spike).

## 3. Components

| File | Role |
|---|---|
| `sim/neosim/antenna.py` | Read SigMF / WAV + sidecar; level calibration; resample, shift, loop with cross-fade; render the antenna signal for a given LO and length |
| `sim/neosim/vboard.py` | The twin accepts an antenna file (recordings + stations); state decode unchanged |
| `sim/neosim/__main__.py` | `vboard` accepts either a stations file or an antenna file (by its `recordings:` key) |
| `tools/kiwi_record.py` | KiwiSDR recording with its sidecar |
| `tools/vboard-run.sh`, `Makefile` | `make vboard REC=antenna.yaml` |

## 4. Tests

- **Synthetic SigMF:** two tones at known antenna levels and frequencies inside a 200 kHz-wide file; the firmware tunes over USB/IP; each tone appears at its offset from the LO and at its level through the model's gain (±1 dB); tuning 200 kHz away removes them.
- **Narrow WAV + sidecar (KiwiSDR-shaped, 12 kHz, 16-bit):** a tone at the centre appears at the expected offset; the uncovered window shows the configured noise floor.
- **Level reference:** `full_scale_dbm` and `reference` give the same level for the same file; a recording with neither is refused with a clear message.
- **Loop seam:** a 1 s file looped over 3 s has no step at the seams (sample-to-sample jump below the signal's own maximum).
- **Spike recording (not in the repository: a broadcast programme is not ours to commit):** a manual check documented in the README of the tool.

## 5. Out of scope

AM/FM demodulation in the shipping DSP (host-software phase), live streaming from a KiwiSDR into a running virtual board (recordings only), committing third-party recordings to the repository.
