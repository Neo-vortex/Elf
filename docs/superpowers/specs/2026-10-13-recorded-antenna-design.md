# Elf Virtual Board: Recorded-Antenna Input (Design Spec)

- **Date:** 2026-10-13
- **Status:** Revised after the user's review (2026-10-13: live streaming wanted; AM demodulation in the host DSP; only openly licensed recordings in the project).
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
6. **Live:** `make vboard KIWI=host:port` streams a public KiwiSDR's I/Q live into the virtual board; tuning the board (CAT `FA`) retunes the remote receiver to follow it, so the virtual board behaves like a remote receiver with the Elf hardware model in front.
7. **AM:** the shipping C# DSP (`NeoRadio.Dsp`, CLI `--mode am`) demodulates AM (envelope detection with carrier tracking, DC removal and AGC), so broadcast stations are heard properly, not by single-sideband zero-beat.
8. **Open data only in the project:** the repository references only recordings whose licence allows reuse (§6); the open wideband recordings are fetched on demand with their attribution.

## 2. Decisions (defaults chosen; open to the user's review)

1. **Formats:** SigMF (`cf32_le`, `ci16_le`; centre frequency and rate from its metadata) and stereo WAV I/Q (16-bit or float; I left, Q right), which is what `kiwirecorder -m iq` writes. A WAV needs a sidecar for the centre frequency.
2. **Level reference:** either `full_scale_dbm` (the antenna power of a full-scale sine in the file) or `reference: {hz, dbm}` (a carrier in the file and its measured antenna level, e.g. the KiwiSDR S-meter reading taken with the recording). Without one the twin refuses the recording: levels decide whether the board overloads, so they are never guessed.
3. **Antenna file (YAML):** lists one or more recordings and optional imaginary stations; each recording has `file`, `center_hz`, `sample_rate` (WAV only), the level reference and an optional `gain_db`. Noise for uncovered parts of the window comes from `noise.dbm_per_hz` (default thermal, −174 dBm/Hz).
4. **Signal path:** each recording is resampled to 96 kHz, shifted by (its centre − the firmware's LO), and added as an antenna signal before the hardware model (front-end gain per frequency, conversion gain, baseband, noise, ADC), exactly where imaginary stations enter. Parts of a recording beyond the board's ±43 kHz usable window are cut by the model as for any station; LO harmonic responses (3·LO, 5·LO) apply only when a recording covers those frequencies.
5. **Looping:** a recording longer than the render length is cut; a shorter one loops with a 20 ms raised-cosine cross-fade at the seam.
6. **KiwiSDR recording and live streaming:** both run `kiwirecorder` from the user's own `kiwiclient` checkout (path in `KIWICLIENT`), as an external program: kiwiclient has no licence file, so none of its code is copied into the project (§6).
   - `tools/kiwi_record.py` records I/Q and the S-meter together and writes the WAV plus a ready sidecar (`center_hz`, `sample_rate`, `reference` = the carrier and its median S-meter reading).
   - **Live** (`sim/neosim/kiwi_live.py`): `kiwirecorder` streams raw I/Q to its standard output into the twin, which buffers about 2 s and serves the board continuously. When the firmware's LO moves outside the stream's 12 kHz, the twin restarts the stream centred on the new LO (one connection at a time; a restart takes 1–3 s, during which the board sees noise only, like relays switching). The level reference comes from the receiver's S-meter, re-read on each retune.
   - Both honour the public receivers' rules: one connection, a user name identifying the Elf project, time limits, and no automated scraping of the receiver list (the user names the receiver).
7. **AM in the host DSP:** `NeoRadio.Dsp` gains an AM demodulator: a channel filter (default 9 kHz wide), envelope detection with a carrier-tracking option (synchronous AM against selective fading), DC removal and AGC; the CLI gains `--mode am`. The twin's CW/FT8 scenarios are unchanged; a new scenario renders an AM station whose programme (a tone sequence) the test recovers.

## 3. Components

| File | Role |
|---|---|
| `sim/neosim/kiwi_live.py` | Live KiwiSDR stream into the twin: `kiwirecorder` subprocess, buffering, retune on LO change |
| `pi/src/NeoRadio.Dsp` (AM demodulator), `NeoRadio.Dsp.Cli` (`--mode am`) | AM for broadcast listening, with unit tests |
| `tools/fetch_open_iq.py` | Fetch an excerpt of an openly licensed wideband recording (HTTP range request), verify it, write its sidecar and attribution |
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
- **AM demodulator (C#):** a synthetic AM signal (carrier + tone sequence, with and without selective fading) demodulates to the tone sequence; DC and level are stable under AGC.
- **Live streaming:** a fake `kiwirecorder` (a local process writing a known I/Q stream to stdout) proves buffering, a retune on LO change, and noise-only output during the restart; the real network is used only in a manual check.
- **Open wideband recording (optional, skips when not fetched):** the KF4FIC 40 m excerpt (§6) through the virtual board shows the band's FT8 signals at 7.074 MHz decoding through the shipping DSP and decoder.
- **KiwiSDR recordings and live receptions** of broadcast programmes stay local, never in the repository: their licence does not allow redistribution.

## 5. Out of scope

FM demodulation; committing any third-party recording to the repository (open ones are fetched on demand, §6); controlling a KiwiSDR beyond tuning one I/Q channel.

## 6. Licences of the data and tools (checked 2026-10-13)

| Source | Licence | Use in the project |
|---|---|---|
| **KF4FIC wideband HF recordings** (Jonathan Fields, KF4FIC; HamSCI 2017 eclipse experiment; Zenodo record 846442): 40 m 7.000–7.300 MHz and 20 m 14.000–14.350 MHz, WAV, 96 kHz, 24-bit stereo I/Q, ≈ 1 h per file (2 GB) | **CC-BY 4.0** (from the Zenodo record's metadata): reuse, modification and redistribution allowed, commercial too, with attribution | `tools/fetch_open_iq.py` fetches an excerpt (default 60 s ≈ 35 MB) by HTTP range request into `build/open_iq/`, with `ATTRIBUTION.md` naming the author, the record and the licence; the centre frequency is not in the WAV header and is established from known signals in the band (FT8 at 7.074 / 14.074 MHz) and recorded in the sidecar |
| Other HamSCI 2017 eclipse records on Zenodo (EA4GPZ 20/30/40 m, 33–44 GB single files; others) | CC-BY 4.0 | Usable the same way when wanted |
| LakeShark Signal Corpus (GitHub) | Code MIT; each recording CC0 or CC-BY | Aimed at decoder testing; usable if a suitable HF recording is found |
| kiwiclient (`jks-prv/kiwiclient`) | **No licence file**: all rights reserved by default | Not copied or vendored; the user installs it; the project only runs it as an external program |
| KiwiSDR receptions (recordings or live) | The receiver owner's service; broadcast programmes are the broadcasters' copyright | Local use only; never committed or published |
| Internet Archive SDR recordings found (e.g. SDR-IQ 60 m 2007, SDRSharp 2019) | No licence declared | Not used in the project |

