#!/usr/bin/env python3
"""Elf CW receiver check: tunes an Elf board (the real one, or the virtual board attached
through vhci_hcd) over its CAT serial port to each CW station in a stations file, records I/Q
from its sound card and decodes the Morse.

  tools/elf_cw.py [STATIONS.yaml] [--seconds S] [--workdir DIR]

The board appears as a standard USB sound card ("Elf HF Receiver", S24_3LE, 96 kHz, stereo:
left = I, right = Q) and a CDC serial port; nothing beyond the kernel's snd-usb-audio and
cdc_acm drivers and `arecord` is needed. Decoding uses the project's shipping C# DSP and the
twin's CW decoder. Exit status 0 when every station decodes within the CER limit."""
from __future__ import annotations

import argparse
import glob
import os
import re
import subprocess
import sys
import termios
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))

from neosim import cwrx  # noqa: E402

CARD_NAME = "Elf HF Receiver"
MAX_CER = 0.05


def find_card() -> int:
    for line in Path("/proc/asound/cards").read_text().splitlines():
        m = re.match(r"\s*(\d+)\s+\[.*\]:.*- (.*)$", line)
        if m and m.group(2).strip() == CARD_NAME:
            return int(m.group(1))
    raise SystemExit(f"no sound card named {CARD_NAME!r} (is the board attached?)")


def find_serial() -> str:
    ports = sorted(glob.glob("/dev/serial/by-id/*Elf*"))
    if not ports:
        raise SystemExit("no Elf serial port under /dev/serial/by-id (is the board attached?)")
    return ports[0]


class Cat:
    def __init__(self, path: str):
        self.fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0                      # raw: no input, output or local processing
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN], a[6][termios.VTIME] = 0, 5
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def ask(self, cmd: str, timeout: float = 2.0) -> str:
        os.write(self.fd, cmd.encode())
        out, t0 = b"", time.monotonic()
        while not out.endswith(b";") and time.monotonic() - t0 < timeout:
            out += os.read(self.fd, 64)
        return out.decode(errors="replace")

    def send(self, cmd: str) -> None:
        os.write(self.fd, cmd.encode())


def record_seconds(stations_doc: dict) -> float:
    """The twin loops its stations every duration_s; a window of one loop plus the longest message
    (≤ 8 s at the scenario speeds) contains a whole message wherever it starts."""
    return float(stations_doc.get("duration_s", 16.0)) + 8.0


def record(card: int, seconds: float, path: Path) -> bytes:
    subprocess.run(["arecord", "-q", "-D", f"hw:{card},0", "-f", "S24_3LE", "-c", "2", "-r", "96000", "-t", "raw",
                    "-d", str(int(round(seconds))), str(path)], check=True)
    return path.read_bytes()


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("stations", nargs="?", type=Path, default=ROOT / "sim/scenarios/vboard/cw_bands.yaml")
    ap.add_argument("--seconds", type=float, default=None,
                    help="recording per station (default: the stations file's loop + 8 s, so a whole message is inside)")
    ap.add_argument("--settle", type=float, default=3.0, help="wait after tuning (relays, twin rendering)")
    ap.add_argument("--workdir", type=Path, default=ROOT / "build/elf_cw")
    args = ap.parse_args(argv)

    card, port = find_card(), find_serial()
    cat = Cat(port)
    ident = cat.ask("ID;")
    print(f"board: card {card}, {port}, {ident}")
    if ident != "ID019;":
        raise SystemExit("the serial port does not answer as an Elf board")
    doc = yaml.safe_load(args.stations.read_text())
    stations = [s for s in doc["stations"] if s["mode"] == "cw"]
    seconds = args.seconds or record_seconds(doc)
    failures = 0
    for st in stations:
        rf = int(st["rf_hz"])
        lo = int(rf - cwrx.LO_OFFSET_HZ)
        cat.send(f"FA{lo:011d};")
        if cat.ask("FA;") != f"FA{lo:011d};":
            print(f"FAIL  {st['id']}: the board did not tune to {lo} Hz")
            failures += 1
            continue
        time.sleep(args.settle)
        work = args.workdir / st["id"]
        work.mkdir(parents=True, exist_ok=True)
        raw = record(card, seconds, work / "iq.raw")
        text = cwrx.decode(cwrx.iq_from_s24le(raw), lo, rf, work)
        cer = cwrx.best_cer(text, st["text"])
        ok = cer <= MAX_CER
        failures += not ok
        print(f"{'PASS' if ok else 'FAIL'}  {st['id']} {rf / 1e6:.6f} MHz: {text!r} (CER {cer:.3f})")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
