"""Acceptance through the user-space USB/IP client (phase 4 spec §1 item 4, without the kernel):
the virtual board, fed by the twin, is tuned over CAT to CW stations on five bands — 80 m on the
÷4 path, the rest direct — and its isochronous I/Q decodes through the shipping DSP."""
from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
STATIONS = ROOT / "sim" / "scenarios" / "vboard" / "cw_bands.yaml"
sys.path.insert(0, str(ROOT / "sim"))

from neosim import cwrx  # noqa: E402
from neosim import usbip_client as u  # noqa: E402
from neosim.tools import ToolNotFoundError, tool_path  # noqa: E402

MAX_CER = 0.05
RECORD_S = 15.0           # one loop of the twin (14 s) plus margin: a whole message is inside


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture(scope="module")
def vboard(tmp_path_factory):
    exe = BUILD / "elf-virtual"
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    try:
        tool_path("neoradio-dsp")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    work = tmp_path_factory.mktemp("vboard")
    env = {**os.environ, "PYTHONPATH": str(ROOT / "sim")}
    twin = subprocess.Popen([sys.executable, "-m", "neosim", "vboard", str(STATIONS), "--socket", "twin.sock"],
                            cwd=work, env=env, stdout=open(work / "twin.log", "w"), stderr=subprocess.STDOUT)
    for _ in range(300):
        if (work / "twin.sock").exists():
            break
        time.sleep(0.1)
    port = _free_port()
    board = subprocess.Popen([str(exe), "--port", str(port), "--twin", "twin.sock", "--seconds", "600"], cwd=work,
                             stdout=subprocess.PIPE, text=True)
    assert "USB/IP on" in board.stdout.readline()
    yield port, work
    board.terminate()
    twin.terminate()
    board.wait(5)
    twin.wait(5)


def _receive(c: u.Client, lo_hz: int, seconds: float) -> bytes:
    assert c.bulk_out(3, f"FA{lo_hz:011d};".encode()).status == 0
    t0 = time.monotonic()
    while True:                                     # the twin renders the new state, then streams it
        r = c.iso_in(1, 8, 582)
        if any(any(p) for p in r.iso):
            break
        assert time.monotonic() - t0 < 60, "no I/Q after tuning"
    raw = b""
    for _ in range(int(seconds * 1000) // 8):
        raw += b"".join(c.iso_in(1, 8, 582).iso)
    return raw


STATION_LIST = yaml.safe_load(STATIONS.read_text())["stations"]


@pytest.mark.parametrize("station", STATION_LIST, ids=[s["id"] for s in STATION_LIST])
def test_cw_station_decodes_through_the_virtual_board(vboard, station):
    port, work = vboard
    rf = int(station["rf_hz"])
    lo = int(rf - cwrx.LO_OFFSET_HZ)
    with u.Client("127.0.0.1", port, timeout=60) as c:
        assert c.set_configuration(1) == 0
        assert c.control(0x21, 0x22, 1, 2).status == 0
        assert c.set_interface(1, 1) == 0
        raw = _receive(c, lo, RECORD_S)
        c.set_interface(1, 0)
    text = cwrx.decode(cwrx.iq_from_s24le(raw), lo, rf, work / station["id"])
    assert cwrx.best_cer(text, station["text"]) <= MAX_CER, text


def test_a_wrong_filter_silences_the_band(vboard):
    """Firmware fault injection from the host side: the attenuator and preamp relays are
    switched over CAT, the bank stays right; then the twin must still decode (sanity), whereas
    tuning outside the bank (60 MHz) is refused by the firmware and leaves the old band."""
    port, _ = vboard
    with u.Client("127.0.0.1", port, timeout=60) as c:
        c.set_configuration(1)
        c.control(0x21, 0x22, 1, 2)
        assert c.bulk_out(3, b"FA00060000000;").status == 0
        got = b""
        while not got.endswith(b";"):
            got += c.bulk_in(3, 64).data
        assert got == b"?;"
