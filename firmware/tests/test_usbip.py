"""The virtual board's USB device over USB/IP, through the user-space client (phase 4 spec §1
item 3): enumeration, CAT over CDC, UAC2 controls and isochronous I/Q, unlink and recovery."""
from __future__ import annotations

import os
import socket
import struct
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
BUILD = Path(os.environ.get("ELF_FW_BUILD", ROOT / "build" / "firmware"))
sys.path.insert(0, str(ROOT / "sim"))

from neosim import usbip_client as u  # noqa: E402

TONE_HZ = 1000.0


def _free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


@pytest.fixture
def board():
    exe = BUILD / "elf-virtual"
    if not exe.exists():
        pytest.skip(f"{exe} not built (make fw)")
    port = _free_port()
    proc = subprocess.Popen([str(exe), "--port", str(port), "--tone", str(TONE_HZ), "--seconds", "30"],
                            stdout=subprocess.PIPE, text=True)
    assert "USB/IP on" in proc.stdout.readline()
    yield port
    proc.terminate()
    proc.wait(5)


def _configured(port: int) -> u.Client:
    c = u.Client("127.0.0.1", port)
    assert c.set_configuration(1) == 0
    return c


def _interfaces(cfg: bytes) -> list[tuple[int, int, int, int]]:
    out, k = [], 0
    while k < len(cfg):
        if cfg[k + 1] == 4:                                   # interface: number, alt, class, subclass
            out.append((cfg[k + 2], cfg[k + 3], cfg[k + 5], cfg[k + 6]))
        k += cfg[k]
    return out


def _endpoints(cfg: bytes) -> dict[int, tuple[int, int]]:
    out, k = {}, 0
    while k < len(cfg):
        if cfg[k + 1] == 5:                                   # endpoint: address -> (type, max packet)
            out[cfg[k + 2]] = (cfg[k + 3] & 3, cfg[k + 4] | cfg[k + 5] << 8)
        k += cfg[k]
    return out


def _samples(packets) -> np.ndarray:
    raw = np.frombuffer(b"".join(packets), dtype=np.uint8).reshape(-1, 3).astype(np.int32)
    v = raw[:, 0] | raw[:, 1] << 8 | raw[:, 2] << 16
    v = np.where(v >= 1 << 23, v - (1 << 24), v)
    return v[0::2] + 1j * v[1::2]


def test_device_list_names_the_board_and_its_interfaces(board):
    (d,) = u.devlist("127.0.0.1", board)
    assert (d.busid, d.vid, d.pid, d.speed) == ("1-1", 0x1209, 0x0001, 2)        # full speed
    assert [i[0] for i in d.interfaces] == [1, 1, 2, 10]                          # audio ×2, CDC, CDC data


def test_descriptors_describe_uac2_iq_and_cdc(board):
    with u.Client("127.0.0.1", board) as c:
        dev = c.get_descriptor(1, 0, 18)
        assert struct.unpack("<HH", dev[8:12]) == (0x1209, 0x0001)
        total = struct.unpack("<H", c.get_descriptor(2, 0, 9)[2:4])[0]
        cfg = c.get_descriptor(2, 0, total)
        assert len(cfg) == total
        assert c.string(2) == "Elf HF Receiver"
    assert _interfaces(cfg) == [(0, 0, 1, 1), (1, 0, 1, 2), (1, 1, 1, 2), (2, 0, 2, 2), (3, 0, 10, 0)]
    eps = _endpoints(cfg)
    assert eps[0x81] == (1, 582)                    # isochronous IN: 97 frames × 6 bytes headroom
    assert eps[0x03] == (2, 64) and eps[0x83] == (2, 64) and eps[0x82][0] == 3
    k = cfg.index(bytes([6, 0x24, 2, 1, 3, 24]))    # UAC2 type I format: 3-byte subslot, 24 bits
    assert k > 0


def test_cat_round_trip_over_cdc(board):
    with _configured(board) as c:
        assert c.control(0x21, 0x22, 1, 2).status == 0                       # DTR
        cmd = b"ID;FA00014050000;FA;XX;"
        assert c.bulk_out(3, cmd).actual == len(cmd)
        got = b""
        while not got.endswith(b"?;"):
            got += c.bulk_in(3, 64).data
        assert got == b"ID019;FA00014050000;?;"


def test_sample_rate_controls_report_96_khz(board):
    with _configured(board) as c:
        cur = c.control(0xA1, 0x01, 0x0100, 0x0400, 4)                    # CUR, SAM_FREQ, clock 4
        assert cur.status == 0 and int.from_bytes(cur.data, "little") == 96000
        rng = c.control(0xA1, 0x02, 0x0100, 0x0400, 14)
        assert struct.unpack("<HIII", rng.data) == (1, 96000, 96000, 0)
        assert c.control(0xA1, 0x01, 0x0200, 0x0400, 1).data == b"\x01"    # clock valid


def test_unsupported_requests_stall(board):
    with _configured(board) as c:
        assert c.control(0x80, 0x55, 0, 0, 4).status == u.EPIPE
        assert c.control(0xA1, 0x01, 0x0300, 0x0400, 4).status == u.EPIPE   # unknown clock control
        assert c.string(2) == "Elf HF Receiver"                              # and the device carries on


def test_iso_stream_is_continuous_at_the_sample_rate(board):
    with _configured(board) as c:
        assert c.set_interface(1, 1) == 0
        t0 = time.monotonic()
        packets = []
        for _ in range(50):
            packets += c.iso_in(1, 8, 582).iso
        elapsed = time.monotonic() - t0
        assert c.set_interface(1, 0) == 0
    iq = _samples(packets[2:])                       # the first packets prime the stream
    assert len(packets) == 400
    assert 0.3 <= elapsed <= 0.6                     # one packet per 1 ms frame: 400 ms of audio
    # 96 kHz: an asynchronous UAC2 source may send one frame more or less per packet
    assert abs(len(iq) - 96 * 398) <= 398
    assert np.allclose(np.abs(iq), 2 ** 22, rtol=1e-6)
    phase = np.unwrap(np.angle(iq))
    slope, icept = np.polyfit(np.arange(len(phase)), phase, 1)
    assert slope * 96000 / (2 * np.pi) == pytest.approx(TONE_HZ, abs=1e-6)
    assert np.max(np.abs(phase - (slope * np.arange(len(phase)) + icept))) < 1e-5   # no lost or repeated sample


def test_a_held_urb_can_be_unlinked(board):
    with _configured(board) as c:
        seq = c.submit(3, u.DIR_IN, 64)               # nothing to read: held like a NAK
        assert c.unlink(seq) == -104                   # -ECONNRESET
        assert c.bulk_out(3, b"ID;").status == 0
        assert c.bulk_in(3, 64).data == b"ID019;"


def test_a_malformed_urb_drops_the_client_but_not_the_server(board):
    with _configured(board) as c:
        hdr = struct.pack(">IIIIIIiiiI", u.CMD_SUBMIT, 99, 0x10002, 1, 0x83, 0, 64, 0, -1, 0) + bytes(8)
        c.sock.sendall(hdr)                            # endpoint field 0x83: not a number 0–15
        with pytest.raises((u.UsbipError, ConnectionError, OSError)):
            c.wait(99)
    with _configured(board) as c:                      # a new client imports the device again
        assert c.string(2) == "Elf HF Receiver"
