"""The digital twin behind the virtual board (phase 4 spec §3): elf-virtual sends what its
firmware wrote to the hardware — the Si5351C register image, the relay outputs and the LO path
select — and receives the CS5361's I2S frames for that state.

The state is decoded here independently of the firmware: the LO from the registers (AN619
arithmetic, si5351_decode), the filter, attenuator and preamp from the relay bits (the order the
board netlist wires them, controller.json). The stations are rendered through the hardware
model for exactly that state, so a firmware fault becomes a wrong or silent band:

- no valid quadrature LO (wrong divider, phase offset or path select): receiver noise only;
- a bank state other than one filter's two relays, or a filter that does not cover the LO:
  the stations are blocked (receiver noise only).

Socket protocol (Unix stream, little-endian):
  board → twin  b"S" + 256 register bytes + u32 relay bits + u8 path select    (state changed)
  board → twin  b"R" + u32 n                                                 (want n frames)
  twin → board  b"D" + u32 n + n × (i32 I, i32 Q)       (signed 24-bit ADC codes, L = I, R = Q)
"""
from __future__ import annotations

import json
import socket
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import yaml

from neosim import hardware, si5351_decode, sources
from neosim.scenario import FS_HZ, WINDOW_HZ, Receiver, Scenario, _station

CHAR_DIR = Path(__file__).resolve().parents[2] / "hardware" / "characterized"
FULL_SCALE = 2 ** 23


@dataclass(frozen=True)
class BoardState:
    lo_hz: float | None
    path: str
    filter: int                    # 0 = muted / invalid bank state
    attenuator: bool
    preamp: bool
    reason: str                    # why the stations are blocked, or ""


def relay_order(char_dir: Path = CHAR_DIR) -> list[str]:
    return json.loads((char_dir / "controller.json").read_text())["relay_chain_order"]


def decode_state(regs: bytes, relays: int, path_high: bool, order: list[str], hw: hardware.Hardware) -> BoardState:
    lo = si5351_decode.lo_state(regs, path_high)
    on = {name for k, name in enumerate(order) if relays >> k & 1}
    banks = sorted({int(n[4:].split("_")[0]) for n in on if n.startswith("bank")})
    filt = banks[0] if len(banks) == 1 and {f"bank{banks[0]}_in", f"bank{banks[0]}_out"} <= on else 0
    reason = lo.reason
    if not reason and filt == 0:
        reason = f"bank relays {sorted(n for n in on if n.startswith('bank'))} select no single filter"
    if not reason:
        want = hw.filter_index(float(lo.lo_hz))
        if want != filt:
            reason = f"filter {filt} selected for an LO in filter {want}"
    return BoardState(None if lo.lo_hz is None else float(lo.lo_hz), lo.path, filt, "atten" in on, "preamp" in on, reason)


class Twin:
    """Renders I/Q for a board state from a stations file (absolute levels, band noise)."""

    def __init__(self, stations_path: Path, hw: hardware.Hardware | None = None):
        """`stations_path`: a stations file (imaginary stations) or an antenna file (recordings, with
        optional imaginary stations; neosim.antenna), told apart by its `recordings` key."""
        d = yaml.safe_load(Path(stations_path).read_text())
        if "recordings" in d:
            from neosim.antenna import Antenna
            ant = Antenna.load(Path(stations_path))
            self.name, self.duration_s, self.seed = ant.name, ant.duration_s, ant.seed
            self.noise_dbm_per_hz, self.stations = ant.noise_dbm_per_hz, ant.stations
        else:
            self.name = d.get("name", Path(stations_path).stem)
            self.duration_s = float(d.get("duration_s", 16.0))
            self.seed = int(d.get("seed", 1))
            self.noise_dbm_per_hz = float(d["noise"]["dbm_per_hz"])
            self.stations = tuple(_station(s, self.noise_dbm_per_hz) for s in d["stations"])
        self.hw = hw or hardware.Hardware.load()
        self.order = relay_order()

    def render(self, state: BoardState) -> np.ndarray:
        """int32 frames (n, 2) for the state, looped by the server."""
        lo = state.lo_hz if state.lo_hz is not None else 10e6
        stations = () if state.reason else self.stations
        sc = Scenario(self.name, lo, self.duration_s, self.seed, 0.0, stations, (),
                      Receiver("hardware", state.preamp, 20 if state.attenuator else 0), self.noise_dbm_per_hz)
        rng = np.random.default_rng(self.seed)
        iq, _excluded, _stats = hardware.receive(sc, rng, self.hw, sources.render_station, FS_HZ, WINDOW_HZ)
        # receive() returns di − j·dq; the ADC delivers I = di (left) and Q = dq (right)
        frames = np.empty((len(iq), 2), dtype=np.int32)
        frames[:, 0] = np.round(iq.real * FULL_SCALE).astype(np.int32)
        frames[:, 1] = np.round(-iq.imag * FULL_SCALE).astype(np.int32)
        return frames


def _recv(conn: socket.socket, n: int) -> bytes:
    out = bytearray()
    while len(out) < n:
        chunk = conn.recv(n - len(out))
        if not chunk:
            raise ConnectionError("board closed the link")
        out += chunk
    return bytes(out)


def serve(stations_path: Path, socket_path: Path, log=print) -> None:
    """Serves one board connection after another until interrupted."""
    twin = Twin(stations_path)
    socket_path = Path(socket_path)
    socket_path.unlink(missing_ok=True)
    srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    srv.bind(str(socket_path))
    srv.listen(1)
    log(f"vboard: twin for {twin.name!r} on {socket_path}")
    try:
        while True:
            conn, _ = srv.accept()
            with conn:
                _session(conn, twin, log)
    finally:
        srv.close()
        socket_path.unlink(missing_ok=True)


def _session(conn: socket.socket, twin: Twin, log) -> None:
    frames = np.zeros((FS_HZ // 10, 2), dtype=np.int32)       # silence until the first state
    pos = 0
    cache: dict[BoardState, np.ndarray] = {}
    try:
        while True:
            kind = _recv(conn, 1)
            if kind == b"S":
                regs = _recv(conn, 256)
                relays, path = struct.unpack("<IB", _recv(conn, 5))
                state = decode_state(regs, relays, bool(path), twin.order, twin.hw)
                if state not in cache:
                    cache[state] = twin.render(state)
                frames, pos = cache[state], 0
                lo = "none" if state.lo_hz is None else f"{state.lo_hz / 1e6:.6f} MHz"
                log(f"vboard: LO {lo} ({state.path}), filter {state.filter}, preamp {int(state.preamp)}, "
                    f"atten {int(state.attenuator)}{', BLOCKED: ' + state.reason if state.reason else ''}")
            elif kind == b"R":
                (n,) = struct.unpack("<I", _recv(conn, 4))
                idx = (pos + np.arange(n)) % len(frames)
                pos = (pos + n) % len(frames)
                conn.sendall(b"D" + struct.pack("<I", n) + frames[idx].astype("<i4").tobytes())
            else:
                raise ConnectionError(f"unknown message {kind!r}")
    except ConnectionError as e:
        log(f"vboard: {e}")

