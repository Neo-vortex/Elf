"""Scenario files: imaginary stations and receive jobs."""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import yaml

from neosim import morse

FS_HZ = 96_000
USABLE_FRACTION = 0.45
WINDOW_HZ = USABLE_FRACTION * FS_HZ  # 43 200 Hz either side of the LO
RECEIVE_MARGIN_HZ = 3_000.0

STATION_MODES = {"ft8", "cw", "carrier"}
RECEIVE_MODES = {"usb", "lsb", "cw"}
DECODERS = {"ft8", "cw"}
_BANDWIDTH_HZ = {"ft8": 50.0, "cw": 100.0, "carrier": 1.0}


class ScenarioError(ValueError):
    """The scenario file is invalid."""


@dataclass(frozen=True)
class Station:
    id: str
    mode: str
    rf_hz: float
    snr_db: float
    msg: str | None = None
    text: str | None = None
    wpm: float | None = None
    audio_hz: float | None = None

    @property
    def bandwidth_hz(self) -> float:
        return _BANDWIDTH_HZ[self.mode]


@dataclass(frozen=True)
class Receive:
    id: str
    mode: str
    dial_hz: float
    decoder: str
    expect: dict


@dataclass(frozen=True)
class Scenario:
    name: str
    lo_hz: float
    duration_s: float
    seed: int
    noise_dbfs: float
    stations: tuple[Station, ...]
    receive: tuple[Receive, ...]


def _require(d: dict, key: str, where: str):
    if key not in d:
        raise ScenarioError(f"{where} needs '{key}'")
    return d[key]


def _station(d: dict) -> Station:
    sid = _require(d, "id", "station")
    where = f"station {sid}"
    mode = _require(d, "mode", where)
    if mode not in STATION_MODES:
        raise ScenarioError(f"{where}: unknown station mode '{mode}'")
    snr = float(_require(d, "snr_db", where))
    if mode == "ft8":
        audio = float(_require(d, "audio_hz", where))
        if not 200 <= audio <= 3000:
            raise ScenarioError(f"{where}: audio_hz must be 200..3000, got {audio}")
        return Station(sid, mode, float(_require(d, "dial_hz", where)) + audio, snr,
                       msg=str(_require(d, "msg", where)), audio_hz=audio)
    rf = float(_require(d, "rf_hz", where))
    if mode == "cw":
        text = str(_require(d, "text", where))
        try:
            morse.keying_units(text)
        except ValueError as e:
            raise ScenarioError(f"{where}: {e}") from e
        wpm = float(_require(d, "wpm", where))
        if not 5 <= wpm <= 60:
            raise ScenarioError(f"{where}: wpm must be 5..60, got {wpm}")
        return Station(sid, mode, rf, snr, text=text, wpm=wpm)
    return Station(sid, mode, rf, snr)


def _receive(d: dict, lo_hz: float) -> Receive:
    rid = _require(d, "id", "receive")
    where = f"receive {rid}"
    mode = _require(d, "mode", where)
    if mode not in RECEIVE_MODES:
        raise ScenarioError(f"{where}: unknown receive mode '{mode}'")
    decoder = _require(d, "decoder", where)
    if decoder not in DECODERS:
        raise ScenarioError(f"{where}: unknown decoder '{decoder}'")
    dial = float(_require(d, "dial_hz", where))
    if abs(dial - lo_hz) + RECEIVE_MARGIN_HZ > WINDOW_HZ:
        raise ScenarioError(f"{where}: dial {dial} Hz is outside the receiver window LO±{WINDOW_HZ:.0f} Hz")
    expect = dict(d.get("expect") or {})
    if decoder == "ft8":
        if not expect.get("contains"):
            raise ScenarioError(f"{where}: ft8 receive needs expect.contains (list of messages)")
        expect["contains"] = [str(m) for m in expect["contains"]]
    else:
        if "text" not in expect:
            raise ScenarioError(f"{where}: cw receive needs expect.text")
        expect.setdefault("max_cer", 0.05)
    return Receive(rid, mode, dial, decoder, expect)


def parse_scenario(data: dict) -> Scenario:
    name = str(_require(data, "name", "scenario"))
    lo = float(_require(data, "lo_hz", "scenario"))
    duration = float(data.get("duration_s", 15.0))
    if not 1.0 <= duration <= 60.0:
        raise ScenarioError(f"duration_s must be 1..60, got {duration}")
    noise = data.get("noise", {"type": "awgn"})
    if noise.get("type") != "awgn":
        raise ScenarioError(f"noise type '{noise.get('type')}' not supported yet: only awgn")
    stations = tuple(_station(s) for s in data.get("stations", []))
    ids = [s.id for s in stations]
    dupes = {i for i in ids if ids.count(i) > 1}
    if dupes:
        raise ScenarioError(f"duplicate station id(s): {sorted(dupes)}")
    receive = tuple(_receive(r, lo) for r in _require(data, "receive", "scenario"))
    adc = data.get("adc", {})
    return Scenario(name, lo, duration, int(data.get("seed", 0)), float(adc.get("noise_dbfs", -100.0)),
                    stations, receive)


def load_scenario(path: str | Path) -> Scenario:
    with open(path, encoding="utf-8") as f:
        return parse_scenario(yaml.safe_load(f))
