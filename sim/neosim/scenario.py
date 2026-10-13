"""Scenario files: imaginary stations and receive jobs."""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import yaml

from neosim import morse

FS_HZ = 96_000
USABLE_FRACTION = 0.45
WINDOW_HZ = USABLE_FRACTION * FS_HZ  # 43 200 Hz either side of the LO
RECEIVE_MARGIN_HZ = 3_000.0

STATION_MODES = {"ft8", "cw", "carrier"}     # plus "recording", made only by neosim.antenna
RECEIVE_MODES = {"usb", "lsb", "cw"}
DECODERS = {"ft8", "cw"}
_BANDWIDTH_HZ = {"ft8": 50.0, "cw": 100.0, "carrier": 1.0}
RECEIVER_MODELS = {"ideal", "hardware"}
ATTENUATOR_DB = {0, 20}
SNR_BANDWIDTH_HZ = 2500.0


class ScenarioError(ValueError):
    """The scenario file is invalid."""


@dataclass(frozen=True)
class Station:
    """`snr_db` is the power while active (key down, FT8 tones) over the noise in 2500 Hz;
    `level_dbm` is the same power at the antenna (absolute scenarios)."""
    id: str
    mode: str
    rf_hz: float
    snr_db: float | None
    msg: str | None = None
    text: str | None = None
    wpm: float | None = None
    audio_hz: float | None = None
    level_dbm: float | None = None
    span_hz: float = 0.0                                          # recordings: the frequency span they cover
    source: object = field(default=None, compare=False, repr=False)   # recordings: the neosim.antenna.Recording

    @property
    def bandwidth_hz(self) -> float:
        return self.span_hz if self.mode == "recording" else _BANDWIDTH_HZ[self.mode]

    def reaches(self, f_ref_hz: float, window_hz: float) -> bool:
        """Whether the station falls in the window f_ref ± window: an ordinary station whole, a
        recording partly (the baseband response cuts the rest)."""
        if self.mode == "recording":
            return abs(self.rf_hz - f_ref_hz) - self.span_hz / 2 < window_hz
        return abs(self.rf_hz - f_ref_hz) + self.bandwidth_hz / 2 <= window_hz


@dataclass(frozen=True)
class Receive:
    id: str
    mode: str
    dial_hz: float
    decoder: str
    expect: dict


@dataclass(frozen=True)
class Receiver:
    model: str = "ideal"
    preamp: bool = False
    attenuator_db: int = 0


@dataclass(frozen=True)
class Scenario:
    """`noise_dbm_per_hz` is set in absolute scenarios (band noise at the antenna, stations
    given as level_dbm); the ideal receiver then works from the equivalent SNRs."""
    name: str
    lo_hz: float
    duration_s: float
    seed: int
    noise_dbfs: float
    stations: tuple[Station, ...]
    receive: tuple[Receive, ...]
    receiver: Receiver = Receiver()
    noise_dbm_per_hz: float | None = None


def _require(d: dict, key: str, where: str):
    if not isinstance(d, dict):
        raise ScenarioError(f"{where} must be a mapping, got {type(d).__name__}")
    if key not in d:
        raise ScenarioError(f"{where} needs '{key}'")
    return d[key]


def _num(value, key: str, where: str, cast=float):
    try:
        return cast(value)
    except (TypeError, ValueError):
        raise ScenarioError(f"{where}: {key} must be a number, got {value!r}") from None


def _station(d: dict, noise_dbm_per_hz: float | None) -> Station:
    sid = _require(d, "id", "station")
    where = f"station {sid}"
    mode = _require(d, "mode", where)
    if mode not in STATION_MODES:
        raise ScenarioError(f"{where}: unknown station mode '{mode}'")
    level = None
    if noise_dbm_per_hz is None:
        if "level_dbm" in d:
            raise ScenarioError(f"{where}: level_dbm needs absolute noise (noise.dbm_per_hz); use snr_db")
        snr = _num(_require(d, "snr_db", where), "snr_db", where)
    else:
        if "snr_db" in d:
            raise ScenarioError(f"{where}: an absolute scenario (noise.dbm_per_hz) gives stations as level_dbm, not snr_db")
        level = _num(_require(d, "level_dbm", where), "level_dbm", where)
        snr = level - (noise_dbm_per_hz + 10 * math.log10(SNR_BANDWIDTH_HZ))
    if mode == "ft8":
        audio = _num(_require(d, "audio_hz", where), "audio_hz", where)
        if not 200 <= audio <= 3000:
            raise ScenarioError(f"{where}: audio_hz must be 200..3000, got {audio}")
        return Station(sid, mode, _num(_require(d, "dial_hz", where), "dial_hz", where) + audio, snr,
                       msg=str(_require(d, "msg", where)), audio_hz=audio, level_dbm=level)
    rf = _num(_require(d, "rf_hz", where), "rf_hz", where)
    if mode == "cw":
        text = str(_require(d, "text", where))
        try:
            morse.keying_units(text)
        except ValueError as e:
            raise ScenarioError(f"{where}: {e}") from e
        wpm = _num(_require(d, "wpm", where), "wpm", where)
        if not 5 <= wpm <= 60:
            raise ScenarioError(f"{where}: wpm must be 5..60, got {wpm}")
        return Station(sid, mode, rf, snr, text=text, wpm=wpm, level_dbm=level)
    return Station(sid, mode, rf, snr, level_dbm=level)


def _receiver(d) -> Receiver:
    if d is None:
        return Receiver()
    if not isinstance(d, dict):
        raise ScenarioError("receiver must be a mapping")
    unknown = set(d) - {"model", "preamp", "attenuator"}
    if unknown:
        raise ScenarioError(f"receiver: unknown key(s) {sorted(unknown)}")
    model = d.get("model", "ideal")
    if model not in RECEIVER_MODELS:
        raise ScenarioError(f"receiver: unknown model '{model}' (ideal or hardware)")
    preamp = d.get("preamp", "off")
    # YAML 1.1 reads a bare on/off as a boolean
    preamp = {True: "on", False: "off"}.get(preamp, preamp)
    if preamp not in ("on", "off"):
        raise ScenarioError(f"receiver: preamp must be on or off, got {preamp!r}")
    att = _num(d.get("attenuator", 0), "attenuator", "receiver", int)
    if att not in ATTENUATOR_DB:
        raise ScenarioError(f"receiver: attenuator must be 0 or 20, got {att}")
    if model == "ideal" and (preamp == "on" or att):
        raise ScenarioError("receiver: preamp and attenuator apply to the hardware model only")
    return Receiver(model, preamp == "on", att)


def _receive(d: dict, lo_hz: float) -> Receive:
    rid = _require(d, "id", "receive")
    where = f"receive {rid}"
    mode = _require(d, "mode", where)
    if mode not in RECEIVE_MODES:
        raise ScenarioError(f"{where}: unknown receive mode '{mode}'")
    decoder = _require(d, "decoder", where)
    if decoder not in DECODERS:
        raise ScenarioError(f"{where}: unknown decoder '{decoder}'")
    dial = _num(_require(d, "dial_hz", where), "dial_hz", where)
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
    if not isinstance(data, dict):
        raise ScenarioError(f"scenario must be a mapping, got {type(data).__name__}")
    name = str(_require(data, "name", "scenario"))
    lo = _num(_require(data, "lo_hz", "scenario"), "lo_hz", "scenario")
    duration = _num(data.get("duration_s", 15.0), "duration_s", "scenario")
    if not 1.0 <= duration <= 60.0:
        raise ScenarioError(f"duration_s must be 1..60, got {duration}")
    noise = data.get("noise", {"type": "awgn"})
    if not isinstance(noise, dict) or noise.get("type") != "awgn":
        raise ScenarioError(f"noise type '{noise.get('type') if isinstance(noise, dict) else noise}' "
                            "not supported yet: only awgn")
    absolute = None
    if "dbm_per_hz" in noise:
        absolute = _num(noise["dbm_per_hz"], "dbm_per_hz", "noise")
        if not -200 <= absolute <= -50:
            raise ScenarioError(f"noise: dbm_per_hz must be -200..-50, got {absolute}")
    receiver = _receiver(data.get("receiver"))
    if receiver.model == "hardware" and absolute is None:
        raise ScenarioError("the hardware receiver needs absolute levels: noise.dbm_per_hz and station level_dbm")
    stations = tuple(_station(s, absolute) for s in data.get("stations", []))
    ids = [s.id for s in stations]
    dupes = {i for i in ids if ids.count(i) > 1}
    if dupes:
        raise ScenarioError(f"duplicate station id(s): {sorted(dupes)}")
    receive = tuple(_receive(r, lo) for r in _require(data, "receive", "scenario"))
    adc = data.get("adc", {})
    if not isinstance(adc, dict):
        raise ScenarioError("adc must be a mapping")
    if receiver.model == "hardware" and adc:
        raise ScenarioError("adc settings apply to the ideal receiver only: the hardware model uses the CS5361 data")
    return Scenario(name, lo, duration, _num(data.get("seed", 0), "seed", "scenario", int),
                    _num(adc.get("noise_dbfs", -100.0), "noise_dbfs", "adc"),
                    stations, receive, receiver, absolute)


def load_scenario(path: str | Path) -> Scenario:
    with open(path, encoding="utf-8") as f:
        try:
            data = yaml.safe_load(f)
        except yaml.YAMLError as e:
            raise ScenarioError(f"{path}: invalid YAML: {e}") from None
    return parse_scenario(data)
