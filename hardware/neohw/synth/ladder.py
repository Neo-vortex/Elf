"""Fast ABCD model of a filter ladder between equal terminations (plan 5.1a, spec 2026-10-19 §4).

Branches run from the source side; a series branch is an impedance in the through path, a shunt
branch an admittance to ground. Inductor impedance comes from a loss model (by default the robust
model of neohw.inductors); capacitors carry a series loss giving `cap_q` at `f_cap_q_hz`. The
relay bank loads each filter port (`Bus`). ngspice remains the authority for reported numbers."""
from __future__ import annotations

import inspect
import math
from dataclasses import dataclass
from typing import Callable

import numpy as np

from neohw import inductors

Z0 = 50.0
PLACES, FORMS = ("series", "shunt"), ("L", "C", "LC_par", "LC_ser")


@dataclass(frozen=True)
class Branch:
    place: str
    form: str
    l: float = 0.0
    c: float = 0.0

    def __post_init__(self):
        if self.place not in PLACES or self.form not in FORMS:
            raise ValueError(f"branch place/form must be in {PLACES}/{FORMS}, got {self.place!r}/{self.form!r}")
        if (self.form != "C" and self.l <= 0) or (self.form != "L" and self.c <= 0):
            raise ValueError(f"{self.form} branch needs positive values, got l={self.l!r} c={self.c!r}")


@dataclass(frozen=True)
class Losses:
    inductor_z: Callable[[float, np.ndarray], np.ndarray]
    cap_q: float = 500.0
    f_cap_q_hz: float = 1e6


def robust_losses(f_centre_hz: float) -> Losses:
    return Losses(inductors.robust_z, 500.0, f_centre_hz)


@dataclass(frozen=True)
class Bus:
    shunt_c: float = 0.0     # relay open-contact and inter-pole capacitance at each filter port
    series_r: float = 0.0    # closed contact of the selected filter's relay


BANK_SIZE = 11


def bank_bus(n: int = BANK_SIZE) -> Bus:
    """Loading of one selected filter in the n-filter bank, from the relay model in
    neohw.parts.relay_dpdt (its defaults are read, not copied). Each port has its own relay,
    pole 1 switching, pole 2 and NC grounded (blocks.bpf._switch):
    - each of the n − 1 released relays puts its open NO–COM contact (COM grounded through the
      closed NC contact) and the NO1–NO2 inter-pole C on the bus;
    - the selected relay puts NO1–NO2 on the bus side and, on the filter side, its open
      COM–NC contact and the COM1–COM2 inter-pole C; its closed contact is in series.
    The filter-side capacitance is lumped with the bus side (it is across 0.05 Ω)."""
    from neohw import parts
    sig = inspect.signature(parts.relay_dpdt).parameters
    open_c, contact_r, inter_pole_c = (sig[k].default for k in ("open_c", "contact_r", "inter_pole_c"))
    return Bus(shunt_c=(n - 1) * (open_c + inter_pole_c) + inter_pole_c + (open_c + inter_pole_c),
               series_r=contact_r)


def _cap_z(c, w, losses: Losses):
    esr = 1 / (2 * math.pi * losses.f_cap_q_hz * c * losses.cap_q)
    return esr + 1 / (1j * w * c)


def branch_z(b: Branch, f: np.ndarray, losses: Losses) -> np.ndarray:
    w = 2 * math.pi * f
    zl = losses.inductor_z(b.l, f) if b.form != "C" else None
    zc = _cap_z(b.c, w, losses) if b.form != "L" else None
    if b.form == "L":
        return zl
    if b.form == "C":
        return zc
    if b.form == "LC_ser":
        return zl + zc
    return zl * zc / (zl + zc)


def s21_s11(branches, f, losses: Losses, bus: Bus = Bus(), z0: float = Z0):
    """S21 and S11 of the ladder between two z0 terminations, each port loaded by `bus`.
    The chain matrix is built left to right: a series z multiplies by [[1, z], [0, 1]],
    a shunt y by [[1, 0], [y, 1]]."""
    f = np.asarray(f, dtype=float)
    one, zero = np.ones_like(f, dtype=complex), np.zeros_like(f, dtype=complex)
    a, b, c, d = one, zero, zero, one

    def series(z):
        nonlocal b, d
        b, d = a * z + b, c * z + d

    def shunt(y):
        nonlocal a, c
        a, c = a + b * y, c + d * y

    w = 2 * math.pi * f
    if bus.shunt_c:
        shunt(1j * w * bus.shunt_c)
    if bus.series_r:
        series(bus.series_r * one)
    for br in branches:
        z = branch_z(br, f, losses)
        if br.place == "series":
            series(z)
        else:
            shunt(1 / z)
    if bus.series_r:
        series(bus.series_r * one)
    if bus.shunt_c:
        shunt(1j * w * bus.shunt_c)
    den = a + b / z0 + c * z0 + d
    return 2 / den, (a + b / z0 - c * z0 - d) / den


def db(x) -> np.ndarray:
    return 20 * np.log10(np.maximum(np.abs(x), 1e-30))


def metrics(branches, lo, hi, losses: Losses, bus: Bus = Bus(), n: int = 41) -> dict:
    """Worst in-band IL and RL; worst 3f rejection (None when 3·lo ≤ hi); attenuation at lo/2
    (None below 50 kHz) and at 2·hi. All in dB, positive."""
    band = np.linspace(lo, hi, n)
    s21, s11 = s21_s11(branches, band, losses, bus)
    out = {"il_db": float(-db(s21).min()), "rl_db": float(-db(s11).max()), "rej3f_db": None,
           "att_half_lo_db": None, "att_2hi_db": float(-db(s21_s11(branches, [2 * hi], losses, bus)[0])[0])}
    if 3 * lo > hi:
        out["rej3f_db"] = float(-db(s21_s11(branches, np.linspace(3 * lo, 3 * hi, n), losses, bus)[0]).max())
    if lo / 2 >= 50e3:
        out["att_half_lo_db"] = float(-db(s21_s11(branches, [lo / 2], losses, bus)[0])[0])
    return out


def from_positions(positions) -> list[Branch]:
    """Today's bpf.Position list (C_series, L_shunt, C_shunt, L_series) as branches, using each
    position's ideal value."""
    out = []
    for p in positions:
        place = "series" if p.kind.endswith("series") else "shunt"
        out.append(Branch(place, "L", l=p.ideal) if p.kind.startswith("L") else Branch(place, "C", c=p.ideal))
    return out
