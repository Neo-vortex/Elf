"""ngspice reference for the fast ladder model (neohw.synth.ladder): today's filter n selected in
the full bank (blocks.bpf_bank), every filter-n inductor and capacitor replaced by the robust
model the fast model uses, swept per frequency segment."""
from __future__ import annotations

import math

import numpy as np

from neohw import inductors, parts
from neohw.blocks import bpf, bpf_bank
from neohw.checks import db
from neohw.checks.bpf import _ports
from neohw.ngspice import run_decks
from neohw.spice import Sp, to_deck
from neohw.synth import ladder

CAP_Q = 500.0


def _robust_r(henries: float, f_seg_hz: float) -> float:
    """robust_r of the nearest row scaled to `henries`, as inductors.robust_z scales it."""
    r = inductors.nearest_row(inductors.filter_series(henries), henries)
    return inductors.robust_r(inductors.Row(r.series, r.mpn, henries, r.q_min, r.f_test_hz, r.srf_min_hz,
                                            r.dcr_max, r.tol), f_seg_hz)


def fixed_r_z(henries: float, f_hz, f_seg_hz: float):
    """inductors.robust_z with the series R evaluated at `f_seg_hz` instead of at each f."""
    series = inductors.filter_series(henries)
    row = inductors.nearest_row(series, henries)
    w = 2 * math.pi * np.asarray(f_hz, dtype=float)
    z = _robust_r(henries, f_seg_hz) + 1j * w * (henries + inductors.lead_l(series))
    return z / (1 + 1j * w * inductors.c_par(row) * z)


def segment_losses(f_seg_hz: float, f_centre_hz: float) -> ladder.Losses:
    """Fast-model losses for comparing one frequency segment with ngspice.

    ngspice AC has no frequency-dependent R, so the reference fixes each inductor's R for the
    whole segment (the band centre in band, 3·centre over the 3f band). The fast model must use
    the same fixed R, not robust_z's R(f), or the comparison measures the two loss rules instead
    of the ladder algebra. Capacitors: ESR from Q = 500 at the centre on both sides."""
    return ladder.Losses(lambda h, f: fixed_r_z(h, f, f_seg_hz), CAP_Q, f_centre_hz)


def _inductor_lines(henries: float, f_seg_hz: float, part):
    """Same topology as robust_z: L + lead L in series with R, C_par across the string."""
    series = inductors.filter_series(henries)
    l_tot = henries + inductors.lead_l(series)
    r_s = _robust_r(henries, f_seg_hz)
    c_p = inductors.c_par(inductors.nearest_row(series, henries))

    def lines(n, k=1.0):
        mid = f"{part.ref}_m"
        return [f"L{part.ref} {n['1']} {mid} {parts._num(l_tot)}",
                f"R{part.ref} {mid} {n['2']} {parts._num(r_s)}",
                f"C{part.ref}_p {n['1']} {n['2']} {parts._num(c_p)}"]
    return lines


def _short_lines(part):
    return lambda n, k=1.0: [f"R{part.ref} {n['1']} {n['2']} 1e-6"]


def _segments(index: int, f: np.ndarray) -> list[tuple[float, np.ndarray]]:
    """(R evaluation frequency, points) per segment: in band at the centre, the 3f band at
    3·centre. Each segment must be a linear grid (one ngspice `sp lin` sweep)."""
    d = bpf.design(index)
    in_band = (f >= d.lo_hz * (1 - 1e-9)) & (f <= d.hi_hz * (1 + 1e-9))
    third = (f >= 3 * d.lo_hz * (1 - 1e-9)) & (f <= 3 * d.hi_hz * (1 + 1e-9))
    if not np.all(in_band | third):
        raise ValueError("every frequency must lie in the band or in the 3f band")
    out = []
    for mask, f_seg in ((in_band, d.centre_hz), (third, 3 * d.centre_hz)):
        pts = f[mask]
        if len(pts) and not np.allclose(np.diff(pts), (pts[-1] - pts[0]) / max(len(pts) - 1, 1)):
            raise ValueError("each segment must be an ascending linear grid")
        out.append((f_seg, mask))
    return out


def _deck(index: int, f_seg_hz: float, pts: np.ndarray):
    """Filter `index` selected in the bank: each of its inductor positions is one robust choke
    of the position's ideal value (the first part of the group, the others shorted), each
    capacitor position one capacitor of the ideal value with ESR from Q = 500 at the centre
    (the first slot, the others removed): exactly the elements ladder.from_positions gives."""
    d = bpf.design(index)
    blk = bpf_bank.build(index)
    prefix = f"f{index}_p"
    groups: dict[str, list] = {}
    for p in blk.circuit.parts:
        if hasattr(p, "select_group") and p.select_group.startswith(prefix):
            groups.setdefault(p.select_group, []).append(p)
    caps: dict[tuple, list] = {}
    for p in blk.circuit.parts:
        if p.ref.startswith("C") and not hasattr(p, "select_group"):
            ends = tuple(sorted(pin.net.name for pin in p.pins))
            caps.setdefault(ends, []).append(p)
    nodes = _position_nodes(d, index)
    for k, pos in enumerate(d.positions):
        if pos.kind.startswith("L"):
            members = sorted(groups[f"{prefix}{k + 1}"], key=lambda p: int(p.ref[1:]))
            for j, p in enumerate(members):
                p.variants = None
                p.spice_lines = _inductor_lines(pos.ideal, f_seg_hz, p) if j == 0 else _short_lines(p)
        else:
            members = sorted(caps[tuple(sorted(nodes[k]))], key=lambda p: int(p.ref[1:]))
            esr = 1 / (2 * math.pi * d.centre_hz * pos.ideal * CAP_Q)
            for j, p in enumerate(members):
                p.variants = None
                p.spice_lines = parts._capacitor_lines(pos.ideal, esr, 0.0, p) if j == 0 else (lambda n, k=1.0: [])
    return to_deck(blk.circuit, f"bpf{index} robust model R at {f_seg_hz:g} Hz", _ports(blk),
                   [Sp(pts[0], pts[-1], len(pts), "lin")])


def _position_nodes(d, index: int) -> list[tuple[str, str]]:
    """The two net names each position connects, as blocks.bpf.add_filter names them."""
    series_at = [k for k, pos in enumerate(d.positions) if pos.kind in ("C_series", "L_series")]
    node, out = f"f{index}_in", []
    for k, pos in enumerate(d.positions):
        if k in series_at:
            nxt = f"f{index}_out" if k == series_at[-1] else f"f{index}_n{k + 1}"
            out.append((node, nxt))
            node = nxt
        else:
            out.append((node, "GND"))
    return out


def spice_s21_db(index: int, f) -> np.ndarray:
    """|S21| in dB from ngspice at `f` (points in the band and/or the 3f band, each a linear
    grid), filter `index` selected in the bank with the robust model (R fixed per segment)."""
    f = np.asarray(f, dtype=float)
    out = np.full(len(f), np.nan)
    for f_seg, mask in _segments(index, f):
        pts = f[mask]
        if not len(pts):
            continue
        deck, raws = _deck(index, f_seg, pts)
        (plot,) = run_decks([deck], raws)[0][raws[0]]
        got = plot.data["frequency"].real
        if len(got) != len(pts) or not np.allclose(got, pts, rtol=1e-9):
            raise RuntimeError("ngspice returned a different frequency grid")
        out[mask] = db(plot.data["s_2_1"])
    return out
