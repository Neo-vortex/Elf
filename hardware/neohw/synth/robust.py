"""Low-loss filter synthesis by optimisation against the robust model (spec 2026-10-19 §4).

Each topology is an HPF section (Chebyshev, series C / shunt L) followed by an LPF section:
Chebyshev (shunt C / series L) or elliptic (the series inductors become parallel-LC tanks whose
zeros start in the 3f band). The optimiser minimises the worst in-band IL t (epigraph form) under
RL, 3f rejection and selectivity-floor constraints, all in the fast ladder model with the relay
bus loading, over log element values within the buyable range. SLSQP can stall from a poor start,
so each topology is tried from the seed and then from seeds perturbed by up to ±10 %."""
from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass

import numpy as np
from scipy.optimize import minimize

from neohw.synth import ladder
from neohw.synth.filters import highpass, lowpass

L_RANGE = (0.1e-6, 1500e-6)    # inside the inductor data (robust_z clamps to the end rows outside it)
C_RANGE = (1e-12, 100e-9)
N_BAND, N_REJ = 41, 41
SEED_RIPPLE_DB = 0.02
STARTS = 5                     # the seed, then perturbed seeds
PERTURB = 0.10                 # ×(1 ± 10 %) per element value
MAX_ITER = 1000
SEL_TOL_DB = 1e-6              # selectivity floors are met to this
SOLVE_MARGIN_DB = 1e-3         # the solver aims this far inside each floor: SLSQP stops a hair outside active constraints


@dataclass(frozen=True)
class Targets:
    index: int
    lo: float
    hi: float
    il_max: float
    rl_min: float
    rej_min: float | None
    att_half_lo_min: float | None
    att_2hi_min: float


@dataclass(frozen=True)
class Topology:
    hpf_order: int
    lpf: str


TOPOLOGIES = tuple(Topology(h, l) for h in (3, 5) for l in ("ell5", "ell7", "cheb7", "cheb9"))


@dataclass(frozen=True)
class Result:
    topology: Topology
    branches: tuple
    metrics: dict
    feasible: bool
    violations: dict


def today_selectivity(index: int) -> dict:
    """Attenuation at lo/2 and 2·hi of today's Chebyshev design (bpf.design) in the robust model
    with the bank bus: the selectivity reference. Task 6 replaces bpf.design; this then reads the
    values recorded in the designs file."""
    from neohw.blocks import bpf
    d = bpf.design(index)
    m = ladder.metrics(ladder.from_positions(d.positions), d.lo_hz, d.hi_hz,
                       ladder.robust_losses(d.centre_hz), ladder.bank_bus())
    return {"att_half_lo_db": m["att_half_lo_db"], "att_2hi_db": m["att_2hi_db"]}


def targets(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Targets:
    """Block targets with margin (rl_min, rej_min) and selectivity floors: today's attenuation
    less 3 dB, at least 20 dB (today's value where it is already below 20 dB)."""
    from neohw.blocks import bpf
    from neohw.checks import bpf as check
    lo, hi = bpf.BANDS[index - 1]
    today = today_selectivity(index)

    def floor(v):
        return None if v is None else min(v, max(20.0, v - 3.0))

    return Targets(index, lo, hi, check.il_limit_db(index), rl_min, rej_min if 3 * lo > hi else None,
                   floor(today["att_half_lo_db"]), floor(today["att_2hi_db"]))


def _hpf(lo, order):
    from neohw.blocks.bpf import EDGE_MARGIN
    return [ladder.Branch("series", "C", c=e.value) if e.kind == "C_series" else ladder.Branch("shunt", "L", l=e.value)
            for e in highpass(lo / EDGE_MARGIN, order, SEED_RIPPLE_DB)]


def _lpf(lo, hi, kind):
    order = int(kind[-1])
    els = lowpass(hi * 1.03, order, SEED_RIPPLE_DB)
    out = [ladder.Branch("shunt", "C", c=e.value) if e.kind == "C_shunt" else ladder.Branch("series", "L", l=e.value)
           for e in els]
    if kind.startswith("ell"):
        series_l = [k for k, b in enumerate(out) if b.form == "L"]
        f_lo, f_hi = (3 * lo, 3 * hi) if 3 * lo > hi else (2 * hi, 4 * hi)
        zeros = np.geomspace(f_lo, f_hi * 1.2, len(series_l))
        for k, fz in zip(series_l, zeros):
            l = out[k].l
            out[k] = ladder.Branch("series", "LC_par", l=l, c=1 / ((2 * math.pi * fz) ** 2 * l))
    return out


def seed(top: Topology, lo: float, hi: float) -> list:
    return _hpf(lo, top.hpf_order) + _lpf(lo, hi, top.lpf)


def _pack(branches):
    x, slots = [], []
    for k, b in enumerate(branches):
        if b.form != "C":
            x.append(math.log(b.l)); slots.append((k, "l"))
        if b.form != "L":
            x.append(math.log(b.c)); slots.append((k, "c"))
    return np.array(x), slots


def _unpack(x, slots, template):
    vals = {k: {"l": b.l, "c": b.c} for k, b in enumerate(template)}
    for v, (k, which) in zip(x, slots):
        vals[k][which] = math.exp(v)
    return [ladder.Branch(b.place, b.form, l=vals[k]["l"], c=vals[k]["c"]) for k, b in enumerate(template)]


def _violations(m, t: Targets) -> dict:
    v = {}
    if m["il_db"] > t.il_max:
        v["il_db"] = m["il_db"] - t.il_max
    if m["rl_db"] < t.rl_min:
        v["rl_db"] = t.rl_min - m["rl_db"]
    if t.rej_min is not None and m["rej3f_db"] < t.rej_min:
        v["rej3f_db"] = t.rej_min - m["rej3f_db"]
    if m["att_2hi_db"] < t.att_2hi_min - SEL_TOL_DB:
        v["att_2hi_db"] = t.att_2hi_min - m["att_2hi_db"]
    if t.att_half_lo_min is not None and m["att_half_lo_db"] < t.att_half_lo_min - SEL_TOL_DB:
        v["att_half_lo_db"] = t.att_half_lo_min - m["att_half_lo_db"]
    return v


def _rank(r: Result):
    """Feasible before infeasible; then lower IL, or smaller total violation."""
    return (0, r.metrics["il_db"]) if r.feasible else (1, sum(r.violations.values()))


def _solve(top: Topology, t: Targets, x0, slots, template, max_iter: int) -> Result:
    losses = ladder.robust_losses(math.sqrt(t.lo * t.hi))
    bus = ladder.bank_bus()
    band = np.linspace(t.lo, t.hi, N_BAND)
    rej = np.linspace(3 * t.lo, 3 * t.hi, N_REJ) if t.rej_min is not None else np.array([])
    sel = [(t.hi * 2, t.att_2hi_min)] + ([(t.lo / 2, t.att_half_lo_min)] if t.att_half_lo_min is not None else [])
    stop = np.concatenate([rej, [f for f, _ in sel]])
    sel_min = np.array([m for _, m in sel])
    lb = [math.log(L_RANGE[0]) if w == "l" else math.log(C_RANGE[0]) for _, w in slots]
    ub = [math.log(L_RANGE[1]) if w == "l" else math.log(C_RANGE[1]) for _, w in slots]
    x0 = np.clip(x0, lb, ub)
    bounds = list(zip(lb, ub)) + [(0.0, 60.0)]

    def cons(x):
        br = _unpack(x[:-1], slots, template)
        s21, s11 = ladder.s21_s11(br, band, losses, bus)
        att = -ladder.db(ladder.s21_s11(br, stop, losses, bus)[0])
        parts_ = [x[-1] + ladder.db(s21), -ladder.db(s11) - t.rl_min - SOLVE_MARGIN_DB]
        if t.rej_min is not None:
            parts_.append(att[:len(rej)] - t.rej_min - SOLVE_MARGIN_DB)
        parts_.append(att[len(rej):] - sel_min - SOLVE_MARGIN_DB)
        return np.concatenate(parts_)

    br0 = _unpack(x0, slots, template)
    il0 = float(-ladder.db(ladder.s21_s11(br0, band, losses, bus)[0]).min())
    res = minimize(lambda x: x[-1], np.append(x0, min(il0, 60.0)), method="SLSQP", bounds=bounds,
                   jac=lambda x: np.eye(len(x))[-1],
                   constraints=[{"type": "ineq", "fun": cons}], options={"maxiter": max_iter, "ftol": 1e-6})
    br = _unpack(res.x[:-1], slots, template)
    m = ladder.metrics(br, t.lo, t.hi, losses, bus)
    v = _violations(m, t)
    return Result(top, tuple(br), m, not v, v)


def optimise(top: Topology, t: Targets, max_iter: int = MAX_ITER, starts: int = STARTS) -> Result:
    """The best of up to `starts` SLSQP runs: the seed first, then seeds with every element value
    scaled by a random factor in 1 ± PERTURB (fixed generator, so results repeat). Stops early
    once a run is feasible and the next start does not improve IL by more than 0.01 dB."""
    template = seed(top, t.lo, t.hi)
    x_seed, slots = _pack(template)
    rng = np.random.default_rng(1000 * t.index + 10 * top.hpf_order + int(top.lpf[-1]))
    best_r = None
    for k in range(starts):
        x0 = x_seed if k == 0 else x_seed + np.log(1 + rng.uniform(-PERTURB, PERTURB, len(x_seed)))
        r = _solve(top, t, x0, slots, template, max_iter)
        if best_r is not None and best_r.feasible and r.feasible and r.metrics["il_db"] > best_r.metrics["il_db"] - 0.01:
            return best_r if _rank(best_r) <= _rank(r) else r
        if best_r is None or _rank(r) < _rank(best_r):
            best_r = r
    return best_r


def _optimise_args(args):
    return optimise(*args)


def best(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Result:
    """The lowest-IL feasible result over TOPOLOGIES (optimised in parallel), or the infeasible
    one with the smallest total violation."""
    t = targets(index, rl_min, rej_min)
    with ProcessPoolExecutor(max_workers=min(len(TOPOLOGIES), os.cpu_count() or 1)) as pool:
        results = list(pool.map(_optimise_args, [(top, t) for top in TOPOLOGIES]))
    return min(results, key=_rank)
