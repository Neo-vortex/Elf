"""Low-loss filter synthesis by optimisation against the robust model (spec 2026-10-19 §4).

Each topology is an HPF section (Chebyshev, series C / shunt L) followed by an LPF section:
Chebyshev (shunt C / series L) or elliptic (the series inductors become parallel-LC tanks whose
zeros start in the 3f band). The optimiser minimises the worst in-band IL t (epigraph form) under
RL, 3f rejection and a stopband-wide selectivity floor, all in the fast ladder model with the
relay bus loading, over log element values within the buyable range. SLSQP can stall from a poor
start, so each topology is tried from the seed and then from seeds perturbed by up to ±10 %.

Selectivity floor (user decision 2026-10-19): at every stopband grid frequency f, attenuation
≥ min(max(20, today(f) − 3), 60) dB, where today(f) is today's Chebyshev design in the same
model. Elements that end at a bound are pruned and the reduced ladder is solved again."""
from __future__ import annotations

import math
import os
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from functools import lru_cache

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

# Stopband grids and floor
N_STOP_HI, N_STOP_LO = 60, 30
STOP_HI_END = 200e6            # high grid: 2·hi .. 200 MHz (2·hi .. 4·hi if 2·hi ≥ 200 MHz)
STOP_LO_START = 10e3           # low grid: max(10 kHz, lo/20) .. lo/2
FLOOR_MIN_DB, FLOOR_BACKOFF_DB, FLOOR_CAP_DB = 20.0, 3.0, 60.0
STOP_OVERSAMPLE = 4            # the floor is held on grids this much finer, which contain the 60/30 points
NOTCH_MIN = 1.3                # every LPF tank's zero at or above 1.3·hi

# Pruning
C_PRUNE = 3e-12                # shunt C or tank C below this is removed
C_SHORT = C_RANGE[1] * 0.999   # series C at the upper bound is shorted


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
    stop_f: tuple = ()         # stopband grid (both sides), Hz
    stop_floor: tuple = ()     # floor at each stop_f, dB


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
    pruned: tuple = ()         # what pruning removed from the topology, in order


def stop_grids(lo: float, hi: float, oversample: int = 1) -> tuple[np.ndarray, np.ndarray]:
    """Low-side (N_STOP_LO points) and high-side (N_STOP_HI points) stopband grids (Hz),
    log-spaced; `oversample` puts that many intervals in each one (the original points stay)."""
    f_hi_end = STOP_HI_END if 2 * hi < STOP_HI_END else 4 * hi
    return (np.geomspace(max(STOP_LO_START, lo / 20), lo / 2, (N_STOP_LO - 1) * oversample + 1),
            np.geomspace(2 * hi, f_hi_end, (N_STOP_HI - 1) * oversample + 1))


def floor_db(today_db):
    """Selectivity floor from today's attenuation: min(max(20, today − 3), 60)."""
    return np.minimum(np.maximum(FLOOR_MIN_DB, np.asarray(today_db, dtype=float) - FLOOR_BACKOFF_DB), FLOOR_CAP_DB)


def _model(lo, hi):
    return ladder.robust_losses(math.sqrt(lo * hi)), ladder.bank_bus()


def today_attenuation(index: int, f) -> np.ndarray:
    """Attenuation (dB) of today's Chebyshev design (bpf.design) in the robust model with the
    bank bus at frequencies f."""
    from neohw.blocks import bpf
    d = bpf.design(index)
    losses, bus = _model(d.lo_hz, d.hi_hz)
    return -ladder.db(ladder.s21_s11(ladder.from_positions(d.positions), np.asarray(f, dtype=float), losses, bus)[0])


def today_selectivity(index: int) -> dict:
    """The selectivity floor curve from today's design on the stopband grids made STOP_OVERSAMPLE
    times finer (a floor held only at the 60/30 points let elliptic stopbands and the floor's
    corners dip up to 0.6 dB below it in between): keys f_lo_hz, floor_lo_db, f_hi_hz,
    floor_hi_db (tuples). The only path from targets() to today's design; Task 6 replaces
    bpf.design and this then reads the values recorded in the designs file."""
    from neohw.blocks import bpf
    lo, hi = bpf.BANDS[index - 1]
    f_lo, f_hi = stop_grids(lo, hi, STOP_OVERSAMPLE)
    return {"f_lo_hz": tuple(f_lo), "floor_lo_db": tuple(floor_db(today_attenuation(index, f_lo))),
            "f_hi_hz": tuple(f_hi), "floor_hi_db": tuple(floor_db(today_attenuation(index, f_hi)))}


def targets(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Targets:
    """Block targets with margin (rl_min, rej_min) and the stopband floor; att_half_lo_min and
    att_2hi_min are the floor at lo/2 and 2·hi (the grid ends)."""
    from neohw.blocks import bpf
    from neohw.checks import bpf as check
    lo, hi = bpf.BANDS[index - 1]
    s = today_selectivity(index)
    return Targets(index, lo, hi, check.il_limit_db(index), rl_min, rej_min if 3 * lo > hi else None,
                   float(s["floor_lo_db"][-1]), float(s["floor_hi_db"][0]),
                   tuple(s["f_lo_hz"]) + tuple(s["f_hi_hz"]), tuple(s["floor_lo_db"]) + tuple(s["floor_hi_db"]))


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


def stop_margin_db(branches, t: Targets) -> float:
    """Minimum over the stopband grids of attenuation − floor (dB)."""
    losses, bus = _model(t.lo, t.hi)
    att = -ladder.db(ladder.s21_s11(list(branches), np.array(t.stop_f), losses, bus)[0])
    return float(np.min(att - np.array(t.stop_floor)))


def _metrics(branches, t: Targets) -> dict:
    losses, bus = _model(t.lo, t.hi)
    m = ladder.metrics(list(branches), t.lo, t.hi, losses, bus)
    m["stop_margin_db"] = stop_margin_db(branches, t) if t.stop_f else None
    return m


def _notches_ok(branches, hi) -> bool:
    return all(1 / (2 * math.pi * math.sqrt(b.l * b.c)) >= NOTCH_MIN * hi * (1 - 1e-9)
               for b in branches if b.form == "LC_par")


def _violations(m, t: Targets, branches=()) -> dict:
    v = {}
    if m["il_db"] > t.il_max:
        v["il_db"] = m["il_db"] - t.il_max
    if m["rl_db"] < t.rl_min:
        v["rl_db"] = t.rl_min - m["rl_db"]
    if t.rej_min is not None and m["rej3f_db"] < t.rej_min:
        v["rej3f_db"] = t.rej_min - m["rej3f_db"]
    if m["att_2hi_db"] < t.att_2hi_min - SEL_TOL_DB:
        v["att_2hi_db"] = t.att_2hi_min - m["att_2hi_db"]
    if t.att_half_lo_min is not None and m["att_half_lo_db"] is not None \
            and m["att_half_lo_db"] < t.att_half_lo_min - SEL_TOL_DB:
        v["att_half_lo_db"] = t.att_half_lo_min - m["att_half_lo_db"]
    if m.get("stop_margin_db") is not None and m["stop_margin_db"] < -SEL_TOL_DB:
        v["stop_margin_db"] = -m["stop_margin_db"]
    bad = [1.0 - 1 / (2 * math.pi * math.sqrt(b.l * b.c)) / (NOTCH_MIN * t.hi)
           for b in branches if b.form == "LC_par"]
    if bad and max(bad) > 1e-9:
        v["notch_fz"] = max(bad)
    return v


def _rank(r: Result):
    """Feasible before infeasible; then lower IL, or smaller total violation."""
    return (0, r.metrics["il_db"]) if r.feasible else (1, sum(r.violations.values()))


def _solve(t: Targets, template: list, max_iter: int) -> list:
    """One SLSQP run from the template's values; returns the solved branches."""
    x0, slots = _pack(template)
    losses, bus = _model(t.lo, t.hi)
    band = np.linspace(t.lo, t.hi, N_BAND)
    rej = np.linspace(3 * t.lo, 3 * t.hi, N_REJ) if t.rej_min is not None else np.array([])
    stop = np.concatenate([rej, t.stop_f])
    stop_min = np.concatenate([np.full(len(rej), t.rej_min if t.rej_min is not None else 0.0), t.stop_floor])
    lb = [math.log(L_RANGE[0]) if w == "l" else math.log(C_RANGE[0]) for _, w in slots]
    ub = [math.log(L_RANGE[1]) if w == "l" else math.log(C_RANGE[1]) for _, w in slots]
    x0 = np.clip(x0, lb, ub)
    bounds = list(zip(lb, ub)) + [(0.0, 60.0)]
    # tank zeros: log fz = −(log L + log C)/2 − log 2π ≥ log(1.3·hi), linear in x
    tank = [(slots.index((k, "l")), slots.index((k, "c"))) for k, b in enumerate(template) if b.form == "LC_par"]
    log_fz_min = math.log(NOTCH_MIN * t.hi * 2 * math.pi)

    def cons(x):
        br = _unpack(x[:-1], slots, template)
        s21, s11 = ladder.s21_s11(br, band, losses, bus)
        att = -ladder.db(ladder.s21_s11(br, stop, losses, bus)[0])
        return np.concatenate([x[-1] + ladder.db(s21), -ladder.db(s11) - t.rl_min - SOLVE_MARGIN_DB,
                               att - stop_min - SOLVE_MARGIN_DB])

    constraints = [{"type": "ineq", "fun": cons}]
    if tank:
        a = np.zeros((len(tank), len(x0) + 1))
        for row, (il, ic) in enumerate(tank):
            a[row, il] = a[row, ic] = -0.5
        constraints.append({"type": "ineq", "fun": lambda x: a @ x - log_fz_min, "jac": lambda x: a})
    il0 = float((-ladder.db(ladder.s21_s11(_unpack(x0, slots, template), band, losses, bus)[0])).max())
    res = minimize(lambda x: x[-1], np.append(x0, min(il0, 60.0)), method="SLSQP", bounds=bounds,
                   jac=lambda x: np.eye(len(x))[-1], constraints=constraints,
                   options={"maxiter": max_iter, "ftol": 1e-6})
    return _unpack(res.x[:-1], slots, template)


def _prune(branches) -> tuple[list, list]:
    """Removes shunt Cs below C_PRUNE, shorts series Cs at the upper bound and turns tanks
    with C below C_PRUNE into plain inductors. Returns the new branches and what was done."""
    out, done = [], []
    for k, b in enumerate(branches):
        if b.place == "shunt" and b.form == "C" and b.c < C_PRUNE:
            done.append(f"shunt C at {k} ({b.c * 1e12:.2g} pF) removed")
        elif b.place == "series" and b.form == "C" and b.c >= C_SHORT:
            done.append(f"series C at {k} ({b.c * 1e9:.3g} nF) shorted")
        elif b.form == "LC_par" and b.c < C_PRUNE:
            done.append(f"tank at {k} ({b.c * 1e12:.2g} pF) to plain L")
            out.append(ladder.Branch(b.place, "L", l=b.l))
        else:
            out.append(b)
    return out, done


def _solve_pruned(top: Topology, t: Targets, template: list, max_iter: int) -> Result:
    """Solve, prune, and solve the reduced ladder again from its current values while anything
    is pruned."""
    pruned = []
    br = _solve(t, template, max_iter)
    while True:
        br, done = _prune(br)
        if not done:
            break
        pruned += done
        br = _solve(t, br, max_iter)
    m = _metrics(br, t)
    v = _violations(m, t, br)
    return Result(top, tuple(br), m, not v, v, tuple(pruned))


def optimise(top: Topology, t: Targets, max_iter: int = MAX_ITER, starts: int = STARTS) -> Result:
    """The best of up to `starts` runs (each solved and pruned): the seed first, then seeds with
    every element value scaled by a random factor in 1 ± PERTURB (fixed generator, so results
    repeat). Stops early once a run is feasible and the next start does not improve IL by more
    than 0.01 dB."""
    template = seed(top, t.lo, t.hi)
    x_seed, slots = _pack(template)
    rng = np.random.default_rng(1000 * t.index + 10 * top.hpf_order + int(top.lpf[-1]))
    best_r = None
    for k in range(starts):
        x0 = x_seed if k == 0 else x_seed + np.log(1 + rng.uniform(-PERTURB, PERTURB, len(x_seed)))
        r = _solve_pruned(top, t, _unpack(x0, slots, template), max_iter)
        if best_r is not None and best_r.feasible and r.feasible and r.metrics["il_db"] > best_r.metrics["il_db"] - 0.01:
            return best_r if _rank(best_r) <= _rank(r) else r
        if best_r is None or _rank(r) < _rank(best_r):
            best_r = r
    return best_r


def _optimise_args(args):
    return optimise(*args)


@lru_cache(maxsize=None)
def best(index: int, rl_min: float = 15.5, rej_min: float = 53.0) -> Result:
    """The lowest-IL feasible result over TOPOLOGIES (optimised in parallel), or the infeasible
    one with the smallest total violation. Cached per arguments: callers must not mutate the
    result's dicts."""
    t = targets(index, rl_min, rej_min)
    with ProcessPoolExecutor(max_workers=min(len(TOPOLOGIES), os.cpu_count() or 1)) as pool:
        results = list(pool.map(_optimise_args, [(top, t) for top in TOPOLOGIES]))
    return min(results, key=_rank)
