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
from dataclasses import dataclass, field, replace
from functools import lru_cache

import numpy as np
from scipy.optimize import minimize
from scipy.special import expit

from neohw import eseries, inductors
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

# Realisation and yield
FLOOR_MARGIN_DB = 0.5          # designs aim this far above the stopband floor so snapping lands on it
SNAP_TOL = 0.02                # one or two chokes within 2 % of the design value
SNAP_MAX = 0.05                # else the closest reachable value within 5 % (the C re-optimisation absorbs it)
N_PENALTY = 50                 # fixed draws per population in the optimiser's yield penalty
PENALTY_SEED = 5
YIELD_WEIGHT_DB = 20.0         # objective = worst IL + this × the (smoothed) fraction of failing draws
YIELD_SOFT_DB = 0.25           # a draw's failure is expit(−margin / this), margin in dB to the real targets
CAP_SERIES = {"china": eseries.E24, "iran": eseries.E12}
CAP_TOL = {"china": 0.01, "iran": 0.02}
N_DENSE = 1001                 # realised designs hold the floor on 1001-point log grids per side
YIELD_GATE_PCT = 97.0          # optimiser gate (the block target is 95 % of 200 builds)
MAX_RETRIES = 3                # each raises rl_min 1 dB, rej_min 2 dB and the floor margin 0.5 dB
RETRY_STEP = (1.0, 2.0, 0.5)


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
    floor_margin: float = 0.0  # the solver holds stop_floor + floor_margin


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


def _models(lo, hi):
    """The two SRF cases every metric is judged on (worst of both): each choke's parallel C at its
    datasheet SRF minimum, and no parallel C (SRF far above), so no design can rely on a
    self-resonance notch."""
    f0 = math.sqrt(lo * hi)
    return (ladder.robust_losses(f0, True), ladder.robust_losses(f0, False)), ladder.bank_bus()


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


def targets(index: int, rl_min: float = 15.5, rej_min: float = 53.0, floor_margin: float = 0.0) -> Targets:
    """Block targets with margin (rl_min, rej_min, floor_margin above the stopband floor) and the
    floor; att_half_lo_min and att_2hi_min are the floor at lo/2 and 2·hi (the grid ends)."""
    from neohw.blocks import bpf
    from neohw.checks import bpf as check
    lo, hi = bpf.BANDS[index - 1]
    s = today_selectivity(index)
    return Targets(index, lo, hi, check.il_limit_db(index), rl_min, rej_min if 3 * lo > hi else None,
                   float(s["floor_lo_db"][-1]), float(s["floor_hi_db"][0]),
                   tuple(s["f_lo_hz"]) + tuple(s["f_hi_hz"]), tuple(s["floor_lo_db"]) + tuple(s["floor_hi_db"]),
                   floor_margin)


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


def _pack(branches, free=("l", "c")):
    """Log values of the free slots ("l" and/or "c") and their (branch, slot) keys."""
    x, slots = [], []
    for k, b in enumerate(branches):
        if b.form != "C" and "l" in free:
            x.append(math.log(b.l)); slots.append((k, "l"))
        if b.form != "L" and "c" in free:
            x.append(math.log(b.c)); slots.append((k, "c"))
    return np.array(x), slots


def _unpack(x, slots, template):
    vals = {k: {"l": b.l, "c": b.c} for k, b in enumerate(template)}
    for v, (k, which) in zip(x, slots):
        vals[k][which] = math.exp(v)
    return [replace(b, l=vals[k]["l"], c=vals[k]["c"]) for k, b in enumerate(template)]


def _worst_att_db(branches, f, t: Targets) -> np.ndarray:
    models, bus = _models(t.lo, t.hi)
    return np.min([-ladder.db(ladder.s21_s11(list(branches), np.asarray(f), m, bus)[0]) for m in models], axis=0)


def stop_margin_db(branches, t: Targets) -> float:
    """Minimum over the stopband grids of attenuation − floor (dB), worst of both SRF cases."""
    return float(np.min(_worst_att_db(branches, t.stop_f, t) - np.array(t.stop_floor)))


def _metrics(branches, t: Targets) -> dict:
    """ladder.metrics, worst of both SRF cases (_models), plus stop_margin_db."""
    models, bus = _models(t.lo, t.hi)
    ms = [ladder.metrics(list(branches), t.lo, t.hi, losses, bus) for losses in models]
    m = {k: (max if k == "il_db" else min)(x[k] for x in ms) if ms[0][k] is not None else None for k in ms[0]}
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
    if m.get("stop_margin_db") is not None and m["stop_margin_db"] < t.floor_margin - SEL_TOL_DB:
        v["stop_margin_db"] = t.floor_margin - m["stop_margin_db"]
    bad = [1.0 - 1 / (2 * math.pi * math.sqrt(b.l * b.c)) / (NOTCH_MIN * t.hi)
           for b in branches if b.form == "LC_par"]
    if bad and max(bad) > 1e-9:
        v["notch_fz"] = max(bad)
    return v


def _rank(r: Result):
    """Feasible before infeasible; then lower IL, or smaller total violation."""
    return (0, r.metrics["il_db"]) if r.feasible else (1, sum(r.violations.values()))


def _tank_rows(template, slots, hi):
    """Tank zeros as linear constraints a @ x + b ≥ 0 on the log values (the epigraph variable
    last): log fz = −(log L + log C)/2 − log 2π ≥ log(1.3·hi). Fixed slots go into b."""
    tanks = [k for k, br in enumerate(template) if br.form == "LC_par"]
    a, b = np.zeros((len(tanks), len(slots) + 1)), np.full(len(tanks), -math.log(NOTCH_MIN * hi * 2 * math.pi))
    for row, k in enumerate(tanks):
        for which in ("l", "c"):
            if (k, which) in slots:
                a[row, slots.index((k, which))] = -0.5
            else:
                b[row] -= 0.5 * math.log(getattr(template[k], which))
    return a, b


def _cons(t: Targets, template: list, slots):
    """Nominal constraints ≥ 0 in both SRF cases: in-band IL ≤ the epigraph variable, RL, 3f
    rejection and the stopband floor plus its margin."""
    models, bus = _models(t.lo, t.hi)
    band = np.linspace(t.lo, t.hi, N_BAND)
    rej = np.linspace(3 * t.lo, 3 * t.hi, N_REJ) if t.rej_min is not None else np.array([])
    stop = np.concatenate([rej, t.stop_f])
    stop_min = np.concatenate([np.full(len(rej), t.rej_min if t.rej_min is not None else 0.0),
                               np.array(t.stop_floor) + t.floor_margin]) + SOLVE_MARGIN_DB

    def cons(x):
        br, out = _unpack(x[:-1], slots, template), []
        for losses in models:
            s21, s11 = ladder.s21_s11(br, band, losses, bus)
            att = -ladder.db(ladder.s21_s11(br, stop, losses, bus)[0])
            out += [x[-1] + ladder.db(s21), -ladder.db(s11) - t.rl_min - SOLVE_MARGIN_DB, att - stop_min]
        return np.concatenate(out)
    return cons


def _solve(t: Targets, template: list, max_iter: int, free=("l", "c"), c_min: float = C_RANGE[0],
           pops=("china", "iran")) -> list:
    """One SLSQP run from the template's values over the `free` slots ("l" and/or "c"; the
    others keep their values), C bounded below by c_min. Minimises the worst in-band IL plus
    the yield penalty over fixed draws in `pops` (spec §4); returns the solved branches."""
    x0, slots = _pack(template, free)
    lb = [math.log(L_RANGE[0]) if w == "l" else math.log(c_min) for _, w in slots]
    ub = [math.log(L_RANGE[1]) if w == "l" else math.log(C_RANGE[1]) for _, w in slots]
    x0 = np.clip(x0, lb, ub)
    a, b0 = _tank_rows(template, slots, t.hi)
    constraints = [{"type": "ineq", "fun": _cons(t, template, slots)}]
    if len(b0):
        constraints.append({"type": "ineq", "fun": lambda x: a @ x + b0, "jac": lambda x: a})
    draws = _penalty_draws(template, pops)

    def objective(x):
        return x[-1] + YIELD_WEIGHT_DB * _fail_fraction(_unpack(x[:-1], slots, template), t, draws)

    il0 = _metrics(_unpack(x0, slots, template), t)["il_db"]
    res = minimize(objective, np.append(x0, min(il0, 60.0)), method="SLSQP",
                   bounds=list(zip(lb, ub)) + [(0.0, 60.0)], constraints=constraints,
                   options={"maxiter": max_iter, "ftol": 1e-6})
    return _unpack(res.x[:-1], slots, template)


def _scale(branches, population: str, rng, n: int, caps: dict | None = None) -> list:
    """n drawn builds as per-branch (kl, kc) factors for ladder.s21_s11(scale=…): every choke and
    every capacitor part uniform within ±its tolerance on its own (as checks.bpf._variation):
    chokes inductors.tolerance of their row (the nearest row for a design value), capacitors
    CAP_TOL; caps gives a branch's capacitor parts (default one part of b.c)."""
    out = []
    for k, b in enumerate(branches):
        kl = kc = None
        if b.form != "C":
            rows = ([inductors.row(inductors.filter_series(nb), nb) for nb, _ in b.parts] if b.parts
                    else [inductors.nearest_row(inductors.filter_series(b.l), b.l)])
            tol = np.array([inductors.tolerance(r, population) for r in rows])
            kl = 1 + rng.uniform(-1, 1, (n, len(rows))) * tol
        if b.form != "L":
            cs = np.array((caps or {}).get(k, (b.c,)))
            kc = (1 + rng.uniform(-CAP_TOL[population], CAP_TOL[population], (n, len(cs)))) @ cs / cs.sum()
        out.append((kl, kc))
    return out


def _mc(branches, t: Targets, scale) -> tuple:
    """Per-draw worst in-band IL, RL and 3f rejection (None without a rejection target), each the
    worst of both SRF cases."""
    models, bus = _models(t.lo, t.hi)
    band = np.linspace(t.lo, t.hi, N_BAND)
    il, rl, rej = [], [], []
    for losses in models:
        s21, s11 = ladder.s21_s11(list(branches), band, losses, bus, scale=scale)
        il.append((-ladder.db(s21)).max(axis=-1)); rl.append((-ladder.db(s11)).min(axis=-1))
        if t.rej_min is not None:
            f3 = np.linspace(3 * t.lo, 3 * t.hi, N_REJ)
            rej.append((-ladder.db(ladder.s21_s11(list(branches), f3, losses, bus, scale=scale)[0])).min(axis=-1))
    return np.max(il, axis=0), np.min(rl, axis=0), (np.min(rej, axis=0) if rej else None)


def _penalty_draws(template, pops) -> list:
    """Fixed draws (common random numbers across iterations) for the yield penalty."""
    return [_scale(template, pop, np.random.default_rng([PENALTY_SEED, k]), N_PENALTY) for k, pop in enumerate(pops)]


def _fail_fraction(branches, t: Targets, draws) -> float:
    """Smoothed fraction of draws missing the real IL, RL or rejection target, summed over the
    populations: a draw counts expit(−margin / YIELD_SOFT_DB), margin its worst dB to a target."""
    from neohw.checks import bpf as check
    total = 0.0
    for scale in draws:
        il, rl, rej = _mc(branches, t, scale)
        margin = np.minimum(t.il_max - il, rl - check.RL_MIN_DB)
        if rej is not None:
            margin = np.minimum(margin, rej - check.REJ_MIN_DB)
        total += float(np.mean(expit(-margin / YIELD_SOFT_DB)))
    return total


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
            out.append(replace(b, form="L", c=0.0))
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
def best(index: int, rl_min: float = 15.5, rej_min: float = 53.0, floor_margin: float = 0.0) -> Result:
    """The lowest-IL feasible result over TOPOLOGIES (optimised in parallel), or the infeasible
    one with the smallest total violation. Cached per arguments: callers must not mutate the
    result's dicts."""
    t = targets(index, rl_min, rej_min, floor_margin)
    with ProcessPoolExecutor(max_workers=min(len(TOPOLOGIES), os.cpu_count() or 1)) as pool:
        results = list(pool.map(_optimise_args, [(top, t) for top in TOPOLOGIES]))
    return min(results, key=_rank)


# Realisation on buyable parts and the yield loop


@lru_cache(maxsize=None)
def _chokes() -> tuple[float, ...]:
    """Every buyable filter choke value: each from its own series (filter_series)."""
    return tuple(sorted(h for s in (inductors.FILTER_SMALL, inductors.FILTER_LARGE)
                        for h in inductors.available(s) if inductors.filter_series(h) == s))


@lru_cache(maxsize=None)
def snap_l(target: float) -> tuple[float, ...]:
    """One choke if one is within SNAP_TOL of `target`, else the closest pair in series (also
    within SNAP_TOL), larger first; else the closer of the two if within SNAP_MAX. Raises
    ValueError beyond that."""
    parts = _chokes()
    err = lambda v: abs(v / target - 1)
    one = min(parts, key=err)
    if err(one) <= SNAP_TOL:
        return (one,)
    pair = min(((a, b) for i, a in enumerate(parts) for b in parts[:i + 1]), key=lambda p: err(sum(p)))
    if err(sum(pair)) <= SNAP_TOL:
        return pair
    closest = (one,) if err(one) <= err(sum(pair)) else pair
    if err(sum(closest)) > SNAP_MAX:
        raise ValueError(f"cannot realise {target * 1e6:.4g} µH with one or two chokes within "
                         f"{SNAP_MAX:.0%} (closest {sum(closest) * 1e6:.4g} µH)")
    return closest


@dataclass(frozen=True)
class Realised:
    branches_china: tuple
    branches_iran: tuple
    l_parts: dict              # branch index → bought choke values (henries), same in both populations
    c_china: dict              # branch index → E24 capacitor values (farads) in parallel
    c_iran: dict               # branch index → E12 capacitor values
    topology: Topology | None
    metrics: dict              # population → nominal metrics; stop_margin_db on the dense grids
    yield_pct: dict = field(default_factory=dict)
    margins: dict = field(default_factory=dict)
    l_deviation: dict = field(default_factory=dict)   # branch index → sum of bought chokes / design value − 1


class DesignError(RuntimeError):
    def __init__(self, message: str, metrics: dict, yield_pct: dict, margins: dict):
        super().__init__(message)
        self.metrics, self.yield_pct, self.margins = metrics, yield_pct, margins


@lru_cache(maxsize=None)
def _dense_grid(index: int, n: int) -> tuple[np.ndarray, np.ndarray]:
    from neohw.blocks import bpf
    lo, hi = bpf.BANDS[index - 1]
    f = np.concatenate([np.geomspace(g[0], g[-1], n) for g in stop_grids(lo, hi)])
    return f, floor_db(today_attenuation(index, f))


def dense_stop_margin_db(branches, t: Targets, n: int = N_DENSE) -> float:
    """Minimum of attenuation − floor on n-point log grids over each stopband side, the floor
    computed from today's design at every point (stricter than the solver's grids); worst of
    both SRF cases."""
    f, fl = _dense_grid(t.index, n)
    return float(np.min(_worst_att_db(branches, f, t) - fl))


def _fit_chokes(branches, l_parts: dict, population: str) -> list:
    """China fits the bought chokes (L = their sum); Iran selects them to the design value
    (each scaled by the same factor, so L = the design value)."""
    out = []
    for k, b in enumerate(branches):
        if k in l_parts:
            parts = l_parts[k]
            scale = 1.0 if population == "china" else b.l / sum(parts)
            fitted = tuple((p, p * scale) for p in parts)
            b = replace(b, l=sum(a for _, a in fitted), parts=fitted)
        out.append(b)
    return out


def _realise_population(branches, l_parts: dict, t: Targets, population: str) -> tuple[list, dict]:
    """Inductors fixed, the capacitors re-optimised (continuous, every constraint kept, C ≥ 3 pF
    so the structure stays that of `branches`) and snapped to one or two preferred values."""
    from neohw.blocks.bpf import C_MIN
    br = _solve(t, _fit_chokes(branches, l_parts, population), MAX_ITER, free=("c",), c_min=C_PRUNE,
                pops=(population,))
    caps = {k: eseries.combo(b.c, CAP_SERIES[population], minimum=C_MIN) for k, b in enumerate(br) if b.form != "L"}
    return [replace(b, c=sum(caps[k])) if k in caps else b for k, b in enumerate(br)], caps


def realise(result: Result, t: Targets) -> Realised:
    """`result` on buyable parts in both populations (same chokes), with each population's
    nominal metrics; the stop margin is judged on the dense grids against the true floor."""
    l_parts = {k: snap_l(b.l) for k, b in enumerate(result.branches) if b.form != "C"}
    china, c_china = _realise_population(result.branches, l_parts, t, "china")
    iran, c_iran = _realise_population(result.branches, l_parts, t, "iran")
    metrics = {pop: {**_metrics(br, t), "stop_margin_db": dense_stop_margin_db(br, t)}
               for pop, br in (("china", china), ("iran", iran))}
    return Realised(tuple(china), tuple(iran), l_parts, c_china, c_iran, result.topology, metrics,
                    margins={"rl_min": t.rl_min, "rej_min": t.rej_min, "floor_margin_db": t.floor_margin},
                    l_deviation={k: sum(p) / result.branches[k].l - 1 for k, p in l_parts.items()})


def fast_yield(realised: Realised, t: Targets, population: str, runs: int = 200, seed: int = 0) -> float:
    """Percentage of `runs` drawn builds (_scale) meeting the real block targets (IL t.il_max, RL
    and 3f rejection of checks.bpf) in both SRF cases, whatever margins `t` carries."""
    from neohw.checks import bpf as check
    br = getattr(realised, f"branches_{population}")
    il, rl, rej = _mc(br, t, _scale(br, population, np.random.default_rng(seed), runs,
                                    getattr(realised, f"c_{population}")))
    ok = (il <= t.il_max) & (rl >= check.RL_MIN_DB)
    if rej is not None:
        ok &= rej >= check.REJ_MIN_DB
    return 100.0 * float(np.mean(ok))


def design(index: int) -> Realised:
    """best, realise and fast_yield in both populations. While a yield is below YIELD_GATE_PCT or
    a realised nominal misses the floor, retries (at most MAX_RETRIES) with rl_min +1 dB,
    rej_min +2 dB and the floor margin +0.5 dB; then raises DesignError with the numbers."""
    rl, rej, fm = 15.5, 53.0, FLOOR_MARGIN_DB
    for attempt in range(MAX_RETRIES + 1):
        real = realise(best(index, rl, rej, fm), targets(index, rl, rej, fm))
        yields = {pop: fast_yield(real, targets(index), pop) for pop in ("china", "iran")}
        margins = {"rl_min": rl, "rej_min": rej, "floor_margin_db": fm, "retries": attempt}
        if all(y >= YIELD_GATE_PCT for y in yields.values()) and \
                all(m["stop_margin_db"] >= 0 for m in real.metrics.values()):
            return replace(real, yield_pct=yields, margins=margins)
        rl, rej, fm = rl + RETRY_STEP[0], rej + RETRY_STEP[1], fm + RETRY_STEP[2]
    raise DesignError(f"filter {index}: yield {yields} (gate {YIELD_GATE_PCT:g} %), nominal {real.metrics}, "
                      f"margins {margins}", real.metrics, yields, margins)
