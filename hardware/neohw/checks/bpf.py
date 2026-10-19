"""BPF bank check (spec §4): per-filter IL, RL, 3f rejection and Monte Carlo in both populations
(China, Iran); muted state; s2p files (China, the population the twin reads)."""
from __future__ import annotations

import math

from pathlib import Path

import numpy as np

from neohw import availability, charfile, export, population
from neohw.blocks import bpf, bpf_bank
from neohw.checks import BlockResult, CheckResult, db
from neohw.ngspice import run_decks
from neohw.spice import Port, Sp, to_deck
from neohw.touchstone import write_s2p
from neohw.tools import git_describe
from neohw.units import parse_si

MC_RUNS = 200
MC_MIN_PCT = 95.0
RL_MIN_DB = 14.0
REJ_MIN_DB = 50.0
MUTE_MIN_DB = 40.0
S2P_SWEEP = Sp(0.05e6, 210e6, 50, "dec")  # decade grid: last point 208 MHz, spec asks for 200
SELECTION_RULE = "filter n with lo_n <= f < hi_n; f = 54 MHz -> 11"


# System-based limit (user decision 2026-10-19, option A): where no inductor with guaranteed Q
# reaches 3 dB, the filter's loss is judged by what it does to reception: the noise floor may
# rise by at most NOISE_RISE_MAX_DB over an ideal receiver in ITU-R P.372 residential man-made
# noise, and IL is capped at IL_SYSTEM_CAP_DB.
SYSTEM_LIMITED = {1: "0.1–0.5 MHz: no chip inductor with guaranteed Q reaches 3 dB (China NL453232: 5.6 dB)"}
IL_SYSTEM_CAP_DB = 8.0
NOISE_RISE_MAX_DB = 1.0
P372_RESIDENTIAL = (72.5, 27.7)   # Fa = c − d·log10(f / MHz) dB above kT0 (ITU-R P.372, man-made, residential)
REST_NF_DB = 7.3                  # what follows the filter, preamp bypassed: the mixer's SSB NF (mixer.json 7.24 dB)


def il_limit_db(index: int) -> float:
    """Spec §2 amendment: 3 dB on filters 1–7 (preamp bypassed), 2 dB on 8–11; the system-limited
    filters have IL_SYSTEM_CAP_DB plus the noise-floor check."""
    if index in SYSTEM_LIMITED:
        return IL_SYSTEM_CAP_DB
    return 3.0 if index <= 7 else 2.0


def noise_floor_rise_db(il_db: float, f_hz: float, rest_nf_db: float = REST_NF_DB) -> float:
    """Rise of the noise floor over an ideal (noiseless) receiver when the band noise at the
    antenna is P.372 residential: the filter is a passive loss at T0, so F = L·F_rest."""
    c, d = P372_RESIDENTIAL
    fa = 10 ** ((c - d * math.log10(f_hz / 1e6)) / 10)
    f_rx = 10 ** ((il_db + rest_nf_db) / 10)
    return 10 * math.log10(1 + (f_rx - 1) / fa)


def has_rejection_target(index: int) -> bool:
    """For filters 1–2, 3x the lower edge lies inside the passband."""
    lo, hi = bpf.BANDS[index - 1]
    return 3 * lo > hi


def _sweeps(index: int) -> list[Sp]:
    lo, hi = bpf.BANDS[index - 1]
    return [Sp(lo, hi, 101, "lin"), Sp(3 * lo, 3 * hi, 61, "lin")]


def _metrics(plots) -> tuple[float, float, float]:
    """(worst in-band IL, worst in-band RL, worst rejection over 3x band), all in dB."""
    band, third = plots[0].data, plots[1].data
    return (float(-db(band["s_2_1"]).min()), float(-db(band["s_1_1"]).max()),
            float(-db(third["s_2_1"]).max()))


def _passes(index: int, il: float, rl: float, rej: float) -> bool:
    return (il <= il_limit_db(index) and rl >= RL_MIN_DB
            and (not has_rejection_target(index) or rej >= REJ_MIN_DB))


def _variation(circuit, rng, pop: str = population.DEFAULT) -> dict[str, float]:
    return {p.ref: 1 + rng.uniform(-population.tol(p, pop), population.tol(p, pop))
            for p in sorted(circuit.parts, key=lambda p: p.ref)
            if population.fitted(p, pop) and population.tol(p, pop) > 0}


def _ports(blk) -> list[Port]:
    return [Port(blk.ports["rf_in"], 1), Port(blk.ports["rf_out"], 2)]


def evaluate(index: int, mc_runs: int, population: str = population.DEFAULT) -> dict:
    """Nominal metrics, Monte Carlo pass count and the 0.05–200 MHz S-parameters of one filter
    selected in the full bank, as fitted in `population`. The worst-case run evaluates every
    inductor's datasheet loss (neohw.inductors) at the upper band edge instead of the centre."""
    blk = bpf_bank.build(index)
    sweeps = _sweeps(index)
    decks = [to_deck(blk.circuit, f"bpf{index} {population} nominal", _ports(blk), sweeps, population=population)]
    for run in range(mc_runs):
        rng = np.random.default_rng([index, run])
        decks.append(to_deck(blk.circuit, f"bpf{index} {population} mc{run}", _ports(blk), sweeps,
                             variation=_variation(blk.circuit, rng, population), population=population))
    worst = bpf_bank.build(index, q_ref="upper")
    decks.append(to_deck(worst.circuit, f"bpf{index} {population} worst-case Q", _ports(worst), sweeps,
                         population=population))
    s2p_deck, s2p_raws = to_deck(blk.circuit, f"bpf{index} {population} s2p", _ports(blk), [S2P_SWEEP],
                                 population=population)
    raws = decks[0][1]
    results = run_decks([d for d, _ in decks], raws)
    (s2p_plot,) = run_decks([s2p_deck], s2p_raws)[0][s2p_raws[0]]
    nominal = _metrics([results[0][r][0] for r in raws])
    mc = [_metrics([res[r][0] for r in raws]) for res in results[1:-1]]
    il_worst = _metrics([results[-1][r][0] for r in raws])[0]
    return {"nominal": nominal, "mc_runs": mc_runs, "il_worst_q_db": il_worst,
            "mc_passed": sum(1 for m in mc if _passes(index, *m)), "s2p": s2p_plot.data}


def mute_isolation_db() -> float:
    """Worst S21 over 0.1–54 MHz with every relay released (the safe state)."""
    blk = bpf_bank.build(None)
    deck, raws = to_deck(blk.circuit, "bank muted", _ports(blk), [Sp(0.1e6, 54e6, 50, "dec")])
    (plot,) = run_decks([deck], raws)[0][raws[0]]
    return float(-db(plot.data["s_2_1"]).max())


def build_sheet(selected: int | None = None) -> list[dict]:
    """Inductor groups of the bank as built (refs match the netlist and BOM): what to buy and
    what the group must measure after selection."""
    circuit = bpf_bank.build(selected).circuit
    groups: dict[str, list] = {}
    for p in circuit.parts:
        if hasattr(p, "select_group"):
            groups.setdefault(p.select_group, []).append(p)
    sheet = []
    for name, members in groups.items():
        members.sort(key=lambda p: int(p.ref[1:]))
        sheet.append({"filter": int(name[1:].split("_")[0]), "group": name,
                      "refs": [p.ref for p in members], "buy_h": [parse_si(p.value[:-1]) for p in members],
                      "target_h": members[0].select_target, "select": "sum within ±3 % of target_h"})
    return sorted(sheet, key=lambda e: int(e["refs"][0][1:]))


def _filter_checks(index: int, r: dict, mc_runs: int, prefix: str) -> list[CheckResult]:
    """IL, RL, 3f rejection (where it applies) and Monte Carlo of one filter in one population;
    `prefix` is "" for China, "iran_" for Iran."""
    il, rl, rej = r["nominal"]
    pct = 100.0 * r["mc_passed"] / mc_runs if mc_runs else 0.0
    name = f"bpf{index}_{prefix}"
    out = [CheckResult(f"{name}il_db", il, f"≤ {il_limit_db(index):g}", il <= il_limit_db(index)),
           CheckResult(f"{name}rl_db", rl, f"≥ {RL_MIN_DB:g}", rl >= RL_MIN_DB)]
    if has_rejection_target(index):
        out.append(CheckResult(f"{name}rej3f_db", rej, f"≥ {REJ_MIN_DB:g}", rej >= REJ_MIN_DB))
    if index in SYSTEM_LIMITED:   # the band's top edge has the least man-made noise
        rise = noise_floor_rise_db(r["il_worst_q_db"], bpf.BANDS[index - 1][1])
        out.append(CheckResult(f"{name}noise_rise_db", rise, f"≤ {NOISE_RISE_MAX_DB:g}",
                               rise <= NOISE_RISE_MAX_DB, "dB"))
    out.append(CheckResult(f"{name}mc_pass_pct", pct, f"≥ {MC_MIN_PCT:g}", pct >= MC_MIN_PCT, "%"))
    return out


def run(out_dir: Path, char_dir: Path, mc_runs: int = MC_RUNS, filters=None) -> BlockResult:
    out_dir, char_dir = Path(out_dir), Path(char_dir)
    indices = list(filters or range(1, len(bpf.BANDS) + 1))
    checks, plots, entries = [], {}, []
    revision = git_describe()
    sheet = build_sheet()
    for index in indices:
        lo, hi = bpf.BANDS[index - 1]
        metrics = {}
        for pop_name in population.POPULATIONS:
            r = evaluate(index, mc_runs, pop_name)
            checks += _filter_checks(index, r, mc_runs, "" if pop_name == population.DEFAULT else f"{pop_name}_")
            il, rl, rej = r["nominal"]
            metrics[pop_name] = {"il_db": il, "rl_db": rl, "rej3f_db": rej if has_rejection_target(index) else None,
                                 "il_worst_q_db": r["il_worst_q_db"], "mc_runs": mc_runs, "mc_passed": r["mc_passed"]}
            if pop_name == population.DEFAULT:
                s = r["s2p"]
        f = s["frequency"].real
        s2p = write_s2p(char_dir / f"bpf_{index}.s2p", f,
                        {k: s[f"s_{k[0]}_{k[1]}"] for k in ("11", "21", "12", "22")},
                        comments=(f"NeoRadio BPF {index}: {lo / 1e6:g}-{hi / 1e6:g} MHz, selected in the bank",
                                  f"population {population.DEFAULT}", f"generated_by {revision}"))
        plots[f"{index}: {lo / 1e6:g}–{hi / 1e6:g} MHz"] = (f, db(s["s_2_1"]))
        entries.append({"index": index, "lo_hz": lo, "hi_hz": hi, "hpf_order": bpf.HPF_ORDER,
                        "lpf_order": bpf.LPF_ORDER[index - 1], **metrics[population.DEFAULT], "s2p": s2p.name,
                        "populations": metrics, "inductors": [e for e in sheet if e["filter"] == index]})
    mute = mute_isolation_db()
    checks.append(CheckResult("bank_mute_isolation_db", mute, f"≥ {MUTE_MIN_DB:g}", mute >= MUTE_MIN_DB))
    result = BlockResult("bpf_bank", checks, plots=plots, logx=True)
    bank = bpf_bank.build(None)
    data = availability.load()
    result.artefacts = {
        "kicad_netlist": export.kicad_netlist(bank.circuit, out_dir / "bpf_bank.net"),
        "bom": export.write_bom(bank.circuit, out_dir / "bpf_bank_bom.csv", data),
        "bom_iran": export.write_bom(bank.circuit, out_dir / "bpf_bank_bom_iran.csv", data, "iran"),
        "svg": export.svg_diagram(bank.circuit, out_dir / "bpf_bank.svg"),
        "characterisation": charfile.write(
            char_dir / "bpf_bank.json", "bpf_bank", "pass" if result.passed else "fail",
            {"ripple_db": list(bpf.RIPPLE_DB), "inductor_data": "hardware/neohw/data/inductors.json", "selection_rule": SELECTION_RULE,
             "filters": entries, "checks": [c.__dict__ for c in checks]}),
    }
    return result
