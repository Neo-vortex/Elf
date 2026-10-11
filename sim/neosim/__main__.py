"""Command line: python -m neosim run SCENARIO... --report FILE"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

from neosim.report import write_report
from neosim.scenario import ScenarioError, load_scenario
from neosim.sweep import sweep_ft8
from neosim.tools import ToolNotFoundError
from neosim.twin import run_scenario


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="neosim")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run scenarios and write an HTML report")
    run.add_argument("scenarios", nargs="+", type=Path)
    run.add_argument("--workdir", type=Path, default=Path("build/sim"))
    run.add_argument("--report", type=Path, default=Path("build/sim/report.html"))
    run.add_argument("--sweep-ft8", nargs=3, type=float, metavar=("START", "STOP", "STEP"))
    run.add_argument("--trials", type=int, default=4)
    args = parser.parse_args(argv)

    try:
        scenarios = [load_scenario(p) for p in args.scenarios]
    except (ScenarioError, OSError) as e:
        print(f"scenario error: {e}", file=sys.stderr)
        return 2

    try:
        results = []
        for path, scenario in zip(args.scenarios, scenarios):
            result = run_scenario(scenario, args.workdir / path.stem)
            print(f"{'PASS' if result.passed else 'FAIL'}  {scenario.name}")
            results.append(result)
        points = None
        if args.sweep_ft8:
            start, stop, step = args.sweep_ft8
            snrs = list(np.arange(start, stop - 1e-9, -abs(step)))
            points = sweep_ft8(snrs, args.trials, args.workdir / "sweep")
    except (ToolNotFoundError, RuntimeError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2

    report = write_report(results, points, args.report)
    print(f"report: {report}")
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
