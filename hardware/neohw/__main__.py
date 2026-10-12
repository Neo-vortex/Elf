"""Command line: python -m neohw run"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

from neohw.checks import attenuator as attenuator_check
from neohw.checks import bias_tee as bias_tee_check
from neohw.checks import bpf as bpf_check
from neohw.checks import mixer as mixer_check
from neohw.checks import preamp as preamp_check
from neohw.checks import protection as protection_check
from neohw.checks import BlockResult, CheckResult
from neohw.ngspice import NgspiceError
from neohw.report import write_report
from neohw.spice import SpiceError
from neohw.tools import ToolNotFoundError

# signal-chain order
CHECKS = {"bias_tee": bias_tee_check.run, "attenuator": attenuator_check.run,
          "protection": protection_check.run, "bpf_bank": bpf_check.run, "preamp": preamp_check.run,
          "mixer": mixer_check.run}


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(prog="neohw")
    sub = parser.add_subparsers(dest="command", required=True)
    run = sub.add_parser("run", help="run block checks, write characterisation files and a report")
    run.add_argument("--out", type=Path, default=Path("build/hw"))
    run.add_argument("--characterized", type=Path, default=Path("characterized"))
    run.add_argument("--report", type=Path, default=Path("build/hw/report.html"))
    args = parser.parse_args(argv)

    results, errored = [], False
    for name, check in CHECKS.items():
        try:
            result = check(args.out, args.characterized)
        except (ToolNotFoundError, NgspiceError, SpiceError, RuntimeError) as e:
            print(f"ERROR {name}\nerror: {e}", file=sys.stderr)
            errored = True
            result = BlockResult(name, [CheckResult("ERROR", float("nan"), str(e).splitlines()[0], False, "")])
        else:
            print(f"{'PASS' if result.passed else 'FAIL'}  {name}")
        results.append(result)
    report = write_report(results, args.report)
    print(f"report: {report}")
    if errored:
        return 2
    return 0 if all(r.passed for r in results) else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
