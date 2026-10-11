"""Run ngspice in batch mode and collect rawfiles."""
from __future__ import annotations

import shutil
import subprocess
import tempfile
from pathlib import Path

from neohw.rawfile import Plot, RawfileError, parse_ascii
from neohw.tools import tool_path


class NgspiceError(RuntimeError):
    """ngspice failed or produced unusable output."""


# ngspice 45 exits 0 and only warns for these, then simulates what is left of the
# circuit. A node reached only through capacitors gives "singular matrix" at the
# operating point; part models must give every node a DC path (e.g. relay
# insulation resistance across open contacts).
_FATAL_WARNINGS = ("is not a valid", "singular matrix", "can't find model")


def _fail(reason: str, deck_path: Path, log: str) -> NgspiceError:
    tail = "\n".join(log.strip().splitlines()[-20:])
    return NgspiceError(f"{reason} (deck: {deck_path})\n--- ngspice log tail ---\n{tail}")


def run_deck(deck: str, rawfiles: list[str], keep_dir: Path | None = None) -> dict[str, list[Plot]]:
    ngspice = tool_path("ngspice")
    tmp = None
    if keep_dir is None:
        tmp = tempfile.mkdtemp(prefix="neohw-")
        workdir = Path(tmp)
    else:
        workdir = Path(keep_dir)
        workdir.mkdir(parents=True, exist_ok=True)
    deck_path = workdir / "deck.cir"
    deck_path.write_text(deck)
    proc = subprocess.run([str(ngspice), "-b", "deck.cir"], cwd=workdir, capture_output=True, text=True)
    log = proc.stdout + proc.stderr
    (workdir / "ngspice.log").write_text(log)
    if (proc.returncode != 0 or "Error" in log or "error:" in log.lower()
            or any(w in log for w in _FATAL_WARNINGS)):
        raise _fail(f"ngspice failed (exit {proc.returncode})", deck_path, log)
    results: dict[str, list[Plot]] = {}
    for name in rawfiles:
        path = workdir / name
        if not path.exists():
            raise _fail(f"ngspice did not write {name}", deck_path, log)
        try:
            results[name] = parse_ascii(path.read_text())
        except RawfileError as e:
            raise _fail(f"could not parse {name}: {e}", deck_path, log) from e
    # on failure the temp dir is left behind: the error names its deck.cir and ngspice.log
    if tmp is not None:
        shutil.rmtree(tmp, ignore_errors=True)
    return results
