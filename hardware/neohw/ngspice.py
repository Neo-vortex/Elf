"""Run ngspice in batch mode and collect rawfiles."""
from __future__ import annotations

import functools
import hashlib
import os
import shutil
import subprocess
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from neohw.rawfile import Plot, RawfileError, parse_ascii
from neohw.tools import REPO_ROOT, tool_path


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


@functools.cache
def _ngspice_version(ngspice: str) -> str:
    proc = subprocess.run([ngspice, "-v"], capture_output=True, text=True)
    return (proc.stdout + proc.stderr).strip()


def _cache_entry(deck: str, rawfiles: list[str], ngspice: str) -> Path | None:
    """Result-cache entry for a deck, keyed by the whole deck text (models are inlined), the
    requested rawfiles and the ngspice version. NEOHW_CACHE moves it (default build/hw-cache)."""
    root = Path(os.environ.get("NEOHW_CACHE") or REPO_ROOT / "build" / "hw-cache")
    key = hashlib.sha256("\0".join([_ngspice_version(ngspice), *rawfiles, deck]).encode()).hexdigest()
    return root / key[:2] / key


def _parse(workdir: Path, rawfiles: list[str], deck_path: Path, log: str) -> dict[str, list[Plot]]:
    results: dict[str, list[Plot]] = {}
    for name in rawfiles:
        path = workdir / name
        if not path.exists():
            raise _fail(f"ngspice did not write {name}", deck_path, log)
        try:
            results[name] = parse_ascii(path.read_text())
        except RawfileError as e:
            raise _fail(f"could not parse {name}: {e}", deck_path, log) from e
    return results


def _store(entry: Path, workdir: Path, rawfiles: list[str]) -> None:
    """Copy a successful run's rawfiles into the cache atomically, replacing an older entry (a fresh
    run); if a concurrent run of the same deck stores it at the same time, either copy is kept."""
    entry.parent.mkdir(parents=True, exist_ok=True)
    tmp = Path(tempfile.mkdtemp(prefix=entry.name[:8] + "-", dir=entry.parent))
    for name in rawfiles:
        shutil.copy2(workdir / name, tmp / name)
    shutil.rmtree(entry, ignore_errors=True)
    try:
        os.rename(tmp, entry)
    except OSError:
        shutil.rmtree(tmp, ignore_errors=True)


def run_deck(deck: str, rawfiles: list[str], keep_dir: Path | None = None) -> dict[str, list[Plot]]:
    """Run one deck. Results are cached by deck text (unless keep_dir is given); with NEOHW_FRESH=1
    (`make hw-fresh`) the cache is not read but is refreshed with the new results."""
    ngspice = tool_path("ngspice")
    entry = None if keep_dir is not None else _cache_entry(deck, rawfiles, str(ngspice))
    fresh = bool(os.environ.get("NEOHW_FRESH"))
    if entry is not None and not fresh and all((entry / name).exists() for name in rawfiles):
        return _parse(entry, rawfiles, entry / "deck.cir", "(cached result)")
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
    results = _parse(workdir, rawfiles, deck_path, log)
    if entry is not None:
        _store(entry, workdir, rawfiles)
    # on failure the temp dir is left behind: the error names its deck.cir and ngspice.log
    if tmp is not None:
        shutil.rmtree(tmp, ignore_errors=True)
    return results


def run_decks(decks: list[str], rawfiles: list[str], workers: int | None = None) -> list[dict[str, list[Plot]]]:
    """Run independent decks in parallel (each is its own ngspice process); results in input order."""
    with ThreadPoolExecutor(max_workers=workers or os.cpu_count()) as pool:
        return list(pool.map(lambda deck: run_deck(deck, rawfiles), decks))
