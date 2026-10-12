import subprocess

import pytest

from neohw import ngspice
from neohw.ngspice import run_deck
from neohw.tools import ToolNotFoundError, tool_path

RC = """rc
V1 in 0 dc 0 ac 1
R1 in out 1k
C1 out 0 1n
.control
set filetype=ascii
ac dec 5 1k 1meg
write a0.raw all
.endc
.end
"""


@pytest.fixture
def cache(tmp_path, monkeypatch):
    try:
        tool_path("ngspice")
    except ToolNotFoundError as e:
        pytest.skip(str(e))
    monkeypatch.setenv("NEOHW_CACHE", str(tmp_path / "cache"))
    monkeypatch.delenv("NEOHW_FRESH", raising=False)
    return tmp_path / "cache"


def _no_ngspice(monkeypatch):
    def boom(*a, **k):
        raise AssertionError("ngspice ran on a cache hit")
    monkeypatch.setattr(ngspice.subprocess, "run", boom)


def test_a_repeated_deck_is_read_from_the_cache(cache, monkeypatch):
    first = run_deck(RC, ["a0.raw"])["a0.raw"][0].data["v(out)"]
    assert any(cache.rglob("a0.raw.gz"))
    _no_ngspice(monkeypatch)
    again = run_deck(RC, ["a0.raw"])["a0.raw"][0].data["v(out)"]
    assert (again == first).all()


def test_any_change_to_the_deck_misses_the_cache(cache, monkeypatch):
    run_deck(RC, ["a0.raw"])
    calls = []
    real = subprocess.run
    monkeypatch.setattr(ngspice.subprocess, "run", lambda *a, **k: calls.append(1) or real(*a, **k))
    run_deck(RC.replace("1k\n", "1.1k\n", 1), ["a0.raw"])
    assert calls          # ngspice ran (twice at most: version probe + simulation)


def test_a_fresh_run_ignores_the_cache_but_refreshes_it(cache, monkeypatch):
    """make hw-fresh re-simulates everything (reproducibility from the code) and stores the
    results, so the cached make hw that follows is all hits."""
    monkeypatch.setenv("NEOHW_FRESH", "1")
    calls = []
    real = subprocess.run
    monkeypatch.setattr(ngspice.subprocess, "run", lambda *a, **k: calls.append(1) or real(*a, **k))
    run_deck(RC, ["a0.raw"])
    run_deck(RC, ["a0.raw"])
    assert len([c for c in calls]) >= 2          # ran both times despite the stored entry
    monkeypatch.delenv("NEOHW_FRESH")
    _no_ngspice(monkeypatch)
    run_deck(RC, ["a0.raw"])                      # now a hit


def test_a_failed_run_is_not_cached(cache):
    bad = RC.replace("C1 out 0 1n", "Q1 out 0 0 NOPE")
    with pytest.raises(ngspice.NgspiceError):
        run_deck(bad, ["a0.raw"])
    assert not cache.exists() or not any(cache.rglob("a0.raw.gz"))


def test_entries_are_compressed(cache):
    run_deck(RC, ["a0.raw"])
    assert list(cache.rglob("a0.raw.gz")) and not [p for p in cache.rglob("*") if p.name == "a0.raw"]


def test_a_failure_to_store_does_not_fail_the_simulation(cache, monkeypatch):
    def full_disk(*a, **k):
        raise OSError(28, "No space left on device")
    monkeypatch.setattr(ngspice, "_store", full_disk)
    assert run_deck(RC, ["a0.raw"])["a0.raw"][0].data["v(out)"] is not None


def test_prune_removes_entries_older_than_the_fresh_run(cache, monkeypatch):
    import os
    import time
    run_deck(RC, ["a0.raw"])
    old = next(cache.rglob("a0.raw.gz")).parent
    os.utime(old, (time.time() - 3600, time.time() - 3600))
    start = time.time() - 1
    run_deck(RC.replace("1k\n", "2k\n", 1), ["a0.raw"])
    removed = ngspice.prune_cache(start)
    assert removed == 1 and not old.exists() and len(list(cache.rglob("a0.raw.gz"))) == 1
