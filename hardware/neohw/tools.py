"""Locate external tools and describe the git revision."""
from __future__ import annotations

import os
import shutil
import subprocess
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]
SOURCE_DIR = Path(__file__).resolve().parent

_ENV = {"ngspice": "NEOHW_NGSPICE", "dot": "NEOHW_DOT"}
_HINT = {
    "ngspice": "install it with `sudo apt install ngspice`",
    "dot": "install graphviz with `sudo apt install graphviz`",
}


class ToolNotFoundError(RuntimeError):
    """An external tool is missing or not executable."""


def tool_path(name: str) -> Path:
    if name not in _ENV:
        raise KeyError(name)
    override = os.environ.get(_ENV[name])
    candidate = override if override else shutil.which(name)
    if candidate:
        path = Path(candidate)
        if path.is_file() and os.access(path, os.X_OK):
            return path
    raise ToolNotFoundError(f"{name} not found (looked at {candidate or 'PATH'}); {_HINT[name]}")


def git_describe() -> str:
    """Last commit that changed the generating sources (hardware/neohw), -dirty if they have edits.

    Scoped to the sources so a committed characterisation file regenerates byte-identically:
    a file can never contain the hash of the commit that adds it.
    """
    try:
        head = subprocess.run(["git", "log", "-1", "--format=%h", "--", str(SOURCE_DIR)], cwd=REPO_ROOT,
                              capture_output=True, text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "status", "--porcelain", "--", str(SOURCE_DIR)], cwd=REPO_ROOT,
                               capture_output=True, text=True, check=True).stdout.strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"
    if not head:
        return "unknown"
    return f"{head}-dirty" if dirty else head
