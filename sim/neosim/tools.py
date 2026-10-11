"""Locate external binaries used by the simulation."""
from __future__ import annotations

import os
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]

DEFAULT_PATHS: dict[str, Path] = {
    "gen_ft8": REPO_ROOT / "third_party" / "ft8_lib" / "gen_ft8",
    "decode_ft8": REPO_ROOT / "third_party" / "ft8_lib" / "decode_ft8",
    "neoradio-dsp": REPO_ROOT / "pi" / "src" / "NeoRadio.Dsp.Cli" / "bin" / "Release" / "net10.0" / "neoradio-dsp",
}

ENV_OVERRIDES: dict[str, str] = {
    "gen_ft8": "NEOSIM_GEN_FT8",
    "decode_ft8": "NEOSIM_DECODE_FT8",
    "neoradio-dsp": "NEOSIM_DSP_CLI",
}


class ToolNotFoundError(RuntimeError):
    """An external tool is missing or not executable."""


def tool_path(name: str) -> Path:
    """Return the path of an external tool, honouring env overrides."""
    if name not in DEFAULT_PATHS:
        raise KeyError(name)
    path = Path(os.environ.get(ENV_OVERRIDES[name], DEFAULT_PATHS[name]))
    if not (path.is_file() and os.access(path, os.X_OK)):
        raise ToolNotFoundError(
            f"{name} not found or not executable at {path}; run `make tools` in {REPO_ROOT}")
    return path
