"""Parse ngspice ASCII rawfiles (set filetype=ascii; write file all)."""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


class RawfileError(ValueError):
    """The rawfile is malformed or truncated."""


@dataclass
class Plot:
    name: str
    variables: list[str]
    data: dict[str, np.ndarray]


def _header(lines: list[str], i: int, key: str) -> tuple[str, int]:
    while i < len(lines) and not lines[i].startswith(key):
        if lines[i].startswith("Values:"):
            raise RawfileError(f"missing header {key!r}")
        i += 1
    if i >= len(lines):
        raise RawfileError(f"missing header {key!r}")
    return lines[i].split(":", 1)[1].strip(), i + 1


def _number(token: str, is_complex: bool) -> complex | float:
    if is_complex:
        re_part, im_part = token.split(",")
        return complex(float(re_part), float(im_part))
    return float(token)


def parse_ascii(text: str) -> list[Plot]:
    lines = text.splitlines()
    plots: list[Plot] = []
    i = 0
    while i < len(lines):
        if not lines[i].startswith("Title:"):
            if lines[i].strip() == "":
                i += 1
                continue
            raise RawfileError(f"line {i + 1}: expected 'Title:', got {lines[i][:40]!r}")
        name, i = _header(lines, i, "Plotname:")
        flags, i = _header(lines, i, "Flags:")
        nvars, i = _header(lines, i, "No. Variables:")
        npoints, i = _header(lines, i, "No. Points:")
        _, i = _header(lines, i, "Variables:")
        is_complex = "complex" in flags
        nvars_i, npoints_i = int(nvars), int(npoints)
        variables = []
        for _ in range(nvars_i):
            parts = lines[i].split()
            variables.append(parts[1])
            i += 1
        if not lines[i].startswith("Values:"):
            raise RawfileError(f"line {i + 1}: expected 'Values:'")
        i += 1
        tokens: list[str] = []
        needed = nvars_i * npoints_i
        while len(tokens) < needed * 2 and i < len(lines) and not lines[i].startswith("Title:"):
            tokens.extend(lines[i].split())
            i += 1
        # every point starts with its index followed by nvars values
        values = []
        j = 0
        for point in range(npoints_i):
            if j >= len(tokens):
                raise RawfileError(f"{name}: expected {npoints_i} points, got {point}")
            j += 1  # point index
            row = tokens[j:j + nvars_i]
            if len(row) < nvars_i:
                raise RawfileError(f"{name}: expected {npoints_i} points, got {point}")
            values.append([_number(t, is_complex) for t in row])
            j += nvars_i
        arr = np.array(values, dtype=complex if is_complex else float)
        plots.append(Plot(name, variables, {v: arr[:, k] for k, v in enumerate(variables)}))
    if not plots:
        raise RawfileError("no plots found")
    return plots
