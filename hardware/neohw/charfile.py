"""Characterisation files: the contract between neohw and the digital twin."""
from __future__ import annotations

import json
from pathlib import Path

from neohw.tools import git_describe

SCHEMA_VERSION = 1


def write(path: Path, block: str, status: str, payload: dict) -> Path:
    if status not in ("pass", "fail"):
        raise ValueError(f"status must be 'pass' or 'fail', got {status!r}")
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    doc = {"schema_version": SCHEMA_VERSION, "block": block, "status": status,
           "generated_by": git_describe(), **payload}
    path.write_text(json.dumps(doc, indent=1))
    return path


def read(path: Path, expected_block: str) -> dict:
    doc = json.loads(Path(path).read_text())
    if doc.get("schema_version") != SCHEMA_VERSION:
        raise ValueError(f"{path}: unsupported schema_version {doc.get('schema_version')}, expected {SCHEMA_VERSION}")
    if doc.get("block") != expected_block:
        raise ValueError(f"{path}: block is {doc.get('block')!r}, expected {expected_block!r}")
    return doc
