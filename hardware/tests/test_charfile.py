import json

import pytest

from neohw import charfile


def test_write_and_read_round_trip(tmp_path):
    p = charfile.write(tmp_path / "a.json", "attenuator", "pass", {"freq_hz": [1.0]})
    doc = json.loads(p.read_text())
    assert doc["schema_version"] == 1 and doc["block"] == "attenuator" and doc["status"] == "pass"
    assert doc["generated_by"]
    assert charfile.read(p, "attenuator")["freq_hz"] == [1.0]


def test_read_rejects_wrong_block_or_schema(tmp_path):
    p = charfile.write(tmp_path / "a.json", "attenuator", "pass", {})
    with pytest.raises(ValueError, match="block"):
        charfile.read(p, "preamp")
    doc = json.loads(p.read_text())
    doc["schema_version"] = 99
    p.write_text(json.dumps(doc))
    with pytest.raises(ValueError, match="schema_version"):
        charfile.read(p, "attenuator")


def test_status_must_be_pass_or_fail(tmp_path):
    with pytest.raises(ValueError):
        charfile.write(tmp_path / "a.json", "attenuator", "maybe", {})
