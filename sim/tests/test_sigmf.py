import json

import numpy as np

from neosim.sigmf import write_sigmf


def test_write_sigmf_round_trip(tmp_path):
    iq = np.array([1 + 2j, -0.5 + 0.25j], dtype=np.complex128)
    meta = write_sigmf(tmp_path / "cap", iq, 96_000, 14_060_000.0)
    assert meta.name == "cap.sigmf-meta"
    doc = json.loads(meta.read_text())
    assert doc["global"]["core:datatype"] == "cf32_le"
    assert doc["global"]["core:sample_rate"] == 96_000
    assert doc["captures"][0]["core:frequency"] == 14_060_000.0
    data = np.fromfile(tmp_path / "cap.sigmf-data", dtype="<c8")
    np.testing.assert_allclose(data, iq.astype(np.complex64))
