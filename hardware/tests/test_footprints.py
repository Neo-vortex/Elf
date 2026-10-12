"""Every footprint a block uses exists in the installed KiCad 9 libraries (project-local
`NeoRadio:` footprints are drawn in the layout phase)."""
from pathlib import Path

import pytest

from neohw.blocks import attenuator, bias_tee, bpf_bank, cal_tone, clock, mixer, power, preamp, protection

LIB = Path("/usr/share/kicad/footprints")


@pytest.mark.skipif(not LIB.exists(), reason="KiCad footprint libraries not installed")
def test_all_footprints_exist_in_the_kicad_libraries():
    blocks = [attenuator.build("energised"), bias_tee.build(), bpf_bank.build(7), cal_tone.build(), clock.build(),
              mixer.build(), power.build(), preamp.build("amp"), protection.build()]
    missing = set()
    for b in blocks:
        for p in b.circuit.parts:
            lib, name = p.footprint.split(":")
            if lib != "NeoRadio" and not (LIB / f"{lib}.pretty" / f"{name}.kicad_mod").exists():
                missing.add(p.footprint)
    assert not missing, sorted(missing)
