"""Every footprint a block uses exists in the installed KiCad 9 libraries (project-local
`NeoRadio:` footprints are drawn in the layout phase)."""
from pathlib import Path

import pytest

from neohw.blocks import attenuator, bias_tee, bpf_bank, cal_tone, clock, controller, mixer, power, preamp, protection

LIB = Path("/usr/share/kicad/footprints")


@pytest.mark.skipif(not LIB.exists(), reason="KiCad footprint libraries not installed")
def test_all_footprints_exist_in_the_kicad_libraries():
    blocks = [attenuator.build("energised"), bias_tee.build(), bpf_bank.build(7), cal_tone.build(), clock.build(),
              mixer.build(), power.build(), controller.build(), preamp.build("amp"), protection.build()]
    missing = set()
    for b in blocks:
        for p in b.circuit.parts:
            lib, name = p.footprint.split(":")
            if lib != "NeoRadio" and not (LIB / f"{lib}.pretty" / f"{name}.kicad_mod").exists():
                missing.add(p.footprint)
    assert not missing, sorted(missing)


import pytest


@pytest.mark.parametrize("smd", ["1210", "1812"])
def test_combined_inductor_footprint_has_smd_and_tht_pads_on_both_terminals(smd, tmp_path):
    from neohw import footprints
    text = footprints.combined_inductor(tmp_path, smd).read_text()
    assert text.count('(pad "1"') == 2 and text.count('(pad "2"') == 2
    assert "smd" in text and "thru_hole" in text and f"L_{smd}_Axial_P10.16mm" in text


@pytest.mark.parametrize("smd", ["1210", "1812"])
def test_the_committed_combined_footprint_is_the_generated_one(smd, tmp_path):
    from neohw import footprints
    committed = footprints.FOOTPRINT_LIB / f"L_{smd}_Axial_P10.16mm.kicad_mod"
    assert committed.read_text() == footprints.combined_inductor(tmp_path, smd).read_text()


def test_combined_footprint_pads_do_not_overlap_and_fit_the_78f_body():
    from neohw import footprints
    for smd, (x, (w, _h)) in footprints.SMD_PADS.items():
        assert x + w / 2 < footprints.THT_PAD_X - footprints.THT_PAD / 2, smd     # SMD clear of THT pad
    assert footprints.BODY[0] >= 7.11 and footprints.BODY[1] >= 2.79           # Bourns 78F max body
