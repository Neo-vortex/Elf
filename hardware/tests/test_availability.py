import pytest

from neohw.availability import lookup

DATA = {
    "G6K-2F": {"ok": True, "hits": [{"name": "G6K-2F", "price": "x", "shop": "y", "instock": True}]},
    "RD16HHF1": {"ok": True, "hits": [{"name": "RD16HHF1", "price": "", "shop": "y", "instock": False}]},
    "nothing": {"ok": True, "hits": []},
}


def test_bom_flags_unknown_and_out_of_stock():
    assert lookup("G6K-2F", DATA) == "available"
    assert lookup("RD16HHF1", DATA) == "out_of_stock"
    assert lookup("nothing", DATA) == "unknown"
    assert lookup("never-searched", DATA) == "unknown"
    assert lookup(None, DATA) == "unknown"


def test_unrelated_hits_do_not_count():
    data = {
        "GDT 90V": {"ok": True, "hits": [{"name": "GDT 230V 3-pin", "instock": True}]},
        "ترانزیستور J310": {"ok": True, "hits": [{"name": "میکروفون سامسونگ", "instock": True},
                                                {"name": "ترانزیستور J-310 TO92", "instock": False}]},
        "G6K-2F": {"ok": True, "hits": [{"name": "رله G6K-2F-5VDC امرن", "instock": True}]},
    }
    assert lookup("GDT 90V", data) == "unknown"
    assert lookup("ترانزیستور J310", data) == "out_of_stock"
    assert lookup("G6K-2F", data) == "available"


def test_inductor_values_parsed_from_shop_titles():
    from neohw.availability import inductor_values
    assert inductor_values("سلف 470 نانو هانری سایز 1210") == [pytest.approx(470e-9)]
    assert inductor_values("سلف 4.7µH میکروهانری SMD") == [pytest.approx(4.7e-6)]
    assert inductor_values("سلف اس ام دی 10UH (1210) TDK") == [pytest.approx(10e-6)]
    assert inductor_values("سلف 270نانوهانری 270NH 0805") == [pytest.approx(270e-9)] * 2
    assert inductor_values("سلف 1.5 ميکرو هانری") == [pytest.approx(1.5e-6)]
    assert inductor_values("رله G6K-2F") == []


def test_value_match_replaces_token_match():
    from neohw.availability import inductor_hit
    data = {"سلف رنگی": {"ok": True, "hits": [{"name": "سلف رنگی 10 میکرو هانری", "instock": True},
                                              {"name": "سلف رنگی 4.7UH", "instock": False}]}}
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 10e-6)) == "available"
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 4.7e-6)) == "out_of_stock"
    assert lookup("سلف رنگی", data, lambda n: inductor_hit(n, 2.2e-6)) == "unknown"
