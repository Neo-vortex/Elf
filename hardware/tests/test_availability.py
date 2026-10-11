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
