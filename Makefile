FT8_DIR := third_party/ft8_lib
SLN     := pi/NeoRadio.slnx
PYTHON  ?= python3
# dotnet on this machine intermittently aborts with "Internal CLR error (0x80131506)" (~1 in 15 runs),
# so each dotnet step gets up to 3 attempts; a real build or test failure still fails after the third.
RETRY = for i in 1 2 3; do $(1) && exit 0; echo "dotnet attempt $$i failed" >&2; done; exit 1

.PHONY: tools dsp ft8 test sim hw clean

tools: ft8 dsp

ft8:
	$(MAKE) -C $(FT8_DIR) -j8

dsp:
	@$(call RETRY,dotnet build $(SLN) -c Release)

test: tools
	@$(call RETRY,dotnet test $(SLN) -c Release)
	cd sim && $(PYTHON) -m pytest -q
	cd hardware && $(PYTHON) -m pytest -q

sim: tools
	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -14 -22 1 --trials 20 --sweep-hardware --hardware-checks

hw:
	cd hardware && $(PYTHON) -m neohw run --out ../build/hw --characterized characterized --report ../build/hw/report.html

clean:
	rm -rf build
	dotnet clean $(SLN) -c Release
