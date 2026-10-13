FT8_DIR := third_party/ft8_lib
SLN     := pi/NeoRadio.slnx
PYTHON  ?= python3
# dotnet on this machine intermittently aborts with "Internal CLR error (0x80131506)" (~1 in 15 runs),
# so each dotnet step gets up to 3 attempts; a real build or test failure still fails after the third.
RETRY = for i in 1 2 3; do $(1) && exit 0; echo "dotnet attempt $$i failed" >&2; done; exit 1

.PHONY: tools dsp ft8 test sim hw hw-fresh fw fw-test clean

tools: ft8 dsp

ft8:
	$(MAKE) -C $(FT8_DIR) -j8

dsp:
	@$(call RETRY,dotnet build $(SLN) -c Release)

test: tools
	@$(call RETRY,dotnet test $(SLN) -c Release)
	cd sim && $(PYTHON) -m pytest -q
	cd hardware && $(PYTHON) -m pytest -q
	$(MAKE) fw-test

sim: tools
	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -14 -22 1 --trials 20 --sweep-hardware --hardware-checks

# simulation results are cached in build/hw-cache by deck content: unchanged blocks are not re-simulated
hw:
	cd hardware && $(PYTHON) -m neohw run --out ../build/hw --characterized characterized --report ../build/hw/report.html

# everything re-simulated (the cache is refreshed, not read): proves the characterisation reproduces
# from the code; a following `make hw` must produce identical files from the cache
hw-fresh:
	cd hardware && NEOHW_FRESH=1 $(PYTHON) -m neohw run --out ../build/hw --characterized characterized --report ../build/hw/report.html

FW_BUILD := build/firmware

fw:
	cmake -S firmware -B $(FW_BUILD) -G Ninja -DCMAKE_BUILD_TYPE=Release
	ninja -C $(FW_BUILD)

# firmware unit tests and the C-vs-Python cross-checks
fw-test: fw
	ctest --test-dir $(FW_BUILD) --output-on-failure
	cd firmware && $(PYTHON) -m pytest -q tests

clean:
	rm -rf build
	dotnet clean $(SLN) -c Release
