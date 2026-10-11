FT8_DIR := third_party/ft8_lib
SLN     := pi/NeoRadio.slnx
PYTHON  ?= python3

.PHONY: tools dsp ft8 test sim clean

tools: ft8 dsp

ft8:
	$(MAKE) -C $(FT8_DIR) -j8

dsp:
	dotnet build $(SLN) -c Release

test: tools
	dotnet test $(SLN) -c Release
	cd sim && $(PYTHON) -m pytest -q

sim: tools
	cd sim && $(PYTHON) -m neosim run scenarios/*.yaml --workdir ../build/sim --report ../build/sim/report.html --sweep-ft8 -8 -24 2 --trials 4

clean:
	rm -rf build
	dotnet clean $(SLN) -c Release
