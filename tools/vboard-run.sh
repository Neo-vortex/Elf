#!/bin/sh
# Runs the virtual board: the digital twin (rendering the stations file) and elf-virtual (the
# firmware, serving its USB device over USB/IP on 127.0.0.1:3240). Ctrl-C stops both.
#   tools/vboard-run.sh [STATIONS.yaml]
set -e
ROOT=$(cd "$(dirname "$0")/.." && pwd)
STATIONS=$(realpath "${1:-$ROOT/sim/scenarios/vboard/cw_bands.yaml}")
WORK="$ROOT/build/vboard"
mkdir -p "$WORK"
cd "$WORK"                      # the socket path is relative: Unix socket paths are limited to 107 bytes
rm -f twin.sock
PYTHONPATH="$ROOT/sim" python3 -m neosim vboard "$STATIONS" --socket twin.sock &
TWIN=$!
trap 'kill $TWIN 2>/dev/null' EXIT INT TERM
i=0
while [ ! -S twin.sock ] && [ $i -lt 300 ]; do sleep 0.1; i=$((i + 1)); done
"$ROOT/build/firmware/elf-virtual" --twin twin.sock
