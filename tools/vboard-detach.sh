#!/bin/sh
# Detaches every USB/IP device imported from 127.0.0.1 (the virtual board).
set -e
for p in $(usbip port 2>/dev/null | sed -n 's/^Port \([0-9][0-9]*\):.*/\1/p'); do
    sudo usbip detach -p "$p"
done
