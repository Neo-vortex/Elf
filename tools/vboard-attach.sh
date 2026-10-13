#!/bin/sh
# Attaches the running virtual board (make vboard) to this PC's USB stack through vhci_hcd.
# Needs root for the kernel module and the attach; the board then appears as the
# "Elf HF Receiver" sound card and a /dev/ttyACM serial port. Detach: tools/vboard-detach.sh
set -e
sudo modprobe vhci-hcd
sudo usbip attach -r 127.0.0.1 -b 1-1
sleep 2
grep -A1 "Elf HF Receiver" /proc/asound/cards || echo "sound card not up yet: check dmesg"
ls /dev/serial/by-id/ 2>/dev/null | grep Elf || echo "serial port not up yet: check dmesg"
