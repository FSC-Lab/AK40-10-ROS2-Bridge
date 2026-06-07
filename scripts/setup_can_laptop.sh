#!/bin/bash
# Bring up the USB-to-CAN dongle on Ubuntu laptop (can0).
# Run once after every reboot: sudo ./scripts/setup_can_laptop.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: run with sudo" >&2
  exit 1
fi

# Bring interface down cleanly before reconfiguring (ignore error if already down)
ip link set can0 down 2>/dev/null || true

ip link set can0 type can bitrate 1000000
ip link set can0 up

echo "can0 is up:"
ip link show can0
