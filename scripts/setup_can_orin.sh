#!/bin/bash
# Bring up the USB-to-CAN dongle on Jetson Orin (can1).
# Run once after every reboot: sudo ./scripts/setup_can_orin.sh

set -e

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: run with sudo" >&2
  exit 1
fi

# Load gs_usb if not already present
if ! lsmod | grep -q gs_usb; then
  echo "Loading gs_usb module..."
  modprobe gs_usb
fi

# Bring interface down cleanly before reconfiguring (ignore error if already down)
ip link set can1 down 2>/dev/null || true

ip link set can1 type can bitrate 1000000
ip link set can1 up

echo "can1 is up:"
ip link show can1
