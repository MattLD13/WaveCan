#!/bin/bash
# Install and enable the WaveCan Bluetooth LE bridge on Raspberry Pi OS.

set -euo pipefail

if [[ $EUID -eq 0 ]]; then
    echo "Run this script as the pi user: bash setup_bluetooth.sh"
    exit 1
fi

repo_dir=$(cd "$(dirname "$0")" && pwd)

sudo apt-get update
sudo apt-get install -y bluez rfkill python3-dbus python3-gi python3-can

sudo rfkill unblock bluetooth
sudo systemctl enable --now bluetooth.service

sudo cp "$repo_dir/systemd/setup-can.service" /etc/systemd/system/setup-can.service
sudo cp "$repo_dir/systemd/wavecan-bluetooth.service" /etc/systemd/system/wavecan-bluetooth.service
sudo systemctl daemon-reload
sudo systemctl enable --now setup-can.service

# Only one WaveCan process should own the motor-control refresh loop.
sudo systemctl disable --now wavecan.service 2>/dev/null || true
sudo systemctl enable --now wavecan-bluetooth.service

sudo systemctl --no-pager --full status wavecan-bluetooth.service
