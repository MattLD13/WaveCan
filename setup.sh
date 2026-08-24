#!/bin/bash
# WaveCan Quick Setup Script for Linux/Raspberry Pi

set -e

echo "======================================"
echo "WaveCan Quick Setup"
echo "======================================"

# Detect OS
if [[ "$OSTYPE" != "linux-gnu"* ]]; then
    echo "ERROR: This script is for Linux only"
    exit 1
fi

# Check if running as root
if [[ $EUID -eq 0 ]]; then
   echo "Please do NOT run this script as root. Use: bash setup.sh"
   exit 1
fi

echo ""
echo "[1/4] Installing system dependencies..."
sudo apt-get update
sudo apt-get install -y python3 python3-pip network-manager

echo ""
echo "[2/4] Installing Python dependencies..."
pip3 install python-can

echo ""
echo "[3/4] Installing optional dependencies (for dev)..."
pip3 install -r requirements-dev.txt || echo "  (dev dependencies optional, skipping)"

echo ""
echo "[4/4] Setting up systemd service for auto-boot..."
sudo cp wavecan.service /etc/systemd/system/wavecan.service
sudo systemctl daemon-reload

echo ""
echo "======================================"
echo "✓ Setup Complete!"
echo "======================================"
echo ""
echo "Next steps:"
echo ""
echo "  1. Start the service now:"
echo "     sudo systemctl start wavecan"
echo ""
echo "  2. Enable auto-boot:"
echo "     sudo systemctl enable wavecan"
echo ""
echo "  3. View logs:"
echo "     sudo journalctl -u wavecan -f"
echo ""
echo "  4. Access dashboard:"
echo "     http://$(hostname -I | awk '{print $1}'):8080"
echo ""
echo "  5. (Optional) Use SocketCAN mode:"
echo "     - Set WAVECAN_RUNTIME_MODE=socketcan in wavecan.service"
echo "     - Set up CAN interface: sudo ip link add dev can1 type can bitrate 1000000"
echo ""
