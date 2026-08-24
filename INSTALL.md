# WaveCan Installation & Setup

## Linux (Raspberry Pi / Desktop)

### Step 1: Install System Dependencies

```bash
# For SocketCAN support
sudo apt-get update
sudo apt-get install -y python3 python3-pip python3-venv

# For WiFi hotspot / NetworkManager (optional but recommended)
sudo apt-get install -y network-manager network-manager-gnome

# For traditional hostapd hotspot (alternative to NetworkManager)
sudo apt-get install -y hostapd dnsmasq
```

### Step 2: Install Python Dependencies

```bash
cd /home/pi/WaveCan

# Install python-can for SocketCAN
pip3 install python-can

# (Optional) Install dev dependencies for testing
pip3 install -r requirements-dev.txt
```

### Step 3: Set up SocketCAN Interface (if using real CAN hardware)

```bash
# On Raspberry Pi with MCP2515 or similar CAN hat:
sudo ip link add dev can1 type can bitrate 1000000
sudo ip link set can1 up

# Verify it's up
ip link show can1
```

To make this persistent across reboots, add to `/etc/network/interfaces`:
```
auto can1
iface can1 can static
    bitrate 1000000
```

### Step 4: Auto-Start on Boot (systemd)

```bash
# Copy systemd service file
sudo cp wavecan.service /etc/systemd/system/

# Enable and start the service
sudo systemctl daemon-reload
sudo systemctl enable wavecan
sudo systemctl start wavecan

# Check status
sudo systemctl status wavecan

# View logs
sudo journalctl -u wavecan -f
```

### Troubleshooting

#### CAN interface not found in socketcan mode
```bash
# Fall back to mock mode
WAVECAN_RUNTIME_MODE=mock python3 main.py
```

#### Permission errors with CAN
```bash
# Grant user access to CAN interface
sudo usermod -a -G dialout $USER
# Log out and back in
```

#### No hotspot available
- Ensure NetworkManager OR hostapd+dnsmasq is installed
- Check: `which nmcli` or `which hostapd`

### Accessing the Dashboard

- **URL**: `http://<device-ip>:8080`
- **Default Host**: `0.0.0.0:8080` (accessible from any interface)
- **Dashboard Features**:
  - Real-time motor telemetry (RPM, temp, current)
  - Motor control slider (0-100%)
  - WiFi connect form (requires nmcli)
  - CAN bus toggle (open/close)

### Environment Variables

```bash
# Set runtime mode (mock or socketcan)
export WAVECAN_RUNTIME_MODE=mock

# Set CAN interface
export WAVECAN_CAN_INTERFACE=can1

# Set HTTP server host/port
export WAVECAN_HTTP_HOST=0.0.0.0
export WAVECAN_HTTP_PORT=8080

# Then run
python3 main.py
```

## Testing

```bash
# Run full test suite
pytest -v

# Run specific test
pytest tests/test_hardware_motor_controller.py -v

# Mock mode smoke test
WAVECAN_RUNTIME_MODE=mock timeout 5 python3 main.py
```
