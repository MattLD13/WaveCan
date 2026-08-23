# WaveCan - Motor Control Platform

## Quick Start (Linux/Raspberry Pi)

### Installation (Choose One)

#### Option A: Automated Setup (Recommended)
```bash
cd /home/pi/WaveCan
bash setup.sh
sudo systemctl start wavecan
sudo systemctl enable wavecan  # Enable auto-boot
```

#### Option B: Manual Setup
```bash
# 1. Install dependencies
sudo apt-get update
sudo apt-get install -y python3 python3-pip network-manager
pip3 install python-can

# 2. Copy systemd service
sudo cp /home/pi/WaveCan/wavecan.service /etc/systemd/system/

# 3. Enable and start
sudo systemctl daemon-reload
sudo systemctl enable wavecan
sudo systemctl start wavecan
```

## Access Dashboard

- **URL**: `http://<your-pi-ip>:8080`
- **Find your Pi's IP**:
  ```bash
  hostname -I
  ```

## Available Endpoints

### Dashboard & Status
- `GET /` - Web UI dashboard
- `GET /api/status` - Motor telemetry JSON
- `GET /api/health` - Server health check

### Motor Control
- `POST /api/motor/cmd` - Send motor command (speed/rpm)
- `POST /api/motor/pid` - Configure PID settings
- `GET /api/motors` - List available motors

### New Network & CAN Features
- `POST /api/network/connect` - Connect to WiFi SSID (Linux/nmcli)
  ```bash
  curl -X POST http://localhost:8080/api/network/connect \
    -H "Content-Type: application/json" \
    -d '{"ssid":"MyWiFi", "password":"pass123"}'
  ```
- `POST /api/can/open` - Enable CAN bus
- `POST /api/can/close` - Disable CAN bus

## Configuration

Edit `/etc/systemd/system/wavecan.service` to change:
- `WAVECAN_RUNTIME_MODE`: `mock` (simulation) or `socketcan` (real hardware)
- `WAVECAN_HTTP_HOST`: `0.0.0.0` (all interfaces) or specific IP
- `WAVECAN_HTTP_PORT`: Default `8080`

Example for SocketCAN mode:
```ini
Environment="WAVECAN_RUNTIME_MODE=socketcan"
Environment="WAVECAN_CAN_INTERFACE=can1"
```

Then reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart wavecan
```

## View Logs

```bash
# Live logs
sudo journalctl -u wavecan -f

# Last 50 lines
sudo journalctl -u wavecan -n 50

# Since last boot
sudo journalctl -u wavecan -b
```

## Troubleshooting

### Service won't start
```bash
sudo systemctl status wavecan
sudo journalctl -u wavecan -n 100
```

### Can't connect to WiFi
- Ensure `nmcli` is installed: `which nmcli`
- Or use hostapd/dnsmasq: `sudo apt-get install hostapd dnsmasq`

### CAN bus not responding
- Check interface: `ip link show can1`
- Bring it up: `sudo ip link set can1 up`
- Or use mock mode: Set `WAVECAN_RUNTIME_MODE=mock`

## Testing

```bash
# Run in foreground (mock mode)
WAVECAN_RUNTIME_MODE=mock python3 /home/pi/WaveCan/main.py

# Run tests
cd /home/pi/WaveCan
pytest -v

# Quick health check
curl http://localhost:8080/api/health
```

## Hotspot (LUSI)

When WiFi is unavailable, WaveCan automatically creates a fallback hotspot:
- **SSID**: `LUSI`
- **Auth**: Open (no password)
- **IP**: `192.168.4.1` (hostapd) or assigned by NetworkManager (nmcli)

Connect and access: `http://192.168.4.1:8080`
