"""
Network Manager for WaveCan
Handles WiFi connectivity detection and provides fallback hotspot (LUSI)
"""

import subprocess
import sys
import os
import time
from typing import Optional, Tuple
from wavecan_platform import log, IS_MICROPYTHON


def check_internet_connectivity() -> bool:
    """
    Check if the device has internet connectivity.
    Tries to reach common DNS servers or ping google.com
    """
    if IS_MICROPYTHON:
        # MicroPython: try importing network module (RP2350)
        try:
            import network
            sta = network.WLAN(network.STA_IF)
            return sta.isconnected()
        except Exception as e:
            log(f"[NetworkManager] MicroPython network check failed: {e}")
            return False

    # Linux/Desktop: check with Linux commands
    try:
        # Check if wlan0 or similar interface is up and has an IP
        result = subprocess.run(
            ["ip", "link", "show"],
            capture_output=True,
            text=True,
            timeout=3
        )
        if "wlan0" not in result.stdout and "wlp" not in result.stdout:
            return False

        # Try to ping a reliable DNS (1.1.1.1 - Cloudflare)
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", "1.1.1.1"],
            capture_output=True,
            text=True,
            timeout=3
        )
        return result.returncode == 0
    except Exception as e:
        log(f"[NetworkManager] Connectivity check failed: {e}")
        return False


def is_hostapd_available() -> bool:
    """Check if hostapd (AP mode) is available on the system"""
    if IS_MICROPYTHON:
        return False

    try:
        result = subprocess.run(
            ["which", "hostapd"],
            capture_output=True,
            timeout=1
        )
        return result.returncode == 0
    except Exception:
        return False


def is_nmcli_available() -> bool:
    """Check if NetworkManager's nmcli is available on the system"""
    if IS_MICROPYTHON:
        return False
    try:
        result = subprocess.run(["which", "nmcli"], capture_output=True, timeout=1)
        return result.returncode == 0
    except Exception:
        return False


def connect_to_wifi(ssid: str, password: Optional[str] = None, interface: Optional[str] = None) -> Tuple[bool, str]:
    """Attempt to connect to a WiFi network using nmcli on Linux.

    Returns (success, message).
    """
    if IS_MICROPYTHON:
        return False, "Not supported on MicroPython"

    if not is_nmcli_available():
        return False, "nmcli not available"

    cmd = ["nmcli", "device", "wifi", "connect", ssid]
    if interface:
        cmd += ["ifname", interface]
    if password:
        cmd += ["password", password]

    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode == 0:
            return True, result.stdout.strip() or "Connected"
        else:
            return False, (result.stderr.strip() or result.stdout.strip() or "nmcli failed")
    except Exception as e:
        return False, str(e)


def get_wifi_interface() -> Optional[str]:
    """Get the WiFi interface name (wlan0, wlp3s0, etc.)"""
    if IS_MICROPYTHON:
        return None

    try:
        result = subprocess.run(
            ["ip", "link", "show"],
            capture_output=True,
            text=True,
            timeout=2
        )
        for line in result.stdout.split('\n'):
            if 'wlan' in line or 'wlp' in line or 'wifis' in line:
                parts = line.split(':')
                if len(parts) >= 2:
                    interface = parts[1].strip()
                    if interface:
                        return interface
    except Exception:
        pass

    return None


def setup_lusi_hotspot() -> bool:
    """
    Set up LUSI hotspot when WiFi is not available.
    Requires hostapd and dnsmasq on Linux/RPi.
    """
    if IS_MICROPYTHON:
        return setup_lusi_hotspot_micropython()

    return setup_lusi_hotspot_linux()


def setup_lusi_hotspot_micropython() -> bool:
    """Set up LUSI hotspot on MicroPython (RP2350)"""
    try:
        import network

        # Configure AP mode
        ap = network.WLAN(network.AP_IF)
        ap.active(True)

        # Set up SSID without password
        ap.config(essid='LUSI', authmode=network.AUTH_OPEN)

        # Get the AP IP
        ap_ip = ap.ifconfig()
        log(f"[NetworkManager] ✓ LUSI hotspot created (AP mode)")
        log(f"[NetworkManager]   SSID: LUSI")
        log(f"[NetworkManager]   Auth: Open (no password)")
        log(f"[NetworkManager]   IP: {ap_ip[0]}")

        return True
    except Exception as e:
        log(f"[NetworkManager] ✗ Failed to create MicroPython hotspot: {e}", "ERROR")
        return False


def setup_lusi_hotspot_linux() -> bool:
    """
    Set up LUSI hotspot on Linux using hostapd + dnsmasq.
    Designed for Raspberry Pi.
    """
    interface = get_wifi_interface()
    if not interface:
        log("[NetworkManager] ✗ No WiFi interface found", "WARN")
        return False

    try:
        # Step 1: Configure interface with static IP
        log(f"[NetworkManager] Configuring {interface} with static IP...")
        subprocess.run(
            ["sudo", "ip", "addr", "add", "192.168.4.1/24", "dev", interface],
            capture_output=True,
            timeout=5
        )

        # Step 2: Create hostapd config
        hostapd_config = f"""
interface={interface}
driver=nl80211
ssid=LUSI
hw_mode=g
channel=6
wmm_enabled=0
auth_algs=1
"""
        hostapd_path = "/tmp/hostapd_lusi.conf"
        with open(hostapd_path, 'w') as f:
            f.write(hostapd_config)

        # Step 3: Start hostapd if available, otherwise try nmcli
        if is_hostapd_available():
            log("[NetworkManager] Starting hostapd...")
            subprocess.Popen(
                ["sudo", "hostapd", hostapd_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(1)
        elif is_nmcli_available():
            # Try NetworkManager hotspot command as a fallback
            try:
                log("[NetworkManager] hostapd not found; attempting nmcli hotspot...")
                subprocess.run(["nmcli", "device", "wifi", "hotspot", "ifname", interface, "ssid", "LUSI"], timeout=5)
                time.sleep(1)
            except Exception as _e:
                log(f"[NetworkManager] nmcli hotspot failed: {_e}", "WARN")
        else:
            log("[NetworkManager] Neither hostapd nor nmcli available; cannot start hotspot", "WARN")

        # Step 4: Create dnsmasq config
        dnsmasq_config = f"""
interface={interface}
dhcp-range=192.168.4.2,192.168.4.20,255.255.255.0,24h
address=/#/192.168.4.1
"""
        dnsmasq_path = "/tmp/dnsmasq_lusi.conf"
        with open(dnsmasq_path, 'w') as f:
            f.write(dnsmasq_config)

        # Step 5: Start dnsmasq if present (optional)
        try:
            log("[NetworkManager] Starting dnsmasq...")
            subprocess.Popen(
                ["sudo", "dnsmasq", "-C", dnsmasq_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            time.sleep(1)
        except Exception:
            # dnsmasq may not be available; fall through and rely on nmcli's built-in DHCP
            pass

        log("[NetworkManager] ✓ LUSI hotspot created successfully")
        log("[NetworkManager]   SSID: LUSI")
        log("[NetworkManager]   Auth: Open (no password)")
        log("[NetworkManager]   IP: 192.168.4.1")
        log("[NetworkManager]   DHCP Range: 192.168.4.2 - 192.168.4.20")

        return True

    except Exception as e:
        log(f"[NetworkManager] ✗ Failed to create hostapd hotspot: {e}", "ERROR")
        return False


def ensure_network_available() -> bool:
    """
    Ensure network connectivity.
    If WiFi not available, create LUSI hotspot.
    Returns True if network is ready (either WiFi or hotspot).
    """
    log("[NetworkManager] Checking network connectivity...")

    if check_internet_connectivity():
        log("[NetworkManager] ✓ Internet connectivity available (WiFi/Ethernet)")
        return True

    log("[NetworkManager] ⚠ No internet connectivity detected")
    log("[NetworkManager] Attempting to create LUSI hotspot fallback...")

    # Prefer hostapd, but fall back to nmcli if hostapd is not available.
    if not is_hostapd_available() and not is_nmcli_available() and not IS_MICROPYTHON:
        log("[NetworkManager] ⚠ Neither hostapd nor nmcli available; cannot create hotspot on this Linux system", "WARN")
        log("[NetworkManager]   Install with: sudo apt-get install hostapd dnsmasq OR install NetworkManager to use nmcli")
        return False

    if setup_lusi_hotspot():
        log("[NetworkManager] ✓ LUSI hotspot is active")
        return True

    log("[NetworkManager] ✗ Failed to create LUSI hotspot", "ERROR")
    return False


if __name__ == "__main__":
    # Test the network manager
    print("Testing Network Manager")
    print(f"Internet available: {check_internet_connectivity()}")
    print(f"hostapd available: {is_hostapd_available()}")
    print(f"WiFi interface: {get_wifi_interface()}")

    # Uncomment to test hotspot creation:
    # ensure_network_available()
