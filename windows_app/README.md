# WaveCan Windows controller

This small Windows desktop application connects directly to the Raspberry Pi over Bluetooth Low Energy. It does not create a hotspot or alter the laptop's Wi-Fi connection.

## Run from source

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements.txt
python wavecan_controller.py
```

Run the commands from the `windows_app` directory while the repository root is on `PYTHONPATH`, or use `build.ps1` to create a standalone executable with the correct import path.

## Build the executable

```powershell
.\build.ps1
```

The resulting application is `dist\WaveCanController.exe`.

## Use

1. Start and verify `wavecan-bluetooth.service` on the Pi.
2. Open the app and select **Connect**. Windows may display a Bluetooth pairing prompt the first time.
3. Select **ARM** only after the mechanism is clear.
4. Choose a motor ID and apply an output or RPM target.
5. Use **Disarm** or **EMERGENCY STOP** before disconnecting.

While armed, the app transmits a heartbeat every 200 ms. The Pi stops all motors if it receives no command or heartbeat for 600 ms.
