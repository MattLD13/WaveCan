# WaveCan documentation index

WaveCan is a live Linux SocketCAN application.

| Area | Purpose | Main files |
| --- | --- | --- |
| SPARK MAX CAN package | Frame encoding, live SocketCAN transport, hardware control, and telemetry decoding | `sparkmax_can/` |
| Application and dashboard | Live startup, HTTP API, control/telemetry loop, and browser UI | `main.py`, `config.py`, `web_server.py`, `dashboard.html`, `wavecan_platform.py` |
| Diagnostics | CAN/API investigation tools; hardware scripts can affect attached motors | `direct_rpm_probe.py`, `remote_pid_probe.py`, `verify_wavecan_remote.py`, live `test_*.py` scripts |

Root modules `rev_sparkmax_protocol.py`, `socketcan_bus.py`, and `hardware_motor_controller.py` are compatibility imports for older scripts. New code should import from `sparkmax_can`.

`windows_app/` contains generated `.venv`, `build`, and `dist` output only; no Windows application source was found. `*.bak` files are backups, not active runtime sources.

## Broader workspace

A separate folder under `Documents/Codex/2026-04-26/can-you-connect-to-a-device/` contains an older WaveCan copy, a separate C++ `sparkcan-ref` library, and hardware investigation scripts. See the [workspace map](WORKSPACE_MAP.md). Some legacy remote patch scripts there contain embedded credentials; do not share that folder as-is.
