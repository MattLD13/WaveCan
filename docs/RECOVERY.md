# Raspberry Pi recovery notes

## Scope

This branch recovers the readable WaveCan working tree from `/home/pi/WaveCan` on host `wavecan-pi` and overlays it on the healthy `MattLD13/WaveCan` GitHub history.

The Pi's `.git` directory was not reused. Six loose objects were zero bytes, ten incomplete temporary pack files were present, `HEAD` was invalid locally, and `git fsck` reported missing blobs. GitHub still had the referenced `main` commit, so the published history remained recoverable.

## Recovered changes

Compared with the last healthy GitHub commit, the Pi contained substantive edits to:

- `config.py`
- `dashboard.html`
- `main.py`
- `web_server.py`

It also contained these previously untracked application files:

- `INSTALL.md`
- `network_manager.py`
- `QUICKSTART.md`
- `setup.sh`
- `wavecan.service`

Additional authored files outside the main checkout were recovered into this repository:

- `/home/pi/start_wavecan_remote.sh` to `tools/start_wavecan_remote.sh`
- `/home/pi/sparkcan-ref/sparkcan_ref_probe.cpp` to `tools/sparkcan_ref_probe.cpp`
- `/etc/systemd/system/setup-can.service` to `systemd/setup-can.service`

The full working tree and relevant VS Code local history were archived separately before cleanup. The upstream `grayson-arendt/sparkcan` checkout was not copied because it is a separate dependency; only the locally authored probe source was retained.

## Intentionally excluded

- Corrupt `.git` metadata and incomplete object packs
- `wavecan-main.log` and `/tmp/wavecan.log`
- Python and pytest caches
- Compiled probe binary and upstream build directory
- `.vscode` and `.claude/settings.local.json`
- `.bak` copies already represented by source/history
- `/home/pi/.codex`, `/home/pi/.wpa_cli_history`, NetworkManager profiles, and all credential stores

A filename/content scan found no embedded private keys, GitHub/AWS tokens, credential URLs, or literal passwords in the publishable source. Wi-Fi password fields in the code are runtime inputs, not stored credentials.

## Dependencies

Declared development/runtime dependencies are:

- `pytest>=7.0`
- `pytest-asyncio>=0.21`
- `python-can>=4.3`
- `requests>=2.31`

`requests` was added during recovery because the real-motor diagnostics import it and pytest could not otherwise collect the complete suite. MicroPython paths reference `adafruit_mcp2515` and `sparkmax_ctr`. There is no lockfile.

## Test baseline

A clean recovery run under Python 3.13.1 now produces 24 passing and 10 failing tests, including seven passing Bluetooth protocol tests. One root-level async test is not marked/configured for `pytest-asyncio`. The remaining failures show disagreements between current REV frame constants/decoding behavior and older mock/controller test expectations.

A mock-mode smoke test successfully started the HTTP service with six simulated motors and returned `status: ok` from `/api/health`. Windows consoles require UTF-8 mode for the current Unicode log symbols.

Run the suite in a clean environment with `requirements-dev.txt` before changing protocol behavior. Hardware behavior should be validated with motors mechanically safe and unloaded.

## Repository hygiene

The recovery removes tracked runtime logs, backup copies, and local editor permissions. `.gitignore` now excludes common secret, database, log, backup, cache, and virtual-environment artifacts.

No project license or dependency lockfile existed at recovery time.
