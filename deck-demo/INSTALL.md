# LUSI Rover Ops

LUSI Rover Ops is a simulation-only Electron cockpit for a 1280×800 handheld display. It is deliberately isolated from WaveCan's Python hardware service: it never imports, starts, or sends requests to `main.py`, `web_server.py`, CAN, Bluetooth, or motor drivers.

## Run from source

Use Desktop Mode and install Node.js 20+ (Node 24 is also supported):

```bash
cd deck-demo
npm ci
npm start
```

The window opens at the target of 1280×800. To start the guided route, click **START GUIDED DEMO** or press `H`. Throttle is the forward/reverse speed command; hold `Space` while pressing `W/A/S/D` (or hold `R2` with the left stick) to drive. Touch arrows are available in the Drive Control panel. Releasing the deadman immediately zeros the simulated output. `B` or `Esc` stops; `X` cycles cameras; `L` toggles the simulated link.

## Package an AppImage

```bash
npm ci
npm run package:linux
```

The AppImage is written to `release/`. To use it without system-wide changes, copy it to a user-owned folder such as `~/Applications/` and launch it from there. SteamOS users can add the AppImage as a non-Steam game from Desktop Mode. The app does not modify SteamOS, networking, permissions, or hardware configuration.

## Validation

```bash
npm test
npm run test:playwright
REQUIRE_BROWSER=1 npm run test:playwright
LUSI_SMOKE=1 npm run smoke:electron
```

The Playwright test uses a headless Chromium browser. The Electron smoke test needs a display; CI runs it under Xvfb. If a local machine has no browser or display, install the test browser/display packages rather than treating a timeout as a pass.
