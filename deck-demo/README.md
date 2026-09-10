# LUSI Rover Ops — Mars Analog 01

`deck-demo/` is a safe, local-only Electron demonstration of a LUSI rover operator cockpit. The central feed is a procedural Three.js Mars terrain; the mission state, map, science procedure, arm macro, and capped drive model never open hardware, radio, CAN, ROS, IPC, or network paths. The purple status ribbon is always SIM.

## Run and test

```bash
npm install
npm start
npm run browser            # browser development server
npm test
npm run test:playwright       # skips when Chromium is unavailable
REQUIRE_BROWSER=1 npm run test:playwright
```

The browser smoke check writes `previews/preview-mars-active-1280x800.png`, `previews/preview-mars-science-1280x800.png`, `previews/preview-mars-assay-1280x800.png`, and `previews/preview-mars-beacon-1280x800.png`. `npm run package:dir` creates an unpacked Linux build; `npm run package:linux` creates an AppImage.

## Steam Deck launch

Copy the project or packaged AppImage to the Deck, open it in Desktop Mode, and run `npm start` (or launch the AppImage). In Gaming Mode, add the AppImage or a small launcher script as a non-Steam game. The viewport is sized for the original 1280×800 LCD and accepts the left stick or WASD, R2 or Space as held deadman, X for camera, B/Esc for stop, and the on-screen hold button for a simulated E-stop. All values and outputs remain local SIM.

## Mission path

Start the mission, drive to the outcrop ring, then use GEO HD and the Science Module in order: classify the rock, collect beyond 10 cm, discard the shallow material through the bridge, verify a load-cell mass above 5 g, capture moisture and temperature, and run hydrogen peroxide → cobalt bicarbonate → blank cuvette → control → 440 nm sample reading. Drive to the field marker, run the visible arm macro, and finish at the final checkpoint. Results are a clearly labeled simulated demonstration only; they are not a life-detection claim.

See [INSTALL.md](INSTALL.md) for packaging notes and [docs/STEAM_DECK_UI_RESEARCH.md](../docs/STEAM_DECK_UI_RESEARCH.md) for design rationale.
