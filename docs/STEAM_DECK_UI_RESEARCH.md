# LUSI Rover Ops operator UI research and design

Date: 2026-09-07. Scope: simulation and UI design only. LUSI Rover Ops does not drive WaveCan hardware.

## Evidence reviewed

- [Valve Steam Deck LCD technical details](https://www.steamdeck.com/en/tech/deck) specifies 1280×800, a 7-inch optically bonded IPS LCD, 400-nit typical brightness, 60 Hz refresh, touch, two analog sticks, analog triggers, four assignable grip buttons, and SteamOS 3. The design therefore uses a fixed 1280×800 target, large contrasty labels, and no hover-only interaction.
- [Valve Steam Deck hardware overview](https://www.steamdeck.com/en/hardware) describes the device as suitable for extended sessions, calls out analog triggers for driving games, and explains that the four rear grip buttons keep the thumb on a stick or trackpad. LUSI reserves a reachable rear-button mapping for a future transport adapter; the demo exposes equivalent keyboard/touch fallbacks.
- [Valve Steam Input API](https://partner.steamgames.com/doc/api/isteaminput) documents action sets such as Menu, Walk, and Drive and warns that handheld gyro data is oriented to the hardware, often requiring a 90-degree pitch counter-rotation. LUSI keeps Drive and Menu actions conceptually separate and does not use gyro in this first driving slice.
- [Blue Robotics: Using a Handheld PC to Control the BlueROV2](https://bluerobotics.com/learn/using-a-handheld-pc-to-control-the-bluerov2/) is a vendor-authored, tested Steam Deck field setup. It reports that handhelds are useful when a full-size laptop is inconvenient, recommends a long USB-C tether cable, and says Steam Deck desktop mode gives the best Cockpit feature support because some touch controls are not fully supported in game mode. That supports a controller-first, touch-assisted cockpit with no dependence on tiny touch targets.
- [Foxglove: Teleoperating LeKiwi from a Steam Deck](https://foxglove.dev/blog/teleoperating-the-lekiwi-from-a-steam-deck) is a firsthand ROS 2 demonstration using the built-in controller and Foxglove visualization on one handheld. It reports a ROS bridge on the robot and simultaneous driving/monitoring. This is a community tutorial rather than a URC or Valve product specification, so it informs the “camera/map plus command” composition but is not evidence of competition requirements.
- [Pittsburgh Robotics URC repository](https://github.com/pitt-rover-project/URC) is a public team software stack. Its README advertises simulation mode and separates teleoperation, autonomy, camera, and bridge code. The repository’s `guis/` directory includes separate equipment, delivery, autonomous, camera, and motor GUI modules. This is a URC team implementation reference, not a usability study.
- [Wisconsin Robotics GUI repository](https://github.com/WisconsinRobotics/GUI) describes a resizable four-panel operator UI: control, console, camera, and map. Its README explicitly says even a HUD would be a large improvement over reading diagnostics one at a time. The current GUI is desktop/web oriented, so LUSI keeps the same information families while consolidating them for 1280×800 handheld use.
- [Wisconsin Robotics WRoverSoftware repository](https://github.com/WisconsinRobotics/WRoverSoftware) documents separate base station, IK solver, and rover station processes. That separation reinforces keeping transport, simulation, and presentation behind replaceable interfaces.
- [SHC-ASTRA basestation-game](https://github.com/SHC-ASTRA/basestation-game) is a public Godot rover control system with Core, Arm, Autonomy, Drone, Science, and Debug tabs. Its Core tab includes drive mode, motor bars, brake indicator, speed limit, PTZ controls, heading, and camera-oriented controls. It is useful evidence for modular task surfaces and visible braking state, but it is a team-specific Godot UI and is not Steam Deck-specific.
- [Lehigh University Space Initiative `urc_software`](https://github.com/Lehigh-University-Space-Initiative/urc_software), inspected at commit `de2878772a502e48530e1ee5fcfcde7c673bc149` (2026-08-23), is ROS 2 Humble C++/ImGui. Its README describes a base station GUI, joystick/SpaceMouse input, video display, telemetry, and simulation/HOOTL mode. `src/base_station_urc/src/gui/guiMain.cpp` loads `ComStatusPanel`, `TelemetryPanel`, `SystemControlPanel`, and `VideoViewPanel` and renders at 60 Hz. `VideoViewPanel.cpp` provides Front/Back/Bottom/Arm camera buttons, resizes images to 960×540, and substitutes a no-downlink image after three seconds. `TelemetryPanel.cpp` shows left/right three-wheel command bars, velocity bars, and a joystick-swap checkbox. `SystemControlPanel.cpp` has motor freeze, Drive/Arm mode, reboot/deploy/close, and LUSI vision controls. LUSI Rover Ops adapts this functional inventory into one bounded cockpit: the feed is central, the right rail is compact, and watchdog/fault state remains persistent.

## Additional design references and adaptation notes

- Paul Millerd’s [X post](https://x.com/p_millerd/status/2094961114971083229) shows an AI-generated spreadsheet and the caption “never expected =rand() would be so valuable.” The useful design lesson is the accompanying critique: introduce variety, cut elements that do not add value, remove “AI tells,” and rewrite copy by hand. LUSI applies that as deliberate information editing: one feed-first composition, short operator verbs, and no decorative sci-fi chrome.
- Disney’s public use of Steam Decks to operate and test animatronics is documented in [this report](https://gamerant.com/steam-deck-disneyland-star-wars-droids/) and [this 2026 overview](https://www.digitalcitizen.life/disney-uses-a-steam-deck-to-control-an-olaf-animatronic/). The reports do not establish Disney’s exact software or control mapping. They do support the practical pattern of a portable, general-purpose Linux computer with physical controls, touchscreen, live status, and a nearby operator who needs immediate feedback. LUSI uses that pattern while keeping the UI and visuals original.
- The LUSI no-downlink asset (`urcAssets/LUSNoDownlink.png`) and the Lehigh GUI source are treated as reference material, not bundled branding. The app keeps its own simulated camera renderer and uses text states such as `LINK LOST` and `OUTPUT ZEROED` so no third-party logo or image is implied.
- NASA public mission-operations material, including [NASA’s mission control overview](https://www.nasa.gov/mission_pages/station/structure/elements/mission_control.html) and [Artemis I mission resources](https://www.nasa.gov/specials/artemis-i/), motivates the hierarchy: mission phase first, vehicle identity, prioritized telemetry, explicit fault visibility, constrained commands, recovery guidance, and event/checkpoint history. *The Martian* and *Project Hail Mary* are conceptual references for disciplined checklists and resource-aware problem solving; no film graphics or dialogue are reproduced. SpaceX software is likewise an industrial-console inspiration only; no proprietary visuals or layouts are copied.

## Implementable 1280×800 specification

The screen is divided into a 64 px header, a 690 px cockpit area, and a 46 px footer. The cockpit uses 18 px outer margins and a 14 px gutter:

| Region | Size | Purpose |
| --- | ---: | --- |
| Header | 64 px | LUSI identity, `SIM ONLY` badge, mission phase, link/latency, battery |
| Camera workspace | flexible left column | Central 30 FPS simulated LUSI Vision canvas with crosshair, clearance tag, feed age, camera selector |
| Course map | 220 px right column | Course path, rover pose, checkpoint and compact position readout |
| Drive control | compact right rail | Throttle/turn command status, explicit throttle purpose, touch directional pad, deadman state |
| Safety/link | 155 px right column | Simulated link toggle, command watchdog, latched stop, event line |
| Telemetry/demo | 140 px below camera | speed, heading, distance and four-step guided demo |
| Footer | 46 px | Always-visible A/B/X/Y/ESC and hold-to-enable hints |

Typography uses a dark blue-green background, high-contrast cyan/green/amber/red state colors, 9–10 px eyebrow labels, and 15–20 px primary values. Color is paired with text (`LINK LOST`, `SAFETY STOP`, `DEADMAN ON`) so state does not rely on hue alone. The camera and map are canvases to keep rendering cheap and deterministic on a 60 Hz LCD.

## Input contract

| Input | Action | Safety behavior |
| --- | --- | --- |
| Left stick or WASD/arrows | throttle + turn | Output is accepted only while `R2` or Space/Shift is held |
| Touch arrows | short drive command | Pointer release clears touch deadman and zeros output |
| A / H / demo button | Start guided demo | Does not move the rover |
| B / Esc / Stop | Stop | Clears command immediately; Stop button latches until cleared |
| X / camera chips | Cycle/select camera | Advances guided demo camera step |
| L | Toggle simulated link | Link loss calls stop and displays zeroed output |
| R1/L1 | Reserved for future camera/map action sets | No hidden behavior in this slice |
| Y | Reserved for future map focus action | No hidden behavior in this slice |

The simulation updates at 30 Hz for input polling and requestAnimationFrame for visuals. The model clamps throttle/turn to [-1, 1], speed to ±0.8 m/s, pose to course bounds, and battery to [0, 100]. A real adapter should publish commands at a bounded watchdog interval and treat stale link packets as zero output.

## Modularity boundary

`src/sim/SimulationEngine.cjs` owns deterministic state and physics. `src/input/InputController.js` merges keyboard, touch, and Gamepad API inputs. `src/ui/DashboardRenderer.js` owns canvas and DOM presentation. `src/main.js` is the composition root. A future ROS/WaveCan adapter should implement the same command/status boundary and leave renderer and input modules unchanged. Arm, science, and autonomy surfaces are intentionally out of scope for this demo; they should become separate action-set modules once their message contracts are defined.
