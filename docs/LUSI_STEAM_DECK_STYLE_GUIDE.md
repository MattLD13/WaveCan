# LUSI Rover Ops — handheld mission-console style guide

Status: design direction for the simulation-only deck-demo cockpit.  
Target: 1280×800 LCD, touch plus gamepad/keyboard input, readable at arm’s length.

## Identity and scope

Use **LUSI** as the visible product identity. The app title and release product name are **LUSI Rover Ops**. “Rover Ops,” “LUSI Vision,” “Mission Phase,” and “Sim Only” are preferred UI terms. Do not expose legacy Luna branding or use “Deck” as product branding.

LUSI’s current public urc_software GUI and urcAssets/LUSNoDownlink.png are references for information architecture and fault language. The app bundles the no-downlink image at assets/LUSNoDownlink.png and shows it only for the explicit lost-link state; it is not used as a generic product logo. Preserve the repository license/attribution and obtain permission for any trademark use. The normal simulated feed and compact mark remain original.

The app is a simulation boundary. It must not import, start, or call WaveCan hardware, CAN, Bluetooth, motor, or network services. A future adapter can replace the simulation state without changing the renderer or input contract.

## Design intent

The handheld is an engineering instrument carried to the vehicle, not a racing-game HUD. The operator should see what the rover sees, understand the current mission phase, issue one constrained command, and recover from a fault without hunting through windows.

The hierarchy is:

1. Central live camera feed and vehicle identity.
2. Current phase, link/watchdog/fault state, and recovery action.
3. Compact drive command status and explicit hold-to-enable behavior.
4. Telemetry, map position, event/checkpoint history, and guided-demo progress.

The X post [“never expected =rand() would be so valuable”](https://x.com/p_millerd/status/2094961114971083229) is a reminder to cut low-value decoration, introduce intentional variety, remove generic AI styling, and rewrite labels by hand. Apply that as information editing: short verbs, purposeful panels, and no ornamental sci-fi chrome.

## Layout rules

- Design to the full 1280×800 screen and reclaim space from decorative chrome. Use layered composition: the 2D camera feed is the base layer, safety and camera cues are small overlays, and controls are a compact utility layer.
- Keep the live feed visually dominant and centered in the full composition. Prefer a 16:9 feed that uses as much of the display as practical, with a narrow utility rail or overlay; do not let a control rail become the primary surface.
- The drive demo feed is a 2D view from a front mounted rover camera. It is not a 3D rover scene or a free camera visualizer.
- Put a five pixel high module and phase status ribbon directly below the top bar and run it across the full screen width. It is a quick status scan, not a detailed status panel. Give each connected module a stable color and show the current phase through segment color or a small visor marker. Put expanded labels and explanations in a focused overlay or utility view.
- Put map, link/watchdog, and stop/recovery state in a slim rail or edge HUD.
- Keep telemetry and mission phase in a bottom ribbon directly associated with the feed.
- Every panel must answer one operator question: “What do I see?”, “Where am I?”, “What command is active?”, or “What failed/how do I recover?”
- Touch targets should be generous; no critical state may rely on hover.
- Keep every-device/every-module future paths in mind: camera selection, drive, arm, science, autonomy, map, and diagnostics should be separable action sets with shared status primitives.

## Drive model

Use one simple control model:

- **Throttle** is the forward/reverse speed command. It is not a score, boost, or racing meter.
- **Turn** is the heading command.
- Show both as small signed command readouts with thin bars.
- Put “THROTTLE = SPEED / FORWARD–REVERSE” beside the control, and show “HOLD TO ENABLE” until the deadman is held.
- Space/Shift, R2, or a touch direction acts as the deadman. Release immediately commands zero.
- Stop/Esc/B latches or clears the safety stop as defined by the input contract.
- Camera, map, and demo actions must never move the rover.

## Visual language

Use an industrial mission-console skin: deep blue-green background, flat dark panels, thin cyan/blue-green dividers, restrained state color, and dense but legible typography. The NASA mission-operations pattern is the source of the hierarchy: mission phase, vehicle identity, prioritized telemetry, explicit faults, constrained actions, recovery guidance, and event/checkpoint history.

Conceptual references to *The Martian* and *Project Hail Mary* mean disciplined checklists, resource-aware decisions, and calm recovery under pressure. SpaceX software is an industrial-console inspiration for hierarchy and command confirmation only. Do not copy film graphics, dialogue, proprietary SpaceX visuals, or recognizable layouts.

| Token | Use |
| --- | --- |
| #071019 / #0f2029 | Shell and panel surfaces |
| #21404b | Dividers and inactive grid |
| #61e7ef | Link, selected camera, active input |
| #7bea9a | Healthy/complete/watchdog state |
| #f6bd60 | Mission cue, pending action, telemetry emphasis |
| #ff6b6b | Fault, stop, lost link |

The module and phase strip uses these semantic states:

Normal quick-scan segments are distinct and stable: LUSI Vision cyan, Drive command amber, Comms green, Watchdog violet, and Simulation route blue. A lost Comms state changes its segment to yellow; a disconnected Comms state changes it to pulsing red. A watchdog fault changes its segment to pulsing red. Expanded labels live in the utility rail or focused overlay, not in the five-pixel ribbon.

| State | Visual treatment | Meaning |
| --- | --- | --- |
| Healthy connection | Stable green or approved LUSI healthy color | Fresh telemetry and command path available |
| Active module | Module color with a brighter edge or restrained pulse | Current operator focus or ownership |
| Disconnected | Pulsing red visor marker and `DISCONNECTED` | No usable connection; output is zero |
| Communications lost | Yellow visor marker and `COMMS LOST` | Device is known but fresh telemetry is missing |
| Bluetooth mode | Blue visor marker and `BLUETOOTH` | Nearby Bluetooth route is selected |
| Long distance/base station | Approved route color and route label | Base station radio route is selected |
| Simulation | Amber `SIM ONLY` marker | No physical route is available |

Visor markers are compact status shapes, not decorative glow. Pulsing is reserved for attention states and stops when the state clears. Every color is paired with a text label for accessibility.

Pair color with text (“LINK LOST”, “OUTPUT ZEROED”, “DEADMAN ON”). Use 9–10 px uppercase eyebrows, 13–15 px labels, 16–20 px primary values, and monospace only for command/status values.

## Safety and feedback components

- Header: LUSI identity, “SIM ONLY”, mission name/phase, link latency, battery.
- Feed: camera name, feed age/frame rate, crosshair, clearance/context tag, and a no-downlink treatment.
- Rail: small course map, drive command card, simulated-link toggle, watchdog interval, and one high-contrast stop button.
- Ribbon: speed, heading, distance, mission steps, and event/checkpoint text.
- Fault state: red status dot plus words, zeroed command bars, actionable “CLEAR STOP” or link recovery state.
- Command confirmation: show the command value and the safety gate (“HOLD TO ENABLE”) in the same visual neighborhood.

## Inspiration translated into behavior

Disney’s public use of handheld PCs for animatronic testing demonstrates the workflow value of a portable computer with physical controls, touchscreen, live status, and an operator who can move around the machine. It does not establish Disney’s exact software mapping; adapt the workflow, not its branding. See [Disneyland’s Steam Deck animatronic report](https://gamerant.com/steam-deck-disneyland-star-wars-droids/) and [the 2026 Olaf overview](https://www.digitalcitizen.life/disney-uses-a-steam-deck-to-control-an-olaf-animatronic/).

LUSI’s public [URC software repository](https://github.com/Lehigh-University-Space-Initiative/urc_software) informs the camera set (Front/Back/Bottom/Arm), telemetry families, simulation/HOOTL boundary, and explicit no-downlink state. The adaptation consolidates independent panels into one bounded feed-first cockpit.

For handheld constraints, [Valve’s technical details](https://www.steamdeck.com/en/tech/deck), [hardware overview](https://www.steamdeck.com/en/hardware), and [Steam Input API](https://partner.steamgames.com/doc/api/isteaminput) support controller-first input, analog triggers/sticks, grip-button reach, 60 Hz rendering, and separate Drive/Menu action sets. [NASA mission control](https://www.nasa.gov/mission_pages/station/structure/elements/mission_control.html) and [Artemis I resources](https://www.nasa.gov/specials/artemis-i/) support phase/status/checklist thinking; these are public references, not copied UI sources.

The hardware mockup uses the original LCD/512 GB day-one front silhouette from Valve’s [official Steamworks Steam Deck SVG line art](https://partner.steamgames.com/doc/steamhardware/steamdeck/svg) as the reference. The app’s screen content remains original LUSI UI; no OLED-specific silhouette is implied.

## Do / don’t

| Do | Don’t |
| --- | --- |
| Center the feed and make it the first read. | Make a map or meter the dominant surface. |
| Explain throttle as speed in plain language. | Present throttle like a racing-game boost gauge. |
| Show “HOLD TO ENABLE”, watchdog, and zeroed output. | Hide safety behavior in a tooltip. |
| Use a narrow rail and bottom ribbon. | Use a wall of equal-size floating cards. |
| Use original feed visuals and labels. | Copy LUSI, NASA, Disney, film, or SpaceX graphics. |
| Keep simulation state visibly labeled. | Imply a hardware connection or live downlink. |
| Confirm command value and recovery action together. | Require the operator to infer what a button will do. |

## Preview review checklist

- At 1280×800, the feed is centered, 16:9-ish, and visually dominant.
- LUSI is the only visible product identity; no legacy Luna or product-level Deck naming appears.
- Camera, map, drive, telemetry, mission phase, link, watchdog, and stop state are all legible without scrolling.
- Throttle’s purpose is obvious in one glance; its visual weight is subordinate to the feed.
- Normal, drive-active, arm-camera, link-lost, and safety-stop previews show textual state plus color.
- Touch buttons are reachable and release-to-zero behavior is clear.
- The screen remains useful in a bright field context: high contrast, no hover-only meaning, no tiny critical controls.
- Preview images, docs, package metadata, and smoke-test title all agree on **LUSI Rover Ops**.
