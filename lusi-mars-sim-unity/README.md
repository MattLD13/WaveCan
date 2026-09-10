# LUSI Mars Simulator

`lusi-mars-sim-unity` is the high-fidelity, local-only Unity path for the LUSI operator simulator. It is intentionally independent from the WaveCan runtime and from `deck-demo`: the project contains only deterministic simulated state, generated terrain, and local input.

## Baseline

- Unity `6000.6.0f1` (Unity 6)
- Active profile: `Deck Performance`
- Reference display: `1280 × 800`
- Target: `30 FPS`
- Scene: `Assets/Scenes/MarsSimulator.unity`

Open the project in Unity Hub, allow Package Manager to resolve the listed packages, open the scene, and press Play. `LusiSimulatorBootstrap` builds the world, rover, camera, mission model, and cockpit at runtime.

The center of the display is always the simulated FPV camera. UI is confined to the five-pixel SIM ribbon, side rails, and bottom status strip. All measurements and science results are explicitly simulated; the science sequence never reports life detection.

## Controls

| Control | Action |
| --- | --- |
| Left stick | Drive and steer |
| Right stick | Camera look |
| R2 | Deadman; required for motion |
| A | Current parked science or arm action |
| X | Cycle camera |
| Y | Toggle minimap detail |
| Hold B for 1.2 s | Simulated emergency stop |
| W A S D | Keyboard drive fallback |
| Arrow keys | Keyboard camera look fallback |
| Space | Keyboard deadman fallback |
| E / C / M / Escape / R | Action / camera / minimap / emergency stop / reset |

## Validation

EditMode and PlayMode tests are under `Assets/Tests`. They cover startup, deadman gating, emergency-stop hold time, science and arm order, and checkpoint completion. The source boundary is deliberately small: there are no project dependencies or code paths for external transport, devices, or hardware.
