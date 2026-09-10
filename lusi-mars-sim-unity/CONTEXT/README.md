# LUSI Mars Simulator Context Pack

This folder is the durable project handoff for the LUSI Mars simulator. It is intended to let a future worker understand the project without needing the original chat history.

Read these files in order:

1. `PRODUCT.md` for the product intent, boundaries, and simulator experience.
2. `COCKPIT_UI.md` for the approved Steam Deck cockpit design.
3. `MISSION.md` for the Mars driving and science demo.
4. `ENGINEERING.md` for architecture, platform targets, validation, and known gaps.

## Hard constraints

* This is a local only simulator. It must never connect to, discover, command, or depend on real rover hardware.
* The target is the original Steam Deck LCD at 1280 by 800.
* The core use is a polished public demonstration where a player drives a LUSI rover on Mars, visits checkpoints, performs simulated science, and uses a simulated arm.
* The center driving view is a true simulated front camera. It is not a 3D map, a fake static camera card, or an information panel.
* The permanent state is simulation, indicated by a purple status ribbon.
* All scientific readings are simulated and must never claim life detection.

## Repository map

* `Assets/Scripts` contains the Unity runtime systems.
* `Assets/Scenes/MarsSimulator.unity` is the entry scene.
* `Assets/Resources/Mars` contains local terrain assets.
* `Assets/Shaders` contains Mars materials.
* `Assets/Tests` contains edit and play mode coverage.
* `Assets/Editor` contains build and project setup helpers.
* `deck-demo` is the older Electron reference. It is useful for prior UI assets and interaction history but is not the high fidelity simulator.

## Working agreement

When changing the project, preserve the decisions in this context pack unless the project owner explicitly changes them. If a design decision is unclear, pause and ask rather than filling the gap with generic interface patterns.
