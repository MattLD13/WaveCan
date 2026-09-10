# Engineering and Validation Context

## Current architecture

The Unity entry scene should contain the runtime bootstrap. At startup it builds the Mars world, rover visual, camera rig, mission state, Steam Deck input system, and cockpit UI.

Core concepts:

* `LusiSimulatorBootstrap` creates the local runtime systems.
* `MarsWorldBuilder` builds terrain, rocks, lighting, haze, waypoints, and Mars context.
* `RoverVisual` is the simulated rover movement and surface contact model.
* `SteamDeckInput` binds controls and enforces the deadman and held emergency stop.
* `MissionDirector`, `SafeMissionState`, `ScienceSequence`, and `ArmSequence` control the ordered mission.
* The cockpit UI observes simulated state only.

## Security boundary

No simulator code may import or invoke:

* WaveCan runtime controls.
* Bluetooth transports.
* CAN transports.
* ROS.
* Serial ports.
* Network services.
* Real rover commands.

The simulator should be independently buildable and testable from the rest of the WaveCan repository.

## Required local validation

The project must be opened with the Unity version pinned in `ProjectSettings/ProjectVersion.txt`. Before release, a local worker must:

1. Resolve packages from a clean clone.
2. Compile the scene with no errors.
3. Run edit mode and play mode tests.
4. Build a Linux x86 64 player.
5. Run it on the original Steam Deck LCD in Desktop Mode.
6. Check 1280 by 800 layout, controller mapping, emergency stop hold, deadman gating, mission completion, and frame pacing.
7. Capture real screenshots and a short gameplay video.

## Known gaps to resolve

* The former generic side rail cockpit does not meet the approved LUSI layout and is being replaced.
* No current cloud environment has a functioning Unity editor, so compile, render, and device claims require independent local evidence.
* A documentation mismatch previously existed: one document described a full mission while another described free driving only. The actual implementation and documentation must be reconciled before release.
* Do not check generated Unity `Library`, `Temp`, `Obj`, `Build`, `Builds`, `Logs`, `UserSettings`, solution, or project files into source control.
