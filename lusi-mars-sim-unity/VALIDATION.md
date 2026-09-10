# LUSI Mars Simulator — Free-Drive Validation

## Current scope

The simulator is currently focused on local free driving. Mission execution remains deferred.

- Unity `6000.6.0f1` with URP, Input System, UGUI, Terrain, and Test Framework packages.
- Procedural foreground regolith mesh plus a matching Terrain collision heightfield.
- Procedural rocks use convex `MeshCollider` components; the rover raycasts the live surface, follows terrain/rock height, and aligns to the hit normal.
- FPV camera is a child of the rover-follow rig and shows the live foreground surface.
- Waypoint pads and beacons are visual-only, so they cannot block surface contact.
- No network, radio, CAN, ROS, serial, Bluetooth, hardware, or rover-control path was found in `Assets/Scripts` or `Assets/Editor`.

## Automated validation

Run from the Unity project directory:

```text
Unity.exe -batchmode -nographics -projectPath . -runTests -testPlatform editmode -testResults logs/editmode-final-free-drive-v2.xml
Unity.exe -batchmode -nographics -projectPath . -runTests -testPlatform playmode -testResults logs/playmode-final-free-drive-v2.xml
```

- EditMode: **4/4 passed** (`logs/editmode-final-textured-guarded.xml`).
- PlayMode: **6/6 passed** (`logs/playmode-final-free-drive-v3.xml`) — startup/profile, deadman gating, terrain motion, climbable rock contact, emergency-stop hold timing, and mission-state ordering.

## Builds

- Windows test player: `Builds/Windows/LusiMarsSimulator.exe`, Mono, `StandaloneWindows64`, build report `105133060` bytes.
- Linux player: `Builds/Linux/LusiMarsSimulator.x86_64`, IL2CPP, `StandaloneLinux64`, build report `974600184` bytes. The Windows-host build explicitly initializes and verifies the Linux x64 sysroot and toolchain before invoking Unity's player build.
- The Linux folder also contains Unity's generated `LusiMarsSimulator_BackUpThisFolder_ButDontShipItWithYourGame`; exclude that folder when staging the runtime for Steam Deck.

Both builds start at the Deck Performance profile (`1280 x 800`, target `30 FPS`). The Windows player was launched successfully with no runtime exception, null-reference, shader, or external-I/O errors in the captured player log.

## Controls

- `WASD` / left stick: drive and steer.
- Hold `Space` / `R2`: deadman; motion is gated until held.
- Arrow keys / right stick: FPV look.
- `C` / `X`: cycle camera.
- `M` / `Y`: toggle minimap detail.
- Hold `Escape` / `B` for 1.2 seconds: simulated emergency stop.
- `E` / `A`: current interaction.

## Manual setup and remaining device validation

Open `Assets/Scenes/MarsSimulator.unity` in Unity and press Play. Configure Steam Input as a standard gamepad before Deck testing. Physical Steam Deck Desktop Mode, the original LCD capture, Deck Performance/Quality FPS measurements, and the full mission acceptance route still require the actual Deck; they were not claimed from this Windows host.

## Surface asset review

The current build stays self-contained while the free candidate is reviewed. The closest Asset Store match found is [Mars Landscape 3D](https://assetstore.unity.com/packages/3d/environments/landscapes/mars-landscape-3d). The supplied [Mars Terrain](https://assetstore.unity.com/packages/3d/environments/landscapes/mars-terrain-139245) reference is not free, so it was not imported.
