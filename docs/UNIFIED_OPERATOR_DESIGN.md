# Unified LUSI Operator Software

Design proposal, September 6, 2026. Documentation only.

## Decision

Build one operator product with a shared web interface, packaged in Electron for Windows and Linux, with browser access for network clients. Retain the existing LUSI ROS 2 Humble control stack behind a rover gateway. Adapt WaveCan's Bluetooth connection and motor diagnostics into that product.

A Steam Deck can own driving while a laptop with a SpaceMouse owns the arm. Other laptops can operate science equipment or observe telemetry. All clients share the same rover identity, module state, faults, and control ownership.

This is feasible as an incremental integration. It does not require rewriting every LUSI module in JavaScript or installing ROS on every operator device. Electron is an operator shell; actuator control remains on the rover.

The first deliverable should unify discovery, connection, monitoring, and isolated simulation. Physical control follows after command ownership and stopping behavior are validated.

See [delivery and validation plan](UNIFIED_OPERATOR_DELIVERY.md) for stages and acceptance conditions.

## Evidence and scope

Source inspection used WaveCan main at commit `1e9f83780bd8a8797de374ced69d1ad084f58574` and LUSI urc_software main at `de2878772a502e48530e1ee5fcfcde7c673bc149`. Links below identify files inspected. No application was built, launched, or connected to motors. Existing documentation claims about physical performance are not new test results.

WaveCan already contains a browser dashboard, a Python HTTP server, SocketCAN support, motor telemetry, and mock motor models. Its separate Windows controller uses Tkinter and Bleak for BLE. These are useful components, but there is no shared Electron application today. [WaveCan architecture](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/docs/ARCHITECTURE.md), [desktop client](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/windows_app/wavecan_controller.py)

WaveCan's BLE bridge starts disarmed and has a local watchdog. Its documentation specifies a 200 ms client heartbeat and 600 ms watchdog. The BLE and web entry points create independent motor runtimes and must not control the same motors together. This is the central integration issue. [Bluetooth design](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/docs/BLUETOOTH.md), [bridge implementation](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/bluetooth_bridge.py)

LUSI already has ROS 2 Humble, Docker, MoveIt Servo, SpaceMouse input, a desktop ground station, video transport, and separate arm and driveline processes. Preserve these useful backend boundaries while replacing operator workflows incrementally. [LUSI repository](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/de2878772a502e48530e1ee5fcfcde7c673bc149/README.md)

## Product experience

The home screen presents rover cards with identity, connection availability, selected route, and module health. The user selects a rover, connects, and chooses Drive, Arm, Science, Cameras, Mission, or Diagnostics.

Remember previously approved devices and connection profiles. Remembering a device permits reconnection and monitoring, not automatic motion enable.

The interface distinguishes Connected, Authorized, Control available, Control owned, Enabled, and Faulted. A Bluetooth connection alone does not mean the CAN interface is healthy or the rover is ready to move.

Present faults in operator language, such as “Arm control held by another operator” or “Camera image is stale.” Put protocol details in diagnostics.

Suggested workspaces:

1. Drive: forward camera, speed, steering, link age, enable state, and stop control.
2. Arm: wrist and overview cameras, gripper pose, active frame, precision speed, SpaceMouse status, and arm ownership.
3. Science: discovered instruments, bounded actions, procedure progress, and results.
4. Cameras: available streams, selected quality, source health, and capture age.
5. Mission: shared task progress, map if positioning exists, annotations, and event log.
6. Diagnostics: motor telemetry, CAN health, device discovery, and approved maintenance actions.

Modules appear according to reported capabilities. A missing module is unavailable, not an empty control panel suggesting it works. Future drone or autonomy modules need their own control policy before being added.

## One interface, two client forms

Use one UI component system and one application protocol in both browser and Electron. Layouts adapt to screen size and selected role. Avoid independent Steam Deck and laptop applications.

Electron packages the UI and a narrow local device service. That service reads SpaceMouse and gamepad input and manages BLE without giving the renderer unrestricted operating system access. Keep context isolation and sandboxing enabled, disable renderer Node integration, validate interprocess requests, and allow only trusted application content. [Electron security guidance](https://www.electronjs.org/docs/latest/tutorial/security)

A browser connects to the rover gateway over the field network. It can provide telemetry, camera views, and validated controls. Direct Bluetooth is an optional capability for supported browsers rather than the universal connection path: Web Bluetooth has limited availability and targets BLE peripherals. [MDN Web Bluetooth](https://developer.mozilla.org/en-US/docs/Web/API/Web_Bluetooth_API)

For Electron BLE, evaluate its device APIs against a packaged native helper. Prefer proving a Bleak based helper first because WaveCan already uses Bleak. This is a prototype choice, not a committed dependency upgrade. WaveCan currently pins an older Bleak major version; current upstream platform support does not establish compatibility of the pinned version. [Electron device access](https://www.electronjs.org/docs/latest/tutorial/devices), [Bleak documentation](https://bleak.readthedocs.io/)

Electron provides packaging consistency but does not make device permissions, drivers, or controller mappings identical across systems.

## Connection modes

### Nearby BLE

The desktop app discovers a WaveCan bridge by its advertised service, verifies rover identity, pairs when necessary, and requests permitted capabilities. The laptop can remain on its existing WiFi connection.

Use BLE for compact commands, small telemetry updates, setup, and local diagnostics. Do not send camera video or large robot assets through the existing BLE protocol.

Initially retain compatibility with the existing motor protocol as an explicitly limited legacy maintenance mode. The current eight byte command has version, operation, motor ID, flags, and one float. It has no ownership token, command sequence, rover boot identity, or Cartesian arm command. Full shared operation requires a new versioned protocol. [Protocol source](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/bluetooth_protocol.py)

A protocol update must define fragmentation, packet ordering, size limits, and recovery for larger BLE messages. Do not assume a larger negotiated MTU is always available.

### Local network

A laptop or browser reaches the rover gateway through Ethernet or WiFi. Discovery can use local service advertisement, with a saved address or QR connection profile when discovery is unavailable.

A field network must work without internet access. WaveCan currently uses internet reachability to decide whether to create a hotspot. Replace that decision with explicit network profiles and local gateway reachability; lack of internet must not disrupt a healthy rover link. Network switching requires an operator action while motion is disabled. [Network manager](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/network_manager.py)

### Long distance base station

The Steam Deck and laptops join a local base station access point or switch. The ground radio connects that network to the rover radio and onboard gateway. Range comes from the radio system, antennas, placement, and environment.

Prefer an IP capable radio link for the common application protocol and video. The installed radio model, topology, usable throughput, and latency remain unverified. A serial only telemetry radio would require a separate adapter and reduced capabilities; it cannot be assumed to carry the same streams.

Keep a small gateway or network appliance at the base station if discovery, routing, or media fanout needs it. The arm laptop should not be a mandatory relay for the Steam Deck. If that laptop closes, driving should not lose its network solely because of the chosen topology.

No cloud account or internet service is required for field operation. Internet remote access is a separate future scope.

### Simulation

The app connects to an isolated simulation gateway with the same capabilities and command contracts. Use distinct identities, endpoints, and transport permissions. A simulated rover must never gain a physical transport by selecting another display tab.

WaveCan's motor mocks can support diagnostic exercises. The full arm trainer needs an actual motion simulation backend. LUSI's component named MockArmHardware publishes physical arm command topics and therefore is not proof of isolation. [LUSI hardware bridge](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/de2878772a502e48530e1ee5fcfcde7c673bc149/src/main_computer_urc/src/MockArmHardware/ArmHardware.cpp)

## Gateway and subsystem architecture

The rover gateway authenticates sessions, advertises capabilities, assigns ownership, validates freshness, and routes accepted intent to subsystem adapters. It does not replace local motor timing or control loops.

Network and BLE frontends feed this same authority. They must not each instantiate competing control runtimes. A standalone WaveCan bench device can host the authority locally. A complete rover can place it on the main computer while downstream controllers enforce their own freshness and ownership checks.

For each actuator group, select exactly one output owner: an existing LUSI controller or a WaveCan controller adapter. Do not run both against the same motor IDs. Direct maintenance control requires a distinct maintenance state that revokes mission ownership.

Preserve ROS within the rover. Operator clients talk to the gateway rather than publishing arbitrary ROS topics. This keeps the UI independent of ROS installation and discovery settings. ROS 2 uses multicast discovery by default; do not assume it will cross every radio or routed field network. [ROS networking documentation](https://docs.ros.org/en/humble/Tutorials/Advanced/Security/Examine-Traffic.html)

The proposed network interface uses HTTPS for discovery, configuration, and snapshots; an authenticated WebSocket for compact control and state events; and a separate media connection. Keep queues bounded and drop superseded motion commands. WebSocket ordered delivery can stall behind lost packets, so deadlines and downstream expiry remain essential. Revisit transport selection if measurements fail the field latency targets.

Use an explicit typed interface rather than a browser accessible generic CAN or ROS publisher. Allowed actions include drive intent, arm twist, gripper request, module action, telemetry subscription, and control release.

## Multiple operators and ownership

Ownership is per module: Drive, Arm including gripper, Science, and controllable camera mounts. Watching a camera requires no motion ownership.

One operator may own several modules. Several operators may own different modules. Two operators cannot command the same module simultaneously.

The gateway grants a short lived lease tied to authenticated session, rover identity, rover boot generation, module, and control epoch. Downstream command acceptance must prevent stale or bypass publishers from defeating this decision.

Handoff sequence:

1. New operator requests the module.
2. Current operator releases, or an authorized supervisor initiates takeover.
3. The module reaches its defined stopped or held state.
4. Gateway invalidates the old epoch and issues a new lease.
5. New operator centers the input and deliberately enables.

A second connection or browser tab never silently steals ownership. Reconnecting a transport does not revive its old lease.

A global software stop is available to authenticated authorized operators even when they do not own the affected module. It latches stop state and revokes motion leases. Reset requires deliberate action and valid state. This command depends on communication and is not a replacement for the physical emergency stop system.

## Steam Deck driving plus laptop arm control

Both devices connect through the base station network to the same rover session. The Deck requests Drive and the laptop requests Arm. The Deck displays the forward camera and drive telemetry; the laptop displays arm cameras and joint feedback.

Default policy: driving is inhibited while the arm is extended for manipulation. The driver sees why movement is blocked. Arm stow status must come from validated state, not a clicked checkbox. Simultaneous base and arm movement should require a separately validated operating mode because it changes reach, stability, and camera relationships.

If the arm laptop disconnects, the arm stops or holds according to its commissioned behavior. Driving may continue only if the rover's shared interlocks permit it. If the gateway fails, all affected modules expire locally.

Use the Deck sticks for proportional drive and steering, a held trigger or bumper for enable, a separate stop binding, and touch or trackpads for menus. Final bindings require operator trials. Do not map a normal Steam menu action to motion.

Steam Input can translate controls into gamepad or desktop input. Verify that the app receives analog gamepad axes, not just mouse emulation, in both Desktop and Gaming modes. Menu overlays, suspend, focus changes, and controller disconnects must invalidate motion input. [Valve input documentation](https://partner.steamgames.com/doc/features/steam_controller/concepts), [Steam Deck desktop FAQ](https://help.steampowered.com/en/faqs/view/671A-4453-E8D2-323C)

Start validation on Windows and Linux laptops, then Steam Deck Desktop mode, then Gaming mode through a Steam shortcut. macOS network monitoring can follow; BLE and SpaceMouse support require explicit testing. Browser availability does not equal full device support.

## Input freshness and link changes

Separate session health, module lease renewal, input freshness, telemetry age, and camera age. A healthy heartbeat must not keep stale joystick motion alive when the input process freezes.

Each motion request needs a bounded validity period, sequence, module identity, session identity, and current ownership epoch. The gateway uses its monotonic clock for expiry. Use negotiated timing and conservative age checks without trusting an arbitrary client wall clock.

A transport change while moving requests a stop and revokes the motion epoch. Connect the new route, verify the same rover, synchronize state, acquire ownership again, center input, and enable. Do not race identical motor commands over BLE and network in the first version.

Detection of a better link may be automatic. Switching active motion control is deliberate. A backup route may carry monitoring or a stop request, but cannot independently become another owner.

Initial laboratory targets are 50 Hz motion intent, 20 Hz telemetry, and a provisional 250 ms stale input cutoff. These are proposed experiments, not approved field limits. WaveCan's existing 600 ms BLE watchdog is a separate observed setting. Choose deployed values from measured loss, latency, controller stopping response, and application needs.

Application acceptance latency and physical stopping time are different measurements. An acknowledged stop request does not establish that every actuator stopped.

## Cameras and bandwidth

Convert existing ROS camera output at a media gateway. WebRTC is a candidate for browser compatible low latency video, but codec support and encoder capacity need tests. Do not route video through the motor command channel.

Subscribe only to needed cameras. Begin with one primary stream per operator. A provisional two stream budget of 2 Mbit/s each plus 30 percent headroom requires roughly 5.2 Mbit/s usable capacity before extra streams or retransmission effects. This is a planning example, not a measured radio requirement.

Where several clients watch the same stream, consider base station fanout so the radio carries one copy. Report camera capture age separately from network round trip time. Freeze detection and stale image warnings must not depend on the picture visibly changing.

Prioritize motion, faults, and essential state over video quality. Adapt resolution and frame rate before allowing video to exhaust the link. Driving with an unavailable required camera should follow an explicit operating policy.

## Security and deployment boundaries

The existing web server has no application authentication or TLS, and the current BLE pairing model is not proof of trusted operator identity. Add application enrollment and role authorization before shared control. Discovery names are hints; confirm persistent rover identity during enrollment.

Support an offline enrollment step using a locally obtained one time credential or physical confirmation. Store desktop credentials in the operating system credential store. Browser sessions need restrictive origins and scoped credentials. A local native helper must bind locally and authenticate its own UI connection.

Bundle the UI and assets for offline use. Version the client API, transport protocol, module capabilities, input profiles, and robot model independently. An incompatible major version permits diagnostics but blocks motion.

Installers should make drivers and permissions visible during setup. Do not require running the whole desktop application as administrator. SteamOS packaging and native helper permissions are acceptance gates, not assumed solved by Electron.

No automatic update during an active mission. Retain a known working release and support rollback while stopped.

## What must change before a unified physical trial

1. Put BLE and network requests behind one authority and one actuator owner.
2. Replace startup enable behavior in the old web runtime with explicit enable.
3. Make stop and neutral authoritative; remove the old HTTP path's suppression of some rapid zero commands from the new mission path.
4. Tie continued movement to fresh operator input, not merely process heartbeat.
5. Enforce module leases below the UI and block competing legacy ROS or CAN writers.
6. Resolve units and mappings in LUSI adapters. The inspected DriveTrainManager reads angular.y and applies a degrees conversion, so standard yaw in radians must not be forwarded blindly.
7. Repair the LUSI all motor loss of signal path before coordinated control.
8. Establish a truly isolated simulator and validated arm geometry.

These are source grounded integration tasks, not changes made by this proposal. [WaveCan HTTP server](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/web_server.py), [WaveCan startup](https://github.com/MattLD13/WaveCan/blob/1e9f83780bd8a8797de374ced69d1ad084f58574/main.py), [LUSI drive mapping](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/de2878772a502e48530e1ee5fcfcde7c673bc149/src/main_computer_urc/src/DriveTrainManager/main.cpp), [LUSI motor manager](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/de2878772a502e48530e1ee5fcfcde7c673bc149/src/shared_code/MotorManager.cpp)

## Repository strategy

Keep this proposal in WaveCan as the integration design. Future UI and connection work can begin here. Keep the existing ROS control packages in urc_software until an explicit team decision changes ownership. A shared versioned API can unify the product without immediately combining repositories.

Do not copy old private team documents or credentials into this repository. This proposal references public code and documentation only. The requested scope includes documentation, not application implementation, service changes, or deployment.
