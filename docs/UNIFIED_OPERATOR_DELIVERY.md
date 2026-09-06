# Unified Operator Delivery and Validation

Companion to [the architecture proposal](UNIFIED_OPERATOR_DESIGN.md). September 6, 2026.

Status: proposed work only. No implementation or runtime testing was performed for this document.

## Intended device and connection coverage

The entries below are targets, not claims of tested support.

| Client | Network operation | Nearby Bluetooth | Primary role | Validation required |
| :--- | :--- | :--- | :--- | :--- |
| Windows Electron | Full operator interface | Native adapter | Arm, mission, diagnostics | SpaceMouse model, BLE pairing, gamepad input, installer |
| Linux Electron | Full operator interface | Native adapter | Full base station | BlueZ, device permissions, graphics, gamepad input |
| Steam Deck Electron | Drive interface first | Later optional capability | Driving and camera view | Desktop and Gaming modes, suspend, Steam Input, focus |
| Browser on laptop | Shared UI and monitoring; control after validation | Optional supported browsers only | Observer or module operator | Secure context, input support, focus and throttling |
| macOS Electron | Network interface target | Later validation | Arm or observer | Device adapter, signing, platform permissions |
| Tablet browser | Monitoring first | Not a baseline requirement | Cameras and science results | Touch layout, browser lifecycle, video decoding |

A device may display a module without owning it. BLE motor maintenance is narrower than complete rover operation. The first release does not promise video over BLE or full arm operation over the legacy protocol.

## Operator walkthroughs

### Workshop quick connect

Open the app, select the nearby bridge, verify identity, pair, and view state. Choose Maintenance, request exclusive ownership of the selected motor group, enable, and apply a bounded command. Releasing input requests stop. Exiting maintenance leaves the group disabled.

The old BLE client remains a separate compatibility path until it can participate in the shared authority. It must not run beside a mission controller for the same motors.

### Two operator field session

Connect the Deck and laptop to the base station access point. Select the same rover. Deck requests Drive; laptop requests Arm. Both see ownership labels and link health.

Driver moves to the work location, stops, and satisfies the manipulation interlock. Arm operator enables SpaceMouse control. The driver continues watching but cannot move the base while the arm policy inhibits driving.

Arm returns to a verified permitted configuration. The driver deliberately enables again. No role reassignment or network reconnection is needed for this normal sequence.

### Move from workshop BLE to radio

Stop and disable the current module. Connect the network route and confirm the same rover. The gateway invalidates old ownership, synchronizes state, and offers a new lease. Center controls and enable. Any delayed BLE request from the old epoch is rejected.

### Training without hardware

Select a simulated rover with an unmistakable Simulation label. Deck and laptop join the same simulated session to practice ownership, driving, manipulation, and failure recovery. A keyboard or recorded input source can substitute for unavailable hardware.

## Staged delivery

### Stage 0: inventory and contracts

Confirm actual radio hardware, rover computers, bus topology, motor IDs, firmware, joint measurements, camera feeds, and current startup services. Record which process owns every actuator group.

Specify command units, coordinate frames, limits, ownership, expiry, acknowledgements, module capabilities, and telemetry quality. Document how legacy processes are prevented from bypassing the gateway.

Exit condition: the team can trace one drive command and one arm command from input to actuator and back to measured or estimated state without ambiguity.

### Stage 1: shared UI and discovery

Create the responsive interface and network gateway in monitoring mode. Present capabilities, faults, connection profiles, and ownership placeholders. Reuse useful WaveCan telemetry concepts.

Exit condition: two laptops view the same rover state without ROS installed locally, with no internet connection required. No physical motion path is active.

### Stage 2: desktop device adapters

Package the same UI in Electron. Prove BLE discovery and pairing, native SpaceMouse input, and gamepad input. Add calibration and neutral detection.

Exit condition: device input is visible in diagnostics on Windows and Linux, and the UI identifies missing permissions or unsupported devices accurately. No physical actuation is required.

### Stage 3: isolated simulation and ownership

Implement the simulator backend, module leases, control epochs, handoff, stale input expiry, and shared interlocks. Exercise Deck driving and laptop arm operation together.

Exit condition: conflicting owners and expired commands are rejected, and simulated motion cannot reach any real controller.

### Stage 4: controlled physical integration

Route approved requests through the existing LUSI controllers or selected WaveCan adapters. Replace incompatible startup, stop, and timeout behavior. Commission units and limits one subsystem at a time.

Exit condition: measured motor behavior matches the declared command contract, local stopping works when the gateway disappears, and only one actuator writer exists.

### Stage 5: field network and media

Integrate the actual radio system, media gateway, camera quality selection, and base station topology. Measure worst observed latency, congestion, loss, and recovery rather than relying on advertised link speed.

Exit condition: all motion freshness and camera age criteria pass under representative congestion and radio interruption. Closing the arm laptop does not remove the Deck's network route.

### Stage 6: deployment and additional modules

Package offline installers, known working releases, enrollment, update control, and rollback. Add science and mission actions incrementally with their own ownership and state models.

Exit condition: a fresh supported laptop can join through the documented setup flow without manual ROS launch commands.

## Required validation scenarios

| Scenario | Expected result |
| :--- | :--- |
| Two clients request Drive simultaneously | Exactly one lease is granted |
| A second browser tab sends commands | It cannot inherit ownership implicitly |
| Old owner sends after handoff | Old epoch is rejected |
| BLE and network send conflicting intent | Only the granted owner and active route are accepted |
| Input reader freezes while connection heartbeat continues | Motion expires on input freshness |
| Renderer crashes while native helper survives | Helper cannot preserve stale movement |
| Gateway crashes or reboots | Local controllers expire; reboot creates a new ownership generation |
| Steam overlay, suspend, focus loss, or controller disconnect | Motion stops and deliberate enable is required |
| Stream freezes while network remains healthy | Camera is marked stale using capture age |
| Video saturates the radio | Video quality drops; bounded control latency is preserved or motion expires |
| Public internet is unavailable | Existing field network remains intact |
| Discovered device uses the same friendly name | Identity validation prevents silent selection |
| Arm is extended and driver requests motion | Interlock rejects base movement |
| Legacy ROS or CAN process starts | It cannot become a second actuator writer |
| Stop arrives immediately after nonzero command | Stop is never suppressed |
| Invalid number or wrong units arrive | Request is rejected with a clear reason |
| Simulation is selected on a machine near the rover | No hardware connection or CAN traffic is created |
| Client API version is incompatible | Diagnostics remain available; motion is blocked |

Test watchdog behavior at the downstream controller as well as at the UI. Stop acknowledgement, command expiry, motor output change, and physical stopping are separate observations.

For BLE, measure sustainable command and notification rates with the actual adapters. For Steam Deck, verify analog axes in each launch mode rather than relying on a desktop cursor test. For SpaceMouse, confirm each supported model and transport.

## Open decisions that affect implementation

1. Which radio model and network topology are actually deployed?
2. Can the base station use a dedicated access point or small network appliance?
3. Which rover computer should host the single command authority?
4. Which motors are owned by existing LUSI processes and which by WaveCan?
5. What is the current arm geometry, feedback quality, and permitted drive configuration?
6. Which SpaceMouse models and laptop operating systems must be supported first?
7. Is initial Steam Deck support limited to Desktop mode?
8. What camera resolutions, encoders, and simultaneous streams are required?
9. Which science and autonomy actions already have stable interfaces?
10. Who may grant control, perform maintenance, and reset a global stop?

These questions do not prevent writing the interface specification or building isolated simulation later. They do prevent a credible promise of full hardware compatibility or fixed field latency today.

## Scope of this repository update

Only this document and UNIFIED_OPERATOR_DESIGN.md are added. Existing source, tests, configuration, services, launch scripts, and README remain untouched. No deployment is requested or performed.
