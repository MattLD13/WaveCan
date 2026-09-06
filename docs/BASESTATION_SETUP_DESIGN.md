# Base Station Setup and Usability

Documentation proposal, September 6, 2026. Companion to [unified operator design](UNIFIED_OPERATOR_DESIGN.md) and [validation plan](UNIFIED_OPERATOR_DELIVERY.md).

## Outcome

Make normal operation: power the station, join its network, open WaveCan, choose the rover, pass readiness checks, and request a module. Every supported client can operate every available module. Device appropriate inputs change the layout, not functionality.

Keep developer installation separate from operator setup. A normal operator should not need to clone repositories, build ROS packages, configure X11, or identify Linux input event numbers.

This review inspected source and documentation only. No radio, workstation, or rover was configured or tested. Findings about the current supported workflow are not universal claims about the capabilities of WSL or a particular operating system.

## Verified setup friction

### Developer environment is part of the current operator workflow

The repository documents Docker and git submodule setup. The Windows guide adds WSL, Docker configuration, a workspace build, and display setup. This is useful for developers but too much repeated work for every operator device. [Repository overview](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/README.md), [Windows setup guide](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/WSL_SETUP.md)

Proposed change: distribute prebuilt versioned backend images and desktop installers. Browser clients need only an enrolled connection to the gateway. Retain WSL and source builds as optional developer tools.

### Display and hardware access are coupled

The base launch script uses Docker host networking, host IPC and PID namespaces, X11 socket forwarding, and input and USB mounts. These requirements should not apply to clients that only need a network connection or touchscreen controls. [Base launcher](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/src/base_station_urc/launch/launchScript.sh)

Proposed change: keep backend processes headless and make local peripheral access an optional desktop helper. Avoid broad host privileges for the UI. Request only the device access actually required.

### Driving assumes two joystick topics

The launch starts joystick device IDs 0 and 1 and starts the SpaceMouse node unconditionally. JoyMapper returns without publishing until both cached joystick messages contain axes. It derives driving from their second axes and publishes its existing coordinate convention. [Base launch](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/src/base_station_urc/launch/base_station_launch.py), [JoyMapper](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/src/base_station_urc/src/joyMapper/joyMapper.cpp)

Proposed change: touch, gamepad, keyboard, and SpaceMouse adapters produce canonical intent independently. The gateway converts into the existing backend convention after ownership and freshness checks. Do not require synthetic second joysticks. Add input calibration and actual device identity selection rather than fixed enumeration order.

### Radio setup remains outside the verified software description

Inspected documentation and deployment configuration identify rover network targets, but do not establish the installed radio model, antenna requirements, routing, or field multicast behavior. Host networking alone does not configure the radio link. [Deployment script](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/softwareUpdate/urc_deploy.py)

Proposed change: maintain an explicit field profile containing discovered or commissioned gateway addresses, interface selection, connection method, and expected services. Keep credentials outside repository configuration and exclude them from support exports.

### Camera discovery is fragile

The reviewed video implementation includes numeric camera selections and separate transport paths. Device enumeration and the presence of an open port are not sufficient evidence that the correct fresh image reaches a client. [Video source](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/src/main_computer_urc/src/VideoStreamer/VideoStreamer.cpp), [vision transport](https://github.com/Lehigh-University-Space-Initiative/urc_software/blob/main/src/base_station_urc/src/LUSIVisionStreamer/main.cpp)

Proposed change: expose named camera capabilities with source identity, capture time, availability, quality, and client decode status. Browser compatible media should be provided by a defined adapter, not assumed from the current desktop stream.

## Proposed station arrangement

Prefer a stable field access point or switch connected to the ground radio. The iPad, Deck, and laptops join that local network. A small optional station computer can host discovery, browser assets, diagnostics, and media fanout if the existing network equipment cannot provide them.

The onboard gateway remains authoritative for control ownership. The station computer does not get to restore movement after a rover gateway restart.

Do not make the arm laptop a mandatory network relay. Closing one operator laptop should only affect its leased modules and any applicable shared interlocks, not disconnect every other client.

Confirm existing hardware before buying or redesigning network equipment. A station appliance is a proposal, not a discovered part of LUSI's setup.

## Guided setup

1. Select Workshop, Field, or Simulation. Each has separate identities and endpoints.
2. Join the intended network or select nearby BLE on a supported client.
3. Discover the rover, or enter a saved address if local discovery fails.
4. Confirm rover identity and enroll the client through an approved offline process.
5. Show network reachability, gateway version, module controller health, input readiness, and camera freshness separately.
6. Select a workspace and request control.
7. Confirm neutral input and deliberate enable.

A remembered rover opens in connected but disarmed state. No reconnect, service restart, or saved profile can automatically restore motion.

For iPad browser use, include a tested secure connection and certificate enrollment path. An IP address alone does not establish a trusted HTTPS connection. Avoid relying on repeated certificate warning bypasses or a captive portal browser. Verify the standalone browser or installed web app flow with no internet.

## Readiness and troubleshooting

A single readiness view should show:

1. Client input healthy.
2. Local station connection healthy.
3. Rover gateway reachable and identity verified.
4. Command channel authorized and responsive.
5. Selected module healthy with fresh state.
6. Required camera fresh and decodable.
7. Ownership available or current owner named.
8. Enable blocked or permitted, with its reason.

Every check distinguishes unknown, passed, failed, and not applicable. Do not turn a failed internet ping into a failed field connection.

Provide specific recovery actions such as selecting an input, checking a cable, reconnecting the intended network, or viewing a stopped service. Maintenance restart actions require authorization and motion inhibition. Diagnostic checks must not move motors, perform active CAN discovery, or reconfigure networks without a separate deliberate maintenance action.

Export a redacted support bundle with software versions, module capabilities, state ages, connection events, and recent faults. Exclude pairing credentials, tokens, passwords, and unnecessary personal information.

## Acceptance

A newly enrolled iPad, Steam Deck, and laptop each connect without a local ROS installation and complete the same available module exercises in simulation.

A returning operator reaches the readiness screen through one app launch or saved web address. Setup works without internet. Missing input hardware does not block unrelated modules.

Unplugging an input, freezing video, losing the radio, suspending a client, and restarting the gateway produce different accurate fault explanations and the required local motion expiry.

Confirm that a second client does not steal ownership, a reopened client stays disarmed, and a closed laptop does not remove the remaining clients' network.

Record actual setup time and recovery time on clean devices. No numerical target is claimed achieved by this documentation.

## Questions for the team

Confirm radio model and network mode, base station physical wiring, available access point, gateway host, camera connections, supported operating system versions, and the current approved startup procedure. Also identify who may perform enrollment and service maintenance.

No application source or deployment configuration is changed by this proposal.
