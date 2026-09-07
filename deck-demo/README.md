# LUSI Rover Ops

A compact, simulation-only LUSI rover operator cockpit for exploring a safe driving workflow. It is a standalone Electron app under `deck-demo/`; the existing WaveCan Python/CAN code stays untouched.

The UI is camera-first: the simulated LUSI Vision feed is the central workspace, with a course map, compact drive/deadman controls, and prioritized telemetry/safety status in the rail and lower strip. Each surface is rendered from a small state model so a future ROS or WaveCan adapter can replace the simulator without coupling transport to presentation.

The explicit link-loss preview uses Lehigh University Space Initiative’s `LUSNoDownlink.png` from `urc_software` (see the style guide for source notes). The LCD hardware mockup references Valve’s official Steamworks front line art and is not used as application branding.

See [INSTALL.md](INSTALL.md) for run, package, controls, and validation commands. Design rationale and source notes are in [docs/STEAM_DECK_UI_RESEARCH.md](../docs/STEAM_DECK_UI_RESEARCH.md).
