# WaveCan workspace map

This map covers the related files found on this computer. Only the first item below is the canonical WaveCan Git repository; the other items live in an adjacent scratch workspace.

## Project 1: WaveCan application

**Canonical location:** `Documents/ChatGPT/New project/`

This Git checkout tracks `MattLD13/WaveCan` on `main`. It is a live Linux SocketCAN web application for SPARK MAX motor control. Its main components are the dashboard/server, SocketCAN adapter, hardware controller, REV frame helpers, tests, and live diagnostic scripts. Start with the repository [README](../README.md), then use the architecture and operations guides linked there.

## Project 2: sparkcan-ref library

**Location:** `Documents/Codex/2026-04-26/can-you-connect-to-a-device/sparkcan-ref/`

This is a separate C++17 CMake library with its own nested Git repository, headers, source files, and control/PID/status examples. Its local README describes CANable use and a stated firmware compatibility range. It is a reference project; the canonical WaveCan app does not link to or build against it.

## Project 3: hardware integration tools

**Location:** top level of `Documents/Codex/2026-04-26/can-you-connect-to-a-device/`

This scratch area contains WaveCan CAN/API probes, remote startup and patch scripts, and a C++ probe for `sparkcan-ref`. Treat these as hardware investigation utilities rather than another application. Inspect each script before running it because it may address a remote host or send commands to a physical motor.

## Duplicate and repository boundaries

The scratch workspace also contains `WaveCan/`, an older, non-Git copy of the Python app. It has diverged from the canonical checkout. Make application changes in `Documents/ChatGPT/New project/` and use the scratch copy only to recover historical investigation context.

The scratch workspace root has no Git repository of its own; `sparkcan-ref/` has its own nested Git metadata. Changes to workspace-level scripts and notes therefore are not versioned by the canonical WaveCan checkout.

Some legacy remote patch scripts in the scratch workspace contain hardcoded credentials and host-key material. Do not share that folder as-is, and rotate any credential that may still be valid. This guide intentionally does not reproduce those values.
