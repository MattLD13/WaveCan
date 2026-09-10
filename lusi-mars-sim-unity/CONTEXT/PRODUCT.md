# Product Intent

## What this is

LUSI Mars Simulator is a high fidelity, local only Unity demonstration of the rover operator experience. A child or visitor should be able to pick up the Steam Deck, understand the screen quickly, drive across a realistic Martian surface, reach meaningful sites, run a science sequence, and use an arm macro.

The player is not operating a real rover. The simulator is deliberately separated from WaveCan, Bluetooth, CAN, ROS, serial devices, radios, and all external motor outputs.

## Why Unity

The previous Electron demonstration remains useful as a fast UI reference. Unity is the chosen experience because the final demo needs a believable FPV driving world, terrain contact, rocks, sunlight, dust haze, rover motion, controller support, and a performance profile appropriate for the Steam Deck.

## Target hardware

* Original Steam Deck, 512 GB launch model.
* 1280 by 800 LCD.
* 30 FPS target as the normal performance profile.
* Standard Steam Input gamepad mapping, with keyboard fallbacks for development.

## Experience principles

* The center view is driving first, never a dashboard.
* Use a restrained modern dark interface. Color is reserved for LUSI branding, the status ribbon, activation state, and emergency control.
* The interface should feel deliberately engineered, inspired by credible space operations, industrial controls, and film production control surfaces, rather than a generic game HUD.
* Every device will eventually be able to operate every compatible LUSI module. The Steam Deck is optimized for driving, but the information model must remain modular enough for arm, science, camera, and future devices.
* Simulation should rehearse real operator flows without pretending that simulation data is real hardware telemetry.
