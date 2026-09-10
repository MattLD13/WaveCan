# Approved Cockpit UI

## Layout

The UI is built around a central camera view.

* The center is 100 percent FPV camera. No labels, cards, crosshairs, or redundant telemetry are placed over it.
* A five pixel ribbon runs across the entire top edge. It is always purple in this simulator to indicate SIM.
* A compact top status band sits immediately below the ribbon.
* The LUSI logo and wordmark must fit fully with no crop or clipped descenders. Keep the mark and wordmark visually close.
* Use vertical separator lines between the data groups, batteries, activation state, and emergency stop region.
* Bottom controls should be tall enough to use the available display but remain outside the driving feed.
* The minimap is a corner element and shows the Mars route map.

## Top status band

Show only useful, glanceable information:

* Speed.
* Heading.
* Distance.
* Mode, shown as SIM.
* Rover battery, label above and a battery rectangle containing the percent.
* Steam Deck battery, same treatment.
* Activation state. It is an `ACTIVATE` or `DEACTIVATE` control, shown green when active and red when inactive.
* Persistent emergency stop control.

Avoid repeated status labels and avoid large unused gaps.

## Emergency stop

The emergency stop is rectangular and visually unmistakable. Use a thick dashed yellow caution border inspired by the supplied emergency stop sign, with red accents and clear hold instruction. It must remain visible at all times.

The input requires a deliberate hold, not an accidental single press. In simulation it latches only the simulated rover state.

## Ribbon states

The ribbon has a solid module color, then may carry an additional connection overlay.

| Meaning | Solid ribbon color | Overlay behavior |
| --- | --- | --- |
| No selected module or nominal rover state | Green | None |
| Robotic arm module | Blue | None |
| Science module | Orange | None |
| Simulation | Purple | Always used by the Mars simulator |
| Bluetooth transport | Keep module color | Long blue visor overlay |
| Brief lost link | Keep module color | Long yellow visor overlay using the same geometry as Bluetooth |
| Link disconnected for more than 30 seconds | Red | Flashing red state |

The visor overlays are long, approximately three times the older short visor length. They are connection information layered over the chosen module state, not replacement module colors. The current simulator remains purple because it is always simulation.

## Motor and module information

When a user opens detailed motor status, show individual motor power and direction, CAN port, and CAN link state. Do not place this persistent detail in the driving view.

Camera choices are front, rear, arm, and overhead. The selected active camera is the only camera in the center view. If no video is present in a future real application, use a clean no feed screen rather than a fake feed.

## Visual language

Use authentic LUSI marks and colors from existing repository assets when they are available. Use dark mode only. Do not introduce unrelated bright palettes, fake panel depth, or decorative sci fi clutter. Spacing, alignment, and clipping deserve more attention than adding more widgets.
