# Mars Demonstration Mission

## Mission identity

Mission name: MARS ANALOG 01, Survey Ridge.

The player begins at BASE, drives to a geology outcrop, conducts a simplified but ordered simulation of the LUSI science workflow, then drives to a field marker for an arm sequence before reaching FINAL.

## Route

1. BASE to GEOLOGY OUTCROP.
2. GEOLOGY OUTCROP to FIELD MARKER after science is complete.
3. FIELD MARKER to FINAL after the arm sequence is complete.

The minimap should make this route immediately visible without obstructing the FPV view.

## Science sequence

The science interaction sequence models the real LUSI science module workflow in safe simulated form:

1. High resolution geology camera.
2. Rock database observation.
3. Deep sample greater than 10 cm.
4. Shallow material discard.
5. Load cell sample greater than 5 g.
6. Moisture and temperature reading.
7. Hydrogen peroxide reagent.
8. Cobalt bicarbonate reagent.
9. Blank cuvette.
10. Control sample.
11. Simulated 440 nm reading and observed color.

All displayed readings must be marked simulated. Do not infer or claim life detection.

## Arm sequence

At the field marker, the player runs a safe simulated arm macro:

1. Place a sample marker.
2. Repair or activate the beacon.

The simulator must make it obvious that these are virtual actions, not commands to a physical arm.

## Controls

* Left stick or W A S D drives and steers.
* Right stick or arrow keys controls the camera.
* R2 or Space is the deadman and is required for motion.
* A or E performs the current parked interaction.
* X or C cycles camera.
* Y or M toggles minimap detail.
* Hold B or Escape for 1.2 seconds to trigger a simulated emergency stop.
* Start plus Select or R resets the mission.
