const test = require('node:test');
const assert = require('node:assert/strict');
const { SimulationEngine, CHECKPOINTS, SCIENCE_STEPS, BEACON_MACRO_STEPS, MAX_SPEED_MPS } = require('../src/sim/SimulationEngine.cjs');

test('initial state is safe and stationary', () => {
  const sim = new SimulationEngine();
  assert.equal(sim.state.connected, true);
  assert.equal(sim.state.deadman, false);
  assert.equal(sim.state.drive.speed, 0);
});

test('drive command is inhibited until the deadman is held', () => {
  const sim = new SimulationEngine();
  sim.commandDrive(1, 0, false);
  sim.tick(0.1);
  assert.equal(sim.state.drive.speed, 0);
  assert.equal(sim.state.event, 'DEADMAN RELEASED');
  sim.commandDrive(1, 0, true);
  sim.tick(0.1);
  assert(sim.state.drive.speed > 0);
});

test('releasing deadman zeros command and speed', () => {
  const sim = new SimulationEngine();
  sim.commandDrive(1, 0, true);
  sim.tick(0.1);
  sim.commandDrive(0.4, 0, false);
  assert.equal(sim.state.drive.throttle, 0);
  assert.equal(sim.state.drive.turn, 0);
  assert.equal(sim.state.drive.speed, 0);
  assert.equal(sim.state.deadman, false);
});

test('link loss immediately zeroes output and recovery is explicit', () => {
  const sim = new SimulationEngine();
  sim.commandDrive(0.5, 0.2, true);
  sim.tick(0.1);
  sim.setConnected(false);
  assert.equal(sim.state.connected, false);
  assert.equal(sim.state.drive.speed, 0);
  assert.match(sim.state.event, /LINK LOST/);
  sim.setConnected(true);
  assert.equal(sim.state.connected, true);
  assert.equal(sim.state.drive.throttle, 0);
});

test('safety stop latches and bounds pose and battery', () => {
  const sim = new SimulationEngine();
  sim.commandDrive(1, 1, true);
  for (let i = 0; i < 150; i++) sim.tick(0.1);
  assert(sim.state.pose.x >= 0.04 && sim.state.pose.x <= 0.96);
  assert(sim.state.pose.y >= 0.08 && sim.state.pose.y <= 0.92);
  assert(sim.state.battery >= 0 && sim.state.battery <= 100);
  sim.setSafetyStop(true);
  assert.equal(sim.state.safetyStop, true);
  assert.equal(sim.state.drive.speed, 0);
  sim.commandDrive(1, 0, true);
  assert.equal(sim.state.drive.speed, 0);
});

test('camera cycle and guided demo progress', () => {
  const sim = new SimulationEngine();
  assert.equal(sim.state.camera, 'front');
  sim.cycleCamera(); sim.cycleCamera();
  assert.equal(sim.state.camera, 'arm');
  sim.startDemo();
  assert.equal(sim.state.demo.active, true);
  sim.advanceDemo(3);
  assert.equal(sim.state.demo.step, 3);
});

test('science procedure enforces parking and the exact assay order', () => {
  const sim = new SimulationEngine();
  assert.equal(sim.performScience('geo-camera').ok, false, 'science must stay locked while driving');
  sim.state.pose = { x: CHECKPOINTS.outcrop.x, y: CHECKPOINTS.outcrop.y, heading: 43 };
  sim.tick(0);
  assert.equal(sim.state.mission.phase, 'science');
  assert.equal(sim.performScience('classify-rock').ok, false, 'geo camera must be used first');
  const ordered = SCIENCE_STEPS.map((step) => step.id);
  assert.equal(sim.performScience('geo-camera').ok, true);
  for (const action of ordered.slice(1)) assert.equal(sim.performScience(action).ok, true, `step ${action} should follow the prior step`);
  assert.equal(sim.state.science.deepSample, true);
  assert.equal(sim.state.science.depthCm > 10, true);
  assert.equal(sim.state.science.shallowDiscarded, true);
  assert.equal(sim.state.science.loadCellG > 5, true);
  assert.equal(sim.state.science.moistureCaptured, true);
  assert.equal(sim.state.science.temperatureCaptured, true);
  assert.equal(sim.state.science.assay.completed.join(','), ordered.slice(6).join(','));
  assert.equal(sim.state.science.assay.absorbance440, 0.37);
  assert.equal(sim.state.mission.phase, 'drive-marker');
});

test('marker macro requires a parked rover and completes before the finish leg', () => {
  const sim = new SimulationEngine();
  sim.state.mission.phase = 'beacon';
  sim.state.mission.currentCheckpoint = 'marker';
  assert.equal(sim.performBeacon('run-macro').ok, false);
  sim.state.pose = { x: CHECKPOINTS.marker.x, y: CHECKPOINTS.marker.y, heading: 43 };
  sim.tick(0);
  assert.equal(sim.state.beacon.parked, true);
  assert.equal(sim.performBeacon('run-macro').ok, true);
  for (let i = 0; i < BEACON_MACRO_STEPS.length * 7; i += 1) sim.tick(0.1);
  assert.equal(sim.state.beacon.markerPlaced, true);
  assert.equal(sim.state.beacon.beaconRepaired, true);
  assert.equal(sim.state.mission.phase, 'drive-final');
  sim.state.pose = { x: CHECKPOINTS.final.x, y: CHECKPOINTS.final.y, heading: 43 };
  sim.tick(0);
  assert.equal(sim.state.mission.complete, true);
});

test('simulation remains bounded and has no transport or hardware path', () => {
  const sim = new SimulationEngine();
  sim.commandDrive(99, 99, true);
  for (let i = 0; i < 40; i += 1) sim.tick(0.1);
  assert(Math.abs(sim.state.drive.speed) <= MAX_SPEED_MPS);
  assert(sim.state.pose.x >= 0.035 && sim.state.pose.x <= 0.965);
  assert(sim.state.pose.y >= 0.045 && sim.state.pose.y <= 0.955);
  assert.equal(typeof sim.connect, 'undefined');
  assert.equal(typeof sim.send, 'undefined');
});
