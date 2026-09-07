const test = require('node:test');
const assert = require('node:assert/strict');
const { SimulationEngine } = require('../src/sim/SimulationEngine.cjs');

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
