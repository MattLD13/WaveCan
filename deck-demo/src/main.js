import { InputController } from './input/InputController.js';
import { SimulationEngine } from './sim/MissionState.js';
import { createRenderer } from './ui/DashboardRenderer.js';

// The renderer and model are intentionally local to this window. There is no
// IPC command surface, network client, radio adapter, or hardware fallback.
const sim = new SimulationEngine();
const renderer = createRenderer();
// Local smoke tests use this handle to place the rover at mission checkpoints;
// it never contains a transport and is harmless outside the test harness.
window.__LUSI_SIM__ = sim;

const setText = (id, value) => { const node = document.getElementById(id); if (node) node.textContent = value; };
const updateDeadman = (held) => {
  const badge = document.getElementById('deadman-badge');
  if (badge) { badge.textContent = held ? 'DEADMAN ON' : 'HOLD TO ENABLE'; badge.classList.toggle('on', held); }
};

const updateController = (info) => {
  const label = info?.connected ? info.label : 'KEYBOARD FALLBACK';
  setText('controller-status', label);
  const dot = document.getElementById('controller-dot');
  dot?.classList.toggle('offline', !info?.connected);
};

const contextAction = () => {
  const state = sim.state;
  const stopped = !state.deadman && Math.abs(state.drive.speed) < 0.02;
  if (!stopped) {
    setText('event-line', 'A ACTION LOCKED • RELEASE DEADMAN + STOP ROVER');
    return;
  }
  if (!state.mission.started) {
    sim.startMission();
  } else if (state.mission.phase === 'science' && state.science.parked) {
    const result = sim.performScience(state.science.step);
    if (!result.ok) setText('event-line', result.message);
  } else if (state.mission.phase === 'beacon' && state.beacon.parked) {
    const result = sim.performBeacon('run-macro');
    if (!result.ok) setText('event-line', result.message);
  } else {
    setText('event-line', 'A ACTION READY • PARK AT THE ACTIVE MISSION OBJECT');
  }
};

const triggerStop = (reason = 'SIM E-STOP LATCHED • OUTPUT ZEROED') => {
  sim.setSafetyStop(true);
  setText('event-line', reason);
};

const input = new InputController({
  onDrive: (throttle, turn, deadman) => sim.commandDrive(throttle, turn, deadman),
  onLook: (yaw, pitch) => sim.setCameraLook(yaw, pitch),
  onDeadman: (held) => {
    updateDeadman(held);
    if (!held && !sim.state.safetyStop) sim.stop('DEADMAN RELEASED • OUTPUT ZEROED');
  },
  onAction: (action, value) => {
    if (action === 'stop' || action === 'b') triggerStop(value || 'SIM E-STOP LATCHED • OUTPUT ZEROED');
    if (action === 'camera' || action === 'x') sim.cycleCamera();
    if (action === 'link') sim.setConnected(!sim.state.connected);
    if (action === 'demo' || action === 'a' || action === 'context') contextAction();
    if (action === 'map' || action === 'y') sim.toggleMapDetail();
    if (action === 'controller') updateController(value);
    if (action === 'gamepad') setText('event-line', value ? 'GAMEPAD CONNECTED • SIM INPUT READY' : 'GAMEPAD DISCONNECTED • OUTPUT ZEROED');
  }
});

updateController(null);

document.querySelectorAll('[data-camera]').forEach((button) => button.addEventListener('click', () => sim.setCamera(button.dataset.camera)));
document.getElementById('demo-button')?.addEventListener('click', () => sim.startMission());
document.getElementById('activate-button')?.addEventListener('click', () => {
  if (!sim.state.mission.started) sim.startMission();
  sim.setControlsActive(!sim.state.controlsActive);
});
document.getElementById('sim-link-toggle')?.addEventListener('click', () => sim.setConnected(!sim.state.connected));
document.getElementById('stop-button')?.addEventListener('click', () => {
  if (sim.state.safetyStop) sim.setSafetyStop(false);
  else triggerStop();
});

document.getElementById('science-action')?.addEventListener('click', () => {
  const result = sim.performScience(sim.state.science.step);
  if (!result.ok) setText('event-line', result.message);
});
document.querySelectorAll('[data-macro]').forEach((button) => button.addEventListener('click', () => {
  const action = button.dataset.macro;
  if (action === 'run-beacon') {
    const result = sim.performBeacon('run-macro');
    if (!result.ok) setText('event-line', result.message);
  } else {
    const result = sim.performScience(action);
    if (!result.ok) setText('event-line', result.message);
  }
}));
document.getElementById('beacon-action')?.addEventListener('click', () => {
  const result = sim.performBeacon('run-macro');
  if (!result.ok) setText('event-line', result.message);
});

// Steam Deck-friendly hold-to-stop affordance. A short tap cannot latch it;
// keyboard ESC/B remain immediate stop bindings for accessibility.
let emergencyTimer = null;
const emergencyButton = document.getElementById('emergency-stop');
const cancelEmergency = () => { if (emergencyTimer) window.clearTimeout(emergencyTimer); emergencyTimer = null; emergencyButton?.classList.remove('armed'); };
const armEmergency = (event) => {
  event?.preventDefault();
  cancelEmergency();
  emergencyButton?.classList.add('armed');
  emergencyTimer = window.setTimeout(() => { emergencyTimer = null; triggerStop('SIM E-STOP LATCHED • HOLD CONFIRMED'); }, 800);
};
emergencyButton?.addEventListener('pointerdown', armEmergency);
emergencyButton?.addEventListener('pointerup', cancelEmergency);
emergencyButton?.addEventListener('pointerleave', cancelEmergency);
emergencyButton?.addEventListener('pointercancel', cancelEmergency);

setInterval(() => input.poll(), 33);
let last = performance.now();
const frame = (now) => {
  const dt = Math.min(0.1, Math.max(0, (now - last) / 1000));
  last = now;
  const state = sim.tick(dt);
  renderer.render(state);
  window.requestAnimationFrame(frame);
};
renderer.render(sim.snapshot());
window.requestAnimationFrame(frame);
