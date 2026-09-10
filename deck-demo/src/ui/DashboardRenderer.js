import { MarsWorld } from '../world/MarsWorld.js';
import { MarsMap } from './MarsMap.js';
import { SCIENCE_STEPS } from '../sim/MissionState.js';

const cameras = {
  front: 'FRONT CAMERA', rear: 'REAR CAMERA', arm: 'ARM CAMERA', overhead: 'OVERHEAD CAMERA', geo: 'GEO HD CAMERA'
};
const phaseOrder = { 'drive-outcrop': 0, science: 1, 'drive-marker': 2, beacon: 3, 'drive-final': 4, complete: 5 };
const phaseLabel = { 'drive-outcrop': 'APPROACH', science: 'SCIENCE', 'drive-marker': 'MARKER APPROACH', beacon: 'BEACON', 'drive-final': 'FINISH', complete: 'COMPLETE' };
const byId = (id) => document.getElementById(id);
const setText = (id, value) => { const node = byId(id); if (node) node.textContent = value; };
const fmt = (value, digits = 1) => Number(value).toFixed(digits);

const scienceInstruction = (state) => {
  if (!state.science.parked) return 'Drive to the geology outcrop. The science module stays locked until the rover is correctly parked.';
  if (state.science.step === 'complete') return 'Science sequence complete. The 440 nm response is a simulated demonstration, not a life-detection claim.';
  const step = SCIENCE_STEPS.find((entry) => entry.id === state.science.step);
  return step ? `Parked. Next: ${step.label}.` : 'Science sequence ready.';
};

const scienceValue = (state, id) => {
  if (!state.science.completed.includes(id)) return '—';
  if (id === 'classify-rock') return `${state.science.rockName.toUpperCase()} / 87%`;
  if (id === 'deep-sample') return `${state.science.depthCm.toFixed(1)} CM`;
  if (id === 'load-cell') return `${state.science.loadCellG.toFixed(1)} G`;
  if (id === 'soil-capture') return `${state.science.moisturePct.toFixed(1)}% / ${state.science.temperatureC.toFixed(1)}°C`;
  if (id === 'sample-reading') return `${state.science.assay.absorbance440.toFixed(2)} @ 440`;
  return 'DONE';
};

const updateScience = (state) => {
  const phase = state.mission.phase;
  const scienceModule = byId('science-module');
  const beaconModule = byId('beacon-module');
  const completeModule = byId('complete-module');
  const showingComplete = state.mission.complete;
  const showingBeacon = phase === 'beacon' || phase === 'drive-final';
  if (scienceModule) scienceModule.hidden = showingComplete || showingBeacon;
  if (beaconModule) beaconModule.hidden = showingComplete || !showingBeacon;
  if (completeModule) completeModule.hidden = !showingComplete;
  setText('module-title', showingComplete ? 'MISSION COMPLETE' : showingBeacon ? 'FIELD BEACON MODULE' : 'SCIENCE MODULE');
  setText('module-status', showingComplete ? 'LOGGED' : showingBeacon ? (state.beacon.parked ? 'PARKED' : 'PARK TO ACCESS') : (state.science.parked ? 'PARKED' : 'PARK TO ACCESS'));
  if (!showingBeacon && !showingComplete) {
    setText('science-park-status', state.science.parked ? 'OUTCROP PARKED' : 'OUTCROP NOT PARKED');
    setText('science-instruction', scienceInstruction(state));
    document.querySelectorAll('[data-science-step]').forEach((node) => {
      const id = node.dataset.scienceStep;
      const complete = state.science.completed.includes(id);
      node.classList.toggle('done', complete);
      node.classList.toggle('current', id === state.science.step && !complete);
      const value = node.querySelector('b');
      if (value) value.textContent = scienceValue(state, id);
    });
    const sampleReadout = byId('sample-readout');
    if (sampleReadout) sampleReadout.hidden = state.science.assay.absorbance440 == null;
    setText('absorbance-value', state.science.assay.absorbance440 == null ? '—' : state.science.assay.absorbance440.toFixed(2));
    setText('color-value', state.science.assay.observedColor || '—');
    const action = byId('science-action');
    const next = SCIENCE_STEPS.find((entry) => entry.id === state.science.step);
    if (action) {
      action.textContent = state.science.step === 'complete' ? 'SCIENCE COMPLETE' : (next ? next.label.toUpperCase() : 'DRIVE TO OUTCROP');
      action.disabled = state.science.step === 'complete' || !state.science.parked || !state.connected || state.safetyStop;
    }
  }
  if (showingBeacon && !showingComplete) {
    setText('beacon-park-status', state.beacon.parked ? 'FIELD MARKER PARKED' : 'FIELD MARKER NOT PARKED');
    setText('beacon-instruction', state.beacon.parked ? (state.beacon.macroRunning ? 'Arm macro running. Keep the rover parked until the beacon pulse is verified.' : state.beacon.beaconRepaired ? 'Beacon active. Drive to the final checkpoint.' : 'Ready. Run the visible arm macro to place the sample marker and repair the beacon.') : 'Park within the field marker ring to enable the visible arm automation.');
    document.querySelectorAll('[data-macro-step-label]').forEach((node) => {
      const step = Number(node.dataset.macroStepLabel);
      node.textContent = step <= state.beacon.macroStep ? 'DONE' : (step === state.beacon.macroStep + 1 && state.beacon.macroRunning ? 'RUNNING' : 'WAIT');
      node.parentElement?.classList.toggle('done', step <= state.beacon.macroStep);
      node.parentElement?.classList.toggle('current', step === state.beacon.macroStep + 1 && state.beacon.macroRunning);
    });
    const fill = byId('macro-progress-fill');
    if (fill) fill.style.width = `${(state.beacon.macroStep / state.beacon.macroTotal) * 100}%`;
    const action = byId('beacon-action');
    if (action) {
      action.textContent = state.beacon.beaconRepaired ? 'BEACON ACTIVE • DRIVE TO FINISH' : state.beacon.macroRunning ? `ARM MACRO ${state.beacon.macroStep}/${state.beacon.macroTotal}` : state.beacon.parked ? 'RUN ARM MACRO' : 'DRIVE TO FIELD MARKER';
      action.disabled = state.beacon.beaconRepaired || state.beacon.macroRunning || !state.beacon.parked || !state.connected || state.safetyStop;
    }
  }
  const macroState = byId('macro-state');
  if (macroState) {
    if (state.mission.complete) macroState.textContent = 'COMPLETE';
    else if (state.beacon.macroRunning) macroState.textContent = `ARM ${state.beacon.macroStep}/${state.beacon.macroTotal}`;
    else if (state.science.step !== 'geo-camera' && state.mission.phase === 'science') macroState.textContent = 'SCIENCE ACTIVE';
    else macroState.textContent = 'STANDBY';
  }
};

export function createRenderer() {
  const cameraCanvas = byId('camera-canvas');
  const mapCanvas = byId('map-canvas');
  const world = new MarsWorld(cameraCanvas);
  const map = new MarsMap(mapCanvas);
  const render = (state) => {
    world.update(state);
    world.render();
    map.render(state);
    const throttle = state.drive.throttle;
    const turn = state.drive.turn;
    setText('connection-label', state.connected ? 'SIM LINK' : 'LINK LOST');
    setText('connection-detail', state.connected ? `${state.latencyMs} ms • LOCAL` : 'OUTPUT ZEROED');
    setText('connection-mini', state.connected ? 'CONNECTED' : 'LOST');
    setText('battery', `${Math.round(state.battery)}%`);
    const batteryFill = byId('battery-fill'); if (batteryFill) batteryFill.style.width = `${Math.round(state.battery)}%`;
    setText('steam-battery', '82%');
    setText('top-mode', 'SIM');
    setText('camera-name', cameras[state.camera] || 'FRONT CAMERA');
    setText('active-feed', (state.camera || 'front').toUpperCase());
    setText('camera-state', state.connected ? 'LIVE • LOCAL' : 'STALE • OUTPUT ZEROED');
    setText('top-speed', fmt(Math.abs(state.drive.speed), 2));
    setText('top-heading', `${Math.round(state.pose.heading).toString().padStart(3, '0')}°`);
    setText('top-distance', fmt(state.distance, 1));
    setText('map-distance', `${fmt(state.distance, 1)} M`);
    setText('map-target', state.mission.complete ? 'COMPLETE' : state.mission.currentCheckpoint.toUpperCase());
    setText('map-detail-status', state.mapDetail ? 'DETAIL' : 'ROUTE');
    setText('speed', `${fmt(Math.abs(state.drive.speed), 2)} M/S`);
    setText('heading', `${Math.round(state.pose.heading).toString().padStart(3, '0')}°`);
    setText('distance', `${fmt(state.distance, 1)} M`);
    setText('link-age', `${Math.round(state.linkAgeMs)} MS`);
    setText('latency-value', `${Math.round(state.latencyMs)} MS`);
    setText('route-value', state.connected ? 'BLUETOOTH' : 'NO LINK');
    setText('safe-drive-value', state.connected && !state.safetyStop ? 'YES' : 'NO');
    setText('can-link', state.connected ? 'CONNECTED' : 'LOST');
    setText('can-port', 'CAN0');
    const drivePower = Math.round(Math.min(1, Math.abs(throttle)) * 100);
    const motorPower = [drivePower, Math.round(drivePower * 0.78), Math.round(drivePower * 0.58), Math.round(drivePower * 0.86)];
    ['m1', 'm2', 'm3', 'm4'].forEach((motor, index) => {
      const value = motorPower[index];
      const fill = byId(`motor-${motor}-power`); if (fill) fill.style.width = `${value}%`;
      setText(`motor-${motor}-value`, `${value}%`);
      setText(`motor-${motor}-direction`, throttle < -0.02 ? 'REV' : value > 0 ? 'FWD' : 'STBY');
    });
    setText('command-state', state.safetyStop ? 'SIM E-STOP' : (state.deadman ? 'DRIVE ACTIVE' : 'READY / HOLD TO ENABLE'));
    const dot = byId('connection-dot'); dot?.classList.toggle('offline', !state.connected);
    const miniDot = byId('connection-mini-dot'); miniDot?.classList.toggle('offline', !state.connected);
    const badge = byId('deadman-badge'); if (badge) { badge.textContent = state.deadman ? 'DEADMAN ON' : 'HOLD TO ENABLE'; badge.classList.toggle('on', state.deadman); }
    const leftDot = byId('left-stick-dot'); if (leftDot) leftDot.style.transform = `translate(${turn * 22}px, ${-throttle * 22}px)`;
    const rightDot = byId('right-stick-dot'); if (rightDot) rightDot.style.transform = `translate(${(state.view?.yaw || 0) * 22}px, ${(state.view?.pitch || 0) * 22}px)`;
    document.querySelectorAll('[data-camera]').forEach((button) => button.classList.toggle('active', button.dataset.camera === state.camera));
    const phase = phaseOrder[state.mission.phase] ?? 0;
    document.querySelectorAll('[data-phase]').forEach((node) => {
      const nodePhase = node.dataset.phase;
      const order = phaseOrder[nodePhase] ?? 0;
      node.classList.toggle('active', nodePhase === state.mission.phase);
      node.classList.toggle('done', order < phase);
    });
    setText('mission-phase-label', phaseLabel[state.mission.phase] || 'APPROACH');
    const objective = {
      'drive-outcrop': 'Drive to geology outcrop', science: 'Run outcrop science sequence', 'drive-marker': 'Drive to field marker', beacon: 'Place marker + repair beacon', 'drive-final': 'Drive to final checkpoint', complete: 'Survey Ridge complete'
    }[state.mission.phase];
    const hint = {
      'drive-outcrop': 'Follow the pale route. Park inside the ring to unlock science.', science: 'Use GEO HD, classify, sample deep, then run the assay in order.', 'drive-marker': 'Science logged. Follow the route to the field marker.', beacon: 'Hold position while the arm macro places and verifies the beacon.', 'drive-final': 'Beacon is active. Drive to the final checkpoint.', complete: 'All mission objects visited. Nice work, rover operator.'
    }[state.mission.phase];
    setText('mission-objective', objective);
    setText('mission-hint', hint);
    const start = byId('demo-button'); if (start) { start.textContent = state.mission.started ? (state.mission.complete ? 'MISSION COMPLETE' : 'MISSION ACTIVE') : 'START MISSION'; start.disabled = state.mission.started; }
    const stop = byId('stop-button'); if (stop) { stop.classList.toggle('latched', state.safetyStop); stop.textContent = state.safetyStop ? 'CLEAR E-STOP' : 'STOP OUTPUT'; }
    const toggle = byId('sim-link-toggle'); toggle?.classList.toggle('on', state.connected);
    const activate = byId('activate-button');
    if (activate) {
      activate.textContent = state.controlsActive ? 'DEACTIVATE' : 'ACTIVATE';
      activate.classList.toggle('active', state.controlsActive);
    }
    setText('event-line', state.event);
    updateScience(state);
  };
  return { render, world, map };
}
