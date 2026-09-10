/* CommonJS test/runtime mirror of MissionState.js. No transport or hardware. */

const MAX_SPEED_MPS = 0.55;
const MAP_SCALE_METERS = 26;
const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const wrapAngle = (degrees) => ((degrees % 360) + 360) % 360;
const CHECKPOINTS = Object.freeze({
  start: Object.freeze({ id: 'start', label: 'BASE CAMP', x: 0.12, y: 0.78, radius: 0.065 }),
  outcrop: Object.freeze({ id: 'outcrop', label: 'GEOLOGY OUTCROP', x: 0.44, y: 0.47, radius: 0.085 }),
  marker: Object.freeze({ id: 'marker', label: 'FIELD MARKER', x: 0.73, y: 0.27, radius: 0.085 }),
  final: Object.freeze({ id: 'final', label: 'FINAL CHECKPOINT', x: 0.89, y: 0.12, radius: 0.075 })
});
const MISSION_ROUTE = Object.freeze([
  { x: 0.12, y: 0.78 }, { x: 0.21, y: 0.66 }, { x: 0.31, y: 0.59 },
  { x: 0.44, y: 0.47 }, { x: 0.56, y: 0.39 }, { x: 0.64, y: 0.34 },
  { x: 0.73, y: 0.27 }, { x: 0.82, y: 0.20 }, { x: 0.89, y: 0.12 }
]);
const SCIENCE_STEPS = [
  { id: 'geo-camera', short: 'GEO CAM', label: 'High-resolution geo camera' },
  { id: 'classify-rock', short: 'ROCK DB', label: 'Classify with rock database' },
  { id: 'deep-sample', short: 'DEPTH', label: 'Collect past 10 cm' },
  { id: 'discard-shallow', short: 'BRIDGE', label: 'Discard shallow material' },
  { id: 'load-cell', short: 'MASS', label: 'Verify load cell > 5 g' },
  { id: 'soil-capture', short: 'SOIL', label: 'Capture moisture + temperature' },
  { id: 'hydrogen-peroxide', short: 'H₂O₂', label: 'Add hydrogen peroxide' },
  { id: 'cobalt-bicarbonate', short: 'COBALT', label: 'Add cobalt bicarbonate' },
  { id: 'blank-cuvette', short: 'BLANK', label: 'Calibrate blank cuvette' },
  { id: 'control-sample', short: 'CONTROL', label: 'Run control sample' },
  { id: 'sample-reading', short: 'READ', label: 'Read sample at 440 nm' }
];
const BEACON_MACRO_STEPS = ['ALIGN ARM', 'PLACE SAMPLE MARKER', 'REPAIR FIELD BEACON', 'VERIFY BEACON PULSE'];
const distance = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

const initialState = () => ({
  connected: true, safetyStop: false, deadman: false,
  drive: { throttle: 0, turn: 0, speed: 0 },
  pose: { x: 0.12, y: 0.78, heading: 43 }, view: { yaw: 0, pitch: 0 }, mapDetail: false, camera: 'front',
  battery: 94, latencyMs: 42, linkAgeMs: 0, elapsed: 0, distance: 0,
  mission: {
    id: 'MARS ANALOG 01', title: 'Survey Ridge', phase: 'drive-outcrop',
    currentCheckpoint: 'outcrop', complete: false, started: false,
    visited: { start: true, outcrop: false, marker: false, final: false }, route: MISSION_ROUTE
  },
  science: {
    parked: false, step: 'geo-camera', completed: [], geoCameraReady: false,
    classified: false, rockName: 'Basaltic breccia', rockConfidence: 0.87,
    deepSample: false, depthCm: 0, shallowDiscarded: false, loadCellG: 0,
    massVerified: false, moistureCaptured: false, moisturePct: null,
    temperatureCaptured: false, temperatureC: null,
    assay: { step: 'hydrogen-peroxide', completed: [], absorbance440: null, observedColor: null, interpretation: 'SIMULATED COLOR RESPONSE — NO LIFE CLAIM' }
  },
  beacon: { parked: false, markerPlaced: false, beaconRepaired: false, macroRunning: false, macroStep: 0, macroTotal: 4, lastStepAt: 0 },
  // Compatibility state for the old guided-demo harness. Mission progress is
  // still controlled by mission.phase above.
  demo: { active: false, step: 0, complete: false },
  event: 'SIMULATION READY • DRIVE TO GEOLOGY OUTCROP'
});

class SimulationEngine {
  constructor() { this.state = initialState(); }
  reset() { this.state = initialState(); return this.snapshot(); }
  snapshot() { return structuredClone(this.state); }
  startMission() {
    if (!this.state.mission.complete) {
      this.state.mission.started = true;
      this.state.demo.active = true;
      this.state.demo.step = Math.max(1, this.state.demo.step);
      this.state.event = 'MISSION ACTIVE • DRIVE TO GEOLOGY OUTCROP';
    }
    return this.snapshot();
  }
  startDemo() { return this.startMission(); }
  advanceDemo(step) {
    if (!this.state.mission.started) this.startMission();
    this.state.demo.step = Math.max(this.state.demo.step, Math.min(4, Number(step) || 0));
    if (this.state.demo.step >= 4 && this.state.mission.complete) {
      this.state.demo.active = false;
      this.state.demo.complete = true;
    }
    if (Number(step) >= 4 && this.state.mission.complete) this.state.event = 'MISSION COMPLETE • SURVEY RIDGE LOGGED';
    return this.snapshot();
  }
  setConnected(connected) {
    this.state.connected = Boolean(connected);
    if (!this.state.connected) this.stop('SIM LINK LOST • OUTPUT ZEROED');
    else { this.state.linkAgeMs = 0; this.state.event = 'SIM LINK RESTORED • OUTPUT INHIBITED UNTIL ENABLE'; }
    return this.snapshot();
  }
  setSafetyStop(stopped) {
    this.state.safetyStop = Boolean(stopped);
    if (stopped) this.stop('SIM E-STOP LATCHED • OUTPUT ZEROED');
    else this.state.event = 'SIM E-STOP CLEARED • HOLD DEADMAN TO DRIVE';
    return this.snapshot();
  }
  setCamera(camera) {
    if (['front', 'rear', 'arm', 'overhead', 'geo'].includes(camera)) this.state.camera = camera;
    return this.snapshot();
  }
  setCameraLook(yaw, pitch) {
    this.state.view.yaw = clamp(Number(yaw) || 0, -1, 1);
    this.state.view.pitch = clamp(Number(pitch) || 0, -1, 1);
    return this.snapshot();
  }
  toggleMapDetail() {
    this.state.mapDetail = !this.state.mapDetail;
    this.state.event = this.state.mapDetail ? 'MINIMAP DETAIL ON • TERRAIN CONTOURS VISIBLE' : 'MINIMAP DETAIL OFF • ROUTE VIEW';
    return this.snapshot();
  }
  cycleCamera(direction = 1) {
    const cameras = ['front', 'rear', 'arm', 'overhead', 'geo'];
    const index = cameras.indexOf(this.state.camera);
    this.state.camera = cameras[(index + direction + cameras.length) % cameras.length];
    return this.snapshot();
  }
  isParked(checkpointId = this.state.mission.currentCheckpoint) {
    const checkpoint = CHECKPOINTS[checkpointId];
    return Boolean(checkpoint && distance(this.state.pose, checkpoint) <= checkpoint.radius);
  }
  commandDrive(throttle, turn, deadman = true) {
    this.state.deadman = Boolean(deadman);
    if (!this.state.connected || this.state.safetyStop || !deadman) {
      this.stop(deadman ? 'SIM OUTPUT INHIBITED' : 'DEADMAN RELEASED');
      return this.snapshot();
    }
    this.state.drive.throttle = clamp(Number(throttle) || 0, -1, 1);
    this.state.drive.turn = clamp(Number(turn) || 0, -1, 1);
    this.state.mission.started = true;
    this.state.event = 'SIM DRIVE COMMAND ACTIVE';
    return this.snapshot();
  }
  stop(event = 'SIM OUTPUT STOPPED') {
    this.state.drive.throttle = 0; this.state.drive.turn = 0; this.state.drive.speed = 0;
    this.state.deadman = false; this.state.event = event;
    return this.snapshot();
  }
  _reject(message) { this.state.event = `SIM • ${message}`; return { ok: false, state: this.snapshot(), message }; }
  _completeScienceStep(id) {
    if (!this.state.science.completed.includes(id)) this.state.science.completed.push(id);
    const index = SCIENCE_STEPS.findIndex((step) => step.id === id);
    const next = SCIENCE_STEPS[index + 1];
    this.state.science.step = next?.id || 'complete';
    if (SCIENCE_STEPS.slice(6).some((step) => step.id === id)) {
      if (!this.state.science.assay.completed.includes(id)) this.state.science.assay.completed.push(id);
      this.state.science.assay.step = next?.id || 'complete';
    }
  }
  performScience(action) {
    const science = this.state.science;
    if (this.state.mission.phase !== 'science' || !this.isParked('outcrop')) {
      science.parked = false; return this._reject('PARK WITHIN OUTCROP RING TO ACCESS SCIENCE');
    }
    science.parked = true;
    if (action !== science.step) return this._reject(`NEXT SCIENCE STEP: ${science.step.toUpperCase()}`);
    switch (action) {
      case 'geo-camera': science.geoCameraReady = true; this.setCamera('geo'); this._completeScienceStep(action); this.state.event = 'GEO CAMERA READY • OPEN ROCK DATABASE'; break;
      case 'classify-rock': if (!science.geoCameraReady) return this._reject('GEO CAMERA MUST BE READY FIRST'); science.classified = true; this._completeScienceStep(action); this.state.event = 'OUTCROP CLASSIFIED • BASALTIC BRECCIA / 87%'; break;
      case 'deep-sample': if (!science.classified) return this._reject('CLASSIFY OUTCROP BEFORE SAMPLING'); science.deepSample = true; science.depthCm = 12.4; this._completeScienceStep(action); this.state.event = 'DEEP SAMPLE COLLECTED • 12.4 CM'; break;
      case 'discard-shallow': if (!science.deepSample) return this._reject('COLLECT THE DEEP SAMPLE FIRST'); science.shallowDiscarded = true; this._completeScienceStep(action); this.state.event = 'SHALLOW MATERIAL DISCARDED • BRIDGE ROTATED'; break;
      case 'load-cell': if (!science.shallowDiscarded) return this._reject('ROTATE BRIDGE TO DISCARD SHALLOW MATERIAL'); science.loadCellG = 6.4; science.massVerified = science.loadCellG > 5; this._completeScienceStep(action); this.state.event = `LOAD CELL PASS • ${science.loadCellG.toFixed(1)} G > 5 G`; break;
      case 'soil-capture': if (!science.massVerified) return this._reject('LOAD CELL MUST EXCEED 5 G'); science.moisturePct = 18.6; science.temperatureC = -13.2; science.moistureCaptured = true; science.temperatureCaptured = true; this._completeScienceStep(action); this.state.event = 'SOIL CAPTURED • MOISTURE 18.6% • TEMP −13.2°C'; break;
      case 'hydrogen-peroxide': if (!science.moistureCaptured) return this._reject('CAPTURE SOIL MOISTURE + TEMPERATURE FIRST'); this._completeScienceStep(action); this.state.event = 'ASSAY 1/5 • HYDROGEN PEROXIDE DISPENSED'; break;
      case 'cobalt-bicarbonate': this._completeScienceStep(action); this.state.event = 'ASSAY 2/5 • COBALT BICARBONATE DISPENSED'; break;
      case 'blank-cuvette': this._completeScienceStep(action); this.state.event = 'ASSAY 3/5 • BLANK CUVETTE CALIBRATED'; break;
      case 'control-sample': this._completeScienceStep(action); this.state.event = 'ASSAY 4/5 • CONTROL SAMPLE ACCEPTED'; break;
      case 'sample-reading': science.assay.absorbance440 = 0.37; science.assay.observedColor = 'amber → copper'; this._completeScienceStep(action); science.assay.step = 'complete'; this.setCamera('front'); this.state.mission.phase = 'drive-marker'; this.state.mission.currentCheckpoint = 'marker'; this.state.event = 'ASSAY COMPLETE • 440 NM 0.37 • SIMULATED COLOR RESPONSE'; break;
      default: return this._reject('UNKNOWN SCIENCE ACTION');
    }
    return { ok: true, state: this.snapshot(), message: this.state.event };
  }
  performBeacon(action = 'run-macro') {
    if (this.state.mission.phase !== 'beacon' || !this.isParked('marker')) {
      this.state.beacon.parked = false; return this._reject('PARK WITHIN FIELD MARKER RING TO ACCESS ARM MACRO');
    }
    this.state.beacon.parked = true;
    if (action !== 'run-macro') return this._reject('USE ARM MACRO TO PLACE MARKER + REPAIR BEACON');
    if (this.state.beacon.macroRunning) return { ok: true, state: this.snapshot(), message: 'ARM MACRO ALREADY RUNNING' };
    this.state.beacon.macroRunning = true; this.state.beacon.macroStep = 0; this.state.beacon.lastStepAt = this.state.elapsed;
    this.state.event = 'ARM MACRO STARTED • ALIGNING TO FIELD MARKER';
    return { ok: true, state: this.snapshot(), message: this.state.event };
  }
  advanceBeaconMacro() {
    const beacon = this.state.beacon;
    if (!beacon.macroRunning) return this.snapshot();
    beacon.macroStep = Math.min(beacon.macroTotal, beacon.macroStep + 1); beacon.lastStepAt = this.state.elapsed;
    this.state.event = `ARM MACRO ${beacon.macroStep}/${beacon.macroTotal} • ${BEACON_MACRO_STEPS[beacon.macroStep - 1]}`;
    if (beacon.macroStep >= beacon.macroTotal) {
      beacon.macroRunning = false; beacon.markerPlaced = true; beacon.beaconRepaired = true;
      this.state.mission.phase = 'drive-final'; this.state.mission.currentCheckpoint = 'final';
      this.state.event = 'FIELD BEACON ACTIVE • DRIVE TO FINAL CHECKPOINT';
    }
    return this.snapshot();
  }
  _updateMissionProximity() {
    const mission = this.state.mission;
    if (mission.phase === 'drive-outcrop' && this.isParked('outcrop')) {
      mission.visited.outcrop = true; mission.phase = 'science'; mission.currentCheckpoint = 'outcrop';
      this.state.science.parked = true; this.state.event = 'OUTCROP PARKED • GEO CAMERA + ROCK DATABASE READY';
    } else if (mission.phase === 'science') this.state.science.parked = this.isParked('outcrop');
    else if (mission.phase === 'drive-marker' && this.isParked('marker')) {
      mission.visited.marker = true; mission.phase = 'beacon'; mission.currentCheckpoint = 'marker';
      this.state.beacon.parked = true; this.state.event = 'FIELD MARKER PARKED • ARM MACRO READY';
    } else if (mission.phase === 'beacon') this.state.beacon.parked = this.isParked('marker');
    else if (mission.phase === 'drive-final' && this.isParked('final')) {
      mission.visited.final = true; mission.phase = 'complete'; mission.currentCheckpoint = 'final'; mission.complete = true;
      this.state.event = 'MISSION COMPLETE • SURVEY RIDGE LOGGED • SIM ONLY';
    }
  }
  tick(dtSeconds) {
    const dt = clamp(Number(dtSeconds) || 0, 0, 0.1);
    this.state.elapsed += dt; this.state.linkAgeMs = this.state.connected ? 0 : this.state.linkAgeMs + dt * 1000;
    const canDrive = this.state.deadman && this.state.connected && !this.state.safetyStop;
    const targetSpeed = canDrive ? this.state.drive.throttle * MAX_SPEED_MPS : 0;
    this.state.drive.speed += (targetSpeed - this.state.drive.speed) * Math.min(1, dt * 8);
    this.state.drive.speed = clamp(this.state.drive.speed, -MAX_SPEED_MPS, MAX_SPEED_MPS);
    this.state.pose.heading = wrapAngle(this.state.pose.heading + this.state.drive.turn * 82 * dt);
    const radians = this.state.pose.heading * Math.PI / 180;
    const mapRate = this.state.drive.speed * dt / MAP_SCALE_METERS;
    this.state.pose.x = clamp(this.state.pose.x + Math.cos(radians) * mapRate, 0.035, 0.965);
    this.state.pose.y = clamp(this.state.pose.y + Math.sin(radians) * mapRate, 0.045, 0.955);
    this.state.distance += Math.abs(this.state.drive.speed * dt);
    this.state.battery = clamp(this.state.battery - Math.abs(this.state.drive.speed) * dt * 0.012, 0, 100);
    this.state.latencyMs = 38 + Math.round(Math.abs(Math.sin(this.state.elapsed * 0.8)) * 12);
    if (this.state.beacon.macroRunning && this.state.beacon.parked && this.state.elapsed - this.state.beacon.lastStepAt >= 0.66) this.advanceBeaconMacro();
    this._updateMissionProximity();
    return this.snapshot();
  }
}

module.exports = { SimulationEngine, initialState, clamp, wrapAngle, CHECKPOINTS, MISSION_ROUTE, SCIENCE_STEPS, BEACON_MACRO_STEPS, MAX_SPEED_MPS };
