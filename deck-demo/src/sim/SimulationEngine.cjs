const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
const wrapAngle = (degrees) => ((degrees % 360) + 360) % 360;

const initialState = () => ({
  connected: true,
  safetyStop: false,
  deadman: false,
  drive: { throttle: 0, turn: 0, speed: 0 },
  pose: { x: 0.12, y: 0.48, heading: 18 },
  camera: 'front',
  battery: 94,
  latencyMs: 42,
  linkAgeMs: 0,
  elapsed: 0,
  distance: 0,
  demo: { active: false, step: 0, complete: false },
  event: 'SIMULATION READY'
});

class SimulationEngine {
  constructor() { this.state = initialState(); }
  reset() { this.state = initialState(); return this.snapshot(); }
  snapshot() { return structuredClone(this.state); }
  setConnected(connected) {
    this.state.connected = Boolean(connected);
    this.state.event = this.state.connected ? 'SIM LINK RESTORED' : 'LINK LOST • OUTPUT ZEROED';
    if (!this.state.connected) this.stop('LINK LOST');
  }
  setSafetyStop(stopped) {
    this.state.safetyStop = Boolean(stopped);
    if (stopped) this.stop('SAFETY STOP LATCHED');
    else this.state.event = 'SAFETY STOP CLEARED';
  }
  setCamera(camera) {
    const cameras = ['front', 'rear', 'arm', 'overhead'];
    if (cameras.includes(camera)) this.state.camera = camera;
  }
  cycleCamera(direction = 1) {
    const cameras = ['front', 'rear', 'arm', 'overhead'];
    const next = (cameras.indexOf(this.state.camera) + direction + cameras.length) % cameras.length;
    this.setCamera(cameras[next]);
  }
  startDemo() {
    this.state.demo = { active: true, step: 1, complete: false };
    this.state.event = 'DEMO STARTED • HOLD DEADMAN TO DRIVE';
  }
  advanceDemo(step) {
    if (!this.state.demo.active) return;
    if (step >= this.state.demo.step) this.state.demo.step = Math.min(4, step);
    if (this.state.demo.step >= 4 && this.state.distance > 0.05) {
      this.state.demo.complete = true;
      this.state.demo.active = false;
      this.state.event = 'DEMO COMPLETE • COURSE CHECKPOINT REACHED';
    }
  }
  commandDrive(throttle, turn, deadman = true) {
    this.state.deadman = Boolean(deadman);
    if (!this.state.connected || this.state.safetyStop || !deadman) {
      this.stop(deadman ? 'OUTPUT INHIBITED' : 'DEADMAN RELEASED');
      return;
    }
    this.state.drive.throttle = clamp(Number(throttle) || 0, -1, 1);
    this.state.drive.turn = clamp(Number(turn) || 0, -1, 1);
    this.state.event = 'DRIVE COMMAND ACTIVE';
  }
  stop(event = 'STOPPED') {
    this.state.drive.throttle = 0;
    this.state.drive.turn = 0;
    this.state.drive.speed = 0;
    this.state.deadman = false;
    this.state.event = event;
  }
  tick(dtSeconds) {
    const dt = clamp(Number(dtSeconds) || 0, 0, 0.1);
    this.state.elapsed += dt;
    this.state.linkAgeMs = this.state.connected ? Math.max(0, this.state.linkAgeMs - dt * 1000) : this.state.linkAgeMs + dt * 1000;
    const targetSpeed = this.state.deadman && this.state.connected && !this.state.safetyStop ? this.state.drive.throttle * 0.8 : 0;
    this.state.drive.speed += (targetSpeed - this.state.drive.speed) * Math.min(1, dt * 8);
    const turnRate = this.state.drive.turn * 75;
    this.state.pose.heading = wrapAngle(this.state.pose.heading + turnRate * dt);
    const radians = this.state.pose.heading * Math.PI / 180;
    this.state.pose.x = clamp(this.state.pose.x + Math.cos(radians) * this.state.drive.speed * dt, 0.04, 0.96);
    this.state.pose.y = clamp(this.state.pose.y + Math.sin(radians) * this.state.drive.speed * dt, 0.08, 0.92);
    this.state.distance += Math.abs(this.state.drive.speed * dt);
    this.state.battery = clamp(this.state.battery - Math.abs(this.state.drive.speed) * dt * 0.012, 0, 100);
    this.state.latencyMs = 38 + Math.round(Math.abs(Math.sin(this.state.elapsed * 0.8)) * 12);
    this.state.linkAgeMs = this.state.connected ? 0 : this.state.linkAgeMs;
    return this.snapshot();
  }
}

module.exports = { SimulationEngine, initialState, clamp, wrapAngle };
