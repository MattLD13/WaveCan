import { InputController } from './input/InputController.js';
import { createRenderer } from './ui/DashboardRenderer.js';

const sim = requireSimulation();
const renderer = createRenderer();
const input = new InputController({
  onDrive: (throttle, turn, deadman) => { sim.commandDrive(throttle, turn, deadman); },
  onDeadman: (held) => { if (!held) sim.stop('DEADMAN RELEASED'); updateDeadmanBadge(held); },
  onAction: (action, value) => {
    if (action === 'stop') { sim.setSafetyStop(true); setTimeout(() => sim.setSafetyStop(false), 700); }
    if (action === 'camera') { sim.cycleCamera(); advanceDemo(3); syncCameraButtons(); }
    if (action === 'link') sim.setConnected(!sim.state.connected);
    if (action === 'demo' || action === 'a') { sim.startDemo(); advanceDemo(1); }
    if (action === 'gamepad') setText('event-line', value ? 'GAMEPAD CONNECTED' : 'GAMEPAD DISCONNECTED • OUTPUT ZEROED');
  }
});

function requireSimulation() {
  // The browser file loader cannot synchronously import the CommonJS test model.
  // Keep this adapter readable and behaviorally identical to SimulationEngine.cjs;
  // both implement the same transport-safe simulation boundary and are tested via
  // the CJS model in tests/simulation.test.cjs.
  const clamp = (value, min, max) => Math.max(min, Math.min(max, value));
  class BrowserSimulation {
    constructor() {
      this.state = {
        connected: true, safetyStop: false, deadman: false,
        drive: { throttle: 0, turn: 0, speed: 0 },
        pose: { x: 0.12, y: 0.48, heading: 18 }, camera: 'front',
        battery: 94, latencyMs: 42, linkAgeMs: 0, elapsed: 0, distance: 0,
        demo: { active: false, step: 0, complete: false }, event: 'SIMULATION READY'
      };
    }
    commandDrive(throttle, turn, deadman) {
      this.state.deadman = Boolean(deadman);
      if (!deadman || !this.state.connected || this.state.safetyStop) {
        this.stop(deadman ? 'OUTPUT INHIBITED' : 'DEADMAN RELEASED'); return;
      }
      this.state.drive.throttle = clamp(Number(throttle) || 0, -1, 1);
      this.state.drive.turn = clamp(Number(turn) || 0, -1, 1);
      this.state.event = 'DRIVE COMMAND ACTIVE';
    }
    stop(event = 'STOPPED') {
      this.state.drive.throttle = 0; this.state.drive.turn = 0; this.state.drive.speed = 0;
      this.state.deadman = false; this.state.event = event;
    }
    setConnected(connected) {
      this.state.connected = Boolean(connected);
      if (!this.state.connected) this.stop('LINK LOST • OUTPUT ZEROED');
      else this.state.event = 'SIM LINK RESTORED';
    }
    setSafetyStop(stopped) {
      this.state.safetyStop = Boolean(stopped);
      if (stopped) this.stop('SAFETY STOP LATCHED'); else this.state.event = 'SAFETY STOP CLEARED';
    }
    cycleCamera() {
      const cameras = ['front', 'rear', 'arm', 'overhead'];
      const next = (cameras.indexOf(this.state.camera) + 1) % cameras.length;
      this.state.camera = cameras[next < 0 ? 0 : next];
    }
    startDemo() {
      this.state.demo = { active: true, step: 1, complete: false };
      this.state.event = 'DEMO STARTED • HOLD DEADMAN TO DRIVE';
    }
    tick(dtSeconds) {
      const dt = clamp(Number(dtSeconds) || 0, 0, 0.1);
      this.state.elapsed += dt;
      this.state.linkAgeMs = this.state.connected ? 0 : this.state.linkAgeMs + dt * 1000;
      const canDrive = this.state.deadman && this.state.connected && !this.state.safetyStop;
      const targetSpeed = canDrive ? this.state.drive.throttle * 0.8 : 0;
      this.state.drive.speed += (targetSpeed - this.state.drive.speed) * Math.min(1, dt * 8);
      this.state.pose.heading = (this.state.pose.heading + this.state.drive.turn * 75 * dt + 360) % 360;
      const radians = this.state.pose.heading * Math.PI / 180;
      this.state.pose.x = clamp(this.state.pose.x + Math.cos(radians) * this.state.drive.speed * dt, 0.04, 0.96);
      this.state.pose.y = clamp(this.state.pose.y + Math.sin(radians) * this.state.drive.speed * dt, 0.08, 0.92);
      this.state.distance += Math.abs(this.state.drive.speed * dt);
      this.state.battery = clamp(this.state.battery - Math.abs(this.state.drive.speed) * dt * 0.012, 0, 100);
      this.state.latencyMs = 38 + Math.round(Math.abs(Math.sin(this.state.elapsed * 0.8)) * 12);
      return this.state;
    }
  }
  return new BrowserSimulation();
}
const setText = (id, value) => { const node = document.getElementById(id); if (node) node.textContent = value; };
function updateDeadmanBadge(held) { const badge=document.getElementById('deadman-badge'); badge.textContent=held?'DEADMAN ON':'HOLD TO ENABLE'; badge.classList.toggle('on',held); }
function setMeter(id, value) { const node=document.getElementById(id); if(node) node.style.width=`${Math.min(100,Math.abs(value)*100)}%`; }
function advanceDemo(step){ if(!sim.state.demo.active)return; sim.state.demo.step=Math.max(sim.state.demo.step,step); if(sim.state.demo.step>=4&&sim.state.distance>.05){sim.state.demo.complete=true;sim.state.demo.active=false;sim.state.event='DEMO COMPLETE • COURSE CHECKPOINT REACHED';} }
function syncCameraButtons(){document.querySelectorAll('[data-camera]').forEach(b=>b.classList.toggle('active',b.dataset.camera===sim.state.camera));}

document.querySelectorAll('[data-camera]').forEach((button)=>button.addEventListener('click',()=>{sim.state.camera=button.dataset.camera;advanceDemo(3);syncCameraButtons();}));
document.getElementById('demo-button').addEventListener('click',()=>{sim.startDemo();advanceDemo(1);});
document.getElementById('sim-link-toggle').addEventListener('click',()=>sim.setConnected(!sim.state.connected));
document.getElementById('stop-button').addEventListener('click',()=>sim.setSafetyStop(!sim.state.safetyStop));
for(const button of document.querySelectorAll('[data-touch]')) { const value=button.dataset.touch; const start=()=>{ if(value==='stop'){sim.setSafetyStop(true);setTimeout(()=>sim.setSafetyStop(false),700);return;} input.setTouchAxis(value==='forward'||value==='back'?'throttle':'turn',value==='forward'?1:value==='back'?-1:value==='left'?-1:1); }; const end=()=>{input.setTouchAxis(value==='forward'||value==='back'?'throttle':'turn',0);}; button.addEventListener('pointerdown',start); button.addEventListener('pointerup',end); button.addEventListener('pointerleave',end); }
setInterval(()=>input.poll(),33);
let last=performance.now(); function frame(now){const dt=(now-last)/1000;last=now;const state=sim.tick(dt);renderer.render(state);setText('position',`${state.pose.x.toFixed(2)} / ${state.pose.y.toFixed(2)}`);setText('throttle-readout',state.drive.throttle.toFixed(2));setText('turn-readout',state.drive.turn.toFixed(2));setMeter('throttle-meter',state.drive.throttle);setMeter('turn-meter',state.drive.turn);requestAnimationFrame(frame);} requestAnimationFrame(frame);
