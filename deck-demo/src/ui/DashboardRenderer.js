const COLORS = { ink: '#e6f1f5', dim: '#86a2ad', cyan: '#61e7ef', green: '#7bea9a', amber: '#f6bd60', red: '#ff6b6b', grid: '#21404b' };
const cameras = { front: 'FRONT CAM', rear: 'REAR CAM', arm: 'ARM CAM', overhead: 'OVERHEAD CAM' };

const byId = (id) => document.getElementById(id);
const setText = (id, value) => { const node = byId(id); if (node) node.textContent = value; };
const fmt = (value, digits = 1) => Number(value).toFixed(digits);

function drawCamera(canvas, state) {
  // Original 2D front-mounted camera illustration: flat horizon, terrain, and obstacles.
  const ctx = canvas.getContext('2d'); const w = canvas.width; const h = canvas.height; const t = state.elapsed;
  ctx.fillStyle = '#0a1820'; ctx.fillRect(0, 0, w, h);
  const horizon = h * 0.42;
  const sky = ctx.createLinearGradient(0, 0, 0, horizon); sky.addColorStop(0, '#102e3b'); sky.addColorStop(1, '#2a6972');
  ctx.fillStyle = sky; ctx.fillRect(0, 0, w, horizon);
  ctx.fillStyle = '#285044'; ctx.fillRect(0, horizon, w, h - horizon);
  ctx.fillStyle = '#35695a'; ctx.beginPath(); ctx.moveTo(0, horizon + 22); ctx.lineTo(w * .16, horizon - 4); ctx.lineTo(w * .32, horizon + 17); ctx.lineTo(w * .51, horizon - 12); ctx.lineTo(w * .68, horizon + 14); ctx.lineTo(w * .84, horizon - 2); ctx.lineTo(w, horizon + 20); ctx.lineTo(w, h); ctx.lineTo(0, h); ctx.closePath(); ctx.fill();
  ctx.strokeStyle = 'rgba(97,231,239,.22)'; ctx.lineWidth = 1;
  for (let y = horizon + 42; y < h; y += 36) { ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(w, y); ctx.stroke(); }
  ctx.fillStyle = '#c8a36a';
  for (const rock of [[.18,.69,19],[.74,.64,25],[.56,.54,13],[.84,.79,33],[.32,.83,22]]) { ctx.beginPath(); ctx.ellipse(rock[0]*w, rock[1]*h, rock[2]*1.25, rock[2]*.72, 0, 0, Math.PI*2); ctx.fill(); }
  ctx.strokeStyle = COLORS.cyan; ctx.setLineDash([9, 9]); ctx.beginPath(); ctx.moveTo(w/2, h*.52); ctx.lineTo(w/2, h*.83); ctx.stroke(); ctx.setLineDash([]);
  ctx.fillStyle = 'rgba(7,16,25,.45)'; ctx.fillRect(0, h - 24, w, 24); ctx.fillStyle = COLORS.ink; ctx.font = '11px ui-monospace, monospace'; ctx.textAlign = 'left'; ctx.fillText(`FRONT CAMERA  •  ${Math.round(30 + Math.sin(t) * 2)} FPS`, 12, h - 9);
  if (state.camera !== 'front') { ctx.fillStyle = 'rgba(7,16,25,.72)'; ctx.fillRect(0,0,w,h); ctx.fillStyle = COLORS.cyan; ctx.font = 'bold 28px sans-serif'; ctx.textAlign = 'center'; ctx.fillText(`${cameras[state.camera]} • SIM VIEW`, w/2, h/2); }
}

function drawMap(canvas, state) {
  const ctx = canvas.getContext('2d'); const w = canvas.width; const h = canvas.height;
  ctx.fillStyle = '#0c1b23'; ctx.fillRect(0,0,w,h); ctx.strokeStyle = COLORS.grid; ctx.lineWidth = 1;
  for (let x=0;x<w;x+=24){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,h);ctx.stroke();} for (let y=0;y<h;y+=24){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(w,y);ctx.stroke();}
  ctx.strokeStyle = '#f6bd60'; ctx.lineWidth = 3; ctx.beginPath(); ctx.moveTo(w*.12,h*.76); ctx.bezierCurveTo(w*.28,h*.65,w*.31,h*.32,w*.52,h*.42); ctx.bezierCurveTo(w*.68,h*.55,w*.66,h*.2,w*.88,h*.24); ctx.stroke();
  ctx.fillStyle = COLORS.green; ctx.beginPath(); ctx.arc(w*.88,h*.24,6,0,Math.PI*2); ctx.fill();
  const x = state.pose.x*w; const y = state.pose.y*h; ctx.save(); ctx.translate(x,y); ctx.rotate(state.pose.heading*Math.PI/180); ctx.fillStyle = COLORS.cyan; ctx.beginPath(); ctx.moveTo(13,0); ctx.lineTo(-9,-8); ctx.lineTo(-9,8); ctx.closePath(); ctx.fill(); ctx.restore();
  ctx.fillStyle = COLORS.dim; ctx.font = '11px sans-serif'; ctx.fillText('START', 10, h-10); ctx.fillStyle = COLORS.green; ctx.fillText('CHECKPOINT', w*.74, h*.18);
}

export function createRenderer() {
  const cameraCanvas = byId('camera-canvas'); const mapCanvas = byId('map-canvas');
  const render = (state) => {
    drawCamera(cameraCanvas, state); drawMap(mapCanvas, state);
    setText('connection-label', state.connected ? 'SIM LINK' : 'LINK LOST'); setText('connection-detail', state.connected ? `${state.latencyMs} ms • 60 Hz` : 'OUTPUT ZEROED');
    byId('connection-dot')?.classList.toggle('offline', !state.connected);
    setText('battery', `${Math.round(state.battery)}%`); setText('camera-name', cameras[state.camera]); setText('speed', `${fmt(Math.abs(state.drive.speed), 2)} m/s`); setText('heading', `${Math.round(state.pose.heading).toString().padStart(3,'0')}°`); setText('distance', `${fmt(state.distance, 2)} m`); setText('link-age', `${Math.round(state.linkAgeMs)} ms`); setText('command-state', state.safetyStop ? 'SAFETY STOP' : (state.deadman ? 'DRIVE ACTIVE' : 'READY / HOLD TO ENABLE'));
    byId('no-downlink-image')?.classList.toggle('visible', !state.connected);
    const commsVisor = byId('module-comms-visor'); commsVisor?.classList.remove('green', 'yellow', 'red'); commsVisor?.classList.add(state.connected ? 'green' : (state.linkAgeMs > 500 ? 'red' : 'yellow'));
    setText('module-comms-detail', state.connected ? `SIM LINK / ${state.latencyMs} ms` : (state.linkAgeMs > 500 ? 'DISCONNECTED / ZEROED' : 'COMMS LOST / ZEROED'));
    setText('module-drive-detail', state.deadman ? 'DRIVE ACTIVE' : 'HOLD TO ENABLE'); byId('module-drive')?.classList.toggle('active', state.deadman);
    const moduleSafety = byId('module-safety')?.querySelector('.visor'); moduleSafety?.classList.toggle('red', state.safetyStop); moduleSafety?.classList.toggle('green', !state.safetyStop);
    document.querySelectorAll('[data-phase]').forEach((node) => { const phase = Number(node.dataset.phase); node.classList.toggle('current', phase === (state.demo.active ? state.demo.step : 1)); node.classList.toggle('done', state.demo.complete || (state.demo.active && phase < state.demo.step)); });
    const stop = byId('stop-button'); stop?.classList.toggle('latched', state.safetyStop); stop.textContent = state.safetyStop ? 'CLEAR STOP' : 'STOP OUTPUT';
    const event = byId('event-line'); event.textContent = state.event;
    byId('sim-link-toggle')?.classList.toggle('on', state.connected);
    const progress = state.demo.complete ? 4 : state.demo.step; document.querySelectorAll('[data-demo-step]').forEach((node) => node.classList.toggle('done', Number(node.dataset.demoStep) < progress));
    byId('demo-status').textContent = state.demo.complete ? 'DEMO COMPLETE' : (state.demo.active ? `GUIDED DEMO • STEP ${state.demo.step}/4` : 'GUIDED DEMO READY');
    byId('drive-meter').style.width = `${Math.min(100, Math.abs(state.drive.speed) / .8 * 100)}%`;
  };
  return { render };
}
