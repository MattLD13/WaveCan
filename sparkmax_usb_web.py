"""Browser UI for six SPARK MAX controllers over one USB-to-CAN bridge."""

from __future__ import annotations

import argparse
import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any

from sparkmax_usb_controller import SparkMaxUsbController
from xbox_xinput import XboxXInput


HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SPARK MAX USB Motor Control</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#111827;color:#f9fafb}
main{max-width:980px;margin:auto;padding:24px}h1{margin:0 0 6px;font-size:28px}.sub{color:#9ca3af;margin-bottom:18px}
.bar,.card{background:#1f2937;border:1px solid #374151;border-radius:12px}.bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:14px;margin-bottom:18px}
#connection{color:#fbbf24;font-weight:700;margin-right:auto}button{border:0;border-radius:8px;padding:10px 14px;color:#fff;background:#2563eb;cursor:pointer;font-weight:700}button.stop{background:#dc2626}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.card{padding:16px}.head{display:flex;justify-content:space-between;align-items:baseline}.name{font-size:20px;font-weight:800}.power{font-variant-numeric:tabular-nums;color:#93c5fd;font-weight:800}
.meta{color:#9ca3af;font-size:13px;margin:6px 0 14px}input{width:100%;accent-color:#60a5fa}.actions{display:flex;justify-content:flex-end;margin-top:12px}button.small{padding:8px 10px;font-size:13px}#message{color:#fca5a5;margin-top:16px;min-height:1.4em}
</style></head><body><main><h1>SPARK MAX USB Motor Control</h1><div class="sub">USB-connected SPARK MAX bridge · CAN IDs 1–6 · keep the mechanism clear</div>
<section class="bar"><span id="connection">Connecting…</span><button class="stop" onclick="stopAll()">EMERGENCY STOP</button></section>
<section id="motors" class="grid"></section><div id="message"></div></main>
<script>
const root=document.querySelector('#motors'),connection=document.querySelector('#connection'),message=document.querySelector('#message');
function pct(v){return `${(Number(v)*100).toFixed(0)}%`} function error(e){message.textContent=String(e)}
function render(s){connection.textContent=s.connected?`Connected via ${s.port} · USB-CAN bridge`:'No SPARK MAX USB bridge detected';
for(const m of s.motors){let c=document.querySelector(`[data-id="${m.id}"]`);if(!c){c=document.createElement('article');c.className='card';c.dataset.id=m.id;c.innerHTML=`<div class="head"><span class="name">CAN ID ${m.id}</span><span class="power"></span></div><div class="meta"></div><input type="range" min="-1" max="1" step="0.01"><div class="actions"><button class="small stop">Stop motor</button></div>`;const slider=c.querySelector('input');slider.oninput=()=>c.querySelector('.power').textContent=pct(slider.value);slider.onchange=()=>setMotor(m.id,slider.value);c.querySelector('button').onclick=()=>setMotor(m.id,0);root.appendChild(c)}c.querySelector('.meta').textContent=m.direct_usb?'Direct USB target':'USB-to-CAN bridge target';c.querySelector('input').value=m.power;c.querySelector('.power').textContent=pct(m.power)}if(s.last_error)message.textContent=s.last_error}
async function refresh(){try{const r=await fetch('/api/status');const s=await r.json();if(!r.ok)throw Error(s.error||'Status failed');render(s)}catch(e){error(e)}}
async function post(path,data={}){message.textContent='';try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const x=await r.json();if(!r.ok)throw Error(x.error||'Command failed');render(x)}catch(e){error(e)}}
function setMotor(id,power){post('/api/motor',{id,power:Number(power)})}function stopAll(){post('/api/stop')}refresh();setInterval(refresh,750);
</script></body></html>"""


# The USB page intentionally mirrors WaveCan's arcade/tank drive workflow:
# drive inputs are blended in the browser and sent as one role-mapped update.
HTML = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SPARK MAX USB Motor Control</title>
<style>
:root{color-scheme:dark;font-family:system-ui,sans-serif}body{margin:0;background:#111827;color:#f9fafb}
main{max-width:1080px;margin:auto;padding:24px}h1{margin:0 0 6px;font-size:28px}.sub{color:#9ca3af;margin-bottom:18px}
.bar,.card{background:#1f2937;border:1px solid #374151;border-radius:12px}.bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;padding:14px;margin-bottom:18px}
#connection{color:#fbbf24;font-weight:700;margin-right:auto}button{border:0;border-radius:8px;padding:10px 14px;color:#fff;background:#2563eb;cursor:pointer;font-weight:700}button.stop{background:#dc2626}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.card{padding:16px}.head{display:flex;justify-content:space-between;align-items:baseline}.name{font-size:20px;font-weight:800}.power{font-variant-numeric:tabular-nums;color:#93c5fd;font-weight:800}
.meta{color:#9ca3af;font-size:13px;margin:6px 0 14px}input{width:100%;accent-color:#60a5fa}.actions{display:flex;justify-content:flex-end;margin-top:12px}button.small{padding:8px 10px;font-size:13px}#message{color:#fca5a5;margin-top:16px;min-height:1.4em}
.drive{margin-bottom:18px}.drive-top{display:flex;gap:10px;align-items:center;flex-wrap:wrap}.drive-top .name{margin-right:auto}.mode{background:#374151;padding:8px 12px;font-size:13px}.mode.active{background:#2563eb}.mode.off.active{background:#4b5563}.drive-layout{display:flex;gap:22px;align-items:center;flex-wrap:wrap;margin-top:16px}.stick-wrap{text-align:center}.stick{width:180px;height:180px;display:block;background:#111827;border:1px solid #4b5563;border-radius:50%;touch-action:none}.drive-values{font-variant-numeric:tabular-nums;color:#93c5fd;font-size:13px;margin-top:6px}.drive-help{color:#9ca3af;font-size:13px;line-height:1.7;min-width:220px}.drive-help kbd{background:#111827;border:1px solid #4b5563;border-radius:4px;padding:2px 5px;color:#fff}.settings{display:flex;gap:14px;flex-wrap:wrap;margin-top:16px;color:#9ca3af;font-size:13px}.settings label{display:flex;align-items:center;gap:8px}.settings input{width:130px}.tank{display:none;gap:16px;align-items:center}.tank-column{text-align:center;color:#9ca3af;font-size:13px}.tank input{width:180px;transform:rotate(-90deg);margin:80px -58px}.mapping{margin-top:16px;border-top:1px solid #374151;padding-top:12px}.mapping-title{color:#d1d5db;font-weight:700;font-size:13px;margin-bottom:8px}.role-row{display:flex;align-items:center;gap:8px;margin:5px 0;color:#9ca3af;font-size:13px}.role-row select,.role-select{background:#111827;color:#f9fafb;border:1px solid #4b5563;border-radius:6px;padding:5px}.role-row select{margin-left:auto}.reverse{display:flex;align-items:center;gap:6px;color:#9ca3af;font-size:13px;margin:8px 0}.reverse input{width:auto}
</style></head><body><main><h1>SPARK MAX USB Motor Control</h1><div class="sub">USB-connected SPARK MAX bridge · CAN IDs 1–6 · keep the mechanism clear</div>
<section class="bar"><span id="connection">Connecting…</span><span id="gamepad">No Gamepad</span><button class="stop" onclick="stopAll()">EMERGENCY STOP</button></section>
<section class="card drive"><div class="drive-top"><span class="name">Drive Control</span><button class="mode" id="arcadeMode" onclick="setDriveMode('arcade')">Arcade Drive</button><button class="mode" id="tankMode" onclick="setDriveMode('tank')">Tank Drive</button><button class="mode off active" id="offMode" onclick="setDriveMode('off')">Disabled</button></div>
<div class="drive-layout"><div class="stick-wrap" id="arcadePanel"><div>Left Stick</div><canvas id="stick" class="stick" width="180" height="180"></canvas><div class="drive-values">Throttle <span id="throttleValue">0%</span> · Turn <span id="turnValue">0%</span></div></div>
<div class="tank" id="tankPanel"><div class="tank-column">Left<input id="tankLeft" type="range" min="-1" max="1" step="0.01" value="0"><div id="tankLeftValue">0%</div></div><div class="tank-column">Right<input id="tankRight" type="range" min="-1" max="1" step="0.01" value="0"><div id="tankRightValue">0%</div></div></div>
<div class="drive-help"><div><kbd>W</kbd>/<kbd>S</kbd> throttle · <kbd>A</kbd>/<kbd>D</kbd> turn</div><div>Gamepad: left stick for arcade; left/right sticks for tank</div><div>Arcade mixing: left = throttle + turn, right = throttle − turn</div></div></div>
<div class="settings"><label>Deadzone <input id="deadzone" type="range" min="0" max="30" step="1" value="8"><span id="deadzoneValue">8%</span></label><label><input id="invertMove" type="checkbox"> invert throttle</label><label><input id="invertTurn" type="checkbox"> invert turn</label></div>
<div class="mapping"><div class="mapping-title">Drive motor mapping</div><div id="mappingRows"></div></div></section>
<section id="motors" class="grid"></section><div id="message"></div></main>
<script>
const root=document.querySelector('#motors'),connection=document.querySelector('#connection'),message=document.querySelector('#message');
const savedMotorInvert=JSON.parse(localStorage.getItem('sparkmax-motor-invert')||'{}');const state={mode:'off',roles:{1:'L',2:'R',3:'AUX',4:'AUX',5:'FL',6:'FR'},invertMotor:savedMotorInvert,throttle:0,turn:0,left:0,right:0,steerLeft:0,steerRight:0,deadzone:.08,invertMove:false,invertTurn:false,ui:{throttle:0,turn:0,left:0,right:0},kb:{throttle:0,turn:0},gp:{throttle:0,turn:0,left:0,right:0},xgp:{throttle:0,turn:0,left:0,right:0},driveBusy:false,driveToken:0,gamepadIndex:null};
const roleOptions=[['NONE','None'],['L','Drive Left'],['R','Drive Right'],['FL','Steer Left'],['FR','Steer Right'],['AUX','Auxiliary']];const keys=new Set();
function pct(v){return `${(Number(v)*100).toFixed(0)}%`}function clamp(v){return Math.max(-1,Math.min(1,Number(v)||0))}function deadzone(v){const a=Math.abs(v);if(a<=state.deadzone)return 0;return Math.sign(v)*(a-state.deadzone)/(1-state.deadzone)}function error(e){message.textContent=String(e)}
function roleSelect(id){const s=document.createElement('select');s.className='role-select';for(const [v,label] of roleOptions){const o=document.createElement('option');o.value=v;o.textContent=label;s.appendChild(o)}s.value=state.roles[id]||'NONE';s.onchange=()=>state.roles[id]=s.value==='NONE'?null:s.value;return s}
function renderMapping(ids){const rows=document.querySelector('#mappingRows');rows.innerHTML='';for(const id of ids){const row=document.createElement('div');row.className='role-row';row.textContent=`CAN ID ${id}`;row.appendChild(roleSelect(id));rows.appendChild(row)}}
function render(s){const onlineIds=s.online_ids||[];connection.textContent=s.connected?`Connected via ${s.port} · USB-CAN bridge · CAN online: ${onlineIds.join(', ')||'checking'}`:'No SPARK MAX USB bridge detected';const ids=[];for(const m of s.motors){ids.push(m.id);let c=document.querySelector(`[data-id="${m.id}"]`);if(!c){c=document.createElement('article');c.className='card';c.dataset.id=m.id;c.innerHTML=`<div class="head"><span class="name">CAN ID ${m.id}</span><span class="power"></span></div><div class="meta"></div><label class="reverse"><input type="checkbox"> Reverse direction</label><select class="role-select"></select><input class="power-slider" type="range" min="-1" max="1" step="0.01"><div class="actions"><button class="small stop">Stop motor</button></div>`;const slider=c.querySelector('input[type=range]');slider.oninput=()=>c.querySelector('.power').textContent=pct(slider.value);slider.onchange=()=>setMotor(m.id,slider.value);c.querySelector('.stop').onclick=()=>setMotor(m.id,0);const reverse=c.querySelector('input[type=checkbox]');reverse.onchange=()=>{state.invertMotor[m.id]=reverse.checked;localStorage.setItem('sparkmax-motor-invert',JSON.stringify(state.invertMotor))};const select=c.querySelector('select');for(const [v,label] of roleOptions){const o=document.createElement('option');o.value=v;o.textContent=label;select.appendChild(o)}select.onchange=()=>state.roles[m.id]=select.value==='NONE'?null:select.value;root.appendChild(c)}const shownPower=state.invertMotor[m.id]?-m.power:m.power;c.querySelector('.meta').textContent=`${onlineIds.includes(m.id)?'CAN online':'No CAN response'} · ${m.direct_usb?'Direct USB target':'USB-to-CAN bridge target'}`;c.querySelector('input[type=checkbox]').checked=!!state.invertMotor[m.id];c.querySelector('input[type=range]').value=shownPower;c.querySelector('.power').textContent=pct(shownPower);c.querySelector('select').value=state.roles[m.id]||'NONE'}renderMapping(ids);if(s.last_error)message.textContent=s.last_error}
async function refresh(){try{const r=await fetch('/api/status');const s=await r.json();if(!r.ok)throw Error(s.error||'Status failed');render(s)}catch(e){error(e)}}async function post(path,data={}){message.textContent='';try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const x=await r.json();if(!r.ok)throw Error(x.error||'Command failed');render(x);return x}catch(e){error(e);return null}}
function setMotor(id,power){const requested=Number(power)||0;post('/api/motor',{id,power:state.invertMotor[id]?-requested:requested})}async function stopAll(){state.mode='off';state.driveToken++;state.throttle=state.turn=state.left=state.right=0;state.ui={throttle:0,turn:0,left:0,right:0};state.kb={throttle:0,turn:0};state.gp={throttle:0,turn:0,left:0,right:0};updateDriveUi();await post('/api/stop')}
function setDriveMode(mode){if(mode==='off'){stopAll();return}state.mode=mode;state.driveToken++;state.throttle=state.turn=state.left=state.right=0;state.ui={throttle:0,turn:0,left:0,right:0};updateDriveUi();sendDrive(true)}
function updateDriveUi(){for(const [id,on] of [['arcadeMode',state.mode==='arcade'],['tankMode',state.mode==='tank'],['offMode',state.mode==='off']])document.querySelector('#'+id).classList.toggle('active',on);document.querySelector('#arcadePanel').style.display=state.mode==='tank'?'none':'block';document.querySelector('#tankPanel').style.display=state.mode==='tank'?'flex':'none';document.querySelector('#throttleValue').textContent=pct(state.throttle);document.querySelector('#turnValue').textContent=pct(state.turn);document.querySelector('#tankLeftValue').textContent=pct(state.left);document.querySelector('#tankRightValue').textContent=pct(state.right);drawStick()}
function chooseInput(){if(Math.abs(state.gp.throttle)>.01||Math.abs(state.gp.turn)>.01||Math.abs(state.gp.left)>.01||Math.abs(state.gp.right)>.01)return state.gp;if(Math.abs(state.xgp.throttle)>.01||Math.abs(state.xgp.turn)>.01||Math.abs(state.xgp.left)>.01||Math.abs(state.xgp.right)>.01)return state.xgp;if(Math.abs(state.ui.throttle)>.01||Math.abs(state.ui.turn)>.01||Math.abs(state.ui.left)>.01||Math.abs(state.ui.right)>.01)return state.ui;return state.kb}
async function sendDrive(force=false){if(state.mode==='off'||state.driveBusy)return;const input=chooseInput();let left,right;if(state.mode==='arcade'){state.throttle+=(input.throttle-state.throttle)*.35;state.turn+=(input.turn-state.turn)*.35;left=clamp(state.throttle+state.turn);right=clamp(state.throttle-state.turn)}else{state.left+=(input.left-state.left)*.35;state.right+=(input.right-state.right)*.35;left=state.left;right=state.right}state.throttle=state.mode==='arcade'?state.throttle:0;state.turn=state.mode==='arcade'?state.turn:0;state.left=state.mode==='tank'?state.left:0;state.right=state.mode==='tank'?state.right:0;updateDriveUi();const token=state.driveToken;state.driveBusy=true;try{await post('/api/drive',{left,right,steer_left:state.steerLeft,steer_right:state.steerRight,roles:state.roles,inverted:state.invertMotor})}finally{state.driveBusy=false;if(token!==state.driveToken){} }}
function applyStick(x,y){const c=document.querySelector('#stick'),r=c.getBoundingClientRect(),dx=(x-r.left)/r.width*2-1,dy=(y-r.top)/r.height*2-1,len=Math.hypot(dx,dy),xx=len>1?dx/len:dx,yy=len>1?dy/len:dy;state.ui.turn=deadzone(xx);state.ui.throttle=deadzone(-yy);updateDriveUi()}let dragging=false;const stick=document.querySelector('#stick');stick.addEventListener('pointerdown',e=>{dragging=true;stick.setPointerCapture(e.pointerId);applyStick(e.clientX,e.clientY)});stick.addEventListener('pointermove',e=>{if(dragging)applyStick(e.clientX,e.clientY)});stick.addEventListener('pointerup',()=>{dragging=false;state.ui.throttle=state.ui.turn=0});stick.addEventListener('pointercancel',()=>{dragging=false;state.ui.throttle=state.ui.turn=0});
document.querySelector('#tankLeft').oninput=e=>{state.ui.left=deadzone(Number(e.target.value));updateDriveUi()};document.querySelector('#tankRight').oninput=e=>{state.ui.right=deadzone(Number(e.target.value));updateDriveUi()};document.querySelector('#deadzone').oninput=e=>{state.deadzone=Number(e.target.value)/100;document.querySelector('#deadzoneValue').textContent=e.target.value+'%'};document.querySelector('#invertMove').onchange=e=>state.invertMove=e.target.checked;document.querySelector('#invertTurn').onchange=e=>state.invertTurn=e.target.checked;
document.addEventListener('keydown',e=>{if(['INPUT','SELECT','TEXTAREA'].includes(e.target.tagName))return;if(e.code==='Space'){e.preventDefault();stopAll();return}keys.add(e.key)});document.addEventListener('keyup',e=>keys.delete(e.key));setInterval(()=>{if(state.mode==='off')return;let t=state.kb.throttle,turn=state.kb.turn;const up=keys.has('w')||keys.has('W')||keys.has('ArrowUp'),down=keys.has('s')||keys.has('S')||keys.has('ArrowDown'),left=keys.has('a')||keys.has('A')||keys.has('ArrowLeft'),right=keys.has('d')||keys.has('D')||keys.has('ArrowRight');if(up)t=Math.min(1,t+.05);if(down)t=Math.max(-1,t-.05);if(left)turn=Math.max(-1,turn-.05);if(right)turn=Math.min(1,turn+.05);if(!up&&!down)t*=.8;if(!left&&!right)turn*=.8;state.kb.throttle=deadzone(state.invertMove?-t:t);state.kb.turn=deadzone(state.invertTurn?-turn:turn)},50);
function pollGamepad(){const pads=navigator.getGamepads?navigator.getGamepads():[];const gp=state.gamepadIndex===null?Array.from(pads).find(Boolean):pads[state.gamepadIndex];if(gp){state.gamepadIndex=gp.index;document.querySelector('#gamepad').textContent='Gamepad connected';const move=deadzone(state.invertMove?gp.axes[1]||0:-(gp.axes[1]||0)),turn=deadzone(state.invertTurn?-(gp.axes[0]||0):gp.axes[0]||0);state.gp={throttle:move,turn,left:move,right:deadzone(-(gp.axes[2]||0))}}else{state.gamepadIndex=null;state.gp={throttle:0,turn:0,left:0,right:0};if(!state.xinputConnected)document.querySelector('#gamepad').textContent='No Gamepad'}requestAnimationFrame(pollGamepad)}
state.xinputConnected=false;async function pollXInput(){try{const r=await fetch('/api/controller');const x=await r.json();state.xinputConnected=!!x.connected;if(x.connected){const a=x.axes||{};const move=deadzone(state.invertMove?-(a.move||0):a.move||0),turn=deadzone(state.invertTurn?-(a.turn||0):a.turn||0);state.xgp={throttle:move,turn,left:move,right:deadzone(a.tank_right||0)};document.querySelector('#gamepad').textContent='Xbox controller connected (Bluetooth/XInput)'}else{state.xgp={throttle:0,turn:0,left:0,right:0};if(!state.gamepadIndex)document.querySelector('#gamepad').textContent='No Gamepad'}}catch(_){state.xinputConnected=false}}pollGamepad();pollXInput();setInterval(pollXInput,100);setInterval(()=>{if(state.mode!=='off')sendDrive()},60);function drawStick(){const c=document.querySelector('#stick'),x=c.getContext('2d'),w=c.width,h=c.height,r=72,cx=w/2,cy=h/2;x.clearRect(0,0,w,h);x.fillStyle='#111827';x.fillRect(0,0,w,h);x.strokeStyle='#4b5563';x.beginPath();x.arc(cx,cy,r,0,Math.PI*2);x.stroke();x.beginPath();x.moveTo(cx,cy-r);x.lineTo(cx,cy+r);x.moveTo(cx-r,cy);x.lineTo(cx+r,cy);x.stroke();const sx=cx+state.turn*r,sy=cy-state.throttle*r;x.fillStyle=Math.abs(state.throttle)+Math.abs(state.turn)>.1?'#60a5fa':'#9ca3af';x.beginPath();x.arc(sx,sy,10,0,Math.PI*2);x.fill()};updateDriveUi();refresh();setInterval(refresh,750);
</script></body></html>"""

class SparkWebServer(ThreadingHTTPServer):
    def __init__(self, address: tuple[str, int], controller: SparkMaxUsbController, xinput: XboxXInput):
        super().__init__(address, SparkRequestHandler)
        self.controller = controller
        self.xinput = xinput


class SparkRequestHandler(BaseHTTPRequestHandler):
    server: SparkWebServer

    def log_message(self, format: str, *args: Any) -> None:
        print(f"[SPARK MAX web] {format % args}")

    def _send(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _json(self, status: int, value: Any) -> None:
        self._send(status, "application/json; charset=utf-8", json.dumps(value).encode())

    def _body(self) -> dict[str, Any]:
        length = int(self.headers.get("Content-Length", "0"))
        if length > 4096:
            raise ValueError("request body too large")
        value = json.loads(self.rfile.read(length).decode() if length else "{}")
        if not isinstance(value, dict):
            raise ValueError("request body must be an object")
        return value

    def do_GET(self) -> None:
        if self.path in {"/", "/index.html"}:
            self._send(200, "text/html; charset=utf-8", HTML.encode())
        elif self.path == "/api/status":
            self._json(200, self.server.controller.get_states())
        elif self.path == "/api/controller":
            self._json(200, self.server.xinput.get_state())
        else:
            self._json(404, {"error": "not found"})

    def do_POST(self) -> None:
        try:
            data = self._body()
            if self.path == "/api/motor":
                self.server.controller.set_motor(int(data["id"]), float(data["power"]))
            elif self.path == "/api/drive":
                raw_roles = data.get("roles", {})
                if not isinstance(raw_roles, dict):
                    raise ValueError("roles must be an object")
                roles = {int(motor_id): str(role) for motor_id, role in raw_roles.items()}
                raw_inverted = data.get("inverted", {})
                if not isinstance(raw_inverted, dict):
                    raise ValueError("inverted must be an object")
                inverted = {int(motor_id): bool(value) for motor_id, value in raw_inverted.items()}
                self.server.controller.set_drive(
                    float(data.get("left", 0.0)),
                    float(data.get("right", 0.0)),
                    float(data.get("steer_left", 0.0)),
                    float(data.get("steer_right", 0.0)),
                    roles,
                    inverted,
                )
            elif self.path == "/api/stop":
                self.server.controller.stop()
            else:
                self._json(404, {"error": "not found"})
                return
            self._json(200, self.server.controller.get_states())
        except (KeyError, TypeError, ValueError) as exc:
            self._json(400, {"error": str(exc)})
        except Exception as exc:
            self._json(500, {"error": str(exc)})


def main() -> int:
    parser = argparse.ArgumentParser(description="Run a browser UI for SPARK MAX controllers over USB")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8081)
    parser.add_argument("--usb-device", default=None, help="USB serial number or device selector")
    parser.add_argument("--open-browser", action="store_true")
    args = parser.parse_args()

    controller = SparkMaxUsbController(range(1, 7))
    xinput = XboxXInput()
    try:
        controller.connect(args.usb_device)
        server = SparkWebServer((args.host, args.port), controller, xinput)
        browser_host = "127.0.0.1" if args.host == "0.0.0.0" else args.host
        url = f"http://{browser_host}:{args.port}/"
        print(f"SPARK MAX browser UI: {url}")
        if args.open_browser:
            threading.Timer(0.2, lambda: webbrowser.open(url)).start()
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            print("\nStopping SPARK MAX browser UI…")
        finally:
            server.server_close()
        return 0
    except Exception as exc:
        print(f"SPARK MAX USB controller error: {exc}")
        return 1
    finally:
        controller.close()


if __name__ == "__main__":
    raise SystemExit(main())
