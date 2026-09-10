const DEADMAN_KEYS = new Set(['Space', 'ShiftLeft', 'ShiftRight']);
const KEY_TO_AXIS = {
  KeyW: ['throttle', 1], ArrowUp: ['throttle', 1], KeyS: ['throttle', -1], ArrowDown: ['throttle', -1],
  KeyA: ['turn', -1], ArrowLeft: ['turn', -1], KeyD: ['turn', 1], ArrowRight: ['turn', 1]
};

export const B_HOLD_MS = 1200;

const deadzone = (value, threshold = 0.12) => {
  const amount = Number(value) || 0;
  if (Math.abs(amount) <= threshold) return 0;
  return Math.sign(amount) * ((Math.abs(amount) - threshold) / (1 - threshold));
};

export const bHoldReached = (startedAt, now, threshold = B_HOLD_MS) => Number.isFinite(startedAt) && Number(now) - startedAt >= threshold;

const vendorNames = { '28de': 'VALVE / STEAM DECK', '045e': 'XBOX', '054c': 'SONY', '057e': 'NINTENDO' };

const vendorIdFor = (id) => {
  const text = String(id || '');
  const labeled = text.match(/(?:vendor|vid|vendorid)[^0-9a-f]*([0-9a-f]{4})/i);
  if (labeled) return labeled[1].toLowerCase();
  const pair = text.match(/\b([0-9a-f]{4})[-_:][0-9a-f]{4}\b/i);
  return pair ? pair[1].toLowerCase() : (/valve|steam deck|steam virtual/i.test(text) ? '28de' : null);
};

export const describeGamepad = (pad) => {
  if (!pad) return { connected: false, label: 'NO CONTROLLER', mapping: '—', vendorId: null, id: '' };
  const id = String(pad.id || 'UNKNOWN CONTROLLER');
  const vendorId = vendorIdFor(id);
  const vendor = vendorNames[vendorId] || (/valve|steam|deck/i.test(id) ? 'STEAM DECK' : 'GENERIC GAMEPAD');
  const mapping = pad.mapping === 'standard' ? 'STANDARD MAPPING' : (pad.mapping ? String(pad.mapping).toUpperCase() : 'UNSPECIFIED MAPPING');
  return { connected: true, label: `${vendor} • ${mapping}`, mapping, vendorId, id };
};

const now = () => (typeof performance !== 'undefined' && typeof performance.now === 'function' ? performance.now() : Date.now());

/**
 * Browser-only input adapter. It emits local intent to the simulation model;
 * it never opens a device, radio, serial, CAN, ROS, or network path.
 */
export class InputController {
  constructor({ onDrive = () => {}, onLook = () => {}, onAction = () => {}, onDeadman = () => {} } = {}) {
    this.onDrive = onDrive;
    this.onLook = onLook;
    this.onAction = onAction;
    this.onDeadman = onDeadman;
    this.keys = new Set();
    this.touch = { throttle: 0, turn: 0 };
    this.touchDeadman = false;
    this.gamepad = null;
    this.deadman = false;
    this.lastGamepad = { a: false, b: false, x: false, y: false, l1: false, r1: false };
    this.controllerInfo = describeGamepad(null);
    this.bKeyTimer = null;
    this.bKeyStartedAt = null;
    this.bPadStartedAt = null;
    this.bPadTriggered = false;
    this.bindKeyboard();
    this.bindGamepadEvents();
    window.addEventListener('blur', () => this.stop('WINDOW BLUR • OUTPUT ZEROED'));
    document.addEventListener('visibilitychange', () => { if (document.hidden) this.stop('APP HIDDEN • OUTPUT ZEROED'); });
  }

  _announceController(pad) {
    const info = describeGamepad(pad);
    const signature = `${info.connected}:${info.id}:${info.mapping}`;
    const previous = `${this.controllerInfo.connected}:${this.controllerInfo.id}:${this.controllerInfo.mapping}`;
    if (signature !== previous) {
      this.controllerInfo = info;
      this.onAction('controller', info);
    }
    return info;
  }

  bindKeyboard() {
    window.addEventListener('keydown', (event) => {
      if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.code)) event.preventDefault();
      if (event.repeat && ['Escape', 'KeyX', 'KeyC', 'KeyL', 'KeyH', 'Enter'].includes(event.code)) return;
      this.keys.add(event.code);
      if (event.code === 'Escape') this.onAction('stop', 'ESC • SIM E-STOP');
      if (event.code === 'KeyB' && this.bKeyStartedAt == null) {
        this.bKeyStartedAt = now();
        this.bKeyTimer = window.setTimeout(() => {
          if (this.keys.has('KeyB')) this.onAction('stop', 'B HOLD 1.2S • SIM E-STOP');
        }, B_HOLD_MS);
      }
      if (event.code === 'KeyC' || event.code === 'KeyX') this.onAction('camera');
      if (event.code === 'KeyL') this.onAction('link');
      if (event.code === 'KeyH' || event.code === 'Enter') this.onAction('context');
      this.updateDeadman();
    });
    window.addEventListener('keyup', (event) => {
      this.keys.delete(event.code);
      if (event.code === 'KeyB') {
        if (this.bKeyTimer) window.clearTimeout(this.bKeyTimer);
        this.bKeyTimer = null;
        this.bKeyStartedAt = null;
      }
      this.updateDeadman();
    });
  }

  bindGamepadEvents() {
    window.addEventListener('gamepadconnected', (event) => {
      this.gamepad = event.gamepad;
      this._announceController(this.gamepad);
      this.onAction('gamepad', true);
    });
    window.addEventListener('gamepaddisconnected', () => {
      this.gamepad = null;
      this._announceController(null);
      this.onAction('gamepad', false);
      this.stop('GAMEPAD DISCONNECTED • OUTPUT ZEROED');
    });
  }

  _keyboardDeadman() { return [...DEADMAN_KEYS].some((key) => this.keys.has(key)); }

  updateDeadman(next = this._keyboardDeadman() || this.touchDeadman) {
    const normalized = Boolean(next);
    if (normalized !== this.deadman) {
      this.deadman = normalized;
      this.onDeadman(normalized);
    }
  }

  setTouchAxis(axis, value) {
    if (!(axis in this.touch)) return;
    this.touch[axis] = Math.max(-1, Math.min(1, Number(value) || 0));
    this.touchDeadman = Object.values(this.touch).some((entry) => Math.abs(entry) > 0.01);
    this.updateDeadman();
  }

  stop(reason = 'INPUT STOPPED') {
    this.keys.clear();
    this.touch = { throttle: 0, turn: 0 };
    this.touchDeadman = false;
    this.deadman = false;
    this.onDeadman(false);
    this.onLook(0, 0);
    this.onAction('stop', reason);
  }

  _readGamepad() {
    const gamepads = navigator.getGamepads ? [...navigator.getGamepads()] : [];
    let pad = this.gamepad;
    if (pad && Number.isInteger(pad.index) && gamepads[pad.index]) pad = gamepads[pad.index];
    if (!pad) pad = gamepads.find(Boolean) || null;
    this.gamepad = pad;
    this._announceController(pad);
    return pad;
  }

  poll() {
    const pad = this._readGamepad();
    let throttle = 0;
    let turn = 0;
    let deadman = this._keyboardDeadman() || this.touchDeadman;
    let lookX = 0;
    let lookY = 0;
    for (const [code, [axis, sign]] of Object.entries(KEY_TO_AXIS)) {
      if (this.keys.has(code)) {
        if (axis === 'throttle') throttle += sign;
        else turn += sign;
      }
    }
    throttle = Math.max(-1, Math.min(1, throttle + this.touch.throttle));
    turn = Math.max(-1, Math.min(1, turn + this.touch.turn));
    if (pad) {
      const leftX = deadzone(pad.axes?.[0]);
      const leftY = deadzone(pad.axes?.[1]);
      lookX = deadzone(pad.axes?.[2]);
      lookY = deadzone(pad.axes?.[3]);
      if (leftX || leftY) { turn = leftX; throttle = -leftY; }
      const triggerHeld = Number(pad.buttons?.[7]?.value) > 0.15 || Boolean(pad.buttons?.[5]?.pressed);
      deadman = Boolean(triggerHeld || deadman);
      this.updateDeadman(deadman);
      const buttons = { a: 0, b: 1, x: 2, y: 3, l1: 4, r1: 5 };
      const pressedB = Boolean(pad.buttons?.[buttons.b]?.pressed);
      if (pressedB && !this.lastGamepad.b) { this.bPadStartedAt = now(); this.bPadTriggered = false; }
      if (pressedB && !this.bPadTriggered && bHoldReached(this.bPadStartedAt, now())) {
        this.bPadTriggered = true;
        this.onAction('stop', 'B HOLD 1.2S • SIM E-STOP');
      }
      if (!pressedB) { this.bPadStartedAt = null; this.bPadTriggered = false; }
      for (const [name, index] of Object.entries(buttons)) {
        const pressed = Boolean(pad.buttons?.[index]?.pressed);
        if (pressed && !this.lastGamepad[name]) {
          if (name === 'a') this.onAction('a');
          else if (name === 'x') this.onAction('camera');
          else if (name === 'y') this.onAction('map');
        }
        this.lastGamepad[name] = pressed;
      }
    } else {
      this.updateDeadman(deadman);
    }
    this.onLook(lookX, lookY);
    this.onDrive(throttle, turn, deadman);
  }
}
