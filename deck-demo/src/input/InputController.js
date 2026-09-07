const DEADMAN_KEYS = new Set(['Space', 'ShiftLeft', 'ShiftRight']);
const KEY_TO_AXIS = {
  KeyW: ['throttle', 1], ArrowUp: ['throttle', 1], KeyS: ['throttle', -1], ArrowDown: ['throttle', -1],
  KeyA: ['turn', -1], ArrowLeft: ['turn', -1], KeyD: ['turn', 1], ArrowRight: ['turn', 1]
};

export class InputController {
  constructor({ onDrive, onAction, onDeadman }) {
    this.onDrive = onDrive;
    this.onAction = onAction;
    this.onDeadman = onDeadman;
    this.keys = new Set();
    this.touch = { throttle: 0, turn: 0 };
    this.touchDeadman = false;
    this.deadman = false;
    this.gamepad = null;
    this.lastGamepad = { a: false, b: false, x: false, y: false, l1: false, r1: false };
    this.bindKeyboard();
    this.bindGamepadEvents();
  }
  bindKeyboard() {
    window.addEventListener('keydown', (event) => {
      if (['Space', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight'].includes(event.code)) event.preventDefault();
      if (event.repeat && ['KeyB', 'KeyX', 'KeyY', 'KeyC', 'KeyL'].includes(event.code)) return;
      this.keys.add(event.code);
      if (event.code === 'Escape' || event.code === 'KeyB') this.onAction('stop');
      if (event.code === 'KeyC' || event.code === 'KeyX') this.onAction('camera');
      if (event.code === 'KeyL') this.onAction('link');
      if (event.code === 'KeyH') this.onAction('demo');
      this.updateDeadman();
    });
    window.addEventListener('keyup', (event) => {
      this.keys.delete(event.code);
      this.updateDeadman();
    });
  }
  bindGamepadEvents() {
    window.addEventListener('gamepadconnected', (event) => { this.gamepad = event.gamepad; this.onAction('gamepad', true); });
    window.addEventListener('gamepaddisconnected', () => { this.gamepad = null; this.onAction('gamepad', false); this.stop(); });
  }
  updateDeadman() {
    const next = [...DEADMAN_KEYS].some((key) => this.keys.has(key));
    if (next !== this.deadman) { this.deadman = next; this.onDeadman(next); }
  }
  setTouchAxis(axis, value) { this.touch[axis] = value; this.touchDeadman = Object.values(this.touch).some((entry) => Math.abs(entry) > 0.01); }
  stop() { this.keys.clear(); this.touch = { throttle: 0, turn: 0 }; this.touchDeadman = false; this.deadman = false; this.onDeadman(false); this.onAction('stop'); }
  poll() {
    const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
    const pad = this.gamepad || [...gamepads].find(Boolean);
    let throttle = 0; let turn = 0; let deadman = this.deadman || this.touchDeadman;
    for (const [code, [axis, sign]] of Object.entries(KEY_TO_AXIS)) {
      if (this.keys.has(code)) { if (axis === 'throttle') throttle += sign; else turn += sign; }
    }
    throttle = Math.max(-1, Math.min(1, throttle + this.touch.throttle));
    turn = Math.max(-1, Math.min(1, turn + this.touch.turn));
    if (pad) {
      const leftX = pad.axes?.[0] || 0; const leftY = pad.axes?.[1] || 0;
      if (Math.abs(leftX) > 0.12 || Math.abs(leftY) > 0.12) { turn = leftX; throttle = -leftY; }
      deadman = Boolean(pad.buttons?.[7]?.value > 0.15 || pad.buttons?.[5]?.pressed || deadman);
      if (deadman !== this.deadman) { this.deadman = deadman; this.onDeadman(deadman); }
      const buttons = { a: 0, b: 1, x: 2, y: 3, l1: 4, r1: 5 };
      for (const [name, index] of Object.entries(buttons)) {
        const pressed = Boolean(pad.buttons?.[index]?.pressed);
        if (pressed && !this.lastGamepad[name]) this.onAction(name);
        this.lastGamepad[name] = pressed;
      }
    }
    this.onDrive(throttle, turn, deadman);
  }
}
