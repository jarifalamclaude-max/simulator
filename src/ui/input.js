// Pilot input: gamepad (Gamepad API, standard mapping), keyboard and touch
// sticks, turned into MAVLink MANUAL_CONTROL fields the way QGroundControl
// sends them (x fwd, y lateral, z throttle 0..1000, r yaw, buttons in SDL order).

const FLIGHT_KEYS = {
  KeyW: ['x', 1], KeyS: ['x', -1], KeyA: ['r', -1], KeyD: ['r', 1], KeyQ: ['y', -1], KeyE: ['y', 1],
  KeyR: ['z', 1], KeyF: ['z', -1], Space: ['z', 1], KeyC: ['z', -1],
  ArrowUp: ['x', 1], ArrowDown: ['x', -1], ArrowLeft: ['y', -1], ArrowRight: ['y', 1],
  KeyI: ['s', 1], KeyK: ['s', -1], KeyJ: ['t', -1], KeyL: ['t', 1],
};

export class InputManager {
  constructor(app) {
    this.app = app;
    this.keys = new Set();
    this.axesKb = { x: 0, y: 0, z: 0, r: 0, s: 0, t: 0 };
    this.touch = { lx: 0, ly: 0, rx: 0, ry: 0, active: false };
    this.gamepadIndex = null;
    this.gamepadName = null;
    this.gamepadBlocked = false;
    this.lastPadButtons = [];
    this.source = 'keyboard';
    window.addEventListener('keydown', (e) => this.onKey(e, true));
    window.addEventListener('keyup', (e) => this.onKey(e, false));
    window.addEventListener('blur', () => this.keys.clear());
    // starting to type in a field drops held flight keys (their keyup may never come to us)
    window.addEventListener('focusin', (e) => { if (isTextEntry(e.target)) this.keys.clear(); });
    try {
      window.addEventListener('gamepadconnected', (e) => { this.gamepadIndex = e.gamepad.index; this.gamepadName = e.gamepad.id; this.app.toast?.(`Controller connected: ${shortName(e.gamepad.id)}`, 'good'); });
      window.addEventListener('gamepaddisconnected', (e) => { if (this.gamepadIndex === e.gamepad.index) { this.gamepadIndex = null; this.gamepadName = null; this.app.toast?.('Controller disconnected', 'bad'); } });
    } catch { /* ignore */ }
  }

  // a text-like field has the keyboard: typing goes there, not to the vehicle
  typing() {
    const a = document.activeElement;
    return isTextEntry(a);
  }

  onKey(e, down) {
    const code = e.code;
    // a release always counts, wherever the focus is (a key held while focus moves must not stick)
    if (!down) { this.keys.delete(code); return; }
    if (this.typing()) return;
    const a = document.activeElement;
    const tag = a?.tagName;
    const control = tag === 'SELECT' || tag === 'BUTTON' || tag === 'INPUT' || tag === 'A';
    // Enter / Backspace on a focused button or menu belong to it (dialogs, toggles)
    if (control && (code === 'Enter' || code === 'NumpadEnter' || code === 'Backspace')) return;
    // a focused select, button, slider or checkbox would turn W/A/S/D, arrows, Space and digits into
    // its own navigation (the mode select changes flight mode on letters): flight keys win
    const isFlight = code in FLIGHT_KEYS || code === 'ShiftLeft' || code === 'ShiftRight';
    if (control && (isFlight || /^Digit[0-9]$/.test(code) || code === 'Equal' || code === 'Minus')) { if (!code.startsWith('Shift')) e.preventDefault(); a.blur?.(); }
    if (!e.repeat) {
      const handled = this.app.hotkey?.(code, e);
      if (handled) { e.preventDefault(); return; }
    }
    if (isFlight) {
      this.keys.add(code);
      if (code.startsWith('Arrow') || code === 'Space') e.preventDefault();
    }
  }

  bindStick(el, side) {
    const knob = el.querySelector('i');
    let id = null;
    const set = (e) => {
      const r = el.getBoundingClientRect();
      let x = ((e.clientX - r.left) / r.width) * 2 - 1, y = ((e.clientY - r.top) / r.height) * 2 - 1;
      const l = Math.hypot(x, y);
      if (l > 1) { x /= l; y /= l; }
      knob.style.transform = `translate(${x * 40}px, ${y * 40}px)`;
      if (side === 'left') { this.touch.lx = x; this.touch.ly = y; } else { this.touch.rx = x; this.touch.ry = y; }
      this.touch.active = true;
    };
    el.addEventListener('pointerdown', (e) => { id = e.pointerId; el.setPointerCapture(id); set(e); });
    el.addEventListener('pointermove', (e) => { if (e.pointerId === id) set(e); });
    const end = (e) => {
      if (e.pointerId !== id) return;
      id = null; knob.style.transform = '';
      if (side === 'left') { this.touch.lx = 0; this.touch.ly = 0; } else { this.touch.rx = 0; this.touch.ry = 0; }
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
  }

  readGamepad() {
    let pads;
    try { pads = navigator.getGamepads ? navigator.getGamepads() : []; }
    catch { this.gamepadBlocked = true; return null; }
    let pad = null;
    if (this.gamepadIndex !== null && pads[this.gamepadIndex]) pad = pads[this.gamepadIndex];
    else for (const p of pads || []) if (p && p.connected) { pad = p; this.gamepadIndex = p.index; this.gamepadName = p.id; break; }
    return pad;
  }

  // returns MANUAL_CONTROL fields; dt for keyboard ramps
  sample(dt, cfg) {
    const jc = cfg.joystick;
    const dz = jc.deadzone ?? 0.06, expo = jc.expo ?? 0;
    const shape = (v) => {
      const a = Math.abs(v);
      if (a < dz) return 0;
      let s = (a - dz) / (1 - dz);
      s = (1 - expo) * s + expo * s * s * s;
      return Math.sign(v) * Math.min(1, s);
    };
    // keyboard: ramp towards key targets
    const target = { x: 0, y: 0, z: 0, r: 0, s: 0, t: 0 };
    for (const k of this.keys) { const m = FLIGHT_KEYS[k]; if (m) target[m[0]] += m[1]; }
    const ramp = 1 - Math.exp(-dt * 5); // first-order ramp, same response at any frame rate
    for (const k of Object.keys(target)) this.axesKb[k] += (Math.max(-1, Math.min(1, target[k])) - this.axesKb[k]) * ramp;
    const kbActive = Object.values(this.axesKb).some((v) => Math.abs(v) > 0.01);
    let x = this.axesKb.x, y = this.axesKb.y, z = this.axesKb.z, r = this.axesKb.r, s = this.axesKb.s, t = this.axesKb.t;
    let buttons = 0;
    const pad = this.readGamepad();
    this.padState = null;
    if (pad) {
      const ax = (m) => (m && pad.axes[m.axis] !== undefined ? (m.invert ? -1 : 1) * pad.axes[m.axis] : 0);
      const gx = shape(ax(jc.axes.x)), gy = shape(ax(jc.axes.y)), gz = shape(ax(jc.axes.z)), gr = shape(ax(jc.axes.r));
      const padActive = Math.abs(gx) + Math.abs(gy) + Math.abs(gz) + Math.abs(gr) > 0;
      if (padActive || !kbActive) { x = gx; y = gy; z = gz; r = gr; }
      const trig = (b) => (pad.buttons[b] ? pad.buttons[b].value : 0);
      const lt = trig(6), rt = trig(7);
      for (const [side, v] of [['left', lt], ['right', rt]]) {
        const d = triggerContribution(jc.triggers[side], v, side);
        s += d.s; t += d.t; z += d.z;
      }
      const map = jc.buttonMap || {};
      pad.buttons.forEach((b, i) => { if (b.pressed && map[i] !== undefined && map[i] >= 0 && map[i] < 32) buttons |= 1 << map[i]; });
      this.padState = { axes: Array.from(pad.axes), buttons: pad.buttons.map((b) => b.value), id: pad.id };
      this.source = padActive ? 'gamepad' : kbActive ? 'keyboard' : this.gamepadName ? 'gamepad' : 'keyboard';
    } else this.source = this.touch.active ? 'touch' : 'keyboard';
    if (this.touch.active && (this.touch.lx || this.touch.ly || this.touch.rx || this.touch.ry)) {
      r = shape(this.touch.lx); z = shape(-this.touch.ly); y = shape(this.touch.rx); x = shape(-this.touch.ry);
    }
    // keyboard Shift presses the joystick button whose BTNn_FUNCTION is shift (k_shift), if any
    if (this.keys.has('ShiftLeft') || this.keys.has('ShiftRight')) { const b = shiftButtonIndex(cfg.params); if (b >= 0) buttons |= 1 << b; }
    const cl = (v) => Math.max(-1, Math.min(1, v));
    return {
      x: Math.round(cl(x) * 1000), y: Math.round(cl(y) * 1000), z: Math.round(500 + cl(z) * 500), r: Math.round(cl(r) * 1000),
      s: Math.round(cl(s) * 400), t: Math.round(cl(t) * 400), buttons: buttons >>> 0,
    };
  }
}

// A focused element that takes typed text (not selects, buttons, checkboxes or sliders)
export function isTextEntry(a) {
  if (!a) return false;
  if (a.isContentEditable || a.tagName === 'TEXTAREA') return true;
  if (a.tagName !== 'INPUT') return false;
  return !/^(button|checkbox|radio|range|color|file|submit|reset|image)$/i.test(a.type || 'text');
}
// Gamepad trigger (0..1) assigned to a function: left trigger is the negative side of a pitch/roll
// pair, while Ascend/Descend act the same on either trigger.
export function triggerContribution(fn, v, side) {
  const d = { s: 0, t: 0, z: 0 };
  const sv = side === 'left' ? -v : v;
  if (fn === 'pitch') d.s = sv; else if (fn === 'roll') d.t = sv; else if (fn === 'up') d.z = v; else if (fn === 'down') d.z = -v;
  return d;
}
// first joystick button with BTNn_FUNCTION = 1 (shift), as joystick.cpp:71-75 detects it; -1 if none
export function shiftButtonIndex(P = {}) {
  for (let i = 0; i < 32; i++) if ((P[`BTN${i}_FUNCTION`] ?? (i === 5 ? 1 : 0)) === 1) return i;
  return -1;
}
export function shortName(id = '') {
  const m = /^(.*?)\s*\(/.exec(id);
  return (m ? m[1] : id).slice(0, 40) || 'Gamepad';
}
export const KEY_HELP = [
  ['W / S', 'Forward / back'], ['A / D', 'Yaw left / right'], ['Q / E', 'Strafe left / right'], ['R / F (Space / C)', 'Ascend / descend'],
  ['I / K', 'Pitch input (6-DOF)'], ['J / L', 'Roll input (6-DOF)'], ['Enter', 'Arm / disarm'], ['1-6', 'Manual, Stabilize, Depth hold, Acro, Position hold, Surface'],
  ['+ / −', 'Pilot gain'], ['V', 'Next camera'], ['G', 'Graphs'], ['P', 'Pause'], ['Backspace', 'Reset vehicle'], ['Shift', 'Presses the joystick button set to shift (BTNn_FUNCTION = 1)'],
  ['Flight keys', 'Work whenever no text field has the focus (a focused menu or button lets go of them)'],
];
