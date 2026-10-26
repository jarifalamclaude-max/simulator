// Port of ArduSub joystick.cpp: MANUAL_CONTROL -> RC overrides, pilot gain,
// trims, input hold, roll/pitch toggle and button functions (AP_JSButton),
// plus the outputs the buttons drive: lights1/2 and video switch
// (SRV_Channel range outputs), the servo camera mount (AP_Mount RC rate
// targeting, MNT1_*), relays (AP_Relay, RELAYn_*), actuators 1-6
// (ArduSub/actuators.cpp, ACTUATORn_INC) and script buttons, mapped to
// SERVOn_FUNCTION channels so every output has the PWM the Pixhawk would send.

const f32 = Math.fround, trunc = Math.trunc;
const clamp = (v, lo, hi) => (v < lo ? lo : v > hi ? hi : v);

// AP_JSButton.h:10-149, labels as in the @Values list (AP_JSButton.cpp:8)
export const BUTTON_FUNCTIONS = {
  0: 'disabled', 1: 'shift', 2: 'arm_toggle', 3: 'arm', 4: 'disarm',
  5: 'mode_manual', 6: 'mode_stabilize', 7: 'mode_depth_hold', 8: 'mode_poshold', 9: 'mode_auto', 10: 'mode_circle', 11: 'mode_guided', 12: 'mode_acro', 13: 'mode_surftrak',
  21: 'mount_center', 22: 'mount_tilt_up', 23: 'mount_tilt_down', 24: 'camera_trigger', 25: 'camera_source_toggle', 26: 'mount_pan_right', 27: 'mount_pan_left',
  31: 'lights1_cycle', 32: 'lights1_brighter', 33: 'lights1_dimmer', 34: 'lights2_cycle', 35: 'lights2_brighter', 36: 'lights2_dimmer',
  41: 'gain_toggle', 42: 'gain_inc', 43: 'gain_dec', 44: 'trim_roll_inc', 45: 'trim_roll_dec', 46: 'trim_pitch_inc', 47: 'trim_pitch_dec', 48: 'input_hold_set', 49: 'roll_pitch_toggle',
  51: 'relay_1_on', 52: 'relay_1_off', 53: 'relay_1_toggle', 54: 'relay_2_on', 55: 'relay_2_off', 56: 'relay_2_toggle', 57: 'relay_3_on', 58: 'relay_3_off', 59: 'relay_3_toggle',
  61: 'actuator_1_inc', 62: 'actuator_1_dec', 63: 'actuator_1_min', 64: 'actuator_1_max', 65: 'actuator_1_center',
  66: 'actuator_2_inc', 67: 'actuator_2_dec', 68: 'actuator_2_min', 69: 'actuator_2_max', 70: 'actuator_2_center',
  71: 'actuator_3_inc', 72: 'actuator_3_dec', 73: 'actuator_3_min', 74: 'actuator_3_max', 75: 'actuator_3_center',
  76: 'actuator_1_min_momentary', 77: 'actuator_1_max_momentary', 78: 'actuator_1_min_toggle', 79: 'actuator_1_max_toggle',
  80: 'actuator_2_min_momentary', 81: 'actuator_2_max_momentary', 82: 'actuator_2_min_toggle', 83: 'actuator_2_max_toggle',
  84: 'actuator_3_min_momentary', 85: 'actuator_3_max_momentary', 86: 'actuator_3_min_toggle', 87: 'actuator_3_max_toggle',
  91: 'custom_1', 92: 'custom_2', 93: 'custom_3', 94: 'custom_4', 95: 'custom_5', 96: 'custom_6',
  101: 'relay_4_on', 102: 'relay_4_off', 103: 'relay_4_toggle', 104: 'relay_1_momentary', 105: 'relay_2_momentary', 106: 'relay_3_momentary', 107: 'relay_4_momentary',
  108: 'script_1', 109: 'script_2', 110: 'script_3', 111: 'script_4',
  112: 'actuator_4_min', 113: 'actuator_4_max', 114: 'actuator_4_center', 115: 'actuator_4_inc', 116: 'actuator_4_dec', 117: 'actuator_4_min_momentary', 118: 'actuator_4_max_momentary', 119: 'actuator_4_min_toggle', 120: 'actuator_4_max_toggle',
  121: 'actuator_5_min', 122: 'actuator_5_max', 123: 'actuator_5_center', 124: 'actuator_5_inc', 125: 'actuator_5_dec', 126: 'actuator_5_min_momentary', 127: 'actuator_5_max_momentary', 128: 'actuator_5_min_toggle', 129: 'actuator_5_max_toggle',
  130: 'actuator_6_min', 131: 'actuator_6_max', 132: 'actuator_6_center', 133: 'actuator_6_inc', 134: 'actuator_6_dec', 135: 'actuator_6_min_momentary', 136: 'actuator_6_max_momentary', 137: 'actuator_6_min_toggle', 138: 'actuator_6_max_toggle',
};
export const buttonFunctionLabel = (n) => (BUTTON_FUNCTIONS[n] ?? `function ${n}`).replace(/_/g, ' ');
// grouping for the joystick panel
export const BUTTON_FUNCTION_GROUPS = [
  ['General', [0, 1, 2, 3, 4]], ['Flight modes', [5, 6, 7, 8, 9, 10, 11, 12, 13]], ['Camera & mount', [21, 22, 23, 24, 25, 26, 27]],
  ['Lights', [31, 32, 33, 34, 35, 36]], ['Gain, trim & input', [41, 42, 43, 44, 45, 46, 47, 48, 49]],
  ['Relays', [51, 52, 53, 54, 55, 56, 57, 58, 59, 101, 102, 103, 104, 105, 106, 107]],
  ['Actuators', [61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, ...Array.from({ length: 27 }, (_, i) => 112 + i)]],
  ['Custom & scripting', [91, 92, 93, 94, 95, 96, 108, 109, 110, 111]],
];

// actuator button decoding: [actuator index, action]
const ACT_ACTIONS_1_3 = ['inc', 'dec', 'min', 'max', 'center'];
const ACT_ACTIONS_EXT = ['min_momentary', 'max_momentary', 'min_toggle', 'max_toggle'];
const ACT_ACTIONS_4_6 = ['min', 'max', 'center', 'inc', 'dec', 'min_momentary', 'max_momentary', 'min_toggle', 'max_toggle'];
export function actuatorAction(fn) {
  if (fn >= 61 && fn <= 75) return [trunc((fn - 61) / 5), ACT_ACTIONS_1_3[(fn - 61) % 5]];
  if (fn >= 76 && fn <= 87) return [trunc((fn - 76) / 4), ACT_ACTIONS_EXT[(fn - 76) % 4]];
  if (fn >= 112 && fn <= 138) return [3 + trunc((fn - 112) / 9), ACT_ACTIONS_4_6[(fn - 112) % 9]];
  return null;
}
// relay button decoding: [relay index, action]
export function relayAction(fn) {
  if (fn >= 51 && fn <= 59) return [trunc((fn - 51) / 3), ['on', 'off', 'toggle'][(fn - 51) % 3]];
  if (fn >= 101 && fn <= 103) return [3, ['on', 'off', 'toggle'][fn - 101]];
  if (fn >= 104 && fn <= 107) return [fn - 104, 'momentary'];
  return null;
}

// QGC / SDL GameController button numbering used by ArduSub's BTNn params (0-15 in
// MANUAL_CONTROL.buttons, 16-31 in the buttons2 extension)
export const SDL_BUTTON_NAMES = ['A', 'B', 'X', 'Y', 'Back', 'Guide', 'Start', 'L-stick', 'R-stick', 'LB', 'RB', 'D-pad up', 'D-pad down', 'D-pad left', 'D-pad right', 'Misc 1', 'Paddle 1', 'Paddle 2', 'Paddle 3', 'Paddle 4', 'Touchpad'];

// SRV_Channel::Aux_servo_function_t values the simulator drives or shows
export const SERVO_FUNCTIONS = { '-1': 'GPIO', 0: 'Disabled', 1: 'RCPassThru', 6: 'Mount1Yaw (pan)', 7: 'Mount1Pitch (tilt)', 8: 'Mount1Roll', 9: 'Mount1Retract', 10: 'CameraTrigger', 33: 'Motor1', 34: 'Motor2', 35: 'Motor3', 36: 'Motor4', 37: 'Motor5', 38: 'Motor6', 39: 'Motor7', 40: 'Motor8', 82: 'Motor9', 83: 'Motor10', 84: 'Motor11', 85: 'Motor12', 120: 'NeoPixel1', 181: 'Lights1', 182: 'Lights2', 183: 'VideoSwitch', 184: 'Actuator1', 185: 'Actuator2', 186: 'Actuator3', 187: 'Actuator4', 188: 'Actuator5', 189: 'Actuator6' };
for (let i = 1; i <= 16; i++) SERVO_FUNCTIONS[50 + i] = `RCIN${i}`;
for (let i = 1; i <= 16; i++) SERVO_FUNCTIONS[93 + i] = `Script${i}`;
export const SRV = { MOUNT_PAN: 6, MOUNT_TILT: 7, MOUNT_ROLL: 8, LIGHTS1: 181, LIGHTS2: 182, VIDEO_SWITCH: 183, ACTUATOR1: 184, RCIN1: 51 };
const motorIndexOfFunction = (f) => (f >= 33 && f <= 40 ? f - 33 : f >= 82 && f <= 85 ? f - 74 : -1);
// Sub defaults: motors on SERVO1-8 (frame setup), SERVO9 rcin9 (lights 1), SERVO10 mount tilt (Parameters.h:427-434)
export const servoFunctionDefault = (n) => (n <= 8 ? 32 + n : n === 9 ? 59 : n === 10 ? 7 : 0);
const srvP = (P, n, k) => P?.[`SERVO${n}_${k}`] ?? ({ FUNCTION: servoFunctionDefault(n), MIN: 1100, MAX: 1900, TRIM: 1500, REVERSED: 0 })[k];
// AP_Mount_Params defaults (4.5) with ArduSub defaults_table MNT1_TYPE 1, MNT1_DEFLT_MODE 3 (RC targeting), MNT1_RC_RATE 30
export const MOUNT_DEFAULTS = { MNT1_TYPE: 1, MNT1_DEFLT_MODE: 3, MNT1_RC_RATE: 30, MNT1_ROLL_MIN: -30, MNT1_ROLL_MAX: 30, MNT1_PITCH_MIN: -90, MNT1_PITCH_MAX: 20, MNT1_YAW_MIN: -180, MNT1_YAW_MAX: 180 };
const mnt = (P, k) => P?.[k] ?? MOUNT_DEFAULTS[k];

export class JoystickHandler {
  constructor(sub) {
    this.sub = sub;
    this.camTilt = 1500; this.camPan = 1500;
    this.lights1 = 0; this.lights2 = 0;
    this.rollTrim = 0; this.pitchTrim = 0; this.zTrim = 0; this.xTrim = 0; this.yTrim = 0;
    this.xLast = 0; this.yLast = 0; this.zLast = 0;
    this.buttonsPrev = 0;
    this.buttons = 0;
    this.controlsResetSinceInputHold = true;
    this.inputHoldEngaged = false;
    this.rollPitchFlag = 0;
    // function-local statics in joystick.cpp
    this.lightsIncreasing = [true, true];
    this.gainToggleLow = false;
    this.videoToggle = false;
    this.relays = [false, false, false, false, false, false];
    this.actuators = [0, 0, 0, 0, 0, 0];
    this.scriptButtons = [0, 1, 2, 3].map(() => ({ pressed: false, count: 0 }));
    this.mount = { mode: 3, roll: 0, pitch: 0, yaw: 0, rateMode: true }; // degrees
    this.videoSwitch = 0;
    this.cameraSource = 1;
    this.nextOutT = 0;
    this.rejected = 0;
    this.pendingSaves = []; // [name, value] written with set_and_save; the UI copies them into the vehicle config
    this.servoFunctionOverrides = {};
  }

  get params() { return this.sub.params; }
  nowMs() { return (this.sub.time ?? 0) * 1000; }
  setAndSave(name, value) {
    const P = this.sub.params;
    if (P[name] === value) return;
    P[name] = value;
    this.pendingSaves.push([name, value]);
  }

  // Sub::init_joystick (joystick.cpp:24) + the boot-time output setup in system.cpp:178-182
  init(P) {
    const s = this.sub;
    if (!((P.JS_GAIN_STEPS ?? 4) >= 1)) this.setAndSave('JS_GAIN_STEPS', 1);
    const steps = trunc(s.params.JS_GAIN_STEPS ?? P.JS_GAIN_STEPS ?? 4);
    const gMin = P.JS_GAIN_MIN ?? 0.25, gMax = P.JS_GAIN_MAX ?? 1, gDef = P.JS_GAIN_DEFAULT ?? 0.5;
    if (steps === 1 || (gDef < gMax + 0.01 && gDef > gMin - 0.01)) s.gain = clamp(gDef, gMin, gMax);
    else s.gain = gMin + (trunc(steps / 2) - 1) * ((gMax - gMin) / (steps - 1));
    s.gain = f32(clamp(s.gain, 0.1, 1));
    this.lights1 = 0; this.lights2 = 0; this.videoSwitch = 0;
    this.updateActuatorsFromJsbuttons();
    this.initializeActuators();
    this.updateLightsFromRcin();
    for (let i = 0; i < 6; i++) { const d = P[`RELAY${i + 1}_DEFAULT`] ?? 0; this.relays[i] = d === 1; }
    this.mount = { mode: mnt(P, 'MNT1_DEFLT_MODE'), roll: 0, pitch: 0, yaw: 0, rateMode: mnt(P, 'MNT1_RC_RATE') > 0 };
    this.nextOutT = s.time ?? 0;
  }

  buttonFunction(i, shift) {
    const P = this.sub.params;
    return (shift ? P[`BTN${i}_SFUNCTION`] : P[`BTN${i}_FUNCTION`]) ?? 0;
  }
  // Sub::jsbutton_function_is_assigned (joystick.cpp:838)
  functionIsAssigned(fn) {
    let shiftAssigned = false, onShift = false;
    for (let i = 0; i < 32; i++) {
      if (this.buttonFunction(i, false) === fn) return true;
      if (this.buttonFunction(i, false) === 1) shiftAssigned = true;
      if (this.buttonFunction(i, true) === fn) onShift = true;
    }
    return shiftAssigned && onShift;
  }

  // ---------------- SRV outputs ----------------
  servoFunction(n) { return this.servoFunctionOverrides[n] ?? srvP(this.sub.params, n, 'FUNCTION'); }
  findServo(fn) { for (let n = 1; n <= 16; n++) if (this.servoFunction(n) === fn) return n; return 0; }
  // Parameters.cpp:846 update_actuators_from_jsbuttons(): legacy servo_n buttons on SERVO9-11
  updateActuatorsFromJsbuttons() {
    for (let a = 0; a < 3; a++) {
      const ch = 9 + a;
      if (this.findServo(SRV.ACTUATOR1 + a)) continue;
      if (this.servoFunction(ch) !== 0) continue;
      const fns = [61 + 5 * a, 62 + 5 * a, 63 + 5 * a, 64 + 5 * a, 65 + 5 * a, 76 + 4 * a, 77 + 4 * a, 78 + 4 * a, 79 + 4 * a];
      if (fns.some((f) => this.functionIsAssigned(f))) this.setAndSave(`SERVO${ch}_FUNCTION`, SRV.ACTUATOR1 + a);
    }
  }
  // Parameters.cpp:936 update_lights_from_rcin(): RCIN9/10 channels become lights1/2 when light buttons exist
  updateLightsFromRcin() {
    const btns = [[32, 33, 31], [35, 36, 34]];
    for (let l = 0; l < 2; l++) {
      if (this.findServo(SRV.LIGHTS1 + l)) continue;
      const ch = this.findServo(SRV.RCIN1 + 8 + l);
      if (!ch) continue;
      if (btns[l].some((f) => this.functionIsAssigned(f))) this.setAndSave(`SERVO${ch}_FUNCTION`, SRV.LIGHTS1 + l);
    }
  }
  // actuators.cpp initialize_actuators(): start at SERVOn_TRIM
  initializeActuators() {
    const P = this.sub.params;
    for (let i = 0; i < 6; i++) {
      const n = this.findServo(SRV.ACTUATOR1 + i);
      if (!n) continue;
      const mn = srvP(P, n, 'MIN'), mx = srvP(P, n, 'MAX');
      this.actuators[i] = f32((srvP(P, n, 'TRIM') - mn) / (mx - mn));
    }
  }
  actuatorOp(i, op) {
    if (i < 0 || i >= 6) return;
    const v = this.actuators[i], inc = this.sub.params[`ACTUATOR${i + 1}_INC`] ?? 0.01;
    switch (op) {
      case 'inc': this.actuators[i] = f32(clamp(v + inc, 0, 1)); break;
      case 'dec': this.actuators[i] = f32(clamp(v - inc, 0, 1)); break;
      case 'min': this.actuators[i] = 0; break;
      case 'max': this.actuators[i] = 1; break;
      case 'center': this.actuators[i] = 0.5; break;
      case 'min_toggle': this.actuators[i] = v < 0.4 ? 0.5 : 0; break;
      case 'max_toggle': this.actuators[i] = v >= 0.6 ? 0.5 : 1; break;
    }
  }
  // AP_Relay: by-index on/off/toggle only act on instances with RELAYn_FUNCTION = 1 (Relay)
  relayOp(i, op) {
    if ((this.sub.params[`RELAY${i + 1}_FUNCTION`] ?? 0) !== 1) return;
    if (op === 'on') this.relays[i] = true; else if (op === 'off') this.relays[i] = false; else if (op === 'toggle') this.relays[i] = !this.relays[i];
  }
  relayPinState(i) { const P = this.sub.params; return (P[`RELAY${i + 1}_INVERTED`] ?? 0) ? !this.relays[i] : this.relays[i]; }

  // AP_Mount servo backend, RC targeting at 50 Hz (MNT1_RC_RATE deg/s from the MOUNT1_* RC
  // channels' norm_input_dz, limited to MNT1_*_MIN/MAX). Fixed 20 ms steps in firmware time so
  // it gives the same answer whoever calls it (the fast loop, a MANUAL_CONTROL or the UI).
  update(now = this.sub.time ?? 0) {
    if (now < this.nextOutT - 1) this.nextOutT = now; // time went backwards (reset)
    let n = 0;
    while (this.nextOutT <= now && n++ < 500) { this.mountStep(0.02); this.nextOutT += 0.02; }
    if (n >= 500) this.nextOutT = now + 0.02;
  }
  mountStep(dt) {
    const P = this.sub.params, rc = this.sub.rc;
    if (mnt(P, 'MNT1_TYPE') === 0 || !rc?.findChannelForOption) return;
    const m = this.mount;
    if (m.mode !== 3) return; // only RC targeting is driven from the pilot
    const inp = (opt) => rc.findChannelForOption(opt)?.normInputDz() ?? 0;
    const r = inp(212), p = inp(213), y = inp(214);
    const rate = mnt(P, 'MNT1_RC_RATE');
    const lim = (v, k) => clamp(v, mnt(P, `MNT1_${k}_MIN`), mnt(P, `MNT1_${k}_MAX`));
    if (rate <= 0) {
      const ang = (v, k) => ((v + 1) * 0.5) * (mnt(P, `MNT1_${k}_MAX`) - mnt(P, `MNT1_${k}_MIN`)) + mnt(P, `MNT1_${k}_MIN`);
      m.roll = ang(r, 'ROLL'); m.pitch = ang(p, 'PITCH'); m.yaw = ang(y, 'YAW'); m.rateMode = false;
      return;
    }
    m.rateMode = true;
    m.roll = lim(m.roll + r * rate * dt, 'ROLL');
    m.pitch = lim(m.pitch + p * rate * dt, 'PITCH');
    const ymin = mnt(P, 'MNT1_YAW_MIN'), ymax = mnt(P, 'MNT1_YAW_MAX');
    m.yaw = ymax - ymin >= 360 ? ((((m.yaw + y * rate * dt) + 180) % 360) + 360) % 360 - 180 : lim(m.yaw + y * rate * dt, 'YAW');
  }
  // SRV_Channels::move_servo for an angle between angle limits
  static movePwm(P, n, value, amin, amax) {
    if (amax <= amin) return null;
    let v = clamp((value - amin) / (amax - amin), 0, 1);
    if (srvP(P, n, 'REVERSED')) v = 1 - v;
    const mn = srvP(P, n, 'MIN'), mx = srvP(P, n, 'MAX');
    return trunc(mn + v * (mx - mn));
  }
  // SRV_Channel::pwm_from_range with high_out 1000
  static rangePwm(P, n, scaled) {
    const mn = srvP(P, n, 'MIN'), mx = srvP(P, n, 'MAX');
    if (mx <= mn) return mn;
    let s = clamp(scaled, 0, 1000);
    if (srvP(P, n, 'REVERSED')) s = 1000 - s;
    return mn + trunc((s * (mx - mn)) / 1000);
  }
  // PWM on SERVO1..16 as the Pixhawk would output it (null = not driven by the simulator)
  servoOutputs() {
    const P = this.sub.params, out = [];
    const motorPwm = this.sub.motors?.pwm ?? [];
    const mntOn = mnt(P, 'MNT1_TYPE') === 1;
    for (let n = 1; n <= 16; n++) {
      const fn = this.servoFunction(n);
      let pwm = null;
      const mi = motorIndexOfFunction(fn);
      if (mi >= 0) pwm = motorPwm[mi] ?? null;
      else if (fn === 1) pwm = this.sub.rc?.all[n - 1]?.radioIn ?? null;
      else if (fn >= 51 && fn <= 66) pwm = this.sub.rc?.all[fn - 51]?.radioIn ?? null;
      else if (fn === SRV.MOUNT_TILT && mntOn) pwm = JoystickHandler.movePwm(P, n, this.mount.pitch * 10, mnt(P, 'MNT1_PITCH_MIN') * 10, mnt(P, 'MNT1_PITCH_MAX') * 10);
      else if (fn === SRV.MOUNT_PAN && mntOn) pwm = JoystickHandler.movePwm(P, n, this.mount.yaw * 10, mnt(P, 'MNT1_YAW_MIN') * 10, mnt(P, 'MNT1_YAW_MAX') * 10);
      else if (fn === SRV.MOUNT_ROLL && mntOn) pwm = JoystickHandler.movePwm(P, n, this.mount.roll * 10, mnt(P, 'MNT1_ROLL_MIN') * 10, mnt(P, 'MNT1_ROLL_MAX') * 10);
      else if (fn === SRV.LIGHTS1) pwm = JoystickHandler.rangePwm(P, n, this.lights1);
      else if (fn === SRV.LIGHTS2) pwm = JoystickHandler.rangePwm(P, n, this.lights2);
      else if (fn === SRV.VIDEO_SWITCH) pwm = JoystickHandler.rangePwm(P, n, this.videoSwitch);
      else if (fn >= SRV.ACTUATOR1 && fn < SRV.ACTUATOR1 + 6) { const mn = srvP(P, n, 'MIN'), mx = srvP(P, n, 'MAX'); pwm = trunc(mn + (mx - mn) * this.actuators[fn - SRV.ACTUATOR1]); }
      out.push({ n, fn, label: SERVO_FUNCTIONS[fn] ?? `function ${fn}`, pwm, min: srvP(P, n, 'MIN'), max: srvP(P, n, 'MAX'), trim: srvP(P, n, 'TRIM'), reversed: !!srvP(P, n, 'REVERSED') });
    }
    return out;
  }
  // brightness of the lights that hang on a lights1/lights2 servo channel (Lumen: 1100 off .. 1900 full)
  lightLevel(k = 0) {
    const n = this.findServo(SRV.LIGHTS1 + k);
    if (!n) return 0;
    const pwm = JoystickHandler.rangePwm(this.sub.params, n, k ? this.lights2 : this.lights1);
    return clamp((pwm - 1100) / 800, 0, 1);
  }
  // physical camera tilt (deg, up positive) from the tilt servo PWM, or null without a tilt servo
  cameraTiltDeg() {
    const P = this.sub.params, n = this.findServo(SRV.MOUNT_TILT);
    if (!n || mnt(P, 'MNT1_TYPE') !== 1) return null;
    const pwm = JoystickHandler.movePwm(P, n, this.mount.pitch * 10, mnt(P, 'MNT1_PITCH_MIN') * 10, mnt(P, 'MNT1_PITCH_MAX') * 10);
    if (pwm === null) return null;
    let v = (pwm - srvP(P, n, 'MIN')) / Math.max(1, srvP(P, n, 'MAX') - srvP(P, n, 'MIN'));
    if (srvP(P, n, 'REVERSED')) v = 1 - v;
    return mnt(P, 'MNT1_PITCH_MIN') + v * (mnt(P, 'MNT1_PITCH_MAX') - mnt(P, 'MNT1_PITCH_MIN'));
  }
  cameraPanDeg() {
    const P = this.sub.params;
    return this.findServo(SRV.MOUNT_PAN) && mnt(P, 'MNT1_TYPE') === 1 ? this.mount.yaw : null;
  }

  // GCS_Mavlink.cpp:583-613: MANUAL_CONTROL is only taken from SYSID_MYGCS and aimed at SYSID_THISMAV
  handleMessage(msg) {
    const P = this.sub.params;
    if ((msg.sysid ?? 255) !== (P.SYSID_MYGCS ?? 255)) { this.rejected++; return false; }
    if ((msg.target ?? P.SYSID_THISMAV ?? 1) !== (P.SYSID_THISMAV ?? 1)) { this.rejected++; return false; }
    // buttons above bit 15 given in .buttons (simulator shortcut) are taken as buttons2
    const b = msg.buttons >>> 0;
    this.transform(msg.x | 0, msg.y | 0, msg.z | 0, msg.r | 0, b & 0xffff, msg.s | 0, msg.t | 0, ((msg.buttons2 ?? 0) | (b >>> 16)) & 0xffff, msg.enabled_extensions ?? 0, msg.aux);
    return true;
  }

  // Sub::transform_manual_control_to_rc_override (joystick.cpp:47). MANUAL_CONTROL: x,y,r in
  // [-1000,1000], z in [0,1000], buttons (0-15) + buttons2 (16-31). Legacy callers pass all 32
  // buttons in `buttons`. s/t (pitch/roll) are used as received; aux1-6 are ignored by ArduSub 4.5.7.
  transform(x, y, z, r, buttons, s = 0, t = 0, buttons2 = 0, enabledExtensions = 0, aux = null) {
    const sub = this.sub, P = sub.params;
    const rpyScale = f32(0.4 * sub.gain);
    const throttleScale = f32(0.8 * sub.gain * f32(P.JS_THR_GAIN ?? 1));
    const rpyCenter = 1500;
    const throttleBase = trunc(f32(1500 - f32(500 * throttleScale))); // int16_t (joystick.cpp:61)
    const now = this.nowMs();
    if (sub.rc) sub.rc.nowMs = now;
    let shift = false;
    this.camTilt = 1500; this.camPan = 1500;
    const all = ((buttons >>> 0) | ((buttons2 & 0xffff) << 16)) >>> 0;
    this.buttons = all;
    for (let i = 0; i < 32; i++) if (all & (1 << i) && this.buttonFunction(i, false) === 1) shift = true;
    this.shift = shift;
    for (let i = 0; i < 32; i++) {
      const bit = (1 << i) >>> 0;
      if (all & bit) this.handlePress(i, shift, !!(this.buttonsPrev & bit));
      else if (this.buttonsPrev & bit) this.handleRelease(i, shift);
    }
    this.buttonsPrev = all;
    if (this.rollPitchFlag === 1) {
      this.pitchTrim = trunc(f32(-x * rpyScale));
      this.rollTrim = trunc(f32(y * rpyScale));
    }
    let zTot, yTot, xTot;
    if (!this.controlsResetSinceInputHold) {
      zTot = this.zTrim + 500; yTot = this.yTrim; xTot = this.xTrim;
      this.controlsResetSinceInputHold = Math.abs(z - 500) < 50 && Math.abs(y) < 50 && Math.abs(x) < 50;
    } else {
      zTot = z + this.zTrim; yTot = y + this.yTrim; xTot = x + this.xTrim;
    }
    // constrain_int16(float) truncates the float toward zero before limiting
    const c = (v) => clamp(trunc(v), 1100, 1900);
    const sc = (v, k) => f32(f32(v * k) + rpyCenter);
    const rc = sub.rc;
    rc.pitch.setOverride(c(s + this.pitchTrim + rpyCenter), now);
    rc.roll.setOverride(c(t + this.rollTrim + rpyCenter), now);
    rc.throttle.setOverride(c(f32(f32(zTot * throttleScale) + throttleBase)), now);
    rc.yaw.setOverride(c(sc(r, rpyScale)), now);
    if (this.rollPitchFlag === 0) {
      rc.forward.setOverride(c(sc(xTot, rpyScale)), now);
      rc.lateral.setOverride(c(sc(yTot, rpyScale)), now);
    } else {
      rc.forward.setOverride(c(sc(this.xTrim, rpyScale)), now);
      rc.lateral.setOverride(c(sc(this.yTrim, rpyScale)), now);
    }
    rc.all[6].setOverride(this.camPan, now);
    rc.all[7].setOverride(this.camTilt, now);
    this.xLast = x; this.yLast = y; this.zLast = z;
    this.update(sub.time ?? 0);
  }

  // Sub::handle_jsbutton_press (joystick.cpp:141)
  handlePress(i, shift, held) {
    const sub = this.sub, P = sub.params;
    const fn = this.buttonFunction(i, shift);
    const lightStep = trunc(1000 / (P.JS_LIGHTS_STEPS ?? 8)); // uint16_t step = 1000.0 / g.lights_steps
    const rel = relayAction(fn);
    if (rel) { const [k, op] = rel; if (op === 'momentary') { if (!held) this.relayOp(k, 'on'); } else if (op !== 'toggle' || !held) this.relayOp(k, op); return; }
    const act = actuatorAction(fn);
    if (act) {
      const [k, op] = act;
      if (op === 'min_toggle' || op === 'max_toggle') { if (!held) this.actuatorOp(k, op); return; }
      if (k === 5 && (op === 'max' || op === 'max_momentary')) return; // joystick.cpp:519-538 has no case for servo_6_max(_momentary)
      this.actuatorOp(k, op.replace('_momentary', ''));
      return;
    }
    switch (fn) {
      case 2: sub.armed ? sub.disarm('joystick') : sub.arm('joystick'); break;
      case 3: sub.arm('joystick'); break;
      case 4: sub.disarm('joystick'); break;
      case 5: sub.setMode('MANUAL', 'button'); break;
      case 6: sub.setMode('STABILIZE', 'button'); break;
      case 7: sub.setMode('ALT_HOLD', 'button'); break;
      case 8: sub.setMode('POSHOLD', 'button'); break;
      case 9: sub.setMode('AUTO', 'button'); break;
      case 10: sub.setMode('CIRCLE', 'button'); break;
      case 11: sub.setMode('GUIDED', 'button'); break;
      case 12: sub.setMode('ACRO', 'button'); break;
      case 13: sub.setMode('SURFTRAK', 'button'); break;
      // camera_mount.set_angle_target(0,0,0) then RC targeting (joystick.cpp:189-195)
      case 21: if ((P.MNT1_TYPE ?? 1) !== 0) { this.mount.roll = 0; this.mount.pitch = 0; this.mount.yaw = 0; this.mount.mode = 3; } break;
      case 22: this.camTilt = 1900; break;
      case 23: this.camTilt = 1100; break;
      case 24: break; // camera_trigger: not implemented in ArduSub 4.5.7
      case 25: if (!held) {
        this.videoToggle = !this.videoToggle;
        this.videoSwitch = this.videoToggle ? 1000 : 0;
        this.cameraSource = this.videoToggle ? 2 : 1;
        sub.text(6, `Video Toggle: Source ${this.cameraSource}`);
      } break;
      case 26: this.camPan = 1900; break;
      case 27: this.camPan = 1100; break;
      case 31: case 34: if (!held) {
        const k = fn === 31 ? 0 : 1;
        const key = k === 0 ? 'lights1' : 'lights2';
        this[key] = clamp(this[key] + (this.lightsIncreasing[k] ? lightStep : -lightStep), 0, 1000);
        if (this[key] >= 1000 || this[key] <= 0) this.lightsIncreasing[k] = !this.lightsIncreasing[k];
      } break;
      case 32: if (!held) this.lights1 = clamp(this.lights1 + lightStep, 0, 1000); break;
      case 33: if (!held) this.lights1 = clamp(this.lights1 - lightStep, 0, 1000); break;
      case 35: if (!held) this.lights2 = clamp(this.lights2 + lightStep, 0, 1000); break;
      case 36: if (!held) this.lights2 = clamp(this.lights2 - lightStep, 0, 1000); break;
      case 41: if (!held) { this.gainToggleLow = !this.gainToggleLow; sub.gain = this.gainToggleLow ? 0.5 : 1; sub.text(6, `#Gain: ${Math.round(sub.gain * 100)}%`); } break;
      case 42: case 43: if (!held) this.gainStep(fn === 42 ? 1 : -1); break;
      case 44: this.rollTrim = trunc(clamp(this.rollTrim + 10, -200, 200)); break;
      case 45: this.rollTrim = trunc(clamp(this.rollTrim - 10, -200, 200)); break;
      case 46: this.pitchTrim = trunc(clamp(this.pitchTrim + 10, -200, 200)); break;
      case 47: this.pitchTrim = trunc(clamp(this.pitchTrim - 10, -200, 200)); break;
      case 48: if (sub.armed && !held) {
        this.zTrim = Math.abs(this.zLast - 500) > 50 ? this.zLast - 500 : 0;
        this.xTrim = Math.abs(this.xLast) > 50 ? this.xLast : 0;
        this.yTrim = Math.abs(this.yLast) > 50 ? this.yLast : 0;
        const last = this.inputHoldEngaged;
        this.inputHoldEngaged = !!(this.zTrim || this.xTrim || this.yTrim);
        if (this.inputHoldEngaged) sub.text(6, '#Input Hold Set');
        else if (last) sub.text(6, '#Input Hold Disabled');
        this.controlsResetSinceInputHold = !this.inputHoldEngaged;
      } break;
      case 49: if (!held) { this.rollPitchFlag = this.rollPitchFlag ? 0 : 1; sub.text(6, this.rollPitchFlag ? '#Attitude Control' : '#Movement Control'); } break;
      case 108: case 109: case 110: case 111: { const b = this.scriptButtons[fn - 108]; if (!b.pressed) b.count++; b.pressed = true; } break;
      default: break; // custom_1-6: not implemented (joystick.cpp:603-620)
    }
  }
  // k_gain_inc / k_gain_dec (joystick.cpp:293-323), including the set_and_save of the constrained limits
  gainStep(dir) {
    const sub = this.sub, P = sub.params;
    this.setAndSave('JS_GAIN_MIN', clamp(P.JS_GAIN_MIN ?? 0.25, 0.1, 0.8));
    this.setAndSave('JS_GAIN_MAX', clamp(P.JS_GAIN_MAX ?? 1, P.JS_GAIN_MIN, 1));
    this.setAndSave('JS_GAIN_STEPS', clamp(trunc(P.JS_GAIN_STEPS ?? 4), 1, 10));
    const minG = P.JS_GAIN_MIN, maxG = P.JS_GAIN_MAX, n = P.JS_GAIN_STEPS;
    if (n === 1) sub.gain = f32(clamp(P.JS_GAIN_DEFAULT ?? 0.5, minG, maxG));
    else sub.gain = f32(clamp(sub.gain + dir * ((maxG - minG) / (n - 1)), minG, maxG));
    sub.text(6, `#Gain is ${Math.round(sub.gain * 100)}%`);
  }
  // Sub::handle_jsbutton_release (joystick.cpp:639)
  handleRelease(i, shift) {
    const fn = this.buttonFunction(i, shift);
    if (fn >= 104 && fn <= 107) { this.relayOp(fn - 104, 'off'); return; }
    const act = actuatorAction(fn);
    if (act && (act[1] === 'min_momentary' || act[1] === 'max_momentary')) { this.actuatorOp(act[0], 'center'); return; }
    if (fn >= 108 && fn <= 111) this.scriptButtons[fn - 108].pressed = false;
  }
  setNeutralControls() {
    const rc = this.sub.rc, now = this.nowMs();
    for (let i = 0; i < 6; i++) rc.all[i].setOverride(1500, now);
    this.pitchTrim = 0; this.rollTrim = 0;
  }
  clearInputHold() { this.xTrim = 0; this.yTrim = 0; this.zTrim = 0; this.inputHoldEngaged = false; }
  // Sub::is_button_pressed / get_and_clear_button_count (scripting bindings)
  isScriptButtonPressed(k) { return !!this.scriptButtons[k - 1]?.pressed; }
  getAndClearScriptButtonCount(k) { const b = this.scriptButtons[k - 1]; if (!b) return 0; const c = b.count; b.count = 0; return c; }
}
