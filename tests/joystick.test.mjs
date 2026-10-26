// joystick.cpp / AP_JSButton / RC_Channel port: integer maths, button functions,
// outputs (lights, mount, relays, actuators) and keyboard/gamepad helpers.
import test from 'node:test';
import assert from 'node:assert/strict';
import { JoystickHandler, BUTTON_FUNCTIONS, actuatorAction, relayAction } from '../src/ardusub/joystick.js';
import { makeSubChannels, RCChannel, RC_OPTION } from '../src/ardusub/rc.js';
import { triggerContribution, shiftButtonIndex, isTextEntry } from '../src/ui/input.js';

const BTN_DEFAULTS = [[0, 0], [5, 0], [7, 0], [6, 0], [4, 0], [1, 0], [3, 0], [21, 0], [48, 0], [23, 27], [22, 26], [42, 47], [43, 46], [33, 45], [32, 44], [0, 0]];
function fakeSub(extra = {}) {
  const P = { JS_GAIN_DEFAULT: 0.5, JS_GAIN_MIN: 0.25, JS_GAIN_MAX: 1, JS_GAIN_STEPS: 4, JS_THR_GAIN: 1, JS_LIGHTS_STEPS: 8, RC3_TRIM: 1100 };
  BTN_DEFAULTS.forEach(([f, s], i) => { P[`BTN${i}_FUNCTION`] = f; P[`BTN${i}_SFUNCTION`] = s; });
  Object.assign(P, extra);
  const sub = { params: P, time: 0, gain: 0.5, armed: false, msgs: [], mode: 'MANUAL', motors: { pwm: [1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500] },
    text(sev, m) { this.msgs.push(m); }, arm() { this.armed = true; }, disarm() { this.armed = false; }, setMode(m) { this.mode = m; } };
  sub.rc = makeSubChannels(P);
  sub.js = new JoystickHandler(sub);
  sub.js.init(P);
  return sub;
}
const press = (sub, buttons, m = {}) => sub.js.transform(m.x ?? 0, m.y ?? 0, m.z ?? 500, m.r ?? 0, buttons, 0, 0, m.buttons2 ?? 0);
const tap = (sub, bit, m) => { press(sub, 1 << bit, m); press(sub, 0, m); };

test('BTN function enum is the full 4.5.7 list (0-13, 21-27, 31-36, 41-49, 51-59, 61-87, 91-96, 101-138)', () => {
  const want = [];
  const r = (a, b) => { for (let i = a; i <= b; i++) want.push(i); };
  r(0, 13); r(21, 27); r(31, 36); r(41, 49); r(51, 59); r(61, 87); r(91, 96); r(101, 138);
  assert.deepEqual(Object.keys(BUTTON_FUNCTIONS).map(Number).sort((a, b) => a - b), want);
  assert.equal(BUTTON_FUNCTIONS[77], 'actuator_1_max_momentary');
  assert.equal(BUTTON_FUNCTIONS[138], 'actuator_6_max_toggle');
  assert.deepEqual(actuatorAction(119), [3, 'min_toggle']);
  assert.deepEqual(actuatorAction(66), [1, 'inc']);
  assert.deepEqual(relayAction(102), [3, 'off']);
});

test('throttle base is int16-truncated as in joystick.cpp:61', () => {
  const sub = fakeSub({ JS_THR_GAIN: 1.3 });
  sub.gain = Math.fround(0.11);
  press(sub, 0, { z: 500 });
  // throttleScale = 0.8*0.11*1.3 = 0.1144 -> base = int16(1442.8) = 1442 -> 500*0.1144 + 1442 = 1499.2 -> 1499
  assert.equal(sub.rc.throttle.radioIn, 1499);
  sub.gain = 0.5; press(sub, 0, { z: 500 });
  assert.equal(sub.rc.throttle.radioIn, 1500);
});

test('light step is uint16-truncated (JS_LIGHTS_STEPS 3 needs 4 presses to reach 1000)', () => {
  const sub = fakeSub({ JS_LIGHTS_STEPS: 3, BTN14_FUNCTION: 32 });
  const seen = [];
  for (let k = 0; k < 4; k++) { tap(sub, 14); seen.push(sub.js.lights1); }
  assert.deepEqual(seen, [333, 666, 999, 1000]);
  // lights1/lights2 cycle keep separate direction state (static bool per case)
  const s2 = fakeSub({ JS_LIGHTS_STEPS: 2, BTN0_FUNCTION: 31, BTN15_FUNCTION: 34 });
  tap(s2, 0); tap(s2, 0); tap(s2, 0);
  assert.equal(s2.js.lights1, 500);
  tap(s2, 15);
  assert.equal(s2.js.lights2, 500);
});

test('lights1 PWM follows SERVOn_FUNCTION 181 and SERVOn_MIN/MAX/REVERSED', () => {
  const sub = fakeSub({ SERVO9_FUNCTION: 181, SERVO9_MIN: 1100, SERVO9_MAX: 1900, BTN14_FUNCTION: 32 });
  for (let k = 0; k < 4; k++) tap(sub, 14);
  const o = sub.js.servoOutputs().find((c) => c.n === 9);
  assert.equal(o.label, 'Lights1');
  assert.equal(o.pwm, 1500);
  assert.equal(sub.js.lightLevel(0), 0.5);
  sub.params.SERVO9_REVERSED = 1;
  assert.equal(sub.js.servoOutputs()[8].pwm, 1500);
  tap(sub, 14);
  assert.equal(sub.js.servoOutputs()[8].pwm, 1400);
  // no channel with lights1: no light
  const none = fakeSub({ SERVO9_FUNCTION: 0, BTN14_FUNCTION: 0 });
  assert.equal(none.js.lightLevel(0), 0);
});

test('relays: momentary press turns on, release off; only RELAYn_FUNCTION = 1 relays switch', () => {
  const sub = fakeSub({ BTN0_FUNCTION: 104, BTN15_FUNCTION: 103, RELAY1_FUNCTION: 1, RELAY4_FUNCTION: 1 });
  press(sub, 1 << 0);
  assert.equal(sub.js.relays[0], true);
  press(sub, 1 << 0); // held
  assert.equal(sub.js.relays[0], true);
  press(sub, 0);
  assert.equal(sub.js.relays[0], false);
  tap(sub, 15);
  assert.equal(sub.js.relays[3], true);
  press(sub, 1 << 15); press(sub, 1 << 15); press(sub, 0); // toggle acts on the first press only
  assert.equal(sub.js.relays[3], false);
  const off = fakeSub({ BTN0_FUNCTION: 104, RELAY1_FUNCTION: 0 });
  press(off, 1);
  assert.equal(off.js.relays[0], false);
});

test('actuators follow ACTUATORn_INC and SERVOn range; momentary release centres', () => {
  const sub = fakeSub({ SERVO11_FUNCTION: 184, SERVO11_TRIM: 1500, BTN0_FUNCTION: 61, BTN15_FUNCTION: 77, ACTUATOR1_INC: 0.05 });
  const pwm = () => sub.js.servoOutputs()[10].pwm;
  assert.equal(pwm(), 1500); // initialize_actuators(): starts at trim
  tap(sub, 0);
  assert.equal(pwm(), 1540);
  press(sub, 1 << 15);
  assert.equal(pwm(), 1900);
  press(sub, 0);
  assert.equal(pwm(), 1500);
  // max_toggle: < 0.6 -> 1, >= 0.6 -> 0.5
  sub.params.BTN15_FUNCTION = 79;
  tap(sub, 15); assert.equal(pwm(), 1900);
  tap(sub, 15); assert.equal(pwm(), 1500);
});

test('actuator 6 max has no press handler in joystick.cpp (kept as in 4.5.7)', () => {
  const sub = fakeSub({ SERVO12_FUNCTION: 189, BTN0_FUNCTION: 131, BTN15_FUNCTION: 130 });
  tap(sub, 0);
  assert.equal(sub.js.actuators[5], 0.5);
  tap(sub, 15);
  assert.equal(sub.js.actuators[5], 0);
});

test('legacy actuator buttons claim a disabled SERVO9-11 at boot (update_actuators_from_jsbuttons)', () => {
  const sub = fakeSub({ SERVO9_FUNCTION: 0, BTN2_SFUNCTION: 63 });
  assert.equal(sub.params.SERVO9_FUNCTION, 184);
  assert.deepEqual(sub.js.pendingSaves.find(([n]) => n === 'SERVO9_FUNCTION'), ['SERVO9_FUNCTION', 184]);
  // the DUBURI file: SERVO9 is lights1, so actuator 1 stays unassigned
  const duburi = fakeSub({ SERVO9_FUNCTION: 181, SERVO11_FUNCTION: 0, BTN2_SFUNCTION: 63 });
  assert.equal(duburi.params.SERVO11_FUNCTION, 0);
  assert.equal(duburi.js.findServo(184), 0);
});

test('shift (k_shift) selects BTNn_SFUNCTION; buttons2 carries 16-31', () => {
  const sub = fakeSub({ BTN2_SFUNCTION: 12, BTN20_FUNCTION: 8 });
  press(sub, (1 << 5) | (1 << 2)); press(sub, 0);
  assert.equal(sub.mode, 'ACRO');
  press(sub, 0, { buttons2: 1 << 4 }); press(sub, 0);
  assert.equal(sub.mode, 'POSHOLD');
  // legacy 32-bit buttons word via handleMessage
  sub.js.handleMessage({ x: 0, y: 0, z: 500, r: 0, buttons: 1 << 2 }); sub.js.handleMessage({ x: 0, y: 0, z: 500, r: 0, buttons: 0 });
  assert.equal(sub.mode, 'ALT_HOLD');
});

test('MANUAL_CONTROL only from SYSID_MYGCS to SYSID_THISMAV (GCS_Mavlink.cpp:584-592)', () => {
  const sub = fakeSub({ SYSID_MYGCS: 255, SYSID_THISMAV: 1 });
  assert.equal(sub.js.handleMessage({ sysid: 1, x: 500, z: 500, buttons: 0 }), false);
  assert.equal(sub.js.handleMessage({ sysid: 255, target: 2, x: 500, z: 500, buttons: 0 }), false);
  assert.equal(sub.rc.forward.radioIn, 1500);
  assert.equal(sub.js.handleMessage({ sysid: 255, target: 1, x: 500, z: 500, buttons: 0 }), true);
  assert.equal(sub.rc.forward.radioIn, 1600);
});

test('gain_inc saves the constrained JS_GAIN_MIN/MAX/STEPS (joystick.cpp:296-298)', () => {
  const sub = fakeSub({ JS_GAIN_MIN: 0.05, JS_GAIN_STEPS: 12 });
  sub.js.gainStep(1);
  assert.equal(sub.params.JS_GAIN_MIN, 0.1);
  assert.equal(sub.params.JS_GAIN_STEPS, 10);
  assert.ok(sub.js.pendingSaves.some(([n, v]) => n === 'JS_GAIN_MIN' && v === 0.1));
  assert.ok(Math.abs(sub.gain - (0.5 + 0.9 / 9)) < 1e-6);
});

test('camera mount: RC8 rate input at MNT1_RC_RATE, limited by MNT1_PITCH_MAX, servo PWM on SERVO10', () => {
  const sub = fakeSub({ MNT1_TYPE: 1, SERVO10_FUNCTION: 7 });
  for (let k = 0; k <= 25; k++) { sub.time = k * 0.02; press(sub, 1 << 10); } // tilt up held 0.5 s
  assert.ok(Math.abs(sub.js.mount.pitch - 15) < 0.7, `pitch ${sub.js.mount.pitch}`);
  for (let k = 26; k < 100; k++) { sub.time = k * 0.02; press(sub, 1 << 10); }
  assert.equal(sub.js.mount.pitch, 20);
  const pwm = sub.js.servoOutputs()[9].pwm;
  assert.equal(pwm, Math.trunc(1100 + (1100 / 1100) * 800));
  assert.equal(Math.round(sub.js.cameraTiltDeg()), 20);
  // mount_center
  press(sub, 1 << 7); press(sub, 0);
  assert.equal(sub.js.mount.pitch, 0);
  // no mount: tilt buttons do nothing on the vehicle
  const nom = fakeSub({ MNT1_TYPE: 0, SERVO10_FUNCTION: 7 });
  for (let k = 0; k < 50; k++) { nom.time = k * 0.02; press(nom, 1 << 10); }
  assert.equal(nom.js.mount.pitch, 0);
  assert.equal(nom.js.cameraTiltDeg(), null);
  assert.equal(nom.js.servoOutputs()[9].pwm, null);
});

test('RC overrides: RC_OVERRIDE_TIME and RC_OPTIONS ignore-overrides', () => {
  const rc = makeSubChannels({ RC_OVERRIDE_TIME: 3 });
  rc.deferRead = true;
  rc.forward.setOverride(1700, 1000);
  assert.equal(rc.forward.radioIn, 1500);
  rc.readInput(1020);
  assert.equal(rc.forward.radioIn, 1700);
  assert.ok(rc.forward.hasOverride(3999));
  assert.ok(!rc.forward.hasOverride(4000));
  const ign = makeSubChannels({ RC_OPTIONS: RC_OPTION.IGNORE_OVERRIDES });
  ign.forward.setOverride(1700, 0);
  assert.equal(ign.forward.radioIn, 1500);
  const never = makeSubChannels({ RC_OVERRIDE_TIME: -1 });
  never.forward.setOverride(1700, 0);
  assert.ok(never.forward.hasOverride(1e9));
  assert.equal(rc.all.length, 16);
  assert.equal(rc.findChannelForOption(213), rc.all[7]);
  const c = new RCChannel({ trim: 1500, dz: 50 });
  c.radioIn = 1700;
  assert.ok(Math.abs(c.normInputDz() - 150 / 350) < 1e-12);
});

test('gamepad triggers: Ascend/Descend have the same sign on either trigger', () => {
  assert.equal(triggerContribution('up', 1, 'left').z, 1);
  assert.equal(triggerContribution('down', 1, 'left').z, -1);
  assert.equal(triggerContribution('up', 0.5, 'right').z, 0.5);
  assert.equal(triggerContribution('pitch', 1, 'left').s, -1);
  assert.equal(triggerContribution('roll', 1, 'right').t, 1);
});

test('keyboard Shift presses the BTNn set to shift; text fields take the keyboard, menus do not', () => {
  assert.equal(shiftButtonIndex({ BTN5_FUNCTION: 1 }), 5);
  assert.equal(shiftButtonIndex({ BTN5_FUNCTION: 2, BTN9_FUNCTION: 1 }), 9);
  assert.equal(shiftButtonIndex({ ...Object.fromEntries(Array.from({ length: 32 }, (_, i) => [`BTN${i}_FUNCTION`, 0])) }), -1);
  assert.equal(isTextEntry({ tagName: 'INPUT', type: 'number' }), true);
  assert.equal(isTextEntry({ tagName: 'INPUT', type: 'checkbox' }), false);
  assert.equal(isTextEntry({ tagName: 'SELECT' }), false);
  assert.equal(isTextEntry({ tagName: 'TEXTAREA' }), true);
});
