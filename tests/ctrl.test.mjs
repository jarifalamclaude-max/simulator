// CTRL port checks against hand-computed ArduSub 4.5.7 behaviour (AP_Motors6DOF, AP_MotorsMulticopter,
// AC_PID family, AP_Math/control.cpp, AC_AttitudeControl(_Sub), AC_PosControl).
import test from 'node:test';
import assert from 'node:assert/strict';
import { Motors6DOF, SPOOL, PWM_TYPE, SubMotorTest, servoChannelMap } from '../src/ardusub/motors.js';
import { AC_PID, AC_PID_2D, AC_PID_Basic, AC_P_1D, AC_P_2D } from '../src/ardusub/pid.js';
import { calcLowpassAlpha, sqrtController, limitAccelXY, LowPassFilter, LowPassFilter2p, NotchFilter, shapeVelAccel, kinematicLimit, filterBankFromParams } from '../src/ardusub/control.js';
import { AttitudeControlSub, inputShapingAngVel, inputShapingAngle } from '../src/ardusub/attitude.js';
import { PosControl } from '../src/ardusub/poscontrol.js';
import { frameTable } from '../src/alloc/frames.js';
import { wrap180cd, constrainFloat, qFromEuler, qToAxisAngle, qFromAxisAngleVec } from '../src/core/math.js';

const near = (a, b, tol = 1e-9, msg = '') => assert.ok(Math.abs(a - b) <= tol, `${msg} ${a} vs ${b}`);
const DT = 0.0025;
const P0 = { MOT_PWM_MIN: 1100, MOT_PWM_MAX: 1900, MOT_SPOOL_TIME: 0.5 };
function spun(params = {}, frame = 2, variant = 'vectored6dof') {
  const m = new Motors6DOF();
  m.configure({ factors: frameTable(frame), variant, params: { ...P0, ...params } });
  m.arm(); m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
  m.setThrottle(0.5);
  for (let i = 0; i < 400; i++) m.output(DT);
  return m;
}

// ---------------- motors ----------------
test('generic mixer: MOT_n_DIRECTION is an int8 factor (0 silences, 2 doubles before the clamp)', () => {
  const m = spun({ MOT_1_DIRECTION: 0, MOT_2_DIRECTION: 2 }, 0, 'generic'); // BlueROV1: motors 1,2 forward 1.0
  m.setForward(0.3); m.output(DT);
  near(m.thrust[0], 0); near(m.thrust[1], 0.6);
  m.setForward(0.8); m.output(DT);
  near(m.thrust[1], 1); // constrain_float(2*0.8, -1, 1)
});

test('vectored mixer: forward/vertical coupling limits the rear thrusters (MOT_FV_CPLNG_K)', () => {
  const m = spun({ MOT_FV_CPLNG_K: 1 }, 1, 'vectored');
  m.setThrottle(0.8); for (let i = 0; i < 400; i++) m.output(DT); // bidirectional throttle 0.6 -> limit 0.4
  m.setForward(1); m.output(DT);
  // motor 3 (dir +1, fwd factor +1) is limited for positive forward: 0.4; motor 1 (dir -1) is not: -1
  near(m.thrust[2], 0.4 * 1 + -1 * 0 + 0, 1e-9); near(m.thrust[0], -1, 1e-9);
});

test('setDesiredSpoolState is ignored while disarmed (AP_Motors_Class.cpp:86)', () => {
  const m = new Motors6DOF();
  m.configure({ factors: frameTable(2), variant: 'vectored6dof', params: P0 });
  m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
  assert.equal(m.desiredSpool, SPOOL.SHUT_DOWN);
  m.arm(); m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
  assert.equal(m.desiredSpool, SPOOL.GROUND_IDLE);
});

test('spool: MOT_SPOOL_TIM_DN sets the spool-down time, ground idle ramps to SPIN_ARM/SPIN_MIN', () => {
  const m = spun({ MOT_SPOOL_TIM_DN: 0.2, MOT_SPIN_ARM: 0.1, MOT_SPIN_MIN: 0.2 });
  assert.equal(m.spool, SPOOL.THROTTLE_UNLIMITED);
  m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
  m.output(DT); // THROTTLE_UNLIMITED -> SPOOLING_DOWN
  let n = 0;
  while (m.spool === SPOOL.SPOOLING_DOWN && n < 1000) { m.output(DT); n++; }
  assert.equal(m.spool, SPOOL.GROUND_IDLE);
  assert.ok(n >= 79 && n <= 81, `${n} steps`); // 0.2 s / dt, ending in the same call
  for (let i = 0; i < 400; i++) m.output(DT);
  near(m.spinUpRatio, 0.5, 1e-9, 'spin_up_ratio -> 0.1/0.2'); // spin_arm / spin_min
});

test('spool: MOT_SAFE_DISARM holds SHUT_DOWN for MOT_SAFE_TIME after arming', () => {
  const m = new Motors6DOF();
  m.configure({ factors: frameTable(2), variant: 'vectored6dof', params: { ...P0, MOT_SAFE_DISARM: 1, MOT_SAFE_TIME: 0.5 } });
  m.arm(); m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
  for (let i = 0; i < 199; i++) m.output(DT);
  assert.equal(m.spool, SPOOL.SHUT_DOWN);
  for (let i = 0; i < 3; i++) m.output(DT);
  assert.notEqual(m.spool, SPOOL.SHUT_DOWN);
});

test('battery current limiter (AP_Motors6DOF.cpp:362) scales every output', () => {
  const m = spun({ MOT_BAT_CURR_MAX: 20, MOT_BAT_CURR_TC: 5 });
  m.setForward(0.5); m.output(DT);
  const free = m.thrust.slice();
  m.setBatteryReadings(16, 30, true); m.battCurrentLast = 30; // steady 30 A, ratio 1.5
  m.output(DT);
  const k = 1 + (DT / (DT + 5)) * (1 - 1.5);
  near(m.outputLimited, k, 1e-12);
  m.thrust.forEach((t, i) => near(t, free[i] * k, 1e-12));
  // predicted current above 1.5 * Imax forces ratio 2.5
  const before = m.outputLimited;
  m.setBatteryReadings(16, 35, true); m.battCurrentLast = 10; // predicted 35 + 25*5 = 160 A
  m.output(DT);
  near(m.outputLimited, before + (DT / (DT + 5)) * (1 - 2.5), 1e-12);
  // unhealthy monitor disables it
  const lim = m.outputLimited; m.setBatteryReadings(16, 100, false); m.output(DT); near(m.outputLimited, lim);
});

test('MOT_THST_HOVER is forced to 0.5 at boot and clamped by get_throttle_hover()', () => {
  const m = new Motors6DOF();
  m.configure({ factors: frameTable(2), variant: 'vectored6dof', params: { ...P0, MOT_THST_HOVER: 0.35 } });
  near(m.throttleHover, 0.5);
  m.setParams({ ...P0, MOT_THST_HOVER: 0.8 }); // runtime edit after boot
  near(m.throttleHover, 0.6875);
});

test('MOT_PWM_TYPE DShot / PWMRange force the 1000-2000 endpoints; RC3 endpoints seed MOT_PWM_*', () => {
  const m = spun({ MOT_PWM_TYPE: PWM_TYPE.DSHOT600 });
  assert.equal(m.pwmMin, 1000); assert.equal(m.pwmMax, 2000);
  m.setForward(1); m.output(DT);
  assert.equal(m.pwm[3], 2000);
  const r = new Motors6DOF();
  r.configure({ factors: frameTable(2), variant: 'vectored6dof', params: { RC3_MIN: 1150, RC3_MAX: 1850 } });
  assert.equal(r.pwmMin, 1150); assert.equal(r.pwmMax, 1850);
});

test('SERVOn_FUNCTION routing, output_ch_all latency and RC_SPEED pulse rate', () => {
  const sm = servoChannelMap({ SERVO1_FUNCTION: 34, SERVO2_FUNCTION: 33 }, 3);
  assert.deepEqual(sm.map, [[1], [0], [2]]); // motor 3 defaults onto the free channel 3
  const m = spun({ SERVO1_FUNCTION: 34, SERVO2_FUNCTION: 33, RC_SPEED: 400 });
  m.setLateral(1);
  const a = m.output(DT).slice(); // hardware still has last loop's values
  const b = m.output(DT).slice();
  assert.equal(a[0], 1500); assert.equal(m.pwm[0], 1900); assert.equal(m.pwm[1], 1100);
  assert.equal(b[0], 1100); assert.equal(b[1], 1900); // motor 2 on SERVO1, motor 1 on SERVO2
  const s = spun({ RC_SPEED: 200 }); // 400 Hz loop, 200 Hz pulses: frames update every other loop
  s.output(DT); // odd phase
  s.setForward(1);
  const o = [s.output(DT)[3], s.output(DT)[3], s.output(DT)[3]];
  assert.deepEqual(o, [1500, 1500, 1900]); // latched at 402 (old), held at 403, latched new at 404
});

test('PWMRange output maps through SERVOn_MIN/MAX/REVERSED', () => {
  const m = spun({ MOT_PWM_TYPE: PWM_TYPE.PWM_RANGE, SERVO4_MIN: 1200, SERVO4_MAX: 1800, SERVO4_REVERSED: 1, RC_SPEED: 400 });
  m.setForward(1); m.setThrottle(0.5); m.output(DT); const o = m.output(DT);
  assert.equal(m.pwm[3], 2000); // motor 4 full forward
  assert.equal(o[3], 1200); // reversed range: scaled 1000 -> 0 -> SERVO_MIN
});

test('motor test: raw PWM to one motor, others keep their value; 500 ms timeout disarms with a 10 s lockout', () => {
  const m = spun();
  let armed = true; const msgs = [];
  const mt = new SubMotorTest({ motors: m, isArmed: () => armed, disarm: () => { armed = false; m.disarm(); }, text: (s, t) => msgs.push(t), throttleRange: () => [1100, 1900] });
  m.setForward(1); m.output(DT);
  assert.ok(mt.handle(10000, { param1: 2, param2: 1, param3: 2100, y: 2 }));
  assert.equal(m.pwm[2], 2100); // not clamped to MOT_PWM_MAX
  assert.equal(m.pwm[0], 1500); // output_min at init
  assert.ok(mt.handle(10200, { param1: 0, param2: 0, param3: 75, y: 2 }));
  assert.equal(m.pwm[0], 1700); assert.equal(m.pwm[2], 2100); // PERCENT on RC3_MIN..MAX
  assert.ok(!mt.handle(10300, { param1: 0, param2: 2, param3: 50, y: 2 }), 'PILOT throttle type rejected');
  assert.ok(!mt.handle(10300, { param1: 0, param2: 1, param3: 1600, y: 1 }), 'only board order');
  assert.ok(mt.verify(10800));
  assert.ok(!mt.verify(10801)); assert.ok(!armed);
  armed = true; m.arm();
  assert.ok(!mt.handle(15000, { param1: 0, param2: 1, param3: 1600, y: 2 }), 'cooldown');
  assert.ok(msgs.includes('Motor test timed out!'));
});

test('mixer and PWM-type changes are queued while armed and applied at disarm', () => {
  const m = spun();
  m.configure({ factors: frameTable(1), variant: 'vectored', params: { ...P0, MOT_2_DIRECTION: -1 } });
  assert.equal(m.factors.length, 8); assert.equal(m.reverse[1], -1); // live MOT_* still applied
  m.disarm();
  assert.equal(m.factors.length, 6); assert.equal(m.variant, 'vectored');
});

test('get_motor_angular_factors / set_reversed for MOTOR_DETECT', () => {
  const m = spun();
  assert.deepEqual(m.getMotorAngularFactors(4), [1, -1, 0]);
  assert.deepEqual(m.setReversed(4, true), { name: 'MOT_5_DIRECTION', value: -1 });
});

// ---------------- math / control.cpp ----------------
test('AP_Math edge cases: calc_lowpass_alpha_dt, wrap_180_cd, constrain NaN, axis-angle epsilon', () => {
  assert.equal(calcLowpassAlpha(0, 2), 0); assert.equal(calcLowpassAlpha(0.01, 0), 1); assert.equal(calcLowpassAlpha(-1, 2), 1);
  near(calcLowpassAlpha(0.01, 10), 0.01 / (0.01 + 1 / (2 * Math.PI * 10)));
  assert.equal(wrap180cd(-18000), 18000); assert.equal(wrap180cd(-54000), 18000); assert.equal(wrap180cd(27000), -9000);
  assert.equal(constrainFloat(NaN, -2, 4), 1);
  assert.deepEqual(qFromAxisAngleVec([1e-8, 0, 0]), [1, 0, 0, 0]);
  assert.deepEqual(qToAxisAngle([1, 1e-9, 0, 0]), [1e-9, 0, 0]);
  const f = new LowPassFilter(1); f.reset(0); assert.equal(f.apply(10, 0), 0); // dt = 0 holds
});

test('sqrt_controller and shaping match hand values', () => {
  near(sqrtController(2, 1, 1, 0), Math.sqrt(3)); // linear_dist 1 -> sqrt(2*1*(2-0.5))
  near(sqrtController(0.5, 1, 1, 0), 0.5);
  near(sqrtController(2, 1, 1, 0.1), Math.sqrt(3)); // |error|/dt = 20 does not bind
  near(sqrtController(0.01, 100, 0, 0.0025), 1); // linear p*e = 1 (inside the |e|/dt = 4 bound)
  near(sqrtController(0.01, 1000, 0, 0.0025), 4); // p*e = 10 clipped to |e|/dt
  // shape_vel_accel: accel slews at jerk_max*dt toward the sqrt-controller target
  const s = { accel: 0 };
  shapeVelAccel(100, 0, 0, s, -250, 250, 500, 0.0025, true);
  near(s.accel, 500 * 0.0025);
  near(inputShapingAngVel(0, 10, 2, 0.01, 0), 0.02); // accel-limited
  near(inputShapingAngle(1, 0.15, 0, 0, 0, 0, 0.01), 1 / 0.15); // linear 1/input_tc when accel_max = 0
  near(inputShapingAngle(1, 0.15, 0, 0, 0.01), 1 / 0.15); // 5-argument overload
  near(kinematicLimit([1, 0, 1], 100, 50, 20), 50 / Math.SQRT1_2);
});

test('limit_accel_xy keeps the cross-track component first', () => {
  const a = [3, 4];
  assert.ok(limitAccelXY([1, 0], a, 4));
  near(a[0], 0); near(a[1], 4);
  const b = [3, 4];
  assert.ok(limitAccelXY([0, 0], b, 2.5)); near(Math.hypot(...b), 2.5);
  const c = [1, 1]; assert.ok(!limitAccelXY([1, 0], c, 0)); // accel_max <= 0: untouched
});

test('LowPassFilter2p clamps the cutoff to 0.4*fs instead of passing through', () => {
  const f = new LowPassFilter2p(50, 30);
  assert.equal(f.pass, false); near(f.cutoffHz, 20);
  assert.equal(new LowPassFilter2p(400, 0).pass, true);
});

test('notch filter attenuates its centre frequency by NOTCH_ATT', () => {
  const n = new NotchFilter(); n.init(400, 40, 20, 40); // Q = 40/20 -> bandwidth 20 Hz, 40 dB
  let peak = 0;
  for (let k = 0; k < 4000; k++) { const y = n.apply(Math.sin((2 * Math.PI * 40 * k) / 400)); if (k > 3000) peak = Math.max(peak, Math.abs(y)); }
  near(peak, 0.01, 0.002); // 10^(-40/20)
  const m = new NotchFilter(); m.init(400, 40, 20, 40); let pk = 0;
  for (let k = 0; k < 4000; k++) { const y = m.apply(Math.sin((2 * Math.PI * 5 * k) / 400)); if (k > 3000) pk = Math.max(pk, Math.abs(y)); }
  assert.ok(pk > 0.95, `pass band ${pk}`);
});

// ---------------- AC_PID family ----------------
test('AC_PID: D_FF feed-forward, FF info, PDMX limit flag, I_term_set', () => {
  const pid = new AC_PID({ p: 1, i: 0, d: 0, ff: 0.2, dff: 0.01, imax: 1, fltt: 0, flte: 0, fltd: 0 });
  pid.updateAll(0, 0, 0.01);
  pid.updateAll(1, 0, 0.01); // target derivative 100/s
  near(pid.getFF(), 0.2 * 1 + 0.01 * 100);
  const q = new AC_PID({ p: 1, i: 0, d: 0, imax: 1, pdmx: 0.3 });
  near(q.updateAll(1, 0, 0.01), 0.3); assert.equal(q.info.PD_limit, true);
  q.setIntegrator(0.1); q.updateAll(1, 0, 0.01); assert.equal(q.info.I_term_set, true);
  q.updateAll(1, 0, 0.01); assert.equal(q.info.I_term_set, false);
  q.setIntegrator(NaN); near(q.integrator, 0); // constrain_float NaN -> midpoint
});

test('AC_PID: SMAX slew limiter reduces P+D under oscillation and is inert at 0', () => {
  const run = (smax) => {
    const pid = new AC_PID({ p: 0.135, i: 0, d: 0.0036, imax: 0.444, fltt: 30, flte: 0, fltd: 30, smax });
    let peak = 0, minMod = 1;
    for (let k = 0; k < 1600; k++) {
      const t = k * DT, meas = Math.sin(2 * Math.PI * 8 * t);
      const out = pid.updateAll(0, meas, DT);
      if (k > 800) { peak = Math.max(peak, Math.abs(out)); minMod = Math.min(minMod, pid.info.Dmod); }
    }
    return { peak, minMod };
  };
  const off = run(0), on = run(2);
  assert.equal(off.minMod, 1);
  assert.ok(on.minMod < 0.6 && on.peak < 0.6 * off.peak, `Dmod ${on.minMod}, peak ${on.peak} vs ${off.peak}`);
});

test('AC_PID_2D anti-windup lets the integrator shrink or rotate but not grow along the limit', () => {
  const mk = () => new AC_PID_2D({ p: 0, i: 1, d: 0, imax: 1000, flte: 0, fltd: 0 });
  const a = mk(); a.setIntegrator([-100, 0]);
  for (let k = 0; k < 100; k++) a.updateAll([10, 0], [0, 0], 0.01, [1, 0]);
  near(a.integrator[0], -90, 1e-9); near(a.integrator[1], 0);
  const b = mk(); b.setIntegrator([100, 0]);
  for (let k = 0; k < 100; k++) b.updateAll([0, 100], [0, 0], 0.01, [0, 1]);
  near(Math.hypot(...b.integrator), 100, 1e-6); assert.ok(b.integrator[1] > 40, `rotated ${b.integrator}`);
  const c = mk(); c.updateAll([NaN, 0], [0, 0], 0.01); assert.deepEqual(c.integrator, [0, 0]);
  c.setIntegratorTM([2, 0], [1, 0], [5, 0]); near(c.integrator[0], 5); // kp = 0
});

test('AC_PID_Basic bool-limit overload and AC_P_1D/AC_P_2D error limits', () => {
  const p = new AC_PID_Basic({ p: 0, i: 1, imax: 10 }); p.setIntegrator(1);
  p.updateAll(0, 1, 0.1, true); near(p.integrator, 1 - 0.1); // limit_pos only (I>0): negative error still integrates
  p.updateAll(1, 0, 0.1, true); near(p.integrator, 0.9); // positive error blocked
  const q = new AC_P_1D(1); q.setLimits(-100, 100, 0); q.setErrorLimits(-5, 7);
  near(q.getErrorMin(), -5); near(q.getErrorMax(), 7);
  near(q.updateAll(20, 0).target, 7);
  const r = new AC_P_2D(1); r.setLimits(100, 0); r.setErrorMax(3); near(r.getErrorMax(), 3);
});

// ---------------- attitude ----------------
const ahrsStub = (e = [0, 0, 0]) => ({ quat: qFromEuler(...e), gyro: [0, 0, 0], gyroLatest: [0, 0, 0], yaw: e[2], roll: e[0], pitch: e[1] });
function att(P = {}, e) {
  const m = new Motors6DOF(); m.configure({ factors: frameTable(2), variant: 'vectored6dof', params: P0 });
  const a = new AttitudeControlSub(ahrsStub(e), m); a.setParams(P); return a;
}

test('attitude: throttle-mix slew, avg max, pre-arm FF branch, relax at 90 deg pitch', () => {
  const a = att();
  a.setThrottleMixMax(); a.p.thrMixMax = 0.9; a.setThrottleMixMax();
  a.updateThrottleRpyMix(); near(a.throttleRpyMix, 0.5 + 2 * DT);
  near(a.getThrottleAvgMax(0.2), Math.max(0.2, 0.2 * (1 - a.throttleRpyMix) + 0.5 * a.throttleRpyMix));
  const b = att({ ATC_RAT_RLL_FF: 0.1, ATC_RAT_RLL_P: 0, ATC_RAT_RLL_I: 0 });
  assert.equal(b.preArmChecks(), null);
  const c = att({ ATC_RAT_RLL_P: 0 });
  assert.equal(c.preArmChecks(), 'ATC_RAT_RLL_P must be > 0');
  const d = att({}, [0, Math.PI / 2, 0]); d.eulerRateTarget = [0.1, 0.2, 0.3];
  d.relaxAttitudeControllers(); assert.deepEqual(d.eulerRateTarget, [0.1, 0.2, 0.3]);
});

test('attitude: setParams is live (no integrator reset) and wires FF/D_FF/SMAX', () => {
  const a = att();
  a.pidRoll.setIntegrator(0.2);
  a.setParams({ ATC_RAT_RLL_P: 0.2, ATC_RAT_RLL_FF: 0.05, ATC_RAT_RLL_D_FF: 0.01, ATC_RAT_RLL_SMAX: 10 });
  near(a.pidRoll.integrator, 0.2); near(a.pidRoll.kp, 0.2); near(a.pidRoll.kff, 0.05); near(a.pidRoll.kdff, 0.01); near(a.pidRoll.slewRateMax, 10);
});

test('attitude: input_euler_angle_roll_pitch_slew_yaw switches at MAX_YAW_ERROR (5 deg)', () => {
  const a = att();
  a.inputEulerAngleRollPitchSlewYaw(0, 0, 3000, 2000); // 30 deg error -> rate mode at +20 deg/s
  assert.ok(a.eulerRateTarget[2] > 0);
  const b = att();
  b.inputEulerAngleRollPitchSlewYaw(0, 0, -400, 2000); // 4 deg -> angle mode, rate toward -4 deg
  assert.ok(b.eulerRateTarget[2] < 0);
});

test('attitude: notch filters follow FILTn_* and ATC_RAT_*_NTF', () => {
  const a = att({ ATC_RAT_RLL_NTF: 1, FILT1_TYPE: 1, FILT1_NOTCH_FREQ: 40, FILT1_NOTCH_Q: 2, FILT1_NOTCH_ATT: 40 });
  a.dt = DT; a.setNotchSampleRate(400);
  assert.ok(a.pidRoll.targetNotch?.initialised);
  assert.equal(filterBankFromParams({}).every((f) => f === null), true);
});

// ---------------- position control ----------------
function posCtl(P = {}) {
  const ahrs = { ...ahrsStub(), positionXYCm: [0, 0], velocityXYCms: [0, 0], positionZUpCm: -100, velocityZUpCms: 0, zAccelCmss: 0, cosYaw: 1, sinYaw: 0 };
  const a = att(P);
  a.ahrs = ahrs;
  const pc = new PosControl(ahrs, a.motors, a); pc.setParams(P);
  return { pc, a, ahrs };
}

test('poscontrol: XY re-entry seeds the integrator from the attitude target, not a stale accel target', () => {
  const { pc } = posCtl({ PSC_VELXY_FF: 0.5 });
  pc.accelTarget = [850, 0, 0]; // left over from an earlier session
  pc.tick = 100; pc.initXYController(); // inactive -> lean_angles_to_accel (level) = 0
  near(pc.accelTarget[0], 0, 1e-9); near(pc.pidVelXY.integrator[0], 0, 1e-9);
  near(pc.pitchTarget, 0); near(pc.rollTarget, 0);
  pc.tick++; pc.accelTarget[0] = 100; pc.velTarget[0] = 0; pc.ahrs.velocityXYCms = [20, 0];
  pc.initXYController(); // active: keeps accel target, subtracts vel*FF
  near(pc.pidVelXY.integrator[0], 100 - 20 * 0.5);
});

test('poscontrol: PSC_ANGLE_MAX and the override bound the lean angle', () => {
  const { pc } = posCtl({ PSC_ANGLE_MAX: 20, ANGLE_MAX: 4500 });
  near(pc.getLeanAngleMaxCd(), 2000);
  pc.setLeanAngleMaxCd(1500); near(pc.getLeanAngleMaxCd(), 1500);
  pc.initXYController(); near(pc.getLeanAngleMaxCd(), 2000); // override cleared on init
  const { pc: p2 } = posCtl({}); near(p2.getLeanAngleMaxCd(), 4500);
});

test('poscontrol: depth targets — climb rate, slewed altitude target, terrain offset', () => {
  const { pc } = posCtl({ PSC_JERK_Z: 50 });
  pc.setMaxSpeedAccelZ(-150, 250, 250); pc.setCorrectionSpeedAccelZ(-150, 250, 250);
  pc.initZController();
  for (let k = 0; k < 400; k++) { pc.tick++; pc.setPosTargetZFromClimbRateCm(50); }
  near(pc.velDesired[2], 50, 1e-3);
  const { pc: q } = posCtl({ PSC_JERK_Z: 50 });
  q.setMaxSpeedAccelZ(-150, 250, 250); q.initZController();
  for (let k = 0; k < 4000; k++) { q.tick++; q.setAltTargetWithSlew(-300); }
  near(q.posTarget[2], -300, 0.5);
  const { pc: r } = posCtl({ PSC_JERK_Z: 50 });
  r.setMaxSpeedAccelZ(-150, 250, 250); r.initZController(); r.setPosOffsetTargetZCm(40);
  for (let k = 0; k < 4000; k++) { r.tick++; r.setPosTargetZFromClimbRateCm(0); }
  near(r.getPosOffsetZCm(), 40, 0.5); near(r.posTarget[2], -60, 0.5);
});

test('poscontrol: setParams is live and the jerk limit follows the attitude limits', () => {
  const { pc } = posCtl({ ATC_ACCEL_R_MAX: 10000, ATC_ACCEL_P_MAX: 10000, PSC_JERK_XY: 5 });
  pc.pidVelXY.setIntegrator([30, 0]); pc.setParams({ PSC_VELXY_P: 2 });
  near(pc.pidVelXY.integrator[0], 30); near(pc.pidVelXY.kp, 2);
  const { pc: q } = posCtl({ ATC_ACCEL_R_MAX: 10000, ATC_ACCEL_P_MAX: 10000, PSC_JERK_XY: 5 });
  q.setMaxSpeedAccelXY(100, 100);
  near(q.jerkMaxXY, 0.5 * Math.sqrt(100 * (100 * Math.PI / 180) * 980.665), 1e-6); // ~207 cm/s^3
});
