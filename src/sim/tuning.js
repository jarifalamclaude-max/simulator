// Step-response tests and simulation-based auto-tuning of ArduSub gains.
// Everything runs on fresh headless Simulation instances, so results are
// reproducible and do not disturb the live session. The tuner only returns
// suggestions: nothing here writes to the caller's configuration.
//
// Method (closest practical port of AC_AutoTune_Multi.cpp, which ArduSub
// itself does not ship):
//   attitude, per axis, inner loop first
//     1. rate twitch, D up / P down until the bounce-back passes AUTOTUNE_AGGR
//        (updating_rate_d_up, :785), last bracket bisected (= RD_DOWN)
//     2. rate twitch, P up / D down until the peak overshoots the target rate
//        by AGGR/2 (updating_rate_p_up_d_down, :895)
//     3. angle twitch, angle P up until overshoot or bounce (updating_angle_p_up,
//        :974), SP backoff 0.9 and ACCEL_MAX from the measured acceleration
//        (set_gains_post_tune, :714); I = P (roll/pitch) or P/10 (yaw)
//   then ATC_INPUT_TC, depth hold (ACCZ -> VELZ -> POSZ cascade) and position
//   hold (VELXY -> POSXY) on the gains accepted so far, every value clamped to
//   the ArduSub @Range, and a final combined verification of all axes.

import { Simulation, MODE } from './simulation.js';
import { RAD, DEG, wrapPI } from '../core/math.js';
import { effectiveWrench } from '../alloc/allocation.js';
import { thrusterLimits } from '../vehicle/config.js';
import { PARAM_INDEX } from '../ardusub/params.js';

export const STEP_AXES = {
  roll: { label: 'Roll', unit: '°', size: 15, band: 0.75 },
  pitch: { label: 'Pitch', unit: '°', size: 15, band: 0.75 },
  yaw: { label: 'Yaw', unit: '°', size: 45, band: 1.0 },
  depth: { label: 'Depth', unit: 'm', size: 0.5, band: 0.02 },
  poshold: { label: 'Position', unit: 'm', size: 0.5, band: 0.03 },
};
const OVERSHOOT_MAX = 10; // % above which a step response is penalised

const yieldToUI = () => new Promise((r) => setTimeout(r, 0));
const round = (v) => +(+v).toPrecision(3);
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

function withParams(cfg, params) {
  return { ...cfg, params: { ...cfg.params, ...params }, sim: { ...cfg.sim } };
}

// ---------------------------------------------------------------------------
// ArduSub 4.5.7 parameter ranges (Sub variants of the @Range metadata)
// ---------------------------------------------------------------------------
const RATE_RANGE = {
  RLL: { P: [0, 0.3], I: [0, 0.5], D: [0, 0.02] }, // AC_AttitudeControl_Sub.cpp:13,20,34
  PIT: { P: [0, 0.3], I: [0, 0.5], D: [0, 0.02] }, // :106,113,127
  YAW: { P: [0, 0.5], I: [0, 0.05], D: [0, 0.02] }, // :199,206,220
};
export const AP_RANGE = {
  ...Object.fromEntries(Object.entries(RATE_RANGE).flatMap(([ax, r]) => [
    [`ATC_RAT_${ax}_P`, r.P], [`ATC_RAT_${ax}_I`, r.I], [`ATC_RAT_${ax}_D`, r.D], [`ATC_RAT_${ax}_IMAX`, [0, 1]], [`ATC_RAT_${ax}_FF`, [0, 0.5]],
    [`ATC_RAT_${ax}_FLTT`, [1, 100]], [`ATC_RAT_${ax}_FLTE`, [1, 100]], [`ATC_RAT_${ax}_FLTD`, [1, 100]]])),
  ATC_ANG_RLL_P: [0, 12], ATC_ANG_PIT_P: [0, 12], ATC_ANG_YAW_P: [0, 6], // AC_AttitudeControl.cpp:86,94,102 (Range{Sub})
  ATC_ACCEL_R_MAX: [0, 180000], ATC_ACCEL_P_MAX: [0, 180000], ATC_ACCEL_Y_MAX: [0, 72000], ATC_INPUT_TC: [0, 1], // :40,57,67,147
  PSC_POSZ_P: [1, 3], PSC_VELZ_P: [1, 8], PSC_VELZ_I: [0.02, 1], PSC_ACCZ_P: [0.2, 1.5], PSC_ACCZ_I: [0, 3], PSC_ACCZ_IMAX: [0, 1000], // AC_PosControl.cpp:91-159
  PSC_POSXY_P: [0.5, 2], PSC_VELXY_P: [0.1, 6], PSC_VELXY_I: [0.02, 1], PSC_VELXY_D: [0, 1], PSC_VELXY_IMAX: [0, 4500], // :237-265
};
/** [min, max] of a parameter: the ArduSub @Range above, else the params.js metadata. */
export function paramRange(name) {
  if (AP_RANGE[name]) return AP_RANGE[name];
  const m = PARAM_INDEX?.[name];
  if (!m) return null;
  if (Array.isArray(m.range) && m.range.length === 2) return m.range;
  if (m.range && Number.isFinite(m.range.min) && Number.isFinite(m.range.max)) return [m.range.min, m.range.max];
  if (Number.isFinite(m.min) && Number.isFinite(m.max)) return [m.min, m.max];
  return null;
}
/** Clamp every value to its range; returns the clamped set and the names that sit on a bound. */
export function clampToRanges(params) {
  const out = {}, atBound = [];
  for (const [k, v0] of Object.entries(params)) {
    const r = paramRange(k);
    let v = round(v0);
    // a zero filter frequency means "off" and is valid outside the 1..100 Hz range
    if (r && !(v === 0 && /_FLT[TED]$/.test(k))) {
      if (v < r[0] || v >= r[1]) atBound.push(k);
      v = clamp(v, r[0], r[1]);
    }
    out[k] = v;
  }
  return { params: out, atBound };
}

// ---------------------------------------------------------------------------
// Test harness
// ---------------------------------------------------------------------------
// noise: true (as configured), false (all off) or 'imu' (gyro/accel noise kept for
// the D term, the scored estimates - AHRS attitude, depth, position - noise-free)
function testConfig(cfg, params, { noise = true, startDepth = 3, positioning = false } = {}) {
  const c = withParams(cfg, params);
  // Tests run in open water (a deep virtual tank, 3 m down) so the result
  // describes the vehicle, not the walls or floor of the test pool.
  c.start = { position: [0, 0, startDepth], yaw: 0 };
  const s = { ...(c.sensors || {}) };
  if (noise === false) Object.assign(s, { gyroNoise: 0, accelNoise: 0, attitudeNoise: 0, depthNoise: 0, positionNoise: 0 });
  else if (noise === 'imu') Object.assign(s, { attitudeNoise: 0, depthNoise: 0, positionNoise: 0 });
  if (positioning) s.positioning = true;
  c.sensors = s;
  c.environment = { ...c.environment, pool: { length: 60, width: 60, depth: 12, floor: 'flat', freeboard: 0, style: 'tiles' }, current: [0, 0, 0], turbulence: { sigma: 0, tau: 2 }, waves: { amplitude: 0, period: 2, direction: 0 } };
  c.hull = { ...(c.hull || {}), floodOverRating: false };
  return c;
}

// depth target in the AC_PosControl frame (inertial-nav altitude, cm up). Uses the
// firmware's test hook when it has one, else drives the Z target directly (the
// depth-hold loop keeps it with the pilot throttle centred).
function setDepthTarget(fw, cmUp) {
  if ('depthTargetOverride' in fw) fw.depthTargetOverride = cmUp;
  else fw.pos?.setPosTargetZCm?.(cmUp);
}

// The controlled variable of each axis, as the controller sees it: AHRS attitude
// (after AHRS_TRIM), inertial-nav depth from the baro (BARO_SPEC_GRAV, sensor
// position and EK3_HGT_DELAY included) and inertial-nav north position.
function measure(sim, axis) {
  const s = sim.sensors;
  if (axis === 'roll') return s.roll * RAD;
  if (axis === 'pitch') return s.pitch * RAD;
  if (axis === 'yaw') return s.yaw * RAD;
  if (axis === 'depth') return -s.positionZUpCm / 100;
  return s.positionXYCm[0] / 100;
}

function session(cfg, geo, params, { axis = 'roll', noise = 'imu', settle = 2, startDepth } = {}) {
  const pos = axis === 'poshold';
  const c = testConfig(cfg, params, { noise, startDepth, positioning: pos });
  const sim = new Simulation(c, geo, { telemetryCapacity: 16 });
  const fw = sim.fw;
  if (!fw.setMode(pos ? MODE.POSHOLD : MODE.ALT_HOLD)) throw new Error(pos ? 'Position hold needs a positioning sensor' : 'Depth hold is not available (no depth sensor)');
  sim.runFor(0.4);
  fw.arm();
  const yaw0 = sim.sensors.yaw * RAD;
  // hold level attitude through the GCS attitude-target path while settling
  const hold = (r, p, y) => fw.setAttitudeTarget(r, p, y);
  hold(0, 0, yaw0);
  sim.runFor(settle);
  return { sim, fw, yaw0, hold };
}

export function stepMetrics(t, y, y0, yf, band) {
  const step = yf - y0;
  const sgn = Math.sign(step) || 1;
  const n = t.length;
  let t10 = NaN, t90 = NaN, peak = -Infinity, settle = 0;
  for (let i = 0; i < n; i++) {
    const prog = (y[i] - y0) / step;
    if (Number.isNaN(t10) && prog >= 0.1) t10 = t[i];
    if (Number.isNaN(t90) && prog >= 0.9) t90 = t[i];
    peak = Math.max(peak, (y[i] - yf) * sgn);
  }
  const tol = Math.max(band, 0.02 * Math.abs(step));
  for (let i = n - 1; i >= 0; i--) if (Math.abs(y[i] - yf) > tol) { settle = t[i]; break; }
  let crossings = 0, prev = 0, reached = false;
  for (let i = 0; i < n; i++) {
    const s = Math.sign(y[i] - yf);
    if (!reached && (y[i] - yf) * sgn >= 0) reached = true;
    if (reached && s !== 0 && prev !== 0 && s !== prev && Math.abs(y[i] - yf) > tol * 0.25) crossings++;
    if (s !== 0) prev = s;
  }
  const tail = y.filter((_, i) => t[i] > t[n - 1] - 1);
  const ss = tail.reduce((a, v) => a + v, 0) / Math.max(1, tail.length) - yf;
  const tail2 = y.filter((_, i) => t[i] > t[n - 1] - 2);
  const rms = Math.sqrt(tail2.reduce((a, v) => a + (v - yf) * (v - yf), 0) / Math.max(1, tail2.length));
  return {
    tailRms: rms,
    riseTime: Number.isNaN(t10) || Number.isNaN(t90) ? NaN : t90 - t10,
    overshoot: Math.max(0, (peak / Math.abs(step)) * 100),
    settlingTime: settle,
    steadyError: ss,
    oscillations: crossings,
    reached: !Number.isNaN(t90),
  };
}

/**
 * Run one step test on a fresh simulation. The step is commanded and scored in
 * the variable the controller regulates (see measure()).
 * opts: { axis, size, settle, duration, params, noise (true | false | 'imu'), startDepth }
 */
export function runStepTest(cfg, geo, opts = {}) {
  const axis = opts.axis ?? 'yaw';
  const meta = STEP_AXES[axis];
  if (!meta) throw new Error(`Unknown step axis ${axis}`);
  const size = opts.size ?? meta.size;
  const { sim, fw, yaw0, hold } = session(cfg, geo, opts.params || {}, { axis, noise: opts.noise ?? true, settle: opts.settle ?? 3, startDepth: opts.startDepth });
  const y0 = measure(sim, axis);
  const yf = axis === 'roll' || axis === 'pitch' ? size : y0 + size;
  const command = () => {
    if (axis === 'depth') setDepthTarget(fw, -yf * 100);
    else if (axis === 'poshold') fw.pos.posTarget[0] = yf * 100;
    else hold(axis === 'roll' ? size : 0, axis === 'pitch' ? size : 0, axis === 'yaw' ? wrapPI((y0 + size) * DEG) * RAD : yaw0);
  };
  const dur = opts.duration ?? (axis === 'yaw' || axis === 'poshold' ? 7 : 6);
  const n = Math.round(dur / sim.dtFw);
  const t = new Float32Array(Math.ceil(n / 2)), y = new Float32Array(t.length), sat = new Uint8Array(t.length), tgt = new Float32Array(t.length);
  let k = 0;
  let unwrap = 0, last = y0;
  for (let i = 0; i < n; i++) {
    if (i === 0 || axis === 'poshold' || (axis === 'depth' && !('depthTargetOverride' in fw)) || i % 400 === 0) command();
    sim.loop();
    if (i % 2 === 0 && k < t.length) {
      let v = measure(sim, axis);
      if (axis === 'yaw') { // unwrap across +-180
        const d = v + unwrap - last;
        if (d > 180) unwrap -= 360; else if (d < -180) unwrap += 360;
        v += unwrap; last = v;
      }
      t[k] = i * sim.dtFw; y[k] = v; sat[k] = fw.motors.saturated ? 1 : 0;
      tgt[k] = axis === 'depth' ? -fw.pos.posTarget[2] / 100 : yf;
      k++;
    }
  }
  const metrics = stepMetrics(t, y, y0, yf, meta.band);
  metrics.saturation = sat.reduce((a, v) => a + v, 0) / sat.length;
  metrics.diverged = isDiverged(t, y, sat, yf, size, axis);
  return { axis, size, y0, yf, t, y, target: tgt, metrics, unit: meta.unit };
}

// Divergence from the error the signal can actually reach (Euler pitch is bounded
// at +-90 deg), or sustained saturation with a large error over the last second.
function isDiverged(t, y, sat, yf, size, axis) {
  if (!y.every(Number.isFinite)) return true;
  let maxErr = 0;
  for (const v of y) maxErr = Math.max(maxErr, Math.abs(v - yf));
  const lim = Math.abs(size) * 3 + (axis === 'depth' || axis === 'poshold' ? 1 : 20);
  if (maxErr > lim) return true;
  if (axis === 'pitch' && y.some((v) => Math.abs(v) > 80)) return true;
  const n = t.length, tEnd = t[n - 1];
  let ns = 0, nt = 0, err = 0;
  for (let i = 0; i < n; i++) if (t[i] > tEnd - 1) { nt++; ns += sat[i]; err += Math.abs(y[i] - yf); }
  return nt > 0 && ns / nt > 0.5 && err / nt > 0.5 * Math.abs(size);
}

export function stepCost(m, axis) {
  if (m.diverged || !m.reached) return 1e3 + (m.diverged ? 1e3 : 0);
  const band = STEP_AXES[axis].band;
  return 0.5 * m.settlingTime + 0.05 * m.overshoot + 0.1 * Math.max(0, m.overshoot - OVERSHOOT_MAX) + 0.4 * m.oscillations + 3 * m.saturation
    + 0.6 * (Number.isFinite(m.riseTime) ? m.riseTime : 3) + 2 * (m.tailRms / band);
}

// full step plus a half step back: the score every candidate is compared on
function evaluate(cfg, geo, axis, params, noise = 'imu') {
  const size = STEP_AXES[axis].size;
  const a = runStepTest(cfg, geo, { axis, params, size, noise });
  const b = runStepTest(cfg, geo, { axis, params, size: -size / 2, noise });
  return { r: a, cost: 0.6 * stepCost(a.metrics, axis) + 0.4 * stepCost(b.metrics, axis) };
}

// ---------------------------------------------------------------------------
// Twitch tests (AC_AutoTune_Multi::twitching_test_rate / twitching_test_angle)
// ---------------------------------------------------------------------------
const AXI = { roll: 0, pitch: 1, yaw: 2 };
const AX = { roll: 'RLL', pitch: 'PIT', yaw: 'YAW' };
const AGGR = 0.1;                 // AUTOTUNE_AGGR default (AC_AutoTune_Multi.cpp:98)
const D_UP_DOWN_MARGIN = 0.2;     // :66
const PI_RATIO_TESTING = 0.1, PI_RATIO_FINAL = 1.0, YAW_PI_RATIO_FINAL = 0.1; // :52-54
const RD_MIN = 0.001, RP_MIN = 0.01, SP_MIN = 0.5, RLPF_MIN = 1, RLPF_MAX = 5, FLTE_MIN = 2.5; // :56-62, AUTOTUNE_MIN_D
const SP_BACKOFF = 0.9;           // :69 (RD/RP backoff are 1.0)
const ACCEL_RP_MIN = 4000, ACCEL_Y_MIN = 1000; // cdeg/s^2, :63-64
const TEST_TIMEOUT = 1.0;         // AUTOTUNE_TESTING_STEP_TIMEOUT_MS
// The search step. AutoTune moves 5% per twitch and needs several agreeing
// twitches (noise); the simulation is deterministic, so it takes larger steps
// and bisects the last bracket.
const STEP_RATIO = 0.15;

// twitch test conditions (AC_AutoTune_Multi::load_twitch_gains / backup): rate
// I at P/10, no target filter, no input shaping and no sqrt angle controller
function twitchParams(axis, g) {
  const A = AX[axis];
  const p = { ATC_ACCEL_R_MAX: 0, ATC_ACCEL_P_MAX: 0, ATC_ACCEL_Y_MAX: 0, ATC_RATE_FF_ENAB: 0, [`ATC_RAT_${A}_FLTT`]: 0 };
  p[`ATC_RAT_${A}_P`] = g.p; p[`ATC_RAT_${A}_I`] = g.p * PI_RATIO_TESTING;
  if (axis === 'yaw') p.ATC_RAT_YAW_FLTE = g.d; else p[`ATC_RAT_${A}_D`] = g.d;
  if (g.sp !== undefined) p[`ATC_ANG_${A}_P`] = g.sp;
  return p;
}

const lpfAlpha = (dt, hz) => (hz > 0 ? dt / (dt + 1 / (2 * Math.PI * hz)) : 1);

/**
 * Body-rate twitch: rate target step on one axis straight into the rate
 * controller (input_rate_step_bf_roll_pitch_yaw), the others held level.
 * Returns the AutoTune measurements: rate max, the minimum after it
 * (bounce-back), the maximum acceleration, the 10-90% rise time.
 */
export function rateTwitch(cfg, geo, axis, params, { rate, stopAt = rate, dir = 1, angleLimit = axis === 'yaw' ? 30 : 20 } = {}) {
  const { sim, fw } = session(cfg, geo, params, { settle: 1.5 });
  const k = AXI[axis], dt = sim.dtFw;
  const P = { ...cfg.params, ...params };
  // rotation_rate_filt: 2 x FLTD for roll/pitch, AUTOTUNE_Y_FILT_FREQ for yaw (:1107-1125)
  const af = lpfAlpha(dt, axis === 'yaw' ? 10 : 2 * (P[`ATC_RAT_${AX[axis]}_FLTD`] || 20));
  const ang0 = [sim.sensors.roll, sim.sensors.pitch, sim.sensors.yaw][k];
  let rf = 0, prev = 0, rMax = 0, rMin = 0, iPk = 0, accMax = 0, t = 0, limit = TEST_TIMEOUT, sat = 0, hitAngle = false;
  const hist = [];
  while (t < limit) {
    fw.att.angVelBody[k] = rate * dir; // rate_bf_*_target override (twitch_test_run)
    sim.loop(); t += dt;
    const ang = wrapPI([sim.sensors.roll, sim.sensors.pitch, sim.sensors.yaw][k] - ang0) * dir;
    rf += af * (sim.sensors.gyro[k] * dir - rf);
    accMax = Math.max(accMax, (rf - prev) / dt); prev = rf;
    hist.push(rf);
    // twitching_test_rate (:501)
    if (rf > rMax) { rMax = rf; rMin = rf; iPk = hist.length - 1; }
    if (rf < rMin && rMax > stopAt * 0.5) rMin = rf;
    if (rMax < stopAt * 0.75) limit = Math.min(TEST_TIMEOUT, t * 3);
    if (fw.motors.saturated) sat++;
    if (rMax > stopAt) break;
    // twitching_abort_rate (:543): angle limit reached while the rate is still rising
    if (ang * RAD >= angleLimit) { hitAngle = rf === rMax; break; }
  }
  // 10-90% rise time of the measured peak (closed-loop rate bandwidth)
  const i10 = hist.findIndex((v) => v >= 0.1 * rMax), i90 = hist.findIndex((v) => v >= 0.9 * rMax);
  const rise = i10 >= 0 && i90 > i10 ? i90 - i10 : 20;
  // bounce-back within two rise times of the peak (the twitch is not stopped on
  // bounce so that a reference twitch can be compared over the same window)
  let low = rMax;
  for (let i = iPk; i < Math.min(hist.length, iPk + Math.max(20, 2 * rise)); i++) low = Math.min(low, hist[i]);
  return { rateMax: rMax, rateMin: rMin, bounce: rMax - low, accelMax: accMax, riseTime: i10 >= 0 && i90 > i10 ? rise * dt : NaN, saturation: sat / Math.max(1, hist.length), time: t, hitAngle };
}

/** Angle twitch: angle target step without shaping (input_angle_step_bf_roll_pitch_yaw). */
export function angleTwitch(cfg, geo, axis, params, { angle, stopAt = angle * (1 + 0.5 * AGGR), dir = 1 } = {}) {
  const { sim, fw, yaw0, hold } = session(cfg, geo, params, { settle: 1.5 });
  const k = AXI[axis], dt = sim.dtFw;
  const P = { ...cfg.params, ...params };
  const af = lpfAlpha(dt, axis === 'yaw' ? 10 : 2 * (P[`ATC_RAT_${AX[axis]}_FLTD`] || 20));
  const get = () => [sim.sensors.roll, sim.sensors.pitch, sim.sensors.yaw][k] * RAD;
  const a0 = get();
  const tgt = (axis === 'yaw' ? yaw0 : a0) + angle * dir;
  const cmd = () => hold(axis === 'roll' ? tgt : 0, axis === 'pitch' ? tgt : 0, axis === 'yaw' ? wrapPI(tgt * DEG) * RAD : yaw0);
  cmd();
  let rf = sim.sensors.gyro[k] * dir * RAD, prev = rf, aMax = 0, aMin = 0, rMax = 0, rMin = 0, accMax = 0, t = 0, limit = TEST_TIMEOUT, i = 0;
  while (t < limit) {
    sim.loop(); t += dt;
    if (++i % 400 === 0) cmd();
    rf += af * (sim.sensors.gyro[k] * dir * RAD - rf);
    accMax = Math.max(accMax, (rf - prev) / dt); prev = rf;
    const a = wrapPI((get() - a0) * DEG) * RAD * dir;
    if (a > aMax) { aMax = a; aMin = a; }
    // twitching_test_angle (:560)
    if (a < aMin && aMax > stopAt * 0.5) aMin = a;
    if (rf > rMax) { rMax = rf; rMin = rf; }
    if (rf < rMin) rMin = rf;
    if (aMax < stopAt * 0.75) limit = Math.min(TEST_TIMEOUT, t * 3);
    if (aMax > stopAt || aMax - aMin > aMax * AGGR) break;
  }
  return { angleMax: aMax, angleMin: aMin, rateMax: rMax, rateMin: rMin, accelMax: accMax * 100, time: t }; // accel in cdeg/s^2
}

// AC_AttitudeControl::max_rate_step_bf_roll (:1076): the rate step that
// saturates the rate controller output, limited to the AutoTune twitch range
function twitchRate(axis, P, g, dt) {
  const A = AX[axis];
  const alpha = Math.min(lpfAlpha(dt, axis === 'yaw' ? g.d : P[`ATC_RAT_${A}_FLTE`] || 0), lpfAlpha(dt, P[`ATC_RAT_${A}_FLTD`] || 0));
  const ar = 1 - alpha;
  const kd = axis === 'yaw' ? P.ATC_RAT_YAW_D || 0 : g.d;
  let r = 1 / ((ar * ar * ar * alpha * kd) / dt + g.p);
  const maxCfg = { roll: P.ATC_RATE_R_MAX, pitch: P.ATC_RATE_P_MAX, yaw: P.ATC_RATE_Y_MAX }[axis];
  if (maxCfg > 0) r = Math.min(r, maxCfg * DEG);
  // AUTOTUNE_TARGET_(MIN_)RATE_RLLPIT_CDS 4500..18000, YAW 1500..9000
  return axis === 'yaw' ? clamp(r, 15 * DEG, 90 * DEG) : clamp(r, 45 * DEG, 180 * DEG);
}

/**
 * ArduPilot-style twitch tune of one attitude axis on top of `work`.
 * Returns the suggested rate/angle gains plus a log of every twitch.
 */
async function twitchTuneAxis(cfg, geo, axis, work, est, { tick, maxIter = 24 }) {
  const A = AX[axis], isYaw = axis === 'yaw';
  const P0 = { ...cfg.params, ...work };
  const dt = 1 / Math.min(1000, Math.max(50, P0.SCHED_LOOP_RATE || 400));
  const rP = paramRange(`ATC_RAT_${A}_P`) ?? [0, 2];
  const pMax = rP[1], pMin = Math.max(RP_MIN, rP[0]);
  // for yaw AutoTune tunes the error filter (rLPF) in place of D (:633-671)
  const dMin = isYaw ? RLPF_MIN : RD_MIN, dMax = isYaw ? RLPF_MAX : (paramRange(`ATC_RAT_${A}_D`) ?? [0, 0.2])[1];
  const g = { p: clamp(P0[`ATC_RAT_${A}_P`] ?? 0.135, pMin, pMax), d: isYaw ? clamp(P0.ATC_RAT_YAW_FLTE || FLTE_MIN, dMin, dMax) : clamp(Math.max(P0[`ATC_RAT_${A}_D`] ?? 0, dMin), dMin, dMax) };
  const log = [];
  const e = est[axis];
  // Twitch rate: max_rate_step (the step that saturates the rate controller) as
  // AutoTune, times AutoTune's step_scaler, which shrinks it by 10% whenever the
  // angle limit comes before the peak (twitching_abort_rate). An ROV is far
  // slower than a multicopter, so the scaler starts where the rate loop (time
  // constant J / (K P) plus the motor lag) peaks, after about three time
  // constants, at 70% of that angle, and may go down to 0.1 instead of 0.5.
  const angleLimit = isYaw ? 30 : 20;
  const tauOf = (gg) => e.J / Math.max(1e-6, e.K * gg.p) + (est.motorTau || 0.05);
  const scale0 = (gg) => clamp((0.7 * angleLimit * DEG) / (3 * tauOf(gg)) / twitchRate(axis, P0, gg, dt), 0.1, 1);
  let scaler = scale0(g);
  const rateOf = (gg = g) => twitchRate(axis, P0, gg, dt) * (isYaw ? 0.75 : 1) * scaler;
  const twitch = (gg, stage, stopRatio, target) => {
    const r = rateTwitch(cfg, geo, axis, { ...work, ...twitchParams(axis, gg) }, { rate: target, stopAt: target * stopRatio, angleLimit });
    log.push({ stage, p: gg.p, d: gg.d, target, scaler, ...r });
    return { ...r, target };
  };
  // An ROV's rate also sags after the peak for reasons a multicopter does not
  // have: the hydrostatic righting moment grows with the angle and the thrusters
  // lose gain near zero thrust. The same twitch at the minimum D (same P, same
  // target) measures that sag, and only the bounce-back beyond it counts as D.
  const refs = new Map();
  const rate = (gg, stage, stopRatio = 1) => {
    let r;
    for (let k = 0; k < 12; k++) {
      r = twitch(gg, stage, stopRatio, rateOf(gg));
      if (!r.hitAngle || scaler <= 0.1) break;
      scaler = Math.max(0.1, scaler * 0.9);
    }
    const key = `${gg.p}|${r.target}|${stopRatio}`;
    if (!refs.has(key)) refs.set(key, gg.d <= dMin * 1.0001 ? r : twitch({ ...gg, d: dMin }, 'REF', stopRatio, r.target));
    r.sag = refs.get(key).bounce;
    return r;
  };
  const bounce = (r) => r.bounce - r.sag > r.rateMax * AGGR;

  // 1. D up / P down (updating_rate_d_up): keep the peak within 80-100% of the
  // target with P, raise D until the bounce-back exceeds AGGR
  let dGood = null, dBad = null;
  for (let it = 0; it < maxIter; it++) {
    await tick('rate D');
    const r = rate(g, 'RD_UP'), target = r.target;
    if (r.rateMax > target) {
      g.p *= 1 - STEP_RATIO;
      if (g.p < pMin) { g.p = pMin; g.d *= 1 - STEP_RATIO; if (g.d <= dMin) { g.d = dMin; break; } }
    } else if (r.rateMax < target * (1 - D_UP_DOWN_MARGIN) && g.p < pMax) {
      g.p = Math.min(pMax, g.p * (1 + STEP_RATIO));
    } else if (bounce(r)) { dBad = g.d; break; }
    else {
      dGood = g.d;
      if (g.d >= dMax) break;
      g.d = Math.min(dMax, g.d * (1 + 2 * STEP_RATIO));
    }
  }
  // RD_DOWN: the largest D without bounce-back, by bisection of the last bracket
  if (dBad !== null) {
    let lo = dGood ?? dMin, hi = dBad;
    for (let it = 0; it < 3 && hi / lo > 1.04; it++) {
      await tick('rate D');
      const m = Math.sqrt(lo * hi);
      if (bounce(rate({ ...g, d: m }, 'RD_DOWN'))) hi = m; else lo = m;
    }
    g.d = lo;
  }

  // 2. P up / D down (updating_rate_p_up_d_down): raise P until the peak passes
  // the target by AGGR/2; bounce-back below the target lowers D and P together
  let pGood = null, pBad = null;
  const overP = (r) => r.rateMax > r.target * (1 + 0.5 * AGGR);
  for (let it = 0; it < maxIter; it++) {
    await tick('rate P');
    const r = rate(g, 'RP_UP', 1 + 0.5 * AGGR), target = r.target;
    if (overP(r)) { pBad = g.p; break; }
    if (r.rateMax < target && r.rateMax > target * (1 - D_UP_DOWN_MARGIN) && bounce(r) && g.d > dMin) {
      g.d = Math.max(dMin, g.d * (1 - STEP_RATIO));
      g.p = Math.max(pMin, g.p * (1 - STEP_RATIO));
      continue;
    }
    pGood = g.p;
    if (g.p >= pMax) break;
    g.p = Math.min(pMax, g.p * (1 + STEP_RATIO));
  }
  if (pBad !== null) {
    let lo = pGood ?? pMin, hi = pBad;
    for (let it = 0; it < 3 && hi / lo > 1.04; it++) {
      await tick('rate P');
      const m = Math.sqrt(lo * hi);
      if (overP(rate({ ...g, p: m }, 'RP_UP', 1 + 0.5 * AGGR))) hi = m; else lo = m;
    }
    g.p = lo;
  }
  // closed-loop rate bandwidth from the last accepted twitch (for the filter check)
  const fin = rate(g, 'CHECK');
  const bwHz = Number.isFinite(fin.riseTime) && fin.riseTime > 0 ? 0.35 / fin.riseTime : NaN;

  // 3. angle P up (updating_angle_p_up) with the tuned rate loop
  const rS = paramRange(`ATC_ANG_${A}_P`) ?? [0, 12];
  const spMax = Math.min(40, rS[1]);
  const angle = isYaw ? 30 : 20; // AUTOTUNE_TARGET_ANGLE_RLLPIT_CD 2000 / _YAW_CD 3000
  g.sp = clamp(P0[`ATC_ANG_${A}_P`] ?? 6, SP_MIN, spMax);
  let spGood = null, spBad = null, accel = 0;
  const angleT = (sp) => angleTwitch(cfg, geo, axis, { ...work, ...twitchParams(axis, { ...g, sp }) }, { angle });
  const over = (r) => r.angleMax > angle * (1 + 0.5 * AGGR) || (r.angleMax > angle && r.rateMin < -r.rateMax * AGGR);
  // SP_DOWN (updating_angle_p_down) while the start value overshoots, then SP_UP
  for (let it = 0; it < maxIter; it++) {
    await tick('angle P');
    const r = angleT(g.sp);
    log.push({ stage: spGood === null && it === 0 ? 'SP' : spBad !== null && spGood === null ? 'SP_DOWN' : 'SP_UP', sp: g.sp, ...r });
    if (over(r)) {
      if (spGood !== null) { spBad = g.sp; break; }
      spBad = g.sp;
      if (g.sp <= SP_MIN) break;
      g.sp = Math.max(SP_MIN, g.sp * (1 - STEP_RATIO));
      continue;
    }
    spGood = g.sp; accel = r.accelMax;
    if (spBad !== null || g.sp >= spMax) break; // came down from an overshoot: done
    g.sp = Math.min(spMax, g.sp * (1 + STEP_RATIO));
  }
  if (spBad !== null && spGood !== null) {
    let lo = spGood, hi = spBad;
    for (let it = 0; it < 2 && hi / lo > 1.04; it++) {
      await tick('angle P');
      const m = Math.sqrt(lo * hi);
      const r = angleT(m);
      log.push({ stage: 'SP_UP', sp: m, ...r });
      if (over(r)) hi = m; else { lo = m; accel = r.accelMax; }
    }
    spGood = lo;
  }
  const sp = Math.max(SP_MIN, (spGood ?? SP_MIN) * SP_BACKOFF);
  const accelMax = Math.max(isYaw ? ACCEL_Y_MIN : ACCEL_RP_MIN, accel);
  return { p: g.p, d: g.d, sp, accelMax, bwHz, log, reachedPMax: g.p >= pMax, reachedSpMax: (spGood ?? 0) >= spMax };
}

// final parameter set of one attitude axis (AC_AutoTune_Multi::save_tuning_gains)
function attitudeSet(axis, tw, P0, margin) {
  const A = AX[axis], isYaw = axis === 'yaw';
  const p = tw.p * margin;
  const out = {
    [`ATC_RAT_${A}_P`]: p,
    [`ATC_RAT_${A}_I`]: p * (isYaw ? YAW_PI_RATIO_FINAL : PI_RATIO_FINAL),
    [`ATC_RAT_${A}_D`]: isYaw ? P0.ATC_RAT_YAW_D ?? 0 : tw.d * margin,
    [`ATC_ANG_${A}_P`]: tw.sp,
    [{ roll: 'ATC_ACCEL_R_MAX', pitch: 'ATC_ACCEL_P_MAX', yaw: 'ATC_ACCEL_Y_MAX' }[axis]]: Math.round(tw.accelMax / 100) * 100,
  };
  if (isYaw) out.ATC_RAT_YAW_FLTE = tw.d;
  const notes = [];
  // filters relative to the tuned bandwidth: a target/derivative filter closer
  // than about 4x the closed-loop rate bandwidth costs phase margin
  if (Number.isFinite(tw.bwHz)) {
    const need = Math.min(100, Math.ceil(4 * tw.bwHz));
    for (const f of isYaw ? ['FLTT'] : ['FLTT', 'FLTD']) {
      const n = `ATC_RAT_${A}_${f}`, cur = P0[n];
      if (cur > 0 && cur < need) { out[n] = need; notes.push(`${n} ${cur} Hz is below 4x the tuned rate bandwidth (${tw.bwHz.toFixed(1)} Hz): raised to ${need} Hz`); }
    }
  }
  const imax = `ATC_RAT_${A}_IMAX`, ir = paramRange(imax);
  if (ir && (P0[imax] < ir[0] || P0[imax] > ir[1])) out[imax] = clamp(P0[imax], ir[0], ir[1]);
  if (P0[`ATC_RAT_${A}_FF`]) notes.push(`ATC_RAT_${A}_FF is not used by the ArduSub 4.5 rate loop (AC_AttitudeControl_Sub.cpp:427 drops get_ff()), so it is left as it is`);
  if (tw.reachedPMax) notes.push(`rate P reached the top of its range (${paramRange(`ATC_RAT_${A}_P`)?.[1]}) before the response overshot: the vehicle has little ${axis} authority`);
  if (tw.reachedSpMax) notes.push(`angle P reached the top of its range before overshooting`);
  return { params: out, notes };
}

// ---------------------------------------------------------------------------
// Plant estimates
// ---------------------------------------------------------------------------
export function plantAxisEstimates(sim) {
  const m = sim.model;
  const th = sim.cfg.thrusters.map((t) => ({ ...t, ...thrusterLimits(t) }));
  const signs = sim.cfg.thrusters.map((t, i) => (t.polarity ?? 1) * ((sim.cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1) < 0 ? -1 : 1));
  const W = effectiveWrench(th, sim.mixer.factors, sim.mixer.cg, signs);
  const tau = Math.max(...sim.cfg.thrusters.map((t) => t.tauUp ?? 0.08));
  const dq = (i) => Math.abs(m.DQ?.[i]?.[i] ?? 0), dl = (i) => Math.abs(m.DL?.[i]?.[i] ?? 0);
  // hydrostatic righting stiffness (N m / rad) from the small-angle period T = 2 pi sqrt(J / kh)
  const kh = (J, T) => (Number.isFinite(T) && T > 0 ? J * (2 * Math.PI / T) ** 2 : 0);
  const Jr = m.Ig[0] + m.MA[3][3], Jp = m.Ig[4] + m.MA[4][4];
  return {
    roll: { J: Jr, K: Math.abs(W[0][3]), DQ: dq(3), DL: dl(3), kh: kh(Jr, sim.info?.rollPeriod) },
    pitch: { J: Jp, K: Math.abs(W[1][4]), DQ: dq(4), DL: dl(4), kh: kh(Jp, sim.info?.pitchPeriod) },
    yaw: { J: m.Ig[8] + m.MA[5][5], K: Math.abs(W[2][5]), DQ: dq(5), DL: dl(5) },
    heave: { M: m.mass + m.MA[2][2], K: Math.abs(W[3][2]) },
    surge: { M: m.mass + m.MA[0][0], K: Math.abs(W[4]?.[0] ?? 0) },
    motorTau: tau,
  };
}

// depth hold cascade at bandwidth scale s: ACCZ (inner) -> VELZ -> POSZ, each loop
// about 3x slower than the one inside it; I:P as in the ArduSub defaults (0.1:0.5)
function depthGains(est, s, P0) {
  const { M, K } = est.heave;
  if (!(K > 1e-6)) return null;
  const accP = (5 * M) / K; // throttle (0..1) -> bidirectional x2 -> force K; accel loop gain ~ unity at s=1
  const ratio = P0.PSC_ACCZ_P > 0 ? clamp((P0.PSC_ACCZ_I ?? 0) / P0.PSC_ACCZ_P, 0.2, 2) : 0.2;
  return { PSC_ACCZ_P: accP * s, PSC_ACCZ_I: accP * s * ratio, PSC_VELZ_P: 3 * s, PSC_POSZ_P: 1.5 * s };
}
// position hold cascade: VELXY (PI, I:P 0.5 as POSCONTROL_VEL_XY_I/P) -> POSXY
function posGains(est, s) {
  if (!(est.surge.K > 1e-6)) return null;
  return { PSC_VELXY_P: 1.5 * s, PSC_VELXY_I: 0.75 * s, PSC_POSXY_P: 0.5 * s };
}

// 1-D search over a cascade bandwidth scale: grid, golden-section refinement and
// a P-down robustness margin. Candidates outside the @Range are clamped; a grid
// point that clamps to an already-tested set is skipped.
async function scaleSearch(cfg, geo, axis, base, make, grid, { quick, margin, tick }) {
  const seen = new Map();
  const ev = (s) => {
    const { params, atBound } = clampToRanges(make(s));
    const key = JSON.stringify(params);
    if (!seen.has(key)) seen.set(key, { s, params, atBound, ...evaluate(cfg, geo, axis, { ...base, ...params }) });
    return seen.get(key);
  };
  let best = null;
  for (const s of grid) {
    await tick(`trial ${grid.indexOf(s) + 1}`);
    const e = ev(s);
    if (!best || e.cost < best.cost) best = e;
  }
  const idx = grid.indexOf(best.s);
  let lo = grid[Math.max(0, idx - 1)], hi = grid[Math.min(grid.length - 1, idx + 1)];
  for (let it = 0; it < (quick ? 1 : 3); it++) {
    await tick('refining');
    const a = lo + 0.382 * (hi - lo), b = lo + 0.618 * (hi - lo);
    const ea = ev(a), eb = ev(b);
    if (ea.cost < best.cost) best = ea;
    if (eb.cost < best.cost) best = eb;
    if (ea.cost < eb.cost) hi = b; else lo = a;
  }
  // back off for robustness unless that costs too much performance
  await tick('robustness margin');
  const backed = ev(best.s * margin);
  return backed.cost <= best.cost * 1.1 ? backed : best;
}

/**
 * Auto-tune all controllable axes. Returns { params, report, estimates, verification, notes }.
 * Suggestions only: `params` holds the changes worth making, nothing is applied.
 * Every candidate is scored on a full step and a half step the other way, the
 * current gains compete as a candidate, and an axis keeps its current gains
 * unless the tuned ones are clearly better with all the other tuned gains in place.
 * onProgress(fraction, text)
 */
export async function autoTune(cfg, geo, { axes = ['roll', 'pitch', 'yaw', 'depth'], onProgress = () => {}, margin = 0.85, quick = false } = {}) {
  const base = new Simulation(cfg, geo, { telemetryCapacity: 16 });
  const est = plantAxisEstimates(base);
  const controllable = base.mixer.analysis.controllable;
  const P0 = { ...cfg.params };
  const report = {}, notes = [];
  const work = {};          // gains accepted so far (later axes are tuned on top of them)
  const total = axes.length + 1;
  let done = 0;
  const stage = (axis) => {
    let k = 0;
    const steps = axis === 'depth' || axis === 'poshold' ? (quick ? 7 : 12) : quick ? 30 : 60;
    return async (txt) => { onProgress(Math.min(0.999, (done + Math.min(0.95, k++ / steps)) / total), `${STEP_AXES[axis].label}: ${txt}`); await yieldToUI(); };
  };
  const att = axes.filter((a) => AXI[a] !== undefined);
  const results = {};
  // ---- attitude: inner loops first, each axis on top of the ones before ----
  for (const axis of att) {
    if (!controllable.includes(axis) && cfg.mixer.source === 'auto') { report[axis] = { skipped: 'axis not controllable with this thruster layout' }; done++; continue; }
    if (!(est[axis].K > 1e-6)) { report[axis] = { skipped: 'no control authority on this axis' }; done++; continue; }
    const tick = stage(axis);
    const tw = await twitchTuneAxis(cfg, geo, axis, work, est, { tick, maxIter: quick ? 10 : 24 });
    const set = attitudeSet(axis, tw, { ...P0, ...work }, margin);
    const { params, atBound } = clampToRanges(set.params);
    results[axis] = { params, atBound, notes: set.notes, twitch: tw.log, bandwidthHz: tw.bwHz };
    Object.assign(work, params);
    done++;
  }
  // ---- input shaping: crispest ATC_INPUT_TC whose shaped steps stay within the overshoot limit ----
  if (att.some((a) => results[a])) {
    const tick = stage(att.find((a) => results[a]));
    // ATC_INPUT_TC sets the feel (AutoTune leaves it alone): the current value
    // stays when the tuned loops follow it within the limits, else the crispest
    // value that does is suggested
    const cur = P0.ATC_INPUT_TC ?? 0.15;
    const cands = [cur, ...(quick ? [0.1, 0.2, 0.3, 0.5] : [0.08, 0.1, 0.12, 0.15, 0.2, 0.25, 0.3, 0.4, 0.5]).filter((v) => Math.abs(v - cur) > 1e-6)];
    let pick = null;
    for (const tc of cands) {
      await tick(`input time constant ${tc} s`);
      const rs = att.filter((a) => results[a]).map((a) => runStepTest(cfg, geo, { axis: a, params: { ...work, ATC_INPUT_TC: tc }, noise: 'imu' }));
      if (rs.every((r) => !r.metrics.diverged && r.metrics.reached && r.metrics.overshoot <= OVERSHOOT_MAX && r.metrics.oscillations <= 1)) { pick = tc; break; }
    }
    if (pick === null) notes.push(`No ATC_INPUT_TC between 0.08 and 0.5 s kept every shaped attitude step within ${OVERSHOOT_MAX}% overshoot; it is left as it is.`);
    else if (pick !== cur) { work.ATC_INPUT_TC = pick; notes.push(`ATC_INPUT_TC ${cur} s let the tuned attitude loops overshoot by more than ${OVERSHOOT_MAX}%: ${pick} s is the crispest value that does not.`); }
    for (const a of att) if (results[a] && work.ATC_INPUT_TC !== undefined) results[a].params.ATC_INPUT_TC = work.ATC_INPUT_TC;
  }
  // ---- depth hold and position hold on the tuned attitude loops ----
  const cascade = [
    ['depth', (s) => depthGains(est, s, P0), quick ? [0.6, 1, 1.6] : [0.5, 0.8, 1.2, 1.8, 2.6]],
    ['poshold', (s) => posGains(est, s), quick ? [0.6, 1.2, 2] : [0.4, 0.7, 1, 1.5, 2.2, 3]],
  ];
  for (const [axis, make, grid] of cascade) {
    if (!axes.includes(axis)) continue;
    if (axis === 'poshold' && !(cfg.sensors?.positioning)) { report[axis] = { skipped: 'position hold needs a positioning sensor (Vehicle > Sensors)' }; done++; continue; }
    if (!make(1)) { report[axis] = { skipped: 'no control authority on this axis' }; done++; continue; }
    const tick = stage(axis);
    const best = await scaleSearch(cfg, geo, axis, work, make, grid, { quick, margin, tick });
    const params = { ...best.params };
    const n = [];
    if (axis === 'depth') {
      // PSC_VELZ_I (ArduSub default 0): only when it clearly helps with the rest in place
      await tick('velocity I');
      const vi = clampToRanges({ PSC_VELZ_I: 0.1 * params.PSC_VELZ_P }).params;
      const e = evaluate(cfg, geo, axis, { ...work, ...params, ...vi });
      if (e.cost < best.cost * 0.95) Object.assign(params, vi); else n.push('PSC_VELZ_I left as it is (no clear gain from an integrator on the velocity loop)');
      const im = 'PSC_ACCZ_IMAX', ir = paramRange(im);
      if (ir && (P0[im] < ir[0] || P0[im] > ir[1])) params[im] = clamp(P0[im], ir[0], ir[1]);
    }
    results[axis] = { params, atBound: best.atBound, notes: n, bandwidth: best.s };
    Object.assign(work, params);
    done++;
  }
  // ---- combined verification: every axis with the whole tuned set against the current gains ----
  const tickV = async (txt) => { onProgress(Math.min(0.999, (done + 0.5) / total), `Verifying: ${txt}`); await yieldToUI(); };
  const verification = {};
  const tuned = Object.keys(results);
  const finalSet = { ...work };
  // a name tuned by several axes (ATC_INPUT_TC) stays while any of them uses it
  const dropGroup = (axis) => { for (const k of Object.keys(results[axis].params)) if (!tuned.some((b) => b !== axis && !verification[b]?.keep && k in results[b].params)) delete finalSet[k]; };
  for (const axis of tuned) {
    await tickV(STEP_AXES[axis].label);
    const before = evaluate(cfg, geo, axis, {});
    const after = evaluate(cfg, geo, axis, finalSet);
    verification[axis] = { before, after, keep: before.cost <= after.cost * 1.03 };
  }
  // an axis that is better off with its current gains drops its group; the
  // others are then checked once more without it (they were tuned on top of it)
  const kept = tuned.filter((a) => verification[a].keep);
  if (kept.length) {
    kept.forEach(dropGroup);
    for (const axis of tuned.filter((a) => !verification[a].keep)) {
      await tickV(STEP_AXES[axis].label);
      const v = verification[axis];
      v.after = evaluate(cfg, geo, axis, finalSet);
      if (v.before.cost <= v.after.cost * 1.03) { v.keep = true; dropGroup(axis); }
    }
  }
  const out = {};
  for (const axis of axes) {
    if (report[axis]) continue;
    const res = results[axis], v = verification[axis];
    if (!res || !v) continue;
    const suggested = res.params;
    const params = v.keep ? {} : Object.fromEntries(Object.entries(suggested).filter(([k]) => k in finalSet));
    Object.assign(out, params);
    const after = v.keep ? v.before : v.after;
    report[axis] = {
      keep: v.keep, before: v.before.r.metrics, after: after.r.metrics, beforeCost: v.before.cost, afterCost: after.cost,
      bandwidth: res.bandwidth ?? res.bandwidthHz, params, suggested, atBound: res.atBound, notes: res.notes,
      method: AXI[axis] !== undefined ? 'twitch (ArduPilot AutoTune: D up/P down, P up/D down, angle P up)' : 'cascade bandwidth search',
      twitch: res.twitch,
      curves: { before: { t: v.before.r.t, y: v.before.r.y }, after: { t: after.r.t, y: after.r.y }, target: after.r.yf, y0: after.r.y0 },
    };
  }
  // report each change once, in its final form
  for (const k of Object.keys(out)) if (P0[k] !== undefined && Math.abs(P0[k] - out[k]) < 1e-9) delete out[k];
  onProgress(1, 'Done');
  return { params: out, report, estimates: est, verification: Object.fromEntries(Object.entries(verification).map(([a, v]) => [a, { keep: v.keep, beforeCost: v.before.cost, afterCost: (v.keep ? v.before : v.after).cost }])), notes };
}
