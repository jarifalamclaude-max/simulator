// Port of ArduPilot libraries/AP_Math/control.cpp (ArduSub 4.5.7) and the
// filter helpers the controllers use (Filter/LowPassFilter, LowPassFilter2p,
// NotchFilter, SlewLimiter, SlewCalculator2D). Function names follow the C++ originals.

import { clamp, constrainFloat, isPositive, isNegative, isZero, sq, safeSqrt } from '../core/math.js';

// AP_Math.cpp:402 calc_lowpass_alpha_dt
export function calcLowpassAlpha(dt, cutoffHz) {
  if (isNegative(dt) || isNegative(cutoffHz)) return 1; // (INTERNAL_ERROR in firmware)
  if (isZero(cutoffHz)) return 1;
  if (isZero(dt)) return 0;
  const rc = 1 / (2 * Math.PI * cutoffHz);
  return dt / (dt + rc);
}

// ---------------- millisecond clock (AP_HAL::millis) ----------------
// SlewLimiter uses AP_HAL::millis(). By default each limiter integrates the dt it is given;
// the firmware can bind the simulated clock with setControlClock(() => timeMs).
let clockFn = null;
export function setControlClock(fn) { clockFn = typeof fn === 'function' ? fn : null; }

// ---------------- control.cpp ----------------
export function sqrtController(error, p, secondOrdLim, dt) {
  let rate;
  if (isNegative(secondOrdLim) || isZero(secondOrdLim)) rate = error * p;
  else if (isZero(p)) {
    if (isPositive(error)) rate = safeSqrt(2 * secondOrdLim * error);
    else if (isNegative(error)) rate = -safeSqrt(2 * secondOrdLim * -error);
    else rate = 0;
  } else {
    const linearDist = secondOrdLim / sq(p);
    if (error > linearDist) rate = safeSqrt(2 * secondOrdLim * (error - linearDist / 2));
    else if (error < -linearDist) rate = -safeSqrt(2 * secondOrdLim * (-error - linearDist / 2));
    else rate = error * p;
  }
  if (isPositive(dt)) return constrainFloat(rate, -Math.abs(error) / dt, Math.abs(error) / dt);
  return rate;
}

export function sqrtController2(err, p, lim, dt) {
  const l = Math.hypot(err[0], err[1]);
  if (!isPositive(l)) return [0, 0];
  const c = sqrtController(l, p, lim, dt);
  return [err[0] * (c / l), err[1] * (c / l)];
}

export function invSqrtController(output, p, Dmax) {
  if (isPositive(Dmax) && isZero(p)) return (output * output) / (2 * Dmax);
  if ((isNegative(Dmax) || isZero(Dmax)) && !isZero(p)) return output / p;
  if ((isNegative(Dmax) || isZero(Dmax)) && isZero(p)) return 0;
  const linearVelocity = Dmax / p;
  if (Math.abs(output) < linearVelocity) return output / p;
  const linearDist = Dmax / sq(p);
  const stopping = linearDist * 0.5 + sq(output) / (2 * Dmax);
  return isPositive(output) ? stopping : -stopping;
}
export const stoppingDistance = (velocity, p, accelMax) => invSqrtController(velocity, p, accelMax);

// ---- 2-vector helpers (Vector2f semantics) ----
export const v2len = (v) => Math.hypot(v[0], v[1]);
export const v2dot = (a, b) => a[0] * b[0] + a[1] * b[1];
export const v2isZero = (v) => isZero(v[0]) && isZero(v[1]); // Vector2::is_zero (component-wise is_zero)
// Vector2::limit_length (vector2.cpp:37): returns true when the vector was shortened; mutates v
export function limitLength2(v, maxLength) {
  const len = v2len(v);
  if (maxLength < len && isPositive(len)) { const k = maxLength / len; v[0] *= k; v[1] *= k; return true; }
  return false;
}

// update_vel_accel: returns new velocity
export function updateVelAccel(vel, accel, dt, limit, velError) {
  let dv = accel * dt;
  if (isPositive(dv * limit) && isPositive(velError * limit)) {
    if (isNegative(vel * limit)) dv = constrainFloat(dv, -Math.abs(vel), Math.abs(vel));
    else dv = 0;
  }
  return vel + dv;
}
// update_pos_vel_accel on an object {pos, vel}
export function updatePosVelAccel(st, accel, dt, limit, posError, velError) {
  let dp = st.vel * dt + accel * 0.5 * sq(dt);
  if (isPositive(dp * limit) && isPositive(posError * limit)) dp = 0;
  st.pos += dp;
  st.vel = updateVelAccel(st.vel, accel, dt, limit, velError);
}
// update_vel_accel_xy: mutates vel [x,y]
export function updateVelAccelXY(vel, accel, dt, limit = [0, 0], velError = [0, 0]) {
  const dv = [accel[0] * dt, accel[1] * dt];
  if (!v2isZero(limit) && !v2isZero(dv)) {
    if (isPositive(v2dot(dv, limit)) && isPositive(v2dot(velError, limit)) && !isNegative(v2dot(vel, limit))) { dv[0] = 0; dv[1] = 0; }
  }
  vel[0] += dv[0]; vel[1] += dv[1];
}
// update_pos_vel_accel_xy: mutates pos, vel
export function updatePosVelAccelXY(pos, vel, accel, dt, limit = [0, 0], posError = [0, 0], velError = [0, 0]) {
  const dp = [vel[0] * dt + accel[0] * 0.5 * sq(dt), vel[1] * dt + accel[1] * 0.5 * sq(dt)];
  if (!isZero(sq(limit[0]) + sq(limit[1]))) {
    if (isPositive(v2dot(dp, limit)) && isPositive(v2dot(posError, limit))) { dp[0] = 0; dp[1] = 0; }
  }
  pos[0] += dp[0]; pos[1] += dp[1];
  updateVelAccelXY(vel, accel, dt, limit, velError);
}

export function shapeAccel(accelInput, s, jerkMax, dt) {
  // s: {accel}
  if (!isPositive(jerkMax)) return;
  if (isPositive(dt)) s.accel += constrainFloat(accelInput - s.accel, -jerkMax * dt, jerkMax * dt);
}
// shape_accel_xy: mutates accel [x,y]
export function shapeAccelXY(accelInput, accel, jerkMax, dt) {
  if (!isPositive(jerkMax)) return;
  if (isPositive(dt)) {
    const d = [accelInput[0] - accel[0], accelInput[1] - accel[1]];
    limitLength2(d, jerkMax * dt);
    accel[0] += d[0]; accel[1] += d[1];
  }
}

export function shapeVelAccel(velInput, accelInput, vel, s, accelMin, accelMax, jerkMax, dt, limitTotal) {
  if (!isNegative(accelMin) || !isPositive(accelMax) || !isPositive(jerkMax)) return;
  const velError = velInput - vel;
  const KPa = isPositive(velError) ? jerkMax / accelMax : jerkMax / -accelMin;
  let accelTarget = sqrtController(velError, KPa, jerkMax, dt);
  accelTarget = constrainFloat(accelTarget, accelMin, accelMax);
  accelTarget += accelInput;
  if (limitTotal) accelTarget = constrainFloat(accelTarget, accelMin, accelMax);
  shapeAccel(accelTarget, s, jerkMax, dt);
}

const CORNER_ACCELERATION_RATIO = 1 / Math.sqrt(2);
// shape_vel_accel_xy: mutates accel [x,y]
export function shapeVelAccelXY(velInput, accelInput, vel, accel, accelMax, jerkMax, dt, limitTotal) {
  if (!isPositive(accelMax) || !isPositive(jerkMax)) return;
  const KPa = jerkMax / accelMax;
  const velError = [velInput[0] - vel[0], velInput[1] - vel[1]];
  let at = sqrtController2(velError, KPa, jerkMax, dt);
  if (v2isZero(velInput)) limitLength2(at, accelMax);
  else {
    const l = v2len(velInput), u = [velInput[0] / l, velInput[1] / l];
    let dir = v2dot(u, at);
    const cross = [at[0] - u[0] * dir, at[1] - u[1] * dir];
    if (sq(dir) <= sq(cross[0]) + sq(cross[1])) limitLength2(at, accelMax);
    else {
      limitLength2(cross, CORNER_ACCELERATION_RATIO * accelMax);
      const dmax = safeSqrt(sq(accelMax) - (sq(cross[0]) + sq(cross[1])));
      dir = constrainFloat(dir, -dmax, dmax);
      at = [cross[0] + u[0] * dir, cross[1] + u[1] * dir];
    }
  }
  at[0] += accelInput[0]; at[1] += accelInput[1];
  if (limitTotal) limitLength2(at, accelMax);
  shapeAccelXY(at, accel, jerkMax, dt);
}

export function shapePosVelAccel(posInput, velInput, accelInput, pos, vel, s, velMin, velMax, accelMin, accelMax, jerkMax, dt, limitTotal) {
  if (isPositive(velMin) || isNegative(velMax) || !isNegative(accelMin) || !isPositive(accelMax) || !isPositive(jerkMax)) return;
  const posError = posInput - pos;
  let accelTcMax, KPv;
  if (isPositive(posError)) { accelTcMax = -0.5 * accelMin; KPv = (0.5 * jerkMax) / -accelMin; }
  else { accelTcMax = 0.5 * accelMax; KPv = (0.5 * jerkMax) / accelMax; }
  let velTarget = sqrtController(posError, KPv, accelTcMax, dt);
  if (isNegative(velMin) || isPositive(velMax)) velTarget = constrainFloat(velTarget, velMin, velMax);
  velTarget += velInput;
  if (limitTotal) velTarget = constrainFloat(velTarget, velMin, velMax);
  shapeVelAccel(velTarget, accelInput, vel, s, accelMin, accelMax, jerkMax, dt, limitTotal);
}
// shape_pos_vel_accel_xy: mutates accel [x,y]
export function shapePosVelAccelXY(posInput, velInput, accelInput, pos, vel, accel, velMax, accelMax, jerkMax, dt, limitTotal) {
  if (isNegative(velMax) || !isPositive(accelMax) || !isPositive(jerkMax)) return;
  const KPv = (0.5 * jerkMax) / accelMax, accelTcMax = 0.5 * accelMax;
  const posError = [posInput[0] - pos[0], posInput[1] - pos[1]];
  const vt = sqrtController2(posError, KPv, accelTcMax, dt);
  if (isPositive(velMax)) limitLength2(vt, velMax);
  vt[0] += velInput[0]; vt[1] += velInput[1];
  if (limitTotal) limitLength2(vt, velMax);
  shapeVelAccelXY(vt, accelInput, vel, accel, accelMax, jerkMax, dt, limitTotal);
}

// limit_accel_xy (control.cpp:369): mutates accel, returns true when limited
export function limitAccelXY(vel, accel, accelMax) {
  if (!isPositive(accelMax)) return false;
  if (sq(accel[0]) + sq(accel[1]) > sq(accelMax)) {
    if (v2isZero(vel)) limitLength2(accel, accelMax);
    else {
      const l = v2len(vel), u = [vel[0] / l, vel[1] / l];
      let dir = v2dot(u, accel);
      const cross = [accel[0] - u[0] * dir, accel[1] - u[1] * dir];
      if (limitLength2(cross, accelMax)) dir = 0;
      else {
        const dmax = safeSqrt(sq(accelMax) - (sq(cross[0]) + sq(cross[1])));
        dir = constrainFloat(dir, -dmax, dmax);
      }
      accel[0] = cross[0] + u[0] * dir; accel[1] = cross[1] + u[1] * dir;
    }
    return true;
  }
  return false;
}

// kinematic_limit (control.cpp:450)
export function kinematicLimit(direction, maxXY, maxZPos, maxZNeg) {
  if (isZero(sq(direction[0]) + sq(direction[1]) + sq(direction[2])) || isZero(maxXY) || isZero(maxZPos) || isZero(maxZNeg)) return 0;
  maxXY = Math.abs(maxXY); maxZPos = Math.abs(maxZPos); maxZNeg = Math.abs(maxZNeg);
  const n = Math.hypot(...direction), d = direction.map((v) => v / n);
  const xyLen = Math.hypot(d[0], d[1]);
  if (isZero(xyLen)) return isPositive(d[2]) ? maxZPos : maxZNeg;
  if (isZero(d[2])) return maxXY;
  const slope = d[2] / xyLen;
  if (isPositive(slope)) return Math.abs(slope) < maxZPos / maxXY ? maxXY / xyLen : Math.abs(maxZPos / d[2]);
  return Math.abs(slope) < maxZNeg / maxXY ? maxXY / xyLen : Math.abs(maxZNeg / d[2]);
}

export function inputExpo(input, expo) {
  input = constrainFloat(input, -1, 1);
  if (expo < 0.95) return ((1 - expo) * input) / (1 - expo * Math.abs(input));
  return input;
}
export const angleToAccel = (deg) => 9.80665 * Math.tan((deg * Math.PI) / 180);
export const accelToAngle = (accel) => (Math.atan(accel / 9.80665) * 180) / Math.PI;

// rc_input_to_roll_pitch (control.cpp:540) -> [rollDeg, pitchDeg]
export function rcInputToRollPitch(rollInUnit, pitchInUnit, angleMaxDeg, angleLimitDeg) {
  angleMaxDeg = Math.min(angleMaxDeg, 85);
  const rc2rad = (angleMaxDeg * Math.PI) / 180;
  const t = [-Math.tan(rc2rad * pitchInUnit), Math.tan(rc2rad * rollInUnit)];
  angleLimitDeg = constrainFloat(angleLimitDeg, 10, angleMaxDeg);
  limitLength2(t, Math.tan((angleLimitDeg * Math.PI) / 180));
  const pitch = -Math.atan(t[0]), roll = Math.atan(Math.cos(pitch) * t[1]);
  return [(roll * 180) / Math.PI, (pitch * 180) / Math.PI];
}

// ---------------- filters ----------------
// Filter/LowPassFilter.cpp DigitalLPF + LowPassFilter (cutoff held, dt per call)
export class LowPassFilter {
  constructor(cutoff = 0) { this.cutoff = cutoff; this.out = 0; this.init = false; }
  setCutoff(c) { this.cutoff = c; }
  apply(x, dt) {
    if (isNegative(this.cutoff) || isNegative(dt)) { this.out = x; return x; }
    if (isZero(this.cutoff)) { this.out = x; return x; }
    if (isZero(dt)) return this.out;
    const rc = 1 / (2 * Math.PI * this.cutoff);
    this.out += (x - this.out) * clamp(dt / (dt + rc), 0, 1);
    if (!this.init) { this.init = true; this.out = x; }
    return this.out;
  }
  reset(v = 0) { this.out = v; this.init = true; }
  get() { return this.out; }
}

// 2-pole Butterworth (ArduPilot DigitalBiquadFilter / LowPassFilter2p, LowPassFilter2p.cpp:51)
export class LowPassFilter2p {
  constructor(sampleHz, cutoffHz) { this.configure(sampleHz, cutoffHz); }
  configure(sampleHz, cutoffHz) {
    this.sampleHz = sampleHz;
    this.d1 = null; this.d2 = null;
    const fc = Math.min(cutoffHz, sampleHz * 0.4); // compute_params: MIN(cutoff, 0.4*fs)
    this.cutoffHz = fc;
    if (!isPositive(fc) || !isPositive(sampleHz)) { this.pass = true; return; }
    this.pass = false;
    const fr = sampleHz / fc;
    const ohm = Math.tan(Math.PI / fr);
    const c = 1 + 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm;
    this.b0 = (ohm * ohm) / c; this.b1 = 2 * this.b0; this.b2 = this.b0;
    this.a1 = (2 * (ohm * ohm - 1)) / c;
    this.a2 = (1 - 2 * Math.cos(Math.PI / 4) * ohm + ohm * ohm) / c;
  }
  apply(x) {
    if (this.pass) return x;
    if (this.d1 === null) { const v = x * (1 / (1 + this.a1 + this.a2)); this.d1 = v; this.d2 = v; }
    const d0 = x - this.d1 * this.a1 - this.d2 * this.a2;
    const out = d0 * this.b0 + this.d1 * this.b1 + this.d2 * this.b2;
    this.d2 = this.d1; this.d1 = d0;
    return out;
  }
  reset() { this.d1 = null; this.d2 = null; }
}

// Filter/NotchFilter.cpp (scalar), with the 5 % centre-frequency slew of init_with_A_and_Q
const NOTCH_MAX_SLEW = 0.05, NOTCH_MAX_SLEW_LOWER = 1 - NOTCH_MAX_SLEW, NOTCH_MAX_SLEW_UPPER = 1 / NOTCH_MAX_SLEW_LOWER;
export class NotchFilter {
  constructor() { this.initialised = false; this.needReset = false; this.centerFreqHz = 0; this.sampleFreqHz = 0; this.s1 = this.s2 = this.n1 = this.n2 = 0; }
  static calculateAandQ(centerHz, bandwidthHz, attenuationDb) {
    const A = Math.pow(10, -attenuationDb / 40);
    let Q = 0;
    if (centerHz > 0.5 * bandwidthHz) {
      const octaves = Math.log2(centerHz / (centerHz - bandwidthHz / 2)) * 2;
      Q = Math.sqrt(Math.pow(2, octaves)) / (Math.pow(2, octaves) - 1);
    }
    return { A, Q };
  }
  init(sampleHz, centerHz, bandwidthHz, attenuationDb) {
    if (centerHz > 0.5 * bandwidthHz && centerHz < 0.5 * sampleHz) {
      this.initialised = false; // force centre frequency change
      const { A, Q } = NotchFilter.calculateAandQ(centerHz, bandwidthHz, attenuationDb);
      this.initWithAandQ(sampleHz, centerHz, A, Q);
    } else this.initialised = false;
  }
  initWithAandQ(sampleHz, centerHz, A, Q) {
    if (this.initialised && centerHz === this.centerFreqHz && sampleHz === this.sampleFreqHz) return;
    let fc = centerHz;
    if (this.initialised && !this.needReset && !isZero(this.centerFreqHz)) fc = constrainFloat(fc, this.centerFreqHz * NOTCH_MAX_SLEW_LOWER, this.centerFreqHz * NOTCH_MAX_SLEW_UPPER);
    if (isPositive(fc) && fc < 0.5 * sampleHz && Q > 0) {
      const omega = (2 * Math.PI * fc) / sampleHz, alpha = Math.sin(omega) / (2 * Q);
      const a0inv = 1 / (1 + alpha);
      this.b0 = (1 + alpha * sq(A)) * a0inv; this.b1 = -2 * Math.cos(omega) * a0inv; this.b2 = (1 - alpha * sq(A)) * a0inv;
      this.a1 = this.b1; this.a2 = (1 - alpha) * a0inv;
      this.centerFreqHz = fc; this.sampleFreqHz = sampleHz; this.initialised = true;
    } else this.initialised = false;
  }
  apply(x) {
    if (!this.initialised || this.needReset) { this.s1 = this.s2 = this.n1 = this.n2 = x; this.needReset = false; return x; }
    const out = x * this.b0 + this.n1 * this.b1 + this.n2 * this.b2 - this.s1 * this.a1 - this.s2 * this.a2;
    this.n2 = this.n1; this.n1 = x; this.s2 = this.s1; this.s1 = out;
    return out;
  }
  reset() { this.needReset = true; }
}

// AP_Filters bank (Filter/AP_Filter.cpp, AP_NotchFilter_params.cpp): FILTn_TYPE (0 none, 1 notch),
// FILTn_NOTCH_FREQ / _Q (default 2) / _ATT (default 40 dB). get_filter(index) returns filters[index-1]
// but rejects index >= AP_FILTER_NUM_FILTERS (8), as in 4.5.7.
export const AP_FILTER_NUM_FILTERS = 8;
export function filterBankFromParams(P = {}) {
  const bank = [];
  for (let i = 1; i <= AP_FILTER_NUM_FILTERS; i++) {
    const type = P[`FILT${i}_TYPE`] ?? 0;
    bank.push(type === 1 ? { type, freq: P[`FILT${i}_NOTCH_FREQ`] ?? 0, q: P[`FILT${i}_NOTCH_Q`] ?? 2, att: P[`FILT${i}_NOTCH_ATT`] ?? 40 } : null);
  }
  return bank;
}
export function filterBankGet(bank, index) {
  if (!bank || index >= AP_FILTER_NUM_FILTERS) return null;
  return bank[index - 1] ?? null;
}
// AP_NotchFilter_params::setup_notch_filter
export function setupNotchFilter(def, filter, sampleRate) {
  if (isZero(def.q) || isZero(def.freq) || isZero(def.att)) return false;
  if (sampleRate !== filter.sampleFreqHz || def.freq !== filter.centerFreqHz) filter.init(sampleRate, def.freq, def.freq / def.q, def.att);
  return true;
}

// Filter/SlewLimiter.cpp
const WINDOW_MS = 300, MODIFIER_GAIN = 1.5, DERIVATIVE_CUTOFF_FREQ = 25, SLEWLIMITER_N_EVENTS = 2;
export class SlewLimiter {
  constructor() {
    this.filter = new LowPassFilter(DERIVATIVE_CUTOFF_FREQ); this.filter.reset(0);
    this.outputSlewRate = 0; this.modifierSlewRate = 0; this.lastSample = 0;
    this.maxPos = 0; this.maxNeg = 0; this.maxPosMs = 0; this.maxNegMs = 0;
    this.posIdx = 0; this.negIdx = 0; this.posMs = new Array(SLEWLIMITER_N_EVENTS).fill(0); this.negMs = new Array(SLEWLIMITER_N_EVENTS).fill(0);
    this.posStored = false; this.negStored = false; this.tMs = 0;
  }
  getSlewRate() { return this.outputSlewRate; }
  // returns a multiplier in (0,1] keeping the P+D output within slewRateMax
  modifier(sample, dt, slewRateMax, slewRateTau = 1) {
    if (!isPositive(dt)) return 1;
    const slewRate = this.filter.apply((sample - this.lastSample) / dt, dt);
    this.lastSample = sample;
    this.tMs += dt * 1000;
    const nowMs = clockFn ? Math.floor(clockFn()) : Math.floor(this.tMs);
    const decay = Math.min(dt, slewRateTau) / slewRateTau;
    const attack = Math.min(2 * decay, 1);
    if (slewRate > this.maxPos) { this.maxPos = slewRate; this.maxPosMs = nowMs; }
    else if (nowMs - this.maxPosMs > WINDOW_MS) this.maxPos *= 1 - decay;
    if (-slewRate > this.maxNeg) { this.maxNeg = -slewRate; this.maxNegMs = nowMs; }
    else if (nowMs - this.maxNegMs > WINDOW_MS) this.maxNeg *= 1 - decay;
    const raw = 0.5 * (this.maxPos + this.maxNeg);
    this.outputSlewRate = Math.min((1 - attack) * this.outputSlewRate + attack * raw, raw);
    if (slewRateMax <= 0) return 1;
    const limitedRaw = 0.5 * (Math.min(this.maxPos, 10 * slewRateMax) + Math.min(this.maxNeg, 10 * slewRateMax));
    if (!this.posStored && slewRate > slewRateMax) {
      if (this.posIdx >= SLEWLIMITER_N_EVENTS) this.posIdx = 0;
      this.posMs[this.posIdx++] = nowMs; this.posStored = true; this.negStored = false;
    }
    if (!this.negStored && -slewRate > slewRateMax) {
      if (this.negIdx >= SLEWLIMITER_N_EVENTS) this.negIdx = 0;
      this.negMs[this.negIdx++] = nowMs; this.negStored = true; this.posStored = false;
    }
    let oldest = nowMs;
    for (let k = 0; k < SLEWLIMITER_N_EVENTS; k++) oldest = Math.min(oldest, this.posMs[k], this.negMs[k]);
    let modIn = limitedRaw;
    if (nowMs - oldest > (SLEWLIMITER_N_EVENTS + 1) * WINDOW_MS) modIn *= Math.exp(-(0.001 * (nowMs - oldest - (SLEWLIMITER_N_EVENTS + 1) * WINDOW_MS)) / slewRateTau);
    this.modifierSlewRate = Math.min((1 - attack) * this.modifierSlewRate + attack * modIn, modIn);
    let mod = 1;
    if (this.modifierSlewRate > slewRateMax) mod = slewRateMax / (slewRateMax + MODIFIER_GAIN * (this.modifierSlewRate - slewRateMax));
    return mod;
  }
}

// Filter/SlewCalculator2D.cpp (get_slew_rate uses the x limiter twice, as in 4.5.7)
export class SlewCalculator2D {
  constructor() { this.x = new SlewLimiter(); this.y = new SlewLimiter(); }
  update(sample, dt) { if (!isPositive(dt)) return; this.x.modifier(sample[0], dt, 0, 1); this.y.modifier(sample[1], dt, 0, 1); }
  getSlewRate() { return safeSqrt(sq(this.x.getSlewRate()) + sq(this.x.getSlewRate())); }
}
