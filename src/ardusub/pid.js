// Ports of ArduPilot AC_PID, AC_PID_Basic, AC_P, AC_P_1D, AC_P_2D, AC_PI, AC_PI_2D, AC_PID_2D (4.5.7).
// Gains are plain fields (kp, ki, ...) so callers and the tuner can read them; setGains() updates
// gains live without touching integrators or filter state.

import { calcLowpassAlpha, sqrtController, sqrtController2, invSqrtController, NotchFilter, SlewLimiter, SlewCalculator2D, filterBankGet, setupNotchFilter, limitLength2 } from './control.js';
import { constrainFloat, isPositive, isNegative, isZero } from '../core/math.js';

const finite2 = (v) => Number.isFinite(v[0]) && Number.isFinite(v[1]);
const newInfo = () => ({ target: 0, actual: 0, error: 0, P: 0, I: 0, D: 0, FF: 0, DFF: 0, Dmod: 1, slew_rate: 0, limit: false, PD_limit: false, reset: false, I_term_set: false });

// libraries/AC_PID/AC_PID.cpp
export class AC_PID {
  constructor(g = {}) {
    this.kp = 0; this.ki = 0; this.kd = 0; this.kff = 0; this.kimax = 0; this.filtT = 0; this.filtE = 0; this.filtD = 0;
    this.kpdmax = 0; this.kdff = 0; this.slewRateMax = 0; this.notchTIdx = 0; this.notchEIdx = 0;
    this.slewRateTau = 1; // _slew_rate_tau (not a parameter, AC_PID.h:42 default 1.0)
    this.slewLimitScale = 1;
    this.setGains(g);
    this.target = 0; this.error = 0; this.derivative = 0; this.targetDerivative = 0; this.integrator = 0;
    this.resetFilterFlag = true; this.iSet = false;
    this.slew = new SlewLimiter();
    this.targetNotch = null; this.errorNotch = null;
    this.info = newInfo();
  }
  // AP_Param load: values are taken as stored (imax and smax through fabsf as the setters do)
  setGains(g) {
    if (g.p !== undefined) this.kp = g.p;
    if (g.i !== undefined) this.ki = g.i;
    if (g.d !== undefined) this.kd = g.d;
    if (g.ff !== undefined) this.kff = g.ff;
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
    if (g.fltt !== undefined) this.filtT = g.fltt;
    if (g.flte !== undefined) this.filtE = g.flte;
    if (g.fltd !== undefined) this.filtD = g.fltd;
    if (g.pdmx !== undefined) this.kpdmax = g.pdmx;
    if (g.dff !== undefined) this.kdff = g.dff;
    if (g.smax !== undefined) this.slewRateMax = Math.abs(g.smax);
    if (g.ntf !== undefined) this.notchTIdx = Math.trunc(g.ntf) || 0;
    if (g.nef !== undefined) this.notchEIdx = Math.trunc(g.nef) || 0;
  }
  // ---- accessors / setters (AC_PID.h) ----
  kP() { return this.kp; } kI() { return this.ki; } kD() { return this.kd; } ff() { return this.kff; } kDff() { return this.kdff; }
  imax(v) { if (v !== undefined) this.kimax = Math.abs(v); return this.kimax; }
  pdmax(v) { if (v !== undefined) this.kpdmax = Math.abs(v); return this.kpdmax; }
  filtTHz(v) { if (v !== undefined) this.filtT = Math.abs(v); return this.filtT; }
  filtEHz(v) { if (v !== undefined) this.filtE = Math.abs(v); return this.filtE; }
  filtDHz(v) { if (v !== undefined) this.filtD = Math.abs(v); return this.filtD; }
  slewLimit(v) { if (v !== undefined) this.slewRateMax = Math.abs(v); return this.slewRateMax; }
  setSlewLimitScale(s) { this.slewLimitScale = s; }
  getSlewRate() { return this.slew.getSlewRate(); }
  getFiltTAlpha(dt) { return calcLowpassAlpha(dt, this.filtT); }
  getFiltEAlpha(dt) { return calcLowpassAlpha(dt, this.filtE); }
  getFiltDAlpha(dt) { return calcLowpassAlpha(dt, this.filtD); }
  setTargetRate(v) { this.info.target = v; }
  setActualRate(v) { this.info.actual = v; }
  getPidInfo() { return this.info; }
  getError() { return this.error; }

  // set_notch_sample_rate (AC_PID.cpp:156): bank = filterBankFromParams(P) (AP_Filters)
  setNotchSampleRate(sampleRate, bank) {
    if (this.notchTIdx === 0 && this.notchEIdx === 0) return;
    if (this.notchTIdx !== 0) {
      if (!this.targetNotch) this.targetNotch = new NotchFilter();
      const f = filterBankGet(bank, this.notchTIdx);
      if (f && !setupNotchFilter(f, this.targetNotch, sampleRate)) { this.targetNotch = null; this.notchTIdx = 0; }
    }
    if (this.notchEIdx !== 0) {
      if (!this.errorNotch) this.errorNotch = new NotchFilter();
      const f = filterBankGet(bank, this.notchEIdx);
      if (f && !setupNotchFilter(f, this.errorNotch, sampleRate)) { this.errorNotch = null; this.notchEIdx = 0; }
    }
  }

  // update_all (AC_PID.cpp:194)
  updateAll(target, measurement, dt, limit = false, boost = 1) {
    if (!Number.isFinite(target) || !Number.isFinite(measurement)) return 0;
    const info = this.info;
    info.reset = this.resetFilterFlag;
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.target = target;
      this.error = this.target - measurement;
      this.derivative = 0;
      this.targetDerivative = 0;
      if (this.targetNotch) { this.targetNotch.reset(); this.target = this.targetNotch.apply(this.target); }
      if (this.errorNotch) { this.errorNotch.reset(); this.error = this.errorNotch.apply(this.error); }
    } else {
      const errorLast = this.error, targetLast = this.target;
      let error = this.target - measurement;
      // notches before FLTT/FLTE
      if (this.targetNotch) target = this.targetNotch.apply(target);
      if (this.errorNotch) error = this.errorNotch.apply(error);
      this.target += this.getFiltTAlpha(dt) * (target - this.target);
      this.error += this.getFiltEAlpha(dt) * (error - this.error);
      if (isPositive(dt)) {
        const derivative = (this.error - errorLast) / dt;
        this.derivative += this.getFiltDAlpha(dt) * (derivative - this.derivative);
        this.targetDerivative = (this.target - targetLast) / dt;
      }
    }
    this.updateI(dt, limit);
    let P = this.error * this.kp, D = this.derivative * this.kd;
    // SMAX slew limiter on the previous cycle's P+D (AC_PID.cpp:248)
    info.Dmod = this.slew.modifier((info.P + info.D) * this.slewLimitScale, dt, this.slewRateMax, this.slewRateTau);
    info.slew_rate = this.slew.getSlewRate();
    P *= info.Dmod; D *= info.Dmod;
    P *= boost; D *= boost;
    info.PD_limit = false;
    if (isPositive(this.kpdmax)) {
      const s = Math.abs(P + D);
      if (s > this.kpdmax) { const k = this.kpdmax / s; P *= k; D *= k; info.PD_limit = true; }
    }
    info.target = this.target; info.actual = measurement; info.error = this.error; info.P = P; info.D = D;
    info.FF = this.target * this.kff; info.DFF = this.targetDerivative * this.kdff;
    return P + D + this.integrator;
  }
  // update_error (AC_PID.cpp:288)
  updateError(error, dt, limit = false) {
    if (!Number.isFinite(error)) return 0;
    this.target = 0;
    const out = this.updateAll(0, -error, dt, limit);
    this.info.target = 0; this.info.actual = 0;
    return out;
  }
  updateI(dt, limit) {
    if (!isZero(this.ki) && isPositive(dt)) {
      if (!limit || (isPositive(this.integrator) && isNegative(this.error)) || (isNegative(this.integrator) && isPositive(this.error))) {
        this.integrator += this.error * this.ki * dt;
        this.integrator = constrainFloat(this.integrator, -this.kimax, this.kimax);
      }
    } else this.integrator = 0;
    this.info.I = this.integrator;
    this.info.limit = limit;
    this.info.I_term_set = this.iSet;
    this.iSet = false;
  }
  getP() { return this.info.P; }
  getI() { return this.integrator; }
  getD() { return this.info.D; }
  getFF() { return this.info.FF + this.info.DFF; }
  resetI() { this.iSet = true; this.integrator = 0; this.info.I = 0; }
  resetFilter() { this.resetFilterFlag = true; }
  setIntegrator(v) { this.iSet = true; this.integrator = constrainFloat(v, -this.kimax, this.kimax); }
  relaxIntegrator(v, dt, tc) {
    v = constrainFloat(v, -this.kimax, this.kimax);
    if (isPositive(dt)) { this.iSet = true; this.integrator += (v - this.integrator) * (dt / (dt + tc)); }
  }
  // autotune helpers (load_gains/save_gains): snapshot/restore in memory
  saveGains() { this._saved = { p: this.kp, i: this.ki, d: this.kd, ff: this.kff, fltt: this.filtT, flte: this.filtE, fltd: this.filtD }; }
  loadGains() { if (this._saved) this.setGains(this._saved); }
}

// libraries/AC_PID/AC_PID_Basic.cpp
export class AC_PID_Basic {
  constructor(g = {}) {
    this.kp = 0; this.ki = 0; this.kd = 0; this.kff = 0; this.kimax = 0; this.filtE = 0; this.filtD = 0;
    this.setGains(g);
    this.target = 0; this.error = 0; this.derivative = 0; this.integrator = 0; this.resetFilterFlag = true;
    this.info = newInfo();
  }
  setGains(g) {
    for (const [k, f] of [['p', 'kp'], ['i', 'ki'], ['d', 'kd'], ['ff', 'kff'], ['flte', 'filtE'], ['fltd', 'filtD']]) if (g[k] !== undefined) this[f] = g[k];
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
  }
  kP() { return this.kp; } kI() { return this.ki; } kD() { return this.kd; } ff() { return this.kff; }
  imax(v) { if (v !== undefined) this.kimax = Math.abs(v); return this.kimax; }
  getError() { return this.error; }
  getPidInfo() { return this.info; }
  // update_all(target, meas, dt, limit_neg, limit_pos); the 4-argument call is the bool-limit overload
  // (AC_PID_Basic.cpp:69): limit_neg = limit && I<0, limit_pos = limit && I>0
  updateAll(target, measurement, dt, limitNeg = false, limitPos = false) {
    if (arguments.length === 4) { const l = !!limitNeg; limitNeg = l && isNegative(this.integrator); limitPos = l && isPositive(this.integrator); }
    if (!Number.isFinite(target) || !Number.isFinite(measurement)) return 0;
    this.target = target;
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.error = this.target - measurement;
      this.derivative = 0;
    } else {
      const last = this.error;
      this.error += calcLowpassAlpha(dt, this.filtE) * (this.target - measurement - this.error);
      if (isPositive(dt)) this.derivative += calcLowpassAlpha(dt, this.filtD) * ((this.error - last) / dt - this.derivative);
    }
    this.updateI(dt, limitNeg, limitPos);
    const i = this.info;
    i.target = this.target; i.actual = measurement; i.error = this.error; i.P = this.error * this.kp; i.I = this.integrator; i.D = this.derivative * this.kd; i.FF = this.target * this.kff;
    return this.error * this.kp + this.integrator + this.derivative * this.kd + this.target * this.kff;
  }
  updateI(dt, limitNeg, limitPos) {
    if (!isZero(this.ki)) {
      if (!((limitNeg && isNegative(this.error)) || (limitPos && isPositive(this.error)))) {
        this.integrator += this.error * this.ki * dt;
        this.integrator = constrainFloat(this.integrator, -this.kimax, this.kimax);
      }
    } else this.integrator = 0;
  }
  getP() { return this.error * this.kp; }
  getI() { return this.integrator; }
  getD() { return this.derivative * this.kd; }
  getFF() { return this.target * this.kff; }
  resetFilter() { this.resetFilterFlag = true; }
  resetI() { this.integrator = 0; }
  setIntegrator(v) { this.integrator = constrainFloat(v, -this.kimax, this.kimax); }
  setIntegratorError(error, i) { this.setIntegrator(i - error * this.kp); }
  setIntegratorTM(target, measurement, i) { this.setIntegratorError(target - measurement, i); }
}

// libraries/AC_PID/AC_P.cpp (angle P controllers)
export class AC_P {
  constructor(kp = 0) { this.kp = kp; }
  kP() { return this.kp; }
  getP(error) { return error * this.kp; }
}

// libraries/AC_PID/AC_PI.cpp (integrator constrained to 0..imax)
export class AC_PI {
  constructor(kp = 0, ki = 0, imax = 0) { this.kp = kp; this.ki = ki; this.kimax = imax; this.integrator = 0; this.outputP = 0; }
  update(measurement, target, dt) {
    const err = target - measurement;
    this.integrator = constrainFloat(this.integrator + this.ki * err * dt, 0, this.kimax);
    this.outputP = this.kp * err;
    return this.outputP + this.integrator;
  }
  getP() { return this.outputP; }
  getI() { return this.integrator; }
}

// libraries/AC_PID/AC_PI_2D.cpp
const AC_PI_2D_FILT_HZ_MIN = 0.01;
export class AC_PI_2D {
  constructor(kp = 0, ki = 0, imax = 0, filtHz = 0, dt = 0.0025) {
    this.kp = kp; this.ki = ki; this.kimax = Math.abs(imax); this.dt = dt; this.filtHzV = 0; this.alpha = 1;
    this.input = [0, 0]; this.integrator = [0, 0]; this.resetFilterFlag = true;
    this.filtHz(filtHz);
  }
  setDt(dt) { this.dt = dt; this.calcFiltAlpha(); }
  filtHz(hz) { this.filtHzV = Math.max(Math.abs(hz), AC_PI_2D_FILT_HZ_MIN); this.calcFiltAlpha(); }
  calcFiltAlpha() { if (isZero(this.filtHzV)) { this.alpha = 1; return; } const rc = 1 / (2 * Math.PI * this.filtHzV); this.alpha = this.dt / (this.dt + rc); }
  setInput(v) {
    if (!finite2(v)) return;
    if (this.resetFilterFlag) { this.resetFilterFlag = false; this.input = v.slice(); }
    this.input = [this.input[0] + (v[0] - this.input[0]) * this.alpha, this.input[1] + (v[1] - this.input[1]) * this.alpha];
  }
  getP() { return [this.input[0] * this.kp, this.input[1] * this.kp]; }
  getI() {
    if (!isZero(this.ki) && !isZero(this.dt)) {
      this.integrator = [this.integrator[0] + this.input[0] * this.ki * this.dt, this.integrator[1] + this.input[1] * this.ki * this.dt];
      limitLength2(this.integrator, this.kimax);
      return this.integrator.slice();
    }
    return [0, 0];
  }
  getIShrink() {
    if (!isZero(this.ki) && !isZero(this.dt)) {
      const l0 = Math.min(Math.hypot(...this.integrator), this.kimax);
      this.integrator = [this.integrator[0] + this.input[0] * this.ki * this.dt, this.integrator[1] + this.input[1] * this.ki * this.dt];
      const l1 = Math.hypot(...this.integrator);
      if (l1 > l0 && isPositive(l1)) this.integrator = this.integrator.map((v) => (v * l0) / l1);
      return this.integrator.slice();
    }
    return [0, 0];
  }
  getPi() { const p = this.getP(), i = this.getI(); return [p[0] + i[0], p[1] + i[1]]; }
  resetI() { this.integrator = [0, 0]; }
  resetFilter() { this.resetFilterFlag = true; }
}

// libraries/AC_PID/AC_P_1D.cpp
export class AC_P_1D {
  constructor(kp = 1) { this.kp = kp; this.errorMin = 0; this.errorMax = 0; this.D1max = 0; this.error = 0; }
  kP() { return this.kp; }
  getError() { return this.error; }
  getErrorMin() { return this.errorMin; }
  getErrorMax() { return this.errorMax; }
  setLimits(outMin, outMax, DOutMax = 0, D2OutMax = 0) {
    this.D1max = 0; this.errorMin = 0; this.errorMax = 0;
    if (isPositive(DOutMax)) this.D1max = DOutMax;
    if (isPositive(D2OutMax) && isPositive(this.kp)) this.D1max = Math.min(this.D1max, D2OutMax / this.kp);
    if (isNegative(outMin) && isPositive(this.kp)) this.errorMin = invSqrtController(outMin, this.kp, this.D1max);
    if (isPositive(outMax) && isPositive(this.kp)) this.errorMax = invSqrtController(outMax, this.kp, this.D1max);
  }
  // set_error_limits (AC_P_1D.cpp:68): narrow the error limits after set_limits()
  setErrorLimits(errorMin, errorMax) {
    if (isNegative(errorMin)) this.errorMin = !isZero(this.errorMin) ? Math.max(this.errorMin, errorMin) : errorMin;
    if (isPositive(errorMax)) this.errorMax = !isZero(this.errorMax) ? Math.min(this.errorMax, errorMax) : errorMax;
  }
  // returns {out, target} (target may be pulled toward the measurement)
  updateAll(target, measurement) {
    this.error = target - measurement;
    if (isNegative(this.errorMin) && this.error < this.errorMin) { this.error = this.errorMin; target = measurement + this.error; }
    else if (isPositive(this.errorMax) && this.error > this.errorMax) { this.error = this.errorMax; target = measurement + this.error; }
    return { out: sqrtController(this.error, this.kp, this.D1max, 0), target };
  }
}

// libraries/AC_PID/AC_P_2D.cpp
export class AC_P_2D {
  constructor(kp = 1) { this.kp = kp; this.errorMax = 0; this.D1max = 0; this.error = [0, 0]; }
  kP() { return this.kp; }
  getError() { return this.error; }
  getErrorMax() { return this.errorMax; }
  setLimits(outMax, DOutMax = 0, D2OutMax = 0) {
    this.D1max = 0; this.errorMax = 0;
    if (isPositive(DOutMax)) this.D1max = DOutMax;
    if (isPositive(D2OutMax) && isPositive(this.kp)) this.D1max = Math.min(this.D1max, D2OutMax / this.kp);
    if (isPositive(outMax) && isPositive(this.kp)) this.errorMax = invSqrtController(outMax, this.kp, this.D1max);
  }
  setErrorMax(errorMax) {
    if (isPositive(errorMax)) this.errorMax = !isZero(this.errorMax) ? Math.min(this.errorMax, errorMax) : errorMax;
  }
  updateAll(target, meas) {
    const e = [target[0] - meas[0], target[1] - meas[1]];
    if (isPositive(this.errorMax) && limitLength2(e, this.errorMax)) target = [meas[0] + e[0], meas[1] + e[1]];
    this.error = e;
    return { out: sqrtController2(e, this.kp, this.D1max, 0), target };
  }
}

// libraries/AC_PID/AC_PID_2D.cpp
export class AC_PID_2D {
  constructor(g = {}) {
    this.kp = 0; this.ki = 0; this.kd = 0; this.kff = 0; this.kimax = 0; this.filtE = 0; this.filtD = 0;
    this.setGains(g);
    this.error = [0, 0]; this.derivative = [0, 0]; this.integrator = [0, 0]; this.target = [0, 0]; this.resetFilterFlag = true;
    this.slewCalc = new SlewCalculator2D();
    this.infoX = newInfo(); this.infoY = newInfo();
  }
  setGains(g) {
    for (const [k, f] of [['p', 'kp'], ['i', 'ki'], ['d', 'kd'], ['ff', 'kff'], ['flte', 'filtE'], ['fltd', 'filtD']]) if (g[k] !== undefined) this[f] = g[k];
    if (g.imax !== undefined) this.kimax = Math.abs(g.imax);
  }
  kP() { return this.kp; } kI() { return this.ki; } kD() { return this.kd; } ff() { return this.kff; }
  imax(v) { if (v !== undefined) this.kimax = Math.abs(v); return this.kimax; }
  getError() { return this.error; }
  updateAll(target, meas, dt, limit = [0, 0]) {
    if (!finite2(target) || !finite2(meas)) return [0, 0]; // AC_PID_2D.cpp:73
    this.target = [target[0], target[1]];
    if (this.resetFilterFlag) {
      this.resetFilterFlag = false;
      this.error = [target[0] - meas[0], target[1] - meas[1]]; this.derivative = [0, 0];
    } else {
      const last = this.error.slice();
      const a = calcLowpassAlpha(dt, this.filtE);
      this.error = [this.error[0] + (target[0] - meas[0] - this.error[0]) * a, this.error[1] + (target[1] - meas[1] - this.error[1]) * a];
      if (isPositive(dt)) {
        const ad = calcLowpassAlpha(dt, this.filtD);
        this.derivative = [0, 1].map((k) => this.derivative[k] + ((this.error[k] - last[k]) / dt - this.derivative[k]) * ad);
      }
    }
    this.updateI(dt, limit);
    const X = this.infoX, Y = this.infoY;
    this.slewCalc.update([X.P + X.D, Y.P + Y.D], dt);
    X.slew_rate = Y.slew_rate = this.slewCalc.getSlewRate();
    for (const [I, k] of [[X, 0], [Y, 1]]) {
      I.target = this.target[k]; I.actual = meas[k]; I.error = this.error[k]; I.P = this.error[k] * this.kp; I.I = this.integrator[k]; I.D = this.derivative[k] * this.kd; I.FF = this.target[k] * this.kff;
    }
    return [0, 1].map((k) => this.error[k] * this.kp + this.integrator[k] + this.derivative[k] * this.kd + this.target[k] * this.kff);
  }
  // update_i (AC_PID_2D.cpp:130): add the increment, then if it points along `limit` stop the
  // integrator from growing in length (it may still shrink or rotate)
  updateI(dt, limit) {
    this.infoX.limit = this.infoY.limit = false;
    const dI = [this.error[0] * this.ki * dt, this.error[1] * this.ki * dt];
    const len0 = Math.hypot(this.integrator[0], this.integrator[1]);
    this.integrator = [this.integrator[0] + dI[0], this.integrator[1] + dI[1]];
    if (isPositive(dI[0] * limit[0] + dI[1] * limit[1]) && limitLength2(this.integrator, len0)) this.infoX.limit = this.infoY.limit = true;
    limitLength2(this.integrator, this.kimax);
  }
  getP() { return [this.error[0] * this.kp, this.error[1] * this.kp]; }
  getI() { return this.integrator; }
  getD() { return [this.derivative[0] * this.kd, this.derivative[1] * this.kd]; }
  getFF() { this.infoX.FF = this.target[0] * this.kff; this.infoY.FF = this.target[1] * this.kff; return [this.infoX.FF, this.infoY.FF]; }
  resetFilter() { this.resetFilterFlag = true; }
  resetI() { this.integrator = [0, 0]; }
  setIntegrator(v) { this.integrator = [v[0], v[1]]; limitLength2(this.integrator, this.kimax); }
  setIntegratorError(error, i) { this.setIntegrator([i[0] - error[0] * this.kp, i[1] - error[1] * this.kp]); }
  setIntegratorTM(target, measurement, i) { this.setIntegratorError([target[0] - measurement[0], target[1] - measurement[1]], i); }
}
