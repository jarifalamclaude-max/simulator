// Port of AC_AttitudeControl + AC_AttitudeControl_Sub (ArduSub 4.5.7).
// Quaternion attitude controller with input shaping, angle P (sqrt controller),
// thrust/heading error split and body-rate PIDs driving the motor mixer, plus the
// throttle-mix machinery, angle boost, control monitor and the reset/EKF-reset helpers.
// setParams(P) updates gains live; it never resets integrators or targets.

import { AC_PID, AC_P } from './pid.js';
import { sqrtController, invSqrtController, filterBankFromParams } from './control.js';
import {
  qmul, qconj, qnormalize, qrotate, qFromEuler, qToEuler, qFromAxisAngleVec, qFromAxisAngle, qToAxisAngle,
  wrapPI, wrap2PI, constrainFloat, isZero, isPositive, isNegative, vcross, vdot, vlen, DEG, RAD,
} from '../core/math.js';

const THRUST_ERROR_ANGLE = 30 * DEG;
const YAW_MAX_ERROR_ANGLE = 45 * DEG;
const ACCEL_RP_MIN = 40 * DEG, ACCEL_RP_MAX = 720 * DEG;
const ACCEL_Y_MIN = 10 * DEG, ACCEL_Y_MAX = 120 * DEG;
const ANGLE_LIMIT_THROTTLE_MAX = 0.8;
const ANGLE_LIMIT_MIN_DEG = 10;
const RATE_RELAX_TC = 0.16;
const RATE_RP_CONTROLLER_OUT_MAX = 1, RATE_YAW_CONTROLLER_OUT_MAX = 1;
const THR_MIX_DEFAULT = 0.5, MIX_MIN_DEFAULT = 0.1, MIX_MAX_DEFAULT = 0.5, MIX_MIN_LIMIT = 0.5, MIX_MAN_LIMIT = 4, MIX_MAX = 5;
const MAX_YAW_ERROR = 5 * DEG; // AC_AttitudeControl_Sub.h:26
export const HEADING_MODE = { RATE_ONLY: 0, ANGLE_ONLY: 1, ANGLE_AND_RATE: 2 };

// ArduSub defaults (AC_AttitudeControl_Sub.h / AC_AttitudeControl.cpp) for P.X ?? default
const RATE_DEF = {
  RLL: { P: 0.135, I: 0.09, D: 0.0036, IMAX: 0.444, FLTT: 30, FLTE: 0, FLTD: 30 },
  PIT: { P: 0.135, I: 0.09, D: 0.0036, IMAX: 0.444, FLTT: 30, FLTE: 0, FLTD: 30 },
  YAW: { P: 0.18, I: 0.018, D: 0, IMAX: 0.222, FLTT: 5, FLTE: 0, FLTD: 5 },
};

export class AttitudeControlSub {
  constructor(ahrs, motors) {
    this.ahrs = ahrs;
    this.motors = motors;
    this.dt = 0.0025;
    this.pidRoll = new AC_PID();
    this.pidPitch = new AC_PID();
    this.pidYaw = new AC_PID();
    this.pAngleRoll = new AC_P(6); this.pAnglePitch = new AC_P(6); this.pAngleYaw = new AC_P(6);
    this.anglePScale = [1, 1, 1]; this.anglePScaleUsed = [1, 1, 1];
    this.useSqrt = true;
    this.rateRpTc = 0;
    this.rateYTc = 0;
    this.attitudeTarget = [1, 0, 0, 0];
    this.eulerAngleTarget = [0, 0, 0];
    this.eulerRateTarget = [0, 0, 0];
    this.angVelTarget = [0, 0, 0];
    this.angVelBody = [0, 0, 0];
    this.sysidAngVelBody = [0, 0, 0];
    this.actuatorSysid = [0, 0, 0];
    this.attitudeAngError = [1, 0, 0, 0];
    this.thrustAngle = 0;
    this.thrustErrorAngle = 0;
    this.feedforwardScalar = 1;
    this.althold_lean_angle_max = 0;
    this.throttleIn = 0;
    this.angleBoostV = 0;
    this.throttleRpyMix = THR_MIX_DEFAULT; this.throttleRpyMixDesired = THR_MIX_DEFAULT;
    this.controlMonitor = { rms_roll_P: 0, rms_roll_D: 0, rms_pitch_P: 0, rms_pitch_D: 0, rms_yaw: 0 };
    this.ekfYawResetMs = 0;
    this.filterBank = [];
    this.p = {
      accelRollMax: 110000, accelPitchMax: 110000, accelYawMax: 110000, rateFF: true, inputTc: 0.15, slewYaw: 6000,
      angVelRollMax: 0, angVelPitchMax: 0, angVelYawMax: 0, angleLimitTc: 1, angleBoost: 1, angleMaxCd: 4500,
      thrMixMin: MIX_MIN_DEFAULT, thrMixMax: MIX_MAX_DEFAULT, thrMixMan: 0.1,
    };
  }

  setParams(P) {
    const rp = (ax) => {
      const d = RATE_DEF[ax], k = (n) => P[`ATC_RAT_${ax}_${n}`];
      return {
        p: k('P') ?? d.P, i: k('I') ?? d.I, d: k('D') ?? d.D, ff: k('FF') ?? 0, imax: k('IMAX') ?? d.IMAX,
        fltt: k('FLTT') ?? d.FLTT, flte: k('FLTE') ?? d.FLTE, fltd: k('FLTD') ?? d.FLTD, smax: k('SMAX') ?? 0,
        pdmx: k('PDMX') ?? 0, dff: k('D_FF') ?? 0, ntf: k('NTF') ?? 0, nef: k('NEF') ?? 0,
      };
    };
    this.pidRoll.setGains(rp('RLL'));
    this.pidPitch.setGains(rp('PIT'));
    this.pidYaw.setGains(rp('YAW'));
    this.pAngleRoll.kp = P.ATC_ANG_RLL_P ?? 6; this.pAnglePitch.kp = P.ATC_ANG_PIT_P ?? 6; this.pAngleYaw.kp = P.ATC_ANG_YAW_P ?? 6;
    this.p = {
      accelRollMax: P.ATC_ACCEL_R_MAX ?? 110000, accelPitchMax: P.ATC_ACCEL_P_MAX ?? 110000, accelYawMax: P.ATC_ACCEL_Y_MAX ?? 110000,
      rateFF: !!(P.ATC_RATE_FF_ENAB ?? 1), inputTc: P.ATC_INPUT_TC ?? 0.15, slewYaw: P.ATC_SLEW_YAW ?? 6000,
      angVelRollMax: P.ATC_RATE_R_MAX ?? 0, angVelPitchMax: P.ATC_RATE_P_MAX ?? 0, angVelYawMax: P.ATC_RATE_Y_MAX ?? 0,
      angleLimitTc: P.ATC_ANG_LIM_TC ?? 1, angleBoost: P.ATC_ANGLE_BOOST ?? 1, angleMaxCd: P.ANGLE_MAX ?? 4500,
      thrMixMin: P.ATC_THR_MIX_MIN ?? MIX_MIN_DEFAULT, thrMixMax: P.ATC_THR_MIX_MAX ?? MIX_MAX_DEFAULT, thrMixMan: P.ATC_THR_MIX_MAN ?? 0.1,
    };
    this.filterBank = filterBankFromParams(P);
    if (isPositive(this.dt)) this.setNotchSampleRate(1 / this.dt);
  }
  configure(P) { this.setParams(P); }
  // Sub::one_hz_loop -> attitude_control.set_notch_sample_rate(filtered loop rate)
  setNotchSampleRate(rate) { for (const pid of this.pids()) pid.setNotchSampleRate(rate, this.filterBank); }
  pids() { return [this.pidRoll, this.pidPitch, this.pidYaw]; }

  // ---- angle P (AC_P) — `angP` kept as a read/write view for callers ----
  get angP() { return [this.pAngleRoll.kp, this.pAnglePitch.kp, this.pAngleYaw.kp]; }
  set angP(v) { this.pAngleRoll.kp = v[0]; this.pAnglePitch.kp = v[1]; this.pAngleYaw.kp = v[2]; }
  getAngleRollP() { return this.pAngleRoll; }
  getAnglePitchP() { return this.pAnglePitch; }
  getAngleYawP() { return this.pAngleYaw; }
  getRateRollPid() { return this.pidRoll; }
  getRatePitchPid() { return this.pidPitch; }
  getRateYawPid() { return this.pidYaw; }
  // autotune: scale factors consumed (and reset to 1) by the next update_ang_vel_target_from_att_error
  setAnglePScale(s) { this.anglePScale = s.slice(); }
  setAnglePScaleMult(s) { this.anglePScale = this.anglePScale.map((v, k) => v * s[k]); }
  getLastAnglePScale() { return this.anglePScaleUsed; }
  setUseSqrtController(v) { this.useSqrt = !!v; }
  useSqrtController(v) { this.useSqrt = !!v; }

  // ---- accessors (ArduPilot units) ----
  setDt(dt) { this.dt = dt; }
  getDt() { return this.dt; }
  getAccelRollMaxCdss() { return this.p.accelRollMax; }
  getAccelPitchMaxCdss() { return this.p.accelPitchMax; }
  getAccelYawMaxCdss() { return this.p.accelYawMax; }
  setAccelRollMaxCdss(v) { this.p.accelRollMax = v; }
  setAccelPitchMaxCdss(v) { this.p.accelPitchMax = v; }
  setAccelYawMaxCdss(v) { this.p.accelYawMax = v; }
  accelRollMaxRadss() { return (this.p.accelRollMax * 0.01) * DEG; }
  accelPitchMaxRadss() { return (this.p.accelPitchMax * 0.01) * DEG; }
  accelYawMaxRadss() { return (this.p.accelYawMax * 0.01) * DEG; }
  getAccelRollMaxRadss() { return this.accelRollMaxRadss(); }
  getAccelPitchMaxRadss() { return this.accelPitchMaxRadss(); }
  getAccelYawMaxRadss() { return this.accelYawMaxRadss(); }
  angVelRollMaxRads() { return this.p.angVelRollMax * DEG; }
  angVelPitchMaxRads() { return this.p.angVelPitchMax * DEG; }
  angVelYawMaxRads() { return this.p.angVelYawMax * DEG; }
  getAngVelRollMaxRads() { return this.angVelRollMaxRads(); }
  getAngVelPitchMaxRads() { return this.angVelPitchMaxRads(); }
  getAngVelYawMaxRads() { return this.angVelYawMaxRads(); }
  getSlewYawMaxDegs() { return !isPositive(this.p.angVelYawMax) ? this.p.slewYaw * 0.01 : Math.min(this.p.angVelYawMax, this.p.slewYaw * 0.01); }
  slewYawMaxRads() { return this.getSlewYawMaxDegs() * DEG; }
  getInputTc() { return this.p.inputTc; }
  setInputTc(v) { this.p.inputTc = constrainFloat(v, 0, 1); }
  setRollPitchRateTc(v) { this.rateRpTc = v; }
  setYawRateTc(v) { this.rateYTc = v; }
  bfFeedforward(v) { this.p.rateFF = !!v; }
  getBfFeedforward() { return !!this.p.rateFF; }
  // accel_limiting(false) zeroes the ATC_ACCEL_*_MAX limits; true restores the stored values
  accelLimiting(enable) {
    if (enable) {
      if (this._accelSaved) { const s = this._accelSaved; for (const k of ['accelRollMax', 'accelPitchMax', 'accelYawMax']) if (isZero(this.p[k])) this.p[k] = s[k]; }
    } else { this._accelSaved = { accelRollMax: this.p.accelRollMax || this._accelSaved?.accelRollMax, accelPitchMax: this.p.accelPitchMax || this._accelSaved?.accelPitchMax, accelYawMax: this.p.accelYawMax || this._accelSaved?.accelYawMax }; this.p.accelRollMax = this.p.accelPitchMax = this.p.accelYawMax = 0; }
  }
  getAttTargetEulerCd() { return this.eulerAngleTarget.map((v) => v * RAD * 100); }
  getAttTargetEulerRad() { return this.eulerAngleTarget; }
  getAttitudeTargetQuat() { return this.attitudeTarget.slice(); }
  getAttitudeTargetAngVel() { return this.angVelTarget; }
  getRateEfTargets() { return this.eulerRateTarget; }
  rateBfTargets() { return [0, 1, 2].map((k) => this.angVelBody[k] + this.sysidAngVelBody[k]); }
  getAttErrorAngleDeg() { return this.thrustErrorAngle * RAD; }
  leanAngleDeg() { return this.thrustAngle * RAD; }
  leanAngleMaxCd() { return this.p.angleMaxCd; }
  getThrottleIn() { return this.throttleIn; }
  angleBoost() { return this.angleBoostV; }
  getAltholdLeanAngleMaxCd() { return Math.max(this.althold_lean_angle_max * RAD, ANGLE_LIMIT_MIN_DEG) * 100; }
  rateBfRollTarget(cds) { this.angVelBody[0] = cds * 0.01 * DEG; }
  rateBfPitchTarget(cds) { this.angVelBody[1] = cds * 0.01 * DEG; }
  rateBfYawTarget(cds) { this.angVelBody[2] = cds * 0.01 * DEG; }
  rateBfRollSysid(r) { this.sysidAngVelBody[0] = r; }
  rateBfPitchSysid(r) { this.sysidAngVelBody[1] = r; }
  rateBfYawSysid(r) { this.sysidAngVelBody[2] = r; }
  actuatorRollSysid(c) { this.actuatorSysid[0] = c; }
  actuatorPitchSysid(c) { this.actuatorSysid[1] = c; }
  actuatorYawSysid(c) { this.actuatorSysid[2] = c; }
  getRpySrate() { return this.pids().map((p) => p.info.slew_rate); }

  // ---- resets ----
  relaxAttitudeControllers() {
    this.attitudeTarget = this.ahrs.quat.slice();
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.attitudeAngError = [1, 0, 0, 0];
    this.angVelTarget = this.ahrs.gyro.slice();
    // ang_vel_to_euler_rate leaves its output untouched at +/-90 deg pitch (AC_AttitudeControl.cpp:1001)
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    this.angVelBody = this.ahrs.gyro.slice();
    this.thrustErrorAngle = 0;
    for (const pid of this.pids()) pid.resetFilter();
    this.resetRateControllerITerms();
  }
  resetRateControllerITerms() { for (const pid of this.pids()) pid.resetI(); }
  resetRateControllerITermsSmoothly() { for (const pid of this.pids()) pid.relaxIntegrator(0, this.dt, RATE_RELAX_TC); }
  resetTargetAndRate(resetRate = true) {
    this.attitudeTarget = this.ahrs.quat.slice();
    if (resetRate) { this.angVelTarget = [0, 0, 0]; this.eulerAngleTarget = [0, 0, 0]; }
  }
  resetYawTargetAndRate(resetRate = true) {
    const shift = this.ahrs.yaw - this.eulerAngleTarget[2];
    this.attitudeTarget = qmul(qFromAxisAngleVec([0, 0, shift]), this.attitudeTarget);
    if (resetRate) { this.eulerRateTarget[2] = 0; this.angVelTarget = eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget); }
  }
  // shift the target to keep the current error after an EKF reset
  inertialFrameReset() {
    this.attitudeTarget = qmul(this.ahrs.quat, this.attitudeAngError);
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
  }
  // Sub::check_ekf_yaw_reset (ArduSub/Attitude.cpp:44) — needs ahrs.getLastYawResetAngle() -> ms
  checkEkfYawReset() {
    const ms = this.ahrs.getLastYawResetAngle?.() ?? 0;
    if (ms !== this.ekfYawResetMs) { this.inertialFrameReset(); this.ekfYawResetMs = ms; return true; }
    return false;
  }

  // ---- input functions ----
  eulerAccel() { return eulerAccelLimit(this.eulerAngleTarget, [this.accelRollMaxRadss(), this.accelPitchMaxRadss(), this.accelYawMaxRadss()]); }
  limitAngVel(v) { return angVelLimit(v, this.angVelRollMaxRads(), this.angVelPitchMaxRads(), this.angVelYawMaxRads()); }

  // input_quaternion: attDesired is updated in place (returned) by the integral of angVelTarget
  inputQuaternion(attDesired, angVelTargetIn) {
    const errQ = qmul(qconj(this.attitudeTarget), attDesired);
    const e = qToAxisAngle(errQ);
    const avt = this.limitAngVel(angVelTargetIn.slice());
    if (this.p.rateFF) {
      this.angVelTarget[0] = inputShapingAngle(wrapPI(e[0]), this.p.inputTc, this.accelRollMaxRadss(), this.angVelTarget[0], avt[0], this.angVelRollMaxRads(), this.dt);
      this.angVelTarget[1] = inputShapingAngle(wrapPI(e[1]), this.p.inputTc, this.accelPitchMaxRadss(), this.angVelTarget[1], avt[1], this.angVelPitchMaxRads(), this.dt);
      this.angVelTarget[2] = inputShapingAngle(wrapPI(e[2]), this.p.inputTc, this.accelYawMaxRadss(), this.angVelTarget[2], avt[2], this.angVelYawMaxRads(), this.dt);
    } else {
      this.attitudeTarget = attDesired.slice();
      this.angVelTarget = avt.slice();
    }
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    const upd = qFromAxisAngleVec([avt[0] * this.dt, avt[1] * this.dt, avt[2] * this.dt]);
    const next = qnormalize(qmul(attDesired, upd));
    for (let k = 0; k < 4; k++) attDesired[k] = next[k];
    this.attitudeControllerRunQuat();
    return attDesired;
  }

  inputEulerAngleRollPitchEulerRateYaw(rollCd, pitchCd, yawRateCds) {
    const roll = rollCd * 0.01 * DEG, pitch = pitchCd * 0.01 * DEG, yawRate = yawRateCds * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    if (this.p.rateFF) {
      const ea = this.eulerAccel();
      this.eulerRateTarget[0] = inputShapingAngle(wrapPI(roll - this.eulerAngleTarget[0]), this.p.inputTc, ea[0], this.eulerRateTarget[0], 0, 0, this.dt);
      this.eulerRateTarget[1] = inputShapingAngle(wrapPI(pitch - this.eulerAngleTarget[1]), this.p.inputTc, ea[1], this.eulerRateTarget[1], 0, 0, this.dt);
      this.eulerRateTarget[2] = inputShapingAngVel(this.eulerRateTarget[2], yawRate, ea[2], this.dt, this.rateYTc);
      this.angVelTarget = this.limitAngVel(eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget));
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      this.eulerAngleTarget[0] = roll;
      this.eulerAngleTarget[1] = pitch;
      this.eulerAngleTarget[2] += yawRate * this.dt;
      this.attitudeTarget = qFromEuler(...this.eulerAngleTarget);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  inputEulerAngleRollPitchYaw(rollCd, pitchCd, yawCd, slewYaw) {
    const roll = rollCd * 0.01 * DEG, pitch = pitchCd * 0.01 * DEG, yaw = yawCd * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    const slewMax = this.slewYawMaxRads();
    if (this.p.rateFF) {
      const ea = this.eulerAccel();
      this.eulerRateTarget[0] = inputShapingAngle(wrapPI(roll - this.eulerAngleTarget[0]), this.p.inputTc, ea[0], this.eulerRateTarget[0], 0, 0, this.dt);
      this.eulerRateTarget[1] = inputShapingAngle(wrapPI(pitch - this.eulerAngleTarget[1]), this.p.inputTc, ea[1], this.eulerRateTarget[1], 0, 0, this.dt);
      this.eulerRateTarget[2] = inputShapingAngle(wrapPI(yaw - this.eulerAngleTarget[2]), this.p.inputTc, ea[2], this.eulerRateTarget[2], 0, 0, this.dt);
      if (slewYaw) this.eulerRateTarget[2] = constrainFloat(this.eulerRateTarget[2], -slewMax, slewMax);
      this.angVelTarget = this.limitAngVel(eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget));
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      this.eulerAngleTarget[0] = roll;
      this.eulerAngleTarget[1] = pitch;
      if (slewYaw) {
        const err = constrainFloat(wrapPI(yaw - this.eulerAngleTarget[2]), -slewMax * this.dt, slewMax * this.dt);
        this.eulerAngleTarget[2] = wrapPI(err + this.eulerAngleTarget[2]);
      } else this.eulerAngleTarget[2] = yaw;
      this.attitudeTarget = qFromEuler(...this.eulerAngleTarget);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  // AC_AttitudeControl_Sub::input_euler_angle_roll_pitch_slew_yaw (used by GUIDED, mode_guided.cpp:506)
  inputEulerAngleRollPitchSlewYaw(rollCd, pitchCd, yawCd, targetYawRate) {
    const yaw = wrapPI(yawCd * 0.01 * DEG);
    const yawError = wrapPI(yaw - this.ahrs.yaw);
    const direction = yawError < 0 ? -1 : 1;
    targetYawRate *= direction;
    if (Math.abs(yawError) > MAX_YAW_ERROR) this.inputEulerAngleRollPitchEulerRateYaw(rollCd, pitchCd, targetYawRate);
    else this.inputEulerAngleRollPitchYaw(rollCd, pitchCd, yawCd, true);
  }

  inputEulerRateRollPitchYaw(rollRateCds, pitchRateCds, yawRateCds) {
    const rr = rollRateCds * 0.01 * DEG, pr = pitchRateCds * 0.01 * DEG, yr = yawRateCds * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    if (this.p.rateFF) {
      const ea = this.eulerAccel();
      this.eulerRateTarget[0] = inputShapingAngVel(this.eulerRateTarget[0], rr, ea[0], this.dt, this.rateRpTc);
      this.eulerRateTarget[1] = inputShapingAngVel(this.eulerRateTarget[1], pr, ea[1], this.dt, this.rateRpTc);
      this.eulerRateTarget[2] = inputShapingAngVel(this.eulerRateTarget[2], yr, ea[2], this.dt, this.rateYTc);
      this.angVelTarget = eulerRateToAngVel(this.eulerAngleTarget, this.eulerRateTarget);
    } else {
      // pitch held inside +/-85 deg to avoid the gimbal-lock discontinuity
      this.eulerAngleTarget[0] = wrapPI(this.eulerAngleTarget[0] + rr * this.dt);
      this.eulerAngleTarget[1] = constrainFloat(this.eulerAngleTarget[1] + pr * this.dt, -85 * DEG, 85 * DEG);
      this.eulerAngleTarget[2] = wrap2PI(this.eulerAngleTarget[2] + yr * this.dt);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
      this.attitudeTarget = qFromEuler(...this.eulerAngleTarget);
    }
    this.attitudeControllerRunQuat();
  }

  inputRateBfRollPitchYaw(rollCds, pitchCds, yawCds) {
    const r = rollCds * 0.01 * DEG, p = pitchCds * 0.01 * DEG, y = yawCds * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    if (this.p.rateFF) {
      this.angVelTarget[0] = inputShapingAngVel(this.angVelTarget[0], r, this.accelRollMaxRadss(), this.dt, this.rateRpTc);
      this.angVelTarget[1] = inputShapingAngVel(this.angVelTarget[1], p, this.accelPitchMaxRadss(), this.dt, this.rateRpTc);
      this.angVelTarget[2] = inputShapingAngVel(this.angVelTarget[2], y, this.accelYawMaxRadss(), this.dt, this.rateYTc);
      this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    } else {
      const upd = qFromAxisAngleVec([r * this.dt, p * this.dt, y * this.dt]);
      this.attitudeTarget = qnormalize(qmul(this.attitudeTarget, upd));
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.attitudeControllerRunQuat();
  }

  // rate loop only, no attitude stabilisation
  inputRateBfRollPitchYaw2(rollCds, pitchCds, yawCds) {
    const r = rollCds * 0.01 * DEG, p = pitchCds * 0.01 * DEG, y = yawCds * 0.01 * DEG;
    this.angVelTarget[0] = inputShapingAngVel(this.angVelTarget[0], r, this.accelRollMaxRadss(), this.dt, this.rateRpTc);
    this.angVelTarget[1] = inputShapingAngVel(this.angVelTarget[1], p, this.accelPitchMaxRadss(), this.dt, this.rateRpTc);
    this.angVelTarget[2] = inputShapingAngVel(this.angVelTarget[2], y, this.accelYawMaxRadss(), this.dt, this.rateYTc);
    this.attitudeTarget = this.ahrs.quat.slice();
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    this.angVelBody = this.angVelTarget.slice();
  }

  // rate loop with integrated rate error (clamped to 30 deg)
  inputRateBfRollPitchYaw3(rollCds, pitchCds, yawCds) {
    const r = rollCds * 0.01 * DEG, p = pitchCds * 0.01 * DEG, y = yawCds * 0.01 * DEG;
    let e = qToAxisAngle(this.attitudeAngError);
    const mag = vlen(e);
    if (mag > THRUST_ERROR_ANGLE) { e = e.map((v) => (v * THRUST_ERROR_ANGLE) / mag); this.attitudeAngError = qFromAxisAngleVec(e); }
    const g = this.ahrs.gyroLatest ?? this.ahrs.gyro;
    const upd = qFromAxisAngleVec([0, 1, 2].map((k) => (this.angVelTarget[k] - g[k]) * this.dt));
    this.attitudeAngError = qmul(upd, this.attitudeAngError);
    this.angVelTarget[0] = inputShapingAngVel(this.angVelTarget[0], r, this.accelRollMaxRadss(), this.dt, this.rateRpTc);
    this.angVelTarget[1] = inputShapingAngVel(this.angVelTarget[1], p, this.accelPitchMaxRadss(), this.dt, this.rateRpTc);
    this.angVelTarget[2] = inputShapingAngVel(this.angVelTarget[2], y, this.accelYawMaxRadss(), this.dt, this.rateYTc);
    this.attitudeTarget = qmul(this.ahrs.quat, this.attitudeAngError);
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    e = qToAxisAngle(this.attitudeAngError);
    const avb = this.updateAngVelTargetFromAttError(e);
    this.angVelBody = [0, 1, 2].map((k) => avb[k] + this.angVelTarget[k]);
    this.attitudeAngError = qnormalize(this.attitudeAngError);
  }

  inputAngleStepBfRollPitchYaw(rollStepCd, pitchStepCd, yawStepCd) {
    const upd = qFromAxisAngleVec([rollStepCd * 0.01 * DEG, pitchStepCd * 0.01 * DEG, yawStepCd * 0.01 * DEG]);
    this.attitudeTarget = qnormalize(qmul(this.attitudeTarget, upd));
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    this.eulerRateTarget = [0, 0, 0];
    this.angVelTarget = [0, 0, 0];
    this.attitudeControllerRunQuat();
  }

  inputThrustVectorRateHeading(thrustVector, headingRateCds, slewYaw = true) {
    let headingRate = headingRateCds * 0.01 * DEG;
    if (slewYaw) { const m = this.slewYawMaxRads(); headingRate = constrainFloat(headingRate, -m, m); }
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    const tq = this.attitudeFromThrustVector(thrustVector, 0);
    const r = thrustVectorRotationAngles(tq, this.attitudeTarget);
    if (this.p.rateFF) {
      this.angVelTarget[0] = inputShapingAngle(r.attitudeError[0], this.p.inputTc, this.accelRollMaxRadss(), this.angVelTarget[0], 0, 0, this.dt);
      this.angVelTarget[1] = inputShapingAngle(r.attitudeError[1], this.p.inputTc, this.accelPitchMaxRadss(), this.angVelTarget[1], 0, 0, this.dt);
      this.angVelTarget[2] = inputShapingAngVel(this.angVelTarget[2], headingRate, this.accelYawMaxRadss(), this.dt, this.rateYTc);
      this.limitAngVel(this.angVelTarget);
    } else {
      const yq = qFromAxisAngleVec([0, 0, headingRate * this.dt]);
      this.attitudeTarget = qmul(qmul(this.attitudeTarget, r.correction), yq);
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    this.attitudeControllerRunQuat();
  }

  inputThrustVectorHeading(thrustVector, headingAngleCd, headingRateCds = 0) {
    const slewMax = this.slewYawMaxRads();
    const headingRate = constrainFloat(headingRateCds * 0.01 * DEG, -slewMax, slewMax);
    const heading = headingAngleCd * 0.01 * DEG;
    this.eulerAngleTarget = qToEuler(this.attitudeTarget);
    const dq = this.attitudeFromThrustVector(thrustVector, heading);
    if (this.p.rateFF) {
      const r = thrustVectorRotationAngles(dq, this.attitudeTarget);
      const e = r.attitudeError;
      this.angVelTarget[0] = inputShapingAngle(e[0], this.p.inputTc, this.accelRollMaxRadss(), this.angVelTarget[0], 0, 0, this.dt);
      this.angVelTarget[1] = inputShapingAngle(e[1], this.p.inputTc, this.accelPitchMaxRadss(), this.angVelTarget[1], 0, 0, this.dt);
      this.angVelTarget[2] = inputShapingAngle(e[2], this.p.inputTc, this.accelYawMaxRadss(), this.angVelTarget[2], headingRate, slewMax, this.dt);
      angVelLimit(this.angVelTarget, this.angVelRollMaxRads(), this.angVelPitchMaxRads(), slewMax);
    } else {
      this.attitudeTarget = dq;
      this.eulerRateTarget = [0, 0, 0];
      this.angVelTarget = [0, 0, 0];
    }
    this.eulerRateTarget = angVelToEulerRate(this.eulerAngleTarget, this.angVelTarget) || this.eulerRateTarget;
    this.attitudeControllerRunQuat();
  }
  // input_thrust_vector_heading(thrust, HeadingCommand{heading_mode, yaw_angle_cd, yaw_rate_cds})
  inputThrustVectorHeadingCommand(thrustVector, h) {
    if (h.headingMode === HEADING_MODE.RATE_ONLY) this.inputThrustVectorRateHeading(thrustVector, h.yawRateCds ?? 0);
    else if (h.headingMode === HEADING_MODE.ANGLE_ONLY) this.inputThrustVectorHeading(thrustVector, h.yawAngleCd ?? 0, 0);
    else this.inputThrustVectorHeading(thrustVector, h.yawAngleCd ?? 0, h.yawRateCds ?? 0);
  }

  attitudeFromThrustVector(thrustVector, headingAngle) {
    const up = [0, 0, -1];
    let tv = thrustVector;
    const l2 = vdot(tv, tv);
    tv = isZero(l2) ? up : tv.map((v) => v / Math.sqrt(l2));
    let cross = vcross(up, tv);
    const angle = Math.acos(constrainFloat(vdot(up, tv), -1, 1));
    const cl = vlen(cross);
    cross = isZero(cl) || isZero(angle) ? up : cross.map((v) => v / cl);
    return qmul(qFromAxisAngle(cross, angle), qFromAxisAngle([0, 0, 1], headingAngle));
  }

  // input_shaping_rate_predictor: target_ang_vel [x,y] updated in place
  inputShapingRatePredictor(errorAngle, targetAngVel, dt) {
    if (this.p.rateFF) {
      targetAngVel[0] = inputShapingAngle(wrapPI(errorAngle[0]), this.p.inputTc, this.accelRollMaxRadss(), targetAngVel[0], 0, 0, dt);
      targetAngVel[1] = inputShapingAngle(wrapPI(errorAngle[1]), this.p.inputTc, this.accelPitchMaxRadss(), targetAngVel[1], 0, 0, dt);
    } else {
      targetAngVel[0] = this.pAngleRoll.kp * this.anglePScale[0] * wrapPI(errorAngle[0]);
      targetAngVel[1] = this.pAnglePitch.kp * this.anglePScale[1] * wrapPI(errorAngle[1]);
    }
    const v = angVelLimit([targetAngVel[0], targetAngVel[1], 0], this.angVelRollMaxRads(), this.angVelPitchMaxRads(), 0);
    targetAngVel[0] = v[0]; targetAngVel[1] = v[1];
    return targetAngVel;
  }

  // ---- core ----
  attitudeControllerRunQuat() {
    const body = this.ahrs.quat;
    const { attitudeError, thrustAngle, thrustErrorAngle, target } = this.thrustHeadingRotationAngles(this.attitudeTarget, body);
    this.attitudeTarget = target;
    this.thrustAngle = thrustAngle;
    this.thrustErrorAngle = thrustErrorAngle;
    let avb = this.limitAngVel(this.updateAngVelTargetFromAttError(attitudeError));
    const rotTargetToBody = qmul(qconj(body), this.attitudeTarget);
    const ff = qrotate(rotTargetToBody, this.angVelTarget);
    this.feedforwardScalar = 1;
    const gyroZ = this.ahrs.gyro[2];
    if (thrustErrorAngle > THRUST_ERROR_ANGLE * 2) {
      avb[2] = gyroZ;
    } else if (thrustErrorAngle > THRUST_ERROR_ANGLE) {
      this.feedforwardScalar = 1 - (thrustErrorAngle - THRUST_ERROR_ANGLE) / THRUST_ERROR_ANGLE;
      avb[0] += ff[0] * this.feedforwardScalar;
      avb[1] += ff[1] * this.feedforwardScalar;
      avb[2] += ff[2];
      avb[2] = gyroZ * (1 - this.feedforwardScalar) + avb[2] * this.feedforwardScalar;
    } else {
      avb = [avb[0] + ff[0], avb[1] + ff[1], avb[2] + ff[2]];
    }
    this.angVelBody = avb;
    if (this.p.rateFF) {
      const upd = qFromAxisAngleVec([this.angVelTarget[0] * this.dt, this.angVelTarget[1] * this.dt, this.angVelTarget[2] * this.dt]);
      this.attitudeTarget = qmul(this.attitudeTarget, upd);
    }
    this.attitudeTarget = qnormalize(this.attitudeTarget);
    this.attitudeAngError = qmul(qconj(body), this.attitudeTarget);
  }

  thrustVectorRotationAngles(target, body) { return thrustVectorRotationAngles(target, body); }

  thrustHeadingRotationAngles(target, body) {
    const r = thrustVectorRotationAngles(target, body);
    const err = r.attitudeError;
    let newTarget = target;
    const headingAccelMax = constrainFloat(this.accelYawMaxRadss() / 2, ACCEL_Y_MIN, ACCEL_Y_MAX);
    if (!isZero(this.pidYaw.kp)) {
      const headingErrorMax = Math.min(invSqrtController(1 / this.pidYaw.kp, this.pAngleYaw.kp, headingAccelMax), YAW_MAX_ERROR_ANGLE);
      if (!isZero(this.pAngleYaw.kp) && Math.abs(err[2]) > headingErrorMax) {
        err[2] = constrainFloat(wrapPI(err[2]), -headingErrorMax, headingErrorMax);
        const hq = qFromAxisAngleVec([0, 0, err[2]]);
        newTarget = qmul(qmul(body, r.correction), hq);
      }
    }
    return { attitudeError: err, thrustAngle: r.thrustAngle, thrustErrorAngle: r.thrustErrorAngle, target: newTarget };
  }

  updateAngVelTargetFromAttError(e) {
    const out = [0, 0, 0];
    const s = this.anglePScale;
    const kr = this.pAngleRoll.kp * s[0], kpch = this.pAnglePitch.kp * s[1], ky = this.pAngleYaw.kp * s[2];
    const ar = this.accelRollMaxRadss(), ap = this.accelPitchMaxRadss(), ay = this.accelYawMaxRadss();
    out[0] = this.useSqrt && !isZero(ar) ? sqrtController(e[0], kr, constrainFloat(ar / 2, ACCEL_RP_MIN, ACCEL_RP_MAX), this.dt) : kr * e[0];
    out[1] = this.useSqrt && !isZero(ap) ? sqrtController(e[1], kpch, constrainFloat(ap / 2, ACCEL_RP_MIN, ACCEL_RP_MAX), this.dt) : kpch * e[1];
    out[2] = this.useSqrt && !isZero(ay) ? sqrtController(e[2], ky, constrainFloat(ay / 2, ACCEL_Y_MIN, ACCEL_Y_MAX), this.dt) : ky * e[2];
    this.anglePScaleUsed = s.slice();
    this.anglePScale = [1, 1, 1];
    return out;
  }

  // AC_AttitudeControl_Sub::rate_controller_run
  rateControllerRun() {
    this.updateThrottleRpyMix();
    const g = this.ahrs.gyroLatest;
    const lim = this.motors.limit;
    this.motors.setRoll(this.pidRoll.updateAll(this.angVelBody[0], g[0], this.dt, lim.roll));
    this.motors.setPitch(this.pidPitch.updateAll(this.angVelBody[1], g[1], this.dt, lim.pitch));
    this.motors.setYaw(this.pidYaw.updateAll(this.angVelBody[2], g[2], this.dt, lim.yaw));
    this.controlMonitorUpdate();
  }

  // ---- throttle ----
  setThrottleOut(throttle, applyAngleBoost, filtCutoff) {
    this.throttleIn = throttle;
    this.updateAltholdLeanAngleMax(throttle);
    this.motors.setThrottleFilterCutoff(filtCutoff);
    this.motors.setThrottle(throttle);
    // (log only in 6DOF: AP_MotorsMulticopter.cpp:391 th_average_max)
    this.motors.setThrottleAvgMax?.(this.getThrottleAvgMax(Math.max(throttle, this.throttleIn)));
  }
  updateAltholdLeanAngleMax(throttle) {
    const thrMax = this.motors.throttleThrustMax;
    if (isZero(thrMax)) { this.althold_lean_angle_max = 0; return; }
    const target = Math.acos(constrainFloat(throttle / (ANGLE_LIMIT_THROTTLE_MAX * thrMax), 0, 1));
    this.althold_lean_angle_max += (this.dt / (this.dt + this.p.angleLimitTc)) * (target - this.althold_lean_angle_max);
  }
  // get_throttle_boosted (not applied by Sub's set_throttle_out; kept for parity / CTUN logging)
  getThrottleBoosted(throttleIn) {
    if (!this.p.angleBoost) { this.angleBoostV = 0; return throttleIn; }
    const cosTilt = Math.cos(this.ahrs.pitch ?? 0) * Math.cos(this.ahrs.roll ?? 0);
    const inverted = constrainFloat(2 * cosTilt, 0, 1);
    const boost = 1 / constrainFloat(cosTilt, 0.5, 1);
    const out = throttleIn * inverted * boost;
    this.angleBoostV = constrainFloat(out - throttleIn, -1, 1);
    return out;
  }
  getThrottleAvgMax(t) {
    t = constrainFloat(t, 0, 1);
    const hover = this.motors.getThrottleHover?.() ?? this.motors.throttleHover ?? 0.5;
    return Math.max(t, t * Math.max(0, 1 - this.throttleRpyMix) + hover * this.throttleRpyMix);
  }
  setThrottleMixMin() { this.throttleRpyMixDesired = this.p.thrMixMin; }
  setThrottleMixMan() { this.throttleRpyMixDesired = this.p.thrMixMan; }
  setThrottleMixMax() { this.throttleRpyMixDesired = this.p.thrMixMax; }
  setThrottleMixValue(v) { this.throttleRpyMixDesired = this.throttleRpyMix = v; }
  getThrottleMix() { return this.throttleRpyMix; }
  isThrottleMixMin() { return this.throttleRpyMix < 1.25 * this.p.thrMixMin; }
  updateThrottleRpyMix() {
    if (this.throttleRpyMix < this.throttleRpyMixDesired) this.throttleRpyMix += Math.min(2 * this.dt, this.throttleRpyMixDesired - this.throttleRpyMix);
    else if (this.throttleRpyMix > this.throttleRpyMixDesired) this.throttleRpyMix -= Math.min(0.5 * this.dt, this.throttleRpyMix - this.throttleRpyMixDesired);
    this.throttleRpyMix = constrainFloat(this.throttleRpyMix, 0.1, MIX_MAX);
  }
  // parameter_sanity_check: returns {name: value} for parameters the firmware would set_and_save
  parameterSanityCheck() {
    const p = this.p, out = {};
    if (p.thrMixMan < 0.1 || p.thrMixMan > MIX_MAN_LIMIT) out.ATC_THR_MIX_MAN = p.thrMixMan = constrainFloat(p.thrMixMan, 0.1, MIX_MAN_LIMIT);
    if (p.thrMixMin < 0.1 || p.thrMixMin > MIX_MIN_LIMIT) out.ATC_THR_MIX_MIN = p.thrMixMin = constrainFloat(p.thrMixMin, 0.1, MIX_MIN_LIMIT);
    if (p.thrMixMax < 0.5 || p.thrMixMax > MIX_MAX) out.ATC_THR_MIX_MAX = p.thrMixMax = constrainFloat(p.thrMixMax, 0.5, MIX_MAX);
    if (p.thrMixMin > p.thrMixMax) { out.ATC_THR_MIX_MIN = p.thrMixMin = MIX_MIN_DEFAULT; out.ATC_THR_MIX_MAX = p.thrMixMax = MIX_MAX_DEFAULT; }
    return out;
  }

  // ---- control monitor (ControlMonitor.cpp) ----
  controlMonitorUpdate() {
    const f = (v, k) => { this.controlMonitor[k] = 0.99 * this.controlMonitor[k] + 0.01 * v * v; };
    const r = this.pidRoll.info, p = this.pidPitch.info, y = this.pidYaw.info;
    f(r.P + r.FF, 'rms_roll_P'); f(r.D, 'rms_roll_D');
    f(p.P + p.FF, 'rms_pitch_P'); f(p.D, 'rms_pitch_D');
    f(y.P + y.D + y.FF, 'rms_yaw');
  }
  controlMonitorRmsOutputRoll() { const c = this.controlMonitor; return Math.sqrt(c.rms_roll_P + c.rms_roll_D); }
  controlMonitorRmsOutputRollP() { return Math.sqrt(this.controlMonitor.rms_roll_P); }
  controlMonitorRmsOutputRollD() { return Math.sqrt(this.controlMonitor.rms_roll_D); }
  controlMonitorRmsOutputPitch() { const c = this.controlMonitor; return Math.sqrt(c.rms_pitch_P + c.rms_pitch_D); }
  controlMonitorRmsOutputPitchP() { return Math.sqrt(this.controlMonitor.rms_pitch_P); }
  controlMonitorRmsOutputPitchD() { return Math.sqrt(this.controlMonitor.rms_pitch_D); }
  controlMonitorRmsOutputYaw() { return Math.sqrt(this.controlMonitor.rms_yaw); }
  // CTRL log record
  controlMonitorLog() { const c = this.controlMonitor; return { RMSRollP: Math.sqrt(c.rms_roll_P), RMSRollD: Math.sqrt(c.rms_roll_D), RMSPitchP: Math.sqrt(c.rms_pitch_P), RMSPitchD: Math.sqrt(c.rms_pitch_D), RMSYaw: Math.sqrt(c.rms_yaw) }; }

  // ---- autotune step sizes (AC_AttitudeControl.cpp:1076) ----
  maxRateStepBf(pid, outMax, angVelMaxDegs) {
    const a = Math.min(pid.getFiltEAlpha(this.dt), pid.getFiltDAlpha(this.dt)), ar = 1 - a;
    const hover = constrainFloat(this.motors.getThrottleHover?.() ?? 0.5, 0.1, 0.5);
    let rateMax = (2 * hover * outMax) / ((ar * ar * ar * a * pid.kd) / this.dt + pid.kp);
    if (isPositive(angVelMaxDegs)) rateMax = Math.min(rateMax, angVelMaxDegs * DEG);
    return rateMax;
  }
  maxRateStepBfRoll() { return this.maxRateStepBf(this.pidRoll, RATE_RP_CONTROLLER_OUT_MAX, this.p.angVelRollMax); }
  maxRateStepBfPitch() { return this.maxRateStepBf(this.pidPitch, RATE_RP_CONTROLLER_OUT_MAX, this.p.angVelPitchMax); }
  maxRateStepBfYaw() { return this.maxRateStepBf(this.pidYaw, RATE_YAW_CONTROLLER_OUT_MAX, this.p.angVelYawMax); }
  maxAngleStepBfRoll() { return this.maxRateStepBfRoll() / this.pAngleRoll.kp; }
  maxAngleStepBfPitch() { return this.maxRateStepBfPitch() / this.pAnglePitch.kp; }
  maxAngleStepBfYaw() { return this.maxRateStepBfYaw() / this.pAngleYaw.kp; }

  // AC_AttitudeControl::pre_arm_checks (AC_AttitudeControl.cpp:1121). Note: ArduSub 4.5.7's
  // AP_Arming_Sub does not call it; it is provided for parity.
  preArmChecks(prefix = 'ATC') {
    for (const [n, v] of [['ANG_PIT', this.pAnglePitch.kp], ['ANG_RLL', this.pAngleRoll.kp], ['ANG_YAW', this.pAngleYaw.kp]]) if (!isPositive(v)) return `${prefix}_${n}_P must be > 0`;
    for (const [n, pid] of [['RAT_RLL', this.pidRoll], ['RAT_PIT', this.pidPitch], ['RAT_YAW', this.pidYaw]]) {
      if (isPositive(pid.kff)) {
        if (isNegative(pid.kp)) return `${prefix}_${n}_P must be >= 0`;
        if (isNegative(pid.ki)) return `${prefix}_${n}_I must be >= 0`;
      } else {
        if (!isPositive(pid.kp)) return `${prefix}_${n}_P must be > 0`;
        if (!isPositive(pid.ki)) return `${prefix}_${n}_I must be > 0`;
      }
      if (isNegative(pid.kd)) return `${prefix}_${n}_D must be >= 0`;
    }
    return null;
  }
}

// ---- free functions (static in C++) ----
export function thrustVectorRotationAngles(target, body) {
  const up = [0, 0, -1];
  const tv = qrotate(target, up), bv = qrotate(body, up);
  const thrustAngle = Math.acos(constrainFloat(vdot(up, bv), -1, 1));
  let cross = vcross(bv, tv);
  const thrustErrorAngle = Math.acos(constrainFloat(vdot(bv, tv), -1, 1));
  const l = vlen(cross);
  if (isZero(l) || isZero(thrustErrorAngle)) cross = up;
  else cross = cross.map((v) => v / l);
  cross = qrotate(qconj(body), cross);
  const correction = qFromAxisAngle(cross, thrustErrorAngle);
  const rot = qToAxisAngle(correction);
  const headingQ = qmul(qmul(qconj(correction), qconj(body)), target);
  const rotH = qToAxisAngle(headingQ);
  return { correction, attitudeError: [rot[0], rot[1], rotH[2]], thrustAngle, thrustErrorAngle };
}
export function inputShapingAngle(errorAngle, inputTc, accelMax, targetAngVel, desiredAngVel = 0, maxAngVel = 0, dt) {
  if (dt === undefined) { dt = desiredAngVel; desiredAngVel = 0; maxAngVel = 0; } // 5-argument overload
  desiredAngVel += sqrtController(errorAngle, 1 / Math.max(inputTc, 0.01), accelMax, dt);
  if (isPositive(maxAngVel)) desiredAngVel = constrainFloat(desiredAngVel, -maxAngVel, maxAngVel);
  return inputShapingAngVel(targetAngVel, desiredAngVel, accelMax, dt, 0);
}
export function inputShapingAngVel(targetAngVel, desiredAngVel, accelMax, dt, inputTc) {
  if (isPositive(inputTc)) {
    const err = desiredAngVel - targetAngVel;
    const acc = sqrtController(err, 1 / Math.max(inputTc, 0.01), 0, dt);
    desiredAngVel = targetAngVel + acc * dt;
  }
  if (isPositive(accelMax)) {
    const d = accelMax * dt;
    return constrainFloat(desiredAngVel, targetAngVel - d, targetAngVel + d);
  }
  return desiredAngVel;
}
export function angVelLimit(v, rollMax, pitchMax, yawMax) {
  if (isZero(rollMax) || isZero(pitchMax)) {
    if (!isZero(rollMax)) v[0] = constrainFloat(v[0], -rollMax, rollMax);
    if (!isZero(pitchMax)) v[1] = constrainFloat(v[1], -pitchMax, pitchMax);
  } else {
    const tx = v[0] / rollMax, ty = v[1] / pitchMax;
    const l = Math.hypot(tx, ty);
    if (l > 1) { v[0] = (tx * rollMax) / l; v[1] = (ty * pitchMax) / l; }
  }
  if (!isZero(yawMax)) v[2] = constrainFloat(v[2], -yawMax, yawMax);
  return v;
}
export function eulerAccelLimit(e, a) {
  const sphi = constrainFloat(Math.abs(Math.sin(e[0])), 0.1, 1), cphi = constrainFloat(Math.abs(Math.cos(e[0])), 0.1, 1);
  const sth = constrainFloat(Math.abs(Math.sin(e[1])), 0.1, 1), cth = constrainFloat(Math.abs(Math.cos(e[1])), 0.1, 1);
  if (isZero(a[0]) || isZero(a[1]) || isZero(a[2]) || isNegative(a[0]) || isNegative(a[1]) || isNegative(a[2])) return a.slice();
  return [a[0], Math.min(a[1] / cphi, a[2] / sphi), Math.min(Math.min(a[0] / sth, a[1] / (sphi * cth)), a[2] / (cphi * cth))];
}
export function eulerRateToAngVel(e, r) {
  const st = Math.sin(e[1]), ct = Math.cos(e[1]), sp = Math.sin(e[0]), cp = Math.cos(e[0]);
  return [r[0] - st * r[2], cp * r[1] + sp * ct * r[2], -sp * r[1] + ct * cp * r[2]];
}
// returns null (output untouched in C++) when cos(pitch) is zero
export function angVelToEulerRate(e, w) {
  const st = Math.sin(e[1]), ct = Math.cos(e[1]), sp = Math.sin(e[0]), cp = Math.cos(e[0]);
  if (isZero(ct)) return null;
  return [w[0] + sp * (st / ct) * w[1] + cp * (st / ct) * w[2], cp * w[1] - sp * w[2], (sp / ct) * w[1] + (cp / ct) * w[2]];
}
