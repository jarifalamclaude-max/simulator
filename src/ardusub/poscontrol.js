// Port of AC_PosControl (ArduSub 4.5.7): vertical and horizontal kinematic shaping, the
// P/PID cascades, lean-angle conversion, terrain (SURFTRAK) offsets, EKF-reset handling,
// vibration compensation and the PSCN/PSCE/PSCD log records. Units follow ArduPilot:
// cm, cm/s, cm/s^2, NEU. is_active_* uses the firmware's loop tick (advanceTick()).
// setParams(P) updates gains live; it never resets integrators or targets.

import { AC_P_1D, AC_P_2D, AC_PID_Basic, AC_PID, AC_PID_2D } from './pid.js';
import {
  shapeVelAccel, shapePosVelAccel, shapeAccel, shapeAccelXY, shapeVelAccelXY, shapePosVelAccelXY, updatePosVelAccel, updateVelAccel,
  updatePosVelAccelXY, updateVelAccelXY, stoppingDistance, angleToAccel, accelToAngle, limitAccelXY, limitLength2, kinematicLimit, filterBankFromParams,
} from './control.js';
import { constrainFloat, isPositive, isNegative, isZero, sq, safeSqrt, GRAVITY_MSS } from '../core/math.js';

const OVERSPEED_GAIN_Z = 2.0;
const RELAX_TC = 0.16;
const THROTTLE_CUTOFF_HZ = 2.0;
const STOP_UP_MAX = 300, STOP_DOWN_MAX = 200;
const VIBE_COMP_P_GAIN = 0.25, VIBE_COMP_I_GAIN = 0.125;
const JERK_XY_DEFAULT = 5, JERK_Z_DEFAULT = 5; // m/s^3 (POSCONTROL_JERK_XY / _Z)

export class PosControl {
  constructor(ahrs, motors, att) {
    this.ahrs = ahrs; this.motors = motors; this.att = att;
    this.dt = 0.0025;
    // ArduSub constructor defaults (AC_PosControl.cpp:33-50, 326-345)
    this.pPosZ = new AC_P_1D(3);
    this.pidVelZ = new AC_PID_Basic({ p: 8, i: 0, d: 0, ff: 0, imax: 1000, flte: 5, fltd: 5 });
    this.pidAccelZ = new AC_PID({ p: 0.5, i: 0.1, d: 0, ff: 0, imax: 100, fltt: 0, flte: 20, fltd: 0 });
    this.pPosXY = new AC_P_2D(1);
    this.pidVelXY = new AC_PID_2D({ p: 1, i: 0.5, d: 0, ff: 0, imax: 1000, flte: 5, fltd: 5 });
    this.velMaxDown = -150; this.velMaxUp = 250; this.accelMaxZ = 250; this.shapingJerkZ = JERK_Z_DEFAULT; this.jerkMaxZ = JERK_Z_DEFAULT * 100;
    this.velMaxXY = 500; this.accelMaxXY = 100; this.shapingJerkXY = JERK_XY_DEFAULT; this.jerkMaxXY = JERK_XY_DEFAULT * 100;
    this.posTarget = [0, 0, 0]; this.velDesired = [0, 0, 0]; this.velTarget = [0, 0, 0]; this.accelDesired = [0, 0, 0]; this.accelTarget = [0, 0, 0];
    this.posOffsetZ = 0; this.velOffsetZ = 0; this.accelOffsetZ = 0; this.posOffsetTargetZ = 0;
    this.limitZ = 0; this.limitXY = [0, 0];
    this.lastUpdateZ = -1e9; this.lastUpdateXY = -1e9; this.tick = 0;
    this.rollTarget = 0; this.pitchTarget = 0; this.yawTarget = 0; this.yawRateTarget = 0;
    this.angleMaxOverrideCd = 0; this.pscAngleMax = 0; this.leanAngleMaxCd = 0;
    this.angleMaxCd = 4500;
    this.fwdPitchIsLimited = false;
    this.velZControlRatio = 2;
    this.vibeCompEnabled = false;
    this.ekfXYResetMs = 0; this.ekfZResetMs = 0;
    this.timingErrors = 0; // INTERNAL_ERROR flow_of_control count (re-init inside update_*_controller)
    this.lastThrottle = 0;
  }
  setParams(P) {
    this.pPosZ.kp = P.PSC_POSZ_P ?? 3;
    this.pidVelZ.setGains({ p: P.PSC_VELZ_P ?? 8, i: P.PSC_VELZ_I ?? 0, d: P.PSC_VELZ_D ?? 0, imax: P.PSC_VELZ_IMAX ?? 1000, ff: P.PSC_VELZ_FF ?? 0, flte: P.PSC_VELZ_FLTE ?? 5, fltd: P.PSC_VELZ_FLTD ?? 5 });
    this.pidAccelZ.setGains({
      p: P.PSC_ACCZ_P ?? 0.5, i: P.PSC_ACCZ_I ?? 0.1, d: P.PSC_ACCZ_D ?? 0, imax: P.PSC_ACCZ_IMAX ?? 100, ff: P.PSC_ACCZ_FF ?? 0,
      fltt: P.PSC_ACCZ_FLTT ?? 0, flte: P.PSC_ACCZ_FLTE ?? 20, fltd: P.PSC_ACCZ_FLTD ?? 0, pdmx: P.PSC_ACCZ_PDMX ?? 0,
      smax: P.PSC_ACCZ_SMAX ?? 0, dff: P.PSC_ACCZ_D_FF ?? 0, ntf: P.PSC_ACCZ_NTF ?? 0, nef: P.PSC_ACCZ_NEF ?? 0,
    });
    this.shapingJerkZ = P.PSC_JERK_Z ?? JERK_Z_DEFAULT;
    this.pPosXY.kp = P.PSC_POSXY_P ?? 1;
    this.pidVelXY.setGains({ p: P.PSC_VELXY_P ?? 1, i: P.PSC_VELXY_I ?? 0.5, d: P.PSC_VELXY_D ?? 0, imax: P.PSC_VELXY_IMAX ?? 1000, ff: P.PSC_VELXY_FF ?? 0, flte: P.PSC_VELXY_FLTE ?? 5, fltd: P.PSC_VELXY_FLTD ?? 5 });
    this.shapingJerkXY = P.PSC_JERK_XY ?? JERK_XY_DEFAULT;
    this.pscAngleMax = P.PSC_ANGLE_MAX ?? 0;
    this.angleMaxCd = P.ANGLE_MAX ?? 4500;
    this.filterBank = filterBankFromParams(P);
    if (isPositive(this.dt)) this.setNotchSampleRate(1 / this.dt);
  }
  configure(P) { this.setParams(P); }
  setNotchSampleRate(rate) { this.pidAccelZ.setNotchSampleRate(rate, this.filterBank); }
  // called once per main loop by the firmware so is_active checks work
  advanceTick() { this.tick++; }
  setDt(dt) { this.dt = dt; }
  getDt() { return this.dt; }
  isActiveZ() { return this.tick - this.lastUpdateZ <= 1; }
  isActiveXY() { return this.tick - this.lastUpdateXY <= 1; }

  // ---- accessors ----
  getPosXYP() { return this.pPosXY; }
  getPosZP() { return this.pPosZ; }
  getVelXYPid() { return this.pidVelXY; }
  getVelZPid() { return this.pidVelZ; }
  getAccelZPid() { return this.pidAccelZ; }
  getZAccelCmss() { return this.ahrs.zAccelCmss; }
  getPosTargetCm() { return this.posTarget; }
  getPosTargetZCm() { return this.posTarget[2]; }
  setPosTargetZCm(z) { this.posTarget[2] = z; }
  setPosTargetXYCm(x, y) { this.posTarget[0] = x; this.posTarget[1] = y; }
  getVelDesiredCms() { return this.velDesired; }
  getVelTargetCms() { return this.velTarget; }
  getVelTargetZCms() { return this.velTarget[2]; }
  setVelDesiredCms(v) { this.velDesired = v.slice(); }
  setVelDesiredXYCms(v) { this.velDesired[0] = v[0]; this.velDesired[1] = v[1]; }
  setVelDesiredZCms(v) { this.velDesired[2] = v; }
  setAccelDesiredXYCmss(a) { this.accelDesired[0] = a[0]; this.accelDesired[1] = a[1]; }
  getAccelTargetCmss() { return this.accelTarget; }
  setPosVelAccel(pos, vel, accel) { this.posTarget = pos.slice(); this.velDesired = vel.slice(); this.accelDesired = accel.slice(); }
  setPosVelAccelXY(pos, vel, accel) { this.posTarget[0] = pos[0]; this.posTarget[1] = pos[1]; this.velDesired[0] = vel[0]; this.velDesired[1] = vel[1]; this.accelDesired[0] = accel[0]; this.accelDesired[1] = accel[1]; }
  getPosErrorCm() { const p = this.ahrs.positionXYCm ?? [0, 0]; return [this.posTarget[0] - p[0], this.posTarget[1] - p[1], this.posTarget[2] - this.ahrs.positionZUpCm]; }
  getPosErrorXYCm() { const e = this.getPosErrorCm(); return Math.hypot(e[0], e[1]); }
  getPosErrorZCm() { return this.posTarget[2] - this.ahrs.positionZUpCm; }
  getMaxSpeedXYCms() { return this.velMaxXY; }
  getMaxAccelXYCmss() { return this.accelMaxXY; }
  getMaxSpeedUpCms() { return this.velMaxUp; }
  getMaxSpeedDownCms() { return this.velMaxDown; }
  getMaxAccelZCmss() { return this.accelMaxZ; }
  getShapingJerkXYCmsss() { return this.shapingJerkXY * 100; }
  setPosErrorMaxXYCm(e) { this.pPosXY.setErrorMax(e); }
  getPosErrorMaxXYCm() { return this.pPosXY.getErrorMax(); }
  getPosErrorZUpCm() { return this.pPosZ.getErrorMax(); }
  getPosErrorZDownCm() { return this.pPosZ.getErrorMin(); }
  getRollCd() { return this.rollTarget; }
  getPitchCd() { return this.pitchTarget; }
  getYawCd() { return this.yawTarget; }
  getYawRateCds() { return this.yawRateTarget; }
  getFwdPitchIsLimited() { return this.fwdPitchIsLimited; }
  setVibeComp(on) { this.vibeCompEnabled = !!on; }
  getVelZControlRatio() { return constrainFloat(this.velZControlRatio, 0, 1); }
  setExternallyLimitedXY() { this.limitXY = [this.accelTarget[0], this.accelTarget[1]]; }
  // terrain offsets (SURFTRAK: mode_surftrak.cpp:81,162)
  setPosOffsetTargetZCm(v) { this.posOffsetTargetZ = v; }
  setPosOffsetZCm(v) { this.posOffsetZ = v; }
  getPosOffsetZCm() { return this.posOffsetZ; }
  getPosOffsetTargetZCm() { return this.posOffsetTargetZ; }
  getVelOffsetZCms() { return this.velOffsetZ; }
  getAccelOffsetZCmss() { return this.accelOffsetZ; }
  // get_lean_angle_max_cd (AC_PosControl.cpp:1028): override, else PSC_ANGLE_MAX, else ANGLE_MAX
  getLeanAngleMaxCd() {
    if (isPositive(this.angleMaxOverrideCd)) return this.angleMaxOverrideCd;
    if (!isPositive(this.pscAngleMax)) return this.att.leanAngleMaxCd?.() ?? this.angleMaxCd;
    return this.pscAngleMax * 100;
  }
  setLeanAngleMaxCd(v) { this.angleMaxOverrideCd = v; }

  // ---------------- vertical ----------------
  setMaxSpeedAccelZ(speedDown, speedUp, accel) {
    speedDown = -Math.abs(speedDown);
    if (isNegative(speedDown)) this.velMaxDown = speedDown;
    if (isPositive(speedUp)) this.velMaxUp = speedUp;
    if (isPositive(accel)) this.accelMaxZ = accel;
    this.jerkMaxZ = this.shapingJerkZ * 100;
    if (isPositive(this.pidAccelZ.filtT)) this.jerkMaxZ = Math.min(this.jerkMaxZ, (Math.min(GRAVITY_MSS * 100, this.accelMaxZ) * (2 * Math.PI * this.pidAccelZ.filtT)) / 5);
    if (isPositive(this.pidAccelZ.filtE)) this.jerkMaxZ = Math.min(this.jerkMaxZ, (Math.min(GRAVITY_MSS * 100, this.accelMaxZ) * (2 * Math.PI * this.pidAccelZ.filtE)) / 5);
  }
  setCorrectionSpeedAccelZ(speedDown, speedUp, accel) {
    this.pPosZ.setLimits(-Math.abs(speedDown), speedUp, accel, 0);
  }
  initZControllerNoDescent() {
    this.initZController();
    this.velDesired[2] = Math.max(0, this.velDesired[2]); this.velTarget[2] = Math.max(0, this.velTarget[2]);
    this.accelDesired[2] = Math.max(0, this.accelDesired[2]); this.accelTarget[2] = Math.max(0, this.accelTarget[2]);
  }
  initZControllerStoppingPoint() {
    this.initZController();
    this.posTarget[2] = this.getStoppingPointZCm();
    this.velDesired[2] = 0; this.accelDesired[2] = 0;
  }
  initZController() {
    const a = this.ahrs;
    this.posTarget[2] = a.positionZUpCm;
    const v = a.velocityZUpCms;
    this.velDesired[2] = v; this.velTarget[2] = v;
    this.pidVelZ.resetFilter(); this.pidVelZ.setIntegrator(0);
    this.accelDesired[2] = constrainFloat(a.zAccelCmss, -this.accelMaxZ, this.accelMaxZ);
    this.accelTarget[2] = this.accelDesired[2];
    this.pidAccelZ.resetFilter();
    this.posOffsetTargetZ = 0; this.posOffsetZ = 0; this.velOffsetZ = 0; this.accelOffsetZ = 0;
    this.pidAccelZ.setIntegrator((this.att.throttleIn - this.motors.throttleHover) * 1000 - this.pidAccelZ.kp * (this.accelTarget[2] - a.zAccelCmss) - this.pidAccelZ.kff * this.accelTarget[2]);
    this.initEkfZReset();
    this.lastUpdateZ = this.tick;
  }
  relaxZController(throttle) {
    this.initZController();
    this.pidAccelZ.relaxIntegrator((throttle - this.motors.throttleHover) * 1000, this.dt, RELAX_TC);
  }
  overspeedGain() {
    if (this.velDesired[2] < this.velMaxDown && !isZero(this.velMaxDown)) return (OVERSPEED_GAIN_Z * this.velDesired[2]) / this.velMaxDown;
    if (this.velDesired[2] > this.velMaxUp && !isZero(this.velMaxUp)) return (OVERSPEED_GAIN_Z * this.velDesired[2]) / this.velMaxUp;
    return 1;
  }
  calculateOverspeedGain() { return this.overspeedGain(); }
  advanceZTarget() {
    const st = { pos: this.posTarget[2], vel: this.velDesired[2] };
    updatePosVelAccel(st, this.accelDesired[2], this.dt, this.limitZ, this.pPosZ.error, this.pidVelZ.error);
    this.posTarget[2] = st.pos; this.velDesired[2] = st.vel;
  }
  inputAccelZ(accel) {
    const jMax = this.jerkMaxZ * this.overspeedGain();
    this.advanceZTarget();
    const s = { accel: this.accelDesired[2] };
    shapeAccel(accel, s, jMax, this.dt);
    this.accelDesired[2] = s.accel;
  }
  // input_vel_accel_z: returns the vel input moved forward by dt (C++ updates it by reference)
  inputVelAccelZ(vel, accel, limitOutput = true) {
    const g = this.overspeedGain();
    const aMax = this.accelMaxZ * g, jMax = this.jerkMaxZ * g;
    this.advanceZTarget();
    const s = { accel: this.accelDesired[2] };
    shapeVelAccel(vel, accel, this.velDesired[2], s, -constrainFloat(aMax, 0, 750), aMax, jMax, this.dt, limitOutput);
    this.accelDesired[2] = s.accel;
    return updateVelAccel(vel, accel, this.dt, 0, 0);
  }
  // input_pos_vel_accel_z: returns {pos, vel} moved forward by dt
  inputPosVelAccelZ(pos, vel, accel, limitOutput = true) {
    const g = this.overspeedGain();
    const aMax = this.accelMaxZ * g, jMax = this.jerkMaxZ * g;
    this.advanceZTarget();
    const s = { accel: this.accelDesired[2] };
    shapePosVelAccel(pos, vel, accel, this.posTarget[2], this.velDesired[2], s, this.velMaxDown, this.velMaxUp, -constrainFloat(aMax, 0, 750), aMax, jMax, this.dt, limitOutput);
    this.accelDesired[2] = s.accel;
    const st = { pos, vel };
    updatePosVelAccel(st, accel, this.dt, 0, 0, 0);
    return st;
  }
  setAltTargetWithSlew(pos) { return this.inputPosVelAccelZ(pos, 0, 0); }
  updatePosOffsetZ(target) {
    const st = { pos: this.posOffsetZ, vel: this.velOffsetZ };
    updatePosVelAccel(st, this.accelOffsetZ, this.dt, Math.min(this.limitZ, 0), this.pPosZ.error, this.pidVelZ.error);
    this.posOffsetZ = st.pos; this.velOffsetZ = st.vel;
    const s = { accel: this.accelOffsetZ };
    shapePosVelAccel(target, 0, 0, this.posOffsetZ, this.velOffsetZ, s, this.velMaxDown, this.velMaxUp, -this.accelMaxZ, this.accelMaxZ, this.jerkMaxZ, this.dt, false);
    this.accelOffsetZ = s.accel;
  }
  setPosTargetZFromClimbRateCm(vel) {
    this.posTarget[2] -= this.posOffsetZ;
    this.velDesired[2] -= this.velOffsetZ;
    this.accelDesired[2] -= this.accelOffsetZ;
    this.inputVelAccelZ(vel, 0);
    this.updatePosOffsetZ(this.posOffsetTargetZ);
    this.posTarget[2] += this.posOffsetZ;
    this.velDesired[2] += this.velOffsetZ;
    this.accelDesired[2] += this.accelOffsetZ;
  }
  landAtClimbRateCm(vel, ignoreDescentLimit) {
    if (ignoreDescentLimit) this.limitZ = Math.max(this.limitZ, 0);
    this.inputVelAccelZ(vel, 0);
  }
  getStoppingPointZCm() {
    const z = this.ahrs.positionZUpCm;
    if (!isPositive(this.pPosZ.kp) || !isPositive(this.accelMaxZ)) return z;
    return z + constrainFloat(stoppingDistance(this.ahrs.velocityZUpCms, this.pPosZ.kp, this.accelMaxZ), -STOP_DOWN_MAX, STOP_UP_MAX);
  }

  updateZController() {
    this.handleEkfZReset();
    if (!this.isActiveZ()) { this.initZController(); this.timingErrors++; }
    this.lastUpdateZ = this.tick;
    const a = this.ahrs, m = this.motors;
    const r = this.pPosZ.updateAll(this.posTarget[2], a.positionZUpCm);
    const scaleZ = a.getControlScaleZ?.() ?? 1;
    this.posTarget[2] = r.target;
    this.velTarget[2] = r.out * scaleZ + this.velDesired[2];
    const vz = a.velocityZUpCms;
    this.accelTarget[2] = this.pidVelZ.updateAll(this.velTarget[2], vz, this.dt, m.limit.throttle_lower, m.limit.throttle_upper) * scaleZ + this.accelDesired[2];
    const hover = m.throttleHover;
    if (hover * 1000 > this.pidAccelZ.kimax) this.pidAccelZ.imax(hover * 1000);
    let thr;
    if (this.vibeCompEnabled) thr = this.getThrottleWithVibrationOverride();
    else {
      thr = this.pidAccelZ.updateAll(this.accelTarget[2], a.zAccelCmss, this.dt, m.limit.throttle_lower || m.limit.throttle_upper) * 0.001;
      thr += this.pidAccelZ.getFF() * 0.001;
    }
    thr += hover;
    this.att.setThrottleOut(thr, true, THROTTLE_CUTOFF_HZ);
    const errorRatio = this.pidVelZ.getError() / this.velMaxDown;
    this.velZControlRatio = constrainFloat(this.velZControlRatio + this.dt * 0.1 * (0.5 - errorRatio), 0, 2);
    this.limitZ = m.limit.throttle_upper ? 1 : m.limit.throttle_lower ? -1 : 0;
    this.lastThrottle = thr;
  }
  getThrottleWithVibrationOverride() {
    const m = this.motors, thrPerAccel = m.throttleHover / (GRAVITY_MSS * 100);
    const I = this.pidAccelZ.getI(), ve = this.pidVelZ.getError();
    if (!(m.limit.throttle_lower || m.limit.throttle_upper) || (isPositive(I) && isNegative(ve)) || (isNegative(I) && isPositive(ve))) {
      this.pidAccelZ.setIntegrator(I + this.dt * thrPerAccel * 1000 * ve * this.pidVelZ.kp * VIBE_COMP_I_GAIN);
    }
    return VIBE_COMP_P_GAIN * thrPerAccel * this.accelTarget[2] + this.pidAccelZ.getI() * 0.001;
  }

  // ---------------- horizontal ----------------
  setMaxSpeedAccelXY(speed, accel) {
    this.velMaxXY = speed; this.accelMaxXY = accel;
    const att = this.att;
    const jm = Math.min(att.angVelRollMaxRads?.() ?? 0, att.angVelPitchMaxRads?.() ?? 0) * GRAVITY_MSS * 100;
    const sm = Math.min(att.accelRollMaxRadss?.() ?? 0, att.accelPitchMaxRadss?.() ?? 0) * GRAVITY_MSS * 100;
    const ff = att.getBfFeedforward?.() ?? true;
    this.jerkMaxXY = this.shapingJerkXY * 100;
    if (isPositive(jm) && ff) this.jerkMaxXY = Math.min(this.jerkMaxXY, jm);
    if (isPositive(sm) && ff) this.jerkMaxXY = Math.min(0.5 * safeSqrt(this.accelMaxXY * sm), this.jerkMaxXY);
  }
  setCorrectionSpeedAccelXY(speed, accel) { this.pPosXY.setLimits(speed, accel, 0); }
  // lean_angles_to_accel (AC_PosControl.cpp:1056) from [roll, pitch, yaw] rad
  leanAnglesToAccel(e) {
    const sr = Math.sin(e[0]), cr = Math.cos(e[0]), sp = Math.sin(e[1]), cp = Math.cos(e[1]), sy = Math.sin(e[2]), cy = Math.cos(e[2]);
    const g = GRAVITY_MSS * 100, k = Math.max(cr * cp, 0.1);
    return [(g * (-cy * sp * cr - sy * sr)) / k, (g * (-sy * sp * cr + cy * sr)) / k, g];
  }
  leanAnglesToAccelXY() {
    const e = (this.att.getAttTargetEulerRad?.() ?? this.att.eulerAngleTarget).slice();
    e[2] = this.ahrs.yaw;
    const acc = this.leanAnglesToAccel(e);
    return [acc[0], acc[1]];
  }
  accelToLeanAngles(ax, ay) {
    const a = this.ahrs;
    const fwd = ax * a.cosYaw + ay * a.sinYaw;
    const right = -ax * a.sinYaw + ay * a.cosYaw;
    const pitch = accelToAngle(-fwd * 0.01) * 100;
    const cp = Math.cos((pitch * Math.PI) / 18000);
    return [accelToAngle(right * cp * 0.01) * 100, pitch];
  }
  getThrustVector() { return [this.accelTarget[0], this.accelTarget[1], -GRAVITY_MSS * 100]; }
  xyAccelMax() {
    const angleMax = Math.min(this.att.getAltholdLeanAngleMaxCd(), this.getLeanAngleMaxCd());
    return angleToAccel(angleMax * 0.01) * 100;
  }
  initXYController() {
    const a = this.ahrs;
    const e = this.att.getAttTargetEulerCd();
    this.rollTarget = e[0]; this.pitchTarget = e[1]; this.yawTarget = e[2];
    this.yawRateTarget = 0; this.angleMaxOverrideCd = 0;
    const p = a.positionXYCm, v = a.velocityXYCms;
    this.posTarget[0] = p[0]; this.posTarget[1] = p[1];
    this.velDesired[0] = v[0]; this.velDesired[1] = v[1];
    this.velTarget[0] = v[0]; this.velTarget[1] = v[1];
    this.accelDesired[0] = 0; this.accelDesired[1] = 0;
    // re-entry: seed from the attitude target, not a stale accel target (AC_PosControl.cpp:527)
    if (!this.isActiveXY()) { const acc = this.leanAnglesToAccelXY(); this.accelTarget[0] = acc[0]; this.accelTarget[1] = acc[1]; }
    const at = [this.accelTarget[0], this.accelTarget[1]];
    limitLength2(at, this.xyAccelMax());
    this.accelTarget[0] = at[0]; this.accelTarget[1] = at[1];
    this.pidVelXY.resetFilter();
    const kff = this.pidVelXY.kff;
    this.pidVelXY.setIntegrator([this.accelTarget[0] - this.velTarget[0] * kff, this.accelTarget[1] - this.velTarget[1] * kff]);
    this.initEkfXYReset();
    this.lastUpdateXY = this.tick;
  }
  getStoppingPointXYCm() {
    const a = this.ahrs, p = a.positionXYCm, v = a.velocityXYCms;
    const out = [p[0], p[1]];
    const vt = Math.hypot(v[0], v[1]);
    if (!isPositive(vt)) return out;
    const d = stoppingDistance(constrainFloat(vt, 0, this.velMaxXY), this.pPosXY.kp, this.accelMaxXY);
    if (!isPositive(d)) return out;
    const t = d / vt;
    return [out[0] + v[0] * t, out[1] + v[1] * t];
  }
  initXYControllerStoppingPoint() {
    this.initXYController();
    const sp = this.getStoppingPointXYCm();
    this.posTarget[0] = sp[0]; this.posTarget[1] = sp[1];
    this.velDesired[0] = this.velDesired[1] = 0;
    this.accelDesired[0] = this.accelDesired[1] = 0;
  }
  relaxVelocityControllerXY() {
    if (isPositive(this.dt)) { const decay = 1 - this.dt / (this.dt + RELAX_TC); this.accelTarget[0] *= decay; this.accelTarget[1] *= decay; }
    this.initXYController();
  }
  softenForLandingXY() {
    if (isPositive(this.dt)) { const p = this.ahrs.positionXYCm, k = this.dt / (this.dt + RELAX_TC); this.posTarget[0] += (p[0] - this.posTarget[0]) * k; this.posTarget[1] += (p[1] - this.posTarget[1]) * k; }
    this.setExternallyLimitedXY();
  }
  stopPosXYStabilisation() { const p = this.ahrs.positionXYCm; this.posTarget[0] = p[0]; this.posTarget[1] = p[1]; }
  stopVelXYStabilisation() {
    const p = this.ahrs.positionXYCm, v = this.ahrs.velocityXYCms;
    this.posTarget[0] = p[0]; this.posTarget[1] = p[1];
    this.velDesired[0] = v[0]; this.velDesired[1] = v[1]; this.velTarget[0] = v[0]; this.velTarget[1] = v[1];
    this.pidVelXY.resetFilter(); this.pidVelXY.resetI();
  }
  standbyXYZReset() {
    this.pidAccelZ.setIntegrator(0);
    const p = this.ahrs.positionXYCm ?? [0, 0];
    this.posTarget = [p[0], p[1], this.ahrs.positionZUpCm];
    this.pidVelXY.resetFilter();
    this.initEkfXYReset();
  }
  xyViews() {
    const pos = [this.posTarget[0], this.posTarget[1]], vel = [this.velDesired[0], this.velDesired[1]], acc = [this.accelDesired[0], this.accelDesired[1]];
    const commit = () => { this.posTarget[0] = pos[0]; this.posTarget[1] = pos[1]; this.velDesired[0] = vel[0]; this.velDesired[1] = vel[1]; this.accelDesired[0] = acc[0]; this.accelDesired[1] = acc[1]; };
    return { pos, vel, acc, commit };
  }
  advanceXYTarget(v) { updatePosVelAccelXY(v.pos, v.vel, v.acc, this.dt, this.limitXY, this.pPosXY.getError(), this.pidVelXY.getError()); }
  inputAccelXY(accel) {
    this.handleEkfXYReset();
    const v = this.xyViews();
    this.advanceXYTarget(v);
    shapeAccelXY(accel, v.acc, this.jerkMaxXY, this.dt);
    v.commit();
  }
  // returns the vel input moved forward by dt
  inputVelAccelXY(vel, accel, limitOutput = true) {
    const v = this.xyViews();
    this.advanceXYTarget(v);
    shapeVelAccelXY(vel, accel, v.vel, v.acc, this.accelMaxXY, this.jerkMaxXY, this.dt, limitOutput);
    v.commit();
    const out = vel.slice();
    updateVelAccelXY(out, accel, this.dt, [0, 0], [0, 0]);
    return out;
  }
  // returns {pos, vel} moved forward by dt
  inputPosVelAccelXY(pos, vel, accel, limitOutput = true) {
    const v = this.xyViews();
    this.advanceXYTarget(v);
    shapePosVelAccelXY(pos, vel, accel, v.pos, v.vel, v.acc, this.velMaxXY, this.accelMaxXY, this.jerkMaxXY, this.dt, limitOutput);
    v.commit();
    const p = pos.slice(), w = vel.slice();
    updatePosVelAccelXY(p, w, accel, this.dt);
    return { pos: p, vel: w };
  }
  // input_pos_xyz with pos_offset_z (terrain) and its buffer scaler
  posOffsetZScaler(posOffsetZ, buffer) {
    if (isZero(buffer)) return 1;
    const err = this.ahrs.positionZUpCm - (this.posTarget[2] - this.posOffsetZ + posOffsetZ);
    return constrainFloat(1 - (Math.abs(err) - 0.5 * buffer) / (0.5 * buffer), 0.01, 1);
  }
  inputPosXYZ(pos, posOffsetZ = 0, buffer = 0) {
    const scaler = this.posOffsetZScaler(posOffsetZ, buffer);
    this.posTarget[2] -= this.posOffsetZ; this.velDesired[2] -= this.velOffsetZ; this.accelDesired[2] -= this.accelOffsetZ;
    const g = this.overspeedGain(), aMaxZ = this.accelMaxZ * g, jMaxZ = this.jerkMaxZ * g;
    const v = this.xyViews();
    this.advanceXYTarget(v);
    this.advanceZTarget();
    let vxy = 0, vz = 0;
    const d = [pos[0] - v.pos[0], pos[1] - v.pos[1], pos[2] - this.posTarget[2]];
    if (isPositive(sq(d[0]) + sq(d[1]) + sq(d[2]))) {
      const n = Math.hypot(...d), u = d.map((x) => x / n);
      const vmax = kinematicLimit(u, this.velMaxXY, this.velMaxUp, this.velMaxDown);
      vxy = vmax * Math.hypot(u[0], u[1]); vz = Math.abs(vmax * u[2]);
    }
    vxy *= scaler;
    shapePosVelAccelXY([pos[0], pos[1]], [0, 0], [0, 0], v.pos, v.vel, v.acc, vxy, this.accelMaxXY, this.jerkMaxXY, this.dt, false);
    v.commit();
    const s = { accel: this.accelDesired[2] };
    shapePosVelAccel(pos[2], 0, 0, this.posTarget[2], this.velDesired[2], s, -vz, vz, -constrainFloat(aMaxZ, 0, 750), aMaxZ, jMaxZ, this.dt, false);
    this.accelDesired[2] = s.accel;
    this.updatePosOffsetZ(posOffsetZ);
    this.posTarget[2] += this.posOffsetZ; this.velDesired[2] += this.velOffsetZ; this.accelDesired[2] += this.accelOffsetZ;
  }

  updateXYController() {
    this.handleEkfXYReset();
    if (!this.isActiveXY()) { this.initXYController(); this.timingErrors++; }
    this.lastUpdateXY = this.tick;
    const a = this.ahrs;
    const scaleXY = a.getControlScaleXY?.() ?? 1;
    const r = this.pPosXY.updateAll([this.posTarget[0], this.posTarget[1]], a.positionXYCm);
    this.posTarget[0] = r.target[0]; this.posTarget[1] = r.target[1];
    this.velTarget[0] = r.out[0] * scaleXY + this.velDesired[0];
    this.velTarget[1] = r.out[1] * scaleXY + this.velDesired[1];
    const acc = this.pidVelXY.updateAll([this.velTarget[0], this.velTarget[1]], a.velocityXYCms, this.dt, this.limitXY);
    this.accelTarget[0] = acc[0] * scaleXY + this.accelDesired[0];
    this.accelTarget[1] = acc[1] * scaleXY + this.accelDesired[1];
    const accelMax = this.xyAccelMax();
    this.limitXY = [this.accelTarget[0], this.accelTarget[1]];
    const at = [this.accelTarget[0], this.accelTarget[1]];
    if (!limitAccelXY([this.velDesired[0], this.velDesired[1]], at, accelMax)) this.limitXY = [0, 0];
    else {
      const fwdU = this.limitXY[0] * a.cosYaw + this.limitXY[1] * a.sinYaw;
      const pitchU = accelToAngle(-Math.min(fwdU, accelMax) * 0.01) * 100;
      const fwdL = at[0] * a.cosYaw + at[1] * a.sinYaw;
      const pitchL = accelToAngle(-fwdL * 0.01) * 100;
      this.fwdPitchIsLimited = isNegative(pitchU) && pitchU < pitchL;
    }
    this.accelTarget[0] = at[0]; this.accelTarget[1] = at[1];
    [this.rollTarget, this.pitchTarget] = this.accelToLeanAngles(at[0], at[1]);
    this.calculateYawAndRateYaw();
  }
  calculateYawAndRateYaw() {
    let turnRate = 0;
    const vd = this.velDesired, ad = this.accelDesired;
    const vl = Math.hypot(vd[0], vd[1]);
    if (isPositive(vl)) {
      const af = (ad[0] * vd[0] + ad[1] * vd[1]) / vl;
      const turn = [ad[0] - (vd[0] * af) / vl, ad[1] - (vd[1] * af) / vl];
      turnRate = Math.hypot(turn[0], turn[1]) / vl;
      if (turn[1] * vd[0] - turn[0] * vd[1] < 0) turnRate = -turnRate;
    }
    if (vl > this.velMaxXY * 0.05) {
      this.yawTarget = ((Math.atan2(vd[1], vd[0]) * 180) / Math.PI) * 100;
      this.yawRateTarget = turnRate * ((180 / Math.PI) * 100);
      return true;
    }
    return false;
  }
  getBearingToTargetCd() {
    const p = this.ahrs.positionXYCm;
    let b = Math.atan2(this.posTarget[1] - p[1], this.posTarget[0] - p[0]) * (18000 / Math.PI);
    if (b < 0) b += 36000;
    return Math.trunc(b);
  }
  crosstrackError() {
    const p = this.ahrs.positionXYCm, e = [p[0] - this.posTarget[0], p[1] - this.posTarget[1]];
    const vd = [this.velDesired[0], this.velDesired[1]];
    if (isZero(sq(vd[0]) + sq(vd[1]))) return Math.hypot(e[0], e[1]);
    const l = Math.hypot(vd[0], vd[1]), dot = (e[0] * vd[0] + e[1] * vd[1]) / l;
    return safeSqrt(Math.max(sq(e[0]) + sq(e[1]) - sq(dot), 0));
  }

  // ---------------- EKF resets (AC_PosControl.cpp:1271) ----------------
  // AHRS optional hooks: getLastPosNorthEastReset() / getLastPosDownReset() -> reset time ms (0 = none)
  initEkfXYReset() { this.ekfXYResetMs = this.ahrs.getLastPosNorthEastReset?.() ?? 0; }
  handleEkfXYReset() {
    const ms = this.ahrs.getLastPosNorthEastReset?.() ?? 0;
    if (ms !== this.ekfXYResetMs) {
      const p = this.ahrs.positionXYCm, v = this.ahrs.velocityXYCms, pe = this.pPosXY.getError(), ve = this.pidVelXY.getError();
      this.posTarget[0] = p[0] + pe[0]; this.posTarget[1] = p[1] + pe[1];
      this.velTarget[0] = v[0] + ve[0]; this.velTarget[1] = v[1] + ve[1];
      this.ekfXYResetMs = ms;
    }
  }
  initEkfZReset() { this.ekfZResetMs = this.ahrs.getLastPosDownReset?.() ?? 0; }
  handleEkfZReset() {
    const ms = this.ahrs.getLastPosDownReset?.() ?? 0;
    if (ms !== 0 && ms !== this.ekfZResetMs) {
      this.posTarget[2] = this.ahrs.positionZUpCm + this.pPosZ.getError();
      this.velTarget[2] = this.ahrs.velocityZUpCms + this.pidVelZ.getError();
      this.ekfZResetMs = ms;
    }
  }

  // AC_PosControl::pre_arm_checks
  preArmChecks(prefix = 'PSC') {
    if (!isPositive(this.pPosXY.kp)) return `${prefix}_POSXY_P must be > 0`;
    if (!isPositive(this.pPosZ.kp)) return `${prefix}_POSZ_P must be > 0`;
    if (!isPositive(this.pidVelZ.kp)) return `${prefix}_VELZ_P must be > 0`;
    if (!isPositive(this.pidAccelZ.kp)) return `${prefix}_ACCZ_P must be > 0`;
    if (!isPositive(this.pidAccelZ.ki)) return `${prefix}_ACCZ_I must be > 0`;
    return null;
  }

  // write_log(): PSCN / PSCE / PSCD records (NED signs for D) — null when the axis is inactive
  writeLog() {
    const a = this.ahrs, out = {};
    if (this.isActiveXY()) {
      const acc = this.leanAnglesToAccelXY(), p = a.positionXYCm, v = a.velNed ?? [a.velocityXYCms[0] / 100, a.velocityXYCms[1] / 100, 0];
      out.PSCN = { TPN: this.posTarget[0], PN: p[0], DVN: this.velDesired[0], TVN: this.velTarget[0], VN: v[0] * 100, DAN: this.accelDesired[0], TAN: this.accelTarget[0], AN: acc[0] };
      out.PSCE = { TPE: this.posTarget[1], PE: p[1], DVE: this.velDesired[1], TVE: this.velTarget[1], VE: v[1] * 100, DAE: this.accelDesired[1], TAE: this.accelTarget[1], AE: acc[1] };
    }
    if (this.isActiveZ()) {
      out.PSCD = { TPD: -this.posTarget[2], PD: -a.positionZUpCm, DVD: -this.velDesired[2], TVD: -this.velTarget[2], VD: -a.velocityZUpCms, DAD: -this.accelDesired[2], TAD: -this.accelTarget[2], AD: -a.zAccelCmss };
    }
    return out;
  }
}
