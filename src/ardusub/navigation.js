// Navigation helpers for ArduSub's GUIDED, AUTO and CIRCLE modes: a compact
// AC_WPNav (straight-line legs with a speed/acceleration/jerk-limited profile
// along the track, WPNAV_* parameters) and AC_Circle (CIRCLE_RADIUS/RATE/OPTIONS).
// They drive AC_PosControl's targets the way the libraries do: the XY
// controller is updated here, the Z controller by the calling mode.

import { clamp, wrapPI, wrap360cd } from '../core/math.js';

const WPNAV_ACCELERATION = 250; // AC_WPNav.h:16
const CIRCLE_ANGULAR_ACCEL_MIN = (2 * Math.PI) / 180; // AC_Circle.cpp AC_CIRCLE_ANGULAR_ACCEL_MIN 2 deg/s/s

export class WPNav {
  constructor(sub) { this.sub = sub; this.dest = null; this.origin = null; this.reached = true; this.yawCd = 0; this.s = 0; this.sv = 0; this.sa = 0; }
  get P() { return this.sub.params; }
  get pos() { return this.sub.pos; }
  speedXY() { return Math.max(20, this.P.WPNAV_SPEED ?? 500); }
  speedUp() { return Math.max(10, Math.abs(this.P.WPNAV_SPEED_UP ?? 250)); }
  speedDn() { return Math.max(10, Math.abs(this.P.WPNAV_SPEED_DN ?? 150)); }
  accelXY() { const a = this.P.WPNAV_ACCEL ?? WPNAV_ACCELERATION; return a > 0 ? a : WPNAV_ACCELERATION; } // get_wp_acceleration
  accelZ() { const a = this.P.WPNAV_ACCEL_Z ?? 100; return a > 0 ? a : 100; }
  radius() { return Math.max(5, this.P.WPNAV_RADIUS ?? 200); }
  jerk() { return Math.max(1, this.P.WPNAV_JERK ?? 1) * 100; } // m/s^3 -> cm/s^3

  // AC_WPNav::wp_and_spline_init
  init() {
    const pos = this.pos;
    pos.setMaxSpeedAccelXY(this.speedXY(), this.accelXY());
    pos.setCorrectionSpeedAccelXY(this.speedXY(), this.accelXY());
    pos.setMaxSpeedAccelZ(-this.speedDn(), this.speedUp(), this.accelZ());
    pos.setCorrectionSpeedAccelZ(-this.speedDn(), this.speedUp(), this.accelZ());
    pos.initXYControllerStoppingPoint();
    pos.initZController();
    this.dest = this.stoppingPoint();
    this.origin = this.dest.slice();
    this.s = 0; this.sv = 0; this.sa = 0; this.len = 0; this.u = [0, 0, 0];
    this.reached = true;
    this.yawCd = this.sub.sensors.yawSensor;
  }
  stoppingPoint() { const p = this.pos.posTarget; return [p[0], p[1], this.sub.sensors.positionZUpCm]; }

  // dest: [north cm, east cm, up cm]; terrainAlt: dest[2] is height above the bottom (rangefinder)
  setDestination(dest, terrainAlt = false) {
    const pt = this.pos.posTarget;
    this.origin = this.reached || !this.dest ? [pt[0], pt[1], pt[2]] : this.currentTarget();
    this.dest = dest.slice();
    this.terrain = !!terrainAlt;
    this.s = 0; this.sv = 0; this.sa = 0;
    this.reached = false;
    this.updateTrack();
    return true;
  }
  currentTarget() { return [0, 1, 2].map((k) => this.origin[k] + this.u[k] * this.s); }
  destUp() {
    if (!this.terrain) return this.dest[2];
    const tOff = this.sub.rangefinderState.terrainOffsetCm;
    return tOff + this.dest[2];
  }
  updateTrack() {
    const d = [this.dest[0] - this.origin[0], this.dest[1] - this.origin[1], this.destUp() - this.origin[2]];
    this.len = Math.hypot(d[0], d[1], d[2]);
    this.u = this.len > 1e-6 ? d.map((v) => v / this.len) : [0, 0, 0];
  }

  // AC_WPNav::update_wpnav: returns false when terrain data is missing
  update(dt) {
    const pos = this.pos;
    let ok = true;
    if (this.terrain) { ok = this.sub.rangefinderAltOk(); if (ok) this.updateTrack(); }
    const u = this.u, uxy = Math.hypot(u[0], u[1]), uz = u[2];
    const vlim = Math.min(uxy > 1e-6 ? this.speedXY() / uxy : Infinity, Math.abs(uz) > 1e-6 ? (uz > 0 ? this.speedUp() : this.speedDn()) / Math.abs(uz) : Infinity);
    const alim = Math.min(uxy > 1e-6 ? this.accelXY() / uxy : Infinity, Math.abs(uz) > 1e-6 ? this.accelZ() / Math.abs(uz) : Infinity);
    const rem = Math.max(0, this.len - this.s);
    const vStop = Math.sqrt(2 * alim * rem);
    const vTgt = Math.min(vlim, vStop);
    const aDes = clamp((vTgt - this.sv) / Math.max(dt, 1e-3), -alim, alim);
    const j = this.jerk() * dt;
    this.sa = clamp(aDes, this.sa - j, this.sa + j);
    if (vTgt < this.sv) this.sa = Math.min(this.sa, aDes); // never overshoot the stopping profile
    this.sv = clamp(this.sv + this.sa * dt, 0, vlim);
    this.s = Math.min(this.len, this.s + this.sv * dt);
    if (this.s >= this.len) { this.sv = 0; this.sa = 0; }
    const tgt = this.currentTarget();
    pos.posTarget[0] = tgt[0]; pos.posTarget[1] = tgt[1];
    if (ok) pos.posTarget[2] = tgt[2];
    for (let k = 0; k < 3; k++) { pos.velDesired[k] = u[k] * this.sv; pos.accelDesired[k] = u[k] * this.sa; }
    if (!ok) { pos.velDesired[2] = 0; pos.accelDesired[2] = 0; }
    pos.updateXYController();
    // heading along the track while moving (wp_nav.get_yaw)
    if (Math.hypot(pos.velDesired[0], pos.velDesired[1]) > 10) this.yawCd = wrap360cd((Math.atan2(pos.velDesired[1], pos.velDesired[0]) * 18000) / Math.PI);
    if (this.s >= this.len) {
      const p = this.sub.sensors.positionXYCm, z = this.sub.sensors.positionZUpCm;
      if (Math.hypot(p[0] - this.dest[0], p[1] - this.dest[1], z - this.destUp()) < this.radius()) this.reached = true;
    }
    return ok;
  }
  distanceToDestination() { if (!this.dest) return 0; const p = this.sub.sensors.positionXYCm; return Math.hypot(p[0] - this.dest[0], p[1] - this.dest[1]); }
  bearingToDestinationCd() { if (!this.dest) return 0; const p = this.sub.sensors.positionXYCm; return wrap360cd((Math.atan2(this.dest[1] - p[1], this.dest[0] - p[0]) * 18000) / Math.PI); }
}

export class CircleNav {
  constructor(sub) { this.sub = sub; this.center = [0, 0, 0]; this.angle = 0; this.angleTotal = 0; this.angVel = 0; this.yawCd = 0; }
  get P() { return this.sub.params; }
  get pos() { return this.sub.pos; }
  radius() { return Math.max(0, this.P.CIRCLE_RADIUS ?? 1000); }
  // AC_Circle::calc_velocities
  calcVelocities() {
    const r = this.radius(), rate = ((this.P.CIRCLE_RATE ?? 2) * Math.PI) / 180;
    const vmaxXY = this.pos.velMaxXY ?? 500, amaxXY = this.pos.accelMaxXY ?? 250;
    if (r <= 0) { this.angVelMax = rate; this.angAccel = Math.max(Math.abs(rate), CIRCLE_ANGULAR_ACCEL_MIN); }
    else {
      const vmax = Math.max(Math.min(vmaxXY, Math.sqrt(0.5 * amaxXY * r)), 0);
      this.angVelMax = clamp(rate, -vmax / r, vmax / r);
      this.angAccel = Math.max(amaxXY / r, CIRCLE_ANGULAR_ACCEL_MIN);
    }
  }
  // AC_Circle::init: centre one radius ahead of the stopping point
  init() {
    const pos = this.pos, s = this.sub.sensors;
    pos.initXYControllerStoppingPoint();
    pos.initZController();
    const yaw = s.yaw, r = this.radius();
    const opts = this.P.CIRCLE_OPTIONS ?? 1;
    this.center = [pos.posTarget[0], pos.posTarget[1], pos.posTarget[2]];
    if (!(opts & 4) && r > 0) { this.center[0] += r * Math.cos(yaw); this.center[1] += r * Math.sin(yaw); }
    this.calcVelocities();
    this.angVel = 0;
    // init_start_angle: bearing from the centre to the vehicle (heading for a panorama)
    const v = s.positionXYCm, dx = v[0] - this.center[0], dy = v[1] - this.center[1];
    this.angle = r <= 0 ? yaw : Math.hypot(dx, dy) < 1 ? wrapPI(yaw - Math.PI) : Math.atan2(dy, dx);
    this.angleTotal = 0;
    this.yawCd = s.yawSensor;
  }
  update(dt, climbRateCms) {
    const pos = this.pos, r = this.radius(), opts = this.P.CIRCLE_OPTIONS ?? 1;
    this.calcVelocities();
    const a = this.angAccel * dt;
    if (this.angVelMax >= 0) this.angVel = Math.min(this.angVel + a, this.angVelMax);
    else this.angVel = Math.max(this.angVel - a, this.angVelMax);
    this.angle = wrapPI(this.angle + this.angVel * dt);
    this.angleTotal += this.angVel * dt;
    if (r > 0) {
      pos.posTarget[0] = this.center[0] + r * Math.cos(this.angle);
      pos.posTarget[1] = this.center[1] + r * Math.sin(this.angle);
      pos.velDesired[0] = -r * this.angVel * Math.sin(this.angle);
      pos.velDesired[1] = r * this.angVel * Math.cos(this.angle);
      pos.accelDesired[0] = -r * this.angVel * this.angVel * Math.cos(this.angle);
      pos.accelDesired[1] = -r * this.angVel * this.angVel * Math.sin(this.angle);
      // heading from the vehicle to the centre, or along the direction of travel (CIRCLE_OPTIONS bit 1)
      const v = this.sub.sensors.positionXYCm;
      let yaw = (Math.atan2(this.center[1] - v[1], this.center[0] - v[0]) * 18000) / Math.PI;
      if (opts & 2) yaw += (this.P.CIRCLE_RATE ?? 2) > 0 ? -9000 : 9000;
      this.yawCd = wrap360cd(yaw);
    } else {
      pos.posTarget[0] = this.center[0]; pos.posTarget[1] = this.center[1];
      pos.velDesired[0] = pos.velDesired[1] = 0; pos.accelDesired[0] = pos.accelDesired[1] = 0;
      this.yawCd = wrap360cd((this.angle * 18000) / Math.PI);
    }
    pos.setPosTargetZFromClimbRateCm(climbRateCms);
    pos.updateXYController();
    return true;
  }
}
