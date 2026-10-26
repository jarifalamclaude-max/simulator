// Port of AP_Motors6DOF (ArduSub 4.5.7) + the AP_MotorsMulticopter / AP_MotorsMatrix / AP_Motors
// pieces it relies on: throttle filter, spool state machine (MOT_SPOOL_TIME/TIM_DN, MOT_SAFE_DISARM/
// TIME, ground-idle ramp), battery-current limiter (MOT_BAT_CURR_MAX/TC), MOT_PWM_TYPE endpoint
// forcing, SERVOn_FUNCTION output routing (SRV_Channels), motor test and MOTOR_DETECT helpers.
// The output stage reproduces Sub::motors_output(): SRV_Channels::output_ch_all() runs before
// motors.output(), so the hardware sees the previous loop's values, and normal PWM pulses are
// emitted at RC_SPEED.
// Extra mixer variants for automatically generated frames (not in the firmware): 'scale' preserves
// the commanded wrench direction under saturation, 'priority' keeps attitude authority first.
// A table compiled into AP_Motors6DOF as a custom frame always runs the 'generic' clamp mixer.

import { LowPassFilter } from './control.js';
import { clamp, constrainFloat, isPositive, isZero } from '../core/math.js';
import { frameTable, frameTestOrder, FRAMES } from '../alloc/frames.js';

export const SPOOL = { SHUT_DOWN: 0, GROUND_IDLE: 1, SPOOLING_UP: 2, THROTTLE_UNLIMITED: 3, SPOOLING_DOWN: 4 };
export const PWM_TYPE = { NORMAL: 0, ONESHOT: 1, ONESHOT125: 2, BRUSHED: 3, DSHOT150: 4, DSHOT300: 5, DSHOT600: 6, DSHOT1200: 7, PWM_RANGE: 8, PWM_ANGLE: 9 };
export const AP_MOTORS_MAX_NUM_MOTORS = 12;
export const AP_MOTORS_THST_HOVER_MIN = 0.125, AP_MOTORS_THST_HOVER_MAX = 0.6875; // AP_MotorsMulticopter.h:12
export const FIRMWARE_VARIANTS = ['generic', 'vectored', 'vectored6dof'];
const MIN_SPOOL_TIME = 0.05; // AP_MotorsMulticopter.cpp:491
const NUM_SERVO_CHANNELS = 16;

// SRV_Channels::get_motor_function (SRV_Channel.h:539): Motor1..8 = 33..40, Motor9..12 = 82..85
export const motorFunction = (i) => (i < 8 ? 33 + i : 82 + (i - 8));
const isDigital = (t) => t >= PWM_TYPE.DSHOT150 && t <= PWM_TYPE.DSHOT1200;

// Motor -> servo output channels (0-based). A motor goes to every channel whose SERVOn_FUNCTION is its
// motor function; if none has it and channel i is unassigned (0), channel i takes it
// (AP_Motors::add_motor_num -> SRV_Channels::set_aux_channel_default).
export function servoChannelMap(P = {}, nMotors = 8) {
  const fn = [];
  for (let ch = 0; ch < NUM_SERVO_CHANNELS; ch++) fn.push(Math.trunc(Number(P[`SERVO${ch + 1}_FUNCTION`] ?? 0)) || 0);
  const map = [];
  for (let m = 0; m < nMotors; m++) {
    const f = motorFunction(m);
    if (!fn.includes(f) && m < NUM_SERVO_CHANNELS && fn[m] === 0) fn[m] = f;
    const chs = [];
    fn.forEach((v, ch) => { if (v === f) chs.push(ch); });
    map.push(chs);
  }
  return { map, functions: fn };
}

const sameTable = (a, b) => a && b && a.length === b.length && a.every((r, i) => r.length === b[i].length && r.every((v, j) => v === b[i][j]));

export class Motors6DOF {
  constructor() {
    this.factors = []; // [roll, pitch, yaw, throttle, forward, lateral] per motor
    this.testOrder = [];
    this.variant = 'generic';
    this.reverse = new Array(AP_MOTORS_MAX_NUM_MOTORS).fill(1);
    this.pwmMin = 1100; this.pwmMax = 1900; this.pwmType = 0;
    this.fvCoupling = 1;
    this.spoolTime = 0.5; this.spoolDownTime = 0; this.safeDisarm = 0; this.safeTime = 1; this.spinArm = 0.1; this.spinMin = 0.15;
    this._throttleHover = 0.5; this._hoverSeen = undefined; this._booted = false;
    this.battCurrMax = 0; this.battCurrTc = 5; this.outputLimited = 1; this.battCurrentLast = 0;
    this.battVoltage = 0; this.battCurrent = 0; this.battHealthy = false; this.battHaveCurrent = false;
    this.rcSpeed = 200;
    this.armed = false; this.interlock = true;
    this.spool = SPOOL.SHUT_DOWN;
    this.desiredSpool = SPOOL.SHUT_DOWN;
    this.spinUpRatio = 0;
    this.disarmSafeTimer = 0;
    this.throttleThrustMax = 0;
    this.throttleAvgMax = 0;
    this.maxThrottle = 1;
    this.rollIn = 0; this.pitchIn = 0; this.yawIn = 0; this.throttleIn = 0; this.forwardIn = 0; this.lateralIn = 0;
    this.rollInFf = 0; this.pitchInFf = 0; this.yawInFf = 0;
    this.throttleFilter = new LowPassFilter(0);
    this.limit = { roll: false, pitch: false, yaw: false, throttle_lower: false, throttle_upper: false };
    this.thrust = [];
    this.pwm = []; // last rc_write() value per motor
    this.servoMap = []; this.servoFunctions = []; this.servoParams = [];
    this.servoPending = []; // SRV_Channel output_pwm (written by rc_write this loop)
    this.servoOut = []; // what the ESCs currently see
    this.escTimer = 0;
    this.test = null; // legacy {motor, pwm} during motor test (set by the firmware each loop)
    this._testing = false;
    this.saturated = false;
    this.pending = null; // reboot-required changes queued while armed
  }

  // Boot-time configuration (AP_Motors6DOF::init + Sub::init_rc_out) plus live parameters.
  // While armed the frame table / mixer / MOT_PWM_TYPE are queued until disarm
  // (AP_MotorsMatrix::set_frame_class_and_type refuses while armed; FRAME_CONFIG is reboot-required).
  configure({ factors, variant, params, testOrder }) {
    const P = params ?? {};
    if (this.armed && (!sameTable(factors, this.factors) || variant !== this.variant || (P.MOT_PWM_TYPE ?? 0) !== this.pwmType)) {
      this.pending = { factors: factors.map((r) => r.slice()), variant, testOrder, params: P };
      this.setParams(P);
      return;
    }
    this.pending = null;
    this.factors = factors.map((r) => r.slice());
    this.variant = variant;
    this.testOrder = (testOrder ?? this.factors.map((_, i) => i + 1)).slice();
    this.pwmType = Math.trunc(P.MOT_PWM_TYPE ?? 0);
    this.rcSpeed = P.RC_SPEED ?? 200;
    if (!this._booted) {
      // ArduSub/Parameters.cpp:801 forces MOT_THST_HOVER to 0.5 at every boot ("ROVs are neutral buoyancy")
      this._throttleHover = 0.5; this._hoverSeen = P.MOT_THST_HOVER; this._booted = true;
    }
    this.setParams(P);
    const n = this.factors.length;
    if (this.thrust.length !== n) { this.thrust = new Array(n).fill(0); this.pwm = new Array(n).fill(1500); }
  }

  // Live parameters: applied immediately, never resets spool/limiter state.
  setParams(P = {}) {
    for (let i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
      // _motor_reverse is an AP_Int8 used as a factor (AP_Motors6DOF.cpp:357): 0 silences, 2 doubles
      const v = Math.trunc(Number(P[`MOT_${i + 1}_DIRECTION`] ?? 1));
      this.reverse[i] = Number.isFinite(v) ? Math.max(-128, Math.min(127, v)) : 1;
    }
    // Sub::init_rc_out: convert_pwm_min_max_param(RC3_MIN, RC3_MAX) when MOT_PWM_MIN/MAX were never saved,
    // then update_throttle_range forces 1000/2000 for digital (DShot) and PWMRange/PWMAngle outputs
    this.pwmMin = Math.trunc(P.MOT_PWM_MIN ?? P.RC3_MIN ?? 1100);
    this.pwmMax = Math.trunc(P.MOT_PWM_MAX ?? P.RC3_MAX ?? 1900);
    if (isDigital(this.pwmType) || this.pwmType === PWM_TYPE.PWM_RANGE || this.pwmType === PWM_TYPE.PWM_ANGLE) { this.pwmMin = 1000; this.pwmMax = 2000; }
    this.fvCoupling = P.MOT_FV_CPLNG_K ?? 1;
    this.spoolTime = P.MOT_SPOOL_TIME ?? 0.5;
    this.spoolDownTime = P.MOT_SPOOL_TIM_DN ?? 0;
    this.safeDisarm = P.MOT_SAFE_DISARM ?? 0;
    this.safeTime = P.MOT_SAFE_TIME ?? 1;
    this.spinArm = P.MOT_SPIN_ARM ?? 0.1;
    this.spinMin = P.MOT_SPIN_MIN ?? 0.15;
    this.battCurrMax = P.MOT_BAT_CURR_MAX ?? 0;
    this.battCurrTc = P.MOT_BAT_CURR_TC ?? 5;
    if (this._booted && P.MOT_THST_HOVER !== undefined && P.MOT_THST_HOVER !== this._hoverSeen) { this._throttleHover = P.MOT_THST_HOVER; this._hoverSeen = P.MOT_THST_HOVER; }
    const sm = servoChannelMap(P, Math.max(this.factors.length, 0));
    this.servoMap = sm.map; this.servoFunctions = sm.functions;
    this.servoParams = [];
    for (let ch = 0; ch < NUM_SERVO_CHANNELS; ch++) {
      this.servoParams.push({ min: P[`SERVO${ch + 1}_MIN`] ?? 1100, max: P[`SERVO${ch + 1}_MAX`] ?? 1900, trim: P[`SERVO${ch + 1}_TRIM`] ?? 1500, reversed: !!(P[`SERVO${ch + 1}_REVERSED`] ?? 0) });
    }
    let nCh = this.factors.length;
    for (const chs of this.servoMap) for (const c of chs) nCh = Math.max(nCh, c + 1);
    if (this.servoPending.length !== nCh) { this.servoPending = new Array(nCh).fill(1500); this.servoOut = new Array(nCh).fill(1500); }
  }

  static fromFrameConfig(frameConfig) {
    const f = FRAMES[frameConfig] ?? FRAMES[5];
    return { factors: frameTable(frameConfig), variant: f.mixer, testOrder: frameTestOrder(frameConfig) };
  }
  static isFirmwareVariant(v) { return FIRMWARE_VARIANTS.includes(v); }

  setRoll(v) { this.rollIn = v; }
  setPitch(v) { this.pitchIn = v; }
  setYaw(v) { this.yawIn = v; }
  setRollFf(v) { this.rollInFf = v; }
  setPitchFf(v) { this.pitchInFf = v; }
  setYawFf(v) { this.yawInFf = v; }
  setThrottle(v) { this.throttleIn = v; }
  setForward(v) { this.forwardIn = v; }
  setLateral(v) { this.lateralIn = v; }
  setThrottleFilterCutoff(hz) { this.throttleFilter.setCutoff(hz); }
  setThrottleAvgMax(v) { this.throttleAvgMax = constrainFloat(v, 0, 1); }
  setInterlock(v) { this.interlock = !!v; }
  // AP_Motors_Class.cpp:86: requests are ignored while disarmed (except SHUT_DOWN)
  setDesiredSpoolState(s) { if (this.armed || s === SPOOL.SHUT_DOWN) this.desiredSpool = s; }
  getDesiredSpoolState() { return this.desiredSpool; }
  getSpoolState() { return this.spool; }
  getThrottleBidirectional() { return constrainFloat(2 * (this.throttleFilter.get() - 0.5), -1, 1); }
  getThrottle() { return constrainFloat(this.throttleFilter.get(), 0, 1); }
  getThrottleIn() { return this.throttleIn; }
  getThrottleThrustMax() { return this.throttleThrustMax; }
  getForward() { return this.forwardIn; }
  getLateral() { return this.lateralIn; }
  // AP_MotorsMulticopter.h:48 get_throttle_hover()
  get throttleHover() { return constrainFloat(this._throttleHover, AP_MOTORS_THST_HOVER_MIN, AP_MOTORS_THST_HOVER_MAX); }
  set throttleHover(v) { this._throttleHover = v; }
  getThrottleHover() { return this.throttleHover; }
  getCurrentLimitMaxThrottle() { return 1; } // AP_Motors6DOF.cpp:283
  getPwmOutputMin() { return this.pwmMin; }
  getPwmOutputMax() { return this.pwmMax; }
  getMotorMask() { let m = 0; for (let i = 0; i < this.factors.length; i++) m |= 1 << i; return m; }
  motorIsEnabled(i) { return i >= 0 && i < this.factors.length; }
  // AP_Motors6DOF::get_motor_angular_factors (used by MOTOR_DETECT)
  getMotorAngularFactors(i) { if (i < 0 || i >= AP_MOTORS_MAX_NUM_MOTORS || !this.factors[i]) return [0, 0, 0]; return [this.factors[i][0], this.factors[i][1], this.factors[i][2]]; }
  // AP_Motors6DOF::set_reversed: returns the parameter the firmware must store (set_and_save)
  setReversed(i, reversed) {
    if (i < 0 || i >= AP_MOTORS_MAX_NUM_MOTORS) return null;
    this.reverse[i] = reversed ? -1 : 1;
    return { name: `MOT_${i + 1}_DIRECTION`, value: this.reverse[i] };
  }
  // Battery monitor feed (contract §2): current_amps() is available when healthy
  setBatteryReadings(voltage, current, healthy = true) {
    this.battVoltage = voltage; this.battCurrent = current; this.battHealthy = !!healthy;
    this.battHaveCurrent = !!healthy && Number.isFinite(current);
  }

  arm() { this.armed = true; }
  disarm() {
    this.armed = false; this.desiredSpool = SPOOL.SHUT_DOWN;
    if (this.pending) { const p = this.pending; this.pending = null; this.configure(p); }
  }
  setArmed(a) { if (a) this.arm(); else this.disarm(); }

  // AP_MotorsMulticopter::output_logic (AP_MotorsMulticopter.cpp:489)
  outputLogic(dt) {
    if (this.armed) {
      if (this.safeDisarm && this.disarmSafeTimer < this.safeTime) this.disarmSafeTimer += dt;
      else this.disarmSafeTimer = this.safeTime;
    } else this.disarmSafeTimer = 0;
    if (!this.armed || !this.interlock) { this.desiredSpool = SPOOL.SHUT_DOWN; this.spool = SPOOL.SHUT_DOWN; }
    const up = Math.max(this.spoolTime, MIN_SPOOL_TIME);
    const dn = this.spoolDownTime > MIN_SPOOL_TIME ? this.spoolDownTime : up;
    const L = this.limit;
    const setLim = (v) => { L.roll = L.pitch = L.yaw = L.throttle_lower = L.throttle_upper = v; };
    switch (this.spool) {
      case SPOOL.SHUT_DOWN:
        setLim(true);
        if (this.desiredSpool !== SPOOL.SHUT_DOWN && this.disarmSafeTimer >= this.safeTime) { this.spool = SPOOL.GROUND_IDLE; break; }
        this.spinUpRatio = 0; this.throttleThrustMax = 0;
        break;
      case SPOOL.GROUND_IDLE:
        setLim(true);
        if (this.desiredSpool === SPOOL.SHUT_DOWN) {
          this.spinUpRatio -= dt / dn;
          if (this.spinUpRatio <= 0) { this.spinUpRatio = 0; this.spool = SPOOL.SHUT_DOWN; }
        } else if (this.desiredSpool === SPOOL.THROTTLE_UNLIMITED) {
          this.spinUpRatio += dt / up;
          if (this.spinUpRatio >= 1) { this.spinUpRatio = 1; this.spool = SPOOL.SPOOLING_UP; }
        } else if (this.desiredSpool === SPOOL.GROUND_IDLE) {
          const armedRatio = this.spinMin > 0 ? this.spinArm / this.spinMin : 0;
          this.spinUpRatio += constrainFloat(armedRatio - this.spinUpRatio, -dt / dn, dt / up);
        }
        this.throttleThrustMax = 0;
        break;
      case SPOOL.SPOOLING_UP:
        setLim(false);
        if (this.desiredSpool !== SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_DOWN; break; }
        this.spinUpRatio = 1;
        this.throttleThrustMax += dt / up;
        if (this.throttleThrustMax >= Math.min(this.getThrottle(), this.getCurrentLimitMaxThrottle())) { this.throttleThrustMax = this.getCurrentLimitMaxThrottle(); this.spool = SPOOL.THROTTLE_UNLIMITED; }
        else if (this.throttleThrustMax < 0) this.throttleThrustMax = 0;
        break;
      case SPOOL.THROTTLE_UNLIMITED:
        setLim(false);
        if (this.desiredSpool !== SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_DOWN; break; }
        this.spinUpRatio = 1;
        this.throttleThrustMax = this.getCurrentLimitMaxThrottle();
        break;
      case SPOOL.SPOOLING_DOWN:
        setLim(false);
        if (this.desiredSpool === SPOOL.THROTTLE_UNLIMITED) { this.spool = SPOOL.SPOOLING_UP; break; }
        this.spinUpRatio = 1;
        this.throttleThrustMax -= dt / dn;
        if (this.throttleThrustMax <= 0) this.throttleThrustMax = 0;
        if (this.throttleThrustMax >= this.getCurrentLimitMaxThrottle()) this.throttleThrustMax = this.getCurrentLimitMaxThrottle();
        else if (isZero(this.throttleThrustMax)) this.spool = SPOOL.GROUND_IDLE;
        break;
    }
  }

  // AP_MotorsMulticopter::update_throttle_filter
  updateThrottleFilter(dt) {
    if (this.armed) {
      this.throttleFilter.apply(this.throttleIn, dt);
      if (this.throttleFilter.get() < 0) this.throttleFilter.reset(0);
      if (this.throttleFilter.get() > 1) this.throttleFilter.reset(1);
    } else this.throttleFilter.reset(0);
  }

  // Sub::motors_output() + AP_MotorsMulticopter::output(). Returns the PWM per servo output channel
  // (index = SERVOn - 1) as the ESCs see it this loop.
  output(dt) {
    // SRV_Channels::output_ch_all() before motors.output(): hardware gets last loop's values
    const hw = this.servoPending.slice();
    if (this.test && this.armed) {
      // ap.motor_test: only verify_motor_test() runs, motors.output() is skipped (ArduSub/motors.cpp:16)
      if (!this._testing) { this.outputMin(); this._testing = true; }
      this.outputTestNum(this.test.motor, this.test.pwm);
    } else {
      this._testing = false;
      this.updateThrottleFilter(dt);
      this.outputLogic(dt);
      this.outputArmedStabilizing(dt);
      this.outputToMotors();
    }
    return this.escFrame(hw, dt);
  }

  // Loops where Sub::motors_output() skips motors.output() (MOTOR_DETECT, ap.motor_test): the
  // channels keep whatever output_test_num()/rc_write() last set; returns the ESC frame.
  outputPassive(dt) { return this.escFrame(this.servoPending.slice(), dt); }

  // RC_SPEED pulse train for PWM/brushed outputs; DShot and OneShot frames go out every push()
  escFrame(hw, dt) {
    const t = this.pwmType;
    const perPush = isDigital(t) || t === PWM_TYPE.ONESHOT || t === PWM_TYPE.ONESHOT125;
    const period = 1 / Math.max(1, this.rcSpeed || 50);
    this.escTimer += dt;
    let latch = perPush;
    if (!perPush && this.escTimer >= period - 1e-9) {
      latch = true;
      this.escTimer -= period;
      if (this.escTimer >= period || this.escTimer < 1e-9) this.escTimer = Math.max(0, this.escTimer % period);
    }
    if (latch) for (let ch = 0; ch < hw.length; ch++) this.servoOut[ch] = hw[ch];
    if (this.servoOut.length !== hw.length) this.servoOut.length = hw.length;
    return this.servoOut;
  }

  thrustToPwm(t) {
    // AP_Motors6DOF::calc_thrust_to_pwm (int16 truncation)
    const up = this.pwmMax - 1500, down = 1500 - this.pwmMin;
    return Math.trunc(1500 + t * (t > 0 ? up : down));
  }

  // AP_Motors::rc_write -> every servo channel carrying this motor's function
  rcWrite(i, pwm) {
    this.pwm[i] = pwm;
    const chs = this.servoMap[i] ?? [i];
    const scaled = this.pwmType === PWM_TYPE.PWM_RANGE || this.pwmType === PWM_TYPE.PWM_ANGLE;
    for (const ch of chs) {
      if (ch >= this.servoPending.length) continue;
      if (!scaled) { this.servoPending[ch] = pwm; continue; }
      const s = this.servoParams[ch];
      if (this.pwmType === PWM_TYPE.PWM_RANGE) { // set_range(1000), offset 1000 (SRV_Channel::pwm_from_range)
        if (s.max <= s.min) { this.servoPending[ch] = s.min; continue; }
        let v = constrainFloat(pwm - 1000, 0, 1000);
        if (s.reversed) v = 1000 - v;
        this.servoPending[ch] = s.min + Math.trunc((v * (s.max - s.min)) / 1000);
      } else { // set_angle(500), offset 1500 (SRV_Channel::pwm_from_angle)
        let v = pwm - 1500;
        if (s.reversed) v = -v;
        v = constrainFloat(v, -500, 500);
        this.servoPending[ch] = v > 0 ? s.trim + Math.trunc((v * (s.max - s.trim)) / 500) : s.trim - Math.trunc((-v * (s.trim - s.min)) / 500);
      }
    }
  }

  // AP_Motors6DOF::output_min
  outputMin() {
    const L = this.limit;
    L.roll = L.pitch = L.yaw = true; L.throttle_lower = L.throttle_upper = false;
    for (let i = 0; i < this.factors.length; i++) this.rcWrite(i, 1500);
  }

  // AP_Motors6DOF::output_to_motors
  outputToMotors() {
    const spinning = this.spool === SPOOL.SPOOLING_UP || this.spool === SPOOL.THROTTLE_UNLIMITED || this.spool === SPOOL.SPOOLING_DOWN;
    for (let i = 0; i < this.factors.length; i++) this.rcWrite(i, spinning ? this.thrustToPwm(this.thrust[i]) : 1500);
  }

  // AP_MotorsMatrix::output_test_num: raw PWM (no clamp) to one motor, others keep their value
  outputTestNum(ch, pwm) {
    if (!this.armed) return false;
    ch = Math.trunc(ch);
    if (!(ch >= 0) || ch > AP_MOTORS_MAX_NUM_MOTORS - 1) return false;
    if (!this.motorIsEnabled(ch)) return false;
    this.rcWrite(ch, Math.trunc(pwm));
    return true;
  }
  // AP_MotorsMatrix::_output_test_seq: by testing order (1..N)
  outputTestSeq(seq, pwm) {
    if (!this.armed) return;
    for (let i = 0; i < this.factors.length; i++) if (this.testOrder[i] === seq) this.rcWrite(i, Math.trunc(pwm));
  }
  // ArduSub MOTOR_DETECT drives the thrusters directly (mode_motordetect.cpp): raw pwm on motor i
  outputDirect(i, pwm) { if (this.motorIsEnabled(i)) this.rcWrite(i, Math.trunc(pwm)); }

  outputArmedStabilizing(dt = 0) {
    const n = this.factors.length;
    const roll = this.rollIn + this.rollInFf, pitch = this.pitchIn + this.pitchInFf, yaw = this.yawIn + this.yawInFf;
    let thr = this.getThrottleBidirectional();
    const fwd = this.forwardIn, lat = this.lateralIn;
    const L = this.limit;
    L.roll = L.pitch = L.yaw = L.throttle_lower = L.throttle_upper = false;
    const tmax = this.throttleThrustMax;
    if (thr <= -tmax) { thr = -tmax; L.throttle_lower = true; }
    if (thr >= tmax) { thr = tmax; L.throttle_upper = true; }
    const F = this.factors, rev = this.reverse;
    const out = this.thrust;
    this.saturated = false;
    if (this.variant === 'vectored6dof') {
      let rptMax = 1, yflMax = 1;
      const rpt = new Array(n), yfl = new Array(n);
      for (let i = 0; i < n; i++) {
        rpt[i] = roll * F[i][0] + pitch * F[i][1] + thr * F[i][3];
        yfl[i] = yaw * F[i][2] + fwd * F[i][4] + lat * F[i][5];
        rptMax = Math.max(rptMax, Math.abs(rpt[i]));
        yflMax = Math.max(yflMax, Math.abs(yfl[i]));
      }
      this.saturated = rptMax > 1 || yflMax > 1;
      for (let i = 0; i < n; i++) out[i] = constrainFloat(rev[i] * (rpt[i] / rptMax + yfl[i] / yflMax), -1, 1);
    } else if (this.variant === 'vectored') {
      const fcLimit = Math.max(0, 1 - this.fvCoupling * Math.abs(thr));
      const dir = [-1, -1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0];
      for (let i = 0; i < n; i++) {
        const rpy = roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2];
        let f = fwd;
        if (!isZero(f) && dir[i] !== 0 && (fwd < 0) === (dir[i] < 0)) f = constrainFloat(fwd, -fcLimit, fcLimit);
        const lin = thr * F[i][3] + f * F[i][4] + lat * F[i][5];
        const raw = rev[i] * (rpy + lin);
        if (Math.abs(raw) > 1) this.saturated = true;
        out[i] = constrainFloat(raw, -1, 1);
      }
    } else if (this.variant === 'scale' || this.variant === 'priority') {
      // simulator-only variants (not in ArduSub)
      const att = new Array(n), lin = new Array(n);
      for (let i = 0; i < n; i++) {
        att[i] = roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2];
        lin[i] = thr * F[i][3] + fwd * F[i][4] + lat * F[i][5];
      }
      if (this.variant === 'scale') {
        let m = 1;
        for (let i = 0; i < n; i++) m = Math.max(m, Math.abs(att[i] + lin[i]));
        this.saturated = m > 1;
        for (let i = 0; i < n; i++) out[i] = constrainFloat(rev[i] * ((att[i] + lin[i]) / m), -1, 1);
      } else {
        let ma = 1;
        for (let i = 0; i < n; i++) ma = Math.max(ma, Math.abs(att[i]));
        for (let i = 0; i < n; i++) att[i] /= ma;
        // largest s in [0,1] so that |att + s*lin| <= 1 for every motor
        let s = 1;
        for (let i = 0; i < n; i++) {
          const a = att[i], l = lin[i];
          if (Math.abs(l) < 1e-12) continue;
          const lim = l > 0 ? (1 - a) / l : (-1 - a) / l;
          s = Math.min(s, Math.max(0, lim));
        }
        this.saturated = ma > 1 || s < 1;
        for (let i = 0; i < n; i++) out[i] = constrainFloat(rev[i] * (att[i] + s * lin[i]), -1, 1);
      }
    } else {
      for (let i = 0; i < n; i++) {
        const raw = rev[i] * (roll * F[i][0] + pitch * F[i][1] + yaw * F[i][2] + thr * F[i][3] + fwd * F[i][4] + lat * F[i][5]);
        if (Math.abs(raw) > 1) this.saturated = true;
        out[i] = constrainFloat(raw, -1, 1);
      }
    }
    this.batteryCurrentLimit(dt);
  }

  // AP_Motors6DOF.cpp:362 current limiting (runs after every mixer branch)
  batteryCurrentLimit(dt) {
    if (this.battCurrMax <= 0 || !this.battHaveCurrent || !isPositive(dt)) return;
    const I = this.battCurrent, Imax = this.battCurrMax;
    const rate = (I - this.battCurrentLast) / dt;
    const predicted = I + rate * dt * 5;
    let ratio = I / Imax;
    const predictedRatio = predicted / Imax;
    this.battCurrentLast = I;
    if (predicted > Imax * 1.5) ratio = 2.5;
    else if (I < Imax && predicted > Imax) ratio = predictedRatio;
    this.outputLimited = constrainFloat(this.outputLimited + (dt / (dt + this.battCurrTc)) * (1 - ratio), 0, 1);
    for (let i = 0; i < this.thrust.length; i++) this.thrust[i] *= this.outputLimited;
  }
}

// ArduSub/motors.cpp:29-134 MAV_CMD_DO_MOTOR_TEST handling (init_motor_test, verify_motor_test,
// handle_do_motor_test). host = { motors, isArmed(), disarm(reason), text(sev, msg), throttleRange() -> [RC3_MIN, RC3_MAX] }.
export const MOTOR_TEST_THROTTLE = { PERCENT: 0, PWM: 1, PILOT: 2 };
export const MOTOR_TEST_ORDER_BOARD = 2;
export class SubMotorTest {
  constructor(host) { this.host = host; this.active = false; this.lastCmdMs = 0; this.lastFailMs = 0; this.lastInitFailMs = 0; }
  initMotorTest(nowMs) {
    const h = this.host;
    if (nowMs < this.lastFailMs + 10000 && this.lastFailMs > 0) { h.text?.(2, '10 second cooldown required after motor test'); return false; }
    if (!h.isArmed()) { h.text?.(4, 'Arm motors before testing motors.'); return false; }
    h.motors.outputMin();
    this.active = true;
    return true;
  }
  // returns false when the test timed out (vehicle disarmed, 10 s lockout)
  verify(nowMs) {
    if (!this.active) return true;
    if (nowMs > this.lastCmdMs + 500) {
      this.host.text?.(6, 'Motor test timed out!');
      this.active = false;
      this.host.disarm?.('motortest');
      this.lastFailMs = nowMs;
      return false;
    }
    return true;
  }
  handle(nowMs, { param1 = 0, param2 = MOTOR_TEST_THROTTLE.PWM, param3 = 1500, y = MOTOR_TEST_ORDER_BOARD } = {}) {
    const h = this.host;
    this.lastCmdMs = nowMs;
    if (!this.active) {
      if (nowMs > this.lastInitFailMs + 2000) {
        if (!this.initMotorTest(nowMs)) { h.text?.(4, 'motor test initialization failed!'); this.lastInitFailMs = nowMs; return false; }
      } else return false;
    }
    if (y !== MOTOR_TEST_ORDER_BOARD) { h.text?.(4, `bad test type ${Number(y).toFixed(2)}`); return false; }
    if (param2 === MOTOR_TEST_THROTTLE.PILOT) { h.text?.(4, `bad throttle type ${Number(param2).toFixed(2)}`); return false; }
    // output_test_num(motor_number, ...) receives param1 as the 0-based output index, as in 4.5.7
    if (param2 === MOTOR_TEST_THROTTLE.PWM) return h.motors.outputTestNum(param1, param3);
    if (param2 === MOTOR_TEST_THROTTLE.PERCENT) {
      const [mn, mx] = h.throttleRange?.() ?? [1100, 1900];
      const thr = mn + (constrainFloat(param3, 0, 100) / 100) * (mx - mn);
      return h.motors.outputTestNum(param1, thr);
    }
    return false;
  }
  // per loop while active (instead of motors.output()): returns the ESC frame
  output(dt) { return this.host.motors.outputPassive(dt); }
}
