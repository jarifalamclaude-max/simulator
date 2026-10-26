// ArduSub 4.5.7 "firmware" emulation: AP_Scheduler task table (ArduSub.cpp:48-114),
// arming (AP_Arming_Sub + AP_Arming), all flight modes (mode_*.cpp), pilot input,
// surface/bottom detection and every failsafe of failsafe.cpp, wired to the
// ported attitude controller, position controller and 6-DOF motor mixer.

import { AttitudeControlSub, eulerRateToAngVel } from './attitude.js';
import { PosControl } from './poscontrol.js';
import * as MotorsMod from './motors.js';
import { makeSubChannels, configureSubChannels } from './rc.js';
import { JoystickHandler } from './joystick.js';
import { WPNav, CircleNav } from './navigation.js';
import { RF_STATUS } from './sensors.js';
import { clamp, wrap180cd, wrap360cd, qToEuler, qFromEuler, RAD } from '../core/math.js';

const { Motors6DOF, SPOOL } = MotorsMod;

export const MODE = { STABILIZE: 0, ACRO: 1, ALT_HOLD: 2, AUTO: 3, GUIDED: 4, CIRCLE: 7, SURFACE: 9, POSHOLD: 16, MANUAL: 19, MOTOR_DETECT: 20, SURFTRAK: 21 };
export const MODE_LABEL = { 0: 'Stabilize', 1: 'Acro', 2: 'Depth Hold', 3: 'Auto', 4: 'Guided', 7: 'Circle', 9: 'Surface', 16: 'Position Hold', 19: 'Manual', 20: 'Motor Detect', 21: 'SurfTrak' };
// Mode::name() (mode.h), used in the firmware's status texts
export const MODE_NAME = { 0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO', 4: 'GUIDED', 7: 'CIRCLE', 9: 'SURFACE', 16: 'POSHOLD', 19: 'MANUAL', 20: 'MOTORDETECT', 21: 'SURFTRAK' };
export const SIM_MODES = [MODE.MANUAL, MODE.STABILIZE, MODE.ACRO, MODE.ALT_HOLD, MODE.SURFTRAK, MODE.POSHOLD, MODE.SURFACE, MODE.GUIDED, MODE.AUTO, MODE.CIRCLE, MODE.MOTOR_DETECT];
// requires_GPS() / has_manual_throttle() per mode.h:188-484
export const MODE_INFO = {
  [MODE.MANUAL]: { manualThrottle: true, gps: false },
  [MODE.STABILIZE]: { manualThrottle: true, gps: false },
  [MODE.ACRO]: { manualThrottle: true, gps: false },
  [MODE.ALT_HOLD]: { manualThrottle: false, gps: false },
  [MODE.SURFTRAK]: { manualThrottle: false, gps: false },
  [MODE.GUIDED]: { manualThrottle: false, gps: true },
  [MODE.AUTO]: { manualThrottle: false, gps: true },
  [MODE.POSHOLD]: { manualThrottle: false, gps: false },
  [MODE.CIRCLE]: { manualThrottle: false, gps: true },
  [MODE.SURFACE]: { manualThrottle: false, gps: true },
  [MODE.MOTOR_DETECT]: { manualThrottle: false, gps: false },
};
const ROLL_PITCH_INPUT_MAX = 4500;
const ACRO_YAW_P_DEFAULT = 3.375;
const ACRO_LEVEL_MAX_ANGLE = 3000;
const MAIN_LOOP_RATE = 400; // config.h:16 (surface/bottom detector counts)
const BOTTOM_DETECTOR_TRIGGER_SEC = 1, SURFACE_DETECTOR_TRIGGER_SEC = 1; // defines.h:13-14
const FS_GCS_TIMEOUT_MS = 2500; // config.h:83
const FS_TERRAIN_TIMEOUT_MS = 1000, FS_TERRAIN_RECOVER_TIMEOUT_MS = 10000; // defines.h:88,148
const RANGEFINDER_TIMEOUT_MS = 1000, RANGEFINDER_WPNAV_FILT_HZ = 0.25; // config.h:58,62
const AUTO_YAW_SLEW_RATE = 60; // config.h:135
const GUIDED_POSVEL_TIMEOUT_MS = 3000, GUIDED_ATTITUDE_TIMEOUT_MS = 1000; // mode_guided.cpp:7-8
const SURFTRAK_INVALID_TARGET = -1;
const PREARM_DISPLAY_PERIOD = 30;
const AUTO_YAW = { HOLD: 0, LOOK_AT_NEXT_WP: 1, ROI: 2, LOOK_AT_HEADING: 3, LOOK_AHEAD: 4, RESETTOARMEDYAW: 5, CORRECT_XTRACK: 6, RATE: 7 };
export const ARMING_CHECK = { ALL: 1, BARO: 2, COMPASS: 4, GPS: 8, INS: 16, PARAMETERS: 32, RC: 64, VOLTAGE: 128, BATTERY: 256, AIRSPEED: 512, LOGGING: 1024, SWITCH: 2048, GPS_CONFIG: 4096, SYSTEM: 8192, MISSION: 16384, RANGEFINDER: 32768 };
// Sub::_failsafe_priorities (Sub.h:577-583): Disarm, Surface, Warn, None
const BATT_FS_PRIORITY = [2, 3, 1, 0];

// ArduSub scheduler_tasks[] (ArduSub.cpp:63-114) and the AP_Vehicle tasks it interleaves with, in priority order
const TASKS = [
  ['fiftyHzLoop', 50, 3],
  ['updateBattCompass', 10, 12],
  ['readRangefinder', 20, 15],
  ['updateAltitude', 10, 18],
  ['threeHzLoop', 3, 21],
  ['updateTurnCounter', 10, 24],
  ['baroAccumulate', 50, 27],
  ['oneHzLoop', 1, 33],
  ['gcsUpdateReceive', 400, 36],
  ['terrainUpdate', 10, 72],
  ['temperatureSensorUpdate', 5, 242],
  ['updateArming', 1, 253],
];
export const SCHEDULER_TASKS = TASKS.map(([name, rate, prio]) => ({ name, rate, prio }));

export class ArduSubSim {
  constructor({ params, mixer, sensors }) {
    this.params = { ...params };
    this.sensors = sensors;
    this.motors = new Motors6DOF();
    this.att = new AttitudeControlSub(sensors, this.motors);
    this.pos = new PosControl(sensors, this.motors, this.att);
    this.rc = makeSubChannels(this.params);
    this.rc.deferRead = true; // overrides reach the channels in rc().read_input() at 50 Hz (fifty_hz_loop)
    this.js = new JoystickHandler(this);
    this.wpNav = new WPNav(this);
    this.circleNav = new CircleNav(this);
    this.messages = [];
    this.time = 0;
    this.tickCounter = 0;
    this.taskLastRun = TASKS.map(() => 0);
    this.taskRuns = Object.fromEntries(TASKS.map(([n]) => [n, 0]));
    this.armed = false;
    this.gain = 0.5;
    this.mode = MODE.MANUAL;
    this.prevMode = MODE.MANUAL;
    this.modeReason = 'boot';
    this.lastPilotHeading = 0;
    this.lastPilotYawInputMs = 0;
    this.desiredClimbRate = 0;
    this.atSurface = false; this.atBottom = false; this.bottomCount = 0; this.surfaceCount = 0;
    // Sub::failsafe (Sub.h:222-241); pilot_input starts latched (Sub.cpp:47)
    this.failsafe = {
      pilotInput: true, gcs: false, ekf: false, terrain: false, leak: false, internalPressure: false, internalTemp: false, crash: false, sensorHealth: false, battery: false,
      lastPilotInput: 0, lastLeakWarnMs: 0, lastGcsWarnMs: 0, lastCrashWarnMs: 0, lastEkfWarnMs: 0, terrainFirstFailureMs: 0, terrainLastFailureMs: 0,
      lastPressGoodMs: 0, lastPressWarnMs: 0, lastTempGoodMs: 0, lastTempWarnMs: 0, lastCrashPassMs: 0, lastEkfGoodMs: 0,
    };
    this.gcsLastSeenMs = 0; // gcs().sysid_myggcs_last_seen_time_ms()
    this.inbox = [];
    this.battFs = { state: 0, highestPriority: 127, lowStartMs: 0, critStartMs: 0 }; // AP_BattMonitor failsafe (0 none, 1 low, 2 critical)
    this.rangefinderState = { enabled: false, altHealthy: false, altCm: 0, inertialAltCm: 0, minCm: 20, maxCm: 700, lastHealthyMs: 0, altCmFilt: null, terrainOffsetCm: 0 };
    this.surftrak = { targetCm: SURFTRAK_INVALID_TARGET, pilotInControl: false, pilotControlStartZCm: 0 };
    this.quarterTurnCount = 0; this.lastTurnState = 0;
    this.attitudeTargetMsg = null;
    this._depthTarget = null; this._depthTargetPending = null;
    this.motorTest = null; // simulator bench test {motor, pwm} (motors.test); MAVLink DO_MOTOR_TEST via doMotorTest()
    this.motorTestProto = MotorsMod.SubMotorTest ? new MotorsMod.SubMotorTest({ motors: this.motors, isArmed: () => this.armed, disarm: (r) => this.disarm(r), text: (s, m) => this.text(s, m), throttleRange: () => [this.rc.throttle.min, this.rc.throttle.max] }) : null;
    this.md = { state: 0, motor: 0, dir: 1, settleMs: 0, thrustMs: 0 };
    this.mission = { items: [], index: -1, state: 'stopped', loiterStartMs: 0, started: false };
    this.autoMode = 'loiter'; this.autoYawMode = AUTO_YAW.HOLD; this.yawLookAtHeading = 0; this.yawLookAtHeadingSlew = 0; this.yawRateOnly = false;
    this.guidedMode = 'wp'; this.guidedUpdateMs = 0; this.guidedAngle = { ms: 0, roll: 0, pitch: 0, yaw: 0, climb: 0 };
    this.circlePilotYawOverride = false;
    this.fsTerrainRecoverStartMs = 0; this.rfRecoveryMs = 0;
    this.preArmOk = false; this.lastPrearmDisplayMs = -1e9; this.prearmReportNow = false; this.lastPrearmResult = true;
    this.ekfOriginSet = false;
    this.mixer = mixer;
    this.applyParams(this.params, true);
    this.js.init(this.params);
    // init_ardupilot (system.cpp:122-145): depth sensor detection is latched at boot
    this.depthSensorPresent = !!sensors.depthSensorPresent;
    this.sensorHealthDepth = this.depthSensorPresent && !!sensors.depthHealthy;
    this.lastPilotHeading = sensors.yawSensor ?? 0;
  }

  text(severity, msg) {
    this.messages.push({ t: this.time, severity, text: msg });
    if (this.messages.length > 200) this.messages.shift();
  }
  get ms() { return Math.floor(this.time * 1000 + 1e-6); }
  get loopHz() { return this._loopHz ?? clamp(this.params.SCHED_LOOP_RATE || 400, 50, 1000); }

  // Live parameter edits: gains and limits change in place, no controller reset, no disarm.
  applyParams(P, first = false) {
    this.params = { ...this.params, ...P };
    const p = this.params;
    // Sub::get_pilot_desired_lean_angles stores the clamp in aparm.angle_max (ArduSub/Attitude.cpp:8)
    if (p.ANGLE_MAX !== undefined) p.ANGLE_MAX = clamp(Math.trunc(p.ANGLE_MAX), 1000, 8000);
    this.att.setParams(p);
    this.pos.setParams(p);
    if (first || !this.motors.setParams) this.motors.configure({ factors: this.mixer.factors, variant: this.mixer.variant, params: p, testOrder: this.mixer.testOrder });
    else this.motors.setParams(p);
    configureSubChannels(this.rc, p);
    this.sensors.setParams?.(p);
    const dt = 1 / this.loopHz;
    if (first) { this.att.dt = dt; this.pos.dt = dt; }
    if (!first) this.text(6, 'Parameters updated');
  }
  setParams(P) { this.applyParams(P); }
  setMixer(mixer) {
    this.mixer = mixer;
    this.motors.configure({ factors: mixer.factors, variant: mixer.variant, params: this.params, testOrder: mixer.testOrder });
  }
  // the vehicle was rebuilt around the firmware (sensors/mixer replaced, state kept)
  rebind({ sensors, mixer, params }) {
    if (sensors && sensors !== this.sensors) { this.sensors = sensors; this.att.ahrs = sensors; this.pos.ahrs = sensors; }
    if (mixer) this.setMixer(mixer);
    if (params) { this.params = { ...this.params, ...params }; this.applyParams({}, false); this.messages.pop(); }
  }
  saveParam(name, value) {
    if (this.js.setAndSave) this.js.setAndSave(name, value);
    else this.params[name] = value;
  }

  positionOk() { return !this.failsafe.ekf && this.sensors.positionOk(); } // Sub::position_ok (system.cpp:212)

  // ---------------- arming (AP_Arming_Sub, AP_Arming) ----------------
  checkEnabled(bit) { const c = this.params.ARMING_CHECK ?? 448; return (c & ARMING_CHECK.ALL) !== 0 || (c & bit) !== 0; }
  checkFailed(report, msg) { if (report) this.text(2, `${this.runningArmingChecks ? 'Arm' : 'PreArm'}: ${msg}`); }
  hasDisarmFunction() {
    const f = (fn) => this.js.functionIsAssigned?.(fn) ?? this.functionIsAssigned(fn);
    return f(2) || f(4); // k_arm_toggle, k_disarm
  }
  functionIsAssigned(fn) { // Sub::jsbutton_function_is_assigned (joystick.cpp:838)
    const P = this.params;
    let shift = false, onShift = false;
    for (let i = 0; i < 32; i++) {
      if ((P[`BTN${i}_FUNCTION`] ?? 0) === fn) return true;
      if ((P[`BTN${i}_FUNCTION`] ?? 0) === 1) shift = true;
      if ((P[`BTN${i}_SFUNCTION`] ?? 0) === fn) onShift = true;
    }
    return shift && onShift;
  }
  preArmChecks(report = true) {
    if (this.armed) return true;
    if (!this.hasDisarmFunction()) { this.checkFailed(report, 'Must assign a disarm or arm_toggle button'); return false; }
    if (!this.mixer.factors.length) { this.checkFailed(report, 'No motors configured'); return false; }
    const s = this.sensors;
    let ok = true;
    // barometer_checks
    if (this.checkEnabled(ARMING_CHECK.BARO) && this.depthSensorPresent && !s.depthHealthy) { this.checkFailed(report, 'Barometer not healthy'); ok = false; }
    // ins_checks / AHRS
    if (this.checkEnabled(ARMING_CHECK.INS) && s.faults?.imu) { this.checkFailed(report, 'Gyros not healthy'); ok = false; }
    // compass_checks
    if (this.checkEnabled(ARMING_CHECK.COMPASS) && s.compass?.used && s.compass?.enabled && !s.compass?.healthy && s.compassReadOnce) { this.checkFailed(report, 'Compass not healthy'); ok = false; }
    // battery_checks (AP_BattMonitor_Backend::arming_checks)
    if (this.checkEnabled(ARMING_CHECK.BATTERY)) { const e = this.batteryArmingCheck(); if (e) { this.checkFailed(report, e); ok = false; } }
    // manual_transmitter_checks -> rc_checks_copter_sub on roll, pitch, throttle, yaw
    if (this.checkEnabled(ARMING_CHECK.RC)) {
      const names = ['Roll', 'Pitch', 'Throttle', 'Yaw'], ch = [this.rc.roll, this.rc.pitch, this.rc.throttle, this.rc.yaw];
      ch.forEach((c, k) => {
        if (c.min > 1300) { this.checkFailed(report, `${names[k]} radio min too high`); ok = false; }
        if (c.max < 1700) { this.checkFailed(report, `${names[k]} radio max too low`); ok = false; }
      });
    }
    // rangefinder_checks
    if (this.checkEnabled(ARMING_CHECK.RANGEFINDER) && (this.params.RNGFND1_TYPE ?? 0) !== 0) {
      const st = s.rangefinder?.status;
      if (st === RF_STATUS.NotConnected || st === RF_STATUS.NoData) { this.checkFailed(report, `Rangefinder 1: ${st === RF_STATUS.NotConnected ? 'Not Connected' : 'No Data'}`); ok = false; }
    }
    // mission_checks (ARMING_MIS_ITEMS)
    if (this.checkEnabled(ARMING_CHECK.MISSION) && (this.params.ARMING_MIS_ITEMS ?? 0) !== 0 && this.mission.items.length === 0) { this.checkFailed(report, 'No mission'); ok = false; }
    return ok;
  }
  // list of the pre-arm failures (HUD); preArm() kept for older callers (null = ok)
  armingCheckFailures() {
    const keep = this.messages.length, was = this.runningArmingChecks;
    this.runningArmingChecks = false;
    this.preArmChecks(true);
    this.runningArmingChecks = was;
    const out = this.messages.splice(keep).map((m) => m.text.replace(/^(PreArm|Arm): /, ''));
    return this.armed ? [] : out;
  }
  preArm() { const f = this.armingCheckFailures(); return f.length ? f[0] : null; }
  batteryArmingCheck() {
    const b = this.sensors.battery, P = this.params;
    if (!b?.present) return '';
    const lowV = P.BATT_LOW_VOLT ?? 0, crtV = P.BATT_CRT_VOLT ?? 0, lowM = P.BATT_LOW_MAH ?? 0, crtM = P.BATT_CRT_MAH ?? 0, cap = P.BATT_CAPACITY ?? 3300;
    const t = this.battFailsafeTypes();
    const chk = [
      [!b.healthy, 'unhealthy'],
      [(P.BATT_ARM_VOLT ?? 0) > 0 && b.voltage < P.BATT_ARM_VOLT, 'below minimum arming voltage'],
      [(P.BATT_ARM_MAH ?? 0) > 0 && cap - b.consumedMah < P.BATT_ARM_MAH, 'below minimum arming capacity'],
      [t.lowV, 'low voltage failsafe'], [t.lowC, 'low capacity failsafe'], [t.crtV, 'critical voltage failsafe'], [t.crtC, 'critical capacity failsafe'],
      [crtM > 0 && lowM > 0 && !(lowM > crtM), 'capacity failsafe critical >= low'],
      [crtV > 0 && lowV > 0 && !(lowV > crtV), 'voltage failsafe critical >= low'],
    ];
    for (const [bad, msg] of chk) if (bad) return `Battery 1 ${msg}`;
    return '';
  }
  // AP_Arming_Sub::arm. Returns false when already armed or a check failed. A failed
  // post-arm origin check (ARMING_CHECK PARAMETERS) returns false with the motors armed, as 4.5.7 does.
  arm(source = 'gcs', doArmingChecks = true) {
    if (this.armed) return false;
    this.runningArmingChecks = true;
    const ok = !doArmingChecks || this.preArmChecks(true);
    this.runningArmingChecks = false;
    if (!ok) return false;
    if (doArmingChecks && (this.params.ARMING_CHECK ?? 448) === 0) this.text(4, 'Warning: Arming Checks Disabled');
    this.armed = true;
    this.initialArmedBearing = this.sensors.yawSensor;
    this.motors.outputMin?.(); // enable_motor_output
    this.motors.arm();
    this.text(6, 'Armed');
    this.lastPilotHeading = this.sensors.yawSensor;
    if (!this.ensureEkfOrigin()) {
      this.text(4, 'Compass performance degraded');
      if (this.checkEnabled(ARMING_CHECK.PARAMETERS)) { this.runningArmingChecks = true; this.checkFailed(true, 'No world position, check ORIGIN_* parameters'); this.runningArmingChecks = false; return false; }
    }
    return true;
  }
  // Sub::ensure_ekf_origin (ArduSub.cpp:383-428)
  ensureEkfOrigin() {
    if (this.ekfOriginSet) return true;
    const P = this.params;
    if ((P.GPS_TYPE ?? 1) !== 0) return false; // wait for the GPS to set the origin
    const lat = P.ORIGIN_LAT ?? 0, lon = P.ORIGIN_LON ?? 0;
    if (lat === 0 || lon === 0) { this.text(4, 'Backup location parameters are missing or zero'); return false; }
    if (Math.abs(lat) > 90 || Math.abs(lon) > 180) { this.text(4, 'Backup location parameters are not valid'); return false; }
    this.ekfOriginSet = true;
    this.text(6, 'Using backup location');
    return true;
  }
  disarm(source = 'gcs') {
    if (!this.armed) return false;
    this.armed = false;
    this.motors.disarm();
    this.mission.state = 'stopped'; this.mission.index = -1; // mission.reset()
    this.js.clearInputHold();
    this.motorTest = null;
    this.motors.test = null;
    if (this.motorTestProto) this.motorTestProto.active = false;
    this.text(6, 'Disarmed');
    return true;
  }

  // ---------------- modes (mode.cpp) ----------------
  setMode(m, reason = 'gcs') {
    if (typeof m === 'string') m = MODE[m];
    if (m === this.mode) { this.modeReason = reason; return true; }
    const info = MODE_INFO[m];
    if (!info) { this.text(4, `No such mode ${m}`); return false; }
    const name = MODE_NAME[m];
    if (info.gps && !this.positionOk()) { this.text(4, `Mode change failed: ${name} requires position`); return false; }
    if (!this.controlCheckBarometer() && MODE_INFO[this.mode]?.manualThrottle && !info.manualThrottle) { this.text(4, `Mode change failed: ${name} need alt estimate`); return false; }
    if (!this.modeInit(m)) { this.text(4, `Flight mode change failed ${name}`); return false; }
    if (this.mode === MODE.AUTO && this.mission.state === 'running') this.mission.state = 'stopped'; // exit_mode
    this.prevMode = this.mode;
    this.mode = m;
    this.modeReason = reason;
    this.text(6, `Mode ${MODE_LABEL[m]}`);
    return true;
  }
  // Sub::control_check_barometer (ArduSub.cpp:334-346)
  controlCheckBarometer() {
    if (!this.depthSensorPresent) { this.text(4, 'Depth sensor is not connected.'); return false; }
    if (this.failsafe.sensorHealth) { this.text(4, 'Depth sensor error.'); return false; }
    return true;
  }
  wpAccel() { const a = this.params.WPNAV_ACCEL ?? 250; return a > 0 ? a : 250; } // AC_WPNav::get_wp_acceleration
  althold_init() {
    const P = this.params;
    if (!this.controlCheckBarometer()) return false;
    this.pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    this.pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    this.pos.initZController();
    this.lastPilotHeading = this.sensors.yawSensor;
    return true;
  }
  modeInit(m) {
    const P = this.params, pos = this.pos;
    switch (m) {
      case MODE.MANUAL: case MODE.ACRO:
        pos.setPosTargetZCm(0);
        this.js.setNeutralControls();
        return true;
      case MODE.STABILIZE:
        pos.setPosTargetZCm(0);
        this.lastPilotHeading = this.sensors.yawSensor;
        return true;
      case MODE.ALT_HOLD:
        return this.althold_init();
      case MODE.SURFTRAK: {
        if (!this.althold_init()) return false;
        this.surftrakReset();
        if (!this.rangefinderAltOk()) this.text(6, 'waiting for a rangefinder reading');
        else if (this.sensors.positionZUpCm >= (P.SURFTRAK_DEPTH ?? -50)) this.text(4, `descend below ${((P.SURFTRAK_DEPTH ?? -50) * 0.01).toFixed(6)} meters to hold range`);
        return true;
      }
      case MODE.SURFACE:
        if (!this.controlCheckBarometer()) return false;
        pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        pos.initZController();
        return true;
      case MODE.POSHOLD:
        if (!this.positionOk()) return false;
        pos.setMaxSpeedAccelXY(P.WPNAV_SPEED ?? 500, this.wpAccel());
        pos.setCorrectionSpeedAccelXY(P.WPNAV_SPEED ?? 500, this.wpAccel());
        pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        pos.initXYControllerStoppingPoint();
        pos.initZController();
        this.att.setThrottleOut(0.5, true, P.PILOT_THR_FILT ?? 0);
        this.att.relaxAttitudeControllers();
        pos.relaxZController(0.5);
        this.lastPilotHeading = this.sensors.yawSensor;
        return true;
      case MODE.GUIDED:
        if (!this.positionOk()) return false;
        this.guidedPosControlStart();
        return true;
      case MODE.AUTO:
        if (!this.positionOk() || this.mission.items.length + 1 < 2) return false; // mission.num_commands() includes home
        this.autoMode = 'loiter';
        if (this.autoYawMode === AUTO_YAW.ROI) this.setAutoYawMode(AUTO_YAW.HOLD);
        this.wpNav.init();
        this.missionStartOrResume();
        return true;
      case MODE.CIRCLE:
        if (!this.positionOk()) return false;
        this.circlePilotYawOverride = false;
        pos.setMaxSpeedAccelXY(this.wpNav.speedXY(), this.wpNav.accelXY());
        pos.setCorrectionSpeedAccelXY(this.wpNav.speedXY(), this.wpNav.accelXY());
        pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
        this.circleNav.init();
        return true;
      case MODE.MOTOR_DETECT:
        this.md = { state: 0, motor: 0, dir: 1, settleMs: 0, thrustMs: 0 };
        return true;
    }
    return false;
  }
  pilotSpeedDn() { const P = this.params; return (P.PILOT_SPEED_DN ?? 0) === 0 ? Math.abs(P.PILOT_SPEED_UP ?? 100) : Math.abs(P.PILOT_SPEED_DN); }

  // ---------------- MAVLink / GCS ----------------
  // UI GCS-link toggle: while down the simulated GCS sends nothing (heartbeat and MANUAL_CONTROL lost)
  setGcsLink(ok) { this.gcsLinkDown = !ok; }
  get missionIndex() { return this.mission.index; }
  // queue a message as the GCS link delivers it; handled in GCS::update_receive
  receive(msg) { this.inbox.push(msg); }
  gcsUpdateReceive() {
    const q = this.inbox;
    this.inbox = [];
    for (const m of q) this.handleMessage(m);
  }
  handleMessage(msg) {
    const P = this.params, mygcs = P.SYSID_MYGCS ?? 255;
    switch (msg.type) {
      case 'HEARTBEAT': if ((msg.sysid ?? 255) === mygcs) this.gcsLastSeenMs = Math.max(1, this.ms); return; // GCS_Common.cpp:3931
      case 'MANUAL_CONTROL': default: this.handleManualControl(msg); return;
    }
  }
  // MANUAL_CONTROL (GCS_Mavlink.cpp:583-616)
  handleManualControl(msg) {
    const P = this.params;
    let ok;
    if (this.js.handleMessage) ok = this.js.handleMessage(msg);
    else {
      ok = (msg.sysid ?? 255) === (P.SYSID_MYGCS ?? 255) && (msg.target ?? P.SYSID_THISMAV ?? 1) === (P.SYSID_THISMAV ?? 1);
      if (ok) this.js.transform(msg.x | 0, msg.y | 0, msg.z | 0, msg.r | 0, msg.buttons >>> 0, msg.s | 0, msg.t | 0);
    }
    if (!ok) return false;
    this.failsafe.lastPilotInput = this.time;
    this.gcsLastSeenMs = Math.max(1, this.ms); // a RC override message counts as a GCS heartbeat
    return true;
  }
  // SET_ATTITUDE_TARGET (GCS_Mavlink.cpp:633-662). thrust (0..1) undefined = type_mask bit 6 (throttle ignored)
  setAttitudeTarget(rollDeg, pitchDeg, yawDeg, thrust) {
    this.attitudeTargetMsg = { t: this.time, roll: rollDeg, pitch: pitchDeg, yaw: yawDeg };
    if (thrust === undefined || thrust === null) return;
    const t = clamp(thrust, 0, 1);
    const climb = t === 0.5 ? 0 : t > 0.5 ? (t - 0.5) * 2 * this.wpNav.speedUp() : (t - 0.5) * 2 * this.wpNav.speedDn();
    if (this.mode !== MODE.GUIDED) return;
    if (this.guidedMode !== 'angle') this.guidedAngleControlStart();
    this.guidedAngle = { ms: this.ms, roll: rollDeg * 100, pitch: pitchDeg * 100, yaw: yawDeg * 100, climb };
  }
  clearAttitudeTarget() { this.attitudeTargetMsg = null; }
  // SET_POSITION_TARGET_GLOBAL_INT altitude while in ALT_HOLD (GCS_Mavlink.cpp:769-771): one-shot depth target, cm up
  setDepthTargetCm(cmUp) {
    if (this.mode !== MODE.ALT_HOLD) return false;
    this.pos.setPosTargetZCm(cmUp);
    return true;
  }
  // Simulator hook used by AUTO-TUNE: the value is applied once as the AC_PosControl Z target
  // (as SET_POSITION_TARGET_GLOBAL_INT does) at the next depth-control update of any depth-holding mode.
  get depthTargetOverride() { return this._depthTarget; }
  set depthTargetOverride(v) { this._depthTarget = v; this._depthTargetPending = v; }
  // SET_POSITION_TARGET_LOCAL_NED in GUIDED: pos [north m, east m, depth m]
  setGuidedTarget(pos, { yawDeg, yawRateDegs } = {}) {
    if (this.mode !== MODE.GUIDED || !Array.isArray(pos) || !pos.slice(0, 3).every(Number.isFinite)) return false;
    if (this.guidedMode !== 'wp') this.guidedPosControlStart();
    this.wpNav.setDestination([pos[0] * 100, pos[1] * 100, -pos[2] * 100], false);
    this.guidedSetYawState(yawDeg !== undefined, (yawDeg ?? 0) * 100, yawRateDegs !== undefined, (yawRateDegs ?? 0) * 100);
    return true;
  }
  setGuidedVelocity(vel, { yawDeg, yawRateDegs } = {}) {
    if (this.mode !== MODE.GUIDED || !Array.isArray(vel)) return false;
    if (this.guidedMode !== 'vel') this.guidedVelControlStart();
    this.guidedUpdateMs = this.ms;
    this.pos.velDesired[0] = vel[0] * 100; this.pos.velDesired[1] = vel[1] * 100; this.pos.velDesired[2] = -vel[2] * 100;
    this.guidedSetYawState(yawDeg !== undefined, (yawDeg ?? 0) * 100, yawRateDegs !== undefined, (yawRateDegs ?? 0) * 100);
    return true;
  }
  // AUTO mission: [{pos:[n,e,d]} | {n,e,d}, hold (s), terrain (d = height above the bottom)]
  setMission(items) {
    if (!Array.isArray(items)) return false;
    const out = [];
    for (const w of items) {
      const p = Array.isArray(w.pos) ? w.pos : [w.n, w.e, w.d ?? w.depth];
      if (!p.slice(0, 3).every(Number.isFinite)) { this.text(4, 'Mission rejected: bad waypoint'); return false; }
      out.push({ n: p[0], e: p[1], d: p[2], hold: Math.max(0, +w.hold || 0), terrain: !!(w.terrain || w.altAboveBottom) });
    }
    this.mission = { items: out, index: -1, state: 'stopped', loiterStartMs: 0, started: false };
    this.text(6, `Mission: ${out.length} waypoints`);
    return true;
  }
  // MAV_CMD_DO_MOTOR_TEST (motors.cpp:84-130): motor 0-based, pwm µs
  doMotorTest(motor, pwm, throttleType = 1) {
    if (!this.motorTestProto) return false;
    return this.motorTestProto.handle(this.ms, { param1: motor, param2: throttleType, param3: pwm });
  }

  // ---------------- pilot input helpers (Attitude.cpp, mode.cpp) ----------------
  getPilotDesiredLeanAngles(rollIn, pitchIn, angleMax) {
    const amax = this.params.ANGLE_MAX;
    angleMax = clamp(angleMax, 1000, amax);
    const scaler = amax / ROLL_PITCH_INPUT_MAX;
    rollIn *= scaler; pitchIn *= scaler;
    const total = Math.hypot(pitchIn, rollIn);
    if (total > angleMax) { const r = angleMax / total; rollIn *= r; pitchIn *= r; }
    rollIn = (18000 / Math.PI) * Math.atan(Math.cos((pitchIn * Math.PI) / 18000) * Math.tan((rollIn * Math.PI) / 18000));
    return [rollIn, pitchIn];
  }
  getPilotDesiredYawRate(stick) { return stick * (this.params.ACRO_YAW_P ?? ACRO_YAW_P_DEFAULT); }
  getPilotDesiredClimbRate(throttle) {
    if (this.failsafe.pilotInput) return 0;
    const mid = this.rc.throttle.getControlMid();
    const dz = clamp(this.params.THR_DZ ?? 100, 0, 400);
    const top = mid + dz * this.gain, bottom = mid - dz * this.gain;
    throttle = clamp(throttle, 0, 1000);
    let rate = 0;
    if (throttle < bottom) rate = (this.pilotSpeedDn() * (throttle - bottom)) / bottom;
    else if (throttle > top) rate = ((this.params.PILOT_SPEED_UP ?? 100) * (throttle - top)) / (1000 - top);
    this.desiredClimbRate = rate;
    return rate;
  }
  getPilotDesiredAngleRates(rollIn, pitchIn, yawIn) {
    const P = this.params;
    const total = Math.hypot(pitchIn, rollIn);
    if (total > ROLL_PITCH_INPUT_MAX) { const r = ROLL_PITCH_INPUT_MAX / total; rollIn *= r; pitchIn *= r; }
    const req = [0, 0, 0];
    if (P.ACRO_EXPO <= 0) { req[0] = rollIn * P.ACRO_RP_P; req[1] = pitchIn * P.ACRO_RP_P; }
    else {
      const expo = Math.min(P.ACRO_EXPO, 1);
      const f = (v) => { const x = v / ROLL_PITCH_INPUT_MAX; return ROLL_PITCH_INPUT_MAX * (expo * x * x * x + (1 - expo) * x) * P.ACRO_RP_P; };
      req[0] = f(rollIn); req[1] = f(pitchIn);
    }
    req[2] = yawIn * P.ACRO_YAW_P;
    if (P.ACRO_TRAINER !== 0) {
      const s = this.sensors;
      const rollA = wrap180cd(s.rollSensor), pitchA = wrap180cd(s.pitchSensor);
      const lvl = [-clamp(rollA, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * P.ACRO_BAL_ROLL, -clamp(pitchA, -ACRO_LEVEL_MAX_ANGLE, ACRO_LEVEL_MAX_ANGLE) * P.ACRO_BAL_PITCH, 0];
      const amax = P.ANGLE_MAX;
      if (P.ACRO_TRAINER === 2) {
        if (rollA > amax) lvl[0] -= P.ACRO_BAL_ROLL * (rollA - amax); else if (rollA < -amax) lvl[0] -= P.ACRO_BAL_ROLL * (rollA + amax);
        if (pitchA > amax) lvl[1] -= P.ACRO_BAL_PITCH * (pitchA - amax); else if (pitchA < -amax) lvl[1] -= P.ACRO_BAL_PITCH * (pitchA + amax);
      }
      const e = this.att.getAttTargetEulerCd().map((v) => (v * Math.PI) / 18000);
      let bf = eulerRateToAngVel(e, lvl);
      if (P.ACRO_TRAINER === 2) { req[0] += bf[0]; req[1] += bf[1]; req[2] += bf[2]; }
      else {
        const mix = clamp(1 - Math.max(Math.abs(rollIn), Math.abs(pitchIn), Math.abs(yawIn)) / 4500, 0, 1) * s.cosPitch;
        bf = bf.map((v) => v * mix);
        for (let k = 0; k < 3; k++) { const lim = Math.abs(Math.abs(req[k]) - Math.abs(bf[k])); req[k] = clamp(req[k] + bf[k], -lim, lim); }
      }
    }
    return req;
  }

  // shared heading-hold logic of stabilize / depth hold / position hold
  runAttitude(targetRoll, targetPitch, targetYawRate) {
    const nowMs = this.ms;
    if (targetYawRate !== 0) {
      this.att.inputEulerAngleRollPitchEulerRateYaw(targetRoll, targetPitch, targetYawRate);
      this.lastPilotHeading = this.sensors.yawSensor;
      this.lastPilotYawInputMs = nowMs;
    } else if (nowMs < this.lastPilotYawInputMs + 250) {
      this.att.inputEulerAngleRollPitchEulerRateYaw(targetRoll, targetPitch, 0);
      this.lastPilotHeading = this.sensors.yawSensor;
    } else {
      this.att.inputEulerAngleRollPitchYaw(targetRoll, targetPitch, this.lastPilotHeading, true);
    }
  }
  // translate_*_rp (motors.cpp:137-184): roll/pitch targets -> lateral/forward, normalised by ANGLE_MAX
  translateRp() {
    const amax = this.params.ANGLE_MAX;
    const lat = clamp(Math.trunc(this.pos.rollTarget), -amax, amax), fwd = clamp(Math.trunc(-this.pos.pitchTarget), -amax, amax);
    return [lat / amax, fwd / amax];
  }

  // ---------------- update_flight_mode ----------------
  runMode() {
    const P = this.params, m = this.motors, rc = this.rc, att = this.att, pos = this.pos;
    const thrFilt = P.PILOT_THR_FILT ?? 0;
    const disarmedCommon = (thr) => {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(thr, true, thrFilt);
      att.relaxAttitudeControllers();
    };
    const yawInputDz = () => rc.yaw.pwmToAngleDzTrim(Math.trunc(rc.yaw.deadZone * this.gain), rc.yaw.trim);
    switch (this.mode) {
      case MODE.MANUAL: {
        if (!this.armed) { disarmedCommon(0); return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        m.setRoll(rc.roll.normInput());
        m.setPitch(rc.pitch.normInput());
        m.setYaw((rc.yaw.normInput() * P.ACRO_YAW_P) / ACRO_YAW_P_DEFAULT);
        m.setThrottle(rc.throttle.normInput());
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.STABILIZE: {
        if (!this.armed) { disarmedCommon(0); this.lastPilotHeading = this.sensors.yawSensor; return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        this.runAttitude(r, p, this.getPilotDesiredYawRate(yawInputDz()));
        att.setThrottleOut(rc.throttle.normInput(), false, thrFilt);
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.ACRO: {
        if (!this.armed) { disarmedCommon(0); return; }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const t = this.getPilotDesiredAngleRates(rc.roll.controlIn, rc.pitch.controlIn, rc.yaw.controlIn);
        att.inputRateBfRollPitchYaw(t[0], t[1], t[2]);
        att.setThrottleOut(rc.throttle.normInput(), false, thrFilt);
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.ALT_HOLD: case MODE.SURFTRAK: {
        // ModeAlthold::run: run_pre(); control_depth() / control_range(); run_post()
        this.altholdRunPre();
        if (this.mode === MODE.SURFTRAK) this.controlRange(); else this.controlDepth();
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.SURFACE: {
        if (!this.armed) {
          m.outputMin?.();
          disarmedCommon(0);
          pos.initZController();
          return;
        }
        // already at the surface: hold depth there (the rest of this loop still runs)
        if (this.atSurface) this.setMode(MODE.ALT_HOLD, 'surface complete');
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        att.inputEulerAngleRollPitchEulerRateYaw(r, p, this.getPilotDesiredYawRate(rc.yaw.controlIn));
        const cmb = clamp(Math.abs(this.wpNav.speedUp()), 1, pos.getMaxSpeedUpCms());
        this.desiredClimbRate = cmb;
        pos.setPosTargetZFromClimbRateCm(cmb);
        pos.updateZController();
        m.setForward(rc.forward.normInput());
        m.setLateral(rc.lateral.normInput());
        return;
      }
      case MODE.POSHOLD: {
        if (!this.armed) {
          disarmedCommon(0.5);
          pos.initXYControllerStoppingPoint();
          pos.relaxZController(0.5);
          this.lastPilotHeading = this.sensors.yawSensor;
          return;
        }
        m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
        const pl = rc.lateral.normInput(), pf = rc.forward.normInput();
        let latOut = 0, fwdOut = 0;
        if (this.positionOk()) {
          if (Math.abs(pl) > 0.1 || Math.abs(pf) > 0.1) pos.initXYControllerStoppingPoint();
          [latOut, fwdOut] = this.translateRp();
          pos.updateXYController();
        } else pos.initXYControllerStoppingPoint();
        m.setForward(fwdOut + pf);
        m.setLateral(latOut + pl);
        const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
        this.runAttitude(r, p, this.getPilotDesiredYawRate(yawInputDz()));
        this.controlDepth();
        return;
      }
      case MODE.GUIDED: this.guidedRun(); return;
      case MODE.AUTO: this.autoRun(); return;
      case MODE.CIRCLE: this.circleRun(); return;
      case MODE.MOTOR_DETECT: this.motorDetectRun(); return;
    }
  }

  // ModeAlthold::run_pre (mode_althold.cpp:36-96)
  altholdRunPre() {
    const P = this.params, m = this.motors, att = this.att, pos = this.pos, rc = this.rc;
    pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    if (!this.armed) {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(0.5, true, P.PILOT_THR_FILT ?? 0);
      att.relaxAttitudeControllers();
      pos.relaxZController(m.throttleHover);
      this.lastPilotHeading = this.sensors.yawSensor;
      return;
    }
    m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
    const tm = this.attitudeTargetMsg;
    if (tm && this.time - tm.t < 5) { att.inputEulerAngleRollPitchYaw(tm.roll * 100, tm.pitch * 100, tm.yaw * 100, true); return; }
    const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, att.getAltholdLeanAngleMaxCd());
    const yawIn = rc.yaw.pwmToAngleDzTrim(Math.trunc(rc.yaw.deadZone * this.gain), rc.yaw.trim);
    this.runAttitude(r, p, this.getPilotDesiredYawRate(yawIn));
  }
  applyPendingDepthTarget() {
    if (this._depthTargetPending === null || this._depthTargetPending === undefined) return;
    this.pos.setPosTargetZCm(this._depthTargetPending);
    this._depthTargetPending = null;
  }
  // ModeAlthold::control_depth (mode_althold.cpp:104-121)
  controlDepth() {
    const pos = this.pos;
    this.applyPendingDepthTarget();
    let rate = this.getPilotDesiredClimbRate(this.rc.throttle.controlIn);
    rate = clamp(rate, -this.pilotSpeedDn(), this.params.PILOT_SPEED_UP ?? 100);
    if (Math.abs(rate) < 0.05) {
      if (this.atSurface) pos.setPosTargetZCm(Math.min(pos.getPosTargetZCm(), (this.params.SURFACE_DEPTH ?? -10) - 5));
      else if (this.atBottom) pos.setPosTargetZCm(Math.max(this.sensors.positionZUpCm + 10, pos.getPosTargetZCm()));
    }
    pos.setPosTargetZFromClimbRateCm(rate);
    pos.updateZController();
  }

  // ---------------- SURFTRAK (mode_surftrak.cpp) ----------------
  setPosOffsetZ(v) { if (this.pos.setPosOffsetZCm) this.pos.setPosOffsetZCm(v); else this.pos.posOffsetZ = v; }
  setPosOffsetTargetZ(v) { if (this.pos.setPosOffsetTargetZCm) this.pos.setPosOffsetTargetZCm(v); else this.pos.posOffsetTargetZ = v; }
  surftrakReset() { this.surftrak.targetCm = SURFTRAK_INVALID_TARGET; this.setPosOffsetZ(0); this.setPosOffsetTargetZ(0); }
  setRangefinderTargetCm(targetCm) {
    const P = this.params, rs = this.rangefinderState, sd = P.SURFTRAK_DEPTH ?? -50;
    let ok = false;
    if (this.mode !== MODE.SURFTRAK) this.text(4, 'wrong mode, rangefinder target not set');
    else if (this.sensors.positionZUpCm >= sd) this.text(4, `descend below ${(sd * 0.01).toFixed(6)} meters to set rangefinder target`);
    else if (targetCm < rs.minCm) this.text(4, 'rangefinder target below minimum, ignored');
    else if (targetCm > rs.maxCm) this.text(4, 'rangefinder target above maximum, ignored');
    else ok = true;
    if (ok) {
      this.surftrak.targetCm = targetCm;
      this.text(6, `rangefinder target is ${(targetCm * 0.01).toFixed(2)} meters`);
      const off = this.sensors.positionZUpCm - targetCm;
      this.setPosOffsetZ(off); this.setPosOffsetTargetZ(off);
    } else this.surftrakReset();
    return ok;
  }
  controlRange() {
    const P = this.params, pos = this.pos, st = this.surftrak;
    this.applyPendingDepthTarget();
    let rate = this.getPilotDesiredClimbRate(this.rc.throttle.controlIn);
    rate = clamp(rate, -this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100);
    const valid = st.targetCm > 0;
    if (Math.abs(rate) < 0.05) {
      if (st.pilotInControl) { this.setRangefinderTargetCm(st.targetCm + this.sensors.positionZUpCm - st.pilotControlStartZCm); st.pilotInControl = false; }
      if (this.atSurface) { pos.setPosTargetZCm(Math.min(pos.getPosTargetZCm(), (P.SURFACE_DEPTH ?? -10) - 5)); this.surftrakReset(); }
      else if (this.atBottom) { pos.setPosTargetZCm(Math.max(this.sensors.positionZUpCm + 10, pos.getPosTargetZCm())); this.surftrakReset(); }
      else this.updateSurfaceOffset();
    } else if (valid && !st.pilotInControl) { st.pilotControlStartZCm = this.sensors.positionZUpCm; st.pilotInControl = true; }
    pos.setPosTargetZFromClimbRateCm(rate);
    pos.updateZController();
  }
  updateSurfaceOffset() {
    if (!this.rangefinderAltOk()) return;
    const rs = this.rangefinderState, sd = this.params.SURFTRAK_DEPTH ?? -50, st = this.surftrak;
    let off = rs.terrainOffsetCm;
    if (!(st.targetCm > 0) && rs.inertialAltCm < sd) this.setRangefinderTargetCm(rs.inertialAltCm - off);
    if (st.targetCm > 0) {
      const desired = off + st.targetCm;
      if (desired >= sd) off += sd - desired; // stay below SURFTRAK_DEPTH
      this.setPosOffsetTargetZ(off);
    }
  }

  // ---------------- GUIDED (mode_guided.cpp) ----------------
  defaultAutoYawMode() {
    switch (this.params.WP_YAW_BEHAVIOR ?? 4) {
      case 0: return AUTO_YAW.HOLD;
      case 2: return AUTO_YAW.LOOK_AT_NEXT_WP;
      case 3: return AUTO_YAW.LOOK_AHEAD;
      case 4: return AUTO_YAW.CORRECT_XTRACK;
      default: return AUTO_YAW.LOOK_AT_NEXT_WP;
    }
  }
  setAutoYawMode(y) {
    if (this.autoYawMode === y) return;
    this.autoYawMode = y;
    if (y === AUTO_YAW.LOOK_AHEAD) this.yawLookAheadBearing = this.sensors.yawSensor;
  }
  getAutoHeading() {
    switch (this.autoYawMode) {
      case AUTO_YAW.LOOK_AT_HEADING: return this.yawLookAtHeading;
      case AUTO_YAW.LOOK_AHEAD: {
        const v = this.sensors.velocityXYCms;
        if (Math.hypot(v[0], v[1]) > 100) this.yawLookAheadBearing = wrap360cd((Math.atan2(v[1], v[0]) * 18000) / Math.PI);
        return this.yawLookAheadBearing ?? this.sensors.yawSensor;
      }
      case AUTO_YAW.RESETTOARMEDYAW: return this.initialArmedBearing ?? 0;
      case AUTO_YAW.CORRECT_XTRACK: {
        const w = this.wpNav;
        if (!w.dest || !w.origin) return w.yawCd;
        const track = (Math.atan2(w.dest[1] - w.origin[1], w.dest[0] - w.origin[0]) * 18000) / Math.PI;
        const vt = this.pos.velTarget ?? [0, 0];
        let err = 0;
        if (Math.hypot(vt[0], vt[1]) >= (this.pos.velMaxXY ?? 500) * 0.1) err = wrap180cd((Math.atan2(vt[1], vt[0]) * 18000) / Math.PI - track);
        const lim = (this.params.XTRACK_ANG_LIM ?? 45) * 100;
        return wrap360cd(track + clamp(err, -lim, lim));
      }
      case AUTO_YAW.LOOK_AT_NEXT_WP: default: return this.wpNav.yawCd;
    }
  }
  guidedPosControlStart() {
    this.guidedMode = 'wp';
    this.wpNav.init();
    this.wpNav.setDestination(this.wpNav.stoppingPoint(), false);
    this.setAutoYawMode(this.defaultAutoYawMode());
  }
  guidedVelControlStart() {
    const P = this.params;
    this.guidedMode = 'vel';
    this.pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    this.pos.setCorrectionSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    this.pos.initZController();
    this.pos.initXYController();
    this.setAutoYawMode(AUTO_YAW.HOLD);
  }
  guidedAngleControlStart() {
    this.guidedMode = 'angle';
    this.pos.setMaxSpeedAccelZ(-this.wpNav.speedDn(), this.wpNav.speedUp(), this.wpNav.accelZ());
    this.pos.setCorrectionSpeedAccelZ(-this.wpNav.speedDn(), this.wpNav.speedUp(), this.wpNav.accelZ());
    this.pos.initZController();
    const e = this.att.getAttTargetEulerCd();
    this.guidedAngle = { ms: this.ms, roll: 0, pitch: 0, yaw: this.sensors.yawSensor, climb: 0 };
    void e;
  }
  guidedSetYawState(useYaw, yawCd, useYawRate, yawRateCds) {
    if (useYaw) {
      this.yawRateOnly = false;
      this.yawLookAtHeading = wrap360cd(yawCd);
      this.yawLookAtHeadingSlew = useYawRate && yawRateCds !== 0 ? Math.min(Math.abs(yawRateCds * 0.01), AUTO_YAW_SLEW_RATE) : AUTO_YAW_SLEW_RATE;
      this.setAutoYawMode(AUTO_YAW.LOOK_AT_HEADING);
    } else if (useYawRate) {
      this.yawRateOnly = true;
      this.yawLookAtHeadingSlew = Math.min(yawRateCds * 0.01, AUTO_YAW_SLEW_RATE);
      this.setAutoYawMode(AUTO_YAW.RATE);
    } else { this.yawRateOnly = false; this.setAutoYawMode(AUTO_YAW.HOLD); }
  }
  guidedPilotYaw() {
    let yr = 0;
    if (!this.failsafe.pilotInput) {
      yr = this.getPilotDesiredYawRate(this.rc.yaw.controlIn);
      if (yr !== 0) this.setAutoYawMode(AUTO_YAW.HOLD);
      else this.setAutoYawMode(this.yawRateOnly ? AUTO_YAW.RATE : AUTO_YAW.LOOK_AT_HEADING);
    }
    return yr;
  }
  guidedAttitude(rollCd, pitchCd, yawRate) {
    const att = this.att;
    if (this.autoYawMode === AUTO_YAW.HOLD) att.inputEulerAngleRollPitchEulerRateYaw(rollCd, pitchCd, yawRate);
    else if (this.autoYawMode === AUTO_YAW.LOOK_AT_HEADING) {
      const rate = this.yawLookAtHeadingSlew * 100;
      if (att.inputEulerAngleRollPitchSlewYaw) att.inputEulerAngleRollPitchSlewYaw(rollCd, pitchCd, this.getAutoHeading(), rate);
      else att.inputEulerAngleRollPitchYaw(rollCd, pitchCd, this.getAutoHeading(), true);
    } else if (this.autoYawMode === AUTO_YAW.RATE) att.inputEulerAngleRollPitchEulerRateYaw(rollCd, pitchCd, this.yawLookAtHeadingSlew * 100);
    else att.inputEulerAngleRollPitchYaw(rollCd, pitchCd, this.getAutoHeading(), true);
  }
  guidedRun() {
    const P = this.params, m = this.motors, att = this.att, pos = this.pos, rc = this.rc;
    if (!this.armed) {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(0, true, P.PILOT_THR_FILT ?? 0);
      att.relaxAttitudeControllers();
      if (this.guidedMode === 'wp') this.wpNav.init();
      else { pos.initZController(); if (this.guidedMode === 'vel') pos.initXYController(); }
      return;
    }
    if (this.guidedMode === 'angle') {
      const g = this.guidedAngle;
      let r = g.roll, p = g.pitch;
      const total = Math.hypot(r, p), amax = Math.min(att.getAltholdLeanAngleMaxCd(), P.ANGLE_MAX);
      if (total > amax) { r *= amax / total; p *= amax / total; }
      let climb = clamp(g.climb, -this.wpNav.speedDn(), this.wpNav.speedUp());
      if (this.ms - g.ms > GUIDED_ATTITUDE_TIMEOUT_MS) { r = 0; p = 0; climb = 0; }
      m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
      att.inputEulerAngleRollPitchYaw(r, p, wrap180cd(g.yaw), true);
      pos.setPosTargetZFromClimbRateCm(climb);
      pos.updateZController();
      return;
    }
    const yr = this.guidedPilotYaw();
    m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
    if (this.guidedMode === 'vel') {
      if (this.ms - this.guidedUpdateMs > GUIDED_POSVEL_TIMEOUT_MS && pos.velDesired.some((v) => v !== 0)) pos.velDesired.fill(0);
      if (pos.stopPosXYStabilisation) pos.stopPosXYStabilisation(); else { const p = this.sensors.positionXYCm; pos.posTarget[0] = p[0]; pos.posTarget[1] = p[1]; }
      pos.updateXYController();
      pos.setPosTargetZFromClimbRateCm(pos.velDesired[2]);
      pos.updateZController();
    } else {
      this.failsafeTerrainSetStatus(this.wpNav.update(pos.dt));
      pos.updateZController();
    }
    const [lat, fwd] = this.translateRp();
    m.setLateral(lat); m.setForward(fwd);
    this.guidedAttitude(rc.roll.controlIn, rc.pitch.controlIn, yr);
  }

  // ---------------- AUTO (mode_auto.cpp, AP_Mission) ----------------
  missionStartOrResume() {
    const ms = this.mission;
    if ((this.params.MIS_RESTART ?? 0) === 1 || ms.index < 0 || ms.state === 'complete') ms.index = 0;
    ms.state = 'running';
    ms.started = false;
  }
  missionUpdate() {
    const ms = this.mission;
    if (ms.state !== 'running') return;
    const w = ms.items[ms.index];
    if (!w) { this.missionComplete(); return; }
    if (!ms.started) {
      ms.started = true; ms.loiterStartMs = 0;
      this.text(6, `Mission: ${ms.index + 1} WP`);
      this.autoWpStart(w);
      return;
    }
    // verify_nav_wp (commands_logic.cpp:436-458)
    if (!this.wpNav.reached) return;
    if (ms.loiterStartMs === 0) ms.loiterStartMs = Math.max(1, this.ms);
    if (Math.trunc((this.ms - ms.loiterStartMs) / 1000) >= w.hold) {
      this.text(6, `Reached command #${ms.index + 1}`);
      ms.index++; ms.started = false;
      if (ms.index >= ms.items.length) this.missionComplete();
    }
  }
  missionComplete() {
    this.mission.state = 'complete';
    this.text(6, 'Mission complete');
    // exit_mission: loiter, else depth hold
    if (!this.autoLoiterStart()) this.setMode(MODE.ALT_HOLD, 'mission end');
  }
  autoWpStart(w) {
    this.autoMode = 'wp';
    this.wpNav.setDestination([w.n * 100, w.e * 100, w.terrain ? w.d * 100 : -w.d * 100], w.terrain);
    if (w.terrain && !this.rangefinderAltOk()) { this.text(4, 'Terrain data (rangefinder) not available'); this.failsafeTerrainOnEvent(); return; }
    if (this.autoYawMode !== AUTO_YAW.ROI) this.setAutoYawMode(this.defaultAutoYawMode());
  }
  autoLoiterStart() {
    if (!this.positionOk()) return false;
    this.autoMode = 'loiter';
    this.wpNav.setDestination(this.wpNav.stoppingPoint(), false);
    this.setAutoYawMode(AUTO_YAW.HOLD);
    return true;
  }
  autoRun() {
    this.missionUpdate();
    if (this.mode !== MODE.AUTO) { this.runMode(); return; }
    const P = this.params, m = this.motors, att = this.att, pos = this.pos, rc = this.rc;
    if (!this.armed) {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(0, true, P.PILOT_THR_FILT ?? 0);
      att.relaxAttitudeControllers();
      if (this.autoMode === 'terrain') pos.relaxZController(m.throttleHover);
      else this.wpNav.init();
      return;
    }
    if (this.autoMode === 'terrain') { this.autoTerrainRecoverRun(); return; }
    let yr = 0;
    if (!this.failsafe.pilotInput) {
      yr = this.getPilotDesiredYawRate(rc.yaw.controlIn);
      if (this.autoMode === 'wp' && yr !== 0) this.setAutoYawMode(AUTO_YAW.HOLD);
    }
    m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
    this.failsafeTerrainSetStatus(this.wpNav.update(pos.dt));
    const [lat, fwd] = this.translateRp();
    m.setLateral(lat); m.setForward(fwd);
    pos.updateZController();
    const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, P.ANGLE_MAX);
    if (this.autoMode === 'loiter' || this.autoYawMode === AUTO_YAW.HOLD) att.inputEulerAngleRollPitchEulerRateYaw(r, p, yr);
    else att.inputEulerAngleRollPitchYaw(r, p, this.getAutoHeading(), true);
  }
  // ModeAuto::auto_terrain_recover_start/run (mode_auto.cpp:427-570)
  autoTerrainRecoverStart() {
    if (this.mode !== MODE.AUTO) return false;
    const st = this.sensors.rangefinder?.status;
    if (st !== RF_STATUS.OutOfRangeLow && st !== RF_STATUS.OutOfRangeHigh && st !== RF_STATUS.Good) return false;
    this.autoMode = 'terrain';
    this.fsTerrainRecoverStartMs = this.ms;
    this.mission.state = 'stopped';
    this.pos.initXYControllerStoppingPoint();
    this.pos.relaxZController(this.motors.throttleHover);
    this.pos.setMaxSpeedAccelZ(-this.wpNav.speedDn(), this.wpNav.speedUp(), this.wpNav.accelZ());
    this.pos.setCorrectionSpeedAccelZ(-this.wpNav.speedDn(), this.wpNav.speedUp(), this.wpNav.accelZ());
    this.text(4, 'Attempting auto failsafe recovery');
    return true;
  }
  autoTerrainRecoverRun() {
    const m = this.motors, pos = this.pos, rc = this.rc, st = this.sensors.rangefinder?.status;
    let climb = 0;
    if (st === RF_STATUS.OutOfRangeLow) { climb = this.wpNav.speedUp(); this.rfRecoveryMs = 0; }
    else if (st === RF_STATUS.OutOfRangeHigh) { climb = -this.wpNav.speedDn(); this.rfRecoveryMs = 0; }
    else if (st === RF_STATUS.Good) {
      if (this.rangefinderState.altHealthy) {
        if (this.rfRecoveryMs === 0) { this.rfRecoveryMs = Math.max(1, this.ms); pos.relaxZController(m.throttleHover); }
        if (this.ms > this.rfRecoveryMs + 1500) {
          this.text(6, 'Terrain failsafe recovery successful!');
          this.failsafeTerrainSetStatus(true);
          this.failsafe.terrain = false;
          this.autoMode = 'loiter';
          this.mission.state = 'running'; this.mission.started = false; // mission.resume()
          this.rfRecoveryMs = 0;
        }
      }
    } else {
      this.text(2, 'Terrain failsafe recovery failure: No Rangefinder!');
      this.failsafeTerrainAct();
      this.rfRecoveryMs = 0;
      return;
    }
    if (this.ms > this.fsTerrainRecoverStartMs + FS_TERRAIN_RECOVER_TIMEOUT_MS) { this.text(2, 'Terrain failsafe recovery timeout!'); this.failsafeTerrainAct(); }
    if (this.mode !== MODE.AUTO) return;
    m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
    pos.updateXYController();
    const [lat, fwd] = this.translateRp();
    m.setLateral(lat); m.setForward(fwd);
    pos.setPosTargetZFromClimbRateCm(climb);
    pos.updateZController();
    const [r, p] = this.getPilotDesiredLeanAngles(rc.roll.controlIn, rc.pitch.controlIn, this.params.ANGLE_MAX);
    this.att.inputEulerAngleRollPitchEulerRateYaw(r, p, 0);
  }

  // ---------------- CIRCLE (mode_circle.cpp) ----------------
  circleRun() {
    const P = this.params, m = this.motors, att = this.att, pos = this.pos, rc = this.rc;
    pos.setMaxSpeedAccelXY(this.wpNav.speedXY(), this.wpNav.accelXY());
    pos.setMaxSpeedAccelZ(-this.pilotSpeedDn(), P.PILOT_SPEED_UP ?? 100, P.PILOT_ACCEL_Z ?? 100);
    if (!this.armed) {
      m.setDesiredSpoolState(SPOOL.GROUND_IDLE);
      att.setThrottleOut(0, true, P.PILOT_THR_FILT ?? 0);
      att.relaxAttitudeControllers();
      this.circleNav.init();
      return;
    }
    const yr = this.getPilotDesiredYawRate(rc.yaw.controlIn);
    if (yr !== 0) this.circlePilotYawOverride = true;
    const climb = this.getPilotDesiredClimbRate(rc.throttle.controlIn);
    m.setDesiredSpoolState(SPOOL.THROTTLE_UNLIMITED);
    this.failsafeTerrainSetStatus(this.circleNav.update(pos.dt, 0));
    const [lat, fwd] = this.translateRp();
    m.setLateral(lat); m.setForward(fwd);
    if (this.circlePilotYawOverride) att.inputEulerAngleRollPitchEulerRateYaw(rc.roll.controlIn, rc.pitch.controlIn, yr);
    else att.inputEulerAngleRollPitchYaw(rc.roll.controlIn, rc.pitch.controlIn, this.circleNav.yawCd, true);
    pos.setPosTargetZFromClimbRateCm(climb);
    pos.updateZController();
  }

  // ---------------- MOTOR_DETECT (mode_motordetect.cpp) ----------------
  motorDetectRun() {
    const m = this.motors, d = this.md, now = this.ms;
    const n = this.mixer.factors.length;
    const allNeutral = () => { for (let i = 0; i < n; i++) m.outputTestNum ? m.outputTestNum(i, 1500) : null; };
    if (!this.armed) { m.setDesiredSpoolState(SPOOL.GROUND_IDLE); allNeutral(); d.state = 0; return; }
    const g = this.sensors.gyroLatest;
    switch (d.state) {
      case 0: d.dir = 1; d.motor = 0; d.settleMs = now; d.state = 1; break; // STANDBY
      case 1: // SETTLING
        allNeutral();
        if (g[0] * g[0] + g[1] * g[1] + g[2] * g[2] > 0.01) d.settleMs = now;
        if (now > d.settleMs + 500) { d.state = 2; d.thrustMs = now; }
        break;
      case 2: // THRUSTING
        if (now < d.thrustMs + 500) { if (!(m.outputTestNum?.(d.motor, 1500 + 300 * d.dir) ?? false)) d.state = 4; }
        else d.state = 3;
        break;
      case 3: { // DETECTING
        const s3 = (v, th) => (v > th ? 1 : 0) - (v < -th ? 1 : 0);
        const dirs = [s3(g[0], 0.4) * d.dir, s3(g[1], 0.4) * d.dir, s3(g[2], 0.5) * d.dir];
        const f = m.getMotorAngularFactors?.(d.motor) ?? (this.mixer.factors[d.motor] || [0, 0, 0]).slice(0, 3);
        if (dirs.every((v, k) => v === f[k])) this.text(6, `Thruster ${d.motor + 1} is ok!`);
        else if (dirs.every((v, k) => -v === f[k])) {
          this.text(6, `Thruster ${d.motor + 1} is reversed! Saving it!`);
          const r = m.setReversed?.(d.motor, true);
          this.saveParam(r?.name ?? `MOT_${d.motor + 1}_DIRECTION`, r?.value ?? -1);
          if (!m.setReversed) m.reverse[d.motor] = -1;
        } else {
          this.text(6, 'Bad thrust read, trying to push the other way...');
          if (d.dir === -1) { this.text(4, `Failed! Please check Thruster ${d.motor + 1} and frame setup!`); d.state = 4; break; }
          d.dir = -1; d.state = 1; break;
        }
        d.state = 1; d.motor++; d.dir = 1;
        if (!(m.motorIsEnabled?.(d.motor) ?? d.motor < n)) { d.state = 4; this.text(4, 'Motor direction detection is complete.'); }
        break;
      }
      case 4: // DONE
        this.setMode(this.prevMode, 'mission end');
        this.disarm('motordetect done');
        break;
    }
  }

  // ---------------- surface / bottom detector (surface_bottom_detector.cpp) ----------------
  setSurfaced(v) { if (this.atSurface === v) return; this.atSurface = v; this.surfaceCount = 0; }
  setBottomed(v) { if (this.atBottom === v) return; this.atBottom = v; this.bottomCount = 0; }
  updateSurfaceBottom() {
    if (!this.armed) { this.setSurfaced(false); this.setBottomed(false); return; }
    const vz = -this.sensors.velocityZUpCms / 100; // NED m/s
    const stationary = vz > -0.05 && vz < 0.05;
    const lim = this.motors.limit, trig = BOTTOM_DETECTOR_TRIGGER_SEC * MAIN_LOOP_RATE;
    if (this.depthSensorPresent && this.sensorHealthDepth) {
      const depthAlt = -this.sensors.depthMeasured; // barometer.get_altitude(), m up
      const sd = (this.params.SURFACE_DEPTH ?? -10) / 100;
      this.setSurfaced(this.atSurface ? depthAlt > sd - 0.05 : depthAlt > sd);
      if (lim.throttle_lower && stationary) {
        if (this.bottomCount < trig) this.bottomCount++;
        else this.setBottomed(true);
      } else this.setBottomed(false);
    } else if (stationary) {
      if (lim.throttle_upper) {
        if (this.surfaceCount < SURFACE_DETECTOR_TRIGGER_SEC * MAIN_LOOP_RATE) this.surfaceCount++;
        else this.setSurfaced(true);
      } else if (lim.throttle_lower) {
        if (this.bottomCount < trig) this.bottomCount++;
        else this.setBottomed(true);
      } else { this.setSurfaced(false); this.setBottomed(false); }
    } else { this.setSurfaced(false); this.setBottomed(false); }
  }

  // ---------------- failsafes (failsafe.cpp) ----------------
  failsafePilotInputCheck() {
    const P = this.params, f = this.failsafe;
    if ((P.FS_PILOT_INPUT ?? 2) === 0) { f.pilotInput = false; return; }
    if (this.ms < Math.floor(f.lastPilotInput * 1000 + 1e-6) + (P.FS_PILOT_TIMEOUT ?? 3) * 1000) { f.pilotInput = false; return; }
    if (f.pilotInput) return;
    f.pilotInput = true;
    this.text(2, 'Lost manual control');
    this.js.setNeutralControls();
    if ((P.FS_PILOT_INPUT ?? 2) === 2) this.disarm('pilot input failsafe');
  }
  failsafeCrashCheck() {
    const P = this.params, f = this.failsafe, now = this.ms;
    if (!this.armed || (P.FS_CRASH_CHECK ?? 0) === 0 || this.mode === MODE.ACRO || this.mode === MODE.MANUAL) { f.lastCrashPassMs = now; f.crash = false; return; }
    const err = this.att.getAttErrorAngleDeg?.() ?? this.att.thrustErrorAngle * RAD;
    if (err <= 30) { f.lastCrashPassMs = now; f.crash = false; return; }
    if (now < f.lastCrashPassMs + 2000) return;
    if (now > f.lastCrashWarnMs + 20000) { f.lastCrashWarnMs = now; this.text(4, 'Crash detected'); }
    if (f.crash) return;
    f.crash = true;
    if ((P.FS_CRASH_CHECK ?? 0) === 2) this.disarm('crash');
  }
  failsafeEkfCheck() {
    const P = this.params, f = this.failsafe, now = this.ms;
    if ((P.FS_EKF_ACTION ?? 0) === 0) { f.lastEkfGoodMs = now; f.ekf = false; return; }
    const v = this.sensors.ekfVariances ?? { vel: 0, mag: 0 }, th = P.FS_EKF_THRESH ?? 0.8;
    if (v.mag < th && v.vel < th) { f.lastEkfGoodMs = now; f.ekf = false; return; }
    if (now < f.lastEkfGoodMs + 2000) { f.ekf = false; return; }
    if (f.ekf) return;
    f.ekf = true;
    if (now > f.lastEkfWarnMs + 20000) { f.lastEkfWarnMs = now; this.text(4, 'EKF bad'); }
    if ((P.FS_EKF_ACTION ?? 0) === 2) this.disarm('ekf failsafe');
  }
  failsafeSensorsCheck() {
    const f = this.failsafe;
    if (!this.depthSensorPresent) return;
    if (this.sensorHealthDepth) { f.sensorHealth = false; return; }
    if (f.sensorHealth) return;
    f.sensorHealth = true;
    this.text(2, 'Depth sensor error!');
    if (this.mode === MODE.ALT_HOLD || this.mode === MODE.SURFACE || MODE_INFO[this.mode]?.gps) {
      if (!this.setMode(MODE.MANUAL, 'bad depth')) this.disarm('bad flow of control');
    }
  }
  failsafeLeakCheck() {
    const P = this.params, f = this.failsafe;
    const status = !!this.sensors.leak;
    if (!status || !(P.FS_LEAK_ENABLE ?? 1)) { f.leak = false; return; }
    if (this.ms > f.lastLeakWarnMs + 20000) { f.lastLeakWarnMs = this.ms; this.text(2, 'Leak Detected'); }
    if (f.leak) return;
    f.leak = true;
    if (P.FS_LEAK_ENABLE === 2 && this.armed) this.setMode(MODE.SURFACE, 'leak failsafe');
  }
  failsafeInternalPressureCheck() {
    const P = this.params, f = this.failsafe, now = this.ms;
    if (!(P.FS_PRESS_ENABLE ?? 0)) return;
    if (this.sensors.internalPressure < (P.FS_PRESS_MAX ?? 105000)) { f.lastPressGoodMs = now; f.lastPressWarnMs = now; f.internalPressure = false; return; }
    if (now > f.lastPressGoodMs + 2000) f.internalPressure = true;
    if (f.internalPressure && now > f.lastPressWarnMs + 30000) { f.lastPressWarnMs = now; this.text(4, 'Internal pressure critical!'); }
  }
  failsafeInternalTemperatureCheck() {
    const P = this.params, f = this.failsafe, now = this.ms;
    if (!(P.FS_TEMP_ENABLE ?? 0)) return;
    if (!(this.sensors.internalTempC >= (P.FS_TEMP_MAX ?? 62))) { f.lastTempGoodMs = now; f.lastTempWarnMs = now; f.internalTemp = false; return; }
    if (now > f.lastTempGoodMs + 2000) f.internalTemp = true;
    if (f.internalTemp && now > f.lastTempWarnMs + 30000) { f.lastTempWarnMs = now; this.text(4, 'Internal temperature critical!'); }
  }
  failsafeGcsCheck() {
    const P = this.params, f = this.failsafe, fs = P.FS_GCS_ENABLE ?? 2;
    if (!fs) return;
    const last = this.gcsLastSeenMs;
    if (last === 0) return;
    const now = this.ms;
    if (now - last < FS_GCS_TIMEOUT_MS) { f.gcs = false; return; }
    if (now - f.lastGcsWarnMs > 30000) { f.lastGcsWarnMs = now; this.text(4, `MYGCS: ${P.SYSID_MYGCS ?? 255}, heartbeat lost`); }
    if (f.gcs || !this.armed) return;
    f.gcs = true;
    if (fs === 2) this.disarm('gcs failsafe');
    else if (fs === 3 && this.armed) { if (!this.setMode(MODE.ALT_HOLD, 'gcs failsafe')) this.disarm('gcs failsafe hold failed'); }
    else if (fs === 4 && this.armed) { if (!this.setMode(MODE.SURFACE, 'gcs failsafe')) this.disarm('gcs failsafe surface failed'); }
  }
  failsafeTerrainSetStatus(ok) {
    const f = this.failsafe, now = Math.max(1, this.ms);
    if (!ok) { f.terrainLastFailureMs = now; if (f.terrainFirstFailureMs === 0) f.terrainFirstFailureMs = now; }
    else if (now - f.terrainLastFailureMs > 100) { f.terrainLastFailureMs = 0; f.terrainFirstFailureMs = 0; }
  }
  failsafeTerrainCheck() {
    const f = this.failsafe;
    const valid = this.mode === MODE.AUTO || this.mode === MODE.GUIDED;
    const trig = valid && f.terrainLastFailureMs - f.terrainFirstFailureMs > FS_TERRAIN_TIMEOUT_MS;
    if (trig !== f.terrain) {
      if (trig) { this.text(2, 'Failsafe terrain triggered'); this.failsafeTerrainOnEvent(); }
      else f.terrain = false;
    }
  }
  failsafeTerrainOnEvent() {
    this.failsafe.terrain = true;
    if (!this.rangefinderState.enabled || !this.autoTerrainRecoverStart()) this.failsafeTerrainAct();
  }
  failsafeTerrainAct() {
    switch (this.params.FS_TERRAIN_ENAB ?? 0) {
      case 1: if (!this.setMode(MODE.POSHOLD, 'terrain failsafe')) this.setMode(MODE.ALT_HOLD, 'terrain failsafe'); break;
      case 2: this.setMode(MODE.SURFACE, 'terrain failsafe'); break;
      default: this.disarm('terrain failsafe');
    }
  }
  // AP_BattMonitor_Backend::check_failsafe_types
  battFailsafeTypes() {
    const P = this.params, b = this.sensors.battery ?? {};
    const v = (P.BATT_FS_VOLTSRC ?? 0) === 1 ? Math.max(b.voltage ?? 0, b.restingVoltage ?? 0) : b.voltage ?? 0;
    const rem = (P.BATT_CAPACITY ?? 3300) - (b.consumedMah ?? 0);
    return {
      crtV: v > 0 && (P.BATT_CRT_VOLT ?? 0) > 0 && v < P.BATT_CRT_VOLT,
      crtC: !!b.hasCurrent && (P.BATT_CRT_MAH ?? 0) > 0 && rem < P.BATT_CRT_MAH,
      lowV: v > 0 && (P.BATT_LOW_VOLT ?? 0) > 0 && v < P.BATT_LOW_VOLT,
      lowC: !!b.hasCurrent && (P.BATT_LOW_MAH ?? 0) > 0 && rem < P.BATT_LOW_MAH,
    };
  }
  // AP_BattMonitor::check_failsafes (AP_BattMonitor.cpp:789-844) -> Sub::handle_battery_failsafe (failsafe.cpp:153-168)
  batteryFailsafeCheck() {
    const P = this.params, b = this.sensors.battery, s = this.battFs, now = this.ms;
    if (!this.armed || !b?.present || !b.healthy) return;
    const t = this.battFailsafeTypes(), timer = (P.BATT_LOW_TIMER ?? 10) * 1000;
    let type = 0;
    if (t.crtV) { if (s.critStartMs === 0) s.critStartMs = Math.max(1, now); else if (timer > 0 && now - s.critStartMs > timer) type = 2; } else s.critStartMs = 0;
    if (!type && t.crtC) type = 2;
    if (!type) {
      if (t.lowV) { if (s.lowStartMs === 0) s.lowStartMs = Math.max(1, now); else if (timer > 0 && now - s.lowStartMs > timer) type = 1; } else s.lowStartMs = 0;
      if (!type && t.lowC) type = 1;
    }
    if (type <= s.state) return;
    const action = type === 1 ? P.BATT_FS_LOW_ACT ?? 0 : P.BATT_FS_CRT_ACT ?? 0;
    const name = type === 1 ? 'low' : 'critical';
    this.text(4, `Battery 1 is ${name} ${b.voltage.toFixed(2)}V used ${Math.round(b.consumedMah)} mAh`);
    s.state = type;
    this.failsafe.battery = true;
    let prio = BATT_FS_PRIORITY.indexOf(action);
    if (prio < 0) prio = BATT_FS_PRIORITY.length;
    if (prio <= s.highestPriority) {
      s.highestPriority = prio;
      if (action === 3) this.setMode(MODE.SURFACE, 'battery failsafe');
      else if (action === 2) this.disarm('battery failsafe');
    }
  }

  // ---------------- sensor tasks ----------------
  // Sub::read_rangefinder (ArduSub/sensors.cpp:29-75), 20 Hz
  readRangefinder() {
    const P = this.params, rs = this.rangefinderState, s = this.sensors;
    const r = s.rangefinderUpdate ? s.rangefinderUpdate() : s.rangefinder;
    rs.enabled = !!r?.enabled;
    rs.altHealthy = !!r?.healthy;
    rs.altCm = Math.trunc((r?.distance_m ?? 0) * 100);
    rs.inertialAltCm = s.positionZUpCm;
    rs.minCm = P.RNGFND1_MIN_CM ?? 20; rs.maxCm = P.RNGFND1_MAX_CM ?? 700;
    if (rs.altHealthy) {
      const now = this.ms;
      const a = (0.05 * 2 * Math.PI * RANGEFINDER_WPNAV_FILT_HZ) / (0.05 * 2 * Math.PI * RANGEFINDER_WPNAV_FILT_HZ + 1);
      if (now - rs.lastHealthyMs > RANGEFINDER_TIMEOUT_MS || rs.altCmFilt === null) rs.altCmFilt = rs.altCm;
      else rs.altCmFilt += (rs.altCm - rs.altCmFilt) * a;
      rs.lastHealthyMs = now;
      rs.terrainOffsetCm = rs.inertialAltCm - rs.altCmFilt;
    }
  }
  rangefinderAltOk() { const rs = this.rangefinderState; return rs.enabled && rs.altHealthy && this.ms - rs.lastHealthyMs < RANGEFINDER_TIMEOUT_MS; }

  // ---------------- scheduler tasks ----------------
  fiftyHzLoop() {
    this.failsafePilotInputCheck();
    this.failsafeCrashCheck();
    this.failsafeEkfCheck();
    this.failsafeSensorsCheck();
    this.rc.readInput?.(this.ms);
    this.js.update?.(this.time); // mount / servo outputs (g2.actuators.update_actuators)
  }
  updateBattCompass() {
    const s = this.sensors, plant = this.plantRef;
    if (plant && s.batteryRead) s.batteryRead(plant, 0.1);
    this.batteryFailsafeCheck();
    if (plant && s.compassRead) { s.compassRead(plant, s.battery?.current ?? 0, this.motors.getThrottle?.() ?? 0); s.compassReadOnce = true; }
  }
  updateAltitude() { // read_barometer (sensors.cpp:4-16)
    const s = this.sensors;
    if (s.baroUpdate) s.baroUpdate();
    if (this.depthSensorPresent) this.sensorHealthDepth = !!s.depthHealthy;
  }
  baroAccumulate() { /* MS5837 accumulates on its own 100 Hz timer (sensors.update) */ }
  threeHzLoop() {
    this.sensors.leakUpdate?.();
    this.failsafeLeakCheck();
    this.failsafeInternalPressureCheck();
    this.failsafeInternalTemperatureCheck();
    this.failsafeGcsCheck();
    this.failsafeTerrainCheck();
  }
  // Sub::update_turn_counter (turn_counter.cpp)
  updateTurnCounter() {
    const yaw = this.sensors.yaw, q = Math.PI / 2;
    const st = yaw >= 0 && yaw < q ? 0 : yaw > q ? 1 : yaw < -q ? 2 : 3;
    const up = { 0: 1, 1: 2, 2: 3, 3: 0 }, dn = { 0: 3, 1: 0, 2: 1, 3: 2 };
    if (st === up[this.lastTurnState]) this.quarterTurnCount++;
    else if (st === dn[this.lastTurnState]) this.quarterTurnCount--;
    this.lastTurnState = st;
  }
  oneHzLoop() {
    this.preArmOk = this.preArmChecks(false);
    if (!this.armed) this.motors.updateThrottleRange?.();
    this.att.setNotchSampleRate?.(this.loopHz);
    this.pos.pidAccelZ?.setNotchSampleRate?.(this.loopHz);
  }
  terrainUpdate() {}
  temperatureSensorUpdate() {}
  // AP_Arming::update (AP_Arming.cpp:181-198): pre-arm failures shown every 30 s
  updateArming() {
    const now = this.time;
    let display = (this.prearmReportNow && now - this.lastPrearmDisplayMs > 4) || now - this.lastPrearmDisplayMs > PREARM_DISPLAY_PERIOD;
    if (display) { this.prearmReportNow = false; this.lastPrearmDisplayMs = now; }
    if ((this.params.ARMING_OPTIONS ?? 0) & 1) display = false;
    const res = this.preArmChecks(display);
    if (!res && this.lastPrearmResult) this.prearmReportNow = true;
    this.lastPrearmResult = res;
  }

  // motors_output (ArduSub/motors.cpp:10-27)
  motorsOutput(dt) {
    const m = this.motors;
    const passive = () => (m.outputPassive ? m.outputPassive(dt) : m.pwm.slice());
    if (this.mode === MODE.MOTOR_DETECT) return passive().slice();
    if (this.motorTestProto?.active) { this.motorTestProto.verify(this.ms); return passive().slice(); }
    m.setInterlock?.(true);
    m.test = this.motorTest && this.armed ? this.motorTest : null;
    return m.output(dt).slice();
  }

  // One AP_Scheduler tick: the FAST_TASKs in order, then the scheduled tasks whose
  // interval (loop rate / task rate, integer ticks) has elapsed. Sensors must
  // already reflect the plant (ins.update).
  fastLoop(dt) {
    this.time += dt;
    this.tickCounter++;
    if (dt > 0) { const hz = Math.round(1 / dt); if (hz !== this._loopHz) { this._loopHz = hz; this.taskInterval = TASKS.map(([, rate]) => Math.max(1, Math.trunc(hz / rate))); } }
    this.pos.advanceTick();
    this.att.dt = dt; this.pos.dt = dt;
    this.motors.setDt?.(dt);
    const s = this.sensors;
    s.throttleOut = this.motors.getThrottle?.() ?? 0.5;
    // battery readings to the motor current limiter every loop (contract §2)
    const b = s.battery;
    this.motors.setBatteryReadings?.(b?.voltage ?? 0, b?.current ?? 0, !!b?.healthy);
    // run_rate_controller
    if (this.mode !== MODE.MANUAL && this.mode !== MODE.MOTOR_DETECT) this.att.rateControllerRun();
    // motors_output
    const pwm = this.motorsOutput(dt);
    // read_AHRS / read_inertia / check_ekf_yaw_reset
    this.att.checkEkfYawReset?.();
    // update_flight_mode
    this.runMode();
    // update_surface_and_bottom_detector
    this.updateSurfaceBottom();
    // scheduled tasks
    for (let i = 0; i < TASKS.length; i++) {
      if (this.tickCounter - this.taskLastRun[i] < this.taskInterval[i]) continue;
      this.taskLastRun[i] = this.tickCounter;
      this.taskRuns[TASKS[i][0]]++;
      this[TASKS[i][0]]();
    }
    return pwm;
  }
  loop(dt) { return this.fastLoop(dt); }

  // telemetry snapshot (cheap)
  status() {
    const s = this.sensors, f = this.failsafe;
    return {
      armed: this.armed, mode: this.mode, modeLabel: MODE_LABEL[this.mode], gain: this.gain,
      roll: s.roll, pitch: s.pitch, yaw: s.yaw,
      depth: s.depthMeasured, climb: s.velocityZUpCms / 100,
      targetEuler: this.att.eulerAngleTarget.slice(),
      rateTarget: this.att.angVelBody.slice(), gyro: s.gyroLatest.slice(),
      depthTarget: -this.pos.posTarget[2] / 100,
      motorsOut: this.motors.thrust.slice(), pwm: this.motors.pwm.slice(),
      saturated: this.motors.saturated, spool: this.motors.spool,
      lights: [this.js.lights1 / 10, this.js.lights2 / 10], camTilt: this.js.camTilt,
      inputHold: this.js.inputHoldEngaged, rollPitchMode: this.js.rollPitchFlag,
      atSurface: this.atSurface, atBottom: this.atBottom, failsafe: f.pilotInput,
      leak: f.leak, internalPressureFs: f.internalPressure, internalTempFs: f.internalTemp,
      gcsFs: f.gcs, ekfFs: f.ekf, crashFs: f.crash, terrainFs: f.terrain, sensorFs: f.sensorHealth, batteryFs: this.battFs.state,
      battery: s.battery ? { ...s.battery } : null,
      rangefinder: s.rangefinder ? { ...s.rangefinder } : null, rangefinderTargetCm: this.surftrak.targetCm,
      internalTempC: s.internalTempC, turns: this.quarterTurnCount / 4,
      mission: { index: this.mission.index, count: this.mission.items.length, state: this.mission.state },
      throttle: this.motors.getThrottleBidirectional(),
      inputs: [this.motors.rollIn, this.motors.pitchIn, this.motors.yawIn, this.motors.getThrottleBidirectional(), this.motors.forwardIn, this.motors.lateralIn],
    };
  }
}
