// Sensor drivers and state estimate feeding the firmware port. Physical sensor
// rates are decoupled from the main loop; the firmware's scheduler calls the
// frontend tasks at ArduSub's rates (ArduSub.cpp:63-87).
//  - IMU backend at INS_GYRO_RATE (1/2/4/8 kHz with INS_FAST_SAMPLE): turn-on
//    bias + bias random walk + white noise, harmonic notch (INS_HNTCH_*/HNTC2_*)
//    and 2-pole low-pass (INS_GYRO_FILTER/INS_ACCEL_FILTER) at the backend rate.
//  - AHRS/EKF3: roll/pitch held by gravity (small Gauss-Markov error), gyro bias
//    learnt only on observable axes, so yaw drifts without a yaw source
//    (EK3_SRC1_YAW / COMPASS_USE). Board misalignment = AHRS_TRIM at boot, so
//    the trim rotation cancels it as on the real vehicle.
//  - Vertical EKF: z/vz/accel-bias Kalman filter at the delayed horizon
//    (EK3_HGT_DELAY is only the filter's compensation), EK3_HGT_I_GATE gate,
//    output predictor with EK3_TAU_OUTPUT (AP_NavEKF3 calcOutputStates); ArduSub
//    forces the height noise to 0.1 m (10 m without a depth sensor, system.cpp:134-141).
//  - Depth: MS5837/Keller conversions (cfg.depthRate, physical latency
//    cfg.depthLatency), BARO_FLTR_RNG mean filter, AP_Baro frontend average at
//    10 Hz (update_altitude), water formula with BARO_SPEC_GRAV, BARO_ALT_OFFSET
//    slew, ground-pressure ratchet (sensors.cpp:9-11), presence from
//    BARO_EXT_BUS / BARO_PROBE_EXT.
//  - Rangefinder (RNGFND1_*), compass (COMPASS_*, motor-current interference),
//    battery monitor (BATT_*, from plant.battery), enclosure temperature
//    (FS_TEMP) and water temperature (TEMP1), leak probes (AP_LeakDetector).

import { qFromEuler, qToEuler, qmul, qconj, qFromAxisAngleVec, qToMat, m3mulv, m3tmulv, wrap360cd, wrapPI, clamp } from '../core/math.js';
import { LowPassFilter2p } from './control.js';
import { GaussMarkov } from '../core/rng.js';
import { depthSensorSpec } from '../vehicle/config.js';

const G = 9.80665;
const LEAKDETECTOR_COOLDOWN = 3; // s, AP_LeakDetector.h:7
const RF_STATUS = { NotConnected: 0, NoData: 1, OutOfRangeLow: 2, OutOfRangeHigh: 3, Good: 4 };
export { RF_STATUS };

// ArduPilot NotchFilter (Filter/NotchFilter.cpp:30-110)
class Notch {
  init(fs, fc, bw, attDb) {
    this.ok = false;
    if (!(fc > 0.5 * bw && fc < 0.5 * fs)) return;
    const A = Math.pow(10, -attDb / 40);
    const oct = Math.log2(fc / (fc - bw / 2)) * 2;
    const Q = Math.sqrt(Math.pow(2, oct)) / (Math.pow(2, oct) - 1);
    if (!(Q > 0)) return;
    const w = (2 * Math.PI * fc) / fs, al = Math.sin(w) / (2 * Q), a0 = 1 / (1 + al);
    this.b0 = (1 + al * A * A) * a0; this.b1 = -2 * Math.cos(w) * a0; this.b2 = (1 - al * A * A) * a0;
    this.a1 = this.b1; this.a2 = (1 - al) * a0;
    this.ok = true;
  }
  apply(x) {
    if (!this.ok) return x;
    if (this.s1 === undefined) { this.s1 = this.s2 = this.n1 = this.n2 = x; return x; }
    const y = x * this.b0 + this.n1 * this.b1 + this.n2 * this.b2 - this.s1 * this.a1 - this.s2 * this.a2;
    this.n2 = this.n1; this.n1 = x; this.s2 = this.s1; this.s1 = y;
    return y;
  }
  reset() { this.s1 = undefined; }
}

export class SensorSuite {
  constructor(rng) {
    this.rng = rng;
    this.attErr = [new GaussMarkov(rng), new GaussMarkov(rng), new GaussMarkov(rng)];
    this.cfg = {};
    this.params = {};
    // fault injection for tests / UI (sensor disconnected or failed)
    this.faults = { depth: false, rangefinder: false, compass: false, battery: false, ekf: false, magOffset: null };
    this.reset();
  }

  // Configure from the vehicle config and parameters. Keeps the sensor state
  // (filters, estimates, biases) so a rebuild does not reboot the sensors.
  configure(cfg, params, loopHz, env) {
    this.cfg = cfg || {};
    this.env = env;
    this.loopHz = loopHz;
    this.spec = depthSensorSpec(this.cfg);
    this.attErr.forEach((g, i) => { g.sigma = i < 2 ? ((this.cfg.attitudeNoise ?? 0) * Math.PI) / 180 : 0; g.tau = 5; });
    this.setParams(params || {});
  }

  // Parameters (live edits allowed): rebuilds only what changed.
  setParams(P) {
    this.params = { ...this.params, ...P };
    const p = this.params;
    // INS backend rate: INS_GYRO_RATE 0..3 = 1/2/4/8 kHz when fast sampling (AP_InertialSensor)
    const fast = (p.INS_FAST_SAMPLE ?? 1) & 1;
    const imuHz = fast ? 1000 * (1 << clamp(Math.trunc(p.INS_GYRO_RATE ?? 0), 0, 3)) : 1000;
    const gf = p.INS_GYRO_FILTER ?? 20, af = p.INS_ACCEL_FILTER ?? 20;
    if (imuHz !== this.imuHz || gf !== this._gf || af !== this._af || !this.gyroF) {
      this.imuHz = imuHz; this._gf = gf; this._af = af;
      this.gyroF = [0, 1, 2].map(() => new LowPassFilter2p(imuHz, gf));
      this.accF = [0, 1, 2].map(() => new LowPassFilter2p(imuHz, af));
    }
    // harmonic notches (INS_HNTCH_*, INS_HNTC2_*), fixed or throttle tracking
    this.notchCfg = ['INS_HNTCH_', 'INS_HNTC2_'].map((k) => ({
      on: (p[k + 'ENABLE'] ?? 0) > 0, freq: p[k + 'FREQ'] ?? 80, bw: p[k + 'BW'] ?? 40, att: p[k + 'ATT'] ?? 40,
      hm: p[k + 'HMNCS'] ?? 3, ref: p[k + 'REF'] ?? 0, mode: p[k + 'MODE'] ?? 1, fmr: p[k + 'FM_RAT'] ?? 1,
    })).filter((n) => n.on);
    const nkey = this.notchCfg.map((n) => n.hm).join();
    if (!this.notches || this._nkey !== nkey) {
      this._nkey = nkey;
      this.notches = this.notchCfg.map((n) => { const hs = []; for (let h = 0; h < 8; h++) if (n.hm & (1 << h)) hs.push(h + 1); return { hs, f: [0, 1, 2].map(() => hs.map(() => new Notch())), fc: 0 }; });
    }
    this.notchCfg.forEach((n, i) => this.updateNotch(i, n.freq));
    this.specGrav = p.BARO_SPEC_GRAV ?? 1;
    this.hgtDelay = clamp((p.EK3_HGT_DELAY ?? 60) / 1000, 0, 0.25);
    this.altOffset = this._booted ? (p.BARO_ALT_OFFSET ?? 0) : 0; // AP_Baro::calibrate zeroes it at boot (AP_Baro.cpp:311)
    // depth sensor present: ArduSub probes MS5837/Keller on BARO_EXT_BUS, or on all
    // external buses for BARO_PROBE_EXT bits 8/9 (AP_Baro.cpp:748-757, 806-875)
    const fitted = (this.cfg.depthSensor ?? 'bar30') !== 'none';
    this.depthSensorPresent = fitted && ((p.BARO_EXT_BUS ?? 1) >= 0 || ((p.BARO_PROBE_EXT ?? 0) & 0x300) !== 0);
    this.depthSensorWarning = fitted && !this.depthSensorPresent ? 'Depth sensor fitted but not probed (BARO_EXT_BUS -1 and BARO_PROBE_EXT without MS5837/Keller)' : '';
    // leak detector instances (AP_LeakDetector.cpp:5-120): type from LEAKn_TYPE, or the board default from the pin
    this.leakInst = [];
    for (let i = 1; i <= 3; i++) {
      const pin = p[`LEAK${i}_PIN`] ?? -1;
      let type = p[`LEAK${i}_TYPE`] ?? -1;
      if (type < 0) type = pin >= 13 && pin <= 15 ? 0 : (pin >= 50 && pin <= 55) || pin === 27 ? 1 : -1;
      if (pin >= 0 && (type === 0 || type === 1)) this.leakInst.push({ pin, type, logic: p[`LEAK${i}_LOGIC`] ?? 0 });
    }
    this.leakConfigWarning = '';
    if (!this.leakInst.length && this.cfg.leakProbeFitted) {
      // a probe fitted in the simulator without LEAKn_PIN: wired as an SOS probe on a digital pin
      this.leakInst.push({ pin: -2, type: 1, logic: 0 });
      this.leakConfigWarning = 'Leak probe fitted but no LEAKn_PIN configured: the real firmware would not read it';
    }
  }

  updateNotch(i, fc) {
    const n = this.notches[i], c = this.notchCfg[i];
    if (!n || Math.abs(fc - n.fc) < 1e-6) return;
    n.fc = fc;
    for (let ax = 0; ax < 3; ax++) n.hs.forEach((h, k) => n.f[ax][k].init(this.imuHz, fc * h, c.bw * h, c.att));
  }

  reset() {
    const rng = this.rng;
    this._booted = false;
    this.t = 0;
    this.imuCount = 0;
    this.prevTruth = null;
    this.gyro = [0, 0, 0];
    this.gyroLatest = [0, 0, 0];
    this.gyroRaw = [0, 0, 0];
    this.accelBody = [0, 0, -G];
    this.accelEf = [0, 0, -G];
    this.quat = [1, 0, 0, 0];
    this.euler = [0, 0, 0];
    this.gyroF?.forEach((f) => f.reset());
    this.accF?.forEach((f) => f.reset());
    this.notches?.forEach((n) => n.f.forEach((a) => a.forEach((f) => f.reset())));
    this.attErr.forEach((g) => { g.x = 0; });
    // turn-on bias draws (scaled by cfg so the sigmas can change live) and random walks
    this.gyroBiasUnit = [rng.gauss(), rng.gauss(), rng.gauss()];
    this.accelBiasUnit = [rng.gauss(), rng.gauss(), rng.gauss()];
    this.gyroWalk = [0, 0, 0]; this.accelWalk = [0, 0, 0];
    this.gyroBiasEst = [0, 0, 0];
    this.yawErr = 0;
    this.mis = null; // board misalignment, latched at the first update
    // baro
    this.convCount = 0;
    this.baroQueue = [];
    this.baroAcc = { sum: 0, n: 0 };
    this.meanPressure = 0;
    this.baroErrors = 0;
    this.baroPress = NaN;
    this.baroLastUpdate = -1e9;
    this.baroAlt = null; // m, up positive (negative underwater), primary baro frontend
    this.baroNew = false;
    this.groundPressure = NaN;
    this.altOffsetActive = 0;
    this.depthHealthy = false;
    this.pressureMeas = NaN; // Pa absolute, external sensor (frontend)
    this.pressureTrue = NaN;
    this.depthOverRange = false;
    this.internalPressure = 101325;
    this.internalGndPressure = NaN;
    // vertical EKF
    this.ekfZ = null;
    this.ekfVar = { vel: 0, pos: 0, hgt: 0, mag: 0 };
    this.hgtRejectStart = -1;
    // horizontal estimate
    this.inav = null;
    this.nextPos = 0;
    this.posMeas = null;
    this.velNed = [0, 0, 0];
    // rangefinder
    this.rfBackend = { status: RF_STATUS.NotConnected, dist: 0, quality: -1, lastMs: -1e9 };
    this.nextRf = 0;
    this.rangefinder = { enabled: false, healthy: false, status: RF_STATUS.NotConnected, distance_m: 0, signalQuality: -1, validCount: 0 };
    // compass
    this.compass = { enabled: false, used: false, healthy: false, headingDeg: 0, field: [0, 0, 0] };
    this.compassInnov = 0;
    // battery monitor
    this.battery = { voltage: 0, current: 0, consumedMah: 0, consumedWh: 0, remainingPct: NaN, healthy: false, hasCurrent: false, restingVoltage: 0, resistance: 0 };
    this.battRes = null;
    this.batteryWarning = '';
    // temperature
    this.internalTempC = NaN;
    this.waterTempC = NaN;
    // leak
    this.leakProbeWet = false;
    this.leak = false;
    this.leakLastDetect = -1e9;
  }

  // plant: VehiclePlant; dt: firmware loop period. Runs the sensor backends
  // (IMU, baro conversions, rangefinder) and the AHRS/EKF prediction.
  update(plant, dt) {
    const c = this.cfg, rng = this.rng, P = this.params, env = this.env;
    const tPrev = this.t;
    this.t += dt;
    const t = this.t;
    if (!this._booted) this.boot(plant);
    // ---------------- IMU backend ----------------
    const w = plant.rates, f = plant.specificForceBody();
    const pt = this.prevTruth || { w, f };
    const bStd = ((c.gyroBias ?? 0.02) * Math.PI) / 180; // deg/s turn-on residual (RMS)
    const bWalk = ((c.gyroBiasWalk ?? 0.0005) * Math.PI) / 180; // deg/s/sqrt(s)
    const aStd = c.accelBias ?? 0.02, aWalk = c.accelBiasWalk ?? 0.0005;
    for (let i = 0; i < 3; i++) {
      this.gyroWalk[i] += bWalk * Math.sqrt(dt) * rng.gauss();
      this.accelWalk[i] += aWalk * Math.sqrt(dt) * rng.gauss();
    }
    const bg = [0, 1, 2].map((i) => this.gyroBiasUnit[i] * bStd + this.gyroWalk[i]);
    const ba = [0, 1, 2].map((i) => this.accelBiasUnit[i] * aStd + this.accelWalk[i]);
    this.gyroBiasTrue = bg;
    const gn = c.gyroNoise ?? 0, an = c.accelNoise ?? 0;
    // notch tracking (throttle mode 1, AP_Vehicle::update_throttle_notch)
    this.notchCfg.forEach((n, i) => {
      if (n.mode === 1 && n.ref > 0) this.updateNotch(i, n.freq * Math.max(n.fmr, Math.sqrt(Math.max(0, this.throttleOut ?? 0.5) / n.ref)));
    });
    // samples at the backend rate (integer sample clock, no drift)
    while ((this.imuCount + 1) / this.imuHz <= t + 1e-9) {
      this.imuCount++;
      const ts = this.imuCount / this.imuHz;
      const a = dt > 0 ? clamp((ts - tPrev) / dt, 0, 1) : 1;
      for (let i = 0; i < 3; i++) {
        let g = pt.w[i] + (w[i] - pt.w[i]) * a + bg[i] + gn * rng.gauss();
        this.gyroRaw[i] = g;
        for (const n of this.notches) for (const nf of n.f[i]) g = nf.apply(g);
        this.gyro[i] = this.gyroF[i].apply(g);
        const fa = pt.f[i] + (f[i] - pt.f[i]) * a + ba[i] + an * rng.gauss();
        this.accelBody[i] = this.accF[i].apply(fa);
      }
    }
    this.prevTruth = { w, f };
    // ---------------- AHRS (EKF3 attitude) ----------------
    const yawSrc = this.yawSourceOk();
    // gyro bias learning: roll/pitch through gravity, yaw only with a yaw source
    for (let i = 0; i < 3; i++) {
      const tau = i < 2 ? 20 : 60;
      if (i < 2 || yawSrc) this.gyroBiasEst[i] += ((bg[i] - this.gyroBiasEst[i]) * dt) / tau;
    }
    this.yawErr += (bg[2] - this.gyroBiasEst[2]) * dt;
    if (yawSrc === 'compass' && this.compass.healthy) this.yawErr += ((this.compassErr - this.yawErr) * dt) / 3;
    else if (yawSrc === 'extnav') this.yawErr -= (this.yawErr * dt) / 1;
    this.yawErr = wrapPI(this.yawErr);
    for (let i = 0; i < 3; i++) this.gyroLatest[i] = this.gyro[i] - this.gyroBiasEst[i];
    let q = plant.quaternion;
    if ((c.attitudeNoise ?? 0) > 0) q = qmul(q, qFromAxisAngleVec(this.attErr.map((g) => g.step(dt))));
    q = qmul(qFromAxisAngleVec([0, 0, this.yawErr]), q);
    // board misalignment (physical) then AHRS_TRIM rotation: vehicle = board * trim^-1
    const trim = qFromEuler(P.AHRS_TRIM_X ?? 0, P.AHRS_TRIM_Y ?? 0, P.AHRS_TRIM_Z ?? 0);
    q = qmul(qmul(q, this.mis), qconj(trim));
    this.quat = q;
    this.euler = qToEuler(q);
    const R = qToMat(q);
    this.accelEf = m3mulv(R, this.accelBody); // specific force in NED
    // ---------------- external depth sensor conversions ----------------
    const pos = plant.position;
    const Rq = qToMat(plant.quaternion);
    const sp = c.depthSensorPos || [0, 0, 0];
    const sPos = [0, 1, 2].map((k) => pos[k] + Rq[3 * k] * sp[0] + Rq[3 * k + 1] * sp[1] + Rq[3 * k + 2] * sp[2]);
    const convHz = Math.max(1, c.depthRate ?? 80), lat = Math.max(0, c.depthLatency ?? 0.02);
    while ((this.convCount + 1) / convHz <= t + 1e-9) {
      this.convCount++;
      const pTrue = env.absolutePressureAt(sPos[0], sPos[1], sPos[2]);
      this.pressureTrue = pTrue;
      this.depthOverRange = pTrue > this.spec.maxPa;
      const pm = this.quantize(pTrue + (c.depthNoise ?? 0) * env.rho * env.g * rng.gauss());
      this.baroQueue.push({ t: this.convCount / convHz + lat, p: pm });
    }
    while (this.baroQueue.length && this.baroQueue[0].t <= t + 1e-9) {
      const b = this.baroQueue.shift();
      if (this.pressureOk(b.p)) { this.baroAcc.sum += b.p; this.baroAcc.n++; }
    }
    // ---------------- enclosure: onboard baro, temperature, leak probes ----------------
    const fl = plant.flood;
    this.internalPressure = fl ? fl.pIn : env.pAtm;
    if (!(this.internalGndPressure > 0)) this.internalGndPressure = this.internalPressure;
    this.updateTemperature(plant, dt);
    this.leakProbeWet = !!(fl && fl.volume > (c.leakProbeVolume ?? 1e-6));
    // ---------------- rangefinder backend ----------------
    this.rangefinderBackend(plant);
    // ---------------- vertical EKF + output predictor ----------------
    this.ekfVertical(plant, dt);
    // ---------------- horizontal positioning (optional DVL/USBL) ----------------
    const vned = m3mulv(Rq, plant.velocityBody);
    const nav = this.inav;
    if (c.positioning) {
      if (t >= this.nextPos) {
        this.nextPos = t + 1 / Math.max(1, c.positionRate ?? 10);
        const pn = c.positionNoise ?? 0;
        this.posMeas = [pos[0] + pn * rng.gauss(), pos[1] + pn * rng.gauss()];
        const innov = Math.hypot(this.posMeas[0] - nav.x, this.posMeas[1] - nav.y);
        this.ekfVar.vel = innov / 2.5; // EK3_POSNE_M_NSE 0.5 m x 5-sigma gate
      }
      const a = 1 - Math.exp(-dt / 0.3);
      nav.vx += (vned[0] - nav.vx) * a;
      nav.vy += (vned[1] - nav.vy) * a;
      nav.x += nav.vx * dt; nav.y += nav.vy * dt;
      if (this.posMeas) { nav.x += (this.posMeas[0] - nav.x) * 2 * dt; nav.y += (this.posMeas[1] - nav.y) * 2 * dt; }
    } else this.ekfVar.vel = 0;
    if (this.faults.ekf) { this.ekfVar.vel = 2; this.ekfVar.mag = 2; }
    this.velNed = vned;
  }

  boot(plant) {
    const P = this.params, c = this.cfg, env = this.env;
    this._booted = true;
    this.mis = c.boardMisalign ? qFromEuler(...c.boardMisalign) : qFromEuler(P.AHRS_TRIM_X ?? 0, P.AHRS_TRIM_Y ?? 0, P.AHRS_TRIM_Z ?? 0);
    this.altOffset = 0; this.altOffsetActive = 0;
    // AP_Baro::calibrate(false): stored BAROn_GND_PRESS if positive, else the
    // current pressure (AP_Baro.cpp:352-356, 924-927). The simulated vehicle is
    // powered on at the surface unless the stored value is requested.
    const stored = P[`BARO${c.depthBaroInstance ?? 2}_GND_PRESS`] ?? 0;
    this.groundPressure = c.baroGndPressFromParams && stored > 0 ? stored : this.quantize(env ? env.pAtm : 101325);
    this.internalTempC = env?.cfg?.airTemperature ?? Math.max(20, env?.cfg?.temperature ?? 20);
    const v = m3mulv(qToMat(plant.quaternion), plant.velocityBody);
    this.inav = { x: plant.position[0], y: plant.position[1], vx: v[0], vy: v[1] };
    // calibrate() let the sensors settle before init: the frontends start healthy with a reading
    const Rq = qToMat(plant.quaternion), sp = c.depthSensorPos || [0, 0, 0], pos = plant.position;
    const s = [0, 1, 2].map((k) => pos[k] + Rq[3 * k] * sp[0] + Rq[3 * k + 1] * sp[1] + Rq[3 * k + 2] * sp[2]);
    this.pressureTrue = env.absolutePressureAt(s[0], s[1], s[2]);
    this.baroAcc = { sum: this.quantize(this.pressureTrue), n: 1 };
    this.internalPressure = plant.flood ? plant.flood.pIn : env.pAtm;
    this.internalGndPressure = this.internalPressure;
    this.baroUpdate();
  }

  quantize(p) {
    const s = this.spec;
    if (!s) return p;
    let v = Math.min(s.maxPa, Math.max(0, p));
    // MS5837 driver output is 0.1 mbar (AP_Baro_MS5611.cpp:514)
    const res = Math.max(10, s.resPa || 0);
    v = Math.round(v / res) * res;
    return v;
  }

  // AP_Baro_Backend::pressure_ok mean filter (BARO_FLTR_RNG, AP_Baro_Backend.cpp:72-100)
  pressureOk(p) {
    if (!Number.isFinite(p)) return false;
    const range = this.params.BARO_FLTR_RNG ?? 0;
    if (range <= 0) return true;
    if (this.meanPressure === 0) { this.meanPressure = p; return true; }
    const d = Math.abs(this.meanPressure - p) / (this.meanPressure + p);
    let k = 0.1, ok = true;
    if (d * 200 > range) { ok = false; k /= d * 10; this.baroErrors++; }
    this.meanPressure = this.meanPressure * (1 - k) + p * k;
    return ok;
  }

  // ---------------- frontend tasks called by the firmware scheduler ----------------
  // AP_Baro::update() from read_barometer (update_altitude, 10 Hz) + ArduSub's
  // ground-pressure ratchet (ArduSub/sensors.cpp:4-16).
  baroUpdate() {
    const t = this.t;
    this.altOffsetActive = Math.abs(this.altOffset - this.altOffsetActive) > 0.01 ? 0.98 * this.altOffsetActive + 0.02 * this.altOffset : this.altOffset;
    if (this.baroAcc.n > 0 && !this.faults.depth) {
      this.baroPress = this.baroAcc.sum / this.baroAcc.n;
      this.baroLastUpdate = t;
    }
    this.baroAcc.sum = 0; this.baroAcc.n = 0;
    const healthy = this.depthSensorPresent && !this.faults.depth && t - this.baroLastUpdate < 0.5;
    this.depthHealthy = healthy;
    if (this.depthSensorPresent) {
      if (healthy) {
        if (!(this.groundPressure > 0)) this.groundPressure = this.baroPress;
        this.pressureMeas = this.baroPress;
        this.baroAlt = (this.groundPressure - this.baroPress) / 9800 / this.specGrav + this.altOffsetActive;
        if (this.baroAlt > 0) this.groundPressure = this.baroPress; // update_calibration()
        this.baroNew = true;
      }
    } else {
      // onboard (air) baro is primary: altitude of the enclosure pressure
      const p = this.internalPressure * (1 + 2e-6 * this.rng.gauss()), gp = this.internalGndPressure;
      this.baroAlt = 153.8462 * 288.15 * (1 - Math.exp(0.190259 * Math.log(p / gp))) + this.altOffsetActive;
      if (this.baroAlt > 0) this.internalGndPressure = p;
      this.baroNew = true;
    }
    return healthy;
  }

  // AP_RangeFinder frontend (read_rangefinder, 20 Hz): status + valid count
  rangefinderUpdate() {
    const P = this.params, b = this.rfBackend, r = this.rangefinder;
    r.enabled = (P.RNGFND1_TYPE ?? 0) !== 0 && (P.RNGFND1_ORIENT ?? 25) === 25;
    if (!r.enabled) { r.status = RF_STATUS.NotConnected; r.healthy = false; r.validCount = 0; return r; }
    let st = b.status;
    if (st !== RF_STATUS.NotConnected && this.t - b.lastMs > 0.2) st = RF_STATUS.NoData;
    r.status = st;
    r.validCount = st === RF_STATUS.Good ? Math.min(10, r.validCount + 1) : 0;
    r.distance_m = b.dist;
    r.signalQuality = b.quality;
    r.minCm = P.RNGFND1_MIN_CM ?? 20; r.maxCm = P.RNGFND1_MAX_CM ?? 700;
    // rangefinder_state.alt_healthy (ArduSub/sensors.cpp:37-40), RANGEFINDER_HEALTH_MAX 3
    r.healthy = st === RF_STATUS.Good && r.validCount >= 3 && (b.quality === -1 || b.quality >= (P.RNGFND_SQ_MIN ?? 90));
    return r;
  }

  rangefinderBackend(plant) {
    const P = this.params, c = this.cfg, b = this.rfBackend;
    if ((P.RNGFND1_TYPE ?? 0) === 0 || this.faults.rangefinder) { b.status = RF_STATUS.NotConnected; return; }
    if (this.t < this.nextRf) return;
    this.nextRf = this.t + 1 / Math.max(1, c.rangefinderRate ?? 10);
    if (b.status === RF_STATUS.NotConnected) b.status = RF_STATUS.NoData;
    const env = this.env, R = qToMat(plant.quaternion), p = plant.position;
    const off = [P.RNGFND1_POS_X ?? 0, P.RNGFND1_POS_Y ?? 0, P.RNGFND1_POS_Z ?? 0];
    const s = [0, 1, 2].map((k) => p[k] + R[3 * k] * off[0] + R[3 * k + 1] * off[1] + R[3 * k + 2] * off[2]);
    const d = [R[2], R[5], R[8]]; // body down axis in NED (ROTATION_PITCH_270)
    if (s[2] + env.surfaceElevation(s[0], s[1]) < 0.02 || d[2] < 0.05) { b.status = RF_STATUS.NoData; return; } // sonar out of water / pointing up
    // ray against the floor profile depth(x) and the walls
    let tt = (env.floorDepth(s[0]) - s[2]) / d[2];
    for (let k = 0; k < 4; k++) tt = (env.floorDepth(s[0] + tt * d[0]) - s[2]) / d[2];
    const fl = env.floor;
    if (Math.abs(d[0]) > 1e-6) { const tw = ((Math.sign(d[0]) * fl.L) / 2 - s[0]) / d[0]; if (tw > 0 && tw < tt) tt = tw; }
    if (Math.abs(d[1]) > 1e-6) { const tw = ((Math.sign(d[1]) * fl.W) / 2 - s[1]) / d[1]; if (tw > 0 && tw < tt) tt = tw; }
    const dist = Math.max(0, tt + ((c.rangefinderNoise ?? 0.005) + 0.002 * tt) * this.rng.gauss());
    const cm = dist * 100, mn = P.RNGFND1_MIN_CM ?? 20, mx = P.RNGFND1_MAX_CM ?? 700;
    b.dist = dist;
    b.lastMs = this.t;
    b.status = cm < mn ? RF_STATUS.OutOfRangeLow : cm > mx ? RF_STATUS.OutOfRangeHigh : RF_STATUS.Good;
    // Ping sonar (TYPE 23) reports confidence: falls off with incidence beyond the 30 deg beam
    const inc = (Math.acos(clamp(d[2], -1, 1)) * 180) / Math.PI;
    b.quality = (P.RNGFND1_TYPE ?? 0) === 23 ? Math.round(clamp(100 - Math.max(0, inc - 15) * 3.3, 0, 100)) : -1;
  }

  // update_batt_compass (10 Hz): compass read
  compassRead(plant, currentA = 0, throttle = 0) {
    const P = this.params, c = this.cfg, cp = this.compass;
    cp.enabled = (P.COMPASS_ENABLE ?? 1) !== 0;
    cp.used = cp.enabled && ((P.COMPASS_USE ?? 1) !== 0 || (P.COMPASS_USE2 ?? 1) !== 0 || (P.COMPASS_USE3 ?? 1) !== 0);
    cp.healthy = cp.enabled && !this.faults.compass && c.compassFitted !== false;
    if (!cp.healthy) return cp;
    // earth field (mGauss, NED) with the local declination, into the body frame
    const decl = ((c.magDeclination ?? 0) * Math.PI) / 180, H = c.magHorizontal ?? 200, Z = c.magDown ?? 400;
    const mNed = [H * Math.cos(decl), H * Math.sin(decl), Z];
    const Rt = qToMat(plant.quaternion);
    const m = m3tmulv(Rt, mNed);
    // motor-current interference (mGauss/A, body) minus COMPASS_MOT compensation (COMPASS_MOTCT 1 throttle, 2 current)
    const k = c.compassMotPerA ?? [-4, 2, 6];
    const mct = P.COMPASS_MOTCT ?? 0, comp = [P.COMPASS_MOT_X ?? 0, P.COMPASS_MOT_Y ?? 0, P.COMPASS_MOT_Z ?? 0];
    const off = this.faults.magOffset || c.magDisturbance || [0, 0, 0];
    for (let i = 0; i < 3; i++) {
      m[i] += k[i] * currentA + off[i] + 2 * this.rng.gauss();
      if (mct === 2) m[i] -= comp[i] * currentA; else if (mct === 1) m[i] -= comp[i] * throttle;
    }
    cp.field = m;
    // tilt-compensated heading with the estimated roll/pitch, + COMPASS_DEC
    const [r, pch] = this.euler;
    const cr = Math.cos(r), sr = Math.sin(r), cpch = Math.cos(pch), spch = Math.sin(pch);
    const hx = m[0] * cpch + m[1] * sr * spch + m[2] * cr * spch;
    const hy = m[1] * cr - m[2] * sr;
    const head = wrapPI(Math.atan2(-hy, hx) + (P.COMPASS_DEC ?? 0));
    cp.headingDeg = ((head * 180) / Math.PI + 360) % 360;
    const trueYaw = qToEuler(plant.quaternion)[2];
    this.compassErr = wrapPI(head - trueYaw);
    // EKF mag innovation test ratio (EK3_MAG_M_NSE 0.05 G over the 0.2 G horizontal field, 3-sigma gate)
    this.compassInnov = wrapPI(head - this.euler[2]);
    this.ekfVar.mag = this.yawSourceOk() === 'compass' ? Math.abs(this.compassInnov) / (3 * Math.atan2(50, H)) : 0;
    return cp;
  }
  yawSourceOk() {
    const P = this.params, src = P.EK3_SRC1_YAW ?? 1;
    if ((src === 1 || src === 3) && this.compass.used && this.compass.healthy) return 'compass';
    if (src === 6 && (P.VISO_TYPE ?? 0) > 0) return 'extnav';
    return '';
  }

  // update_batt_compass (10 Hz): AP_BattMonitor read from the plant's battery (contract §2)
  batteryRead(plant, dt) {
    const P = this.params, b = this.battery, pb = plant.battery;
    const mon = P.BATT_MONITOR ?? 0;
    if (mon === 0) { b.healthy = false; b.present = false; return b; }
    b.present = true;
    if (!pb || this.faults.battery) { b.healthy = false; this.batteryWarning = pb ? '' : 'BATT_MONITOR set but the plant has no battery model'; return b; }
    b.hasCurrent = mon !== 3; // 3 = analog voltage only
    b.voltage = (pb.voltage ?? 0) + 0.005 * this.rng.gauss();
    b.current = b.hasCurrent ? Math.max(0, (pb.current ?? 0) + 0.02 * this.rng.gauss()) : 0;
    if (b.hasCurrent) { b.consumedMah += (b.current * dt * 1000) / 3600; b.consumedWh += (b.current * b.voltage * dt) / 3600; }
    const cap = P.BATT_CAPACITY ?? 3300;
    b.remainingPct = b.hasCurrent && cap > 10 ? clamp(100 * (1 - b.consumedMah / cap), 0, 100) : NaN;
    b.healthy = true;
    this.updateResistance(b, dt);
    return b;
  }
  // AP_BattMonitor_Backend::update_resistance_estimate (AP_BattMonitor_Backend.cpp:63-108)
  updateResistance(b, dt) {
    if (!this.battRes) this.battRes = { vFilt: b.voltage, iFilt: b.current, iMax: 0, vRef: 0, iRef: 0 };
    const s = this.battRes;
    if (b.hasCurrent && b.current > 0) {
      s.iMax = Math.max(s.iMax, b.current);
      const di = b.current - s.iFilt;
      if (Math.abs(di) > 1e-7) {
        if (b.voltage > s.vRef) { s.vRef = b.voltage; s.iRef = b.current; }
        const fa = clamp(dt / (dt + 0.5), 0, 0.5);
        const ra = Math.min(1, (0.1 * Math.abs(di)) / s.iMax);
        const est = -(b.voltage - s.vFilt) / di;
        if (est > 0) b.resistance = b.resistance * (1 - ra) + est * ra;
        if (s.vRef > b.voltage && b.current > s.iRef) b.resistance = Math.min(b.resistance, (s.vRef - b.voltage) / (b.current - s.iRef));
        s.vFilt = s.vFilt * (1 - fa) + b.voltage * fa;
        s.iFilt = s.iFilt * (1 - fa) + b.current * fa;
      }
    }
    b.restingVoltage = Math.max(b.voltage, b.voltage + b.current * b.resistance);
  }

  // enclosure temperature (first-order thermal model) and water temperature (TEMP1)
  updateTemperature(plant, dt) {
    const c = this.cfg, env = this.env;
    const tw = env?.cfg?.temperature ?? 20;
    const wet = (plant.diag?.submerged ?? 1) > 0.5;
    const tAmb = wet ? tw : env?.cfg?.airTemperature ?? 25;
    const heat = (c.enclosureHeatW ?? 8) + 0.03 * Math.abs((this.battery.voltage || 0) * (this.battery.current || 0));
    const Rth = c.enclosureRth ?? (wet ? 1.0 : 4.0), tau = c.enclosureTau ?? 600;
    const tEq = tAmb + heat * Rth;
    this.internalTempC += ((tEq - this.internalTempC) * dt) / tau;
    if (c.internalTempOverride !== undefined && c.internalTempOverride !== null) this.internalTempC = c.internalTempOverride;
    this.waterTempC = (this.params.TEMP1_TYPE ?? 0) !== 0 ? tw : NaN;
  }

  // AP_LeakDetector::update (three_hz_loop): probes, LEAKn_LOGIC, 3 s cooldown
  leakUpdate() {
    let any = false;
    for (const L of this.leakInst) {
      // SOS probe: output high (3.3 V) when wet, low when dry
      const level = this.cfg.leakProbeFitted !== false && this.leakProbeWet ? 1 : 0; // no probe: pin pulled low
      const reading = L.type === 0 ? (level * 3.3 > 2.0 ? 1 : 0) : level;
      if (reading !== L.logic) any = true;
    }
    if (any) this.leakLastDetect = this.t;
    this.leak = this.t < this.leakLastDetect + LEAKDETECTOR_COOLDOWN;
    return this.leak;
  }

  // vertical EKF at the delayed time horizon plus output predictor
  ekfVertical(plant, dt) {
    const P = this.params;
    const azUp = -(this.accelEf[2] + G);
    const nd = Math.max(0, Math.round(this.hgtDelay / dt));
    if (!this.ekfZ) {
      const v = m3mulv(qToMat(plant.quaternion), plant.velocityBody);
      const z0 = this.baroAlt ?? 0;
      this.ekfZ = { z: z0, vz: -v[2], ab: 0, P: [0.25, 0, 0, 0, 0.25, 0, 0, 0, 0.01], hist: [], ozInt: 0, ovInt: 0, oz: z0, ov: -v[2] };
    }
    const E = this.ekfZ;
    // IMU/output history (oldest first), length nd+1
    E.hist.push({ az: azUp, oz: E.oz, ov: E.ov });
    while (E.hist.length > nd + 1) E.hist.shift();
    const del = E.hist[0];
    // ---- predict the delayed state with the delayed IMU sample ----
    const a = del.az - E.ab;
    E.z += E.vz * dt + 0.5 * a * dt * dt;
    E.vz += a * dt;
    const Pm = E.P, F = [1, dt, -0.5 * dt * dt, 0, 1, -dt, 0, 0, 1];
    const FP = mul3(F, Pm), Pn = mul3T(FP, F);
    const qa = (P.EK3_ACC_P_NSE ?? 0.35) * dt, qb = (P.EK3_ABIAS_P_NSE ?? 0.003) * dt;
    Pn[4] += qa * qa; Pn[0] += 0.25 * qa * qa * dt * dt; Pn[8] += qb * qb;
    E.P = Pn;
    // ---- fuse the baro at the delayed horizon ----
    if (this.baroNew && this.baroAlt !== null) {
      this.baroNew = false;
      const Rn = this.depthSensorPresent ? 0.1 : 10; // ahrs.set_alt_measurement_noise (system.cpp:134-141)
      const y = this.baroAlt - E.z, S = E.P[0] + Rn * Rn;
      const gate = Math.max(1, (P.EK3_HGT_I_GATE ?? 500) / 100);
      const ratio = (y * y) / (S * gate * gate);
      this.ekfVar.hgt = Math.sqrt(ratio);
      const timedOut = ratio > 1 && this.hgtRejectStart >= 0 && this.t - this.hgtRejectStart > 5;
      if (ratio <= 1 || timedOut) {
        if (timedOut) { E.z = this.baroAlt; E.P[0] = Rn * Rn; } // height reset after 5 s of rejection
        else {
          const K = [E.P[0] / S, E.P[3] / S, E.P[6] / S];
          E.z += K[0] * y; E.vz += K[1] * y; E.ab += K[2] * y;
          const p0 = [E.P[0], E.P[1], E.P[2]];
          for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) E.P[3 * i + j] -= K[i] * p0[j];
        }
        this.hgtRejectStart = -1;
      } else if (this.hgtRejectStart < 0) this.hgtRejectStart = this.t;
      E.ab = clamp(E.ab, -(P.EK3_ACC_BIAS_LIM ?? 1), P.EK3_ACC_BIAS_LIM ?? 1);
    }
    // ---- output predictor at the current time (AP_NavEKF3 calcOutputStates) ----
    const ac = azUp - E.ab;
    E.oz += E.ov * dt + 0.5 * ac * dt * dt;
    E.ov += ac * dt;
    const tau = clamp((P.EK3_TAU_OUTPUT ?? 25) * 0.01, dt, 10);
    const g = dt / tau;
    const ez = E.z - del.oz, ev = E.vz - del.ov;
    E.ozInt += ez; E.ovInt += ev;
    const cz = ez * g + E.ozInt * g * g * 0.1, cv = ev * g + E.ovInt * g * g * 0.1;
    E.oz += cz; E.ov += cv;
    for (const h of E.hist) { h.oz += cz; h.ov += cv; }
    this.ekfVar.pos = Math.sqrt(Math.max(0, E.P[0]));
  }

  // ---- AHRS-style accessors ----
  get roll() { return this.euler[0]; }
  get pitch() { return this.euler[1]; }
  get yaw() { return this.euler[2]; }
  get yawSensor() { return wrap360cd(Math.trunc((this.euler[2] * 18000) / Math.PI)); }
  get rollSensor() { return Math.trunc((this.euler[0] * 18000) / Math.PI); }
  get pitchSensor() { return Math.trunc((this.euler[1] * 18000) / Math.PI); }
  get cosRoll() { return Math.cos(this.euler[0]); }
  get cosPitch() { return Math.cos(this.euler[1]); }
  get cosYaw() { return Math.cos(this.euler[2]); }
  get sinYaw() { return Math.sin(this.euler[2]); }
  // inertial nav (cm, NEU)
  get positionZUpCm() { return this.ekfZ ? this.ekfZ.oz * 100 : 0; }
  get velocityZUpCms() { return this.ekfZ ? this.ekfZ.ov * 100 : 0; }
  get zAccelCmss() { return -(this.accelEf[2] + G) * 100; }
  get positionXYCm() { return this.inav ? [this.inav.x * 100, this.inav.y * 100] : [0, 0]; }
  get velocityXYCms() { return this.inav ? [this.inav.vx * 100, this.inav.vy * 100] : [0, 0]; }
  // AP_Baro::get_altitude() of the primary (depth) sensor, m down
  get depthMeasured() { return this.baroAlt === null ? 0 : -this.baroAlt; }
  // horizontal absolute position from the simulated DVL/USBL (sub.js adds failsafe.ekf)
  positionOk() { return !!this.cfg.positioning; }
  get positionSourceWarning() {
    const P = this.params;
    if (!this.cfg.positioning) return '';
    const src = P.EK3_SRC1_POSXY ?? 3;
    return (src === 6 && (P.VISO_TYPE ?? 0) > 0) || src === 4 ? '' : 'Simulated DVL/USBL positioning is on, but EK3_SRC1_POSXY/VISO_TYPE do not configure an external position source';
  }
  get ekfVariances() { return this.ekfVar; }
}

// 3x3 helpers (row-major)
function mul3(A, B) {
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] = A[3 * i] * B[j] + A[3 * i + 1] * B[3 + j] + A[3 * i + 2] * B[6 + j];
  return C;
}
function mul3T(A, B) { // A * B^T
  const C = new Array(9);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) C[3 * i + j] = A[3 * i] * B[3 * j] + A[3 * i + 1] * B[3 * j + 1] + A[3 * i + 2] * B[3 * j + 2];
  return C;
}
