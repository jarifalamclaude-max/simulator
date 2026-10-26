// ArduSub firmware framework: scheduler, failsafes, arming, mode entry rules,
// sensors, live parameter edits and reproducibility.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { sampleGeometry } from '../src/vehicle/sample.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { defaultConfig, applyParamFile } from '../src/vehicle/config.js';
import { parseParamFile } from '../src/ardusub/params.js';
import { Simulation, MODE } from '../src/sim/simulation.js';
import { SIM_MODES } from '../src/ardusub/sub.js';
import { waterDensity } from '../src/hydro/water.js';

const geo = analyzeGeometry(sampleGeometry().positions);
const duburi = parseParamFile(readFileSync(new URL('../profiles/duburi_pixhawk_param.param', import.meta.url), 'utf8')).params;

function makeSim(mod = () => {}, { file = true } = {}) {
  const cfg = defaultConfig();
  if (file) applyParamFile(cfg, duburi, { syncPolarity: true });
  cfg.mixer.source = 'frame';
  cfg.mass.mass = waterDensity(cfg.environment.temperature) * geo.volume * 0.99;
  cfg.start = { position: [0, 0, 2], yaw: 0 };
  mod(cfg);
  return new Simulation(cfg, geo);
}
const has = (sim, txt) => sim.fw.messages.some((m) => (txt instanceof RegExp ? txt.test(m.text) : m.text === txt));
const withPos = (c) => { c.sensors.positioning = true; c.sensors.positioningSource = 'user'; };

test('scheduler: ArduSub task rates from integer tick intervals (no drift)', () => {
  const sim = makeSim();
  let mc = 0;
  const h = sim.fw.handleManualControl.bind(sim.fw);
  sim.fw.handleManualControl = (m) => { mc++; return h(m); };
  sim.runFor(10);
  const r = sim.fw.taskRuns;
  assert.equal(r.fiftyHzLoop, 500);
  assert.equal(r.updateAltitude, 100);
  assert.equal(r.updateBattCompass, 100);
  assert.equal(r.readRangefinder, 200);
  assert.equal(r.threeHzLoop, Math.floor(4000 / 133)); // 400/3 -> 133 ticks, as AP_Scheduler
  assert.equal(r.oneHzLoop, 10);
  assert.equal(r.gcsUpdateReceive, 4000);
  assert.equal(mc, 500, 'MANUAL_CONTROL at exactly 50 Hz');
  assert.equal(sim.sensors.imuHz, 1000);
});

test('pilot-input failsafe: latched at boot, 50 Hz check, disarm after FS_PILOT_TIMEOUT', () => {
  const sim = makeSim((c) => { c.params.FS_PILOT_INPUT = 2; c.params.FS_GCS_ENABLE = 0; });
  assert.equal(sim.fw.failsafe.pilotInput, true, 'Sub.cpp:47');
  sim.runFor(0.5);
  assert.equal(sim.fw.failsafe.pilotInput, false);
  sim.fw.setMode(MODE.STABILIZE);
  assert.ok(sim.fw.arm());
  sim.setJoystickConnected(false);
  sim.runFor(2.9);
  assert.ok(sim.fw.armed);
  sim.runFor(0.2);
  assert.ok(!sim.fw.armed && has(sim, 'Lost manual control'));
  assert.ok(!has(sim, 'Manual control regained'));
});

test('GCS heartbeat failsafe: FS_GCS_ENABLE 2 disarms, 3 enters ALT_HOLD', () => {
  const sim = makeSim((c) => { c.params.FS_GCS_ENABLE = 2; c.params.FS_PILOT_INPUT = 0; });
  sim.fw.setMode(MODE.STABILIZE);
  sim.runFor(1.2);
  assert.ok(sim.fw.arm());
  sim.setGcsLink(false);
  sim.runFor(2.0);
  assert.ok(sim.fw.armed);
  sim.runFor(1.0);
  assert.ok(!sim.fw.armed && sim.fw.failsafe.gcs);
  // the 30 s warning timer starts at boot (failsafe.cpp:338): first text once millis() > 30 s
  sim.runFor(27);
  assert.ok(has(sim, 'MYGCS: 255, heartbeat lost'));
  sim.setGcsLink(true);
  sim.runFor(1);
  assert.equal(sim.fw.failsafe.gcs, false);
  const s2 = makeSim((c) => { c.params.FS_GCS_ENABLE = 3; c.params.FS_PILOT_INPUT = 0; });
  s2.runFor(1.2);
  s2.fw.setMode(MODE.STABILIZE);
  assert.ok(s2.fw.arm());
  s2.setGcsLink(false);
  s2.runFor(3.2);
  assert.ok(s2.fw.armed && s2.fw.mode === MODE.ALT_HOLD && s2.fw.failsafe.gcs);
  // a wrong SYSID_MYGCS: the GCS is never accepted (and pilot input is lost)
  const s3 = makeSim((c) => { c.params.SYSID_MYGCS = 200; c.params.FS_PILOT_INPUT = 1; });
  s3.runFor(3.5);
  assert.equal(s3.fw.gcsLastSeenMs, 0);
  assert.ok(s3.fw.failsafe.pilotInput);
});

test('crash check: attitude error > 30 deg for 2 s warns and disarms (FS_CRASH_CHECK 2)', () => {
  const sim = makeSim((c) => { c.params.FS_CRASH_CHECK = 2; });
  sim.runFor(20.5); // the 20 s warning interval counts from boot (failsafe.cpp:404)
  sim.fw.setMode(MODE.STABILIZE);
  assert.ok(sim.fw.arm());
  sim.fw.att.getAttErrorAngleDeg = () => 40;
  sim.runFor(1.9);
  assert.ok(sim.fw.armed);
  sim.runFor(0.3);
  assert.ok(!sim.fw.armed && has(sim, 'Crash detected'));
});

test('EKF failsafe: variance over FS_EKF_THRESH for 2 s, position_ok false', () => {
  const sim = makeSim((c) => { c.params.FS_EKF_ACTION = 2; withPos(c); });
  sim.runFor(20.5);
  sim.fw.setMode(MODE.STABILIZE);
  assert.ok(sim.fw.arm());
  assert.ok(sim.fw.positionOk());
  sim.sensors.faults.ekf = true;
  sim.runFor(2.2);
  assert.ok(!sim.fw.armed && has(sim, 'EKF bad') && !sim.fw.positionOk());
});

test('depth sensor failsafe and control_check_barometer', () => {
  const sim = makeSim();
  assert.ok(sim.fw.setMode(MODE.ALT_HOLD));
  assert.ok(sim.fw.arm());
  sim.runFor(1);
  sim.sensors.faults.depth = true;
  sim.runFor(1);
  assert.ok(has(sim, 'Depth sensor error!') && sim.fw.mode === MODE.MANUAL && sim.fw.failsafe.sensorHealth);
  assert.equal(sim.fw.setMode(MODE.ALT_HOLD), false);
  assert.ok(has(sim, 'Depth sensor error.') && has(sim, 'Mode change failed: ALT_HOLD need alt estimate'));
  // no external depth sensor at all
  const none = makeSim((c) => { c.sensors.depthSensor = 'none'; });
  assert.equal(none.fw.depthSensorPresent, false);
  assert.equal(none.fw.setMode(MODE.ALT_HOLD), false);
  assert.ok(has(none, 'Depth sensor is not connected.'));
  assert.ok(none.fw.setMode(MODE.MANUAL) && none.fw.arm());
  none.runFor(1);
  assert.ok(none.fw.armed, 'manual flight without a depth sensor');
});

test('mode entry rules follow mode.h (requires_GPS) and the init() checks', () => {
  for (const m of [MODE.SURFTRAK, MODE.GUIDED, MODE.AUTO, MODE.CIRCLE, MODE.MOTOR_DETECT]) assert.ok(SIM_MODES.includes(m));
  const sim = makeSim((c) => { c.sensors.positioning = false; c.sensors.positioningSource = 'user'; });
  const fw = sim.fw;
  for (const [m, name] of [[MODE.SURFACE, 'SURFACE'], [MODE.GUIDED, 'GUIDED'], [MODE.AUTO, 'AUTO'], [MODE.CIRCLE, 'CIRCLE']]) {
    assert.equal(fw.setMode(m), false);
    assert.ok(has(sim, `Mode change failed: ${name} requires position`));
  }
  assert.equal(fw.setMode(MODE.POSHOLD), false, 'POSHOLD fails in init, not as requires_GPS');
  assert.ok(has(sim, 'Flight mode change failed POSHOLD'));
  assert.ok(fw.setMode(MODE.SURFTRAK) && has(sim, 'waiting for a rangefinder reading'));
  assert.ok(fw.setMode(MODE.MOTOR_DETECT));
  // with a position source
  const p = makeSim(withPos);
  assert.ok(p.fw.setMode(MODE.SURFACE) && p.fw.setMode(MODE.GUIDED) && p.fw.setMode(MODE.CIRCLE) && p.fw.setMode(MODE.POSHOLD));
  assert.equal(p.fw.setMode(MODE.AUTO), false, 'AUTO needs a mission');
  assert.ok(p.fw.setMission([{ pos: [1, 0, 2] }]) && p.fw.setMode(MODE.AUTO));
  // the leak failsafe's SURFACE action fails without position, as in 4.5.7
  const leak = makeSim((c) => { c.params.FS_LEAK_ENABLE = 2; c.hull.leakSensor = true; c.hull.leakSensorSource = 'user'; c.sensors.positioning = false; c.sensors.positioningSource = 'user'; });
  leak.fw.setMode(MODE.ALT_HOLD);
  assert.ok(leak.fw.arm());
  leak.plant.startLeak('test');
  leak.runFor(2);
  assert.ok(leak.fw.failsafe.leak && leak.fw.mode === MODE.ALT_HOLD && has(leak, 'Mode change failed: SURFACE requires position'));
});

test('arming: AP_Arming_Sub rules (no ATC gain check, disarm button, RC calibration, ARMING_CHECK)', () => {
  const a = makeSim((c) => { c.params.ATC_RAT_YAW_I = 0; c.params.ATC_ANG_RLL_P = 0; });
  assert.ok(a.fw.arm(), 'zero gains are valid Sub ranges');
  assert.equal(a.fw.arm(), false, 'AP_Arming::arm returns false when already armed');
  const b = makeSim((c) => { for (let i = 0; i < 32; i++) { if ([2, 4].includes(c.params[`BTN${i}_FUNCTION`])) c.params[`BTN${i}_FUNCTION`] = 0; if ([2, 4].includes(c.params[`BTN${i}_SFUNCTION`])) c.params[`BTN${i}_SFUNCTION`] = 0; } });
  assert.equal(b.fw.arm(), false);
  assert.ok(has(b, 'Arm: Must assign a disarm or arm_toggle button'));
  const r = makeSim((c) => { c.params.RC2_MIN = 1400; c.params.ARMING_CHECK = 448; });
  assert.equal(r.fw.arm(), false);
  assert.ok(has(r, 'Arm: Roll radio min too high'));
  const off = makeSim((c) => { c.params.RC2_MIN = 1400; c.params.ARMING_CHECK = 0; });
  assert.ok(off.fw.arm() && has(off, 'Warning: Arming Checks Disabled'));
  assert.deepEqual(off.fw.armingCheckFailures(), []);
});

test('leak detector driver: LEAKn_LOGIC and the 3 s cooldown', () => {
  const sim = makeSim((c) => { c.params.LEAK1_PIN = 27; c.params.LEAK1_TYPE = 1; c.params.LEAK1_LOGIC = 0; c.hull.leakSensor = true; c.hull.leakSensorSource = 'user'; });
  sim.runFor(1);
  assert.equal(sim.sensors.leak, false);
  sim.plant.startLeak('test');
  sim.runFor(1);
  assert.ok(sim.sensors.leak);
  sim.plant.clearFlood();
  sim.runFor(2);
  assert.ok(sim.sensors.leak, 'still latched within LEAKDETECTOR_COOLDOWN_MS');
  sim.runFor(1.5);
  assert.equal(sim.sensors.leak, false);
  const inv = makeSim((c) => { c.params.LEAK1_PIN = 27; c.params.LEAK1_TYPE = 1; c.params.LEAK1_LOGIC = 1; c.hull.leakSensor = true; });
  inv.runFor(1);
  assert.ok(inv.sensors.leak, 'dry level high: a dry SOS probe reads as a leak');
});

test('internal temperature failsafe after 2 s over FS_TEMP_MAX, warning every 30 s', () => {
  const sim = makeSim((c) => { c.params.FS_TEMP_ENABLE = 1; c.params.FS_TEMP_MAX = 62; c.sensors.internalTempOverride = 70; });
  sim.runFor(1.5);
  assert.equal(sim.fw.failsafe.internalTemp, false);
  sim.runFor(1.5);
  assert.ok(sim.fw.failsafe.internalTemp);
  sim.runFor(30);
  assert.ok(has(sim, 'Internal temperature critical!'));
});

test('battery monitor and failsafe: BATT_LOW_VOLT with BATT_LOW_TIMER, action disarm', () => {
  const probe = makeSim((c) => { c.params.BATT_MONITOR = 4; });
  probe.runFor(0.5);
  const b = probe.sensors.battery;
  assert.ok(b.healthy && b.voltage > 10 && Number.isFinite(b.current), JSON.stringify(b));
  const v = b.voltage;
  const lo = makeSim((c) => { c.params.BATT_MONITOR = 4; c.params.BATT_ARM_VOLT = v + 5; });
  lo.runFor(0.5);
  assert.equal(lo.fw.arm(), false);
  assert.ok(has(lo, 'Arm: Battery 1 below minimum arming voltage'));
  const sim = makeSim((c) => { c.params.BATT_MONITOR = 4; c.params.BATT_FS_LOW_ACT = 2; c.params.BATT_LOW_TIMER = 10; c.params.ARMING_CHECK = 0; });
  sim.fw.setMode(MODE.STABILIZE);
  sim.runFor(0.5);
  assert.ok(sim.fw.arm());
  sim.fw.applyParams({ BATT_LOW_VOLT: v + 5 });
  sim.runFor(9.5);
  assert.ok(sim.fw.armed, 'BATT_LOW_TIMER not elapsed');
  sim.runFor(1.0);
  assert.ok(!sim.fw.armed && has(sim, /^Battery 1 is low/) && sim.fw.battFs.state === 1);
  const none = makeSim((c) => { c.params.BATT_MONITOR = 0; });
  none.runFor(0.3);
  assert.equal(none.sensors.battery.healthy, false);
});

test('terrain failsafe: AUTO waypoint above the bottom without a rangefinder disarms', () => {
  const sim = makeSim((c) => { withPos(c); c.params.RNGFND1_TYPE = 0; c.params.FS_TERRAIN_ENAB = 0; });
  assert.ok(sim.fw.setMission([{ pos: [1, 0, 1], terrain: true }]));
  assert.ok(sim.fw.setMode(MODE.AUTO) && sim.fw.arm());
  sim.runFor(0.5);
  assert.ok(has(sim, 'Terrain data (rangefinder) not available'));
  assert.ok(!sim.fw.armed);
});

test('AUTO flies the waypoint list; CIRCLE turns; GUIDED goes to a target', () => {
  const sim = makeSim((c) => { withPos(c); c.params.WPNAV_SPEED = 50; c.environment.pool = { ...c.environment.pool, length: 20, width: 20, depth: 5 }; });
  sim.fw.setMission([{ pos: [1.0, 0, 2.0], hold: 1 }, { pos: [1.0, 1.0, 2.5] }]);
  assert.ok(sim.fw.setMode(MODE.AUTO) && sim.fw.arm());
  sim.runFor(40);
  assert.ok(has(sim, 'Reached command #1') && has(sim, 'Reached command #2'), sim.fw.messages.map((m) => m.text).join('|'));
  const p = sim.plant.position;
  assert.ok(Math.hypot(p[0] - 1, p[1] - 1) < 0.4 && Math.abs(p[2] - 2.5) < 0.3, `${p}`);
  assert.equal(sim.fw.mission.state, 'complete');
  const g = makeSim((c) => { withPos(c); c.environment.pool = { ...c.environment.pool, length: 20, width: 20, depth: 5 }; });
  assert.ok(g.fw.setMode(MODE.GUIDED) && g.fw.arm());
  assert.ok(g.fw.setGuidedTarget([-1, 0.5, 1.5]));
  g.runFor(25);
  const q = g.plant.position;
  assert.ok(Math.hypot(q[0] + 1, q[1] - 0.5) < 0.3 && Math.abs(q[2] - 1.5) < 0.3, `${q}`);
  const c = makeSim((cc) => { withPos(cc); cc.params.CIRCLE_RADIUS = 100; cc.params.CIRCLE_RATE = 10; cc.environment.pool = { ...cc.environment.pool, length: 20, width: 20, depth: 5 }; });
  assert.ok(c.fw.setMode(MODE.CIRCLE) && c.fw.arm());
  c.runFor(15);
  assert.ok(c.fw.circleNav.angleTotal > 1, `${c.fw.circleNav.angleTotal}`);
});

test('SURFTRAK holds the range to the bottom with a rangefinder', () => {
  const sim = makeSim((c) => { c.params.RNGFND1_TYPE = 23; c.params.RNGFND1_ORIENT = 25; c.environment.pool = { ...c.environment.pool, depth: 4, floor: 'flat' }; c.start = { position: [0, 0, 2], yaw: 0 }; });
  sim.runFor(1);
  assert.ok(sim.sensors.rangefinder.healthy, JSON.stringify(sim.sensors.rangefinder));
  assert.ok(Math.abs(sim.sensors.rangefinder.distance_m - (4 - 2)) < 0.3, `${sim.sensors.rangefinder.distance_m}`);
  assert.ok(sim.fw.setMode(MODE.SURFTRAK) && sim.fw.arm());
  sim.runFor(3);
  assert.ok(sim.fw.surftrak.targetCm > 0 && has(sim, /^rangefinder target is/));
});

test('bottom detector counts cumulatively to 400 ticks (surface_bottom_detector.cpp)', () => {
  const sim = makeSim();
  const fw = sim.fw;
  fw.armed = true;
  fw.sensors = Object.create(fw.sensors, { velocityZUpCms: { value: 0 }, depthMeasured: { value: 2 } });
  let first = -1;
  for (let i = 0; i < 600 && first < 0; i++) { fw.motors.limit.throttle_lower = i % 4 !== 3; fw.updateSurfaceBottom(); if (fw.atBottom) first = i; }
  // 400 counted ticks out of 3-in-4 contact: bottomed near tick 533 (a consecutive counter never triggers)
  assert.ok(first > 520 && first < 545, `${first}`);
  fw.armed = false;
});

test('tether turn counter tracks quarter turns', () => {
  const sim = makeSim();
  const fw = sim.fw;
  let yaw = 0;
  fw.sensors = Object.create(fw.sensors, { yaw: { get: () => yaw } });
  for (let k = 0; k <= 400; k++) { yaw = Math.atan2(Math.sin(k * 0.05), Math.cos(k * 0.05)); fw.updateTurnCounter(); }
  assert.equal(fw.quarterTurnCount, Math.floor((400 * 0.05) / (Math.PI / 2)));
});

test('baro: ground pressure ratchet keeps depth >= 0 above the water; IMU rate from INS_GYRO_RATE', () => {
  const sim = makeSim((c) => { c.start = { position: [0, 0, 0], yaw: 0 }; c.mass.mass *= 0.7; });
  sim.runFor(4);
  assert.ok(sim.sensors.depthMeasured > -0.005, `${sim.sensors.depthMeasured}`);
  const fast = makeSim((c) => { c.params.INS_GYRO_RATE = 1; c.params.INS_FAST_SAMPLE = 1; });
  assert.equal(fast.sensors.imuHz, 2000);
});

test('live parameter edits and keep-state rebuilds keep the vehicle armed and its controller state', () => {
  const sim = makeSim((c) => { c.params.FS_PILOT_INPUT = 2; });
  sim.fw.setMode(MODE.ALT_HOLD);
  assert.ok(sim.fw.arm());
  sim.runFor(3);
  const fw = sim.fw, iBefore = fw.pos.pidAccelZ.integrator ?? fw.pos.pidAccelZ.info?.I;
  fw.applyParams({ ATC_RAT_RLL_P: 0.2, PSC_VELZ_P: 6 });
  assert.ok(fw.armed);
  sim.cfg.mass.mass *= 1.001;
  for (let k = 0; k < 8; k++) { sim.rebuild(sim.cfg, sim.geo, { keepState: true }); sim.runFor(0.0025 * (k + 1)); }
  assert.equal(sim.fw, fw, 'same firmware object');
  assert.ok(fw.pos.pidAccelZ.integrator === undefined || Math.abs(fw.pos.pidAccelZ.integrator - iBefore) < 50);
  sim.runFor(2);
  assert.ok(fw.armed && fw.mode === MODE.ALT_HOLD && !has(sim, 'Lost manual control'));
});

test('reset is reproducible from the seed', () => {
  const run = (sim, t) => { sim.fw.setMode(MODE.STABILIZE); sim.fw.arm(); sim.setPilotInput({ x: 300, z: 600 }); sim.runFor(t); return sim.plant.position.slice(); };
  const a = makeSim((c) => { c.environment.turbulence = { sigma: 0.05, tau: 1 }; });
  const pa = run(a, 4);
  const b = makeSim((c) => { c.environment.turbulence = { sigma: 0.05, tau: 1 }; });
  run(b, 2.5);
  b.setPilotInput({ x: 0, z: 500 });
  b.reset();
  const pb = run(b, 4);
  for (let k = 0; k < 3; k++) assert.ok(Math.abs(pa[k] - pb[k]) < 1e-9, `${pa} vs ${pb}`);
});
