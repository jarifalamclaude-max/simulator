// Pool geometry (any size, sloped floors), hydrostatic pressure with
// compressible water, the pressure-based depth sensor, pressure housings
// (compression, flooding) and ArduSub's leak / internal-pressure failsafes.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { box } from '../src/geometry/primitives.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { sampleGeometry } from '../src/vehicle/sample.js';
import { defaultConfig, thrusterFromPreset, applyParamFile, normalizeConfig, allParamsForExport } from '../src/vehicle/config.js';
import { parseParamFile } from '../src/ardusub/params.js';
import { buildPlantModel } from '../src/vehicle/model.js';
import { Environment, floorProfile } from '../src/physics/environment.js';
import { VehiclePlant } from '../src/physics/body.js';
import { RNG } from '../src/core/rng.js';
import { qToMat } from '../src/core/math.js';
import { Simulation, MODE } from '../src/sim/simulation.js';
import { eos80Density, eos80SecantBulkModulus, waterDensity } from '../src/hydro/water.js';

const rel = (a, b) => Math.abs(a - b) / Math.abs(b);

test('EOS-80 reproduces the UNESCO check values', () => {
  // Fofonoff & Millard (1983): secant bulk modulus [bar] and density [kg/m^3] at (S, T68, p bar)
  const K = [[0, 5, 0, 20337.80375], [35, 5, 0, 22185.93358], [35, 5, 1000, 25577.49819], [0, 25, 0, 22100.72106], [35, 25, 0, 23726.34949], [35, 25, 1000, 27108.94504]];
  for (const [S, T, p, ref] of K) assert.ok(rel(eos80SecantBulkModulus(S, T, p), ref) < 1e-8, `K(${S},${T},${p})`);
  const R = [[0, 5, 0, 999.96675], [0, 25, 0, 997.04796], [35, 5, 0, 1027.67547], [35, 25, 0, 1023.34306], [0, 5, 1000, 1044.12802], [0, 25, 1000, 1037.90204], [35, 5, 1000, 1069.48914], [35, 25, 1000, 1062.53817]];
  for (const [S, T, p, ref] of R) assert.ok(Math.abs(eos80Density(S, T, p) - ref) < 1e-4, `rho(${S},${T},${p})`);
  // salt-water surface density now follows EOS-80 (fresh part from Tanaka 2001)
  assert.ok(Math.abs(waterDensity(25, 35) - 1023.34) < 0.01);
});

test('hydrostatic pressure: rho g h near the surface, compressibility at depth', () => {
  const env = new Environment({ temperature: 10, salinity: 35, pool: { length: 100, width: 100, depth: 6000 } }, new RNG(1));
  const g = env.g, rho0 = env.rho;
  assert.ok(rel(env.pressureGauge(10), rho0 * g * 10) < 3e-5, 'shallow water is incompressible to 3e-5');
  // independent fine trapezoid integration of dp/dz = rho(p) g
  let p = 0;
  const n = 60000, dz = 4000 / n;
  for (let i = 0; i < n; i++) { const a = g * env.densityAtGauge(p); const b = g * env.densityAtGauge(p + a * dz); p += 0.5 * (a + b) * dz; }
  assert.ok(rel(env.pressureGauge(4000), p) < 1e-6, `${env.pressureGauge(4000)} vs ${p}`);
  // water at 4000 m is ~1.7 % denser, so the column weighs ~0.85 % more than rho0 g h
  const extra = env.pressureGauge(4000) / (rho0 * g * 4000) - 1;
  assert.ok(extra > 0.007 && extra < 0.010, `extra ${extra}`);
  assert.ok(env.densityAtDepth(4000) > rho0 * 1.015 && env.densityAtDepth(4000) < rho0 * 1.02);
  // absolute pressure includes the air
  assert.equal(env.absolutePressureAt(0, 0, -1), env.pAtm);
  assert.ok(rel(env.absolutePressureAt(0, 0, 10) - env.pAtm, env.pressureGauge(10)) < 1e-12);
});

test('sloped floor profile', () => {
  const pool = { length: 20, width: 10, depth: 3, floor: 'sloped', shallowDepth: 1, shallowEnd: 'south', slopeStart: 5, slopeEnd: 15 };
  const f = floorProfile(pool);
  assert.equal(f.depthAt(-10), 1); // south wall: shallow
  assert.equal(f.depthAt(10), 3); // north wall: deep
  assert.ok(Math.abs(f.depthAt(0) - 2) < 1e-12); // middle of the slope
  assert.ok(Math.abs(f.slopeAt(0) - 0.2) < 1e-12 && f.slopeAt(-8) === 0 && f.slopeAt(8) === 0);
  const g = floorProfile({ ...pool, shallowEnd: 'north' });
  assert.equal(g.depthAt(10), 1);
  assert.ok(g.slopeAt(0) < 0);
  // a near-vertical step is widened to 60 degrees
  const s = floorProfile({ ...pool, slopeStart: 10, slopeEnd: 10 });
  assert.ok(s.b - s.a >= 2 * Math.tan(Math.PI / 6) - 1e-9);
  assert.equal(floorProfile({ ...pool, floor: 'flat' }).depthAt(-10), 3);
});

const boxGeo = analyzeGeometry(new Float32Array(box([], [0, 0, 0], [0.4, 0.3, 0.2])), { targetCells: 40 });
function boxPlant(envCfg, mods = {}) {
  const cfg = defaultConfig();
  cfg.model.source = 'test';
  const env = new Environment({ ...cfg.environment, ...envCfg }, new RNG(1));
  cfg.mass.mass = (mods.massRatio ?? 1) * env.rho * boxGeo.volume;
  cfg.mass.cgOffset = [0, 0, 0.03];
  cfg.hydro.damping.mode = 'manual';
  cfg.hydro.damping.linear = [5, 5, 5, 0.05, 0.05, 0.05];
  cfg.hydro.damping.quadratic = [30, 30, 30, 0.2, 0.2, 0.2];
  cfg.thrusters = [thrusterFromPreset({ name: 'aft', pos: [-0.2, 0, 0.03], axis: [1, 0, 0] })];
  cfg.thrusters.forEach((t) => (t.deductionMode = 'manual'));
  Object.assign(cfg.hull, mods.hull || {});
  const { plant } = buildPlantModel(cfg, boxGeo, env);
  return { sim: new VehiclePlant(plant, env), env, plant };
}

test('heavy vehicle slides down a slick sloped floor and rests on it', () => {
  const pool = { length: 12, width: 6, depth: 3, floor: 'sloped', shallowDepth: 1, shallowEnd: 'south', slopeStart: 2, slopeEnd: 10 };
  const { sim, env, plant } = boxPlant({ pool, contact: { stiffness: 30000, damping: 900, friction: 0.05 } }, { massRatio: 1.4 });
  sim.reset({ position: [-3, 0, env.floorDepth(-3) - 0.25] });
  for (let i = 0; i < 4 * 800; i++) sim.step(1 / 800);
  const x = sim.position[0];
  assert.ok(x > -2.5, `slid downhill to x=${x}`);
  // every contact point is on or just above the local floor: no sinking into the slope
  const R = qToMat(sim.quaternion), p = sim.position;
  let worst = -Infinity;
  for (const c of plant.contacts) {
    const wx = p[0] + R[0] * c[0] + R[1] * c[1] + R[2] * c[2];
    const wz = p[2] + R[6] * c[0] + R[7] * c[1] + R[8] * c[2];
    worst = Math.max(worst, wz - env.floorDepth(wx));
  }
  assert.ok(worst < 0.01 && worst > -0.02, `deepest contact ${worst} m relative to the floor`);
  // and the hull lies along the slope (pitched nose-down toward the deep end, about atan(0.25))
  const pitch = Math.asin(-R[6]);
  assert.ok(Math.abs(pitch + Math.atan(0.25)) < 0.05, `pitch ${pitch}`);
});

test('buoyancy follows water density and hull compression at depth', () => {
  const deep = { length: 50, width: 50, depth: 600 };
  const rigid = boxPlant({ temperature: 10, salinity: 35, pool: deep });
  rigid.sim.reset({ position: [0, 0, 500] });
  rigid.sim.step(1e-4);
  const pg = rigid.env.pressureGauge(500 - 0.03);
  assert.ok(Math.abs(rigid.sim.diag.buoyancyScale - rigid.env.densityAtGauge(rigid.sim.diag.pressureGauge) / rigid.env.rho) < 1e-12);
  assert.ok(rigid.sim.diag.buoyancyScale > 1.002, 'denser water at 500 m lifts a rigid hull');
  const soft = boxPlant({ temperature: 10, salinity: 35, pool: deep }, { hull: { compressibility: 1, depthRating: 0 } });
  soft.sim.reset({ position: [0, 0, 500] });
  soft.sim.step(1e-4);
  const expect = (soft.env.densityAtGauge(soft.sim.diag.pressureGauge) / soft.env.rho) * (1 - (0.01 / 1e6) * soft.sim.diag.pressureGauge);
  assert.ok(Math.abs(soft.sim.diag.buoyancyScale - expect) < 1e-12 && soft.sim.diag.buoyancyScale < 0.997, `scale ${soft.sim.diag.buoyancyScale} pg ${pg}`);
});

test('housing floods below its rating until the trapped air reaches outside pressure', () => {
  const { sim, env, plant } = boxPlant({ pool: { length: 20, width: 20, depth: 40 } }, { hull: { depthRating: 10, airVolume: 2, leakArea: 4 } });
  sim.reset({ position: [0, 0, 5] });
  for (let i = 0; i < 800; i++) sim.step(1 / 800);
  assert.equal(sim.flood.volume, 0, 'dry above the rating');
  sim.reset({ position: [0, 0, 30] });
  sim.y[2] = 30;
  const hold = () => { sim.y[2] = 30; for (let k = 7; k < 13; k++) sim.y[k] = 0; };
  for (let i = 0; i < 400 * 60; i++) { sim.step(1 / 400); hold(); }
  const f = sim.flood;
  assert.ok(f.active && f.cause === 'rating');
  // Boyle: p_in = p0 Va / (Va - V) -> V_eq = Va (1 - p0 / p_out)
  const pOut = env.pAtm + env.pressureGauge(30);
  const Veq = plant.hull.airVolume * (1 - f.p0 / pOut);
  assert.ok(Math.abs(f.volume - Veq) / Veq < 0.02, `${f.volume} vs ${Veq}`);
  assert.ok(Math.abs(f.pIn - pOut) / pOut < 0.02);
});

const geo = analyzeGeometry(sampleGeometry().positions);
const duburi = parseParamFile(readFileSync(new URL('../profiles/duburi_pixhawk_param.param', import.meta.url), 'utf8')).params;
function makeSim(mod = () => {}) {
  const cfg = defaultConfig();
  applyParamFile(cfg, duburi, { syncPolarity: true });
  cfg.mixer.source = 'frame';
  cfg.mass.mass = waterDensity(cfg.environment.temperature) * geo.volume * 0.985;
  cfg.sensors.depthNoise = 0;
  mod(cfg);
  return new Simulation(cfg, geo);
}

test('depth sensor: AP_Baro water formula, specific gravity and range limit', () => {
  const fresh = makeSim((c) => { c.start = { position: [0, 0, 3], yaw: 0 }; c.params.BARO_SPEC_GRAV = 1.0; });
  fresh.runFor(1);
  const d = fresh.plant.position[2];
  const expect = fresh.env.pressureGauge(d) / 9800;
  // AP_Baro publishes the 10 Hz mean of the MS5837 conversions (~70 ms behind the vehicle)
  assert.ok(Math.abs(fresh.sensors.depthMeasured - expect) < 0.01, `${fresh.sensors.depthMeasured} vs ${expect}`);
  assert.ok(Math.abs(fresh.sensors.depthMeasured - (fresh.sensors.pressureMeas - fresh.sensors.groundPressure) / 9800) < 1e-9, 'water formula');
  // sea water read with SPEC_GRAV 1.0 shows ~2.6 % too deep (1025 x 9.807 / 9800)
  const sea = makeSim((c) => { c.environment.salinity = 35; c.environment.temperature = 15; c.start = { position: [0, 0, 3], yaw: 0 }; c.params.BARO_SPEC_GRAV = 1.0; });
  sea.runFor(1);
  // against the depth of the pressure the sensor published (the vehicle moves during the 10 Hz average)
  const ratio = sea.sensors.depthMeasured / ((sea.sensors.pressureMeas - sea.sensors.groundPressure) / (sea.env.rho * sea.env.g));
  assert.ok(ratio > 1.02 && ratio < 1.03, `ratio ${ratio}`);
  // a Bar02 (2 bar absolute) stops reading at about 10 m
  const bar02 = makeSim((c) => { c.sensors.depthSensor = 'bar02'; c.environment.pool = { length: 30, width: 30, depth: 40 }; c.start = { position: [0, 0, 20], yaw: 0 }; });
  bar02.runFor(1);
  assert.ok(bar02.sensors.depthOverRange);
  assert.ok(Math.abs(bar02.sensors.depthMeasured - (2e5 - bar02.sensors.groundPressure) / 9800) < 0.01, `${bar02.sensors.depthMeasured}`);
});

test('ArduSub leak failsafe: warn after 20 s, SURFACE mode with FS_LEAK_ENABLE 2', () => {
  // the DUBURI file has no leak detector (LEAK1_PIN -1): fit one for this test
  // SURFACE requires a position estimate in 4.5.7 (mode.h:453): give the vehicle a DVL/USBL
  const sim = makeSim((c) => { c.params.FS_LEAK_ENABLE = 2; c.hull.leakSensor = true; c.hull.leakSensorSource = 'user'; c.sensors.positioning = true; c.sensors.positioningSource = 'user'; c.start = { position: [0, 0, 2], yaw: 0 }; });
  sim.fw.setMode(MODE.ALT_HOLD);
  sim.runFor(0.5);
  assert.ok(sim.fw.arm());
  sim.runFor(1);
  sim.plant.startLeak('test');
  sim.runFor(2);
  assert.ok(sim.sensors.leak, 'leak detector wet');
  assert.equal(sim.fw.mode, MODE.SURFACE);
  assert.ok(sim.fw.status().leak);
  sim.runFor(20);
  assert.ok(sim.fw.messages.some((m) => m.text === 'Leak Detected' && m.severity === 2));
});

test('ArduSub internal-pressure failsafe trips 2 s after the enclosure pressure passes FS_PRESS_MAX', () => {
  const sim = makeSim((c) => { c.params.FS_PRESS_ENABLE = 1; c.params.FS_PRESS_MAX = 105000; c.hull.leakArea = 20; c.start = { position: [0, 0, 3.5], yaw: 0 }; });
  sim.plant.startLeak('test');
  let t = 0;
  while (sim.sensors.internalPressure < 105000 && t < 60) { sim.runFor(0.1); t += 0.1; }
  assert.ok(sim.sensors.internalPressure >= 105000, `internal ${sim.sensors.internalPressure}`);
  assert.ok(!sim.fw.status().internalPressureFs);
  sim.runFor(2.5);
  assert.ok(sim.fw.status().internalPressureFs);
});

test('shrinking the pool keeps the vehicle and its start position inside', () => {
  const sim = makeSim((c) => { c.start = { position: [10, 4, 3], yaw: 0 }; });
  sim.runFor(0.2);
  sim.cfg.environment.pool = { ...sim.cfg.environment.pool, length: 4, width: 3, depth: 1.5 };
  sim.rebuild(sim.cfg, sim.geo, { keepState: true });
  const p = sim.plant.position, r = sim.plant.boundR;
  assert.ok(Math.abs(p[0]) <= 2 - r + 1e-9 && Math.abs(p[1]) <= 1.5 - r + 1e-9 && p[2] <= 1.5 - r + 1e-9, `${p}`);
  const k = sim.keepInPool(sim.cfg.start.position);
  assert.ok(k.moved && Math.abs(k.p[0]) <= 2 - r + 1e-9);
});

test('the leak detector follows LEAKn_PIN / LEAKn_TYPE from the parameter file', () => {
  const cfg = defaultConfig();
  applyParamFile(cfg, duburi, { syncPolarity: true });
  assert.equal(cfg.hull.leakSensor, false, 'DUBURI file: LEAK1_PIN -1, no detector');
  const sim = makeSim((c) => { c.start = { position: [0, 0, 2], yaw: 0 }; });
  sim.plant.startLeak('test');
  sim.runFor(2);
  assert.ok(sim.plant.flood.volume > 0 && !sim.sensors.leak, 'water comes in but nothing detects it');
  const wired = defaultConfig();
  applyParamFile(wired, { LEAK1_PIN: 27, LEAK1_TYPE: 0 });
  assert.equal(wired.hull.leakSensor, true);
});

test('values from the file survive when the simulator starts simulating a parameter', () => {
  // a vehicle saved before FS_LEAK_ENABLE was simulated: the file's 0 sat in extraParams
  const old = { schema: 'duburi.vehicle', params: { FRAME_CONFIG: 2 }, extraParams: { FS_LEAK_ENABLE: 0, COMPASS_OFS_X: 306.1 } };
  const a = normalizeConfig(JSON.parse(JSON.stringify(old)));
  assert.equal(a.params.FS_LEAK_ENABLE, 0);
  assert.ok(!('FS_LEAK_ENABLE' in a.extraParams) && a.params.COMPASS_OFS_X === 306.1); // every ArduSub param is now defined
  assert.equal(allParamsForExport(a).FS_LEAK_ENABLE, 0);
  // saved again after the update with the new default persisted: the file value still wins
  const b = normalizeConfig({ ...old, params: { FRAME_CONFIG: 2, FS_LEAK_ENABLE: 1 } });
  assert.equal(b.params.FS_LEAK_ENABLE, 0);
  // a value the user changed in the simulator wins over the stale copy
  const c = normalizeConfig({ ...old, params: { FRAME_CONFIG: 2, FS_LEAK_ENABLE: 2 } });
  assert.equal(c.params.FS_LEAK_ENABLE, 2);
  // every parameter of the 943-line DUBURI file is either simulated or kept, none lost
  const d = defaultConfig();
  applyParamFile(d, duburi);
  const exp = allParamsForExport(d);
  for (const [k, v] of Object.entries(duburi)) assert.equal(exp[k], v, k);
});
