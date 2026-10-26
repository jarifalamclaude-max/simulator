import test from 'node:test';
import assert from 'node:assert/strict';
import { box } from '../src/geometry/primitives.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { defaultConfig, thrusterFromPreset } from '../src/vehicle/config.js';
import { buildPlantModel, inertiaTensor } from '../src/vehicle/model.js';
import { Environment } from '../src/physics/environment.js';
import { VehiclePlant } from '../src/physics/body.js';
import { ThrusterState, parametricCurve, THRUSTER_PRESETS } from '../src/physics/thruster.js';
import { Battery, cellOcv } from '../src/physics/battery.js';
import { RNG } from '../src/core/rng.js';
import { qFromEuler, qToMat } from '../src/core/math.js';
import { shapeGeometry } from '../src/vehicle/shapes.js';
import { bbox } from '../src/geometry/mesh.js';

const boxGeo = (s) => analyzeGeometry(new Float32Array(box([], [0, 0, 0], s)), { targetCells: 32 });
const geo432 = boxGeo([0.4, 0.3, 0.2]);

function vehicle(geo, { env: envCfg = {}, mass, massRatio = 1, cgOffset = [0, 0, 0.03], linear = [5, 5, 5, 0.05, 0.05, 0.05], quadratic = [30, 30, 30, 0.2, 0.2, 0.2], thrusters = [], battery, mod } = {}) {
  const cfg = defaultConfig();
  cfg.model.source = 'test';
  const env = new Environment({ ...cfg.environment, ...envCfg }, new RNG(1));
  cfg.mass.mass = mass ?? massRatio * env.rho * geo.volume;
  cfg.mass.cgOffset = cgOffset;
  cfg.hydro.damping.mode = 'manual';
  cfg.hydro.damping.linear = linear;
  cfg.hydro.damping.quadratic = quadratic;
  cfg.thrusters = thrusters;
  cfg.thrusters.forEach((t) => (t.deductionMode = 'manual'));
  if (battery) cfg.battery = battery;
  if (mod) mod(cfg);
  const { plant, info } = buildPlantModel(cfg, geo, env);
  return { sim: new VehiclePlant(plant, env), plant, info, env, cfg };
}

const T200 = parametricCurve(THRUSTER_PRESETS['t200-16v'].curve);

test('polarity: a reversed unit mirrors the curve about 1500 us (paired motors give equal thrust)', () => {
  const fwd = new ThrusterState({ table: T200, polarity: 1, tauUp: 0, tauDown: 0 });
  const rev = new ThrusterState({ table: T200, polarity: -1, tauUp: 0, tauDown: 0 });
  for (let out = -1; out <= 1.0001; out += 0.1) {
    // ArduSub: pwm = 1500 + 400 * out * MOT_n_DIRECTION
    fwd.update(1500 + 400 * out, 0.01); rev.update(1500 - 400 * out, 0.01);
    assert.ok(Math.abs(fwd.T - rev.T) < 1e-9, `out ${out}: ${fwd.T} vs ${rev.T}`);
  }
  rev.update(1100, 0.01);
  assert.ok(Math.abs(rev.T - 51.5) < 1e-6, 'positive output on a reversed motor reaches the strong side');
  // a mirrored pair on the plant: no spurious yaw, equal applied thrust
  const S = Math.SQRT1_2;
  const ths = [thrusterFromPreset({ name: 'R', pos: [-0.15, 0.1, 0.03], axis: [S, -S, 0] }), thrusterFromPreset({ name: 'L', pos: [-0.15, -0.1, 0.03], axis: [S, S, 0], polarity: -1 })];
  const { sim } = vehicle(geo432, { thrusters: ths, env: { pool: { length: 60, width: 60, depth: 10 } } });
  sim.reset({ position: [0, 0, 3] });
  // (short run: left alone the box's Munk yaw instability eventually breaks the symmetry)
  for (let i = 0; i < 400; i++) sim.step(1 / 800, [1800, 1200]);
  assert.equal(sim.thrusters[0].T, sim.thrusters[1].T);
  assert.ok(Math.abs(sim.thrusters[0].Teff - sim.thrusters[1].Teff) < 0.005 * Math.abs(sim.thrusters[0].Teff), `${sim.thrusters[0].Teff} vs ${sim.thrusters[1].Teff}`);
  assert.ok(Math.abs(sim.rates[2]) < 0.01, `yaw rate ${sim.rates[2]}`);
});

test('thrust curve is exactly zero inside the ESC deadband', () => {
  const t = new ThrusterState({ table: T200, tauUp: 0, tauDown: 0 });
  for (const p of [1475, 1490, 1500, 1515, 1525]) { t.update(p, 0.01); assert.equal(t.T, 0, `pwm ${p}`); }
  t.update(1530, 0.01); assert.ok(t.T > 0);
});

test('rotor dynamics: thrust ~ n|n| (zero initial slope), braking constant through a reversal, dwell at zero', () => {
  const t = new ThrusterState({ table: T200, tauUp: 0.05, tauDown: 0.075, reverseDwell: 0.03 });
  t.update(1900, 0.005);
  assert.ok(t.T < 0.02 * 51.5, `first step only ${t.T}`); // first-order on T would give ~10 %
  for (let i = 0; i < 200; i++) t.update(1900, 0.005);
  assert.ok(Math.abs(t.T - 51.5) < 0.1);
  // reversal: decelerates with tauDown, parks at zero for the dwell, then spins up the other way
  let tZero = null, tNeg = null;
  for (let i = 1; i <= 200; i++) {
    t.update(1100, 0.001);
    if (tZero === null && t.T === 0) tZero = i;
    if (tNeg === null && t.T < 0) tNeg = i;
  }
  const n0 = 1, nCmd = -Math.sqrt(1); // normalised speeds
  const expectZero = 0.075 * Math.log((n0 - nCmd) / (0 - nCmd)) * 1000; // ms to cross zero with tauDown
  assert.ok(Math.abs(tZero - expectZero) <= 2, `zero crossing at ${tZero} ms, expected ${expectZero.toFixed(1)}`);
  assert.ok(tNeg - tZero >= 29 && tNeg - tZero <= 32, `dwell ${tNeg - tZero} ms`);
});

test('thrust and current follow the bus voltage (T200 tables)', () => {
  const def = { ...thrusterFromPreset({}, 't200-16v'), table: T200, tauUp: 0, tauDown: 0 };
  const t = new ThrusterState(def);
  t.update(1900, 0.01, 16); const T16 = t.T, I16 = t.current;
  t.update(1900, 0.01, 12); const T12 = t.T;
  t.update(1900, 0.01, 20); const T20 = t.T, I20 = t.current;
  assert.ok(Math.abs(T16 - 51.5) < 0.1 && Math.abs(T12 / T16 - 3.71 / 5.25) < 0.01 && Math.abs(T20 / T16 - 6.7 / 5.25) < 0.01, `${T12} ${T16} ${T20}`);
  assert.ok(Math.abs(I16 - 24) < 0.5 && Math.abs(I20 - 32) < 0.5, `${I16} ${I20}`);
  t.update(1700, 0.01, 16);
  assert.ok(t.current > 4 && t.current < 10, `half-ish throttle current ${t.current}`); // BR data: ~2.4 kgf at ~7 A
  t.update(1900, 0.01, 5); assert.equal(t.T, 0, 'ESC browns out below its minimum voltage');
});

test('battery: OCV curve, ohmic + polarisation sag, recovery, coulomb counting', () => {
  for (let s = 0.05; s <= 1; s += 0.05) assert.ok(cellOcv(s) > cellOcv(s - 0.05));
  const b = new Battery();
  assert.ok(Math.abs(b.voltage - 16.8) < 1e-9 && b.remainingPct === 100);
  b.step(0.001, 50);
  assert.ok(Math.abs(b.restingVoltage - b.voltage - 50 * 0.03) < 0.01, `instant sag ${b.restingVoltage - b.voltage}`);
  const v0 = b.voltage;
  for (let i = 0; i < 600; i++) b.step(0.1, 50);
  assert.ok(b.voltage < v0 - 0.4, 'more sag under sustained load');
  assert.ok(Math.abs(b.consumedMah - (50 * 60.001) / 3.6) < 1, `${b.consumedMah} mAh`);
  const loaded = b.voltage;
  b.step(0.1, 0); const off = b.voltage;
  for (let i = 0; i < 1200; i++) b.step(0.1, 0);
  assert.ok(off > loaded + 1.4 && b.voltage > off + 0.2 && Math.abs(b.voltage - b.restingVoltage) < 0.05, 'recovers once the load drops');
  const e = new Battery({ cells: 4, capacityMah: 1000, chemistry: 'lipo' });
  for (let i = 0; i < 3600; i++) e.step(1, 1);
  assert.ok(e.remainingPct < 0.01 && e.restingVoltage < 13.2, `${e.remainingPct} ${e.restingVoltage}`);
});

test('plant draws thruster + hotel current from the battery, and a sagging pack gives less thrust', () => {
  const mk = (soc0) => vehicle(geo432, { thrusters: [thrusterFromPreset({ name: 'a', pos: [-0.2, 0, 0.03], axis: [1, 0, 0] }), thrusterFromPreset({ name: 'b', pos: [0, 0, 0.03], axis: [0, 0, 1] })], battery: { soc0 }, env: { pool: { length: 100, width: 60, depth: 10 } } }).sim;
  const full = mk(1), low = mk(0.1);
  for (const s of [full, low]) { s.reset({ position: [-40, 0, 3] }); for (let i = 0; i < 800; i++) s.step(1 / 800, [1900, 1900]); }
  const sum = full.thrusters.reduce((a, t) => a + t.current, 0) + 2;
  assert.ok(Math.abs(full.battery.current - sum) < 1e-9 && sum > 40, `${full.battery.current} vs ${sum}`);
  assert.ok(full.battery.voltage < full.battery.restingVoltage - 1, 'bus sags under ~50 A');
  assert.ok(low.thrusters[0].T < 0.85 * full.thrusters[0].T, `${low.thrusters[0].T} vs ${full.thrusters[0].T}`);
});

test('energy is conserved with damping off (RK4, full 6x6 added mass, CG below CB)', () => {
  const pool = { length: 80, width: 80, depth: 60 };
  const env0 = new Environment({ temperature: 10, pool }, new RNG(1));
  const z0 = 30, pg = env0.pressureGauge(z0);
  const { sim } = vehicle(geo432, { env: { temperature: 10, pool, surfaceDrag: 0 }, mass: (env0.densityAtGauge(pg) / env0.rho) * env0.rho * geo432.volume, linear: [0, 0, 0, 0, 0, 0], quadratic: [0, 0, 0, 0, 0, 0] });
  sim.reset({ position: [0, 0, z0], quaternion: qFromEuler(0.3, -0.2, 0.5), velocity: [0.4, -0.2, 0.05], rates: [0.6, -0.4, 0.9] });
  // buoyancy grows slightly with depth (compressible water): potential = integral of B dz
  let zb0, B0;
  const E = () => {
    const y = sim.y, M = sim.M, nu = [y[7], y[8], y[9], y[10], y[11], y[12]];
    let K = 0;
    for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) K += 0.5 * nu[i] * M[i][j] * nu[j];
    const R = qToMat(sim.quaternion), c = sim.m.cb, B = sim.diag.buoyancyMag;
    const zb = y[2] + R[6] * c[0] + R[7] * c[1] + R[8] * c[2];
    if (zb0 === undefined) { zb0 = zb; B0 = B; }
    return K - sim.m.mass * sim.env.g * y[2] + B0 * zb0 + 0.5 * (B + B0) * (zb - zb0);
  };
  sim.step(1 / 800);
  const E0 = E();
  let worst = 0;
  for (let i = 0; i < 8000; i++) { sim.step(1 / 800); worst = Math.max(worst, Math.abs(E() - E0)); }
  const K0 = E0 + sim.m.mass * sim.env.g * z0 - B0 * zb0;
  assert.ok(worst < 1e-4 * Math.max(1, K0) + 1e-3, `energy drift ${worst} J (energy ${K0} J)`);
});

for (const [name, size, mass] of [['2 kg', [0.15, 0.12, 0.08], 2], ['30 kg', [0.4, 0.3, 0.2], 30]]) {
  test(`contact is stable for a ${name} vehicle at 800 and 400 Hz`, () => {
    const geo = boxGeo(size);
    for (const hz of [800, 400]) {
      const { sim, env } = vehicle(geo, { mass, cgOffset: [0, 0, 0.01], env: { pool: { length: 10, width: 10, depth: 3 } } });
      const D = env.floorDepth(0);
      sim.reset({ position: [0, 0, D - size[2] / 2 - 0.05], quaternion: qFromEuler(0.1, -0.08, 0.2) });
      let maxRate = 0;
      for (let i = 0; i < 6 * hz; i++) { sim.step(1 / hz); maxRate = Math.max(maxRate, Math.hypot(...sim.rates)); }
      const p = sim.position;
      assert.ok(Math.hypot(...sim.rates) < 0.01 && Math.hypot(...sim.velocityBody) < 0.01, `${hz} Hz: still moving`);
      assert.ok(maxRate < 5, `${hz} Hz: max rate ${maxRate}`);
      assert.ok(Math.abs(p[2] - (D - size[2] / 2 + 0.01 * 0)) < 0.012, `${hz} Hz: rests at ${p[2]} (floor ${D})`);
    }
  });
}

test('wave Froude-Krylov + added-mass force is (rho V + A11) a_fluid, and a neutral body follows the particle', () => {
  const waves = { amplitude: 0.2, period: 2, direction: 0 };
  const pool = { length: 300, width: 300, depth: 20 };
  const { sim, env, plant } = vehicle(geo432, { env: { waves, pool }, cgOffset: [0, 0, 0] });
  sim.reset({ position: [0, 0, 1.5] });
  env.time = 0.5; // quarter period: horizontal acceleration at its peak
  const out = new Float64Array(13);
  sim.deriv(sim.y, out, true);
  const a = env.fluidAccel(0, 0, 1.5);
  const want = (env.rho * plant.volume * sim.diag.buoyancyScale + sim.MAe[0][0]) * a[0] + sim.MAe[0][2] * a[2];
  assert.ok(Math.abs(a[0]) > 0.1 && Math.abs(sim.diag.fluidInertia[0] - want) < 1e-6 * Math.abs(want), `${sim.diag.fluidInertia[0]} vs ${want}`);
  // Morison: m = rho V, so the body moves with the water
  env.time = 0;
  // compare the surge velocity amplitude (insensitive to slow drift) with the particle's
  let vmax = 0, vmin = 0;
  for (let i = 0; i < 800 * 14; i++) {
    env.step(1 / 800); sim.step(1 / 800);
    if (i > 800 * 8) { const R = qToMat(sim.quaternion), v = sim.velocityBody, vx = R[0] * v[0] + R[1] * v[1] + R[2] * v[2]; vmax = Math.max(vmax, vx); vmin = Math.min(vmin, vx); }
  }
  const pr = env.waveProfile(0, sim.position[2]), up = 0.2 * env.waveOmega * pr.fu;
  assert.ok(Math.abs((vmax - vmin) / 2 / up - 1) < 0.1, `surge velocity amplitude ${(vmax - vmin) / 2} vs particle ${up}`);
});

test('finite-depth wave dispersion and profiles', () => {
  const env = new Environment({ waves: { amplitude: 0.1, period: 4, direction: 0 }, pool: { length: 30, width: 10, depth: 1.5 } }, new RNG(1));
  const w = (2 * Math.PI) / 4, k = env.waveK, h = 1.5;
  assert.ok(Math.abs(env.g * k * Math.tanh(k * h) - w * w) < 1e-9);
  assert.ok(k > (w * w) / env.g * 1.5, 'shallow water: shorter waves than deep-water dispersion');
  // vertical velocity vanishes at the floor, horizontal does not
  const pf = env.waveProfile(0, h);
  assert.ok(Math.abs(pf.fw) < 1e-12 && pf.fu > 0.5 / (k * h) * 0.5);
  // deep water reduces to exp(-k z)
  const deep = new Environment({ waves: { amplitude: 0.1, period: 1, direction: 0 }, pool: { depth: 50 } }, new RNG(1));
  assert.ok(Math.abs(deep.waveProfile(0, 1).fu - Math.exp(-deep.waveK)) < 1e-9);
  assert.equal(new Environment({ temperature: 0 }, new RNG(1)).T68, 0, '0 degC is not replaced by 20 degC');
});

test('prop reaction torque: sign from handedness, mirror pair cancels', () => {
  const single = vehicle(geo432, { thrusters: [thrusterFromPreset({ name: 'a', pos: [-0.2, 0, 0.03], axis: [1, 0, 0] })] });
  assert.equal(single.plant.thrusters[0].hand, 1);
  single.sim.reset({ position: [0, 0, 2] });
  for (let i = 0; i < 400; i++) single.sim.step(1 / 800, [1900]);
  const th = single.sim.thrusters[0];
  assert.ok(Math.abs(th.Q + 0.12 * 0.076 * th.T * th.vent) < 1e-9 && th.Q < -0.2, `Q ${th.Q}`);
  const pair = vehicle(geo432, { thrusters: [thrusterFromPreset({ name: 'r', pos: [-0.2, 0.1, 0.03], axis: [1, 0, 0] }), thrusterFromPreset({ name: 'l', pos: [-0.2, -0.1, 0.03], axis: [1, 0, 0] })] });
  assert.deepEqual(pair.plant.thrusters.map((t) => t.hand), [1, -1]);
  pair.sim.reset({ position: [0, 0, 2] });
  for (let i = 0; i < 400; i++) pair.sim.step(1 / 800, [1900, 1900]);
  assert.ok(Math.abs(pair.sim.diag.thrust[3]) < 1e-9, `roll torque ${pair.sim.diag.thrust[3]}`);
});

test('reverse thrust uses deductionRev', () => {
  const t = new ThrusterState({ table: T200, tauUp: 0, tauDown: 0, deduction: 0.1, deductionRev: 0.5, inflowModel: false });
  t.update(1100, 0.01);
  assert.ok(Math.abs(t.effective(0, 5) - 0.5 * t.T) < 1e-9);
  t.update(1900, 0.01);
  assert.ok(Math.abs(t.effective(0, 5) - 0.9 * t.T) < 1e-9);
});

test('sealed hull: buoyancy elements carry the filled cavity (no jump at full submergence)', () => {
  const o = [];
  box(o, [0, 0, 0.09], [0.4, 0.3, 0.02]); box(o, [0, 0, -0.09], [0.4, 0.3, 0.02]);
  box(o, [0.19, 0, 0], [0.02, 0.3, 0.16]); box(o, [-0.19, 0, 0], [0.02, 0.3, 0.16]);
  box(o, [0, 0.14, 0], [0.36, 0.02, 0.16]); box(o, [0, -0.14, 0], [0.36, 0.02, 0.16]);
  const geo = analyzeGeometry(new Float32Array(o), { sealedHull: true, targetCells: 40 });
  const sum = geo.buoyEls.reduce((s, e) => s + e.v, 0);
  assert.ok(Math.abs(sum / geo.volume - 1) < 1e-9 && geo.volume > 0.022, `${sum} vs ${geo.volume}`);
  const { sim } = vehicle(geo, { mass: 10, cgOffset: [0, 0, 0] });
  const B = (z) => { sim.reset({ position: [0, 0, z] }); sim.step(1e-5); return sim.diag.buoyancyMag; };
  const r = sim.boundR;
  assert.ok(Math.abs(B(r - 0.005) / B(r + 0.005) - 1) < 0.01);
});

test('auto inertia uses the voxel volume it was integrated over (thin plate)', () => {
  const geo = analyzeGeometry(new Float32Array(box([], [0, 0, 0], [0.4, 0.4, 0.0127])), { targetCells: 64 });
  const cfg = defaultConfig();
  cfg.mass.mass = 10; cfg.mass.cgMode = 'cb-offset'; cfg.mass.cgOffset = [0, 0, 0];
  const I = inertiaTensor(cfg, geo, geo.centroid);
  const exact = (10 * (0.16 + 0.16)) / 12;
  assert.ok(Math.abs(I[8] / exact - 1) < 0.03, `Izz ${I[8]} vs ${exact}`);
});

test('estimated added mass matches the displaced volume, has rotational terms and sits at the CG', () => {
  const sg = shapeGeometry({ type: 'openframe' });
  const geo = analyzeGeometry(sg.positions);
  const { info } = vehicle(geo, { cgOffset: [0, 0, 0.03] });
  const MA = info.MA;
  assert.ok(MA[0][0] < 0.8 * 1000 * geo.volume * 2, 'surge added mass not inflated by the bbox');
  for (let i = 3; i < 6; i++) assert.ok(MA[i][i] > 0.02, `rotational term ${i}: ${MA[i][i]}`);
  assert.ok(Math.abs(MA[0][4]) > 1e-3, 'surge-pitch coupling from the CG shift');
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) assert.ok(Math.abs(MA[i][j] - MA[j][i]) < 1e-12);
});

test('flooded water adds mass and inertia, CFD damping is moved to the CG', () => {
  const { sim } = vehicle(geo432, { mod: (c) => { c.hull.airVolume = 5; } });
  sim.reset({ position: [0, 0, 2] });
  sim.step(1 / 800);
  const M0 = sim.M[0][0];
  sim.flood.volume = 0.004;
  sim.step(1 / 800);
  assert.ok(Math.abs(sim.M[0][0] - M0 - sim.env.rho * 0.004) < 1e-6 * M0);
  const DL = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => (i === j ? [10, 10, 10, 1, 1, 1][i] : 0)));
  const DQ = DL.map((r) => r.slice());
  const { info } = vehicle(geo432, { mod: (c) => { c.hydro.damping = { ...c.hydro.damping, mode: 'cfd', DL, DQ, ref: [0, 0, 0] }; } });
  // reference 3 cm above the CG: sway-roll coupling D24 = -D22 * dz appears
  assert.ok(Math.abs(info.DL[1][3]) > 0.1 && Math.abs(info.DL[1][3] - info.DL[3][1]) < 1e-12, `D24 ${info.DL[1][3]}`);
});

test('torpedo shape has the requested overall length', () => {
  const g = shapeGeometry({ type: 'torpedo', length: 0.46, diameter: 0.2, ducts: false });
  assert.ok(Math.abs(bbox(g.positions).size[0] - 0.46) < 2e-3);
});

test('stiff damping on a light vehicle is sub-stepped instead of diverging', () => {
  const geo = boxGeo([0.1, 0.08, 0.06]);
  const { sim } = vehicle(geo, { linear: [400, 400, 400, 0.5, 0.5, 0.5], quadratic: [800, 800, 800, 1, 1, 1], env: { pool: { length: 20, width: 20, depth: 5 } } });
  sim.reset({ position: [0, 0, 2], velocity: [2, -1, 0.5], rates: [3, -2, 4] });
  let most = 0;
  for (let i = 0; i < 400; i++) { sim.step(1 / 200); most = Math.max(most, sim.rkSubsteps); }
  assert.ok(most > 1 && sim.y.every(Number.isFinite), `substeps ${most}`);
  assert.ok(Math.hypot(...sim.velocityBody) < 1e-3 && Math.hypot(...sim.rates) < 1e-2);
});
