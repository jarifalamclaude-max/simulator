import test from 'node:test';
import assert from 'node:assert/strict';
import { box } from '../src/geometry/primitives.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { defaultConfig, thrusterFromPreset } from '../src/vehicle/config.js';
import { buildPlantModel } from '../src/vehicle/model.js';
import { Environment } from '../src/physics/environment.js';
import { VehiclePlant } from '../src/physics/body.js';
import { RNG } from '../src/core/rng.js';
import { qFromEuler, qToEuler } from '../src/core/math.js';
import { sampleGeometry } from '../src/vehicle/sample.js';

const boxGeo = analyzeGeometry(new Float32Array(box([], [0, 0, 0], [0.4, 0.3, 0.2])), { targetCells: 40 });

function boxVehicle(mods = {}) {
  const cfg = defaultConfig();
  cfg.model.source = 'test';
  const env = new Environment({ ...cfg.environment, temperature: 4 }, new RNG(1));
  const rho = env.rho;
  cfg.mass.mass = mods.mass ?? rho * boxGeo.volume;
  cfg.mass.cgMode = 'cb-offset';
  cfg.mass.cgOffset = mods.cgOffset ?? [0, 0, 0.03];
  cfg.hydro.damping.mode = 'manual';
  cfg.hydro.damping.linear = mods.linear ?? [5, 5, 5, 0.05, 0.05, 0.05];
  cfg.hydro.damping.quadratic = mods.quadratic ?? [30, 30, 30, 0.2, 0.2, 0.2];
  cfg.thrusters = mods.thrusters ?? [thrusterFromPreset({ name: 'aft', pos: [-0.2, 0, 0.03], axis: [1, 0, 0] })];
  cfg.thrusters.forEach((t) => (t.deductionMode = 'manual'));
  const { plant, info } = buildPlantModel(cfg, boxGeo, env);
  const sim = new VehiclePlant(plant, env);
  return { sim, info, env, cfg };
}

test('box volume and centroid from voxels', () => {
  assert.ok(Math.abs(boxGeo.volume - 0.024) / 0.024 < 0.03, String(boxGeo.volume));
  boxGeo.centroid.forEach((c) => assert.ok(Math.abs(c) < 0.005));
});

test('neutrally buoyant vehicle holds depth and attitude', () => {
  const { sim } = boxVehicle();
  sim.reset({ position: [0, 0, 1.5] });
  for (let i = 0; i < 8000; i++) sim.step(1 / 800);
  const p = sim.position;
  assert.ok(Math.abs(p[2] - 1.5) < 0.01, `depth drifted to ${p[2]}`);
  const e = qToEuler(sim.quaternion);
  assert.ok(Math.abs(e[0]) < 1e-3 && Math.abs(e[1]) < 1e-3);
});

test('positively buoyant vehicle floats with the right draft', () => {
  const env0 = new Environment({ temperature: 4 }, new RNG(1));
  const m = 0.5 * env0.rho * boxGeo.volume; // half submerged
  const { sim } = boxVehicle({ mass: m, cgOffset: [0, 0, 0.05] });
  sim.reset({ position: [0, 0, 0.5] });
  for (let i = 0; i < 24000; i++) sim.step(1 / 800);
  const sub = sim.diag.submerged;
  assert.ok(Math.abs(sub - 0.5) < 0.04, `submerged fraction ${sub}`);
});

test('roll free-decay period matches restoring-moment theory', () => {
  const { sim, info } = boxVehicle({ linear: [5, 5, 5, 0, 0, 0], quadratic: [30, 30, 30, 0, 0, 0] });
  sim.reset({ position: [0, 0, 2], quaternion: qFromEuler(0.15, 0, 0) });
  const dt = 1 / 800;
  let last = null, crossings = [];
  for (let i = 0; i < 20000 && crossings.length < 5; i++) {
    sim.step(dt);
    const r = qToEuler(sim.quaternion)[0];
    if (last !== null && last > 0 && r <= 0) crossings.push(sim.t);
    last = r;
  }
  const T = (crossings[crossings.length - 1] - crossings[0]) / (crossings.length - 1);
  assert.ok(Math.abs(T - info.rollPeriod) / info.rollPeriod < 0.05, `period ${T} vs ${info.rollPeriod}`);
});

test('steady surge speed balances thrust and drag', () => {
  const c = boxGeo.centroid;
  const { sim } = boxVehicle({ thrusters: [thrusterFromPreset({ name: 'aft', pos: [-0.2, c[1], c[2] + 0.03], axis: [1, 0, 0] })], linear: [5, 5, 5, 0.5, 0.5, 0.5], quadratic: [30, 30, 30, 2, 2, 2] });
  sim.thrusters[0].def.inflowModel = false;
  // a lone prop's reaction torque rolls the box and seeds its (physical) Munk yaw instability
  sim.thrusters[0].torqueRatio = 0;
  sim.reset({ position: [-10, 0, 2] });
  // steady after ~4 s; much longer and the box's Munk yaw instability (A22 > A11) takes over
  for (let i = 0; i < 4800; i++) sim.step(1 / 800, [1700]);
  const T = sim.thrusters[0].Teff;
  const u = sim.velocityBody[0];
  const drag = 5 * u + 30 * u * u;
  assert.ok(T > 5, `thrust ${T}`);
  assert.ok(Math.abs(drag - T) / T < 0.03, `drag ${drag} vs thrust ${T} at u=${u}`);
});

test('in-flow reduces thrust at speed', () => {
  const a = boxVehicle(), b = boxVehicle();
  b.sim.thrusters[0].def.inflowModel = false;
  for (const v of [a, b]) v.sim.thrusters[0].torqueRatio = 0;
  for (const v of [a, b]) { v.sim.reset({ position: [-10, 0, 2] }); for (let i = 0; i < 8000; i++) v.sim.step(1 / 800, [1850]); }
  assert.ok(a.sim.velocityBody[0] < b.sim.velocityBody[0], 'in-flow model should lower top speed');
});

test('heavy vehicle settles on the pool floor without exploding', () => {
  const env0 = new Environment({ temperature: 4 }, new RNG(1));
  const { sim, env } = boxVehicle({ mass: 1.3 * env0.rho * boxGeo.volume });
  sim.reset({ position: [0, 0, env.cfg.pool.depth - 0.5] });
  for (let i = 0; i < 16000; i++) sim.step(1 / 800);
  const z = sim.position[2];
  assert.ok(z < env.cfg.pool.depth && z > env.cfg.pool.depth - 0.2, `rest depth ${z}`);
  assert.ok(Math.abs(sim.velocityBody[2]) < 0.01);
});

test('sample vehicle builds a valid, stable model', () => {
  const sg = sampleGeometry();
  const geo = analyzeGeometry(sg.positions);
  const cfg = defaultConfig();
  const env = new Environment(cfg.environment, new RNG(1));
  cfg.mass.mass = env.rho * geo.volume * 0.99;
  const { plant, info } = buildPlantModel(cfg, geo, env);
  assert.ok(info.GM > 0.01, `GM ${info.GM}`);
  assert.equal(plant.thrusters.length, 8);
  assert.ok(info.netBuoyancyN > 0);
  const sim = new VehiclePlant(plant, env);
  sim.reset({ position: [0, 0, 1] });
  for (let i = 0; i < 4000; i++) sim.step(1 / 800);
  assert.ok(Number.isFinite(sim.position[2]));
});
