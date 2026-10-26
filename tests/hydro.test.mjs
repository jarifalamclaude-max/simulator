import test from 'node:test';
import assert from 'node:assert/strict';
import { sphere, ellipsoid, box, tube } from '../src/geometry/primitives.js';
import { bbox, makeGrid, voxelizeSolid, voxelizeSurface, massProps, topology, signedVolume, occupancyField, surfaceNets, quadPanels } from '../src/geometry/mesh.js';
import { solveAddedMass, shiftMassMatrix } from '../src/hydro/bem.js';
import { ellipsoidAddedMass, waterDensity, kinematicViscosity } from '../src/hydro/water.js';
import { parseSTL, writeBinarySTL } from '../src/geometry/stl.js';

const rel = (a, b) => Math.abs(a - b) / Math.abs(b);

test('water properties are physical', () => {
  assert.ok(Math.abs(waterDensity(20, 0) - 998.2) < 0.1);
  assert.ok(Math.abs(waterDensity(4, 0) - 1000.0) < 0.05);
  assert.ok(Math.abs(waterDensity(15, 35) - 1026) < 2);
  const nu = kinematicViscosity(20, 0);
  assert.ok(nu > 0.95e-6 && nu < 1.06e-6, String(nu));
});

test('STL round trip and topology', () => {
  const pos = new Float32Array(box([], [0, 0, 0], [1, 2, 3]));
  const back = parseSTL(writeBinarySTL(pos));
  assert.equal(back.triCount, 12);
  const t = topology(back.positions);
  assert.ok(t.watertight);
  assert.ok(Math.abs(signedVolume(back.positions).volume - 6) < 1e-5);
});

test('voxel volume and inertia of a cylinder match analytic values', () => {
  const pos = new Float32Array(tube([], [0, 0, 0], [1, 0, 0], 0.1, 0.0001, 0.6, 48));
  const bb = bbox(pos);
  const g = makeGrid(bb, 0.008);
  voxelizeSolid(pos, g);
  const mp = massProps(g);
  const V = Math.PI * 0.01 * 0.6;
  assert.ok(rel(mp.volume, V) < 0.03, `volume ${mp.volume} vs ${V}`);
  // unit density inertia about x: V r^2 / 2
  assert.ok(rel(mp.inertiaUnit[0], (V * 0.01) / 2) < 0.06, 'Ixx');
});

test('overlapping solids are united (winding rule)', () => {
  const pos = new Float32Array([...box([], [0, 0, 0], [0.4, 0.2, 0.2]), ...box([], [0.1, 0, 0], [0.4, 0.2, 0.2])]);
  const g = makeGrid(bbox(pos), 0.01);
  voxelizeSolid(pos, g);
  const V = massProps(g).volume;
  assert.ok(rel(V, 0.5 * 0.2 * 0.2) < 0.03, `union volume ${V}`);
});

test('thin plate survives conservative voxelisation', () => {
  const pos = new Float32Array(box([], [0, 0, 0], [0.3, 0.004, 0.2]));
  const g = makeGrid(bbox(pos), 0.02);
  voxelizeSurface(pos, g);
  assert.ok(g.count() > 0);
});

test('BEM added mass of a sphere = rho V / 2 within 2%', () => {
  const pos = new Float32Array(sphere([], [0, 0, 0], 0.1, 32, 48));
  const g = makeGrid(bbox(pos), 0.018, 2);
  const panels = quadPanels(surfaceNets(occupancyField(pos, g, { keepThin: false }), g, 0.5));
  const res = solveAddedMass(panels, { rho: 1000 });
  const target = 0.5 * 1000 * (4 / 3) * Math.PI * 0.001;
  for (let i = 0; i < 3; i++) assert.ok(rel(res.M[i][i], target) < 0.02, `m${i}${i} ${res.M[i][i]} vs ${target}`);
  for (let i = 3; i < 6; i++) assert.ok(Math.abs(res.M[i][i]) < 1e-3 * target, `rot ${res.M[i][i]}`);
});

test('BEM matches Lamb ellipsoid added mass (3 : 1.5 : 1) within 5%', () => {
  const a = 0.15, b = 0.075, c = 0.05;
  const pos = new Float32Array(ellipsoid([], [0, 0, 0], [a, b, c], 32, 48));
  const g = makeGrid(bbox(pos), 0.01, 2);
  const panels = quadPanels(surfaceNets(occupancyField(pos, g, { keepThin: false }), g, 0.5));
  const res = solveAddedMass(panels, { rho: 1000 });
  const ref = ellipsoidAddedMass(a, b, c, 1000).values;
  for (let i = 0; i < 6; i++) assert.ok(rel(res.M[i][i], ref[i]) < 0.06, `axis ${i}: BEM ${res.M[i][i]} Lamb ${ref[i]}`);
});

test('moving the reference point keeps translational terms and adds coupling', () => {
  const M = [[2,0,0,0,0,0],[0,3,0,0,0,0],[0,0,4,0,0,0],[0,0,0,1,0,0],[0,0,0,0,1,0],[0,0,0,0,0,1]];
  const S = shiftMassMatrix(M, [0, 0, 0], [0, 0, 0.1]);
  assert.equal(S[0][0], 2);
  assert.ok(Math.abs(S[4][4] - (1 + 2 * 0.01)) < 1e-12, 'pitch inertia grows by m*d^2');
  assert.ok(Math.abs(S[0][4]) > 0, 'surge-pitch coupling appears');
});
