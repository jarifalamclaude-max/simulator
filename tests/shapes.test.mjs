// Simple parametric bodies (Frame / Vehicle > Simple shape).
import test from 'node:test';
import assert from 'node:assert/strict';
import { shapeGeometry, DEFAULT_SHAPE } from '../src/vehicle/shapes.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { heavyLayout } from '../src/alloc/frames.js';

test('closed box body has the right volume', () => {
  const g = shapeGeometry({ type: 'box', length: 0.4, width: 0.3, height: 0.2, ducts: false });
  const a = analyzeGeometry(g.positions, { targetCells: 40 });
  assert.ok(Math.abs(a.volume - 0.024) / 0.024 < 0.03, `${a.volume}`);
});

test('torpedo body volume matches cylinder + ellipsoidal ends', () => {
  const L = 1.2, D = 0.2, r = D / 2;
  const g = shapeGeometry({ type: 'torpedo', length: L, diameter: D, ducts: false });
  const a = analyzeGeometry(g.positions, { targetCells: 60 });
  // cylinder L - 1.3 D plus caps protruding 0.75 D and 0.55 D: overall length exactly L
  const V = Math.PI * r * r * (L - 1.3 * D) + (2 / 3) * Math.PI * r * r * (0.75 * D) + (2 / 3) * Math.PI * r * r * (0.55 * D);
  assert.ok(Math.abs(a.volume - V) / V < 0.05, `${a.volume} vs ${V}`);
});

test('open frame carries a duct at every thruster and follows the thrusters', () => {
  const th = heavyLayout().map((t) => ({ ...t, propDiameter: 0.076 }));
  const g = shapeGeometry({ ...DEFAULT_SHAPE }, th);
  assert.ok(g.parts.some((p) => p.name === 'Thrusters'));
  const a = analyzeGeometry(g.positions);
  assert.ok(a.volume > 0.005 && a.volume < 0.04, `${a.volume}`);
  // move one thruster outward: the body grows to include it
  const moved = th.map((t, i) => (i === 0 ? { ...t, pos: [0.4, t.pos[1], t.pos[2]] } : t));
  const b = analyzeGeometry(shapeGeometry({ ...DEFAULT_SHAPE }, moved).positions);
  assert.ok(b.bbox.max[0] > a.bbox.max[0] + 0.1);
});

test('displaced volume of thin plates is integrated exactly, not voxel-staircased', async () => {
  const { box } = await import('../src/geometry/primitives.js');
  const plates = [];
  box(plates, [0, 0.163, 0], [0.46, 0.0127, 0.26]);
  box(plates, [0, -0.163, 0], [0.46, 0.0127, 0.26]);
  const a = analyzeGeometry(new Float32Array(plates));
  const exact = 2 * 0.46 * 0.0127 * 0.26;
  assert.equal(a.volumeSource, 'ray');
  assert.ok(Math.abs(a.volume / exact - 1) < 0.003, `${a.volume} vs ${exact} (voxels: ${a.voxelVolume})`);
  // the open-frame body keeps its volume while a thruster is dragged around
  const th = heavyLayout().map((t) => ({ ...t, propDiameter: 0.076 }));
  const vols = [0.15, 0.19, 0.23].map((y) => analyzeGeometry(shapeGeometry({ ...DEFAULT_SHAPE }, th.map((t, i) => (i === 0 ? { ...t, pos: [0.156, y, 0.085] } : t))).positions).volume);
  assert.ok(Math.max(...vols) / Math.min(...vols) - 1 < 0.025, vols.map((v) => (v * 1000).toFixed(2)).join(' '));
});
