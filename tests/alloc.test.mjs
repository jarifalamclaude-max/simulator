import test from 'node:test';
import assert from 'node:assert/strict';
import { svd, pinv, matMul, qFromEuler, qToEuler, qrotate, qToMat, m3mulv } from '../src/core/math.js';
import { analyzeLayout, tableQuality } from '../src/alloc/allocation.js';
import { heavyLayout, vectoredLayout, frameTable, layoutFromTable, FRAMES } from '../src/alloc/frames.js';

const close = (a, b, tol = 1e-6) => assert.ok(Math.abs(a - b) <= tol, `${a} != ${b}`);

test('svd reconstructs and pinv is a generalized inverse', () => {
  const A = [[1, 2, 3, 4], [2, 4, 6, 8.0001], [0, 1, 0, 1]];
  const { U, S, V } = svd(A);
  for (let i = 0; i < 3; i++)
    for (let j = 0; j < 4; j++) {
      let s = 0;
      for (let k = 0; k < S.length; k++) s += U[i][k] * S[k] * V[j][k];
      close(s, A[i][j], 1e-9);
    }
  const P = pinv(A);
  const APA = matMul(matMul(A, P), A);
  for (let i = 0; i < 3; i++) for (let j = 0; j < 4; j++) close(APA[i][j], A[i][j], 1e-6);
});

test('quaternion/euler round trip follows ArduPilot 321 convention', () => {
  const e = [0.3, -0.2, 2.5];
  const q = qFromEuler(...e);
  const back = qToEuler(q);
  e.forEach((v, i) => close(back[i], v, 1e-12));
  // body x axis rotated by yaw 90deg points east in NED
  const q2 = qFromEuler(0, 0, Math.PI / 2);
  const x = qrotate(q2, [1, 0, 0]);
  close(x[0], 0, 1e-12); close(x[1], 1, 1e-12);
  // matrix agrees with qrotate
  const M = qToMat(q);
  const v = [0.3, 0.7, -1.1];
  const a = qrotate(q, v), b = m3mulv(M, v);
  a.forEach((val, i) => close(val, b[i], 1e-12));
  // positive pitch lifts the nose: body x has negative NED z
  const qp = qFromEuler(0, 0.4, 0);
  assert.ok(qrotate(qp, [1, 0, 0])[2] < 0);
});

const T200 = { fwdMax: 51.5, revMax: 40.2 };
const withT = (layout) => layout.map((t) => ({ ...t, ...T200 }));

test('BlueROV2 Heavy layout is fully 6-DOF', () => {
  const r = analyzeLayout(withT(heavyLayout()), [0, 0, 0]);
  assert.equal(r.controllable.length, 6, r.summary);
  // authorities are positive in both directions
  for (const a of r.controllable) assert.ok(r.authority[a].pos > 0 && r.authority[a].neg > 0, a);
});

test('auto factors for the Heavy layout reproduce ArduSub VECTORED_6DOF signs', () => {
  const r = analyzeLayout(withT(heavyLayout()), [0, 0, 0]);
  const table = frameTable(2);
  // compare sign pattern of every non-zero ArduSub factor
  for (let i = 0; i < 8; i++)
    for (let k = 0; k < 6; k++) {
      if (table[i][k] === 0) continue;
      assert.equal(Math.sign(r.factors[i][k]), Math.sign(table[i][k]), `motor ${i + 1} axis ${k}`);
    }
});

test('standard BlueROV2 (vectored) has no independent pitch: 5-DOF', () => {
  const r = analyzeLayout(withT(vectoredLayout()), [0, 0, 0]);
  assert.equal(r.controllable.length, 5, r.summary);
  assert.ok(!r.controllable.includes('pitch'));
  assert.notEqual(r.status.pitch, 'full');
});

test('SimpleROV-3 gives surge, yaw and heave', () => {
  const lay = withT(layoutFromTable(frameTable(4)));
  const r = analyzeLayout(lay, [0, 0, 0]);
  assert.deepEqual([...r.controllable].sort(), ['forward', 'throttle', 'yaw']);
});

test('ArduSub VECTORED_6DOF table on the Heavy geometry has no wrong-signed axes', () => {
  const lay = withT(heavyLayout());
  const q = tableQuality(lay, frameTable(2), [0, 0, 0], null);
  for (const row of q.rows) assert.ok(row.main > 0, row.axis);
});

test('reversing one thruster with MOT_DIRECTION but not hardware is detected as coupling', () => {
  const lay = withT(heavyLayout());
  const signs = [1, -1, 1, 1, 1, 1, 1, 1];
  const q = tableQuality(lay, frameTable(2), [0, 0, 0], signs);
  const yaw = q.rows.find((r) => r.axis === 'yaw');
  assert.ok(yaw.off.length > 0, 'expected cross-coupling when a motor is reversed');
  assert.ok(FRAMES[7].warn);
});
