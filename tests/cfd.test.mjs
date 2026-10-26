// Lattice-Boltzmann CFD lab: lattice constants, solver invariants, momentum
// exchange, moving walls, case building and the damping assembly.
import test from 'node:test';
import assert from 'node:assert/strict';
import { Q, C, W, OPP, geq } from '../src/cfd/d3q19.js';
import { LBMCPU } from '../src/cfd/cpu.js';
import { buildCase, staircaseFraction, assemble, caseSettings, runCoefficients, DOFS } from '../src/cfd/study.js';
import { VoxelGrid } from '../src/geometry/mesh.js';
import { box, sphere } from '../src/geometry/primitives.js';

test('D3Q19 weights, opposites and isotropy', () => {
  assert.equal(Q, 19);
  assert.ok(Math.abs(W.reduce((a, b) => a + b, 0) - 1) < 1e-15);
  for (let i = 0; i < Q; i++) for (let k = 0; k < 3; k++) assert.equal(C[OPP[i]][k] + C[i][k], 0);
  for (let a = 0; a < 3; a++) for (let b = 0; b < 3; b++) {
    const s = C.reduce((acc, c, i) => acc + W[i] * c[a] * c[b], 0);
    assert.ok(Math.abs(s - (a === b ? 1 / 3 : 0)) < 1e-15);
  }
});

test('equilibrium carries the right density and momentum', () => {
  const rho = 1.02, u = [0.05, -0.02, 0.01];
  let m0 = 0; const m1 = [0, 0, 0];
  for (let i = 0; i < Q; i++) { const g = geq(i, rho, ...u); m0 += g; for (let k = 0; k < 3; k++) m1[k] += C[i][k] * g; }
  assert.ok(Math.abs(m0 - (rho - 1)) < 1e-14);
  for (let k = 0; k < 3; k++) assert.ok(Math.abs(m1[k] - rho * u[k]) < 1e-14);
});

test('free stream stays uniform with no body', () => {
  const nx = 16, ny = 10, nz = 10;
  const s = new LBMCPU({ nx, ny, nz, mask: new Uint8Array(nx * ny * nz) }, { tau0: 0.56, uInf: [0.06, 0.01, 0] });
  // start from the free-stream equilibrium
  for (let i = 0; i < Q; i++) s.A.fill(geq(i, 1, 0.06, 0.01, 0), i * s.N, (i + 1) * s.N);
  s.step(40);
  let maxErr = 0;
  for (let id = 0; id < s.N; id++) { const m = s.macro(id); maxErr = Math.max(maxErr, Math.abs(m[0] - 0.06), Math.abs(m[1] - 0.01), Math.abs(m[2]), Math.abs(m[3] - 1)); }
  assert.ok(maxErr < 2e-6, `max deviation ${maxErr}`);
});

function blockCase(nx, ny, nz, lo, hi) {
  const mask = new Uint8Array(nx * ny * nz);
  for (let z = lo[2]; z < hi[2]; z++) for (let y = lo[1]; y < hi[1]; y++) for (let x = lo[0]; x < hi[0]; x++) mask[x + nx * (y + ny * z)] = 1;
  return mask;
}

test('drag on a symmetric block points downstream with no side force', () => {
  const nx = 36, ny = 20, nz = 20;
  const mask = blockCase(nx, ny, nz, [10, 7, 7], [16, 13, 13]);
  const ref = [12.5, 9.5, 9.5];
  const s = new LBMCPU({ nx, ny, nz, mask }, { tau0: 0.6, uInf: [0, 0, 0], ref });
  for (let k = 0; k < 30; k++) { s.setInflow([0.05 * Math.min(1, (k + 1) / 10), 0, 0]); s.step(10); }
  const { F, M } = s.force();
  assert.ok(F[0] > 0, 'drag along the flow');
  assert.ok(Math.abs(F[1]) < 1e-6 * F[0] && Math.abs(F[2]) < 1e-6 * F[0], `side forces ${F}`);
  for (const m of M) assert.ok(Math.abs(m) < 1e-5 * F[0] * 6, `moment ${M}`);
});

test('a spinning body feels a torque opposing the spin', () => {
  const nx = 24, ny = 24, nz = 24;
  const mask = new Uint8Array(nx * ny * nz);
  const c = 11.5;
  for (let z = 0; z < nz; z++) for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) if ((x - c) ** 2 + (y - c) ** 2 + (z - c) ** 2 <= 25) mask[x + nx * (y + ny * z)] = 1;
  const s = new LBMCPU({ nx, ny, nz, mask }, { tau0: 0.8, omega: [0, 0, 0.004], ref: [c, c, c], spongeFrac: 0 });
  s.step(150);
  const { F, M } = s.force();
  assert.ok(M[2] < 0, `torque ${M}`);
  assert.ok(Math.abs(M[0]) < 0.05 * Math.abs(M[2]) && Math.abs(M[1]) < 0.05 * Math.abs(M[2]));
  assert.ok(Math.hypot(...F) < 0.05 * Math.abs(M[2]));
  // Stokes rotlet: M = 8 pi mu R^3 omega; a sphere of 5 cells in a box of 24 at
  // creeping flow should be within a factor ~1.6 of it (still spinning up, staircase radius)
  const mu = (0.8 - 0.5) / 3;
  const stokes = 8 * Math.PI * mu * 125 * 0.004;
  assert.ok(Math.abs(M[2]) > 0.5 * stokes && Math.abs(M[2]) < 1.6 * stokes, `|M| ${Math.abs(M[2])} vs Stokes ${stokes}`);
});

test('staircase fraction separates curved from lattice-aligned surfaces', () => {
  const mk = (n, f) => { const g = new VoxelGrid(n, n, n, 1, [0, 0, 0]); for (let k = 0; k < n; k++) for (let j = 0; j < n; j++) for (let i = 0; i < n; i++) if (f(i, j, k)) g.data[i + n * (j + n * k)] = 1; return g; };
  const sph = staircaseFraction(mk(18, (i, j, k) => (i - 8.5) ** 2 + (j - 8.5) ** 2 + (k - 8.5) ** 2 <= 49));
  const cube = staircaseFraction(mk(18, (i, j, k) => i > 2 && j > 2 && k > 2 && i < 15 && j < 15 && k < 15));
  assert.ok(sph.fraction > 0.8, `sphere ${sph.fraction}`);
  assert.ok(cube.fraction < 0.2, `cube ${cube.fraction}`);
});

test('cases map each tow direction onto the lattice x axis', () => {
  const pos = Float32Array.from(box([], [0, 0, 0], [0.4, 0.3, 0.2]));
  const L = 0.4, N = 10;
  const surge = buildCase(pos, { dof: 'surge', N, ellRef: L });
  const sway = buildCase(pos, { dof: 'sway', N, ellRef: L });
  const heave = buildCase(pos, { dof: 'heave', N, ellRef: L });
  // projected (frontal) areas in cells: surge sees Y x Z, sway X x Z, heave X x Y
  const cellsAlong = (cs) => { let lo = Infinity, hi = -Infinity; for (let z = 0; z < cs.nz; z++) for (let y = 0; y < cs.ny; y++) for (let x = 0; x < cs.nx; x++) if (cs.mask[x + cs.nx * (y + cs.ny * z)]) { lo = Math.min(lo, x); hi = Math.max(hi, x); } return hi - lo + 1; };
  // centre-inside marking: a 7.5-cell body is 7 or 8 cells long
  assert.ok(Math.abs(cellsAlong(surge) - 0.4 / 0.04) <= 0.5);
  assert.ok(Math.abs(cellsAlong(sway) - 0.3 / 0.04) <= 0.5);
  assert.ok(Math.abs(cellsAlong(heave) - 0.2 / 0.04) <= 0.5);
  assert.ok(Math.abs(surge.proj[0].area / ((0.3 / 0.04) * (0.2 / 0.04)) - 1) < 0.1, `surge frontal ${surge.proj[0].area}`);
  assert.ok(Math.abs(heave.proj[0].area / ((0.4 / 0.04) * (0.3 / 0.04)) - 1) < 0.1, `heave frontal ${heave.proj[0].area}`);
  // the reference point (CG at the origin) sits inside the body
  for (const cs of [surge, sway, heave]) {
    const r = cs.ref.map(Math.round);
    assert.equal(cs.mask[r[0] + cs.nx * (r[1] + cs.ny * r[2])], 1);
  }
});

test('coefficients map back to the body frame with the right signs', () => {
  const pos = Float32Array.from(box([], [0, 0, 0], [0.4, 0.3, 0.2]));
  for (const dof of ['surge', 'sway', 'heave']) {
    const cs = buildCase(pos, { dof, N: 10, ellRef: 0.4 });
    const st = caseSettings(cs, 1000);
    // water pushes the body along +x of the lattice: drag opposes the tow
    const C6 = runCoefficients(cs, st, { F: [1, 0, 0], M: [0, 0, 0] });
    const j = DOFS.findIndex((d) => d.key === dof);
    assert.ok(C6[j] < 0, `${dof}: ${C6}`);
    for (let i = 0; i < 6; i++) if (i !== j) assert.equal(C6[i], 0);
  }
  for (const dof of ['roll', 'pitch', 'yaw']) {
    const cs = buildCase(pos, { dof, N: 10, ellRef: 0.4 });
    const st = caseSettings(cs, 1000);
    const j = DOFS.findIndex((d) => d.key === dof);
    // a moment against the spin direction, given in lattice axes
    const M = st.omegaDir.map((v) => -v);
    const C6 = runCoefficients(cs, st, { F: [0, 0, 0], M });
    assert.ok(C6[j] < 0, `${dof}: ${C6}`);
  }
});

test('assembly turns coefficients into Fossen damping matrices', () => {
  const rho = 1000, nu = 1e-6, L = 0.5;
  // drag C = -0.3 - 50/Re in surge, a small surge->pitch coupling (towed both
  // ways: couplings need both directions), pitch and yaw spin damping
  const runs = [1, -1].flatMap((sign) => [500, 2000].map((Re) => ({ dof: 'surge', sign, Re, C: [-0.3 - 50 / Re, 0, 0, 0, 0.01, 0] })));
  runs.push(...[500, 2000].map((Re) => ({ dof: 'pitch', sign: 1, Re, C: [0, 0, 0, 0, -0.05, 0] })));
  runs.push(...[500, 2000].map((Re) => ({ dof: 'yaw', sign: 1, Re, C: [0, 0, 0, 0, 0, -0.02 - 4 / Re] })));
  const { DQ, DL } = assemble(runs, { rho, nu, ellRef: L });
  assert.ok(Math.abs(DQ[0][0] - rho * L * L * 0.3) < 1e-9);
  assert.ok(Math.abs(DL[0][0] - rho * L * 50 * nu) < 1e-12);
  assert.ok(Math.abs(DQ[4][0] - -rho * L ** 3 * 0.01) < 1e-9);
  assert.ok(Math.abs(DQ[5][5] - rho * L ** 5 * 0.02) < 1e-12);
  assert.ok(Math.abs(DL[5][5] - rho * L ** 3 * 4 * nu) < 1e-15);
  assert.equal(DQ[1][1], 0);
});

test('the voxelised sphere case is closed and sized correctly', () => {
  const pos = Float32Array.from(sphere([], [0, 0, 0], 0.05, 24, 32));
  const cs = buildCase(pos, { dof: 'surge', N: 10, ellRef: 0.1 });
  assert.ok(Math.abs(cs.proj[0].area - Math.PI * 25) < 12, `frontal ${cs.proj[0].area}`);
  assert.ok(cs.stair.fraction > 0.7);
  // interpolated walls: (almost) every boundary link finds the surface, q in (0, 1];
  // links whose two ends were classified on the same side keep plain bounce-back
  assert.equal(cs.wall, 'interpolated');
  assert.ok(cs.linkStats.hits >= 0.9 * cs.linkStats.links, `${cs.linkStats.hits}/${cs.linkStats.links}`);
  assert.ok(cs.linkStats.qMin > 0 && cs.linkStats.qMax <= 1);
  assert.equal(cs.kappa[0], 1);
  // half-way bounce-back gets the empirical staircase correction instead
  const bb = buildCase(pos, { dof: 'surge', N: 10, ellRef: 0.1, wall: 'bounce-back' });
  assert.ok(bb.kappa[0] < 0.95 && bb.kappa[0] > 0.7 && bb.q === null);
});

test('grid convergence recovers a second-order solution and its uncertainty', async () => {
  const { gridConvergence } = await import('../src/cfd/study.js');
  // phi(h) = 1.09 + 30 h^2, grids N = 32, 23, 16
  const pts = [32, 23, 16].map((N) => ({ h: 1 / N, v: 1.09 + 30 / (N * N) }));
  const g = gridConvergence(pts);
  assert.equal(g.status, 'monotonic');
  assert.ok(Math.abs(g.p - 2) < 1e-3, `p ${g.p}`);
  assert.ok(Math.abs(g.value - 1.09) < 1e-4, `extrapolated ${g.value}`);
  assert.ok(g.gci > 0 && g.gci < 0.05);
  // oscillating grids are not extrapolated
  const o = gridConvergence([{ h: 1 / 32, v: 1.1 }, { h: 1 / 23, v: 1.05 }, { h: 1 / 16, v: 1.12 }]);
  assert.equal(o.status, 'oscillatory');
  assert.equal(o.value, 1.1);
});
