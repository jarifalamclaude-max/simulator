// Auto-tune and CFD study robustness: run termination, grid convergence order,
// dissipative damping, step tests in the controller's own frame, ArduSub ranges.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { sampleGeometry } from '../src/vehicle/sample.js';
import { analyzeGeometry } from '../src/geometry/analyze.js';
import { defaultConfig, applyParamFile } from '../src/vehicle/config.js';
import { parseParamFile } from '../src/ardusub/params.js';
import { waterDensity } from '../src/hydro/water.js';
import { box } from '../src/geometry/primitives.js';
import { gridConvergence } from '../src/core/convergence.js';
import { buildCase, caseSettings, convergence, runCPU, assemble, dissipative, DOFS } from '../src/cfd/study.js';
import { nextPerFrame, perFrameCap } from '../src/cfd/runner.js';
import { runStepTest, autoTune, paramRange, AP_RANGE, STEP_AXES } from '../src/sim/tuning.js';

const geo = analyzeGeometry(sampleGeometry().positions);
const duburi = parseParamFile(readFileSync(new URL('../profiles/duburi_pixhawk_param.param', import.meta.url), 'utf8')).params;
function makeCfg() {
  const cfg = defaultConfig();
  applyParamFile(cfg, duburi, { syncPolarity: true });
  cfg.mixer.source = 'frame';
  cfg.mass.mass = waterDensity(cfg.environment.temperature) * geo.volume * 0.985;
  return cfg;
}

// symmetric eigenvalues by Jacobi (small test helper)
function minEig(A) {
  const n = A.length, M = A.map((r) => r.slice());
  for (let s = 0; s < 80; s++) for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
    if (Math.abs(M[p][q]) < 1e-300) continue;
    const th = (M[q][q] - M[p][p]) / (2 * M[p][q]), t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1)), c = 1 / Math.sqrt(t * t + 1), sn = t * c;
    for (let k = 0; k < n; k++) { const a = M[k][p], b = M[k][q]; M[k][p] = c * a - sn * b; M[k][q] = sn * a + c * b; }
    for (let k = 0; k < n; k++) { const a = M[p][k], b = M[q][k]; M[p][k] = c * a - sn * b; M[q][k] = sn * a + c * b; }
  }
  return Math.min(...M.map((r, i) => r[i]));
}
const sym = (D) => D.map((r, i) => r.map((v, j) => 0.5 * (v + D[j][i])));

test('convergence() ends every run at tMax, however sparse the force samples', () => {
  const cs = { kind: 't', N: 12, R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], ref: [0, 0, 0], sign: 1, axis: 0 };
  const st = caseSettings(cs, 2000);
  // one sample per 400 steps (the old GPU runner on a fast GPU): never 4 per window
  const samples = [];
  let res = { done: false };
  for (let step = 400; step < 1e6 && !res.done; step += 400) {
    samples.push({ step, F: [1 + 0.01 * Math.sin(step), 0, 0], M: [0, 0, 0] });
    res = convergence(samples, step, cs, st);
    if (res.done) assert.ok(step <= st.tMax * st.Tc + 400, `stopped at ${step}`);
  }
  assert.ok(res.done);
  assert.equal(res.converged, false);
});

test('GPU runner keeps enough samples per window on coarse grids (runner replay)', () => {
  for (const N of [8, 12, 17, 23, 48]) for (const kind of ['t', 'r']) {
    const cs = { kind, N, R: [[1, 0, 0], [0, 1, 0], [0, 0, 1]], ref: [0, 0, 0], sign: 1, axis: 0 };
    const st = caseSettings(cs, 2000);
    st.omegaDir = [1, 0, 0];
    let steps = 0, pending = null, perFrame = 40, conv = { done: false }, frames = 0;
    const samples = [];
    while (!conv.done && frames++ < 1e5) {
      perFrame = nextPerFrame(perFrame, 10, st); // a fast GPU: the old code climbed to 400
      assert.ok(perFrame <= perFrameCap(st));
      if (pending) { samples.push({ step: pending, F: [1, 0, 0], M: [1, 0, 0] }); conv = convergence(samples, pending, cs, st); pending = null; if (conv.done) break; }
      for (let n = perFrame; n > 0;) { const k = Math.min(n, st.sampleEvery - (steps % st.sampleEvery)); steps += k; n -= k; if (steps % st.sampleEvery === 0 && !pending) pending = steps; }
    }
    assert.ok(conv.done && conv.converged, `N ${N} ${kind}`);
    assert.ok(steps <= st.tMax * st.Tc, `N ${N} ${kind}: ${steps} steps`);
  }
});

test('CPU CFD run on a coarse grid terminates and reports the LES eddy viscosity', () => {
  const pos = Float32Array.from(box([], [0, 0, 0], [0.5, 0.3, 0.25]));
  const cs = buildCase(pos, { dof: 'surge', sign: 1, N: 6, ellRef: 0.5, cg: [0, 0, 0] });
  const st = caseSettings(cs, 2000);
  const r = runCPU(cs, 2000);
  assert.ok(r.steps <= st.tMax * st.Tc + st.sampleEvery, `${r.steps} steps`);
  assert.ok(r.C.every(Number.isFinite));
  assert.ok(r.C[0] < 0, 'drag opposes the tow');
  assert.ok(r.nuT && Number.isFinite(r.nuT.mean) && r.nuT.mean >= 0 && r.nuT.max >= r.nuT.mean);
});

test('GCI: extrapolation order clamped to the formal order, two-grid safety factor 3', () => {
  // phi(h) = 1 + 40 h^3: observed order 3, extrapolated with p = 2 (conservative)
  const pts = [32, 23, 16].map((N) => ({ h: 1 / N, v: 1 + 40 / N ** 3 }));
  const g = gridConvergence(pts);
  assert.ok(Math.abs(g.p - 3) < 1e-3, `p ${g.p}`);
  assert.equal(g.pUsed, 2);
  const r = 32 / 23, f1 = pts[0].v, f2 = pts[1].v;
  assert.ok(Math.abs(g.value - (r * r * f1 - f2) / (r * r - 1)) < 1e-12);
  assert.ok(Math.abs(g.gci - (1.25 * Math.abs((f1 - f2) / f1)) / (r * r - 1)) < 1e-12);
  // first-order scheme: pFormal 1
  assert.equal(gridConvergence(pts, { pFormal: 1 }).pUsed, 1);
  // two grids: Fs = 3
  const two = gridConvergence(pts.slice(0, 2));
  assert.equal(two.status, 'two grids');
  assert.ok(Math.abs(two.gci - (3 * Math.abs((f1 - f2) / f1)) / (r * r - 1)) < 1e-12);
});

test('damping assembly cannot inject energy (indefinite-matrix guard)', () => {
  const rho = 1000, nu = 1e-6, L = 0.5;
  // surge drag that "grows" with Re (K > 0, rejected) with a large 1/Re sway coupling
  const runs = [1, -1].flatMap((sign) => [500, 2000].map((Re) => ({ dof: 'surge', sign, Re, C: [-0.3 + 20 / Re, 0.2 - 400 / Re, 0, 0, 0, 0] })));
  runs.push(...[1, -1].flatMap((sign) => [500, 2000].map((Re) => ({ dof: 'sway', sign, Re, C: [0, -0.4 - 30 / Re, 0, 0, 0, 0] }))));
  const out = assemble(runs, { rho, nu, ellRef: L });
  // the rejected surge fit drops the Re term of the whole column
  assert.equal(out.details.surge.K[1], 0);
  assert.ok(out.DL[1][0] === 0);
  // both matrices dissipative
  assert.ok(minEig(sym(out.DL)) > -1e-12, `DL ${minEig(sym(out.DL))}`);
  assert.ok(minEig(sym(out.DQ.map((r, i) => r.map((v, j) => v / ((i < 3 ? 1 : L) * (j < 3 ? 1 : L)))))) > -1e-9);
  // a strongly indefinite matrix is projected; a PSD one is left alone
  const bad = [[1, 3, 0, 0, 0, 0], [3, 1, 0, 0, 0, 0], [0, 0, 1, 0, 0, 0], [0, 0, 0, 1, 0, 0], [0, 0, 0, 0, 1, 0], [0, 0, 0, 0, 0, 1]];
  const p = dissipative(bad, 1);
  assert.ok(p.clipped > 0 && minEig(sym(p.D)) > -1e-12);
  const good = dissipative(out.DL, L);
  assert.equal(good.clipped, 0);
  // couplings from one tow direction are not kept (even and odd parts inseparable)
  const one = assemble([500, 2000].map((Re) => ({ dof: 'surge', sign: 1, Re, C: [-0.3 - 50 / Re, 0, 0.05, 0, 0, 0] })), { rho, nu, ellRef: L });
  assert.ok(one.DQ[2][0] === 0);
  assert.ok(one.DQ[0][0] > 0);
  // spin damping carries the frozen-geometry model uncertainty
  const spin = assemble([500, 2000].map((Re) => ({ dof: 'yaw', sign: 1, Re, C: [0, 0, 0, 0, 0, -0.02 - 4 / Re] })), { rho, nu, ellRef: L });
  assert.ok(spin.details.yaw.approximation && spin.details.yaw.uncertainty >= 0.3);
  assert.equal(DOFS.length, 6);
});

test('LES eddy viscosity enters the Reynolds fit when the grids are not extrapolated', () => {
  const rho = 1000, nu = 1e-6, L = 0.5;
  const C = (Re) => [-0.3 - 50 / Re, 0, 0, 0, 0, 0];
  // runs at nominal Re 500 / 2000 that really saw nu (1 + r)
  const runs = [[500, 0.1], [2000, 0.6]].map(([Re, r]) => ({ dof: 'surge', sign: 1, Re, N: 16, C: C(Re / (1 + r)), nuT: { mean: r, max: 3 * r } }));
  const out = assemble(runs, { rho, nu, ellRef: L });
  assert.ok(Math.abs(out.details.surge.K[0] + 50) < 1e-9, `K ${out.details.surge.K[0]}`);
  assert.ok(Math.abs(out.details.surge.Cinf[0] + 0.3) < 1e-12);
  assert.ok(out.warnings.some((w) => /eddy viscosity/.test(w)));
});

test('depth step is commanded and scored in the baro / inertial-nav frame (seawater)', () => {
  const cfg = makeCfg();
  cfg.environment.salinity = 35; // BARO_SPEC_GRAV stays 1: baro depth is ~2.6% off true depth
  const r = runStepTest(cfg, geo, { axis: 'depth', noise: false });
  assert.ok(Math.abs(r.yf - r.y0 - STEP_AXES.depth.size) < 1e-9);
  assert.ok(Math.abs(r.target[r.target.length - 1] - r.yf) < 1e-3, 'target in the measured frame');
  assert.ok(Math.abs(r.metrics.steadyError) < STEP_AXES.depth.band, `steady error ${r.metrics.steadyError}`);
  assert.ok(r.metrics.settlingTime < r.t[r.t.length - 1] - 0.1, 'settles inside the window');
});

test('attitude step is scored on the AHRS (AHRS_TRIM does not count as error)', () => {
  const cfg = makeCfg();
  assert.ok(Math.abs(cfg.params.AHRS_TRIM_Y) > 0.01, 'profile has a pitch trim');
  const r = runStepTest(cfg, geo, { axis: 'pitch', noise: false });
  assert.ok(Math.abs(r.metrics.steadyError) < 0.6, `steady error ${r.metrics.steadyError}`);
  // an unstable pitch loop is flagged even though Euler pitch stops at 90 deg
  const bad = runStepTest(cfg, geo, { axis: 'pitch', noise: false, params: { ATC_RAT_PIT_P: -0.3, ATC_RAT_PIT_I: 0, ATC_RAT_PIT_D: 0 } });
  assert.equal(bad.metrics.diverged, true);
});

test('auto-tune suggests gains inside the ArduSub ranges and never touches the config', async () => {
  const cfg = makeCfg();
  const before = JSON.stringify(cfg.params);
  const res = await autoTune(cfg, geo, { axes: ['roll', 'depth'], quick: true });
  assert.equal(JSON.stringify(cfg.params), before);
  for (const axis of ['roll', 'depth']) {
    const rep = res.report[axis];
    assert.ok(!rep.skipped, `${axis}: ${rep.skipped}`);
    for (const [k, v] of Object.entries(rep.suggested)) {
      const r = paramRange(k);
      if (r && !(v === 0 && /_FLT[TED]$/.test(k))) assert.ok(v >= r[0] - 1e-12 && v <= r[1] + 1e-12, `${k} ${v} outside ${r}`);
    }
    assert.ok(Number.isFinite(rep.beforeCost) && Number.isFinite(rep.afterCost));
    if (!rep.keep) assert.ok(rep.afterCost < rep.beforeCost);
  }
  // the rate loop, angle loop and acceleration limit of the axis are all covered
  for (const k of ['ATC_RAT_RLL_P', 'ATC_RAT_RLL_I', 'ATC_RAT_RLL_D', 'ATC_ANG_RLL_P', 'ATC_ACCEL_R_MAX']) assert.ok(k in res.report.roll.suggested, k);
  for (const k of ['PSC_ACCZ_P', 'PSC_ACCZ_I', 'PSC_VELZ_P', 'PSC_POSZ_P']) assert.ok(k in res.report.depth.suggested, k);
  assert.equal(AP_RANGE.ATC_ANG_YAW_P[1], 6);
  assert.ok(res.verification.roll && res.verification.depth);
});
