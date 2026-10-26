// Drives one GPU lattice-Boltzmann run from the page's animation frames:
// a few solver steps per frame (adapted to keep the page responsive), forces
// read back asynchronously, convergence checked on the averaged forces.

import { caseSettings, applyRamp, convergence, windowMean, runCoefficients, primary, decimate, snapshotStride, eddyViscosityRatio, nearWallCells } from './study.js';
import { LBMGPU } from './gpu.js';

// At most one force readback completes per frame, so the steps per frame are
// capped to keep about a dozen samples in every averaging window (and the
// convergence test meaningful) on small lattices and fast GPUs.
export const perFrameCap = (st) => Math.max(st.sampleEvery, Math.floor((st.win * st.Tc) / 12));
export function nextPerFrame(perFrame, dtMs, st) {
  // keep the page near 30+ fps while the GPU works
  if (dtMs > 45) perFrame = Math.max(1, Math.floor(perFrame * 0.75));
  else if (dtMs < 26) perFrame = Math.ceil(perFrame * 1.15);
  return Math.min(perFrame, perFrameCap(st), 400);
}

export function runGPU(canvas, cs, Re, { onProgress = () => {}, onFrame = () => {}, signal, snapshot = false } = {}) {
  const st = caseSettings(cs, Re);
  const solver = new LBMGPU(canvas, { nx: cs.nx, ny: cs.ny, nz: cs.nz, mask: cs.mask, q: cs.q }, st.params);
  const cells = cs.nx * cs.ny * cs.nz;
  return new Promise((resolve, reject) => {
    const samples = [];
    let conv = { done: false };
    let perFrame = Math.max(1, Math.min(40, perFrameCap(st), Math.round(1.5e6 / cells)));
    let last = performance.now();
    const t0 = last;
    let lastReport = 0;
    const finish = (err) => {
      let field = null, nuT = null;
      if (!err && snapshot) { try { field = solver.snapshot(snapshotStride(cs)); } catch (e) { console.warn('flow snapshot failed', e); } }
      if (!err) {
        try {
          if (!samples.length) { const f = solver.force(); samples.push({ step: solver.steps, F: f.F, M: f.M }); }
          // near-wall LES eddy viscosity from the full state (one synchronous read)
          const S5 = solver.readState(), { nx, ny, TX, AW } = solver;
          nuT = eddyViscosityRatio((id, g) => {
            const x = id % nx, y = Math.floor(id / nx) % ny, z = Math.floor(id / (nx * ny));
            const t = 4 * ((z % TX) * nx + x + AW * (Math.floor(z / TX) * ny + y));
            for (let i = 0; i < 19; i++) g[i] = S5[i >> 2][t + (i & 3)];
          }, nearWallCells(cs), st.params.tau0, st.params.smag);
        } catch (e) { console.warn('LES diagnostic failed', e); }
      }
      try { solver.dispose(); } catch { /* ignore */ }
      if (err) { reject(err); return; }
      const end = samples[samples.length - 1].step;
      const W = st.win * st.Tc;
      const mean = windowMean(samples, end - W, end) ?? { ...samples[samples.length - 1], n: 1 };
      const sec = (performance.now() - t0) / 1000;
      resolve({ Re, mean, nuT, steps: solver.steps, converged: !!conv.converged, rel: conv.rel, C: runCoefficients(cs, st, mean), history: decimate(samples.map((s) => [s.step / st.Tc, primary(s, cs, st)]), 240), seconds: sec, mlups: (cells * solver.steps) / sec / 1e6, field });
    };
    const frame = () => {
      if (signal?.aborted) { finish(new Error('cancelled')); return; }
      try {
        const now = performance.now();
        const dt = now - last;
        last = now;
        perFrame = nextPerFrame(perFrame, dt, st);
        // readbacks that never complete must not keep the run alive past tMax
        if (solver.steps > 1.5 * st.tMax * st.Tc) { conv = { done: true, converged: false, rel: conv.rel }; onFrame(solver); finish(); return; }
        const r = solver.pollForce();
        if (r) {
          samples.push({ step: r.step, F: r.F, M: r.M });
          conv = convergence(samples, r.step, cs, st);
          if (now - lastReport > 150 || conv.done) {
            lastReport = now;
            onProgress({ step: r.step, fraction: Math.min(1, r.step / (st.tMax * st.Tc)), rel: conv.rel, value: primary(r, cs, st), t: r.step / st.Tc, mlups: (cells * solver.steps) / Math.max(1e-3, (now - t0) / 1000) / 1e6, perFrame });
          }
          if (conv.done) { onFrame(solver); finish(); return; }
        }
        let n = perFrame;
        while (n > 0) {
          const toSample = st.sampleEvery - (solver.steps % st.sampleEvery);
          const k = Math.min(n, toSample);
          applyRamp(solver, solver.steps, cs, st);
          solver.step(k);
          n -= k;
          if (solver.steps % st.sampleEvery === 0) solver.requestForce();
        }
        onFrame(solver);
        requestAnimationFrame(frame);
      } catch (e) { finish(e); }
    };
    requestAnimationFrame(frame);
  });
}
