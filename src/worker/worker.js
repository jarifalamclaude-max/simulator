// Background worker: geometry analysis, BEM added mass, step tests, auto-tune
// and the CFD lab (lattice building, CPU lattice-Boltzmann runs) run here so
// the 3D view stays smooth.

import { analyzeGeometry } from '../geometry/analyze.js';
import { runBEM, runBEMConverged } from '../hydro/hull.js';
import { runStepTest, autoTune } from '../sim/tuning.js';
import { buildCase, runCPU } from '../cfd/study.js';
import WASM_B64 from 'virtual:wasm-kernels';
import { initKernels, base64ToBytes } from '../wasm/kernels.js';

// heavy kernels in WebAssembly (synchronous compile is fine in a worker)
initKernels(base64ToBytes(WASM_B64), { sync: true });

function progress(id, minMs = 60) {
  let last = 0;
  return (f, text, data) => {
    const now = Date.now();
    if (now - last < minMs && f < 1 && !(data && data.slice)) return;
    last = now;
    self.postMessage({ id, progress: f, text, data });
  };
}

self.onmessage = async (e) => {
  const { id, type, payload } = e.data;
  const onProgress = progress(id);
  try {
    let result, transfer = [];
    if (type === 'analyze') {
      result = analyzeGeometry(payload.positions, { ...payload.opts, onProgress });
    } else if (type === 'bem') {
      result = runBEM(payload, onProgress);
    } else if (type === 'bemconv') {
      result = runBEMConverged(payload, onProgress);
    } else if (type === 'steptest') {
      result = runStepTest(payload.cfg, payload.geo, payload.opts);
    } else if (type === 'autotune') {
      result = await autoTune(payload.cfg, payload.geo, { ...payload.opts, onProgress });
    } else if (type === 'cfdcase') {
      result = buildCase(payload.positions, payload.opts);
      transfer = [result.mask.buffer, ...(result.q ? [result.q.buffer] : [])];
    } else if (type === 'cfdrun') {
      const p = progress(id, 250);
      result = runCPU(payload.cs, payload.Re, { sliceEvery: 1200, snapshot: !!payload.snapshot, onProgress: (m) => p(m.fraction, null, m) });
      if (result.field) transfer = [result.field.data.buffer];
    } else throw new Error('Unknown job ' + type);
    self.postMessage({ id, result }, transfer);
  } catch (err) {
    self.postMessage({ id, error: err && err.message ? err.message : String(err) });
  }
};
