// Runs heavy jobs in a Web Worker (built from an inlined bundle), with an
// in-page fallback when workers are unavailable or fail to start.

import WORKER_SOURCE from 'virtual:worker-source';
import { analyzeGeometry } from '../geometry/analyze.js';
import { runStepTest, autoTune } from '../sim/tuning.js';
import { runBEM, runBEMConverged } from '../hydro/hull.js';
import { buildCase, runCPU } from '../cfd/study.js';

let workerUrl = null;
function makeWorker() {
  if (!workerUrl) workerUrl = URL.createObjectURL(new Blob([WORKER_SOURCE], { type: 'text/javascript' }));
  return new Worker(workerUrl);
}

export class Jobs {
  constructor() {
    this.pending = new Map();
    this.seq = 0;
    try {
      this.worker = makeWorker();
      this.worker.onmessage = (e) => this.onMessage(e.data);
      this.worker.onerror = (e) => this.fail(e);
    } catch (err) {
      console.warn('Workers unavailable, running jobs in the page', err);
      this.worker = null;
    }
  }
  // a worker that cannot start (CSP, script error): finish its jobs in the page
  fail(e) {
    console.warn('Worker failed, switching to in-page jobs:', e?.message || e);
    try { this.worker?.terminate(); } catch { /* ignore */ }
    this.worker = null;
    const jobs = [...this.pending.values()];
    this.pending.clear();
    for (const p of jobs) this.inline(p.type, p.payload, p.onProgress).then(p.resolve, p.reject);
  }
  onMessage(m) {
    const p = this.pending.get(m.id);
    if (!p) return;
    if (m.progress !== undefined) { p.onProgress?.(m.progress, m.text, m.data); return; }
    this.pending.delete(m.id);
    if (m.error) p.reject(new Error(m.error)); else p.resolve(m.result);
  }
  run(type, payload, onProgress) {
    if (!this.worker) return this.inline(type, payload, onProgress);
    const id = ++this.seq;
    return new Promise((resolve, reject) => {
      this.pending.set(id, { resolve, reject, onProgress, type, payload });
      this.worker.postMessage({ id, type, payload });
    });
  }
  // long job in its own worker so it can be cancelled without touching others
  runDedicated(type, payload, onProgress) {
    let w;
    try { w = makeWorker(); } catch { const pr = this.inline(type, payload, onProgress); return { promise: pr, cancel() {} }; }
    let done = false, rejectFn;
    const promise = new Promise((resolve, reject) => {
      rejectFn = reject;
      w.onmessage = (e) => {
        const m = e.data;
        if (m.progress !== undefined) { onProgress?.(m.progress, m.text, m.data); return; }
        done = true; w.terminate();
        if (m.error) reject(new Error(m.error)); else resolve(m.result);
      };
      w.onerror = (e) => { if (done) return; done = true; w.terminate(); reject(new Error(e.message || 'Worker error')); };
      w.postMessage({ id: 1, type, payload });
    });
    return { promise, cancel() { if (!done) { done = true; w.terminate(); rejectFn(new Error('cancelled')); } } };
  }
  async inline(type, payload, onProgress) {
    await new Promise((r) => setTimeout(r, 0));
    if (type === 'analyze') return analyzeGeometry(payload.positions, { ...payload.opts, onProgress });
    if (type === 'steptest') return runStepTest(payload.cfg, payload.geo, payload.opts);
    if (type === 'autotune') return autoTune(payload.cfg, payload.geo, { ...payload.opts, onProgress });
    if (type === 'bem') return runBEM(payload, onProgress);
    if (type === 'bemconv') return runBEMConverged(payload, onProgress);
    if (type === 'cfdcase') return buildCase(payload.positions, payload.opts);
    if (type === 'cfdrun') return runCPU(payload.cs, payload.Re, { onProgress: (m) => onProgress?.(m.fraction, null, m), maxSeconds: 600, snapshot: !!payload.snapshot });
    throw new Error('Unknown job ' + type);
  }
}
