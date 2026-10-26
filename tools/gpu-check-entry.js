// Browser entry used by tools/gpu-check.mjs: runs the same lattice case on the
// GPU and CPU solvers and reports the differences.
import { LBMGPU, gpuSupport } from '../src/cfd/gpu.js';
import { LBMCPU } from '../src/cfd/cpu.js';

window.gpuCheck = function gpuCheck({ steps = 60, rotate = true, interp = false } = {}) {
  const sup = gpuSupport();
  if (!sup.ok) return { ok: false, reason: sup.reason };
  const nx = 40, ny = 24, nz = 20;
  const mask = new Uint8Array(nx * ny * nz);
  for (let z = 6; z < 14; z++) for (let y = 8; y < 16; y++) for (let x = 10; x < 18; x++) mask[x + nx * (y + ny * z)] = 1;
  mask[19 + nx * (12 + ny * 10)] = 1; // a lone voxel next to the block
  const p = { tau0: 0.53, smag: 0.12, uInf: [0.06, 0.004, -0.003], omega: rotate ? [0.001, -0.002, 0.003] : [0, 0, 0], ref: [13.5, 11.5, 9.5], spongeFrac: 0.2, spongeAmp: 0.5 };
  let q = null;
  if (interp) {
    // pseudo-random wall distances on every boundary link
    q = new Uint8Array(nx * ny * nz * 19);
    let seed = 12345;
    const rnd = () => { seed = (seed * 1103515245 + 12345) & 0x7fffffff; return seed / 0x7fffffff; };
    const C = [[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1], [1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1], [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1]];
    for (let z = 1; z < nz - 1; z++) for (let y = 1; y < ny - 1; y++) for (let x = 1; x < nx - 1; x++) {
      const id = x + nx * (y + ny * z);
      if (mask[id]) continue;
      for (let i = 1; i < 19; i++) if (mask[(x - C[i][0]) + nx * ((y - C[i][1]) + ny * (z - C[i][2]))]) q[id * 19 + i] = 1 + Math.floor(rnd() * 255);
    }
  }
  const cpu = new LBMCPU({ nx, ny, nz, mask, q }, p);
  const canvas = document.createElement('canvas');
  const gpu = new LBMGPU(canvas, { nx, ny, nz, mask, q }, p);
  const t0 = performance.now();
  gpu.step(steps);
  const fg = gpu.force();
  const tg = performance.now() - t0;
  cpu.step(steps);
  const fc = cpu.force();
  // compare full state
  const st = gpu.readState();
  let maxDiff = 0, maxVal = 0;
  for (let z = 0; z < nz; z++) for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
    const id = x + nx * (y + ny * z);
    if (mask[id]) continue;
    const ax = (z % gpu.TX) * nx + x, ay = Math.floor(z / gpu.TX) * ny + y;
    for (let i = 0; i < 19; i++) {
      const gv = st[i >> 2][4 * (ax + gpu.AW * ay) + (i & 3)];
      const cv = cpu.A[i * cpu.N + id];
      maxDiff = Math.max(maxDiff, Math.abs(gv - cv));
      maxVal = Math.max(maxVal, Math.abs(cv));
    }
  }
  const rel = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) / Math.max(1e-12, Math.hypot(...b));
  return { ok: true, interp, renderer: sup.renderer, maxDraw: sup.maxDraw, steps, gpuMs: tg, F: { gpu: fg.F, cpu: fc.F, rel: rel(fg.F, fc.F) }, M: { gpu: fg.M, cpu: fc.M, rel: rel(fg.M, fc.M) }, state: { maxDiff, maxVal } };
};
