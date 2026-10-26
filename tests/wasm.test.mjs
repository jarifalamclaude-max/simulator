// The WebAssembly kernels must reproduce the JavaScript reference solvers.
import test from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { initKernels, kernels, disableKernels } from '../src/wasm/kernels.js';
import { solveAddedMass } from '../src/hydro/bem.js';
import { LBMCPU } from '../src/cfd/cpu.js';
import { geq } from '../src/cfd/d3q19.js';
import { ellipsoid } from '../src/geometry/primitives.js';
import { bemHull } from '../src/hydro/hull.js';

initKernels(readFileSync(new URL('../src/wasm/kernels.wasm', import.meta.url)), { sync: true });

test('kernels load', () => { assert.ok(kernels(), 'WebAssembly kernels'); });

test('panel method: WebAssembly matches JavaScript', () => {
  const pos = Float32Array.from(ellipsoid([], [0, 0, 0], [0.3, 0.15, 0.1], 24, 36));
  const { panels } = bemHull(pos, { targetPanels: 500 });
  const js = solveAddedMass(panels, { rho: 1000, wasm: false });
  const wa = solveAddedMass(panels, { rho: 1000 });
  assert.equal(wa.engine, 'wasm');
  assert.equal(js.engine, 'js');
  for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
    const scale = Math.max(1e-9, Math.abs(js.M[i][i]), Math.abs(js.M[j][j]));
    assert.ok(Math.abs(wa.M[i][j] - js.M[i][j]) / scale < 2e-5, `M[${i}][${j}] ${wa.M[i][j]} vs ${js.M[i][j]}`);
  }
});

test('lattice Boltzmann: WebAssembly matches JavaScript (bounce-back and Bouzidi, spinning)', () => {
  const nx = 30, ny = 18, nz = 16;
  const mask = new Uint8Array(nx * ny * nz);
  for (let z = 5; z < 11; z++) for (let y = 6; y < 12; y++) for (let x = 8; x < 14; x++) mask[x + nx * (y + ny * z)] = 1;
  const q = new Uint8Array(nx * ny * nz * 19);
  let seed = 3;
  const C = [[0, 0, 0], [1, 0, 0], [-1, 0, 0], [0, 1, 0], [0, -1, 0], [0, 0, 1], [0, 0, -1], [1, 1, 0], [-1, -1, 0], [1, -1, 0], [-1, 1, 0], [1, 0, 1], [-1, 0, -1], [1, 0, -1], [-1, 0, 1], [0, 1, 1], [0, -1, -1], [0, 1, -1], [0, -1, 1]];
  for (let z = 1; z < nz - 1; z++) for (let y = 1; y < ny - 1; y++) for (let x = 1; x < nx - 1; x++) {
    const id = x + nx * (y + ny * z);
    if (mask[id]) continue;
    for (let i = 1; i < 19; i++) if (mask[(x - C[i][0]) + nx * ((y - C[i][1]) + ny * (z - C[i][2]))]) { seed = (seed * 16807) % 2147483647; q[id * 19 + i] = 1 + (seed % 255); }
  }
  for (const withQ of [false, true]) {
    const p = { tau0: 0.54, smag: 0.12, uInf: [0.05, 0.003, 0], omega: [0.001, 0, 0.002], ref: [10.5, 8.5, 7.5], spongeFrac: 0.2, spongeAmp: 0.5 };
    const js = new LBMCPU({ nx, ny, nz, mask, q: withQ ? q : null }, p);
    const wa = new LBMCPU({ nx, ny, nz, mask, q: withQ ? q : null }, { ...p, useWasm: true });
    assert.ok(wa.wasm && !js.wasm);
    js.step(40); wa.step(40);
    let md = 0;
    for (let k = 0; k < js.A.length; k++) md = Math.max(md, Math.abs(js.A[k] - wa.A[k]));
    assert.ok(md < 1e-6, `state difference ${md}`);
    const fj = js.force(), fw = wa.force();
    for (let k = 0; k < 3; k++) assert.ok(Math.abs(fj.F[k] - fw.F[k]) < 1e-5 * Math.max(1, Math.abs(fj.F[0])), `F ${fj.F} vs ${fw.F}`);
  }
});

test('free stream stays uniform in the WebAssembly solver', () => {
  const nx = 12, ny = 8, nz = 8;
  const s = new LBMCPU({ nx, ny, nz, mask: new Uint8Array(nx * ny * nz) }, { tau0: 0.56, uInf: [0.06, 0, 0.01], useWasm: true });
  for (let i = 0; i < 19; i++) s.A.fill(geq(i, 1, 0.06, 0, 0.01), i * s.N, (i + 1) * s.N);
  s.step(25);
  let e = 0;
  for (let id = 0; id < s.N; id++) { const m = s.macro(id); e = Math.max(e, Math.abs(m[0] - 0.06), Math.abs(m[2] - 0.01)); }
  assert.ok(e < 2e-6, `deviation ${e}`);
  disableKernels();
});
