// Exports the panel-method hulls and our added-mass results for the
// Capytaine cross-check (tools/bem-crosscheck.py).
//   node tools/bem-export.mjs out.json
import { writeFileSync, readFileSync } from 'node:fs';
import { bemHull } from '../src/hydro/hull.js';
import { solveAddedMass } from '../src/hydro/bem.js';
import { ellipsoid } from '../src/geometry/primitives.js';
import { sampleGeometry } from '../src/vehicle/sample.js';
import { ellipsoidAddedMass } from '../src/hydro/water.js';
import { initKernels } from '../src/wasm/kernels.js';

initKernels(readFileSync(new URL('../src/wasm/kernels.wasm', import.meta.url)), { sync: true });
const cases = [
  { name: 'ellipsoid 0.6 x 0.3 x 0.2 m', positions: Float32Array.from(ellipsoid([], [0, 0, 0], [0.3, 0.15, 0.1], 48, 72)), analytic: ellipsoidAddedMass(0.3, 0.15, 0.1, 1000).values },
  { name: 'DUBURI sample ROV hull', positions: sampleGeometry().positions },
];
const out = [];
for (const c of cases) {
  for (const target of [900, 1800, 3600]) {
    const { panels, quads } = bemHull(c.positions, { targetPanels: target });
    const t0 = Date.now();
    const r = solveAddedMass(panels, { rho: 1000 });
    // shared vertex list for the mesh (dedupe by position)
    const key = (v) => v.map((x) => x.toFixed(7)).join(',');
    const index = new Map(), verts = [], faces = [];
    for (const q of quads) {
      const f = q.map((v) => { const k = key(v); if (!index.has(k)) { index.set(k, verts.length); verts.push(v); } return index.get(k); });
      faces.push(f);
    }
    out.push({ name: c.name, targetPanels: target, panels: r.panels, seconds: (Date.now() - t0) / 1000, engine: r.engine, M: r.M, analytic: c.analytic || null, vertices: verts, faces });
    console.log(c.name, r.panels, 'panels', r.engine, ((Date.now() - t0) / 1000).toFixed(1), 's', r.M.map((row, i) => row[i].toFixed(4)).join(' '));
  }
}
writeFileSync(process.argv[2] || 'bem-crosscheck.json', JSON.stringify(out));
