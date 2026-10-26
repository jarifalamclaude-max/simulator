// Smooth closed hull for the panel method, extracted from the voxelised STL
// (robust to open shells, overlapping parts and thin plates).

import { bbox, makeGrid, occupancyField, surfaceNets, quadPanels } from '../geometry/mesh.js';
import { solveAddedMass } from './bem.js';
import { gridConvergence } from '../core/convergence.js';

// The hull is meshed at the voxel size that gives about targetPanels panels
// (panel count scales with 1/cell^2, so a few secant steps are enough).
export function bemHull(positions, { targetPanels = 1800, onProgress } = {}) {
  const bb = bbox(positions);
  const L = Math.max(...bb.size);
  let cell = L / 26;
  let best = null;
  for (let attempt = 0; attempt < 7; attempt++) {
    const g = makeGrid(bb, cell, 2);
    const f = occupancyField(positions, g, { keepThin: true });
    const quads = surfaceNets(f, g, 0.5);
    const panels = quadPanels(quads);
    onProgress?.(0.02 + attempt * 0.015, `Meshing hull (${panels.length} panels)`);
    const err = Math.abs(Math.log(panels.length / targetPanels));
    if (!best || err < best.err) best = { panels, quads, cell, err };
    if (panels.length >= targetPanels * 0.85 && panels.length <= targetPanels * 1.2) break;
    cell = Math.max(L / 400, cell * Math.sqrt(panels.length / targetPanels));
  }
  return { panels: best.panels, quads: best.quads, cell: best.cell };
}

export function runBEM(payload, onProgress) {
  const hull = bemHull(payload.positions, { targetPanels: payload.targetPanels ?? 1800, onProgress });
  const res = solveAddedMass(hull.panels, { rho: payload.rho, ref: payload.ref ?? [0, 0, 0], onProgress: (f, t) => onProgress?.(0.15 + 0.85 * f, t) });
  const tri = [];
  for (const q of hull.quads) for (const v of [q[0], q[1], q[2], q[0], q[2], q[3]]) tri.push(v[0], v[1], v[2]);
  return { M: res.M, ref: res.ref, panels: res.panels, asymmetry: res.asymmetry, iterations: res.iterations, cell: hull.cell, hull: new Float32Array(tri) };
}

/**
 * Panel-converged added mass: the panel method on three hull resolutions,
 * each matrix element Richardson-extrapolated in the panel size
 * (h ~ 1/sqrt(panels)), with the grid convergence index of every diagonal
 * term as its uncertainty.
 */
export function runBEMConverged(payload, onProgress) {
  const levels = payload.levels ?? [900, 1800, 3600];
  const runs = [];
  for (let k = 0; k < levels.length; k++) {
    runs.push(runBEM({ ...payload, targetPanels: levels[k] }, (f, t) => onProgress?.((k + f) / levels.length, `${t || 'Working'} · ${k + 1}/${levels.length}`)));
  }
  runs.sort((a, b) => b.panels - a.panels); // finest first
  const fine = runs[0];
  const scale = Math.max(...fine.M.map((r, i) => Math.abs(r[i])), 1e-12);
  const M = fine.M.map((r) => r.slice());
  const unc = [0, 0, 0, 0, 0, 0], status = ['', '', '', '', '', ''], order = [0, 0, 0, 0, 0, 0];
  for (let l = 0; l < 6; l++) for (let j = 0; j < 6; j++) {
    const pts = runs.map((r) => ({ h: 1 / Math.sqrt(r.panels), v: r.M[l][j] }));
    const g = gridConvergence(pts);
    const ok = g.status === 'monotonic' || g.status === 'converged';
    if (ok && (l === j || Math.abs(fine.M[l][j]) > 1e-3 * scale)) M[l][j] = g.value;
    if (l === j) { unc[l] = g.gci; status[l] = g.status; order[l] = g.p; }
  }
  // keep it symmetric
  for (let l = 0; l < 6; l++) for (let j = l + 1; j < 6; j++) { const v = 0.5 * (M[l][j] + M[j][l]); M[l][j] = v; M[j][l] = v; }
  onProgress?.(1, 'Done');
  return { M, ref: fine.ref, panels: fine.panels, levels: runs.map((r) => r.panels), uncertainty: unc, status, order, perLevel: runs.map((r) => ({ panels: r.panels, diag: r.M.map((row, i) => row[i]) })), asymmetry: fine.asymmetry, hull: fine.hull, cell: fine.cell };
}
