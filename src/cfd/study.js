// CFD tow-tank study: turns the hull into lattice cases, runs translation and
// rotation tests with the lattice-Boltzmann solver, and converts the measured
// forces into the 6 x 6 linear + quadratic damping matrices of the vehicle model.
//
// Scaling. A run at lattice speed V, viscosity nu_lat and body length N cells
// measures generalised forces G_i. They are made dimensionless,
//   C_i = G_i / (rho V^2 N^p_i)   (p = 2 for forces, 3 for moments),
// at two Reynolds numbers and fitted to C(Re) = C_inf + K / Re. At full scale
// (water viscosity, real speeds) this gives, for motion in DOF j,
//   DQ_ij = -rho s_j^2 L^p_i C_inf_i      DL_ij = -rho s_j L^(p_i-1) K_i nu
// with s_j = 1 for translation and s_j = L for rotation (V = omega L).
//
// Wall position correction. Half-way bounce-back on a voxel staircase acts
// about 0.4 cell outside the cell centres on curved surfaces; flat faces
// aligned with the lattice are not affected. Forces are scaled by
// A / (A + delta P) of the projected area, with delta = 0.4 cell times the
// measured staircase fraction of the voxel surface. Calibrated on spheres at
// Re 100 (8, 12 and 16 cells per diameter: drag within 3% of the standard
// drag curve, against +10 to +28% uncorrected); see tools/validate-cfd.mjs.

import { VoxelGrid, voxelizeSolid, voxelizeSurface, surfaceNets } from '../geometry/mesh.js';
import { LBMCPU } from './cpu.js';
import { Q, C, W, SMAG_K } from './d3q19.js';
import { gridConvergence } from '../core/convergence.js';

export const DOFS = [
  { key: 'surge', label: 'Surge', kind: 't', axis: 0 },
  { key: 'sway', label: 'Sway', kind: 't', axis: 1 },
  { key: 'heave', label: 'Heave', kind: 't', axis: 2 },
  { key: 'roll', label: 'Roll', kind: 'r', axis: 0 },
  { key: 'pitch', label: 'Pitch', kind: 'r', axis: 1 },
  { key: 'yaw', label: 'Yaw', kind: 'r', axis: 2 },
];
export const QUALITY = {
  quick: { label: 'Quick', N: 24, note: '24 cells along the vehicle' },
  standard: { label: 'Standard', N: 32, note: '32 cells along the vehicle' },
  fine: { label: 'Fine', N: 48, note: '48 cells along the vehicle (desktop GPU)' },
  ultra: { label: 'Ultra', N: 64, note: '64 cells along the vehicle (needs a strong GPU with 2 GB+)' },
  test: { label: 'Test', N: 8, note: 'smoke test only', hidden: true },
};
// grids of a convergence study: fine, medium, coarse (refinement ratio about 1.4)
export const gridLevels = (N, gci) => (gci ? [N, Math.round(N / Math.SQRT2), Math.round(N / 2)] : [N]);
export const RE_PAIR = [500, 2000];
export const RE_SINGLE = [1500];
export const V_TRANS = 0.08; // lattice speed of the tow (Mach 0.14)
export const V_ROT = 0.1;    // omega * L in lattice units
export const SMAG = 0.12;

// body -> lattice rotations (rows are lattice axes in body coordinates);
// the tow direction always maps to lattice -x so the water flows along +x
const ORIENT = {
  0: [[-1, 0, 0], [0, -1, 0], [0, 0, 1]],
  1: [[0, -1, 0], [1, 0, 0], [0, 0, 1]],
  2: [[0, 0, -1], [0, 1, 0], [1, 0, 0]],
};
export function orientation(dof, sign = 1) {
  const R = ORIENT[dof.kind === 't' ? dof.axis : 0].map((r) => r.slice());
  if (dof.kind === 't' && sign < 0) { R[0] = R[0].map((v) => -v); R[1] = R[1].map((v) => -v); }
  return R;
}
const mul = (R, v) => [0, 1, 2].map((i) => R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2]);
const mulT = (R, v) => [0, 1, 2].map((i) => R[0][i] * v[0] + R[1][i] * v[1] + R[2][i] * v[2]);

/**
 * Build a lattice case. positions: hull triangles in the vehicle frame (m).
 * opts: { dof, sign, N, ellRef, cg, margins? {up, down, side} (multiples of ellRef) }
 */
export function buildCase(positions, opts) {
  const dof = DOFS.find((d) => d.key === opts.dof);
  if (!dof) throw new Error('Unknown DOF ' + opts.dof);
  const sign = opts.sign ?? 1;
  const R = orientation(dof, sign);
  const cg = opts.cg ?? [0, 0, 0];
  const L = opts.ellRef, N = opts.N, cell = L / N;
  const P = new Float32Array(positions.length);
  const bmin = [Infinity, Infinity, Infinity], bmax = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    const d = [positions[i] - cg[0], positions[i + 1] - cg[1], positions[i + 2] - cg[2]];
    for (let k = 0; k < 3; k++) {
      const v = R[k][0] * d[0] + R[k][1] * d[1] + R[k][2] * d[2];
      P[i + k] = v;
      if (v < bmin[k]) bmin[k] = v;
      if (v > bmax[k]) bmax[k] = v;
    }
  }
  const m = { ...(dof.kind === 't' ? { up: 1.0, down: 2.5, side: 0.8 } : { up: 0.75, down: 0.75, side: 0.75 }), ...(opts.margins || {}) };
  const lo = [bmin[0] - m.up * L, bmin[1] - m.side * L, bmin[2] - m.side * L];
  const hi = [bmax[0] + m.down * L, bmax[1] + m.side * L, bmax[2] + m.side * L];
  const nx = Math.ceil((hi[0] - lo[0]) / cell), ny = Math.ceil((hi[1] - lo[1]) / cell), nz = Math.ceil((hi[2] - lo[2]) / cell);
  // voxelise only the body region (+2 cells), aligned with the domain lattice
  const i0 = [0, 1, 2].map((k) => Math.max(0, Math.floor((bmin[k] - lo[k]) / cell) - 2));
  const i1 = [0, 1, 2].map((k, q) => Math.min([nx, ny, nz][q] - 1, Math.ceil((bmax[k] - lo[k]) / cell) + 2));
  const sub = new VoxelGrid(i1[0] - i0[0] + 1, i1[1] - i0[1] + 1, i1[2] - i0[2] + 1, cell, [0, 1, 2].map((k) => lo[k] + i0[k] * cell));
  voxelizeSolid(P, sub);
  const surf = new VoxelGrid(sub.nx, sub.ny, sub.nz, cell, sub.origin.slice());
  voxelizeSurface(P, surf);
  // Cells whose centre is inside the hull are solid. Cells that only touch the
  // surface are added where the hull is thinner than a cell (plates, duct
  // walls) - those have no inside cell around them - plus the cells that join
  // such thin parts to thick ones, so no slit opens at the junction. Next to
  // thick parts they are left out: they would inflate the body by a cell.
  thinSurfaceCells(sub, surf);
  const mask = new Uint8Array(nx * ny * nz);
  let solid = 0;
  for (let k = 0; k < sub.nz; k++) for (let j = 0; j < sub.ny; j++) for (let i = 0; i < sub.nx; i++) {
    if (!sub.data[i + sub.nx * (j + sub.ny * k)]) continue;
    mask[(i + i0[0]) + nx * ((j + i0[1]) + ny * (k + i0[2]))] = 1;
    solid++;
  }
  if (!solid) throw new Error('The hull produced no solid cells at this resolution');
  const proj = [0, 1, 2].map((a) => projection(sub, a));
  const stair = staircaseFraction(sub);
  // interpolated (Bouzidi) walls: exact wall position on every boundary link
  const interp = opts.wall !== 'bounce-back';
  const links = interp ? linkDistances(P, mask, [nx, ny, nz], lo, cell, i0, i1) : null;
  // without them, correct the half-cell staircase shift empirically
  const delta = interp ? 0 : WALL_SHIFT * stair.fraction;
  const kappa = proj.map((p) => p.area / (p.area + delta * p.perimeter));
  const ref = [0, 1, 2].map((k) => (0 - lo[k]) / cell - 0.5);
  return {
    dof: dof.key, kind: dof.kind, axis: dof.axis, sign, R, N, ellRef: L, cell, origin: lo,
    nx, ny, nz, mask, solid, ref, proj, stair, kappa,
    q: links ? links.q : null, wall: interp ? 'interpolated' : 'bounce-back', linkStats: links ? links.stats : null,
    frontal: proj[0].area,
    blockage: dof.kind === 't' ? proj[0].area / (ny * nz) : 0,
  };
}

/**
 * Distance from each fluid node to the hull surface along every boundary
 * link, as a fraction q of the link (0..1], quantised to 1/255. Index
 * q[cell * 19 + i] is the link from the fluid cell towards cell - c_i (solid);
 * 0 means "no boundary link". Triangles are binned on the lattice for speed.
 */
export function linkDistances(P, mask, dims, origin, cell, i0, i1) {
  const [nx, ny, nz] = dims;
  const Ncell = nx * ny * nz;
  const q = new Uint8Array(Ncell * 19);
  const bx = i1[0] - i0[0] + 1, by = i1[1] - i0[1] + 1, bz = i1[2] - i0[2] + 1;
  const nb = bx * by * bz;
  const tri = P.length / 9;
  const binOf = (i, j, k) => (i - i0[0]) + bx * ((j - i0[1]) + by * (k - i0[2]));
  const cellRange = (t) => {
    const r = [];
    for (let a = 0; a < 3; a++) {
      const v0 = P[9 * t + a], v1 = P[9 * t + 3 + a], v2 = P[9 * t + 6 + a];
      const lo = Math.floor((Math.min(v0, v1, v2) - origin[a]) / cell), hi = Math.floor((Math.max(v0, v1, v2) - origin[a]) / cell);
      r.push(Math.max(i0[a], lo), Math.min(i1[a], hi));
    }
    return r;
  };
  // CSR bins
  const count = new Uint32Array(nb + 1);
  for (let t = 0; t < tri; t++) {
    const [a0, a1, b0, b1, c0, c1] = cellRange(t);
    for (let k = c0; k <= c1; k++) for (let j = b0; j <= b1; j++) for (let i = a0; i <= a1; i++) count[binOf(i, j, k) + 1]++;
  }
  for (let b = 0; b < nb; b++) count[b + 1] += count[b];
  const items = new Uint32Array(count[nb]);
  const fill = count.slice(0, nb);
  for (let t = 0; t < tri; t++) {
    const [a0, a1, b0, b1, c0, c1] = cellRange(t);
    for (let k = c0; k <= c1; k++) for (let j = b0; j <= b1; j++) for (let i = a0; i <= a1; i++) items[fill[binOf(i, j, k)]++] = t;
  }
  // Moller-Trumbore on the segment o + t d, t in (0, 1]
  const hit = (t, ox, oy, oz, dx, dy, dz) => {
    const p = 9 * t;
    const e1x = P[p + 3] - P[p], e1y = P[p + 4] - P[p + 1], e1z = P[p + 5] - P[p + 2];
    const e2x = P[p + 6] - P[p], e2y = P[p + 7] - P[p + 1], e2z = P[p + 8] - P[p + 2];
    const hx = dy * e2z - dz * e2y, hy = dz * e2x - dx * e2z, hz = dx * e2y - dy * e2x;
    const a = e1x * hx + e1y * hy + e1z * hz;
    if (Math.abs(a) < 1e-20) return Infinity;
    const f = 1 / a;
    const sx = ox - P[p], sy = oy - P[p + 1], sz = oz - P[p + 2];
    const u = f * (sx * hx + sy * hy + sz * hz);
    if (u < -1e-9 || u > 1 + 1e-9) return Infinity;
    const qx = sy * e1z - sz * e1y, qy = sz * e1x - sx * e1z, qz = sx * e1y - sy * e1x;
    const v = f * (dx * qx + dy * qy + dz * qz);
    if (v < -1e-9 || u + v > 1 + 1e-9) return Infinity;
    const tt = f * (e2x * qx + e2y * qy + e2z * qz);
    return tt > -0.6 && tt <= 1.6 ? tt : Infinity;
  };
  const stats = { links: 0, hits: 0, near: 0, qMin: 1, qMax: 0 };
  for (let k = Math.max(1, i0[2]); k <= Math.min(nz - 2, i1[2]); k++)
    for (let j = Math.max(1, i0[1]); j <= Math.min(ny - 2, i1[1]); j++)
      for (let i = Math.max(1, i0[0]); i <= Math.min(nx - 2, i1[0]); i++) {
        const id = i + nx * (j + ny * k);
        if (mask[id]) continue;
        for (let d = 1; d < 19; d++) {
          const c = C[d];
          const si = i - c[0], sj = j - c[1], sk = k - c[2];
          if (!mask[si + nx * (sj + ny * sk)]) continue;
          stats.links++;
          const ox = origin[0] + (i + 0.5) * cell, oy = origin[1] + (j + 0.5) * cell, oz = origin[2] + (k + 0.5) * cell;
          const dx = -c[0] * cell, dy = -c[1] * cell, dz = -c[2] * cell;
          // first surface crossing along the link (t in (0, 1]); thin parts can
          // have their surface just past the solid node (t up to 1.6: wall at
          // the solid node) or just behind the fluid node (t > -0.6: wall at it)
          let best = Infinity, beyond = Infinity, behind = -Infinity;
          for (let ck = Math.min(k, sk); ck <= Math.max(k, sk); ck++) for (let cj = Math.min(j, sj); cj <= Math.max(j, sj); cj++) for (let ci = Math.min(i, si); ci <= Math.max(i, si); ci++) {
            if (ci < i0[0] || cj < i0[1] || ck < i0[2] || ci > i1[0] || cj > i1[1] || ck > i1[2]) continue;
            const b = binOf(ci, cj, ck);
            for (let n = count[b]; n < count[b + 1]; n++) {
              const tt = hit(items[n], ox, oy, oz, dx, dy, dz);
              if (tt === Infinity) continue;
              if (tt > 1e-7 && tt <= 1 + 1e-7) { if (tt < best) best = tt; }
              else if (tt > 1) { if (tt < beyond) beyond = tt; }
              else if (tt > behind) behind = tt;
            }
          }
          let qv;
          if (best <= 1 + 1e-7) { qv = Math.min(1, best); stats.hits++; }
          else if (beyond < Infinity) { qv = 1; stats.near++; }
          else if (behind > -Infinity) { qv = 1 / 255; stats.near++; }
          else qv = 0.5; // no surface found near the link: plain bounce-back
          if (qv < stats.qMin) stats.qMin = qv;
          if (qv > stats.qMax) stats.qMax = qv;
          q[id * 19 + d] = Math.max(1, Math.min(255, Math.round(qv * 255)));
        }
      }
  return { q, stats };
}

function thinSurfaceCells(solid, surf) {
  const { nx, ny, nz } = solid;
  const S = solid.data, T = surf.data;
  const near = (i, j, k, set) => {
    for (let dk = -1; dk <= 1; dk++) for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
      const a = i + di, b = j + dj, c = k + dk;
      if (a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
      if (set[a + nx * (b + ny * c)]) return true;
    }
    return false;
  };
  const thin = new Uint8Array(S.length);
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const id = i + nx * (j + ny * k);
    if (T[id] && !S[id] && !near(i, j, k, S)) thin[id] = 1;
  }
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const id = i + nx * (j + ny * k);
    if (T[id] && !S[id] && !thin[id] && near(i, j, k, thin)) S[id] = 2; // bridge
  }
  for (let id = 0; id < S.length; id++) if (thin[id] || S[id]) S[id] = 1;
}

// projected area (cells) and outline length (cell edges) along lattice axis a
function projection(g, a) {
  const b = (a + 1) % 3, c = (a + 2) % 3;
  const n = [g.nx, g.ny, g.nz];
  const img = new Uint8Array(n[b] * n[c]);
  const ijk = [0, 0, 0];
  for (ijk[2] = 0; ijk[2] < g.nz; ijk[2]++) for (ijk[1] = 0; ijk[1] < g.ny; ijk[1]++) for (ijk[0] = 0; ijk[0] < g.nx; ijk[0]++) {
    if (g.data[ijk[0] + g.nx * (ijk[1] + g.ny * ijk[2])]) img[ijk[b] + n[b] * ijk[c]] = 1;
  }
  let area = 0, perimeter = 0;
  for (let v = 0; v < n[c]; v++) for (let u = 0; u < n[b]; u++) {
    if (!img[u + n[b] * v]) continue;
    area++;
    if (u === 0 || !img[u - 1 + n[b] * v]) perimeter++;
    if (u === n[b] - 1 || !img[u + 1 + n[b] * v]) perimeter++;
    if (v === 0 || !img[u + n[b] * (v - 1)]) perimeter++;
    if (v === n[c] - 1 || !img[u + n[b] * (v + 1)]) perimeter++;
  }
  return { area, perimeter };
}

// 0 for lattice-aligned flat faces, ~1 for a voxelised sphere: compares the
// exposed voxel-face area with a smooth (surface-nets) area of the same mask
export function staircaseFraction(g) {
  const field = new Float32Array(g.data.length);
  for (let i = 0; i < field.length; i++) field[i] = g.data[i];
  const unit = new VoxelGrid(g.nx, g.ny, g.nz, 1, [0, 0, 0]);
  const quads = surfaceNets(field, unit, 0.5);
  let smooth = 0;
  for (const q of quads) {
    const d1 = [0, 1, 2].map((k) => q[2][k] - q[0][k]), d2 = [0, 1, 2].map((k) => q[3][k] - q[1][k]);
    smooth += 0.5 * Math.hypot(d1[1] * d2[2] - d1[2] * d2[1], d1[2] * d2[0] - d1[0] * d2[2], d1[0] * d2[1] - d1[1] * d2[0]);
  }
  let faces = 0;
  const at = (i, j, k) => (i < 0 || j < 0 || k < 0 || i >= g.nx || j >= g.ny || k >= g.nz ? 0 : g.data[i + g.nx * (j + g.ny * k)]);
  for (let k = 0; k < g.nz; k++) for (let j = 0; j < g.ny; j++) for (let i = 0; i < g.nx; i++) {
    if (!at(i, j, k)) continue;
    faces += !at(i + 1, j, k) + !at(i - 1, j, k) + !at(i, j + 1, k) + !at(i, j - 1, k) + !at(i, j, k + 1) + !at(i, j, k - 1);
  }
  const ratio = faces ? smooth / faces : 1;
  // calibration: lattice-aligned boxes 0.89-0.97 (edge rounding), voxelised spheres 0.65-0.69
  const fraction = Math.min(1, Math.max(0, (STAIR_FLAT - ratio) / (STAIR_FLAT - STAIR_SPHERE)));
  return { ratio, fraction, faces, smooth };
}
export const WALL_SHIFT = 0.4;
export const STAIR_FLAT = 0.93;
export const STAIR_SPHERE = 0.675;

/** Solver parameters and run control for one case at one Reynolds number. */
export function caseSettings(cs, Re) {
  const V = cs.kind === 't' ? V_TRANS : V_ROT;
  const nu = (V * cs.N) / Re;
  const Tc = cs.N / V;
  let omegaDir = [0, 0, 0], omega = [0, 0, 0];
  if (cs.kind === 'r') {
    const e = [0, 0, 0]; e[cs.axis] = cs.sign;
    omegaDir = mul(cs.R, e);
    omega = omegaDir.map((v) => (v * V) / cs.N);
  }
  return {
    Re, V, nu, Tc,
    params: { tau0: 3 * nu + 0.5, smag: SMAG, uInf: [0, 0, 0], omega: [0, 0, 0], ref: cs.ref, spongeFrac: cs.kind === 't' ? 0.18 : 0, spongeAmp: 0.5 },
    omegaTarget: omega, omegaDir,
    rampT: 1.0,            // characteristic times to ramp up the tow / spin
    tMin: cs.kind === 't' ? 8 : 6,
    tMax: cs.kind === 't' ? 24 : 16,
    win: 3,                // averaging window (characteristic times)
    tol: 0.02,
    sampleEvery: Math.max(4, Math.round(Tc / 12)),
  };
}

// ---- LES diagnostic: Smagorinsky eddy viscosity next to the hull ----
// The Reynolds fit C(Re) = C_inf + K / Re needs the viscosity the runs really
// have; with the LES closure that is nu (1 + nu_t / nu). Fluid cells with a
// boundary link:
export function nearWallCells(cs) {
  const { nx, ny, nz, mask } = cs;
  const out = [];
  for (let z = 1; z < nz - 1; z++) for (let y = 1; y < ny - 1; y++) for (let x = 1; x < nx - 1; x++) {
    const id = x + nx * (y + ny * z);
    if (mask[id]) continue;
    for (let i = 1; i < Q; i++) if (mask[(x - C[i][0]) + nx * ((y - C[i][1]) + ny * (z - C[i][2]))]) { out.push(id); break; }
  }
  return Int32Array.from(out);
}
/**
 * Mean and max of nu_t / nu over `cells`, from post-collision populations.
 * get(id, g) fills g[0..18] with the shifted populations g = f - w of cell id.
 * The regularised collision keeps (1 - 1/tau) of the non-equilibrium stress, so
 * the pre-collision stress (which sets tau) follows by fixed-point iteration.
 */
export function eddyViscosityRatio(get, cells, tau0, smag) {
  const smagK = SMAG_K * smag * smag;
  if (!(smagK > 0) || !cells.length) return { mean: 0, max: 0, cells: cells.length };
  const g = new Float64Array(Q);
  let sum = 0, max = 0;
  for (const id of cells) {
    get(id, g);
    let rho = 1, jx = 0, jy = 0, jz = 0;
    for (let i = 0; i < Q; i++) { rho += g[i]; jx += C[i][0] * g[i]; jy += C[i][1] * g[i]; jz += C[i][2] * g[i]; }
    const ux = jx / rho, uy = jy / rho, uz = jz / rho, uu = ux * ux + uy * uy + uz * uz;
    let xx = 0, yy = 0, zz = 0, xy = 0, xz = 0, yz = 0;
    for (let i = 0; i < Q; i++) {
      const c = C[i], cu = c[0] * ux + c[1] * uy + c[2] * uz;
      const ne = g[i] - W[i] * (rho - 1 + rho * (3 * cu + 4.5 * cu * cu - 1.5 * uu));
      xx += c[0] * c[0] * ne; yy += c[1] * c[1] * ne; zz += c[2] * c[2] * ne; xy += c[0] * c[1] * ne; xz += c[0] * c[2] * ne; yz += c[1] * c[2] * ne;
    }
    const qPost = Math.sqrt(xx * xx + yy * yy + zz * zz + 2 * (xy * xy + xz * xz + yz * yz));
    let qn = qPost, tau = tau0;
    for (let k = 0; k < 30; k++) { tau = 0.5 * (tau0 + Math.sqrt(tau0 * tau0 + (smagK * qn) / rho)); qn = qPost / Math.abs(1 - 1 / tau); }
    tau = 0.5 * (tau0 + Math.sqrt(tau0 * tau0 + (smagK * qn) / rho));
    const r = (tau - tau0) / (tau0 - 0.5);
    sum += r; if (r > max) max = r;
  }
  return { mean: sum / cells.length, max, cells: cells.length };
}
export const LES_WARN = 0.2; // near-wall nu_t / nu above which the Reynolds fit is flagged

export function rampFactor(step, st) {
  const t = step / st.Tc;
  return t >= st.rampT ? 1 : 0.5 * (1 - Math.cos((Math.PI * t) / st.rampT));
}
export function applyRamp(solver, step, cs, st) {
  const r = rampFactor(step, st);
  if (cs.kind === 't') solver.setInflow([st.V * r, 0, 0]);
  else solver.p.omega = st.omegaTarget.map((v) => v * r);
}
export function primary(sample, cs, st) {
  if (cs.kind === 't') return sample.F[0];
  return sample.M[0] * st.omegaDir[0] + sample.M[1] * st.omegaDir[1] + sample.M[2] * st.omegaDir[2];
}
export function windowMean(samples, from, to) {
  const s = samples.filter((x) => x.step > from && x.step <= to);
  if (!s.length) return null;
  const F = [0, 0, 0], M = [0, 0, 0];
  for (const x of s) for (let k = 0; k < 3; k++) { F[k] += x.F[k] / s.length; M[k] += x.M[k] / s.length; }
  return { F, M, n: s.length };
}
// converged when the last two averaging windows agree within tol; a run always
// ends at tMax, however few force samples its windows hold (the GPU runner gets
// at most one readback per frame)
export function convergence(samples, step, cs, st) {
  const W = st.win * st.Tc;
  const a = step >= st.tMin * st.Tc ? windowMean(samples, step - W, step) : null, b = a ? windowMean(samples, step - 2 * W, step - W) : null;
  const ok = a && b && a.n >= 4 && b.n >= 4;
  const rel = ok ? Math.abs(primary(a, cs, st) - primary(b, cs, st)) / Math.max(Math.abs(primary(a, cs, st)), 1e-12) : NaN;
  if (step >= st.tMax * st.Tc) return { done: true, converged: ok && rel < st.tol, rel };
  if (!ok) return { done: false, rel: NaN };
  return { done: rel < st.tol, rel, converged: rel < st.tol };
}

/** Dimensionless body-frame coefficients (6) of a finished run, sign-normalised. */
export function runCoefficients(cs, st, mean) {
  const kap = cs.kind === 't' ? cs.kappa[0] : 0.5 * (cs.kappa[(cs.axis + 1) % 3] + cs.kappa[(cs.axis + 2) % 3]);
  const corr = (cs.kind === 't' ? (1 - cs.blockage) ** 2 : 1) * kap;
  const Fb = mulT(cs.R, mean.F), Mb = mulT(cs.R, mean.M);
  const V2 = st.V * st.V, N = cs.N;
  return [...Fb.map((v) => (cs.sign * v * corr) / (V2 * N * N)), ...Mb.map((v) => (cs.sign * v * corr) / (V2 * N * N * N))];
}

export { gridConvergence };

// Spin damping comes from a frozen mask with a moving-wall condition: the hull
// never sweeps through the water, so the wake history is not that of a turning
// body (a quasi-steady approximation; exact only for bodies of revolution).
// Its uncertainty gets this model term on top of the grid uncertainty.
export const SPIN_MODEL_UNCERTAINTY = 0.3;

/**
 * Damping must not inject energy: the symmetric part of D (scaled so that every
 * DOF has the units of a speed, s = [1, 1, 1, L, L, L]) is projected onto the
 * positive semidefinite cone by clipping its negative eigenvalues (Fossen 2011
 * sec. 6.4); the skew part does no work and is kept. Returns { D, clipped }.
 */
export function dissipative(D, L = 1) {
  const s = [1, 1, 1, L, L, L];
  const n = 6;
  const S = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (0.5 * (D[i][j] + D[j][i])) / (s[i] * s[j])));
  const { values, vectors } = jacobiEigen(S);
  let neg = 0, scale = 0;
  for (const v of values) { if (v < 0) neg += -v; scale = Math.max(scale, Math.abs(v)); }
  if (!(neg > 1e-12 * Math.max(scale, 1e-30))) return { D: D.map((r) => r.slice()), clipped: 0 };
  const out = Array.from({ length: n }, () => new Array(n).fill(0));
  for (let i = 0; i < n; i++) for (let j = 0; j < n; j++) {
    let v = 0;
    for (let k = 0; k < n; k++) if (values[k] > 0) v += vectors[i][k] * values[k] * vectors[j][k];
    out[i][j] = v * s[i] * s[j] + 0.5 * (D[i][j] - D[j][i]);
  }
  return { D: out, clipped: neg / Math.max(scale, 1e-30) };
}
// cyclic Jacobi eigen-decomposition of a small symmetric matrix (columns of vectors)
function jacobiEigen(A0) {
  const n = A0.length, A = A0.map((r) => r.slice());
  const V = Array.from({ length: n }, (_, i) => Array.from({ length: n }, (_, j) => (i === j ? 1 : 0)));
  for (let sweep = 0; sweep < 60; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    if (off < 1e-30) break;
    for (let p = 0; p < n; p++) for (let q = p + 1; q < n; q++) {
      if (Math.abs(A[p][q]) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * A[p][q]);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), sn = t * c;
      for (let k = 0; k < n; k++) { const akp = A[k][p], akq = A[k][q]; A[k][p] = c * akp - sn * akq; A[k][q] = sn * akp + c * akq; }
      for (let k = 0; k < n; k++) { const apk = A[p][k], aqk = A[q][k]; A[p][k] = c * apk - sn * aqk; A[q][k] = sn * apk + c * aqk; }
      for (let k = 0; k < n; k++) { const vkp = V[k][p], vkq = V[k][q]; V[k][p] = c * vkp - sn * vkq; V[k][q] = sn * vkp + c * vkq; }
    }
  }
  return { values: A.map((r, i) => r[i]), vectors: V };
}

/**
 * Assemble damping matrices from finished runs.
 * runs: [{ dof, sign, Re, N, C: [6], nuT?: { mean, max }, wall? }]  (several N per case = grid study)
 * opts: { rho, nu, ellRef, linearExtra: [6] (added to the diagonal), pFormal }
 */
export function assemble(runs, { rho, nu, ellRef: L, linearExtra = [0, 0, 0, 0, 0, 0], pFormal } = {}) {
  const DQ = Array.from({ length: 6 }, () => [0, 0, 0, 0, 0, 0]);
  const DL = Array.from({ length: 6 }, () => [0, 0, 0, 0, 0, 0]);
  const details = {};
  const warnings = [];
  const mean6 = (list) => [0, 1, 2, 3, 4, 5].map((i) => list.reduce((a, c) => a + c[i], 0) / list.length);
  // formal order of the wall treatment: second order with interpolated (Bouzidi) walls, first on a staircase
  const pF = pFormal ?? (runs.length && runs.every((r) => !r.wall || r.wall === 'interpolated') ? 2 : 1);
  DOFS.forEach((dof, j) => {
    const rs = runs.filter((r) => r.dof === dof.key);
    if (!rs.length) return;
    const Res = [...new Set(rs.map((r) => r.Re))].sort((a, b) => a - b);
    const signs = [...new Set(rs.map((r) => r.sign ?? 1))];
    const coef = {}, grid = {}, ReFit = {}, les = {};
    for (const Re of Res) {
      const atRe = rs.filter((r) => r.Re === Re);
      const Ns = [...new Set(atRe.map((r) => r.N ?? 0))].sort((a, b) => b - a);
      const perN = Ns.map((N) => ({ h: N ? 1 / N : 1, C: mean6(atRe.filter((r) => (r.N ?? 0) === N).map((r) => r.C)) }));
      let extrapolated = false;
      if (perN.length >= 2) {
        coef[Re] = [0, 1, 2, 3, 4, 5].map((i) => {
          const g = gridConvergence(perN.map((x) => ({ h: x.h, v: x.C[i] })), { pFormal: pF });
          const ok = g.status === 'monotonic' || g.status === 'two grids' || g.status === 'converged';
          if (i === j) { grid[Re] = g; extrapolated = ok; }
          return ok ? g.value : g.fine;
        });
      } else coef[Re] = perN[0].C;
      // effective Reynolds number of the fit: the LES eddy viscosity (measured
      // next to the hull on the finest grid) adds to the molecular one. It scales
      // with the cell size squared, so a grid-extrapolated value has none.
      const fine = atRe.filter((r) => (r.N ?? 0) === Ns[0] && r.nuT);
      const nt = fine.length ? { mean: fine.reduce((a, r) => a + r.nuT.mean, 0) / fine.length, max: Math.max(...fine.map((r) => r.nuT.max)) } : null;
      les[Re] = nt;
      ReFit[Re] = nt && !extrapolated ? Re / (1 + nt.mean) : Re;
      if (nt && nt.mean > LES_WARN && !extrapolated) warnings.push(`${dof.label} at Re ${Re}: the LES eddy viscosity next to the hull averages ${(nt.mean * 100).toFixed(0)}% of the molecular viscosity (max ${(nt.max * 100).toFixed(0)}%); the Reynolds fit uses the effective Re ${ReFit[Re].toFixed(0)}.`);
    }
    let Cinf, K;
    if (Res.length >= 2) {
      const lo = Res[0], hi = Res[Res.length - 1];
      const Cl = coef[lo], Ch = coef[hi], rl = ReFit[lo], rh = ReFit[hi];
      K = Cl.map((v, i) => (v - Ch[i]) / (1 / rl - 1 / rh));
      Cinf = Ch.map((v, i) => v - K[i] / rh);
      // drag must not grow towards high Re (else keep the high-Re value) and must
      // oppose the motion; a rejected diagonal fit drops the Re terms of the
      // whole column, so no off-diagonal 1/Re term is left without its diagonal
      if (K[j] > 0 || Cinf[j] > 0) {
        const clampedK = K[j] > 0;
        K[j] = Math.min(0, K[j]); Cinf[j] = Math.min(Ch[j], Ch[j] - K[j] / rh);
        if (clampedK) for (let i = 0; i < 6; i++) if (i !== j) { Cinf[i] = Ch[i]; K[i] = 0; }
      }
      for (let i = 0; i < 6; i++) if (i !== j && Math.abs(Cinf[i]) > 2 * Math.max(Math.abs(Cl[i]), Math.abs(Ch[i]))) { Cinf[i] = Ch[i]; K[i] = 0; }
      details[dof.key] = { Re: Res, ReFit: Res.map((r) => ReFit[r]), C: { [lo]: Cl, [hi]: Ch }, Cinf, K, grid, les };
    } else {
      Cinf = coef[Res[0]]; K = [0, 0, 0, 0, 0, 0];
      details[dof.key] = { Re: Res, ReFit: Res.map((r) => ReFit[r]), C: { [Res[0]]: Cinf }, Cinf, K, grid, les };
    }
    // a force even in the speed (lift on an asymmetric hull) and an odd one
    // (drag) can only be told apart with both tow directions (the mean of
    // sign * C keeps the odd part); with one direction the couplings are dropped
    const both = signs.includes(1) && signs.includes(-1);
    details[dof.key].couplings = both ? 'odd part of both tow directions' : 'not measured (one tow direction)';
    const s = dof.kind === 't' ? 1 : L;
    for (let i = 0; i < 6; i++) {
      if (i !== j && !both) continue;
      const p = i < 3 ? 2 : 3;
      DQ[i][j] = -rho * s * s * L ** p * Cinf[i];
      DL[i][j] = -rho * s * L ** (p - 1) * K[i] * nu;
    }
    DQ[j][j] = Math.max(0, DQ[j][j]);
    DL[j][j] = Math.max(0, DL[j][j]) + (linearExtra[j] || 0);
    const gHi = grid[Res[Res.length - 1]];
    const gci = gHi ? gHi.gci : NaN;
    if (dof.kind === 'r') {
      details[dof.key].approximation = 'frozen geometry (spin modelled by a moving-wall condition on a fixed hull)';
      details[dof.key].uncertainty = Math.hypot(Number.isFinite(gci) ? gci : 0, SPIN_MODEL_UNCERTAINTY);
    } else details[dof.key].uncertainty = gci;
    details[dof.key].gridUncertainty = gci;
    details[dof.key].gridStatus = gHi ? gHi.status : 'single grid';
  });
  // energy check on the assembled matrices
  const pl = dissipative(DL, L), pq = dissipative(DQ, L);
  if (pl.clipped > 1e-9) warnings.push(`The linear damping matrix was not dissipative (negative eigenvalue ${(pl.clipped * 100).toFixed(1)}% of the largest): its symmetric part was projected to positive semidefinite.`);
  if (pq.clipped > 1e-9) warnings.push(`The quadratic damping couplings were too strong for the diagonal (negative eigenvalue ${(pq.clipped * 100).toFixed(1)}% of the largest): projected to positive semidefinite.`);
  return { DQ: pq.D, DL: pl.D, details, warnings, pFormal: pF };
}

/**
 * Run one case to convergence on the CPU (worker / Node).
 * onProgress({ step, fraction, rel, value, slice? })
 */
export function runCPU(cs, Re, { onProgress = () => {}, sliceEvery = 0, maxSeconds = Infinity, snapshot = false } = {}) {
  const st = caseSettings(cs, Re);
  const solver = new LBMCPU({ nx: cs.nx, ny: cs.ny, nz: cs.nz, mask: cs.mask, q: cs.q }, { ...st.params, useWasm: true });
  const samples = [];
  let conv = { done: false };
  const t0 = Date.now();
  let lastSlice = 0;
  while (!conv.done) {
    applyRamp(solver, solver.steps, cs, st);
    solver.step(st.sampleEvery);
    const f = solver.force();
    samples.push({ step: solver.steps, F: f.F, M: f.M });
    conv = convergence(samples, solver.steps, cs, st);
    const frac = Math.min(1, solver.steps / (st.tMax * st.Tc));
    const msg = { step: solver.steps, fraction: frac, rel: conv.rel, value: primary(f, cs, st), t: solver.steps / st.Tc };
    if (sliceEvery && Date.now() - lastSlice > sliceEvery) { lastSlice = Date.now(); msg.slice = solver.slice(cs.kind === 't' ? 'y' : 'z', Math.round(cs.ref[cs.kind === 't' ? 1 : 2])); }
    onProgress(msg);
    if ((Date.now() - t0) / 1000 > maxSeconds) { conv = { done: true, converged: false, rel: conv.rel }; break; }
  }
  const Wn = st.win * st.Tc;
  const mean = windowMean(samples, solver.steps - Wn, solver.steps) ?? { ...samples[samples.length - 1], n: 1 };
  const NC = solver.N, A = solver.A;
  const nuT = eddyViscosityRatio((id, g) => { for (let i = 0; i < Q; i++) g[i] = A[i * NC + id]; }, nearWallCells(cs), st.params.tau0, st.params.smag);
  return { Re, mean, nuT, steps: solver.steps, converged: !!conv.converged, rel: conv.rel, C: runCoefficients(cs, st, mean), history: decimate(samples.map((s) => [s.step / st.Tc, primary(s, cs, st)]), 240), seconds: (Date.now() - t0) / 1000, mlups: (cs.nx * cs.ny * cs.nz * solver.steps) / Math.max(1e-3, (Date.now() - t0) / 1000) / 1e6, field: snapshot ? solver.snapshot(snapshotStride(cs)) : null };
}

export const snapshotStride = (cs) => (cs.nx * cs.ny * cs.nz > 1.5e6 ? 2 : 1);

export function decimate(a, n) {
  if (a.length <= n) return a;
  const out = [];
  for (let i = 0; i < n; i++) out.push(a[Math.round((i * (a.length - 1)) / (n - 1))]);
  return out;
}
