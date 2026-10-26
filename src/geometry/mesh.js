// Mesh analysis for arbitrary (possibly messy) STL hulls.
//
// Everything downstream (buoyancy, inertia estimate, drag estimate, panel-method
// added mass, CFD solid mask, thruster-jet blockage, contacts) works on voxel
// grids derived here, which makes the pipeline robust to the usual CAD export
// problems: open shells, overlapping parts, inconsistent winding.
//
// Frames: all outputs are in the vehicle frame (FRD: x forward, y starboard,
// z down), metres.

import { vcross, vdot, clamp } from '../core/math.js';

export const AXIS_NAMES = ['+X', '-X', '+Y', '-Y', '+Z', '-Z'];
export function axisVec(s) {
  const sgn = s[0] === '-' ? -1 : 1;
  const k = { X: 0, Y: 1, Z: 2 }[s[s.length - 1].toUpperCase()];
  if (k === undefined) throw new Error('bad axis ' + s);
  const v = [0, 0, 0];
  v[k] = sgn;
  return v;
}

export const UNIT_SCALES = { m: 1, cm: 0.01, mm: 0.001, in: 0.0254, ft: 0.3048 };

// Guess the unit of an STL from its size (ROVs are 0.2 - 3 m).
export function guessUnits(bboxSize) {
  const L = Math.max(...bboxSize);
  if (L > 50) return 'mm';
  if (L > 5) return 'cm';
  return 'm';
}

export function bbox(positions) {
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k++) {
      const v = positions[i + k];
      if (v < min[k]) min[k] = v;
      if (v > max[k]) max[k] = v;
    }
  }
  return { min, max, size: [max[0] - min[0], max[1] - min[1], max[2] - min[2]], center: [(max[0] + min[0]) / 2, (max[1] + min[1]) / 2, (max[2] + min[2]) / 2] };
}

// Build the model->vehicle transform: v_vehicle = R * (s * v_model) + t.
export function makeModelTransform({ units = 'm', scale = 1, forward = '+X', up = '+Z' } = {}) {
  const f = axisVec(forward), u = axisVec(up);
  if (Math.abs(vdot(f, u)) > 1e-9) throw new Error('Forward and up axes must be perpendicular');
  const ez = [-u[0], -u[1], -u[2]]; // FRD: z points down
  const ex = f;
  const ey = vcross(ez, ex);
  const s = (UNIT_SCALES[units] ?? 1) * scale;
  const R = [ex[0], ex[1], ex[2], ey[0], ey[1], ey[2], ez[0], ez[1], ez[2]];
  return { R, s };
}

export function transformPositions(positions, { R, s }, translation = [0, 0, 0]) {
  const out = new Float32Array(positions.length);
  for (let i = 0; i < positions.length; i += 3) {
    const x = positions[i] * s, y = positions[i + 1] * s, z = positions[i + 2] * s;
    out[i] = R[0] * x + R[1] * y + R[2] * z + translation[0];
    out[i + 1] = R[3] * x + R[4] * y + R[5] * z + translation[1];
    out[i + 2] = R[6] * x + R[7] * y + R[8] * z + translation[2];
  }
  return out;
}

// Topology check: weld coincident vertices and count edge usage.
export function topology(positions, tolRel = 1e-6) {
  const bb = bbox(positions);
  const tol = Math.max(...bb.size, 1e-9) * tolRel;
  const inv = 1 / tol;
  const map = new Map();
  const triCount = positions.length / 9;
  const idx = new Uint32Array(triCount * 3);
  let nv = 0;
  for (let i = 0; i < triCount * 3; i++) {
    const x = Math.round(positions[i * 3] * inv), y = Math.round(positions[i * 3 + 1] * inv), z = Math.round(positions[i * 3 + 2] * inv);
    const key = x + ',' + y + ',' + z;
    let id = map.get(key);
    if (id === undefined) { id = nv++; map.set(key, id); }
    idx[i] = id;
  }
  const edges = new Map();
  let degenerate = 0;
  for (let t = 0; t < triCount; t++) {
    const a = idx[t * 3], b = idx[t * 3 + 1], c = idx[t * 3 + 2];
    if (a === b || b === c || a === c) { degenerate++; continue; }
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      const key = p < q ? p * 4194304 + q : q * 4194304 + p; // exact for < 4M vertices
      const e = edges.get(key);
      if (e) { e.n++; e.dir += p < q ? 1 : -1; } else edges.set(key, { n: 1, dir: p < q ? 1 : -1 });
    }
  }
  let boundary = 0, nonManifold = 0, inconsistent = 0;
  for (const e of edges.values()) {
    if (e.n === 1) boundary++;
    else if (e.n > 2) nonManifold++;
    else if (e.dir !== 0) inconsistent++;
  }
  return { vertices: nv, triangles: triCount, degenerate, boundaryEdges: boundary, nonManifoldEdges: nonManifold, inconsistentEdges: inconsistent, watertight: boundary === 0 && nonManifold === 0 };
}

// Signed volume via divergence theorem (valid for closed, consistently wound meshes).
export function signedVolume(positions) {
  let V = 0, cx = 0, cy = 0, cz = 0;
  for (let i = 0; i < positions.length; i += 9) {
    const ax = positions[i], ay = positions[i + 1], az = positions[i + 2];
    const bx = positions[i + 3], by = positions[i + 4], bz = positions[i + 5];
    const qx = positions[i + 6], qy = positions[i + 7], qz = positions[i + 8];
    const v = (ax * (by * qz - bz * qy) - ay * (bx * qz - bz * qx) + az * (bx * qy - by * qx)) / 6;
    V += v;
    cx += v * (ax + bx + qx) / 4; cy += v * (ay + by + qy) / 4; cz += v * (az + bz + qz) / 4;
  }
  return { volume: V, centroid: Math.abs(V) > 1e-15 ? [cx / V, cy / V, cz / V] : [0, 0, 0] };
}

// ---------------------------------------------------------------------------
// Voxel grids
// ---------------------------------------------------------------------------
export class VoxelGrid {
  constructor(nx, ny, nz, cell, origin) {
    this.nx = nx; this.ny = ny; this.nz = nz; this.cell = cell; this.origin = origin;
    this.data = new Uint8Array(nx * ny * nz);
  }
  idx(i, j, k) { return i + this.nx * (j + this.ny * k); }
  get(i, j, k) {
    if (i < 0 || j < 0 || k < 0 || i >= this.nx || j >= this.ny || k >= this.nz) return 0;
    return this.data[i + this.nx * (j + this.ny * k)];
  }
  center(i, j, k) {
    const c = this.cell, o = this.origin;
    return [o[0] + (i + 0.5) * c, o[1] + (j + 0.5) * c, o[2] + (k + 0.5) * c];
  }
  count() { let n = 0; for (let i = 0; i < this.data.length; i++) n += this.data[i] ? 1 : 0; return n; }
  clone() { const g = new VoxelGrid(this.nx, this.ny, this.nz, this.cell, this.origin.slice()); g.data.set(this.data); return g; }
  dims() { return [this.nx, this.ny, this.nz]; }
}

export function makeGrid(bb, cell, margin = 1) {
  const n = [0, 1, 2].map((k) => Math.max(1, Math.ceil(bb.size[k] / cell - 1e-4) + 2 * margin));
  const origin = [0, 1, 2].map((k) => bb.center[k] - (n[k] * cell) / 2);
  return new VoxelGrid(n[0], n[1], n[2], cell, origin);
}

// Inside test by axis-aligned ray casting along all three axes with a
// winding-number rule (handles overlapping solids) and parity fallback for
// open shells. A cell is inside when at least `votes` of 3 rays agree.
export function voxelizeSolid(positions, grid, votes = 2) {
  const n = [grid.nx, grid.ny, grid.nz];
  const vote = new Uint8Array(grid.data.length);
  const triCount = positions.length / 9;
  const jitter = [0.0137 * grid.cell, 0.0291 * grid.cell, 0.0213 * grid.cell];
  const stats = { openRays: 0, rays: 0 };
  for (let a = 0; a < 3; a++) {
    const b = (a + 1) % 3, c = (a + 2) % 3;
    const nb = n[b], nc = n[c], na = n[a];
    const ob = grid.origin[b] + jitter[b], oc = grid.origin[c] + jitter[c];
    const bins = new Array(nb * nc);
    for (let t = 0; t < triCount; t++) {
      const p = t * 9;
      const b0 = positions[p + b], b1 = positions[p + 3 + b], b2 = positions[p + 6 + b];
      const c0 = positions[p + c], c1 = positions[p + 3 + c], c2 = positions[p + 6 + c];
      const ib0 = Math.max(0, Math.ceil((Math.min(b0, b1, b2) - ob) / grid.cell - 0.5));
      const ib1 = Math.min(nb - 1, Math.floor((Math.max(b0, b1, b2) - ob) / grid.cell - 0.5));
      const ic0 = Math.max(0, Math.ceil((Math.min(c0, c1, c2) - oc) / grid.cell - 0.5));
      const ic1 = Math.min(nc - 1, Math.floor((Math.max(c0, c1, c2) - oc) / grid.cell - 0.5));
      for (let ic = ic0; ic <= ic1; ic++)
        for (let ib = ib0; ib <= ib1; ib++) {
          const k = ib + nb * ic;
          (bins[k] || (bins[k] = [])).push(t);
        }
    }
    const tbuf = [], sbuf = [];
    for (let ic = 0; ic < nc; ic++) {
      for (let ib = 0; ib < nb; ib++) {
        const list = bins[ib + nb * ic];
        if (!list) continue;
        const pb = ob + (ib + 0.5) * grid.cell, pc = oc + (ic + 0.5) * grid.cell;
        tbuf.length = 0; sbuf.length = 0;
        for (const t of list) {
          const p = t * 9;
          const B0 = positions[p + b], B1 = positions[p + 3 + b], B2 = positions[p + 6 + b];
          const C0 = positions[p + c], C1 = positions[p + 3 + c], C2 = positions[p + 6 + c];
          const w0 = (B2 - B1) * (pc - C1) - (C2 - C1) * (pb - B1);
          const w1 = (B0 - B2) * (pc - C2) - (C0 - C2) * (pb - B2);
          const w2 = (B1 - B0) * (pc - C0) - (C1 - C0) * (pb - B0);
          const area = w0 + w1 + w2;
          if (area === 0) continue;
          if (area > 0 ? (w0 < 0 || w1 < 0 || w2 < 0) : (w0 > 0 || w1 > 0 || w2 > 0)) continue;
          const A0 = positions[p + a], A1 = positions[p + 3 + a], A2 = positions[p + 6 + a];
          tbuf.push((w0 * A0 + w1 * A1 + w2 * A2) / area);
          sbuf.push(area < 0 ? 1 : -1);
        }
        if (tbuf.length === 0) continue;
        stats.rays++;
        const order = tbuf.map((_, i) => i).sort((x, y) => tbuf[x] - tbuf[y]);
        let total = 0;
        for (const i of order) total += sbuf[i];
        const useParity = total !== 0;
        if (useParity) stats.openRays++;
        let w = 0, par = 0, ptr = 0;
        const oa = grid.origin[a];
        for (let ia = 0; ia < na; ia++) {
          const pa = oa + (ia + 0.5) * grid.cell;
          while (ptr < order.length && tbuf[order[ptr]] < pa) { w += sbuf[order[ptr]]; par ^= 1; ptr++; }
          const inside = useParity ? par === 1 : w > 0;
          if (inside) {
            const ijk = [0, 0, 0];
            ijk[a] = ia; ijk[b] = ib; ijk[c] = ic;
            vote[ijk[0] + grid.nx * (ijk[1] + grid.ny * ijk[2])]++;
          }
        }
      }
    }
  }
  let filled = 0;
  for (let i = 0; i < vote.length; i++) {
    if (vote[i] >= votes) { grid.data[i] = 1; filled++; }
  }
  return { filled, openRayFraction: stats.rays ? stats.openRays / stats.rays : 0 };
}

// Mark every cell intersected by a triangle (Akenine-Moller SAT). Guarantees that
// thin plates and wires end up solid, which matters for CFD walls and BEM hulls.
export function voxelizeSurface(positions, grid) {
  const h = grid.cell * 0.5 * 1.0001;
  const o = grid.origin, c = grid.cell;
  const triCount = positions.length / 9;
  let marked = 0;
  const v0 = [0, 0, 0], v1 = [0, 0, 0], v2 = [0, 0, 0];
  for (let t = 0; t < triCount; t++) {
    const p = t * 9;
    const mn = [0, 1, 2].map((k) => Math.min(positions[p + k], positions[p + 3 + k], positions[p + 6 + k]));
    const mx = [0, 1, 2].map((k) => Math.max(positions[p + k], positions[p + 3 + k], positions[p + 6 + k]));
    const i0 = clamp(Math.floor((mn[0] - o[0]) / c), 0, grid.nx - 1), i1 = clamp(Math.floor((mx[0] - o[0]) / c), 0, grid.nx - 1);
    const j0 = clamp(Math.floor((mn[1] - o[1]) / c), 0, grid.ny - 1), j1 = clamp(Math.floor((mx[1] - o[1]) / c), 0, grid.ny - 1);
    const k0 = clamp(Math.floor((mn[2] - o[2]) / c), 0, grid.nz - 1), k1 = clamp(Math.floor((mx[2] - o[2]) / c), 0, grid.nz - 1);
    for (let k = k0; k <= k1; k++)
      for (let j = j0; j <= j1; j++)
        for (let i = i0; i <= i1; i++) {
          const id = i + grid.nx * (j + grid.ny * k);
          if (grid.data[id]) continue;
          const cx = o[0] + (i + 0.5) * c, cy = o[1] + (j + 0.5) * c, cz = o[2] + (k + 0.5) * c;
          v0[0] = positions[p] - cx; v0[1] = positions[p + 1] - cy; v0[2] = positions[p + 2] - cz;
          v1[0] = positions[p + 3] - cx; v1[1] = positions[p + 4] - cy; v1[2] = positions[p + 5] - cz;
          v2[0] = positions[p + 6] - cx; v2[1] = positions[p + 7] - cy; v2[2] = positions[p + 8] - cz;
          if (triBoxOverlap(v0, v1, v2, h)) { grid.data[id] = 1; marked++; }
        }
  }
  return marked;
}

function triBoxOverlap(v0, v1, v2, h) {
  const e0 = [v1[0] - v0[0], v1[1] - v0[1], v1[2] - v0[2]];
  const e1 = [v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2]];
  const e2 = [v0[0] - v2[0], v0[1] - v2[1], v0[2] - v2[2]];
  const axisTest = (ax, ay, az) => {
    const p0 = ax * v0[0] + ay * v0[1] + az * v0[2];
    const p1 = ax * v1[0] + ay * v1[1] + az * v1[2];
    const p2 = ax * v2[0] + ay * v2[1] + az * v2[2];
    const r = h * (Math.abs(ax) + Math.abs(ay) + Math.abs(az));
    return !(Math.min(p0, p1, p2) > r || Math.max(p0, p1, p2) < -r);
  };
  for (const e of [e0, e1, e2]) {
    if (!axisTest(0, -e[2], e[1])) return false;
    if (!axisTest(e[2], 0, -e[0])) return false;
    if (!axisTest(-e[1], e[0], 0)) return false;
  }
  for (let k = 0; k < 3; k++) {
    if (Math.min(v0[k], v1[k], v2[k]) > h || Math.max(v0[k], v1[k], v2[k]) < -h) return false;
  }
  const n = vcross(e0, e1);
  const d = -vdot(n, v0);
  const r = h * (Math.abs(n[0]) + Math.abs(n[1]) + Math.abs(n[2]));
  return Math.abs(d) <= r;
}

export function union(a, b) {
  const g = a.clone();
  for (let i = 0; i < g.data.length; i++) g.data[i] = a.data[i] | b.data[i];
  return g;
}

// Morphological dilation / erosion with a 3x3x3 box structuring element.
export function dilate(grid, r = 1) {
  let cur = grid;
  for (let it = 0; it < r; it++) {
    const out = cur.clone();
    const { nx, ny, nz } = cur;
    for (let k = 0; k < nz; k++)
      for (let j = 0; j < ny; j++)
        for (let i = 0; i < nx; i++) {
          if (cur.data[i + nx * (j + ny * k)]) continue;
          let hit = 0;
          for (let dk = -1; dk <= 1 && !hit; dk++)
            for (let dj = -1; dj <= 1 && !hit; dj++)
              for (let di = -1; di <= 1 && !hit; di++) if (cur.get(i + di, j + dj, k + dk)) hit = 1;
          if (hit) out.data[i + nx * (j + ny * k)] = 1;
        }
    cur = out;
  }
  return cur;
}
export function erode(grid, r = 1) {
  let cur = grid;
  for (let it = 0; it < r; it++) {
    const out = cur.clone();
    const { nx, ny, nz } = cur;
    for (let k = 0; k < nz; k++)
      for (let j = 0; j < ny; j++)
        for (let i = 0; i < nx; i++) {
          if (!cur.data[i + nx * (j + ny * k)]) continue;
          let miss = 0;
          for (let dk = -1; dk <= 1 && !miss; dk++)
            for (let dj = -1; dj <= 1 && !miss; dj++)
              for (let di = -1; di <= 1 && !miss; di++) if (!cur.get(i + di, j + dj, k + dk)) miss = 1;
          if (miss) out.data[i + nx * (j + ny * k)] = 0;
        }
    cur = out;
  }
  return cur;
}
// Fill cavities that are not connected to the outside (sealed hulls, enclosed water).
export function fillEnclosed(grid) {
  const { nx, ny, nz } = grid;
  const outside = new Uint8Array(grid.data.length);
  const stack = [];
  const push = (i, j, k) => {
    if (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz) return;
    const id = i + nx * (j + ny * k);
    if (outside[id] || grid.data[id]) return;
    outside[id] = 1;
    stack.push(id);
  };
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) { push(0, j, k); push(nx - 1, j, k); }
  for (let k = 0; k < nz; k++) for (let i = 0; i < nx; i++) { push(i, 0, k); push(i, ny - 1, k); }
  for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) { push(i, j, 0); push(i, j, nz - 1); }
  while (stack.length) {
    const id = stack.pop();
    const i = id % nx, j = Math.floor(id / nx) % ny, k = Math.floor(id / (nx * ny));
    push(i + 1, j, k); push(i - 1, j, k); push(i, j + 1, k); push(i, j - 1, k); push(i, j, k + 1); push(i, j, k - 1);
  }
  const g = grid.clone();
  for (let i = 0; i < g.data.length; i++) g.data[i] = outside[i] ? 0 : 1;
  return g;
}

// ---------------------------------------------------------------------------
// Derived properties
// ---------------------------------------------------------------------------
export function massProps(grid) {
  const dV = grid.cell ** 3;
  let n = 0, sx = 0, sy = 0, sz = 0;
  const { nx, ny, nz } = grid;
  for (let k = 0; k < nz; k++)
    for (let j = 0; j < ny; j++)
      for (let i = 0; i < nx; i++) {
        if (!grid.data[i + nx * (j + ny * k)]) continue;
        const c = grid.center(i, j, k);
        n++; sx += c[0]; sy += c[1]; sz += c[2];
      }
  const volume = n * dV;
  if (n === 0) return { volume: 0, centroid: [0, 0, 0], inertiaUnit: [0, 0, 0, 0, 0, 0, 0, 0, 0] };
  const cen = [sx / n, sy / n, sz / n];
  let Ixx = 0, Iyy = 0, Izz = 0, Ixy = 0, Ixz = 0, Iyz = 0;
  for (let k = 0; k < nz; k++)
    for (let j = 0; j < ny; j++)
      for (let i = 0; i < nx; i++) {
        if (!grid.data[i + nx * (j + ny * k)]) continue;
        const c = grid.center(i, j, k);
        const x = c[0] - cen[0], y = c[1] - cen[1], z = c[2] - cen[2];
        Ixx += y * y + z * z; Iyy += x * x + z * z; Izz += x * x + y * y;
        Ixy -= x * y; Ixz -= x * z; Iyz -= y * z;
      }
  const self = (grid.cell ** 2) / 6; // own inertia of each cube about its centre
  const f = dV;
  Ixx = (Ixx + n * self) * f; Iyy = (Iyy + n * self) * f; Izz = (Izz + n * self) * f;
  Ixy *= f; Ixz *= f; Iyz *= f;
  return { volume, centroid: cen, inertiaUnit: [Ixx, Ixy, Ixz, Ixy, Iyy, Iyz, Ixz, Iyz, Izz] };
}

// Projected areas and strip-theory integrals for drag estimates.
export function projections(grid, ref = [0, 0, 0]) {
  const { nx, ny, nz, cell } = grid;
  const dA = cell * cell;
  const out = { area: [0, 0, 0], k3: { roll: 0, pitch: 0, yaw: 0 } };
  // columns along x (projection on y-z plane)
  for (let k = 0; k < nz; k++)
    for (let j = 0; j < ny; j++) {
      let hit = false;
      for (let i = 0; i < nx && !hit; i++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (!hit) continue;
      const c = grid.center(0, j, k);
      const y = c[1] - ref[1], z = c[2] - ref[2];
      out.area[0] += dA;
      out.k3.pitch += Math.abs(z) ** 3 * dA; // x-velocity from pitch = q*z
      out.k3.yaw += Math.abs(y) ** 3 * dA; // x-velocity from yaw = -r*y
    }
  // columns along y (projection on x-z plane)
  for (let k = 0; k < nz; k++)
    for (let i = 0; i < nx; i++) {
      let hit = false;
      for (let j = 0; j < ny && !hit; j++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (!hit) continue;
      const c = grid.center(i, 0, k);
      const x = c[0] - ref[0], z = c[2] - ref[2];
      out.area[1] += dA;
      out.k3.roll += Math.abs(z) ** 3 * dA; // y-velocity from roll = -p*z
      out.k3.yaw += Math.abs(x) ** 3 * dA; // y-velocity from yaw = r*x
    }
  // columns along z (projection on x-y plane)
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      let hit = false;
      for (let k = 0; k < nz && !hit; k++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (!hit) continue;
      const c = grid.center(i, j, 0);
      const x = c[0] - ref[0], y = c[1] - ref[1];
      out.area[2] += dA;
      out.k3.roll += Math.abs(y) ** 3 * dA; // z-velocity from roll = p*y
      out.k3.pitch += Math.abs(x) ** 3 * dA; // z-velocity from pitch = -q*x
    }
  return out;
}

// Silhouette columns of the projections (one entry per hit column: its two
// transverse coordinates), so the strip-theory integrals can be re-evaluated
// about any reference point (e.g. the CG) without the voxel grid.
export function projectionColumns(grid) {
  const { nx, ny, nz, cell } = grid;
  const cx = [], cy = [], cz = [];
  for (let k = 0; k < nz; k++)
    for (let j = 0; j < ny; j++) {
      let hit = false;
      for (let i = 0; i < nx && !hit; i++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (hit) { const c = grid.center(0, j, k); cx.push(c[1], c[2]); }
    }
  for (let k = 0; k < nz; k++)
    for (let i = 0; i < nx; i++) {
      let hit = false;
      for (let j = 0; j < ny && !hit; j++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (hit) { const c = grid.center(i, 0, k); cy.push(c[0], c[2]); }
    }
  for (let j = 0; j < ny; j++)
    for (let i = 0; i < nx; i++) {
      let hit = false;
      for (let k = 0; k < nz && !hit; k++) hit = !!grid.data[i + nx * (j + ny * k)];
      if (hit) { const c = grid.center(i, j, 0); cz.push(c[0], c[1]); }
    }
  return { dA: cell * cell, x: Float32Array.from(cx), y: Float32Array.from(cy), z: Float32Array.from(cz) };
}
// projections() from stored columns about ref (same integrals, same result)
export function projectionsFromColumns(cols, ref = [0, 0, 0]) {
  const dA = cols.dA, out = { area: [0, 0, 0], k3: { roll: 0, pitch: 0, yaw: 0 } };
  const a3 = (v) => Math.abs(v) ** 3 * dA;
  for (let n = 0; n < cols.x.length; n += 2) { out.area[0] += dA; out.k3.yaw += a3(cols.x[n] - ref[1]); out.k3.pitch += a3(cols.x[n + 1] - ref[2]); }
  for (let n = 0; n < cols.y.length; n += 2) { out.area[1] += dA; out.k3.yaw += a3(cols.y[n] - ref[0]); out.k3.roll += a3(cols.y[n + 1] - ref[2]); }
  for (let n = 0; n < cols.z.length; n += 2) { out.area[2] += dA; out.k3.pitch += a3(cols.z[n] - ref[0]); out.k3.roll += a3(cols.z[n + 1] - ref[1]); }
  return out;
}

// Coarse buoyancy elements: groups of fine cells, each with volume, centroid and
// half-extent, used for surface-aware buoyancy at run time.
// Displaced volume and its centroid by exact integration along rays. Along a
// ray the inside length is exact (from the mesh crossings); across, the rays
// sit on a fine grid (cell / sub), so thin plates and small parts no longer
// depend on how they line up with the voxel grid. Done along x, y and z; each
// axis is weighted by how little surface runs parallel to its rays, because
// that is where the across-ray (midpoint rule) error comes from. Overlapping
// closed parts count once (winding number > 0).
export function rayVolume(positions, bb, cell, { sub = 6 } = {}) {
  const triCount = positions.length / 9;
  const par = [0, 0, 0];
  for (let t = 0; t < triCount; t++) {
    const p = t * 9;
    const ux = positions[p + 3] - positions[p], uy = positions[p + 4] - positions[p + 1], uz = positions[p + 5] - positions[p + 2];
    const vx = positions[p + 6] - positions[p], vy = positions[p + 7] - positions[p + 1], vz = positions[p + 8] - positions[p + 2];
    const n = [uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx];
    const len = Math.hypot(n[0], n[1], n[2]);
    if (!len) continue;
    for (let a = 0; a < 3; a++) { const na = n[a] / len; par[a] += (len / 2) * Math.sqrt(Math.max(0, 1 - na * na)); }
  }
  const h = cell / sub;
  const out = [];
  let rays = 0, open = 0;
  for (let a = 0; a < 3; a++) {
    const b = (a + 1) % 3, c = (a + 2) % 3;
    // different sub-pixel offsets per axis keep the three estimates independent
    const ob = bb.min[b] - h + h * ((0.381966 + 0.29 * a) % 1), oc = bb.min[c] - h + h * ((0.618034 + 0.47 * a) % 1);
    const nb = Math.ceil((bb.max[b] - ob) / h) + 1, nc = Math.ceil((bb.max[c] - oc) / h) + 1;
    const bins = new Array(nb * nc);
    for (let t = 0; t < triCount; t++) {
      const p = t * 9;
      const b0 = positions[p + b], b1 = positions[p + 3 + b], b2 = positions[p + 6 + b];
      const c0 = positions[p + c], c1 = positions[p + 3 + c], c2 = positions[p + 6 + c];
      const ib0 = Math.max(0, Math.ceil((Math.min(b0, b1, b2) - ob) / h - 0.5)), ib1 = Math.min(nb - 1, Math.floor((Math.max(b0, b1, b2) - ob) / h - 0.5));
      const ic0 = Math.max(0, Math.ceil((Math.min(c0, c1, c2) - oc) / h - 0.5)), ic1 = Math.min(nc - 1, Math.floor((Math.max(c0, c1, c2) - oc) / h - 0.5));
      for (let ic = ic0; ic <= ic1; ic++) for (let ib = ib0; ib <= ib1; ib++) { const k = ib + nb * ic; (bins[k] || (bins[k] = [])).push(t); }
    }
    let V = 0, Ma = 0, Mb = 0, Mc = 0;
    const tb = [], sb = [], ord = [];
    for (let ic = 0; ic < nc; ic++) for (let ib = 0; ib < nb; ib++) {
      const list = bins[ib + nb * ic];
      if (!list) continue;
      const pb = ob + (ib + 0.5) * h, pc = oc + (ic + 0.5) * h;
      tb.length = 0; sb.length = 0;
      for (const t of list) {
        const p = t * 9;
        const B0 = positions[p + b], B1 = positions[p + 3 + b], B2 = positions[p + 6 + b];
        const C0 = positions[p + c], C1 = positions[p + 3 + c], C2 = positions[p + 6 + c];
        const w0 = (B2 - B1) * (pc - C1) - (C2 - C1) * (pb - B1);
        const w1 = (B0 - B2) * (pc - C2) - (C0 - C2) * (pb - B2);
        const w2 = (B1 - B0) * (pc - C0) - (C1 - C0) * (pb - B0);
        const area = w0 + w1 + w2;
        if (area === 0 || (area > 0 ? (w0 < 0 || w1 < 0 || w2 < 0) : (w0 > 0 || w1 > 0 || w2 > 0))) continue;
        tb.push((w0 * positions[p + a] + w1 * positions[p + 3 + a] + w2 * positions[p + 6 + a]) / area);
        sb.push(area < 0 ? 1 : -1);
      }
      if (!tb.length) continue;
      rays++;
      ord.length = 0;
      for (let i = 0; i < tb.length; i++) ord.push(i);
      ord.sort((x, y) => tb[x] - tb[y]);
      let tot = 0;
      for (const i of ord) tot += sb[i];
      const parity = tot !== 0; // an open mesh along this ray: fall back to even-odd
      if (parity) open++;
      let w = 0, par2 = 0, start = 0, inside = false;
      for (const i of ord) {
        w += sb[i]; par2 ^= 1;
        const now = parity ? par2 === 1 : w > 0;
        if (now && !inside) start = tb[i];
        else if (!now && inside) { const t0 = start, t1 = tb[i], L = t1 - t0; V += L; Ma += (t1 * t1 - t0 * t0) / 2; Mb += pb * L; Mc += pc * L; }
        inside = now;
      }
    }
    const A = h * h;
    const M = [0, 0, 0];
    M[a] = Ma * A; M[b] = Mb * A; M[c] = Mc * A;
    out.push({ V: V * A, M });
  }
  const w = par.map((p) => 1 / (p * p + 1e-12));
  const W = w[0] + w[1] + w[2];
  const volume = (w[0] * out[0].V + w[1] * out[1].V + w[2] * out[2].V) / W;
  const mom = [0, 1, 2].map((k) => (w[0] * out[0].M[k] + w[1] * out[1].M[k] + w[2] * out[2].M[k]) / W);
  return { volume, centroid: volume > 0 ? mom.map((m) => m / volume) : [0, 0, 0], perAxis: out.map((o) => o.V), weights: w.map((x) => x / W), openFraction: rays ? open / rays : 1 };
}

export function buoyancyElements(grid, targetLongest = 14) {
  const longest = Math.max(grid.nx, grid.ny, grid.nz);
  const g = Math.max(1, Math.round(longest / targetLongest));
  const dV = grid.cell ** 3;
  const map = new Map();
  for (let k = 0; k < grid.nz; k++)
    for (let j = 0; j < grid.ny; j++)
      for (let i = 0; i < grid.nx; i++) {
        if (!grid.data[i + grid.nx * (j + grid.ny * k)]) continue;
        const key = Math.floor(i / g) + ',' + Math.floor(j / g) + ',' + Math.floor(k / g);
        let e = map.get(key);
        const c = grid.center(i, j, k);
        if (!e) { e = { n: 0, s: [0, 0, 0], mn: c.slice(), mx: c.slice() }; map.set(key, e); }
        e.n++;
        for (let q = 0; q < 3; q++) { e.s[q] += c[q]; if (c[q] < e.mn[q]) e.mn[q] = c[q]; if (c[q] > e.mx[q]) e.mx[q] = c[q]; }
      }
  const els = [];
  for (const e of map.values()) {
    els.push({
      p: [e.s[0] / e.n, e.s[1] / e.n, e.s[2] / e.n],
      v: e.n * dV,
      h: [0, 1, 2].map((q) => (e.mx[q] - e.mn[q]) / 2 + grid.cell / 2),
    });
  }
  return els;
}

// Contact points approximating the convex hull (extremes along 26 directions).
export function contactPoints(grid) {
  const dirs = [];
  for (let a = -1; a <= 1; a++) for (let b = -1; b <= 1; b++) for (let c = -1; c <= 1; c++) if (a || b || c) dirs.push([a, b, c]);
  const best = dirs.map(() => ({ d: -Infinity, p: null }));
  for (let k = 0; k < grid.nz; k++)
    for (let j = 0; j < grid.ny; j++)
      for (let i = 0; i < grid.nx; i++) {
        if (!grid.data[i + grid.nx * (j + grid.ny * k)]) continue;
        const c = grid.center(i, j, k);
        dirs.forEach((d, n) => {
          const p = [c[0] + 0.5 * grid.cell * d[0], c[1] + 0.5 * grid.cell * d[1], c[2] + 0.5 * grid.cell * d[2]];
          const s = p[0] * d[0] + p[1] * d[1] + p[2] * d[2];
          if (s > best[n].d) best[n] = { d: s, p };
        });
      }
  const pts = [];
  for (const b of best) {
    if (!b.p) continue;
    if (!pts.some((q) => Math.hypot(q[0] - b.p[0], q[1] - b.p[1], q[2] - b.p[2]) < grid.cell * 0.5)) pts.push(b.p);
  }
  return pts;
}

// Distance along a ray until it enters a solid cell (3D DDA). Returns Infinity if none.
export function rayHitDistance(grid, origin, dir, maxDist) {
  const c = grid.cell;
  let p = [(origin[0] - grid.origin[0]) / c, (origin[1] - grid.origin[1]) / c, (origin[2] - grid.origin[2]) / c];
  let ijk = p.map(Math.floor);
  const step = dir.map((d) => (d > 0 ? 1 : d < 0 ? -1 : 0));
  const tDelta = dir.map((d) => (d !== 0 ? Math.abs(1 / d) * c : Infinity));
  const tMax = [0, 1, 2].map((q) => {
    if (dir[q] === 0) return Infinity;
    const frac = p[q] - ijk[q];
    return (dir[q] > 0 ? 1 - frac : frac) * tDelta[q];
  });
  let t = 0;
  while (t <= maxDist) {
    if (grid.get(ijk[0], ijk[1], ijk[2])) return t;
    const q = tMax[0] < tMax[1] ? (tMax[0] < tMax[2] ? 0 : 2) : tMax[1] < tMax[2] ? 1 : 2;
    t = tMax[q];
    tMax[q] += tDelta[q];
    ijk[q] += step[q];
    if (ijk[q] < -1 || ijk[q] > [grid.nx, grid.ny, grid.nz][q]) return Infinity;
  }
  return Infinity;
}

// Closed quad surface of a voxel solid. Faces are greedily merged in-plane up to
// `maxMerge` cells per side to reduce panel count for the BEM solver.
export function voxelSurfacePanels(grid, maxMerge = 2) {
  const { nx, ny, nz, cell } = grid;
  const n = [nx, ny, nz];
  const panels = [];
  for (let a = 0; a < 3; a++) {
    const b = (a + 1) % 3, c = (a + 2) % 3;
    for (const s of [1, -1]) {
      for (let ia = 0; ia < n[a]; ia++) {
        // mask of exposed faces in this slice
        const mask = new Uint8Array(n[b] * n[c]);
        for (let ic = 0; ic < n[c]; ic++)
          for (let ib = 0; ib < n[b]; ib++) {
            const ijk = [0, 0, 0];
            ijk[a] = ia; ijk[b] = ib; ijk[c] = ic;
            if (!grid.get(ijk[0], ijk[1], ijk[2])) continue;
            const nb = ijk.slice(); nb[a] += s;
            if (!grid.get(nb[0], nb[1], nb[2])) mask[ib + n[b] * ic] = 1;
          }
        // greedy merge
        for (let ic = 0; ic < n[c]; ic++)
          for (let ib = 0; ib < n[b]; ib++) {
            if (!mask[ib + n[b] * ic]) continue;
            let w = 1;
            while (w < maxMerge && ib + w < n[b] && mask[ib + w + n[b] * ic]) w++;
            let hgt = 1;
            outer: while (hgt < maxMerge && ic + hgt < n[c]) {
              for (let q = 0; q < w; q++) if (!mask[ib + q + n[b] * (ic + hgt)]) break outer;
              hgt++;
            }
            for (let r = 0; r < hgt; r++) for (let q = 0; q < w; q++) mask[ib + q + n[b] * (ic + r)] = 0;
            const center = [0, 0, 0];
            center[a] = grid.origin[a] + (ia + (s > 0 ? 1 : 0)) * cell;
            center[b] = grid.origin[b] + (ib + w / 2) * cell;
            center[c] = grid.origin[c] + (ic + hgt / 2) * cell;
            const normal = [0, 0, 0];
            normal[a] = s;
            panels.push({ c: center, n: normal, axis: a, hb: (w * cell) / 2, hc: (hgt * cell) / 2, area: w * hgt * cell * cell });
          }
      }
    }
  }
  return panels;
}

// Mesh used for rendering a voxel grid (debug / CFD solid preview): triangle soup.
export function voxelSurfaceTriangles(grid) {
  const panels = voxelSurfacePanels(grid, 64);
  const out = new Float32Array(panels.length * 18);
  let o = 0;
  for (const p of panels) {
    const a = p.axis, b = (a + 1) % 3, c = (a + 2) % 3;
    const corner = (sb, sc) => { const v = p.c.slice(); v[b] += sb * p.hb; v[c] += sc * p.hc; return v; };
    let q = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)];
    if (p.n[a] < 0) q = [q[0], q[3], q[2], q[1]];
    for (const v of [q[0], q[1], q[2], q[0], q[2], q[3]]) { out[o++] = v[0]; out[o++] = v[1]; out[o++] = v[2]; }
  }
  return out;
}

// ---------------------------------------------------------------------------
// Smooth hull extraction (for BEM / visual previews)
// ---------------------------------------------------------------------------

// Fractional occupancy per cell (supersampled 2x2x2), with thin features that
// intersect a cell lifted just above the iso level so they are not lost.
export function occupancyField(positions, grid, { keepThin = true } = {}) {
  const fine = new VoxelGrid(grid.nx * 2, grid.ny * 2, grid.nz * 2, grid.cell / 2, grid.origin.slice());
  voxelizeSolid(positions, fine);
  const f = new Float32Array(grid.nx * grid.ny * grid.nz);
  for (let k = 0; k < grid.nz; k++)
    for (let j = 0; j < grid.ny; j++)
      for (let i = 0; i < grid.nx; i++) {
        let s = 0;
        for (let dk = 0; dk < 2; dk++) for (let dj = 0; dj < 2; dj++) for (let di = 0; di < 2; di++) s += fine.get(2 * i + di, 2 * j + dj, 2 * k + dk);
        f[i + grid.nx * (j + grid.ny * k)] = s / 8;
      }
  if (keepThin) {
    // Thin sheets (plates, duct walls thinner than about a cell) have no cell
    // with half its volume inside, so they would vanish from the iso-surface.
    // Cells next to such a sheet are lifted just above the iso level. A sheet
    // is detected geometrically: an axis ray through the cell centre crosses
    // the surface twice (in and out) within 1.2 cells. Surfaces of thick parts
    // are left alone, so their iso-surface stays where the occupancy puts it.
    const surf = new VoxelGrid(grid.nx, grid.ny, grid.nz, grid.cell, grid.origin.slice());
    voxelizeSurface(positions, surf);
    const thin = thinSheetCells(positions, grid, f, surf.data);
    const { nx, ny, nz } = grid;
    const nearThin = (i, j, k) => {
      for (let dk = -1; dk <= 1; dk++) for (let dj = -1; dj <= 1; dj++) for (let di = -1; di <= 1; di++) {
        const a = i + di, b = j + dj, c = k + dk;
        if (a < 0 || b < 0 || c < 0 || a >= nx || b >= ny || c >= nz) continue;
        if (thin[a + nx * (b + ny * c)]) return true;
      }
      return false;
    };
    const lift = [];
    for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
      const id = i + nx * (j + ny * k);
      // bridge: surface cells touching a sheet join it to the part it is fixed to
      if (thin[id] || (surf.data[id] && f[id] < 0.55 && nearThin(i, j, k))) lift.push(id);
    }
    for (const id of lift) f[id] = Math.max(f[id], 0.55);
  }
  return f;
}

// Cells (touched by the surface, less than ~half full) that sit on a sheet
// thinner than 1.2 cells: along one axis a ray through the centre enters and
// leaves the solid within that distance.
export function thinSheetCells(positions, grid, occ, touched) {
  const { nx, ny, nz, cell } = grid;
  const o = grid.origin;
  const N = nx * ny * nz, tri = positions.length / 9;
  // triangles binned on the grid (CSR)
  const count = new Uint32Array(N + 1);
  const range = (t, a) => {
    const v0 = positions[9 * t + a], v1 = positions[9 * t + 3 + a], v2 = positions[9 * t + 6 + a];
    const n = a === 0 ? nx : a === 1 ? ny : nz;
    return [Math.max(0, Math.floor((Math.min(v0, v1, v2) - o[a]) / cell)), Math.min(n - 1, Math.floor((Math.max(v0, v1, v2) - o[a]) / cell))];
  };
  const forBins = (t, fn) => {
    const [a0, a1] = range(t, 0), [b0, b1] = range(t, 1), [c0, c1] = range(t, 2);
    for (let k = c0; k <= c1; k++) for (let j = b0; j <= b1; j++) for (let i = a0; i <= a1; i++) fn(i + nx * (j + ny * k));
  };
  for (let t = 0; t < tri; t++) forBins(t, (b) => count[b + 1]++);
  for (let b = 0; b < N; b++) count[b + 1] += count[b];
  const items = new Uint32Array(count[N]);
  const fill = count.slice(0, N);
  for (let t = 0; t < tri; t++) forBins(t, (b) => { items[fill[b]++] = t; });
  // ray (axis a through point p) - triangle intersection parameter along the axis
  const cross = (t, a, p) => {
    const b = (a + 1) % 3, c = (a + 2) % 3, q = 9 * t;
    const B0 = positions[q + b], B1 = positions[q + 3 + b], B2 = positions[q + 6 + b];
    const C0 = positions[q + c], C1 = positions[q + 3 + c], C2 = positions[q + 6 + c];
    const w0 = (B2 - B1) * (p[c] - C1) - (C2 - C1) * (p[b] - B1);
    const w1 = (B0 - B2) * (p[c] - C2) - (C0 - C2) * (p[b] - B2);
    const w2 = (B1 - B0) * (p[c] - C0) - (C1 - C0) * (p[b] - B0);
    const area = w0 + w1 + w2;
    if (area === 0) return NaN;
    if (area > 0 ? w0 < 0 || w1 < 0 || w2 < 0 : w0 > 0 || w1 > 0 || w2 > 0) return NaN;
    return (w0 * positions[q + a] + w1 * positions[q + 3 + a] + w2 * positions[q + 6 + a]) / area - p[a];
  };
  const thin = new Uint8Array(N);
  const reach = 1.5 * cell, span = 1.2 * cell;
  const hits = [];
  for (let k = 0; k < nz; k++) for (let j = 0; j < ny; j++) for (let i = 0; i < nx; i++) {
    const id = i + nx * (j + ny * k);
    if (!touched[id] || occ[id] >= 0.55) continue;
    const p = [o[0] + (i + 0.5) * cell, o[1] + (j + 0.5) * cell, o[2] + (k + 0.5) * cell];
    let isThin = false;
    for (let a = 0; a < 3 && !isThin; a++) {
      hits.length = 0;
      const idx = [i, j, k];
      for (let d = -2; d <= 2; d++) {
        const cidx = idx.slice(); cidx[a] += d;
        if (cidx[a] < 0 || cidx[a] >= (a === 0 ? nx : a === 1 ? ny : nz)) continue;
        const bin = cidx[0] + nx * (cidx[1] + ny * cidx[2]);
        for (let n = count[bin]; n < count[bin + 1]; n++) {
          const s = cross(items[n], a, p);
          if (s === s && Math.abs(s) <= reach) hits.push(s);
        }
      }
      if (hits.length < 2) continue;
      hits.sort((x, y) => x - y);
      // dedupe (a triangle seen from two bins, or shared edges)
      const u = [hits[0]];
      for (let m = 1; m < hits.length; m++) if (hits[m] - u[u.length - 1] > 1e-6 * cell) u.push(hits[m]);
      for (let m = 0; m + 1 < u.length && !isThin; m++) if (u[m + 1] - u[m] < span && u[m] <= 0.6 * cell && u[m + 1] >= -0.6 * cell) isThin = true;
    }
    if (isThin) thin[id] = 1;
  }
  return thin;
}

// Naive surface nets: returns quads [{v:[p0,p1,p2,p3]}] with outward winding
// (field > iso is solid). Field samples live at cell centres of `grid`.
export function surfaceNets(field, grid, iso = 0.5) {
  const { nx, ny, nz } = grid;
  const F = (i, j, k) => (i < 0 || j < 0 || k < 0 || i >= nx || j >= ny || k >= nz ? 0 : field[i + nx * (j + ny * k)]);
  const P = (i, j, k) => grid.center(i, j, k);
  const vid = new Int32Array((nx + 1) * (ny + 1) * (nz + 1)).fill(-1);
  const verts = [];
  const dkey = (i, j, k) => i + 1 + (nx + 1) * (j + 1 + (ny + 1) * (k + 1));
  const corners = [];
  for (let c = 0; c < 8; c++) corners.push([c & 1, (c >> 1) & 1, (c >> 2) & 1]);
  const edges = [];
  for (let a = 0; a < 8; a++) for (let b = a + 1; b < 8; b++) {
    const d = corners[a].map((v, q) => Math.abs(v - corners[b][q]));
    if (d[0] + d[1] + d[2] === 1) edges.push([a, b]);
  }
  // dual cells (i..i+1, j..j+1, k..k+1) for i in [-1, nx-1]
  for (let k = -1; k < nz; k++)
    for (let j = -1; j < ny; j++)
      for (let i = -1; i < nx; i++) {
        const vals = corners.map(([a, b, c]) => F(i + a, j + b, k + c));
        let mask = 0;
        vals.forEach((v, q) => { if (v > iso) mask |= 1 << q; });
        if (mask === 0 || mask === 255) continue;
        let sx = 0, sy = 0, sz = 0, n = 0;
        for (const [a, b] of edges) {
          const va = vals[a], vb = vals[b];
          if ((va > iso) === (vb > iso)) continue;
          const t = (iso - va) / (vb - va);
          const pa = P(i + corners[a][0], j + corners[a][1], k + corners[a][2]);
          const pb = P(i + corners[b][0], j + corners[b][1], k + corners[b][2]);
          sx += pa[0] + t * (pb[0] - pa[0]); sy += pa[1] + t * (pb[1] - pa[1]); sz += pa[2] + t * (pb[2] - pa[2]);
          n++;
        }
        vid[dkey(i, j, k)] = verts.length;
        verts.push([sx / n, sy / n, sz / n]);
      }
  const quads = [];
  // sample edges between (i,j,k) and its +axis neighbour
  for (let a = 0; a < 3; a++) {
    const b = (a + 1) % 3, c = (a + 2) % 3;
    for (let k = -1; k < nz; k++)
      for (let j = -1; j < ny; j++)
        for (let i = -1; i < nx; i++) {
          const p = [i, j, k];
          const q = p.slice(); q[a] += 1;
          const inA = F(p[0], p[1], p[2]) > iso, inB = F(q[0], q[1], q[2]) > iso;
          if (inA === inB) continue;
          // 4 dual cells sharing this edge: offsets in b and c of 0 / -1
          const cell = (db, dc) => { const d = p.slice(); d[b] += db; d[c] += dc; return vid[dkey(d[0], d[1], d[2])]; };
          const v00 = cell(-1, -1), v10 = cell(0, -1), v11 = cell(0, 0), v01 = cell(-1, 0);
          if (v00 < 0 || v10 < 0 || v11 < 0 || v01 < 0) continue;
          // normal points from solid to empty: +a if A is inside
          const ring = inA ? [v00, v10, v11, v01] : [v00, v01, v11, v10];
          quads.push(ring.map((id) => verts[id]));
        }
  }
  return quads;
}

// Planar panels (centroid, unit normal, area, polygon) from quads.
export function quadPanels(quads) {
  const panels = [];
  for (const q of quads) {
    const c = [0, 1, 2].map((k) => (q[0][k] + q[1][k] + q[2][k] + q[3][k]) / 4);
    const d1 = [0, 1, 2].map((k) => q[2][k] - q[0][k]), d2 = [0, 1, 2].map((k) => q[3][k] - q[1][k]);
    const nrm = vcross(d1, d2);
    const l = Math.hypot(nrm[0], nrm[1], nrm[2]);
    if (l < 1e-14) continue;
    const area = l / 2;
    const n = nrm.map((v) => v / l);
    // project vertices onto the mean plane
    const verts = q.map((p) => { const d = (p[0] - c[0]) * n[0] + (p[1] - c[1]) * n[1] + (p[2] - c[2]) * n[2]; return [p[0] - d * n[0], p[1] - d * n[1], p[2] - d * n[2]]; });
    panels.push({ c, n, area, verts });
  }
  return panels;
}

export function quadsToTriangles(quads) {
  const out = new Float32Array(quads.length * 18);
  let o = 0;
  for (const q of quads) for (const v of [q[0], q[1], q[2], q[0], q[2], q[3]]) { out[o++] = v[0]; out[o++] = v[1]; out[o++] = v[2]; }
  return out;
}
