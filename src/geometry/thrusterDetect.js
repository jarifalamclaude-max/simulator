// Thruster detection on an uploaded design: finds the thrusters in the hull
// mesh and proposes a motor layout for the custom frame builder.
//
//  1. Part names (OBJ o/g, multi-solid ASCII STL): "T200_3", "Thruster 5",
//     "LeftMotor2", ".../T200 (4)/Nozzle" mark thruster parts; the number is
//     the motor number, "T200"/"T500" the thruster model.
//  2. Shape, for files without usable names: vertices are welded, the mesh is
//     split into connected parts, overlapping parts are grouped (nozzle +
//     motor + propeller) and each group is fitted with a surface of revolution
//     (the axis is the line every wall normal passes through). A duct has a
//     complete outer wall and a complete inner wall facing the other way; its
//     outside diameter classifies it as a Blue Robotics T200 (nozzle ~100 mm),
//     T500 (~140 mm class) or a generic thruster. Solid rods, pipes, housings
//     and the hull are rejected (solid, too long for the diameter, too big).
//     Ducts fused into the hull (one connected mesh) are found with a circle
//     Hough transform of the wall faces about the body axes and 45° diagonals.
//  3. Direction: the shape only gives the axis. Its sign and the motor order
//     come from the ArduSub factor table of FRAME_CONFIG (or another frame
//     with as many motors): every thruster's force and torque must match the
//     factor row of its motor slot, signs included. Without a match vertical
//     thrusters push down, horizontal ones away from the centre, flagged
//     low-confidence.
//
// Positions are in the vehicle frame (FRD, metres, model origin), i.e. after
// the unit/axis transform chosen at upload. Pure JS (no DOM).

import { FRAMES } from '../alloc/frames.js';
import { vnorm, vcross, vsub } from '../core/math.js';

const S2 = Math.SQRT1_2;
export const MAX_MOTORS = 12;

// Nominal sizes (Blue Robotics drawings, approximate): nozzle outside
// diameter and propeller diameter (the T200 is ~113 mm long overall).
export const THRUSTER_TYPES = {
  t200: { label: 'T200', od: 0.1, prop: 0.076, preset: 't200-16v' },
  t500: { label: 'T500', od: 0.14, prop: 0.104, preset: 't500-20v' },
  generic: { label: 'Thruster', od: null, prop: null, preset: 'generic-small' },
};

const LIMITS = {
  minOD: 0.03, maxOD: 0.3, // m
  maxDuctAspect: 1.6, // duct length / OD
  maxAspect: 2.6, // whole group (duct + motor) length / OD
  wall: [0.55, 0.97], // ID / OD
  maxGroup: 0.45, // m: anything bigger is hull / frame
  coverage: 0.7, // fraction of the circumference a wall must cover
};

// ---------------------------------------------------------------- names

const KEYWORD = /^(thrusters?|motors?|props?|propellers?|nozzles?|ducts?)(\d*)$/;
const MODEL = /^(t200|t500)$/;
const NOT_THRUSTER = /^(mounts?|mounting|brackets?|holders?|clamps?|screws?|bolts?|nuts?|washers?|cables?|wires?|penetrators?|spacers?|standoffs?|plates?)$/;

const tokensOf = (s) => s.replace(/([a-z0-9])([A-Z])/g, '$1 $2').toLowerCase().split(/[^a-z0-9]+/).filter(Boolean);

// "T200_3" -> {thruster: true, type: 't200', number: 3}. Path segments
// ("Thrusters/T200 (4)/Nozzle") are searched from the innermost one.
export function thrusterNameInfo(name) {
  const none = { thruster: false, type: null, number: null };
  const segs = String(name || '').split(/[/\\:|]+/).map(tokensOf).filter((t) => t.length);
  let hit = false, type = null, number = null;
  for (let s = segs.length - 1; s >= 0; s--) {
    const tk = segs[s];
    let at = -1;
    for (let i = 0; i < tk.length; i++) {
      if (NOT_THRUSTER.test(tk[i])) return none;
      if (MODEL.test(tk[i])) { type = type || tk[i]; if (at < 0) at = i; continue; }
      const m = KEYWORD.exec(tk[i]);
      if (m) { if (at < 0) at = i; if (m[2] && number === null) number = +m[2]; }
    }
    if (at < 0) continue;
    hit = true;
    // a stand-alone number in the same segment, after the keyword first
    const num = (i) => (/^\d+$/.test(tk[i]) ? +tk[i] : null);
    for (let i = at + 1; i < tk.length && number === null; i++) number = num(i);
    for (let i = at - 1; i >= 0 && number === null; i--) number = num(i);
  }
  if (!hit) return none;
  return { thruster: true, type, number: number >= 1 && number <= MAX_MOTORS ? number : null };
}

// short name for a thruster part: the innermost path segment that carries a
// motor number ("Assembly/T200_3/Nozzle" -> "T200_3"), else one that names a thruster
export function thrusterLabel(name) {
  const segs = String(name || '').split(/[/\\:|]+/).map((x) => x.trim()).filter(Boolean);
  const pick = [...segs].reverse().find((x) => thrusterNameInfo(x).number) || [...segs].reverse().find((x) => thrusterNameInfo(x).thruster) || segs[segs.length - 1] || 'Thruster';
  return pick.slice(0, 24);
}

// ---------------------------------------------------------------- mesh helpers

function bboxOf(positions, tris) {
  const min = [Infinity, Infinity, Infinity], max = [-Infinity, -Infinity, -Infinity];
  const visit = (t) => {
    for (let q = 0; q < 9; q++) {
      const v = positions[t * 9 + q], k = q % 3;
      if (v < min[k]) min[k] = v;
      if (v > max[k]) max[k] = v;
    }
  };
  if (tris) for (let i = 0; i < tris.length; i++) visit(tris[i]);
  else for (let t = 0, n = positions.length / 9; t < n; t++) visit(t);
  return { min, max, size: [0, 1, 2].map((k) => max[k] - min[k]), center: [0, 1, 2].map((k) => (max[k] + min[k]) / 2) };
}

function basis(a) {
  const t = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = vnorm(vcross(a, t));
  return [u, vcross(a, u)];
}

// Weld coincident vertices (quantised positions, open-addressing hash) and
// label the connected triangle sets. comp[t] = component of triangle t,
// vid[3t + k] = welded vertex of corner k.
export function meshComponents(positions, tolRel = 2e-6) {
  const nT = Math.floor(positions.length / 9), nV = nT * 3;
  const bb = bboxOf(positions, null);
  const inv = 1 / (Math.max(...bb.size, 1e-9) * tolRel);
  let cap = 1024;
  while (cap < nV / 2) cap <<= 1;
  let table = new Int32Array(cap).fill(-1);
  let kx = new Int32Array(cap), ky = new Int32Array(cap), kz = new Int32Array(cap);
  const hash = (x, y, z, mask) => (Math.imul(x, 73856093) ^ Math.imul(y, 19349663) ^ Math.imul(z, 83492791)) & mask;
  const vid = new Int32Array(nV);
  let count = 0;
  for (let i = 0; i < nV; i++) {
    const x = Math.round((positions[3 * i] - bb.min[0]) * inv), y = Math.round((positions[3 * i + 1] - bb.min[1]) * inv), z = Math.round((positions[3 * i + 2] - bb.min[2]) * inv);
    let h = hash(x, y, z, cap - 1);
    for (;;) {
      const id = table[h];
      if (id < 0) {
        table[h] = count; kx[count] = x; ky[count] = y; kz[count] = z; vid[i] = count++;
        break;
      }
      if (kx[id] === x && ky[id] === y && kz[id] === z) { vid[i] = id; break; }
      h = (h + 1) & (cap - 1);
    }
    if (count > cap * 0.6) { // grow and rehash
      const n2 = cap * 2;
      const t2 = new Int32Array(n2).fill(-1);
      const grow = (a) => { const b = new Int32Array(n2); b.set(a); return b; };
      kx = grow(kx); ky = grow(ky); kz = grow(kz);
      for (let id = 0; id < count; id++) {
        let g = hash(kx[id], ky[id], kz[id], n2 - 1);
        while (t2[g] >= 0) g = (g + 1) & (n2 - 1);
        t2[g] = id;
      }
      table = t2; cap = n2;
    }
  }
  const parent = new Int32Array(count);
  for (let i = 0; i < count; i++) parent[i] = i;
  const find = (a) => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
  const unite = (a, b) => { a = find(a); b = find(b); if (a !== b) parent[a < b ? b : a] = a < b ? a : b; };
  for (let t = 0; t < nT; t++) { unite(vid[3 * t], vid[3 * t + 1]); unite(vid[3 * t], vid[3 * t + 2]); }
  const label = new Int32Array(count).fill(-1);
  const comp = new Int32Array(nT);
  let nc = 0;
  for (let t = 0; t < nT; t++) {
    const r = find(vid[3 * t]);
    if (label[r] < 0) label[r] = nc++;
    comp[t] = label[r];
  }
  return { comp, count: nc, vertices: count, vid };
}

// Connected parts of the kept triangles; parts that overlap (a motor inside
// its nozzle, the propeller, a strut) are grouped. Parts bigger than a
// thruster (hull, frame plates) are returned separately and never grouped.
function groupParts(positions, comps, keep, modelExtent) {
  const { comp, count } = comps;
  const n = new Int32Array(count);
  for (let t = 0; t < comp.length; t++) if (keep[t]) n[comp[t]]++;
  const off = new Int32Array(count + 1);
  for (let c = 0; c < count; c++) off[c + 1] = off[c] + n[c];
  const tris = new Int32Array(off[count]);
  const fill = off.slice(0, count);
  for (let t = 0; t < comp.length; t++) if (keep[t]) tris[fill[comp[t]]++] = t;
  const small = [], big = [];
  for (let c = 0; c < count; c++) {
    if (!n[c]) continue;
    const list = tris.subarray(off[c], off[c + 1]);
    const bb = bboxOf(positions, list);
    const e = Math.max(...bb.size);
    const item = { tris: list, bb };
    if (e > LIMITS.maxGroup || (e > 0.2 && e > 0.6 * modelExtent && count > 1)) big.push(item); else small.push(item);
  }
  // union of parts where the smaller one sits mostly (>= 50 % of its box)
  // inside the bigger one; a bar running between two thrusters is not smaller
  const pad = 0.001;
  const vol = (b) => (b.size[0] + 2 * pad) * (b.size[1] + 2 * pad) * (b.size[2] + 2 * pad);
  const overlap = (a, b) => {
    let v = 1;
    for (let k = 0; k < 3; k++) {
      const lo = Math.max(a.min[k], b.min[k]) - pad, hi = Math.min(a.max[k], b.max[k]) + pad;
      if (hi <= lo) return 0;
      v *= hi - lo;
    }
    const [s, l] = vol(a) <= vol(b) ? [a, b] : [b, a];
    return Math.max(...s.size) <= 1.1 * Math.max(...l.size) ? v / vol(s) : 0;
  };
  const order = small.map((_, i) => i).sort((i, j) => small[i].bb.min[0] - small[j].bb.min[0]);
  const parent = small.map((_, i) => i);
  const find = (a) => { while (parent[a] !== a) { parent[a] = parent[parent[a]]; a = parent[a]; } return a; };
  for (let p = 0; p < order.length; p++) {
    const A = small[order[p]].bb;
    for (let q = p + 1; q < order.length; q++) {
      const B = small[order[q]].bb;
      if (B.min[0] > A.max[0] + pad) break;
      if (overlap(A, B) >= 0.5) { const a = find(order[p]), b = find(order[q]); if (a !== b) parent[b] = a; }
    }
  }
  const groups = new Map();
  small.forEach((it, i) => { const r = find(i); if (!groups.has(r)) groups.set(r, []); groups.get(r).push(it); });
  const clusters = [...groups.values()].map((list) => {
    const len = list.reduce((s, it) => s + it.tris.length, 0);
    const t = new Int32Array(len);
    let o = 0;
    for (const it of list) { t.set(it.tris, o); o += it.tris.length; }
    return { tris: t, bb: bboxOf(positions, t) };
  });
  return { clusters, big };
}

// Faces of a triangle set, relative to its box centre (sampled when huge).
function faceSet(positions, tris, maxN = 40000) {
  const n0 = tris.length, step = Math.max(1, n0 / maxN), n = Math.min(n0, Math.ceil(n0 / step));
  const o = bboxOf(positions, tris).center;
  const A = new Float64Array(n), N = new Float64Array(3 * n), C = new Float64Array(3 * n), V = new Float64Array(9 * n);
  let k = 0;
  for (let s = 0; s < n; s++) {
    const b = tris[Math.min(n0 - 1, Math.floor(s * step))] * 9;
    for (let q = 0; q < 9; q++) V[9 * k + q] = positions[b + q] - o[q % 3];
    const p = 9 * k;
    const ax = V[p + 3] - V[p], ay = V[p + 4] - V[p + 1], az = V[p + 5] - V[p + 2];
    const bx = V[p + 6] - V[p], by = V[p + 7] - V[p + 1], bz = V[p + 8] - V[p + 2];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const l = Math.hypot(nx, ny, nz);
    if (!(l > 1e-14)) continue;
    A[k] = 0.5 * l;
    N[3 * k] = nx / l; N[3 * k + 1] = ny / l; N[3 * k + 2] = nz / l;
    for (let q = 0; q < 3; q++) C[3 * k + q] = (V[p + q] + V[p + 3 + q] + V[p + 6 + q]) / 3;
    k++;
  }
  return { n: k, A, N, C, V, o };
}

// Eigen-decomposition of a symmetric 3x3 matrix (cyclic Jacobi); vectors are columns.
export function eigSym3(M) {
  const A = M.map((r) => r.slice());
  const V = [[1, 0, 0], [0, 1, 0], [0, 0, 1]];
  for (let sweep = 0; sweep < 40; sweep++) {
    const off = A[0][1] ** 2 + A[0][2] ** 2 + A[1][2] ** 2;
    if (off <= 1e-30 * (A[0][0] ** 2 + A[1][1] ** 2 + A[2][2] ** 2 + 1e-300)) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      const apq = A[p][q];
      if (Math.abs(apq) < 1e-300) continue;
      const th = (A[q][q] - A[p][p]) / (2 * apq);
      const t = Math.sign(th || 1) / (Math.abs(th) + Math.sqrt(th * th + 1));
      const c = 1 / Math.sqrt(t * t + 1), s = t * c;
      for (let k = 0; k < 3; k++) { const x = A[k][p], y = A[k][q]; A[k][p] = c * x - s * y; A[k][q] = s * x + c * y; }
      for (let k = 0; k < 3; k++) { const x = A[p][k], y = A[q][k]; A[p][k] = c * x - s * y; A[q][k] = s * x + c * y; }
      for (let k = 0; k < 3; k++) { const x = V[k][p], y = V[k][q]; V[k][p] = c * x - s * y; V[k][q] = s * x + c * y; }
    }
  }
  return { values: [A[0][0], A[1][1], A[2][2]], vectors: [0, 1, 2].map((j) => [V[0][j], V[1][j], V[2][j]]) };
}

// ---------------------------------------------------------------- surface of revolution

// Welded vertices of a face group with angle-weighted normals (independent of
// how quads are split). On a surface of revolution every vertex normal lies in
// the meridian plane, also at sharp rims, so its line meets the axis exactly.
function vertexSet(positions, tris, vid, o, maxN = 12000) {
  const local = new Map();
  const acc = [];
  const p = positions;
  for (let i = 0; i < tris.length; i++) {
    const b = tris[i] * 9;
    const ax = p[b + 3] - p[b], ay = p[b + 4] - p[b + 1], az = p[b + 5] - p[b + 2];
    const bx = p[b + 6] - p[b], by = p[b + 7] - p[b + 1], bz = p[b + 8] - p[b + 2];
    let nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const l = Math.hypot(nx, ny, nz);
    if (!(l > 1e-14)) continue;
    nx /= l; ny /= l; nz /= l;
    for (let q = 0; q < 3; q++) {
      // corner angle at vertex q
      const c0 = b + 3 * q, c1 = b + 3 * ((q + 1) % 3), c2 = b + 3 * ((q + 2) % 3);
      const ex = p[c1] - p[c0], ey = p[c1 + 1] - p[c0 + 1], ez = p[c1 + 2] - p[c0 + 2];
      const fx = p[c2] - p[c0], fy = p[c2 + 1] - p[c0 + 1], fz = p[c2 + 2] - p[c0 + 2];
      const ang = Math.atan2(Math.hypot(ey * fz - ez * fy, ez * fx - ex * fz, ex * fy - ey * fx), ex * fx + ey * fy + ez * fz);
      const g = vid[tris[i] * 3 + q];
      let k = local.get(g);
      if (k === undefined) { k = acc.length / 8; local.set(g, k); acc.push(p[c0] - o[0], p[c0 + 1] - o[1], p[c0 + 2] - o[2], 0, 0, 0, 0, 0); }
      acc[8 * k + 3] += ang * nx; acc[8 * k + 4] += ang * ny; acc[8 * k + 5] += ang * nz; acc[8 * k + 6] += ang; acc[8 * k + 7] += l / 6;
    }
  }
  const nv = acc.length / 8, step = Math.max(1, nv / maxN), n = Math.min(nv, Math.ceil(nv / step));
  const A = new Float64Array(n), N = new Float64Array(3 * n), C = new Float64Array(3 * n);
  let m = 0;
  for (let s = 0; s < n; s++) {
    const k = Math.min(nv - 1, Math.floor(s * step)), l = Math.hypot(acc[8 * k + 3], acc[8 * k + 4], acc[8 * k + 5]);
    if (!(l > 0.3 * acc[8 * k + 6])) continue; // the two sides of a blade: no usable normal
    A[m] = acc[8 * k + 7];
    for (let q = 0; q < 3; q++) { C[3 * m + q] = acc[8 * k + q]; N[3 * m + q] = acc[8 * k + 3 + q] / l; }
    m++;
  }
  return { n: m, A, N, C, o };
}

// in-plane data of a point set for a trial axis: tangent k, k·q, weight, q, index
function planeData(P, a) {
  const [u, v] = basis(a);
  const K = P.scratch || (P.scratch = new Float64Array(7 * P.n));
  let m = 0;
  for (let i = 0; i < P.n; i++) {
    const nx = P.N[3 * i], ny = P.N[3 * i + 1], nz = P.N[3 * i + 2];
    const nu = nx * u[0] + ny * u[1] + nz * u[2], nv = nx * v[0] + ny * v[1] + nz * v[2];
    const ip = Math.hypot(nu, nv);
    if (ip < 0.2) continue; // end faces: no information about the centre
    const cx = P.C[3 * i], cy = P.C[3 * i + 1], cz = P.C[3 * i + 2];
    const qu = cx * u[0] + cy * u[1] + cz * u[2], qv = cx * v[0] + cy * v[1] + cz * v[2];
    const ku = -nv / ip, kv = nu / ip, o = 7 * m++;
    K[o] = ku; K[o + 1] = kv; K[o + 2] = ku * qu + kv * qv; K[o + 3] = P.A[i] * ip; K[o + 4] = qu; K[o + 5] = qv; K[o + 6] = i;
  }
  return { u, v, K, m };
}
// distance from c to the normal line of point j; Infinity when the normal
// does not point along the radius (a plate side, a strut)
function missOf(K, j, cu, cv) {
  const o = 7 * j, du = K[o + 4] - cu, dv = K[o + 5] - cv, d = Math.hypot(du, dv);
  if (d > 0.002 && Math.abs(K[o + 1] * du - K[o] * dv) < 0.9 * d) return Infinity;
  return Math.abs(K[o] * du + K[o + 1] * dv);
}

// For a trial axis a: the point c (in the plane normal to a) closest to every
// normal line, by least squares trimmed of the lines that miss it (mounting
// plates, struts, blades). R = mean squared miss, capped at the trim distance.
function solveCentre(P, a) {
  const { u, v, K, m } = planeData(P, a);
  let cu = 0, cv = 0, tau = Infinity;
  for (let it = 0; it < 3; it++) {
    let Suu = 0, Suv = 0, Svv = 0, bu = 0, bv = 0, W = 0;
    for (let j = 0; j < m; j++) {
      if (it && missOf(K, j, cu, cv) > tau) continue;
      const ku = K[7 * j], kv = K[7 * j + 1], d = K[7 * j + 2], w = K[7 * j + 3];
      Suu += w * ku * ku; Suv += w * ku * kv; Svv += w * kv * kv; bu += w * ku * d; bv += w * kv * d; W += w;
    }
    const det = Suu * Svv - Suv * Suv;
    if (!(W > 0) || det <= 1e-3 * W * W) return null; // normals (nearly) all parallel
    cu = (Svv * bu - Suv * bv) / det; cv = (Suu * bv - Suv * bu) / det;
    // trim distance: a few % of the typical radius about the new centre
    let rr = 0, ww = 0;
    for (let j = 0; j < m; j++) { const w = K[7 * j + 3]; rr += w * Math.hypot(K[7 * j + 4] - cu, K[7 * j + 5] - cv); ww += w; }
    tau = Math.max(0.0005, 0.01 * (rr / ww)) * (it === 0 ? 6 : 1);
  }
  let R = 0, WA = 0;
  for (let j = 0; j < m; j++) {
    const e = missOf(K, j, cu, cv), w = K[7 * j + 3];
    R += w * Math.min(e * e, tau * tau); WA += w;
  }
  return { a, u, v, cu, cv, R: R / WA, tau };
}

// Area fraction consistent with a surface of revolution about (a, c): points
// whose normal line meets the axis, and points on faces normal to the axis.
function revolutionScore(P, f) {
  const { K, m } = planeData(P, f.a);
  const a = f.a;
  let good = 0, tot = 0;
  for (let i = 0; i < P.n; i++) {
    tot += P.A[i];
    if (Math.abs(P.N[3 * i] * a[0] + P.N[3 * i + 1] * a[1] + P.N[3 * i + 2] * a[2]) >= 0.9) good += P.A[i];
  }
  for (let j = 0; j < m; j++) {
    const o = 7 * j, i = K[o + 6];
    if (Math.abs(P.N[3 * i] * a[0] + P.N[3 * i + 1] * a[1] + P.N[3 * i + 2] * a[2]) >= 0.9) continue;
    if (missOf(K, j, f.cu, f.cv) <= Math.max(0.0008, 0.015 * Math.hypot(K[o + 4] - f.cu, K[o + 5] - f.cv))) good += P.A[i];
  }
  return tot > 0 ? good / tot : 0;
}

// polish the axis direction (pattern search on the sphere, minimising R)
function refineAxis(P, a0) {
  let best = solveCentre(P, vnorm(a0));
  if (!best) return null;
  if (best.R <= (0.02 * best.tau) ** 2) return best; // already exact (aligned with a body axis)
  let step = 0.15;
  while (step > 3e-4) {
    let moved = false;
    for (const [p, q] of [[1, 0], [-1, 0], [0, 1], [0, -1]]) {
      const a = vnorm([0, 1, 2].map((k) => best.a[k] + step * (p * best.u[k] + q * best.v[k])));
      const f = solveCentre(P, a);
      if (f && f.R < best.R * (1 - 1e-9) - 1e-18) { best = f; moved = true; break; }
    }
    if (!moved) step *= 0.5;
  }
  return best;
}

// start from the principal directions of the normals (a duct wall's axis is
// the one most perpendicular to them) and the body axes; keep the best fit
function fitRevolution(P) {
  const T = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  for (let i = 0; i < P.n; i++) {
    const n = [P.N[3 * i], P.N[3 * i + 1], P.N[3 * i + 2]];
    for (let r = 0; r < 3; r++) for (let c = 0; c < 3; c++) T[r][c] += P.A[i] * n[r] * n[c];
  }
  const init = [...eigSym3(T).vectors, [1, 0, 0], [0, 1, 0], [0, 0, 1]].filter((a, i, all) => all.findIndex((b) => Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) > 0.999) === i);
  const scored = init.map((a) => { const f = solveCentre(P, a); return f ? { f, s: revolutionScore(P, f) } : null; }).filter(Boolean).sort((x, y) => y.s - x.s);
  let best = null;
  for (const c of scored.filter((c, i) => i === 0 || (i === 1 && c.s >= scored[0].s - 0.05))) {
    const f = refineAxis(P, c.f.a);
    const s = f ? revolutionScore(P, f) : 0;
    const pick = f && s >= c.s - 0.01 ? { f, s } : c; // keep the start if polishing made it worse
    if (!best || pick.s > best.s) best = pick;
  }
  return best;
}

// Walls of a fitted surface of revolution grouped into rings (outer wall,
// inner wall, motor body...), each with radius range, circumference coverage,
// facing (+1 = normal points away from the axis) and axial extent.
function ringsOf(F, f) {
  const { a, u, v, cu, cv } = f;
  const faces = [];
  let zLo = Infinity, zHi = -Infinity;
  for (let i = 0; i < F.n; i++) {
    for (let q = 0; q < 3; q++) {
      const z = F.V[9 * i + 3 * q] * a[0] + F.V[9 * i + 3 * q + 1] * a[1] + F.V[9 * i + 3 * q + 2] * a[2];
      if (z < zLo) zLo = z;
      if (z > zHi) zHi = z;
    }
    const nx = F.N[3 * i], ny = F.N[3 * i + 1], nz = F.N[3 * i + 2];
    const nu = nx * u[0] + ny * u[1] + nz * u[2], nv = nx * v[0] + ny * v[1] + nz * v[2];
    const ip = Math.hypot(nu, nv);
    if (ip < 0.3) continue;
    const cx = F.C[3 * i], cy = F.C[3 * i + 1], cz = F.C[3 * i + 2];
    const du = cx * u[0] + cy * u[1] + cz * u[2] - cu, dv = cx * v[0] + cy * v[1] + cz * v[2] - cv;
    const d = Math.hypot(du, dv);
    if (d < 1e-6) continue;
    const radial = (nu * du + nv * dv) / (ip * d);
    if (Math.abs(radial) < 0.95) continue;
    let r = 0, z0 = Infinity, z1 = -Infinity;
    for (let q = 0; q < 3; q++) {
      const x = F.V[9 * i + 3 * q], y = F.V[9 * i + 3 * q + 1], w = F.V[9 * i + 3 * q + 2];
      r += Math.hypot(x * u[0] + y * u[1] + w * u[2] - cu, x * v[0] + y * v[1] + w * v[2] - cv) / 3;
      const z = x * a[0] + y * a[1] + w * a[2];
      if (z < z0) z0 = z;
      if (z > z1) z1 = z;
    }
    faces.push({ r, A: F.A[i], facing: radial > 0 ? 1 : -1, ang: Math.atan2(dv, du), z0, z1 });
  }
  return { rings: clusterRings(faces), zLo, zHi };
}

// split wall faces into rings by radius (and by facing when the winding is consistent)
function clusterRings(faces) {
  if (!faces.length) return [];
  const rMax = Math.max(...faces.map((x) => x.r));
  let fo = 0, fa = 0;
  for (const x of faces) if (x.r >= 0.97 * rMax) { fo += x.A * x.facing; fa += x.A; }
  const reliable = fa > 0 && Math.abs(fo / fa) >= 0.8;
  const orient = fo >= 0 ? 1 : -1;
  const sets = reliable ? [faces.filter((x) => x.facing * orient > 0), faces.filter((x) => x.facing * orient < 0)] : [faces];
  const rings = [];
  for (const set of sets) {
    set.sort((p, q) => p.r - q.r);
    let cur = [];
    const flush = () => { if (cur.length) rings.push(ringStats(cur, reliable ? orient : 0)); cur = []; };
    for (const x of set) {
      if (cur.length && x.r - cur[cur.length - 1].r > Math.max(0.0015, 0.03 * x.r)) flush();
      cur.push(x);
    }
    flush();
  }
  return rings.sort((p, q) => q.r - p.r);
}

function ringStats(list, orient) {
  const bins = new Float64Array(24);
  let A = 0, rA = 0, fA = 0, rMin = Infinity, rMax = -Infinity, z0 = Infinity, z1 = -Infinity;
  for (const x of list) {
    A += x.A; rA += x.r * x.A; fA += x.facing * x.A;
    rMin = Math.min(rMin, x.r); rMax = Math.max(rMax, x.r); z0 = Math.min(z0, x.z0); z1 = Math.max(z1, x.z1);
    bins[Math.min(23, Math.floor(((x.ang + Math.PI) / (2 * Math.PI)) * 24))] += x.A;
  }
  let filled = 0;
  for (const b of bins) if (b >= 0.15 * (A / 24)) filled++;
  // facing relative to the outward winding (0 = unknown winding)
  return { r: rA / A, rMin, rMax, A, coverage: filled / 24, facing: orient ? (fA / A) * orient : 0, rawFacing: fA / A, z0, z1 };
}

const zOverlap = (p, q) => Math.min(p.z1, q.z1) - Math.max(p.z0, q.z0);

// outer wall + inner wall of a duct among the rings
function pickDuct(rings) {
  const full = rings.filter((g) => g.coverage >= LIMITS.coverage);
  const outer = full.find((g) => g.facing >= 0);
  if (!outer) return { outer: null, inner: null };
  const inner = full.find((g) => g !== outer && g.rMax < outer.rMin && g.r >= LIMITS.wall[0] * outer.r && g.r <= LIMITS.wall[1] * outer.r
    && g.facing <= 0 && zOverlap(g, outer) >= 0.3 * Math.min(g.z1 - g.z0, outer.z1 - outer.z0));
  return { outer, inner: inner || null };
}

// Fit one group of faces: axis, centre, outside / inside diameter, lengths.
function measureGroup(positions, tris, vid) {
  const F = faceSet(positions, tris);
  if (F.n < 8) return null;
  const fit = fitRevolution(vertexSet(positions, tris, vid, F.o));
  if (!fit) return null;
  const { f, s } = fit;
  const { rings, zLo, zHi } = ringsOf(F, f);
  const { outer, inner } = pickDuct(rings);
  const z0 = outer ? Math.min(outer.z0, inner ? inner.z0 : Infinity) : zLo, z1 = outer ? Math.max(outer.z1, inner ? inner.z1 : -Infinity) : zHi;
  const zm = (z0 + z1) / 2;
  const pos = [0, 1, 2].map((k) => F.o[k] + f.u[k] * f.cu + f.v[k] * f.cv + f.a[k] * zm);
  return {
    axis: f.a.slice(), pos, fitScore: s,
    od: outer ? 2 * outer.rMax : null, id: inner ? 2 * inner.rMin : null,
    ductLength: outer ? z1 - z0 : null, length: zHi - zLo,
    coverage: outer ? Math.min(outer.coverage, inner ? inner.coverage : 1) : 0,
    hollow: !!inner,
  };
}

// T200 / T500 / generic from the measured duct; ok: false (with the reason) = not a thruster.
export function classifyDuct({ od, id, ductLength, length }) {
  if (!(od >= LIMITS.minOD && od <= LIMITS.maxOD)) return { ok: false, reason: 'size' };
  if (ductLength / od > LIMITS.maxDuctAspect || (length ?? ductLength) / od > LIMITS.maxAspect) return { ok: false, reason: 'too long for its diameter' };
  if (!id) return { ok: false, reason: 'solid' };
  const ratio = id / od;
  if (ratio < LIMITS.wall[0] || ratio > LIMITS.wall[1]) return { ok: false, reason: 'wall thickness' };
  let type = 'generic', match = 0;
  for (const k of ['t200', 't500']) {
    const T = THRUSTER_TYPES[k];
    const e = Math.abs(od - T.od) / T.od;
    if (e <= 0.15 && id >= T.prop) {
      const m = Math.exp(-((e / 0.07) ** 2));
      if (m > match) { match = m; type = k; }
    }
  }
  return { ok: true, type, match };
}

// ---------------------------------------------------------------- ducts fused into the hull

const HOUGH_AXES = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [S2, S2, 0], [S2, -S2, 0], [S2, 0, S2], [S2, 0, -S2], [0, S2, S2], [0, S2, -S2]];
const POP = Uint8Array.from({ length: 256 }, (_, i) => { let c = 0; for (let b = i; b; b >>= 1) c += b & 1; return c; });

// Circle Hough transform of wall faces for each candidate axis: every wall
// face votes for centres one radius away on both sides of its normal. A full
// ring gathers votes from all directions (8 sectors); flat walls only from one.
function fusedDucts(positions, keep, modelSize) {
  const all = [];
  for (let t = 0; t < keep.length; t++) if (keep[t]) all.push(t);
  const stride = Math.max(1, all.length / 300000);
  const idx = [];
  for (let s = 0; s * stride < all.length; s++) idx.push(all[Math.floor(s * stride)]);
  const nF = idx.length;
  const FN = new Float32Array(3 * nF), FC = new Float64Array(3 * nF), FA = new Float32Array(nF);
  for (let i = 0; i < nF; i++) {
    const b = idx[i] * 9, p = positions;
    const ax = p[b + 3] - p[b], ay = p[b + 4] - p[b + 1], az = p[b + 5] - p[b + 2];
    const bx = p[b + 6] - p[b], by = p[b + 7] - p[b + 1], bz = p[b + 8] - p[b + 2];
    const nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const l = Math.hypot(nx, ny, nz);
    FA[i] = 0.5 * l * stride;
    if (l > 0) { FN[3 * i] = nx / l; FN[3 * i + 1] = ny / l; FN[3 * i + 2] = nz / l; }
    for (let q = 0; q < 3; q++) FC[3 * i + q] = (p[b + q] + p[b + 3 + q] + p[b + 6 + q]) / 3;
  }
  const cell = Math.max(0.004, Math.max(...modelSize) / 220);
  const r0 = 0.015, rTop = 0.09, dr = Math.max(0.002, cell / 2), nr = Math.floor((rTop - r0) / dr) + 1;
  const ducts = [];
  for (const a of HOUGH_AXES) {
    const [u, v] = basis(a);
    // wall faces for this axis (sampled to a bounded number)
    const wall = [];
    for (let i = 0; i < nF; i++) {
      if (!(FA[i] > 0)) continue;
      const n = [FN[3 * i], FN[3 * i + 1], FN[3 * i + 2]];
      if (Math.abs(n[0] * a[0] + n[1] * a[1] + n[2] * a[2]) > 0.5) continue;
      wall.push(i);
    }
    if (wall.length < 24) continue;
    const ws = Math.max(1, wall.length / 80000), nW = Math.min(wall.length, Math.ceil(wall.length / ws));
    const Q = new Float64Array(2 * nW), M = new Float64Array(2 * nW), Z = new Float64Array(nW), W = new Float64Array(nW), I = new Int32Array(nW);
    let umin = Infinity, umax = -Infinity, vmin = Infinity, vmax = -Infinity;
    for (let s = 0; s < nW; s++) {
      const i = wall[Math.min(wall.length - 1, Math.floor(s * ws))];
      const c = [FC[3 * i], FC[3 * i + 1], FC[3 * i + 2]], n = [FN[3 * i], FN[3 * i + 1], FN[3 * i + 2]];
      const qu = c[0] * u[0] + c[1] * u[1] + c[2] * u[2], qv = c[0] * v[0] + c[1] * v[1] + c[2] * v[2];
      const nu = n[0] * u[0] + n[1] * u[1] + n[2] * u[2], nv = n[0] * v[0] + n[1] * v[1] + n[2] * v[2], ip = Math.hypot(nu, nv);
      Q[2 * s] = qu; Q[2 * s + 1] = qv; M[2 * s] = nu / ip; M[2 * s + 1] = nv / ip;
      Z[s] = c[0] * a[0] + c[1] * a[1] + c[2] * a[2]; W[s] = FA[i] * ws * ip; I[s] = idx[i];
      umin = Math.min(umin, qu); umax = Math.max(umax, qu); vmin = Math.min(vmin, qv); vmax = Math.max(vmax, qv);
    }
    umin -= rTop + cell; vmin -= rTop + cell; umax += rTop + cell; vmax += rTop + cell;
    const nx = Math.ceil((umax - umin) / cell) + 1, ny = Math.ceil((vmax - vmin) / cell) + 1;
    if (nx * ny * nr > 16e6) continue; // vehicle far too big for this cell size
    const acc = new Float32Array(nx * ny * nr), msk = new Uint8Array(nx * ny * nr);
    for (let s = 0; s < nW; s++) {
      const mu = M[2 * s], mv = M[2 * s + 1];
      const sec = Math.floor(((Math.atan2(mv, mu) + Math.PI) / (2 * Math.PI)) * 8) & 7, opp = (sec + 4) & 7;
      for (let ir = 0; ir < nr; ir++) {
        const r = r0 + ir * dr;
        for (const sg of [1, -1]) {
          const ix = Math.round((Q[2 * s] - sg * r * mu - umin) / cell), iy = Math.round((Q[2 * s + 1] - sg * r * mv - vmin) / cell);
          const id = (ir * ny + iy) * nx + ix;
          acc[id] += W[s];
          msk[id] |= 1 << (sg > 0 ? sec : opp);
        }
      }
    }
    // candidate centres: 3x3 sums with votes from (nearly) every direction
    const cand = [];
    for (let ir = 0; ir < nr; ir++) {
      const r = r0 + ir * dr, thr = 0.004 * 2 * Math.PI * r;
      for (let iy = 1; iy < ny - 1; iy++)
        for (let ix = 1; ix < nx - 1; ix++) {
          const id = (ir * ny + iy) * nx + ix;
          if (acc[id] < thr / 4) continue;
          let sum = 0, m = 0;
          for (let dy = -1; dy <= 1; dy++) for (let dx = -1; dx <= 1; dx++) { const j = id + dy * nx + dx; sum += acc[j]; m |= msk[j]; }
          if (sum >= thr && POP[m] >= 6) cand.push({ ix, iy, ir, r, w: sum / r });
        }
    }
    cand.sort((p, q) => q.w - p.w);
    const kept = [];
    for (const c of cand) {
      if (kept.length >= 64) break;
      if (kept.some((k) => Math.abs(k.ix - c.ix) <= 2 && Math.abs(k.iy - c.iy) <= 2 && Math.abs(k.ir - c.ir) <= 2)) continue;
      kept.push(c);
    }
    // verify: complete rings along the axis, exact centre and radii
    const rings = [];
    for (const c of kept) {
      const cu0 = umin + c.ix * cell, cv0 = vmin + c.iy * cell;
      const sup = [];
      for (let s = 0; s < nW; s++) {
        const du = Q[2 * s] - cu0, dv = Q[2 * s + 1] - cv0, d = Math.hypot(du, dv);
        if (d < 1e-6 || Math.abs(d - c.r) > Math.max(0.003, 0.06 * c.r) + cell) continue;
        if (Math.abs((M[2 * s] * du + M[2 * s + 1] * dv) / d) < 0.9) continue;
        sup.push(s);
      }
      if (sup.length < 12) continue;
      let Suu = 0, Suv = 0, Svv = 0, bu = 0, bv = 0;
      for (const s of sup) {
        const ku = -M[2 * s + 1], kv = M[2 * s], w = W[s], d = ku * Q[2 * s] + kv * Q[2 * s + 1];
        Suu += w * ku * ku; Suv += w * ku * kv; Svv += w * kv * kv; bu += w * ku * d; bv += w * kv * d;
      }
      const det = Suu * Svv - Suv * Suv;
      if (!(det > 1e-3 * (Suu + Svv) ** 2)) continue;
      const cu = (Svv * bu - Suv * bv) / det, cv = (Suu * bv - Suv * bu) / det;
      const faces = [];
      for (const s of sup) {
        const du = Q[2 * s] - cu, dv = Q[2 * s + 1] - cv, d = Math.hypot(du, dv);
        if (d < 1e-6 || Math.abs(d - c.r) > Math.max(0.002, 0.04 * c.r) + cell / 2) continue;
        const radial = (M[2 * s] * du + M[2 * s + 1] * dv) / d;
        if (Math.abs(radial) < 0.95) continue;
        const b = I[s] * 9;
        let r = 0, z0 = Infinity, z1 = -Infinity;
        for (let q = 0; q < 3; q++) {
          const x = positions[b + 3 * q], y = positions[b + 3 * q + 1], w = positions[b + 3 * q + 2];
          r += Math.hypot(x * u[0] + y * u[1] + w * u[2] - cu, x * v[0] + y * v[1] + w * v[2] - cv) / 3;
          const z = x * a[0] + y * a[1] + w * a[2];
          z0 = Math.min(z0, z); z1 = Math.max(z1, z);
        }
        faces.push({ r, A: W[s], facing: radial > 0 ? 1 : -1, ang: Math.atan2(dv, du), z0, z1, zc: Z[s] });
      }
      // separate rings that share a centre line but sit at different stations along it
      faces.sort((p, q) => p.zc - q.zc);
      let seg = [];
      const flush = () => {
        if (seg.length >= 12) {
          const g = ringStats(seg, 0);
          if (g.coverage >= 0.75 && g.A / (2 * Math.PI * g.r) >= 0.003) rings.push({ ...g, cu, cv });
        }
        seg = [];
      };
      for (const x of faces) {
        if (seg.length && x.zc - seg[seg.length - 1].zc > Math.max(0.01, 0.3 * c.r)) flush();
        seg.push(x);
      }
      flush();
    }
    // one ring per wall (neighbouring candidates find the same one)
    rings.sort((p, q) => q.A - p.A);
    const uniq = [];
    for (const g of rings) {
      const dup = uniq.find((k) => Math.hypot(k.cu - g.cu, k.cv - g.cv) <= Math.max(0.003, 0.05 * g.r) && Math.abs(k.r - g.r) <= Math.max(0.002, 0.04 * g.r)
        && Math.sign(k.rawFacing) === Math.sign(g.rawFacing) && zOverlap(k, g) > 0);
      if (!dup) uniq.push(g);
    }
    // outer + inner wall of opposite facing = duct
    uniq.sort((p, q) => q.r - p.r);
    const used = new Set();
    for (const O of uniq) {
      if (used.has(O)) continue;
      const I2 = uniq.find((g) => g !== O && !used.has(g) && Math.hypot(g.cu - O.cu, g.cv - O.cv) <= Math.max(0.004, 0.06 * O.r)
        && g.rMax < O.rMin && g.r >= LIMITS.wall[0] * O.r && g.r <= LIMITS.wall[1] * O.r
        && zOverlap(g, O) >= 0.3 * Math.min(g.z1 - g.z0, O.z1 - O.z0)
        && !(Math.abs(O.rawFacing) >= 0.6 && Math.abs(g.rawFacing) >= 0.6 && O.rawFacing * g.rawFacing > 0));
      if (!I2) continue;
      used.add(O); used.add(I2);
      const z0 = Math.min(O.z0, I2.z0), z1 = Math.max(O.z1, I2.z1), zm = (z0 + z1) / 2;
      ducts.push({
        axis: a.slice(), pos: [0, 1, 2].map((k) => u[k] * O.cu + v[k] * O.cv + a[k] * zm),
        od: 2 * O.rMax, id: 2 * I2.rMin, ductLength: z1 - z0, length: z1 - z0, coverage: Math.min(O.coverage, I2.coverage), fitScore: 0.8, hollow: true,
      });
    }
  }
  return ducts;
}

// ---------------------------------------------------------------- directions and motor order

const shortFrame = (fc) => FRAMES[fc].name.replace(/\s*\(.*\)\s*$/, '');

// ArduSub input that a thruster pushing along `axis` at `pos` drives:
// [roll, pitch, yaw, throttle, forward, lateral], moments over lever L.
function signature(pos, axis, ref, L) {
  const m = vcross(vsub(pos, ref), axis);
  return [m[0] / L, m[1] / L, m[2] / L, -axis[2], axis[0], axis[1]];
}
const cosine = (w, f) => {
  let d = 0, a = 0, b = 0;
  for (let k = 0; k < 6; k++) { d += w[k] * f[k]; a += w[k] * w[k]; b += f[k] * f[k]; }
  return a > 0 && b > 0 ? d / Math.sqrt(a * b) : 0;
};
// every clearly non-zero factor of the slot is driven, with the right sign
function signsAgree(w, row) {
  const rm = Math.max(...row.map(Math.abs)), wm = Math.max(...w.map(Math.abs)) || 1;
  return row.every((f, k) => Math.abs(f) < 0.2 * rm || (Math.sign(w[k]) === Math.sign(f) && Math.abs(w[k]) >= 0.12 * wm));
}

// minimum-cost assignment (Hungarian algorithm), cost n x n -> col of each row
export function hungarian(cost) {
  const n = cost.length, INF = 1e18;
  const u = new Float64Array(n + 1), v = new Float64Array(n + 1), p = new Int32Array(n + 1), way = new Int32Array(n + 1);
  for (let i = 1; i <= n; i++) {
    p[0] = i;
    let j0 = 0;
    const minv = new Float64Array(n + 1).fill(INF), used = new Uint8Array(n + 1);
    do {
      used[j0] = 1;
      const i0 = p[j0];
      let delta = INF, j1 = 0;
      for (let j = 1; j <= n; j++) {
        if (used[j]) continue;
        const cur = cost[i0 - 1][j - 1] - u[i0] - v[j];
        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
      }
      for (let j = 0; j <= n; j++) {
        if (used[j]) { u[p[j]] += delta; v[j] -= delta; } else minv[j] -= delta;
      }
      j0 = j1;
    } while (p[j0] !== 0);
    do { const j1 = way[j0]; p[j0] = p[j1]; j0 = j1; } while (j0);
  }
  const out = new Array(n);
  for (let j = 1; j <= n; j++) out[p[j] - 1] = j - 1;
  return out;
}

// frames with n motors, the configured one first (identical tables once)
function frameCandidates(n, current) {
  const keys = Object.keys(FRAMES).map(Number).filter((k) => k !== 7 && FRAMES[k].table.length === n);
  if (FRAMES[current] && FRAMES[current].table.length === n) keys.sort((a, b) => (b === current) - (a === current) || a - b);
  const seen = new Set();
  return keys.filter((k) => { const s = JSON.stringify(FRAMES[k].table); if (seen.has(s)) return false; seen.add(s); return true; });
}

// default when no frame fits: vertical pushes down (+Z, like the ArduSub
// vectored frames), horizontal away from the centre (tangential: forward / right)
function defaultSign(t, ref) {
  const a = t.axis;
  if (Math.abs(a[2]) >= 0.7) return a[2] >= 0 ? 1 : -1;
  const rx = t.pos[0] - ref[0], ry = t.pos[1] - ref[1], rl = Math.hypot(rx, ry), ah = Math.hypot(a[0], a[1]);
  const radial = rl > 1e-6 && ah > 1e-6 ? (a[0] * rx + a[1] * ry) / (rl * ah) : 0;
  if (Math.abs(radial) >= 0.2) return radial > 0 ? 1 : -1;
  return (Math.abs(a[0]) >= Math.abs(a[1]) ? a[0] : a[1]) >= 0 ? 1 : -1;
}

// horizontal first, then vertical; front-right, front-left, rear-right, rear-left
function defaultOrder(list, ref) {
  const key = (t) => (Math.abs(t.axis[2]) >= 0.7 ? 4 : 0) + (t.pos[0] >= ref[0] ? 0 : 2) + (t.pos[1] >= ref[1] ? 0 : 1);
  return list.map((t, i) => i).sort((i, j) => key(list[i]) - key(list[j]) || list[j].pos[0] - list[i].pos[0] || list[j].pos[1] - list[i].pos[1]);
}

/**
 * Motor order and push direction of detected thrusters.
 * @param list [{pos, axis (sign free), number?}]
 * @param opts { frameConfig (FRAME_CONFIG to match first), cg (reference for torques) }
 * @returns { thrusters (ordered, axis signed, motor, directionConfidence), frame, directions, notes }
 */
export function resolveDirections(list, { frameConfig = 2, cg = null } = {}) {
  const n = list.length, notes = [];
  if (!n) return { thrusters: [], frame: null, directions: 'none', notes };
  const ref = cg || [0, 1, 2].map((k) => list.reduce((s, t) => s + t.pos[k], 0) / n);
  const L = Math.sqrt(list.reduce((s, t) => s + vsub(t.pos, ref).reduce((q, x) => q + x * x, 0), 0) / n) || 0.1;
  const Wt = list.map((t) => signature(t.pos, vnorm(t.axis), ref, L));
  const nums = list.map((t) => t.number);
  const byName = nums.every((k) => k >= 1 && k <= n) && new Set(nums).size === n ? nums.map((k) => k - 1) : null;
  let best = null;
  for (const fc of frameCandidates(n, frameConfig)) {
    const table = FRAMES[fc].table;
    const S = Wt.map((w) => table.map((row) => cosine(w, row)));
    const tries = byName ? [{ slots: byName, names: true }] : [];
    tries.push({ slots: hungarian(S.map((r) => r.map((s) => 1 - Math.abs(s)))), names: false });
    let hit = null;
    for (const tr of tries) {
      let sum = 0, ok = true;
      const signs = tr.slots.map((j, i) => {
        const s = S[i][j], sg = s >= 0 ? 1 : -1;
        sum += Math.abs(s);
        if (Math.abs(s) < 0.55 || !signsAgree(Wt[i].map((x) => x * sg), table[j])) ok = false;
        return sg;
      });
      if (ok) { hit = { fc, slots: tr.slots, signs, conf: tr.slots.map((j, i) => Math.abs(S[i][j])), score: sum / n, names: tr.names }; break; }
    }
    if (!hit) continue;
    if (fc === frameConfig) { best = hit; break; }
    if (!best || hit.score > best.score) best = hit;
  }
  if (best) {
    const out = new Array(n);
    best.slots.forEach((j, i) => { out[j] = { ...list[i], axis: vnorm(list[i].axis).map((x) => x * best.signs[i] || 0), motor: j + 1, directionConfidence: best.conf[i] }; });
    if (byName && !best.names) notes.push(`The motor numbers in the part names do not fit FRAME_CONFIG ${best.fc}: thrusters renumbered by position.`);
    if (best.fc !== frameConfig) notes.push(`Your FRAME_CONFIG is ${frameConfig}, the layout matches FRAME_CONFIG ${best.fc} (${shortFrame(best.fc)}).`);
    return { thrusters: out, frame: { config: best.fc, name: shortFrame(best.fc), score: best.score, current: best.fc === frameConfig }, directions: 'frame', notes };
  }
  const order = byName ? list.map((_, i) => i).sort((i, j) => nums[i] - nums[j]) : defaultOrder(list, ref);
  const thrusters = order.map((i, k) => ({ ...list[i], axis: vnorm(list[i].axis).map((x) => x * defaultSign(list[i], ref) || 0), motor: k + 1, directionConfidence: 0.3 }));
  notes.push(`No ArduSub frame with ${n} motors matches this layout: directions follow a default rule (vertical push down, horizontal away from the centre). Check each one.`);
  return { thrusters, frame: null, directions: 'default', notes };
}

// ---------------------------------------------------------------- main entry

// "front-right", "vert left", "rear" (within 3 cm of the centre: no word)
const where = (p, axis, ref) => {
  const fb = Math.abs(p[0] - ref[0]) < 0.03 ? '' : p[0] > ref[0] ? 'front' : 'rear';
  const lr = Math.abs(p[1] - ref[1]) < 0.03 ? '' : p[1] > ref[1] ? 'right' : 'left';
  return `${Math.abs(axis[2]) >= 0.7 ? 'vert ' : ''}${[fb, lr].filter(Boolean).join('-') || 'centre'}`;
};

/**
 * Find the thrusters of a design.
 * @param positions triangle soup in the vehicle frame (FRD, m)
 * @param opts { parts: [{name, start, count}], frameConfig, cg, maxThrusters }
 * @returns { method: 'names'|'shape'|'hull'|'names+shape'|'none', thrusters, frame, directions, summary, notes }
 *   thruster: { motor, name, pos, axis, type, preset, propDiameter, od, id, length, hollow, confidence, directionConfidence, source, partName, number }
 */
export function detectThrusters(positions, opts = {}) {
  const nT = Math.floor(positions.length / 9);
  const notes = [];
  const none = (why) => ({ method: 'none', thrusters: [], frame: null, directions: 'none', summary: why, notes });
  if (!nT) return none('Empty mesh');
  const model = bboxOf(positions, null);
  const modelExtent = Math.max(...model.size);
  const comps = meshComponents(positions);
  const used = new Uint8Array(nT);
  const found = [];

  // 1. part names
  const partOf = new Int32Array(nT).fill(-1);
  const named = [];
  for (const p of opts.parts || []) {
    const info = thrusterNameInfo(p.name);
    const start = Math.max(0, p.start | 0), end = Math.min(nT, start + Math.max(0, p.count | 0));
    if (!info.thruster || end <= start) continue;
    const k = named.push({ ...p, info }) - 1;
    for (let t = start; t < end; t++) partOf[t] = k;
  }
  if (named.length) {
    const keep = new Uint8Array(nT);
    for (let t = 0; t < nT; t++) keep[t] = partOf[t] >= 0 ? 1 : 0;
    const { clusters, big } = groupParts(positions, comps, keep, Infinity);
    if (big.length) notes.push(`${big.length} part(s) named like a thruster are bigger than ${LIMITS.maxGroup * 100} cm and were ignored.`);
    for (const c of clusters) {
      const ext = Math.max(...c.bb.size);
      if (ext < 0.01) continue; // a screw or a label, not a thruster
      // the named part with most triangles in the group gives the number / model
      const votes = new Map();
      for (const t of c.tris) votes.set(partOf[t], (votes.get(partOf[t]) || 0) + 1);
      const ranked = [...votes.entries()].sort((a, b) => b[1] - a[1]).map(([k]) => named[k]);
      const number = ranked.find((p) => p.info.number)?.info.number ?? null;
      const nameType = ranked.find((p) => p.info.type)?.info.type ?? null;
      const m = measureGroup(positions, c.tris, comps.vid);
      const cls = m && m.od ? classifyDuct(m) : { ok: false };
      const type = nameType || (cls.ok ? cls.type : 'generic');
      const conf = m && m.od ? 0.8 + 0.15 * m.fitScore : 0.7;
      found.push({
        pos: m && m.od ? m.pos : c.bb.center, axis: m ? m.axis : [1, 0, 0], od: m?.od ?? null, id: m?.id ?? null, length: m?.length ?? ext,
        hollow: !!m?.hollow, type, confidence: Math.min(0.97, conf), source: 'name', partName: ranked[0].name, number,
      });
      for (const t of c.tris) used[t] = 1;
    }
    const nums = found.map((t) => t.number).filter(Boolean);
    if (new Set(nums).size !== nums.length) { notes.push('Some part names carry the same motor number: numbers ignored.'); found.forEach((t) => (t.number = null)); }
  }

  // 2. shape of separate parts
  const keep = new Uint8Array(nT);
  for (let t = 0; t < nT; t++) keep[t] = used[t] ? 0 : 1;
  const shapeStart = found.length;
  const { clusters } = groupParts(positions, comps, keep, modelExtent);
  const rejected = {};
  for (const c of clusters) {
    const e = c.bb.size.slice().sort((p, q) => q - p);
    if (e[1] < 0.02 || e[0] > LIMITS.maxGroup) continue; // too small to hold a 30 mm duct, or hull-sized
    const m = measureGroup(positions, c.tris, comps.vid);
    if (!m || !m.od) continue;
    const cls = classifyDuct(m);
    if (!cls.ok) { rejected[cls.reason] = (rejected[cls.reason] || 0) + 1; continue; }
    if (m.fitScore < 0.6 || m.coverage < LIMITS.coverage) continue;
    if (found.some((t) => Math.hypot(...vsub(t.pos, m.pos)) < 0.5 * m.od)) continue;
    const conf = 0.3 + 0.3 * m.fitScore + 0.15 * m.coverage + 0.2 * (cls.type === 'generic' ? 0.3 : cls.match);
    found.push({ ...m, type: cls.type, confidence: Math.min(0.95, conf), source: 'shape', partName: null, number: null });
    for (const t of c.tris) used[t] = 1;
  }

  // 3. ducts fused into the hull, when no thruster is a separate part
  let hullRings = false;
  if (!found.length) {
    for (const d of fusedDucts(positions, keep, model.size)) {
      const cls = classifyDuct(d);
      if (!cls.ok) continue;
      if (found.some((t) => Math.hypot(...vsub(t.pos, d.pos)) < 0.5 * Math.max(d.od, t.od || 0))) continue;
      found.push({ ...d, type: cls.type, confidence: Math.min(0.8, 0.25 + 0.3 * d.coverage + 0.25 * (cls.type === 'generic' ? 0.3 : cls.match)), source: 'hull', partName: null, number: null });
      hullRings = true;
    }
  }
  if (!found.length) {
    const why = Object.entries(rejected).map(([k, v]) => `${v} × ${k}`).join(', ');
    return { ...none('No thrusters found in the design'), notes: why ? [`Round parts rejected: ${why}.`] : notes };
  }
  if (found.length > (opts.maxThrusters ?? MAX_MOTORS)) {
    found.sort((p, q) => q.confidence - p.confidence);
    notes.push(`${found.length} thruster-like parts found; ArduSub drives at most ${MAX_MOTORS}: the ${MAX_MOTORS} most certain are kept.`);
    found.length = opts.maxThrusters ?? MAX_MOTORS;
  }

  // 4. motor order and directions from the ArduSub frame tables
  const dir = resolveDirections(found, { frameConfig: opts.frameConfig ?? 2, cg: opts.cg || null });
  const ref = opts.cg || [0, 1, 2].map((k) => found.reduce((s, t) => s + t.pos[k], 0) / found.length);
  const thrusters = dir.thrusters.map((t) => {
    const T = THRUSTER_TYPES[t.type] || THRUSTER_TYPES.generic;
    const prop = T.prop ?? (t.id ? Math.max(0.02, t.id - 0.004) : t.od ? 0.75 * t.od : 0.06);
    const label = t.partName ? thrusterLabel(t.partName) : `T${t.motor} ${where(t.pos, t.axis, ref)}`;
    return {
      motor: t.motor, name: label, pos: t.pos.map((x) => +x.toFixed(4) || 0), axis: t.axis.map((x) => +x.toFixed(5) || 0),
      type: t.type, preset: T.preset, propDiameter: +prop.toFixed(4),
      od: t.od, id: t.id, length: t.length, hollow: t.hollow,
      confidence: +t.confidence.toFixed(2), directionConfidence: +t.directionConfidence.toFixed(2),
      source: t.source, partName: t.partName, number: t.number,
    };
  });
  const nName = thrusters.filter((t) => t.source === 'name').length;
  const method = hullRings ? 'hull' : nName === thrusters.length ? 'names' : nName ? 'names+shape' : 'shape';
  // "8 × T200 detected (part names) — matched FRAME_CONFIG 2 Vectored 6DOF"
  const counts = {};
  thrusters.forEach((t) => (counts[t.type] = (counts[t.type] || 0) + 1));
  const kinds = Object.keys(counts);
  const what = kinds.length === 1 && kinds[0] !== 'generic' ? `${thrusters.length} × ${THRUSTER_TYPES[kinds[0]].label}` : `${thrusters.length} thruster${thrusters.length > 1 ? 's' : ''}`;
  const how = { names: '(part names)', 'names+shape': `(${nName} by part name, ${thrusters.length - nName} by shape)`, shape: 'by shape', hull: 'by duct shape in the hull' }[method];
  const tail = dir.frame ? ` — matched FRAME_CONFIG ${dir.frame.config} ${dir.frame.name}` : ' — check directions';
  if (kinds.length > 1) notes.unshift(`Models: ${kinds.map((k) => `${counts[k]} × ${THRUSTER_TYPES[k].label}`).join(', ')}.`);
  if (shapeStart < found.length && found.slice(shapeStart).some((t) => t.confidence < 0.6)) notes.push('Some thrusters were found with low confidence: check them.');
  return { method, thrusters, frame: dir.frame, directions: dir.directions, summary: `${what} detected ${how}${tail}`, notes: [...notes, ...dir.notes] };
}

// words for a push direction ("down", "back-right", "forward 30° up"...)
export function directionWord(axis) {
  const a = vnorm(axis);
  if (a[2] >= 0.9) return 'down';
  if (a[2] <= -0.9) return 'up';
  const h = Math.hypot(a[0], a[1]);
  const fx = a[0] / h, fy = a[1] / h;
  const ns = fx > 0.38 ? 'forward' : fx < -0.38 ? 'back' : '';
  const ew = fy > 0.38 ? 'right' : fy < -0.38 ? 'left' : '';
  const base = ns && ew ? `${ns}-${ew}` : ns || ew;
  const tilt = Math.round((Math.asin(Math.min(1, Math.abs(a[2]))) * 180) / Math.PI);
  return tilt >= 10 ? `${base} ${tilt}° ${a[2] > 0 ? 'down' : 'up'}` : base;
}
