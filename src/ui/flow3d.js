// 3D flow view from a CFD-lab snapshot: streamlines around the hull and the
// hull coloured by pressure coefficient, drawn in the vehicle frame so they
// follow the vehicle in the pool scene.

import * as THREE from 'three';
import { toThree, fromThree } from './scene3d.js';

const lerp3 = (a, b, t) => [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t];
// sequential (magnitude): dark violet -> magenta -> orange -> pale yellow, readable on light and dark water
const SEQ = [[0.13, 0.07, 0.36], [0.55, 0.12, 0.55], [0.93, 0.38, 0.22], [0.99, 0.82, 0.38]];
export function speedColor(t) {
  t = Math.min(1, Math.max(0, t)) * (SEQ.length - 1);
  const i = Math.min(SEQ.length - 2, Math.floor(t));
  return lerp3(SEQ[i], SEQ[i + 1], t - i);
}
// diverging (pressure): blue (suction) - grey - orange (stagnation)
export function cpColor(cp) {
  const t = Math.min(1, Math.max(-1, cp));
  const mid = [0.78, 0.8, 0.82], neg = [0.16, 0.42, 0.85], pos = [0.88, 0.36, 0.12];
  return t < 0 ? lerp3(mid, neg, -t) : lerp3(mid, pos, t);
}

/**
 * flow: { field: {nx, ny, nz, stride, data}, cs: {R, origin, cell, kind, axis, sign, N}, V, cg }
 * Coordinates: field index f -> lattice index i = f * stride -> lattice metres
 * P = origin + (i + 0.5) cell -> vehicle frame x = cg + R^T P.
 */
export class FlowView {
  constructor(flow) {
    this.flow = flow;
    const { field } = flow;
    this.F = field;
    this.n = [field.nx, field.ny, field.nz];
    // solid bounding box in field indices
    const lo = [Infinity, Infinity, Infinity], hi = [-Infinity, -Infinity, -Infinity];
    for (let k = 0; k < field.nz; k++) for (let j = 0; j < field.ny; j++) for (let i = 0; i < field.nx; i++) {
      if (!Number.isNaN(field.data[4 * (i + field.nx * (j + field.ny * k)) + 3])) continue;
      lo[0] = Math.min(lo[0], i); lo[1] = Math.min(lo[1], j); lo[2] = Math.min(lo[2], k);
      hi[0] = Math.max(hi[0], i); hi[1] = Math.max(hi[1], j); hi[2] = Math.max(hi[2], k);
    }
    this.body = { lo, hi };
  }
  at(i, j, k) { const n = this.n; return 4 * (i + n[0] * (j + n[1] * k)); }
  // trilinear velocity (field-index units per step); null inside solids / outside
  vel(x, y, z) {
    const n = this.n, d = this.F.data;
    if (x < 0 || y < 0 || z < 0 || x > n[0] - 1.001 || y > n[1] - 1.001 || z > n[2] - 1.001) return null;
    const i = Math.floor(x), j = Math.floor(y), k = Math.floor(z);
    const fx = x - i, fy = y - j, fz = z - k;
    const out = [0, 0, 0];
    let wsum = 0;
    for (let c = 0; c < 8; c++) {
      const di = c & 1, dj = (c >> 1) & 1, dk = (c >> 2) & 1;
      const o = this.at(i + di, j + dj, k + dk);
      if (Number.isNaN(d[o + 3])) continue;
      const w = (di ? fx : 1 - fx) * (dj ? fy : 1 - fy) * (dk ? fz : 1 - fz);
      out[0] += w * d[o]; out[1] += w * d[o + 1]; out[2] += w * d[o + 2]; wsum += w;
    }
    if (wsum < 0.5) return null;
    const s = this.F.stride;
    return [out[0] / wsum / s, out[1] / wsum / s, out[2] / wsum / s];
  }
  toVehicle(x, y, z) {
    const { cs, cg } = this.flow;
    const s = this.F.stride;
    const P = [cs.origin[0] + (x * s + 0.5) * cs.cell, cs.origin[1] + (y * s + 0.5) * cs.cell, cs.origin[2] + (z * s + 0.5) * cs.cell];
    const R = cs.R;
    return [0, 1, 2].map((a) => cg[a] + R[0][a] * P[0] + R[1][a] * P[1] + R[2][a] * P[2]);
  }
  toField(v) {
    const { cs, cg } = this.flow;
    const d = [v[0] - cg[0], v[1] - cg[1], v[2] - cg[2]];
    const R = cs.R, s = this.F.stride;
    const P = [0, 1, 2].map((a) => R[a][0] * d[0] + R[a][1] * d[1] + R[a][2] * d[2]);
    return [0, 1, 2].map((a) => ((P[a] - cs.origin[a]) / cs.cell - 0.5) / s);
  }

  trace(seed, dir = 1, maxSteps = 700) {
    const pts = [seed.slice()], spd = [];
    let p = seed.slice();
    const V = this.flow.V / this.F.stride;
    for (let n = 0; n < maxSteps; n++) {
      const u1 = this.vel(...p);
      if (!u1) break;
      const m1 = Math.hypot(...u1);
      if (m1 < 0.02 * V) break;
      const h = (0.45 / m1) * dir;
      const mid = [p[0] + 0.5 * h * u1[0], p[1] + 0.5 * h * u1[1], p[2] + 0.5 * h * u1[2]];
      const u2 = this.vel(...mid);
      if (!u2) break;
      p = [p[0] + h * u2[0], p[1] + h * u2[1], p[2] + h * u2[2]];
      pts.push(p);
      spd.push(Math.hypot(...u2) / V);
    }
    spd.push(spd[spd.length - 1] ?? 0);
    return { pts, spd };
  }

  seeds() {
    const { lo, hi } = this.body;
    const n = this.n;
    const out = [];
    if (this.flow.cs.kind === 't') {
      // a rake across the flow, a little upstream of the hull
      const x0 = Math.max(1, lo[0] - Math.max(2, Math.round(0.3 * (hi[0] - lo[0]))));
      const ey = (hi[1] - lo[1]) * 0.3 + 1, ez = (hi[2] - lo[2]) * 0.3 + 1;
      const ny = 16, nz = 11;
      for (let b = 0; b < nz; b++) for (let a = 0; a < ny; a++) {
        const y = lo[1] - ey + ((hi[1] - lo[1] + 2 * ey) * (a + 0.5)) / ny;
        const z = lo[2] - ez + ((hi[2] - lo[2] + 2 * ez) * (b + 0.5)) / nz;
        if (y > 0 && z > 0 && y < n[1] - 1 && z < n[2] - 1) out.push([x0, y, z]);
      }
    } else {
      // points in a shell around the spinning hull
      let s = 7;
      const rnd = () => { s = (s * 16807) % 2147483647; return s / 2147483647; };
      const pad = [0, 1, 2].map((a) => (hi[a] - lo[a]) * 0.25 + 1);
      for (let tries = 0; out.length < 140 && tries < 5000; tries++) {
        const p = [0, 1, 2].map((a) => lo[a] - pad[a] + rnd() * (hi[a] - lo[a] + 2 * pad[a]));
        const v = this.vel(...p);
        if (v && Math.hypot(...v) > 0.05 * this.flow.V / this.F.stride) out.push(p);
      }
    }
    return out;
  }

  streamlines() {
    const pos = [], col = [];
    for (const seed of this.seeds()) {
      const lines = this.flow.cs.kind === 't' ? [this.trace(seed, 1)] : [this.trace(seed, 1, 260), this.trace(seed, -1, 260)];
      for (const { pts, spd } of lines) {
        for (let i = 0; i + 1 < pts.length; i++) {
          const a = toThree(this.toVehicle(...pts[i])), b = toThree(this.toVehicle(...pts[i + 1]));
          pos.push(a.x, a.y, a.z, b.x, b.y, b.z);
          const ca = speedColor(spd[i] / 1.4), cb = speedColor(spd[i + 1] / 1.4);
          col.push(...ca, ...cb);
        }
      }
    }
    const g = new THREE.BufferGeometry();
    g.setAttribute('position', new THREE.Float32BufferAttribute(pos, 3));
    g.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
    const lines = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.95 }));
    lines.renderOrder = 5;
    return lines;
  }

  // colours for a hull mesh (three.js positions in the vehicle frame)
  hullColors(posAttr) {
    const col = new Float32Array(posAttr.count * 3);
    const cache = new Map();
    for (let v = 0; v < posAttr.count; v++) {
      const key = `${posAttr.getX(v).toFixed(4)},${posAttr.getY(v).toFixed(4)},${posAttr.getZ(v).toFixed(4)}`;
      let c = cache.get(key);
      if (!c) {
        const cp = this.cpAt(fromThree({ x: posAttr.getX(v), y: posAttr.getY(v), z: posAttr.getZ(v) }));
        c = Number.isNaN(cp) ? [0.55, 0.58, 0.6] : cpColor(cp);
        cache.set(key, c);
      }
      col[3 * v] = c[0]; col[3 * v + 1] = c[1]; col[3 * v + 2] = c[2];
    }
    return col;
  }

  // pressure coefficient at a vehicle-frame point: nearest fluid sample
  cpAt(v) {
    const f = this.toField(v);
    const n = this.n, d = this.F.data;
    let best = Infinity, rho = NaN;
    const ci = Math.round(f[0]), cj = Math.round(f[1]), ck = Math.round(f[2]);
    for (let r = 1; r <= 3 && Number.isNaN(rho); r++)
      for (let k = ck - r; k <= ck + r; k++) for (let j = cj - r; j <= cj + r; j++) for (let i = ci - r; i <= ci + r; i++) {
        if (i < 0 || j < 0 || k < 0 || i >= n[0] || j >= n[1] || k >= n[2]) continue;
        const o = this.at(i, j, k);
        if (Number.isNaN(d[o + 3])) continue;
        const dd = (i - f[0]) ** 2 + (j - f[1]) ** 2 + (k - f[2]) ** 2;
        if (dd < best) { best = dd; rho = d[o + 3]; }
      }
    if (Number.isNaN(rho)) return NaN;
    const V = this.flow.cs.kind === 't' ? this.flow.V : this.flow.V * 0.5;
    return ((rho - 1) / 3) / (0.5 * V * V);
  }
}

export { fromThree };
