// Potential-flow boundary element method (constant-strength source panels,
// Hess & Smith type) for the 6x6 added-mass matrix of a deeply submerged body
// in unbounded fluid (infinite-frequency limit, no free surface).
//
//   phi(x) = sum_k sigma_k * int_Sk G(x,xi) dS,   G = -1/(4 pi |x - xi|)
//   d(phi_j)/dn = N_j on S   (N = [n, r x n], n outward from the body)
//   => sigma_i/2 + sum_k K_ik sigma_k = N_j(i)
//   m_lj = -rho * int_S phi_l N_j dS
//
// Panels are planar polygons {c (centroid), n (unit normal, outward), area,
// verts (CCW about n)}. Near-field integrals use sub-triangle quadrature, the
// self potential uses the exact polygon formula.

import { kernels } from '../wasm/kernels.js';

const FOUR_PI = 4 * Math.PI;

export function rectPanelToPolygon(p) {
  const a = p.axis, b = (a + 1) % 3, c = (a + 2) % 3;
  const corner = (sb, sc) => { const v = p.c.slice(); v[b] += sb * p.hb; v[c] += sc * p.hc; return v; };
  let verts = [corner(-1, -1), corner(1, -1), corner(1, 1), corner(-1, 1)];
  if (p.n[a] < 0) verts = [verts[0], verts[3], verts[2], verts[1]];
  return { c: p.c, n: p.n, area: p.area, verts };
}

// integral over the polygon of 1/|P - xi| dA for P in the polygon plane (inside)
function polygonInvR(P, verts) {
  let s = 0;
  for (let e = 0; e < verts.length; e++) {
    const A = verts[e], B = verts[(e + 1) % verts.length];
    const t = [B[0] - A[0], B[1] - A[1], B[2] - A[2]];
    const L = Math.hypot(t[0], t[1], t[2]);
    if (L < 1e-15) continue;
    t[0] /= L; t[1] /= L; t[2] /= L;
    const AP = [A[0] - P[0], A[1] - P[1], A[2] - P[2]];
    const s1 = AP[0] * t[0] + AP[1] * t[1] + AP[2] * t[2];
    const s2 = s1 + L;
    const perp = [AP[0] - s1 * t[0], AP[1] - s1 * t[1], AP[2] - s1 * t[2]];
    const d = Math.hypot(perp[0], perp[1], perp[2]);
    if (d < 1e-15) continue;
    s += d * (Math.asinh(s2 / d) - Math.asinh(s1 / d));
  }
  return s;
}

function quadrature(panel, m) {
  // fan-triangulate from centroid, split each triangle into m^2 sub-triangles
  const pts = [], w = [];
  const v = panel.verts, c = panel.c;
  for (let e = 0; e < v.length; e++) {
    const A = c, B = v[e], C = v[(e + 1) % v.length];
    const ab = [B[0] - A[0], B[1] - A[1], B[2] - A[2]], ac = [C[0] - A[0], C[1] - A[1], C[2] - A[2]];
    const cr = [ab[1] * ac[2] - ab[2] * ac[1], ab[2] * ac[0] - ab[0] * ac[2], ab[0] * ac[1] - ab[1] * ac[0]];
    const area = 0.5 * Math.hypot(cr[0], cr[1], cr[2]);
    if (area < 1e-18) continue;
    const sub = area / (m * m);
    for (let i = 0; i < m; i++)
      for (let j = 0; j < m - i; j++) {
        // upward triangle (i,j),(i+1,j),(i,j+1) centroid
        const u = (i + 1 / 3) / m, vv = (j + 1 / 3) / m;
        pts.push([A[0] + u * ab[0] + vv * ac[0], A[1] + u * ab[1] + vv * ac[1], A[2] + u * ab[2] + vv * ac[2]]);
        w.push(sub);
        if (i + j < m - 1) {
          const u2 = (i + 2 / 3) / m, v2 = (j + 2 / 3) / m;
          pts.push([A[0] + u2 * ab[0] + v2 * ac[0], A[1] + u2 * ab[1] + v2 * ac[1], A[2] + u2 * ab[2] + v2 * ac[2]]);
          w.push(sub);
        }
      }
  }
  return { pts, w };
}

/**
 * @param panels  polygons {c, n, area, verts}
 * @param opts    { rho, ref:[x,y,z] (moment reference), tol, maxIter, onProgress }
 */
export function solveAddedMass(panelsIn, opts = {}) {
  const panels = panelsIn.map((p) => (p.verts ? p : rectPanelToPolygon(p)));
  const rho = opts.rho ?? 1000;
  const ref = opts.ref ?? [0, 0, 0];
  const N = panels.length;
  const onProgress = opts.onProgress || (() => {});
  const KW = opts.wasm === false ? null : kernels();
  if (KW && N <= 7000 && panels.every((p) => p.verts.length === 4 || p.verts.length === 3)) {
    const r = solveWasm(KW, panels, rho, ref, opts, onProgress);
    if (r) return r;
  }
  const K = N <= 2800 ? new Float64Array(N * N) : new Float32Array(N * N);
  const cx = new Float64Array(N), cy = new Float64Array(N), cz = new Float64Array(N);
  const nx = new Float64Array(N), ny = new Float64Array(N), nz = new Float64Array(N);
  const size = new Float64Array(N), area = new Float64Array(N);
  panels.forEach((p, i) => {
    cx[i] = p.c[0]; cy[i] = p.c[1]; cz[i] = p.c[2];
    nx[i] = p.n[0]; ny[i] = p.n[1]; nz[i] = p.n[2];
    let r = 0;
    for (const v of p.verts) r = Math.max(r, Math.hypot(v[0] - p.c[0], v[1] - p.c[1], v[2] - p.c[2]));
    size[i] = 2 * r;
    area[i] = p.area;
  });
  const qcache = new Map();
  const quad = (k, m) => {
    const key = k * 8 + m;
    let q = qcache.get(key);
    if (!q) {
      q = quadrature(panels[k], m);
      if (qcache.size < 120000) qcache.set(key, q);
    }
    return q;
  };
  const level = (d, sz) => (d > 4 * sz ? 0 : d > 2 * sz ? 2 : d > sz ? 4 : 6);

  for (let i = 0; i < N; i++) {
    const xi = cx[i], yi = cy[i], zi = cz[i], n0 = nx[i], n1 = ny[i], n2 = nz[i];
    const row = i * N;
    for (let k = 0; k < N; k++) {
      if (k === i) { K[row + k] = 0; continue; }
      const dx = xi - cx[k], dy = yi - cy[k], dz = zi - cz[k];
      const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const m = level(d, size[k]);
      if (m === 0) {
        K[row + k] = (area[k] * (n0 * dx + n1 * dy + n2 * dz)) / (FOUR_PI * d * d * d);
      } else {
        const { pts, w } = quad(k, m);
        let s = 0;
        for (let q = 0; q < pts.length; q++) {
          const p = pts[q];
          const ex = xi - p[0], ey = yi - p[1], ez = zi - p[2];
          const r2 = ex * ex + ey * ey + ez * ez;
          s += (w[q] * (n0 * ex + n1 * ey + n2 * ez)) / (r2 * Math.sqrt(r2));
        }
        K[row + k] = s / FOUR_PI;
      }
    }
    if ((i & 63) === 0) onProgress(0.55 * (i / N), 'Building influence matrix');
  }

  const rhs = [];
  for (let j = 0; j < 6; j++) rhs.push(new Float64Array(N));
  for (let i = 0; i < N; i++) {
    const rx = cx[i] - ref[0], ry = cy[i] - ref[1], rz = cz[i] - ref[2];
    rhs[0][i] = nx[i]; rhs[1][i] = ny[i]; rhs[2][i] = nz[i];
    rhs[3][i] = ry * nz[i] - rz * ny[i];
    rhs[4][i] = rz * nx[i] - rx * nz[i];
    rhs[5][i] = rx * ny[i] - ry * nx[i];
  }
  const matvec = (x, y) => {
    for (let i = 0; i < N; i++) {
      let s = 0.5 * x[i];
      const row = i * N;
      for (let k = 0; k < N; k++) s += K[row + k] * x[k];
      y[i] = s;
    }
  };
  const sigmas = [], iterations = [];
  for (let j = 0; j < 6; j++) {
    const { x, iters, resid } = gmres(matvec, rhs[j], N, { tol: opts.tol ?? 1e-8, maxIter: opts.maxIter ?? 400, restart: 60 });
    sigmas.push(x);
    iterations.push({ iters, resid });
    onProgress(0.55 + 0.3 * ((j + 1) / 6), 'Solving source strengths');
  }

  const phis = sigmas.map(() => new Float64Array(N));
  const acc = new Float64Array(6);
  for (let i = 0; i < N; i++) {
    const P = [cx[i], cy[i], cz[i]];
    acc.fill(0);
    for (let k = 0; k < N; k++) {
      let g;
      if (k === i) g = -polygonInvR(P, panels[k].verts) / FOUR_PI;
      else {
        const dx = P[0] - cx[k], dy = P[1] - cy[k], dz = P[2] - cz[k];
        const d = Math.sqrt(dx * dx + dy * dy + dz * dz);
        const m = level(d, size[k]);
        if (m === 0) g = -area[k] / (FOUR_PI * d);
        else {
          const { pts, w } = quad(k, m);
          let s = 0;
          for (let q = 0; q < pts.length; q++) s += w[q] / Math.hypot(P[0] - pts[q][0], P[1] - pts[q][1], P[2] - pts[q][2]);
          g = -s / FOUR_PI;
        }
      }
      for (let j = 0; j < 6; j++) acc[j] += g * sigmas[j][k];
    }
    for (let j = 0; j < 6; j++) phis[j][i] = acc[j];
    if ((i & 63) === 0) onProgress(0.85 + 0.15 * (i / N), 'Integrating pressures');
  }
  const M = Array.from({ length: 6 }, () => new Array(6).fill(0));
  for (let l = 0; l < 6; l++)
    for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let i = 0; i < N; i++) s += phis[l][i] * rhs[j][i] * area[i];
      M[l][j] = -rho * s;
    }
  let asym = 0, mag = 0;
  for (let l = 0; l < 6; l++)
    for (let j = 0; j < 6; j++) {
      asym = Math.max(asym, Math.abs(M[l][j] - M[j][l]));
      mag = Math.max(mag, Math.abs(M[l][j]));
    }
  const Ms = M.map((row, l) => row.map((v, j) => 0.5 * (v + M[j][l])));
  onProgress(1, 'Done');
  return { M: Ms, raw: M, panels: N, iterations, asymmetry: mag ? asym / mag : 0, ref, engine: 'js' };
}

// Same algorithm with the heavy O(N^2) parts (influence matrix, mat-vec,
// potential integration) in the WebAssembly kernels.
function solveWasm(KW, panels, rho, ref, opts, onProgress) {
  const N = panels.length;
  if (!KW.bem_init(N)) return null;
  const f64 = (ptr, len) => new Float64Array(KW.memory.buffer, ptr, len);
  const C = f64(KW.bem_centroids(), 3 * N), Nn = f64(KW.bem_normals(), 3 * N), A = f64(KW.bem_areas(), N), V = f64(KW.bem_verts(), 12 * N);
  panels.forEach((p, i) => {
    C.set(p.c, 3 * i); Nn.set(p.n, 3 * i); A[i] = p.area;
    for (let e = 0; e < 4; e++) V.set(p.verts[Math.min(e, p.verts.length - 1)], 12 * i + 3 * e);
  });
  onProgress(0.05, 'Building influence matrix');
  KW.bem_assemble();
  onProgress(0.55, 'Solving source strengths');
  const xPtr = KW.alloc(N * 8), yPtr = KW.alloc(N * 8);
  const sPtr = KW.alloc(6 * N * 8), pPtr = KW.alloc(6 * N * 8);
  if (!xPtr || !yPtr || !sPtr || !pPtr) return null;
  const matvec = (x, y) => { f64(xPtr, N).set(x); KW.bem_matvec(xPtr, yPtr); y.set(f64(yPtr, N)); };
  const rhs = [];
  for (let j = 0; j < 6; j++) rhs.push(new Float64Array(N));
  panels.forEach((p, i) => {
    const rx = p.c[0] - ref[0], ry = p.c[1] - ref[1], rz = p.c[2] - ref[2], n = p.n;
    rhs[0][i] = n[0]; rhs[1][i] = n[1]; rhs[2][i] = n[2];
    rhs[3][i] = ry * n[2] - rz * n[1]; rhs[4][i] = rz * n[0] - rx * n[2]; rhs[5][i] = rx * n[1] - ry * n[0];
  });
  const iterations = [];
  const sig = f64(sPtr, 6 * N);
  for (let j = 0; j < 6; j++) {
    const { x, iters, resid } = gmres(matvec, rhs[j], N, { tol: opts.tol ?? 1e-8, maxIter: opts.maxIter ?? 400, restart: 60 });
    f64(sPtr, 6 * N).set(x, j * N);
    iterations.push({ iters, resid });
    onProgress(0.55 + 0.3 * ((j + 1) / 6), 'Solving source strengths');
  }
  void sig;
  onProgress(0.85, 'Integrating pressures');
  KW.bem_potentials(sPtr, pPtr, 6);
  const phi = f64(pPtr, 6 * N);
  const M = Array.from({ length: 6 }, () => new Array(6).fill(0));
  for (let l = 0; l < 6; l++)
    for (let j = 0; j < 6; j++) {
      let s = 0;
      for (let i = 0; i < N; i++) s += phi[l * N + i] * rhs[j][i] * panels[i].area;
      M[l][j] = -rho * s;
    }
  let asym = 0, mag = 0;
  for (let l = 0; l < 6; l++) for (let j = 0; j < 6; j++) { asym = Math.max(asym, Math.abs(M[l][j] - M[j][l])); mag = Math.max(mag, Math.abs(M[l][j])); }
  const Ms = M.map((row, l) => row.map((v, j) => 0.5 * (v + M[j][l])));
  onProgress(1, 'Done');
  return { M: Ms, raw: M, panels: N, iterations, asymmetry: mag ? asym / mag : 0, ref, engine: 'wasm' };
}

// Restarted GMRES (Saad) for dense nonsymmetric systems.
export function gmres(matvec, b, n, { tol = 1e-8, maxIter = 300, restart = 50 } = {}) {
  const x = new Float64Array(n);
  const r = new Float64Array(n), w = new Float64Array(n);
  const bnorm = Math.sqrt(b.reduce((s, v) => s + v * v, 0)) || 1;
  let total = 0, resid = 1;
  while (total < maxIter) {
    matvec(x, w);
    for (let i = 0; i < n; i++) r[i] = b[i] - w[i];
    const beta = Math.sqrt(r.reduce((s, v) => s + v * v, 0));
    resid = beta / bnorm;
    if (resid < tol) break;
    const m = restart;
    const V = [Float64Array.from(r, (v) => v / beta)];
    const H = Array.from({ length: m + 1 }, () => new Float64Array(m));
    const cs = new Float64Array(m), sn = new Float64Array(m);
    const g = new Float64Array(m + 1);
    g[0] = beta;
    let k = 0;
    for (; k < m && total < maxIter; k++, total++) {
      matvec(V[k], w);
      for (let j = 0; j <= k; j++) {
        let h = 0;
        const Vj = V[j];
        for (let i = 0; i < n; i++) h += w[i] * Vj[i];
        H[j][k] = h;
        for (let i = 0; i < n; i++) w[i] -= h * Vj[i];
      }
      const hn = Math.sqrt(w.reduce((s, v) => s + v * v, 0));
      H[k + 1][k] = hn;
      V.push(Float64Array.from(w, (v) => (hn > 0 ? v / hn : 0)));
      for (let j = 0; j < k; j++) {
        const t = cs[j] * H[j][k] + sn[j] * H[j + 1][k];
        H[j + 1][k] = -sn[j] * H[j][k] + cs[j] * H[j + 1][k];
        H[j][k] = t;
      }
      const den = Math.hypot(H[k][k], H[k + 1][k]) || 1;
      cs[k] = H[k][k] / den;
      sn[k] = H[k + 1][k] / den;
      H[k][k] = cs[k] * H[k][k] + sn[k] * H[k + 1][k];
      H[k + 1][k] = 0;
      g[k + 1] = -sn[k] * g[k];
      g[k] = cs[k] * g[k];
      resid = Math.abs(g[k + 1]) / bnorm;
      if (resid < tol) { k++; total++; break; }
    }
    const y = new Float64Array(k);
    for (let i = k - 1; i >= 0; i--) {
      let s = g[i];
      for (let j = i + 1; j < k; j++) s -= H[i][j] * y[j];
      y[i] = s / H[i][i];
    }
    for (let j = 0; j < k; j++) {
      const Vj = V[j];
      for (let i = 0; i < n; i++) x[i] += y[j] * Vj[i];
    }
    if (resid < tol) break;
  }
  return { x, iters: total, resid };
}

// Change the reference point of a 6x6 rigid-body/added-mass matrix from O to G.
// M_G = H^T M_O H with H = [[I, -S(r_GO)], [0, I]], r_GO = O - G.
export function shiftMassMatrix(M, O, G) {
  const r = [O[0] - G[0], O[1] - G[1], O[2] - G[2]];
  const S = [[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]];
  const H = Array.from({ length: 6 }, (_, i) => Array.from({ length: 6 }, (_, j) => (i === j ? 1 : 0)));
  for (let i = 0; i < 3; i++) for (let j = 0; j < 3; j++) H[i][3 + j] = -S[i][j];
  const HT = H[0].map((_, j) => H.map((row) => row[j]));
  const mul = (A, B) => A.map((row) => B[0].map((_, j) => row.reduce((s, v, k) => s + v * B[k][j], 0)));
  return mul(mul(HT, M), H);
}
