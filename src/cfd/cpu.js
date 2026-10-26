// Reference D3Q19 lattice-Boltzmann solver on the CPU (regularised BGK with a
// Smagorinsky LES closure), used for validation, in Node tests and as the
// fallback when the browser cannot render to float textures. The GPU solver
// implements exactly the same algorithm.
//
// Boundary conditions:
//   * solid cells: half-way bounce-back, moving-wall correction u_w = omega x r
//   * x = 0, y and z faces: free-stream equilibrium (rho = 1, u = uInf)
//   * x = nx-1: zero-gradient outflow, with a sponge layer that raises the
//     viscosity smoothly over the last part of the domain
// Distributions are stored shifted, g = f - w, for float32 precision.

import { Q, C, W, OPP, SMAG_K } from './d3q19.js';
import { kernels } from '../wasm/kernels.js';

const CX = Int32Array.from(C, (c) => c[0]), CY = Int32Array.from(C, (c) => c[1]), CZ = Int32Array.from(C, (c) => c[2]);
const WQ = Float64Array.from(W), OPQ = Int32Array.from(OPP);

export class LBMCPU {
  /**
   * c: { nx, ny, nz, mask (Uint8Array, 1 = solid) }
   * p: { tau0, smag, uInf [3], omega [3], ref [3], spongeFrac, spongeAmp }
   */
  constructor(c, p) {
    this.nx = c.nx; this.ny = c.ny; this.nz = c.nz;
    this.N = this.nx * this.ny * this.nz;
    this.mask = c.mask;
    this.q = c.q || null; // interpolated-wall distances (Uint8, q * 255), see study.linkDistances
    this.p = { smag: 0.12, uInf: [0, 0, 0], omega: [0, 0, 0], ref: [0, 0, 0], spongeFrac: 0.2, spongeAmp: 0.5, ...p };
    // WebAssembly kernel (one active lattice per thread) or plain JS arrays
    const KW = this.p.useWasm ? kernels() : null;
    this.wasm = null;
    if (KW && KW.lbm_init(this.nx, this.ny, this.nz, this.q ? 1 : 0)) {
      this.wasm = KW;
      const buf = KW.memory.buffer;
      this.viewA = new Float32Array(buf, KW.lbm_a(), Q * this.N);
      this.viewB = new Float32Array(buf, KW.lbm_b(), Q * this.N);
      new Uint8Array(buf, KW.lbm_mask(), this.N).set(this.mask);
      if (this.q) new Uint8Array(buf, KW.lbm_q(), this.N * 19).set(this.q);
      this.par = new Float64Array(buf, KW.lbm_par(), 32);
      new Float64Array(buf, KW.lbm_sponge(), this.nx).set(Float64Array.from({ length: this.nx }, (_, x) => this.sponge(x)));
      KW.lbm_set_current(0);
      this.A = this.viewA; this.B = this.viewB;
    } else {
      this.A = new Float32Array(Q * this.N);
      this.B = new Float32Array(Q * this.N);
    }
    this.off = C.map((c) => c[0] + this.nx * (c[1] + this.ny * c[2]));
    this.geqInf = new Float64Array(Q);
    this.setInflow(this.p.uInf);
    this.steps = 0;
    this.links = this.findLinks();
  }

  setInflow(u) {
    this.p.uInf = u.slice();
    const [ux, uy, uz] = u, uu = ux * ux + uy * uy + uz * uz;
    for (let i = 0; i < Q; i++) {
      const cu = C[i][0] * ux + C[i][1] * uy + C[i][2] * uz;
      this.geqInf[i] = W[i] * (3 * cu + 4.5 * cu * cu - 1.5 * uu);
    }
  }

  // fluid cell + direction i whose upstream neighbour (x - c_i) is solid
  findLinks() {
    const { nx, ny, nz, mask } = this;
    const cells = [], dirs = [];
    for (let z = 0; z < nz; z++) for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
      const id = x + nx * (y + ny * z);
      if (mask[id]) continue;
      for (let i = 1; i < Q; i++) {
        const sx = x - C[i][0], sy = y - C[i][1], sz = z - C[i][2];
        if (sx < 0 || sy < 0 || sz < 0 || sx >= nx || sy >= ny || sz >= nz) continue;
        if (mask[sx + nx * (sy + ny * sz)]) { cells.push(id); dirs.push(i); }
      }
    }
    return { cells: Int32Array.from(cells), dirs: Uint8Array.from(dirs) };
  }

  sponge(x) {
    const x0 = Math.floor(this.nx * (1 - this.p.spongeFrac));
    if (x <= x0) return 0;
    const s = (x - x0) / Math.max(1, this.nx - 1 - x0);
    return this.p.spongeAmp * s * s;
  }

  step(n = 1) {
    if (this.wasm) {
      const P = this.par, p = this.p;
      P[0] = p.tau0; P[1] = SMAG_K * p.smag * p.smag;
      P[2] = p.omega[0]; P[3] = p.omega[1]; P[4] = p.omega[2];
      P[5] = p.ref[0]; P[6] = p.ref[1]; P[7] = p.ref[2];
      P.set(this.geqInf, 8);
      this.wasm.lbm_set_current(this.A === this.viewA ? 0 : 1);
      this.wasm.lbm_step(n);
      const cur = this.wasm.lbm_current();
      this.A = cur ? this.viewB : this.viewA; this.B = cur ? this.viewA : this.viewB;
      this.steps += n;
      return;
    }
    for (let k = 0; k < n; k++) this.step1();
  }

  step1() {
    const { nx, ny, nz, N, mask, A, B, off, geqInf } = this;
    const { tau0, smag } = this.p;
    const smagK = SMAG_K * smag * smag;
    const g = this._g || (this._g = new Float64Array(Q));
    const eq = this._eq || (this._eq = new Float64Array(Q));
    if (!this._sp) this._sp = Float64Array.from({ length: nx }, (_, x) => this.sponge(x));
    const sp = this._sp;
    for (let z = 0; z < nz; z++) {
      for (let y = 0; y < ny; y++) {
        const rowInterior = z > 0 && z < nz - 1 && y > 0 && y < ny - 1;
        const rowBase = nx * (y + ny * z);
        for (let x = 0; x < nx; x++) {
          const id = x + rowBase;
          if (mask[id]) { for (let i = 0; i < Q; i++) B[i * N + id] = 0; continue; }
          const interior = rowInterior && x > 0 && x < nx - 1;
          for (let i = 0; i < Q; i++) {
            let src;
            if (interior) src = id - off[i];
            else {
              const sx = x - CX[i], sy = y - CY[i], sz = z - CZ[i];
              if (sy < 0 || sy >= ny || sz < 0 || sz >= nz || sx < 0) { g[i] = geqInf[i]; continue; }
              src = (sx >= nx ? nx - 1 : sx) + nx * (sy + ny * sz);
            }
            if (mask[src] === 0) { g[i] = A[i * N + src]; continue; }
            g[i] = this.wallValue(A, id, x, y, z, i);
          }
          // macroscopic
          let rho = 1, jx = 0, jy = 0, jz = 0;
          for (let i = 0; i < Q; i++) { const v = g[i]; rho += v; jx += CX[i] * v; jy += CY[i] * v; jz += CZ[i] * v; }
          const ux = jx / rho, uy = jy / rho, uz = jz / rho;
          const uu15 = 1.5 * (ux * ux + uy * uy + uz * uz), rm1 = rho - 1;
          let pxx = 0, pyy = 0, pzz = 0, pxy = 0, pxz = 0, pyz = 0;
          for (let i = 0; i < Q; i++) {
            const cx = CX[i], cy = CY[i], cz = CZ[i];
            const cu = cx * ux + cy * uy + cz * uz;
            const e = WQ[i] * (rm1 + rho * (3 * cu + 4.5 * cu * cu - uu15));
            eq[i] = e;
            const ne = g[i] - e;
            pxx += cx * cx * ne; pyy += cy * cy * ne; pzz += cz * cz * ne;
            pxy += cx * cy * ne; pxz += cx * cz * ne; pyz += cy * cz * ne;
          }
          const qn = Math.sqrt(pxx * pxx + pyy * pyy + pzz * pzz + 2 * (pxy * pxy + pxz * pxz + pyz * pyz));
          const tau = 0.5 * (tau0 + Math.sqrt(tau0 * tau0 + smagK * qn / rho)) + sp[x];
          const keep = 1 - 1 / tau;
          const t2 = (pxx + pyy + pzz) / 3;
          for (let i = 0; i < Q; i++) {
            const cx = CX[i], cy = CY[i], cz = CZ[i];
            const fneq = 4.5 * WQ[i] * (cx * cx * pxx + cy * cy * pyy + cz * cz * pzz - t2 + 2 * (cx * cy * pxy + cx * cz * pxz + cy * cz * pyz));
            B[i * N + id] = eq[i] + keep * fneq;
          }
        }
      }
    }
    this.A = B; this.B = A;
    this.steps++;
  }

  // Population entering fluid cell (x,y,z) along direction i from the wall
  // (x - c_i is solid), from the post-collision state A: half-way bounce-back,
  // or Bouzidi linear interpolation when the wall distance q is known, with
  // the moving-wall term for spinning bodies (Lallemand & Luo 2003).
  wallQ(id, i) { const qb = this.q ? this.q[id * 19 + i] : 0; return qb ? qb / 255 : 0.5; }
  wallValue(A, id, x, y, z, i) {
    const { N, nx, ny, nz, mask } = this;
    const o = OPQ[i];
    const qb = this.q ? this.q[id * 19 + i] : 0;
    const qq = qb ? qb / 255 : 0.5;
    const om = this.p.omega, ref = this.p.ref;
    let corr = 0;
    if (om[0] !== 0 || om[1] !== 0 || om[2] !== 0) {
      const lx = x - qq * CX[i] - ref[0], ly = y - qq * CY[i] - ref[1], lz = z - qq * CZ[i] - ref[2];
      corr = 6 * WQ[i] * (CX[i] * (om[1] * lz - om[2] * ly) + CY[i] * (om[2] * lx - om[0] * lz) + CZ[i] * (om[0] * ly - om[1] * lx));
    }
    const go = A[o * N + id];
    if (!qb) return go + corr;
    if (qq < 0.5) {
      const fx = x + CX[i], fy = y + CY[i], fz = z + CZ[i];
      if (fx < 0 || fy < 0 || fz < 0 || fx >= nx || fy >= ny || fz >= nz) return go + corr;
      const ff = fx + nx * (fy + ny * fz);
      if (mask[ff]) return go + corr;
      return 2 * qq * go + (1 - 2 * qq) * A[o * N + ff] + corr;
    }
    return (go + (2 * qq - 1) * A[i * N + id] + corr) / (2 * qq);
  }

  // Momentum-exchange force and moment (about ref) on the solid, for the
  // current post-collision state (lattice units). On a moving (spinning) wall
  // the Galilean-invariant form F = sum[(c_o - u_w) f_o - (c_i - u_w) f_i] is
  // used (Wen et al. 2014; Krueger et al. 2017 sec. 11.4.4): the conventional
  // sum less u_w (f_o - f_i) per link, u_w = omega x l at the wall point.
  force() {
    const { nx, ny, N, A } = this;
    const ref = this.p.ref, om = this.p.omega;
    const moving = om[0] !== 0 || om[1] !== 0 || om[2] !== 0;
    const F = [0, 0, 0], M = [0, 0, 0];
    const { cells, dirs } = this.links;
    for (let k = 0; k < cells.length; k++) {
      const id = cells[k], i = dirs[k], o = OPP[i];
      const x = id % nx, y = Math.floor(id / nx) % ny, z = Math.floor(id / (nx * ny));
      const qq = this.wallQ(id, i);
      const lx = x - qq * CX[i] - ref[0], ly = y - qq * CY[i] - ref[1], lz = z - qq * CZ[i] - ref[2];
      const go = A[o * N + id], gi = this.wallValue(A, id, x, y, z, i);
      const s = go + gi + 2 * W[i];
      let dx = -CX[i] * s, dy = -CY[i] * s, dz = -CZ[i] * s; // c_opp * s
      if (moving) {
        const d = go - gi; // f_o - f_i (the weights cancel)
        dx -= (om[1] * lz - om[2] * ly) * d; dy -= (om[2] * lx - om[0] * lz) * d; dz -= (om[0] * ly - om[1] * lx) * d;
      }
      F[0] += dx; F[1] += dy; F[2] += dz;
      M[0] += ly * dz - lz * dy; M[1] += lz * dx - lx * dz; M[2] += lx * dy - ly * dx;
    }
    return { F, M };
  }

  // Macroscopic field for the 3D flow view (same layout as LBMGPU.snapshot)
  snapshot(stride = 1) {
    const { nx, ny, nz, mask } = this;
    const sx = Math.ceil(nx / stride), sy = Math.ceil(ny / stride), sz = Math.ceil(nz / stride);
    const data = new Float32Array(sx * sy * sz * 4);
    for (let k = 0; k < sz; k++) for (let j = 0; j < sy; j++) for (let i = 0; i < sx; i++) {
      const id = i * stride + nx * (j * stride + ny * (k * stride));
      const o = 4 * (i + sx * (j + sy * k));
      if (mask[id]) { data[o + 3] = NaN; continue; }
      const m = this.macro(id);
      data[o] = m[0]; data[o + 1] = m[1]; data[o + 2] = m[2]; data[o + 3] = m[3];
    }
    return { nx: sx, ny: sy, nz: sz, stride, data };
  }

  // speed, signed out-of-plane vorticity and pressure on a slice
  // axis: 'y' (x-z plane), 'z' (x-y plane) or 'x' (y-z plane)
  macro(id) {
    const { N, A } = this;
    let rho = 1, jx = 0, jy = 0, jz = 0;
    for (let i = 0; i < Q; i++) { const v = A[i * N + id]; rho += v; jx += C[i][0] * v; jy += C[i][1] * v; jz += C[i][2] * v; }
    return [jx / rho, jy / rho, jz / rho, rho];
  }
  slice(axis, index) {
    const { nx, ny, nz, mask } = this;
    const [W2, H2] = axis === 'y' ? [nx, nz] : axis === 'z' ? [nx, ny] : [ny, nz];
    const speed = new Float32Array(W2 * H2), vort = new Float32Array(W2 * H2), pres = new Float32Array(W2 * H2), solid = new Uint8Array(W2 * H2);
    const at = (a, b) => (axis === 'y' ? [a, index, b] : axis === 'z' ? [a, b, index] : [index, a, b]);
    const idOf = (x, y, z) => x + nx * (y + ny * z);
    const vel = (a, b) => {
      a = Math.max(0, Math.min(W2 - 1, a)); b = Math.max(0, Math.min(H2 - 1, b));
      const [x, y, z] = at(a, b);
      const id = idOf(x, y, z);
      return mask[id] ? [0, 0, 0, 1] : this.macro(id);
    };
    // in-plane components (u along a, v along b)
    const comp = axis === 'y' ? [0, 2] : axis === 'z' ? [0, 1] : [1, 2];
    for (let b = 0; b < H2; b++) for (let a = 0; a < W2; a++) {
      const k = a + W2 * b;
      const [x, y, z] = at(a, b);
      const id = idOf(x, y, z);
      if (mask[id]) { solid[k] = 1; continue; }
      const m = this.macro(id);
      speed[k] = Math.hypot(m[0], m[1], m[2]);
      pres[k] = (m[3] - 1) / 3;
      const ua = vel(a + 1, b)[comp[1]] - vel(a - 1, b)[comp[1]];
      const ub = vel(a, b + 1)[comp[0]] - vel(a, b - 1)[comp[0]];
      vort[k] = 0.5 * (ua - ub);
    }
    return { w: W2, h: H2, speed, vort, pres, solid };
  }
}
