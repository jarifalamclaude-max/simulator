// DUBURI heavy numerical kernels, compiled to WebAssembly (clang --target=wasm32,
// -O3 -msimd128). Freestanding: no libc, a bump allocator over linear memory.
//
//  * Panel method (BEM): influence-matrix assembly with adaptive sub-triangle
//    quadrature, the dense mat-vec used by GMRES, and the potential integration.
//    Same numerics as src/hydro/bem.js.
//  * Lattice-Boltzmann D3Q19 step (regularised BGK + Smagorinsky LES, half-way
//    or Bouzidi bounce-back with moving walls, free-stream / outflow faces,
//    sponge). Same numerics as src/cfd/cpu.js.
//
// Build: npm run build:wasm   (tools/build-wasm.mjs)

typedef unsigned char u8;
typedef unsigned int u32;
typedef int i32;
#define EXPORT(name) __attribute__((export_name(#name)))
#define IMPORT(name) __attribute__((import_module("env"), import_name(#name)))

IMPORT(log) double js_log(double);

extern unsigned char __heap_base;
static u32 heap_top = 0;

EXPORT(reset_heap) void reset_heap(void) { heap_top = (u32)&__heap_base; }
EXPORT(alloc) u32 alloc(u32 bytes) {
  if (!heap_top) heap_top = (u32)&__heap_base;
  u32 p = (heap_top + 15u) & ~15u;
  u32 end = p + bytes;
  u32 have = __builtin_wasm_memory_size(0) * 65536u;
  if (end > have) {
    u32 need = (end - have + 65535u) / 65536u;
    if (__builtin_wasm_memory_grow(0, need) == (u32)-1) return 0;
  }
  heap_top = end;
  return p;
}

static inline double dsqrt(double x) { return __builtin_sqrt(x); }
static inline float fsqrt(float x) { return __builtin_sqrtf(x); }
static inline double dabs(double x) { return x < 0 ? -x : x; }
static double dasinh(double x) {
  double a = dabs(x);
  double r = js_log(a + dsqrt(a * a + 1.0));
  return x < 0 ? -r : r;
}

// =====================================================================
// Panel method
// =====================================================================
#define FOUR_PI 12.566370614359172

static u32 Np;            // panels
static double *pc;        // centroids 3N
static double *pn;        // normals 3N
static double *pa;        // areas N
static double *ps;        // size (2 * max vertex distance) N
static double *pv;        // vertices 12N (4 per panel)
static float *K;          // influence matrix N*N
static double *qpts[3];   // quadrature points per level (m = 2, 4, 6), 4*m*m per panel
static double *qw[3];     // weights
static const int QM[3] = {2, 4, 6};

EXPORT(bem_init) u32 bem_init(u32 n) {
  reset_heap();
  Np = n;
  pc = (double *)alloc(n * 3 * 8); pn = (double *)alloc(n * 3 * 8);
  pa = (double *)alloc(n * 8); ps = (double *)alloc(n * 8); pv = (double *)alloc(n * 12 * 8);
  K = (float *)alloc(n * n * 4);
  for (int l = 0; l < 3; l++) { int m = QM[l]; qpts[l] = (double *)alloc(n * 4 * m * m * 3 * 8); qw[l] = (double *)alloc(n * 4 * m * m * 8); }
  return K ? 1 : 0;
}
EXPORT(bem_centroids) u32 bem_centroids(void) { return (u32)pc; }
EXPORT(bem_normals) u32 bem_normals(void) { return (u32)pn; }
EXPORT(bem_areas) u32 bem_areas(void) { return (u32)pa; }
EXPORT(bem_verts) u32 bem_verts(void) { return (u32)pv; }

static void quad_build(void) {
  for (u32 k = 0; k < Np; k++) {
    const double *c = pc + 3 * k;
    double r = 0;
    for (int e = 0; e < 4; e++) {
      const double *v = pv + 12 * k + 3 * e;
      double dx = v[0] - c[0], dy = v[1] - c[1], dz = v[2] - c[2];
      double d = dsqrt(dx * dx + dy * dy + dz * dz);
      if (d > r) r = d;
    }
    ps[k] = 2 * r;
    for (int l = 0; l < 3; l++) {
      int m = QM[l], cnt = 0;
      double *P = qpts[l] + (u32)k * 4 * m * m * 3, *W = qw[l] + (u32)k * 4 * m * m;
      for (int e = 0; e < 4; e++) {
        const double *B = pv + 12 * k + 3 * e, *Cv = pv + 12 * k + 3 * ((e + 1) & 3);
        double ab[3] = {B[0] - c[0], B[1] - c[1], B[2] - c[2]}, ac[3] = {Cv[0] - c[0], Cv[1] - c[1], Cv[2] - c[2]};
        double cr0 = ab[1] * ac[2] - ab[2] * ac[1], cr1 = ab[2] * ac[0] - ab[0] * ac[2], cr2 = ab[0] * ac[1] - ab[1] * ac[0];
        double area = 0.5 * dsqrt(cr0 * cr0 + cr1 * cr1 + cr2 * cr2);
        double sub = area / (m * m);
        for (int i = 0; i < m; i++)
          for (int j = 0; j < m - i; j++) {
            double u = (i + 1.0 / 3) / m, vv = (j + 1.0 / 3) / m;
            P[3 * cnt] = c[0] + u * ab[0] + vv * ac[0]; P[3 * cnt + 1] = c[1] + u * ab[1] + vv * ac[1]; P[3 * cnt + 2] = c[2] + u * ab[2] + vv * ac[2];
            W[cnt++] = area < 1e-18 ? 0 : sub;
            if (i + j < m - 1) {
              double u2 = (i + 2.0 / 3) / m, v2 = (j + 2.0 / 3) / m;
              P[3 * cnt] = c[0] + u2 * ab[0] + v2 * ac[0]; P[3 * cnt + 1] = c[1] + u2 * ab[1] + v2 * ac[1]; P[3 * cnt + 2] = c[2] + u2 * ab[2] + v2 * ac[2];
              W[cnt++] = area < 1e-18 ? 0 : sub;
            }
          }
      }
      while (cnt < 4 * m * m) W[cnt++] = 0;
    }
  }
}

static inline int qlevel(double d, double sz) { return d > 4 * sz ? -1 : d > 2 * sz ? 0 : d > sz ? 1 : 2; }

// K[i,k] = d/dn_i of the unit-source potential of panel k at centroid i
EXPORT(bem_assemble) void bem_assemble(void) {
  quad_build();
  for (u32 i = 0; i < Np; i++) {
    double xi = pc[3 * i], yi = pc[3 * i + 1], zi = pc[3 * i + 2], n0 = pn[3 * i], n1 = pn[3 * i + 1], n2 = pn[3 * i + 2];
    float *row = K + (u32)i * Np;
    for (u32 k = 0; k < Np; k++) {
      if (k == i) { row[k] = 0; continue; }
      double dx = xi - pc[3 * k], dy = yi - pc[3 * k + 1], dz = zi - pc[3 * k + 2];
      double d2 = dx * dx + dy * dy + dz * dz, d = dsqrt(d2);
      int l = qlevel(d, ps[k]);
      if (l < 0) { row[k] = (float)(pa[k] * (n0 * dx + n1 * dy + n2 * dz) / (FOUR_PI * d2 * d)); continue; }
      int m = QM[l], cnt = 4 * m * m;
      const double *P = qpts[l] + (u32)k * cnt * 3, *W = qw[l] + (u32)k * cnt;
      double s = 0;
      for (int q = 0; q < cnt; q++) {
        double ex = xi - P[3 * q], ey = yi - P[3 * q + 1], ez = zi - P[3 * q + 2];
        double r2 = ex * ex + ey * ey + ez * ez;
        s += W[q] * (n0 * ex + n1 * ey + n2 * ez) / (r2 * dsqrt(r2));
      }
      row[k] = (float)(s / FOUR_PI);
    }
  }
}

// y = (I/2 + K) x
EXPORT(bem_matvec) void bem_matvec(const double *x, double *y) {
  for (u32 i = 0; i < Np; i++) {
    const float *row = K + (u32)i * Np;
    double s0 = 0, s1 = 0, s2 = 0, s3 = 0;
    u32 k = 0;
    for (; k + 4 <= Np; k += 4) { s0 += row[k] * x[k]; s1 += row[k + 1] * x[k + 1]; s2 += row[k + 2] * x[k + 2]; s3 += row[k + 3] * x[k + 3]; }
    for (; k < Np; k++) s0 += row[k] * x[k];
    y[i] = 0.5 * x[i] + (s0 + s1) + (s2 + s3);
  }
}

// integral over panel k of 1/|P - xi| for P inside the panel plane
static double polygon_invr(const double *P, u32 k) {
  double s = 0;
  for (int e = 0; e < 4; e++) {
    const double *A = pv + 12 * k + 3 * e, *B = pv + 12 * k + 3 * ((e + 1) & 3);
    double t0 = B[0] - A[0], t1 = B[1] - A[1], t2 = B[2] - A[2];
    double L = dsqrt(t0 * t0 + t1 * t1 + t2 * t2);
    if (L < 1e-15) continue;
    t0 /= L; t1 /= L; t2 /= L;
    double a0 = A[0] - P[0], a1 = A[1] - P[1], a2 = A[2] - P[2];
    double s1 = a0 * t0 + a1 * t1 + a2 * t2, s2 = s1 + L;
    double p0 = a0 - s1 * t0, p1 = a1 - s1 * t1, p2 = a2 - s1 * t2;
    double d = dsqrt(p0 * p0 + p1 * p1 + p2 * p2);
    if (d < 1e-15) continue;
    s += d * (dasinh(s2 / d) - dasinh(s1 / d));
  }
  return s;
}

// phi_j(i) = sum_k G_ik sigma_j(k) for nrhs source vectors (sigma: nrhs*N, phi: nrhs*N)
EXPORT(bem_potentials) void bem_potentials(const double *sigma, double *phi, u32 nrhs) {
  for (u32 i = 0; i < Np; i++) {
    const double *Pp = pc + 3 * i;
    double acc[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    for (u32 k = 0; k < Np; k++) {
      double g;
      if (k == i) g = -polygon_invr(Pp, k) / FOUR_PI;
      else {
        double dx = Pp[0] - pc[3 * k], dy = Pp[1] - pc[3 * k + 1], dz = Pp[2] - pc[3 * k + 2];
        double d = dsqrt(dx * dx + dy * dy + dz * dz);
        int l = qlevel(d, ps[k]);
        if (l < 0) g = -pa[k] / (FOUR_PI * d);
        else {
          int m = QM[l], cnt = 4 * m * m;
          const double *P = qpts[l] + (u32)k * cnt * 3, *W = qw[l] + (u32)k * cnt;
          double s = 0;
          for (int q = 0; q < cnt; q++) {
            double ex = Pp[0] - P[3 * q], ey = Pp[1] - P[3 * q + 1], ez = Pp[2] - P[3 * q + 2];
            s += W[q] / dsqrt(ex * ex + ey * ey + ez * ez);
          }
          g = -s / FOUR_PI;
        }
      }
      for (u32 j = 0; j < nrhs; j++) acc[j] += g * sigma[j * Np + k];
    }
    for (u32 j = 0; j < nrhs; j++) phi[j * Np + i] = acc[j];
  }
}

// =====================================================================
// Lattice Boltzmann D3Q19
// =====================================================================
static const i32 CX[19] = {0, 1, -1, 0, 0, 0, 0, 1, -1, 1, -1, 1, -1, 1, -1, 0, 0, 0, 0};
static const i32 CY[19] = {0, 0, 0, 1, -1, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 1, -1, 1, -1};
static const i32 CZ[19] = {0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 1, -1, -1, 1, 1, -1, -1, 1};
static const i32 OPP[19] = {0, 2, 1, 4, 3, 6, 5, 8, 7, 10, 9, 12, 11, 14, 13, 16, 15, 18, 17};
#define W0 (1.0 / 3.0)
#define W1 (1.0 / 18.0)
#define W2 (1.0 / 36.0)
static const double WQ[19] = {W0, W1, W1, W1, W1, W1, W1, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2, W2};

static i32 NX, NY, NZ;
static u32 NC;
static float *FA, *FB;     // distributions (shifted, g = f - w), SoA [i * NC + cell]
static u8 *MASK, *QW;      // solid mask, wall distances (q * 255, [cell * 19 + i])
static double *PAR;        // tau0, smagK, om0, om1, om2, r0, r1, r2, geqInf[19]
static double *SPONGE;     // per-x extra relaxation time

EXPORT(lbm_init) u32 lbm_init(i32 nx, i32 ny, i32 nz, i32 with_q) {
  reset_heap();
  NX = nx; NY = ny; NZ = nz; NC = (u32)nx * ny * nz;
  FA = (float *)alloc(NC * 19 * 4); FB = (float *)alloc(NC * 19 * 4);
  MASK = (u8 *)alloc(NC);
  QW = with_q ? (u8 *)alloc(NC * 19) : 0;
  PAR = (double *)alloc(32 * 8);
  SPONGE = (double *)alloc((u32)nx * 8);
  if (!FA || !FB || !MASK || !PAR || !SPONGE || (with_q && !QW)) return 0;
  for (u32 i = 0; i < NC * 19; i++) { FA[i] = 0; FB[i] = 0; }
  return 1;
}
EXPORT(lbm_a) u32 lbm_a(void) { return (u32)FA; }
EXPORT(lbm_b) u32 lbm_b(void) { return (u32)FB; }
EXPORT(lbm_mask) u32 lbm_mask(void) { return (u32)MASK; }
EXPORT(lbm_q) u32 lbm_q(void) { return (u32)QW; }
EXPORT(lbm_par) u32 lbm_par(void) { return (u32)PAR; }
EXPORT(lbm_sponge) u32 lbm_sponge(void) { return (u32)SPONGE; }

static inline double wall_value(const float *A, u32 id, i32 x, i32 y, i32 z, int i) {
  int o = OPP[i];
  u32 qb = QW ? QW[id * 19 + i] : 0;
  double qq = qb ? qb / 255.0 : 0.5;
  double om0 = PAR[2], om1 = PAR[3], om2 = PAR[4];
  double corr = 0;
  if (om0 != 0 || om1 != 0 || om2 != 0) {
    double lx = x - qq * CX[i] - PAR[5], ly = y - qq * CY[i] - PAR[6], lz = z - qq * CZ[i] - PAR[7];
    corr = 6 * WQ[i] * (CX[i] * (om1 * lz - om2 * ly) + CY[i] * (om2 * lx - om0 * lz) + CZ[i] * (om0 * ly - om1 * lx));
  }
  double go = A[(u32)o * NC + id];
  if (!qb) return go + corr;
  if (qq < 0.5) {
    i32 fx = x + CX[i], fy = y + CY[i], fz = z + CZ[i];
    if (fx < 0 || fy < 0 || fz < 0 || fx >= NX || fy >= NY || fz >= NZ) return go + corr;
    u32 ff = (u32)fx + (u32)NX * ((u32)fy + (u32)NY * (u32)fz);
    if (MASK[ff]) return go + corr;
    return 2 * qq * go + (1 - 2 * qq) * A[(u32)o * NC + ff] + corr;
  }
  return (go + (2 * qq - 1) * A[(u32)i * NC + id] + corr) / (2 * qq);
}

// n steps; returns which buffer holds the latest state (0: A, 1: B)
static int cur = 0;
EXPORT(lbm_current) i32 lbm_current(void) { return cur; }
EXPORT(lbm_set_current) void lbm_set_current(i32 c) { cur = c; }

EXPORT(lbm_step) void lbm_step(i32 nsteps) {
  i32 off[19];
  for (int i = 0; i < 19; i++) off[i] = CX[i] + NX * (CY[i] + NY * CZ[i]);
  const double tau0 = PAR[0], smagK = PAR[1];
  const double *geqInf = PAR + 8;
  for (i32 s = 0; s < nsteps; s++) {
    const float *A = cur ? FB : FA;
    float *B = cur ? FA : FB;
    for (i32 z = 0; z < NZ; z++) for (i32 y = 0; y < NY; y++) {
      int rowInterior = z > 0 && z < NZ - 1 && y > 0 && y < NY - 1;
      u32 rowBase = (u32)NX * ((u32)y + (u32)NY * (u32)z);
      for (i32 x = 0; x < NX; x++) {
        u32 id = (u32)x + rowBase;
        if (MASK[id]) { for (int i = 0; i < 19; i++) B[(u32)i * NC + id] = 0; continue; }
        int interior = rowInterior && x > 0 && x < NX - 1;
        double g[19];
        for (int i = 0; i < 19; i++) {
          u32 src;
          if (interior) src = (u32)((i32)id - off[i]);
          else {
            i32 sx = x - CX[i], sy = y - CY[i], sz = z - CZ[i];
            if (sy < 0 || sy >= NY || sz < 0 || sz >= NZ || sx < 0) { g[i] = geqInf[i]; continue; }
            src = (u32)(sx >= NX ? NX - 1 : sx) + (u32)NX * ((u32)sy + (u32)NY * (u32)sz);
          }
          if (MASK[src] == 0) { g[i] = A[(u32)i * NC + src]; continue; }
          g[i] = wall_value(A, id, x, y, z, i);
        }
        double rho = 1, jx = 0, jy = 0, jz = 0;
        for (int i = 0; i < 19; i++) { double v = g[i]; rho += v; jx += CX[i] * v; jy += CY[i] * v; jz += CZ[i] * v; }
        double ux = jx / rho, uy = jy / rho, uz = jz / rho;
        double uu15 = 1.5 * (ux * ux + uy * uy + uz * uz), rm1 = rho - 1;
        double eq[19], pxx = 0, pyy = 0, pzz = 0, pxy = 0, pxz = 0, pyz = 0;
        for (int i = 0; i < 19; i++) {
          double cu = CX[i] * ux + CY[i] * uy + CZ[i] * uz;
          double e = WQ[i] * (rm1 + rho * (3 * cu + 4.5 * cu * cu - uu15));
          eq[i] = e;
          double ne = g[i] - e;
          pxx += CX[i] * CX[i] * ne; pyy += CY[i] * CY[i] * ne; pzz += CZ[i] * CZ[i] * ne;
          pxy += CX[i] * CY[i] * ne; pxz += CX[i] * CZ[i] * ne; pyz += CY[i] * CZ[i] * ne;
        }
        double qn = dsqrt(pxx * pxx + pyy * pyy + pzz * pzz + 2 * (pxy * pxy + pxz * pxz + pyz * pyz));
        double tau = 0.5 * (tau0 + dsqrt(tau0 * tau0 + smagK * qn / rho)) + SPONGE[x];
        double keep = 1 - 1 / tau, t2 = (pxx + pyy + pzz) / 3;
        for (int i = 0; i < 19; i++) {
          double cx = CX[i], cy = CY[i], cz = CZ[i];
          double fneq = 4.5 * WQ[i] * (cx * cx * pxx + cy * cy * pyy + cz * cz * pzz - t2 + 2 * (cx * cy * pxy + cx * cz * pxz + cy * cz * pyz));
          B[(u32)i * NC + id] = (float)(eq[i] + keep * fneq);
        }
      }
    }
    cur ^= 1;
  }
}
