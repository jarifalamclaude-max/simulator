// GPU D3Q19 lattice-Boltzmann solver (WebGL2 fragment shaders, float32
// render targets). Same algorithm as cpu.js: regularised BGK + Smagorinsky,
// half-way bounce-back with moving-wall correction, free-stream equilibrium
// faces, zero-gradient outflow with a sponge layer, momentum-exchange forces.
//
// The 3D lattice is stored as a 2D atlas of z-slices; the 19 shifted
// distributions live in five RGBA32F textures (ping-pong pairs). A mask
// texture marks solid cells (1.0) and fluid cells next to a solid (0.5) so
// that most cells skip the boundary checks.

import { Q, C, W, OPP, SMAG_K } from './d3q19.js';

const CH = 'xyzw';
const f1 = (v) => { const s = String(+v); return /[.eE]/.test(s) ? s : `${s}.0`; };
const cvec = (i) => `vec3(${f1(C[i][0])}, ${f1(C[i][1])}, ${f1(C[i][2])})`;

export function gpuSupport() {
  try {
    const c = document.createElement('canvas');
    const gl = c.getContext('webgl2');
    if (!gl) return { ok: false, reason: 'WebGL2 is not available in this browser' };
    if (!gl.getExtension('EXT_color_buffer_float')) return { ok: false, reason: 'this GPU/browser cannot render to float textures' };
    const info = gl.getExtension('WEBGL_debug_renderer_info');
    const res = { ok: true, maxTex: gl.getParameter(gl.MAX_TEXTURE_SIZE), maxDraw: gl.getParameter(gl.MAX_DRAW_BUFFERS), renderer: info ? gl.getParameter(info.UNMASKED_RENDERER_WEBGL) : gl.getParameter(gl.RENDERER) };
    gl.getExtension('WEBGL_lose_context')?.loseContext();
    return res;
  } catch (e) { return { ok: false, reason: e.message }; }
}

const HEADER_Q = 'uniform sampler2D Q0, Q1, Q2, Q3, Q4;\n';
const HEADER = `#version 300 es
precision highp float; precision highp int; precision highp sampler2D;
uniform sampler2D T0, T1, T2, T3, T4, S;
uniform ivec3 N; uniform int TX;
uniform vec3 omega, refp;
ivec2 atlas(ivec3 p) { return ivec2((p.z % TX) * N.x + p.x, (p.z / TX) * N.y + p.y); }
ivec3 cellOf(ivec2 fc) { int tx = fc.x / N.x, ty = fc.y / N.y; return ivec3(fc.x - tx * N.x, fc.y - ty * N.y, ty * TX + tx); }
float maskAt(ivec2 a) { return texelFetch(S, a, 0).r; }
`;

function readOwn() {
  let s = 'vec4 a0 = texelFetch(T0, fc, 0), a1 = texelFetch(T1, fc, 0), a2 = texelFetch(T2, fc, 0), a3 = texelFetch(T3, fc, 0), a4 = texelFetch(T4, fc, 0);\n  float own[19];\n';
  for (let i = 0; i < Q; i++) s += `  own[${i}] = a${i >> 2}.${CH[i & 3]};\n`;
  return s;
}

// GLSL computing the population entering the cell from the wall along
// direction i (same rules as LBMCPU.wallValue): plain bounce-back, or Bouzidi
// linear interpolation with the stored wall distance q.
function wallCode(i, tgt, interp) {
  const o = OPP[i], c = cvec(i), w6 = f1(6 * W[i]);
  if (!interp) return `${tgt} = own[${o}] + ${w6} * dot(${c}, cross(omega, pf - 0.5 * ${c} - refp));`;
  return `{
      float qv = texelFetch(Q${i >> 2}, fc, 0).${CH[i & 3]};
      float qe = qv > 0.0 ? qv : 0.5;
      float corr = ${w6} * dot(${c}, cross(omega, pf - qe * ${c} - refp));
      if (qv <= 0.0) ${tgt} = own[${o}] + corr;
      else if (qv < 0.5) {
        ivec3 ff = p + ivec3(${C[i][0]}, ${C[i][1]}, ${C[i][2]});
        if (ff.x >= 0 && ff.y >= 0 && ff.z >= 0 && ff.x < N.x && ff.y < N.y && ff.z < N.z && maskAt(atlas(ff)) < 0.75)
          ${tgt} = 2.0 * qv * own[${o}] + (1.0 - 2.0 * qv) * texelFetch(T${o >> 2}, atlas(ff), 0).${CH[o & 3]} + corr;
        else ${tgt} = own[${o}] + corr;
      } else ${tgt} = (own[${o}] + (2.0 * qv - 1.0) * own[${i}] + corr) / (2.0 * qv);
    }`;
}

function stepSource(outTex, interp) {
  let s = HEADER + (interp ? HEADER_Q : '') + `uniform float tau0, smagK, spongeStart, spongeAmp;
uniform float geqInf[19];
`;
  outTex.forEach((t, j) => { s += `layout(location=${j}) out vec4 o${j};\n`; });
  s += `void main() {
  ivec2 fc = ivec2(gl_FragCoord.xy);
  ivec3 p = cellOf(fc);
  ${outTex.map((_, j) => `o${j} = vec4(0.0);`).join(' ')}
  float m0 = maskAt(fc);
  if (p.z >= N.z || m0 > 0.75) return;
  bool near = m0 > 0.25;
  ${readOwn()}
  float g[19];
  g[0] = own[0];
  vec3 pf = vec3(p);
  ivec3 s; ivec2 sa;
`;
  for (let i = 1; i < Q; i++) {
    const [cx, cy, cz] = C[i];
    s += `  s = p - ivec3(${cx}, ${cy}, ${cz});
  if (s.y < 0 || s.y >= N.y || s.z < 0 || s.z >= N.z || s.x < 0) g[${i}] = geqInf[${i}];
  else {
    s.x = min(s.x, N.x - 1);
    sa = atlas(s);
    if (near && maskAt(sa) > 0.75) ${wallCode(i, `g[${i}]`, interp)}
    else g[${i}] = texelFetch(T${i >> 2}, sa, 0).${CH[i & 3]};
  }
`;
  }
  const sum = (k) => C.map((c, i) => (c[k] === 0 ? '' : `${c[k] > 0 ? '+' : '-'} g[${i}]`)).join(' ');
  s += `  float rho = 1.0${C.map((_, i) => ` + g[${i}]`).join('')};
  vec3 u = vec3(0.0 ${sum(0)}, 0.0 ${sum(1)}, 0.0 ${sum(2)}) / rho;
  float uu15 = 1.5 * dot(u, u), rm1 = rho - 1.0;
  float eq[19];
  float pxx = 0.0, pyy = 0.0, pzz = 0.0, pxy = 0.0, pxz = 0.0, pyz = 0.0, cu, ne;
`;
  for (let i = 0; i < Q; i++) {
    const [cx, cy, cz] = C[i];
    s += `  cu = dot(${cvec(i)}, u); eq[${i}] = ${f1(W[i])} * (rm1 + rho * (3.0 * cu + 4.5 * cu * cu - uu15)); ne = g[${i}] - eq[${i}];`;
    if (cx) s += ' pxx += ne;';
    if (cy) s += ' pyy += ne;';
    if (cz) s += ' pzz += ne;';
    if (cx * cy) s += ` pxy += ${f1(cx * cy)} * ne;`;
    if (cx * cz) s += ` pxz += ${f1(cx * cz)} * ne;`;
    if (cy * cz) s += ` pyz += ${f1(cy * cz)} * ne;`;
    s += '\n';
  }
  s += `  float qn = sqrt(pxx * pxx + pyy * pyy + pzz * pzz + 2.0 * (pxy * pxy + pxz * pxz + pyz * pyz));
  float sp = 0.0;
  if (pf.x > spongeStart) { float r = (pf.x - spongeStart) / max(1.0, float(N.x - 1) - spongeStart); sp = spongeAmp * r * r; }
  float tau = 0.5 * (tau0 + sqrt(tau0 * tau0 + smagK * qn / rho)) + sp;
  float keep = 1.0 - 1.0 / tau;
  float t2 = (pxx + pyy + pzz) / 3.0;
  float post[20];
  post[19] = 0.0;
`;
  for (let i = 0; i < Q; i++) {
    const [cx, cy, cz] = C[i];
    const terms = [];
    if (cx) terms.push('pxx'); if (cy) terms.push('pyy'); if (cz) terms.push('pzz');
    if (cx * cy) terms.push(`${f1(2 * cx * cy)} * pxy`);
    if (cx * cz) terms.push(`${f1(2 * cx * cz)} * pxz`);
    if (cy * cz) terms.push(`${f1(2 * cy * cz)} * pyz`);
    s += `  post[${i}] = eq[${i}] + keep * ${f1(4.5 * W[i])} * (${terms.length ? terms.join(' + ') : '0.0'} - t2);\n`;
  }
  outTex.forEach((t, j) => { s += `  o${j} = vec4(post[${4 * t}], post[${4 * t + 1}], post[${4 * t + 2}], post[${4 * t + 3}]);\n`; });
  return s + '}\n';
}

const BLOCK = 16;
function forceSource(interp) {
  let s = HEADER + (interp ? HEADER_Q : '') + `uniform ivec2 AT;
layout(location=0) out vec4 oF;
layout(location=1) out vec4 oM;
void main() {
  ivec2 bc = ivec2(gl_FragCoord.xy) * ${BLOCK};
  vec3 F = vec3(0.0), M = vec3(0.0);
  for (int yy = 0; yy < ${BLOCK}; yy++) for (int xx = 0; xx < ${BLOCK}; xx++) {
    ivec2 fc = bc + ivec2(xx, yy);
    if (fc.x >= AT.x || fc.y >= AT.y) continue;
    ivec3 p = cellOf(fc);
    if (p.z >= N.z) continue;
    float m0 = maskAt(fc);
    if (m0 < 0.25 || m0 > 0.75) continue;
    ${readOwn()}
    vec3 pf = vec3(p), l, dp;
    ivec3 s; float sv, gi, qw;
`;
  for (let i = 1; i < Q; i++) {
    s += `    s = p - ivec3(${C[i][0]}, ${C[i][1]}, ${C[i][2]});
    if (s.x >= 0 && s.x < N.x && s.y >= 0 && s.y < N.y && s.z >= 0 && s.z < N.z && maskAt(atlas(s)) > 0.75) {
      ${wallCode(i, 'gi', interp)}
      qw = ${interp ? `texelFetch(Q${i >> 2}, fc, 0).${CH[i & 3]}` : '0.0'};
      l = pf - (qw > 0.0 ? qw : 0.5) * ${cvec(i)} - refp;
      sv = own[${OPP[i]}] + gi + ${f1(2 * W[i])};
      // Galilean-invariant momentum exchange on the moving wall (as LBMCPU.force)
      dp = -${cvec(i)} * sv - cross(omega, l) * (own[${OPP[i]}] - gi); F += dp; M += cross(l, dp);
    }
`;
  }
  return s + `  }
  oF = vec4(F, 0.0); oM = vec4(M, 0.0);
}
`;
}

function visSource() {
  return HEADER + `uniform int axis, idx, field; uniform float uref; uniform vec4 view; // view: x0, y0, w, h in pixels
out vec4 o;
vec4 macroAt(ivec3 p) {
  p = clamp(p, ivec3(0), N - 1);
  ivec2 fc = atlas(p);
  if (maskAt(fc) > 0.75) return vec4(0.0, 0.0, 0.0, 1.0);
  vec4 a0 = texelFetch(T0, fc, 0), a1 = texelFetch(T1, fc, 0), a2 = texelFetch(T2, fc, 0), a3 = texelFetch(T3, fc, 0), a4 = texelFetch(T4, fc, 0);
  float g[19];
  g[0] = a0.x; g[1] = a0.y; g[2] = a0.z; g[3] = a0.w; g[4] = a1.x; g[5] = a1.y; g[6] = a1.z; g[7] = a1.w; g[8] = a2.x; g[9] = a2.y;
  g[10] = a2.z; g[11] = a2.w; g[12] = a3.x; g[13] = a3.y; g[14] = a3.z; g[15] = a3.w; g[16] = a4.x; g[17] = a4.y; g[18] = a4.z;
  float rho = 1.0; vec3 j = vec3(0.0);
${C.map((c, i) => `  rho += g[${i}]; j += ${cvec(i)} * g[${i}];`).join('\n')}
  return vec4(j / rho, rho);
}
ivec3 cellAt(ivec2 ab) { return axis == 0 ? ivec3(ab.x, idx, ab.y) : axis == 1 ? ivec3(ab.x, ab.y, idx) : ivec3(idx, ab.x, ab.y); }
vec3 seqMap(float t) {
  t = clamp(t, 0.0, 1.0);
  vec3 c0 = vec3(0.035, 0.09, 0.13), c1 = vec3(0.224, 0.529, 0.898), c2 = vec3(0.92, 0.97, 0.99);
  return t < 0.6 ? mix(c0, c1, t / 0.6) : mix(c1, c2, (t - 0.6) / 0.4);
}
vec3 divMap(float t) {
  t = clamp(t, -1.0, 1.0);
  vec3 mid = vec3(0.11, 0.15, 0.18), neg = vec3(0.224, 0.529, 0.898), pos = vec3(0.851, 0.349, 0.149);
  return t < 0.0 ? mix(mid, neg, -t) : mix(mid, pos, t);
}
void main() {
  vec2 q = (gl_FragCoord.xy - view.xy) / view.zw;
  ivec2 dims = axis == 0 ? N.xz : axis == 1 ? N.xy : N.yz;
  if (q.x < 0.0 || q.y < 0.0 || q.x >= 1.0 || q.y >= 1.0) { o = vec4(0.024, 0.075, 0.106, 1.0); return; }
  ivec2 ab = ivec2(q * vec2(dims));
  ab.y = dims.y - 1 - ab.y; // second coordinate grows downwards (z is down)
  ivec3 p = cellAt(ab);
  if (maskAt(atlas(p)) > 0.75) { o = vec4(0.79, 0.83, 0.85, 1.0); return; }
  vec4 m = macroAt(p);
  if (field == 0) { o = vec4(seqMap(length(m.xyz) / (1.6 * uref)), 1.0); return; }
  if (field == 2) { o = vec4(divMap(((m.w - 1.0) / 3.0) / (0.5 * uref * uref) / 1.2), 1.0); return; }
  int ca = axis == 0 ? 0 : axis == 1 ? 0 : 1;
  int cb = axis == 0 ? 2 : axis == 1 ? 1 : 2;
  float dvb = macroAt(cellAt(ab + ivec2(1, 0)))[cb] - macroAt(cellAt(ab - ivec2(1, 0)))[cb];
  float dua = macroAt(cellAt(ab + ivec2(0, 1)))[ca] - macroAt(cellAt(ab - ivec2(0, 1)))[ca];
  float w = 0.5 * (dvb - dua);
  o = vec4(divMap(w / (0.25 * uref)), 1.0);
}
`;
}

const VS = `#version 300 es
void main() { vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2); gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0); }
`;

export class LBMGPU {
  /**
   * canvas: HTMLCanvasElement used for the context (and flow display)
   * c: { nx, ny, nz, mask }, p: same parameters as LBMCPU
   */
  constructor(canvas, c, p) {
    const gl = canvas.getContext('webgl2', { antialias: false, depth: false, stencil: false, alpha: false, preserveDrawingBuffer: false, powerPreference: 'high-performance' });
    if (!gl) throw new Error('WebGL2 is not available');
    if (!gl.getExtension('EXT_color_buffer_float')) throw new Error('This GPU cannot render to float textures');
    this.gl = gl; this.canvas = canvas;
    this.lost = false;
    canvas.addEventListener('webglcontextlost', (e) => { e.preventDefault(); this.lost = true; }, { once: true });
    this.nx = c.nx; this.ny = c.ny; this.nz = c.nz;
    this.p = { smag: 0.12, uInf: [0, 0, 0], omega: [0, 0, 0], ref: [0, 0, 0], spongeFrac: 0.2, spongeAmp: 0.5, ...p };
    const maxTex = gl.getParameter(gl.MAX_TEXTURE_SIZE);
    let tx = Math.max(1, Math.min(this.nz, Math.floor(maxTex / this.nx), Math.ceil(Math.sqrt((this.nz * this.ny) / this.nx))));
    let ty = Math.ceil(this.nz / tx);
    if (tx * this.nx > maxTex || ty * this.ny > maxTex) throw new Error(`Lattice too large for this GPU (max texture ${maxTex})`);
    this.TX = tx; this.AW = tx * this.nx; this.AH = ty * this.ny;
    this.steps = 0;
    this.geqInf = new Float32Array(Q);
    this.setInflow(this.p.uInf);

    // textures
    const mk = (internal, w, h, format, type, data = null) => {
      const t = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, t);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
      gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
      gl.texImage2D(gl.TEXTURE_2D, 0, internal, w, h, 0, format, type, data);
      return t;
    };
    const zeros = new Float32Array(this.AW * this.AH * 4);
    this.sets = [0, 1].map(() => Array.from({ length: 5 }, () => mk(gl.RGBA32F, this.AW, this.AH, gl.RGBA, gl.FLOAT, zeros)));
    // mask atlas: 255 solid, 128 fluid touching a solid, 0 fluid
    const mask = c.mask, nx = this.nx, ny = this.ny, nz = this.nz;
    const atlasMask = new Uint8Array(this.AW * this.AH);
    for (let z = 0; z < nz; z++) for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
      const id = x + nx * (y + ny * z);
      let v = 0;
      if (mask[id]) v = 255;
      else {
        for (let i = 1; i < Q && !v; i++) {
          const sx = x - C[i][0], sy = y - C[i][1], sz = z - C[i][2];
          if (sx >= 0 && sy >= 0 && sz >= 0 && sx < nx && sy < ny && sz < nz && mask[sx + nx * (sy + ny * sz)]) v = 128;
        }
      }
      atlasMask[(z % tx) * nx + x + this.AW * (Math.floor(z / tx) * ny + y)] = v;
    }
    this.maskTex = mk(gl.R8, this.AW, this.AH, gl.RED, gl.UNSIGNED_BYTE, atlasMask);
    // wall distances for interpolated bounce-back, same atlas layout
    this.interp = !!c.q;
    if (this.interp) {
      const qa = Array.from({ length: 5 }, () => new Uint8Array(this.AW * this.AH * 4));
      for (let z = 0; z < nz; z++) for (let y = 0; y < ny; y++) for (let x = 0; x < nx; x++) {
        const id = x + nx * (y + ny * z);
        if (atlasMask[(z % tx) * nx + x + this.AW * (Math.floor(z / tx) * ny + y)] !== 128) continue;
        const t = 4 * ((z % tx) * nx + x + this.AW * (Math.floor(z / tx) * ny + y));
        for (let d = 1; d < Q; d++) { const v = c.q[id * 19 + d]; if (v) qa[d >> 2][t + (d & 3)] = v; }
      }
      this.qTex = qa.map((data) => mk(gl.RGBA8, this.AW, this.AH, gl.RGBA, gl.UNSIGNED_BYTE, data));
    }

    // step programs (split the five outputs when MRT is limited)
    const maxDraw = gl.getParameter(gl.MAX_DRAW_BUFFERS);
    const groups = maxDraw >= 5 ? [[0, 1, 2, 3, 4]] : maxDraw >= 3 ? [[0, 1, 2], [3, 4]] : [[0, 1], [2, 3], [4]];
    this.stepPasses = groups.map((g) => ({ tex: g, prog: this.program(stepSource(g, this.interp)) }));
    this.fbs = [0, 1].map((k) => this.stepPasses.map((pass) => {
      const fb = gl.createFramebuffer();
      gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
      pass.tex.forEach((t, j) => gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + j, gl.TEXTURE_2D, this.sets[k][t], 0));
      const st = gl.checkFramebufferStatus(gl.FRAMEBUFFER);
      if (st !== gl.FRAMEBUFFER_COMPLETE) throw new Error(`Float framebuffer incomplete (${st})`);
      return fb;
    }));
    this.cur = 0;

    // force reduction
    this.FW = Math.ceil(this.AW / BLOCK); this.FH = Math.ceil(this.AH / BLOCK);
    this.forceProg = this.program(forceSource(this.interp));
    this.forceTex = [0, 1].map(() => mk(gl.RGBA32F, this.FW, this.FH, gl.RGBA, gl.FLOAT, null));
    this.forceFb = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.forceFb);
    this.forceTex.forEach((t, j) => gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0 + j, gl.TEXTURE_2D, t, 0));
    this.forceBytes = this.FW * this.FH * 16;
    this.pbo = gl.createBuffer();
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.bufferData(gl.PIXEL_PACK_BUFFER, this.forceBytes * 2, gl.STREAM_READ);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    this.pending = null;

    this.visProg = this.program(visSource());
    this.vao = gl.createVertexArray();
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  program(fs) {
    const gl = this.gl;
    const sh = (type, src) => {
      const s = gl.createShader(type);
      gl.shaderSource(s, src);
      gl.compileShader(s);
      if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error('Shader compile failed: ' + gl.getShaderInfoLog(s));
      return s;
    };
    const pr = gl.createProgram();
    gl.attachShader(pr, sh(gl.VERTEX_SHADER, VS));
    gl.attachShader(pr, sh(gl.FRAGMENT_SHADER, fs));
    gl.linkProgram(pr);
    if (!gl.getProgramParameter(pr, gl.LINK_STATUS)) throw new Error('Shader link failed: ' + gl.getProgramInfoLog(pr));
    const loc = {};
    const n = gl.getProgramParameter(pr, gl.ACTIVE_UNIFORMS);
    for (let i = 0; i < n; i++) { const u = gl.getActiveUniform(pr, i); loc[u.name.replace(/\[0\]$/, '')] = gl.getUniformLocation(pr, u.name); }
    return { pr, loc };
  }

  setInflow(u) {
    this.p.uInf = u.slice();
    const [ux, uy, uz] = u, uu = ux * ux + uy * uy + uz * uz;
    for (let i = 0; i < Q; i++) {
      const cu = C[i][0] * ux + C[i][1] * uy + C[i][2] * uz;
      this.geqInf[i] = W[i] * (3 * cu + 4.5 * cu * cu - 1.5 * uu);
    }
  }

  bindCommon(prog, set) {
    const gl = this.gl, L = prog.loc;
    gl.useProgram(prog.pr);
    for (let k = 0; k < 5; k++) {
      gl.activeTexture(gl.TEXTURE0 + k);
      gl.bindTexture(gl.TEXTURE_2D, this.sets[set][k]);
      if (L[`T${k}`]) gl.uniform1i(L[`T${k}`], k);
    }
    gl.activeTexture(gl.TEXTURE5);
    gl.bindTexture(gl.TEXTURE_2D, this.maskTex);
    if (L.S) gl.uniform1i(L.S, 5);
    if (this.interp && L.Q0) for (let k = 0; k < 5; k++) {
      gl.activeTexture(gl.TEXTURE6 + k);
      gl.bindTexture(gl.TEXTURE_2D, this.qTex[k]);
      if (L[`Q${k}`]) gl.uniform1i(L[`Q${k}`], 6 + k);
    }
    if (L.N) gl.uniform3i(L.N, this.nx, this.ny, this.nz);
    if (L.TX) gl.uniform1i(L.TX, this.TX);
    if (L.omega) gl.uniform3fv(L.omega, this.p.omega);
    if (L.refp) gl.uniform3fv(L.refp, this.p.ref);
  }

  step(n = 1) {
    if (this.lost) throw new Error('The GPU context was lost');
    const gl = this.gl;
    const x0 = Math.floor(this.nx * (1 - this.p.spongeFrac));
    gl.bindVertexArray(this.vao);
    gl.viewport(0, 0, this.AW, this.AH);
    for (let k = 0; k < n; k++) {
      const next = 1 - this.cur;
      this.stepPasses.forEach((pass, pi) => {
        this.bindCommon(pass.prog, this.cur);
        const L = pass.prog.loc;
        gl.uniform1f(L.tau0, this.p.tau0);
        gl.uniform1f(L.smagK, SMAG_K * this.p.smag * this.p.smag);
        gl.uniform1f(L.spongeStart, x0);
        gl.uniform1f(L.spongeAmp, this.p.spongeAmp);
        gl.uniform1fv(L.geqInf, this.geqInf);
        gl.bindFramebuffer(gl.FRAMEBUFFER, this.fbs[next][pi]);
        gl.drawBuffers(pass.tex.map((_, j) => gl.COLOR_ATTACHMENT0 + j));
        gl.drawArrays(gl.TRIANGLES, 0, 3);
      });
      this.cur = next;
      this.steps++;
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
  }

  // Momentum exchange on the current state; reduced on the GPU.
  forcePass() {
    const gl = this.gl;
    gl.bindVertexArray(this.vao);
    this.bindCommon(this.forceProg, this.cur);
    gl.uniform2i(this.forceProg.loc.AT, this.AW, this.AH);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.forceFb);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0, gl.COLOR_ATTACHMENT1]);
    gl.viewport(0, 0, this.FW, this.FH);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }
  reduce(buf) {
    const n = this.FW * this.FH;
    const F = [0, 0, 0], M = [0, 0, 0];
    for (let i = 0; i < n; i++) { F[0] += buf[4 * i]; F[1] += buf[4 * i + 1]; F[2] += buf[4 * i + 2]; }
    for (let i = 0; i < n; i++) { M[0] += buf[4 * n + 4 * i]; M[1] += buf[4 * n + 4 * i + 1]; M[2] += buf[4 * n + 4 * i + 2]; }
    return { F, M };
  }
  // synchronous (stalls the pipeline): tests and the final sample
  force() {
    const gl = this.gl;
    this.forcePass();
    const buf = new Float32Array(this.FW * this.FH * 8);
    gl.readBuffer(gl.COLOR_ATTACHMENT0);
    gl.readPixels(0, 0, this.FW, this.FH, gl.RGBA, gl.FLOAT, buf, 0);
    gl.readBuffer(gl.COLOR_ATTACHMENT1);
    gl.readPixels(0, 0, this.FW, this.FH, gl.RGBA, gl.FLOAT, buf, this.FW * this.FH * 4);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    return this.reduce(buf);
  }
  // asynchronous: request now, poll later (no pipeline stall)
  requestForce() {
    if (this.pending) return false;
    const gl = this.gl;
    this.forcePass();
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.readBuffer(gl.COLOR_ATTACHMENT0);
    gl.readPixels(0, 0, this.FW, this.FH, gl.RGBA, gl.FLOAT, 0);
    gl.readBuffer(gl.COLOR_ATTACHMENT1);
    gl.readPixels(0, 0, this.FW, this.FH, gl.RGBA, gl.FLOAT, this.forceBytes);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    this.pending = { sync: gl.fenceSync(gl.SYNC_GPU_COMMANDS_COMPLETE, 0), step: this.steps, t0: performance.now() };
    gl.flush();
    return true;
  }
  pollForce() {
    const pd = this.pending;
    if (!pd) return null;
    const gl = this.gl;
    const st = gl.clientWaitSync(pd.sync, 0, 0);
    if (st !== gl.ALREADY_SIGNALED && st !== gl.CONDITION_SATISFIED) return null;
    gl.deleteSync(pd.sync);
    const buf = new Float32Array(this.FW * this.FH * 8);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, this.pbo);
    gl.getBufferSubData(gl.PIXEL_PACK_BUFFER, 0, buf);
    gl.bindBuffer(gl.PIXEL_PACK_BUFFER, null);
    this.pending = null;
    return { ...this.reduce(buf), step: pd.step, t: performance.now() };
  }

  // draw a slice of the flow into the canvas
  render({ axis = 'y', index, field = 'speed', uref = 0.05 } = {}) {
    const gl = this.gl, cv = this.canvas;
    const ax = axis === 'y' ? 0 : axis === 'z' ? 1 : 2;
    const dims = ax === 0 ? [this.nx, this.nz] : ax === 1 ? [this.nx, this.ny] : [this.ny, this.nz];
    const lim = ax === 0 ? this.ny : ax === 1 ? this.nz : this.nx;
    const idx = Math.max(0, Math.min(lim - 1, Math.round(index ?? lim / 2)));
    const s = Math.min(cv.width / dims[0], cv.height / dims[1]);
    const w = dims[0] * s, h = dims[1] * s;
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.bindVertexArray(this.vao);
    gl.viewport(0, 0, cv.width, cv.height);
    this.bindCommon(this.visProg, this.cur);
    const L = this.visProg.loc;
    gl.uniform1i(L.axis, ax); gl.uniform1i(L.idx, idx);
    gl.uniform1i(L.field, field === 'speed' ? 0 : field === 'pressure' ? 2 : 1);
    gl.uniform1f(L.uref, uref);
    gl.uniform4f(L.view, (cv.width - w) / 2, (cv.height - h) / 2, w, h);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
  }

  // Macroscopic field (ux, uy, uz, rho per cell, lattice units) for the 3D flow
  // view, optionally sub-sampled every `stride` cells. Solid cells give NaN.
  snapshot(stride = 1) {
    const gl = this.gl;
    if (!this.macroProg) {
      this.macroProg = this.program(HEADER + `out vec4 o;
void main() {
  ivec2 fc = ivec2(gl_FragCoord.xy);
  ivec3 p = cellOf(fc);
  if (p.z >= N.z || maskAt(fc) > 0.75) { o = vec4(0.0, 0.0, 0.0, -1.0); return; }
  vec4 a0 = texelFetch(T0, fc, 0), a1 = texelFetch(T1, fc, 0), a2 = texelFetch(T2, fc, 0), a3 = texelFetch(T3, fc, 0), a4 = texelFetch(T4, fc, 0);
  float g[19];
  g[0] = a0.x; g[1] = a0.y; g[2] = a0.z; g[3] = a0.w; g[4] = a1.x; g[5] = a1.y; g[6] = a1.z; g[7] = a1.w; g[8] = a2.x; g[9] = a2.y;
  g[10] = a2.z; g[11] = a2.w; g[12] = a3.x; g[13] = a3.y; g[14] = a3.z; g[15] = a3.w; g[16] = a4.x; g[17] = a4.y; g[18] = a4.z;
  float rho = 1.0; vec3 j = vec3(0.0);
${C.map((c, i) => `  rho += g[${i}]; j += ${cvec(i)} * g[${i}];`).join('\n')}
  o = vec4(j / rho, rho);
}
`);
      this.macroTex = null;
    }
    if (!this.macroTex) {
      this.macroTex = gl.createTexture();
      gl.bindTexture(gl.TEXTURE_2D, this.macroTex);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA32F, this.AW, this.AH, 0, gl.RGBA, gl.FLOAT, null);
      this.macroFb = gl.createFramebuffer();
      gl.bindFramebuffer(gl.FRAMEBUFFER, this.macroFb);
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.macroTex, 0);
    }
    gl.bindVertexArray(this.vao);
    this.bindCommon(this.macroProg, this.cur);
    gl.bindFramebuffer(gl.FRAMEBUFFER, this.macroFb);
    gl.drawBuffers([gl.COLOR_ATTACHMENT0]);
    gl.viewport(0, 0, this.AW, this.AH);
    gl.drawArrays(gl.TRIANGLES, 0, 3);
    const buf = new Float32Array(this.AW * this.AH * 4);
    gl.readBuffer(gl.COLOR_ATTACHMENT0);
    gl.readPixels(0, 0, this.AW, this.AH, gl.RGBA, gl.FLOAT, buf);
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    const { nx, ny, nz, TX } = this;
    const sx = Math.ceil(nx / stride), sy = Math.ceil(ny / stride), sz = Math.ceil(nz / stride);
    const data = new Float32Array(sx * sy * sz * 4);
    for (let k = 0; k < sz; k++) for (let j = 0; j < sy; j++) for (let i = 0; i < sx; i++) {
      const x = i * stride, y = j * stride, z = k * stride;
      const t = 4 * ((z % TX) * nx + x + this.AW * (Math.floor(z / TX) * ny + y));
      const o = 4 * (i + sx * (j + sy * k));
      if (buf[t + 3] < 0) { data[o] = data[o + 1] = data[o + 2] = 0; data[o + 3] = NaN; continue; }
      data[o] = buf[t]; data[o + 1] = buf[t + 1]; data[o + 2] = buf[t + 2]; data[o + 3] = buf[t + 3];
    }
    return { nx: sx, ny: sy, nz: sz, stride, data };
  }

  // read back the full state (tests / debugging)
  readState() {
    const gl = this.gl;
    const out = [];
    const fb = gl.createFramebuffer();
    gl.bindFramebuffer(gl.FRAMEBUFFER, fb);
    for (let k = 0; k < 5; k++) {
      gl.framebufferTexture2D(gl.FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, this.sets[this.cur][k], 0);
      const buf = new Float32Array(this.AW * this.AH * 4);
      gl.readPixels(0, 0, this.AW, this.AH, gl.RGBA, gl.FLOAT, buf);
      out.push(buf);
    }
    gl.bindFramebuffer(gl.FRAMEBUFFER, null);
    gl.deleteFramebuffer(fb);
    return out;
  }

  dispose() {
    const gl = this.gl;
    try {
      this.sets.flat().forEach((t) => gl.deleteTexture(t));
      this.forceTex.forEach((t) => gl.deleteTexture(t));
      gl.deleteTexture(this.maskTex);
      this.qTex?.forEach((t) => gl.deleteTexture(t));
      if (this.macroTex) gl.deleteTexture(this.macroTex);
      gl.deleteBuffer(this.pbo);
    } catch { /* context may be gone */ }
  }
}
