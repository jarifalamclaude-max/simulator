// STL reading/writing (binary and ASCII). Pure JS, no DOM, so it runs in
// workers and in Node tests.

export function parseSTL(buffer) {
  const bytes = buffer instanceof ArrayBuffer ? new Uint8Array(buffer) : new Uint8Array(buffer.buffer, buffer.byteOffset, buffer.byteLength);
  if (bytes.byteLength < 15) throw new Error('File is too small to be an STL');
  const dv = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
  // Binary STL: 80-byte header + uint32 count + 50 bytes per triangle.
  if (bytes.byteLength >= 84) {
    const n = dv.getUint32(80, true);
    if (84 + n * 50 === bytes.byteLength && n > 0) {
      return parseBinary(dv, n, headerText(bytes));
    }
  }
  // ASCII
  const text = new TextDecoder('utf-8', { fatal: false }).decode(bytes);
  if (/^\s*solid/i.test(text) && /facet/i.test(text)) return parseASCII(text);
  // Some exporters write binary files whose size does not match the count.
  if (bytes.byteLength >= 84) {
    const n = Math.floor((bytes.byteLength - 84) / 50);
    if (n > 0) return parseBinary(dv, n, headerText(bytes));
  }
  throw new Error('Not a recognised STL file (neither binary nor ASCII)');
}

function headerText(bytes) {
  let s = '';
  for (let i = 0; i < 80; i++) {
    const c = bytes[i];
    if (c === 0) break;
    s += c >= 32 && c < 127 ? String.fromCharCode(c) : '';
  }
  return s.trim();
}

function parseBinary(dv, n, header) {
  const positions = new Float32Array(n * 9);
  let o = 84, p = 0, bad = 0;
  for (let t = 0; t < n; t++) {
    o += 12; // skip facet normal (we recompute)
    for (let k = 0; k < 9; k++) {
      const v = dv.getFloat32(o, true);
      if (!Number.isFinite(v)) bad++;
      positions[p++] = Number.isFinite(v) ? v : 0;
      o += 4;
    }
    o += 2;
  }
  return { positions, triCount: n, format: 'binary', name: header || 'binary STL', invalidValues: bad };
}

// ASCII files may hold several `solid name … endsolid` blocks (one per CAD
// body): their triangle ranges are returned as `parts` [{name, start, count}].
function parseASCII(text) {
  const re = /vertex\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)\s+([-+0-9.eE]+)/g;
  const solids = [];
  const sre = /^[ \t]*solid\b[ \t]*([^\r\n]*)/gim;
  let s;
  while ((s = sre.exec(text))) solids.push({ at: s.index, name: s[1].trim(), verts: 0 });
  const out = [];
  let m, si = -1, lead = 0;
  while ((m = re.exec(text))) {
    while (si + 1 < solids.length && solids[si + 1].at <= m.index) si++;
    out.push(+m[1], +m[2], +m[3]);
    if (si >= 0) solids[si].verts++;
    else lead++;
  }
  const triCount = Math.floor(out.length / 9);
  if (triCount === 0) throw new Error('ASCII STL contains no facets');
  const positions = new Float32Array(out.slice(0, triCount * 9));
  const nm = /^\s*solid\s+([^\r\n]*)/i.exec(text);
  const res = { positions, triCount, format: 'ascii', name: (nm && nm[1].trim()) || 'ASCII STL', invalidValues: 0 };
  if (solids.some((p) => p.name)) {
    const parts = [];
    let v = lead;
    for (const p of solids) {
      const start = Math.min(triCount, Math.floor(v / 3)), end = Math.min(triCount, Math.floor((v + p.verts) / 3));
      if (end > start) parts.push({ name: p.name, start, count: end - start });
      v += p.verts;
    }
    if (parts.length) res.parts = parts;
  }
  return res;
}

export function writeBinarySTL(positions, header = 'DUBURI ROV simulator') {
  const n = positions.length / 9;
  const buf = new ArrayBuffer(84 + n * 50);
  const dv = new DataView(buf);
  for (let i = 0; i < Math.min(80, header.length); i++) dv.setUint8(i, header.charCodeAt(i) & 0x7f);
  dv.setUint32(80, n, true);
  let o = 84;
  for (let t = 0; t < n; t++) {
    const b = t * 9;
    const ax = positions[b + 3] - positions[b], ay = positions[b + 4] - positions[b + 1], az = positions[b + 5] - positions[b + 2];
    const bx = positions[b + 6] - positions[b], by = positions[b + 7] - positions[b + 1], bz = positions[b + 8] - positions[b + 2];
    let nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
    const l = Math.hypot(nx, ny, nz) || 1;
    dv.setFloat32(o, nx / l, true); dv.setFloat32(o + 4, ny / l, true); dv.setFloat32(o + 8, nz / l, true);
    o += 12;
    for (let k = 0; k < 9; k++) { dv.setFloat32(o, positions[b + k], true); o += 4; }
    dv.setUint16(o, 0, true);
    o += 2;
  }
  return buf;
}
