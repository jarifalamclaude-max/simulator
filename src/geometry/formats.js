// Mesh file formats for the vehicle design: STL (binary/ASCII), OBJ and PLY
// (ASCII and binary). Everything is turned into a flat triangle soup
// { positions: Float32Array (x,y,z per vertex, 3 vertices per triangle), triCount, format, name },
// plus `parts` [{name, start, count}] (triangle ranges) when the file names its
// bodies (OBJ o/g, multi-solid ASCII STL; PLY has no standard part names).
// Pure JS: runs in the page, in workers and in Node tests.

import { parseSTL } from './stl.js';

export const DESIGN_EXTENSIONS = ['.stl', '.obj', '.ply'];
export const DESIGN_ACCEPT = '.stl,.obj,.ply,model/stl,model/obj,application/sla';

export function parseMesh(buffer, filename = '') {
  const ext = (filename.split('.').pop() || '').toLowerCase();
  const bytes = buffer instanceof ArrayBuffer ? new Uint8Array(buffer) : new Uint8Array(buffer.buffer, buffer.byteOffset, buffer.byteLength);
  const head = new TextDecoder('ascii').decode(bytes.subarray(0, Math.min(256, bytes.length)));
  if (ext === 'ply' || /^ply\s/.test(head)) return parsePLY(bytes);
  if (ext === 'obj' || (ext !== 'stl' && /^\s*(#|v\s|o\s|g\s|mtllib)/m.test(head) && !/^\s*solid/.test(head))) return parseOBJ(new TextDecoder('utf-8').decode(bytes));
  return parseSTL(buffer);
}

// Wavefront OBJ: v / f records (faces may be polygons, indices may be negative
// or carry /vt/vn parts); materials are ignored. Object (o) and group (g)
// names are kept as triangle ranges in `parts` [{name: 'object/group', start, count}].
export function parseOBJ(text) {
  const verts = [];
  const tris = [];
  let name = 'OBJ mesh';
  let obj = '', grp = '', cur = '', curStart = 0;
  const ranges = [];
  const closePart = () => { const n = tris.length / 3 - curStart; if (n > 0) ranges.push({ name: cur, start: curStart, count: n }); };
  const lines = text.split(/\r?\n/);
  for (let ln = 0; ln < lines.length; ln++) {
    let line = lines[ln];
    while (line.endsWith('\\') && ln + 1 < lines.length) line = line.slice(0, -1) + ' ' + lines[++ln];
    const s = line.trim();
    if (!s || s[0] === '#') continue;
    const parts = s.split(/\s+/);
    const tag = parts[0];
    if (tag === 'v') {
      verts.push(+parts[1], +parts[2], +parts[3]);
    } else if (tag === 'f') {
      const n = verts.length / 3;
      const idx = [];
      for (let k = 1; k < parts.length; k++) {
        const v = parseInt(parts[k].split('/')[0], 10);
        if (!Number.isFinite(v) || v === 0) continue;
        idx.push(v > 0 ? v - 1 : n + v);
      }
      for (let k = 1; k + 1 < idx.length; k++) tris.push(idx[0], idx[k], idx[k + 1]);
    } else if (tag === 'o' || tag === 'g') {
      const label = parts.slice(1).join(' ');
      if (tag === 'o') { obj = label; grp = ''; if (parts[1]) name = label; }
      else grp = label === 'default' ? '' : label;
      const nm = [obj, grp].filter(Boolean).join('/');
      if (nm !== cur) { closePart(); cur = nm; curStart = tris.length / 3; }
    }
  }
  closePart();
  const out = soup(verts, tris, 'obj', name);
  if (ranges.some((r) => r.name)) out.parts = ranges;
  return out;
}

// Stanford PLY: ascii, binary_little_endian, binary_big_endian; vertex x/y/z
// and face vertex_indices (any list/count types); other properties skipped.
export function parsePLY(bytes) {
  const dec = new TextDecoder('ascii');
  // header ends with "end_header\n"
  let end = -1;
  const probe = dec.decode(bytes.subarray(0, Math.min(bytes.length, 64 * 1024)));
  const m = /end_header\r?\n/.exec(probe);
  if (!m) throw new Error('PLY header not found');
  end = m.index + m[0].length;
  const header = probe.slice(0, m.index).split(/\r?\n/);
  let format = 'ascii';
  const elements = [];
  for (const raw of header) {
    const p = raw.trim().split(/\s+/);
    if (p[0] === 'format') format = p[1];
    else if (p[0] === 'element') elements.push({ name: p[1], count: +p[2], props: [] });
    else if (p[0] === 'property' && elements.length) {
      const el = elements[elements.length - 1];
      if (p[1] === 'list') el.props.push({ name: p[4], list: true, countType: p[2], type: p[3] });
      else el.props.push({ name: p[2], type: p[1] });
    }
  }
  const verts = [], tris = [];
  const vEl = elements.find((e) => e.name === 'vertex');
  if (!vEl) throw new Error('PLY file has no vertices');
  const xi = vEl.props.findIndex((p) => p.name === 'x'), yi = vEl.props.findIndex((p) => p.name === 'y'), zi = vEl.props.findIndex((p) => p.name === 'z');
  if (xi < 0 || yi < 0 || zi < 0) throw new Error('PLY vertices have no x/y/z');
  const addFace = (idx) => { for (let k = 1; k + 1 < idx.length; k++) tris.push(idx[0], idx[k], idx[k + 1]); };
  if (format === 'ascii') {
    const tokens = dec.decode(bytes.subarray(end)).split(/\s+/).filter(Boolean);
    let t = 0;
    for (const el of elements) {
      for (let r = 0; r < el.count; r++) {
        const vals = [];
        let face = null;
        for (const p of el.props) {
          if (p.list) {
            const n = +tokens[t++];
            const list = [];
            for (let k = 0; k < n; k++) list.push(+tokens[t++]);
            if (p.name === 'vertex_indices' || p.name === 'vertex_index') face = list;
          } else vals.push(+tokens[t++]);
        }
        if (el === vEl) verts.push(vals[xi], vals[yi], vals[zi]);
        else if (el.name === 'face' && face) addFace(face);
      }
    }
  } else {
    const little = format === 'binary_little_endian';
    if (!little && format !== 'binary_big_endian') throw new Error(`Unsupported PLY format ${format}`);
    const dv = new DataView(bytes.buffer, bytes.byteOffset + end, bytes.byteLength - end);
    let o = 0;
    const SIZE = { char: 1, int8: 1, uchar: 1, uint8: 1, short: 2, int16: 2, ushort: 2, uint16: 2, int: 4, int32: 4, uint: 4, uint32: 4, float: 4, float32: 4, double: 8, float64: 8 };
    const read = (type) => {
      let v;
      switch (type) {
        case 'char': case 'int8': v = dv.getInt8(o); break;
        case 'uchar': case 'uint8': v = dv.getUint8(o); break;
        case 'short': case 'int16': v = dv.getInt16(o, little); break;
        case 'ushort': case 'uint16': v = dv.getUint16(o, little); break;
        case 'int': case 'int32': v = dv.getInt32(o, little); break;
        case 'uint': case 'uint32': v = dv.getUint32(o, little); break;
        case 'float': case 'float32': v = dv.getFloat32(o, little); break;
        case 'double': case 'float64': v = dv.getFloat64(o, little); break;
        default: throw new Error(`Unknown PLY type ${type}`);
      }
      o += SIZE[type];
      return v;
    };
    for (const el of elements) {
      for (let r = 0; r < el.count; r++) {
        const vals = [];
        let face = null;
        for (const p of el.props) {
          if (p.list) {
            const n = read(p.countType);
            const list = new Array(n);
            for (let k = 0; k < n; k++) list[k] = read(p.type);
            if (p.name === 'vertex_indices' || p.name === 'vertex_index') face = list;
          } else vals.push(read(p.type));
        }
        if (el === vEl) verts.push(vals[xi], vals[yi], vals[zi]);
        else if (el.name === 'face' && face) addFace(face);
      }
    }
  }
  return soup(verts, tris, `ply-${format}`, 'PLY mesh');
}

function soup(verts, tris, format, name) {
  const triCount = tris.length / 3;
  if (!triCount) throw new Error('The file contains no faces');
  const nv = verts.length / 3;
  const positions = new Float32Array(triCount * 9);
  let bad = 0;
  for (let t = 0; t < tris.length; t++) {
    const i = tris[t];
    if (i < 0 || i >= nv) { bad++; continue; }
    positions[3 * t] = verts[3 * i]; positions[3 * t + 1] = verts[3 * i + 1]; positions[3 * t + 2] = verts[3 * i + 2];
  }
  if (bad) throw new Error(`${bad} face indices point outside the vertex list`);
  return { positions, triCount, format, name, invalidValues: 0 };
}
