// Thruster detection on uploaded designs: part names, duct shape, ducts fused
// into the hull, and direction / motor order from the ArduSub frame tables.
import test from 'node:test';
import assert from 'node:assert/strict';
import { box, tube, cylinder } from '../src/geometry/primitives.js';
import { parseOBJ, parseMesh } from '../src/geometry/formats.js';
import { parseSTL, writeBinarySTL } from '../src/geometry/stl.js';
import { makeModelTransform, transformPositions } from '../src/geometry/mesh.js';
import { detectThrusters, thrusterNameInfo, resolveDirections, classifyDuct, directionWord, eigSym3, hungarian } from '../src/geometry/thrusterDetect.js';
import { heavyLayout, vectoredLayout } from '../src/alloc/frames.js';
import { analyzeLayout } from '../src/alloc/allocation.js';
import { sampleGeometry } from '../src/vehicle/sample.js';

const MM = 0.005; // position tolerance (5 mm)
const near = (a, b, tol, msg) => assert.ok(Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) <= tol, `${msg}: ${a} vs ${b}`);
const parallel = (a, b) => Math.abs(a[0] * b[0] + a[1] * b[1] + a[2] * b[2]);
const same = (a, b, msg) => assert.ok(a[0] * b[0] + a[1] * b[1] + a[2] * b[2] > 0.999, `${msg}: ${a} vs ${b}`);

// a box hull and a T200-sized duct (OD 100, ID 88 mm) with a motor at every thruster
function rov(layout, { seg = 24 } = {}) {
  const parts = [{ name: 'Frame', tris: box([], [0, 0, 0], [0.22, 0.12, 0.12]) }];
  layout.forEach((t, i) => {
    parts.push({ name: `T200_${i + 1}/Nozzle`, tris: tube([], t.pos, t.axis, 0.05, 0.044, 0.09, seg) });
    parts.push({ name: `T200_${i + 1}/Motor`, tris: cylinder([], t.pos, t.axis, 0.021, 0.06, 16) });
  });
  return parts;
}
const soup = (parts) => new Float32Array(parts.flatMap((p) => p.tris));

// OBJ as a CAD tool writes it: millimetres, Z up, X forward (so Y is to port),
// one object per body, shared vertices
function objText(parts, nameOf = (p) => p.name) {
  const lines = ['# test export'];
  let base = 1;
  for (const p of parts) {
    const [o, g] = nameOf(p).split('/');
    lines.push(`o ${o}`);
    if (g) lines.push(`g ${g}`);
    const map = new Map(), vs = [], idx = [];
    for (let i = 0; i < p.tris.length; i += 3) {
      const v = [p.tris[i] * 1000, -p.tris[i + 1] * 1000, -p.tris[i + 2] * 1000].map((x) => +x.toFixed(4));
      const k = v.join(' ');
      if (!map.has(k)) { map.set(k, vs.length); vs.push(k); }
      idx.push(map.get(k));
    }
    vs.forEach((k) => lines.push(`v ${k}`));
    for (let i = 0; i < idx.length; i += 3) lines.push(`f ${idx[i] + base} ${idx[i + 1] + base} ${idx[i + 2] + base}`);
    base += vs.length;
  }
  return lines.join('\n');
}
// what the upload flow does: units and axes chosen in Vehicle, origin = file origin
const toVehicle = (parsed, units = 'mm') => transformPositions(parsed.positions, makeModelTransform({ units, forward: '+X', up: '+Z' }));

function asciiSTL(parts) {
  const out = [];
  for (const p of parts) {
    out.push(`solid ${p.name}`);
    for (let i = 0; i < p.tris.length; i += 9) {
      out.push('  facet normal 0 0 0', '    outer loop');
      for (let k = 0; k < 3; k++) out.push(`      vertex ${p.tris[i + 3 * k]} ${p.tris[i + 3 * k + 1]} ${p.tris[i + 3 * k + 2]}`);
      out.push('    endloop', '  endfacet');
    }
    out.push(`endsolid ${p.name}`);
  }
  return new TextEncoder().encode(out.join('\n'));
}

function checkHeavy(r, msg) {
  const H = heavyLayout();
  assert.equal(r.thrusters.length, 8, `${msg}: ${r.summary}`);
  assert.equal(r.frame?.config, 2, `${msg}: ${r.summary}`);
  r.thrusters.forEach((t, i) => {
    assert.equal(t.motor, i + 1);
    near(t.pos, H[i].pos, MM, `${msg} motor ${i + 1} position`);
    same(t.axis, H[i].axis, `${msg} motor ${i + 1} direction`);
    assert.equal(t.type, 't200', `${msg} motor ${i + 1}`);
    assert.ok(Math.abs(t.od - 0.1) < 0.002 && Math.abs(t.id - 0.088) < 0.002, `${msg} motor ${i + 1} OD ${t.od} ID ${t.id}`);
  });
}

// ---------------------------------------------------------------- parsers

test('OBJ and multi-solid ASCII STL keep their part names as triangle ranges', () => {
  const parts = rov(heavyLayout().slice(0, 2));
  const obj = parseOBJ(objText(parts));
  assert.equal(obj.triCount, soup(parts).length / 9);
  assert.equal(obj.format, 'obj');
  assert.equal(obj.name, 'T200_2'); // last object name, as before
  assert.deepEqual(obj.parts.map((p) => p.name), ['Frame', 'T200_1/Nozzle', 'T200_1/Motor', 'T200_2/Nozzle', 'T200_2/Motor']);
  let start = 0;
  obj.parts.forEach((p, i) => { assert.equal(p.start, start); assert.equal(p.count, parts[i].tris.length / 9); start += p.count; });
  const stl = parseSTL(asciiSTL(parts));
  assert.equal(stl.format, 'ascii');
  assert.equal(stl.name, 'Frame');
  assert.deepEqual(stl.parts.map((p) => [p.name, p.start, p.count]), obj.parts.map((p) => [p.name, p.start, p.count]));
  // binary STL has no names; the old return shape is untouched
  const bin = parseSTL(writeBinarySTL(soup(parts)));
  assert.equal(bin.parts, undefined);
  assert.deepEqual(Object.keys(bin).sort(), ['format', 'invalidValues', 'name', 'positions', 'triCount']);
  assert.ok(parseMesh(new TextEncoder().encode(objText(parts)), 'rov.obj').parts.length === 5);
});

test('thruster part names give the model and the motor number', () => {
  const cases = [
    ['T200_3', { thruster: true, type: 't200', number: 3 }],
    ['Thruster 5', { thruster: true, type: null, number: 5 }],
    ['LeftMotor2', { thruster: true, type: null, number: 2 }],
    ['Assembly 1/T500 (4)/Nozzle', { thruster: true, type: 't500', number: 4 }],
    ['ROV 2/Thruster', { thruster: true, type: null, number: null }],
    ['prop', { thruster: true, type: null, number: null }],
    ['Product1', { thruster: false, type: null, number: null }],
    ['Motor_Mount_1', { thruster: false, type: null, number: null }],
    ['Frame plate', { thruster: false, type: null, number: null }],
  ];
  for (const [name, want] of cases) assert.deepEqual(thrusterNameInfo(name), want, name);
});

// ---------------------------------------------------------------- detection

test('8 named T200s in an OBJ (mm, Z up) are found and match FRAME_CONFIG 2', () => {
  const parsed = parseOBJ(objText(rov(heavyLayout())));
  const r = detectThrusters(toVehicle(parsed), { parts: parsed.parts, frameConfig: 2 });
  assert.equal(r.method, 'names');
  assert.match(r.summary, /^8 × T200 detected \(part names\) — matched FRAME_CONFIG 2 Vectored 6DOF/);
  checkHeavy(r, 'names');
  r.thrusters.forEach((t, i) => { assert.equal(t.number, i + 1); assert.equal(t.source, 'name'); assert.equal(t.name, `T200_${i + 1}`); });
  // the resulting custom frame is fully controllable
  const a = analyzeLayout(r.thrusters.map((t) => ({ ...t, fwdMax: 51.5, revMax: 40.2 })), [0, 0, 0]);
  assert.equal(a.controllable.length, 6, a.summary);
  // all thrusters in one group ("Thrusters"): split into separate thrusters, numbered by the frame
  const one = parseOBJ(objText(rov(heavyLayout()), (p) => (p.name === 'Frame' ? 'Frame' : 'Thrusters')));
  assert.deepEqual(one.parts.map((p) => p.name), ['Frame', 'Thrusters']);
  const r1 = detectThrusters(toVehicle(one), { parts: one.parts, frameConfig: 2 });
  assert.equal(r1.method, 'names');
  checkHeavy(r1, 'one group');
});

test('without names the same design is found by the duct shape', () => {
  const parts = rov(heavyLayout());
  const r = detectThrusters(soup(parts), { frameConfig: 2 });
  assert.equal(r.method, 'shape');
  assert.match(r.summary, /8 × T200 detected by shape — matched FRAME_CONFIG 2/);
  checkHeavy(r, 'shape');
  // same result after a binary STL round trip in inches, Y up (forward +X)
  const inch = new Float32Array(soup(parts).length);
  const src = soup(parts);
  for (let i = 0; i < src.length; i += 3) { inch[i] = src[i] / 0.0254; inch[i + 1] = -src[i + 2] / 0.0254; inch[i + 2] = src[i + 1] / 0.0254; }
  const stl = parseSTL(writeBinarySTL(inch));
  const veh = transformPositions(stl.positions, makeModelTransform({ units: 'in', forward: '+X', up: '+Y' }));
  checkHeavy(detectThrusters(veh, { frameConfig: 2 }), 'inches, Y up');
  // the built-in sample ROV (frame plates, tubes and mounting plates around the ducts)
  checkHeavy(detectThrusters(sampleGeometry().positions, { frameConfig: 2, cg: [0, 0, 0.02] }), 'sample ROV');
});

test('ducts fused into the hull (one connected mesh) are found by ring detection', () => {
  const o = box([], [0, 0, 0], [0.22, 0.12, 0.12]);
  const hull = [o.slice(0, 3), o.slice(3, 6)];
  for (const t of heavyLayout()) {
    const s = o.length;
    tube(o, t.pos, t.axis, 0.05, 0.044, 0.09, 24);
    o.push(...o.slice(s, s + 3), ...hull[0], ...hull[1]); // a sliver joining duct and hull
  }
  const r = detectThrusters(new Float32Array(o), { frameConfig: 2 });
  assert.equal(r.method, 'hull');
  checkHeavy(r, 'fused');
});

test('pipes, solid rods, pucks and housings are not thrusters', () => {
  const o = box([], [0, 0, 0], [0.4, 0.3, 0.2]);
  tube(o, [0, 0.25, 0], [1, 0, 0], 0.05, 0.044, 0.4, 24); // long pipe
  cylinder(o, [0, -0.25, 0], [1, 0, 0], 0.05, 0.3, 24); // solid rod
  cylinder(o, [0.3, 0, 0], [0, 0, 1], 0.05, 0.03, 24); // solid puck of T200 size
  cylinder(o, [-0.3, 0, 0], [1, 0, 0], 0.057, 0.3, 40); // 4" electronics tube
  const r = detectThrusters(new Float32Array(o));
  assert.equal(r.thrusters.length, 0, r.summary);
  assert.equal(r.method, 'none');
  assert.equal(classifyDuct({ od: 0.1, id: 0.088, ductLength: 0.09, length: 0.11 }).type, 't200');
  assert.equal(classifyDuct({ od: 0.14, id: 0.112, ductLength: 0.1 }).type, 't500');
  assert.equal(classifyDuct({ od: 0.06, id: 0.052, ductLength: 0.05 }).type, 'generic');
  assert.equal(classifyDuct({ od: 0.1, id: null, ductLength: 0.05 }).ok, false);
});

test('T500 and a tilted generic thruster are measured on any axis', () => {
  const o = box([], [0, 0, 0], [0.3, 0.2, 0.15]);
  tube(o, [0.3, 0, 0], [1, 0, 0], 0.07, 0.056, 0.1, 32);
  const ax = [0.30305, -0.50508, 0.80812];
  tube(o, [-0.25, 0.1, 0.05], ax, 0.03, 0.026, 0.05, 12); // coarse: 12 segments
  const r = detectThrusters(new Float32Array(o));
  assert.equal(r.thrusters.length, 2, r.summary);
  const t500 = r.thrusters.find((t) => t.type === 't500'), gen = r.thrusters.find((t) => t.type === 'generic');
  near(t500.pos, [0.3, 0, 0], MM, 'T500');
  assert.ok(parallel(t500.axis, [1, 0, 0]) > 0.999);
  assert.equal(t500.preset, 't500-20v');
  near(gen.pos, [-0.25, 0.1, 0.05], MM, 'generic');
  assert.ok(parallel(gen.axis, ax) > 0.999, `${gen.axis}`);
  assert.ok(Math.abs(gen.od - 0.06) < 0.002 && gen.propDiameter > 0.04 && gen.propDiameter < 0.052, `${gen.od} ${gen.propDiameter}`);
  // no 2-motor ArduSub frame: default directions, flagged
  assert.equal(r.directions, 'default');
  assert.match(r.summary, /check directions$/);
  assert.ok(r.thrusters.every((t) => t.directionConfidence < 0.5));
});

test('a BlueROV2 (6 thrusters) matches FRAME_CONFIG 1 even when FRAME_CONFIG is 2', () => {
  const r = detectThrusters(soup(rov(vectoredLayout())).slice(0), { frameConfig: 2 });
  assert.equal(r.frame?.config, 1, r.summary);
  assert.ok(r.notes.some((n) => /FRAME_CONFIG is 2/.test(n)));
  const V = vectoredLayout();
  r.thrusters.forEach((t, i) => { near(t.pos, V[i].pos, MM, `motor ${i + 1}`); same(t.axis, V[i].axis, `motor ${i + 1}`); });
});

test('directions: frame table signs, name numbering, default rule', () => {
  const H = heavyLayout();
  // axes with random signs and scrambled order: the frame table restores both
  const scrambled = [5, 2, 7, 0, 3, 6, 1, 4].map((k, i) => ({ pos: H[k].pos, axis: H[k].axis.map((v) => v * (i % 2 ? -1 : 1)) }));
  const r = resolveDirections(scrambled, { frameConfig: 2 });
  assert.equal(r.directions, 'frame');
  r.thrusters.forEach((t, i) => { near(t.pos, H[i].pos, 1e-9, `slot ${i + 1}`); same(t.axis, H[i].axis, `slot ${i + 1}`); });
  // name numbers that fit the table are kept; numbers that do not are replaced by position
  const named = H.map((t, i) => ({ pos: t.pos, axis: t.axis.map((v) => -v), number: i + 1 }));
  assert.ok(resolveDirections(named, { frameConfig: 2 }).thrusters.every((t, i) => t.number === i + 1));
  const wrong = H.map((t, i) => ({ pos: t.pos, axis: t.axis, number: ((i + 4) % 8) + 1 }));
  const rw = resolveDirections(wrong, { frameConfig: 2 });
  assert.ok(rw.notes.some((n) => /renumbered by position/.test(n)));
  rw.thrusters.forEach((t, i) => near(t.pos, H[i].pos, 1e-9, `renumbered ${i + 1}`));
  // three thrusters fit no frame: vertical push down, horizontal away from the centre
  const d = resolveDirections([{ pos: [0.2, 0, 0], axis: [-1, 0, 0] }, { pos: [0, 0.2, 0], axis: [0, 0, -1] }, { pos: [-0.2, 0, 0], axis: [1, 0, 0] }], { frameConfig: 2 });
  assert.equal(d.directions, 'default');
  assert.deepEqual(d.thrusters.map((t) => t.axis), [[1, 0, 0], [-1, 0, 0], [0, 0, 1]]);
  assert.equal(directionWord([0, 0, 1]), 'down');
  assert.equal(directionWord([-Math.SQRT1_2, Math.SQRT1_2, 0]), 'back-right');
});

test('math helpers: symmetric eigen-decomposition and assignment', () => {
  const { values, vectors } = eigSym3([[4, 1, 0], [1, 3, 1], [0, 1, 2]]);
  vectors.forEach((v, k) => {
    const Av = [4 * v[0] + v[1], v[0] + 3 * v[1] + v[2], v[1] + 2 * v[2]];
    Av.forEach((x, i) => assert.ok(Math.abs(x - values[k] * v[i]) < 1e-10));
  });
  assert.deepEqual(hungarian([[4, 1, 3], [2, 0, 5], [3, 2, 2]]), [1, 0, 2]);
});
