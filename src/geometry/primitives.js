// Closed, outward-wound triangle primitives used to build sample vehicles and
// test bodies. All functions append to a plain JS array of numbers (x,y,z...).

import { vnorm, vcross } from '../core/math.js';

function basis(axis) {
  const a = vnorm(axis);
  const t = Math.abs(a[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
  const u = vnorm(vcross(a, t));
  const v = vcross(a, u);
  return [a, u, v];
}
const add = (o, p, q, r) => { o.push(p[0], p[1], p[2], q[0], q[1], q[2], r[0], r[1], r[2]); };
const quad = (o, a, b, c, d) => { add(o, a, b, c); add(o, a, c, d); };

export function box(out, center, size) {
  const [cx, cy, cz] = center, [hx, hy, hz] = size.map((s) => s / 2);
  const P = (sx, sy, sz) => [cx + sx * hx, cy + sy * hy, cz + sz * hz];
  quad(out, P(1, -1, -1), P(1, 1, -1), P(1, 1, 1), P(1, -1, 1)); // +x
  quad(out, P(-1, -1, -1), P(-1, -1, 1), P(-1, 1, 1), P(-1, 1, -1)); // -x
  quad(out, P(-1, 1, -1), P(-1, 1, 1), P(1, 1, 1), P(1, 1, -1)); // +y
  quad(out, P(-1, -1, -1), P(1, -1, -1), P(1, -1, 1), P(-1, -1, 1)); // -y
  quad(out, P(-1, -1, 1), P(1, -1, 1), P(1, 1, 1), P(-1, 1, 1)); // +z
  quad(out, P(-1, -1, -1), P(-1, 1, -1), P(1, 1, -1), P(1, -1, -1)); // -z
  return out;
}

export function cylinder(out, center, axis, radius, length, seg = 24) {
  const [a, u, v] = basis(axis);
  const h = length / 2;
  const ring = (s) => Array.from({ length: seg }, (_, i) => {
    const t = (2 * Math.PI * i) / seg;
    const cu = Math.cos(t) * radius, cv = Math.sin(t) * radius;
    return [center[0] + a[0] * s * h + u[0] * cu + v[0] * cv, center[1] + a[1] * s * h + u[1] * cu + v[1] * cv, center[2] + a[2] * s * h + u[2] * cu + v[2] * cv];
  });
  const top = ring(1), bot = ring(-1);
  const ct = [center[0] + a[0] * h, center[1] + a[1] * h, center[2] + a[2] * h];
  const cb = [center[0] - a[0] * h, center[1] - a[1] * h, center[2] - a[2] * h];
  for (let i = 0; i < seg; i++) {
    const j = (i + 1) % seg;
    quad(out, bot[i], bot[j], top[j], top[i]);
    add(out, ct, top[i], top[j]);
    add(out, cb, bot[j], bot[i]);
  }
  return out;
}

// Annular duct (closed solid with a through-hole), e.g. a thruster nozzle.
export function tube(out, center, axis, rOut, rIn, length, seg = 24) {
  const [a, u, v] = basis(axis);
  const h = length / 2;
  const ring = (r, s) => Array.from({ length: seg }, (_, i) => {
    const t = (2 * Math.PI * i) / seg;
    const cu = Math.cos(t) * r, cv = Math.sin(t) * r;
    return [center[0] + a[0] * s * h + u[0] * cu + v[0] * cv, center[1] + a[1] * s * h + u[1] * cu + v[1] * cv, center[2] + a[2] * s * h + u[2] * cu + v[2] * cv];
  });
  const ot = ring(rOut, 1), ob = ring(rOut, -1), it = ring(rIn, 1), ib = ring(rIn, -1);
  for (let i = 0; i < seg; i++) {
    const j = (i + 1) % seg;
    quad(out, ob[i], ob[j], ot[j], ot[i]); // outer wall, outward
    quad(out, ib[i], it[i], it[j], ib[j]); // inner wall, facing the hole
    quad(out, ot[i], ot[j], it[j], it[i]); // top annulus
    quad(out, ob[i], ib[i], ib[j], ob[j]); // bottom annulus
  }
  return out;
}

export function sphere(out, center, radius, nLat = 16, nLon = 24) {
  const P = (i, j) => {
    const th = (Math.PI * i) / nLat, ph = (2 * Math.PI * j) / nLon;
    return [center[0] + radius * Math.sin(th) * Math.cos(ph), center[1] + radius * Math.sin(th) * Math.sin(ph), center[2] + radius * Math.cos(th)];
  };
  for (let i = 0; i < nLat; i++)
    for (let j = 0; j < nLon; j++) {
      const a = P(i, j), b = P(i + 1, j), c = P(i + 1, j + 1), d = P(i, j + 1);
      if (i > 0) add(out, a, b, d);
      if (i < nLat - 1) add(out, b, c, d);
    }
  return out;
}

export function ellipsoid(out, center, radii, nLat = 20, nLon = 32) {
  const tmp = [];
  sphere(tmp, [0, 0, 0], 1, nLat, nLon);
  for (let i = 0; i < tmp.length; i += 3) out.push(center[0] + tmp[i] * radii[0], center[1] + tmp[i + 1] * radii[1], center[2] + tmp[i + 2] * radii[2]);
  return out;
}
