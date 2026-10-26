// Cockpit-style HUD drawn on a 2D canvas over the 3D view: artificial
// horizon with target markers, heading tape, depth tape with target and climb
// arrow, speed readout, thruster output bars, status line and messages.

import { RAD } from '../core/math.js';
import { MODE } from '../ardusub/sub.js';

const DEPTH_MODES = new Set([MODE.ALT_HOLD, MODE.POSHOLD, MODE.SURFACE, MODE.SURFTRAK, MODE.GUIDED, MODE.AUTO, MODE.CIRCLE].filter((m) => m !== undefined));

// battery as the firmware sees it (sensors.battery), falling back to the plant's battery model
export function batteryInfo(sim) {
  const b = sim.sensors?.battery ?? sim.fw?.battery;
  if (b && Number.isFinite(b.voltage)) return b;
  const pb = sim.plant?.battery;
  if (pb && Number.isFinite(pb.voltage)) return { voltage: pb.voltage, current: pb.current ?? 0, consumedMah: pb.consumedMah ?? 0, remainingPct: pb.remainingPct, healthy: true };
  return null;
}
const FS_NAMES = { pilotInput: 'PILOT INPUT', leak: 'LEAK', internalPressure: 'INTERNAL PRESSURE', internalTemp: 'INTERNAL TEMP', temperature: 'INTERNAL TEMP', battery: 'BATTERY', batteryLow: 'BATTERY LOW', batteryCritical: 'BATTERY CRITICAL', gcs: 'GCS', ekf: 'EKF', sensors: 'SENSORS', crash: 'CRASH', terrain: 'TERRAIN', mainloop: 'CPU', baro: 'DEPTH SENSOR' };
// every failsafe flag the firmware has raised (sub.failsafe booleans, as Sub::failsafe in Sub.h)
export function activeFailsafes(sim, st = {}) {
  const f = sim.fw?.failsafe || {}, out = [];
  for (const [k, v] of Object.entries(f)) if (v === true && !/^last|Warn|warn|Tick|Ms$/.test(k)) out.push(FS_NAMES[k] ?? k.replace(/([A-Z])/g, ' $1').toUpperCase());
  if (st.failsafe && !out.includes('PILOT INPUT')) out.unshift('PILOT INPUT');
  return out;
}
// pre-arm check failures, cached (the checks are not free)
const pfCache = new WeakMap();
export function armingFailures(sim) {
  const fw = sim.fw;
  if (!fw) return [];
  const c = pfCache.get(fw), now = fw.time ?? 0;
  if (c && Math.abs(now - c.t) < 0.5) return c.v;
  let v = [];
  try {
    const r = fw.armingCheckFailures?.() ?? fw.preArmFailures ?? fw.preArm?.();
    v = Array.isArray(r) ? r.filter(Boolean).map(String) : r ? [String(r)] : [];
  } catch { v = []; }
  pfCache.set(fw, { t: now, v });
  return v;
}

const INK = '#e3eef3', INK2 = 'rgba(227,238,243,0.62)', SHADOW = 'rgba(2,10,14,0.85)', ACC = '#5cc8e0', FOAM = '#ffb547', REV = '#4f8dff';

export class HUD {
  constructor(container) {
    this.canvas = document.createElement('canvas');
    container.append(this.canvas);
    this.g = this.canvas.getContext('2d');
    this.inset = { left: 66, top: 54, bottom: 0, right: 0 };
    this.visible = true;
    this.compact = false;
  }
  setInsets(i) { Object.assign(this.inset, i); }
  resize() {
    const r = this.canvas.getBoundingClientRect();
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    const w = Math.max(1, Math.round(r.width * dpr)), h = Math.max(1, Math.round(r.height * dpr));
    if (this.canvas.width !== w || this.canvas.height !== h) { this.canvas.width = w; this.canvas.height = h; }
    this.dpr = dpr; this.W = r.width; this.H = r.height;
  }

  draw(sim, extra = {}) {
    this.resize();
    const g = this.g;
    g.setTransform(this.dpr, 0, 0, this.dpr, 0, 0);
    g.clearRect(0, 0, this.W, this.H);
    if (!this.visible) return;
    const st = sim.fw.status();
    const L = this.inset.left, T = this.inset.top, B = this.H - this.inset.bottom, R = this.W - this.inset.right;
    const cx = (L + R) / 2, cy = (T + B) / 2;
    const small = R - L < 620;
    g.lineWidth = 1.5;
    g.font = '500 12px "IBM Plex Mono", monospace';
    const dx = R - (small ? 54 : 74);
    if (extra.cameraMode === 'fpv') {
      // camera view: full overlay horizon like a video HUD
      this.horizon(g, cx, cy, Math.min(R - L, B - T) * (small ? 0.24 : 0.2), st, sim);
      this.depthTape(g, dx, cy, Math.min(320, (B - T) * 0.48), st, sim);
    } else {
      // outside views: the vehicle is in the middle, so attitude becomes an instrument in the corner
      const r = small ? 44 : 60;
      const icx = R - r - (small ? 18 : 28), icy = B - r - (small ? 44 : 50);
      g.fillStyle = 'rgba(4,14,20,0.62)';
      g.beginPath(); g.arc(icx, icy, r + 18, 0, Math.PI * 2); g.fill();
      this.horizon(g, icx, icy, r, st, sim, true);
      const top = T + 64, bottom = icy - r - 44;
      const h = Math.min(320, bottom - top);
      if (h > 90) this.depthTape(g, dx, (top + bottom) / 2, h, st, sim);
    }
    this.headingTape(g, cx, T + 26, Math.min(420, (R - L) * 0.6), st.yaw * RAD, st);
    if (!small) this.speeds(g, L + 18, T + 70, sim, st, extra);
    else this.compactStatus(g, cx, T + 70, sim, extra);
    this.thrusterBars(g, L + 18, small ? T + 200 : B - 18, st, sim, small);
    this.messages(g, cx, B - (small ? 210 : 22), sim);
    // banners, most important first
    const bn = [];
    if (extra.paused) bn.push(['PAUSED', FOAM]);
    const fsl = activeFailsafes(sim, st);
    if (fsl.length) bn.push([`FAILSAFE · ${fsl.join(' · ')}`, '#ff8a8a']);
    if (extra.gcsLinkDown) bn.push(['GCS LINK DOWN (simulated)', '#fab219']);
    const fl = sim.plant.flood, hull = sim.plant.m.hull;
    if (fl && fl.active && fl.volume > 0) bn.push([`FLOODING · ${(fl.volume * 1e6).toFixed(0)} mL IN THE HOUSING`, '#ff8a8a']);
    else if (hull && hull.rating > 0 && sim.plant.position[2] > hull.rating) bn.push([`BELOW RATED DEPTH (${hull.rating} m)`, '#ff8a8a']);
    if (sim.sensors.depthOverRange) bn.push([`DEPTH SENSOR OVER RANGE (${sim.sensors.spec.maxBar} bar)`, '#fab219']);
    if (!st.armed) { const pf = armingFailures(sim); if (pf.length) bn.push([`PREARM: ${pf[0]}${pf.length > 1 ? ` (+${pf.length - 1})` : ''}`, '#fab219']); }
    bn.slice(0, 4).forEach(([t, c], i) => this.banner(g, cx, T + 98 + i * 34, t, c));
  }

  compactStatus(g, cx, y, sim) {
    const b = batteryInfo(sim), parts = [];
    if (b && b.healthy !== false) parts.push(`${b.voltage.toFixed(1)} V ${Number.isFinite(b.remainingPct) ? `${Math.round(b.remainingPct)}%` : ''}`);
    const rf = sim.sensors?.rangefinder;
    if (rf?.healthy) parts.push(`RNG ${rf.distance_m.toFixed(2)} m`);
    if (parts.length) this.text(g, parts.join('   '), cx, y, INK2, 'center', '500 11px "IBM Plex Mono", monospace');
  }

  text(g, s, x, y, color = INK, align = 'left', font) {
    if (font) g.font = font;
    g.textAlign = align;
    g.lineWidth = 3; g.strokeStyle = SHADOW; g.strokeText(s, x, y);
    g.fillStyle = color; g.fillText(s, x, y);
  }
  line(g, x1, y1, x2, y2, color = INK, w = 1.5) {
    g.strokeStyle = SHADOW; g.lineWidth = w + 2; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
    g.strokeStyle = color; g.lineWidth = w; g.beginPath(); g.moveTo(x1, y1); g.lineTo(x2, y2); g.stroke();
  }

  horizon(g, cx, cy, r, st, sim, compact = false) {
    const roll = st.roll, pitch = st.pitch * RAD;
    const pxPerDeg = r / (compact ? 40 : 32);
    g.save();
    g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.clip();
    g.translate(cx, cy);
    g.rotate(-roll);
    const off = pitch * pxPerDeg;
    // sky/water tint
    g.fillStyle = 'rgba(92,200,224,0.07)'; g.fillRect(-r * 2, -r * 2 + off, r * 4, r * 2);
    g.fillStyle = 'rgba(255,181,71,0.05)'; g.fillRect(-r * 2, off, r * 4, r * 2);
    this.line(g, -r * 1.2, off, r * 1.2, off, INK, 1.5);
    g.font = '500 11px "IBM Plex Mono", monospace';
    for (let d = -60; d <= 60; d += compact ? 10 : 5) {
      if (d === 0) continue;
      const y = off - d * pxPerDeg;
      if (Math.abs(y) > r) continue;
      const w = d % 10 === 0 ? r * 0.36 : r * 0.16;
      this.line(g, -w, y, w, y, INK2, 1);
      if (compact ? d % 20 === 0 : d % 10 === 0) { this.text(g, String(d), w + 6, y + 4, INK2, 'left'); if (!compact) this.text(g, String(d), -w - 6, y + 4, INK2, 'right'); }
    }
    g.restore();
    // target attitude: the horizon the pilot would see at the commanded attitude
    if (st.armed && st.mode !== MODE.MANUAL) {
      const tr = st.targetEuler[0], tp = st.targetEuler[1] * RAD;
      g.save();
      g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.clip();
      g.translate(cx, cy); g.rotate(-tr);
      g.setLineDash([7, 5]);
      g.strokeStyle = ACC; g.lineWidth = 2;
      g.beginPath(); g.moveTo(-r, tp * pxPerDeg); g.lineTo(r, tp * pxPerDeg); g.stroke();
      g.setLineDash([]);
      g.restore();
    }
    g.save();
    g.restore();
    // frame ring & roll scale
    g.strokeStyle = 'rgba(227,238,243,0.25)'; g.lineWidth = 1;
    g.beginPath(); g.arc(cx, cy, r, 0, Math.PI * 2); g.stroke();
    for (const d of [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60]) {
      const a = (d * Math.PI) / 180 - Math.PI / 2;
      const l = d % 30 === 0 ? 12 : 7;
      this.line(g, cx + Math.cos(a) * r, cy + Math.sin(a) * r, cx + Math.cos(a) * (r + l), cy + Math.sin(a) * (r + l), INK2, 1);
    }
    const ra = -roll - Math.PI / 2;
    g.fillStyle = FOAM;
    g.beginPath();
    g.moveTo(cx + Math.cos(ra) * (r - 2), cy + Math.sin(ra) * (r - 2));
    g.lineTo(cx + Math.cos(ra + 0.06) * (r - 14), cy + Math.sin(ra + 0.06) * (r - 14));
    g.lineTo(cx + Math.cos(ra - 0.06) * (r - 14), cy + Math.sin(ra - 0.06) * (r - 14));
    g.closePath(); g.fill();
    // vehicle symbol
    g.strokeStyle = SHADOW; g.lineWidth = 5;
    const sym = () => { g.beginPath(); g.moveTo(cx - r * 0.42, cy); g.lineTo(cx - r * 0.14, cy); g.lineTo(cx - r * 0.07, cy + r * 0.07); g.moveTo(cx + r * 0.42, cy); g.lineTo(cx + r * 0.14, cy); g.lineTo(cx + r * 0.07, cy + r * 0.07); g.stroke(); };
    sym(); g.strokeStyle = FOAM; g.lineWidth = 2.5; sym();
    g.fillStyle = FOAM; g.beginPath(); g.arc(cx, cy, 3, 0, Math.PI * 2); g.fill();
    const ty = cy + r + (compact ? 34 : 22);
    this.text(g, `R ${(roll * RAD).toFixed(1)}°`, cx - r - (compact ? 10 : 0), ty, INK, 'left', '500 12px "IBM Plex Mono", monospace');
    this.text(g, `P ${pitch.toFixed(1)}°`, cx + r + (compact ? 10 : 0), ty, INK, 'right');
  }

  headingTape(g, cx, y, w, hdg, st) {
    const span = 90, ppd = w / span;
    g.save();
    g.beginPath(); g.rect(cx - w / 2, y - 22, w, 36); g.clip();
    g.font = '500 11px "IBM Plex Mono", monospace';
    const start = Math.floor((hdg - span / 2) / 5) * 5;
    for (let d = start; d <= hdg + span / 2; d += 5) {
      const x = cx + (d - hdg) * ppd;
      const dd = ((d % 360) + 360) % 360;
      const major = dd % 30 === 0;
      this.line(g, x, y + 8, x, y + (major ? -4 : 2), INK2, 1);
      if (major) {
        const lbl = { 0: 'N', 90: 'E', 180: 'S', 270: 'W' }[dd] ?? String(dd).padStart(3, '0');
        this.text(g, lbl, x, y - 8, dd % 90 === 0 ? FOAM : INK2, 'center');
      }
    }
    if (st.armed && st.mode !== MODE.MANUAL && st.mode !== MODE.ACRO) {
      let td = st.targetEuler[2] * RAD - hdg;
      td = ((td + 540) % 360) - 180;
      const x = cx + Math.max(-span / 2, Math.min(span / 2, td)) * ppd;
      g.fillStyle = ACC; g.beginPath(); g.moveTo(x, y + 10); g.lineTo(x - 6, y + 17); g.lineTo(x + 6, y + 17); g.closePath(); g.fill();
    }
    g.restore();
    const h = ((hdg % 360) + 360) % 360;
    g.fillStyle = 'rgba(4,14,20,0.85)'; g.strokeStyle = FOAM; g.lineWidth = 1.5;
    g.beginPath(); g.roundRect(cx - 30, y + 12, 60, 22, 5); g.fill(); g.stroke();
    this.text(g, `${h.toFixed(0).padStart(3, '0')}°`, cx, y + 28, INK, 'center', '600 14px "IBM Plex Mono", monospace');
  }

  depthTape(g, x, cy, h, st, sim) {
    // tape, readout and target in ArduSub's own depth estimate (inertial nav / baro frame, what
    // QGC and Cockpit show); the pool geometry below is placed from the true depth
    const zUp = sim.sensors?.positionZUpCm;
    const depth = Number.isFinite(zUp) ? -zUp / 100 : Number.isFinite(st.depth) ? st.depth : sim.plant.position[2];
    const trueDepth = sim.plant.position[2];
    const span = 3, ppm = h / span;
    g.save();
    g.beginPath(); g.rect(x - 40, cy - h / 2, 80, h); g.clip();
    g.font = '500 11px "IBM Plex Mono", monospace';
    // pool floor below the vehicle (hatched) and the housing's rated depth
    const floorD = sim.env.floorDepth(sim.plant.position[0]);
    const yf = cy + (floorD - trueDepth) * ppm;
    if (yf < cy + h / 2) {
      g.fillStyle = 'rgba(255,181,71,0.10)'; g.fillRect(x - 40, yf, 80, cy + h / 2 - yf);
      g.strokeStyle = 'rgba(255,181,71,0.35)'; g.lineWidth = 1; g.beginPath();
      for (let k = -80; k < h; k += 9) { g.moveTo(x - 40 + k, yf); g.lineTo(x - 40 + k + 30, yf + 30); }
      g.stroke();
      this.line(g, x - 30, yf, x + 30, yf, FOAM, 1.5);
    }
    const rating = sim.plant.m.hull?.rating;
    if (rating > 0) {
      const yr = cy + (rating - trueDepth) * ppm;
      if (yr > cy - h / 2 && yr < cy + h / 2) { g.setLineDash([5, 4]); this.line(g, x - 30, yr, x + 30, yr, '#ff8a8a', 1.5); g.setLineDash([]); }
    }
    const s0 = Math.floor((depth - span / 2) * 10) / 10;
    for (let d = s0; d <= depth + span / 2; d += 0.1) {
      const yy = cy + (d - depth) * ppm;
      const major = Math.abs((d * 2) - Math.round(d * 2)) < 1e-6;
      this.line(g, x, yy, x + (major ? 12 : 6), yy, INK2, 1);
      if (major) this.text(g, (Math.abs(d) < 1e-6 ? 0 : d).toFixed(1), x - 6, yy + 4, INK2, 'right');
    }
    // surface line
    const ys = cy + (0 - trueDepth) * ppm;
    this.line(g, x - 30, ys, x + 30, ys, ACC, 1.5);
    if (st.armed && Number.isFinite(st.depthTarget) && DEPTH_MODES.has(st.mode)) {
      const yt = Math.max(cy - h / 2, Math.min(cy + h / 2, cy + (st.depthTarget - depth) * ppm));
      g.fillStyle = ACC; g.beginPath(); g.moveTo(x + 14, yt); g.lineTo(x + 24, yt - 6); g.lineTo(x + 24, yt + 6); g.closePath(); g.fill();
    }
    g.restore();
    g.fillStyle = 'rgba(4,14,20,0.85)'; g.strokeStyle = FOAM; g.lineWidth = 1.5;
    g.beginPath(); g.roundRect(x - 44, cy - 13, 62, 26, 5); g.fill(); g.stroke();
    this.text(g, depth.toFixed(2), x - 13, cy + 5, INK, 'center', '600 14px "IBM Plex Mono", monospace');
    this.text(g, 'DEPTH m', x - 13, cy - h / 2 - 10, INK2, 'center', '500 10px "IBM Plex Mono", monospace');
    this.text(g, `true ${trueDepth.toFixed(2)}`, x - 13, cy + 26, INK2, 'center', '500 10px "IBM Plex Mono", monospace');
    // climb arrow
    const climb = st.climb; // m/s up
    const len = Math.max(-1, Math.min(1, climb / 0.5)) * (h / 2 - 20);
    if (Math.abs(len) > 2) {
      this.line(g, x + 34, cy, x + 34, cy - len, climb > 0 ? '#7fe0ad' : FOAM, 3);
    }
    this.text(g, `${climb >= 0 ? '▲' : '▼'} ${Math.abs(climb).toFixed(2)}`, x - 13, cy + h / 2 + 16, INK2, 'center', '500 11px "IBM Plex Mono", monospace');
  }

  speeds(g, x, y, sim, st, extra = {}) {
    const v = sim.plant.velocityBody, p = sim.plant.position;
    g.font = '500 12px "IBM Plex Mono", monospace';
    const rows = [
      ['SURGE', `${v[0].toFixed(2)} m/s`], ['SWAY', `${v[1].toFixed(2)} m/s`], ['HEAVE', `${v[2].toFixed(2)} m/s`],
      ['NORTH', `${p[0].toFixed(2)} m`], ['EAST', `${p[1].toFixed(2)} m`],
      ['TO FLOOR', `${(sim.env.floorDepth(p[0]) - p[2]).toFixed(2)} m`],
      // what the pressure sensor reads and the depth ArduSub computes from it
      ['PRESSURE', Number.isFinite(sim.sensors.pressureMeas) ? `${(sim.sensors.pressureMeas / 1e5).toFixed(4)} bar` : '—'],
      ['SENSOR', `${sim.sensors.depthMeasured.toFixed(2)} m`],
      ['GAIN', `${Math.round(st.gain * 100)} %`], ['LIGHTS', `${Math.round(st.lights?.[0] ?? 0)} %`],
    ];
    const b = batteryInfo(sim);
    if (b) {
      rows.push(['BATTERY', b.healthy === false ? 'no monitor' : `${b.voltage.toFixed(2)} V`]);
      if (b.healthy !== false) rows.push(['CURRENT', `${(b.current ?? 0).toFixed(1)} A`], ['USED', `${Math.round(b.consumedMah ?? 0)} mAh${Number.isFinite(b.remainingPct) ? ` ${Math.round(b.remainingPct)}%` : ''}`]);
    }
    const rf = sim.sensors?.rangefinder;
    if (rf) rows.push(['RANGEFINDER', rf.healthy ? `${rf.distance_m.toFixed(2)} m` : 'no data']);
    const tc = sim.sensors?.internalTempC;
    if (Number.isFinite(tc)) rows.push(['INT TEMP', `${tc.toFixed(1)} °C`]);
    rows.push(['GCS LINK', extra.gcsLinkDown ? 'DOWN' : sim.fw.failsafe?.gcs ? 'LOST' : 'OK']);
    const flags = [];
    if (st.inputHold) flags.push('INPUT HOLD');
    if (st.rollPitchMode) flags.push('ATTITUDE INPUT');
    if (st.atSurface) flags.push('AT SURFACE');
    if (st.saturated) flags.push('THRUSTERS SATURATED');
    if (st.leak) flags.push('LEAK FAILSAFE');
    if (st.internalPressureFs) flags.push('INTERNAL PRESSURE');
    if (st.atBottom) flags.push('AT BOTTOM');
    g.fillStyle = 'rgba(4,14,20,0.6)';
    g.beginPath(); g.roundRect(x - 10, y - 16, 172, rows.length * 18 + flags.length * 18 + 16, 8); g.fill();
    rows.forEach(([k, val], i) => { this.text(g, k, x, y + i * 18, INK2, 'left'); this.text(g, val, x + 152, y + i * 18, INK, 'right'); });
    let yy = y + rows.length * 18 + 2;
    for (const f of flags) { this.text(g, f, x, yy, FOAM, 'left'); yy += 18; }
  }

  thrusterBars(g, x, yb, st, sim, small) {
    const n = st.motorsOut.length;
    if (!n) return;
    const bw = small ? 10 : 14, gap = small ? 6 : 8, hh = small ? 44 : 64;
    const w = n * (bw + gap) - gap;
    g.fillStyle = 'rgba(4,14,20,0.6)';
    g.beginPath(); g.roundRect(x - 8, yb - hh * 2 - 30, w + 16, hh * 2 + 40, 8); g.fill();
    const mid = yb - hh - 12;
    this.line(g, x - 4, mid, x + w + 4, mid, INK2, 1);
    for (let i = 0; i < n; i++) {
      const v = st.motorsOut[i];
      const bx = x + i * (bw + gap);
      g.fillStyle = 'rgba(227,238,243,0.08)';
      g.fillRect(bx, mid - hh, bw, hh * 2);
      g.fillStyle = v >= 0 ? FOAM : REV;
      const hgt = Math.max(-1, Math.min(1, v)) * hh;
      g.fillRect(bx, hgt >= 0 ? mid - hgt : mid, bw, Math.abs(hgt));
      this.text(g, String(i + 1), bx + bw / 2, yb, INK2, 'center', '500 10px "IBM Plex Mono", monospace');
    }
    this.text(g, 'THRUSTERS', x, mid - hh - 8, INK2, 'left', '500 10px "IBM Plex Mono", monospace');
  }

  messages(g, cx, y, sim) {
    const now = sim.fw.time;
    const msgs = sim.fw.messages.filter((m) => now - m.t < 6).slice(-3);
    msgs.forEach((m, i) => {
      const a = Math.max(0, Math.min(1, (6 - (now - m.t)) / 1.5));
      g.globalAlpha = a;
      const col = m.severity <= 3 ? '#ff9a9a' : m.severity === 4 ? '#fab219' : INK;
      this.text(g, m.text, cx, y - (msgs.length - 1 - i) * 18, col, 'center', '500 13px "IBM Plex Sans", sans-serif');
    });
    g.globalAlpha = 1;
  }

  banner(g, cx, y, s, color) {
    g.font = '700 18px "Saira Semi Condensed", sans-serif';
    const w = g.measureText(s).width + 28;
    g.fillStyle = 'rgba(4,14,20,0.8)'; g.strokeStyle = color; g.lineWidth = 1.5;
    g.beginPath(); g.roundRect(cx - w / 2, y - 18, w, 28, 6); g.fill(); g.stroke();
    this.text(g, s, cx, y + 2, color, 'center');
  }
}
