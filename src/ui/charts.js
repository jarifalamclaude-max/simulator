// Canvas time-series charts for live telemetry and step responses.
// One y-axis per chart, thin 2px lines, legend + direct end labels, recessive
// grid, crosshair tooltip on hover.

import { h } from './dom.js';

const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();

export class StripChart {
  constructor({ title, unit = '', series, window = 20, yMin, yMax, symmetric = false, height = 160, static: isStatic = false }) {
    this.static = isStatic;
    this.title = title; this.unit = unit; this.series = series; this.window = window;
    this.yMin = yMin; this.yMax = yMax; this.symmetric = symmetric;
    this.canvas = h('canvas', { style: { width: '100%', height: '100%', display: 'block' }, 'aria-label': `${title} chart` });
    this.legend = h('div', { class: 'row', style: { gap: '10px', fontSize: '0.72rem', color: 'var(--ink-2)' } },
      series.map((s) => h('span', { class: 'row', style: { gap: '5px' } }, h('i', { style: { width: '14px', height: '0', borderTop: `2px ${s.dash ? 'dashed' : 'solid'} ${s.color}`, display: 'inline-block' } }), s.name)));
    this.el = h('div', { class: 'card', style: { padding: '8px 10px', gap: '4px', minWidth: '0', minHeight: `${height}px` } },
      h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', { style: { fontSize: '0.8rem', fontWeight: 600 } }, title, unit ? h('span', { style: { color: 'var(--ink-3)', fontWeight: 400 } }, ` ${unit}`) : null), this.legend),
      h('div', { style: isStatic ? { position: 'relative', height: `${height - 34}px` } : { position: 'relative', flex: '1', minHeight: '0' } }, this.canvas));
    this.hover = null;
    this.lastData = null;
    // static charts (step responses) redraw on hover; live charts redraw every tick anyway
    this.canvas.addEventListener('pointermove', (e) => { const r = this.canvas.getBoundingClientRect(); this.hover = (e.clientX - r.left) / r.width; if (this.static && this.lastData) this.draw(this.lastData); });
    this.canvas.addEventListener('pointerleave', () => { this.hover = null; if (this.static && this.lastData) this.draw(this.lastData); });
  }

  colorOf(s) {
    if (!s._c) s._c = s.color.startsWith('var(') ? css(s.color.slice(4, -1)) || '#888' : s.color;
    return s._c;
  }

  // data: { t: Float32Array, [key]: Float32Array }
  draw(data) {
    this.lastData = data;
    const c = this.canvas;
    const r = c.getBoundingClientRect();
    if (r.width < 10 || r.height < 10) return;
    const dpr = Math.min(window.devicePixelRatio || 1, 2);
    if (c.width !== Math.round(r.width * dpr) || c.height !== Math.round(r.height * dpr)) { c.width = Math.round(r.width * dpr); c.height = Math.round(r.height * dpr); }
    const g = c.getContext('2d');
    g.setTransform(dpr, 0, 0, dpr, 0, 0);
    const W = r.width, H = r.height;
    g.clearRect(0, 0, W, H);
    const t = data.t;
    if (!t || t.length < 2) { g.fillStyle = css('--ink-3'); g.font = '12px "IBM Plex Sans", sans-serif'; g.fillText('Waiting for data…', 8, 20); return; }
    const tEnd = t[t.length - 1], tStart = data.fixedStart ?? Math.max(t[0], tEnd - this.window);
    let i0 = 0;
    while (i0 < t.length - 1 && t[i0] < tStart) i0++;
    let lo = Infinity, hi = -Infinity;
    for (const s of this.series) {
      const y = data[s.key];
      if (!y) continue;
      for (let i = i0; i < y.length; i++) if (Number.isFinite(y[i])) { if (y[i] < lo) lo = y[i]; if (y[i] > hi) hi = y[i]; }
    }
    if (this.yMin !== undefined) lo = Math.min(lo, this.yMin);
    if (this.yMax !== undefined) hi = Math.max(hi, this.yMax);
    if (!Number.isFinite(lo)) { lo = -1; hi = 1; }
    if (this.symmetric) { const m = Math.max(Math.abs(lo), Math.abs(hi)); lo = -m; hi = m; }
    if (hi - lo < 1e-6) { hi += 0.5; lo -= 0.5; }
    const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
    const step = niceStep((hi - lo) / 4);
    const left = 40, right = 34, top = 6, bottom = 18;
    const X = (tt) => left + ((tt - tStart) / Math.max(1e-6, tEnd - tStart)) * (W - left - right);
    const Y = (v) => top + (1 - (v - lo) / (hi - lo)) * (H - top - bottom);
    g.font = '10px "IBM Plex Mono", monospace';
    g.fillStyle = css('--ink-3'); g.strokeStyle = css('--grid'); g.lineWidth = 1;
    for (let v = Math.ceil(lo / step) * step; v <= hi; v += step) {
      const y = Y(v);
      g.beginPath(); g.moveTo(left, y); g.lineTo(W - right, y); g.stroke();
      g.textAlign = 'right'; g.fillText(fmtTick(v, step), left - 5, y + 3);
    }
    g.textAlign = 'center';
    const tStep = niceStep((tEnd - tStart) / 4);
    for (let tt = Math.ceil(tStart / tStep) * tStep; tt <= tEnd; tt += tStep) g.fillText(`${fmtTick(tt, tStep)}s`, X(tt), H - 4);
    // series
    for (const s of this.series) {
      const y = data[s.key];
      if (!y) continue;
      g.strokeStyle = this.colorOf(s); g.lineWidth = 2; g.setLineDash(s.dash ? [5, 4] : []);
      g.beginPath();
      let started = false;
      const stride = Math.max(1, Math.floor((t.length - i0) / (W * 1.5)));
      for (let i = i0; i < y.length; i += stride) {
        if (!Number.isFinite(y[i])) { started = false; continue; }
        const px = X(t[i]), py = Y(y[i]);
        if (!started) { g.moveTo(px, py); started = true; } else g.lineTo(px, py);
      }
      g.stroke();
      g.setLineDash([]);
      // end label
      const last = y[y.length - 1];
      if (Number.isFinite(last)) { g.fillStyle = css('--ink-2'); g.textAlign = 'left'; g.fillText(fmtTick(last, step / 10), W - right + 3, Y(last) + 3); }
    }
    // hover crosshair
    if (this.hover !== null) {
      const px = left + this.hover * W - left;
      const tt = tStart + ((px - left) / (W - left - right)) * (tEnd - tStart);
      if (tt >= tStart && tt <= tEnd) {
        let k = i0;
        while (k < t.length - 1 && t[k] < tt) k++;
        g.strokeStyle = css('--ink-3'); g.beginPath(); g.moveTo(X(t[k]), top); g.lineTo(X(t[k]), H - bottom); g.stroke();
        const lines = [`t ${t[k].toFixed(2)} s`, ...this.series.filter((s) => data[s.key]).map((s) => `${s.name} ${fmtTick(data[s.key][k], step / 10)}`)];
        const bw = Math.max(...lines.map((l) => g.measureText(l).width)) + 14;
        let bx = X(t[k]) + 8;
        if (bx + bw > W - 4) bx = X(t[k]) - bw - 8;
        g.fillStyle = 'rgba(4,14,20,0.92)'; g.fillRect(bx, top + 4, bw, lines.length * 14 + 8);
        g.textAlign = 'left';
        lines.forEach((l, j) => { g.fillStyle = j === 0 ? css('--ink-3') : css('--ink'); g.fillText(l, bx + 7, top + 17 + j * 14); });
      }
    }
  }
}

function niceStep(raw) {
  const p = Math.pow(10, Math.floor(Math.log10(Math.max(raw, 1e-9))));
  const f = raw / p;
  return (f < 1.5 ? 1 : f < 3.5 ? 2 : f < 7.5 ? 5 : 10) * p;
}
function fmtTick(v, step) {
  if (!Number.isFinite(v)) return '—';
  const d = Math.max(0, Math.min(4, -Math.floor(Math.log10(Math.max(step, 1e-9)))));
  return v.toFixed(d);
}
