// Live graph dock: target vs actual for roll, pitch, yaw and depth, fed from
// the simulation telemetry ring buffer.

import { h, seg, saveFile } from './dom.js';
import { StripChart } from './charts.js';

export class GraphDock {
  constructor(app, host) {
    this.app = app;
    this.window = 20;
    this.charts = [
      new StripChart({ title: 'Roll', unit: '°', series: [{ name: 'actual', key: 'roll', color: 'var(--series-1)' }, { name: 'target', key: 'roll_t', color: 'var(--series-2)', dash: true }] }),
      new StripChart({ title: 'Pitch', unit: '°', series: [{ name: 'actual', key: 'pitch', color: 'var(--series-1)' }, { name: 'target', key: 'pitch_t', color: 'var(--series-2)', dash: true }] }),
      new StripChart({ title: 'Yaw', unit: '°', series: [{ name: 'actual', key: 'yaw', color: 'var(--series-1)' }, { name: 'target', key: 'yaw_t', color: 'var(--series-2)', dash: true }] }),
      new StripChart({ title: 'Depth', unit: 'm', series: [{ name: 'actual', key: 'depth', color: 'var(--series-1)' }, { name: 'target', key: 'depth_t', color: 'var(--series-2)', dash: true }, { name: 'sensor', key: 'depth_meas', color: 'var(--series-3)' }] }),
    ];
    // resolve CSS variables once for canvas strokes
    const cs = getComputedStyle(document.documentElement);
    for (const c of this.charts) for (const s of c.series) if (s.color.startsWith('var(')) s.color = cs.getPropertyValue(s.color.slice(4, -1)).trim();
    for (const c of this.charts) c.legend.querySelectorAll('i').forEach((el, i) => { el.style.borderTopColor = c.series[i].color; });
    const winSeg = seg([[10, '10 s'], [20, '20 s'], [60, '60 s']], this.window, (v) => { this.window = v; this.charts.forEach((c) => (c.window = v)); });
    host.append(
      h('div', { class: 'dock-head' },
        h('b', { style: { fontFamily: 'var(--font-display)', letterSpacing: '0.06em' } }, 'LIVE GRAPHS'),
        h('span', { class: 'note', style: { fontSize: '0.72rem' } }, 'Dashed line: what the ArduSub controller is asking for. Solid: what the vehicle did.'),
        h('div', { class: 'bar-spacer' }), winSeg,
        h('button', { class: 'btn', onclick: () => saveFile(`duburi-telemetry-${Date.now()}.csv`, app.sim.telemetry.toCSV(), 'text/csv') }, 'Export CSV')),
      h('div', { class: 'dock-grid' }, this.charts.map((c) => c.el)));
  }
  reset() { /* telemetry is cleared by the simulation on reset */ }
  tick() {
    const tel = this.app.sim.telemetry;
    const n = Math.min(tel.count, Math.ceil(this.window * 50) + 5);
    const data = { t: tel.series('t', n) };
    for (const c of this.charts) for (const s of c.series) if (!data[s.key]) data[s.key] = tel.series(s.key, n);
    // unwrap yaw for display; the target is drawn relative to the unwrapped actual heading
    const y = data.yaw, yt = data.yaw_t;
    const raw = Float32Array.from(y);
    let off = 0;
    for (let i = 1; i < y.length; i++) { const d = raw[i] + off - y[i - 1]; if (d > 180) off -= 360; else if (d < -180) off += 360; y[i] = raw[i] + off; }
    for (let i = 0; i < yt.length; i++) if (Number.isFinite(yt[i])) yt[i] = y[i] + (((yt[i] - raw[i]) % 360) + 540) % 360 - 180;
    for (const c of this.charts) c.draw(data);
  }
}
