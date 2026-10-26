// Motor test panel (like QGroundControl's Motors page): spin one thruster at a
// time with a raw PWM and see what it does to the vehicle, plus a plain-words
// description of each thruster's push and turning effect.

import { h, fmt, toast } from '../dom.js';
import { thrusterLimits } from '../../vehicle/config.js';

export function thrusterEffect(t, cg) {
  const a = t.axis, n = Math.hypot(...a) || 1;
  const pol = t.polarity ?? 1;
  const F = a.map((v) => (v / n) * pol);
  const r = [t.pos[0] - cg[0], t.pos[1] - cg[1], t.pos[2] - cg[2]];
  const M = [r[1] * F[2] - r[2] * F[1], r[2] * F[0] - r[0] * F[2], r[0] * F[1] - r[1] * F[0]];
  const words = [];
  const f = (v, pos, neg) => { if (Math.abs(v) > 0.2) words.push(`${v > 0 ? pos : neg}`); };
  f(F[0], 'forward', 'backward');
  f(F[1], 'right', 'left');
  f(F[2], 'down', 'up');
  const push = words.length ? `pushes ${words.join('-')}` : 'pushes weakly';
  const turns = [];
  const m = (v, pos, neg) => { if (Math.abs(v) > 0.02) turns.push(`${v > 0 ? pos : neg} (${fmt(Math.abs(v) * 100, 0)} cm arm)`); };
  m(M[0], 'rolls right', 'rolls left');
  m(M[1], 'pitches nose up', 'pitches nose down');
  m(M[2], 'yaws right', 'yaws left');
  return { F, M, text: `${push}${turns.length ? '; ' + turns.join(', ') : ''}` };
}

export default {
  id: 'motors', title: 'Motor test', short: 'Motors', icon: 'motor', sub: 'Spin one thruster at a time',
  create(app) {
    const cfg = app.cfg;
    const el = h('div', { class: 'section', style: { gap: '16px' } });
    const pwm = cfg.thrusters.map(() => 1600);
    let seqTimer = null;
    const stop = () => { app.sim.fw.motorTest = null; clearTimeout(seqTimer); seqTimer = null; };
    const fwOk = () => {
      const fw = app.sim.fw;
      if (!fw.armed) { toast('Arm the vehicle first (Enter, or the ARM button)', 'bad'); return null; }
      return fw;
    };
    const live = [];

    el.append(h('div', { class: 'card' },
      h('b', {}, 'How it works'),
      h('p', { class: 'note' }, 'Arm, then hold a Spin button. The selected motor gets the raw PWM below and all others sit at 1500 µs, exactly like a MAVLink motor test: MOT_n_DIRECTION is not applied, so you see how the thruster is really wired. Watch the orange (forward) or blue (reverse) arrow and the vehicle in the 3D view.'),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: () => {
          const fw = fwOk(); if (!fw) return;
          stop();
          let i = 0;
          const next = () => {
            if (i >= cfg.thrusters.length || !app.sim.fw.armed) { stop(); toast('Motor sequence finished'); return; }
            app.sim.fw.motorTest = { motor: i, pwm: pwm[i] };
            toast(`Motor ${i + 1}: ${cfg.thrusters[i].name}`, '', 1600);
            i++;
            seqTimer = setTimeout(() => { app.sim.fw.motorTest = null; seqTimer = setTimeout(next, 600); }, 1500);
          };
          next();
        } }, 'Spin each motor in turn'),
        h('button', { class: 'btn danger', onclick: () => { stop(); toast('Motor test stopped'); } }, 'Stop'))));

    const cg = app.sim.mixer.cg;
    cfg.thrusters.forEach((t, i) => {
      const eff = thrusterEffect(t, cg);
      const lim = thrusterLimits(t);
      const val = h('span', { class: 'mono note' }, `${pwm[i]} µs`);
      const slider = h('input', { type: 'range', min: 1100, max: 1900, step: 10, value: pwm[i], 'aria-label': `Motor ${i + 1} test PWM`, oninput: () => { pwm[i] = +slider.value; val.textContent = `${pwm[i]} µs`; const fw = app.sim.fw; if (fw.motorTest?.motor === i) fw.motorTest = { motor: i, pwm: pwm[i] }; } });
      const spin = h('button', { class: 'btn', style: { touchAction: 'none' } }, 'Hold to spin');
      spin.addEventListener('pointerdown', (e) => { const fw = fwOk(); if (!fw) return; spin.setPointerCapture(e.pointerId); clearTimeout(seqTimer); fw.motorTest = { motor: i, pwm: pwm[i] }; spin.classList.add('primary'); });
      const release = () => { if (app.sim.fw.motorTest?.motor === i && !seqTimer) app.sim.fw.motorTest = null; spin.classList.remove('primary'); };
      spin.addEventListener('pointerup', release);
      spin.addEventListener('pointercancel', release);
      spin.addEventListener('keydown', (e) => { if ((e.key === ' ' || e.key === 'Enter') && !e.repeat) { const fw = fwOk(); if (fw) fw.motorTest = { motor: i, pwm: pwm[i] }; e.preventDefault(); } });
      spin.addEventListener('keyup', release);
      const out = h('span', { class: 'mono note' }, '—');
      live.push(out);
      const dir = cfg.params[`MOT_${i + 1}_DIRECTION`] ?? 1;
      el.append(h('div', { class: 'card' },
        h('div', { class: 'row', style: { justifyContent: 'space-between' } }, h('b', {}, `Motor ${i + 1} · ${t.name}`),
          h('span', { class: `pill ${dir < 0 ? 'warn' : 'muted'}` }, `MOT_${i + 1}_DIRECTION ${dir}`)),
        h('p', { class: 'note' }, `Above 1500 µs this thruster ${eff.text}. Range ${fmt(lim.fwdMax, 1)} N forward, ${fmt(lim.revMax, 1)} N reverse.`),
        h('div', { style: { display: 'grid', gridTemplateColumns: '1fr auto auto', gap: '10px', alignItems: 'center' } }, slider, val, spin),
        out));
    });
    return {
      el,
      tick() {
        const p = app.sim.plant;
        live.forEach((o, i) => { const th = p.thrusters[i]; if (th) o.textContent = `Now: ${Math.round(th.pwm ?? 1500)} µs · ${fmt(th.Teff, 1)} N`; });
      },
      dispose: stop,
    };
  },
};
