// Non-blocking notice shown after thruster detection on an uploaded design:
// what was found and how, the mixer the allocator built from it, one row per
// motor (model, place, push direction, confidence, Flip) and Accept / Discard.

import { h, icon, fmt } from './dom.js';
import { THRUSTER_TYPES, directionWord } from '../geometry/thrusterDetect.js';
import { AXIS_LABEL } from '../alloc/allocation.js';

const SOURCE = { name: 'part name', shape: 'duct shape', hull: 'ring in the hull' };

/**
 * @param app      the App (cfg.thrusters already hold the detected layout)
 * @param det      detectThrusters() result
 * @param actions  { accept(), discard(), flip(i), openFrame() }
 */
export function thrusterNotice(app, det, actions) {
  const cfg = app.cfg;
  const an = app.sim?.mixer?.analysis;
  const lowDir = det.directions !== 'frame';
  const rows = det.thrusters.map((d, i) => {
    const t = cfg.thrusters[i];
    if (!t) return null;
    const flipped = t.axis.some((v, k) => Math.abs(v + d.axis[k]) < 1e-6 && Math.abs(v) > 1e-6);
    const conf = Math.min(d.confidence, lowDir ? d.directionConfidence : 1);
    const pill = conf >= 0.6 ? 'good' : 'warn';
    const where = `${fmt(t.pos[0] * 1000, 0)}, ${fmt(t.pos[1] * 1000, 0)}, ${fmt(t.pos[2] * 1000, 0)} mm`;
    return h('div', { class: 'detect-row' },
      h('b', { class: 'mono' }, `${i + 1}`),
      h('div', { class: 'detect-what' },
        h('span', {}, `${THRUSTER_TYPES[d.type]?.label ?? 'Thruster'} · pushes ${directionWord(t.axis)}${lowDir && !flipped ? ' ?' : ''}${flipped ? ' (flipped)' : ''}`),
        h('span', { class: 'note' }, `${where} · by ${SOURCE[d.source] ?? d.source}${d.od ? ` · Ø ${fmt(d.od * 1000, 0)} mm` : ''}`)),
      h('span', { class: `pill ${pill}`, title: lowDir ? 'Detection × direction confidence (no frame table matched)' : 'Detection confidence' }, `${Math.round(conf * 100)}%`),
      h('button', { class: 'btn small', title: `Thruster ${i + 1} pushes the other way`, onclick: () => actions.flip(i) }, 'Flip'));
  });
  const mix = an ? an.summary : '';
  const missing = an ? ['roll', 'pitch', 'yaw', 'throttle', 'forward', 'lateral'].filter((a) => !an.controllable.includes(a)).map((a) => AXIS_LABEL[a]) : [];
  const ic = icon('thruster');
  ic.classList.add('detect-icon');
  return h('section', { class: 'detect-notice', role: 'status', 'aria-live': 'polite', 'aria-label': 'Thrusters found in your design' },
    h('div', { class: 'row', style: { justifyContent: 'space-between', flexWrap: 'nowrap', alignItems: 'flex-start' } },
      h('div', { class: 'row', style: { gap: '8px', flexWrap: 'nowrap', alignItems: 'flex-start' } }, ic, h('b', {}, det.summary)),
      h('button', { class: 'icon-btn close', title: 'Keep this layout and close', 'aria-label': 'Keep this layout and close', onclick: () => actions.accept() }, icon('close'))),
    h('p', { class: 'note' }, `Custom frame built from them (Automatic mixer): ${mix || '—'}${missing.length ? ` — no ${missing.join(', ')}` : ''}.`,
      lowDir ? ' Directions are a guess: check each one in Frame > Try it.' : ` Push directions and motor order follow the FRAME_CONFIG ${det.frame.config} table; spin directions follow your MOT_n_DIRECTION.`),
    ...det.notes.map((n) => h('p', { class: 'note warn' }, n)),
    h('div', { class: 'detect-list' }, rows),
    h('div', { class: 'row', style: { justifyContent: 'flex-end' } },
      h('button', { class: 'btn', onclick: () => actions.openFrame() }, 'Open in Frame'),
      h('button', { class: 'btn danger', title: 'Put back the thrusters you had before the upload', onclick: () => actions.discard() }, 'Discard'),
      h('button', { class: 'btn primary', onclick: () => actions.accept() }, 'Accept')));
}
