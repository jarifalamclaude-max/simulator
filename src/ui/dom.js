// Small DOM toolkit (no framework): element builder, inputs bound to config
// paths, toasts, in-page confirm dialogs, icons, file save/load.

export function h(tag, props = {}, ...children) {
  const el = document.createElement(tag);
  for (const [k, v] of Object.entries(props || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k === 'class') el.className = v;
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2).toLowerCase(), v);
    else if (k === 'html') el.innerHTML = v;
    else if (k in el && k !== 'list' && typeof v !== 'string') el[k] = v;
    else el.setAttribute(k, v === true ? '' : String(v));
  }
  for (const c of children.flat(Infinity)) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}
export const $ = (sel, root = document) => root.querySelector(sel);
export function clear(el) { while (el.firstChild) el.firstChild.remove(); return el; }

export const fmt = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '—');
export const fmtSigned = (v, d = 2) => (Number.isFinite(v) ? (v >= 0 ? '+' : '') + v.toFixed(d) : '—');
let uid = 0;
export const nextId = (p = 'f') => `${p}${++uid}`;

// ---- icons (line icons, 24 grid) ----
const ICONS = {
  hull: 'M3 13c3 4 15 4 18 0M5 13V9h14v4M9 9V6h6v3',
  thruster: 'M12 3v4M12 17v4M4 12h4M16 12h4M8.5 8.5l7 7M15.5 8.5l-7 7M12 12m-3 0a3 3 0 1 0 6 0a3 3 0 1 0 -6 0',
  mixer: 'M4 6h10M18 6h2M4 12h4M12 12h8M4 18h12M20 18h0M16 4v4M10 10v4M18 16v4',
  frame: 'M7 6h10a2 2 0 0 1 2 2v8a2 2 0 0 1-2 2H7a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2zM3 8l2 2M21 8l-2 2M3 16l2-2M21 16l-2-2M12 9v6M9 12h6',
  wave: 'M2 12c2-3 4-3 6 0s4 3 6 0 4-3 6 0M2 17c2-3 4-3 6 0s4 3 6 0 4-3 6 0M2 7c2-3 4-3 6 0s4 3 6 0 4-3 6 0',
  flask: 'M9 3h6M10 3v6l-5 9a2 2 0 0 0 2 3h10a2 2 0 0 0 2-3l-5-9V3M7.5 15h9',
  env: 'M12 3v18M3 12h18M5 5l14 14',
  params: 'M4 7h16M4 12h16M4 17h10',
  gamepad: 'M6 8h12a4 4 0 0 1 4 4v1a3 3 0 0 1-5.4 1.8L15 13H9l-1.6 1.8A3 3 0 0 1 2 13v-1a4 4 0 0 1 4-4zM7 11v3M5.5 12.5h3M16 11.5h0M18 13h0',
  motor: 'M12 2v4M12 18v4M2 12h4M18 12h4M12 12m-5 0a5 5 0 1 0 10 0a5 5 0 1 0 -10 0',
  chart: 'M4 20V4M4 20h16M7 15l4-5 3 3 5-7',
  log: 'M6 4h12v16H6zM9 8h6M9 12h6M9 16h4',
  team: 'M4 21V4l8 3 8-3v17M4 12l8 3 8-3',
  help: 'M12 21a9 9 0 1 0 0-18a9 9 0 0 0 0 18zM9.5 9a2.5 2.5 0 1 1 3.5 2.3c-.6.3-1 .9-1 1.6V14M12 17.5h0',
  play: 'M7 5l12 7-12 7z',
  pause: 'M8 5v14M16 5v14',
  reset: 'M4 12a8 8 0 1 0 2.3-5.7M4 4v4h4',
  camera: 'M3 8h4l2-3h6l2 3h4v11H3zM12 17a4 4 0 1 0 0-8a4 4 0 0 0 0 8z',
  close: 'M6 6l12 12M18 6L6 18',
  graph: 'M3 17l5-6 4 4 4-7 5 5',
  download: 'M12 4v11M7 10l5 5 5-5M5 20h14',
  upload: 'M12 20V9M7 14l5-5 5 5M5 4h14',
  target: 'M12 12m-8 0a8 8 0 1 0 16 0a8 8 0 1 0 -16 0M12 12m-3 0a3 3 0 1 0 6 0a3 3 0 1 0 -6 0',
  speed: 'M4 15a8 8 0 1 1 16 0M12 15l4-5',
};
export function icon(name) {
  const svg = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  svg.setAttribute('viewBox', '0 0 24 24');
  svg.setAttribute('aria-hidden', 'true');
  const p = document.createElementNS('http://www.w3.org/2000/svg', 'path');
  p.setAttribute('d', ICONS[name] || ICONS.help);
  svg.append(p);
  return svg;
}

// ---- toasts ----
let toastHost = null;
export function toast(msg, kind = '', ms = 3200) {
  if (!toastHost) { toastHost = h('div', { class: 'toasts', role: 'status', 'aria-live': 'polite' }); document.getElementById('app').append(toastHost); }
  const t = h('div', { class: `toast ${kind}` }, msg);
  toastHost.append(t);
  setTimeout(() => t.remove(), ms);
  while (toastHost.children.length > 4) toastHost.firstChild.remove();
}

// ---- in-page confirm (alert/confirm are unavailable in sandboxed viewers) ----
export function confirmDialog(title, body, { ok = 'Confirm', cancel = 'Cancel', danger = false } = {}) {
  return new Promise((resolve) => {
    const close = (v) => { back.remove(); resolve(v); };
    const back = h('div', { class: 'modal-back', onclick: (e) => { if (e.target === back) close(false); } },
      h('div', { class: 'modal', role: 'dialog', 'aria-modal': 'true' },
        h('h3', {}, title),
        typeof body === 'string' ? h('p', { class: 'note' }, body) : body,
        h('div', { class: 'row', style: { justifyContent: 'flex-end' } },
          h('button', { class: 'btn', onclick: () => close(false) }, cancel),
          h('button', { class: `btn ${danger ? 'danger' : 'primary'}`, onclick: () => close(true) }, ok))));
    document.getElementById('app').append(back);
    back.querySelector('.btn.primary, .btn.danger')?.focus();
  });
}
export function infoDialog(title, body, actions = []) {
  const back = h('div', { class: 'modal-back', onclick: (e) => { if (e.target === back) back.remove(); } });
  const m = h('div', { class: 'modal', role: 'dialog', 'aria-modal': 'true' }, h('h3', {}, title), body,
    h('div', { class: 'row', style: { justifyContent: 'flex-end' } }, ...actions, h('button', { class: 'btn primary', onclick: () => back.remove() }, 'Close')));
  back.append(m);
  document.getElementById('app').append(back);
  return back;
}

// ---- bound inputs ----
export function getPath(obj, path) { return path.split('.').reduce((o, k) => (o == null ? undefined : o[k]), obj); }
export function setPath(obj, path, val) {
  const ks = path.split('.');
  let o = obj;
  for (let i = 0; i < ks.length - 1; i++) { if (o[ks[i]] == null) o[ks[i]] = /^\d+$/.test(ks[i + 1]) ? [] : {}; o = o[ks[i]]; }
  o[ks[ks.length - 1]] = val;
}
/**
 * numberField({label, get, set, step, min, max, unit, digits, hint})
 */
export function numberField(o) {
  const id = nextId('n');
  const inp = h('input', { type: 'number', id, step: o.step ?? 'any', min: o.min, max: o.max, inputmode: 'decimal' });
  const show = () => { const v = o.get(); inp.value = Number.isFinite(v) ? (o.digits !== undefined ? (+v).toFixed(o.digits) : +(+v).toPrecision(6)) : ''; };
  show();
  inp.addEventListener('change', () => {
    const v = parseFloat(inp.value);
    const bad = !Number.isFinite(v) || (o.min !== undefined && v < o.min) || (o.max !== undefined && v > o.max);
    inp.classList.toggle('invalid', bad);
    if (bad) { inp.title = `Enter a number${o.min !== undefined ? ' ≥ ' + o.min : ''}${o.max !== undefined ? ' ≤ ' + o.max : ''}`; return; }
    inp.title = '';
    o.set(v);
  });
  // Enter commits the value and hands the keyboard back to the flight keys
  inp.addEventListener('keydown', (e) => { if (e.key === 'Enter') { e.preventDefault(); inp.blur(); } });
  const wrap = h('label', { class: 'field', for: id }, h('span', {}, o.label, o.unit ? h('em', {}, ` (${o.unit})`) : null), inp);
  wrap.refresh = show;
  if (o.hint) wrap.title = o.hint;
  return wrap;
}
export function selectField({ label, options, get, set, hint }) {
  const id = nextId('s');
  const sel = h('select', { id }, options.map(([v, l]) => h('option', { value: String(v) }, l)));
  sel.value = String(get());
  sel.addEventListener('change', () => set(sel.value));
  const wrap = h('label', { class: 'field', for: id }, h('span', {}, label), sel);
  if (hint) wrap.title = hint;
  wrap.refresh = () => { sel.value = String(get()); };
  return wrap;
}
export function vec3Field({ label, get, set, unit = 'm', step = 0.001, names = ['X fwd', 'Y right', 'Z down'] }) {
  const fields = [0, 1, 2].map((k) => numberField({ label: names[k], unit: k === 0 ? unit : undefined, step, get: () => get()[k], set: (v) => { const a = get().slice(); a[k] = v; set(a); } }));
  const el = h('div', { class: 'field' }, h('span', {}, label), h('div', { class: 'grid3' }, fields));
  el.refresh = () => fields.forEach((f) => f.refresh());
  return el;
}
export function seg(options, value, onChange) {
  const el = h('div', { class: 'seg', role: 'group' });
  const draw = (v) => {
    clear(el);
    for (const [k, label] of options) el.append(h('button', { type: 'button', class: k === v ? 'on' : '', 'aria-pressed': k === v ? 'true' : 'false', onclick: () => { draw(k); onChange(k); } }, label));
  };
  draw(value);
  el.set = draw;
  return el;
}
export function checkbox(label, get, set) {
  const id = nextId('c');
  const inp = h('input', { type: 'checkbox', id });
  inp.checked = !!get();
  inp.addEventListener('change', () => set(inp.checked));
  return h('label', { class: 'row check', for: id, style: { gap: '8px', cursor: 'pointer', flexWrap: 'nowrap', alignItems: 'flex-start' } }, inp, h('span', { class: 'note' }, label));
}

// ---- files ----
let downloadsCap;
async function downloadsCapability() {
  if (downloadsCap !== undefined) return downloadsCap;
  try { downloadsCap = window.claude && typeof window.claude.use === 'function' ? await window.claude.use('downloads') : null; } catch { downloadsCap = null; }
  return downloadsCap;
}
const ALLOWED = ['txt', 'json', 'csv', 'md', 'html', 'svg', 'png', 'zip', 'pdf'];
export async function saveFile(filename, data, mime = 'application/octet-stream') {
  const inViewer = !!(window.claude && typeof window.claude.use === 'function');
  if (inViewer) {
    const d = await downloadsCapability();
    if (d) {
      let name = filename;
      const ext = name.split('.').pop().toLowerCase();
      if (!ALLOWED.includes(ext)) name += '.txt';
      try { await d.save({ filename: name, data }); toast(`Saved ${name}`, 'good'); return true; }
      catch (e) { if (e && e.code === 'declined') return false; }
    }
    if (typeof data === 'string') { showTextForCopy(filename, data); return false; }
    toast('Saving files is not available in this view. Open the HTML file directly to export.', 'bad');
    return false;
  }
  const blob = data instanceof Blob ? data : new Blob([data], { type: mime });
  const a = h('a', { href: URL.createObjectURL(blob), download: filename });
  document.body.append(a);
  a.click();
  setTimeout(() => { URL.revokeObjectURL(a.href); a.remove(); }, 1500);
  toast(`Saved ${filename}`, 'good');
  return true;
}
export function showTextForCopy(filename, text) {
  const ta = h('textarea', { rows: 12, readonly: true, style: { width: '100%' } });
  ta.value = text;
  const copy = h('button', { class: 'btn', onclick: async () => {
    try { await navigator.clipboard.writeText(text); toast('Copied', 'good'); } catch { ta.select(); toast('Press Ctrl+C to copy', ''); }
  } }, 'Copy');
  infoDialog(`Copy ${filename}`, h('div', { class: 'section' }, h('p', { class: 'note' }, 'This view cannot save files directly. Copy the contents and save them as ' + filename + '.'), ta), [copy]);
}
export function pickFile(accept) {
  return new Promise((resolve) => {
    const inp = h('input', { type: 'file', accept, style: { display: 'none' } });
    inp.addEventListener('change', () => { resolve(inp.files[0] || null); inp.remove(); });
    document.body.append(inp);
    inp.click();
  });
}
export const readText = (file) => file.text();
export const readBuffer = (file) => file.arrayBuffer();
export function readDataURL(file) {
  return new Promise((res, rej) => { const r = new FileReader(); r.onload = () => res(r.result); r.onerror = rej; r.readAsDataURL(file); });
}

// ---- storage (per-viewer convenience; may be unavailable) ----
export const storage = {
  get(k, d = null) { try { const v = localStorage.getItem(k); return v === null ? d : JSON.parse(v); } catch { return d; } },
  set(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); return true; } catch { return false; } },
  del(k) { try { localStorage.removeItem(k); } catch { /* ignore */ } },
};
export function idbPut(key, value) {
  return new Promise((resolve) => {
    try {
      const rq = indexedDB.open('duburi-sim', 1);
      rq.onupgradeneeded = () => rq.result.createObjectStore('blobs');
      rq.onsuccess = () => { const tx = rq.result.transaction('blobs', 'readwrite'); tx.objectStore('blobs').put(value, key); tx.oncomplete = () => resolve(true); tx.onerror = () => resolve(false); };
      rq.onerror = () => resolve(false);
    } catch { resolve(false); }
  });
}
export function idbGet(key) {
  return new Promise((resolve) => {
    try {
      const rq = indexedDB.open('duburi-sim', 1);
      rq.onupgradeneeded = () => rq.result.createObjectStore('blobs');
      rq.onsuccess = () => { const tx = rq.result.transaction('blobs', 'readonly'); const g = tx.objectStore('blobs').get(key); g.onsuccess = () => resolve(g.result ?? null); g.onerror = () => resolve(null); };
      rq.onerror = () => resolve(null);
    } catch { resolve(null); }
  });
}
