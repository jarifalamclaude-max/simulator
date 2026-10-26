// Team panel: team name, logo and colour shown in the top bar, and separate
// vehicle profiles per team so several teams can share one simulator.

import { h, toast, confirmDialog, pickFile, readDataURL, saveFile, storage, idbGet, idbPut } from '../dom.js';
import { normalizeConfig } from '../../vehicle/config.js';
import { slug } from './params.js';
import { toBase64, fromBase64 } from './logs.js';

const KEY = 'duburi.v1';

// Downscale a logo to at most 256 px so it fits comfortably in browser storage.
async function logoFromFile(file) {
  const url = await readDataURL(file);
  if (file.type === 'image/svg+xml') {
    if (url.length > 300_000) throw new Error('SVG logo is larger than 300 KB');
    return url;
  }
  const img = await new Promise((res, rej) => { const i = new Image(); i.onload = () => res(i); i.onerror = () => rej(new Error('Not an image')); i.src = url; });
  const s = Math.min(1, 256 / Math.max(img.naturalWidth, img.naturalHeight));
  const c = document.createElement('canvas');
  c.width = Math.max(1, Math.round(img.naturalWidth * s)); c.height = Math.max(1, Math.round(img.naturalHeight * s));
  c.getContext('2d').drawImage(img, 0, 0, c.width, c.height);
  return c.toDataURL('image/png');
}

export default {
  id: 'team', title: 'Team', short: 'Team', icon: 'team', sub: 'Name, logo and vehicle profiles',
  create(app) {
    const el = h('div', { class: 'section', style: { gap: '18px' } });
    const t = app.team;
    const commit = () => { app.saveTeams(); app.applyTeamBranding(); };

    // ---- branding ----
    const name = h('input', { type: 'text', value: t.name || '', maxlength: 40, 'aria-label': 'Team name', onchange: () => { t.name = name.value.trim() || 'Team'; commit(); app.renderRail(); } });
    const color = h('input', { type: 'color', value: t.color || '#ffb547', class: 'swatch', 'aria-label': 'Team colour', oninput: () => { t.color = color.value; commit(); } });
    const preview = t.logo ? h('img', { src: t.logo, alt: `${t.name} logo`, style: { width: '72px', height: '72px', objectFit: 'contain', borderRadius: '10px', border: '1px solid var(--line)', background: 'rgba(255,255,255,0.04)' } })
      : h('div', { class: 'logo-fallback', style: { width: '72px', height: '72px', display: 'grid', placeItems: 'center', borderRadius: '10px', border: '1px dashed var(--line-2)', color: t.color, fontFamily: 'var(--font-display)', fontSize: '2rem', fontWeight: 700 } }, (t.name || 'D').replace(/^team\s+/i, '').slice(0, 1).toUpperCase());
    el.append(h('div', { class: 'section' }, h('h3', {}, 'This team'),
      h('div', { style: { display: 'grid', gridTemplateColumns: 'auto 1fr', gap: '14px', alignItems: 'center' } }, preview,
        h('div', { class: 'section', style: { gap: '8px' } },
          h('label', { class: 'field' }, h('span', {}, 'Team name'), name),
          h('div', { class: 'row' }, h('span', { class: 'note' }, 'Accent colour'), color))),
      h('div', { class: 'row' },
        h('button', { class: 'btn primary', onclick: async () => {
          const f = await pickFile('image/png,image/jpeg,image/svg+xml,image/webp');
          if (!f) return;
          try { t.logo = await logoFromFile(f); commit(); app.refreshPanel(); toast('Logo updated', 'good'); } catch (e) { toast(`Logo not loaded: ${e.message}`, 'bad'); }
        } }, t.logo ? 'Change logo' : 'Add team logo'),
        t.logo ? h('button', { class: 'btn', onclick: () => { t.logo = null; commit(); app.refreshPanel(); } }, 'Remove logo') : null),
      h('p', { class: 'note' }, 'The DUBURI wordmark stays at the top; your logo and team name sit next to it. PNG, JPG, WebP or SVG; large images are scaled down to 256 px.')));

    // ---- profiles ----
    const list = h('div', { class: 'section', style: { gap: '6px' } });
    for (const team of app.teams) {
      const active = team.id === app.team.id;
      list.append(h('div', { class: 'card', style: { flexDirection: 'row', alignItems: 'center', gap: '10px', padding: '8px 10px', borderColor: active ? 'var(--accent)' : '' } },
        team.logo ? h('img', { src: team.logo, alt: '', style: { width: '28px', height: '28px', objectFit: 'contain' } }) : h('span', { style: { width: '28px', textAlign: 'center', color: team.color, fontFamily: 'var(--font-display)', fontWeight: 700 } }, (team.name || 'T').replace(/^team\s+/i, '').slice(0, 1).toUpperCase()),
        h('b', { style: { flex: '1', minWidth: 0, overflow: 'hidden', textOverflow: 'ellipsis', whiteSpace: 'nowrap' } }, team.name),
        active ? h('span', { class: 'pill good' }, 'Active') : h('button', { class: 'btn', onclick: () => app.switchTeam(team.id) }, 'Switch'),
        !active ? h('button', { class: 'btn danger', title: `Delete ${team.name}`, onclick: async () => {
          if (!await confirmDialog(`Delete ${team.name}?`, 'Removes this team and its saved vehicle from this browser. Export it first if you may need it again.', { ok: 'Delete', danger: true })) return;
          app.teams = app.teams.filter((x) => x.id !== team.id);
          storage.del(`${KEY}.team.${team.id}.cfg`);
          app.saveTeams(); app.refreshPanel(); toast(`${team.name} deleted`);
        } }, 'Delete') : null));
    }
    const newName = h('input', { type: 'text', placeholder: 'New team name', maxlength: 40, 'aria-label': 'New team name' });
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Teams on this computer'), list,
      h('div', { style: { display: 'grid', gridTemplateColumns: '1fr auto', gap: '8px' } }, newName,
        h('button', { class: 'btn primary', onclick: async () => {
          const nm = newName.value.trim();
          if (!nm) { newName.focus(); toast('Type a team name'); return; }
          const id = `${slug(nm)}-${Date.now().toString(36)}`;
          app.teams.push({ id, name: nm, logo: null, color: '#5cc8e0' });
          app.saveTeams();
          await app.switchTeam(id);
        } }, 'Add team')),
      h('p', { class: 'note' }, 'Each team keeps its own vehicle, parameters, thrusters and logo in this browser. A new team starts from the sample 8-thruster vehicle with ArduSub default parameters (FRAME_CONFIG 2).')));

    // ---- share ----
    el.append(h('div', { class: 'section' }, h('h3', {}, 'Share a team profile'),
      h('div', { class: 'row' },
        h('button', { class: 'btn', onclick: async () => {
          app.saveNow();
          const doc = { schema: 'duburi.team', version: 1, team: { name: t.name, logo: t.logo, color: t.color }, vehicle: app.cfg };
          if (app.cfg.model.source === 'stl') { const buf = await idbGet(`stl:${t.id}`); if (buf && buf.byteLength < 60e6) doc.stl = { name: app.cfg.model.fileName, base64: toBase64(buf) }; }
          saveFile(`${slug(t.name)}.duburi-team.json`, JSON.stringify(doc), 'application/json');
        } }, 'Export this team'),
        h('button', { class: 'btn', onclick: async () => {
          const f = await pickFile('.json,application/json'); if (!f) return;
          let doc;
          try { doc = JSON.parse(await f.text()); if (doc.schema !== 'duburi.team') throw new Error('not a team file'); } catch (e) { toast(`${f.name}: ${e.message}`, 'bad'); return; }
          const nm = doc.team?.name || 'Imported team';
          const id = `${slug(nm)}-${Date.now().toString(36)}`;
          let cfg;
          try { cfg = normalizeConfig(doc.vehicle); } catch (e) { toast(e.message, 'bad'); return; }
          storage.set(`${KEY}.team.${id}.cfg`, cfg);
          if (doc.stl?.base64) await idbPut(`stl:${id}`, fromBase64(doc.stl.base64));
          app.teams.push({ id, name: nm, logo: doc.team?.logo || null, color: doc.team?.color || '#5cc8e0' });
          app.saveTeams();
          await app.switchTeam(id);
        } }, 'Import a team')),
      h('p', { class: 'note' }, 'Profiles live in this browser only. Export to move a team to another computer, or to send your setup to another team.')));
    return { el };
  },
};
