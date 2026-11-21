// controls_export.mjs — EXPORT on the Controls tab (RMP1-B; D9): the driver file's three regions — the controls,
// the header's button map, the feel —, each with Copy, and the way this browser writes them: Chrome or Edge with
// the folder connected writes ONLY between the three marker pairs (all three or none, the file's line endings
// kept, a hand edit refused unless "Overwrite"); until the file carries the `control words` and `feel` markers
// (robot two's update places them) it offers Copy only. Elsewhere: pick the driver file, get the updated one back.

import { h, icon } from './dom.mjs';
import { exportRegions, spliceRegions, driverFilePath, regionKey, REGIONS } from '../model/driver_file.mjs';
import { copyText } from '../host/files_download.mjs';
import { robotName } from '../model/robot_names.mjs';

const TITLES = { controls: 'Which button does what (kControls)', 'control words': 'The button map at the top of the file', feel: 'The feel (kFeel, kLoop)' };

/** Open the drawer: `x` = {app, robot, id, state, issues, meta, select, written}. */
export async function openControlsExport(x) {
    if (document.querySelector('.drawer')) {
        return;
    }
    const { app, id } = x;
    const path = driverFilePath(id);
    const close = () => {
        scrim.remove();
        drawer.remove();
        document.removeEventListener('keydown', onKey);
        document.getElementById('export')?.focus();  // the keyboard's place: back where it opened
    };
    const onKey = (e) => e.key === 'Escape' && close();
    document.addEventListener('keydown', onKey);
    const scrim = h('div', { class: 'scrim', onclick: close });
    const body = h('div', { class: 'drawer-body' });
    const drawer = h('aside', { class: 'drawer', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'cx-title' },
        h('div', { class: 'drawer-head' }, icon('code'), h('h2', { id: 'cx-title' }, `Export ${robotName(x.id)}'s controls`), h('span', { class: 'spacer' }),
          h('button', { class: 'btn icon ghost', 'aria-label': 'Close', onclick: close }, icon('close'))),
        body);
    document.body.append(scrim, drawer);
    const errors = x.issues.filter((i) => i.level === 'error');
    if (errors.length) {
        body.append(h('div', { class: 'banner error', role: 'alert' }, icon('error'), h('div', {},
            h('strong', {}, `Fix ${errors.length === 1 ? 'this' : `these ${errors.length}`} first: the build would refuse the driver file.`),
            h('ul', {}, errors.slice(0, 8).map((e) => {
                const where = e.at >= 0 ? { control: e.at } : e.at === -2 ? { feel: true } : e.button ? { button: e.button } : e.mechanism ? { mechanism: e.mechanism } : null;
                return h('li', {}, e.words, ' ', where ? h('button', { class: 'btn ghost', onclick: () => { close(); x.select(where); } }, 'Go there') : null);
            })))));
        drawer.querySelector('button[aria-label="Close"]').focus();
        return;
    }
    const notes = x.issues.filter((i) => i.code === 'noteStale');
    if (notes.length) {
        body.append(h('div', { class: 'banner warn' }, icon('warn'), h('div', {}, h('strong', {}, 'Check these notes in the file after you write:'),
            h('ul', {}, notes.map((n) => h('li', {}, n.words))))));
    }
    const regions = exportRegions(x.state, x.robot);
    const all = REGIONS.map((r) => regions[r].join('\n'));
    const status = h('div', { 'aria-live': 'polite' });
    const say = (cls, text, ...more) => status.replaceChildren(h('div', { class: `banner ${cls}` }, icon(cls === 'ok' ? 'check' : 'warn'), h('div', {}, text, ...more)));
    const copy = (text, what) => async () => {
        const ok = await copyText(text);
        say(ok ? 'ok' : 'warn', ok ? `Copied ${what}. Paste it in place of the same lines of ${path}.` : 'This browser would not copy: select the code and copy it by hand.');
    };
    const done = (out) => {
        const ex = { ...(app.project().exports || {}) };
        for (const r of REGIONS) {
            ex[regionKey(id, r)] = out.hashes[r];
        }
        app.setProject({ ...app.project(), exports: ex }, 'exported');
        x.written(out.hashes);
    };
    const handEdit = (out, again) => {
        say('warn', out.words);
        status.append(h('div', { class: 'row', style: 'margin-top:8px' },
            h('button', { class: 'btn', onclick: () => say('ok', 'Kept. Nothing was written.') }, 'Keep their change'),
            h('button', { class: 'btn danger', onclick: () => again(true) }, 'Overwrite')));
    };
    const files = app.files();
    const write = async (overwrite) => {
        const text = await files.readTeam(path);
        if (text === null) {
            say('warn', `There is no ${path} in this folder.`);
            return;
        }
        const out = spliceRegions(text, id, regions, { base: x.meta.base, overwrite });
        if (!out.ok) {
            return out.why === 'handEdit' ? handEdit(out, write) : say('warn', out.words);
        }
        if (out.changed) {
            await files.writeTeam(path, out.text);
        }
        done(out);
        const saved = await app.saveQuietly();
        return say(saved === false ? 'warn' : 'ok', `${out.changed ? `Wrote ${path}` : `${path} already says this`}: only the three marked parts changed, `
            + 'with the file\'s own line endings.', saved === false ? ' src/team/route_maker.json was NOT saved (it changed in the folder): press Save.' : '');
    };
    const download = async (overwrite) => {
        const picked = await files.pickText();
        if (!picked) {
            return;
        }
        const out = spliceRegions(picked.text, id, regions, { base: x.meta.base, overwrite });
        if (!out.ok) {
            return out.why === 'handEdit' ? handEdit(out, download) : say('warn', out.words);
        }
        await files.offerDownload(path.split('/').pop(), out.text);
        done(out);
        app.setProject(app.project(), 'unsaved');
        return say('ok', `Downloaded the updated ${path.split('/').pop()}: put it in src/team/ in place of the old one. Then Save the project (Ctrl+S).`);
    };
    const card = h('div', { class: 'card' });
    if (files.canWriteFolder && files.connected()) {
        const text = await files.readTeam(path);
        const ready = text !== null && spliceRegions(text, id, regions, { base: x.meta.base, overwrite: true }).ok;
        card.append(h('h3', {}, `Write it into ${path}`),
            ready ? h('p', {}, 'Only the lines between its three marker pairs change.')
                : h('div', { class: 'banner warn' }, icon('warn'), h('div', {}, text === null ? `${path} is not in this folder.`
                    : `${path} has no "control words" and "feel" markers yet: it gets them with ${robotName('two')}'s update. Until then, Copy each part.`)),
            h('button', { class: 'btn primary', disabled: !ready, onclick: () => write(false) }, icon('download'), `Write into ${path}`));
    } else if (files.canWriteFolder) {
        card.append(h('h3', {}, `Write it into ${path}`), h('p', {}, 'Connect your shulib folder once (the top bar); then Export writes the three parts.'));
    } else {
        card.append(h('h3', {}, `Put it into your ${path.split('/').pop()}`),
            h('p', {}, 'This browser cannot write into a folder (Chrome or Edge can). Pick your driver file: you get the updated one back as a download.'),
            h('button', { class: 'btn primary', onclick: () => download(false) }, icon('download'), `Pick ${path.split('/').pop()} and download the updated one`));
    }
    body.append(h('p', { class: 'muted', style: 'margin:0' }, `Three parts of ${path}, each between its own two marker lines. Every line is the build's to judge; the comments are your words.`),
        ...REGIONS.map((r, k) => h('div', { class: 'region' }, h('div', { class: 'row' }, h('h3', {}, TITLES[r]), h('span', { class: 'spacer' }),
            h('button', { class: 'btn small', onclick: copy(all[k], `"${TITLES[r]}"`) }, icon('copy'), 'Copy')),
            h('pre', { class: 'code', tabindex: '0', 'aria-label': TITLES[r] }, all[k]))),
        h('div', { class: 'row' }, h('button', { class: 'btn', onclick: copy(all.join('\n\n'), 'all three parts') }, icon('copy'), 'Copy all three')),
        card, status);
    drawer.querySelector('.drawer-body button').focus();
}
