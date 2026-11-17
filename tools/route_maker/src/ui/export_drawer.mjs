// export_drawer.mjs — EXPORT (RMP1 brief D4, A.3.7; §1.4's states). The route's C++ — exactly what will be
// written — with Copy (always there), and the way this browser writes it: in Chrome or Edge with the shulib
// folder connected, straight into the ONE marked region of src/team/skills_route.cpp (its line endings kept;
// a hand edit there refused unless the student chooses "Overwrite"); elsewhere, "pick your skills_route.cpp,
// get the updated one back". While any step has an error the drawer shows ONLY the list of errors, each one click
// from its step, and no code: the C++ is made only from a route with no error (a step still missing a number has
// nothing to print). The same hand-edit choice (keep / overwrite) in both ways of writing. Robot two's mirrored
// line is CHECKED, never written (D6, R2).

import { h, icon } from './dom.mjs';
import { exportRegion } from '../model/export.mjs';
import { MIRRORED_LINE, SKILLS_ROUTE_FILE, ROBOT_TWO_AUTONS_FILE } from '../model/region.mjs';
import { copyText } from '../host/files_download.mjs';

/** Open the export drawer. */
export async function openExportDrawer(app) {
    if (document.querySelector('.drawer')) {
        return;  // one at a time (Ctrl+E pressed again)
    }
    const route = app.route();
    const errors = app.issues().filter((x) => x.level === 'error');
    const files = app.files();
    const close = () => {
        scrim.remove();
        drawer.remove();
        document.removeEventListener('keydown', onKey);
        app.focusField();
    };
    const onKey = (e) => {
        if (e.key === 'Escape') {
            close();
        }
    };
    document.addEventListener('keydown', onKey);
    const scrim = h('div', { class: 'scrim', onclick: close });
    const body = h('div', { class: 'drawer-body' });
    const drawer = h('aside', { class: 'drawer', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'export-title' },
        h('div', { class: 'drawer-head' }, icon('code'), h('h2', { id: 'export-title' }, 'Export the route'), h('span', { class: 'spacer' }),
          h('button', { class: 'btn icon ghost', 'aria-label': 'Close', onclick: close }, icon('close'))),
        body);
    document.body.append(scrim, drawer);

    const refuse = (title, list) => {
        body.append(h('div', { class: 'banner error', role: 'alert' }, icon('error'),
            h('div', {}, h('strong', {}, title),
              h('ul', {}, list.slice(0, 6).map((x) => h('li', {}, x.stepIndex >= 0 ? `Step ${x.stepIndex + 1}: ` : '', x.words, ' ',
                  h('button', { class: 'btn ghost', onclick: () => { close(); app.select(x.stepIndex >= 0 ? x.stepIndex : 'start'); } }, 'Go there')))),
              list.length > 6 ? h('p', { class: 'hint' }, `…and ${list.length - 6} more: the status bar's "Go to step" walks them.`) : null)));
        drawer.querySelector('button[aria-label="Close"]').focus();
    };
    if (errors.length) {
        refuse(`Fix ${errors.length === 1 ? 'this error' : `these ${errors.length} errors`} first: the robot would not run this route.`, errors);
        return;
    }
    let code;
    try {
        code = exportRegion(route);
    } catch (e) {
        // never reached while the checks hold (every number the export prints is one they ask for): if it is,
        // the student sees why, not a dead button
        refuse('The route builder could not write this route. Tell the coordinator these words:', [{ stepIndex: -1, words: String(e && e.message ? e.message : e) }]);
        return;
    }

    const status = h('div', { 'aria-live': 'polite' });
    const say = (cls, text) => status.replaceChildren(h('div', { class: `banner ${cls}` }, icon(cls === 'ok' ? 'check' : cls === 'error' ? 'error' : 'warn'), h('div', {}, text)));
    const copyBtn = h('button', { class: 'btn', onclick: async () => {
        const ok = await copyText(code);
        say(ok ? 'ok' : 'warn', ok ? 'Copied. Paste it in place of everything from the ROUTE MAKER BEGIN line to its END line.'
                                   : 'This browser would not copy: select the code above and copy it by hand.');
    } }, icon('copy'), 'Copy the code');

    const writeCard = h('div', { class: 'card' });
    /** The student's choice for a hand edit: keep it (nothing written) or overwrite it (`again(true)`). */
    const askHandEdit = (out, again) => {
        say('warn', out.words);
        status.append(h('div', { class: 'row', style: 'margin-top:8px' },
            h('button', { class: 'btn', onclick: () => say('ok', 'Kept. Nothing was written. Copy the code if you want it.') }, 'Keep their change'),
            h('button', { class: 'btn danger', onclick: () => again(true) }, 'Overwrite')));
    };
    const writeOut = async (overwrite) => {
        const out = await files.exportToFile(route, app.project(), { overwrite });
        if (out.ok) {
            app.setProject(out.project, 'exported');
            const saved = await app.saveQuietly();  // false: route_maker.json changed in the folder since this page read it
            say(saved === false ? 'warn' : 'ok', `${out.words}. Only the lines between ROUTE MAKER BEGIN and END changed; the file keeps its own line endings.`
                + (saved === false ? ' src/team/route_maker.json was NOT saved: it changed in the folder since this page read it. Press Save to choose which to keep.' : ''));
        } else if (out.why === 'handEdit') {
            askHandEdit(out, writeOut);
        } else {
            say('warn', out.words);
        }
    };
    const downloadOut = async (overwrite) => {
        const out = await files.exportByDownload(route, app.project(), { overwrite });
        if (out.ok) {
            // the project now remembers this export (its hash) -- but only in this page until route_maker.json is
            // saved: without that, the next export from the saved file would take this code for a hand edit
            app.setProject(out.project, 'unsaved');
            say('ok', `${out.words}. Now save the project too (Save, Ctrl+S): route_maker.json remembers what was exported, so the next export knows this code is the builder's.`);
            status.append(h('div', { class: 'row', style: 'margin-top:8px' },
                h('button', { class: 'btn primary', onclick: () => app.save() }, icon('save'), 'Save route_maker.json')));
        } else if (out.why === 'handEdit') {
            askHandEdit(out, downloadOut);
        } else if (out.why !== 'cancelled') {
            say('warn', out.words);
        }
    };
    if (files.canWriteFolder) {
        const st = files.connected() ? await files.skillsFileState() : { state: 'unknown' };
        const mirror = files.connected() ? await files.mirroredLine() : 'unknown';
        writeCard.append(h('h3', {}, `Write it into ${SKILLS_ROUTE_FILE}`));
        if (!files.connected()) {
            writeCard.append(h('p', {}, 'Connect your shulib folder (the repository\'s top folder) once; then Export writes the route there.'),
                h('button', { class: 'btn primary', onclick: async () => {
                    close();
                    await app.connect();
                    openExportDrawer(app);
                } }, icon('folder'), 'Connect your shulib folder'));
        } else if (st.state === 'ready') {
            writeCard.append(h('p', {}, `Folder: ${files.folderName()}. The builder writes ONLY between the ROUTE MAKER lines.`),
                h('button', { class: 'btn primary', onclick: () => writeOut(false) }, icon('download'), `Write into ${SKILLS_ROUTE_FILE}`));
        } else {
            writeCard.append(h('div', { class: 'banner warn' }, icon('warn'),
                h('div', {}, st.state === 'noFile' || st.state === 'noMarkers'
                    ? `${SKILLS_ROUTE_FILE} ${st.state === 'noFile' ? 'is not in this folder yet' : 'has no ROUTE MAKER lines yet'}: it arrives with robot two's update. Until then, use Copy.`
                    : st.words || 'The skills-route file cannot be written.')),
                h('button', { class: 'btn primary', disabled: true }, `Write into ${SKILLS_ROUTE_FILE}`));
        }
        writeCard.append(h('p', { class: 'hint' }, mirror === 'present' ? `Robot two runs this route mirrored: its line is in ${ROBOT_TWO_AUTONS_FILE}. ✓`
            : mirror === 'missing' ? `The line that runs this route mirrored on robot two is missing from ${ROBOT_TWO_AUTONS_FILE}: tell the coordinator. The line: ${MIRRORED_LINE}`
            : 'Robot two\'s mirrored line is checked once the folder is connected.'));
    } else {
        writeCard.append(h('h3', {}, 'Put it into your skills_route.cpp'),
            h('p', {}, 'This browser cannot write into a folder (Chrome or Edge can). Pick your src/team/skills_route.cpp: you get the updated file back as a download to put in its place.'),
            h('button', { class: 'btn primary', onclick: () => downloadOut(false) }, icon('download'), 'Pick skills_route.cpp and download the updated one'),
            h('p', { class: 'hint' }, 'Robot two\'s mirrored line is not checked in this browser.'));
    }
    body.append(
        h('p', { class: 'muted', style: 'margin:0' }, 'The route as C++ — every step explained in a comment, the same lines you could type. The robot runs exactly this.'),
        h('pre', { class: 'code', tabindex: '0', 'aria-label': 'The exported code' }, code),
        h('div', { class: 'row' }, copyBtn),
        writeCard,
        status,
    );
    copyBtn.focus();
}
