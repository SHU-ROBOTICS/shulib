// to_robot.mjs — the export drawer's cards after a result (RMP1-F): the result (green, red or a question), "Will it
// build?", where a copy is pasted, and Next: the commands that build and upload, robot one or two, each with Copy.

import { h, icon } from './dom.mjs';
import { copyText } from '../host/files_download.mjs';
import { robotCommands, TESTER } from '../model/to_robot.mjs';
import { ROUTE_NAME } from '../model/route.mjs';
import { SKILLS_ROUTE_FILE } from '../model/region.mjs';

/** A result card: `kind` 'ok' (its icon ✓), 'error' (!) or 'warn' (a question); its title, then `body`. */
export function resultCard(kind, title, ...body) {
    return h('div', { class: `banner ${kind} xr-card`, role: kind === 'error' ? 'alert' : null, 'data-result': kind },
        icon(kind === 'ok' ? 'check' : kind === 'error' ? 'error' : 'warn'), h('div', {}, h('strong', { class: 'xr-title' }, title), body));
}

/** A small Copy button for `text`; it says "Copied" for a moment. */
export function copyButton(text, what) {
    const words = h('span', {}, 'Copy');
    const b = h('button', { class: 'btn small', 'aria-label': `Copy ${what}`, onclick: async () => {
        const ok = await copyText(text);
        b.dataset.copied = ok ? 'yes' : 'no';
        words.textContent = ok ? 'Copied' : 'Select it and copy';
        setTimeout(() => {
            words.textContent = 'Copy';
        }, 1600);
    } }, icon('copy'), words);
    return b;
}

/** "Will it build?" (D3): what the page checked, and the one thing only the build checks. `missing`: the robots whose
 *  autons file lacks its addTeamRoutes line (read when the folder is connected; null: not read here). */
export function willItBuild(kind, missing = null) {
    const line = (mark, text) => h('li', { class: mark === '✗' ? 'bad' : null }, h('span', { class: 'xr-mark' }, mark), text);
    return h('div', { class: 'xr-build' }, h('h4', {}, 'Will it build?'), h('ul', {},
        line('✓', 'Every step follows the library\'s rules: the page checked each one.'),
        kind === 'copy' ? line('•', 'The page cannot see your editor: paste it exactly as in the picture.')
            : kind === 'write' ? line('✓', 'The file has exactly this code between its two ROUTE MAKER lines (read back after writing).')
                : line('✓', 'The downloaded file has exactly this code between its two ROUTE MAKER lines: replace yours with it.'),
        missing === null ? line('•', 'Not checked here: the addTeamRoutes line in each robot\'s autons file (Edge or Chrome checks it once the folder is connected).')
            : missing.length === 0 ? line('✓', 'Both robots register every route: each autons file has its addTeamRoutes line.')
            : missing.map((m) => line('✗', `Robot ${m.robot} will not list the routes: ${m.file} is missing ${m.line} — tell the coordinator.`)),
        line('→', 'The C++ compiler checks the rest when you build: that is the next step.')));
}

/** Where a copy goes: the two ROUTE MAKER lines and everything between them, selected, pasted over. */
export function pastePicture() {
    const marker = (end) => `// ═══ ROUTE MAKER — route "${ROUTE_NAME}" — ${end} ═══`;
    return h('figure', { class: 'xr-paste' },
        h('div', { class: 'xr-paste-code', 'aria-hidden': 'true' },
            h('div', {}, '#include …'),
            h('div', { class: 'sel' }, marker('BEGIN')),
            h('div', { class: 'sel mid' }, '… the old routes …'),
            h('div', { class: 'sel' }, marker('END')),
            h('div', {}, 'std::span<const TeamRoute> teamRoutes() …')),
        h('figcaption', {}, `In ${SKILLS_ROUTE_FILE}: select from the BEGIN line to the END line, both included, and paste.`));
}

/** Next (D4): build and upload for robot one or two, each command with Copy; how you know each worked; the one command. */
export function nextCard(names) {
    const windows = typeof navigator !== 'undefined' && /Windows/.test(navigator.userAgent);
    const list = h('ol', { class: 'xr-cmds' });
    const helper = h('div', { class: 'xr-helper' });
    const row = (n, ...content) => h('li', {}, h('span', { class: 'xr-num' }, String(n)), h('div', {}, content));
    const cmd = (text, what, worked) => [h('div', { class: 'xr-cmd' }, h('code', { 'data-cmd': what }, text), copyButton(text, what)),
                                         h('small', {}, worked)];
    const seg = h('div', { class: 'seg', role: 'group', 'aria-label': 'Which robot' });
    const show = (which) => {
        const c = robotCommands(which, { windows });
        seg.querySelectorAll('button').forEach((b) => b.setAttribute('aria-pressed', String(b.dataset.robot === which)));
        seg.querySelectorAll('button').forEach((b) => b.classList.toggle('on', b.dataset.robot === which));
        list.replaceChildren(
            row(1, cmd(c.build, 'the build command', 'It worked when make ends with no line saying "error".')),
            row(2, cmd(c.upload, 'the upload command', 'It worked when pros says "Finished uploading". Plug in the brain (USB) first.')),
            row(3, h('div', {}, 'On the brain: ', h('strong', {}, `${TESTER.name} (slot ${TESTER.slot})`), ' → ',
                     h('strong', {}, names.length === 1 ? names[0] : `one of: ${names.join(', ')}`), ' → ', h('strong', {}, 'RUN'))));
        helper.replaceChildren(h('span', {}, 'Or all of it in one command, which says plainly ✓ or ✗ (it needs Python 3, from python.org; without it, use 1 and 2):'),
            h('div', { class: 'xr-cmd' }, h('code', { 'data-cmd': 'the one command' }, c.helper), copyButton(c.helper, 'the one command')));
    };
    for (const [which, label] of [['one', 'Robot one'], ['two', 'Robot two']]) {
        seg.append(h('button', { type: 'button', 'data-robot': which, onclick: () => show(which) }, label));
    }
    show('one');
    return h('section', { class: 'card xr-next', 'aria-labelledby': 'xr-next-title' },
        h('div', { class: 'card-head' }, h('h3', { id: 'xr-next-title' }, 'Next: build and upload'), seg),
        h('p', { class: 'muted' }, 'In the PROS terminal, in your shulib folder (VS Code: PROS, then Integrated Terminal):'),
        list, helper);
}
