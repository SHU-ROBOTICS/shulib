// controls_tab.mjs — THE CONTROLS TAB (RMP1-B; RMP1-D: one step at a time): 1 · Buttons — the V5 controller drawn
// large, one editor beside it for the button clicked — and 2 · Driving feel (feel_tab.mjs). Export writes the driver
// file's three regions. The map is read from the driver file (G4); every refusal is the build's, in plain words.

import { h, s, icon, mount } from './dom.mjs';
import { fmt, parseTyped } from '../model/numbers.mjs';
import { BUTTONS, buttonsOf, actionsFor, mechOf, checkControls, addControl, editControl, removeControl, followDriving,
         checkMap, checkWords, pageWords, ownWords, removeFreeLine, toSaved, fromSaved, FEEL_REFUSAL } from '../model/controls.mjs';
import { checkFeel, editDriving, drivingFromSaved } from '../model/feel.mjs';
import { readDriverFile, driverFilePath, regionKey, REGIONS } from '../model/driver_file.mjs';
import { historyOf, commit, undo as undoH, redo as redoH, canUndo, canRedo } from '../model/undo.mjs';
import { renderFeel } from './feel_tab.mjs';
import { openControlsExport } from './controls_export.mjs';
import { robotName } from '../model/robot_names.mjs';

const KIND = { arm: 'arm', lift: 'lift', power: 'motor', pneumatic: 'piston', flywheel: 'flywheel' };
const PARTNER = { L1: 'L2', L2: 'L1', R1: 'R2', R2: 'R1', Up: 'Down', Down: 'Up', Left: 'Right', Right: 'Left', X: 'B', B: 'X', Y: 'A', A: 'Y' };

/** Where each button is drawn (viewBox 0 0 1000 462), and its label's side and height (one slot each, never shared). */
const PAD = {
    L2: { x: 330, y: 34, w: 110, h: 30, side: 'L', ly: 34 }, L1: { x: 330, y: 94, w: 110, h: 30, side: 'L', ly: 94 },
    R2: { x: 670, y: 34, w: 110, h: 30, side: 'R', ly: 34 }, R1: { x: 670, y: 94, w: 110, h: 30, side: 'R', ly: 94 },
    Up: { x: 346, y: 252, r: 19, side: 'L', ly: 234 }, Left: { x: 308, y: 290, r: 19, side: 'L', ly: 298 },
    Right: { x: 384, y: 290, r: 19, side: 'L', ly: 362 }, Down: { x: 346, y: 328, r: 19, side: 'L', ly: 426 },
    X: { x: 654, y: 252, r: 19, side: 'R', ly: 234 }, A: { x: 692, y: 290, r: 19, side: 'R', ly: 298 },
    Y: { x: 616, y: 290, r: 19, side: 'R', ly: 362 }, B: { x: 654, y: 328, r: 19, side: 'R', ly: 426 },
};
const EDGE = { L: 214, R: 786 };  // the label columns' inner edges
const HIT = 26;  // a button's invisible hit radius (half-height for a shoulder): at least 32 px at the page's narrowest

/** The d-pad's arrows (a triangle around the button's centre). */
const ARROW = { Up: [[0, -8], [-7, 5], [7, 5]], Down: [[0, 8], [-7, -5], [7, -5]], Left: [[-8, 0], [5, -7], [5, 7]], Right: [[8, 0], [-5, -7], [-5, 7]] };

/** The tab (header): `app` is the shell's (project, files, toasts, the baked robots). */
export function createControlsTab(app) {
    const baked = app.baked;
    const ids = Object.keys(baked.robots);
    const meta = {};
    const ui = { robot: 'one', sel: null, pick: null, step: 'buttons', more: false, words: false, axis: 'forward', probe: 0.3 };
    const el = {
        root: h('div', { class: 'controls', id: 'controls', hidden: true }), top: h('div', { class: 'ctl-top' }),
        body: h('div', { class: 'ctl-body', id: 'ctl-body', role: 'tabpanel' }), buttons: h('div', { class: 'ctl-buttons' }),
        pad: h('section', { class: 'card pad-card' }), editor: h('aside', { class: 'ctl-editor', 'aria-label': 'The button clicked', 'aria-live': 'polite' }),
    };
    el.buttons.append(el.pad, el.editor);
    el.root.append(el.top, el.body);  // mounted once: the scrollers keep their place across re-draws
    // Esc closes the editor wherever the keyboard is (a click on a label leaves it on the page), never under a drawer
    document.addEventListener('keydown', (e) => e.key === 'Escape' && ui.sel && !el.root.hidden && ui.step === 'buttons'
        && !/INPUT|TEXTAREA/.test(document.activeElement.tagName) && !document.querySelector('.drawer, .dialog, .popover') && close());
    const fileState = {};
    // one history per robot: an undo always acts on the robot shown
    let hist = Object.fromEntries(ids.map((id) => [id, historyOf(load(id))]));
    let loaded = Object.fromEntries(ids.map((id) => [id, hist[id].present]));  // as read: an unchanged robot is not written into the project

    /** One robot's state from the project, else its driver file as built into the page, else nothing (G4). */
    function load(id) {
        const robot = baked.robots[id];
        const saved = app.project().robots && app.project().robots[id];
        const file = baked.driverFiles && baked.driverFiles[id];
        meta[id] = { problems: [], base: {}, source: 'none', locked: '' };
        const r = file ? readDriverFile(new TextDecoder().decode(Uint8Array.from(atob(file.base64), (ch) => ch.charCodeAt(0))), id) : null;
        fileState[id] = r && r.ok ? complete(r.state, id) : null;  // what the file's words were written for
        if (saved && saved.controls) {
            const c = fromSaved(saved.controls);
            if (c.ok && saved.driving) {
                meta[id].source = 'project';
                meta[id].base = baseFromExports(id);
                return { controls: c.state, driving: drivingFromSaved(saved.driving) };
            }
            meta[id].locked = `src/team/route_maker.json's controls for ${robotName(id)} cannot be read here (${c.words || 'no driving block'}): `
                + 'open it in the newest route builder. Below is the driver file, read only.';
        }
        if (r && r.ok) {
            meta[id] = { ...meta[id], problems: r.problems, base: r.hashes, source: 'baked' };
            return fileState[id];
        }
        if (r) {
            meta[id].problems = [r.words];
        }
        return complete({ controls: { list: [] }, driving: { mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' } }, id);
    }
    function baseFromExports(id) {
        const ex = app.project().exports || {};
        return Object.fromEntries(REGIONS.map((r) => [r, ex[regionKey(id, r)]]));
    }
    /** Map rows for what the file did not say (a map it could not read): the page's plain words. */
    function complete(st, id) {
        let c = st.controls;
        if (!c.banner) {
            c = followDriving(c, null, st.driving);
            c = { ...c, list: c.list.map((x) => (x.banner ? x : editControl({ list: [x] }, 0, {}, baked.robots[id]).list[0])) };
        }
        return { ...st, controls: c };
    }

    const robot = () => baked.robots[ui.robot];
    const state = () => hist[ui.robot].present;
    const locked = () => !!meta[ui.robot].locked;
    let memo = { st: null, out: [] };
    /** Every problem of the robot shown (worked out once per state: every pane reads it). */
    function issues() {
        const st = state();
        if (memo.st !== st) {
            const out = checkControls(st.controls.list, robot(), { rezero: st.driving.rezeroButton });
            for (const f of checkFeel(st.driving, { holonomic: !(robot().drive && robot().drive.tank), defaultOnly: !!baked.feelDefaultOnly })) {
                out.push({ ...f, at: -2, refusal: f.refusal || (f.level === 'error' ? FEEL_REFUSAL : undefined) });
            }
            memo = { st, out: [...out, ...checkMap(st.controls, st.driving.rezeroButton), ...checkWords(st.controls, fileState[ui.robot] && fileState[ui.robot].controls, robot(), loaded[ui.robot].controls)] };
        }
        return memo.out;
    }
    /** One edit (one undo entry): `fn(state)` -> the next state of the robot shown. A map line calling a button free
     *  that this edit takes goes in the same edit (it would lie). Returns (and says) what went, '' when nothing did;
     *  null when the robot is read only (the refusal is the toast). */
    function edit(fn, sel) {
        if (locked()) {
            app.toast(meta[ui.robot].locked, 'error');
            return null;
        }
        const stale = (st) => checkMap(st.controls, st.driving.rezeroButton).map((x) => [x.free, st.controls.banner.free[x.free]]);
        const was = new Set(stale(state()).map(([, r]) => r));
        let next = fn(state());
        const gone = stale(next).filter(([, r]) => !was.has(r));
        for (const [k] of [...gone].reverse()) {
            next = { ...next, controls: removeFreeLine(next.controls, k) };
        }
        hist = { ...hist, [ui.robot]: commit(hist[ui.robot], next) };
        if (sel !== undefined) {
            ui.sel = sel;
        }
        keep();
        render();
        const n = gone.length;
        const said = n ? `The button map's ${gone.map(([, r]) => `"${r.key}"`).join(' and ')} line${n > 1 ? 's' : ''} said free, so ${n > 1 ? 'they' : 'it'} went too (Ctrl+Z undoes both).` : '';
        if (said) {
            app.toast(said);
        }
        return said;
    }
    /** The robots' controls into the project (G3a's shape, under robots.<id>) — only robots the student changed. A
     *  robot taken in for the first time brings the hashes its driver file was READ at, so a re-opened project
     *  still knows the file's untouched parts from a teammate's hand edit. */
    function keep() {
        const robots = { ...(app.project().robots || {}) };
        const exports = { ...(app.project().exports || {}) };
        for (const id of ids) {
            const st = hist[id].present;
            if (meta[id].locked || (st === loaded[id] && !(robots[id] && robots[id].controls))) {
                continue;
            }
            if (!(robots[id] && robots[id].controls)) {
                for (const r of REGIONS) {
                    if (meta[id].base[r] && !exports[regionKey(id, r)]) {
                        exports[regionKey(id, r)] = meta[id].base[r];
                    }
                }
            }
            robots[id] = { ...(robots[id] || {}), controls: toSaved(st.controls), driving: st.driving };
        }
        app.setRobots(robots, exports);
    }

    // ── the controller ──
    function jobOf(b) {
        const st = state();
        const i = st.controls.list.findIndex((c) => buttonsOf(c).includes(b));
        if (i < 0) {
            return null;
        }
        const c = st.controls.list[i];
        const m = mechOf(robot(), c.mechanism);
        const name = m ? m.name : c.mechanism;
        const pos = m && (m.kind === 'arm' || m.kind === 'lift');
        const word = c.form === 'whileHeld' ? (!c.back ? 'hold to run' : b === c.button ? (pos ? 'up (hold)' : 'forward (hold)') : (pos ? 'down (hold)' : 'back (hold)'))
            : c.form === 'toggle' ? 'toggle' : c.form === 'preset' ? `to ${typeof c.preset === 'number' ? `level ${c.preset}` : c.preset}` : 'home';
        return { i, c, name, word, text: `${name}: ${word}` };
    }
    function controller() {
        const st = state();
        const errs = issues().filter((x) => x.level === 'error');
        const rezero = st.driving.rezeroButton;
        const under = [s('path', { class: 'pad-body', d: 'M380 140 H620 Q690 140 712 200 L750 332 Q764 398 702 408 Q660 412 634 372 L606 334 H394 L366 372 Q340 412 298 408 Q236 398 250 332 L288 200 Q310 140 380 140 Z' }),
            s('rect', { class: 'pad-screen', x: 454, y: 156, width: 92, height: 54, rx: 6 })];
        const over = [];
        const mode = st.driving.mode || 'arcade';
        const STICKS = { arcade: ['drive', 'turn'], tankSticks: ['left side', 'right side'], arcadeField: ['drive (field)', 'turn'] };
        const sticks = STICKS[mode] || STICKS.arcade;  // an unknown mode is the feel's error, never a broken drawing
        for (const [x, side, word] of [[400, 'L', sticks[0]], [600, 'R', sticks[1]]]) {
            const toFeel = () => step('feel');
            over.push(s('g', { class: 'stick', role: 'button', tabindex: 0, 'aria-label': `${side === 'L' ? 'Left' : 'Right'} stick: ${word} (the driving feel)`,
                              onclick: toFeel, onkeydown: key(toFeel) },
                s('circle', { cx: x, cy: 228, r: 38, class: 'stick-base' }), s('circle', { cx: x, cy: 228, r: 20, class: 'stick-knob' })));
            over.push(callout(side, 166, x, 228, [`${side === 'L' ? 'left' : 'right'} stick`, word], 'stick', toFeel, under));
        }
        for (const b of BUTTONS) {
            const p = PAD[b];
            const job = jobOf(b);
            const bad = errs.some((x) => x.button === b || (job && x.at === job.i));
            const sel = isSel(b, job) ? ' sel' : '';
            const kind = job ? (bad ? 'used bad' : 'used') : b === rezero ? 'rezero' : 'free';
            const shape = p.w ? s('rect', { x: p.x - p.w / 2, y: p.y - p.h / 2, width: p.w, height: p.h, rx: 8 }) : s('circle', { cx: p.x, cy: p.y, r: p.r });
            const words = job ? job.text : b === rezero ? 're-zero (the feel)' : 'free';
            const arrow = ARROW[b] && s('path', { class: 'cbtn-arrow', d: ARROW[b].map(([dx, dy], k) => `${k ? 'L' : 'M'}${p.x + dx} ${p.y + dy}`).join(' ') + 'Z' });
            const hit = p.w ? s('rect', { class: 'cbtn-hit', x: p.x - p.w / 2, y: p.y - HIT, width: p.w, height: 2 * HIT })
                : s('circle', { class: 'cbtn-hit', cx: p.x, cy: p.y, r: HIT });
            over.push(s('g', { class: `cbtn ${job ? 'used' : b === rezero ? 'rezero' : 'free'}${bad ? ' bad' : ''}${sel}`, role: 'button', tabindex: 0, 'data-button': b,
                              'aria-label': `${b}: ${words}${bad ? ' (has an error)' : ''}`, onclick: () => press(b), onkeydown: key(() => press(b, true)) },
                s('title', {}, `${b}: ${words}${job ? '' : b === rezero ? '' : ' (click to give it a job)'}`),
                hit, shape, arrow || s('text', { x: p.x, y: p.y + 0.5, class: 'cbtn-name' }, b)));
            const lines = job ? [`${b} · ${job.name}`, job.word] : b === rezero ? [`${b} · re-zero`, 'driving feel'] : [`${b} · free`];
            over.push(callout(p.side, p.ly, p.x, p.y, lines, kind + sel, () => press(b), under, b));
        }
        return s('svg', { class: 'pad', viewBox: '0 0 1000 462', role: 'group', 'aria-label': `${robotName(ui.robot)}'s controller: click a button` }, under, over);
    }
    /** A label: one or two lines in a pill at its side's column, its leader (drawn under the buttons) to (tx, ty). */
    function callout(side, y, tx, ty, lines, kind, onclick, under, b) {
        const shown = lines.map((t) => (t.length > 20 ? `${t.slice(0, 19)}…` : t));
        const x = side === 'L' ? EDGE.L - 14 : EDGE.R + 14;
        const hgt = shown.length * 21 + 12;
        const w = 28 + Math.max(...shown.map((t) => t.length)) * 10;  // re-measured once drawn (fitLabels)
        under.push(s('path', { class: `leader ${kind.split(' ').filter((k) => k === 'sel' || k === 'bad').join(' ')}`, d: `M${EDGE[side]} ${y} L${tx} ${ty}` }));
        return s('g', { class: `callout ${kind}`, 'aria-hidden': 'true', 'data-label': b || null, 'data-side': side, onclick },
            s('rect', { x: side === 'L' ? EDGE.L - w : EDGE.R, y: y - hgt / 2, width: w, height: hgt, rx: 12 }),
            s('text', { x, y: y - (shown.length - 1) * 10.5, 'text-anchor': side === 'L' ? 'end' : 'start' },
              shown.map((t, k) => s('tspan', { x, dy: k ? 21 : 0, class: k ? 'job' : 'name' }, t))));
    }
    /** Each label's pill to its words' drawn width (the font is the viewer's: measured, not guessed). */
    function fitLabels() {
        for (const g of el.pad.querySelectorAll('.callout')) {
            try {
                const box = g.querySelector('text').getBBox();
                const r = g.querySelector('rect');
                const left = g.dataset.side === 'L';
                const w = (left ? EDGE.L - box.x : box.x + box.width - EDGE.R) + 14;
                r.setAttribute('x', left ? EDGE.L - w : EDGE.R);
                r.setAttribute('width', w);
            } catch {
                // not drawn (hidden): the estimate stands
            }
        }
    }
    /** The control chosen: by its index, or by a button that has one ("Go there" on a stale map line names the button). */
    function selAt() {
        const sel = ui.sel || {};
        const job = sel.button && jobOf(sel.button);
        return sel.control !== undefined && state().controls.list[sel.control] ? sel.control : job ? job.i : -1;
    }
    const isSel = (b, job) => ui.sel && (ui.sel.button === b || (job && selAt() === job.i));
    const key = (f) => (e) => (e.key === 'Enter' || e.key === ' ') && (e.preventDefault(), f(e));

    function press(b, byKey = false) {
        const job = jobOf(b);
        select(job ? { control: job.i } : { button: b });
        if (byKey) {  // the keyboard into the editor (its first choice or field); Esc brings it back to the button
            el.editor.querySelector('.choice, .issue .btn, select, input, .btn:not(.icon)')?.focus();
        }
    }
    function select(sel) {
        ui.sel = sel && !sel.feel ? sel : null;
        ui.pick = null;
        if (sel && sel.feel) {
            ui.more = true;  // "Go there" on the feel: every feel setting in view
        }
        step(sel && sel.feel ? 'feel' : sel ? 'buttons' : ui.step);
        if (sel && !sel.feel && window.matchMedia('(max-width: 1260px)').matches) {
            el.editor.scrollIntoView({ block: 'nearest' });  // stacked under the controller on a narrow screen
        }
    }
    function step(id) {
        ui.step = id;
        render();
    }
    /** Close the editor; the keyboard goes back to the button it was for. */
    function close() {
        const c = ui.sel && ui.sel.control !== undefined ? state().controls.list[ui.sel.control] : null;
        const b = ui.sel && (ui.sel.button || (c && buttonsOf(c)[0]));
        ui.sel = null;
        ui.pick = null;
        render();
        if (b) {
            el.root.querySelector(`[data-button="${b}"]`)?.focus({ preventScroll: true });
        }
    }
    /** The jobs offered for `m` on `b`, each saying in full what it picks (G4: the student's click chooses the second
     *  button, the position or the speed — the page never fills one in). */
    function menuActions(b, m) {
        const st = state();
        const taken = new Set([...st.controls.list.flatMap(buttonsOf), st.driving.rezeroButton]);
        const pos = m.kind === 'arm' || m.kind === 'lift';
        const out = [];
        for (const a of actionsFor(m).filter((x) => x.form !== 'homeAtDriverStart' && x.form !== 'autonOnly')) {
            const p = PARTNER[b];
            if (a.form === 'whileHeld' && a.two) {
                out.push(!taken.has(p) ? { ...a, back: p, words: `${pos ? 'Move' : 'Run'} while held: ${b} ${pos ? 'up' : 'forward'}, ${p} ${pos ? 'down' : 'back'}` }
                    : { ...a, back: null, words: `${pos ? 'Move' : 'Run'} while held: ${b} ${pos ? 'up' : 'forward'}, a second button you pick` });
            } else if (m.kind === 'pneumatic' && a.form === 'preset') {
                out.push(...(m.positions || []).map((x) => ({ ...a, preset: x, words: `Go to "${x}" on one press` })));
            } else if (m.kind === 'flywheel' && (m.presets || []).length) {
                out.push(...m.presets.map((x) => ({ ...a, [a.form === 'toggle' ? 'speed' : 'preset']: x.name,
                    words: a.form === 'toggle' ? `Each press: spin up to "${x.name}" / stop` : `Spin up to "${x.name}" on one press` })));
            } else {
                out.push(a);
            }
        }
        return out;
    }
    /** Give button `b` the job `a` on `m`, exactly as its choice said it. */
    function assign(b, m, a) {
        const st = state();
        const c = { form: a.form, mechanism: m.ident, button: b };
        for (const k of ['back', 'preset', 'speed']) {
            if (a[k] !== undefined) {
                c[k] = a[k];
            }
        }
        const gone = edit((x) => ({ ...x, controls: addControl(x.controls, c, robot()) }), { control: st.controls.list.length });
        if (gone === null) {
            return;
        }
        app.toast(`${a.form === 'whileHeld' && a.two ? (c.back ? `Added: ${b} and ${c.back}. Change either, and type its volts, in the panel.` : 'Added: choose its second button and type its volts in the panel.')
            : a.form === 'whileHeld' ? 'Added: type its volts in the panel (12 is full power).' : 'Added: finish it in the panel.'}${gone ? ` ${gone}` : ''}`);
    }

    function controlTitle(c) {
        const b = buttonsOf(c);
        const what = { whileHeld: 'hold', toggle: 'toggle', preset: 'press: go to', homeOnPress: 'press: home', homeAtDriverStart: 'home at driver start', autonOnly: 'autons only' }[c.form];
        return `${b.length ? `${b.join(' / ')} · ` : ''}${what}${c.form === 'preset' ? ` ${typeof c.preset === 'number' ? `level ${c.preset}` : c.preset}` : ''}`;
    }

    // ── the editor beside the controller: ONE thing — the button clicked, else where to start ──
    function editor() {
        const st = state();
        const sel = ui.sel || {};
        const all = issues();
        const at = selAt();
        if (at >= 0) {
            const mine = buttonsOf(st.controls.list[at]);
            // a button doing two jobs is said on BOTH controls; a stale map line beside the control that took its button
            return controlForm(at, all.filter((x) => x.at === at || (x.button && mine.includes(x.button) && x.at !== -2)));
        }
        if (sel.mechanism && mechOf(robot(), sel.mechanism)) {
            return mechanismView(mechOf(robot(), sel.mechanism));
        }
        if (sel.button) {
            const b = sel.button;
            return b === st.driving.rezeroButton
                ? [head([b], `${b} is the re-zero button`),
                    h('p', {}, `It re-zeroes field arcade's heading (Driving feel, More settings). To give ${b} a mechanism, move the re-zero to a free button there first.`),
                    h('button', { class: 'btn', onclick: () => select({ feel: true }) }, 'Open Driving feel')]
                : freeView(b);
        }
        const ready = all.filter((x) => x.level === 'error').length === 0;
        return [h('h2', { class: 'insp-title' }, 'Give a button a job'),
            meta[ui.robot].locked ? h('div', { class: 'banner error' }, icon('error'), h('div', {}, meta[ui.robot].locked)) : null,
            h('p', { class: 'lead' }, 'Click a button on the controller, or its label.'),
            h('h3', { class: 'ed-sub' }, 'Your mechanisms'),
            h('ul', { class: 'mech-list' }, (robot().mechanisms || []).map((m) => {
                const keys = st.controls.list.filter((c) => c.mechanism === m.ident).flatMap(buttonsOf);
                const missing = all.some((x) => x.mechanism === m.ident && x.at === -1);
                return h('li', {}, h('button', { class: `mech-row${missing ? ' has-error' : ''}`, onclick: () => select({ mechanism: m.ident }), 'aria-label': `The ${m.name}` },
                    h('strong', {}, m.name), h('small', {}, KIND[m.kind] || m.kind),
                    h('span', { class: 'keys' }, keys.length ? keys.map(keycap) : missing ? 'no button yet' : 'no button')));
            })),
            noButtonJobs(),
            meta[ui.robot].problems.map((p) => h('div', { class: 'issue warn' }, h('span', { class: 'ic' }, icon('warn')), h('div', {}, `The driver file: ${p}.`))),
            ready ? h('div', { class: 'banner ok' }, icon('check'), h('div', {}, 'Ready to export: the build will take these controls.')) : null,
            all.filter((x) => x.code === 'mapStale' || x.code === 'noteStale').map((x) => issueBox(x)),
            h('p', { class: 'hint source' }, sourceWords())];
    }
    const keycap = (b) => h('kbd', { class: 'keycap' }, b);
    /** The editor's head: the button(s), a title, a line under it, and Close (Esc). */
    function head(keys, title, sub) {
        return h('div', { class: 'ed-head' }, keys.map(keycap),
            h('div', { class: 'ed-title' }, h('h2', { class: 'insp-title' }, title), sub ? h('p', { class: 'muted' }, sub) : null),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Close (Esc)', title: 'Close (Esc)', onclick: () => close() }, icon('close')));
    }
    /** A free button: pick the mechanism, then its job — each a large button saying in full what it picks (G4). */
    function freeView(b) {
        const m = ui.pick && mechOf(robot(), ui.pick);
        const pick = (id) => {
            ui.pick = id;
            render();
            el.editor.querySelector('.choice')?.focus({ preventScroll: true });
        };
        if (!m) {
            return [head([b], `${b} is free`, 'Which mechanism should it run?'),
                h('div', { class: 'choices' }, (robot().mechanisms || []).map((x) => h('button', { class: 'choice', onclick: () => pick(x.ident) },
                    h('strong', {}, x.name), h('small', {}, KIND[x.kind] || x.kind)))),
                h('button', { class: 'btn ghost', onclick: () => close() }, 'Keep it free')];
        }
        return [head([b], `${b} → the ${m.name}`, `What should ${b} do?`),
            h('div', { class: 'choices', role: 'group', 'aria-label': `The ${m.name}'s jobs` }, menuActions(b, m).map((a) => h('button', {
                class: 'choice', 'aria-label': `${m.name}: ${a.words}`, onclick: () => assign(b, m, a) }, a.words))),
            h('button', { class: 'btn ghost', onclick: () => pick(null) }, icon('backup'), 'Another mechanism')];
    }
    /** A mechanism (from the list, or Export's "Go there"): its jobs, and the jobs that need no button. */
    function mechanismView(m) {
        const st = state();
        const mine = st.controls.list.map((c, i) => [c, i]).filter(([c]) => c.mechanism === m.ident);
        const free = BUTTONS.filter((b) => !jobOf(b) && b !== st.driving.rezeroButton);
        return [head([], `The ${m.name}`, KIND[m.kind] || m.kind),
            mine.length ? h('div', { class: 'row' }, mine.map(([c, i]) => h('button', { class: 'chip-btn', onclick: () => select({ control: i }) }, controlTitle(c))))
                : h('p', { class: 'inline-err' }, 'No button yet.'),
            h('p', { class: 'hint' }, `To give it a button, click a free one on the controller — free now: ${free.join(', ') || 'none'}.`),
            h('h3', { class: 'ed-sub' }, 'A job with no button'),
            h('div', { class: 'choices' }, actionsFor(m).filter((a) => a.form === 'homeAtDriverStart' || a.form === 'autonOnly').map((a) => h('button', { class: 'choice', onclick: () => {
                edit((x) => ({ ...x, controls: addControl(x.controls, { form: a.form, mechanism: m.ident }, robot()) }), { control: st.controls.list.length });
            } }, a.words)))];
    }
    function sourceWords() {
        const src = meta[ui.robot].source;
        return src === 'project' ? 'These are your project file\'s controls (src/team/route_maker.json).'
            : src === 'baked' ? `Read from ${driverFilePath(ui.robot)} as this route builder was built (${baked.build.commit}, ${baked.build.date}). `
                + (app.files().connected() ? meta[ui.robot].folder || ''
                    : app.files().canWriteFolder ? 'Connect your shulib folder to read today\'s.' : 'If your team changed it since, Export checks the file you pick against this.')
                : src === 'folder' ? `Read from ${driverFilePath(ui.robot)} in your folder.`
                    : 'No driver file was found: every mechanism needs a job (or "autons only").';
    }
    function issueBox(x) {
        const fixes = x.code === 'mapStale'
            ? [h('button', { class: 'btn', onclick: () => edit((y) => ({ ...y, controls: removeFreeLine(y.controls, x.free) })) }, 'Remove that line')]
            : x.code === 'wordsStale' ? [h('button', { class: 'btn', onclick: () => edit((y) => ({ ...y, controls: pageWords(y.controls, x.at, robot(), x) })) }, 'Use the page\'s words')] : [];
        return h('div', { class: `issue ${x.level}`, role: x.level === 'error' ? 'alert' : 'status' },
            h('span', { class: 'ic' }, icon(x.level === 'error' ? 'error' : 'warn')),
            h('div', {}, x.words, x.refusal ? h('span', { class: 'lib', title: 'The build\'s own words' }, `the build: ${x.refusal}`) : null,
              fixes.length ? h('div', { class: 'fixes' }, fixes) : null));
    }
    function controlForm(i, mine) {
        const st = state();
        const c = st.controls.list[i];
        const m = mechOf(robot(), c.mechanism);
        const set = (patch) => {
            const gone = edit((x) => ({ ...x, controls: editControl(x.controls, i, patch, robot()) }));
            const after = state().controls.list[i];
            const lost = after && !('words' in patch) && !('banner' in patch)
                && ['words', 'banner'].filter((p) => ownWords(c, m, p) && !ownWords(after, mechOf(robot(), after.mechanism), p));
            if (lost && lost.length) {
                app.toast(`${lost.includes('words') ? 'Its job changed, so its words in the file are the page\'s again (Ctrl+Z brings yours back).'
                    : 'Your map rows could not follow it to its new buttons, so they are the page\'s again (Ctrl+Z brings yours back).'}${gone ? ` ${gone}` : ''}`);
            }
        };
        const btnSel = (label, value, field, pick) => selectField(label, value ?? '', [...(pick || value === undefined ? [['', 'Choose one']] : []), ...BUTTONS.map((b) => {
            const j = jobOf(b);
            return [b, `${b}${b === st.driving.rezeroButton ? ' (the re-zero)' : j && j.i !== i ? ' (taken)' : ''}`];
        })], (v) => set({ [field]: v || (field === 'back' ? null : undefined) }), field);
        const body = [selectField('Mechanism', c.mechanism, (robot().mechanisms || []).map((x) => [x.ident, `${x.name} (${KIND[x.kind] || x.kind})`]),
            (v) => set({ mechanism: v }), 'mechanism')];
        const acts = actionsFor(m);
        const cur = c.form === 'whileHeld' ? (c.back !== undefined ? 'whileHeld2' : 'whileHeld1') : c.form;
        body.push(selectField('What it does', cur, acts.map((a) => [a.form === 'whileHeld' ? (a.two ? 'whileHeld2' : 'whileHeld1') : a.form, a.words]),
            (v) => set(v === 'whileHeld2' ? { form: 'whileHeld', back: c.back || null }
                : v === 'whileHeld1' ? { form: 'whileHeld', back: undefined }
                    : { form: v, back: undefined, volts: undefined, button: v === 'homeAtDriverStart' || v === 'autonOnly' ? undefined : c.button }), 'form'));
        if (c.form === 'whileHeld') {
            const two = c.back !== undefined;
            body.push(h('div', { class: 'field-row' }, btnSel(two ? (m && (m.kind === 'arm' || m.kind === 'lift') ? 'Up button' : 'Forward button') : 'Button', c.button, 'button', two),
                two ? btnSel(m && (m.kind === 'arm' || m.kind === 'lift') ? 'Down button' : 'Back button', c.back, 'back', true) : null));
            body.push(numField('Volts while held', c.volts, 'V', (v) => set({ volts: v }), 'volts', '12 is full power'));
        } else if (c.form !== 'homeAtDriverStart' && c.form !== 'autonOnly') {
            body.push(btnSel('Button', c.button, 'button'));
        }
        if (c.form === 'preset' || (c.form === 'toggle' && m && m.kind === 'flywheel')) {
            const field = c.form === 'preset' ? 'preset' : 'speed';
            const opts = m && m.kind === 'pneumatic' ? (m.positions || []).map((p) => [`N${p}`, p])
                : (m && m.presets || []).map((p) => (p.ledger !== undefined ? [`L${p.ledger}`, `level ${p.ledger}: "${p.name}"`] : [`N${p.name}`, `"${p.name}"`]));
            const v = c[field];
            body.push(selectField(m && m.kind === 'pneumatic' ? 'Position' : m && m.kind === 'flywheel' ? 'Speed' : 'Level',
                v === undefined ? '' : typeof v === 'number' ? `L${v}` : `N${v}`, [['', opts.length ? 'Choose one' : 'it has none yet'], ...opts],
                (x) => set({ [field]: x === '' ? undefined : x[0] === 'L' ? Number(x.slice(1)) : x.slice(1) }), field));
        }
        if ((c.form === 'preset' && m && m.kind !== 'pneumatic') || c.form === 'homeOnPress' || c.form === 'homeAtDriverStart') {
            body.push(numField('Time limit', c.timeLimitS, 's', (v) => set({ timeLimitS: v }), 'timeLimitS', 'the most it may take: type it'));
        }
        body.push(h('details', { class: 'section fold', open: ui.words || mine.some((x) => x.code === 'wordsStale'), ontoggle: (e) => { ui.words = e.target.open; } },
            h('summary', { onclick: (e) => { ui.words = !e.currentTarget.parentElement.open; } }, 'Its words in the file'),  // at once: a re-draw may come before 'toggle'
            textField('Comment on its line', c.words || '', (v) => set({ words: v || undefined }), 'words'),
            (c.banner || []).map((r, k) => h('label', { class: 'fld' }, h('span', {}, `In the button map: "${r.key}"`),
                h('textarea', { class: 'txt', rows: Math.max(2, r.lines.reduce((n, l) => n + Math.ceil(l.length / 38), 0)), name: `banner${k}`, onchange: (e) => {
                    const lines = e.target.value.split('\n').map((l) => l.trim()).filter(Boolean);
                    set({ banner: c.banner.map((x, j) => (j === k ? { ...x, lines: lines.length ? lines : x.lines } : x)) });
                } }, r.lines.join('\n'))))));
        const keys = buttonsOf(c);
        const job = acts.find((a) => (a.form === 'whileHeld' ? (a.two ? 'whileHeld2' : 'whileHeld1') : a.form) === cur);
        return [head(keys, m ? m.name : c.mechanism, job ? job.words : controlTitle(c)),
            mine.map((x) => issueBox(x)), body,
            h('div', { class: 'section row' }, h('button', { class: 'btn danger', onclick: () => {
                if (edit((x) => ({ ...x, controls: removeControl(x.controls, i) }), null) === null) {
                    return;
                }
                app.toast(`${keys.length ? `${keys.join(' and ')} ${keys.length > 1 ? 'are' : 'is'} free now` : 'Job removed'} (Ctrl+Z brings it back).`);
            } }, icon('trash'), keys.length ? `Make ${keys.join(' and ')} free` : 'Remove this job'))];
    }
    function selectField(label, value, options, onSet, name) {
        return h('label', { class: 'fld' }, h('span', {}, label),
            h('select', { class: 'sel', name, 'aria-label': label, onchange: (e) => onSet(e.target.value) },
              options.map(([v, t]) => h('option', { value: v, selected: String(v) === String(value) }, t))));
    }
    function numField(label, value, unit, onSet, name, placeholder = '') {
        const wrap = h('div', { class: `unit${Number.isFinite(value) ? '' : ' bad'}` });
        wrap.append(h('input', { type: 'text', inputmode: 'decimal', name, 'aria-label': `${label} (${unit})`, placeholder, value: Number.isFinite(value) ? fmt(value) : '',
            onchange: (e) => {
                const t = e.target.value.trim();
                const v = t === '' ? undefined : parseTyped(t);
                if (v !== undefined && Number.isNaN(v)) {
                    wrap.classList.add('bad');
                    app.toast(`Not a number: type digits, like 2.5 (in ${unit}).`, 'error');
                    return;
                }
                onSet(v);
            }, onkeydown: (e) => e.key === 'Enter' && e.target.blur() }), h('em', {}, unit));
        return h('label', { class: 'fld' }, h('span', {}, label), wrap);
    }
    function textField(label, value, onSet, name) {
        return h('label', { class: 'fld' }, h('span', {}, label),
            h('input', { class: 'txt', type: 'text', name, value, maxlength: 90, onchange: (e) => onSet(e.target.value.trim()) }));
    }

    // ── the feel (feel_tab.mjs draws it; its edits come back here) ──
    const feelApi = {
        driving: () => state().driving, robot, ui, locked, toast: (t, c) => app.toast(t, c),
        issues: () => issues().filter((x) => x.at === -2),
        free: () => BUTTONS.filter((b) => !jobOf(b)),
        preview: () => render(),  // a slider moving: redraw, nothing kept yet
        set(patch) {
            edit((x) => {
                const driving = editDriving(x.driving, patch);
                return { controls: followDriving(x.controls, x.driving, driving), driving };
            });
        },
    };

    // ── the shell's calls ──
    function render() {
        if (el.root.hidden) {
            return;
        }
        const a = document.activeElement;
        const again = a && el.root.contains(a) && (a.dataset.button ? `[data-button="${a.dataset.button}"]` : a.name ? `[name="${a.name}"]`
            : a.id ? `#${a.id}` : null);
        const steps = [['buttons', '1', 'Buttons'], ['feel', '2', 'Driving feel']];
        mount(el.top,
            h('div', { class: 'steps-switch', role: 'tablist', 'aria-label': 'The Controls tab, step by step', onkeydown: (e) => {
                const k = { ArrowRight: 1, ArrowLeft: -1 }[e.key];
                if (k) {
                    step(steps[(steps.findIndex(([id]) => id === ui.step) + k + steps.length) % steps.length][0]);
                    el.top.querySelector('[aria-selected="true"]')?.focus();
                }
            } }, steps.map(([id, n, t]) => h('button', { role: 'tab', id: `ctl-step-${id}`, 'aria-selected': String(ui.step === id), 'aria-controls': 'ctl-body',
                tabindex: ui.step === id ? 0 : -1, class: ui.step === id ? 'on' : '', 'aria-label': `Step ${n}: ${t}`, onclick: () => step(id) }, h('span', { class: 'num' }, n), t))),
            h('span', { class: 'spacer' }),
            ids.length > 1 ? h('div', { class: 'robot-pick' }, h('span', { class: 'muted' }, 'Robot'), h('div', { class: 'seg', role: 'radiogroup', 'aria-label': 'Robot' }, ids.map((id) => h('button', {
                role: 'radio', 'aria-checked': String(ui.robot === id), class: ui.robot === id ? 'on' : '',
                onclick: () => { ui.robot = id; ui.sel = null; ui.pick = null; render(); app.changed(); } }, robotName(id))))) : null);
        el.body.setAttribute('aria-labelledby', `ctl-step-${ui.step}`);
        if (ui.step === 'feel') {
            mount(el.body, renderFeel(feelApi));
        } else {
            if (el.body.firstChild !== el.buttons) {
                mount(el.body, el.buttons);
            }
            mount(el.pad, h('div', { class: 'card-head' }, h('h2', {}, `${robotName(ui.robot)}'s controller`),
                h('span', { class: 'hint' }, 'Click a button to give it a job.'),
                h('span', { class: 'legend', 'aria-hidden': 'true' }, h('i', { class: 'used' }), 'has a job', h('i', { class: 'free' }), 'free')), controller());
            el.pad.setAttribute('aria-label', `${robotName(ui.robot)}'s controller`);
            mount(el.editor, editor());
            fitLabels();
        }
        if (again) {
            el.root.querySelector(again)?.focus({ preventScroll: true });  // a re-draw never drops the keyboard's place
        }
    }
    /** The jobs with no button, each one plain sentence (click it to change it). */
    function noButtonJobs() {
        const list = state().controls.list.map((c, i) => [c, i]).filter(([c]) => !buttonsOf(c).length);
        return list.length ? h('div', { class: 'no-button' }, list.map(([c, i]) => {
            const name = mechOf(robot(), c.mechanism)?.name || c.mechanism;
            return h('button', { class: 'link-line', 'data-control': i, onclick: () => select({ control: i }) }, c.form === 'autonOnly'
                ? `The ${name} runs in autons only: no button in driver control.`
                : `The ${name} homes by itself when driver control starts (${Number.isFinite(c.timeLimitS) ? `${fmt(c.timeLimitS)} s` : 'no time limit yet'}): no button needed.`);
        })) : null;
    }
    return {
        element: el.root,
        show(on) {
            el.root.hidden = !on;
            render();
        },
        render,
        issues: () => issues(),
        errors: () => issues().filter((x) => x.level === 'error'),
        undo() {
            hist = { ...hist, [ui.robot]: undoH(hist[ui.robot]) };
            keep();
            render();
        },
        redo() {
            hist = { ...hist, [ui.robot]: redoH(hist[ui.robot]) };
            keep();
            render();
        },
        canUndo: () => canUndo(hist[ui.robot]), canRedo: () => canRedo(hist[ui.robot]),
        openExport: () => openControlsExport({ app, robot: robot(), id: ui.robot, state: state(), issues: issues(), meta: meta[ui.robot],
                                                select, written: (hashes) => { meta[ui.robot].base = hashes; } }),
        folderConnected: readFolder,
        /** A project was opened: its robots' controls (or the driver files again, today's when a folder is connected).
         *  A robot changed here stays one Ctrl+Z away; with `keepChanged`, one the project has no controls for is kept.
         *  Returns what happened to the changes, in words ('' when there were none). */
        reload(keepChanged = false) {
            const out = { kept: [], back: [], lost: [] };
            for (const id of ids) {
                const mine = hist[id].present !== loaded[id];
                const theirs = app.project().robots && app.project().robots[id] && app.project().robots[id].controls;
                if (keepChanged && mine && !theirs) {
                    out.kept.push(robotName(id));
                    continue;
                }
                const next = load(id);
                const undoable = canUndo(hist[id]) && !meta[id].locked;
                hist = { ...hist, [id]: undoable ? commit(hist[id], next) : historyOf(next) };
                loaded = { ...loaded, [id]: next };
                if (mine) {
                    out[undoable ? 'back' : 'lost'].push(robotName(id));
                }
            }
            if (out.kept.length) {
                keep();
            }
            ui.sel = null;
            render();
            readFolder();
            return [out.kept.length ? `Your Controls changes for ${out.kept.join(' and ')} are kept (it has none of its own).` : '',
                    out.back.length ? `Its Controls replaced yours for ${out.back.join(' and ')}: Ctrl+Z on the Controls tab brings yours back.` : '',
                    out.lost.length ? `Its Controls for ${out.lost.join(' and ')} are read only here, so your changes there are gone.` : ''].filter(Boolean).join(' ');
        },
        snapshot: () => JSON.parse(JSON.stringify({ robot: ui.robot, state: state(), meta: meta[ui.robot], issues: issues() })),
    };
    /** The folder is connected: today's driver files are what the words are checked against, and a robot the project
     *  has not taken over (and not changed here) is read from them. */
    async function readFolder() {
        for (const id of ids) {
            const text = await app.files().readTeam(driverFilePath(id));
            const r = text === null ? null : readDriverFile(text, id);
            if (!(r && r.ok)) {
                meta[id].folder = r ? `Today's file in your folder was not read in (${r.words.replace(/\.$/, '')}).` : `Your folder has no ${driverFilePath(id)}.`;
                continue;
            }
            fileState[id] = complete(r.state, id);
            if (meta[id].source !== 'baked' && meta[id].source !== 'none') {
                continue;  // the project's
            }
            if (hist[id].present !== loaded[id]) {  // today's file never replaces an edit
                meta[id].folder = 'You changed it here, so today\'s file in your folder is not read in: Export asks before replacing anything that differs.';
                continue;
            }
            meta[id] = { ...meta[id], problems: r.problems, base: r.hashes, source: 'folder' };
            hist = { ...hist, [id]: { ...hist[id], present: fileState[id], future: [] } };  // unchanged here: today's file is it
            loaded = { ...loaded, [id]: fileState[id] };
        }
        memo = { st: null, out: [] };
        render();
    }
}
