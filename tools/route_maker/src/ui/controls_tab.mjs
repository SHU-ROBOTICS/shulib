// controls_tab.mjs — THE CONTROLS TAB (RMP1-B; the brief §1.2, §1.4, D9, D14): the V5 controller drawn in SVG —
// click a button to give it a job —, each mechanism and its controls on the left, the selected control's form on
// the right, DRIVING FEEL under the controller, and Export: the robot's driver file's three regions. The map is
// read from the driver file (G4: the page never invents one), every refusal is the build's, said in plain words.

import { h, s, icon, mount } from './dom.mjs';
import { fmt, parseTyped } from '../model/numbers.mjs';
import { BUTTONS, buttonsOf, actionsFor, mechOf, checkControls, addControl, editControl, removeControl, followDriving,
         checkMap, checkWords, pageWords, ownWords, removeFreeLine, toSaved, fromSaved, FEEL_REFUSAL } from '../model/controls.mjs';
import { checkFeel, editDriving, drivingFromSaved } from '../model/feel.mjs';
import { readDriverFile, driverFilePath, regionKey, REGIONS } from '../model/driver_file.mjs';
import { historyOf, commit, undo as undoH, redo as redoH, canUndo, canRedo } from '../model/undo.mjs';
import { renderFeel } from './feel_tab.mjs';
import { openControlsExport } from './controls_export.mjs';

const KIND = { arm: 'arm', lift: 'lift', power: 'motor', pneumatic: 'piston', flywheel: 'flywheel' };
const PARTNER = { L1: 'L2', L2: 'L1', R1: 'R2', R2: 'R1', Up: 'Down', Down: 'Up', Left: 'Right', Right: 'Left', X: 'B', B: 'X', Y: 'A', A: 'Y' };

/** Where each button is drawn, and which side its label sits on. */
const PAD = {
    L2: { x: 238, y: 38, w: 100, h: 26, side: 'L', ly: 38 }, L1: { x: 238, y: 90, w: 100, h: 26, side: 'L', ly: 90 },
    R2: { x: 662, y: 38, w: 100, h: 26, side: 'R', ly: 38 }, R1: { x: 662, y: 90, w: 100, h: 26, side: 'R', ly: 90 },
    Up: { x: 296, y: 224, r: 17, side: 'L', ly: 214 }, Left: { x: 258, y: 262, r: 17, side: 'L', ly: 258 },
    Right: { x: 334, y: 262, r: 17, side: 'L', ly: 302 }, Down: { x: 296, y: 300, r: 17, side: 'L', ly: 346 },
    X: { x: 604, y: 224, r: 17, side: 'R', ly: 214 }, Y: { x: 566, y: 262, r: 17, side: 'R', ly: 258 },
    A: { x: 642, y: 262, r: 17, side: 'R', ly: 302 }, B: { x: 604, y: 300, r: 17, side: 'R', ly: 346 },
};
const HIT = 26;  // a button's invisible hit radius (half-height for a shoulder): at least 32 px at the page's narrowest

/** The d-pad's arrows (a triangle around the button's centre). */
const ARROW = { Up: [[0, -7], [-6, 4], [6, 4]], Down: [[0, 7], [-6, -4], [6, -4]], Left: [[-7, 0], [4, -6], [4, 6]], Right: [[7, 0], [-4, -6], [-4, 6]] };

/** The tab (header): `app` is the shell's (project, files, toasts, the baked robots). */
export function createControlsTab(app) {
    const baked = app.baked;
    const ids = Object.keys(baked.robots);
    const meta = {};
    const ui = { robot: 'one', sel: null, axis: 'forward', probe: 0.3, pop: null };
    const el = {
        root: h('div', { class: 'controls', id: 'controls', hidden: true }),
        left: h('div', { class: 'pane-body' }), mid: h('div', { class: 'controls-mid' }), right: h('div', { class: 'pane-body', 'aria-live': 'polite' }),
    };
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
            meta[id].locked = `src/team/route_maker.json's controls for ${robot.name} cannot be read here (${c.words || 'no driving block'}): `
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
    /** One edit (one undo entry): `fn(state)` -> the next state of the robot shown. */
    function edit(fn, sel) {
        if (locked()) {
            app.toast(meta[ui.robot].locked, 'error');
            return;
        }
        const next = fn(state());
        hist = { ...hist, [ui.robot]: commit(hist[ui.robot], next) };
        if (sel !== undefined) {
            ui.sel = sel;
        }
        keep();
        render();
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
        return { i, c, text: `${name}: ${word}` };
    }
    function controller() {
        const st = state();
        const errs = issues().filter((x) => x.level === 'error');
        const rezero = st.driving.rezeroButton;
        const parts = [s('path', { class: 'pad-body', d: 'M330 112 H570 Q640 112 662 172 L700 304 Q714 370 652 380 Q610 384 584 344 L556 306 H344 L316 344 Q290 384 248 380 Q186 370 200 304 L238 172 Q260 112 330 112 Z' }),
            s('rect', { class: 'pad-screen', x: 404, y: 128, width: 92, height: 54, rx: 6 })];
        const mode = st.driving.mode || 'arcade';
        const STICKS = { arcade: ['drive', 'turn'], tankSticks: ['left side', 'right side'], arcadeField: ['drive (field)', 'turn'] };
        const sticks = STICKS[mode] || STICKS.arcade;  // an unknown mode is the feel's error, never a broken drawing
        for (const [x, side, word] of [[350, 'L', sticks[0]], [550, 'R', sticks[1]]]) {
            parts.push(s('g', { class: 'stick', role: 'button', tabindex: 0, 'aria-label': `${side === 'L' ? 'Left' : 'Right'} stick: ${word} (the driving feel)`,
                               onclick: () => select({ feel: true }), onkeydown: key(() => select({ feel: true })) },
                s('circle', { cx: x, cy: 200, r: 38, class: 'stick-base' }), s('circle', { cx: x, cy: 200, r: 20, class: 'stick-knob' })));
            parts.push(label(side, side === 'L' ? 150 : 750, 182, x + (side === 'L' ? -38 : 38), 200, `${side === 'L' ? 'left' : 'right'} stick: ${word}`, 'stick'));
        }
        for (const b of BUTTONS) {
            const p = PAD[b];
            const job = jobOf(b);
            const bad = errs.some((x) => x.button === b || (job && x.at === job.i));
            const cls = `cbtn ${job ? 'used' : b === rezero ? 'rezero' : 'free'}${bad ? ' bad' : ''}${isSel(b, job) ? ' sel' : ''}`;
            const shape = p.w ? s('rect', { x: p.x - p.w / 2, y: p.y - p.h / 2, width: p.w, height: p.h, rx: 8 }) : s('circle', { cx: p.x, cy: p.y, r: p.r });
            const words = job ? job.text : b === rezero ? 're-zero (the feel)' : 'free';
            const arrow = ARROW[b] && s('path', { class: 'cbtn-arrow', d: ARROW[b].map(([dx, dy], k) => `${k ? 'L' : 'M'}${p.x + dx} ${p.y + dy}`).join(' ') + 'Z' });
            const hit = p.w ? s('rect', { class: 'cbtn-hit', x: p.x - p.w / 2, y: p.y - HIT, width: p.w, height: 2 * HIT })
                : s('circle', { class: 'cbtn-hit', cx: p.x, cy: p.y, r: HIT });
            parts.push(s('g', { class: cls, role: 'button', tabindex: 0, 'data-button': b, 'aria-label': `${b}: ${words}${bad ? ' (has an error)' : ''}`,
                               onclick: () => press(b), onkeydown: key(() => press(b)) },
                s('title', {}, `${b}: ${words}${job ? '' : b === rezero ? '' : ' (click to give it a job)'}`),
                hit, shape, arrow || s('text', { x: p.x, y: p.y + 0.5, class: 'cbtn-name' }, b)));
            const lx = p.side === 'L' ? 150 : 750;
            parts.push(label(p.side, lx, p.ly, p.x + (p.side === 'L' ? -(p.w ? p.w / 2 : p.r) : (p.w ? p.w / 2 : p.r)), p.y, `${b} · ${words}`,
                             job ? (bad ? 'bad' : 'used') : b === rezero ? 'rezero' : 'free'));
        }
        return s('svg', { class: 'pad', viewBox: '-110 0 1120 420', role: 'group', 'aria-label': `${robot().name}'s controller: click a button` }, parts);
    }
    function label(side, x, y, tx, ty, text, kind) {
        const shown = text.length > 32 ? `${text.slice(0, 31)}…` : text;
        const w = 20 + shown.length * 7.6;
        const x0 = side === 'L' ? x - w : x;
        return s('g', { class: `callout ${kind}`, 'aria-hidden': 'true' },
            s('path', { class: 'leader', d: `M${x} ${y} L${tx} ${ty}` }),
            s('rect', { x: x0, y: y - 13, width: w, height: 26, rx: 13 }),
            s('text', { x: x0 + w / 2, y: y + 0.5 }, shown));
    }
    const isSel = (b, job) => ui.sel && ((ui.sel.button === b) || (job && ui.sel.control === job.i));
    const key = (f) => (e) => (e.key === 'Enter' || e.key === ' ') && (e.preventDefault(), f(e));

    function press(b) {
        const job = jobOf(b);
        select(job ? { control: job.i } : { button: b });
        if (!job && b !== state().driving.rezeroButton) {
            openPopover(b, el.root.querySelector(`[data-button="${b}"]`));  // the drawing was re-made: the new button
        }
    }
    function select(sel) {
        ui.sel = sel;
        closePopover();
        render();
        if (sel && sel.feel) {
            el.root.querySelector('.feel-card')?.scrollIntoView({ behavior: matchMedia('(prefers-reduced-motion: reduce)').matches ? 'auto' : 'smooth', block: 'start' });
        }
        if (sel && window.matchMedia('(max-width: 1100px)').matches) {
            el.root.dataset.right = 'open';
        }
    }

    /** A free button's popover: mechanism -> the actions its kind allows (§1.2). */
    function openPopover(b, anchor) {
        closePopover();
        const items = [h('h3', {}, `What should ${b} do?`)];
        for (const m of robot().mechanisms || []) {
            const acts = menuActions(b, m);
            items.push(h('div', { class: 'menu-group', role: 'group', 'aria-label': `The ${m.name}` },
                h('div', { class: 'menu-group-head' }, h('span', { class: 'mech-chip' }, m.name), h('small', {}, KIND[m.kind] || m.kind)),
                acts.map((a) => h('button', { class: 'menu-item', role: 'menuitem', 'aria-label': `${m.name}: ${a.words}`, onclick: () => assign(b, m, a) }, a.words))));
        }
        const pop = h('div', { class: 'popover', role: 'menu', 'aria-label': `Give ${b} a job` }, items);
        document.body.append(pop);
        // beside the button (right of it, else left), kept on the screen
        const r = anchor.getBoundingClientRect();
        const pw = pop.offsetWidth;
        const ph = pop.offsetHeight;
        const left = r.right + 12 + pw <= window.innerWidth - 8 ? r.right + 12 : r.left - 12 - pw;
        pop.style.left = `${Math.max(8, left)}px`;
        pop.style.top = `${Math.max(8, Math.min(window.innerHeight - ph - 8, r.top + r.height / 2 - 40))}px`;
        const outside = (e) => !pop.contains(e.target) && closePopover();
        const onKey = (e) => {
            if (e.key === 'Escape') {
                closePopover();
                el.root.querySelector(`[data-button="${b}"]`)?.focus();
            }
        };
        ui.pop = { pop, outside, onKey };
        setTimeout(() => {
            document.addEventListener('pointerdown', outside, true);
            document.addEventListener('keydown', onKey);
            pop.querySelector('.menu-item')?.focus();
        }, 0);
    }
    function closePopover() {
        if (ui.pop) {
            ui.pop.pop.remove();
            document.removeEventListener('pointerdown', ui.pop.outside, true);
            document.removeEventListener('keydown', ui.pop.onKey);
            ui.pop = null;
        }
    }
    /** The menu's jobs for `m` on `b`, each saying in full what it picks (G4: the student's click chooses the second
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
    /** Give button `b` the job `a` on `m`, exactly as the menu item said it. */
    function assign(b, m, a) {
        closePopover();
        const st = state();
        const c = { form: a.form, mechanism: m.ident, button: b };
        for (const k of ['back', 'preset', 'speed']) {
            if (a[k] !== undefined) {
                c[k] = a[k];
            }
        }
        edit((x) => ({ ...x, controls: addControl(x.controls, c, robot()) }), { control: st.controls.list.length });
        app.toast(a.form === 'whileHeld' && a.two ? (c.back ? `Added: ${b} and ${c.back}. Change either, and type its volts, on the right.` : `Added: choose its second button and type its volts on the right.`)
            : a.form === 'whileHeld' ? 'Added: type its volts on the right (12 is full power).' : 'Added: finish it on the right.');
    }

    // ── the left pane: each mechanism and its controls ──
    function mechanisms() {
        const st = state();
        const errs = issues();
        return h('ul', { class: 'mechs' }, (robot().mechanisms || []).map((m) => {
            const mine = st.controls.list.map((c, i) => [c, i]).filter(([c]) => c.mechanism === m.ident);
            const missing = errs.some((x) => x.mechanism === m.ident && x.at === -1);
            return h('li', { class: `mech-card${missing ? ' has-error' : ''}` },
                h('div', { class: 'mech-head' }, h('strong', {}, m.name), h('span', { class: 'chip' }, KIND[m.kind] || m.kind)),
                h('div', { class: 'row' }, mine.map(([c, i]) => h('button', { class: `chip-btn${ui.sel && ui.sel.control === i ? ' sel' : ''}`, onclick: () => select({ control: i }) },
                    controlTitle(c))),
                  mine.length ? null : h('span', { class: 'inline-err' }, 'no button yet'),
                  h('button', { class: 'btn ghost small', onclick: () => select({ mechanism: m.ident }), 'aria-label': `Add a job for the ${m.name}` }, icon('plus'), 'Add')));
        }));
    }
    function controlTitle(c) {
        const b = buttonsOf(c);
        const what = { whileHeld: 'hold', toggle: 'toggle', preset: 'press: go to', homeOnPress: 'press: home', homeAtDriverStart: 'home at driver start', autonOnly: 'autons only' }[c.form];
        return `${b.length ? `${b.join(' / ')} · ` : ''}${what}${c.form === 'preset' ? ` ${typeof c.preset === 'number' ? `level ${c.preset}` : c.preset}` : ''}`;
    }

    // ── the right pane ──
    function inspector() {
        const st = state();
        const sel = ui.sel || {};
        const all = issues();
        if (sel.control !== undefined && st.controls.list[sel.control]) {
            const mine = buttonsOf(st.controls.list[sel.control]);
            // a button doing two jobs is said on BOTH controls; a stale map line beside the control that took its button
            return controlForm(sel.control, all.filter((x) => x.at === sel.control || (x.button && mine.includes(x.button) && x.at !== -2)));
        }
        if (sel.mechanism) {
            const m = mechOf(robot(), sel.mechanism);
            const free = BUTTONS.filter((b) => !jobOf(b) && b !== st.driving.rezeroButton);
            return [h('h2', { class: 'insp-title' }, icon('mechanism'), `The ${m.name}`),
                h('p', { class: 'hint' }, `Click a free button on the controller to give the ${m.name} a job — free now: ${free.join(', ') || 'none'}. Or a job with no button:`),
                actionsFor(m).filter((a) => a.form === 'homeAtDriverStart' || a.form === 'autonOnly').map((a) => h('button', { class: 'menu-item', onclick: () => {
                    edit((x) => ({ ...x, controls: addControl(x.controls, { form: a.form, mechanism: m.ident }, robot()) }), { control: st.controls.list.length });
                } }, a.words))];
        }
        if (sel.button) {
            const b = sel.button;
            return b === st.driving.rezeroButton
                ? [h('h2', { class: 'insp-title' }, `Button ${b}`), h('p', {}, `${b} is the drive's re-zero button (Driving feel, below the controller). To give ${b} a mechanism, move the re-zero to a free button first.`)]
                : [h('h2', { class: 'insp-title' }, `Button ${b} is free`), h('p', { class: 'hint' }, 'Pick what it does in the menu by the button, or click it again.')];
        }
        const ready = all.filter((x) => x.level === 'error').length === 0;
        return [h('h2', { class: 'insp-title' }, icon('help'), `${robot().name}'s controls`),
            meta[ui.robot].locked ? h('div', { class: 'banner error' }, icon('error'), h('div', {}, meta[ui.robot].locked)) : null,
            h('p', {}, 'Click a button on the controller to see or change what it does. A free button gets a job from a short menu.'),
            h('p', { class: 'hint' }, sourceWords()),
            meta[ui.robot].problems.map((p) => h('div', { class: 'issue warn' }, h('span', { class: 'ic' }, icon('warn')), h('div', {}, `The driver file: ${p}.`))),
            ready ? h('div', { class: 'banner ok' }, icon('check'), h('div', {}, 'Ready to export: the build will take these controls.')) : null,
            all.filter((x) => x.code === 'mapStale' || x.code === 'noteStale').map((x) => issueBox(x))];
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
            edit((x) => ({ ...x, controls: editControl(x.controls, i, patch, robot()) }));
            const after = state().controls.list[i];
            const lost = after && !('words' in patch) && !('banner' in patch)
                && ['words', 'banner'].filter((p) => ownWords(c, m, p) && !ownWords(after, mechOf(robot(), after.mechanism), p));
            if (lost && lost.length) {
                app.toast(lost.includes('words') ? 'Its job changed, so its words in the file are the page\'s again (Ctrl+Z brings yours back).'
                    : 'Your map rows could not follow it to its new buttons, so they are the page\'s again (Ctrl+Z brings yours back).');
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
        body.push(h('div', { class: 'section' }, h('h3', {}, 'Its words in the file'),
            textField('Comment on its line', c.words || '', (v) => set({ words: v || undefined }), 'words'),
            (c.banner || []).map((r, k) => h('label', { class: 'fld' }, h('span', {}, `In the button map: "${r.key}"`),
                h('textarea', { class: 'txt', rows: Math.max(2, r.lines.reduce((n, l) => n + Math.ceil(l.length / 38), 0)), name: `banner${k}`, onchange: (e) => {
                    const lines = e.target.value.split('\n').map((l) => l.trim()).filter(Boolean);
                    set({ banner: c.banner.map((x, j) => (j === k ? { ...x, lines: lines.length ? lines : x.lines } : x)) });
                } }, r.lines.join('\n'))))));
        return [h('h2', { class: 'insp-title' }, icon('mechanism'), controlTitle(c)),
            h('p', { class: 'muted', style: 'margin-top:-8px' }, m ? `The ${m.name}` : c.mechanism),
            mine.map((x) => issueBox(x)), body,
            h('div', { class: 'section row' }, h('button', { class: 'btn danger', onclick: () => {
                edit((x) => ({ ...x, controls: removeControl(x.controls, i) }), null);
                app.toast('Control removed (Ctrl+Z brings it back).');
            } }, icon('trash'), 'Remove this control'))];
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
        const again = a && el.root.contains(a) && (a.dataset.button ? `[data-button="${a.dataset.button}"]` : a.name ? `[name="${a.name}"]` : null);
        mount(el.root,
            h('aside', { class: 'pane left', 'aria-label': 'Mechanisms' },
              h('div', { class: 'pane-head' }, h('h2', {}, 'Mechanisms'), h('span', { class: 'spacer' }),
                ids.length > 1 ? h('div', { class: 'seg', role: 'radiogroup', 'aria-label': 'Robot' }, ids.map((id) => h('button', {
                    role: 'radio', 'aria-checked': String(ui.robot === id), class: ui.robot === id ? 'on' : '',
                    onclick: () => { ui.robot = id; ui.sel = null; render(); app.changed(); } }, baked.robots[id].name.replace('robot ', '')))) : null),
              el.left),
            el.mid, h('aside', { class: 'pane right', 'aria-label': 'The selected control' },
              h('div', { class: 'pane-head' }, h('h2', {}, 'Control'), h('span', { class: 'spacer' }),
                h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Close the control', onclick: () => delete el.root.dataset.right }, icon('close'))),
              el.right));
        mount(el.left, mechanisms(), noButtonJobs());
        mount(el.mid, h('div', { class: 'card pad-card' }, h('div', { class: 'card-head' }, h('h2', {}, `${robot().name}'s controller`),
            h('span', { class: 'hint' }, 'Click a button. Labels show each one\'s job; dashed means free.')), controller()),
            renderFeel(feelApi));
        mount(el.right, inspector());
        if (again) {
            el.root.querySelector(again)?.focus({ preventScroll: true });  // a re-draw never drops the keyboard's place
        }
    }
    function noButtonJobs() {
        const list = state().controls.list.map((c, i) => [c, i]).filter(([c]) => !buttonsOf(c).length);
        return list.length ? h('div', { class: 'section' }, h('h3', {}, 'With no button'), list.map(([c, i]) => h('button', { class: 'chip-btn', onclick: () => select({ control: i }) },
            `${mechOf(robot(), c.mechanism)?.name || c.mechanism}: ${c.form === 'autonOnly' ? 'autons only' : `homes when driver control starts (${Number.isFinite(c.timeLimitS) ? `${fmt(c.timeLimitS)} s` : '? s'})`}`))) : null;
    }
    return {
        element: el.root,
        show(on) {
            el.root.hidden = !on;
            if (!on) {
                closePopover();
            }
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
                    out.kept.push(baked.robots[id].name);
                    continue;
                }
                const next = load(id);
                const undoable = canUndo(hist[id]) && !meta[id].locked;
                hist = { ...hist, [id]: undoable ? commit(hist[id], next) : historyOf(next) };
                loaded = { ...loaded, [id]: next };
                if (mine) {
                    out[undoable ? 'back' : 'lost'].push(baked.robots[id].name);
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
