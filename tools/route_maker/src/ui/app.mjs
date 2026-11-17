// app.mjs — THE ROUTE BUILDER'S SHELL (RMP1 brief §1.4, A.6; run RMP1-A1): the store (one state object, every
// route edit through the undo history), the top bar (the route's name, the save state, undo/redo, the folder,
// the theme, help, PREVIEW — shown, not yet working — and Export), the three panes (steps, field, inspector),
// the status bar (the route's checks; the library build this page was made from), the keyboard, the toasts, and
// a draft kept in this browser between visits (a convenience, never instead of the project file: it says "your
// unsaved route is back" only when it differs from what was last saved or opened, and a connected folder's own
// route_maker.json is compared with the page at connect — the page asks before showing anything but the file's).
// Save never replaces a route_maker.json that changed in the folder since the page read it without asking. A route is
// DRAWN before it is kept: an opened file the page cannot show leaves the page as it was (never the next visit's
// draft), and a draft that cannot be shown starts a new route with a toast — never a page that will not start.
// The route's NAME is not editable here: §6.3 fixes it ("skills", which both robots' code registers).

import { h, icon, mount, ask } from './dom.mjs';
import { createFieldView } from './field_view.mjs';
import { renderStepList } from './step_list.mjs';
import { renderInspector } from './inspector.mjs';
import { openExportDrawer } from './export_drawer.mjs';
import { openHelp, coachCard } from './help.mjs';
import { TOOLS, mechanismActions, KIND_LABEL } from './words.mjs';
import { makeFiles, memoryHost } from '../host/files.mjs';
import { folderHost } from '../host/files_fsaccess.mjs';
import { downloadHost } from '../host/files_download.mjs';
import { bakedRobots, routeRobots } from '../host/robots.mjs';
import { capabilities, capabilityWords } from '../host/capabilities.mjs';
import { makeEngine, PREVIEW_COMING } from '../host/engine.mjs';
import { newProject, loadProject, saveProject, PROJECT_FILE } from '../model/project.mjs';
import { addStep, updateStep, removeStep, moveStep, duplicateStep, setStart, plannedPoses, normalizeRoute, startPose } from '../model/route.mjs';
import { checkRoute, applyFix, TURN_FIRST_EDGE_DEG } from '../model/checks.mjs';
import { historyOf, commit, undo as undoH, redo as redoH, canUndo, canRedo } from '../model/undo.mjs';
import { bearingDeg, wrapDeg } from '../model/geometry.mjs';

import { kMaxCurvePoints } from '../model/curve.mjs';

const DRAFT_KEY = 'shulib-route-builder:draft';
const SAVED_KEY = 'shulib-route-builder:saved';  // the project text last saved or opened (the draft is "unsaved" only when it differs)
const THEME_KEY = 'shulib-route-builder:theme';
const COACH_KEY = 'shulib-route-builder:coach-dismissed';

function store(key, value) {
    try {
        if (value === undefined) {
            return localStorage.getItem(key);
        }
        if (value === null) {
            localStorage.removeItem(key);
        } else {
            localStorage.setItem(key, value);
        }
    } catch {
        // a browser that keeps no site data: the draft and the theme are conveniences only
    }
    return null;
}

async function main() {
    const baked = bakedRobots();
    const picture = JSON.parse(document.getElementById('shulib-field').textContent);
    const caps = capabilities();
    const engine = makeEngine();
    let files = makeFiles(caps.writeFolder ? await folderHost() : downloadHost());

    // ── the state ──
    const madeWith = `shulib route builder (API ${baked.build.api})`;
    let project = newProject(madeWith);
    let restoredDraft = false;
    let startState = 'new';
    const draft = store(DRAFT_KEY);
    // a draft that is only a new, empty project (a visit with no edit, or every edit undone) is nothing to bring back
    if (draft && draft !== saveProject(project)) {
        const l = loadProject(draft);
        if (l.ok) {
            project = l.project;
            restoredDraft = draft !== store(SAVED_KEY);  // the same as last saved: nothing unsaved to announce
            startState = restoredDraft ? 'draft' : 'saved';
        }
    }
    let hist = historyOf(project.routes[0]);
    // The connected folder's route_maker.json as this page last read or wrote it (null: there is none, or the page
    // could not take the one there; undefined: not read yet). Save compares the folder's file with it before replacing
    // it, so it asks before replacing a file the page did not take.
    let folderBase;
    const ui = {
        selected: null, tool: 'select', curveDraft: [], snap: true,
        saveState: startState, coach: !store(COACH_KEY) && project.routes[0].steps.length === 0,
    };
    let dragBase = null;  // the route before a drag (a drag is ONE undo entry, committed when it ends)
    let derived = null;

    const route = () => hist.present;
    const robot = () => baked.robots[route().robot] || baked.robots.one;
    const field = () => robot().field;
    const size = () => (robot().drive && robot().drive.footprintIn) || 15;
    function derive() {
        if (derived && derived.route === route()) {
            return derived;
        }
        const r = route();
        const robots = routeRobots(baked, r);
        const issues = checkRoute(r, robots.length ? robots : [robot()], field());
        const levels = {};
        for (const x of issues) {
            const k = x.stepIndex === -1 ? 'start' : x.stepIndex;
            if (x.level === 'error' || !levels[k]) {
                levels[k] = x.level;
            }
        }
        derived = { route: r, issues, levels, plan: plannedPoses(r) };
        return derived;
    }

    // ── the DOM ──
    const els = {};
    const toolBtns = {};
    const app = {
        state: () => ui, route, robot, field, size, picture,
        plan: () => derive().plan, issues: () => derive().issues, levels: () => derive().levels,
        files: () => files, project: () => ({ ...project, routes: [route()] }),
        robotsWords,
        hud(f) {
            els.hud.textContent = `x ${f.x.toFixed(1)}   y ${f.y.toFixed(1)} in`;
        },
        select(sel) {
            ui.selected = sel;
            render();
            if (typeof sel === 'number' && window.matchMedia('(max-width: 1100px)').matches) {
                els.app.dataset.right = 'open';
            }
        },
        selectHandle(kind, index) {
            app.select(kind === 'start' || kind === 'heading' ? 'start' : index);
        },
        beginDrag() {
            dragBase = route();
        },
        dragTo(d) {
            let r = dragBase;
            if (d.handle === 'start') {
                r = setStart(r, { ...r.start, x: d.x, y: d.y });
            } else if (d.handle === 'heading') {
                r = setStart(r, { ...r.start, headingDeg: d.headingDeg });
            } else if (d.handle === 'point') {
                r = updateStep(r, d.index, { x: d.x, y: d.y });
            } else if (d.handle === 'curvePoint') {
                const pts = r.steps[d.index].points.map((p, k) => (k === d.point ? { ...p, x: d.x, y: d.y } : p));
                r = updateStep(r, d.index, { points: pts });
            }
            hist = { ...hist, present: r };
            renderFieldOnly();
        },
        endDrag(moved) {
            if (dragBase && moved) {
                const r = hist.present;
                hist = commit({ ...hist, present: dragBase }, r);
                changed();
            } else if (dragBase) {
                hist = { ...hist, present: dragBase };
            }
            dragBase = null;
            render();
        },
        fieldClick(p, ev) {
            const r = route();
            if (!r.start) {
                edit(setStart(r, { x: p.x, y: p.y, headingDeg: 0 }), 'start');
                toast('Robot one\'s start is placed. Drag its red knob to turn it, then add a step.');
                return;
            }
            const at = insertAt();
            const plan = derive().plan;
            const from = at === 0 ? r.start : plan[at - 1] && plan[at - 1].to;
            switch (ui.tool) {
                case 'driveTo':
                case 'reverseTo': {
                    let faceFirst = false;
                    if (from && robot().drive.tank) {
                        const b = bearingDeg(from.x, from.y, p.x, p.y);
                        const leg = ui.tool === 'reverseTo' ? wrapDeg(b + 180) : b;
                        faceFirst = Number.isFinite(leg) && Math.abs(wrapDeg(leg - from.headingDeg)) > TURN_FIRST_EDGE_DEG;
                    }
                    const kind = ui.tool;
                    ui.tool = 'select';  // one step at a time: the new step is selected, its details on the right
                    edit(addStep(r, { do: kind, x: p.x, y: p.y, faceFirst }, at), at);
                    break;
                }
                case 'face':
                    ui.tool = 'select';
                    edit(addStep(r, { do: 'face', x: p.x, y: p.y }, at), at);
                    break;
                case 'curveTo': {
                    // a second click on the same spot (the double-click that finishes) is not a new point
                    const last = ui.curveDraft[ui.curveDraft.length - 1];
                    const again = last && Math.hypot(last.x - p.x, last.y - p.y) < 1;
                    if (!again && ui.curveDraft.length < kMaxCurvePoints) {
                        ui.curveDraft = [...ui.curveDraft, { x: p.x, y: p.y }];
                    }
                    if (again || ui.curveDraft.length >= kMaxCurvePoints || ev.detail >= 2) {
                        app.finishCurve();
                    } else {
                        render();
                    }
                    break;
                }
                default:
                    app.select(null);
            }
        },
        backgroundClick(p, ev) {
            if (!route().start) {
                app.fieldClick(p, ev);
            } else {
                app.select(null);
            }
        },
        finishCurve() {
            if (ui.curveDraft.length) {
                const at = insertAt();
                const pts = ui.curveDraft;
                ui.curveDraft = [];
                ui.tool = 'select';
                edit(addStep(route(), { do: 'curveTo', points: pts }, at), at);
            }
        },
        update(i, patch) {
            edit(updateStep(route(), i, patch), ui.selected);
        },
        /** The inspector's start numbers. An emptied box never makes a hole; a start ALREADY missing parts (a
         *  hand-edited file) takes each one as it is typed, so a start missing two parts can be filled in one by one. */
        setStart(s) {
            const was = route().start || {};
            if (['x', 'y', 'headingDeg'].every((k) => Number.isFinite(s[k]) || !Number.isFinite(was[k]))) {
                edit(setStart(route(), s), 'start');
            }
        },
        move(from, to) {
            if (to < 0 || to >= route().steps.length) {
                return;
            }
            edit(moveStep(route(), from, to), to);
        },
        remove(i) {
            edit(removeStep(route(), i), route().steps.length > 1 ? Math.min(i, route().steps.length - 2) : null);
            toast(`Step ${i + 1} deleted (Ctrl+Z brings it back).`);
        },
        duplicate(i) {
            edit(duplicateStep(route(), i), i + 1);
        },
        fix(f) {
            const r = applyFix(route(), f, addStep, updateStep);
            const sel = f.kind === 'homeFirst' || f.kind === 'homeWhileDriving' ? 0 : f.kind === 'nameSkills' ? 'start' : f.stepIndex;
            edit(r, sel);
            if (f.kind === 'homeFirst' || f.kind === 'homeWhileDriving' || f.kind === 'waitForHome') {
                toast('Added. Type its time limit: the most the home (or the wait) may take.');
                setTimeout(() => {
                    const inp = document.querySelector('.pane.right input[name="boundS"]');
                    if (inp) {
                        inp.focus();
                    }
                }, 0);
            }
        },
        setProject(p, state) {
            project = { ...p, routes: [route()] };
            ui.saveState = state || ui.saveState;
            persistDraft();
            renderTop();
        },
        /** After a write into the folder: save route_maker.json too (its export hash). true: saved; false: NOT saved,
         *  the folder's file changed since this page read it (Save asks); null: no folder. */
        async saveQuietly() {
            if (files.canWriteFolder && files.connected()) {
                const out = await files.saveProject(app.project(), { base: folderBase });
                if (!out.ok) {
                    ui.saveState = 'unsaved';
                    renderTop();
                    return false;
                }
                savedAs(out);
                renderTop();
                return true;
            }
            return null;
        },
        save: () => save(),
        connect: () => connect(),
        focusField: () => els.fieldView.element.focus({ preventScroll: true }),
        showCoach() {
            ui.coach = true;
            store(COACH_KEY, null);
            render();
        },
        hideCoach() {
            ui.coach = false;
            store(COACH_KEY, '1');
            render();
        },
    };

    function insertAt() {
        return typeof ui.selected === 'number' ? ui.selected + 1 : route().steps.length;
    }
    function edit(r, select) {
        hist = commit(hist, normalizeRoute(r));
        ui.selected = select === undefined ? ui.selected : select;
        if (ui.coach && hist.present.steps.length > 0) {
            ui.coach = false;
        }
        changed();
        render();
    }
    function changed() {
        ui.saveState = 'unsaved';
        persistDraft();
    }
    function persistDraft() {
        store(DRAFT_KEY, saveProject({ ...project, routes: [route()] }));
    }
    function robotsWords() {
        const r = route();
        const list = routeRobots(baked, r).map((x) => x.name);
        const shared = baked.sharedRoute && baked.sharedRoute.present;
        return `The route is written in robot one's numbers${r.mirroredFor ? ' and robot two runs it MIRRORED (one line in its own file)' : ''}. `
            + `It is checked on ${list.join(' and ')}'s mechanisms`
            + (shared ? '.' : `; robot two's update (the shared skills-route file) is not in this build of the route builder yet, so robot two is checked once it is.`);
    }
    function toast(text, cls = '') {
        const t = h('div', { class: `toast ${cls}`, role: 'status' }, text);
        els.toasts.replaceChildren(t);  // one at a time: the newest says what just happened
        setTimeout(() => t.remove(), cls === 'error' ? 9000 : 4200);  // a refusal stays long enough to read
    }

    function setTool(id) {
        if (id === 'mechanism') {
            openMechanismMenu();
            return;
        }
        if (id === 'wait') {
            if (!route().start) {
                toast('Place robot one\'s start first: click the field where it starts.');
                return;
            }
            const at = insertAt();
            edit(addStep(route(), { do: 'wait' }, at), at);
            toast('Wait added: type how many seconds.');
            return;
        }
        if (ui.tool === 'curveTo' && id !== 'curveTo') {
            ui.curveDraft = [];
        }
        ui.tool = id;
        render();
    }
    function openMechanismMenu() {
        if (!route().start) {
            toast('Place robot one\'s start first: click the field where it starts.');
            return;
        }
        const close = () => {
            pop.remove();
            document.removeEventListener('pointerdown', outside, true);
            document.removeEventListener('keydown', onKey);
        };
        const outside = (e) => !pop.contains(e.target) && close();
        const onKey = (e) => e.key === 'Escape' && close();
        const mechs = robot().mechanisms || [];
        const items = [];
        for (const m of mechs) {
            items.push(h('h3', {}, `${m.name} · ${KIND_LABEL[m.kind] || m.kind}`));
            const acts = mechanismActions(m.kind);
            if (!acts.length) {
                items.push(h('button', { class: 'menu-item', disabled: true }, 'Flywheel steps come later', h('small', {}, '')));
            }
            for (const a of acts) {
                items.push(h('button', { class: 'menu-item', onclick: () => {
                    close();
                    const at = insertAt();
                    const step = { do: a.do, mechanism: m.ident };
                    if (a.do === 'move' && (m.presets || []).length === 1) {
                        const p = m.presets[0];
                        step.preset = p.ledger !== undefined ? p.ledger : p.name;
                    }
                    edit(addStep(route(), step, at), at);
                    toast('Added: choose its level or position and type its time limit on the right.');
                } }, icon(a.do === 'home' ? 'home' : a.do === 'waitUntilDone' ? 'wait' : 'mechanism'), a.words));
            }
        }
        if (!mechs.length) {
            items.push(h('p', { class: 'hint', style: 'padding:8px' }, `${robot().name} declares no mechanism yet.`));
        }
        const pop = h('div', { class: 'popover', role: 'menu', 'aria-label': 'Mechanisms' }, items);
        const r = toolBtns.mechanism.getBoundingClientRect();
        pop.style.left = `${Math.max(8, r.left)}px`;
        pop.style.top = `${r.bottom + 6}px`;
        document.body.append(pop);
        setTimeout(() => {
            document.addEventListener('pointerdown', outside, true);
            document.addEventListener('keydown', onKey);
            const first = pop.querySelector('.menu-item:not(:disabled)');
            if (first) {
                first.focus();
            }
        }, 0);
    }

    /** A save went through: remember it (the folder's base, and what "saved" means for the draft). */
    function savedAs(out) {
        ui.saveState = out.where === 'folder' ? 'saved' : 'downloaded';
        if (out.where === 'folder') {
            folderBase = out.text;
        }
        store(SAVED_KEY, out.text);
    }
    async function save(force = false) {
        try {
            const out = await files.saveProject(app.project(), { base: folderBase, force });
            if (!out.ok && out.why === 'changedOnDisk') {
                const choice = await ask({
                    title: `${PROJECT_FILE} changed in the folder`,
                    words: `${out.words} Saving now would replace that route with the one on this page. Open the folder's to see it, or replace it.`,
                    choices: [{ label: 'Open the folder\'s route', value: 'open', kind: 'primary' },
                              { label: 'Replace it with this page\'s route', value: 'replace', kind: 'danger' },
                              { label: 'Cancel', value: null }] });
                if (choice === 'replace') {
                    return save(true);
                }
                if (choice === 'open') {
                    adoptFolderText(out.diskText);
                }
                return false;
            }
            savedAs(out);
            toast(out.words);
            renderTop();
            return true;
        } catch (e) {
            toast(`Not saved: ${e.message}`, 'error');
            renderTop();
            return false;
        }
    }
    /**
     * Make an opened project the page's (the undo history keeps the page's route, one Ctrl+Z away). It is DRAWN
     * first and kept (the draft, "last saved") only once it is on screen: a route this page cannot draw puts
     * everything back as it was and throws, so a failed open never becomes the draft the next visit starts from.
     * `folderText`: the folder's own text of it (Save's base), or undefined when it did not come from the folder.
     */
    function adoptProject(p, folderText) {
        const before = { project, hist, folderBase, selected: ui.selected, saveState: ui.saveState, coach: ui.coach };
        project = p;
        hist = commit(hist, normalizeRoute(p.routes[0]));
        if (folderText !== undefined) {
            folderBase = folderText;
        }
        ui.selected = null;
        ui.saveState = 'saved';
        ui.coach = false;
        try {
            render();
        } catch (e) {
            ({ project, hist, folderBase } = before);
            ui.selected = before.selected;
            ui.saveState = before.saveState;
            ui.coach = before.coach;
            render();
            throw e;
        }
        persistDraft();
        store(SAVED_KEY, saveProject(app.project()));
    }
    /** Make the folder's project text the page's route. true: it is the page's now. */
    function adoptFolderText(text) {
        const l = loadProject(text);
        if (!l.ok) {
            toast(`${PROJECT_FILE} in the folder was not opened: ${l.words}`, 'error');
            return false;
        }
        try {
            adoptProject(l.project, text);
        } catch (e) {
            folderBase = null;  // the folder's file was NOT taken: Save asks before replacing it
            toast(`${PROJECT_FILE} in the folder was not opened: this page cannot show its route (${e.message}). `
                + 'The page is as it was.', 'error');
            return false;
        }
        return true;
    }
    /**
     * The folder was connected (or remembered): read its route_maker.json and compare. The same route: "Saved".
     * An empty page: the folder's route opens. Two different routes: the student chooses — the page never shows
     * a stale draft as the team's route without asking.
     */
    async function syncWithFolder() {
        if (!(files.canWriteFolder && files.connected())) {
            return;
        }
        let disk;
        try {
            disk = await files.readProjectFile();
        } catch (e) {
            toast(`Could not read ${PROJECT_FILE}: ${e.message}`, 'error');
            return;
        }
        folderBase = disk;
        if (disk === null) {
            return;  // no project file yet: the first Save makes it
        }
        const l = loadProject(disk);
        if (!l.ok) {
            toast(`${PROJECT_FILE} in the folder was not opened: ${l.words}`, 'error');
            return;
        }
        const mine = saveProject(app.project());
        if (saveProject(l.project) === mine) {
            ui.saveState = 'saved';
            store(SAVED_KEY, mine);
            renderTop();
            return;
        }
        if (!route().start && route().steps.length === 0) {
            if (adoptFolderText(disk)) {
                toast(`Opened ${PROJECT_FILE} from the folder.`);
            }
            return;
        }
        const choice = await ask({
            title: 'The folder has a different route',
            words: `${PROJECT_FILE} in ${files.folderName()} is not the route on this page. Open the folder's (what the team `
                + 'last saved there, a git pull included), or keep this page\'s (Save then replaces the file).',
            choices: [{ label: 'Open the folder\'s route', value: 'open', kind: 'primary' },
                      { label: 'Keep this page\'s route', value: 'keep' }],
            cancel: 'keep' });
        if (choice === 'open') {
            adoptFolderText(disk);
        } else {
            ui.saveState = 'unsaved';
            renderTop();
        }
    }
    async function openProj() {
        try {
            const out = await files.openProject();
            if (!out.ok) {
                if (out.words) {
                    toast(out.words, 'error');
                }
                return;
            }
            try {
                adoptProject(out.project, out.where === 'folder' ? out.text : undefined);
            } catch (e) {
                toast(`Not opened: this page cannot show the route in that file (${e.message}). The page is as it was.`, 'error');
                return;
            }
            toast('Opened the project.');
        } catch (e) {
            toast(`Not opened: ${e.message}`, 'error');
        }
    }
    async function connect() {
        const r = await files.connect();
        if (r && r.ok === false) {
            toast(r.words, 'error');
        } else if (files.connected()) {
            toast(`Connected: ${files.folderName()}. Save and Export now write into it.`);
            await syncWithFolder();
        }
        renderTop();
    }
    function undo() {
        hist = undoH(hist);
        changed();
        render();
    }
    function redo() {
        hist = redoH(hist);
        changed();
        render();
    }
    function theme() {
        const root = document.documentElement;
        const dark = root.dataset.theme ? root.dataset.theme === 'dark' : window.matchMedia('(prefers-color-scheme: dark)').matches;
        root.dataset.theme = dark ? 'light' : 'dark';
        store(THEME_KEY, root.dataset.theme);
        renderTop();
    }

    // ── build the shell ──
    const savedTheme = store(THEME_KEY);
    if (savedTheme === 'light' || savedTheme === 'dark') {
        document.documentElement.dataset.theme = savedTheme;
    }
    els.toolbar = h('div', { class: 'toolbar', role: 'toolbar', 'aria-label': 'Add a step' },
        TOOLS.map((t) => {
            const b = h('button', { class: 'tool', 'aria-pressed': 'false', title: `${t.help} (${t.key})`, 'aria-label': `${t.label} (${t.key})`,
                                    'data-tool': t.id, onclick: () => setTool(t.id) }, icon(t.icon), t.label, h('kbd', {}, t.key));
            toolBtns[t.id] = b;
            return b;
        }),
        h('span', { class: 'sep' }),
        els.snapBtn = h('button', { class: 'tool', 'aria-pressed': 'true', title: 'Snap to the half-tile grid and the walls (hold Alt to place freely)',
                                    onclick: () => { ui.snap = !ui.snap; render(); } }, icon('snap'), 'Snap', h('kbd', {}, 'Alt')));
    els.hud = h('div', { class: 'hud', 'aria-live': 'off' }, 'x —   y — in');
    els.modeHint = h('div', { class: 'mode-hint', role: 'status' });
    els.fieldPane = h('section', { class: 'fieldpane', 'aria-label': 'The field' });
    els.zoom = h('div', { class: 'zoom' },
        h('button', { class: 'btn icon ghost', 'aria-label': 'Zoom in (+)', title: 'Zoom in (+)', onclick: () => els.fieldView.zoom(1 / 1.25) }, icon('plus')),
        h('button', { class: 'btn icon ghost', 'aria-label': 'Zoom out (-)', title: 'Zoom out (-)', onclick: () => els.fieldView.zoom(1.25) }, icon('minus')),
        h('button', { class: 'btn icon ghost', 'aria-label': 'Fit the field (0)', title: 'Fit the field (0)', onclick: () => els.fieldView.fit() }, icon('fit')));
    els.coach = h('div');
    els.steps = h('div', { class: 'pane-body' });
    els.inspector = h('div', { class: 'pane-body', 'aria-live': 'polite' });
    els.top = h('header', { class: 'topbar' });
    els.status = h('footer', { class: 'statusbar' });
    els.toasts = h('div', { class: 'toasts', 'aria-live': 'polite' });
    els.app = h('div', { class: 'app', id: 'app' },
        els.top,
        h('main', { class: 'workspace' },
          h('aside', { class: 'pane left', 'aria-label': 'Steps' },
            h('div', { class: 'pane-head' }, h('h2', {}, 'Route'), h('span', { class: 'spacer' }),
              h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Close the steps', onclick: () => delete els.app.dataset.left }, icon('close'))),
            els.steps),
          els.fieldPane,
          h('aside', { class: 'pane right', 'aria-label': 'The selected step' },
            h('div', { class: 'pane-head' }, h('h2', {}, 'Step'), h('span', { class: 'spacer' }),
              h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Close the step', onclick: () => delete els.app.dataset.right }, icon('close'))),
            els.inspector)),
        els.status);
    document.body.append(els.app, els.toasts);
    els.fieldView = createFieldView(els.fieldPane, app);
    els.fieldPane.append(els.toolbar, els.hud, els.zoom, els.modeHint, els.coach);

    function renderTop() {
        const r = route();
        const saveWords = { new: ['New route', ''], saved: ['Saved', 'ok'], downloaded: ['Downloaded', 'ok'], exported: ['Exported', 'ok'],
                            unsaved: ['Unsaved changes', 'warn'], draft: ['Draft kept in this browser', 'warn'] }[ui.saveState];
        const dark = document.documentElement.dataset.theme ? document.documentElement.dataset.theme === 'dark'
            : window.matchMedia('(prefers-color-scheme: dark)').matches;
        mount(els.top,
            h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Steps', onclick: () => { els.app.dataset.left = els.app.dataset.left ? '' : 'open'; } }, icon('list')),
            h('div', { class: 'brand' }, h('span', { class: 'brand-mark', 'aria-hidden': 'true' }, icon('curve')),
              h('span', { class: 'hide-narrow' }, 'Route builder'), h('small', {}, 'shulib')),
            h('span', { class: 'sep hide-narrow' }),
            h('span', { class: 'chip robot hide-narrow', title: 'The route is written in robot one\'s numbers' }, 'Robot one'),
            h('span', { class: 'route-name', title: 'The route\'s name on the brain\'s menu (both robots\' code registers it by this name)' }, r.name),
            h('span', { class: `chip ${saveWords[1]}`, id: 'save-state', role: 'status' }, saveWords[0]),
            h('span', { class: 'spacer' }),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Undo (Ctrl+Z)', title: 'Undo (Ctrl+Z)', disabled: !canUndo(hist), onclick: undo }, icon('undo')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Redo (Ctrl+Shift+Z)', title: 'Redo (Ctrl+Shift+Z)', disabled: !canRedo(hist), onclick: redo }, icon('redo')),
            h('span', { class: 'sep' }),
            files.canWriteFolder
                ? h('button', { class: 'btn hide-narrow', onclick: connect, title: capabilityWords(caps) }, icon('folder'),
                    files.connected() ? files.folderName() : 'Connect your shulib folder')
                : h('span', { class: 'chip hide-narrow', title: capabilityWords(caps) }, 'Save downloads · Export copies'),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Open a project', title: 'Open a project', onclick: openProj }, icon('open')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Save (Ctrl+S)', title: 'Save (Ctrl+S)', onclick: save }, icon('save')),
            h('button', { class: 'btn icon ghost', 'aria-label': dark ? 'Light theme' : 'Dark theme', title: dark ? 'Light theme' : 'Dark theme', onclick: theme }, icon(dark ? 'sun' : 'moon')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Help and shortcuts (?)', title: 'Help and shortcuts (?)', onclick: () => openHelp(app) }, icon('help')),
            h('span', { class: 'sep' }),
            h('button', { class: 'btn big', id: 'preview', disabled: !engine.available, title: PREVIEW_COMING, 'aria-describedby': 'preview-why' }, icon('play'), 'PREVIEW'),
            h('span', { class: 'sr-only', id: 'preview-why' }, PREVIEW_COMING),
            exportButton(),
            h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Step details', onclick: () => { els.app.dataset.right = els.app.dataset.right ? '' : 'open'; } }, icon('panel')));
    }
    /** Export: always clickable (it lists what to fix, each one click away); with errors it carries their count. */
    function exportButton() {
        const errs = derive().issues.filter((x) => x.level === 'error');
        const first = errs.slice(0, 3).map((x) => `${x.stepIndex >= 0 ? `step ${x.stepIndex + 1}: ` : ''}${x.words}`);
        return h('button', { class: `btn big primary${errs.length ? ' has-errors' : ''}`, id: 'export', onclick: () => openExportDrawer(app),
                             title: errs.length ? `Fix ${errs.length === 1 ? 'this' : `these ${errs.length}`} first (Ctrl+E lists them): ${first.join(' · ')}`
                                                : 'Export the C++ (Ctrl+E)' },
                 icon('code'), 'Export',
                 errs.length ? h('span', { class: 'count', 'aria-label': `${errs.length} error${errs.length === 1 ? '' : 's'} to fix first` }, String(errs.length)) : null);
    }
    function renderStatus() {
        const issues = derive().issues;
        const errors = issues.filter((x) => x.level === 'error');
        const warns = issues.filter((x) => x.level === 'warn');
        const first = errors[0] || warns[0];
        mount(els.status,
            errors.length ? h('span', { class: 'chip error' }, icon('error'), `${errors.length} error${errors.length === 1 ? '' : 's'}`) : null,
            warns.length ? h('span', { class: 'chip warn' }, icon('warn'), `${warns.length} warning${warns.length === 1 ? '' : 's'}`) : null,
            !errors.length && !warns.length
                ? (route().steps.length ? h('span', { class: 'chip ok' }, icon('check'), 'Ready to export') : h('span', { class: 'chip' }, 'No steps yet'))
                : null,
            first ? h('button', { class: 'link', onclick: () => app.select(first.stepIndex >= 0 ? first.stepIndex : 'start') },
                      first.stepIndex >= 0 ? `Go to step ${first.stepIndex + 1}` : 'Go to the start') : null,
            h('span', { class: 'build', title: 'The robots and the library this page was built with' },
              `Library API ${baked.build.api} · built from ${baked.build.commit} on ${baked.build.date} · the dashed path is the plan`));
    }
    function renderFieldOnly() {
        derived = null;
        els.fieldView.render();
    }
    function render() {
        derived = null;
        for (const [id, b] of Object.entries(toolBtns)) {
            b.setAttribute('aria-pressed', String(ui.tool === id));
        }
        els.snapBtn.setAttribute('aria-pressed', String(ui.snap));
        const hints = { driveTo: 'Click where the robot drives to', reverseTo: 'Click where the robot backs up to', face: 'Click the point to turn and face',
                        curveTo: ui.curveDraft.length ? `${ui.curveDraft.length} point${ui.curveDraft.length === 1 ? '' : 's'} — click more; Enter or double-click finishes; Esc cancels`
                                                      : 'Click the curve\'s points; Enter or double-click finishes' };
        const noStart = !route().start;
        const startHole = !noStart && !startPose(route().start);  // a hand-edited file's start missing a number
        els.modeHint.hidden = ui.coach || !(noStart || startHole || hints[ui.tool]);  // the coach card says it already, over the same spot
        els.modeHint.textContent = noStart ? 'Click the field where robot one starts'
            : startHole ? 'Robot one\'s start is missing a number: select the start (the top of the list) and type it' : hints[ui.tool] || '';
        mount(els.coach, ui.coach ? coachCard(app) : null);
        renderTop();
        renderStepList(els.steps, app);
        renderInspector(els.inspector, app);
        els.fieldView.render();
        renderStatus();
        document.documentElement.dataset.steps = String(route().steps.length);
    }

    // ── the keyboard (§1.4's shortcuts) ──
    document.addEventListener('keydown', (e) => {
        const typing = /INPUT|SELECT|TEXTAREA/.test(document.activeElement.tagName);
        const ctrl = e.ctrlKey || e.metaKey;
        if (ctrl && e.key.toLowerCase() === 'z') {
            if (!typing) {
                e.preventDefault();
                (e.shiftKey ? redo : undo)();
            }
            return;
        }
        if (ctrl && e.key.toLowerCase() === 'y' && !typing) {
            e.preventDefault();
            redo();
            return;
        }
        if (ctrl && e.key.toLowerCase() === 's') {
            e.preventDefault();
            save();
            return;
        }
        if (ctrl && e.key.toLowerCase() === 'e') {
            e.preventDefault();
            openExportDrawer(app);
            return;
        }
        if (ctrl && e.key.toLowerCase() === 'd' && typeof ui.selected === 'number' && !typing) {
            e.preventDefault();
            app.duplicate(ui.selected);
            return;
        }
        if (e.altKey && !typing && (e.key === 'ArrowUp' || e.key === 'ArrowDown') && typeof ui.selected === 'number') {
            e.preventDefault();
            app.move(ui.selected, ui.selected + (e.key === 'ArrowUp' ? -1 : 1));
            return;
        }
        if (typing || ctrl || e.altKey || document.querySelector('.drawer, .dialog, .popover')) {
            return;
        }
        if (e.key === 'Escape') {
            ui.curveDraft = [];
            ui.tool = 'select';
            render();
            return;
        }
        if (e.key === 'Enter' && ui.tool === 'curveTo') {
            app.finishCurve();
            return;
        }
        if ((e.key === 'Delete' || e.key === 'Backspace') && typeof ui.selected === 'number') {
            e.preventDefault();
            app.remove(ui.selected);
            return;
        }
        if (e.key === '?') {
            openHelp(app);
            return;
        }
        if (e.key === '+' || e.key === '=') {
            els.fieldView.zoom(1 / 1.25);
            return;
        }
        if (e.key === '-') {
            els.fieldView.zoom(1.25);
            return;
        }
        if (e.key === '0') {
            els.fieldView.fit();
            return;
        }
        const t = TOOLS.find((x) => x.key.toLowerCase() === e.key.toLowerCase());
        if (t) {
            setTool(t.id);
        }
    });
    window.addEventListener('beforeunload', persistDraft);

    try {
        render();
    } catch (e) {
        if (startState === 'new') {
            throw e;  // nothing came from this browser: the page itself failed ("did not start", below)
        }
        // the route kept in this browser cannot be shown (one this page cannot draw): never a page that will not start
        // again — a new route, and the student told why; the project file is not touched
        project = newProject(madeWith);
        hist = historyOf(project.routes[0]);
        ui.selected = null;
        ui.saveState = 'new';
        ui.coach = !store(COACH_KEY);
        restoredDraft = false;
        store(DRAFT_KEY, null);
        render();
        toast(`Your route from last time could not be shown (${e.message}), so a new route is open. Your project file is `
            + `not changed: Open your ${PROJECT_FILE.split('/').pop()} to carry on.`, 'error');
    }
    els.fieldView.fit();
    if (restoredDraft) {
        toast('Your unsaved route from last time is back (kept in this browser). Save it into the project file to keep it.');
    }
    syncWithFolder();  // a folder remembered from last time (Chrome/Edge): compare its project file with the page
    // for the page tests: the app is up, and a read-only look at its state
    // (and, for the file layer's page tests, an in-memory folder in place of the real one: the folder dialog itself
    // is the manual Windows check's, A.4)
    window.routeBuilder = { route: () => JSON.parse(JSON.stringify(route())), issues: () => derive().issues, ready: true,
                            useMemoryFolder: async (files0, opts) => {
                                const host = memoryHost(files0, opts || {});
                                files = makeFiles(host);
                                folderBase = undefined;
                                window.routeBuilder.memory = host;
                                render();
                                await syncWithFolder();
                                return true;
                            } };
    document.documentElement.dataset.ready = '1';
}

main().catch((e) => {
    document.documentElement.dataset.ready = 'error';
    document.body.append(h('div', { class: 'too-old', role: 'alert' }, h('h1', {}, 'The route builder did not start'),
        h('p', {}, `${e && e.message ? e.message : e}. Reload the page; if it happens again, tell the coordinator with these words.`)));
    throw e;
});
