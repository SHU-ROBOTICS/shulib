// app.mjs — THE ROUTE BUILDER'S SHELL (RMP1 brief §1.4, A.6; run RMP1-A1): the store (one state object, every
// route edit through the undo history), the top bar (the route's name, the save state, undo/redo, the folder,
// the theme, help, PREVIEW — preview.mjs — and Export), the three panes (steps, field, inspector),
// the status bar (the route's checks; the library build this page was made from), the keyboard, the toasts, and
// a draft kept in this browser between visits (a convenience, never instead of the project file: it says "your
// unsaved route is back" only when it differs from what was last saved or opened, and a connected folder's own
// route_maker.json is compared with the page at connect — the page asks before showing anything but the file's).
// Save never replaces a route_maker.json that changed in the folder since the page read it without asking. A route is
// DRAWN before it is kept: an opened file the page cannot show leaves the page as it was (never the next visit's
// draft), and a draft that cannot be shown starts a new route with a toast — never a page that will not start.
// RMP1-C: the project's NAMED routes — the switcher (routes_menu.mjs) adds, renames, duplicates and deletes them, one
// route on the field at a time, and the undo history holds the whole set (an undo can bring a deleted route back).
// Robot two's mirrored start and path are drawn beside robot one's; a mirrored route asks, on the route (no dialog in
// the way, no default: Export waits for the answer), whether both robots run it at the same time. Arrow keys nudge the
// selected point 1 in (Shift: 0.25 in). RMP1-G: H hides the selected step on the field, L locks it, O focuses on it and
// its neighbours (Esc leaves); hide, lock and sections are route edits (undo, saved), focus is not. RMP1-I: served by
// tools/route_builder.py, the page takes the local host (no folder dialog) and the top bar's Send to robot.

import { h, icon, mount, ask, openMenu } from './dom.mjs';
import { createRoutesMenu } from './routes_menu.mjs';
import { createFieldView } from './field_view.mjs';
import { renderStepList } from './step_list.mjs';
import { renderInspector } from './inspector.mjs';
import { openExportDrawer } from './export_drawer.mjs';
import { openHelp, coachCard } from './help.mjs';
import { TOOLS, mechanismActions, KIND_LABEL } from './words.mjs';
import { round3 } from '../model/numbers.mjs';
import { targetKind } from '../model/lift.mjs';
import { makeFiles, memoryHost } from '../host/files.mjs';
import { folderHost } from '../host/files_fsaccess.mjs';
import { downloadHost } from '../host/files_download.mjs';
import { localPage, localHost, sendToRobot, sendLog } from '../host/files_local.mjs';
import { createSendPanel } from './send_panel.mjs';
import { bakedRobots, routeRobots } from '../host/robots.mjs';
import { capabilities, capabilityWords } from '../host/capabilities.mjs';
import { makeEngine } from '../host/engine.mjs';
import { createPreview } from './preview.mjs';
import { openMechMenu } from './mech_menu.mjs';
import { newProject, loadProject, saveProject, PROJECT_FILE, addRoute, renameRouteAt, duplicateRoute, deleteRoute } from '../model/project.mjs';
import { addStep, updateStep, removeStep, moveStep, duplicateStep, setStart, plannedPoses, normalizeRoute, startPose } from '../model/route.mjs';
import { checkRoute, applyFix, TURN_FIRST_EDGE_DEG, LEVELS_FROM, noLevels } from '../model/checks.mjs';
import { historyOf, commit, undo as undoH, redo as redoH, canUndo, canRedo } from '../model/undo.mjs';
import { bearingDeg, wrapDeg } from '../model/geometry.mjs';

import { kMaxCurvePoints } from '../model/curve.mjs';
import { createControlsTab } from './controls_tab.mjs';
import { setView, isHidden, isLocked, sectionsOf, indicesOf } from '../model/step_view.mjs';

const DRAFT_KEY = 'shulib-route-builder:draft';
const SAVED_KEY = 'shulib-route-builder:saved';  // the project text last saved or opened (the draft is "unsaved" only when it differs)
const THEME_KEY = 'shulib-route-builder:theme';
const COACH_KEY = 'shulib-route-builder:coach-dismissed';
const TAB_KEY = 'shulib-route-builder:tab';  // RMP1-B: the Route or the Controls tab, as last left

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
    const local = localPage();
    let files = makeFiles(local ? localHost(local) : caps.writeFolder ? await folderHost() : downloadHost());

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
    // the history holds the WHOLE set of routes and which one is on the field: {routes, at}
    const setOf = (p) => ({ routes: p.routes.map(normalizeRoute), at: 0 });
    let hist = historyOf(setOf(project));
    // The connected folder's route_maker.json as this page last read or wrote it (null: there is none, or the page
    // could not take the one there; undefined: not read yet). Save compares the folder's file with it before replacing
    // it, so it asks before replacing a file the page did not take.
    let folderBase;
    const ui = {
        selected: null, point: null, tool: 'select', curveDraft: [], snap: true, focus: false, focusAt: null, here: null,
        saveState: startState, coach: !store(COACH_KEY) && project.routes.every((r) => r.steps.length === 0),
        tab: store(TAB_KEY) === 'controls' ? 'controls' : 'route',
    };
    let dragBase = null;  // the route before a drag (a drag is ONE undo entry, committed when it ends)
    let derived = null;

    const routes = () => hist.present.routes;
    const at = () => Math.min(hist.present.at, routes().length - 1);
    const route = () => routes()[at()];
    /** The set with the route on the field replaced by `r`. */
    const withRoute = (r) => ({ ...hist.present, routes: routes().map((x, k) => (k === at() ? r : x)) });
    const robot = () => baked.robots[route().robot] || baked.robots.one;
    const field = () => robot().field;
    const size = () => (robot().drive && robot().drive.footprintIn) || 15;
    const issueCache = new WeakMap();  // each route's issues, by its set (a route's names depend on the others)
    /** Route `k`'s issues (its checks need the whole set: the names, the skills route). */
    function issuesOf(k) {
        const set = hist.present.routes;
        const r = set[k];
        const hit = issueCache.get(r);
        if (hit && hit.set === set) {
            return hit.issues;
        }
        const robots = routeRobots(baked, r);
        const issues = checkRoute(r, robots.length ? robots : [robot()], field(), { routes: set, index: k });
        issueCache.set(r, { set, issues });
        return issues;
    }
    /** The errors of every OTHER route, each with its route: Export writes them all, so it refuses on any. */
    function otherErrors() {
        return routes().flatMap((r, k) => (k === at() ? [] : issuesOf(k).filter((x) => x.level === 'error').map((x) => ({ ...x, route: k }))));
    }
    function derive() {
        if (derived && derived.route === route() && derived.set === routes()) {
            return derived;
        }
        const r = route();
        const issues = issuesOf(at());
        const levels = {};
        for (const x of issues) {
            const k = x.stepIndex === -1 ? 'start' : x.stepIndex;
            if (x.level === 'error' || !levels[k]) {
                levels[k] = x.level;
            }
        }
        derived = { route: r, set: routes(), issues, levels, plan: plannedPoses(r) };
        return derived;
    }

    // ── the DOM ──
    const els = {};
    const toolBtns = {};
    const app = {
        state: () => ui, route, robot, field, size, picture,
        plan: () => derive().plan, issues: () => derive().issues, levels: () => derive().levels,
        routes, at, otherErrors, issuesOf,
        files: () => files, project: () => ({ ...project, routes: routes() }),
        robotsWords,
        hud(f) {
            els.hud.textContent = `x ${f.x.toFixed(1)}   y ${f.y.toFixed(1)} in`;
        },
        /** `open`: a narrow window's step drawer slides in (a click; never a drag, which would cover the field). */
        select(sel, point = null, open = true, here = null) {
            pick(sel, point, here);
            render();
            if (open && typeof sel === 'number' && window.matchMedia('(max-width: 1100px)').matches) {
                els.app.dataset.right = 'open';
            }
        },
        selectHandle(kind, index, point, open, here = null) {
            app.select(kind === 'start' || kind === 'heading' ? 'start' : index, kind === 'curvePoint' ? point : null, open, here);
        },
        beginDrag() {
            dragBase = hist.present;
        },
        dragTo(d) {
            let r = dragBase.routes[dragBase.at];
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
            hist = { ...hist, present: { ...dragBase, routes: dragBase.routes.map((x, k) => (k === dragBase.at ? r : x)) } };
            renderFieldOnly();
        },
        endDrag(moved) {
            if (dragBase && moved) {
                const set = hist.present;
                hist = commit({ ...hist, present: dragBase }, set);
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
                const ask = r.mirroredFor && r.together === undefined;
                if (ask && window.matchMedia('(max-width: 1100px)').matches) {
                    els.app.dataset.right = 'open';  // the question is on the right: a narrow window shows it
                }
                toast(ask ? 'Robot one\'s start is placed; robot two\'s mirrored one is the blue square. On the right: do both robots run this at the same time?'
                    : 'Robot one\'s start is placed. Drag its red knob to turn it, then add a step.');
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
            const sel = f.kind === 'homeFirst' || f.kind === 'homeWhileDriving' ? 0
                : ['nameSkills', 'together', 'moveStart'].includes(f.kind) ? 'start' : f.stepIndex;
            edit(r, sel);
            if (f.kind === 'homeFirst' || f.kind === 'homeWhileDriving' || f.kind === 'waitForHome') {
                toast('Added. Type its time limit: the most the home (or the wait) may take.');
                // now (edit() drew it): a deferred focus could land after the student has moved on (RMP1-B's verifier)
                const inp = document.querySelector('.pane.right input[name="boundS"]');
                if (inp) {
                    inp.focus();
                }
            }
        },
        setProject(p, state) {
            project = { ...p, routes: routes() };
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
        // RMP1-B: the Controls tab's calls
        baked,
        toast: (t, c) => toast(t, c),
        changed: () => renderTop(),
        setRobots(robots, exports = project.exports) {
            project = { ...project, robots, exports, routes: routes() };
            changed();
            renderTop();
            renderStatus();
        },
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
        // ── RMP1-C: the named routes ──
        switchRoute(k) {
            if (k !== at() && routes()[k]) {
                hist = { ...hist, present: { ...hist.present, at: k } };  // which route shows is not an edit
                pick(null);
                ui.focus = false;
                ui.curveDraft = [];
                ui.tool = 'select';
                render();
                els.fieldView.fit();
            }
        },
        addRoute(name) {
            editSet(addRoute(hist.present, name));
            toast(`Route "${route().name}" added: place robot one's start on the field.`);
        },
        renameRoute(k, name) {
            if (name === undefined) {
                routesMenu.rename(k);
                return;
            }
            const was = routes()[k].name;
            editSet(renameRouteAt(hist.present, k, name));
            toast(`Renamed "${was}" to "${name}" (Ctrl+Z puts it back).`);
        },
        duplicateRoute(k, name) {
            editSet(duplicateRoute(hist.present, k, name));
            toast(`Duplicated as "${route().name}".`);
        },
        deleteRoute(k) {
            const was = routes()[k].name;
            const next = deleteRoute(hist.present, k);
            if (next !== hist.present) {
                editSet(next);
                toast(`Route "${was}" deleted (Ctrl+Z brings it back).`);
            }
        },
        setMirror(on) {
            const r = route();
            edit({ ...r, mirroredFor: on ? 'two' : undefined, together: on ? r.together : undefined }, 'start');
        },
        setTogether(v) {
            edit({ ...route(), together: v }, 'start');
        },
    };
    const routesMenu = createRoutesMenu(app);

    /** D9: move the selected point (the start, a step's point, a curve point) by (dx, dy) in; one undo entry. false:
     *  nothing selected has a point. Up is +y (the field's frame, as drawn). */
    function nudge(dx, dy) {
        const r = route();
        const move = (p) => ({ ...p, x: round3(p.x + dx), y: round3(p.y + dy) });
        if (ui.selected === 'start' && r.start && Number.isFinite(r.start.x) && Number.isFinite(r.start.y)) {
            edit(setStart(r, move(r.start)), 'start');
            return true;
        }
        const s = typeof ui.selected === 'number' ? r.steps[ui.selected] : null;
        const curve = s && s.points && Number.isInteger(ui.point) && s.points[ui.point] && Number.isFinite(s.points[ui.point].x);
        if (!curve && !(s && Number.isFinite(s.x) && Number.isFinite(s.y))) {
            return false;
        }
        if (isHidden(s) || isLocked(s)) {  // RMP1-G: a point nobody can see, or one locked, does not move
            const hidden = isHidden(s);
            toast(`Step ${ui.selected + 1} is ${hidden ? 'hidden' : 'locked'}: ${hidden ? 'show it (H)' : 'unlock it (L)'} to move it, or type its numbers.`);
            return true;
        }
        edit(updateStep(r, ui.selected, curve ? { points: s.points.map((p, k) => (k === ui.point ? move(p) : p)) } : move(s)));
        return true;
    }
    function insertAt() {
        return typeof ui.selected === 'number' ? ui.selected + 1 : route().steps.length;
    }
    /** The selection and the curve point picked ON THE FIELD are set only together: the point never outlives its pick.
     *  `here`: {k, n} when the pick was the k-th of n points on one spot (RMP1-G D4); focus follows the selection. */
    function pick(sel, point = null, here = null) {
        ui.selected = sel;
        ui.point = point;
        ui.here = here;
        if (sel !== null) {
            ui.focusAt = sel;
        }
    }
    /** `select` undefined (the nudge): the selection AND its picked point stay; any other edit drops the point. */
    function edit(r, select) {
        hist = commit(hist, withRoute(normalizeRoute(r)));
        ui.here = null;  // what is on the clicked spot may have changed
        if (select !== undefined) {
            pick(select);
        }
        if (ui.coach && route().steps.length > 0) {
            ui.coach = false;
        }
        changed();
        render();
    }
    /** A change to the set of routes (add, rename, duplicate, delete): one undo entry, the new `at` on the field. */
    function editSet(set) {
        hist = commit(hist, { ...set, routes: set.routes.map(normalizeRoute) });
        pick(null);
        ui.focus = false;
        changed();
        render();
        els.fieldView.fit();
    }
    function changed() {
        ui.saveState = 'unsaved';
        persistDraft();
    }
    function persistDraft() {
        store(DRAFT_KEY, saveProject({ ...project, routes: routes() }));
    }
    function robotsWords() {
        const r = route();
        const list = routeRobots(baked, r).map((x) => x.name);
        const shared = baked.sharedRoute && baked.sharedRoute.present;
        return `The route is written in robot one's numbers${r.mirroredFor ? '; robot two runs it MIRRORED' : '; robot two runs it as written'}. `
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
        if (id === 'straight') {  // D9: "drive forward _ in" from where the robot is
            if (!route().start) {
                toast('Place robot one\'s start first: click the field where it starts.');
                return;
            }
            const at = insertAt();
            edit(addStep(route(), { do: 'forward' }, at), at);
            toast('Added: type how far it drives (switch it to Backwards to back up).');
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
    /** RMP1-J D1: M's menu in two steps (mech_menu.mjs); the step it adds is selected, its details on the right. */
    function openMechanismMenu() {
        if (!route().start) {
            toast('Place robot one\'s start first: click the field where it starts.');
            return;
        }
        const at = insertAt();
        openMechMenu(toolBtns.mechanism, robot(), route(), at, (step, words) => {
            edit(addStep(route(), step, at), at);
            toast(step.do === 'set' ? `Added: ${words}.` : step.do === 'move' ? 'Added: pick its height on the right.'
                : step.do === 'run' ? 'Added: type its volts and how long, on the right.' : 'Added: type its time limit on the right.');
        });
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
     * `keepControls` (RMP1-B): a robot whose controls were changed here and are not in `p` keeps them. Returns what
     * happened to the Controls changes, in words ('' when there were none).
     */
    function adoptProject(p, folderText, keepControls = false) {
        const before = { project, hist, folderBase, selected: ui.selected, point: ui.point, saveState: ui.saveState, coach: ui.coach };
        project = p;
        hist = commit(hist, setOf(p));
        if (folderText !== undefined) {
            folderBase = folderText;
        }
        pick(null);
        ui.focus = false;
        ui.saveState = 'saved';
        ui.coach = false;
        try {
            render();
        } catch (e) {
            ({ project, hist, folderBase } = before);
            pick(before.selected, before.point);
            ui.saveState = before.saveState;
            ui.coach = before.coach;
            render();
            throw e;
        }
        persistDraft();
        store(SAVED_KEY, saveProject(app.project()));
        return controlsTab.reload(keepControls);  // RMP1-B: the opened project's controls; the page's one Ctrl+Z away
    }
    /** Make the folder's project text the page's route (`said`: the toast, with what happened to the Controls). true: it is the page's now. */
    function adoptFolderText(text, { keepControls = false, said = '' } = {}) {
        const l = loadProject(text);
        if (!l.ok) {
            toast(`${PROJECT_FILE} in the folder was not opened: ${l.words}`, 'error');
            return false;
        }
        try {
            const note = [said, adoptProject(l.project, text, keepControls)].filter(Boolean).join(' ');
            if (note) {
                toast(note);
            }
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
     * An empty route: the folder's project opens (Controls changed here and not in it are kept). Two different routes:
     * the student chooses — the page never shows a stale draft as the team's route without asking.
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
            adoptFolderText(disk, { keepControls: true, said: `Opened ${PROJECT_FILE} from the folder.` });  // Controls changes kept
            return;
        }
        const controls = Object.values(app.project().robots || {}).some((x) => x && x.controls);
        const choice = await ask({
            title: `The folder has a different ${controls ? 'project' : 'route'}`,
            words: `${PROJECT_FILE} in ${files.folderName()} is not the route${controls ? ' and Controls' : ''} on this page. Open the folder's (what the team `
                + `last saved there, a git pull included${controls ? '; Ctrl+Z on the Controls tab brings this page\'s back' : ''}), or keep this page's (Save then replaces the file).`,
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
            let note;
            try {
                note = adoptProject(out.project, out.where === 'folder' ? out.text : undefined);
            } catch (e) {
                toast(`Not opened: this page cannot show the route in that file (${e.message}). The page is as it was.`, 'error');
                return;
            }
            toast(`Opened the project.${note ? ` ${note}` : ''}`);
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
            await controlsTab.folderConnected();
        }
        renderTop();
    }
    function undo() {
        if (ui.tab === 'controls') {
            controlsTab.undo();
            renderTop();
            renderStatus();
            return;
        }
        travel(undoH(hist));
    }
    function redo() {
        if (ui.tab === 'controls') {
            controlsTab.redo();
            renderTop();
            renderStatus();
            return;
        }
        travel(redoH(hist));
    }
    /** Undo / redo: the step at the selected place may be another one now (its point was never picked), or the route
     *  on the field another route (nothing of it was selected). */
    function travel(h) {
        const same = h.present.at === hist.present.at;
        hist = h;
        pick(same ? ui.selected : null);
        ui.focus = ui.focus && same;
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
    const controlsTab = createControlsTab(app);
    els.app.insertBefore(controlsTab.element, els.status);
    els.fieldView = createFieldView(els.fieldPane, app);
    els.fieldPane.append(els.toolbar, els.hud, els.zoom, els.modeHint, els.coach);
    app.refresh = () => render();  // RMP1-A2u: PREVIEW redraws the shell (the step list lit, a run put away)
    app.preview = createPreview(app, els, engine, baked, toast);
    app.sendPanel = local ? createSendPanel({ send: (r, onLine) => sendToRobot(local, r, onLine), log: () => sendLog(local),
        onChange: (end) => {
            renderTop();
            if (end && !app.sendPanel.element.isConnected) {  // the drawer was closed mid-send: say how it ended
                toast(`${end.kind === 'uploaded' ? '✓' : '✗'} ${end.words}${end.then ? `: ${end.then}` : ''}`, end.kind === 'uploaded' ? '' : 'error');
            }
        } }) : null;

    /** RMP1-I: a send is running — nothing is written (Save, Export) until it ends: what it builds is what it started with. */
    const sending = () => !!(app.sendPanel && app.sendPanel.busy());
    const SENDING = 'A send to the robot is running: wait for it to finish.';
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
            h('div', { class: 'tabs', role: 'tablist', 'aria-label': 'What to build' },
              [['route', 'Route', 'curve'], ['controls', 'Controls', 'mechanism']].map(([id, t, ic]) => h('button', {
                  role: 'tab', class: `tab${ui.tab === id ? ' on' : ''}`, 'aria-selected': String(ui.tab === id), id: `tab-${id}`,
                  onclick: () => showTab(id) }, icon(ic), t))),
            ui.tab === 'route' ? h('span', { class: 'chip robot hide-narrow', title: 'The route is written in robot one\'s numbers' }, 'Robot one') : null,
            ui.tab === 'route' ? routesMenu.button() : null,
            h('span', { class: `chip ${saveWords[1]}`, id: 'save-state', role: 'status' }, saveWords[0]),
            h('span', { class: 'spacer' }),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Undo (Ctrl+Z)', title: 'Undo (Ctrl+Z)', disabled: ui.tab === 'controls' ? !controlsTab.canUndo() : !canUndo(hist), onclick: undo }, icon('undo')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Redo (Ctrl+Shift+Z)', title: 'Redo (Ctrl+Shift+Z)', disabled: ui.tab === 'controls' ? !controlsTab.canRedo() : !canRedo(hist), onclick: redo }, icon('redo')),
            h('span', { class: 'sep' }),
            files.canWriteFolder
                ? h('button', { class: 'btn hide-narrow', onclick: connect, title: capabilityWords(caps),
                    'aria-label': files.connected() ? files.folderName() : 'Connect your shulib folder' }, icon('folder'),
                    h('span', { class: 'cap-words' }, files.connected() ? files.folderName() : 'Connect your shulib folder'))
                : h('span', { class: 'chip cap hide-narrow', title: capabilityWords(caps) }, 'Save downloads · Export copies'),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Open a project', title: 'Open a project', onclick: openProj }, icon('open')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Save (Ctrl+S)', title: sending() ? SENDING : 'Save (Ctrl+S)', disabled: sending(), onclick: save }, icon('save')),
            h('button', { class: 'btn icon ghost', 'aria-label': dark ? 'Light theme' : 'Dark theme', title: dark ? 'Light theme' : 'Dark theme', onclick: theme }, icon(dark ? 'sun' : 'moon')),
            h('button', { class: 'btn icon ghost', 'aria-label': 'Help and shortcuts (?)', title: 'Help and shortcuts (?)', onclick: () => openHelp(app) }, icon('help')),
            h('span', { class: 'sep' }),
            ui.tab === 'route' ? app.preview.topbar() : null,  // PREVIEW plays the route: not on the Controls tab
            exportButton(),
            sendButton(),
            h('button', { class: 'btn icon ghost pane-toggle', 'aria-label': 'Step details', onclick: () => { els.app.dataset.right = els.app.dataset.right ? '' : 'open'; } }, icon('panel')));
    }
    /** Export: always clickable (it lists what to fix, each one click away); with errors it carries their count. */
    function exportButton() {
        if (ui.tab === 'controls') {
            const n = controlsTab.errors().length;
            return h('button', { class: `btn big primary${n ? ' has-errors' : ''}`, id: 'export', disabled: sending(), onclick: () => controlsTab.openExport(),
                                 title: n ? `Fix ${n === 1 ? 'this' : `these ${n}`} first (Ctrl+E lists them)` : 'Export the driver file\'s controls and feel (Ctrl+E)' },
                     icon('code'), 'Export', n ? h('span', { class: 'count', 'aria-label': `${n} error${n === 1 ? '' : 's'} to fix first` }, String(n)) : null);
        }
        const errs = derive().issues.filter((x) => x.level === 'error');
        const first = errs.slice(0, 3).map((x) => `${x.stepIndex >= 0 ? `step ${x.stepIndex + 1}: ` : ''}${x.words}`);
        return h('button', { class: `btn big${app.sendPanel ? '' : ' primary'}${errs.length ? ' has-errors' : ''}`, id: 'export', disabled: sending(), onclick: () => openExportDrawer(app),
                             title: errs.length ? `Fix ${errs.length === 1 ? 'this' : `these ${errs.length}`} first (Ctrl+E lists them): ${first.join(' · ')}`
                                                : 'Export the C++ (Ctrl+E)' },
                 icon('code'), 'Export',
                 errs.length ? h('span', { class: 'count', 'aria-label': `${errs.length} error${errs.length === 1 ? '' : 's'} to fix first` }, String(errs.length)) : null);
    }
    /** RMP1-I: Send to robot — writes the routes (when they changed), then builds and uploads (the export drawer shows it). */
    function sendButton() {
        const panel = app.sendPanel;
        if (!panel || ui.tab !== 'route') {
            return null;
        }
        return h('button', { class: 'btn big primary', id: 'send', disabled: panel.busy(), onclick: () => openExportDrawer(app, { send: panel.robot() }),
                             title: `Write the routes into src/team/skills_route.cpp, then build and upload them to robot ${panel.robot()}` },
                 icon('send'), panel.busy() ? 'Sending…' : `Send to robot ${panel.robot()}`);
    }
    function renderStatus() {
        if (ui.tab === 'controls') {
            const n = controlsTab.errors().length;
            mount(els.status, n ? h('span', { class: 'chip error' }, icon('error'), `${n} error${n === 1 ? '' : 's'}`) : h('span', { class: 'chip ok' }, icon('check'), 'Ready to export'),
                h('span', { class: 'build', title: 'The robots and the library this page was built with' },
                  `Library API ${baked.build.api} · built from ${baked.build.commit} on ${baked.build.date}`));
            return;
        }
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
    /** Show the Route or the Controls tab (RMP1-B). */
    function showTab(id) {
        ui.tab = id;
        store(TAB_KEY, id);
        render();
    }
    function render() {
        els.app.dataset.tab = ui.tab;
        controlsTab.show(ui.tab === 'controls');
        if (ui.tab === 'controls') {
            renderTop();
            renderStatus();
            return;
        }
        derived = null;
        for (const [id, b] of Object.entries(toolBtns)) {
            b.setAttribute('aria-pressed', String(ui.tool === id));
        }
        els.snapBtn.setAttribute('aria-pressed', String(ui.snap));
        els.focusBtn.setAttribute('aria-pressed', String(ui.focus));  // RMP1-G
        const hints = { driveTo: 'Click where the robot drives to', reverseTo: 'Click where the robot backs up to', face: 'Click the point to turn and face',
                        curveTo: ui.curveDraft.length ? `${ui.curveDraft.length} point${ui.curveDraft.length === 1 ? '' : 's'} — click more; Enter or double-click finishes; Esc cancels`
                                                      : 'Click the curve\'s points; Enter or double-click finishes' };
        const noStart = !route().start;
        const startHole = !noStart && !startPose(route().start);  // a hand-edited file's start missing a number
        const focusWords = ui.focus ? `Focus: ${ui.focusAt === 'start' ? 'the start' : `step ${ui.focusAt + 1}`} and its neighbours. Esc or O shows all` : '';  // RMP1-G
        els.modeHint.hidden = ui.coach || !(noStart || startHole || hints[ui.tool] || focusWords);  // the coach card says it already, over the same spot
        els.modeHint.textContent = noStart ? 'Click the field where robot one starts'
            : startHole ? 'Robot one\'s start is missing a number: select the start (the top of the list) and type it' : hints[ui.tool] || focusWords;
        mount(els.coach, ui.coach ? coachCard(app) : null);
        renderTop();
        renderStepList(els.steps, app);
        renderInspector(els.inspector, app);
        els.fieldView.render();
        renderStatus();
        document.documentElement.dataset.steps = String(route().steps.length);
        app.preview.sync();
    }

    // ── RMP1-G: hide, lock, focus and sections (the step list's, the inspector's and the keys' calls) ──────────
    Object.assign(app, {
        toggleHidden(i) {
            const on = !isHidden(route().steps[i]);
            edit(setView(route(), [i], { hidden: on }));
            toast(on ? `Step ${i + 1} is hidden on the field (it still runs). H shows it.` : `Step ${i + 1} shows again.`);
        },
        toggleLocked(i) {
            const on = !isLocked(route().steps[i]);
            edit(setView(route(), [i], { locked: on }));
            toast(on ? `Step ${i + 1} is locked: it cannot be dragged, and clicks pass through it. Its numbers can still be typed.` : `Step ${i + 1} is unlocked.`);
        },
        showAll() {
            edit(setView(route(), route().steps.map((_, k) => k), { hidden: false }));
        },
        setFocus(on) {
            if (on && ui.selected === null) {
                toast('Select a step first (click it in the list), then Focus.');
                return;
            }
            ui.focus = on;
            render();
        },
        /** A section's steps hidden (or locked) together: all of them, or none when all already are. */
        sectionSet(head, key) {
            const sec = sectionsOf(route()).find((x) => x.head === head);
            const idx = indicesOf(sec);
            const is = key === 'hidden' ? isHidden : isLocked;
            edit(setView(route(), idx, { [key]: !idx.every((k) => is(route().steps[k])) }));
        },
        setSection(i, patch) {
            edit(setView(route(), [i], patch));
        },
    });
    els.focusBtn = h('button', { class: 'tool', 'aria-pressed': 'false', 'data-tool': 'focus', title: 'Focus: only the selected step and its neighbours, the rest faded (O; Esc leaves)',
                                 'aria-label': 'Focus (O)', onclick: () => app.setFocus(!ui.focus) }, icon('target'), 'Focus', h('kbd', {}, 'O'));
    els.toolbar.append(els.focusBtn);
    /** H, L, O and Esc's focus (true: the key was this block's). */
    function viewKey(e) {
        const k = e.key.toLowerCase();
        const i = ui.selected;
        if ((k === 'h' || k === 'l') && !e.shiftKey) {
            if (typeof i === 'number') {
                (k === 'h' ? app.toggleHidden : app.toggleLocked)(i);
            } else {
                toast(`Select a step first, then ${k === 'h' ? 'H hides' : 'L locks'} it.`);
            }
            return true;
        }
        if (k === 'o' && !e.shiftKey) {
            app.setFocus(!ui.focus);
            return true;
        }
        if (e.key === 'Escape' && ui.focus && ui.tool === 'select' && !ui.curveDraft.length) {
            app.setFocus(false);
            return true;
        }
        return false;
    }
    // ── (end RMP1-G) ──

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
        if (ctrl && ['s', 'e'].includes(e.key.toLowerCase()) && sending()) {
            e.preventDefault();
            toast(SENDING);
            return;
        }
        if (ctrl && e.key.toLowerCase() === 's') {
            e.preventDefault();
            save();
            return;
        }
        if (ctrl && e.key.toLowerCase() === 'e') {
            e.preventDefault();
            if (ui.tab === 'controls') {
                controlsTab.openExport();
            } else {
                openExportDrawer(app);
            }
            return;
        }
        if (ui.tab === 'controls') {
            if (e.key === '?' && !typing) {
                openHelp(app);
            }
            return;  // the route's shortcuts are the Route tab's
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
        const arrow = { ArrowLeft: [-1, 0], ArrowRight: [1, 0], ArrowUp: [0, 1], ArrowDown: [0, -1] }[e.key];
        if (arrow && !app.preview.showing() && nudge(arrow[0] * (e.shiftKey ? 0.25 : 1), arrow[1] * (e.shiftKey ? 0.25 : 1))) {
            e.preventDefault();
            return;
        }
        if (viewKey(e)) {
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
        hist = historyOf(setOf(project));
        pick(null);
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
    // a folder remembered from last time (Chrome/Edge): compare its project file with the page; read today's driver files
    syncWithFolder().then(() => controlsTab.folderConnected());
    // for the page tests: the app is up, and a read-only look at its state
    // (and, for the file layer's page tests, an in-memory folder in place of the real one: the folder dialog itself
    // is the manual Windows check's, A.4)
    window.routeBuilder = { route: () => JSON.parse(JSON.stringify(route())), issues: () => derive().issues, ready: true,
                            routes: () => JSON.parse(JSON.stringify(routes())), at,
                            controls: () => controlsTab.snapshot(), tab: (id) => showTab(id),
                            useMemoryFolder: async (files0, opts) => {
                                const host = memoryHost(files0, opts || {});
                                files = makeFiles(host);
                                folderBase = undefined;
                                window.routeBuilder.memory = host;
                                render();
                                await syncWithFolder();
                                await controlsTab.folderConnected();
                                return true;
                            } };
    // RMP1-A2u: the page tests' robots and PREVIEW engine (a fixture robot with levels, a stand-in engine)
    const rebake = (patch) => { Object.assign(baked, patch); derived = null; render(); };
    Object.assign(window.routeBuilder, { useRobots: (r) => rebake({ robots: { ...baked.robots, ...r } }) }, app.preview.testHooks(rebake));
    document.documentElement.dataset.ready = '1';
}

main().catch((e) => {
    document.documentElement.dataset.ready = 'error';
    document.body.append(h('div', { class: 'too-old', role: 'alert' }, h('h1', {}, 'The route builder did not start'),
        h('p', {}, `${e && e.message ? e.message : e}. Reload the page; if it happens again, tell the coordinator with these words.`)));
    throw e;
});
