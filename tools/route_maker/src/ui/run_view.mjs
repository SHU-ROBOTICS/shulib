// run_view.mjs — SEE THE MECHANISMS WORK (RMP1-K), on the page tools/route_builder.py serves:
//   * Check the mechanisms (D1): the route's mechanism steps alone (check_route.mjs), said in one screen, then sent like
//     Send to robot — written beside the team's routes, never into the project;
//   * Watch the robot (D2): the brain's own lines (run_log.mjs) mark the step list — done, started, stopped (the robot's
//     words), failed and carried on, skipped — with one summary line coloured by the robot's own verdict, the END estimate
//     on the field (a LIVE one only), and the raw lines folded; an edit to the route after its run clears its marks;
//   * What is on robot one (D5): each slot against the wanted table.
// One idea per screen; the one highlighted button is the next thing to do.

import { h, s, icon, mount } from './dom.mjs';
import { cameraCard } from './camera_view.mjs';
import { checkRoute as buildCheck, checkName, checkWords } from '../model/check_route.mjs';
import { newWatch, readLine, summaryWords, runTone, pageMarks, routeVerbs, runDiffers } from '../model/run_log.mjs';
import { libraryCalls, exportRegion } from '../model/export.mjs';
import { checkRoute as issuesOf } from '../model/checks.mjs';
import { routeRobots } from '../host/robots.mjs';
import { fieldToPicture } from '../model/field.mjs';
import { squareCorners } from '../model/geometry.mjs';
import { reflect } from '../model/pair.mjs';
import { watchRobot, robotSlots } from '../host/files_local.mjs';

const KEEP_LINES = 2000;

/** A modal card: {el, body, close}. */
function dialog(title, onClose = () => {}) {
    const close = () => {
        scrim.remove();
        el.remove();
        document.removeEventListener('keydown', onKey, true);
        onClose();
    };
    const onKey = (e) => e.key === 'Escape' && (e.stopPropagation(), close());
    const scrim = h('div', { class: 'scrim', onclick: close });
    const body = h('div', { class: 'rk-body' });
    const el = h('div', { class: 'dialog rk-dialog', role: 'dialog', 'aria-modal': 'true', 'aria-labelledby': 'rk-title' },
        h('div', { class: 'rk-head' }, h('h2', { id: 'rk-title' }, title),
          h('button', { class: 'btn icon ghost', 'aria-label': 'Close', onclick: close }, icon('close'))), body);
    document.addEventListener('keydown', onKey, true);
    document.body.append(scrim, el);
    return { el, body, close };
}

/** The run view over `app` and the helper `local`; `fieldSvg()` is the field's SVG (the END estimate is drawn on it). */
export function createRunView(app, { local, fieldSvg }) {
    let watch = null;      // run_log.mjs's state while watching, and after
    let mode = 'idle';     // 'opening' | 'live' | 'paused' | 'ended'
    let said = '';         // the helper's last words (an error, a pause)
    let saved = '';
    let raw = [];
    let ctrl = null;
    const el = h('section', { class: 'run-view', 'aria-live': 'polite', hidden: true });

    const robotsFor = (r) => (routeRobots(app.baked, r).length ? routeRobots(app.baked, r) : [app.robot()]);
    const asRun = new WeakMap();  // run → its route's C++ when the page first marked it: an edit since clears the marks
    const sig = (route) => {
        try {
            return exportRegion([route], app.robot());
        } catch {
            return JSON.stringify(route.steps);
        }
    };
    const EDITED = 'You changed this route after the run: its marks are cleared. Send it, then run it again.';
    const sendRobots = () => ['one', ...((app.baked.sharedRoute && app.baked.sharedRoute.robots || []).includes('two') ? ['two'] : [])];

    /** The last run, and its marks on the route on the field ([] when the run is of another route). */
    function marks() {
        const run = watch && watch.runs[watch.runs.length - 1];
        const route = app.route();
        if (!run) {
            return { run: null, list: [] };
        }
        const mine = run.name === route.name || run.name === checkName(route, app.robot());
        if (mine && !asRun.has(run)) {
            asRun.set(run, sig(route));
        }
        if (mine && asRun.get(run) !== sig(route)) {
            return { run, list: [], differs: EDITED, check: run.name !== route.name };
        }
        if (run.name === route.name) {
            const differs = runDiffers(run, routeVerbs(route, app.robot()));
            return { run, differs, list: differs ? [] : pageMarks(run, route.steps.map(libraryCalls)) };
        }
        if (run.name === checkName(route, app.robot())) {
            const c = buildCheck(route, app.robot(), app.routes().filter((r) => r !== route));
            const differs = c.route ? runDiffers(run, routeVerbs(c.route, app.robot())) : c.why;
            const list = [];
            if (!differs) {
                pageMarks(run, c.route.steps.map(() => 1)).forEach((m, k) => {
                    if (c.from[k] >= 0) {
                        list[c.from[k]] = m;
                    }
                });
            }
            return { run, list, check: true, differs };
        }
        return { run, list: [], other: run.name };
    }

    /** The step list's run marks (step_list.mjs calls this for each card); the panel follows an edit (its first card). */
    let shownDiffers = '';
    function markCard(card, i) {
        const k = marks();
        if (i === 0 && !el.hidden && (k.differs || '') !== shownDiffers) {
            requestAnimationFrame(render);
        }
        const m = k.list[i];
        if (!m) {
            return;
        }
        const words = m.mech ? `On the robot: ${m.words}`  // a started mechanism's own end
            : m.state === 'done' ? `On the robot: done${m.seconds !== undefined ? ` in ${m.seconds.toFixed(2)} s` : ''}${m.at ? `, at ${m.at}` : ''}${m.outcome ? ` (${m.outcome})` : ''}`
                : m.state === 'started' ? 'On the robot: started (not finished yet)'
                    : m.state === 'stopped' ? `Stopped here on the robot: ${m.words}` : m.state === 'failed' ? `Failed on the robot: ${m.words} — carried on`
                        : 'Skipped on the robot';
        card.classList.add(`rv-${m.state}`);
        card.querySelector('.step-main').append(h('div', { class: 'step-robot', 'data-state': m.state },
            icon({ done: 'check', skipped: 'minus', failed: 'warn', started: 'play' }[m.state] || 'error'), h('span', {}, words)));
    }

    /** The robot's END estimate on the field, beside the plan (robot two's, reflected back into robot one's numbers). */
    function drawEnd(run) {
        const svg = fieldSvg();
        const old = svg && svg.querySelector('.rk-end');
        if (old) {
            old.remove();
        }
        if (!svg || !run || !run.end || !run.end.live) {  // NOT LIVE: the robot itself does not trust it
            return;
        }
        const mirror = app.robot().skillsMirror;
        const e = watch.robot === 'two' && mirror ? reflect(run.end, mirror) : run.end;
        const P = (q) => fieldToPicture(q.x, q.y, app.picture);
        const pts = squareCorners(e.x, e.y, e.headingDeg, app.size()).map(P).map((q) => `${q.px},${q.py}`).join(' ');
        const c = P(e);
        svg.append(s('g', { class: 'rk-end' }, s('polygon', { points: pts }),
            s('text', { x: c.px, y: c.py, 'text-anchor': 'middle', 'dominant-baseline': 'middle' }, 'END')));
    }

    function render() {
        const { run, other, check, differs } = marks();
        shownDiffers = differs || '';
        const steps = run ? Object.keys(run.steps).length : 0;
        const summary = run ? summaryWords(run) : null;
        const head = mode === 'opening' ? 'Opening the robot\'s log…' : mode === 'paused' ? said
            : mode === 'ended' ? (said || 'Stopped watching.') : !run ? 'Watching the robot. On the brain: Auton Tester → the route → RUN.'
                : summary || `Run ${run.number} of "${run.name}"${steps ? `: ${steps} step${steps === 1 ? '' : 's'} so far` : ': running'}`;
        const how = !run ? null : summary ? null : run.live ? 'Live: each step ticks as the robot finishes it.'
            : 'Each step is marked when the robot says how it ended (an older program says it at the run\'s end).';
        const fold = h('details', { class: 'rk-log' }, h('summary', {}, 'Show the robot\'s log'),
            h('pre', { class: 'code' }, raw.slice(-400).join('\n') || 'Nothing yet.'), saved ? h('p', { class: 'hint' }, `Saved: ${saved}`) : null);
        const tone = mode === 'ended' && said ? 'bad' : runTone(run);  // the robot's own verdict first; carried on: amber, never a ✓
        mount(el,
            h('div', { class: `rk-status ${tone}` },
              icon(tone === 'bad' ? 'error' : tone === 'warn' ? 'warn' : tone === 'ok' ? 'check' : 'play'), h('strong', { id: 'rk-summary' }, head)),
            cameraCard(watch, raw, mode),  // RMP1-N: the Camera tab, only when the robot prints [CAM] lines
            how ? h('p', { class: 'rk-how' }, how) : null,
            run && run.end && !run.end.live ? h('p', { class: 'rk-how', id: 'rk-notlive' }, 'Its END estimate was NOT LIVE (the robot does not trust it): not drawn.') : null,
            other ? h('p', { class: 'rk-how' }, `This run is of "${other}": switch to that route to see its steps.`) : null,
            check && !differs ? h('p', { class: 'rk-how' }, 'A check of this route\'s mechanisms: its mechanism steps are marked.') : null,
            differs ? h('p', { class: 'rk-why', id: 'rk-differs' }, differs) : null,
            fold,
            h('div', { class: 'row' }, mode === 'ended'
                ? [h('button', { class: 'btn', onclick: () => startWatch() }, 'Watch again'), h('button', { class: 'btn ghost', onclick: hide }, 'Close')]
                : h('button', { class: 'btn ghost', onclick: stopWatch }, 'Stop watching')));
        el.hidden = false;
        drawEnd(run);
    }

    let pending = false;
    function refresh() {  // at most one repaint per frame, however fast the lines come
        if (!pending) {
            pending = true;
            requestAnimationFrame(() => {
                pending = false;
                render();
                app.refresh();
            });
        }
    }

    /** Start watching the brain (a watch already open: just show it). */
    async function startWatch() {
        if (mode === 'opening' || mode === 'live' || mode === 'paused') {
            el.hidden = false;
            return;
        }
        ctrl = new AbortController();
        watch = newWatch();
        raw = [];
        said = '';
        mode = 'opening';
        render();
        try {
            await watchRobot(local, (m) => {
                if (m.saved) {
                    saved = m.saved;
                } else if (m.open) {
                    mode = 'live';
                } else if (m.paused) {
                    mode = 'paused';
                    said = m.paused;
                } else if (m.error) {
                    said = m.error;
                } else if (typeof m.line === 'string') {
                    raw.push(m.line);
                    raw = raw.length > KEEP_LINES ? raw.slice(-KEEP_LINES) : raw;
                    readLine(watch, m.line);
                }
                refresh();
            }, { signal: ctrl.signal });
        } catch (e) {
            if (!ctrl.signal.aborted) {
                said = e && e.message ? e.message : String(e);
            }
        }
        mode = 'ended';
        refresh();
    }
    function stopWatch() {
        if (ctrl) {
            ctrl.abort();
        }
        mode = 'ended';
        render();
    }
    function hide() {
        el.hidden = true;
        watch = null;
        drawEnd(null);
        app.refresh();
    }

    /** Check the mechanisms: the one screen that says what will move, then Send. */
    function check() {
        const route = app.route();
        const robot = app.robot();
        const d = dialog(`Check the mechanisms of "${route.name}"`);
        const c = buildCheck(route, robot, app.routes());
        const errs = c.route ? [...app.issues().filter((x) => x.level === 'error'), ...app.otherErrors(),
                                ...issuesOf(c.route, robotsFor(c.route), app.field(), { routes: [...app.routes(), c.route], index: app.routes().length })
                                    .filter((x) => x.level === 'error')] : [];
        if (!c.route || errs.length) {
            mount(d.body, h('p', { class: 'rk-why' }, c.route ? `Fix the routes' errors first (Export lists them): ${errs[0].words}` : c.why));
            return;
        }
        const words = checkWords(c.route, robot).split('; ');
        const go = sendRobots().map((r, k) => h('button', { class: `btn${k === 0 ? ' primary' : ''}`, 'data-check': r, onclick: () => send(r) },
            icon('send'), `Send the check to robot ${r}`));
        mount(d.body,
            h('p', {}, 'On the brain it is ', h('strong', {}, `"${c.route.name}"`), '. It moves:'),
            h('ul', { class: 'rk-moves' }, words.map((w) => h('li', {}, w))),
            h('p', { class: 'rk-safe' }, icon('warn'), 'The robot stays where it is — clear the mechanisms\' paths.'),
            h('div', { class: 'row' }, go));
        go[0].focus();
        async function send(robotId) {
            go.forEach((b) => { b.disabled = true; });
            const files = app.files();
            const out = await files.exportToFile([...app.routes(), c.route], app.project(), { robot });
            if (!out.ok) {
                mount(d.body, h('p', { class: 'rk-why' }, `Not written: ${out.words}`));
                return;
            }
            app.setProject(out.project, 'exported');
            await app.saveQuietly().catch(() => false);
            mount(d.body, h('p', {}, `Sending "${c.route.name}" to robot ${robotId}…`), app.sendPanel.element,
                h('p', { class: 'hint' }, 'When the upload ends, the check comes out of src/team/skills_route.cpp again.'));
            await app.sendPanel.start(robotId);
            // The check is the brain's alone: the team's file is written back without it, however the send ended.
            const back = await files.exportToFile(app.routes(), app.project(), { robot }).catch((e) => ({ ok: false, words: e.message }));
            if (back.ok) {
                app.setProject(back.project, 'exported');
                await app.saveQuietly().catch(() => false);
            }
            const said = back.ok ? 'src/team/skills_route.cpp holds the team\'s routes again (the check is only on the brain).'
                : `src/team/skills_route.cpp STILL holds the check (${back.words}): Export before you build Competition.`;
            d.body.append(h('p', { class: back.ok ? 'hint' : 'rk-why', id: 'rk-back' }, said));
            if (!back.ok) {
                app.toast(said, 'error');
            }
        }
    }

    /** What is on robot `robot`: each slot against the wanted table, the first wrong one highlighted. */
    async function slots(robot) {
        const d = dialog(`What is on robot ${robot}`);
        mount(d.body, h('p', {}, 'Reading the brain\'s programs over USB…'));
        let got;
        try {
            got = await robotSlots(local, robot);
        } catch (e) {
            got = { rows: null, why: e && e.message ? e.message : String(e) };
        }
        if (!got.rows) {
            mount(d.body, h('p', { class: 'rk-why' }, got.why));
            return;
        }
        const firstBad = got.rows.find((r) => !r.ok);
        mount(d.body,
            h('p', { class: `rk-verdict ${firstBad ? 'bad' : 'ok'}` }, firstBad ? `Fix this first: ${firstBad.fix}.` : 'Every slot holds what it should.'),
            h('table', { class: 'rk-slots' }, h('thead', {}, h('tr', {}, h('th', {}, 'Slot'), h('th', {}, 'Wanted'), h('th', {}, 'On the brain'))),
              h('tbody', {}, got.rows.map((r) => h('tr', { class: `${r.ok ? 'ok' : 'bad'}${r === firstBad ? ' first' : ''}`, 'data-slot': r.slot },
                  h('td', {}, String(r.slot)), h('td', {}, r.want),
                  h('td', {}, h('span', { class: 'rk-mark' }, r.ok ? '✓' : '✗'), ' ', r.unread ? 'could not be read' : r.have || 'empty',
                    r.date || r.description ? h('small', {}, [r.date ? `uploaded ${r.date.slice(0, 16).replace('T', ' ')}` : '', r.description].filter(Boolean).join(' · ')) : null))))),
            h('p', { class: 'hint' }, 'Read-only: nothing on the brain was changed.'));
    }

    return { element: el, markCard, watch: startWatch, stopWatch, check, slots, state: () => ({ watch, mode, said }) };
}
