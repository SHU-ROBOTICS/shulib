// preview.mjs — PREVIEW in the page (RMP1-A2u; brief D5, D6, A.3.10, §1.4): the button, the run through the engine
// (engine.mjs: off the page's thread, with progress and Cancel), then the robot playing on the field from the
// record — its path solid beside the dashed plan, robot two mirrored beside it with the pair check's band — the step
// list lit from the record's spans, and the keys (Space, Left / Right, Shift for 0.1 s, Home / End, Esc, Ctrl+Enter).
// A run belongs to the route it ran: change the route and the run is put away (an undo back to it brings it back).

import { h, icon } from './dom.mjs';
import { createDock } from './timeline.mjs';
import { createStrip } from './mech_strip.mjs';
import { mechName, stepTitle } from './words.mjs';
import { routeRobots } from '../host/robots.mjs';
import { makeEngine } from '../host/engine.mjs';
import { routeKey, playbackOf, poseAt, sampleIndexAt, stepAt, stepTime, mechanismsAt } from '../model/playback.mjs';
import { plannedHeights } from '../model/lift.mjs';
import { ROBOT_TWO_ONLY } from '../model/checks.mjs';
import { frontBayOf } from '../model/obstacles.mjs';
import { measureEmptyLimits } from '../model/mech_steps.mjs';
import { robotName } from '../model/robot_names.mjs';

const PHASES = {
    starting: 'Starting the preview engine…',
    running: 'Running the route through the real library on the host test plant…',
    two: `Running ${robotName('two')}'s mirrored route…`,
};
const TITLE = 'PREVIEW: run this route through the real library and watch the robot drive it here (Ctrl+Enter)';
/** PREVIEW without robot two's run: not in this build (`where`), or not in its engine (the worker's `noPair`). */
const noTwo = (where) => `PREVIEW plays ${robotName('one')}: ${robotName('two')}'s mirrored run is not in this ${where} yet.`;
/** An engine built before RMP1-L words it itself (no `noPair`), in the old names: said in the page's. */
const oldNoPair = (w) => /mirrored run is not in this preview engine yet\.$/.test(w || '');

/** The controller. `app` is the shell (app.mjs), `els` its elements, `engine` engine.mjs's, `toast` the shell's. */
export function createPreview(app, els, engine, baked, toast) {
    const st = { phase: 'idle', play: null, key: '', t: 0, playing: false, speed: 1, stale: false, words: '', progress: '',
                 details: false, pairWords: '', stepIndex: -1, scrubbing: false, mechs: [] };
    const cache = new Map();
    let latest = null;  // RMP1-J D4: the last run opened (its route and play), for the time limits' suggestions
    let raf = 0;
    let last = 0;
    const reduced = () => window.matchMedia('(prefers-reduced-motion: reduce)').matches;

    const btn = h('button', { class: 'btn big', id: 'preview', 'aria-describedby': 'preview-why', onclick: () => run() }, icon('play'), 'PREVIEW');
    const why = h('span', { class: 'sr-only', id: 'preview-why' });
    function renderButton() {
        btn.disabled = !engine.available || st.phase === 'running';
        btn.title = engine.available ? TITLE : engine.why;
        why.textContent = btn.title;
        btn.setAttribute('aria-busy', String(st.phase === 'running'));
        btn.classList.toggle('accent', engine.available);
    }

    const showing = () => st.phase === 'open' && st.play && !st.stale;
    const dock = createDock(els.fieldPane, {
        state: () => st, seek, toggle, step, close, cancel: () => engine.cancel(),
        speed: (s) => { st.speed = s; dock.render(); },
        details: () => { st.details = !st.details; dock.render(); },
        scrub: (on) => { st.scrubbing = on; if (on) pause(); },
        stepName: (i) => stepTitle(app.route().steps[i] || {}, app.robot())[0],
    });
    const strip = createStrip(els.fieldPane);
    /** RMP1-E: the mechanism step `i` of the run's route uses and its planned target (run() keeps them), or null. */
    const playingMech = (i) => (i >= 0 && st.mechs[i]) || null;

    // ── the frame: the robots on the field, the dock's playhead, the step playing ──
    function frame() {
        if (!showing()) {
            els.fieldView.drawRun(null);
            strip.show(null);
            return;
        }
        const p = st.play;
        const robot = (run) => run && { id: run.id, pose: poseAt(run.samples, st.t), path: run.path, upTo: sampleIndexAt(run.samples, st.t),
                                        throughAt: run.through ? run.through.s : null,
                                        size: ((baked.robots[run.record.robot] || {}).drive || {}).footprintIn || 15,
                                        robot: baked.robots[run.record.robot] };  // RMP1-O: its claw
        els.fieldView.drawRun({ key: st.key, robots: [robot(p.one), robot(p.two)].filter(Boolean), pair: p.pair });
        const i = stepAt(p, st.t);
        if (i !== st.stepIndex) {
            st.stepIndex = i;
            for (const c of document.querySelectorAll('.step[data-step]')) {
                c.classList.toggle('pv-playing', Number(c.dataset.step) === i);
            }
            const now = document.querySelector('.step.pv-playing');
            if (now && st.playing) {
                now.scrollIntoView({ block: 'nearest' });
            }
        }
        strip.show(p.one.record);
        strip.update(mechanismsAt(p.one.record, sampleIndexAt(p.one.samples, st.t), playingMech(i)));
        dock.update();
    }
    function tick(now) {
        const dt = Math.max(0, Math.min((now - last) / 1000, 0.1));  // a frame's stamp is its start: it can precede play()
        last = now;
        st.t = Math.min(st.t + dt * st.speed, st.play.duration);
        if (st.t >= st.play.duration) {
            st.playing = false;
        }
        frame();
        raf = st.playing ? requestAnimationFrame(tick) : 0;
    }
    function play() {
        if (!showing() || st.playing) {
            return;
        }
        if (st.t >= st.play.duration) {
            st.t = 0;
        }
        st.playing = true;
        last = performance.now();
        raf = requestAnimationFrame(tick);
        frame();
    }
    function pause() {
        st.playing = false;
        cancelAnimationFrame(raf);
        raf = 0;
        if (showing()) {
            frame();
        }
    }
    function toggle() {
        return st.playing ? pause() : play();
    }
    function seek(t) {
        if (showing()) {
            st.t = Math.min(Math.max(t, 0), st.play.duration);
            frame();
        }
    }
    function step(dir) {
        if (showing()) {
            const playing = st.playing;
            pause();
            seek(stepTime(st.play, st.t, dir, playing));
        }
    }
    function open(run) {
        latest = run;
        Object.assign(st, { phase: 'open', play: run.play, key: run.key, pairWords: run.pairWords, t: 0, stale: false,
                            words: '', stepIndex: -2, details: false, mechs: run.mechs });
        dock.render();
        app.refresh();
        frame();
        if (!reduced()) {
            play();
        }
        requestAnimationFrame(() => dock.element.querySelector('.track')?.focus({ preventScroll: true }));
    }
    function close() {
        pause();
        Object.assign(st, { phase: 'idle', play: null, key: '', stale: false, words: '' });
        dock.render();
        app.refresh();
    }

    async function run() {
        if (st.phase === 'running') {
            return;
        }
        if (!engine.available) {
            toast(engine.why);
            return;
        }
        const r = app.route();
        // RMP1-C: robot two's errors (together? too close? skills mirrored?) are the export's -- robot one runs the same route;
        // RMP1-J D4: an EMPTY mechanism time limit runs with none (the cut), so PREVIEW measures it -- Export still asks
        const err = app.issues().find((x) => x.level === 'error' && !ROBOT_TWO_ONLY.has(x.code) && !(x.field === 'boundS' && x.empty));
        const notReady = !r.start ? `Place ${robotName('one')}'s start first: click the field where it starts.`
            : !r.steps.length ? 'Add a step first: PREVIEW plays the route\'s steps.'
                : err ? `PREVIEW runs a route the robot would run: fix ${err.stepIndex >= 0 ? `step ${err.stepIndex + 1}` : 'the start'} first. ${err.words}` : '';
        if (notReady) {
            toast(notReady, 'error');
            if (err) {
                app.select(err.stepIndex >= 0 ? err.stepIndex : 'start');
            }
            return;
        }
        const one = app.robot();
        const two = routeRobots(baked, r).find((x) => x !== one);
        const robots = { one, ...(two ? { two } : {}) };
        const key = routeKey(r);
        if (cache.has(key)) {
            open(cache.get(key));
            return;
        }
        pause();
        Object.assign(st, { phase: 'running', play: null, key: '', stale: false, progress: PHASES.starting, words: '' });
        renderButton();
        dock.render();
        app.refresh();
        const measured = measureEmptyLimits(r);
        let out;
        try {
            out = await engine.preview(measured.route, robots, (ph) => {
                st.progress = PHASES[ph] || st.progress;
                dock.render();
            });
        } catch (e) {
            out = { ok: false, words: `The preview engine stopped: ${e && e.message ? e.message : e}` };
        }
        st.phase = 'idle';
        renderButton();
        if (!out.ok) {
            st.words = out.cancelled ? '' : out.words;
            dock.render();
            toast(out.words, out.cancelled ? '' : 'error');
            return;
        }
        if (!out.result.one || out.result.one.schema !== 1) {
            st.words = 'This preview engine wrote a record this page cannot read. Reload the page; if it happens again, tell the coordinator.';
            dock.render();
            return;
        }
        const res = { ...out.result };
        const moved = (rec) => rec && (rec.samples || []).length > 0;
        if (!moved(res.one)) {  // refused before the first pace: nothing to play
            st.words = `The library refused this route before the robot moved (${res.one.word}): ${res.one.why || 'no reason given'}`;
            dock.render();
            toast(st.words, 'error');
            return;
        }
        let pairWords = out.noPair || oldNoPair(out.pairWords) ? noTwo('preview engine') : out.pairWords || (r.mirroredFor && !robots.two ? noTwo('build of the route builder') : '');
        if (res.two && !moved(res.two)) {
            pairWords = `${robotName('two')}'s mirrored run was refused (${res.two.word}): ${res.two.why || 'no reason given'}`;
            Object.assign(res, { two: null, verdict: null });
        }
        const names = (robot) => r.steps.map((x) => ((robot.mechanisms || []).find((m) => m.ident === x.mechanism) || {}).name || '');
        const planned = plannedHeights(r, one);  // the route that RAN: an edit made during the run never reaches its strip
        const mechs = r.steps.map((x, i) => (x.mechanism ? { name: mechName(one, x.mechanism), target: planned[i].after[x.mechanism] ?? null } : null));
        const bays = Object.fromEntries(Object.entries(baked.robots).map(([id, x]) => [id, frontBayOf(x)]));  // RMP1-Q
        const solid = { obstacles: (one.field || {}).obstacles || [], size: (one.drive || {}).footprintIn || 15, bays };  // RMP1-H
        const { filled } = measured;
        const runRec = { key, play: playbackOf(res, { one: names(one), two: two ? names(two) : [] }, solid), pairWords, mechs, route: r, filled };
        if (r.together === false && runRec.play.pair) {  // RMP1-C D6: one robot at a time -- robot two's run shown, no gap verdict
            runRec.play.pair = null;
            runRec.pairWords = `One robot at a time (this route's answer): ${robotName('two')}'s mirrored run is shown, the too-close check is not.`;
        }
        cache.set(key, runRec);
        if (cache.size > 8) {
            cache.delete(cache.keys().next().value);
        }
        open(runRec);
        if (filled) {
            toast(`PREVIEW ran ${filled === 1 ? 'a step' : `${filled} steps`} with no time limit yet, to time ${filled === 1 ? 'it' : 'them'}: `
                + 'select one to use its suggestion.');
        }
    }

    // ── the keys (only on the Route tab, never while typing or in a dialog) ──
    let space = null;
    document.addEventListener('keydown', (e) => {
        const a = document.activeElement;
        if (/INPUT|SELECT|TEXTAREA/.test(a.tagName) || els.app.dataset.tab === 'controls'
            || document.querySelector('.drawer, .dialog, .popover')) {
            return;
        }
        if ((e.ctrlKey || e.metaKey) && e.key === 'Enter') {
            e.preventDefault();
            run();
            return;
        }
        if (e.ctrlKey || e.metaKey || e.altKey) {
            return;
        }
        if (e.key === 'Escape' && st.phase === 'running') {
            engine.cancel();
            return;
        }
        if (e.key === 'Escape' && (st.phase === 'open' || st.words) && app.state().tool === 'select') {
            close();
            return;
        }
        if (!showing()) {
            return;
        }
        if (e.key === ' ' && !e.repeat && !a.closest('button, a, [role="button"]')) {
            e.preventDefault();
            space = { panned: false };
        } else if (e.key === 'ArrowLeft' || e.key === 'ArrowRight') {
            e.preventDefault();
            const dir = e.key === 'ArrowRight' ? 1 : -1;
            if (e.shiftKey) {
                pause();
                seek(st.t + 0.1 * dir);
            } else {
                step(dir);
            }
        } else if (e.key === 'Home' || e.key === 'End') {
            e.preventDefault();
            pause();
            seek(e.key === 'Home' ? 0 : st.play.duration);
        }
    });
    document.addEventListener('pointerdown', () => space && (space.panned = true), true);
    document.addEventListener('keyup', (e) => {
        if (e.key === ' ' && space) {
            const tapped = !space.panned;
            space = null;
            if (tapped) {
                toggle();
            }
        }
    });

    let keyOf = { route: null, key: '' };
    const currentKey = () => {
        if (keyOf.route !== app.route()) {
            keyOf = { route: app.route(), key: routeKey(app.route()) };
        }
        return keyOf.key;
    };

    renderButton();
    return {
        available: () => engine.available,
        showing,  // RMP1-C: while a run shows, the arrow keys are its (the field's nudge waits)
        topbar: () => [btn, why],
        run,
        /** After every redraw of the shell: a run whose route changed is put away (and comes back on an undo). */
        sync() {
            if (st.phase === 'open' && st.play) {
                const stale = currentKey() !== st.key;
                if (stale !== st.stale) {
                    st.stale = stale;
                    if (stale) {
                        pause();
                    }
                    dock.render();
                    app.refresh();
                }
            }
            st.stepIndex = -2;
            frame();
        },
        /** A step card was clicked: while a run shows, go to where that step began. */
        selected(i) {
            const s = showing() && st.play.one.steps[i];
            if (s && s.t0 !== null) {
                pause();
                seek(s.t0);
            }
        },
        /** How step `i` went in the run on screen (null: no run showing). */
        stepInfo(i) {
            return showing() ? st.play.one.steps[i] || null : null;
        },
        /** RMP1-J D4: the last run opened, even put away ({route, play}), or null. */
        lastRun: () => latest,
        /** For the page tests (as useMemoryFolder): another engine, and the state. */
        testHooks(rebake) {
            return {
                useEngine({ text, timeoutMs, sharedRoute }) {
                    engine = makeEngine({ source: { text }, timeoutMs });
                    if (sharedRoute) {
                        rebake({ sharedRoute });
                    }
                    cache.clear();
                    renderButton();
                    return true;
                },
                preview: () => ({ phase: st.phase, t: st.t, playing: st.playing, stale: st.stale, step: st.stepIndex, speed: st.speed,
                                  duration: st.play && st.play.duration, words: st.words, pairWords: st.pairWords,
                                  two: !!(st.play && st.play.two), instance: st.play && st.play.one.record.standinInstance,
                                  run: st.play && st.play.one.record.standinRun }),
                previewRecord: () => st.play && st.play.one.record,
                pause,
                /** One frame `ms` after the last (the speed's arithmetic, without the screen's clock). */
                frameAfter(ms) {
                    const now = performance.now();
                    last = now - ms;
                    tick(now);
                },
            };
        },
    };
}
