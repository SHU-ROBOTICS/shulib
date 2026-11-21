// mech_tab.mjs — THE MECHANISMS TAB (RMP1-M): one card per mechanism of the robot chosen — what it is, its ports, its
// buttons, its levels, the routes that use it — and Test (one mechanism, or every one): a test route through RMP1-K's
// check (the same "what will move" screen, Send and write-back), then the robot's own lines tick the card.

import { h, icon, mount } from './dom.mjs';
import { fmt } from '../model/numbers.mjs';
import { unitOf, homeEnd } from '../model/lift.mjs';
import { testRefusal, testerRefusal, testResults, OLD_FACTS } from '../model/check_route.mjs';

const KEY = 'shulib-route-builder:mech-tests';  // the last result of each card, with its time (this browser only)
const KIND_ICON = { lift: 'lift', arm: 'arm', power: 'motor', pneumatic: 'piston', flywheel: 'motor' };
const TONE_ICON = { ok: 'check', warn: 'warn', bad: 'error', '': 'play' };

/** A robot as the page names it (one place: RMP1-L's names replace this line). */
export const robotWords = (which) => ({ one: 'robot one', two: 'robot two' }[which] || `robot ${which}`);

/** What `m` is, in a few plain words. */
export function whatItIs(m, held) {
    switch (m.kind) {
        case 'lift':
        case 'arm':
            return `${m.kind === 'lift' ? 'a lift' : 'an arm'}: ${fmt(m.range[0])} to ${fmt(m.range[1])} ${unitOf(m)}`;
        case 'power':
            return held ? 'a motor that runs while a button is held' : 'a motor';
        case 'pneumatic':
            return `a piston: ${m.positions.join(' / ')}`;
        default:
            return `a ${m.kind}`;
    }
}

/** Its ports, one entry each: "port 10, turns reversed", "3-wire port A"; an owed fact said as owed. */
export function portWords(m) {
    const w = m.wiring;
    if (!w) {
        return ['not in this page\'s robot facts (made before the Mechanisms tab)'];
    }
    const out = w.motors.map((x) => (x.port === null ? 'port not known yet: the builders\' answer'
        : `port ${x.port}, ${x.reversed === null ? 'which way it turns not known yet' : x.reversed ? 'turns reversed' : 'turns forward'}`));
    out.push(...w.lines.map((l) => (l === null ? '3-wire port not known yet: the builders\' answer' : `3-wire port ${l}`)));
    if (w.levels === null) {
        out.push('which position is which not known yet');
    }
    const side = (what, p) => (p === null ? `${what}: not known yet` : `${what}: ${typeof p === 'string' ? '3-wire port ' : 'port '}${p}`);
    if (w.homeSwitch !== undefined) {
        out.push(side(`${homeEnd(m)} switch`, w.homeSwitch));
    }
    if (w.angleSensor !== undefined) {
        out.push(side('angle sensor', w.angleSensor));
    }
    if (w.pieceSensor !== undefined) {
        out.push(side('piece sensor', w.pieceSensor));
    }
    return out;
}

/** Its levels: "low 8 in"; an owed one "not measured yet"; none: "no levels yet". Null for a kind without levels. */
export function levelWords(m) {
    if (m.kind !== 'lift' && m.kind !== 'arm') {
        return null;
    }
    const ps = m.presets || [];
    return ps.length ? ps.map((p) => `${p.name} ${p.owed ? 'not measured yet' : `${fmt(p.value)} ${unitOf(m)}`}`) : ['no levels yet'];
}

/** The routes (index, name) with a step on `ident`, and that step: [{k, name, step}]. */
export function routesUsing(routes, ident) {
    return routes.map((r, k) => ({ k, name: r.name, step: r.steps.findIndex((s) => s.mechanism === ident) })).filter((x) => x.step >= 0);
}

/** When a result was had: its time today, else its day too ("Nov 19, 4:59 PM"). */
export function whenWords(at, now = Date.now()) {
    const d = new Date(at);
    const time = d.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    return d.toDateString() === new Date(now).toDateString() ? time : `${d.toLocaleDateString([], { month: 'short', day: 'numeric' })}, ${time}`;
}

function load() {
    try {
        return JSON.parse(localStorage.getItem(KEY) || '{}') || {};
    } catch {
        return {};
    }
}

/** The tab over the shell `app` (its routes, the Controls tab's jobs, RMP1-K's run view). */
export function createMechTab(app) {
    const ids = Object.keys(app.baked.robots).filter((id) => id === 'one' || id === 'two');
    const ui = { robot: ids[0] || 'one' };
    let results = load();      // {robot: {ident: {tone, words, at}}}
    let pending = null;        // the test sent last: {robot, name, route, watch, after, run} — only a run after `after` is its own
    const el = h('div', { class: 'mechs', id: 'mechanisms', hidden: true });

    const robot = () => app.baked.robots[ui.robot];
    const save = () => {
        try {
            localStorage.setItem(KEY, JSON.stringify(results));
        } catch {
            // a browser that keeps no site data: the results stay for this visit
        }
    };

    /** The robot's lines onto the cards of the test sent last (RMP1-K's watch; each line repaints): only a run that
     *  started after its send (an earlier run of the same name is not this test's), on the robot it was sent to. */
    function readRun() {
        const st = app.runView && app.runView.state();
        const w = st && st.watch;
        if (!w) {
            return null;
        }
        const since = w === pending.watch ? w.runs.slice(pending.after) : w.runs;  // a new watch opened after the send
        const run = since.findLast((r) => r.name === pending.name && (r.robot || pending.robot) === pending.robot);
        if (!run) {
            return null;
        }
        const again = pending.run === run;  // the same run, repainted: its time stays unless its tone changed
        pending.run = run;
        const got = testResults(run, pending.route, app.baked.robots[pending.robot]);
        const mine = (results[pending.robot] = results[pending.robot] || {});
        for (const [ident, r] of Object.entries(got)) {
            mine[ident] = { ...r, at: again && mine[ident] && mine[ident].tone === r.tone ? mine[ident].at : Date.now() };
        }
        save();
        return run;
    }

    function test(idents) {
        const which = ui.robot;
        app.runView.test(idents, which, {
            onSend: (route) => {  // written: from here on, only a new run of this name is this test's
                const w = app.runView.state().watch;
                pending = { robot: which, name: route.name, route, watch: w, after: w ? w.runs.length : 0, run: null };
                render();
            },
            onEnd: (end) => {  // not uploaded: nothing reached the robot, so the cards keep their last results
                if (pending && (!end || end.kind !== 'uploaded')) {
                    pending = null;
                    render();
                }
            },
        });
    }

    function card(m) {
        const jobs = app.jobsOf(ui.robot, m.ident);
        const why = testRefusal(m);
        const testing = pending && !pending.run && pending.robot === ui.robot && pending.route.steps.some((s) => s.mechanism === m.ident);
        const r = testing ? null : (results[ui.robot] || {})[m.ident];
        const uses = routesUsing(app.routes(), m.ident);
        const levels = levelWords(m);
        const row = (label, ...v) => h('div', { class: 'mc-row' }, h('dt', {}, label), h('dd', {}, ...v));
        return h('article', { class: `card mech-card${r ? ` ${r.tone || 'run'}` : ''}`, 'data-mech': m.ident, 'aria-label': m.name },
            h('div', { class: 'mc-head' }, h('span', { class: 'mc-icon', 'aria-hidden': 'true' }, icon(KIND_ICON[m.kind] || 'mechanism')),
              h('h2', {}, m.name),
              h('button', { class: 'btn', 'data-test': m.ident, disabled: !app.runView, title: why || `Test the ${m.name} on the robot: it moves, the robot stays where it is`,
                            onclick: () => test([m.ident]) }, icon('play'), 'Test')),
            h('dl', {},
              row('What it is', whatItIs(m, jobs.some((j) => /hold/.test(j.words)))),
              row('Ports', ...portWords(m).map((w) => h('span', { class: /not known|not in this page/.test(w) ? 'owed' : '' }, w))),
              row('Buttons', ...(jobs.length ? jobs.map((j) => h('span', {}, j.words)) : [h('span', { class: 'muted' }, 'no button')])),
              levels ? row('Levels', ...levels.map((w) => h('span', { class: /not measured|no levels/.test(w) ? 'muted' : '' }, w))) : null,
              row('Routes', ...(uses.length ? uses.map((u) => h('button', { class: 'link', 'data-route': u.name, onclick: () => app.openStep(u.k, u.step) }, u.name))
                  : [h('span', { class: 'muted' }, 'none yet')]))),
            r ? h('p', { class: `mc-result ${r.tone}`, role: 'status' }, icon(TONE_ICON[r.tone] || 'play'), h('span', {}, r.words),
                  r.tone ? h('small', {}, whenWords(r.at)) : null)
                : testing ? h('p', { class: 'mc-result', role: 'status' }, icon('play'), h('span', {}, 'Waiting for the robot\'s run…')) : null);
    }

    function render() {
        if (el.hidden) {
            return;
        }
        const run = pending ? readRun() : null;
        const mechs = robot().mechanisms || [];
        const watching = app.runView && ['opening', 'live', 'paused'].includes(app.runView.state().mode);
        const old = mechs.some((m) => !m.wiring);
        const notice = old ? OLD_FACTS : testerRefusal(robot());
        mount(el,
            h('div', { class: 'ctl-top' },
              ids.length > 1 ? h('div', { class: 'robot-pick' }, h('span', { class: 'muted' }, 'Robot'),
                  h('div', { class: 'seg', role: 'radiogroup', 'aria-label': 'Robot' }, ids.map((id) => h('button', {
                      role: 'radio', 'aria-checked': String(ui.robot === id), class: ui.robot === id ? 'on' : '',
                      onclick: () => { ui.robot = id; render(); } }, robotWords(id))))) : null,
              notice ? h('span', { class: 'mc-notice', id: 'mech-notice', role: 'note' }, icon('warn'), notice) : null,
              h('span', { class: 'spacer' }),
              pending && watching && !run ? h('span', { class: 'mc-watch', role: 'status' }, icon('play'),
                  `On the brain: Auton Tester → "${pending.name}" → RUN`) : null,
              h('button', { class: notice ? 'btn' : 'btn primary', id: 'test-all', disabled: !app.runView || !mechs.length, onclick: () => test(null),  // a notice: it is the one highlight
                            title: app.runView ? 'Test every mechanism in one run' : 'Testing needs the route builder on the team laptop (tools/route_builder.py)' },
                icon('play'), 'Test every mechanism')),
            h('div', { class: 'ctl-body' },
              mechs.length ? h('div', { class: 'mech-grid' }, mechs.map(card))
                  : h('p', { class: 'muted' }, `${robotWords(ui.robot)} has no mechanisms in its robot file.`)));
    }

    return {
        element: el,
        show(on) {
            el.hidden = !on;
            render();
        },
        render,
        state: () => ({ robot: ui.robot, pending: pending && { robot: pending.robot, name: pending.name }, results }),
    };
}
