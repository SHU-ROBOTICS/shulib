// check_route.mjs — CHECK THE MECHANISMS (RMP1-K D1): from a route, a bench CHECK route of its mechanism steps alone.
//
// Every mechanism step (and every wait) in order, with its own numbers; every drive, turn and curve left out, so the
// robot stays where it is. A start form runs to its end here (startMove → move, startRun → run, startHome → home): with
// no drive beside it, the next command to that mechanism or the routine's end would cut it; a waitUntilDone, which only
// collected a start, is left out. A stall- or switch-homed arm or lift whose first step in the check is a move gets a home
// first (it would stop NOT_HOMED). Each check step carries on past a failure, so one run shows every mechanism. Named
// "<route> mechs"; exported by the ordinary export (no new step kind) beside the team's routes, never saved in the
// project. Pure: no DOM.

import { MECHANISM_KINDS, ROUTE_NAME, newRoute } from './route.mjs';
import { nameProblem } from './checks.mjs';
import { plannedHeights, moveTarget, unitOf, homeEnd } from './lift.mjs';
import { fmt, round3 } from './numbers.mjs';
import { pageMarks, routeVerbs, runDiffers } from './run_log.mjs';

/** A home the check adds takes at most this (s): a home that finds its stop sooner ends sooner. */
export const CHECK_HOME_S = 3;
const KEEP = new Set([...MECHANISM_KINDS, 'wait']);
const BLOCKING = { startMove: 'move', startRun: 'run', startHome: 'home' };
const mechOf = (robot, ident) => ((robot && robot.mechanisms) || []).find((m) => m.ident === ident) || null;

/** The check route's name: "<route> mechs", the route's name cut to fit the brain's menu. */
export function checkName(route, robot = {}) {
    const max = route.mirroredFor ? robot.mirroredAutonNameMaxChars || 25 : robot.autonNameMaxChars || 29;
    return `${route.name.slice(0, max - 6).trim()} mechs`;
}

/**
 * The check route for `route` on `robot` (robot one's description): {route, from} — `from[k]` the source step index of
 * check step k (-1 for an added home) — or {route: null, why} in words.
 */
export function checkRoute(route, robot, routes = [route]) {
    const kept = route.steps.map((s, i) => [s, i]).filter(([s]) => KEEP.has(s.do) && s.do !== 'waitUntilDone');
    if (!kept.some(([s]) => MECHANISM_KINDS.has(s.do))) {
        return { route: null, why: `"${route.name}" has no mechanism steps: there is nothing to check.` };
    }
    const steps = [];
    const from = [];
    const seen = new Set();
    for (const [s, i] of kept) {
        const m = s.mechanism !== undefined ? mechOf(robot, s.mechanism) : null;
        if (m && !seen.has(m.ident) && (s.do === 'move' || s.do === 'startMove') && (m.homing === 'stall' || m.homing === 'switch')) {
            steps.push({ do: 'home', mechanism: m.ident, boundS: CHECK_HOME_S, care: { onFail: 'carryOn' } });
            from.push(-1);
        }
        if (m) {
            seen.add(m.ident);
        }
        const { label, care, view, touches, ...rest } = s;  // eslint-disable-line no-unused-vars
        steps.push({ ...rest, do: BLOCKING[s.do] || s.do, care: { onFail: 'carryOn' } });
        from.push(i);
    }
    const name = checkName(route, robot);
    const twin = routes.find((r) => r.name.toLowerCase() === name.toLowerCase());
    const p = nameProblem(name, !!route.mirroredFor, robot);
    if (twin || p) {
        return { route: null, why: twin ? `A route is already named "${name}": rename it, then check again.` : p.words };
    }
    if (testNames(robot).has(name.toLowerCase())) {  // RMP1-M: the brain would have one name for two different runs
        return { route: null, why: `Its check would be named "${name}", like a test on the Mechanisms tab: rename the route, then check again.` };
    }
    return {
        route: { name, kind: route.kind, robot: route.robot, mirroredFor: route.mirroredFor, together: false, start: route.start,
                 endActionAtS: route.endActionAtS, hardStopAtS: route.hardStopAtS, steps },
        from,
    };
}

/** True when `name` is a check route's (the page never saves one; the robot's log may name one). */
export const isCheckName = (name) => / mechs$/.test(name) && name !== ROUTE_NAME;

/**
 * What the check will move, in words: one clause per mechanism in the order it first moves —
 * "the cascade finds its bottom, goes to 6 in, then 18 in; the claw goes to closed, then open; the right toggle runs 0.8 s".
 */
export function checkWords(check, robot) {
    const heights = plannedHeights(check, robot);
    const order = [];
    const said = new Map();
    check.steps.forEach((s, i) => {
        const m = mechOf(robot, s.mechanism);
        const name = m ? m.name : s.mechanism;
        if (s.do === 'wait' || s.do === 'waitUntilDone' || !name) {
            return;
        }
        if (!said.has(name)) {
            said.set(name, []);
            order.push(name);
        }
        const list = said.get(name);
        const prev = list.length ? list[list.length - 1].kind : null;
        let kind = s.do.replace(/^start/, '').toLowerCase();
        let words;
        if (kind === 'home') {
            words = findWords(m);
        } else if (kind === 'move') {
            const t = m ? moveTarget(s, m, heights[i].before[s.mechanism] ?? null) : null;
            const at = typeof s.preset === 'string' && s.preset ? s.preset : t === null ? 'its level' : `${fmt(t)} ${unitOf(m)}`;
            words = prev === 'move' ? `then ${at}` : `goes to ${at}`;
        } else if (kind === 'set') {
            words = prev === 'set' ? `then ${s.position}` : `goes to ${s.position}`;
        } else {
            kind = 'run';
            words = `runs ${fmt(s.forS)} s at ${fmt(s.volts)} V`;
        }
        list.push({ kind, words });
    });
    return order.map((n) => `the ${n} ${said.get(n).map((x) => x.words).join(', ')}`).join('; ');
}

// ── RMP1-M D3: TEST ONE MECHANISM (the Mechanisms tab) ──────────────────────────────────────────────────────────────

/** "finds its bottom" / "finds its top" (where its home is: RMP1-M D5), "finds its zero" between. */
export const findWords = (m) => `finds its ${homeEnd(m)}`;

/** A test's numbers: each move at most TEST_MOVE_S; a motor runs TEST_RUN_V one way, then the other, TEST_RUN_S each. */
export const TEST_MOVE_S = 3;
export const TEST_RUN_V = 6;
export const TEST_RUN_S = 0.5;
const carry = { onFail: 'carryOn' };

/** A test's name on the brain: "<mechanism> mechs" (cut to fit the menu), "all mechs" for every one. */
export const testName = (m) => (m ? `${m.name.slice(0, 19)} mechs` : 'all mechs');
const testNames = (robot) => new Set([testName(null), ...((robot && robot.mechanisms) || []).map((m) => testName(m).toLowerCase())]);

/** The page's robot facts were made before the Mechanisms tab (no "wiring"): not an owed fact, an old robots.json. */
export const OLD_FACTS = 'This page\'s robot facts were made before the Mechanisms tab, so they have no ports: nothing is sent. '
    + 'Tell the coordinator: the route builder\'s site needs rebuilding.';

/** Why robot `robot`'s Auton Tester (what a test runs on) does not build yet ('' when it does): a fact it needs is owed. */
export function testerRefusal(robot) {
    const owed = (robot && robot.testerOwed) || [];
    return owed.length ? `This robot's Auton Tester does not build until ${owed.length === 1 ? 'one fact is' : `${owed.length} facts are`} typed `
        + `(station 13's quick pass), the first: ${owed[0]}. Nothing is sent.` : '';
}

/** Why `m` cannot be tested by a route ('' when it can): a port not known yet, a kind or homing a route cannot move. */
export function testRefusal(m) {
    const w = m.wiring;
    if (!w) {
        return OLD_FACTS;
    }
    const sensors = [w.homeSwitch, w.angleSensor, w.pieceSensor];
    if (w.motors.some((x) => x.port === null) || w.lines.some((x) => x === null) || sensors.some((x) => x === null)) {
        return `The ${m.name}'s port is not known yet (the builders' answer): nothing is sent.`;
    }
    if (m.kind === 'flywheel') {
        return `A route cannot spin the ${m.name} yet: test it with its button in Drive.`;
    }
    if ((m.kind === 'lift' || m.kind === 'arm') && m.homing === 'never') {
        return `The ${m.name} never finds its zero, so a route cannot move it: test it with its buttons in Drive.`;
    }
    return '';
}

/** One mechanism's test steps (testRefusal(m) first): a lift or arm finds its bottom (stall or switch), each measured
 *  level (none: the middle of its range), then the bottom; a motor runs one way, then the other; a piston each position,
 *  then its start. */
export function testSteps(m) {
    const id = m.ident;
    if (m.kind === 'power') {
        return [{ do: 'run', mechanism: id, volts: TEST_RUN_V, forS: TEST_RUN_S, care: carry },
                { do: 'run', mechanism: id, volts: -TEST_RUN_V, forS: TEST_RUN_S, care: carry }];
    }
    if (m.kind === 'pneumatic') {
        const start = m.start && m.positions.includes(m.start) ? m.start : m.positions[0];
        return [...m.positions.filter((p) => p !== start), start].map((position) => ({ do: 'set', mechanism: id, position, care: carry }));
    }
    const out = m.homing === 'stall' || m.homing === 'switch' ? [{ do: 'home', mechanism: id, boundS: CHECK_HOME_S, care: carry }] : [];
    const levels = (m.presets || []).filter((p) => !p.owed && Number.isFinite(p.value));
    const goals = levels.length ? levels.map((p) => ({ preset: p.ledger !== undefined ? p.ledger : p.name }))
        : [{ to: round3((m.range[0] + m.range[1]) / 2) }];
    for (const g of goals) {
        out.push({ do: 'move', mechanism: id, ...g, boundS: TEST_MOVE_S, care: carry });
    }
    out.push({ do: 'move', mechanism: id, bottom: true, boundS: TEST_MOVE_S, care: carry });
    return out;
}

/**
 * The TEST route of `idents` on `robot` (all of its mechanisms when `idents` is null): {route, left: [{name, why}]} — the
 * robot stays where it is, every step bounded and carrying on — or {route: null, why, left}. Named "<mechanism> mechs"
 * ("all mechs" for every one), a check's name: Send writes it beside the team's routes, then takes it out again.
 */
export function mechanismTest(idents, robot, routes = []) {
    const all = (robot && robot.mechanisms) || [];
    const mechs = idents ? all.filter((m) => idents.includes(m.ident)) : all;
    const left = mechs.map((m) => ({ name: m.name, why: testRefusal(m) })).filter((x) => x.why);
    const steps = mechs.filter((m) => !testRefusal(m)).flatMap(testSteps);
    if (!steps.length) {
        return { route: null, why: left.length ? left[0].why : 'This robot has no mechanisms to test.', left };
    }
    if (testerRefusal(robot)) {
        return { route: null, why: testerRefusal(robot), left };
    }
    const name = testName(idents && mechs.length === 1 ? mechs[0] : null);
    const same = (n) => n.toLowerCase() === name.toLowerCase();
    if (routes.some((r) => same(r.name) || same(checkName(r, robot)))) {
        const r = routes.find((x) => same(x.name) || same(checkName(x, robot)));
        return { route: null, why: same(r.name) ? `A route is already named "${name}": rename it, then test again.`
            : `The check of route "${r.name}" is also named "${name}" on the brain: rename that route, then test again.`, left };
    }
    const p = nameProblem(name, true, robot);
    if (p) {
        return { route: null, why: p.words, left };
    }
    const start = (routes.find((r) => r.start) || {}).start || { x: -36, y: -55.7, headingDeg: 0 };
    return { route: { ...newRoute(name), start, together: false, steps }, left };
}

/**
 * How each mechanism of test `route` did in `run`, from the robot's own lines: {ident: {tone, words}} — 'bad': a step of
 * it stopped or failed (the robot's words); 'warn': a step done but not as asked, or (the run over) skipped or never said;
 * 'ok': every step done (UNCONFIRMED only where nothing on the robot can confirm it); '': not over yet.
 */
export function testResults(run, route, robot) {
    const differs = runDiffers(run, routeVerbs(route, robot));
    const marks = pageMarks(run, route.steps.map(() => 1));
    const over = !!(run.summary || run.verdict);
    const out = {};
    for (const id of new Set(route.steps.map((s) => s.mechanism).filter((x) => x !== undefined))) {
        const ms = marks.filter((_, k) => route.steps[k].mechanism === id);
        const bad = ms.find((x) => x && (x.state === 'stopped' || x.state === 'failed'));
        const odd = ms.find((x) => x && x.state === 'done' && x.outcome && x.outcome !== 'UNCONFIRMED');
        out[id] = differs ? { tone: 'warn', words: 'The robot ran another version of this test: Test again.' }
            : bad ? { tone: 'bad', words: `${bad.state === 'stopped' ? 'Stopped' : 'Failed'}: ${bad.words}` }
                : ms.every((x) => x && x.state === 'done')
                    ? odd ? { tone: 'warn', words: `Done, but the robot says ${odd.outcome}` }
                        : { tone: 'ok', words: ms.some((x) => x.outcome) ? 'Worked (UNCONFIRMED: nothing on the robot can confirm it)' : 'Worked: every step done' }
                    : over ? { tone: 'warn', words: 'Not every step said it was done: see the robot\'s log (Route tab)' }
                        : { tone: '', words: 'Testing…' };
    }
    return out;
}
