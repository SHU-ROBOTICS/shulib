// check_route.mjs — CHECK THE MECHANISMS (RMP1-K D1): from a route, a bench CHECK route of its mechanism steps alone.
//
// Every mechanism step (and every wait) in order, with its own numbers; every drive, turn and curve left out, so the
// robot stays where it is. A start form runs to its end here (startMove → move, startRun → run, startHome → home): with
// no drive beside it, the next command to that mechanism or the routine's end would cut it; a waitUntilDone, which only
// collected a start, is left out. A stall- or switch-homed arm or lift whose first step in the check is a move gets a home
// first (it would stop NOT_HOMED). Each check step carries on past a failure, so one run shows every mechanism. Named
// "<route> mechs"; exported by the ordinary export (no new step kind) beside the team's routes, never saved in the
// project. Pure: no DOM.

import { MECHANISM_KINDS, ROUTE_NAME } from './route.mjs';
import { nameProblem } from './checks.mjs';
import { plannedHeights, moveTarget, unitOf } from './lift.mjs';
import { fmt } from './numbers.mjs';

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
 * "the cascade homes, goes to 6 in, then 18 in; the claw goes to closed, then open; the right toggle runs 0.8 s".
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
            words = 'homes';
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
