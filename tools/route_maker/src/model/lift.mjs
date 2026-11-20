// lift.mjs — WHERE EACH ARM AND LIFT IS, step by step, as the route plans it (RMP1-C D7).
//
// The library has no relative move: "go up 6 in" is written as the constant target `cascade.to(<height>_in)`, so the
// page tracks each arm's and lift's PLANNED height through the steps, in order. Unknown until something sets it:
// a home (its declaration's home height, `homeAt`), a move to a number or a level, "back to the bottom" (the bottom of
// its travel); a mechanism that rests on its stop at power-up starts at its home height. A level the robot's facts
// still OWE has no height (never a placeholder as a fact). Pure: no DOM.

import { round3 } from './numbers.mjs';

const MOVES = new Set(['move', 'startMove']);
/** The units an arm's and a lift's numbers are in. */
export const unitOf = (m) => (m && m.kind === 'arm' ? 'deg' : 'in');
const positional = (m) => m && (m.kind === 'arm' || m.kind === 'lift');

/** The bottom of `m`'s travel on the stored grid, never below it (the build refuses a target past the travel). */
export function bottomOf(m) {
    const lo = m.range[0];
    let v = round3(lo);
    if (v < lo) {
        v = round3(v + 0.001);
    }
    return v;
}

/** Which way a move's target is given: 'preset', 'up', 'down', 'to', 'bottom' — or null (none, or more than one). */
export function targetKind(step) {
    const given = ['preset', 'up', 'down', 'to', 'bottom'].filter((k) => (k === 'bottom' ? step.bottom === true : step[k] !== undefined));
    return given.length === 1 ? given[0] : null;
}

/** A move's target height on `m` from the height `before` it (null: unknown), or null when it cannot be known. */
export function moveTarget(step, m, before) {
    switch (targetKind(step)) {
        case 'preset': {
            const p = (m.presets || []).find((q) => (typeof step.preset === 'number' ? q.ledger === step.preset : q.name === step.preset && q.ledger === undefined));
            return p && !p.owed ? p.value : null;
        }
        case 'to':
            return Number.isFinite(step.to) ? step.to : null;
        case 'bottom':
            return bottomOf(m);
        case 'up':
            return before === null || !Number.isFinite(step.up) ? null : round3(before + step.up);
        case 'down':
            return before === null || !Number.isFinite(step.down) ? null : round3(before - step.down);
        default:
            return null;
    }
}

/**
 * Every arm's and lift's planned height before and after each step of `route` on `robot`: [{before, after}], each a
 * {ident: height | null}. The steps in their order (the plan: a skip or a failure is not followed — every target the
 * export writes is absolute, so the robot goes there whatever came before).
 */
export function plannedHeights(route, robot) {
    const mechs = (robot && robot.mechanisms || []).filter(positional);
    let now = {};
    for (const m of mechs) {
        now[m.ident] = m.homing === 'restsAtStop' && Number.isFinite(m.homeAt) ? m.homeAt : null;
    }
    return route.steps.map((step) => {
        const before = now;
        const m = mechs.find((x) => x.ident === step.mechanism);
        if (m && (step.do === 'home' || step.do === 'startHome')) {
            now = { ...now, [m.ident]: Number.isFinite(m.homeAt) ? m.homeAt : null };
        } else if (m && MOVES.has(step.do)) {
            now = { ...now, [m.ident]: moveTarget(step, m, before[m.ident]) };
        }
        return { before, after: now };
    });
}
