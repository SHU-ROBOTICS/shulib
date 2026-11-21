// gently.mjs — "Leave gently" on a Back up (RMP1-P): wait, then back out slowly. Two fields of its own, `gentlyWaitS`
// and `gentlySpeedInPerS` (null = on, the number not typed), so the step's own care is never touched: the export writes
// the page's Wait, then the back-up capped at the slower of that speed and the step's own top speed. On by default for
// a Back up added, or switched to, right after a drop (D2). Pure: no DOM.

import { updateStep } from './route.mjs';
import { fmt } from './numbers.mjs';

export const GENTLY = Object.freeze({ waitS: 0.5, speedInPerS: 5, maxWaitS: 2, minSpeed: 2, maxSpeed: 20 });
/** The library's time limit for a motion with none typed (motion_config.hpp: MotionConfig::defaultTimeout). */
export const LIBRARY_TIMEOUT_S = 5;
/** A capped leg's time beyond length / speed, with room (robot one on the host test plant: 0.31 s at 5 in/s, 0.76 at 20). */
const settleS = (v) => 0.3 + 0.04 * v;
const BACK_UP = new Set(['reverseTo', 'backward']);

export const isBackUp = (step) => !!step && BACK_UP.has(step.do);
const hasGently = (step) => step.gentlyWaitS !== undefined || step.gentlySpeedInPerS !== undefined;
export const isGentle = (step) => isBackUp(step) && hasGently(step);

/** A drop: a piston set off its `grabs` position, or onto a `releases` one (the robot's description: the claw "open"). */
export function isDrop(step, robot) {
    if (!step || step.do !== 'set' || typeof step.position !== 'string') {
        return false;
    }
    const m = ((robot && robot.mechanisms) || []).find((x) => x.ident === step.mechanism);
    return !!m && ((m.releases || []).includes(step.position) || ((m.grabs || []).length > 0 && !m.grabs.includes(step.position)));
}

/** Step `i` follows a drop directly, or with only waits between. */
export function afterDrop(steps, i, robot) {
    for (let k = i - 1; k >= 0; --k) {
        if (steps[k].do !== 'wait' && steps[k].do !== 'waitUntilDone') {
            return isDrop(steps[k], robot);
        }
    }
    return false;
}

/** The edit that turns it on (the defaults) or off. */
export const gentlyPatch = (on) => (on ? { gentlyWaitS: GENTLY.waitS, gentlySpeedInPerS: GENTLY.speedInPerS }
                                       : { gentlyWaitS: undefined, gentlySpeedInPerS: undefined });

/** The speed it backs out at: its own, or the step's typed top speed when that is slower. */
export function gentleSpeed(step) {
    const own = step.care && step.care.maxSpeedInPerS;
    return own > 0 && own < step.gentlySpeedInPerS ? own : step.gentlySpeedInPerS;
}

/** The step as the export and PREVIEW run it: a gentle one's top speed is gentleSpeed(). */
export const capped = (step) => (isGentle(step) ? { ...step, care: { ...(step.care || {}), maxSpeedInPerS: gentleSpeed(step) } } : step);

/** After an edit to step `i` (`was`: it before; undefined when just added): on for a step that just BECAME a Back up
 *  after a drop, off for one that stopped being a Back up. Loading never calls this. */
export function followGently(route, i, was, robot) {
    const s = route.steps[i];
    if (isBackUp(s) && !isBackUp(was) && !isGentle(s) && afterDrop(route.steps, i, robot)) {
        return updateStep(route, i, gentlyPatch(true));
    }
    return s && !isBackUp(s) && hasGently(s) ? updateStep(route, i, gentlyPatch(false)) : route;
}

/** The checks' [code, field, words] for a step whose leg is `legIn` long (refused outside the bounds). */
export function gentlyProblems(s, legIn) {
    if (!hasGently(s)) {
        return [];
    }
    if (!isGentle(s)) {
        return [['RCG1', 'gentlyWait', 'Leave gently is for a Back up only.']];
    }
    const out = [];
    const w = s.gentlyWaitS;
    if (!Number.isFinite(w) || w <= 0 || w > GENTLY.maxWaitS) {
        out.push(['RCG1', 'gentlyWait', `Leave gently waits more than 0 s and at most ${GENTLY.maxWaitS} s${Number.isFinite(w) ? '' : ': type it'}.`]);
    }
    const v = s.gentlySpeedInPerS;
    if (!Number.isFinite(v) || v < GENTLY.minSpeed || v > GENTLY.maxSpeed) {
        out.push(['RCG1', 'gentlySpeed', `Leave gently backs out at ${GENTLY.minSpeed} to ${GENTLY.maxSpeed} in/s${Number.isFinite(v) ? '' : ': type it'}.`]);
        return out;
    }
    const sp = gentleSpeed(s);
    const limit = s.care && s.care.timeoutS > 0 ? s.care.timeoutS : LIBRARY_TIMEOUT_S;
    const needs = legIn / sp + settleS(sp);
    if (Number.isFinite(needs) && needs > limit) {
        out.push(['RCG2', 'gentlySpeed', `At ${fmt(sp)} in/s this ${fmt(Math.round(legIn * 10) / 10)} in back-up needs about `
            + `${fmt(Math.round(needs * 10) / 10)} s, over its ${fmt(limit)} s time limit: it would be cut short. Back up less, back out `
            + 'faster, or give it a longer time limit.']);
    }
    return out;
}
