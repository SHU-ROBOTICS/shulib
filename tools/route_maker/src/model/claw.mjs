// claw.mjs — WHERE THE CLAW IS (RMP1-O). A route's point is the robot's centre; the claw grabs at the robot's
// clawSpot {forwardIn, sideIn} (robots.json: 15/2 - kClawGrabInsetIn ahead, on the centre line), turned with it.
// Pure: no DOM.

import { fmt, round3 } from './numbers.mjs';

const MIN_LEG_IN = 1;  // the least a drive placed by the claw may move its centre (the claw then lands within 0.006 in)

/** The robot's claw spot, or null (a robot with no claw). */
export function clawSpotOf(robot) {
    const c = robot && robot.clawSpot;
    return c && Number.isFinite(c.forwardIn) && Number.isFinite(c.sideIn) ? c : null;
}

/** Where the claw is when the robot is at `pose` {x, y, headingDeg}. */
export function clawAt(pose, spot) {
    const r = (pose.headingDeg * Math.PI) / 180;
    const [c, s] = [Math.cos(r), Math.sin(r)];
    return { x: pose.x + spot.forwardIn * c - spot.sideIn * s, y: pose.y + spot.forwardIn * s + spot.sideIn * c };
}

/**
 * The centre that puts the claw on `q`, driving straight from `from` ({x, y}) — the robot faces along the leg
 * (`back`: its back leads), or ends at a fixed `headingDeg` (a curve's nose). {x, y}, or {refused: words}.
 */
export function centreForClaw(q, from, spot, { back = false, headingDeg } = {}) {
    if (Number.isFinite(headingDeg)) {
        const c = clawAt({ x: 0, y: 0, headingDeg }, spot);
        return { x: round3(q.x - c.x), y: round3(q.y - c.y) };
    }
    const [f, s] = back ? [-spot.forwardIn, -spot.sideIn] : [spot.forwardIn, spot.sideIn];
    const d = Math.hypot(q.x - from.x, q.y - from.y);
    // the leg's direction u: the claw's sideways offset s is the leg's perpendicular part of q - from; the centre is
    // sqrt(d² - s²) - f along it. Under MIN_LEG_IN the centre's 0.001 in rounding turns the leg, and the claw, off q
    if (!(d > Math.abs(s)) || Math.sqrt(d * d - s * s) - f < MIN_LEG_IN) {
        return { refused: `Too close: the claw is ${fmt(spot.forwardIn)} in ahead of the centre. Point further from where this step starts.` };
    }
    const a = Math.atan2(q.y - from.y, q.x - from.x) - Math.asin(s / d);
    const [ux, uy] = [Math.cos(a), Math.sin(a)];
    return { x: round3(q.x - f * ux + s * uy), y: round3(q.y - f * uy - s * ux) };
}

/** How step `i`'s claw point is placed: the drive's start and direction — or null (not a drive that ends at a point). */
export function clawLeg(step, entry) {
    if (!entry || !entry.from) {
        return null;
    }
    if (step.do === 'driveTo' || step.do === 'reverseTo') {
        return { from: entry.from, back: step.do === 'reverseTo' };
    }
    if ((step.do === 'curveTo' || step.do === 'reverseCurveTo') && entry.curve) {
        // the leg from the knot before the last AS BUILT (a first point within 1 in of the start is dropped: the start's)
        const k = step.points.length - 1;
        const last = step.points[k];
        const before = entry.curve.K(entry.curve.segments() - 1);
        const nose = last.headingDeg !== undefined && last.headingDeg !== null ? last.headingDeg : undefined;
        return { from: { x: before.x, y: before.y }, back: step.do === 'reverseCurveTo', headingDeg: nose, point: k };
    }
    return null;
}

/** A mechanism step on the claw (by its name). */
export function isClawStep(step, robot) {
    const m = ((robot && robot.mechanisms) || []).find((x) => x.ident === step.mechanism);
    return !!m && /claw/i.test(m.name);
}
