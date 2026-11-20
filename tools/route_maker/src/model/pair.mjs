// pair.mjs — ROBOT TWO ON THE FIELD, and the too-close check (RMP1-C D4, D5).
//
// A route mirrored for robot two runs on robot two reflected across the field's skills diagonal; every number here
// comes from the C++ (the robots' descriptions: `skillsMirror`, `pairCheck`, baked by build.py) — never retyped.
// The check is test/mirrored_pair_check.hpp's rule on the page's own plan: when BOTH robots run the route at the
// same time, their squares stay at least `gapIn` apart at every moment of the plan, the start included. The page's
// routes are pure mirrors (no isMirrored branch), so robot two at a moment is robot one's pose reflected.
// Pure: no DOM.

import { squareCorners, wrapDeg, DEG } from './geometry.mjs';

/** `p` ({x, y, headingDeg}) reflected by `mirror` ({cos2a, sin2a}: the line through the origin at heading a). */
export function reflect(p, mirror) {
    const { cos2a: c, sin2a: s } = mirror;
    return { x: c * p.x + s * p.y + 0, y: s * p.x - c * p.y + 0,
             headingDeg: wrapDeg(Math.atan2(s, c) / DEG - p.headingDeg) };
}

function segDist(p, a, b) {
    const vx = b.x - a.x;
    const vy = b.y - a.y;
    const wx = p.x - a.x;
    const wy = p.y - a.y;
    let t = (vx * wx + vy * wy) / (vx * vx + vy * vy);
    t = t < 0 ? 0 : t > 1 ? 1 : t;
    return Math.hypot(wx - t * vx, wy - t * vy);
}
function overlap(a, b) {
    for (const q of [a, b]) {
        for (let i = 0; i < 2; ++i) {
            const nx = -(q[i + 1].y - q[i].y);
            const ny = q[i + 1].x - q[i].x;
            const pa = a.map((v) => v.x * nx + v.y * ny);
            const pb = b.map((v) => v.x * nx + v.y * ny);
            if (Math.max(...pa) < Math.min(...pb) || Math.max(...pb) < Math.min(...pa)) {
                return false;
            }
        }
    }
    return true;
}

/** The gap (in) between two robots' squares of side `size`: 0 when they overlap or touch (mirrored_pair_check.hpp). */
export function squareGap(p, q, size) {
    const a = squareCorners(p.x, p.y, p.headingDeg, size);
    const b = squareCorners(q.x, q.y, q.headingDeg, size);
    if (overlap(a, b)) {
        return 0;
    }
    let d = Infinity;
    for (let i = 0; i < 4; ++i) {
        for (let j = 0; j < 4; ++j) {
            d = Math.min(d, segDist(a[i], b[j], b[(j + 1) % 4]), segDist(b[i], a[j], a[(j + 1) % 4]));
        }
    }
    return d;
}

const lerp = (a, b, t) => a + (b - a) * t;
const turn = (from, to, at) => {
    const sweep = wrapDeg(to - from);
    const n = Math.max(1, Math.ceil(Math.abs(sweep) / 3));
    return Array.from({ length: n + 1 }, (_, i) => ({ ...at, headingDeg: wrapDeg(from + (sweep * i) / n) }));
};
const line = (a, b, fromDeg, toDeg) => {
    const n = Math.max(1, Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.5));
    const sweep = wrapDeg(toDeg - fromDeg);
    return Array.from({ length: n + 1 }, (_, i) => ({ x: lerp(a.x, b.x, i / n), y: lerp(a.y, b.y, i / n),
                                                      headingDeg: wrapDeg(fromDeg + (sweep * i) / n) }));
};

/** Robot one's planned poses through step `i` (its plan entry `p`), every 0.5 in and every 3 deg. */
export function stepPoses(step, p) {
    if (!p || !p.from || !p.to) {
        return [];
    }
    if (p.curve) {
        return p.curve.samples(0.5);
    }
    const drive = ['driveTo', 'reverseTo', 'forward', 'backward'].includes(step.do);
    if (drive) {
        const leg = Number.isFinite(p.bearingDeg) ? p.bearingDeg : p.from.headingDeg;
        if (step.faceFirst) {
            return [...turn(p.from.headingDeg, leg, p.from), ...line(p.from, p.to, leg, leg)];
        }
        return line(p.from, p.to, p.from.headingDeg, leg);
    }
    if (p.from.headingDeg !== p.to.headingDeg) {
        return turn(p.from.headingDeg, p.to.headingDeg, p.from);
    }
    return [p.to];
}

/**
 * The first moment of the plan the two robots come closer than `gapIn` — {stepIndex (-1: the start), one, two, gap} —
 * or null when they never do (or there is no start). `plan` is plannedPoses(route).
 */
export function tooClose(route, plan, mirror, { gapIn, squareIn }) {
    const check = (pose, stepIndex) => {
        const two = reflect(pose, mirror);
        const gap = squareGap(pose, two, squareIn);
        return gap < gapIn ? { stepIndex, one: pose, two, gap } : null;
    };
    const s0 = plan.length ? plan[0].from : route.start;
    if (!s0 || ![s0.x, s0.y, s0.headingDeg].every(Number.isFinite)) {
        return null;
    }
    const first = check(s0, -1);
    if (first) {
        return first;
    }
    for (let i = 0; i < route.steps.length; ++i) {
        for (const pose of stepPoses(route.steps[i], plan[i])) {
            const hit = check(pose, i);
            if (hit) {
                return hit;
            }
        }
    }
    return null;
}

/**
 * Where robot one's start goes to clear the gap — the least move that does without pushing the robot into a wall
 * (`walls`: {inner, slack}, the RC3 rule: no corner further past a wall's face than the start's already is): straight
 * away from the mirror line, or, against a wall, along it. On the stored grid: {start, by, along} (along: it slides
 * along a wall), or null (clear already, or no such move). A pure mirror's gap is twice the square's clearance.
 */
export function startAwayFromLine(start, mirror, { gapIn, squareIn }, round, walls) {
    const a = Math.atan2(mirror.sin2a, mirror.cos2a) / 2;
    const side = Math.sign(-start.x * Math.sin(a) + start.y * Math.cos(a)) || 1;
    const m = { x: -side * Math.sin(a), y: side * Math.cos(a) };  // the line's normal, toward the start's side
    const gapOf = (s) => squareGap(s, reflect(s, mirror), squareIn);
    if (gapOf(start) >= gapIn) {
        return null;
    }
    const cs = (s) => squareCorners(s.x, s.y, s.headingDeg, squareIn);
    const past = (s) => Math.max(...cs(s).map((c) => Math.max(Math.abs(c.x), Math.abs(c.y)) - walls.inner));
    const lim = walls.inner + Math.min(walls.slack, Math.max(0, past(start)));
    const xs = cs(start).map((c) => c.x);
    const ys = cs(start).map((c) => c.y);
    const e = 0.0006;  // a hair inside, so the stored grid's rounding never crosses it
    const box = [[e - lim - Math.min(...xs), lim - e - Math.max(...xs)], [e - lim - Math.min(...ys), lim - e - Math.max(...ys)]];
    const need = gapIn / 2 - Math.min(...cs(start).map((c) => c.x * m.x + c.y * m.y));
    // the moves v with v·m = by: by·m + t·(-m.y, m.x), t clipped to the box; |v| is least at the t nearest 0
    const along = (by) => {
        let lo = Number.NEGATIVE_INFINITY;
        let hi = Number.POSITIVE_INFINITY;
        for (const [base, dir, [b0, b1]] of [[by * m.x, -m.y, box[0]], [by * m.y, m.x, box[1]]]) {
            if (Math.abs(dir) < 1e-12) {
                lo = base < b0 || base > b1 ? Number.POSITIVE_INFINITY : lo;
            } else {
                lo = Math.max(lo, Math.min((b0 - base) / dir, (b1 - base) / dir));
                hi = Math.min(hi, Math.max((b0 - base) / dir, (b1 - base) / dir));
            }
        }
        return lo > hi ? null : Math.min(Math.max(0, lo), hi);
    };
    for (let k = 0; k < 100; ++k) {
        const by = need + k * 0.001;
        const t = along(by);
        if (t === null) {
            return null;
        }
        const s = { ...start, x: round(start.x + by * m.x - t * m.y), y: round(start.y + by * m.y + t * m.x) };
        if (gapOf(s) >= gapIn && past(s) <= lim - walls.inner) {
            return { start: s, by: Math.hypot(s.x - start.x, s.y - start.y), along: Math.abs(t) > 1e-9 };
        }
    }
    return null;
}
