// obstacles.mjs — THE FIELD'S SOLID STRUCTURES (RMP1-H): every step's swept square checked against the goals and the
// Loaders. Their footprints come from the C++ (robots.json's field.obstacles, test/override_field_obstacles.hpp, each
// cited to the game manual) — never typed here. The sweep is the plan's own (pair.mjs stepPoses: every 0.5 in and
// 3 deg), with the hull of each two neighbouring squares, so a pass between two samples is not missed.
//   red    the square goes into a structure on the way (D3), or more than TOUCH_IN into it where the step ENDS (D4:
//          touching it, to place a cone, is allowed) -- or, the step touching it on purpose, more than its `overhangIn`
//          (a red or blue goal narrows above its lip: the robot's front can reach over the base), at most THROUGH_IN
//   amber  it passes closer than CLEAR_IN, unless the step says it touches that one on purpose (`touches`, D4)
// A step is judged on what IT does: one that starts already that close and only backs away is not flagged.
// PREVIEW (runThrough): a run whose square goes more than THROUGH_IN into one is THROUGH, as a wall is.
// Pure: no DOM.

import { squareCorners, bearingDeg, wrapDeg } from './geometry.mjs';
import { stepPoses, reflect } from './pair.mjs';
import { startPose } from './route.mjs';
import { fmt } from './numbers.mjs';
import { robotName } from './robot_names.mjs';

/** D4: real overlap past this (in) is red; less is touching. */
export const TOUCH_IN = 0.5;
/** D3: passing closer than this (in) is amber — the field's elements may sit 1 in off the drawing (<T5>a, HA-258). */
export const CLEAR_IN = 1.0;
/** PREVIEW: RPV1's THROUGH allowance (route_preview_model.hpp kThroughAllowanceIn), the walls' and the Loaders'. */
export const THROUGH_IN = 1.0;
const EPS = 0.01;
const NOISE = 1e-9;  // floating point: a limit met exactly is not past it
const DEG = 180 / Math.PI;

const polyOf = (o) => o.footprint.map(([x, y]) => ({ x, y }));
const reachOf = (o) => Math.max(...o.footprint.map(([x, y]) => Math.hypot(x - o.x, y - o.y)));
const centreOf = (o) => ({ x: o.footprint.reduce((a, p) => a + p[0], 0) / o.footprint.length,
                           y: o.footprint.reduce((a, p) => a + p[1], 0) / o.footprint.length });

function segDist(p, a, b) {
    const vx = b.x - a.x;
    const vy = b.y - a.y;
    const l2 = vx * vx + vy * vy;
    const t = l2 ? Math.min(Math.max(((p.x - a.x) * vx + (p.y - a.y) * vy) / l2, 0), 1) : 0;
    return Math.hypot(p.x - a.x - t * vx, p.y - a.y - t * vy);
}

/** A polygon's extent along (nx, ny): [least, most]. */
function span(poly, nx, ny) {
    let lo = Infinity;
    let hi = -Infinity;
    for (const v of poly) {
        const t = v.x * nx + v.y * ny;
        lo = Math.min(lo, t);
        hi = Math.max(hi, t);
    }
    return [lo, hi];
}

/** The gap (in) between two convex polygons, or, when they overlap, minus the least push that parts them. A gap of
 *  `enough` or more is not measured exactly: a lower bound at least that big is returned. */
export function signedGap(a, b, enough = Infinity) {
    let depth = Infinity;
    for (const poly of [a, b]) {
        for (let i = 0; i < poly.length; ++i) {
            const p = poly[i];
            const q = poly[(i + 1) % poly.length];
            const len = Math.hypot(q.x - p.x, q.y - p.y);
            if (len === 0) {
                continue;
            }
            const nx = (q.y - p.y) / len;
            const ny = (p.x - q.x) / len;
            const [a0, a1] = span(a, nx, ny);
            const [b0, b1] = span(b, nx, ny);
            const overlap = Math.min(a1 - b0, b1 - a0);  // the push out
            if (overlap <= 0) {
                if (-overlap >= enough) {
                    return -overlap;
                }
                let d = Infinity;
                for (const [u, w] of [[a, b], [b, a]]) {
                    for (const v of u) {
                        for (let k = 0; k < w.length; ++k) {
                            d = Math.min(d, segDist(v, w[k], w[(k + 1) % w.length]));
                        }
                    }
                }
                return d;
            }
            depth = Math.min(depth, overlap);
        }
    }
    return -depth;
}

/** The convex hull of points (monotone chain). */
function hull(points) {
    const p = [...points].sort((a, b) => a.x - b.x || a.y - b.y);
    const cross = (o, a, b) => (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
    const half = (list) => {
        const out = [];
        for (const v of list) {
            while (out.length >= 2 && cross(out[out.length - 2], out[out.length - 1], v) <= 0) {
                out.pop();
            }
            out.push(v);
        }
        out.pop();
        return out;
    };
    return [...half(p), ...half(p.reverse())];
}

/**
 * How close each structure comes along `poses` ({x, y, headingDeg}, a step's planned sweep): [{o, d0, dmin, end, at}] —
 * d is the gap (in), below 0 how far in; d0 at the first pose, dmin the least, `end` when the last pose is that close,
 * `at` the pose where it is least.
 */
export function sweep(poses, obstacles, size) {
    const reach = size / Math.SQRT2;
    const box = poses.reduce((b, q) => [Math.min(b[0], q.x), Math.min(b[1], q.y), Math.max(b[2], q.x), Math.max(b[3], q.y)],
                             [Infinity, Infinity, -Infinity, -Infinity]);
    return obstacles.map((o) => {
        const r = reachOf(o);
        const far = Math.hypot(Math.max(box[0] - o.x, o.x - box[2], 0), Math.max(box[1] - o.y, o.y - box[3], 0)) - reach - r;
        if (far > CLEAR_IN + 1) {  // the whole step is far from it: no pose needs a look
            return { o, d0: far, dmin: far, end: true, at: poses[0] || null };
        }
        const poly = polyOf(o);
        let prev = null;
        let d0 = Number.NaN;
        let dmin = Infinity;
        let dEnd = Infinity;
        let at = null;
        poses.forEach((q, k) => {
            const far = Math.hypot(q.x - o.x, q.y - o.y) - reach - r;  // a lower bound on the gap
            let d = far;
            let sq = null;
            if (far <= CLEAR_IN + 1) {
                sq = squareCorners(q.x, q.y, q.headingDeg, size);
                d = signedGap(sq, poly, CLEAR_IN + 1);
                if (prev && d > 0) {  // between two samples: the hull of both squares (a translation's exact sweep)
                    d = Math.min(d, Math.max(signedGap(hull([...prev, ...sq]), poly, CLEAR_IN + 1), 0));
                }
            }
            if (k === 0) {
                d0 = d;
            }
            if (d < dmin) {
                dmin = d;
                at = q;
            }
            dEnd = d;
            prev = sq;
        });
        return { o, d0, dmin, end: dEnd <= dmin + EPS, at };
    });
}

const NONE = new Set();
const ALL = { has: () => true };
/** Over a base on purpose: no deeper than PREVIEW's THROUGH, so a plan cleared here never runs THROUGH where it ends. */
const overOf = (o) => Math.min(o.overhangIn || 0, THROUGH_IN);
/** How far (in) a step may go into `h`'s structure: on the way nothing; where it ends a touch, or on purpose its overhang. */
const allowed = (h, touches) => (!h.end ? EPS : touches.has(h.o.id) ? Math.max(TOUCH_IN, overOf(h.o)) : TOUCH_IN);
const goesIn = (h, touches = NONE) => h.dmin < -allowed(h, touches) - NOISE && h.dmin < h.d0 - EPS;
const comesClose = (h) => h.dmin < CLEAR_IN && h.dmin < h.d0 - EPS;
const worst = (list) => list.sort((a, b) => a.dmin - b.dmin)[0] || null;
const sizeOf = (robot) => (robot && robot.drive && robot.drive.footprintIn) || 15;
const FIELD_OF = { forward: 'inches', backward: 'inches', turnTo: 'headingDeg', curveTo: 'points', reverseCurveTo: 'points' };
const up = (v) => (Math.ceil(v * 10 - 10 * NOISE) / 10).toFixed(1);  // a red's depth: never shown at its limit
const down = (v) => (Math.floor(v * 10 + 10 * NOISE) / 10).toFixed(1);  // an amber's: never shown past it
const purposeOf = (o) => (o.kind === 'Loader' ? 'taking a load' : 'placing a cone');

/** Robot two's "on purpose": each touched structure's mirror image, where one stands (a goal's; a Loader has none). */
function twinsOf(touches, obs, mirror) {
    const twins = new Set();
    for (const o of obs.filter((k) => touches.has(k.id))) {
        const image = o.footprint.map(([x, y]) => reflect({ x, y, headingDeg: 0 }, mirror));
        const twin = obs.find((k) => k.footprint.length === image.length
            && k.footprint.every(([x, y]) => image.some((v) => Math.hypot(v.x - x, v.y - y) < EPS)));
        if (twin) {
            twins.add(twin.id);
        }
    }
    return twins;
}

/** The ids of the structures step `i` comes close to that "on purpose" would clear (none while another is red anyway). */
export function nearStructures(route, plan, field, size, i) {
    const obs = (field && field.obstacles) || [];
    const hits = sweep(stepPoses(route.steps[i], plan[i]), obs, size);
    return hits.some((h) => goesIn(h, ALL)) ? [] : hits.filter(comesClose).map((h) => h.o.id);
}

/** Where to go around `o` on the way from `from` to `to` (a straight leg, turned first): a point on the 0.5 in grid
 *  whose two legs stay inside the walls, clear `o` by CLEAR_IN, and come no closer to any other structure than the
 *  straight leg did (`before`, its sweep) or the step says on purpose (`touches`) — or null. */
function detour(from, to, o, obs, size, wall, back, before, touches) {
    const len = Math.hypot(to.x - from.x, to.y - from.y);
    const n = { x: -(to.y - from.y) / len, y: (to.x - from.x) / len };
    const c = centreOf(o);
    const side = Math.sign((c.x - from.x) * n.x + (c.y - from.y) * n.y) || 1;
    const leg = (a, b) => {
        const bd = bearingDeg(a.x, a.y, b.x, b.y);
        const h = back ? wrapDeg(bd + 180) : bd;
        return stepPoses({ do: 'driveTo', faceFirst: true }, { from: a, to: { x: b.x, y: b.y, headingDeg: h }, bearingDeg: h });
    };
    const was = new Map(before.map((h) => [h.o.id, h]));
    const fine = (h) => (h.o.id === o.id ? h.dmin >= CLEAR_IN
        : !goesIn(h, touches) && (!comesClose(h) || comesClose(was.get(h.o.id)) || touches.has(h.o.id)));
    for (const sgn of [-side, side]) {
        for (const extra of [0.5, 3, 6, 9]) {
            const off = reachOf(o) + size / Math.SQRT2 + CLEAR_IN + extra;
            const w = { x: Math.round((c.x + sgn * n.x * off) * 2) / 2 + 0, y: Math.round((c.y + sgn * n.y * off) * 2) / 2 + 0 };  // never -0
            const one = leg(from, w);
            const poses = [...one, ...leg(one[one.length - 1], to)];
            const inWalls = poses.every((q) => squareCorners(q.x, q.y, q.headingDeg, size).every((k) => Math.abs(k.x) <= wall && Math.abs(k.y) <= wall));
            if (inWalls && sweep(poses, obs, size).every(fine)) {
                return w;
            }
        }
    }
    return null;
}

const DETOURS = new WeakMap();  // a field's structures -> the way around each red already searched (the costly part)
function detourOf(step, p, o, obs, size, wall, before, touches) {
    const memo = DETOURS.get(obs) || new Map();
    DETOURS.set(obs, memo);
    const key = JSON.stringify([p.from, p.to, step.do, !!step.faceFirst, o.id, size, wall, [...touches].sort()]);
    if (!memo.has(key)) {
        if (memo.size > 500) {
            memo.clear();
        }
        memo.set(key, detour(p.from, { x: step.x, y: step.y }, o, obs, size, wall, step.do === 'reverseTo', before, touches));
    }
    return memo.get(key);
}

const CURVE = new Set(['curveTo', 'reverseCurveTo']);
const INCHES = new Set(['forward', 'backward']);

/** A red step's words: where, which, how far in, and the plain fix. */
function inWords(step, p, h, size) {
    const d = up(-h.dmin);
    if (h.end && -h.dmin <= overOf(h.o) + NOISE) {
        return `Where this step ends, the robot's ${size} in square is ${d} in into the ${h.o.name}'s base. If its front reaches over the `
            + 'base on purpose (placing a cone), say so below; if not, stop short of it.';
    }
    if (!CURVE.has(step.do) && h.at.x === p.from.x && h.at.y === p.from.y) {
        return `Turning in place here, a corner of the robot goes ${d} in into the ${h.o.name}: it is solid. Move the step before it away from it first.`;
    }
    if (h.end) {
        return `Where this step ends, the robot's ${size} in square is ${d} in into the ${h.o.name}: it is solid. Stop short of it: `
            + `${CURVE.has(step.do) ? 'move the last curve point' : INCHES.has(step.do) ? 'change the inches' : 'move the point'}.`;
    }
    return `On the way, near (${fmt(Math.round(h.at.x * 10) / 10)}, ${fmt(Math.round(h.at.y * 10) / 10)}), the robot's ${size} in square goes ${d} in `
        + `into the ${h.o.name}: it is solid, the robot cannot drive through it. `
        + `${CURVE.has(step.do) ? 'Move a curve point away from it.' : INCHES.has(step.do) ? 'Change the inches.' : 'Move the point, or go around it.'}`;
}

/**
 * RCO1 (red), RCO2 (amber) and RCO3 (robot two's red: a mirrored route's path and "on purpose" reflected — the Loaders
 * are not mirror images) for `route`; `issue` is checks.mjs's issue maker. One per step, the worst: robot one's red (a
 * red no switch lifts before one it does), robot two's, then robot one's amber.
 */
export function obstacleIssues(route, plan, robots, field, issue) {
    const obs = (field && field.obstacles) || [];
    const s0 = startPose(route.start);
    if (!obs.length || !s0) {
        return [];
    }
    const out = [];
    const one = robots[0];
    const size = sizeOf(one);
    const mirror = route.mirroredFor && one.skillsMirror ? one.skillsMirror : null;
    const size2 = sizeOf(robots[1] || one);
    const extra = (h, level) => ({ obstacle: { id: h.o.id, at: h.at, level } });
    const inside = (pose, sz) => worst(sweep([pose], obs, sz).filter((h) => h.dmin < -TOUCH_IN));
    const h0 = inside(s0, size);
    const h1 = !h0 && mirror ? inside(reflect(s0, mirror), size2) : null;
    if (h0) {
        out.push(issue('RCO1', 'error', -1, 'start', `At the start the robot's ${size} in square is ${up(-h0.dmin)} in inside the `
            + `${h0.o.name}: it is solid. Move the start off it.`, '', extra(h0, 'error')));
    } else if (h1) {
        out.push(issue('RCO3', 'error', -1, 'start', `${robotName('two')} (mirrored) starts ${up(-h1.dmin)} in inside the ${h1.o.name}: it is `
            + `solid. Move ${robotName('one')}'s start so its mirror image is clear of it.`, '', extra(h1, 'error')));
    }
    route.steps.forEach((step, i) => {
        const p = plan[i];
        const poses = stepPoses(step, p);
        if (poses.length < 2) {
            return;
        }
        const f = FIELD_OF[step.do] || 'x';
        const touches = new Set(step.touches || []);
        const hits = sweep(poses, obs, size);
        const red = worst(hits.filter((h) => goesIn(h, ALL))) || worst(hits.filter((h) => goesIn(h, touches)));  // one no switch lifts first
        if (red) {
            let fix = null;
            if (!red.end && (step.do === 'driveTo' || step.do === 'reverseTo') && Number.isFinite(step.x)) {  // around it, on the way
                const w = detourOf(step, p, red.o, obs, size, field.wallInnerFaceIn, hits, touches);
                fix = w && { kind: 'detour', stepIndex: i, x: w.x, y: w.y, words: `Go around: add a point at (${fmt(w.x)}, ${fmt(w.y)})` };
            }
            out.push(issue('RCO1', 'error', i, f, inWords(step, p, red, size), '', { fix, ...extra(red, 'error') }));
            return;
        }
        const twins = mirror && twinsOf(touches, obs, mirror);
        const two = mirror ? worst(sweep(poses.map((q) => reflect(q, mirror)), obs, size2).filter((h) => goesIn(h, twins))) : null;
        if (two) {
            out.push(issue('RCO3', 'error', i, f, `${robotName('two')} runs this mirrored, and its ${size2} in square goes ${up(-two.dmin)} in `
                + `into the ${two.o.name} during this step${two.o.kind === 'Loader' ? ' (the Loaders are not mirror images)' : ''}: it is `
                + `solid. Move the point so ${robotName('two')}'s path clears it too.`, '', extra(two, 'error')));
            return;
        }
        const near = worst(hits.filter((h) => comesClose(h) && !touches.has(h.o.id)));
        if (near) {
            const words = near.dmin > 0
                ? `On the way the robot passes ${down(near.dmin)} in from the ${near.o.name}: closer than ${fmt(CLEAR_IN)} in, so a little `
                    + `drift and it clips it. Give it more room, or, if it is meant to touch it (${purposeOf(near.o)}), say so below.`
                : `The robot touches the ${near.o.name} here (${down(-near.dmin)} in into it). If it is meant to (${purposeOf(near.o)}), `
                    + 'say so below; if not, give it more room.';
            out.push(issue('RCO2', 'warn', i, 'touches', words, '', extra(near, 'warn')));  // no box marked: the field shows it
        }
    });
    return out;
}

/** PREVIEW: the first moment a run's square ({t, x, y, theta} samples) goes more than THROUGH_IN into a structure. */
export function runThrough(samples, obstacles, size) {
    const reach = size / Math.SQRT2;
    const near = obstacles.map((o) => ({ o, poly: polyOf(o), r: reachOf(o) }));
    for (let s = 0; s < samples.length; ++s) {
        const [t, x, y, th] = samples[s];
        for (const { o, poly, r } of near) {
            if (Math.hypot(x - o.x, y - o.y) > reach + r) {
                continue;
            }
            const d = signedGap(squareCorners(x, y, th * DEG, size), poly);
            if (d < -THROUGH_IN) {
                return { kind: 'THROUGH', element: o.name, t, s, depth: -d, structure: true };
            }
        }
    }
    return null;
}
