// front_bay.test.mjs — THE OPEN FRONT (RMP1-Q): a goal centred on the claw sits INSIDE the robot. Each rule pinned at its
// edge; the robots are robot x with robot one's bay (the tests own these numbers; the C++ pins robot one's own).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { checkRoute } from '../src/model/checks.mjs';
import { newRoute, setStart, addStep, updateStep, plannedPoses } from '../src/model/route.mjs';
import { sweep, runThrough, nearStructures, frontBayOf, robotOutline, notchedGap, bayRadiusAt } from '../src/model/obstacles.mjs';
import { reflect, stepPoses } from '../src/model/pair.mjs';
import { squareCorners } from '../src/model/geometry.mjs';
import { centreForClaw } from '../src/model/claw.mjs';
import { loadProject } from '../src/model/project.mjs';
import { playbackOf } from '../src/model/playback.mjs';
import { fixtureRobot, seeded } from './helpers.mjs';
import { fixture } from './standin_engine.mjs';

const X = fixtureRobot('x');
const FIELD = X.field;
const OBS = FIELD.obstacles;
const ob = (id) => OBS.find((o) => o.id === id);
const SPOT = { forwardIn: 6.268, sideIn: 0 };  // 15 / 2 - 1.232
const ONE = { ...X, id: 'one', clawSpot: SPOT, frontBay: { centreForwardIn: 6.268, lipRadiusIn: 3.22, bodyRadiusIn: 2.8 } };
const BAY = frontBayOf(ONE);
const RED = ob('red-goal-right-wall');
const BLACK = ob('black-goal-right-wall');
const poly = (o) => o.footprint.map(([x, y]) => ({ x, y }));

function route(start, steps, extra = { mirroredFor: undefined, together: undefined }) {
    let r = setStart({ ...newRoute('test'), ...extra }, start);
    for (const s of steps) {
        r = addStep(r, s);
    }
    return r;
}
const rco = (r, robots = [ONE]) => checkRoute(r, robots, FIELD, { routes: [newRoute(), r], index: 1 })
    .filter((i) => /^RCO/.test(i.code)).map((i) => `${i.code} ${i.stepIndex} ${i.words}`);
/** The robot's centre with the goal `back` in behind the claw and `side` in to its left, heading `h`. */
const poseAt = (o, back = 0, side = 0, h = 0) => {
    const r = (h * Math.PI) / 180;
    const f = SPOT.forwardIn - back;
    return { x: o.x - f * Math.cos(r) + side * Math.sin(r), y: o.y - f * Math.sin(r) - side * Math.cos(r), headingDeg: h };
};
/** A straight drive from `from` in out, ending at poseAt(o, back, side, h). */
const drive = (o, back, side, h = 0, from = 12, touches) => {
    const q = poseAt(o, back, side, h);
    const r = (h * Math.PI) / 180;
    return route({ x: q.x - from * Math.cos(r), y: q.y - from * Math.sin(r), headingDeg: h }, [{ do: 'driveTo', x: q.x, y: q.y, touches }]);
};

test('the bay comes from robots.json frontBay; a robot without one, or one that is not open at its front, has none', () => {
    assert.deepEqual(BAY, { f: 6.268, lip: 3.22, body: 2.8 });
    assert.equal(frontBayOf(X), null);
    const bad = (b) => frontBayOf({ ...X, frontBay: { centreForwardIn: 6.268, lipRadiusIn: 3.22, bodyRadiusIn: 2.8, ...b } });
    for (const b of [{ lipRadiusIn: 2.7 }, { bodyRadiusIn: 0 }, { lipRadiusIn: 7.5 }, { centreForwardIn: 7.5 }, { centreForwardIn: -4.3 }, { bodyRadiusIn: 'x' }]) {
        assert.equal(bad(b), null, JSON.stringify(b));
    }
});

test('the outline: the square, open 3.22 in either side of the centre line back to the claw\'s spot and round the back', () => {
    const at = { x: 0, y: 0, headingDeg: 0 };
    assert.deepEqual(robotOutline(at, 15, null), squareCorners(0, 0, 0, 15), 'no bay: the square, exactly');
    const out = robotOutline(at, 15, BAY);
    const inside = (p) => notchedGap(out, [p, { x: p.x + 1e-6, y: p.y }, { x: p.x, y: p.y + 1e-6 }]) < 0;
    assert.ok(!inside({ x: 6.268, y: 0 }), 'the claw\'s spot is open');
    assert.ok(!inside({ x: 7.49, y: 3.21 }) && inside({ x: 7.49, y: 3.23 }), 'the mouth: 3.22 in either side, at the front edge');
    assert.ok(!inside({ x: 3.06, y: 0 }) && inside({ x: 3.03, y: 0 }), 'the back: 3.22 in behind the claw\'s spot (3.048)');
    assert.ok(inside({ x: 0, y: 0 }) && inside({ x: -7.4, y: 7.4 }), 'the rest is solid');
    for (const p of out) {
        assert.ok(Math.abs(p.x) <= 7.5 + 1e-9 && Math.abs(p.y) <= 7.5 + 1e-9);
    }
    assert.deepEqual([out[4], out[out.length - 1]], [{ x: 7.5, y: -3.22 }, { x: 7.5, y: 3.22 }], 'the mouth\'s two corners');
    const back = out.slice(5, -1);
    assert.ok(back.length > 50 && back.every((p) => Math.abs(Math.hypot(p.x - 6.268, p.y) - 3.22) < 1e-9 && p.x <= 6.268 + 1e-9),
              'the back: on the circle, behind the claw\'s spot');
});

test('a red goal centred on the claw is IN the bay: no RCO1, no amber -- at the end, and on the straight way in from any heading', () => {
    assert.deepEqual(rco(drive(RED, 0, 0)), []);
    for (const h of [0, 15, 30, 45, 90, -121.528, 180]) {  // -121.5: White Bot's way into this goal
        assert.deepEqual(rco(drive(RED, 0, 0, h)), [], `heading ${h}`);
        const [hit] = sweep(stepPoses(drive(RED, 0, 0, h).steps[0], plannedPoses(drive(RED, 0, 0, h))[0]), [RED], 15, BAY);
        assert.ok(hit.inBay && hit.dmin > -0.01, `heading ${h}: ${hit.dmin}`);
    }
    for (const o of OBS.filter((k) => k.overhangIn > 0)) {
        assert.deepEqual(rco(drive(o, 0, 0, 90)), [], o.id);
    }
    assert.match(rco(drive(RED, 0, 0), [X])[0], /^RCO1 0 Where this step ends, the robot's 15 in square is 4\.1 in into/, 'robot x, solid: red as today');
});

const PAST = (d, o = RED) => `RCO1 0 Where this step ends, the ${o.name} is ${d} in past the edge of the robot's open front: it is solid there. Stop short of it: move the point.`;

test('0.6 in behind the claw hits the back of the bay: RED; the edge 0.5 in deep sits 0.556 in behind', () => {
    assert.deepEqual(rco(drive(RED, 0.6, 0)), [PAST('0.6')], 'worded for the bay, never "the square"');
    assert.match(rco(drive(RED, 0.55, 0))[0], /^RCO2 0 The robot touches the red goal by the right wall here \(0\.4 in into it\)/);
    assert.deepEqual(rco(drive(RED, 0.56, 0)), [PAST('0.6')]);
    assert.deepEqual(rco(drive(RED, 0.004, 0)), [], '0.004 in: still in (0.01 in past the disc at most)');
    assert.match(rco(drive(RED, 0.02, 0))[0], /^RCO2 0 The robot touches the red goal by the right wall here \(0\.0 in into it\)/, 'past it: a touch');
});

test('a goal SEATED in the bay meets the frame at its lip: "on purpose" clears a touch there, never the overhang', () => {
    assert.deepEqual(rco(drive(RED, 0.6, 0, 0, 12, [RED.id])), [PAST('0.6')], 'nothing to reach over');
    assert.deepEqual(rco(drive(RED, 0.3, 0, 0, 12, [RED.id])), [], 'a touch: said');
    assert.match(rco(drive(RED, 0.3, 0))[0], /^RCO2 0 The robot touches the red goal by the right wall here \(0\.2 in into it\)/);
    // its front over a goal beside the bay (not seated): the overhang, exactly as robot x
    const front = drive(RED, SPOT.forwardIn - 9.8, 4.8, 9);
    assert.deepEqual(rco(front), rco(front, [X]));
    assert.match(rco(front)[0], /^RCO1 0 .* square is 0\.8 in into the red goal by the right wall's base\. If its front reaches over/);
    assert.deepEqual(rco(drive(RED, SPOT.forwardIn - 9.8, 4.8, 9, 12, [RED.id])), [], 'on purpose: its overhang');
});

test('a goal across the robot\'s front corner and the bay\'s mouth: as deep as on the square, red, and PREVIEW\'s THROUGH', () => {
    for (const side of [5.4, -5.4]) {  // its centre 0.1 in past the front edge, 5.4 in off the centre line
        const r = drive(RED, SPOT.forwardIn - 7.6, side);
        assert.deepEqual(rco(r), rco(r, [X]), `${side}`);
        assert.match(rco(r)[0], /^RCO1 0 Where this step ends, the robot's 15 in square is 2\.8 in into the red goal by the right wall: it is solid/);
        assert.deepEqual(rco(drive(RED, SPOT.forwardIn - 7.6, side, 0, 12, [RED.id])), rco(r), 'on purpose: still red');
        const [h] = sweep([poseAt(RED, SPOT.forwardIn - 7.6, side)], [RED], 15, BAY);
        const [sq] = sweep([poseAt(RED, SPOT.forwardIn - 7.6, side)], [RED], 15);
        assert.ok(Math.abs(h.dmin - sq.dmin) < 1e-9 && !h.seated, `${h.dmin} vs ${sq.dmin}`);
        const q = poseAt(RED, SPOT.forwardIn - 7.6, side);
        const run = [0, 4, 8, 12].map((k, i) => [i * 0.1, q.x - 12 + k, q.y, 0]);
        assert.ok(Math.abs(runThrough(run, OBS, 15, BAY).depth - 2.706) < 0.002, 'THROUGH, as robot x');
    }
});

test('deep in the robot (a red\'s number): pushed out through its open front, and behind the disc the fans measure it', () => {
    const [mid] = sweep([poseAt(RED, SPOT.forwardIn, 0, 316)], [RED], 15, BAY);  // at the robot's centre, reaching the bay
    assert.ok(Math.abs(mid.dmin + 6.272) < 0.002 && mid.seated, `the least push out: to the claw's spot, 6.268 in (${mid.dmin})`);
    const [fan] = sweep([poseAt(RED, SPOT.forwardIn - 3.5, 3, 4)], [RED], 15, BAY);
    assert.ok(Math.abs(fan.dmin + 2.993) < 0.002, `${fan.dmin}`);
    // its centre behind the front edge but BESIDE the bay (5.4 in off the centre line): the square's corner, its words
    const side = drive(RED, SPOT.forwardIn - 6, 5.4);
    assert.match(rco(side)[0], /^RCO1 0 Where this step ends, the robot's 15 in square is \d\.\d in into the red goal by the right wall: it is solid/);
    assert.ok(!sweep([poseAt(RED, SPOT.forwardIn - 6, 5.4)], [RED], 15, BAY)[0].seated);
});

test('a goal ahead of the claw\'s spot pushed into the mouth\'s side: its corner in the robot measures it (0.712 in)', () => {
    assert.deepEqual(rco(drive(RED, -0.6, 1.0)), [PAST('0.8')]);
    const [h] = sweep([poseAt(RED, -0.6, 1.0)], [RED], 15, BAY);
    assert.ok(Math.abs(h.dmin + 0.712) < 0.002 && h.seated, `${h.dmin}`);
});

test('0.6 in to the side: RED (0.54 in into the bay\'s side); 0.3 in: amber (0.27)', () => {
    assert.deepEqual(rco(drive(RED, 0, 0.6)), [PAST('0.6')]);
    assert.deepEqual(rco(drive(RED, 0, -0.6)), [PAST('0.6')]);
    assert.match(rco(drive(RED, 0, 0.3))[0], /^RCO2 0 The robot touches the red goal by the right wall here \(0\.2 in into it\)/);
    const [h] = sweep([poseAt(RED, 0, 0.6)], [RED], 15, BAY);
    assert.ok(Math.abs(h.dmin + 0.539) < 0.002, `${h.dmin}`);
});

test('a black goal (full width above its lip) has the BODY\'s 2.80 in: centred, its corners 0.42 in into it -- a touch', () => {
    const [w] = rco(drive(BLACK, 0, 0));
    assert.equal(w, 'RCO2 0 The robot touches the black goal by the right wall here (0.4 in into it). If it is meant to (placing a cone), say so below; if not, give it more room.');
    const [h] = sweep([poseAt(BLACK, 0, 0)], [BLACK], 15, BAY);
    assert.ok(Math.abs(h.dmin + (81.9 / 25.4 - 2.8)) < 0.003 && !h.inBay, `${h.dmin}`);
    assert.deepEqual(rco(drive(BLACK, 0, 0, 0, 12, [BLACK.id])), [], 'said: clear');
    assert.deepEqual(rco(drive(BLACK, 0.2, 0)), [PAST('0.7', BLACK)], 'no overhang to reach over');
    assert.match(rco(drive(ob('centre-goal'), 0, 0))[0], /^RCO2 0 The robot touches the centre goal here \(0\.4 in into it\)/);
});

test('a Loader and a wall at the bay: solid, exactly as robot x', () => {
    const lo = ob('loader-red-wall-left');
    const face = Math.max(...lo.footprint.map((p) => p[1]));
    for (const into of [0.3, 1.2, 2]) {  // the Loader's face this far past the claw's spot, heading -90 (into the red wall)
        const r = route({ x: lo.x, y: face + SPOT.forwardIn - into + 12, headingDeg: -90 }, [{ do: 'driveTo', x: lo.x, y: face + SPOT.forwardIn - into }]);
        assert.deepEqual(rco(r), rco(r, [X]), `${into}`);
        assert.match(rco(r)[0], /^RCO1 0 .* into the left Loader on the red wall: it is solid/);
    }
    const wall = (robots) => checkRoute(route({ x: 0, y: 50, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: 64 }]), robots, FIELD, {})
        .filter((i) => i.level === 'error').map((i) => `${i.code} ${i.words}`);
    assert.ok(wall([ONE]).length > 0);
    assert.deepEqual(wall([ONE]), wall([X]), 'the walls: the square');
});

/** Goal `o` reaches into the bay at robot pose `q` (worked here, not by the page's code): a point of its outline, every
 *  0.05 in, inside the square and in the bay -- ahead of the claw's spot within r of the centre line, or within r of it. */
function inNotch(q, o) {
    if (Math.hypot(q.x - o.x, q.y - o.y) > 7.5 * Math.SQRT2 + 3.3) {
        return false;
    }
    const r = o.overhangIn > 0 ? 3.22 : 2.8;
    const [c, s] = [Math.cos((q.headingDeg * Math.PI) / 180), Math.sin((q.headingDeg * Math.PI) / 180)];
    const pts = poly(o).flatMap((a, i, all) => {
        const b = all[(i + 1) % all.length];
        const n = Math.ceil(Math.hypot(b.x - a.x, b.y - a.y) / 0.05);
        return Array.from({ length: n }, (_, k) => ({ x: a.x + ((b.x - a.x) * k) / n, y: a.y + ((b.y - a.y) * k) / n }));
    });
    return pts.some((p) => {
        const [fx, fy] = [(p.x - q.x) * c + (p.y - q.y) * s, -(p.x - q.x) * s + (p.y - q.y) * c];
        return fx < 7.5 && ((fx >= 6.268 && Math.abs(fy) < r) || Math.hypot(fx - 6.268, fy) < r);
    });
}

test('robot x (no bay) and every route with no goal in the bay: the same issues, word for word (1500 seeded drives)', () => {
    const rnd = seeded(20261120);
    let differed = 0;
    let reached = 0;
    for (let n = 0; n < 1500; ++n) {
        const g = OBS[Math.floor(rnd() * 9)];  // a goal
        const h = Math.round(rnd() * 360 - 180);
        const end = poseAt(g, rnd() * 10 - 5, rnd() * 12 - 6, h);
        const r = drive(g, 0, 0, h, 6 + rnd() * 20);
        const t = updateStep(r, 0, { x: Math.round(end.x * 100) / 100, y: Math.round(end.y * 100) / 100, faceFirst: rnd() < 0.5 });
        const all = (robots) => checkRoute(t, robots, FIELD, { routes: [newRoute(), t], index: 1 }).map((i) => `${i.code} ${i.stepIndex} ${i.words}`);
        const intoBay = stepPoses(t.steps[0], plannedPoses(t)[0]).some((q) => OBS.some((o) => o.kind === 'goal' && inNotch(q, o)));
        reached += intoBay ? 1 : 0;
        if (!intoBay) {
            assert.deepEqual(all([ONE]), all([X]), JSON.stringify(t.steps[0]));
        } else if (JSON.stringify(all([ONE])) !== JSON.stringify(all([X]))) {
            ++differed;
        }
    }
    assert.ok(reached > 300 && differed > 100, `the search reached the bay (${reached}, ${differed} changed)`);
});

test('robot two runs the route mirrored: its goal centred in ITS bay is clear; 0.6 in behind is RCO3; no bay on robot two: RCO3', () => {
    const mirror = X.skillsMirror;
    const twin = ob('red-goal-red-wall');  // the mirror image of the red goal by the right wall
    const r = { ...drive(RED, 0, 0, 30), mirroredFor: 'two', together: false };
    const end = reflect(plannedPoses(r)[0].to, mirror);
    const [h] = sweep([end], [twin], 15, BAY);
    assert.ok(h.inBay, 'robot two\'s goal: centred in its bay');
    assert.deepEqual(rco(r, [ONE, { ...ONE, id: 'two' }]), []);
    assert.match(rco(r, [ONE, { ...X, id: 'two' }])[0], /^RCO3 0 Black Bot runs this mirrored, and its 15 in square goes 4\.\d in into the red goal by the red wall/);
    const deep = { ...drive(RED, 0.6, 0, 30), mirroredFor: 'two', together: false };
    assert.match(rco(deep, [ONE, { ...ONE, id: 'two' }])[0], /^RCO1 0 /, 'robot one\'s red first');
    const front = { ...drive(RED, SPOT.forwardIn - 9.8, 4.8, 30), mirroredFor: 'two', together: false };  // over its base
    assert.match(rco(front, [ONE, { ...ONE, id: 'two' }])[0], /^RCO1 0 .* 1\.0 in into the red goal by the right wall's base/);
    assert.deepEqual(rco({ ...front, steps: [{ ...front.steps[0], touches: [RED.id] }] }, [ONE, { ...ONE, id: 'two' }]), [], 'the one switch: both');
    const small = { ...ONE, id: 'two', frontBay: { centreForwardIn: 6.268, lipRadiusIn: 2.5, bodyRadiusIn: 2.4 } };  // its own, smaller
    assert.deepEqual(rco(r, [ONE, small]), ['RCO3 0 Black Bot runs this mirrored, and the edge of its open front goes 0.8 in into the red goal by the red '
        + 'wall during this step: it is solid. Move the point so Black Bot\'s path clears it too.'], 'worded for its bay');
    // robot two's START: its goal in its own bay (robot one's start mirrored), backing out
    const out = route(poseAt(RED, 0, 0, 30), [{ do: 'backward', inches: 12 }], { mirroredFor: 'two', together: false });
    assert.deepEqual(rco(out, [ONE, { ...ONE, id: 'two' }]), []);
    assert.match(rco(out, [ONE, { ...X, id: 'two' }])[0], /^RCO3 -1 Black Bot \(mirrored\) starts 4\.5 in inside the red goal by the red wall/, 'no bay on robot two');
    assert.deepEqual(rco(out, [ONE, small]), ['RCO3 -1 Black Bot (mirrored) starts with the red goal by the red wall 0.8 in past the edge of its open '
        + 'front: it is solid there. Move White Bot\'s start so its mirror image is clear of it.'], 'its start in its own bay: worded for it');
    assert.deepEqual(rco(route(poseAt(RED, 0.8, 0), [{ do: 'backward', inches: 12 }])), ['RCO1 -1 At the start the red goal by the right wall is 0.8 in '
        + 'past the edge of the robot\'s open front: it is solid there. Move the start off it.']);
});

test('a step is judged on what IT does: backing straight out of the bay is clean; turning in place there is not', () => {
    const at = poseAt(RED, 0, 0, 30);
    const out = route(at, [{ do: 'backward', inches: 12 }]);
    assert.deepEqual(rco(out), []);
    assert.deepEqual(rco(route(at, [{ do: 'turnTo', headingDeg: 35 }])), [PAST('0.6')], '5 deg: the bay swings');
    assert.deepEqual(rco(route(at, [])), [], 'starting with it in the bay: no RCO1');
});

test('the "on purpose" switch: not offered for a goal in the bay; offered for a touch at its back', () => {
    const at = (r) => nearStructures(r, plannedPoses(r), FIELD, 15, 0, BAY);
    assert.deepEqual(at(drive(RED, 0, 0)), []);
    assert.deepEqual(at(drive(RED, 0.1, 0)), [RED.id]);
    assert.deepEqual(nearStructures(drive(RED, 0, 0), plannedPoses(drive(RED, 0, 0)), FIELD, 15, 0), [], 'no bay: red, no switch');
});

test('between two samples the bay is swept pose by pose, never a hull that fills it', () => {
    const tiny = { id: 't', name: 'test goal', kind: 'goal', x: 0, y: 0, overhangIn: 1, footprint: [[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]] };
    const a = { x: -6.268, y: 0, headingDeg: 0 };
    const [h] = sweep([a, { x: -6.768, y: 0, headingDeg: 0 }], [tiny], 15, BAY);
    assert.ok(Math.abs(h.dmin - (3.22 - Math.SQRT2 / 2)) < 0.002, `in the bay, ${h.dmin} in clear of its back`);
    assert.ok(Math.abs(notchedGap(robotOutline(a, 15, BAY), poly(tiny)) - (3.22 - Math.SQRT2 / 2)) < 0.002);
    // both samples clear (the goal in the mouth 1.12 in from its side; then 0.9 in ahead of the jaw), the poses between
    // carry it over the jaw's corner: a touch -- never missed, and never found at 1 in apart
    const [mouth, ahead] = [{ x: -7.2, y: -1.6, headingDeg: 0 }, { x: -8.9, y: -4, headingDeg: 0 }];
    assert.ok(sweep([mouth], [tiny], 15, BAY)[0].dmin > 1.1 && sweep([ahead], [tiny], 15, BAY)[0].dmin > 0.89);
    for (const pair of [[mouth, ahead], [ahead, mouth]]) {
        assert.ok(Math.abs(sweep(pair, [tiny], 15, BAY)[0].dmin) < 1e-9, 'a touch on the way');
    }
    const third = (t) => ({ x: -7.2 - 1.7 * t, y: -1.6 - 2.4 * t, headingDeg: 0 });
    assert.ok(Math.min(...[1 / 3, 2 / 3].map((t) => sweep([third(t)], [tiny], 15, BAY)[0].dmin)) > 0.3, 'a pose every 1 in misses it');
});

test('PREVIEW: a run ending with the goal in its bay is not THROUGH; 1.2 in behind the claw it is; robot x is THROUGH', () => {
    const run = (back) => [0, 4, 8, 12].map((k, i) => {
        const q = poseAt(RED, back, 0, 0);
        return [i * 0.1, q.x - 12 + k, q.y, 0];
    });
    assert.equal(runThrough(run(0), OBS, 15, BAY), null);
    const hit = runThrough(run(1.2), OBS, 15, BAY);
    assert.ok(hit && hit.element === RED.name && hit.s === 3 && hit.depth > 1 && hit.depth < 1.2, JSON.stringify(hit));
    assert.equal(runThrough(run(0), OBS, 15).element, RED.name, 'no bay: THROUGH');
    const f = fixture('ready').result;
    const rec = { ...f.one, robot: 'one', samples: run(0), field: [], pageSteps: [], steps: [] };
    assert.equal(playbackOf({ ...f, one: rec }, {}, { obstacles: OBS, size: 15, bays: { one: BAY } }).one.through, null);
    assert.equal(playbackOf({ ...f, one: rec }, {}, { obstacles: OBS, size: 15, bays: { two: BAY } }).one.through.element, RED.name,
        'robot two\'s bay is not robot one\'s');
    const two = { ...rec, robot: 'two' };  // each run reads ITS robot's bay
    assert.equal(playbackOf({ ...f, one: two }, {}, { obstacles: OBS, size: 15, bays: { one: BAY } }).one.through.element, RED.name);
    assert.equal(playbackOf({ ...f, one: two }, {}, { obstacles: OBS, size: 15, bays: { two: BAY } }).one.through, null);
});

test('the bay is drawn with the radius it is checked with: a black or centre goal in it, the body\'s; else the lip\'s', () => {
    assert.equal(bayRadiusAt(poseAt(RED), 15, BAY, OBS), 3.22);
    assert.equal(bayRadiusAt(poseAt(BLACK), 15, BAY, OBS), 2.8);
    assert.equal(bayRadiusAt(poseAt(ob('centre-goal'), -2), 15, BAY, OBS), 2.8, 'half in the mouth');
    assert.equal(bayRadiusAt(poseAt(BLACK, -5), 15, BAY, OBS), 3.22, '5 in ahead of the claw: not in it');
    assert.equal(bayRadiusAt({ x: 0, y: -40, headingDeg: 0 }, 15, BAY, OBS), 3.22);
    assert.equal(bayRadiusAt(poseAt(RED), 15, null, OBS), 0, 'no bay');
});

const HERE = path.dirname(fileURLToPath(import.meta.url));
const TEAM = JSON.parse(fs.readFileSync(path.join(HERE, 'fixtures', 'projects', 'white_bot.TEST-v10-2026-11-20.json'), 'utf8'));

test('White Bot\'s working route (the teammate, 2026-11-20): the bay changes ONLY its goal checks -- and what it still flags, and why', () => {
    const loaded = loadProject(JSON.stringify(TEAM));
    assert.ok(loaded.ok, loaded.words);
    const r = loaded.project.routes[0];
    const issues = (robots) => checkRoute(r, robots, FIELD, { routes: [r], index: 0 }).map((i) => `${i.code} ${i.stepIndex + 1} ${i.words}`);
    // the same robot with and without its bay (RMP1-L: a check names the robot by its id, so both are White Bot)
    const { frontBay, ...noBay } = ONE;
    const [now, before] = [issues([ONE]), issues([noBay])];
    assert.deepEqual(now.filter((w) => !/^RCO/.test(w)), before.filter((w) => !/^RCO/.test(w)), 'every other check: unchanged');
    const said = (list) => list.filter((w) => /^RCO/.test(w)).map((w) => w.split(':')[0]);
    assert.deepEqual(said(before), [
        'RCO1 5 Where this step ends, the robot\'s 15 in square is 7.8 in into the red goal by the right wall',
        'RCO1 9 Turning in place here, a corner of the robot goes 7.9 in into the red goal by the right wall',
        'RCO1 15 Where this step ends, the robot\'s 15 in square is 1.1 in into the red goal by the right wall']);
    // step 5 ends with the goal's centre 3.3 in BEHIND the claw (2.97 in ahead of the robot's centre; the claw is 6.27): 3.5 in
    // into the bay's back -- on the robot the drive stops against the goal and its 2.5 s time limit carries on. Step 9 turns
    // there. Step 15 ends with the goal at the robot's front-left corner, nowhere near the bay: unchanged
    assert.deepEqual(said(now), [
        'RCO1 5 Where this step ends, the red goal by the right wall is 3.5 in past the edge of the robot\'s open front',
        'RCO1 9 Turning in place here, the edge of the robot\'s open front goes 3.5 in into the red goal by the right wall',
        'RCO1 15 Where this step ends, the robot\'s 15 in square is 1.1 in into the red goal by the right wall']);
    // step 5 placed by its claw on the goal's centre (one drag of the cross): its RCO1 is gone
    const plan = plannedPoses(r);
    const c = centreForClaw({ x: RED.x, y: RED.y }, plan[4].from, SPOT);
    const byClaw = updateStep(r, 4, c);
    const fixed = checkRoute(byClaw, [ONE], FIELD, { routes: [byClaw], index: 0 }).filter((i) => /^RCO/.test(i.code));
    assert.deepEqual(fixed.map((i) => i.stepIndex + 1), [9, 15], 'step 5: clear');
    assert.match(fixed[0].words, /^Turning in place here, the edge of the robot's open front goes 0\.6 in into the red goal/, 'step 9 turns 4.6 deg with the goal in the bay');
});
