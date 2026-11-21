// obstacles.test.mjs — THE FIELD'S SOLID STRUCTURES (RMP1-H): the goals and the Loaders from robots.json, every step's
// swept square checked against them, the "on purpose" flag, the one-click way around, and PREVIEW's THROUGH a
// structure. Adversarial: each rule pinned at its edge, each route built to slip past ONE rule.
import test from 'node:test';
import assert from 'node:assert/strict';
import { checkRoute, applyFix, ROBOT_TWO_ONLY } from '../src/model/checks.mjs';
import { newRoute, setStart, addStep, updateStep, normalizeStep, plannedPoses } from '../src/model/route.mjs';
import { sweep, signedGap, runThrough, nearStructures, TOUCH_IN, CLEAR_IN, THROUGH_IN } from '../src/model/obstacles.mjs';
import { reflect, stepPoses } from '../src/model/pair.mjs';
import { squareCorners } from '../src/model/geometry.mjs';
import { routeKey, playbackOf } from '../src/model/playback.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { newProject, saveProject, loadProject } from '../src/model/project.mjs';
import { fixtureRobot, seeded } from './helpers.mjs';
import { fixture } from './standin_engine.mjs';

const X = fixtureRobot('x');
const FIELD = X.field;
const OBS = FIELD.obstacles;
const ob = (id) => OBS.find((o) => o.id === id);
const FLAT = Math.max(...ob('centre-goal').footprint.map((p) => p[1]));  // the centre goal's flat, from the C++
const HALF = 7.5;

function route(start, steps, extra = { mirroredFor: undefined, together: undefined }) {
    let r = setStart({ ...newRoute('test'), ...extra }, start);
    for (const s of steps) {
        r = addStep(r, s);
    }
    return r;
}
const mine = (r) => checkRoute(r, [X], FIELD, { routes: [newRoute(), r], index: 1 }).filter((i) => /^RCO/.test(i.code));
const sq = (x, y, h = 0) => squareCorners(x, y, h, 15);
const poly = (o) => o.footprint.map(([x, y]) => ({ x, y }));

test('the structures come from robots.json (the C++, the manual): nine goals, four Loaders, each footprint where the drawing puts it', () => {
    assert.equal(OBS.length, 13);
    assert.equal(new Set(OBS.map((o) => o.id)).size, 13);
    assert.equal(OBS.filter((o) => o.kind === 'goal').length, 9);
    assert.equal(OBS.filter((o) => o.kind === 'Loader').length, 4);
    for (const o of OBS) {
        assert.match(o.source, /Game Manual Appendix A/);
        const xs = o.footprint.map((p) => p[0]);
        const ys = o.footprint.map((p) => p[1]);
        if (o.kind === 'goal') {  // 5.61 in flat to flat, R3.22 corners (A7)
            assert.ok(Math.abs(Math.max(...xs) - o.x - 2.805) < 0.002 && Math.abs(o.x - Math.min(...xs) - 2.805) < 0.002, o.id);
            assert.ok(o.footprint.every(([x, y]) => Math.hypot(x - o.x, y - o.y) <= 3.226), o.id);  // on the 0.001 grid
        } else {  // 4.02 wide, 3.74 deep from the red or blue wall's face (A9, A10)
            assert.ok(Math.abs(Math.max(...xs) - Math.min(...xs) - 4.02) < 0.002 && Math.abs(Math.max(...ys) - Math.min(...ys) - 3.74) < 0.002, o.id);
            assert.ok(Math.abs(Math.abs(o.y) - FIELD.wallInnerFaceIn) < 0.001 && Math.abs(Math.abs(o.x) - 58.764) < 0.001, o.id);
        }
    }
    assert.deepEqual([ob('centre-goal').x, ob('centre-goal').y], [0, 0]);
    assert.deepEqual([ob('red-goal-right-wall').x, ob('red-goal-right-wall').y], [47.091, -23.547]);
});

test('a drive THROUGH a goal is red: which one, how far in, where, and a way around that is clear', () => {
    const r = route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y: 0, faceFirst: true }]);
    const [e] = mine(r);
    assert.equal(e.code, 'RCO1');
    assert.equal(e.level, 'error');
    assert.equal(e.stepIndex, 0);
    assert.equal(e.obstacle.id, 'centre-goal');
    assert.match(e.words, /goes 10\.4 in into the centre goal: it is solid/);  // 7.5 + 2.805, the square centred on it (a red rounds up)
    assert.ok(Math.abs(e.obstacle.at.x) <= 0.25 && e.obstacle.at.y === 0, 'the moment: centred on the goal');
    assert.match(e.fix.words, /^Go around: add a point at \(/);
    const fixed = applyFix(r, e.fix, addStep, updateStep);
    assert.equal(fixed.steps.length, 2);
    assert.deepEqual([fixed.steps[0].x, fixed.steps[0].y], [e.fix.x, e.fix.y]);
    assert.ok(fixed.steps.every((s) => s.faceFirst), 'the corner is turned');
    assert.deepEqual(checkRoute(fixed, [X], FIELD, { routes: [newRoute(), fixed], index: 1 }).filter((i) => i.level === 'error' || /^RCO/.test(i.code)), []);
});

test('the way around never leaves the field: a leg between a goal and a wall goes around on the open side', () => {
    const r = route({ x: -52, y: -40, headingDeg: 90 }, [{ do: 'driveTo', x: -52, y: -5, faceFirst: true }]);
    const [e] = mine(r);
    assert.equal(e.obstacle.id, 'black-goal-left-wall');
    assert.ok(e.fix && e.fix.x > -47, `around on the field's side, not into the left wall (${e.fix && e.fix.x})`);
    const fixed = applyFix(r, e.fix, addStep, updateStep);
    assert.deepEqual(checkRoute(fixed, [X], FIELD, { routes: [newRoute(), fixed], index: 1 }).filter((i) => i.level === 'error'), []);
});

test('the way around never trades one structure for another: when every point beside it comes near another, none is offered', () => {
    // from (-60, -54) to (-36, -12) through the black goal by the left wall: around it on its open side runs the robot
    // into the black goal by the red wall's corner
    const r = route({ x: -60, y: -54, headingDeg: 0 }, [{ do: 'driveTo', x: -36, y: -12, faceFirst: true }]);
    const [e] = mine(r);
    assert.equal(e.code, 'RCO1');
    assert.equal(e.fix, null);
    assert.match(e.words, /Move the point, or go around it\.$/);
});

test(`passing within ${CLEAR_IN} in is amber; at the edge: ${CLEAR_IN + 0.005} in clear, ${CLEAR_IN - 0.005} in amber`, () => {
    const pass = (gap) => mine(route({ x: -30, y: FLAT + HALF + gap, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y: FLAT + HALF + gap }]));
    assert.deepEqual(pass(CLEAR_IN + 0.005), []);
    const [w] = pass(CLEAR_IN - 0.005);
    assert.equal(w.code, 'RCO2');
    assert.equal(w.level, 'warn');
    assert.match(w.words, /passes 0\.9 in from the centre goal: closer than 1 in/, 'an amber rounds down: never "1.0 in" closer than 1 in');
    assert.deepEqual(pass(0.6).map((i) => [i.code, i.obstacle.id]), [['RCO2', 'centre-goal']]);
});

test('ending AGAINST a goal (placing a cone) is a touch, not red; only 0.5 in past it is red', () => {
    assert.equal(TOUCH_IN, 0.5, 'the brief\'s allowance (D4)');
    const end = (into) => mine(route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -(FLAT + HALF) + into }]));
    const [t] = end(0.3);
    assert.equal(t.code, 'RCO2');
    assert.match(t.words, /touches the centre goal here \(0\.3 in into it\)/);
    assert.deepEqual(end(0.49).map((i) => i.code), ['RCO2']);
    assert.deepEqual(end(0.51).map((i) => i.code), ['RCO1']);
    assert.deepEqual(end(0).map((i) => i.code), ['RCO2'], 'flush: touching');
});

test('"touches it on purpose" silences THAT structure\'s amber, never a red, never another structure\'s', () => {
    const at = (into, touches) => mine(route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -(FLAT + HALF) + into, touches }]));
    assert.deepEqual(at(0.3, ['centre-goal']), []);
    assert.deepEqual(at(0.3, ['red-goal-right-wall']).map((i) => i.code), ['RCO2']);
    assert.deepEqual(at(0.8, ['centre-goal']).map((i) => i.code), ['RCO1']);
});

test('a step is judged on what IT does: backing away from a goal it touches is clean; turning there is not', () => {
    const r = route({ x: 0, y: -(FLAT + HALF) + 0.3, headingDeg: 90 }, [{ do: 'reverseTo', x: 0, y: -30 }]);
    assert.deepEqual(mine(r), []);
    // the step that goes IN is the red one; the one that backs out of it is not (one red, where the mistake is)
    const inOut = route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -6 }, { do: 'reverseTo', x: 0, y: -30 }]);
    assert.deepEqual(mine(inOut).map((i) => [i.code, i.stepIndex]), [['RCO1', 0]]);
    // a turn in place beside the goal: both headings clear (1.2 in), a corner swings 1.9 in into it at 45 deg
    const t = route({ x: 0, y: -(FLAT + HALF + 1.2), headingDeg: 90 }, [{ do: 'turnTo', headingDeg: 0 }]);
    const [e] = mine(t);
    assert.equal(e.code, 'RCO1');
    assert.equal(e.field, 'headingDeg');
    assert.match(e.words, /^Turning in place here, a corner of the robot goes 2\.0 in into the centre goal/);  // 1.907, rounded up
});

test('a curve is checked along the library\'s own curve, not its points: points clear, the bend through a goal is red', () => {
    const r = route({ x: -40, y: 16, headingDeg: -45 }, [{ do: 'curveTo', points: [{ x: 16, y: -16 }, { x: 30, y: -16 }] }]);
    for (const p of r.steps[0].points) {
        assert.ok(OBS.every((o) => signedGap(sq(p.x, p.y, -45), poly(o)) > CLEAR_IN && signedGap(sq(p.x, p.y, 0), poly(o)) > CLEAR_IN),
                  `point (${p.x}, ${p.y}) clear on its own`);
    }
    const [e] = mine(r);
    assert.equal(e.code, 'RCO1');
    assert.equal(e.field, 'points');
    assert.equal(e.obstacle.id, 'centre-goal');
    assert.match(e.words, /Move a curve point away from it\.$/);
    assert.equal(e.fix, null, 'no one-click detour for a curve');
});

test('the sweep between two samples is the hull: a pass the samples straddle is not missed', () => {
    const [h] = sweep([{ x: -15, y: 0, headingDeg: 0 }, { x: 15, y: 0, headingDeg: 0 }], [ob('centre-goal')], 15);
    assert.ok(signedGap(sq(-15, 0), poly(ob('centre-goal'))) > 4 && signedGap(sq(15, 0), poly(ob('centre-goal'))) > 4);
    assert.equal(h.dmin, 0);
});

test('the start: inside a structure is red; touching one is not', () => {
    const [e] = mine(route({ x: 58.764, y: -62, headingDeg: 90 }, []));  // the red wall's right Loader: 3.0 in in
    assert.equal(e.code, 'RCO1');
    assert.equal(e.stepIndex, -1);
    assert.match(e.words, /square is 3\.1 in inside the right Loader on the red wall/);  // 3.035, rounded up
    assert.deepEqual(mine(route({ x: 58.764, y: -66.465 + HALF - 0.3, headingDeg: 90 }, [])), []);
});

test('robot two (mirrored): its reflected path is checked too — the Loaders are NOT mirror images — and PREVIEW does not wait for it', () => {
    const mirror = X.skillsMirror;
    const r = route({ x: 62, y: -40, headingDeg: -90 }, [{ do: 'driveTo', x: 62, y: -55 }], { mirroredFor: 'two', together: false });
    const two = reflect({ x: 62, y: -55, headingDeg: -90 }, mirror);
    assert.ok(signedGap(sq(two.x, two.y, two.headingDeg), poly(ob('loader-red-wall-right'))) < -1, 'robot two ends in the red wall\'s Loader');
    const [e] = mine(r);
    assert.equal(e.code, 'RCO3');
    assert.ok(ROBOT_TWO_ONLY.has('RCO3'));
    assert.match(e.words, /^Black Bot runs this mirrored, and its 15 in square goes 3\.1 in into the right Loader on the red wall during this step \(the Loaders are not mirror images\): it is solid/);
    assert.deepEqual(mine({ ...r, mirroredFor: undefined }), [], 'robot one alone: the right wall has no Loader');
});

test('the flag is page-only: stored sorted and once, saved and opened back, never in the export or the preview\'s key', () => {
    assert.deepEqual(normalizeStep({ do: 'wait', seconds: 1, touches: ['b', 'a', 'b', 3, ''] }).touches, ['a', 'b']);
    assert.equal(normalizeStep({ do: 'wait', seconds: 1, touches: [] }).touches, undefined);
    const plain = route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -10 }], {});
    const flagged = updateStep(plain, 0, { touches: ['centre-goal'] });
    assert.equal(exportRegion([flagged], X), exportRegion([plain], X));
    assert.equal(routeKey(flagged), routeKey(plain));
    const text = saveProject({ ...newProject(), routes: [flagged] });
    const back = loadProject(text);
    assert.ok(back.ok, back.words);
    assert.deepEqual(back.project.routes[0].steps[0].touches, ['centre-goal']);
    assert.equal(saveProject(back.project), text);
});

test(`PREVIEW: a run whose square goes more than ${THROUGH_IN} in into a structure is THROUGH it, as a wall`, () => {
    const f = fixture('ready').result;
    const one = f.one;
    const k = Math.floor(one.samples.length / 2);
    const [, x, y] = one.samples[k];
    const block = (dx) => ({ id: 'b', name: 'test block', kind: 'goal', x: x + dx, y,
                             footprint: [[x + dx - 1, y - 1], [x + dx + 1, y - 1], [x + dx + 1, y + 1], [x + dx - 1, y + 1]] });
    const hit = runThrough(one.samples, [block(0)], 15);
    assert.ok(hit && hit.s <= k && hit.element === 'test block' && hit.depth > THROUGH_IN);
    // the first sample past the allowance, never one before it
    const before = one.samples[hit.s - 1];
    assert.ok(signedGap(sq(before[1], before[2], (before[3] * 180) / Math.PI), poly(block(0))) >= -THROUGH_IN);
    const p = playbackOf(f, {}, { obstacles: [block(0)], size: 15 });
    assert.equal(p.one.through.element, 'test block');
    assert.match(p.headline.word, /, but THROUGH the test block$/);
    assert.equal(p.headline.mark, 'failed');
    const failed = p.one.steps.find((s) => s.state === 'failed');
    assert.equal(failed.words, 'Went THROUGH the test block');
    assert.match(failed.library[0], /\(\d+\.\d\d in into it\): the host test plant drives through it\.$/);
    assert.equal(playbackOf(f).one.through, null, 'no structures given: nothing');
    // an engine THROUGH earlier than the page's is kept; a later one is dropped (the robot never gets there)
    const wall = (s) => ({ kind: 'THROUGH', element: 'left wall', s, t: one.samples[s][0], step: 1, depth: 1.5 });
    const early = playbackOf({ ...f, one: { ...one, field: [wall(hit.s - 1)] } }, {}, { obstacles: [block(0)], size: 15 });
    assert.equal(early.one.through.element, 'left wall');
    const late = playbackOf({ ...f, one: { ...one, field: [wall(hit.s + 5)] } }, {}, { obstacles: [block(0)], size: 15 });
    assert.equal(late.one.through.element, 'test block');
    assert.equal(late.one.notes.filter((n) => n.kind === 'THROUGH').length, 1);
});

// ── the review's round (2026-11-20): each rule below pinned by a mutant that passed every test before it ──

const RED = ob('red-goal-right-wall');  // a red goal: its body narrows above its lip (A7), overhangIn 1.057
const westOf = (o, into, touches, extra) => route({ x: o.x - 30, y: o.y, headingDeg: 0 },
    [{ do: 'driveTo', x: o.x - Math.max(...o.footprint.map((p) => o.x - p[0])) - HALF + into, y: o.y, touches }], extra);

test('a red or blue goal: a front that ENDS over its base on purpose (placing a cone) is clear up to the overhang; nothing else is', () => {
    assert.equal(RED.overhangIn, 1.057, 'A7: (142.5 - 88.8) / 2 mm, from the C++');
    assert.equal(ob('black-goal-right-wall').overhangIn, 0);
    assert.equal(ob('centre-goal').overhangIn, 0);
    const [e] = mine(westOf(RED, 0.9));
    assert.equal(e.code, 'RCO1', 'not said: red (past the 0.5 in touch)');
    assert.match(e.words, /^Where this step ends, the robot's 15 in square is 0\.9 in into the red goal by the right wall's base\. If its front reaches over the base on purpose \(placing a cone\), say so below/);
    assert.equal(e.fix, null, 'no way around a point that is IN it');
    const r = westOf(RED, 0.9);
    assert.deepEqual(nearStructures(r, plannedPoses(r), FIELD, 15, 0), ['red-goal-right-wall'], 'the switch that clears it is offered');
    assert.deepEqual(mine(westOf(RED, 0.9, ['red-goal-right-wall'])), [], 'said: clear');
    assert.deepEqual(mine(westOf(RED, 1.1, ['red-goal-right-wall'])).map((i) => i.code), ['RCO1'], 'past the overhang: red');
    const black = ob('black-goal-right-wall');
    assert.deepEqual(mine(westOf(black, 0.9, [black.id])).map((i) => i.code), ['RCO1'], 'a black goal stands full width: no overhang');
});

test('the overhang on a mirrored route (the page\'s default): robot two places its cone on the mirror-image goal, so the one switch clears both', () => {
    assert.equal(newRoute().mirroredFor, 'two', 'the default route is mirrored');
    const codes = (r, robots = [X]) => checkRoute(r, robots, FIELD, { routes: [newRoute(), r], index: 1 })
        .filter((i) => /^RCO/.test(i.code)).map((i) => `${i.code} ${i.words}`);
    assert.deepEqual(codes(westOf(RED, 0.9, [RED.id], {})), [], 'said: robot one AND robot two clear');
    const [unsaid] = codes(westOf(RED, 0.9, undefined, {}));
    assert.match(unsaid, /^RCO1 .*0\.9 in into the red goal by the right wall's base\. If its front reaches over the base on purpose/, 'unsaid: robot one\'s red, with the switch');
    assert.match(codes(westOf(RED, 1.1, [RED.id], {}))[0], /^RCO1 .*1\.1 in into the red goal by the right wall: it is solid/);
    // a robot two 1 in bigger ends 0.5 in deeper: 0.9 in into the red goal by the RED wall (the mirror image), past the touch
    const X16 = { ...X, drive: { ...X.drive, footprintIn: 16 } };
    const r = westOf(RED, 0.4, undefined, {});
    const [two] = codes(r, [X, X16]);
    assert.equal(two, 'RCO3 Black Bot runs this mirrored, and its 16 in square goes 0.9 in into the red goal by the red wall during this step: it is solid. Move the point so Black Bot\'s path clears it too.',
        'no Loader in it: no Loader words');
    assert.deepEqual(codes({ ...r, steps: [{ ...r.steps[0], touches: [RED.id] }] }, [X, X16]), [], 'the switch on robot one\'s goal lifts robot two\'s twin');
    assert.match(codes({ ...r, steps: [{ ...r.steps[0], touches: [ob('red-goal-red-wall').id] }] }, [X, X16])[0], /^RCO3 /,
        'the twin\'s own id on robot one\'s step lifts nothing for robot two');
});

test('over a base on purpose: up to 1 in (PREVIEW\'s THROUGH, inside A7\'s 1.057), the guide\'s number; past it reads 1.1', () => {
    assert.ok(RED.overhangIn > THROUGH_IN, 'the data keeps A7\'s number; the check stops at PREVIEW\'s');
    assert.match(mine(westOf(RED, 1.0))[0].words, /square is 1\.0 in into the red goal by the right wall's base\. If its front reaches over/);
    assert.deepEqual(mine(westOf(RED, 1.0, [RED.id])), [], 'said: 1 in exactly is not past it');
    for (const into of [1.01, 1.05]) {
        assert.match(mine(westOf(RED, into, [RED.id]))[0].words, /square is 1\.1 in into the red goal by the right wall: it is solid\. Stop short of it/);
        assert.doesNotMatch(mine(westOf(RED, into))[0].words, /say so below/, 'no switch would clear it: none promised');
    }
    assert.match(mine(westOf(RED, 0.51))[0].words, /square is 0\.6 in into the red goal by the right wall's base/, 'past the touch: never 0.5');
    // 1 in past by floating point only (-6.4 + 7.5 - 0.1) is 1 in: the words promise the switch, and the switch clears it
    const syn = { id: 'syn', name: 'test goal', kind: 'goal', x: 3.1, y: 0, overhangIn: 1.057, footprint: [[0.1, -3], [6.1, -3], [6.1, 3], [0.1, 3]] };
    const at = (touches) => {
        const r = route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'driveTo', x: -6.4, y: 0, touches }]);
        assert.ok(-sweep(stepPoses(r.steps[0], plannedPoses(r)[0]), [syn], 15)[0].dmin > 1, 'the noise is there');
        return checkRoute(r, [X], { ...FIELD, obstacles: [syn] }, { routes: [newRoute(), r], index: 1 }).filter((i) => /^RCO/.test(i.code));
    };
    assert.match(at()[0].words, /square is 1\.0 in into the test goal's base\. If its front reaches over/);
    assert.deepEqual(at(['syn']), []);
});

test('"say so below" only when the switch is there: a red no switch lifts is named before one it does', () => {
    // the turn in place clips the black goal by the left wall; the step then ends 1 in over the blue goal's base
    const r = route({ x: -47, y: -10.4, headingDeg: 0 }, [{ do: 'driveTo', x: -47.09, y: 14.21, faceFirst: true }]);
    const [e] = mine(r);
    assert.match(e.words, /^Turning in place here, a corner of the robot goes \d\.\d in into the black goal by the left wall: it is solid/);
    assert.equal(e.obstacle.id, 'black-goal-left-wall');
    assert.deepEqual(nearStructures(r, plannedPoses(r), FIELD, 15, 0), []);
    // and everywhere: 1000 seeded drives ending over a red or blue goal's base (the verifier's search)
    const rnd = seeded(7);
    const goals = OBS.filter((o) => o.overhangIn > 0);
    let promised = 0;
    for (let n = 0; n < 1000; ++n) {
        const g = goals[Math.floor(rnd() * goals.length)];
        const a = Math.floor(rnd() * 8) * Math.PI / 4;
        const dist = 2.805 + HALF - 0.5 - rnd() * 0.6;
        const [ex, ey] = [g.x + Math.cos(a) * dist, g.y + Math.sin(a) * dist];
        const L = 15 + rnd() * 40;
        const [sx, sy] = [ex + Math.cos(a) * L + (rnd() - 0.5), ey + Math.sin(a) * L + (rnd() - 0.5)];
        if (Math.abs(sx) > 60 || Math.abs(sy) > 60) {
            continue;
        }
        const t = route({ x: Math.round(sx * 10) / 10, y: Math.round(sy * 10) / 10, headingDeg: 0 },
            [{ do: 'driveTo', x: Math.round(ex * 100) / 100, y: Math.round(ey * 100) / 100, faceFirst: true }]);
        const red = mine(t).find((i) => i.code === 'RCO1' && i.stepIndex === 0);
        if (red && /say so below/.test(red.words)) {
            ++promised;
            assert.ok(nearStructures(t, plannedPoses(t), FIELD, 15, 0).includes(red.obstacle.id), `${JSON.stringify(t.start)} -> ${JSON.stringify(t.steps[0])}`);
        }
    }
    assert.ok(promised > 100, `the search reached the overhang (${promised})`);
});

test('on the way, ANY overlap is red (D3) -- the touch allowance and the switch are for where a step ends (D4)', () => {
    const along = (into, touches) => mine(route({ x: -30, y: -(FLAT + HALF) + into, headingDeg: 0 },
        [{ do: 'driveTo', x: 30, y: -(FLAT + HALF) + into, touches }]));
    const [e] = along(0.3);
    assert.equal(e.code, 'RCO1');
    assert.match(e.words, /^On the way, near \(-?[\d.]+, -10\), the robot's 15 in square goes 0\.3 in into the centre goal/);
    assert.deepEqual(along(0.3, ['centre-goal']).map((i) => i.code), ['RCO1'], 'on purpose never lifts a red on the way');
    assert.deepEqual(along(0.02).map((i) => i.code), ['RCO1']);
    assert.match(along(0.02)[0].words, /goes 0\.1 in into/, 'a red never reads 0.0');
    assert.deepEqual(along(0).map((i) => i.code), ['RCO2'], 'flush on the way: a touch');
    // a red AT its limit never reads as the limit: 0.55 in where it ends says 0.6
    const end = mine(route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -(FLAT + HALF) + 0.51 }]));
    assert.match(end[0].words, /square is 0\.6 in into the centre goal: it is solid\. Stop short of it: move the point\.$/);
    // ...and an amber touch never reads past it: 0.499 says 0.4; exactly 0.5 is not MORE than 0.5 (amber)
    const touch = (into) => mine(route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -(FLAT + HALF) + into }]));
    assert.match(touch(0.499)[0].words, /\(0\.4 in into it\)/);
    assert.deepEqual(touch(0.5).map((i) => i.code), ['RCO2']);
});

test('square-on, stopping 0.5 in short of a goal\'s flat: amber (the goal\'s corners against the square\'s edge)', () => {
    const [w] = mine(route({ x: 0, y: -30, headingDeg: 90 }, [{ do: 'driveTo', x: 0, y: -(FLAT + HALF) - 0.5 }]));
    assert.equal(w.code, 'RCO2');
    assert.match(w.words, /^On the way the robot passes 0\.5 in from the centre goal/);
});

test('the words follow the step and the structure: a Loader is for taking a load; a straight drive has inches', () => {
    const lo = ob('loader-red-wall-left');
    const face = Math.max(...lo.footprint.map((p) => p[1]));  // its face into the field
    const [w] = mine(route({ x: lo.x, y: -40, headingDeg: -90 }, [{ do: 'driveTo', x: lo.x, y: face + HALF - 0.3 }]));
    assert.equal(w.code, 'RCO2');
    assert.equal(w.obstacle.id, lo.id);
    assert.match(w.words, /\(taking a load\)/);
    assert.doesNotMatch(w.words, /cone/);
    const [f] = mine(route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'forward', inches: 60 }]));
    assert.equal(f.code, 'RCO1');
    assert.equal(f.field, 'inches');
    assert.match(f.words, /Change the inches\.$/);
});

test('the "on purpose" switch is offered only where it clears something: an amber, a liftable end -- never on a red, a clean step', () => {
    const at = (r, i = 0) => nearStructures(r, plannedPoses(r), FIELD, 15, i);
    const amber = route({ x: -30, y: FLAT + HALF + 0.6, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y: FLAT + HALF + 0.6 }]);
    assert.deepEqual(at(amber), ['centre-goal'], 'ONE switch: the structure it comes near, not every one');
    const red = route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y: 0 }]);
    assert.deepEqual(at(red), [], 'through it: no switch clears that');
    const clean = route({ x: -60, y: 0, headingDeg: 90 }, [{ do: 'driveTo', x: -60, y: 10 }]);
    assert.deepEqual(at(clean), []);
    assert.deepEqual(mine(clean), []);
});

test('the way around: a drive not turned first gets both legs turned; a back-up stays a back-up; the near side first', () => {
    const r = route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y: 0, faceFirst: false }]);
    const [e] = mine(r);
    const fixed = applyFix(r, e.fix, addStep, updateStep);
    assert.deepEqual(fixed.steps.map((s) => s.faceFirst), [true, true]);
    assert.deepEqual(checkRoute(fixed, [X], FIELD, { routes: [newRoute(), fixed], index: 1 }).filter((i) => i.level === 'error' || /^RCO/.test(i.code)), []);
    const back = route({ x: 30, y: 0, headingDeg: 0 }, [{ do: 'reverseTo', x: -30, y: 0, faceFirst: true }]);
    const [b] = mine(back);
    const bf = applyFix(back, b.fix, addStep, updateStep);
    assert.deepEqual(bf.steps.map((s) => s.do), ['reverseTo', 'reverseTo'], 'backing up both legs, never a turn to drive forward');
    assert.deepEqual(checkRoute(bf, [X], FIELD, { routes: [newRoute(), bf], index: 1 }).filter((i) => i.level === 'error' || /^RCO/.test(i.code)), []);
    // a line just above the goal's centre goes around above it; just below, below it (the short way)
    const side = (y) => mine(route({ x: -30, y, headingDeg: 0 }, [{ do: 'driveTo', x: 30, y, faceFirst: true }]))[0].fix.y;
    assert.ok(side(1) > 0 && side(-1) < 0, `${side(1)} / ${side(-1)}`);
});

test('the deepest structure is the one named: through the centre goal 4.7 in, clipping another 1.7 in', () => {
    const [e] = mine(route({ x: -30, y: 0, headingDeg: 0 }, [{ do: 'driveTo', x: 40, y: 14, faceFirst: true }]));
    assert.equal(e.obstacle.id, 'centre-goal');
    assert.match(e.words, /goes 4\.\d in into the centre goal/);
});

test('the start\'s 0.5 in allowance at its edge; robot two\'s mirrored start inside a Loader', () => {
    const start = (into) => mine(route({ x: 0, y: -(FLAT + HALF) + into, headingDeg: 90 }, []));
    assert.deepEqual(start(0.49), []);
    assert.deepEqual(start(0.51).map((i) => [i.code, i.stepIndex]), [['RCO1', -1]]);
    assert.deepEqual(start(1.0).map((i) => [i.code, i.stepIndex]), [['RCO1', -1]]);
    const two = mine(route({ x: 60.965, y: -58.764, headingDeg: 180 }, [], { mirroredFor: 'two', together: false }));
    assert.deepEqual(two.map((i) => [i.code, i.stepIndex, i.obstacle.id]), [['RCO3', -1, 'loader-red-wall-right']]);
    assert.match(two[0].words, /^Black Bot \(mirrored\) starts \d\.\d in inside the right Loader on the red wall: it is solid\. Move White Bot's start/);
    assert.ok(ROBOT_TWO_ONLY.has(two[0].code));
});

test('PREVIEW: a turned square is a turned square (a corner 1.4 in into a goal is THROUGH); robot two\'s run is checked too', () => {
    const turned = [[0, -40, 0, 0], [0.1, -12, 0, Math.PI / 4], [0.2, -40, 0, 0]];
    const hit = runThrough(turned, OBS, 15);
    assert.ok(hit && hit.element === 'centre goal' && hit.s === 1 && Math.abs(hit.depth - (10.607 - 12 + 2.805)) < 0.01, JSON.stringify(hit));
    assert.equal(runThrough(turned.map(([t, x, y]) => [t, x, y, 0]), OBS, 15), null, 'square to the goal: 1.7 in clear');
    const f = fixture('ready').result;
    const [, x, y] = f.one.samples[Math.floor(f.one.samples.length / 2)];
    const block = { id: 'b', name: 'test block', kind: 'goal', x, y, footprint: [[x - 1, y - 1], [x + 1, y - 1], [x + 1, y + 1], [x - 1, y + 1]] };
    const p = playbackOf({ ...f, two: f.one }, {}, { obstacles: [block], size: 15 });
    assert.equal(p.two.through.element, 'test block');
});

test('the gap is measured both ways and exactly: corner to corner is the diagonal, not an axis\'s bound', () => {
    const box = (x, y) => [{ x, y }, { x: x + 1, y }, { x: x + 1, y: y + 1 }, { x, y: y + 1 }];
    assert.ok(Math.abs(signedGap(box(0, 0), box(2, 2)) - Math.SQRT2) < 1e-12, 'corner to corner: sqrt 2, never the axes\' 1');
    assert.ok(signedGap(box(0, 0), box(5, 5), 2) >= 2, 'past `enough`: a bound at least that big');
    assert.equal(signedGap(box(0, 0), box(0.5, 0.25)), -0.5, 'overlapping: minus the least push out');
});
