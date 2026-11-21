// gently.test.mjs — RMP1-P "Leave gently": the export's exact lines, D2's default, the bounds, the round trip.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { exportRegion, libraryCalls } from '../src/model/export.mjs';
import { newRoute, setStart, addStep, updateStep, normalizeRoute } from '../src/model/route.mjs';
import { followGently, afterDrop, isDrop, gentlyPatch, GENTLY, LIBRARY_TIMEOUT_S } from '../src/model/gently.mjs';
import { checkRoute, applyFix } from '../src/model/checks.mjs';
import { routeLines } from '../src/model/route_lines.mjs';
import { routeVerbs } from '../src/model/run_log.mjs';
import { loadProject, saveProject, newProject } from '../src/model/project.mjs';
import { fixtureRobot, GOLDENS, goldenNames, REPO } from './helpers.mjs';

const X = fixtureRobot('x');
// robot one's two pistons as its description says them: the claw's `grabs` position marked, the flipper's not
const CLAW = { ...X, mechanisms: [...X.mechanisms, { name: 'claw', ident: 'claw', kind: 'pneumatic', positions: ['closed', 'open'], grabs: ['closed'] },
                                  { name: 'flipper', ident: 'flipper', kind: 'pneumatic', positions: ['closed', 'open'] }] };
const base = () => setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 });
const route = (...steps) => steps.reduce((r, s) => addStep(r, s), base());
const body = (r, robot = X) => exportRegion(r, robot).split('\n');

test('ON, 0.5 s / 5 in/s: the exact lines -- the page\'s Wait, then the capped back-up', () => {
    const r = route({ do: 'forward', inches: 6 }, { do: 'backward', inches: 4, ...gentlyPatch(true) });
    const L = body(r);
    const at = L.findIndex((l) => l.startsWith('    // Step 2'));
    assert.deepEqual(L.slice(at, at + 5), [
        '    // Step 2 — LEAVE GENTLY: WAIT 0.5 s, then BACK UP 4 in, straight back, to (-34, -55.7) in, at most 5',
        '    //   in/s. (The brain\'s log calls the wait step 2 and the back-up step 3.)',
        '    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },',
        '           "wait 0.5 s");',
        '    r.reverseTo(-34_in, -55.7_in, {.maxLinearSpeed = 5_in / 1_s});',
    ]);
});

test('ON, edited 1 s / 8 in/s on a turned-first Back up with a label and carry-on: label, onFail before EACH of its three calls', () => {
    const r = route({ do: 'forward', inches: 6 },
                    { do: 'reverseTo', x: -40, y: -50, faceFirst: true, label: 'out', gentlyWaitS: 1, gentlySpeedInPerS: 8, care: { onFail: 'carryOn' } },
                    { do: 'turnTo', headingDeg: 90 });
    const L = body(r);
    const at = L.findIndex((l) => l.startsWith('    r.label("out")'));
    assert.deepEqual(L.slice(at, at + 8), [
        '    r.label("out");',
        '    r.onFail(chassis::carryOn);',
        '    r.then([&] { return ctx.guard.pause(1_s) == sequence::GuardedWaitResult::Satisfied; },',
        '           "wait 1 s");',
        '    r.onFail(chassis::carryOn);',
        '    r.faceAway(-40_in, -50_in);',
        '    r.onFail(chassis::carryOn);',
        '    r.reverseTo(-40_in, -50_in, {.maxLinearSpeed = 8_in / 1_s});',
    ]);
    const words = L.filter((l) => l.startsWith('    //')).map((l) => l.replace(/^ {4}\/\/ {1,3}/, '')).join(' ');
    assert.match(words, /Step 2 — "out": LEAVE GENTLY: WAIT 1 s, then BACK UP to \(-40, -50\) in/);
    assert.match(words, /the drive: at most 8 in\/s\./);
    assert.match(words, /calls the wait step 2, the turn step 3 and the back-up step 4\./);
    assert.match(words, /Step 3 — TURN .*\(The brain's log calls this step 5\.\)/);  // every later number moves by its wait
    assert.equal(libraryCalls(r.steps[1]), 3);
});

test('OFF: the same route exports exactly as before -- no wait, no cap, no words', () => {
    const on = route({ do: 'forward', inches: 6 }, { do: 'backward', inches: 4, ...gentlyPatch(true) });
    const off = updateStep(on, 1, gentlyPatch(false));
    assert.deepEqual(off.steps[1], { do: 'backward', inches: 4 });
    assert.equal(exportRegion(off, X), exportRegion(route({ do: 'forward', inches: 6 }, { do: 'backward', inches: 4 }), X));
    assert.doesNotMatch(exportRegion(off, X), /GENTLY|pause|maxLinearSpeed/);
});

test('the goldens that never turn it on carry no trace of it (their bytes: export.test.mjs)', () => {
    for (const n of goldenNames().filter((g) => !g.includes('gently'))) {
        const text = fs.readFileSync(path.join(GOLDENS, `${n}.route.json`), 'utf8');
        assert.doesNotMatch(text, /gentlyWaitS/, n);
        assert.doesNotMatch(fs.readFileSync(path.join(GOLDENS, `${n}.cpp`), 'utf8'), /LEAVE GENTLY/, n);
    }
});

test('save -> open: the same file, the same export', () => {
    const r = route({ do: 'set', mechanism: 'hood', position: 'up' }, { do: 'backward', inches: 4, gentlyWaitS: 1.25, gentlySpeedInPerS: 7.5 });
    const p = { ...newProject(), routes: [{ ...r, robot: 'x' }] };
    const text = saveProject(p);
    assert.match(text, /"gentlyWaitS": 1.25,\s*"gentlySpeedInPerS": 7.5/);
    const back = loadProject(text);
    assert.ok(back.ok, back.words);
    assert.equal(saveProject(back.project), text);
    assert.equal(exportRegion(back.project.routes, X), exportRegion([{ ...r, robot: 'x' }], X));
});

test('D2: a Back up ADDED right after a drop starts ON (after a claw open; after a drop then waits); after a drive, a close or the FLIPPER: OFF', () => {
    const add = (...before) => {
        const r = route(...before);
        return followGently(addStep(r, { do: 'backward', inches: 4 }), r.steps.length, undefined, CLAW).steps.at(-1);
    };
    const open = { do: 'set', mechanism: 'claw', position: 'open' };
    const on = { do: 'backward', inches: 4, gentlyWaitS: GENTLY.waitS, gentlySpeedInPerS: GENTLY.speedInPerS };
    assert.deepEqual(add({ do: 'forward', inches: 6 }, open), on);
    assert.deepEqual(add(open, { do: 'wait', seconds: 1 }), on);
    assert.deepEqual(add(open, { do: 'waitUntilDone', mechanism: 'claw', boundS: 1 }, { do: 'wait', seconds: 1 }), on);
    assert.deepEqual(add(open, { do: 'forward', inches: 6 }), { do: 'backward', inches: 4 });
    assert.deepEqual(add({ do: 'set', mechanism: 'claw', position: 'closed' }), { do: 'backward', inches: 4 });
    assert.deepEqual(add({ do: 'set', mechanism: 'flipper', position: 'open' }), { do: 'backward', inches: 4 }, 'the front toggle is no drop');
    assert.deepEqual(add(), { do: 'backward', inches: 4 });
    assert.deepEqual(add({ do: 'wait', seconds: 1 }), { do: 'backward', inches: 4 });
});

test('a drop is what the robot\'s description marks: off a `grabs` position or onto a `releases` one -- never a name', () => {
    const Y = fixtureRobot('y');  // its grip: "shut" grabs, "loose" releases (made by the C++)
    // only the keys this chunk reads (RMP1-M's `start` and `wiring` ride along, pinned by its own tests)
    const { name, ident, kind, positions, grabs, releases } = Y.mechanisms.find((m) => m.ident === 'grip');
    assert.deepEqual({ name, ident, kind, positions, grabs, releases }, { name: 'grip', ident: 'grip', kind: 'pneumatic', positions: ['shut', 'loose'],
                                                                          grabs: ['shut'], releases: ['loose'] });
    const set = (mechanism, position) => ({ do: 'set', mechanism, position });
    assert.ok(isDrop(set('grip', 'loose'), Y) && !isDrop(set('grip', 'shut'), Y));
    assert.ok(isDrop(set('claw', 'open'), CLAW) && !isDrop(set('claw', 'closed'), CLAW));
    assert.ok(!isDrop(set('flipper', 'open'), CLAW) && !isDrop(set('hood', 'up'), X) && !isDrop(set('hood', 'down'), X));
    const lone = { mechanisms: [{ ident: 'drop', kind: 'pneumatic', positions: ['in', 'out'], releases: ['out'] }] };
    assert.ok(isDrop(set('drop', 'out'), lone) && !isDrop(set('drop', 'in'), lone));
    assert.ok(!isDrop(set('claw', undefined), CLAW) && !isDrop(set('nope', 'open'), CLAW) && !isDrop(set('claw', 'open'), null));
    assert.ok(!isDrop({ do: 'move', mechanism: 'claw', preset: 'open' }, CLAW));
    assert.ok(!afterDrop([set('claw', 'open'), { do: 'backward', inches: 4 }], 1, X), 'a robot with no claw: no drop');
});

test('D2: a Drive switched to Backwards after a drop turns it on; back to a Drive turns it off; an existing Back up is never changed', () => {
    const r = route({ do: 'set', mechanism: 'claw', position: 'open' }, { do: 'forward', inches: 4 });
    const was = r.steps[1];
    const back = followGently(updateStep(r, 1, { do: 'backward' }), 1, was, CLAW);
    assert.equal(back.steps[1].gentlyWaitS, 0.5);
    const fwd = followGently(updateStep(back, 1, { do: 'forward' }), 1, back.steps[1], CLAW);
    assert.deepEqual(fwd.steps[1], { do: 'forward', inches: 4 });
    // turned off by the student: the next edit leaves it off
    const off = followGently(updateStep(back, 1, gentlyPatch(false)), 1, back.steps[1], CLAW);
    const next = followGently(updateStep(off, 1, { inches: 5 }), 1, off.steps[1], CLAW);
    assert.equal(next.steps[1].gentlyWaitS, undefined);
    assert.ok(afterDrop(next.steps, 1, CLAW));
});

test('its speed is its own: on then off, and Backwards on then off, give back the step EXACTLY (a typed top speed kept)', () => {
    const typed = { do: 'backward', inches: 4, care: { timeoutS: 2, maxSpeedInPerS: 10 } };
    const r = route(typed);
    const on = updateStep(r, 0, gentlyPatch(true));
    assert.deepEqual(on.steps[0], { ...typed, gentlyWaitS: 0.5, gentlySpeedInPerS: 5 });
    assert.deepEqual(updateStep(on, 0, gentlyPatch(false)).steps[0], typed);
    const capped = route({ do: 'set', mechanism: 'claw', position: 'open' }, { do: 'forward', inches: 4, care: { maxSpeedInPerS: 12 } });
    const b = followGently(updateStep(capped, 1, { do: 'backward' }), 1, capped.steps[1], CLAW);
    assert.equal(b.steps[1].gentlyWaitS, 0.5);
    assert.deepEqual(b.steps[1].care, { maxSpeedInPerS: 12 });
    assert.deepEqual(followGently(updateStep(b, 1, { do: 'forward' }), 1, b.steps[1], CLAW).steps[1], capped.steps[1]);
});

test('the back-out speed: the slower of its own and the step\'s typed top speed -- in the export and in PREVIEW\'s lines', () => {
    const at = (care) => {
        const r = route({ do: 'forward', inches: 6 }, { do: 'backward', inches: 4, ...gentlyPatch(true), care });
        return [exportRegion(r, X).match(/maxLinearSpeed = (\S+)_in/)[1], routeLines(r, X).trimEnd().split('\n')[4]];
    };
    assert.deepEqual(at({ maxSpeedInPerS: 10 }), ['5', 'step do=reverseTo x=-34 y=-55.7 face=0 speed=5 wait=0.5']);
    assert.deepEqual(at({ maxSpeedInPerS: 3 }), ['3', 'step do=reverseTo x=-34 y=-55.7 face=0 speed=3 wait=0.5']);
    assert.deepEqual(at({ maxSpeedInPerS: 0 }), ['5', 'step do=reverseTo x=-34 y=-55.7 face=0 speed=5 wait=0.5']);
    assert.deepEqual(at(undefined), ['5', 'step do=reverseTo x=-34 y=-55.7 face=0 speed=5 wait=0.5']);
});

test('loading NEVER turns it on: a saved Back up right after a claw open opens, saves and exports as it was', () => {
    const r = route({ do: 'set', mechanism: 'claw', position: 'open' }, { do: 'backward', inches: 4 });
    const text = saveProject({ ...newProject(), routes: [r] });
    const back = loadProject(text);
    assert.ok(back.ok, back.words);
    assert.equal(back.project.routes[0].steps[1].gentlyWaitS, undefined);
    assert.equal(saveProject(back.project), text);
    assert.equal(exportRegion(normalizeRoute(back.project.routes[0]), CLAW), exportRegion(r, CLAW));
});

const issuesOf = (r, codes = ['RCG1', 'RCG2']) => checkRoute(r, [X], { wallInnerFaceIn: 70.25 }).filter((x) => codes.includes(x.code))
    .map((x) => [x.code, x.level, x.field, x.words]);

test('the bounds: wait above 0 and at most 2 s, speed 2 to 20 in/s -- refused (red) in words outside them, and off a Back up', () => {
    const errs = (step) => issuesOf(route(step), ['RCG1']).map(([c, level, f, w]) => [level, f, w]);
    const ok = (w, v) => ({ do: 'backward', inches: 4, gentlyWaitS: w, gentlySpeedInPerS: v });
    assert.deepEqual(errs(ok(0.5, 5)), []);
    assert.deepEqual(errs(ok(2, 20)), []);
    assert.deepEqual(errs(ok(0.001, 2)), []);
    assert.deepEqual(errs(ok(0, 5)), [['error', 'gentlyWait', 'Leave gently waits more than 0 s and at most 2 s.']]);
    assert.deepEqual(errs(ok(2.001, 5)), [['error', 'gentlyWait', 'Leave gently waits more than 0 s and at most 2 s.']]);
    assert.deepEqual(errs(ok(null, 5)), [['error', 'gentlyWait', 'Leave gently waits more than 0 s and at most 2 s: type it.']]);
    assert.deepEqual(errs(ok(0.5, 1.999)), [['error', 'gentlySpeed', 'Leave gently backs out at 2 to 20 in/s.']]);
    assert.deepEqual(errs(ok(0.5, 20.001)), [['error', 'gentlySpeed', 'Leave gently backs out at 2 to 20 in/s.']]);
    assert.deepEqual(errs({ do: 'backward', inches: 4, gentlyWaitS: 0.5 }), [['error', 'gentlySpeed', 'Leave gently backs out at 2 to 20 in/s: type it.']]);
    assert.deepEqual(errs({ do: 'backward', inches: 4, gentlySpeedInPerS: 5 }), [['error', 'gentlyWait', 'Leave gently waits more than 0 s and at most 2 s: type it.']]);
    assert.deepEqual(errs({ do: 'forward', inches: 4, gentlyWaitS: 0.5 }), [['error', 'gentlyWait', 'Leave gently is for a Back up only.']]);
    // an emptied box keeps the option (a hole the check asks for), never turns it off
    assert.equal(updateStep(route(ok(0.5, 5)), 0, { gentlyWaitS: null }).steps[0].gentlyWaitS, null);
    assert.equal(updateStep(route(ok(0.5, 5)), 0, { gentlySpeedInPerS: null }).steps[0].gentlySpeedInPerS, null);
});

test('its time: a gentle back-up its time limit cannot finish is refused in words (22 in at 5 in/s fits the robot\'s 5 s; 23 does not)', () => {
    const back = (inches, care) => issuesOf(route({ do: 'backward', inches, ...gentlyPatch(true), care }));
    assert.deepEqual(back(22), []);
    assert.deepEqual(back(23), [['RCG2', 'error', 'gentlySpeed', 'At 5 in/s this 23 in back-up needs about 5.1 s, over its 5 s time '
        + 'limit: it would be cut short. Back up less, back out faster, or give it a longer time limit.']]);
    assert.equal(back(26)[0][3].slice(0, 60), 'At 5 in/s this 26 in back-up needs about 5.7 s, over its 5 s');
    assert.deepEqual(back(26, { timeoutS: 6 }), []);  // a longer time limit typed
    assert.match(back(14, { maxSpeedInPerS: 3 })[0][3], /^At 3 in\/s this 14 in/);  // the slower typed top speed counts
    assert.deepEqual(back(13, { maxSpeedInPerS: 3 }), []);
    // faster, the slow-down at the end is longer (the host test plant finishes 44.7 in at 10 in/s, 84.8 at 20)
    const at = (inches, v) => issuesOf(route({ do: 'backward', inches, gentlyWaitS: 0.5, gentlySpeedInPerS: v })).length;
    assert.deepEqual([at(42.9, 10), at(43.1, 10), at(77.9, 20), at(78.1, 20), at(9.2, 2), at(9.3, 2)], [0, 1, 0, 1, 0, 1]);
    assert.deepEqual(issuesOf(route({ do: 'reverseTo', x: -36, y: -80, faceFirst: true, ...gentlyPatch(true) })).map((x) => x[0]), ['RCG2']);
    assert.deepEqual(issuesOf(route({ do: 'backward', inches: 40 })), [], 'off: no time rule of its own');
});

test('the page\'s 5 s is the library\'s: MotionConfig::defaultTimeout, and no robot sets its own', () => {
    const cfg = fs.readFileSync(path.join(REPO, 'include', 'shulib', 'motion', 'motion_config.hpp'), 'utf8');
    assert.match(cfg, new RegExp(`double defaultTimeout = ${LIBRARY_TIMEOUT_S}\\.0;`));
    const robots = fs.readdirSync(path.join(REPO, 'src'), { recursive: true }).filter((f) => /\.(hpp|cpp|h)$/.test(f));
    assert.ok(robots.length > 10);
    assert.deepEqual(robots.filter((f) => fs.readFileSync(path.join(REPO, 'src', f), 'utf8').includes('defaultTimeout')), []);
});

test('Go around on a gentle Back up: the wait and the slow speed move to the FIRST leg (the one off the drop); the second is plain', () => {
    const r = route({ do: 'set', mechanism: 'claw', position: 'open' },
                    { do: 'reverseTo', x: -60, y: -40, faceFirst: true, ...gentlyPatch(true), care: { onFail: 'carryOn' } });
    const out = applyFix(r, { kind: 'detour', stepIndex: 1, x: -50, y: -50 }, addStep, updateStep);
    assert.deepEqual(out.steps.slice(1), [
        { do: 'reverseTo', x: -50, y: -50, faceFirst: true, gentlyWaitS: 0.5, gentlySpeedInPerS: 5 },
        { do: 'reverseTo', x: -60, y: -40, faceFirst: true, care: { onFail: 'carryOn' } },
    ]);
    const plain = route({ do: 'reverseTo', x: -60, y: -40, faceFirst: true });
    assert.deepEqual(applyFix(plain, { kind: 'detour', stepIndex: 0, x: -50, y: -50 }, addStep, updateStep).steps,
                     [{ do: 'reverseTo', x: -50, y: -50, faceFirst: true }, { do: 'reverseTo', x: -60, y: -40, faceFirst: true }]);
});

test('PREVIEW runs what the export writes: the lines carry the wait and the cap; the brain\'s verbs put the wait first', () => {
    const r = route({ do: 'forward', inches: 6 }, { do: 'backward', inches: 4, gentlyWaitS: 0.5, gentlySpeedInPerS: 5 },
                    { do: 'reverseTo', x: -40, y: -50, faceFirst: true, gentlyWaitS: 1, gentlySpeedInPerS: 8 });
    const lines = routeLines(r, X).trimEnd().split('\n');
    assert.equal(lines[4], 'step do=reverseTo x=-34 y=-55.7 face=0 speed=5 wait=0.5');
    assert.equal(lines[5], 'step do=reverseTo x=-40 y=-50 face=1 speed=8 wait=1');
    assert.deepEqual(routeVerbs(r, X), ['driveTo', 'wait 0.5 s', 'reverseTo', 'wait 1 s', 'faceAway', 'reverseTo']);
    assert.deepEqual(r.steps.map(libraryCalls), [1, 2, 3]);
});
