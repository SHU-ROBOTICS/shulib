// named_routes.test.mjs — RMP1-C: named routes (D1, D2), "both robots at the same time?" (D3), robot two's too-close
// check (D5), a lift's plain steps (D7), a pneumatic's buttons (D8) and small moves (D9). Adversarial: each case is
// built to slip past one rule, and each rule is pinned at its edge.
import test from 'node:test';
import assert from 'node:assert/strict';
import { checkRoute, applyFix, wallOvershoot, ROBOT_TWO_ONLY, nameProblem } from '../src/model/checks.mjs';
import { newRoute, setStart, addStep, updateStep, plannedPoses, normalizeRoute } from '../src/model/route.mjs';
import { newProject, loadProject, saveProject, addRoute, renameRouteAt, duplicateRoute, deleteRoute, freeName, routeCap } from '../src/model/project.mjs';
import { exportRegion, routeIdents } from '../src/model/export.mjs';
import { plannedHeights, moveTarget, bottomOf } from '../src/model/lift.mjs';
import { reflect, squareGap, tooClose, stepPoses } from '../src/model/pair.mjs';
import { historyOf, commit, undo, redo } from '../src/model/undo.mjs';
import { mechanismActions, positionWords } from '../src/ui/words.mjs';
import fs from 'node:fs';
import path from 'node:path';
import { fixtureRobot, ledgerRobot, GOLDENS } from './helpers.mjs';

const X = fixtureRobot('x');
const FIELD = X.field;
const START = { x: -36, y: -55.7, headingDeg: 0 };
const route = (steps, start = START, extra = { together: false }) => steps.reduce((r, s) => addStep(r, s), setStart({ ...newRoute(), ...extra }, start));
const codes = (issues) => issues.map((i) => i.code);
const at = (issues, code) => issues.filter((i) => i.code === code);

// ── D1: the set of routes ────────────────────────────────────────────────────────────────────────────────────

test('D1: add / rename / duplicate / delete -- rename keeps the steps, delete never takes "skills" or the last route, undo brings each back', () => {
    const skills = route([{ do: 'wait', seconds: 1 }]);
    let h = historyOf({ routes: [skills], at: 0 });
    h = commit(h, addRoute(h.present, 'left'));
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills', 'left']);
    assert.equal(h.present.at, 1, 'the new route is on the field');
    assert.equal(h.present.routes[1].mirroredFor, undefined, 'a new route runs as written on robot two until the mirror is turned on');
    h = commit(h, renameRouteAt(h.present, 0 + 1, 'left side'));
    h = commit(h, duplicateRoute(h.present, 0, 'skills copy'));
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills', 'skills copy', 'left side']);
    assert.deepEqual(h.present.routes[1].steps, skills.steps, 'the copy carries the steps');
    assert.equal(h.present.at, 1);
    const renamed = renameRouteAt(h.present, 1, 'practice');
    assert.deepEqual(renamed.routes[1].steps, skills.steps, 'a rename keeps the steps');
    assert.equal(deleteRoute(h.present, 0), h.present, '"skills" is never deleted');
    assert.equal(deleteRoute({ routes: [skills], at: 0 }, 0).routes.length, 1, 'nor the last route');
    h = commit(h, deleteRoute(h.present, 2));
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills', 'skills copy']);
    h = undo(h);
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills', 'skills copy', 'left side'], 'undo brings the deleted route back');
    h = undo(undo(undo(h)));
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills']);
    h = redo(h);
    assert.deepEqual(h.present.routes.map((r) => r.name), ['skills', 'left']);
    // deleting the route BEFORE the one on the field keeps the same route on the field (4 routes: the shift is seen)
    const set = { routes: [skills, { ...newRoute('a') }, { ...newRoute('b') }, { ...newRoute('c') }], at: 2 };
    assert.equal(deleteRoute(set, 1).routes[deleteRoute(set, 1).at].name, 'b');
    assert.equal(deleteRoute({ ...set, at: 1 }, 1).at, 1, 'the deleted route\'s place: the one after it');
    assert.equal(deleteRoute({ ...set, at: 3 }, 3).at, 2, 'the last deleted: the one before it');
});

test('D1: freeName never repeats a name -- a route\'s or another auton\'s -- capitals aside, inside the C++\'s mirrored budget', () => {
    assert.equal(X.mirroredAutonNameMaxChars, 25);
    const routes = [newRoute(), newRoute('Route 2'), newRoute('route 3')];
    assert.equal(freeName(routes, X), 'route 4');
    assert.equal(freeName(routes, X, 'left'), 'left');
    assert.equal(freeName([...routes, newRoute('LEFT')], X, 'left'), 'left 2');
    assert.equal(freeName([], X, '40IN forward'), '40IN forward 2', 'the 40 in\'s name is taken');
    const long = 'x'.repeat(30);
    assert.equal(freeName([], X, long), long.slice(0, 25), 'never past 25');
    assert.equal(freeName([newRoute(long.slice(0, 25))], X, long), `${'x'.repeat(23)} 2`);
    assert.equal(freeName([newRoute(`${'x'.repeat(23)} 2`), newRoute(long.slice(0, 25))], X, long), `${'x'.repeat(23)} 3`);
    // the budget is the description's, never a number typed here
    const small = { ...X, mirroredAutonNameMaxChars: 12 };
    assert.equal(freeName([], small, long), 'x'.repeat(12));
    assert.equal(freeName([newRoute('x'.repeat(12))], small, long), `${'x'.repeat(10)} 2`);
});

test('D1: a route may not take another auton\'s name (the 40 in, the acceptance run), capitals aside -- the brain\'s menu would drop it', () => {
    assert.deepEqual(X.otherAutons, ['40in forward', 'TEST 24in turn back'], 'baked from the C++ (route_maker_describe.hpp)');
    for (const n of ['40in forward', '40IN FORWARD', 'TEST 24in turn back', 'test 24IN turn back']) {
        const rc21 = at(checkRoute(route([], START, { name: n, mirroredFor: undefined }), [X], FIELD, { routes: [newRoute(), newRoute(n)], index: 1 }), 'RC21');
        assert.equal(rc21.length, 1, n);
        assert.match(rc21[0].words, /is another auton on the brain's menu/, n);
        assert.equal(rc21[0].library, 'the name is already registered');
    }
    for (const n of ['40in forward 2', '40in', 'TEST 24in turn']) {
        assert.deepEqual(at(checkRoute(route([], START, { name: n, mirroredFor: undefined }), [X], FIELD, { routes: [newRoute(), newRoute(n)], index: 1 }), 'RC21'), [], n);
    }
    assert.equal(routeCap(X), X.maxAutons - 2, 'the menu less the other autons');
    assert.equal(routeCap({ ...X, otherAutons: ['a', 'b', 'c'] }), X.maxAutons - 3);
});

test('D2: robot two always runs the skills route MIRRORED (ROB1-BT) -- an unmirrored skills route is refused, its fix one click; any other route may be either', () => {
    const plain = route([], START, { mirroredFor: undefined, together: undefined });
    const rcm = at(checkRoute(plain, [X], FIELD), 'RCM1');
    assert.equal(rcm.length, 1);
    assert.match(rcm[0].words, /^Black Bot runs the skills route MIRRORED, always/);
    assert.equal(rcm[0].fix.words, 'Mirror it for Black Bot');
    const fixed = applyFix(plain, rcm[0].fix);
    assert.equal(fixed.mirroredFor, 'two');
    assert.deepEqual(at(checkRoute(fixed, [X], FIELD), 'RCM1'), []);
    assert.deepEqual(at(checkRoute({ ...plain, name: 'left' }, [X], FIELD, { routes: [newRoute(), { ...plain, name: 'left' }], index: 1 }), 'RCM1'), []);
    assert.ok(['RCT1', 'RCP1', 'RCM1', 'RCO3'].every((c) => ROBOT_TWO_ONLY.has(c)), 'PREVIEW (robot one\'s run) never waits for these');
    assert.equal(ROBOT_TWO_ONLY.size, 4);
});

test('D1: an OLD project (one route "skills", A1\'s file) opens unchanged and saves to the same bytes', () => {
    // RMP1-H: along y = -60, clear of the black goal by the red wall (along y = -55.7 the square clips it by 1.7 in)
    const old = saveProject({ ...newProject(), routes: [route([{ do: 'driveTo', x: -12, y: -60, faceFirst: false }], { ...START, y: -60 }, {})] });
    assert.ok(!old.includes('together'), 'A1 never wrote "together"');
    const back = loadProject(old);
    assert.ok(back.ok);
    assert.equal(back.project.routes.length, 1);
    assert.equal(saveProject(back.project), old);
    // ...and it asks the question (D3) instead of guessing: no silent default
    assert.deepEqual(codes(checkRoute(back.project.routes[0], [X], FIELD)), ['RCT1']);
});

// ── D2: the export of N routes ───────────────────────────────────────────────────────────────────────────────

test('D2: every route in the ONE region, in order; each its own C++ names; the list says mirror and together per route', () => {
    assert.deepEqual(routeIdents([newRoute(), newRoute('left side 2'), newRoute('40in test'), newRoute('LEFT side 2'), newRoute('!!'), newRoute('Skills')]),
                     ['skills', 'leftSide2', 'route40inTest', 'leftSide22', 'route5', 'skills2']);
    const routes = [{ ...newRoute(), together: true }, { ...newRoute('alone'), mirroredFor: 'two', together: false },
                    { ...newRoute('as written'), mirroredFor: undefined }, { ...newRoute('unasked'), mirroredFor: 'two' }];
    const text = exportRegion(routes, X);
    assert.equal(text.split('ROUTE MAKER').length - 1, 2, 'one BEGIN, one END');
    assert.ok(text.startsWith('// ═══ ROUTE MAKER — route "skills" — BEGIN'), 'the markers name the region "skills" whatever the routes');
    const list = text.slice(text.indexOf('const TeamRoute kTeamRoutes[] = {')).split('\n').slice(1, 5);
    assert.deepEqual(list.map((l) => l.replace(/\s+\/\/.*$/, '').trim()), [
        '{&skillsRoute, Mirror::ForRobotTwo, Together::Yes},', '{&aloneRoute, Mirror::ForRobotTwo, Together::No},',
        '{&asWrittenRoute, Mirror::None, Together::No},', '{&unaskedRoute, Mirror::ForRobotTwo, Together::NotAnswered},']);
    for (const r of routes) {
        assert.equal(text.split(`e.name = "${r.name}";`).length - 1, 1, r.name);
    }
    assert.equal(exportRegion(routes, X), exportRegion(JSON.parse(JSON.stringify(routes)), X), 'the same routes, the same bytes');
    assert.equal(exportRegion(routes[0]), exportRegion([routes[0]]), 'one route is a list of one');
});

// ── D3, D5: together, and too close ───────────────────────────────────────────────────────────────────────────

const ONE = { ...X };  // robot x carries the C++'s pair numbers and mirror (route_maker_describe.hpp bakes them)
const LEAD = { x: -36, y: 54.076, headingDeg: -90 };  // the team lead's first real start (the brief §1)

test('RMP1-L: the first words a student meets name the right robot, exactly (RC3, RCT1, a mirrored name too long)', () => {
    assert.deepEqual(at(checkRoute(addStep(newRoute(), { do: 'wait', seconds: 1 }), [ONE], FIELD), 'RC3').map((i) => i.words),
        ['Place White Bot\'s start on the field first: every step is planned from it.']);
    assert.deepEqual(at(checkRoute(route([], START, { together: undefined }), [ONE], FIELD), 'RCT1').map((i) => i.words),
        ['Black Bot runs this route mirrored. Will both robots run it at the SAME time? Answer it, so the too-close check knows.']);
    assert.equal(nameProblem('M'.repeat(26), true, ONE).words, 'Black Bot shows this route as "MMMMMMMMMMMMMMMMMMMMMMMMMM SKILLS MIR" '
        + 'on one screen line, so a mirrored route\'s name is at most 25 characters.');
    assert.equal(nameProblem('M'.repeat(25), true, ONE), null, '25 fits');
});

test('D3: a mirrored route asks, ONCE, both ways one click; a route not mirrored never asks', () => {
    const asked = at(checkRoute(route([], START, { together: undefined }), [ONE], FIELD), 'RCT1');
    assert.equal(asked.length, 1);
    assert.match(asked[0].words, /Will both robots run it at the SAME time\?/);
    assert.equal(applyFix(route([], START, { together: undefined }), asked[0].fix).together, true);
    assert.equal(applyFix(route([], START, { together: undefined }), asked[0].alsoFix).together, false);
    assert.deepEqual(at(checkRoute(route([], START, { mirroredFor: undefined, together: undefined }), [ONE], FIELD), 'RCT1'), []);
    assert.deepEqual(at(checkRoute({ ...newRoute(), start: null }, [ONE], FIELD), 'RCT1'), [], 'no start yet: asked when it is placed');
});

test('D5: the team lead\'s real start is TOO CLOSE together (4.35 in, at the start) and clear 3 in further out; "one at a time": no check', () => {
    assert.deepEqual(ONE.pairCheck, { gapIn: 6, squareIn: 15 });
    const two = reflect(LEAD, ONE.skillsMirror);
    assert.deepEqual([two.x, two.y, two.headingDeg], [-54.076, 36, 0], 'robot two\'s start: across y = -x, facing 0 (robot one faces -90)');
    const close = at(checkRoute(route([], LEAD, { together: true }), [ONE], FIELD), 'RCP1');
    assert.equal(close.length, 1);
    assert.equal(close[0].stepIndex, -1, 'at the START');
    assert.ok(Math.abs(close[0].pair.gap - 3.076 * Math.SQRT2) < 0.01, `${close[0].pair.gap}`);
    assert.match(close[0].words, /At the START White Bot at \(-36, 54\.076\) and Black Bot \(mirrored\) at \(-54\.076, 36\) are 4\.4 in apart: they must stay 6 in apart\. Move White Bot's start away from the diagonal\./);
    assert.deepEqual(at(checkRoute(route([], { ...LEAD, y: 57.076 }, { together: true }), [ONE], FIELD), 'RCP1'), [], '3 in further out: clear');
    assert.deepEqual(at(checkRoute(route([], LEAD, { together: false }), [ONE], FIELD), 'RCP1'), [], 'one at a time: never checked');
    // its one-click fix moves the start straight off the diagonal (along its normal: dx = dy) by the LEAST that clears it
    const fixed = applyFix(route([], LEAD, { together: true }), close[0].fix);
    assert.deepEqual(at(checkRoute(fixed, [ONE], FIELD), 'RCP1'), [], 'fixed');
    const dx = fixed.start.x - LEAD.x;
    const dy = fixed.start.y - LEAD.y;
    assert.ok(dx > 0 && Math.abs(dx - dy) < 0.002, `${dx}, ${dy}`);
    const gapAt = (k) => squareGap({ ...LEAD, x: LEAD.x + dx * k, y: LEAD.y + dy * k }, reflect({ ...LEAD, x: LEAD.x + dx * k, y: LEAD.y + dy * k }, ONE.skillsMirror), 15);
    assert.ok(gapAt(1) >= 6 && gapAt(0.99) < 6, `the least move: ${gapAt(1)} vs ${gapAt(0.99)}`);
    assert.equal(fixed.start.headingDeg, LEAD.headingDeg, 'the heading kept');
    assert.equal(applyFix(route([], LEAD, { together: true }), close[0].alsoFix).together, false, '"they don\'t run at the same time"');
    // the mirror image of the team lead's start: the OTHER side of the line -- moved away on ITS side, the least again
    const other = { x: -54.076, y: 36, headingDeg: 0 };
    const o = at(checkRoute(route([], other, { together: true }), [ONE], FIELD), 'RCP1');
    const oFixed = applyFix(route([], other, { together: true }), o[0].fix);
    assert.deepEqual(at(checkRoute(oFixed, [ONE], FIELD), 'RCP1'), []);
    assert.ok(Math.hypot(oFixed.start.x - other.x, oFixed.start.y - other.y) < 1.5, JSON.stringify(oFixed.start));
    assert.ok(oFixed.start.x + oFixed.start.y < other.x + other.y, 'away from the line, on its own side');
});

test('D5: a start FLUSH on a wall near the diagonal -- the one-click fix slides it along the wall, never into it (the least such move)', () => {
    const flush = FIELD.wallInnerFaceIn - 7.5;  // the 15 in square's side on the blue wall's face
    for (const [x, h] of [[-50, -90], [-52, 0], [-58, 0]]) {
        const s0 = { x, y: flush, headingDeg: h };
        const close = at(checkRoute(route([], s0, { together: true }), [ONE], FIELD), 'RCP1');
        assert.equal(close.length, 1, `${x}`);
        assert.match(close[0].fix.words, /^Slide the start \d+\.\d in along the blue wall, away from the diagonal$/, `${x}`);
        const fixed = applyFix(route([], s0, { together: true }), close[0].fix);
        const left = checkRoute(fixed, [ONE], FIELD).filter((i) => i.level === 'error');
        assert.deepEqual(left.map((i) => i.code), [], `${x}: ${left.map((i) => i.words)}`);
        assert.ok(wallOvershoot(fixed.start.x, fixed.start.y, h, FIELD, 15) <= 1e-9, `${x}: not into the wall`);
        assert.ok(fixed.start.x > x, `${x}: along the wall, away from the line`);
        // the least such move: no direction (every 0.1 deg) clears the gap inside the walls by less
        const gapOf = (q) => squareGap(q, reflect(q, ONE.skillsMirror), 15);
        let best = Infinity;
        for (let k = 0; k < 3600; ++k) {
            const d = { x: Math.cos(k * Math.PI / 1800), y: Math.sin(k * Math.PI / 1800) };
            const at = (t) => ({ ...s0, x: x + d.x * t, y: flush + d.y * t });
            if (gapOf(at(30)) < 6) {
                continue;
            }
            let lo = 0;
            let hi = 30;
            for (let i = 0; i < 40; ++i) {
                [lo, hi] = gapOf(at((lo + hi) / 2)) >= 6 ? [lo, (lo + hi) / 2] : [(lo + hi) / 2, hi];
            }
            if (wallOvershoot(at(hi).x, at(hi).y, h, FIELD, 15) <= 1e-9) {
                best = Math.min(best, hi);
            }
        }
        const moved = Math.hypot(fixed.start.x - x, fixed.start.y - flush);
        assert.ok(moved <= best + 0.01, `${x}: moved ${moved}, the least is ${best}`);
    }
});

test('D4: robot two\'s pose is robot one\'s reflected -- a heading the mirror CHANGES (0 -> -90, 30 -> -120), not only one it keeps', () => {
    // across y = -x: (x, y) -> (-y, -x), a heading h -> -90 - h (by hand, not by the page's formula)
    for (const [h, want] of [[0, -90], [30, -120], [90, 180], [-135, 45], [-90, 0]]) {
        const two = reflect({ x: -36, y: -55.7, headingDeg: h }, ONE.skillsMirror);
        assert.deepEqual([two.x, two.y], [55.7, 36]);
        assert.ok(Math.abs(((two.headingDeg - want + 540) % 360) - 180) < 1e-9, `${h} -> ${two.headingDeg}, not ${want}`);
    }
});

test('D5: MID-PATH -- a clear start that drives toward the diagonal is caught at that step, robot two where its mirror is', () => {
    const r = route([{ do: 'driveTo', x: -36, y: -24, faceFirst: true }, { do: 'driveTo', x: -10, y: 10, faceFirst: true }], START, { together: true });
    const hit = at(checkRoute(r, [ONE], FIELD), 'RCP1');
    assert.equal(hit.length, 1);
    assert.equal(hit[0].stepIndex, 1, 'step 2 crosses toward the line');
    assert.match(hit[0].words, /During step 2 White Bot at .* and Black Bot \(mirrored\) at .* Keep White Bot's whole robot 3 in off/);
    const p = hit[0].pair;
    assert.ok(p.gap < 6);
    const two = reflect(p.one, ONE.skillsMirror);
    assert.ok(Math.abs(two.x - p.two.x) < 1e-9 && Math.abs(two.y - p.two.y) < 1e-9, 'robot two is robot one reflected at the same moment');
    // the same route far from the diagonal: never
    const far = route([{ do: 'driveTo', x: 12, y: -55.7, faceFirst: false }], START, { together: true });
    assert.equal(tooClose(far, plannedPoses(far), ONE.skillsMirror, ONE.pairCheck), null);
});

test('D5: the path BETWEEN two clear ends is checked -- a straight drive and a curve across the diagonal, a turn in place and a face-first turn whose corners sweep too near', () => {
    const gapOf = (q) => squareGap(q, reflect(q, ONE.skillsMirror), 15);
    const cross = { x: -40, y: -10, headingDeg: 45 };  // 35.4 in off the line; the drive ends 35.4 in off its far side
    const clear = { x: 8.485, y: 8.485, headingDeg: 45 };  // 12 in off the line, its sides along it: 9 in apart; at 0 deg only 2.8
    for (const [what, start, step] of [
        ['a straight drive across', cross, { do: 'driveTo', x: 10, y: 40, faceFirst: false }],
        ['a curve across', cross, { do: 'curveTo', points: [{ x: 10, y: 40, headingDeg: 45 }] }],
        ['a turn in place, 45 -> 135 deg', clear, { do: 'turnTo', headingDeg: 135 }],
        ['a face-first turn, 45 -> -45 deg, then a drive along the line', clear, { do: 'driveTo', x: 22.627, y: -5.657, faceFirst: true }],
    ]) {
        const r = route([step], start, { together: true });
        const plan = plannedPoses(r);
        assert.ok(gapOf(plan[0].from) >= 6 && gapOf(plan[0].to) >= 6, `${what}: both ends clear (${gapOf(plan[0].from)}, ${gapOf(plan[0].to)})`);
        const hit = at(checkRoute(r, [ONE], FIELD), 'RCP1');
        assert.equal(hit.length, 1, what);
        assert.equal(hit[0].stepIndex, 0, what);
        assert.ok(hit[0].pair.gap < 6, what);
    }
    // the drive along the line alone (already facing it): never too close
    const along = route([{ do: 'driveTo', x: 22.627, y: -5.657, faceFirst: true }], { ...clear, headingDeg: -45 }, { together: true });
    assert.deepEqual(at(checkRoute(along, [ONE], FIELD), 'RCP1'), []);
});

test('D5: GL23 (the page and the suite agree) -- the page\'s plan of the golden keeps the robots 9.0 in apart at the least; the C++ case holds the host test plant within 1 in of it', () => {
    const p = loadProject(fs.readFileSync(path.join(GOLDENS, 'gl23_pair_near_the_diagonal.route.json'), 'utf8')).project;
    const r = p.routes[0];
    assert.deepEqual([r.mirroredFor, r.together], ['two', true]);
    assert.deepEqual(at(checkRoute(r, [X], FIELD, { routes: p.routes, index: 0 }), 'RCP1'), []);
    const plan = plannedPoses(r);
    const least = Math.min(...r.steps.flatMap((s, i) => [plan[i].from, ...stepPoses(s, plan[i])]).map((q) => squareGap(q, reflect(q, X.skillsMirror), 15)));
    assert.ok(Math.abs(least - 9.0) < 0.01, `${least}`);
});

// ── D7: a lift's plain steps ──────────────────────────────────────────────────────────────────────────────────

const LIFT = X.mechanisms.find((m) => m.ident === 'lift');  // switch-homed, 0 to 24 in, home at 0
const ARM = X.mechanisms.find((m) => m.ident === 'arm');    // rests at its stop (0 deg) at power-up, 0 to 160

test('D7: the height tracker -- home, up, down, to, bottom; unknown until a home (the arm: known at power-up)', () => {
    const steps = [{ do: 'move', mechanism: 'lift', to: 10, boundS: 2 }, { do: 'home', mechanism: 'lift', boundS: 3 },
                   { do: 'move', mechanism: 'lift', up: 6, boundS: 2 }, { do: 'startMove', mechanism: 'lift', up: 4.5, boundS: 2 },
                   { do: 'waitUntilDone', mechanism: 'lift', boundS: 1 }, { do: 'move', mechanism: 'lift', down: 2.5, boundS: 2 },
                   { do: 'move', mechanism: 'lift', preset: 'high', boundS: 2 }, { do: 'move', mechanism: 'lift', bottom: true, boundS: 2 },
                   { do: 'move', mechanism: 'arm', up: 30, boundS: 2 }];
    const h = plannedHeights({ steps: steps.map((s) => normalizeRoute({ steps: [s] }).steps[0]) }, X);
    assert.deepEqual(h.map((x) => x.after.lift), [10, 0, 6, 10.5, 10.5, 8, 22, 0, 0]);
    assert.deepEqual(h.map((x) => x.after.arm), [0, 0, 0, 0, 0, 0, 0, 0, 30]);
    assert.equal(plannedHeights({ steps: [] }, X).length, 0);
    assert.equal(plannedHeights({ steps: [{ do: 'move', mechanism: 'lift', up: 6 }] }, X)[0].after.lift, null, 'up from nowhere: unknown');
    assert.equal(bottomOf({ range: [0.0004, 24] }), 0.001, 'the bottom on the stored grid is never below the travel');
    assert.equal(moveTarget({ down: 5 }, LIFT, null), null);
    // a home STARTED while the route goes on sets the height too: up 6 after it is known, and exported as a height
    const started = route([{ do: 'startHome', mechanism: 'lift', boundS: 3 }, { do: 'waitUntilDone', mechanism: 'lift', boundS: 3 },
                           { do: 'move', mechanism: 'lift', up: 6, boundS: 2 }]);
    assert.deepEqual(plannedHeights(started, X).map((x) => x.after.lift), [0, 0, 6]);
    assert.deepEqual(checkRoute(started, [X], FIELD).filter((i) => i.level === 'error'), []);
    assert.match(exportRegion(started, X), /r\.move\(lift\.to\(6_in\), /);
    // a level the facts still OWE has no height (never its placeholder): "up" after it is from an unknown height
    const owed = ledgerRobot();
    const c = owed.mechanisms[0];
    owed.mechanisms[0] = { ...c, homeAt: 0, presets: c.presets.map((q, k) => (k === 1 ? { ...q, owed: true } : q)) };
    const fromOwed = route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, { do: 'move', mechanism: 'cascade', preset: 1, boundS: 2 },
                            { do: 'move', mechanism: 'cascade', up: 6, boundS: 2 }]);
    assert.deepEqual(plannedHeights(fromOwed, owed).map((x) => x.after.cascade), [0, null, null]);
    const rcl1 = at(checkRoute(fromOwed, [owed], FIELD), 'RCL1');
    assert.ok(rcl1.some((i) => i.stepIndex === 2 && /from where\?/.test(i.words)), JSON.stringify(rcl1.map((i) => i.words)));
});

test('D7: a relative step from an UNKNOWN height is refused with "Home it first"; one past the travel with its one-click fix', () => {
    const unknown = checkRoute(route([{ do: 'move', mechanism: 'lift', up: 6, boundS: 2 }]), [X], FIELD);
    const rcl1 = at(unknown, 'RCL1');
    assert.equal(rcl1.length, 0, 'RC25 says it (home first) -- never twice');
    assert.equal(at(unknown, 'RC25').length, 1);
    const homeless = { ...X, mechanisms: X.mechanisms.map((m) => (m.ident === 'lift' ? { ...m, homing: 'none', homeAt: undefined } : m)) };
    const none = at(checkRoute(route([{ do: 'move', mechanism: 'lift', down: 1, boundS: 2 }]), [homeless], FIELD), 'RCL1');
    assert.equal(none.length, 1);
    assert.match(none[0].words, /Down 1 in from where\? .*Pick a height instead: click its bar\./);
    for (const [step, words, fixed] of [
        [{ do: 'move', mechanism: 'lift', up: 30, boundS: 2 }, /to 30 in, past the top of its travel \(0 to 24 in\)/, { up: 24 }],
        [{ do: 'move', mechanism: 'lift', down: 0.5, boundS: 2 }, /to -0\.5 in, below the bottom/, { down: undefined, bottom: undefined }],
        [{ do: 'move', mechanism: 'lift', to: 24.001, boundS: 2 }, /past the top/, { to: 24 }],
        [{ do: 'move', mechanism: 'lift', to: -1, boundS: 2 }, /below the bottom/, { bottom: true }],
    ]) {
        const r = route([{ do: 'home', mechanism: 'lift', boundS: 3 }, step]);
        const rcl2 = at(checkRoute(r, [X], FIELD), 'RCL2');
        assert.equal(rcl2.length, 1, JSON.stringify(step));
        assert.match(rcl2[0].words, words);
        if (rcl2[0].fix) {
            const after = applyFix(r, rcl2[0].fix, addStep, updateStep);
            assert.deepEqual(at(checkRoute(after, [X], FIELD), 'RCL2'), [], `${JSON.stringify(step)}: fixed`);
            for (const [k, v] of Object.entries(fixed)) {
                assert.equal(after.steps[1][k], v, `${JSON.stringify(step)}: ${k}`);
            }
        }
    }
    // "down" past the bottom from a height above it: the fix goes down only to the bottom
    const down = route([{ do: 'home', mechanism: 'lift', boundS: 3 }, { do: 'move', mechanism: 'lift', up: 6, boundS: 2 },
                        { do: 'move', mechanism: 'lift', down: 10, boundS: 2 }]);
    const d2 = at(checkRoute(down, [X], FIELD), 'RCL2');
    assert.equal(d2.length, 1);
    assert.equal(d2[0].fix.words, 'Go down only 6 in (to the bottom)');
    const dFixed = applyFix(down, d2[0].fix, addStep, updateStep);
    assert.equal(dFixed.steps[2].down, 6);
    assert.deepEqual(at(checkRoute(dFixed, [X], FIELD), 'RCL2'), []);
    // a move of 0 is refused, either way
    for (const k of ['up', 'down']) {
        const zero = at(checkRoute(route([{ do: 'home', mechanism: 'lift', boundS: 3 }, { do: 'move', mechanism: 'lift', [k]: 0, boundS: 2 }]), [X], FIELD), 'RCL1');
        assert.equal(zero.length, 1, k);
        assert.match(zero[0].words, /above 0/);
    }
    // the edges pass: up to exactly 24, down to exactly 0, to 0
    for (const step of [{ up: 24 }, { to: 0 }, { to: 24 }, { bottom: true }]) {
        const r = route([{ do: 'home', mechanism: 'lift', boundS: 3 }, { do: 'move', mechanism: 'lift', boundS: 2, ...step }]);
        assert.deepEqual(at(checkRoute(r, [X], FIELD), 'RCL2'), [], JSON.stringify(step));
    }
    const twice = at(checkRoute(route([{ do: 'home', mechanism: 'lift', boundS: 3 }, { do: 'move', mechanism: 'lift', up: 2, to: 4, boundS: 2 }]), [X], FIELD), 'RC18');
    assert.match(twice[0].words, /more than once/);
});

test('D7, D8: the mechanism menu -- a lift: up / down / to / the bottom / home / wait (a level only when it has one); a pneumatic: one button per position', () => {
    const words = (m, levels) => mechanismActions(m, levels).map((a) => a.words);
    assert.deepEqual(words(LIFT, false), ['Go up _ in', 'Go down _ in', 'Move to _ in', 'Back to the bottom',
                                          'Find the bottom (do this first)', 'Wait until it is done']);
    assert.ok(words(LIFT, true).includes('Move to a level'));
    assert.deepEqual(words(ARM, true).slice(0, 3), ['Turn up _ deg', 'Turn down _ deg', 'Move to _ deg']);
    assert.ok(!words(ARM, true).includes('Back to the bottom'));
    assert.deepEqual(mechanismActions(LIFT, false)[0].step, { mechanism: 'lift', do: 'move', up: null }, 'the number left EMPTY: never invented');
    const claw = { ident: 'claw', name: 'claw', kind: 'pneumatic', positions: ['open', 'closed'] };
    assert.deepEqual(words(claw), ['Open claw', 'Close claw']);
    assert.deepEqual(mechanismActions(claw).map((a) => a.step), [{ mechanism: 'claw', do: 'set', position: 'open' }, { mechanism: 'claw', do: 'set', position: 'closed' }]);
    assert.equal(positionWords({ name: 'ramp' }, 'down'), 'Ramp down');
    assert.equal(positionWords({ name: 'front toggle' }, 'out'), 'Front toggle out');
    // the ledger robot's cascade (robot one's shape): the words come from ITS declaration
    assert.equal(mechanismActions(ledgerRobot().mechanisms[0], true)[0].words, 'Go up _ in');
});

// ── D9: small moves ──────────────────────────────────────────────────────────────────────────────────────────

test('D9: drive forward / back up N in -- from the PLANNED pose (after a turn, after a curve), refused where it is unknown or too short', () => {
    const r = route([{ do: 'forward', inches: 6 }, { do: 'turnTo', headingDeg: 90 }, { do: 'backward', inches: 2.5 },
                     { do: 'curveTo', points: [{ x: -20, y: -40, headingDeg: 0 }] }, { do: 'forward', inches: 1 }]);
    const plan = plannedPoses(r);
    assert.deepEqual([plan[0].point.x, plan[0].point.y], [-30, -55.7]);
    assert.deepEqual([plan[2].point.x, plan[2].point.y], [-30, -58.2], 'back up 2.5 facing 90: y goes DOWN');
    assert.equal(plan[2].to.headingDeg, 90, 'backing up keeps the heading');
    const end = plan[3].to;
    assert.ok(Math.abs(plan[4].point.x - (end.x + Math.cos(end.headingDeg * Math.PI / 180))) < 1e-3, 'from the curve\'s end, along its heading');
    assert.deepEqual(codes(checkRoute(r, [X], FIELD)).filter((c) => c === 'RC1'), []);
    const short = at(checkRoute(route([{ do: 'forward', inches: 0.999 }]), [X], FIELD), 'RC1');
    assert.match(short[0].words, /too short/);
    assert.deepEqual(at(checkRoute(route([{ do: 'forward', inches: 1 }]), [X], FIELD), 'RC1'), [], '1 in: the edge passes');
    assert.match(at(checkRoute(route([{ do: 'backward' }]), [X], FIELD), 'RC1')[0].words, /Type how far it backs up/);
    assert.match(at(checkRoute(route([{ do: 'forward', inches: 30 }], { x: 50, y: 0, headingDeg: 0 }), [X], FIELD), 'RC1')[0].words, /into/);
    const after = route([{ do: 'driveTo', x: Number.NaN, y: 0 }, { do: 'forward', inches: 3 }]);
    assert.ok(at(checkRoute(after, [X], FIELD), 'RC1').some((i) => i.stepIndex === 1 && /not known/.test(i.words)), 'after a step with an error');
    assert.match(exportRegion(r, X), /Step 1 — DRIVE FORWARD 6 in, straight on, to \(-30, -55\.7\) in\.\n    r\.driveTo\(-30_in, -55\.7_in\);/);
    assert.match(exportRegion(r, X), /r\.reverseTo\(-30_in, -58\.2_in\);/);
});
