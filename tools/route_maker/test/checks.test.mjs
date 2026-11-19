// checks.test.mjs — the page refuses what the library refuses (RMP1 brief §2.2's RC cases for RMP1-A1:
// RC1–RC3, RC15–RC21, RC24–RC26). Adversarial: every case is a route built to slip past ONE rule, and each
// rule is also pinned at its EDGE (the value just inside passes). The C++ twins that prove the library does
// refuse are the goldens the suite runs (GL1b for RC24, GL18 for RC25, CV1 for RC26).
import test from 'node:test';
import assert from 'node:assert/strict';
import { checkRoute, applyFix, wallOvershoot } from '../src/model/checks.mjs';
import { newRoute, setStart, addStep, updateStep, duplicateStep, plannedPoses } from '../src/model/route.mjs';
import { libraryWords } from '../src/model/library_words.mjs';
import { startTitle } from '../src/ui/words.mjs';
import { fixtureRobot, ledgerRobot } from './helpers.mjs';

const X = fixtureRobot('x');
const Y = fixtureRobot('y');
const FIELD = X.field;
const START = { x: -36, y: -55.7, headingDeg: 0 };

function route(steps, start = START) {
    let r = setStart(newRoute(), start);
    for (const s of steps) {
        r = addStep(r, s);
    }
    return r;
}
const codes = (issues, level) => issues.filter((i) => !level || i.level === level).map((i) => i.code);
const at = (issues, code) => issues.filter((i) => i.code === code);

test('a clean route on robot x has no issue at all', () => {
    const r = route([
        { do: 'home', mechanism: 'lift', boundS: 3 },
        { do: 'driveTo', x: -12, y: -55.7, faceFirst: false },
        { do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 },
        { do: 'curveTo', points: [{ x: 0, y: -50 }, { x: 12, y: -36, headingDeg: 90 }] },
        { label: 'end', do: 'wait', seconds: 0.5 },
    ]);
    assert.deepEqual(checkRoute(r, [X], FIELD), []);
});

test('RC1: a point whose 15 in square is more than 1 in past a wall -- AT the 1 in edge: x = 63.704 allowed, 63.705 refused', () => {
    // the wall face is 70.2047 in, the half-width 7.5: 63.704 puts a corner 0.99928 in past it, 63.705 1.00028 in --
    // the two grid points either side of the 1 in slack (no grid point lands ON it), so a slack anywhere outside
    // [0.99928, 1.00028) changes one of them (0 in, 0.99 in, 1.09 in ...)
    const refused = (x) => at(checkRoute(route([{ do: 'driveTo', x, y: -55.7, faceFirst: false }]), [X], FIELD), 'RC1');
    assert.ok(refused(63.705).some((i) => i.level === 'error' && /right wall/.test(i.words)));
    assert.deepEqual(refused(63.704), []);
    assert.ok(wallOvershoot(63.704, 0, 0, FIELD, 15) < 1 && wallOvershoot(63.705, 0, 0, FIELD, 15) > 1);
    // the square's HALF-WIDTH counts (RM15's oracle): the centre at 63.8 is far inside the wall
    assert.ok(63.8 < FIELD.wallInnerFaceIn);
    assert.ok(refused(63.8).length === 1);
});

test('RC1 ON its edge: a corner EXACTLY 1 in past the wall is allowed ("more than 1 in" is THROUGH), 0.001 in more is not', () => {
    // a robot whose footprint puts the grid point x = 62's corner exactly 1 in past the right wall's face (the numbers
    // are the test's: 2 x (70.2047... + 1 - 62)) -- the only way to land ON the 1 in, since the wall face is not on the grid
    const size = 2 * (FIELD.wallInnerFaceIn + 1 - 62);
    const exact = { ...X, drive: { ...X.drive, footprintIn: size } };
    assert.equal(wallOvershoot(62, -55.7, 0, FIELD, size), 1, 'exactly 1, to the last bit');
    const rc1 = (x) => at(checkRoute(route([{ do: 'driveTo', x, y: -55.7, faceFirst: false }]), [exact], FIELD), 'RC1');
    assert.deepEqual(rc1(62), [], 'exactly 1 in: allowed');
    assert.equal(rc1(62.001).length, 1, '1.001 in: refused');
});

test('RC1 / RC24 on a robot\'s OWN numbers (never a 15 in or 15 deg literal): an 18 in robot steering within 8 deg', () => {
    const big = { ...X, drive: { ...X.drive, footprintIn: 18, maxSteerDeg: 8 } };
    const rc1 = (x) => at(checkRoute(route([{ do: 'driveTo', x, y: -55.7, faceFirst: false }]), [big], FIELD), 'RC1');
    // half-width 9: the edge moves 1.5 in in -- 62.204 allowed (0.99928 in past), 62.205 refused
    assert.deepEqual(rc1(62.204), []);
    assert.equal(rc1(62.205).length, 1);
    assert.match(rc1(62.205)[0].words, /18 in square/);
    assert.equal(at(checkRoute(route([{ do: 'driveTo', x: 62.205, y: -55.7, faceFirst: false }]), [X], FIELD), 'RC1').length, 0,
                 'the same point is fine for a 15 in robot');
    // a 10 deg leg: a warning on robot x (15 deg), an ERROR on this robot (8 deg), the library's sentence with ITS 8.0
    const ten = route([{ do: 'driveTo', x: -36 + 24 * Math.cos(Math.PI / 18), y: -55.7 + 24 * Math.sin(Math.PI / 18), faceFirst: false }]);
    assert.equal(at(checkRoute(ten, [X], FIELD), 'RC24')[0].level, 'warn');
    const i = at(checkRoute(ten, [big], FIELD), 'RC24')[0];
    assert.equal(i.level, 'error');
    assert.match(i.library, /\(steers within 8\.0 deg\)/);
    assert.match(i.words, /only within 8 deg/);
});

test('RC24 AT its edge: a leg exactly as far off its line as the robot steers is a warning, not an error (the library steers within it)', () => {
    const leg = route([{ do: 'driveTo', x: -12, y: -51.7, faceFirst: false }]);
    const off = plannedPoses(leg)[0].offDeg;  // the page's own number for this leg, to the last bit
    const exact = { ...X, drive: { ...X.drive, maxSteerDeg: off } };
    assert.equal(at(checkRoute(leg, [exact], FIELD), 'RC24')[0].level, 'warn', 'off == maxSteer: steered (the C++ refuses only ABOVE it)');
    const below = { ...X, drive: { ...X.drive, maxSteerDeg: off - 1e-9 } };
    assert.equal(at(checkRoute(leg, [below], FIELD), 'RC24')[0].level, 'error');
});

test('RC1: turning in place near a wall sweeps the corners (a square turned 45 deg reaches 10.6 in)', () => {
    // flush against the red wall (centre 7.6 in from it): facing 0 fits, the turn to 45 deg sweeps a corner through
    const start = { x: 0, y: -(FIELD.wallInnerFaceIn - 7.6), headingDeg: 0 };
    const issues = checkRoute(route([{ do: 'turnTo', headingDeg: 45 }], start), [X], FIELD);
    assert.ok(at(issues, 'RC1').some((i) => /Turning in place/.test(i.words)), JSON.stringify(issues));
    assert.deepEqual(at(checkRoute(route([{ do: 'turnTo', headingDeg: 0 }], start), [X], FIELD), 'RC1'), []);
    // a QUARTER turn ends square again -- both its ends fit, so only the sampled SWEEP (a corner ~3 in through the
    // wall at 45 deg) refuses it: a check of the two ends alone would pass it
    const quarter = checkRoute(route([{ do: 'turnTo', headingDeg: 90 }], start), [X], FIELD);
    assert.ok(at(quarter, 'RC1').some((i) => /Turning in place here, a corner of the robot goes 3\.\d in/.test(i.words)), JSON.stringify(quarter));
    assert.ok(wallOvershoot(start.x, start.y, 0, FIELD, 15) <= 1 && wallOvershoot(start.x, start.y, 90, FIELD, 15) <= 1);
    // and the same through a face (the turn of a turn-first drive is swept the same way)
    const face = checkRoute(route([{ do: 'face', x: 0, y: 0 }], start), [X], FIELD);
    assert.ok(at(face, 'RC1').some((i) => /Turning in place/.test(i.words)));
});

test('RC2: the SAMPLED library curve is checked, not its points -- a bend that bulges past a wall between two inside points', () => {
    // both points inside; the end heading forces the Hermite segment out past the right wall
    const r = route([{ do: 'curveTo', points: [{ x: 58, y: -40 }, { x: 58, y: 30, headingDeg: 180 }] }], { x: 30, y: -55.7, headingDeg: 0 });
    const issues = checkRoute(r, [X], FIELD);
    assert.ok(at(issues, 'RC2').length >= 1, JSON.stringify(issues));
    for (const p of r.steps[0].points) {
        assert.ok(wallOvershoot(p.x, p.y, 0, FIELD, 15) <= 1, 'each POINT is on the field: only the sampled path finds it');
    }
});

test('RC3: no start with steps; a start through a wall', () => {
    let r = addStep(newRoute(), { do: 'wait', seconds: 1 });
    assert.ok(codes(checkRoute(r, [X], FIELD)).includes('RC3'));
    r = setStart(r, { x: -36, y: -64, headingDeg: 0 });
    assert.ok(at(checkRoute(r, [X], FIELD), 'RC3').some((i) => /red wall/.test(i.words)));
    assert.deepEqual(checkRoute(newRoute(), [X], FIELD), [], 'an EMPTY route (no start, no steps) is fine: it exports §6.3');
});

test('RC3: a start MISSING a part (a hand-edited file\'s null) -- one RC3, nothing planned from it (never 0), "?" in the list', () => {
    for (const k of ['x', 'y', 'headingDeg']) {
        const start = { ...START };
        delete start[k];
        // a drive 0.5 in from (0, -55.7) and a face 0.5 in from (-36, 0): read as 0, the hole would make one of them
        // "no clear direction" (RC1) -- and every step would be planned from a robot that is nowhere
        const r = route([{ do: 'driveTo', x: 0.5, y: -55.7 }, { do: 'face', x: -36, y: 0.5 }], start);
        assert.equal(r.start[k], undefined, `${k}: the hole is kept (left OUT)`);
        const issues = checkRoute(r, [X], FIELD);
        assert.deepEqual(codes(issues, 'error'), ['RC3'], `${k}: the start's error, and nothing measured from the hole`);
        assert.match(at(issues, 'RC3')[0].words, /needs an x, a y and a heading/);
        assert.deepEqual(plannedPoses(r).map((e) => [e.from, e.to]), [[null, null], [null, null]], `${k}: no pose planned from it`);
    }
    assert.deepEqual(startTitle({ y: -36, headingDeg: 90 }), ['Start at (?, -36)', 'facing 90°']);
    assert.deepEqual(startTitle({ x: -36, y: -36 }), ['Start at (-36, -36)', 'facing ?°']);
    assert.deepEqual(startTitle({}), ['Start at (?, ?)', 'facing ?°']);
    assert.deepEqual(startTitle(null), ['Start: not placed yet', 'click the field to place robot one']);
    // and the whole start planned again the moment the part is typed
    const whole = route([{ do: 'driveTo', x: 0.5, y: -55.7 }], START);
    assert.deepEqual(plannedPoses(whole)[0].from, { ...START });
});

test('RC15: a skip lands only on a LATER label that exists', () => {
    const base = [{ do: 'wait', seconds: 1 }, { label: 'b', do: 'wait', seconds: 1 }];
    const missing = route([{ do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'nope' } } }, ...base]);
    assert.ok(at(checkRoute(missing, [X], FIELD), 'RC15').some((i) => /No step has the label/.test(i.words)));
    const back = route([...base, { do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'b' } } }]);
    assert.ok(at(checkRoute(back, [X], FIELD), 'RC15').some((i) => /FORWARD/.test(i.words)));
    const self = route([{ label: 'me', do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'me' } } }]);
    assert.ok(at(checkRoute(self, [X], FIELD), 'RC15').length === 1, 'a skip to its OWN step is not forward');
    const empty = route([{ do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: '' } } }]);
    assert.equal(at(checkRoute(empty, [X], FIELD), 'RC15')[0].library, libraryWords('skipEmpty'));
    const ok = route([{ do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'b' } } }, ...base]);
    assert.deepEqual(at(checkRoute(ok, [X], FIELD), 'RC15'), []);
});

test('RC16: two steps with one label; a label the C++ literal cannot carry', () => {
    const dup = route([{ label: 'a', do: 'wait', seconds: 1 }, { label: 'a', do: 'wait', seconds: 1 }]);
    const issues = at(checkRoute(dup, [X], FIELD), 'RC16');
    assert.equal(issues.length, 1);
    assert.equal(issues[0].stepIndex, 1, 'the SECOND one is marked');
    for (const bad of ['say "hi"', 'back\\slash', ' lead', 'x'.repeat(30), 'tab\there']) {
        assert.ok(at(checkRoute(route([{ label: bad, do: 'wait', seconds: 1 }]), [X], FIELD), 'RC16').length === 1, bad);
    }
    assert.deepEqual(at(checkRoute(route([{ label: 'x'.repeat(29), do: 'wait', seconds: 1 }]), [X], FIELD), 'RC16'), []);
});

test('RC17: a wait on a mechanism nothing started (the library THROWS), and one a skip can pass over', () => {
    const never = route([{ do: 'waitUntilDone', mechanism: 'intake', boundS: 1 }]);
    const i1 = at(checkRoute(never, [X], FIELD), 'RC17');
    assert.equal(i1.length, 1);
    assert.equal(i1[0].library, libraryWords('waitNeverStarted'));
    const skippable = route([
        { do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'w' } } },
        { do: 'startRun', mechanism: 'intake', volts: 12, forS: 1 },
        { label: 'w', do: 'waitUntilDone', mechanism: 'intake', boundS: 1 },
    ]);
    assert.ok(at(checkRoute(skippable, [X], FIELD), 'RC17').some((i) => /fails and skips to "w"/.test(i.words)),
              'a skip that jumps the start is a path without it (RM19)');
    const stopFirst = route([
        { do: 'driveTo', x: -12, y: -55.7 },
        { do: 'startRun', mechanism: 'intake', volts: 12, forS: 1 },
        { do: 'waitUntilDone', mechanism: 'intake', boundS: 1 },
    ]);
    assert.deepEqual(at(checkRoute(stopFirst, [X], FIELD), 'RC17'), [], 'a drive that STOPS the route on failure never reaches the wait');
    const blocking = route([{ do: 'run', mechanism: 'intake', volts: 12, forS: 1 }, { do: 'waitUntilDone', mechanism: 'intake', boundS: 1 }]);
    assert.deepEqual(at(checkRoute(blocking, [X], FIELD), 'RC17'), [], 'a blocking step began a step on it too');
});

test('RC18: a mechanism, a kind, a preset or a position the robot does not have -- on EVERY robot of the route', () => {
    const issues = checkRoute(route([
        { do: 'move', mechanism: 'wheelie', preset: 'up', boundS: 1 },
        { do: 'set', mechanism: 'lift', position: 'up' },
        { do: 'move', mechanism: 'arm', preset: 'nope', boundS: 1 },
        { do: 'set', mechanism: 'hood', position: 'sideways' },
    ]), [X], FIELD);
    const rc18 = at(issues, 'RC18');
    assert.deepEqual(rc18.map((i) => i.stepIndex), [0, 1, 2, 3]);
    assert.equal(rc18[2].library, libraryWords('noPreset'));
    assert.equal(rc18[3].library, libraryWords('noPosition'));
    // robot x has lift "low"; robot y has no "lift": the shared route breaks robot y's build
    const both = checkRoute(route([{ do: 'home', mechanism: 'lift', boundS: 3 }, { do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }]), [X, Y], FIELD);
    assert.ok(at(both, 'RC18').some((i) => i.robot === 'y'), JSON.stringify(both));
    // a ledger level vs a named preset are not interchangeable
    const one = ledgerRobot();
    const okLedger = checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 }]), [one], FIELD);
    assert.deepEqual(at(okLedger, 'RC18'), []);
    const named = checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, { do: 'move', mechanism: 'cascade', preset: 'L2', boundS: 2 }]), [one], FIELD);
    assert.equal(at(named, 'RC18').length, 1, 'a ledger level must be written by its index (facts::lv(i)), never its placeholder name');
    const past = checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, { do: 'move', mechanism: 'cascade', preset: 3, boundS: 2 }]), [one], FIELD);
    assert.equal(at(past, 'RC18').length, 1, 'level 3 is past this robot\'s levels');
});

test('RC18: an arm or lift with NO levels (robot one\'s cascade since ROB1-C2) -- a move says where levels come from, never "choose one"', () => {
    const lift = ledgerRobot().mechanisms[0];
    const none = { ...ledgerRobot(), mechanisms: [{ ...lift, presets: [] }] };
    const words = 'Robot one\'s cascade has no levels yet: record them at Bench Tests station 13.';
    for (const step of [{ do: 'move', mechanism: 'cascade', boundS: 2 }, { do: 'startMove', mechanism: 'cascade', preset: 1, boundS: 2 }]) {
        const rc18 = at(checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, step]), [none], FIELD), 'RC18');
        assert.deepEqual(rc18.map((x) => [x.level, x.stepIndex, x.field, x.words]), [['error', 1, 'preset', words]], JSON.stringify(step));
        assert.equal(rc18[0].library, step.preset === undefined ? '' : libraryWords('noPreset'), 'a level named anyway: the library refuses it too');
    }
    // its home and a wait on it are not refused; with ONE level the move is the old rule again (choose it)
    const home = route([{ do: 'startHome', mechanism: 'cascade', boundS: 3 }, { do: 'waitUntilDone', mechanism: 'cascade', boundS: 1 }]);
    assert.deepEqual(at(checkRoute(home, [none], FIELD), 'RC18'), []);
    const oneLevel = { ...none, mechanisms: [{ ...lift, presets: [lift.presets[0]] }] };
    const choose = at(checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, { do: 'move', mechanism: 'cascade', boundS: 2 }]), [oneLevel], FIELD), 'RC18');
    assert.deepEqual(choose.map((x) => x.words), ['Choose which level it moves to.']);
});

test('RC19: the motion options -- 0 or more (0 is the robot\'s own), a heading tolerance under 180, only on a step that drives', () => {
    const drive = (care) => at(checkRoute(route([{ do: 'driveTo', x: -12, y: -55.7, care }]), [X], FIELD), 'RC19');
    for (const f of ['timeoutS', 'maxSpeedInPerS', 'maxTurnDegPerS', 'toleranceIn', 'toleranceDeg']) {
        assert.deepEqual(drive({ [f]: -1 }).map((i) => i.field), [f], `${f} -1: the library throws (MotionOptions: ... >= 0)`);
        assert.deepEqual(drive({ [f]: 0 }), [], `${f} 0: the robot's own`);
        assert.deepEqual(drive({ [f]: 'fast' }).map((i) => i.field), [f], `${f} "fast" (a hand-edited file): not a number`);
    }
    // an Angle wraps: 190 deg would be -170, so the page refuses 180 and up; 179.999 is the last one allowed
    assert.deepEqual(drive({ toleranceDeg: 180 }).map((i) => i.field), ['toleranceDeg']);
    assert.deepEqual(drive({ toleranceDeg: 179.999 }), []);
    // a mechanism step takes no motion option (the export would not write it: refused, not silently dropped)
    const mech = at(checkRoute(route([{ do: 'home', mechanism: 'lift', boundS: 2, care: { timeoutS: 1 } }]), [X], FIELD), 'RC19');
    assert.deepEqual(mech.map((i) => `${i.field}: ${i.words}`), ['timeoutS: Only a step that drives or turns takes this.']);
    // ...but if-it-fails is every step's
    assert.deepEqual(at(checkRoute(route([{ do: 'home', mechanism: 'lift', boundS: 2, care: { onFail: 'carryOn' } }]), [X], FIELD), 'RC19'), []);
});

test('RC19: a time limit of 0 or none; a run longer than nothing; volts past 12', () => {
    const issues = checkRoute(route([
        { do: 'home', mechanism: 'lift', boundS: 0 },
        { do: 'home', mechanism: 'lift' },
        { do: 'run', mechanism: 'intake', volts: 12.5, forS: 0 },
        { do: 'waitUntilDone', mechanism: 'intake', boundS: -1 },
        { do: 'wait', seconds: 0 },
    ]), [X], FIELD);
    const rc19 = at(issues, 'RC19');
    assert.deepEqual(rc19.map((i) => `${i.stepIndex}:${i.field}`).sort(), ['0:boundS', '1:boundS', '2:forS', '2:volts', '3:boundS', '4:seconds'].sort());
    assert.equal(rc19.find((i) => i.stepIndex === 0).library, libraryWords('homeBound'));
    assert.deepEqual(at(checkRoute(route([{ do: 'run', mechanism: 'intake', volts: -12, forS: 0.001 }]), [X], FIELD), 'RC19'), []);
});

test('RC20: homing a mechanism declared never homed; RC25: moving it to a level', () => {
    const issues = checkRoute(route([{ do: 'home', mechanism: 'tilt', boundS: 2 }, { do: 'move', mechanism: 'tilt', preset: 'mid', boundS: 2 }]), [Y], FIELD);
    assert.equal(at(issues, 'RC20')[0].library, libraryWords('homeNever'));
    assert.ok(at(issues, 'RC25').some((i) => /never homed/.test(i.words)));
});

test('RC21: the route name (29 printable, no %, not NONE; 25 when mirrored; no " or \\)', () => {
    const named = (n, mirrored = true) => at(checkRoute({ ...newRoute(n), mirroredFor: mirrored ? 'two' : undefined }, [X], FIELD), 'RC21')
        .filter((i) => !i.contract);  // the library's rules (§6.3's name, below, is the contract's)
    assert.equal(named('NONE')[0].library, libraryWords('autonNone'));
    assert.equal(named('50%')[0].library, libraryWords('autonName'));
    assert.equal(named('').length, 1);
    assert.equal(named('x'.repeat(26)).length, 1, '26 is past the mirrored budget');
    assert.deepEqual(named('x'.repeat(25)), []);
    assert.deepEqual(named('x'.repeat(29), false), []);
    assert.equal(named('x'.repeat(30), false).length, 1);
    // a quote or a backslash would end the C++ literal and the marker line's name early: refused
    for (const n of ['a"b', 'a\\b']) {
        assert.deepEqual(named(n).map((i) => i.words), ['A route name cannot contain " or \\.'], n);
    }
});

test('RC21 / §6.3: the route is "skills" -- the name both robots\' code registers and the markers carry; any other is refused, with the fix', () => {
    assert.deepEqual(at(checkRoute(newRoute(), [X], FIELD), 'RC21'), []);
    const other = checkRoute(newRoute('skills A'), [X], FIELD);
    const i = at(other, 'RC21');
    assert.equal(i.length, 1);
    assert.equal(i[0].level, 'error');
    assert.ok(i[0].contract);
    assert.match(i[0].words, /registers the skills route as "skills"/);
    const fixed = applyFix(newRoute('skills A'), i[0].fix, addStep, updateStep);
    assert.equal(fixed.name, 'skills');
    assert.deepEqual(at(checkRoute(fixed, [X], FIELD), 'RC21'), []);
    // a name the LIBRARY's rules refuse as well (a hand-edited file): the page has no rename, so the same one-click fix
    // is on that error too -- and no sentence of the page's asks for a name it cannot take
    for (const [n, mirrored] of [['NONE', true], ['a"b', true], ['a\\b', true], ['', true], ['50%', true], ['x'.repeat(26), true],
                                 ['x'.repeat(30), false], ['caf\u00e9', true]]) {
        const r = { ...newRoute(n), mirroredFor: mirrored ? 'two' : undefined };
        const rc21 = at(checkRoute(r, [X], FIELD), 'RC21');
        assert.equal(rc21.length, 1, JSON.stringify(n));
        assert.ok(!rc21[0].contract, `${JSON.stringify(n)}: the library's own rule refused it`);
        assert.deepEqual(rc21[0].fix, { kind: 'nameSkills', stepIndex: -1 }, `${JSON.stringify(n)}: Name it "skills" is offered`);
        assert.doesNotMatch(rc21[0].words, /pick another|use 1 to/, `${JSON.stringify(n)}: words the page cannot act on`);
        const named = applyFix(r, rc21[0].fix, addStep, updateStep);
        assert.deepEqual(at(checkRoute(named, [X], FIELD), 'RC21'), [], `${JSON.stringify(n)}: fixed`);
    }
});

test('RC24 (R6): a tank leg NOT turned first -- an ERROR past the robot\'s own 15 deg, a WARNING from 0.5 deg', () => {
    const leg = (deg, faceFirst = false) => {
        const rad = deg * Math.PI / 180;
        return checkRoute(route([{ do: 'driveTo', x: -36 + 24 * Math.cos(rad), y: -55.7 + 24 * Math.sin(rad), faceFirst }]), [X], FIELD);
    };
    assert.equal(at(leg(16), 'RC24')[0].level, 'error');
    assert.match(at(leg(16), 'RC24')[0].library, /tank moveTo not steered: it starts 16\.0 deg off its target heading \(steers within 15\.0 deg\) -- turn first: face\(\)/);
    assert.equal(at(leg(10), 'RC24')[0].level, 'warn');
    assert.equal(at(leg(0.6), 'RC24')[0].level, 'warn');
    assert.deepEqual(at(leg(0.4), 'RC24'), [], 'within 0.5 deg it is on its line');
    assert.deepEqual(at(leg(45, true), 'RC24'), [], 'turned first, it is straight');
    // the one-click fix turns it on
    const r = route([{ do: 'driveTo', x: -36, y: -31.7, faceFirst: false }]);
    const issue = at(checkRoute(r, [X], FIELD), 'RC24')[0];
    const fixed = applyFix(r, issue.fix, addStep, updateStep);
    assert.equal(fixed.steps[0].faceFirst, true);
    assert.deepEqual(at(checkRoute(fixed, [X], FIELD), 'RC24'), []);
    // a holonomic robot has no such rule
    assert.deepEqual(at(checkRoute(r, [{ ...X, drive: { ...X.drive, tank: false } }], FIELD), 'RC24'), []);
});

test('RC25 (R1): a move on a stall- or switch-homed mechanism needs a FINISHED home on every path', () => {
    const one = ledgerRobot();
    const move = { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 };
    const none = checkRoute(route([move]), [one], FIELD);
    const i = at(none, 'RC25')[0];
    assert.equal(i.level, 'error');
    assert.match(i.words, /HOMED/);
    // the fix "home it first" adds a home whose time limit is EMPTY for the student to type
    const fixed = applyFix(route([move]), i.fix, addStep, updateStep);
    assert.equal(fixed.steps[0].do, 'home');
    assert.equal(fixed.steps[0].boundS, undefined, 'no invented number (§4)');
    assert.ok(at(checkRoute(fixed, [one], FIELD), 'RC19').some((x) => x.stepIndex === 0 && x.field === 'boundS'));
    // the second fix: home while the first leg drives, and wait for it before the move
    const driving = route([{ do: 'driveTo', x: -12, y: -55.7 }, move]);
    const j = at(checkRoute(driving, [one], FIELD), 'RC25')[0];
    const f2 = applyFix(driving, j.alsoFix, addStep, updateStep);
    assert.deepEqual(f2.steps.map((s) => s.do), ['startHome', 'driveTo', 'waitUntilDone', 'move']);
    assert.deepEqual(at(checkRoute({ ...f2, steps: f2.steps.map((s) => (s.boundS === undefined && s.mechanism ? { ...s, boundS: 2 } : s)) }, [one], FIELD), 'RC25'), []);
    // a startHome still RUNNING when the move comes (a new command replaces the home) -- RM21's case
    const running = checkRoute(route([{ do: 'startHome', mechanism: 'cascade', boundS: 3 }, move]), [one], FIELD);
    assert.ok(at(running, 'RC25').some((x) => /still homing/.test(x.words)));
    // startHome, waitUntilDone, then another command between? the wait finished it
    const waited = checkRoute(route([{ do: 'startHome', mechanism: 'cascade', boundS: 3 }, { do: 'waitUntilDone', mechanism: 'cascade', boundS: 3 }, move]), [one], FIELD);
    assert.deepEqual(at(waited, 'RC25'), []);
    // a home a skip can pass over is not on every path
    const skipped = checkRoute(route([
        { do: 'driveTo', x: -12, y: -55.7, care: { onFail: { skipTo: 'm' } } },
        { do: 'home', mechanism: 'cascade', boundS: 3 },
        { label: 'm', ...move },
    ]), [one], FIELD);
    assert.ok(at(skipped, 'RC25').some((x) => x.level === 'error' && /passed over/.test(x.words)));
    // a home that CARRIES ON when it fails: a warning on the move after it
    const carry = checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3, care: { onFail: 'carryOn' } }, move]), [one], FIELD);
    assert.deepEqual(at(carry, 'RC25').map((x) => x.level), ['warn']);
    // ...and the same when it is the WAIT for a started home that fails and carries on: the home is NOT finished
    // on that path (RM21's error on the failure path) -- a warning, never "homed"
    const waitCarry = checkRoute(route([{ do: 'startHome', mechanism: 'cascade', boundS: 3 },
                                         { do: 'waitUntilDone', mechanism: 'cascade', boundS: 3, care: { onFail: 'carryOn' } }, move]), [one], FIELD);
    assert.deepEqual(at(waitCarry, 'RC25').map((x) => [x.level, x.stepIndex]), [['warn', 2]]);
    assert.match(at(waitCarry, 'RC25')[0].words, /home fails \(step 2 fails and carries on\)/);
    // the "still homing" fix puts the wait BEFORE the move (and its time limit empty), and the error is gone
    const startThenMove = route([{ do: 'startHome', mechanism: 'cascade', boundS: 3 }, move]);
    const still = at(checkRoute(startThenMove, [one], FIELD), 'RC25')[0];
    assert.equal(still.fix.kind, 'waitForHome');
    const waited2 = applyFix(startThenMove, still.fix, addStep, updateStep);
    assert.deepEqual(waited2.steps.map((x) => x.do), ['startHome', 'waitUntilDone', 'move']);
    assert.equal(waited2.steps[1].boundS, undefined, 'no invented number (§4)');
    assert.deepEqual(at(checkRoute(waited2, [one], FIELD), 'RC25'), []);
    // a mechanism homed at power-up (robot x's arm rests on its stop) needs nothing
    assert.deepEqual(at(checkRoute(route([{ do: 'move', mechanism: 'arm', preset: 'score', boundS: 2 }]), [X], FIELD), 'RC25'), []);
    // robot x's lift is SWITCH-homed: included (the page cannot know the switch is pressed at boot, P6 (3))
    assert.equal(at(checkRoute(route([{ do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }]), [X], FIELD), 'RC25').length, 1);
});

test('RC26: a TIGHT bend is a WARNING (the library drives it, cutting the corner) -- Export stays open', () => {
    const tight = route([{ do: 'curveTo', points: [{ x: -24, y: -55.7 }, { x: -34, y: -47.7 }, { x: -12, y: -40 }] }]);
    const i = at(checkRoute(tight, [X], FIELD), 'RC26');
    assert.deepEqual(i.map((x) => x.level), ['warn']);
    assert.match(i[0].words, /tightest bend/);
    assert.match(i[0].library, /tight/i);
});

test('RC26 (R6): checkCurve\'s refusals from the PLANNED start, in the library\'s words, with the fix', () => {
    const behind = route([{ do: 'curveTo', points: [{ x: -60, y: -55.7 }, { x: -60, y: -30 }] }]);
    const i = at(checkRoute(behind, [X], FIELD), 'RC26')[0];
    assert.equal(i.level, 'error');
    assert.match(i.library, /^curve not driven: its first point is 180 deg off the robot's heading \(at most 90\) -- turn first: face\(\)$/);
    const fixed = applyFix(behind, i.fix, addStep, updateStep);
    assert.equal(fixed.steps[0].do, 'face');
    assert.ok(!at(checkRoute(fixed, [X], FIELD), 'RC26').some((x) => x.level === 'error'));
    const tooClose = route([{ do: 'curveTo', points: [{ x: -20, y: -55 }, { x: -19.5, y: -55 }] }]);
    assert.match(at(checkRoute(tooClose, [X], FIELD), 'RC26')[0].library, /point 2 is 0\.50 in from the point before it/);
    const reversed = route([{ do: 'reverseCurveTo', points: [{ x: -20, y: -55.7 }] }]);
    assert.match(at(checkRoute(reversed, [X], FIELD), 'RC26')[0].library, /off the robot's back .* turn first: faceAway\(\)/);
});

test('the same issue on robot one and robot two reads ONCE; a ledger level passes on both when both carry the ledger', () => {
    const one = ledgerRobot();
    const two = { ...ledgerRobot(), id: 'two', name: 'robot two' };
    const move = { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 };
    const noHome = at(checkRoute(route([move]), [one, two], FIELD), 'RC25');
    assert.equal(noHome.length, 1, 'one RC25, not one per robot');
    const homed = checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, move]), [one, two], FIELD);
    assert.deepEqual(homed, [], 'robot two\'s cascade level through its OWN ledger: nothing to say');
    // robot two WITHOUT its ledger marked (route_maker_robots.hpp before it is given one): every level refused there
    const bare = { ...two, mechanisms: two.mechanisms.map((m) => ({ ...m, presets: m.presets.map(({ ledger, ...p }) => p) })) };
    assert.ok(at(checkRoute(route([{ do: 'home', mechanism: 'cascade', boundS: 3 }, move]), [one, bare], FIELD), 'RC18')
        .some((x) => x.robot === 'two'));
});

test('Duplicate leaves no label behind (labels are unique: no RC16 after a copy)', () => {
    const r = route([{ label: 'grab', do: 'wait', seconds: 1 }]);
    const d = duplicateStep(r, 0);
    assert.equal(d.steps.length, 2);
    assert.equal(d.steps[1].label, undefined);
    assert.deepEqual(at(checkRoute(d, [X], FIELD), 'RC16'), []);
});

test('a step kind this builder does not write is an ERROR in memory too (a file with one is refused at open)', () => {
    const r = { ...route([{ do: 'wait', seconds: 1 }]) };
    r.steps = [...r.steps, { do: 'wallReset', wall: 'red' }];
    const i = checkRoute(r, [X], FIELD).filter((x) => x.field === 'do');
    assert.equal(i.length, 1);
    assert.equal(i[0].level, 'error');
    assert.equal(i[0].stepIndex, 1);
});

test('the page\'s own rule: a drive to where the robot already is has no direction (the library throws at 0)', () => {
    const issues = checkRoute(route([{ do: 'driveTo', x: -36.5, y: -55.7 }]), [X], FIELD);
    assert.equal(at(issues, 'RC1')[0].library, libraryWords('noBearing'));
});
