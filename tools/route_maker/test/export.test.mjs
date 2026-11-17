// export.test.mjs — the C++ the builder writes (D4; §6.3; GL0; UN1, UN3; [R6], [R10], [R12]).
// The goldens the C++ suite compiles and RUNS (test/route_maker_golden_test.cpp) must be exactly what this
// exporter writes now — so a change to the exporter is a change the suite judges.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { exportRegion, markerLine } from '../src/model/export.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { findRegion, regionText, sha256Hex } from '../src/model/region.mjs';
import { goldens } from '../goldens.mjs';
import { GOLDENS, REPO } from './helpers.mjs';

const START = { x: -36, y: -55.7, headingDeg: 0 };
const one = (step) => exportRegion(addStep(setStart(newRoute(), START), step));

/**
 * §6.3's empty region, as the brief writes it (extracted from its lines 2156-2181, 2026-11-14) — the CONTRACT ROB1-BT's
 * file carries byte for byte. gl0_empty.cpp is that text, and it has NO route.json: `goldens.mjs --write` never
 * re-makes it from the exporter, so the exporter is held to the contract, not to itself. Its hash is pinned here
 * as well, so editing the file to follow a drifted exporter is a deliberate two-place change (and a contract
 * change: both chunks at once, the coordinator's).
 */
const GL0_SHA256 = 'def1fbed7268dd80495427d09428ac9c1ee0f6a86b70227b3c0d165d36ee0680';

test('GL0: an EMPTY route exports §6.3\'s empty region byte for byte (the contract text, pinned -- never re-made from the exporter)', () => {
    const g = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
    assert.equal(sha256Hex(g), GL0_SHA256, 'gl0_empty.cpp is no longer §6.3\'s text');
    assert.ok(!fs.existsSync(path.join(GOLDENS, 'gl0_empty.route.json')), 'GL0 must not be re-made by goldens.mjs --write');
    assert.equal(`${exportRegion(newRoute())}\n`, g);
    assert.ok(g.startsWith('// ═══ ROUTE MAKER — route "skills" — BEGIN ═'), 'the golden IS the contract text');
});

test('GL0 against the REAL file: src/team/skills_route.cpp\'s region, once robot two\'s update puts it there, IS the empty export (or the builder\'s)', (t) => {
    const f = path.join(REPO, 'src', 'team', 'skills_route.cpp');
    if (!fs.existsSync(f)) {
        t.skip('src/team/skills_route.cpp is not in this tree yet (ROB1-BT creates it per §6.3)');
        return;
    }
    const text = fs.readFileSync(f, 'utf8');
    const found = findRegion(text);
    assert.ok(found.ok, found.words);
    const region = regionText(text, found);
    // ROB1-BT's starter is the empty region; after the students' first export it is a route -- then it must at
    // least be one this exporter could have written (its first and last lines, and a route named for the contract)
    if (region.includes('NO STEPS YET')) {
        assert.equal(region, exportRegion(newRoute()), 'the starter region drifted from the exporter: the first export would be refused as a hand edit');
    } else {
        assert.equal(found.name, 'skills');
    }
});

test('the goldens are FRESH: every committed export is what the exporter writes now (the suite runs them)', () => {
    const list = goldens();
    assert.ok(list.length >= 17, `only ${list.length} goldens`);
    for (const [name, project, cpp] of list) {
        assert.equal(fs.readFileSync(path.join(GOLDENS, `${name}.route.json`), 'utf8'), project,
                     `${name}.route.json is not canonical: node tools/route_maker/goldens.mjs --write, then read the diff`);
        assert.equal(fs.readFileSync(path.join(GOLDENS, `${name}.cpp`), 'utf8'), cpp,
                     `${name}.cpp is stale: node tools/route_maker/goldens.mjs --write, then read the diff`);
    }
});

test('markers: 91 characters for any route name, the name in quotes', () => {
    for (const n of ['skills', 'a', 'x'.repeat(25)]) {
        for (const w of ['BEGIN', 'END']) {
            const l = markerLine(n, w);
            assert.ok([...l].length >= 91);
            assert.ok(l.includes(`route "${n}" — ${w} ═══`));
        }
    }
});

test('[R12] on one step: label, then onFail, then the call; a turn-first drive is face THEN drive, its onFail before each', () => {
    const t = one({ label: 'go', do: 'driveTo', x: 0, y: -40, faceFirst: true, care: { onFail: { skipTo: 'go2' } } });
    const lines = t.split('\n').map((l) => l.trim());
    const i = lines.indexOf('r.label("go");');
    assert.ok(i > 0);
    assert.deepEqual(lines.slice(i, i + 5), [
        'r.label("go");',
        'r.onFail(chassis::skipTo("go2"));',
        'r.face(0_in, -40_in);',
        'r.onFail(chassis::skipTo("go2"));',
        'r.driveTo(0_in, -40_in);',
    ]);
    const back = one({ do: 'reverseTo', x: -60, y: -40, faceFirst: true }).split('\n').map((l) => l.trim());
    const f = back.indexOf('r.faceAway(-60_in, -40_in);');
    assert.ok(f > 0 && back[f + 1] === 'r.reverseTo(-60_in, -40_in);', 'reversing: its BACK turned to the point first');
    assert.ok(!one({ do: 'driveTo', x: 0, y: -55.7, faceFirst: false }).includes('r.face('), 'off: no turn written');
});

test('a turn-first drive\'s comment says the speed, time and tolerance are the DRIVE\'s, never the turn\'s', () => {
    const t = one({ do: 'driveTo', x: 0, y: -36, faceFirst: true, care: { timeoutS: 2, maxTurnDegPerS: 60 } });
    const comment = t.split('\n').filter((l) => /^\s*\/\//.test(l) && !/═/.test(l)).join(' ').replace(/\s*\/\/\s*/g, ' ');
    assert.match(comment, /First it turns to face the point \(at the robot's own turn settings\); the drive: at most 2 s, turning at most 60 deg\/s\./);
    assert.ok(t.includes('r.face(0_in, -36_in);'), 'the face: no options (the robot\'s own)');
    assert.ok(/r\.driveTo\(0_in, -36_in, \{\.timeout = 2_s, \.maxAngularSpeed = /.test(t), 'the caps on the drive');
});

test('the BRAIN\'s step numbers: a turn-first drive is two steps in the brain\'s log, and every comment after it says its number there', () => {
    let r = setStart(newRoute(), START);
    r = addStep(r, { do: 'wait', seconds: 1 });                                // page 1, brain 1
    r = addStep(r, { do: 'driveTo', x: 0, y: -36, faceFirst: true });          // page 2, brain 2 (turn) + 3 (drive)
    r = addStep(r, { do: 'driveTo', x: 12, y: -36, faceFirst: false });        // page 3, brain 4
    r = addStep(r, { do: 'reverseTo', x: -12, y: -36, faceFirst: true });      // page 4, brain 5 + 6
    r = addStep(r, { do: 'turnTo', headingDeg: 90 });                         // page 5, brain 7
    const text = exportRegion(r).replace(/\n\s*\/\/\s+/g, ' ');  // each comment joined onto one line
    assert.ok(!/Step 1 — [^\n]*brain's log/.test(text), 'before any turn-first drive the numbers agree: nothing said');
    assert.match(text, /Step 2 — [^\n]*\(The brain's log calls the turn step 2 and the drive step 3\.\)/);
    assert.match(text, /Step 3 — [^\n]*\(The brain's log calls this step 4\.\)/);
    assert.match(text, /Step 4 — [^\n]*\(The brain's log calls the turn step 5 and the drive step 6\.\)/);
    assert.match(text, /Step 5 — [^\n]*\(The brain's log calls this step 7\.\)/);
});

test('a wait is the RUN GUARD\'s pause inside r.then -- never r.pause (the 57 s cut cannot end that one)', () => {
    const t = one({ do: 'wait', seconds: 0.5 });
    assert.ok(t.includes('r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },'));
    assert.ok(!/r\.pause\(/.test(t));
});

test('UN1: every number carries its unit; the only bare forms are the caps\' two spellings and a level index', () => {
    for (const [name, , cpp] of goldens()) {
        const code = cpp.split('\n').map((l) => l.replace(/\/\/.*$/, '')).join('\n');
        const stripped = code
            .replace(/units::AngularVelocity\{-?\d+(\.\d+)? \* math::Angle::kPi \/ 180\.0\}/g, 'CAP')
            .replace(/-?\d+(\.\d+)?_in \/ 1_s/g, 'CAP')
            .replace(/facts::lv\(\d+\)/g, 'LEVEL')
            .replace(/-?\d+(\.\d+)?_(in|deg|s|volt)\b/g, 'Q')
            .replace(/"[^"]*"/g, 'STR');
        const bare = stripped.match(/(?<![A-Za-z_])\d+(\.\d+)?(?![A-Za-z_])/g) || [];
        assert.deepEqual(bare, [], `${name}: a number without its unit: ${bare.join(', ')}`);
    }
    // every turn cap is followed by its comment
    const t = one({ do: 'turnTo', headingDeg: 90, care: { maxTurnDegPerS: 270 } });
    assert.match(t, /\.maxAngularSpeed = units::AngularVelocity\{270 \* math::Angle::kPi \/ 180\.0\}\}\);  \/\/ at most 270 deg\/s/);
    assert.ok(!/\(\d+_deg\)\.radians\(\)/.test(t), 'never through _deg: it wraps (P5)');
});

test('UN3: headings exported in (-180, 180]; 180 written 180_deg, -180 never', () => {
    assert.ok(one({ do: 'turnTo', headingDeg: -180 }).includes('r.turnTo(180_deg);'));
    assert.ok(one({ do: 'turnTo', headingDeg: 270 }).includes('r.turnTo(-90_deg);'));
    assert.ok(one({ do: 'turnTo', headingDeg: 540 }).includes('r.turnTo(180_deg);'));
    assert.ok(exportRegion(setStart(newRoute(), { x: 0, y: 0, headingDeg: -180 })).includes('StartPose::field(0_in, 0_in, 180_deg)'));
});

test('MotionOptions in declaration order (C++20 designated initializers must be), 0 meaning the robot\'s own', () => {
    const t = one({ do: 'driveTo', x: 0, y: -40, care: { headingDeg: 1, toleranceDeg: 5, toleranceIn: 2, maxTurnDegPerS: 90, maxSpeedInPerS: 30, timeoutS: 2.5 } });
    assert.match(t, /\{\.timeout = 2\.5_s, \.maxLinearSpeed = 30_in \/ 1_s, \.maxAngularSpeed = units::AngularVelocity\{90 \* math::Angle::kPi \/ 180\.0\}, \.positionTolerance = 2_in, \.headingTolerance = 5_deg\}/);
    assert.ok(!one({ do: 'driveTo', x: 0, y: -40, care: { timeoutS: 0 } }).includes('.timeout'), '0 = the robot\'s own: not written');
});

test('a level of the facts ledger is named through facts::lv(i), a named preset by its name', () => {
    assert.ok(one({ do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 }).includes('r.move(cascade[facts::lv(2).name], 2_s);'));
    assert.ok(one({ do: 'startMove', mechanism: 'lift', preset: 'low', boundS: 1.5 }).includes('r.startMove(lift["low"], 1.5_s);'));
    assert.ok(one({ do: 'set', mechanism: 'claw', position: 'open' }).includes('r.set(claw["open"]);'));
    assert.ok(one({ do: 'run', mechanism: 'intake', volts: -6, forS: 0.3 }).includes('r.run(intake, -6_volt, 0.3_s);'));
});

test('the export depends on the route ALONE: no time, no hash, no path in it; the same route, the same bytes', () => {
    const r = addStep(setStart(newRoute(), START), { do: 'wait', seconds: 1 });
    const a = exportRegion(r);
    assert.equal(a, exportRegion(JSON.parse(JSON.stringify(r))));
    assert.ok(!/sha256|20\d\d-\d\d|\/home\/|C:\\|#include|using namespace/.test(a));
});

test('every step is numbered as the page numbers it, in plain words (no "Routine" or "precondition")', () => {
    for (const [name, , cpp] of goldens()) {
        const comments = cpp.split('\n').filter((l) => /^\s*\/\/ Step \d+ — /.test(l));
        const nums = comments.map((l) => Number(/Step (\d+)/.exec(l)[1]));
        assert.deepEqual(nums, nums.map((_, i) => i + 1), name);
        assert.ok(!/precondition/i.test(comments.join('\n')), name);
    }
});
