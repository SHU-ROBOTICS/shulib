// module.test.mjs — THE PREVIEW ENGINE, the WebAssembly module under Node (RMP1-A2e; the RMP1 brief A.2, A.3.10, A.8):
// WG1 (the runner and the exported C++ give the same record), WG3 (the same bytes from both link forms, from a reused
// instance, and — given ROUTE_MAKER_WASM_RECORDS — from another OS's run), WG7 (the no-pace stop inside the module),
// SM1–SM3 (each page step's span and verdict, exact), RG1 (no build path inside), the lines it refuses, and what
// the record says checked against the route itself (the writer is shared by both sides of WG1–WG3).
//
//   ROUTE_MAKER_WASM=build/wasm node --test tools/route_maker/test/wasm/
//   ROUTE_MAKER_WASM_RECORDS=<file>        compare every golden's record with that file's (WG3 across OSes)
//   ROUTE_MAKER_WASM_WRITE_RECORDS=<file>  write them there (CI's Linux job, for the other rows)
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { routeLines } from '../../src/model/route_lines.mjs';
import { addStep, newRoute, setStart } from '../../src/model/route.mjs';
import { REPO, goldenNames } from '../helpers.mjs';
import { WASM, goldenRoutes, instance, needWasm, robot, spansHold, withoutPageSteps } from './engine.mjs';

const opts = { skip: needWasm };
const X = robot('x');
const START = { x: -36, y: -55.7, headingDeg: 0 };
const routeFrom = (start, ...steps) => steps.reduce((r, s) => addStep(r, s), setStart({ ...newRoute(), robot: 'x' }, start));
const route = (...steps) => routeFrom(START, ...steps);
const json = (r) => JSON.parse(r.out);

let site;
let goldens;
async function engine() {
    site = site || (await instance('preview.mjs'));
    goldens = goldens || goldenRoutes();
    return site;
}

test('WG1: every golden — the runner on its lines and its EXPORTED C++ compiled in — the same record, byte for byte', opts, async () => {
    const mod = await engine();
    const twin = await instance('preview_test.mjs');
    const listed = twin.run(['goldens']).out.trim().split('\n').map((l) => l.split(' ')[0]).sort();
    assert.deepEqual(listed, ['gl0_empty', ...goldenNames()].sort(), 'the test module compiles every golden in (route_maker_goldens.cpp)');
    assert.ok(listed.includes('wg8_distinct_numbers'), 'WG8 is a golden');
    for (const g of goldens) {
        const a = mod.run(['preview', g.lines]);
        const b = twin.run(['golden', g.name]);
        assert.equal(a.err, '', `${g.name}: ${a.err}`);
        assert.equal(a.rc, b.rc, g.name);
        assert.ok(a.out.length > 1000 && a.out.includes(`"pageSteps": ${g.route.steps.length ? '[{' : '[]'}`), `${g.name}: no record`);
        assert.equal(withoutPageSteps(a.out), withoutPageSteps(b.out), `${g.name}: the runner and the export ran differently`);
        assert.ok(b.out.includes('"pageSteps": null'), g.name);
    }
});

test('any robot\'s own list: robot one runs on ITS mechanisms, every fact its ledger still owes marked a PLACEHOLDER (RPV1-S1)', opts, async () => {
    const mod = await engine();
    const one = json(mod.run(['preview', goldens.find((g) => g.name === 'smoke_robot_one').lines]));
    assert.equal(one.robot, 'one');
    assert.match(one.robotLine, /ROBOT ONE's mechanisms \(cascade, right toggle, left toggle, claw, flipper, ramp\)/);
    assert.equal(one.ledgerFile, 'src/robots/robot_one_facts.hpp');
    // every `owed(<placeholder>, Stage::<s>, "<name>")` in the ledger file itself (comments aside), and only those
    const code = fs.readFileSync(path.join(REPO, 'src', 'robots', 'robot_one_facts.hpp'), 'utf8')
        .split('\n').filter((l) => !l.trim().startsWith('//')).join('\n');
    const owed = [...code.matchAll(/\bowed\([\s\S]*?,\s*Stage::\w+,\s*"((?:[^"\\]|\\.)*)"\s*\)/g)].map((m) => m[1]);
    assert.ok(owed.length > 0, 'robot one\'s ledger still owes facts (when none is owed, this test changes)');
    assert.deepEqual([...one.owed].sort(), owed.sort(), 'the record\'s owed facts are the ledger\'s');
    // the cascade (a lift on the library's starting values) is a PLACEHOLDER time; the claw (a piston) is not
    const on = (what) => one.steps.filter((s) => s.what === what).map((s) => s.placeholder);
    assert.equal(on('cascade').length, 1);
    assert.match(on('cascade')[0], /^PLACEHOLDER numbers — this time is what its declaration predicts/);
    assert.deepEqual(on('claw'), ['', '']);
    const x = json(mod.run(['preview', goldens.find((g) => g.name === 'gl4_mechanisms').lines]));
    assert.deepEqual([x.robot, x.owed, x.ledgerFile], ['x', [], ''], 'a fixture robot has no ledger');
    // tripwire: no golden moves to a ledger level, because no real robot has a level yet
    const levels = ['one', 'two'].flatMap((id) => robot(id).mechanisms.flatMap((m) => (m.presets || []).map((p) => `${id}/${m.name}/${p.name}`)));
    assert.deepEqual(levels, [], 'a real robot has levels now: add a golden on it that moves to a ledger level, so WG1 '
                     + 'runs the export\'s facts::lv(i) (RMP1-PROGRESS, RMP1-A2e review item 5)');
});

test('WG3: the SITE form and the ONE FILE, a reused instance and a fresh one: the same bytes (and the other OSes\' when given)', opts, async () => {
    const mod = await engine();
    const one = await instance('preview_single.mjs');
    const records = {};
    for (const g of goldens) {
        const a = mod.run(['preview', g.lines]).out;
        assert.equal(one.run(['preview', g.lines]).out, a, `${g.name}: the one file differs`);
        assert.equal(mod.run(['preview', g.lines]).out, a, `${g.name}: a second run on the same instance differs`);
        records[g.name] = a;
    }
    const fresh = await instance('preview.mjs');
    assert.equal(fresh.run(['preview', goldens[1].lines]).out, records[goldens[1].name]);
    if (process.env.ROUTE_MAKER_WASM_WRITE_RECORDS) {
        fs.writeFileSync(process.env.ROUTE_MAKER_WASM_WRITE_RECORDS, JSON.stringify(records));
    }
    if (process.env.ROUTE_MAKER_WASM_RECORDS) {
        const want = JSON.parse(fs.readFileSync(process.env.ROUTE_MAKER_WASM_RECORDS, 'utf8'));
        assert.deepEqual(Object.keys(records).sort(), Object.keys(want).sort());
        for (const [name, text] of Object.entries(want)) {
            assert.equal(records[name], text, `${name}: this OS's record differs from the Linux job's`);
        }
    }
});

test('WG7: a route that loops without letting time pass — callMain returns 1 WITH its record; the instance runs on', opts, async () => {
    const twin = await instance('preview_test.mjs');
    const lines = `${routeLines(route({ do: 'wait', seconds: 0.25 }), X)}step do=testLoop\nstep do=turnTo h=90\n`;
    const r = twin.run(['preview', lines]);
    assert.equal(r.rc, 1);
    const rec = json(r);
    assert.equal(rec.looped, true);
    assert.match(rec.loopSentence, /keeps looping without letting time pass/);
    assert.deepEqual(rec.pageSteps.map((p) => p.verdict), ['ok', 'looping', 'notReached']);
    assert.equal(rec.pageSteps[1].first, rec.pageSteps[0].last);
    assert.equal(rec.pageSteps[2].first, null);
    assert.equal(rec.pageSteps[2].lib, null);
    const g = goldens.find((x) => x.name === 'gl1_drive_turn_first');
    const after = twin.run(['preview', g.lines]);
    const fresh = (await instance('preview_test.mjs')).run(['preview', g.lines]);
    assert.equal(after.out, fresh.out, 'the same instance, after the stop, runs a route as a fresh one does');
    assert.equal((await engine()).run(['preview', lines]).rc, 2, 'the shipped module refuses the test-only step');
});

test('one instance runs PREVIEW hundreds of times: the page\'s wrapper gives the module its stack back after each call', opts, async () => {
    const mod = await engine();
    const long = `shulib-route 1\n${'#'.repeat(4000)}\n`;  // refused at once; 4 KB of argument on each call
    for (let i = 0; i < 400; ++i) {
        assert.equal(mod.run(['preview', long]).rc, 2);
    }
    assert.equal(mod.run(['preview', goldens[1].lines]).rc, 0);
});

test('SM: on every golden the page steps tile the run and a motion\'s span is its motion\'s, pace for pace', opts, async () => {
    const mod = await engine();
    for (const g of goldens) {
        const rec = json(mod.run(['preview', g.lines]));
        assert.equal(rec.pageSteps.length, g.route.steps.length, g.name);
        if (rec.pageSteps.length) {
            spansHold(rec, g.name);
        }
    }
});

test('SM1–SM3: a skip, a carry-on, a stop and the 57 s cut — each page step\'s verdict, and the brain\'s numbers', opts, async () => {
    const mod = await engine();
    const blocked = (onFail) => ({ do: 'driveTo', x: 0, y: -55.7, faceFirst: false, care: { timeoutS: 0.3, ...(onFail ? { onFail } : {}) } });
    const rest = [{ do: 'driveTo', x: -20, y: -40, faceFirst: true }, { do: 'turnTo', headingDeg: 90 }, { label: 'after', do: 'wait', seconds: 0.2 }];
    const ran = (lines) => {
        const r = mod.run(['preview', lines]);
        assert.equal(r.rc, 0, 'a failed, skipped, jumped or cut step is rc 0: the record says it (README)');
        return json(r);
    };
    const verdicts = (r) => {
        const rec = ran(routeLines(r, X));
        spansHold(rec, 'SM');
        return rec.pageSteps.map((p) => [p.verdict, p.lib]);
    };
    // SM1: the skip lands on "after": the two steps between are JUMPED (the turn-first drive is two brain steps)
    assert.deepEqual(verdicts(route(blocked({ skipTo: 'after' }), ...rest)),
                     [['failedWentOn', [1, 1]], ['jumped', [2, 3]], ['jumped', [4, 4]], ['ok', [5, 5]]]);
    // SM2: carry on — the failure is counted, everything after runs
    assert.deepEqual(verdicts(route(blocked('carryOn'), ...rest)),
                     [['failedWentOn', [1, 1]], ['ok', [2, 3]], ['ok', [4, 4]], ['ok', [5, 5]]]);
    // stop (the default): every later step SKIPPED, as the brain's log says
    assert.deepEqual(verdicts(route(blocked(), ...rest)),
                     [['failed', [1, 1]], ['skipped', [2, 3]], ['skipped', [4, 4]], ['skipped', [5, 5]]]);
    // SM3: the 57 s cut ends the wait: CUT, and every later step skipped
    const cut = ran(routeLines(route({ do: 'wait', seconds: 60 }, ...rest), X));
    assert.deepEqual(cut.pageSteps.map((p) => p.verdict), ['cut', 'skipped', 'skipped', 'skipped']);
    assert.equal(cut.cut, true);
    assert.equal(cut.pageSteps[0].last, cut.pageSteps[1].first);
    assert.ok(Math.abs(cut.samples[cut.pageSteps[0].last][0] - 57) <= 0.01, 'the wait ends AT the cut');
});

test('RG1: no build path inside the module (the .wasm, and the one file\'s DECODED module); describe is the native program\'s', opts, async () => {
    const bad = ['/home/', '/Users/', 'C:\\', '/runner/', '.claude'];
    const single = fs.readFileSync(path.join(WASM, 'preview_single.mjs'), 'utf8');
    const b64 = /base64,([A-Za-z0-9+/=]+)/.exec(single) || /"([A-Za-z0-9+/]{100000,}={0,2})"/.exec(single);
    assert.ok(b64, 'the one file carries its module as base64');
    const decoded = Buffer.from(b64[1], 'base64');
    assert.equal(decoded.subarray(0, 4).toString('latin1'), '\0asm', 'the one file\'s module decodes to WebAssembly');
    for (const blob of [fs.readFileSync(path.join(WASM, 'preview.wasm')), decoded, Buffer.from(single)]) {
        for (const b of bad) {
            assert.equal(blob.indexOf(b), -1, `a build path (${b}) in the module`);
        }
    }
    assert.ok(decoded.includes('test/../src/robots/mechanism_table.hpp'), 'a kept file name is relative (W10)');
    const r = (await engine()).run(['describe', '--robot', 'x']);
    assert.equal(r.rc, 2);
    assert.match(r.err, /native program only/);
});

test('the lines the export could not have written are refused by name — never run', opts, async () => {
    const mod = await engine();
    const head = routeLines(route(), X);
    const refused = {
        'step do=move mech=lift preset=nope bound=1\n': /no_such_preset_on_this_mechanism/,
        'step do=set mech=hood pos=sideways\n': /no_such_position_on_this_mechanism/,
        'step do=run mech=lift volts=6 for=1\n': /not a power mechanism/,
        'step do=home mech=intake bound=1\n': /not an arm or a lift/,
        'step do=move mech=cascade preset=L1 bound=1\n': /no mechanism "cascade"/,
        'step do=wallReset wall=red\n': /cannot run a "wallReset"/,
        'step do=driveTo x=1 y=2\n': /needs "face"/,
        'step do=driveTo x=1e3 y=2 face=0\n': /"x" needs a number/,
        'step do=wait s=1 timeout=2\n': /does not take one of its words/,
        'step do=turnTo h=5 h=6\n': /given twice/,
        'step do=driveTo x=1 y=2 face=0 fail=skipTo\n': /fail is carryOn/,
        'step do=driveTo x=1 y=2 face=2\n': /face is 0 or 1/,
        'step do=driveTo x=1 y=2 face=0 p=1,2\n': /only a curve, has points/,
        'step do=curveTo\n': /only a curve, has points/,
        'step do=testLoop\n': /cannot run a "testLoop"/,
    };
    for (const [line, why] of Object.entries(refused)) {
        const r = mod.run(['preview', head + line]);
        assert.equal(r.rc, 2, line);
        assert.equal(r.out, '', `${line}: nothing runs`);
        assert.match(r.err, why, line);
    }
    assert.match(mod.run(['preview', 'shulib-route 2\n']).err, /version 1/);
});

test('what the record says, checked against the route itself (WG1–WG3 compare two records from ONE writer)', opts, async () => {
    const mod = await engine();
    // the start, a drive's target and where the robot ends: x, y and heading each in its own place
    const d = json(mod.run(['preview', routeLines(routeFrom({ x: -36, y: -55.7, headingDeg: 30 },
                                                             { do: 'driveTo', x: -20, y: -40, faceFirst: true }), X)]));
    assert.deepEqual(d.samples[0].slice(0, 3), [0, -36, -55.7]);
    assert.ok(Math.abs(d.samples[0][3] - Math.PI / 6) < 1e-12, 'the start heading, in radians');
    assert.ok(Math.abs(d.samples[1][0] - 0.01) < 1e-9, 'a sample every 10 ms');
    const end = d.samples.at(-1);
    assert.ok(Math.hypot(end[1] + 20, end[2] + 40) < 1, `the drive ends at (-20, -40), not at ${end[1]}, ${end[2]}`);
    const drive = d.steps.find((s) => s.what === 'MoveToPose');
    assert.deepEqual(drive.target.slice(0, 2), [-20, -40]);
    assert.deepEqual(d.pageSteps.map((p) => [p.do, p.verdict, p.lib]), [['driveTo', 'ok', [1, 2]]]);
    // park and stop as the route set them
    const ps = json(mod.run(['preview', routeLines({ ...route({ do: 'wait', seconds: 0.5 }), endActionAtS: 55.5, hardStopAtS: 58.25 }, X)]));
    assert.deepEqual([ps.park, ps.stop], [55.5, 58.25]);
    // a wait on a mechanism never started THROWS: the step "threw", the next never reached, neither has brain steps
    const t = mod.run(['preview', routeLines(route({ do: 'waitUntilDone', mechanism: 'arm', boundS: 1 }, { do: 'turnTo', headingDeg: 90 }), X)]);
    assert.equal(t.rc, 1);
    const th = json(t);
    assert.deepEqual([th.threw, th.word], [true, 'PROGRAM ERROR']);
    assert.match(th.why, /started no step on that mechanism/);
    assert.deepEqual(th.pageSteps.map((p) => [p.verdict, p.first, p.lib]), [['threw', null, null], ['notReached', null, null]]);
    // through the right wall: a TOUCH, then the THROUGH that ends the check
    const w = mod.run(['preview', routeLines(route({ do: 'driveTo', x: 70, y: -55.7, faceFirst: false }), X)]);
    assert.equal(w.rc, 1);
    const wr = json(w);
    assert.deepEqual(wr.field.map((f) => [f.kind, f.element, f.step]), [['TOUCH', 'right wall', 1], ['THROUGH', 'right wall', 1]]);
    assert.equal(wr.through, true);
    assert.equal(wr.throughAt, wr.field[1].s);
    assert.ok(wr.samples[wr.throughAt][1] > 60, 'THROUGH where the robot meets the right wall');
});

test('a route the registry refuses: its sentence, nothing run, every page step not reached', opts, async () => {
    const mod = await engine();
    const head = routeLines(route(), X);
    for (const [lines, why] of [[head.replace('park=57 stop=59', 'park=59 stop=57'), /hardStopAt must be finite and >= endActionAt/],
                                [head.replace('name=skills', 'name='), /has no name/]]) {
        const r = mod.run(['preview', `${lines}step do=wait s=1\nstep do=turnTo h=90\n`]);
        assert.equal(r.rc, 1, lines);
        const rec = json(r);
        assert.deepEqual([rec.refused, rec.word, rec.samples.length], [true, 'REFUSED', 0]);
        assert.match(rec.why, why);
        assert.deepEqual(rec.pageSteps.map((p) => [p.verdict, p.first, p.lib]), [['notReached', null, null], ['notReached', null, null]]);
    }
});

test('the page\'s preview(): a record parsed, a refusal\'s words, and no word of one call left in the next', opts, async () => {
    const e = await instance('preview.mjs');
    const bad = e.preview(`${routeLines(route(), X)}step do=turnTo h=5 h=6\n`);
    assert.deepEqual([bad.rc, bad.record, bad.text], [2, null, '']);
    assert.match(bad.words, /given twice/);
    const g = (goldens || goldenRoutes())[1];
    const good = e.preview(g.lines);
    assert.deepEqual([good.rc, good.words], [0, '']);
    assert.deepEqual(good.record, JSON.parse(good.text));
    assert.equal(good.record.schema, 1);
    assert.equal(good.record.pageSteps.length, g.route.steps.length);
});
