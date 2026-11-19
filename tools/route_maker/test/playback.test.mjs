// playback.test.mjs — PREVIEW's record played (RMP1-A2u): every number the screen draws comes from the record. The
// fixtures are REAL runs (test/fixtures/preview/, recorded by the engine's native twin — README "PREVIEW").
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { routeKey, playbackOf, poseAt, sampleIndexAt, stepAt, stepTime, thinPath, ownersOf, noteWords } from '../src/model/playback.mjs';
import { ROUTES, PAIR, projectOf } from './fixtures/record.mjs';

const DIR = path.join(path.dirname(fileURLToPath(import.meta.url)), 'fixtures', 'preview');
const fixture = (n) => JSON.parse(fs.readFileSync(path.join(DIR, `${n}.preview.json`), 'utf8'));
// robot one's mechanisms, ident -> .name, as its description had them when the fixtures were recorded
const ONE = { cascade: 'cascade', rightToggle: 'right toggle', leftToggle: 'left toggle', claw: 'claw', flipper: 'flipper', ramp: 'ramp' };
const namesOf = (n) => fixture(n).project.routes[0].steps.map((x) => ONE[x.mechanism] || '');
const play = (n) => playbackOf(fixture(n).result, { one: namesOf(n), two: namesOf(n) });

test('poseAt: a pace\'s own pose exactly; straight between two; held before the first and after the last', () => {
    const s = [[0, 0, 0, 0], [0.01, 1, 2, Math.PI / 2], [0.02, 3, 2, Math.PI]];
    assert.deepEqual(poseAt(s, 0.01), { x: 1, y: 2, headingDeg: 90 });
    const mid = poseAt(s, 0.015);
    assert.ok(Math.abs(mid.x - 2) < 1e-9 && Math.abs(mid.y - 2) < 1e-9 && Math.abs(mid.headingDeg - 135) < 1e-9);
    assert.deepEqual(poseAt(s, -5), { x: 0, y: 0, headingDeg: 0 });
    assert.deepEqual(poseAt(s, 9), { x: 3, y: 2, headingDeg: 180 });
    assert.equal(sampleIndexAt(s, 0.0199), 1);
    assert.equal(sampleIndexAt(s, 0.02), 2);
    assert.equal(sampleIndexAt([], 1), 0);
});

test('poseAt on a real run: every pace of the ready route is the record\'s own pose', () => {
    const p = play('ready');
    for (const s of p.one.samples) {
        const q = poseAt(p.one.samples, s[0]);
        assert.deepEqual([q.x, q.y], [s[1], s[2]]);
        assert.ok(Math.abs(q.headingDeg - (s[3] * 180) / Math.PI) < 1e-9);
    }
});

test('a ready route: every step done, in order, tiling the run; the home marked PLACEHOLDER (robot one\'s owed facts)', () => {
    const p = play('ready');
    assert.deepEqual(p.one.steps.map((s) => s.state), ['ok', 'ok', 'ok', 'ok', 'ok']);
    for (let i = 1; i < p.one.steps.length; ++i) {
        assert.equal(p.one.steps[i].t0, p.one.steps[i - 1].t1, `step ${i + 1} starts where step ${i} ends`);
    }
    assert.match(p.one.steps[0].placeholder, /^PLACEHOLDER numbers/);
    assert.equal(p.one.steps[1].placeholder, '');
    assert.ok(p.owed.length > 0 && p.owed.every((x) => typeof x === 'string'));
    assert.deepEqual(p.headline, { word: 'FINISHED', mark: 'ok' });
    assert.equal(p.two, null);
    assert.equal(p.park, 57);
});

test('THROUGH a wall (RPV1\'s field check): its step red saying where, the steps after it greyed, the headline as RPV1 says it', () => {
    const f = fixture('through');
    const hit = f.result.one.field.find((x) => x.kind === 'THROUGH');
    const p = playbackOf(f.result);
    assert.deepEqual(p.one.steps.map((s) => [s.verdict, s.state]), [['ok', 'failed'], ['ok', 'skipped']]);
    assert.equal(p.one.steps[0].words, 'Went THROUGH the right wall');
    assert.equal(p.one.steps[0].library[0], `On the robot it hits the right wall at ${hit.t.toFixed(2)} s, in step 1 (${hit.depth.toFixed(2)} in past `
                 + 'its face): the host test plant has no walls.');
    assert.equal(p.one.steps[1].words, 'After the THROUGH: the robot would not get here', 'not "Done": the robot would not get there');
    assert.deepEqual(p.headline, { word: 'FINISHED, but THROUGH the right wall', mark: 'failed' });
    assert.equal(p.one.through.s, f.result.one.throughAt, 'the moment the drawing fades from');
    // a TOUCH (normal: driving up to a wall) marks nothing
    const touch = playbackOf({ ...f.result, one: { ...f.result.one, field: [{ ...hit, kind: 'TOUCH' }], through: false } });
    assert.deepEqual(touch.one.steps.map((s) => s.state), ['ok', 'ok']);
    assert.equal(touch.one.through, null);
    // at a step's last pace (shared with the next), the NEXT step owns it; the run's last pace, the last step
    const at = (s) => playbackOf({ ...f.result, one: { ...f.result.one, field: [{ ...hit, s }] } }).one.steps.map((x) => x.state);
    const [a, b] = f.result.one.pageSteps;
    assert.deepEqual(at(a.last), ['ok', 'failed']);
    assert.deepEqual(at(b.last), ['ok', 'failed']);
    assert.deepEqual(at(a.first), ['failed', 'skipped']);
    // in the park (after the last step): no step is marked, the headline still says it
    const park = playbackOf({ ...f.result, one: { ...f.result.one, field: [{ ...hit, s: b.last + 3 }] } });
    assert.deepEqual(park.one.steps.map((x) => x.state), ['ok', 'ok']);
    assert.equal(park.one.through.at, -1);
    assert.equal(park.headline.mark, 'failed');
});

test('a field note says the PAGE\'s step, never RPV1\'s from-below number; after the last step, says so', () => {
    const f = fixture('through');
    const p = playbackOf(f.result);
    assert.deepEqual(p.one.notes.map((n) => n.at), [0, 0, 0]);
    assert.ok(p.one.notes.every((n) => noteWords(n).includes('in step 1')));
    // the ready route: RPV1 counts the turn-first drive as two (steps 2 and 3), so its curve is RPV1's step 4 — the page's 3
    const r = fixture('ready').result.one;
    const curve = r.pageSteps[2];
    const touch = { kind: 'TOUCH', element: 'left wall', s: curve.first + 5, t: r.samples[curve.first + 5][0], step: 4, depth: 0.1 };
    const q = playbackOf({ one: { ...r, field: [touch] } });
    assert.match(noteWords(q.one.notes[0]), /^Touches the left wall at \d+\.\d\d s, in step 3: normal, not a failure\.$/);
    assert.match(noteWords({ ...touch, kind: 'PIECE', element: 'Toggle', at: -1 }), /after the last step \(the host test plant does not push pieces\)/);
});

test('the program stopped (threw: a real run) and a step that never let time pass (looping): red, the library\'s own sentence last', () => {
    const t = playbackOf(fixture('threw').result);
    assert.deepEqual(t.one.steps.map((s) => [s.verdict, s.state]), [['ok', 'ok'], ['threw', 'failed'], ['notReached', 'skipped']]);
    assert.equal(t.one.steps[1].words, 'The program stopped here');
    assert.equal(t.one.steps[1].library.at(-1), fixture('threw').result.one.why);
    assert.match(t.one.steps[1].library.at(-1), /^r\.waitUntilDone\(\.\.\.\): this routine started no step on that mechanism/);
    assert.deepEqual(t.headline, { word: 'PROGRAM ERROR', mark: 'failed' });
    // looping: shaped as the runner writes it (route_maker_runner.hpp: a reached step with no end, the record's looped)
    const f = fixture('failed').result.one;
    const loop = { ...f, looped: true, loopSentence: 'the test\'s own no-pace sentence', word: 'LOOPED',
                   pageSteps: f.pageSteps.map((p, i) => (i === 1 ? { ...p, verdict: 'looping' } : p)) };
    const l = playbackOf({ one: loop });
    assert.equal(l.one.steps[1].state, 'failed');
    assert.equal(l.one.steps[1].words, 'Never let time pass: stopped');
    assert.equal(l.one.steps[1].library.at(-1), 'the test\'s own no-pace sentence');
    assert.equal(l.headline.mark, 'failed');
});

test('a failed step: red, with the library\'s OWN line; the steps after it greyed with theirs', () => {
    const p = play('failed');
    assert.deepEqual(p.one.steps.map((s) => [s.verdict, s.state]),
                     [['ok', 'ok'], ['failed', 'failed'], ['skipped', 'skipped'], ['skipped', 'skipped']]);
    assert.deepEqual(p.one.steps[1].library, ["routine 'skills' STOPPED at step 2 (driveTo): motion TIMEOUT — skipping the rest; drive safed"]);
    assert.deepEqual(p.one.steps[2].library, ["routine 'skills': step 3 (turnTo) skipped — stopped at step 2"]);
    assert.deepEqual(p.one.steps[0].library, [], 'a done step quotes nothing');
    assert.equal(p.headline.mark, 'failed');
});

test('carry on and skip ahead: both failures red, the jumped step greyed, the landing step names the landing', () => {
    const p = play('carry');
    assert.deepEqual(p.one.steps.map((s) => s.verdict), ['failedWentOn', 'failedWentOn', 'jumped', 'ok']);
    assert.deepEqual(p.one.steps.map((s) => s.state), ['failed', 'failed', 'skipped', 'ok'], 'a jumped step is greyed, never "done"');
    assert.match(p.one.steps[0].library[0], /step 1 \(driveTo\) FAILED: motion TIMEOUT — carrying on/);
    assert.match(p.one.steps[1].library[0], /skipping to 'finish'/);
    assert.deepEqual(p.one.steps[2].library, ["routine 'skills': step 3 (claw) jumped — skipping to 'finish'"]);
    assert.deepEqual(p.one.steps[3].library, ["routine 'skills': the skip to 'finish' landed at step 4 (wait 0.3 s)"]);
});

test('the 57 s cut: the wait cut amber, the rest skipped, the headline "cut"', () => {
    const p = play('cut');
    assert.deepEqual(p.one.steps.map((s) => s.state), ['ok', 'cut', 'skipped']);
    assert.ok(p.one.steps[1].t1 >= 57 && p.one.steps[1].t1 < 57.05);
    assert.deepEqual(p.headline, { word: 'CUT BY DEADLINE', mark: 'cut' });
    assert.ok(p.duration > 57 && p.duration < 59);
});

test('stepAt: the step whose span holds t; a zero-length step never "plays"; before and after: none', () => {
    const p = play('failed');
    for (const [i, s] of p.one.steps.entries()) {
        if (s.t1 > s.t0) {
            assert.equal(stepAt(p, (s.t0 + s.t1) / 2), i);
            assert.equal(stepAt(p, s.t0), i);
        }
    }
    assert.equal(stepAt(p, p.one.steps[1].t1), -1, 'the skipped steps take no time');
    assert.equal(stepAt(p, -1), -1);
});

test('stepTime: next and previous walk every step\'s start, then the run\'s ends', () => {
    const p = play('ready');
    const starts = p.one.steps.map((s) => s.t0);
    let t = 0;
    const seen = [];
    for (let k = 0; k < 10 && t < p.duration; ++k) {
        t = stepTime(p, t, 1);
        seen.push(t);
    }
    assert.deepEqual(seen, [...starts.slice(1), p.duration]);
    assert.equal(stepTime(p, p.duration, -1), starts[starts.length - 1]);
    assert.equal(stepTime(p, starts[2] + 0.2, -1), starts[2], 'inside a step: back to its start');
    assert.equal(stepTime(p, starts[2] + 0.01, -1, true), starts[1], 'PLAYING, just after a start: the step before (a player\'s "previous")');
    assert.equal(stepTime(p, starts[2] + 0.01, -1), starts[2], 'paused: this step\'s start');
    assert.equal(stepTime(p, 0, -1), 0);
});

test('stepTime: a step of 30 ms is a stop BOTH ways (Left never passes over it while paused)', () => {
    const steps = [[0, 1], [1, 1.03], [1.03, 2]].map(([t0, t1], i) => ({ n: i + 1, t0, t1, state: 'ok' }));
    const p = { one: { steps }, duration: 2.5 };
    const walk = (t, dir) => {
        const out = [];
        for (let k = 0; k < 4; ++k) {
            t = stepTime(p, t, dir);
            out.push(t);
        }
        return out;
    };
    assert.deepEqual(walk(0, 1), [1, 1.03, 2.5, 2.5]);
    assert.deepEqual(walk(2.5, -1), [1.03, 1, 0, 0]);
    assert.equal(stepAt(p, 1.01), 1, 'and it plays');
});

test('the two-robot PREVIEW: robot two\'s own run from ITS start (the reflected one), and the pair check\'s verdict', () => {
    const ok = play('pair_ok');
    assert.ok(ok.two && ok.pair.ok && ok.pair.checked);
    assert.deepEqual(ok.two.samples[0].slice(1, 3), [60, 36], 'robot two sits at robot one\'s start reflected: (x, y) -> (-y, -x)');
    const bad = play('pair_fail');
    assert.equal(bad.pair.ok, false);
    assert.equal(bad.pair.gapIn, 6);
    assert.ok(bad.pair.failAt > 0 && bad.pair.failAt < bad.duration);
    assert.match(bad.pair.sentence, /the two robots come within 6\.0 in/);
    // the reflection holds pace by pace: robot two is robot one mirrored across y = -x
    for (let i = 0; i < bad.one.samples.length; i += 25) {
        const [, x, y] = bad.one.samples[i];
        const [, x2, y2] = bad.two.samples[i];
        assert.ok(Math.abs(x2 + y) < 1e-6 && Math.abs(y2 + x) < 1e-6, `pace ${i}`);
    }
});

test('routeKey: the same route in any key order is one key; any number changed is another', () => {
    const r = fixture('ready').project.routes[0];
    const rev = (o) => Object.fromEntries(Object.entries(o).reverse());
    const shuffled = { ...rev(r), start: rev(r.start), steps: r.steps.map(rev) };
    assert.equal(routeKey(shuffled), routeKey(r));
    const moved = { ...r, steps: r.steps.map((s, i) => (i === 1 ? { ...s, x: s.x + 0.001 } : s)) };
    assert.notEqual(routeKey(moved), routeKey(r));
    assert.notEqual(routeKey({ ...r, start: { ...r.start, headingDeg: 89 } }), routeKey(r));
});

test('thinPath keeps the first and last pace, and never a gap longer than its step plus one pace', () => {
    const p = play('ready');
    const path = thinPath(p.one.samples, 0.25);
    assert.equal(path[0].i, 0);
    assert.equal(path[path.length - 1].i, p.one.samples.length - 1);
    assert.ok(path.length < p.one.samples.length);
    for (let k = 1; k < path.length; ++k) {
        for (let i = path[k - 1].i + 1; i < path[k].i; ++i) {
            const s = p.one.samples[i];
            assert.ok(Math.hypot(s[1] - path[k - 1].x, s[2] - path[k - 1].y) < 0.25, `pace ${i} is within the step`);
        }
    }
});

test('each step seen from below belongs to ONE page step: [first, last) — the shared pace is the next step\'s; a start form owns its mechanism; the end action and a skipped step own nothing', () => {
    const step = (kind, s0, extra = {}) => ({ kind, s0, what: kind === 'mechanism' ? 'lift' : 'MoveToPose', endAction: false, mark: 'ok', sentence: '', placeholder: '', ...extra });
    const names = ['', 'lift', 'lift', '', ''];
    const rec = {
        pageSteps: [{ n: 1, do: 'driveTo', verdict: 'ok', first: 0, last: 10 }, { n: 2, do: 'startMove', verdict: 'ok', first: 10, last: 10 },
                    { n: 3, do: 'move', verdict: 'failedWentOn', first: 10, last: 20 }, { n: 4, do: 'driveTo', verdict: 'failed', first: 20, last: 20 },
                    { n: 5, do: 'wait', verdict: 'skipped', first: 20, last: 20 }],
        steps: [step('turn', 0), step('mechanism', 10, { placeholder: 'P' }), step('mechanism', 10, { mark: 'failed', sentence: 'M', placeholder: 'R' }),
                step('drive', 20, { mark: 'failed', sentence: 'D' }), step('brake', 20, { endAction: true, placeholder: 'Q', mark: 'failed', sentence: 'B' })],
        lines: [], samples: Array.from({ length: 21 }, (_, i) => [i / 100, 0, 0, 0]),
    };
    // the start form owns ONE mechanism step (the first to begin); the move started on the same pace owns the next;
    // a drive refused at once owns its own drive; the end action, nobody
    assert.deepEqual(ownersOf(rec, names), [0, 1, 2, 3, -1]);
    const p = playbackOf({ one: rec }, { one: names });
    assert.deepEqual(p.one.steps.map((s) => s.placeholder), ['', 'P', 'R', '', '']);
    assert.deepEqual(p.one.steps.map((s) => s.library), [[], [], ['M'], ['D'], []]);
    // a start form and a drive begun on the same pace, the drive listed first: each takes its own family
    const both = { ...rec, pageSteps: [{ n: 1, do: 'startMove', verdict: 'ok', first: 0, last: 0 }, { n: 2, do: 'driveTo', verdict: 'ok', first: 0, last: 9 }],
                   steps: [step('drive', 0), step('mechanism', 0, { placeholder: 'P' })] };
    assert.deepEqual(ownersOf(both, ['lift', '']), [1, 0]);
    // the real runs: in "carry" step 2's failed drive starts on step 1's last pace — it is step 2's alone
    const owners = ownersOf(fixture('carry').result.one, namesOf('carry'));
    assert.deepEqual(owners, [0, 1, -1]);
    assert.deepEqual(ownersOf(fixture('failed').result.one, namesOf('failed')), [0, 1, -1], 'the brake on the skipped steps\' pace is nobody\'s');
});

test('start forms on ONE pace are matched BY NAME (RPV1 lists them in the robot\'s slot order): a real run, code order opposite', () => {
    const f = fixture('two_starts');
    const rec = f.result.one;
    assert.deepEqual(rec.steps.filter((s) => s.kind === 'mechanism').map((s) => s.what), ['cascade', 'right toggle'], 'RPV1: slot order');
    const names = namesOf('two_starts');
    assert.deepEqual(names, ['right toggle', 'cascade', '', ''], 'the page\'s: startRun rightToggle, then startHome cascade');
    assert.deepEqual(ownersOf(rec, names), [1, 0, 2, -1]);
    const p = playbackOf(f.result, { one: names });
    assert.equal(p.one.steps[0].placeholder, '', 'the right toggle uses no PLACEHOLDER');
    assert.match(p.one.steps[1].placeholder, /^PLACEHOLDER numbers/, 'the cascade\'s home does');
    // two page steps starting ONE mechanism on one pace cannot be told apart: neither takes the step (nor a later span)
    const twice = { ...rec, pageSteps: [{ ...rec.pageSteps[0], do: 'startHome' }, ...rec.pageSteps.slice(1)] };
    assert.deepEqual(ownersOf(twice, ['cascade', 'cascade', '', '']), [-1, -1, 2, -1]);
    // with no names (a mechanism the robot lacks), a mechanism step goes nowhere — not to the drive begun on its pace
    assert.deepEqual(ownersOf(rec), [-1, -1, 2, -1]);
});

test('BOTH robots: robot two\'s own verdict is kept on a page step where it went otherwise; the headline is the worse', () => {
    const f = fixture('pair_ok');
    const two = f.result.two;
    const [, b, c] = two.pageSteps;
    const hit = { kind: 'THROUGH', element: 'blue Loader', s: b.first + 10, t: two.samples[b.first + 10][0], step: 2, depth: 1.5 };
    const bad = { ...two, through: true, throughAt: hit.s, field: [hit], word: 'FINISHED',
                  pageSteps: two.pageSteps.map((p) => (p === c ? { ...p, verdict: 'skipped', first: null, last: null } : p)) };
    const p = playbackOf({ ...f.result, two: bad });
    assert.deepEqual(p.one.steps.map((s) => s.state), ['ok', 'ok', 'ok'], 'robot one\'s own run is untouched');
    assert.equal(p.one.steps[0].two, undefined, 'the same on both: nothing extra');
    assert.equal(p.one.steps[1].two.words, 'Went THROUGH the blue Loader');
    assert.match(p.one.steps[1].two.library[0], /^On the robot it hits the blue Loader at \d+\.\d\d s, in step 2/);
    assert.equal(p.one.steps[2].two.state, 'skipped');
    assert.deepEqual(p.headline, { word: 'FINISHED', mark: 'failed', two: 'FINISHED, but THROUGH the blue Loader' });
    const same = playbackOf(f.result);
    assert.deepEqual(same.headline, { word: 'FINISHED', mark: 'ok', two: '' });
});

test('the recorder (fixtures/record.mjs) makes every fixture\'s route exactly, and marks the prototype-made pairs', () => {
    const names = fs.readdirSync(DIR).filter((f) => f.endsWith('.preview.json')).map((f) => f.replace('.preview.json', ''));
    assert.deepEqual(names.sort(), Object.keys(ROUTES).sort(), 'one route per fixture');
    for (const n of names) {
        const f = fixture(n);
        assert.deepEqual(f.project, JSON.parse(projectOf(n)), n);
        assert.equal(/PROTOTYPE of the runner's pair mode/.test(f.madeBy), PAIR.has(n), `${n}: ${f.madeBy}`);
        assert.equal(!!f.result.two, PAIR.has(n));
    }
});
