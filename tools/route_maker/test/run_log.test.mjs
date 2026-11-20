// run_log.test.mjs — RMP1-K D2: the brain's own lines read into how each step of a run went (run_log.mjs), over the
// REAL 11-19 lines (test/fixtures/robot_log) and the API 2.11 per-step line as routine_step_line_test.cpp pins it.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { newWatch, readLine, summaryWords, runTone, pageMarks } from '../src/model/run_log.mjs';

const LOGS = path.join(path.dirname(fileURLToPath(import.meta.url)), 'fixtures', 'robot_log');
const read = (name) => fs.readFileSync(path.join(LOGS, name), 'latin1').split('\n')
    .map((l) => Buffer.from(l, 'latin1').toString('utf8'));  // the bytes as pros printed them (\r, NULs, UTF-8)
const feed = (lines, w = newWatch()) => lines.reduce(readLine, w);

test('D2: the real 11-19 run 3 -- stopped at step 2, the rest skipped, "1 of 23", the END estimate, robot one', () => {
    const w = feed(read('robot_one_2026-11-19_run3.log'));
    assert.equal(w.robot, 'one', 'the boot line\'s variant');
    assert.equal(w.runs.length, 1);
    const run = w.runs[0];
    assert.equal(run.number, 3);
    assert.equal(run.name, 'skills');
    assert.equal(run.live, false, 'a build before API 2.11: no per-step line, so read at the end');
    assert.deepEqual(run.steps[2], { state: 'stopped', verb: 'driveTo', words: 'motion TIMEOUT' });
    for (let n = 3; n <= 23; ++n) {
        assert.equal(run.steps[n].state, 'skipped', `step ${n}`);
    }
    assert.equal(run.steps[1].state, 'done', 'step 1 done: read from "1 of 23 steps done" before the stop');
    assert.equal(run.steps[24], undefined);
    assert.deepEqual(run.summary, { word: 'STOPPED', done: 1, total: 23, stoppedVerb: 'driveTo', why: 'motion TIMEOUT', failed: 0, jumped: 0 });
    assert.equal(summaryWords(run), '1 of 23 steps done — stopped at step 2, driveTo: motion TIMEOUT');
    assert.deepEqual(run.end, { x: 52.51, y: -13.32, headingDeg: -117.4, live: true });
    assert.equal(run.verdict, 'ROUTINE STOPPED', 'the robot\'s own verdict line');
    assert.equal(run.endAction, 'succeeded');
    assert.equal(runTone(run), 'bad');
});

test('D2: the real run 1, cut by the USB mid-run -- step 7 failed and carried on (amber), no end, nothing invented', () => {
    const w = feed(read('robot_one_2026-11-19_run1_cut.log'));
    const run = w.runs[0];
    assert.equal(run.number, 1);
    assert.deepEqual(run.steps[7], { state: 'failed', verb: 'driveTo', words: 'motion TIMEOUT' });
    assert.deepEqual(Object.keys(run.steps), ['7'], 'no step marked done without the robot saying so');
    assert.equal(run.summary, null);
    assert.equal(summaryWords(run), null);
    assert.equal(run.end, null);
});

test('D2: API 2.11\'s per-step line ticks each step LIVE, with its seconds and a mechanism\'s end', () => {
    const w = feed([
        '[ATS] booted: shulib AUTON TESTER build [shulib-robot-variant=tank-autontest] -- runs ONE of the team\'s autons',
        '[t=  12.00] [ATS] run 1: \'skills mechs\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown, the abort armed',
        '[t=  15.10] [RTN] routine \'skills mechs\': step 1 (cascade) done in 1.08 s, at 0.0 in\r',
        '[t=  16.52] [RTN] routine \'skills mechs\': step 2 (cascade) done in 1.42 s, at 18.0 in',
        '[t=  16.83] [RTN] routine \'skills mechs\': step 3 (claw) done in 0.30 s, at \'closed\' (UNCONFIRMED)',
        '[t=  16.83] [RTN] routine \'skills mechs\': step 4 (right toggle) started in 0.00 s',
    ]);
    assert.equal(w.robot, 'two');
    const run = w.runs[0];
    assert.equal(run.live, true);
    assert.equal(run.name, 'skills mechs');
    assert.deepEqual(run.steps[2], { state: 'done', verb: 'cascade', words: 'done', seconds: 1.42, at: '18.0 in', outcome: null });
    assert.deepEqual(run.steps[3], { state: 'done', verb: 'claw', words: 'done', seconds: 0.3, at: '\'closed\'', outcome: 'UNCONFIRMED' });
    assert.equal(run.steps[4].words, 'started');
    assert.equal(run.steps[4].state, 'started', 'a start is not a finished step: never done, never a ✓');
    assert.equal(summaryWords(run), null, 'no summary before the run says it ended');
    assert.equal(runTone(run), '', 'no colour before the robot says how it ended');
    readLine(w, '[t=  18.00] [WARN][RTN] routine \'skills mechs\': step 5 (claw) FAILED: mechanism step FAILED (STALLED) — carrying on; drive safed');
    readLine(w, '[t=  19.00] [ATS] run 1: routine DONE, 5 of 6 steps done, 1 failed, 0 jumped; faults this run: none');
    assert.equal(run.steps[5].state, 'failed');
    assert.equal(run.steps[6].state, 'done', 'the last, unticked step: done by the count');
    assert.equal(summaryWords(run), '5 of 6 steps done — 1 failed and carried on');
    assert.equal(runTone(run), 'warn');
});

test('D2: garbled, partial and stray lines change nothing; never half-read', () => {
    const w = feed([
        '[RTN] routine \'skills\': step 1 (face) done in 0.84 s',  // before any run: ignored
        '[t=   1.00] [ATS] run 2: \'skills\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown',
        '[t=   2.00] [RTN] routine \'skills\': step 3 (reverseTo) skip',  // cut mid-word
        '[t=   2.\0\0 [RTN] routine \'skil',
        '[t=   2.50] [RTN] routine \'left 1\': step 2 (driveTo) done in 0.84 s',  // another routine's line, a step untouched
        '\x1b[31m[t=   3.00] [WARN][RTN] routine \'skills\' STOPPED at step 1 (face): turn TIMEOUT — skipping the rest; drive safed\x1b[0m',
        '[t=   3.10] [ATS] run 9: routine OK, 9 of 9 steps done; faults this run: none',  // another run's number
        '[t=   3.20] [RTN] routine \'skills\': step x (face) done in 0.84 s',
        '[t=   3.30] [RTN] routine \'skills\': step 2 (driveTo) done in abc s',
    ]);
    const run = w.runs[0];
    assert.deepEqual(Object.keys(run.steps), ['1']);
    assert.equal(run.steps[1].state, 'stopped', 'its colour codes read through');
    assert.equal(run.summary, null);
    readLine(w, '[t=   4.00] [RTN] routine \'skills\': step 1 (face) done in 0.84 s');
    assert.equal(run.steps[1].state, 'stopped', 'the robot\'s own failure is never painted over');
});

test('D2: a stop line lost (the count disagrees): nothing painted done; the summary in the robot\'s words, a spaced name too', () => {
    const w = feed([
        '[t=   1.00] [ATS] run 4: \'skills\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown',
        '[t=   9.00] [ATS] run 4: routine STOPPED, 1 of 5 steps done, stopped at right toggle (mechanism step FAILED); faults this run: 1',
    ]);
    const run = w.runs[0];
    assert.deepEqual(run.steps, {}, '5 unmarked steps are not "1 done": none painted');
    assert.deepEqual(run.summary, { word: 'STOPPED', done: 1, total: 5, stoppedVerb: 'right toggle', why: 'mechanism step FAILED', failed: 0, jumped: 0 });
    assert.equal(summaryWords(run), '1 of 5 steps done — stopped at right toggle: mechanism step FAILED');
    assert.equal(runTone(run), 'bad', 'red by the robot\'s own word, its stop line lost or not');
});

test('D2: page marks -- a turned-first drive is two of the brain\'s steps; worst wins; done only when every call is', () => {
    const run = { steps: { 1: { state: 'done' }, 2: { state: 'done' }, 3: { state: 'done' }, 4: { state: 'stopped', words: 'x' },
                           5: { state: 'skipped' }, 6: { state: 'skipped' } } };
    const m = pageMarks(run, [2, 1, 2, 1]);
    assert.deepEqual(m.map((x) => x && x.state), ['done', 'done', 'stopped', 'skipped']);
    assert.deepEqual(m.map((x) => x.lib), [1, 3, 4, 6], 'each page step\'s first number in the brain\'s log');
    const half = pageMarks({ steps: { 1: { state: 'done' } } }, [2]);
    assert.deepEqual(half, [null], 'the turn done, the drive not yet: not done');
    assert.equal(pageMarks({ steps: { 1: { state: 'failed' }, 2: { state: 'done' } } }, [2])[0].state, 'failed', 'the turn failed and carried on: amber, not green');
    assert.equal(pageMarks({ steps: { 1: { state: 'done' }, 2: { state: 'started' } } }, [2])[0].state, 'started', 'started beats done');
    assert.equal(pageMarks({ steps: { 1: { state: 'started' }, 2: { state: 'skipped' } } }, [2])[0].state, 'skipped');
});

test('D2: a run of ANOTHER version of the route is said, never painted on this one', async () => {
    const { routeVerbs, runDiffers } = await import('../src/model/run_log.mjs');
    const robot = { mechanisms: [{ ident: 'cascade', name: 'cascade' }, { ident: 'claw', name: 'claw' }] };
    const route = { steps: [{ do: 'driveTo', x: 0, y: 0, faceFirst: true }, { do: 'reverseTo', x: 1, y: 1 }, { do: 'move', mechanism: 'cascade' },
                            { do: 'set', mechanism: 'claw' }, { do: 'wait', seconds: 0.5 }, { do: 'forward', inches: 6 }, { do: 'backward', inches: 6 }] };
    const verbs = routeVerbs(route, robot);
    assert.deepEqual(verbs, ['face', 'driveTo', 'reverseTo', 'cascade', 'claw', 'wait 0.5 s', 'driveTo', 'reverseTo']);
    const run = (steps, summary = null) => ({ steps, summary });
    assert.equal(runDiffers(run({ 2: { verb: 'driveTo' }, 4: { verb: 'cascade' }, 1: { verb: '' } }), verbs), null);
    assert.match(runDiffers(run({ 2: { verb: 'cascade' } }), verbs), /its step 2 is cascade, here it is driveTo/);
    assert.match(runDiffers(run({ 9: { verb: 'face' } }), verbs), /its step 9 is face, here it is not there/);
    assert.match(runDiffers(run({}, { total: 23 }), verbs), /23 steps; this one has 8/);
    const real = newWatch();
    read('robot_one_2026-11-19_run3.log').forEach((l) => readLine(real, l));
    assert.match(runDiffers(real.runs[0], verbs), /another version/, 'the real 23-step run is not this 8-step route');
});

const START = (n, name) => `[t=   1.00] [ATS] run ${n}: '${name}' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown`;

test('D2 (review): the colour is the robot\'s own verdict first — a lost or garbled line never turns a stopped run green', () => {
    const tone = (lines) => {
        const run = feed([START(1, 'x'), ...lines]).runs[0];
        return [runTone(run), summaryWords(run)];
    };
    assert.deepEqual(tone(["[ATS] run 1: 'x' ROUTINE STOPPED -- ROUTINE_RAN; in time, end action succeeded",
                           '[ATS] run 1: routine STOPPED, 1 of 3 steps done, stopped at cascade (mechanism step FAILED); faults this run: 1']),
                     ['bad', '1 of 3 steps done — stopped at cascade: mechanism step FAILED'], 'its STOPPED line lost');
    assert.deepEqual(tone(["[ATS] run 1: 'x' DONE, 1 FAILED -- ROUTINE_RAN; in time, end action succeeded",
                           '[ATS] run 1: routine DONE, 2 of 3 steps done, 1 failed, 0 jumped; faults this run: 1']),
                     ['warn', '2 of 3 steps done — 1 failed and carried on'], 'its FAILED line lost: still amber, the count said');
    assert.deepEqual(tone(["[ATS] run 1: 'x' ROUTINE STOPPED -- ROUTINE_RAN; in time, end action FAILED",
                           '[ATS] run 1: routine OK, 3 of 3 steps done; faults this run: none']),
                     ['bad', '3 of 3 steps done — the end action FAILED'], 'every step done, the end action failed');
    assert.deepEqual(tone(["[ATS] run 1: 'x' ROUTINE STOPPED -- ROUTINE_RAN; in time, end action succeeded",
                           '[ATS] run 1: routine STOPPED, 3 of 5 steps done, 1 failed, 1 jumped; faults this run: 1']),
                     ['bad', '3 of 5 steps done — the robot says STOPPED — 1 failed'], 'a skip that never landed: red, never "carried on"');
    assert.deepEqual(tone(["[ATS] run 1: 'x' HARD STOP -- ROUTINE_RAN; cut at the deadline, end action succeeded, HARD STOP fired",
                           '[ATS] run 1: routine OK, 3 of 3 steps done; faults this run: none']),
                     ['bad', '3 of 3 steps done — the robot says HARD STOP']);
    assert.deepEqual(tone(["[ATS] run 1: 'x' ABORTED in the countdown by B -- runAuton was never called; nothing moved"]),
                     ['bad', 'The robot says ABORTED'], 'a run that never got to its count still ends');
    assert.deepEqual(tone(["[ATS] run 1: 'x' FINISHED -- ROUTINE_RAN; in time, end action succeeded",
                           '[ATS] run 1: routine OK, 3 of 3 steps done; faults this run: none']), ['ok', '3 of 3 steps done']);
    const garbled = feed([START(1, 'x'), "[WARN][RTN] routine 'x' STOPPED at step 2 (cascade): mechanism step FAILED (STALLED) â€” skipping the rest; drive safed",
                          "[RTN] routine 'x': step 3 (claw) skipped ��� stopped at step 2"]).runs[0];
    assert.deepEqual([garbled.steps[2], garbled.steps[3].state], [{ state: 'stopped', verb: 'cascade', words: 'mechanism step FAILED (STALLED)' }, 'skipped'],
                     'an em dash in pieces still reads');
});

test('D2 (review): run numbers start at 1 at every boot — a new boot\'s run 1 gets its own summary and END', () => {
    const w = feed([START(1, 'skills'), "[ATS] run 1: 'skills' FINISHED -- ROUTINE_RAN; in time, end action succeeded",
                    '[ATS] run 1: routine OK, 2 of 2 steps done; faults this run: none',
                    '[ATS] run 1: seated (0.00, 0.00) facing 0.0 deg; END estimate (1.00, 2.00) facing 3.0 deg LIVE -- measure the tape',
                    '[ATS] booted: shulib AUTON TESTER build [shulib-robot-variant=one-autontest] -- runs ONE of the team\'s autons',
                    START(1, 'skills mechs'), "[RTN] routine 'skills mechs': step 1 (cascade) done in 1.00 s, at 6.0 in",
                    "[ATS] run 1: 'skills mechs' DONE, 1 FAILED -- ROUTINE_RAN; in time, end action succeeded",
                    '[ATS] run 1: routine DONE, 1 of 2 steps done, 1 failed, 0 jumped; faults this run: 1',
                    '[ATS] run 1: seated (0.00, 0.00) facing 0.0 deg; END estimate (4.00, 5.00) facing 6.0 deg NOT LIVE -- measure the tape']);
    assert.equal(w.runs.length, 2);
    const [first, second] = w.runs;
    assert.deepEqual([first.summary.done, first.end.x, first.verdict], [2, 1, 'FINISHED'], 'the first boot\'s run kept as it was');
    assert.deepEqual([second.summary.done, second.summary.failed, second.end.x, second.verdict], [1, 1, 4, 'DONE, 1 FAILED']);
    assert.equal(second.end.live, false, 'NOT LIVE read as such (the page does not draw it)');
    assert.equal(runTone(second), 'warn');
});

test('D2 (review): a started mechanism\'s own end lands on the step that started it', () => {
    const w = feed([START(1, 'x'), "[RTN] routine 'x': step 1 (intake) started in 0.00 s", "[RTN] routine 'x': step 2 (arm) started in 0.00 s",
                    "[WARN][MECH] 'intake': CANCELLED in 0.31s (replaced by a newer command)",
                    "[MECH] 'arm': SUCCEEDED -- reached in 1.20s", "[MECH] 'arm': SUCCEEDED -- reached in 9.99s",
                    "[RTN] routine 'x': step 3 (intake) started in 0.00 s", "[MECH] 'intake': current cap 2.00 A per motor -- set and read back"]);
    const run = w.runs[0];
    assert.deepEqual([run.steps[1].state, run.steps[1].words, run.steps[1].mech], ['failed', 'started, then CANCELLED in 0.31 s (replaced by a newer command)', true]);
    assert.deepEqual([run.steps[2].state, run.steps[2].words], ['done', 'started, then SUCCEEDED in 1.20 s'], 'the first end only');
    assert.equal(run.steps[3].state, 'started', 'a line that is not an outcome changes nothing');
    readLine(w, "[ATS] run 1: 'x' FINISHED -- ROUTINE_RAN; in time, end action succeeded");
    readLine(w, '[ATS] run 1: routine OK, 3 of 3 steps done; faults this run: none');
    assert.equal(runTone(run), 'ok', 'a mechanism cut is the card\'s, not the routine\'s verdict');
    assert.equal(summaryWords(run), '3 of 3 steps done');
});

test('D2 (review): the stop bounds the fill — a run whose skipped lines were lost still fills the steps before its stop', () => {
    const run = feed([START(1, 'x'), "[WARN][RTN] routine 'x' STOPPED at step 2 (driveTo): motion TIMEOUT — skipping the rest; drive safed",
                      '[ATS] run 1: routine STOPPED, 1 of 4 steps done, stopped at driveTo (motion TIMEOUT); faults this run: 1']).runs[0];
    assert.deepEqual(Object.fromEntries(Object.entries(run.steps).map(([k, v]) => [k, v.state])), { 1: 'done', 2: 'stopped' });
});

test('D2 (review): the library\'s own per-step lines (d4_lines.log, which routine_step_line_test.cpp checks the library prints)', () => {
    const run = feed(read('d4_lines.log')).runs[0];
    assert.deepEqual(run.steps[1], { state: 'done', verb: 'claw', words: 'done', seconds: 0.51, at: '\'open\'', outcome: null });
    assert.deepEqual(run.steps[2], { state: 'done', verb: 'claw', words: 'done', seconds: 0.51, at: '\'closed\'', outcome: 'UNCONFIRMED' });
});
