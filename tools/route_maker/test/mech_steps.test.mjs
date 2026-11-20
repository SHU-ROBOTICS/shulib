// mech_steps.test.mjs — RMP1-J's rules (src/model/mech_steps.mjs; playback.mjs's measured times). Adversarial: every
// golden's mechanism steps REBUILT through the new menu and height control must export the committed C++ byte for byte.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fixtureRobot, ledgerRobot, GOLDENS } from './helpers.mjs';
import { fixture } from './standin_engine.mjs';
import { menuRows, menuActions, heightPatch, barHeight, barPick, asHeight, waitsFor, WAIT_PAIR, suggestS, limitHint, measureEmptyLimits,
         homeCard, limitOf, limitWords } from '../src/model/mech_steps.mjs';
import { badgeSpots, badgeWord } from '../src/ui/mech_badges.mjs';
import { mechanismActions } from '../src/ui/words.mjs';
import { noLevels, applyFix, checkRoute } from '../src/model/checks.mjs';
import { addStep, updateStep, normalizeRoute, newRoute, plannedPoses, MECHANISM_KINDS } from '../src/model/route.mjs';
import { loadProject } from '../src/model/project.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { targetKind } from '../src/model/lift.mjs';
import { playbackOf, ownersOf, tookOf } from '../src/model/playback.mjs';
import { stepPoses } from '../src/model/pair.mjs';

const X = fixtureRobot('x');
const FIELD = { wallInnerFaceIn: 70.25 };
const route = (steps, start = { x: -36, y: -55.7, headingDeg: 0 }) => ({ ...newRoute(), robot: 'x', start, steps });
const actionsOf = (m, r, at) => menuActions(m, mechanismActions(m, !noLevels(m)), r, at);
const ONE = { name: 'robot one', mechanisms: [{ name: 'cascade', ident: 'cascade', kind: 'lift', units: 'in', homing: 'stall', homeAt: 0, range: [0, 20], presets: [] },
                                              { name: 'right toggle', ident: 'rightToggle', kind: 'power' },
                                              { name: 'claw', ident: 'claw', kind: 'pneumatic', positions: ['closed', 'open'] },
                                              { name: 'ramp', ident: 'ramp', kind: 'pneumatic', positions: ['up', 'down'] }] };

// ── D1: the menu's two steps ─────────────────────────────────────────────────────────────────────────────────────

test('D1: one row per mechanism, its kind as an icon; then each kind of step ONCE -- nothing today\'s menu offers is lost', () => {
    assert.deepEqual(menuRows(ONE).map((r) => [r.name, r.icon]), [['Cascade', 'lift'], ['Right toggle', 'motor'], ['Claw', 'piston'], ['Ramp', 'piston']]);
    const started = (m) => route([{ do: m.kind === 'power' ? 'startRun' : 'startMove', mechanism: m.ident }]);
    for (const robot of [X, ledgerRobot(), ONE]) {
        for (const m of robot.mechanisms) {
            const today = mechanismActions(m, !noLevels(m));
            const now = actionsOf(m, started(m), 1);
            const keys = now.map((a) => (a.step.do === 'set' ? `set:${a.step.position}` : a.step.do));
            assert.equal(new Set(keys).size, keys.length, `${m.ident}: no step offered twice: ${keys}`);
            for (const a of today) {
                const k = a.step.do === 'set' ? `set:${a.step.position}` : a.step.do;
                assert.ok(keys.includes(k), `${m.ident}: today's "${a.words}" is still offered (${k})`);
            }
            assert.equal(now.length, new Set(today.map((a) => (a.step.do === 'set' ? `set:${a.step.position}` : a.step.do))).size);
            const move = now.find((a) => a.step.do === 'move');
            if (move) {
                assert.deepEqual(move.step, { mechanism: m.ident, do: 'move' }, 'no height invented: the height control asks');
                assert.equal(move.words, `Move the ${m.name}`);
            }
        }
    }
    const lift = X.mechanisms.find((m) => m.ident === 'lift');
    assert.deepEqual(actionsOf(lift, route([]), 0).map((a) => [a.words, a.icon]), [['Move the lift', 'lift'], ['Home it', 'home']], 'no wait: nothing started it; the move wears its kind\'s icon');
    assert.deepEqual(actionsOf(X.mechanisms.find((m) => m.ident === 'hood'), route([]), 0).map((a) => a.icon), ['piston', 'piston', 'piston']);
    // "Wait for it" only once a step BEFORE the new one starts THIS mechanism without waiting
    const r = route([{ do: 'startRun', mechanism: 'intake', volts: 6, forS: 1 }, { do: 'move', mechanism: 'lift', to: 4, boundS: 2 },
                     { do: 'startMove', mechanism: 'lift', to: 8, boundS: 2 }]);
    assert.ok(!actionsOf(lift, r, 2).some((a) => a.step.do === 'waitUntilDone'), 'the start is AFTER the new step');
    assert.ok(!actionsOf(lift, r, 1).some((a) => a.step.do === 'waitUntilDone'), 'a move that waits is no start; the intake is another');
    assert.ok(actionsOf(lift, r, 3).some((a) => a.step.do === 'waitUntilDone'));
    assert.deepEqual(actionsOf(X.mechanisms.find((m) => m.ident === 'flywheel'), r, 3), [], 'a flywheel: none yet (its row says so)');
});

// ── D2: one height writes today's step ───────────────────────────────────────────────────────────────────────────

/** A golden's route made again through the page's new controls: the menu's step, the height control's patch (an old
 *  project's up / down typed in its own field), "keep driving", the numbers typed. */
function rebuild(r, robot) {
    let out = { ...r, steps: [] };
    r.steps.forEach((s, i) => {
        if (!MECHANISM_KINDS.has(s.do)) {
            out = addStep(out, s, i);
            return;
        }
        const m = robot.mechanisms.find((x) => x.ident === s.mechanism);
        const base = { startMove: 'move', startHome: 'home', startRun: 'run' }[s.do] || s.do;
        const a = actionsOf(m, out, i).find((x) => x.step.do === base && (base !== 'set' || x.step.position === s.position));
        assert.ok(a, `the menu offers step ${i + 1}'s ${s.do} on the ${m.ident}`);
        out = addStep(out, a.step, i);
        const tk = base === 'move' ? targetKind(s) : null;
        const patch = tk === 'preset' ? heightPatch({ level: m.presets.find((p) => (p.ledger ?? p.name) === s.preset) })
            : tk === 'bottom' ? heightPatch({ bottom: true }) : tk === 'to' ? heightPatch({ to: s.to }) : tk ? { [tk]: s[tk] } : {};
        if (s.do !== base) {
            patch.do = WAIT_PAIR[base];  // "Keep driving while it moves"
        }
        for (const k of ['label', 'volts', 'forS', 'boundS', 'care']) {
            if (s[k] !== undefined) {
                patch[k] = s[k];
            }
        }
        out = updateStep(out, i, patch);
    });
    return normalizeRoute(out);
}

test('D2 golden: every x-robot golden with a mechanism step, rebuilt through the new menu and height control, exports the committed C++ byte for byte', () => {
    const names = fs.readdirSync(GOLDENS).filter((n) => n.endsWith('.route.json') && /"(move|startMove|home|startHome|set|run|startRun|waitUntilDone)"/.test(fs.readFileSync(path.join(GOLDENS, n), 'utf8')));
    let n = 0;
    for (const name of names) {
        const p = loadProject(fs.readFileSync(path.join(GOLDENS, name), 'utf8')).project;
        if (p.routes[0].robot !== 'x') {
            continue;
        }
        const again = p.routes.map((r) => rebuild(r, X));
        assert.deepEqual(again, p.routes, `${name}: the same steps`);
        assert.equal(`${exportRegion(again, X)}\n`, fs.readFileSync(path.join(GOLDENS, name.replace('.route.json', '.cpp')), 'utf8'), name);
        ++n;
    }
    assert.ok(n >= 7, `only ${n} goldens rebuilt`);
});

test('D2: the bar picks 0.5 in (1 deg) inside the travel, never past it; each pick is today\'s step kind', () => {
    const lift = { kind: 'lift', range: [0.2, 20] };
    assert.equal(barHeight(lift, 0.5), 10);
    assert.equal(barHeight(lift, 0), 0.2, 'never below the bottom (0 rounds below 0.2)');
    assert.equal(barHeight(lift, 1), 20);
    assert.equal(barHeight(lift, 1.4), 20);
    assert.equal(barHeight(lift, 0.26), 5.5);
    assert.equal(barHeight({ kind: 'arm', range: [0, 160] }, 0.333), 53);
    assert.deepEqual(normalizeRoute(updateStep(route([{ do: 'move', mechanism: 'lift', up: 6 }]), 0, heightPatch({ to: 7.25 }))).steps[0],
                     { do: 'move', mechanism: 'lift', to: 7.25 }, 'a height replaces the up');
    assert.deepEqual(normalizeRoute(updateStep(route([{ do: 'move', mechanism: 'lift', to: 6 }]), 0, heightPatch({ bottom: true }))).steps[0],
                     { do: 'move', mechanism: 'lift', bottom: true });
    assert.deepEqual(normalizeRoute(updateStep(route([{ do: 'move', mechanism: 'cascade', bottom: true }]), 0, heightPatch({ level: { ledger: 0, name: 'floor' } }))).steps[0],
                     { do: 'move', mechanism: 'cascade', preset: 0 }, 'a facts level by its index (0 too)');
    assert.deepEqual(normalizeRoute(updateStep(route([{ do: 'move', mechanism: 'lift', to: 6 }]), 0, heightPatch({ to: undefined }))).steps[0],
                     { do: 'move', mechanism: 'lift', to: null }, 'an emptied height is a hole the checks ask for');
    assert.equal(heightPatch({ to: 0.1 + 0.2 }).to, 0.3, 'on the 0.001 grid (an old "up" made a height adds)');
});

test('D2: a click on a measured level\'s mark picks the LEVEL (its preset), anywhere else a height; an owed level is no mark', () => {
    const lift = X.mechanisms.find((m) => m.ident === 'lift');  // 0-24 in; floor 0, low 8, high 22
    const f = (v) => v / 24;
    assert.deepEqual(barPick(lift, f(8)), { level: lift.presets[1] });
    assert.deepEqual(barPick(lift, f(8.6)), { level: lift.presets[1] }, 'within 3% of the travel (0.72 in)');
    assert.deepEqual(barPick(lift, f(8.8)), { to: 9 }, 'past it: a height');
    assert.deepEqual(barPick(lift, f(21.4)), { level: lift.presets[2] });
    assert.deepEqual(barPick(lift, f(12)), { to: 12 });
    assert.deepEqual(barPick({ ...lift, presets: lift.presets.map((p) => ({ ...p, owed: true })) }, f(8)), { to: 8 }, 'an owed level has no mark');
    const r = normalizeRoute(updateStep(route([{ do: 'move', mechanism: 'lift' }]), 0, heightPatch(barPick(lift, f(22)))));
    assert.deepEqual(r.steps[0], { do: 'move', mechanism: 'lift', preset: 'high' });
    assert.match(exportRegion([{ ...r, steps: [{ do: 'home', mechanism: 'lift', boundS: 2 }, { ...r.steps[0], boundS: 2 }] }], X), /lift\[/, 'the export names the level');
});

test('D2: an old project\'s up / down open, edit and export unchanged; "make it a height" keeps the code, only its comment says "to"', () => {
    const text = fs.readFileSync(path.join(GOLDENS, 'gl21_lift_heights.route.json'), 'utf8');
    const golden = fs.readFileSync(path.join(GOLDENS, 'gl21_lift_heights.cpp'), 'utf8');
    const r = loadProject(text).project.routes[0];
    const edited = r.steps.reduce((acc, s, i) => (s.up !== undefined || s.down !== undefined ? updateStep(acc, i, { [s.up !== undefined ? 'up' : 'down']: s.up ?? s.down }) : acc), r);
    assert.equal(`${exportRegion([edited], X)}\n`, golden);
    const made = [1, 2, 3, 7, 9].reduce((acc, i) => updateStep(acc, i, asHeight(acc, X, i)), r);
    assert.deepEqual([1, 2, 3, 7, 9].map((i) => made.steps[i].to), [6, 16.5, 12, 30, 80]);
    const code = (t) => t.split('\n').filter((l) => !/^\s*\/\//.test(l));
    assert.deepEqual(code(`${exportRegion([made], X)}\n`), code(golden), 'the same C++, line for line');
    assert.notEqual(`${exportRegion([made], X)}\n`, golden, 'its words say "to" now');
    assert.equal(asHeight(route([{ do: 'move', mechanism: 'lift', up: 6, boundS: 2 }]), X, 0), null, 'up from an unknown height: no height to make');
});

// ── D3: where the route waits ────────────────────────────────────────────────────────────────────────────────────

test('D3: a started step -- the wait that collects it, the step that needs it first, or nothing', () => {
    const r = route([{ do: 'startMove', mechanism: 'lift', to: 4 }, { do: 'run', mechanism: 'intake' }, { do: 'waitUntilDone', mechanism: 'lift' },
                     { do: 'startRun', mechanism: 'intake' }, { do: 'move', mechanism: 'arm', to: 9 }, { do: 'run', mechanism: 'intake' },
                     { do: 'startHome', mechanism: 'arm' }]);
    assert.deepEqual(waitsFor(r, 0), { wait: 2 }, 'another mechanism in between does not count');
    assert.deepEqual(waitsFor(r, 3), { needed: 5 });
    assert.deepEqual(waitsFor(r, 6), {});
    const end = applyFix(r, { kind: 'waitForHome', stepIndex: r.steps.length, mechanism: 'arm' }, addStep, updateStep);
    assert.deepEqual(end.steps.at(-1), { do: 'waitUntilDone', mechanism: 'arm' }, '"Wait for it at the end": the last step');
    assert.deepEqual(waitsFor(end, 6), { wait: 7 });
    assert.deepEqual(applyFix(r, { kind: 'waitForHome', stepIndex: 5, mechanism: 'intake' }, addStep, updateStep).steps[5],
                     { do: 'waitUntilDone', mechanism: 'intake' }, 'its fix: a wait before the step that needs it');
});

// ── D4: the time limit from PREVIEW ──────────────────────────────────────────────────────────────────────────────

test('D4: PREVIEW\'s time + 0.5 s, up to the next 0.1 s (never down)', () => {
    for (const [took, want] of [[0.9, 1.4], [0.78, 1.3], [0.8, 1.3], [1.1999999999999749, 1.7], [0, 0.5], [0.90001, 1.5], [2.5, 3]]) {
        assert.equal(suggestS(took), want, String(took));
    }
});

const NAMES = { mechanisms: ['cascade', 'cascade', 'cascade', 'cascade', 'claw', 'right toggle'], ready: ['cascade', '', '', 'claw', ''],
                failed: ['claw', '', '', ''], two_starts: ['right toggle', 'cascade', '', ''], through: ['', ''], threw: ['', 'claw', ''],
                waits: ['cascade', 'cascade', '', 'cascade', 'cascade'], two_moves: ['cascade', 'cascade', 'cascade', '', 'cascade'],
                carry: ['', '', 'claw', ''] };
const runOf = (name) => {
    const f = fixture(name);
    return { route: f.route, play: playbackOf(f.result, { one: NAMES[name] }) };
};

test('D4: the library\'s own time for the step -- a height move owns its mechanism step (its PLACEHOLDER too), a drive turned first is timed by its drive, a wait by its span', () => {
    const mech = runOf('mechanisms');
    assert.deepEqual(ownersOf(fixture('mechanisms').result.one, NAMES.mechanisms).slice(0, 6), [0, 1, 2, 3, 4, 5], 'moveTo steps own theirs');
    assert.deepEqual(mech.play.one.steps.map((s) => !!s.placeholder), [true, true, true, true, false, false]);
    const hint = limitHint(mech, mech.route, 1);
    assert.deepEqual({ ...hint, took: Math.round(hint.took * 1000) / 1000 }, { took: 0.78, s: 1.3, placeholder: true });
    const ready = runOf('ready');
    const drive = limitHint(ready, ready.route, 1);
    assert.equal(Math.round(drive.took * 100) / 100, 1.58, 'the drive alone, not its turn first (2.47 s)');
    assert.equal(limitOf(ready.route.steps[1]), 'timeoutS');
    const two = runOf('two_starts');
    assert.equal(Math.round(limitHint(two, two.route, 1).took * 100) / 100, 0.32, 'a started home: its own run, not its zero-length step');
    assert.equal(limitHint(two, two.route, 0), null, 'a run has no time limit');
    // a wait: its own span (the move it collects outlasted the drive) -- and one that threw is no number
    const waits = runOf('waits');
    const w = waits.play.one.steps[3];
    assert.equal(Math.round(limitHint(waits, waits.route, 3).took * 100) / 100, Math.round((w.t1 - w.t0) * 100) / 100);
    assert.ok(w.t1 - w.t0 > 0.5, 'a real wait');
    const threw = runOf('threw');
    assert.deepEqual(limitHint(threw, threw.route, 1), { notRun: 'The program stopped here' }, 'a wait that threw: no number, and not "its limit stopped it"');
});

test('D4: a start owns its time only through its own library step -- none (two on one pace), or one the routine\'s end cancelled: no number', () => {
    const t = (pace) => pace / 100;
    assert.equal(tookOf({ do: 'startMoveTo', first: 5, last: 5, verdict: 'ok' }, [], t), null);
    assert.equal(tookOf({ do: 'startRun', first: 5, last: 5, verdict: 'ok' }, [], t), null);
    assert.deepEqual(tookOf({ do: 'waitUntilDone', first: 5, last: 75, verdict: 'ok' }, [], t), { s: 0.7, ok: true, ranOut: false, verdict: 'ok' });
    const lib = { t0: 0.05, t1: 0.3, mark: 'ok', verdict: 'SUCCEEDED' };
    assert.equal(tookOf({ do: 'waitUntilDone', first: 5, last: 75, verdict: 'ok' }, [lib], t).s, 0.7, 'a wait: its own span, whatever began in it');
    assert.deepEqual(tookOf({ do: 'moveTo', first: 5, last: 75, verdict: 'failed' }, [lib], t), { s: 0.25, ok: false, ranOut: false, verdict: 'SUCCEEDED' },
                     'the library\'s step finished but the page step failed: not ok');
    const two = runOf('two_moves');
    assert.deepEqual([1, 2].map((i) => limitHint(two, two.route, i)), [{ untimed: true }, { untimed: true }]);
    assert.equal(limitHint(two, two.route, 4).s, suggestS(two.play.one.steps[4].t1 - two.play.one.steps[4].t0), 'the wait that collects them: its span');
    const waits = runOf('waits');
    assert.deepEqual(limitHint(waits, waits.route, 4), { notRun: 'it was stopped before it was done' }, 'the last start: the routine\'s end cancels it');
    assert.equal(Math.round(limitHint(waits, waits.route, 1).took * 100) / 100, 1.62, 'a started move collected later: its library step');
});

test('D4: a wait\'s number is its mechanism\'s time after the steps before it -- PLACEHOLDER and the stand-in drive said in its line; a start PREVIEW ran but cannot time says so', () => {
    const waits = runOf('waits');  // startMove cascade (PLACEHOLDER gains), driveTo, waitUntilDone
    assert.match(waits.play.one.steps[3].placeholder, /^PLACEHOLDER numbers/, 'the wait\'s mark is its mechanism\'s');
    assert.equal(runOf('threw').play.one.steps[1].placeholder, '', 'a wait for the claw (no PLACEHOLDER): none');
    const hint = limitHint(waits, waits.route, 3);
    assert.deepEqual({ ...hint, took: Math.round(hint.took * 100) / 100 }, { took: 0.69, s: 1.2, placeholder: true, wait: true, afterDrive: true });
    assert.equal(limitWords(hint, { drive: false, current: 3, name: 'cascade' }).words,
                 'PREVIEW waited 0.69 s here on the host test plant (on PLACEHOLDER numbers, after the stand-in drive, not your robot), + 0.5 s: time it on the robot too.');
    // a hand-made run: the wait right after its start (no drive between) is the mechanism's time alone
    const own = (steps, took, t0 = steps.map((x, k) => k)) => {
        const r = route(steps);
        return { route: r, play: { one: { steps: steps.map((x, k) => ({ state: 'ok', words: 'Done', placeholder: '', t0: t0[k], took: took[k] })) } } };
    };
    const ok = (sec) => ({ s: sec, ok: true, ranOut: false, verdict: 'SUCCEEDED' });
    const near = own([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'set', mechanism: 'hood', position: 'out' }, { do: 'waitUntilDone', mechanism: 'lift' }],
                     [ok(1), ok(0.3), ok(0.4)]);
    assert.equal(limitHint(near, near.route, 2).afterDrive, false);
    assert.equal(limitWords(limitHint(near, near.route, 2), { drive: false }).words, 'PREVIEW waited 0.4 s here on the host test plant, + 0.5 s.');
    const far = own([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'turnTo', headingDeg: 90 }, { do: 'set', mechanism: 'hood', position: 'out' },
                     { do: 'waitUntilDone', mechanism: 'lift' }], [ok(1), ok(0.5), ok(0.3), ok(0.2)]);
    assert.equal(limitHint(far, far.route, 3).afterDrive, true, 'a turn since the start: the stand-in drive');
    const drove = own([{ do: 'driveTo', x: -36, y: -36 }, { do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'waitUntilDone', mechanism: 'lift' }],
                      [ok(1), ok(1), ok(0.4)]);
    assert.equal(limitHint(drove, drove.route, 2).afterDrive, false, 'a drive BEFORE the start it collects: not in its time');
    const arm = own([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'move', mechanism: 'arm', to: 30 }, { do: 'waitUntilDone', mechanism: 'lift' }],
                    [ok(1), ok(0.6), ok(0.4)]);
    arm.play.one.steps[1].placeholder = 'PLACEHOLDER numbers: arm';
    assert.equal(limitWords(limitHint(arm, arm.route, 2), { drive: false }).words, 'PREVIEW waited 0.4 s here on the host test plant (on PLACEHOLDER numbers), + 0.5 s.',
                 'a PLACEHOLDER arm move since the start: the wait\'s number is on it too');
    const armFirst = own([{ do: 'move', mechanism: 'arm', to: 30 }, { do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'waitUntilDone', mechanism: 'lift' }],
                         [ok(0.6), ok(1), ok(0.4)]);
    armFirst.play.one.steps[0].placeholder = 'PLACEHOLDER numbers: arm';
    assert.equal(limitHint(armFirst, armFirst.route, 2).placeholder, false, 'one BEFORE the start: not in its time');
    // two starts of the cascade on one pace: both ran ("Done"), neither can be timed apart -- never "did not finish"
    const two = runOf('two_moves');
    assert.deepEqual([1, 2].map((i) => [two.play.one.steps[i].state, limitHint(two, two.route, i)]), [['ok', { untimed: true }], ['ok', { untimed: true }]]);
    assert.deepEqual(limitWords({ untimed: true }, { drive: false, current: 3, name: 'cascade' }),
                     { words: 'PREVIEW ran it but cannot time it on its own (another step starts the cascade at the same moment): no suggestion.', use: null });
    const lone = own([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'startMove', mechanism: 'arm', to: 9 }], [null, ok(1)], [0.5, 0.5]);
    assert.deepEqual(limitHint(lone, lone.route, 0), { untimed: false }, 'another mechanism on its pace: not "another step starts it"');
    const later = own([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'startMove', mechanism: 'lift', to: 9 }], [null, ok(1)], [0.5, 0.9]);
    assert.deepEqual(limitHint(later, later.route, 0), { untimed: false }, 'its mechanism started again LATER: not "at the same moment"');
    assert.equal(limitWords({ untimed: false }, { drive: false }).words, 'PREVIEW ran it but cannot time it on its own: no suggestion.');
    const stalled = own([{ do: 'startMove', mechanism: 'lift', to: 8 }], [{ s: 0.6, ok: false, ranOut: false, verdict: 'STALLED' }]);
    assert.deepEqual(limitHint(stalled, stalled.route, 0), { notRun: 'the library ended it STALLED' }, 'a started step "Done" whose library step failed: never "(Done)"');
});

test('D4: no suggestion without a run, for a route changed before the step, for a step it did not finish; time limits alone change nothing', () => {
    const mech = runOf('mechanisms');
    const r = mech.route;
    assert.equal(limitHint(null, r, 1), null);
    assert.ok(limitHint(mech, updateStep(updateStep(r, 0, { boundS: 9 }), 1, { boundS: 1.3 }), 1), 'only limits changed');
    const ready = runOf('ready');  // its drive (step 2) took 1.58 s
    for (const [patch, why] of [[{ care: { timeoutS: 2.1 } }, 'a drive limit above its time'], [{ view: { hidden: true } }, 'hidden'],
                                [{ view: { locked: true } }, 'locked'], [{ touches: ['wall'] }, 'a touch on purpose']]) {
        assert.ok(limitHint(ready, updateStep(ready.route, 1, patch), 2), `${why}: the run still speaks for the next step`);
    }
    assert.equal(limitHint(ready, updateStep(ready.route, 1, { care: { timeoutS: 1.5 } }), 2), null, 'a drive limit BELOW its time: it would stop there');
    assert.ok(limitHint(ready, updateStep(ready.route, 0, { boundS: undefined }), 2), 'an emptied mechanism limit: PREVIEW runs it to the cut');
    assert.equal(limitHint(ready, updateStep(ready.route, 0, { boundS: 0.3 }), 2), null, 'the home took 0.32 s: 0.3 would stop it');
    const typed = updateStep(ready.route, 1, { care: { timeoutS: 3 } });
    assert.equal(limitHint({ ...ready, route: typed }, updateStep(typed, 1, { care: { timeoutS: undefined } }), 2), null,
                 'a drive limit emptied: the robot\'s own, not known here');
    const carry = runOf('carry');  // step 1 ran out of its 0.4 s and the route carried on
    assert.deepEqual(limitHint(carry, carry.route, 1), { ranOut: true });
    assert.equal(limitHint(carry, updateStep(carry.route, 0, { care: { timeoutS: 3 } }), 1), null, 'a step that ran out given longer: another run');
    assert.ok(limitHint(mech, updateStep(r, 3, { bottom: undefined, to: 4 }), 1), 'a LATER step changed: step 2 ran the same');
    assert.equal(limitHint(mech, updateStep(r, 0, { mechanism: 'arm' }), 1), null, 'an earlier step changed');
    assert.equal(limitHint(mech, updateStep(r, 1, { up: 7 }), 1), null, 'the step itself changed');
    assert.equal(limitHint(mech, { ...r, start: { ...r.start, x: -35 } }, 1), null, 'the start moved');
    assert.equal(limitHint(mech, { ...r, steps: r.steps.slice(0, 1) }, 1), null, 'no such step any more');
    const failed = runOf('failed');
    assert.deepEqual(limitHint(failed, failed.route, 1), { ranOut: true }, 'its 0.4 s limit stopped it: no number');
    assert.deepEqual(limitHint(failed, failed.route, 2), { notRun: 'Skipped: the route had stopped' }, 'skipped: never ran -- said, not "run PREVIEW"');
    const through = runOf('through');
    assert.equal(through.play.one.through.kind, 'THROUGH');
    assert.deepEqual(limitHint(through, through.route, 0), { notRun: `Went THROUGH the ${through.play.one.through.element}` },
                     'it went THROUGH the wall: the robot would not, so no time to suggest');
    const r2 = fixture('ready');  // a THROUGH put in its drive (step 2): the curve after it has a limit and a time of its own
    const hit = { ...r2.result, one: { ...r2.result.one, field: [{ kind: 'THROUGH', element: 'right wall', s: 200, t: 2, step: 2, depth: 1.2 }] } };
    const past = { route: r2.route, play: playbackOf(hit, { one: NAMES.ready }) };
    assert.ok(runOf('ready').play.one.steps[2].took.ok, 'without the THROUGH the curve is timed');
    assert.deepEqual([1, 2].map((i) => limitHint(past, past.route, i)),
                     [{ notRun: 'Went THROUGH the right wall' }, { notRun: 'After the THROUGH: the robot would not get here' }], 'no time from a run past a wall');
    const before = JSON.stringify(r);
    limitHint(mech, r, 1);
    assert.equal(JSON.stringify(r), before, 'a suggestion never writes');
});

test('D4: the line says where the number comes from (the host test plant; a drive: the stand-in, not your robot), why there is none, and never "run PREVIEW" after a run that did not finish the step', () => {
    const mech = { took: 0.78, s: 1.3, placeholder: true };
    assert.deepEqual(limitWords(mech, { drive: false, current: undefined, previewable: true }),
                     { words: 'PREVIEW took 0.78 s here on the host test plant (on PLACEHOLDER numbers), + 0.5 s.', use: 1.3, inUse: false });
    assert.equal(limitWords({ ...mech, placeholder: false }, { drive: false, current: 1.3 }).words, 'PREVIEW took 0.78 s here on the host test plant, + 0.5 s.');
    assert.equal(limitWords(mech, { drive: false, current: 1.3 }).inUse, true, 'clicked: "In use."');
    assert.deepEqual(limitWords({ took: 1.58, s: 2.1, placeholder: false }, { drive: true, current: undefined }),
                     { words: 'PREVIEW\'s stand-in drive (not your robot) took 1.58 s here, + 0.5 s: time it on the robot too.', use: 2.1, inUse: false });
    assert.deepEqual(limitWords({ ranOut: true }, { drive: true, current: undefined }),
                     { words: 'In PREVIEW the robot\'s own time limit stopped it: type a longer one and run PREVIEW again.', use: null }, 'never "empty it" when it is empty');
    assert.deepEqual(limitWords({ ranOut: true }, { drive: true, current: 0.4 }),
                     { words: 'In PREVIEW its time limit stopped it: give it longer and run PREVIEW again.', use: null });
    assert.equal(limitWords({ ranOut: true }, { drive: false, current: 0.4 }).words, 'In PREVIEW its time limit stopped it: give it longer and run PREVIEW again.');
    assert.deepEqual(limitWords({ notRun: 'it was stopped before it was done' }, { drive: false, current: undefined, previewable: true }),
                     { words: 'PREVIEW did not finish this step (it was stopped before it was done): no suggestion.', use: null });
    assert.deepEqual(limitWords(null, { drive: false, current: undefined, previewable: true }), { words: 'Run PREVIEW to get a suggestion.', use: null });
    assert.equal(limitWords(null, { drive: false, current: 2, previewable: true }), null, 'a typed limit: nothing to ask');
    assert.equal(limitWords(null, { drive: true, current: undefined, previewable: true }), null, 'a drive may stay empty: nothing to ask');
    assert.equal(limitWords(null, { drive: false, current: undefined, previewable: false }), null, 'no engine: no PREVIEW to ask for');
});

test('D4: PREVIEW times an EMPTY mechanism limit with none (the cut); the route keeps it empty, a typed one, a run\'s time and a drive\'s limit untouched', () => {
    const r = route([{ do: 'startHome', mechanism: 'lift' }, { do: 'waitUntilDone', mechanism: 'lift', boundS: 2 }, { do: 'run', mechanism: 'intake', volts: 6 },
                     { do: 'driveTo', x: -36, y: -36 }, { do: 'move', mechanism: 'lift', to: 4 }, { do: 'startMove', mechanism: 'lift', to: 8 },
                     { do: 'home', mechanism: 'arm' }, { do: 'waitUntilDone', mechanism: 'lift' }]);
    const { route: ran, filled } = measureEmptyLimits(r);
    assert.equal(filled, 5);
    assert.deepEqual(ran.steps.map((s) => s.boundS), [57, 2, undefined, undefined, 57, 57, 57, 57], 'every kind with a mechanism limit, the wait too');
    assert.equal(ran.steps[2].forS, undefined);
    assert.equal(ran.steps[3].care, undefined);
    assert.equal(r.steps[0].boundS, undefined, 'the route itself is not changed');
    assert.ok(checkRoute(r, [X], FIELD).some((x) => x.code === 'RC19' && x.field === 'boundS' && x.empty), 'Export still asks for it');
});

// ── D6: the home card ────────────────────────────────────────────────────────────────────────────────────────────

test('D6: the FIRST height move of a lift that must be homed, in a route that never homes it: one card, and its click is today\'s fix', () => {
    const r = route([{ do: 'driveTo', x: -36, y: -36 }, { do: 'move', mechanism: 'lift', to: 4 }, { do: 'startMove', mechanism: 'lift', to: 8 }]);
    const card = homeCard(r, X, 1);
    assert.deepEqual(card.fix, { kind: 'homeWhileDriving', stepIndex: 1, mechanism: 'lift' });
    assert.equal(homeCard(r, X, 2), null, 'once: not on the second move');
    assert.equal(homeCard(r, X, 0), null);
    const fixed = applyFix(r, card.fix, addStep, updateStep);
    assert.deepEqual(fixed.steps.map((s) => s.do), ['startHome', 'driveTo', 'waitUntilDone', 'move', 'startMove']);
    assert.ok(!checkRoute(fixed, [X], FIELD).some((x) => x.code === 'RC25'), 'the home rule holds after it');
    assert.equal(homeCard(fixed, X, 3), null, 'homed: no card');
    assert.equal(homeCard(route([...r.steps, { do: 'home', mechanism: 'lift' }]), X, 1), null, 'a home anywhere: no card (the rule says where)');
    assert.equal(homeCard(r, { mechanisms: [{ ...X.mechanisms[1], homing: 'restsAtStop' }] }, 1), null, 'one that rests on its stop needs none');
    assert.ok(homeCard(route([{ do: 'move', mechanism: 'arm', to: 9 }]), { mechanisms: [{ ...X.mechanisms[0], homing: 'stall' }] }, 0), 'a stall-homed arm too');
    const keep = route([{ do: 'driveTo', x: -36, y: -36 }, { do: 'startMove', mechanism: 'lift', to: 4 }, { do: 'move', mechanism: 'lift', to: 8 }]);
    assert.deepEqual(homeCard(keep, X, 1).fix, { kind: 'homeWhileDriving', stepIndex: 1, mechanism: 'lift' }, 'a first move that keeps driving');
    assert.equal(homeCard(keep, X, 2), null, 'and not on the move after it');
});

// ── D5: the badges ───────────────────────────────────────────────────────────────────────────────────────────────

test('D5: a badge\'s one word, for every mechanism step kind', () => {
    const lift = X.mechanisms.find((m) => m.ident === 'lift');
    const arm = X.mechanisms.find((m) => m.ident === 'arm');
    for (const [step, m, word] of [[{ do: 'set', position: 'closed' }, null, 'close'], [{ do: 'set', position: 'open' }, null, 'open'],
                                   [{ do: 'set', position: 'up' }, null, 'up'], [{ do: 'move', to: 18 }, lift, '18 in'], [{ do: 'startMove', to: 95 }, arm, '95 deg'],
                                   [{ do: 'move', bottom: true }, lift, 'bottom'], [{ do: 'move', preset: 2 }, lift, 'level 2'], [{ do: 'move', up: 6 }, lift, 'up 6'],
                                   [{ do: 'move', down: 2.5 }, lift, 'down 2.5'], [{ do: 'move' }, lift, 'move'], [{ do: 'home' }, lift, 'home'],
                                   [{ do: 'startHome' }, lift, 'home'], [{ do: 'waitUntilDone' }, lift, 'wait'], [{ do: 'startRun' }, null, 'run']]) {
        assert.equal(badgeWord(step, m), word, JSON.stringify(step));
    }
});

test('D5: badges sit where each step starts, stacked first-on-top; a step after a drive with no x or y has no place (never a throw)', () => {
    const r = route([{ do: 'driveTo', x: -36, y: -36 }, { do: 'set', mechanism: 'hood', position: 'out' }, { do: 'startRun', mechanism: 'intake' },
                     { do: 'driveTo', x: -12, y: -36 }, { do: 'waitUntilDone', mechanism: 'intake' }]);
    const plan = plannedPoses(r);
    const spots = badgeSpots(r, plan, 15);
    assert.deepEqual(spots.map((b) => [b.i, b.at.x, b.at.y, b.row, b.dir]), [[1, -36, -36, 1, 'up'], [2, -36, -36, 0, 'up'], [4, -12, -36, 0, 'up']]);
    const holed = updateStep(r, 0, { x: undefined });
    assert.doesNotThrow(() => badgeSpots(holed, plannedPoses(holed), 15));
    assert.deepEqual(badgeSpots(holed, plannedPoses(holed), 15).map((b) => [b.i, b.at.x, b.at.y]), [[4, -12, -36]],
                     'steps 2 and 3 have no place (none drawn; the field still draws); the next full point gives one again');
    const fromFile = loadProject(JSON.stringify({ schemaVersion: 1, madeWith: 'a hand edit', robots: {}, exports: {},
                                                  routes: [{ ...r, steps: [{ do: 'driveTo', y: -20 }, ...r.steps.slice(1, 3)] }] })).project.routes[0];
    assert.equal(fromFile.steps[0].x, undefined, 'the file opens with its hole');
    assert.deepEqual(badgeSpots(fromFile, plannedPoses(fromFile), 15), []);
});

test('D5: at the start the stack sits past the robot, away from its heading knob, toward the field\'s middle (the start stays draggable)', () => {
    const half = 7.5 * Math.SQRT2;
    const at = (start, steps) => {
        const r = route(steps, start);
        return badgeSpots(r, plannedPoses(r), 15).map((b) => [b.i, b.dir, Math.round(b.at.x * 1000) / 1000, Math.round(b.at.y * 1000) / 1000, b.row, b.n]);
    };
    const two = [{ do: 'startHome', mechanism: 'lift' }, { do: 'set', mechanism: 'hood', position: 'out' }];
    const r3 = (v) => Math.round(v * 1000) / 1000;
    for (const [x, y, heading, dir] of [[-36, -55.7, 90, 'right'], [-36, -62.75, -90, 'right'], [36, -55.7, 90, 'left'], [-36, -55.7, 45, 'right'],
                                        [-36, -55.7, 0, 'up'], [-36, -62.75, 180, 'up'], [-36, -55.7, -10, 'up'], [-36, 55.7, 0, 'down'], [12, 55.7, -120, 'left']]) {
        const ax = dir === 'right' ? x + half : dir === 'left' ? x - half : x;
        const ay = dir === 'up' ? y + half : dir === 'down' ? y - half : y;
        assert.deepEqual(at({ x, y, headingDeg: heading }, two), [[0, dir, r3(ax), r3(ay), dir === 'up' ? 1 : 0, 2], [1, dir, r3(ax), r3(ay), dir === 'up' ? 0 : 1, 2]],
                         `(${x}, ${y}) facing ${heading}: ${dir}`);
    }
    assert.deepEqual(at({ x: -36, y: -55.7, headingDeg: 90 }, [{ do: 'driveTo', x: -36, y: -36 }, { do: 'startHome', mechanism: 'lift' }]),
                     [[1, 'up', -36, -36, 0, 1]], 'elsewhere: over its spot');
});

test('D5: a band runs from a started step to the wait that collects it, and stops there; none for a step only needed later', () => {
    const r = route([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'driveTo', x: -36, y: -36 }, { do: 'driveTo', x: -12, y: -36 },
                     { do: 'waitUntilDone', mechanism: 'lift' }, { do: 'driveTo', x: -12, y: -12 }, { do: 'startRun', mechanism: 'intake' },
                     { do: 'driveTo', x: 0, y: -12 }, { do: 'run', mechanism: 'intake' }]);
    const plan = plannedPoses(r);
    const spots = badgeSpots(r, plan, 15);
    const band = spots.find((b) => b.i === 0).band;
    assert.deepEqual(band, [plan[0].from, ...stepPoses(r.steps[1], plan[1]), ...stepPoses(r.steps[2], plan[2])]);
    assert.deepEqual(band.at(-1), { ...plan[3].from }, 'it ends where the route waits');
    assert.equal(spots.find((b) => b.i === 5).band, null, 'a later step NEEDS the intake (no wait): no band');
    assert.equal(spots.find((b) => b.i === 3).band, null, 'the wait itself: none');
});

test('D5: a band across a drive with no x keeps only finite points (never NaN in the polyline)', () => {
    const r = route([{ do: 'startMove', mechanism: 'lift', to: 8 }, { do: 'driveTo', x: -36, y: -36 }, { do: 'driveTo', y: -20 },
                     { do: 'turnTo', headingDeg: 180 }, { do: 'waitUntilDone', mechanism: 'lift' }]);
    const plan = plannedPoses(r);
    assert.ok(stepPoses(r.steps[3], plan[3]).some((q) => !Number.isFinite(q.x)), 'the turn after the hole has no x of its own');
    const band = badgeSpots(r, plan, 15).find((b) => b.i === 0).band;
    assert.ok(band && band.length >= 2, 'the band still runs');
    assert.ok(band.every((q) => Number.isFinite(q.x) && Number.isFinite(q.y)), JSON.stringify(band));
});
