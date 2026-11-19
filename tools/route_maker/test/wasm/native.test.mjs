// native.test.mjs — THE NATIVE PROGRAM AND THE MODULE, one route each (RMP1-A2e; the RMP1 brief A.2 [W2, W8, W9]):
// WG2 — every golden (WG8 among them) run by `shulib_route_maker preview` (GCC, the suite's flags) and by the
// WebAssembly module: every step, verdict, pace, time and line EXACT, the poses within 1e-5 in / 1e-6 rad (the two
// maths libraries round sin, cos and atan2 differently in the last bit, A.2); WG2s — the same over a seeded sweep of
// random routes of every step kind (ROUTE_MAKER_SWEEP routes on robot x, 300 by default, and a third as many on each
// of robots one and two with 3-decimal numbers; the same seeds every run), each module record also held to SM's span
// rule. Each prints the largest difference it measured beside the bound. And `describe` reads a robot's source lines
// from its own build's tree, never the working directory's (W10 made the kept file names relative).
//
//   ROUTE_MAKER_WASM=build/wasm ROUTE_MAKER_NATIVE=build/test/shulib_route_maker node --test tools/route_maker/test/wasm/
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { routeLines } from '../../src/model/route_lines.mjs';
import { addStep, newRoute, setStart } from '../../src/model/route.mjs';
import { REPO, seeded } from '../helpers.mjs';
import { HEAD_BOUND, NATIVE, POS_BOUND, compareRecords, goldenRoutes, instance, native, needNative, robot, spansHold } from './engine.mjs';

const opts = { skip: needNative };

function compareRuns(mod, cases, t, each = () => {}) {
    let maxPos = 0;
    let maxHead = 0;
    const bad = [];
    for (const c of cases) {
        const n = native(c.lines);
        const w = mod.run(['preview', c.lines]);
        assert.equal(n.err, '', `${c.name}: ${n.err}`);
        assert.equal(n.rc, w.rc, `${c.name}: exit codes`);
        const r = compareRecords(n.out, w.out);
        each(JSON.parse(w.out), c.name);
        maxPos = Math.max(maxPos, r.maxPos);
        maxHead = Math.max(maxHead, r.maxHead);
        if (r.problems.length) {
            bad.push(`${c.name}: ${r.problems.slice(0, 3).join('; ')}`);
        }
    }
    t.diagnostic(`${cases.length} routes; largest difference ${maxPos.toExponential(2)} in (bound ${POS_BOUND}), `
                 + `${maxHead.toExponential(2)} rad (bound ${HEAD_BOUND})`);
    assert.deepEqual(bad, [], 'native and WebAssembly ran differently');
}

test('WG2: every golden (WG8 among them) — the native program and the module: the same run', opts, async (t) => {
    const goldens = goldenRoutes();
    assert.ok(goldens.some((g) => g.name === 'wg8_distinct_numbers'));
    compareRuns(await instance('preview.mjs'), goldens, t);
});

/** What a sweep may do with robot `desc`'s mechanisms (by C++ name). Robot x's lists are spelled out, in the order
 *  the first sweeps used, so its seeds stay the routes measured in the report. */
function kitOf(desc) {
    if (desc.id === 'x') {
        return { moves: { arm: ['stow', 'score', 'high'], lift: ['floor', 'low', 'high'] }, homes: ['arm', 'lift'],
                 sets: { hood: ['down', 'middle', 'up'] }, runs: ['intake'], any: ['arm', 'lift', 'intake', 'hood', 'flywheel'] };
    }
    const of = (kinds) => desc.mechanisms.filter((m) => kinds.includes(m.kind));
    return {
        moves: Object.fromEntries(of(['arm', 'lift']).filter((m) => (m.presets || []).length).map((m) => [m.ident, m.presets.map((p) => p.name)])),
        homes: of(['arm', 'lift']).map((m) => m.ident),
        sets: Object.fromEntries(of(['pneumatic']).map((m) => [m.ident, m.positions])),
        runs: of(['power']).map((m) => m.ident),
        any: desc.mechanisms.map((m) => m.ident),
    };
}

/** WG2s's route `seed` on robot `desc`: 8–24 random steps of every A1 kind the robot can take, every choice of care
 *  and failure (mostly carry on, so most routes run long — the poses drift apart over a long closed-loop run, W2),
 *  its numbers with `dp` decimals. */
function sweepRoute(seed, desc, dp = 2) {
    const rnd = seeded(seed);
    const kit = kitOf(desc);
    const int = (lo, hi) => lo + Math.floor(rnd() * (hi - lo + 1));
    const num = (lo, hi) => int(lo * 10 ** dp, hi * 10 ** dp) / 10 ** dp;
    const pick = (a) => a[int(0, a.length - 1)];
    const pickOf = (a) => (a.length === 1 ? a[0] : pick(a));  // robot x's lone hood and intake drew no number
    const can = { move: Object.keys(kit.moves).length, startMove: Object.keys(kit.moves).length, home: kit.homes.length,
                  startHome: kit.homes.length, set: Object.keys(kit.sets).length, run: kit.runs.length, startRun: kit.runs.length };
    const n = int(8, 24);
    const labels = Array.from({ length: n }, (_, i) => (rnd() < 0.3 ? `p ${i}` : undefined));
    const started = new Set();
    let r = setStart({ ...newRoute(), robot: desc.id || 'x' }, { x: num(-60, 60), y: num(-60, 60), headingDeg: num(-179.9, 180) });
    for (let i = 0; i < n; ++i) {
        let kind = pick(['driveTo', 'reverseTo', 'turnTo', 'face', 'faceAway', 'curveTo', 'reverseCurveTo', 'move',
                         'startMove', 'set', 'run', 'startRun', 'home', 'startHome', 'waitUntilDone', 'wait']);
        if (can[kind] === 0) {
            kind = 'wait';  // a kind this robot has no mechanism for
        }
        if (kind === 'waitUntilDone' && !started.size && rnd() < 0.9) {
            kind = 'wait';  // a wait on nothing started THROWS: kept, but rare
        }
        const s = { do: kind };
        if (['driveTo', 'reverseTo', 'face', 'faceAway'].includes(kind)) {
            s.x = num(-60, 60);
            s.y = num(-60, 60);
            if (kind === 'driveTo' || kind === 'reverseTo') {
                s.faceFirst = rnd() < 0.5;
            }
        } else if (kind === 'turnTo') {
            s.headingDeg = num(-179.9, 180);
        } else if (kind.endsWith('urveTo')) {
            s.points = Array.from({ length: int(1, 4) }, () => ({ x: num(-60, 60), y: num(-60, 60),
                                                                   ...(rnd() < 0.3 ? { headingDeg: num(-179.9, 180) } : {}) }));
        } else if (kind === 'move' || kind === 'startMove') {
            s.mechanism = pick(Object.keys(kit.moves));
            s.preset = pick(kit.moves[s.mechanism]);
            s.boundS = num(0.3, 3);
        } else if (kind === 'set') {
            s.mechanism = pickOf(Object.keys(kit.sets));
            s.position = pick(kit.sets[s.mechanism]);
        } else if (kind === 'run' || kind === 'startRun') {
            s.mechanism = pickOf(kit.runs);
            s.volts = num(-12, 12);
            s.forS = num(0.1, 1.5);
        } else if (kind === 'home' || kind === 'startHome') {
            s.mechanism = pick(kit.homes);
            s.boundS = num(0.5, 3);
        } else if (kind === 'waitUntilDone') {  // mostly a mechanism started before; now and then one never started (a throw)
            s.mechanism = started.size && rnd() < 0.97 ? pick([...started]) : pick(kit.any);
            s.boundS = num(0.3, 2);
        } else {
            s.seconds = num(0.1, 4);
        }
        if (s.mechanism && kind !== 'waitUntilDone') {
            started.add(s.mechanism);
        }
        if (labels[i]) {
            s.label = labels[i];
        }
        const care = {};
        if (['driveTo', 'reverseTo', 'turnTo', 'face', 'faceAway', 'curveTo', 'reverseCurveTo'].includes(kind)) {
            for (const [f, lo, hi] of [['timeoutS', 0.5, 4], ['maxSpeedInPerS', 10, 60], ['maxTurnDegPerS', 60, 400],
                                       ['toleranceIn', 0.5, 3], ['toleranceDeg', 0.5, 5]]) {
                if (rnd() < 0.25) {
                    care[f] = num(lo, hi);
                }
            }
        }
        const later = labels.filter((l, j) => l && j > i);
        const f = rnd();
        if (f < 0.75) {
            care.onFail = 'carryOn';
        } else if (f < 0.9 && later.length) {
            care.onFail = { skipTo: pick(later) };
        }
        if (Object.keys(care).length) {
            s.care = care;
        }
        r = addStep(r, s);
    }
    return r;
}

test('WG2s: a seeded sweep of random routes of every step kind, on robots x, one and two — native and WebAssembly agree (W2)', opts, async (t) => {
    const count = Number(process.env.ROUTE_MAKER_SWEEP || 300);
    const from = Number(process.env.ROUTE_MAKER_SWEEP_FROM || 1);
    const cases = [];
    for (const [id, n, dp] of [['x', count, 2], ['one', Math.ceil(count / 3), 3], ['two', Math.ceil(count / 3), 3]]) {
        const desc = { ...robot(id), id };
        for (let i = 0; i < n; ++i) {
            cases.push({ name: `robot ${id} seed ${from + i}`, lines: routeLines(sweepRoute(from + i, desc, dp), desc) });
        }
    }
    compareRuns(await instance('preview.mjs'), cases, t, spansHold);
});

test('describe reads a robot\'s C++ names from its own build\'s tree, whatever the working directory', opts, () => {
    const describe = (cwd) => spawnSync(NATIVE, ['describe', '--robot', 'one'], { cwd, encoding: 'utf8' }).stdout;
    const want = describe(REPO);
    assert.ok(JSON.parse(want).mechanisms.some((m) => m.ident === 'cascade'), 'the names are read at all');
    // a decoy tree beside the program: the same robot files, every declared name changed
    const decoy = fs.mkdtempSync(path.join(path.dirname(NATIVE), 'describe-decoy-'));
    try {
        fs.mkdirSync(path.join(decoy, 'test'));
        fs.cpSync(path.join(REPO, 'src', 'robots'), path.join(decoy, 'src', 'robots'), { recursive: true });
        for (const f of fs.readdirSync(path.join(decoy, 'src', 'robots'))) {
            const file = path.join(decoy, 'src', 'robots', f);
            fs.writeFileSync(file, fs.readFileSync(file, 'utf8').replace(/(constexpr\s+[\w:]+\s+)(\w+)/g, '$1DECOY_$2'));
        }
        assert.equal(describe(decoy), want, 'run from another tree, it read that tree\'s files');
        assert.equal(describe(path.parse(REPO).root), want, 'run from the file system\'s root');
    } finally {
        fs.rmSync(decoy, { recursive: true, force: true });
    }
});
