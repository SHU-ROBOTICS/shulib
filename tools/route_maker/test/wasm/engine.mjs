// engine.mjs — shared by the preview engine's node tests (RMP1-A2e; not a test file itself): where the module and
// the native program are, how each runs a route, every golden's lines, and how two records are compared.
//
//   ROUTE_MAKER_WASM    the built module's folder (build/wasm) — the module tests run only when it is set
//   ROUTE_MAKER_NATIVE  the native shulib_route_maker (build/test/shulib_route_maker) — WG2 and WG2s need it
//   ROUTE_MAKER_SITE    the built site (build/route_maker_site): robots one's and two's baked descriptions
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { pathToFileURL } from 'node:url';
import { loadProject } from '../../src/model/project.mjs';
import { newRoute } from '../../src/model/route.mjs';
import { routeLines } from '../../src/model/route_lines.mjs';
import { engineInstance } from '../../src/host/engine_module.mjs';
import { GOLDENS, REPO, fixtureRobot, goldenNames } from '../helpers.mjs';

const env = (k) => (process.env[k] ? path.resolve(process.env[k]) : '');
export const WASM = env('ROUTE_MAKER_WASM');
export const NATIVE = env('ROUTE_MAKER_NATIVE');
const SITE = env('ROUTE_MAKER_SITE') || path.join(REPO, 'build', 'route_maker_site');

/** The skip reason when the module is not asked for (the page tests' convention: an env var turns them on). */
export const needWasm = !WASM ? 'set ROUTE_MAKER_WASM (the built module, build/wasm) to run the engine tests' : false;
export const needNative = needWasm || (!NATIVE ? 'set ROUTE_MAKER_NATIVE (build/test/shulib_route_maker) for WG2' : false);

/** A module instance (`file` in ROUTE_MAKER_WASM) through the page's own wrapper: run(args) -> {rc, out, err}, and
 *  the page's own preview(lines). */
export async function instance(file = 'preview.mjs') {
    const f = path.join(WASM, file);
    if (!fs.existsSync(f)) {
        throw new Error(`${f} is missing: build the module (tools/route_maker/README.md)`);
    }
    const { default: create } = await import(pathToFileURL(f).href);
    const e = await engineInstance(create);
    const kept = (fn) => (arg) => {
        const saved = process.exitCode;
        const r = fn(arg);
        process.exitCode = saved;  // Emscripten's Node glue sets the exit code from main's: it is the test's to set
        return r;
    };
    return { run: kept(e.call), preview: kept(e.preview) };
}

/** The native program's run of `lines`. */
export function native(lines) {
    const r = spawnSync(NATIVE, ['preview', '-'], { input: lines, encoding: 'utf8', maxBuffer: 256 << 20, timeout: 300000 });
    return { rc: r.status, out: r.stdout, err: r.stderr };
}

/** A robot's description: the fixture robots' committed ones, robots one's and two's baked into the built site. */
const robots = {};
export function robot(id) {
    if (!robots[id]) {
        if (['x', 'y', 'z'].includes(id)) {
            robots[id] = fixtureRobot(id);
        } else {
            const f = path.join(SITE, 'robots.json');
            if (!fs.existsSync(f)) {
                throw new Error(`${f} is missing: build the site (tools/route_maker/build.py) for robots one and two`);
            }
            robots[id] = JSON.parse(fs.readFileSync(f, 'utf8')).robots[id];
        }
    }
    return robots[id];
}

/** Every golden: {name, route, lines} — gl0_empty is the empty route on robot x (§6.3's text, no route.json). */
export function goldenRoutes() {
    const out = [{ name: 'gl0_empty', route: { ...newRoute(), robot: 'x' } }];
    for (const name of goldenNames()) {
        const p = loadProject(fs.readFileSync(path.join(GOLDENS, `${name}.route.json`), 'utf8'));
        out.push({ name, route: p.project.routes[0] });
    }
    return out.map((g) => ({ ...g, lines: routeLines(g.route, robot(g.route.robot)) }));
}

/** `text` without its pageSteps line (the compiled twin has none to give: WG1). */
export const withoutPageSteps = (text) => text.split('\n').filter((l) => !l.startsWith('"pageSteps": ')).join('\n');

/** W2's bounds: positions within 1e-5 in, headings within 1e-6 rad. */
export const POS_BOUND = 1e-5;
export const HEAD_BOUND = 1e-6;

/**
 * WG2's comparison of a native and a WebAssembly record: everything EXACT but the poses (a sample's x, y, theta; a
 * step's target; a field note's depth), which must agree within the bounds. Returns {problems, maxPos, maxHead}.
 */
export function compareRecords(a, b) {
    const A = JSON.parse(a);
    const B = JSON.parse(b);
    const problems = [];
    let maxPos = 0;
    let maxHead = 0;
    const near = (x, y, bound, what, head) => {
        const d = Math.abs(x - y);
        if (head) {
            maxHead = Math.max(maxHead, d);
        } else {
            maxPos = Math.max(maxPos, d);
        }
        if (!(d <= bound)) {
            problems.push(`${what}: ${x} vs ${y}`);
        }
    };
    for (const k of new Set([...Object.keys(A), ...Object.keys(B)])) {
        if (k === 'samples' || k === 'steps' || k === 'field') {
            continue;
        }
        if (JSON.stringify(A[k]) !== JSON.stringify(B[k])) {
            problems.push(`${k} differs`);
        }
    }
    const pairs = (k, f) => {
        if (A[k].length !== B[k].length) {
            problems.push(`${k}: ${A[k].length} vs ${B[k].length}`);
            return;
        }
        A[k].forEach((x, i) => f(x, B[k][i], i));
    };
    pairs('samples', (p, q, i) => {
        if (p[0] !== q[0]) {
            problems.push(`sample ${i}: t ${p[0]} vs ${q[0]}`);
        }
        near(p[1], q[1], POS_BOUND, `sample ${i} x`, false);
        near(p[2], q[2], POS_BOUND, `sample ${i} y`, false);
        near(p[3], q[3], HEAD_BOUND, `sample ${i} theta`, true);
    });
    pairs('steps', (p, q, i) => {
        const { target: ta, ...pa } = p;
        const { target: tb, ...pb } = q;
        if (JSON.stringify(pa) !== JSON.stringify(pb) || (ta === null) !== (tb === null)) {
            problems.push(`step ${i} differs: ${JSON.stringify(pa)} vs ${JSON.stringify(pb)}`);
        } else if (ta) {
            near(ta[0], tb[0], POS_BOUND, `step ${i} target x`, false);
            near(ta[1], tb[1], POS_BOUND, `step ${i} target y`, false);
            near(ta[2], tb[2], HEAD_BOUND, `step ${i} target heading`, true);
        }
    });
    pairs('field', (p, q, i) => {
        const { depth: da, ...pa } = p;
        const { depth: db, ...pb } = q;
        if (JSON.stringify(pa) !== JSON.stringify(pb)) {
            problems.push(`field note ${i} differs`);
        }
        near(da, db, POS_BOUND, `field note ${i} depth`, false);
    });
    return { problems, maxPos, maxHead };
}

/** SM's rule on one record: the page steps tile the run from the arming, a start form takes no time, every motion
 *  lies in a page step, and a one-motion page step's span is its motion's exactly. */
export function spansHold(rec, name) {
    const page = rec.pageSteps.filter((p) => p.first !== null);
    if (!page.length) {
        return;
    }
    assert.equal(page[0].first, 0, `${name}: the first page step starts at the arming`);
    page.forEach((p, i) => {
        assert.ok(p.last >= p.first, `${name} ${p.n}`);
        if (i > 0) {
            assert.equal(p.first, page[i - 1].last, `${name}: page step ${p.n} starts where ${page[i - 1].n} ended`);
        }
    });
    const motions = rec.steps.filter((s) => !s.endAction && s.kind !== 'mechanism');
    for (const m of motions) {
        assert.ok(page.some((p) => p.first <= m.s0 && m.s1 <= p.last), `${name}: ${m.what} at ${m.s0} lies in a page step`);
    }
    for (const p of page.filter((q) => ['startMove', 'startRun', 'startHome'].includes(q.do) && q.verdict === 'ok')) {
        assert.equal(p.last, p.first, `${name}: page step ${p.n} (${p.do}) starts its mechanism and returns at once`);
    }
    for (const p of page) {
        const one = ['turnTo', 'face', 'faceAway', 'curveTo', 'reverseCurveTo'].includes(p.do) && p.lib && p.lib[0] === p.lib[1];
        if (one && p.verdict !== 'skipped' && p.verdict !== 'jumped') {
            assert.ok(motions.some((m) => m.s0 === p.first && m.s1 === p.last), `${name}: page step ${p.n}'s span is its motion's`);
        }
    }
}
