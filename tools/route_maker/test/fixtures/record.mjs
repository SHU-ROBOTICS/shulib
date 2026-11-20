// record.mjs — records the stand-in engine's fixtures (test/fixtures/preview/*.preview.json): each route below, run
// by the engine's NATIVE twin (`shulib_route_maker preview -`, RMP1-A2e) on robot one as a built site describes it.
// Re-record when the record's schema, robot one's facts or the API change:
//
//   node tools/route_maker/test/fixtures/record.mjs build/test/shulib_route_maker build/route_maker_site/robots.json [names...]
//
// `--out DIR` writes them into DIR instead: CI's Linux job does so for the browser rows, which have no native program
// (preview.test.mjs reads DIR from ROUTE_MAKER_NATIVE_RECORDS).
//
// The two PAIR fixtures cannot be re-recorded yet: their robot two and pair verdict came from a prototype of the
// runner's pair mode (their `madeBy` says so). This script leaves them alone until the runner has one.
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { loadProject, newProject, saveProject } from '../../src/model/project.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
const START = { x: -36, y: -60, headingDeg: 90 };

/** name -> [start, steps]: the route each fixture runs. */
export const ROUTES = {
    ready: [START, [
        { do: 'home', mechanism: 'cascade', boundS: 2 },
        { do: 'driveTo', x: -24, y: -36, faceFirst: true },
        { do: 'curveTo', points: [{ x: -12, y: -12 }, { x: 12, y: -12 }] },
        { do: 'set', mechanism: 'claw', position: 'open' },
        { do: 'wait', seconds: 0.5 },
    ]],
    failed: [START, [
        { do: 'set', mechanism: 'claw', position: 'open' },
        { do: 'driveTo', x: -36, y: -24, faceFirst: false, care: { timeoutS: 0.4 } },
        { do: 'turnTo', headingDeg: 0 },
        { do: 'wait', seconds: 0.5 },
    ]],
    carry: [START, [
        { do: 'driveTo', x: -36, y: -24, faceFirst: false, care: { timeoutS: 0.4, onFail: 'carryOn' } },
        { do: 'driveTo', x: -36, y: -12, faceFirst: false, care: { timeoutS: 0.2, onFail: { skipTo: 'finish' } } },
        { do: 'set', mechanism: 'claw', position: 'open' },
        { label: 'finish', do: 'wait', seconds: 0.3 },
    ]],
    cut: [START, [
        { do: 'driveTo', x: -36, y: -48, faceFirst: false },
        { do: 'wait', seconds: 60 },
        { do: 'set', mechanism: 'claw', position: 'open' },
    ]],
    pair_ok: [START, [
        { do: 'driveTo', x: -36, y: -36, faceFirst: false },
        { do: 'driveTo', x: -60, y: -36, faceFirst: true },
        { do: 'wait', seconds: 0.5 },
    ]],
    pair_fail: [START, [
        { do: 'driveTo', x: -36, y: -24, faceFirst: false },
        { do: 'driveTo', x: -12, y: 0, faceFirst: true },
        { do: 'wait', seconds: 0.5 },
    ]],
    // a route the page accepts whose real run goes THROUGH the right wall (the curve's follower swings out)
    through: [{ x: -36, y: -36, headingDeg: 0 }, [
        { do: 'curveTo', points: [{ x: 30, y: -36 }, { x: 60.5, y: -10 }, { x: 60.5, y: 30 }] },
        { do: 'wait', seconds: 0.5 },
    ]],
    // two start forms on one pace, in the code's order (right toggle, cascade) opposite robot one's slots
    // (cascade, right toggle): RPV1 lists them by slot, so only their names say which is which
    two_starts: [START, [
        { do: 'startRun', mechanism: 'rightToggle', volts: 6, forS: 0.5 },
        { do: 'startHome', mechanism: 'cascade', boundS: 2 },
        { do: 'driveTo', x: -36, y: -36, faceFirst: false },
        { do: 'wait', seconds: 0.3 },
    ]],
    // RMP1-E: the cascade up, down and back, the claw opened, a toggle run backward (the mechanism strip's run)
    mechanisms: [START, [
        { do: 'home', mechanism: 'cascade', boundS: 2 },
        { do: 'move', mechanism: 'cascade', up: 6, boundS: 3 },
        { do: 'move', mechanism: 'cascade', down: 2, boundS: 3 },
        { do: 'move', mechanism: 'cascade', bottom: true, boundS: 3 },
        { do: 'set', mechanism: 'claw', position: 'open' },
        { do: 'run', mechanism: 'rightToggle', volts: -6, forS: 0.5 },
    ]],
    // RMP1-J D3 / D4: a started move collected by a wait the drive leaves time for (the wait's own span), and a last
    // started move the routine's end cancels (the library stops it: no time to suggest)
    waits: [START, [
        { do: 'home', mechanism: 'cascade', boundS: 2 },
        { do: 'startMove', mechanism: 'cascade', to: 18, boundS: 3 },
        { do: 'driveTo', x: -36, y: -56, faceFirst: false },
        { do: 'waitUntilDone', mechanism: 'cascade', boundS: 3 },
        { do: 'startMove', mechanism: 'cascade', to: 4, boundS: 3 },
    ]],
    // two starts of the cascade on one pace: the library replaces the first; neither page step owns a time of its own
    two_moves: [START, [
        { do: 'home', mechanism: 'cascade', boundS: 2 },
        { do: 'startMove', mechanism: 'cascade', to: 10, boundS: 3 },
        { do: 'startMove', mechanism: 'cascade', to: 18, boundS: 3 },
        { do: 'driveTo', x: -36, y: -56, faceFirst: false },
        { do: 'waitUntilDone', mechanism: 'cascade', boundS: 3 },
    ]],
    // the program stops: a wait on a mechanism the run never started throws (the page refuses it first: RC15)
    threw: [START, [
        { do: 'driveTo', x: -36, y: -48, faceFirst: false },
        { do: 'waitUntilDone', mechanism: 'claw', boundS: 1 },
        { do: 'wait', seconds: 0.3 },
    ]],
};
export const PAIR = new Set(['pair_ok', 'pair_fail']);

/** The project file of fixture `name`, as the page saves it. */
export function projectOf(name) {
    const [start, steps] = ROUTES[name];
    const p = newProject('shulib route builder');
    p.routes[0] = { ...p.routes[0], start, steps };
    return saveProject(p);
}

async function main(args) {
    const o = args.indexOf('--out');
    const out = o >= 0 ? args.splice(o, 2)[1] : path.join(HERE, 'preview');
    const [native, robotsFile, ...names] = args;
    if (!native || !robotsFile || !out) {
        throw new Error('usage: record.mjs <shulib_route_maker> <robots.json> [--out DIR] [names...]');
    }
    const lines = path.join(HERE, '..', '..', 'src', 'model', 'route_lines.mjs');
    if (!fs.existsSync(lines)) {
        throw new Error('route_lines.mjs (RMP1-A2e) is not in this tree: the native runner reads its lines');
    }
    const { routeLines } = await import(pathToFileURL(lines).href);
    const one = JSON.parse(fs.readFileSync(robotsFile, 'utf8')).robots.one;
    for (const name of names.length ? names : Object.keys(ROUTES)) {
        if (PAIR.has(name)) {
            console.log(`${name}: not re-recorded (robot two needs the runner's pair mode)`);
            continue;
        }
        const text = projectOf(name);
        const route = loadProject(text).project.routes[0];
        const r = spawnSync(native, ['preview', '-'], { input: routeLines(route, one), encoding: 'utf8', maxBuffer: 256 << 20, timeout: 120000 });
        if (r.status === null || r.status > 1) {
            throw new Error(`${name}: rc ${r.status} ${r.stderr}`);
        }
        const madeBy = `shulib_route_maker preview (the engine's native twin), ${new Date().toISOString().slice(0, 10)}: test/fixtures/record.mjs`;
        const rec = { madeBy, project: JSON.parse(text), result: { rc: r.status, one: JSON.parse(r.stdout) } };
        fs.mkdirSync(out, { recursive: true });
        fs.writeFileSync(path.join(out, `${name}.preview.json`), `${JSON.stringify(rec)}\n`);
        console.log(name, 'rc', r.status, rec.result.one.word);
    }
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
    await main(process.argv.slice(2));
}
