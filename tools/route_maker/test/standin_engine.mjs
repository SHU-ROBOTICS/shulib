// standin_engine.mjs — THE STAND-IN ENGINE (RMP1-A2u; not a test file itself): a Worker script that answers PREVIEW
// with RECORDED runs (test/fixtures/preview/*.preview.json) instead of the WebAssembly module, through the page's own
// worker.mjs — so every screen state is tested with real records while the module is built beside it (A.7). A route
// it has no recording of is answered rc 2 with words naming that, never with another route's run.
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadProject } from '../src/model/project.mjs';
import { routeKey } from '../src/model/playback.mjs';

const HERE = path.dirname(fileURLToPath(import.meta.url));
export const FIXTURES = path.join(HERE, 'fixtures', 'preview');
const WORKER = path.join(HERE, '..', 'src', 'worker.mjs');

/** A recorded fixture: {name, text (its project file), route, result}. */
export function fixture(name) {
    const f = JSON.parse(fs.readFileSync(path.join(FIXTURES, `${name}.preview.json`), 'utf8'));
    const text = `${JSON.stringify(f.project, null, 2)}\n`;
    return { name, text, route: loadProject(text).project.routes[0], result: f.result };
}

/** The stand-in's own backend (runs INSIDE the Worker: no imports; STANDIN is defined before it). */
function standinBackend() {
    const canon = (v) => (Array.isArray(v) ? `[${v.map(canon).join(',')}]`
        : v && typeof v === 'object' ? `{${Object.keys(v).filter((k) => v[k] !== undefined).sort().map((k) => `${JSON.stringify(k)}:${canon(v[k])}`).join(',')}}`
            : JSON.stringify(v));
    const busy = (ms) => {
        for (const end = Date.now() + ms; Date.now() < end;) { /* the module's run is synchronous too */ }
    };
    // a run the library refused before its first pace, as RPV1's refusedModel writes it: no paces, no steps, REFUSED
    const refused = (rec, why) => ({ ...rec, samples: [], steps: [], lines: [], field: [], refused: true, word: 'REFUSED', why,
                                     pageSteps: rec.pageSteps.map((q) => ({ ...q, verdict: 'notReached', first: null, last: null, lib: null })) });
    self.shulibBackend = async () => {
        if (STANDIN.failLoad) {
            throw new Error(STANDIN.failLoad);
        }
        const made = ++STANDIN.made;
        const find = (route) => STANDIN.runs[canon(route)];
        return {
            preview(route) {
                const run = find(route);
                if (STANDIN.hang && run && run.name === STANDIN.hang) {
                    for (;;) { /* a run that never returns: the page must stop it */ }
                }
                busy(STANDIN.delayMs);
                const why = run && STANDIN.refuse[run.name];
                return why ? { rc: 1, record: refused(run.one, why) }
                    : run ? { rc: STANDIN.rcOf[run.name] ?? run.rc,
                              record: { ...run.one, ...STANDIN.patch[run.name], standinInstance: made, standinRun: ++STANDIN.ran } }
                    : { rc: 2, record: null, words: 'the stand-in engine has no recorded run of this route' };
            },
            pair(route) {
                const run = find(route);
                const two = run && (STANDIN.two[run.name] || (run.two && { record: run.two, verdict: run.verdict }));
                return two ? { rc: STANDIN.pairRc[run.name] ?? 0, record: two.record, verdict: two.verdict }
                    : { rc: 2, record: null, words: 'the stand-in engine has no recorded run of robot two' };
            },
        };
    };
}

/** The Worker script: the stand-in backend, then the page's own worker.mjs. */
export function standinScript(names, { delayMs = 0, hang = null, failLoad = '', rcOf = {}, refuse = {}, two = {}, pairRc = {}, patch = {} } = {}) {
    const runs = {};
    for (const n of names) {
        const f = fixture(n);
        runs[routeKey(f.route)] = { name: n, ...f.result };
    }
    return `const STANDIN = ${JSON.stringify({ runs, delayMs, hang, failLoad, rcOf, refuse, two, pairRc, patch, made: 0, ran: 0 })};\n(${standinBackend.toString()})();\n`
        + fs.readFileSync(WORKER, 'utf8');
}
