// goldens.mjs — the route builder's goldens: read them, and RE-MAKE them (chunk RMP1-A1).
//
//   node tools/route_maker/goldens.mjs --write
//
// For every test/goldens/route_maker/*.route.json: rewrite it in the project file's canonical form
// (load -> save) and write its export, `<name>.cpp` (exportRegion + a final newline) — the file the C++
// suite compiles and runs (test/route_maker_golden_test.cpp). test/export.test.mjs fails when a committed
// golden differs from what this would write; run this, then READ the diff before committing it.
// gl0_empty.cpp is NOT one of them: it is §6.3's contract text, kept by hand (no route.json), so the exporter
// is held to it (export.test.mjs's GL0) instead of re-making it.
//
// The golden routes are TEST routes on the suite's fixture robots (and smoke routes on robots one and
// two): placeholders that exercise the exporter and the library, never a route for the team (G4).

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { loadProject, saveProject } from './src/model/project.mjs';
import { exportRegion } from './src/model/export.mjs';

export const GOLDEN_DIR = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'test', 'goldens', 'route_maker');

/** Every golden: [name, its canonical project text, its export text]. */
export function goldens() {
    const out = [];
    for (const f of fs.readdirSync(GOLDEN_DIR).filter((n) => n.endsWith('.route.json')).sort()) {
        const name = f.slice(0, -'.route.json'.length);
        const text = fs.readFileSync(path.join(GOLDEN_DIR, f), 'utf8');
        const loaded = loadProject(text);
        if (!loaded.ok) {
            throw new Error(`${f}: ${loaded.words}`);
        }
        out.push([name, saveProject(loaded.project), `${exportRegion(loaded.project.routes[0])}\n`]);
    }
    return out;
}

if (process.argv.includes('--write')) {
    for (const [name, project, cpp] of goldens()) {
        fs.writeFileSync(path.join(GOLDEN_DIR, `${name}.route.json`), project);
        fs.writeFileSync(path.join(GOLDEN_DIR, `${name}.cpp`), cpp);
        console.log(`wrote ${name}.route.json and ${name}.cpp`);
    }
}
