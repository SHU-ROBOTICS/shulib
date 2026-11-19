// build_engine.test.mjs — build.py --engine (RMP1-A2u): PREVIEW's Worker script put into both forms of the page, and
// refused when it could not run there. Runs build.py (Python) on a FAKE engine folder: a glue shaped as Emscripten
// writes it and a .wasm of a few bytes; the real module is the engine tests' and the page tests' (README "PREVIEW").
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { REPO, fixtureRobot } from './helpers.mjs';

const BUILD = path.join(REPO, 'tools', 'route_maker', 'build.py');
const HOST = path.join(REPO, 'tools', 'route_maker', 'src');
const HAS_A2E = ['host/engine_module.mjs', 'model/route_lines.mjs'].every((f) => fs.existsSync(path.join(HOST, f)));
const skip = process.platform === 'win32' ? 'build.py runs in the Linux job' : false;
const GLUE = 'async function createPreview(moduleArg={}){var Module=moduleArg;var _scriptName=import.meta.url;return Module}export default createPreview;';
/** The glue as the classic Worker script carries it: no export, its own URL read from self.location. */
const CLASSIC_GLUE = GLUE.replace('export default createPreview;', '').replace('import.meta.url', 'self.location.href');

function build(glue, single = glue, { drop = '' } = {}) {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'rm-engine-'));
    try {
        const eng = path.join(dir, 'wasm');
        fs.mkdirSync(eng);
        fs.writeFileSync(path.join(eng, 'preview.mjs'), glue);
        fs.writeFileSync(path.join(eng, 'preview_single.mjs'), single);
        fs.writeFileSync(path.join(eng, 'preview.wasm'), Buffer.from([0, 97, 115, 109, 1, 0, 0, 0]));
        if (drop) {
            fs.rmSync(path.join(eng, drop));
        }
        fs.writeFileSync(path.join(dir, 'robots.json'), JSON.stringify({ one: { ...fixtureRobot('x'), id: 'one' }, two: { ...fixtureRobot('y'), id: 'two' } }));
        const out = path.join(dir, 'site');
        const r = spawnSync('python3', [BUILD, '--out', out, '--robots', path.join(dir, 'robots.json'), '--engine', eng], { encoding: 'utf8', timeout: 120000 });
        const read = (f) => (fs.existsSync(path.join(out, f)) ? fs.readFileSync(path.join(out, f), 'utf8') : null);
        return { rc: r.status, err: r.stderr, worker: read('engine/worker.mjs'), wasm: fs.existsSync(path.join(out, 'engine', 'preview.wasm')),
                 site: read('app.html'), one: read('shulib-route-builder.html') };
    } finally {
        fs.rmSync(dir, { recursive: true, force: true });
    }
}

test('--engine: the Worker script is the glue (its export made plain), the backend, then worker.mjs — in both forms', { skip: skip || (!HAS_A2E && 'the engine\'s modules (RMP1-A2e) are not in this tree') }, () => {
    const b = build(GLUE);
    assert.equal(b.rc, 0, b.err);
    assert.ok(!b.worker.includes('export default'), 'a Worker script built from parts has no export');
    assert.ok(!b.worker.includes('import.meta') && b.worker.startsWith(CLASSIC_GLUE), 'a CLASSIC script: import.meta.url is self.location.href');
    assert.ok(b.worker.startsWith('async function createPreview('));
    assert.ok(b.worker.includes("self.shulibBackend = () => __shulib['host/engine_backend.mjs'].wasmBackend(createPreview);"));
    assert.ok(b.worker.endsWith(fs.readFileSync(path.join(HOST, 'worker.mjs'), 'utf8')), 'worker.mjs last, whole');
    assert.ok(b.wasm);
    assert.ok(b.site.includes('<meta name="shulib-engine" content="engine/worker.mjs">'));
    assert.ok(b.one.includes(`<script type="text/plain" id="shulib-engine">${CLASSIC_GLUE}`));
});

test('--engine REFUSED: a glue whose export is not there once, one a classic Worker cannot run, or a script that would end the page\'s element early', { skip }, () => {
    for (const f of ['preview.mjs', 'preview.wasm', 'preview_single.mjs']) {
        const b = build(GLUE, GLUE, { drop: f });
        assert.equal(b.rc, 1, f);
        assert.match(b.err, new RegExp(`has no ${f.replace('.', '\\.')} \\(build the module`), f);
    }
    const topAwait = build(`const m = await import('node:module');${GLUE}`);  // module-only code at the top
    assert.equal(topAwait.rc, 1);
    assert.match(topAwait.err, /does not begin with 'async function createPreview\(': code at its top could not run in a classic Worker/);
    for (const meta of ['import.meta.resolve("x")', 'import.meta']) {  // only import.meta.url has a classic twin
        const b = build(GLUE.replace('return Module', `return ${meta}`));
        assert.equal(b.rc, 1, meta);
        assert.match(b.err, /uses import\.meta beyond import\.meta\.url: it cannot run as a classic Worker script/, meta);
    }
    if (!HAS_A2E) {
        const b = build(GLUE);
        assert.equal(b.rc, 1);
        assert.match(b.err, /--engine needs src\/host\/engine_module\.mjs \(the preview engine, RMP1-A2e\)/);
        return;
    }
    assert.match(build(GLUE.replace('export default createPreview;', '')).err, /does not end with 'export default createPreview;' once/);
    assert.match(build(`${GLUE}\n${GLUE}`).err, /does not end with 'export default createPreview;' once/);
    for (const bad of ['"</script>"', '"</SCRIPT"', '"<!--"']) {
        const b = build(GLUE, GLUE.replace('return Module', `return ${bad}`));
        assert.equal(b.rc, 1);
        assert.match(b.err, /holds <\/script or <!--: it cannot sit inside the page/);
    }
});

test('no --engine: no engine in either form (PREVIEW says it comes next)', { skip }, () => {
    const dir = fs.mkdtempSync(path.join(os.tmpdir(), 'rm-noengine-'));
    try {
        fs.writeFileSync(path.join(dir, 'robots.json'), JSON.stringify({ one: { ...fixtureRobot('x'), id: 'one' }, two: { ...fixtureRobot('y'), id: 'two' } }));
        const r = spawnSync('python3', [BUILD, '--out', path.join(dir, 'site'), '--robots', path.join(dir, 'robots.json')], { encoding: 'utf8', timeout: 120000 });
        assert.equal(r.status, 0, r.stderr);
        assert.ok(!fs.existsSync(path.join(dir, 'site', 'engine')));
        for (const f of ['app.html', 'shulib-route-builder.html']) {
            const html = fs.readFileSync(path.join(dir, 'site', f), 'utf8');
            assert.ok(!html.includes('<script type="text/plain" id="shulib-engine">') && !html.includes('<meta name="shulib-engine"'), f);
        }
    } finally {
        fs.rmSync(dir, { recursive: true, force: true });
    }
});
