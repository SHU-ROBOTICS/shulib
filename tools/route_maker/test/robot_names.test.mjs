// RMP1-L: White Bot and Black Bot (the team lead, 2026-11-20) -- the robots' names in what people see, from ONE place.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { robotName, robotBridge } from '../src/model/robot_names.mjs';

const SRC = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'src');
const files = (dir) => fs.readdirSync(dir, { withFileTypes: true }).flatMap((e) => (e.isDirectory() ? files(path.join(dir, e.name))
    : /\.(mjs|js|html)$/.test(e.name) ? [path.join(dir, e.name)] : []));
const rel = (f) => path.relative(SRC, f).split(path.sep).join('/');
/** The file's code lines, comments dropped (a `//` after a space or a line's start; HTML comments): an identifier is
 *  camelCase, so "robot one" in one is a string the page shows. */
const code = (text) => text.replace(/\/\*[\s\S]*?\*\//g, '').replace(/(^|\s)\/\/.*$/gm, '$1').replace(/<!--[\s\S]*?-->/g, '')
    .split('\n').map((s) => s.trim()).filter(Boolean);

test('the names: one -> White Bot, two -> Black Bot; the bridge once, "White Bot (robot one)"; another id plainly', () => {
    assert.equal(robotName('one'), 'White Bot');
    assert.equal(robotName('two'), 'Black Bot');
    assert.equal(robotBridge('one'), 'White Bot (robot one)');
    assert.equal(robotBridge('two'), 'Black Bot (robot two)');
    assert.equal(robotName('x'), 'robot x');
    assert.equal(robotBridge('x'), 'robot x');
});

test('ONE place spells them: no other page file says White Bot or Black Bot, in any capitals', () => {
    const all = files(SRC);
    assert.ok(all.length > 40 && all.some((f) => rel(f) === 'ui/app.mjs'), `the page's files: ${all.length}`);
    const others = all.filter((f) => rel(f) !== 'model/robot_names.mjs' && /white bot|black bot/i.test(fs.readFileSync(f, 'utf8')));
    assert.deepEqual(others.map(rel), []);
});

test('no string the page shows still says "robot one" / "robot two" (or builds "robot ${...}") -- only the exported C++ does', () => {
    // export.mjs writes the C++ (its comments say ROBOT ONE: the goldens, byte for byte); robot_names.mjs is the source
    const keep = new Set(['model/export.mjs', 'model/robot_names.mjs']);
    const said = files(SRC).filter((f) => !keep.has(rel(f))).flatMap((f) => code(fs.readFileSync(f, 'utf8'))  // "whole robot ${fmt(..)} in": a size
        .filter((s) => /\brobot (one|two)\b|\brobot \$\{(?!fmt\()/i.test(s)).map((s) => `${rel(f)}: ${s.slice(0, 100)}`));
    assert.deepEqual(said, []);
    assert.ok(code(fs.readFileSync(path.join(SRC, 'model', 'export.mjs'), 'utf8')).some((s) => /ROBOT ONE's numbers/.test(s)),
        'the scan reads code lines at all (the C++ comment export.mjs writes is one)');
});

test('no page code shows a robot\'s description name ("robot one" in robots.json): only robotName(id) names a robot', () => {
    // the common reads; names.page.test.mjs walks the screens with robots.json's names set to LEAK for the rest
    const read = /\brobot(\(\))?\.name\b|\.robot\.name\b|robots\[[^\]]+\]\.name\b|\b(one|two)\.name\b|routeRobots\([^)]*\)\.map\(\(?(\w+)\)? => \3\.name\b/;
    const said = files(SRC).filter((f) => rel(f) !== 'model/export.mjs').flatMap((f) => code(fs.readFileSync(f, 'utf8'))
        .filter((s) => read.test(s)).map((s) => `${rel(f)}: ${s.slice(0, 100)}`));
    assert.deepEqual(said, []);
    for (const s of ['robot.name', 'robot().name', 'x.robot.name', 'baked.robots[id].name', 'one.name', 'routeRobots(baked, r).map((x) => x.name)']) {
        assert.ok(read.test(s), `the scan sees ${s}`);
    }
});
