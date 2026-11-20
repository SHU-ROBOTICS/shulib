// files_local.test.mjs — RMP1-I: the local host (files_local.mjs) against a stand-in helper's fetch — the request shapes,
// a refusal or a stopped helper surfacing as the error kinds the other hosts use — Send's stream read AS IT COMES, and
// sendStage over tools/upload_auton.py's own lines. The answers are plain objects, not `Response`: Node's Response loads
// a WebAssembly parser that a capped run (ulimit -v) cannot reserve memory for.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { localPage, localHost, sendToRobot, sendLog, GONE } from '../src/host/files_local.mjs';
import { makeFiles } from '../src/host/files.mjs';
import { sendStage } from '../src/model/to_robot.mjs';
import { newProject } from '../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { GOLDENS } from './helpers.mjs';

const PAGE = { token: 'tok-123', folder: 'shulib' };
const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\r\n\r\n${EMPTY.replace(/\n/g, '\r\n')}\r\n`;
const ROUTE = addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }), { do: 'wait', seconds: 1 });

/** An answer as fetch gives it (what files_local.mjs reads of one): `body` a string, or a ReadableStream for Send's. */
const reply = (body, status = 200) => ({ ok: status >= 200 && status < 300, status, body,
                                         text: async () => (body === null || body === undefined ? '' : String(body)) });

/** A stand-in helper: the files under src/team/ in `store`, everything else refused (403), every request recorded. */
function helper(store, { refuseWrites = false } = {}) {
    const calls = [];
    const fetch = async (url, init) => {
        calls.push({ url, ...init });
        const u = new URL(url, 'http://127.0.0.1:5000/app.html');
        if (init.headers['X-Shulib-Token'] !== PAGE.token) {
            return reply('refused: the token', 403);
        }
        const p = u.searchParams.get('path');
        if (u.pathname !== '/api/file' || !/^src\/team\/[^/]+$/.test(p)) {
            return reply(`refused: not ${p}`, 403);
        }
        if (init.method === 'GET') {
            return p in store ? reply(store[p]) : reply(null, 204);
        }
        if (refuseWrites) {
            return reply('not written: No space left on device', 500);
        }
        store[p] = init.body;
        return reply('written');
    };
    return { fetch, calls, store };
}

test('localPage: the helper\'s meta (token and folder); none, or an empty one, is the site', () => {
    const doc = (m) => ({ querySelector: (q) => (q === 'meta[name="shulib-local"]' ? m : null) });
    assert.deepEqual(localPage(doc({ content: 'abc', dataset: { folder: 'my-shulib' } })), { token: 'abc', folder: 'my-shulib' });
    assert.equal(localPage(doc(null)), null);
    assert.equal(localPage(doc({ content: '', dataset: {} })), null);
    assert.deepEqual(localPage(doc({ content: 'abc', dataset: {} })), { token: 'abc', folder: 'shulib' }, 'no folder named: "shulib"');
    assert.equal(localPage(null), null);
});

test('readText / writeText: the request shapes, byte for byte, the token on every one', async () => {
    const hp = helper({ 'src/team/a.cpp': 'x\r\n✓\r\n' });
    const host = localHost(PAGE, { fetch: hp.fetch });
    assert.equal(host.kind, 'local');
    assert.equal(host.connected(), true);
    assert.equal(host.folderName(), 'shulib');
    assert.equal(await host.readText('src/team/a.cpp'), 'x\r\n✓\r\n');
    assert.equal(await host.readText('src/team/none.cpp'), null, 'a missing file is null, as the folder host\'s NotFoundError');
    await host.writeText('src/team/b.cpp', 'a\r\nb ✓');
    assert.equal(hp.store['src/team/b.cpp'], 'a\r\nb ✓');
    const [read, , write] = hp.calls;
    assert.equal(read.url, 'api/file?path=src%2Fteam%2Fa.cpp');
    assert.equal(read.method, 'GET');
    assert.equal(read.cache, 'no-store');
    assert.deepEqual(read.headers, { 'X-Shulib-Token': 'tok-123' });
    assert.equal(write.method, 'POST');
    assert.equal(write.body, 'a\r\nb ✓');
    await assert.rejects(host.readText('src/robots/robot_one_facts.hpp'), /refused: not src\/robots/);
    await assert.rejects(host.writeText('src/team/../x', 'y'), /refused/);
    await assert.rejects(localHost({ ...PAGE, token: 'wrong' }, { fetch: hp.fetch }).readText('src/team/a.cpp'), /refused: the token/);
    const gone = localHost(PAGE, { fetch: async () => { throw new TypeError('Failed to fetch'); } });
    await assert.rejects(gone.readText('src/team/a.cpp'), (e) => e.message === GONE);
});

test('through files.mjs: Export writes and reads back; a refusal and a stopped helper are "failed" in words', async () => {
    const hp = helper({ 'src/team/skills_route.cpp': SKILLS });
    const out = await makeFiles(localHost(PAGE, { fetch: hp.fetch })).exportToFile([ROUTE], newProject());
    assert.ok(out.ok, out.words);
    assert.match(hp.store['src/team/skills_route.cpp'], /wait 1 s/);
    assert.ok(!/[^\r]\n/.test(hp.store['src/team/skills_route.cpp']), 'the file\'s CRLF kept');
    const full = helper({ 'src/team/skills_route.cpp': SKILLS }, { refuseWrites: true });
    const no = await makeFiles(localHost(PAGE, { fetch: full.fetch })).exportToFile([ROUTE], newProject());
    assert.equal(no.why, 'failed');
    assert.equal(no.words, 'The folder refused: not written: No space left on device');
    assert.equal(full.store['src/team/skills_route.cpp'], SKILLS);
    const gone = await makeFiles(localHost(PAGE, { fetch: async () => { throw new TypeError('NetworkError'); } })).exportToFile([ROUTE], newProject());
    assert.equal(gone.why, 'failed');
    assert.equal(gone.words, `The folder refused: ${GONE}`);
});

/** A streamed response: `chunks` (strings or byte arrays) sent one by one, each when `next()` is called, then the end
 *  (`cut`: no end — `fail()` breaks the connection, the browser's "network error"). */
function streamed(chunks, { cut = false } = {}) {
    const enc = new TextEncoder();
    let ctl;
    const body = new ReadableStream({ start: (c) => { ctl = c; } });
    let i = 0;
    return {
        res: reply(body),
        next() {
            const c = chunks[i++];
            ctl.enqueue(typeof c === 'string' ? enc.encode(c) : c);
            if (i === chunks.length && !cut) {
                ctl.close();
            }
        },
        fail: () => ctl.error(new TypeError('network error')),
    };
}

test('sendToRobot: each line AS IT COMES (a line split across reads, a ✓ split mid-character), then the code', async () => {
    const ok = new TextEncoder().encode(`${JSON.stringify({ line: '✓ BUILT' })}\n`);
    const s = streamed([`${JSON.stringify({ line: 'Building robot one\'s Auton Tester' })}\n{"li`, `ne": "Uploading"}\n`,
                        ok.slice(0, 11), ok.slice(11), `${JSON.stringify({ done: true, code: 0 })}\n`]);
    let call;
    const lines = [];
    let first;
    const seen = new Promise((resolve) => { first = resolve; });
    const out = sendToRobot(PAGE, 'two', (l) => { lines.push(l); first(); }, { fetch: async (url, init) => { call = { url, ...init }; return s.res; } });
    s.next();
    await seen;
    assert.deepEqual(lines, ['Building robot one\'s Auton Tester'], 'the first line before the rest of the stream exists');
    for (let k = 0; k < 4; k++) {
        s.next();
    }
    assert.equal(await out, 0);
    assert.deepEqual(lines, ['Building robot one\'s Auton Tester', 'Uploading', '✓ BUILT']);
    assert.equal(call.url, 'api/send?robot=two');
    assert.equal(call.method, 'POST');
    assert.deepEqual(call.headers, { 'X-Shulib-Token': 'tok-123' });
});

test('sendToRobot: a refusal (one already running) in the helper\'s words; a helper stopped mid-send says so', async () => {
    const busy = async () => reply('A send is already running: wait for it to finish.', 409);
    await assert.rejects(sendToRobot(PAGE, 'one', () => {}, { fetch: busy }), /already running/);
    const cut = streamed([`${JSON.stringify({ line: 'Building' })}\n`]);
    const p = sendToRobot(PAGE, 'one', () => {}, { fetch: async () => cut.res });
    cut.next();
    await assert.rejects(p, (e) => e.message === GONE, 'no "done": the helper stopped');
    const broken = streamed([`${JSON.stringify({ line: 'Building' })}\n`], { cut: true });
    const seen = [];
    let shown;
    const first = new Promise((resolve) => { shown = resolve; });
    const b = sendToRobot(PAGE, 'one', (l) => { seen.push(l); shown(); }, { fetch: async () => broken.res });
    broken.next();
    await first;
    broken.fail();
    await assert.rejects(b, (e) => e.message === GONE, 'the connection broken mid-send: the helper\'s words, not the browser\'s');
    assert.deepEqual(seen, ['Building'], 'the lines before the break were shown');
    await assert.rejects(sendToRobot(PAGE, 'one', () => {}, { fetch: async () => { throw new TypeError('x'); } }), (e) => e.message === GONE);
    const failed = streamed([`${JSON.stringify({ line: '✗ NOT BUILT' })}\n${JSON.stringify({ done: true, code: 1 })}`]);
    const q = sendToRobot(PAGE, 'one', () => {}, { fetch: async () => failed.res });
    failed.next();
    assert.equal(await q, 1, 'the last line needs no newline');
    assert.equal(await sendLog(PAGE, { fetch: async () => reply('make: ok') }), 'make: ok');
    assert.equal(await sendLog(PAGE, { fetch: async () => reply('nothing sent yet', 404) }), '');
});

test('sendStage: tools/upload_auton.py\'s own lines, said plainly', () => {
    assert.deepEqual(sendStage('Building robot two\'s Auton Tester: make ROBOT=tank PROGRAM=autontest (a few minutes the first time; Ctrl+C stops it)...'),
        { kind: 'step', words: 'Building robot two\'s Auton Tester… (a few minutes the first time)' });
    assert.deepEqual(sendStage('✓ BUILT'), { kind: 'built', words: 'BUILT' });
    assert.deepEqual(sendStage('Uploading to slot 2 on COM3...'), { kind: 'step', words: 'Uploading to slot 2 on COM3…' });
    assert.deepEqual(sendStage('✓ UPLOADED to slot 2. On the brain: shulib Auton Tester → skills or left 1 → RUN'),
        { kind: 'uploaded', words: 'UPLOADED to slot 2', then: 'On the brain: shulib Auton Tester → skills or left 1 → RUN' });
    assert.deepEqual(sendStage('✗ NOT BUILT'), { kind: 'failed', words: 'NOT BUILT', then: '' });
    assert.deepEqual(sendStage('✗ NOT UPLOADED: No brain found. Plug the brain in with USB (not the controller), turn it on, then run this again.'),
        { kind: 'failed', words: 'NOT UPLOADED', then: 'No brain found. Plug the brain in with USB (not the controller), turn it on, then run this again.' });
    assert.deepEqual(sendStage('✗ make and pros not found. Run this in the PROS terminal (VS Code: PROS, then Integrated Terminal)'),
        { kind: 'failed', words: 'make and pros not found', then: 'Run this in the PROS terminal (VS Code: PROS, then Integrated Terminal)' });
    assert.deepEqual(sendStage('✗ STOPPED (Ctrl+C). What make and pros printed so far: build/upload_auton.log'),
        { kind: 'failed', words: 'STOPPED (Ctrl+C)', then: 'What make and pros printed so far: build/upload_auton.log' });
    assert.deepEqual(sendStage('  src/team/skills_route.cpp:6: \'r\' was not declared in this scope'),
        { kind: 'detail', words: 'src/team/skills_route.cpp:6: \'r\' was not declared in this scope' });
    assert.deepEqual(sendStage('  (everything make printed: C:\\shulib\\build\\upload_auton.log)'), { kind: 'log' });
    assert.deepEqual(sendStage('  (make from the PROS toolchain: C:\\pros\\bin\\make.exe)'), { kind: 'detail', words: '(make from the PROS toolchain: C:\\pros\\bin\\make.exe)' });
    assert.equal(sendStage(''), null);
    assert.equal(sendStage('   \r'), null);
});
