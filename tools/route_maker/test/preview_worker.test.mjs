// preview_worker.test.mjs — PREVIEW's engine off the page's thread (RMP1-A2u): engine.mjs (the page's side: progress,
// cancel, the 20 s stop, a new Worker after any failure) and worker.mjs (the Worker's side: a new instance after a
// run that returns non-zero, P-W1f), each against a fake of the other.
import test from 'node:test';
import assert from 'node:assert/strict';
import { makeEngine, TIMEOUT_MS, TIMEOUT_WORDS, LOAD_MS, LOAD_WORDS, PREVIEW_COMING } from '../src/host/engine.mjs';

/** A fake Worker the test drives: what the page posted, and a way to answer. */
function fakeWorkers() {
    const made = [];
    class W {
        constructor(url, opts) {
            this.url = url;
            this.opts = opts;
            this.posted = [];
            this.terminated = false;
            made.push(this);
        }
        postMessage(m) {
            this.posted.push(m);
        }
        terminate() {
            this.terminated = true;
        }
        answer(m) {
            this.onmessage({ data: m });
        }
    }
    return { W, made };
}
function fakeTimers() {
    const t = { now: 0, list: [] };
    t.setTimeout = (f, ms) => (t.list.push({ f, at: t.now + ms, ms }), t.list.length);
    t.clearTimeout = (h) => h && (t.list[h - 1].f = null);
    t.advance = (ms) => {
        t.now += ms;
        for (const x of t.list) {
            if (x.f && x.at <= t.now) {
                const f = x.f;
                x.f = null;
                f();
            }
        }
    };
    return t;
}
const ROBOTS = { one: { id: 'one' } };
const engineWith = (opts = {}) => {
    const w = fakeWorkers();
    const timers = fakeTimers();
    const e = makeEngine({ source: { url: 'engine/worker.mjs' }, browser: { preview: true }, WorkerClass: w.W, timers, ...opts });
    return { e, w, timers };
};

test('no engine in this build, or a browser too old: PREVIEW is not offered, and says why', async () => {
    const none = makeEngine({ source: null, browser: { preview: true }, WorkerClass: class {} });
    assert.equal(none.available, false);
    assert.equal(none.why, PREVIEW_COMING);
    assert.deepEqual(await none.preview({}, ROBOTS), { ok: false, words: PREVIEW_COMING });
    const old = makeEngine({ source: { url: 'x' }, browser: { preview: false }, WorkerClass: class {} });
    assert.equal(old.available, false);
    assert.match(old.why, /too old to run the preview/);
});

test('a PREVIEW: a CLASSIC Worker, its phases passed on, its result returned; the Worker kept for the next', async () => {
    const { e, w } = engineWith();
    const phases = [];
    const p = e.preview({ steps: [] }, ROBOTS, (x) => phases.push(x));
    assert.equal(w.made.length, 1);
    assert.equal(w.made[0].opts, undefined, 'classic: Chromium refuses a module Worker from a blob: URL over file://');
    const sent = w.made[0].posted[0];
    assert.deepEqual(sent.robots, ROBOTS);
    assert.equal(sent.pair, false, 'robot two only when the page sends it');
    w.made[0].answer({ id: sent.id, phase: 'starting' });
    w.made[0].answer({ id: sent.id, phase: 'running' });
    w.made[0].answer({ id: sent.id, done: { ok: true, result: { one: 1 } } });
    assert.deepEqual(await p, { ok: true, result: { one: 1 } });
    assert.deepEqual(phases, ['starting', 'running']);
    e.preview({}, { ...ROBOTS, two: { id: 'two' } });
    assert.equal(w.made.length, 1, 'the same Worker (its module already made)');
    assert.equal(w.made[0].posted[1].pair, true);
});

test('THE 20 s STOP: a run that never answers is terminated, the page says so, and the next PREVIEW gets a new Worker', async () => {
    assert.equal(TIMEOUT_MS, 20000);
    const { e, w, timers } = engineWith();
    const p = e.preview({}, ROBOTS);
    w.made[0].answer({ id: 1, phase: 'running' });
    timers.advance(19999);
    assert.equal(w.made[0].terminated, false);
    timers.advance(1);
    assert.deepEqual(await p, { ok: false, words: TIMEOUT_WORDS });
    assert.equal(w.made[0].terminated, true);
    w.made[0].answer({ id: 1, done: { ok: true, result: {} } });  // too late: nothing is waiting for it
    e.preview({}, ROBOTS);
    assert.equal(w.made.length, 2);
});

test('CANCEL: the Worker goes at once, the run says "stopped", a late answer is dropped, the next PREVIEW starts afresh', async () => {
    const { e, w, timers } = engineWith();
    const p = e.preview({}, ROBOTS);
    assert.deepEqual(await e.preview({}, ROBOTS), { ok: false, words: 'A PREVIEW is already running.' });
    e.cancel();
    assert.deepEqual(await p, { ok: false, cancelled: true, words: 'PREVIEW stopped.' });
    assert.equal(w.made[0].terminated, true);
    assert.equal(timers.list[0].f, null, 'its timer cleared');
    const q = e.preview({}, ROBOTS);
    assert.equal(w.made.length, 2);
    w.made[0].answer({ id: 1, done: { ok: true, result: { stale: true } } });
    w.made[1].answer({ id: w.made[1].posted[0].id, done: { ok: true, result: { fresh: true } } });
    assert.deepEqual(await q, { ok: true, result: { fresh: true } });
});

test('one PREVIEW at a time; an answer for another run is ignored', async () => {
    const { e, w } = engineWith();
    const p = e.preview({}, ROBOTS);
    assert.deepEqual(await e.preview({}, ROBOTS), { ok: false, words: 'A PREVIEW is already running.' });
    w.made[0].answer({ id: 999, done: { ok: true, result: { wrong: true } } });
    w.made[0].answer({ id: 1, done: { ok: true, result: { right: true } } });
    assert.deepEqual((await p).result, { right: true });
});

test('an engine that does not load: the reason said, everything else works, and PREVIEW may be pressed again', async () => {
    const { e, w } = engineWith();
    const p = e.preview({}, ROBOTS);
    let prevented = false;
    w.made[0].onerror({ message: 'blocked', preventDefault: () => { prevented = true; } });
    const out = await p;
    assert.equal(out.ok, false);
    assert.match(out.words, /did not load \(blocked\)\. Everything else works/);
    assert.ok(prevented && w.made[0].terminated);
    e.preview({}, ROBOTS);
    assert.equal(w.made.length, 2);
});

// ── worker.mjs, in a fake Worker scope ──
async function loadWorker(backend) {
    const posted = [];
    const made = { n: 0 };
    globalThis.self = { postMessage: (m) => posted.push(m), shulibBackend: async () => backend(++made.n) };
    await import(`../src/worker.mjs?${Math.random()}`);
    const send = async (m) => {
        await globalThis.self.onmessage({ data: m });
        return posted.filter((x) => x.id === m.id);
    };
    return { send, made };
}
const REC = { word: 'FINISHED' };

test('worker.mjs: "starting" once (the module made), "running" each time, then the record', async () => {
    const { send, made } = await loadWorker(() => ({ preview: () => ({ rc: 0, record: REC }) }));
    assert.deepEqual(await send({ id: 1, route: {}, robots: ROBOTS }),
                     [{ id: 1, phase: 'starting' }, { id: 1, phase: 'running' },
                      { id: 1, done: { ok: true, result: { one: REC, two: null, verdict: null }, pairWords: '' } }]);
    assert.deepEqual((await send({ id: 2, route: {}, robots: ROBOTS })).map((m) => m.phase || 'done'), ['running', 'done']);
    assert.equal(made.n, 1, 'one instance for two clean runs');
});

test('worker.mjs: a NEW instance after any run that returns non-zero (P-W1f), never after a clean one', async () => {
    const rcs = [1, 0, 2, 0];
    const { send, made } = await loadWorker(() => ({ preview: () => {
        const rc = rcs.shift();
        return rc === 2 ? { rc, record: null, words: 'refused' } : { rc, record: REC };
    } }));
    await send({ id: 1, route: {}, robots: ROBOTS });
    await send({ id: 2, route: {}, robots: ROBOTS });
    assert.equal(made.n, 2, 'remade after rc 1');
    const refused = await send({ id: 3, route: {}, robots: ROBOTS });
    assert.deepEqual(refused[refused.length - 1].done, { ok: false, words: 'refused' });
    await send({ id: 4, route: {}, robots: ROBOTS });
    assert.equal(made.n, 3, 'remade after rc 2');
});

test('worker.mjs: robot two — the pair when the engine has it, plain words when it does not; a throw ends cleanly', async () => {
    const withPair = await loadWorker(() => ({ preview: () => ({ rc: 0, record: REC }), pair: () => ({ rc: 0, record: 'two', verdict: 'v' }) }));
    const a = await withPair.send({ id: 1, route: {}, robots: { one: {}, two: {} }, pair: true });
    assert.deepEqual(a.map((m) => m.phase || 'done'), ['starting', 'running', 'two', 'done']);
    assert.deepEqual(a[3].done.result, { one: REC, two: 'two', verdict: 'v' });
    const noPair = await loadWorker(() => ({ preview: () => ({ rc: 0, record: REC }) }));
    const b = await noPair.send({ id: 1, route: {}, robots: { one: {}, two: {} }, pair: true });
    assert.deepEqual(b[b.length - 1].done, { ok: true, result: { one: REC, two: null, verdict: null }, pairWords: '', noPair: true },
        'no pair mode: said by the page, in its robots\' names (RMP1-L)');
    // robot one's run returned 1 (its instance put away): an engine with no pair mode is not reloaded to find that out
    const rc1 = await loadWorker(() => ({ preview: () => ({ rc: 1, record: REC }) }));
    const d = await rc1.send({ id: 1, route: {}, robots: { one: {}, two: {} }, pair: true });
    assert.deepEqual(d.map((m) => m.phase || 'done'), ['starting', 'running', 'done']);
    assert.equal(rc1.made.n, 1);
    const throws = await loadWorker(() => ({ preview: () => { throw new Error('route lines: a preset is missing'); } }));
    const c = await throws.send({ id: 1, route: {}, robots: ROBOTS });
    assert.deepEqual(c[c.length - 1].done, { ok: false, words: 'The preview engine stopped: route lines: a preset is missing' });
    await throws.send({ id: 2, route: {}, robots: ROBOTS });
    assert.equal(throws.made.n, 2, 'remade after a throw');
});

test('worker.mjs: robot two\'s run is held to P-W1f too — a pair that returns non-zero, or one after robot one did, gets a NEW instance', async () => {
    const pairRc = await loadWorker((n) => ({ preview: () => ({ rc: 0, record: REC }), pair: () => ({ rc: 1, record: `two-${n}`, verdict: 'v' }) }));
    await pairRc.send({ id: 1, route: {}, robots: { one: {}, two: {} }, pair: true });
    assert.equal(pairRc.made.n, 1);
    const next = await pairRc.send({ id: 2, route: {}, robots: ROBOTS });
    assert.equal(next[0].phase, 'starting', 'remade after robot two\'s rc 1');
    assert.equal(pairRc.made.n, 2);
    const oneRc = await loadWorker((n) => ({ preview: () => ({ rc: 1, record: REC }), pair: () => ({ rc: 0, record: `two-${n}`, verdict: 'v' }) }));
    const a = await oneRc.send({ id: 1, route: {}, robots: { one: {}, two: {} }, pair: true });
    assert.deepEqual(a.map((m) => m.phase || 'done'), ['starting', 'running', 'starting', 'two', 'done']);
    assert.equal(a.at(-1).done.result.two, 'two-2', 'robot two ran on the NEW instance, not the one robot one left');
});

test('LOADING gets 60 s of its own (a slow connection), each phase its own stop; a run after the load gets 20 s', async () => {
    assert.equal(LOAD_MS, 60000);
    const { e, w, timers } = engineWith();
    const p = e.preview({}, ROBOTS);
    timers.advance(59000);
    w.made[0].answer({ id: 1, phase: 'starting' });  // the module begins to load: its 60 s start again
    timers.advance(59000);
    assert.equal(w.made[0].terminated, false, 'two slow minutes, each phase inside its own time');
    w.made[0].answer({ id: 1, phase: 'running' });
    timers.advance(19999);
    assert.equal(w.made[0].terminated, false);
    w.made[0].answer({ id: 1, done: { ok: true, result: {} } });
    assert.equal((await p).ok, true);
    const q = e.preview({}, ROBOTS);  // the engine is loaded: the 20 s from the start
    timers.advance(20000);
    assert.deepEqual(await q, { ok: false, words: TIMEOUT_WORDS });
    const r = e.preview({}, ROBOTS);  // a NEW Worker: loading again
    timers.advance(60000);
    assert.deepEqual(await r, { ok: false, words: LOAD_WORDS });
});

test('a Worker that cannot even be made (a refused script, a refused message): said, nothing left running, PREVIEW again', async () => {
    let fail = true;
    const { W, made } = fakeWorkers();
    class Refused extends W {
        constructor(...a) {
            super(...a);
            if (fail) {
                throw new Error('SecurityError');
            }
        }
    }
    const timers = fakeTimers();
    const e = makeEngine({ source: { url: 'x' }, browser: { preview: true }, WorkerClass: Refused, timers });
    const out = await e.preview({}, ROBOTS);
    assert.equal(out.ok, false);
    assert.match(out.words, /did not start \(SecurityError\)\. Everything else works/);
    assert.ok(timers.list.every((x) => x.f === null), 'no stop left armed');
    fail = false;
    e.preview({}, ROBOTS);
    assert.equal(made.length, 2);
    assert.equal(made[1].posted.length, 1, 'the next PREVIEW runs: nothing was left "running"');
});
