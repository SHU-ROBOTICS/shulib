// preview.test.mjs — PREVIEW IN THE PAGE, in a real browser (RMP1-A2u; brief A.4, A.7's A2u row, §2.5). The engine
// here is the STAND-IN (test/standin_engine.mjs): recorded runs of the real engine, played through the page's own
// Worker, progress, Cancel and 20 s stop — every screen state, while the WebAssembly module is built beside it.
// Runs when ROUTE_MAKER_BROWSER is set, on the built site (ROUTE_MAKER_SITE); every page fails on a CSP violation, a
// page error or a console error. With ROUTE_MAKER_SHOTS, §2.5.2's PREVIEW screens in light and dark, 1440 and 1024.
//
//   ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/preview.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { standinScript, fixture } from '../standin_engine.mjs';
import { poseAt, thinPath, sampleIndexAt, playbackOf } from '../../src/model/playback.mjs';
import { squareCorners } from '../../src/model/geometry.mjs';
import { TIMEOUT_WORDS } from '../../src/host/engine.mjs';
import { noStructures } from './harness.mjs';

const BROWSER = process.env.ROUTE_MAKER_BROWSER || '';
const REPO = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..', '..', '..', '..');
const SITE = path.resolve(process.env.ROUTE_MAKER_SITE || path.join(REPO, 'build', 'route_maker_site'));
const SHOTS = process.env.ROUTE_MAKER_SHOTS ? path.resolve(process.env.ROUTE_MAKER_SHOTS) : '';
const skip = !BROWSER ? 'set ROUTE_MAKER_BROWSER (firefox, chromium, msedge or chrome) to run the page tests' : false;
const ALL = ['ready', 'failed', 'carry', 'cut', 'pair_ok', 'pair_fail', 'through', 'two_starts', 'threw'];
const REAL = fs.existsSync(path.join(SITE, 'engine', 'worker.mjs'));  // a site built with --engine: the real module
// in CI, a tree WITH the engine's sources must run it: a site built without it would skip the real engine silently
const MUST = !!process.env.CI && fs.existsSync(path.join(REPO, 'tools', 'route_maker', 'wasm', 'CMakeLists.txt'));
const NATIVE = process.env.ROUTE_MAKER_NATIVE || path.join(REPO, 'build', 'test', 'shulib_route_maker');
// the native twin's records made from this tree by CI's Linux job (test/fixtures/record.mjs --out): the browser rows'
const NATIVE_RECORDS = process.env.ROUTE_MAKER_NATIVE_RECORDS ? path.resolve(process.env.ROUTE_MAKER_NATIVE_RECORDS) : '';
// the one warning allowed (W6): Firefox on every instance of the module's legacy exception instructions
const ALLOWED_WARNINGS = ['The WebAssembly exception handling \'try\' instruction is deprecated'];
const SHARED = { present: true, robots: ['one', 'two'], file: 'src/team/skills_route.cpp' };  // robot two's update built in
const ONE_ROBOT = { present: false, robots: ['one'], file: 'src/team/skills_route.cpp' };  // a build without it (no shared file)
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1024, 768], ['dark', 1024, 768]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));

let browser;
let server;
let base;

function serve(root) {
    const mime = { '.html': 'text/html; charset=utf-8', '.mjs': 'text/javascript', '.json': 'application/json', '.wasm': 'application/wasm' };
    return http.createServer((req, res) => {
        const f = path.join(root, decodeURIComponent(new URL(req.url, 'http://x').pathname));
        const ok = f.startsWith(root) && fs.existsSync(f) && !fs.statSync(f).isDirectory();
        res.writeHead(ok ? 200 : 404, { 'content-type': mime[path.extname(f)] || 'application/octet-stream' });
        res.end(ok ? fs.readFileSync(f) : undefined);
    });
}

/** The page with the stand-in engine; every console error, page error, CSP report and outside request recorded. */
async function openPage({ form = 'site', viewport = { width: 1440, height: 900 }, scheme = 'light', engine = {}, names = ALL, shared = false,
                          reducedMotion = 'no-preference' } = {}) {
    const ctx = await browser.newContext({ viewport, colorScheme: scheme, reducedMotion });
    const page = await ctx.newPage();
    const problems = [];
    page.on('console', (m) => (m.type() === 'error' || (m.type() === 'warning' && !ALLOWED_WARNINGS.some((w) => m.text().includes(w))))
        && problems.push(`console ${m.type()}: ${m.text()}`));
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('request', (r) => {
        const u = r.url();
        if (!(form === 'site' ? u.startsWith(base) : u.startsWith('file:')) && !/^(data|blob):/.test(u)) {
            problems.push(`a request outside the page: ${u}`);
        }
    });
    await page.goto(form === 'site' ? `${base}/app.html` : pathToFileURL(path.join(SITE, 'shulib-route-builder.html')).href);
    await page.waitForFunction(() => document.documentElement.dataset.ready === '1');
    await page.evaluate(() => {
        window.__csp = [];
        document.addEventListener('securitypolicyviolation', (e) => window.__csp.push(`${e.violatedDirective} ${e.blockedURI}`));
    });
    if (engine) {  // null: the page's own engine (a site built with --engine)
        await page.evaluate(([text, t, s]) => window.routeBuilder.useEngine({ text, timeoutMs: t, sharedRoute: s }),
                            [engine.text ?? standinScript(names, engine), engine.timeoutMs, shared ? SHARED : null]);
    }
    await page.evaluate((r) => window.routeBuilder.useRobots(r), noStructures());  // RMP1-H: the recordings predate the check
    const coach = page.getByRole('button', { name: 'Got it' });
    if (await coach.count()) {
        await coach.click();
    }
    return { page, done: async () => {
        const csp = await page.evaluate(() => window.__csp);
        await ctx.close();
        return [...problems, ...csp.map((c) => `CSP violation: ${c}`)];
    } };
}

/** This page re-baked as a build without robot two's update: PREVIEW plays robot one alone. */
const withoutTwo = (page) => page.evaluate(([text, s]) => window.routeBuilder.useEngine({ text, sharedRoute: s }), [standinScript(ALL), ONE_ROBOT]);
/** Open a recorded fixture's project file through the page's own Open. */
async function openFixture(page, name) {
    const f = fixture(name);
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
    await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(f.text, 'utf8') });
    // the route ITSELF (two fixtures share a step count: waiting on the count raced the file's read in Chromium)
    await page.waitForFunction((want) => JSON.stringify(window.routeBuilder.route()) === want, JSON.stringify(f.route));
    return f;
}
const pv = (page) => page.evaluate(() => window.routeBuilder.preview());
/** The step list, opened where a narrow window keeps it closed (1024 wide). */
async function openSteps(page) {
    const toggle = page.getByRole('button', { name: 'Steps', exact: true });
    if (await toggle.isVisible() && await page.evaluate(() => document.querySelector('.app').dataset.left !== 'open')) {
        await toggle.click();
        await page.waitForFunction(() => document.querySelector('.pane.left').getAnimations().length === 0);
    }
}
/** The step list and the step's pane closed again where they cover the field (a narrow window). */
async function closePanes(page) {
    for (const name of ['Close the steps', 'Close the step']) {
        const b = page.getByRole('button', { name, exact: true });
        if (await b.isVisible() && await page.evaluate(() => !!(document.querySelector('.app').dataset.left || document.querySelector('.app').dataset.right))) {
            await b.click();
        }
    }
    await page.waitForFunction(() => [...document.querySelectorAll('.pane')].every((e) => e.getAnimations().length === 0));
}
/** PREVIEW by the keyboard, then wait for the run to show. */
async function preview(page) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('Control+Enter');
    await page.waitForFunction(() => window.routeBuilder.preview().phase !== 'running');
}
/** Pause and put the playhead at `t` with the keys only (Home, then Shift+Right in 0.1 s). */
async function stop(page) {
    await page.evaluate(() => window.routeBuilder.pause());
}
async function shot(page, name) {
    if (SHOTS) {
        fs.mkdirSync(SHOTS, { recursive: true });
        await page.waitForFunction(() => !document.querySelector('.toast'), null, { timeout: 10000 });  // at rest
        await page.waitForTimeout(400);
        await page.screenshot({ path: path.join(SHOTS, `${BROWSER}-${name}.png`) });
    }
}
/** The page's picture points of the record's robot square at `t` (what the page must draw, from the record alone). */
async function expectedSquare(page, samples, t) {
    const pic = await page.evaluate(() => JSON.parse(document.getElementById('shulib-field').textContent));
    const q = poseAt(samples, t);
    return squareCorners(q.x, q.y, q.headingDeg, 15).map((c) => [pic.originPx[0] + c.x * pic.pxPerIn, pic.originPx[1] - c.y * pic.pxPerIn]);
}
const drawn = (page, who, part = 'g-run-robot') => page.evaluate(([w, c]) => (document.querySelector(`.g-run.${w} .${c}`).getAttribute('points') || '')
    .trim().split(/\s+/).filter(Boolean).map((p) => p.split(',').map(Number)), [who, part]);
/** Field inches to the page's picture points. */
async function toPx(page) {
    const pic = await page.evaluate(() => JSON.parse(document.getElementById('shulib-field').textContent));
    return (x, y) => [pic.originPx[0] + x * pic.pxPerIn, pic.originPx[1] - y * pic.pxPerIn];
}
const pctOf = (style) => Number(/(-?[\d.]+)%/.exec(style)[1]);
/** How many px of each step's span show on the track, none under another ({n: px}, sampled every 0.25 px). */
const shown = (page) => page.evaluate(() => {
    const css = document.head.appendChild(Object.assign(document.createElement('style'), { textContent: '.track .tl-seg { pointer-events: auto !important }' }));
    const r = document.querySelector('.track').getBoundingClientRect();
    const out = {};
    for (let x = r.left + 0.125; x < r.right; x += 0.25) {
        const n = /^Step (\d+)/.exec(document.elementFromPoint(x, r.top + r.height / 2)?.closest('.tl-seg')?.title)?.[1];
        out[n] = (out[n] || 0) + 0.25;
    }
    css.remove();
    return out;
});
/** About this run's line starting `words` is in view as it opens (not scrolled to). */
const aboutLineInView = (page, words) => page.evaluate((w) => {
    const box = document.querySelector('.dock-details');
    const line = [...box.querySelectorAll('p')].find((e) => e.textContent.startsWith(w));
    return !!line && box.scrollTop === 0 && line.getBoundingClientRect().bottom <= box.getBoundingClientRect().bottom;
}, words);
const NO_TWO = 'PREVIEW plays White Bot: Black Bot\'s mirrored run is not in this build of the route builder yet.';
function near(a, b, what) {
    assert.equal(a.length, b.length, `${what}: ${a.length} points drawn, ${b.length} in the record`);
    a.forEach((p, i) => assert.ok(Math.hypot(p[0] - b[i][0], p[1] - b[i][1]) < 0.01, `${what}: corner ${i} ${p} vs ${b[i]}`));
}

test.describe('PREVIEW in the page', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        const pw = await import('playwright');
        browser = BROWSER === 'firefox' ? await pw.firefox.launch({ firefoxUserPrefs: { 'privacy.bounceTrackingProtection.mode': 0 } })
            : await pw.chromium.launch(BROWSER === 'msedge' || BROWSER === 'chrome' ? { channel: BROWSER } : {});
        server = serve(SITE);
        await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
        base = `http://127.0.0.1:${server.address().port}`;
    });
    test.after(async () => {
        await browser?.close();
        server?.close();
    });

    test('a route plays: the robot IS the record\'s pose at every moment, the plan stays dashed, the steps lit; Space, Left / Right, Shift, Home / End, a click on a step', async () => {
        const p = await openPage();
        const { page } = p;
        const f = await openFixture(page, 'ready');
        const samples = f.result.one.samples;
        // WG4's other half: a browser that CAN run the engine is told nothing (RM31: one byte of its check changed)
        assert.equal(await page.evaluate(() => window.shulibBrowser.preview), true);
        assert.equal(await page.locator('#too-old-preview').count(), 0);
        assert.equal(await page.locator('#preview').isEnabled(), true);
        await preview(page);
        const s0 = await pv(page);
        assert.equal(s0.phase, 'open');
        assert.equal(s0.playing, true, 'it plays at once');
        await page.waitForFunction(() => !window.routeBuilder.preview().playing, null, { timeout: 20000 });
        const end = await pv(page);
        assert.equal(end.t, end.duration, 'it plays to the end and stops there');
        assert.equal(end.duration, samples[samples.length - 1][0]);
        assert.ok(await page.locator('.g-plan').count() > 0, 'the plan is still drawn, dashed, beside the run');
        assert.equal(await page.locator('.step.pv-ok').count(), 5, 'every step lit as done');
        const play = playbackOf(f.result);
        for (const [i, s] of play.one.steps.entries()) {
            assert.equal(await page.locator(`.step[data-step="${i}"] .step-run span`).first().textContent(), `Done in ${(s.t1 - s.t0).toFixed(2)} s`);
        }
        near(await drawn(page, 'one'), await expectedSquare(page, samples, end.t), 'at the end');
        // the keys: Home, then Right walks every step's start
        await page.keyboard.press('Home');
        assert.equal((await pv(page)).t, 0);
        assert.equal((await drawn(page, 'one', 'g-run-done')).length, 2, 'back at the start, the path driven is the start alone');
        assert.match(await page.locator('.dock-words').textContent(), /^PLACEHOLDER numbers/, 'the step playing used a PLACEHOLDER: the dock says so');
        // the path: the whole run faint, the part driven so far solid — the record's own paces, thinned
        const px = await toPx(page);
        const path = thinPath(samples);
        near(await drawn(page, 'one', 'g-run-all'), path.map((q) => px(q.x, q.y)), 'the whole path');
        const starts = f.result.one.pageSteps.map((s) => samples[s.first][0]);
        for (let k = 1; k < starts.length; ++k) {
            await page.keyboard.press('ArrowRight');
            const st = await pv(page);
            assert.equal(st.t, starts[k], `Right: step ${k + 1}'s start`);
            assert.equal(st.step, k, 'that step is the one playing');
            assert.equal(await page.locator(`.step[data-step="${k}"].pv-playing`).count(), 1, 'its card highlighted');
            near(await drawn(page, 'one'), await expectedSquare(page, samples, st.t), `at step ${k + 1}`);
            const pose = poseAt(samples, st.t);
            const upTo = sampleIndexAt(samples, st.t);
            near(await drawn(page, 'one', 'g-run-done'), [...path.filter((q) => q.i <= upTo), pose].map((q) => px(q.x, q.y)), `driven by step ${k + 1}`);
            const rad = (pose.headingDeg * Math.PI) / 180;
            near((await drawn(page, 'one', 'g-run-front')).slice(0, 1), [px(pose.x + Math.cos(rad) * 6.5, pose.y + Math.sin(rad) * 6.5)], 'the front points ahead');
            // the timeline: the playhead and the time where the run is
            assert.ok(Math.abs(pctOf(await page.locator('.playhead').getAttribute('style')) - (100 * st.t) / st.duration) < 1e-3);
            assert.equal(await page.locator('.dock-time').textContent(), `${st.t.toFixed(2)} s / ${st.duration.toFixed(2)} s`);
        }
        await page.keyboard.press('Shift+ArrowRight');
        assert.ok(Math.abs((await pv(page)).t - (starts[4] + 0.1)) < 1e-9, 'Shift+Right: 0.1 s on');
        await page.keyboard.press('ArrowLeft');
        assert.equal((await pv(page)).t, starts[4], 'Left: back to this step\'s start');
        await page.keyboard.press('End');
        assert.equal((await pv(page)).t, end.duration);
        // Space: a tap plays and pauses; from the end it starts again
        await page.keyboard.press('Space');
        const again = await pv(page);
        assert.equal(again.playing, true);
        assert.ok(again.t < 1, `from the start again (t ${again.t})`);
        await page.keyboard.press('Space');
        assert.equal((await pv(page)).playing, false);
        // holding Space to pan the field (a drag) does not play
        await page.keyboard.down(' ');
        const fb = await page.locator('.field-svg').boundingBox();
        await page.mouse.move(fb.x + fb.width / 2, fb.y + fb.height / 3);
        await page.mouse.down();
        await page.mouse.move(fb.x + fb.width / 2 + 40, fb.y + fb.height / 3 + 10);
        await page.mouse.up();
        await page.keyboard.up(' ');
        assert.equal((await pv(page)).playing, false, 'Space held for a pan is not a tap');
        // a click on a step card goes to where it began
        await page.locator('.step[data-step="2"]').click();
        assert.equal((await pv(page)).t, starts[2]);
        assert.match(await page.locator('.dock-step').textContent(), /^Step 3: Curve through 2 points$/);
        await page.locator('.step[data-step="3"]').focus();  // and by the keyboard
        await page.keyboard.press('Enter');
        assert.equal((await pv(page)).t, starts[3]);
        // the timeline: a click on the track goes to that moment
        const box = await page.locator('.track').boundingBox();
        await page.mouse.click(box.x + box.width / 2, box.y + box.height / 2);
        assert.ok(Math.abs((await pv(page)).t - end.duration / 2) < end.duration / 50);
        assert.deepEqual(await p.done(), []);
    });

    test('the TIMELINE: every step\'s span where and how it went; a drag follows the pointer and pauses; the speeds; the run against the 60 s', async () => {
        const cut = fixture('cut').result.one;
        const pageSteps = cut.pageSteps.map((q, i) => (i === 0 ? { ...q, last: 2 } : i === 1 ? { ...q, first: 2 } : q));
        const starts = fixture('two_starts').result.one.pageSteps.map((q, i) => (i === 0 ? { ...q, verdict: 'failedWentOn' } : q));
        const p = await openPage({ engine: { patch: { cut: { pageSteps }, two_starts: { pageSteps: starts } } } });
        const { page } = p;
        const f = await openFixture(page, 'carry');
        await preview(page);
        await stop(page);
        const play = playbackOf(f.result);
        const segs = (await page.evaluate(() => [...document.querySelectorAll('.track .tl-seg')].map((e) => ({ cls: e.className, style: e.getAttribute('style'),
                                                                                                           w: e.offsetWidth, title: e.title }))))
            .sort((a, b) => Number(/\d+/.exec(a.title)) - Number(/\d+/.exec(b.title)));
        const ran = play.one.steps.filter((x) => x.t0 !== null);
        assert.equal(segs.length, ran.length);
        for (const [k, x] of ran.entries()) {
            assert.match(segs[k].cls, new RegExp(`\\btl-seg ${x.state}\\b`), `step ${x.n}: ${x.state}`);
            assert.ok(Math.abs(pctOf(segs[k].style) - (100 * x.t0) / play.duration) < 1e-3, `step ${x.n} begins where it began`);
            assert.ok(x.t1 === x.t0 || segs[k].w >= 3, `step ${x.n}: a span you can see (${segs[k].w} px)`);
        }
        assert.ok(Math.abs(Number(/width:\s*([\d.]+)%/.exec(await page.locator('.budget-run').getAttribute('style'))[1]) - (100 * play.duration) / play.period) < 1e-3,
                  'the run against the period');
        // a drag: it pauses the run, the time follows the pointer, and Space plays on from there
        await page.keyboard.press('Space');
        assert.equal((await pv(page)).playing, true);
        await page.waitForFunction(() => document.querySelector('.dock').getAnimations().length === 0);  // risen
        const box = await page.locator('.track').boundingBox();
        const at = (q) => [box.x + box.width * q, box.y + box.height / 2];
        await page.mouse.move(...at(0.2));
        await page.mouse.down();
        assert.equal((await pv(page)).playing, false, 'a drag pauses');
        assert.ok(Math.abs((await pv(page)).t - 0.2 * play.duration) < play.duration / 100);
        assert.deepEqual(await page.locator('.track').boundingBox(), box, 'the scrubber stays where it was while the run played');
        await page.mouse.move(...at(0.45), { steps: 4 });
        await page.mouse.move(...at(0.7), { steps: 4 });
        assert.ok(Math.abs((await pv(page)).t - 0.7 * play.duration) < play.duration / 100, 'the time follows the pointer');
        await page.mouse.up();
        await page.mouse.move(...at(0.95));
        assert.ok(Math.abs((await pv(page)).t - 0.7 * play.duration) < play.duration / 100, 'after the button is let go, a move is only a move');
        await page.locator('.track').focus();
        await page.keyboard.press('Space');
        assert.equal((await pv(page)).playing, true);
        await stop(page);
        // a long sentence (step 1's failure) or a short one: the scrubber does not move, wide or narrow
        for (const k of ['Home', 'End', 'Home']) {
            await page.keyboard.press(k);
            assert.deepEqual(await page.locator('.track').boundingBox(), box, `${k}: the scrubber where it was`);
        }
        await page.setViewportSize({ width: 700, height: 900 });
        await page.waitForTimeout(100);
        const narrow = await page.locator('.track').boundingBox();
        for (const k of ['End', 'Home', 'End']) {
            await page.keyboard.press(k);
            assert.deepEqual(await page.locator('.track').boundingBox(), narrow, `700 px wide, ${k}: the scrubber where it was`);
        }
        await page.setViewportSize({ width: 1440, height: 900 });
        // the speeds: the one chosen is pressed, and a frame moves the run that many times as far
        for (const [label, k] of [['0.5×', 0.5], ['2×', 2], ['4×', 4], ['1×', 1]]) {
            await page.getByRole('button', { name: label, exact: true }).click();
            assert.equal(await page.getByRole('button', { name: label, exact: true }).getAttribute('aria-pressed'), 'true');
            assert.match(await page.getByRole('button', { name: label, exact: true }).getAttribute('title'), /^Play at /);
            await page.keyboard.press('Home');
            await page.evaluate(() => window.routeBuilder.frameAfter(50));
            assert.ok(Math.abs((await pv(page)).t - 0.05 * k) < 1e-9, `${label}: 50 ms of the screen is ${0.05 * k} s of the run`);
        }
        // a frame stamped before the last (its stamp is when it began, which can precede the press): time never runs back
        await page.keyboard.press('Home');
        await page.evaluate(() => window.routeBuilder.frameAfter(-20));
        assert.equal((await pv(page)).t, 0);
        assert.match(await page.locator('.dock-time').textContent(), /^0\.00 s \//);
        assert.match(await page.locator('.dock-step').textContent(), /^Step 1: /);
        // a step too short to see lies OVER the next one's start, its whole edge showing: a step that took no time
        // (carry's 3, skipped over), one of 20 ms in a 58 s run (cut's 1, made that short); of two that began on
        // one pace (two_starts' 1 and 2), the worse (1, made failed)
        for (const [name, n, px, next] of [['carry', 3, 1.75, 4], ['cut', 1, 2.75, 2], ['two_starts', 1, 1.75, 3]]) {
            if (name !== 'carry') {
                await openFixture(page, name);
                await preview(page);
                await stop(page);
            }
            const on = await shown(page);
            assert.ok(on[n] >= px, `${name}: step ${n} shows ${on[n]} px of its edge (${JSON.stringify(on)})`);
            assert.ok(on[next] > 100, `${name}: step ${next} after it still shows`);
        }
        assert.deepEqual(await p.done(), []);
    });

    test('BOTH robots, robot two alone failing: its step says so, the headline is red with both words, About this run lists robot two', async () => {
        const f = fixture('pair_ok');
        const two = f.result.two;
        const [, b, c] = two.pageSteps;
        const hit = { kind: 'THROUGH', element: 'blue Loader', s: b.first + 10, t: two.samples[b.first + 10][0], step: 2, depth: 1.5 };
        const bad = { ...two, through: true, throughAt: hit.s, field: [hit],
                      pageSteps: two.pageSteps.map((q) => (q === c ? { ...q, verdict: 'skipped', first: null, last: null } : q)) };
        const p = await openPage({ shared: true, engine: { two: { pair_ok: { record: bad, verdict: f.result.verdict } } } });
        const { page } = p;
        await openFixture(page, 'pair_ok');
        await preview(page);
        await stop(page);
        assert.equal(await page.locator('.step.pv-ok').count(), 3, 'robot one\'s own steps all went');
        assert.equal(await page.locator('.step[data-step="1"] .step-lib.two').textContent(),
                     `Black Bot: Went THROUGH the blue Loader. On the robot it hits the blue Loader at ${hit.t.toFixed(2)} s, in step 2 (1.50 in past its face): the host test plant has no walls.`);
        assert.match(await page.locator('.step[data-step="2"]').getAttribute('aria-label'), /Black Bot: Skipped: the route had stopped$/);
        assert.equal(await page.locator('.step[data-step="0"] .step-lib.two').count(), 0);
        assert.equal(await page.locator('.dock .chip.error').textContent(), 'White Bot: FINISHED · Black Bot: FINISHED, but THROUGH the blue Loader');
        await page.keyboard.press('Home');
        await page.waitForFunction(() => document.querySelector('.dock').getAnimations().length === 0);
        const box = await page.locator('.track').boundingBox();
        const lines = await page.locator('.dock-row.foot .dock-words').evaluate((e) => e.offsetHeight);
        await page.locator('.step[data-step="1"]').click();
        assert.deepEqual(await page.locator('.track').boundingBox(), box, 'a three-line sentence does not move the scrubber');
        assert.equal(await page.locator('.dock-row.foot .dock-words').evaluate((e) => e.offsetHeight), lines, 'the sentence keeps its two lines');
        assert.match(await page.locator('.dock-row.foot .dock-words').getAttribute('title'), /no walls\.$/, 'the whole sentence on hover');
        assert.match(await page.locator('.dock-words').textContent(), /^Step 2: Black Bot: Went THROUGH the blue Loader\./);
        await page.getByRole('button', { name: /About this run/ }).click();
        const about = await page.locator('.dock-details').textContent();
        assert.match(about, /Black Bot: FINISHED, but THROUGH the blue Loader/);
        assert.match(about, /Every line the library printed for Black Bot \(\d+\)/);
        assert.deepEqual(await p.done(), []);
    });

    test('the keys and the edges: Esc cancels a run; arrows typed in a box are the box\'s; a skipped step\'s card goes nowhere; reduced motion starts paused', async () => {
        const slow = await openPage({ engine: { delayMs: 3000 } });
        await openFixture(slow.page, 'failed');
        await slow.page.locator('#preview').click();
        await slow.page.locator('.dock .spinner').waitFor();
        await slow.page.locator('.field-svg').focus();
        await slow.page.keyboard.press('Escape');
        await slow.page.waitForFunction(() => window.routeBuilder.preview().phase === 'idle');
        assert.match(await slow.page.locator('.toast').textContent(), /PREVIEW stopped\./);
        assert.deepEqual(await slow.done(), []);

        // steps never reached (as the runner writes them after a throw: no span at all)
        const rec = fixture('failed').result.one;
        const pageSteps = rec.pageSteps.map((q, i) => (i < 2 ? q : { ...q, verdict: 'notReached', first: null, last: null }));
        const p = await openPage({ engine: { patch: { failed: { pageSteps } } } });
        const { page } = p;
        await openFixture(page, 'failed');
        await preview(page);
        await stop(page);
        const starts = rec.pageSteps.map((q) => rec.samples[q.first][0]);
        await page.locator('.step[data-step="1"]').click();
        assert.equal((await pv(page)).t, starts[1]);
        await page.locator('.step[data-step="2"]').click();
        assert.equal((await pv(page)).t, starts[1], 'a step never reached has no moment: its card leaves the time where it was');
        const box = page.locator('.pane.right input').first();
        await box.focus();
        await page.keyboard.press('ArrowRight');
        await page.keyboard.press('End');
        assert.equal((await pv(page)).t, starts[1], 'keys typed in a box move the box\'s cursor, not the run');
        assert.deepEqual(await p.done(), []);

        const calm = await openPage({ reducedMotion: 'reduce' });
        await openFixture(calm.page, 'ready');
        await preview(calm.page);
        const st = await pv(calm.page);
        assert.equal(st.phase, 'open');
        assert.equal(st.playing, false, 'reduced motion: it waits for Space');
        assert.equal(st.t, 0);
        assert.deepEqual(await calm.done(), []);
    });

    test('a record this page cannot read, and robot two refused before it moved: said in plain words', async () => {
        const p = await openPage({ engine: { patch: { ready: { schema: 2 } } } });
        await openFixture(p.page, 'ready');
        await preview(p.page);
        assert.equal((await pv(p.page)).phase, 'idle');
        assert.match(await p.page.locator('.dock-words').textContent(), /^This preview engine wrote a record this page cannot read\./);
        assert.deepEqual(await p.done(), []);
        const f = fixture('pair_ok');
        const refused = { ...f.result.two, samples: [], steps: [], refused: true, word: 'REFUSED', why: 'the test\'s own refusal of robot two' };
        const q = await openPage({ shared: true, engine: { two: { pair_ok: { record: refused, verdict: null } } } });
        await openFixture(q.page, 'pair_ok');
        await preview(q.page);
        const st = await pv(q.page);
        assert.equal(st.phase, 'open', 'robot one still plays');
        assert.equal(st.two, false);
        assert.equal(st.pairWords, 'Black Bot\'s mirrored run was refused (REFUSED): the test\'s own refusal of robot two');
        assert.deepEqual(await q.done(), []);
    });

    test('a run belongs to its route: an edit puts it away (said in the dock, the playing stopped), an undo brings it back WITHOUT running again', async () => {
        const p = await openPage();
        const { page } = p;
        await openFixture(page, 'ready');
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).run, 1, 'the engine ran it once');
        await page.locator('.step[data-step="0"]').click();
        await page.locator('.dock .btn.play').click();  // playing again, then the route changes under it
        assert.equal((await pv(page)).playing, true);
        await page.keyboard.press('Delete');
        assert.equal((await pv(page)).stale, true);
        assert.equal((await pv(page)).playing, false, 'a run put away stops playing');
        assert.equal(await page.locator('.g-run').count(), 0, 'no robot from an old route');
        assert.equal(await page.locator('.step.pv-ok').count(), 0, 'no step lit from an old route');
        assert.match(await page.locator('.dock').textContent(), /The route changed since this PREVIEW/);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        assert.equal((await pv(page)).stale, false);
        assert.equal(await page.locator('.g-run.one').count(), 1);
        await page.keyboard.press('Escape');
        assert.equal((await pv(page)).phase, 'idle', 'Esc closes it');
        await preview(page);
        assert.equal((await pv(page)).phase, 'open');
        assert.equal((await pv(page)).run, 1, 'the same run, from memory: the engine was not asked again');
        assert.deepEqual(await p.done(), []);
    });

    test('a failed step: red with the library\'s OWN line, the steps after it greyed, the dock saying it as the step plays', async () => {
        const p = await openPage();
        const { page } = p;
        await openFixture(page, 'failed');
        await preview(page);
        await stop(page);
        await page.keyboard.press('Home');
        await page.keyboard.press('ArrowRight');
        const failed = page.locator('.step[data-step="1"]');
        assert.match(await failed.getAttribute('class'), /pv-failed/);
        assert.equal(await failed.locator('.step-lib').textContent(),
                     "routine 'skills' STOPPED at step 2 (driveTo): motion TIMEOUT — skipping the rest; drive safed");
        assert.match(await failed.getAttribute('aria-label'), /In the preview: Failed: the route stopped here$/);
        assert.equal(await page.locator('.step.pv-skipped').count(), 2);
        assert.match(await page.locator('.dock-words').textContent(), /^Step 2: Failed: the route stopped here\. routine 'skills' STOPPED at step 2/);
        assert.match(await page.locator('.dock .chip.error').textContent(), /ROUTINE STOPPED/);
        // a narrower window wraps the dock's rows: the field still ends where the dock begins
        const dock = () => page.evaluate(() => {
            const d = document.querySelector('.dock');
            return { h: `${d.offsetHeight}px`, room: d.parentElement.style.getPropertyValue('--dock-h') };
        });
        const wide = await dock();
        await page.setViewportSize({ width: 700, height: 900 });
        await page.waitForFunction((h) => `${document.querySelector('.dock').offsetHeight}px` !== h, wide.h);
        await page.waitForTimeout(100);
        const narrow = await dock();
        assert.equal(narrow.room, narrow.h, `the room kept for the dock (was ${wide.h})`);
        assert.deepEqual(await p.done(), []);
    });

    test('a run that went THROUGH a wall (a route the page accepts; the real follower swung out): its step red, saying where', async () => {
        const p = await openPage();
        const { page } = p;
        await openFixture(page, 'through');
        await preview(page);
        await stop(page);
        const card = page.locator('.step[data-step="0"]');
        assert.match(await card.getAttribute('class'), /pv-failed/);
        assert.match(await card.locator('.step-lib').textContent(), /^On the robot it hits the right wall at \d+\.\d\d s/);
        assert.match(await card.getAttribute('aria-label'), /In the preview: Went THROUGH the right wall$/);
        const after = page.locator('.step[data-step="1"]');
        assert.match(await after.getAttribute('class'), /pv-skipped/, 'the step after it is not "done": the robot would not get there');
        assert.match(await after.getAttribute('aria-label'), /In the preview: After the THROUGH: the robot would not get here$/);
        assert.equal(await page.locator('.dock .chip.error').textContent(), 'FINISHED, but THROUGH the right wall');
        // the robot and its path after the THROUGH are drawn faded (RPV1); before it, not
        const f = fixture('through');
        const hit = f.result.one.field.find((x) => x.kind === 'THROUGH');
        await page.keyboard.press('Home');
        assert.equal(await page.locator('.g-run.one.through').count(), 0);
        assert.equal((await drawn(page, 'one', 'g-run-after')).length, 0);
        await page.keyboard.press('End');
        assert.equal(await page.locator('.g-run.one.through').count(), 1);
        const px = await toPx(page);
        const path = thinPath(f.result.one.samples);
        const k = path.filter((q) => q.i <= hit.s).length;
        near(await drawn(page, 'one', 'g-run-done'), path.slice(0, k).map((q) => px(q.x, q.y)), 'solid up to the THROUGH');
        const last = f.result.one.samples.at(-1);
        near(await drawn(page, 'one', 'g-run-after'), [...path.slice(k - 1), { x: last[1], y: last[2] }].map((q) => px(q.x, q.y)), 'faded after it');
        await page.getByRole('button', { name: /About this run/ }).click();
        const about = await page.locator('.dock-details').textContent();
        assert.ok(about.includes(`On the robot it hits the right wall at ${hit.t.toFixed(2)} s, in step 1 (`), 'the page\'s step, not RPV1\'s');
        assert.match(about, /Touches the right wall at \d+\.\d\d s, in step 1: normal, not a failure\./);
        await page.locator('.step[data-step="0"]').click();
        assert.deepEqual(await p.done(), []);
    });

    test('carry on and skip ahead; the 57 s cut amber against the run\'s 60 s', async () => {
        const p = await openPage();
        const { page } = p;
        await openFixture(page, 'carry');
        await preview(page);
        assert.equal(await page.locator('.step.pv-failed').count(), 2);
        assert.match(await page.locator('.step[data-step="2"]').getAttribute('class'), /pv-skipped/);
        assert.match(await page.locator('.step[data-step="2"] .step-lib').textContent(), /jumped — skipping to 'finish'/);
        await openFixture(page, 'cut');
        await preview(page);
        await stop(page);
        assert.match(await page.locator('.step[data-step="1"]').getAttribute('class'), /pv-cut/);
        assert.match(await page.locator('.dock .chip.warn').textContent(), /CUT BY DEADLINE/);
        assert.match(await page.locator('.budget').textContent(), /of the 60 s · cut at 57 s/);
        assert.deepEqual(await p.done(), []);
    });

    test('PLACEHOLDER facts marked: the step that used one, and the run\'s list of robot one\'s owed facts', async () => {
        const p = await openPage();
        const { page } = p;
        const f = await openFixture(page, 'ready');
        await preview(page);
        await stop(page);
        assert.equal(await page.locator('.step[data-step="0"] .ph').textContent(), 'PLACEHOLDER');
        assert.equal(await page.locator('.step[data-step="1"] .ph').count(), 0);
        await page.getByRole('button', { name: /About this run/ }).click();
        const details = await page.locator('.dock-details').textContent();
        assert.match(details, new RegExp(`${f.result.one.owed.length} of White Bot's facts are PLACEHOLDERS`));
        for (const fact of f.result.one.owed) {
            assert.ok(details.includes(fact), fact);
        }
        assert.match(details, /The host test plant, not the robot\./);
        assert.deepEqual(await p.done(), []);
    });

    test('the TWO-ROBOT preview: robot two drives the mirror from its own start, the 6 in band, the pair check\'s verdict and its moment', async () => {
        const p = await openPage({ shared: true });
        const { page } = p;
        const f = await openFixture(page, 'pair_fail');
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).two, true);
        assert.equal(await page.locator('.g-band').count(), 1, 'the band along the mirror line');
        // the band IS the pair check's gap: its long edges 3 in each side of y = -x (half of the verdict's 6 in)
        const band = await page.evaluate(() => {
            const pic = JSON.parse(document.getElementById('shulib-field').textContent);
            return document.querySelector('.g-band').getAttribute('points').split(' ').map((q) => q.split(',').map(Number))
                .map(([px, py]) => [(px - pic.originPx[0]) / pic.pxPerIn, (pic.originPx[1] - py) / pic.pxPerIn]);
        });
        for (const k of [1, 2, 4, 5]) {
            assert.ok(Math.abs(Math.abs(band[k][0] + band[k][1]) / Math.SQRT2 - f.result.verdict.gapIn / 2) < 1e-6, `band corner ${k}`);
        }
        assert.equal(await page.locator('.g-too-close').count(), 2, 'both robots where the check failed');
        const px = await toPx(page);
        const v = f.result.verdict;
        const closeAt = await page.evaluate(() => [...document.querySelectorAll('.g-too-close')].map((e) => e.getAttribute('points').split(' ').map((q) => q.split(',').map(Number))));
        for (const [k, q] of [v.one, v.two].entries()) {
            near(closeAt[k], squareCorners(q[0], q[1], (q[2] * 180) / Math.PI, 15).map((c) => px(c.x, c.y)), `the ${k ? 'second' : 'first'} robot where it was too close`);
        }
        assert.ok(Math.abs(pctOf(await page.locator('.pair-mark').getAttribute('style')) - (100 * v.failAt) / (await pv(page)).duration) < 1e-3, 'the mark at the moment');
        await page.locator('.pair-mark').click();
        const t = (await pv(page)).t;
        assert.equal(t, f.result.verdict.failAt);
        near(await drawn(page, 'two'), await expectedSquare(page, f.result.two.samples, t), 'robot two');
        near(await drawn(page, 'one'), await expectedSquare(page, f.result.one.samples, t), 'robot one');
        assert.match(await page.locator('.dock-words').textContent(), /come too close\. auton 'skills': the two robots come within 6\.0 in/);
        await openFixture(page, 'pair_ok');
        await preview(page);
        await stop(page);
        assert.equal(await page.locator('.pair-mark').count(), 0);
        assert.equal(await page.locator('.g-too-close').count(), 0);
        await page.getByRole('button', { name: /About this run/ }).click();
        assert.match(await page.locator('.dock-details').textContent(), /the two robots stay at least 80\.94 in apart/);
        assert.deepEqual(await p.done(), []);
    });

    test('robot two IS in this build (ROB1-BT): both robots play; a build without it plays ONE robot, and one plain line says so', async () => {
        assert.deepEqual(JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).sharedRoute.robots, ['one', 'two']);
        const p = await openPage();  // the site as built
        const { page } = p;
        await openFixture(page, 'pair_ok');
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).two, true);
        assert.equal(await page.locator('.g-run.two').count(), 1);
        assert.equal((await pv(page)).pairWords, '');
        // a build without robot two's update (the page re-baked): the SAME route plays robot one alone, drawn afresh
        await withoutTwo(page);
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).two, false);
        assert.equal(await page.locator('.g-run.two').count(), 0);
        assert.equal((await pv(page)).pairWords, NO_TWO);
        await page.getByRole('button', { name: /About this run/ }).click();
        assert.ok(await aboutLineInView(page, NO_TWO), 'About this run says it first, in view (not under the owed facts)');
        // RMP1-L: robot two in the build, but an engine with no pair mode -- the worker's flag, said in the robots' names
        await page.evaluate(([text, s]) => window.routeBuilder.useEngine({ text, sharedRoute: s }), [standinScript(ALL, { noPair: true }), SHARED]);
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).two, false);
        assert.equal((await pv(page)).pairWords, 'PREVIEW plays White Bot: Black Bot\'s mirrored run is not in this preview engine yet.');
        // an engine built BEFORE RMP1-L (the helper serves a checkout's page with the last build's or the published engine):
        // its worker's own sentence, in the old names, is said in the page's
        await page.evaluate(([text, s]) => window.routeBuilder.useEngine({ text, sharedRoute: s }), [standinScript(ALL, { noPair: true,
            oldWords: 'PREVIEW plays robot one: robot two\'s mirrored run is not in this preview engine yet.' }), SHARED]);
        await preview(page);
        await stop(page);
        assert.equal((await pv(page)).pairWords, 'PREVIEW plays White Bot: Black Bot\'s mirrored run is not in this preview engine yet.');
        // robot two's run, while it runs: its phase in its name
        await page.evaluate(([text, s]) => window.routeBuilder.useEngine({ text, sharedRoute: s }), [standinScript(ALL, { pairDelayMs: 3000 }), SHARED]);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.locator('.dock-phase', { hasText: 'Running Black Bot\'s mirrored route…' }).waitFor({ timeout: 10000 });
        await page.waitForFunction(() => window.routeBuilder.preview().phase !== 'running');
        await stop(page);
        assert.equal((await pv(page)).two, true);
        assert.deepEqual(await p.done(), []);
    });

    test('progress and CANCEL; the 20 s stop; a new engine instance after a run that returns non-zero', async () => {
        const slow = await openPage({ engine: { delayMs: 4000 } });
        await openFixture(slow.page, 'ready');
        await slow.page.locator('#preview').click();
        await slow.page.locator('.dock .spinner').waitFor();
        assert.match(await slow.page.locator('.dock-phase').textContent(), /Running the route through the real library|Starting the preview engine/);
        assert.equal(await slow.page.locator('#preview').isDisabled(), true, 'one PREVIEW at a time');
        await slow.page.waitForFunction(() => document.querySelector('.dock').getAnimations().length === 0);
        const [toast, dock] = await Promise.all(['.toast', '.dock'].map((q) => slow.page.locator(q).first().boundingBox()));
        assert.ok(toast.y + toast.height <= dock.y, `the open's toast sits above the dock, not on its words (${toast.y + toast.height} > ${dock.y})`);
        await slow.page.getByRole('button', { name: 'Cancel' }).click();
        await slow.page.waitForFunction(() => window.routeBuilder.preview().phase === 'idle');
        assert.equal(await slow.page.locator('.dock').isHidden(), true);
        assert.match(await slow.page.locator('.toast').textContent(), /PREVIEW stopped\./);
        assert.deepEqual(await slow.done(), []);

        const hung = await openPage({ engine: { hang: 'ready', timeoutMs: 1500 } });
        await openFixture(hung.page, 'ready');
        await preview(hung.page);
        assert.equal(await hung.page.locator('.dock-words').textContent(), TIMEOUT_WORDS);
        await openFixture(hung.page, 'failed');
        await preview(hung.page);
        assert.equal((await pv(hung.page)).phase, 'open', 'after the stop, a NEW Worker runs the next route');
        assert.deepEqual(await hung.done(), []);

        // both robots run each PREVIEW, robot two's mirrored run second (these fixtures record it): after robot one's rc 1,
        // robot two's run is the new instance's first, and carry's run is on it too
        const rc = await openPage({ engine: { rcOf: { pair_fail: 1 } } });
        await openFixture(rc.page, 'pair_ok');
        await preview(rc.page);
        assert.deepEqual([(await pv(rc.page)).instance, (await pv(rc.page)).two], [1, true]);
        await openFixture(rc.page, 'pair_fail');
        await preview(rc.page);
        assert.deepEqual([(await pv(rc.page)).instance, (await pv(rc.page)).two], [1, true], 'rc 1 ran on the instance it had...');
        await openFixture(rc.page, 'carry');
        await preview(rc.page);
        assert.equal((await pv(rc.page)).instance, 2, '...and the run after it gets a new one (P-W1f), only one');
        assert.deepEqual(await rc.done(), []);
    });

    test('the engine that does not load, or stops: the reason in plain words, the builder still works, PREVIEW may be pressed again', async () => {
        const broken = await openPage({ engine: { text: 'this is not a script (' } });
        await openFixture(broken.page, 'ready');
        await preview(broken.page);
        assert.match(await broken.page.locator('.dock-words').textContent(), /^The preview engine did not load \(.*\)\. Everything else works/);
        assert.equal(await broken.page.locator('#preview').isEnabled(), true);
        assert.ok(await broken.page.locator('#export').isEnabled());
        await broken.page.locator('.field-svg').focus();
        await broken.page.keyboard.press('Escape');
        assert.equal(await broken.page.locator('.dock').isHidden(), true, 'Esc closes the error, as its button says');
        // a page error is what this test plants: only that one is allowed
        assert.ok((await broken.done()).every((x) => /SyntaxError|syntax|expected expression|Unexpected/i.test(x)), 'only the planted error');

        const stops = await openPage({ engine: { failLoad: 'out of memory' } });
        await openFixture(stops.page, 'ready');
        await preview(stops.page);
        assert.equal(await stops.page.locator('.dock-words').textContent(), 'The preview engine stopped: out of memory');
        assert.deepEqual(await stops.done(), []);
    });

    test('not ready: no start, no step, or an error the robot would refuse — said, and the step with the error selected', async () => {
        const p = await openPage();
        const { page } = p;
        await preview(page);
        assert.match(await page.locator('.toast').textContent(), /Place White Bot's start first/);
        // a start and no step
        const empty = JSON.parse(fixture('ready').text);
        empty.routes[0].steps = [];
        const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
        await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(empty), 'utf8') });
        await page.waitForFunction(() => window.routeBuilder.route().start && window.routeBuilder.route().steps.length === 0);
        await page.waitForFunction(() => !document.querySelector('.toast'), null, { timeout: 10000 });
        await preview(page);
        assert.match(await page.locator('.toast').textContent(), /Add a step first: PREVIEW plays the route's steps\./);
        // an error on step 5, with step 1 selected: PREVIEW says so and selects step 5
        await openFixture(page, 'ready');
        await page.locator('.step[data-step="4"]').click();
        await page.locator('.pane.right input[name="seconds"]').fill('');
        await page.locator('.pane.right input[name="seconds"]').press('Enter');
        await page.locator('.step[data-step="0"]').click();
        assert.equal(await page.locator('.step[data-step="4"].selected').count(), 0);
        await preview(page);
        assert.match(await page.locator('.toast.error').textContent(), /PREVIEW runs a route the robot would run: fix step 5 first/);
        assert.equal((await pv(page)).phase, 'idle');
        assert.equal(await page.locator('.step[data-step="4"].selected').count(), 1, 'the step with the error is selected');
        assert.deepEqual(await p.done(), []);
    });

    test('RMP1-C: a route both robots run TOGETHER that the page finds too close still PREVIEWs -- robot one runs it the same either way; the too-close error waits for Export', async () => {
        const p = await openPage({ shared: true, engine: { together: ['pair_fail'] } });
        const { page } = p;
        const f = fixture('pair_fail');
        const project = JSON.parse(f.text);
        project.routes[0].together = true;
        const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
        await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(JSON.stringify(project), 'utf8') });
        await page.waitForFunction(() => window.routeBuilder.route().together === true);
        const errors = (await page.evaluate(() => window.routeBuilder.issues())).filter((x) => x.level === 'error');
        assert.deepEqual(errors.map((x) => [x.code, x.stepIndex]), [['RCP1', 1]], 'the page: too close during step 2');
        await preview(page);
        assert.equal((await pv(page)).phase, 'open', 'PREVIEW played');
        assert.equal(await page.locator('.toast.error').count(), 0);
        assert.equal((await pv(page)).two, true, 'robot two\'s mirrored run beside it');
        assert.match(await page.locator('.dock-words').textContent(), /come too close/);
        assert.deepEqual(await p.done(), []);
    });

    test('a route the library refuses before it moves (a record with no paces): said in plain words, nothing played', async () => {
        const why = 'the test\'s own refusal sentence';
        const p = await openPage({ engine: { refuse: { ready: why } } });
        const { page } = p;
        await openFixture(page, 'ready');
        await preview(page);
        assert.equal((await pv(page)).phase, 'idle');
        assert.equal(await page.locator('.dock-words').textContent(), `The library refused this route before the robot moved (REFUSED): ${why}`);
        assert.equal(await page.locator('.g-run').count(), 0);
        assert.deepEqual(await p.done(), [], 'no page error');
    });

    test('WG4 [W16]: a browser that cannot make the engine\'s Worker is told in plain words; the builder still opens', async () => {
        const ctx = await browser.newContext();
        await ctx.addInitScript(() => {
            window.Worker = class { constructor() { throw new Error('no Workers here'); } };
        });
        const page = await ctx.newPage();
        await page.goto(`${base}/app.html`);
        await page.waitForFunction(() => document.documentElement.dataset.ready === '1');
        assert.equal(await page.evaluate(() => window.shulibBrowser.preview), false);
        assert.match(await page.locator('#too-old-preview').textContent(), /too old to run the route preview/);
        assert.ok(await page.locator('#export').isVisible(), 'the builder itself opens');
        await ctx.close();
    });

    test('the ONE FILE over file://: PREVIEW through a blob Worker under its own policy, no request', async () => {
        const p = await openPage({ form: 'onefile' });
        await openFixture(p.page, 'failed');
        await preview(p.page);
        assert.equal((await pv(p.page)).phase, 'open');
        assert.equal(await p.page.locator('.step.pv-failed').count(), 1);
        assert.deepEqual(await p.done(), []);
    });

    test('THE REAL ENGINE (a site built with --engine): the page\'s own Worker runs the module, and its record is the engine\'s native twin\'s', { skip: !REAL && !MUST && 'the site has no engine (build.py --engine)' }, async () => {
        assert.ok(REAL, 'this tree has the engine (tools/route_maker/wasm/) but the site was built without it: build.py --engine');
        // the native twin's record of the same route, from this tree: made now (the program), or by CI's Linux job
        // (ROUTE_MAKER_NATIVE_RECORDS). Off CI with neither, the committed fixture less the API's and the ledger's own
        // members: its run is robot one's facts', so it goes red when they change (re-record: record.mjs)
        for (const name of ['ready', 'mechanisms']) {  // RMP1-E: the mechanisms' route too (the strip's)
            const f = fixture(name);
            let want;
            let skipKeys = ['samples', 'steps', 'field', 'mechanisms'];
            const kept = NATIVE_RECORDS && path.join(NATIVE_RECORDS, `${name}.preview.json`);
            if (fs.existsSync(NATIVE)) {
                const { routeLines } = await import('../../src/model/route_lines.mjs');
                const one = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots.one;
                const r = spawnSync(NATIVE, ['preview', '-'], { input: routeLines(f.route, one), encoding: 'utf8', maxBuffer: 256 << 20, timeout: 60000 });
                assert.ok(r.status === 0, `the native twin (${NATIVE}): rc ${r.status} ${r.stderr}`);
                want = JSON.parse(r.stdout);
            } else if (kept) {
                assert.ok(fs.existsSync(kept), `${kept} is missing: CI's Linux job writes it (record.mjs --out)`);
                want = JSON.parse(fs.readFileSync(kept, 'utf8')).result.one;
            } else {
                assert.ok(!MUST, 'in CI the native twin\'s record must be here: set ROUTE_MAKER_NATIVE_RECORDS (record.mjs --out)');
                want = f.result.one;
                skipKeys = [...skipKeys, 'api', 'owed', 'robotLine', 'ledgerFile', 'notices'];
            }
            for (const form of ['site', 'onefile']) {
                const p = await openPage({ form, engine: null });
                await openFixture(p.page, name);
                await preview(p.page);
                assert.equal((await pv(p.page)).phase, 'open', `${name}, ${form}: ${(await pv(p.page)).words}`);
                const got = await p.page.evaluate(() => window.routeBuilder.previewRecord());
                for (const k of Object.keys(want).filter((x) => !skipKeys.includes(x))) {
                    assert.deepEqual(got[k], want[k], `${name}, ${form}: ${k}`);
                }
                // poses: the module's maths library rounds in the last bit differently from the native one (A.2, WG2's bounds)
                assert.equal(got.samples.length, want.samples.length);
                got.samples.forEach((s, i) => {
                    assert.equal(s[0], want.samples[i][0]);
                    assert.ok(Math.abs(s[1] - want.samples[i][1]) <= 1e-5 && Math.abs(s[2] - want.samples[i][2]) <= 1e-5
                              && Math.abs(s[3] - want.samples[i][3]) <= 1e-6, `${name}, ${form}: pace ${i}`);
                });
                assert.deepEqual(got.steps.map(({ target, ...s }) => s), want.steps.map(({ target, ...s }) => s), `${name}, ${form}: the steps`);
                const notes = (r) => r.field.map(({ depth, ...x }) => x);
                assert.deepEqual(notes(got), notes(want), `${name}, ${form}: the field notes`);
                got.field.forEach((x, i) => assert.ok(Math.abs(x.depth - want.field[i].depth) <= 1e-5, `${name}, ${form}: note ${i}'s depth`));
                // RMP1-E: the mechanisms -- a lift's or an arm's position within the poses' bound, the rest exact
                assert.deepEqual(got.mechanisms.map(({ at, ...m }) => m), want.mechanisms.map(({ at, ...m }) => m), `${name}, ${form}: the mechanisms`);
                got.mechanisms.forEach((m, k) => {
                    assert.equal(m.at.length, want.mechanisms[k].at.length, `${name}, ${form}: ${m.name}'s states`);
                    m.at.forEach((v, i) => assert.ok(['lift', 'arm'].includes(m.kind)
                        ? Math.abs(v - want.mechanisms[k].at[i]) <= 1e-5 : v === want.mechanisms[k].at[i], `${name}, ${form}: ${m.name} at pace ${i}`));
                });
                assert.equal(await p.page.locator('.step.pv-ok').count(), f.route.steps.length);
                assert.deepEqual(await p.done(), []);
            }
        }
    });

    // ── §2.5.2: PREVIEW playing mid-route, a failed step, the mirrored pair with the band — light and dark, 1440 and 1024 ──
    for (const v of VARIANTS) {
        test(`§2.5.2 [${v.tag}] PREVIEW mid-route; a failed step; the two robots and the band`, async () => {
            const p = await openPage(v);
            const { page } = p;
            await openFixture(page, 'ready');
            await preview(page);
            await stop(page);
            await page.keyboard.press('Home');
            for (let k = 0; k < 2; ++k) {
                await page.keyboard.press('ArrowRight');
            }
            for (let k = 0; k < 12; ++k) {
                await page.keyboard.press('Shift+ArrowRight');
            }
            await shot(page, `preview-playing-${v.tag}`);
            await page.getByRole('button', { name: /About this run/ }).click();
            await shot(page, `preview-about-${v.tag}`);
            await openFixture(page, 'failed');
            await preview(page);
            await stop(page);
            await page.keyboard.press('Home');
            await page.keyboard.press('ArrowRight');
            for (let k = 0; k < 3; ++k) {
                await page.keyboard.press('Shift+ArrowRight');
            }
            await shot(page, `preview-failed-${v.tag}`);
            assert.deepEqual(await p.done(), []);
            const two = await openPage({ ...v, shared: true });  // robot two's update built in: both robots play
            await openFixture(two.page, 'pair_fail');
            await preview(two.page);
            await stop(two.page);
            await two.page.locator('.pair-mark').click();
            await shot(two.page, `preview-pair-${v.tag}`);
            assert.deepEqual(await two.done(), []);
        });

        test(`§2.5.2 [${v.tag}] PREVIEW's other states: running, stopped at 20 s, no engine, refused, put away, one robot, a THROUGH, too old`, async () => {
            const slow = await openPage({ ...v, engine: { delayMs: 5000 } });
            await openFixture(slow.page, 'ready');
            await slow.page.locator('#preview').click();
            await slow.page.locator('.dock .spinner').waitFor();
            await slow.page.waitForTimeout(400);
            if (SHOTS) {  // at rest is impossible while it runs: the toast of the open is let go first
                await slow.page.screenshot({ path: path.join(SHOTS, `${BROWSER}-preview-running-${v.tag}.png`) });
            }
            await slow.page.getByRole('button', { name: 'Cancel' }).click();
            assert.deepEqual(await slow.done(), []);
            const hung = await openPage({ ...v, engine: { hang: 'ready', timeoutMs: 1000 } });
            await openFixture(hung.page, 'ready');
            await preview(hung.page);
            await shot(hung.page, `preview-stopped-${v.tag}`);
            assert.deepEqual(await hung.done(), []);
            const none = await openPage({ ...v, engine: { failLoad: 'the engine file is damaged' } });
            await openFixture(none.page, 'ready');
            await preview(none.page);
            await shot(none.page, `preview-no-engine-${v.tag}`);
            assert.deepEqual(await none.done(), []);
            const p = await openPage({ ...v, engine: { refuse: { failed: 'the library\'s own refusal' } } });
            const { page } = p;
            await openFixture(page, 'failed');
            await preview(page);
            await shot(page, `preview-refused-${v.tag}`);
            await openFixture(page, 'through');
            await preview(page);
            await stop(page);
            await openSteps(page);
            await page.locator('.step[data-step="0"]').click();
            await shot(page, `preview-through-${v.tag}`);
            await closePanes(page);
            await page.keyboard.press('End');
            await shot(page, `preview-through-after-${v.tag}`);
            await withoutTwo(page);  // a build without robot two's update
            await openFixture(page, 'pair_ok');
            await preview(page);
            await stop(page);
            await page.getByRole('button', { name: /About this run/ }).click();
            assert.ok(await aboutLineInView(page, NO_TWO), 'the one-robot line in the picture');
            await shot(page, `preview-one-robot-${v.tag}`);
            await openSteps(page);
            await page.locator('.step[data-step="2"]').click();
            await page.keyboard.press('Delete');
            await closePanes(page);
            await shot(page, `preview-stale-${v.tag}`);
            assert.deepEqual(await p.done(), []);
            const ctx = await browser.newContext({ viewport: v.viewport, colorScheme: v.scheme });
            await ctx.addInitScript(() => {
                window.Worker = class { constructor() { throw new Error('no Workers here'); } };
            });
            const old = await ctx.newPage();
            await old.goto(`${base}/app.html`);
            await old.waitForFunction(() => document.documentElement.dataset.ready === '1');
            await shot(old, `preview-too-old-${v.tag}`);
            await ctx.close();
        });
    }
});
