// camera_view.page.test.mjs — RMP1-N in a browser, on the page tools/route_builder.py serves (a stand-in checkout, the
// fake `pros terminal` replaying shulib Drive's lines, one each 0.05 s, blank lines as the pauses): the Camera tab waits
// for the first picture, says "Sees nothing" through a quiet second without greying, draws a tag, a Pin and a colour
// where the camera saw them (positions scaled from 320 x 240), greys a picture 0.35 s after its last line ("Sees nothing"
// after 1.5 s); labels stay in the picture; lines that come while no frame is painted (a burst, a hidden page) are all
// read, and a hidden spell's lines show as old; the tabs follow the runs; no [CAM] line, no tab; no sensor, one sentence.
// ROUTE_MAKER_SHOTS: light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawn, execFileSync } from 'node:child_process';
import { SITE, SHOTS, REPO, skip, launch, shot } from './harness.mjs';
import { saveProject, newProject } from '../../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../../src/model/route.mjs';

const at = (r) => setStart({ ...r, together: false }, { x: -36, y: -60, headingDeg: 90 });
const PROJECT = saveProject({ ...newProject('the test'), routes: [addStep(at(newRoute()), { do: 'wait', seconds: 1 }),
                                                                  addStep(at(newRoute('left 1')), { do: 'wait', seconds: 2 })] });
const pause = (s) => Array(Math.round(s / 0.05)).fill('');
const every = (n, line) => Array(n).fill([line, '']).flat();  // a line every 0.1 s, for n lines
// shulib Drive's lines (drive_program.cpp's CameraTrace through TermSink: camera_log.test.mjs pins both)
const BOOT = ['', '[DRV] booted: shulib DRIVE build [shulib-robot-variant=one-drive] -- robot one\'s DRIVER FILE drives it',
    '[t=   0.30] [DRV] shulib-robot-variant=one-drive',
    '[t=   0.40] [CAM] AI Vision on port 7: tags, objects and colours on (1); a line every 100 ms: tag id c(x,y) WxH | obj class (x,y) WxH score% | col id (x,y) WxH',
    "[t=   0.40] [CAM] class 0: 'redBluePin'", "[t=   0.40] [CAM] class 4: 'cup'"];
const SEEN = '[t=   3.00] [CAM] n 3 | tag 3 c(200,90) 48x44 | obj 0 (40,120) 50x100 87% | col 2 (250,170) 40x30';
const CAMERA = [...BOOT, ...pause(0.6), '[t=   1.00] [CAM] n 0', '[t=   1.05] [CAS] R1 0 R2 0', ...pause(1), '[t=   2.00] [CAM] n 0', ...pause(1),
    ...every(19, SEEN), SEEN.replace('87%', '88%')];  // 2 s of the same picture, its last line told apart, then nothing
const EDGES = [...BOOT.slice(0, 5), ...pause(0.3), ...every(10, '[t=   1.00] [CAM] n 2 | obj 0 (40,0) 200x240 91% | tag 28 c(13,4) 10x6'),  // a Pin up close; a tag
    ...every(10, '[t=   2.00] [CAM] n 2 | obj 0 (400,100) 50x50 80% | tag 3 c(297,202) 124x21'),
    ...every(5, '[t=  79.90] [CAM] n 2 | tag 2 c(175,142) 14x13 | tag 15 c(213,237) 36x13'), '[t=   3.00] [CAM] n 0'];  // the 11-20 log :143204
const BURST = [...pause(1.5), ...BOOT, SEEN, '[t=   3.00] [CAS] R1 0 R2 0', ...pause(1.5), ...every(10, '[t=   5.00] [CAM] n 1 | tag 5 c(100,100) 30x30')];
const RUN = (n, rest) => `[t=   ${n}.00] [ATS] run ${n}: 'elsewhere' ${rest}`;
const RUNS = [...BOOT.slice(0, 3), RUN(1, '(1 step) -- RUN touched'), ...BOOT.slice(3), ...every(5, SEEN), ...pause(1.5),
    RUN(2, '(1 step) -- RUN touched'), ...every(5, SEEN), ...pause(1.5), RUN(2, 'ABORTED -- the test'), ...every(30, SEEN)];
const NO_CAM = [...BOOT.slice(0, 3), '[t=   0.50] [CAS] R1 0 R2 0', '[t=   0.55] [DRV] the last line'];
const NO_SENSOR = [...BOOT.slice(0, 3), '[t=   0.40] [CAM] no AI Vision sensor on any port: nothing to trace', '[t=   0.55] [DRV] the last line'];
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
const WORK = path.join(REPO, 'build', 'route_builder_camera_pagetest');
const TOOL = path.join(REPO, 'tools', 'route_builder.py');
const UNPLUG = path.join(WORK, 'unplug');  // the fake terminal ends as an unplugged brain's does when this file exists

let browser;
let helper;
let url;
let made;
const open = new Set();
const terminal = () => path.join(WORK, 'terminal.log');

function replay(lines) {
    fs.rmSync(UNPLUG, { force: true });
    fs.writeFileSync(path.join(made.root, 'src', 'team', 'route_maker.json'), PROJECT);
    fs.writeFileSync(terminal(), lines.join('\n'));
}

async function opened(v) {
    const ctx = await browser.newContext({ viewport: v.viewport, colorScheme: v.scheme });
    open.add(ctx);
    const page = await ctx.newPage();
    const problems = [];
    const base = new URL(url).origin;
    page.on('console', (m) => ['error', 'warning'].includes(m.type()) && problems.push(`console ${m.type()}: ${m.text()}`));
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('request', (r) => !r.url().startsWith(base) && !/^(data|blob):/.test(r.url()) && problems.push(`a request outside the page: ${r.url()}`));
    await page.goto(url);
    await page.waitForFunction(() => document.documentElement.dataset.ready, null, { timeout: 30000 });
    await page.waitForFunction(() => window.routeBuilder.routes().length === 2, null, { timeout: 10000 });
    const got = page.getByRole('button', { name: 'Got it' });
    if (await got.count()) {
        await got.click();
    }
    if (SHOTS) {
        await page.locator('.toast').waitFor({ state: 'detached', timeout: 10000 });
    }
    await page.evaluate(() => {  // every [sentence, grey, age words, ms] the card shows, as it changes
        window.__cam = [];
        setInterval(() => {
            const p = document.querySelector('#cam-sees');
            const now = p && [p.firstChild ? p.firstChild.textContent : '', !!document.querySelector('.cam-frame.stale'), p.querySelector('.cam-age')?.textContent || ''];
            const last = window.__cam[window.__cam.length - 1];
            if (now && (!last || now.some((x, k) => x !== last[k]))) {
                window.__cam.push([...now, performance.now()]);
            }
        }, 20);
    });
    return { page, done: async () => { open.delete(ctx); await ctx.close(); return problems; } };
}
/** Watch the robot from the route menu (a reader the last page left open frees itself within a ping: try again); returns
 *  a filter for Chromium's console line about each refused try's 409. */
async function watch(page) {
    let refused = 0;
    for (let k = 0; k < 8; ++k) {
        await page.locator('#route-switch').click();
        await page.locator('#watch-robot').click();
        await page.waitForFunction(() => /^(?!Opening)/.test(document.querySelector('#rk-summary').textContent), null, { timeout: 30000 });
        if (!/already open/.test(await page.locator('#rk-summary').textContent())) {
            break;
        }
        refused += 1;
        await page.waitForTimeout(1000);
    }
    return (problems) => problems.filter((x) => !(refused && /status of 409 \(Conflict\)/.test(x)));
}
const lastLine = (page) => page.locator('.rk-log pre', { hasText: 'the last line' }).waitFor({ state: 'attached', timeout: 30000 });
const sees = (page, words, timeout = 30000) => page.locator('#cam-sees', { hasText: words }).waitFor({ timeout });
const record = (page) => page.evaluate(() => window.__cam);
/** Seconds from the first record saying `words` to the first grey one after it. */
const greyAfter = (seq, words) => {
    const k = seq.findIndex((x) => x[0] === words);
    const g = seq.findIndex((x, j) => j > k && x[1]);
    return k < 0 || g < 0 ? NaN : (seq[g][3] - seq[k][3]) / 1000;
};
/** Paint nothing until `__release()`: the page's frames held, as a busy or (`hidden`) a hidden page's are. */
const hold = (page, hidden) => page.evaluate((hide) => {
    const raf = window.requestAnimationFrame.bind(window);
    const held = [];
    window.requestAnimationFrame = (f) => held.push(f);
    const seen = (v) => ['hidden', 'visibilityState'].forEach((k, i) => Object.defineProperty(document, k, { configurable: true, get: () => [v, v ? 'hidden' : 'visible'][i] }));
    if (hide) {
        seen(true);
        document.dispatchEvent(new Event('visibilitychange'));
    }
    window.__release = () => {
        window.requestAnimationFrame = raf;
        if (hide) {
            seen(false);
            document.dispatchEvent(new Event('visibilitychange'));
        }
        held.splice(0).forEach((f) => raf(f));
    };
}, hidden);

test.describe('RMP1-N: what the camera sees, in Watch the robot', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'robots.json')), 'build the site first (tools/route_maker/build.py)');
        const py = process.platform === 'win32' ? 'python' : 'python3';
        made = JSON.parse(execFileSync(py, [TOOL, 'stand-in', WORK, path.join(SITE, 'robots.json')], { encoding: 'utf8' }));
        replay(NO_CAM);
        const env = { ...process.env, ...made.env, PYTHONIOENCODING: 'utf-8', FAKE_TERMINAL: terminal(), FAKE_TERMINAL_EVERY: '0.05', FAKE_UNPLUG: UNPLUG };
        helper = spawn(made.python, [TOOL, '--root', made.root, '--site', 'http://127.0.0.1:9/', '--no-browser'], { env, stdio: ['ignore', 'pipe', 'pipe'] });
        url = await new Promise((ok, bad) => {
            let out = '';
            helper.stdout.on('data', (d) => {
                out += d;
                const m = /Route builder: (\S+)/.exec(out);
                if (m) {
                    ok(m[1]);
                }
            });
            helper.stderr.on('data', (d) => { out += d; });
            helper.on('exit', (c) => bad(new Error(`the helper ended (${c}): ${out}`)));
            setTimeout(() => bad(new Error(`no link from the helper in 30 s: ${out}`)), 30000);
        });
        browser = await launch();
    });
    test.afterEach(async () => {
        for (const ctx of open) {
            await ctx.close();
        }
        open.clear();
    });
    test.after(async () => {
        await browser?.close();
        if (helper && helper.exitCode === null) {
            const ended = new Promise((ok) => helper.on('exit', ok));
            helper.kill(process.platform === 'win32' ? 'SIGTERM' : 'SIGINT');
            await ended;
        }
        fs.rmSync(WORK, { recursive: true, force: true });
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] the Camera tab: waiting, "Sees nothing" through a quiet second, a tag, a Pin and a colour where they were, then grey`, async () => {
            replay(CAMERA);
            const p = await opened(v);
            const { page } = p;
            const mine = await watch(page);
            const FIRST = 'Sees: AprilTag 3, redBluePin (87 %), colour 2';
            const LAST = 'Sees: AprilTag 3, redBluePin (88 %), colour 2';
            await sees(page, FIRST);
            assert.equal(await page.locator('.cam-frame.stale').count(), 0, 'just seen: not grey');
            await shot(page, `camera-${v.tag}`);
            await sees(page, LAST, 5000);
            await page.locator('.cam-frame.stale').waitFor({ timeout: 3000 });
            // the boxes, in the picture's pixels and on the screen (scaled from 320 x 240)
            const boxes = await page.$eval('.cam-frame', (svg) => {
                const f = svg.getBoundingClientRect();
                return { aspect: f.width / f.height, items: [...svg.querySelectorAll('.cam-box')].map((g) => {
                    const r = g.querySelector('rect');
                    const b = r.getBoundingClientRect();
                    return { kind: g.dataset.kind, label: g.querySelector('text').textContent, attr: ['x', 'y', 'width', 'height'].map((a) => +r.getAttribute(a)),
                             screen: [(b.left - f.left) * 320 / f.width, (b.top - f.top) * 240 / f.height, b.width * 320 / f.width, b.height * 240 / f.height] };
                }) };
            });
            assert.ok(Math.abs(boxes.aspect - 4 / 3) < 0.01, `the picture is 4:3: ${boxes.aspect}`);
            assert.deepEqual(boxes.items.map(({ kind, label, attr }) => ({ kind, label, attr })), [
                { kind: 'tag', label: '3', attr: [176, 68, 48, 44] },
                { kind: 'obj', label: 'redBluePin (88 %)', attr: [40, 120, 50, 100] },
                { kind: 'col', label: 'colour 2', attr: [250, 170, 40, 30] }]);
            boxes.items.forEach((b) => b.screen.forEach((q, k) => assert.ok(Math.abs(q - b.attr[k]) < 2.5, `${b.kind} on screen ${b.screen} vs ${b.attr}`)));
            const stroke = await page.$eval('.cam-box[data-kind="col"] rect', (r) => getComputedStyle(r).getPropertyValue('stroke'));
            assert.equal(stroke, 'rgb(119, 115, 127)', 'the boxes greyed');
            // the age counts up while the robot stays quiet
            await page.locator('#cam-sees .cam-age', { hasText: 'last seen 2 s ago' }).waitFor({ timeout: 5000 });
            const seq = await record(page);
            assert.deepEqual(seq.map((x) => x[0]).filter((x, k, a) => x !== a[k - 1]),
                ['Camera on port 7: waiting for its first picture.', 'Sees nothing', FIRST, LAST], JSON.stringify(seq));
            assert.ok(seq.every(([w, grey]) => !grey || w === LAST), `a line every second (nothing seen) or 0.1 s (something seen) is never grey: ${JSON.stringify(seq)}`);
            const gap = greyAfter(seq, LAST);
            assert.ok(gap >= 0.3 && gap <= 0.75, `grey 0.35 s after the last picture's line, not ${gap} s`);
            assert.deepEqual(seq.map((x) => x[2]).filter((x, k, a) => x && x !== a[k - 1]), ['last seen under a second ago', 'last seen 1 s ago', 'last seen 2 s ago']);
            // the tabs: Camera picked (no run yet), the run's lines out of the way; Run brings them back
            assert.equal(await page.locator('.cam-tabs [data-tab="camera"]').getAttribute('aria-selected'), 'true');
            assert.equal(await page.locator('.rk-log').isVisible(), false, 'one idea: the log steps aside');
            assert.equal(await page.locator('#rk-summary').isVisible(), false, 'watching: the status steps aside too');
            await page.locator('.cam-tabs [data-tab="run"]').click();
            assert.equal(await page.locator('.cam-body').isVisible(), false);
            assert.equal(await page.locator('.rk-log').isVisible(), true);
            await page.locator('.cam-tabs [data-tab="camera"]').click();
            await shot(page, `camera-stale-${v.tag}`);
            await page.getByRole('button', { name: 'Stop watching' }).click();
            assert.equal(await page.locator('#rk-summary').textContent(), 'Stopped watching.');
            assert.equal(await page.locator('#rk-summary').isVisible(), true, 'on Camera, the helper\'s last words are kept');
            assert.deepEqual(mine(await p.done()), []);
        });
    }

    test('a Pin up close keeps its label in the picture; a box past the picture: no stray label, one line; "Sees nothing" greys after 1.5 s', async () => {
        replay(EDGES);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        const mine = await watch(page);
        await sees(page, 'Sees: redBluePin (91 %), AprilTag 28');  // the tag as robot one's camera saw it (11-20 log :109002)
        const inside = await page.$eval('.cam-frame', (svg) => {
            const f = svg.getBoundingClientRect();
            return [...svg.querySelectorAll('.cam-box text')].map((t) => {
                const b = t.getBoundingClientRect();
                return [b.left - f.left, b.top - f.top, f.right - b.right, f.bottom - b.bottom];
            });
        });
        assert.equal(inside.length, 2);
        assert.ok(inside.flat().every((d) => d >= -0.5), `each label inside the picture: ${JSON.stringify(inside)}`);
        assert.equal(await page.locator('.cam-edge').isVisible(), false);
        await sees(page, 'Sees: redBluePin (80 %), AprilTag 3', 5000);
        assert.equal(await page.locator('.cam-edge').textContent(), 'A box runs past the 320 x 240 picture this view assumes.');
        assert.equal(await page.locator('.cam-edge').isVisible(), true);
        assert.deepEqual(await page.$$eval('.cam-box', (g) => g.map((x) => x.dataset.kind)), ['tag'], 'the Pin at x 400 is not drawn (its label never pinned to the edge)');
        await sees(page, 'Sees: AprilTag 2, AprilTag 15', 5000);  // only part of a box past the picture: drawn, and said
        assert.deepEqual(await page.$$eval('.cam-box text', (t) => t.map((x) => x.textContent)), ['2', '15']);
        assert.equal(await page.locator('.cam-edge').isVisible(), true);
        await sees(page, 'Sees nothing', 5000);
        await page.locator('.cam-frame.stale').waitFor({ timeout: 4000 });
        const gap = greyAfter(await record(page), 'Sees nothing');
        assert.ok(gap >= 1.45 && gap <= 1.95, `"Sees nothing" greys 1.5 s after its line, not ${gap} s`);
        assert.equal(await page.locator('.cam-edge').isVisible(), false);
        await page.getByRole('button', { name: 'Stop watching' }).click();
        assert.deepEqual(mine(await p.done()), []);
    });

    test('lines that come while no frame is painted are all read: a burst; a hidden page\'s, shown as old; a dropped boot forgotten', async () => {
        replay(BURST);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        const mine = await watch(page);
        await hold(page, false);  // the burst (boot, class names, a picture, a [CAS] line) comes while one paint waits
        await page.waitForTimeout(2600);
        assert.equal(await page.locator('.cam').count(), 0, 'nothing painted since the hold');
        await page.evaluate(() => window.__release());
        await sees(page, 'Sees: AprilTag 3, redBluePin (87 %), colour 2', 2000);
        assert.equal(await page.locator('.cam-frame.stale').count(), 0, 'just read: not grey');
        await hold(page, true);  // hidden while the robot sees tag 5 for 1 s, then goes quiet
        await page.waitForTimeout(3000);
        const shown = Date.now();
        await page.evaluate(() => window.__release());
        await sees(page, 'Sees: AprilTag 5seen while this page was hidden', 2000);
        assert.ok(Date.now() - shown < 1500);
        const seq = await record(page);
        assert.ok(seq.filter(([w]) => w === 'Sees: AprilTag 5').every(([, grey]) => grey), `never shown as just seen: ${JSON.stringify(seq)}`);
        const dropped = await page.evaluate(async () => {  // more lines than the page keeps (2000) came unpainted: a boot among them
            const { cameraCard } = await import(new URL('src/ui/camera_view.mjs', location.href).href);  // the page's own module
            const w = { lines: 0, runs: [], robot: 'one' };
            let raw = [];
            const add = (...lines) => { raw = [...raw, ...lines].slice(-2000); w.lines += lines.length; };
            const said = () => cameraCard(w, raw, 'live').querySelector('#cam-sees').firstChild.textContent;
            add('[t=   0.40] [CAM] AI Vision on port 7: on (1)', "[t=   0.40] [CAM] class 0: 'redBluePin'", '[t=   1.00] [CAM] n 1 | obj 0 (40,120) 50x100 87%');
            const before = said();
            add('[t=   0.40] [CAM] AI Vision on port 3: on (1)', ...Array(2000).fill('[t=   1.00] [CAS] R1 0 R2 0'), '[t=  30.00] [CAM] n 1 | obj 0 (40,120) 50x100 60%');
            return [before, said()];
        });
        assert.deepEqual(dropped, ['Sees: redBluePin (87 %)', 'Sees: class 0 (60 %)'], 'an old boot\'s names never name a new boot\'s objects');
        await page.getByRole('button', { name: 'Stop watching' }).click();
        assert.deepEqual(mine(await p.done()), []);
    });

    test('the tabs follow the runs (Run when one starts); a red status, and the helper\'s error, stay on the Camera tab', async () => {
        replay(RUNS);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        const mine = await watch(page);
        await page.locator('#cam-sees').waitFor({ state: 'attached', timeout: 30000 });
        assert.equal(await page.locator('.cam-tabs [data-tab="run"]').getAttribute('aria-selected'), 'true', 'a run already on: its tab first');
        await page.locator('.cam-tabs [data-tab="camera"]').click();
        await page.locator('.cam-tabs [data-tab="run"][aria-selected="true"]').waitFor({ timeout: 5000 });  // run 2 starts
        await page.locator('.cam-tabs [data-tab="camera"]').click();
        await page.locator('.rk-status.bad').waitFor({ timeout: 5000 });  // run 2: ABORTED, the watch still live
        assert.equal(await page.locator('.rk-status.bad').isVisible(), true, 'a red status is never hidden by the Camera tab');
        assert.equal(await page.locator('.cam-body').isVisible(), true);
        fs.writeFileSync(UNPLUG, '');
        await page.locator('#rk-summary', { hasText: 'unplugged' }).waitFor({ timeout: 10000 });
        assert.equal(await page.locator('#rk-summary').isVisible(), true);
        assert.equal(await page.locator('.cam-body').isVisible(), true);
        assert.deepEqual(mine(await p.done()), []);
    });

    test('no [CAM] line: no Camera tab; no sensor: the card with one sentence and no picture', async () => {
        for (const [lines, want] of [[NO_CAM, null], [NO_SENSOR, 'No camera on White Bot: no AI Vision sensor on any port.']]) {
            replay(lines);
            const p = await opened(VARIANTS[0]);
            const { page } = p;
            const mine = await watch(page);
            await lastLine(page);
            await page.waitForTimeout(300);
            if (want) {
                assert.equal(await page.locator('#cam-sees').textContent(), want);
                assert.equal(await page.locator('.cam-frame').count(), 0);
            } else {
                assert.equal(await page.locator('.cam').count(), 0, 'no camera lines: nothing new on the screen');
            }
            await page.getByRole('button', { name: 'Stop watching' }).click();
            assert.deepEqual(mine(await p.done()), []);
        }
    });
});
