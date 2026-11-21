// page.test.mjs — THE ROUTE BUILDER IN A REAL BROWSER (RMP1 brief A.4; WG5, WG6, the old-browser refusal; §2.5).
//
// Runs only when ROUTE_MAKER_BROWSER is set (firefox | chromium | msedge | chrome), against the built folder
// ROUTE_MAKER_SITE (default build/route_maker_site, made by tools/route_maker/build.py): the SITE form from a tiny
// static server, the ONE FILE over file:// with every request refused. Every page FAILS on a Content-Security-
// Policy violation, a page error or a console error; console warnings are allowed only from an exact list
// (empty in RMP1-A1: there is no WebAssembly yet — A2e adds Firefox's one, W6). With ROUTE_MAKER_SHOTS set, the
// UX acceptance screenshots (§2.5.2) are written there — EVERY A1 screen in light and dark, at 1440 x 900 and
// 1024 x 768 — and every behaviour asserted on the way is asserted in all four; without it, in one (light, 1440).
//
//   ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/*.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { fileURLToPath, pathToFileURL } from 'node:url';
import { ledgerRobot } from '../helpers.mjs';
// Firefox's bounce-tracking protection warns about the tests' own 127.0.0.1 server on Windows ("classified as a
// bounce tracker"): a warning from the browser's privacy feature, not the page's. It is turned off in the test browser.
const FIREFOX_PREFS = { 'privacy.bounceTrackingProtection.mode': 0 };

const BROWSER = process.env.ROUTE_MAKER_BROWSER || '';
const HERE = path.dirname(fileURLToPath(import.meta.url));
const REPO = path.resolve(HERE, '..', '..', '..', '..');
const SITE = path.resolve(process.env.ROUTE_MAKER_SITE || path.join(REPO, 'build', 'route_maker_site'));
const SHOTS = process.env.ROUTE_MAKER_SHOTS ? path.resolve(process.env.ROUTE_MAKER_SHOTS) : '';
const skip = !BROWSER ? 'set ROUTE_MAKER_BROWSER (firefox, chromium, msedge or chrome) to run the page tests' : false;

const CSP_ONE_FILE = "default-src 'none'; script-src 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline'; img-src data: blob:; worker-src blob:; connect-src 'self'";
const CSP_SITE = "default-src 'none'; script-src 'self' 'unsafe-inline' 'wasm-unsafe-eval'; style-src 'unsafe-inline'; img-src 'self' data: blob:; worker-src 'self' blob:; connect-src 'self'";
const ALLOWED_WARNINGS = [];  // A2e: Firefox's "The WebAssembly exception handling 'try' instruction is deprecated..." (W6)

/** §2.5.2's four: light and dark, 1440 x 900 and 1024 x 768 (all four when the screenshots are taken). */
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1024, 768], ['dark', 1024, 768]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));

const SKILLS_HEAD = '#include "team/skills_route.hpp"\r\n\r\nnamespace shulib::bench {\r\n\r\n';
const BEGIN = '// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════';
const END = '// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════';
const MIRRORED = 'addTeamRoutes(registry, TeamRobot::Two);';  // robot two's line that registers the team's routes (RMP1-C D2)
// The level tests' robot: robot x (the suite's description) with a cascade whose levels are facts-ledger levels, still
// owed -- robot one's cascade as it was before ROB1-C2. Robot one's own cascade has NO levels now (its own test below).
const LEVELS = ledgerRobot();
LEVELS.name = 'robot x (test)';
LEVELS.mechanisms[0].presets.forEach((q) => { q.owed = true; });

let pw;
let browser;
let server;
let base;

async function launch(extra = {}) {
    pw = pw || (await import('playwright'));
    if (BROWSER === 'firefox') {
        return pw.firefox.launch({ firefoxUserPrefs: FIREFOX_PREFS, ...extra });
    }
    if (BROWSER === 'msedge' || BROWSER === 'chrome') {
        return pw.chromium.launch({ channel: BROWSER, ...extra });
    }
    return pw.chromium.launch(extra);
}

function serve(root) {
    const mime = { '.html': 'text/html; charset=utf-8', '.mjs': 'text/javascript', '.js': 'text/javascript', '.json': 'application/json',
                   '.wasm': 'application/wasm', '.png': 'image/png' };
    return http.createServer((req, res) => {
        const f = path.join(root, decodeURIComponent(new URL(req.url, 'http://x').pathname));
        if (!f.startsWith(root) || !fs.existsSync(f) || fs.statSync(f).isDirectory()) {
            res.writeHead(404);
            res.end();
            return;
        }
        res.writeHead(200, { 'content-type': mime[path.extname(f)] || 'application/octet-stream' });
        res.end(fs.readFileSync(f));
    });
}

/** A page whose every console error, page error, CSP report and outside request is recorded. */
async function openPage(form, { viewport = { width: 1440, height: 900 }, scheme = 'light', b = browser } = {}) {
    const ctx = await b.newContext({ viewport, colorScheme: scheme, acceptDownloads: true });
    const problems = [];
    const page = await ctx.newPage();
    page.on('console', (m) => {
        if (m.type() === 'error' || (m.type() === 'warning' && !ALLOWED_WARNINGS.some((w) => m.text().includes(w)))) {
            problems.push(`console ${m.type()}: ${m.text()}`);
        }
    });
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('request', (r) => {
        const u = r.url();
        const local = form === 'site' ? u.startsWith(base) : u.startsWith('file:');
        if (!local && !u.startsWith('data:') && !u.startsWith('blob:')) {
            problems.push(`a request outside the page: ${u}`);
        }
    });
    const url = form === 'site' ? `${base}/app.html` : pathToFileURL(path.join(SITE, 'shulib-route-builder.html')).href;
    await page.goto(url);
    await ready(page);
    return { page, ctx, problems, done: async () => {
        const csp = await page.evaluate(() => window.__csp);
        await ctx.close();
        return [...problems, ...csp.map((c) => `CSP violation: ${c}`)];
    } };
}

/** The app is up (after a load or a reload), and CSP violations are recorded (they reach the page in every engine). */
async function ready(page) {
    await page.waitForFunction(() => document.documentElement.dataset.ready, null, { timeout: 30000 });
    await page.evaluate(() => {
        window.__csp = window.__csp || [];
        document.addEventListener('securitypolicyviolation', (e) => window.__csp.push(`${e.violatedDirective} ${e.blockedURI}`));
    });
}

async function shot(page, name) {
    if (SHOTS) {
        fs.mkdirSync(SHOTS, { recursive: true });
        await page.waitForTimeout(400);  // the drawers' and toasts' 200 ms animations finish first
        await page.screenshot({ path: path.join(SHOTS, `${BROWSER}-${name}.png`) });
    }
}

/** Before a screenshot only: the last toast fades, so the screen is as a student sees it at rest. */
async function settle(page) {
    if (SHOTS) {
        await page.waitForTimeout(3500);
    }
}

/** The page (screen) point of field inches (x, y). */
function fieldPoint(page, x, y) {
    return page.evaluate(([fx, fy]) => {
        const pic = JSON.parse(document.getElementById('shulib-field').textContent);
        const svg = document.querySelector('.field-svg');
        const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(svg.getScreenCTM());
        return { x: pt.x, y: pt.y };
    }, [x, y]);
}

/** Click the field at field inches (x, y). */
async function clickField(page, x, y, opts = {}) {
    const at = await fieldPoint(page, x, y);
    const mods = opts.modifiers || [];
    for (const m of mods) {
        await page.keyboard.down(m);
    }
    await page.mouse.click(at.x, at.y);
    for (const m of mods) {
        await page.keyboard.up(m);
    }
}

const route = (page) => page.evaluate(() => window.routeBuilder.route());
const issues = (page) => page.evaluate(() => window.routeBuilder.issues());
const errorsOf = async (page) => (await issues(page)).filter((x) => x.level === 'error');

/** Type `value` into the inspector's field `name` and press Enter (the student's number, never the page's). */
async function typeIn(page, name, value, scope = '.pane.right') {
    await page.locator(`${scope} input[name="${name}"]`).fill(String(value));
    await page.locator(`${scope} input[name="${name}"]`).press('Enter');
}

/** Open the export drawer by the keyboard, from the field. */
async function openExport(page) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('Control+e');
    await page.locator('.drawer').waitFor();
}
async function closeDrawer(page) {
    await page.keyboard.press('Escape');
    await page.locator('.drawer').waitFor({ state: 'detached' });
}

/** Pick a file in the page's own file chooser (Open, or the download way's "pick skills_route.cpp"). */
async function pickFile(page, click, name, text) {
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), click()]);
    await fc.setFiles({ name, mimeType: name.endsWith('.json') ? 'application/json' : 'text/plain', buffer: Buffer.from(text, 'utf8') });
}

async function selectStep(page, viewport, i) {
    if (viewport.width <= 1100) {
        await page.getByRole('button', { name: 'Steps', exact: true }).click();  // the list is a drawer here
        await page.locator(`.step[data-step="${i}"]`).click();
        await page.getByRole('button', { name: 'Close the steps' }).click();
    } else {
        await page.locator(`.step[data-step="${i}"]`).click();
    }
    await page.waitForTimeout(250);  // the drawers' 200 ms slide
}

/** Robot one and its twin robot two (the mirrored route is checked on both) are LEVELS on this page from now on. */
const useLevels = (page) => page.evaluate((r) => window.routeBuilder.useRobots(r), { one: LEVELS, two: { ...LEVELS, id: 'two', name: 'robot two (test)' } });

/** M, then the mechanism, then what it does (RMP1-J D1's two steps). */
async function mech(page, ident, words) {
    await page.keyboard.press('m');
    await page.locator(`.mech-menu [data-mech="${ident}"]`).click();
    await page.locator(`.mech-menu [data-act="${words}"]`).click();
}
/** A step setting robot one's claw to `position`: its own button in the menu (RMP1-C D8: "Open claw"). */
async function setClaw(page, position) {
    await mech(page, 'claw', `${position === 'open' ? 'Open' : 'Close'} claw`);
}

/** RMP1-C D3: answer "do both robots run it at the same time?" on the route (the start's inspector). */
async function answerTogether(page, yes = false) {
    await page.locator(`.pane.right [data-together="${yes}"]`).click();
}
const routeName = (page) => page.locator('#route-switch strong').textContent();

// A READY route on robot one, built the way a student builds it (every number typed is the test's, never the page's):
// the start on the red tiles facing blue (NOT flush against the wall: turning there would put a corner 2.4 in into
// it -- the page says so, RC1), a home with its time limit, a drive (turned first), a curve, the claw opened, a wait.
// The screens of §2.5.2 and the docs' pictures are taken of it. RMP1-H: 24 in right of where it was, so it clears
// every structure (from (-36, -60) to (-24, -36) the square went through the black goal by the red wall).
async function readyRoute(page) {
    await page.getByRole('button', { name: 'Got it' }).click();
    await clickField(page, -12, -60.3);  // snaps to the half-tile grid
    await typeIn(page, 'startH', 90);
    await answerTogether(page, false);
    await mech(page, 'cascade', 'Find the bottom');
    await typeIn(page, 'boundS', 2);
    await page.keyboard.press('d');
    await clickField(page, 0, -36);
    await page.keyboard.press('c');
    await clickField(page, 12, -12);
    await clickField(page, 36, -12);
    await page.keyboard.press('Enter');
    await setClaw(page, 'open');
    await page.keyboard.press('p');
    await typeIn(page, 'seconds', 0.5);
    const r = await route(page);
    assert.deepEqual(r.steps.map((x) => x.do), ['home', 'driveTo', 'curveTo', 'set', 'wait']);
    assert.deepEqual([r.start.x, r.start.y], [-12, -60], 'snapped to the 12 in grid');
    assert.deepEqual(await errorsOf(page), [], 'a ready route');
}

test.describe('the route builder in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        server = serve(SITE);
        await new Promise((ok) => server.listen(0, '127.0.0.1', ok));
        base = `http://127.0.0.1:${server.address().port}`;
    });
    test.after(async () => {
        if (browser) {
            await browser.close();
        }
        if (server) {
            server.close();
        }
    });

    test('WG5: each form carries its OWN exact policy, the data: favicon and the app\'s marker; WG6: no request leaves the page; the REAL field picture is drawn', async () => {
        for (const [form, csp] of [['site', CSP_SITE], ['onefile', CSP_ONE_FILE]]) {
            const p = await openPage(form);
            const meta = await p.page.evaluate(() => ({
                csp: document.querySelector('meta[http-equiv="Content-Security-Policy"]').content,
                icon: document.querySelector('link[rel="icon"]').getAttribute('href'),
                app: document.querySelector('meta[name="shulib-app"]').content,
                wasmAllowed: document.querySelector('meta[http-equiv="Content-Security-Policy"]').content.includes("'wasm-unsafe-eval'"),
            }));
            assert.equal(meta.csp, csp, form);
            assert.equal(meta.icon, 'data:,');
            assert.equal(meta.app, 'route-builder');
            assert.ok(meta.wasmAllowed, `${form}: the policy must allow the preview's WebAssembly (landmine 21)`);
            assert.ok(await p.page.locator('#export').isVisible());
            // the field is the PICTURE: its <image> is an inline WebP that this browser decodes at full size and draws
            const pic = await p.page.evaluate(async () => {
                const img = document.querySelector('.field-svg image');
                const href = img.getAttribute('href') || '';
                const probe = new Image();
                probe.src = href;
                await probe.decode();
                const box = img.getBBox();
                return { kind: href.slice(0, 23), w: probe.naturalWidth, h: probe.naturalHeight, drawnW: box.width };
            });
            assert.deepEqual(pic, { kind: 'data:image/webp;base64,', w: 1138, h: 1138, drawnW: 1138 }, `${form}: the field picture`);
            assert.deepEqual(await p.done(), [], `${form}: errors or outside requests`);
        }
    });

    test('the top bar fits at every laptop width: PREVIEW and Export whole, on screen, never one over the other, on both tabs (1280 px pushed Export off the screen once the Controls tab joined the bar)', async () => {
        for (const width of [1101, 1180, 1280, 1366, 1440, 1536, 1920]) {
            const p = await openPage('site', { viewport: { width, height: 800 } });
            const { page } = p;
            await page.getByRole('button', { name: 'Got it' }).click();
            for (const tab of ['Route', 'Controls']) {
                await page.getByRole('tab', { name: tab }).click();
                const boxes = {};
                for (const sel of ['#export', '#preview']) {
                    if (await page.locator(sel).count()) {
                        const b = await page.locator(sel).boundingBox();
                        const whole = await page.locator(sel).evaluate((e) => e.scrollWidth <= e.clientWidth + 1);
                        assert.ok(b && b.x >= 0 && b.x + b.width <= width, `${width} px, ${tab}: ${sel} is on the screen (${JSON.stringify(b)})`);
                        assert.ok(whole, `${width} px, ${tab}: ${sel}'s words fit inside it`);
                        boxes[sel] = b;
                    }
                }
                assert.ok(boxes['#export'], `${width} px, ${tab}: Export is drawn`);
                if (boxes['#preview']) {
                    assert.ok(boxes['#preview'].x + boxes['#preview'].width <= boxes['#export'].x, `${width} px: PREVIEW ends before Export starts`);
                }
                if (tab === 'Route') {
                    const name = await page.locator('#route-switch strong').evaluate((e) => ({ s: e.scrollWidth, c: e.clientWidth }));
                    assert.ok(name.s <= name.c + 1, `${width} px: the route's name is whole (${name.s} > ${name.c})`);
                }
                const bar = await page.locator('.topbar').evaluate((e) => ({ s: e.scrollWidth, c: e.clientWidth }));
                assert.ok(bar.s <= bar.c + 1, `${width} px, ${tab}: nothing in the top bar runs past it (${bar.s} > ${bar.c})`);
            }
            assert.deepEqual(await p.done(), []);
        }
    });

    test('an error stands: Export LISTS it, one click from its step -- no code, no page error (a wait with no seconds, a fresh level move, "Find the bottom first")', async () => {
        const p = await openPage('site');
        const { page } = p;
        await useLevels(page);
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await answerTogether(page, false);
        await page.keyboard.press('p');  // a wait: its seconds empty (nothing invented)
        assert.equal(await page.locator('#export .count').textContent(), '1', 'the Export button carries the count');
        assert.match(await page.locator('#export').getAttribute('title'), /^Fix this first \(Ctrl\+E lists them\): step 1: A wait needs a time above 0 s\./);
        for (const how of ['keyboard', 'button']) {
            if (how === 'keyboard') {
                await openExport(page);
            } else {
                await page.locator('#export').click();
                await page.locator('.drawer').waitFor();
            }
            const banner = page.locator('.drawer .banner.error');
            assert.match(await banner.textContent(), /Fix this error first: the robot would not run this route\.Step 1: A wait needs a time above 0 s\./);
            assert.equal(await page.locator('.drawer pre.code').count(), 0, 'no code from a route with an error');
            await closeDrawer(page);
        }
        // "Go there" takes the student to the step
        await openExport(page);
        await page.locator('.drawer').getByRole('button', { name: 'Go there' }).click();
        assert.equal(await page.locator('.pane.right .insp-title').textContent(), 'Step 1');
        await page.keyboard.press('Delete');
        // a height move just added: no height, no time -- and no home before it
        await mech(page, 'cascade', 'Move the cascade');
        await openExport(page);
        assert.match(await page.locator('.drawer .banner.error').textContent(), /Fix these \d errors first/);
        await closeDrawer(page);
        // the home card's fix (RMP1-J D6) leaves the home's time EMPTY: still listed, never a dead button
        await page.getByRole('button', { name: 'Find the bottom at the start' }).click();
        const said = await page.locator('.toast', { hasText: 'Type its time limit' }).last().textContent();  // RMP1-M D5: its toast too
        assert.doesNotMatch(said, /\bhom(e|es|ed|ing)\b/i);
        assert.match(said, /find where it is/);
        await openExport(page);
        assert.match(await page.locator('.drawer .banner.error').textContent(), /Step 1: Type the most time this may take/);
        await closeDrawer(page);
        assert.deepEqual(await p.done(), [], 'no page error on the way');
    });

    test('a route built by clicking -- start, drive, curve, a cascade level with "find the bottom first" -- and exported', async () => {
        const p = await openPage('site');
        const { page } = p;
        await useLevels(page);
        await page.getByRole('button', { name: 'Got it' }).click();
        // the skills route keeps its name (RMP1-C D1): shown in the switcher, no box to type it in
        assert.equal(await routeName(page), 'skills');
        assert.equal(await page.locator('input[name="routeName"], .topbar input.route-name').count(), 0);
        // RMP1-H: right of the black goal by the red wall (from (-36, -55.7) the drive to (-12, -36) went through it)
        await clickField(page, -9, -55.7);
        await answerTogether(page, false);  // (its curve comes near the diagonal: named_routes.page.test.mjs checks that)
        let r = await route(page);
        assert.ok(Math.abs(r.start.x - -9) < 0.4 && Math.abs(r.start.y - -55.7) < 0.4, `the start where it was clicked: ${JSON.stringify(r.start)}`);
        assert.equal(r.start.x, Number(r.start.x.toFixed(3)), 'stored on the 0.001 grid (W7)');
        await page.keyboard.press('d');
        await clickField(page, -12, -36);
        r = await route(page);
        assert.equal(r.steps[0].do, 'driveTo');
        assert.equal(r.steps[0].faceFirst, true, 'a leg off its line turns to face its point first by default (R6)');
        await page.keyboard.press('c');
        await clickField(page, 0, -24);
        await clickField(page, 24, -24);
        await page.keyboard.press('Enter');
        r = await route(page);
        assert.equal(r.steps[1].do, 'curveTo');
        assert.equal(r.steps[1].points.length, 2);
        // a curve finished by a DOUBLE-click: the second click is not a second point on the same spot
        await page.keyboard.press('c');
        await clickField(page, 30, -12);
        const dbl = await fieldPoint(page, 40, -6);
        await page.mouse.dblclick(dbl.x, dbl.y);
        r = await route(page);
        assert.equal(r.steps[2].do, 'curveTo');
        assert.equal(r.steps[2].points.length, 2, `a double-click made ${r.steps[2].points.length} points`);
        await page.keyboard.press('Delete');
        assert.equal((await route(page)).steps.length, 2);
        // a cascade level, without a home: the page refuses it, and its ONE card offers the fix (RMP1-J D6; a cascade
        // WITH levels: its menu offers the move, and no "no levels yet")
        await mech(page, 'cascade', 'Move the cascade');
        const rc25 = (await issues(page)).filter((x) => x.code === 'RC25');
        assert.equal(rc25.length, 1);
        assert.equal(await page.locator('.pane.right .home-card').count(), 1);
        await page.locator('.pane.right [data-level="2"]').click();
        assert.equal((await route(page)).steps[2].preset, 2);
        // the levels are still OWED in the facts: no height is shown as a fact
        assert.match(await page.locator('.pane.right [data-level="2"]').textContent(), /not measured yet$/);
        // a typed "2s" is not a number: refused in place, nothing stored
        await typeIn(page, 'boundS', '2s');
        assert.ok(await page.getByText('Not a number: type digits, like 2.5 (in s).').isVisible());
        assert.equal((await route(page)).steps[2].boundS, undefined);
        await typeIn(page, 'boundS', 2);
        // its time limit takes the keyboard AT ONCE (asked in the click's own task): a deferred focus once landed after
        // the next keys, so a redo pressed then was typed into the box instead (RMP1-B's verifier)
        assert.equal(await page.evaluate(() => {
            [...document.querySelectorAll('button')].find((b) => b.textContent.trim() === 'Find the bottom at the start').click();
            return document.activeElement.getAttribute('name');
        }), 'boundS');
        r = await route(page);
        assert.deepEqual(r.steps.map((x) => x.do), ['startHome', 'driveTo', 'curveTo', 'waitUntilDone', 'move'], 'home while the first leg drives');
        assert.equal(r.steps[0].boundS, undefined, 'the home\'s time limit is left EMPTY for the student');
        await typeIn(page, 'boundS', 3);
        // undo / redo by the keyboard
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        assert.equal((await route(page)).steps[0].boundS, undefined);
        await page.keyboard.press('Control+Shift+z');
        assert.equal((await route(page)).steps[0].boundS, 3);
        await page.locator('.step[data-step="3"]').click();
        await typeIn(page, 'boundS', 2);  // the wait for the home
        assert.deepEqual(await errorsOf(page), []);
        // reorder by the keyboard: the move up one, then back (every later pose re-planned)
        await page.locator('.step[data-step="4"]').click();
        await page.keyboard.press('Alt+ArrowUp');
        assert.deepEqual((await route(page)).steps.map((x) => x.do), ['startHome', 'driveTo', 'curveTo', 'move', 'waitUntilDone']);
        await page.keyboard.press('Alt+ArrowDown');
        assert.deepEqual((await route(page)).steps.map((x) => x.do), ['startHome', 'driveTo', 'curveTo', 'waitUntilDone', 'move']);
        // the steps that end at one spot share ONE number pill (none hidden under another): the home at the start,
        // the drive's end, and the curve's end where the cascade is waited for and raised
        const pills = await page.evaluate(() => [...document.querySelectorAll('.g-badge-text')].map((t) => t.textContent).sort());
        assert.deepEqual(pills, ['1', '2', '3\u20135']);
        // a drive's END handle, under its pill, drags (the pill takes no pointer): (-12, -36) -> (-12, -24) -- ONE undo
        // entry: one Ctrl+Z puts back exactly the route before the drag (the reorder just before it is NOT undone)
        const before = await route(page);
        const from = await fieldPoint(page, -12, -36);
        const to = await fieldPoint(page, -12, -24);
        await page.mouse.move(from.x, from.y);
        await page.mouse.down();
        await page.mouse.move(to.x, to.y, { steps: 8 });
        await page.mouse.up();
        r = await route(page);
        assert.deepEqual([r.steps[1].x, r.steps[1].y], [-12, -24], 'the drive\'s end dragged (not the field panned)');
        await page.keyboard.press('Control+z');
        assert.deepEqual(await route(page), before, 'one undo puts back exactly the route before the drag');
        await page.keyboard.press('Control+Shift+z');
        assert.deepEqual([(await route(page)).steps[1].x, (await route(page)).steps[1].y], [-12, -24], 'and redo brings the drag back');
        await page.keyboard.press('Control+z');
        // export: the drawer shows exactly the code; then write it into an in-memory folder (the file layer's own rules)
        await page.evaluate((m) => window.routeBuilder.useMemoryFolder({
            'src/team/skills_route.cpp': '#include "x"\r\n\r\n// ═══ ROUTE MAKER — route "skills" — BEGIN ═══════════════════════════════════════════════\r\nold\r\n// ═══ ROUTE MAKER — route "skills" — END ═════════════════════════════════════════════════\r\n// after\r\n',
            'src/team/autons.cpp': m,
        }), MIRRORED);
        await openExport(page);
        const code = await page.locator('.drawer pre.code').textContent();
        assert.ok(code.includes('r.startHome(cascade, 3_s);'));
        assert.ok(code.includes('r.move(cascade[facts::lv('), 'a level through the facts ledger');
        assert.ok(code.includes('r.face(-12_in, -36_in);'), 'turn-first written');
        // the starter region above is "old" -- not the builder's: a hand edit, refused until "Overwrite"
        await page.getByRole('button', { name: 'Write into src/team/skills_route.cpp' }).click();
        await page.getByRole('button', { name: 'Overwrite' }).click();
        const written = await page.evaluate(() => window.routeBuilder.memory.store['src/team/skills_route.cpp']);
        assert.ok(written.startsWith('#include "x"\r\n\r\n// ═══ ROUTE MAKER'), 'nothing before the region changed');
        assert.ok(written.endsWith('═\r\n// after\r\n'), 'nothing after it changed, CRLF kept');
        assert.ok(!/[^\r]\n/.test(written), 'no bare LF written into a CRLF file');
        assert.ok(written.includes(code.replace(/\n/g, '\r\n')));
        // RMP1-F: read back, then "Will it build?" names the autons line this folder lacks (robot one's), and only it
        const build = await page.locator('.drawer [data-result="ok"] .xr-build').textContent();
        assert.match(build, /White Bot will not list the routes: src\/team\/robot_one_autons\.cpp is missing/);
        assert.doesNotMatch(build, /Black Bot will not/);
        // and the project saved with it (the export's hash), into the folder
        const saved = JSON.parse(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']));
        assert.match(saved.exports['src/team/skills_route.cpp#skills'], /^sha256:[0-9a-f]{64}$/);
        assert.deepEqual(await p.done(), []);
    });

    test('the ONE FILE over file:// builds, checks and copies with no network; PREVIEW (or, built without its engine, says it comes next); what it offers for the folder is what this browser allows there', async (t) => {
        const p = await openPage('onefile');
        const { page } = p;
        // landmine 23 / A.10: the folder needs a SECURE page with the File System Access API -- over file:// that is
        // this browser's call. Recorded on every row, and the page must offer exactly what the browser allows.
        const caps = await page.evaluate(() => ({ secure: window.isSecureContext, picker: typeof window.showDirectoryPicker === 'function' }));
        t.diagnostic(`the ONE FILE over file:// in ${BROWSER}: isSecureContext=${caps.secure}, showDirectoryPicker=${caps.picker}`);
        const offers = caps.secure && caps.picker;
        assert.equal(await page.getByRole('button', { name: 'Connect your shulib folder' }).count(), offers ? 1 : 0);
        assert.equal(await page.getByText('Save downloads · Export copies').count(), offers ? 0 : 1);
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await answerTogether(page, false);
        await page.keyboard.press('d');
        // RMP1-H: through the LEFT wall -- toward the right wall the leg went through two goals, and robot two (mirrored)
        // through a Loader, too: ONE error is the point here
        await clickField(page, -64.5, -36, { modifiers: ['Alt'] });  // Alt: no snapping (it would snap flush to the wall)
        assert.ok((await issues(page)).some((x) => x.code === 'RC1' && x.level === 'error'), 'through the left wall: refused');
        // a site built with the engine (build.py --engine) has PREVIEW; one without says it comes next (preview.test.mjs
        // plays it)
        const engine = fs.existsSync(path.join(SITE, 'engine', 'worker.mjs'));
        assert.equal(await page.locator('#preview').isDisabled(), !engine);
        assert.match(await page.locator('#preview-why').textContent(), engine ? /^PREVIEW: run this route/ : /PREVIEW comes next/);
        await openExport(page);
        assert.ok(await page.getByText(/Fix this error first/).isVisible(), 'Export refused while a step has an error');
        assert.deepEqual(await p.done(), []);
    });

    test('Save and Open with no folder (every browser: Chrome and Edge too, before a folder is connected) -- the same route and the same code back; a NEWER file refused by name', async () => {
        const p = await openPage('site');
        const { page } = p;
        await readyRoute(page);
        await openExport(page);
        const code = await page.locator('.drawer pre.code').textContent();
        await closeDrawer(page);
        const mine = await route(page);
        const [dl] = await Promise.all([page.waitForEvent('download'), page.keyboard.press('Control+s')]);
        assert.equal(dl.suggestedFilename(), 'route_maker.json');
        assert.equal(await page.locator('#save-state').textContent(), 'Downloaded');
        await shot(page, 'saved');  // the guide's Save picture: the save state and what to do with the file
        const text = fs.readFileSync(await dl.path(), 'utf8');
        assert.equal(JSON.parse(text).schemaVersion, 1);
        assert.deepEqual(JSON.parse(text).routes[0], mine);
        // change the page, then Open the saved file: the route AND its export come back exactly
        await page.keyboard.press('p');
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json', text);
        await page.getByText('Opened the project.').waitFor();
        assert.deepEqual(await route(page), mine);
        await openExport(page);
        assert.equal(await page.locator('.drawer pre.code').textContent(), code);
        await closeDrawer(page);
        // a project from a NEWER route builder (A1b's trigger on a drive): refused by name, the page untouched
        const newer = JSON.parse(text);
        newer.routes[0].steps[1].during = [{ name: 'raise', at: { percent: 60 }, then: { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 } }];
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json', JSON.stringify(newer));
        assert.match(await page.locator('.toast.error').textContent(), /a step field "during" .* NEWER route builder\. Nothing was opened/);
        assert.deepEqual(await route(page), mine);
        assert.deepEqual(await p.done(), []);
    });

    test('the browser draft: after a reload it is announced only when it differs from what was last saved', async () => {
        const p = await openPage('site');
        const { page } = p;
        // a visit with no edit leaves only a new, empty project behind: nothing to bring back, nothing announced
        await page.reload();
        await ready(page);
        assert.equal(await page.locator('#save-state').textContent(), 'New route');
        assert.equal(await page.getByText('Your unsaved route from last time is back').count(), 0, 'an empty route is not unsaved work');
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        await Promise.all([page.waitForEvent('download'), page.keyboard.press('Control+s')]);
        await page.reload();
        await ready(page);
        assert.equal(await page.locator('#save-state').textContent(), 'Saved');
        assert.equal(await page.getByText('Your unsaved route from last time is back').count(), 0, 'nothing unsaved: nothing announced');
        assert.equal((await route(page)).steps.length, 1);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('p');
        await page.reload();
        await ready(page);
        assert.equal(await page.locator('#save-state').textContent(), 'Draft kept in this browser');
        assert.ok(await page.getByText('Your unsaved route from last time is back').isVisible());
        assert.equal((await route(page)).steps.length, 2);
        assert.deepEqual(await p.done(), []);
    });

    test('a project file whose start is missing a number (a hand edit\'s null, a key left out) opens: "?" in the list, its error, nothing drawn from it -- typed in, one part at a time; a reload starts', async () => {
        const p = await openPage('site');
        const { page } = p;
        const file = (start, steps = [{ do: 'wait', seconds: 1 }]) => JSON.stringify({ schemaVersion: 1, madeWith: 'a hand edit',
            robots: {}, exports: {},
            routes: [{ name: 'skills', kind: 'skills', robot: 'one', mirroredFor: 'two', together: false, start, endActionAtS: 57, hardStopAtS: 59, steps }] });
        const startCard = page.locator('.start-card');
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json',
                       file({ x: null, y: -36, headingDeg: 90 }));
        await page.getByText('Opened the project.').waitFor();
        assert.deepEqual((await route(page)).start, { y: -36, headingDeg: 90 }, 'the hole is kept: never x = 0');
        assert.deepEqual((await errorsOf(page)).map((x) => [x.code, x.stepIndex]), [['RC3', -1]]);
        assert.match(await startCard.textContent(), /Start at \(\?, -36\)facing 90°/);
        assert.match(await startCard.getAttribute('class'), /has-error/);
        assert.equal(await page.locator('.field-svg .g-robot').count(), 0, 'a start missing a number is no place: not drawn');
        assert.match(await page.locator('.mode-hint').textContent(), /start is missing a number/);
        assert.ok(await page.locator('.pane.right').getByText('The start needs an x, a y and a heading').isVisible());
        // a reload starts (the draft holds the same route) and shows it again
        await page.reload();
        await ready(page);
        assert.equal(await page.evaluate(() => document.documentElement.dataset.ready), '1');
        assert.deepEqual((await route(page)).start, { y: -36, headingDeg: 90 });
        assert.equal(await page.locator('#save-state').textContent(), 'Saved', 'what was opened, unchanged');
        // the student types it: the start is whole again, drawn, and its error gone
        await typeIn(page, 'startX', -36);
        assert.deepEqual((await route(page)).start, { x: -36, y: -36, headingDeg: 90 });
        assert.deepEqual(await errorsOf(page), []);
        assert.equal(await page.locator('.field-svg .g-robot').count(), 1);
        assert.match(await startCard.textContent(), /Start at \(-36, -36\)facing 90°/);
        // a start missing TWO parts (a key left out, and a null), and no step yet: a curve being clicked is drawn from
        // its own first point (not from a start that is nowhere); and each typed part is kept as it is typed
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json',
                       file({ y: null, headingDeg: 90 }, []));
        await page.getByText('Opened the project.').waitFor();
        assert.deepEqual((await route(page)).start, { headingDeg: 90 });
        await page.locator('.field-svg').focus();
        await page.keyboard.press('c');
        await clickField(page, 0, 0);
        await clickField(page, 12, 12);
        const draft = await page.locator('.field-svg polyline.g-draft').getAttribute('points');
        assert.equal(draft.trim().split(/\s+/).length, 2, `the two clicked points only: ${draft}`);
        assert.doesNotMatch(draft, /NaN/);
        await page.keyboard.press('Escape');
        assert.equal((await route(page)).steps.length, 0, 'Esc: no curve added');
        await typeIn(page, 'startX', -36);
        assert.deepEqual((await route(page)).start, { x: -36, headingDeg: 90 }, 'the first part kept while the other is still missing');
        assert.match(await startCard.textContent(), /Start at \(-36, \?\)/);
        await typeIn(page, 'startY', -55.7);
        assert.deepEqual((await route(page)).start, { x: -36, y: -55.7, headingDeg: 90 });
        assert.deepEqual(await errorsOf(page), []);
        // and the page's own boxes never MAKE a hole: emptying x is refused (the start stays whole)
        await typeIn(page, 'startX', '');
        assert.deepEqual((await route(page)).start, { x: -36, y: -55.7, headingDeg: 90 });
        assert.deepEqual(await p.done(), []);
    });

    test('a project file with a name the page does not take (a hand edit: "NONE", the brain menu\'s own) -- its error offers Name it "skills", one click', async () => {
        const p = await openPage('site');
        const { page } = p;
        const r = { name: 'NONE', kind: 'skills', robot: 'one', mirroredFor: 'two', together: false, start: { x: -36, y: -36, headingDeg: 90 },
                    endActionAtS: 57, hardStopAtS: 59, steps: [{ do: 'wait', seconds: 1 }] };
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json',
                       JSON.stringify({ schemaVersion: 1, madeWith: 'a hand edit', robots: {}, exports: {}, routes: [r] }));
        await page.getByText('Opened the project.').waitFor();
        assert.deepEqual((await errorsOf(page)).map((x) => x.code), ['RC21']);
        const box = page.locator('.pane.right .issue.error');
        assert.match(await box.textContent(), /^NONE is the brain menu's own choice for "no auton", so no route can have that name\./);
        assert.equal(await page.locator('.pane.right').getByRole('button', { name: 'Rename' }).count(), 1, 'any other name: Rename');
        await box.getByRole('button', { name: 'Name it "skills"' }).click();
        assert.equal((await route(page)).name, 'skills');
        assert.deepEqual(await errorsOf(page), []);
        assert.equal(await routeName(page), 'skills');
        assert.match(await page.locator('.pane.right .route-name-fixed').textContent(), /keeps its name\.$/);
        assert.deepEqual(await p.done(), []);
    });

    test('a route the page cannot draw (a fault the TEST puts in): Open and the folder leave the page as it was, never as the next visit\'s draft; a draft that cannot be drawn starts a new route', async () => {
        const p = await openPage('site');
        const { page, ctx } = p;
        // the fault: the n-th <ol> the page makes from now on throws (the step list is one) -- armed in the page, or
        // across a reload through sessionStorage
        await ctx.addInitScript(() => {
            window.__rbBreakIn = Number(sessionStorage.getItem('rb-break-in') || 0);
            sessionStorage.removeItem('rb-break-in');
            const make = Document.prototype.createElement;
            Document.prototype.createElement = function createElement(tag, ...rest) {
                if (String(tag).toLowerCase() === 'ol' && window.__rbBreakIn > 0 && --window.__rbBreakIn === 0) {
                    throw new Error('the test made this route undrawable');
                }
                return make.call(this, tag, ...rest);
            };
        });
        await page.reload();
        await ready(page);
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        const mine = await route(page);
        const theirs = JSON.stringify({ schemaVersion: 1, madeWith: 'a teammate', robots: {}, exports: {},
                                        routes: [{ ...mine, steps: [{ do: 'wait', seconds: 7 }] }] });
        // what this browser keeps, read at once (a closed tab or a crash runs no "before unload" to put it right)
        const kept = () => page.evaluate(() => ['draft', 'saved'].map((k) => localStorage.getItem(`shulib-route-builder:${k}`)));
        // Open: the file is read, the page cannot draw it -> put back, said so, and NOT kept as the draft
        await page.evaluate(() => { window.__rbBreakIn = 1; });
        await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json', theirs);
        await page.locator('.toast.error').waitFor();
        assert.match(await page.locator('.toast.error').textContent(),
                     /^Not opened: this page cannot show the route in that file \(the test made this route undrawable\)\. The page is as it was\.$/);
        assert.equal(await page.evaluate(() => window.__rbBreakIn), 0, 'the fault was hit');
        assert.deepEqual(await route(page), mine);
        assert.equal(await page.locator('.step[data-step]').count(), 1, 'the page shows its own route again');
        assert.equal(await page.locator('#save-state').textContent(), 'Unsaved changes');
        const [draft1, saved1] = await kept();
        assert.deepEqual(JSON.parse(draft1).routes[0], mine, 'the draft is still the page\'s route: kept only once drawn');
        assert.equal(saved1, null, 'nothing recorded as saved or opened');
        await page.reload();
        await ready(page);
        assert.deepEqual(await route(page), mine, 'the next visit starts from the page\'s route, not the file it could not show');
        // the draft cannot be drawn at the next visit: a new route, and the student told; the page STARTS
        await page.evaluate(() => sessionStorage.setItem('rb-break-in', '1'));
        await page.reload();
        await ready(page);
        assert.equal(await page.evaluate(() => document.documentElement.dataset.ready), '1', 'the page started');
        assert.equal(await page.evaluate(() => window.__rbBreakIn), 0, 'the fault was hit');
        const fresh = await route(page);
        assert.deepEqual([fresh.start, fresh.steps], [null, []]);
        assert.equal(await page.locator('#save-state').textContent(), 'New route');
        assert.match(await page.locator('.toast.error').textContent(),
                     /^Your route from last time could not be shown \(the test made this route undrawable\), so a new route is open\. Your project file is not changed: Open your route_maker\.json to carry on\.$/);
        // the folder's route_maker.json, adopted on this empty page at connect, cannot be drawn: put back, said so
        await page.evaluate(() => { window.__rbBreakIn = 2; });  // useMemoryFolder draws once before it connects
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), theirs);
        assert.equal(await page.evaluate(() => window.__rbBreakIn), 0, 'the fault was hit');
        assert.match(await page.locator('.toast.error').textContent(),
                     /^src\/team\/route_maker\.json in the folder was not opened: this page cannot show its route \(the test made this route undrawable\)\. The page is as it was\.$/);
        const still = await route(page);
        assert.deepEqual([still.start, still.steps], [null, []]);
        const [draft2, saved2] = await kept();
        assert.ok(draft2 === null || JSON.parse(draft2).routes[0].steps.length === 0, 'the folder\'s route is not the draft');
        assert.equal(saved2, null);
        // the folder's file was not taken: Save ASKS before replacing it; Cancel leaves it
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+s');
        await page.getByRole('alertdialog').waitFor();
        await page.getByRole('button', { name: 'Cancel' }).click();
        assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']), theirs, 'not replaced');
        await page.reload();
        await ready(page);
        const after = await route(page);
        assert.deepEqual([after.start, after.steps], [null, []], 'nor kept as the draft');
        assert.deepEqual(await p.done(), []);
    });

    test('a connected folder: its route_maker.json is compared at connect, and Save asks before replacing one that changed there', async () => {
        const p = await openPage('site');
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        const theirs = await page.evaluate(() => {
            const r = window.routeBuilder.route();
            r.steps = [{ do: 'wait', seconds: 7 }];
            return JSON.stringify({ schemaVersion: 1, madeWith: 'a teammate', routes: [r], robots: {}, exports: {} }, null, 2);
        });
        // the folder has a teammate's route (a git pull): the page asks, and the folder's opens
        const connecting = page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), theirs);
        await page.getByRole('alertdialog').waitFor();
        assert.match(await page.getByRole('alertdialog').textContent(), /The folder has a different route/);
        await page.getByRole('button', { name: 'Open the folder\'s route' }).click();
        await connecting;
        assert.deepEqual((await route(page)).steps, [{ do: 'wait', seconds: 7 }]);
        assert.equal(await page.locator('#save-state').textContent(), 'Saved');
        // the page's edit, then the file changes in the folder again: Save ASKS; Cancel leaves the file alone
        await page.locator('.field-svg').focus();
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 2);
        const again = theirs.replace('"seconds": 7', '"seconds": 8');
        await page.evaluate((t) => { window.routeBuilder.memory.store['src/team/route_maker.json'] = t; }, again);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+s');
        await page.getByRole('alertdialog').waitFor();
        assert.match(await page.getByRole('alertdialog').textContent(), /changed in the folder since this page read it/);
        await page.getByRole('button', { name: 'Cancel' }).click();
        assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']), again, 'not replaced');
        // and "Replace" is the student's choice
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+s');
        await page.getByRole('button', { name: 'Replace it with this page\'s route' }).click();
        await page.getByText('Saved src/team/route_maker.json').waitFor();
        const now = JSON.parse(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']));
        assert.deepEqual(now.routes[0].steps.map((s) => s.seconds), [7, 2]);
        assert.deepEqual(await p.done(), []);
    });

    test('the download way (a browser that cannot write a folder): pick the file, a hand edit refused, Overwrite, and Save asked for', {
        skip: BROWSER !== 'firefox' && 'Chrome and Edge write into the folder (the tests above); this branch is the other browsers\' -- Firefox here',
    }, async () => {
        const p = await openPage('site');
        const { page } = p;
        await readyRoute(page);
        await openExport(page);
        const code = await page.locator('.drawer pre.code').textContent();
        const handEdited = `${SKILLS_HEAD}${BEGIN}\r\nint tunedByHand;\r\n${END}\r\n\r\n}  // namespace shulib::bench\r\n`;
        const pickBtn = () => page.getByRole('button', { name: 'Pick skills_route.cpp and download the updated one' }).click();
        await pickFile(page, pickBtn, 'skills_route.cpp', handEdited);
        await page.getByRole('button', { name: 'Overwrite' }).waitFor();
        assert.match(await page.locator('.drawer .banner.warn').textContent(), /changed the code between the ROUTE MAKER lines by hand/);
        await settle(page);
        await shot(page, 'download-hand-edit');
        const [dl] = await Promise.all([page.waitForEvent('download'),
                                       pickFile(page, () => page.getByRole('button', { name: 'Overwrite' }).click(), 'skills_route.cpp', handEdited)]);
        assert.equal(dl.suggestedFilename(), 'skills_route.cpp');
        const back = fs.readFileSync(await dl.path(), 'utf8');
        assert.ok(!back.includes('tunedByHand'));
        assert.ok(back.includes(code.replace(/\n/g, '\r\n')), 'the route, CRLF kept');
        assert.ok(back.startsWith(SKILLS_HEAD) && back.endsWith('\r\n\r\n}  // namespace shulib::bench\r\n'), 'nothing outside the region moved');
        // the project must be saved now, or the next export from the file takes this code for a hand edit
        assert.equal(await page.locator('#save-state').textContent(), 'Unsaved changes');
        const [pj] = await Promise.all([page.waitForEvent('download'), page.getByRole('button', { name: 'Save route_maker.json' }).click()]);
        const saved = JSON.parse(fs.readFileSync(await pj.path(), 'utf8'));
        assert.match(saved.exports['src/team/skills_route.cpp#skills'], /^sha256:[0-9a-f]{64}$/);
        assert.deepEqual(await p.done(), []);
    });

    test('the old-browser refusal: a Firefox without WebAssembly is told, in plain words, the preview cannot run', { skip: BROWSER !== 'firefox' && 'the refusal is proven in Firefox (its WebAssembly switch)' }, async () => {
        const old = await launch({ firefoxUserPrefs: { 'javascript.options.wasm': false } });
        try {
            const p = await openPage('onefile', { b: old });
            const box = p.page.locator('#too-old-preview');
            assert.ok(await box.isVisible());
            assert.match(await box.textContent(), /too old to run the route preview.*Chrome or Edge 97, Firefox 115 or Safari 16.*still opens, saves and exports/s);
            await shot(p.page, 'old-browser-refusal');
            assert.ok(await p.page.locator('#export').isVisible(), 'the builder itself still opens');
            assert.deepEqual(await p.done(), []);
        } finally {
            await old.close();
        }
    });

    // ── §2.5.2: EVERY A1 screen, light and dark, at 1440 x 900 and 1024 x 768 (screenshots with ROUTE_MAKER_SHOTS) ──
    for (const v of VARIANTS) {
        test(`§2.5.2 [${v.tag}] first run; the ready route; [R6] a drive NOT turned first; a curve with a refused point`, async () => {
            const p = await openPage('site', v);
            const { page } = p;
            await shot(page, `first-run-${v.tag}`);
            await readyRoute(page);
            await selectStep(page, v.viewport, 1);
            assert.ok(await page.getByText('Ready to export').isVisible());
            await settle(page);
            await shot(page, `route-${v.tag}`);
            await page.locator('.pane.right input[name="faceFirst"]').uncheck();
            const rc24 = (await issues(page)).filter((x) => x.code === 'RC24');
            assert.equal(rc24.length, 1, 'the drive starts ~25 deg off its line: refused on a tank');
            assert.equal(rc24[0].level, 'error');
            assert.ok(await page.getByRole('button', { name: 'Turn to face it first' }).isVisible(), 'the one-click fix offered');
            await settle(page);
            await shot(page, `turn-first-off-${v.tag}`);
            await page.getByRole('button', { name: 'Turn to face it first' }).click();
            assert.equal((await route(page)).steps[1].faceFirst, true);
            // the curve's first point moved behind the robot (typed: the student's number)
            await selectStep(page, v.viewport, 2);
            await typeIn(page, 'x', 0, '.pane.right table.points tr:first-child');
            await typeIn(page, 'y', -48, '.pane.right table.points tr:first-child');
            // the library's own verdict (curve.mjs is checkCurve ported, pinned by CV1): the path into point 1 doubles
            // back on the robot's heading -- refused, in the library's words, the point named
            const rc26 = (await issues(page)).filter((x) => x.code === 'RC26');
            assert.equal(rc26.length, 1);
            assert.equal(rc26[0].level, 'error');
            assert.match(rc26[0].words, /will not drive this curve \(point 1\)/);
            assert.match(rc26[0].library, /^curve not driven: point 1 doubles back/);
            await settle(page);
            await shot(page, `curve-refused-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });

        test(`§2.5.2 [${v.tag}] [R1] a cascade move with no home; a step missing its number, and Export listing it`, async () => {
            const p = await openPage('site', v);
            const { page } = p;
            await page.getByRole('button', { name: 'Got it' }).click();
            await clickField(page, -12, -60.3);  // RMP1-H: the ready route's start and drive, clear of the goals
            await typeIn(page, 'startH', 90);
            await answerTogether(page, false);
            await settle(page);
            await shot(page, `start-placed-${v.tag}`);
            await useLevels(page);
            await page.keyboard.press('d');
            await clickField(page, 0, -36);
            await mech(page, 'cascade', 'Move the cascade');
            await page.locator('.pane.right [data-level="2"]').click();
            await typeIn(page, 'boundS', 1.5);
            await selectStep(page, v.viewport, 1);  // (at 1024 the step's details are a drawer: open it)
            const rc25 = (await issues(page)).filter((x) => x.code === 'RC25');
            assert.equal(rc25.length, 1);
            // RMP1-J D6: ONE card, one click -- not the rule's two fixes
            assert.ok(await page.getByRole('button', { name: 'Find the bottom at the start' }).isVisible());
            assert.equal(await page.getByRole('button', { name: 'Find the bottom first' }).count(), 0);
            await settle(page);
            await shot(page, `home-it-first-${v.tag}`);
            await page.getByRole('button', { name: 'Find the bottom at the start' }).click();
            await settle(page);
            await shot(page, `inline-error-${v.tag}`);  // the home just added: its time limit empty and marked, typed by the student
            await page.locator('.field-svg').focus();
            await page.keyboard.press('Control+e');
            await page.locator('.drawer').waitFor();
            assert.match(await page.locator('.drawer .banner.error').textContent(), /Step 1: Type the most time this may take/);
            await settle(page);
            await shot(page, `export-errors-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });

        test(`§2.5.2 [${v.tag}] robot one's cascade has NO levels (ROB1-C2): home and a wait offered, where levels come from said -- never an empty list`, async () => {
            const p = await openPage('site', v);
            const { page } = p;
            await page.getByRole('button', { name: 'Got it' }).click();
            await clickField(page, -36, -60.3);
            await typeIn(page, 'startH', 90);
            await page.locator('.field-svg').focus();
            await page.keyboard.press('m');
            // RMP1-J D1: the mechanisms first, then what the chosen one does (the cascade: ONE move -- a height, no levels)
            const entries = async (ident) => {
                await page.locator(`.mech-menu [data-mech="${ident}"]`).click();
                const out = await page.locator('.mech-menu .menu-item.big').evaluateAll((els) => els.map((e) => e.textContent.trim()));
                await page.keyboard.press('Escape');  // back to the list
                return out;
            };
            assert.deepEqual(await entries('cascade'), ['Move the cascade', 'Find the bottom']);
            assert.deepEqual(await entries('claw'), ['Close claw', 'Open claw']);
            assert.deepEqual(await entries('ramp'), ['Ramp up', 'Ramp down']);
            await page.waitForFunction(() => !document.querySelector('.toast'), null, { timeout: 15000 });
            await shot(page, `mechanism-menu-${v.tag}`);
            await page.keyboard.press('Escape');
            // a project from when the cascade had levels: its move opens -- no empty list, the same words, Export lists it
            const old = JSON.stringify({ schemaVersion: 1, madeWith: 'an earlier route builder', robots: {}, exports: {},
                                         routes: [{ name: 'skills', kind: 'skills', robot: 'one', mirroredFor: 'two', together: false, start: { x: -36, y: -55.7, headingDeg: 90 },
                                                    endActionAtS: 57, hardStopAtS: 59, steps: [{ do: 'home', mechanism: 'cascade', boundS: 3 },
                                                                                               { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 }] }] });
            await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json', old);
            await page.getByText('Opened the project.').waitFor();
            await selectStep(page, v.viewport, 1);
            assert.equal(await page.locator('.pane.right [data-level]:not([data-level="bottom"])').count(), 0, 'no level list');
            // robot two, robot one's twin (ROB1-BT), builds the mirrored route: its cascade has none either, said on its own line
            const said = (who) => `${{ one: 'White Bot', two: 'Black Bot' }[who]}'s cascade has no levels yet: record them at Bench Tests station 13.`;
            assert.deepEqual((await errorsOf(page)).map((x) => [x.code, x.stepIndex, x.words]), [['RC18', 1, said('one')], ['RC18', 1, said('two')]]);
            for (const who of ['one', 'two']) {
                assert.ok(await page.locator('.pane.right .issue.error').getByText(said(who)).isVisible(), who);
            }
            await openExport(page);
            assert.match(await page.locator('.drawer .banner.error').textContent(), /Step 2: White Bot's cascade has no levels yet/);
            await closeDrawer(page);
            assert.deepEqual(await p.done(), []);
        });

        test(`§2.5.2 [${v.tag}] the export drawer's states: the folder not connected; Copy only (no skills-route file yet, robot two's line missing); no markers; a hand edit`, async () => {
            const p = await openPage('site', v);
            const { page } = p;
            await readyRoute(page);
            // (1) Chrome/Edge before "Connect": one button
            await page.evaluate(() => window.routeBuilder.useMemoryFolder({}, { connected: false }));
            await openExport(page);
            assert.ok(await page.locator('.drawer').getByRole('button', { name: 'Connect your shulib folder' }).isVisible());
            await settle(page);
            await shot(page, `export-not-connected-${v.tag}`);
            await closeDrawer(page);
            // (2) TODAY's folder: no src/team/skills_route.cpp yet (robot two's update adds it), robot two's line not there
            await page.evaluate(() => window.routeBuilder.useMemoryFolder({ 'src/team/autons.cpp': 'void registerAutons() {}\n' }));
            await openExport(page);
            assert.ok(await page.getByText('src/team/skills_route.cpp is not in this folder yet: it arrives with Black Bot\'s update. Until then, use Copy.').isVisible());
            assert.ok(await page.getByRole('button', { name: 'Write into src/team/skills_route.cpp' }).isDisabled());
            assert.ok(await page.getByRole('button', { name: 'Copy the code' }).isEnabled());
            assert.ok(await page.getByText(/The line that registers the routes on Black Bot \(robot two\) is missing from src\/team\/autons\.cpp/).isVisible());
            await settle(page);
            await shot(page, `export-copy-only-${v.tag}`);
            await closeDrawer(page);
            // (3) the file is there but has no markers (someone deleted them): the builder leaves it alone
            await page.evaluate((m) => window.routeBuilder.useMemoryFolder({ 'src/team/skills_route.cpp': 'int x;\n', 'src/team/autons.cpp': m }), MIRRORED);
            await openExport(page);
            assert.ok(await page.getByText('src/team/skills_route.cpp has no ROUTE MAKER lines: someone removed them.').isVisible());
            assert.ok(await page.getByText('Black Bot (robot two) registers every route: its line is in src/team/autons.cpp. ✓').isVisible());
            await settle(page);
            await shot(page, `export-no-markers-${v.tag}`);
            await closeDrawer(page);
            // (4) a hand edit between the markers: keep it or overwrite it -- the student's choice
            const skills = `${SKILLS_HEAD}${BEGIN}\r\nint tunedByHand;\r\n${END}\r\n\r\n}  // namespace shulib::bench\r\n`;
            await page.evaluate(([s, m]) => window.routeBuilder.useMemoryFolder({ 'src/team/skills_route.cpp': s, 'src/team/autons.cpp': m }), [skills, MIRRORED]);
            await openExport(page);
            await page.getByRole('button', { name: 'Write into src/team/skills_route.cpp' }).click();
            await page.getByRole('button', { name: 'Keep their change' }).waitFor();
            await settle(page);
            await shot(page, `export-hand-edit-${v.tag}`);
            await page.getByRole('button', { name: 'Keep their change' }).click();
            assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/skills_route.cpp']), skills, 'kept: nothing written');
            assert.deepEqual(await p.done(), []);
        });

        test(`§2.5.2 [${v.tag}] a project from a newer route builder, refused by name`, async () => {
            const p = await openPage('site', v);
            const { page } = p;
            await page.getByRole('button', { name: 'Got it' }).click();
            const newer = JSON.stringify({ schemaVersion: 1, madeWith: 'a newer route builder', robots: {}, exports: {},
                                           routes: [{ name: 'skills', kind: 'skills', robot: 'one', mirroredFor: 'two', start: { x: -36, y: -60, headingDeg: 90 },
                                                      endActionAtS: 57, hardStopAtS: 59, steps: [{ do: 'wallReset', wall: 'red', sensor: 'right' }] }] });
            await pickFile(page, () => page.getByRole('button', { name: 'Open a project' }).click(), 'route_maker.json', newer);
            assert.match(await page.locator('.toast.error').textContent(), /a step kind "wallReset" .* NEWER route builder/);
            await shot(page, `newer-file-${v.tag}`);  // no settle: the refusal is the toast itself
            assert.equal((await route(page)).start, null, 'nothing opened');
            assert.deepEqual(await p.done(), []);
        });
    }
});
