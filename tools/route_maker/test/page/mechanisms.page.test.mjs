// mechanisms.page.test.mjs — THE MECHANISMS IN PREVIEW (RMP1-E), in a real browser: the strip beside the field plays
// the record's own states in step with the robot, one mechanism accented. The engine is the STAND-IN (recorded runs:
// test/fixtures/preview/mechanisms.preview.json is the brief's route). With ROUTE_MAKER_SHOTS, the strip light and
// dark at 1440 and 1280, a short window's, and guide 22's three PREVIEW pictures (docs-*, light 1440).
//
//   ROUTE_MAKER_BROWSER=firefox node --test tools/route_maker/test/page/mechanisms.page.test.mjs
import test from 'node:test';
import assert from 'node:assert/strict';
import { SHOTS, launch, noStructures, openPage, serve, shot, skip } from './harness.mjs';
import { fixture, standinScript } from '../standin_engine.mjs';
import { sampleIndexAt } from '../../src/model/playback.mjs';
import { loadProject } from '../../src/model/project.mjs';

const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900], ['light', 1280, 800]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));

let browser;
let site;

/** The page with the stand-in (`standin`: its options), fixture `name` opened (`together`: robot two's question
 *  answered) and PREVIEWed, paused at the start; `run: false` stops before PREVIEW. */
async function previewed(name, v = VARIANTS[0], { standin = {}, together, run = true } = {}) {
    const p = await openPage(browser, site.base, v);
    const { page } = p;
    await page.evaluate((text) => window.routeBuilder.useEngine({ text }), standinScript(['mechanisms', 'ready', 'failed'], { together: ['mechanisms', 'ready', 'failed'], ...standin }));
    await page.evaluate((r) => window.routeBuilder.useRobots(r), noStructures());  // RMP1-H: the recordings predate the check
    const coach = page.getByRole('button', { name: 'Got it' });
    if (await coach.count()) {
        await coach.click();
    }
    const f = fixture(name);
    const project = JSON.parse(f.text);
    if (together !== undefined) {
        project.routes[0].together = together;
    }
    const text = `${JSON.stringify(project, null, 2)}\n`;
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
    await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(text, 'utf8') });
    await page.waitForFunction((want) => JSON.stringify(window.routeBuilder.route()) === want, JSON.stringify(loadProject(text).project.routes[0]));
    await page.locator('.field-svg').focus();
    if (run) {
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open');
        await page.evaluate(() => window.routeBuilder.pause());
        await page.keyboard.press('Home');
    }
    return { ...p, f, rec: f.result.one };
}
/** A picture at rest: no toast left. */
async function picture(page, name) {
    await page.waitForFunction(() => !document.querySelector('.toast'), null, { timeout: 15000 });
    await shot(page, name, true);
}
/** Step `k`'s start (Home, then Right k times). */
async function toStep(page, k) {
    await page.keyboard.press('Home');
    for (let j = 0; j < k; ++j) {
        await page.keyboard.press('ArrowRight');
    }
}
const t = (page) => page.evaluate(() => window.routeBuilder.preview().t);
/** What the strip shows: each mechanism's value (the bar's exact number), words, fill and whether it is accented. */
const strip = (page) => page.evaluate(() => [...document.querySelectorAll('.mech-strip .ms-mech')].map((e) => ({
    name: e.dataset.mech, on: e.classList.contains('on'), word: (e.querySelector('.ms-word, .ms-value') || {}).textContent,
    now: e.querySelector('.ms-track')?.getAttribute('aria-valuenow') ?? null, fill: e.querySelector('.ms-fill')?.style.height ?? null,
    mark: e.querySelector('.ms-target') ? !e.querySelector('.ms-target').hidden : null, to: e.querySelector('.ms-to')?.textContent ?? null,
    rod: e.querySelector('.ms-rod')?.getAttribute('width') ?? null })));

test.describe('RMP1-E — the mechanisms in PREVIEW', { skip }, () => {
    test.before(async () => {
        site = await serve();
        browser = await launch();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('the cascade\'s bar IS the record\'s height wherever the playback is: scrubbed, stepped, played', async () => {
        // BUG CAUGHT: a bar a pace behind the robot, a bar that only moves while playing (not when scrubbed), a fill
        // that does not match its number.
        const p = await previewed('mechanisms');
        const { page, rec } = p;
        const cascade = rec.mechanisms[0];
        const check = async (why) => {
            const now = await t(page);
            const want = cascade.at[sampleIndexAt(rec.samples, now)];
            const [c] = await strip(page);
            assert.equal(c.name, 'cascade');
            assert.equal(Number(c.now), want, `${why} (t ${now})`);
            const level = (want - cascade.bottom) / (cascade.top - cascade.bottom);
            assert.ok(Math.abs(parseFloat(c.fill) - 100 * Math.min(Math.max(level, 0), 1)) < 1e-3, `${why}: the fill`);
            assert.equal(c.word, `${(Math.round(want * 10) / 10 || 0).toFixed(1)} in`);
            return want;
        };
        assert.equal(await check('at the start'), cascade.at[0]);
        const box = await page.locator('.track').boundingBox();
        const heights = [];
        for (const f of [0.1, 0.2, 0.3, 0.45, 0.6, 0.75, 0.9]) {  // the track, clicked
            await page.mouse.click(box.x + box.width * f, box.y + box.height / 2);
            heights.push(await check(`scrubbed to ${f}`));
        }
        assert.ok(Math.max(...heights) > 3 && Math.min(...heights) < 1, `it rises and comes back (${heights.join(', ')})`);
        await page.keyboard.press('Home');
        for (let k = 0; k < 4; ++k) {  // each step's start, then 0.1 s on
            await page.keyboard.press('ArrowRight');
            await check(`step ${k + 2}'s start`);
            await page.keyboard.press('Shift+ArrowRight');
            await check(`step ${k + 2} + 0.1 s`);
        }
        await page.keyboard.press('Home');
        for (const ms of [40, 300, 700, 1200]) {  // played: one frame `ms` on at 1x
            await page.evaluate((x) => window.routeBuilder.frameAfter(x), ms);
            await check(`played ${ms} ms`);
        }
        assert.deepEqual(await p.done(), []);
    });

    test('the claw\'s piston changes AT its step; only the mechanism the playing step uses is accented, its target marked', async () => {
        // BUG CAUGHT: the piston changing before or after its step, two highlights at once, a target on a lift whose
        // step is not playing, a stopped toggle drawn running.
        const p = await previewed('mechanisms');
        const { page, rec } = p;
        const starts = rec.pageSteps.map((s) => rec.samples[s.first][0]);
        const at = async (k) => {
            await toStep(page, k);
            assert.equal(await t(page), starts[k]);
            return strip(page);
        };
        const want = ['cascade', 'cascade', 'cascade', 'cascade', 'claw', 'right toggle'];
        const targets = [0, 6, 4, 0];
        for (let k = 0; k < want.length; ++k) {
            const s = await at(k);
            assert.deepEqual(s.filter((m) => m.on).map((m) => m.name), [want[k]], `step ${k + 1}: ONE accented`);
            assert.equal(s[0].mark, k < 4, `step ${k + 1}: the cascade's target marked only while its step plays`);
            assert.equal(s[0].to, k < 4 ? `to ${targets[k]} in` : '');
        }
        const claw = async () => (await strip(page)).find((m) => m.name === 'claw');
        const before = await at(4);
        assert.equal(before.find((m) => m.name === 'claw').word, 'closed', 'closed where its step begins');
        const rod = before.find((m) => m.name === 'claw').rod;
        assert.equal(rod, '4', 'closed: the rod in');
        await page.keyboard.press('Shift+ArrowLeft');
        assert.equal((await claw()).word, 'closed', 'closed before its step');
        await page.keyboard.press('Shift+ArrowRight');
        await page.keyboard.press('Shift+ArrowRight');
        assert.equal((await claw()).word, 'open', 'open just after its step began');
        assert.equal((await claw()).rod, '18', 'open (its last position): the rod all the way out');
        const run = await at(5);
        assert.equal(run.find((m) => m.name === 'right toggle').word, 'off', 'off where its step begins');
        await page.keyboard.press('Shift+ArrowRight');
        const running = (await strip(page)).filter((m) => m.word === 'backward');
        assert.deepEqual(running.map((m) => m.name), ['right toggle'], 'the right toggle runs backward; the left does not');
        assert.equal(await page.locator('.ms-motor.running').count(), 1);
        await page.keyboard.press('End');
        assert.equal((await strip(page)).filter((m) => m.on).length, 0, 'the end: nothing accented');
        assert.deepEqual(await p.done(), []);
    });

    test('a step that uses no mechanism (a drive, a wait) accents none: the last mechanism\'s highlight does not linger', async () => {
        // BUG CAUGHT: the cascade still lit while the robot drives after its step.
        const p = await previewed('ready');  // home the cascade, drive, curve, open the claw, wait
        const want = ['cascade', null, null, 'claw', null];
        for (let k = 0; k < want.length; ++k) {
            await toStep(p.page, k);
            assert.deepEqual((await strip(p.page)).filter((m) => m.on).map((m) => m.name), want[k] ? [want[k]] : [], `step ${k + 1}`);
        }
        assert.deepEqual(await p.done(), []);
    });

    test('the targets are the plan of the route that RAN: a step deleted during the run, then undone, marks 0, 6, 4, 0 in', async () => {
        // BUG CAUGHT: the plan taken from the route on screen when the run opens (the edited one), not the run's own.
        const p = await previewed('mechanisms', VARIANTS[0], { standin: { delayMs: 2500 }, run: false });
        const { page } = p;
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'running');
        await page.locator('.step[data-step="0"]').click();
        await page.keyboard.press('Delete');
        assert.equal((await page.evaluate(() => window.routeBuilder.route())).steps.length, 5);
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open');
        assert.equal(await page.evaluate(() => window.routeBuilder.preview().stale), true, 'the run is put away: the route changed');
        await page.keyboard.press('Control+z');
        await page.waitForFunction(() => window.routeBuilder.preview().stale === false);
        await page.evaluate(() => window.routeBuilder.pause());
        await page.locator('.dock .track').focus();
        const targets = [];
        for (let k = 0; k < 4; ++k) {
            await toStep(page, k);
            targets.push((await strip(page))[0].to);
        }
        assert.deepEqual(targets, ['to 0 in', 'to 6 in', 'to 4 in', 'to 0 in']);
        assert.deepEqual(await p.done(), []);
    });

    test('[light-1366x657] a short window scrolls the strip: the accented mechanism is brought into view', async () => {
        // BUG CAUGHT: the highlight on a row scrolled out of sight (a laptop's 768 px screen less the browser's bars).
        const v = { scheme: 'light', viewport: { width: 1366, height: 657 }, tag: 'light-1366x657' };
        const p = await previewed('mechanisms', v);
        const { page } = p;
        assert.ok(await page.evaluate(() => { const e = document.querySelector('.mech-strip'); return e.scrollHeight > e.clientHeight; }),
                  'this window is short enough to scroll the strip');
        const inView = () => page.evaluate(() => {
            const e = document.querySelector('.mech-strip');
            const r = e.querySelector('.ms-mech.on').getBoundingClientRect();
            const s = e.getBoundingClientRect();
            const top = s.top + e.clientTop;
            return r.top >= top - 0.5 && r.bottom <= top + e.clientHeight + 0.5;
        });
        for (let k = 0; k < 6; ++k) {
            await toStep(page, k);
            assert.ok(await inView(), `step ${k + 1}'s mechanism in view`);
        }
        await toStep(page, 3);  // the cascade accented; the strip then scrolled to its end by hand
        await page.evaluate(() => { document.querySelector('.mech-strip').scrollTop = 1e6; });
        await page.keyboard.press('ArrowRight');  // the claw
        assert.ok(await inView(), 'the claw in view');
        await page.evaluate(() => { document.querySelector('.mech-strip').scrollTop = 1e6; });
        await page.keyboard.press('ArrowLeft');  // back to the cascade
        assert.ok(await inView(), 'the cascade scrolled back into view');
        await shot(page, `mechanisms-short-${v.tag}`, true);
        assert.deepEqual(await p.done(), []);
    });

    test('robot one\'s, said; the plant\'s, said in About this run; a run with no mechanisms shows no strip; the strip goes with the run', async () => {
        const p = await previewed('mechanisms');
        const { page } = p;
        assert.equal(await page.locator('.mech-strip h3').textContent(), 'White Bot\'s mechanisms');
        assert.match(await page.locator('.mech-strip [data-mech="cascade"] .ph').getAttribute('title'), /^PLACEHOLDER numbers/);
        await page.getByRole('button', { name: /About this run/ }).click();
        assert.match(await page.locator('.dock-details .ms-about').textContent(), /host test plant's too/);
        // beside the field, never over it: the field, the zoom and the dock keep clear of it
        const [s, svg, dock, zoom, bar] = await Promise.all(['.mech-strip', '.field-svg', '.dock', '.zoom', '.toolbar'].map((q) => page.locator(q).boundingBox()));
        assert.ok(svg.x + svg.width <= s.x && zoom.x + zoom.width <= s.x, 'the field and the zoom left of the strip');
        assert.ok(s.y + s.height <= dock.y && s.y >= bar.y + bar.height, 'between the toolbar and the dock');
        await page.keyboard.press('Escape');
        assert.equal(await page.locator('.mech-strip').isVisible(), false, 'closed with the run');
        assert.equal(await page.locator('.fieldpane.with-strip').count(), 0, 'the field takes its width back');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open');
        assert.equal(await page.locator('.mech-strip').isVisible(), true);
        assert.deepEqual(await p.done(), []);
        const old = await previewed('ready', VARIANTS[0], { standin: { omit: { ready: ['mechanisms'] } } });  // an engine before RMP1-E
        assert.equal(await old.page.locator('.mech-strip').isVisible(), false);
        await old.page.getByRole('button', { name: /About this run/ }).click();
        assert.equal(await old.page.locator('.dock-details').count(), 1);
        assert.equal(await old.page.locator('.dock-details .ms-about').count(), 0, 'no mechanisms: no line about them');
        assert.deepEqual(await old.done(), []);
    });

    test('guide 22\'s PREVIEW pictures (with ROUTE_MAKER_SHOTS): robot one partway through the curve; a drive that ran out of time', { skip: !SHOTS }, async () => {
        const p = await previewed('ready', VARIANTS[0], { together: false });  // one at a time: no too-close mark
        await toStep(p.page, 2);
        for (let k = 0; k < 12; ++k) {
            await p.page.keyboard.press('Shift+ArrowRight');
        }
        await picture(p.page, 'docs-preview');
        assert.deepEqual(await p.done(), []);
        const q = await previewed('failed', VARIANTS[0], { together: false });
        await toStep(q.page, 1);
        for (let k = 0; k < 3; ++k) {
            await q.page.keyboard.press('Shift+ArrowRight');
        }
        await picture(q.page, 'docs-preview-failed');
        assert.deepEqual(await q.done(), []);
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] the strip at rest beside the field: no overlap, the cascade mid-move accented; the claw opening`, async () => {
            const p = await previewed('mechanisms', v, { together: false });  // robot two's question answered: no red
            const { page } = p;
            assert.equal(await page.locator('.ask-together.unanswered').count(), 0);
            await page.keyboard.press('ArrowRight');
            for (let k = 0; k < 4; ++k) {
                await page.keyboard.press('Shift+ArrowRight');
            }
            const [s, dock] = await Promise.all(['.mech-strip', '.dock'].map((q) => page.locator(q).boundingBox()));
            assert.ok(s.y + s.height <= dock.y, 'above the dock');
            assert.ok(await page.evaluate(() => { const e = document.querySelector('.mech-strip'); return e.scrollHeight <= e.clientHeight; }),
                      'every mechanism in view without scrolling');
            await picture(page, `mechanisms-cascade-${v.tag}`);
            for (let k = 0; k < 3; ++k) {
                await page.keyboard.press('ArrowRight');
            }
            await page.keyboard.press('Shift+ArrowRight');
            await picture(page, `mechanisms-claw-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });
    }
});
