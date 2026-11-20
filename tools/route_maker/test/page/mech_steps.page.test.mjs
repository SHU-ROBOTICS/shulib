// mech_steps.page.test.mjs — RMP1-J in a browser: M's two-step menu (D1), the height bar (D2), wait or keep driving
// (D3), the time limit PREVIEW suggests (D4: the REAL engine when the site has it), the badges on the field (D5), the
// home card (D6). Every number typed is the test's. ROUTE_MAKER_SHOTS: the menu's two steps, the height bar and a
// route with three badges, light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, SHOTS, skip, launch, serve, openPage, shot } from './harness.mjs';
import { allowedWarning } from './allowed_warnings.mjs';
import { suggestS } from '../../src/model/mech_steps.mjs';
import { fixtureRobot, ledgerRobot } from '../helpers.mjs';

const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());
const toScreen = (page, x, y) => page.evaluate(([fx, fy]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(document.querySelector('.field-svg').getScreenCTM());
    return { x: pt.x, y: pt.y };
}, [x, y]);
async function clickField(page, x, y) {
    const at = await toScreen(page, x, y);
    await page.mouse.click(at.x, at.y);
}
async function typeIn(page, name, value) {
    await page.locator(`.pane.right input[name="${name}"]`).fill(String(value));
    await page.locator(`.pane.right input[name="${name}"]`).press('Enter');
}
/** A fresh page, robot one's start typed exactly, "one at a time" answered. */
async function started(v) {
    const p = await openPage(browser, site.base, v);
    const { page } = p;
    await page.getByRole('button', { name: 'Got it' }).click();
    await clickField(page, -36, -55.7);
    await typeIn(page, 'startX', -36);
    await typeIn(page, 'startY', -55.7);
    await typeIn(page, 'startH', 90);
    await page.locator('.pane.right [data-together="false"]').click();
    return p;
}
/** M, the mechanism, then what it does (the two clicks). */
async function add(page, mech, words) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('m');
    await page.locator(`.mech-menu [data-mech="${mech}"]`).click();
    await page.locator(`.mech-menu [data-act="${words}"]`).click();
}
/** Robot one (and its twin) are `r` on this page from now on. */
const useRobot = (page, r) => page.evaluate((x) => window.routeBuilder.useRobots({ one: x, two: { ...x, id: 'two', name: 'robot two (test)' } }), { ...r, id: 'one' });
const focused = (page) => page.evaluate(() => document.activeElement.dataset.mech);
const boxOf = (loc) => loc.evaluate((e) => {
    const b = e.getBoundingClientRect();
    return { l: b.left, r: b.right, t: b.top, b: b.bottom };
});
const overlap = (a, b) => a.l < b.r && b.l < a.r && a.t < b.b && b.t < a.b;
const menuWords = (page, sel) => page.locator(`.mech-menu ${sel}`).evaluateAll((els) => els.map((e) => e.childNodes[1].textContent.trim()));

test.describe('RMP1-J in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('D1: M -> a mechanism -> what it does adds that step after the selected one; Esc goes back, then closes; the arrows and Enter walk it', async () => {
        const p = await started();
        const { page } = p;
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -36, -36);
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        await page.locator('.step[data-step="0"]').click();  // the drive selected: the next step goes after it
        await page.locator('.field-svg').focus();
        await page.keyboard.press('m');
        assert.deepEqual(await menuWords(page, '.mech-row'), ['Cascade', 'Right toggle', 'Left toggle', 'Claw', 'Flipper', 'Ramp'], 'one row each');
        assert.equal(await page.evaluate(() => document.activeElement.dataset.mech), 'cascade');
        await page.locator('.mech-menu [data-mech="claw"]').click();
        assert.deepEqual(await menuWords(page, '.menu-item.big'), ['Close claw', 'Open claw']);
        await page.keyboard.press('Escape');  // back to the list, on the claw
        assert.equal(await page.evaluate(() => document.activeElement.dataset.mech), 'claw');
        await page.keyboard.press('Escape');  // then closed
        assert.equal(await page.locator('.mech-menu').count(), 0);
        await page.keyboard.press('m');
        for (let k = 0; k < 3; ++k) {
            await page.keyboard.press('ArrowDown');
        }
        await page.keyboard.press('Enter');
        await page.keyboard.press('ArrowDown');  // Open claw
        await page.keyboard.press('Enter');
        let r = await route(page);
        assert.deepEqual(r.steps.map((s) => s.do), ['driveTo', 'set', 'wait']);
        assert.deepEqual(r.steps[1], { do: 'set', mechanism: 'claw', position: 'open' });
        assert.equal(await page.locator('.pane.right .insp-title').textContent(), 'Step 2', 'the new step is selected');
        // the cascade: one move, its home; "Wait for it" only once a step before starts it without waiting
        await page.locator('.field-svg').focus();
        await page.keyboard.press('m');
        await page.locator('.mech-menu [data-mech="cascade"]').click();
        assert.deepEqual(await menuWords(page, '.menu-item.big'), ['Move the cascade', 'Home it']);
        await page.locator('.mech-menu .menu-back').click();
        assert.equal(await page.locator('.mech-menu .mech-row').count(), 6, '"‹ Back" shows the list again');
        await page.locator('.mech-menu [data-mech="rightToggle"]').click();
        assert.deepEqual(await menuWords(page, '.menu-item.big'), ['Run it']);
        await page.locator('.mech-menu [data-act="Run it"]').click();
        await page.locator('.pane.right [data-waits="false"]').click();  // keep driving while it runs
        assert.equal((await route(page)).steps[2].do, 'startRun');
        assert.match(await page.locator('.pane.right').textContent(), /Nothing waits for it: if the route ends before it is done, the library stops it\./);
        await add(page, 'rightToggle', 'Wait for it');
        r = await route(page);
        assert.deepEqual(r.steps.map((s) => s.do), ['driveTo', 'set', 'startRun', 'waitUntilDone', 'wait']);
        await page.locator('.step[data-step="2"]').click();
        assert.match(await page.locator('.pane.right').textContent(), /The route waits for it at step 4\./);
        assert.deepEqual(await p.done(), []);
    });

    test('D1: Up from the first row is the last; a mechanism with no steps yet cannot be picked; ‹ Back keeps the place; a click outside closes it', async () => {
        const p = await started();
        const { page } = p;
        await useRobot(page, { ...fixtureRobot('x'), name: 'robot x (test)' });
        await page.locator('.field-svg').focus();
        await page.keyboard.press('m');
        assert.deepEqual(await menuWords(page, '.mech-row'), ['Arm', 'Lift', 'Flywheel', 'Intake', 'Hood']);
        assert.deepEqual(await page.locator('.mech-menu .mech-row').evaluateAll((els) => els.map((e) => e.disabled)), [false, false, true, false, false]);
        assert.equal(await focused(page), 'arm');
        await page.keyboard.press('ArrowUp');
        assert.equal(await focused(page), 'hood', 'Up from the first row: the last');
        await page.keyboard.press('ArrowDown');
        await page.keyboard.press('ArrowDown');
        await page.keyboard.press('ArrowDown');
        assert.equal(await focused(page), 'intake', 'the flywheel (no steps yet) is passed over');
        await page.locator('.mech-menu [data-mech="flywheel"]').click({ force: true });
        assert.equal(await page.locator('.mech-menu .mech-row').count(), 5, 'a click on it opens nothing');
        await page.locator('.mech-menu [data-mech="lift"]').click();
        await page.locator('.mech-menu .menu-back').click();
        assert.equal(await focused(page), 'lift', '‹ Back keeps the place, as Esc does');
        await page.locator('.pane.right .insp-title').click();
        assert.equal(await page.locator('.mech-menu').count(), 0, 'a click outside closes it');
        assert.deepEqual((await route(page)).steps, [], 'and adds nothing');
        assert.deepEqual(await p.done(), []);
    });

    test('D3: keep driving -- nothing waits: one click puts a wait at the end; a later step uses it again: one click puts the wait before THAT step', async () => {
        const p = await started();
        const { page } = p;
        await add(page, 'rightToggle', 'Run it');
        await page.locator('.pane.right [data-waits="false"]').click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('p');  // a pause after it: the route goes on
        await typeIn(page, 'seconds', 1);
        await page.locator('.step[data-step="0"]').click();
        assert.match(await page.locator('.pane.right').textContent(), /Nothing waits for it: if the route ends before it is done, the library stops it\./);
        await page.locator('.pane.right').getByRole('button', { name: 'Wait for it at the end' }).click();
        const ids = async () => (await route(page)).steps.map((s) => `${s.do} ${s.mechanism}`);
        assert.deepEqual(await ids(), ['startRun rightToggle', 'wait undefined', 'waitUntilDone rightToggle'], 'at the END, after the pause');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        assert.deepEqual(await ids(), ['startRun rightToggle', 'wait undefined']);
        await page.locator('.step[data-step="0"]').click();
        await add(page, 'rightToggle', 'Run it');  // after step 1: it runs the toggle again
        await page.locator('.step[data-step="0"]').click();
        assert.match(await page.locator('.pane.right').textContent(), /Step 2 uses the right toggle again before it may be done\./);
        await page.locator('.pane.right').getByRole('button', { name: 'Wait for it before step 2' }).click();
        assert.deepEqual(await ids(), ['startRun rightToggle', 'waitUntilDone rightToggle', 'run rightToggle', 'wait undefined'], 'the wait right before the step that needs it');
        assert.deepEqual(await p.done(), []);
    });

    test('D2: the bar marks only MEASURED levels; a click on a mark picks the LEVEL; the arrow is where it is before, the fill where it goes', async () => {
        const p = await started();
        const { page } = p;
        const r = ledgerRobot();  // the cascade 0-40 in: floor 0, L1 8, L2 16 (owed here)
        r.mechanisms[0].homeAt = 0;
        r.mechanisms[0].presets[2].owed = true;
        await useRobot(page, r);
        await add(page, 'cascade', 'Move the cascade');
        await page.locator('.pane.right .home-card').getByRole('button', { name: 'Home it at the start' }).click();
        await page.locator('.step[data-step="2"]').click();
        assert.equal(await page.locator('.pane.right .hbar-tick').count(), 2, 'floor and L1: an owed level has no mark (never a placeholder shown)');
        const tick = await page.locator('.pane.right .hbar-tick').nth(1).boundingBox();
        await page.mouse.click(tick.x + tick.width / 2, tick.y + 0.5);
        assert.deepEqual((await route(page)).steps[2], { do: 'move', mechanism: 'cascade', preset: 1 }, 'the level (it follows a re-measure), not 8 in');
        await typeIn(page, 'height', 12);
        assert.equal(await page.locator('.pane.right .hbar-now').evaluate((e) => e.style.bottom), '0%', 'before this step: homed, at the bottom');
        assert.equal(await page.locator('.pane.right .hbar-fill').evaluate((e) => e.style.height), '30%', 'where it goes: 12 of 40 in');
        assert.deepEqual(await p.done(), []);
    });

    test('D2 + D6: one height (the bar, a typed number, the bottom) writes today\'s step; the first height move with no home shows ONE card, and its click inserts today\'s fix', async () => {
        const p = await started();
        const { page } = p;
        await add(page, 'cascade', 'Move the cascade');
        assert.deepEqual((await route(page)).steps, [{ do: 'move', mechanism: 'cascade' }], 'no height and no time invented');
        assert.equal(await page.locator('.pane.right .home-card').count(), 1);
        assert.match(await page.locator('.pane.right .home-card').textContent(), /^The cascade must find its bottom first\.Home it at the start/);
        assert.equal(await page.locator('.pane.right .issue', { hasText: 'HOMED' }).count(), 0, 'the card stands for the home rule');
        // the bar: a click a quarter of the way up its 0-20 in travel is 5 in
        const bar = await page.locator('.pane.right [data-hbar]').boundingBox();
        await page.mouse.click(bar.x + bar.width / 2, bar.y + bar.height * 0.75);
        assert.deepEqual((await route(page)).steps[0], { do: 'move', mechanism: 'cascade', to: 5 });
        await typeIn(page, 'height', 18);
        assert.deepEqual((await route(page)).steps[0], { do: 'move', mechanism: 'cascade', to: 18 });
        assert.match(await page.locator('.pane.right [data-heights]').textContent(), /^not known → 18 in$/);
        await page.locator('.pane.right [data-level="bottom"]').click();
        assert.deepEqual((await route(page)).steps[0], { do: 'move', mechanism: 'cascade', bottom: true });
        await page.locator('.pane.right .home-card').getByRole('button', { name: 'Home it at the start' }).click();
        const r = await route(page);
        assert.deepEqual(r.steps, [{ do: 'startHome', mechanism: 'cascade' }, { do: 'waitUntilDone', mechanism: 'cascade' },
                                   { do: 'move', mechanism: 'cascade', bottom: true }], 'today\'s "home while the first leg drives"');
        // three badges at the start: past the robot, away from its knob -- a press on the robot grabs the robot
        const robot = await boxOf(page.locator('.g-start'));
        const knob = await boxOf(page.locator('.g-heading-knob'));
        for (const b of await page.locator('.g-mech rect').all()) {
            const box = await boxOf(b);
            assert.ok(!overlap(box, robot) && !overlap(box, knob), `a badge clear of the start robot and its knob: ${JSON.stringify(box)}`);
        }
        assert.equal(await page.evaluate(([x, y]) => !!document.elementFromPoint(x, y).closest('.g-start'), [(robot.l + robot.r) / 2, robot.t + (robot.b - robot.t) / 4]), true,
                     'the robot\'s upper half is the robot\'s');
        await page.locator('.step[data-step="2"]').click();
        assert.equal(await page.locator('.pane.right .home-card').count(), 0, 'homed now: no card');
        assert.match(await page.locator('.pane.right [data-heights]').textContent(), /^0 in → 0 in$/);
        // an old project's "up 6": shown, edited in its own field, and made a height in one click
        const old = { ...r, steps: [r.steps[0], r.steps[1], { do: 'move', mechanism: 'cascade', up: 6, boundS: 2 }] };
        const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
        await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json',
                            buffer: Buffer.from(JSON.stringify({ schemaVersion: 1, madeWith: 'an earlier route builder', robots: {}, exports: {}, routes: [{ ...old, together: false }] })) });
        await page.getByText('Opened the project.').waitFor();
        await page.locator('.step[data-step="2"]').click();
        assert.equal(await page.locator('.pane.right [data-heights]').textContent(), 'up 6 in → 6 in Make it a height');
        await typeIn(page, 'up', 7);
        assert.deepEqual((await route(page)).steps[2], { do: 'move', mechanism: 'cascade', up: 7, boundS: 2 });
        await page.locator('.pane.right [data-heights]').getByRole('button', { name: 'Make it a height' }).click();
        assert.deepEqual((await route(page)).steps[2], { do: 'move', mechanism: 'cascade', to: 7, boundS: 2 });
        assert.deepEqual(await p.done(), []);
    });

    test('D5: each mechanism step is a badge where it starts; a click selects it; hidden: gone; a started move has a band to its wait', async () => {
        const p = await started();
        const { page } = p;
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -36, -36);
        await typeIn(page, 'x', -36);
        await typeIn(page, 'y', -36);
        await add(page, 'claw', 'Close claw');
        await add(page, 'rightToggle', 'Run it');
        await page.locator('.pane.right [data-waits="false"]').click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -12, -36);
        await typeIn(page, 'x', -12);
        await typeIn(page, 'y', -36);
        await add(page, 'rightToggle', 'Wait for it');
        assert.deepEqual((await route(page)).steps.map((s) => s.do), ['driveTo', 'set', 'startRun', 'driveTo', 'waitUntilDone']);
        const badges = await page.locator('.g-mech').evaluateAll((els) => els.map((e) => {
            const b = e.querySelector('rect').getBoundingClientRect();
            return { i: Number(e.dataset.badge), x: b.x + b.width / 2, bottom: b.y + b.height, text: e.querySelector('text').textContent };
        }));
        assert.deepEqual(badges.map((b) => [b.i, b.text]), [[1, '2 close'], [2, '3 run'], [4, '5 wait']]);
        for (const [b, at] of [[badges[0], [-36, -36]], [badges[1], [-36, -36]], [badges[2], [-12, -36]]]) {
            const q = await toScreen(page, ...at);
            assert.ok(Math.abs(b.x - q.x) < 1, `step ${b.i + 1}'s badge over its start: ${b.x} vs ${q.x}`);
            assert.ok(b.bottom < q.y, `above the spot's number: ${b.bottom} vs ${q.y}`);
        }
        assert.ok(badges[0].bottom < badges[1].bottom - 10, 'two at one spot: stacked in step order, the first on top');
        assert.equal(await page.locator('.g-mech-band[data-band="2"]').count(), 1, 'the started run: a band to its wait');
        assert.match(await page.locator('.g-mech[data-badge="4"]').getAttribute('class'), /badge-error/, 'the wait\'s empty time limit: red, as its row');
        assert.doesNotMatch(await page.locator('.g-mech[data-badge="1"]').getAttribute('class'), /badge-(error|warn)/);
        // a drive's x emptied: the steps after it have no place, so no badge -- the field still draws
        await page.locator('.step[data-step="0"]').click();
        await typeIn(page, 'x', '');
        assert.equal((await route(page)).steps[0].x, undefined);
        assert.equal(await page.locator('.g-start').count(), 1, 'the start is still drawn');
        assert.deepEqual(await page.locator('.g-mech').evaluateAll((els) => els.map((e) => e.dataset.badge)), ['4'], 'the wait after the next full point keeps its place');
        await typeIn(page, 'x', -36);
        assert.equal(await page.locator('.g-mech').count(), 3);
        await page.locator('.g-mech[data-badge="1"]').click();
        assert.equal(await page.locator('.pane.right .insp-title').textContent(), 'Step 2');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('h');  // RMP1-G: hidden on the field
        assert.equal(await page.locator('.g-mech[data-badge="1"]').count(), 0);
        await page.keyboard.press('h');
        await page.keyboard.press('o');  // focus on step 2: step 5's badge fades and takes no click
        assert.match(await page.locator('.g-mech[data-badge="4"]').getAttribute('class'), /faded/);
        assert.doesNotMatch(await page.locator('.g-mech[data-badge="1"]').getAttribute('class'), /faded/);
        await page.keyboard.press('o');
        await page.keyboard.press('l');  // locked: a click on its badge passes through (it selects nothing)
        const b2 = await page.locator('.g-mech[data-badge="1"] rect').boundingBox();
        await page.locator('.step[data-step="0"]').click();
        await page.mouse.click(b2.x + b2.width / 2, b2.y + b2.height / 2);
        assert.notEqual(await page.locator('.pane.right .insp-title').textContent(), 'Step 2');
        // no page error (a badge once threw here and blanked the field); the browsers' NaN notes are the field's own
        // lines for the drive after the hole (field_view.mjs's drawing, older than RMP1-J; told to the coordinator)
        const nan = /^console (warning|error): .*(Unexpected value .*NaN|attribute \w+: Expected (length|number), ".*NaN)/;
        assert.deepEqual((await p.done()).filter((x) => !nan.test(x)), []);
    });

    test('D4: the time limit stays empty; PREVIEW (the real library) times it; "Use" writes PREVIEW\'s time + 0.5 s, only on the click -- a drive\'s into its own limit; a second PREVIEW replaces the first', async (t) => {
        const p = await started();
        const { page } = p;
        if (!(await page.evaluate(() => !document.getElementById('preview').disabled))) {
            t.skip('this site was built without the engine (build.py --engine)');
            await p.done();
            return;
        }
        const hint = () => page.locator('.pane.right .limit-hint');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -36, -36);
        await typeIn(page, 'x', -36);
        await typeIn(page, 'y', -36);
        await add(page, 'cascade', 'Move the cascade');
        await page.locator('.pane.right .home-card').getByRole('button', { name: 'Home it at the start' }).click();
        assert.deepEqual((await route(page)).steps.map((s) => s.do), ['startHome', 'driveTo', 'waitUntilDone', 'move']);
        assert.match(await hint().textContent(), /^Run PREVIEW to get a suggestion\.$/);
        assert.equal(await page.locator('.pane.right .issue', { hasText: 'Type the most time' }).count(), 0, 'an empty limit: its marked box, no issue box beside it');
        await page.locator('.step[data-step="2"]').click();
        await typeIn(page, 'boundS', 3);
        assert.equal(await hint().count(), 0, 'a typed limit: nothing to ask');
        await typeIn(page, 'boundS', '');  // emptied again: the card's wait is timed by PREVIEW too
        assert.match(await hint().textContent(), /^Run PREVIEW to get a suggestion\.$/);
        await page.locator('.step[data-step="3"]').click();
        await typeIn(page, 'height', 12);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open', null, { timeout: 30000 });
        assert.match(await page.locator('.toast').textContent(), /^PREVIEW ran 3 steps with no time limit yet/);
        const rec = await page.evaluate(() => window.routeBuilder.previewRecord());
        const mine = rec.steps.filter((s) => s.kind === 'mechanism' && s.what === 'cascade');
        assert.equal(mine.length, 2, 'the home and the move, from below');
        const drive = rec.steps.find((s) => s.kind === 'drive');
        await page.evaluate(() => window.routeBuilder.pause());
        for (const [i, lib] of [[3, mine[1]], [0, mine[0]]]) {
            await page.locator(`.step[data-step="${i}"]`).click();
            const took = lib.t1 - lib.t0;
            const use = page.locator('.pane.right .limit-hint [data-use]');
            assert.equal(Number(await use.getAttribute('data-use')), suggestS(took), `step ${i + 1}: the library's ${took} s + 0.5 s`);
            assert.ok(lib.placeholder, 'robot one\'s cascade is on PLACEHOLDER gains');
            assert.match(await hint().textContent(), /^PREVIEW took [\d.]+ s here on the host test plant \(on PLACEHOLDER numbers\), \+ 0\.5 s\. Use /);
            if (i === 3) {
                await shot(page, 'rmp1j-time-suggestion-light-1440', true);
            }
            assert.equal((await route(page)).steps[i].boundS, undefined, 'never written without the click');
            await use.click();
            assert.equal((await route(page)).steps[i].boundS, suggestS(took));
            assert.match(await hint().textContent(), /In use\.$/);
        }
        // the wait: its span, after the stand-in drive, on the cascade's PLACEHOLDER numbers -- all said in its line
        await page.locator('.step[data-step="2"]').click();
        const ws = rec.pageSteps[2];
        const wTook = rec.samples[ws.last][0] - rec.samples[ws.first][0];
        assert.match(await hint().textContent(),
                     /^PREVIEW waited [\d.]+ s here on the host test plant \(on PLACEHOLDER numbers, after the stand-in drive, not your robot\), \+ 0\.5 s: time it on the robot too\. Use /);
        assert.equal(Number(await page.locator('.pane.right .limit-hint [data-use]').getAttribute('data-use')), suggestS(wTook));
        await page.locator('.pane.right .limit-hint [data-use]').click();
        assert.equal((await route(page)).steps[2].boundS, suggestS(wTook));
        // the drive: PREVIEW's stand-in drive, said so; its "Use" fills ITS time limit, nothing else
        await page.locator('.step[data-step="1"]').click();
        const dTook = drive.t1 - drive.t0;
        assert.match(await hint().textContent(), /^PREVIEW's stand-in drive \(not your robot\) took [\d.]+ s here, \+ 0\.5 s: time it on the robot too\. Use /);
        assert.equal(Number(await page.locator('.pane.right .limit-hint [data-use]').getAttribute('data-use')), suggestS(dTook));
        const was = (await route(page)).steps[1];
        await page.locator('.pane.right .limit-hint [data-use]').click();
        assert.deepEqual((await route(page)).steps[1], { ...was, care: { ...(was.care || {}), timeoutS: suggestS(dTook) } });
        assert.deepEqual((await page.evaluate(() => window.routeBuilder.issues())).filter((x) => x.level === 'error'), []);
        // a second PREVIEW replaces the first: the move made 14 in -- the first run no longer speaks for it; the new one does
        await page.locator('.step[data-step="3"]').click();
        await typeIn(page, 'height', 14);
        assert.equal(await hint().count(), 0, 'the step changed (and its limit is typed): no line');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open' && !window.routeBuilder.preview().stale, null, { timeout: 30000 });
        const again = (await page.evaluate(() => window.routeBuilder.previewRecord())).steps.filter((s) => s.kind === 'mechanism' && s.what === 'cascade')[1];
        assert.notEqual(again.t1 - again.t0, mine[1].t1 - mine[1].t0, 'a different move, a different time');
        await page.evaluate(() => window.routeBuilder.pause());
        await page.locator('.step[data-step="3"]').click();
        assert.equal(Number(await hint().getAttribute('data-took')), again.t1 - again.t0, 'the newest run\'s time');
        assert.deepEqual((await p.done()).filter((x) => !allowedWarning(x.replace(/^console warning: /, ''))), []);  // the engine's one (Firefox)
    });

    for (const v of VARIANTS) {
        test(`RMP1-J screens [${v.tag}]: the menu's two steps, the height bar, a route with three badges`, async () => {
            const p = await started(v);
            const { page } = p;
            await page.locator('.field-svg').focus();
            await page.keyboard.press('d');
            await clickField(page, -36, -36);
            await typeIn(page, 'x', -36);
            await typeIn(page, 'y', -36);
            await add(page, 'cascade', 'Move the cascade');
            await shot(page, `rmp1j-home-card-${v.tag}`, true);
            await page.locator('.pane.right .home-card').getByRole('button', { name: 'Home it at the start' }).click();
            for (const k of [0, 2, 3]) {  // the time limits typed: a route with no red step
                await page.locator(`.step[data-step="${k}"]`).click();
                await typeIn(page, 'boundS', 2);
            }
            await typeIn(page, 'height', 12);
            await add(page, 'claw', 'Close claw');
            await page.locator('.step[data-step="3"]').click();
            await shot(page, `rmp1j-height-bar-${v.tag}`, true);
            await page.locator('.field-svg').focus();
            await page.keyboard.press('m');
            await page.locator('.mech-menu').waitFor();
            await shot(page, `rmp1j-menu-mechanisms-${v.tag}`);
            await page.locator('.mech-menu [data-mech="claw"]').click();
            await shot(page, `rmp1j-menu-actions-${v.tag}`);
            await page.keyboard.press('Escape');
            await page.keyboard.press('Escape');
            await page.locator('.start-card').click();
            await shot(page, `rmp1j-badges-${v.tag}`, true);
            assert.ok(await page.locator('.g-mech').count() >= 3);
            assert.deepEqual(await p.done(), []);
        });
    }
});
