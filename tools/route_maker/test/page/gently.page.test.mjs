// gently.page.test.mjs — RMP1-P in a browser: "Leave gently" is a Back up's first control, on for a Back up added
// right after the claw opens (its sub-line, its two numbers, the card's "gently" tag), off after a drive or the
// flipper; one click turns it off. ROUTE_MAKER_SHOTS: the inspector with it on and the card's tag, light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, SHOTS, skip, launch, serve, openPage, shot } from './harness.mjs';
import { allowedWarning } from './allowed_warnings.mjs';

const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());
async function clickField(page, x, y) {
    const at = await page.evaluate(([fx, fy]) => {
        const pic = JSON.parse(document.getElementById('shulib-field').textContent);
        const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(document.querySelector('.field-svg').getScreenCTM());
        return { x: pt.x, y: pt.y };
    }, [x, y]);
    await page.mouse.click(at.x, at.y);
}
async function typeIn(page, name, value) {
    await page.locator(`.pane.right input[name="${name}"]`).fill(String(value));
    await page.locator(`.pane.right input[name="${name}"]`).press('Enter');
}
async function key(page, k) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press(k);
}
/** Robot one's start, a drive ahead, the claw opened (step 2, selected). */
async function dropped(v) {
    const p = await openPage(browser, site.base, v);
    const { page } = p;
    await page.getByRole('button', { name: 'Got it' }).click();
    await clickField(page, -36, -55.7);
    await typeIn(page, 'startX', -36);
    await typeIn(page, 'startY', -55.7);
    await typeIn(page, 'startH', 90);
    await page.locator('.pane.right [data-together="false"]').click();
    await key(page, 'd');
    await clickField(page, -36, -36);
    await key(page, 'm');
    await page.locator('.mech-menu [data-mech="claw"]').click();
    await page.locator('.mech-menu [data-act="Open claw"]').click();
    return p;
}
/** ...then a Back up clicked behind and to the side (turned first on a tank): step 3. */
async function dropThenBackUp(v) {
    const p = await dropped(v);
    await key(p.page, 'b');
    await clickField(p.page, -46, -46);
    return p;
}
/** ...then Straight, 4 in, switched to Backwards: a straight back-out, step 3 (the guide's picture). */
async function dropThenStraightBack(v) {
    const p = await dropped(v);
    await key(p.page, 'f');
    await typeIn(p.page, 'inches', 4);
    await p.page.locator('.pane.right input[name="backwards"]').click();
    return p;
}
const toggle = (page) => page.locator('.pane.right input[name="leaveGently"]');
const sub = (page) => page.locator('.pane.right .gently .switch small').textContent();
const tag = (page, i) => page.locator(`.step[data-step="${i}"] .step-sub .gently`);

test.describe('RMP1-P in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    for (const v of VARIANTS) {
        test(`D1, D2: a straight Back up after the claw opens starts "Leave gently" ON -- the first control, its sub-line, its numbers, the tag (${v.tag})`, async () => {
            const p = await dropThenStraightBack(v);
            const { page } = p;
            let r = await route(page);
            assert.deepEqual(r.steps.map((s) => s.do), ['driveTo', 'set', 'backward']);
            assert.deepEqual(r.steps[2], { do: 'backward', inches: 4, gentlyWaitS: 0.5, gentlySpeedInPerS: 5 }, 'its own fields: care untouched');
            assert.equal(await page.locator('.pane.right .insp-title').textContent(), 'Step 3');
            assert.equal(await page.locator('.pane.right input, .pane.right select').first().getAttribute('name'), 'leaveGently', 'the FIRST control');
            assert.ok(await toggle(page).isChecked());
            assert.equal(await sub(page), 'waits ½ s, then backs out slowly (5 in/s)');
            assert.equal(await page.locator('.pane.right input[name="gentlyWait"]').inputValue(), '0.5');
            assert.equal(await page.locator('.pane.right input[name="gentlySpeed"]').inputValue(), '5');
            assert.equal(await tag(page, 2).textContent(), 'gently');
            assert.equal(await tag(page, 0).count(), 0);
            await shot(page, `gently-on-${v.tag}`, true);
            await page.locator('.step[data-step="1"]').click();  // the card unselected: its tag in full
            await shot(page, `gently-card-${v.tag}`);
            await page.locator('.step[data-step="2"]').click();
            // the numbers, edited: the sub-line says them
            await typeIn(page, 'gentlyWait', 1);
            await typeIn(page, 'gentlySpeed', 8);
            r = await route(page);
            assert.deepEqual([r.steps[2].gentlyWaitS, r.steps[2].gentlySpeedInPerS, r.steps[2].care], [1, 8, undefined]);
            assert.equal(await sub(page), 'waits 1 s, then backs out slowly (8 in/s)');
            // outside the bounds: refused in words
            await typeIn(page, 'gentlyWait', 3);
            assert.match(await page.locator('.pane.right .issue.error').first().textContent(), /Leave gently waits more than 0 s and at most 2 s\./);
            // emptied: still on, a hole the check asks for -- never turned off with its speed left behind
            await typeIn(page, 'gentlyWait', '');
            r = await route(page);
            assert.equal(r.steps[2].gentlyWaitS, null);
            assert.ok(await toggle(page).isChecked());
            assert.match(await page.locator('.pane.right .issue.error').first().textContent(), /at most 2 s: type it\./);
            await typeIn(page, 'gentlyWait', 0.5);
            await typeIn(page, 'gentlySpeed', 25);
            assert.match(await page.locator('.pane.right .issue.error').first().textContent(), /Leave gently backs out at 2 to 20 in\/s\./);
            // too slow for its time limit: 10 in at 2 in/s
            await typeIn(page, 'gentlySpeed', 2);
            await typeIn(page, 'inches', 10);
            assert.match(await page.locator('.pane.right .issue.error').first().textContent(), /At 2 in\/s this 10 in back-up needs about 5\.4 s, over its 5 s time limit/);
            await typeIn(page, 'gentlySpeed', 5);
            assert.equal(await page.locator('.pane.right .issue.error').count(), 0);
            // one click turns it off: no wait, no cap, no tag -- and it stays off
            await toggle(page).click();
            r = await route(page);
            assert.deepEqual(r.steps[2], { do: 'backward', inches: 10 });
            assert.equal(await tag(page, 2).count(), 0);
            assert.equal(await page.locator('.pane.right input[name="gentlyWait"]').count(), 0);
            assert.equal(await sub(page), 'waits ½ s, then backs out slowly (5 in/s)');
            await typeIn(page, 'inches', 5);  // any other edit
            assert.equal((await route(page)).steps[2].gentlyWaitS, undefined, 'an edit never turns it back on');
            assert.deepEqual(await p.done(), []);
        });
    }

    test('D3: PREVIEW (the real library) runs the wait, then the back-up at most 5 in/s, as ONE page step -- its brain numbers, its time', async (t) => {
        const p = await dropThenBackUp();
        const { page } = p;
        if (!(await page.evaluate(() => !document.getElementById('preview').disabled))) {
            await p.done();
            t.skip('this site was built without the engine (build.py --engine)');
            return;
        }
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open', null, { timeout: 30000 });
        const rec = await page.evaluate(() => window.routeBuilder.previewRecord());
        const ps = rec.pageSteps[2];
        assert.equal(ps.verdict, 'ok');
        assert.deepEqual(ps.lib, [3, 5], 'the wait, the turn, the back-up');
        assert.ok(rec.lines.some((l) => /\(wait 0\.5 s\)/.test(l.text)), 'the wait is a library step, named as the export names it');
        const run = rec.samples.slice(ps.first, ps.last + 1);
        assert.ok(run.at(-1)[0] - run[0][0] >= 0.5 + 0.5, 'the wait, then the back-up');
        const fastest = Math.max(...run.slice(1).map((q, k) => Math.hypot(q[1] - run[k][1], q[2] - run[k][2]) / (q[0] - run[k][0])));
        assert.ok(fastest <= 5 * 1.1 && fastest >= 5 * 0.5, `backed out at most 5 in/s (fastest ${fastest.toFixed(2)})`);
        assert.deepEqual((await p.done()).filter((x) => !allowedWarning(x.replace(/^console warning: /, ''))), []);  // the engine's one (Firefox)
    });

    test('D2: a Back up clicked after the claw opens turns first (said so); after a DRIVE or the FLIPPER it starts OFF; Straight switched to Backwards turns ON', async () => {
        const p = await dropThenBackUp();
        const { page } = p;
        let r = await route(page);
        assert.deepEqual([r.steps[2].do, r.steps[2].faceFirst, r.steps[2].gentlyWaitS], ['reverseTo', true, 0.5]);
        assert.equal(await sub(page), 'waits ½ s, turns at its own speed, then backs out slowly (5 in/s)');
        await key(page, 'b');
        await clickField(page, -58, -30);
        r = await route(page);
        assert.deepEqual(r.steps.map((s) => s.do), ['driveTo', 'set', 'reverseTo', 'reverseTo']);
        assert.equal(r.steps[3].gentlyWaitS, undefined);
        assert.ok(!(await toggle(page).isChecked()));
        assert.equal(await tag(page, 3).count(), 0);
        // F after the claw opens: a straight forward, then Backwards
        await page.locator('.step[data-step="1"]').click();
        await key(page, 'f');
        await typeIn(page, 'inches', 4);
        await page.locator('.pane.right input[name="backwards"]').click();
        r = await route(page);
        assert.equal(r.steps[2].do, 'backward');
        assert.equal(r.steps[2].gentlyWaitS, 0.5);
        assert.equal(await tag(page, 2).textContent(), 'gently');
        assert.ok(await toggle(page).isChecked());
        // Backwards off: a drive forward never waits first
        await page.locator('.pane.right input[name="backwards"]').click();
        r = await route(page);
        assert.deepEqual(r.steps[2], { do: 'forward', inches: 4 });
        assert.equal(await toggle(page).count(), 0);
        // the front toggle (the flipper) pushed out: no drop -- its description marks no grab
        await page.locator('.step[data-step="4"]').click();
        await key(page, 'm');
        await page.locator('.mech-menu [data-mech="flipper"]').click();
        await page.locator('.mech-menu [data-act="Open flipper"]').click();
        await key(page, 'b');
        await clickField(page, -58, -20);
        r = await route(page);
        assert.deepEqual(r.steps[5], { do: 'set', mechanism: 'flipper', position: 'open' });
        assert.equal(r.steps[6].do, 'reverseTo');
        assert.equal(r.steps[6].gentlyWaitS, undefined);
        assert.ok(!(await toggle(page).isChecked()));
        assert.deepEqual(await p.done(), []);
    });
});
