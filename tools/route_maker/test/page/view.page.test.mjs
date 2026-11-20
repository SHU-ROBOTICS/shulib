// view.page.test.mjs — RMP1-G in a browser: hide (D1), lock (D2), focus (D3), click-through on a crowded spot (D4),
// sections (D5), undo / redo (D6). Every route here is the test's own (G4). ROUTE_MAKER_SHOTS: a crowded route before
// and with focus on, light and dark, at 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, SHOTS, skip, launch, serve, openPage, shot } from './harness.mjs';

let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());
const selected = async (page) => {
    const t = await page.locator('.pane.right .insp-title').textContent();
    return /^Step (\d+)$/.test(t.trim()) ? Number(t.trim().slice(5)) : t.trim();
};
const here = (page) => page.evaluate(() => document.querySelector('.pane.right .view-line.here')?.textContent || '');

/** The screen point of field point (x, y). */
const at = (page, x, y) => page.evaluate(([fx, fy]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn)
        .matrixTransform(document.querySelector('.field-svg').getScreenCTM());
    return { x: pt.x, y: pt.y };
}, [x, y]);
async function clickField(page, x, y) {
    const p = await at(page, x, y);
    await page.mouse.click(p.x, p.y);
}
async function dragField(page, [x0, y0], [dx, dy]) {
    const p = await at(page, x0, y0);
    await page.mouse.move(p.x, p.y);
    await page.mouse.down();
    await page.mouse.move(p.x + dx / 2, p.y + dy / 2, { steps: 4 });
    await page.mouse.move(p.x + dx, p.y + dy, { steps: 4 });
    await page.mouse.up();
}
const key = async (page, k) => {
    await page.locator('.field-svg').focus();
    await page.keyboard.press(k);
};
const pickStep = (page, n) => page.locator(`.step[data-step="${n - 1}"]`).click();
const xy = (s) => [s.x, s.y];
const pills = (page) => page.evaluate(() => [...document.querySelectorAll('.g-badge-text')].map((t) => [t.textContent, t.parentNode.classList.contains('faded')]));
const pillsLit = async (page) => (await pills(page)).filter(([, f]) => !f).map(([t]) => t).sort();
const routesMenu = async (page) => {
    await page.locator('#route-switch').click();
    return page.locator('.popover[aria-label="Routes"]');
};
const FIVE = [[-12, -55.7], [-12, -36], [0, -36], [12, -36], [12, -12]];

/** A page with `steps` (drives to the test's points [x, y, back?], or a step object as it is) open from an in-memory folder. */
async function withRoute(steps, opts) {
    const p = await openPage(browser, site.base, opts);
    await p.page.getByRole('button', { name: 'Got it' }).click();
    const project = { schemaVersion: 1, madeWith: 'test', routes: [{ name: 'skills', kind: 'skills', robot: 'one', mirroredFor: 'two', together: false,
        start: { x: -36, y: -55.7, headingDeg: 0 }, endActionAtS: 57, hardStopAtS: 59,
        steps: steps.map((s) => (Array.isArray(s) ? { do: s[2] ? 'reverseTo' : 'driveTo', x: s[0], y: s[1], faceFirst: true } : s)) }], robots: {}, exports: {} };
    await p.page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
    assert.equal((await route(p.page)).steps.length, steps.length);
    return p;
}

const A = [-12, -36];
const B = [12, -36];

test.describe('RMP1-G in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('D1 + D4: a hidden step is not drawn and a click there takes the step under it; Show all; H undone and redone', async () => {
        const p = await withRoute([A, B, A]);  // steps 1 and 3 on one spot, 3 on top
        const { page } = p;
        await clickField(page, ...A);
        assert.equal(await selected(page), 3, 'the top one first');
        assert.match(await here(page), /^1 of 2 here/);
        await pickStep(page, 3);
        await key(page, 'h');
        assert.deepEqual((await route(page)).steps[2].view, { hidden: true });
        assert.equal(await page.locator('.g-step[data-step="2"]').count(), 0, 'not drawn');
        assert.equal(await page.locator('.g-handle[data-index="2"]').count(), 0);
        assert.match(await page.locator('.step[data-step="2"]').textContent(), /Hidden on the field/);
        assert.match(await page.locator('.view-bar').textContent(), /1 step hidden on the field/);
        assert.ok(!(await page.locator('.g-badge-text').allTextContents()).some((t) => /3/.test(t)), 'its number is not on the field either');
        await clickField(page, 50, 50);  // nothing selected
        await clickField(page, ...A);
        assert.equal(await selected(page), 1, 'the click reaches the step under it');
        assert.equal(await here(page), '', 'alone there now');
        await key(page, 'Control+z');
        assert.equal((await route(page)).steps[2].view, undefined, 'undo shows it');
        await key(page, 'Control+Shift+z');
        assert.deepEqual((await route(page)).steps[2].view, { hidden: true }, 'redo hides it');
        await key(page, 'l');  // step 1 (selected): locked
        await page.locator('.view-bar').getByRole('button', { name: 'Show all' }).click();
        assert.equal((await route(page)).steps[2].view, undefined);
        assert.deepEqual((await route(page)).steps[0].view, { locked: true }, 'Show all shows; it unlocks nothing');
        assert.equal(await page.locator('.view-bar').count(), 0);
        assert.deepEqual(await p.done(), []);
    });

    test('D2: a locked step does not move on a drag or a nudge, a click passes through it, its numbers can still be typed', async () => {
        const p = await withRoute([A, B, A]);
        const { page } = p;
        await pickStep(page, 3);
        await key(page, 'l');
        assert.deepEqual((await route(page)).steps[2].view, { locked: true });
        assert.match(await page.locator('.pane.right .view-line.locked').textContent(), /Locked on the field/);
        await key(page, 'ArrowUp');
        assert.deepEqual([(await route(page)).steps[2].x, (await route(page)).steps[2].y], A, 'no nudge');
        assert.match(await page.locator('.toast').textContent(), /Step 3 is locked: unlock it \(L\) to move it/);
        await clickField(page, ...A);
        assert.equal(await selected(page), 1, 'the click passes through to step 1');
        await dragField(page, A, [60, 0]);
        let r = await route(page);
        assert.deepEqual([r.steps[2].x, r.steps[2].y], A, 'the locked step stays');
        assert.notDeepEqual([r.steps[0].x, r.steps[0].y], A, 'the drag took the step under it');
        // alone on its spot: a drag there moves nothing (a pan)
        await pickStep(page, 2);
        await key(page, 'l');
        await dragField(page, B, [0, 60]);
        r = await route(page);
        assert.deepEqual([r.steps[1].x, r.steps[1].y], B);
        await pickStep(page, 3);
        await page.locator('.pane.right input[name="x"]').fill('-10');
        await page.locator('.pane.right input[name="x"]').press('Enter');
        assert.equal((await route(page)).steps[2].x, -10, 'typed numbers still go in');
        await key(page, 'l');
        assert.equal((await route(page)).steps[2].view, undefined, 'L again unlocks');
        assert.deepEqual(await p.done(), []);
    });

    test('D3: focus draws the selected step and its neighbours full, fades the rest out of reach, follows the selection; Esc leaves', async () => {
        const p = await withRoute([[-12, -55.7], [-12, -36], [0, -36], [12, -36], [12, -12]]);
        const { page } = p;
        const faded = () => page.evaluate(() => [...document.querySelectorAll('.g-step.faded')].map((g) => Number(g.dataset.step)));
        await key(page, 'o');
        assert.match(await page.locator('.toast').textContent(), /Select a step first/);
        assert.deepEqual(await faded(), []);
        await pickStep(page, 3);
        await key(page, 'o');
        assert.deepEqual(await faded(), [0, 4]);
        assert.equal(await page.locator('.g-start.faded').count(), 1, 'the start is not step 3\'s neighbour');
        assert.ok(Number(await page.locator('.g-step[data-step="4"]').evaluate((g) => getComputedStyle(g).opacity)) < 0.5, 'drawn faded');
        assert.match(await page.locator('.mode-hint').textContent(), /Focus: step 3 and its neighbours/);
        assert.equal(await page.locator('[data-tool="focus"]').getAttribute('aria-pressed'), 'true');
        assert.equal(await page.locator('.field-svg.focusing').count(), 1);
        assert.ok(Number(await page.locator('.g-twin').first().evaluate((g) => getComputedStyle(g).opacity)) < 0.5, 'robot two faded too');
        assert.deepEqual(await pillsLit(page), ['2', '3', '4']);
        assert.deepEqual((await pills(page)).filter(([, f]) => f).map(([t]) => t).sort(), ['1', '5'], 'only the numbers out of focus fade');
        const knob = await page.locator('.g-heading-knob').evaluate((c) => { const b = c.getBoundingClientRect(); return { x: b.x + b.width / 2, y: b.y + b.height / 2 }; });
        await page.mouse.move(knob.x, knob.y);
        await page.mouse.down();
        await page.mouse.move(knob.x + 40, knob.y + 80, { steps: 6 });
        await page.mouse.up();
        assert.equal((await route(page)).start.headingDeg, 0, 'the faded start\'s heading knob does not turn');
        await clickField(page, 12, -12);  // step 5's point: faded, not clickable
        assert.notEqual(await selected(page), 5);
        await clickField(page, 12, -36);  // step 4's: a neighbour
        assert.equal(await selected(page), 4);
        assert.deepEqual(await faded(), [0, 1], 'focus follows the selection');
        await key(page, 'Escape');
        assert.deepEqual(await faded(), []);
        assert.equal(await page.locator('.field-svg.focusing').count(), 0);
        await pickStep(page, 1);
        await page.locator('[data-tool="focus"]').click();
        assert.deepEqual(await faded(), [2, 3, 4], 'step 1: the start and step 2 are its neighbours');
        assert.equal(await page.locator('.g-start.faded').count(), 0);
        await page.locator('[data-tool="focus"]').click();
        assert.deepEqual(await faded(), []);
        assert.deepEqual(await p.done(), []);
    });

    test('D4: three points on one spot cycle on clicks ("k of 3"), the hover names them, a drag takes the one picked', async () => {
        const p = await withRoute([A, B, A, B, A]);  // steps 1, 3 and 5 on A
        const { page } = p;
        const pa = await at(page, ...A);
        await page.mouse.move(pa.x, pa.y);
        assert.match(await page.locator('.g-tip').textContent(), /^Step 5: .*\(3 here: click again for the next\)$/);
        assert.ok(await page.locator('.g-tip').isVisible());
        const seen = [];
        for (let k = 0; k < 4; ++k) {
            await page.mouse.click(pa.x, pa.y);
            seen.push([await selected(page), (await here(page)).slice(0, 6)]);
        }
        assert.deepEqual(seen, [[5, '1 of 3'], [3, '2 of 3'], [1, '3 of 3'], [5, '1 of 3']]);
        await page.mouse.click(pa.x, pa.y);  // step 3 again
        assert.equal(await selected(page), 3);
        await dragField(page, A, [0, -48]);
        const r = await route(page);
        assert.deepEqual([[r.steps[0].x, r.steps[0].y], [r.steps[4].x, r.steps[4].y]], [A, A], 'steps 1 and 5 stay');
        assert.notDeepEqual([r.steps[2].x, r.steps[2].y], A, 'step 3 moved');
        assert.deepEqual(await p.done(), []);
    });

    test('D4: a press takes the step highlighted when it is on the spot, else the top one; hiding there clears "k of n"', async () => {
        let p = await withRoute([A, B, A, B, A]);
        let { page } = p;
        const pa = await at(page, ...A);
        for (let k = 0; k < 3; ++k) {
            await page.mouse.click(pa.x, pa.y);
        }
        assert.deepEqual([await selected(page), (await here(page)).slice(0, 6)], [1, '3 of 3']);
        await key(page, 'h');
        assert.equal(await here(page), '', 'two left there now: the line goes');
        await dragField(page, A, [0, -48]);
        let r = await route(page);
        assert.notDeepEqual(xy(r.steps[4]), A, 'step 5, the top one shown, moved');
        assert.deepEqual([xy(r.steps[0]), xy(r.steps[2])], [A, A]);
        assert.deepEqual(await p.done(), []);
        p = await withRoute([A, B, A]);
        ({ page } = p);
        await clickField(page, ...A);
        await clickField(page, ...A);
        assert.equal(await selected(page), 1);
        const pa3 = await at(page, ...A);
        await page.mouse.move(0, 0);
        await page.mouse.move(pa3.x, pa3.y, { steps: 3 });
        assert.match(await page.locator('.g-tip').textContent(), /^Step 1: .*\(selected · 2 here: click for the next\)$/,
                     'the tip names the one a drag moves (the selected, under step 3), not the top one');
        await pickStep(page, 3);  // the top one, from the list
        await dragField(page, A, [60, 0]);
        r = await route(page);
        assert.deepEqual([xy(r.steps[0]), await selected(page)], [A, 3], 'the highlighted step 3 moved, not step 1');
        assert.notDeepEqual(xy(r.steps[2]), A);
        await key(page, 'Control+z');
        await clickField(page, ...A);
        await clickField(page, ...A);
        await clickField(page, 50, 50);  // nothing selected
        await dragField(page, A, [60, 0]);
        r = await route(page);
        assert.deepEqual(xy(r.steps[0]), A, 'nothing highlighted: the top one, step 3, moved');
        assert.notDeepEqual(xy(r.steps[2]), A);
        assert.deepEqual(await p.done(), []);
        p = await withRoute([[-36, -55.7], B]);  // step 1 ends on the start
        ({ page } = p);
        await clickField(page, -36, -55.7);
        assert.match(await here(page), /^The start: 1 of 2 here/);
        await clickField(page, -36, -55.7);
        assert.deepEqual([await selected(page), (await here(page)).slice(0, 6)], [1, '2 of 2']);
        assert.deepEqual(await p.done(), []);
    });

    test('D1 + D2 from the row and the inspector: the eye and the lock (a click, Enter), a hidden step\'s nudge, Unlock, Shift+H', async () => {
        const p = await withRoute([A, B, [0, -12], { do: 'wait', seconds: 1 }]);
        const { page } = p;
        await pickStep(page, 1);
        await page.locator('.step[data-step="1"]').hover();
        await page.locator('.step[data-step="1"] .step-act[aria-label="Hide step 2 on the field (H)"]').click();
        assert.deepEqual((await route(page)).steps[1].view, { hidden: true });
        assert.equal(await selected(page), 1, 'the eye does not select its row');
        await page.locator('.step[data-step="2"]').focus();
        await page.keyboard.press('Tab');
        await page.keyboard.press('Tab');
        assert.equal(await page.evaluate(() => document.activeElement.getAttribute('aria-label')), 'Lock step 3 (L)');
        await page.keyboard.press('Enter');
        assert.deepEqual((await route(page)).steps[2].view, { locked: true }, 'Enter on the lock locks');
        await pickStep(page, 2);
        const off = await at(page, 60, 60);
        await page.mouse.move(off.x, off.y);  // on the field: no row hovered
        assert.ok(await page.locator('.step[data-step="1"] .step-acts').isVisible(), 'the selected row shows its eye and lock');
        assert.ok(!(await page.locator('.step[data-step="0"] .step-acts').isVisible()), 'the others do not');
        const box = (sel) => page.locator(`.step[data-step="1"] ${sel}`).boundingBox();
        const [acts, words] = [await box('.step-acts'), await box('.step-main')];
        assert.ok(words.x + words.width <= acts.x, `the buttons sit beside its words, not on them (${JSON.stringify([words, acts])})`);
        assert.ok((await box('.grip')).x >= acts.x + acts.width, 'and the grip stays on the row, after them');
        await key(page, 'ArrowUp');
        assert.deepEqual(xy((await route(page)).steps[1]), B, 'a hidden step does not move');
        assert.equal(await page.locator('.toast', { hasText: 'Step 2 is hidden: show it (H) to move it' }).count(), 1);
        assert.match(await page.locator('.pane.right .view-line.hidden').textContent(), /Hidden on the field/);
        await key(page, 'Shift+L');
        await key(page, 'Shift+H');
        assert.deepEqual((await route(page)).steps[1].view, { hidden: true }, 'Shift+H and Shift+L are not H and L');
        await key(page, 'l');
        await page.locator('.pane.right .view-line.locked').getByRole('button', { name: 'Unlock (L)' }).click();
        assert.deepEqual((await route(page)).steps[1].view, { hidden: true }, 'Unlock unlocks');
        await pickStep(page, 4);  // a Wait: nothing to nudge
        await key(page, 'l');
        await key(page, 'ArrowUp');
        assert.equal(await page.locator('.toast', { hasText: 'Step 4 is locked: unlock it' }).count(), 0, 'no "move it" for a step with no point');
        assert.deepEqual(await p.done(), []);
    });

    test('D3: focus ends when the route on the field changes (a new route, a switch, an undo back); a shared spot\'s pill lists only the steps in focus; Esc in a curve cancels the curve first', async () => {
        const bad = { do: 'driveTo', x: A[0], y: A[1], faceFirst: true, care: { onFail: { skipTo: 'nowhere' } } };  // an error, on A
        const p = await withRoute([A, B, bad, [12, -12]]);  // steps 1 and 3 on A
        const { page } = p;
        const pressed = () => page.locator('[data-tool="focus"]').getAttribute('aria-pressed');
        const red = (t) => page.evaluate((x) => [...document.querySelectorAll('.g-badge-text')].filter((e) => e.textContent === x)
            .map((e) => e.parentNode.classList.contains('badge-error')), t);
        await pickStep(page, 1);
        assert.deepEqual(await red('1, 3'), [true], 'no focus: A\'s pill is red, from step 3');
        await key(page, 'o');
        assert.deepEqual(await pillsLit(page), ['1', '2'], 'A\'s pill says 1, not "1, 3": step 3 is out of focus');
        assert.deepEqual(await red('1'), [false], 'and its colour is step 1\'s, not the faded step 3\'s red');
        await pickStep(page, 3);
        assert.deepEqual(await pillsLit(page), ['2', '3', '4']);
        assert.deepEqual(await red('3'), [true], 'step 3 in focus: red');
        await page.locator('[data-tool="curveTo"]').click();
        await clickField(page, 24, 0);
        await clickField(page, 30, 12);
        await key(page, 'Escape');
        assert.equal(await page.locator('[data-tool="curveTo"]').getAttribute('aria-pressed'), 'false', 'the curve is cancelled');
        assert.equal(await pressed(), 'true', 'focus stays');
        let menu = await routesMenu(page);
        await menu.getByRole('button', { name: 'New route' }).click();
        await page.locator('.dialog input.txt').fill('probe');
        await page.getByRole('button', { name: 'Add the route' }).click();
        assert.deepEqual([(await route(page)).name, await pressed(), await page.locator('.field-svg.focusing').count()], ['probe', 'false', 0]);
        await clickField(page, 0, 0);
        await key(page, 'd');
        await clickField(page, 24, 0);
        await pickStep(page, 1);
        await key(page, 'o');
        assert.equal(await pressed(), 'true');
        menu = await routesMenu(page);
        await menu.locator('.route-pick', { hasText: 'skills' }).click();
        assert.deepEqual([(await route(page)).name, await pressed()], ['skills', 'false'], 'a switch ends it');
        await pickStep(page, 2);
        await key(page, 'o');
        menu = await routesMenu(page);
        await menu.locator('.route-pick', { hasText: 'probe' }).click();
        await pickStep(page, 1);
        await key(page, 'o');
        for (let k = 0; k < 6 && (await route(page)).name !== 'skills'; ++k) {
            await key(page, 'Control+z');
        }
        assert.deepEqual([(await route(page)).name, await pressed(), await page.locator('.g-start.faded').count()], ['skills', 'false', 0], 'an undo back to another route ends it');
        await pickStep(page, 2);
        await key(page, 'o');
        const other = { schemaVersion: 1, madeWith: 'test', routes: [{ name: 'skills', kind: 'skills', robot: 'one', start: { x: -36, y: -55.7, headingDeg: 0 },
            endActionAtS: 57, hardStopAtS: 59, steps: [{ do: 'driveTo', x: 24, y: 24, faceFirst: true }] }], robots: {}, exports: {} };
        const opening = page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(other, null, 2)}\n`);
        await page.getByRole('button', { name: 'Open the folder\'s route' }).click({ timeout: 10000 });  // it asks first: the route differs
        await opening;
        assert.deepEqual([(await route(page)).steps.length, await pressed()], [1, 'false'], 'opening a project ends it');
        assert.deepEqual(await p.done(), []);
    });

    test('D5 from the menus: hide and focus from a step\'s ..., rename a section; a folded section still lists the selected step', async () => {
        const p = await withRoute(FIVE);
        const { page } = p;
        const menuItem = async (n, words) => {
            await page.locator(`.step[data-step="${n - 1}"]`).hover();
            await page.locator(`.step[data-step="${n - 1}"] .step-act[aria-label="More for step ${n}"]`).click();
            await page.locator(`.popover[aria-label="Step ${n}"] .menu-item`, { hasText: words }).click();
        };
        await pickStep(page, 2);
        await menuItem(2, 'Hide on the field');
        assert.deepEqual((await route(page)).steps[1].view, { hidden: true });
        await menuItem(3, 'Focus on this step');
        assert.deepEqual([await selected(page), await page.locator('[data-tool="focus"]').getAttribute('aria-pressed')], [3, 'true']);
        await key(page, 'Escape');
        await menuItem(3, 'Start a section here');
        await page.locator('.dialog input.txt').fill('Claw 1');
        await page.getByRole('button', { name: 'Start the section' }).click();
        const row = page.locator('.section-row[data-section="Claw 1"]');
        await row.getByRole('button', { name: 'More for "Claw 1"' }).click();
        await page.locator('.popover[aria-label="Section"] .menu-item', { hasText: 'Rename the section' }).click();
        await page.locator('.dialog input.txt').fill('Claw 2');
        await page.locator('.dialog').getByRole('button', { name: 'Rename' }).click();
        assert.equal((await route(page)).steps[2].view.section, 'Claw 2');
        await pickStep(page, 4);
        await page.locator('.section-row[data-section="Claw 2"]').getByRole('button', { name: 'Fold "Claw 2"' }).click();
        assert.deepEqual(await page.evaluate(() => [...document.querySelectorAll('.step[data-step]')].map((c) => Number(c.dataset.step) + 1)), [1, 2, 4],
                         'folded: only the selected step 4 still listed');
        assert.deepEqual(await p.done(), []);
    });

    test('D5: a section from a step\'s menu hides and locks its steps together, folds, and every change undoes', async () => {
        const p = await withRoute([[-12, -55.7], [-12, -36], [0, -36], [12, -36], [12, -12], [24, -12]]);
        const { page } = p;
        const view = async () => (await route(page)).steps.map((s) => s.view || null);
        const startSection = async (n, name) => {
            await pickStep(page, n);
            await page.locator(`.step[data-step="${n - 1}"] .step-act[aria-label="More for step ${n}"]`).click();
            await page.locator(`.popover[aria-label="Step ${n}"] .menu-item`, { hasText: 'Start a section here' }).click();
            await page.locator('.dialog input.txt').fill(name);
            await page.getByRole('button', { name: 'Start the section' }).click();
        };
        await startSection(3, 'Claw 1');
        await startSection(5, 'Back');
        const row = page.locator('.section-row[data-section="Claw 1"]');
        assert.match(await row.textContent(), /Claw 1steps 3–4/);
        await row.getByRole('button', { name: 'Hide "Claw 1" on the field' }).click();
        await row.getByRole('button', { name: 'Lock "Claw 1"' }).click();
        assert.deepEqual(await view(), [null, null, { hidden: true, locked: true, section: 'Claw 1' }, { hidden: true, locked: true },
                                        { section: 'Back' }, null]);
        assert.deepEqual(await page.evaluate(() => [...document.querySelectorAll('.g-step')].map((g) => Number(g.dataset.step))), [0, 1, 4, 5]);
        await row.getByRole('button', { name: 'Fold "Claw 1"' }).click();
        assert.equal((await view())[2].folded, true);
        await clickField(page, 50, 50);
        assert.equal(await page.locator('.step[data-step="2"], .step[data-step="3"]').count(), 0, 'folded: its steps out of the list');
        assert.equal(await page.locator('.step[data-step="4"]').count(), 1);
        await key(page, 'Control+z');
        assert.equal(await page.locator('.step[data-step="2"], .step[data-step="3"]').count(), 2, 'undo unfolds');
        await key(page, 'Control+z');
        assert.deepEqual((await view())[3], { hidden: true }, 'undo unlocks');
        await key(page, 'Control+Shift+z');
        assert.deepEqual((await view())[3], { hidden: true, locked: true }, 'redo locks');
        await row.getByRole('button', { name: 'Show "Claw 1" on the field' }).click();
        assert.deepEqual((await view()).slice(2, 4), [{ locked: true, section: 'Claw 1' }, { locked: true }], 'all hidden: one click shows them all');
        await row.getByRole('button', { name: 'More for "Claw 1"' }).click();
        await page.locator('.popover[aria-label="Section"] .menu-item', { hasText: 'Remove the section' }).click();
        assert.equal(await page.locator('.section-row[data-section="Claw 1"]').count(), 0);
        assert.deepEqual((await view())[2], { locked: true }, 'its steps stay');
        assert.deepEqual(await p.done(), []);
    });

    const CROWDED = [[-12, -55.7], A, [-12, -48, 1], A, [0, -36], [-12, -36, 1], [-24, -24], [-12, -36, 1], [-12, -24], [-24, -24],
                     [-12, -36, 1], [-36, -24]];
    for (const v of (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])) {
        const [scheme, width, height] = v;
        test(`RMP1-G screens [${scheme}-${width}]: a crowded route, then focus on step 6`, async () => {
            const p = await withRoute(CROWDED, { scheme, viewport: { width, height } });
            const { page } = p;
            await shot(page, `rmp1g-crowded-${scheme}-${width}`, true);
            await pickStep(page, 6);
            await key(page, 'o');
            await page.locator('.step[data-step="5"]').hover();
            await shot(page, `rmp1g-focus-${scheme}-${width}`, true);
            assert.deepEqual(await page.evaluate(() => [...document.querySelectorAll('.g-step:not(.faded)')].map((g) => Number(g.dataset.step))), [4, 5, 6]);
            assert.deepEqual(await p.done(), []);
        });
    }
});
