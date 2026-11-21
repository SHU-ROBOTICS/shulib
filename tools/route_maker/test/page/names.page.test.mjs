// names.page.test.mjs — RMP1-L in a browser: the words a student meets first name White Bot and Black Bot, each the
// right one, and the names robots.json carries ("robot one", the C++ description's) are never what the page shows.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, skip, launch, serve, openPage } from './harness.mjs';

let browser;
let site;
const hint = (page) => page.locator('.mode-hint').textContent();
const toast = (page) => page.locator('.toast').last().textContent();
/** Field inches (x, y) on the screen. */
const fieldPoint = (page, x, y) => page.evaluate(([fx, fy]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn)
        .matrixTransform(document.querySelector('.field-svg').getScreenCTM());
    return { x: pt.x, y: pt.y };
}, [x, y]);
/** Every word on screen: the text, the tooltips and the labels a screen reader says (not the exported C++ shown in a
 *  <pre>: its comments say "robot one", byte for byte the goldens'). */
const shown = (page) => page.evaluate(() => [[...document.querySelectorAll('pre')].reduce((t, e) => t.replace(e.innerText, ''), document.body.innerText),
    ...[...document.querySelectorAll('[aria-label], [title], [placeholder], [alt]')].flatMap((e) => ['aria-label', 'title', 'placeholder', 'alt']
        .map((a) => e.getAttribute(a) || '')), ...[...document.querySelectorAll('svg title')].map((e) => e.textContent)].join('\n'));
/** Nothing from robots.json's names, and no "robot one/two" but the bridge's "White Bot (robot one)". */
async function noLeak(page, where) {
    const text = await shown(page);
    assert.doesNotMatch(text, /LEAK/, `${where}: a name read from robots.json is shown`);
    const said = text.replace(/(White|Black) Bot \(robot (one|two)\)/g, '').match(/.{0,40}\brobot (one|two)\b.{0,20}/gi);
    assert.equal(said, null, `${where}: ${said}`);
}

test.describe('RMP1-L: White Bot and Black Bot', { skip }, () => {
    test.before(async () => {
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('the first words: the empty field, the top bar, "place its start first", the start placed, the help\'s bridge (once)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        assert.equal(await hint(page), 'Click the field where White Bot starts');
        assert.equal(await page.locator('.topbar .chip.robot').textContent(), 'White Bot');
        assert.equal(await page.locator('.topbar .chip.robot').getAttribute('title'), 'The route is written in White Bot\'s numbers');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('m');
        assert.equal(await toast(page), 'Place White Bot\'s start first: click the field where it starts.');
        const at = await fieldPoint(page, -12, -60);
        await page.mouse.click(at.x, at.y);
        assert.equal(await toast(page), 'White Bot\'s start is placed; Black Bot\'s mirrored one is the blue square. On the right: do both '
            + 'robots run this at the same time?');
        await page.getByRole('button', { name: 'Help and shortcuts (?)' }).click();
        const help = page.locator('.dialog[aria-labelledby="help-title"]');
        assert.match(await help.locator('p').first().textContent(), /place the start of White Bot \(robot one\), add steps/);
        assert.equal((await help.textContent()).match(/\(robot (one|two)\)/g).length, 1, 'the bridge once, at the first mention');
        await noLeak(page, 'the help');
        assert.deepEqual(await p.done(), []);
    });

    test('robots.json\'s names ("robot one") never reach the screen: the robots\' sentence, the routes menu, Black Bot as written, the controller, its export, an empty mechanism menu', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        const baked = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
        const leak = { one: { ...baked.one, name: 'LEAK one' }, two: { ...baked.two, name: 'LEAK two' } };
        await page.evaluate((r) => window.routeBuilder.useRobots(r), leak);
        await page.getByRole('button', { name: 'Got it' }).click();
        await noLeak(page, 'the first screen');
        const at = await fieldPoint(page, -12, -60);
        await page.mouse.click(at.x, at.y);
        await page.locator('.pane.right [data-together="false"]').click();
        await page.locator('.start-card').click();
        const robots = page.locator('.pane.right .section', { has: page.locator('h3', { hasText: /^The robots$/ }) }).locator('p');
        assert.equal(await robots.textContent(), 'The route is written in White Bot\'s numbers; Black Bot runs it MIRRORED. '
            + 'It is checked on White Bot and Black Bot\'s mechanisms.');
        await noLeak(page, 'the start\'s inspector');
        await page.locator('#route-switch').click();
        assert.equal(await page.locator('.route-row[data-route="skills"] small').textContent(), 'no steps · Black Bot mirrored · one at a time');
        await page.getByRole('button', { name: 'New route' }).click();
        await page.getByRole('button', { name: 'Add the route' }).click();
        await page.locator('.start-card').click();
        assert.equal(await page.locator('.pane.right .section', { has: page.locator('h3', { hasText: /^Black Bot$/ }) }).locator('p.hint').textContent(),
            'Black Bot runs it as written, from White Bot\'s start: for a route one robot runs.');
        await page.locator('#route-switch').click();
        assert.equal(await page.locator('.route-row[data-route="route 2"] small').textContent(), 'no steps · Black Bot as written');
        await noLeak(page, 'the routes menu');
        await page.keyboard.press('Escape');
        await page.getByRole('tab', { name: 'Controls' }).click();
        await page.getByRole('radio', { name: 'Black Bot' }).click();
        assert.equal(await page.locator('svg.pad').getAttribute('aria-label'), 'Black Bot\'s controller: click a button');
        await noLeak(page, 'Black Bot\'s controller');
        await page.keyboard.press('Control+e');
        assert.equal(await page.locator('#cx-title').textContent(), 'Export Black Bot\'s controls');
        await noLeak(page, 'the controls export');
        await page.keyboard.press('Escape');
        await page.getByRole('tab', { name: 'Route' }).click();
        await page.evaluate((r) => window.routeBuilder.useRobots(r), { one: { ...leak.one, mechanisms: [] } });
        const start2 = await fieldPoint(page, 36, -60);
        await page.mouse.click(start2.x, start2.y);  // route 2's start: as written, nothing asked
        assert.equal(await toast(page), 'White Bot\'s start is placed. Drag its red knob to turn it, then add a step.');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('m');
        assert.equal(await page.locator('.mech-menu p.hint').textContent(), 'White Bot declares no mechanism yet.');
        await noLeak(page, 'the empty mechanism menu');
        assert.deepEqual(await p.done(), []);
    });
});
