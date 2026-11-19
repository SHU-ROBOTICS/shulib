// controls.page.test.mjs — THE CONTROLS TAB AND DRIVING FEEL IN A REAL BROWSER (RMP1-B; the brief §1.2, §2.5).
// Runs only with ROUTE_MAKER_BROWSER set, on the built site (the robots and driver files of THIS commit baked in);
// every page fails on a CSP violation, a page error, a console error or a request leaving the page. With
// ROUTE_MAKER_SHOTS: the Controls tab with a popover and the Driving feel with a curve at 0.6, light and dark, 1440 and 1024.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { BROWSER, REPO, SITE, skip, VARIANTS, launch, serve, openPage, shot } from './harness.mjs';
import { shapeAxis, DEFAULT_ONLY_REFUSAL } from '../../src/model/feel.mjs';
import { editControl, toSaved } from '../../src/model/controls.mjs';
import { readDriverFile } from '../../src/model/driver_file.mjs';

const MARKED = fs.existsSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'driver_one_marked.hpp'))
    ? fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'driver_one_marked.hpp'), 'utf8') : '';
const TODAY = fs.readFileSync(path.join(REPO, 'src', 'team', 'robot_one_driver.hpp'), 'utf8');
// while the drive program's boot line names the default feel, the build (and so the page) refuses any other
const DEFAULT_ONLY = fs.readFileSync(path.join(REPO, 'src', 'programs', 'drive_program.cpp'), 'utf8').includes('driverLineMatchesFeel');
/** The text outside the driver file's three marked regions (the markers kept). */
const outside = (text) => text.replace(/(\/\/ \[(driver file|control words|feel) one\]\r?\n)[\s\S]*?(\/\/ \[\/\2 one\])/g, '$1$3');

let browser;
let site;
const controls = (page) => page.evaluate(() => window.routeBuilder.controls());
const errorsOf = async (page) => (await controls(page)).issues.filter((x) => x.level === 'error');

/** Open the Controls tab (the coach card closed first: it belongs to the Route tab). */
async function openControls(page) {
    await page.getByRole('button', { name: 'Got it' }).click();
    await page.getByRole('tab', { name: 'Controls' }).click();
    await page.locator('svg.pad').waitFor();
}
/** Pick a file in the page's own file chooser (the download way's "Pick robot_one_driver.hpp"). */
async function pickFile(page, click, name, text) {
    const [fc] = await Promise.all([page.waitForEvent('filechooser'), click()]);
    await fc.setFiles({ name, mimeType: 'text/plain', buffer: Buffer.from(text, 'utf8') });
}
/** Each region's text in a driver file (LF), by its markers. */
const regionsIn = (text) => Object.fromEntries(['driver file', 'control words', 'feel'].map((r) => {
    const m = new RegExp(`// \\[${r} one\\]\\r?\\n([\\s\\S]*?)\\r?\\n// \\[/${r} one\\]`).exec(text);
    return [r, m ? m[1].replace(/\r\n/g, '\n') : null];
}));
async function rightPane(page, viewport) {
    if (viewport && viewport.width <= 1100) {
        await page.waitForTimeout(250);
    }
}

test.describe('the Controls tab in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        if (browser) {
            await browser.close();
        }
        if (site) {
            site.server.close();
        }
    });

    test('robot one\'s map is read back EXACTLY from its driver file: every button\'s job on the controller, Y the re-zero, the d-pad free', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const c = await controls(page);
        assert.equal(c.robot, 'one');
        assert.deepEqual(c.state.controls.list.map((x) => `${x.form} ${x.mechanism} ${[x.button, x.back].filter(Boolean).join('/')}`), [
            'whileHeld cascade R1/R2', 'toggle claw A', 'toggle flipper B', 'toggle ramp X', 'whileHeld rightToggle L1', 'whileHeld leftToggle L2',
            'homeAtDriverStart cascade ']);
        assert.deepEqual(c.meta.problems, [], 'the file\'s map and feel were read without a problem');
        assert.deepEqual(c.issues, [], 'the builder\'s map passes every rule');
        assert.match(await page.locator('[data-button="R1"]').getAttribute('aria-label'), /^R1: cascade: up \(hold\)$/);
        assert.match(await page.locator('[data-button="L1"]').getAttribute('aria-label'), /^L1: right toggle: hold to run$/);
        assert.match(await page.locator('[data-button="Y"]').getAttribute('aria-label'), /re-zero/);
        for (const b of ['Up', 'Down', 'Left', 'Right']) {
            assert.equal(await page.locator(`[data-button="${b}"]`).getAttribute('class'), 'cbtn free', b);
        }
        assert.ok(await page.locator('.statusbar').getByText('Ready to export').isVisible());
        assert.equal(await page.locator('#preview').count(), 0, 'PREVIEW is the route\'s');
        // every button: hover help with its whole job, and a target of at least 32 px (the design system's --hit)
        for (const b of ['L1', 'L2', 'R1', 'R2', 'Up', 'Down', 'Left', 'Right', 'X', 'B', 'Y', 'A']) {
            assert.ok((await page.locator(`[data-button="${b}"] > title`).textContent()).startsWith(`${b}: `), b);
            const box = await page.locator(`[data-button="${b}"] .cbtn-hit`).boundingBox();
            assert.ok(box.width >= 32 && box.height >= 32, `${b}: ${box.width} x ${box.height} px`);
        }
        // the keyboard: Enter on a button selects it and keeps the focus on it (the drawing is re-made)
        await page.locator('[data-button="A"]').focus();
        await page.keyboard.press('Enter');
        assert.equal(await page.evaluate(() => document.activeElement.dataset.button), 'A');
        assert.match(await page.locator('#controls .pane.right .insp-title').textContent(), /A · toggle/);
        // ? opens the help here too, and it speaks of this tab
        await page.keyboard.press('?');
        assert.ok(await page.locator('.dialog').getByText(/The Controls tab: click a button/).isVisible());
        await page.keyboard.press('Escape');
        assert.deepEqual(await p.done(), []);
    });

    test('a free button gets a job in two clicks; the map\'s "d-pad FREE" line goes stale and is fixed; a refusal in plain words with the build\'s name; undo and redo', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('[data-button="Up"]').click();
        // the menu opens BESIDE the button clicked (the drawing is re-made on a click: never at the page's corner)
        const [btn, pop] = [await page.locator('[data-button="Up"]').boundingBox(), await page.locator('.popover').boundingBox()];
        const gap = pop.x >= btn.x + btn.width ? pop.x - (btn.x + btn.width) : btn.x - (pop.x + pop.width);
        assert.ok(gap >= 0 && gap <= 24, `beside it (a gap of ${gap} px): ${JSON.stringify({ btn, pop })}`);
        assert.ok(pop.y <= btn.y + btn.height && pop.y + pop.height >= btn.y, `level with it: ${JSON.stringify({ btn, pop })}`);
        await page.locator('.popover').getByRole('menuitem', { name: 'claw: Go to "open" on one press', exact: true }).click();
        assert.equal(await page.locator('.popover').count(), 0, 'the menu closes once a job is chosen');
        let c = await controls(page);
        const added = c.state.controls.list[7];
        assert.deepEqual([added.form, added.mechanism, added.button, added.preset], ['preset', 'claw', 'Up', 'open']);
        // the header's map said the d-pad is FREE: now it would lie -- an error, with its fix beside the control
        assert.deepEqual((await errorsOf(page)).map((x) => x.code), ['mapStale']);
        assert.equal(await page.locator('#export .count').textContent(), '1');
        await page.getByRole('button', { name: 'Remove that line' }).click();
        assert.deepEqual(await errorsOf(page), []);
        // a refusal: the claw's toggle moved onto B, the flipper's -- the build's own name beside the plain words
        await page.locator('[data-button="A"]').click();
        await page.locator('.pane.right select[name="button"]').selectOption('B');
        const errs = await errorsOf(page);
        assert.deepEqual(errs.map((x) => x.refusal), ['button_B_does_two_jobs']);
        assert.ok(await page.locator('.pane.right .lib', { hasText: 'button_B_does_two_jobs' }).isVisible());
        assert.match(await page.locator('[data-button="B"]').getAttribute('class'), /\bbad\b/);
        // undo, redo by the keyboard (from the page, not a text box)
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.deepEqual(await errorsOf(page), []);
        assert.equal((await controls(page)).state.controls.list[1].button, 'A');
        await page.keyboard.press('Control+Shift+z');
        assert.equal((await controls(page)).state.controls.list[1].button, 'B');
        await page.keyboard.press('Control+z');
        // the project keeps it in G3a's shape (robots.one.controls, per kind, each with its place)
        c = await controls(page);
        assert.equal(c.state.controls.list.length, 8);
        // connecting the folder afterwards reads today's file only for a robot not changed here: the edit stays,
        // and so does its undo (the review's finding)
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/robot_one_driver.hpp': t }), TODAY);
        await page.getByRole('tab', { name: 'Controls' }).click();
        c = await controls(page);
        assert.equal(c.state.controls.list.length, 8, 'the claw on Up is still there');
        assert.equal(c.state.controls.list[7].button, 'Up');
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.equal((await controls(page)).state.controls.list[7].button, 'Up', 'one undo takes back the map line, not the claw');
        assert.deepEqual(await p.done(), []);
    });

    test('driving feel: the curve and the dead zone move the LIBRARY\'s curve (the readout is shapeAxis), the ramp says its time, and the export writes kFeel', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('input[name="feel-expo"]').fill('0.6');
        await page.locator('input[name="feel-expo"]').press('Enter');
        const want = Math.round(shapeAxis(0.3, { deadband: 0.05, expo: 0.6 }) * 100);
        assert.equal(await page.locator('.readout strong').textContent(), `30 % stick → ${want} % speed`);
        assert.ok(want < 30, 'a small push is slower with a curve');
        // trying another push moves only the dot: the graph is the same element, the readout the library's
        await page.evaluate(() => { document.querySelector('svg.graph').dataset.same = '1'; });
        await page.locator('.g-probe').focus();
        await page.keyboard.press('ArrowRight');
        const at35 = Math.round(shapeAxis(0.35, { deadband: 0.05, expo: 0.6 }) * 100);
        assert.equal(await page.locator('.readout strong').textContent(), `35 % stick → ${at35} % speed`);
        assert.equal(await page.locator('svg.graph').getAttribute('data-same'), '1', 'the graph was re-made for a dot move');
        await page.keyboard.press('ArrowLeft');
        await page.locator('input[name="feel-ramp"]').fill('4');
        await page.locator('input[name="feel-ramp"]').press('Enter');
        assert.ok(await page.getByText('full speed from rest in 0.25 s').isVisible());
        const c = await controls(page);
        assert.deepEqual(c.state.driving.forward, { curve: 0.6 });
        assert.equal(c.state.driving.rampPerSecond, 4);
        // the dead zone typed out of range (0.9996 is stored as 1, the page's 3 decimals): the build's refusal (D10)
        await page.locator('input[name="feel-deadband"]').fill('0.9996');
        await page.locator('input[name="feel-deadband"]').press('Enter');
        assert.ok((await errorsOf(page)).some((x) => x.refusal === 'the feel is out of range'));
        // a typed word is said to be no number, and nothing is kept
        await page.locator('input[name="feel-deadband"]').fill('abc');
        await page.locator('input[name="feel-deadband"]').press('Enter');
        assert.ok(await page.getByText('Not a number: type digits, like 0.25.').isVisible());
        await page.locator('input[name="feel-deadband"]').fill('0.1');
        await page.locator('input[name="feel-deadband"]').press('Enter');
        await page.keyboard.press('Control+e');
        await page.locator('.drawer').waitFor();
        if (DEFAULT_ONLY) {
            // the robot's programs take only the default feel today: refused with the drive program's own words, and
            // Export lists it with "Go there"
            assert.deepEqual((await errorsOf(page)).map((x) => x.refusal), [DEFAULT_ONLY_REFUSAL]);
            assert.ok(await page.locator('.drawer').getByText(/cannot take a feel other than the library's default/).isVisible());
            await page.locator('.drawer').getByRole('button', { name: 'Go there' }).click();
            await page.getByRole('button', { name: 'Use the library\'s defaults' }).click();
            assert.deepEqual(await errorsOf(page), []);
        } else {
            const code = await page.locator('.drawer pre.code').nth(2).textContent();
            assert.match(code, /inline constexpr DriverProfile kFeel\{\n {4}\.forward = \{\.deadband = 0\.1, \.expo = 0\.6\},\n {4}\.slew = \{\.ratePerSecond = 4\},\n {4}\.mode = ArcadeBody,\n\};/);
            assert.match(code, /Forward: dead zone 0\.1, curve 0\.6\./, 'the comment says the new feel, not the old default\'s');
        }
        assert.deepEqual(await p.done(), []);
    });

    test('export: today\'s driver file (no "control words" / "feel" markers) is Copy only; the marked file is written ONLY between its markers, CRLF kept; a hand edit is the student\'s to keep or overwrite', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        // (1) today's file: read, never written
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/robot_one_driver.hpp': t }), TODAY);
        await page.getByRole('tab', { name: 'Controls' }).click();
        await page.keyboard.press('Control+e');
        await page.locator('.drawer').waitFor();
        if (!TODAY.includes('// [control words one]')) {
            assert.ok(await page.getByText(/has no "control words" and "feel" markers yet/).isVisible());
            assert.ok(await page.getByRole('button', { name: 'Write into src/team/robot_one_driver.hpp' }).isDisabled());
        }
        assert.ok(await page.getByRole('button', { name: 'Copy all three' }).isEnabled());
        await page.keyboard.press('Escape');
        // (2) the marked file, CRLF (a Windows checkout): a change written into its three parts only
        const crlf = MARKED.replace(/\n/g, '\r\n');
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/robot_one_driver.hpp': t }), crlf);
        await page.locator('select[name="onRelease"]').selectOption('brake');  // a feel change every build takes
        // ...and the FIRST region (the header's map) changes length: Up gets a job, the 3-line "d-pad FREE" row goes
        await page.locator('[data-button="Up"]').click();
        await page.locator('.popover').getByRole('menuitem', { name: 'claw: Go to "open" on one press', exact: true }).click();
        await page.getByRole('button', { name: 'Remove that line' }).click();
        await page.keyboard.press('Control+e');
        const shown = await page.locator('.drawer pre.code').allTextContents();
        await page.getByRole('button', { name: 'Write into src/team/robot_one_driver.hpp' }).click();
        await page.getByText(/only the three marked parts changed/).waitFor();
        const written = await page.evaluate(() => window.routeBuilder.memory.store['src/team/robot_one_driver.hpp']);
        assert.ok(!/[^\r]\n/.test(written), 'no bare LF written into a CRLF file');
        assert.notEqual(written, crlf);
        assert.equal(outside(written), outside(crlf), 'nothing outside the three marked parts changed');
        assert.deepEqual(Object.values(regionsIn(written)), shown, 'each part between its OWN markers, as the drawer showed it');
        assert.notEqual(regionsIn(written)['control words'].split('\n').length, regionsIn(crlf)['control words'].split('\n').length);
        assert.match(written, /\.releaseBrakeMode = Brake, +\/\/ let go of the sticks: the drive brakes\r\n/);
        assert.match(written, /\/\/ {3}let go of the sticks and the drive brakes\r\n/, 'the button map says it too');
        const saved = JSON.parse(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']));
        for (const r of ['controls', 'control words', 'feel']) {
            assert.match(saved.exports[`src/team/robot_one_driver.hpp#${r}`], /^sha256:[0-9a-f]{64}$/, r);
        }
        assert.equal(saved.robots.one.driving.onRelease, 'brake');
        await page.keyboard.press('Escape');
        // (3) a teammate changed THE FEEL by hand since: refused until the student says
        const hand = written.replace('// field-centric\'s "away from me"', '// ours');
        await page.evaluate((t) => { window.routeBuilder.memory.store['src/team/robot_one_driver.hpp'] = t; }, hand);
        await page.locator('select[name="onRelease"]').selectOption('hold');
        await page.keyboard.press('Control+e');
        await page.getByRole('button', { name: 'Write into src/team/robot_one_driver.hpp' }).click();
        await page.getByRole('button', { name: 'Keep their change' }).click();
        assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/robot_one_driver.hpp']), hand, 'kept: nothing written');
        assert.deepEqual(await p.done(), []);
    });

    test('a project whose feel names a drive mode this builder does not know: the controller is still drawn, and the mode is said to be wrong', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const { readDriverFile } = await import('../../src/model/driver_file.mjs');
        const { toSaved } = await import('../../src/model/controls.mjs');
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { one: { controls: toSaved(readDriverFile(MARKED, 'one').state.controls), driving: { mode: 'tank' } } };
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.equal(await page.locator('svg.pad').count(), 1);
        assert.ok((await errorsOf(page)).some((x) => /"tank" is not a drive mode/.test(x.words)));
        assert.deepEqual(await p.done(), []);
    });

    test('a robot\'s controls this route builder cannot read (a newer one\'s) are shown read only, and saving the OTHER robot leaves them untouched', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        const newer = { controls: { toggles: [{ at: 0, mechanism: 'claw', button: 'A', glow: 'blue' }] }, driving: { mode: 'arcade' } };
        project.robots = { two: newer };
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
        await page.getByRole('tab', { name: 'Controls' }).click();
        await page.getByRole('radio', { name: 'two' }).click();
        assert.match((await controls(page)).meta.locked, /cannot be read here \(a "toggles" entry has a field "glow"/);
        await page.getByRole('radio', { name: 'one' }).click();
        await page.locator('[data-button="Up"]').click();
        await page.locator('.popover').getByRole('menuitem', { name: 'claw: Go to "open" on one press', exact: true }).click();
        await page.keyboard.press('Control+s');
        await page.waitForFunction(() => /"one"/.test(window.routeBuilder.memory.store['src/team/route_maker.json'] || ''));
        const saved = JSON.parse(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']));
        assert.deepEqual(saved.robots.two, newer, 'robot two\'s block exactly as the newer builder wrote it');
        assert.ok(saved.robots.one.controls, 'robot one\'s edit saved');
        assert.deepEqual(await p.done(), []);
    });

    test('saved, closed and opened again: Export writes into the untouched driver file with NO hand-edit question (the review\'s finding)', async () => {
        let p = await openPage(browser, site.base);
        let { page } = p;
        await openControls(page);
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/robot_one_driver.hpp': t }), MARKED);
        await page.getByRole('tab', { name: 'Controls' }).click();
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+s');
        await page.waitForFunction(() => /"one"/.test(window.routeBuilder.memory.store['src/team/route_maker.json'] || ''));
        const store = await page.evaluate(() => ({ ...window.routeBuilder.memory.store }));
        assert.equal(store['src/team/robot_one_driver.hpp'], MARKED, 'Save writes the project file only');
        assert.deepEqual(await p.done(), []);
        p = await openPage(browser, site.base);
        ({ page } = p);
        await page.getByRole('button', { name: 'Got it' }).click();
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), store);
        await page.getByRole('tab', { name: 'Controls' }).click();
        const c = await controls(page);
        assert.deepEqual([c.meta.source, c.state.driving.onRelease], ['project', 'brake']);
        await page.keyboard.press('Control+e');
        await page.getByRole('button', { name: 'Write into src/team/robot_one_driver.hpp' }).click();
        await page.getByText(/only the three marked parts changed/).waitFor();
        assert.equal(await page.getByRole('button', { name: 'Keep their change' }).count(), 0, 'no hand edit was claimed');
        assert.match(await page.evaluate(() => window.routeBuilder.memory.store['src/team/robot_one_driver.hpp']), /\.releaseBrakeMode = Brake,/);
        assert.deepEqual(await p.done(), []);
    });

    test('undo acts on the robot SHOWN: robot one\'s edit is not undone from robot two\'s screen (the review\'s finding)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.getByRole('radio', { name: 'two' }).click();
        const two = (await controls(page)).state;
        assert.ok(await page.getByRole('button', { name: 'Undo (Ctrl+Z)' }).isDisabled(), 'nothing to undo on robot two');
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.deepEqual((await controls(page)).state, two, 'robot two unchanged');
        await page.getByRole('radio', { name: 'one' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake', 'robot one\'s edit still there');
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.equal((await controls(page)).state.driving.onRelease, 'coast');
        assert.deepEqual(await p.done(), []);
    });

    test('Controls changed on a page with no route, then the folder connected: kept when its project has none, else one Ctrl+Z away (the verifier\'s finding)', async () => {
        // the team's project in the folder: a route with a start, no Controls
        let p = await openPage(browser, site.base);
        let { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/robot_one_driver.hpp': t }), MARKED);
        const box = await page.locator('.field-svg').boundingBox();
        await page.mouse.click(box.x + box.width * 0.3, box.y + box.height * 0.8);
        await page.keyboard.press('Control+s');
        await page.waitForFunction(() => !!window.routeBuilder.memory.store['src/team/route_maker.json']);
        const team = await page.evaluate(() => ({ ...window.routeBuilder.memory.store }));
        assert.deepEqual(JSON.parse(team['src/team/route_maker.json']).robots || {}, {}, 'no Controls in it');
        assert.deepEqual(await p.done(), []);
        // a student changes the Controls on a fresh page FIRST, then connects: the route opens, the change stays
        p = await openPage(browser, site.base);
        ({ page } = p);
        await openControls(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), team);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake');
        assert.ok((await page.evaluate(() => window.routeBuilder.route())).start, 'the team\'s route is open');
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Opened src\/team\/route_maker\.json from the folder\. Your Controls changes for robot one are kept/);
        assert.match(await page.locator('.pane-body').last().textContent(), /You changed it here, so today's file in your folder is not read in/, 'never "Connect your folder" once it is');
        await page.keyboard.press('Control+s');
        await page.waitForFunction(() => /"one"/.test(window.routeBuilder.memory.store['src/team/route_maker.json'] || ''));
        const both = await page.evaluate(() => ({ ...window.routeBuilder.memory.store }));
        const saved = JSON.parse(both['src/team/route_maker.json']);
        assert.deepEqual([!!saved.routes[0].start, saved.robots.one.driving.onRelease], [true, 'brake'], 'Save keeps the route AND the change');
        assert.deepEqual(await p.done(), []);
        // the folder's project now has robot one's own Controls: they open, and Ctrl+Z brings the page's back
        p = await openPage(browser, site.base);
        ({ page } = p);
        await openControls(page);
        await page.locator('select[name="onRelease"]').selectOption('hold');
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), both);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake', 'the folder\'s');
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Ctrl\+Z on the Controls tab brings yours back/);
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.equal((await controls(page)).state.driving.onRelease, 'hold', 'the page\'s, one Ctrl+Z away');
        assert.deepEqual(await p.done(), []);
        // a page WITH a route asks first, and says so of the Controls; opened, today's driver file comes in, and Ctrl+Z
        // still brings the page's change back
        p = await openPage(browser, site.base);
        ({ page } = p);
        await page.getByRole('button', { name: 'Got it' }).click();
        await page.mouse.click(box.x + box.width * 0.6, box.y + box.height * 0.8);
        await page.getByRole('tab', { name: 'Controls' }).click();
        await page.locator('select[name="onRelease"]').selectOption('hold');
        const asked = page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), team);
        await page.getByRole('alertdialog').waitFor();
        assert.match(await page.getByRole('alertdialog').textContent(), /different project[\s\S]*route and Controls[\s\S]*Ctrl\+Z on the Controls tab/);
        await page.getByRole('button', { name: 'Open the folder\'s route' }).click();
        await asked;
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.deepEqual([(await controls(page)).meta.source, (await controls(page)).state.driving.onRelease], ['folder', 'coast']);
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.equal((await controls(page)).state.driving.onRelease, 'hold');
        assert.deepEqual(await p.done(), []);
    });

    test('Controls changed here, then a folder project whose controls for that robot this page cannot read: the change is gone, and the page says so (guide 22\'s one exception)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.getByRole('radio', { name: 'two' }).click();
        await page.locator('select[name="onRelease"]').selectOption('brake');
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { two: { controls: { toggles: [{ at: 0, mechanism: 'claw', button: 'A', glow: 'blue' }] }, driving: { mode: 'arcade' } } };
        const folder = { 'src/team/route_maker.json': `${JSON.stringify(project, null, 2)}\n`, 'src/team/robot_one_driver.hpp': 'int x;\n' };
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), folder);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Its Controls for robot two are read only here, so your changes there are gone\./);
        assert.doesNotMatch((await page.locator('.toast').allTextContents()).join(' '), /brings yours back|are kept/);
        assert.match((await controls(page)).meta.locked, /cannot be read here/);
        // the right pane says why today's file is not read in: never "You changed it here" for a change that is gone
        const pane = async () => page.locator('.pane-body').last().textContent();
        assert.match(await pane(), /Your folder has no src\/team\/robot_two_driver\.hpp\./);
        assert.doesNotMatch(await pane(), /You changed it here/);
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.notEqual((await controls(page)).state.driving.onRelease, 'brake', 'nothing to bring back');
        await page.getByRole('radio', { name: 'one' }).click();
        assert.match(await pane(), /Today's file in your folder was not read in \(src\/team\/robot_one_driver\.hpp has no /);
        assert.doesNotMatch(await pane(), /You changed it here/);
        assert.deepEqual(await p.done(), []);
    });

    test('words the team types after a remap are theirs: the file\'s stale (start) row is flagged, "Use the page\'s words" replaces only that row, and true words are taken (the verifier\'s finding)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const move = async (b, field, to) => {
            await page.locator(`[data-button="${b}"]`).click();
            await page.locator(`.pane.right select[name="${field}"]`).selectOption(to);
        };
        await move('L1', 'button', 'Up');  // the side toggles parked, the cascade onto L1 / L2, the toggles onto R1 / R2
        await move('L2', 'button', 'Down');
        await move('R1', 'button', 'L1');
        await move('R2', 'back', 'L2');
        await move('Up', 'button', 'R1');
        await move('Down', 'button', 'R2');
        const errs = await errorsOf(page);
        assert.deepEqual(errs.map((x) => x.code), ['wordsStale']);
        assert.match(errs[0].words, /map row "\(start\)" names R1, R2/);
        await page.getByRole('button', { name: 'home at driver start', exact: true }).click();
        const start = errs[0].at;
        const row = (await controls(page)).state.controls.list[start].banner[0].lines;
        await page.getByRole('button', { name: 'Use the page\'s words' }).click();
        assert.deepEqual(await errorsOf(page), []);
        assert.match((await controls(page)).state.controls.list[start].words, /^\[delete-able\]/, 'its comment untouched');
        // undone, the team writes the row TRUE itself: taken
        await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
        await page.keyboard.press('Control+z');
        assert.equal((await errorsOf(page)).length, 1);
        await page.getByRole('button', { name: 'home at driver start', exact: true }).click();
        await page.locator('.pane.right textarea[name="banner0"]').fill(row.join('\n').replaceAll('R1 / R2', 'L1 / L2'));
        await page.locator('.pane.right textarea[name="banner0"]').blur();
        assert.deepEqual(await errorsOf(page), []);
        assert.match((await controls(page)).state.controls.list[start].banner[0].lines.join(' '), /L1 \/ L2 wait/);
        assert.deepEqual(await p.done(), []);
    });

    test('a project opened again: its own words, written for its own map, are judged against that map, not the driver file\'s (the verifier\'s finding)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const one = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots.one;
        const r = readDriverFile(TODAY, 'one');
        let c = r.state.controls;
        const at = (mech, form) => c.list.findIndex((x) => x.mechanism === mech && x.form === form);
        for (const [mech, patch] of [['rightToggle', { button: 'Up' }], ['leftToggle', { button: 'Down' }], ['cascade', { button: 'L1', back: 'L2' }],
                                     ['rightToggle', { button: 'R1' }], ['leftToggle', { button: 'R2' }]]) {
            c = editControl(c, at(mech, 'whileHeld'), patch, one);
        }
        const s = at('cascade', 'homeAtDriverStart');
        c = editControl(c, s, { banner: c.list[s].banner.map((x) => ({ ...x, lines: x.lines.map((l) => l.replaceAll('R1 / R2', 'L1 / L2')) })) }, one);
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { one: { controls: toSaved(c), driving: r.state.driving } };
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
        await page.getByRole('tab', { name: 'Controls' }).click();
        const got = await controls(page);
        assert.deepEqual([got.meta.source, got.state.controls.list[s].banner[0].lines.join(' ').includes('L1 / L2 wait')], ['project', true]);
        assert.deepEqual(got.issues.filter((x) => x.level === 'error'), []);
        assert.deepEqual(await p.done(), []);
    });

    test('G4: the menu says in full what each job picks -- each piston position by name; the second button named, or left to the student when taken', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('[data-button="Left"]').click();
        const items = await page.locator('.popover [role="menuitem"]').evaluateAll((els) => els.map((e) => e.getAttribute('aria-label')));
        assert.deepEqual(items.filter((x) => x.startsWith('claw: ')), ['claw: Each press flips it', 'claw: Go to "closed" on one press', 'claw: Go to "open" on one press']);
        assert.ok(items.includes('cascade: Move while held: Left up, Right down'), items.join(' | '));
        await page.keyboard.press('Escape');
        await page.locator('[data-button="Right"]').click();
        await page.locator('.popover').getByRole('menuitem', { name: 'ramp: Go to "down" on one press', exact: true }).click();
        await page.locator('[data-button="Left"]').click();
        await page.locator('.popover').getByRole('menuitem', { name: 'cascade: Move while held: Left up, a second button you pick', exact: true }).click();
        const c = await controls(page);
        const added = c.state.controls.list.at(-1);
        assert.deepEqual([added.form, added.button, added.back], ['whileHeld', 'Left', null], 'the second button is the student\'s to choose');
        assert.ok(c.issues.some((x) => x.code === 'unfinished' && /second button/.test(x.words)));
        assert.deepEqual(await p.done(), []);
    });

    test('the download way (Firefox, Safari): pick the driver file, get it back written only between its markers; a hand edit asks first, and Keep downloads nothing', {
        skip: BROWSER !== 'firefox' && 'Chrome and Edge write into the folder (the tests above); this branch is the other browsers\' -- Firefox here',
    }, async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        assert.ok(await page.getByText(/Export checks the file you pick against this/).isVisible(), 'never "connect a folder" where it cannot be');
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.keyboard.press('Control+e');
        const pick = () => page.getByRole('button', { name: 'Pick robot_one_driver.hpp and download the updated one' }).click();
        const [dl] = await Promise.all([page.waitForEvent('download'), pickFile(page, pick, 'robot_one_driver.hpp', MARKED)]);
        assert.equal(dl.suggestedFilename(), 'robot_one_driver.hpp');
        const back = fs.readFileSync(await dl.path(), 'utf8');
        assert.match(back, /\.releaseBrakeMode = Brake, +\/\/ let go of the sticks: the drive brakes\n/, 'the change, not the file picked');
        assert.equal(outside(back), outside(MARKED), 'nothing outside the three marked parts changed');
        // a teammate's hand edit in the file picked: asked first; Keep downloads nothing
        const hand = MARKED.replace('// field-centric\'s "away from me"', '// ours');
        await pickFile(page, pick, 'robot_one_driver.hpp', hand);
        let downloads = 0;
        page.on('download', () => { downloads += 1; });
        await page.getByRole('button', { name: 'Keep their change' }).click();
        await page.getByText('Kept. Nothing was written.').waitFor();
        assert.equal(downloads, 0);
        assert.deepEqual(await p.done(), []);
    });

    // ── §2.5.2 (B): the Controls tab with a popover; the Driving feel with a curve at 0.6 ──
    for (const v of VARIANTS) {
        test(`§2.5.2 [${v.tag}] the controller with a free button's menu open; a refusal; the driving feel at a curve of 0.6; the export`, async () => {
            const p = await openPage(browser, site.base, v);
            const { page } = p;
            await openControls(page);
            await shot(page, `controls-${v.tag}`);
            await page.locator('[data-button="Up"]').click();
            await page.locator('.popover').waitFor();
            await shot(page, `controls-popover-${v.tag}`);
            await page.keyboard.press('Escape');
            await page.locator('[data-button="A"]').click();
            await rightPane(page, v.viewport);
            await page.locator('.pane.right select[name="button"]').selectOption('B');
            await shot(page, `controls-refused-${v.tag}`, true);
            await page.keyboard.press('Control+z');
            await page.locator('input[name="feel-expo"]').fill('0.6');
            await page.locator('input[name="feel-expo"]').press('Enter');
            await page.locator('.feel-card').scrollIntoViewIfNeeded();
            await shot(page, `feel-${v.tag}`, true);
            await page.locator('svg.pad').click({ position: { x: 5, y: 5 } });
            await page.keyboard.press('Control+z');  // back to the file's map: the export shows its three parts
            await page.keyboard.press('Control+e');
            await page.locator('.drawer pre.code').nth(2).waitFor();
            await page.locator('.drawer').waitFor();
            await shot(page, `controls-export-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });
    }
});
