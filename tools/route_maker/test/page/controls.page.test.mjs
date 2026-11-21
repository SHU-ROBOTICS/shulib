// controls.page.test.mjs — THE CONTROLS TAB AND DRIVING FEEL IN A REAL BROWSER (RMP1-B; RMP1-D: one step at a time).
// Runs only with ROUTE_MAKER_BROWSER set, on the built site (the robots and driver files of THIS commit baked in);
// every page fails on a CSP violation, a page error, a console error or a request leaving the page. With
// ROUTE_MAKER_SHOTS: both steps, a free button's choices, a refusal and the export, light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { BROWSER, REPO, SITE, skip, VARIANTS, launch, serve, openPage, shot } from './harness.mjs';
import { shapeAxis, DEFAULT_ONLY_REFUSAL, editDriving } from '../../src/model/feel.mjs';
import { editControl, addControl, removeFreeLine, checkMap, followDriving, toSaved } from '../../src/model/controls.mjs';
import { readDriverFile, exportRegions, REGIONS } from '../../src/model/driver_file.mjs';

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
/** Step 2, Driving feel (`more`: its "More settings" open too). */
async function feel(page, more = true) {
    await page.getByRole('tab', { name: 'Step 2: Driving feel' }).click();
    if (more && !(await page.locator('details.feel-more').evaluate((d) => d.open))) {
        await page.locator('details.feel-more > summary').click();
    }
}
const buttons = (page) => page.getByRole('tab', { name: 'Step 1: Buttons' }).click();
/** Give free button `b` a job: the button, its mechanism, then the job (each a large button in the editor). */
async function give(page, b, mech, words) {
    await page.locator(`[data-button="${b}"]`).click();
    await page.locator('.ctl-editor .choice', { hasText: new RegExp(`^${mech}`) }).click();
    await page.locator('.ctl-editor').getByRole('button', { name: `${mech}: ${words}`, exact: true }).click();
}
/** The keyboard off any text box (Ctrl+Z is the page's there). */
const away = (page) => page.evaluate(() => document.activeElement && document.activeElement.blur());
const editor = (page) => page.locator('.ctl-editor');
const title = (page) => editor(page).locator('.insp-title').first().textContent();
/** A design token's colour as the browser computes it (rgb(...)). */
const token = (page, name) => page.evaluate((v) => {
    const d = document.body.appendChild(document.createElement('i'));
    d.style.color = `var(${v})`;
    const c = getComputedStyle(d).color;
    d.remove();
    return c;
}, name);
/** Robot one's controls as built into the page (the robots the site baked in). */
const ONE = () => JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots.one;
const CLAW_OPEN = { form: 'preset', mechanism: 'claw', button: 'Up', preset: 'open' };

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
            // clearly marked (D2): drawn dashed, and its label says "free"
            assert.notEqual(await page.locator(`[data-button="${b}"] > circle:not(.cbtn-hit)`).evaluate((x) => getComputedStyle(x).strokeDasharray), 'none', b);
            assert.equal(await page.locator(`[data-label="${b}"]`).textContent(), `${b} · free`);
        }
        assert.ok(await page.locator('.statusbar').getByText('Ready to export').isVisible());
        assert.equal(await page.locator('#preview').count(), 0, 'PREVIEW is the route\'s');
        // every button: hover help with its whole job, and a target of at least 32 px (the design system's --hit)
        for (const b of ['L1', 'L2', 'R1', 'R2', 'Up', 'Down', 'Left', 'Right', 'X', 'B', 'Y', 'A']) {
            assert.ok((await page.locator(`[data-button="${b}"] > title`).textContent()).startsWith(`${b}: `), b);
            const box = await page.locator(`[data-button="${b}"] .cbtn-hit`).boundingBox();
            assert.ok(box.width >= 32 && box.height >= 32, `${b}: ${box.width} x ${box.height} px`);
        }
        // a click keeps the keyboard on the button (the drawing is re-made); Enter takes it into the editor, Esc back
        await page.locator('[data-button="A"]').click();
        assert.equal(await page.evaluate(() => document.activeElement.dataset.button), 'A');
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'claw');
        assert.match(await editor(page).locator('.ed-head').textContent(), /^A\s*claw\s*Each press flips it/);
        await page.keyboard.press('Enter');
        assert.equal(await page.evaluate(() => document.activeElement.name), 'mechanism', 'Enter on a used button: the keyboard on its first field');
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'Give a button a job');
        assert.equal(await page.evaluate(() => document.activeElement.dataset.button), 'A');
        // ? opens the help here too, and it speaks of this tab
        await page.keyboard.press('?');
        assert.ok(await page.locator('.dialog').getByText(/The Controls tab: click a button/).isVisible());
        await page.keyboard.press('Escape');
        assert.deepEqual(await p.done(), []);
    });

    test('a free button gets a job in one action: the map\'s "d-pad FREE" line goes with it (one Ctrl+Z); a refusal in plain words with the build\'s name, the chosen button still red; undo and redo', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('[data-button="Up"]').click();
        // the editor opens BESIDE the controller (never over it), and says what it is for
        const [pad, ed] = [await page.locator('.pad-card').boundingBox(), await editor(page).boundingBox()];
        assert.ok(ed.x >= pad.x + pad.width, `beside the controller: ${JSON.stringify({ pad, ed })}`);
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'Up is free');
        await give(page, 'Up', 'claw', 'Go to "open" on one press');
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'claw', 'once a job is chosen, the editor shows it');
        let c = await controls(page);
        const added = c.state.controls.list[7];
        assert.deepEqual([added.form, added.mechanism, added.button, added.preset], ['preset', 'claw', 'Up', 'open']);
        // the header's map said the d-pad is FREE: that line would lie, so it goes in the SAME action, said in words
        // (the tutorial's finding: it used to be an error and a second click, "Remove that line")
        assert.deepEqual(await errorsOf(page), []);
        assert.deepEqual(c.state.controls.banner.free, []);
        assert.equal(await page.locator('.toast').textContent(), 'Added: finish it in the panel. The button map\'s "d-pad" line said free, so it went too (Ctrl+Z undoes both).');
        await away(page);
        await page.keyboard.press('Control+z');
        c = await controls(page);
        assert.deepEqual([c.state.controls.list.length, c.state.controls.banner.free.map((r) => r.key)], [7, ['d-pad']], 'one Ctrl+Z: the job and the line');
        await page.keyboard.press('Control+Shift+z');
        assert.deepEqual((await controls(page)).state.controls.banner.free, []);
        // a refusal: the claw's toggle moved onto B, the flipper's -- the build's own name beside the plain words
        await page.locator('[data-button="A"]').click();
        await editor(page).locator('select[name="button"]').selectOption('B');
        const errs = await errorsOf(page);
        assert.deepEqual(errs.map((x) => x.refusal), ['button_B_does_two_jobs']);
        assert.ok(await editor(page).locator('.lib', { hasText: 'button_B_does_two_jobs' }).isVisible());
        assert.match(await page.locator('[data-button="B"]').getAttribute('class'), /\bbad\b/);
        // B is the button chosen AND refused: it and its label stay red (the accent never hides an error)
        const red = await token(page, '--failed');
        assert.match(await page.locator('[data-button="B"]').getAttribute('class'), /\bsel\b/);
        assert.equal(await page.locator('[data-button="B"] > circle:not(.cbtn-hit)').evaluate((x) => getComputedStyle(x).stroke), red);
        assert.equal(await page.locator('[data-label="B"] rect').evaluate((x) => getComputedStyle(x).stroke), red);
        assert.equal(await page.locator('[data-label="B"] tspan').first().evaluate((x) => getComputedStyle(x).fill), red);
        assert.equal(await page.locator('svg.pad .leader.sel').evaluate((x) => getComputedStyle(x).stroke), red, 'its leader too');
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
        c = await controls(page);
        assert.deepEqual([c.state.controls.list.length, c.state.controls.banner.free.map((r) => r.key)], [7, ['d-pad']], 'one undo takes back the claw and its map line');
        assert.deepEqual(await p.done(), []);
    });

    test('driving feel: the curve and the dead zone move the LIBRARY\'s curve (the readout is shapeAxis), the ramp says its time, and the export writes kFeel', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await feel(page);
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');  // a feel change every build takes
        // ...and the FIRST region (the header's map) changes length: Up gets a job, the 3-line "d-pad FREE" row goes
        await buttons(page);
        await give(page, 'Up', 'claw', 'Go to "open" on one press');
        assert.deepEqual(await errorsOf(page), [], 'one action: the stale map line went with it');
        await page.keyboard.press('Control+e');
        const shown = await page.locator('.drawer pre.code').allTextContents();
        await page.getByRole('button', { name: 'Write into src/team/robot_one_driver.hpp' }).click();
        await page.getByText(/only the three marked parts changed/).waitFor();
        const written = await page.evaluate(() => window.routeBuilder.memory.store['src/team/robot_one_driver.hpp']);
        assert.ok(!/[^\r]\n/.test(written), 'no bare LF written into a CRLF file');
        assert.notEqual(written, crlf);
        assert.equal(outside(written), outside(crlf), 'nothing outside the three marked parts changed');
        assert.deepEqual(Object.values(regionsIn(written)), shown, 'each part between its OWN markers, as the drawer showed it');
        // byte for byte what the two actions wrote before (the job, then "Remove that line"), made here by the model
        const one = ONE();
        let st = readDriverFile(crlf, 'one').state;
        const driving = editDriving(st.driving, { onRelease: 'brake' });
        st = { controls: addControl(followDriving(st.controls, st.driving, driving), CLAW_OPEN, one), driving };
        const [stale] = checkMap(st.controls, driving.rezeroButton);
        st = { ...st, controls: removeFreeLine(st.controls, stale.free) };
        const want = exportRegions(st, one);
        assert.deepEqual(Object.values(regionsIn(written)), REGIONS.map((r) => want[r].join('\n')), 'the same controls, the same bytes');
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
        await feel(page);
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
        await page.getByRole('radio', { name: 'Black Bot' }).click();
        assert.equal(await page.locator('.pad-card h2').textContent(), 'Black Bot\'s controller', 'RMP1-L: the robot by its name');
        assert.match((await controls(page)).meta.locked, /^src\/team\/route_maker\.json's controls for Black Bot cannot be read here \(a "toggles" entry has a field "glow"/);
        // read only: a job chosen, or a button made free, is refused -- the toast says so, never "Added" or "free now"
        const before = (await controls(page)).state.controls.list.length;
        await page.locator('svg.pad .cbtn.free').first().click();
        await editor(page).locator('.choice').first().click();
        await editor(page).locator('.choice').first().click();
        assert.match(await page.locator('.toast').textContent(), /cannot be read here/);
        await page.locator('svg.pad .cbtn.used').first().click();
        await editor(page).getByRole('button', { name: /^Make .* free$/ }).click();
        assert.match(await page.locator('.toast').textContent(), /cannot be read here/);
        assert.equal((await controls(page)).state.controls.list.length, before);
        await page.getByRole('radio', { name: 'White Bot' }).click();
        await give(page, 'Up', 'claw', 'Go to "open" on one press');
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await away(page);
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.getByRole('radio', { name: 'Black Bot' }).click();
        const two = (await controls(page)).state;
        assert.ok(await page.getByRole('button', { name: 'Undo (Ctrl+Z)' }).isDisabled(), 'nothing to undo on robot two');
        await away(page);
        await page.keyboard.press('Control+z');
        assert.deepEqual((await controls(page)).state, two, 'robot two unchanged');
        await page.getByRole('radio', { name: 'White Bot' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake', 'robot one\'s edit still there');
        await away(page);
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), team);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake');
        assert.ok((await page.evaluate(() => window.routeBuilder.route())).start, 'the team\'s route is open');
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Opened src\/team\/route_maker\.json from the folder\. Your Controls changes for White Bot are kept/);
        await buttons(page);
        assert.match(await editor(page).textContent(), /You changed it here, so today's file in your folder is not read in/, 'never "Connect your folder" once it is');
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('hold');
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), both);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.equal((await controls(page)).state.driving.onRelease, 'brake', 'the folder\'s');
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Ctrl\+Z on the Controls tab brings yours back/);
        await away(page);
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
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('hold');
        const asked = page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), team);
        await page.getByRole('alertdialog').waitFor();
        assert.match(await page.getByRole('alertdialog').textContent(), /different project[\s\S]*route and Controls[\s\S]*Ctrl\+Z on the Controls tab/);
        await page.getByRole('button', { name: 'Open the folder\'s route' }).click();
        await asked;
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.deepEqual([(await controls(page)).meta.source, (await controls(page)).state.driving.onRelease], ['folder', 'coast']);
        await away(page);
        await page.keyboard.press('Control+z');
        assert.equal((await controls(page)).state.driving.onRelease, 'hold');
        assert.deepEqual(await p.done(), []);
    });

    test('Controls changed here, then a folder project whose controls for that robot this page cannot read: the change is gone, and the page says so (guide 22\'s one exception)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.getByRole('radio', { name: 'Black Bot' }).click();
        await feel(page);
        await page.locator('select[name="onRelease"]').selectOption('brake');
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { two: { controls: { toggles: [{ at: 0, mechanism: 'claw', button: 'A', glow: 'blue' }] }, driving: { mode: 'arcade' } } };
        const folder = { 'src/team/route_maker.json': `${JSON.stringify(project, null, 2)}\n`, 'src/team/robot_one_driver.hpp': 'int x;\n' };
        await page.evaluate((s) => window.routeBuilder.useMemoryFolder(s), folder);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.match((await page.locator('.toast').allTextContents()).join(' '), /Its Controls for Black Bot are read only here, so your changes there are gone\./);
        assert.doesNotMatch((await page.locator('.toast').allTextContents()).join(' '), /brings yours back|are kept/);
        assert.match((await controls(page)).meta.locked, /cannot be read here/);
        // the editor says why today's file is not read in: never "You changed it here" for a change that is gone
        await buttons(page);
        const pane = async () => editor(page).textContent();
        assert.match(await pane(), /Your folder has no src\/team\/robot_two_driver\.hpp\./);
        assert.doesNotMatch(await pane(), /You changed it here/);
        await away(page);
        await page.keyboard.press('Control+z');
        assert.notEqual((await controls(page)).state.driving.onRelease, 'brake', 'nothing to bring back');
        await page.getByRole('radio', { name: 'White Bot' }).click();
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
            await editor(page).locator(`select[name="${field}"]`).selectOption(to);
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
        const homing = async () => {  // the no-button sentence in the editor's start screen
            await page.getByRole('button', { name: 'Close (Esc)' }).click();
            await page.locator('.ctl-editor .link-line', { hasText: 'homes by itself' }).click();
        };
        await homing();
        const start = errs[0].at;
        const row = (await controls(page)).state.controls.list[start].banner[0].lines;
        await page.getByRole('button', { name: 'Use the page\'s words' }).click();
        assert.deepEqual(await errorsOf(page), []);
        assert.match((await controls(page)).state.controls.list[start].words, /^\[delete-able\]/, 'its comment untouched');
        // undone, the team writes the row TRUE itself: taken
        await away(page);
        await page.keyboard.press('Control+z');
        assert.equal((await errorsOf(page)).length, 1);
        await homing();
        await editor(page).locator('textarea[name="banner0"]').fill(row.join('\n').replaceAll('R1 / R2', 'L1 / L2'));
        await editor(page).locator('textarea[name="banner0"]').blur();
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

    test('G4: the choices say in full what each job picks -- each piston position by name; the second button named, or left to the student when taken', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await page.locator('[data-button="Left"]').click();
        const jobs = async (mech) => {
            await page.locator('.ctl-editor .choice', { hasText: new RegExp(`^${mech}`) }).click();
            const out = await page.locator('.ctl-editor .choice').evaluateAll((els) => els.map((e) => e.getAttribute('aria-label')));
            await page.getByRole('button', { name: 'Another mechanism' }).click();
            return out;
        };
        assert.deepEqual(await jobs('claw'), ['claw: Each press flips it', 'claw: Go to "closed" on one press', 'claw: Go to "open" on one press']);
        assert.ok((await jobs('cascade')).includes('cascade: Move while held: Left up, Right down'));
        await page.keyboard.press('Escape');
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'Give a button a job', 'Escape closes the editor');
        await give(page, 'Right', 'ramp', 'Go to "down" on one press');
        await give(page, 'Left', 'cascade', 'Move while held: Left up, a second button you pick');
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
        await feel(page);
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

    // ── RMP1-D: one step at a time, readable labels, one highlight, nothing cut off ──
    // 1251: the editor under the controller (up to 1260; side by side here, the labels would be 12.9 px); 1261: beside it
    const SIZES = [[1251, 800], [1261, 800], [1280, 800], [1366, 768], [1440, 900]];
    /** Everything the tab draws stays on screen: nothing clipped, nothing under the status bar, no sideways overflow. */
    const unclipped = (page) => page.evaluate(() => {
        const out = [];
        const root = document.querySelector('#controls');
        const bar = document.querySelector('.statusbar').getBoundingClientRect();
        const r = root.getBoundingClientRect();
        if (r.bottom > bar.top + 0.5 || r.right > window.innerWidth + 0.5) {
            out.push(`the tab ends at ${r.bottom} x ${r.right} (the status bar at ${bar.top}, the window ${window.innerWidth} wide)`);
        }
        for (const el of [root, ...root.querySelectorAll('*')]) {
            const st = getComputedStyle(el);
            const clips = (o) => o === 'hidden' || o === 'clip';
            if ((clips(st.overflowY) && el.scrollHeight > el.clientHeight + 1) || (clips(st.overflowX) && el.scrollWidth > el.clientWidth + 1)) {
                out.push(`${el.tagName}.${el.className.baseVal ?? el.className} clips its content`);
            }
            if (st.overflowX !== 'visible' && el.scrollWidth > el.clientWidth + 1 && !el.matches('input, select, textarea')) {
                out.push(`${el.tagName}.${el.className.baseVal ?? el.className} scrolls sideways`);
            }
        }
        if (root.scrollHeight > root.clientHeight + 1) {
            out.push(`the tab itself overflows (${root.scrollHeight} > ${root.clientHeight})`);
        }
        return out;
    });

    test('RMP1-D: two steps, ONE on screen at a time -- Buttons first; the tabs switch by click and by arrow key; a stick opens the feel', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const shown = async () => ({ pad: await page.locator('svg.pad').isVisible(), feel: await page.locator('.feel-card').count(),
                                     sel: await page.locator('.steps-switch [aria-selected="true"]').getAttribute('aria-label') });
        assert.deepEqual(await shown(), { pad: true, feel: 0, sel: 'Step 1: Buttons' });
        await page.getByRole('tab', { name: 'Step 2: Driving feel' }).click();
        assert.deepEqual(await shown(), { pad: false, feel: 1, sel: 'Step 2: Driving feel' });
        assert.equal(await page.locator('svg.pad').count(), 0, 'the controller is not drawn behind the feel');
        assert.equal(await page.locator('.ctl-editor').isVisible(), false);
        await page.keyboard.press('ArrowLeft');
        assert.deepEqual(await shown(), { pad: true, feel: 0, sel: 'Step 1: Buttons' });
        assert.equal(await page.evaluate(() => document.activeElement.id), 'ctl-step-buttons', 'the keyboard stays on the tabs');
        await page.keyboard.press('ArrowRight');
        assert.deepEqual(await shown(), { pad: false, feel: 1, sel: 'Step 2: Driving feel' });
        assert.equal(await page.evaluate(() => document.activeElement.id), 'ctl-step-feel');
        await page.keyboard.press('ArrowLeft');
        await page.getByRole('button', { name: /^Left stick/ }).click();
        assert.deepEqual(await shown(), { pad: false, feel: 1, sel: 'Step 2: Driving feel' });
        assert.equal(await page.locator('#ctl-body').getAttribute('aria-labelledby'), 'ctl-step-feel');
        assert.deepEqual(await p.done(), []);
    });

    for (const [width, height] of SIZES) {
        test(`RMP1-D [${width} x ${height}]: every label on the controller is at least 13 px and none overlaps another; nothing in either step is cut off`, async () => {
            const p = await openPage(browser, site.base, { viewport: { width, height } });
            const { page } = p;
            await openControls(page);
            const { labels, k, names, shapes, padBody } = await page.evaluate(() => {
                const svg = document.querySelector('svg.pad');
                const k = svg.getScreenCTM().a;  // the drawing's scale: a font in its units is k times as big on the screen
                const box = svg.getBoundingClientRect();
                const r = (x) => x.getBoundingClientRect().toJSON();
                return { k, padBody: r(svg.querySelector('.pad-body')),
                    names: [...svg.querySelectorAll('.cbtn-name')].map((t) => parseFloat(getComputedStyle(t).fontSize) * k),
                    shapes: [...svg.querySelectorAll('.cbtn > rect:not(.cbtn-hit), .cbtn > circle:not(.cbtn-hit), .stick-base')].map((x) => ({ id: x.closest('[data-button]')?.dataset.button || 'a stick', ...r(x) })),
                    labels: [...svg.querySelectorAll('.callout')].map((g) => ({
                        text: g.textContent, rect: r(g.querySelector('rect')), words: r(g.querySelector('text')),
                        px: Math.min(...[...g.querySelectorAll('tspan')].map((t) => parseFloat(getComputedStyle(t).fontSize) * k)), box: box.toJSON() })) };
            });
            assert.equal(labels.length, 14, 'twelve buttons and two sticks, each labelled');
            assert.ok(Math.min(...names) >= 13, `the letters on the buttons are drawn at ${Math.min(...names).toFixed(2)} px`);
            const apart = (a, b) => a.right <= b.left || b.right <= a.left || a.bottom <= b.top || b.bottom <= a.top;
            for (const l of labels) {
                assert.ok(l.px >= 13, `"${l.text}" is drawn at ${l.px.toFixed(2)} px`);
                const inside = (a, b) => a.left >= b.left - 0.5 && a.right <= b.right + 0.5 && a.top >= b.top - 0.5 && a.bottom <= b.bottom + 0.5;
                assert.ok(inside(l.words, l.rect), `"${l.text}": its words fit its pill`);
                // the pill hugs its words (measured, 14 units each side), never the rough guess of 10 units a letter
                assert.ok(l.rect.width - l.words.width <= 28 * k + 2, `"${l.text}": its pill is ${(l.rect.width - l.words.width).toFixed(1)} px wider than its words`);
                assert.ok(inside(l.rect, l.box), `"${l.text}": inside the drawing (${JSON.stringify(l.rect)})`);
                assert.ok(apart(l.rect, padBody), `"${l.text}": off the controller's body`);
                for (const x of shapes) {
                    assert.ok(apart(l.rect, x), `"${l.text}" covers ${x.id}`);
                }
            }
            for (const [i, a] of labels.entries()) {
                for (const b of labels.slice(i + 1)) {
                    assert.ok(apart(a.rect, b.rect), `"${a.text}" overlaps "${b.text}"`);
                }
            }
            assert.deepEqual(await unclipped(page), [], 'Buttons, nothing chosen');
            await page.locator('[data-button="R1"]').click();
            // the editor clicked is in view whole (stacked under the controller, the click scrolls it in)
            const [ed, view] = [await editor(page).boundingBox(), await page.locator('#ctl-body').boundingBox()];
            assert.ok(ed.y >= view.y - 0.5 && ed.y + ed.height <= view.y + view.height + 0.5, `the editor in view: ${JSON.stringify({ ed, view })}`);
            assert.deepEqual(await unclipped(page), [], 'Buttons, the cascade\'s two-button form');
            await page.locator('.ctl-editor details.fold > summary').click();
            assert.deepEqual(await unclipped(page), [], 'Buttons, its words open');
            await feel(page, false);
            assert.deepEqual(await unclipped(page), [], 'Driving feel');
            const body = await page.locator('#ctl-body').evaluate((b) => [b.scrollHeight, b.clientHeight]);
            assert.ok(body[0] <= body[1] + 1, `the essentials fit without scrolling: ${body}`);
            await feel(page);
            assert.deepEqual(await unclipped(page), [], 'Driving feel, More settings open');
            const last = page.getByRole('button', { name: 'Use the library\'s defaults' });
            await last.scrollIntoViewIfNeeded();
            const [l, b] = [await last.boundingBox(), await page.locator('#ctl-body').boundingBox()];
            assert.ok(l.y >= b.y && l.y + l.height <= b.y + b.height + 0.5, `the last setting can be scrolled into view: ${JSON.stringify({ l, b })}`);
            assert.deepEqual(await p.done(), []);
        });
    }

    test('RMP1-D: a click opens ONE editor with the button\'s job; the selected button is the controller\'s only accent; a new job is drawn at once; Undo brings the old one back; Esc closes', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const label = (b) => page.locator(`[data-button="${b}"]`).getAttribute('aria-label');
        const tones = [await token(page, '--accent'), await token(page, '--accent-strong')];
        /** Everything on the controller painted in the accent (any shape, line or word), counted by what it belongs to. */
        const accent = () => page.evaluate((want) => {
            const out = {};
            for (const x of document.querySelectorAll('svg.pad *')) {
                const st = getComputedStyle(x);
                if (want.includes(st.fill) || want.includes(st.stroke)) {
                    const b = x.closest('[data-button]');
                    const who = b ? `button ${b.dataset.button}` : x.closest('.callout') ? `label ${x.closest('.callout').dataset.label}`
                        : x.matches('.leader') ? 'leader' : x.closest('.stick') ? 'stick' : x.tagName;
                    out[who] = (out[who] || 0) + (who === 'leader' ? 1 : 0);
                }
            }
            return out;
        }, tones);
        assert.deepEqual(await accent(), {}, 'nothing chosen: no accent on the controller');
        await page.locator('[data-label="X"]').click();  // the label opens it too
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'ramp');
        assert.match(await editor(page).locator('.ed-head').textContent(), /^X\s*ramp\s*Each press flips it/);
        assert.deepEqual(await accent(), { 'button X': 0, 'label X': 0, leader: 1 }, 'the one button chosen, its label and its one leader');
        // Esc closes it though a label click leaves the keyboard on the page (D4; guide 22: "Esc or × closes it")
        assert.equal(await page.evaluate(() => document.activeElement.tagName), 'BODY');
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'Give a button a job');
        assert.deepEqual(await accent(), {});
        // Esc over the Export drawer closes only the drawer
        await page.locator('[data-label="X"]').click();
        await page.keyboard.press('Control+e');
        await page.keyboard.press('Escape');
        assert.equal(await page.locator('.drawer').count(), 0);
        assert.equal(await title(page), 'ramp', 'the editor behind it stays');
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'Give a button a job');
        // Esc while typing in a box keeps the editor and the words typed
        await page.locator('[data-button="L1"]').click();
        await editor(page).locator('input[name="volts"]').fill('9');
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'right toggle', 'Esc in a text box closes nothing');
        assert.equal(await editor(page).locator('input[name="volts"]').inputValue(), '9');
        // its words fold, opened and re-drawn in the same task (a typed volt), stays open (Chromium's toggle came late)
        await page.evaluate(() => {
            document.querySelector('.ctl-editor details.fold > summary').click();
            const box = document.querySelector('.ctl-editor input[name="volts"]');
            box.value = '10';
            box.dispatchEvent(new Event('change'));
        });
        assert.equal((await controls(page)).state.controls.list[4].volts, 10, 're-drawn');
        assert.equal(await page.locator('.ctl-editor details.fold').evaluate((d) => d.open), true, 'still open after the re-draw');
        await away(page);
        await page.keyboard.press('Control+z');
        await page.locator('[data-label="X"]').click();
        await page.getByRole('button', { name: 'Make X free' }).click();
        assert.equal(await label('X'), 'X: free');
        await page.locator('[data-button="X"]').click();
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'X is free');
        await give(page, 'X', 'claw', 'Go to "open" on one press');
        assert.equal(await page.evaluate(() => document.activeElement.tagName), 'BODY', 'the choice clicked is drawn away');
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'Give a button a job', 'Esc closes it after a job is chosen with the mouse');
        assert.equal(await label('X'), 'X: claw: to open', 'the controller shows the new job');
        assert.match(await page.locator('[data-label="X"]').textContent(), /X · claw\s*to open/);
        await away(page);
        await page.keyboard.press('Control+z');
        assert.equal(await label('X'), 'X: free');
        await page.keyboard.press('Control+z');
        assert.equal(await label('X'), 'X: ramp: toggle', 'Undo brings the old job back');
        // the keyboard: Enter on a free button opens its choices with the keyboard on the first; Esc closes, back on the button
        await page.locator('[data-button="Down"]').focus();
        await page.keyboard.press('Enter');
        assert.equal(await page.evaluate(() => document.activeElement.className), 'choice');
        await page.keyboard.press('Escape');
        assert.equal(await editor(page).locator('.insp-title').textContent(), 'Give a button a job');
        assert.equal(await page.evaluate(() => document.activeElement.dataset.button), 'Down');
        assert.deepEqual(await accent(), {});
        // a mechanism from the list: Esc closes it too; its "job with no button" is added at once
        await page.getByRole('button', { name: 'The cascade', exact: true }).click();
        await page.keyboard.press('Escape');
        assert.equal(await title(page), 'Give a button a job');
        await page.getByRole('button', { name: 'The cascade', exact: true }).click();
        await editor(page).getByRole('button', { name: 'No button: autons only', exact: true }).click();
        const last = (await controls(page)).state.controls.list.at(-1);
        assert.deepEqual([last.form, last.mechanism, last.button], ['autonOnly', 'cascade', undefined]);
        assert.match(await editor(page).locator('.ed-head').textContent(), /cascade/);
        // the re-zero button: its one way out opens the feel, with every setting in view
        await page.locator('[data-button="Y"]').click();
        assert.equal(await title(page), 'Y is the re-zero button');
        await page.getByRole('button', { name: 'Open Driving feel' }).click();
        assert.equal(await page.locator('.steps-switch [aria-selected="true"]').getAttribute('aria-label'), 'Step 2: Driving feel');
        assert.ok(await page.locator('select[name="rezeroButton"]').isVisible(), 'More settings open: the re-zero button in view');
        assert.deepEqual(await p.done(), []);
    });

    test('RMP1-D: the feel shows the essentials first -- style, dead zone, curve, the graph; "More settings" is closed and holds every other setting', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        await feel(page, false);
        const visible = async (sel) => page.locator(sel).isVisible();
        assert.equal(await page.locator('details.feel-more').evaluate((d) => d.open), false, 'closed at first');
        for (const sel of ['[role="radiogroup"][aria-label="Drive mode"]', 'input[name="feel-deadband"]', 'input[name="feel-expo"]', 'svg.graph']) {
            assert.ok(await visible(sel), sel);
        }
        const more = ['input[name="feel-ramp"]', 'input[name="feel-ramp-slider"]', 'select[name="onRelease"]', 'select[name="rezeroButton"]', '[role="tablist"][aria-label="Which stick axis"]'];
        for (const sel of more) {
            assert.equal(await visible(sel), false, `${sel} waits under More settings`);
        }
        // opened, then re-drawn at once (a typed curve, before the browser's 'toggle' event): it stays open (Chromium closed it)
        await page.evaluate(() => {
            document.querySelector('details.feel-more > summary').click();
            const box = document.querySelector('input[name="feel-expo"]');
            box.value = '0.2';
            box.dispatchEvent(new Event('change'));
        });
        assert.equal((await controls(page)).state.driving.forward.curve, 0.2, 're-drawn');
        assert.equal(await page.locator('details.feel-more').evaluate((d) => d.open), true, 'still open after the re-draw');
        for (const sel of more) {
            assert.ok(await visible(sel) && await page.locator(sel).isEnabled(), sel);
        }
        // the axis picked there is the one the essentials shape, and it is said above them
        await page.getByRole('tab', { name: 'Turning' }).click();
        assert.equal(await page.locator('.axis-title').textContent(), 'Turning');
        assert.equal(await page.locator('details.feel-more').evaluate((d) => d.open), true, 'a re-draw keeps it open');
        await page.locator('input[name="feel-expo"]').fill('0.4');
        await page.locator('input[name="feel-expo"]').press('Enter');
        assert.deepEqual((await controls(page)).state.driving.turn, { curve: 0.4 });
        assert.ok(await page.getByRole('button', { name: 'Use the library\'s defaults' }).isVisible());
        // Export's "Go there" on a setting under More settings opens it, the setting in view
        await page.locator('input[name="feel-ramp"]').fill('-1');
        await page.locator('input[name="feel-ramp"]').press('Enter');
        await page.locator('details.feel-more > summary').click();
        assert.equal(await page.locator('details.feel-more').evaluate((d) => d.open), false);
        await buttons(page);
        await page.keyboard.press('Control+e');
        await page.locator('.drawer li', { hasText: 'The ramp must be' }).getByRole('button', { name: 'Go there' }).click();
        assert.equal(await page.locator('details.feel-more').evaluate((d) => d.open), true);
        assert.ok(await page.locator('input[name="feel-ramp"]').isVisible());
        assert.deepEqual(await p.done(), []);
    });

    test('RMP1-D: a map line left stale in a saved project -- Export\'s "Go there" opens the job that took its button (never "Up is free"), its fix beside it', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const r = readDriverFile(MARKED, 'one');
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { one: { controls: toSaved(addControl(r.state.controls, CLAW_OPEN, ONE())), driving: r.state.driving } };
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
        await page.getByRole('tab', { name: 'Controls' }).click();
        assert.deepEqual((await errorsOf(page)).map((x) => [x.code, x.button]), [['mapStale', 'Up']]);
        // another edit leaves it: only a line THIS edit makes stale goes by itself
        await page.locator('[data-button="L1"]').click();
        await editor(page).locator('input[name="volts"]').fill('10');
        await editor(page).locator('input[name="volts"]').press('Enter');
        assert.equal((await controls(page)).state.controls.list[4].volts, 10);
        assert.deepEqual((await errorsOf(page)).map((x) => [x.code, x.button]), [['mapStale', 'Up']]);
        await page.keyboard.press('Control+e');
        await page.locator('.drawer').getByRole('button', { name: 'Go there' }).click();
        assert.equal(await title(page), 'claw', 'the job on Up, not "Up is free"');
        assert.match(await editor(page).locator('.ed-head').textContent(), /^Up\s*claw/);
        assert.equal(await editor(page).locator('.choice').count(), 0, 'no mechanism offered for a button that has a job');
        assert.match(await page.locator('[data-button="Up"]').getAttribute('class'), /\bsel\b/);
        await editor(page).getByRole('button', { name: 'Remove that line' }).click();
        assert.deepEqual(await errorsOf(page), []);
        assert.deepEqual(await p.done(), []);
    });

    test('RMP1-D: every edit that takes a button the map calls free removes that line in the same edit and says so -- a job moved by its Button field, the re-zero moved, two lines at once', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await openControls(page);
        const free = async () => (await controls(page)).state.controls.banner.free.map((r) => r.key);
        const said = async () => {
            const t = await page.locator('.toast').textContent({ timeout: 5000 });
            await page.evaluate(() => document.querySelector('.toast').remove());  // the next check reads a new one
            return t;
        };
        const WENT = 'The button map\'s "d-pad" line said free, so it went too (Ctrl+Z undoes both).';
        // the claw's button moved from A onto Left
        await page.locator('[data-button="A"]').click();
        await editor(page).locator('select[name="button"]').selectOption('Left');
        assert.deepEqual([(await controls(page)).state.controls.list[1].button, await free(), await errorsOf(page)], ['Left', [], []]);
        assert.equal(await said(), WENT);
        await away(page);
        await page.keyboard.press('Control+z');
        assert.deepEqual([(await controls(page)).state.controls.list[1].button, await free()], ['A', ['d-pad']], 'one Ctrl+Z: the move and the line');
        // the re-zero moved onto Down (Driving feel)
        await feel(page);
        await page.locator('select[name="rezeroButton"]').selectOption('Down');
        assert.deepEqual([(await controls(page)).state.driving.rezeroButton, await free(), await errorsOf(page)], ['Down', [], []]);
        assert.equal(await said(), WENT);
        await away(page);
        await page.keyboard.press('Control+z');
        assert.deepEqual([(await controls(page)).state.driving.rezeroButton, await free()], ['Y', ['d-pad']], 'one Ctrl+Z: the re-zero and the line');
        // a hand-written map with three free lines: a job on Up and Down takes two of them in one edit, the third stays
        const r = readDriverFile(MARKED, 'one');
        const lines = ['Up', 'Down', 'Left / Right'].map((key) => ({ key, lines: ['FREE'] }));
        const project = JSON.parse(fs.readFileSync(path.join(REPO, 'test', 'goldens', 'route_maker', 'smoke_robot_one.route.json'), 'utf8'));
        project.robots = { one: { controls: toSaved({ ...r.state.controls, banner: { ...r.state.controls.banner, free: lines } }), driving: r.state.driving } };
        await page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
        await page.getByRole('tab', { name: 'Controls' }).click();
        await buttons(page);
        assert.deepEqual([await free(), await errorsOf(page)], [['Up', 'Down', 'Left / Right'], []]);
        await page.locator('.toast').evaluateAll((ts) => ts.forEach((t) => t.remove()));
        await give(page, 'Up', 'right toggle', 'Run while held: Up forward, Down back');
        assert.deepEqual(await free(), ['Left / Right'], 'the two lines it took, not the third');
        assert.deepEqual((await errorsOf(page)).filter((x) => x.code === 'mapStale'), []);
        assert.equal(await said(), 'Added: Up and Down. Change either, and type its volts, in the panel. '
            + 'The button map\'s "Up" and "Down" lines said free, so they went too (Ctrl+Z undoes both).');
        await away(page);
        await page.keyboard.press('Control+z');
        assert.deepEqual([(await controls(page)).state.controls.list.length, await free()], [7, ['Up', 'Down', 'Left / Right']], 'one Ctrl+Z: the job and both lines');
        assert.deepEqual(await p.done(), []);
    });

    test('RMP1-D: the graph\'s dot follows EVERY drag (none selects the graph\'s words); a feel number typed keeps the step\'s scroll (the tutorial\'s findings)', async () => {
        const p = await openPage(browser, site.base, { viewport: { width: 1280, height: 800 } });
        const { page } = p;
        await openControls(page);
        await feel(page, false);
        const g = await page.locator('svg.graph').boundingBox();
        const at = (f) => Math.round((((f * 360 - 46) / 300) * 200 - 100));  // the stick (%) under a point f of the graph's width
        const drag = async (from, to) => {
            await page.mouse.move(g.x + g.width * from, g.y + g.height * 0.5);
            await page.mouse.down();
            for (let i = 1; i <= 8; ++i) {
                await page.mouse.move(g.x + g.width * (from + ((to - from) * i) / 8), g.y + g.height * 0.5);
            }
            await page.mouse.up();
            return Number(await page.locator('.g-probe').getAttribute('aria-valuenow'));
        };
        for (const [from, to] of [[0.55, 0.8], [0.8, 0.3], [0.3, 0.9], [0.9, 0.2]]) {
            const got = await drag(from, to);
            assert.ok(Math.abs(got - at(to)) <= 1, `a drag to ${to} of the graph ended at ${got} % (want ${at(to)})`);
            assert.equal(await page.evaluate(() => String(getSelection())), '', 'no words selected');
        }
        // a drag the browser cancels lets the dot go: moving on does not move it
        await page.mouse.move(g.x + g.width * 0.5, g.y + g.height * 0.5);
        await page.mouse.down();
        await page.evaluate(() => window.dispatchEvent(new PointerEvent('pointercancel')));
        const kept = Number(await page.locator('.g-probe').getAttribute('aria-valuenow'));
        await page.mouse.move(g.x + g.width * 0.85, g.y + g.height * 0.5, { steps: 4 });
        await page.mouse.up();
        assert.equal(Number(await page.locator('.g-probe').getAttribute('aria-valuenow')), kept, 'cancelled: the dot stays');
        // the step scrolled down (More open), then a curve typed: the scroll stays (it jumped to the top)
        await page.locator('details.feel-more > summary').click();
        const body = page.locator('#ctl-body');
        const top = await body.evaluate((b) => { b.scrollTop = b.scrollHeight; return b.scrollTop; });
        assert.ok(top > 0, 'the step scrolls at this size, More open');
        await page.locator('input[name="feel-expo"]').fill('0.6');
        await page.locator('input[name="feel-expo"]').press('Enter');
        assert.deepEqual((await controls(page)).state.driving.forward, { curve: 0.6 }, 're-drawn');
        assert.equal(await body.evaluate((b) => b.scrollTop), top);
        assert.deepEqual(await p.done(), []);
    });

    // ── the screenshots (ROUTE_MAKER_SHOTS): both steps, light and dark, 1440 and 1280 ──
    const SHOT_VARIANTS = VARIANTS.length > 1 ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]]
        .map(([scheme, w, hh]) => ({ scheme, viewport: { width: w, height: hh }, tag: `${scheme}-${w}` })) : VARIANTS;
    for (const v of SHOT_VARIANTS) {
        test(`RMP1-D [${v.tag}] the controller; a free button's choices; a refusal; the driving feel at a curve of 0.6; the export`, async () => {
            const p = await openPage(browser, site.base, v);
            const { page } = p;
            await openControls(page);
            await shot(page, `controls-${v.tag}`);
            await page.locator('[data-button="Up"]').click();
            await page.locator('.ctl-editor .choice', { hasText: /^claw/ }).click();
            await shot(page, `controls-menu-${v.tag}`);
            await page.keyboard.press('Escape');
            await page.locator('[data-button="A"]').click();
            await editor(page).locator('select[name="button"]').selectOption('B');
            await shot(page, `controls-refused-${v.tag}`, true);
            await away(page);
            await page.keyboard.press('Control+z');
            await feel(page, false);
            await page.locator('input[name="feel-expo"]').fill('0.6');
            await page.locator('input[name="feel-expo"]').press('Enter');
            await shot(page, `feel-${v.tag}`, true);
            await away(page);
            await page.keyboard.press('Control+z');  // back to the file's map: the export shows its three parts
            await buttons(page);
            await page.keyboard.press('Control+e');
            await page.locator('.drawer pre.code').nth(2).waitFor();
            await page.locator('.drawer').waitFor();
            await shot(page, `controls-export-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });
    }
});
