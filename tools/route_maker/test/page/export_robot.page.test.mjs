// export_robot.page.test.mjs — RMP1-F in a browser: Export shows THIS browser's step and only it (D1), a write is read
// back and the result is never silent (D2), "Will it build?" (D3) and Next with robot one's and robot two's commands
// (D4). Every fault is the test's (a write the folder changes, markers gone, a refusal). ROUTE_MAKER_SHOTS: the screens,
// light and dark, at 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, SHOTS, BROWSER, skip, launch, serve, openPage, shot } from './harness.mjs';
import { saveProject, newProject } from '../../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../../src/model/route.mjs';
import { robotCommands } from '../../src/model/to_robot.mjs';
import { GOLDENS } from '../helpers.mjs';

const FILE = 'src/team/skills_route.cpp';
const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\r\n\r\nnamespace shulib::bench {\r\n\r\n${EMPTY.replace(/\n/g, '\r\n')}\r\n}  // namespace shulib::bench\r\n`;
const LINES = { 'src/team/autons.cpp': 'addTeamRoutes(registry, TeamRobot::Two);', 'src/team/robot_one_autons.cpp': 'addTeamRoutes(registry, TeamRobot::One);' };
const at = (r, x, y) => setStart({ ...r, together: false }, { x, y, headingDeg: 90 });
const SKILLS_ROUTE = addStep(addStep(at(newRoute(), -36, -60), { do: 'driveTo', x: -36, y: -36 }), { do: 'wait', seconds: 1 });
const PROJECT = saveProject({ ...newProject('the test'), routes: [SKILLS_ROUTE, addStep(at(newRoute('left 1'), -36, -60), { do: 'wait', seconds: 2 })] });
const ONE_ROUTE = saveProject({ ...newProject('the test'), routes: [SKILLS_ROUTE] });
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));

let browser;
let site;
const drawer = (page) => page.locator('.drawer');
const result = (page) => page.locator('.drawer .xr-result');

/** The page with the test's two routes (or `project`) opened from a file (the page's own Open), checked clean. */
async function opened(v, project = PROJECT) {
    const p = await openPage(browser, site.base, v);
    await p.page.getByRole('button', { name: 'Got it' }).click();
    const [fc] = await Promise.all([p.page.waitForEvent('filechooser'), p.page.getByRole('button', { name: 'Open a project' }).click()]);
    await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(project, 'utf8') });
    await p.page.getByText('Opened the project.').waitFor();
    if (SHOTS) {
        await p.page.locator('.toast').waitFor({ state: 'detached', timeout: 10000 });  // the pictures without it
    }
    const errors = await p.page.evaluate(() => window.routeBuilder.issues().filter((x) => x.level === 'error'));
    assert.deepEqual(errors, [], 'the test\'s routes are ready to export');
    // the clipboard the test reads (every browser: the page's Copy uses the Clipboard API on a secure page)
    await p.page.evaluate(() => {
        window.__copied = [];
        Object.defineProperty(navigator.clipboard, 'writeText', { value: async (t) => { window.__copied.push(t); }, configurable: true });
    });
    return p;
}
async function openExport(page) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('Control+e');
    await drawer(page).waitFor();
}
const folder = (page, files) => page.evaluate((f) => window.routeBuilder.useMemoryFolder(f), files);
/** The browser refuses every copy (the Clipboard API and the old way both). */
const refuseCopies = (page) => page.evaluate(() => {
    Object.defineProperty(navigator.clipboard, 'writeText', { value: async () => { throw new DOMException('Write permission denied.', 'NotAllowedError'); } });
    document.execCommand = () => false;
});
const code = (page) => page.locator('.drawer pre.code').textContent();
const regionOf = (text) => text.slice(text.indexOf('// ═══ ROUTE MAKER'), text.lastIndexOf('═') + 1).replace(/\r\n/g, '\n');

test.describe('RMP1-F: export to the robot', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), 'build the site first (tools/route_maker/build.py)');
        site = await serve();
        browser = await launch();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] D1: this browser's step and ONLY it -- Chromium the folder, Firefox the download -- one highlighted button`, async () => {
            const p = await opened(v);
            const { page } = p;
            await openExport(page);
            const folderWay = await page.evaluate(() => typeof window.showDirectoryPicker === 'function' && window.isSecureContext);
            if (BROWSER === 'chromium' || BROWSER === 'firefox') {
                assert.equal(folderWay, BROWSER === 'chromium', `${BROWSER}: the File System Access API ${folderWay ? 'present' : 'absent'}`);
            }
            const d = drawer(page);
            assert.equal(await d.getByRole('button', { name: 'Connect your shulib folder' }).count(), folderWay ? 1 : 0);
            assert.equal(await d.getByRole('button', { name: `Write into ${FILE}` }).count(), folderWay ? 1 : 0);
            assert.equal(await d.getByRole('button', { name: 'Pick skills_route.cpp and download the updated one' }).count(), folderWay ? 0 : 1);
            assert.equal(await d.getByText(/Chrome or Edge: the page writes it/).count(), folderWay ? 1 : 0);
            assert.equal(await d.getByText(/This browser cannot write into a folder/).count(), folderWay ? 0 : 1);
            assert.equal(await d.locator('.btn.primary').count(), 1, 'one highlighted step');
            assert.equal(await d.locator('.xr-step.now').count(), 1);
            if (folderWay) {
                assert.ok(await d.getByRole('button', { name: `Write into ${FILE}` }).isDisabled(), 'step 2 waits for step 1');
            } else {
                assert.ok(await d.getByRole('button', { name: 'Copy the path' }).isVisible(), 'step 2: the exact path, with Copy');
            }
            assert.equal(await d.locator('details.xr-code').getAttribute('open'), null, 'the code is one click away, not a wall');
            await shot(page, `xr-step-${v.tag}`, true);
            assert.deepEqual(await p.done(), []);
        });

        test(`[${v.tag}] D2-D4: Write, READ BACK, the green card names the routes and steps; Will it build?; Next for robot one and two`, async () => {
            const p = await opened(v);
            const { page } = p;
            await folder(page, { [FILE]: SKILLS, ...LINES });
            await openExport(page);
            assert.equal(await drawer(page).locator('.xr-step.done').count(), 1, 'step 1 done: connected');
            const primary = drawer(page).locator('.btn.primary');
            assert.equal(await primary.count(), 1, 'one highlight once connected');
            assert.equal(await primary.textContent(), `Write into ${FILE}`);
            await page.getByRole('button', { name: `Write into ${FILE}` }).click();
            const card = result(page).locator('[data-result="ok"]');
            await card.waitFor();
            assert.equal(await card.locator('.xr-title').textContent(), `Written and checked: ${FILE}`);
            assert.equal(await card.locator('.xr-sum').textContent(), '2 routes (skills, left 1), 3 steps');
            assert.match(await card.locator('.xr-build').textContent(), /Will it build\?.*Every step follows the library's rules.*\(read back after writing\).*The C\+\+ compiler checks the rest/);
            assert.equal(regionOf(await page.evaluate((f) => window.routeBuilder.memory.store[f], FILE)), await code(page), 'the file has exactly the code');
            assert.equal(await drawer(page).locator('.xr-steps').count(), 0, 'the steps have done their job');
            assert.match(await card.locator('.xr-build').textContent(), /Both robots register every route/);
            const next = drawer(page).locator('.xr-next');
            const cmd = (what) => next.locator(`code[data-cmd="${what}"]`).textContent();
            const windows = await page.evaluate(() => /Windows/.test(navigator.userAgent));  // CI's Windows rows: `python`
            assert.equal(await cmd('the build command'), 'make ROBOT=one PROGRAM=autontest');
            assert.equal(await cmd('the upload command'), 'pros upload --slot 6 --name "shulib Auton Tester"');
            assert.equal(await cmd('the one command'), robotCommands('one', { windows }).helper);
            assert.match(robotCommands('one', { windows }).helper, windows ? /^python tools/ : /^python3 tools/);
            assert.match(await next.textContent(), /In the PROS terminal, in your shulib folder.*needs Python 3, from python\.org; without it, use 1 and 2/);
            assert.match(await next.textContent(), /On the brain: shulib Auton Tester \(slot 6\) → one of: skills, left 1 → RUN/);
            assert.match(await next.textContent(), /make ends with no line saying "error".*pros says "Finished uploading"/);
            await shot(page, `xr-written-${v.tag}`, true);
            await next.getByRole('button', { name: 'Robot two' }).click();
            assert.equal(await cmd('the build command'), 'make ROBOT=tank PROGRAM=autontest');
            assert.equal(await cmd('the one command'), robotCommands('two', { windows }).helper);
            await next.getByRole('button', { name: 'Copy the build command' }).click();
            assert.deepEqual(await page.evaluate(() => window.__copied), ['make ROBOT=tank PROGRAM=autontest']);
            assert.deepEqual(await p.done(), []);
        });

        test(`[${v.tag}] D2: a write that reads back different is RED, never green; Write again after the fix`, async () => {
            const p = await opened(v);
            const { page } = p;
            await folder(page, { [FILE]: SKILLS, ...LINES });
            await page.evaluate(() => {
                const m = window.routeBuilder.memory;
                window.__write = m.writeText;
                m.writeText = async (f, t) => {
                    m.store[f] = t.replace('Robot one registers', 'Robot 1 registers');  // something changed it as it was written
                };
            });
            await openExport(page);
            await page.getByRole('button', { name: `Write into ${FILE}` }).click();
            const card = result(page).locator('[data-result="error"]');
            await card.waitFor();
            assert.equal(await card.locator('.xr-title').textContent(), 'Written, but it read back different', 'the file DID change');
            assert.match(await card.textContent(), /not what the page wrote.*The fix: close the file in your editor/);
            assert.equal(await drawer(page).locator('.xr-next').count(), 0, 'no Next after a failure');
            assert.equal(await drawer(page).locator('.btn.primary').count(), 1, 'one highlight: the fix');
            assert.equal(await card.locator('.btn.primary').count(), 1);
            assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']), undefined, 'not marked exported, nothing saved');
            await shot(page, `xr-read-back-differs-${v.tag}`, true);
            await page.evaluate(() => {
                window.routeBuilder.memory.writeText = window.__write;
            });
            // what is in the file now is not the builder's: Write again ASKS (never a silent overwrite), Overwrite writes
            await card.getByRole('button', { name: 'Write again' }).click();
            await result(page).locator('[data-result="warn"]').getByRole('button', { name: 'Overwrite' }).click();
            await result(page).locator('[data-result="ok"]').waitFor();
            assert.equal(regionOf(await page.evaluate((f) => window.routeBuilder.memory.store[f], FILE)), await code(page));
            assert.deepEqual(await p.done(), []);
        });
    }

    test('D2: markers gone before the write, and a folder that refuses: red, each with its fix; a hand edit kept: red', async () => {
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await folder(page, { [FILE]: SKILLS, ...LINES });
        await openExport(page);
        await page.evaluate((f) => {
            window.routeBuilder.memory.store[f] = 'int x;\n';
        }, FILE);
        await page.getByRole('button', { name: `Write into ${FILE}` }).click();
        let card = result(page).locator('[data-result="error"]');
        await card.waitFor();
        assert.equal(await card.locator('.xr-title').textContent(), `Not written into ${FILE}`);
        assert.match(await card.textContent(), /no ROUTE MAKER lines.*someone removed them.*The fix: ask the coordinator to put back its one pair of ROUTE MAKER lines \(git checkout src\/team\/skills_route\.cpp/);
        assert.doesNotMatch(await card.textContent(), /robot two's update|use Copy/, 'that update has landed; Copy pastes over lines that are not there');
        assert.equal(await drawer(page).getByRole('button', { name: 'Copy the code' }).count(), 0, 'nowhere: Copy pastes over lines that are not there');
        await page.keyboard.press('Escape');
        // the same file when the drawer opens: the banner says so, with the same fix, Write waits, no Copy, focus on the fix
        const fixFirst = async (words) => {
            await openExport(page);
            assert.match(await drawer(page).locator('.xr-steps .banner.warn').textContent(), words);
            assert.ok(await drawer(page).getByRole('button', { name: `Write into ${FILE}` }).isDisabled());
            assert.equal(await drawer(page).getByRole('button', { name: 'Copy the code' }).count(), 0);
            assert.equal(await drawer(page).locator('.btn.primary').count(), 0);
            assert.ok(await page.evaluate(() => document.activeElement.matches('.xr-steps .banner.warn')), 'focus on the fix');
            await page.keyboard.press('Escape');
        };
        await fixFirst(/has no ROUTE MAKER lines: someone removed them\.The fix: ask the coordinator/);
        await folder(page, { [FILE]: SKILLS + SKILLS, ...LINES });
        await fixFirst(/has 2 BEGIN and 2 END ROUTE MAKER lines.*The fix: ask the coordinator/);
        // no skills-route file at all (an old copy): the banner says Copy, so Copy is offered
        await folder(page, { ...LINES });
        await openExport(page);
        assert.match(await drawer(page).locator('.xr-steps .banner.warn').textContent(), /not in this folder yet.*use Copy/);
        assert.ok(await page.evaluate(() => document.activeElement.textContent === 'Copy the code'));
        await page.keyboard.press('Escape');
        // the folder refuses the write (permission taken back)
        await folder(page, { [FILE]: SKILLS, ...LINES });
        await page.evaluate(() => {
            window.routeBuilder.memory.writeText = async () => {
                throw new DOMException('The request is not allowed by the user agent', 'NotAllowedError');
            };
        });
        await openExport(page);
        await page.getByRole('button', { name: `Write into ${FILE}` }).click();
        card = result(page).locator('[data-result="error"]');
        await card.waitFor();
        assert.match(await card.textContent(), /The folder refused: The request is not allowed.*The fix: connect your shulib folder again/);
        assert.ok(await card.getByRole('button', { name: 'Connect your shulib folder' }).isVisible());
        await page.keyboard.press('Escape');
        // a hand edit: Keep their change -> red, nothing written
        const hand = SKILLS.replace('NO STEPS YET', 'TUNED BY HAND');
        await folder(page, { [FILE]: hand, ...LINES });
        await openExport(page);
        await page.getByRole('button', { name: `Write into ${FILE}` }).click();
        await page.getByRole('button', { name: 'Keep their change' }).click();
        card = result(page).locator('[data-result="error"]');
        assert.equal(await card.locator('.xr-title').textContent(), 'Not written: their change was kept');
        assert.equal(await page.evaluate((f) => window.routeBuilder.memory.store[f], FILE), hand);
        assert.deepEqual(await p.done(), []);
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] D2: Copy says where it goes, with the picture of the two lines; then Next`, async () => {
            const p = await opened(v);
            const { page } = p;
            await openExport(page);
            await drawer(page).locator('.xr-steps').getByRole('button', { name: 'Copy the code' }).click();
            const card = result(page).locator('[data-result="ok"]');
            await card.waitFor();
            assert.equal(await card.locator('.xr-title').textContent(), 'Copied');
            assert.match(await card.textContent(), /paste it over the two ROUTE MAKER lines in src\/team\/skills_route\.cpp/);
            assert.match(await card.locator('figure.xr-paste').textContent(), /ROUTE MAKER — route "skills" — BEGIN.*ROUTE MAKER — route "skills" — END.*select from the BEGIN line to the END line, both included/);
            assert.deepEqual(await page.evaluate(() => window.__copied), [await code(page)], 'the code, byte for byte');
            const build = await card.locator('.xr-build').textContent();
            assert.match(build, /cannot see your editor.*Not checked here: the addTeamRoutes line/);
            assert.doesNotMatch(build, /read back|has exactly this code/, 'a copy is not checked in the file');
            assert.ok(await drawer(page).locator('.xr-next').isVisible());
            await shot(page, `xr-copied-${v.tag}`, true);
            assert.deepEqual(await p.done(), []);
        });
    }

    test('D2: a Copy the browser refuses is RED (never "Copied"): the code opens to copy by hand, no Next', async () => {
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await refuseCopies(page);
        await openExport(page);
        await drawer(page).locator('.xr-steps').getByRole('button', { name: 'Copy the code' }).click();
        const card = result(page).locator('[data-result="error"]');
        await card.waitFor();
        assert.equal(await card.locator('.xr-title').textContent(), 'Not copied');
        assert.match(await card.textContent(), /The fix: select the code below and copy it by hand/);
        assert.equal(await result(page).locator('[data-result="ok"]').count(), 0);
        assert.equal(await drawer(page).locator('.xr-next').count(), 0, 'no Next after a failure');
        assert.notEqual(await drawer(page).locator('details.xr-code').getAttribute('open'), null, 'the code is open to copy');
        assert.deepEqual(await p.done(), []);
    });

    test('D2: after a write, route_maker.json NOT saved (it changed in the folder) is said on the green card', async () => {
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await folder(page, { [FILE]: SKILLS, ...LINES });
        await page.evaluate(() => {
            window.routeBuilder.memory.store['src/team/route_maker.json'] = '{"a teammate": "pulled"}\n';  // after the page read the folder
        });
        await openExport(page);
        await page.getByRole('button', { name: `Write into ${FILE}` }).click();
        const card = result(page).locator('[data-result="ok"]');
        await card.waitFor();
        assert.match(await card.locator('.xr-warn').textContent(), /route_maker\.json was NOT saved: it changed in the folder since this page read it/);
        assert.equal(await page.evaluate(() => window.routeBuilder.memory.store['src/team/route_maker.json']), '{"a teammate": "pulled"}\n', 'theirs kept');
        assert.deepEqual(await p.done(), []);
    });

    test('D4: one route on a Windows browser: Next names it alone, the one command is `python`; a refused command copy says so', async () => {
        const p = await opened(VARIANTS[0], ONE_ROUTE);
        const { page } = p;
        await page.evaluate(() => Object.defineProperty(navigator, 'userAgent', { get: () => 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/131.0.0.0 Safari/537.36' }));
        await openExport(page);
        await drawer(page).locator('.xr-steps').getByRole('button', { name: 'Copy the code' }).click();
        const next = drawer(page).locator('.xr-next');
        await next.waitFor();
        assert.match(await next.textContent(), /On the brain: shulib Auton Tester \(slot 6\) → skills → RUN/);
        assert.equal(await next.locator('code[data-cmd="the one command"]').textContent(), 'python tools/upload_auton.py one');
        await refuseCopies(page);
        const b = next.getByRole('button', { name: 'Copy the build command' });
        await b.click();
        assert.equal(await b.textContent(), 'Select it and copy');
        assert.deepEqual(await p.done(), []);
    });

    test('D2: the download way (Firefox): the downloaded file has the code, and the card says to replace the file and save', {
        skip: BROWSER !== 'firefox' && 'Chrome and Edge write into the folder; the download way is the other browsers\'',
    }, async () => {
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await openExport(page);
        // the wrong file picked: named, and the one fix is to pick again
        let [fc] = await Promise.all([page.waitForEvent('filechooser'),
            page.getByRole('button', { name: 'Pick skills_route.cpp and download the updated one' }).click()]);
        await fc.setFiles({ name: 'autons.cpp', mimeType: 'text/plain', buffer: Buffer.from(LINES['src/team/autons.cpp'], 'utf8') });
        const red = result(page).locator('[data-result="error"]');
        await red.waitFor();
        assert.match(await red.textContent(), /Not downloaded.*You picked autons\.cpp: the routes go into src\/team\/skills_route\.cpp\..*The fix: pick src\/team\/skills_route\.cpp/);
        assert.equal(await drawer(page).locator('.btn.primary').count(), 1, 'one highlight: pick again');
        assert.equal(await drawer(page).getByRole('button', { name: 'Copy the code' }).count(), 1, 'the real file has its lines: Copy stays');
        // a skills_route.cpp without its markers: their fix, and Copy goes (it pastes over lines that are not there)
        [fc] = await Promise.all([page.waitForEvent('filechooser'), red.getByRole('button', { name: 'Pick skills_route.cpp' }).click()]);
        await fc.setFiles({ name: 'skills_route.cpp', mimeType: 'text/plain', buffer: Buffer.from('int x;\n', 'utf8') });
        await red.getByText(/someone removed them/).waitFor();
        assert.match(await red.textContent(), /Not downloaded.*The fix: ask the coordinator to put back/);
        assert.equal(await drawer(page).getByRole('button', { name: 'Copy the code' }).count(), 0);
        [fc] = await Promise.all([page.waitForEvent('filechooser'),
            page.getByRole('button', { name: 'Pick skills_route.cpp and download the updated one' }).click()]);
        const [dl] = await Promise.all([page.waitForEvent('download'),
            fc.setFiles({ name: 'skills_route.cpp', mimeType: 'text/plain', buffer: Buffer.from(SKILLS, 'utf8') })]);
        assert.equal(regionOf(fs.readFileSync(await dl.path(), 'utf8')), await code(page));
        const card = result(page).locator('[data-result="ok"]');
        assert.equal(await card.locator('.xr-title').textContent(), 'Downloaded skills_route.cpp');
        assert.match(await card.textContent(), /Now replace src\/team\/skills_route\.cpp with it\..*Use the NEWEST skills_route\.cpp in your Downloads: a second download gets a number, like skills_route\(1\)\.cpp\..*Then save the project/);
        const build = await card.locator('.xr-build').textContent();
        assert.match(build, /The downloaded file has exactly this code between its two ROUTE MAKER lines: replace yours with it\..*Not checked here: the addTeamRoutes line/);
        assert.doesNotMatch(build, /read back/);
        assert.ok(await card.getByRole('button', { name: 'Save route_maker.json' }).isVisible());
        await card.getByRole('button', { name: 'Copy the path' }).click();
        assert.deepEqual(await page.evaluate(() => window.__copied), [FILE]);
        assert.ok(await drawer(page).locator('.xr-next').isVisible());
        await shot(page, 'xr-downloaded', true);
        assert.deepEqual(await p.done(), []);
    });
});
