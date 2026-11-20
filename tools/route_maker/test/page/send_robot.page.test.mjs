// send_robot.page.test.mjs — RMP1-I in a browser: the page tools/route_builder.py serves (a stand-in checkout,
// upload_auton's fake make and pros) opens with NO folder dialog, Export writes src/team/skills_route.cpp and reads it
// back, and Send to robot shows upload_auton's lines in order, ✓ UPLOADED with the routes' names, or ✗ in its words.
// The site form keeps Export as before. The review's cases: a robot whose autons file lacks its line is never sent to;
// a send already running, a helper stopped mid-send and an upload_auton that ends with no ✗ are each a red card; Export
// and Save wait for a send. ROUTE_MAKER_SHOTS: the screens, light and dark, at 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import http from 'node:http';
import path from 'node:path';
import { spawn, execFileSync } from 'node:child_process';
import { SITE, SHOTS, BROWSER, REPO, skip, launch, serve, openPage, shot } from './harness.mjs';
import { saveProject, newProject } from '../../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../../src/model/route.mjs';
import { GOLDENS } from '../helpers.mjs';
import { TESTER } from '../../src/model/to_robot.mjs';
import { GONE } from '../../src/host/files_local.mjs';

const FILE = 'src/team/skills_route.cpp';
const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\r\n\r\nnamespace shulib::bench {\r\n\r\n${EMPTY.replace(/\n/g, '\r\n')}\r\n}  // namespace shulib::bench\r\n`;
const at = (r, x, y) => setStart({ ...r, together: false }, { x, y, headingDeg: 90 });
const PROJECT = saveProject({ ...newProject('the test'), routes: [
    addStep(addStep(at(newRoute(), -36, -60), { do: 'driveTo', x: -36, y: -36 }), { do: 'wait', seconds: 1 }),
    addStep(at(newRoute('left 1'), -36, -60), { do: 'wait', seconds: 2 })] });
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
const WORK = path.join(REPO, 'build', 'route_builder_pagetest');
const TOOL = path.join(REPO, 'tools', 'route_builder.py');

let browser;
let helper;
let url;
let made;

/** The team's files as each test starts, and the fake tools' case for its sends ("ok 1.5": make takes 1.5 s). */
function teamFiles(fakeCase) {
    const team = path.join(made.root, 'src', 'team');
    fs.writeFileSync(path.join(team, 'skills_route.cpp'), SKILLS);
    fs.writeFileSync(path.join(team, 'route_maker.json'), PROJECT);
    fs.writeFileSync(path.join(team, 'autons.cpp'), 'addTeamRoutes(registry, TeamRobot::Two);\n');
    fs.writeFileSync(path.join(team, 'robot_one_autons.cpp'), 'addTeamRoutes(registry, TeamRobot::One);\n');
    fs.writeFileSync(made.env.FAKE_CASE_FILE, fakeCase);
    fs.writeFileSync(made.env.FAKE_CALLS, '');
}

/** Start the helper on the stand-in checkout (`url`: the link it printed); stop it (as Ctrl+C does). */
async function startHelper() {
    helper = spawn(made.python, [TOOL, '--root', made.root, '--site', 'http://127.0.0.1:9/', '--no-browser'],
                   { env: { ...process.env, ...made.env, PYTHONIOENCODING: 'utf-8' }, stdio: ['ignore', 'pipe', 'pipe'] });
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
}
async function stopHelper() {
    if (helper && helper.exitCode === null) {
        const ended = new Promise((ok) => helper.on('exit', ok));
        helper.kill(process.platform === 'win32' ? 'SIGTERM' : 'SIGINT');
        await ended;
    }
}
/** A send started from somewhere else (another tab): resolves to its status when it has ended. */
function sendElsewhere(robot) {
    const s = JSON.parse(fs.readFileSync(path.join(made.root, 'build', 'route_builder.json'), 'utf8'));
    return new Promise((ok, bad) => {
        const req = http.request({ host: '127.0.0.1', port: s.port, method: 'POST', path: `/${s.token}/api/send?robot=${robot}`,
                                   headers: { 'X-Shulib-Token': s.token } }, (res) => {
            res.resume();
            res.on('end', () => ok(res.statusCode));
        });
        req.on('error', bad);
        req.end();
    });
}
/** Until `cond()` (polled), at most `ms`. */
async function until(cond, ms = 10000) {
    const end = Date.now() + ms;
    while (!cond() && Date.now() < end) {
        await new Promise((ok) => setTimeout(ok, 50));
    }
    return cond();
}
/** What the sends ran: make and upload_auton's pros calls. RMP1-K: a send that ends well opens Watch the robot (its own
 *  `pros lsusb` and `pros terminal`, as the page's watch's, not the send's), left out up to the next make. */
const calls = () => {
    const all = fs.readFileSync(made.env.FAKE_CALLS, 'utf8').trim().split(/\r?\n/).filter(Boolean);
    const out = [];
    let sending = false;
    for (const c of all) {
        sending = c.startsWith('make') ? true : c.startsWith('pros terminal') ? false : sending;
        if (sending && !c.startsWith('pros terminal')) {
            out.push(c);
        }
        sending = sending && !c.startsWith('pros upload');
    }
    return out.join('\n');
};
/** The problems, less the ones Chromium logs for an answer this test asked for (`expected`: a 409, a helper gone). */
const besides = (problems, expected) => problems.filter((x) => !expected.test(x));

/** The helper's page in a fresh context (the link it printed); every problem recorded, as openPage does. */
async function opened(v) {
    const ctx = await browser.newContext({ viewport: v.viewport, colorScheme: v.scheme });
    const page = await ctx.newPage();
    const problems = [];
    const base = new URL(url).origin;
    page.on('console', (m) => ['error', 'warning'].includes(m.type()) && problems.push(`console ${m.type()}: ${m.text()}`));
    page.on('pageerror', (e) => problems.push(`page error: ${e}`));
    page.on('request', (r) => !r.url().startsWith(base) && !/^(data|blob):/.test(r.url()) && problems.push(`a request outside the page: ${r.url()}`));
    await page.goto(url);
    await page.waitForFunction(() => document.documentElement.dataset.ready, null, { timeout: 30000 });
    await page.evaluate(() => {
        window.__csp = [];
        document.addEventListener('securitypolicyviolation', (e) => window.__csp.push(`${e.violatedDirective} ${e.blockedURI}`));
    });
    await page.waitForFunction(() => window.routeBuilder.routes().length === 2, null, { timeout: 10000 });  // the folder's project
    const got = page.getByRole('button', { name: 'Got it' });
    if (await got.count()) {
        await got.click();
    }
    if (SHOTS) {
        await page.locator('.toast').waitFor({ state: 'detached', timeout: 10000 });
    }
    return { page, done: async () => {
        const csp = await page.evaluate(() => window.__csp);
        await ctx.close();
        return [...problems, ...csp.map((c) => `CSP violation: ${c}`)];
    } };
}
const drawer = (page) => page.locator('.drawer');
async function writeFromDrawer(page) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('Control+e');
    await page.getByRole('button', { name: `Write into ${FILE}` }).click();
    await drawer(page).locator('[data-result="ok"]').waitFor();
}
/** The panel's lines in the order they appeared (a MutationObserver from before the send). */
const watch = (page) => page.evaluate(() => {
    window.__seen = [];
    new MutationObserver(() => {
        for (const el of document.querySelectorAll('.send-steps li, .send-panel .xr-title')) {
            if (!window.__seen.includes(el.textContent)) {
                window.__seen.push(el.textContent);
            }
        }
    }).observe(document.body, { childList: true, subtree: true, characterData: true });
});

test.describe('RMP1-I: Send to robot', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'robots.json')), 'build the site first (tools/route_maker/build.py)');
        const py = process.platform === 'win32' ? 'python' : 'python3';
        made = JSON.parse(execFileSync(py, [TOOL, 'stand-in', WORK, path.join(SITE, 'robots.json')], { encoding: 'utf8' }));
        teamFiles('ok');
        await startHelper();
        browser = await launch();
    });
    test.after(async () => {
        await browser?.close();
        await stopHelper();
        fs.rmSync(WORK, { recursive: true, force: true });
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] no folder dialog; Export writes and reads back; Send to robot one: the lines in order, then ✓ UPLOADED`, async () => {
            teamFiles('ok 1.5');
            const p = await opened(v);
            const { page } = p;
            assert.equal(await page.getByRole('button', { name: 'Connect your shulib folder' }).count(), 0, 'no folder to connect');
            assert.equal(await page.locator('#send').textContent(), 'Send to robot one');
            assert.ok(await page.locator('#send').evaluate((b) => b.classList.contains('primary')), 'Send is the one highlight');
            assert.ok(!(await page.locator('#export').evaluate((b) => b.classList.contains('primary'))));
            await shot(page, `send-topbar-${v.tag}`);
            await writeFromDrawer(page);
            const disk = fs.readFileSync(path.join(made.root, FILE), 'utf8');
            assert.match(disk, /e\.name = "skills";/);
            assert.ok(!/[^\r]\n/.test(disk), 'the file\'s CRLF kept');
            const card = drawer(page).locator('.send-card');
            assert.equal(await card.locator('.btn.primary').textContent(), 'Send to robot one');
            assert.equal(await card.locator('[data-send="two"]').textContent(), 'Send to robot two');
            assert.equal(await drawer(page).locator('.xr-next code').count(), 0, 'no commands to copy: one button');
            await shot(page, `send-result-${v.tag}`, true);
            await watch(page);
            await card.locator('.btn.primary').click();
            await page.locator('.send-steps li.now', { hasText: 'Building robot one' }).waitFor();
            assert.equal(await page.locator('#send').textContent(), 'Sending…');
            assert.ok(await page.locator('#send').isDisabled(), 'one send at a time');
            assert.ok(await page.locator('#export').isDisabled(), 'nothing written while it builds');
            assert.ok(await page.getByRole('button', { name: 'Save (Ctrl+S)' }).isDisabled());
            assert.ok(await card.locator('[data-send="two"]').isDisabled());
            assert.equal(await drawer(page).locator('.xr-build').count(), 0, 'Will it build? gives way to the build itself');
            await shot(page, `send-mid-${v.tag}`);
            const ok = page.locator('.send-panel [data-result="ok"]');
            await ok.waitFor({ timeout: 30000 });
            assert.equal(await ok.locator('.xr-title').textContent(), `UPLOADED to slot ${TESTER.slot}`);
            assert.equal(await ok.locator('.send-brain').textContent(), 'On the brain: shulib Auton Tester → skills or left 1 → RUN');
            const seen = await page.evaluate(() => window.__seen);
            const order = ['Building robot one', '✓BUILT', `Uploading to slot ${TESTER.slot}`, `UPLOADED to slot ${TESTER.slot}`].map((w) => seen.findIndex((s) => s.includes(w)));
            assert.ok(!order.includes(-1) && order.every((x, k) => k === 0 || x > order[k - 1]), `in order: ${JSON.stringify(seen)}`);
            assert.equal(await page.locator('#send').textContent(), 'Send to robot one');
            assert.ok(!(await page.locator('#export').isDisabled()), 'Export again once it has ended');
            // make on every core (`make -jN`, N this computer's: tools/upload_auton.py's JOBS, 2026-11-20)
            assert.deepEqual(calls().split('\n').map((c) => c.replace(/^make -j\d+ /, 'make -jN ')),
                ['make -jN ROBOT=one PROGRAM=autontest', 'pros lsusb', `pros upload --slot ${TESTER.slot} --name shulib Auton Tester --description no git | skills, left 1 . /dev/ttyACM0`]);
            await shot(page, `send-ok-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });

        test(`[${v.tag}] ✗: a compile error inside the route's lines, in plain words; Show everything`, async () => {
            teamFiles('region');
            const p = await opened(v);
            const { page } = p;
            await writeFromDrawer(page);
            await drawer(page).locator('.send-card .btn.primary').click();
            const bad = page.locator('.send-panel [data-result="error"]');
            await bad.waitFor({ timeout: 30000 });
            assert.equal(await bad.locator('.xr-title').textContent(), 'NOT BUILT');
            assert.match(await bad.textContent(), /skills_route\.cpp:6: 'r' was not declared in this scope.*re-export from the route builder/);
            assert.equal(await page.locator('.send-panel [data-result="ok"]').count(), 0);
            assert.equal(await page.locator('.send-steps li.ok').count(), 0, 'never ✓ BUILT');
            await shot(page, `send-failed-${v.tag}`);  // before Show everything: its log names this laptop's folders
            await page.getByText('Show everything').click();
            await page.locator('.send-all pre', { hasText: '[ERRORS]' }).waitFor();
            assert.deepEqual(await p.done(), []);
        });
    }

    test('the top bar\'s Send writes the changed routes first, then sends; closed mid-send, a toast says how it ended', async () => {
        teamFiles('ok 1');
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.locator('#send').click();
        await drawer(page).locator('[data-result="ok"]', { hasText: 'Written and checked' }).waitFor();
        assert.match(fs.readFileSync(path.join(made.root, FILE), 'utf8'), /e\.name = "left 1";/, 'written before the send');
        await page.locator('.send-steps li', { hasText: 'Building robot one' }).waitFor();
        await page.keyboard.press('Escape');
        await page.locator('.toast', { hasText: `✓ UPLOADED to slot ${TESTER.slot}: On the brain: shulib Auton Tester → skills or left 1 → RUN` }).waitFor({ timeout: 30000 });
        assert.equal(await page.locator('#send').isDisabled(), false);
        assert.deepEqual(await p.done(), []);
    });

    test('a robot whose autons file lacks its line: never sent to (it would never list the routes), its ✗ stays on screen', async () => {
        const one = path.join(made.root, 'src', 'team', 'robot_one_autons.cpp');
        teamFiles('ok');
        fs.writeFileSync(one, '// someone removed the line\n');
        let p = await opened(VARIANTS[0]);
        await p.page.locator('#send').click();  // the top bar's Send: it writes, then would send
        await drawer(p.page).locator('[data-result="ok"]', { hasText: 'Written and checked' }).waitFor();
        await new Promise((ok) => setTimeout(ok, 500));
        assert.equal(calls(), '', 'nothing built or uploaded');
        assert.equal(await drawer(p.page).locator('.xr-build li.bad', { hasText: 'Robot one will not list the routes' }).count(), 1, 'the ✗ stays');
        assert.ok(await drawer(p.page).locator('[data-send="one"]').isDisabled());
        assert.match(await drawer(p.page).locator('[data-lacks="one"]').textContent(),
            /src\/team\/robot_one_autons\.cpp is missing addTeamRoutes\(registry, TeamRobot::One\);.*tell the coordinator/);
        assert.ok(!(await drawer(p.page).locator('[data-send="two"]').isDisabled()), 'robot two\'s line is there: it can be sent to');
        await shot(p.page, 'send-not-listed-light-1440', true);
        await drawer(p.page).locator('[data-send="two"]').click();  // robot two: sent, and robot one's ✗ stays on screen
        await p.page.locator('.send-panel [data-result="ok"]').waitFor({ timeout: 30000 });
        assert.equal(await drawer(p.page).locator('.xr-build li.bad', { hasText: 'Robot one will not list the routes' }).count(), 1);
        assert.match(calls(), /^make -j\d+ ROBOT=tank PROGRAM=autontest/);
        assert.deepEqual(await p.done(), []);
        teamFiles('ok');  // the line there when the drawer opened, gone by the time Send is pressed: read again, not sent
        p = await opened(VARIANTS[0]);
        await writeFromDrawer(p.page);
        fs.writeFileSync(one, '// addTeamRoutes(registry, TeamRobot::One);  (commented out)\n');
        await drawer(p.page).locator('.send-card .btn.primary').click();
        await drawer(p.page).locator('[data-lacks="one"]').waitFor();
        assert.equal(calls(), '');
        assert.equal(await p.page.locator('.send-steps li').count(), 0);
        assert.deepEqual(await p.done(), []);
    });

    test('a send already running (another tab): a red card in the helper\'s words, and Send again once it ends', async () => {
        teamFiles('ok 3');
        fs.rmSync(made.env.FAKE_PIDS, { force: true });
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await writeFromDrawer(page);
        const other = sendElsewhere('one');
        assert.ok(await until(() => fs.existsSync(made.env.FAKE_PIDS) && fs.readFileSync(made.env.FAKE_PIDS, 'utf8').includes('make')), 'the other send builds');
        await drawer(page).locator('.send-card .btn.primary').click();
        const bad = page.locator('.send-panel [data-result="error"]');
        await bad.waitFor();
        assert.match(await bad.textContent(), /already running: wait for it to finish/);
        assert.ok(!(await page.locator('#send').isDisabled()), 'this page is not sending');
        assert.equal(await other, 200);
        assert.ok(!(await drawer(page).locator('.send-card .btn.primary').isDisabled()));
        assert.deepEqual(besides(await p.done(), /^console error: Failed to load resource: .* 409 \(Conflict\)$/), []);
    });

    test('upload_auton ending with no ✗ line (a crash): the panel says it ended, in red — never "Sending…" for ever', async () => {
        teamFiles('ok');
        const log = path.join(made.root, 'build', 'upload_auton.log');
        fs.rmSync(log, { recursive: true, force: true });
        fs.mkdirSync(log, { recursive: true });  // upload_auton cannot write its log: a traceback, exit 1
        try {
            const p = await opened(VARIANTS[0]);
            await writeFromDrawer(p.page);
            await drawer(p.page).locator('.send-card .btn.primary').click();
            await p.page.locator('.send-panel [data-result="error"]', { hasText: 'Not sent (tools/upload_auton.py ended with 1)' }).waitFor({ timeout: 30000 });
            assert.equal(await p.page.locator('#send').textContent(), 'Send to robot one');
            assert.equal(calls(), '');
            assert.deepEqual(await p.done(), []);
        } finally {
            fs.rmSync(log, { recursive: true, force: true });
        }
    });

    test('the helper stopped mid-send (Ctrl+C in its terminal): the send says it stopped; Send again says to start it again', async () => {
        teamFiles('ok 5');
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        try {
            await writeFromDrawer(page);
            await drawer(page).locator('.send-card .btn.primary').click();
            await page.locator('.send-steps li', { hasText: 'Building robot one' }).waitFor();
            await stopHelper();
            const bad = page.locator('.send-panel [data-result="error"]');
            await bad.waitFor({ timeout: 30000 });  // upload_auton's own "✗ STOPPED (Ctrl+C)", or the helper's GONE
            assert.match(await bad.textContent(), /STOPPED \(Ctrl\+C\)|has stopped: start it again/);
            assert.ok(!(await page.locator('#send').isDisabled()), 'Send is not stuck on "Sending…"');
            await drawer(page).locator('.send-card .btn.primary').click();  // the helper is gone now
            await page.locator('.send-panel [data-result="error"]', { hasText: GONE }).waitFor();
            assert.ok(!(await page.locator('#send').isDisabled()));
            assert.deepEqual(besides(await p.done(), /^console error: Failed to load resource: net::ERR_CONNECTION_REFUSED$/), []);
        } finally {
            await startHelper();  // the same port: the next test's page opens as before
        }
    });

    test('the site form (no helper): no Send, Export is the one highlight, as before', async () => {
        const site = await serve();
        try {
            const p = await openPage(browser, site.base);
            assert.equal(await p.page.locator('#send').count(), 0);
            assert.equal(await p.page.locator('meta[name="shulib-local"]').count(), 0);
            assert.ok(await p.page.locator('#export').evaluate((b) => b.classList.contains('primary')));
            assert.deepEqual(await p.done(), []);
        } finally {
            site.server.close();
        }
    });
});
