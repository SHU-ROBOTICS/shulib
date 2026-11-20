// run_view.page.test.mjs — RMP1-K in a browser, on the page tools/route_builder.py serves (a stand-in checkout,
// upload_auton's fake make and pros, the fake `pros terminal` replaying lines): Check the mechanisms says what will move,
// sends a CHECK route beside the team's routes (each start form run to its end) and writes the team's file back without it
// when the upload ends; the run then paints the steps live from API 2.11's lines; the REAL 11-19 run paints its stop, its
// skips and its summary; a stopped run whose stop line was lost is red by the robot's own word, a NOT LIVE END is not drawn,
// an edit after the run clears its marks; What is on robot one shows the table. ROUTE_MAKER_SHOTS: light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawn, execFileSync } from 'node:child_process';
import { SITE, SHOTS, REPO, skip, launch, shot } from './harness.mjs';
import { saveProject, newProject } from '../../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../../src/model/route.mjs';
import { GOLDENS } from '../helpers.mjs';

const FILE = 'src/team/skills_route.cpp';
const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\n\nnamespace shulib::bench {\n\n${EMPTY}\n}  // namespace shulib::bench\n`;
const MECHS = [{ do: 'driveTo', x: -36, y: -36 }, { do: 'startHome', mechanism: 'cascade', boundS: 2 },
               { do: 'waitUntilDone', mechanism: 'cascade', boundS: 2 }, { do: 'move', mechanism: 'cascade', to: 6, boundS: 2 },
               { do: 'set', mechanism: 'claw', position: 'closed' }, { do: 'run', mechanism: 'rightToggle', volts: 8, forS: 0.8 },
               { do: 'move', mechanism: 'cascade', to: 18, boundS: 3 }, { do: 'set', mechanism: 'claw', position: 'open' },
               { do: 'wait', seconds: 1 }];
// the 11-19 run's 23 library steps, as its own lines name them (step 1 succeeded silently: a turned-first drive's turn)
const REAL = ['driveTo*', 'reverseTo', 'face', 'face', 'driveTo', 'face', 'face', 'face', 'driveTo', 'face', 'face', 'driveTo',
              'reverseTo*', 'face', 'face', 'driveTo', 'face', 'face', 'driveTo', 'face', 'driveTo'];
const realStep = (v, k) => (v.endsWith('*') ? { do: v.slice(0, -1), x: -30 + k, y: -30 + k, faceFirst: true }
    : { do: v, x: -30 + k, y: -20 + k });
const at = (r) => setStart({ ...r, together: false }, { x: -36, y: -60, headingDeg: 90 });
const LEFT = addStep(at(newRoute('left 1')), { do: 'wait', seconds: 2 });
const PROJECT = saveProject({ ...newProject('the test'), routes: [MECHS.reduce(addStep, at(newRoute())), LEFT] });
const PROJECT_REAL = saveProject({ ...newProject('the test'), routes: [REAL.map(realStep).reduce(addStep, at(newRoute())), LEFT] });
const LIVE = ['[ATS] booted: shulib AUTON TESTER build [shulib-robot-variant=one-autontest] -- runs ONE of the team\'s autons',
    '[t=  10.00] [ATS] run 1: \'skills mechs\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown, the abort armed',
    '[t=  14.60] [RTN] routine \'skills mechs\': step 1 (cascade) done in 1.10 s, at 0.0 in',
    '[t=  15.80] [RTN] routine \'skills mechs\': step 2 (cascade) done in 1.20 s, at 6.0 in',
    '[t=  16.10] [RTN] routine \'skills mechs\': step 3 (claw) done in 0.30 s, at \'closed\' (UNCONFIRMED)',
    '[t=  16.90] [RTN] routine \'skills mechs\': step 4 (right toggle) done in 0.80 s',
    '[t=  19.40] [WARN][RTN] routine \'skills mechs\': step 5 (cascade) FAILED: mechanism step FAILED (STALLED) — carrying on; drive safed',
    '[t=  19.70] [RTN] routine \'skills mechs\': step 6 (claw) done in 0.30 s, at \'open\'',
    '[t=  20.70] [RTN] routine \'skills mechs\': step 7 (wait 1 s) done in 1.00 s',
    '[t=  20.80] [ATS] run 1: \'skills mechs\' DONE, 1 FAILED -- ROUTINE_RAN; in time, end action succeeded',
    '[t=  20.80] [ATS] run 1: routine DONE, 6 of 7 steps done, 1 failed, 0 jumped; faults this run: 1, last MECHANISM_STALLED -- see the [ERROR] line',
    '[t=  20.81] [ATS] run 1: seated (-36.00, -60.00) facing 90.0 deg (the IMU read 0.0); END estimate (-36.10, -59.80) facing 90.2 deg LIVE -- measure the tape'];
const SLOTS = { 1: ['shulib Drive', 'v0.1.1-470-gabc1234 | no routes', '2026-11-20T08:10:00'], 2: ['shulib Auton Tester', 'x', '2026-11-19T21:00:00'],
                3: ['Bench Tests', 'Made with PROS', '2026-11-18T19:00:00'], 4: ['shulib Calibrate', 'Made with PROS', '2026-11-18T19:05:00'],
                5: ['PROS button (bench bot)', 'Made with PROS', '2026-11-19T18:00:00'],
                6: ['shulib Auton Tester', 'v0.1.1-470-gabc1234 | skills', '2026-11-20T09:30:00'],
                7: ['shulib Tune', 'Made with PROS', '2026-11-19T20:00:00'], 8: ['old code', '', ''] };
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
const WORK = path.join(REPO, 'build', 'route_builder_runview_pagetest');
const TOOL = path.join(REPO, 'tools', 'route_builder.py');
const REAL_LOG = path.join(REPO, 'tools', 'route_maker', 'test', 'fixtures', 'robot_log', 'robot_one_2026-11-19_run3.log');

let browser;
const open = new Set();  // contexts a failed test left open: closed after it (a page left watching holds the one reader)
let helper;
let url;
let made;
const terminal = () => path.join(WORK, 'terminal.log');

function teamFiles(replay, project = PROJECT) {
    const team = path.join(made.root, 'src', 'team');
    fs.writeFileSync(path.join(team, 'skills_route.cpp'), SKILLS);
    fs.writeFileSync(path.join(team, 'route_maker.json'), project);
    fs.writeFileSync(path.join(team, 'autons.cpp'), 'addTeamRoutes(registry, TeamRobot::Two);\n');
    fs.writeFileSync(path.join(team, 'robot_one_autons.cpp'), 'addTeamRoutes(registry, TeamRobot::One);\n');
    fs.writeFileSync(made.env.FAKE_CASE_FILE, 'ok');
    fs.writeFileSync(made.env.FAKE_CALLS, '');
    fs.copyFileSync(replay, terminal());
}

async function startHelper() {
    const env = { ...process.env, ...made.env, PYTHONIOENCODING: 'utf-8', FAKE_TERMINAL: terminal(), FAKE_TERMINAL_EVERY: '0.05',
                  FAKE_SLOTS: JSON.stringify(SLOTS) };
    helper = spawn(made.python, [TOOL, '--root', made.root, '--site', 'http://127.0.0.1:9/', '--no-browser'], { env, stdio: ['ignore', 'pipe', 'pipe'] });
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
    await page.waitForFunction(() => window.routeBuilder.routes().length === 2, null, { timeout: 10000 });
    const got = page.getByRole('button', { name: 'Got it' });
    if (await got.count()) {
        await got.click();
    }
    if (SHOTS) {
        await page.locator('.toast').waitFor({ state: 'detached', timeout: 10000 });
    }
    open.add(ctx);
    return { page, done: async () => { open.delete(ctx); await ctx.close(); return problems; } };
}
const menu = async (page, id) => {
    await page.locator('#route-switch').click();
    await page.locator(id).click();
};
/** Watch the robot from the route menu; a reader the last page left open frees itself within a ping: try again. Returns
 *  a filter for the page's problems: Chromium logs each refused try's 409 as a console error, which is this wait's. */
async function watchFromMenu(page) {
    let refused = 0;
    for (let k = 0; k < 8; ++k) {
        await menu(page, '#watch-robot');
        await page.waitForFunction(() => /^(?!Opening)/.test(document.querySelector('#rk-summary').textContent), null, { timeout: 30000 });
        if (!/already open/.test(await page.locator('#rk-summary').textContent())) {
            break;
        }
        refused += 1;
        await page.waitForTimeout(1000);
    }
    return (problems) => problems.filter((x) => !(refused && /status of 409 \(Conflict\)/.test(x)));
}
const cards = (page) => page.$$eval('.steps .step[data-step]', (els) => els.map((e) => [...e.classList].find((c) => c.startsWith('rv-')) || ''));

test.describe('RMP1-K: check the mechanisms, watch the run, what is on the robot', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'robots.json')), 'build the site first (tools/route_maker/build.py)');
        const py = process.platform === 'win32' ? 'python' : 'python3';
        made = JSON.parse(execFileSync(py, [TOOL, 'stand-in', WORK, path.join(SITE, 'robots.json')], { encoding: 'utf8' }));
        teamFiles(REAL_LOG);
        await startHelper();
        browser = await launch();
    });
    test.afterEach(async () => {
        for (const ctx of open) {
            await ctx.close();
        }
        open.clear();
    });
    test.after(async () => {
        await browser?.close();
        await stopHelper();
        fs.rmSync(WORK, { recursive: true, force: true });
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] Check the mechanisms: one screen of what moves, sent beside the routes, painted live; the next Send leaves it out`, async () => {
            fs.writeFileSync(path.join(WORK, 'live.log'), LIVE.join('\n'));
            teamFiles(path.join(WORK, 'live.log'));
            const p = await opened(v);
            const { page } = p;
            await menu(page, '#check-mechs');
            const dlg = page.locator('.rk-dialog');
            assert.equal(await dlg.locator('h2').textContent(), 'Check the mechanisms of "skills"');
            assert.deepEqual(await dlg.locator('.rk-moves li').allTextContents(),
                ['the cascade homes, goes to 6 in, then 18 in', 'the claw goes to closed, then open', 'the right toggle runs 0.8 s at 8 V']);
            assert.equal(await dlg.locator('.rk-safe').textContent(), 'The robot stays where it is — clear the mechanisms\' paths.');
            assert.equal(await dlg.locator('.btn.primary').count(), 1, 'one highlight');
            assert.equal(await dlg.locator('.btn.primary').textContent(), 'Send the check to robot one');
            await shot(page, `check-${v.tag}`);
            await dlg.locator('.btn.primary').click();
            await page.locator('.send-panel [data-result="ok"]').waitFor({ timeout: 30000 });
            // the upload carried the check (its names, read from the file it built), then the file is the team's again
            await dlg.locator('#rk-back', { hasText: 'src/team/skills_route.cpp holds the team\'s routes again' }).waitFor({ timeout: 10000 });
            const disk = fs.readFileSync(path.join(made.root, FILE), 'utf8');
            assert.ok(!disk.includes('mechs'), 'written back without the check: a Competition built now lists only the team\'s routes');
            assert.match(disk, /e\.name = "skills";/);
            assert.match(disk, /e\.name = "left 1";/);
            const saved = fs.readFileSync(path.join(made.root, 'src/team/route_maker.json'), 'utf8');
            assert.ok(!saved.includes('mechs'), 'the project file never holds the check');
            assert.equal(await page.evaluate(() => window.routeBuilder.routes().length), 2);
            const up = fs.readFileSync(made.env.FAKE_CALLS, 'utf8').trim().split(/\r?\n/).filter((c) => c.startsWith('pros upload'));
            assert.equal(up.length, 1);
            assert.match(up[0], /^pros upload --slot 6 --name shulib Auton Tester --description no git \| skills, left 1, skills mechs \. \/dev\/ttyACM0$/);
            await dlg.locator('button[aria-label="Close"]').click();
            // sent: the watch opens by itself, and the robot's own lines tick the mechanism steps
            await page.locator('#rk-summary', { hasText: '6 of 7 steps done — 1 failed and carried on' }).waitFor({ timeout: 30000 });
            assert.equal(await page.locator('.rk-status').getAttribute('class'), 'rk-status warn', 'a step failed: amber, never a ✓');
            assert.deepEqual(await cards(page), ['', 'rv-done', '', 'rv-done', 'rv-done', 'rv-done', 'rv-failed', 'rv-done', 'rv-done'],
                'the drive is not the check\'s, nor the waitUntilDone (its start ran to its end); each mechanism step as the robot said');
            assert.equal(await page.locator('.step[data-step="6"] .step-robot').textContent(), 'Failed on the robot: mechanism step FAILED (STALLED) — carried on');
            assert.equal(await page.locator('.step[data-step="3"] .step-robot').textContent(), 'On the robot: done in 1.20 s, at 6.0 in');
            await page.locator('.field-svg .rk-end').waitFor({ timeout: 10000 });  // its line comes just after the summary's
            assert.equal(await page.locator('.field-svg .rk-end').count(), 1, 'the END estimate on the field');
            await shot(page, `run-check-${v.tag}`);
            // the next ordinary Send writes the team's routes only too
            await page.locator('#send').click();
            await page.locator('.drawer .send-panel [data-result="ok"]').waitFor({ timeout: 30000 });
            assert.ok(!fs.readFileSync(path.join(made.root, FILE), 'utf8').includes('mechs'), 'the check is gone');
            assert.deepEqual(await p.done(), []);
        });

        test(`[${v.tag}] Watch the robot: the REAL 11-19 run -- its stop in its own words, the skips, the summary, the END`, async () => {
            teamFiles(REAL_LOG, PROJECT_REAL);
            const p = await opened(v);
            const { page } = p;
            const mine = await watchFromMenu(page);
            await page.locator('#rk-summary', { hasText: '1 of 23 steps done — stopped at step 2, driveTo: motion TIMEOUT' }).waitFor({ timeout: 60000 });
            assert.equal(await page.locator('.rk-status').getAttribute('class'), 'rk-status bad', 'stopped: red');
            const marks = await cards(page);
            assert.equal(marks[0], 'rv-stopped', 'page step 1 is the brain\'s steps 1-2: the turn done, the drive stopped');
            assert.deepEqual(marks.slice(1), Array(REAL.length - 1).fill('rv-skipped'));
            assert.equal(await page.locator('.step[data-step="0"] .step-robot').textContent(), 'Stopped here on the robot: motion TIMEOUT');
            await page.locator('.field-svg .rk-end').waitFor({ timeout: 10000 });  // its line comes just after the summary's
            assert.equal(await page.locator('.field-svg .rk-end').count(), 1);
            await page.getByText('Show the robot\'s log').click();
            assert.match(await page.locator('.rk-log pre').textContent(), /END estimate \(52\.51, -13\.32\)/);
            assert.match(await page.locator('.rk-log .hint').textContent(), /^Saved: build\/robot_runs\/.+\.log$/);
            await shot(page, `run-real-${v.tag}`);
            await page.getByRole('button', { name: 'Stop watching' }).click();
            assert.deepEqual(mine(await p.done()), []);
        });

        test(`[${v.tag}] Watch: a stop line lost is red by the robot's word; a started step's end; a NOT LIVE END not drawn; an edit clears the marks`,
            { skip: v !== VARIANTS[0] && 'one size is enough' }, async () => {
                fs.writeFileSync(path.join(WORK, 'lost.log'), [
                    '[t=  10.00] [ATS] run 1: \'skills\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown, the abort armed',
                    '[t=  14.00] [RTN] routine \'skills\': step 1 (driveTo) done in 1.00 s',
                    '[t=  14.00] [RTN] routine \'skills\': step 2 (cascade) started in 0.00 s',
                    '[t=  14.90] [MECH] \'cascade\': SUCCEEDED -- reached in 0.90s',
                    '[t=  14.90] [RTN] routine \'skills\': step 3 (cascade) done in 0.90 s, at 0.0 in',
                    '[t=  17.00] [ATS] run 1: \'skills\' ROUTINE STOPPED -- ROUTINE_RAN; in time, end action succeeded',
                    '[t=  17.00] [ATS] run 1: routine STOPPED, 3 of 9 steps done, stopped at cascade (mechanism step FAILED); faults this run: 1',
                    '[t=  17.01] [ATS] run 1: seated (-36.00, -60.00) facing 90.0 deg (the IMU read 0.0); END estimate (-36.10, -59.80) facing 90.2 deg NOT LIVE -- measure the tape',
                ].join('\n'));
                teamFiles(path.join(WORK, 'lost.log'));
                const p = await opened(v);
                const { page } = p;
                const mine = await watchFromMenu(page);
                await page.locator('#rk-summary', { hasText: '3 of 9 steps done — stopped at cascade: mechanism step FAILED' }).waitFor({ timeout: 30000 });
                assert.equal(await page.locator('.rk-status').getAttribute('class'), 'rk-status bad', 'the robot said STOPPED: red, its stop line lost or not');
                assert.equal(await page.locator('.step[data-step="1"] .step-robot').textContent(), 'On the robot: started, then SUCCEEDED in 0.90 s');
                await page.locator('#rk-notlive').waitFor({ timeout: 10000 });
                assert.equal(await page.locator('.field-svg .rk-end').count(), 0, 'NOT LIVE: the robot does not trust it, nor does the page');
                assert.deepEqual((await cards(page)).slice(0, 3), ['rv-done', 'rv-done', 'rv-done']);
                await page.locator('.step[data-step="8"]').click();
                await page.keyboard.press('Delete');
                await page.waitForFunction(() => window.routeBuilder.route().steps.length === 8, null, { timeout: 10000 });
                await page.locator('#rk-differs', { hasText: 'You changed this route after the run' }).waitFor({ timeout: 10000 });
                assert.deepEqual((await cards(page)).filter(Boolean), [], 'no mark left on the changed route');
                await page.keyboard.press('Control+z');
                await page.locator('#rk-differs').waitFor({ state: 'detached', timeout: 10000 });
                assert.deepEqual((await cards(page)).slice(0, 3), ['rv-done', 'rv-done', 'rv-done'], 'undone: the marks come back');
                await page.getByRole('button', { name: 'Stop watching' }).click();
                assert.deepEqual(mine(await p.done()), []);
            });

        test(`[${v.tag}] What is on robot one: each slot against the wanted table, the first wrong one highlighted`, async () => {
            teamFiles(REAL_LOG);
            const p = await opened(v);
            const { page } = p;
            await page.locator('#route-switch').click();
            await page.locator('[data-slots="one"]').click();
            const dlg = page.locator('.rk-dialog');
            await dlg.locator('.rk-slots').waitFor({ timeout: 30000 });
            assert.equal(await dlg.locator('.rk-verdict').textContent(),
                'Fix this first: slot 2 holds shulib Auton Tester: upload shulib Competition there.');
            assert.equal(await dlg.locator('tr.first').getAttribute('data-slot'), '2');
            assert.deepEqual(await dlg.locator('tbody .rk-mark').allTextContents(), ['✓', '✗', '✓', '✓', '✓', '✓', '✓', '✓']);
            assert.equal(await dlg.locator('tr[data-slot="5"] td').nth(1).textContent(), "the PROS button's build (optional)");
            assert.equal(await dlg.locator('tr[data-slot="8"] td').nth(1).textContent(), "the team's old code");
            assert.match(await dlg.locator('tr[data-slot="6"] small').textContent(), /uploaded 2026-11-20 09:30 · v0\.1\.1-470-gabc1234 \| skills/);
            const ran = fs.readFileSync(made.env.FAKE_CALLS, 'utf8');
            assert.ok(!/upload|rm-|write/.test(ran), `read-only: ${ran}`);
            await shot(page, `slots-${v.tag}`);
            assert.deepEqual(await p.done(), []);
        });
    }
});
