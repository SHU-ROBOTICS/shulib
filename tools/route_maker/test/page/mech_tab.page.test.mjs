// mech_tab.page.test.mjs — RMP1-M in a browser, on the page tools/route_builder.py serves (a stand-in checkout,
// upload_auton's fake make and pros, the fake `pros terminal` replaying lines): the Mechanisms tab shows robot one's
// mechanisms from the site's own robots.json (each card's ports its own; robot two's owed direction said as owed); a
// card's route opens at its first step; Test says what will move BEFORE anything is sent, sends the one-mechanism test
// beside the team's routes and writes the file back; the robot's own lines tick the card. An owed port: Test says why and
// sends nothing. D5: no "home" on the screens a person reads. The review's cases: a Test sent again is never ticked by
// the earlier run; robot two's Test is refused while its Auton Tester does not build, else offered to robot two only; the
// tab never edits the route. ROUTE_MAKER_SHOTS: light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { spawn, execFileSync } from 'node:child_process';
import { SITE, SHOTS, BROWSER, REPO, skip, launch, shot } from './harness.mjs';
import { saveProject, newProject } from '../../src/model/project.mjs';
import { newRoute, setStart, addStep } from '../../src/model/route.mjs';
import { GOLDENS } from '../helpers.mjs';

const FILE = 'src/team/skills_route.cpp';
const EMPTY = fs.readFileSync(path.join(GOLDENS, 'gl0_empty.cpp'), 'utf8');
const SKILLS = `#include "team/skills_route.hpp"\n\nnamespace shulib::bench {\n\n${EMPTY}\n}  // namespace shulib::bench\n`;
const at = (r) => setStart({ ...r, together: false }, { x: -36, y: -60, headingDeg: 90 });
const STEPS = [{ do: 'driveTo', x: -36, y: -36 }, { do: 'home', mechanism: 'cascade', boundS: 2 }, { do: 'move', mechanism: 'cascade', to: 6, boundS: 2 },
               { do: 'set', mechanism: 'claw', position: 'closed' }];
const LEFT = addStep(at(newRoute('left 1')), { do: 'set', mechanism: 'claw', position: 'open' });
const PROJECT = saveProject({ ...newProject('the test'), routes: [STEPS.reduce(addStep, at(newRoute())), LEFT] });
const TEST_RUN = ['[ATS] booted: shulib AUTON TESTER build [shulib-robot-variant=one-autontest] -- runs ONE of the team\'s autons',
    '[t=  10.00] [ATS] run 1: \'cascade mechs\' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown, the abort armed',
    '[t=  11.00] [RTN] routine \'cascade mechs\': step 1 (cascade) done in 0.90 s, at 0.0 in',
    '[t=  12.00] [RTN] routine \'cascade mechs\': step 2 (cascade) done in 1.00 s, at 10.0 in',
    '[t=  13.00] [RTN] routine \'cascade mechs\': step 3 (cascade) done in 1.00 s, at 0.0 in',
    '[t=  13.10] [ATS] run 1: \'cascade mechs\' FINISHED -- ROUTINE_RAN; in time, end action succeeded',
    '[t=  13.10] [ATS] run 1: routine OK, 3 of 3 steps done; faults this run: 0'];
const BOOT_ONLY = [TEST_RUN[0]];  // the robot booted, and never ran the test
const ON_TWO = [TEST_RUN[0].replace('variant=one-', 'variant=tank-'), ...TEST_RUN.slice(1)];  // robot two's brain ran a test of that name
const HOME_WORD = /\bhom(e|es|ed|ing)\b/i;
const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
// one stand-in checkout per browser: the gate runs Firefox and Chromium at once, from one tree
const WORK = path.join(REPO, 'build', `route_builder_mechtab_pagetest_${BROWSER}`);
const TOOL = path.join(REPO, 'tools', 'route_builder.py');

let browser;
const open = new Set();
let helper;
let url;
let made;
let robots;
const terminal = () => path.join(WORK, 'terminal.log');

function teamFiles(lines) {
    const team = path.join(made.root, 'src', 'team');
    fs.writeFileSync(path.join(team, 'skills_route.cpp'), SKILLS);
    fs.writeFileSync(path.join(team, 'route_maker.json'), PROJECT);
    fs.writeFileSync(path.join(team, 'autons.cpp'), 'addTeamRoutes(registry, TeamRobot::Two);\n');
    fs.writeFileSync(path.join(team, 'robot_one_autons.cpp'), 'addTeamRoutes(registry, TeamRobot::One);\n');
    for (const id of ['one', 'two']) {  // the robots' driver files, as the team's checkout has them: the Controls tab's buttons
        fs.copyFileSync(path.join(REPO, 'src', 'team', `robot_${id}_driver.hpp`), path.join(team, `robot_${id}_driver.hpp`));
    }
    fs.writeFileSync(made.env.FAKE_CASE_FILE, 'ok');
    fs.writeFileSync(made.env.FAKE_CALLS, '');
    fs.writeFileSync(terminal(), lines.join('\n'));
}
async function startHelper() {
    const env = { ...process.env, ...made.env, PYTHONIOENCODING: 'utf-8', FAKE_TERMINAL: terminal(), FAKE_TERMINAL_EVERY: '0.05' };
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
const uploads = () => fs.readFileSync(made.env.FAKE_CALLS, 'utf8').trim().split(/\r?\n/).filter((c) => c.startsWith('pros upload'));
const until = async (f, what, ms = 30000) => {
    for (const end = Date.now() + ms; !f(); await new Promise((ok) => setTimeout(ok, 100))) {
        assert.ok(Date.now() < end, `timed out: ${what}`);
    }
};
/** Test the `ident` card, Send, wait for the upload and the write-back, close the dialog. */
async function sendTest(page, ident) {
    const before = uploads().length;
    await page.locator(`[data-test="${ident}"]`).click();
    await page.locator('.rk-dialog .btn.primary').click();
    await until(() => uploads().length === before + 1, 'the upload');
    await page.locator('.rk-dialog #rk-back').waitFor({ timeout: 30000 });
    await page.locator('.rk-dialog button[aria-label="Close"]').click();
}
const tone = (card) => card.evaluate((e) => ['ok', 'warn', 'bad'].filter((c) => e.classList.contains(c)));
/** What a person reads on the page now: its visible text, the robot's own log lines and the keyboard's key names left out. */
const readable = (page) => page.evaluate(() => {
    const keep = [...document.querySelectorAll('.rk-log, kbd, .keys')];
    keep.forEach((e) => { e.dataset.hide = e.style.display; e.style.display = 'none'; });
    const t = document.body.innerText;
    keep.forEach((e) => { e.style.display = e.dataset.hide; });
    return t;
});

test.describe('RMP1-M: the Mechanisms tab', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'robots.json')), 'build the site first (tools/route_maker/build.py)');
        robots = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
        const py = process.platform === 'win32' ? 'python' : 'python3';
        made = JSON.parse(execFileSync(py, [TOOL, 'stand-in', WORK, path.join(SITE, 'robots.json')], { encoding: 'utf8' }));
        teamFiles(TEST_RUN);
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
        test(`[${v.tag}] every mechanism of robot one, its own ports; Test says what moves BEFORE sending, then the robot's lines tick the card`, async () => {
            teamFiles(TEST_RUN);
            const p = await opened(v);
            const { page } = p;
            await page.locator('#tab-mechanisms').click();
            const one = robots.one.mechanisms;
            assert.deepEqual(await page.locator('.mech-card').evaluateAll((els) => els.map((e) => e.dataset.mech)), one.map((m) => m.ident), 'the table\'s order');
            for (const m of one) {  // each card's ports are ITS OWN, as the site's robots.json has them
                const ports = await page.locator(`.mech-card[data-mech="${m.ident}"] .mc-row`, { hasText: 'Ports' }).locator('dd span').allTextContents();
                const want = [...m.wiring.motors.map((x) => `port ${x.port}, turns ${x.reversed ? 'reversed' : 'forward'}`), ...m.wiring.lines.map((l) => `3-wire port ${l}`)];
                assert.deepEqual(ports, want, m.name);
            }
            const cascade = page.locator('.mech-card[data-mech="cascade"]');
            const row = (c, label) => c.locator('.mc-row', { hasText: label }).locator('dd');
            assert.equal(await row(cascade, 'What it is').textContent(), `a lift: ${one[0].range[0]} to ${one[0].range[1]} in`);
            assert.equal(await row(cascade, 'Levels').textContent(), one[0].presets.length ? one[0].presets.map((x) => `${x.name} ${x.owed ? 'not measured yet' : `${x.value} in`}`).join('') : 'no levels yet');
            assert.deepEqual(await row(cascade, 'Routes').locator('button').allTextContents(), ['skills']);
            assert.deepEqual(await row(page.locator('.mech-card[data-mech="claw"]'), 'Routes').locator('button').allTextContents(), ['skills', 'left 1']);
            const buttons = await row(cascade, 'Buttons').textContent();
            const ctl = await page.evaluate(() => window.routeBuilder.controls());  // the Controls tab's robot one: the same buttons
            assert.ok(ctl.state.controls.list.some((x) => x.mechanism === 'cascade'), 'the driver file gives the cascade its buttons');
            for (const c of ctl.state.controls.list.filter((x) => x.mechanism === 'cascade')) {
                for (const b of [c.button, c.back].filter(Boolean)) {
                    assert.ok(buttons.includes(b), `${b} in "${buttons}"`);
                }
            }
            assert.equal(await page.locator('.btn.primary:visible').count(), 1, 'one highlight: Test every mechanism');
            assert.ok(await page.evaluate(() => document.documentElement.scrollWidth <= window.innerWidth), 'nothing wider than the window');
            assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'no "home" on the tab');
            await shot(page, `mechs-${v.tag}`);
            // robot two: its toggles' directions are owed today, said as owed (never robot one's)
            await page.getByRole('radio', { name: 'Black Bot' }).click();
            await shot(page, `mechs-two-${v.tag}`);
            const right = robots.two.mechanisms.find((m) => m.ident === 'rightToggle');
            assert.equal(await page.locator('.mech-card[data-mech="rightToggle"] .mc-row', { hasText: 'Ports' }).locator('dd').textContent(),
                `port ${right.wiring.motors[0].port}, ${right.wiring.motors[0].reversed === null ? 'which way it turns not known yet' : `turns ${right.wiring.motors[0].reversed ? 'reversed' : 'forward'}`}`);
            await page.getByRole('radio', { name: 'White Bot' }).click();
            // Test: the "what will move" screen, and NOTHING sent until its Send
            await cascade.locator('[data-test="cascade"]').click();
            const dlg = page.locator('.rk-dialog');
            assert.equal(await dlg.locator('h2').textContent(), 'Test the cascade');
            await page.waitForTimeout(1500);  // a send started on its own would have written the file by now
            assert.ok(!fs.readFileSync(path.join(made.root, FILE), 'utf8').includes('mechs'), 'nothing written before Send');
            assert.deepEqual(uploads(), [], 'nothing sent before Send');
            assert.deepEqual(await dlg.locator('.rk-moves li').allTextContents(), [`the cascade finds its bottom, goes to ${(one[0].range[0] + one[0].range[1]) / 2} in, then ${one[0].range[0]} in`]);
            assert.equal(await dlg.locator('.rk-safe').textContent(), 'The robot stays where it is — clear the mechanisms\' paths.');
            assert.equal(await dlg.locator('.btn.primary').textContent(), 'Send the test to White Bot');
            assert.deepEqual(uploads(), [], 'nothing sent before Send');
            assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'no "home" in the dialog');
            await shot(page, `mechs-test-${v.tag}`);
            await dlg.locator('.btn.primary').click();
            await page.locator('.send-panel [data-result="ok"]').waitFor({ timeout: 30000 });
            await dlg.locator('#rk-back', { hasText: 'src/team/skills_route.cpp holds the team\'s routes again' }).waitFor({ timeout: 10000 });
            assert.ok(!fs.readFileSync(path.join(made.root, FILE), 'utf8').includes('mechs'), 'written back without the test');
            assert.ok(!fs.readFileSync(path.join(made.root, 'src/team/route_maker.json'), 'utf8').includes('mechs'), 'never in the project');
            const up = uploads();
            assert.equal(up.length, 1);
            assert.match(up[0], /--description no git \| skills, left 1, cascade mechs /);
            await dlg.locator('button[aria-label="Close"]').click();
            // sent: the watch opens by itself; the robot's own lines tick the card
            await page.locator('.mech-card[data-mech="cascade"].ok').waitFor({ timeout: 30000 });
            assert.match(await cascade.locator('.mc-result').textContent(), /^Worked: every step done\d{1,2}:\d{2}/);
            assert.equal(await page.locator('.mech-card.ok, .mech-card.warn, .mech-card.bad').count(), 1, 'only the card tested');
            await shot(page, `mechs-ran-${v.tag}`);
            // a card's route opens on the Route tab at its first step on that mechanism
            await row(cascade, 'Routes').locator('button', { hasText: 'skills' }).click();
            await page.waitForFunction(() => document.querySelector('#app').dataset.tab === 'route');
            assert.equal(await page.locator('.step.selected').getAttribute('data-step'), '1');
            assert.deepEqual(await p.done(), []);
        });
    }

    test('the top bar fits with three tabs and Send from 1280 px, on every tab', async () => {
        teamFiles(TEST_RUN);
        for (const width of [1280, 1366, 1440, 1536]) {  // under 1280 the Route tab's bar with Send ran past the window before RMP1-M too
            const p = await opened({ viewport: { width, height: 800 }, scheme: 'light' });
            const { page } = p;
            for (const tab of ['route', 'controls', 'mechanisms']) {
                await page.locator(`#tab-${tab}`).click();
                for (const sel of ['#export', '#send', '#test-all']) {
                    if (await page.locator(`${sel}:visible`).count()) {
                        const b = await page.locator(sel).boundingBox();
                        assert.ok(b && b.x >= 0 && b.x + b.width <= width, `${width} px, ${tab}: ${sel} is on the screen (${JSON.stringify(b)})`);
                        assert.ok(await page.locator(sel).evaluate((e) => e.scrollWidth <= e.clientWidth + 1), `${width} px, ${tab}: ${sel}'s words fit`);
                    }
                }
                const bar = await page.locator('.topbar').evaluate((e) => ({ s: e.scrollWidth, c: e.clientWidth }));
                assert.ok(bar.s <= bar.c + 1, `${width} px, ${tab}: nothing in the top bar runs past it (${bar.s} > ${bar.c})`);
            }
            assert.deepEqual(await p.done(), []);
        }
    });

    test('an OWED port: Test says why and sends nothing; the Route, Controls and help screens never say "home"', async () => {
        teamFiles(TEST_RUN);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.evaluate((one) => window.routeBuilder.useRobots({ one: { ...one, mechanisms: one.mechanisms.map((m) => (m.ident === 'claw'
            ? { ...m, wiring: { motors: [], lines: [null] } } : m)) } }), robots.one);
        await page.locator('#tab-mechanisms').click();
        const claw = page.locator('.mech-card[data-mech="claw"]');
        assert.equal(await claw.locator('.owed').textContent(), '3-wire port not known yet: the builders\' answer');
        await claw.locator('[data-test="claw"]').click();
        assert.equal(await page.locator('.rk-dialog .rk-why').textContent(), 'The claw\'s port is not known yet (the builders\' answer): nothing is sent.');
        assert.equal(await page.locator('.rk-dialog [data-check]').count(), 0, 'no Send');
        assert.deepEqual(uploads(), []);
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        // Test every mechanism: every one but the claw, which it names
        await page.locator('#test-all').click();
        const moves = await page.locator('.rk-dialog .rk-moves li').allTextContents();
        assert.deepEqual(moves.map((w) => w.split(' ').slice(0, 3).join(' ')),
            ['the cascade finds', 'the right toggle', 'the left toggle', 'the flipper goes', 'the ramp goes']);
        assert.equal(await page.locator('#rk-left').textContent(), 'Not tested: The claw\'s port is not known yet (the builders\' answer): nothing is sent.');
        assert.match(await page.locator('.rk-dialog').textContent(), /"all mechs"/);
        assert.deepEqual(uploads(), [], 'nothing sent before Send');
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        // the Route tab with the cascade's find step selected (its card, its menu), the Controls tab, the help
        await page.locator('#tab-route').click();
        await page.locator('.step[data-step="1"]').click();
        assert.match(await page.locator('.step[data-step="1"]').textContent(), /cascade: find bottom/);
        assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'the Route tab, the find step selected');
        await page.keyboard.press('m');
        await page.locator('.mech-row[data-mech="cascade"]').click();
        assert.ok((await readable(page)).includes('Find the bottom'), 'the M menu says it');
        assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'the M menu');
        await page.keyboard.press('Escape');
        await page.keyboard.press('Escape');
        await page.locator('#tab-controls').click();
        assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'the Controls tab');
        await page.locator('#tab-route').click();
        await page.locator('button[aria-label="Help and shortcuts (?)"]').click();
        assert.match(await page.locator('#help-find').textContent(), /^Find the bottom: the motors count turns from wherever they were at power-up/);
        assert.deepEqual((await readable(page)).match(HOME_WORD), null, 'the help (its key names aside)');
        assert.deepEqual(await p.done(), []);
    });
    test('a Test sent again: the earlier run of the same name never ticks the card -- only the robot\'s new run does', async () => {
        teamFiles(TEST_RUN);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.locator('#tab-mechanisms').click();
        const cascade = page.locator('.mech-card[data-mech="cascade"]');
        await sendTest(page, 'cascade');
        await page.locator('.mech-card[data-mech="cascade"].ok').waitFor({ timeout: 30000 });
        // the robot is re-sent the same test, and never runs it: the old run of "cascade mechs" is still in the watch
        fs.writeFileSync(terminal(), BOOT_ONLY.join('\n'));
        await page.locator('[data-test="cascade"]').click();
        await page.locator('.rk-dialog .btn.primary').click();
        await cascade.locator('.mc-result', { hasText: 'Waiting for the robot\'s run…' }).waitFor({ timeout: 10000 });
        assert.deepEqual(await tone(cascade), [], 'written, not yet run: neither green nor red');
        await until(() => uploads().length === 2, 'the second upload');
        await page.locator('.rk-dialog #rk-back').waitFor({ timeout: 30000 });
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        await page.waitForTimeout(3000);  // the watch reopened and read the boot line
        assert.deepEqual(await tone(cascade), [], 'the earlier run never ticks the new test');
        assert.equal(await cascade.locator('.mc-result').textContent(), 'Waiting for the robot\'s run…');
        assert.equal(await page.locator('.mc-watch').textContent(), 'On the brain: Auton Tester → "cascade mechs" → RUN');
        // sent to robot one, but the watch reads robot TWO's brain running a test of that name: not this test's run
        fs.writeFileSync(terminal(), ON_TWO.join('\n'));
        await sendTest(page, 'cascade');
        await page.waitForTimeout(3000);
        assert.deepEqual(await tone(cascade), [], 'another robot\'s run never ticks it');
        assert.equal(await cascade.locator('.mc-result').textContent(), 'Waiting for the robot\'s run…');
        // the robot runs it this time: ticked from ITS lines
        fs.writeFileSync(terminal(), TEST_RUN.join('\n'));
        await sendTest(page, 'cascade');
        await page.locator('.mech-card[data-mech="cascade"].ok').waitFor({ timeout: 30000 });
        assert.match(await cascade.locator('.mc-result').textContent(), /^Worked: every step done\d{1,2}:\d{2}/);
        assert.equal(await page.locator('.mc-watch').count(), 0, 'its run read: the RUN hint goes');
        assert.deepEqual(await p.done(), []);
    });

    test('robot two: while its Auton Tester does not build, Test says why and sends nothing; once it builds, the test goes to robot two ONLY', async () => {
        teamFiles(TEST_RUN);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.locator('#tab-mechanisms').click();
        await page.getByRole('radio', { name: 'Black Bot' }).click();
        const owed = robots.two.testerOwed;
        assert.ok(Array.isArray(owed), 'the site\'s robots.json says what robot two\'s Auton Tester waits for');
        if (owed.length) {  // robot two today (2026-11-20): five Stage-B facts owed
            assert.match(await page.locator('#mech-notice').textContent(), /Auton Tester does not build until/);
            assert.equal(await page.locator('.btn.primary:visible').count(), 0, 'nothing to send on this robot: the notice is the one highlight');
            for (const sel of ['[data-test="cascade"]', '#test-all']) {
                await page.locator(sel).click();
                assert.ok((await page.locator('.rk-dialog .rk-why').textContent()).includes(`the first: ${owed[0]}. Nothing is sent.`), sel);
                assert.equal(await page.locator('.rk-dialog [data-check]').count(), 0, `${sel}: no Send`);
                await page.locator('.rk-dialog button[aria-label="Close"]').click();
            }
            assert.deepEqual(uploads(), [], 'nothing sent');
        }
        for (const m of robots.two.mechanisms.filter((x) => x.kind === 'pneumatic')) {  // a piston's owed levels: said as owed
            const said = await page.locator(`.mech-card[data-mech="${m.ident}"] .mc-row`, { hasText: 'Ports' }).locator('dd').textContent();
            assert.equal(said.includes('which position is which not known yet'), m.wiring.levels === null, m.name);
        }
        // a robot two whose Auton Tester builds: offered to robot two, and only to it
        await page.evaluate((two) => window.routeBuilder.useRobots({ two: { ...two, testerOwed: [] } }), robots.two);
        assert.equal(await page.locator('#mech-notice').count(), 0);
        assert.equal(await page.locator('#test-all.primary').count(), 1, 'its Auton Tester builds: Test every mechanism is the highlight again');
        await page.locator('[data-test="cascade"]').click();
        assert.deepEqual(await page.locator('.rk-dialog [data-check]').evaluateAll((els) => els.map((e) => [e.dataset.check, e.textContent])),
                         [['two', 'Send the test to Black Bot']]);
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        assert.deepEqual(await p.done(), []);
        // a checkout whose build does not take the routes (no shared route file): refused for robot two, nothing offered
        fs.rmSync(path.join(made.root, FILE));
        const q = await opened(VARIANTS[0]);
        await q.page.evaluate((two) => window.routeBuilder.useRobots({ two: { ...two, testerOwed: [] } }), robots.two);
        await q.page.locator('#tab-mechanisms').click();
        await q.page.getByRole('radio', { name: 'Black Bot' }).click();
        await q.page.locator('[data-test="cascade"]').click();
        assert.equal(await q.page.locator('.rk-dialog .rk-why').textContent(), 'Black Bot\'s build does not take the routes yet: nothing is sent.');  // RMP1-L: the robot by its name
        assert.equal(await q.page.locator('.rk-dialog [data-check]').count(), 0);
        assert.deepEqual(uploads(), []);
        assert.deepEqual(await q.done(), []);
    });

    test('the Mechanisms tab never edits the route: Delete, the arrows, Ctrl+D, Ctrl+Z and Ctrl+Y change nothing there', async () => {
        teamFiles(TEST_RUN);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.locator('.step[data-step="1"]').click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+d');  // two edits and one undo on the Route tab: there is an undo AND a redo
        await page.keyboard.press('Control+d');
        await page.keyboard.press('Control+z');
        const edited = await page.evaluate(() => JSON.stringify(window.routeBuilder.route()));
        assert.equal(JSON.parse(edited).steps.length, STEPS.length + 1);
        await page.locator('#tab-mechanisms').click();
        for (const key of ['Delete', 'Backspace', 'ArrowLeft', 'ArrowUp', 'Control+d', 'Control+z', 'Control+y', 'Control+Shift+z']) {
            await page.keyboard.press(key);
            assert.equal(await page.evaluate(() => JSON.stringify(window.routeBuilder.route())), edited, `${key} on the Mechanisms tab`);
        }
        await page.locator('#tab-route').click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+y');  // and the Route tab's own redo and undo still have them
        assert.equal((await page.evaluate(() => window.routeBuilder.route())).steps.length, STEPS.length + 2);
        await page.keyboard.press('Control+z');
        await page.keyboard.press('Control+z');
        assert.equal((await page.evaluate(() => window.routeBuilder.route())).steps.length, STEPS.length);
        assert.deepEqual(await p.done(), []);
    });
    test('a Test that never reaches the robot (the file not written, the build refused) leaves the card as it was', async () => {
        teamFiles(TEST_RUN);
        const p = await opened(VARIANTS[0]);
        const { page } = p;
        await page.locator('#tab-mechanisms').click();
        const cascade = page.locator('.mech-card[data-mech="cascade"]');
        await sendTest(page, 'cascade');
        await page.locator('.mech-card[data-mech="cascade"].ok').waitFor({ timeout: 30000 });
        const was = await cascade.locator('.mc-result').textContent();
        // the team's file lost its marked region after the page opened: nothing is written, nothing waits
        fs.writeFileSync(path.join(made.root, FILE), '// no route region\n');
        await page.locator('[data-test="cascade"]').click();
        await page.locator('.rk-dialog .btn.primary').click();
        await page.locator('.rk-dialog .rk-why', { hasText: 'Not written' }).waitFor({ timeout: 10000 });
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        assert.deepEqual(await tone(cascade), ['ok']);
        assert.equal(await cascade.locator('.mc-result').textContent(), was, 'its last result, as it was');
        assert.equal(uploads().length, 1);
        // written, but the build refuses (an OWED fact): the card goes back to its last result
        fs.writeFileSync(path.join(made.root, FILE), SKILLS);
        fs.writeFileSync(made.env.FAKE_CASE_FILE, 'owed');
        await page.locator('[data-test="cascade"]').click();
        await page.locator('.rk-dialog .btn.primary').click();
        await page.locator('.rk-dialog .send-panel [data-result="error"]').waitFor({ timeout: 30000 });
        await page.locator('.rk-dialog #rk-back').waitFor({ timeout: 30000 });
        await page.locator('.rk-dialog button[aria-label="Close"]').click();
        assert.deepEqual(await tone(cascade), ['ok']);
        assert.equal(await cascade.locator('.mc-result').textContent(), was);
        assert.equal(await page.locator('.mc-watch').count(), 0, 'no "run it on the brain" for a test that is not there');
        assert.equal((await page.evaluate(() => window.routeBuilder.mechanisms())).pending, null);
        assert.deepEqual(await p.done(), []);
    });
});
