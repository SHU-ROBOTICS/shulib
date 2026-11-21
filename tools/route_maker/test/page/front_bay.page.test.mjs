// front_bay.page.test.mjs — RMP1-Q in a browser: each robot drawn with its open front (the outline the goals are checked
// with), robot x still a square; a goal centred on the claw: no issue, Export writes it, a touch at the bay's back offers
// "on purpose"; PREVIEW's THROUGH reads each robot's bay; White Bot's working route loaded: its goal checks, and step 5
// placed by its claw on the goal's centre clears it. ROUTE_MAKER_SHOTS: a goal in the bay, light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { SITE, REPO, skip, launch, serve, openPage, SHOTS, BROWSER } from './harness.mjs';
import { fixture, standinScript } from '../standin_engine.mjs';

const LIP = 3.22;
const SPOT = 7.5 - 1.232;  // the claw's spot ahead of the centre (the team lead's measure), never read from the page
const TEAM = fs.readFileSync(path.join(path.dirname(fileURLToPath(import.meta.url)), '..', 'fixtures', 'projects', 'white_bot.TEST-v10-2026-11-20.json'), 'utf8');
const RED = { x: 47.091, y: -23.547 };  // the red goal by the right wall (A10)
const FIRST = { x: 50.369, y: -18.204 };  // White Bot's first drive from (60, -2.505), placed by its claw on RED's centre
let browser;
let site;

async function withProject(text, opts = {}) {
    const p = await openPage(browser, site.base, opts);
    await p.page.getByRole('button', { name: 'Got it' }).click();
    await p.page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), text);
    return p;
}
const project = (start, steps) => `${JSON.stringify({ schemaVersion: 1, madeWith: 'test', routes: [{ name: 'skills', kind: 'skills', robot: 'one',
    mirroredFor: 'two', together: false, start, endActionAtS: 57, hardStopAtS: 59, steps }], robots: {}, exports: {} }, null, 2)}\n`;
/** A drawn polygon's points in field inches, in the robot's frame at `pose` (x forward, y left). */
const local = (page, sel, pose) => page.evaluate(([q, p]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const r = (p.headingDeg * Math.PI) / 180;
    return document.querySelector(q).getAttribute('points').trim().split(/\s+/).map((v) => {
        const [px, py] = v.split(',').map(Number);
        const [x, y] = [(px - pic.originPx[0]) / pic.pxPerIn - p.x, (pic.originPx[1] - py) / pic.pxPerIn - p.y];
        return { x: x * Math.cos(r) + y * Math.sin(r), y: -x * Math.sin(r) + y * Math.cos(r) };
    });
}, [sel, pose]);
const rco = async (page) => (await page.evaluate(() => window.routeBuilder.issues())).filter((i) => /^RCO/.test(i.code))
    .map((i) => `${i.code} ${i.stepIndex + 1} ${i.words.split(':')[0]}`);

test.describe('RMP1-Q in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('the robot is drawn with its open front: 3.22 in either side, back to the claw\'s spot and round; robot x: a square', async () => {
        const baked = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
        for (const id of ['one', 'two']) {
            assert.deepEqual(baked[id].frontBay, { centreForwardIn: SPOT, lipRadiusIn: LIP, bodyRadiusIn: 2.8 }, `robot ${id}'s bay`);
        }
        const start = { x: -36, y: -40, headingDeg: 30 };
        const p = await withProject(project(start, [{ do: 'driveTo', x: -10, y: -40, faceFirst: true }]));
        const { page } = p;
        const pts = await local(page, '.g-start .g-robot', start);
        const near = (a, b) => Math.hypot(a.x - b.x, a.y - b.y) < 0.02;
        for (const c of [{ x: 7.5, y: 7.5 }, { x: -7.5, y: 7.5 }, { x: -7.5, y: -7.5 }, { x: 7.5, y: -7.5 }, { x: 7.5, y: -LIP }, { x: 7.5, y: LIP }]) {
            assert.ok(pts.some((q) => near(q, c)), `a corner at (${c.x}, ${c.y}): ${JSON.stringify(pts.slice(0, 6))}`);
        }
        const arc = pts.filter((q) => q.x < 7.4 && Math.abs(q.y) < 7.4);
        assert.ok(arc.length > 50 && arc.every((q) => Math.abs(Math.hypot(q.x - SPOT, q.y) - LIP) < 0.02 && q.x <= SPOT + 0.02), 'the back: round the claw\'s spot');
        const ghost = await local(page, '.g-ghost', { x: -10, y: -40, headingDeg: 0 });
        assert.equal(ghost.length, pts.length, 'the step\'s end too');
        assert.equal((await local(page, '.g-twin .g-twin-robot', start)).length, pts.length, 'robot two (mirrored) too');
        await page.evaluate((x) => window.routeBuilder.useRobots({ one: { ...x, id: 'one' } }), JSON.parse(fs.readFileSync(
            path.join(REPO, 'test', 'goldens', 'route_maker', 'robots', 'x.json'), 'utf8')));
        assert.equal((await local(page, '.g-start .g-robot', start)).length, 4, 'a robot with no bay: its square');
        assert.deepEqual(await p.done(), []);
    });

    test('a red goal centred on the claw: no issue, and Export writes it; 0.1 in deeper a touch, "on purpose" offered', async () => {
        const p = await withProject(project({ x: 60, y: -2.505, headingDeg: -120 }, [{ do: 'driveTo', ...FIRST, faceFirst: true }]));
        const { page } = p;
        assert.deepEqual((await page.evaluate(() => window.routeBuilder.issues())).map((i) => i.code), []);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+e');
        await page.locator('.drawer').waitFor();
        assert.equal(await page.locator('.drawer .banner.error').count(), 0, 'no refusal');
        assert.match(await page.locator('.drawer pre.code').textContent(), /ROUTE MAKER[\s\S]*driveTo/);
        await page.keyboard.press('Escape');
        await page.locator('.step[data-step="0"]').click();
        for (const [k, v] of [['x', 50.317], ['y', -18.289]]) {  // 0.1 in further along its line
            await page.locator(`.pane.right input[name="${k}"]`).fill(String(v));
            await page.locator(`.pane.right input[name="${k}"]`).press('Enter');
        }
        assert.deepEqual(await rco(page), ['RCO2 1 The robot touches the red goal by the right wall here (0.1 in into it). If it is meant to (placing a cone), say so below; if not, give it more room.']);
        const sw = page.locator('.pane.right input[name="touches-red-goal-right-wall"]');
        assert.equal(await sw.count(), 1, 'the switch: offered for the touch at the bay\'s back');
        await sw.check();
        assert.deepEqual(await rco(page), []);
        assert.deepEqual(await p.done(), []);
    });

    test('a black goal centred on the claw: the bay drawn with the body\'s 2.80 in, the radius it is checked with', async () => {
        const end = { x: 50.369, y: 18.204 };  // the mirror image of FIRST: the black goal by the right wall
        const p = await withProject(project({ x: 60, y: 2.505, headingDeg: 120 }, [{ do: 'driveTo', ...end, faceFirst: true }]));
        const { page } = p;
        const ghost = await local(page, '.g-ghost', { ...end, headingDeg: (Math.atan2(end.y - 2.505, end.x - 60) * 180) / Math.PI });
        const arc = ghost.filter((q) => q.x < 7.4 && Math.abs(q.y) < 7.4);
        assert.ok(arc.length > 50 && arc.every((q) => Math.abs(Math.hypot(q.x - SPOT, q.y) - 2.8) < 0.02), JSON.stringify(arc.slice(0, 3)));
        assert.deepEqual(await rco(page), ['RCO2 1 The robot touches the black goal by the right wall here (0.4 in into it). If it is meant to (placing a cone), say so below; if not, give it more room.']);
        assert.deepEqual(await p.done(), []);
    });

    test('PREVIEW reads each robot\'s own bay: a run ending with the red goal in it is not THROUGH; robots without one, THROUGH', async () => {
        // two_starts' recorded run (a clean route), its drive's last paces moved to drive straight at the red goal by the
        // red wall until it is centred on the claw (heading 0): the page's own check of the run, with the site's structures
        const f = fixture('two_starts');
        const goal = { x: 23.547, y: -47.091 };
        const samples = f.result.one.samples.map((q, i) => (i <= 140 ? q
            : [q[0], goal.x - SPOT - (12 * Math.max(0, 154 - i)) / 14, goal.y, 0]));
        const play = async (robots) => {
            const p = await openPage(browser, site.base);
            const { page } = p;
            await page.evaluate((text) => window.routeBuilder.useEngine({ text }), standinScript(['two_starts'], { patch: { two_starts: { samples } } }));
            if (robots) {
                await page.evaluate((r) => window.routeBuilder.useRobots(r), robots);
            }
            await page.getByRole('button', { name: 'Got it' }).click();
            const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
            await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(f.text, 'utf8') });
            await page.waitForFunction((want) => JSON.stringify(window.routeBuilder.route()) === want, JSON.stringify(f.route));
            assert.deepEqual((await page.evaluate(() => window.routeBuilder.issues())).filter((x) => /^RCO/.test(x.code)), [], 'the PLAN is clear');
            await page.locator('.field-svg').focus();
            await page.keyboard.press('Control+Enter');
            await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open');
            await page.evaluate(() => window.routeBuilder.pause());
            await page.keyboard.press('End');
            return { p, page, dock: await page.locator('.dock').textContent(), faded: await page.locator('.g-run.one.through').count() };
        };
        const mine = await play(null);
        assert.doesNotMatch(mine.dock, /THROUGH/, mine.dock);
        assert.equal(mine.faded, 0);
        assert.deepEqual(await mine.p.done(), []);
        const baked = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
        const none = Object.fromEntries(Object.entries(baked).map(([id, r]) => [id, { ...r, frontBay: undefined }]));
        const theirs = await play(none);
        assert.match(theirs.dock, /FINISHED, but THROUGH the red goal by the red wall/, 'no bay: THROUGH');
        assert.equal(theirs.faded, 1);
        assert.deepEqual(await theirs.p.done(), []);
    });

    test('White Bot\'s working route: its goal checks read the bay; step 5 placed by its claw on the goal\'s centre is clear', async () => {
        const p = await withProject(TEAM);
        const { page } = p;
        assert.deepEqual(await rco(page), [
            'RCO1 5 Where this step ends, the red goal by the right wall is 3.5 in past the edge of the robot\'s open front',
            'RCO1 9 Turning in place here, the edge of the robot\'s open front goes 3.5 in into the red goal by the right wall',
            'RCO1 15 Where this step ends, the robot\'s 15 in square is 1.1 in into the red goal by the right wall']);
        await page.locator('.step[data-step="4"]').click();
        await page.locator('.pane.right input[name="byClaw"]').check();
        for (const [k, v] of [['clawX', RED.x], ['clawY', RED.y]]) {
            await page.locator(`.pane.right input[name="${k}"]`).fill(String(v));
            await page.locator(`.pane.right input[name="${k}"]`).press('Enter');
        }
        const said = await rco(page);
        assert.ok(!said.some((w) => w.startsWith('RCO1 5 ')), JSON.stringify(said));
        assert.deepEqual(said.map((w) => w.split(' ')[1]), ['9', '15'], 'step 9 turns with the goal in the bay; 15 is the robot\'s corner');
        const claw = await page.evaluate(() => [...document.querySelectorAll('.field-svg [data-claw][data-handle="claw"]')]
            .filter((g) => g.dataset.index === '4').map((g) => ({ x: Number(g.dataset.x), y: Number(g.dataset.y) }))[0]);
        assert.ok(Math.hypot(claw.x - RED.x, claw.y - RED.y) < 0.002, JSON.stringify(claw));
        await page.locator('.pane.right input[name="byClaw"]').uncheck();
        assert.deepEqual(await p.done(), []);
    });

    test('screenshots (ROUTE_MAKER_SHOTS): a goal in the bay, light and dark, 1440 and 1280', { skip: !SHOTS && 'set ROUTE_MAKER_SHOTS' }, async () => {
        for (const [scheme, width, height] of [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]]) {
            // White Bot's start and first drive, placed by its claw on the red goal's centre (robot two's mirror image: off the picture)
            const one = JSON.parse(project({ x: 60, y: -2.505, headingDeg: -120 }, [{ do: 'driveTo', ...FIRST, faceFirst: true }]));
            const p = await withProject(`${JSON.stringify(one, null, 2)}\n`, { scheme, viewport: { width, height } });
            const { page } = p;
            await page.locator('.step[data-step="0"]').click();
            const screen = (fx, fy) => page.evaluate(([x, y]) => {
                const pic = JSON.parse(document.getElementById('shulib-field').textContent);
                const s = new DOMPoint(pic.originPx[0] + x * pic.pxPerIn, pic.originPx[1] - y * pic.pxPerIn)
                    .matrixTransform(document.querySelector('.field-svg').getScreenCTM());
                return { x: s.x, y: s.y };
            }, [fx, fy]);
            let g = await screen(RED.x + 2, RED.y + 3);
            await page.mouse.move(g.x, g.y);
            for (let k = 0; k < 6; ++k) {
                await page.mouse.wheel(0, -100);
            }
            g = await screen(RED.x + 2, RED.y + 3);  // pan it to the pane's middle, dragging the empty field beside it
            const box = await page.locator('.field-svg').boundingBox();
            const from = await screen(RED.x - 12, RED.y - 6);
            await page.mouse.move(from.x, from.y);
            await page.mouse.down();
            await page.mouse.move(from.x + (box.x + box.width / 2 - g.x) / 2, from.y + (box.y + box.height / 2 - g.y) / 2, { steps: 4 });
            await page.mouse.move(from.x + box.x + box.width / 2 - g.x, from.y + box.y + box.height / 2 - g.y, { steps: 4 });
            await page.mouse.up();
            await page.mouse.move(5, 5);
            await page.waitForTimeout(4500);
            fs.mkdirSync(SHOTS, { recursive: true });
            await page.screenshot({ path: path.join(SHOTS, `${BROWSER}-bay-${scheme}-${width}.png`) });
            assert.deepEqual((await page.evaluate(() => window.routeBuilder.issues())).map((i) => i.code), []);
            assert.deepEqual(await p.done(), []);
        }
    });
});
