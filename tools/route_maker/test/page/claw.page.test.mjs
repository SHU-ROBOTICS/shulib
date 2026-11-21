// claw.page.test.mjs — RMP1-O in a browser: the claw's cross on the start and a step's end (drawn where the claw IS:
// 15/2 - 1.232 in ahead, on the centre line), a drive dragged by its cross (the claw lands on the pointer), Ctrl while
// dragging the point, "Place by the claw" typed (and refused when too close), the inspector's line, no cross on a
// robot with no claw. ROUTE_MAKER_SHOTS: the selected drive before a claw step, light and dark, at 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, skip, launch, serve, openPage, noStructures, SHOTS, BROWSER } from './harness.mjs';
import { plannedPoses } from '../../src/model/route.mjs';

const FORWARD = 7.5 - 1.232;  // the team lead's measure (2026-11-20), never read from the page
let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());

/** Screen point of field (x, y), whole pixels (what a mouse sends), and the field point under it. */
const at = (page, x, y) => page.evaluate(([fx, fy]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const m = document.querySelector('.field-svg').getScreenCTM();
    const s = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(m);
    const px = { x: Math.round(s.x), y: Math.round(s.y) };
    const back = new DOMPoint(px.x, px.y).matrixTransform(m.inverse());
    return { ...px, field: { x: (back.x - pic.originPx[0]) / pic.pxPerIn, y: (pic.originPx[1] - back.y) / pic.pxPerIn } };
}, [x, y]);

/** Every cross: its data-x/y, and the field point its two lines CROSS at (what the student sees). */
const crosses = (page) => page.evaluate(() => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    return [...document.querySelectorAll('.field-svg [data-claw]')].map((g) => {
        const [a, b] = [...g.querySelectorAll('line')].map((l) => ['x1', 'y1', 'x2', 'y2'].map((k) => Number(l.getAttribute(k))));
        const mid = [(a[0] + a[2] + b[0] + b[2]) / 4, (a[1] + a[3] + b[1] + b[3]) / 4];
        return { x: Number(g.dataset.x), y: Number(g.dataset.y), index: g.dataset.index, handle: g.dataset.handle || null, big: g.classList.contains('big'),
                 drawn: { x: (mid[0] - pic.originPx[0]) / pic.pxPerIn, y: (pic.originPx[1] - mid[1]) / pic.pxPerIn } };
    });
});
/** A drawn robot (its square and front marker in `g`): its centre, the way it faces, and its cross, in field inches. */
const drawnRobot = (page, g, body, nose) => page.evaluate(([gs, bs, ns]) => {
    const pic = JSON.parse(document.getElementById('shulib-field').textContent);
    const f = ([x, y]) => ({ x: (x - pic.originPx[0]) / pic.pxPerIn, y: (pic.originPx[1] - y) / pic.pxPerIn });
    const pts = (el) => el.getAttribute('points').trim().split(/\s+/).map((q) => f(q.split(',').map(Number)));
    const root = document.querySelector(gs);
    const sq = pts(root.querySelector(bs)).slice(0, 4);  // the square's corners (RMP1-Q: its open front follows)
    const c = { x: sq.reduce((a, q) => a + q.x, 0) / sq.length, y: sq.reduce((a, q) => a + q.y, 0) / sq.length };
    const tip = pts(root.querySelector(ns))[0];
    const cl = root.querySelector('[data-claw]');
    return { c, h: (Math.atan2(tip.y - c.y, tip.x - c.x) * 180) / Math.PI, claw: cl && { x: Number(cl.dataset.x), y: Number(cl.dataset.y) } };
}, [g, body, nose]);
const ahead = (p, h) => ({ x: p.x + FORWARD * Math.cos((h * Math.PI) / 180), y: p.y + FORWARD * Math.sin((h * Math.PI) / 180) });
const near = (a, b, tol, what) => assert.ok(Math.hypot(a.x - b.x, a.y - b.y) <= tol, `${what}: (${a.x}, ${a.y}) vs (${b.x}, ${b.y})`);
const START = { x: -36, y: -55.7, headingDeg: 0 };

async function withRoute(steps, { robots, ...opts } = {}) {
    const p = await openPage(browser, site.base, opts);
    await p.page.getByRole('button', { name: 'Got it' }).click();
    await p.page.evaluate((r) => window.routeBuilder.useRobots(r), robots || noStructures());
    const project = { schemaVersion: 1, madeWith: 'test', routes: [{ name: 'skills', kind: 'skills', robot: 'one', mirroredFor: 'two', together: false,
        start: START, endActionAtS: 57, hardStopAtS: 59, steps }], robots: {}, exports: {} };
    await p.page.evaluate((t) => window.routeBuilder.useMemoryFolder({ 'src/team/route_maker.json': t }), `${JSON.stringify(project, null, 2)}\n`);
    return p;
}
async function drag(page, from, to, { ctrl = false } = {}) {
    await page.mouse.move(from.x, from.y);
    await page.mouse.down();
    if (ctrl) {
        await page.keyboard.down('Control');
    }
    await page.mouse.move((from.x + to.x) / 2, (from.y + to.y) / 2, { steps: 4 });
    await page.mouse.move(to.x, to.y, { steps: 4 });
    await page.mouse.up();
    if (ctrl) {
        await page.keyboard.up('Control');
    }
}
const ahead3 = (p, h, k) => ({ x: p.x + k * Math.cos((h * Math.PI) / 180), y: p.y + k * Math.sin((h * Math.PI) / 180) });
const clawOf = async (page, i) => (await crosses(page)).find((c) => c.handle === 'claw' && c.index === String(i));
const snapOff = (page) => page.locator('.tool', { hasText: 'Snap' }).click();
const pickStep = (page, n) => page.locator(`.step[data-step="${n - 1}"]`).click();

test.describe('RMP1-O in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('the cross: at the start and every drive\'s end, 6.268 in ahead on the centre line, DRAWN there; bigger before a claw step', async () => {
        const baked = JSON.parse(fs.readFileSync(path.join(SITE, 'robots.json'), 'utf8')).robots;
        for (const id of ['one', 'two']) {
            assert.deepEqual(baked[id].clawSpot, { forwardIn: 6.268, sideIn: 0 }, `robot ${id}'s claw spot`);
        }
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }, { do: 'set', mechanism: 'claw', position: 'closed' },
                                   { do: 'reverseTo', x: -30, y: -30, faceFirst: true }]);
        const { page } = p;
        const r = await route(page);
        const h1 = (Math.atan2(-20 + 55.7, 0 + 36) * 180) / Math.PI;
        const h2 = (Math.atan2(-30 + 20, -30 - 0) * 180) / Math.PI + 180;
        const want = [ahead(START, 0), ahead(r.steps[0], h1), ahead(r.steps[2], h2)];
        const got = await crosses(page);
        for (const w of want) {
            const c = got.find((g) => Math.hypot(g.x - w.x, g.y - w.y) < 0.01);
            assert.ok(c, `a cross at (${w.x.toFixed(3)}, ${w.y.toFixed(3)}): ${JSON.stringify(got.map((g) => [g.x, g.y]))}`);
            near(c.drawn, w, 0.01, 'its lines cross there');
        }
        assert.ok(!got.some((g) => g.big), 'nothing selected: none big');
        await pickStep(page, 1);
        const sel = (await crosses(page)).filter((c) => c.big);
        assert.equal(sel.length, 1, 'the drive before the claw step: its cross big');
        near(sel[0], want[1], 0.01, 'the big one');
        assert.equal(sel[0].handle, null, 'the big one takes no pointer (a badge there stays clickable)');
        assert.ok((await crosses(page)).some((c) => c.handle === 'claw' && c.index === '0' && Math.hypot(c.x - want[1].x, c.y - want[1].y) < 0.01),
                  'the drive\'s own cross under it is the handle');
        assert.deepEqual(await p.done(), []);
    });

    test('drag by the cross: the claw lands on the pointer (0.1 in), the centre computed back along the leg; Ctrl on the point does the same', async () => {
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }, { do: 'reverseTo', x: -24, y: -10, faceFirst: true }]);
        const { page } = p;
        await snapOff(page);
        for (const [n, how] of [[1, 'cross'], [2, 'cross'], [1, 'ctrl']]) {
            const before = (await crosses(page)).find((c) => c.handle === 'claw' && c.index === String(n - 1));
            const r0 = await route(page);
            const from = how === 'cross' ? await at(page, before.x, before.y) : await at(page, r0.steps[n - 1].x, r0.steps[n - 1].y);
            const to = await at(page, n === 1 ? 10.37 : -40.2, n === 1 ? -9.81 : -2.6);
            await drag(page, from, to, { ctrl: how === 'ctrl' });
            const r = await route(page);
            const now = (await crosses(page)).find((c) => c.handle === 'claw' && c.index === String(n - 1));
            near(now, to.field, 0.1, `step ${n} by the ${how}: the claw on the pointer`);
            near(now.drawn, to.field, 0.1, `step ${n} by the ${how}: drawn there`);
            const prev = n === 1 ? START : r.steps[0];
            const d = Math.hypot(r.steps[n - 1].x - prev.x, r.steps[n - 1].y - prev.y);
            const dq = Math.hypot(to.field.x - prev.x, to.field.y - prev.y);
            assert.ok(Math.abs(Math.abs(d - dq) - FORWARD) < 0.1, `step ${n}: the centre ${FORWARD} in along the leg from the claw (${d} vs ${dq})`);
            assert.ok(n === 1 ? d < dq : d > dq, n === 1 ? 'driving: the centre short of the claw' : 'backing: the centre past it');
            await page.keyboard.press('Control+z');
            assert.deepEqual([(await route(page)).steps[n - 1].x, (await route(page)).steps[n - 1].y], [r0.steps[n - 1].x, r0.steps[n - 1].y], 'one undo');
        }
        assert.deepEqual(await p.done(), []);
    });

    test('every robot drawn carries ITS cross: a Straight\'s end and a curve\'s (and their lines), robot two\'s mirrored start (its own spot); the claw step ITSELF selected: one big cross', async () => {
        // BUG CAUGHT (review): no cross or line on a Straight or a curve; robot two drawn with robot one's spot; no big
        // cross when the claw step itself is selected; the backing line not said
        const robots = noStructures();
        robots.two.clawSpot = { forwardIn: 3, sideIn: 0 };  // robot two's own (told apart from robot one's 6.268)
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }, { do: 'set', mechanism: 'claw', position: 'closed' },
                                   { do: 'reverseTo', x: -12, y: -36, faceFirst: true }, { do: 'forward', inches: 12 },
                                   { do: 'curveTo', points: [{ x: 10, y: -30 }, { x: 24, y: -12 }] }], { robots });
        const { page } = p;
        const plan = plannedPoses(await route(page));
        const got = await crosses(page);
        for (const i of [3, 4]) {
            const w = ahead(plan[i].to, plan[i].to.headingDeg);
            assert.ok(got.some((c) => Math.hypot(c.x - w.x, c.y - w.y) < 0.01 && Math.hypot(c.drawn.x - w.x, c.drawn.y - w.y) < 0.01), `step ${i + 1}'s cross`);
            await pickStep(page, i + 1);
            assert.equal(await page.locator('.pane.right .claw-line').textContent(), `The claw reaches (${+w.x.toFixed(3)}, ${+w.y.toFixed(3)}).`, `step ${i + 1}'s line`);
        }
        await pickStep(page, 3);
        assert.match(await page.locator('.pane.right .claw-line').textContent(), /^The claw reaches \(-?[\d.]+, -?[\d.]+\), after the robot backs over it \(to grab, drive forward\)\.$/);
        const one = await drawnRobot(page, '.g-start', '.g-robot', '.g-front');
        near(one.claw, ahead3(one.c, one.h, FORWARD), 0.01, 'robot one\'s start: its own 6.268');
        const two = await drawnRobot(page, '.g-twin', '.g-twin-robot', '.g-twin-front');
        near(two.claw, ahead3(two.c, two.h, 3), 0.01, 'robot two\'s mirrored start: ITS spot (3 in)');
        await pickStep(page, 2);
        const big = (await crosses(page)).filter((c) => c.big);
        assert.equal(big.length, 1, 'the claw step itself selected: one big cross');
        near(big[0], ahead(plan[0].to, plan[0].to.headingDeg), 0.01, 'where it grabs');
        assert.deepEqual(await p.done(), []);
    });

    test('"Place by the claw" on: a plain drag of a drive\'s point puts the CLAW on the pointer, not snapped (Snap on, beside the grid); a curve\'s point still drags as a point', async () => {
        // BUG CAUGHT (review): the switch ignored by a drag (the centre on the pointer, 6.27 in off); the claw pulled onto the
        // centre's grid (up to 1.8 in off); the switch taking over a curve's point (it lands 6.27 in behind the pointer)
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }, { do: 'curveTo', points: [{ x: 20, y: -10 }, { x: 30, y: 10 }] }]);
        const { page } = p;
        await pickStep(page, 1);
        await page.locator('.pane.right input[name="byClaw"]').check();
        const r0 = await route(page);
        const to = await at(page, 11.3, -11.2);  // 0.7 and 0.8 in from the 12 in grid lines: Snap pulls a point onto them
        await drag(page, await at(page, r0.steps[0].x, r0.steps[0].y), to);
        near(await clawOf(page, 0), to.field, 0.1, 'the claw on the pointer');
        const q = await at(page, 29.6, 17.7);  // far from every snap line
        await drag(page, await at(page, 30, 10), q);
        const r = await route(page);
        near(r.steps[1].points[1], q.field, 0.1, 'the curve\'s point itself on the pointer');
        assert.deepEqual(r.steps[1].points[0], r0.steps[1].points[0]);
        assert.deepEqual(await p.done(), []);
    });

    test('a curve by its cross: its LAST point moves (the first stays) and the claw is on the pointer; Ctrl on its FIRST point drags that point only', async () => {
        const p = await withRoute([{ do: 'curveTo', points: [{ x: -20, y: -40 }, { x: 0, y: -20 }] }]);
        const { page } = p;
        await snapOff(page);
        const to = await at(page, 12.2, -6.3);
        await drag(page, await at(page, (await clawOf(page, 0)).x, (await clawOf(page, 0)).y), to);
        const r1 = await route(page);
        assert.deepEqual(r1.steps[0].points[0], { x: -20, y: -40 }, 'the first point stays');
        near(await clawOf(page, 0), to.field, 0.1, 'the claw on the pointer');
        const to2 = await at(page, -24.4, -35.1);
        await drag(page, await at(page, -20, -40), to2, { ctrl: true });
        const r2 = await route(page);
        assert.deepEqual(r2.steps[0].points[1], r1.steps[0].points[1], 'the last point stays');
        near(r2.steps[0].points[0], to2.field, 0.1, 'the first point on the pointer');
        assert.deepEqual(await p.done(), []);
    });

    test('too close, by the cross or Ctrl on the point: refused in words at the pointer, the step kept', async () => {
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }]);
        const { page } = p;
        await snapOff(page);
        const r0 = await route(page);
        const to = await at(page, START.x + 3, START.y + 2);  // 3.6 in from where the drive starts: under the claw's reach
        for (const from of [await at(page, (await clawOf(page, 0)).x, (await clawOf(page, 0)).y), await at(page, 0, -20)]) {
            await page.mouse.move(from.x, from.y);
            await page.mouse.down();
            await page.keyboard.down('Control');
            await page.mouse.move(to.x, to.y);
            assert.match(await page.locator('.g-tip:not([hidden])').textContent(), /^Too close: the claw is 6\.268 in ahead of the centre\./);
            await page.mouse.up();
            await page.keyboard.up('Control');
            assert.deepEqual((await route(page)).steps[0], r0.steps[0], 'the step kept');
        }
        assert.deepEqual(await p.done(), []);
    });

    test('"Place by the claw" typed on a reverseTo: the claw lands where it was typed (backing: the centre past it)', async () => {
        const p = await withRoute([{ do: 'reverseTo', x: 0, y: -20, faceFirst: true }]);
        const { page } = p;
        await pickStep(page, 1);
        await page.locator('.pane.right input[name="byClaw"]').check();
        const y0 = Number(await page.locator('.pane.right input[name="clawY"]').inputValue());
        await page.locator('.pane.right input[name="clawX"]').fill('-10');
        await page.locator('.pane.right input[name="clawX"]').press('Enter');
        near(await clawOf(page, 0), { x: -10, y: y0 }, 0.01, 'the claw where it was typed');
        const s = (await route(page)).steps[0];
        assert.ok(Math.hypot(s.x - START.x, s.y - START.y) > Math.hypot(-10 - START.x, y0 - START.y), 'the centre past the claw');
        assert.deepEqual(await p.done(), []);
    });

    test('the inspector: one line says where the claw reaches; "Place by the claw" types the claw\'s point (too close: refused in words)', async () => {
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }]);
        const { page } = p;
        await pickStep(page, 1);
        const h1 = (Math.atan2(-20 + 55.7, 0 + 36) * 180) / Math.PI;
        const c = ahead({ x: 0, y: -20 }, h1);
        const line = page.locator('.pane.right .claw-line');
        assert.equal(await line.textContent(), `The claw reaches (${+c.x.toFixed(3)}, ${+c.y.toFixed(3)}).`);
        await page.locator('.pane.right input[name="byClaw"]').check();
        const x = page.locator('.pane.right input[name="clawX"]');
        assert.equal(Number(await x.inputValue()), +c.x.toFixed(3));
        await x.fill('12');
        await x.press('Enter');
        const s = (await route(page)).steps[0];
        const claw = (await crosses(page)).find((g) => g.handle === 'claw');
        near(claw, { x: 12, y: +c.y.toFixed(3) }, 0.01, 'the claw where it was typed');
        assert.match(await line.textContent(), new RegExp(`^The robot's centre: \\(${s.x}, ${s.y}\\)\\.$`));
        await page.locator('.pane.right input[name="clawX"]').fill('-33');
        await page.locator('.pane.right input[name="clawX"]').press('Enter');  // a change re-draws the form: one box at a time
        const s2 = (await route(page)).steps[0];
        await page.locator('.pane.right input[name="clawY"]').fill('-52');
        await page.locator('.pane.right input[name="clawY"]').press('Enter');
        assert.match(await page.locator('.pane.right [role="alert"]:not([hidden])').first().textContent(), /^Too close: the claw is 6\.268 in ahead of the centre\./);
        assert.deepEqual((await route(page)).steps[0], s2, 'refused: the step kept');
        assert.deepEqual(await p.done(), []);
    });

    test('a robot with no claw: no cross, no line, no switch', async () => {
        const robots = noStructures();
        for (const r of Object.values(robots)) {
            delete r.clawSpot;
        }
        const p = await withRoute([{ do: 'driveTo', x: 0, y: -20, faceFirst: true }], { robots });
        const { page } = p;
        await pickStep(page, 1);
        assert.equal(await page.locator('.field-svg [data-claw]').count(), 0);
        assert.equal(await page.locator('.pane.right .claw-line').count(), 0);
        assert.equal(await page.locator('.pane.right input[name="byClaw"]').count(), 0);
        assert.deepEqual(await p.done(), []);
    });

    test('screenshots (ROUTE_MAKER_SHOTS): the drive before a claw step, light and dark, 1440 and 1280', { skip: !SHOTS && 'set ROUTE_MAKER_SHOTS' }, async () => {
        for (const [scheme, width, height] of [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]]) {
            // heading 0: the cross ahead, clear of the badges stacked above the spot
            const p = await withRoute([{ do: 'driveTo', x: -12, y: -55.7, faceFirst: true }, { do: 'set', mechanism: 'claw', position: 'closed' }],
                                      { scheme, viewport: { width, height } });
            await pickStep(p.page, 1);
            fs.mkdirSync(SHOTS, { recursive: true });
            await p.page.waitForTimeout(400);
            await p.page.screenshot({ path: path.join(SHOTS, `${BROWSER}-claw-${scheme}-${width}.png`) });
            const spot = await at(p.page, -12, -60);  // zoomed on the robot (the guide's picture)
            await p.page.mouse.move(spot.x, spot.y);
            for (let k = 0; k < 5; ++k) {
                await p.page.mouse.wheel(0, -100);
            }
            await p.page.mouse.move(5, 5);
            await p.page.waitForTimeout(4500);  // the toasts gone
            await p.page.screenshot({ path: path.join(SHOTS, `${BROWSER}-claw-zoom-${scheme}-${width}.png`) });
            await p.page.locator('.pane.right input[name="byClaw"]').check();
            await p.page.waitForTimeout(300);
            await p.page.locator('.pane.right').screenshot({ path: path.join(SHOTS, `${BROWSER}-claw-by-${scheme}-${width}.png`) });
            assert.deepEqual(await p.done(), []);
        }
    });
});
