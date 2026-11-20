// obstacles.page.test.mjs — RMP1-H in a browser: the field's solid structures drawn ON their elements in the real
// picture (field.webp, read here), a drive through a goal red with its one-click way around, a touch amber and
// "touches it on purpose", PREVIEW's THROUGH a structure. ROUTE_MAKER_SHOTS: the screens, light and dark, 1440 and 1280.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, SHOTS, skip, launch, serve, openPage, shot } from './harness.mjs';
import { fixture, standinScript } from '../standin_engine.mjs';
import { squareCorners } from '../../src/model/geometry.mjs';

const VARIANTS = (SHOTS ? [['light', 1440, 900], ['dark', 1440, 900], ['light', 1280, 800], ['dark', 1280, 800]] : [['light', 1440, 900]])
    .map(([scheme, width, height]) => ({ scheme, viewport: { width, height }, tag: `${scheme}-${width}` }));
let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());
const structureIssues = async (page) => (await page.evaluate(() => window.routeBuilder.issues())).filter((x) => /^RCO/.test(x.code));
const highlighted = (page) => page.evaluate(() => [...document.querySelectorAll('.g-obstacle.hit-error, .g-obstacle.hit-warn')].map((x) => x.dataset.obstacle));
const switches = (page) => page.locator('.pane.right .touch-purpose input');

async function clickField(page, x, y) {
    const at = await page.evaluate(([fx, fy]) => {
        const pic = JSON.parse(document.getElementById('shulib-field').textContent);
        const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn)
            .matrixTransform(document.querySelector('.field-svg').getScreenCTM());
        return { x: pt.x, y: pt.y };
    }, [x, y]);
    await page.mouse.click(at.x, at.y);
}
async function typeIn(page, name, value) {
    await page.locator(`.pane.right input[name="${name}"]`).fill(String(value));
    await page.locator(`.pane.right input[name="${name}"]`).press('Enter');
}
/** The start typed exactly, "one at a time" answered (the skills route asks). */
async function startAt(page, x, y, h) {
    await page.getByRole('button', { name: 'Got it' }).click();
    await clickField(page, x, y);
    await typeIn(page, 'startX', x);
    await typeIn(page, 'startY', y);
    await typeIn(page, 'startH', h);
    await page.locator('.pane.right [data-together="false"]').click();
}
/** A drive to (x, y): clicked, then typed exactly. */
async function driveTo(page, x, y) {
    await page.locator('.field-svg').focus();
    await page.keyboard.press('d');
    await clickField(page, x, y);
    await typeIn(page, 'x', x);
    await typeIn(page, 'y', y);
}

test.describe('RMP1-H in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('the thirteen structures: quiet outlines under the route, named on hover, never a handle -- each ON its element in the picture', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        const drawn = await page.evaluate(() => [...document.querySelectorAll('.g-obstacle')].map((e) => ({
            id: e.dataset.obstacle, title: e.querySelector('title').textContent, handle: !!e.closest('[data-handle]'),
            first: e.parentElement === e.parentElement.parentElement.firstElementChild,
            pts: e.getAttribute('points').trim().split(/\s+/).map((q) => q.split(',').map(Number)) })));
        assert.equal(drawn.length, 13);
        assert.ok(drawn.every((d) => / — solid$/.test(d.title) && !d.handle && d.first), JSON.stringify(drawn.map((d) => d.title)));
        assert.ok(drawn.some((d) => d.title === 'red goal by the right wall — solid'));
        // ON the picture: the pixels inside each outline are the element's, a ring 0.6-1.6 in outside it the floor's. A goal
        // (a top view) fills its outline; a Loader's 23 in top is drawn about 3 in outward (the picture's camera is above
        // the centre), so inside its floor outline only its clear base shows -- its ring, and its base, are checked
        const score = await page.evaluate(async (list) => {
            const pic = JSON.parse(document.getElementById('shulib-field').textContent);
            const img = new Image();
            img.src = pic.url;
            await img.decode();
            const c = document.createElement('canvas');
            c.width = pic.width;
            c.height = pic.height;
            const g = c.getContext('2d');
            g.drawImage(img, 0, 0);
            const px = g.getImageData(0, 0, pic.width, pic.height).data;
            const k = pic.pxPerIn;
            const wall = (70.2047 - 0.3) * k;
            const floor = (i) => {
                const mx = Math.max(px[i], px[i + 1], px[i + 2]);
                const mn = Math.min(px[i], px[i + 1], px[i + 2]);
                return mx - mn < 28 && (mx + mn) / 2 >= 60 && (mx + mn) / 2 <= 215;
            };
            const seg = (x, y, a, b) => {
                const vx = b[0] - a[0];
                const vy = b[1] - a[1];
                const t = Math.max(0, Math.min(1, ((x - a[0]) * vx + (y - a[1]) * vy) / (vx * vx + vy * vy)));
                return Math.hypot(x - a[0] - t * vx, y - a[1] - t * vy);
            };
            return list.map((d) => {
                const xs = d.pts.map((q) => q[0]);
                const ys = d.pts.map((q) => q[1]);
                const r = { id: d.id, inside: [0, 0], ring: [0, 0] };
                for (let y = Math.floor(Math.min(...ys) - 2 * k); y <= Math.max(...ys) + 2 * k; ++y) {
                    for (let x = Math.floor(Math.min(...xs) - 2 * k); x <= Math.max(...xs) + 2 * k; ++x) {
                        if (x < 0 || y < 0 || x >= pic.width || y >= pic.height
                            || Math.abs(x - pic.originPx[0]) > wall || Math.abs(y - pic.originPx[1]) > wall) {
                            continue;
                        }
                        let inside = false;
                        let dist = Infinity;
                        d.pts.forEach((a, i) => {
                            const b = d.pts[(i + 1) % d.pts.length];
                            if ((a[1] > y) !== (b[1] > y) && x < a[0] + ((y - a[1]) * (b[0] - a[0])) / (b[1] - a[1])) {
                                inside = !inside;
                            }
                            dist = Math.min(dist, seg(x, y, a, b));
                        });
                        const where = inside && dist >= 0.4 * k ? r.inside : !inside && dist >= 0.6 * k && dist <= 1.6 * k ? r.ring : null;
                        if (where) {
                            where[0] += floor((y * pic.width + x) * 4) ? 0 : 1;
                            where[1] += 1;
                        }
                    }
                }
                return { id: r.id, inside: r.inside[0] / r.inside[1], ring: r.ring[0] / r.ring[1] };
            });
        }, drawn);
        for (const s of score) {
            const loader = s.id.startsWith('loader');
            assert.ok(s.inside >= (loader ? 0.15 : 0.85) && s.ring <= (loader ? 0.15 : 0.10),
                      `${s.id}: ${(s.inside * 100).toFixed(0)} % of its inside and ${(s.ring * 100).toFixed(0)} % of the ring around it is NOT floor`);
        }
        // nothing highlighted until a step goes into one; a start dropped ON the centre goal is red, the goal the one highlight
        assert.equal(await page.locator('.g-obstacle.hit-error, .g-obstacle.hit-warn').count(), 0);
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, 0, 0);
        assert.deepEqual((await structureIssues(page)).map((x) => [x.code, x.stepIndex]), [['RCO1', -1]]);
        assert.deepEqual(await page.evaluate(() => [...document.querySelectorAll('.g-obstacle.hit-error, .g-obstacle.hit-warn')].map((x) => x.dataset.obstacle)),
                         ['centre-goal']);
        assert.deepEqual(await p.done(), []);
    });

    for (const v of VARIANTS) {
        test(`[${v.tag}] a drive THROUGH the centre goal: red, which one, how far, where -- the one highlight -- and the way around in one click`, async () => {
            const p = await openPage(browser, site.base, v);
            const { page } = p;
            await startAt(page, -30, 0, 0);
            await driveTo(page, 30, 0);
            const [e] = await structureIssues(page);
            assert.equal(e.code, 'RCO1');
            assert.match(e.words, /goes 10\.4 in into the centre goal: it is solid/);
            const box = page.locator('.pane.right .issue.error');
            assert.match(await box.textContent(), /the centre goal: it is solid, the robot cannot drive through it/);
            assert.deepEqual(await highlighted(page), ['centre-goal'], 'ONE highlight');
            assert.equal(await switches(page).count(), 0, 'no "on purpose" on a red no switch clears');
            // the robot at THAT moment: its square where the check found it deepest
            const moment = page.locator('.g-obstacle-moment.error');
            assert.equal(await moment.count(), 1);
            const pic = await page.evaluate(() => JSON.parse(document.getElementById('shulib-field').textContent));
            const want = squareCorners(e.obstacle.at.x, e.obstacle.at.y, e.obstacle.at.headingDeg, 15)
                .map((c) => [pic.originPx[0] + c.x * pic.pxPerIn, pic.originPx[1] - c.y * pic.pxPerIn]);
            const got = (await moment.getAttribute('points')).trim().split(/\s+/).map((q) => q.split(',').map(Number));
            assert.ok(Math.abs(e.obstacle.at.x) < 0.3 && got.every((q, k) => Math.hypot(q[0] - want[k][0], q[1] - want[k][1]) < 0.01), JSON.stringify({ got, want }));
            await shot(page, `obstacle-red-${v.tag}`, true);  // at rest: the toasts gone
            await box.getByRole('button', { name: /^Go around: add a point at \(/ }).click();
            const r = await route(page);
            assert.equal(r.steps.length, 2);
            assert.deepEqual(await structureIssues(page), []);
            assert.deepEqual((await page.evaluate(() => window.routeBuilder.issues())).filter((x) => x.level === 'error'), []);
            assert.equal(await page.locator('.g-obstacle.hit-error').count(), 0);
            assert.deepEqual(await p.done(), []);
        });

        test(`[${v.tag}] ending against a goal is a touch (amber) -- "touches it on purpose" silences it; Ctrl+Z brings it back`, async () => {
            const p = await openPage(browser, site.base, v);
            const { page } = p;
            await startAt(page, 0, -30, 90);
            await driveTo(page, 0, -10.005);  // the square's front 0.3 in into the centre goal's flat (7.5 + 2.805)
            const [w] = await structureIssues(page);
            assert.equal(w.code, 'RCO2');
            assert.equal(w.level, 'warn');
            assert.match(await page.locator('.pane.right .issue.warn').textContent(), /touches the centre goal here \(0\.3 in into it\)/);
            assert.equal(await page.locator('.g-obstacle.hit-warn[data-obstacle="centre-goal"]').count(), 1);
            const sw = page.locator('.pane.right input[name="touches-centre-goal"]');
            assert.equal(await switches(page).count(), 1, 'ONE switch: the structure it touches');
            assert.match(await page.locator('.pane.right .touch-purpose').textContent(), /Touches the centre goal on purpose/);
            await shot(page, `obstacle-touch-${v.tag}`, true);
            await sw.check();
            assert.deepEqual(await structureIssues(page), []);
            assert.deepEqual((await route(page)).steps[0].touches, ['centre-goal']);
            assert.ok(await page.locator('.pane.right input[name="touches-centre-goal"]').isChecked(), 'still there to untick');
            await page.locator('.pane.right input[name="touches-centre-goal"]').uncheck();
            assert.deepEqual((await structureIssues(page)).map((x) => x.code), ['RCO2'], 'unticked: the amber is back');
            assert.equal((await route(page)).steps[0].touches, undefined);
            await page.locator('.pane.right input[name="touches-centre-goal"]').check();
            await page.locator('.field-svg').focus();
            await page.keyboard.press('Control+z');
            assert.deepEqual((await structureIssues(page)).map((x) => x.code), ['RCO2']);
            // another step selected: its own (none) is the highlight, never step 1's
            await page.locator('.field-svg').focus();
            await page.keyboard.press('p');
            await typeIn(page, 'seconds', 0.5);
            assert.deepEqual(await highlighted(page), [], 'the wait selected: nothing lit');
            assert.equal(await switches(page).count(), 0, 'a clean step: no switch');
            await page.locator('.step[data-step="0"]').click();
            assert.deepEqual(await highlighted(page), ['centre-goal'], 'the drive selected: its structure lit');
            assert.deepEqual(await p.done(), []);
        });
    }

    test('PREVIEW: a run whose square goes into a goal is THROUGH it -- its step red saying which, the rest greyed, the path faded after', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        // a recorded run of a clean route (two_starts: a drive from (-36, -60) to (-36, -36), 2.15 in left of the black goal
        // by the red wall), its drive's paces bent up to 10 in right: the real follower swinging into the goal
        const f = fixture('two_starts');
        const one = f.result.one;
        const drive = one.pageSteps[2];
        const samples = one.samples.map((q, i) => [q[0], q[1] + (10 * Math.min(i, drive.last)) / drive.last, q[2], q[3]]);
        await page.evaluate((text) => window.routeBuilder.useEngine({ text }), standinScript(['two_starts'], { patch: { two_starts: { samples } } }));
        await page.getByRole('button', { name: 'Got it' }).click();
        const [fc] = await Promise.all([page.waitForEvent('filechooser'), page.getByRole('button', { name: 'Open a project' }).click()]);
        await fc.setFiles({ name: 'route_maker.json', mimeType: 'application/json', buffer: Buffer.from(f.text, 'utf8') });
        await page.waitForFunction((want) => JSON.stringify(window.routeBuilder.route()) === want, JSON.stringify(f.route));
        assert.deepEqual(await structureIssues(page), [], 'the PLAN is clear of it');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+Enter');
        await page.waitForFunction(() => window.routeBuilder.preview().phase === 'open');
        await page.evaluate(() => window.routeBuilder.pause());
        const card = page.locator('.step[data-step="2"]');
        assert.match(await card.getAttribute('class'), /pv-failed/);
        assert.match(await card.getAttribute('aria-label'), /In the preview: Went THROUGH the black goal by the red wall$/);
        assert.match(await card.locator('.step-lib').textContent(), /^On the robot it hits the black goal by the red wall at \d+\.\d\d s, in step 3 \(\d+\.\d\d in into it\): the host test plant drives through it\./);
        assert.match(await page.locator('.step[data-step="3"]').getAttribute('class'), /pv-skipped/);
        assert.equal(await page.locator('.dock .chip.error').textContent(), 'FINISHED, but THROUGH the black goal by the red wall');
        await page.keyboard.press('Home');
        assert.equal(await page.locator('.g-run.one.through').count(), 0);
        await page.keyboard.press('End');
        assert.equal(await page.locator('.g-run.one.through').count(), 1, 'faded after the THROUGH');
        await shot(page, 'obstacle-preview-light-1440');
        assert.deepEqual(await p.done(), []);
    });
});
