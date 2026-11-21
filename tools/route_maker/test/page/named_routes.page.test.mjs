// named_routes.page.test.mjs — RMP1-C in a browser: the route switcher (D1), robot two on the field and "both robots
// at the same time?" (D3–D5), the cascade's plain steps and the claw's buttons (D7, D8), small moves and the nudge
// keys (D9). Every number typed is the test's. ROUTE_MAKER_SHOTS: the screens, light and dark, 1440 and 1024.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { SITE, skip, VARIANTS, launch, serve, openPage, shot } from './harness.mjs';
import { squareCorners } from '../../src/model/geometry.mjs';

let browser;
let site;
const route = (page) => page.evaluate(() => window.routeBuilder.route());
const routes = (page) => page.evaluate(() => window.routeBuilder.routes());
const errorsOf = async (page) => (await page.evaluate(() => window.routeBuilder.issues())).filter((x) => x.level === 'error');

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
/** The start placed and typed exactly (the field's snap is the student's drag; typed numbers are exact). */
async function startAt(page, x, y, h) {
    await clickField(page, x, y);
    await typeIn(page, 'startX', x);
    await typeIn(page, 'startY', y);
    await typeIn(page, 'startH', h);
}
const menu = (page) => page.locator('.popover[aria-label="Routes"]');
async function openRoutes(page) {
    await page.locator('#route-switch').click();
    await menu(page).waitFor();
}
/** The page's own name box: type, then OK (or read the refusal under it). */
async function nameBox(page, text) {
    await page.locator('.dialog input.txt').fill(text);
    return (await page.locator('.dialog .inline-err').textContent()) || '';
}

test.describe('RMP1-C in a browser', { skip }, () => {
    test.before(async () => {
        assert.ok(fs.existsSync(path.join(SITE, 'app.html')), `build it first: python3 tools/route_maker/build.py --out ${SITE} --describe ...`);
        browser = await launch();
        site = await serve();
    });
    test.after(async () => {
        await browser?.close();
        site?.server.close();
    });

    test('D1: the switcher -- add (a taken or refused name said under the box, OK held), rename, duplicate, delete confirmed in the page, undo; every route exported, an error on ANY one stops Export', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        assert.equal(await page.locator('#route-switch strong').textContent(), 'skills');
        await openRoutes(page);
        assert.ok(await menu(page).getByRole('button', { name: 'Rename "skills"' }).isDisabled(), 'the skills route keeps its name');
        assert.ok(await menu(page).getByRole('button', { name: 'Delete "skills"' }).isDisabled(), 'and stays');
        await menu(page).getByRole('button', { name: 'New route' }).click();
        assert.equal(await page.locator('.dialog input.txt').inputValue(), 'route 2', 'a free name offered');
        assert.match(await nameBox(page, 'Skills'), /"skills" is taken/);
        assert.ok(await page.getByRole('button', { name: 'Add the route' }).isDisabled());
        assert.match(await nameBox(page, 'NONE'), /NONE is the brain menu/);
        assert.match(await nameBox(page, 'a"b'), /cannot contain "/);
        assert.match(await nameBox(page, '40IN FORWARD'), /"40in forward" is another auton on the brain's menu/, 'the other autons\' names, capitals aside');
        assert.match(await nameBox(page, 'TEST 24in turn back'), /"TEST 24in turn back" is another auton/);
        assert.equal(await nameBox(page, 'left side'), '');
        await page.getByRole('button', { name: 'Add the route' }).click();
        assert.deepEqual((await routes(page)).map((r) => r.name), ['skills', 'left side']);
        assert.equal(await page.locator('#route-switch strong').textContent(), 'left side', 'the new route on the field');
        assert.equal(await page.locator('#route-switch .count').textContent(), '2/2');
        assert.equal((await route(page)).mirroredFor, undefined, 'a new route runs as written on robot two');
        await clickField(page, 24, -60);  // RMP1-H: clear of the red goal by the red wall (at -48 its start is in it)
        assert.equal(await page.locator('[data-twin]').count(), 0, 'not mirrored: no robot two drawn');
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        // rename in the menu
        await openRoutes(page);
        await menu(page).getByRole('button', { name: 'Rename "left side"' }).click();
        await nameBox(page, 'left 1');
        await page.getByRole('button', { name: 'Rename', exact: true }).click();
        assert.equal((await route(page)).name, 'left 1');
        assert.equal((await route(page)).steps.length, 1, 'a rename keeps the steps');
        // duplicate, then delete it: Keep it does nothing, Delete does, Ctrl+Z brings it back
        await openRoutes(page);
        await menu(page).getByRole('button', { name: 'Duplicate "left 1"' }).click();
        assert.equal(await page.locator('.dialog input.txt').inputValue(), 'left 1 copy');
        await page.getByRole('button', { name: 'Duplicate', exact: true }).click();
        assert.deepEqual((await routes(page)).map((r) => [r.name, r.steps.length]), [['skills', 0], ['left 1', 1], ['left 1 copy', 1]]);
        await openRoutes(page);
        await shot(page, 'routes-menu-light-1440');
        await menu(page).getByRole('button', { name: 'Delete "left 1 copy"' }).click();
        assert.match(await page.locator('.dialog').textContent(), /Delete "left 1 copy"\?Its 1 step goes with it\. Undo \(Ctrl\+Z\) brings it back\./);
        await page.getByRole('button', { name: 'Keep it' }).click();
        assert.equal((await routes(page)).length, 3);
        await openRoutes(page);
        await menu(page).getByRole('button', { name: 'Delete "left 1 copy"' }).click();
        await page.getByRole('button', { name: 'Delete the route' }).click();
        assert.deepEqual((await routes(page)).map((r) => r.name), ['skills', 'left 1']);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        assert.deepEqual((await routes(page)).map((r) => r.name), ['skills', 'left 1', 'left 1 copy'], 'undo brings it back');
        // switch to "skills": no start there yet -- an error on a route NOT on the field still stops Export, by name
        await openRoutes(page);
        await menu(page).locator('[data-route="skills"] .route-pick').click();
        assert.equal((await route(page)).name, 'skills');
        await clickField(page, -36, -55.7);
        await page.locator('.pane.right [data-together="false"]').click();
        await page.keyboard.press('p');
        await typeIn(page, 'seconds', 1);
        await openRoutes(page);
        await menu(page).locator('[data-route="left 1"] .route-pick').click();
        await page.keyboard.press('p');  // a wait with no seconds, on "left 1"
        await openRoutes(page);
        await menu(page).locator('[data-route="skills"] .route-pick').click();
        assert.deepEqual(await errorsOf(page), [], 'the route on the field is clean');
        // an error on the route on the field too: each listed under its own route's name
        await page.keyboard.press('p');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+e');
        const listed = await page.locator('.drawer .banner.error li').allTextContents();
        assert.equal(listed.length, 2);
        assert.match(listed[0], /^"skills" Step 2: A wait needs a time/);
        assert.match(listed[1], /^"left 1" Step 2: A wait needs a time/);
        await page.keyboard.press('Escape');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        await page.keyboard.press('Control+e');
        assert.match(await page.locator('.drawer .banner.error').textContent(), /"left 1" Step 2: A wait needs a time/);
        await page.locator('.drawer').getByRole('button', { name: 'Go there' }).click();
        assert.equal((await route(page)).name, 'left 1', '"Go there" switches to the route');
        await typeIn(page, 'seconds', 2);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+e');
        const code = await page.locator('.drawer pre.code').textContent();
        assert.match(await page.locator('#export-title').textContent(), /Export the 3 routes/);
        for (const [name, id] of [['skills', 'skills'], ['left 1', 'left1'], ['left 1 copy', 'left1Copy']]) {
            assert.ok(code.includes(`e.name = "${name}";`), name);
            assert.ok(code.includes(`{&${id}Route, `), id);
        }
        assert.ok(code.includes('{&skillsRoute, Mirror::ForRobotTwo, Together::No},'));
        assert.ok(code.includes('{&left1Route, Mirror::None, Together::No},'));
        assert.deepEqual(await p.done(), []);
    });

    test('D3-D5: robot two on the field, the question on the route, the team lead\'s real start TOO CLOSE together -- its fix, and "one at a time"', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await startAt(page, -36, 54.076, -90);
        assert.equal(await page.locator('[data-twin]').count(), 1, 'robot two drawn');
        assert.equal(await page.locator('.g-twin-label').textContent(), 'Black Bot (mirrored)');
        assert.equal(await page.locator('[data-mirror]').count(), 1, 'the diagonal drawn');
        assert.match(await page.locator('.pane.right').textContent(), /Black Bot starts at \(-54\.076, 36\) facing 0 deg/);
        const q = (await errorsOf(page)).map((x) => x.code);
        assert.deepEqual(q, ['RCT1'], 'asked, no default');
        assert.ok(await page.locator('.ask-together.unanswered').isVisible());
        await page.locator('.pane.right [data-together="true"]').click();
        const close = await errorsOf(page);
        assert.deepEqual(close.map((x) => [x.code, x.stepIndex]), [['RCP1', -1]]);
        assert.match(close[0].words, /At the START White Bot at \(-36, 54\.076\) and Black Bot \(mirrored\) at \(-54\.076, 36\) are 4\.4 in apart/);
        assert.equal(await page.locator('.field-svg .g-too-close').count(), 2, 'both squares marked');
        await shot(page, 'too-close-light-1440');
        await page.locator('.pane.right .issue.error').getByRole('button', { name: /Move the start .* in away from the diagonal/ }).click();
        assert.deepEqual(await errorsOf(page), [], 'fixed in one click');
        assert.equal(await page.locator('.field-svg .g-too-close').count(), 0);
        const moved = (await route(page)).start;
        // straight off the diagonal y = -x (along its normal: x and y up by the same), the least that clears 6 in: ~0.83 in
        assert.ok(moved.x > -36 && Math.abs((moved.x + 36) - (moved.y - 54.076)) < 0.002 && Math.hypot(moved.x + 36, moved.y - 54.076) < 0.9,
                  `straight away from the diagonal: ${JSON.stringify(moved)}`);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Control+z');
        assert.equal((await errorsOf(page))[0].code, 'RCP1');
        await page.locator('.pane.right [data-together="false"]').click();
        assert.deepEqual(await errorsOf(page), [], 'one at a time: never too close');
        assert.ok(await page.locator('.pane.right input[name="mirrored"]').isDisabled(), 'robot two always runs the skills route mirrored');
        // another route: the mirror turned on asks; turned off, robot two is hidden
        await openRoutes(page);
        await menu(page).getByRole('button', { name: 'New route' }).click();
        await page.getByRole('button', { name: 'Add the route' }).click();
        await clickField(page, 24, -60);  // RMP1-H: (24, -48) is in the red goal by the red wall
        await page.locator('.pane.right input[name="mirrored"]').click();
        assert.deepEqual((await errorsOf(page)).map((x) => x.code), ['RCT1']);
        assert.equal(await page.locator('[data-twin]').count(), 1);
        await page.locator('.pane.right input[name="mirrored"]').click();
        assert.equal((await route(page)).mirroredFor, undefined);
        assert.equal(await page.locator('[data-twin]').count(), 0, 'not mirrored: hidden');
        assert.equal(await page.locator('.ask-together').count(), 0);
        assert.deepEqual(await p.done(), []);
    });

    test('D4: robot two drawn AT the mirror -- its square, its front and its path, at a heading the mirror changes (30 deg -> -120 deg)', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await startAt(page, -36, 54.076, 30);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -12, 36);
        await typeIn(page, 'x', -12);
        await typeIn(page, 'y', 36);
        const pic = await page.evaluate(() => JSON.parse(document.getElementById('shulib-field').textContent));
        const px = (x, y) => [pic.originPx[0] + x * pic.pxPerIn, pic.originPx[1] - y * pic.pxPerIn];
        const pts = (sel) => page.evaluate((q) => document.querySelector(q).getAttribute('points').trim().split(/\s+/).map((v) => v.split(',').map(Number)), sel);
        const near = (a, b, what) => {
            assert.equal(a.length, b.length, what);
            a.forEach((q, i) => assert.ok(Math.hypot(q[0] - b[i][0], q[1] - b[i][1]) < 0.01, `${what} ${i}: ${q} vs ${b[i]}`));
        };
        // across y = -x: (x, y) -> (-y, -x), a heading h -> -90 - h (worked here by hand, not by the page's reflect)
        near(await pts('.g-twin-robot'), squareCorners(-54.076, 36, -120, 15).map((c) => px(c.x, c.y)), 'robot two\'s square');
        const rad = (-120 * Math.PI) / 180;
        near((await pts('.g-twin-front')).slice(0, 1), [px(-54.076 + Math.cos(rad) * 6.5, 36 + Math.sin(rad) * 6.5)], 'its front\'s tip');
        const path = await pts('.g-twin-path');
        near([path[0], path[path.length - 1]], [px(-54.076, 36), px(-36, 12)], 'its path, start to end');
        await page.locator('.start-card').click();
        assert.match(await page.locator('.pane.right').textContent(), /Black Bot starts at \(-54\.076, 36\) facing -120 deg/);
        assert.deepEqual(await p.done(), []);
    });

    test('D9: the arrows nudge the curve point picked ON THE FIELD -- never a point of another curve picked in the list; a dragged point is the picked one', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await startAt(page, -36, -55.7, 0);
        await page.locator('.pane.right [data-together="false"]').click();
        await page.locator('.field-svg').focus();
        for (const [a, b] of [[[0, -24], [24, -24]], [[30, -12], [40, -6]]]) {
            await page.keyboard.press('c');
            await clickField(page, ...a);
            await clickField(page, ...b);
            await page.keyboard.press('Enter');
        }
        await page.keyboard.press('Escape');
        const pointsOf = async () => (await route(page)).steps.map((x) => x.points.map((q) => [q.x, q.y]));
        const before = await pointsOf();
        await clickField(page, ...before[0][1]);  // curve 1's second point, picked on the field
        await page.locator('.field-svg').focus();
        await page.keyboard.press('ArrowRight');
        let now = await pointsOf();
        assert.deepEqual(now[0][1], [before[0][1][0] + 1, before[0][1][1]], 'the picked point moved 1 in');
        await page.locator('.step[data-step="1"]').click();  // curve 2, picked in the list: no point of it picked
        await page.locator('.field-svg').focus();
        await page.keyboard.press('ArrowRight');
        assert.deepEqual(await pointsOf(), now, 'nothing moved: not curve 1\'s point, not a point of curve 2 nobody picked');
        // drag curve 2's first point: it is the picked one now
        const at = await page.evaluate(([fx, fy]) => {
            const pic = JSON.parse(document.getElementById('shulib-field').textContent);
            const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(document.querySelector('.field-svg').getScreenCTM());
            return { x: pt.x, y: pt.y };
        }, now[1][0]);
        await page.mouse.move(at.x, at.y);
        await page.mouse.down();
        await page.mouse.move(at.x + 20, at.y, { steps: 5 });
        await page.mouse.up();
        const dragged = (await pointsOf())[1][0];
        assert.notDeepEqual(dragged, now[1][0], 'dragged');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('ArrowUp');
        now = await pointsOf();
        assert.deepEqual(now[1][0], [dragged[0], Math.round((dragged[1] + 1) * 1000) / 1000], 'the dragged point is the one nudged');
        assert.deepEqual(now[1][1], before[1][1]);
        // the verifier's cases: a point picked, then a NEW curve drawn, the picked curve deleted, an undo -- each time
        // the selected curve is another one, and none of its points was picked
        const arrowMovesNothing = async (why) => {
            const was = await pointsOf();
            await page.locator('.field-svg').focus();
            await page.keyboard.press('ArrowRight');
            assert.deepEqual(await pointsOf(), was, why);
            return was;
        };
        await clickField(page, ...now[0][1]);  // curve 1's point picked; the new curve goes in after curve 1
        await page.locator('.field-svg').focus();
        await page.keyboard.press('c');
        await clickField(page, -30, 10);
        await clickField(page, -20, 24);
        await page.keyboard.press('Enter');
        const three = await arrowMovesNothing('a new curve drawn: nothing moved');
        assert.deepEqual([three[0], three[2]], now);
        await clickField(page, ...three[1][1]);  // the new curve's point picked; Delete: curve 2 takes its place, step 2
        await page.keyboard.press('Delete');
        assert.deepEqual(await arrowMovesNothing('the picked curve deleted: nothing moved'), now);
        await clickField(page, ...now[1][0]);  // curve 2's point picked; the undo puts the new curve back at step 2
        await page.keyboard.press('Control+z');
        assert.deepEqual(await arrowMovesNothing('after an undo: nothing moved'), three);
        // an undo that shows another route selects nothing of it
        await openRoutes(page);
        await menu(page).getByRole('button', { name: 'New route' }).click();
        await page.getByRole('button', { name: 'Add the route' }).click();
        await clickField(page, 30, 30);  // route 2's start, selected
        await page.keyboard.press('Control+z');
        await page.keyboard.press('Control+z');
        assert.equal(await page.locator('#route-switch strong').textContent(), 'skills');
        assert.equal(await page.locator('.step.selected').count(), 0, 'nothing of the skills route selected');
        assert.deepEqual(await p.done(), []);
    });

    test('D9 at 1024 px: a click on a step\'s point opens the step drawer; a drag of it does not (the drawer would cover the field)', async () => {
        const p = await openPage(browser, site.base, { viewport: { width: 1024, height: 768 } });
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await clickField(page, -36, -55.7);
        await page.getByRole('button', { name: 'Close the step', exact: true }).click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -36, -24);
        await page.keyboard.press('Escape');
        const drawer = () => page.evaluate(() => document.querySelector('.app').dataset.right || 'closed');
        assert.equal(await drawer(), 'closed');
        const s = (await route(page)).steps[0];
        const at = await page.evaluate(([fx, fy]) => {
            const pic = JSON.parse(document.getElementById('shulib-field').textContent);
            const pt = new DOMPoint(pic.originPx[0] + fx * pic.pxPerIn, pic.originPx[1] - fy * pic.pxPerIn).matrixTransform(document.querySelector('.field-svg').getScreenCTM());
            return { x: pt.x, y: pt.y };
        }, [s.x, s.y]);
        await page.mouse.move(at.x, at.y);
        await page.mouse.down();
        await page.mouse.move(at.x + 30, at.y, { steps: 5 });
        await page.mouse.up();
        const moved = (await route(page)).steps[0];
        assert.ok(moved.x > s.x, 'dragged');
        assert.equal(await drawer(), 'closed', 'a drag opens no drawer over the field');
        await page.locator('.field-svg').focus();
        await page.keyboard.press('ArrowUp');
        assert.deepEqual([(await route(page)).steps[0].x, (await route(page)).steps[0].y], [moved.x, Math.round((moved.y + 1) * 1000) / 1000], 'the dragged step is the one nudged');
        await clickField(page, moved.x, moved.y + 1);
        assert.equal(await drawer(), 'open', 'a click opens it, as before');
        assert.deepEqual(await p.done(), []);
    });

    test('D7-D9: the cascade\'s plain steps tracked from its home, the claw\'s buttons, drive forward / back up, the nudge keys', async () => {
        const p = await openPage(browser, site.base);
        const { page } = p;
        await page.getByRole('button', { name: 'Got it' }).click();
        await startAt(page, -36, -55.7, 0);
        await page.locator('.pane.right [data-together="false"]').click();
        // RMP1-J: M -> the mechanism -> what it does; one height (D2); the home card (D6)
        const add = async (mech, words) => {
            await page.locator('.field-svg').focus();
            await page.keyboard.press('m');
            await page.locator(`.mech-menu [data-mech="${mech}"]`).click();
            await page.locator(`.mech-menu [data-act="${words}"]`).click();
        };
        await add('cascade', 'Move the cascade');
        assert.ok((await errorsOf(page)).some((x) => x.code === 'RC25'), 'a move with no home: home it first');
        await page.locator('.pane.right .home-card').getByRole('button', { name: 'Home it at the start' }).click();
        await typeIn(page, 'boundS', 3);
        await page.locator('.step[data-step="1"]').click();
        await typeIn(page, 'boundS', 3);
        await page.locator('.step[data-step="2"]').click();
        await typeIn(page, 'height', 6);
        await typeIn(page, 'boundS', 2);
        assert.equal(await page.locator('.pane.right [data-heights]').textContent(), '0 in → 6 in');
        await add('cascade', 'Move the cascade');
        await page.locator('.pane.right [data-level="bottom"]').click();
        await typeIn(page, 'boundS', 2);
        await add('claw', 'Close claw');
        let r = await route(page);
        assert.deepEqual(r.steps.map((s) => [s.do, s.to ?? s.bottom ?? s.position ?? null]),
                         [['startHome', null], ['waitUntilDone', null], ['move', 6], ['move', true], ['set', 'closed']]);
        assert.deepEqual(await errorsOf(page), []);
        assert.match(await page.locator('.step[data-step="4"]').textContent(), /Close claw/);
        // small moves: F adds "drive forward", typed; Backwards turns it into "back up"
        await page.locator('.field-svg').focus();
        await page.keyboard.press('f');
        await typeIn(page, 'inches', 1);
        r = await route(page);
        assert.deepEqual(r.steps[5], { do: 'forward', inches: 1 });
        assert.match(await page.locator('.pane.right').textContent(), /to \(-35, -55\.7\)/);
        // the nudge: the start selected, Right 1 in, Shift+Up 0.25 in; one undo each; arrows in a box are the box's
        await page.locator('.start-card').click();
        await page.locator('.field-svg').focus();
        await page.keyboard.press('ArrowRight');
        await page.keyboard.press('Shift+ArrowUp');
        assert.deepEqual((await route(page)).start, { x: -35, y: -55.45, headingDeg: 0 });
        await page.keyboard.press('Control+z');
        assert.deepEqual((await route(page)).start, { x: -35, y: -55.7, headingDeg: 0 });
        await page.locator('.pane.right input[name="startX"]').press('ArrowLeft');
        assert.equal((await route(page)).start.x, -35, 'typing: the box keeps its arrows');
        // a drive's point, 1 in from another: placed with the keys, exact
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -12, -48);
        await typeIn(page, 'x', -12);
        await typeIn(page, 'y', -48);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Escape');
        await page.keyboard.press('ArrowDown');
        assert.deepEqual([(await route(page)).steps[6].x, (await route(page)).steps[6].y], [-12, -49]);
        // a second point 1 in from it, placed by a click once zoomed in there (the wheel): two handles that do not touch
        // (each at most 8 px across the pointer's 1 in), the click a new point, not a grab of the first
        const near = await page.evaluate(() => {
            const pic = JSON.parse(document.getElementById('shulib-field').textContent);
            const pt = new DOMPoint(pic.originPx[0] - 12 * pic.pxPerIn, pic.originPx[1] + 48.5 * pic.pxPerIn)
                .matrixTransform(document.querySelector('.field-svg').getScreenCTM());
            return { x: pt.x, y: pt.y };
        });
        await page.mouse.move(near.x, near.y);
        for (let k = 0; k < 12; ++k) {
            await page.mouse.wheel(0, -100);
        }
        await page.locator('.field-svg').focus();
        await page.keyboard.press('d');
        await clickField(page, -12, -48);
        assert.equal((await route(page)).steps.length, 8, 'a new drive, not the first one\'s handle grabbed');
        await typeIn(page, 'x', -12);
        await typeIn(page, 'y', -48);
        await page.locator('.field-svg').focus();
        await page.keyboard.press('Escape');
        const boxes = await page.evaluate(() => [...document.querySelectorAll('.g-handle[data-handle="point"]')].slice(-2)
            .map((c) => c.getBoundingClientRect()).map((b) => ({ x: b.x + b.width / 2, y: b.y + b.height / 2, r: b.width / 2 })));
        assert.equal(boxes.length, 2);
        assert.ok(Math.hypot(boxes[0].x - boxes[1].x, boxes[0].y - boxes[1].y) > boxes[0].r + boxes[1].r + 2,
                  `two handles 1 in apart overlap: ${JSON.stringify(boxes)}`);
        assert.deepEqual(await p.done(), []);
    });

    for (const v of VARIANTS) {
        test(`RMP1-C screens [${v.tag}]: the routes menu, robot two too close, the mechanism menu`, async () => {
            const p = await openPage(browser, site.base, v);
            const { page } = p;
            await page.getByRole('button', { name: 'Got it' }).click();
            await startAt(page, -36, 54.076, -90);
            await page.locator('.pane.right [data-together="true"]').click();
            await page.locator('.field-svg').focus();
            await page.keyboard.press('d');
            await clickField(page, -12, 36);
            await shot(page, `rmp1c-too-close-${v.tag}`, true);
            await page.keyboard.press('m');
            await page.locator('.popover').waitFor();
            await shot(page, `rmp1c-mechanism-menu-${v.tag}`);
            await page.keyboard.press('Escape');  // RMP1-J: Esc on the list closes it
            await page.locator('#route-switch').click();
            await menu(page).waitFor();
            await shot(page, `rmp1c-routes-menu-${v.tag}`);
            await page.keyboard.press('Escape');
            assert.deepEqual(await p.done(), []);
        });
    }
});
