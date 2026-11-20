// step_view.test.mjs — RMP1-G: a step's `view` (hidden, locked, its section) is saved and opened exactly, an older
// project opens unchanged, a newer view is refused by name, and NOTHING of it reaches the C++ or the preview's lines:
// every golden exports byte for byte the committed .cpp with every step hidden, locked and in a folded section.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { normalizeView, setView, sectionsOf, indicesOf, focusSet, isHidden, isLocked } from '../src/model/step_view.mjs';
import { loadProject, saveProject } from '../src/model/project.mjs';
import { newRoute, setStart, addStep, duplicateStep, moveStep, removeStep, normalizeRoute } from '../src/model/route.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { routeLines } from '../src/model/route_lines.mjs';
import { goldens, goldenRobot, GOLDEN_DIR } from '../goldens.mjs';

const START = { x: -36, y: -55.7, headingDeg: 0 };
const route5 = () => [[-12, -55.7], [-12, -36], [0, -36], [12, -36], [12, -12]]
    .reduce((r, [x, y]) => addStep(r, { do: 'driveTo', x, y, faceFirst: true }), setStart(newRoute(), START));
const project = (r) => ({ schemaVersion: 1, madeWith: 'test', routes: [r], robots: {}, exports: {} });
/** Every step hidden and locked; the first a folded section, the third an open one. */
const allViewed = (r) => {
    let v = setView(r, r.steps.map((_, i) => i), { hidden: true, locked: true });
    v = setView(v, [0], { section: 'Start', folded: true });
    return r.steps.length > 2 ? setView(v, [2], { section: 'Claw 1' }) : v;
};

test('normalizeView keeps only what is set, in one order; folded only on a section; a blank name is no section', () => {
    assert.equal(normalizeView(undefined), undefined);
    assert.equal(normalizeView({ hidden: false, locked: false, folded: true }), undefined);
    assert.equal(normalizeView({ section: '   ', folded: true }), undefined);
    assert.equal(normalizeView({ hidden: 'yes', locked: 1 }), undefined, 'only true is set');
    assert.deepEqual(Object.keys(normalizeView({ folded: true, section: ' Claw 1 ', locked: true, hidden: true })), ['hidden', 'locked', 'section', 'folded']);
    assert.equal(normalizeView({ section: ' Claw 1 ' }).section, 'Claw 1');
});

test('setView merges into each step named and drops an emptied view; the route is new, the old one untouched', () => {
    const r = route5();
    const a = setView(r, [1, 3], { hidden: true });
    assert.deepEqual(a.steps.map(isHidden), [false, true, false, true, false]);
    assert.equal(r.steps[1].view, undefined, 'pure');
    const b = setView(a, [1], { locked: true });
    assert.deepEqual(b.steps[1].view, { hidden: true, locked: true });
    const c = setView(b, [1], { hidden: false, locked: false });
    assert.ok(!('view' in c.steps[1]), 'nothing set: no view key at all');
    assert.deepEqual(c.steps[3].view, { hidden: true });
});

test('sections run from their head to the step before the next head; steps before the first are in none', () => {
    let r = setView(route5(), [1], { section: 'Claw 1' });
    r = setView(r, [3], { section: 'Back', folded: true });
    assert.deepEqual(sectionsOf(r), [{ name: 'Claw 1', head: 1, from: 1, to: 2, folded: false }, { name: 'Back', head: 3, from: 3, to: 4, folded: true }]);
    assert.deepEqual(indicesOf(sectionsOf(r)[1]), [3, 4]);
    assert.deepEqual(sectionsOf(route5()), []);
});

test('a section stays where it was when its first step is deleted or moved away (its steps never drop into a folded one)', () => {
    let r = setView(route5(), [1], { section: 'Claw 1', folded: true });
    r = setView(r, [2], { hidden: true });
    r = setView(r, [3], { section: 'Back', locked: true });  // Claw 1 = steps 1-2 (folded), Back = 3-4
    const span = (x) => sectionsOf(x).map((c) => [c.name, c.from, c.to, c.folded]);
    const d = removeStep(r, 3);
    assert.deepEqual(span(d), [['Claw 1', 1, 2, true], ['Back', 3, 3, false]], 'step 4 still in Back, not in the folded Claw 1');
    assert.deepEqual(d.steps[3].view, { section: 'Back' }, 'the name only: the lock was the deleted step\'s');
    const d1 = removeStep(r, 1);
    assert.deepEqual(span(d1), [['Claw 1', 1, 1, true], ['Back', 2, 3, false]], 'the fold goes with the name');
    assert.deepEqual(d1.steps[1].view, { hidden: true, section: 'Claw 1', folded: true }, 'its own hidden kept');
    const m = moveStep(r, 3, 0);  // Back's first step to the top: Back keeps step 4, the moved step is in no section
    assert.deepEqual(span(m), [['Claw 1', 2, 3, true], ['Back', 4, 4, false]]);
    assert.deepEqual(m.steps[0].view, { locked: true });
    const down = moveStep(r, 1, 2);  // Claw 1's first step one down: still in Claw 1, now second; the fold opens
    assert.deepEqual(span(down), [['Claw 1', 1, 2, false], ['Back', 3, 4, false]]);
    assert.deepEqual([down.steps[1].x, down.steps[2].x], [r.steps[2].x, r.steps[1].x]);
    // a one-step section: deleted, it goes; moved, it goes with its step; a move to where it is changes nothing
    const one = setView(route5(), [2], { section: 'Turn' });
    assert.deepEqual(span(removeStep(setView(one, [3], { section: 'B' }), 2)), [['B', 2, 3, false]]);
    assert.deepEqual(span(moveStep(setView(one, [3], { section: 'B' }), 2, 0)), [['Turn', 0, 2, false], ['B', 3, 4, false]]);
    assert.equal(moveStep(one, 2, 2), one);
    assert.equal(moveStep(one, 4, 9), one, 'past the end is where it is');
});

test('no move puts a step away unseen: a folded section a step lands in unfolds; every step listed before is listed after', () => {
    const pts = [[-12, -55.7], [-12, -36], [0, -36], [12, -36], [12, -12], [24, -12], [24, 0], [36, 0]];
    const base = pts.reduce((r, [x, y]) => addStep(r, { do: 'driveTo', x, y, faceFirst: true }), setStart(newRoute(), START));
    const heads = [[1, 'A'], [3, 'B'], [6, 'C'], [7, 'D']];  // none: step 1; A: 2-3; B: 4-6; C, D: one step each
    const id = (s) => `${s.x},${s.y}`;
    const listed = (r) => new Set(r.steps.filter((_, i) => !sectionsOf(r).some((c) => c.folded && c.from <= i && i <= c.to)).map(id));
    let n = 0;
    for (let mask = 0; mask < 16; ++mask) {
        const r = heads.reduce((x, [i, name], k) => setView(x, [i], { section: name, folded: !!(mask & (1 << k)) }), base);
        for (let from = 0; from < pts.length; ++from) {
            for (let to = 0; to < pts.length; ++to) {
                const m = moveStep(r, from, to);
                const after = listed(m);
                const what = `mask ${mask}, ${from} -> ${to}`;
                assert.ok(from === to ? m === r : after.has(id(r.steps[from])), `${what}: the moved step is listed (no move: the route as it was)`);
                [...listed(r)].forEach((k) => assert.ok(after.has(k), `${what}: ${k} still listed`));
                ++n;
            }
        }
    }
    assert.equal(n, 1024);
    // the verifier's keys: Claw 1 (steps 2-3) folded, Back's first step (4) Alt+Down then Alt+Up: same order, step 4 listed
    let r = setView(setView(base, [1], { section: 'Claw 1', folded: true }), [3], { section: 'Back' });
    const back = moveStep(moveStep(r, 3, 4), 4, 3);
    assert.deepEqual(back.steps.map(id), r.steps.map(id));
    assert.ok(listed(back).has(id(r.steps[3])), 'step 4 is still in the list');
    assert.deepEqual(sectionsOf(back).map((c) => [c.name, c.from, c.to, c.folded]), [['Claw 1', 1, 3, false], ['Back', 4, 7, false]],
                     'it joined Claw 1, which opened (Ctrl+Z restores it exactly)');
    r = setView(r, [3], { folded: true });
    assert.deepEqual(sectionsOf(moveStep(r, 1, 0)).map((c) => c.folded), [true, true], 'Claw 1\'s first step to the top: in no section, none opens');
});

test('focus: the step and the one before and after; the start is step 1\'s neighbour; nothing past the ends', () => {
    const r = route5();
    assert.deepEqual([...focusSet(r, 2)].sort(), [1, 2, 3]);
    assert.deepEqual([...focusSet(r, 0)].map(String).sort(), ['0', '1', 'start']);
    assert.deepEqual([...focusSet(r, 'start')].map(String).sort(), ['0', 'start']);
    assert.deepEqual([...focusSet(r, 4)].sort(), [3, 4]);
    assert.deepEqual([...focusSet(r, 9)], [], 'a step gone (deleted): nothing in focus');
    assert.deepEqual([...focusSet(r, null)], []);
});

test('a duplicate shows and is unlocked, and starts no section; normalize puts view LAST', () => {
    const r = setView(route5(), [1], { hidden: true, locked: true, section: 'Claw 1' });
    const d = duplicateStep(r, 1);
    assert.ok(!('view' in d.steps[2]));
    assert.deepEqual(d.steps[1].view, { hidden: true, locked: true, section: 'Claw 1' });
    const n = normalizeRoute({ ...r, steps: [{ view: { locked: true }, care: { timeoutS: 2 }, do: 'driveTo', y: 1, x: 2, label: 'a' }] });
    assert.deepEqual(Object.keys(n.steps[0]), ['label', 'do', 'x', 'y', 'care', 'view']);
});

test('the project round-trips the view exactly (save -> open -> save is the same bytes); an old project opens unchanged', () => {
    const p = project(normalizeRoute(allViewed(route5())));
    const text = saveProject(p);
    assert.match(text, /"view": \{\n\s+"hidden": true,\n\s+"locked": true,\n\s+"section": "Start",\n\s+"folded": true\n/);
    const l = loadProject(text);
    assert.ok(l.ok, l.words);
    assert.equal(saveProject(l.project), text);
    assert.deepEqual(l.project.routes[0].steps.map((s) => s.view), p.routes[0].steps.map((s) => s.view));
    // every committed golden is a project from before RMP1-G: it opens, and saves, as it was
    for (const f of fs.readdirSync(GOLDEN_DIR).filter((n) => n.endsWith('.route.json'))) {
        const t = fs.readFileSync(path.join(GOLDEN_DIR, f), 'utf8');
        const g = loadProject(t);
        assert.ok(g.ok, `${f}: ${g.words}`);
        assert.equal(saveProject(g.project), t, f);
        assert.ok(!t.includes('"view"'), f);
    }
});

test('a view this builder cannot write is refused by name, nothing opened (a newer builder\'s)', () => {
    const bad = [{ hidden: true, colour: 'red' }, 'hidden', [true], { hidden: 'yes' }, { locked: 1 }, { folded: null, section: 'a' }, { section: 3 },
                 null, 5, true];
    for (const view of bad) {
        const r = { ...route5(), steps: route5().steps.map((s, i) => (i === 1 ? { ...s, view } : s)) };
        const l = loadProject(JSON.stringify(project(r)));
        assert.equal(l.ok, false, JSON.stringify(view));
        assert.match(l.words, /a "view" of .* \(route "skills", step 2\).*NEWER/, JSON.stringify(view));
    }
    const ok = loadProject(JSON.stringify(project(setView(route5(), [1], { hidden: true, locked: true, section: 'x', folded: true }))));
    assert.ok(ok.ok, ok.words);
});

test('D6: the export and the preview\'s lines ignore the view -- every golden byte for byte the committed .cpp, hidden, locked and in sections', () => {
    let n = 0;
    for (const [name, text] of goldens()) {
        const cpp = fs.readFileSync(path.join(GOLDEN_DIR, `${name}.cpp`), 'utf8');
        const l = loadProject(text);
        const viewed = loadProject(saveProject({ ...l.project, routes: l.project.routes.map(allViewed) }));
        assert.ok(viewed.ok, viewed.words);
        assert.ok(viewed.project.routes.some((r) => r.steps.some((s) => s.view)), `${name}: the views were kept`);
        const robot = goldenRobot(l.project.routes[0].robot);
        assert.equal(`${exportRegion(viewed.project.routes, robot)}\n`, cpp, name);
        if (robot) {
            for (const [k, r] of l.project.routes.entries()) {
                const lines = (x) => { try { return routeLines(x, robot); } catch (e) { return `throws: ${e.message}`; } };
                assert.equal(lines(viewed.project.routes[k]), lines(r), `${name} route ${k}`);
            }
        }
        ++n;
    }
    assert.ok(n >= 20, `only ${n} goldens`);
});
