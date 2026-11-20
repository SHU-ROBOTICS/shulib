// project.test.mjs — RT1–RT5: a route saved and re-opened identically (D3; §2.2), and the undo history.
import test from 'node:test';
import assert from 'node:assert/strict';
import { loadProject, saveProject, newProject } from '../src/model/project.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { checkRoute } from '../src/model/checks.mjs';
import { addStep, updateStep, removeStep, moveStep, duplicateStep, setStart, renameRoute, newRoute } from '../src/model/route.mjs';
import { historyOf, commit, undo, redo, HISTORY_LIMIT } from '../src/model/undo.mjs';
import { goldens } from '../goldens.mjs';
import fs from 'node:fs';
import path from 'node:path';
import { fixtureRobot, seeded, goldenNames, GOLDENS } from './helpers.mjs';

test('RT1: every golden route.json -> load -> save is byte-identical', () => {
    // the FILE's own bytes, read from disk (never a re-saved copy: that would only test that saving is idempotent)
    const files = goldenNames().map((n) => [n, fs.readFileSync(path.join(GOLDENS, `${n}.route.json`), 'utf8')]);
    assert.ok(files.length >= 17);
    assert.ok(files.some(([, t]) => t.includes('"onFail"')), 'the goldens carry every field RT1 must keep (onFail among them)');
    for (const [name, text] of files) {
        const l = loadProject(text);
        assert.ok(l.ok, name);
        assert.equal(saveProject(l.project), text, name);
    }
    // a Windows checkout's CRLF and a byte-order mark load the same project
    const [, text] = goldens()[3];
    const l = loadProject(`﻿${text.replace(/\n/g, '\r\n')}`);
    assert.ok(l.ok);
    assert.equal(saveProject(l.project), text);
});

/** A random edit of every kind the page makes (seeded). */
function randomEdit(r, rnd) {
    const n = r.steps.length;
    const pick = Math.floor(rnd() * 9);
    const x = (rnd() - 0.5) * 100;
    const y = (rnd() - 0.5) * 100;
    switch (pick) {
        case 0: return setStart(r, { x, y, headingDeg: rnd() * 360 });
        case 1: return addStep(r, { do: 'driveTo', x, y, faceFirst: rnd() < 0.5, care: rnd() < 0.5 ? { timeoutS: rnd() * 4, onFail: 'carryOn' } : undefined });
        case 2: return addStep(r, { do: 'curveTo', points: [{ x, y }, { x: x + 10, y: y + 5, headingDeg: rnd() * 360 }] }, Math.floor(rnd() * (n + 1)));
        case 3: return addStep(r, { do: 'move', mechanism: 'lift', preset: 'low', boundS: rnd() * 3 });
        case 4: return n ? updateStep(r, Math.floor(rnd() * n), { label: `L${Math.floor(rnd() * 1000)}` }) : r;
        case 5: return n ? removeStep(r, Math.floor(rnd() * n)) : r;
        case 6: return n > 1 ? moveStep(r, Math.floor(rnd() * n), Math.floor(rnd() * n)) : r;
        case 7: return n ? duplicateStep(r, Math.floor(rnd() * n)) : r;
        default: return n ? updateStep(r, Math.floor(rnd() * n), { care: { onFail: { skipTo: 'L1' } } }) : renameRoute(r, `r${Math.floor(rnd() * 99)}`);
    }
}

test('RT2: load -> 60 edits -> undo every one -> deep-equal the original, and the export byte-identical', () => {
    const [, text] = goldens().find(([n]) => n === 'gl4_mechanisms');
    const original = loadProject(text).project.routes[0];
    let h = historyOf(original);
    const rnd = seeded(5);
    for (let i = 0; i < 60; ++i) {
        h = commit(h, randomEdit(h.present, rnd));
    }
    const last = h.present;
    const edits = h.past.length;
    assert.ok(edits >= 50, 'most random edits change the route');
    while (h.past.length) {
        h = undo(h);
    }
    assert.deepEqual(h.present, original);
    assert.equal(exportRegion(h.present), exportRegion(original));
    // and redo all the way gives the last state again
    let r2 = h;
    while (r2.future.length) {
        r2 = redo(r2);
    }
    assert.equal(r2.past.length, edits);
    assert.deepEqual(r2.present, last);
});

test('RT3: 200 seeded edits of every kind -> save -> load -> the same state', () => {
    const rnd = seeded(2026);
    let r = newRoute();
    for (let i = 0; i < 200; ++i) {
        r = randomEdit(r, rnd);
        const p = { ...newProject(), routes: [r] };
        const back = loadProject(saveProject(p));
        assert.ok(back.ok, back.words);
        assert.deepEqual(back.project.routes[0], r, `edit ${i}`);
    }
});

test('RT4: a file from a NEWER route builder is refused by name, nothing loaded', () => {
    const p = JSON.parse(saveProject(newProject()));
    p.schemaVersion = 2;
    const l = loadProject(JSON.stringify(p));
    assert.equal(l.ok, false);
    assert.match(l.words, /NEWER route builder \(schemaVersion 2/);
    assert.equal(l.project, undefined);
    for (const bad of ['{', '[]', '{"routes": []}', '{"schemaVersion": 1, "routes": []}', '{"schemaVersion": 1, "routes": [{"name": "s", "steps": [{}]}]}']) {
        assert.equal(loadProject(bad).ok, false, bad);
    }
    // a schemaVersion no builder wrote (0, -1, 1.5) is not opened either
    for (const v of [0, -1, 1.5]) {
        assert.equal(loadProject(JSON.stringify({ ...JSON.parse(saveProject(newProject())), schemaVersion: v })).ok, false, String(v));
    }
});

test('RT7 (D3, never half-read): a schemaVersion-1 file with a part this builder cannot write is refused BY NAME, nothing opened', () => {
    const base = JSON.parse(saveProject({ ...newProject(), routes: [addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }),
                                                                          { do: 'driveTo', x: -12, y: -55.7, faceFirst: false })] }));
    const variant = (edit) => {
        const p = JSON.parse(JSON.stringify(base));
        edit(p);
        return loadProject(JSON.stringify(p));
    };
    const cases = [
        // A1b's trigger, exactly as D3's sample writes it: opened here, the export would leave it out
        ['a step field "during"', (p) => { p.routes[0].steps[0].during = [{ name: 'raise', at: { percent: 60 }, then: { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 } }]; }],
        // A1b's wall reset: a step kind A1 has no code for
        ['a step kind "wallReset"', (p) => { p.routes[0].steps.push({ do: 'wallReset', wall: 'red', sensor: 'right', maxCorrectionIn: 3 }); }],
        ['a "care" field "maxAccel"', (p) => { p.routes[0].steps[0].care = { maxAccel: 3 }; }],
        ['a curve point field "speed"', (p) => { p.routes[0].steps.push({ do: 'curveTo', points: [{ x: 0, y: -40, speed: 3 }] }); }],
        ['a route field "who"', (p) => { p.routes[0].who = { 0: 'one' }; }],
        ['a "together" of "yes"', (p) => { p.routes[0].together = 'yes'; }],  // RMP1-C D3: true or false, or not answered
        ['a "bottom" of false', (p) => { p.routes[0].steps.push({ do: 'move', mechanism: 'lift', bottom: false, boundS: 2 }); }],
        ['a "match" route', (p) => { p.routes[0].kind = 'match'; }],
        ['an "onFail" of "retry"', (p) => { p.routes[0].steps[0].care = { onFail: 'retry' }; }],
        ['a "endActionAtS" that is not a number', (p) => { p.routes[0].endActionAtS = 'soon'; }],
    ];
    for (const [named, edit] of cases) {
        const l = variant(edit);
        assert.equal(l.ok, false, named);
        assert.equal(l.project, undefined, named);
        assert.ok(l.words.includes(named), `${named}: ${l.words}`);
        assert.match(l.words, /NEWER route builder.*Nothing was opened/);
    }
    // what A1 itself writes always opens (every golden, RT1) -- and an unknown TOP-level block (RMP1-B's robots.<id>) is kept
    assert.ok(variant((p) => { p.robots = { one: { controls: { whileHeld: [] } } }; }).ok);
    // RMP1-C D1: a project of NAMED routes opens, every route kept in order (A1 refused a second route)
    const two = variant((p) => { p.routes.push({ ...p.routes[0], name: 'practice', mirroredFor: undefined }); });
    assert.ok(two.ok);
    assert.deepEqual(two.project.routes.map((r) => [r.name, r.mirroredFor]), [['skills', 'two'], ['practice', undefined]]);
});

test('RT6: save -> open is EXACT for a step with a hole in it -- an emptied curve point, a typed-over care field, a start part', () => {
    let r = setStart({ ...newRoute(), together: true }, { x: -36, y: -55.7, headingDeg: 0 });
    r = addStep(r, { do: 'curveTo', points: [{ x: -24, y: -50 }, { x: -12, y: -24 }] });
    // the inspector empties point 2's x (an empty box is "nothing typed")
    r = updateStep(r, 0, { points: [r.steps[0].points[0], { ...r.steps[0].points[1], x: undefined }] });
    assert.deepEqual(r.steps[0].points[1], { y: -24 }, 'the hole is a MISSING x -- never NaN, which JSON writes as null and reads as 0');
    const holes = (route) => checkRoute(route, [fixtureRobot('x')], fixtureRobot('x').field).filter((i) => i.level === 'error').map((i) => i.words);
    assert.deepEqual(holes(r), ['Every curve point needs an x and a y.']);
    const once = saveProject({ ...newProject(), routes: [r] });
    const back = loadProject(once);
    assert.ok(back.ok);
    assert.deepEqual(back.project.routes[0], r, 'reopened exactly');
    assert.deepEqual(holes(back.project.routes[0]), ['Every curve point needs an x and a y.'], 'the error is still there (not x = 0)');
    assert.equal(saveProject(back.project), once, 'and saved again, the same bytes');
    // a NaN reaching the model (a number JSON cannot hold) is never stored: a care field, a point, the start
    const nan = addStep(setStart(newRoute(), { x: -36, y: Number.NaN, headingDeg: 0 }),
                        { do: 'driveTo', x: -12, y: -55.7, care: { timeoutS: Number.NaN, maxSpeedInPerS: 20 } });
    assert.deepEqual(nan.start, { x: -36, headingDeg: 0 });
    assert.deepEqual(nan.steps[0].care, { maxSpeedInPerS: 20 });
    const nanBack = loadProject(saveProject({ ...newProject(), routes: [nan] })).project.routes[0];
    assert.deepEqual(nanBack, nan);
    assert.ok(checkRoute(nanBack, [fixtureRobot('x')], fixtureRobot('x').field).some((i) => i.code === 'RC3' && i.level === 'error'),
              'the start with no y is still an error after a reopen (never y = 0)');
    // a null in a hand-edited file is a hole too, never 0
    const hand = JSON.parse(once);
    hand.routes[0].steps[0].points[1].x = null;
    hand.routes[0].start.y = null;
    const handBack = loadProject(JSON.stringify(hand)).project.routes[0];
    assert.deepEqual(handBack.steps[0].points[1], { y: -24 });
    assert.equal(handBack.start.y, undefined);
});

test('RT5: a step whose mechanism is not on the robot is KEPT and marked, never dropped', () => {
    const r = addStep(setStart(newRoute(), { x: -36, y: -55.7, headingDeg: 0 }), { do: 'move', mechanism: 'wheelie', preset: 'up', boundS: 1 });
    const back = loadProject(saveProject({ ...newProject(), routes: [r] }));
    assert.equal(back.project.routes[0].steps.length, 1);
    assert.ok(checkRoute(back.project.routes[0], [fixtureRobot('x')], fixtureRobot('x').field).some((i) => i.code === 'RC18' && i.stepIndex === 0));
});

test('an unknown field (a later builder\'s) is kept and written back; the hand-edit hashes are kept', () => {
    const p = { ...newProject(), exports: { 'src/team/skills_route.cpp#skills': 'sha256:00' }, future: { a: 1 } };
    const back = loadProject(saveProject(p));
    assert.deepEqual(back.project.future, { a: 1 });
    assert.equal(back.project.exports['src/team/skills_route.cpp#skills'], 'sha256:00');
});

test('the undo history is bounded, and an edit after an undo drops the redo list', () => {
    let h = historyOf(0);
    for (let i = 1; i <= HISTORY_LIMIT + 50; ++i) {
        h = commit(h, i);
    }
    assert.equal(h.past.length, HISTORY_LIMIT);
    h = undo(undo(h));
    assert.equal(h.future.length, 2);
    h = commit(h, 'new');
    assert.equal(h.future.length, 0);
    assert.equal(commit(h, h.present), h, 'committing the same state is not an edit');
});
