// numbers.test.mjs — the ONE number rule (W7) and NF1. Adversarial: each case tries to make a stored number
// and its printed form disagree.
import test from 'node:test';
import assert from 'node:assert/strict';
import { round3, fmt, parseTyped, isStored } from '../src/model/numbers.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { loadProject } from '../src/model/project.mjs';
import { everyNumberStored, newRoute, setStart, addStep, updateStep } from '../src/model/route.mjs';
import { goldens } from '../goldens.mjs';
import { seeded } from './helpers.mjs';

test('round3: the 0.001 grid, never -0, NaN stays NaN', () => {
    assert.equal(round3(12.3456), 12.346);
    assert.equal(round3(-0.0004), 0);
    assert.ok(Object.is(round3(-0.0004), 0), 'a "-0" never reaches the file');
    assert.ok(Number.isNaN(round3(Number.NaN)));
    assert.ok(Number.isNaN(round3(Infinity)));
    assert.ok(Number.isNaN(round3('12')), 'a string is not silently a number');
});

test('fmt: at most 3 decimals, no trailing zeros, no exponent, no -0', () => {
    assert.equal(fmt(3), '3');
    assert.equal(fmt(-55.7), '-55.7');
    assert.equal(fmt(0.5), '0.5');
    assert.equal(fmt(12.3456), '12.346');
    assert.equal(fmt(1e-7), '0');
    assert.equal(fmt(-1e-7), '0');
    assert.equal(fmt(123456.0001), '123456');
    assert.equal(fmt(1e21 / 1e21), '1');
    assert.throws(() => fmt(Number.NaN), /not a finite number/);
});

test('parseTyped: plain decimals only (no exponent, hex, units or commas)', () => {
    assert.equal(parseTyped(' 7 '), 7);
    assert.equal(parseTyped('-3.5'), -3.5);
    assert.equal(parseTyped('.5'), 0.5);
    assert.equal(parseTyped('12.34567'), 12.346);
    for (const bad of ['', '12in', '1e3', '0x10', '1,5', '--1', '.', 'NaN', 'Infinity']) {
        assert.ok(Number.isNaN(parseTyped(bad)), `"${bad}" must not read as a number`);
    }
});

test('NF1: every number an edit stores is on the grid, and the export prints exactly fmt() of it', () => {
    const rnd = seeded(77);
    let r = setStart(newRoute(), { x: -36.123456, y: -55.70049, headingDeg: 359.9996 });
    for (let i = 0; i < 60; ++i) {
        const x = (rnd() - 0.5) * 120;
        const y = (rnd() - 0.5) * 120;
        const kind = i % 4;
        if (kind === 0) {
            r = addStep(r, { do: 'driveTo', x, y, faceFirst: rnd() < 0.5, care: { timeoutS: rnd() * 5, maxSpeedInPerS: rnd() * 60 } });
        } else if (kind === 1) {
            r = addStep(r, { do: 'turnTo', headingDeg: (rnd() - 0.5) * 1000, care: { maxTurnDegPerS: rnd() * 400 } });
        } else if (kind === 2) {
            r = addStep(r, { do: 'curveTo', points: [{ x, y }, { x: x + rnd() * 20, y: y + rnd() * 20, headingDeg: rnd() * 720 }] });
        } else {
            r = addStep(r, { do: 'wait', seconds: rnd() * 3 });
        }
        r = updateStep(r, r.steps.length - 1, { label: `s${i}` });
    }
    assert.ok(everyNumberStored(r), 'every stored number is on the 0.001 grid');
    const text = exportRegion(r);
    // every number token in the export is fmt() of itself (so of the stored value): no 4th decimal, no exponent
    for (const m of text.matchAll(/-?\d+(?:\.\d+)?(?=_in|_deg|_s\b|_volt| \* math)/g)) {
        const tok = m[0];
        assert.equal(fmt(Number(tok)), tok, `"${tok}" is not the one formatter's text`);
        assert.ok(isStored(Number(tok)), `"${tok}" is not a stored number`);
    }
    // and the start prints exactly as stored
    assert.match(text, new RegExp(`StartPose::field\\(${fmt(r.start.x)}_in, ${fmt(r.start.y)}_in, ${fmt(r.start.headingDeg)}_deg\\)`));
    assert.equal(r.start.headingDeg, 0, '359.9996 deg is stored wrapped and rounded: 0');
});

test('NF1: a HOLE (a start\'s or a curve point\'s part left out) stores no number -- not one off the grid; a NaN is', () => {
    let r = addStep(setStart(newRoute(), { y: -36, headingDeg: 90 }), { do: 'curveTo', points: [{ x: 12.5, y: 0 }, { y: 24 }] });
    assert.deepEqual([r.start, r.steps[0].points[1]], [{ y: -36, headingDeg: 90 }, { y: 24 }]);
    assert.ok(everyNumberStored(r), 'holes are not numbers off the grid');
    r = { ...r, start: { ...r.start, x: Number.NaN } };  // past the model's rounding (never stored by it): refused
    assert.ok(!everyNumberStored(r));
    r = { ...r, start: { y: -36, headingDeg: 90.0004 } };
    assert.ok(!everyNumberStored(r), 'a 4th decimal is still caught beside a hole');
});

test('NF1: every golden route stores only grid numbers, and its export prints them through fmt()', () => {
    for (const [name, project] of goldens()) {
        const loaded = loadProject(project);
        assert.ok(loaded.ok, name);
        assert.ok(everyNumberStored(loaded.project.routes[0]), `${name}: a number off the grid`);
    }
});
