// feel.test.mjs — DRIVING FEEL (RMP1-B; the brief's FE1, FE2, FE4 and the feel region's round trip).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { shapeAxis, axisValid, checkFeel, rampWords, rampSeconds, exportFeel, parseFeel, kFeelText, editDriving, feelComment } from '../src/model/feel.mjs';
import { FE2_ROWS, feelRowsHeader } from '../goldens.mjs';
import { GOLDENS } from './helpers.mjs';

/** C's %a text as the exact double it names (BigInt mantissa, so nothing rounds on the way). */
export function fromHex(s) {
    const m = /^(-?)0x([0-9a-f])(?:\.([0-9a-f]*))?p([+-]\d+)$/i.exec(s.trim());
    assert.ok(m, `not a %a number: ${s}`);
    const frac = m[3] || '';
    const mant = BigInt(`0x${m[2]}${frac}`);
    const v = Number(mant) * 2 ** (Number(m[4]) - 4 * frac.length);
    return m[1] ? -v : v;
}

test('FE1: the page\'s shapeAxis equals the C++ at all 9 630 points of the golden, bit for bit (the sign of zero counts)', () => {
    const lines = fs.readFileSync(path.join(GOLDENS, 'feel_curve.txt'), 'utf8').trim().split('\n');
    assert.equal(lines.length, 9630);
    let differ = 0;
    let first = '';
    for (const l of lines) {
        const [x, db, ex, want] = l.split(' ').map(fromHex);
        const got = shapeAxis(x, { deadband: db, expo: ex });
        if (!Object.is(got, want)) {
            ++differ;
            first = first || `${l} -> ${got}`;
        }
    }
    assert.equal(differ, 0, `differs at ${differ} points, first: ${first}`);
    // the test can fail: a centred stick is +0 (not -0) and the curve keeps full speed at the rail
    assert.ok(Object.is(shapeAxis(-0.03, { deadband: 0.05, expo: 0.6 }), 0));
    assert.equal(shapeAxis(1, { deadband: 0.3, expo: 1 }), 1);
});

test('FE1b: and at the EDGES the grid never lands on -- each dead zone\'s edge and the doubles either side of it, -0, the limits (the second review)', () => {
    const lines = fs.readFileSync(path.join(GOLDENS, 'feel_edges.txt'), 'utf8').trim().split('\n');
    assert.equal(lines.length, 490);
    const bad = lines.filter((l) => {
        const [x, db, ex, want] = l.split(' ').map(fromHex);
        return !Object.is(shapeAxis(x, { deadband: db, expo: ex }), want);
    });
    assert.deepEqual(bad, []);
    assert.ok(lines.some((l) => l.startsWith('-0x0p+0 ')), 'a -0 stick is in the table');
});

test('FE2: checkFeel refuses exactly what AxisShaping::valid() refuses, at every boundary (feel_valid_rows.hpp is the C++ twin, fresh)', () => {
    for (const [deadband, curve] of FE2_ROWS) {
        const cpp = deadband >= 0 && deadband < 1 && curve >= 0 && curve <= 1;  // input_shaping.hpp's valid(), as written there
        for (const axis of ['forward', 'strafe', 'turn']) {  // DriverProfile::valid() checks all three
            const page = checkFeel({ [axis]: { deadband, curve } }, { holonomic: true }).filter((x) => x.level === 'error').length === 0;
            assert.equal(page, cpp, `${axis}: deadband ${deadband}, curve ${curve}`);
        }
        assert.equal(axisValid({ deadband, expo: curve }), cpp);
    }
    assert.equal(FE2_ROWS.filter(([d, e]) => axisValid({ deadband: d, expo: e })).length, 4, 'only 0 / 0.999.. x 0 / 1 pass');
    assert.equal(fs.readFileSync(path.join(GOLDENS, 'feel_valid_rows.hpp'), 'utf8'), feelRowsHeader(),
                 'feel_valid_rows.hpp is stale: node tools/route_maker/goldens.mjs --write, then read the diff');
    // the page's own stricter rule: a negative ramp (the C++ reads it as off) and a ramp that is not a number
    assert.equal(checkFeel({ rampPerSecond: -1 })[0].field, 'rampPerSecond');
    assert.equal(checkFeel({ rampPerSecond: Number.NaN })[0].field, 'rampPerSecond');
    assert.equal(checkFeel({ rampPerSecond: Infinity })[0].field, 'rampPerSecond', 'no C++ literal for it');
    assert.deepEqual(checkFeel({ rampPerSecond: 0 }), []);
});

test('FE4: the ramp readout -- 4 per second is full speed from rest in 0.25 s, and a stop as long (the slew limits both ways); 0 is no ramp', () => {
    assert.equal(rampWords(4), 'full speed from rest in 0.25 s, and a stop from full speed takes as long');
    assert.equal(rampSeconds(2.5), 0.4);
    assert.equal(rampSeconds(0), null);
    assert.equal(rampSeconds(-3), null);
    assert.match(rampWords(0), /^no ramp/);
});

test('the feel region: the library\'s default round-trips, and only the stated fields are written, in declaration order', () => {
    const def = exportFeel({ mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' });
    assert.ok(def.includes('inline constexpr DriverProfile kFeel{.mode = ArcadeBody};'));
    const back = parseFeel(def);
    assert.ok(back.ok, back.words);
    assert.deepEqual(exportFeel(back.driving), def);
    const full = { mode: 'tankSticks', turn: { curve: 0.3 }, forward: { deadband: 0.08, curve: 0.6 }, rampPerSecond: 4, rezeroButton: 'Left', onRelease: 'brake' };
    const k = kFeelText(full).join('\n');
    // forward before yaw (declaration order), whatever order the block was edited in; a long one is a block
    assert.ok(k.indexOf('.forward') < k.indexOf('.yaw') && k.indexOf('.yaw') < k.indexOf('.slew') && k.indexOf('.slew') < k.indexOf('.mode'), k);
    assert.ok(k.startsWith('inline constexpr DriverProfile kFeel{\n    .forward = {.deadband = 0.08, .expo = 0.6},'));
    const r = parseFeel(exportFeel(full));
    assert.ok(r.ok, r.words);
    assert.deepEqual({ ...r.driving, words: undefined }, { ...full, words: undefined });
    // a hand edit the page did not write is not read (never guessed at)
    const odd = exportFeel(full).map((l) => l.replace('.slew', '.slow'));
    assert.equal(parseFeel(odd).ok, false);
    const tick = [...def.slice(0, -1), '    .tickPeriod = 10_ms,', '};'];
    assert.equal(parseFeel(tick).ok, false, 'a kLoop with a third line is not the page\'s');
});

test('a feel edit rewrites the words that describe it (a stale comment is never kept); an unchanged feel keeps the file\'s', () => {
    const one = fs.readFileSync(path.join(GOLDENS, 'driver_one_marked.hpp'), 'utf8').split('\n');
    const from = one.indexOf('// [feel one]') + 1;
    const lines = one.slice(from, one.indexOf('// [/feel one]'));
    const r = parseFeel(lines);
    assert.ok(r.ok, r.words);
    assert.match(r.driving.words.comment.join(' '), /The library's default, as robot two's/);
    const edited = editDriving(r.driving, { forward: { deadband: 0.08, curve: 0.6 } });
    const text = exportFeel(edited).join('\n');
    assert.doesNotMatch(text, /The library's default/, 'the old comment said "no curve": it must go');
    assert.match(text, /Forward: dead zone 0\.08, curve 0\.6\./);
    const relz = exportFeel(editDriving(r.driving, { onRelease: 'brake' })).join('\n');
    assert.match(relz, /\.releaseBrakeMode = Brake, +\/\/ let go of the sticks: the drive brakes/);
    assert.deepEqual(feelComment({}), feelComment({ mode: 'arcade' }));
});

test('every feel the page writes, it reads back -- the one-line form with an axis carrying BOTH a dead zone and a curve included (the review\'s finding)', () => {
    const axes = [undefined, { deadband: 0.1 }, { curve: 0.3 }, { deadband: 0.1, curve: 0.3 }];
    let oneLine = 0;
    for (const forward of axes) {
        for (const turn of axes) {
            for (const ramp of [undefined, 4]) {
                for (const mode of ['arcade', 'tankSticks']) {
                    const d = { mode, rezeroButton: 'Y', onRelease: 'coast', ...(forward ? { forward } : {}), ...(turn ? { turn } : {}),
                                ...(ramp !== undefined ? { rampPerSecond: ramp } : {}) };
                    const lines = exportFeel(d);
                    oneLine += kFeelText(d).length === 1 ? 1 : 0;
                    const r = parseFeel(lines);
                    assert.ok(r.ok, `${JSON.stringify(d)}: ${r.words}`);
                    assert.deepEqual(exportFeel(r.driving), lines, JSON.stringify(d));
                }
            }
        }
    }
    assert.ok(oneLine > 10, `the one-line form was reached ${oneLine} times`);
});

test('hexDouble writes the exact double C++ reads: a subnormal without the hidden 1, and no literal for an infinity (the second review)', async () => {
    const { hexDouble } = await import('../goldens.mjs');
    for (const v of [Number.MIN_VALUE, -Number.MIN_VALUE * 3, 2 ** -1023, 0.05, -1, 1 - 2 ** -53]) {
        assert.ok(Object.is(fromHex(hexDouble(v)), v), `${v} -> ${hexDouble(v)}`);
    }
    assert.throws(() => hexDouble(Infinity), /no C\+\+ literal/);
});

test('a project file\'s feel is judged as the export PRINTS it: a dead zone of 0.9996 is the 1 the file would say, and refused (the second review)', async () => {
    const { drivingFromSaved } = await import('../src/model/feel.mjs');
    const d = { mode: 'arcade', forward: { deadband: 0.9996, curve: 0.3 }, rampPerSecond: 4.00001 };
    assert.deepEqual(checkFeel(d).filter((x) => x.level === 'error'), [], 'unrounded it would pass');
    const r = drivingFromSaved(d);
    assert.deepEqual(r.forward, { deadband: 1, curve: 0.3 });
    assert.equal(r.rampPerSecond, 4);
    assert.ok(checkFeel(r).some((x) => x.level === 'error'), 'rounded, the page refuses what the build would');
});

test('while the robot\'s programs take only the default feel, any other is refused with the drive program\'s own words; the default passes', async () => {
    const { DEFAULT_ONLY_REFUSAL, isDefaultFeel } = await import('../src/model/feel.mjs');
    const src = fs.readFileSync(path.join(GOLDENS, '..', '..', '..', 'src', 'programs', 'drive_program.cpp'), 'utf8');
    if (src.includes('driverLineMatchesFeel')) {
        assert.ok(src.includes(`"${DEFAULT_ONLY_REFUSAL}`), 'the page quotes the static_assert\'s own sentence');
    }
    const def = { mode: 'tankSticks', rezeroButton: 'Up', onRelease: 'brake' };
    assert.ok(isDefaultFeel(def));
    assert.deepEqual(checkFeel(def, { defaultOnly: true }), []);
    for (const d of [{ forward: { curve: 0.6 } }, { turn: { deadband: 0.1 } }, { rampPerSecond: 4 }, { strafe: { curve: 0.01 } }]) {
        assert.equal(isDefaultFeel(d), false, JSON.stringify(d));
        assert.deepEqual(checkFeel(d, { defaultOnly: true }).filter((x) => x.level === 'error').map((x) => x.refusal), [DEFAULT_ONLY_REFUSAL], JSON.stringify(d));
        assert.deepEqual(checkFeel(d).filter((x) => x.level === 'error'), [], 'without the rule it is a valid feel');
    }
    assert.ok(isDefaultFeel({ forward: { deadband: 0.05, curve: 0 }, rampPerSecond: 0 }), 'the default spelled out is the default');
    for (const d of [{ forward: { deadband: 0 } }, { forward: { deadband: 0.04 } }, { turn: { deadband: 0.049 } }]) {
        assert.equal(isDefaultFeel(d), false, `below the default dead zone: ${JSON.stringify(d)}`);
    }
});

test('the feel\'s comment is kept while its numbers are; it says HA-112 while an axis is the library\'s unfelt default (the review\'s finding)', () => {
    const one = fs.readFileSync(path.join(GOLDENS, 'driver_one_marked.hpp'), 'utf8').split('\n');
    const r = parseFeel(one.slice(one.indexOf('// [feel one]') + 1, one.indexOf('// [/feel one]')));
    const said = (d) => exportFeel(d).join('\n');
    // "Use the library's defaults" on a feel that is the default: nothing changes, the HA-112 note included
    assert.equal(said(editDriving(r.driving, { forward: undefined, turn: undefined, strafe: undefined, rampPerSecond: undefined })), said(r.driving));
    // only the release changed: the comment (and its HA-112) stays, the release line follows
    const brake = said(editDriving(r.driving, { onRelease: 'brake' }));
    assert.match(brake, /HA-112/);
    assert.match(brake, /The library's default, as robot two's/);
    // a curve on forward only: the page's comment, which still says turn is the library's unfelt default
    const curve = said(editDriving(r.driving, { forward: { curve: 0.6 } }));
    assert.doesNotMatch(curve, /The library's default, as robot two's/);
    assert.match(curve, /HA-112/);
    const allSet = feelComment({ forward: { deadband: 0.1, curve: 0.6 }, turn: { deadband: 0.08 } }).join(' ');
    assert.doesNotMatch(allSet, /HA-112/, 'no axis at the default: nothing unfelt to say');
});
