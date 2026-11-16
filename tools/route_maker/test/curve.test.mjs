// curve.test.mjs — CV1: the page's curve IS the library's (RMP1 brief [R6], §2.2 CV1).
// test/goldens/route_maker/curves.txt is made by the C++ (shulib_route_maker curves: the REAL CurvePath,
// checkCurve and appendCurveCheckWords) and kept fresh by the suite. Here every curve is rebuilt by
// curve.mjs: the verdict, the why, the point and the library's sentence EXACTLY; the length and 64 sampled
// positions within 0.001 in.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { CurvePath, checkCurve, curveCheckWords, kDefaultCurveConfig } from '../src/model/curve.mjs';
import { GOLDENS } from './helpers.mjs';

function readCurves() {
    const lines = fs.readFileSync(path.join(GOLDENS, 'curves.txt'), 'utf8').split('\n').filter((l) => l && !l.startsWith('#'));
    const out = [];
    for (let i = 0; i < lines.length; i += 4) {
        const c = lines[i].split(' ');
        const v = lines[i + 1];
        const L = Number(lines[i + 2].split(' ')[1]);
        const S = lines[i + 3].split(' ').slice(1).map(Number);
        const n = Number(c[6]);
        const points = [];
        for (let k = 0; k < n; ++k) {
            const [x, y, h] = c.slice(7 + 3 * k, 10 + 3 * k);
            const p = { x: Number(x), y: Number(y) };
            if (h !== '-') {
                p.headingDeg = Number(h);
            }
            points.push(p);
        }
        const vparts = v.split(' ');
        out.push({
            id: Number(c[1]), reverse: c[2] === '1', start: { x: Number(c[3]), y: Number(c[4]), headingDeg: Number(c[5]) }, points,
            verdict: vparts[1], why: vparts[2], point: Number(vparts[3]), words: vparts.slice(4).join(' '), length: L, samples: S,
        });
    }
    return out;
}

const curves = readCurves();

test('CV1 reads 200 curves, every verdict among them', () => {
    assert.equal(curves.length, 200);
    const seen = new Set(curves.map((c) => `${c.verdict}/${c.why}`));
    for (const v of ['Ok/None', 'StartBehind/None', 'TightBend/None', 'Invalid/TooClose', 'Invalid/NoDirection',
                     'Invalid/AtRobot', 'Invalid/DoublesBack']) {
        assert.ok(seen.has(v), `the golden has no ${v} curve`);
    }
});

test('CV1: every verdict, why, point and sentence equals the library\'s; every position within 0.001 in', () => {
    let worst = 0;
    for (const c of curves) {
        const path = new CurvePath(c.points, c.reverse).build(c.start, kDefaultCurveConfig);
        const chk = checkCurve(path, kDefaultCurveConfig);
        const tag = `curve ${c.id}`;
        assert.equal(chk.verdict, c.verdict, `${tag}: verdict`);
        assert.equal(chk.why, c.why, `${tag}: why`);
        assert.equal(chk.point, c.point, `${tag}: point`);
        assert.equal(curveCheckWords(chk, kDefaultCurveConfig), c.words, `${tag}: the library's words`);
        assert.ok(Math.abs(path.length() - c.length) < 1e-3, `${tag}: length ${path.length()} vs ${c.length}`);
        for (let k = 0; k < 64; ++k) {
            const s = path.at(path.length() * k / 63);
            const d = Math.hypot(s.x - c.samples[2 * k], s.y - c.samples[2 * k + 1]);
            worst = Math.max(worst, d);
            assert.ok(d < 1e-3, `${tag}: sample ${k} is ${d} in from the library's`);
        }
    }
    assert.ok(worst < 1e-3);
});

test('CV1 is not vacuous: a curve.mjs without the 90 deg chord rule disagrees with the library somewhere', () => {
    // The mutation RM23 run by hand in the log removes the rule from curve.mjs itself; this case pins that the
    // golden HAS curves the rule decides (DoublesBack), so the mutation cannot pass by luck.
    assert.ok(curves.filter((c) => c.why === 'DoublesBack').length >= 5);
});
