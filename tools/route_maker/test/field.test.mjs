// field.test.mjs — the field picture tells the truth (FP1, FP2), and the snapping rules (§1.4).
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { pictureToField, fieldToPicture, snapPoint, snapHeading, onField } from '../src/model/field.mjs';
import { fixtureRobot, REPO } from './helpers.mjs';

const PIC = JSON.parse(fs.readFileSync(path.join(REPO, 'tools/route_maker/assets/field.json'), 'utf8'));
const FIELD = fixtureRobot('x').field;  // the C++'s numbers (override_walls.hpp, override_elements.hpp)

test('FP1: the five landmarks -- the centre goal and the four Toggle places -- map to within 1 in of the C++ positions', () => {
    const want = { 'centre goal': FIELD.centreGoal };
    for (const t of FIELD.toggles) {
        want[t.name] = { x: t.x, y: t.y };
    }
    assert.equal(Object.keys(want).length, 5);
    for (const [name, at] of Object.entries(want)) {
        const px = PIC.landmarks[name];
        assert.ok(px, `field.json has no "${name}"`);
        const f = pictureToField(px[0], px[1], PIC);
        const d = Math.hypot(f.x - at.x, f.y - at.y);
        assert.ok(d < 1.0, `${name}: the picture puts it at (${f.x.toFixed(2)}, ${f.y.toFixed(2)}), the C++ at (${at.x}, ${at.y}): ${d.toFixed(2)} in`);
    }
});

test('FP3: the SCALE, from the picture\'s own tile seams (not the walls the transform is fitted to): each within 1 in, the outer span 93.6 in within 1 in', () => {
    // The field is six tiles wall face to wall face: a tile is 2 x the C++'s wallInnerFaceIn / 6 (23.40 in).
    const tile = (2 * FIELD.wallInnerFaceIn) / 6;
    const seams = PIC.tileSeamsPx;
    assert.ok(seams && Object.keys(seams.x).length === 4 && Object.keys(seams.y).length === 4, 'field.json has the eight measured seams');
    const at = {};
    for (const [k, px] of Object.entries(seams.x)) {
        const f = pictureToField(px, PIC.originPx[1], PIC);
        at[`x${k}`] = f.x;
        assert.ok(Math.abs(f.x - Number(k) * tile) < 1.0, `the seam at x = ${(Number(k) * tile).toFixed(1)} in is pictured at ${f.x.toFixed(2)}`);
    }
    for (const [k, py] of Object.entries(seams.y)) {
        const f = pictureToField(PIC.originPx[0], py, PIC);
        at[`y${k}`] = f.y;
        assert.ok(Math.abs(f.y - Number(k) * tile) < 1.0, `the seam at y = ${(Number(k) * tile).toFixed(1)} in is pictured at ${f.y.toFixed(2)}`);
    }
    // the span between the outer seams is independent of the origin: it is the scale itself (a 2 % scale error is
    // ~1.9 in here, while it moves a single seam only ~0.9 in)
    for (const axis of ['x', 'y']) {
        const span = at[`${axis}2`] - at[`${axis}-2`];
        assert.ok(Math.abs(span - 4 * tile) < 1.0, `${axis}: the outer seams are ${span.toFixed(2)} in apart through the page's scale, not ${(4 * tile).toFixed(1)}`);
    }
});

test('FP2: red at the bottom, blue at the top (the first red and blue tape pixels from the TOP of the picture)', () => {
    const red = pictureToField(...PIC.redTapePx, PIC);
    const blue = pictureToField(...PIC.blueTapePx, PIC);
    assert.ok(red.y < -50, `red tape at y = ${red.y}`);
    assert.ok(blue.y > 50, `blue tape at y = ${blue.y}`);
});

test('the picture <-> field transform round-trips, +x right, +y up', () => {
    const p = fieldToPicture(24, 36, PIC);
    const back = pictureToField(p.px, p.py, PIC);
    assert.ok(Math.abs(back.x - 24) < 1e-9 && Math.abs(back.y - 36) < 1e-9);
    assert.ok(fieldToPicture(10, 0, PIC).px > fieldToPicture(0, 0, PIC).px, '+x is to the right');
    assert.ok(fieldToPicture(0, 10, PIC).py < fieldToPicture(0, 0, PIC).py, '+y is up the picture');
});

test('snapping: the 12 in grid, flush to a wall, 15 deg headings; never past the threshold', () => {
    assert.deepEqual(snapPoint(11.2, -23.1, FIELD, 15, 2), { x: 12, y: -24 });
    assert.deepEqual(snapPoint(9.5, 5, FIELD, 15, 2), { x: 9.5, y: 5 }, '2.5 and 5 in from the grid: not snapped');
    const flush = FIELD.wallInnerFaceIn - 7.5;
    assert.equal(snapPoint(-61.9, 0, FIELD, 15, 2).x, Number((-flush).toFixed(3)), 'flush against the left wall');
    assert.equal(snapHeading(52), 45);
    assert.equal(snapHeading(-176), 180, '-180 is written +180');
    assert.equal(snapHeading(-172), -165, 'the nearest step, not the wall of the range');
    assert.equal(snapHeading(37.4, 1), 37);
    assert.ok(onField(70, -70, FIELD) && !onField(71, 0, FIELD));
});
