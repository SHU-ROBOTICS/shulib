// claw.test.mjs — RMP1-O: where the claw is, and a drive placed BY the claw (model/claw.mjs).
// The claw must land on the point asked for once the PLAN (plannedPoses: the pose the library ends at) is applied,
// not merely satisfy the formula that made it.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { execFileSync } from 'node:child_process';
import { clawSpotOf, clawAt, centreForClaw, clawLeg, isClawStep } from '../src/model/claw.mjs';
import { plannedPoses, normalizeRoute, newRoute } from '../src/model/route.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { goldens, goldenRobot } from '../goldens.mjs';
import { loadProject } from '../src/model/project.mjs';
import { fixtureRobot, REPO, GOLDENS } from './helpers.mjs';

const SPOT = { forwardIn: 6.268, sideIn: 0 };  // 15 / 2 - 1.232 (kClawGrabInsetIn, the team lead, 2026-11-20)
const near = (a, b, tol, what) => assert.ok(Math.hypot(a.x - b.x, a.y - b.y) <= tol, `${what}: (${a.x}, ${a.y}) vs (${b.x}, ${b.y})`);

/** Step `do` from `start` placed by the claw on `q`; returns the claw where the PLAN ends. */
function landed(start, step, q, spot = SPOT) {
    const route = { name: 't', robot: 'one', start, steps: [step] };
    const leg = clawLeg(step, plannedPoses(route)[0]);
    const c = centreForClaw(q, leg.from, spot, leg);
    assert.ok(!c.refused, c.refused);
    const placed = step.points ? { ...step, points: step.points.map((p, k) => (k === leg.point ? { ...p, ...c } : p)) } : { ...step, ...c };
    const to = plannedPoses({ ...route, steps: [placed] })[0].to;
    return { claw: clawAt(to, spot), centre: c, to };
}

test('the claw is forwardIn ahead of the centre on its centre line, at every heading', () => {
    for (let h = -179; h <= 180; h += 7) {
        const c = clawAt({ x: 10, y: -20, headingDeg: h }, SPOT);
        assert.ok(Math.abs(Math.hypot(c.x - 10, c.y + 20) - 6.268) < 1e-9, `distance at ${h}`);
        const b = (Math.atan2(c.y + 20, c.x - 10) * 180) / Math.PI;
        assert.ok(Math.abs(((b - h + 540) % 360) - 180) < 1e-9, `bearing at ${h}: ${b}`);
    }
    near(clawAt({ x: 0, y: 0, headingDeg: 90 }, { forwardIn: 6, sideIn: 2 }), { x: -2, y: 6 }, 1e-9, 'side is to the LEFT');
});

test('a straight drive placed by the claw: the claw lands on the point (every direction), the centre short of it on the leg', () => {
    const start = { x: -40, y: -30, headingDeg: 0 };
    for (let a = 0; a < 360; a += 15) {
        const q = { x: -40 + 30 * Math.cos((a * Math.PI) / 180), y: -30 + 30 * Math.sin((a * Math.PI) / 180) };
        const r = landed(start, { do: 'driveTo', faceFirst: true }, q);
        near(r.claw, q, 0.003, `driveTo at ${a} deg`);
        assert.ok(Math.abs(Math.hypot(r.centre.x - q.x, r.centre.y - q.y) - 6.268) < 0.003, 'the centre 6.268 in short');
        assert.ok(Math.hypot(r.centre.x + 40, r.centre.y + 30) < 30, 'the centre between the start and the point (not past it)');
    }
});

test('backing up by the claw: the centre is PAST the point (the claw trails it), and the claw lands on it', () => {
    const start = { x: 0, y: 0, headingDeg: 180 };
    const q = { x: 24, y: 10 };
    const r = landed(start, { do: 'reverseTo' }, q);
    near(r.claw, q, 0.003, 'reverseTo');
    assert.ok(Math.hypot(r.centre.x, r.centre.y) > Math.hypot(q.x, q.y), 'the centre beyond the point');
});

test('a curve\'s last leg by the claw: from the point before it, with and without a nose heading, forward and back', () => {
    const start = { x: -50, y: -50, headingDeg: 45 };
    const q = { x: 20, y: 5 };
    for (const kind of ['curveTo', 'reverseCurveTo']) {
        near(landed(start, { do: kind, points: [{ x: -20, y: -30 }, { x: 0, y: 0 }] }, q).claw, q, 0.003, `${kind}, two points`);
        near(landed(start, { do: kind, points: [{ x: 0, y: 0 }] }, q).claw, q, 0.003, `${kind}, one point (the leg from the start)`);
        near(landed(start, { do: kind, points: [{ x: -20, y: -30 }, { x: 0, y: 0, headingDeg: 120 }] }, q).claw, q, 0.003, `${kind}, a nose`);
        // a nose of 0 is a nose (not "none": the chord's direction would leave the claw 4.4 in off)
        const zero = landed(start, { do: kind, points: [{ x: -20, y: -30 }, { x: 0, y: 0, headingDeg: 0 }] }, q);
        near(zero.claw, q, 0.003, `${kind}, a nose of 0`);
        assert.equal(zero.to.headingDeg, 0, `${kind}: it ends facing its nose`);
    }
    const leg = clawLeg({ do: 'curveTo', points: [{ x: 1, y: 2 }, { x: 3, y: 4 }] }, plannedPoses({ start, steps: [{ do: 'curveTo', points: [{ x: 1, y: 2 }, { x: 3, y: 4 }] }] })[0]);
    assert.deepEqual([leg.from, leg.point], [{ x: 1, y: 2 }, 1]);
});

test('a curve whose first point the library DROPS (within 1 in of the start, no nose): the last leg runs from the start', () => {
    // the reviewer's cases (a student begins a curve by clicking on the robot): 0.338 and 0.877 in off before the fix
    const start = { x: 0, y: 0, headingDeg: 0 };
    for (const p0 of [{ x: 0.9, y: 0.3 }, { x: 0, y: 0.95 }, { x: -0.6, y: 0.7 }]) {
        for (const q of [{ x: 10, y: 8 }, { x: 24, y: 0 }, { x: 4, y: 12 }]) {
            for (const kind of ['curveTo', 'reverseCurveTo']) {
                const step = { do: kind, points: [p0, { x: 20, y: 5 }] };
                assert.equal(plannedPoses({ start, steps: [step] })[0].curve.firstDrivenPoint(), 2, 'the first point dropped');
                near(landed(start, step, q).claw, q, 0.003, `${kind} from (${p0.x}, ${p0.y}) to (${q.x}, ${q.y})`);
            }
        }
    }
});

test('a point closer to the leg\'s start than the claw reaches, plus 1 in, is REFUSED in words; past it is placed', () => {
    const from = { x: 0, y: 0 };
    const r = centreForClaw({ x: 6, y: 0 }, from, SPOT);
    assert.match(r.refused, /^Too close: the claw is 6\.268 in ahead of the centre\./);
    assert.ok(centreForClaw({ x: 0, y: 6.268 }, from, SPOT).refused, 'exactly the reach: refused (the robot would not move)');
    assert.ok(centreForClaw({ x: 0, y: 7.267 }, from, SPOT).refused, 'the centre would move under 1 in: refused');
    assert.deepEqual(centreForClaw({ x: 0, y: 7.3 }, from, SPOT), { x: 0, y: 1.032 });
    assert.ok(!centreForClaw({ x: 3, y: 0 }, from, SPOT, { back: true }).refused, 'backing up reaches any point but the start');
});

test('EVERY placement the page accepts lands the claw on the point once the centre is rounded (just past the reach, every direction)', () => {
    // BUG CAUGHT (review): 0.0004-0.01 in past the reach the centre sat a few thousandths from the start, and rounding it
    // to 0.001 in turned the leg: the claw 12.5 in off while the page accepted the placement
    const start = { x: 10.123, y: -20.456, headingDeg: 0 };
    let placed = 0;
    for (let a = 1; a < 360; a += 7) {
        for (const extra of [0.0004, 0.001, 0.002, 0.01, 0.1, 0.5, 0.999, 1.0005, 1.001, 1.01, 1.5, 3]) {
            for (const kind of ['driveTo', 'reverseTo']) {
                const d = 6.268 + extra;
                const q = { x: start.x + d * Math.cos((a * Math.PI) / 180), y: start.y + d * Math.sin((a * Math.PI) / 180) };
                const step = { do: kind, faceFirst: true };
                const c = centreForClaw(q, start, SPOT, clawLeg(step, plannedPoses({ start, steps: [step] })[0]));
                if (c.refused) {
                    assert.ok(kind === 'driveTo' && extra < 1.001, `${kind} ${extra} past at ${a}: refused`);
                    continue;
                }
                placed += 1;
                const to = plannedPoses({ start, steps: [{ ...step, ...c }] })[0].to;
                near(clawAt(to, SPOT), q, 0.006, `${kind} ${extra} in past the reach at ${a} deg`);
            }
        }
    }
    assert.ok(placed > 52 * 15, `placed ${placed}`);
});

test('a sideways claw: the claw still lands on the point (the leg turns to keep it on the line)', () => {
    const spot = { forwardIn: 6, sideIn: 2.5 };
    for (const [step, q] of [[{ do: 'driveTo' }, { x: 30, y: 12 }], [{ do: 'reverseTo' }, { x: -25, y: 18 }]]) {
        near(landed({ x: 0, y: 0, headingDeg: 0 }, step, q, spot).claw, q, 0.003, `${step.do} sideways`);
    }
});

test('the robots: robot x (a test robot) has no claw; a description\'s clawSpot is read; both real robots are 6.268 / 0', (t) => {
    assert.equal(clawSpotOf(fixtureRobot('x')), null);
    assert.equal(clawSpotOf({ clawSpot: { forwardIn: 'a', sideIn: 0 } }), null);
    assert.deepEqual(clawSpotOf({ clawSpot: SPOT }), SPOT);
    const bin = path.join(REPO, 'build', 'test', 'shulib_route_maker');
    if (!fs.existsSync(bin)) {
        t.skip('build/test/shulib_route_maker is not built: the C++ suite pins the descriptions');
        return;
    }
    for (const id of ['one', 'two']) {
        const d = JSON.parse(execFileSync(bin, ['describe', '--robot', id], { encoding: 'utf8' }));
        assert.deepEqual(d.clawSpot, SPOT, `robot ${id}`);
        near(landed({ x: 0, y: 0, headingDeg: 0 }, { do: 'driveTo' }, { x: 30, y: 30 }, clawSpotOf(d)).claw, { x: 30, y: 30 }, 0.003, id);
    }
});

test('a claw step is a mechanism step on the claw (by its name)', () => {
    const robot = { mechanisms: [{ name: 'claw', ident: 'claw' }, { name: 'ramp', ident: 'ramp' }] };
    assert.ok(isClawStep({ do: 'set', mechanism: 'claw', position: 'closed' }, robot));
    assert.ok(!isClawStep({ do: 'set', mechanism: 'ramp', position: 'up' }, robot));
    assert.ok(!isClawStep({ do: 'driveTo', x: 1, y: 2 }, robot));
});

test('the export does not know the claw: every golden exports byte for byte with a claw on its robot; a step placed by it exports its CENTRE', () => {
    const names = goldens().map(([name]) => name);
    assert.ok(names.length >= 17);
    for (const name of names) {
        const loaded = loadProject(fs.readFileSync(path.join(GOLDENS, `${name}.route.json`), 'utf8'));
        const robot = goldenRobot(loaded.project.routes[0].robot);
        const committed = fs.readFileSync(path.join(GOLDENS, `${name}.cpp`), 'utf8');
        assert.equal(`${exportRegion(loaded.project.routes, robot && { ...robot, clawSpot: SPOT })}\n`, committed, name);
    }
    const c = centreForClaw({ x: 30, y: 0 }, { x: 0, y: 0 }, SPOT);
    const route = normalizeRoute({ ...newRoute(), start: { x: 0, y: 0, headingDeg: 0 }, steps: [{ do: 'driveTo', ...c }] });
    assert.ok(exportRegion([route], null).includes('r.driveTo(23.732_in, 0_in'), 'the centre, 30 - 6.268');
});
