// check_route.test.mjs — RMP1-K D1: the bench CHECK route of a route's mechanism steps (check_route.mjs).
import test from 'node:test';
import assert from 'node:assert/strict';
import { checkRoute, checkName, checkWords, CHECK_HOME_S } from '../src/model/check_route.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { checkRoute as issuesOf } from '../src/model/checks.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { fixtureRobot } from './helpers.mjs';

const X = fixtureRobot('x');  // arm (rests at its stop), lift (switch-homed), flywheel, intake (power), hood (pneumatic)
const START = { x: -36, y: -55.7, headingDeg: 0 };
const route = (steps, name = 'skills') => steps.reduce((r, s) => addStep(r, s), { ...setStart(newRoute(name), START) });

const EVERY = [
    { do: 'driveTo', x: -12, y: -55.7, label: 'go' },
    { do: 'home', mechanism: 'lift', boundS: 2 },
    { do: 'move', mechanism: 'lift', preset: 'low', boundS: 2, label: 'lift up', care: { onFail: { skipTo: 'go' } } },
    { do: 'startMove', mechanism: 'arm', preset: 'score', boundS: 1.5 },
    { do: 'turnTo', headingDeg: 90 },
    { do: 'waitUntilDone', mechanism: 'arm', boundS: 1 },
    { do: 'set', mechanism: 'hood', position: 'up' },
    { do: 'run', mechanism: 'intake', volts: 8, forS: 0.8 },
    { do: 'wait', seconds: 0.5 },
    { do: 'curveTo', points: [{ x: 0, y: -40 }, { x: 12, y: -24 }] },
    { do: 'startRun', mechanism: 'intake', volts: -6, forS: 0.4 },
    { do: 'startHome', mechanism: 'lift', boundS: 2 },
    { do: 'reverseTo', x: -24, y: -30, faceFirst: true },
    { do: 'waitUntilDone', mechanism: 'lift', boundS: 2 },
    { do: 'move', mechanism: 'lift', to: 18, boundS: 3 },
];

test('D1: every mechanism step and wait kept in order with its own numbers; every drive, turn and curve left out', () => {
    const src = route(EVERY);
    const c = checkRoute(src, X);
    assert.ok(c.route, c.why);
    assert.deepEqual(c.route.steps.map((s) => s.do), ['home', 'move', 'move', 'set', 'run', 'wait', 'run', 'home', 'move'],
                     'each start form runs to its end (nothing drives beside it); each waitUntilDone left out');
    assert.deepEqual(c.from, [1, 2, 3, 6, 7, 8, 10, 11, 14]);
    c.route.steps.forEach((s, k) => {
        const { label, care, view, touches, ...want } = src.steps[c.from[k]];  // eslint-disable-line no-unused-vars
        want.do = want.do.replace(/^start(\w)/, (_, c1) => c1.toLowerCase());
        assert.deepEqual({ ...s, care: undefined }, { ...want, care: undefined }, `step ${k}: its own numbers`);
        assert.deepEqual(s.care, { onFail: 'carryOn' }, 'each check step carries on: one run shows every mechanism');
        assert.equal(s.label, undefined, 'no label (a skip has nowhere to go in a check)');
    });
    assert.equal(c.route.name, 'skills mechs');
    assert.deepEqual(c.route.start, src.start, 'the start is the route\'s (the robot does not move)');
    assert.equal(c.route.together, false);
    assert.deepEqual(issuesOf(c.route, [X], X.field, { routes: [src, c.route], index: 1 }).filter((x) => x.level === 'error'), []);
});

test('D1: a home first only when the first step on a switch- or stall-homed mechanism is a move', () => {
    const noHome = checkRoute(route([{ do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }, { do: 'move', mechanism: 'arm', preset: 'high', boundS: 1 },
                                     { do: 'move', mechanism: 'lift', preset: 'high', boundS: 2 }]), X);
    assert.deepEqual(noHome.route.steps.map((s) => `${s.do} ${s.mechanism}`), ['home lift', 'move lift', 'move arm', 'move lift'],
                     'the lift (switch) gets ONE home before its first move; the arm rests at its stop: none');
    assert.deepEqual(noHome.route.steps[0], { do: 'home', mechanism: 'lift', boundS: CHECK_HOME_S, care: { onFail: 'carryOn' } });
    assert.deepEqual(noHome.from, [-1, 0, 1, 2]);
    const started = checkRoute(route([{ do: 'startHome', mechanism: 'lift', boundS: 2 }, { do: 'waitUntilDone', mechanism: 'lift', boundS: 2 },
                                      { do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }]), X);
    assert.deepEqual(started.route.steps.map((s) => s.do), ['home', 'move'], 'a route that homes it already: no second home');
    const y = fixtureRobot('y');  // swing: an arm homed by stall
    const stall = checkRoute(route([{ do: 'startMove', mechanism: 'swing', preset: 'up', boundS: 1 }]), y);
    assert.deepEqual(stall.route.steps.map((s) => s.do), ['home', 'move'], 'a stall-homed arm too');
});

test('D1: a route with no mechanism step: nothing to check, said', () => {
    for (const steps of [[], [{ do: 'driveTo', x: 0, y: 0 }], [{ do: 'wait', seconds: 1 }, { do: 'turnTo', headingDeg: 0 }]]) {
        const c = checkRoute(route(steps), X);
        assert.equal(c.route, null);
        assert.match(c.why, /no mechanism steps: there is nothing to check/);
    }
});

test('D1: its name fits the brain\'s menu, and a route already named so is said, never overwritten', () => {
    assert.equal(checkName({ name: 'skills', mirroredFor: 'two' }, X), 'skills mechs');
    const long = { name: 'a very long route name here', mirroredFor: 'two' };
    assert.equal(checkName(long, X), 'a very long route n mechs');
    assert.ok(checkName(long, X).length <= X.mirroredAutonNameMaxChars);
    assert.ok(checkName({ ...long, mirroredFor: undefined }, X).length <= X.autonNameMaxChars);
    const src = route([{ do: 'set', mechanism: 'hood', position: 'up' }]);
    const c = checkRoute(src, X, [src, { ...newRoute('Skills Mechs') }]);
    assert.equal(c.route, null);
    assert.match(c.why, /already named "skills mechs"/);
});

test('D1: the team\'s routes\' C++ is unchanged by the check beside them; the check\'s own is the ordinary export, no drive', () => {
    const team = [route(EVERY), route([{ do: 'set', mechanism: 'hood', position: 'down' }], 'left 1')];
    const c = checkRoute(team[0], X, team);
    const alone = exportRegion(team, X);
    const both = exportRegion([...team, c.route], X);
    const block = (text, id) => text.slice(text.indexOf(`chassis::RoutineResult ${id}Steps`), text.indexOf(`competition::AutonEntry ${id}Route`));
    for (const id of ['skills', 'left1']) {
        assert.equal(block(both, id), block(alone, id), `the route "${id}" written byte for byte`);
        assert.ok(block(alone, id).length > 100);
    }
    const mine = block(both, 'skillsMechs');
    assert.ok(mine.includes('chassis::Routine r{ctx, "skills mechs"};'));
    assert.ok(!/r\.(driveTo|reverseTo|turnTo|face|faceAway|curveTo|reverseCurveTo)\(/.test(mine), 'the robot stays where it is');
    assert.equal((mine.match(/r\.onFail\(chassis::carryOn\);/g) || []).length, c.route.steps.length);
    assert.ok(both.includes('{&skillsMechsRoute, Mirror::ForRobotTwo, Together::No},'));
    assert.equal(exportRegion(team, X), alone, 'and the next ordinary export leaves it out');
});

test('D1: one screen of words — what moves, mechanism by mechanism, in the order each first moves', () => {
    const c = checkRoute(route(EVERY), X);
    assert.equal(checkWords(c.route, X),
        'the lift homes, goes to low, homes, goes to 18 in; the arm goes to score; the hood goes to up; the intake runs 0.8 s at 8 V, runs 0.4 s at -6 V');
    const two = checkRoute(route([{ do: 'set', mechanism: 'hood', position: 'up' }, { do: 'set', mechanism: 'hood', position: 'down' }]), X);
    assert.equal(checkWords(two.route, X), 'the hood goes to up, then down');
});

test('D1 (review): a start form in the check runs to its end — never cut by the next command to it or the routine\'s end', () => {
    // Bug caught: startRun(intake 2 s) then set(hood) in the check: the routine ends at once, the run guard cancels the
    // intake after 0.3 s, and the robot still logs "started" — the check promised "runs 2 s at 12 V".
    for (const [kind, extra, blocking] of [['startRun', { volts: 12, forS: 2 }, 'run'], ['startMove', { preset: 'score', boundS: 1.5 }, 'move'],
                                             ['startHome', { boundS: 2 }, 'home']]) {
        const mech = kind === 'startRun' ? 'intake' : kind === 'startHome' ? 'lift' : 'arm';
        for (const after of [[], [{ do: kind, mechanism: mech, ...extra }], [{ do: 'waitUntilDone', mechanism: mech, boundS: 2 }]]) {
            const c = checkRoute(route([{ do: kind, mechanism: mech, ...extra }, ...after]), X);
            const mine = exportRegion([c.route], X);
            assert.ok(!/r\.start(Run|Move|Home)\(/.test(mine), `${kind} then ${after.map((s) => s.do)}: no start form left`);
            assert.ok(!/r\.waitUntilDone\(/.test(mine), 'and no waitUntilDone (nothing is left running for it)');
            assert.deepEqual(c.route.steps.map((s) => s.do), Array(after[0] && after[0].do === kind ? 2 : 1).fill(blocking),
                             `${kind} then ${after.map((s) => s.do)}: the blocking form, each`);
        }
    }
});
