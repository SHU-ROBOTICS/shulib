// mech_tab.test.mjs — RMP1-M: the Mechanisms tab's model. D3: each kind's TEST route (check_route.mjs mechanismTest),
// its refusals (an owed port sends nothing), its words and how the robot's own lines tick a card; D1: a card's words
// (mech_tab.mjs); D5: no "home" anywhere a person reads it.
import test from 'node:test';
import assert from 'node:assert/strict';
import { mechanismTest, testSteps, testRefusal, testerRefusal, testResults, checkWords, checkRoute, checkName, OLD_FACTS, CHECK_HOME_S, TEST_MOVE_S, TEST_RUN_S,
         TEST_RUN_V } from '../src/model/check_route.mjs';
import { checkRoute as issuesOf } from '../src/model/checks.mjs';
import { exportRegion } from '../src/model/export.mjs';
import { newWatch, readLine } from '../src/model/run_log.mjs';
import { newRoute, setStart, addStep } from '../src/model/route.mjs';
import { findWord, homeEnd } from '../src/model/lift.mjs';
import { actionsFor, checkControls } from '../src/model/controls.mjs';
import { menuActions } from '../src/model/mech_steps.mjs';
import { stepTitle, mechanismActions, homingWords } from '../src/ui/words.mjs';
import { badgeWord } from '../src/ui/mech_badges.mjs';
import { whatItIs, portWords, levelWords, routesUsing, robotWords, whenWords } from '../src/ui/mech_tab.mjs';
import { fixtureRobot } from './helpers.mjs';

const X = fixtureRobot('x');  // arm (rests at its stop), lift (switch, levels), flywheel, intake (power), hood (3 positions)
const Y = fixtureRobot('y');  // wrist (absolute), swing (stall), dr4b (rests), tilt (never), roller, catapult, grip (starts loose)
const mech = (r, id) => r.mechanisms.find((m) => m.ident === id);
const ONE = {  // shaped like robot one: a stall-homed cascade with NO levels, its ports typed; its right toggle's DIRECTION owed (robot two's today)
    ...X, id: 'one', name: 'robot one',
    mechanisms: [{ name: 'cascade', ident: 'cascade', kind: 'lift', units: 'in', homing: 'stall', homeAt: 0, range: [0, 20], unhomedVolts: 4, presets: [],
                   wiring: { motors: [{ port: 10, reversed: true }, { port: 1, reversed: false }], lines: [] } },
                 { name: 'right toggle', ident: 'rightToggle', kind: 'power', pieceSensor: false, wiring: { motors: [{ port: 9, reversed: null }], lines: [] } },
                 { name: 'claw', ident: 'claw', kind: 'pneumatic', positions: ['closed', 'open'], start: 'closed', wiring: { motors: [], lines: ['A'] } }],
};
const plain = (steps) => steps.map((s) => { const { care, ...rest } = s; return rest; });  // eslint-disable-line no-unused-vars
const HOME_WORD = /\bhom(e|es|ed|ing)\b/i;

test('D3: a switch-homed lift with levels -- finds its bottom FIRST, each level, then back to the bottom; every step bounded and carrying on', () => {
    const steps = testSteps(mech(X, 'lift'));
    assert.deepEqual(plain(steps), [
        { do: 'home', mechanism: 'lift', boundS: CHECK_HOME_S },
        { do: 'move', mechanism: 'lift', preset: 'floor', boundS: TEST_MOVE_S },
        { do: 'move', mechanism: 'lift', preset: 'low', boundS: TEST_MOVE_S },
        { do: 'move', mechanism: 'lift', preset: 'high', boundS: TEST_MOVE_S },
        { do: 'move', mechanism: 'lift', bottom: true, boundS: TEST_MOVE_S }]);
    assert.ok(steps.every((s) => s.care && s.care.onFail === 'carryOn'), 'one failure never hides the rest');
    assert.equal(steps.findIndex((s) => s.do === 'move'), 1, 'nothing moves before it has found its bottom');
});

test('D3: robot one\'s cascade (stall, NO levels) -- finds its bottom, the middle of its range, then the bottom', () => {
    const t = mechanismTest(['cascade'], ONE, []);
    assert.ok(t.route, t.why);
    assert.deepEqual(plain(t.route.steps), [{ do: 'home', mechanism: 'cascade', boundS: CHECK_HOME_S },
        { do: 'move', mechanism: 'cascade', to: 10, boundS: TEST_MOVE_S }, { do: 'move', mechanism: 'cascade', bottom: true, boundS: TEST_MOVE_S }]);
    assert.equal(t.route.name, 'cascade mechs');
    assert.equal(checkWords(t.route, ONE), 'the cascade finds its bottom, goes to 10 in, then 0 in');
    // an owed level is never a target (its number is a placeholder): the middle again
    const owed = { ...ONE.mechanisms[0], presets: [{ name: 'L1', value: 6, ledger: 0, owed: true }] };
    assert.deepEqual(plain(testSteps(owed)).map((s) => s.to ?? s.preset ?? s.do), ['home', 10, 'move']);
    // one that is not owed is
    assert.deepEqual(plain(testSteps({ ...owed, presets: [{ name: 'L1', value: 6, ledger: 0 }] })).map((s) => s.preset ?? s.to ?? s.do), ['home', 0, 'move']);
});

test('D3: homed at power-up (rests at its stop, absolute) -- no find step; never homed -- refused in words, nothing sent', () => {
    assert.deepEqual(plain(testSteps(mech(X, 'arm'))).map((s) => s.do), ['move', 'move', 'move', 'move']);
    assert.deepEqual(plain(testSteps(mech(Y, 'wrist'))).map((s) => s.preset ?? (s.bottom ? 'bottom' : s.do)), ['flat', 'bottom']);
    assert.deepEqual(plain(testSteps(mech(Y, 'swing'))).map((s) => s.do), ['home', 'move', 'move', 'move'], 'an arm homed by stall: found first');
    assert.match(testRefusal(mech(Y, 'tilt')), /never finds its zero/);
    const t = mechanismTest(['tilt'], Y, []);
    assert.equal(t.route, null);
    assert.match(t.why, /test it with its buttons in Drive/);
});

test('D3: a motor runs one way, then the other; a piston goes to each position, then back to its START', () => {
    assert.deepEqual(plain(testSteps(mech(X, 'intake'))), [{ do: 'run', mechanism: 'intake', volts: TEST_RUN_V, forS: TEST_RUN_S },
                                                         { do: 'run', mechanism: 'intake', volts: -TEST_RUN_V, forS: TEST_RUN_S }]);
    assert.deepEqual(plain(testSteps(mech(X, 'hood'))).map((s) => s.position), ['middle', 'up', 'down'], 'starts down: ends down');
    assert.deepEqual(plain(testSteps(mech(Y, 'grip'))).map((s) => s.position), ['shut', 'loose'], 'starts loose (not its first position): ends loose');
});

test('D3: an OWED port -- Test says why and sends nothing; Test every mechanism leaves it out and says so', () => {
    const owed = { ...ONE, mechanisms: ONE.mechanisms.map((m) => (m.ident === 'claw' ? { ...m, wiring: { motors: [], lines: [null] } } : m)) };
    const one = mechanismTest(['claw'], owed, []);
    assert.equal(one.route, null);
    assert.equal(one.why, 'The claw\'s port is not known yet (the builders\' answer): nothing is sent.');
    const motor = { ...ONE.mechanisms[0], wiring: { motors: [{ port: null, reversed: null }, { port: 1, reversed: false }], lines: [] } };
    assert.match(testRefusal(motor), /port is not known yet/);
    // no wiring at all: robots.json made before this tab — said as THAT, never blamed on the builders
    assert.equal(testRefusal({ ...ONE.mechanisms[0], wiring: undefined }), OLD_FACTS);
    assert.doesNotMatch(OLD_FACTS, /builders/);
    assert.equal(mechanismTest(null, { ...ONE, mechanisms: ONE.mechanisms.map(({ wiring, ...m }) => m) }, []).why, OLD_FACTS);  // eslint-disable-line no-unused-vars
    const all = mechanismTest(null, owed, []);
    assert.deepEqual(all.left.map((x) => x.name), ['claw']);
    assert.ok(!all.route.steps.some((s) => s.mechanism === 'claw'), 'the owed one is not in the test');
    assert.equal(all.route.name, 'all mechs');
    // an owed DIRECTION is not a refusal (a run both ways is the same either way round)
    assert.equal(testRefusal(ONE.mechanisms[1]), '');
});

test('D3: Test every mechanism -- every testable one, in the table\'s order; the route passes the page\'s checks and exports', () => {
    const START = { x: -36, y: -55.7, headingDeg: 0 };
    const team = addStep(setStart(newRoute(), START), { do: 'driveTo', x: -12, y: -55.7 });
    const t = mechanismTest(null, X, [team]);
    assert.deepEqual([...new Set(t.route.steps.map((s) => s.mechanism))], ['arm', 'lift', 'intake', 'hood'], 'the flywheel: no route can spin it yet');
    assert.deepEqual(t.left.map((x) => x.name), ['flywheel']);
    assert.deepEqual(t.route.start, START, 'the team\'s start (the robot does not move)');
    assert.equal(t.route.together, false);
    assert.ok(t.route.steps.every((s) => s.care.onFail === 'carryOn' && (s.do === 'run' ? s.forS > 0 : s.do === 'set' || s.boundS > 0)), 'every step bounded');
    assert.deepEqual(issuesOf(t.route, [X], X.field, { routes: [team, t.route], index: 1 }).filter((x) => x.level === 'error'), []);
    assert.doesNotThrow(() => exportRegion([team, t.route], X));
    assert.equal(exportRegion([team], X), exportRegion([team], X), 'the team\'s routes alone: the ordinary export');
    // a name the project already has is refused, in words
    const twin = mechanismTest(null, X, [team, { ...newRoute('All Mechs'), steps: [] }]);
    assert.equal(twin.route, null);
    assert.match(twin.why, /already named "all mechs"/);
    assert.match(mechanismTest(null, { ...X, mechanisms: [] }, []).why, /no mechanisms/);
});

const line = (name, body) => `[t=   1.00] [RTN] routine '${name}': ${body}`;
const ats = (body) => `[t=   1.00] [ATS] run 1: ${body}`;
function ran(test, lines) {
    const w = newWatch();
    [ats(`'${test.name}' (SKILLS, park T+57.00s, stop T+59.00s, start FIELD) -- RUN touched; 3 s countdown, the abort armed`), ...lines]
        .forEach((l) => readLine(w, l));
    return testResults(w.runs[0], test, X);
}

test('D3: the robot\'s own lines tick each card -- green every step done, red a failure in its words, amber done-but-not-as-asked; nothing before', () => {
    const t = mechanismTest(['lift', 'intake', 'hood'], X, []).route;
    // lift: 5 steps (1-5); intake 2 (6-7); hood 3 (8-10)
    const done = (n, verb, extra = '') => line(t.name, `step ${n} (${verb}) done in 0.50 s${extra}`);
    const ok = [1, 2, 3, 4, 5].map((n) => done(n, 'lift', ', at 0.0 in'));
    assert.deepEqual(ran(t, ok.slice(0, 2)), { lift: { tone: '', words: 'Testing…' }, intake: { tone: '', words: 'Testing…' }, hood: { tone: '', words: 'Testing…' } });
    const all = [...ok, done(6, 'intake'), done(7, 'intake'), done(8, 'hood', ", at 'middle'"), done(9, 'hood', ", at 'up' (UNCONFIRMED)"), done(10, 'hood', ", at 'down'")];
    const r = ran(t, [...all, ats('routine OK, 10 of 10 steps done')]);
    assert.equal(r.lift.tone, 'ok');
    assert.equal(r.intake.tone, 'ok');
    assert.deepEqual(r.hood, { tone: 'ok', words: 'Worked (UNCONFIRMED: nothing on the robot can confirm it)' });
    // a STALL on the lift: red in the robot's words; the others still green (each step carried on)
    const stalled = [...all.slice(0, 2), `[t=   2.00] [WARN][RTN] routine '${t.name}': step 3 (lift) FAILED: mechanism step FAILED (STALLED) — carrying on; drive safed`, ...all.slice(3)];
    const s = ran(t, [...stalled, ats('routine DONE, 9 of 10 steps done, 1 failed, 0 jumped')]);
    assert.deepEqual(s.lift, { tone: 'bad', words: 'Failed: mechanism step FAILED (STALLED)' });
    assert.equal(s.intake.tone, 'ok');
    // done, but with an outcome other than SUCCEEDED or UNCONFIRMED: amber
    const odd = ran(t, [...all.slice(0, 5), done(6, 'intake', ' (TIMED_OUT)'), ...all.slice(6), ats('routine OK, 10 of 10 steps done')]);
    assert.equal(odd.intake.tone, 'warn');
    // the run is over and a step never said it was done (a skip, a lost line): amber, never a tick
    const lost = ran(t, [...all.slice(0, 9), ats('routine STOPPED, 9 of 10 steps done')]);
    assert.equal(lost.hood.tone, 'warn');
    // another version of the test on the brain: said, never ticked
    const other = ran(t, [line(t.name, 'step 1 (claw) done in 0.50 s')]);
    assert.equal(other.lift.tone, 'warn');
    assert.match(other.lift.words, /another version/);
});

test('D1: a card in plain words -- what it is, its ports and which way each turns, an owed fact said as owed, its levels, the routes using it', () => {
    assert.equal(whatItIs(ONE.mechanisms[0]), 'a lift: 0 to 20 in');
    assert.equal(whatItIs(mech(X, 'arm')), 'an arm: 0 to 160 deg');
    assert.equal(whatItIs(ONE.mechanisms[1], true), 'a motor that runs while a button is held');
    assert.equal(whatItIs(ONE.mechanisms[2]), 'a piston: closed / open');
    assert.deepEqual(portWords(ONE.mechanisms[0]), ['port 10, turns reversed', 'port 1, turns forward']);
    assert.deepEqual(portWords(ONE.mechanisms[1]), ['port 9, which way it turns not known yet']);
    assert.deepEqual(portWords(ONE.mechanisms[2]), ['3-wire port A']);
    assert.deepEqual(portWords(mech(X, 'lift')), ['port 2, turns forward', 'port 3, turns reversed', 'bottom switch: 3-wire port B']);
    assert.deepEqual(portWords(mech(X, 'intake')), ['port 5, turns forward', 'piece sensor: port 9']);
    assert.deepEqual(portWords({ ...ONE.mechanisms[2], wiring: { motors: [], lines: [null] } }), ['3-wire port not known yet: the builders\' answer']);
    assert.deepEqual(portWords({ ...ONE.mechanisms[0], wiring: { motors: [{ port: null, reversed: null }], lines: [] } }), ['port not known yet: the builders\' answer']);
    // every port of every fixture mechanism is its own (a card never shows another mechanism's port)
    for (const r of [X, Y]) {
        for (const m of r.mechanisms) {
            const want = [...m.wiring.motors.map((x) => `port ${x.port},`), ...m.wiring.lines.map((l) => `3-wire port ${l}`)];
            const got = portWords(m);
            want.forEach((w, k) => assert.ok(got[k].startsWith(w), `${r.id} ${m.name}: ${got[k]} is not ${w}`));
        }
    }
    assert.deepEqual(levelWords(ONE.mechanisms[0]), ['no levels yet']);
    assert.deepEqual(levelWords(mech(X, 'lift')), ['floor 0 in', 'low 8 in', 'high 22 in']);
    assert.deepEqual(levelWords({ ...ONE.mechanisms[0], presets: [{ name: 'L1', value: 6, ledger: 0, owed: true }] }), ['L1 not measured yet']);
    assert.equal(levelWords(ONE.mechanisms[1]), null);
    const routes = [{ name: 'skills', steps: [{ do: 'driveTo' }, { do: 'set', mechanism: 'claw' }] }, { name: 'left', steps: [{ do: 'wait' }] },
                    { name: 'right', steps: [{ do: 'run', mechanism: 'rightToggle' }, { do: 'set', mechanism: 'claw' }] }];
    assert.deepEqual(routesUsing(routes, 'claw'), [{ k: 0, name: 'skills', step: 1 }, { k: 2, name: 'right', step: 1 }]);
    assert.deepEqual(routesUsing(routes, 'cascade'), []);
    assert.deepEqual(['one', 'two'].map(robotWords), ['robot one', 'robot two']);
});

test('D5: "find the bottom" (or the top) wherever a person reads it -- never "home"', () => {
    const top = { ...ONE.mechanisms[0], homeAt: 20 };
    assert.deepEqual([homeEnd(ONE.mechanisms[0]), homeEnd(top), homeEnd({ ...top, homeAt: 5 }), homeEnd(undefined)], ['bottom', 'top', 'zero', 'zero']);
    assert.deepEqual([findWord(ONE.mechanisms[0]), findWord(top, true), findWord(null)], ['find the bottom', 'Find the top', 'find its zero']);
    // every word the page builds for every mechanism of every robot: the menu, the step list, the badges, the checks, the controls
    const said = [];
    for (const r of [X, Y, ONE]) {
        for (const m of r.mechanisms) {
            said.push(homingWords(m), ...mechanismActions(m).map((a) => a.words), ...actionsFor(m).map((a) => a.words));
            said.push(...menuActions(m, mechanismActions(m), newRoute(), 0).map((a) => a.words));
            for (const d of ['home', 'startHome']) {
                said.push(...stepTitle({ do: d, mechanism: m.ident, boundS: 2 }, r), badgeWord({ do: d, mechanism: m.ident }, m));
            }
        }
        const t = mechanismTest(null, r, []);
        if (t.route) {
            said.push(checkWords(t.route, r));
        }
    }
    // the checks' words on a route that never finds the bottom, finds it while moving, or fails to
    const START = { x: -36, y: -55.7, headingDeg: 0 };
    const route = (steps) => steps.reduce((r, s) => addStep(r, s), setStart(newRoute(), START));
    for (const steps of [[{ do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }],
        [{ do: 'startHome', mechanism: 'lift', boundS: 2 }, { do: 'move', mechanism: 'lift', preset: 'low', boundS: 2 }],
        [{ do: 'home', mechanism: 'lift', boundS: 2, care: { onFail: 'carryOn' } }, { do: 'move', mechanism: 'lift', up: 2, boundS: 2 }],
        [{ do: 'move', mechanism: 'lift', up: 2, boundS: 2 }]]) {
        said.push(...issuesOf(route(steps), [X], X.field, {}).map((x) => x.words));
    }
    said.push(...issuesOf(route([{ do: 'home', mechanism: 'tilt', boundS: 2 }, { do: 'move', mechanism: 'tilt', preset: 'mid', boundS: 2 }]), [Y], Y.field, {}).map((x) => x.words));
    said.push(...checkControls([{ mechanism: 'lift', form: 'preset', preset: 'low', button: 'R1', timeLimitS: 2 }], X).map((x) => x.words));
    said.push(...checkControls([{ mechanism: 'tilt', form: 'homeOnPress', button: 'R1', timeLimitS: 2 }], Y).map((x) => x.words));
    const bad = said.filter((w) => HOME_WORD.test(w.replace(/NOT_HOMED/g, '')));
    assert.deepEqual(bad, [], 'a person reads "home" here');
    assert.ok(said.some((w) => /find the bottom|finds its bottom/i.test(w)), 'and "find the bottom" is said');
});

test('D3: an owed SENSOR port (bottom switch, angle sensor, piece sensor) refuses too -- the placeholder would be read', () => {
    const lift = mech(X, 'lift');
    assert.equal(testRefusal(lift), '');
    assert.match(testRefusal({ ...lift, wiring: { ...lift.wiring, homeSwitch: null } }), /lift's port is not known yet/);
    const wrist = mech(Y, 'wrist');
    assert.match(testRefusal({ ...wrist, wiring: { ...wrist.wiring, angleSensor: null } }), /port is not known yet/);
    const intake = mech(X, 'intake');
    assert.match(testRefusal({ ...intake, wiring: { ...intake.wiring, pieceSensor: null } }), /port is not known yet/);
    assert.deepEqual(portWords({ ...intake, wiring: { ...intake.wiring, pieceSensor: null } }), ['port 5, turns forward', 'piece sensor: not known yet']);
});

test('D3: a never-homed ARM is refused like a never-homed lift', () => {
    const arm = { ...mech(Y, 'swing'), homing: 'never' };
    assert.match(testRefusal(arm), /never finds its zero/);
    assert.equal(mechanismTest(['swing'], { ...Y, mechanisms: [arm] }, []).route, null);
});

test('D3: a robot whose Auton Tester does not build yet (a Stage-B fact owed) -- every Test says why and sends nothing', () => {
    const two = { ...ONE, id: 'two', testerOwed: ['the right side toggle\'s run direction', 'the claw\'s line level that OPENS it'] };
    assert.equal(testerRefusal(ONE), '');
    assert.equal(testerRefusal({ ...ONE, testerOwed: [] }), '');
    const why = testerRefusal(two);
    assert.match(why, /Auton Tester does not build until 2 facts are typed .*the right side toggle's run direction\. Nothing is sent\./);
    for (const idents of [['cascade'], ['claw'], null]) {
        const t = mechanismTest(idents, two, []);
        assert.equal(t.route, null, `${idents}: refused`);
        assert.equal(t.why, why);
    }
    // a piston whose levels are owed: its card says which position is which is not known
    assert.deepEqual(portWords({ ...ONE.mechanisms[2], start: null, wiring: { motors: [], lines: ['A'], levels: null } }),
                     ['3-wire port A', 'which position is which not known yet']);
    assert.deepEqual(plain(testSteps({ ...ONE.mechanisms[2], start: null })).map((s) => s.position), ['open', 'closed'], 'no start: back to its first');
});

test('D3: a test and a route\'s check never share a name on the brain -- either way round, refused in words', () => {
    const named = (n) => ({ ...newRoute(n), steps: [{ do: 'set', mechanism: 'claw', position: 'open' }] });
    for (const [route, test] of [['cascade', ['cascade']], ['all', null], ['Claw', ['claw']]]) {
        const r = named(route);
        assert.equal(checkName(r, ONE).toLowerCase(), mechanismTest(test, ONE, []).route.name.toLowerCase(), `${route}: the same name`);
        const t = mechanismTest(test, ONE, [r]);
        assert.equal(t.route, null, `a test beside route "${route}"`);
        assert.match(t.why, new RegExp(`check of route "${route}" is also named`));
        const c = checkRoute(r, ONE, [r]);
        assert.equal(c.route, null, `route "${route}"'s check`);
        assert.match(c.why, /like a test on the Mechanisms tab/);
    }
    assert.ok(checkRoute(named('skills'), ONE).route, 'any other name: checked as before');
    assert.ok(mechanismTest(['cascade'], ONE, [named('skills')]).route);
});

test('D3: a step the robot STOPPED turns its card red in the robot\'s words; the ones after it, never said, amber', () => {
    const t = mechanismTest(['lift', 'intake', 'hood'], X, []).route;
    const done = (n, verb) => line(t.name, `step ${n} (${verb}) done in 0.50 s`);
    const s = ran(t, [done(1, 'lift'), done(2, 'lift'),
        `[t=   2.00] [WARN][RTN] routine '${t.name}' STOPPED at step 3 (lift): mechanism step FAILED (STALLED) — skipping the rest; drive safed`,
        ats('routine STOPPED, 2 of 10 steps done, stopped at lift (mechanism step FAILED (STALLED))')]);
    assert.deepEqual(s.lift, { tone: 'bad', words: 'Stopped: mechanism step FAILED (STALLED)' });
    assert.equal(s.intake.tone, 'warn');
    assert.equal(s.hood.tone, 'warn');
});

test('D1: a result from another day says its day, not only its time', () => {
    const at = new Date(2026, 10, 19, 16, 59).getTime();
    const time = new Date(at).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' });
    assert.equal(whenWords(at, new Date(2026, 10, 19, 23, 0).getTime()), time, 'today: the time');
    const other = whenWords(at, new Date(2026, 10, 21, 8, 0).getTime());
    assert.ok(other.endsWith(`, ${time}`) && other !== time && /19/.test(other), other);
});

test('D1: a switch is named for the end it finds -- "top switch" on a lift that finds its top', () => {
    assert.match(portWords(mech(X, 'lift')).at(-1), /^bottom switch: 3-wire port B$/);
    assert.match(portWords({ ...mech(X, 'lift'), homeAt: 24 }).at(-1), /^top switch: 3-wire port B$/);
});
