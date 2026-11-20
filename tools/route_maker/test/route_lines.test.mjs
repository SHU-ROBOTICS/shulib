// route_lines.test.mjs — THE LINES the preview engine runs (route_lines.mjs; RMP1-A2e, the RMP1 brief A.2 [W7, W8]).
// NF1's other half: every number the lines carry is the export's own text, character for character, step by step, on
// every golden — so PREVIEW runs the numbers the C++ prints. And the names: a mechanism by its declaration's `.name`, a
// ledger level by its preset's name, every name one word.
import test from 'node:test';
import assert from 'node:assert/strict';
import { goldens } from '../goldens.mjs';
import { loadProject } from '../src/model/project.mjs';
import { addStep, newRoute, setStart } from '../src/model/route.mjs';
import { enc, routeLines } from '../src/model/route_lines.mjs';
import { fixtureRobot, ledgerRobot } from './helpers.mjs';

const NUM = /^-?\d+(\.\d+)?$/;

/** Every number word of one line: `key=N`, and each part of a curve point `p=x,y[,h]`. */
function lineNumbers(line) {
    const out = [];
    for (const w of line.split(' ').slice(1)) {
        const [k, v] = [w.slice(0, w.indexOf('=')), w.slice(w.indexOf('=') + 1)];
        for (const part of k === 'p' ? v.split(',') : [v]) {
            if (NUM.test(part) && !['face'].includes(k)) {
                out.push(part);
            }
        }
    }
    return out;
}

/** Every number literal in a piece of the export's CODE (comments dropped; the caps' fixed `1_s` and `180.0` and a
 *  level index are the spelling, not the route's numbers). */
function codeNumbers(code) {
    const c = code.split('\n').map((l) => l.replace(/\/\/.*$/, '')).join('\n')
        .replace(/ \/ 1_s/g, '').replace(/ \/ 180\.0/g, '').replace(/facts::lv\(\d+\)/g, '').replace(/"[^"]*"/g, '');
    // ... and an arm's `.to(<degrees>)`, a plain number (the library refuses `_deg` there, RMP1-C D7)
    return [...c.matchAll(/(?<![A-Za-z_\d.])(-?\d+(?:\.\d+)?)(?=_(?:in|deg|s|volt)\b| \* math::Angle::kPi|\)(?=, ))/g)].map((m) => m[1]);
}

const sorted = (a) => [...new Set(a)].sort();

test('NF1 (the lines): on every golden, each step\'s numbers are the export\'s own text, character for character', () => {
    for (const [name0, project, cpp0] of goldens()) {
        const all = loadProject(project).project.routes;
        const routeBlocks = cpp0.split('chassis::RoutineResult ').slice(1);  // RMP1-C: one block per route, in order
        assert.equal(routeBlocks.length, all.length, `${name0}: one steps function per route`);
        all.forEach((route, k) => {
            const name = `${name0} route ${k + 1}`;
            const cpp = routeBlocks[k].split('\n}  // namespace')[0];
            const robot = route.robot === 'x' ? fixtureRobot('x') : { id: route.robot, mechanisms: route.steps
                .filter((s) => s.mechanism).map((s) => ({ ident: s.mechanism, name: s.mechanism, presets: [] })) };
            const lines = routeLines(route, robot).trimEnd().split('\n');
            const steps = lines.filter((l) => l.startsWith('step '));
            const blocks = cpp.split(/\n(?=    \/\/ Step \d+ — )/).slice(1);
            assert.equal(steps.length, route.steps.length, name);
            assert.equal(blocks.length, steps.length, `${name}: one export block per step`);
            steps.forEach((l, i) => {
                const code = blocks[i].split(/\n\n/)[0];
                assert.deepEqual(sorted(lineNumbers(l)), sorted(codeNumbers(code)), `${name} step ${i + 1}: ${l}`);
            });
            const entry = cpp.slice(cpp.indexOf('competition::AutonEntry '));
            assert.deepEqual(sorted([...lineNumbers(lines[1]), ...lineNumbers(lines[2])]), sorted(codeNumbers(entry)),
                             `${name}: the start and the schedule`);
        });
    }
});

test('the lines name a mechanism by its declaration\'s .name, a ledger level by its preset\'s name, every name one word', () => {
    const x = fixtureRobot('x');
    const two = { id: 'two', name: 'robot two', mechanisms: [{ ident: 'clawPivot6', name: 'claw pivot 6', kind: 'power' }] };
    let r = setStart({ ...newRoute(), robot: 'two' }, { x: -36, y: -55.7, headingDeg: 0 });
    r = addStep(r, { do: 'run', mechanism: 'clawPivot6', volts: 6, forS: 0.5, label: 'grab it', care: { onFail: { skipTo: 'grab it' } } });
    assert.match(routeLines(r, two), /^step do=run mech=claw%20pivot%206 volts=6 for=0\.5 label=grab%20it skipTo=grab%20it$/m);
    const one = ledgerRobot();
    const lv = addStep(setStart(newRoute(), { x: 0, y: 0, headingDeg: 0 }), { do: 'move', mechanism: 'cascade', preset: 2, boundS: 2 });
    assert.match(routeLines(lv, one), /^step do=move mech=cascade preset=L2 bound=2$/m);
    assert.throws(() => routeLines(addStep(lv, { do: 'move', mechanism: 'cascade', preset: 5, boundS: 2 }), one), /no level 5/);
    assert.throws(() => routeLines(addStep(lv, { do: 'home', mechanism: 'cascade', boundS: 2 }), x), /no mechanism "cascade"/);
    assert.throws(() => routeLines(addStep(lv, { do: 'wallReset' }), one), /cannot run a "wallReset"/);
    assert.equal(enc('a b%c"d—'), 'a%20b%25c%22d%E2%80%94', 'UTF-8 bytes, %XX');
    assert.ok(routeLines(newRoute(), x).includes('\nstart relative\n'), 'no start: relative, as the empty region');
});

test('a mechanism step\'s motion options are never written (the export writes none), a motion\'s in MotionOptions\' order', () => {
    const x = fixtureRobot('x');
    let r = setStart({ ...newRoute(), robot: 'x' }, { x: 0, y: 0, headingDeg: 0 });
    r = addStep(r, { do: 'home', mechanism: 'lift', boundS: 2, care: { timeoutS: 3, onFail: 'carryOn' } });
    r = addStep(r, { do: 'turnTo', headingDeg: 90, care: { toleranceDeg: 2, maxTurnDegPerS: 90, timeoutS: 1.5, maxSpeedInPerS: 0 } });
    const steps = routeLines(r, x).split('\n').filter((l) => l.startsWith('step '));
    assert.equal(steps[0], 'step do=home mech=lift bound=2 fail=carryOn');
    assert.equal(steps[1], 'step do=turnTo h=90 timeout=1.5 turn=90 tolDeg=2', '0 = the robot\'s own: not written');
});
