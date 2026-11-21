// controls.test.mjs — THE CONTROLS TAB'S MODEL (RMP1-B; the brief's CT agreement, CT21–CT23, CT-HS's node half).
// Each test tries to break one rule; the C++ half of the agreement is the compile-refusal harness's rm_ct_*.cpp.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import { BUTTONS, checkControls, exportControls, exportControlWords, parseControls, parseControlWords, toSaved, fromSaved,
         addControl, editControl, followDriving, checkMap, removeFreeLine, controlCode, checkWords, pageWords, defaultWords,
         REZERO_REFUSAL, FEEL_REFUSAL, keyButtons } from '../src/model/controls.mjs';
import { readDriverFile, exportRegions, spliceRegions, findRegions, regionOf, REGIONS, markers } from '../src/model/driver_file.mjs';
import { regionHash } from '../src/model/region.mjs';
import { checkFeel } from '../src/model/feel.mjs';
import { cases, goldens as ctGoldens, ROBOTS, stateOf } from '../controls_cases.mjs';
import { refusalTUs, controlsGoldens, REFUSALS_DIR } from '../goldens.mjs';
import { GOLDENS, REPO } from './helpers.mjs';
import { CONTROLS_HEADING as CONTROLS_HEADING_FOR_TEST } from '../src/model/controls.mjs';

const read = (p) => fs.readFileSync(path.join(REPO, p), 'utf8');
const marked = (id) => fs.readFileSync(path.join(GOLDENS, `driver_${id}_marked.hpp`), 'utf8');
const errors = (k) => {
    const r = ROBOTS()[k.robot];
    const out = checkControls(k.list, r, { rezero: k.driving.rezeroButton })
        .map((x) => x.refusal);
    if (checkFeel(k.driving).some((x) => x.level === 'error')) {
        out.push(FEEL_REFUSAL);
    }
    return out;
};

test('CT agreement: every configuration the page refuses, it refuses with the build\'s OWN name -- and only that one', () => {
    const list = cases();
    assert.ok(list.length >= 38, `${list.length} cases`);
    for (const k of list) {
        const got = errors(k);
        assert.ok(got.includes(k.expect), `${k.name}: the page says ${JSON.stringify(got)}, the build ${k.expect}`);
        if (k.name !== 'more_than_24') {
            assert.deepEqual([...new Set(got)], [k.expect], `${k.name}: one refusal, the one the build names`);
        }
    }
});

test('CT agreement: every refusal the build can raise on a driver file has a case (a new library refusal with no page rule turns this red)', () => {
    const inputs = read('include/shulib/mechanism/driver_inputs.hpp');
    const block = inputs.slice(inputs.indexOf('namespace refused {'), inputs.indexOf('}  // namespace refused'));
    const names = [...block.matchAll(/inline void (\w+)\(\)/g)].map((m) => m[1]);
    assert.equal(names.length, 30, 'driver_inputs.hpp\'s refused:: names (29 + the one-button hold, ROB1-C2)');
    const covered = new Set(cases().map((k) => k.expect));
    // the 13th button: a project file edited by hand (the picture has twelve); its C++ twin is MECH-A4's TU
    const z = checkControls([{ form: 'toggle', mechanism: 'hood', button: 'Z' }], ROBOTS().x).map((x) => x.refusal);
    assert.ok(z.includes('a_control_names_a_button_that_is_not_one_of_the_twelve'));
    assert.match(read('test/compile_refusals/d_button_not_one_of_the_twelve.cpp'), /^\/\/ expect: a_control_names_a_button_that_is_not_one_of_the_twelve$/m);
    covered.add('a_control_names_a_button_that_is_not_one_of_the_twelve');
    for (const n of names) {
        assert.ok(covered.has(n), `refused::${n} has no case`);
    }
    // ...and the targets' own refusals (kinds.hpp's operator[], raised inside a control's argument)
    for (const n of ['no_such_preset_on_this_mechanism', 'no_such_position_on_this_mechanism', 'a_power_mechanism_has_no_presets_run_it_with_r_run']) {
        assert.ok(covered.has(n), `kinds.hpp's ${n} has no case`);
    }
    // ...and each name the page says is one the headers really raise
    const kinds = read('include/shulib/mechanism/kinds.hpp');
    const file = read('src/common/driver_file.hpp');
    for (const n of covered) {
        assert.ok(names.includes(n) || kinds.includes(`void ${n}()`) || file.includes(n), `"${n}" is not a refusal of the library`);
    }
    for (const b of BUTTONS) {
        assert.ok(covered.has(`button_${b}_does_two_jobs`), b);
    }
    assert.ok(file.includes(`"${REZERO_REFUSAL}`) && file.includes(`"${FEEL_REFUSAL}`), 'DriverFile\'s two static_assert sentences');
});

test('CT agreement: the generated compile-refusal TUs are fresh, one per case, each expecting the page\'s refusal (no orphan)', () => {
    const tus = refusalTUs();
    const onDisk = fs.readdirSync(REFUSALS_DIR).filter((n) => n.startsWith('rm_ct_')).sort();
    assert.deepEqual(onDisk, tus.map(([n]) => n).sort(), 'node tools/route_maker/goldens.mjs --write, then read the diff');
    for (const [name, text] of tus) {
        assert.equal(fs.readFileSync(path.join(REFUSALS_DIR, name), 'utf8').replace(/\r\n/g, '\n'), text, `${name} is stale`);
    }
    for (const k of cases()) {
        const tu = fs.readFileSync(path.join(REFUSALS_DIR, `rm_ct_${k.name}.cpp`), 'utf8').replace(/\r\n/g, '\n');  // CRLF on Windows
        assert.ok(tu.endsWith(`// expect: ${k.expect}\n`), k.name);
        // the harness reads the reason from a diagnostic's MESSAGE: a namespace or file named after it would fool a
        // weaker judge, so no TU's own code may carry its expected words
        assert.ok(!tu.split('// expect:')[0].replace(/^\/\/.*$/gm, '').includes(k.expect), `${k.name} names its own reason in code`);
    }
});

test('CT21: the configurations the page passes are clean in the page, and their compiled goldens are fresh', () => {
    for (const g of ctGoldens()) {
        assert.deepEqual(checkControls(g.list, ROBOTS()[g.robot], { rezero: g.driving.rezeroButton }), [], g.name);
        assert.deepEqual(checkFeel(g.driving).filter((x) => x.level === 'error'), [], g.name);
    }
    for (const [name, text] of controlsGoldens()) {
        assert.equal(fs.readFileSync(path.join(GOLDENS, name), 'utf8'), text, `${name} is stale: goldens.mjs --write`);
    }
});

test('CT22: robot one\'s map (the builder\'s, main 3c29b73) round-trips BYTE-IDENTICAL through the page -- all three regions, and robot two\'s', () => {
    for (const id of ['one', 'two']) {
        const text = marked(id);
        const r = readDriverFile(text, id);
        assert.ok(r.ok, r.words);
        assert.deepEqual(r.problems, [], id);
        assert.ok(r.marked, `${id}: the fixture carries all three marker pairs`);
        const out = spliceRegions(text, id, exportRegions(r.state, { id }), { base: r.hashes });
        assert.ok(out.ok, out.words);
        assert.equal(out.changed, false, `${id}: the page wrote back exactly the file`);
        assert.equal(out.text, text);
        // ...and through the project file's shape (save -> open) too
        const back = fromSaved(JSON.parse(JSON.stringify(toSaved(r.state.controls))));
        assert.ok(back.ok, back.words);
        assert.deepEqual(exportRegions({ ...r.state, controls: back.state }, { id }), exportRegions(r.state, { id }));
    }
    const one = readDriverFile(marked('one'), 'one').state;
    assert.deepEqual(one.controls.list.map((c) => [c.form, c.mechanism, c.button, c.back, c.volts, c.timeLimitS]), [
        ['whileHeld', 'cascade', 'R1', 'R2', 12, undefined], ['toggle', 'claw', 'A', undefined, undefined, undefined],
        ['toggle', 'flipper', 'B', undefined, undefined, undefined], ['toggle', 'ramp', 'X', undefined, undefined, undefined],
        ['whileHeld', 'rightToggle', 'L1', undefined, 12, undefined], ['whileHeld', 'leftToggle', 'L2', undefined, 12, undefined],
        ['homeAtDriverStart', 'cascade', undefined, undefined, undefined, 3]]);
    assert.equal(one.controls.list[6].words, '[delete-able] home it at the first driver start', 'the one marked line keeps its mark');
});

test('CT23: in the REAL driver files of this tree, the header\'s button map has each control\'s rows where the code puts it (its WORDS are the team\'s: read back, not judged)', () => {
    for (const id of ['one', 'two']) {
        const p = `src/team/robot_${id}_driver.hpp`;
        const text = read(p);
        const r = readDriverFile(text, id);
        assert.ok(r.ok, r.words);
        assert.deepEqual(r.problems.filter((x) => !/markers/.test(x)), [], `${p}: ${r.problems.join('; ')}`);
        const f = findRegions(text, id);
        assert.equal(exportControlWords(r.state.controls).join('\n'), regionOf(f, 'control words'), `${p}: the map and the controls disagree`);
    }
});

test('CT23 can fail: a button swapped in the code but not in the map, and a map line left from an old map, are refused by name', () => {
    const text = marked('one');
    const swapped = text.replace('toggle(claw, Button::A)', 'toggle(claw, Button::Up)');
    const r = readDriverFile(swapped, 'one');
    assert.ok(r.ok);
    assert.ok(r.problems.some((x) => /button map/.test(x)), 'the map still says A is the claw');
    // A and X swapped in the code only: the map's order no longer follows the code
    const xa = text.replace('toggle(claw, Button::A)', 'toggle(claw, Button::Q)').replace('toggle(ramp, Button::X)', 'toggle(ramp, Button::A)').replace('toggle(claw, Button::Q)', 'toggle(claw, Button::X)');
    assert.ok(readDriverFile(xa, 'one').problems.some((x) => /button map/.test(x)));
    // the d-pad said FREE, and a preset now takes Up: the map lies
    const st = readDriverFile(text, 'one').state.controls;
    const withUp = addControl(st, { form: 'preset', mechanism: 'claw', preset: 'open', button: 'Up' }, ROBOTS().x);
    assert.equal(checkMap(withUp)[0].code, 'mapStale');
    assert.deepEqual(checkMap(removeFreeLine(withUp, 0)), []);
});

test('writes: only between the three marker pairs, all or none; the file\'s own CRLF kept; a hand edit refused until Overwrite', () => {
    const id = 'one';
    const lf = marked(id);
    const crlf = lf.replace(/\n/g, '\r\n');
    const r = readDriverFile(crlf, id);
    assert.ok(r.ok);
    const changed = { ...r.state, driving: { ...r.state.driving, forward: { curve: 0.6 } } };
    const regions = exportRegions(changed, { id });
    const out = spliceRegions(crlf, id, regions, { base: r.hashes });
    assert.ok(out.ok, out.words);
    assert.ok(out.changed);
    assert.ok(!/[^\r]\n/.test(out.text), 'no bare LF in a CRLF file');
    const before = crlf.split('\r\n');
    const after = out.text.split('\r\n');
    const f = findRegions(crlf, id);
    const outside = (lines, ff) => lines.filter((_, i) => !REGIONS.some((rg) => i >= ff.found[rg].from && i < ff.found[rg].to));
    assert.deepEqual(outside(after, findRegions(out.text, id)), outside(before, f), 'every line outside the regions unchanged');
    assert.match(out.text, /\.forward = \{\.expo = 0\.6\}/);
    // a hand edit in ONE region refuses the WHOLE write (nothing half-written)
    const edited = crlf.replace('// field-centric\'s "away from me"', '// mine');
    const refused = spliceRegions(edited, id, regions, { base: r.hashes });
    assert.equal(refused.why, 'handEdit');
    assert.deepEqual(refused.regions, ['feel']);
    assert.ok(spliceRegions(edited, id, regions, { base: r.hashes, overwrite: true }).ok);
    // TODAY's file has no `control words` / `feel` markers: read, never written
    const today = read('src/team/robot_one_driver.hpp');
    if (!today.includes(markers(id)['control words'][0])) {
        const t = spliceRegions(today, id, regions, { base: {} });
        assert.equal(t.why, 'noMarkers');
        assert.deepEqual(t.regions, ['control words', 'feel']);
    }
    assert.equal(spliceRegions('int x;\n', id, regions, { base: {} }).words, 'src/team/robot_one_driver.hpp has no "controls" or '
        + '"control words" or "feel" markers yet, so the route builder will not write into it (it gets them with Black Bot\'s update). '
        + 'Until then use Copy.');  // RMP1-L: the robot named
    // a marker twice is not a region
    const twice = lf.replace('// [feel one]', '// [feel one]\n// [feel one]');
    assert.equal(spliceRegions(twice, id, regions, { base: r.hashes }).why, 'noMarkers');
    // the hashes it returns are the regions' (LF), so a CRLF checkout is not taken for a hand edit next time
    assert.equal(out.hashes.feel, regionHash(regions.feel.join('\n')));
});

test('a mixed-line-ending file: the lines outside the regions keep their OWN endings', () => {
    const lf = marked('one');
    const lines = lf.split('\n');
    const mixed = lines.map((l, i) => (i < 5 ? `${l}\r` : l)).join('\n');  // the first lines CRLF, the rest LF
    const r = readDriverFile(mixed, 'one');
    const out = spliceRegions(mixed, 'one', exportRegions(r.state, { id: 'one' }), { base: r.hashes });
    assert.ok(out.ok, out.words);
    const f0 = findRegions(mixed, 'one');
    const f1 = findRegions(out.text, 'one');
    const outside = (ff) => ff.raw.filter((_, i) => !REGIONS.some((rg) => i >= ff.found[rg].from && i < ff.found[rg].to));
    assert.deepEqual(outside(f1), outside(f0), 'byte for byte, each with its own ending');
    // the regions take the FILE's line ending (its first line's: CRLF here), as region.mjs does for the route
    assert.ok(f1.raw.slice(f1.found.feel.from, f1.found.feel.to).every((l) => l.endsWith('\r')));
});

test('edits: default words follow a change, the student\'s own are kept and RE-KEYED to the new buttons', () => {
    const x = ROBOTS().x;
    let st = { list: [], table: 'kRobotXMechanisms' };
    st = addControl(st, { form: 'whileHeld', mechanism: 'arm', button: 'R1', back: 'R2', volts: 6 }, x);
    assert.equal(st.list[0].words, 'R1 up / R2 down while held; let go: it holds');
    assert.deepEqual(st.list[0].banner.map((r) => r.key), ['R1', 'R2']);
    st = editControl(st, 0, { button: 'L1', back: 'L2' }, x);
    assert.equal(st.list[0].words, 'L1 up / L2 down while held; let go: it holds', 'a default comment follows');
    assert.deepEqual(st.list[0].banner.map((r) => r.key), ['L1', 'L2']);
    st = editControl(st, 0, { banner: [{ key: 'L1 / L2', lines: ['the arm, our words'] }] }, x);
    st = editControl(st, 0, { button: 'Up', back: 'Down' }, x);
    assert.deepEqual(st.list[0].banner, [{ key: 'Up / Down', lines: ['the arm, our words'] }], 'own words kept, key follows');
    st = editControl(st, 0, { form: 'preset', preset: 'score', back: undefined, volts: undefined, timeLimitS: 1 }, x);
    assert.equal(st.list[0].banner[0].key, 'Up');
    assert.match(st.list[0].banner[0].lines[0], /goes to "score"/, 'a new job gets the page\'s words again');
    const words = exportControlWords(followDriving(st, null, { mode: 'tankSticks', rezeroButton: 'Y', onRelease: 'brake' }));
    assert.ok(words.some((l) => /the LEFT side/.test(l)) && words.some((l) => /drive brakes/.test(l)));
});

test('G4: a robot whose driver file cannot be read starts with NO controls -- never an invented map', () => {
    const r = readDriverFile('// nothing here\n', 'one');
    assert.equal(r.ok, false);
    const none = checkControls([], ROBOTS().x).map((x) => x.refusal);
    assert.equal(none.length, ROBOTS().x.mechanisms.length);
    assert.ok(none.every((n) => n === 'a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant'));
});

test('the project file\'s shape (G3a\'s per-kind lists, D14): exact back, and a field it does not write refused by name', () => {
    const st = stateOf(cases()[0]).controls;
    const saved = toSaved(st);
    assert.deepEqual(Object.keys(saved).slice(0, 6), ['whileHeld', 'toggles', 'presets', 'homeOnPress', 'homeAtDriverStart', 'autonOnly']);
    assert.deepEqual(fromSaved(saved).state.list, st.list);
    const odd = JSON.parse(JSON.stringify(saved));
    odd.toggles[0].slowMode = true;
    assert.match(fromSaved(odd).words, /field "slowMode"/);
    const holes = JSON.parse(JSON.stringify(saved));
    holes.whileHeld[0].at = 99;
    assert.equal(fromSaved(holes).ok, false);
});

test('reading the controls refuses what it would have to guess (a constant target, a second statement)', () => {
    assert.equal(parseControls(['inline constexpr DriverControls kControls = controls(kRobotXMechanisms,', '    preset(arm.to(95), Button::Up, 1_s));']).ok, false);
    assert.equal(parseControls(['int x;']).ok, false);
    const ok = parseControls(['inline constexpr DriverControls kControls = controls(kRobotXMechanisms);']);
    assert.ok(ok.ok);
    assert.deepEqual(exportControls(ok.state, ROBOTS().x), ['// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════'.slice(0, 0) || [], 'inline constexpr DriverControls kControls = controls(kRobotXMechanisms);'].flat());
    assert.equal(parseControlWords(['//   left stick   up / down: drive', '//   Q            nothing'], { list: [] }).ok, true);
});

test('a feel or map the page cannot read is NEVER overwritten silently: no base for it, so a write asks first (the review\'s finding)', () => {
    const text = marked('one');
    const odd = text.replace('inline constexpr DriverProfile kFeel{.mode = ArcadeBody};', 'inline constexpr DriverProfile kFeel{.yaw = {.expo = 0.4}, .mode = ArcadeBody, };');
    assert.notEqual(odd, text);
    const r = readDriverFile(odd, 'one');
    assert.ok(r.ok);
    assert.ok(r.problems.some((x) => /THE FEEL/.test(x)), 'the feel is said to be unread');
    assert.equal(r.hashes.feel, undefined);
    assert.ok(r.hashes.controls && r.hashes['control words'], 'the parts it did read keep their base');
    const out = spliceRegions(odd, 'one', exportRegions(r.state, { id: 'one' }), { base: r.hashes });
    assert.equal(out.ok, false);
    assert.equal(out.why, 'handEdit');
    assert.deepEqual(out.regions, ['feel']);
    assert.ok(spliceRegions(odd, 'one', exportRegions(r.state, { id: 'one' }), { base: r.hashes, overwrite: true }).ok);
});

test('a control moved to ANOTHER mechanism drops the old one\'s target: a flywheel toggle given to a piston writes toggle(grip, ...), never grip["shoot"] (the review\'s finding)', () => {
    const fw = ROBOTS().x.mechanisms.find((m) => m.ident === 'flywheel');
    const grip = ROBOTS().y.mechanisms.find((m) => m.ident === 'grip');
    const robot = { id: 'q', mechanisms: [fw, grip] };
    let st = addControl({ list: [] }, { form: 'toggle', mechanism: 'flywheel', speed: 'shoot', button: 'X' }, robot);
    assert.equal(controlCode(st.list[0], fw), 'toggle(flywheel["shoot"], Button::X)');
    st = editControl(st, 0, { mechanism: 'grip' }, robot);
    assert.equal(st.list[0].speed, undefined);
    assert.equal(controlCode(st.list[0], grip), 'toggle(grip, Button::X)');
    // ...and a form the new mechanism cannot take becomes one it can (a piston cannot be held to run)
    st = addControl({ list: [] }, { form: 'whileHeld', mechanism: 'flywheel', button: 'R1', back: 'R2', volts: 12 }, robot);
    st = editControl(st, 0, { mechanism: 'grip' }, robot);
    assert.equal(st.list[0].form, 'toggle');
    assert.equal(st.list[0].back, undefined);
});

test('the map\'s FREE line is wrong once the RE-ZERO moves onto it, as once a control does (the review\'s finding)', () => {
    const r = readDriverFile(marked('one'), 'one');
    assert.deepEqual(checkMap(r.state.controls, r.state.driving.rezeroButton), []);
    const stale = checkMap(r.state.controls, 'Up');
    assert.equal(stale.length, 1);
    assert.equal(stale[0].code, 'mapStale');
    assert.equal(stale[0].button, 'Up');
});

test('a level by the ledger reads back at any index, 10 and up included (the review\'s note)', () => {
    const lines = [CONTROLS_HEADING_FOR_TEST, 'inline constexpr DriverControls kControls = controls(kRobotXMechanisms,',
                   '    preset(lift[facts::lv(12).name], Button::Up, 2_s));'];
    const r = parseControls(lines);
    assert.ok(r.ok, r.words);
    assert.equal(r.state.list[0].preset, 12);
});

test('a project file\'s volts are judged as the export prints them: 0.0004 V is the 0 V the file would say, and refused (the second review)', () => {
    const saved = toSaved({ list: [{ form: 'whileHeld', mechanism: 'intake', button: 'R1', back: 'R2', volts: 12 }] });
    const key = Object.keys(saved).find((k) => Array.isArray(saved[k]) && saved[k].length);
    saved[key][0].volts = 0.0004;
    const back = fromSaved(saved);
    assert.ok(back.ok, back.words);
    assert.equal(back.state.list[0].volts, 0);
    assert.ok(checkControls(back.state.list, ROBOTS().x).some((x) => x.refusal === 'a_hold_needs_volts_above_0_and_at_most_12'));
});

test('G4: a two-button hold whose second button is not chosen yet is "unfinished" -- never a one-way hold, never a guessed button', () => {
    const st = addControl({ list: [] }, { form: 'whileHeld', mechanism: 'intake', button: 'A', back: null, volts: 12 }, ROBOTS().x);
    const errs = checkControls(st.list, ROBOTS().x).filter((x) => x.at === 0);
    assert.equal(errs.length, 1);
    assert.equal(errs[0].code, 'unfinished');
    assert.match(errs[0].words, /second button/);
    assert.equal(errs.filter((x) => x.refusal).length, 0, 'no build refusal is claimed for it');
});

test('overlapping or nested marker pairs, and an END marker twice, are refused: Copy only, nothing written (the review\'s finding)', () => {
    const text = marked('one');
    const r = readDriverFile(text, 'one');
    const regions = exportRegions({ ...r.state, driving: { ...r.state.driving, onRelease: 'brake' } }, { id: 'one' });
    // the control words' END moved below the controls region: one part inside another
    const nested = text.replace('// [/control words one]\n', '').replace('// [/driver file one]\n', '// [/driver file one]\n// [/control words one]\n');
    const n = spliceRegions(nested, 'one', regions, { base: r.hashes, overwrite: true });
    assert.equal(n.why, 'noMarkers');
    assert.match(n.words, /overlap/);
    assert.ok(readDriverFile(nested, 'one').problems.some((x) => /overlap/.test(x)));
    const endTwice = text.replace('// [/feel one]', '// [/feel one]\n// [/feel one]');
    assert.equal(spliceRegions(endTwice, 'one', regions, { base: r.hashes, overwrite: true }).why, 'noMarkers');
    assert.ok(spliceRegions(text, 'one', regions, { base: r.hashes }).ok, 'the marked file itself is written');
});

test('a write whose FIRST region changes length: every region lands between its own markers, nothing outside moves (the review\'s finding: splice order)', () => {
    for (const eol of ['\n', '\r\n']) {
        const text = marked('one').replace(/\n/g, eol);
        const r = readDriverFile(text, 'one');
        let c = addControl(r.state.controls, { form: 'preset', mechanism: 'claw', preset: 'open', button: 'Up' }, ROBOTS().one);
        c = removeFreeLine(c, checkMap(c)[0].free);  // the map loses its 3-line "d-pad FREE" row, gains one for Up
        c = addControl(c, { form: 'preset', mechanism: 'ramp', preset: 'down', button: 'Down' }, ROBOTS().one);
        const regions = exportRegions({ controls: c, driving: { ...r.state.driving, onRelease: 'brake' } }, { id: 'one' });
        assert.notEqual(regions['control words'].length, regionOf(findRegions(text, 'one'), 'control words').split('\n').length);
        const out = spliceRegions(text, 'one', regions, { base: r.hashes });
        assert.ok(out.ok, out.words);
        const f0 = findRegions(text, 'one');
        const f1 = findRegions(out.text, 'one');
        assert.deepEqual(f1.problems, []);
        for (const rg of REGIONS) {
            assert.equal(regionOf(f1, rg), regions[rg].join('\n'), `${rg} is the export`);
        }
        const outside = (ff) => ff.raw.filter((_, i) => !REGIONS.some((rg) => i >= ff.found[rg].from && i < ff.found[rg].to));
        assert.deepEqual(outside(f1), outside(f0), 'every line outside the regions, its markers included');
        assert.ok(readDriverFile(out.text, 'one').ok);
    }
});

test('a control\'s NEW job gets the page\'s words: the file\'s own comment and map rows never describe the old one (the review\'s finding)', () => {
    const one = readDriverFile(marked('one'), 'one').state.controls;
    const at = (st, mech, form) => st.list.findIndex((c) => c.mechanism === mech && c.form === form);
    // the flipper's toggle moved to the claw: "the front toggle" must go
    const flip = editControl(one, at(one, 'flipper', 'toggle'), { mechanism: 'claw' }, ROBOTS().one).list[at(one, 'flipper', 'toggle')];
    assert.equal(flip.words, defaultWords(flip, ROBOTS().one.mechanisms.find((m) => m.ident === 'claw')));
    assert.doesNotMatch(JSON.stringify(flip.banner), /front toggle|flipper/);
    // the driver-start home made a home on a button: "[delete-able] ... first driver start" must go
    const home = editControl(one, at(one, 'cascade', 'homeAtDriverStart'), { form: 'homeOnPress', button: 'Up' }, ROBOTS().one).list[at(one, 'cascade', 'homeAtDriverStart')];
    assert.doesNotMatch(home.words, /driver start/);
    assert.doesNotMatch(JSON.stringify(home.banner), /driver control starts/);
    // a button move keeps the team's words, re-keyed
    const moved = editControl(one, at(one, 'claw', 'toggle'), { button: 'Up' }, ROBOTS().one).list[at(one, 'claw', 'toggle')];
    assert.equal(moved.words, 'each press: open / close');
    assert.deepEqual(moved.banner.map((r) => r.key), ['Up']);
    assert.match(moved.banner[0].lines[0], /each press flips it/, 'the team\'s own words, kept');
    // robot two's 12 V hold made 6 V: "full power" must go from its line AND its map row
    const two = readDriverFile(marked('two'), 'two').state.controls;
    const robotTwo = { id: 'two', mechanisms: [...new Set(two.list.map((c) => c.mechanism))].map((ident) => ({ ident, name: ident, kind: 'power' })) };
    const h = two.list.findIndex((c) => c.form === 'whileHeld' && c.volts === 12);
    assert.ok(h >= 0 && /full power/.test(JSON.stringify(two.list[h])), 'the fixture says "full power" on a 12 V hold');
    const six = editControl(two, h, { volts: 6 }, robotTwo).list[h];
    assert.doesNotMatch(JSON.stringify([six.words, six.banner]), /full power/);
});

test('kept words that name a button whose job changed are refused until fixed (the page\'s words, one click); a note about it is flagged', () => {
    const r = readDriverFile(marked('one'), 'one');
    const one = ROBOTS().one;
    const st = r.state.controls;
    assert.deepEqual(checkWords(st, st, one), [], 'the file as read: nothing to say');
    // the cascade's hold moved to L1 / L2, the side toggles to R1 / R2: the (start) row still says "R1 / R2 wait"
    const at = (mech, form) => st.list.findIndex((c) => c.mechanism === mech && c.form === form);
    let s = editControl(st, at('rightToggle', 'whileHeld'), { button: 'Up' }, one);
    s = editControl(s, at('leftToggle', 'whileHeld'), { button: 'Down' }, one);
    s = editControl(s, at('cascade', 'whileHeld'), { button: 'L1', back: 'L2' }, one);
    s = editControl(s, at('rightToggle', 'whileHeld'), { button: 'R1' }, one);
    s = editControl(s, at('leftToggle', 'whileHeld'), { button: 'R2' }, one);
    const stale = checkWords(s, st, one).filter((x) => x.level === 'error');
    assert.deepEqual(stale.map((x) => [x.code, x.at]), [['wordsStale', at('cascade', 'homeAtDriverStart')]]);
    assert.match(stale[0].words, /R1, R2/);
    const fixed = pageWords(s, stale[0].at, one);
    assert.deepEqual(checkWords(fixed, st, one).filter((x) => x.level === 'error'), []);
    // the cascade's 12 V changed: the note "The cascade's 12 V is CHOSEN" is flagged (a warning: the team's prose)
    const v = editControl(st, at('cascade', 'whileHeld'), { volts: 8 }, one);
    const notes = checkWords(v, st, one);
    assert.ok(notes.some((x) => x.code === 'noteStale' && x.level === 'warn' && /cascade/.test(x.words)), JSON.stringify(notes));
    // the cascade's two buttons swapped: "R1 / R2 wait" is still true (both still the cascade's), not flagged; a comment
    // of the team's on the hold itself saying "R1 lifts" is (R1 lowers it now)
    const swapped = editControl(st, at('cascade', 'whileHeld'), { button: 'R2', back: 'R1' }, one);
    assert.deepEqual(checkWords(swapped, st, one), []);
    const own = editControl(st, at('cascade', 'whileHeld'), { words: 'R1 lifts, R2 lowers' }, one);
    const ownSwapped = editControl(own, at('cascade', 'whileHeld'), { button: 'R2', back: 'R1' }, one);
    assert.deepEqual(checkWords(ownSwapped, st, one).map((x) => [x.code, x.at]), [['wordsStale', at('cascade', 'whileHeld')]]);
    // "A held" is English, not the button A
    assert.deepEqual(checkWords({ ...v, notes: ['A held button gives it at most its unhomed volts'] }, st, one).filter((x) => /"A held/.test(x.words)), []);
});

test('words the team TYPES are judged against the jobs when typed, the file\'s own against the file (the verifier\'s finding: true new words refused)', () => {
    const r = readDriverFile(marked('one'), 'one');
    const one = ROBOTS().one;
    const st = r.state.controls;
    const at = (mech, form) => st.list.findIndex((c) => c.mechanism === mech && c.form === form);
    const remap = (s0, hold, toggles) => {
        let s = editControl(s0, at('rightToggle', 'whileHeld'), { button: 'Up' }, one);
        s = editControl(s, at('leftToggle', 'whileHeld'), { button: 'Down' }, one);
        s = editControl(s, at('cascade', 'whileHeld'), { button: hold[0], back: hold[1] }, one);
        s = editControl(s, at('rightToggle', 'whileHeld'), { button: toggles[0] }, one);
        return editControl(s, at('leftToggle', 'whileHeld'), { button: toggles[1] }, one);
    };
    const errs = (s) => checkWords(s, st, one).filter((x) => x.level === 'error');
    const moved = remap(st, ['L1', 'L2'], ['R1', 'R2']);
    const start = at('cascade', 'homeAtDriverStart');
    const hold = at('cascade', 'whileHeld');
    const startRow = moved.list[start].banner;
    assert.equal(startRow.length, 1);
    // the (start) row rewritten TRUTHFULLY: nothing to say
    const truthful = editControl(moved, start, { banner: startRow.map((x) => ({ ...x, lines: x.lines.map((l) => l.replaceAll('R1 / R2', 'L1 / L2')) })) }, one);
    assert.deepEqual(errs(truthful), []);
    // true words of the team's own on the moved hold, its comment and its map rows
    let typed = editControl(truthful, hold, { words: 'L1 raises it, L2 lowers it' }, one);
    typed = editControl(typed, hold, { banner: [{ key: 'L1', lines: ['HOLD: the cascade goes UP (L1)'] }, { key: 'L2', lines: ['HOLD: the cascade goes DOWN (L2)'] }] }, one);
    assert.deepEqual(errs(typed), []);
    // ONE of the row's two "R1 / R2" lines fixed: the other is still the file's, still refused
    const half = editControl(moved, start, { banner: startRow.map((x) => ({ ...x, lines: x.lines.map((l, k) => (k === 1 ? l.replace('R1 / R2', 'L1 / L2') : l)) })) }, one);
    assert.match(startRow[0].lines[1], /R1 \/ R2/);
    assert.match(startRow[0].lines.at(-1), /R1 \/ R2/);
    assert.deepEqual(errs(half).map((x) => [x.at, x.rows]), [[start, [0]]]);
    // the team's TRUE words go stale when the jobs change AFTER they were typed
    const back = remap(typed, ['R1', 'R2'], ['L1', 'L2']);
    assert.deepEqual(errs(back).map((x) => [x.at, x.comment, x.rows]).sort(), [[hold, true, [0, 1]], [start, false, [0]]].sort());
    // "Use the page's words" replaces only what is stale: the (start) row, never the [delete-able] comment
    const flagged = errs(moved)[0];
    assert.deepEqual([flagged.at, flagged.comment, flagged.rows], [start, false, [0]]);
    assert.match(flagged.words, /map row "\(start\)"/);
    const fixed = pageWords(moved, start, one, flagged);
    assert.equal(fixed.list[start].words, st.list[start].words);
    assert.match(fixed.list[start].words, /\[delete-able\]/);
    assert.doesNotMatch(JSON.stringify(fixed.list[start].banner), /R1|R2/);
    assert.deepEqual(errs(fixed), []);
    // the team's words are never saved with their jobs (the project keeps G3a's shape): opened again, a line that is
    // neither the file's nor typed since is judged against the project as opened
    assert.doesNotMatch(JSON.stringify(toSaved(typed)), /wordsFor/);
    const reopened = fromSaved(toSaved(truthful)).state;
    assert.deepEqual(checkWords(reopened, st, one, reopened).filter((x) => x.level === 'error'), []);
    assert.equal(checkWords(reopened, st, one).filter((x) => x.level === 'error').length, 1, 'against the file, the same line names L1 / L2');
});

test('a combined map row ("R1 / R2") moved to Left / Right keeps the team\'s words, keyed "Left/Right", and reads back (the verifier\'s finding)', () => {
    const r = readDriverFile(marked('two'), 'two');
    const two = r.state.controls;
    const robotTwo = { id: 'two', mechanisms: [...new Set(two.list.map((c) => c.mechanism))].map((ident) => ({ ident, name: ident, kind: 'power' })) };
    const i = two.list.findIndex((c) => (c.banner || []).length === 1 && c.back !== undefined && / \/ /.test(c.banner[0].key));
    assert.ok(i >= 0, 'robot two has a combined row');
    for (const [b, k, key] of [['Left', 'Right', 'Left/Right'], ['Right', 'Down', 'Right/Down'], ['Up', 'Down', 'Up / Down'], ['Down', 'Left', 'Down / Left']]) {
        const moved = editControl(two, i, { button: b, back: k }, robotTwo);
        assert.deepEqual(moved.list[i].banner, [{ ...two.list[i].banner[0], key }]);
        const regions = exportRegions({ controls: moved, driving: r.state.driving }, { id: 'two' });
        const back = readDriverFile(spliceRegions(marked('two'), 'two', regions, { base: r.hashes }).text, 'two');
        assert.ok(back.ok, back.words);
        assert.deepEqual(back.state.controls.list[i].banner, moved.list[i].banner, `${key} reads back onto its control`);
    }
});

test('a map key names the buttons GM22\'s C++ reads in it: split at "/" and "," only (the page and the suite agree on every row)', () => {
    // both robots' GM22 cases read the map through test/driver_map_rows.hpp (2026-11-20: each robot's map its own)
    assert.match(read('test/driver_map_rows.hpp'), /key\.find_first_of\("\/,", from\)/, 'GM22 splits at "/" and ","');
    assert.match(read('test/robot_two_driver_test.cpp'), /driver_map::keyButtons\(key\)/, 'robot two\'s GM22 reads it the same way');
    assert.deepEqual(keyButtons('R1 / R2'), ['R1', 'R2']);
    assert.deepEqual(keyButtons('Left/Right'), ['Left', 'Right']);
    assert.deepEqual(keyButtons('B, X'), ['B', 'X']);
    assert.deepEqual(keyButtons('d-pad'), ['Up', 'Down', 'Left', 'Right']);
    assert.deepEqual(keyButtons('Up and Down'), [], 'no button there for GM22 either');
});
