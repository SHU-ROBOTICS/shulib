// controls.mjs — THE DRIVER CONTROLS (RMP1-B): the driver file's `kControls` list as data, its checks (one per
// build refusal of include/shulib/mechanism/driver_inputs.hpp's controls(), plus DriverFile's re-zero rule), and
// the two regions it writes: `controls` (the WHICH BUTTON heading and the statement) and `control words` (the
// header's stick-and-button map, made from the same choices so the two cannot disagree). Both are read back too,
// exactly the forms written here. The project keeps them in G3a's shape (D14; toSaved/fromSaved). Pure.

import { fmt, round3 } from './numbers.mjs';
import { findWord } from './lift.mjs';

/** The controller's twelve buttons, in hal::ControllerButton's order (the build names them in this order). */
export const BUTTONS = ['L1', 'L2', 'R1', 'R2', 'Up', 'Down', 'Left', 'Right', 'X', 'B', 'Y', 'A'];
export const MAX_CONTROLS = 24;  // DriverControls::kCap
export const CONTROLS_HEADING = '// ═══ WHICH BUTTON DOES WHAT ═══════════════════════════════════════════════════════════════';

const isPosition = (m) => m && (m.kind === 'arm' || m.kind === 'lift');
export const mechOf = (robot, ident) => (robot.mechanisms || []).find((m) => m.ident === ident);

/** The buttons a control takes (a two-way hold: forward then back). */
export function buttonsOf(c) {
    if (c.form === 'whileHeld') {
        return c.back ? [c.button, c.back] : [c.button];
    }
    return c.button ? [c.button] : [];
}

/** What the page offers for a mechanism's kind (§1.2): [{form, words}]. */
export function actionsFor(m) {
    if (!m) {
        return [];
    }
    const auton = { form: 'autonOnly', words: 'No button: autons only' };
    switch (m.kind) {
        case 'power': return [{ form: 'whileHeld', two: true, words: 'Run while held: one button forward, one back' },
                              { form: 'whileHeld', two: false, words: 'Run while held: one button, one way' }, auton];
        case 'arm':
        case 'lift': return [{ form: 'whileHeld', two: true, words: 'Move while held: one button up, one down' },
                             { form: 'preset', words: 'Go to a level on one press' },
                             { form: 'homeOnPress', words: `${findWord(m, true)} on one press` },
                             { form: 'homeAtDriverStart', words: `${findWord(m, true)} when driver control starts` }, auton];
        case 'pneumatic': return [{ form: 'toggle', words: 'Each press flips it' },
                                  { form: 'preset', words: 'Go to one position on one press' }, auton];
        case 'flywheel': return [{ form: 'toggle', words: 'Each press: spin up to a speed / stop' },
                                 { form: 'preset', words: 'Spin up to a speed on one press' }, auton];
        default: return [auton];
    }
}

/** A target's words: a ledger level by index, else its name. */
function targetWords(m, preset) {
    if (typeof preset === 'number') {
        const p = (m.presets || []).find((q) => q.ledger === preset);
        return p ? `level ${preset} "${p.name}"` : `level ${preset}`;
    }
    return `"${preset}"`;
}

/** The comment on a control's line, in plain words (the default; the student may change it). */
export function defaultWords(c, m) {
    const name = m ? m.name : c.mechanism;
    switch (c.form) {
        case 'whileHeld':
            if (!c.back) {
                return 'held: runs (one way); let go: stops';
            }
            return isPosition(m) ? `${c.button} up / ${c.back} down while held; let go: it holds` : 'forward / back while held; let go: it stops';
        case 'toggle': return m && m.kind === 'flywheel' ? `each press: spin up to "${c.speed}" / stop`
            : `each press: ${(m && m.positions && m.positions[1]) || 'one'} / ${(m && m.positions && m.positions[0]) || 'the other'}`;
        case 'preset': return m && m.kind === 'pneumatic' ? `press: ${name} ${c.preset}` : `press: goes to ${m ? targetWords(m, c.preset) : c.preset}`;
        case 'homeOnPress': return 'press: homes it';
        case 'homeAtDriverStart': return 'home it at the first driver start';
        case 'autonOnly': return 'autons only: no button';
        default: return '';
    }
}

/** The header map's rows for a control (the default): [{key, lines}]. */
export function defaultBanner(c, m) {
    const name = m ? `the ${m.name}` : c.mechanism;
    const t = (s) => (Number.isFinite(c.timeLimitS) ? `${s} (at most ${fmt(c.timeLimitS)} s)` : s);
    switch (c.form) {
        case 'whileHeld':
            if (!c.back) {
                return [{ key: c.button, lines: [`HOLD: ${name} runs (one way only); let go: it stops`] }];
            }
            return isPosition(m)
                ? [{ key: c.button, lines: [`HOLD: ${name} goes UP; let go: it stops and holds where it is`] },
                   { key: c.back, lines: [`HOLD: ${name} goes DOWN; let go: it stops and holds`] }]
                : [{ key: c.button, lines: [`HOLD: ${name} runs forward; let go: it stops`] },
                   { key: c.back, lines: [`HOLD: ${name} runs back; let go: it stops`] }];
        case 'toggle':
            return [{ key: c.button, lines: [m && m.kind === 'flywheel' ? `${name}: press = spin up to "${c.speed}", press again = stop`
                : `${name}: press = ${(m && m.positions && m.positions[1]) || 'one way'}, press again = ${(m && m.positions && m.positions[0]) || 'back'}`] }];
        case 'preset':
            return [{ key: c.button, lines: [m && m.kind === 'pneumatic' ? `${name}: press = ${c.preset}` : t(`${name} goes to ${m ? targetWords(m, c.preset) : c.preset}`)] }];
        case 'homeOnPress': return [{ key: c.button, lines: [t(`${name} HOMES`)] }];
        case 'homeAtDriverStart': return [{ key: '(start)', lines: [t(`the FIRST time driver control starts in a run, ${name} HOMES`)] }];
        default: return [];
    }
}

const STICKS = {
    arcade: [{ key: 'left stick', lines: ['up / down: drive forward / back'] }, { key: 'right stick', lines: ['left / right: turn'] }],
    tankSticks: [{ key: 'left stick', lines: ['up / down: the LEFT side forward / back'] }, { key: 'right stick', lines: ['up / down: the RIGHT side forward / back'] }],
    arcadeField: [{ key: 'left stick', lines: ['pushes the robot that way on the FIELD, whichever way it faces'] }, { key: 'right stick', lines: ['left / right: turn'] }],
};
/** The map's rows the driving block decides (defaults). */
export const sticksRows = (mode) => STICKS[mode || 'arcade'].map((r) => ({ ...r, lines: [...r.lines] }));
export const rezeroRows = (button, mode) => [{ key: button || 'Y', lines: [mode === 'arcadeField'
    ? 'field-centric re-zero: press it with the robot facing away from you' : 'field-centric re-zero. Field-centric is OFF, so it changes nothing yet'] }];
export const releaseRows = (r) => [{ key: '', lines: [`let go of the sticks and the drive ${{ coast: 'coasts', brake: 'brakes', hold: 'holds where it is' }[r || 'coast']}`] }];

// ── the checks ─────────────────────────────────────────────────────────────────────────────────────
/** The page's plain words for each build refusal (the library's name rides beside them). */
const SAY = {
    whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset: (n) => `Hold-to-run is for a motor: give ${n} a toggle or a press instead.`,
    whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two: (n) => `${n} needs two buttons (one up, one down): one button runs only a plain motor, one way.`,
    a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot: (n) => `Choose the speed ${n}'s toggle spins up to.`,
    a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed: (n) => `A toggle is for a piston or a flywheel: ${n} cannot be toggled.`,
    homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift: (n) => `Only an arm or a lift finds its bottom: ${n} cannot.`,
    a_control_names_a_mechanism_this_robot_does_not_have: (n) => `This robot has no mechanism called ${n}.`,
    a_control_names_a_button_that_is_not_one_of_the_twelve: (n, b) => `"${b}" is not one of the controller's twelve buttons.`,
    a_hold_needs_volts_above_0_and_at_most_12: (n) => `${n}'s volts must be above 0 and at most 12.`,
    a_preset_or_a_homing_needs_a_time_limit_above_zero: (n) => `Type ${n}'s time limit: the most this may take, above 0 s.`,
    two_controls_of_the_same_kind_on_one_mechanism: (n) => `${n} has two controls that do the same job: remove one.`,
    a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed: (n) => `${n} has more than two positions, so a toggle cannot know where to go: use "go to one position" buttons.`,
    a_homing_control_on_a_mechanism_that_is_never_homed: (n) => `${n} never finds its zero (its robot file says so): it cannot have a button for it.`,
    a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant: (n) => `${n} has no button. Give it one, or choose "autons only".`,
    a_mechanism_marked_autonOnly_also_has_a_control: (n) => `${n} is "autons only" and also has a button: remove one of them.`,
    more_controls_than_fit: () => `More than ${MAX_CONTROLS} controls: the driver file holds at most ${MAX_CONTROLS}.`,
    a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control: (n, _, m) => `${n} must ${findWord(m)} before it can go to a level (and before a hold moves it at full volts): add "${findWord(m)} when driver control starts" or a button that does it.`,
    a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it: (n) => `${n} never finds its zero and its robot file gives it no "unhomed volts", so its buttons would not move it.`,
    a_preset_on_a_mechanism_that_is_never_homed: (n) => `${n} never finds its zero, so it never knows where a level is: it cannot have a level button.`,
    no_such_preset_on_this_mechanism: (n, p) => `${n} has no level or speed ${p}.`,
    no_such_position_on_this_mechanism: (n, p) => `${n} has no position ${p}.`,
    a_power_mechanism_has_no_presets_run_it_with_r_run: (n) => `${n} is a plain motor: it has no levels. Give it "run while held" instead.`,
    rezero: (n, b) => `${b} is the drive's re-zero button (the feel's): ${n} cannot use it. Pick another button, or move the re-zero.`,
};
/** The build's own words for the two refusals that are not a refused:: name. */
export const REZERO_REFUSAL = "a mechanism's button is the loop's re-zero button";
export const FEEL_REFUSAL = 'the feel is out of range';

const realBound = (s) => typeof s === 'number' && s > 0 && s < 1e9;

/**
 * Every build refusal the controls would meet: [{level: 'error', refusal, at (control index or -1), mechanism,
 * button, words}]. The order of controls() itself; `rezero` is the feel's re-zero button.
 */
export function checkControls(list, robot, { rezero = 'Y' } = {}) {
    const out = [];
    const say = (refusal, at, c, extra) => {
        const m = c && mechOf(robot, c.mechanism);
        const n = m ? `the ${m.name}` : c ? `"${c.mechanism}"` : '';
        out.push({ level: 'error', refusal, at, mechanism: c && c.mechanism, words: SAY[refusal](n, extra, m), ...(extra !== undefined ? { button: extra } : {}) });
    };
    if (list.length > MAX_CONTROLS) {
        say('more_controls_than_fit', -1, null);
    }
    const used = new Set();
    const take = (b, i, c) => {
        if (!BUTTONS.includes(b)) {
            say('a_control_names_a_button_that_is_not_one_of_the_twelve', i, c, b);
        } else if (used.has(b)) {
            out.push({ level: 'error', refusal: `button_${b}_does_two_jobs`, at: i, mechanism: c.mechanism, button: b,
                       words: `${b} does two jobs: give one of them another button.` });
        } else {
            used.add(b);
        }
    };
    list.slice(0, MAX_CONTROLS).forEach((c, i) => {
        const m = mechOf(robot, c.mechanism);
        if (!m) {
            say('a_control_names_a_mechanism_this_robot_does_not_have', i, c);
            return;
        }
        const ask = (words) => out.push({ level: 'error', code: 'unfinished', at: i, mechanism: c.mechanism, words });
        if (c.form !== 'homeAtDriverStart' && c.form !== 'autonOnly' && c.button === undefined) {
            ask(`Choose ${m ? `the ${m.name}'s` : 'its'} button.`);
            return;
        }
        if (c.form === 'whileHeld' && c.back === null) {
            ask(`Choose ${m ? `the ${m.name}'s` : 'its'} second button (the ${isPosition(m) ? 'down' : 'back'} one).`);
            return;
        }
        if (c.form === 'preset' && c.preset === undefined) {
            ask(`Choose where ${m ? `the ${m.name}` : 'it'} goes on this button.`);
            return;
        }
        // the words themselves (each a consteval refusal in driver_inputs.hpp / kinds.hpp)
        if (c.form === 'whileHeld' && !c.back && m.kind !== 'power') {
            say('whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two', i, c);
            return;
        }
        if (c.form === 'whileHeld' && (m.kind === 'pneumatic' || m.kind === 'flywheel')) {
            say('whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset', i, c);
            return;
        }
        if (c.form === 'toggle' && m.kind === 'flywheel' && c.speed === undefined) {
            say('a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot', i, c);
            return;
        }
        if (c.form === 'toggle' && m.kind !== 'pneumatic' && m.kind !== 'flywheel') {
            say('a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed', i, c);
            return;
        }
        if ((c.form === 'homeOnPress' || c.form === 'homeAtDriverStart') && !isPosition(m)) {
            say('homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift', i, c);
            return;
        }
        const target = c.form === 'preset' ? c.preset : c.form === 'toggle' && m.kind === 'flywheel' ? c.speed : undefined;
        if (target !== undefined && m.kind === 'power') {
            say('a_power_mechanism_has_no_presets_run_it_with_r_run', i, c);
            return;
        }
        if (target !== undefined && presetOf(m, target) === undefined) {
            say(m.kind === 'pneumatic' ? 'no_such_position_on_this_mechanism' : 'no_such_preset_on_this_mechanism', i, c,
                typeof target === 'number' ? `level ${target}` : `"${target}"`);
            return;
        }
        // controls()'s own rules
        switch (c.form) {
            case 'whileHeld':
                take(c.button, i, c);
                if (c.back) {
                    take(c.back, i, c);
                }
                if (!(c.volts > 0 && c.volts <= 12)) {
                    say('a_hold_needs_volts_above_0_and_at_most_12', i, c);
                }
                break;
            case 'toggle':
                take(c.button, i, c);
                if (m.kind === 'pneumatic' && (m.positions || []).length !== 2) {
                    say('a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed', i, c);
                }
                break;
            case 'preset':
                take(c.button, i, c);
                if (m.kind !== 'pneumatic' && !realBound(c.timeLimitS)) {
                    say('a_preset_or_a_homing_needs_a_time_limit_above_zero', i, c);
                }
                break;
            case 'homeOnPress':
            case 'homeAtDriverStart':
                if (c.form === 'homeOnPress') {
                    take(c.button, i, c);
                }
                if (!realBound(c.timeLimitS)) {
                    say('a_preset_or_a_homing_needs_a_time_limit_above_zero', i, c);
                }
                if (m.homing === 'never') {
                    say('a_homing_control_on_a_mechanism_that_is_never_homed', i, c);
                }
                break;
            default:
                break;
        }
        for (let j = i + 1; j < Math.min(list.length, MAX_CONTROLS); ++j) {
            const d = list[j];
            if (d.mechanism !== c.mechanism || d.form !== c.form) {
                continue;
            }
            if (c.form === 'preset' && !sameTarget(m, c.preset, d.preset)) {
                continue;
            }
            say('two_controls_of_the_same_kind_on_one_mechanism', j, d);
        }
    });
    for (const m of robot.mechanisms || []) {
        const mine = list.slice(0, MAX_CONTROLS).filter((c) => c.mechanism === m.ident);
        const marked = mine.some((c) => c.form === 'autonOnly');
        const controlled = mine.some((c) => c.form !== 'autonOnly');
        const anyAt = mine.length ? list.indexOf(mine[0]) : -1;
        const c0 = { mechanism: m.ident };
        if (!controlled && !marked) {
            say('a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant', -1, c0);
        }
        if (controlled && marked) {
            say('a_mechanism_marked_autonOnly_also_has_a_control', anyAt, c0);
        }
        const holds = mine.some((c) => c.form === 'whileHeld');
        const presets = mine.some((c) => c.form === 'preset');
        const homes = mine.some((c) => c.form === 'homeOnPress' || c.form === 'homeAtDriverStart');
        if (!isPosition(m) || !(holds || presets)) {
            continue;
        }
        const unhomed = m.unhomedVolts > 0;
        if ((m.homing === 'switch' || m.homing === 'stall') && !homes && (presets || !unhomed)) {
            say('a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control', anyAt, c0);
        }
        if (m.homing === 'never' && presets) {
            say('a_preset_on_a_mechanism_that_is_never_homed', anyAt, c0);
        }
        if (m.homing === 'never' && holds && !unhomed) {
            say('a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it', anyAt, c0);
        }
    }
    // DriverFile's static_assert: no control takes the loop's re-zero button
    list.slice(0, MAX_CONTROLS).forEach((c, i) => {
        if (buttonsOf(c).includes(rezero) && c.form !== 'homeAtDriverStart' && c.form !== 'autonOnly') {
            const m = mechOf(robot, c.mechanism);
            out.push({ level: 'error', refusal: REZERO_REFUSAL, at: i, mechanism: c.mechanism, button: rezero,
                       words: SAY.rezero(m ? `the ${m.name}` : c.mechanism, rezero) });
        }
    });
    return out;
}

/** A mechanism's preset (by ledger index or name), position (a pneumatic) or speed; undefined when it has none. */
export function presetOf(m, target) {
    if (m.kind === 'pneumatic') {
        const i = (m.positions || []).indexOf(target);
        return i >= 0 ? { name: target, index: i } : undefined;
    }
    if (typeof target === 'number') {
        return (m.presets || []).find((p) => p.ledger === target);
    }
    return (m.presets || []).find((p) => p.name === target && p.ledger === undefined)
        || (m.presets || []).find((p) => p.name === target);
}

/** Two presets aim at the same target (the C++ compares the VALUE and the position, not the name). */
function sameTarget(m, a, b) {
    const pa = presetOf(m, a);
    const pb = presetOf(m, b);
    if (!pa || !pb) {
        return a === b;
    }
    return m.kind === 'pneumatic' ? pa.index === pb.index : pa.value === pb.value;
}

// ── the controls region ────────────────────────────────────────────────────────────────────────────
/** The C++ of one control (no comma, no comment). */
export function controlCode(c, m) {
    const b = (x) => `Button::${x}`;
    const target = (p) => (typeof p === 'number' ? `${c.mechanism}[facts::lv(${p}).name]` : `${c.mechanism}["${p}"]`);
    switch (c.form) {
        case 'whileHeld': return c.back ? `whileHeld(${c.mechanism}, ${b(c.button)}, ${b(c.back)}, ${fmt(c.volts)}_volt)`
            : `whileHeld(${c.mechanism}, ${b(c.button)}, ${fmt(c.volts)}_volt)`;
        case 'toggle': return c.speed !== undefined && m && m.kind === 'flywheel' ? `toggle(${target(c.speed)}, ${b(c.button)})`
            : `toggle(${c.mechanism}, ${b(c.button)})`;
        case 'preset': return m && m.kind === 'pneumatic' || c.timeLimitS === undefined ? `preset(${target(c.preset)}, ${b(c.button)})`
            : `preset(${target(c.preset)}, ${b(c.button)}, ${fmt(c.timeLimitS)}_s)`;
        case 'homeOnPress': return `homeOnPress(${c.mechanism}, ${b(c.button)}, ${fmt(c.timeLimitS)}_s)`;
        case 'homeAtDriverStart': return `homeAtDriverStart(${c.mechanism}, ${fmt(c.timeLimitS)}_s)`;
        case 'autonOnly': return `autonOnly(${c.mechanism})`;
        default: throw new Error(`no control form "${c.form}"`);
    }
}

/** The `controls` region's lines: the heading, `kControls = controls(<table>, ...)` one commented line each, notes. */
export function exportControls(state, robot) {
    const list = state.list;
    const table = state.table || `kRobot${robot.id[0].toUpperCase()}${robot.id.slice(1)}Mechanisms`;
    const head = state.head || [CONTROLS_HEADING];
    const open = `inline constexpr DriverControls kControls = controls(${table}`;
    if (!list.length) {
        return [...head, `${open});`, ...(state.notes || []).map(note)];
    }
    const codes = list.map((c, i) => `    ${controlCode(c, mechOf(robot, c.mechanism))}${i === list.length - 1 ? ');' : ','}`);
    const col = Math.max((state.layout && state.layout.commentColumn) || 0, ...codes.map((s) => s.length + 2));
    const lines = codes.map((s, i) => (list[i].words ? `${s.padEnd(col)}// ${list[i].words}` : s));
    return [...head, `${open},`, ...lines, ...(state.notes || []).map(note)];
}
const note = (l) => (l ? `// ${l}` : '//');

const NUM = '(\\d+(?:\\.\\d+)?)';
const ID = '([A-Za-z_]\\w*)';
const BTN = 'Button::(\\w+)';
const TGT = `${ID}(?:\\["([^"\\\\]*)"\\]|\\[facts::lv\\((\\d+)\\)\\.name\\])`;
const FORMS = [
    [new RegExp(`^whileHeld\\(${ID}, ${BTN}, ${BTN}, ${NUM}_volt\\)$`), (m) => ({ form: 'whileHeld', mechanism: m[1], button: m[2], back: m[3], volts: num(m[4]) })],
    [new RegExp(`^whileHeld\\(${ID}, ${BTN}, ${NUM}_volt\\)$`), (m) => ({ form: 'whileHeld', mechanism: m[1], button: m[2], volts: num(m[3]) })],
    [new RegExp(`^toggle\\(${ID}, ${BTN}\\)$`), (m) => ({ form: 'toggle', mechanism: m[1], button: m[2] })],
    [new RegExp(`^toggle\\(${TGT}, ${BTN}\\)$`), (m) => ({ form: 'toggle', mechanism: m[1], speed: tgt(m), button: m[4] })],
    [new RegExp(`^preset\\(${TGT}, ${BTN}, ${NUM}_s\\)$`), (m) => ({ form: 'preset', mechanism: m[1], preset: tgt(m), button: m[4], timeLimitS: num(m[5]) })],
    [new RegExp(`^preset\\(${TGT}, ${BTN}\\)$`), (m) => ({ form: 'preset', mechanism: m[1], preset: tgt(m), button: m[4] })],
    [new RegExp(`^homeOnPress\\(${ID}, ${BTN}, ${NUM}_s\\)$`), (m) => ({ form: 'homeOnPress', mechanism: m[1], button: m[2], timeLimitS: num(m[3]) })],
    [new RegExp(`^homeAtDriverStart\\(${ID}, ${NUM}_s\\)$`), (m) => ({ form: 'homeAtDriverStart', mechanism: m[1], timeLimitS: num(m[2]) })],
    [new RegExp(`^autonOnly\\(${ID}\\)$`), (m) => ({ form: 'autonOnly', mechanism: m[1] })],
];
const num = (s) => round3(Number(s));
const tgt = (m) => (m[3] !== undefined ? Number(m[3]) : m[2]);

/**
 * The controls a `controls` region says: {ok, state: {list, table, head, notes, layout}} or {ok: false, line, words}.
 * Only the forms this page writes; anything else (a constant target, a second statement) is a hand edit it will
 * not guess at.
 */
export function parseControls(lines) {
    const bad = (i, why) => ({ ok: false, line: i + 1, words: `line ${i + 1} of the controls: ${why}` });
    let i = 0;
    const head = [];
    while (i < lines.length && /^\/\/( |$)/.test(lines[i])) {
        head.push(lines[i++]);
    }
    const open = /^inline constexpr DriverControls kControls = controls\(([A-Za-z_]\w*)(,|\);)$/.exec(lines[i] || '');
    if (!open) {
        return bad(i, '`inline constexpr DriverControls kControls = controls(<the robot\'s list>,` was expected');
    }
    const state = { table: open[1], head, list: [], notes: [], layout: {} };
    ++i;
    let closed = open[2] === ');';
    while (!closed && i < lines.length) {
        const m = /^ {4}(\S.*?)(?:( +)\/\/ (.*))?$/.exec(lines[i]);
        if (!m) {
            return bad(i, 'a control line was expected');
        }
        let code = m[1];
        if (code.endsWith('));')) {
            closed = true;
            code = code.slice(0, -2);
        } else if (code.endsWith(',')) {
            code = code.slice(0, -1);
        } else {
            return bad(i, 'a control line ends with "," (or "));" for the last)');
        }
        const f = FORMS.find(([re]) => re.test(code));
        if (!f) {
            return bad(i, `"${code}" is not a control the page writes`);
        }
        const c = f[1](f[0].exec(code));
        if (m[3] !== undefined) {
            c.words = m[3];
            state.layout.commentColumn = state.layout.commentColumn || lines[i].indexOf('//', 4 + m[1].length);
        }
        state.list.push(c);
        ++i;
    }
    if (!closed) {
        return bad(i, 'the list never closes with "));"');
    }
    for (; i < lines.length; ++i) {
        if (!/^\/\/( |$)/.test(lines[i])) {
            return bad(i, 'only comment lines may follow the list');
        }
        state.notes.push(lines[i].slice(3));
    }
    return { ok: true, state };
}

// ── the control words region (the header's stick-and-button map) ───────────────────────────────────
const KEY_COL = 13;  // a key, padded: the words start at column 18 ("//   " + 13)
/** One row for a control's buttons together: "R1 / R2", or "Left/Right" where that is too long for the key. */
const pairKey = (keys) => (keys.join(' / ').length <= KEY_COL - 2 ? keys.join(' / ') : keys.join('/'));

function rowLines(r) {
    if (!r.key) {
        return r.lines.map((l) => `//   ${l}`);
    }
    return r.lines.map((l, k) => (k === 0 ? `//   ${r.key.padEnd(KEY_COL)}${l}` : `//${' '.repeat(16)}${l}`));
}

/** The map's rows in the order written: sticks, each button control's, the re-zero, free, no-button controls', release, notes. */
export function mapRows(state) {
    const b = state.banner || {};
    const withButton = state.list.filter((c) => buttonsOf(c).length);
    const noButton = state.list.filter((c) => !buttonsOf(c).length);
    return [...(b.sticks || []), ...withButton.flatMap((c) => c.banner || []), ...(b.rezero || []), ...(b.free || []),
            ...noButton.flatMap((c) => c.banner || []), ...(b.release || []), ...(b.notes || [])];
}

/** The `control words` region's lines. */
export function exportControlWords(state) {
    const b = state.banner || {};
    return [...(b.head || []), ...mapRows(state).flatMap(rowLines), ...(b.tail || [])];
}

/** The map's lines as rows ({key, lines}); lines that are not map rows are its head (before) and tail (after). */
function readRows(lines) {
    const rows = [];
    const head = [];
    const tail = [];
    for (const l of lines) {
        const cont = /^\/\/ {16}(\S.*)$/.exec(l);
        const row = /^\/\/ {3}(\S.*)$/.exec(l);
        if (tail.length) {
            tail.push(l);
        } else if (cont && rows.length && rows[rows.length - 1].key) {
            rows[rows.length - 1].lines.push(cont[1]);
        } else if (row) {
            const k = /^(\S(?:\S| (?! ))*)( {2,})(\S.*)$/.exec(row[1]);
            if (k && k[1].length + k[2].length === KEY_COL) {
                rows.push({ key: k[1], lines: [k[3]] });
            } else {
                rows.push({ key: '', lines: [row[1]] });
            }
        } else if (rows.length) {
            tail.push(l);
        } else {
            head.push(l);
        }
    }
    return { rows, head, tail };
}

/** Which buttons a map key names ("R1 / R2", "Left/Right", "d-pad", "B, X"), as GM22's C++ reads them. */
export function keyButtons(key) {
    const out = [];
    for (const t of key.split(/\s*[/,]\s*/)) {
        if (t === 'd-pad') {
            out.push('Up', 'Down', 'Left', 'Right');
        } else if (BUTTONS.includes(t)) {
            out.push(t);
        }
    }
    return out;
}

/**
 * Read the map back onto `state.list` (each control gets its rows, by its buttons, in order): {ok, state} with
 * `state.banner` and each control's `banner`, or {ok: false, words} when the map and the controls disagree.
 */
export function parseControlWords(lines, state, { rezero = 'Y' } = {}) {
    const { rows, head, tail } = readRows(lines);
    const fail = (why) => ({ ok: false, words: `the button map in the header ${why}` });
    let i = 0;
    const banner = { head, tail, sticks: [], rezero: [], free: [], release: [], notes: [] };
    while (i < rows.length && /stick/.test(rows[i].key)) {
        banner.sticks.push(rows[i++]);
    }
    const list = state.list.map((c) => ({ ...c, banner: [] }));
    for (const c of list.filter((x) => buttonsOf(x).length)) {
        const keys = buttonsOf(c);
        if (rows[i] && rows[i].key === pairKey(keys)) {
            c.banner = [rows[i++]];
        } else if (keys.every((k, j) => rows[i + j] && rows[i + j].key === k)) {
            c.banner = rows.slice(i, i + keys.length);
            i += keys.length;
        } else {
            return fail(`does not say what ${keys.join(' / ')} does where the controls put it (it says "${rows[i] ? rows[i].key || rows[i].lines[0] : 'nothing'}" there)`);
        }
    }
    if (rows[i] && rows[i].key === rezero) {
        banner.rezero.push(rows[i++]);
    }
    const taken = new Set(list.flatMap(buttonsOf));
    const noButton = list.filter((x) => !buttonsOf(x).length);
    const startKey = (c) => (c.form === 'homeAtDriverStart' ? '(start)' : '(autons)');
    while (rows[i] && rows[i].key && !noButton.some((c) => startKey(c) === rows[i].key)) {
        const clash = keyButtons(rows[i].key).find((x) => taken.has(x));
        if (clash) {
            return fail(`says "${rows[i].key}" ${rows[i].lines[0].split(/[;—.]/)[0]}, but ${clash} has a control`);
        }
        banner.free.push(rows[i++]);
    }
    for (const c of noButton) {
        if (rows[i] && rows[i].key === startKey(c)) {
            c.banner = [rows[i++]];
        }
    }
    while (rows[i] && !rows[i].key && /^let go of the sticks/.test(rows[i].lines[0])) {
        banner.release.push(rows[i++]);
    }
    for (; i < rows.length; ++i) {
        const named = keyButtons(rows[i].key);
        if (rows[i].key && named.length) {
            return fail(`lists "${rows[i].key}" after the controls (a line left from an old map)`);
        }
        banner.notes.push(rows[i]);
    }
    return { ok: true, state: { ...state, list, banner } };
}

// ── the project file's shape (G3a's `controls` block, D14) ─────────────────────────────────────────
const KIND_KEY = { whileHeld: 'whileHeld', toggle: 'toggles', preset: 'presets', homeOnPress: 'homeOnPress',
                   homeAtDriverStart: 'homeAtDriverStart', autonOnly: 'autonOnly' };
const FIELD_ORDER = ['at', 'mechanism', 'button', 'back', 'volts', 'speed', 'preset', 'timeLimitS', 'words', 'banner'];

/** The controls as G3a's per-kind lists, each entry with `at` (its place in the statement, from 1). */
export function toSaved(state) {
    const out = { whileHeld: [], toggles: [], presets: [], homeOnPress: [], homeAtDriverStart: [], autonOnly: [] };
    state.list.forEach((c, i) => {
        const e = {};
        for (const k of FIELD_ORDER) {
            const v = k === 'at' ? i + 1 : c[k];
            if (v !== undefined) {
                e[k] = v;
            }
        }
        out[KIND_KEY[c.form]].push(e);
    });
    for (const k of ['table', 'head', 'notes', 'layout', 'banner']) {
        if (state[k] !== undefined) {
            out[k] = state[k];
        }
    }
    return out;
}

/** The saved block back into a state: {ok, state} or {ok: false, words}. */
export function fromSaved(saved) {
    if (!saved || typeof saved !== 'object') {
        return { ok: false, words: 'the controls block is not a set of lists' };
    }
    const list = [];
    for (const [form, key] of Object.entries(KIND_KEY)) {
        for (const e of saved[key] || []) {
            if (!e || typeof e.at !== 'number' || typeof e.mechanism !== 'string') {
                return { ok: false, words: `a "${key}" entry has no "at" or "mechanism"` };
            }
            const unknown = Object.keys(e).find((k) => !FIELD_ORDER.includes(k));
            if (unknown) {
                return { ok: false, words: `a "${key}" entry has a field "${unknown}" this route builder does not write` };
            }
            const c = { form, ...e };
            delete c.at;
            for (const k of ['volts', 'timeLimitS']) {  // judged as the export prints them (W7)
                if (typeof c[k] === 'number') {
                    c[k] = round3(c[k]);
                }
            }
            list[e.at - 1] = c;
        }
    }
    if (list.some((c) => c === undefined) || list.length !== Object.values(KIND_KEY).reduce((n, k) => n + (saved[k] || []).length, 0)) {
        return { ok: false, words: 'the controls\' "at" places are not 1, 2, 3, ... once each' };
    }
    const state = { list };
    for (const k of ['table', 'head', 'notes', 'layout', 'banner']) {
        if (saved[k] !== undefined) {
            state[k] = saved[k];
        }
    }
    return { ok: true, state };
}

// ── edits (pure: each returns a new state) ─────────────────────────────────────────────────────────
const sameRows = (a, b) => JSON.stringify(a || []) === JSON.stringify(b || []);
const clean = (o) => Object.fromEntries(Object.entries(o).filter(([, v]) => v !== undefined));

/** `c`'s map rows re-keyed to its buttons (the student's words kept), or null when they no longer fit. */
function rekey(rows, c) {
    const keys = buttonsOf(c);
    if (rows.length === 1 && keys.length > 1) {
        return [{ ...rows[0], key: pairKey(keys) }];
    }
    if (rows.length === keys.length) {
        return rows.map((r, k) => ({ ...r, key: keys[k] }));
    }
    return null;
}

/** A control added at the end of the list, with the page's plain words (the student may change them). */
export function addControl(state, c, robot) {
    const m = mechOf(robot, c.mechanism);
    const next = clean({ ...c, words: defaultWords(c, m), banner: defaultBanner(c, m) });
    return { ...state, list: [...state.list, next] };
}

/** What a control DOES — everything but which buttons. */
const JOB = ['form', 'mechanism', 'volts', 'speed', 'preset', 'timeLimitS'];
const sameJob = (a, b) => JOB.every((k) => a[k] === b[k]) && (a.back === undefined) === (b.back === undefined);
/** The student's own words on a control (not the page's): its comment ('words'), its map rows ('banner'), or either. */
export const ownWords = (c, m, part) => (part !== 'banner' && c.words !== undefined && c.words !== defaultWords(c, m))
    || (part !== 'words' && !!(c.banner && c.banner.length) && !sameRows(c.banner, defaultBanner(c, m)));
/** Each line a control says, with where: [text, -1] its comment, [text, k] a line of map row k. */
const linesOf = (c) => [...(c.words !== undefined ? [[c.words, -1]] : []), ...(c.banner || []).flatMap((r, k) => r.lines.map((l) => [l, k]))];
const pageLines = (c, m) => new Set([defaultWords(c, m), ...defaultBanner(c, m).flatMap((r) => r.lines)]);

/** Control `i` changed by `patch`. A new JOB gets the page's words again (the old ones described the old job); a
 *  button move keeps the student's own, re-keyed to the new buttons. */
export function editControl(state, i, patch, robot) {
    const old = state.list[i];
    let next = clean({ ...old, ...patch });
    const m0 = mechOf(robot, old.mechanism);
    const m1 = mechOf(robot, next.mechanism);
    if (next.mechanism !== old.mechanism) {
        // another mechanism: its old target (a flywheel's speed, a level, a position) means nothing here
        next = clean({ ...next, speed: undefined, preset: 'preset' in patch ? next.preset : undefined });
        if (!actionsFor(m1).some((a) => a.form === next.form && (a.form !== 'whileHeld' || a.two === !!next.back))) {
            next = clean({ ...next, form: (actionsFor(m1)[0] || {}).form, back: undefined });
        }
    }
    const job = sameJob(old, next);
    if (!('words' in patch) && (!job || old.words === undefined || old.words === defaultWords(old, m0))) {
        next.words = defaultWords(next, m1);
    }
    if (!('banner' in patch)) {
        const own = job && old.banner && old.banner.length && !sameRows(old.banner, defaultBanner(old, m0));
        next.banner = (own && rekey(old.banner, next)) || defaultBanner(next, m1);
    }
    // a line the student types is true for the buttons' jobs NOW: kept here, it is judged against them (checkWords)
    const typed = 'words' in patch || 'banner' in patch;
    const had = new Set(linesOf(old).map(([l]) => l));
    const page = pageLines(next, m1);
    const jobs = jobsOf(state);
    const wordsFor = {};
    for (const [l] of linesOf(next)) {
        const w = (old.wordsFor || {})[l] || (typed && !had.has(l) && !page.has(l) ? jobs : undefined);
        if (w) {
            wordsFor[l] = w;
        }
    }
    next.wordsFor = Object.keys(wordsFor).length ? wordsFor : undefined;
    return { ...state, list: state.list.map((c, k) => (k === i ? clean(next) : c)) };
}

/** Control `i`'s words back to the page's: all of them, or only the `stale` parts checkWords named. */
export function pageWords(state, i, robot, stale) {
    const c = state.list[i];
    const m = mechOf(robot, c.mechanism);
    const rows = defaultBanner(c, m);
    const patch = {};
    if (!stale || stale.comment) {
        patch.words = defaultWords(c, m);
    }
    if (!stale || stale.rows.length) {
        const byKey = new Map(rows.map((r) => [r.key, r]));
        patch.banner = stale && stale.rows.every((k) => byKey.has(c.banner[k].key))
            ? c.banner.map((r, k) => (stale.rows.includes(k) ? byKey.get(r.key) : r)) : rows;
    }
    return editControl(state, i, patch, robot);
}

/** The buttons a text names ("R1 / R2 wait", "press A"): the twelve names as words ("A held" is English, not A). */
export function buttonsNamed(text) {
    return [...new Set([...String(text).matchAll(/(?<![\w-])(L1|L2|R1|R2|Up|Down|Left|Right|X|B|Y|A(?! [a-z]))(?![\w-])/g)].map((m) => m[1]))];
}

/** Each button's mechanism and its role there (up/down, forward/back): what a kept word may now misstate. */
function jobsOf(state) {
    const out = {};
    for (const c of state.list) {
        buttonsOf(c).forEach((b, k) => { out[b] = { mech: c.mechanism, role: JSON.stringify([c.form, k, c.back === undefined]) }; });
    }
    return out;
}

/**
 * Words that may now be false (D9: the map never disagrees with the code). Each line is judged against the jobs it
 * was written for: a line typed here, the jobs when it was typed; the file's own, `before` (the driver file as
 * read); any other (a project's), `opened`. A line naming one of its control's own buttons whose role changed, or
 * another button whose mechanism changed, is an error, fixed by the page's words; a note naming such a button, or a
 * mechanism whose controls changed, a warning (the team's prose: they check it). The page's own lines are true.
 */
export function checkWords(state, before, robot, opened = before) {
    if (!before) {
        return [];
    }
    const now = jobsOf(state);
    const was = jobsOf(before);
    const fromOpened = jobsOf(opened || before);
    const fileLines = new Set(before.list.flatMap((c) => linesOf(c).map(([l]) => l)));
    const same = (a, b) => JSON.stringify(a || {}) === JSON.stringify(b || {});
    const elsewhere = new Set(BUTTONS.filter((b) => (now[b] || {}).mech !== (was[b] || {}).mech));
    const jobs = (s, id) => JSON.stringify(s.list.filter((c) => c.mechanism === id).map((c) => JOB.map((k) => c[k])));
    const changedMechs = (robot.mechanisms || []).filter((m) => jobs(state, m.ident) !== jobs(before, m.ident));
    const out = [];
    state.list.forEach((c, i) => {
        const m = mechOf(robot, c.mechanism);
        const mine = new Set(buttonsOf(c));
        const page = pageLines(c, m);
        const named = new Set();
        const stale = { comment: false, rows: [] };
        for (const [l, k] of linesOf(c)) {
            const base = (c.wordsFor || {})[l] || (fileLines.has(l) ? was : fromOpened);
            const bad = page.has(l) ? [] : buttonsNamed(l).filter((b) => (mine.has(b) ? !same(now[b], base[b]) : !same((now[b] || {}).mech, (base[b] || {}).mech)));
            bad.forEach((b) => named.add(b));
            if (bad.length && k < 0) {
                stale.comment = true;
            } else if (bad.length && !stale.rows.includes(k)) {
                stale.rows.push(k);
            }
        }
        if (named.size) {
            const where = [stale.comment ? 'its comment' : '', ...stale.rows.map((k) => `its map row "${c.banner[k].key}"`)].filter(Boolean).join(' and ');
            out.push({ level: 'error', code: 'wordsStale', at: i, mechanism: c.mechanism, ...stale,
                       words: `${m ? `The ${m.name}'s` : 'This'} control: ${where} ${stale.rows.length + (stale.comment ? 1 : 0) > 1 ? 'name' : 'names'} ${[...named].join(', ')}, which now ${named.size > 1 ? 'do' : 'does'} something else. Use the page's words there, or change them below.` });
        }
    });
    const b = state.banner || {};
    const notes = [...(state.notes || []), ...(b.notes || []).flatMap((r) => r.lines)];
    for (const line of notes) {
        const named = buttonsNamed(line).filter((x) => elsewhere.has(x));
        const mech = changedMechs.find((m) => [m.ident, m.name].some((n) => new RegExp(`\\b${n.replace(/[^\w ]/g, '\\$&')}\\b`, 'i').test(line)));
        if (named.length || mech) {
            out.push({ level: 'warn', code: 'noteStale', at: -1,
                       words: `A note in the file speaks of ${named.length ? named.join(', ') : `the ${mech.name}`}, whose job changed here: "${line.trim()}". Check it in the file after you write, and correct it by hand if it no longer holds.` });
        }
    }
    return out;
}

/** Control `i` removed. */
export const removeControl = (state, i) => ({ ...state, list: state.list.filter((_, k) => k !== i) });

/** The map's driving rows after the feel changed from `was` to `now` (sticks, re-zero, release follow it). */
export function followDriving(state, was, now) {
    const b = { ...(state.banner || {}) };
    const mode = (d) => (d && d.mode) || 'arcade';
    if (!b.sticks || !was || mode(was) !== mode(now)) {
        b.sticks = sticksRows(now.mode);
    }
    if (!b.rezero || !was || was.rezeroButton !== now.rezeroButton || mode(was) !== mode(now)) {
        b.rezero = rezeroRows(now.rezeroButton, now.mode);
    }
    if (!b.release || !was || was.onRelease !== now.onRelease) {
        b.release = releaseRows(now.onRelease);
    }
    return { ...state, banner: b };
}

/** The map's own problems: a "free" line naming a button a control (or the re-zero) now takes (the map would lie). */
export function checkMap(state, rezero) {
    const taken = new Set([...state.list.flatMap(buttonsOf), ...(rezero ? [rezero] : [])]);
    const out = [];
    ((state.banner && state.banner.free) || []).forEach((r, k) => {
        const clash = keyButtons(r.key).filter((b) => taken.has(b));
        if (clash.length) {
            out.push({ level: 'error', code: 'mapStale', free: k, at: -1, button: clash[0],
                       words: `The button map in the file's header says "${r.key}: ${r.lines[0]}", but ${clash.join(', ')} now ${clash.length > 1 ? 'have jobs' : 'has a job'}. Remove that line (you can write it again by hand afterwards).` });
        }
    });
    return out;
}

/** The free line `k` removed from the map. */
export const removeFreeLine = (state, k) => ({ ...state, banner: { ...state.banner, free: state.banner.free.filter((_, j) => j !== k) } });
