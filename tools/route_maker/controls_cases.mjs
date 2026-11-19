// controls_cases.mjs — RMP1-B's controls AGREEMENT data (the brief's CT1–CT31, R15): on the suite's fixture robots
// (every kind, every homing method — robots one and two have too few kinds to reach every refusal), each
// configuration the page refuses, and the C++ the page would write for it. goldens.mjs --write turns each into a
// compile-refusal TU (test/compile_refusals/rm_ct_*.cpp) whose `// expect:` is the page's refusal: the build must
// refuse it with THAT name. The CT21 goldens are configurations every rule passes, compiled through DriverFile by
// the suite (test/goldens/route_maker/controls_*.hpp). Test data only: no map for the team (G4).

import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { exportRegions, readDriverFile } from './src/model/driver_file.mjs';
import { BUTTONS } from './src/model/controls.mjs';

const GOLDENS = path.join(path.dirname(fileURLToPath(import.meta.url)), '..', '..', 'test', 'goldens', 'route_maker');
const robot = (id) => JSON.parse(fs.readFileSync(path.join(GOLDENS, 'robots', `${id}.json`), 'utf8'));

/** A never-homed arm with no unhomed volts (no fixture robot has one): its description and its C++. */
const W = {
    id: 'w', name: 'robot w (test)',
    mechanisms: [{ name: 'nh', ident: 'nh', kind: 'arm', units: 'deg', homing: 'never', range: [0, 160], unhomedVolts: 0, presets: [] }],
    preamble: ['inline constexpr Arm nh{.name = "nh", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold,',
               '                       .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160},',
               '                       .gravity = noGravity, .home = neverHomed};',
               'inline constexpr MechanismTable kRobotWMechanisms = mechanisms(wire(nh, motors(1)));'],
};
/** Robot one's mechanisms as its description says them (src/robots/mechanism_table.hpp, ROB1-C2): the CT21 golden
 *  on robot one compiles against the REAL declarations, so a drift here fails the suite. No levels: none are typed. */
const ONE = {
    id: 'one', name: 'robot one',
    mechanisms: [{ name: 'cascade', ident: 'cascade', kind: 'lift', units: 'in', homing: 'stall', unhomedVolts: 4, presets: [] },
                 { name: 'right toggle', ident: 'rightToggle', kind: 'power' }, { name: 'left toggle', ident: 'leftToggle', kind: 'power' },
                 { name: 'claw', ident: 'claw', kind: 'pneumatic', positions: ['closed', 'open'] },
                 { name: 'flipper', ident: 'flipper', kind: 'pneumatic', positions: ['closed', 'open'] },
                 { name: 'ramp', ident: 'ramp', kind: 'pneumatic', positions: ['up', 'down'] }],
};
let robots;
export const ROBOTS = () => (robots = robots || { x: robot('x'), y: robot('y'), w: W, one: ONE });  // read once
const NS = { x: 'mech_fixture::robot_x', y: 'mech_fixture::robot_y', w: 'mech_fixture', one: 'shulib::bench::robot_one' };
export const TABLE = { x: 'kRobotXMechanisms', y: 'kRobotYMechanisms', w: 'kRobotWMechanisms', one: 'kRobotOneMechanisms' };

const hold = (mechanism, button, back, volts) => ({ form: 'whileHeld', mechanism, button, ...(back ? { back } : {}), volts });
const preset = (mechanism, p, button, timeLimitS) => ({ form: 'preset', mechanism, preset: p, button, ...(timeLimitS !== undefined ? { timeLimitS } : {}) });
const toggle = (mechanism, button, speed) => ({ form: 'toggle', mechanism, button, ...(speed !== undefined ? { speed } : {}) });
const homeAtStart = (mechanism, timeLimitS) => ({ form: 'homeAtDriverStart', mechanism, timeLimitS });
const homeOnPress = (mechanism, button, timeLimitS) => ({ form: 'homeOnPress', mechanism, button, timeLimitS });
const autonOnly = (mechanism) => ({ form: 'autonOnly', mechanism });

const BASE_X = () => [hold('arm', 'R1', 'R2', 6), preset('arm', 'score', 'Right', 1.5), hold('lift', 'L1', 'L2', 12),
    homeAtStart('lift', 1.5), toggle('flywheel', 'X', 'shoot'), hold('intake', 'A', 'B', 12), autonOnly('hood')];
const BASE_Y = () => [hold('wrist', 'R1', 'R2', 6), hold('swing', 'L1', 'L2', 6), homeAtStart('swing', 1.5),
    preset('swing', 'up', 'Up', 1.5), preset('dr4b', 'low', 'Down', 2), hold('tilt', 'X', 'B', 3), hold('roller', 'A', undefined, 8),
    autonOnly('catapult'), toggle('grip', 'Left')];
const FEEL = { mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' };

const swap = (list, i, c) => list.map((x, k) => (k === i ? c : x));
const without = (list, i) => list.filter((_, k) => k !== i);

/** Every refusal case: {name, robot, list, driving?, expect}. */
export function cases() {
    const out = [
        ['hold_on_a_pneumatic', 'x', swap(BASE_X(), 6, hold('hood', 'Up', 'Down', 6)), 'whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset'],
        ['one_button_hold_on_an_arm', 'x', swap(BASE_X(), 0, hold('arm', 'R1', undefined, 6)), 'whileHeld_with_one_button_is_for_a_power_mechanism_an_arm_or_a_lift_needs_two'],
        ['flywheel_toggle_without_its_speed', 'x', swap(BASE_X(), 4, toggle('flywheel', 'X')), 'a_toggle_on_a_flywheel_names_its_speed_like_toggle_flywheel_shoot'],
        ['toggle_on_a_power', 'x', swap(BASE_X(), 5, toggle('intake', 'A')), 'a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed'],
        ['home_on_press_on_a_power', 'x', [...BASE_X(), homeOnPress('intake', 'Up', 1)], 'homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift'],
        ['another_robots_mechanism', 'x', [...BASE_X(), toggle('claw', 'Up')], 'a_control_names_a_mechanism_this_robot_does_not_have'],
        ['hold_at_12_5_volts', 'x', swap(BASE_X(), 0, hold('arm', 'R1', 'R2', 12.5)), 'a_hold_needs_volts_above_0_and_at_most_12'],
        ['hold_at_0_volts', 'x', swap(BASE_X(), 5, hold('intake', 'A', 'B', 0)), 'a_hold_needs_volts_above_0_and_at_most_12'],
        ['preset_with_no_time_limit', 'x', swap(BASE_X(), 1, preset('arm', 'score', 'Right', 0)), 'a_preset_or_a_homing_needs_a_time_limit_above_zero'],
        ['home_at_start_with_no_time_limit', 'x', swap(BASE_X(), 3, homeAtStart('lift', 0)), 'a_preset_or_a_homing_needs_a_time_limit_above_zero'],
        ['two_holds_on_one_arm', 'x', [...BASE_X(), hold('arm', 'Up', 'Down', 3)], 'two_controls_of_the_same_kind_on_one_mechanism'],
        ['two_presets_to_one_level', 'x', [...BASE_X(), preset('arm', 'score', 'Up', 1)], 'two_controls_of_the_same_kind_on_one_mechanism'],
        ['toggle_on_three_positions', 'x', swap(BASE_X(), 6, toggle('hood', 'Up')), 'a_toggle_needs_a_two_position_pneumatic_or_a_flywheel_named_speed'],
        ['home_button_on_a_never_homed', 'y', [...BASE_Y(), homeOnPress('tilt', 'Right', 1)], 'a_homing_control_on_a_mechanism_that_is_never_homed'],
        ['a_mechanism_with_no_control', 'x', without(BASE_X(), 6), 'a_mechanism_has_no_driver_control_mark_it_autonOnly_if_that_is_meant'],
        ['autons_only_with_a_control', 'x', [...BASE_X(), preset('hood', 'up', 'Up')], 'a_mechanism_marked_autonOnly_also_has_a_control'],
        ['more_than_24', 'x', [...BASE_X(), ...Array.from({ length: 18 }, () => autonOnly('hood'))], 'more_controls_than_fit'],
        ['switch_homed_lift_no_way_to_home', 'x', without(BASE_X(), 3), 'a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control'],
        ['stall_homed_swing_level_no_home', 'y', without(BASE_Y(), 2), 'a_mechanism_that_must_be_homed_has_no_way_to_home_in_driver_control'],
        ['never_homed_no_unhomed_volts', 'w', [hold('nh', 'R1', 'R2', 6)], 'a_never_homed_mechanism_needs_unhomedVolts_for_its_buttons_to_move_it'],
        ['level_on_a_never_homed', 'y', [...BASE_Y(), preset('tilt', 'mid', 'Right', 1)], 'a_preset_on_a_mechanism_that_is_never_homed'],
        ['no_such_level_on_an_arm', 'x', [...BASE_X(), preset('arm', 'hihg', 'Up', 1)], 'no_such_preset_on_this_mechanism'],
        ['no_such_level_on_a_lift', 'x', [...BASE_X(), preset('lift', 'hihg', 'Up', 1)], 'no_such_preset_on_this_mechanism'],
        ['no_such_flywheel_speed', 'x', swap(BASE_X(), 4, toggle('flywheel', 'X', 'shot')), 'no_such_preset_on_this_mechanism'],
        ['no_such_pneumatic_position', 'y', [...BASE_Y(), preset('grip', 'tight', 'Right')], 'no_such_position_on_this_mechanism'],
        ['preset_on_a_power_mechanism', 'x', [...BASE_X(), preset('intake', 'fast', 'Up', 1)], 'a_power_mechanism_has_no_presets_run_it_with_r_run'],
        ['a_control_on_the_rezero_button', 'x', BASE_X(), "a mechanism's button is the loop's re-zero button", { ...FEEL, rezeroButton: 'A' }],
        ['a_feel_out_of_range', 'x', BASE_X(), 'the feel is out of range', { ...FEEL, forward: { curve: 1.5 } }],
        // each refusal's OTHER conditions (the review: one case per name let a page rule drop a condition unseen)
        ['two_button_hold_on_a_flywheel', 'x', swap(BASE_X(), 4, hold('flywheel', 'X', 'Up', 6)), 'whileHeld_is_for_a_power_mechanism_an_arm_or_a_lift_use_toggle_or_preset'],
        ['home_at_start_on_a_power', 'x', [...BASE_X(), homeAtStart('intake', 1)], 'homeOnPress_and_homeAtDriverStart_are_for_an_arm_or_a_lift'],
        ['toggle_on_an_arm', 'x', [...BASE_X(), toggle('arm', 'Up')], 'a_toggle_is_for_a_two_position_pneumatic_or_a_flywheel_speed'],
        ['two_presets_to_one_position', 'y', swap([...BASE_Y(), preset('grip', 'shut', 'Right')], 8, preset('grip', 'shut', 'Left')), 'two_controls_of_the_same_kind_on_one_mechanism'],
        ['rezero_on_a_holds_back_button', 'x', BASE_X(), "a mechanism's button is the loop's re-zero button", { ...FEEL, rezeroButton: 'B' }],
        ['hold_at_12_001_volts', 'x', swap(BASE_X(), 5, hold('intake', 'A', 'B', 12.001)), 'a_hold_needs_volts_above_0_and_at_most_12'],
        ['exactly_24_two_of_a_kind', 'x', [...BASE_X(), ...Array.from({ length: 17 }, () => autonOnly('hood'))], 'two_controls_of_the_same_kind_on_one_mechanism'],
        ['a_turn_curve_out_of_range', 'x', BASE_X(), 'the feel is out of range', { ...FEEL, turn: { curve: 1.5 } }],
        ['a_strafe_dead_zone_out_of_range', 'x', BASE_X(), 'the feel is out of range', { ...FEEL, strafe: { deadband: 1 } }],
    ];
    // a button doing two jobs, each of the twelve (the base's own button again, or two levels on a free one)
    for (const b of BUTTONS) {
        const base = BASE_X();
        const taken = base.some((c) => c.button === b || c.back === b);
        const list = taken ? [...base, preset('arm', 'high', b, 1)] : [...base, preset('arm', 'high', b, 1), preset('arm', 'stow', b, 1)];
        out.push([`button_${b}_twice`, 'x', list, `button_${b}_does_two_jobs`, { ...FEEL, rezeroButton: b === 'Y' ? 'Up' : 'Y' }]);
    }
    return out.map(([name, rid, list, expect, driving]) => ({ name, robot: rid, list, expect, driving: driving || FEEL }));
}

/** The configurations every rule passes (CT21): compiled through DriverFile by the suite. */
export function goldens() {
    return [
        { name: 'x_every_form', robot: 'x', list: [...BASE_X(), preset('flywheel', 'shoot', 'Up', 2), preset('lift', 'high', 'Down', 2.5)],
          driving: { mode: 'tankSticks', forward: { deadband: 0.08, curve: 0.6 }, turn: { curve: 0.3 }, rampPerSecond: 4, rezeroButton: 'Left', onRelease: 'brake' } },
        { name: 'y_homes_and_holds', robot: 'y', list: [...BASE_Y(), homeOnPress('swing', 'Right', 1)],
          driving: { mode: 'arcadeField', turn: { deadband: 0.1 }, rezeroButton: 'Y', onRelease: 'hold' } },
        { name: 'y_pneumatic_positions', robot: 'y', list: swap([...BASE_Y(), preset('grip', 'loose', 'Right')], 8, preset('grip', 'shut', 'Left')),
          driving: { mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' } },
        // robot one's REAL declarations: every form its kinds take, 12 V exactly, and a stall-homed cascade with unhomed
        // volts held with no home (passes: the refusal cases' boundaries, from the passing side)
        { name: 'one_every_form', robot: 'one', list: [hold('cascade', 'R1', 'R2', 12), homeOnPress('cascade', 'Up', 3), homeAtStart('cascade', 3),
            preset('claw', 'open', 'A'), preset('claw', 'closed', 'B'), toggle('flipper', 'X'), preset('ramp', 'down', 'Left'),
            preset('ramp', 'up', 'Right'), hold('rightToggle', 'L1', 'L2', 6), autonOnly('leftToggle')],
          driving: { mode: 'arcade', rezeroButton: 'Y', onRelease: 'brake' } },
        { name: 'one_holds_only', robot: 'one', list: [hold('cascade', 'R1', 'R2', 12), toggle('claw', 'A'), toggle('flipper', 'B'), toggle('ramp', 'X'),
            hold('rightToggle', 'L1', undefined, 12), hold('leftToggle', 'L2', undefined, 12)],
          driving: { mode: 'arcade', rezeroButton: 'Y', onRelease: 'coast' } },
        // the builder's map as typed on 2026-11-15 (the marked fixture): guide 21 quotes it from here, so a team export
        // never makes that chapter stale
        { name: 'one_builders_map', robot: 'one', ...builders('one') },
    ];
}
/** A marked fixture's controls and feel, exactly as the file says them. */
function builders(id) {
    const r = readDriverFile(fs.readFileSync(path.join(GOLDENS, `driver_${id}_marked.hpp`), 'utf8'), id);
    return { list: r.state.controls.list, controls: r.state.controls, driving: r.state.driving };
}

/** A case's state for the page (the controls tab's {controls, driving}), made only from the case. */
export const stateOf = (k) => ({ controls: { list: k.list, table: TABLE[k.robot] }, driving: k.driving });

/** The C++ a case writes, inside its own namespace (and the robot's), with DriverFile instantiated. */
export function translationUnit(k, header) {
    const r = ROBOTS()[k.robot];
    const regions = exportRegions(stateOf(k), r);
    const ns = `${NS[k.robot]}::rm_${k.name}`;
    const extra = k.list.some((c) => c.mechanism === 'claw') ? ['using mech_fixture::robot_two::claw;  // another robot\'s'] : [];
    return [...header, '#include "refusal_common.hpp"', '#include "common/driver_file.hpp"',
            `namespace ${ns} {`, 'using namespace shulib::mechanism::words;', ...(r.preamble || []), ...extra,
            ...regions.controls, ...regions['control words'], ...regions.feel,
            'inline shulib::bench::DriverFile<kControls, kFeel, kLoop> driver{};', `}  // namespace ${ns}`].join('\n') + '\n';
}
