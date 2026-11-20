// library_words.mjs — the LIBRARY'S OWN SENTENCES the page shows beside its plain words (RMP1 brief §1.1's
// `refusals.json`, landmine 14 — kept as one module checked against the headers, not a generated file).
//
// Each entry is the exact text of a refusal the library raises (a SHULIB_PRECONDITION's sentence, a
// checkAutonEntry refusal, a log line) for a route the page refuses first. test/library_words.test.mjs reads the
// C++ headers named here, joins their adjacent string literals, and fails when a sentence is no longer there
// word for word — so the day the library's words change, the page's test goes red until they follow.
//
// Two other shapes:
//   * a BUILD refusal (R9): a name the robot's code cannot have (`lift["hihg"]`) never reaches run time — the
//     declaration's consteval `operator[]` calls a function that is not constexpr and GCC stops the build
//     naming it. Its entry carries `ident` (the function, defined in `file`) and `twin` (the compile-refusal
//     fixture whose `// expect:` line is that name: the suite proves the build does stop there);
//   * a sentence the library BUILDS from pieces (RC24's log line: three `appendLiteral`s with the numbers
//     between): one entry per piece, each pinned on its own, and the page joins them (tankNotSteeredWords).
//
// Pure data (and the one joiner).

/** {key: {file, text}} — `file` the header (from the repository root) that raises `text`. */
export const LIBRARY_WORDS = Object.freeze({
    waitNeverStarted: {
        file: 'include/shulib/chassis/routine.hpp',
        text: 'r.waitUntilDone(...): this routine started no step on that mechanism -- start one first '
            + '(r.startRun / r.startMove / r.startSpinUp / r.startHome)',
    },
    waitBound: {
        file: 'include/shulib/chassis/routine.hpp',
        text: 'r.waitUntilDone(...): the time limit must be finite and above 0',
    },
    noBearing: {
        file: 'include/shulib/chassis/routine.hpp',
        text: 'Routine: face/driveTo/faceAway/reverseTo target is (numerically) where the robot already is — no defined bearing',
    },
    labelEmpty: {
        file: 'include/shulib/chassis/routine.hpp',
        text: 'r.label(...): a label needs a name -- r.label("third piece")',
    },
    skipEmpty: {
        file: 'include/shulib/chassis/routine.hpp',
        text: 'r.onFail(...): a skip needs the name of the step to skip to -- write skipTo("third piece")',
    },
    moveBound: {
        file: 'include/shulib/mechanism/service.hpp',
        text: 'r.move(...): the time limit must be finite and above 0',
    },
    homeBound: {
        file: 'include/shulib/mechanism/service.hpp',
        text: 'r.home(...): the time limit must be finite and above 0',
    },
    homeNever: {
        file: 'include/shulib/mechanism/service.hpp',
        text: 'r.home(...): the mechanism is declared neverHomed -- it has no way to find its zero '
            + '(give it .home = homeBySwitch / homeByStall / restsAtStop / absoluteSensor)',
    },
    runBound: {
        file: 'include/shulib/mechanism/service.hpp',
        text: 'r.run(...): the time limit must be finite and above 0',
    },
    runVolts: {
        file: 'include/shulib/mechanism/service.hpp',
        text: 'r.run(...): the volts must be a finite number',
    },
    noPreset: {
        file: 'include/shulib/mechanism/kinds.hpp',
        ident: 'no_such_preset_on_this_mechanism',
        twin: 'test/compile_refusals/k_misspelt_lift_preset.cpp',
        text: 'the build stops: no_such_preset_on_this_mechanism',
    },
    noPosition: {
        file: 'include/shulib/mechanism/kinds.hpp',
        ident: 'no_such_position_on_this_mechanism',
        twin: 'test/compile_refusals/n23_misspelt_position.cpp',
        text: 'the build stops: no_such_position_on_this_mechanism',
    },
    travelLimits: {
        file: 'include/shulib/mechanism/kinds.hpp',
        ident: 'target_is_outside_the_travel_limits',
        twin: 'test/compile_refusals/k_lift_past_top.cpp',
        text: 'the build stops: target_is_outside_the_travel_limits',
    },
    guardPause: {
        file: 'include/shulib/sequence/run_guard.hpp',
        text: 'RunGuard::pause: duration must be finite and > 0',
    },
    autonName: {
        file: 'include/shulib/competition/auton_registry.hpp',
        text: "the name must be 1..29 printable ASCII characters with no percent sign (it is shown as 'AUTON: <name>' on one screen line)",
    },
    autonTaken: {
        file: 'include/shulib/competition/auton_registry.hpp',
        text: 'the name is already registered',
    },
    autonNone: {
        file: 'include/shulib/competition/auton_registry.hpp',
        text: "NONE is the selector's own choice -- pick another name",
    },
    // RC24: the motion layer's Warn line, `tank moveTo not steered: it starts <off> deg off its target heading
    // (steers within <max> deg) -- turn first: face()` — three literals with the two numbers between.
    tankNotSteered1: {
        file: 'include/shulib/motion/move_to_pose.hpp',
        text: 'tank moveTo not steered: it starts ',
    },
    tankNotSteered2: {
        file: 'include/shulib/motion/move_to_pose.hpp',
        text: ' deg off its target heading (steers within ',
    },
    tankNotSteered3: {
        file: 'include/shulib/motion/move_to_pose.hpp',
        text: ' deg) -- turn first: face()',
    },
});

/** RC24's library line for a leg `offDeg` off its line on a robot that steers within `maxSteerDeg` (both printed
 *  with one decimal, as `appendNum(line, v, 1, 1)` prints them). */
export function tankNotSteeredWords(offDeg, maxSteerDeg) {
    return `${libraryWords('tankNotSteered1')}${Number(offDeg).toFixed(1)}${libraryWords('tankNotSteered2')}`
        + `${Number(maxSteerDeg).toFixed(1)}${libraryWords('tankNotSteered3')}`;
}

/** The library's sentence for `key` ('' for an unknown key, which the test refuses). */
export function libraryWords(key) {
    return LIBRARY_WORDS[key] ? LIBRARY_WORDS[key].text : '';
}
