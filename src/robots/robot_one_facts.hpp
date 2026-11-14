#pragma once
//
// robot_one_facts.hpp — ROBOT ONE'S FACTS LEDGER (chunk ROB1-A, 2026-11-11; the ROB1 brief's D3 (a),
// re-check R2, R4, R9, R10, R11). Every number about robot one — the 2026 CASCADE robot, built ~Nov 14 —
// that nobody has measured yet is written ONCE, HERE, and robot one's declarations
// (src/robots/mechanism_table.hpp's robot_one region) read ONLY these. Nothing about robot one exists in
// this project's records: no census, no reading, no photo. NOTHING HERE HAS RUN ON A ROBOT.
//
// ═══ WHAT "OWED" MEANS ════════════════════════════════════════════════════════════════
// A fact is written as
//     owed(<placeholder>, Stage::A, "<what it is, in plain words>")
// — a PLACEHOLDER: a number chosen only so that every rule the library checks passes (stall below the
// cap, rows increasing, levels inside the range, the homing push DOWN — landmine 7), so the HOST suite
// builds and runs robot one's cascade on the host test plant before the robot exists. It is NOT a
// measurement and must never reach a motor: under each fact a check (ROBOT_ONE_CHECK, below) FAILS THE
// BUILD of every robot-one program that needs it, naming it —
//     static assertion failed: robot one: OWED -- the cascade's motor ports (src/robots/robot_one_facts.hpp)
// — so `make ROBOT=one PROGRAM=...` refuses, one line per owed fact, and (the Makefile's variant stamp)
// leaves no OTHER robot's or program's package behind in bin/ to upload. (The one exception, the brief's
// D3: a rebuild of the SAME robot and program that once linked — every fact it needs typed — and is
// refused now leaves that last good package in bin/; it carries no placeholder.) Every other build —
// robot two's, the bench bot's, the host suite — is untouched by an OWED fact; a TYPED value that breaks
// a library rule (a level above the top, rows out of order) fails every build, robot two's too, because
// robot one's region compiles everywhere: the gate's fourteen builds are re-run after each typing.
//
// ═══ THE STAGES — which program needs a fact (needs(), below) ════════════════════════════
//   Stage::A        "what is plugged where": the cascade's ports, directions (as the builder sees them;
//                   Bench Tests' station 13 then confirms each), cartridge, gear teeth, the STARTING
//                   stall signature and current cap (CHOSEN, not measured — [R2]: station 13 cuts out on
//                   the signature, so they cannot wait for its record), every non-drive motor port the
//                   brain's Devices screen lists, and the flipper's KIND. Needed by ALL SIX programs —
//                   Bench Tests included, because it powers declared mechanisms.
//   Stage::B        "how it moves": the cascade's measured rows (bottom, every level, top), its levels,
//                   its hold volts, its homing push, its volts before it is homed. Needed by EVERY
//                   program but two (needsStageB(), a case per program): Bench Tests and Calibrate must
//                   build on Stage A alone, because Bench Tests is the tool that MEASURES Stage B
//                   (landmine 5). Today that leaves Drive, the Auton Tester, Competition and shulib Tune
//                   (TUNE1-B1's, classified at ROB1-A's integration onto it: it homes the cascade with the
//                   homing push, jogs it at the unhomed volts and moves it to its first and last level); a
//                   program added later is classified there or the build fails.
//   Stage::Recorded written down, gating NO build (the camera: VIS1-B reads it — D12, [R9]).
// The drivetrain needs no ledger: src/robots/chassis_table.hpp's kRobotOneChassis leaves each unknown
// field UNSET, and every station and program that needs one refuses by name; Calibrate discovers the
// drive's ports, directions, sides, scale and track width itself.
//
// ═══ HOW A FACT IS TYPED (the coordinator, from the builder's form — docs/robot-facts-form.md; G4) ════
// When the builder sends a fact, `owed(` becomes `typed(` with the placeholder KEPT and the real value in
// its own place, and WHO measured it, HOW and WHEN:
//     typed(<placeholder, unchanged>, <the value>, Stage::A, "<what>", "<who, how, when>")
// The value is stated SEPARATELY from the placeholder ([R10]): flipping `owed(` to `typed(` without
// writing a value does not compile, and a typed value EQUAL to its placeholder is refused unless the
// source says "same as the placeholder, checked" (the builder confirmed it). A typed fact needs a
// non-empty source, and never one containing "gate copy" ([R4]: that marker belongs only to the src build
// gate's throwaway copy, below). A value chosen at the bench is typed with the source "chosen, not
// measured -- <who>, <when>". A fact is NEVER "corrected" to match a belief, and NEVER invented to unblock
// a build (landmine 17). What is NOT a ledger edit: a third or fourth cascade motor, a flipper block —
// those are DECLARATION edits in src/robots/mechanism_table.hpp (the motor count is structural). A
// flipper block's NUMBERS are not: THIS LEDGER HOLDS NO FLIPPER NUMBER YET (only its kind), so before a
// block is pasted each of its numbers (port, cartridge, teeth, stall, cap, range, angles — or a
// pneumatic's line and stroke time) becomes an owed fact here with its check and its stage, and the
// block reads them as `facts::…`; test/robot_one_facts_test.cpp's region scan fails the host suite on
// any literal number or bare word left in robot one's region (`make` itself does not run that scan).
//
// ═══ THE SRC BUILD GATE'S PROOF ([R4]) ════════════════════════════════════════════════
// tools/src_build_gate.py proves both halves before the robot exists: on the REAL tree each of robot
// one's six builds links with nothing it needs owed, or fails naming EXACTLY its owed facts; and in a
// COPY of the working tree under build/src-gate-facts/ (never this file) every `owed(p, ...)` is typed
// as `typed(p, p, ..., "gate copy -- same as the placeholder, checked")`, the gate writes the marker file
// `build/ROBOT-ONE-GATE-COPY-ONLY` at the COPY's root (which turns kRefuseGateCopySource below off — IN
// THE COPY ONLY), and all six builds must link and pass every check robot two's do. A ledger copied back
// from there cannot link: the marker stays behind in the copy, so this file refuses the "gate copy"
// source again — in EVERY build, robot two's too.
//
// ═══ THE FORM OF EVERY CHECK ═════════════════════════════════════════════════════════
// One ROBOT_ONE_CHECK(fact, "<its name, verbatim>") below each fact: plain static_asserts, NEVER a
// compile-time-only (immediate) function (the brief's probe P1: one is evaluated even where `||` short-
// circuits, so it fails EVERY build, and GCC then names the function, not the fact — landmines 2, 3).
// The macro holds the only four static_asserts in this file; test/robot_one_facts_test.cpp proves every
// fact has exactly one check below it, naming it verbatim, and that kRobotOneFacts lists every fact.
//
// ═══ ITS NAMES ([R11]) ════════════════════════════════════════════════════════════════
// This file opens NO namespace called `robot_one`: the src build gate's per-robot rule derives robot
// namespaces from the mechanism table file and fails any other src/ file that names one. Its facts live
// in shulib::bench::robot_one_facts (a different identifier), and robot one's region reads them as
// `facts::kCascadePorts.value()` (an alias the region declares).
//
// PROS-FREE ON PURPOSE: the host suite includes it (through the mechanism table) from src/.

#include <cstddef>
#include <type_traits>

#include "build_selection.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench::robot_one_facts {

using namespace units::literals;

// ═══ THE MACHINERY ══════════════════════════════════════════════════════════════════

/// Which programs need a fact (file banner, THE STAGES).
enum class Stage {
    A,         ///< what is plugged where — every program, Bench Tests included
    B,         ///< how it moves — every program but Bench Tests and Calibrate (needsStageB())
    Recorded,  ///< written down, needed by no program (the camera — VIS1-B reads it)
};

/// True when program `p` needs Stage B's facts — "how it moves" (file banner). A case per PROGRAM and no
/// default (the review of ROB1-A): a program added to src/build_selection.hpp's `Program` without a case
/// here is -Wswitch — a host compile ERROR (-Werror) and a src build gate FAILURE — so nobody can add a
/// program that moves the cascade (TUNE1-B1's "shulib Tune" is one) and have it build on Stage A alone.
/// Only the two programs that MEASURE Stage B are false; every other is true, and so is a value nobody
/// handled (fail closed). tools/src_build_gate.py's STAGE_PROGRAMS restates this switch and its step 0
/// reads it back from this file, case by case, so the two cannot disagree.
constexpr bool needsStageB(Program p) {
    switch (p) {
        case Program::BenchTests:      // measures Stage B at station 13 (landmine 5)
        case Program::Calibrate:       // measures the drive; powers no declared mechanism
            return false;
        case Program::Drive:
        case Program::AutonTester:
        case Program::Competition:
        case Program::Tune:            // homes the cascade, jogs it unhomed, moves it to its first and last level
        case Program::InventedXDrive:  // never a robot-one build; classified so the switch is complete
            return true;
    }
    return true;  // a program nobody handled NEEDS Stage B (fail closed)
}

/// True when program `p` needs the facts of stage `s` (file banner). A case per stage, no default.
constexpr bool needs(Program p, Stage s) {
    switch (s) {
        case Stage::A:
            return true;
        case Stage::B:
            return needsStageB(p);
        case Stage::Recorded:
            return false;
    }
    return true;  // unreachable: a stage nobody handled is NEEDED (fail closed)
}

/// One fact: its placeholder and its typed value kept APART ([R10]), whether it is typed, the stage
/// that needs it, its name in plain words, and (typed) who measured it, how and when.
template <class T>
struct Fact {
    T placeholder;          ///< the number the host builds on while it is owed (file banner)
    bool typed;             ///< true once the builder's value is typed
    T typedValue;           ///< the builder's value (equal to the placeholder while owed)
    Stage needed;           ///< which programs need it
    const char* name;       ///< what it is, in plain words — quoted verbatim by its check
    const char* source;     ///< typed: who, how, when; owed: nullptr
    /// What the declarations read: the typed value once typed, else the placeholder.
    constexpr const T& value() const { return typed ? typedValue : placeholder; }
};

/// An OWED fact: a placeholder only (file banner).
template <class T>
constexpr Fact<T> owed(T placeholder, Stage needed, const char* name) {
    return Fact<T>{placeholder, false, placeholder, needed, name, nullptr};
}

/// A TYPED fact: the placeholder KEPT, the value in its own place, and its source (file banner).
template <class T>
constexpr Fact<T> typed(T placeholderKept, T value, Stage needed, const char* name, const char* source) {
    return Fact<T>{placeholderKept, true, value, needed, name, source};
}

/// `word` appears in `text` (both NUL-terminated; a null text contains nothing).
constexpr bool contains(const char* text, const char* word) {
    if (text == nullptr || word == nullptr) {
        return false;
    }
    for (; *text != '\0'; ++text) {
        const char* a = text;
        const char* b = word;
        while (*a != '\0' && *b != '\0' && *a == *b) {
            ++a;
            ++b;
        }
        if (*b == '\0') {
            return true;
        }
    }
    return false;
}

/// [R4] OFF ONLY in the src build gate's throwaway copy (file banner): a "gate copy" source is refused
/// at make in the repository's own ledger, so a ledger copied back from there can never link. The switch
/// is NOT in this file's text (the review of ROB1-A: a text switch the copy flipped came back WITH a
/// copied-back ledger, and that ledger linked): it is a MARKER FILE the gate writes at the copy's root,
/// `build/ROBOT-ONE-GATE-COPY-ONLY`, and only there — a path relative to THIS file's directory, so the
/// copy's ledger sees the copy's marker and the repository's ledger the repository's build/, where the
/// gate's step 0 refuses one. Copying the copy's ledger (or all of its src/) back brings no marker.
#if __has_include("../../build/ROBOT-ONE-GATE-COPY-ONLY")
inline constexpr bool kRefuseGateCopySource = false;
#else
inline constexpr bool kRefuseGateCopySource = true;
#endif

/// A typed fact's source is real: non-empty, and not the gate copy's marker (while it is refused).
constexpr bool realSource(const char* source) {
    return source != nullptr && *source != '\0' && !(kRefuseGateCopySource && contains(source, "gate copy"));
}

// The ledger's own field-wise equality ([R10]; the library's HeightAt has no ==), one per value type.
constexpr bool same(double a, double b) { return a == b; }
constexpr bool same(int a, int b) { return a == b; }
template <class E>
    requires std::is_enum_v<E>
constexpr bool same(E a, E b) {
    return a == b;
}
constexpr bool same(units::Length a, units::Length b) { return a.value() == b.value(); }
constexpr bool same(units::Time a, units::Time b) { return a.value() == b.value(); }
constexpr bool same(const char* a, const char* b) {
    if (a == nullptr || b == nullptr) {
        return a == b;
    }
    while (*a != '\0' && *a == *b) {
        ++a;
        ++b;
    }
    return *a == *b;
}
constexpr bool same(const mechanism::HeightAt& a, const mechanism::HeightAt& b) {
    return same(a.height, b.height) && same(a.degrees, b.degrees);
}
constexpr bool same(const mechanism::Motor& a, const mechanism::Motor& b) {
    return a.present == b.present && a.reversed == b.reversed;
}

// THE CHECK BELOW EVERY FACT (file banner, THE FORM OF EVERY CHECK). `words` is the fact's name,
// verbatim — the first assert proves it, so the OWED line names exactly the fact. The second is the
// refusal itself (P1b's form: robot one's build of a program that needs the fact, while it is owed).
// The third and fourth are [R4]/[R10]'s. Named ROBOT_ONE_CHECK — never SHULIB_ROBOT_*: that prefix is
// a build switch's, and the src build gate's step 0 reads it as one.
#define ROBOT_ONE_CHECK(fact, words)                                                                         \
    static_assert(::shulib::bench::robot_one_facts::same((fact).name, words),                                \
                  "robot one: the check below a fact must quote its name verbatim -- " words);               \
    static_assert(!(::shulib::bench::kRobotId == ::shulib::bench::RobotId::RobotOne &&                       \
                    ::shulib::bench::robot_one_facts::needs(::shulib::bench::kProgram, (fact).needed)) ||    \
                      (fact).typed,                                                                          \
                  "robot one: OWED -- " words " (src/robots/robot_one_facts.hpp)");                          \
    static_assert(!(fact).typed || ::shulib::bench::robot_one_facts::realSource((fact).source),             \
                  "robot one: a typed fact needs a real source (who, how, when; never 'gate copy') -- " words); \
    static_assert(!(fact).typed ||                                                                           \
                      !::shulib::bench::robot_one_facts::same((fact).typedValue, (fact).placeholder) ||       \
                      ::shulib::bench::robot_one_facts::contains((fact).source, "same as the placeholder, checked"), \
                  "robot one: TYPED but still the placeholder -- " words)

// ═══ THE VALUE TYPES (each with its same()) ══════════════════════════════════════════════

/// Smart ports 1..21 — the most a Devices screen can list.
inline constexpr std::size_t kMaxListedPorts = 21;

/// A list of smart ports, as read off the brain's Devices screen.
struct PortList {
    int port[kMaxListedPorts];  ///< the ports, any order (unused slots 0)
    int count;                  ///< how many are listed
};
constexpr bool same(const PortList& a, const PortList& b) {
    if (a.count != b.count) {
        return false;
    }
    for (std::size_t i = 0; i < kMaxListedPorts; ++i) {
        if (a.port[i] != b.port[i]) {
            return false;
        }
    }
    return true;
}

/// The cascade's two motors: one value per motor, in the declaration's order. (TWO is structural: a
/// third motor is a declaration edit in src/robots/mechanism_table.hpp, not a ledger edit.)
template <class T>
struct Pair {
    T first;   ///< motor 1
    T second;  ///< motor 2
};
template <class T>
constexpr bool same(const Pair<T>& a, const Pair<T>& b) {
    return same(a.first, b.first) && same(a.second, b.second);
}

/// A gear train's tooth counts, motor gear first.
struct Teeth {
    int motor;  ///< the gear on the motor
    int spool;  ///< the gear on the spool's shaft
};
constexpr bool same(const Teeth& a, const Teeth& b) { return a.motor == b.motor && a.spool == b.spool; }

/// The cascade's MEASURED rows: {height, the station 13 RECORD's output angle} at the bottom stop, at
/// every level and at the top stop, strictly increasing. FIVE is this ledger's count (the bottom, three
/// levels between, the top): a different count is typed by changing the 5 here (three to eight).
inline constexpr std::size_t kCascadeRowCount = 5;
struct Rows {
    mechanism::HeightAt row[kCascadeRowCount];  ///< bottom first, top last
};
constexpr bool same(const Rows& a, const Rows& b) {
    for (std::size_t i = 0; i < kCascadeRowCount; ++i) {
        if (!same(a.row[i], b.row[i])) {
            return false;
        }
    }
    return true;
}

/// One named level by stack height (a name of at most 15 letters; an unused slot {nullptr, 0_in}).
struct Level {
    const char* name;  ///< what the driver file and the autons call it
    units::Length at;  ///< its height (the same point the rows measure)
};
/// The six level slots (the library's kMaxPresets): the declaration spells all six ([R8]).
struct Levels {
    Level level[mechanism::kMaxPresets];  ///< in order; unused slots last
};
constexpr bool same(const Level& a, const Level& b) { return same(a.name, b.name) && same(a.at, b.at); }
constexpr bool same(const Levels& a, const Levels& b) {
    for (std::size_t i = 0; i < mechanism::kMaxPresets; ++i) {
        if (!same(a.level[i], b.level[i])) {
            return false;
        }
    }
    return true;
}

/// The flipper's kind (AF1-S's D7 — still owed by the builder). Whatever is typed, the build checks
/// that robot one's list agrees (src/robots/mechanism_table.hpp): an Arm or a Pneumatic named
/// "flipper" declared, or none at all.
enum class FlipperKind {
    NotOnTheRobotYet,  ///< no flipper on the robot yet — a valid answer
    MotorArm,          ///< a motor arm (an Arm declaration; its motor port is then a non-drive motor)
    Pneumatic,         ///< a pneumatic (a Pneumatic declaration on a 3-wire line)
    PassiveBar,        ///< a passive bar — nothing to declare
};

/// Where the AI Vision camera sits, from the robot's centre (midway between the wheels, both ways).
struct CameraMount {
    units::Length forward;  ///< ahead of the centre
    units::Length left;     ///< left of the centre
    units::Length height;   ///< above the floor
    double tiltDegrees;     ///< tilted down from level
};
constexpr bool same(const CameraMount& a, const CameraMount& b) {
    return same(a.forward, b.forward) && same(a.left, b.left) && same(a.height, b.height) &&
           same(a.tiltDegrees, b.tiltDegrees);
}

// ═══ THE FACTS ═════════════════════════════════════════════════════════════════════════
// Every one OWED on 2026-11-11: robot one does not exist in our records. The form's item is named in
// each comment (docs/robot-facts-form.md). The placeholders are the brief's probe P3's.

// ── STAGE A — what is plugged where (every program) ──────────────────────────────────────

// Form B6: every motor port the Devices screen lists that is NOT a drive motor. Each must be DECLARED in
// robot one's list — the check is in src/robots/mechanism_table.hpp — or shulib Calibrate would take it
// for a drive candidate and power it (landmine 4, [R9]). A flipper motor included.
inline constexpr auto kNonDriveMotorPorts =
    owed(PortList{{8, 9}, 2}, Stage::A, "every motor port the Devices screen lists that is not a drive motor");
ROBOT_ONE_CHECK(kNonDriveMotorPorts, "every motor port the Devices screen lists that is not a drive motor");

// Form C1: the cascade's motor ports, motor 1 then motor 2. The placeholders 8 and 9 (and B6's above) are
// checked against kRobotOneChassis's IMU port in EVERY build: type the form's A4 in the same edit as B6 and
// C1, or a real IMU on 8 or 9 fails every build by name (guide 21's order, step 1).
inline constexpr auto kCascadePorts = owed(Pair<int>{8, 9}, Stage::A, "the cascade's motor ports");
ROBOT_ONE_CHECK(kCascadePorts, "the cascade's motor ports");

// Form C1: which way each cascade motor is mounted, AS THE BUILDER SEES THEM (two motors facing each
// other = {Forward, Reversed}, HA-255); station 13's direction check confirms each, and a correction is
// a typed edit here.
inline constexpr auto kCascadeDirections = owed(Pair<mechanism::Motor>{mechanism::words::Forward,
                                                                      mechanism::words::Reversed},
                                                Stage::A, "the cascade motors' directions");
ROBOT_ONE_CHECK(kCascadeDirections, "the cascade motors' directions");

// Form C1: the cascade motors' cartridge (the colour of the insert).
inline constexpr auto kCascadeCartridge =
    owed(mechanism::words::Red, Stage::A, "the cascade motors' cartridge");
ROBOT_ONE_CHECK(kCascadeCartridge, "the cascade motors' cartridge");

// Form C2: the gear teeth from motor to spool (counted).
inline constexpr auto kCascadeTeeth = owed(Teeth{12, 60}, Stage::A, "the cascade's gear teeth, motor then spool");
ROBOT_ONE_CHECK(kCascadeTeeth, "the cascade's gear teeth, motor then spool");

// Form C6 (i): the STARTING stall signature — CHOSEN, not measured ([R2]), from chapter 18's order
// rule: the heaviest hold < the signature < the cap - 0.2 A. Station 13 cuts out on it. The MEASURED
// signature (C6 (ii), station 13's RECORD) is typed over it later.
inline constexpr auto kCascadeStallAmps = owed(1.0, Stage::A, "the cascade's starting stall signature, amps");
ROBOT_ONE_CHECK(kCascadeStallAmps, "the cascade's starting stall signature, amps");
inline constexpr auto kCascadeStallRpm =
    owed(5.0, Stage::A, "the cascade's starting stall signature, below this motor rpm");
ROBOT_ONE_CHECK(kCascadeStallRpm, "the cascade's starting stall signature, below this motor rpm");
inline constexpr auto kCascadeStallTime =
    owed(0.3_s, Stage::A, "the cascade's starting stall signature, lasting this long");
ROBOT_ONE_CHECK(kCascadeStallTime, "the cascade's starting stall signature, lasting this long");

// Form C6 (i): the STARTING current cap per motor — CHOSEN, not measured ([R2]): at most what the string
// can take (HA-222), and more than the signature + 0.2 A.
inline constexpr auto kCascadeCapAmps = owed(1.5, Stage::A, "the cascade's starting current cap per motor, amps");
ROBOT_ONE_CHECK(kCascadeCapAmps, "the cascade's starting current cap per motor, amps");

// Form D16: the flipper's KIND ("not on the robot yet" is an answer, typed like any other).
inline constexpr auto kFlipperKind = owed(FlipperKind::NotOnTheRobotYet, Stage::A, "the flipper's kind");
ROBOT_ONE_CHECK(kFlipperKind, "the flipper's kind");

// ── STAGE B — how it moves (Drive, the Auton Tester, Competition, Tune) ──────────────────────

// Form C7: station 13's RECORD at the bottom stop, each level and the top stop — the TAPED height (the
// form's C4 point) and the RECORD's `output ... deg` (NOT its inches: until this is typed, station 13's
// inches come from these placeholders — landmine 6). The bottom row is where homing puts the cascade;
// the top row is its top stop. A row AT each level makes each level exact whatever the spool's layering
// does between rows (HA-254).
inline constexpr auto kCascadeRows = owed(Rows{{{0_in, 0.0}, {8_in, 360.0}, {16_in, 720.0}, {24_in, 1080.0},
                                                {40_in, 1800.0}}},
                                          Stage::B, "the cascade's measured rows, bottom stop to top stop");
ROBOT_ONE_CHECK(kCascadeRows, "the cascade's measured rows, bottom stop to top stop");

// Form C5: the cascade's levels by stack height — a name (15 letters at most) and a height, FOUR TO SIX.
// The default driver file names the first four on the d-pad (src/team/robot_one_driver.hpp), so fewer than
// four named levels fails every build, robot two's too (an unnamed slot is no preset) — a robot with
// fewer is a driver-file edit first.
inline constexpr auto kCascadeLevels =
    owed(Levels{{{"floor", 0_in}, {"L1", 8_in}, {"L2", 16_in}, {"L3", 24_in}, {nullptr, 0_in}, {nullptr, 0_in}}},
         Stage::B, "the cascade's levels, by name and stack height");
ROBOT_ONE_CHECK(kCascadeLevels, "the cascade's levels, by name and stack height");

// Form C8 (i): the smallest volts that hold the empty cascade halfway without sagging.
inline constexpr auto kCascadeHoldVolts = owed(1.0, Stage::B, "the cascade's hold volts");
ROBOT_ONE_CHECK(kCascadeHoldVolts, "the cascade's hold volts");

// Form C8 (ii): the homing push, volts — NEGATIVE = DOWN into the bottom stop (the builder's ruling: stall
// gently down, no switch). Chosen at the bench with the coordinator. (The placeholder is -6 V, not the brief's probe's
// -3 V: on the host test plant a stalled motor draws 2.5 A at 12 V, so -3 V can never reach the 1.0 A
// placeholder signature and the host's homing test would never end on a stall — MECH-S-A's CA9 pushes its
// cascade down at -6 V for the same reason. Whether -6 V is gentle on the real strings is the bench's call.)
inline constexpr auto kCascadeHomingPushVolts =
    owed(-6.0, Stage::B, "the cascade's homing push, volts (negative = down)");
ROBOT_ONE_CHECK(kCascadeHomingPushVolts, "the cascade's homing push, volts (negative = down)");

// Form C8 (iii): the most a held button may give the cascade before it is homed, volts.
inline constexpr auto kCascadeUnhomedVolts = owed(4.0, Stage::B, "the cascade's volts before it is homed");
ROBOT_ONE_CHECK(kCascadeUnhomedVolts, "the cascade's volts before it is homed");

// ── RECORDED — gating no build (D12, [R9]) ──────────────────────────────────────────────────

// Form E18: the AI Vision camera's port and where it sits. Nothing in ROB1-A reads them; VIS1-B wires
// robot one's camera from here.
inline constexpr auto kCameraPort = owed(3, Stage::Recorded, "the AI Vision camera's port");
ROBOT_ONE_CHECK(kCameraPort, "the AI Vision camera's port");
inline constexpr auto kCameraMount =
    owed(CameraMount{6_in, 0_in, 10_in, 0.0}, Stage::Recorded, "where the AI Vision camera sits");
ROBOT_ONE_CHECK(kCameraMount, "where the AI Vision camera sits");

// ═══ THE COUNT (the Drive boot line, RPV1-B's hatch, the tests) ═════════════════════════════

/// One fact as the counts and the boot line see it.
struct FactSummary {
    const char* name;  ///< its name
    Stage needed;      ///< its stage
    bool typed;        ///< typed yet?
};
template <class T>
constexpr FactSummary summary(const Fact<T>& f) {
    return FactSummary{f.name, f.needed, f.typed};
}

/// EVERY fact above, in order (test/robot_one_facts_test.cpp proves none is missing).
inline constexpr FactSummary kRobotOneFacts[] = {
    summary(kNonDriveMotorPorts), summary(kCascadePorts),      summary(kCascadeDirections),
    summary(kCascadeCartridge),   summary(kCascadeTeeth),      summary(kCascadeStallAmps),
    summary(kCascadeStallRpm),    summary(kCascadeStallTime),  summary(kCascadeCapAmps),
    summary(kFlipperKind),        summary(kCascadeRows),       summary(kCascadeLevels),
    summary(kCascadeHoldVolts),   summary(kCascadeHomingPushVolts), summary(kCascadeUnhomedVolts),
    summary(kCameraPort),         summary(kCameraMount),
};

/// How many facts the ledger holds.
inline constexpr int kRobotOneFactCount = static_cast<int>(sizeof kRobotOneFacts / sizeof kRobotOneFacts[0]);

/// How many are still OWED.
constexpr int owedCount() {
    int n = 0;
    for (const FactSummary& f : kRobotOneFacts) {
        n += f.typed ? 0 : 1;
    }
    return n;
}
inline constexpr int kRobotOneOwedCount = owedCount();

/// How many program `p` needs that are still OWED (zero on every program robot one's build links).
constexpr int owedFor(Program p) {
    int n = 0;
    for (const FactSummary& f : kRobotOneFacts) {
        n += (!f.typed && needs(p, f.needed)) ? 1 : 0;
    }
    return n;
}

/// The cascade's bottom stop: its FIRST measured row's height — where homing puts it, and the range's
/// bottom (no separate fact, so the two cannot disagree).
constexpr units::Length cascadeBottom() { return kCascadeRows.value().row[0].height; }

/// The cascade's top stop: its LAST measured row's height — the range's top (no separate fact).
constexpr units::Length cascadeTop() { return kCascadeRows.value().row[kCascadeRowCount - 1].height; }

/// The cascade's stall signature, from its three facts — the declaration's `.stall` AND its homing's.
constexpr mechanism::Stall cascadeStall() {
    return mechanism::Stall{.amps = kCascadeStallAmps.value(),
                            .belowMotorRpm = kCascadeStallRpm.value(),
                            .lasting = kCascadeStallTime.value()};
}

/// The level in slot `i` (0..5) — the declaration and the driver file spell each slot ([R8]).
constexpr const Level& lv(std::size_t i) { return kCascadeLevels.value().level[i]; }

}  // namespace shulib::bench::robot_one_facts
