#pragma once
//
// robot_one_facts.hpp — ROBOT ONE'S FACTS LEDGER (chunk ROB1-A, 2026-11-11; the ROB1 brief's D3 (a),
// re-check R2, R4, R9, R10, R11). Every number about robot one — the 2026 CASCADE robot, built by
// 2026-11-15 — is written ONCE, HERE, and robot one's declarations (src/robots/mechanism_table.hpp's
// robot_one region) read ONLY these. Since ROB1-C2 (2026-11-15) the builder's first answers are TYPED —
// read off the robot (ports, cartridges, the gearing, the three pistons' letters, the flipper's kind) —
// with the bench's starting values chosen, not measured; what nobody has said is still OWED. No census,
// reading or photo of robot one is in this project's records. NOTHING HERE HAS RUN ON A ROBOT.
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
//                   brain's Devices screen lists, the flipper's KIND, the side toggles' ports (and which
//                   is the right one), cartridge and starting cap, and the three pistons' 3-wire letters.
//                   Needed by ALL SIX programs — Bench Tests included, because it powers declared
//                   mechanisms.
//   Stage::B        "how it moves": the cascade's measured rows (bottom, every level, top), its levels,
//                   its hold volts, its homing push and the homing's own (gentle) stall signature, its
//                   volts before it is homed; which way each side
//                   toggle runs; which line level is each piston's active position, and each stroke
//                   time (ROB1-C2: Bench Tests' station 13 is where each of those is found). Needed by EVERY
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
// a build (landmine 17). What is NOT a ledger edit: a third or fourth cascade motor, a new mechanism —
// those are DECLARATION edits in src/robots/mechanism_table.hpp (the motor count is structural). A
// new mechanism's NUMBERS are not: before its block is written, each of its numbers (port, cartridge,
// teeth, stall, cap, range, angles — or a pneumatic's line, levels and stroke time) becomes a fact here
// with its check and its stage, and the block reads them as `facts::…` (ROB1-C2 did exactly that for the
// side toggles and the three pistons); test/robot_one_facts_test.cpp's region scan fails the host suite on
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
constexpr bool same(char a, char b) { return a == b; }  // a 3-wire letter
constexpr bool same(bool a, bool b) { return a == b; }  // a line level (High / Low)
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
/// every named level and at the top stop, strictly increasing. TWO is this ledger's count since ROB1-C2
/// (the bottom stop and the top stop: the cascade has no levels yet — kCascadeLevels); a level typed later
/// brings its own row, typed by changing the 2 here (the library takes two to eight —
/// mechanism::measured(), spec.hpp's position_a_lookup_table_needs_two_to_eight_points).
inline constexpr std::size_t kCascadeRowCount = 2;
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

/// The flipper's kind (AF1-S's D7 — the coordinator's assignment, 2026-11-15, from the builder's answer "B will be
/// the Forward Toggle pistons": a pneumatic, the front toggle). Whatever is typed, the build checks
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
// Every one OWED on 2026-11-11: robot one did not exist in our records. The form's item is named in each
// comment (docs/robot-facts-form.md). The placeholders are the brief's probe P3's. ROB1-C2 (2026-11-15) typed
// the builder's first answers — "the team's builder, 2026-11-15, read off the robot" — and the starting values
// chosen for the bench ("chosen, not measured"), and added the facts of the mechanisms he named that night
// (the side toggles, the claw, the front toggle — the flipper — and the ramp). A fact ROB1-C2 added already
// typed keeps a placeholder of its own (never read: value() is the typed one) so the form stays the ledger's.

// ── STAGE A — what is plugged where (every program) ──────────────────────────────────────

// Form B6: every motor port the Devices screen lists that is NOT a drive motor. Each must be DECLARED in
// robot one's list — the check is in src/robots/mechanism_table.hpp — or shulib Calibrate would take it
// for a drive candidate and power it (landmine 4, [R9]): the cascade's 10 and 1, the side toggles' 9 and 2.
inline constexpr auto kNonDriveMotorPorts =
    typed(PortList{{8, 9}, 2}, PortList{{10, 1, 9, 2}, 4}, Stage::A,
          "every motor port the Devices screen lists that is not a drive motor",
          "the team's builder, 2026-11-15, read off the robot: the cascade on 10 and 1, the side toggles on 9 and 2");
ROBOT_ONE_CHECK(kNonDriveMotorPorts, "every motor port the Devices screen lists that is not a drive motor");

// Form C1: the cascade's motor ports, motor 1 then motor 2.
inline constexpr auto kCascadePorts = typed(Pair<int>{8, 9}, Pair<int>{10, 1}, Stage::A, "the cascade's motor ports",
                                            "the team's builder, 2026-11-15, read off the robot");
ROBOT_ONE_CHECK(kCascadePorts, "the cascade's motor ports");

// Form C1: which way each cascade motor is mounted. The builder said the two FACE EACH OTHER (= {Forward,
// Reversed}, HA-255); which of the two is Forward (port 10) is the coordinator's reading, not his words.
// Station 13's direction check confirms each, and a correction is a typed edit here.
inline constexpr auto kCascadeDirections =
    typed(Pair<mechanism::Motor>{mechanism::words::Forward, mechanism::words::Reversed},
          Pair<mechanism::Motor>{mechanism::words::Forward, mechanism::words::Reversed}, Stage::A,
          "the cascade motors' directions",
          "the team's builder, 2026-11-15, read off the robot: the two motors FACE EACH OTHER; port 10 taken as "
          "Forward and port 1 as Reversed by the coordinator, station 13 confirms -- same as the placeholder, checked");
ROBOT_ONE_CHECK(kCascadeDirections, "the cascade motors' directions");

// Form C1: the cascade motors' cartridge (the colour of the insert).
inline constexpr auto kCascadeCartridge = typed(mechanism::words::Red, mechanism::words::Blue, Stage::A,
                                                "the cascade motors' cartridge",
                                                "the team's builder, 2026-11-15, read off the robot (blue)");
ROBOT_ONE_CHECK(kCascadeCartridge, "the cascade motors' cartridge");

// Form C2: the gear teeth from motor to spool (counted). DIRECT: no gears — the motor turns the spool —
// typed as the 1:1 a tooth count can say (gears(1, 1) = one motor turn per spool turn, the same as `direct`).
inline constexpr auto kCascadeTeeth =
    typed(Teeth{12, 60}, Teeth{1, 1}, Stage::A, "the cascade's gear teeth, motor then spool",
          "the team's builder, 2026-11-15, read off the robot: DIRECT, no gears -- the motor turns the spool (1:1)");
ROBOT_ONE_CHECK(kCascadeTeeth, "the cascade's gear teeth, motor then spool");

// Form C6 (i): the STARTING stall signature — CHOSEN, not measured ([R2]), from chapter 18's order
// rule: the heaviest hold < the signature < the cap - 0.2 A. Station 13 cuts out on it. The MEASURED
// signature (C6 (ii), station 13's RECORD) is typed over it later.
inline constexpr auto kCascadeStallAmps =
    typed(1.0, 1.5, Stage::A, "the cascade's starting stall signature, amps",
          "chosen, not measured -- the coordinator, 2026-11-15 (blue direct-drive spool, strings both ways; "
          "the bench tunes it)");
ROBOT_ONE_CHECK(kCascadeStallAmps, "the cascade's starting stall signature, amps");
inline constexpr auto kCascadeStallRpm =
    typed(5.0, 5.0, Stage::A, "the cascade's starting stall signature, below this motor rpm",
          "chosen, not measured -- the coordinator, 2026-11-15 -- same as the placeholder, checked");
ROBOT_ONE_CHECK(kCascadeStallRpm, "the cascade's starting stall signature, below this motor rpm");
inline constexpr auto kCascadeStallTime =
    typed(0.3_s, 0.3_s, Stage::A, "the cascade's starting stall signature, lasting this long",
          "chosen, not measured -- the coordinator, 2026-11-15 -- same as the placeholder, checked");
ROBOT_ONE_CHECK(kCascadeStallTime, "the cascade's starting stall signature, lasting this long");

// Form C6 (i): the STARTING current cap per motor — CHOSEN, not measured ([R2]): at most what the string
// can take (HA-222), and more than the signature + 0.2 A.
inline constexpr auto kCascadeCapAmps = typed(1.5, 2.0, Stage::A, "the cascade's starting current cap per motor, amps",
                                              "chosen, not measured -- the coordinator, 2026-11-15");
ROBOT_ONE_CHECK(kCascadeCapAmps, "the cascade's starting current cap per motor, amps");

// Form D16: the flipper's KIND ("not on the robot yet" is an answer, typed like any other). The FRONT
// TOGGLE is robot one's flipper for the autons' Toggle step (AF1S): two forward-facing pistons on one
// solenoid, pushing forward where the camera looks — declared as the Pneumatic named "flipper". The builder
// named the front toggle's pistons (B); making them the autons' flipper is the COORDINATOR's assignment, so
// the source says so. (The SIDE toggles are not the flipper: they are two power mechanisms, below.)
inline constexpr auto kFlipperKind =
    typed(FlipperKind::NotOnTheRobotYet, FlipperKind::Pneumatic, Stage::A, "the flipper's kind",
          "chosen, not measured -- the coordinator, 2026-11-15, from the builder's answer 'B will be the Forward "
          "Toggle pistons': the front toggle (two forward pistons on one solenoid, 3-wire B) is the autons' flipper");
ROBOT_ONE_CHECK(kFlipperKind, "the flipper's kind");

// Form G21: the SIDE TOGGLES' two motor ports, either order (each its own arm on its own motor, driven
// separately). WHICH ONE IS THE RIGHT is the fact below; the left is the other.
inline constexpr auto kSideTogglePorts = typed(Pair<int>{11, 12}, Pair<int>{9, 2}, Stage::A,
                                               "the side toggles' motor ports, either order",
                                               "the team's builder, 2026-11-15, read off the robot");
ROBOT_ONE_CHECK(kSideTogglePorts, "the side toggles' motor ports, either order");

// Form G21: which of the two is the RIGHT side toggle's port (9 or 2 — the left side toggle is on the
// other; leftTogglePort() below).
inline constexpr auto kRightTogglePort =
    typed(9, 9, Stage::A, "the right side toggle's port",
          "same as the placeholder, checked -- the team's builder, 2026-11-15: '9 is right side and left is 2'");
ROBOT_ONE_CHECK(kRightTogglePort, "the right side toggle's port");

// Form G21: the side toggles' cartridge — both motors the same. DIRECT drive (no gears): a power
// mechanism states no gearing, so there is no tooth count to type.
inline constexpr auto kSideToggleCartridge = typed(mechanism::words::Red, mechanism::words::Green, Stage::A,
                                                   "the side toggles' cartridge",
                                                   "the team's builder, 2026-11-15, read off the robot (green, direct)");
ROBOT_ONE_CHECK(kSideToggleCartridge, "the side toggles' cartridge");

// Form G21: the side toggles' STARTING current cap per motor — CHOSEN, not measured, to bound the push: a
// toggle held against its stop while its button is held pushes with at most this (the motor's own limit is
// 2.5 A) — whether that is gentle enough on the real arm nobody has measured. The cap is written into the
// motors when a program's mechanism service binds, and by Bench Tests' station 13 while it powers a toggle
// (S13UX). The bench raises it if a flip is too weak.
inline constexpr auto kSideToggleCapAmps =
    typed(1.0, 1.5, Stage::A, "the side toggles' starting current cap per motor, amps",
          "chosen, not measured -- the coordinator's ROB1-C2 run, 2026-11-15 (a toggle held against its stop at "
          "12 V; the bench tunes it)");
ROBOT_ONE_CHECK(kSideToggleCapAmps, "the side toggles' starting current cap per motor, amps");

// Calibrate's two thresholds for robot one's 10-motor drive (5 per side): one pulsed motor drags its four
// side-mates and turned the robot only ~1.0 deg at 9 V (Calibrate runs 2 and 4, 2026-11-16; the default
// needs 2.0), so the floor is lowered and "still" tightened with it: a drift adds at most 0.4 deg to a 0.4 s pulse.
inline constexpr auto kCalibratePulseYawFloorDeg =
    typed(2.0, 0.6, Stage::A, "Calibrate's pulse yaw floor, degrees",
          "chosen, not measured -- the coordinator, 2026-11-16 (one motor of ten turned robot one 1.0 deg at 9 V)");
ROBOT_ONE_CHECK(kCalibratePulseYawFloorDeg, "Calibrate's pulse yaw floor, degrees");
inline constexpr auto kCalibrateRestYawRateDegPerS =
    typed(2.0, 1.0, Stage::A, "Calibrate's rest yaw rate, degrees per second",
          "chosen, not measured -- the coordinator, 2026-11-16 (so a drift adds at most 0.4 deg to a 0.4 s pulse)");
ROBOT_ONE_CHECK(kCalibrateRestYawRateDegPerS, "Calibrate's rest yaw rate, degrees per second");

// The autons' drive settings (the library's defaults are the placeholders). Chosen 2026-11-17 after robot one's
// first auton runs: 60 in/s felt too fast (the team lead: "its scary how fast the robot goes"), and at each target
// the heading rocked +-2-3 deg until the 5 s limit (its 10 geared motors have slack) -- a softer heading and a 2 deg
// arrival. Tune them on the field.
inline constexpr auto kDriveMaxSpeedInPerS =
    typed(60.0, 40.0, Stage::B, "the autons' top drive speed, in/s",
          "chosen, not measured -- the coordinator, 2026-11-17 (the team lead: slower)");
ROBOT_ONE_CHECK(kDriveMaxSpeedInPerS, "the autons' top drive speed, in/s");
inline constexpr auto kDriveMaxTurnRadPerS =
    typed(6.0, 4.0, Stage::B, "the autons' top turn rate, rad/s",
          "chosen, not measured -- the coordinator, 2026-11-17 (slower turns, as the drive)");
ROBOT_ONE_CHECK(kDriveMaxTurnRadPerS, "the autons' top turn rate, rad/s");
inline constexpr auto kDriveHeadingKp =
    typed(4.0, 2.4, Stage::B, "the autons' heading correction gain",
          "chosen, not measured -- the coordinator, 2026-11-17 (the heading rocked +-2-3 deg at each target at 4.0)");
ROBOT_ONE_CHECK(kDriveHeadingKp, "the autons' heading correction gain");
inline constexpr auto kDriveHeadingSettleDeg =
    typed(1.15, 4.0, Stage::B, "the autons' heading arrival, degrees",
          "chosen, not measured -- the coordinator, 2026-11-17 (2 deg did not settle: the heading rocked +-1.5 deg "
          "with a jumpy rate on the hard route's 4th drive; a drive still steers to its point)");
ROBOT_ONE_CHECK(kDriveHeadingSettleDeg, "the autons' heading arrival, degrees");

// Forms G22-G24: the three pistons' 3-wire ports (the letter on the brain).
inline constexpr auto kClawLine = typed('F', 'A', Stage::A, "the claw's 3-wire port",
                                        "the team's builder, 2026-11-15, read off the robot");
ROBOT_ONE_CHECK(kClawLine, "the claw's 3-wire port");
inline constexpr auto kFlipperLine = typed('G', 'B', Stage::A, "the front toggle's 3-wire port (the flipper)",
                                           "the team's builder, 2026-11-15, read off the robot: both pistons on one "
                                           "solenoid");
ROBOT_ONE_CHECK(kFlipperLine, "the front toggle's 3-wire port (the flipper)");
inline constexpr auto kRampLine = typed('H', 'C', Stage::A, "the ramp's 3-wire port",
                                        "the team's builder, 2026-11-15, read off the robot");
ROBOT_ONE_CHECK(kRampLine, "the ramp's 3-wire port");

// ── STAGE B — how it moves (Drive, the Auton Tester, Competition, Tune) ──────────────────────

// Form C7: station 13's RECORD at the bottom stop and the top stop (and, once levels are named, at each
// level) — the TAPED height (the form's C4 point) and the RECORD's `output ... deg` (NOT its inches: until
// this is typed, station 13's inches come from these placeholders — landmine 6). The bottom row is where
// homing puts the cascade; the top row is its top stop.
inline constexpr auto kCascadeRows =
    typed(Rows{{{0_in, 0.0}, {40_in, 1800.0}}}, Rows{{{0_in, 0.0}, {20_in, 3759.0}}}, Stage::B,
          "the cascade's measured rows, bottom stop to top stop",
          "station 13 RECORD, the team lead, 2026-11-17: on the bottom stop -130.8 deg, raised 20 in (taped, about "
          "1 in under the top) +3628.2 deg -- 3759.0 deg above the bottom stop, where homing puts 0");
ROBOT_ONE_CHECK(kCascadeRows, "the cascade's measured rows, bottom stop to top stop");

// Form C5: the cascade's levels by stack height — a name (15 letters at most) and a height, NONE TO SIX.
// The builder, 2026-11-15: the cascade is MANUAL for now (no presets; preset heights later), so robot
// one's levels are typed as NONE. The rule that remains is wherever a preset is BOUND: a driver file (or an
// auton) that names `facts::lv(i).name` for a slot with no name fails the build by name — so the day
// presets come back, their levels are typed here first (with a row AT each — HA-254, kCascadeRowCount).
inline constexpr auto kCascadeLevels =
    typed(Levels{{{"floor", 0_in}, {"L1", 8_in}, {"L2", 16_in}, {"L3", 24_in}, {nullptr, 0_in}, {nullptr, 0_in}}},
          Levels{}, Stage::B, "the cascade's levels, by name and stack height",
          "the team's builder, 2026-11-15: none yet -- 'Later we will have preset heights for the cascade but for "
          "now it will be manual'");
ROBOT_ONE_CHECK(kCascadeLevels, "the cascade's levels, by name and stack height");

// Form C8 (i): the smallest volts that hold the empty cascade halfway without sagging.
inline constexpr auto kCascadeHoldVolts =
    typed(1.0, 1.0, Stage::B, "the cascade's hold volts",
          "same as the placeholder, checked -- the coordinator, 2026-11-17: a starting value, not measured (its motors "
          "also HOLD on release); raise it if the cascade sags");
ROBOT_ONE_CHECK(kCascadeHoldVolts, "the cascade's hold volts");

// Form C8 (ii): the homing push, volts — NEGATIVE = DOWN into the bottom stop (the builder's ruling: stall
// gently down, no switch). Chosen at the bench with the coordinator. The push must draw MORE than the
// homing signature below when it stalls on the bottom stop, or the home never ends on a stall (TIMED_OUT,
// never homed): on the host test plant a stalled motor draws 2.5 A at 12 V and proportionally less below
// it (HA-183, HA-93: invented), so past 12 x 0.45 / 2.5 = 2.2 V. (The placeholder is ROB1-A's -6 V.)
inline constexpr auto kCascadeHomingPushVolts =
    typed(-6.0, -3.0, Stage::B, "the cascade's homing push, volts (negative = down)",
          "chosen, not measured -- the coordinator, 2026-11-17 (gentle: station 13 jogged it cleanly at 3 V; a 3 V "
          "stall draws more than the 0.45 A homing signature on the host model)");
ROBOT_ONE_CHECK(kCascadeHomingPushVolts, "the cascade's homing push, volts (negative = down)");

// Form C8 (ii): the HOMING's own stall signature, amps — what ends the gentle push on the bottom stop. NOT
// the jam signature above (kCascadeStallAmps): a gentle push draws far less than a jam at full power, so
// the 1.5 A jam signature could never end a 3 V home (chapter 18: "a homing push is gentle, so its stall is
// SMALL"; the review of ROB1-C2). CHOSEN, not measured: chapter 18's starting value for a gentle push. Its
// speed and time are the jam signature's (kCascadeStallRpm, kCascadeStallTime). Tune's HOME checks it on the
// robot; Tune's COPY of a `.home` line types its amps here.
inline constexpr auto kCascadeHomingStallAmps =
    typed(0.4, 0.45, Stage::B, "the cascade's homing stall signature, amps",
          "chosen, not measured -- the coordinator's ROB1-C2 run, 2026-11-15 (chapter 18's starting homing signature "
          "for a gentle push; the push must draw more when it stalls)");
ROBOT_ONE_CHECK(kCascadeHomingStallAmps, "the cascade's homing stall signature, amps");

// Form C8 (iii): the most a held button may give the cascade before it is homed, volts.
inline constexpr auto kCascadeUnhomedVolts =
    typed(4.0, 4.0, Stage::B, "the cascade's volts before it is homed",
          "same as the placeholder, checked -- the coordinator, 2026-11-17: chosen (station 13 jogged it cleanly at 3 "
          "and 6 V)");
ROBOT_ONE_CHECK(kCascadeUnhomedVolts, "the cascade's volts before it is homed");

// Form G21: which way each side toggle's motor turns when it RUNS (held: it runs ONE way only). Two
// separate motors running mirror-image ways — so the placeholders differ; station 13's direction check
// (or the builder) says which is right. A side toggle that runs the wrong way is fixed HERE, never by its
// button.
inline constexpr auto kRightToggleDirection =
    typed(mechanism::words::Forward, mechanism::words::Forward, Stage::B, "the right side toggle's run direction",
          "same as the placeholder, checked against the builder's words (2026-11-15: right toggle clockwise, left "
          "counter-clockwise; each piston open / out / down on its button), NOT watched at station 13 -- the team lead, "
          "2026-11-17: 'go for it' -- confirm in Drive");
ROBOT_ONE_CHECK(kRightToggleDirection, "the right side toggle's run direction");
inline constexpr auto kLeftToggleDirection =
    typed(mechanism::words::Reversed, mechanism::words::Reversed, Stage::B, "the left side toggle's run direction",
          "same as the placeholder, checked against the builder's words (2026-11-15: right toggle clockwise, left "
          "counter-clockwise; each piston open / out / down on its button), NOT watched at station 13 -- the team lead, "
          "2026-11-17: 'go for it' -- confirm in Drive");
ROBOT_ONE_CHECK(kLeftToggleDirection, "the left side toggle's run direction");

// Forms G22-G24: which line LEVEL puts each piston in its ACTIVE position — the claw OPEN, the front
// toggle OUT (open), the ramp DOWN (High = the solenoid powered). Station 13 fires each and the builder
// says which way it went. Whatever it is, each piston STARTS (and is made safe) in the position its
// solenoid holds UNPOWERED (unpoweredPosition() below), so nothing moves at power-up — a wrong guess
// here only swaps the two NAMES, which an auton reads (`claw["open"]`).
inline constexpr auto kClawOpenLevel =
    typed(mechanism::words::High, mechanism::words::High, Stage::B, "the claw's line level that OPENS it",
          "same as the placeholder, checked against the builder's words (2026-11-15: right toggle clockwise, left "
          "counter-clockwise; each piston open / out / down on its button), NOT watched at station 13 -- the team lead, "
          "2026-11-17: 'go for it' -- confirm in Drive");
ROBOT_ONE_CHECK(kClawOpenLevel, "the claw's line level that OPENS it");
inline constexpr auto kFlipperOpenLevel =
    typed(mechanism::words::High, mechanism::words::High, Stage::B, "the front toggle's line level that pushes it OUT (open)",
          "same as the placeholder, checked against the builder's words (2026-11-15: right toggle clockwise, left "
          "counter-clockwise; each piston open / out / down on its button), NOT watched at station 13 -- the team lead, "
          "2026-11-17: 'go for it' -- confirm in Drive");
ROBOT_ONE_CHECK(kFlipperOpenLevel, "the front toggle's line level that pushes it OUT (open)");
inline constexpr auto kRampDownLevel =
    typed(mechanism::words::High, mechanism::words::High, Stage::B, "the ramp's line level that puts it DOWN",
          "same as the placeholder, checked against the builder's words (2026-11-15: right toggle clockwise, left "
          "counter-clockwise; each piston open / out / down on its button), NOT watched at station 13 -- the team lead, "
          "2026-11-17: 'go for it' -- confirm in Drive");
ROBOT_ONE_CHECK(kRampDownLevel, "the ramp's line level that puts it DOWN");

// Forms G22-G24: how long a full stroke takes — the wait before a piston's move counts as done. A STARTING
// value CHOSEN, not measured (robot two's claw's provisional 0.5 s, HA-185); timed at the bench later.
inline constexpr auto kClawStrokeTime =
    typed(0.4_s, 0.5_s, Stage::B, "the claw's stroke time",
          "chosen, not measured -- the coordinator's ROB1-C2 run, 2026-11-15 (robot two's claw's provisional "
          "time, HA-185)");
ROBOT_ONE_CHECK(kClawStrokeTime, "the claw's stroke time");
inline constexpr auto kFlipperStrokeTime =
    typed(0.4_s, 0.5_s, Stage::B, "the front toggle's stroke time",
          "chosen, not measured -- the coordinator's ROB1-C2 run, 2026-11-15 (robot two's claw's provisional "
          "time, HA-185)");
ROBOT_ONE_CHECK(kFlipperStrokeTime, "the front toggle's stroke time");
inline constexpr auto kRampStrokeTime =
    typed(0.4_s, 0.5_s, Stage::B, "the ramp's stroke time",
          "chosen, not measured -- the coordinator's ROB1-C2 run, 2026-11-15 (robot two's claw's provisional "
          "time, HA-185)");
ROBOT_ONE_CHECK(kRampStrokeTime, "the ramp's stroke time");
// The most volts anything drives the cascade DOWN with (API 2.10's maxDownVolts): 60 % of 12 V. Up keeps 12 V.
inline constexpr auto kCascadeMaxDownVolts =
    typed(12.0, 7.2, Stage::B, "the cascade's most volts DOWN",
          "chosen, not measured -- the team lead, 2026-11-20: 'it needs to go down to 60' (60 % of 12 V)");
ROBOT_ONE_CHECK(kCascadeMaxDownVolts, "the cascade's most volts DOWN");

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
    summary(kNonDriveMotorPorts),   summary(kCascadePorts),          summary(kCascadeDirections),
    summary(kCascadeCartridge),     summary(kCascadeTeeth),          summary(kCascadeStallAmps),
    summary(kCascadeStallRpm),      summary(kCascadeStallTime),      summary(kCascadeCapAmps),
    summary(kFlipperKind),          summary(kSideTogglePorts),       summary(kRightTogglePort),
    summary(kSideToggleCartridge),  summary(kSideToggleCapAmps),     summary(kCalibratePulseYawFloorDeg),
    summary(kCalibrateRestYawRateDegPerS), summary(kDriveMaxSpeedInPerS), summary(kDriveMaxTurnRadPerS),
    summary(kDriveHeadingKp),       summary(kDriveHeadingSettleDeg),  summary(kClawLine),
    summary(kFlipperLine),          summary(kRampLine),              summary(kCascadeRows),
    summary(kCascadeLevels),        summary(kCascadeHoldVolts),      summary(kCascadeHomingPushVolts),
    summary(kCascadeHomingStallAmps), summary(kCascadeUnhomedVolts),  summary(kRightToggleDirection),  summary(kLeftToggleDirection),
    summary(kClawOpenLevel),        summary(kFlipperOpenLevel),      summary(kRampDownLevel),
    summary(kClawStrokeTime),       summary(kFlipperStrokeTime),     summary(kRampStrokeTime),
    summary(kCascadeMaxDownVolts),
    summary(kCameraPort),           summary(kCameraMount),
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

/// The cascade's JAM stall signature, from its three facts — the declaration's `.stall`.
constexpr mechanism::Stall cascadeStall() {
    return mechanism::Stall{.amps = kCascadeStallAmps.value(),
                            .belowMotorRpm = kCascadeStallRpm.value(),
                            .lasting = kCascadeStallTime.value()};
}

/// The cascade's HOMING stall signature — its own amps (kCascadeHomingStallAmps: a gentle push's stall), the
/// jam signature's speed and time — the declaration's `homeByStall(...)`.
constexpr mechanism::Stall cascadeHomingStall() {
    return mechanism::Stall{.amps = kCascadeHomingStallAmps.value(),
                            .belowMotorRpm = kCascadeStallRpm.value(),
                            .lasting = kCascadeStallTime.value()};
}

/// The level in slot `i` (0..5) — the declaration and the driver file spell each slot ([R8]).
constexpr const Level& lv(std::size_t i) { return kCascadeLevels.value().level[i]; }

/// The LEFT side toggle's port: the side toggle port that is not the right one's (kSideTogglePorts holds
/// both, kRightTogglePort says which is the right). 0 — no smart port, which the wiring refuses by name in
/// every build — when the right one's is neither of the two (a typing slip).
constexpr int leftTogglePort() {
    const Pair<int>& both = kSideTogglePorts.value();
    const int right = kRightTogglePort.value();
    return right == both.first ? both.second : (right == both.second ? both.first : 0);
}

/// Of a two-position piston whose ACTIVE position (`active` — open, out, down) is at line level
/// `activeLevel`, the position its solenoid holds UNPOWERED (Low): its start and safe position, so nothing
/// moves at power-up or when the run ends (the ROB1-C2 rule; forms G22-G24).
constexpr const char* unpoweredPosition(bool activeLevel, const char* rest, const char* active) {
    return activeLevel == mechanism::words::Low ? active : rest;
}

}  // namespace shulib::bench::robot_one_facts
