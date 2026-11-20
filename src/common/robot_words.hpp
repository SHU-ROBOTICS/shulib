#pragma once
//
// robot_words.hpp — THE WORDS A PROGRAM SAYS THAT ARE TRUE OF ONE ROBOT ONLY (chunk ROB1-A, 2026-11-11;
// the ROB1 brief's §1.1 `this_robot.hpp` words and re-check R7): its driver file's and autons file's
// paths, the mechanism row's first word, the controller's word for an absent mechanism, the boot's label
// and refusal words, and (since ROB1-BT) its name in its calibration record. Chosen per robot HERE, by
// wordsFor(RobotId) — a case per robot, no default (-Wswitch fails the src build gate).
//
// SINCE ROB1-BT (2026-11-16) BOTH ROBOTS SAY THE MECHANISM WORDS: robot two was rebuilt as robot one's
// twin (its claws retired — the team lead, 2026-11-13), so the two robots' words differ only in their
// own paths and names. The CLAW words — robot two's until then, byte for byte — are kClawWords now: the
// words of the bench bot and the invented X-drive, which print none of them (they run no driver, auton or
// boot-from-record program) and so the host suite's default, whose claw tests read the retired claw robot
// (test/mechanism_fixtures.hpp's mech_fixture::robot_two).
//
// ═══ HOW THEY REACH A PROGRAM — TWO FORMS, and why ══════════════════════════════════════════
// (1) A DEFAULTED `Words`: src/common/boot_source.hpp's decideBootSourceFrom() (and
//     src/common/dropped_autons.hpp's line) take a LAST, DEFAULTED template argument `Words =
//     ThisRobotWords` and a last defaulted parameter `Words = {}` — so every existing call, robot two's
//     tests included, is unchanged, and a host test passes `WordsOf<RobotId::RobotOne>{}` to see robot
//     one's words (the brief's T10). Robot two's packages were compared function by function with this
//     form in place (T6): unchanged.
// (2) AN OVERLOAD that takes `Words` EXPLICITLY: the driver panel's clawRowText(), controllerWord() and
//     controllerScreenLines() (src/common/driver_panel.hpp). The claw words' three are their base text, byte
//     for byte, and the mechanism words' (both robots' since ROB1-BT) are separate overloads that wrap them;
//     each program picks with `?:` or `if constexpr` on `ThisRobotWords::value.claw`. WHY not form (1) here: T6 found that making those three
//     defaulted templates changed GCC's INLINING in robot two's Drive and Competition (clawRowText and
//     controllerScreenLines were inlined where the base calls them) — robot two's machine code changed. The
//     overloads leave robot two's functions exactly as they were (ROB1-PROGRESS, delta 1).
// Either way the words travel as a TYPE, not a value, on purpose: `-Wformat=2` is on (Makefile), so a
// printf format may never come from a variable, and a format is chosen between TWO LITERALS with `?:` on
// `Words::value.claw` — a constant expression, so GCC checks both literals' arguments and keeps ONLY the
// chosen one in the package (robot one's literals never reach robot two's packages; the brief's R7
// "that GCC folds the defaulted words" is settled by construction, not by the optimiser — landmine 18).
//
// The words are DATA only — no driver file, no autons: src/common/this_robot.hpp (which includes this
// file) selects those; this file is light enough for src/common/boot_source.hpp and
// src/common/driver_panel.hpp, whose host tests link no team file.
//
// PROS-FREE ON PURPOSE.

#include <cstddef>
#include <string_view>

#include "build_selection.hpp"

namespace shulib::bench {

/// One robot's words (file banner).
struct RobotWords {
    /// TRUE = the "claw" words (a robot whose every declared motor is a claw pivot — kClawWords); FALSE =
    /// the general "mechanism" words. The boot's step-5 formats choose on it (constant expression).
    bool claw;
    /// What the programs call its driver file: "robot two's driver file".
    const char* driverFileWords;
    /// Its driver file's path, from the repository root.
    const char* driverFilePath;
    /// Its autons file's path, from the repository root.
    const char* autonsFilePath;
    /// The mechanism row's first word on the Drive and Competition screens: four letters.
    const char* rowWord;
    /// The controller's word for an absent declared mechanism (at most 19 characters, HA-57).
    const char* lcdAbsent;
    /// The boot port check's label for a declared mechanism MOTOR port the description does not list.
    const char* declaredMotorLabel;
    /// Its name in its calibration record (ROB1-BT): robotRecordName(id). Calibrate writes it; the boot
    /// refuses a record that names another robot, or none. nullptr = a robot that never boots from a record.
    const char* recordName;
    /// The route builder's routes both robots register (ROB1-BT, RMP1-C): fixed THERE, not in the autons file.
    /// nullptr = a robot that registers none.
    const char* routeFilePath;
};

/// Robot two's words — robot one's twin since ROB1-BT: the mechanism words, its own paths and name.
inline constexpr RobotWords kRobotTwoWords{
    .claw = false,
    .driverFileWords = "robot two's driver file",
    .driverFilePath = "src/team/robot_two_driver.hpp",
    .autonsFilePath = "src/team/autons.cpp",
    .rowWord = "MECH",
    .lcdAbsent = "MECH ABSENT",
    .declaredMotorLabel = "mechanism motors (declared, not drive motors)",
    .recordName = robotRecordName(RobotId::RobotTwo),
    .routeFilePath = "src/team/skills_route.cpp",
};

/// Robot one's words: a cascade and its pistons — "mechanism", never "claw".
inline constexpr RobotWords kRobotOneWords{
    .claw = false,
    .driverFileWords = "robot one's driver file",
    .driverFilePath = "src/team/robot_one_driver.hpp",
    .autonsFilePath = "src/team/robot_one_autons.cpp",
    .rowWord = "MECH",
    .lcdAbsent = "MECH ABSENT",
    .declaredMotorLabel = "mechanism motors (declared, not drive motors)",
    .recordName = robotRecordName(RobotId::RobotOne),
    .routeFilePath = "src/team/skills_route.cpp",
};

/// The CLAW words — robot two's until ROB1-BT, byte for byte (file banner): the bench bot's and the
/// invented X-drive's, and so the host suite's default. No record name: neither boots from a record.
inline constexpr RobotWords kClawWords{
    .claw = true,
    .driverFileWords = "robot two's driver file",
    .driverFilePath = "src/team/robot_two_driver.hpp",
    .autonsFilePath = "src/team/autons.cpp",
    .rowWord = "CLAW",
    .lcdAbsent = "CLAW ABSENT",
    .declaredMotorLabel = "claw pivots (declared, not drive motors)",
    .recordName = nullptr,
    .routeFilePath = nullptr,
};

/// Robot `id`'s words. A case per robot and no default (file banner).
constexpr const RobotWords& wordsFor(RobotId id) {
    switch (id) {
        case RobotId::RobotTwo:
            return kRobotTwoWords;
        case RobotId::RobotOne:
            return kRobotOneWords;
        case RobotId::BenchBot:
            return kClawWords;
        case RobotId::InventedXDrive:
            return kClawWords;
    }
    __builtin_unreachable();
}

/// Robot `R`'s words as a TYPE (file banner): `Words::value` is a constant expression wherever it is
/// read, so a choice between two literals on it is decided at compile time.
template <RobotId R>
struct WordsOf {
    /// The robot.
    static constexpr RobotId robot = R;
    /// Its words.
    static constexpr const RobotWords& value = wordsFor(R);
};

/// THIS build's robot's words — form (1)'s default, and what form (2)'s callers pick on (file banner).
using ThisRobotWords = WordsOf<kRobotId>;

/// The length of a NUL-terminated word, at compile time.
constexpr std::size_t wordLength(const char* s) {
    std::size_t n = 0;
    while (s[n] != '\0') {
        ++n;
    }
    return n;
}

/// True when two NUL-terminated words are the same (a null word equals only a null word).
constexpr bool sameWord(const char* a, const char* b) {
    return (a == nullptr || b == nullptr) ? a == b : std::string_view{a} == std::string_view{b};
}

// The two robots that boot from a calibration record each carry THEIR OWN record name (the boot's step 0
// refuses a record that names another robot — a missing or swapped name here would switch that off).
static_assert(sameWord(kRobotOneWords.recordName, robotRecordName(RobotId::RobotOne)) &&
                  sameWord(kRobotTwoWords.recordName, robotRecordName(RobotId::RobotTwo)),
              "robot one's and robot two's words must carry their own record names (robotRecordName)");

// Every robot's row word is FOUR letters (the claw row's budget, src/common/driver_panel.hpp's
// clawRowWorstLength(), counts four), and every controller word fits its 19 characters.
static_assert(wordLength(kRobotTwoWords.rowWord) == 4 && wordLength(kRobotOneWords.rowWord) == 4 &&
                  wordLength(kClawWords.rowWord) == 4,
              "a row word is four letters (the claw row's width budget counts four)");
static_assert(wordLength(kRobotTwoWords.lcdAbsent) <= 19 && wordLength(kRobotOneWords.lcdAbsent) <= 19 &&
                  wordLength(kClawWords.lcdAbsent) <= 19,
              "the controller line holds 19 characters");

}  // namespace shulib::bench
