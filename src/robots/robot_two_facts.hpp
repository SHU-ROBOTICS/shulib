#pragma once
//
// robot_two_facts.hpp — ROBOT TWO'S FACTS (chunk ROB1-BT, 2026-11-16).
//
// Robot two is robot one's TWIN: the same build, rebuilt (the team lead, 2026-11-13: "IT IS LITERALLY THE
// SAME ROBOT BUILD JUST A MIRRORED ROUTINE" — A4 register HA-256). So this file has robot one's facts — the
// same names, order and stages (src/robots/robot_one_facts.hpp explains each one) — but its OWN values,
// because each robot is wired and measured on its own.
//
// THE SAME BUILD (the team lead, 2026-11-20: "Use robot one's"): every build fact is ROBOT ONE'S VALUE
// (`one::kX.value()`, so a re-measured robot one carries robot two with it), to be confirmed on robot two by
// Bench Tests tile 1 (the ports) and station 13 (each mechanism's direction). Five wiring facts stay OWED until
// station 13's quick pass on robot two confirms them: the side toggles' run directions and the three pistons'
// open levels. Until then robot two's Bench Tests and Calibrate build; its Drive, Auton Tester, Competition and
// Tune REFUSE, one line per owed fact:
//     static assertion failed: robot two: OWED -- the right side toggle's run direction (src/robots/robot_two_facts.hpp)
//
// TO TYPE A FACT (the coordinator, from the builder's answer — never a guess):
//     owed(one::kCascadePorts.value(), Stage::A, "the cascade's motor ports")
//  becomes
//     typed(one::kCascadePorts.placeholder, Pair<int>{10, 1}, Stage::A, "the cascade's motor ports",
//           "the team's builder, <date>, read off robot two")
// The kept placeholder is robot one's own PLACEHOLDER (`.placeholder`, which never changes), not its value:
// typing robot one later can then never make a typed robot-two value "still the placeholder".
//
// test/robot_twin_test.cpp fails the host suite if this file's facts stop matching robot one's list.
// NOTHING HERE HAS RUN ON A ROBOT. PROS-free: the host suite includes it (through the mechanism table).

#include <cstddef>

#include "build_selection.hpp"
#include "robots/robot_one_facts.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench::robot_two_facts {

namespace one = robot_one_facts;  // robot one's facts: robot two's placeholders

// The ledger's machinery is robot one's (one machinery for both robots). Named one by one, so a fact this
// file lacks can never be read from robot one's by mistake.
using one::contains;
using one::Fact;
using one::FactSummary;
using one::FlipperKind;
using one::Level;
using one::Levels;
using one::needs;
using one::owed;
using one::Pair;
using one::PortList;
using one::Rows;
using one::same;
using one::Stage;
using one::summary;
using one::Teeth;
using one::typed;
using one::unpoweredPosition;

// The "gate copy" source is refused here, except in the src build gate's throwaway copy, where the gate
// writes this marker file (robot one's ledger explains why it is a FILE).
#if __has_include("../../build/ROBOT-TWO-GATE-COPY-ONLY")
inline constexpr bool kRefuseGateCopySource = false;
#else
inline constexpr bool kRefuseGateCopySource = true;
#endif

/// A typed fact's source is real: not empty, and not the gate copy's (while that is refused).
constexpr bool realSource(const char* source) {
    return source != nullptr && *source != '\0' && !(kRefuseGateCopySource && contains(source, "gate copy"));
}

// THE CHECK BELOW EVERY FACT — robot one's four checks, for robot two: the name is quoted verbatim; an
// OWED fact stops every robot-two build whose program needs it; a typed fact has a real source; a typed
// value equal to its placeholder must say so.
#define ROBOT_TWO_CHECK(fact, words)                                                                         \
    static_assert(::shulib::bench::robot_one_facts::same((fact).name, words),                                \
                  "robot two: the check below a fact must quote its name verbatim -- " words);               \
    static_assert(!(::shulib::bench::kRobotId == ::shulib::bench::RobotId::RobotTwo &&                       \
                    ::shulib::bench::robot_one_facts::needs(::shulib::bench::kProgram, (fact).needed)) ||    \
                      (fact).typed,                                                                          \
                  "robot two: OWED -- " words " (src/robots/robot_two_facts.hpp)");                          \
    static_assert(!(fact).typed || ::shulib::bench::robot_two_facts::realSource((fact).source),             \
                  "robot two: a typed fact needs a real source (who, how, when; never 'gate copy') -- " words); \
    static_assert(!(fact).typed ||                                                                           \
                      !::shulib::bench::robot_one_facts::same((fact).typedValue, (fact).placeholder) ||       \
                      ::shulib::bench::robot_one_facts::contains((fact).source, "same as the placeholder, checked"), \
                  "robot two: TYPED but still the placeholder -- " words)

// ═══ THE FACTS — robot one's, in robot one's order: robot one's values but five (the same build) ═══

// ── STAGE A — what is plugged where (every program) ──
inline constexpr auto kNonDriveMotorPorts = typed(one::kNonDriveMotorPorts.placeholder, one::kNonDriveMotorPorts.value(), Stage::A,
                                                 "every motor port the Devices screen lists that is not a drive motor",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kNonDriveMotorPorts, "every motor port the Devices screen lists that is not a drive motor");
inline constexpr auto kCascadePorts = typed(one::kCascadePorts.placeholder, one::kCascadePorts.value(), Stage::A, "the cascade's motor ports",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadePorts, "the cascade's motor ports");
inline constexpr auto kCascadeDirections =
    typed(one::kCascadeDirections.placeholder, one::kCascadeDirections.value(), Stage::A, "the cascade motors' directions",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeDirections, "the cascade motors' directions");
inline constexpr auto kCascadeCartridge =
    typed(one::kCascadeCartridge.placeholder, one::kCascadeCartridge.value(), Stage::A, "the cascade motors' cartridge",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeCartridge, "the cascade motors' cartridge");
inline constexpr auto kCascadeTeeth =
    typed(one::kCascadeTeeth.placeholder, one::kCascadeTeeth.value(), Stage::A, "the cascade's gear teeth, motor then spool",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeTeeth, "the cascade's gear teeth, motor then spool");
inline constexpr auto kCascadeStallAmps =
    typed(one::kCascadeStallAmps.placeholder, one::kCascadeStallAmps.value(), Stage::A, "the cascade's starting stall signature, amps",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeStallAmps, "the cascade's starting stall signature, amps");
inline constexpr auto kCascadeStallRpm =
    typed(one::kCascadeStallRpm.placeholder, one::kCascadeStallRpm.value(), Stage::A, "the cascade's starting stall signature, below this motor rpm",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeStallRpm, "the cascade's starting stall signature, below this motor rpm");
inline constexpr auto kCascadeStallTime =
    typed(one::kCascadeStallTime.placeholder, one::kCascadeStallTime.value(), Stage::A, "the cascade's starting stall signature, lasting this long",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeStallTime, "the cascade's starting stall signature, lasting this long");
inline constexpr auto kCascadeCapAmps =
    typed(one::kCascadeCapAmps.placeholder, one::kCascadeCapAmps.value(), Stage::A, "the cascade's starting current cap per motor, amps",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeCapAmps, "the cascade's starting current cap per motor, amps");
inline constexpr auto kFlipperKind = typed(one::kFlipperKind.placeholder, one::kFlipperKind.value(), Stage::A, "the flipper's kind",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kFlipperKind, "the flipper's kind");
inline constexpr auto kSideTogglePorts =
    typed(one::kSideTogglePorts.placeholder, one::kSideTogglePorts.value(), Stage::A, "the side toggles' motor ports, either order",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kSideTogglePorts, "the side toggles' motor ports, either order");
inline constexpr auto kRightTogglePort = typed(one::kRightTogglePort.placeholder, one::kRightTogglePort.value(), Stage::A, "the right side toggle's port",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kRightTogglePort, "the right side toggle's port");
inline constexpr auto kSideToggleCartridge =
    typed(one::kSideToggleCartridge.placeholder, one::kSideToggleCartridge.value(), Stage::A, "the side toggles' cartridge",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kSideToggleCartridge, "the side toggles' cartridge");
inline constexpr auto kSideToggleCapAmps =
    typed(one::kSideToggleCapAmps.placeholder, one::kSideToggleCapAmps.value(), Stage::A, "the side toggles' starting current cap per motor, amps",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kSideToggleCapAmps, "the side toggles' starting current cap per motor, amps");
// Calibrate's two thresholds: robot one's, for the twin's same 10-motor drive (robot one's ledger says why).
inline constexpr auto kCalibratePulseYawFloorDeg =
    typed(one::kCalibratePulseYawFloorDeg.placeholder, 0.6, Stage::A, "Calibrate's pulse yaw floor, degrees",
          "chosen, not measured -- the coordinator, 2026-11-17 (robot one's, for the twin's same drive)");
ROBOT_TWO_CHECK(kCalibratePulseYawFloorDeg, "Calibrate's pulse yaw floor, degrees");
inline constexpr auto kCalibrateRestYawRateDegPerS =
    typed(one::kCalibrateRestYawRateDegPerS.placeholder, 1.0, Stage::A, "Calibrate's rest yaw rate, degrees per second",
          "chosen, not measured -- the coordinator, 2026-11-17 (robot one's, for the twin's same drive)");
ROBOT_TWO_CHECK(kCalibrateRestYawRateDegPerS, "Calibrate's rest yaw rate, degrees per second");
// The autons' drive settings: robot one's, for the same build (robot one's ledger says why).
inline constexpr auto kDriveMaxSpeedInPerS = typed(one::kDriveMaxSpeedInPerS.placeholder, 40.0, Stage::B,
    "the autons' top drive speed, in/s", "chosen, not measured -- the coordinator, 2026-11-17 (robot one's)");
ROBOT_TWO_CHECK(kDriveMaxSpeedInPerS, "the autons' top drive speed, in/s");
inline constexpr auto kDriveMaxTurnRadPerS = typed(one::kDriveMaxTurnRadPerS.placeholder, 4.0, Stage::B,
    "the autons' top turn rate, rad/s", "chosen, not measured -- the coordinator, 2026-11-17 (robot one's)");
ROBOT_TWO_CHECK(kDriveMaxTurnRadPerS, "the autons' top turn rate, rad/s");
inline constexpr auto kDriveHeadingKp = typed(one::kDriveHeadingKp.placeholder, 2.4, Stage::B,
    "the autons' heading correction gain", "chosen, not measured -- the coordinator, 2026-11-17 (robot one's)");
ROBOT_TWO_CHECK(kDriveHeadingKp, "the autons' heading correction gain");
inline constexpr auto kDriveHeadingSettleDeg = typed(one::kDriveHeadingSettleDeg.placeholder, 4.0, Stage::B,
    "the autons' heading arrival, degrees", "chosen, not measured -- the coordinator, 2026-11-17 (robot one's)");
ROBOT_TWO_CHECK(kDriveHeadingSettleDeg, "the autons' heading arrival, degrees");
inline constexpr auto kClawLine = typed(one::kClawLine.placeholder, one::kClawLine.value(), Stage::A, "the claw's 3-wire port",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kClawLine, "the claw's 3-wire port");
inline constexpr auto kFlipperLine =
    typed(one::kFlipperLine.placeholder, one::kFlipperLine.value(), Stage::A, "the front toggle's 3-wire port (the flipper)",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kFlipperLine, "the front toggle's 3-wire port (the flipper)");
inline constexpr auto kRampLine = typed(one::kRampLine.placeholder, one::kRampLine.value(), Stage::A, "the ramp's 3-wire port",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kRampLine, "the ramp's 3-wire port");

// ── STAGE B — how it moves (Drive, the Auton Tester, Competition, Tune) ──
inline constexpr auto kCascadeRows =
    typed(one::kCascadeRows.placeholder, one::kCascadeRows.value(), Stage::B, "the cascade's measured rows, bottom stop to top stop",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeRows, "the cascade's measured rows, bottom stop to top stop");
inline constexpr auto kCascadeLevels =
    typed(one::kCascadeLevels.placeholder, one::kCascadeLevels.value(), Stage::B, "the cascade's levels, by name and stack height",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeLevels, "the cascade's levels, by name and stack height");
inline constexpr auto kCascadeHoldVolts = typed(one::kCascadeHoldVolts.placeholder, one::kCascadeHoldVolts.value(), Stage::B, "the cascade's hold volts",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeHoldVolts, "the cascade's hold volts");
inline constexpr auto kCascadeHomingPushVolts = typed(one::kCascadeHomingPushVolts.placeholder, one::kCascadeHomingPushVolts.value(), Stage::B,
                                                     "the cascade's homing push, volts (negative = down)",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeHomingPushVolts, "the cascade's homing push, volts (negative = down)");
inline constexpr auto kCascadeHomingStallAmps =
    typed(one::kCascadeHomingStallAmps.placeholder, one::kCascadeHomingStallAmps.value(), Stage::B, "the cascade's homing stall signature, amps",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeHomingStallAmps, "the cascade's homing stall signature, amps");
inline constexpr auto kCascadeUnhomedVolts =
    typed(one::kCascadeUnhomedVolts.placeholder, one::kCascadeUnhomedVolts.value(), Stage::B, "the cascade's volts before it is homed",
          "robot one's value, same as the placeholder, checked on robot one: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kCascadeUnhomedVolts, "the cascade's volts before it is homed");
inline constexpr auto kRightToggleDirection =
    owed(one::kRightToggleDirection.value(), Stage::B, "the right side toggle's run direction");
ROBOT_TWO_CHECK(kRightToggleDirection, "the right side toggle's run direction");
inline constexpr auto kLeftToggleDirection =
    owed(one::kLeftToggleDirection.value(), Stage::B, "the left side toggle's run direction");
ROBOT_TWO_CHECK(kLeftToggleDirection, "the left side toggle's run direction");
inline constexpr auto kClawOpenLevel =
    owed(one::kClawOpenLevel.value(), Stage::B, "the claw's line level that OPENS it");
ROBOT_TWO_CHECK(kClawOpenLevel, "the claw's line level that OPENS it");
inline constexpr auto kFlipperOpenLevel = owed(one::kFlipperOpenLevel.value(), Stage::B,
                                               "the front toggle's line level that pushes it OUT (open)");
ROBOT_TWO_CHECK(kFlipperOpenLevel, "the front toggle's line level that pushes it OUT (open)");
inline constexpr auto kRampDownLevel =
    owed(one::kRampDownLevel.value(), Stage::B, "the ramp's line level that puts it DOWN");
ROBOT_TWO_CHECK(kRampDownLevel, "the ramp's line level that puts it DOWN");
inline constexpr auto kClawStrokeTime = typed(one::kClawStrokeTime.placeholder, one::kClawStrokeTime.value(), Stage::B, "the claw's stroke time",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kClawStrokeTime, "the claw's stroke time");
inline constexpr auto kFlipperStrokeTime =
    typed(one::kFlipperStrokeTime.placeholder, one::kFlipperStrokeTime.value(), Stage::B, "the front toggle's stroke time",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kFlipperStrokeTime, "the front toggle's stroke time");
inline constexpr auto kRampStrokeTime = typed(one::kRampStrokeTime.placeholder, one::kRampStrokeTime.value(), Stage::B, "the ramp's stroke time",
          "robot one's value: the same build (the team lead, 2026-11-20: 'use robot one\'s') -- confirm on robot two: Bench Tests tile 1 (ports), station 13 (directions)");
ROBOT_TWO_CHECK(kRampStrokeTime, "the ramp's stroke time");

// ── RECORDED — gating no build (the camera: VIS1-B reads it) ──
inline constexpr auto kCameraPort = owed(one::kCameraPort.value(), Stage::Recorded, "the AI Vision camera's port");
ROBOT_TWO_CHECK(kCameraPort, "the AI Vision camera's port");
inline constexpr auto kCameraMount =
    owed(one::kCameraMount.value(), Stage::Recorded, "where the AI Vision camera sits");
ROBOT_TWO_CHECK(kCameraMount, "where the AI Vision camera sits");

// ═══ THE COUNT, AND THE HELPERS (robot one's, over robot two's facts) ═══════════════════════

/// EVERY fact above, in order (test/robot_twin_test.cpp: the same names and stages as robot one's).
inline constexpr FactSummary kRobotTwoFacts[] = {
    summary(kNonDriveMotorPorts),     summary(kCascadePorts),         summary(kCascadeDirections),
    summary(kCascadeCartridge),       summary(kCascadeTeeth),         summary(kCascadeStallAmps),
    summary(kCascadeStallRpm),        summary(kCascadeStallTime),     summary(kCascadeCapAmps),
    summary(kFlipperKind),            summary(kSideTogglePorts),      summary(kRightTogglePort),
    summary(kSideToggleCartridge),    summary(kSideToggleCapAmps),    summary(kCalibratePulseYawFloorDeg),
    summary(kCalibrateRestYawRateDegPerS), summary(kDriveMaxSpeedInPerS), summary(kDriveMaxTurnRadPerS),
    summary(kDriveHeadingKp),       summary(kDriveHeadingSettleDeg),  summary(kClawLine),
    summary(kFlipperLine),            summary(kRampLine),             summary(kCascadeRows),
    summary(kCascadeLevels),          summary(kCascadeHoldVolts),     summary(kCascadeHomingPushVolts),
    summary(kCascadeHomingStallAmps), summary(kCascadeUnhomedVolts),  summary(kRightToggleDirection),
    summary(kLeftToggleDirection),    summary(kClawOpenLevel),        summary(kFlipperOpenLevel),
    summary(kRampDownLevel),          summary(kClawStrokeTime),       summary(kFlipperStrokeTime),
    summary(kRampStrokeTime),         summary(kCameraPort),           summary(kCameraMount),
};

/// How many facts the ledger holds.
inline constexpr int kRobotTwoFactCount = static_cast<int>(sizeof kRobotTwoFacts / sizeof kRobotTwoFacts[0]);

/// How many are still OWED.
constexpr int owedCount() {
    int n = 0;
    for (const FactSummary& f : kRobotTwoFacts) {
        n += f.typed ? 0 : 1;
    }
    return n;
}
inline constexpr int kRobotTwoOwedCount = owedCount();

/// How many program `p` needs that are still OWED (zero on every program robot two's build links).
constexpr int owedFor(Program p) {
    int n = 0;
    for (const FactSummary& f : kRobotTwoFacts) {
        n += (!f.typed && needs(p, f.needed)) ? 1 : 0;
    }
    return n;
}

/// The cascade's bottom stop: its first measured row (where homing puts it).
constexpr units::Length cascadeBottom() { return kCascadeRows.value().row[0].height; }

/// The cascade's top stop: its last measured row.
constexpr units::Length cascadeTop() { return kCascadeRows.value().row[one::kCascadeRowCount - 1].height; }

/// The cascade's JAM stall signature — the declaration's `.stall`.
constexpr mechanism::Stall cascadeStall() {
    return mechanism::Stall{.amps = kCascadeStallAmps.value(),
                            .belowMotorRpm = kCascadeStallRpm.value(),
                            .lasting = kCascadeStallTime.value()};
}

/// The cascade's HOMING stall signature — its own amps, the jam signature's speed and time.
constexpr mechanism::Stall cascadeHomingStall() {
    return mechanism::Stall{.amps = kCascadeHomingStallAmps.value(),
                            .belowMotorRpm = kCascadeStallRpm.value(),
                            .lasting = kCascadeStallTime.value()};
}

/// The level in slot `i` (0..5).
constexpr const Level& lv(std::size_t i) { return kCascadeLevels.value().level[i]; }

/// The LEFT side toggle's port: the side-toggle port that is not the right one's (0 — refused by the
/// wiring — when the right one's is neither).
constexpr int leftTogglePort() {
    const Pair<int>& both = kSideTogglePorts.value();
    const int right = kRightTogglePort.value();
    return right == both.first ? both.second : (right == both.second ? both.first : 0);
}

}  // namespace shulib::bench::robot_two_facts
