// mechanism/spec.hpp — THE BUILD CHECK at run time (chunk MECH-A1, the MECH brief's §3.1 "each spec
// refusal at run time").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * ONE PROPERTY, ONE REASON. Every case starts from a declaration that is valid (asserted) and
//     breaks exactly ONE property; specRefusal() must return exactly that Refusal, and its sentence.
//     The cases are collected, and the test FAILS if any enumerator of `Refusal` is never produced —
//     so a reason appended to spec.hpp without a case here is a red test, not a silent gap.
//   * THE ORDER. Several properties broken at once, then fixed one at a time ("peeling"): each fix
//     must reveal the NEXT reason in check order. This is what makes a swap of two checks visible
//     (mutation M7) — a one-property case alone cannot see the order.
//   * THE BOUNDARIES of every numeric rule: 2.49 A accepted and 2.5 A refused (N10); 0.1 rpm accepted
//     and 0 refused; a linkage limit exactly at its formula's reach accepted and 0.01 past refused; a
//     preset exactly AT a limit accepted; a 23-character name accepted and 24 refused; an absolute
//     reading 0 and 359.9 accepted, 360 refused; four motor roles, three stages, a 2-point table, 0 s
//     lasting and settle, and +-12 V exactly accepted (MECH-A1's review).
//   * BOTH SIDES of every two-sided rule, and a NaN wherever a number is range-checked (MECH-A1's
//     review: a preset below its range and a home above it had no case; every range check is "not
//     inside", so a NaN is refused like any number outside).
//   * THE SENTENCES: every reason has one, non-empty and unique; None has none; a byte past the list
//     reads "an unknown refusal".
//   * THE CURRENT CAP (MECH-S-A, the MECH-S brief's CA1–CA3): a written cap below 0, above 2.5 or NaN
//     refused, and one above 0 but under one milliamp (0.0009, 0.0004 — refusal 73, MECH-S-A's review: a V5
//     limit is whole mA, so it would go out as 0 mA) — 2.5 and 0.001 exactly accepted, and NO cap, which a
//     cap WRITTEN as 0 or -0.0 is too (0 = none stated: the brief's CA1 "≤ 0" is §1.1's "0 = none" here — a
//     logged delta); every written signature (a power's, an arm's, its homing's, a flywheel's) AT the cap
//     refused, 0.01 A below accepted; the reasons LAST in the check order (an old reason still wins over
//     them; the cap's own range, 71 then 73, before the signature's 72) and last in the list (71, 72, 73).
// The same reasons AT BUILD, through a declaration, are test/compile_refusals/'s (MECH brief §3.0).

#include "doctest.h"

#include <cstddef>
#include <cstring>
#include <functional>
#include <limits>
#include <set>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "shulib/mechanism/spec.hpp"

using shulib::mechanism::Confirm;
using shulib::mechanism::Geometry;
using shulib::mechanism::Gravity;
using shulib::mechanism::HomeMethod;
using shulib::mechanism::Kind;
using shulib::mechanism::MechanismSpec;
using shulib::mechanism::Refusal;
using shulib::mechanism::refusalText;
using shulib::mechanism::SensorKind;
using shulib::mechanism::specProblem;
using shulib::mechanism::specRefusal;
using shulib::mechanism::Units;

namespace {

namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;

/// A number that is not one: every range check is written "not inside", so it must be refused.
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The valid declarations every case starts from — one per shape a rule reads.
MechanismSpec arm() { return rx::arm.spec(); }             // position, rotary, rests at its stop
MechanismSpec lift() { return rx::lift.spec(); }           // position, spool, switch-homed
MechanismSpec flywheel() { return rx::flywheel.spec(); }   // speed
MechanismSpec intake() { return rx::intake.spec(); }       // power with a distance piece sensor
MechanismSpec hood() { return rx::hood.spec(2); }          // a three-position pneumatic on two lines
MechanismSpec wrist() { return ry::wrist.spec(); }         // position, absolute sensor
MechanismSpec swing() { return ry::swing.spec(); }         // position, stall-homed
MechanismSpec dr4b() { return ry::dr4b.spec(); }           // position, linkage
MechanismSpec tilt() { return ry::tilt.spec(); }           // position, measured table
MechanismSpec roller() { return ry::roller.spec(); }       // power with an optical piece sensor
MechanismSpec claw() { return r2::claw.spec(1); }          // robot two's claw: two positions, one line

struct Case {
    const char* what;
    std::function<MechanismSpec()> make;
    Refusal expected;
};

void setName(MechanismSpec& s, const char* text) {
    std::memset(s.name, 0, sizeof s.name);
    for (std::size_t i = 0; text[i] != '\0' && i < sizeof s.name; ++i) {
        s.name[i] = text[i];
    }
}

/// Every one-property case (header). Each lambda breaks ONE property of a valid base.
std::vector<Case> oneProperty() {
    std::vector<Case> c;
    const auto add = [&](const char* what, std::function<MechanismSpec()> make, Refusal r) {
        c.push_back(Case{what, std::move(make), r});
    };
    add("an empty name", [] { auto s = arm(); s.name[0] = '\0'; return s; }, Refusal::AMechanismHasNoName);
    add("a name with no NUL in 24 bytes", [] { auto s = arm(); std::memset(s.name, 'x', sizeof s.name); return s; },
        Refusal::AMechanismNameDoesNotFit23Characters);
    add("an UNSET kind", [] { auto s = arm(); s.kind = static_cast<Kind>(0); return s; }, Refusal::AMechanismSKindIsUNSET);
    add("a kind byte past the enum", [] { auto s = arm(); s.kind = static_cast<Kind>(9); return s; },
        Refusal::AMechanismSKindIsUNSET);
    add("a motor kind with no motor roles", [] { auto s = arm(); s.motorCount = 0; return s; },
        Refusal::AMotorMechanismNeeds14MotorRoles);
    add("a motor kind with five motor roles", [] { auto s = arm(); s.motorCount = 5; return s; },
        Refusal::AMotorMechanismNeeds14MotorRoles);
    add("a motor kind with a 3-wire line", [] { auto s = arm(); s.lineCount = 1; return s; },
        Refusal::AMotorMechanismHas3WireLineRolesMakeTwoMechanisms);
    add("no cartridge", [] { auto s = arm(); s.cartridge = static_cast<shulib::hal::MotorCartridge>(0); return s; },
        Refusal::AMotorMechanismSCartridgeIsUNSET);
    add("no safe brake mode", [] { auto s = intake(); s.safeMode = static_cast<shulib::config::SafeBrakeMode>(0); return s; },
        Refusal::AMotorMechanismSSafeBrakeModeIsUNSET);
    add("a pneumatic with a motor role", [] { auto s = hood(); s.motorCount = 1; return s; },
        Refusal::APneumaticMechanismHasMotorRoles);
    add("a pneumatic with no line", [] { auto s = hood(); s.lineCount = 0; return s; },
        Refusal::APneumaticMechanismNeeds12LineRoles);
    add("a pneumatic with three lines", [] { auto s = hood(); s.lineCount = 3; return s; },
        Refusal::APneumaticMechanismNeeds12LineRoles);
    add("a distance sensor in the angle slot", [] { auto s = arm(); s.angleSensor.kind = SensorKind::Distance; return s; },
        Refusal::TheAngleSensorRoleIsNotARotationSensor);
    add("an angle sensor on a power mechanism", [] { auto s = intake(); s.angleSensor.kind = SensorKind::Rotation; return s; },
        Refusal::AnAngleSensorRoleOnAMechanismThatIsNotPositionOrSpeed);
    add("a rotation sensor in the home-switch slot", [] { auto s = lift(); s.homeSwitch.kind = SensorKind::Rotation; return s; },
        Refusal::TheHomeSwitchRoleIsNotALimitSwitch);
    add("a home switch on a flywheel", [] { auto s = flywheel(); s.homeSwitch.kind = SensorKind::Switch; return s; },
        Refusal::AHomeSwitchRoleOnAMechanismThatIsNotPosition);
    add("a rotation sensor as a piece sensor", [] { auto s = intake(); s.pieceSensor.kind = SensorKind::Rotation; return s; },
        Refusal::APieceSensorCannotBeARotationSensor);
    add("a distance piece sensor with no threshold", [] { auto s = intake(); s.pieceSensor.seesWithinIn = 0.0; return s; },
        Refusal::ADistancePieceSensorNeedsItsSeesWithinDistance0);
    add("an optical threshold of 0", [] { auto s = roller(); s.pieceSensor.seesProximity = 0.0; return s; },
        Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01);
    add("an optical threshold past 1", [] { auto s = roller(); s.pieceSensor.seesProximity = 1.01; return s; },
        Refusal::AnOpticalPieceSensorNeedsItsProximityThresholdIn01);
    add("maxVolts 0", [] { auto s = intake(); s.power.maxVolts = 0.0; return s; }, Refusal::PowerMaxVoltsIsNotIn012);
    add("maxVolts past 12", [] { auto s = intake(); s.power.maxVolts = 12.01; return s; }, Refusal::PowerMaxVoltsIsNotIn012);
    add("UNSET units", [] { auto s = arm(); s.position.units = static_cast<Units>(0); return s; }, Refusal::PositionUnitsUNSET);
    add("no gearing", [] { auto s = arm(); s.position.gearRatio = 0.0; return s; }, Refusal::PositionGearRatioMustBe0);
    add("a rotary arm in inches", [] { auto s = arm(); s.position.units = Units::Inches; return s; },
        Refusal::PositionARotaryMechanismIsInDegrees);
    add("a spool lift in degrees", [] { auto s = lift(); s.position.units = Units::Degrees; return s; },
        Refusal::PositionALiftIsInInches);
    add("a spool with no diameter", [] { auto s = lift(); s.position.spoolDiameterIn = 0.0; return s; },
        Refusal::PositionSpoolDiameterMustBe0);
    add("a linkage with no arm length", [] { auto s = dr4b(); s.position.armLengthIn = 0.0; return s; },
        Refusal::PositionArmLengthMustBe0);
    add("a linkage with 0 stages", [] { auto s = dr4b(); s.position.armStages = 0; return s; }, Refusal::PositionAnArmNeeds13Stages);
    add("a linkage with 4 stages", [] { auto s = dr4b(); s.position.armStages = 4; return s; }, Refusal::PositionAnArmNeeds13Stages);
    add("a linkage whose top limit reaches 0.01 in past its formula (10 + 2 x 14 = 38)",
        [] { auto s = dr4b(); s.position.maxPosition = 38.01; return s; },
        Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul);
    add("a linkage whose bottom limit reaches past its formula (10 - 28 = -18)",
        [] { auto s = dr4b(); s.position.minPosition = -18.01; return s; },
        Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul);
    add("a linkage whose pivot height is not a number", [] { auto s = dr4b(); s.position.pivotHeightIn = kNaN; return s; },
        Refusal::PositionAnArmSTravelLimitsReachPastItsLinkageTheHeightFormul);
    add("a table of 1 point", [] { auto s = tilt(); s.position.tablePoints = 1; return s; }, Refusal::PositionALookupTableNeeds28Points);
    add("a table of 9 points", [] { auto s = tilt(); s.position.tablePoints = 9; return s; }, Refusal::PositionALookupTableNeeds28Points);
    add("a table whose heights fall", [] { auto s = tilt(); s.position.table[2][0] = 5.0; return s; },
        Refusal::PositionTheLookupTableIsNotStrictlyIncreasing);
    add("a table whose angles repeat", [] { auto s = tilt(); s.position.table[2][1] = 30.0; return s; },
        Refusal::PositionTheLookupTableIsNotStrictlyIncreasing);
    add("limits min == max", [] { auto s = arm(); s.position.maxPosition = s.position.minPosition; return s; },
        Refusal::PositionTravelLimitsMissingOrMinMax);
    add("no profile speed", [] { auto s = arm(); s.position.maxVelocity = 0.0; return s; },
        Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0);
    add("no profile acceleration", [] { auto s = arm(); s.position.maxAcceleration = 0.0; return s; },
        Refusal::PositionTheProfileNeedsMaxVelocityAndMaxAcceleration0);
    add("a tolerance of 0", [] { auto s = arm(); s.position.tolerance = 0.0; return s; }, Refusal::PositionToleranceMustBe0);
    add("a negative settle time (its OWN reason since MECH-A1's review, never the tolerance's)",
        [] { auto s = arm(); s.position.settleTime = -0.1; return s; }, Refusal::PositionTheSettleTimeIsNegative);
    add("UNSET gravity", [] { auto s = arm(); s.position.gravity = static_cast<Gravity>(0); return s; }, Refusal::PositionGravityUNSET);
    add("UNSET homing", [] { auto s = arm(); s.position.home = static_cast<HomeMethod>(0); return s; },
        Refusal::PositionHomingMethodUNSET);
    add("switch homing with no switch", [] { auto s = lift(); s.homeSwitch.kind = SensorKind::None; return s; },
        Refusal::PositionHomingBySwitchNeedsAHomeSwitchRole);
    add("absolute homing with no angle sensor", [] { auto s = wrist(); s.angleSensor.kind = SensorKind::None; return s; },
        Refusal::PositionAbsoluteHomingNeedsAnAngleSensorRotationRole);
    add("switch homing with no push", [] { auto s = lift(); s.position.homeVolts = 0.0; return s; },
        Refusal::PositionHomingNeedsHomeVolts);
    add("STALL homing with no push (each method of the pair, MECH-A1's review)",
        [] { auto s = swing(); s.position.homeVolts = 0.0; return s; }, Refusal::PositionHomingNeedsHomeVolts);
    add("stall homing with no signature", [] { auto s = swing(); s.position.homeStall = {}; return s; },
        Refusal::PositionStallHomingNeedsItsStallSignature);
    add("a switch-home at the bottom pushing UP", [] { auto s = lift(); s.position.homeVolts = 3.0; return s; },
        Refusal::PositionHomingPushesAwayFromItsHomeEnd);
    add("a stall-home at the bottom pushing UP", [] { auto s = swing(); s.position.homeVolts = 3.0; return s; },
        Refusal::PositionHomingPushesAwayFromItsHomeEnd);
    add("a stall-home at the TOP pushing DOWN",
        [] { auto s = swing(); s.position.homePosition = s.position.maxPosition; s.position.homeVolts = -3.0; return s; },
        Refusal::PositionHomingPushesAwayFromItsHomeEnd);
    add("a home position below the range", [] { auto s = arm(); s.position.homePosition = -1.0; return s; },
        Refusal::PositionTheHomePositionIsOutsideTheTravelLimits);
    add("a home position 0.01 ABOVE the range (both sides, MECH-A1's review)",
        [] { auto s = arm(); s.position.homePosition = 160.01; return s; },
        Refusal::PositionTheHomePositionIsOutsideTheTravelLimits);
    add("a home position that is not a number", [] { auto s = arm(); s.position.homePosition = kNaN; return s; },
        Refusal::PositionTheHomePositionIsOutsideTheTravelLimits);
    add("an arm with no stall signature", [] { auto s = arm(); s.position.stall = {}; return s; },
        Refusal::PositionTheStallSignatureIsUNSET);
    add("seven presets", [] { auto s = arm(); s.position.presetCount = 7; return s; }, Refusal::PositionMorePresetsThanFit);
    add("a preset with no name", [] { auto s = arm(); s.position.presets[1].name[0] = '\0'; return s; },
        Refusal::PositionAPresetHasNoName);
    add("a preset 0.01 past the top limit", [] { auto s = arm(); s.position.presets[2].value = 160.01; return s; },
        Refusal::PositionAPresetIsOutsideTheTravelLimits);
    add("a preset 0.01 BELOW the bottom limit (both sides, MECH-A1's review)",
        [] { auto s = arm(); s.position.presets[0].value = -0.01; return s; }, Refusal::PositionAPresetIsOutsideTheTravelLimits);
    add("a preset that is not a number", [] { auto s = arm(); s.position.presets[1].value = kNaN; return s; },
        Refusal::PositionAPresetIsOutsideTheTravelLimits);
    add("a speed with no gearing", [] { auto s = flywheel(); s.speed.gearRatio = 0.0; return s; },
        Refusal::SpeedGearRatioAndMaxRpmMustBe0);
    add("a speed with no free speed", [] { auto s = flywheel(); s.speed.maxRpm = 0.0; return s; },
        Refusal::SpeedGearRatioAndMaxRpmMustBe0);
    add("no at-speed band", [] { auto s = flywheel(); s.speed.atSpeedBandRpm = 0.0; return s; }, Refusal::SpeedTheAtSpeedBandMustBe0);
    add("a negative at-speed lasting (its OWN reason, never the band's)",
        [] { auto s = flywheel(); s.speed.atSpeedTime = -0.1; return s; }, Refusal::SpeedTheAtSpeedLastingIsNegative);
    add("a flywheel with no stall signature", [] { auto s = flywheel(); s.speed.stall = {}; return s; },
        Refusal::SpeedTheStallSignatureIsUNSET);
    add("seven speed presets", [] { auto s = flywheel(); s.speed.presetCount = 7; return s; }, Refusal::SpeedMorePresetsThanFit);
    add("a speed preset with no name", [] { auto s = flywheel(); s.speed.presets[0].name[0] = '\0'; return s; },
        Refusal::SpeedAPresetHasNoName);
    add("a speed preset 0.01 past the free speed", [] { auto s = flywheel(); s.speed.presets[0].value = 3600.01; return s; },
        Refusal::SpeedAPresetIsPastMaxRpm);
    add("a speed preset 0.01 past the free speed, reversed", [] { auto s = flywheel(); s.speed.presets[0].value = -3600.01; return s; },
        Refusal::SpeedAPresetIsPastMaxRpm);
    add("one position", [] { auto s = hood(); s.pneumatic.positionCount = 1; return s; }, Refusal::Pneumatic2Or3Positions);
    add("four positions", [] { auto s = hood(); s.pneumatic.positionCount = 4; return s; }, Refusal::Pneumatic2Or3Positions);
    add("a start index past the positions", [] { auto s = hood(); s.pneumatic.start = 3; return s; },
        Refusal::PneumaticStartSafeIsNotOneOfItsPositions);
    add("a safe index past the positions", [] { auto s = hood(); s.pneumatic.safe = 3; return s; },
        Refusal::PneumaticStartSafeIsNotOneOfItsPositions);
    add("no move time", [] { auto s = hood(); s.pneumatic.actuationTime = 0.0; return s; }, Refusal::PneumaticActuationTimeMustBe0);
    add("a negative confirm window (its OWN reason, never the move time's)",
        [] { auto s = hood(); s.pneumatic.confirmWindow = -0.1; return s; }, Refusal::PneumaticTheConfirmWindowIsNegative);
    add("a position with no name", [] { auto s = hood(); s.pneumatic.positions[1].name[0] = '\0'; return s; },
        Refusal::PneumaticAPositionHasNoName);
    add("two positions alike on both lines", [] { auto s = hood(); s.pneumatic.positions[1].levels[0] = false; return s; },
        Refusal::PneumaticTwoPositionsHaveTheSameLineLevels);
    add("two positions alike on the ONE wired line (they differ only on an unwired one)",
        [] { auto s = claw(); s.pneumatic.positions[1].levels[0] = false; s.pneumatic.positions[1].levels[1] = true; return s; },
        Refusal::PneumaticTwoPositionsHaveTheSameLineLevels);
    add("a start naming no position", [] { auto s = hood(); s.pneumatic.startNamed = false; return s; },
        Refusal::PneumaticItsStartNamesNoneOfItsPositions);
    add("a safe naming no position", [] { auto s = hood(); s.pneumatic.safeNamed = false; return s; },
        Refusal::PneumaticItsSafeNamesNoneOfItsPositions);
    add("a preset name past 15 characters", [] { auto s = arm(); s.presetNameTooLong = true; return s; },
        Refusal::APresetOrPositionNameDoesNotFit15Characters);
    add("two arm presets alike", [] { auto s = arm(); std::strcpy(s.position.presets[1].name, "stow"); s.position.presets[1].value = 0.0; return s; },
        Refusal::TwoPresetsOrPositionsShareAName);
    add("two flywheel presets alike",
        [] { auto s = flywheel(); s.speed.presets[1] = s.speed.presets[0]; s.speed.presets[1].value = 100.0; s.speed.presetCount = 2; return s; },
        Refusal::TwoPresetsOrPositionsShareAName);
    add("two pneumatic positions alike by name", [] { auto s = hood(); std::strcpy(s.pneumatic.positions[2].name, "down"); return s; },
        Refusal::TwoPresetsOrPositionsShareAName);
    add("an absolute reading of 360", [] { auto s = wrist(); s.position.absoluteSensorAtZeroDeg = 360.0; return s; },
        Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360);
    add("a negative absolute reading", [] { auto s = wrist(); s.position.absoluteSensorAtZeroDeg = -0.1; return s; },
        Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360);
    add("an arm stall signature at 2.5 A (N10)", [] { auto s = arm(); s.position.stall.amps = 2.5; return s; },
        Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    add("a flywheel stall signature at 3.0 A", [] { auto s = flywheel(); s.speed.stall.amps = 3.0; return s; },
        Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    add("a power's optional signature at 2.5 A", [] { auto s = intake(); s.power.stall = {.amps = 2.5, .speedRadS = 1.0, .time = 0.5}; return s; },
        Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    add("a HOMING signature at 2.5 A", [] { auto s = swing(); s.position.homeStall.amps = 2.5; return s; },
        Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    add("an arm signature at 0 rpm (N10)", [] { auto s = arm(); s.position.stall.speedRadS = 0.0; return s; },
        Refusal::AStallSignatureNeedsBelowMotorRpmAbove0);
    add("a homing signature at 0 rpm", [] { auto s = swing(); s.position.homeStall.speedRadS = 0.0; return s; },
        Refusal::AStallSignatureNeedsBelowMotorRpmAbove0);
    add("a power signature lasting 0 (MECH-A1)", [] { auto s = intake(); s.power.stall = {.amps = 2.0, .speedRadS = 1.0, .time = 0.0}; return s; },
        Refusal::AStallSignatureNeedsLastingAbove0);
    add("a HOMING signature lasting 0: it would zero on its own spin-up", [] { auto s = swing(); s.position.homeStall.time = 0.0; return s; },
        Refusal::AStallSignatureNeedsLastingAbove0);
    add("an arm signature lasting 0", [] { auto s = arm(); s.position.stall.time = 0.0; return s; },
        Refusal::AStallSignatureNeedsLastingAbove0);
    // MECH-S-A (CA1): a WRITTEN current cap outside (0, 2.5] — each side, and a NaN ("not inside").
    add("a current cap of -0.5 A", [] { auto s = lift(); s.position.stall.amps = 1.0; s.currentCapAmps = -0.5; return s; },
        Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    add("a current cap of 2.51 A", [] { auto s = arm(); s.position.stall.amps = 1.0; s.currentCapAmps = 2.51; return s; },
        Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    add("a current cap of 3.0 A on a flywheel", [] { auto s = flywheel(); s.currentCapAmps = 3.0; return s; },
        Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    add("a current cap that is not a number", [] { auto s = intake(); s.currentCapAmps = kNaN; return s; },
        Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
    // MECH-S-A review (refusal 73): above 0 but under one milliamp — the motor would be sent 0 mA. 0.0009 is
    // the edge's near side (0.001 itself is accepted below), 0.0004 the reviewer's case, the smallest
    // positive double the far end.
    add("a current cap of 0.0009 A (under one milliamp)", [] { auto s = arm(); s.position.stall.amps = 0.0005; s.currentCapAmps = 0.0009; return s; },
        Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    add("a current cap of 0.0004 A (it would be sent as 0 mA)", [] { auto s = intake(); s.currentCapAmps = 0.0004; return s; },
        Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    add("a current cap of the smallest positive double", [] { auto s = flywheel(); s.currentCapAmps = std::numeric_limits<double>::denorm_min(); return s; },
        Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    // MECH-S-A (CA2): every WRITTEN signature AT the cap — it could never trip (a capped motor never draws more).
    add("an arm's stall signature AT its cap", [] { auto s = arm(); s.position.stall.amps = 1.2; s.currentCapAmps = 1.2; return s; },
        Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    add("a lift's stall signature ABOVE its cap", [] { auto s = lift(); s.currentCapAmps = 1.2; return s; },  // 2.0 A
        Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    add("a HOMING signature AT the cap (the move's below it)",
        [] { auto s = swing(); s.position.stall.amps = 0.5; s.position.homeStall.amps = 1.0; s.currentCapAmps = 1.0; return s; },
        Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    add("a flywheel's signature AT its cap", [] { auto s = flywheel(); s.speed.stall.amps = 1.5; s.currentCapAmps = 1.5; return s; },
        Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    add("a power's optional signature AT its cap",
        [] { auto s = intake(); s.power.stall = {.amps = 0.8, .speedRadS = 1.0, .time = 0.5}; s.currentCapAmps = 0.8; return s; },
        Refusal::AStallSignatureMustBeBelowTheCurrentCap);
    add("a power signature with its amps left out (MECH-A1)", [] { auto s = intake(); s.power.stall = {.amps = 0.0, .speedRadS = 1.0, .time = 0.5}; return s; },
        Refusal::AStallSignatureNeedsAmpsAbove0);
    add("a homing signature with negative amps", [] { auto s = swing(); s.position.homeStall.amps = -1.5; return s; },
        Refusal::AStallSignatureNeedsAmpsAbove0);
    add("a lift with no shape (MECH-A1)", [] { auto s = lift(); s.position.geometry = static_cast<Geometry>(0); return s; },
        Refusal::PositionGeometryUNSET);
    // MECH-A1's review: the voltages a position mechanism states, each within what a motor is commanded.
    add("a homing push of -12.01 V", [] { auto s = lift(); s.position.homeVolts = -12.01; return s; },
        Refusal::PositionTheHomingPushIsPast12Volts);
    add("a homing push of +12.01 V (checked before the push's direction)",
        [] { auto s = lift(); s.position.homeVolts = 12.01; return s; }, Refusal::PositionTheHomingPushIsPast12Volts);
    add("a homing push that is not a number", [] { auto s = swing(); s.position.homeVolts = kNaN; return s; },
        Refusal::PositionTheHomingPushIsPast12Volts);
    add("a gravity hold of 12.01 V", [] { auto s = arm(); s.position.kG = 12.01; return s; },
        Refusal::PositionTheGravityHoldIsPast12Volts);
    add("a gravity hold of -12.01 V", [] { auto s = lift(); s.position.kG = -12.01; return s; },
        Refusal::PositionTheGravityHoldIsPast12Volts);
    add("a gravity hold that is not a number", [] { auto s = arm(); s.position.kG = kNaN; return s; },
        Refusal::PositionTheGravityHoldIsPast12Volts);
    add("unhomedVolts of 12.01", [] { auto s = tilt(); s.position.unhomedVolts = 12.01; return s; },
        Refusal::PositionUnhomedVoltsIsNotIn0To12);
    add("a negative unhomedVolts", [] { auto s = tilt(); s.position.unhomedVolts = -0.01; return s; },
        Refusal::PositionUnhomedVoltsIsNotIn0To12);
    add("unhomedVolts that is not a number", [] { auto s = wrist(); s.position.unhomedVolts = kNaN; return s; },
        Refusal::PositionUnhomedVoltsIsNotIn0To12);
    // MECH-A3: an angle sensor absolute within ONE turn (HA-186) cannot home a whole turn of travel.
    add("absolute homing over a whole output turn: an arm's -180..180", [] {
        auto s = wrist();
        s.position.minPosition = -180.0;
        s.position.maxPosition = 180.0;
        return s;
    }, Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn);
    add("absolute homing on a SPOOL lift's shaft (24 in on a 1.5 in spool: 5.1 turns)", [] {
        auto s = lift();
        s.position.home = HomeMethod::Absolute;
        s.position.absoluteSensorAtZeroDeg = 0.0;
        s.homeSwitch = {};
        s.angleSensor.kind = SensorKind::Rotation;
        return s;
    }, Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn);
    add("absolute homing on a measured table whose angles span 400 deg", [] {
        auto s = tilt();
        s.position.table[2][1] = 400.0;
        s.position.home = HomeMethod::Absolute;
        s.position.homePosition = 0.0;
        s.angleSensor.kind = SensorKind::Rotation;
        return s;
    }, Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn);
    return c;
}

/// Every valid shape at the exact edges the rules allow (each must be None).
std::vector<Case> boundariesAccepted() {
    std::vector<Case> c;
    const auto add = [&](const char* what, std::function<MechanismSpec()> make) {
        c.push_back(Case{what, std::move(make), Refusal::None});
    };
    add("a 23-character name", [] { auto s = arm(); setName(s, "abcdefghijklmnopqrstuvw"); return s; });
    add("a stall signature at 2.49 A", [] { auto s = arm(); s.position.stall.amps = 2.49; return s; });
    // MECH-S-A (CA1, CA2): a cap of 2.5 A exactly (the motor's own) and none; each signature 0.01 A below it.
    add("a current cap of exactly 2.5 A (signature 2.49)", [] { auto s = arm(); s.position.stall.amps = 2.49; s.currentCapAmps = 2.5; return s; });
    add("no current cap (0 = none stated)", [] { auto s = lift(); s.currentCapAmps = 0.0; return s; });
    add("a current cap WRITTEN as -0.0: no cap, like 0", [] { auto s = lift(); s.currentCapAmps = -0.0; return s; });
    add("a current cap of exactly 0.001 A, one milliamp (refusal 73's edge)", [] { auto s = arm(); s.position.stall.amps = 0.0005; s.currentCapAmps = 0.001; return s; });
    add("a tiny current cap, 0.01 A, over a 0.005 A signature", [] { auto s = arm(); s.position.stall.amps = 0.005; s.currentCapAmps = 0.01; return s; });
    add("an arm's signature 0.01 A below its cap", [] { auto s = arm(); s.position.stall.amps = 1.19; s.currentCapAmps = 1.2; return s; });
    add("a homing signature 0.01 A below its cap",
        [] { auto s = swing(); s.position.stall.amps = 0.5; s.position.homeStall.amps = 0.99; s.currentCapAmps = 1.0; return s; });
    add("a flywheel's signature 0.01 A below its cap", [] { auto s = flywheel(); s.speed.stall.amps = 1.49; s.currentCapAmps = 1.5; return s; });
    add("a power's signature 0.01 A below its cap",
        [] { auto s = intake(); s.power.stall = {.amps = 0.79, .speedRadS = 1.0, .time = 0.5}; s.currentCapAmps = 0.8; return s; });
    add("a capped power with no signature at all", [] { auto s = intake(); s.currentCapAmps = 0.5; return s; });
    add("a stall signature at 0.1 motor rpm", [] { auto s = arm(); s.position.stall.speedRadS = 0.1 * 2.0 * 3.141592653589793 / 60.0; return s; });
    add("a linkage top limit exactly at its formula's reach", [] { auto s = dr4b(); s.position.maxPosition = 38.0; return s; });
    add("a linkage bottom limit exactly at its formula's reach", [] { auto s = dr4b(); s.position.minPosition = -18.0; return s; });
    add("a preset exactly at the top limit", [] { auto s = arm(); s.position.presets[2].value = 160.0; return s; });
    add("a preset exactly at the bottom limit", [] { auto s = arm(); s.position.presets[0].value = 0.0; return s; });
    add("a speed preset exactly at the free speed", [] { auto s = flywheel(); s.speed.presets[0].value = 3600.0; return s; });
    add("an absolute reading of 0", [] { auto s = wrist(); s.position.absoluteSensorAtZeroDeg = 0.0; return s; });
    add("an absolute reading of 359.9", [] { auto s = wrist(); s.position.absoluteSensorAtZeroDeg = 359.9; return s; });
    add("an optical threshold of exactly 1", [] { auto s = roller(); s.pieceSensor.seesProximity = 1.0; return s; });
    add("maxVolts exactly 12", [] { auto s = intake(); s.power.maxVolts = 12.0; return s; });
    add("a table of exactly 8 points", [] {
        auto s = tilt();
        for (std::size_t i = 0; i < 8; ++i) {
            s.position.table[i][0] = 2.0 * static_cast<double>(i);
            s.position.table[i][1] = 10.0 * static_cast<double>(i);
        }
        s.position.tablePoints = 8;
        return s;
    });
    add("a switch home strictly INSIDE the range pushing up", [] { auto s = lift(); s.position.homePosition = 4.0; s.position.homeVolts = 3.0; return s; });
    add("a switch home strictly INSIDE the range pushing down", [] { auto s = lift(); s.position.homePosition = 4.0; s.position.homeVolts = -3.0; return s; });
    add("a stall-home at the TOP pushing UP", [] { auto s = swing(); s.position.homePosition = s.position.maxPosition; s.position.homeVolts = 3.0; return s; });
    add("a never-homed tilt whose home field is outside its range (not read)", [] { auto s = tilt(); s.position.homePosition = -100.0; return s; });
    add("a power with NO signature (optional on a power, N10)", [] { auto s = intake(); s.power.stall = {}; return s; });
    add("a pneumatic with a zero confirm window", [] { auto s = hood(); s.pneumatic.confirmWindow = 0.0; return s; });
    // MECH-A1's review: the far edge of every count and the zero of every "not negative".
    add("FOUR motor roles (the most)", [] { auto s = intake(); s.motorCount = 4; return s; });
    add("SIX presets (the most)", [] {
        auto s = arm();
        for (std::size_t i = 3; i < 6; ++i) {
            const char name[] = {'p', static_cast<char>('0' + i), '\0'};
            std::memcpy(s.position.presets[i].name, name, sizeof name);
            s.position.presets[i].value = 10.0 * static_cast<double>(i);
        }
        s.position.presetCount = 6;
        return s;
    });
    add("SIX speed presets (the most)", [] {
        auto s = flywheel();
        for (std::size_t i = 0; i < 6; ++i) {
            const char name[] = {'s', static_cast<char>('0' + i), '\0'};
            std::memcpy(s.speed.presets[i].name, name, sizeof name);
            s.speed.presets[i].value = 100.0 * static_cast<double>(i);
        }
        s.speed.presetCount = 6;
        return s;
    });
    add("a linkage of THREE stages (the most)", [] { auto s = dr4b(); s.position.armStages = 3; return s; });
    add("a table of exactly 2 points (the fewest)", [] { auto s = tilt(); s.position.tablePoints = 2; return s; });
    add("a speed preset exactly at the free speed, reversed", [] { auto s = flywheel(); s.speed.presets[0].value = -3600.0; return s; });
    add("an at-speed lasting of 0", [] { auto s = flywheel(); s.speed.atSpeedTime = 0.0; return s; });
    add("a settle time of 0", [] { auto s = arm(); s.position.settleTime = 0.0; return s; });
    add("a home position exactly at the top limit", [] { auto s = arm(); s.position.homePosition = 160.0; return s; });
    add("a homing push of exactly -12 V", [] { auto s = lift(); s.position.homeVolts = -12.0; return s; });
    add("a top-homed push of exactly +12 V", [] { auto s = swing(); s.position.homePosition = s.position.maxPosition; s.position.homeVolts = 12.0; return s; });
    add("a gravity hold of exactly 12 V", [] { auto s = arm(); s.position.kG = 12.0; return s; });
    add("a gravity hold of exactly -12 V (elastics pulling harder than gravity)", [] { auto s = lift(); s.position.kG = -12.0; return s; });
    add("unhomedVolts of exactly 12", [] { auto s = tilt(); s.position.unhomedVolts = 12.0; return s; });
    add("unhomedVolts of 0 (nothing moves it before homing)", [] { auto s = tilt(); s.position.unhomedVolts = 0.0; return s; });
    // MECH-A3: absolute homing over just under one output turn, and on a linkage (its formula spans at
    // most 180 deg) — accepted.
    add("absolute homing over 359.9 deg of travel", [] {
        auto s = wrist();
        s.position.minPosition = -179.95;
        s.position.maxPosition = 179.95;
        return s;
    });
    add("absolute homing on a measured table spanning 359.9 deg", [] {
        auto s = tilt();
        s.position.table[2][1] = 359.9;
        s.position.home = HomeMethod::Absolute;
        s.position.homePosition = 0.0;
        s.angleSensor.kind = SensorKind::Rotation;
        return s;
    });
    add("absolute homing on a linkage (never past 180 deg of its formula)", [] {
        auto s = dr4b();
        s.position.home = HomeMethod::Absolute;
        s.position.homePosition = 0.0;
        s.angleSensor.kind = SensorKind::Rotation;
        return s;
    });
    return c;
}

}  // namespace

TEST_CASE("MECH-A1 spec: every base declaration is valid (the cases below start from a valid spec)") {
    for (const auto& [what, s] : std::vector<std::pair<const char*, MechanismSpec>>{
             {"robot x arm", arm()}, {"robot x lift", lift()}, {"robot x flywheel", flywheel()},
             {"robot x intake", intake()}, {"robot x hood", hood()}, {"robot y wrist", wrist()},
             {"robot y swing", swing()}, {"robot y dr4b", dr4b()}, {"robot y tilt", tilt()},
             {"robot y roller", roller()}, {"robot two claw", claw()}}) {
        CAPTURE(what);
        CHECK(specRefusal(s) == Refusal::None);
        CHECK(specProblem(s) == nullptr);
    }
}

TEST_CASE("MECH-A1 spec: ONE property wrong gives exactly THAT refusal and its sentence -- and every refusal is reachable") {
    std::set<int> hit;
    for (const Case& k : oneProperty()) {
        CAPTURE(k.what);
        const MechanismSpec s = k.make();
        const Refusal got = specRefusal(s);
        CHECK(static_cast<int>(got) == static_cast<int>(k.expected));
        REQUIRE(specProblem(s) != nullptr);
        CHECK(std::string{specProblem(s)} == std::string{refusalText(k.expected)});
        hit.insert(static_cast<int>(got));
    }
    // EVERY reason the list holds was produced by a case: a reason appended to spec.hpp with no case
    // here turns this red (the list's last enumerator is the bound).
    const int last = static_cast<int>(Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    for (int r = 1; r <= last; ++r) {
        CAPTURE(r);
        CHECK(hit.count(r) == 1U);
    }
    CHECK(hit.count(0) == 0U);
}

TEST_CASE("MECH-A1 spec: the exact boundaries every numeric rule allows are ACCEPTED") {
    for (const Case& k : boundariesAccepted()) {
        CAPTURE(k.what);
        const MechanismSpec s = k.make();
        CHECK(specRefusal(s) == Refusal::None);
    }
}

TEST_CASE("MECH-A1 spec: the CHECK ORDER, peeled -- each fix reveals the NEXT reason (M7's net)") {
    SUBCASE("a position mechanism wrong in nine ways") {
        MechanismSpec s = arm();
        s.cartridge = static_cast<shulib::hal::MotorCartridge>(0);
        s.safeMode = static_cast<shulib::config::SafeBrakeMode>(0);
        s.angleSensor.kind = SensorKind::Distance;
        s.position.stall.amps = 2.5;
        s.position.gearRatio = 0.0;
        s.position.gravity = static_cast<Gravity>(0);
        s.position.presets[2].value = 170.0;
        s.presetNameTooLong = true;
        std::strcpy(s.position.presets[1].name, "stow");
        const MechanismSpec good = arm();
        CHECK(specRefusal(s) == Refusal::AMotorMechanismSCartridgeIsUNSET);
        s.cartridge = good.cartridge;
        CHECK(specRefusal(s) == Refusal::AMotorMechanismSSafeBrakeModeIsUNSET);
        s.safeMode = good.safeMode;
        CHECK(specRefusal(s) == Refusal::TheAngleSensorRoleIsNotARotationSensor);
        s.angleSensor = good.angleSensor;
        CHECK(specRefusal(s) == Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
        s.position.stall = good.position.stall;
        CHECK(specRefusal(s) == Refusal::PositionGearRatioMustBe0);
        s.position.gearRatio = good.position.gearRatio;
        CHECK(specRefusal(s) == Refusal::PositionGravityUNSET);
        s.position.gravity = good.position.gravity;
        CHECK(specRefusal(s) == Refusal::PositionAPresetIsOutsideTheTravelLimits);
        s.position.presets[2].value = good.position.presets[2].value;
        CHECK(specRefusal(s) == Refusal::APresetOrPositionNameDoesNotFit15Characters);
        s.presetNameTooLong = false;
        CHECK(specRefusal(s) == Refusal::TwoPresetsOrPositionsShareAName);
        s.position.presets[1] = good.position.presets[1];
        CHECK(specRefusal(s) == Refusal::None);
    }
    SUBCASE("a lift wrong in seven ways") {
        MechanismSpec s = lift();
        s.homeSwitch.kind = SensorKind::Rotation;
        s.position.homeStall = {.amps = 0.0, .speedRadS = 1.0, .time = 0.2};
        s.position.geometry = static_cast<Geometry>(0);
        s.position.minPosition = 30.0;
        s.position.tolerance = 0.0;
        s.position.homeVolts = 3.0;
        s.position.stall = {};
        const MechanismSpec good = lift();
        CHECK(specRefusal(s) == Refusal::TheHomeSwitchRoleIsNotALimitSwitch);
        s.homeSwitch = good.homeSwitch;
        CHECK(specRefusal(s) == Refusal::AStallSignatureNeedsAmpsAbove0);
        s.position.homeStall = good.position.homeStall;
        CHECK(specRefusal(s) == Refusal::PositionGeometryUNSET);
        s.position.geometry = good.position.geometry;
        CHECK(specRefusal(s) == Refusal::PositionTravelLimitsMissingOrMinMax);
        s.position.minPosition = good.position.minPosition;
        CHECK(specRefusal(s) == Refusal::PositionToleranceMustBe0);
        s.position.tolerance = good.position.tolerance;
        CHECK(specRefusal(s) == Refusal::PositionHomingPushesAwayFromItsHomeEnd);
        s.position.homeVolts = good.position.homeVolts;
        CHECK(specRefusal(s) == Refusal::PositionTheStallSignatureIsUNSET);
        s.position.stall = good.position.stall;
        CHECK(specRefusal(s) == Refusal::None);
    }
    SUBCASE("an arm wrong in the review's new ways -- each split field AFTER its old partner, each voltage in its place") {
        MechanismSpec s = arm();
        s.position.tolerance = 0.0;
        s.position.settleTime = -1.0;
        s.position.kG = 13.0;
        s.position.homePosition = 170.0;
        s.position.unhomedVolts = 13.0;
        s.position.presets[0].value = -5.0;
        const MechanismSpec good = arm();
        CHECK(specRefusal(s) == Refusal::PositionToleranceMustBe0);
        s.position.tolerance = good.position.tolerance;
        CHECK(specRefusal(s) == Refusal::PositionTheSettleTimeIsNegative);
        s.position.settleTime = good.position.settleTime;
        CHECK(specRefusal(s) == Refusal::PositionTheGravityHoldIsPast12Volts);
        s.position.kG = good.position.kG;
        CHECK(specRefusal(s) == Refusal::PositionTheHomePositionIsOutsideTheTravelLimits);
        s.position.homePosition = good.position.homePosition;
        CHECK(specRefusal(s) == Refusal::PositionUnhomedVoltsIsNotIn0To12);
        s.position.unhomedVolts = good.position.unhomedVolts;
        CHECK(specRefusal(s) == Refusal::PositionAPresetIsOutsideTheTravelLimits);
        s.position.presets[0] = good.position.presets[0];
        CHECK(specRefusal(s) == Refusal::None);
    }
    SUBCASE("a stall-homed arm pushing 13 V the wrong way with no signature -- the push's size before its signature and direction") {
        MechanismSpec s = swing();
        s.position.homeVolts = 13.0;
        s.position.homeStall = {};
        const MechanismSpec good = swing();
        CHECK(specRefusal(s) == Refusal::PositionTheHomingPushIsPast12Volts);
        s.position.homeVolts = 3.0;
        CHECK(specRefusal(s) == Refusal::PositionStallHomingNeedsItsStallSignature);
        s.position.homeStall = good.position.homeStall;
        CHECK(specRefusal(s) == Refusal::PositionHomingPushesAwayFromItsHomeEnd);
        s.position.homeVolts = good.position.homeVolts;
        CHECK(specRefusal(s) == Refusal::None);
    }
    SUBCASE("MECH-S-A (CA3): the cap reasons come LAST -- an old reason wins over them, the range (71, 73) before the signature (72)") {
        MechanismSpec s = lift();
        s.position.tolerance = 0.0;     // an old reason
        s.currentCapAmps = 2.6;         // the cap's range
        s.position.homeStall = {.amps = 0.0, .speedRadS = 0.0, .time = 0.0};
        const MechanismSpec good = lift();
        CHECK(specRefusal(s) == Refusal::PositionToleranceMustBe0);
        s.position.tolerance = good.position.tolerance;
        CHECK(specRefusal(s) == Refusal::ACurrentCapMustBeAbove0AndAtMost25A);
        s.currentCapAmps = 1.5;         // in range now, but below the lift's 2.0 A signature
        CHECK(specRefusal(s) == Refusal::AStallSignatureMustBeBelowTheCurrentCap);
        s.position.stall.amps = 1.49;
        CHECK(specRefusal(s) == Refusal::None);
        // under one milliamp (73) is the CAP's fault, named before the signature's (72), though the lift's
        // 1.49 A signature is far above it too
        s.currentCapAmps = 0.0005;
        CHECK(specRefusal(s) == Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
        s.currentCapAmps = 1.5;
        CHECK(specRefusal(s) == Refusal::None);
        // and the OLD 2.5 A rule still fires first for a signature at 2.5 under a cap of 2.5
        s.position.stall.amps = 2.5;
        s.currentCapAmps = 2.5;
        CHECK(specRefusal(s) == Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    }
    SUBCASE("a flywheel and a pneumatic: the split fields after their partners") {
        MechanismSpec f = flywheel();
        f.speed.atSpeedBandRpm = 0.0;
        f.speed.atSpeedTime = -1.0;
        CHECK(specRefusal(f) == Refusal::SpeedTheAtSpeedBandMustBe0);
        f.speed.atSpeedBandRpm = flywheel().speed.atSpeedBandRpm;
        CHECK(specRefusal(f) == Refusal::SpeedTheAtSpeedLastingIsNegative);
        MechanismSpec h = hood();
        h.pneumatic.actuationTime = 0.0;
        h.pneumatic.confirmWindow = -1.0;
        CHECK(specRefusal(h) == Refusal::PneumaticActuationTimeMustBe0);
        h.pneumatic.actuationTime = hood().pneumatic.actuationTime;
        CHECK(specRefusal(h) == Refusal::PneumaticTheConfirmWindowIsNegative);
    }
    SUBCASE("a pneumatic wrong in eight ways") {
        MechanismSpec s = hood();
        s.motorCount = 1;
        s.pieceSensor.kind = SensorKind::Rotation;
        s.pneumatic.actuationTime = 0.0;
        s.pneumatic.positions[0].name[0] = '\0';
        s.pneumatic.positions[2].levels[0] = false;  // "up" becomes {Low, High}: still unique
        s.pneumatic.positions[1].levels[0] = false;  // "middle" becomes {Low, Low}: "down"'s
        s.presetNameTooLong = true;
        s.pneumatic.startNamed = false;
        const MechanismSpec good = hood();
        CHECK(specRefusal(s) == Refusal::APneumaticMechanismHasMotorRoles);
        s.motorCount = 0;
        CHECK(specRefusal(s) == Refusal::APieceSensorCannotBeARotationSensor);
        s.pieceSensor = good.pieceSensor;
        CHECK(specRefusal(s) == Refusal::PneumaticActuationTimeMustBe0);
        s.pneumatic.actuationTime = good.pneumatic.actuationTime;
        CHECK(specRefusal(s) == Refusal::PneumaticAPositionHasNoName);
        s.pneumatic.positions[0] = good.pneumatic.positions[0];
        CHECK(specRefusal(s) == Refusal::PneumaticTwoPositionsHaveTheSameLineLevels);
        s.pneumatic.positions[1] = good.pneumatic.positions[1];
        CHECK(specRefusal(s) == Refusal::APresetOrPositionNameDoesNotFit15Characters);
        s.presetNameTooLong = false;
        CHECK(specRefusal(s) == Refusal::PneumaticItsStartNamesNoneOfItsPositions);
        s.pneumatic.startNamed = true;
        CHECK(specRefusal(s) == Refusal::None);  // "up" at {Low, High} is still a distinct position
    }
}

TEST_CASE("MECH-A1 spec: every refusal has its own sentence; None has none; a byte past the list is 'unknown'") {
    std::set<std::string> sentences;
    const int last = static_cast<int>(Refusal::ACurrentCapMustBeAtLeastOneMilliamp);
    for (int r = 1; r <= last; ++r) {
        CAPTURE(r);
        const char* text = refusalText(static_cast<Refusal>(r));
        REQUIRE(text != nullptr);
        CHECK(std::string{text}.size() > 10U);
        CHECK(sentences.insert(text).second);  // unique: G3a mirrors these by sentence
    }
    CHECK(refusalText(Refusal::None) == nullptr);
    CHECK(std::string{refusalText(static_cast<Refusal>(last + 1))} == "an unknown refusal");
    // Pinned verbatim — the never-trip pair's sentences are the guide's (N10) and G3a's words.
    CHECK(std::string{refusalText(Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit)}
          == "a stall signature's amps must be below the motor's 2.5 A current limit (a motor limited at "
             "2.5 A never reads more: the signature would never trip)");
    CHECK(std::string{refusalText(Refusal::AStallSignatureNeedsBelowMotorRpmAbove0)}
          == "a stall signature needs belowMotorRpm above 0 (a stalled motor still reads a little creep: at 0 "
             "it would never trip)");
    // MECH-S-A's two, verbatim (the guide's words; G3a Part 5 mirrors them by sentence).
    CHECK(std::string{refusalText(Refusal::ACurrentCapMustBeAbove0AndAtMost25A)}
          == "a current cap must be above 0 and at most 2.5 A (the motor's own limit is 2.5 A; leave .currentCap "
             "out for no cap)");
    CHECK(std::string{refusalText(Refusal::AStallSignatureMustBeBelowTheCurrentCap)}
          == "a stall signature's amps must be below the current cap (a capped motor never draws more than its "
             "cap: the signature would never trip)");
    CHECK(std::string{refusalText(Refusal::ACurrentCapMustBeAtLeastOneMilliamp)}
          == "a current cap must be at least 0.001 A (a V5 motor holds its limit in whole milliamps: a smaller "
             "cap would be sent as 0 mA, a motor with no torque at all)");
}

TEST_CASE("MECH-A1 spec: refusal 1..N keep their numbers (append-only: G3a mirrors the list)") {
    // A reason inserted in the middle would renumber everything after it; these pin both ends and the
    // re-check's appended block.
    CHECK(static_cast<int>(Refusal::AMechanismHasNoName) == 1);
    CHECK(static_cast<int>(Refusal::AbsoluteHomingTheSensorSReadingAtZeroMustBeIn0360) == 55);
    CHECK(static_cast<int>(Refusal::PositionTheStallSignatureIsUNSET) == 56);
    CHECK(static_cast<int>(Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit) == 59);
    CHECK(static_cast<int>(Refusal::AStallSignatureNeedsBelowMotorRpmAbove0) == 60);
    CHECK(static_cast<int>(Refusal::AStallSignatureNeedsLastingAbove0) == 61);
    CHECK(static_cast<int>(Refusal::AStallSignatureNeedsAmpsAbove0) == 62);
    CHECK(static_cast<int>(Refusal::PositionGeometryUNSET) == 63);
    // MECH-A1's review appended six, after every earlier number.
    CHECK(static_cast<int>(Refusal::PositionTheSettleTimeIsNegative) == 64);
    CHECK(static_cast<int>(Refusal::SpeedTheAtSpeedLastingIsNegative) == 65);
    CHECK(static_cast<int>(Refusal::PneumaticTheConfirmWindowIsNegative) == 66);
    CHECK(static_cast<int>(Refusal::PositionTheHomingPushIsPast12Volts) == 67);
    CHECK(static_cast<int>(Refusal::PositionTheGravityHoldIsPast12Volts) == 68);
    CHECK(static_cast<int>(Refusal::PositionUnhomedVoltsIsNotIn0To12) == 69);
    // MECH-A3 appended one, after every earlier number.
    CHECK(static_cast<int>(Refusal::PositionAbsoluteHomingTravelSpansAWholeSensorTurn) == 70);
    // MECH-S-A appended two, after every earlier number (the brief's CA3, MA7).
    CHECK(static_cast<int>(Refusal::ACurrentCapMustBeAbove0AndAtMost25A) == 71);
    CHECK(static_cast<int>(Refusal::AStallSignatureMustBeBelowTheCurrentCap) == 72);
    // MECH-S-A's review appended one, after every earlier number (a cap under one milliamp).
    CHECK(static_cast<int>(Refusal::ACurrentCapMustBeAtLeastOneMilliamp) == 73);
}

TEST_CASE("MECH-A1 spec: nameOf reads up to the NUL, bounded by the field") {
    MechanismSpec s{};
    CHECK(shulib::mechanism::nameOf(s).empty());
    setName(s, "claw pivot 6");
    CHECK(shulib::mechanism::nameOf(s) == "claw pivot 6");
    std::memset(s.name, 'y', sizeof s.name);  // no NUL: bounded by the 24 bytes, never past them
    CHECK(shulib::mechanism::nameOf(s).size() == shulib::mechanism::kNameBytes);
    CHECK(std::string{shulib::mechanism::kindWord(Kind::Position)} == "position");
    CHECK(std::string{shulib::mechanism::kindWord(static_cast<Kind>(0))} == "UNSET");
}
