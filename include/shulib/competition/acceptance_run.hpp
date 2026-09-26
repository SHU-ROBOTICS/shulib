#pragma once
//
// competition::acceptanceRun — the LIBRARY's acceptance run: the one routine shulib ships
// (chunk COMP-a, 2026-09-26; the COMP brief's ruling C3 and amendment B18).
//
// ── What it is, and what it is not ──────────────────────────────────────────────────
// It is NOT season content. It is the test a team runs to see the competition program, the
// selector, the guard and the motion stack work end to end on THEIR robot before trusting any
// auton of their own: drive 24 in forward, turn 90 degrees left, then drive back to the start —
// on a tank, "back to the start" means FACING it first (a tank cannot drive sideways, and the
// library never plans a turn for you — chassis.hpp's D12), so the run is four motions:
//
//     moveTo (24, 0) facing 0  ->  turnTo 90  ->  face (0, 0)  ->  driveTo (0, 0)
//
// It is written in the START FRAME: register it with StartPose::relative(), and wherever the robot
// is placed becomes (0, 0) facing 0. The EXPECTED END is the start position, facing the opposite
// way it started (180 degrees in the start frame): on the tape, back on the mark it was placed on,
// turned round. Every leg takes the robot's configured motion defaults (no number here is invented
// for it). The season's autons are the students' (the guardrail): the students' file registers
// this entry beside their own, and otherwise ships empty apart from the registration pattern.
//
// PROS-free, allocation-free. Host-tested on the plant (test/competition_runner_test.cpp); NOTHING
// HERE HAS RUN ON A ROBOT.

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::competition {

/// The acceptance run's name on the selector and in the log (screen-safe, within the budget).
inline constexpr const char* kAcceptanceRunName = "TEST 24in turn back";
static_assert(ui::screenText(kAcceptanceRunName, kMaxAutonNameChars));

/// The acceptance run (file banner): four motions in the START frame, ending on the start
/// position facing 180 degrees. Returns the Routine's verdict.
[[nodiscard]] inline chassis::RoutineResult acceptanceRun(AutonContext& ctx) {
    chassis::Routine r{ctx.chassis, "acceptance-run"};
    r.moveTo(math::Pose2d{units::Length{24.0}, units::Length{0.0}, math::Angle{}})
        .turnTo(math::Angle::degrees(90.0))
        .face(units::Length{0.0}, units::Length{0.0})
        .driveTo(units::Length{0.0}, units::Length{0.0});
    return r.result();
}

/// The registry row for the acceptance run: its name, `kind`, the routine, a RELATIVE start and
/// the schedule the CALLER states (never defaulted — the COMP brief's C6). The end action is the
/// default (brake). Register it like any other entry.
[[nodiscard]] inline AutonEntry acceptanceRunEntry(AutonKind kind, units::Time endActionAt,
                                                   units::Time hardStopAt) {
    AutonEntry e{};
    e.name = kAcceptanceRunName;
    e.kind = kind;
    e.routine = &acceptanceRun;
    e.start = StartPose::relative();
    e.endActionAt = endActionAt;
    e.hardStopAt = hardStopAt;
    return e;
}

}  // namespace shulib::competition
