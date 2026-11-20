#pragma once
//
// route_preview_fixtures.hpp — the autons the route preview's tests run (chunk RPV1-A, 2026-11-12).
// FIXTURES, never the team's: the team's autons are src/team/autons.cpp, and the preview runs those
// (tools/route_preview.py). Each one here reproduces a case the RPV1 brief probed (its P- and C-
// probes) or a rule a test tries to break; the comment on each says which. Say "host test plant"
// (the project's one name for it): nothing here has run on a robot.
//
// The claw routes name THE CLAW ROBOT — the old robot two, retired at ROB1-BT (2026-11-16), kept as
// test/mechanism_fixtures.hpp's `mech_fixture::robot_two`; robot x's through `mech_fixture::robot_x` —
// RPV1-B1's fixture robot with a lift.
// A pneumatic position is a compile-time lookup (`claw["open"]` — the brief's landmine 12), so every
// position below is a literal.

#include <stdexcept>

#include "mechanism_fixtures.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/auton_wall_reset.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

namespace rp_fixture {

using namespace shulib::units::literals;
namespace comp = shulib::competition;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

/// One entry: `park` is endActionAt, `stop` hardStopAt (seconds).
[[nodiscard]] inline comp::AutonEntry entry(const char* name, comp::AutonKind kind, comp::AutonRoutine fn,
                                            comp::StartPose start, double park, double stop) {
    comp::AutonEntry e{};
    e.name = name;
    e.kind = kind;
    e.routine = fn;
    e.start = start;
    e.endActionAt = Time{park};
    e.hardStopAt = Time{stop};
    return e;
}

/// A FIELD start (inches, degrees).
[[nodiscard]] inline comp::StartPose field(double x, double y, double deg) {
    return comp::StartPose::field(Length{x}, Length{y}, Angle::degrees(deg));
}

// ── P2's shape: a long SKILLS auton (forty rounds of drive, pause, claw, pivot) ─────────────────────
/// From (-48, -48) facing 0: forward 40 in, back again, a 0.5 s pause, the claw, pivot 6 for 0.4 s —
/// forty times (160 steps). Longer than 60 s: the guard cuts it at its park time.
inline RoutineResult skillsLong(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "skills-long"};
    for (int i = 0; i < 40; ++i) {
        if (i % 2 == 0) {
            r.moveTo(Pose2d{-8_in, -48_in, 0_deg});
        } else {
            r.reverseTo(-48_in, -48_in);
        }
        r.pause(0.5_s);
        if (i % 2 == 0) {
            r.set(claw["open"]);
        } else {
            r.set(claw["closed"]);
        }
        r.run(clawPivot6, 12_volt, 0.4_s);
    }
    return r.result();
}

// ── RP35: every one of robot two's mechanisms, then a drive (the rig's default must bind them all) ──
/// The claw open, pivot 6 and pivot 7 for 0.3 s each, the claw closed, a 20 in drive — RELATIVE.
inline RoutineResult everyMechanism(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "every"};
    r.set(claw["open"]);
    r.run(clawPivot6, 12_volt, 0.3_s);
    r.run(clawPivot7, 12_volt, 0.3_s);
    r.set(claw["closed"]);
    r.moveTo(Pose2d{20_in, 0_in, 0_deg});
    return r.result();
}

// ── P8's chain: seven steps (and a grab), two of which (the pause, the then()) leave no trace from below
/// moveTo, pause, claw, pivot, turnTo, then(action), moveTo, claw closed — from a FIELD start at
/// (-48, -48, 0).
inline RoutineResult sevenSteps(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "seven"};
    r.moveTo(Pose2d{-24_in, -48_in, 0_deg});
    r.pause(0.3_s);
    r.set(claw["open"]);
    r.run(clawPivot6, 6_volt, 0.3_s);
    r.turnTo(90_deg);
    r.then([] { return true; }, "an action");
    r.moveTo(Pose2d{-24_in, -24_in, 90_deg});
    r.set(claw["closed"]);  // a GRAB with no sensor: UNCONFIRMED, counted done
    return r.result();
}

// ── RP6: step 3 times out; 4 and 5 are skipped ─────────────────────────────────────────────────────
/// moveTo, turnTo, then a moveTo with a 0.3 s time limit it cannot meet (step 3), then two more.
inline RoutineResult stepThreeTimesOut(comp::AutonContext& ctx) {
    Routine r{ctx, "times-out"};
    r.moveTo(Pose2d{-24_in, -48_in, 0_deg});
    r.turnTo(90_deg);
    r.moveTo(Pose2d{-24_in, -10_in, 90_deg}, {.timeout = 0.3_s});
    r.moveTo(Pose2d{-24_in, 0_in, 90_deg});
    r.turnTo(0_deg);
    return r.result();
}

// ── C1: a loop after the cut — every step skipped, one log line each, no pace ──────────────────────
/// `for (;;) { moveTo(A); moveTo(B); }` on a 57/59 s skills schedule.
inline RoutineResult loopsForever(comp::AutonContext& ctx) {
    Routine r{ctx, "forever"};
    for (;;) {
        r.moveTo(Pose2d{-8_in, -48_in, 0_deg});
        r.reverseTo(-48_in, -48_in);
    }
}

// ── C2 / RP28: a mechanism started in the background, its verdict arriving during the drive ───────
/// startRun(pivot 6, 12 V, 1 s), a 40 in drive, waitUntilDone(pivot 6) — RELATIVE.
inline RoutineResult backgroundPivot(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "background"};
    r.startRun(clawPivot6, 12_volt, 1_s);
    r.moveTo(Pose2d{40_in, 0_in, 0_deg});
    r.waitUntilDone(clawPivot6, 2_s);
    return r.result();
}

// ── P7a: an auton that throws after its first step ──────────────────────────────────────────────────
/// One 20 in drive, then a std::runtime_error with its own words.
inline RoutineResult throwsAfterOne(comp::AutonContext& ctx) {
    Routine r{ctx, "thrower"};
    r.moveTo(Pose2d{20_in, 0_in, 0_deg});
    throw std::runtime_error("a student's helper threw: no such stack");
}

// ── P6: a 200 in drive from the field's centre — the plant has no walls ─────────────────────────────
/// RELATIVE: 200 in straight ahead.
inline RoutineResult twoHundredInches(comp::AutonContext& ctx) {
    Routine r{ctx, "far"};
    r.moveTo(Pose2d{200_in, 0_in, 0_deg});
    return r.result();
}

// ── P9: a wall reset on the tester rig (no sensors) — REFUSED, counted done ─────────────────────────
/// FIELD start at (-48, -48, 0): a 10 in drive, the everyday wall reset (left wall, left sensor), a
/// 10 in drive.
inline RoutineResult wallResetRefused(comp::AutonContext& ctx) {
    namespace walls = shulib::localization::override2026;
    Routine r{ctx, "wall"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.then(
        [&ctx] {
            return comp::resetFromWall(ctx, {.wall = walls::kLeftWall,
                                             .sensor = shulib::localization::WallSide::Left,
                                             .maxCorrection = 3_in})
                .continues();
        },
        "left wall x");
    r.moveTo(Pose2d{-28_in, -48_in, 0_deg});
    return r.result();
}

// ── RP31: a helper with its own Routine, which stops at ITS step 2 ──────────────────────────────────
/// The helper "score": a 10 in drive (to x = -28), then a step on robot x's intake — not on robot two, so it
/// stops there ("is not on this robot").
inline RoutineResult scoreHelper(comp::AutonContext& ctx) {
    Routine r{ctx, "score"};
    r.moveTo(Pose2d{-28_in, -48_in, 0_deg});
    r.run(mech_fixture::robot_x::intake, 12_volt, 0.3_s);
    return r.result();
}
/// The main routine: a drive, the helper (its result folded in by a then()), a turn.
inline RoutineResult mainWithHelper(comp::AutonContext& ctx) {
    Routine r{ctx, "main"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.then([&ctx] { return scoreHelper(ctx).ok; }, "score");
    r.turnTo(90_deg);
    return r.result();
}

// ── RP28's failing form (B1): robot x's intake run in the background until it sees a piece ──────────
/// startRun(intake, 12 V, until seesPiece, 0.6 s) — the host test plant has no pieces, so it ends
/// TIMED_OUT while the 40 in drive is open; the drive itself settles.
inline RoutineResult backgroundIntakeTimesOut(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_x;
    Routine r{ctx, "intake-bg"};
    r.startRun(intake, 12_volt, seesPiece, 0.6_s);
    r.moveTo(Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}

// ── RP24 (B1): robot x's lift — a placeholder-numbers mechanism ─────────────────────────────────────
/// The lift to "high" and back to "floor", a drive between.
inline RoutineResult liftUpAndDown(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_x;
    Routine r{ctx, "lift"};
    r.move(lift["high"], 6_s);
    r.moveTo(Pose2d{20_in, 0_in, 0_deg});
    r.move(lift["floor"], 6_s);
    return r.result();
}

// ── The review's cases (RPV1-A's review, 2026-11-13) ───────────────────────────────────────────────
/// A drive straight into a claw step, no pause: the claw begins on the pace the drive ended — the EQUAL
/// edge of "the robot drove while it ran" (a claw after a drive is never "in the background").
inline RoutineResult driveThenClaw(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "drive-claw"};
    r.moveTo(Pose2d{-28_in, -48_in, 0_deg});
    r.set(claw["open"]);
    return r.result();
}
/// A curve (CURVE1-A) through two points, then a drive — from (-48, -48, 0).
inline RoutineResult curveThenDrive(comp::AutonContext& ctx) {
    Routine r{ctx, "curve"};
    r.curveTo({{-24_in, -36_in}, {0_in, -24_in, 0_deg}});
    r.moveTo(Pose2d{20_in, -24_in, 0_deg});
    return r.result();
}
/// A step that never ENDS and logs nothing: only a bound from outside ends it (the launcher's). (Until API 2.11 a loop
/// of steps that succeed was this fixture; since RMP1-K D4 each logs its "done in" line, so the no-pace stop ends it.)
inline RoutineResult loopsSilently(comp::AutonContext& ctx) {
    Routine r{ctx, "silent"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.then([] {
        for (volatile bool spin = true; spin;) {
        }
    }, "nothing");
    return r.result();
}
/// A drive the deadline CUTS (a 1 s park), then the auton's own code throws once it sees the cut: the
/// throw comes AFTER step 1 ended, not inside it.
inline RoutineResult throwsAfterTheCut(comp::AutonContext& ctx) {
    Routine r{ctx, "throw-after-cut"};
    r.moveTo(Pose2d{60_in, -48_in, 0_deg});
    if (!r.result().ok) {
        throw std::runtime_error("the auton threw after the cut");
    }
    return r.result();
}
/// A 40 in drive (RELATIVE) — the case that throws INSIDE it is the test's (a degradation that throws).
inline RoutineResult fortyInches(comp::AutonContext& ctx) {
    Routine r{ctx, "forty"};
    r.moveTo(Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}
/// A drive and the claw open; the entry's own END ACTION closes the claw and brakes (parkClosingClaw).
inline RoutineResult driveThenOpen(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "drive-open"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.set(claw["open"]);
    return r.result();
}
/// An end action of the entry's own that commands a mechanism: every step of it is the park's.
inline bool parkClosingClaw(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "park"};
    r.set(claw["closed"]);
    r.brake();
    return r.result().ok;
}

// ── The verification round's cases (RPV1-A, 2026-11-13): WHERE a run threw, beyond a drive ──────────
// The throw itself is the test's (a pacer layer that throws once the robot has stopped driving).
/// A drive, then a BLOCKING 3 s run of claw pivot 6, then a drive — from (-48, -48, 0): a throw while
/// pivot 6 runs is INSIDE step 2.
inline RoutineResult driveThenBlockingPivot(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "drive-pivot"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.run(clawPivot6, 6_volt, 3_s);
    r.moveTo(Pose2d{-28_in, -48_in, 0_deg});
    return r.result();
}
/// A drive, the claw opened (a blocking step that SUCCEEDS), a 1 s pause, a drive: a throw in the pause
/// is DURING A WAIT, after step 2 — and the claw, which ended before it, was not where it threw.
inline RoutineResult driveClawPause(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "drive-claw-pause"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.set(claw["open"]);
    r.pause(1_s);
    r.moveTo(Pose2d{-28_in, -48_in, 0_deg});
    return r.result();
}
/// Pivot 7 started in the background (3 s), a drive while it runs, a 1 s pause: a throw in the pause ends
/// pivot 7 by the unwind, but the robot drove while it ran — the code had moved on: DURING A WAIT, after
/// step 2, never inside pivot 7.
inline RoutineResult backgroundPivotDrivePause(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "bg-drive-pause"};
    r.startRun(clawPivot7, 6_volt, 3_s);
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.pause(1_s);
    return r.result();
}
/// A drive, pivot 7 started (3 s), a 0.2 s pause, then a BLOCKING 2 s run of pivot 6 with no drive
/// between: pivot 6 runs while pivot 7 does, but the robot did not drive during it — it is NOT a
/// background step, and a throw while it runs is INSIDE step 3. (The pause puts a pace between the two
/// begins: two mechanism steps begun on one pace are ordered by their list, not by the code.)
inline RoutineResult backgroundPivotThenBlockingPivot(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "bg-then-blocking"};
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.startRun(clawPivot7, 6_volt, 3_s);
    r.pause(0.2_s);
    r.run(clawPivot6, 6_volt, 2_s);
    return r.result();
}
/// Pivot 7 started, then pivot 6, with nothing paced between (two start forms on ONE pace), a drive during
/// both, then both collected: from below the two began at the same instant — numbered in robot two's list
/// order (pivot 6 first), not the code's.
inline RoutineResult twoStartsOnePace(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_two;
    Routine r{ctx, "two-starts"};
    r.startRun(clawPivot7, 6_volt, 0.5_s);
    r.startRun(clawPivot6, 6_volt, 0.5_s);
    r.moveTo(Pose2d{-38_in, -48_in, 0_deg});
    r.waitUntilDone(clawPivot7, 1_s);
    r.waitUntilDone(clawPivot6, 1_s);
    return r.result();
}
/// Robot y (test/mechanism_fixtures.hpp): its STALL-homed arm homed and moved, its MEASURED-shape lift
/// (declared neverHomed: a move needs a zero, so it is NOT_HOMED at once — the route stops there).
inline RoutineResult robotYSwingAndTilt(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_y;
    Routine r{ctx, "y"};
    r.home(swing, 4_s);
    r.move(swing["up"], 4_s);
    r.move(tilt["mid"], 6_s);
    return r.result();
}
/// Robot y's linkage lift (restsAtStop: homed at boot) to "low".
inline RoutineResult robotYDr4b(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_y;
    Routine r{ctx, "dr4b"};
    r.move(dr4b["low"], 6_s);
    return r.result();
}
/// Robot y's three piece sensors on fakes by kind, none of which ever sees a piece (the host test plant
/// has none): the roller (an optical sensor) and the catapult (a limit switch) run until they see one, in
/// the background, and time out; the grip (a distance sensor, confirmWithin) shuts on nothing — UNCONFIRMED.
inline RoutineResult robotYPieceSensors(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_y;
    Routine r{ctx, "pieces"};
    r.startRun(roller, 12_volt, seesPiece, 0.3_s);
    r.startRun(catapult, 6_volt, seesPiece, 0.3_s);
    r.pause(0.5_s);
    r.set(grip["shut"]);
    return r.result();
}
/// Robot x's FLYWHEEL to "shoot" (3000 rpm) — a Speed mechanism on fakes by kind.
inline RoutineResult spinUpFlywheel(comp::AutonContext& ctx) {
    using namespace mech_fixture::robot_x;
    Routine r{ctx, "fly"};
    r.spinUp(flywheel["shoot"], 4_s);
    return r.result();
}

// ── RP29: touching is not a failure; going through is ──────────────────────────────────────────────
/// (a) The robot placed with its back on the red wall, turned 0.1 degrees: a 10 in drive away from it.
inline RoutineResult awayFromRedWall(comp::AutonContext& ctx) {
    Routine r{ctx, "away"};
    r.moveTo(Pose2d{-40_in, -52.7_in, 90.1_deg});
    return r.result();
}
/// (b) Backing up to the LEFT Loader on the red wall (its face at y = -66.4647): the robot's back edge
/// stops `overlap` inches into it. (c) is the same with a larger overlap.
template <int TenthsIn>
inline RoutineResult backToLoader(comp::AutonContext& ctx) {
    Routine r{ctx, "loader"};
    const double faceY = -70.2047 + 3.74;
    r.reverseTo(-58.77_in, Length{faceY + 7.5 - TenthsIn / 10.0});
    return r.result();
}
/// (d) Along the red wall 0.1 in inside its face, clear of the Loader, the start pieces and the
/// Toggle: from x = -48 to -37 at heading 0.
inline RoutineResult hugsRedWall(comp::AutonContext& ctx) {
    Routine r{ctx, "hug"};
    r.moveTo(Pose2d{-37_in, Length{-70.2047 + 7.5 + 0.1}, 0_deg});
    return r.result();
}

}  // namespace rp_fixture
