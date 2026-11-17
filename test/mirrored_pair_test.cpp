// THE PAIR AT ONCE — MIRROR1-A's T11 (the MIRROR1 brief's §2.2, D11; A4 register HA-260), over the check in
// test/mirrored_pair_check.hpp. Everything here is the host test plant; nothing here has run on a robot.
//
// The check returns a VERDICT and these cases assert it: a PLANTED crash is a PASSING test of the check
// (it must FAIL that route, naming the moment and the motions); only a registered team route that crashes
// turns the suite red (test/autons_test.cpp). Every route here is written in ROBOT ONE's numbers, from a
// FIELD start in the red wall's Quadrant, and registered MIRRORED — robot two is the reflection.
//
// Each case attacks ONE rule and names the bug it would catch.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "mirrored_pair_check.hpp"
#include "route_preview_model.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::isMirrored;
using shulib::competition::mirrored;
using shulib::competition::StartPose;
using shulib::competition::override2026::kSkillsMirror;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

namespace {

const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{Length{12.0}};
    return k;
}

/// A mirrored SKILLS entry of `routine` from FIELD `start` (60 s: park at 55, stop at 58).
AutonEntry mirroredEntry(const char* name, shulib::competition::AutonRoutine routine, const Pose2d& start) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Skills;
    e.routine = routine;
    e.start = StartPose::field(start);
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    return mirrored(e, kSkillsMirror);
}

mirror_pair::PairOptions options(double weak = 1.0) {
    mirror_pair::PairOptions o;
    o.kin = &tank();
    o.weakScale = weak;
    return o;
}

/// Robot one's least clearance from the mirror line y = -x over a track: its square's nearest corner,
/// on its own side (x + y < 0) positive.
double leastClearance(const mirror_pair::Track& t) {
    double least = 1e300;
    for (const mirror_pair::Pace& p : t.paces) {
        mirror_pair::V2 c[4];
        mirror_pair::square(p.truth, mirror_pair::kRobotSquareIn, c);
        for (const mirror_pair::V2& k : c) {
            least = std::min(least, -(k.x + k.y) / std::sqrt(2.0));
        }
    }
    return least;
}

// ── the routes (robot one's numbers) ────────────────────────────────────────────────────────────────

/// The brief's probe P1 route — its FIRST evidence — which crosses the diagonal (landmine 13).
RoutineResult crossesTheDiagonal(AutonContext& ctx) {
    Routine r{ctx, "P1 route"};
    r.driveTo(-12_in, -58_in);
    r.face(-12_in, -34_in);
    r.driveTo(-12_in, -34_in);
    r.turnTo(45_deg);
    r.moveTo(Pose2d{0_in, -22_in, 45_deg});
    r.faceAway(-24_in, -22_in);
    r.reverseTo(-24_in, -22_in);
    r.curveTo({{-6_in, -12_in}, {12_in, -18_in, -30_deg}});
    r.reverseCurveTo({{-6_in, -30_in}, {-24_in, -40_in, 0_deg}});
    r.followTrajectory({Pose2d{0_in, -40_in, 0_deg}, Pose2d{12_in, -40_in, 0_deg}});
    r.face(30_in, -10_in);
    r.hold(300_ms);
    r.pause(200_ms);
    r.driveTo(30_in, -10_in);
    return r.result();
}

/// The brief's probe P5 route G: the same fourteen step kinds, every point kept on robot one's side.
RoutineResult ownSide(AutonContext& ctx) {
    Routine r{ctx, "route G"};
    r.driveTo(-12_in, -58_in);
    r.face(-12_in, -40_in);
    r.driveTo(-12_in, -40_in);
    r.turnTo(45_deg);
    r.moveTo(Pose2d{-2_in, -30_in, 45_deg});
    r.faceAway(-24_in, -36_in);
    r.reverseTo(-24_in, -36_in);
    r.curveTo({{-6_in, -26_in}, {12_in, -32_in, -30_deg}});
    r.reverseCurveTo({{-6_in, -44_in}, {-24_in, -54_in, 0_deg}});
    r.followTrajectory({Pose2d{-10_in, -54_in, 0_deg}, Pose2d{2_in, -54_in, 0_deg}});
    r.face(10_in, -44_in);
    r.hold(300_ms);
    r.pause(200_ms);
    r.driveTo(10_in, -44_in);
    return r.result();
}

/// ROBOT ONE ONLY to the centre: robot one drives in beside the centre goal, holds, backs out; robot two
/// waits on its own side for as long (the guide's isMirrored recipe).
RoutineResult centreForRobotOne(AutonContext& ctx) {
    Routine r{ctx, "centre, robot one"};
    r.driveTo(-12_in, -58_in);
    r.face(-12_in, -30_in);
    if (!isMirrored(ctx)) {
        r.driveTo(-12_in, -30_in);
        r.face(-6_in, -6_in);
        r.driveTo(-6_in, -6_in);
        r.hold(1500_ms);
        r.reverseTo(-12_in, -30_in);
    } else {
        r.pause(6_s);  // robot two waits where it is
    }
    r.face(-30_in, -40_in);
    r.driveTo(-30_in, -40_in);
    return r.result();
}

/// The same branch TIMED WRONG: robot two waits only 1 s, then drives to ITS side of the centre while
/// robot one still holds there.
RoutineResult centreBothTimedWrong(AutonContext& ctx) {
    Routine r{ctx, "centre, timed wrong"};
    r.driveTo(-12_in, -58_in);
    r.face(-12_in, -30_in);
    if (!isMirrored(ctx)) {
        r.driveTo(-12_in, -30_in);
        r.face(-6_in, -6_in);
        r.driveTo(-6_in, -6_in);
        r.hold(3_s);
        r.reverseTo(-12_in, -30_in);
    } else {
        r.pause(1_s);
        r.driveTo(-12_in, -30_in);
        r.face(-6_in, -6_in);
        r.driveTo(-6_in, -6_in);
        r.reverseTo(-12_in, -30_in);
    }
    return r.result();
}

/// Robot one parked, still: a half-second pause and nothing else (the gap cases).
RoutineResult standStill(AutonContext& ctx) {
    Routine r{ctx, "still"};
    r.pause(500_ms);
    return r.result();
}

/// A FIELD start square-on to the mirror line (heading 45 deg: an edge parallel to y = -x) with its
/// square `clearance` in off the line: the pure mirror's gap is twice that.
Pose2d squareOn(double clearance) {
    const double d = 7.5 + clearance;  // the centre's distance from the line
    return Pose2d{Length{-d / std::sqrt(2.0)}, Length{-d / std::sqrt(2.0)}, 45_deg};
}

}  // namespace

// Bug caught: a pair check that cannot fail (MM15: robot two compared with the reflection of robot one —
// the same place — or with anything but robot one AT THE SAME MOMENT); the brief's own first evidence route
// shipped as a recipe. P1's route crosses the diagonal: the check FAILS it, naming the moment, the
// motion each robot was in and both positions — and the robots' squares overlap later in the curve
// (the brief's P5: touching from 10.1 s of the probe's bare run).
// One robot size for the pair check and the route preview (MIRROR1-A merge item 23): both robots are 15 in.
static_assert(mirror_pair::kRobotSquareIn == route_preview::kPreviewFootprintIn,
              "the mirrored pair check and the route preview must use the same robot square");

TEST_CASE("MIRROR1 T11 (i) the brief's P1 route crashes the pair: FAILED, naming the moment, the motions, both places") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const AutonEntry e = mirroredEntry("P1 route", &crossesTheDiagonal, Pose2d{-36_in, -58_in, 0_deg});
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(e, options(), &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK_FALSE(v.ok);
    CHECK(v.failGapIn < mirror_pair::kMirrorGapIn);
    CHECK(v.failAt > 9.95);  // MEASURED 10.02 s, in the curve (MIRROR1-PROGRESS.md)
    CHECK(v.failAt < 10.10);
    // robot two IS robot one's reflection at that moment (a pure mirror): (x, y) -> (-y, -x)
    CHECK(std::fabs(v.two.x().value() + v.one.y().value()) < 1e-6);
    CHECK(std::fabs(v.two.y().value() + v.one.x().value()) < 1e-6);
    CHECK(v.stepOne.find("CurveTo") != std::string::npos);
    CHECK(v.stepTwo.find("CurveTo") != std::string::npos);
    CHECK(v.leastGapIn == 0.0);  // they touch
    // The whole sentence, as guide chapter 17 quotes it (a review finding, 2026-11-13: the quote was
    // unpinned): robot one's motion named WITH its target in the route's own numbers — the curve's end
    // (12, -18), the line a student finds — and "motion 8" is the 8th DRIVE motion (here also step 8).
    CHECK(v.sentence
          == "auton 'P1 route': the two robots come within 6.0 in (5.76 in apart) at 10.02 s -- robot one at (-5.9, "
             "-12.7) in motion 8 (CurveTo ending at (12.0, -18.0)), robot two at (12.7, 5.9) in motion 8 (CurveTo). "
             "Keep robot one's whole robot 3.0 in off the mirror line, or send ONE robot there with isMirrored(ctx).");
    // both runs ran the whole route: the crash is the route's, not a broken run
    CHECK(t.first.result.routine.ok);
    CHECK(t.second.result.routine.ok);
    CHECK(t.second.result.mirrored);
}

// Bug caught: a check so strict a route kept on its own side fails, or one that measures something else
// than the two squares at the same moment. In a PURE mirror the least gap IS twice robot one's least
// clearance from the line (the brief's P5: 8.43 = 2 x 4.21 in): landed on to 1e-6 in.
TEST_CASE("MIRROR1 T11 (ii) route G, kept on robot one's side: PASSES, the least gap exactly twice the clearance") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const AutonEntry e = mirroredEntry("route G", &ownSide, Pose2d{-36_in, -58_in, 0_deg});
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(e, options(), &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK(v.ok);
    CHECK(v.leastGapIn >= mirror_pair::kMirrorGapIn);
    const double clearance = leastClearance(t.first);
    MESSAGE("least gap " << v.leastGapIn << " in, robot one's least clearance " << clearance << " in");
    CHECK(std::fabs(v.leastGapIn - 2.0 * clearance) <= 1e-6);
    CHECK(t.first.result.routine.ok);
    CHECK(t.second.result.routine.ok);
    CHECK(t.first.paces.size() == t.second.paces.size());
}

// Bug caught: a check that only holds for a perfect robot. The ROBOT is not mirrored: a 9 % weak right
// side on BOTH plants makes robot two's timing and path differ from the reflection (HA-260) — route G
// still keeps them apart (the brief's P5 F: 8.75 in).
TEST_CASE("MIRROR1 T11 (iii) route G with a 9 % weak side on both robots: PASSES") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const AutonEntry e = mirroredEntry("route G weak", &ownSide, Pose2d{-36_in, -58_in, 0_deg});
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(e, options(0.91), &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK(v.ok);
    CHECK(v.leastGapIn >= mirror_pair::kMirrorGapIn);
    CHECK(t.first.paces.size() != t.second.paces.size());  // the weak side moved robot two's timing
}

// Bug caught: MM16 (isMirrored always false — robot two drives into the centre with robot one); a check
// that cannot see a one-robot branch timed wrong. Robot one alone to the centre while robot two waits:
// PASSES. The same branch with robot two coming in while robot one still holds there: FAILS.
TEST_CASE("MIRROR1 T11 (iv) isMirrored(ctx): one robot to the centre PASSES; the branch timed wrong FAILS") {
    {
        const AutonEntry e = mirroredEntry("centre one", &centreForRobotOne, Pose2d{-36_in, -58_in, 0_deg});
        std::pair<mirror_pair::Track, mirror_pair::Track> t;
        const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(e, options(), &t);
        MESSAGE(v.sentence);
        CHECK(v.ok);
        CHECK(t.first.result.routine.ok);
        CHECK(t.second.result.routine.ok);
        CHECK(t.first.result.routine.steps != t.second.result.routine.steps);  // the branch really differed
    }
    {
        const AutonEntry e = mirroredEntry("centre wrong", &centreBothTimedWrong, Pose2d{-36_in, -58_in, 0_deg});
        const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(e, options());
        MESSAGE(v.sentence);
        CHECK(v.checked);
        CHECK_FALSE(v.ok);
        // robot one at the centre spot (arriving or holding) when robot two comes in
        CHECK(std::hypot(v.one.x().value() + 6.0, v.one.y().value() + 6.0) < 1.5);
        CHECK(v.stepTwo.find("MoveToPose") != std::string::npos);
        CHECK(v.stepOne == "motion 5 (MoveToPose to (-6.0, -6.0))");  // the branch's driveTo(-6, -6)
    }
}

// Bug caught (a review finding, 2026-11-13): a failure naming only "motion N (Kind)" — which counts DRIVE
// motions, not the Routine's steps — so a student cannot find the line. Robot one's step name carries the
// motion's TARGET from its own live record, in the route's numbers, by kind: a turn's heading, a curve's
// END, a drive's or a strafe's point; a hold (or a brake) has none; robot two's (and every name without
// the target) is the bare form. Hand-built tracks, so every form is landed on exactly.
TEST_CASE("MIRROR1 T11 the step names: a turn's heading, a curve's end, a drive's point; a hold none") {
    mirror_pair::Track t;
    t.names = {{1, "TurnTo"}, {2, "CurveTo"}, {3, "MoveToPose"}, {4, "HoldPose"}, {5, "StrafeTo"}, {6, "ReverseCurveTo"}};
    const auto pace = [&t](std::uint32_t id, std::uint32_t last, const Pose2d& target, bool has) {
        mirror_pair::Pace p;
        p.motion = id;
        p.lastMotion = last;
        p.target = target;
        p.hasTarget = has;
        t.paces.push_back(p);
    };
    pace(0, 0, Pose2d{}, false);
    pace(1, 0, Pose2d{-36_in, -58_in, 90_deg}, true);
    pace(2, 1, Pose2d{12_in, -18_in, -30_deg}, true);
    pace(3, 2, Pose2d{-6_in, -6_in, 45_deg}, true);
    pace(4, 3, Pose2d{-6_in, -6_in, 45_deg}, true);
    pace(0, 4, Pose2d{}, false);
    pace(5, 4, Pose2d{2_in, -50_in, 0_deg}, true);
    pace(6, 5, Pose2d{-24_in, -54_in, 0_deg}, true);
    pace(3, 6, Pose2d{}, false);  // a motion whose live record has not come yet: no target
    CHECK(t.stepAt(0, true) == "before its first motion");
    CHECK(t.stepAt(1, true) == "motion 1 (TurnTo to 90.0 deg)");
    CHECK(t.stepAt(2, true) == "motion 2 (CurveTo ending at (12.0, -18.0))");
    CHECK(t.stepAt(3, true) == "motion 3 (MoveToPose to (-6.0, -6.0))");
    CHECK(t.stepAt(4, true) == "motion 4 (HoldPose)");
    CHECK(t.stepAt(5, true) == "between motions, after motion 4 (HoldPose)");
    CHECK(t.stepAt(6, true) == "motion 5 (StrafeTo to (2.0, -50.0))");
    CHECK(t.stepAt(7, true) == "motion 6 (ReverseCurveTo ending at (-24.0, -54.0))");
    CHECK(t.stepAt(8, true) == "motion 3 (MoveToPose)");
    CHECK(t.stepAt(2) == "motion 2 (CurveTo)");  // without the target: the bare form (robot two's)
    CHECK(t.stepAt(9, true) == "its run over (stopped)");
}

// Bug caught: a RELATIVE mirrored entry "checked" against field positions it does not have, or skipped
// silently. It is SKIPPED, by name; an unmirrored entry has no pair.
TEST_CASE("MIRROR1 T11 (v) a RELATIVE mirrored entry is skipped, by name") {
    AutonEntry e{};
    e.name = "relative";
    e.kind = AutonKind::Skills;
    e.routine = &standStill;
    e.start = StartPose::relative();
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    const mirror_pair::PairVerdict none = mirror_pair::checkMirroredPair(e, options());
    CHECK_FALSE(none.checked);
    CHECK(none.sentence == "auton 'relative': not a mirrored entry -- no pair to check");
    const mirror_pair::PairVerdict rel = mirror_pair::checkMirroredPair(mirrored(e, kSkillsMirror), options());
    CHECK_FALSE(rel.checked);
    CHECK(rel.sentence
          == "auton 'relative': a RELATIVE mirrored entry -- its field positions are wherever the drivers place the "
             "robots, so the pair check is SKIPPED");
}

// Bug caught: the gap constant drifting from the guide's 3 in a side, or the threshold off by a hair. The
// constant is 6 in; robot one square-on to the line with its square 2.95 in off it (a 5.9 in gap) FAILS,
// 3.05 in (6.1) PASSES — real runs through runAuton; and the EQUALITY edge lands exactly: two hand-built
// tracks exactly 6.0 in apart PASS ("closer than 6 in" fails), the next double below FAILS.
TEST_CASE("MIRROR1 T11 (vi) the gap: 6 in; 5.9 in apart FAILS, 6.1 in PASSES; exactly 6.0 passes, below it fails") {
    CHECK(mirror_pair::kMirrorGapIn == 6.0);
    CHECK(mirror_pair::kRobotSquareIn == 15.0);
    {
        const mirror_pair::PairVerdict v =
            mirror_pair::checkMirroredPair(mirroredEntry("gap 5.9", &standStill, squareOn(2.95)), options());
        CHECK_FALSE(v.ok);
        CHECK(v.leastGapIn == doctest::Approx(5.9).epsilon(1e-9));
    }
    {
        const mirror_pair::PairVerdict v =
            mirror_pair::checkMirroredPair(mirroredEntry("gap 6.1", &standStill, squareOn(3.05)), options());
        CHECK(v.ok);
        CHECK(v.leastGapIn == doctest::Approx(6.1).epsilon(1e-9));
    }
    const auto still = [](double x) {
        mirror_pair::Track t;
        mirror_pair::Pace p;
        p.truth = Pose2d{Length{x}, 0_in, 0_deg};
        t.paces.assign(3, p);
        return t;
    };
    const mirror_pair::Track a = still(0.0);
    const mirror_pair::PairVerdict at = mirror_pair::comparePair("edge", a, still(21.0));
    CHECK(at.leastGapIn == 6.0);  // exactly: 21 - 7.5 - 7.5
    CHECK(at.ok);
    const mirror_pair::PairVerdict below = mirror_pair::comparePair("edge", a, still(std::nextafter(21.0, 0.0)));
    CHECK(below.leastGapIn < 6.0);
    CHECK_FALSE(below.ok);
}

// Bug caught (T9's half): the registry walk skipping a mirrored entry, or reporting an unmirrored one.
// A registry of three — an unmirrored entry, route G mirrored, P1's route mirrored — yields exactly two
// verdicts, in order: route G passes, P1's route fails.
TEST_CASE("MIRROR1 T11 checkEveryMirroredEntry: one verdict per MIRRORED entry, in order") {
    shulib::competition::AutonRegistry reg{
        shulib::competition::AutonPeriods{.match = shulib::units::Time{30.0}, .skills = shulib::units::Time{60.0}}};
    AutonEntry plain{};
    plain.name = "plain";
    plain.kind = AutonKind::Skills;
    plain.routine = &ownSide;
    plain.start = StartPose::field(Pose2d{-36_in, -58_in, 0_deg});
    plain.endActionAt = 55_s;
    plain.hardStopAt = 58_s;
    reg.add(plain);
    reg.add(mirroredEntry("route G mirrored", &ownSide, Pose2d{-36_in, -58_in, 0_deg}));
    reg.add(mirroredEntry("P1 mirrored", &crossesTheDiagonal, Pose2d{-36_in, -58_in, 0_deg}));
    const std::vector<mirror_pair::PairVerdict> vs = mirror_pair::checkEveryMirroredEntry(reg, tank());
    REQUIRE(vs.size() == 2U);
    CHECK(vs[0].ok);
    CHECK(vs[0].sentence.rfind("auton 'route G mirrored':", 0) == 0);
    CHECK_FALSE(vs[1].ok);
    CHECK(vs[1].sentence.rfind("auton 'P1 mirrored':", 0) == 0);
}
