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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "mirrored_pair_check.hpp"
#include "motion_test_rig.hpp"
#include "route_preview_model.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
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

// ═══ MIRROR1-B: a TANK's half-turn leg is REFUSED (the team lead's D6 ruling, 2026-11-15: "Block it") ═══
// One step that drives AND turns about 180 deg: on a tank robot one ends it beside its target and robot
// two, sweeping the same way, beside its own on the other side (MIRROR1-A's T3). The check fails it,
// naming the step, with the fix: turn first, then drive. Every route here is far from the diagonal, so
// the gap alone would pass them.

namespace {

const shulib::kinematics::MatrixKinematics& xdrive() {
    static const shulib::kinematics::MatrixKinematics k = shulib::kinematics::xDrive(Length{7.0});
    return k;
}
const shulib::kinematics::MatrixKinematics& hdrive() {  // strafe authority 0.35: between a tank and an X-drive
    static const shulib::kinematics::MatrixKinematics k = motion_rig::hBotKinematics();
    return k;
}

/// A mirrored SKILLS entry of `routine` from a RELATIVE start (T11 (v)'s form).
AutonEntry relativeEntry(const char* name, shulib::competition::AutonRoutine routine) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Skills;
    e.routine = routine;
    e.start = StartPose::relative();
    e.endActionAt = 55_s;
    e.hardStopAt = 58_s;
    return mirrored(e, kSkillsMirror);
}

/// A track's [MIR] lines; a line that starts so; robot two's end from robot one's mirror (inches).
int mirLines(const mirror_pair::Track& t) {
    int n = 0;
    for (const std::string& l : t.lines) {
        n += l.rfind("[MIR] ", 0) == 0 ? 1 : 0;
    }
    return n;
}
bool hasLine(const mirror_pair::Track& t, const char* start) {
    return std::any_of(t.lines.begin(), t.lines.end(), [start](const std::string& l) { return l.rfind(start, 0) == 0; });
}
long linesStarting(const mirror_pair::Track& t, const char* start) {
    return std::count_if(t.lines.begin(), t.lines.end(), [start](const std::string& l) { return l.rfind(start, 0) == 0; });
}
double endOffMirror(const std::pair<mirror_pair::Track, mirror_pair::Track>& t) {
    const Pose2d w = kSkillsMirror.reflection.apply(t.first.paces.back().truth);
    const Pose2d e = t.second.paces.back().truth;
    return std::hypot((w.x() - e.x()).value(), (w.y() - e.y()).value());
}

/// MIRROR1-A T3's 36 in leg: 36 in ahead, ending facing back (180 deg from the start).
RoutineResult halfTurnLeg36(AutonContext& ctx) {
    Routine r{ctx, "leg 36"};
    r.moveTo(Pose2d{0_in, -58_in, 180_deg}, {.timeout = 6_s});
    return r.result();
}

/// The fix the check gives for it: turn first, then back up.
RoutineResult halfTurnLeg36Fixed(AutonContext& ctx) {
    Routine r{ctx, "leg 36, fixed"};
    r.faceAway(0_in, -58_in);
    r.reverseTo(0_in, -58_in, {.timeout = 6_s});
    return r.result();
}

/// The same leg, the ruling's words taken literally: turnTo(180), then driveTo the point — now BEHIND.
RoutineResult halfTurnLeg36TurnThenDrive(AutonContext& ctx) {
    Routine r{ctx, "turn, drive"};
    r.turnTo(180_deg);
    r.driveTo(0_in, -58_in, {.timeout = 6_s});
    return r.result();
}

/// A driveTo 36 in straight BEHIND (its heading is the bearing: 180 deg off).
RoutineResult backUp36(AutonContext& ctx) {
    Routine r{ctx, "back 36"};
    r.driveTo(-36_in, -58_in, {.timeout = 6_s});
    return r.result();
}

/// The same, in two steps: turn first, then drive.
RoutineResult backUp36TwoSteps(AutonContext& ctx) {
    Routine r{ctx, "back 36, two steps"};
    r.face(-36_in, -58_in);
    r.driveTo(-36_in, -58_in, {.timeout = 6_s});
    return r.result();
}

/// A turn of exactly 180 deg (its [MIR] line is a TURN's: allowed), a drive, then a trajectory leg
/// back that ends facing 0 (a LEG's line): the third drive motion is the one refused.
RoutineResult turnDriveThenLegBack(AutonContext& ctx) {
    Routine r{ctx, "later leg"};
    r.turnTo(180_deg);
    r.driveTo(-50_in, -58_in);
    r.followTrajectory({Pose2d{-36_in, -58_in, 0_deg}});
    return r.result();
}

/// A 16 in moveTo ahead ending facing `deg` (MIRROR1-A T3's 16 in leg, its heading varied).
template <int kHundredthsDeg>
RoutineResult legFacing(AutonContext& ctx) {
    Routine r{ctx, "leg facing"};
    r.moveTo(Pose2d{-20_in, -58_in, Angle::degrees(kHundredthsDeg / 100.0)}, {.timeout = 4_s});
    return r.result();
}

/// A moveTo `kTenthsIn` / 10 in ahead of the start (-36, -58), ending facing back.
template <int kTenthsIn>
RoutineResult shortLeg(AutonContext& ctx) {
    Routine r{ctx, "short leg"};
    r.moveTo(Pose2d{Length{-36.0 + kTenthsIn / 10.0}, -58_in, 180_deg}, {.timeout = 4_s});
    return r.result();
}

/// Two half-turn legs, a turn between them: the FIRST is named.
RoutineResult twoLegs(AutonContext& ctx) {
    Routine r{ctx, "two legs"};
    using shulib::chassis::carryOn;
    r.onFail(carryOn).moveTo(Pose2d{-20_in, -58_in, 180_deg}, {.timeout = 4_s});
    r.onFail(carryOn).turnTo(0_deg);
    r.moveTo(Pose2d{-4_in, -58_in, 180_deg}, {.timeout = 4_s});
    return r.result();
}

/// A leg to a point ahead and to the LEFT, ending facing back: neither forwards nor backwards.
RoutineResult sideLeg(AutonContext& ctx) {
    Routine r{ctx, "side leg"};
    r.moveTo(Pose2d{-26_in, -40_in, 180_deg}, {.timeout = 6_s});
    return r.result();
}

/// The three steps the check gives for it.
RoutineResult sideLegFixed(AutonContext& ctx) {
    Routine r{ctx, "side leg, fixed"};
    r.faceAway(-26_in, -40_in);
    r.reverseTo(-26_in, -40_in, {.timeout = 6_s});
    r.turnTo(180_deg);
    return r.result();
}

/// RELATIVE: 36 in ahead, ending facing back — and its fix.
RoutineResult relativeLeg(AutonContext& ctx) {
    Routine r{ctx, "rel leg"};
    r.moveTo(Pose2d{36_in, 0_in, 180_deg}, {.timeout = 6_s});
    return r.result();
}
RoutineResult relativeLegFixed(AutonContext& ctx) {
    Routine r{ctx, "rel leg, fixed"};
    r.faceAway(36_in, 0_in);
    r.reverseTo(36_in, 0_in, {.timeout = 6_s});
    return r.result();
}

/// Into the centre beside robot two (a crash), then straight back the way it came (a half-turn leg).
RoutineResult centreThenBack(AutonContext& ctx) {
    Routine r{ctx, "both"};
    r.face(-4_in, -4_in);
    r.driveTo(-4_in, -4_in);
    r.driveTo(-30_in, -50_in);
    return r.result();
}

}  // namespace

// Bug caught: the ruling not enforced (the check passes a tank's half-turn leg), enforced but unnamed, or
// a fix the student cannot act on. MIRROR1-A T3's own 36 in leg (a point AHEAD, ending facing back —
// a backwards leg): the check FAILS it in one sentence naming the step in robot one's numbers, with
// the fix for THIS shape — turn first, then back up. The gap alone would pass it (the robots stay
// 61 in apart). Robot two ends 14.2 in off the mirror (T3's number: this is T3's leg), and robot one's
// own run fails too: the leg times out beside its target. The fix as given PASSES, robot two ending
// on the mirror; the ruling's words taken literally (turnTo(180), then driveTo the point, now BEHIND)
// are refused again — that is why the sentence names the steps.
TEST_CASE("MIRROR1-B T12 (i) a 36 in TANK leg that turns 180 deg while it drives: REFUSED, naming it; its fix passes") {
    const Pose2d start{-36_in, -58_in, 0_deg};
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict v =
        mirror_pair::checkMirroredPair(mirroredEntry("leg 36", &halfTurnLeg36, start), options(), &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK_FALSE(v.ok);
    CHECK(v.halfTurn.found);
    CHECK(v.halfTurn.ran);
    CHECK(v.halfTurn.step == "motion 1 (MoveToPose to (0.0, -58.0))");
    CHECK(v.halfTurn.turnDeg == doctest::Approx(180.0).epsilon(1e-9));
    CHECK(v.leastGapIn > 60.0);  // MEASURED 61.63: the gap passes; the leg alone fails it
    CHECK(v.sentence
          == "auton 'leg 36': motion 1 (MoveToPose to (0.0, -58.0)) at 0.01 s turns 180.0 deg while it drives, which "
             "a tank cannot mirror -- turn first, then back up: faceAway(0.0, -58.0), then reverseTo(0.0, -58.0).");
    CHECK(mirLines(t.first) == 0);
    CHECK(mirLines(t.second) == 1);
    const double off = endOffMirror(t);
    MESSAGE("robot two's end off the mirror: " << off << " in");
    CHECK(off > 14.15);  // MEASURED 14.1992 — T3's 36 in leg, 14.199
    CHECK(off < 14.25);
    CHECK_FALSE(t.first.result.routine.ok);  // robot one's leg times out beside its target too

    const mirror_pair::PairVerdict fixed =
        mirror_pair::checkMirroredPair(mirroredEntry("leg 36, fixed", &halfTurnLeg36Fixed, start), options(), &t);
    MESSAGE(fixed.sentence);
    CHECK(fixed.ok);
    CHECK_FALSE(fixed.halfTurn.found);
    CHECK(hasLine(t.second, "[MIR] mirrored turnTo turns 180.0 deg"));  // the faceAway's: a TURN's line
    CHECK(t.first.result.routine.ok);
    CHECK(t.second.result.routine.ok);
    MESSAGE("fixed: robot two's end off the mirror " << endOffMirror(t) << " in");
    CHECK(endOffMirror(t) < 0.1);  // MEASURED 0.0453 (the faceAway's own exact half turn, as in (ii))

    const mirror_pair::PairVerdict literal =
        mirror_pair::checkMirroredPair(mirroredEntry("turn, drive", &halfTurnLeg36TurnThenDrive, start), options());
    MESSAGE(literal.sentence);
    CHECK_FALSE(literal.ok);
    CHECK(literal.halfTurn.step == "motion 2 (MoveToPose to (0.0, -58.0))");
    CHECK(literal.halfTurn.fix == "turn first, then drive: face(0.0, -58.0), then driveTo(0.0, -58.0)");
}

// Bug caught: a turn's [MIR] line refused as if it were a leg, so the fix the sentence gives would fail
// too, or the wrong shape's fix. A driveTo 36 in straight BEHIND is the same leg, driving forwards:
// REFUSED, the fix face, then driveTo. Written so, it PASSES: the face is an exact half turn (robot two
// logs ITS [MIR] line, a TURN's), and robot two ends within 0.1 in of the mirror.
TEST_CASE("MIRROR1-B T12 (ii) a driveTo straight behind is REFUSED; turn first, then drive, PASSES") {
    const Pose2d start{0_in, -58_in, 0_deg};
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict one =
        mirror_pair::checkMirroredPair(mirroredEntry("back 36", &backUp36, start), options(), &t);
    MESSAGE(one.sentence);
    CHECK_FALSE(one.ok);
    CHECK(one.halfTurn.step == "motion 1 (MoveToPose to (-36.0, -58.0))");
    CHECK(one.halfTurn.fix == "turn first, then drive: face(-36.0, -58.0), then driveTo(-36.0, -58.0)");
    CHECK(endOffMirror(t) > 14.15);  // MEASURED 14.1992: the same leg as (i), number for number
    CHECK(endOffMirror(t) < 14.25);
    const mirror_pair::PairVerdict two =
        mirror_pair::checkMirroredPair(mirroredEntry("back 36, two steps", &backUp36TwoSteps, start), options(), &t);
    MESSAGE(two.sentence);
    CHECK(two.checked);
    CHECK(two.ok);
    CHECK_FALSE(two.halfTurn.found);
    CHECK(mirLines(t.second) == 1);
    CHECK(hasLine(t.second, "[MIR] mirrored turnTo turns 180.0 deg"));  // the face's: a TURN's line
    CHECK(t.first.result.routine.ok);
    CHECK(t.second.result.routine.ok);
    MESSAGE("two steps: robot two's end off the mirror " << endOffMirror(t) << " in");
    CHECK(endOffMirror(t) < 0.1);  // MEASURED 0.0453 (the turn sweeps the same way; the drive does not)
}

// Bug caught: the wrong step named — the turn's line taken for a leg (motion 1), the pace BEFORE the
// leg's line (motion 2), only the run's first [MIR] line looked at, or robot two's own numbers
// ((58.0, 36.0)) printed instead of the route's. A turn of exactly 180 deg (allowed), a drive, then a
// trajectory leg back facing 0: the third drive motion is the one named.
TEST_CASE("MIRROR1-B T12 (iii) a LATER leg is named, in robot one's numbers, after an allowed turn") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(
        mirroredEntry("later leg", &turnDriveThenLegBack, Pose2d{-36_in, -58_in, 0_deg}), options(), &t);
    MESSAGE(v.sentence);
    CHECK_FALSE(v.ok);
    REQUIRE(mirLines(t.second) == 2);  // the turn's line first, then the leg's
    CHECK(v.halfTurn.step == "motion 3 (MoveToPose to (-36.0, -58.0))");
    CHECK(v.halfTurn.at == doctest::Approx(2.80).epsilon(1e-3));  // MEASURED 2.80 s
    CHECK(v.sentence
          == "auton 'later leg': motion 3 (MoveToPose to (-36.0, -58.0)) at 2.80 s turns 180.0 deg while it drives, "
             "which a tank cannot mirror -- turn first, then drive: face(-36.0, -58.0), then driveTo(-36.0, -58.0).");
}

// Bug caught: the refusal applied to every drivetrain. An X-drive's half-turn moveTo is measured and
// pinned in T3 (0.31 in along the way): it logs its [MIR] line and PASSES.
TEST_CASE("MIRROR1-B T12 (iv) an X-drive's half-turn moveTo: its [MIR] line, and PASSES") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    mirror_pair::PairOptions o = options();
    o.kin = &xdrive();
    const mirror_pair::PairVerdict v =
        mirror_pair::checkMirroredPair(mirroredEntry("x leg", &legFacing<18000>, Pose2d{-36_in, -58_in, 0_deg}), o, &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK(v.ok);
    CHECK_FALSE(v.halfTurn.found);
    CHECK(mirLines(t.second) == 1);
    CHECK(hasLine(t.second, "[MIR] mirrored moveTo turns 180.0 deg"));
    CHECK(endOffMirror(t) < 0.05);  // MEASURED 0.0179
}

// Bug caught: the check judging the heading itself (its own window) instead of the [MIR] line, or
// refusing only the exact tie. It refuses what the line names: within 1 deg of a half turn. Each leg
// here is NOT the tie and mirrors exactly on the plant (MEASURED <= 1.2e-12 in); inside the window it is
// a coin toss on a robot (MIRROR1-A: robot one's end moves 14.5 in on a 36 in leg), so it is refused.
// The samples sit off the window's edge on both sides of 180, as T3's do: 179.05 and 180.95 (inside),
// 178.95 and 181.05 (outside). The edge itself is NOT claimed: its comparison sits at floating
// precision (MEASURED on this plant: 179.0 refused, 181.0 — the same distance the other way — passes).
TEST_CASE("MIRROR1-B T12 (v) near the half turn: 179.05, 179.5, 180.95 REFUSED; 178.9, 178.95, 181.05 PASS") {
    const Pose2d start{-36_in, -58_in, 0_deg};
    struct Leg {
        const char* name;
        shulib::competition::AutonRoutine routine;
        bool refused;
        double deg;  ///< the heading change the [MIR] line gives (to 0.1: MEASURED "179.1" for 179.05)
    };
    for (const Leg& leg :
         {Leg{"leg 179.5", &legFacing<17950>, true, 179.5}, Leg{"leg 179.05", &legFacing<17905>, true, 179.1},
          Leg{"leg 180.95", &legFacing<18095>, true, 179.1}, Leg{"leg 178.95", &legFacing<17895>, false, 0.0},
          Leg{"leg 181.05", &legFacing<18105>, false, 0.0}, Leg{"leg 178.9", &legFacing<17890>, false, 0.0}}) {
        CAPTURE(leg.name);
        std::pair<mirror_pair::Track, mirror_pair::Track> t;
        const mirror_pair::PairVerdict v =
            mirror_pair::checkMirroredPair(mirroredEntry(leg.name, leg.routine, start), options(), &t);
        MESSAGE(v.sentence);
        CHECK(v.ok == !leg.refused);
        CHECK(v.halfTurn.found == leg.refused);
        CHECK(mirLines(t.second) == (leg.refused ? 1 : 0));
        MESSAGE(leg.name << ": robot two's end off the mirror " << endOffMirror(t) << " in");
        CHECK(endOffMirror(t) <= 1e-9);  // no tie: mirrored exactly (MEASURED <= 1.2e-12 in)
        if (leg.refused) {
            CHECK(v.halfTurn.step == "motion 1 (MoveToPose to (-20.0, -58.0))");
            CHECK(v.halfTurn.turnDeg == doctest::Approx(leg.deg).epsilon(1e-9));
        }
    }
}

// Bug caught: the half-turn failure HIDING a crash. A route that drives into the centre beside robot
// two (the gap fails at 1.99 s) and then straight back (a half-turn leg at 3.11 s): both are named,
// the leg's sentence first.
TEST_CASE("MIRROR1-B T12 (vi) a crash AND a half-turn leg: both named") {
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(
        mirroredEntry("both", &centreThenBack, Pose2d{-30_in, -50_in, 0_deg}), options());
    MESSAGE(v.sentence);
    CHECK_FALSE(v.ok);
    CHECK(v.halfTurn.found);
    CHECK(v.failGapIn < mirror_pair::kMirrorGapIn);
    const std::string leg =
        "auton 'both': motion 3 (MoveToPose to (-30.0, -50.0)) at 3.11 s turns 180.0 deg while it drives, which a "
        "tank cannot mirror -- turn first, then drive: face(-30.0, -50.0), then driveTo(-30.0, -50.0).";
    CHECK(v.sentence.rfind(leg + " auton 'both': the two robots come within 6.0 in", 0) == 0);
    CHECK(v.sentence.find("in motion 2 (MoveToPose to (-4.0, -4.0))") != std::string::npos);
}

// Bug caught: a RELATIVE mirrored entry carrying a tank's half-turn leg past the ruling. Its GAP is
// skipped (no field positions), but the leg needs none: robot two runs alone and the leg is refused,
// in robot one's relative numbers, `checked` so the registry case (autons (1d)) fails it. Its fix
// passes (skipped as before); on an X-drive the entry is skipped whole.
TEST_CASE("MIRROR1-B T12 (vii) a RELATIVE mirrored entry: a tank's half-turn leg is REFUSED; its fix is skipped") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(relativeEntry("rel leg", &relativeLeg), options(), &t);
    MESSAGE(v.sentence);
    CHECK(v.checked);
    CHECK_FALSE(v.ok);
    CHECK((v.ok || !v.checked) == false);  // autons (1d)'s own assertion fails it
    CHECK(v.sentence
          == "auton 'rel leg': motion 1 (MoveToPose to (36.0, 0.0)) at 0.01 s turns 180.0 deg while it drives, which "
             "a tank cannot mirror -- turn first, then back up: faceAway(36.0, 0.0), then reverseTo(36.0, 0.0).");
    CHECK(t.first.paces.empty());  // robot two only
    CHECK(mirLines(t.second) == 1);
    const std::string skipped = "a RELATIVE mirrored entry -- its field positions are wherever the drivers place the "
                                "robots, so the pair check is SKIPPED";
    const mirror_pair::PairVerdict fixed =
        mirror_pair::checkMirroredPair(relativeEntry("rel leg, fixed", &relativeLegFixed), options(), &t);
    MESSAGE(fixed.sentence);
    CHECK_FALSE(fixed.checked);
    CHECK(fixed.ok);
    CHECK(fixed.sentence == "auton 'rel leg, fixed': " + skipped);
    CHECK(t.second.result.routine.ok);
    mirror_pair::PairOptions x = options();
    x.kin = &xdrive();
    const mirror_pair::PairVerdict xv = mirror_pair::checkMirroredPair(relativeEntry("rel leg", &relativeLeg), x);
    CHECK_FALSE(xv.checked);
    CHECK(xv.ok);
    CHECK(xv.sentence == "auton 'rel leg': " + skipped);
}

// Bug caught (the review's G1): the LAST leg named instead of the first, so the student fixes a later step
// first. Two half-turn legs (the first carries on after its timeout), a turn between them (it logs a
// TURN's line too: the first leg ended facing back): the first leg is named.
TEST_CASE("MIRROR1-B T12 (viii) two half-turn legs: the FIRST is named") {
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(
        mirroredEntry("two legs", &twoLegs, Pose2d{-36_in, -58_in, 0_deg}), options(), &t);
    MESSAGE(v.sentence);
    CHECK_FALSE(v.ok);
    REQUIRE(linesStarting(t.second, "[MIR] mirrored moveTo turns ") == 2);  // both legs logged their line
    CHECK(linesStarting(t.second, "[MIR] mirrored turnTo turns ") == 1);    // MEASURED: the turn between
    CHECK(v.halfTurn.step == "motion 1 (MoveToPose to (-20.0, -58.0))");
    CHECK(v.halfTurn.at == doctest::Approx(0.01).epsilon(1e-6));
}

// Bug caught: a turn written as a moveTo refused as if it drove ("while it drives" would be false and the
// fix meaningless), or the in-place rule swallowing a real leg. A moveTo to the robot's own spot,
// facing back, logs its [MIR] line but drives nowhere: like a turnTo, NOT refused (MEASURED: robot two
// ends exactly on the mirror). The line between is the library's arrival tolerance, 0.5 in: a 0.4 in
// leg passes, 0.6 in and 1 in are refused (MEASURED: robot two 0.14, 0.21, 0.36 in off the mirror).
TEST_CASE("MIRROR1-B T12 (ix) a 'leg' that only turns is not refused; 0.4 in passes, 0.6 in is refused") {
    const Pose2d start{-36_in, -58_in, 0_deg};
    CHECK(mirror_pair::kInPlaceIn == 0.5);
    struct Leg {
        const char* name;
        shulib::competition::AutonRoutine routine;
        bool refused;
    };
    for (const Leg& leg : {Leg{"in place", &shortLeg<0>, false}, Leg{"0.4 in", &shortLeg<4>, false},
                           Leg{"0.6 in", &shortLeg<6>, true}, Leg{"1 in", &shortLeg<10>, true}}) {
        CAPTURE(leg.name);
        std::pair<mirror_pair::Track, mirror_pair::Track> t;
        const mirror_pair::PairVerdict v =
            mirror_pair::checkMirroredPair(mirroredEntry(leg.name, leg.routine, start), options(), &t);
        MESSAGE(v.sentence);
        MESSAGE(leg.name << ": robot two's end off the mirror " << endOffMirror(t) << " in");
        CHECK(mirLines(t.second) == 1);  // every one logs its [MIR] moveTo line
        CHECK(hasLine(t.second, "[MIR] mirrored moveTo turns 180.0 deg"));
        CHECK(v.ok == !leg.refused);
        CHECK(v.halfTurn.found == leg.refused);
    }
}

// Bug caught: the tank test widened past a tank (G8: `< 1.0`), refusing a drivetrain that can strafe.
// The library's own test for a tank is a strafe authority of EXACTLY 0; an H-drive (0.35) is not a
// tank: its half-turn moveTo logs its [MIR] line and is NOT refused (MEASURED: robot two 0.027 in off the
// mirror).
TEST_CASE("MIRROR1-B T12 (x) an H-drive (strafe authority 0.35): its half-turn moveTo is not refused") {
    CHECK(hdrive().strafeAuthority() == doctest::Approx(0.35));
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    mirror_pair::PairOptions o = options();
    o.kin = &hdrive();
    const mirror_pair::PairVerdict v =
        mirror_pair::checkMirroredPair(mirroredEntry("h leg", &legFacing<18000>, Pose2d{-36_in, -58_in, 0_deg}), o, &t);
    MESSAGE(v.sentence);
    MESSAGE("H-drive: robot two's end off the mirror " << endOffMirror(t) << " in");
    CHECK(v.checked);
    CHECK(v.ok);
    CHECK_FALSE(v.halfTurn.found);
    CHECK(hasLine(t.second, "[MIR] mirrored moveTo turns 180.0 deg"));
    CHECK(endOffMirror(t) < 0.05);  // MEASURED 0.0271
}

// Bug caught: a fix that changes where the robot ends up facing. A leg to a point ahead and to the
// left that ends facing back drives neither forwards nor backwards along its end heading: the fix is
// turn first, back up, then turnTo the leg's own end heading — and those three steps PASS.
TEST_CASE("MIRROR1-B T12 (xi) a leg whose end heading is not its drive's: the fix ends with a turnTo, and passes") {
    const Pose2d start{-36_in, -58_in, 0_deg};
    std::pair<mirror_pair::Track, mirror_pair::Track> t;
    const mirror_pair::PairVerdict v =
        mirror_pair::checkMirroredPair(mirroredEntry("side leg", &sideLeg, start), options(), &t);
    MESSAGE(v.sentence);
    CHECK_FALSE(v.ok);
    CHECK(v.halfTurn.fix
          == "turn first, then back up: faceAway(-26.0, -40.0), then reverseTo(-26.0, -40.0), then turnTo(180.0 deg)");
    const mirror_pair::PairVerdict fixed =
        mirror_pair::checkMirroredPair(mirroredEntry("side leg, fixed", &sideLegFixed, start), options(), &t);
    MESSAGE(fixed.sentence);
    MESSAGE("fixed: robot two's end off the mirror " << endOffMirror(t) << " in");
    CHECK(fixed.ok);
    CHECK(mirLines(t.second) == 0);
    CHECK(t.first.result.routine.ok);
    CHECK(t.second.result.routine.ok);
    CHECK(endOffMirror(t) <= 1e-9);  // MEASURED 3.0e-13: no half turn left anywhere
}

// Bug caught (the review's G2, G4, G5): a defensive branch of the search going wrong unseen — they do not
// happen in a real run today, so the track is PLANTED. (a) A leg line after the run's last pace is
// still refused, never quietly, and claims no time. (b) A pace between motions after the line is not
// the leg. (c) The leg's first pace without a target: the target from its next pace. (d) A turnTo's
// line is never a leg.
TEST_CASE("MIRROR1-B T12 (xii) planted tracks: a line after the last pace, a pace between, no target, a turn") {
    const std::string mir =
        "[MIR] mirrored moveTo turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the same way as robot "
        "one, not the mirror way -- turn first, in two 90 deg turns, where it matters";
    const shulib::math::Reflection identity{};
    mirror_pair::Pace before;
    before.t = 0.01;
    before.lines = 0;  // the line is logged after this pace
    mirror_pair::Track a;
    a.lines = {mir};
    a.paces = {before};
    const mirror_pair::HalfTurnLeg la = mirror_pair::findHalfTurnLeg(a, identity);
    CHECK(la.found);
    CHECK_FALSE(la.ran);
    CHECK(mirror_pair::halfTurnSentence("late", la)
          == "auton 'late': a leg logged after its run's last pace turns 180.0 deg while it drives, which a tank "
             "cannot mirror -- turn first, then drive.");

    mirror_pair::Pace between;  // after the line, no motion yet
    between.t = 0.02;
    between.lines = 1;
    between.truth = Pose2d{0_in, 0_in, 0_deg};
    mirror_pair::Pace leg = between;  // the leg: 20 in behind, facing it
    leg.t = 0.03;
    leg.motion = 5;
    leg.hasTarget = true;
    leg.target = Pose2d{-20_in, 0_in, 180_deg};
    mirror_pair::Track b;
    b.lines = {mir};
    b.names = {{5, "MoveToPose"}};
    b.paces = {before, between, leg};
    const mirror_pair::HalfTurnLeg lb = mirror_pair::findHalfTurnLeg(b, identity);
    CHECK(lb.found);
    CHECK(lb.ran);
    CHECK(lb.step == "motion 5 (MoveToPose to (-20.0, 0.0))");
    CHECK(lb.at == doctest::Approx(0.03));
    CHECK(lb.fix == "turn first, then drive: face(-20.0, 0.0), then driveTo(-20.0, 0.0)");

    mirror_pair::Pace noTarget = leg;  // the leg's first pace carries no target yet
    noTarget.hasTarget = false;
    noTarget.target = Pose2d{};
    mirror_pair::Track c;
    c.lines = {mir};
    c.names = {{5, "MoveToPose"}};
    c.start = Pose2d{0_in, 0_in, 0_deg};
    noTarget.t = 0.01;
    noTarget.lines = 1;
    leg.t = 0.02;
    c.paces = {noTarget, leg};
    const mirror_pair::HalfTurnLeg lc = mirror_pair::findHalfTurnLeg(c, identity);
    CHECK(lc.found);
    CHECK(lc.step == "motion 5 (MoveToPose to (-20.0, 0.0))");
    CHECK(lc.at == doctest::Approx(0.01));
    CHECK(lc.fix == "turn first, then drive: face(-20.0, 0.0), then driveTo(-20.0, 0.0)");

    // (d) A turnTo's line is a turn by its VERB, whatever its record says. In a real run a TurnTo's
    // target is the robot's own spot (turn_to.hpp: "turn in place"), so the in-place rule would skip it
    // too — the review round's MB4 survived on that; here the target is planted 20 in away.
    mirror_pair::Track d = c;
    d.lines = {"[MIR] mirrored turnTo turns 180.0 deg: within 1 deg of a half turn, robot two may sweep the same "
               "way as robot one, not the mirror way -- write two 90 deg turns where it matters"};
    d.names = {{5, "TurnTo"}};
    CHECK_FALSE(mirror_pair::findHalfTurnLeg(d, identity).found);
}
