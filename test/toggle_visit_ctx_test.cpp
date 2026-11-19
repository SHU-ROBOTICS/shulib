// The Toggle step's EVERYDAY form, through a REAL robot::Robot and competition::runAuton (chunk AF1S-A2,
// 2026-11-15; the AF1S brief's §1.1 "the everyday form", §1.3 and §2.2's A2 tests). Say "host test plant",
// never "simulator": every number here is the test's; nothing has run on a robot.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════════════════
//   * M-A2-1 — THE SHADOW CONTEXT'S LINE: the everyday form reads the camera through
//     `ctx.chassis.deps().ctx`, which is the motion scheduler's SHADOW context, not the Robot's. A slot not
//     carried there is nullptr in every auton: every everyday call would say NO CAMERA. Pinned twice — the
//     slot seen from inside a running auton IS the Robot's, and a RELATIVE-start skills auton FLIPS a
//     Toggle (the plants judge: two strokes, scored ours).
//   * "Two forms, ONE body": the everyday call and the explicit call over the SAME slot give the same report
//     and the same [TGL] line on two identical worlds — byte for byte.
//   * Rule 1's ORDER survives the new door: a MATCH auton is NOT SKILLS before anything about the camera; no
//     Toggle block, or a block that is not ready, is NO CAMERA naming WHY (the slot's own sentence), with
//     nothing commanded — and the robot still drives in the same auton.
//   * The optional POSE BOUND reaches the tracker: with the block's position gate on, a bound stated makes the
//     gate run (`positionChecked`), none skips it — and a FIELD start checks the wall, a RELATIVE one says
//     `wall not checked`.
//   * lookAtToggle(ctx, ...) never moves anything.
//   * A call outside a live run is a programming error (a precondition), as the explicit form's.
//   (AF1S-A2's review, 2026-11-16:)
//   * The excursion against the ROBOT FILE's frozenMoveIn — two files — is a REFUSAL in the everyday form
//     (SPOTS TOO CLOSE), never an exception out of runAuton: at the EQUALITY edge (frozenMoveIn exactly half
//     the excursion) the route's onFail(skipTo(...)) applies and the run goes on; one ulp under it, it flips.
//     competition::toggleVisitBlockProblem() says the same at build.
//   * A pose bound under a start that is NOT FIELD is DROPPED (the estimate is in the robot's own frame, so
//     the position gate would drop the real Toggle as "no band") — and the line says so.
//   * Every number the everyday form hands the body is the slot's and the route's: the bound's position AND
//     heading halves, every tracker number (detail::toggleCameraOf, field by field) — and lookAtToggle(ctx)
//     takes the bound as checkAndFlipToggle(ctx) does.
//   * A MIRRORED entry (MIRROR1-A, landed first): the everyday form visits the IMAGE Toggle.
//   (The verification round, 2026-11-16:)
//   * The excursion check's PLACE in rule 1, in BOTH forms: after NOT SKILLS and the camera's own checks, before
//     FLIPPER NOT READY — a MATCH auton with the spots too close is NOT SKILLS; the spots too close with the flipper
//     left OUT is SPOTS TOO CLOSE (the everyday form) or the precondition (the explicit form), never FLIPPER NOT
//     READY; a tracker the camera's own check refuses is NO CAMERA first.
//   * The tracker numbers handed on are each one's OWN: the 16 numbers of the toggleCameraOf case are pairwise
//     distinct (checked), so a copy that takes one from another cannot pass.

#include "doctest.h"

#include <cmath>
#include <cstddef>
#include <cstring>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/competition/override_mirror.hpp"
#include "shulib/competition/toggle_visit.hpp"
#include "shulib/core/check.hpp"
#include "shulib/units/literals.hpp"
#include "toggle_robot_rig.hpp"

namespace sc = shulib::competition;
namespace ov = shulib::vision::override2026;
namespace trr = toggle_robot_rig;
using shulib::units::Length;
using shulib::units::Time;
using namespace shulib::units::literals;

namespace {

/// What the auton under test saw and returned (the routines below are plain function pointers: an
/// AutonRoutine, as a team's file registers it).
struct Seen {
    shulib::vision::ToggleCamera* slot = nullptr;
    bool slotSeen = false;
    sc::ToggleVisitReport report{};
    sc::TogglePoseBound bound{};
    sc::ToggleVisitRequest request = trr::request();
    bool lookOnly = false;
    bool driveAfter = false;
    bool driveOk = false;
};
Seen g_seen;

shulib::chassis::RoutineResult visitOnce(sc::AutonContext& ctx) {
    g_seen.slot = ctx.chassis.deps().ctx->toggleCamera();
    g_seen.slotSeen = true;
    g_seen.report = g_seen.lookOnly ? sc::lookAtToggle(ctx, g_seen.request, g_seen.bound)
                                    : sc::checkAndFlipToggle(ctx, g_seen.request, g_seen.bound);
    shulib::chassis::Routine r{ctx, "after"};
    if (g_seen.driveAfter) {
        // Back straight off 8 in (the robot faces the wall), wherever the estimate's frame is.
        const shulib::math::Pose2d p = ctx.chassis.pose();
        const double th = p.heading().radians();
        r.reverseTo(p.x() - Length{8.0 * std::cos(th)}, p.y() - Length{8.0 * std::sin(th)}, {.timeout = 3_s});
        g_seen.driveOk = r.result().ok;
    }
    return r.result();
}

void reset() { g_seen = Seen{}; }

}  // namespace

TEST_CASE("AF1S-A2 ctx: M-A2-1 -- the slot reaches the auton through the SHADOW context, and a RELATIVE-start "
          "skills auton FLIPS a Toggle") {
    reset();
    trr::World w;
    shulib::robot::Robot& robot = w.build();
    REQUIRE(robot.toggleCamera() != nullptr);
    const sc::AutonRunResult res =
        w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative()));
    REQUIRE(res.started());
    REQUIRE(g_seen.slotSeen);
    CHECK(g_seen.slot == robot.toggleCamera());  // the Robot's own slot, through the shadow context
    CHECK(g_seen.slot == robot.context().toggleCamera());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::Flipped);
    CHECK(g_seen.report.pushes == 2);
    CHECK(w.toggles[0]->strokes() == 2);
    CHECK(w.toggles[0]->scoredColour() == tv_rig::kRed);
    const std::vector<std::string> lines = w.lines("TGL");
    REQUIRE(lines.size() == 1U);
    CHECK(lines[0].find("YELLOW -> RED after 2 pushes") != std::string::npos);
    CHECK(lines[0].find("wall not checked (relative start)") != std::string::npos);
}

// Bug caught: an everyday form that skips the wall check under a FIELD start (the one check that catches a
// route naming the wrong Toggle), or that loses it through the context's entry.
TEST_CASE("AF1S-A2 ctx: a FIELD-start skills auton flips, the wall CHECKED (the line has no 'wall not checked')") {
    reset();
    trr::World w;
    w.build();
    const sc::AutonRunResult res = w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::field(w.truth())));
    REQUIRE(res.started());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::Flipped);
    CHECK(g_seen.report.wallChecked);
    CHECK(w.toggles[0]->scoredColour() == tv_rig::kRed);
    const std::vector<std::string> lines = w.lines("TGL");
    REQUIRE(lines.size() == 1U);
    CHECK(lines[0].find("wall not checked") == std::string::npos);
    CHECK(lines[0].find("seating not checked") != std::string::npos);
    // The same robot facing the WRONG wall (the route names the right wall's Toggle, the robot faces the
    // blue wall): WRONG WALL before anything is read or moved.
    reset();
    trr::Options o;
    o.start = tv_rig::parkedAt(ov::toggles::kBlueWall, 16.0);
    trr::World w2{o};
    w2.build();
    REQUIRE(w2.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::field(w2.truth()))).started());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::WrongWall);
    CHECK(w2.devices.fakeCamera(trr::kCameraPort).reads() == 0U);  // nothing read
    CHECK(w2.toggles[0]->strokes() == 0);
}

// Bug caught: the new door changing rule 1's ORDER — a MATCH auton must be NOT SKILLS before anything about the
// camera (its "ours" is the alliance's, ALLY1, after the 21st), whether the robot has a Toggle camera or not.
TEST_CASE("AF1S-A2 ctx: a MATCH auton is NOT SKILLS -- with a camera, with no Toggle block, with one not ready -- "
          "and nothing is read, pushed or driven") {
    const auto matchOn = [](const shulib::config::RobotConfig& c) {
        reset();
        trr::World w;
        w.build(c);
        const shulib::math::Pose2d before = w.truth();
        REQUIRE(w.run(trr::skillsEntry("match", &visitOnce, sc::StartPose::relative(), sc::AutonKind::Match))
                    .started());
        CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::NotSkills);
        CHECK(g_seen.report.outcome() == shulib::manipulation::MechanismOutcome::Unconfirmed);
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
        CHECK(w.toggles[0]->strokes() == 0);
        CHECK(std::fabs(w.flipperPlant.trueDeg()) < 1.0);
        CHECK(motion_rig::posErr(w.truth(), before) < 0.05);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0] == "right wall Toggle: not a SKILLS auton (MATCH) -- nothing pushed");
    };
    matchOn(trr::robotFile());
    shulib::config::RobotConfig none = trr::robotFile();
    none.sensors.toggleCamera = {};
    matchOn(none);
    shulib::config::RobotConfig unmeasured = trr::robotFile();
    unmeasured.sensors.camera.placement.measured = false;
    matchOn(unmeasured);
}

// Bug caught: an everyday NO CAMERA that says only "no camera" (the team cannot tell a missing block from an
// unmeasured mount), that pushes or drives anyway, or that leaves the robot unable to drive the rest of the
// auton.
TEST_CASE("AF1S-A2 ctx: NO CAMERA names WHY -- no Toggle block, or the slot's own sentence -- nothing is pushed "
          "or driven, ONE Warn line, and the auton drives on") {
    const auto noCamera = [](const shulib::config::RobotConfig& c, const std::string& why) {
        reset();
        g_seen.driveAfter = true;
        trr::World w;
        w.build(c);
        const shulib::math::Pose2d before = w.truth();
        REQUIRE(w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::field(w.truth()))).started());
        CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::NoCamera);
        CHECK(std::string{g_seen.report.detail} == why);
        CHECK(g_seen.report.outcome() == shulib::manipulation::MechanismOutcome::Unconfirmed);
        CHECK(g_seen.report.pushes == 0);
        CHECK(w.toggles[0]->strokes() == 0);
        CHECK(std::fabs(w.flipperPlant.trueDeg()) < 1.0);
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0] == "right wall Toggle: no camera -- " + why.substr(0, 100) + " -- nothing pushed");
        CHECK(lines[0].size() <= 160U);
        // ...and the auton DRIVES on: the drive after it reached its point.
        CHECK(g_seen.driveOk);
        CHECK(w.truth().x().value() < before.x().value() - 6.0);  // it backed off the wall
    };
    shulib::config::RobotConfig none = trr::robotFile();
    none.sensors.toggleCamera = {};
    noCamera(none, "no Toggle block is declared (the robot file's sensors.toggleCamera)");
    shulib::config::RobotConfig unmeasured = trr::robotFile();
    unmeasured.sensors.camera.placement.measured = false;
    noCamera(unmeasured, "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it");
    shulib::config::RobotConfig badTracker = trr::robotFile();
    badTracker.sensors.toggleCamera.tracker.aliveSeconds = 0.0;
    noCamera(badTracker,
             "the Toggle block's tracker: aliveSeconds UNSET -- how old the newest usable read may be");
}

// Bug caught (a refused CAMERA, not a refused block): the provider refuses the camera at boot — the slot is
// declared but not ready, and the everyday call says so (the boot line has the camera's own reason).
TEST_CASE("AF1S-A2 ctx: the camera refused at boot -- NO CAMERA 'the camera was not built', the robot drives") {
    reset();
    g_seen.driveAfter = true;
    trr::World w;
    w.devices.refusePort(trr::kCameraPort);
    shulib::robot::Robot& robot = w.build();
    CHECK(robot.camera() == nullptr);
    REQUIRE(w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative())).started());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::NoCamera);
    CHECK(std::string{g_seen.report.detail} == "the camera was not built (the boot line's camera words say why)");
    CHECK(g_seen.slot == robot.context().toggleCamera());  // present (declared), not ready
    CHECK(g_seen.driveOk);
}

// Bug caught: an everyday form that ignores the route's pose bound (the robot file's position gate would never
// run from an auton), or that states one when the route did not (the gate run on a pose taken as exact) — in
// checkAndFlipToggle(ctx) AND in lookAtToggle(ctx), each its own call into detail::toggleCameraOf.
TEST_CASE("AF1S-A2 ctx: the optional POSE BOUND reaches the tracker -- the block's position gate runs with one "
          "stated and is skipped without, for the flip and for the look alone") {
    shulib::config::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera.tracker.positionToleranceIn = 3.0;
    c.sensors.toggleCamera.tracker.maxPoseToleranceIn = 4.0;
    c.sensors.toggleCamera.tracker.maxFixRadiusIn = 8.0;
    for (const bool lookOnly : {false, true}) {
        CAPTURE(lookOnly);
        const auto run = [&c, lookOnly](sc::TogglePoseBound bound) {
            reset();
            g_seen.bound = bound;
            g_seen.lookOnly = lookOnly;
            trr::World w;
            w.build(c);
            REQUIRE(w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::field(w.truth()))).started());
            return g_seen.report;
        };
        const sc::ToggleVisitVerdict good =
            lookOnly ? sc::ToggleVisitVerdict::SeenNotOurs : sc::ToggleVisitVerdict::Flipped;
        const sc::ToggleVisitReport with = run({.poseTolerance = Length{1.0}, .headingToleranceDeg = 1.0});
        CHECK(with.positionChecked);
        CHECK(with.verdict == good);
        const sc::ToggleVisitReport without = run({});
        CHECK_FALSE(without.positionChecked);
        CHECK(without.verdict == good);
        // A bound LOOSER than the block's maxPoseToleranceIn is stated but not trusted: the gate is skipped.
        const sc::ToggleVisitReport loose = run({.poseTolerance = Length{6.0}, .headingToleranceDeg = 1.0});
        CHECK_FALSE(loose.positionChecked);
    }
}

// Bug caught (review, 2026-11-16): a bound stated under a RELATIVE start handed to the tracker anyway — runAuton
// set the estimate in the robot's own frame, so the position gate compares it with the season's Toggle places
// and drops the REAL Toggle in plain view ("no band"), every visit UNKNOWN BEFORE for the wrong reason. The
// docs' own literal bound; the gate configured on. The bound is DROPPED and the line says so; under a FIELD
// start the same bound runs the gate (the case above).
TEST_CASE("AF1S-A2 ctx: a pose bound under a RELATIVE start is DROPPED -- it flips, the gate not run, and the line "
          "says 'wall and position not checked (relative start)'") {
    shulib::config::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera.tracker.positionToleranceIn = 3.0;
    c.sensors.toggleCamera.tracker.maxPoseToleranceIn = 4.0;
    c.sensors.toggleCamera.tracker.maxFixRadiusIn = 8.0;
    for (const bool lookOnly : {false, true}) {
        CAPTURE(lookOnly);
        reset();
        g_seen.bound = {.poseTolerance = 2_in, .headingToleranceDeg = 3};
        g_seen.lookOnly = lookOnly;
        trr::World w;
        w.build(c);
        REQUIRE(w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative())).started());
        CHECK(g_seen.report.verdict ==
              (lookOnly ? sc::ToggleVisitVerdict::SeenNotOurs : sc::ToggleVisitVerdict::Flipped));
        CHECK_FALSE(g_seen.report.positionChecked);
        CHECK_FALSE(g_seen.report.wallChecked);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0].find(", wall and position not checked (relative start)") != std::string::npos);
        CHECK(lines[0].size() <= 160U);
    }
    // With NO bound stated under the same start, the line says only what it always said.
    reset();
    trr::World w;
    w.build(c);
    REQUIRE(w.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative())).started());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::Flipped);
    REQUIRE(w.lines("TGL").size() == 1U);
    CHECK(w.lines("TGL")[0].find(", wall not checked (relative start)") != std::string::npos);
    CHECK(w.lines("TGL")[0].find("position") == std::string::npos);
}

// Bug caught: an everyday look that moves something (it is the D1 (c) building block: it never pushes).
TEST_CASE("AF1S-A2 ctx: lookAtToggle(ctx, ...) -- SEEN, NOT OURS on a yellow Toggle, nothing pushed or driven") {
    reset();
    g_seen.lookOnly = true;
    trr::World w;
    w.build();
    const shulib::math::Pose2d before = w.truth();
    REQUIRE(w.run(trr::skillsEntry("look", &visitOnce, sc::StartPose::field(w.truth()))).started());
    CHECK(g_seen.report.verdict == sc::ToggleVisitVerdict::SeenNotOurs);
    CHECK(g_seen.report.before == shulib::vision::ToggleColour::Yellow);
    CHECK(g_seen.report.pushes == 0);
    CHECK(w.toggles[0]->strokes() == 0);
    CHECK(std::fabs(w.flipperPlant.trueDeg()) < 1.0);
    CHECK(motion_rig::posErr(w.truth(), before) < 0.05);
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() > 0U);
}

namespace {
Seen g_explicit;
shulib::chassis::RoutineResult visitExplicit(sc::AutonContext& ctx) {
    shulib::vision::ToggleCamera* slot = ctx.chassis.deps().ctx->toggleCamera();
    g_explicit.slot = slot;
    g_explicit.slotSeen = true;
    if (slot == nullptr) {  // never dereferenced: the case's own check says the slot did not arrive
        return shulib::chassis::Routine{ctx, "after"}.result();
    }
    sc::ExplicitToggleCamera cam{};
    cam.feed = slot->feed();
    cam.model = slot->model();
    cam.tracker = slot->tracker();
    g_explicit.report = sc::checkAndFlipToggle(cam, ctx.chassis, ctx.guard, ctx.mechanismService, ctx.entry,
                                               g_explicit.request, ctx.telemetry);
    return shulib::chassis::Routine{ctx, "after"}.result();
}
}  // namespace

// Bug caught: an everyday form that is a second copy of the step (two bodies drift): on two identical worlds,
// the everyday call and the explicit call over the SAME slot must give the same visit, byte for byte.
TEST_CASE("AF1S-A2 ctx: TWO forms, ONE body -- the everyday call and the explicit call over the same slot give the "
          "same report and the same [TGL] line") {
    trr::Options o;
    o.toggles[0].plant.mode = shulib::sim::ToggleMode::Normal;
    reset();
    trr::World a{o};
    a.build();
    REQUIRE(a.run(trr::skillsEntry("toggle", &visitOnce, sc::StartPose::field(a.truth()))).started());
    g_explicit = Seen{};
    trr::World b{o};
    b.build();
    REQUIRE(b.run(trr::skillsEntry("toggle", &visitExplicit, sc::StartPose::field(b.truth()))).started());
    REQUIRE(g_explicit.slotSeen);
    REQUIRE(g_explicit.slot != nullptr);  // the slot arrived through the shadow context (M-A2-1 stops here)
    const sc::ToggleVisitReport& x = g_seen.report;
    const sc::ToggleVisitReport& y = g_explicit.report;
    CHECK(x.verdict == sc::ToggleVisitVerdict::Flipped);
    CHECK(x.verdict == y.verdict);
    CHECK(x.before == y.before);
    CHECK(x.after == y.after);
    CHECK(x.pushes == y.pushes);
    CHECK(x.seconds == y.seconds);
    CHECK(x.secondsNeeded == y.secondsNeeded);
    CHECK(x.range.lo == y.range.lo);
    CHECK(x.range.hi == y.range.hi);
    CHECK(a.lines("TGL") == b.lines("TGL"));
    CHECK(a.lines("MECH") == b.lines("MECH"));
    CHECK(motion_rig::posErr(a.truth(), b.truth()) == 0.0);
    CHECK(a.toggles[0]->strokes() == b.toggles[0]->strokes());
}

// Bug caught: an everyday call outside a live run that does something instead of failing loudly (an
// AutonContext lives for one guarded run — the explicit form's precondition, kept).
TEST_CASE("AF1S-A2 ctx: outside a live run the everyday call and the everyday look are a PRECONDITION") {
    trr::World w;
    shulib::robot::Robot& robot = w.build();
    const sc::AutonEntry e = trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative());
    sc::AutonContext ctx{robot.chassis(), w.guard, w.sink, w.svc.mechanisms(), e, &w.svc};
    REQUIRE_FALSE(w.guard.running());
    CHECK_THROWS_AS((void)sc::checkAndFlipToggle(ctx, trr::request()), shulib::PreconditionError);
    CHECK_THROWS_AS((void)sc::lookAtToggle(ctx, trr::request()), shulib::PreconditionError);
    CHECK(w.lines("TGL").empty());
    CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
}

// ═══ AF1S-A2's review (2026-11-16) ══════════════════════════════════════════════════════════════════

namespace {
/// What a team's route writes around one visit: the visit with onFail(skipTo("next")), then the next leg.
struct RouteSeen {
    sc::ToggleVisitReport report{};
    bool reachedNext = false;
};
RouteSeen g_route;
shulib::chassis::RoutineResult visitThenNext(sc::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "route"};
    r.onFail(shulib::chassis::skipTo("next")).then([&] {
        g_route.report = g_seen.lookOnly ? sc::lookAtToggle(ctx, g_seen.request, g_seen.bound)
                                         : sc::checkAndFlipToggle(ctx, g_seen.request, g_seen.bound);
        return g_route.report.outcome();
    }, "toggle");
    r.label("next").then([&] {
        g_route.reachedNext = true;
        return shulib::manipulation::MechanismOutcome::Succeeded;
    }, "next leg");
    return r.result();
}
}  // namespace

// Bug caught (review, 2026-11-16, major): the excursion lookFrom - pushFrom checked against the ROBOT FILE's
// frozenMoveIn as a PRECONDITION in the everyday form — two numbers typed at two stations, so a bench change to
// frozenMoveIn after the route was written made the first visit THROW out of then(), the Routine and runAuton:
// the route's onFail never applied, no [TGL] line, the rest of the skills run lost, while the slot said READY.
// At the EQUALITY edge (frozenMoveIn exactly half the excursion: 2 x 6.0 == 16 - 4) it is SPOTS TOO CLOSE —
// nothing read or moved, ONE Warn line naming both numbers, UNCONFIRMED so the skip applies; one ulp under
// the edge the same route flips. The explicit form keeps its precondition (AF1S-A1's case pins it).
TEST_CASE("AF1S-A2 ctx: the excursion against the robot file's frozenMoveIn is a REFUSAL, SPOTS TOO CLOSE, at the "
          "equality edge -- the route's skip applies and the run goes on; one ulp under it, it flips") {
    for (const bool lookOnly : {false, true}) {
        CAPTURE(lookOnly);
        reset();
        g_route = RouteSeen{};
        g_seen.lookOnly = lookOnly;
        shulib::config::RobotConfig c = trr::robotFile();
        c.sensors.toggleCamera.tracker.frozenMoveIn = 6.0;  // 2 x 6.0 == lookFrom - pushFrom == 16 - 4: the EDGE
        REQUIRE(g_seen.request.lookFrom.value() - g_seen.request.pushFrom.value() == 12.0);
        trr::World w;
        shulib::robot::Robot& robot = w.build(c);
        REQUIRE(robot.toggleCamera() != nullptr);  // the slot is READY: the block alone is fine
        sc::AutonRunResult res{};
        REQUIRE_NOTHROW(res = w.run(trr::skillsEntry("toggle", &visitThenNext, sc::StartPose::field(w.truth()))));
        REQUIRE(res.started());
        CHECK(g_route.report.verdict == sc::ToggleVisitVerdict::SpotsTooClose);
        CHECK(g_route.report.outcome() == shulib::manipulation::MechanismOutcome::Unconfirmed);
        CHECK_FALSE(g_route.report.ok());
        CHECK(g_route.reachedNext);  // the route's onFail(skipTo("next")) applied
        CHECK(res.routine.failedOn == 1);
        CHECK(res.routine.stoppedAt == 0);
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);  // nothing read
        CHECK(w.toggles[0]->strokes() == 0);                          // nothing pushed
        CHECK(std::fabs(w.flipperPlant.trueDeg()) < 1.0);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0] == "right wall Toggle: spots too close -- lookFrom - pushFrom 12.0 in, needs more than 2 x "
                          "frozenMoveIn 6.0 in -- nothing pushed");
        CHECK(lines[0].size() <= 160U);
        int warns = 0;
        for (int i = 0; i < w.sink.size(); ++i) {
            if (w.sink.at(i).subsystem == "TGL" && w.sink.at(i).level == shulib::hal::LogLevel::Warn) {
                ++warns;
            }
        }
        CHECK(warns == 1);
    }
    // One ulp UNDER the edge: twice it is just under the excursion — the step runs and flips.
    reset();
    g_route = RouteSeen{};
    shulib::config::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera.tracker.frozenMoveIn = std::nextafter(6.0, 0.0);
    REQUIRE(2.0 * c.sensors.toggleCamera.tracker.frozenMoveIn < 12.0);
    trr::World w;
    w.build(c);
    REQUIRE(w.run(trr::skillsEntry("toggle", &visitThenNext, sc::StartPose::field(w.truth()))).started());
    CHECK(g_route.report.verdict == sc::ToggleVisitVerdict::Flipped);
    CHECK(w.toggles[0]->scoredColour() == tv_rig::kRed);
    CHECK(g_route.reachedNext);
}

namespace {
/// The order case's auton (the verification round, 2026-11-16): the flipper sent OUT to its "flip" preset first
/// when the scene asks (a route that forgot to stow it), then ONE visit — the everyday form, or the EXPLICIT form
/// over the slot's own handles with the scene's change to them, its precondition caught and kept.
struct OrderScene {
    bool flipOut = false;
    bool explicitForm = false;
    void (*change)(sc::ExplicitToggleCamera&) = nullptr;  ///< the explicit handles' change, if any
    bool visited = false;
    bool threw = false;
    std::string what;
    sc::ToggleVisitReport report{};
};
OrderScene g_order;
shulib::chassis::RoutineResult flipOutThenVisit(sc::AutonContext& ctx) {
    shulib::chassis::Routine r{ctx, "order"};
    if (g_order.flipOut) {
        r.move(toggle_robot::flipper["flip"], 2_s);
    }
    r.then([&] {
        g_order.visited = true;
        if (!g_order.explicitForm) {
            g_order.report = sc::checkAndFlipToggle(ctx, trr::request());
            return g_order.report.outcome();
        }
        sc::ExplicitToggleCamera cam{};
        if (shulib::vision::ToggleCamera* slot = ctx.chassis.deps().ctx->toggleCamera(); slot != nullptr) {
            cam.feed = slot->feed();
            cam.model = slot->model();
            cam.tracker = slot->tracker();
        }
        if (g_order.change != nullptr) {
            g_order.change(cam);
        }
        try {
            g_order.report = sc::checkAndFlipToggle(cam, ctx.chassis, ctx.guard, ctx.mechanismService, ctx.entry,
                                                    trr::request(), ctx.telemetry);
        } catch (const shulib::PreconditionError& e) {
            g_order.threw = true;
            g_order.what = e.what();
            return shulib::manipulation::MechanismOutcome::Unconfirmed;
        }
        return g_order.report.outcome();
    }, "toggle");
    return r.result();
}

/// One scene of the order case: the robot file's frozenMoveIn (6.0: the EQUALITY edge against the 12-in excursion,
/// so the spots are too close; 2.0: the file's own, fine), the flipper sent out or not, the form, the auton's kind,
/// the explicit handles' change.
struct Order {
    double frozenMoveIn = 2.0;
    bool flipOut = false;
    bool explicitForm = false;
    sc::AutonKind kind = sc::AutonKind::Skills;
    void (*change)(sc::ExplicitToggleCamera&) = nullptr;
};
void runOrder(const Order& o, trr::World& w) {
    g_order = OrderScene{};
    g_order.flipOut = o.flipOut;
    g_order.explicitForm = o.explicitForm;
    g_order.change = o.change;
    shulib::config::RobotConfig c = trr::robotFile();
    c.sensors.toggleCamera.tracker.frozenMoveIn = o.frozenMoveIn;
    w.build(c);
    REQUIRE(w.run(trr::skillsEntry("order", &flipOutThenVisit, sc::StartPose::field(w.truth()), o.kind)).started());
    REQUIRE(g_order.visited);  // the flipper's move (when asked) succeeded and the visit ran
}
void refuseAliveSeconds(sc::ExplicitToggleCamera& cam) { cam.tracker.aliveSeconds = 0.0; }
}  // namespace

// Bug caught (the verification round, 2026-11-16 — mutants V-ORDER and V-ORDER2 survived the whole suite, and so
// did V-ORDER2X and V-ORDER3, cut for the explicit form): the excursion check's PLACE in rule 1 unpinned. It sits
// after the camera's own checks and before FLIPPER NOT READY, in both forms (the banner's rule 1). Each refusal is
// shown ALONE on the same scene first, so no answer below is the only one its scene could give. (In the everyday
// form a slot that is not ready hands the body no tracker numbers — frozenMoveIn 0 — so the camera's own checks
// answer first there by construction; the explicit form's caller can hand both, so its order is pinned here.)
TEST_CASE("AF1S-A2 ctx: the excursion check's PLACE in rule 1 -- after NOT SKILLS and the camera's own checks, "
          "before FLIPPER NOT READY, in both forms") {
    const std::string spotsLine = "right wall Toggle: spots too close -- lookFrom - pushFrom 12.0 in, needs more than "
                                  "2 x frozenMoveIn 6.0 in -- nothing pushed";
    // ── each refusal ALONE ──
    {  // the flipper sent out, the spots fine: FLIPPER NOT READY (it truly is away from `stow`) — in both forms
        for (const bool explicitForm : {false, true}) {
            CAPTURE(explicitForm);
            trr::World w;
            runOrder({.frozenMoveIn = 2.0, .flipOut = true, .explicitForm = explicitForm}, w);
            CHECK_FALSE(g_order.threw);
            CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::FlipperNotReady);
            CHECK(std::string{g_order.report.detail} == "is not at its back position (put it there first)");
            CHECK(w.flipperPlant.trueDeg() > 80.0);
            CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
        }
    }
    {  // the spots too close, the flipper stowed: SPOTS TOO CLOSE (everyday), the precondition (explicit)
        trr::World w;
        runOrder({.frozenMoveIn = 6.0}, w);
        CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::SpotsTooClose);
        trr::World w2;
        runOrder({.frozenMoveIn = 6.0, .explicitForm = true}, w2);
        CHECK(g_order.threw);
    }
    {  // a MATCH auton, the spots fine: NOT SKILLS
        trr::World w;
        runOrder({.frozenMoveIn = 2.0, .kind = sc::AutonKind::Match}, w);
        CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::NotSkills);
    }
    // ── the excursion check AFTER NOT SKILLS (V-ORDER2, V-ORDER2X): a MATCH auton with the spots too close ──
    for (const bool explicitForm : {false, true}) {
        CAPTURE(explicitForm);
        trr::World w;
        runOrder({.frozenMoveIn = 6.0, .explicitForm = explicitForm, .kind = sc::AutonKind::Match}, w);
        CHECK_FALSE(g_order.threw);
        CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::NotSkills);
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0] == "right wall Toggle: not a SKILLS auton (MATCH) -- nothing pushed");
    }
    // ── the excursion check BEFORE FLIPPER NOT READY (V-ORDER): the spots too close AND the flipper left out ──
    {
        trr::World w;
        runOrder({.frozenMoveIn = 6.0, .flipOut = true}, w);
        CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::SpotsTooClose);
        CHECK(w.flipperPlant.trueDeg() > 80.0);  // still out: the step never moves it
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
        CHECK(w.toggles[0]->strokes() == 0);
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        CHECK(lines[0] == spotsLine);
        // The explicit form: its precondition, at the same place — never FLIPPER NOT READY.
        trr::World w2;
        runOrder({.frozenMoveIn = 6.0, .flipOut = true, .explicitForm = true}, w2);
        CHECK(g_order.threw);
        CHECK(g_order.what.find("lookFrom - pushFrom must be MORE than twice the tracker's frozenMoveIn")
              != std::string::npos);
        CHECK(g_order.report.verdict != sc::ToggleVisitVerdict::FlipperNotReady);
        CHECK(w2.lines("TGL").empty());  // a programming error: no verdict, no line
    }
    // ── the excursion check AFTER the camera's own checks (V-ORDER3): the explicit form, a tracker the camera's
    //    check refuses AND the spots too close — NO CAMERA naming the tracker, not the precondition ──
    {
        trr::World w;
        runOrder({.frozenMoveIn = 6.0, .explicitForm = true, .change = &refuseAliveSeconds}, w);
        CHECK_FALSE(g_order.threw);
        CHECK(g_order.report.verdict == sc::ToggleVisitVerdict::NoCamera);
        CHECK(std::string{g_order.report.detail} == "aliveSeconds UNSET -- how old the newest usable read may be");
        CHECK(w.devices.fakeCamera(trr::kCameraPort).reads() == 0U);
    }
}

// Bug caught: no way to see the pair AT BUILD — a route that can see its robot file's block can static_assert
// it. The same edge as the run-time refusal; a block not declared, or one whose own numbers are refused, is
// the block's problem (toggleCameraProblem), not the pair's; a request's own problem comes first.
namespace {
constexpr shulib::vision::ToggleCameraConfig blockWithFrozen(double frozenMoveIn) {
    shulib::vision::ToggleCameraConfig b{};
    b.tracker = {.ids = {1, 2, 3},
                 .holdSeconds = 0.15,
                 .aliveSeconds = 0.10,
                 .minAspect = 3.0,
                 .minLengthFraction = 0.6,
                 .maxLengthFraction = 1.3,
                 .minRangeIn = 2.0,
                 .maxRangeIn = 96.0,
                 .bearingWindowDeg = 25.0,
                 .frozenMoveIn = frozenMoveIn,
                 .frozenTurnDeg = 2.0};
    b.feed.minReadPeriodSeconds = 0.033;
    return b;
}
constexpr sc::ToggleVisitRequest kEdgeVisit = sc::toggleVisit({.toggle = ov::toggles::kRightWall,
                                                              .flipper = toggle_robot::flipper["flip"],
                                                              .back = toggle_robot::flipper["stow"],
                                                              .lookFrom = 16_in,
                                                              .offBy = 6_in,
                                                              .pushFrom = 4_in,
                                                              .lookWithin = 0.6_s,
                                                              .flipBound = 1.2_s,
                                                              .backBound = 1.2_s,
                                                              .settle = 0.3_s,
                                                              .driveBound = 1.5_s,
                                                              .margin = 0.3_s,
                                                              .maxImageAge = 0.1_s});
static_assert(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(5.0)) == nullptr);
static_assert(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(6.0)) != nullptr);  // the equality edge
}  // namespace

TEST_CASE("AF1S-A2 toggleVisitBlockProblem: the pair at build -- the equality edge refused, one ulp under it "
          "accepted; an undeclared or refused block and a request's own problem each in their place") {
    CHECK(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(std::nextafter(6.0, 0.0))) == nullptr);
    const char* p = sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(6.0));
    REQUIRE(p != nullptr);
    CHECK(std::string{p}.find("MORE than twice the Toggle block's frozenMoveIn") != std::string::npos);
    CHECK(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(7.0)) != nullptr);
    // Not the pair's to judge: no block (the step says NO CAMERA), a block whose frozenMoveIn is unset or NaN
    // (toggleCameraProblem names it).
    CHECK(sc::toggleVisitBlockProblem(kEdgeVisit, shulib::vision::ToggleCameraConfig{}) == nullptr);
    CHECK(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(0.0)) == nullptr);
    CHECK(sc::toggleVisitBlockProblem(kEdgeVisit, blockWithFrozen(std::nan(""))) == nullptr);
    // A request's own problem comes FIRST, with its own sentence.
    sc::ToggleVisitRequest bad = kEdgeVisit;
    bad.maxPushes = 3;
    REQUIRE(sc::toggleVisitRequestProblem(bad) != nullptr);
    CHECK(std::string{sc::toggleVisitBlockProblem(bad, blockWithFrozen(6.0))} == sc::toggleVisitRequestProblem(bad));
}

namespace {
/// Converts to any member's type, so `T{AnyMember{} x n}` compiles exactly when the aggregate T has n members
/// or more (never defined: used only where nothing is evaluated).
struct AnyMember {
    template <class M>
    operator M() const noexcept;
};
/// True when the aggregate T takes N brace initialisers but not N + 1 — it has EXACTLY N members.
template <class T, std::size_t... I>
constexpr bool membersExactly(std::index_sequence<I...>) {
    return requires { T{(static_cast<void>(I), AnyMember{})...}; }
           && !requires { T{(static_cast<void>(I), AnyMember{})..., AnyMember{}}; };
}
}  // namespace

// Bug caught (review, 2026-11-16 — mutants R1 and R10 survived): the everyday form's handles losing a number on
// the way — the bound's HEADING half (a heading taken as exact: the field fix too narrow right after a wall reset),
// or a tracker number changed after it was copied from the slot. An ASYMMETRIC bound, and a block with all 16 of the
// tracker's numbers set (3 colour ids, 13 doubles) to PAIRWISE DISTINCT values — checked below, so a copy that takes
// one field from another (mutants XW1, XW2) cannot pass — compared field by field, handles against slot and slot
// against the robot file; under a RELATIVE start the bound is dropped whole. (The verification round, 2026-11-16:
// the first cut said "each distinct" over values that repeated — 2.0 three times, 3.0 twice — and "17 numbers".)
TEST_CASE("AF1S-A2 detail::toggleCameraOf: the slot's feed, model and EVERY tracker number, the bound's position AND "
          "heading -- and no bound under a RELATIVE start") {
    shulib::config::RobotConfig c = trr::robotFile();
    shulib::vision::ToggleTrackerConfig& t = c.sensors.toggleCamera.tracker;
    // The ids stay the camera's taught colours (1, 2, 3 — the slot checks them); every double differs from every
    // other number in the block, and the tracker accepts them all (the slot is READY below).
    t.holdSeconds = 0.15;
    t.aliveSeconds = 0.10;
    t.minAspect = 3.25;
    t.minLengthFraction = 0.6;
    t.maxLengthFraction = 1.3;
    t.minRangeIn = 1.5;
    t.maxRangeIn = 96.0;
    t.bearingWindowDeg = 25.0;
    t.frozenMoveIn = 2.25;
    t.frozenTurnDeg = 2.5;
    t.positionToleranceIn = 3.5;
    t.maxPoseToleranceIn = 4.0;
    t.maxFixRadiusIn = 8.0;
    // The case's premise, CHECKED: 16 numbers — the ids block's EXACTLY 3 members and the config's EXACTLY 14 (the
    // ids block and 13 more), counted by brace-initialising each (a member added ANYWHERE stops the build here,
    // even a small one the ids' padding would hide from sizeof — add it to this list and to sameNumbers below);
    // pairwise distinct.
    static_assert(membersExactly<shulib::vision::ToggleColourIds>(std::make_index_sequence<3>{})
                      && membersExactly<shulib::vision::ToggleTrackerConfig>(std::make_index_sequence<14>{}),
                  "ToggleTrackerConfig gained or lost a member: add it to this case's numbers and to sameNumbers");
    const double numbers[] = {static_cast<double>(t.ids.red), static_cast<double>(t.ids.blue),
                              static_cast<double>(t.ids.yellow), t.holdSeconds, t.aliveSeconds, t.minAspect,
                              t.minLengthFraction, t.maxLengthFraction, t.minRangeIn, t.maxRangeIn,
                              t.bearingWindowDeg, t.frozenMoveIn, t.frozenTurnDeg, t.positionToleranceIn,
                              t.maxPoseToleranceIn, t.maxFixRadiusIn};
    static_assert(std::size(numbers) == 16U);
    for (std::size_t i = 0; i < std::size(numbers); ++i) {
        for (std::size_t j = i + 1; j < std::size(numbers); ++j) {
            CAPTURE(i);
            CAPTURE(j);
            CHECK(numbers[i] != numbers[j]);
        }
    }
    const auto sameNumbers = [](const shulib::vision::ToggleTrackerConfig& a,
                                const shulib::vision::ToggleTrackerConfig& b) {
        CHECK(a.ids.red == b.ids.red);
        CHECK(a.ids.blue == b.ids.blue);
        CHECK(a.ids.yellow == b.ids.yellow);
        CHECK(a.holdSeconds == b.holdSeconds);
        CHECK(a.aliveSeconds == b.aliveSeconds);
        CHECK(a.minAspect == b.minAspect);
        CHECK(a.minLengthFraction == b.minLengthFraction);
        CHECK(a.maxLengthFraction == b.maxLengthFraction);
        CHECK(a.minRangeIn == b.minRangeIn);
        CHECK(a.maxRangeIn == b.maxRangeIn);
        CHECK(a.bearingWindowDeg == b.bearingWindowDeg);
        CHECK(a.frozenMoveIn == b.frozenMoveIn);
        CHECK(a.frozenTurnDeg == b.frozenTurnDeg);
        CHECK(a.positionToleranceIn == b.positionToleranceIn);
        CHECK(a.maxPoseToleranceIn == b.maxPoseToleranceIn);
        CHECK(a.maxFixRadiusIn == b.maxFixRadiusIn);
    };
    trr::World w;
    shulib::robot::Robot& robot = w.build(c);
    shulib::vision::ToggleCamera* slot = robot.toggleCamera();
    REQUIRE(slot != nullptr);
    const sc::TogglePoseBound bound{.poseTolerance = 2_in, .headingToleranceDeg = 7.0};
    for (const bool field : {true, false}) {
        CAPTURE(field);
        const sc::AutonEntry e =
            trr::skillsEntry("toggle", &visitOnce, field ? sc::StartPose::field(w.truth()) : sc::StartPose::relative());
        sc::AutonContext ctx{robot.chassis(), w.guard, w.sink, w.svc.mechanisms(), e, &w.svc};
        const sc::ExplicitToggleCamera cam = sc::detail::toggleCameraOf(ctx, bound);
        CHECK(cam.feed == slot->feed());
        CHECK(cam.model == slot->model());
        CHECK(cam.absence == nullptr);
        CHECK(sc::detail::poseBoundDropped(ctx, bound) == !field);
        CHECK(cam.poseTolerance.value() == (field ? 2.0 : 0.0));
        CHECK(cam.headingToleranceDeg == (field ? 7.0 : 0.0));
        {
            INFO("the handles against the slot");
            sameNumbers(cam.tracker, slot->tracker());
        }
        {
            INFO("the slot against the robot file");  // the same 16 numbers, each set
            sameNumbers(slot->tracker(), t);
        }
    }
    // No bound stated: nothing dropped, nothing stated.
    const sc::AutonEntry e = trr::skillsEntry("toggle", &visitOnce, sc::StartPose::relative());
    sc::AutonContext ctx{robot.chassis(), w.guard, w.sink, w.svc.mechanisms(), e, &w.svc};
    CHECK_FALSE(sc::detail::poseBoundDropped(ctx, sc::TogglePoseBound{}));
}

namespace {
sc::ToggleVisitReport g_mirrored{};
shulib::chassis::RoutineResult visitRedWall(sc::AutonContext& ctx) {
    g_mirrored = sc::checkAndFlipToggle(ctx, trr::request(ov::toggles::kRedWall));
    return shulib::chassis::Routine{ctx, "after"}.result();
}
}  // namespace

// Bug caught (owed by whichever of MIRROR1-A and AF1S-A2 landed second — MIRROR1-A landed first): an everyday
// visit that does not swap the Toggle for its IMAGE under a mirrored entry. Robot two truly parked at the RIGHT
// wall's look spot; the route, written for robot one, names the RED wall's Toggle; through runAuton on a real
// Robot. Mirrored: runAuton seats the estimate at robot two's start (the right wall's look spot), and the step
// FLIPS the right wall's Toggle (ours RED either way), the wall checked against the IMAGE Toggle, its line naming
// the right wall — without the swap, that estimate faces 90 degrees off the red wall: WRONG WALL. The control,
// the same entry NOT mirrored: the estimate is seated at robot one's start, and the visit names the red wall.
TEST_CASE("AF1S-A2 ctx: a MIRRORED entry -- the everyday form visits the IMAGE Toggle (robot one's red wall is "
          "robot two's right wall); unmirrored, the same route names the red wall") {
    for (const bool mirror : {true, false}) {
        CAPTURE(mirror);
        g_mirrored = sc::ToggleVisitReport{};
        trr::Options o;
        o.start = tv_rig::parkedAt(ov::toggles::kRightWall, 16.0);
        trr::World w{o};
        w.build();
        const sc::AutonEntry asWritten = trr::skillsEntry(
            "toggle", &visitRedWall, sc::StartPose::field(tv_rig::parkedAt(ov::toggles::kRedWall, 16.0)));
        const sc::AutonEntry e = mirror ? sc::mirrored(asWritten, sc::override2026::kSkillsMirror) : asWritten;
        REQUIRE(w.run(e).started());
        const std::vector<std::string> lines = w.lines("TGL");
        REQUIRE(lines.size() == 1U);
        if (mirror) {
            CHECK(g_mirrored.verdict == sc::ToggleVisitVerdict::Flipped);
            CHECK(g_mirrored.toggle == ov::toggles::kRightWall);
            CHECK(g_mirrored.wallChecked);
            CHECK(w.toggles[0]->strokes() == 2);
            CHECK(w.toggles[0]->scoredColour() == tv_rig::kRed);
            CHECK(lines[0].rfind("right wall Toggle: YELLOW -> RED after 2 pushes", 0) == 0);
        } else {
            CHECK(g_mirrored.toggle == ov::toggles::kRedWall);
            CHECK(lines[0].rfind("red wall Toggle: ", 0) == 0);
        }
    }
}
