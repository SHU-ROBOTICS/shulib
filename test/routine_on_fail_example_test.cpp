// STEP1-A — the code the recipe chapter's on-failure section and the cookbook's two STEP1-A recipes
// quote (docs/guide/09-the-recipe-api.md, case guide-09e; docs/cookbook/02-when-a-step-fails.md,
// cases cookbook-02c and cookbook-02d): every quoted line is here VERBATIM, compiled, and run through
// the REAL competition::runAuton on the host test plant (the doc gate's check-examples finds each line
// in this file). Every recipe is written in the CONTEXT form — `Routine r{ctx, "..."}` inside an auton
// taking `AutonContext& ctx` — because only that form gives the Routine the run guard, and so the
// deadline rule (a Routine built from a bare Chassis has only the CANCELLED half, and says so once).
// Robot x is test/mechanism_fixtures.hpp's. Nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <string>
#include <string_view>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"

namespace step1a {
using namespace mech_fixture::robot_x;  // the robot's own namespace: its declarations and the words

// ── cookbook-02c: carry on past a missed piece ──────────────────────────────────────────
RoutineResult twoPieces(AutonContext& ctx) {
    Routine r{ctx, "two-pieces"};
    r.driveTo(24_in, 0_in, {.positionTolerance = 1_in});      // "arrive within 1 in"
    r.onFail(carryOn).run(intake, 12_volt, seesPiece, 1_s);  // a miss is counted, not the end
    r.driveTo(48_in, 0_in, {.positionTolerance = 1_in});      // the next piece, straight on
    r.onFail(carryOn).run(intake, 12_volt, seesPiece, 1_s);
    r.move(arm["score"], 1.5_s);
    return r.result();
}

// ── cookbook-02d: give up on this piece, go for the next ────────────────────────────────
RoutineResult nextPiece(AutonContext& ctx) {
    Routine r{ctx, "next-piece"};
    r.driveTo(24_in, 0_in, {.positionTolerance = 1_in});
    r.onFail(skipTo("second piece")).run(intake, 12_volt, seesPiece, 1_s);
    r.move(arm["score"], 1.5_s);                              // only worth doing with a piece
    r.move(arm["stow"], 1.5_s);
    r.label("second piece").driveTo(48_in, 0_in, {.positionTolerance = 1_in});
    r.run(intake, 12_volt, seesPiece, 1_s);
    return r.result();
}

// ── guide-09e: the three choices and a label ─────────────────────────────────────────────
RoutineResult threeChoices(AutonContext& ctx) {
    Routine r{ctx, "three-choices"};
    r.onFail(carryOn).driveTo(24_in, 0_in, {.timeout = 2.5_s, .positionTolerance = 1_in});
    r.onFail(skipTo("third piece")).run(intake, 12_volt, seesPiece, 2_s);
    r.move(arm["score"], 1.5_s);
    r.label("third piece").face(0_in, 48_in, {.headingTolerance = 3_deg});
    return r.result();
}
}  // namespace step1a

namespace {
/// How many RTN lines contain `needle`.
int rtn(const auton_rig::RobotX& x, const std::string& needle) { return x.count("RTN", needle); }
}  // namespace

TEST_CASE("cookbook-02c: carry on past a missed piece -- the miss is counted, the auton goes on") {
    // Bug caught: the recipe's promise broken — the first grab misses (no piece until after it), the
    // auton must still drive to the second piece, grab it and score: DONE, 1 FAILED, never a stop.
    auton_rig::RobotX x;
    bool firstGrabOver = false;
    x.world.onPace = [&] {
        // the piece shows up only once the robot is near the second spot (x past 44 in)
        if (!firstGrabOver && x.rig.h.truePose().x().value() > 44.0) {
            firstGrabOver = true;
            x.world.sensorSeesAt = x.now();
        }
    };
    const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                   auton_rig::entryFor("two-pieces", &step1a::twoPieces), nullptr,
                                                   x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CHECK(firstGrabOver);
    CHECK(res.routine.steps == 5);
    CHECK(res.routine.completed == 4);
    CHECK(res.routine.failedOn == 1);
    CHECK(res.routine.firstFailedAt == 2);
    CHECK(res.routine.stoppedAt == 0);
    CHECK_FALSE(res.routine.ok);
    CHECK(res.guard.endActionSucceeded);
    CHECK(rtn(x, "routine 'two-pieces': step 2 (intake) FAILED: mechanism step FAILED (TIMED_OUT) — carrying on; "
                 "drive safed") == 1);
    CHECK(std::abs(x.armPlant.trueDeg() - 95.0) < 1.5);  // it scored the second piece
}

TEST_CASE("cookbook-02d: give up on this piece, go for the next -- the score is jumped, the auton resumes at "
          "the label") {
    // Bug caught: the recipe scoring with empty jaws (the jumped arm moves run), or never reaching the
    // second piece (the skip never lands).
    auton_rig::RobotX x;
    bool atSecond = false;
    x.world.onPace = [&] {
        if (!atSecond && x.rig.h.truePose().x().value() > 44.0) {
            atSecond = true;
            x.world.sensorSeesAt = x.now();
        }
    };
    const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                   auton_rig::entryFor("next-piece", &step1a::nextPiece), nullptr,
                                                   x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CHECK(atSecond);
    CHECK(res.routine.steps == 6);
    CHECK(res.routine.jumped == 2);
    CHECK(res.routine.failedOn == 1);
    CHECK(res.routine.completed == 3);
    CHECK(res.routine.stoppedAt == 0);
    CHECK(std::string_view{res.routine.skipPendingTo}.empty());
    CHECK(std::abs(x.armPlant.trueDeg()) < 5.0);  // the arm never moved to score
    CHECK(rtn(x, "jumped — skipping to 'second piece'") == 2);
    CHECK(rtn(x, "routine 'next-piece': step 3 (arm) jumped — skipping to 'second piece'") == 1);  // guide 11 quotes it
    CHECK(rtn(x, "the skip to 'second piece' landed at step 5 (driveTo)") == 1);
}

TEST_CASE("guide-09e: the three choices and a label, as the chapter writes them") {
    // Bug caught: the chapter's listing not compiling, or not doing what its prose says: the drive
    // arrives (within 1 in), the grab misses and skips the score, the face lands on its label.
    auton_rig::RobotX x;
    const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                   auton_rig::entryFor("three-choices", &step1a::threeChoices),
                                                   nullptr, x.sink, x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    CHECK(res.routine.steps == 4);
    CHECK(res.routine.completed == 2);
    CHECK(res.routine.failedOn == 1);
    CHECK(res.routine.jumped == 1);
    CHECK(res.routine.stoppedAt == 0);
    CHECK(std::abs(x.armPlant.trueDeg()) < 5.0);
}
