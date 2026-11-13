// STEP1-A — the ON-FAILURE CHOICE (chassis/routine.hpp's STEP1-A block; the STEP1 brief's D1, D3, A1–A4,
// A7), on the host test plant: the default rule unchanged (ST1, against golden chains captured before
// STEP1-A existed), carry on (ST6, ST27), skip ahead to a label (ST4, ST5, ST28), a target behind
// (ST24), the modifiers' one-step binding (ST26), a CANCELLED then() action (ST16's then() half), the
// guard-less Routine's one Warn (ST25's half). The deadline's rule is test/routine_cut_rule_test.cpp's;
// the mechanism marks (A6) are test/mechanism_auton_steps_test.cpp's. Every case names the bug it would
// catch. Say "host test plant", never "simulator". Nothing here has run on a robot.

#include "doctest.h"

#include <cmath>
#include <string>
#include <string_view>
#include <vector>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "routine_golden_chains.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using golden::Tank;
using shulib::chassis::carryOn;
using shulib::chassis::OnFail;
using shulib::chassis::Routine;
using shulib::chassis::RoutineStopCause;
using shulib::chassis::skipTo;
using shulib::chassis::stopChain;
using shulib::control::ExitReason;
using shulib::hal::LogLevel;
using shulib::manipulation::MechanismOutcome;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;

namespace {

/// How many RTN lines AT `level` contain `needle`.
int rtn(const Tank& t, LogLevel level, std::string_view needle) {
    int n = 0;
    for (int i = 0; i < t.sink.size(); ++i) {
        const auto& e = t.sink.at(i);
        n += (e.subsystem == "RTN" && e.level == level && e.message.find(needle) != std::string::npos) ? 1 : 0;
    }
    return n;
}
/// How many RTN lines (any level) contain `needle`.
int rtnAny(const Tank& t, std::string_view needle) {
    return rtn(t, LogLevel::Warn, needle) + rtn(t, LogLevel::Info, needle) + rtn(t, LogLevel::Debug, needle) +
           rtn(t, LogLevel::Error, needle);
}

}  // namespace

TEST_CASE("STEP1-A ST1: the DEFAULT rule is bit-identical -- golden chains captured at the base (ef6162e)") {
    // Bug caught: ANY change to the default policy — a count, a stop or skip line, a cause, an exit, a
    // mechanism verdict — from the on-failure machinery threaded through every step's door and through
    // recordStop. The literals were printed by the SAME chains (test/routine_golden_chains.hpp) compiled
    // against the base tree, before STEP1-A existed (the executor's probe, logged in
    // docs/internal/chunks/STEP1-PROGRESS.md: base and STEP1-A outputs byte-identical). Every step form; five failure kinds
    // (a motion timeout, a wait timeout, a then() bool, a then() mechanism verdict, an incomplete
    // trajectory) and a mechanism step's TIMED_OUT.
    CHECK(golden::chainEverything() == R"GOLD(ok=1 steps=16 completed=16 skipped=0 stoppedAt=0 stoppedName= cause=0 exit=0 mv=0)GOLD");
    CHECK(golden::chainFails(0) == R"GOLD(ok=0 steps=5 completed=1 skipped=3 stoppedAt=2 stoppedName=strafeTo cause=1 exit=2 mv=0
1|routine 'g2-fails' STOPPED at step 2 (strafeTo): motion TIMEOUT — skipping the rest; drive safed
2|routine 'g2-fails': step 3 (driveTo) skipped — stopped at step 2
2|routine 'g2-fails': step 4 (pause) skipped — stopped at step 2
2|routine 'g2-fails': step 5 (never) skipped — stopped at step 2)GOLD");
    CHECK(golden::chainFails(1) == R"GOLD(ok=0 steps=5 completed=1 skipped=3 stoppedAt=2 stoppedName=wait for nothing cause=2 exit=0 mv=0
1|routine 'g2-fails' STOPPED at step 2 (wait for nothing): wait TIMEOUT — skipping the rest; drive safed
2|routine 'g2-fails': step 3 (driveTo) skipped — stopped at step 2
2|routine 'g2-fails': step 4 (pause) skipped — stopped at step 2
2|routine 'g2-fails': step 5 (never) skipped — stopped at step 2)GOLD");
    CHECK(golden::chainFails(2) == R"GOLD(ok=0 steps=5 completed=1 skipped=3 stoppedAt=2 stoppedName=says no cause=3 exit=0 mv=0
1|routine 'g2-fails' STOPPED at step 2 (says no): action FAILED — skipping the rest; drive safed
2|routine 'g2-fails': step 3 (driveTo) skipped — stopped at step 2
2|routine 'g2-fails': step 4 (pause) skipped — stopped at step 2
2|routine 'g2-fails': step 5 (never) skipped — stopped at step 2)GOLD");
    CHECK(golden::chainFails(3) == R"GOLD(ok=0 steps=5 completed=1 skipped=3 stoppedAt=2 stoppedName=grab cause=4 exit=0 mv=0
1|routine 'g2-fails' STOPPED at step 2 (grab): mechanism FAILED (TIMED_OUT) — skipping the rest; drive safed
2|routine 'g2-fails': step 3 (driveTo) skipped — stopped at step 2
2|routine 'g2-fails': step 4 (pause) skipped — stopped at step 2
2|routine 'g2-fails': step 5 (never) skipped — stopped at step 2)GOLD");
    CHECK(golden::chainFails(4) == R"GOLD(ok=0 steps=5 completed=1 skipped=3 stoppedAt=2 stoppedName=followTrajectory cause=1 exit=2 mv=0
1|routine 'g2-fails' STOPPED at step 2 (followTrajectory): motion TIMEOUT — skipping the rest; drive safed
2|routine 'g2-fails': step 3 (driveTo) skipped — stopped at step 2
2|routine 'g2-fails': step 4 (pause) skipped — stopped at step 2
2|routine 'g2-fails': step 5 (never) skipped — stopped at step 2)GOLD");
    CHECK(golden::chainMechanisms() == R"GOLD(ok=0 steps=9 completed=5 skipped=3 stoppedAt=6 stoppedName=intake cause=5 exit=0 mv=3
1|routine 'g6-mechanisms' STOPPED at step 6 (intake): mechanism step FAILED (TIMED_OUT) — skipping the rest; drive safed
2|routine 'g6-mechanisms': step 7 (arm) skipped — stopped at step 6
2|routine 'g6-mechanisms': step 8 (flywheel) skipped — stopped at step 6
2|routine 'g6-mechanisms': step 9 (intake) skipped — stopped at step 6)GOLD");
}

TEST_CASE("STEP1-A ST1: under the default rule the six appended fields stay at their defaults") {
    // Bug caught: a default chain that fills a carried-on field (failedOn, firstFailed..., jumped,
    // skipPendingTo) — the programs read them to choose "DONE, n FAILED", so a default stop must leave
    // them empty, and ok() must read exactly stoppedAt == 0.
    Tank t;
    Routine r{t.ch, "default"};
    r.then([] { return true; }, "fine").then([] { return false; }, "fails").then([] { return true; }, "skipped");
    const auto res = r.result();
    CHECK_FALSE(r.ok());
    CHECK(res.stoppedAt == 2);
    CHECK(res.failedOn == 0);
    CHECK(res.firstFailedAt == 0);
    CHECK(std::string_view{res.firstFailedName}.empty());
    CHECK(res.firstFailedCause == RoutineStopCause::None);
    CHECK(res.jumped == 0);
    CHECK(std::string_view{res.skipPendingTo}.empty());
    Tank u;
    Routine clean{u.ch, "clean"};
    clean.then([] { return true; }, "fine");
    CHECK(clean.ok());
    CHECK(clean.result().ok);
}

TEST_CASE("STEP1-A ST6: carry on goes on, counts the failure, and says so") {
    // Bug caught: the choice ignored (SM1: the chain stops), a carry-on that leaves ok() true (SM3), a
    // failure that fills the STOP fields (stoppedAt must stay 0: nothing stopped), a missing Warn.
    Tank t;
    Routine r{t.ch, "carry"};
    bool nextRan = false;
    r.then([] { return true; }, "first")
        .onFail(carryOn)
        .then([] { return false; }, "miss")
        .then([&] { nextRan = true; }, "next");
    const auto res = r.result();
    CHECK(nextRan);
    CHECK_FALSE(r.ok());
    CHECK_FALSE(res.ok);
    CHECK(res.steps == 3);
    CHECK(res.completed == 2);
    CHECK(res.stoppedAt == 0);
    CHECK(res.skipped == 0);
    CHECK(res.cause == RoutineStopCause::None);
    CHECK(res.failedOn == 1);
    CHECK(res.firstFailedAt == 2);
    CHECK(std::string_view{res.firstFailedName} == "miss");
    CHECK(res.firstFailedCause == RoutineStopCause::ActionFailed);
    CHECK(res.jumped == 0);
    CHECK(rtn(t, LogLevel::Warn, "routine 'carry': step 2 (miss) FAILED: action FAILED — carrying on; drive safed") == 1);
    CHECK(rtnAny(t, "STOPPED") == 0);
    // A second carried-on failure counts; the FIRST one's fields stay the first's.
    r.onFail(carryOn).waitFor([] { return false; }, 0.05_s, "second");
    CHECK(r.result().failedOn == 2);
    CHECK(r.result().firstFailedAt == 2);
    CHECK(r.result().firstFailedCause == RoutineStopCause::ActionFailed);
}

TEST_CASE("STEP1-A ST27: carry on SAFES the drive exactly as a stop does -- the moving robot is cancelled and "
          "rests") {
    // Bug caught: a carry-on that skips Chassis::cancel (SM22) — a Tier-3 motion left running under the
    // chain would keep driving the robot through the next steps, from a place the chain never meant.
    Tank t;
    shulib::motion::MoveToPose m{t.ch.deps(), Pose2d{Length{60.0}, Length{0.0}, Angle{}},
                                 motion_rig::motionConfig(), 5.0};
    t.ch.scheduler().async(m);
    (void)t.ch.waitUntil([] { return false; }, 0.4_s);  // the robot is moving now
    REQUIRE(t.ch.scheduler().hasActiveMotion());
    REQUIRE(t.rig.h.truePose().x().value() > 3.0);
    const int cancelledBefore = t.ch.scheduler().motionsCancelled();
    Routine r{t.ch, "safe"};
    r.onFail(carryOn).then([] { return false; }, "miss");
    CHECK(t.ch.scheduler().motionsCancelled() == cancelledBefore + 1);
    CHECK_FALSE(t.ch.scheduler().hasActiveMotion());
    for (shulib::hal::IMotor* motor : t.rig.h.context().driveMotors()) {
        CHECK(motor->commandedVoltage().value() == 0.0);
        CHECK(motor->brakeMode() == shulib::hal::BrakeMode::Brake);
    }
    // The next step starts from rest: through a 0.3 s pause the robot does not move.
    const Pose2d before = t.rig.h.truePose();
    r.pause(0.3_s);
    CHECK(motion_rig::posErr(t.rig.h.truePose(), before) < 1e-6);
    CHECK(r.result().completed == 1);
}

TEST_CASE("STEP1-A ST4: a skip passes over exactly the steps between, and resumes AT the label") {
    // Bug caught: a skip that runs a step it should pass (issuing a drive), lands one step late or
    // early, or counts the jumped steps wrong; SM1 (the choice ignored).
    Tank t;
    Routine r{t.ch, "st4"};
    bool ran[8]{};
    const int motions0 = t.ch.scheduler().motionsStarted();
    r.then([&] { ran[1] = true; }, "one")
        .then([&] { ran[2] = true; }, "two")
        .onFail(skipTo("B"))
        .then([&] { ran[3] = true; return false; }, "three")
        .then([&] { ran[4] = true; }, "four")
        .driveTo(10_in, 0_in)  // five: a drive the skip must not issue
        .label("B")
        .then([&] { ran[6] = true; }, "six")
        .then([&] { ran[7] = true; }, "seven");
    const auto res = r.result();
    CHECK(ran[1]);
    CHECK(ran[2]);
    CHECK(ran[3]);
    CHECK_FALSE(ran[4]);
    CHECK(ran[6]);
    CHECK(ran[7]);
    CHECK(t.ch.scheduler().motionsStarted() == motions0);  // the jumped drive was never issued
    CHECK(res.steps == 7);
    CHECK(res.completed == 4);
    CHECK(res.jumped == 2);
    CHECK(res.failedOn == 1);
    CHECK(res.firstFailedAt == 3);
    CHECK(res.stoppedAt == 0);
    CHECK(res.skipped == 0);
    CHECK(std::string_view{res.skipPendingTo}.empty());
    CHECK_FALSE(res.ok);
    CHECK(rtn(t, LogLevel::Warn, "routine 'st4': step 3 (three) FAILED: action FAILED — skipping to 'B'; drive safed") == 1);
    CHECK(rtn(t, LogLevel::Info, "routine 'st4': step 4 (four) jumped — skipping to 'B'") == 1);
    CHECK(rtn(t, LogLevel::Info, "routine 'st4': step 5 (driveTo) jumped — skipping to 'B'") == 1);
    CHECK(rtn(t, LogLevel::Info, "jumped") == 2);
    CHECK(rtn(t, LogLevel::Info, "routine 'st4': the skip to 'B' landed at step 6 (six)") == 1);
}

TEST_CASE("STEP1-A ST5: a skip that never lands jumps every remaining step -- nothing is commanded") {
    // Bug caught: a misspelled target that lets the chain resume anyway, or issues a motion after the
    // failure; skipPendingTo empty when the skip never landed.
    Tank t;
    Routine r{t.ch, "st5"};
    bool after = false;
    r.onFail(skipTo("B")).then([] { return false; }, "fails");
    const int motions0 = t.ch.scheduler().motionsStarted();
    const double clock0 = t.rig.h.clock().now().value();
    r.driveTo(10_in, 0_in).pause(0.5_s).label("b").then([&] { after = true; }, "lower-case b").turnTo(Angle::degrees(90.0));
    const auto res = r.result();
    CHECK_FALSE(after);
    CHECK(t.ch.scheduler().motionsStarted() == motions0);
    CHECK(t.rig.h.clock().now().value() == clock0);  // not even the pause ran
    CHECK(std::string_view{res.skipPendingTo} == "B");
    CHECK(res.jumped == 4);
    CHECK(res.steps == 5);
    CHECK(res.stoppedAt == 0);
    CHECK(res.failedOn == 1);
    CHECK_FALSE(res.ok);
}

TEST_CASE("STEP1-A ST28: labels match WHOLE and EXACTLY -- 'B2', 'b' and 'B ' are not 'B'") {
    // Bug caught: a prefix match (SM2: "B2" lands a skip to "B"), a case-insensitive one, a trimmed one.
    Tank t;
    Routine r{t.ch, "st28"};
    bool landed = false;
    r.onFail(skipTo("B"))
        .then([] { return false; }, "fails")
        .label("B2")
        .then([&] { landed = true; }, "B2")
        .label("b")
        .then([&] { landed = true; }, "b")
        .label("B ")
        .then([&] { landed = true; }, "B-space");
    CHECK_FALSE(landed);
    CHECK(r.result().jumped == 3);
    CHECK(std::string_view{r.result().skipPendingTo} == "B");
    // ... and the exact one does land.
    r.label("B").then([&] { landed = true; }, "B");
    CHECK(landed);
    CHECK(std::string_view{r.result().skipPendingTo}.empty());
}

TEST_CASE("STEP1-A ST24: a skip target BEHIND stops the chain at once -- and is said where it is written") {
    // Bug caught: a behind target treated as one that never comes (SM19: the chain would jump to the
    // end instead of stopping, "never landed" for a name plainly written above); the Warn at the
    // onFail line missing (SM33) — on a bench run whose step SUCCEEDS, that Warn is the only sign.
    SUBCASE("the target is an earlier step's label") {
        Tank t;
        Routine r{t.ch, "behind"};
        r.then([] { return true; }, "one").label("A").then([] { return true; }, "two");
        r.onFail(skipTo("A"));
        CHECK(rtn(t, LogLevel::Warn, "routine 'behind': step 3: its skip target 'A' is behind it: a skip only goes "
                                     "forward") == 1);
        r.then([] { return false; }, "three").then([] { return true; }, "four");
        const auto res = r.result();
        CHECK(res.stoppedAt == 3);
        CHECK(res.cause == RoutineStopCause::ActionFailed);
        CHECK(res.skipped == 1);
        CHECK(res.failedOn == 0);
        CHECK(res.jumped == 0);
        CHECK(std::string_view{res.skipPendingTo}.empty());
        CHECK(rtn(t, LogLevel::Warn, "routine 'behind' STOPPED at step 3 (three): action FAILED — its skip target 'A' "
                                     "is behind it: a skip only goes forward — skipping the rest; drive safed") == 1);
    }
    SUBCASE("the target is the failing step's OWN label -- label first, then onFail") {
        Tank t;
        Routine r{t.ch, "own"};
        r.label("X").onFail(skipTo("X"));
        CHECK(rtn(t, LogLevel::Warn, "its skip target 'X' is behind it") == 1);
        r.then([] { return false; }, "self");
        CHECK(r.result().stoppedAt == 1);
        CHECK(rtn(t, LogLevel::Warn, "STOPPED at step 1 (self): action FAILED — its skip target 'X' is behind it") == 1);
    }
    SUBCASE("the target is the failing step's OWN label -- onFail first, then label") {
        Tank t;
        Routine r{t.ch, "own2"};
        r.onFail(skipTo("X"));
        CHECK(rtn(t, LogLevel::Warn, "behind") == 0);
        r.label("X");
        CHECK(rtn(t, LogLevel::Warn, "its skip target 'X' is behind it") == 1);
        r.then([] { return false; }, "self");
        CHECK(r.result().stoppedAt == 1);
    }
    SUBCASE("the Warn on a run where the step SUCCEEDS -- nothing else changes") {
        Tank t;
        Routine r{t.ch, "bench"};
        r.label("A").then([] { return true; }, "one").onFail(skipTo("A")).then([] { return true; }, "two");
        CHECK(rtn(t, LogLevel::Warn, "routine 'bench': step 2: its skip target 'A' is behind it") == 1);
        CHECK(r.ok());
        CHECK(r.result().steps == 2);
        CHECK(r.result().completed == 2);
        CHECK(r.result().failedOn == 0);
    }
    SUBCASE("the label memory is the last 32 -- two back and 32 back are BEHIND, 33 back is forgotten") {
        // Bug caught: review RM1 — the memory cut below 32 (a target two labels back would read as one that
        // never comes, and the skip would pass over the rest of the auton); the 32 pinned at its edge
        // both ways (31 fails the 32 case, 33 fails the 33 case).
        for (const int k : {2, 32, 33}) {
            CAPTURE(k);
            std::vector<std::string> names;
            for (int i = 1; i <= k; ++i) {
                names.push_back("L" + std::to_string(i));
            }
            Tank t;
            Routine r{t.ch, "memory"};
            for (const std::string& n : names) {
                r.label(n.c_str()).then([] { return true; }, "s");
            }
            r.onFail(skipTo("L1")).then([] { return false; }, "miss").then([] { return true; }, "after");
            const auto res = r.result();
            if (k <= 32) {
                CHECK(res.stoppedAt == k + 1);
                CHECK(res.failedOn == 0);
                CHECK(res.jumped == 0);
                CHECK(rtn(t, LogLevel::Warn, "its skip target 'L1' is behind it") == 2);  // where written + the stop
            } else {
                CHECK(res.stoppedAt == 0);  // forgotten: the same as a target that never comes (D3)
                CHECK(res.failedOn == 1);
                CHECK(res.jumped == 1);
                CHECK(std::string_view{res.skipPendingTo} == "L1");
                CHECK(rtnAny(t, "behind") == 0);
            }
        }
    }
    SUBCASE("a label on a step a skip JUMPED counts as passed -- a later skip back to it is behind") {
        // Bug caught: review RM2 — remembering labels only while no skip is pending: "A" (jumped) would
        // read as ahead, and the second skip would pass over every remaining step instead of stopping.
        Tank t;
        Routine r{t.ch, "jumped-label"};
        r.onFail(skipTo("B")).then([] { return false; }, "one");
        r.label("A").then([] { return true; }, "two");  // jumped — its label is still passed
        r.label("B").then([] { return true; }, "three");  // the skip lands
        r.onFail(skipTo("A"));
        CHECK(rtn(t, LogLevel::Warn, "routine 'jumped-label': step 4: its skip target 'A' is behind it") == 1);
        r.then([] { return false; }, "four").then([] { return true; }, "five");
        const auto res = r.result();
        CHECK(res.stoppedAt == 4);
        CHECK(res.failedOn == 1);
        CHECK(res.jumped == 1);
        CHECK(res.skipped == 1);
        CHECK(std::string_view{res.skipPendingTo}.empty());
    }
    SUBCASE("a FORWARD target logs no Warn") {
        Tank t;
        Routine r{t.ch, "forward"};
        r.label("A").then([] { return true; }, "one").onFail(skipTo("B")).then([] { return true; }, "two");
        r.label("B").then([] { return true; }, "three");
        CHECK(rtnAny(t, "behind") == 0);
        CHECK(r.ok());
    }
}

TEST_CASE("STEP1-A ST26: a modifier binds to ONE step, and is used up at its door whatever happens there") {
    // Bug caught: an onFail that sticks (the second failing step carried on too), a modifier NOT used
    // up by a step that is skipped or jumped (SM20: it leaks onto a later step, or the next onFail
    // throws "two in a row"), two in a row accepted, a bad name accepted, a refusal that counts.
    SUBCASE("onFail(carryOn), then TWO failing steps: the second stops the chain") {
        Tank t;
        Routine r{t.ch, "one-step"};
        r.onFail(carryOn).then([] { return false; }, "a").then([] { return false; }, "b").then([] { return true; }, "c");
        CHECK(r.result().failedOn == 1);
        CHECK(r.result().stoppedAt == 2);
        CHECK(r.result().skipped == 1);
    }
    SUBCASE("a modifier on a step SKIPPED after a stop is used up") {
        Tank t;
        Routine r{t.ch, "skipped"};
        r.then([] { return false; }, "stops");
        r.onFail(carryOn).label("L").then([] { return true; }, "skipped");
        CHECK_NOTHROW(r.onFail(carryOn));
        CHECK_NOTHROW(r.label("M"));
        CHECK(r.result().skipped == 1);
    }
    SUBCASE("a modifier on a JUMPED step is used up -- it never reaches the step the skip lands on") {
        Tank t;
        Routine r{t.ch, "jumped"};
        r.onFail(skipTo("L"))
            .then([] { return false; }, "fails")
            .onFail(carryOn)
            .then([] { return true; }, "jumped")
            .label("L")
            .then([] { return false; }, "lands");
        CHECK(r.result().jumped == 1);
        CHECK(r.result().stoppedAt == 3);  // "lands" had NO choice of its own: the default stops
        CHECK(r.result().failedOn == 1);
    }
    SUBCASE("a modifier on a step that THROWS a precondition is used up, and nothing is counted") {
        Tank t;
        Routine r{t.ch, "throws"};
        r.onFail(carryOn).label("T");
        CHECK_THROWS_AS(r.driveTo(0_in, 0_in), shulib::PreconditionError);  // to where the robot is
        CHECK(r.result().steps == 0);
        CHECK_NOTHROW(r.onFail(carryOn));
        CHECK_NOTHROW(r.label("U"));
    }
    SUBCASE("two in a row, and bad names, are refused -- the counters untouched") {
        Tank t;
        Routine r{t.ch, "refused"};
        r.then([] { return true; }, "one");
        r.onFail(carryOn);
        CHECK_THROWS_AS(r.onFail(stopChain), shulib::PreconditionError);
        r.label("A");
        CHECK_THROWS_AS(r.label("B"), shulib::PreconditionError);
        CHECK_THROWS_AS((void)skipTo(nullptr), shulib::PreconditionError);
        CHECK_THROWS_AS((void)skipTo(""), shulib::PreconditionError);
        Tank u;
        Routine s{u.ch, "names"};
        CHECK_THROWS_AS(s.label(nullptr), shulib::PreconditionError);
        CHECK_THROWS_AS(s.label(""), shulib::PreconditionError);
        CHECK_THROWS_AS(s.onFail(OnFail{OnFail::Kind::SkipTo, nullptr}), shulib::PreconditionError);
        CHECK_THROWS_AS(s.onFail(OnFail{OnFail::Kind::SkipTo, ""}), shulib::PreconditionError);
        CHECK(r.result().steps == 1);
        CHECK(r.result().completed == 1);
        CHECK(s.result().steps == 0);
        CHECK(rtnAny(u, "no run guard") == 0);  // a refused onFail logged nothing
    }
}

TEST_CASE("STEP1-A ST16: a then() action that returns CANCELLED stops the chain whatever its choice; false carries "
          "on") {
    // Bug caught: SM34 — a then() returning ExitReason::Cancelled or MechanismOutcome::Cancelled
    // following the step's carryOn / skipTo (a cancel is the deadline, an abandon or the fault
    // policy talking: the chain must stop); SM9 seen without any guard — this Routine has none, so
    // only the CANCELLED rule can stop it.
    for (const bool skip : {false, true}) {
        CAPTURE(skip);
        const OnFail choice = skip ? skipTo("later") : carryOn;
        {
            Tank t;
            Routine r{t.ch, "exit-cancelled"};
            r.onFail(choice).then([] { return ExitReason::Cancelled; }, "exit").then([] { return true; }, "after");
            CHECK(r.result().stoppedAt == 1);
            CHECK(r.result().failedOn == 0);
            CHECK(r.result().skipped == 1);
            CHECK(rtn(t, LogLevel::Warn, "— the step was CANCELLED, so its on-failure choice is ignored") == 1);
        }
        {
            Tank t;
            Routine r{t.ch, "outcome-cancelled"};
            r.onFail(choice).then([] { return MechanismOutcome::Cancelled; }, "op").then([] { return true; }, "after");
            CHECK(r.result().stoppedAt == 1);
            CHECK(r.result().cause == RoutineStopCause::MechanismFailed);
            CHECK(r.result().mechanismVerdict == MechanismOutcome::Running);  // F1's then(): never a step verdict
            CHECK(r.result().failedOn == 0);
        }
        {
            Tank t;
            Routine r{t.ch, "false-goes-on"};
            r.onFail(choice).then([] { return false; }, "no").label("later").then([] { return true; }, "after");
            CHECK(r.result().stoppedAt == 0);
            CHECK(r.result().failedOn == 1);
            CHECK(r.result().completed == 1);
        }
        {
            Tank t;
            Routine r{t.ch, "timeout-goes-on"};
            r.onFail(choice).then([] { return ExitReason::TimedOut; }, "slow").label("later").then([] { return true; },
                                                                                                    "after");
            CHECK(r.result().stoppedAt == 0);
            CHECK(r.result().failedOn == 1);
        }
    }
}

TEST_CASE("STEP1-A ST25: a guard-less Routine Warns ONCE at its first carryOn / skipTo; a guarded one never") {
    // Bug caught: SM35 — the Warn removed (a carry-on chain built as Routine r{chassis} has no deadline
    // rule beyond CANCELLED, and the bench must say so); a Warn per call; a Warn for stopChain; a Warn
    // for a Routine that HAS a guard.
    Tank t;
    Routine r{t.ch, "no-guard"};
    r.onFail(stopChain).then([] { return true; }, "one");
    CHECK(rtnAny(t, "no run guard") == 0);
    r.onFail(carryOn).then([] { return true; }, "two");
    CHECK(rtn(t, LogLevel::Warn, "routine 'no-guard': on-failure choices with no run guard — the deadline cannot stop "
                                 "this routine; build it as Routine r{ctx, \"no-guard\"} in an auton") == 1);
    r.onFail(skipTo("x")).then([] { return true; }, "three");
    r.onFail(carryOn).then([] { return true; }, "four");
    CHECK(rtnAny(t, "no run guard") == 1);
    // A context with a guard (an AutonContext; its guard idle here) never Warns.
    auton_rig::RobotX x;
    const shulib::competition::AutonEntry e = auton_rig::entryFor("guarded", nullptr);
    shulib::competition::AutonContext ctx{x.chassis, x.guard, x.sink, x.svc.mechanisms(), e, &x.svc};
    Routine g{ctx, "guarded"};
    g.onFail(carryOn).then([] { return false; }, "miss").onFail(skipTo("y")).then([] { return false; }, "miss2");
    CHECK(x.count("RTN", "no run guard") == 0);
    CHECK(g.result().failedOn == 2);
    // A context WITHOUT a guard member is guard-less: it Warns.
    struct NoGuardCtx {
        shulib::chassis::Chassis& chassis;
        shulib::mechanism::MechanismService* mechanismService;
    } ng{x.chassis, &x.svc};
    Routine n{ng, "two-member"};
    n.onFail(carryOn);
    CHECK(x.count("RTN", "routine 'two-member': on-failure choices with no run guard") == 1);
}

TEST_CASE("STEP1-A ST25: a chassis-only Routine JUMPING a mechanism step never reaches for a service") {
    // Bug caught: the A6 mark looked up through mechs() on a Routine with no service — a throw out of
    // the auton for a step that was only being passed over (the brief's L8).
    using namespace mech_fixture::robot_x;
    Tank t;
    Routine r{t.ch, "chassis-only"};
    CHECK_NOTHROW(r.onFail(skipTo("L"))
                      .then([] { return false; }, "fails")
                      .move(arm["score"], 1_s)
                      .startRun(intake, 12_volt, 1_s)
                      .set(hood["up"])
                      .stop(intake)
                      .label("L")
                      .then([] { return true; }, "lands"));
    CHECK(r.result().jumped == 4);
    CHECK(r.result().completed == 1);
}
