// STEP1-B — PARTWAY TRIGGERS (chassis/routine.hpp's "STEP1-B: partway triggers"; the STEP1 brief's §1.2,
// B1–B9, D6–D8), on the MECH-A4 rig: robot x on the host test plant, its pacer chain the program's —
// Chassis -> RunGuard -> MechanismService -> World — so a trigger's fire inside the drive's tick commands
// a slot the service then ticks while the robot drives. Say "host test plant", never "simulator"; every
// number is the plant's (measured on it: the 40 in leg is 180 Running ticks, settles at 1.80 s 0.24 in
// SHORT — 99.39 % — and the 90 deg turn settles at 99.16 %).
//
// Each case attacks ONE rule (the brief's §2.2 numbers) and names the bug it would catch. Where a test
// claims an EQUALITY edge it lands on it exactly: a mark set to the progress the plant reached on a
// given tick (the leg is bit-identical run to run — ST19), read from a first, Tier-3 run of the same leg.
// Nothing here has run on a robot (A4: HA-217, HA-218).

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/motion/leg_progress.hpp"

using auton_rig::RobotTwo;
using auton_rig::RobotX;
using shulib::chassis::carryOn;
using shulib::chassis::LegObserver;
using shulib::chassis::RoutineStopCause;
using shulib::chassis::skipTo;
using shulib::control::ExitReason;
using shulib::manipulation::MechanismOutcome;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::LegProgress;
using shulib::units::Length;
using shulib::units::Time;
using namespace mech_fixture::robot_x;  // arm, lift, flywheel, intake, hood — and the words

namespace {

constexpr double kPi = 3.14159265358979323846;
/// Robot x's slots, in its list's order.
constexpr std::size_t kArm = 0, kLift = 1, kFly = 2, kIntake = 3;

/// The two members the Routine's context constructor reads (no guard: a test needs no runAuton).
struct Ctx {
    shulib::chassis::Chassis& chassis;
    shulib::mechanism::MechanismService* mechanismService;
};

/// One pace's view, taken FIRST in the world's pace — after the tick that just ran, so the pose is
/// the one that tick used, and each slot's step count says whether that tick fired a trigger.
struct Sample {
    double t = 0.0;
    Pose2d pose{};
    std::uint32_t begun[4]{};
    double liftIn = 0.0;
};

/// Samples every pace of `x` into `out`.
void sampleInto(RobotX& x, std::vector<Sample>& out) {
    x.world.onPace = [&x, &out] {
        Sample s;
        s.t = x.now();
        s.pose = x.chassis.pose();
        for (std::size_t i = 0; i < 4; ++i) {
            s.begun[i] = x.svc.slot(i).stepsBegun();
        }
        s.liftIn = x.liftInches();
        out.push_back(s);
    };
}

/// A Tier-3 run of a leg on a fresh robot x — the progress of every tick that did not end it, and
/// the clock of each. The routine's leg is bit-identical to it (ST19), so its numbers are the
/// routine leg's.
struct Seen {
    std::vector<LegProgress> p;
    std::vector<double> t;
    ExitReason exit = ExitReason::Running;
};
struct SeenSpy {
    Seen* seen = nullptr;
    RobotX* x = nullptr;
    static void call(void* self, const LegProgress& p) {
        SeenSpy& s = *static_cast<SeenSpy*>(self);
        s.seen->p.push_back(p);
        s.seen->t.push_back(s.x->now());
    }
};
Seen tier3Drive(const Pose2d& target, const shulib::chassis::MotionOptions& o = {}) {
    RobotX x;
    Seen seen;
    SeenSpy spy{&seen, &x};
    seen.exit = x.chassis.moveTo(target, o, LegObserver{&SeenSpy::call, &spy});
    return seen;
}

/// A Tier-3 run of a TURN on a fresh robot x (ST9 (edge) and ST22's degrees cases).
Seen tier3Turn(Angle heading) {
    RobotX x;
    Seen seen;
    SeenSpy spy{&seen, &x};
    seen.exit = x.chassis.turnTo(heading, {}, LegObserver{&SeenSpy::call, &spy});
    return seen;
}

const Pose2d kForty{Length{40.0}, Length{0.0}, Angle{}};

/// The index of the first sample whose begun[slot] rose above `before`; -1 when none did.
int firstFire(const std::vector<Sample>& s, std::size_t slot, std::uint32_t before = 0) {
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i].begun[slot] > before) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

/// How many RTN lines contain `needle`.
int rtn(const RobotX& x, const std::string& needle) { return x.count("RTN", needle); }

/// The message a call fails its precondition with, or "" when it does not.
template <class F>
std::string preconditionOf(F&& f) {
    try {
        f();
    } catch (const shulib::PreconditionError& e) {
        return e.what();
    }
    return {};
}

}  // namespace

TEST_CASE("STEP1-B ST9 + ST13: fires ONCE, on the tick the mark is reached -- and the lift moves while the robot "
          "drives") {
    // Bug caught: SM5 `>` for `>=` (the equality case below), a fire a tick late (progress read from the
    // pose before), SM7 a trigger that fires twice (every tick past the mark, or again at the settle),
    // and a mechanism that waits for the drive to end before it moves (ST13: the point of a trigger).
    RobotX x;
    std::vector<Sample> s;
    sampleInto(x, s);
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "st9"};
    REQUIRE(x.chassis.pose().x().value() == 0.0);
    const double lift0 = x.liftInches();
    r.atPercent(60).startMove(lift["high"], 6_s).driveTo(40_in, 0_in);
    const double legEnd = x.chassis.lastCompleted().endTime.value();
    REQUIRE(x.chassis.lastExitReason() == ExitReason::Settled);
    // The first sample whose progress (D7 (a): along the line from the live start (0, 0) to (40, 0))
    // reached 0.60 is the tick that fired, and nothing fired before it.
    int reached = -1;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const LegProgress p = shulib::motion::legProgressAlong(Pose2d{}, kForty, s[i].pose);
        if (p.fraction >= 0.60) {
            reached = static_cast<int>(i);
            break;
        }
    }
    REQUIRE(reached > 10);
    CHECK(firstFire(s, kLift) == reached);
    CHECK(x.svc.slot(kLift).stepsBegun() == 1);  // once: never again past the mark, never at the settle
    // ST13: the service ran the lift WHILE the robot drove — it rose before the leg ended.
    int risenAt = -1;
    for (std::size_t i = 0; i < s.size(); ++i) {
        if (s[i].liftIn > lift0 + 1.0) {
            risenAt = static_cast<int>(i);
            break;
        }
    }
    REQUIRE(risenAt >= 0);
    CHECK(s[static_cast<std::size_t>(risenAt)].t < legEnd);
    // The drive is ONE step; a succeeding trigger counts nothing (D7 (a)).
    CHECK(r.ok());
    CHECK(r.result().steps == 1);
    CHECK(r.result().completed == 1);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Running);  // the fired start's report
    r.waitUntilDone(lift, 6_s);  // collects the trigger's step, like any start form's
    CHECK(r.ok());
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
    CHECK(std::abs(x.liftInches() - 22.0) < 0.5);
}

TEST_CASE("STEP1-B ST9 (edge): a mark EXACTLY equal to a tick's progress fires ON that tick (>=), percent, "
          "inches and degrees") {
    // Bug caught: SM5 — `>` for `>=`: the fire moves one tick later. The marks are set to the progress
    // the plant reached on tick k exactly (a Tier-3 run of the same leg; ST19 makes them bit-identical).
    const Seen seen = tier3Drive(kForty);
    REQUIRE(seen.exit == ExitReason::Settled);
    // tick k: a mid-leg tick whose `along` the trigger can be set to exactly, and whose fraction a
    // percent round-trips to exactly (p * 100 / 100 == p).
    std::size_t k = 0;
    for (std::size_t i = 40; i < seen.p.size(); ++i) {
        const double f = seen.p[i].fraction;
        if (seen.p[i].live && (f * 100.0) / 100.0 == f && seen.p[i].along.value() > seen.p[i - 1].along.value()) {
            k = i;
            break;
        }
    }
    REQUIRE(k > 0);
    for (const bool percent : {true, false}) {
        CAPTURE(percent);
        RobotX x;
        std::vector<Sample> s;
        sampleInto(x, s);
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "edge"};
        if (percent) {
            r.atPercent(seen.p[k].fraction * 100.0).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
        } else {
            r.afterDistance(seen.p[k].along).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
        }
        // The observer's call i is pace i (one Running tick per pace), so the fire is seen at sample k.
        REQUIRE(s.size() == seen.p.size());
        CHECK(firstFire(s, kIntake) == static_cast<int>(k));
        CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
    }
    // DEGREES (review, fix round): the turn's `>=` — a mark set to the angle turned on tick k exactly
    // (one an Angle stores unchanged) fires on tick k, not k + 1.
    const Seen turn = tier3Turn(Angle::degrees(90.0));
    REQUIRE(turn.exit == ExitReason::Settled);
    std::size_t kt = 0;
    for (std::size_t i = 10; i < turn.p.size(); ++i) {
        const double v = turn.p[i].turnedRad;
        if (turn.p[i].live && v > 0.3 && Angle::radians(v).radians() == v && v > turn.p[i - 1].turnedRad) {
            kt = i;
            break;
        }
    }
    REQUIRE(kt > 0);
    RobotX x;
    std::vector<Sample> s;
    sampleInto(x, s);
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "edge-deg"};
    r.afterTurning(Angle::radians(turn.p[kt].turnedRad)).startRun(intake, 12_volt, 0.5_s).turnTo(90_deg);
    REQUIRE(s.size() == turn.p.size());
    CHECK(firstFire(s, kIntake) == static_cast<int>(kt));
    CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
}

TEST_CASE("STEP1-B ST10 + ST11 + ST20: a leg that ends EARLY fires nothing -- timed out at ~58 %, cut -- and the "
          "later wait is a counted failure") {
    // Bug caught: SM6 a trigger fired on a timed-out leg (at the timeout, or by the settle rule applied
    // to any end); a trigger read before it was reached; an unfired trigger's wait that throws, or
    // answers SUCCEEDED (P1 (b)) — ST20: MechanismNotStarted, "the trigger never fired", no exception.
    const Seen probe = tier3Drive(kForty);
    std::size_t k58 = 0;
    while (k58 < probe.p.size() && probe.p[k58].fraction < 0.58) {
        ++k58;
    }
    REQUIRE(k58 < probe.p.size());
    const double timeout = probe.t[k58];  // the leg times out ON the tick it would first read 58 %
    RobotX x;
    std::vector<Sample> s;
    sampleInto(x, s);
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "near-miss"};
    // ... plus a mark PAST the leg (50 in on a 40 in drive): on a leg that did not arrive, B3's "did not
    // fire: the drive was" line is not said (the leg failed; its own line says why).
    r.onFail(carryOn)
        .atPercent(60)
        .startMove(lift["high"], 2_s)
        .afterDistance(50_in)
        .startMove(arm["score"], 2_s)
        .driveTo(40_in, 0_in, {.timeout = Time{timeout}});
    REQUIRE(x.chassis.lastExitReason() == ExitReason::TimedOut);
    CHECK(rtn(x, "did not fire") == 0);
    CHECK(x.svc.slot(kArm).stepsBegun() == 0);
    double furthest = 0.0;
    for (const Sample& e : s) {
        furthest = std::max(furthest, e.pose.x().value() / 40.0);
    }
    CHECK(furthest > 0.50);  // a NEAR miss: the leg got most of the way
    CHECK(furthest < 0.60);
    CHECK(x.svc.slot(kLift).stepsBegun() == 0);  // nothing commanded, on any tick, nor at the end
    CHECK(r.result().failedOn == 1);
    CHECK(r.result().firstFailedCause == RoutineStopCause::MotionFailed);
    CHECK(r.result().steps == 1);
    CHECK_NOTHROW(r.waitUntilDone(lift, 1_s));
    CHECK(r.result().cause == RoutineStopCause::MechanismNotStarted);
    CHECK(r.result().stoppedAt == 2);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::Running);
    CHECK(rtn(x, "STOPPED at step 2 (lift): mechanism NOT STARTED (the trigger never fired: the robot did not "
                 "reach the mark)") == 1);
    // ... and CUT mid-leg (an outside cancel between ticks, at ~10 in): the 20 % trigger fired before
    // it, the 60 % one never does.
    RobotX y;
    bool cut = false;
    y.world.onPace = [&y, &cut] {
        if (!cut && y.chassis.pose().x().value() > 10.0) {
            cut = true;
            y.chassis.cancel();
        }
    };
    Ctx cy{y.chassis, &y.svc};
    Routine c{cy, "cut"};
    c.atPercent(20).startRun(intake, 12_volt, 0.5_s).atPercent(60).startMove(lift["high"], 2_s).driveTo(40_in, 0_in);
    REQUIRE(cut);
    CHECK(y.chassis.lastExitReason() == ExitReason::Cancelled);
    CHECK(y.svc.slot(kIntake).stepsBegun() == 1);
    CHECK(y.svc.slot(kLift).stepsBegun() == 0);
    CHECK(c.result().stoppedAt == 1);
    CHECK(c.result().exit == ExitReason::Cancelled);
}

TEST_CASE("STEP1-B ST31: ONCE through an overshoot -- the 100 % mark crossed inside the leg is not fired again "
          "at its settle") {
    // Bug caught: SM7 — the settle rule (B3) firing a trigger that already fired in a tick: the motor-lag
    // plant (kA 0.02, P4's) carries the 40 in leg PAST 100 % (to 100.53 % — measured; the estimate does
    // not fall back below 100 % before the leg settles), so a 100 % mark fires in a tick and the leg then
    // SETTLES. The intake must be commanded once.
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    auto cfg = motion_rig::plantConfig();
    cfg.plant.wheelFf.kA = 0.02;
    motion_rig::MotionRig rig{kin, cfg, &sink};
    mech_rig::Rig svcRig{kRobotXMechanisms};  // the service, outside the chassis's chain (commands only)
    struct Paced final : shulib::motion::ITickPacer {
        motion_rig::MotionRig* rig = nullptr;
        std::vector<double> frac;
        shulib::chassis::Chassis* ch = nullptr;
        shulib::mechanism::MechanismService* svc = nullptr;
        std::vector<std::uint32_t> begun;
        void pace() override {
            frac.push_back(ch->pose().x().value() / 40.0);
            begun.push_back(svc->slot(kIntake).stepsBegun());
            rig->h.plant().step(Time{0.01});
        }
    } w;
    w.rig = &rig;
    w.svc = &svcRig.svc;
    shulib::chassis::Chassis ch{rig.deps, w, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}};
    w.ch = &ch;
    Ctx ctx{ch, &svcRig.svc};
    Routine r{ctx, "overshoot"};
    r.atPercent(100).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
    REQUIRE(ch.lastExitReason() == ExitReason::Settled);
    std::size_t crossed = 0;
    while (crossed < w.frac.size() && w.frac[crossed] < 1.0) {
        ++crossed;
    }
    REQUIRE(crossed < w.frac.size());      // the leg went past its target...
    REQUIRE(crossed + 5 < w.frac.size());  // ... with ticks to spare before it settled
    CHECK(w.begun[crossed] == 1);          // fired on the crossing tick, inside the leg
    CHECK(w.begun[crossed - 1] == 0);
    CHECK(svcRig.svc.slot(kIntake).stepsBegun() == 1);  // and not again at the settle
    CHECK(r.ok());
}

TEST_CASE("STEP1-B ST12: by INCHES on a drive, by DEGREES on turnTo and on face") {
    // Bug caught: afterDistance read as a percent or from the wrong origin; afterTurning measuring the
    // drive's heading error or the long way; face() not reaching the observer overload.
    {
        RobotX x;
        std::vector<Sample> s;
        sampleInto(x, s);
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "inches"};
        r.afterDistance(20_in).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
        int reached = -1;
        for (std::size_t i = 0; i < s.size(); ++i) {
            if (shulib::motion::legProgressAlong(Pose2d{}, kForty, s[i].pose).along.value() >= 20.0) {
                reached = static_cast<int>(i);
                break;
            }
        }
        REQUIRE(reached > 0);
        CHECK(firstFire(s, kIntake) == reached);
        CHECK(r.ok());
    }
    for (const bool viaFace : {false, true}) {
        CAPTURE(viaFace);
        RobotX x;
        std::vector<Sample> s;
        sampleInto(x, s);
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "degrees"};
        if (viaFace) {
            r.afterTurning(45_deg).startSpinUp(flywheel["shoot"], 2_s).face(0_in, 40_in);  // a turn to 90
        } else {
            r.afterTurning(45_deg).startSpinUp(flywheel["shoot"], 2_s).turnTo(90_deg);
        }
        REQUIRE(x.chassis.lastExitReason() == ExitReason::Settled);
        CHECK(std::string{x.chassis.lastCompleted().name} == "TurnTo");
        int reached = -1;
        for (std::size_t i = 0; i < s.size(); ++i) {
            const LegProgress p = shulib::motion::turnProgress(Angle{}, Angle::degrees(90.0), s[i].pose.heading());
            if (p.turnedRad >= Angle::degrees(45.0).radians()) {
                reached = static_cast<int>(i);
                break;
            }
        }
        REQUIRE(reached > 0);
        CHECK(firstFire(s, kFly) == reached);
        CHECK(x.svc.slot(kFly).stepsBegun() == 1);
        CHECK(r.result().completed == 1);
    }
}

TEST_CASE("STEP1-B ST21: an unfired trigger's wait NEVER answers an older step's verdict") {
    // Bug caught: SM13 — the wait after an unfired trigger collecting the verdict of the step the routine
    // began on the same mechanism EARLIER (P1 (b): "done" in 0.000 s, though nothing was started).
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "older"};
    r.move(lift["low"], 3_s);  // SUCCEEDED: an older verdict on the lift
    REQUIRE(r.lastMechanism().outcome == MechanismOutcome::Succeeded);
    r.onFail(carryOn).afterDistance(30_in).startMove(lift["high"], 2_s).driveTo(40_in, 0_in, {.timeout = 0.3_s});
    REQUIRE(x.chassis.lastExitReason() == ExitReason::TimedOut);
    r.onFail(carryOn).waitUntilDone(lift, 1_s);
    CHECK(r.result().failedOn == 2);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::Running);  // the default report, not SUCCEEDED
    CHECK(rtn(x, "step 3 (lift) FAILED: mechanism NOT STARTED (the trigger never fired: the robot did not reach "
                 "the mark)") == 1);
    r.waitUntilDone(lift, 1_s);  // a second wait: the same answer (a wait never clears the mark)
    CHECK(r.result().cause == RoutineStopCause::MechanismNotStarted);
    CHECK(r.result().completed == 1);  // only the first move
}

TEST_CASE("STEP1-B ST22 + ST36: a leg that ARRIVES fires the marks ON it as it arrives -- and only those (D6 (i), "
          "narrowed)") {
    // Bug caught: SM15 the settle-short rule inverted (a 100 % mark on a leg that settles 0.24 in short
    // never fires — D15's second question answered "no"); SM31 the settle firing EVERY unfired trigger
    // ("after 50 in" on a 40 in drive starting the lift where the author never meant — L18). The bound
    // is the leg's length from its live start + the effective tolerance (0.5 in here), INCLUSIVE.
    const Seen probe = tier3Drive(kForty);
    REQUIRE(probe.exit == ExitReason::Settled);
    double furthest = 0.0;
    for (const LegProgress& p : probe.p) {
        furthest = std::max(furthest, p.fraction);
    }
    REQUIRE(furthest < 1.0);  // the leg never reached 100 % (or 40 in) inside a tick: a settle fire
    REQUIRE(probe.p.back().length.value() == 40.0);
    REQUIRE(probe.p.back().positionTolerance.value() == 0.5);
    {
        RobotX x;
        std::vector<Sample> s;
        sampleInto(x, s);
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "on-the-leg"};
        r.atPercent(100)
            .startRun(intake, 12_volt, 0.5_s)
            .afterDistance(40_in)
            .startMove(arm["score"], 2_s)
            .afterDistance(40.5_in)  // EXACTLY 40 + 0.5: on the leg (<=)
            .startSpinUp(flywheel["shoot"], 2_s)
            .afterDistance(Length{std::nextafter(40.5, 41.0)})  // one ulp past it: never
            .startMove(lift["high"], 2_s)
            .driveTo(40_in, 0_in);
        const double end = x.chassis.lastCompleted().endTime.value();
        CHECK(firstFire(s, kIntake) == -1);  // nothing fired inside a tick...
        CHECK(firstFire(s, kArm) == -1);
        CHECK(firstFire(s, kFly) == -1);
        CHECK(x.svc.slot(kIntake).stepsBegun() == 1);  // ... the three ON the leg fired as it arrived
        CHECK(x.svc.slot(kArm).stepsBegun() == 1);
        CHECK(x.svc.slot(kFly).stepsBegun() == 1);
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);  // the one past it never
        CHECK(x.now() == end);                       // at the arrival: no pace after the leg
        CHECK(rtn(x, "routine 'on-the-leg': step 1 (driveTo): trigger after 40.5 in on 'lift' did not fire: the "
                     "drive was 40.0 in") == 1);
        CHECK(r.ok());  // the drive succeeded; the lift's trigger simply never fired
        r.waitUntilDone(lift, 1_s);
        CHECK(r.result().cause == RoutineStopCause::MechanismNotStarted);
        CHECK(rtn(x, "(the trigger never fired: the robot did not reach the mark)") == 1);
    }
    {
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "past-the-leg"};
        r.afterDistance(40.4_in).startRun(intake, 12_volt, 0.5_s).afterDistance(50_in).startMove(lift["high"], 2_s)
            .afterDistance(41_in).startMove(arm["score"], 2_s).driveTo(40_in, 0_in);
        CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);
        CHECK(x.svc.slot(kArm).stepsBegun() == 0);
        CHECK(rtn(x, "trigger after 50.0 in on 'lift' did not fire: the drive was 40.0 in") == 1);
        CHECK(rtn(x, "trigger after 41.0 in on 'arm' did not fire: the drive was 40.0 in") == 1);
    }
    {
        // The 90 deg turn (settles at 99.16 %): 90 fires (<= 90 + 1.15), 120 never.
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "turn"};
        r.afterTurning(90_deg).startRun(intake, 12_volt, 0.5_s).afterTurning(120_deg).startMove(lift["high"], 2_s)
            .turnTo(90_deg);
        CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);
        CHECK(rtn(x, "trigger after 120.0 deg on 'lift' did not fire: the turn was 90.0 deg") == 1);
    }
    {
        // The turn's bound EXACTLY (review, fix round): size + tolerance, read from a Tier-3 run of the same
        // turn, is ON the leg (<=); one ulp past it is not.
        const Seen t = tier3Turn(Angle::degrees(90.0));
        REQUIRE(t.exit == ExitReason::Settled);
        const double bound = t.p.back().sizeRad + t.p.back().headingToleranceRad;
        REQUIRE(Angle::radians(bound).radians() == bound);
        const double past = std::nextafter(bound, 4.0);
        REQUIRE(Angle::radians(past).radians() == past);
        double turnedMost = 0.0;
        for (const LegProgress& p : t.p) {
            turnedMost = std::max(turnedMost, p.turnedRad);
        }
        REQUIRE(turnedMost < bound);  // never reached inside a tick: only the settle can fire it
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "turn-edge"};
        r.afterTurning(Angle::radians(bound)).startRun(intake, 12_volt, 0.5_s).afterTurning(Angle::radians(past))
            .startMove(lift["high"], 2_s).turnTo(90_deg);
        CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);
    }
}

TEST_CASE("STEP1-B ST36 + B5: a ZERO-size turn fires nothing during it; at its settle a percent fires, an angle "
          "only within its tolerance") {
    // Bug caught: a zero-size leg dividing by its size (inf >= every mark: everything fires on the first
    // tick); the settle bound special-cased for it (B5: size 0 + the tolerance, no special case).
    RobotX x;
    std::vector<Sample> s;
    sampleInto(x, s);
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "zero"};
    REQUIRE(x.chassis.pose().heading().radians() == 0.0);
    r.atPercent(50)
        .startRun(intake, 12_volt, 0.5_s)
        .afterTurning(0.5_deg)  // <= 0 + 1.15 deg: fires at the settle
        .startSpinUp(flywheel["shoot"], 2_s)
        .afterTurning(5_deg)  // never
        .startMove(lift["high"], 2_s)
        .turnTo(0_deg);
    REQUIRE(x.chassis.lastExitReason() == ExitReason::Settled);
    CHECK(firstFire(s, kIntake) == -1);
    CHECK(firstFire(s, kFly) == -1);
    CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
    CHECK(x.svc.slot(kFly).stepsBegun() == 1);
    CHECK(x.svc.slot(kLift).stepsBegun() == 0);
    CHECK(rtn(x, "trigger after 5.0 deg on 'lift' did not fire: the turn was 0.0 deg") == 1);
    // A zero-size MOVE (moveTo the pose it is at): the same rule, in inches.
    RobotX y;
    Ctx cy{y.chassis, &y.svc};
    Routine m{cy, "zero-move"};
    m.atPercent(100).startRun(intake, 12_volt, 0.5_s).afterDistance(0.4_in).startMove(arm["score"], 2_s)
        .afterDistance(0.6_in).startMove(lift["high"], 2_s).moveTo(Pose2d{});
    REQUIRE(y.chassis.lastExitReason() == ExitReason::Settled);
    CHECK(y.svc.slot(kIntake).stepsBegun() == 1);
    CHECK(y.svc.slot(kArm).stepsBegun() == 1);
    CHECK(y.svc.slot(kLift).stepsBegun() == 0);
}

namespace {
/// ST29's auton: the 40 in drive, its mark at `gMark` along it (or, for the probe, a Tier-3 drive whose
/// observer records each tick's progress and whether the guard's time was up on it).
Length gMark{0.0};
std::vector<LegProgress> gProbe;
std::vector<bool> gExpired;
shulib::sequence::RunGuard* gGuard = nullptr;
void probeObserver(void*, const LegProgress& p) {
    gProbe.push_back(p);
    gExpired.push_back(gGuard->running() && gGuard->expired());
}
RoutineResult probeAuton(shulib::competition::AutonContext& ctx) {
    gGuard = &ctx.guard;
    (void)ctx.chassis.moveTo(kForty, {}, LegObserver{&probeObserver, nullptr});
    Routine r{ctx, "probe"};
    return r.result();
}
RoutineResult markAuton(shulib::competition::AutonContext& ctx) {
    Routine r{ctx, "deadline"};
    r.afterDistance(gMark).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
    return r.result();
}
}  // namespace

TEST_CASE("STEP1-B ST29: NEVER on the deadline tick -- a mark reached exactly on it does not fire; one tick "
          "earlier it does") {
    // Bug caught: SM24 — the trigger trusting the drive instead of asking the guard. The motion TICKS once
    // at the deadline instant before the guard cuts it (the brief's probe P6): a trigger reached on that
    // tick would start a mechanism AFTER the buzzer. The deadline: the scoring phase ends at T+1.00 (the
    // park), mid-leg. The marks are the progress of the deadline tick and of the tick before it, exactly.
    gProbe.clear();
    gExpired.clear();
    {
        RobotX x;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                       auton_rig::entryFor("probe", &probeAuton, 1.0, 2.5), nullptr,
                                                       x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    }
    std::size_t d = 0;
    while (d < gExpired.size() && !gExpired[d]) {
        ++d;
    }
    REQUIRE(d < gExpired.size());  // the leg was observed ON the deadline tick (P6)...
    REQUIRE(d + 1 == gExpired.size());  // ... and that was its last observed tick (the guard cut it)
    REQUIRE(gProbe[d].along.value() > gProbe[d - 1].along.value());
    for (const bool onTheDeadline : {true, false}) {
        CAPTURE(onTheDeadline);
        gMark = onTheDeadline ? gProbe[d].along : gProbe[d - 1].along;
        RobotX x;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                       auton_rig::entryFor("deadline", &markAuton, 1.0, 2.5), nullptr,
                                                       x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
        CHECK(res.routine.exit == ExitReason::Cancelled);  // the guard cut the leg (no settle fire)
        CHECK(x.svc.slot(kIntake).stepsBegun() == (onTheDeadline ? 0u : 1u));
    }
}

TEST_CASE("STEP1-B ST33: a REFUSED start fails the drive step AFTER the leg -- and never touches the drive") {
    // Bug caught: SM28 — the fire reusing the start steps' recording (positionStep -> startedMechanism ->
    // recordStop -> Chassis::cancel(), refused inside a tick): the checker's X3, a CANCELLED leg at 12.5 in
    // with a PRECONDITION fault, or a throw out of the step; and a SUCCEEDING fire that counts a step. Each
    // run-time refusal: the leg completes SETTLED, no PRECONDITION fault, no exception, no cancel during
    // the leg; then MechanismStepFailed with the verdict, under the drive step's choice.
    struct Owner final : shulib::hal::ICancellable {
        void cancel() override {}
    };
    struct Case {
        const char* what;
        MechanismOutcome verdict;
        const char* mechLine;
    };
    const Case cases[] = {
        {"absent at boot", MechanismOutcome::Absent, "'intake': ABSENT in 0.00s (not built at boot: port 5: nothing"},
        {"not on this robot", MechanismOutcome::Absent,
         "'claw pivot 6' is not on this robot (its wiring list does not have it): the step did nothing"},
        {"refused", MechanismOutcome::Refused, "'intake': REFUSED in 0.00s (another owner holds its claim)"},
        {"not homed", MechanismOutcome::NotHomed, "'lift': NOT_HOMED"},
    };
    for (const Case& c : cases) {
        for (const bool goOn : {false, true}) {
            CAPTURE(c.what);
            CAPTURE(goOn);
            const std::string w = c.what;
            RobotX x{true, /*liftOnSwitchAtBoot=*/w != "not homed", [&w](shulib::mechanism::Devices (&d)[5]) {
                         if (w == "absent at boot") {
                             d[3] = shulib::mechanism::Devices{.absence = "port 5: nothing answered"};
                         }
                     }};
            Owner owner;
            if (w == "refused") {
                REQUIRE(x.mIntake.tryClaim(owner));
            }
            Ctx ctx{x.chassis, &x.svc};
            Routine r{ctx, "refused-start"};
            const int cancels0 = x.chassis.scheduler().motionsCancelled();
            const int precond0 = x.rig.latch.raiseCount(shulib::diag::FaultCode::Precondition);
            if (goOn) {
                r.onFail(carryOn);
            }
            CHECK_NOTHROW([&] {
                if (w == "not on this robot") {
                    r.atPercent(50).startRun(mech_fixture::robot_two::clawPivot6, 12_volt, 0.5_s).driveTo(40_in, 0_in);
                } else if (w == "not homed") {
                    r.atPercent(50).startMove(lift["high"], 2_s).driveTo(40_in, 0_in);
                } else {
                    r.atPercent(50).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
                }
            }());
            CHECK(x.chassis.lastExitReason() == ExitReason::Settled);  // the leg ran to its own verdict
            CHECK(x.chassis.lastCompleted().endTime.value() == doctest::Approx(1.80));
            CHECK(x.chassis.scheduler().motionsCancelled() == cancels0);
            CHECK(x.rig.latch.raiseCount(shulib::diag::FaultCode::Precondition) == precond0);
            CHECK(x.count("MECH", c.mechLine) == 1);
            CHECK(r.lastMechanism().outcome == c.verdict);
            if (goOn) {
                CHECK(r.result().failedOn == 1);
                CHECK(r.result().firstFailedCause == RoutineStopCause::MechanismStepFailed);
                CHECK(r.result().stoppedAt == 0);
                r.pause(0.1_s);  // the next step runs
                CHECK(r.result().completed == 1);
            } else {
                CHECK(r.result().stoppedAt == 1);
                CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
                CHECK(r.result().mechanismVerdict == c.verdict);
                CHECK(std::string{r.result().stoppedName} == "driveTo");
            }
            CHECK(r.result().steps == (goOn ? 2 : 1));  // the drive is ONE step, the trigger none
            CHECK(rtn(x, std::string{goOn ? "(driveTo) FAILED: mechanism step FAILED (" : "(driveTo): mechanism step FAILED ("} +
                             shulib::manipulation::mechanismOutcomeName(c.verdict) + ": its trigger at 50 % on '") == 1);
        }
    }
    // A SUCCEEDING trigger: steps and completed rise by exactly one — the drive's.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "ok"};
    r.pause(0.1_s);
    r.atPercent(50).startRun(intake, 12_volt, 0.5_s).atPercent(70).startSpinUp(flywheel["shoot"], 2_s)
        .driveTo(40_in, 0_in);
    CHECK(r.result().steps == 2);
    CHECK(r.result().completed == 2);
    CHECK(x.svc.slot(kIntake).stepsBegun() == 1);
    CHECK(x.svc.slot(kFly).stepsBegun() == 1);
}

TEST_CASE("STEP1-B ST33 (b): a failed LEG with a refused trigger -- the leg's own verdict, the trigger named") {
    // Bug caught: a refused trigger's verdict REPLACING the leg's own failure (the stop would say "the
    // mechanism" when the drive timed out), or the refusal vanishing from the line.
    RobotX x{true, false};  // the lift is not homed
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "both"};
    r.atPercent(10).startMove(lift["high"], 2_s).driveTo(40_in, 0_in, {.timeout = 0.5_s});
    CHECK(r.result().cause == RoutineStopCause::MotionFailed);
    CHECK(r.result().exit == ExitReason::TimedOut);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::NotHomed);
    CHECK(rtn(x, "STOPPED at step 1 (driveTo): motion TIMEOUT (its trigger at 10 % on 'lift' was NOT_HOMED)") == 1);
}

TEST_CASE("STEP1-B ST30: a trigger WRITTEN WRONG is refused AT ITS LINE, by the mechanism's own checks (D8 (a))") {
    // Bug caught: SM32 — the arm skipping the start's checks: the mistake would surface only on a run
    // whose drive REACHES the mark (a bench run that stops early never shows it), inside the drive's tick
    // (B7's backstop: thrown after a SETTLED leg) — or, with SM26 on top, as a fake CANCELLED drive with
    // a PRECONDITION fault. Each throws at the ARM with the service's own message, before any motion
    // (no record, the clock unmoved), the counters untouched, nothing armed after; the same arguments to
    // the blocking step throw the same message.
    {
        RobotTwo two;  // claw pivot 6: no piece sensor, no stall signature
        using mech_fixture::robot_two::clawPivot6;
        struct Ctx2 {
            shulib::chassis::Chassis& chassis;
            shulib::mechanism::MechanismService* mechanismService;
        } ctx{two.chassis, &two.svc};
        Routine r{ctx, "wrong"};
        const double t0 = two.rig.h.clock().now().value();
        const std::string sees =
            preconditionOf([&] { (void)r.atPercent(50).startRun(clawPivot6, 12_volt, seesPiece, 2_s); });
        CHECK(sees.find("declares NO piece sensor") != std::string::npos);
        const std::string stall = preconditionOf([&] { (void)r.atPercent(50).startRun(clawPivot6, 12_volt, stalls, 2_s); });
        CHECK(stall.find("no stall signature") != std::string::npos);
        CHECK(two.rig.h.clock().now().value() == t0);
        CHECK(two.chassis.scheduler().motionsStarted() == 0);
        CHECK(r.result().steps == 0);
        CHECK(two.svc.slot(0).stepsBegun() == 0);
        CHECK(preconditionOf([&] { r.run(clawPivot6, 12_volt, seesPiece, 2_s); }) == sees);
        CHECK(preconditionOf([&] { r.run(clawPivot6, 12_volt, stalls, 2_s); }) == stall);
        r.pause(0.05_s);  // nothing armed after: the next step is not refused
        CHECK(r.result().steps == 1);
        // The WHOLE line, drive included: it throws at the arm, so the drive never starts. (Under SM32 the
        // drive runs and the fire's error is held and thrown after a SETTLED leg — B7's backstop; with
        // SM26 on top, the scheduler's catch turns it into a CANCELLED leg and a PRECONDITION fault. The
        // captures below say which way a mutant went red.)
        const int started0 = two.chassis.scheduler().motionsStarted();
        const std::string whole =
            preconditionOf([&] { r.atPercent(50).startRun(clawPivot6, 12_volt, seesPiece, 2_s).driveTo(40_in, 0_in); });
        const ExitReason legExit = two.chassis.lastExitReason();
        const int preconditionFaults = two.rig.latch.raiseCount(shulib::diag::FaultCode::Precondition);
        CAPTURE(static_cast<int>(legExit));
        CAPTURE(preconditionFaults);
        CHECK(whole == sees);
        CHECK(two.chassis.scheduler().motionsStarted() == started0);  // no motion: refused at its line
        CHECK(preconditionFaults == 0);
        CHECK(r.result().steps == 1);
    }
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "wrong-x"};
    // orAfter longer than the bound; NaN volts; a CONSTANT target past the arm's limits (a goal no build
    // would make — written by hand); a constant speed past the flywheel's free speed.
    const shulib::mechanism::PositionGoal past{&arm, "arm", nullptr, 175.0, false, shulib::mechanism::Units::Degrees};
    const shulib::mechanism::SpeedGoal tooFast{&flywheel, "flywheel", nullptr, 4000.0, false};
    const std::string orAfter =
        preconditionOf([&] { (void)r.atPercent(50).startRun(intake, 12_volt, seesPiece.orAfter(3_s), 2_s); });
    const std::string nanVolts =
        preconditionOf([&] { (void)r.atPercent(50).startRun(intake, shulib::units::Voltage{NAN}, 1_s); });
    const std::string limit = preconditionOf([&] { (void)r.atPercent(50).startMove(past, 1_s); });
    const std::string speed = preconditionOf([&] { (void)r.atPercent(50).startSpinUp(tooFast, 1_s); });
    const std::string bound = preconditionOf([&] { (void)r.atPercent(50).startHome(lift, Time{0.0}); });
    CHECK(orAfter.find("orAfter's) must be shorter") != std::string::npos);
    CHECK(nanVolts.find("the volts must be a finite number") != std::string::npos);
    CHECK(limit.find("CONSTANT target outside the travel limits") != std::string::npos);
    CHECK(speed.find("CONSTANT speed past the free speed") != std::string::npos);
    CHECK(bound.find("r.home(...): the time limit must be finite and above 0") != std::string::npos);
    CHECK(x.chassis.scheduler().motionsStarted() == 0);
    CHECK(x.now() == 0.0);
    CHECK(r.result().steps == 0);
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(x.svc.slot(i).stepsBegun() == 0);
    }
    CHECK(preconditionOf([&] { r.run(intake, 12_volt, seesPiece.orAfter(3_s), 2_s); }) == orAfter);
    CHECK(preconditionOf([&] { r.run(intake, shulib::units::Voltage{NAN}, 1_s); }) == nanVolts);
    CHECK(preconditionOf([&] { r.move(past, 1_s); }) == limit);
    CHECK(preconditionOf([&] { r.spinUp(tooFast, 1_s); }) == speed);
    CHECK(preconditionOf([&] { r.home(lift, Time{0.0}); }) == bound);
    // startHome on a mechanism declared neverHomed (robot y's tilt; its service outside the chain — the
    // arm throws before anything moves, so nothing is paced).
    mech_rig::Rig yRig{mech_fixture::robot_y::kRobotYMechanisms};
    Ctx cy{x.chassis, &yRig.svc};
    Routine ry{cy, "never-homed"};
    const std::string never = preconditionOf([&] { (void)ry.atPercent(50).startHome(mech_fixture::robot_y::tilt, 1_s); });
    CHECK(never.find("declared neverHomed") != std::string::npos);
    CHECK(preconditionOf([&] { ry.home(mech_fixture::robot_y::tilt, 1_s); }) == never);
    CHECK(ry.result().steps == 0);
    r.driveTo(10_in, 0_in);  // nothing armed after any of them
    CHECK(r.ok());
    CHECK(x.svc.slot(kIntake).stepsBegun() == 0);
}

TEST_CASE("STEP1-B ST32: a JUMPED or STOPPED leg fires nothing -- and marks each trigger's mechanism for a later "
          "wait") {
    // Bug caught: a trigger armed on a leg the chain passes over left armed (the next leg fires it), or
    // fired anyway; a later wait on it throwing (no start ever ran) instead of the counted answer.
    {
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "jumped"};
        r.onFail(skipTo("later")).then([] { return false; }, "miss");
        r.atPercent(50).startMove(lift["high"], 2_s).driveTo(40_in, 0_in);
        CHECK(x.chassis.scheduler().motionsStarted() == 0);
        CHECK(r.result().jumped == 1);
        r.label("later").waitUntilDone(lift, 1_s);
        CHECK(r.result().cause == RoutineStopCause::MechanismNotStarted);
        CHECK(rtn(x, "mechanism NOT STARTED (the trigger never fired: its drive was skipped)") == 1);
        r.driveTo(10_in, 0_in);  // nothing left armed: the next leg is a plain one
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);
    }
    {
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "stopped"};
        r.then([] { return false; }, "miss");
        r.atPercent(50).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
        CHECK(r.result().skipped == 1);
        CHECK(x.chassis.scheduler().motionsStarted() == 0);
        CHECK(x.svc.slot(kIntake).stepsBegun() == 0);
    }
}

TEST_CASE("STEP1-B ST34: a mark OUT OF RANGE is refused at its arm; 100 % and 180 deg are accepted") {
    // Bug caught: a range check missing or off by one edge — "at 0 %" (the start step's job), a percent
    // past 100, a NaN that compares false with every progress (a trigger that silently never fires), a
    // distance of 0 (fires on the first tick), 270_deg (stored -90: never reached).
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "ranges"};
    for (const double p : {0.0, -5.0, 150.0, std::nan(""), std::nextafter(100.0, 101.0)}) {
        CAPTURE(p);
        CHECK(preconditionOf([&] { (void)r.atPercent(p); }).find("above 0 and at most 100") != std::string::npos);
    }
    for (const double d : {0.0, -3.0, std::numeric_limits<double>::infinity()}) {
        CAPTURE(d);
        CHECK(preconditionOf([&] { (void)r.afterDistance(Length{d}); }).find("finite and above 0") != std::string::npos);
    }
    CHECK(preconditionOf([&] { (void)r.afterTurning(0_deg); }).find("above 0") != std::string::npos);
    CHECK(preconditionOf([&] { (void)r.afterTurning(270_deg); }).find("270_deg is -90") != std::string::npos);
    CHECK(r.result().steps == 0);
    // The edges that are IN: exactly 100 %, exactly 180 deg (+pi after wrapping), the smallest percent.
    // (each on a Routine of its own: a mark written alone is PENDING — the next call on that Routine would
    // refuse it, and its destructor says it never fired)
    {
        Routine q{ctx, "in-1"};
        CHECK(preconditionOf([&] { (void)q.atPercent(100.0); }).empty());
    }
    {
        Routine q{ctx, "in-2"};
        CHECK(preconditionOf([&] { (void)q.atPercent(1e-9); }).empty());
    }
    {
        Routine q{ctx, "in-3"};
        CHECK(preconditionOf([&] { (void)q.afterTurning(180_deg); }).empty());
    }
    {
        Routine q{ctx, "in-4"};
        CHECK(preconditionOf([&] { (void)q.afterDistance(Length{1e-9}); }).empty());
    }
    r.afterTurning(180_deg).startRun(intake, 12_volt, 0.5_s).turnTo(180_deg, {.timeout = 4_s});
    CHECK(x.svc.slot(kIntake).stepsBegun() == 1);  // a 180 deg turn reaches its 180 deg mark (at worst at its settle)
}

TEST_CASE("STEP1-B ST35: the arming rules -- modifiers before the arm, nothing else while armed, a throw leaves "
          "nothing armed") {
    // Bug caught: SM38 — an arm's precondition leaving its triggers armed (the NEXT leg would fire them);
    // a step taken while a trigger is armed (its handle discarded: the trigger rides the wrong drive, or
    // none); onFail / label applied to the arm instead of passed to the leg.
    {
        // onFail and label BEFORE the arm pass through to the leg: it is the labelled, carried-on step.
        RobotX x;
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "pass-through"};
        r.onFail(carryOn).label("L").atPercent(60).startMove(lift["high"], 2_s).driveTo(40_in, 0_in,
                                                                                         {.timeout = 0.2_s});
        CHECK(r.result().failedOn == 1);
        CHECK(r.result().firstFailedAt == 1);
        CHECK(std::string{r.result().firstFailedName} == "driveTo");
        CHECK(r.result().stoppedAt == 0);
        r.onFail(skipTo("L")).pause(0.05_s);  // "L" is behind: the leg carried it
        CHECK(rtn(x, "its skip target 'L' is behind it") == 1);
    }
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "armed"};
    // Anything but the handle while armed: a precondition, and nothing armed after it.
    const auto armThen = [&](auto&& misuse) {
        auto h = r.atPercent(60).startRun(intake, 12_volt, 0.5_s);
        (void)h;
        const std::string m = preconditionOf(misuse);
        CHECK(m.find("a partway trigger is armed") != std::string::npos);
    };
    armThen([&] { r.onFail(carryOn); });
    armThen([&] { r.label("x"); });
    armThen([&] { r.pause(0.1_s); });
    armThen([&] { (void)r.atPercent(30); });
    armThen([&] { (void)r.afterDistance(5_in); });
    armThen([&] { (void)r.afterTurning(10_deg); });
    armThen([&] { r.driveTo(10_in, 0_in); });
    CHECK(r.result().steps == 0);
    CHECK(x.chassis.scheduler().motionsStarted() == 0);
    // An arm that throws (a range, B8, D8, the fifth trigger) and a leg that throws (driveTo the point the
    // robot is at): nothing armed, no modifier pending, the counters untouched.
    const auto clean = [&](auto&& bad) {
        r.onFail(carryOn).label("y");
        CHECK_FALSE(preconditionOf(bad).empty());
        r.onFail(carryOn);  // no "two in a row": the earlier onFail was cleared
        r.label("z");       // ... and the label
        r.pause(0.01_s);    // not armed: the step runs (and uses up the two modifiers)
    };
    clean([&] { (void)r.atPercent(150); });
    clean([&] {
        (void)r.atPercent(20).startRun(intake, 12_volt, 0.5_s).atPercent(40).startRun(intake, 6_volt, 0.5_s);
    });  // B8: two on one mechanism
    clean([&] { (void)r.atPercent(20).startRun(intake, 12_volt, seesPiece.orAfter(3_s), 2_s); });  // D8
    clean([&] {
        (void)r.atPercent(10)
            .startRun(intake, 12_volt, 0.5_s)
            .atPercent(20)
            .startMove(arm["score"], 1_s)
            .atPercent(30)
            .startMove(lift["low"], 1_s)
            .atPercent(40)
            .startSpinUp(flywheel["shoot"], 1_s)
            .atPercent(50)
            .startMove(arm["high"], 1_s);
    });  // a fifth (on a mechanism already armed too: the room is checked first)
    clean([&] { r.atPercent(20).startRun(intake, 12_volt, 0.5_s).driveTo(0_in, 0_in); });  // the leg throws
    CHECK(r.result().steps == 5);  // the five pauses only
    CHECK(r.result().completed == 5);
    CHECK(x.svc.slot(kIntake).stepsBegun() == 0);
    CHECK(preconditionOf([&] {
              (void)r.atPercent(20).startRun(intake, 12_volt, 0.5_s).atPercent(40).startRun(intake, 6_volt, 0.5_s);
          }).find("two partway triggers on ONE mechanism") != std::string::npos);
}

TEST_CASE("STEP1-B ST35: at most FOUR triggers ride one leg -- the fifth, on a fifth mechanism, is refused") {
    // Bug caught: the room check removed or loosened (kMaxTriggers past 4: the brief's "up to 4 triggers per
    // leg"). Robot y has six motor mechanisms, so the fifth trigger is on a mechanism of its own (B8 cannot
    // be what refuses it). Its service is outside the chain: the fifth throws at its arm, before any motion.
    RobotX x;
    mech_rig::Rig yRig{mech_fixture::robot_y::kRobotYMechanisms};
    Ctx cy{x.chassis, &yRig.svc};
    Routine r{cy, "five"};
    namespace ry = mech_fixture::robot_y;
    const std::string m = preconditionOf([&] {
        (void)r.atPercent(10)
            .startMove(ry::wrist["flat"], 1_s)
            .atPercent(20)
            .startMove(ry::swing["up"], 1_s)
            .atPercent(30)
            .startMove(ry::dr4b["low"], 1_s)
            .atPercent(40)
            .startRun(ry::roller, 12_volt, 0.5_s)
            .atPercent(50)
            .startRun(ry::catapult, 12_volt, 0.5_s);
    });
    CHECK(m.find("at most 4 partway triggers ride one drive") != std::string::npos);
    CHECK(x.chassis.scheduler().motionsStarted() == 0);
    r.pause(0.01_s);  // nothing armed after it
    CHECK(r.result().steps == 1);
    // Four is accepted (and fired at the settle of a short leg: a percent is always on the leg).
    r.atPercent(10).startRun(ry::roller, 12_volt, 0.5_s).atPercent(20).startRun(ry::catapult, 12_volt, 0.5_s)
        .atPercent(30).startMove(ry::dr4b["low"], 1_s).atPercent(40).startMove(ry::swing["up"], 1_s).driveTo(6_in, 0_in);
    CHECK(yRig.svc.slot(4).stepsBegun() == 1);
    CHECK(yRig.svc.slot(5).stepsBegun() == 1);
}

TEST_CASE("STEP1-B ST35 (B9 (4)): a Routine destroyed with a trigger still ARMED logs one Warn; one never armed "
          "logs none") {
    // Bug caught: a discarded armed handle on the auton's LAST line uploading silently from a plain `make`
    // (the discard is only a warning there — L20): nothing would ever say the trigger never fired.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    {
        Routine r{ctx, "left-armed"};
        auto h = r.atPercent(60).startRun(intake, 12_volt, 0.5_s).atPercent(70).startMove(arm["score"], 1_s);
        (void)h;
    }
    CHECK(x.countAt(shulib::hal::LogLevel::Warn, "RTN",
                    "routine 'left-armed': 2 trigger(s) armed but no drive followed: they never fired") == 1);
    {
        Routine r{ctx, "clean"};
        r.atPercent(60).startRun(intake, 12_volt, 0.5_s).driveTo(10_in, 0_in);
        Routine never{ctx, "never"};
    }
    CHECK(x.count("RTN", "armed but no drive followed") == 1);
}

TEST_CASE("STEP1-B: a trigger handle drives ONCE; a Routine with no service refuses a trigger, nothing armed") {
    // Bug caught: a kept handle re-running its leg with no trigger (a silent plain drive the author did
    // not write), and a chassis-only Routine arming a trigger it can never fire.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "once"};
    auto h = r.atPercent(50).startRun(intake, 12_volt, 0.5_s);
    h.driveTo(10_in, 0_in);
    CHECK(preconditionOf([&] { h.driveTo(20_in, 0_in); }).find("already ran") != std::string::npos);
    CHECK(r.result().steps == 1);
    Routine bare{x.chassis, "bare"};
    CHECK(preconditionOf([&] { (void)bare.atPercent(50).startRun(intake, 12_volt, 0.5_s); })
              .find("needs the robot's mechanism service") != std::string::npos);
    bare.pause(0.01_s);  // not armed
    CHECK(bare.result().steps == 1);
}

// ── STEP1-B's review fix round (2026-11-11): each case below attacks a rule a reviewer found unpinned or
// broken; the bug it catches is named in it. ─────────────────────────────────────────────────────────

TEST_CASE("STEP1-B (review): NEVER on a tick the scheduler's FAULT ABORT ends -- an ODO_STUCK latched before the "
          "mark's tick") {
    // Bug caught: the observer (so the fire) running on a Running tick the scheduler then CANCELS for an
    // abort-mask fault raised since the leg started — "never on the tick that ends it" broken on the fault
    // path (the deadline path is ST29's): a mechanism started as the drive aborts. The reference run finds
    // the pace whose tick fires the 60 % mark; the fault is latched one pace before it.
    int fireIdx = -1;
    {
        RobotX x;
        int n = 0;
        x.world.onPace = [&] {
            if (fireIdx < 0 && x.svc.slot(kIntake).stepsBegun() > 0) {
                fireIdx = n;
            }
            ++n;
        };
        Ctx ctx{x.chassis, &x.svc};
        Routine r{ctx, "reference"};
        r.atPercent(60).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
        REQUIRE(r.ok());
    }
    REQUIRE(fireIdx > 2);
    RobotX x;
    int n = 0;
    x.world.onPace = [&] {
        if (n == fireIdx - 1) {  // after the tick BEFORE the fire tick: latched when the fire tick starts
            x.rig.latch.raise(shulib::diag::FaultCode::OdoStuck, "TST", "latched before the mark's tick");
        }
        ++n;
    };
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "abort"};
    r.atPercent(60).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
    REQUIRE(x.chassis.lastExitReason() == ExitReason::Cancelled);
    REQUIRE(x.chassis.lastCompleted().abortFault == shulib::diag::FaultCode::OdoStuck);
    CHECK(n == fireIdx);  // the leg ended on the very tick that would have fired (no pace after it)...
    CHECK(x.svc.slot(kIntake).stepsBegun() == 0);  // ... and nothing was started on it
    CHECK(r.result().stoppedAt == 1);             // the cut always stops (STEP1-A's first rule)
    CHECK(r.result().cause == RoutineStopCause::MotionFailed);
    CHECK(r.result().exit == ExitReason::Cancelled);
    // The accessor itself: None with no motion.
    CHECK(x.chassis.scheduler().pendingAbortFault() == shulib::diag::FaultCode::None);
}

TEST_CASE("STEP1-B (review): a handle acts for ITS line only -- a kept handle after another line, a mark started "
          "twice") {
    // Bug caught: the "drives once" guard checking only that SOMETHING is armed — a kept drive handle then
    // drives a leg carrying ANOTHER line's turn mark (the kind split bypassed: the mark can never fire, and
    // the line's wording says "the turn was 0.0 deg" on a drive), or adds its trigger to that line.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "reuse"};
    auto h = r.atPercent(50).startRun(intake, 12_volt, 0.5_s);
    h.driveTo(20_in, 0_in);
    REQUIRE(r.result().steps == 1);
    const int started = x.chassis.scheduler().motionsStarted();
    {
        auto t = r.afterTurning(30_deg).startSpinUp(flywheel["shoot"], 2_s);
        (void)t;
        CHECK(preconditionOf([&] { h.driveTo(40_in, 0_in); }).find("from an EARLIER trigger line") !=
              std::string::npos);
    }
    CHECK(x.chassis.scheduler().motionsStarted() == started);  // no drive ran
    CHECK(x.svc.slot(kFly).stepsBegun() == 0);
    r.pause(0.01_s);  // the other line was cleared with the refusal: nothing armed after it
    CHECK(r.result().steps == 2);
    {
        auto t = r.afterTurning(30_deg).startSpinUp(flywheel["shoot"], 2_s);
        (void)t;
        CHECK(preconditionOf([&] { (void)h.atPercent(80).startMove(lift["high"], 2_s); })
                  .find("from an EARLIER trigger line") != std::string::npos);
    }
    CHECK(preconditionOf([&] { (void)h.atPercent(80); }).find("already ran") != std::string::npos);
    // A MARK kept and started twice: the second start is refused (it would arm a trigger on no line).
    auto m = r.atPercent(30);
    m.startRun(intake, 12_volt, 0.5_s).driveTo(30_in, 0_in);
    CHECK(preconditionOf([&] { (void)m.startMove(lift["high"], 2_s); }).find("already started") != std::string::npos);
    // A mark written on the handle and never started: the handle's leg is refused.
    auto g = r.atPercent(30).startRun(intake, 12_volt, 0.5_s);
    (void)g.atPercent(60);
    CHECK(preconditionOf([&] { g.driveTo(10_in, 0_in); }).find("written on this handle but not started") !=
          std::string::npos);
    r.pause(0.01_s);  // cleared
    CHECK(r.result().steps == 4);
    CHECK(x.svc.slot(kLift).stepsBegun() == 0);
}

TEST_CASE("STEP1-B (verification): a MARK kept from an EARLIER line is refused while a LATER line's mark is "
          "pending -- a turn never carries a drive mark") {
    // The one state where arm()'s line-number check decides: a mark IS pending (the later line's own), so
    // "already started" cannot refuse it. The two cases share one message, so the STATE here, not the
    // wording, is what pins this half. A fresh robot and Routine, and line 1 must SETTLE: the turn then
    // really would run if the kept mark were accepted (a stopped Routine would skip it, and the motion
    // and Warn checks below would prove nothing).
    // Bug caught (verification round, RF2d): arm() checking only that SOME mark is pending -- the kept
    // DRIVE mark then arms onto a TURN line, silently consuming that line's pending mark, the line's
    // handle drives the turn, and the turn carries a drive mark (the kind split bypassed; the Warn says
    // "the drive was 0.0 in" on a turnTo).
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "kept-mark"};
    auto m1 = r.afterDistance(5_in);                          // line 1: a DRIVE mark, kept
    m1.startRun(intake, 12_volt, 0.5_s).driveTo(20_in, 0_in); // line 1 runs
    REQUIRE(r.result().steps == 1);
    REQUIRE(r.result().completed == 1);  // settled: the Routine is still running
    REQUIRE(r.result().stoppedAt == 0);
    const int beforeTurn = x.chassis.scheduler().motionsStarted();
    auto h2 = r.afterTurning(30_deg).startSpinUp(flywheel["shoot"], 2_s);  // line 2: a TURN line
    (void)h2.atPercent(50);                                                // line 2's own mark, pending
    CHECK(preconditionOf([&] { (void)m1.startMove(lift["high"], 2_s); }).find("from an earlier trigger line") !=
          std::string::npos);
    // ... and line 2 was cleared with the refusal (B9 (3)): its handle drives nothing.
    CHECK(preconditionOf([&] { h2.turnTo(90_deg); }).find("already ran") != std::string::npos);
    CHECK(x.chassis.scheduler().motionsStarted() == beforeTurn);  // no turn ran
    CHECK(r.result().steps == 1);
    CHECK(x.svc.slot(kLift).stepsBegun() == 0);
    CHECK(x.svc.slot(kFly).stepsBegun() == 0);
    CHECK(x.count("RTN", "did not fire") == 0);  // no mislabelled "the drive was 0.0 in" on a turn
    r.pause(0.01_s);  // nothing armed after it: the Routine goes on
    CHECK(r.result().steps == 2);
    CHECK(r.result().completed == 2);
}

TEST_CASE("STEP1-B (review): two failing triggers -- the step names the FIRST TO FAIL, by the order they FIRED, "
          "not armed") {
    // Bug caught: the failing trigger chosen in ARMING order (the 90 % intake, armed first, named though the
    // 20 % lift failed first), or the LAST failing one; lastMechanism() answering the last FIRED trigger's
    // report when one failed.
    struct Owner final : shulib::hal::ICancellable {
        void cancel() override {}
    };
    RobotX x{true, /*liftOnSwitchAtBoot=*/false};  // the lift is NOT homed
    Owner owner;
    REQUIRE(x.mIntake.tryClaim(owner));  // the intake is REFUSED
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "two-fail"};
    r.atPercent(90).startRun(intake, 12_volt, 0.5_s).atPercent(20).startMove(lift["high"], 2_s).driveTo(40_in, 0_in);
    REQUIRE(x.chassis.lastExitReason() == ExitReason::Settled);
    CHECK(r.result().cause == RoutineStopCause::MechanismStepFailed);
    CHECK(r.result().mechanismVerdict == MechanismOutcome::NotHomed);
    CHECK(r.lastMechanism().outcome == MechanismOutcome::NotHomed);
    CHECK(rtn(x, "STOPPED at step 1 (driveTo): mechanism step FAILED (NOT_HOMED: its trigger at 20 % on 'lift')") ==
          1);
    // One REFUSED then one that starts: the refusal is the verdict (not the later, succeeding start's).
    RobotX y;
    Owner o2;
    REQUIRE(y.mIntake.tryClaim(o2));
    Ctx cy{y.chassis, &y.svc};
    Routine q{cy, "refused-then-ok"};
    q.onFail(carryOn).atPercent(80).startSpinUp(flywheel["shoot"], 2_s).atPercent(30).startRun(intake, 12_volt, 0.5_s)
        .driveTo(40_in, 0_in);
    CHECK(y.svc.slot(kFly).stepsBegun() == 1);
    CHECK(q.result().firstFailedCause == RoutineStopCause::MechanismStepFailed);
    CHECK(q.lastMechanism().outcome == MechanismOutcome::Refused);
    CHECK(rtn(y, "(driveTo) FAILED: mechanism step FAILED (REFUSED: its trigger at 30 % on 'intake')") == 1);
    // A SUCCEEDED step earlier, then a trigger that starts: lastMechanism() is the trigger's start report
    // (Running), never the older verdict.
    RobotX z;
    Ctx cz{z.chassis, &z.svc};
    Routine w{cz, "older-ok"};
    w.move(lift["low"], 3_s);
    REQUIRE(w.lastMechanism().outcome == MechanismOutcome::Succeeded);
    w.atPercent(50).startRun(intake, 12_volt, 0.5_s).driveTo(20_in, 0_in);
    CHECK(z.svc.slot(kIntake).stepsBegun() == 1);
    CHECK(w.lastMechanism().outcome == MechanismOutcome::Running);
}

TEST_CASE("STEP1-B (review): a line that ENDS AT ITS MARK is refused by the next call, and said at the end") {
    // Bug caught: `r.atPercent(60);` alone (a plain `make` only warns) arming nothing and saying nothing —
    // the next step, `r.startMove(lift["high"], 2_s)`, then starts the lift at 0 %, not 60 %.
    RobotX x;
    Ctx ctx{x.chassis, &x.svc};
    {
        Routine r{ctx, "mark-alone"};
        (void)r.atPercent(60);
        CHECK(preconditionOf([&] { r.startMove(lift["high"], 2_s); }).find("mark is written") != std::string::npos);
        CHECK(x.svc.slot(kLift).stepsBegun() == 0);
        r.pause(0.01_s);  // cleared with the refusal
        CHECK(r.result().steps == 1);
        (void)r.afterDistance(10_in);  // and one left pending at the end
    }
    CHECK(x.countAt(shulib::hal::LogLevel::Warn, "RTN",
                    "routine 'mark-alone': a trigger's mark was written but no start and no drive followed: it "
                    "never fired") == 1);
    {
        Routine r{ctx, "each-kind"};
        (void)r.afterTurning(10_deg);
        CHECK(preconditionOf([&] { r.label("x"); }).find("mark is written") != std::string::npos);
        (void)r.afterDistance(10_in);
        CHECK(preconditionOf([&] { r.onFail(carryOn); }).find("mark is written") != std::string::npos);
        (void)r.atPercent(10);
        CHECK(preconditionOf([&] { (void)r.atPercent(20); }).find("mark is written") != std::string::npos);
    }
    CHECK(x.count("RTN", "no start and no drive followed") == 1);  // nothing pending at the second's end
}

namespace {
/// The SETTLE at the deadline: a probe auton finds when the 40 in leg settles (and when its run began);
/// the mark auton then runs the same leg with a scoring deadline exactly on that settle tick, or after it.
double gRunStart = 0.0;
double gSettledAt = 0.0;
RobotX* gX = nullptr;
ExitReason gLegExit = ExitReason::Running;
double gLegEnd = 0.0;
std::uint32_t gBegunAtLegEnd = 99;
bool gExpiredAtLegEnd = false;
RoutineResult settleProbeAuton(shulib::competition::AutonContext& ctx) {
    gRunStart = gX->now() + ctx.guard.remaining().value() - 13.0;  // when the GUARD's run began
    (void)ctx.chassis.moveTo(kForty);
    gSettledAt = ctx.chassis.lastCompleted().endTime.value();
    Routine r{ctx, "settle-probe"};
    return r.result();
}
RoutineResult settleMarkAuton(shulib::competition::AutonContext& ctx) {
    Routine r{ctx, "settle-deadline"};
    r.atPercent(100).startRun(intake, 12_volt, 0.5_s).driveTo(40_in, 0_in);
    gLegExit = ctx.chassis.lastExitReason();  // read HERE: the end action's motion comes after scoring
    gLegEnd = ctx.chassis.lastCompleted().endTime.value();
    gBegunAtLegEnd = gX->svc.slot(kIntake).stepsBegun();
    gExpiredAtLegEnd = ctx.guard.expired();
    return r.result();
}
}  // namespace

TEST_CASE("STEP1-B (review): NEVER at an arrival ON the deadline tick -- the settle fire asks the guard too (B3)") {
    // Bug caught: the settle fire (B3: a leg that ARRIVES fires its on-leg marks) skipping B3's "unless the
    // guard has expired": a leg that settles on the deadline tick itself (the motion ticks once more there,
    // P6) would start a mechanism AFTER the buzzer. "At 100 %" on robot x's 40 in leg fires ONLY at its
    // settle (it settles 0.24 in short), so the settle path alone is attacked. One tick's margin: it fires.
    {
        RobotX x;
        gX = &x;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard,
                                                       auton_rig::entryFor("probe", &settleProbeAuton, 13.0, 14.0),
                                                       nullptr, x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
    }
    REQUIRE(std::abs(gRunStart) < 1e-9);  // the guard's run begins at 0 (the auton itself at 0.02, after the
    const double settleIn = gSettledAt;   // live wait): a deadline of `settleIn` IS the settle tick's time
    for (const bool onTheDeadline : {true, false}) {
        CAPTURE(onTheDeadline);
        RobotX x;
        gX = &x;
        gLegExit = ExitReason::Running;
        gLegEnd = 0.0;
        gBegunAtLegEnd = 99;
        gExpiredAtLegEnd = !onTheDeadline;
        const double endAt = onTheDeadline ? settleIn : settleIn + 0.005;  // or before the NEXT tick
        const auto res = shulib::competition::runAuton(
            x.chassis, x.guard, auton_rig::entryFor("deadline", &settleMarkAuton, endAt, endAt + 1.5), nullptr, x.sink,
            x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == shulib::competition::AutonRunOutcome::RoutineRan);
        CHECK(gLegExit == ExitReason::Settled);  // it ARRIVED...
        CHECK(gLegEnd == gSettledAt);
        CHECK(gExpiredAtLegEnd == onTheDeadline);  // ... ON the deadline tick (or one tick before it)
        CHECK(gBegunAtLegEnd == (onTheDeadline ? 0u : 1u));
        CHECK(x.svc.slot(kIntake).stepsBegun() == (onTheDeadline ? 0u : 1u));  // and never after it
    }
}

TEST_CASE("STEP1-B (review): an armed reverseTo and faceAway are the plain verbs' legs -- backing up, facing away") {
    // Bug caught: the armed leg's reverse flag dropped (reverseTo driven nose-first, faceAway turned to face
    // the point): the trigger still fires, so only the leg itself shows it. Each armed leg and its plain
    // twin, on twin rigs, end bit-identical (ST19: an observer changes nothing).
    for (const bool turn : {false, true}) {
        CAPTURE(turn);
        RobotX a;
        RobotX b;
        Ctx ca{a.chassis, &a.svc};
        Ctx cb{b.chassis, &b.svc};
        Routine armed{ca, "armed"};
        Routine plain{cb, "plain"};
        if (turn) {
            armed.atPercent(50).startRun(intake, 12_volt, 0.5_s).faceAway(0_in, 40_in);
            plain.faceAway(0_in, 40_in);
        } else {
            armed.atPercent(50).startRun(intake, 12_volt, 0.5_s).reverseTo(-20_in, 0_in);
            plain.reverseTo(-20_in, 0_in);
        }
        REQUIRE(plain.ok());
        CHECK(armed.ok());
        CHECK(a.svc.slot(kIntake).stepsBegun() == 1);
        CHECK(a.chassis.pose().x().value() == b.chassis.pose().x().value());
        CHECK(a.chassis.pose().y().value() == b.chassis.pose().y().value());
        CHECK(a.chassis.pose().heading().radians() == b.chassis.pose().heading().radians());
        CHECK(std::string{a.chassis.lastCompleted().name} == std::string{b.chassis.lastCompleted().name});
        CHECK(a.chassis.lastCompleted().endTime.value() == b.chassis.lastCompleted().endTime.value());
        // and the leg is the one written: facing away from (0, 40) is -90 deg; backing to (-20, 0) faces +x
        if (turn) {
            CHECK(std::abs(a.chassis.pose().heading().radians() + kPi / 2.0) < 0.05);
        } else {
            CHECK(std::abs(a.chassis.pose().heading().radians()) < 0.05);
            CHECK(a.chassis.pose().x().value() < -19.0);
        }
    }
}

TEST_CASE("STEP1-B (review): a start form's NAME names the trigger in its lines") {
    // Bug caught: the start form's `name` accepted and silently dropped (the plain start forms use theirs).
    RobotX x{true, /*liftOnSwitchAtBoot=*/false};
    Ctx ctx{x.chassis, &x.svc};
    Routine r{ctx, "named"};
    r.onFail(carryOn).atPercent(20).startMove(lift["high"], 2_s, "raise").driveTo(40_in, 0_in);
    CHECK(rtn(x, "(NOT_HOMED: its trigger 'raise' at 20 % on 'lift')") == 1);
    r.afterDistance(60_in).startRun(intake, 12_volt, 0.5_s, "feed").driveTo(50_in, 0_in);
    CHECK(rtn(x, "step 2 (driveTo): trigger 'feed' after 60.0 in on 'intake' did not fire: the drive was 1") == 1);
}
