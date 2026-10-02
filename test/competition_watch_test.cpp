// competition_watch_test.cpp — "shulib Competition"'s WATCH (src/programs/competition_watch.hpp;
// chunk COMP-b, 2026-09-30; the COMP brief's B7, B22 and B30). Every case names the bug it would
// catch.
//
// The watch is the only thing between a field's DISABLE (or the end of the autonomous period) and
// an auton that is still driving: it must end the run at once, and it must never end one for any
// other reason — in a match a driver's bumped button must not end the auton (the reason it is not
// the tester's watch). So the decision, the latch, the every-tick request, the order of its read
// and its pace, and the chain it sits in are each tried here against the input that would break
// them. Host test plant only; nothing here has run on a robot (worksheet Station K).

#include "doctest.h"

#include <cstdint>
#include <vector>

#include "competition_rig.hpp"
#include "motion_test_rig.hpp"
#include "common/guarded_run.hpp"
#include "common/state_screens.hpp"
#include "programs/competition_watch.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::CompetitionCause;
using shulib::bench::competitionCauseOf;
using shulib::bench::competitionCauseShort;
using shulib::bench::competitionCauseWord;
using shulib::bench::CompetitionWatch;
using shulib::bench::kStatusAutonomous;
using shulib::bench::kStatusConnected;
using shulib::bench::kStatusDisabled;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::units::Length;
using shulib::units::Time;

namespace {

const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

/// A status script that counts its reads and stamps them into a shared sequence.
struct CountingStatus final : shulib::bench::ICompetitionStatus {
    std::uint8_t next = comp_rig::kAutonEnabled;
    int reads = 0;
    std::vector<char>* order = nullptr;
    std::uint8_t read() override {
        ++reads;
        if (order != nullptr) {
            order->push_back('r');
        }
        return next;
    }
};

/// An inner pacer that counts and stamps.
struct CountingPacer final : shulib::motion::ITickPacer {
    int paces = 0;
    std::vector<char>* order = nullptr;
    void pace() override {
        ++paces;
        if (order != nullptr) {
            order->push_back('p');
        }
    }
};

}  // namespace

// (W1) Bug caught: a cause dropped (a disable that does not end the run; a period that ends with
// the auton still driving), the precedence flipped (a disabled robot reported as "left
// autonomous"), or a bit that must NOT matter making a difference (the connected bit — a Timed
// Run and a field both run autonomous; the SYSTEM bit). All 256 status bytes.
TEST_CASE("watch (W1): competitionCauseOf — every status byte; DISABLED first; only two bits matter") {
    for (int s = 0; s < 256; ++s) {
        CAPTURE(s);
        const auto b = static_cast<std::uint8_t>(s);
        const CompetitionCause want = (b & kStatusDisabled) != 0      ? CompetitionCause::Disabled
                                      : (b & kStatusAutonomous) == 0 ? CompetitionCause::LeftAutonomous
                                                                     : CompetitionCause::None;
        CHECK(competitionCauseOf(b) == want);
    }
    // The four statuses that matter, by name.
    CHECK(competitionCauseOf(comp_rig::kAutonEnabled) == CompetitionCause::None);
    CHECK(competitionCauseOf(kStatusAutonomous) == CompetitionCause::None);  // a Timed Run: no field
    CHECK(competitionCauseOf(comp_rig::kDisabledOnField) == CompetitionCause::Disabled);
    CHECK(competitionCauseOf(kStatusDisabled | kStatusAutonomous) == CompetitionCause::Disabled);
    CHECK(competitionCauseOf(comp_rig::kDriverOnField) == CompetitionCause::LeftAutonomous);
    CHECK(competitionCauseOf(0) == CompetitionCause::LeftAutonomous);  // driver control, no field
    for (const CompetitionCause c : {CompetitionCause::None, CompetitionCause::Disabled,
                                     CompetitionCause::LeftAutonomous}) {
        CHECK(std::string_view{competitionCauseWord(c)}.size() <= 27);
        CHECK(std::string_view{competitionCauseShort(c)}.size() <= 15);
    }
}

// (W2) Bug caught: a DISARMED watch that still reads or latches (it would end a run that is not
// its own — the tester's pattern of arming per run), or a pace counter that counts only while armed
// (the boot's chain proof would see no pace and refuse a good build).
TEST_CASE("watch (W2): disarmed, the watch only paces — and paces() counts every pace") {
    CountingPacer inner;
    CountingStatus status;
    status.next = comp_rig::kDisabledOnField;  // a disable: a disarmed watch must do nothing
    CompetitionWatch watch{inner, status};
    for (int i = 0; i < 5; ++i) {
        watch.pace();
    }
    CHECK(status.reads == 0);
    CHECK(inner.paces == 5);
    CHECK(watch.cause() == CompetitionCause::None);
    CHECK_FALSE(watch.armed());
    CHECK(watch.paces() == 5);
    watch.arm();
    watch.pace();
    CHECK(watch.paces() == 6);
    CHECK(watch.cause() == CompetitionCause::Disabled);
    watch.disarm();
    watch.pace();
    CHECK(watch.paces() == 7);
    CHECK(status.reads == 1);  // latched: it stops reading
}

// (W3) Bug caught: a latch that forgets (the status flickers back and the run resumes), a second
// cause overwriting the first (the log would blame the wrong thing), statusAtLatch taken at the
// wrong read, or arm() keeping an old latch (the next autonomous() would be abandoned at T+0).
TEST_CASE("watch (W3): the latch holds the FIRST cause and its status byte; arm() clears it") {
    CountingPacer inner;
    CountingStatus status;
    CompetitionWatch watch{inner, status};
    watch.arm();
    watch.pace();
    watch.pace();
    CHECK(watch.cause() == CompetitionCause::None);
    status.next = comp_rig::kDriverOnField;  // the period ended
    watch.pace();
    CHECK(watch.cause() == CompetitionCause::LeftAutonomous);
    CHECK(watch.statusAtLatch() == comp_rig::kDriverOnField);
    status.next = comp_rig::kAutonEnabled;  // back — the latch holds
    watch.pace();
    CHECK(watch.cause() == CompetitionCause::LeftAutonomous);
    status.next = comp_rig::kDisabledOnField;  // a "more serious" cause — never replaces it
    watch.pace();
    CHECK(watch.cause() == CompetitionCause::LeftAutonomous);
    CHECK(watch.statusAtLatch() == comp_rig::kDriverOnField);
    watch.disarm();
    CHECK(watch.cause() == CompetitionCause::LeftAutonomous);  // readable until the next arm
    status.next = comp_rig::kAutonEnabled;
    watch.arm();
    CHECK(watch.cause() == CompetitionCause::None);
    CHECK(watch.statusAtLatch() == 0);
    watch.pace();
    CHECK(watch.cause() == CompetitionCause::None);
}

// (W4) Bug caught: the request made only on the latch EDGE. The guard DROPS a request made while no
// run is live (run_guard.hpp's ABANDON banner), and the program arms the watch just BEFORE runAuton
// arms the guard — so a disable latched in that gap must still abandon the run once it is live.
// Shown on the plant: latched while idle (no request), then the run's first pace requests it and
// the run is abandoned with its end action skipped.
TEST_CASE("watch (W4): latched while idle -> no request; a run live -> requested every pace") {
    comp_rig::CompetitionStack s{tank()};
    s.watch.arm();
    s.status.status = comp_rig::kDisabledOnField;
    s.chassis.wait(Time{0.01});
    CHECK(s.watch.cause() == CompetitionCause::Disabled);
    CHECK_FALSE(s.guard.abandonRequested());  // idle: nothing to abandon
    int pacesBeforeRequest = -1;
    const auto rep = s.guard.run(
        s.chassis, RunGuardConfig{.endActionAt = Time{2.0}, .hardStopAt = Time{3.0}},
        [&] {
            for (int i = 0; i < 20; ++i) {
                s.chassis.wait(Time{0.01});
                if (s.guard.abandonRequested() && pacesBeforeRequest < 0) {
                    pacesBeforeRequest = i;
                }
            }
        },
        [] { return true; });
    CHECK(pacesBeforeRequest == 0);  // the FIRST live pace requests it
    CHECK(rep.abandoned);
    CHECK_FALSE(rep.endActionRan);
    s.watch.disarm();
}

// (W5) Bug caught: the watch advancing the world BEFORE it reads (a disable seen one tick late), or
// pacing its inner pacer twice or not at all (the robot clock drifts, or the world stops).
TEST_CASE("watch (W5): each armed pace reads the status, THEN paces the inner pacer, once") {
    std::vector<char> order;
    CountingPacer inner;
    inner.order = &order;
    CountingStatus status;
    status.order = &order;
    CompetitionWatch watch{inner, status};
    watch.arm();
    for (int i = 0; i < 4; ++i) {
        watch.pace();
    }
    CHECK(order == std::vector<char>{'r', 'p', 'r', 'p', 'r', 'p', 'r', 'p'});
    CHECK(inner.paces == 4);
}

// (W6) Bug caught (B30; the boot's chain proof): a Chassis whose pacer is not the guard over this
// watch — built on the watch (the guard never paces), on the plant (neither does) — or a watch
// never attached / attached to another guard (it latches a disable and asks nobody). The shared
// proof must say true for the right chain only, over THIS watch, commanding nothing.
TEST_CASE("watch (W6): pacerChainProven over the competition watch — the right chain only") {
    comp_rig::CompetitionStack s{tank(), 1, 1.0, 0};
    CHECK(shulib::bench::pacerChainProven(s.chassis, s.guard, s.watch));
    RunGuard other{s.plant};
    s.watch.attachGuard(other);
    CHECK_FALSE(shulib::bench::pacerChainProven(s.chassis, s.guard, s.watch));
    s.watch.attachGuard(s.guard);
    shulib::chassis::Chassis onWatch{s.rig.deps, s.watch, motion_rig::chassisConfig()};
    CHECK_FALSE(shulib::bench::pacerChainProven(onWatch, s.guard, s.watch));
    shulib::chassis::Chassis onPlant{s.rig.deps, s.plant, motion_rig::chassisConfig()};
    CHECK_FALSE(shulib::bench::pacerChainProven(onPlant, s.guard, s.watch));
    CHECK(s.plant.distance == 0.0);
    CHECK(s.probe.energizedWhileIdle == 0);
    // A watch built but never attached: the same chain, the back link missing.
    motion_rig::MotionRig rig2{tank()};
    motion_rig::PlantPacer plant2{rig2.h};
    comp_rig::ScriptedStatus status2;
    status2.clock = &rig2.h.clock();
    CompetitionWatch unattached{plant2, status2};
    RunGuard guard2{unattached};
    shulib::chassis::Chassis onChain{rig2.deps, guard2, motion_rig::chassisConfig()};
    CHECK_FALSE(shulib::bench::pacerChainProven(onChain, guard2, unattached));
    unattached.attachGuard(guard2);
    CHECK(shulib::bench::pacerChainProven(onChain, guard2, unattached));
}

// (W7) Bug caught (B22, the one-tick latch; B7's "<= one tick of latency"): a disable mid-motion
// that lets the drive keep going — the watch reading after the world advances, or the guard's cut
// arriving late. On the plant, a live 30 in moveTo under the guard; the status goes DISABLED at a
// chosen instant: the world advances at most ONE more tick after the watch latched, the plant
// travels at most one tick's worth, the run is ABANDONED with its end action skipped, and the drive
// ends at 0 V under Brake.
TEST_CASE("watch (W7): a disable mid-motion — at most one more commanded tick, then safe") {
    comp_rig::CompetitionStack s{tank()};
    const double start = s.now();
    s.status.onRead = [&](double now) {
        if (now - start >= 0.6) {
            s.status.status = comp_rig::kDisabledOnField;
        }
    };
    s.watch.arm();
    const auto rep = s.guard.run(
        s.chassis, RunGuardConfig{.endActionAt = Time{20.0}, .hardStopAt = Time{25.0}},
        [&] { (void)s.chassis.moveTo(Pose2d{Length{30.0}, Length{0.0}, Angle{}}); },
        [&] { return s.chassis.brake() == shulib::control::ExitReason::Settled; });
    s.watch.disarm();
    CHECK(s.watch.cause() == CompetitionCause::Disabled);
    CHECK(rep.abandoned);
    CHECK_FALSE(rep.endActionRan);
    REQUIRE(s.probe.travelAtLatch >= 0.0);
    CHECK(s.probe.travelAtLatch > 1.0);  // it WAS driving when the disable came
    // One more commanded tick at most: 60 in/s cap x 10 ms, plus rounding slack.
    CHECK(s.plant.distance - s.probe.travelAtLatch <= 60.0 * 0.01 + 1e-6);
    CHECK(s.driveSafe());
    CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
}

// (W8) Bug caught (COMP-b's review, PB1): the request's one read before anything moves --
// latchNow() reading more than once, or not at all; not latching what it read; keeping an OLDER
// latch (a skipped request would carry the previous run's cause); arming the watch (a pace after it
// would ask a guard to abandon); or pacing the world. arm() after it starts clean.
TEST_CASE("watch (W8): latchNow() -- ONE read, latched, an older latch cleared, never armed") {
    CountingStatus status;
    CountingPacer inner;
    CompetitionWatch watch{inner, status};
    // An older latch: armed, a disable seen on a pace.
    watch.arm();
    status.next = comp_rig::kDisabledOnField;
    watch.pace();
    REQUIRE(watch.cause() == CompetitionCause::Disabled);
    watch.disarm();
    // The period is on: the read latches nothing, and the old cause is gone.
    status.next = comp_rig::kAutonEnabled;
    const int readsBefore = status.reads;
    const int pacesBefore = inner.paces;
    CHECK(watch.latchNow() == CompetitionCause::None);
    CHECK(status.reads == readsBefore + 1);
    CHECK(inner.paces == pacesBefore);  // it reads; it never paces
    CHECK(watch.cause() == CompetitionCause::None);
    CHECK(watch.statusAtLatch() == 0);
    CHECK_FALSE(watch.armed());
    // Driver control already: LEFT AUTONOMOUS, latched with its status byte, still not armed.
    status.next = comp_rig::kDriverOnField;
    CHECK(watch.latchNow() == CompetitionCause::LeftAutonomous);
    CHECK(watch.cause() == CompetitionCause::LeftAutonomous);
    CHECK(watch.statusAtLatch() == comp_rig::kDriverOnField);
    CHECK_FALSE(watch.armed());
    // Disabled wins over the autonomous bit, as in competitionCauseOf.
    status.next = static_cast<std::uint8_t>(kStatusDisabled | kStatusAutonomous | kStatusConnected);
    CHECK(watch.latchNow() == CompetitionCause::Disabled);
    // arm() starts the run clean.
    watch.arm();
    CHECK(watch.cause() == CompetitionCause::None);
    CHECK(watch.statusAtLatch() == 0);
}
