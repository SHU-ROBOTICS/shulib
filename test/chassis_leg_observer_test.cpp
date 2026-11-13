// STEP1-B — the Chassis's leg OBSERVER overloads (chassis.hpp's "leg OBSERVER": moveTo / turnTo with a
// LegObserver), on the host test plant: a 12 in tank on the A2 plant, the memoryless plant (kA 0) and
// the motor-lag plant (kA 0.02). Say "host test plant", never "simulator"; every number is the plant's.
//
// Each case attacks ONE rule (the STEP1 brief's §2.2): ST19 THE SEAM ADDS NOTHING — an observer that
// does nothing leaves the leg bit-identical to the plain verb (every pace's TRUE pose, the exit, its
// tick, the record's name), and it is called exactly once per tick that did NOT end the leg — never on
// the exit tick (SM14), never through a trailing pace (SM23: a waitUntil-built seam paces once more,
// the brief's probe P5); the progress it hands over is D7 (a)'s and the start is captured at the
// motion's first LIVE tick, not at the call.

#include "doctest.h"

#include <cmath>
#include <string>
#include <vector>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/motion/leg_progress.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/units/literals.hpp"

using namespace shulib::units::literals;
using shulib::chassis::Chassis;
using shulib::chassis::LegObserver;
using shulib::chassis::MotionOptions;
using shulib::control::ExitReason;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::motion::LegProgress;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// The world below the chassis: one 10 ms plant step per pace, and every pace's TRUE pose (the
/// bit-identity witness, as STEP1-A's ST2).
struct World final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    std::vector<double> trace;
    int paces = 0;
    void pace() override {
        ++paces;
        rig->h.plant().step(Time{0.01});
        const Pose2d p = rig->h.truePose();
        trace.push_back(p.x().value());
        trace.push_back(p.y().value());
        trace.push_back(p.heading().radians());
    }
};

/// A 12 in tank, the plant's kA chosen per case; `imu` (optional) is the hostile IMU's boot window.
struct Tank {
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    motion_rig::MotionRig rig;
    World world;
    Chassis ch;
    explicit Tank(double plantKa, shulib::sim::DegradationModel* imu = nullptr)
        : rig{kin, cfgFor(plantKa), &sink, imu},
          ch{rig.deps, world, shulib::chassis::ChassisConfig{.motion = motion_rig::motionConfig()}} {
        world.rig = &rig;
    }
    static shulib::sim::SimHarnessConfig cfgFor(double kA) {
        auto c = motion_rig::plantConfig();
        c.plant.wheelFf.kA = kA;
        return c;
    }
    [[nodiscard]] double now() { return rig.h.clock().now().value(); }
};

/// The observer under test: records every call (the clock, the progress, the pose the estimate held).
struct Spy {
    Tank* t = nullptr;
    std::vector<double> at;
    std::vector<LegProgress> seen;
    std::vector<Pose2d> estimate;
    static void call(void* self, const LegProgress& p) {
        Spy& s = *static_cast<Spy*>(self);
        s.at.push_back(s.t->now());
        s.seen.push_back(p);
        s.estimate.push_back(s.t->ch.pose());
    }
    LegObserver observer() { return LegObserver{&Spy::call, this}; }
};

void doNothing(void*, const LegProgress&) {}

const Pose2d kForty{Length{40.0}, Length{0.0}, Angle{}};

}  // namespace

TEST_CASE("STEP1-B ST19: an observer that does nothing leaves the leg BIT-IDENTICAL to the plain verb") {
    // Bug caught: a seam that changes the leg it watches — a trailing pace (the waitUntil build, P5's
    // finding: SM23), a different config, a renamed record ("MoveToPose" is read by results and tests,
    // L7). The witness: every pace's TRUE pose, the exit, the exit TICK and the record, compared EXACTLY.
    for (const double kA : {0.0, 0.02}) {
        CAPTURE(kA);
        for (const bool nullFn : {false, true}) {
            CAPTURE(nullFn);
            const LegObserver o = nullFn ? LegObserver{} : LegObserver{&doNothing, nullptr};
            Tank a{kA};
            Tank b{kA};
            const MotionOptions opts{.positionTolerance = 1_in};
            const ExitReason ea = a.ch.moveTo(kForty, opts);
            const ExitReason eb = b.ch.moveTo(kForty, opts, o);
            CHECK(ea == ExitReason::Settled);
            CHECK(ea == eb);
            CHECK(a.ch.lastCompleted().endTime.value() == b.ch.lastCompleted().endTime.value());
            CHECK(std::string{b.ch.lastCompleted().name} == "MoveToPose");
            CHECK(std::string{a.ch.lastCompleted().name} == std::string{b.ch.lastCompleted().name});
            CHECK(a.world.trace == b.world.trace);
            CHECK(a.now() == b.now());
            const ExitReason ta = a.ch.turnTo(Angle::degrees(90.0));
            const ExitReason tb = b.ch.turnTo(Angle::degrees(90.0), {}, o);
            CHECK(ta == ExitReason::Settled);
            CHECK(ta == tb);
            CHECK(std::string{b.ch.lastCompleted().name} == "TurnTo");
            CHECK(a.ch.lastCompleted().endTime.value() == b.ch.lastCompleted().endTime.value());
            CHECK(a.world.trace == b.world.trace);
            CHECK(a.ch.pose().x().value() == b.ch.pose().x().value());
            CHECK(a.ch.pose().heading().radians() == b.ch.pose().heading().radians());
            CHECK(a.ch.scheduler().completedCount() == b.ch.scheduler().completedCount());
            CHECK(a.world.trace.size() > 300);  // a real trace, not two empty vectors
        }
    }
}

TEST_CASE("STEP1-B ST19: called exactly once per tick that did not end the leg -- never on the exit tick") {
    // Bug caught: the observer called on the exit tick too (SM14 — a trigger could fire on the tick
    // the leg ENDED, outside D15's rule), or not every tick (a mark crossed between calls fires late).
    // Under waitUntilSettled a leg of N ticks paces N - 1 times (no trailing pace), so the count of
    // Running ticks IS the pace count; the last call is one tick (10 ms) before the exit tick.
    for (const double kA : {0.0, 0.02}) {
        CAPTURE(kA);
        Tank t{kA};
        Spy s;
        s.t = &t;
        const int paces0 = t.world.paces;
        const double t0 = t.now();
        REQUIRE(t.ch.moveTo(kForty, {}, s.observer()) == ExitReason::Settled);
        const int legPaces = t.world.paces - paces0;
        CHECK(static_cast<int>(s.seen.size()) == legPaces);
        CHECK(s.at.front() == t0);  // the first tick, at the call's clock (nothing paced before it)
        const double end = t.ch.lastCompleted().endTime.value();
        CHECK(s.at.back() < end);
        CHECK(s.at.back() == doctest::Approx(end - 0.01).epsilon(1e-12));
        for (std::size_t i = 1; i < s.at.size(); ++i) {
            CHECK(s.at[i] > s.at[i - 1]);  // once per tick: no tick called twice
        }
        // The turn, the same.
        Spy u;
        u.t = &t;
        const int p1 = t.world.paces;
        REQUIRE(t.ch.turnTo(Angle::degrees(-60.0), {}, u.observer()) == ExitReason::Settled);
        CHECK(static_cast<int>(u.seen.size()) == t.world.paces - p1);
        CHECK(u.at.back() == doctest::Approx(t.ch.lastCompleted().endTime.value() - 0.01).epsilon(1e-12));
    }
}

TEST_CASE("STEP1-B ST19: the progress handed over is D7 (a)'s, from the pose THAT tick used") {
    // Bug caught: progress computed from a stale or a later pose (a trigger fires a tick early or late),
    // the drive's fraction not along the line, the turn's not the short way, or the leg's geometry and
    // EFFECTIVE tolerance (B3 reads them after the leg) missing or the robot's own when the step's own
    // options loosened them.
    Tank t{0.0};
    Spy s;
    s.t = &t;
    REQUIRE(t.ch.moveTo(kForty, {.positionTolerance = 2_in, .headingTolerance = 5_deg}, s.observer()) ==
            ExitReason::Settled);
    REQUIRE(s.seen.size() > 50);
    // The rig's estimate goes live at its first update with a ready IMU: at most the first tick waits.
    std::size_t first = 0;
    while (first < s.seen.size() && !s.seen[first].live) {
        ++first;
    }
    CHECK(first <= 1);
    for (std::size_t i = first; i < s.seen.size(); ++i) {
        const LegProgress& p = s.seen[i];
        CHECK(p.live);
        CHECK(p.hasDirection);
        CHECK(p.length.value() == 40.0);  // the live start was (0, 0)
        CHECK(p.sizeRad == 0.0);
        CHECK(p.turnedRad == 0.0);
        // the projection on +x of THIS tick's pose (to the last bits: x * 40 / 40 may round by an ulp; a
        // pose one tick stale or late is off by ~0.4 in)
        CHECK(p.along.value() == doctest::Approx(s.estimate[i].x().value()).epsilon(1e-12));
        CHECK(p.fraction == doctest::Approx(s.estimate[i].x().value() / 40.0).epsilon(1e-12));
        CHECK(p.positionTolerance.value() == 2.0);  // the loosened settle distance, not the robot's 0.5
        CHECK(p.headingToleranceRad == doctest::Approx(5.0 * 3.14159265358979323846 / 180.0));
    }
    CHECK(s.seen[first].fraction == 0.0);  // the first live tick IS the start
    CHECK(s.seen.back().fraction > 0.9);
    // The robot's own tolerance when no option loosens it.
    Spy d;
    d.t = &t;
    REQUIRE(t.ch.moveTo(Pose2d{Length{40.0}, Length{20.0}, Angle::degrees(90.0)}, {}, d.observer()) !=
            ExitReason::Running);
    CHECK(d.seen.back().positionTolerance.value() == motion_rig::motionConfig().translationSettle.maxError);
    CHECK(d.seen.back().headingToleranceRad == motion_rig::motionConfig().headingSettle.maxError);
    // The turn: the angle turned the short way, from the heading at its first live tick.
    Spy u;
    u.t = &t;
    const double h0 = t.ch.pose().heading().radians();
    REQUIRE(t.ch.turnTo(Angle::degrees(0.0), {}, u.observer()) == ExitReason::Settled);
    const double size = std::abs(Angle::radians(h0).errorTo(Angle::degrees(0.0)));
    for (std::size_t i = 0; i < u.seen.size(); ++i) {
        CHECK(u.seen[i].sizeRad == size);
        CHECK(u.seen[i].length.value() == 0.0);
        CHECK(u.seen[i].along.value() == 0.0);
        const double turned = (Angle::radians(h0).errorTo(Angle::degrees(0.0)) > 0 ? 1.0 : -1.0) *
                              Angle::radians(h0).errorTo(u.estimate[i].heading());
        CHECK(u.seen[i].turnedRad == turned);
    }
    CHECK(u.seen.front().turnedRad == 0.0);
    CHECK(u.seen.back().fraction > 0.9);
}

TEST_CASE("STEP1-B ST19: through the boot wait the observer says NOT live; the start is the first LIVE tick's") {
    // Bug caught: the start captured at the call (during the boot window the estimate is garbage — a
    // trigger would measure from it), or progress reported as live before the estimate is.
    shulib::sim::ImuHostileConfig ic{};
    ic.calibrationEnd = Time{0.5};
    ic.rateBiasMax = shulib::units::AngularVelocity{0.0};
    ic.headingNoiseSigmaRad = 0.0;
    ic.yawRateNoiseSigmaRadPerS = 0.0;
    shulib::sim::ImuHostileModel imu{ic};
    Tank t{0.0, &imu};
    Spy s;
    s.t = &t;
    REQUIRE(t.ch.moveTo(kForty, {.timeout = 6_s}, s.observer()) == ExitReason::Settled);
    std::size_t firstLive = 0;
    while (firstLive < s.seen.size() && !s.seen[firstLive].live) {
        CHECK(s.seen[firstLive].fraction == 0.0);
        ++firstLive;
    }
    CHECK(firstLive >= 40);  // ~0.5 s of boot wait, every tick said "not live"
    REQUIRE(firstLive < s.seen.size());
    CHECK(s.at[firstLive] >= 0.5);
    CHECK(s.seen[firstLive].fraction == 0.0);  // the first live tick is the start: progress 0
    for (std::size_t i = firstLive; i < s.seen.size(); ++i) {
        CHECK(s.seen[i].live);  // once live, live to the end
    }
}

TEST_CASE("STEP1-B: an observer overload validates its options at the door, exactly as the plain verb") {
    // Bug caught: the overload skipping MotionOptions::validate (a NaN timeout reaching the motion).
    Tank t{0.0};
    Spy s;
    s.t = &t;
    CHECK_THROWS_AS(t.ch.moveTo(kForty, {.timeout = Time{-1.0}}, s.observer()), shulib::PreconditionError);
    CHECK_THROWS_AS(t.ch.turnTo(Angle::degrees(10.0), {.positionTolerance = Length{NAN}}, s.observer()),
                    shulib::PreconditionError);
    CHECK(s.seen.empty());
    CHECK(t.ch.scheduler().motionsStarted() == 0);
}
