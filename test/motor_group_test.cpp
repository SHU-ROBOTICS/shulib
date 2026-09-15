// hal::MotorGroup — the unit tests (chunk R3b Part 1). Each names the bug it catches and
// the mutation that must turn it red (Session-2 brief §7 numbering):
//   1  fan-out: every member receives every setVoltage / setBrakeMode    (skip member N)
//   3  MEDIAN with one FROZEN member: the aggregate is untouched            (median → mean)
//   5  MotionDeps with MORE motors than wheels THROWS, naming MotorGroup    (== → >=)
//   6  temperature() = max, current() = mean per member, sum exposed        (max → mean)
//  16  a member ~20 % slow (port 18's signature) is NOT a disagreement;
//      the same member OPPOSITE in sign IS                                  (threshold; sign test)
// The plant-level cases (2: N = 1 bit-identity through a routine; 4: the fault route; 7: the
// N = 5 equivalence sweeps) live in motor_group_plant_test.cpp.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <span>
#include <string>

#include "shulib/chassis/robot_context.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::BrakeMode;
using shulib::hal::IMotor;
using shulib::hal::MotorGroup;
using shulib::hal::fake::FakeMotor;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Voltage;

namespace {

/// N fakes and a group over them, in one place.
template <std::size_t N>
struct Fakes {
    std::array<FakeMotor, N> motors{};
    std::array<IMotor*, N> ptrs{};
    Fakes() {
        for (std::size_t i = 0; i < N; ++i) {
            ptrs[i] = &motors[i];
        }
    }
    [[nodiscard]] std::span<IMotor* const> span() const { return ptrs; }
};

}  // namespace

// ═══ 1. Fan-out ═════════════════════════════════════════════════════════════════════
// Bug caught: a silent surplus member — a fan-out loop that stops one short, or a brake
// mode that reaches only the first member (the exact defect the group exists to end).
TEST_CASE("MotorGroup 1: every member receives every setVoltage and setBrakeMode") {
    Fakes<5> f;
    MotorGroup g{f.span()};
    REQUIRE(g.memberCount() == 5);

    g.setVoltage(Voltage{7.25});
    for (const FakeMotor& m : f.motors) {
        CHECK(m.commandedVoltage().value() == 7.25);
    }
    CHECK(g.commandedVoltage().value() == 7.25);

    // Over the ceiling: each member clamps for itself, and the mirror shows the clamp.
    g.setVoltage(Voltage{15.0});
    for (const FakeMotor& m : f.motors) {
        CHECK(m.commandedVoltage().value() == 12.0);
    }
    CHECK(g.commandedVoltage().value() == 12.0);
    g.setVoltage(Voltage{-40.0});
    for (const FakeMotor& m : f.motors) {
        CHECK(m.commandedVoltage().value() == -12.0);
    }
    CHECK(g.commandedVoltage().value() == -12.0);

    g.setBrakeMode(BrakeMode::Hold);
    for (const FakeMotor& m : f.motors) {
        CHECK(m.brakeMode() == BrakeMode::Hold);
    }
    CHECK(g.brakeMode() == BrakeMode::Hold);  // the first member's read-back
    g.setBrakeMode(BrakeMode::Coast);
    for (const FakeMotor& m : f.motors) {
        CHECK(m.brakeMode() == BrakeMode::Coast);
    }

    // A non-finite command is rejected BEFORE any member sees it: nothing half-applies.
    g.setVoltage(Voltage{3.0});
    CHECK_THROWS_AS(g.setVoltage(Voltage{std::numeric_limits<double>::quiet_NaN()}),
                    PreconditionError);
    for (const FakeMotor& m : f.motors) {
        CHECK(m.commandedVoltage().value() == 3.0);
    }
    CHECK(g.commandedVoltage().value() == 3.0);

    // A sweep of commands, every member every time — order and count both pinned.
    shulib::sim::Rng rng{11};
    for (int i = 0; i < 500; ++i) {
        const double v = rng.uniform(-14.0, 14.0);
        g.setVoltage(Voltage{v});
        const double expected = std::clamp(v, -12.0, 12.0);
        for (const FakeMotor& m : f.motors) {
            REQUIRE(m.commandedVoltage().value() == expected);
        }
        REQUIRE(g.commandedVoltage().value() == expected);
    }
}

TEST_CASE("MotorGroup: construction refuses an empty span and a null member") {
    std::array<IMotor*, 0> none{};
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{none}}), PreconditionError);
    FakeMotor a;
    std::array<IMotor*, 3> withNull{&a, nullptr, &a};
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{withNull}}), PreconditionError);
    std::array<IMotor*, MotorGroup::kMaxMembers + 1> tooMany{};
    tooMany.fill(&a);
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{tooMany}}), PreconditionError);
    std::array<IMotor*, 1> one{&a};
    MotorGroup g{std::span<IMotor* const>{one}};
    CHECK(&g.member(0) == &a);
    CHECK_THROWS_AS((void)g.member(1), PreconditionError);
}

// ═══ 3. Median with a frozen member ═════════════════════════════════════════════════
// Bug caught: the 1/N drift — a mean lets a dead port's frozen last-good reading drag the
// side's position by 1/N of all further travel, silently and forever.
TEST_CASE("MotorGroup 3: position/velocity are the MEDIAN — one frozen member cannot drag them") {
    Fakes<5> f;
    MotorGroup g{f.span()};
    // Four live members agree at 10 rad; the fifth froze at 3 rad (a dead port).
    for (std::size_t i = 0; i < 4; ++i) {
        f.motors[i].setPosition(AngleDim{10.0});
        f.motors[i].setVelocity(AngularVelocity{6.5});
    }
    f.motors[4].setPosition(AngleDim{3.0});
    f.motors[4].setVelocity(AngularVelocity{0.0});
    CHECK(g.position().value() == 10.0);  // exact — a mean would read 8.6
    CHECK(g.velocity().value() == 6.5);   // exact — a mean would read 5.2
    // Whichever member froze: the median does not care about position in the span.
    for (std::size_t frozen = 0; frozen < 5; ++frozen) {
        for (std::size_t i = 0; i < 5; ++i) {
            f.motors[i].setPosition(AngleDim{i == frozen ? -40.0 : 123.456});
            f.motors[i].setVelocity(AngularVelocity{i == frozen ? 0.0 : -9.75});
        }
        CHECK(g.position().value() == 123.456);
        CHECK(g.velocity().value() == -9.75);
    }
    // Even N >= 4: one frozen member still cannot reach the two middle values.
    Fakes<4> e;
    MotorGroup g4{e.span()};
    for (std::size_t i = 0; i < 3; ++i) {
        e.motors[i].setPosition(AngleDim{50.0});
    }
    e.motors[3].setPosition(AngleDim{7.0});
    CHECK(g4.position().value() == 50.0);
    e.motors[3].setPosition(AngleDim{9000.0});  // and a runaway member cannot either
    CHECK(g4.position().value() == 50.0);

    // Property sweep: the median is order-independent and never outside [min, max]; with
    // a majority agreeing it IS the agreed value, exactly.
    shulib::sim::Rng rng{3};
    for (int trial = 0; trial < 300; ++trial) {
        const double agreed = rng.uniform(-1000.0, 1000.0);
        const std::size_t bad = static_cast<std::size_t>(rng.uniform(0.0, 4.999));
        for (std::size_t i = 0; i < 5; ++i) {
            f.motors[i].setPosition(AngleDim{i == bad ? rng.uniform(-5000.0, 5000.0) : agreed});
        }
        REQUIRE(g.position().value() == agreed);
    }
}

// ═══ 5. The tightened guard ═════════════════════════════════════════════════════════
// Bug caught: six silent motors — a context with more motors than kinematic wheels used to
// be ACCEPTED (the old `>=`), and the pipeline commanded wheel-count motors and no more.
TEST_CASE("MotorGroup 5: MotionDeps with MORE motors than wheels throws, naming MotorGroup") {
    using shulib::hal::fake::FakeBattery;
    using shulib::hal::fake::FakeClock;
    using shulib::hal::fake::FakeGps;
    using shulib::hal::fake::FakeImu;
    using shulib::hal::fake::FakeRotation;
    using shulib::hal::fake::FakeTagSource;
    using shulib::hal::fake::FakeTelemetrySink;
    using shulib::hal::fake::FakeVision;
    using shulib::localization::TrackingWheel;

    FakeClock clock;
    FakeImu imu;
    FakeGps gps;
    FakeBattery battery;
    FakeTagSource tags;
    FakeVision vision;
    FakeTelemetrySink sink;
    FakeRotation fwd, lat;
    shulib::localization::PilonsOdometry odom{
        imu, TrackingWheel::forward(fwd, Length{2.0}, Length{0.0}),
        TrackingWheel::lateral(lat, Length{2.0}, Length{0.0})};
    shulib::localization::ComplementaryFusion fusion;
    shulib::localization::Localizer loc{clock, imu, odom, fusion};
    shulib::diag::FaultLatch latch{sink, clock};
    shulib::diag::HealthMonitor health{latch};
    const shulib::kinematics::TankKinematics kin{Length{12.0}};  // TWO wheels

    Fakes<4> f;  // four raw motors: two per side, no groups
    auto ctxOf = [&](std::span<IMotor* const> motors) {
        return shulib::chassis::RobotContext{{.clock = &clock,
                                              .driveMotors = motors,
                                              .imu = &imu,
                                              .gps = &gps,
                                              .battery = &battery,
                                              .telemetry = &sink,
                                              .tags = &tags,
                                              .vision = &vision}};
    };
    auto depsOf = [&](shulib::chassis::RobotContext& ctx) {
        return shulib::motion::MotionDeps{.ctx = &ctx,
                                          .localizer = &loc,
                                          .kinematics = &kin,
                                          .faults = &latch,
                                          .health = &health};
    };

    // Four motors on a two-wheel kinematics: REFUSED, and the message says what to do.
    shulib::chassis::RobotContext four = ctxOf(f.span());
    std::string what;
    try {
        depsOf(four).validate();
    } catch (const PreconditionError& e) {
        what = e.what();
    }
    CHECK(what.find("MotorGroup") != std::string::npos);
    CHECK(what.find("EQUAL") != std::string::npos);

    // Three motors: also refused (it used to be accepted by `>=` too).
    std::array<IMotor*, 3> three{&f.motors[0], &f.motors[1], &f.motors[2]};
    shulib::chassis::RobotContext c3 = ctxOf(three);
    CHECK_THROWS_AS(depsOf(c3).validate(), PreconditionError);

    // One motor: refused (fewer than wheels indexes past the span).
    std::array<IMotor*, 1> one{&f.motors[0]};
    shulib::chassis::RobotContext c1 = ctxOf(one);
    CHECK_THROWS_AS(depsOf(c1).validate(), PreconditionError);

    // The FIX the message names: two groups of two, one per wheel — accepted.
    std::array<IMotor*, 2> leftM{&f.motors[0], &f.motors[1]};
    std::array<IMotor*, 2> rightM{&f.motors[2], &f.motors[3]};
    MotorGroup left{std::span<IMotor* const>{leftM}};
    MotorGroup right{std::span<IMotor* const>{rightM}};
    std::array<IMotor*, 2> wheels{&left, &right};
    shulib::chassis::RobotContext grouped = ctxOf(wheels);
    std::array<MotorGroup*, 2> groups{&left, &right};
    shulib::motion::MotionDeps ok = depsOf(grouped);
    ok.motorGroups = groups;
    CHECK_NOTHROW(ok.validate());

    // A null group entry is a contract breach, not a later null dereference.
    std::array<MotorGroup*, 2> withNull{&left, nullptr};
    ok.motorGroups = withNull;
    CHECK_THROWS_AS(ok.validate(), PreconditionError);
}

// ═══ 6. Current and temperature aggregation ═════════════════════════════════════════
// Bug caught: a thermal blind spot (a mean temperature hides the one member about to
// throttle) and a threshold-semantics change (a summed current would trip a per-motor
// stall threshold at N × the real draw).
TEST_CASE("MotorGroup 6: temperature() is the MAX; current() the MEAN per member; sum exposed") {
    Fakes<3> f;
    MotorGroup g{f.span()};
    f.motors[0].setTemperature(30.0);
    f.motors[1].setTemperature(55.0);  // the one about to throttle
    f.motors[2].setTemperature(41.0);
    CHECK(g.temperature() == 55.0);   // a mean would read 42.0 and hide it
    f.motors[0].setTemperature(61.5);
    CHECK(g.temperature() == 61.5);

    f.motors[0].setCurrent(Current{1.0});
    f.motors[1].setCurrent(Current{2.0});
    f.motors[2].setCurrent(Current{3.0});
    CHECK(g.current().value() == 2.0);       // per member: a member at 2 A reads as 2 A
    CHECK(g.totalCurrent().value() == 6.0);  // the side's draw, outside the IMotor surface

    // N = 1: the bare motor's numbers, bit for bit (no arithmetic can leak in).
    Fakes<1> one;
    MotorGroup g1{one.span()};
    one.motors[0].setCurrent(Current{2.4});
    one.motors[0].setTemperature(47.3);
    one.motors[0].setPosition(AngleDim{0.1 + 0.2});
    one.motors[0].setVelocity(AngularVelocity{-17.123456789});
    CHECK(g1.current().value() == 2.4);
    CHECK(g1.totalCurrent().value() == 2.4);
    CHECK(g1.temperature() == 47.3);
    CHECK(g1.position().value() == 0.1 + 0.2);
    CHECK(g1.velocity().value() == -17.123456789);
}

// ═══ 16. The disagreement observable — port 18's signature is tolerated, a fight is not ═══
// Bug caught: a threshold that flags a member merely SLOW (robot two's port 18 travels
// ~20 % short of its side-mates on every push — a fact, A4 register HA-130, not a fight;
// the floor that tolerates it is HA-133) and would cut a
// healthy drive on every run; or a detector that no longer sees a member turning AGAINST
// the command (the coupled-train hazard the whole detector exists for).
TEST_CASE("MotorGroup 16: a ~20 % slow member is NOT a disagreement; the same member opposite IS") {
    Fakes<5> f;
    MotorGroup g{f.span()};
    auto setVel = [&](double mates, double member3) {
        for (std::size_t i = 0; i < 5; ++i) {
            f.motors[i].setVelocity(AngularVelocity{i == 3 ? member3 : mates});
        }
    };
    const int persist = g.thresholds().persistTicks;
    REQUIRE(persist == 25);  // the drive program's window — one set of numbers

    // At rest, commanded nothing: nothing is evaluated, nothing is flagged.
    g.setVoltage(Voltage{0.0});
    setVel(0.0, 0.0);
    for (int t = 0; t < 3 * persist; ++t) {
        const auto& v = g.evaluateDisagreement();
        REQUIRE_FALSE(v.evaluated);
        REQUIRE(v.disagreeingMask == 0);
    }
    CHECK(g.disagreeingMembers() == 0);

    // Under command, member 3 turning 20 % SLOWER than its mates: evaluated, NOT flagged,
    // for far longer than the persistence window.
    g.setVoltage(Voltage{6.0});
    setVel(30.0, 24.0);
    for (int t = 0; t < 4 * persist; ++t) {
        const auto& v = g.evaluateDisagreement();
        REQUIRE(v.evaluated);
        REQUIRE(v.fastestRadS == 30.0);
        REQUIRE(v.disagreeingMask == 0);
        REQUIRE(v.persistedMask == 0);
    }
    CHECK(g.disagreeingMembers() == 0);
    CHECK(g.disagreeingMask() == 0);

    // Reverse command, same 20 % shortfall: still not a fight.
    g.setVoltage(Voltage{-6.0});
    setVel(-30.0, -24.0);
    for (int t = 0; t < 4 * persist; ++t) {
        REQUIRE(g.evaluateDisagreement().disagreeingMask == 0);
    }

    // The SAME member turning the OPPOSITE way to its mates: disagreeing at once, PERSISTED
    // on the 25th consecutive tick and not the 24th, counted and identified by bit.
    g.setVoltage(Voltage{6.0});
    setVel(30.0, -24.0);
    for (int t = 1; t <= persist; ++t) {
        const auto& v = g.evaluateDisagreement();
        REQUIRE(v.disagreeingMask == (std::uint32_t{1} << 3));
        if (t < persist) {
            REQUIRE(v.persistedMask == 0);
            REQUIRE(g.disagreeingMembers() == 0);
        } else {
            REQUIRE(v.persistedMask == (std::uint32_t{1} << 3));
            REQUIRE(g.disagreeingMembers() == 1);
            REQUIRE(g.disagreeingMask() == (std::uint32_t{1} << 3));
        }
    }
    // Its reading no longer belongs in the aggregate either — the median ignores it.
    CHECK(g.velocity().value() == 30.0);

    // Recovery: the member agrees again → the streak resets and the count clears.
    setVel(30.0, 29.0);
    (void)g.evaluateDisagreement();
    CHECK(g.disagreeingMembers() == 0);
    CHECK(g.disagreeingMask() == 0);

    // A frozen member (0 rad/s while its mates turn) IS a disagreement — the dead-port
    // signature the median hides from odometry and the monitor must still report.
    setVel(30.0, 0.0);
    for (int t = 0; t < persist; ++t) {
        (void)g.evaluateDisagreement();
    }
    CHECK(g.disagreeingMembers() == 1);
    g.resetDisagreement();
    CHECK(g.disagreeingMembers() == 0);
    CHECK(g.lastVerdict().persistedMask == 0);
}
