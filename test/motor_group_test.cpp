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

TEST_CASE("MotorGroup: construction refuses an empty span, too many, and an ALL-ABSENT group") {
    // CHANGED AT R3d PART 2 (ruling P4). This case used to require that a nullptr member be
    // refused; a nullptr is now an ABSENT member — a drive port found dead at boot — and the
    // contract is "at least ONE present" rather than "every member present". The refusals that
    // remain are the ones that are still real: nothing at all, more members than the brain has
    // ports, and a group of nothing but holes (which has no reading it could report).
    std::array<IMotor*, 0> none{};
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{none}}), PreconditionError);
    FakeMotor a;
    std::array<IMotor*, MotorGroup::kMaxMembers + 1> tooMany{};
    tooMany.fill(&a);
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{tooMany}}), PreconditionError);
    std::array<IMotor*, 3> allAbsent{nullptr, nullptr, nullptr};
    CHECK_THROWS_AS((MotorGroup{std::span<IMotor* const>{allAbsent}}), PreconditionError);

    // A hole beside a present member CONSTRUCTS, and says so.
    std::array<IMotor*, 3> withHole{&a, nullptr, &a};
    MotorGroup holed{std::span<IMotor* const>{withHole}};
    CHECK(holed.memberCount() == 3);   // slots, so a panel's indices do not shift
    CHECK(holed.presentCount() == 2);
    CHECK(holed.absentMask() == 0b010u);
    CHECK(holed.isPresent(0));
    CHECK_FALSE(holed.isPresent(1));
    CHECK_FALSE(holed.isPresent(9));   // out of range is not present, never out of bounds
    CHECK_THROWS_AS((void)holed.member(1), PreconditionError);  // there is no object to hand back

    std::array<IMotor*, 1> one{&a};
    MotorGroup g{std::span<IMotor* const>{one}};
    CHECK(&g.member(0) == &a);
    CHECK(g.presentCount() == 1);
    CHECK(g.absentMask() == 0u);
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

// ═══ 8. THE ABSENT MEMBER (chunk R3d Part 2, ruling P4) ═════════════════════════════
// Bug caught: a hole in the members span treated as a member. Every one of these is a way a
// dead drive port would silently corrupt a side's numbers instead of being dropped from them:
// a command sent to nothing (a crash), a hole counted in the median (the 1/N drift this class
// exists to avoid, arriving through the other door), a hole counted in the per-member current
// mean (a side with a dead port reading cooler than it is), a hole reported PRESENT to the
// coupled-side monitor (a dead port flagged as a FIGHTING one, and a real fight hidden behind
// it).
//
// Mutations these must fail under: count the absent member in the median; report it present to
// the monitor; divide the current by memberCount() instead of presentCount().
TEST_CASE("MotorGroup 8: an ABSENT member is skipped, excluded and reported absent") {
    // Five SLOTS, four present: slot 2 is a hole. The fifth fake exists but is NOT in the span
    // — it is the evidence that a skipped member is never touched.
    std::array<FakeMotor, 5> motors{};
    std::array<IMotor*, 5> ptrs{&motors[0], &motors[1], nullptr, &motors[3], &motors[4]};
    MotorGroup g{std::span<IMotor* const>{ptrs}};

    CHECK(g.memberCount() == 5);
    CHECK(g.presentCount() == 4);
    CHECK(g.absentMask() == 0b00100u);

    SUBCASE("fan-out skips the hole, and never touches what is not in the span") {
        g.setVoltage(Voltage{7.25});
        g.setBrakeMode(BrakeMode::Brake);
        for (const std::size_t i : {std::size_t{0}, std::size_t{1}, std::size_t{3},
                                    std::size_t{4}}) {
            CHECK(motors[i].commandedVoltage().value() == 7.25);
            CHECK(motors[i].brakeMode() == BrakeMode::Brake);
        }
        // The motor behind the hole was never in the span and was never commanded: its state is
        // the FakeMotor default, untouched.
        CHECK(motors[2].commandedVoltage().value() == 0.0);
        CHECK(motors[2].brakeMode() == BrakeMode::Coast);
        CHECK(g.commandedVoltage().value() == 7.25);
    }

    SUBCASE("the median is over the PRESENT members — exactly the 4-member median") {
        // Deliberately asymmetric readings, and a WILD value on the motor behind the hole: if
        // the hole were read (as 0, or as that motor), the median would move.
        const double p[5] = {1.0, 2.0, -999.0, 3.0, 4.0};
        for (std::size_t i = 0; i < 5; ++i) {
            motors[i].setPosition(AngleDim{p[i]});
            motors[i].setVelocity(AngularVelocity{10.0 * p[i]});
        }
        // The four PRESENT readings are 1, 2, 3, 4 — median = 2.5, exactly.
        CHECK(g.position().value() == 2.5);
        CHECK(g.velocity().value() == 25.0);

        // And it equals a group built over exactly those four, bit for bit — the property the
        // hole must not change.
        std::array<IMotor*, 4> four{&motors[0], &motors[1], &motors[3], &motors[4]};
        MotorGroup dense{std::span<IMotor* const>{four}};
        CHECK(g.position().value() == dense.position().value());
        CHECK(g.velocity().value() == dense.velocity().value());
    }

    SUBCASE("current() is per PRESENT member, and temperature() is the max of the present") {
        const double amps[5] = {1.0, 2.0, 100.0, 3.0, 4.0};
        const double temp[5] = {30.0, 31.0, 999.0, 32.0, 33.0};
        for (std::size_t i = 0; i < 5; ++i) {
            motors[i].setCurrent(Current{amps[i]});
            motors[i].setTemperature(temp[i]);
        }
        CHECK(g.totalCurrent().value() == 10.0);         // 1 + 2 + 3 + 4
        CHECK(g.current().value() == 10.0 / 4.0);        // per PRESENT member, not per SLOT
        CHECK(g.current().value() != 10.0 / 5.0);        // the mutation this pins
        CHECK(g.temperature() == 33.0);                  // the hole's 999 is not a member
    }

    SUBCASE("brakeMode() reads a PRESENT member even when slot 0 is the hole") {
        std::array<IMotor*, 3> holeFirst{nullptr, &motors[1], &motors[2]};
        MotorGroup h{std::span<IMotor* const>{holeFirst}};
        motors[1].setBrakeMode(BrakeMode::Hold);
        CHECK(h.brakeMode() == BrakeMode::Hold);   // NOT a read through a null slot 0
        CHECK(h.presentCount() == 2);
        CHECK(h.absentMask() == 0b001u);
        motors[1].setTemperature(41.0);
        motors[2].setTemperature(44.0);
        CHECK(h.temperature() == 44.0);
    }

    SUBCASE("the monitor is told present = FALSE, and never flags the hole") {
        // The side is commanded hard and every PRESENT member turns together: a healthy side
        // with a dead port. The hole must not appear in disagreeingMask() — a dead port
        // reported as a fighting one is a driver told the wrong thing, and it would hide a
        // real fight behind it.
        g.setVoltage(Voltage{8.0});
        for (std::size_t i = 0; i < 5; ++i) {
            motors[i].setVelocity(AngularVelocity{20.0});
        }
        for (int t = 0; t < 40; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMembers() == 0);
        CHECK(g.disagreeingMask() == 0u);
        CHECK(g.lastVerdict().presentCount == 4);   // the monitor counted four, not five

        // Now make ONE present member fight. It — and only it — is flagged, and the hole's bit
        // stays clear.
        motors[4].setVelocity(AngularVelocity{-20.0});
        for (int t = 0; t < 40; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMembers() == 1);
        CHECK(g.disagreeingMask() == (std::uint32_t{1} << 4));
        CHECK((g.disagreeingMask() & g.absentMask()) == 0u);
    }
}

TEST_CASE("MotorGroup 8b: an ALL-PRESENT group is bit-identical to what it was before holes existed") {
    // The local half of the bit-identity pin (the whole pre-existing suite is the other half).
    // Every read below is over "the present members" now; with nothing absent that must be the
    // SAME NUMBER, not an approximation of it.
    Fakes<5> f;
    MotorGroup g{f.span()};
    CHECK(g.presentCount() == 5);
    CHECK(g.absentMask() == 0u);
    const double p[5] = {1.5, -2.25, 0.125, 7.0, -0.5};
    for (std::size_t i = 0; i < 5; ++i) {
        f.motors[i].setPosition(AngleDim{p[i]});
        f.motors[i].setVelocity(AngularVelocity{p[i] * 3.0});
        f.motors[i].setCurrent(Current{std::abs(p[i])});
        f.motors[i].setTemperature(20.0 + p[i]);
    }
    CHECK(g.position().value() == 0.125);        // the middle of the five, no arithmetic at all
    CHECK(g.velocity().value() == 0.375);
    CHECK(g.current().value() == (1.5 + 2.25 + 0.125 + 7.0 + 0.5) / 5.0);
    CHECK(g.totalCurrent().value() == 1.5 + 2.25 + 0.125 + 7.0 + 0.5);
    CHECK(g.temperature() == 27.0);
    CHECK(&g.member(0) == &f.motors[0]);
    CHECK(&g.member(4) == &f.motors[4]);
}

// ═══ P2-10. RUNTIME ABSENCE — markAbsent (chunk T4 Part 2) ══════════════════════════
// Bug caught: a member marked dead mid-run that still drags the median, still cools the per-member
// current, still reaches the fight monitor (a dead port read as a FIGHT, which now CUTS the drive),
// or — the other way — one that stops being COMMANDED (the old program's rule: a member whose reads
// failed while its motor still turns keeps pulling). And a mark that THROWS mid-match, or that takes
// a side's last member (a group with nothing to report).
//
// Mutations these must fail under: markAbsent leaves the member in the median; leaves it in the
// monitor; stops commanding it; allows the last member; member(i) throwing for a marked member.
TEST_CASE("T4 P2-10: markAbsent — out of every read and the monitor, STILL commanded, refused "
          "without a throw out of range / twice / on a hole / on the last member") {
    std::array<FakeMotor, 5> motors{};
    std::array<IMotor*, 5> ptrs{&motors[0], &motors[1], nullptr, &motors[3], &motors[4]};
    const int labels[5] = {11, 12, 13, 14, 15};
    MotorGroup g{std::span<IMotor* const>{ptrs}, {}, std::span<const int>{labels}};
    const double p[5] = {1.0, 2.0, -999.0, 3.0, 50.0};  // member 4 will be the dead one
    for (std::size_t i = 0; i < 5; ++i) {
        motors[i].setPosition(AngleDim{p[i]});
        motors[i].setVelocity(AngularVelocity{10.0 * p[i]});
        motors[i].setCurrent(Current{i == 4 ? 9.0 : 1.0});
        motors[i].setTemperature(i == 4 ? 80.0 : 30.0 + static_cast<double>(i));
    }
    REQUIRE(g.presentCount() == 4);
    REQUIRE(g.position().value() == 2.5);  // median of 1, 2, 3, 50

    REQUIRE(g.markAbsent(4));
    CHECK(g.presentCount() == 3);
    CHECK_FALSE(g.isPresent(4));
    CHECK(g.runtimeAbsentMask() == (std::uint32_t{1} << 4));
    CHECK(g.absentMask() == 0b00100u);  // the BOOT holes are unchanged by a runtime mark
    CHECK(g.memberCount() == 5);

    SUBCASE("out of every read: the median, the mean current, the sum, the max temperature") {
        CHECK(g.position().value() == 2.0);   // median of 1, 2, 3 — the 50 is gone
        CHECK(g.velocity().value() == 20.0);
        CHECK(g.current().value() == 1.0);    // (1 + 1 + 1) / 3, not (… + 9) / 4
        CHECK(g.totalCurrent().value() == 3.0);
        CHECK(g.temperature() == 33.0);       // the 80 °C of the dead member is not a member's
        // And identical to a group built over the three survivors.
        std::array<IMotor*, 3> three{&motors[0], &motors[1], &motors[3]};
        MotorGroup dense{std::span<IMotor* const>{three}};
        CHECK(g.position().value() == dense.position().value());
        CHECK(g.velocity().value() == dense.velocity().value());
        CHECK(g.current().value() == dense.current().value());
    }

    SUBCASE("STILL commanded — voltage and brake mode both reach the marked member") {
        g.setVoltage(Voltage{-6.5});
        g.setBrakeMode(BrakeMode::Hold);
        CHECK(motors[4].commandedVoltage().value() == -6.5);
        CHECK(motors[4].brakeMode() == BrakeMode::Hold);
        CHECK(motors[2].commandedVoltage().value() == 0.0);  // the BOOT hole is still skipped
    }

    SUBCASE("the monitor is told present = FALSE: its stale reading can neither fight nor stall") {
        g.setVoltage(Voltage{8.0});
        for (const std::size_t i : {std::size_t{0}, std::size_t{1}, std::size_t{3}}) {
            motors[i].setVelocity(AngularVelocity{20.0});
        }
        motors[4].setVelocity(AngularVelocity{-20.0});  // would be a fight if it were fed
        for (int t = 0; t < 40; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMembers() == 0);
        CHECK(g.lastVerdict().presentCount == 3);
        motors[4].setVelocity(AngularVelocity{0.0});  // …or a stall
        for (int t = 0; t < 40; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMembers() == 0);
    }

    SUBCASE("member(i) RETURNS a marked member (it exists, it is commanded) and still throws for a "
            "boot hole") {
        CHECK(&g.member(4) == &motors[4]);
        CHECK_THROWS_AS((void)g.member(2), PreconditionError);
        CHECK_THROWS_AS((void)g.member(5), PreconditionError);
    }

    SUBCASE("REFUSED, with nothing changed and no throw: out of range, a second mark, a boot hole, "
            "and the LAST present member") {
        CHECK_FALSE(g.markAbsent(5));
        CHECK_FALSE(g.markAbsent(99));
        CHECK_FALSE(g.markAbsent(4));  // already marked
        CHECK_FALSE(g.markAbsent(2));  // a boot hole is not a member to mark
        CHECK(g.presentCount() == 3);
        CHECK(g.runtimeAbsentMask() == (std::uint32_t{1} << 4));
        REQUIRE(g.markAbsent(0));
        REQUIRE(g.markAbsent(1));
        REQUIRE(g.presentCount() == 1);
        CHECK_FALSE(g.markAbsent(3));  // the last one answers: a group must have a reading
        CHECK(g.presentCount() == 1);
        CHECK(g.isPresent(3));
        CHECK(g.position().value() == 3.0);
        // first() skips holes AND marks: give every member its own mode, so the read-back can only
        // be member 3's (the review of 2026-09-30: with every mode left at the default this check
        // could not fail).
        motors[0].setBrakeMode(BrakeMode::Hold);
        motors[1].setBrakeMode(BrakeMode::Brake);
        motors[3].setBrakeMode(BrakeMode::Coast);
        motors[4].setBrakeMode(BrakeMode::Hold);
        CHECK(g.brakeMode() == BrakeMode::Coast);
    }

    SUBCASE("the FIRST member marked: temperature() and brakeMode() read the first PRESENT member, "
            "not the marked one's held readings") {
        // BUG CAUGHT (the review of 2026-09-30, G1): first() returning a runtime-MARKED member —
        // its held temperature would seed the group's (and MOTOR_OVER_TEMP reads the group's,
        // motion.hpp's health tick), and brakeMode() would read back a dead port's last mode.
        std::array<FakeMotor, 4> m{};
        std::array<IMotor*, 4> q{&m[0], &m[1], &m[2], &m[3]};
        MotorGroup first{std::span<IMotor* const>{q}};
        for (std::size_t i = 0; i < 4; ++i) {
            m[i].setTemperature(i == 0 ? 80.0 : 30.0 + static_cast<double>(i));
        }
        first.setBrakeMode(BrakeMode::Coast);
        m[0].setBrakeMode(BrakeMode::Hold);  // the dead port's last mode
        REQUIRE(first.markAbsent(0));
        CHECK(first.temperature() == 33.0);  // not the marked member's 80 °C
        CHECK(first.brakeMode() == BrakeMode::Coast);
    }

    SUBCASE("labels: one port number per slot, 0 out of range; none given is 0 everywhere") {
        CHECK(g.label(0) == 11);
        CHECK(g.label(2) == 13);  // a hole still has its port — the log names it
        CHECK(g.label(4) == 15);
        CHECK(g.label(5) == 0);
        Fakes<2> f;
        MotorGroup plain{f.span()};
        CHECK(plain.label(0) == 0);
        CHECK(plain.label(1) == 0);
        const int wrong[1] = {7};
        CHECK_THROWS_AS((MotorGroup{f.span(), {}, std::span<const int>{wrong}}), PreconditionError);
    }
}

// ═══ P2-10. The exclusion mask — setMonitorExcluded (chunk T4 Part 2) ════════════════
// Bug caught: an exclusion that also drops the member from the reads or the command (the mask is
// for the MONITOR only — a FROZEN member is still a member until it is marked), an exclusion that
// does not reach the monitor (a dead member's stale velocity starts a fight streak and cuts), and a
// mask that keeps bits it should drop (a hole's, a marked member's, one past the range).
TEST_CASE("T4 P2-10: setMonitorExcluded — fed present = FALSE to the monitor, and NOTHING else "
          "changes; dropped bits dropped; a zero mask is bit-identical") {
    std::array<FakeMotor, 4> motors{};
    std::array<IMotor*, 4> ptrs{&motors[0], &motors[1], &motors[2], nullptr};
    MotorGroup g{std::span<IMotor* const>{ptrs}};
    const double p[3] = {1.0, 2.0, 40.0};
    for (std::size_t i = 0; i < 3; ++i) {
        motors[i].setPosition(AngleDim{p[i]});
        motors[i].setCurrent(Current{static_cast<double>(i + 1)});
    }
    CHECK(g.monitorExcludedMask() == 0u);  // the default

    g.setMonitorExcluded(0b0100u);
    CHECK(g.monitorExcludedMask() == 0b0100u);
    // Every read and the command are untouched.
    CHECK(g.presentCount() == 3);
    CHECK(g.isPresent(2));
    CHECK(g.position().value() == 2.0);         // median of 1, 2, 40 — the excluded 40 is in it
    CHECK(g.totalCurrent().value() == 6.0);
    g.setVoltage(Voltage{9.0});
    CHECK(motors[2].commandedVoltage().value() == 9.0);

    SUBCASE("…but the monitor is told present = FALSE for it") {
        motors[0].setVelocity(AngularVelocity{20.0});
        motors[1].setVelocity(AngularVelocity{20.0});
        motors[2].setVelocity(AngularVelocity{-20.0});  // a fight, if it were fed
        for (int t = 0; t < 40; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMembers() == 0);
        CHECK(g.lastVerdict().presentCount == 2);
        g.setMonitorExcluded(0u);  // cleared: the next 25 ticks flag it — the mask is the only cause
        for (int t = 0; t < 25; ++t) {
            g.evaluateDisagreement();
        }
        CHECK(g.disagreeingMask() == 0b0100u);
    }

    SUBCASE("bits past the range, on a hole and on a marked member are dropped") {
        g.setMonitorExcluded(0xFFFFFFFFu);
        CHECK(g.monitorExcludedMask() == 0b0111u);  // slot 3 is a hole; nothing past 4 slots
        REQUIRE(g.markAbsent(1));
        CHECK(g.monitorExcludedMask() == 0b0101u);  // a mark clears its bit
        g.setMonitorExcluded(0b0010u);
        CHECK(g.monitorExcludedMask() == 0u);
    }

    SUBCASE("a mask of 0 set explicitly is bit-identical to one never set") {
        Fakes<3> a;
        Fakes<3> b;
        MotorGroup ga{a.span()};
        MotorGroup gb{b.span()};
        gb.setMonitorExcluded(0u);
        const double v[3] = {5.0, 5.5, -0.5};
        for (std::size_t i = 0; i < 3; ++i) {
            a.motors[i].setVelocity(AngularVelocity{v[i]});
            b.motors[i].setVelocity(AngularVelocity{v[i]});
        }
        ga.setVoltage(Voltage{6.0});
        gb.setVoltage(Voltage{6.0});
        for (int t = 0; t < 30; ++t) {
            const auto va = ga.evaluateDisagreement();
            const auto vb = gb.evaluateDisagreement();
            CHECK(va.persistedMask == vb.persistedMask);
            CHECK(va.disagreeingMask == vb.disagreeingMask);
            CHECK(va.fastestRadS == vb.fastestRadS);
            CHECK(va.presentCount == vb.presentCount);
        }
        CHECK(ga.disagreeingMask() != 0u);  // the run did flag something (member 2 is stalled)
    }
}
