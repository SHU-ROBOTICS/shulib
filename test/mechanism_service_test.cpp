// The mechanism service's seams, its claimant, its lookups and its mode changes (chunk MECH-A2, the
// MECH brief's §1.6 and §3.2).
//
// Every case tries to break one rule the rest of the core will lean on:
//   * EXACTLY ONE tick per control tick, whichever pacer a driver loop is handed (P8) — a service
//     that ticked twice runs every behaviour at double rate; one that ticked zero times freezes them;
//   * the service ticks BEFORE the world advances (the pacer seam's order);
//   * the slot IS the claimant: the guard's reach makes a running step inert, safe, released,
//     CANCELLED, and the NEXT tick commands nothing (the "half-safe" failure hal/mechanism.hpp
//     measured); a claim another owner holds is never commanded, and a claim TAKEN OVER mid-step ends
//     the step and the commanding;
//   * lookups by NAME and KIND (P12), never by address;
//   * the three mode changes — and the D1 handover: a driver task PROS deleted leaves its intake at
//     12 V into the auton unless enterAutonomous() drops it.
// The power and pneumatic behaviours, the evidence rule and the guard's deadline cut have their own
// files (mechanism_service_power_test.cpp, ..._pneumatic_test.cpp, ..._evidence_test.cpp,
// run_guard_mechanism_cut_test.cpp).

#include "doctest.h"

#include <algorithm>
#include <array>
#include <functional>
#include <span>
#include <stdexcept>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_optical.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace rx = mech_fixture::robot_x;
namespace r2 = mech_fixture::robot_two;
using mech::MechanismOutcome;
using mech::Until;
using mech_rig::kNoObjectIn;
using mech_rig::Rig;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using namespace shulib::units::literals;

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

namespace lists {
using namespace shulib::mechanism::words;
/// Robot x's intake and its three-position hood: the power and pneumatic half of robot x.
inline constexpr MechanismTable kIntakeHood =
    mechanisms(wire(rx::intake, motors(5), pieceSensor(9)), wire(rx::hood, lines('C', 'D')));
/// A different robot's intake: the SAME name and kind as robot x's, another declaration.
inline constexpr Power otherIntake{.name = "intake", .motors = {Forward}, .cartridge = Red, .safe = Brake};
/// A pneumatic that happens to be named "intake".
inline constexpr Pneumatic pneumaticNamedIntake{
    .name = "intake", .positions = {{"in", {Low}}, {"out", {High}}}, .start = "in", .safe = "in", .moveTime = 0.2_s};
/// Another robot's hood whose positions are in another ORDER: its "up" is index 0.
inline constexpr Pneumatic reorderedHood{.name = "hood",
                                         .positions = {{"up", {High, High}}, {"down", {Low, Low}}, {"middle", {High, Low}}},
                                         .start = "down",
                                         .safe = "down",
                                         .moveTime = 0.3_s};
}  // namespace lists

/// An IMotor over a FakeMotor whose setVoltage() can run a hook once — to re-enter the service from
/// inside a tick, or to THROW out of one (the stand-in for a task PROS deleted mid-tick).
struct HookMotor final : hal::IMotor {
    hal::fake::FakeMotor inner;
    std::function<void()> once;
    void setVoltage(Voltage v) override {
        inner.setVoltage(v);
        if (once) {
            const std::function<void()> f = once;
            once = nullptr;
            f();
        }
    }
    [[nodiscard]] Voltage commandedVoltage() const override { return inner.commandedVoltage(); }
    void setBrakeMode(hal::BrakeMode m) override { inner.setBrakeMode(m); }
    [[nodiscard]] hal::BrakeMode brakeMode() const override { return inner.brakeMode(); }
    [[nodiscard]] shulib::units::AngleDim position() const override { return inner.position(); }
    [[nodiscard]] shulib::units::AngularVelocity velocity() const override { return inner.velocity(); }
    [[nodiscard]] shulib::units::Current current() const override { return inner.current(); }
    [[nodiscard]] double temperature() const override { return inner.temperature(); }
};

/// A spy claimant: counts its cancel() calls (an operation the service must NEVER cancel).
struct SpyClaimant final : hal::ICancellable {
    int cancels = 0;
    void cancel() override { ++cancels; }
};

/// A service bound BY HAND over one power mechanism on a HookMotor (bind(specs, devices)). The
/// devices are declared BEFORE the service: they outlive it (bind()'s rule; the destructor's reach).
struct HandPower {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    HookMotor motor;
    std::array<hal::IMotor*, 1> ms{&motor};
    hal::MotorMechanism group{ms, hal::BrakeMode::Hold, "claw pivot 6"};
    mech::MechanismSpec spec = r2::clawPivot6.spec();
    mech_rig::World world;
    mech::MechanismService svc{world};
    HandPower() {
        world.clock = &clock;
        const mech::Devices d{.motors = &group};
        svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
                 sink);
    }
};

}  // namespace

// ═══ THE TWO SEAMS ═══════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 seams P8: exactly ONE service tick per driver tick -- the loop handed the raw pacer, the service, or guard{service}") {
    for (int wiring = 0; wiring < 3; ++wiring) {
        CAPTURE(wiring);
        Rig r{r2::kRobotTwoMechanisms};
        shulib::sequence::RunGuard guard{r.svc};
        shulib::motion::ITickPacer* loopPacer = &r.world;
        if (wiring == 1) {
            loopPacer = &r.svc;
        } else if (wiring == 2) {
            loopPacer = &guard;
        }
        for (int i = 0; i < 600; ++i) {
            r.svc.tickFromDriverLoop();  // the driver loop's stated place
            loopPacer->pace();           // then whatever pacer the loop was handed
        }
        CHECK(r.svc.ticks() == 600);
        CHECK(r.world.paces == 600);
    }
}

TEST_CASE("MECH-A2 seams: the pacer seam ticks on EVERY pace, and only one driver-loop tick is skipped after tickFromDriverLoop()") {
    Rig r{r2::kRobotTwoMechanisms};
    r.pace(10);
    CHECK(r.svc.ticks() == 10);
    CHECK(r.world.paces == 10);
    r.svc.tickFromDriverLoop();
    CHECK(r.svc.ticks() == 11);
    r.svc.pace();  // follows a driver tick: advances the world only
    CHECK(r.svc.ticks() == 11);
    CHECK(r.world.paces == 11);
    r.svc.pace();  // the next one ticks again
    CHECK(r.svc.ticks() == 12);
    // guard{service}: the guard's pace (inert, no run live) reaches the service, which ticks
    shulib::sequence::RunGuard guard{r.svc};
    for (int i = 0; i < 5; ++i) {
        guard.pace();
    }
    CHECK(r.svc.ticks() == 17);
    CHECK(r.world.paces == 17);
}

TEST_CASE("MECH-A2 seams: UNBOUND, pace() forwards and ticks nothing; tickFromDriverLoop() ticks nothing; the accessors refuse") {
    shulib::hal::fake::FakeClock clock;
    mech_rig::World world;
    world.clock = &clock;
    mech::MechanismService svc{world};
    CHECK_FALSE(svc.bound());
    for (int i = 0; i < 5; ++i) {
        svc.pace();
    }
    svc.tickFromDriverLoop();
    svc.pace();
    CHECK(svc.ticks() == 0);
    CHECK(world.paces == 6);
    CHECK(svc.size() == 0U);
    CHECK(svc.mechanisms().empty());
    CHECK(svc.slotPtr(0) == nullptr);
    CHECK(svc.slotFor("claw") == nullptr);
    CHECK(preconditionOf([&] { (void)svc.clock(); }).find("not bound") != std::string::npos);
    CHECK(preconditionOf([&] { (void)svc.faults(); }).find("not bound") != std::string::npos);
    CHECK(preconditionOf([&] { (void)svc.telemetry(); }).find("not bound") != std::string::npos);
    CHECK(preconditionOf([&] { (void)svc.slot(0); }).find("no mechanism at this index") != std::string::npos);
    // the mode changes on an unbound service are harmless
    svc.enterDriverControl();
    CHECK(svc.enterAutonomous() == 0);
    svc.leaveDriverControl();
}

TEST_CASE("MECH-A2 seams: EVERY mode change forgets a driver tick whose pace never came -- the next pace() ticks (exactly once per tick across a mode change)") {
    // The driver loop's last tickFromDriverLoop(), then PROS deletes the task before its pace(): the
    // first pace of whatever comes next must tick, or every mechanism misses a tick at the handover.
    for (int change = 0; change < 3; ++change) {
        CAPTURE(change);
        Rig r{r2::kRobotTwoMechanisms};
        r.svc.tickFromDriverLoop();  // no pace() follows: the task is gone
        if (change == 0) {
            (void)r.svc.enterAutonomous();
        } else if (change == 1) {
            r.svc.enterDriverControl();
        } else {
            r.svc.leaveDriverControl();
        }
        const long before = r.svc.ticks();
        r.svc.pace();
        CHECK(r.svc.ticks() == before + 1);
        CHECK(r.world.paces == 1);
    }
}

TEST_CASE("MECH-A2 seams: the service ticks BEFORE the world advances -- the world's first step already sees the command") {
    Rig r{r2::kRobotTwoMechanisms};
    double seenByWorld = -1.0;
    hal::fake::FakeMotor& m6 = r.devices.fakeMotor(6);
    r.world.steps.push_back([&](double) {
        if (seenByWorld < 0.0) {
            seenByWorld = m6.commandedVoltage().value();
        }
    });
    r.svc.power(0).run(12_volt, Until::time(1_s), 2_s);
    r.svc.pace();
    CHECK(seenByWorld == doctest::Approx(12.0));
}

TEST_CASE("MECH-A2 seams: a re-entered tick is a precondition; a tick that never finished leaves the flag until a mode change clears it") {
    SUBCASE("re-entered from inside a tick") {
        HandPower h;
        h.svc.power(0).drive(6_volt);
        h.motor.once = [&] { h.svc.tickFromDriverLoop(); };
        const std::string msg = preconditionOf([&] { h.svc.pace(); });
        CHECK(msg.find("a tick began inside another") != std::string::npos);
    }
    SUBCASE("a throw out of a tick (the stand-in for a task deleted mid-tick), cleared by enterDriverControl()") {
        HandPower h;
        h.svc.power(0).drive(6_volt);
        h.motor.once = [] { throw std::runtime_error("task deleted"); };
        CHECK_THROWS_AS(h.svc.pace(), std::runtime_error);
        // the flag is stuck: every later tick refuses, naming why
        CHECK(preconditionOf([&] { h.svc.pace(); }).find("never finished") != std::string::npos);
        h.svc.enterDriverControl();
        CHECK_NOTHROW(h.svc.pace());
    }
    SUBCASE("... and cleared by enterAutonomous()") {
        HandPower h;
        h.svc.power(0).drive(6_volt);
        h.motor.once = [] { throw std::runtime_error("task deleted"); };
        CHECK_THROWS_AS(h.svc.tickFromDriverLoop(), std::runtime_error);
        CHECK_FALSE(preconditionOf([&] { h.svc.pace(); }).empty());
        (void)h.svc.enterAutonomous();
        const long before = h.svc.ticks();
        CHECK_NOTHROW(h.svc.pace());
        CHECK(h.svc.ticks() == before + 1);
    }
}

// ═══ BIND ════════════════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 bind: robot two over its built devices -- three slots in the list's order, the guard's span the built ones") {
    Rig r{r2::kRobotTwoMechanisms};
    CHECK(r.svc.bound());
    REQUIRE(r.svc.size() == 3U);
    CHECK(r.svc.slot(0).kind() == mech::Kind::Power);
    CHECK(std::string{r.svc.slot(0).name()} == "claw pivot 6");
    CHECK(std::string{r.svc.slot(1).name()} == "claw pivot 7");
    CHECK(r.svc.slot(2).kind() == mech::Kind::Pneumatic);
    CHECK(r.svc.slot(2).index() == 2);
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        CHECK(r.svc.slot(i).built());
        CHECK(std::string{r.svc.slot(i).absence()}.empty());
        CHECK_FALSE(r.svc.slot(i).running());
        CHECK(r.svc.slot(i).stepsBegun() == 0U);
        CHECK(r.svc.slot(i).commandSeq() == 0U);
        CHECK_FALSE(r.svc.slot(i).holdsClaim());
    }
    // THE SAME devices the parts built (landmine 20): the guard's span is the parts' built() span
    REQUIRE(r.svc.mechanisms().size() == r.parts.built().size());
    for (std::size_t i = 0; i < 3; ++i) {
        CHECK(r.svc.mechanisms()[i] == r.parts.built()[i]);
    }
    CHECK(&r.svc.slot(0).spec() == &r2::kRobotTwoMechanisms.specs[0]);
    CHECK(preconditionOf([&] { (void)r.svc.slot(3); }).find("no mechanism at this index") != std::string::npos);
    CHECK(preconditionOf([&] { (void)r.svc.pneumatic(0); }).find("not a pneumatic") != std::string::npos);
    CHECK(preconditionOf([&] { (void)r.svc.power(2); }).find("not a power") != std::string::npos);
}

TEST_CASE("MECH-A2 bind: an ABSENT mechanism is a slot that is not built and is not in the guard's span") {
    Rig r{r2::kRobotTwoMechanisms, [](hal::fake::FakeDeviceProvider& d) { d.refusePort(7); }};
    REQUIRE(r.svc.size() == 3U);
    CHECK(r.svc.slot(0).built());
    CHECK_FALSE(r.svc.slot(1).built());
    CHECK_FALSE(std::string{r.svc.slot(1).absence()}.empty());
    REQUIRE(r.svc.mechanisms().size() == 2U);
    CHECK(std::string{r.svc.mechanisms()[0]->name()} == "claw pivot 6");
    CHECK(std::string{r.svc.mechanisms()[1]->name()} == "claw");
}

TEST_CASE("MECH-A2 bind: every precondition, each checked BEFORE anything is stored (a refused bind leaves the service unbound)") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    mech_rig::World world;
    world.clock = &clock;
    hal::fake::FakeMotor m1, m2;
    std::array<hal::IMotor*, 1> one{&m1};
    std::array<hal::IMotor*, 2> two{&m1, &m2};
    hal::MotorMechanism g1{one, hal::BrakeMode::Hold, "claw pivot 6"};
    hal::MotorMechanism g2{two, hal::BrakeMode::Hold, "claw pivot 6"};
    hal::fake::FakeDigitalOut lineA, lineB;
    std::array<hal::IDigitalOut*, 1> l1{&lineA};
    std::array<hal::IDigitalOut*, 2> l2{&lineA, &lineB};
    hal::PneumaticMechanism p1{l1, false, "claw"};
    hal::PneumaticMechanism p2{l2, false, "claw"};
    hal::fake::FakeDistance distance;
    hal::fake::FakeOptical optical;
    hal::fake::FakeRotation rotation;
    hal::fake::FakeReadFaults counter;
    const mech::MechanismSpec pivot = r2::clawPivot6.spec();
    const mech::MechanismSpec claw = r2::kRobotTwoMechanisms.specs[2];
    const mech::MechanismSpec intake = rx::kRobotXMechanisms.specs[3];

    const auto bindOne = [&](const mech::MechanismSpec& s, const mech::Devices& d) {
        mech::MechanismService svc{world};
        const std::string msg = preconditionOf([&] {
            svc.bind(std::span<const mech::MechanismSpec>{&s, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
                     sink);
        });
        CHECK_FALSE(svc.bound());  // nothing stored
        CHECK(svc.size() == 0U);
        return msg;
    };
    // (A POSITION or SPEED mechanism was refused here until MECH-A3 built their behaviours; bind() now
    // takes them — their device preconditions are test/mechanism_position_test.cpp's.)
    // an invalid spec: its own sentence
    mech::MechanismSpec bad = pivot;
    bad.power.maxVolts = 13.0;
    CHECK(bindOne(bad, mech::Devices{.motors = &g1}) == std::string{mech::specProblem(bad)});
    // the actuators
    CHECK(bindOne(pivot, mech::Devices{}).find("needs its motors") != std::string::npos);
    CHECK(bindOne(pivot, mech::Devices{.motors = &g1, .pneumatic = &p1}).find("needs its motors") != std::string::npos);
    CHECK(bindOne(pivot, mech::Devices{.motors = &g2}).find("motor count") != std::string::npos);
    CHECK(bindOne(claw, mech::Devices{.pneumatic = &p2}).find("line count") != std::string::npos);
    CHECK(bindOne(claw, mech::Devices{.motors = &g1}).find("needs its lines") != std::string::npos);
    CHECK(bindOne(pivot, mech::Devices{.motors = &g1, .absence = "port 6: nothing plugged in"})
              .find("ABSENT mechanism's Devices carries no actuator") != std::string::npos);
    CHECK(bindOne(pivot, mech::Devices{.motors = &g1, .absence = nullptr}).find("absence is null") != std::string::npos);
    // the sensors
    CHECK(bindOne(pivot, mech::Devices{.motors = &g1, .angleSensor = &rotation}).find("angle sensor") != std::string::npos);
    CHECK(bindOne(pivot, mech::Devices{.motors = &g1, .pieceDistance = &distance}).find("does not have") != std::string::npos);
    CHECK(bindOne(intake, mech::Devices{.motors = &g1, .pieceOptical = &optical}).find("does not have") != std::string::npos);
    CHECK(bindOne(intake, mech::Devices{.motors = &g1, .pieceDistance = &distance, .pieceOptical = &optical})
              .find("does not have") != std::string::npos);
    CHECK(bindOne(intake, mech::Devices{.motors = &g1, .pieceFaults = &counter}).find("was not built") != std::string::npos);
    CHECK(bindOne(intake, mech::Devices{.motors = &g1, .pieceDistance = &distance, .homeSwitchFaults = &counter})
              .find("home switch") != std::string::npos);
    // ... and a declared piece sensor MISSING this boot is LEGAL: it runs without one (N8)
    mech::MechanismService ok{world};
    const mech::Devices noSensor{.motors = &g1};
    ok.bind(std::span<const mech::MechanismSpec>{&intake, 1}, std::span<const mech::Devices>{&noSensor, 1}, clock, faults,
            sink);
    CHECK(ok.bound());
    CHECK(ok.slot(0).pieceSensorMissing());
    CHECK(ok.slot(0).declaresPieceSensor());
    CHECK_FALSE(ok.slot(0).hasPieceSensor());
}

TEST_CASE("MECH-A2 bind: the list-level preconditions -- twice, sizes, more than 8, two names alike") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    mech_rig::World world;
    world.clock = &clock;
    hal::fake::FakeMotor m1;
    std::array<hal::IMotor*, 1> one{&m1};
    hal::MotorMechanism g1{one, hal::BrakeMode::Hold, "claw pivot 6"};
    const mech::MechanismSpec pivot = r2::clawPivot6.spec();
    const mech::Devices d{.motors = &g1};
    {
        mech::MechanismService svc{world};
        svc.bind(std::span<const mech::MechanismSpec>{&pivot, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
                 sink);
        CHECK(svc.bound());
        CHECK(preconditionOf([&] {
                  svc.bind(std::span<const mech::MechanismSpec>{&pivot, 1}, std::span<const mech::Devices>{&d, 1}, clock,
                           faults, sink);
              }).find("called twice") != std::string::npos);
        CHECK(svc.size() == 1U);
    }
    {
        mech::MechanismService svc{world};
        const std::array<mech::Devices, 2> dd{d, d};
        CHECK(preconditionOf([&] {
                  svc.bind(std::span<const mech::MechanismSpec>{&pivot, 1}, dd, clock, faults, sink);
              }).find("one Devices per spec") != std::string::npos);
    }
    {
        mech::MechanismService svc{world};
        std::array<mech::MechanismSpec, 9> nine{};
        nine.fill(pivot);
        std::array<mech::Devices, 9> nd{};
        nd.fill(d);
        CHECK(preconditionOf([&] { svc.bind(nine, nd, clock, faults, sink); }).find("more than 8") != std::string::npos);
        CHECK_FALSE(svc.bound());
    }
    {
        mech::MechanismService svc{world};
        const std::array<mech::MechanismSpec, 2> twins{pivot, pivot};
        hal::fake::FakeMotor m2;
        std::array<hal::IMotor*, 1> other{&m2};
        hal::MotorMechanism g2{other, hal::BrakeMode::Hold, "claw pivot 6"};
        const std::array<mech::Devices, 2> dd{d, mech::Devices{.motors = &g2}};
        CHECK(preconditionOf([&] { svc.bind(twins, dd, clock, faults, sink); }).find("share a name") != std::string::npos);
        CHECK_FALSE(svc.bound());
    }
}

// ═══ THE CLAIMANT ════════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 claimant: the guard's reach -- a running step goes inert, safe, released, CANCELLED, and the NEXT tick commands nothing") {
    Rig r{lists::kIntakeHood, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(9).setDistance(Length{kNoObjectIn}); }};
    mech::PowerSlot& intake = r.svc.power(0);
    hal::fake::FakeMotor& m = r.devices.fakeMotor(5);
    intake.run(12_volt, Until::sees(), 3_s);
    r.pace(5);
    REQUIRE(m.commandedVoltage().value() == doctest::Approx(12.0));
    hal::IMechanism* mm = r.svc.mechanisms()[0];
    REQUIRE(mm->claimed());
    REQUIRE(mm->claimant() == &intake);
    REQUIRE(intake.holdsClaim());
    // what RunGuard's cancel-all does: the registered claimant's cancel(), then the safe state
    mm->claimant()->cancel();
    mm->applySafeState();
    CHECK_FALSE(intake.running());
    CHECK(intake.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK_FALSE(mm->claimed());
    CHECK(m.commandedVoltage().value() == 0.0);
    CHECK(m.brakeMode() == hal::BrakeMode::Coast);  // the intake's declared safe state
    CHECK(r.lastMech() == "'intake': CANCELLED in 0.05s");
    // THE HALF-SAFE FAILURE: a live behaviour would re-command on its next tick. It must not.
    r.pace(20);
    CHECK(m.commandedVoltage().value() == 0.0);
    CHECK_FALSE(mm->claimed());
    // idempotent
    intake.cancel();
    CHECK(intake.lastStep().outcome == MechanismOutcome::Cancelled);
}

TEST_CASE("MECH-A2 claimant: a claim TAKEN OVER mid-step -- never commanded again; the step ends CANCELLED, said") {
    Rig r{lists::kIntakeHood, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(9).setDistance(Length{kNoObjectIn}); }};
    mech::PowerSlot& intake = r.svc.power(0);
    hal::fake::FakeMotor& m = r.devices.fakeMotor(5);
    intake.run(12_volt, Until::sees(), 3_s);
    r.pace(3);
    hal::IMechanism* mm = r.svc.mechanisms()[0];
    mm->releaseClaim();  // released outside the slot ...
    SpyClaimant spy;
    REQUIRE(mm->tryClaim(spy));  // ... and taken by another owner
    m.setVoltage(Voltage{3.0});  // that owner's command
    r.pace(10);
    CHECK(m.commandedVoltage().value() == doctest::Approx(3.0));  // never overwritten
    CHECK(mm->claimant() == &spy);
    CHECK(spy.cancels == 0);
    CHECK_FALSE(intake.running());
    CHECK(intake.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.lastMech() == "'intake': CANCELLED in 0.03s (its claim was taken by another owner)");
}

TEST_CASE("MECH-A2 claimant: a claim RELEASED outside the slot (no cancel, nobody took it) -- the slot stops commanding AND leaves it safe") {
    // What a T3-era driver loop does at entry to every leftover claim (it releases, never cancels):
    // the slot that was driving the mechanism must neither keep commanding it nor leave it at 12 V.
    Rig r{lists::kIntakeHood, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(9).setDistance(Length{kNoObjectIn}); }};
    mech::PowerSlot& intake = r.svc.power(0);
    hal::fake::FakeMotor& m = r.devices.fakeMotor(5);
    intake.run(12_volt, Until::sees(), 3_s);
    r.pace(4);
    REQUIRE(m.commandedVoltage().value() == doctest::Approx(12.0));
    hal::IMechanism* mm = r.svc.mechanisms()[0];
    mm->releaseClaim();  // released outside the slot, no cancel(), no safe state
    r.pace(1);
    CHECK(m.commandedVoltage().value() == 0.0);              // not left at 12 V by nobody
    CHECK(m.brakeMode() == hal::BrakeMode::Coast);           // its declared safe state
    CHECK_FALSE(mm->claimed());                              // the slot did not take it back
    CHECK(intake.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.lastMech() == "'intake': CANCELLED in 0.04s (its claim was released outside the service)");
    r.pace(20);
    CHECK(m.commandedVoltage().value() == 0.0);
    CHECK_FALSE(mm->claimed());
    // a hold-to-run the same way
    intake.drive(9_volt);
    r.pace(2);
    REQUIRE(m.commandedVoltage().value() == doctest::Approx(9.0));
    mm->releaseClaim();
    r.pace(3);
    CHECK(m.commandedVoltage().value() == 0.0);
    CHECK_FALSE(mm->claimed());
}

TEST_CASE("MECH-A2 claimant (A2-D17): a PNEUMATIC whose claim is RELEASED outside the slot mid-step KEEPS its level -- no mode change makes one safe either") {
    // A stale F1 operation's cancel() releases whatever claim is on the mechanism; a released motor is
    // made safe (A2-D2), but a pneumatic's level is what it holds — a claw must not drop it (HA-157).
    Rig r{r2::kRobotTwoMechanisms};
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    hal::fake::FakeDigitalOut& lineA = r.devices.fakeDigitalOut(1);
    REQUIRE(claw.commandedPosition() == 0);  // "closed": its start AND its safe position (Low)
    claw.set(1);                             // "open" (High), mid-move from here
    r.pace(3);
    REQUIRE(claw.running());
    REQUIRE(lineA.commanded());
    hal::IMechanism* mm = r.svc.mechanisms()[2];
    mm->releaseClaim();  // released outside the slot: no cancel(), no safe state, nobody took it
    r.pace(1);
    CHECK(lineA.commanded());  // the open level KEPT — not driven to its safe "closed"
    CHECK(claw.commandedPosition() == 1);
    CHECK_FALSE(claw.running());
    CHECK(claw.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.lastMech() == "'claw': CANCELLED in 0.03s (its claim was released outside the service)");
    CHECK_FALSE(mm->claimed());  // the slot did not take it back
    r.pace(60);                  // and nothing re-commands it afterwards
    CHECK(lineA.commanded());
    CHECK_FALSE(mm->claimed());
}

TEST_CASE("MECH-A2 claimant: a service DESTROYED mid-behaviour ends what it holds -- safe, released, no claimant left pointing at it") {
    // The devices, the clock, the latch and the sink OUTLIVE the service (bind()'s rule); the service
    // goes out of scope with a step and a hold-to-run mid-flight (a test rig, MECH-A5's tester chain).
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeMotor m6, m7;
    std::array<hal::IMotor*, 1> ms6{&m6};
    std::array<hal::IMotor*, 1> ms7{&m7};
    hal::MotorMechanism pivot6{ms6, hal::BrakeMode::Hold, "claw pivot 6"};
    hal::MotorMechanism pivot7{ms7, hal::BrakeMode::Hold, "claw pivot 7"};
    hal::fake::FakeDigitalOut lineA;
    std::array<hal::IDigitalOut*, 1> lines{&lineA};
    hal::PneumaticMechanism claw{lines, false, "claw"};
    const std::array<mech::MechanismSpec, 3> specs{r2::kRobotTwoMechanisms.specs[0], r2::kRobotTwoMechanisms.specs[1],
                                                    r2::kRobotTwoMechanisms.specs[2]};
    const std::array<mech::Devices, 3> devices{mech::Devices{.motors = &pivot6}, mech::Devices{.motors = &pivot7},
                                               mech::Devices{.pneumatic = &claw}};
    SpyClaimant spy;
    {
        mech_rig::World world;
        world.clock = &clock;
        mech::MechanismService svc{world};
        svc.bind(specs, devices, clock, faults, sink);
        svc.power(0).run(12_volt, Until::time(5_s), 6_s);  // a STEP mid-flight
        svc.power(1).drive(9_volt);                        // a HOLD-TO-RUN mid-flight
        svc.pace();
        REQUIRE(m6.commandedVoltage().value() == doctest::Approx(12.0));
        REQUIRE(m7.commandedVoltage().value() == doctest::Approx(9.0));
        REQUIRE(pivot6.claimant() == &svc.slot(0));
        REQUIRE(pivot7.claimant() == &svc.slot(1));
        REQUIRE(claw.tryClaim(spy));  // ANOTHER owner's claim: not the service's to end
        m6.setBrakeMode(hal::BrakeMode::Coast);  // proof the SAFE state is applied, not only 0 V
        m7.setBrakeMode(hal::BrakeMode::Coast);
    }
    CHECK(m6.commandedVoltage().value() == 0.0);
    CHECK(m7.commandedVoltage().value() == 0.0);
    CHECK(m6.brakeMode() == hal::BrakeMode::Hold);
    CHECK(m7.brakeMode() == hal::BrakeMode::Hold);
    CHECK_FALSE(pivot6.claimed());  // no claimant left pointing into a destroyed slot
    CHECK(pivot6.claimant() == nullptr);
    CHECK_FALSE(pivot7.claimed());
    CHECK(pivot7.claimant() == nullptr);
    CHECK(claw.claimant() == &spy);  // another owner's claim untouched ...
    CHECK(spy.cancels == 0);         // ... and never cancelled
    CHECK(sink.last().message == "'claw pivot 6': CANCELLED in 0.01s (its service was destroyed)");
}

TEST_CASE("MECH-A2 claimant: a device that THROWS while the destroyed service makes it safe -- its claim is STILL released, the next slot still ended, nothing escapes") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    HookMotor m6;  // its safe-state command throws (a broken adapter)
    hal::fake::FakeMotor m7;
    std::array<hal::IMotor*, 1> ms6{&m6};
    std::array<hal::IMotor*, 1> ms7{&m7};
    hal::MotorMechanism pivot6{ms6, hal::BrakeMode::Hold, "claw pivot 6"};
    hal::MotorMechanism pivot7{ms7, hal::BrakeMode::Hold, "claw pivot 7"};
    const std::array<mech::MechanismSpec, 2> specs{r2::kRobotTwoMechanisms.specs[0], r2::kRobotTwoMechanisms.specs[1]};
    const std::array<mech::Devices, 2> devices{mech::Devices{.motors = &pivot6}, mech::Devices{.motors = &pivot7}};
    bool escaped = false;
    try {
        mech_rig::World world;
        world.clock = &clock;
        mech::MechanismService svc{world};
        svc.bind(specs, devices, clock, faults, sink);
        svc.power(0).drive(12_volt);
        svc.power(1).drive(12_volt);
        svc.pace();
        REQUIRE(pivot6.claimed());
        REQUIRE(pivot7.claimed());
        m6.once = [] { throw std::runtime_error("broken adapter"); };
    } catch (...) {
        escaped = true;
    }
    CHECK_FALSE(escaped);
    CHECK_FALSE(pivot6.claimed());  // released although its safe state threw: no claimant left dangling
    CHECK(pivot6.claimant() == nullptr);
    CHECK_FALSE(pivot7.claimed());  // the slot after it still ended ...
    CHECK(m7.commandedVoltage().value() == 0.0);  // ... safe
}

TEST_CASE("MECH-A2 claimant: another owner's claim -- every step REFUSED and nothing commanded; drive() and stop() are silent") {
    Rig r{lists::kIntakeHood, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(9).setDistance(Length{kNoObjectIn}); }};
    hal::fake::FakeMotor& m = r.devices.fakeMotor(5);
    SpyClaimant spy;
    REQUIRE(r.svc.mechanisms()[0]->tryClaim(spy));
    m.setVoltage(Voltage{4.0});
    m.setBrakeMode(hal::BrakeMode::Brake);
    mech::PowerSlot& intake = r.svc.power(0);
    intake.run(12_volt, Until::time(0.5_s), 1_s);
    CHECK(intake.lastStep().outcome == MechanismOutcome::Refused);
    CHECK_FALSE(intake.running());
    CHECK(r.lastMech() == "'intake': REFUSED in 0.00s (another owner holds its claim)");
    CHECK(r.lastMechLevel() == hal::LogLevel::Warn);
    const std::uint32_t seq = intake.commandSeq();
    intake.drive(12_volt);
    intake.stop();
    r.pace(10);
    CHECK(intake.commandSeq() == seq);  // neither drive() nor stop() spoke
    CHECK(m.commandedVoltage().value() == doctest::Approx(4.0));
    CHECK(m.brakeMode() == hal::BrakeMode::Brake);  // stop() did not repaint another owner's mechanism
    CHECK(r.svc.mechanisms()[0]->claimant() == &spy);
    CHECK(spy.cancels == 0);
    // the pneumatic: REFUSED, its lines untouched
    hal::fake::FakeDigitalOut& c = r.devices.fakeDigitalOut(3);
    hal::fake::FakeDigitalOut& dline = r.devices.fakeDigitalOut(4);
    SpyClaimant spy2;
    REQUIRE(r.svc.mechanisms()[1]->tryClaim(spy2));
    r.svc.pneumatic(1).set(2);  // "up" = High, High
    CHECK(r.svc.pneumatic(1).lastStep().outcome == MechanismOutcome::Refused);
    CHECK_FALSE(c.commanded());
    CHECK_FALSE(dline.commanded());
}

TEST_CASE("MECH-A2 claimant: an ABSENT mechanism -- every step ABSENT, said with why; nothing commanded, nothing claimed") {
    Rig r{lists::kIntakeHood, [](hal::fake::FakeDeviceProvider& d) {
              d.refusePort(5);
              d.refuseLine(3);
          }};
    mech::PowerSlot& intake = r.svc.power(0);
    CHECK_FALSE(intake.built());
    intake.run(12_volt, Until::time(0.5_s), 1_s);
    CHECK(intake.lastStep().outcome == MechanismOutcome::Absent);
    CHECK_FALSE(intake.running());
    CHECK(intake.stepsBegun() == 1U);
    const std::string line = r.lastMech();
    CHECK(line.rfind("'intake': ABSENT in 0.00s (not built at boot: ", 0) == 0);
    intake.drive(12_volt);
    intake.stop();
    r.pace(5);
    CHECK(intake.commandedVolts() == 0.0);
    CHECK(r.devices.fakeMotor(5).commandedVoltage().value() == 0.0);
    r.svc.pneumatic(1).set(1);
    CHECK(r.svc.pneumatic(1).lastStep().outcome == MechanismOutcome::Absent);
    CHECK(r.svc.pneumatic(1).commandedPosition() == mech::PneumaticSlot::kNoPosition);
    CHECK(r.svc.mechanisms().empty());
    intake.cancel();  // the guard never reaches it, but a direct cancel is harmless
    CHECK(intake.lastStep().outcome == MechanismOutcome::Absent);
}

TEST_CASE("MECH-A2 claimant: the most recent command wins -- a new step, a hold-to-run or a stop ends a running step CANCELLED") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PowerSlot& p6 = r.svc.power(0);
    hal::fake::FakeMotor& m6 = r.devices.fakeMotor(6);
    p6.run(12_volt, Until::time(2_s), 3_s);
    r.pace(10);
    p6.run(-6_volt, Until::time(1_s), 2_s);
    CHECK(r.mechLines().back() == "'claw pivot 6': CANCELLED in 0.10s (replaced by a newer command)");
    CHECK(p6.running());
    CHECK(p6.holdsClaim());  // the claim carried over: no gap another owner could take
    r.pace(1);
    CHECK(m6.commandedVoltage().value() == doctest::Approx(-6.0));
    p6.drive(9_volt);
    CHECK(r.mechLines().back() == "'claw pivot 6': CANCELLED in 0.01s (replaced by a newer command)");
    CHECK_FALSE(p6.running());
    r.pace(1);
    CHECK(m6.commandedVoltage().value() == doctest::Approx(9.0));
    p6.run(12_volt, Until::time(1_s), 2_s);
    r.pace(3);
    p6.stop();
    CHECK(r.mechLines().back() == "'claw pivot 6': CANCELLED in 0.03s (stopped)");
    CHECK(m6.commandedVoltage().value() == 0.0);
    CHECK(m6.brakeMode() == hal::BrakeMode::Hold);
    CHECK_FALSE(p6.holdsClaim());
    // the claw: a set() replaced mid-move
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    claw.set(1);
    r.pace(10);
    claw.set(0);
    CHECK(r.mechLines().back() == "'claw': CANCELLED in 0.10s (replaced by a newer command)");
    CHECK(claw.running());
}

TEST_CASE("MECH-A2 claimant: commandSeq counts EVERY command; stepsBegun every step; before any step there is no verdict") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PowerSlot& p6 = r.svc.power(0);
    CHECK(p6.stepsBegun() == 0U);
    CHECK(p6.lastStep().outcome == MechanismOutcome::Running);  // a default report, not a verdict ...
    CHECK_FALSE(p6.running());                                  // ... and nothing is running
    p6.run(12_volt, Until::time(0.1_s), 1_s);
    CHECK(p6.commandSeq() == 1U);
    CHECK(p6.stepsBegun() == 1U);
    p6.drive(3_volt);
    CHECK(p6.commandSeq() == 2U);
    p6.drive(3_volt);
    CHECK(p6.commandSeq() == 3U);  // each call speaks
    p6.stop();
    CHECK(p6.commandSeq() == 4U);
    CHECK(p6.stepsBegun() == 1U);
    r.svc.pneumatic(2).set(1);
    CHECK(r.svc.pneumatic(2).commandSeq() == 1U);
    CHECK(r.svc.pneumatic(2).stepsBegun() == 1U);
}

// ═══ LOOKUP ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 lookup: by KIND and NAME, never by address -- and a pneumatic goal checks its position too") {
    Rig r{lists::kIntakeHood};
    CHECK(r.svc.find(rx::intake) == &r.svc.power(0));
    CHECK(r.svc.find(rx::hood) == &r.svc.pneumatic(1));
    CHECK(r.svc.find(rx::hood["middle"]) == &r.svc.pneumatic(1));
    CHECK(r.svc.find(rx::hood["up"]) == &r.svc.pneumatic(1));
    // ANOTHER declaration with the same name and kind IS found (landmine 6: autons are per robot)
    CHECK(r.svc.find(lists::otherIntake) == &r.svc.power(0));
    // the same name, another KIND: not it
    CHECK(r.svc.find(lists::pneumaticNamedIntake) == nullptr);
    // another robot's hood whose "up" is at index 0: robot x's index 0 is "down" -> not it
    CHECK(r.svc.find(lists::reorderedHood["up"]) == nullptr);
    CHECK(r.svc.find(lists::reorderedHood["middle"]) == nullptr);  // index 2 is "up" here
    // a mechanism this robot does not have
    CHECK(r.svc.find(r2::clawPivot6) == nullptr);
    CHECK(r.svc.find(r2::claw) == nullptr);
    CHECK(r.svc.find(r2::claw["closed"]) == nullptr);
    // an unnamed declaration, a hand-made goal
    const mech::Power unnamed{};
    CHECK(r.svc.find(unnamed) == nullptr);
    CHECK(r.svc.find(mech::PneumaticGoal{nullptr, "hood", 9, "up"}) == nullptr);
    CHECK(r.svc.find(mech::PneumaticGoal{nullptr, "hood", 0, nullptr}) == nullptr);
    // any kind, by name
    CHECK(r.svc.slotFor("hood") == &r.svc.slot(1));
    CHECK(r.svc.slotFor("intake") == &r.svc.slot(0));
    CHECK(r.svc.slotFor("Hood") == nullptr);
    CHECK(r.svc.slotFor("") == nullptr);
    CHECK(r.svc.slotFor(nullptr) == nullptr);
    CHECK(r.svc.slotPtr(1) == &r.svc.slot(1));
    CHECK(r.svc.slotPtr(2) == nullptr);
}

// ═══ THE MODE CHANGES ════════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 driver entry: a leftover claim of ANOTHER owner is released WITHOUT its cancel(), counted, said -- and its motor made SAFE") {
    Rig r{r2::kRobotTwoMechanisms};
    SpyClaimant spy;
    hal::IMechanism* pivot7 = r.svc.mechanisms()[1];
    REQUIRE(pivot7->tryClaim(spy));  // an operation autonomous was running when PROS deleted its task
    // ... and its LAST command: 12 V, Coast (the dead operation's, which nothing will ever stop)
    r.devices.fakeMotor(7).setBrakeMode(hal::BrakeMode::Coast);
    r.devices.fakeMotor(7).setVoltage(Voltage{12.0});
    r.svc.enterDriverControl();
    CHECK(spy.cancels == 0);  // never called: it may have lived on the deleted task's stack
    CHECK_FALSE(pivot7->claimed());
    // T3's rule — "then made safe at entry like any other": released FIRST, so the stop that follows
    // finds it unclaimed and applies the declared safe state (robot two's pivots: Hold), not 12 V.
    CHECK(r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
    CHECK(r.devices.fakeMotor(7).brakeMode() == hal::BrakeMode::Hold);
    CHECK(r.svc.leftoverClaimsReleased() == 1);
    CHECK(r.mechLines().front() ==
          "'claw pivot 7': a claim left by another owner RELEASED as driver control began (its operation was not "
          "cancelled -- it may have lived on a deleted task)");
    // its button works from the first tick
    r.svc.power(1).drive(12_volt);
    r.svc.tickFromDriverLoop();
    CHECK(r.devices.fakeMotor(7).commandedVoltage().value() == doctest::Approx(12.0));
}

TEST_CASE("MECH-A2 driver entry: the service's own autonomous-era behaviours are dropped -- a running step CANCELLED, motors safe, claims released") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PowerSlot& p6 = r.svc.power(0);
    p6.run(12_volt, Until::time(5_s), 6_s);
    r.pace(10);
    REQUIRE(r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    r.svc.enterDriverControl();
    CHECK(p6.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.mechLines().back() == "'claw pivot 6': CANCELLED in 0.10s (driver control began)");
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(r.devices.fakeMotor(6).brakeMode() == hal::BrakeMode::Hold);
    CHECK_FALSE(p6.holdsClaim());
    CHECK(r.svc.leftoverClaimsReleased() == 0);  // its OWN claim is not a leftover
    r.pace(10);
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 driver entry: the claw goes to START on the FIRST entry only; a disable keeps its level, entry after entry") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    hal::fake::FakeDigitalOut& lineA = r.devices.fakeDigitalOut(1);
    CHECK(claw.commandedPosition() == 0);  // "closed" = Low, made at boot by the parts
    claw.set(1);                           // the auton opens it ...
    r.pace(60);
    REQUIRE(lineA.commanded());
    REQUIRE(claw.commandedPosition() == 1);
    r.svc.enterDriverControl();  // ... the FIRST entry closes it (T4: "starts closed")
    CHECK_FALSE(lineA.commanded());
    CHECK(claw.commandedPosition() == 0);
    claw.set(1);  // the driver opens it
    r.pace(60);
    REQUIRE(lineA.commanded());
    r.svc.leaveDriverControl();  // a disable: the claw keeps what it holds
    CHECK(lineA.commanded());
    r.svc.enterDriverControl();  // the SECOND entry sends nothing
    CHECK(lineA.commanded());
    CHECK(claw.commandedPosition() == 1);
}

TEST_CASE("MECH-A2 leave: a disable stops every MOTOR mechanism (safe, released), keeps every pneumatic level, leaves another owner's claim alone") {
    Rig r{r2::kRobotTwoMechanisms};
    r.svc.enterDriverControl();
    mech::PowerSlot& p6 = r.svc.power(0);
    p6.drive(12_volt);
    r.svc.tickFromDriverLoop();
    REQUIRE(r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    // pivot 7: an operation the program ticks itself took it DURING the loop
    SpyClaimant spy;
    REQUIRE(r.svc.mechanisms()[1]->tryClaim(spy));
    r.devices.fakeMotor(7).setVoltage(Voltage{5.0});
    // the claw mid-move
    r.svc.pneumatic(2).set(1);
    r.svc.tickFromDriverLoop();
    r.svc.leaveDriverControl();
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(r.devices.fakeMotor(6).brakeMode() == hal::BrakeMode::Hold);
    CHECK_FALSE(p6.holdsClaim());
    CHECK(r.devices.fakeMotor(7).commandedVoltage().value() == doctest::Approx(5.0));  // not ours
    CHECK(r.svc.mechanisms()[1]->claimant() == &spy);
    CHECK(spy.cancels == 0);
    CHECK(r.devices.fakeDigitalOut(1).commanded());  // the open level kept
    CHECK(r.svc.pneumatic(2).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(r.mechLines().back() == "'claw': CANCELLED in 0.00s (driver control ended)");
    CHECK_FALSE(r.svc.mechanisms()[2]->claimed());
    for (int i = 0; i < 10; ++i) {  // nothing re-commands
        r.svc.tickFromDriverLoop();
    }
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 enterAutonomous (D1): a driver task PROS deleted leaves its hold-to-run at 12 V -- the call drops it; without it the auton runs it") {
    for (const bool call : {false, true}) {
        CAPTURE(call);
        Rig r{r2::kRobotTwoMechanisms};
        r.svc.enterDriverControl();
        for (int i = 0; i < 100; ++i) {  // the driver holds R1: pivot 6 at 12 V
            r.svc.power(0).drive(12_volt);
            r.svc.tickFromDriverLoop();
            r.world.pace();
        }
        // PROS deletes the driver task here: leaveDriverControl() never runs.
        int released = -1;
        if (call) {
            released = r.svc.enterAutonomous();
        }
        int at12 = 0;
        for (int i = 0; i < 269; ++i) {  // an auton that never names pivot 6
            r.svc.pace();
            if (r.devices.fakeMotor(6).commandedVoltage().value() > 11.0) {
                ++at12;
            }
        }
        if (call) {
            CHECK(at12 == 0);
            CHECK(released == 0);
            CHECK(r.devices.fakeMotor(6).brakeMode() == hal::BrakeMode::Hold);
            CHECK_FALSE(r.svc.power(0).holdsClaim());
        } else {
            CHECK(at12 == 269);  // the control: the defect the call exists for
        }
    }
}

TEST_CASE("MECH-A2 enterAutonomous (D1): a foreign claim released WITHOUT its cancel(), counted, its motor made SAFE; a pneumatic KEEPS its level; a running step CANCELLED") {
    Rig r{r2::kRobotTwoMechanisms};
    SpyClaimant spy;
    REQUIRE(r.svc.mechanisms()[1]->tryClaim(spy));
    r.devices.fakeMotor(7).setBrakeMode(hal::BrakeMode::Coast);  // the dead operation's last command
    r.devices.fakeMotor(7).setVoltage(Voltage{12.0});
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    claw.set(1);  // the driver opened the claw; its step is mid-move when the task dies
    r.svc.tickFromDriverLoop();
    REQUIRE(claw.running());
    CHECK(r.svc.enterAutonomous() == 1);
    CHECK(spy.cancels == 0);
    CHECK_FALSE(r.svc.mechanisms()[1]->claimed());
    // released FIRST, then stopped: the dead operation's 12 V does not run into the auton
    CHECK(r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
    CHECK(r.devices.fakeMotor(7).brakeMode() == hal::BrakeMode::Hold);
    CHECK(r.svc.leftoverClaimsReleased() == 1);
    CHECK(r.devices.fakeDigitalOut(1).commanded());  // KEPT: open
    CHECK(claw.commandedPosition() == 1);
    CHECK(claw.lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK_FALSE(r.svc.mechanisms()[2]->claimed());
    const std::vector<std::string> lines = r.mechLines();
    CHECK(std::find(lines.begin(), lines.end(),
                    "'claw pivot 7': a claim left by another owner RELEASED as autonomous began (its operation was not "
                    "cancelled -- it may have lived on a deleted task)") != lines.end());
    CHECK(lines.back() == "'claw': CANCELLED in 0.00s (autonomous began)");
    CHECK(r.svc.enterAutonomous() == 0);  // nothing left the second time
}
