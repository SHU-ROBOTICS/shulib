// The POWER behaviour (chunk MECH-A2, the MECH brief's §1.7 "Power" and §3.2): run at a voltage
// until a done-condition, within a bound; the driver's hold-to-run; the step preconditions.
//
// Each case tries to break one rule, against the host test plant where a motor must actually turn
// or jam (sim::ArmPlant as a roller: no gravity, no stops — its constants INVENTED, HA-183):
//   * every done-condition alone and OR-ed ends the step with the right verdict and WORD;
//   * a stall ends a step SUCCEEDED only when `stalls` asked for it — otherwise STALLED, with
//     MECHANISM_STALLED raised exactly once;
//   * the declared safe state lands on EVERY exit, and the volts never pass maxVolts;
//   * a condition that can never be met is refused at the step with NOTHING commanded (D7);
//   * a declared piece sensor MISSING this boot runs the whole bound with the motor running and ends
//     UNCONFIRMED, COUNTED done, naming the sensor — never a false SUCCEEDED (P13, N8 (a));
//   * the hold-to-run is watched by a STATED stall signature; with none, nothing but the release cuts
//     it (HA-154 — the documented limit).

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/until.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/sim/mechanism_plant.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;
namespace rx = mech_fixture::robot_x;
using mech::DoneBy;
using mech::MechanismOutcome;
using mech::Until;
using mech_rig::kNoObjectIn;
using mech_rig::Rig;
using shulib::diag::FaultCode;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using namespace shulib::units::literals;

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
/// Robot y's power and pneumatic half: an OPTICAL roller (port 6, sensor 9), a limit-switch catapult
/// WITH a stall signature (port 7, switch 'E'), a grip with a DISTANCE sensor (line 'A', sensor 8).
inline constexpr MechanismTable kY = mechanisms(wire(ry::roller, motors(6), pieceSensor(9)),
                                                wire(ry::catapult, motors(7), pieceSensor('E')),
                                                wire(ry::grip, lines('A'), pieceSensor(8)));
/// A power mechanism whose ceiling is 8 V.
inline constexpr Power gentle{.name = "gentle", .motors = {Forward}, .cartridge = Green, .safe = Brake, .maxVolts = 8_volt};
inline constexpr MechanismTable kGentle = mechanisms(wire(gentle, motors(3)));
/// A declaration written WITHOUT `.name` (on no robot's list; a hand-made Until can still name it).
inline constexpr Power nameless{.motors = {Forward}, .cartridge = Blue, .pieceSensor = opticalSensor(0.5), .safe = Coast};
}  // namespace lists

constexpr std::size_t kRoller = 0;
constexpr std::size_t kCatapult = 1;
constexpr std::size_t kGrip = 2;

/// Robot y's half, booted, with "no piece" at both sensors and a ROLLER PLANT on each motor.
struct YRig {
    Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) {
              d.fakeOptical(9).setProximity(0.0);
              d.fakeDistance(8).setDistance(Length{kNoObjectIn});
              d.fakeDigitalIn(5).setState(false);
          }};
    std::array<hal::fake::FakeMotor*, 1> rollerM{&r.devices.fakeMotor(6)};
    std::array<hal::fake::FakeMotor*, 1> catapultM{&r.devices.fakeMotor(7)};
    shulib::sim::ArmPlant roller{rollerM, rollerCfg()};
    shulib::sim::ArmPlant catapult{catapultM, rollerCfg()};
    YRig() {
        r.world.steps.push_back([this](double dt) { roller.step(dt); });
        r.world.steps.push_back([this](double dt) { catapult.step(dt); });
    }
    /// A free roller: Blue-ish, no gravity, no stops (HA-183: invented).
    static shulib::sim::ArmPlantConfig rollerCfg() {
        shulib::sim::ArmPlantConfig c;
        c.gearRatio = 1.0;
        c.kS = 0.3;
        c.kV = 12.0 / 62.8;  // 600 rpm free
        c.kA = 0.05 * c.kV;
        return c;
    }
};

}  // namespace

// ═══ THE DONE-CONDITIONS ═════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 power: by TIME -- SUCCEEDED at its time, the motor run the whole while, then safe") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(12_volt, Until::time(0.5_s), 1_s);
    int ran = 0;
    while (roller.running() && ran < 500) {
        y.r.svc.pace();
        if (std::abs(roller.commandedVolts()) > 0.1) {
            ++ran;
        }
    }
    CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(roller.lastStep().doneBy == DoneBy::Time);
    CHECK(roller.lastStep().elapsed == doctest::Approx(0.5));
    CHECK(ran == 50);
    CHECK(y.r.lastMech() == "'roller': SUCCEEDED -- time in 0.50s");
    CHECK(y.r.lastMechLevel() == hal::LogLevel::Info);
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(y.r.devices.fakeMotor(6).brakeMode() == hal::BrakeMode::Coast);
    CHECK_FALSE(roller.holdsClaim());
    CHECK(y.roller.trueSpeedDegS() > 100.0);  // it really turned
}

TEST_CASE("MECH-A2 power: its OWN sensor sees a piece -> SUCCEEDED 'sees a piece' that tick, mid-run") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(12_volt, mech::seesPiece, 3_s);
    for (int i = 0; i < 30; ++i) {
        y.r.svc.pace();
    }
    REQUIRE(roller.running());
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    y.r.devices.fakeOptical(9).setProximity(0.8);
    y.r.svc.pace();
    CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(roller.lastStep().doneBy == DoneBy::Sees);
    CHECK(y.r.lastMech() == "'roller': SUCCEEDED -- sees a piece in 0.30s");
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 power: ANOTHER mechanism's sensor (seesPiece(grip)) ends the step when THAT one sees a piece") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    y.r.devices.fakeOptical(9).setProximity(0.9);  // its OWN sensor sees one the whole time: irrelevant
    roller.run(12_volt, mech::seesPiece(ry::grip), 3_s);
    for (int i = 0; i < 40; ++i) {
        y.r.svc.pace();
    }
    REQUIRE(roller.running());
    y.r.devices.fakeDistance(8).setDistance(Length{1.0});
    y.r.devices.fakeDistance(8).setConfidence(1.0);
    y.r.svc.pace();
    CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(roller.lastStep().doneBy == DoneBy::Sees);
    CHECK(roller.lastStep().elapsed == doctest::Approx(0.40));
}

TEST_CASE("MECH-A2 power: STOPS SEEING -> SUCCEEDED 'stopped seeing a piece' (an eject)") {
    YRig y;
    y.r.devices.fakeOptical(9).setProximity(0.9);
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(-6_volt, mech::stopsSeeingPiece, 2_s);
    for (int i = 0; i < 20; ++i) {
        y.r.svc.pace();
    }
    REQUIRE(roller.running());
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(-6.0));
    y.r.devices.fakeOptical(9).setProximity(0.1);
    y.r.svc.pace();
    CHECK(roller.lastStep().doneBy == DoneBy::StopsSeeing);
    CHECK(y.r.lastMech() == "'roller': SUCCEEDED -- stopped seeing a piece in 0.20s");
}

TEST_CASE("MECH-A2 power: a condition already met at the FIRST tick ends the step before the motor is ever commanded") {
    YRig y;
    y.r.devices.fakeOptical(9).setProximity(0.9);
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(12_volt, mech::seesPiece, 3_s);
    y.r.svc.pace();
    CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(roller.lastStep().elapsed == 0.0);
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(y.roller.trueSpeedDegS() == 0.0);
}

TEST_CASE("MECH-A2 power: seesPiece.orAfter -- SUCCEEDED by TIME with no piece; by the SENSOR when one comes first") {
    {
        YRig y;
        mech::PowerSlot& roller = y.r.svc.power(kRoller);
        roller.run(12_volt, mech::seesPiece.orAfter(0.6_s), 2_s);
        y.r.paceUntilDone(kRoller);
        CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(roller.lastStep().doneBy == DoneBy::Time);
        CHECK(roller.lastStep().elapsed == doctest::Approx(0.6));
    }
    {
        YRig y;
        mech::PowerSlot& roller = y.r.svc.power(kRoller);
        roller.run(12_volt, mech::seesPiece.orAfter(0.6_s), 2_s);
        y.r.world.steps.push_back([&y](double) {
            if (y.r.clock.now().value() >= 0.285) {
                y.r.devices.fakeOptical(9).setProximity(0.7);
            }
        });
        y.r.paceUntilDone(kRoller);
        CHECK(roller.lastStep().doneBy == DoneBy::Sees);
        CHECK(roller.lastStep().elapsed == doctest::Approx(0.30));
    }
    {   // the tie: the piece seen on the very tick its time is up -> the EVIDENCE names it
        YRig y;
        mech::PowerSlot& roller = y.r.svc.power(kRoller);
        roller.run(12_volt, mech::seesPiece.orAfter(0.5_s), 1_s);
        for (int i = 0; i < 50; ++i) {
            y.r.svc.pace();
        }
        y.r.devices.fakeOptical(9).setProximity(0.7);
        y.r.svc.pace();
        CHECK(roller.lastStep().doneBy == DoneBy::Sees);
    }
}

TEST_CASE("MECH-A2 power: the BOUND -> TIMED_OUT (the sensor answered NO all along), and Until::bound() runs exactly its bound") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(12_volt, mech::seesPiece, 1_s);
    const int n = y.r.paceUntilDone(kRoller);
    CHECK(n == 101);
    CHECK(roller.lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK(roller.lastStep().doneBy == DoneBy::None);
    CHECK_FALSE(roller.lastStep().countedDone);
    CHECK(y.r.lastMech() == "'roller': TIMED_OUT in 1.00s");
    CHECK(y.r.lastMechLevel() == hal::LogLevel::Warn);
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(y.r.faults.faultCount() == 0);  // a step's own bound raises nothing (TIMED_OUT is not a fault, F1)
    roller.run(6_volt, Until::bound(), 0.25_s);
    y.r.paceUntilDone(kRoller);
    CHECK(roller.lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK(roller.lastStep().elapsed == doctest::Approx(0.25).epsilon(0.05));
}

TEST_CASE("MECH-A2 power: a time EQUAL to the bound ends SUCCEEDED by time (the time is checked first, by the same arithmetic)") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    for (const double t : {0.4, 0.37, 1.0, 0.1}) {
        CAPTURE(t);
        roller.run(12_volt, Until::time(Time{t}), Time{t});
        y.r.paceUntilDone(kRoller);
        CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(roller.lastStep().doneBy == DoneBy::Time);
    }
}

TEST_CASE("MECH-A2 power: the time and the bound EXACTLY equal on an exact clock -- still SUCCEEDED by time (the coordinator's MB5, 2026-10-15)") {
    // With 0.01 s ticks the elapsed time never lands on the bound exactly (0.01 has no exact binary
    // form), so the case above passes whether the time check is `>=` or `>`. A tick of 1/128 s is
    // exact, and so is every elapsed time below: the tick where elapsed == the time == the bound is
    // the one the rule is about.
    YRig y;
    y.r.world.dt = 1.0 / 128.0;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    for (const double t : {0.25, 0.5, 1.0}) {
        CAPTURE(t);
        roller.run(12_volt, Until::time(Time{t}), Time{t});
        y.r.paceUntilDone(kRoller);
        CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(roller.lastStep().doneBy == DoneBy::Time);
        CHECK(roller.lastStep().elapsed == t);  // landed on it exactly: the edge was really tested
    }
}

TEST_CASE("MECH-A2 power: a STALL -- SUCCEEDED when `stalls` asked; STALLED + MECHANISM_STALLED once when it did not") {
    {
        YRig y;
        y.catapult.setJammed(true);  // the squeeze closes on a piece at once
        mech::PowerSlot& cata = y.r.svc.power(kCatapult);
        cata.run(12_volt, mech::stalls, 2_s);
        y.r.paceUntilDone(kCatapult);
        CHECK(cata.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(cata.lastStep().doneBy == DoneBy::Stalled);
        // the signature: >= 2.0 A below 2 rpm for 0.2 s; stall-grade from the first commanded tick
        CHECK(cata.lastStep().elapsed == doctest::Approx(0.21).epsilon(0.05));
        CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 0);
        CHECK(y.r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
        CHECK(y.r.devices.fakeMotor(7).brakeMode() == hal::BrakeMode::Hold);
    }
    {
        YRig y;
        mech::PowerSlot& cata = y.r.svc.power(kCatapult);
        cata.run(12_volt, Until::time(3_s), 4_s);
        for (int i = 0; i < 50; ++i) {
            y.r.svc.pace();
        }
        REQUIRE(cata.running());
        y.catapult.setJammed(true);  // a jam half a second in
        y.r.paceUntilDone(kCatapult);
        CHECK(cata.lastStep().outcome == MechanismOutcome::Stalled);
        CHECK(cata.lastStep().doneBy == DoneBy::Stalled);
        CHECK(cata.lastStep().elapsed == doctest::Approx(0.71).epsilon(0.03));
        CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 1);
        CHECK(y.r.lastMech().rfind("'catapult': STALLED -- stalled in 0.7", 0) == 0);
        CHECK(y.r.lastMech().find("(a stated stall signature tripped)") != std::string::npos);
        CHECK(y.r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
        CHECK_FALSE(cata.holdsClaim());
        y.r.pace(100);  // once: nothing re-raises after the verdict
        CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 1);
    }
}

TEST_CASE("MECH-A2 power: conditions OR-ed -- seesPiece | stalls ends on whichever comes first, named by the one that did") {
    {   // the jam first: SUCCEEDED by the stall `stalls` asked for, no fault
        YRig y;
        mech::PowerSlot& cata = y.r.svc.power(kCatapult);
        cata.run(12_volt, mech::seesPiece | mech::stalls, 2_s);
        y.r.pace(30);
        REQUIRE(cata.running());
        y.catapult.setJammed(true);
        y.r.paceUntilDone(kCatapult);
        CHECK(cata.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(cata.lastStep().doneBy == DoneBy::Stalled);
        CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 0);
    }
    {   // the piece first: SUCCEEDED by the switch, the stall never reached
        YRig y;
        mech::PowerSlot& cata = y.r.svc.power(kCatapult);
        cata.run(12_volt, mech::seesPiece | mech::stalls, 2_s);
        y.r.pace(30);
        y.r.devices.fakeDigitalIn(5).setState(true);  // the catapult's limit switch: loaded
        y.r.pace();
        CHECK(cata.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(cata.lastStep().doneBy == DoneBy::Sees);
        CHECK(cata.lastStep().elapsed == doctest::Approx(0.30));
    }
    {   // a time OR a stall, neither met: the time wins at its time (the bound is later)
        YRig y;
        mech::PowerSlot& cata = y.r.svc.power(kCatapult);
        cata.run(12_volt, Until::time(0.4_s) | mech::stalls, 2_s);
        y.r.paceUntilDone(kCatapult);
        CHECK(cata.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(cata.lastStep().doneBy == DoneBy::Time);
        CHECK(cata.lastStep().elapsed == doctest::Approx(0.4));
    }
}

TEST_CASE("MECH-A2 power: a spin-up is NOT a stall -- a free roller with a signature runs its whole time") {
    YRig y;
    mech::PowerSlot& cata = y.r.svc.power(kCatapult);
    cata.run(12_volt, Until::time(1_s), 2_s);
    y.r.paceUntilDone(kCatapult);
    CHECK(cata.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(cata.lastStep().doneBy == DoneBy::Time);
    CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 0);
}

TEST_CASE("MECH-A2 power: the volts are CLAMPED to the declaration's maxVolts -- both signs, a step and a hold-to-run") {
    Rig r{lists::kGentle};
    mech::PowerSlot& g = r.svc.power(0);
    g.run(12_volt, Until::time(0.2_s), 1_s);
    r.svc.pace();
    CHECK(r.devices.fakeMotor(3).commandedVoltage().value() == doctest::Approx(8.0));
    g.run(-12_volt, Until::time(0.2_s), 1_s);
    r.svc.pace();
    CHECK(r.devices.fakeMotor(3).commandedVoltage().value() == doctest::Approx(-8.0));
    g.run(5_volt, Until::time(0.2_s), 1_s);
    r.svc.pace();
    CHECK(r.devices.fakeMotor(3).commandedVoltage().value() == doctest::Approx(5.0));
    g.drive(11_volt);
    r.svc.pace();
    CHECK(r.devices.fakeMotor(3).commandedVoltage().value() == doctest::Approx(8.0));
    g.stop();
    CHECK(r.devices.fakeMotor(3).brakeMode() == hal::BrakeMode::Brake);  // its declared safe state
}

TEST_CASE("MECH-A2 power: the declared SAFE STATE lands on EVERY exit -- time, sees, stops seeing, stalls, the bound, a cancel, a stop") {
    const auto check = [](YRig& y, std::size_t i, int port, hal::BrakeMode safe) {
        CHECK_FALSE(y.r.svc.slot(i).running());
        CHECK(y.r.devices.fakeMotor(static_cast<std::uint8_t>(port)).commandedVoltage().value() == 0.0);
        CHECK(y.r.devices.fakeMotor(static_cast<std::uint8_t>(port)).brakeMode() == safe);
        CHECK_FALSE(y.r.svc.slot(i).holdsClaim());
    };
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    roller.run(12_volt, Until::time(0.2_s), 1_s);
    y.r.paceUntilDone(kRoller);
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    y.r.devices.fakeOptical(9).setProximity(0.9);
    roller.run(12_volt, mech::seesPiece, 1_s);
    y.r.paceUntilDone(kRoller);
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    y.r.devices.fakeOptical(9).setProximity(0.0);
    roller.run(12_volt, mech::stopsSeeingPiece, 1_s);
    y.r.paceUntilDone(kRoller);
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    roller.run(12_volt, mech::seesPiece, 0.3_s);
    y.r.paceUntilDone(kRoller);
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    roller.run(12_volt, Until::bound(), 1_s);
    y.r.pace(5);
    roller.cancel();
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    roller.run(12_volt, Until::bound(), 1_s);
    y.r.pace(5);
    roller.stop();
    check(y, kRoller, 6, hal::BrakeMode::Coast);
    y.catapult.setJammed(true);
    mech::PowerSlot& cata = y.r.svc.power(kCatapult);
    cata.run(12_volt, mech::stalls, 1_s);
    y.r.paceUntilDone(kCatapult);
    check(y, kCatapult, 7, hal::BrakeMode::Hold);
    cata.run(12_volt, Until::time(1_s), 2_s);
    y.r.paceUntilDone(kCatapult);
    CHECK(cata.lastStep().outcome == MechanismOutcome::Stalled);
    check(y, kCatapult, 7, hal::BrakeMode::Hold);
}

// ═══ THE STEP'S PRECONDITIONS: a condition that can never be met, refused with NOTHING commanded ═══

TEST_CASE("MECH-A2 power preconditions (D7): seesPiece / stopsSeeingPiece / stalls that can never be met -- refused, nothing commanded") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PowerSlot& p6 = r.svc.power(0);  // claw pivot 6: no piece sensor, no stall signature
    const auto nothingCommanded = [&] {
        CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
        CHECK_FALSE(p6.holdsClaim());
        CHECK(p6.commandSeq() == 0U);
        CHECK(p6.stepsBegun() == 0U);
        CHECK(r.mechLines().empty());
    };
    CHECK(preconditionOf([&] { p6.run(12_volt, mech::seesPiece, 1_s); }).find("declares NO piece sensor") !=
          std::string::npos);
    nothingCommanded();
    CHECK(preconditionOf([&] { p6.run(-6_volt, mech::stopsSeeingPiece, 1_s); }).find("declares NO piece sensor") !=
          std::string::npos);
    nothingCommanded();
    CHECK(preconditionOf([&] { p6.run(6_volt, mech::stalls, 1_s); }).find("no stall signature") != std::string::npos);
    nothingCommanded();
    // ANOTHER mechanism with none (the build refuses seesPiece(clawPivot7); a hand-made Until reaches here)
    CHECK(preconditionOf([&] { p6.run(12_volt, Until::sees(r2::clawPivot7), 1_s); }).find("declares NO piece sensor") !=
          std::string::npos);
    nothingCommanded();
    // a mechanism this robot does not have
    CHECK(preconditionOf([&] { p6.run(12_volt, Until::sees(rx::intake), 1_s); }).find("does not have") !=
          std::string::npos);
    nothingCommanded();
    r.pace(10);
    nothingCommanded();
}

TEST_CASE("MECH-A2 power preconditions: the numbers -- volts, the bound, the time, orAfter vs the bound, two sensors") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    const double nan = std::nan("");
    CHECK(preconditionOf([&] { roller.run(Voltage{nan}, Until::time(0.5_s), 1_s); }).find("finite") != std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(0.5_s), Time{0.0}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(0.5_s), Time{nan}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(Time{0.0}), 1_s); }).find("time condition") != std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(Time{-1.0}), 1_s); }).find("time condition") != std::string::npos);
    // a time ALONE past the bound would never end it by time
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(2_s), 1_s); }).find("shorter than") != std::string::npos);
    // orAfter beside a sensor must be SHORTER than the bound (equal: the bound would always win the race it starts)
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece.orAfter(1_s), 1_s); }).find("shorter than") !=
          std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece.orAfter(3_s), 1_s); }).find("shorter than") !=
          std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece | Until::sees(ry::grip), 1_s); }).find("two different") !=
          std::string::npos);
    CHECK(preconditionOf([&] { roller.drive(Voltage{nan}); }).find("finite") != std::string::npos);
    CHECK(roller.commandSeq() == 0U);
    CHECK(y.r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    // ... and the legal edges: a time equal to the bound; orAfter just under it
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::time(1_s), 1_s); }).empty());
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece.orAfter(0.99_s), 1_s); }).empty());
}

TEST_CASE("MECH-A2 power preconditions: the step's OWN sensor beside a NAME -- one sensor on that very mechanism (it runs), two on any other (refused); a nameless declaration refused for what it is") {
    YRig y;
    mech::PowerSlot& roller = y.r.svc.power(kRoller);
    mech::PowerSlot& cata = y.r.svc.power(kCatapult);
    // On the roller ITSELF, `seesPiece | stopsSeeingPiece(roller)` reads ONE sensor: accepted, and it
    // reads that sensor (the MECH-A2 review: it was refused as "two different mechanisms' sensors").
    y.r.devices.fakeOptical(9).setProximity(0.9);  // a piece in front of the roller
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece | mech::stopsSeeingPiece(ry::roller), 1_s); })
              .empty());
    CHECK(y.r.paceUntilDone(kRoller) == 1);
    CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(roller.lastStep().doneBy == DoneBy::Sees);
    // The same words on ANOTHER mechanism read two sensors: refused, NOTHING commanded.
    CHECK(preconditionOf([&] { cata.run(6_volt, mech::seesPiece | mech::stopsSeeingPiece(ry::roller), 1_s); })
              .find("two different") != std::string::npos);
    CHECK(preconditionOf([&] { cata.run(6_volt, mech::stopsSeeingPiece(ry::roller) | mech::seesPiece, 1_s); })
              .find("two different") != std::string::npos);
    CHECK(cata.commandSeq() == 0U);
    CHECK(cata.stepsBegun() == 0U);
    CHECK(y.r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
    // A declaration with no `.name`: never read as the step's own sensor — refused, said what it is.
    CHECK(preconditionOf([&] { roller.run(12_volt, Until::sees(lists::nameless), 1_s); }).find("no .name") !=
          std::string::npos);
    CHECK(preconditionOf([&] { roller.run(12_volt, mech::seesPiece | Until::stopsSeeing(lists::nameless), 1_s); })
              .find("no .name") != std::string::npos);
    CHECK(roller.stepsBegun() == 1U);  // only the accepted step above began
}

// ═══ A DECLARED SENSOR MISSING THIS BOOT (P13, N8 (a)) ══════════════════════════════════════

TEST_CASE("MECH-A2 power P13: a declared sensor MISSING this boot -- both conditions run the whole bound, motor running, UNCONFIRMED COUNTED, the sensor named") {
    for (const bool stops : {false, true}) {
        CAPTURE(stops);
        Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) { d.refusePort(9); }};  // the roller's optical: refused
        mech::PowerSlot& roller = r.svc.power(kRoller);
        REQUIRE(roller.built());
        REQUIRE(roller.pieceSensorMissing());
        roller.run(stops ? -6_volt : 12_volt, stops ? Until::stopsSeeing() : Until::sees(), 1_s);
        int ran = 0;
        while (roller.running() && ran < 500) {
            r.svc.pace();
            if (std::abs(roller.commandedVolts()) > 0.1) {
                ++ran;
            }
        }
        CHECK(ran == 100);  // the motor really ran its time
        CHECK(roller.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK(roller.lastStep().countedDone);
        CHECK(roller.lastStep().doneBy == DoneBy::Time);
        CHECK(roller.lastStep().elapsed == doctest::Approx(1.0));
        CHECK(r.lastMech() ==
              "'roller': UNCONFIRMED -- time in 1.00s (its piece sensor is missing this boot, port 9; counted done)");
    }
}

TEST_CASE("MECH-A2 power P13: the same step with the sensor PRESENT and seeing still SUCCEEDS at once -- and ANOTHER mechanism's missing sensor is named") {
    {
        Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) { d.fakeOptical(9).setProximity(0.9); }};
        mech::PowerSlot& roller = r.svc.power(kRoller);
        roller.run(12_volt, Until::sees(), 1_s);
        CHECK(r.paceUntilDone(kRoller) == 1);
        CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    }
    {
        Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) { d.refusePort(8); }};  // the GRIP's distance sensor
        mech::PowerSlot& roller = r.svc.power(kRoller);
        roller.run(12_volt, mech::seesPiece(ry::grip), 0.5_s);
        r.paceUntilDone(kRoller);
        CHECK(roller.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK(roller.lastStep().countedDone);
        CHECK(r.lastMech() ==
              "'roller': UNCONFIRMED -- time in 0.50s (the piece sensor of 'grip' is missing this boot, port 8; counted "
              "done)");
    }
    {   // the catapult's limit switch on a line: named by its letter (the robot's list names it)
        Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) { d.refuseLine(5); }};
        mech::PowerSlot& cata = r.svc.power(kCatapult);
        REQUIRE(cata.pieceSensorMissing());
        cata.run(6_volt, Until::sees(), 0.2_s);
        r.paceUntilDone(kCatapult);
        CHECK(r.lastMech() ==
              "'catapult': UNCONFIRMED -- time in 0.20s (its piece sensor is missing this boot, 3-wire E; counted done)");
    }
}

// ═══ THE DRIVER'S HOLD-TO-RUN ════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 power drive(): the hold-to-run commands its volts every tick, open-ended, until a stop") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PowerSlot& p6 = r.svc.power(0);
    p6.drive(12_volt);
    for (int i = 0; i < 500; ++i) {
        r.svc.tickFromDriverLoop();
        r.world.pace();
        REQUIRE(r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    }
    CHECK(p6.holdsClaim());
    CHECK_FALSE(p6.running());  // a hold-to-run is not a step
    CHECK(r.mechLines().empty());
    p6.drive(-12_volt);  // the other button
    r.svc.tickFromDriverLoop();
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(-12.0));
    p6.stop();  // the release
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
    CHECK(r.devices.fakeMotor(6).brakeMode() == hal::BrakeMode::Hold);
    CHECK_FALSE(p6.holdsClaim());
    r.svc.tickFromDriverLoop();
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 power drive(): a STATED signature watches the hold-to-run -- STALLED once, safe, released, the command number bumped") {
    YRig y;
    mech::PowerSlot& cata = y.r.svc.power(kCatapult);
    cata.drive(12_volt);
    for (int i = 0; i < 20; ++i) {  // spinning freely: no trip
        cata.drive(12_volt);
        y.r.svc.tickFromDriverLoop();
        y.r.world.pace();
    }
    y.catapult.setJammed(true);  // into a jam
    std::uint32_t seqWritten = 0;
    int t = 0;
    for (; t < 200; ++t) {
        if (cata.commandSeq() != seqWritten && t > 0) {
            break;  // superseded: what the driver's input sees (MECH-A4 stops pressing here)
        }
        cata.drive(12_volt);
        seqWritten = cata.commandSeq();
        y.r.svc.tickFromDriverLoop();
        y.r.world.pace();
    }
    CHECK(cata.lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(cata.stepsBegun() == 0U);  // a hold-to-run is not a step -- its STALLED is still lastStep()'s
    CHECK_FALSE(cata.running());
    CHECK(y.r.faults.raiseCount(FaultCode::MechanismStalled) == 1);
    CHECK(t * 0.01 == doctest::Approx(0.22).epsilon(0.1));  // >= 2.0 A below 2 rpm for 0.2 s
    CHECK(y.r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
    CHECK_FALSE(cata.holdsClaim());
    CHECK(y.r.lastMech().find("(a stated stall signature tripped while held to run)") != std::string::npos);
    // the elapsed is the hold-to-run's own: from the first drive() of this hold, 0.2 s free + the trip
    CHECK(cata.lastStep().elapsed > 0.35);
    y.r.pace(50);
    CHECK(y.r.devices.fakeMotor(7).commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 power drive(): with NO stall signature nothing but the release cuts a jammed hold-to-run (HA-154, the stated limit)") {
    Rig r{r2::kRobotTwoMechanisms};
    std::array<hal::fake::FakeMotor*, 1> m{&r.devices.fakeMotor(6)};
    shulib::sim::ArmPlantConfig cfg;
    shulib::sim::ArmPlant pivot{m, cfg};
    pivot.setJammed(true);
    r.world.steps.push_back([&pivot](double dt) { pivot.step(dt); });
    r.svc.power(0).drive(12_volt);
    for (int i = 0; i < 500; ++i) {
        r.svc.tickFromDriverLoop();
        r.world.pace();
    }
    CHECK(r.devices.fakeMotor(6).commandedVoltage().value() == doctest::Approx(12.0));
    CHECK(r.devices.fakeMotor(6).current().value() == doctest::Approx(2.5));
    CHECK(r.faults.raiseCount(FaultCode::MechanismStalled) == 0);
}

TEST_CASE("MECH-A2 power: a NEW hold-to-run and a NEW step each get their OWN stall window -- a jam still drawing current does not re-trip on the first tick") {
    // StallDetector keeps its window open after a trip; drive() (a new hold) and run() reset it. Over
    // the host test plant a jammed joint reads 0 A at 0 V between a trip and the next command, which
    // hides a missing reset — a real jammed motor under Hold can keep drawing. So the catapult's motor
    // is held at stall grade BY HAND (no plant): 2.4 A, 0 rpm. Its signature: >= 2.0 A below 2 rpm
    // for 0.2 s.
    Rig r{lists::kY, [](hal::fake::FakeDeviceProvider& d) { d.fakeDigitalIn(5).setState(false); }};
    hal::fake::FakeMotor& m = r.devices.fakeMotor(7);
    m.setCurrent(shulib::units::Current{2.4});
    m.setVelocity(shulib::units::AngularVelocity{0.0});
    mech::PowerSlot& cata = r.svc.power(kCatapult);
    const auto trips = [&] { return r.faults.raiseCount(FaultCode::MechanismStalled); };
    const auto holdUntilTrip = [&](int already) {
        int n = 0;
        while (trips() == already && n < 200) {
            cata.drive(6_volt);  // held: restated every tick
            r.svc.tickFromDriverLoop();
            r.world.pace();
            ++n;
        }
        return n;
    };
    // 1. a hold-to-run into the jam trips after its window
    const int first = holdUntilTrip(0);
    REQUIRE(trips() == 1);
    CHECK(first >= 20);
    cata.stop();  // the button let go; the jam still draws (the hand-held reading)
    r.pace(5);
    // 2. pressed AGAIN: its OWN window — the old one (still open) must not trip it at once
    const int second = holdUntilTrip(1);
    REQUIRE(trips() == 2);
    CHECK(second >= 20);
    // 3. a STEP after a stall: its own window too (the signature trips it, unasked: STALLED)
    cata.run(6_volt, Until::bound(), 2_s);
    r.paceUntilDone(kCatapult);
    REQUIRE(cata.lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(cata.lastStep().elapsed >= 0.2 - 1e-9);
    cata.run(6_volt, Until::bound(), 2_s);  // again, the jam still drawing
    r.paceUntilDone(kCatapult);
    REQUIRE(cata.lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(cata.lastStep().elapsed >= 0.2 - 1e-9);
    CHECK(trips() == 4);
}

TEST_CASE("MECH-A2 power P13: bound from BARE SPECS (no robot's list), a missing sensor's line says its port is unknown to the service") {
    hal::fake::FakeClock clock;
    hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeMotor m;
    std::array<hal::IMotor*, 1> ms{&m};
    hal::MotorMechanism group{ms, hal::BrakeMode::Hold, "catapult"};
    const mech::MechanismSpec spec = ry::catapult.spec();  // declares a limit switch ...
    const mech::Devices d{.motors = &group};               // ... not built this boot
    mech_rig::World world;
    world.clock = &clock;
    mech::MechanismService svc{world};
    svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults, sink);
    mech::PowerSlot& cata = svc.power(0);
    REQUIRE(cata.pieceSensorMissing());
    char where[48];
    cata.pieceSensorPlace(where, sizeof where);
    CHECK(std::string{where} == "its port unknown to the service");
    cata.run(6_volt, Until::sees(), 0.2_s);
    for (int i = 0; i < 100 && cata.running(); ++i) {
        svc.pace();
    }
    CHECK(cata.lastStep().outcome == MechanismOutcome::Unconfirmed);
    CHECK(cata.lastStep().countedDone);
    CHECK(sink.last().message ==
          "'catapult': UNCONFIRMED -- time in 0.20s (its piece sensor is missing this boot, its port unknown to the "
          "service; counted done)");
}
