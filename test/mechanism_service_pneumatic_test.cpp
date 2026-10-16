// The PNEUMATIC behaviour (chunk MECH-A2, the MECH brief's §1.7 "Pneumatic" and §3.2): a set()
// writes each line's level NOW; the verdict comes after the move time — confirmed by a piece sensor
// when the position asks for one, counted done UNCONFIRMED when there is none (the team lead's
// Q2 (c)); the end-of-run guard's safe state is a declared POSITION across every line.
//
// Each case tries to break one rule: each position's level on EACH line (robot x's three-position
// hood on two lines); the safe POSITION, not line 0's level fanned out (a hood whose safe position
// is "middle" = High, Low tells them apart, which a safe position of all-Low cannot); a sensorless
// grab COUNTED done; a sensor that never sees NOT counted; a sensor that stops answering in the
// window said so; the claim held only until the verdict while the level persists.

#include "doctest.h"

#include <array>
#include <span>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;
using mech::DoneBy;
using mech::MechanismOutcome;
using mech_rig::kNoObjectIn;
using mech_rig::Rig;
using shulib::units::Length;
using namespace shulib::units::literals;

namespace lists {
using namespace shulib::mechanism::words;
inline constexpr MechanismTable kHood = mechanisms(wire(rx::hood, lines('C', 'D')));
/// A hood whose START and SAFE positions are "middle" (High on line 1, Low on line 2): the per-line
/// levels differ, so a safe state that fanned line 0's level out would land High, High = "up".
inline constexpr Pneumatic midHood{.name = "hood",
                                   .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
                                   .start = "middle",
                                   .safe = "middle",
                                   .moveTime = 0.3_s};
inline constexpr MechanismTable kMidHood = mechanisms(wire(midHood, lines('C', 'D')));
inline constexpr MechanismTable kGrip = mechanisms(wire(ry::grip, lines('A'), pieceSensor(8)));
}  // namespace lists

struct HoodLines {
    hal::fake::FakeDigitalOut& c;
    hal::fake::FakeDigitalOut& d;
    [[nodiscard]] bool is(bool l1, bool l2) const { return c.commanded() == l1 && d.commanded() == l2; }
};

}  // namespace

TEST_CASE("MECH-A2 pneumatic: robot x's hood on TWO lines -- each position's level on EACH line, then SUCCEEDED after its move time") {
    Rig r{lists::kHood};
    HoodLines lines{r.devices.fakeDigitalOut(3), r.devices.fakeDigitalOut(4)};
    mech::PneumaticSlot& hood = r.svc.pneumatic(0);
    CHECK(hood.commandedPosition() == 0);  // "down" at boot (the parts made the lines at START)
    CHECK(lines.is(false, false));
    struct Want {
        std::uint8_t p;
        bool l1, l2;
    };
    for (const Want w : {Want{1, true, false}, Want{2, true, true}, Want{0, false, false}, Want{2, true, true}}) {
        CAPTURE(w.p);
        hood.set(w.p);
        CHECK(lines.is(w.l1, w.l2));  // the lines NOW, before any tick
        CHECK(hood.commandedPosition() == w.p);
        CHECK(hood.running());
        CHECK(hood.holdsClaim());
        const int n = r.paceUntilDone(0);
        CHECK(n == 31);  // ticks at 0.00 .. 0.30: the verdict at the move time
        CHECK(hood.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(hood.lastStep().doneBy == DoneBy::Time);
        CHECK_FALSE(hood.holdsClaim());  // released at the verdict ...
        CHECK(lines.is(w.l1, w.l2));     // ... the level persists
    }
    CHECK(r.lastMech() == "'hood': SUCCEEDED -- time in 0.30s");
    CHECK(hood.lastStepBound() == doctest::Approx(0.3));
}

TEST_CASE("MECH-A2 pneumatic: the guard's safe state is the declared safe POSITION on every line -- claimed mid-step and unclaimed") {
    {
        Rig r{lists::kHood};  // safe = "down" = Low, Low
        HoodLines lines{r.devices.fakeDigitalOut(3), r.devices.fakeDigitalOut(4)};
        mech::PneumaticSlot& hood = r.svc.pneumatic(0);
        hood.set(1);  // "middle"
        r.pace(5);
        hal::IMechanism* m = r.svc.mechanisms()[0];
        REQUIRE(m->claimant() == &hood);
        m->claimant()->cancel();  // the guard's reach, mid-step
        m->applySafeState();
        CHECK(lines.is(false, false));
        CHECK(hood.lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(hood.commandedPosition() == 0);  // read back from the lines: the guard's work shows
    }
    {
        Rig r{lists::kMidHood};  // start AND safe = "middle" = High, Low
        HoodLines lines{r.devices.fakeDigitalOut(3), r.devices.fakeDigitalOut(4)};
        mech::PneumaticSlot& hood = r.svc.pneumatic(0);
        CHECK(lines.is(true, false));  // made at START per line by the parts
        CHECK(hood.commandedPosition() == 1);
        hood.set(2);  // "up" = High, High
        r.paceUntilDone(0);
        REQUIRE(lines.is(true, true));
        r.svc.mechanisms()[0]->applySafeState();  // unclaimed: the run's own cancel-all
        CHECK(lines.is(true, false));             // "middle" -- NOT line 0's High fanned out (High, High)
        CHECK(hood.commandedPosition() == 1);
        hood.set(0);
        r.pace(3);
        r.svc.mechanisms()[0]->claimant()->cancel();  // claimed: the cancel applies it
        CHECK(lines.is(true, false));
    }
}

TEST_CASE("MECH-A2 pneumatic: robot two's claw -- a sensorless GRAB is UNCONFIRMED and COUNTED done; a position that confirms nothing SUCCEEDS") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    hal::fake::FakeDigitalOut& lineA = r.devices.fakeDigitalOut(1);
    claw.set(1);  // open: no confirmation asked
    r.paceUntilDone(2);
    CHECK(lineA.commanded());
    CHECK(claw.lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.lastMech() == "'claw': SUCCEEDED -- time in 0.50s");
    claw.set(0);  // closed: `grabs`, and the claw has NO sensor
    CHECK_FALSE(lineA.commanded());
    r.paceUntilDone(2);
    CHECK(claw.lastStep().outcome == MechanismOutcome::Unconfirmed);
    CHECK(claw.lastStep().countedDone);
    CHECK(claw.lastStep().doneBy == DoneBy::Time);
    CHECK(claw.lastStep().elapsed == doctest::Approx(0.5));
    CHECK(r.lastMech() == "'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)");
    CHECK(r.lastMechLevel() == hal::LogLevel::Warn);
    CHECK_FALSE(lineA.commanded());
    CHECK_FALSE(claw.holdsClaim());
}

TEST_CASE("MECH-A2 pneumatic: a grip WITH a sensor -- SUCCEEDED when it sees within the window; UNCONFIRMED, NOT counted, when it never does") {
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) {
                  d.fakeDistance(8).setDistance(Length{1.0});
                  d.fakeDistance(8).setConfidence(1.0);
              }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(0);  // "shut": grabs
        r.paceUntilDone(0);
        CHECK(grip.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(grip.lastStep().doneBy == DoneBy::Sees);
        CHECK(r.lastMech() == "'grip': SUCCEEDED -- sees a piece in 0.25s");
    }
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(8).setDistance(Length{kNoObjectIn}); }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(0);
        const int n = r.paceUntilDone(0);
        CHECK(grip.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(grip.lastStep().countedDone);
        CHECK(grip.lastStep().elapsed == doctest::Approx(0.45));  // move time 0.25 + window 0.2
        CHECK(n == 46);
        CHECK(r.lastMech() == "'grip': UNCONFIRMED in 0.45s (its piece sensor did not see a piece)");
        CHECK(grip.lastStepBound() == doctest::Approx(0.45));
    }
    {   // the piece arrives INSIDE the window
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(8).setDistance(Length{kNoObjectIn}); }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(0);
        r.pace(35);
        REQUIRE(grip.running());
        r.devices.fakeDistance(8).setDistance(Length{0.8});
        r.devices.fakeDistance(8).setConfidence(1.0);
        r.pace();
        CHECK(grip.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(grip.lastStep().elapsed == doctest::Approx(0.35));
    }
    {   // seen BEFORE the move time is not asked: the world cannot be asked mid-move
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) {
                  d.fakeDistance(8).setDistance(Length{0.8});
                  d.fakeDistance(8).setConfidence(1.0);
              }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(0);
        r.pace(10);
        CHECK(grip.running());
    }
}

TEST_CASE("MECH-A2 pneumatic: a RELEASE confirms the piece is gone; 'still sees a piece' otherwise") {
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(8).setDistance(Length{kNoObjectIn}); }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(1);  // "loose": releases
        r.paceUntilDone(0);
        CHECK(grip.lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(grip.lastStep().doneBy == DoneBy::StopsSeeing);
    }
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) {
                  d.fakeDistance(8).setDistance(Length{1.0});
                  d.fakeDistance(8).setConfidence(1.0);
              }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(1);
        r.paceUntilDone(0);
        CHECK(grip.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(grip.lastStep().countedDone);
        CHECK(r.lastMech() == "'grip': UNCONFIRMED in 0.45s (its piece sensor still sees a piece)");
    }
}

TEST_CASE("MECH-A2 pneumatic: the sensor stops answering in the window -> UNCONFIRMED, NOT counted, said; MISSING this boot -> COUNTED") {
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) { d.fakeDistance(8).setDistance(Length{kNoObjectIn}); }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        grip.set(0);
        r.devices.fakeReadFaults(8).failing(true);  // the cable out
        r.paceUntilDone(0);
        CHECK(grip.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(grip.lastStep().countedDone);
        CHECK(r.lastMech() == "'grip': UNCONFIRMED in 0.45s (its piece sensor stopped answering: no evidence on 21 of 21 reads)");
    }
    {
        Rig r{lists::kGrip, [](hal::fake::FakeDeviceProvider& d) { d.refusePort(8); }};
        mech::PneumaticSlot& grip = r.svc.pneumatic(0);
        REQUIRE(grip.built());
        REQUIRE(grip.pieceSensorMissing());
        grip.set(0);
        const int n = r.paceUntilDone(0);
        CHECK(n == 26);  // counted at the move time: no window to wait out with no sensor
        CHECK(grip.lastStep().outcome == MechanismOutcome::Unconfirmed);
        CHECK(grip.lastStep().countedDone);
        CHECK(r.lastMech() ==
              "'grip': UNCONFIRMED -- time in 0.25s (its piece sensor is missing this boot, port 8; counted done)");
    }
}

TEST_CASE("MECH-A2 pneumatic: set() refuses a position it does not have; a set on robot two's one line writes ONE level") {
    Rig r{r2::kRobotTwoMechanisms};
    mech::PneumaticSlot& claw = r.svc.pneumatic(2);
    bool threw = false;
    try {
        claw.set(2);  // the claw has two positions
    } catch (const shulib::PreconditionError&) {
        threw = true;
    }
    CHECK(threw);
    CHECK(claw.stepsBegun() == 0U);
    claw.set(1);
    CHECK(r.devices.fakeDigitalOut(1).commanded());
    CHECK_FALSE(r.devices.fakeDigitalOut(2).commanded());  // no second line exists to touch
}
