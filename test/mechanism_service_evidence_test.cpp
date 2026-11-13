// Piece sensors as POSITIVE evidence, and the READ-FAULT rule (chunk MECH-A2; the MECH brief's N8 as
// amended, D7, D8, landmines 21-22, and its R probe as tests).
//
// THE FAILURE THIS FILE EXISTS FOR: a PROS adapter answers a failed read with its last good value —
// so a distance sensor whose cable came out while the intake was EMPTY (or that was dead from boot:
// its hold starts at the far no-object distance) is byte for byte PROS's 9999 "no object" reading,
// and an optical or a switch holds its last level. An eject step "until it stops seeing the piece"
// then SUCCEEDS at once with the motor never run — a false success (the R probe: R2, R3, R8). The
// rule: a sample taken while the sensor's read-fault count moved since the end of the service's last
// tick is UNKNOWN, and UNKNOWN satisfies neither condition.
//
// The R cases run through the REAL hal::pros adapters made by the REAL ProsDeviceProvider over the
// PROS shim (a pulled cable = the shim's `disconnected`: every read answers PROS_ERR) — the code a
// robot runs, not a fake standing in for it. One case runs WITHOUT the channel to show the false
// success it closes is real (so a mutation that removes the rule has something to turn red).

#include "doctest.h"

#include <cmath>
#include <limits>
#include <span>
#include <string>

#include "mechanism_fixtures.hpp"
#include "mechanism_service_rig.hpp"
#include "pros/shim_control.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_digital_in.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_optical.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
using mech::DoneBy;
using mech::MechanismOutcome;
using mech::Until;
using Seen = mech::Slot::Seen;
using mech_rig::kNoObjectIn;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using namespace shulib::units::literals;

struct NoPace final : shulib::motion::ITickPacer {
    void pace() override {}
};

namespace lists {
using namespace shulib::mechanism::words;
/// Robot y's power and pneumatic half: an OPTICAL roller (sensor 9, 0.5), a limit-switch catapult
/// (line 'E' = 5), a DISTANCE grip (sensor 8, 1.5 in).
inline constexpr MechanismTable kYHalf = mechanisms(wire(ry::roller, motors(6), pieceSensor(9)),
                                                    wire(ry::catapult, motors(7), pieceSensor('E')),
                                                    wire(ry::grip, lines('A'), pieceSensor(8)));
/// Robot y's roller alone.
inline constexpr MechanismTable kRoller = mechanisms(wire(ry::roller, motors(6), pieceSensor(9)));
}  // namespace lists

/// One power step on a REAL piece-sensor adapter (the distance on port 9 or the optical on port 8),
/// the probe's loop verbatim: `during(i)`, a driver-loop tick, count a tick the motor ran, 10 ms.
struct RResult {
    MechanismOutcome outcome;
    DoneBy doneBy;
    bool counted;
    double elapsed;
    int ran;
    std::string line;
};
template <class Prep, class During>
RResult powerCase(const mech::MechanismSpec& spec, bool optical, bool withChannel, Until until, double bound, Prep prep,
                  During during) {
    pros::shim::resetDistances();
    pros::shim::resetOpticals();
    shulib::hal::fake::FakeDeviceRegistry reg;
    reg.setKind(9, hal::DeviceKind::Distance);
    reg.setKind(8, hal::DeviceKind::Optical);
    hal::pros::ProsDeviceProvider prov{reg};
    prep();
    hal::fake::FakeClock clock;
    hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeMotor m;
    hal::IMotor* ms[1]{&m};
    hal::MotorMechanism group{std::span<hal::IMotor* const>{ms, 1}, hal::BrakeMode::Coast, spec.name};
    mech::Devices d{.motors = &group};
    if (optical) {
        d.pieceOptical = prov.optical(8);
    } else {
        d.pieceDistance = prov.distance(9);
    }
    REQUIRE((optical ? static_cast<const void*>(d.pieceOptical) : static_cast<const void*>(d.pieceDistance)) != nullptr);
    if (withChannel) {
        d.pieceFaults = prov.readFaults(optical ? 8 : 9);
        REQUIRE(d.pieceFaults != nullptr);
    }
    // the boot's first read happened before the cable came out (the census saw the sensor)
    if (optical) {
        (void)d.pieceOptical->proximity();
    } else {
        (void)d.pieceDistance->distance();
        (void)d.pieceDistance->confidence();
    }
    NoPace np;
    mech::MechanismService svc{np};
    svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults, sink);
    svc.power(0).run(Voltage{optical ? 12.0 : -6.0}, until, Time{bound});
    int ran = 0;
    for (int i = 0; i < 1000 && svc.power(0).running(); ++i) {
        during(i);
        svc.tickFromDriverLoop();
        if (std::abs(svc.power(0).commandedVolts()) > 0.1) {
            ++ran;
        }
        clock.advance(Time{0.01});
    }
    const mech::StepReport& r = svc.power(0).lastStep();
    return RResult{r.outcome, r.doneBy, r.countedDone, r.elapsed, ran, sink.size() > 0 ? sink.last().message : ""};
}

}  // namespace

// ═══ THE EVIDENCE TABLE ══════════════════════════════════════════════════════════════════

TEST_CASE("MECH-A2 evidence: seen() -- YES and NO only on a fresh reading, UNKNOWN otherwise, for each kind of piece sensor") {
    // robot y: an OPTICAL roller (0.5), a limit-switch catapult, a DISTANCE grip (1.5 in)
    mech_rig::Rig r{lists::kYHalf};
    const mech::Slot& roller = r.svc.slot(0);
    const mech::Slot& catapult = r.svc.slot(1);
    const mech::Slot& grip = r.svc.slot(2);
    hal::fake::FakeDistance& dist = r.devices.fakeDistance(8);
    hal::fake::FakeOptical& opt = r.devices.fakeOptical(9);
    hal::fake::FakeDigitalIn& sw = r.devices.fakeDigitalIn(5);

    // DISTANCE (1.5 in): near WITH confidence = YES; near with confidence 0 = UNKNOWN (a held value);
    // at or past the threshold = NO whatever the confidence (9999's honest 393.66 in has confidence 0)
    dist.setDistance(Length{1.0});
    dist.setConfidence(1.0);
    CHECK(grip.seen() == Seen::Yes);
    CHECK(grip.sees());
    dist.setConfidence(0.0);
    CHECK(grip.seen() == Seen::Unknown);
    CHECK_FALSE(grip.sees());
    dist.setDistance(Length{1.5});
    CHECK(grip.seen() == Seen::No);  // AT the threshold: not closer than it
    dist.setDistance(Length{kNoObjectIn});
    CHECK(grip.seen() == Seen::No);
    dist.setConfidence(1.0);
    CHECK(grip.seen() == Seen::No);
    dist.setDistance(Length{1.4999});
    CHECK(grip.seen() == Seen::Yes);
    // OPTICAL (0.5): at or above = YES, below = NO
    opt.setProximity(0.5);
    CHECK(roller.seen() == Seen::Yes);
    opt.setProximity(0.49);
    CHECK(roller.seen() == Seen::No);
    // SWITCH (pressed = true, HA-184): pressed = YES, released = NO
    sw.setState(true);
    CHECK(catapult.seen() == Seen::Yes);
    sw.setState(false);
    CHECK(catapult.seen() == Seen::No);
    // a reading that is NOT A NUMBER is no evidence either way (the F4 seams promise finite readings;
    // a comparison with NaN is false, so an unguarded optical would read NaN as "no piece")
    const double nan = std::nan("");
    opt.setProximity(nan);
    CHECK(roller.seen() == Seen::Unknown);
    dist.setDistance(Length{nan});
    CHECK(grip.seen() == Seen::Unknown);
    dist.setDistance(Length{1.0});
    dist.setConfidence(nan);
    CHECK(grip.seen() == Seen::Unknown);
    // an INFINITE confidence (the seam promises [0, 1]) beside a near distance: not a reading either --
    // the only confidence the `> 0` test alone would take as YES (a NaN fails it by itself)
    dist.setConfidence(std::numeric_limits<double>::infinity());
    CHECK(grip.seen() == Seen::Unknown);
    dist.setDistance(Length{std::numeric_limits<double>::infinity()});
    dist.setConfidence(1.0);
    CHECK(grip.seen() == Seen::Unknown);
    // a mechanism with NO piece sensor: UNKNOWN, never NO
    mech_rig::Rig two{mech_fixture::robot_two::kRobotTwoMechanisms};
    CHECK(two.svc.slot(0).seen() == Seen::Unknown);
    CHECK_FALSE(two.svc.slot(0).declaresPieceSensor());
    CHECK(two.svc.slot(2).seen() == Seen::Unknown);
}

TEST_CASE("MECH-A2 evidence: a read-fault count that MOVED makes every kind's sample UNKNOWN; fresh again once the device answers, from the next tick") {
    mech_rig::Rig r{lists::kYHalf};
    r.devices.fakeDistance(8).setDistance(Length{1.0});
    r.devices.fakeDistance(8).setConfidence(1.0);
    r.devices.fakeOptical(9).setProximity(0.9);
    r.devices.fakeDigitalIn(5).setState(true);
    struct Case {
        std::size_t slot;
        hal::fake::FakeReadFaults* faults;
    };
    for (const Case c : {Case{0, &r.devices.fakeReadFaults(9)}, Case{1, &r.devices.fakeLineReadFaults(5)},
                         Case{2, &r.devices.fakeReadFaults(8)}}) {
        CAPTURE(c.slot);
        const mech::Slot& s = r.svc.slot(c.slot);
        REQUIRE(s.seen() == Seen::Yes);
        c.faults->failing(true);  // the device stops answering: each read is screened
        CHECK(s.seen() == Seen::Unknown);
        CHECK(s.seen() == Seen::Unknown);  // two reads in one tick AGREE (landmine 22)
        r.pace();
        CHECK(s.seen() == Seen::Unknown);  // still failing: still a held value
        c.faults->failing(false);          // it answers again ...
        CHECK(s.seen() == Seen::Unknown);  // ... but the count moved since the last tick's end
        r.pace();
        CHECK(s.seen() == Seen::Yes);  // fresh from the next tick
    }
}

TEST_CASE("MECH-A2 evidence (HA-184): a limit switch whose PRESSED level is LOW -- low reads YES, high reads NO, and a step reads it so") {
    // No word sets `pressedLevel` yet (every declaration's is the default, true), but a spec built by
    // hand — and G3a's generated robot file — can: the service must compare against it, never assume.
    hal::fake::FakeClock clock;
    hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeMotor m;
    hal::IMotor* ms[1]{&m};
    hal::MotorMechanism group{std::span<hal::IMotor* const>{ms, 1}, hal::BrakeMode::Hold, "catapult"};
    hal::fake::FakeDigitalIn sw;
    mech::MechanismSpec spec = ry::catapult.spec();
    REQUIRE(spec.pieceSensor.kind == mech::SensorKind::Switch);
    REQUIRE(spec.pieceSensor.pressedLevel);  // the declarations' default
    spec.pieceSensor.pressedLevel = false;   // a switch that reads LOW when pressed
    REQUIRE(mech::specRefusal(spec) == mech::Refusal::None);
    const mech::Devices d{.motors = &group, .pieceSwitch = &sw};
    NoPace np;
    mech::MechanismService svc{np};
    svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults, sink);
    sw.setState(false);
    CHECK(svc.slot(0).seen() == Seen::Yes);
    sw.setState(true);
    CHECK(svc.slot(0).seen() == Seen::No);
    // a step: `seesPiece` ends on the LOW at its first tick; `stopsSeeingPiece` on the HIGH
    sw.setState(false);
    svc.power(0).run(Voltage{6.0}, Until::sees(), Time{1.0});
    svc.tickFromDriverLoop();
    CHECK(svc.power(0).lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(svc.power(0).lastStep().doneBy == DoneBy::Sees);
    sw.setState(true);
    svc.power(0).run(Voltage{-6.0}, Until::stopsSeeing(), Time{1.0});
    svc.tickFromDriverLoop();
    CHECK(svc.power(0).lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(svc.power(0).lastStep().doneBy == DoneBy::StopsSeeing);
}

TEST_CASE("MECH-A2 evidence (landmine 22): a read fault counted BEFORE bind (the boot census) is not charged to the first tick -- the first sample is fresh") {
    // The slot takes its first read-fault snapshot at BIND: a sensor whose count moved during the boot
    // (one failed read, then answering) must read fresh on the first tick — not UNKNOWN, which would
    // turn an honest TIMED_OUT into UNCONFIRMED ("stopped answering") and slip a seen piece one tick.
    for (const bool piece : {false, true}) {
        CAPTURE(piece);
        mech_rig::Rig r{lists::kRoller, [piece](hal::fake::FakeDeviceProvider& d) {
                            d.fakeOptical(9).setProximity(piece ? 0.9 : 0.0);
                            hal::fake::FakeReadFaults& f = d.fakeReadFaults(9);
                            f.failing(true);
                            REQUIRE(f.faultedReads() == 1);  // one read failed during the boot ...
                            f.failing(false);                // ... then the sensor answers again
                        }};
        REQUIRE(r.devices.fakeReadFaults(9).faultedReads() == 1);
        mech::PowerSlot& roller = r.svc.power(0);
        roller.run(12_volt, Until::sees(), 1_s);
        r.paceUntilDone(0);
        if (piece) {
            CHECK(roller.lastStep().outcome == MechanismOutcome::Succeeded);
            CHECK(roller.lastStep().elapsed == 0.0);  // at the FIRST tick
        } else {
            CHECK(roller.lastStep().outcome == MechanismOutcome::TimedOut);
            CHECK(r.lastMech() == "'roller': TIMED_OUT in 1.00s");
        }
    }
}

// ═══ THE R PROBE, through the REAL adapters ══════════════════════════════════════════════

TEST_CASE("MECH-A2 evidence R: through the REAL ProsDistance / ProsOptical -- a sensor that FAILS is never read as an answer") {
    const mech::MechanismSpec intake = rx::intake.spec();  // a distance sensor, 2 in
    const mech::MechanismSpec roller = ry::roller.spec();  // an optical sensor, 0.5
    auto& ds = pros::shim::distanceState(9);
    auto& os = pros::shim::opticalState(8);
    const auto nothing = [](int) {};

    SUBCASE("R1 a working sensor, the piece leaves at 0.5 s (eject): SUCCEEDED") {
        const RResult r = powerCase(intake, false, true, Until::stopsSeeing(), 2.0, [&] { ds.distanceMm = 25; },
                                    [&](int i) {
                                        if (i == 50) {
                                            ds.distanceMm = 9999;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Succeeded);
        CHECK(r.doneBy == DoneBy::StopsSeeing);
        CHECK(r.elapsed == doctest::Approx(0.50));
        CHECK(r.ran == 50);
    }
    SUBCASE("R2 the cable out while EMPTY, then a piece loaded; eject: UNCONFIRMED, NOT counted, the motor RUN") {
        const RResult r = powerCase(intake, false, true, Until::stopsSeeing(), 1.0, [&] { ds.distanceMm = 9999; },
                                    [&](int i) {
                                        if (i == 0) {
                                            ds.disconnected = true;
                                            ds.distanceMm = 25;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(r.counted);
        CHECK(r.elapsed == doctest::Approx(1.00));
        CHECK(r.ran == 100);
        CHECK(r.line == "'intake': UNCONFIRMED in 1.00s (its piece sensor stopped answering: no evidence on 101 of 101 reads)");
    }
    SUBCASE("R3 the cable out from boot (the hold's initial far value); eject: UNCONFIRMED, NOT counted") {
        const RResult r = powerCase(intake, false, true, Until::stopsSeeing(), 1.0, [&] {
            ds.disconnected = true;
            ds.distanceMm = 25;
        }, nothing);
        CHECK(r.outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(r.counted);
        CHECK(r.ran == 100);
    }
    SUBCASE("R4 a piece present, the cable out at 0.3 s (held near); eject: UNCONFIRMED") {
        const RResult r = powerCase(intake, false, true, Until::stopsSeeing(), 1.0, [&] { ds.distanceMm = 25; },
                                    [&](int i) {
                                        if (i == 30) {
                                            ds.disconnected = true;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Unconfirmed);
        CHECK(r.ran == 100);
        CHECK(r.line.find("no evidence on 71 of 101 reads") != std::string::npos);
    }
    SUBCASE("R5 the cable out, a piece there; intake until it sees one: UNCONFIRMED (never TIMED_OUT: it could not look)") {
        const RResult r = powerCase(intake, false, true, Until::sees(), 1.0, [&] { ds.distanceMm = 9999; },
                                    [&](int i) {
                                        if (i == 0) {
                                            ds.disconnected = true;
                                            ds.distanceMm = 25;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Unconfirmed);
        CHECK(r.ran == 100);
    }
    SUBCASE("R6 the cable out; seesPiece.orAfter(0.5 s): SUCCEEDED by time (the team's own choice)") {
        const RResult r = powerCase(intake, false, true, Until::sees().orAfter(Time{0.5}), 1.0,
                                    [&] { ds.distanceMm = 9999; }, [&](int i) {
                                        if (i == 0) {
                                            ds.disconnected = true;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Succeeded);
        CHECK(r.doneBy == DoneBy::Time);
        CHECK(r.elapsed == doctest::Approx(0.50));
        CHECK(r.ran == 50);
    }
    SUBCASE("R7 a 0.2 s drop-out, then the sensor answers with the piece GONE: SUCCEEDED on the fresh read") {
        const RResult r = powerCase(intake, false, true, Until::stopsSeeing(), 2.0, [&] { ds.distanceMm = 25; },
                                    [&](int i) {
                                        if (i == 20) {
                                            ds.disconnected = true;
                                            ds.distanceMm = 9999;
                                        }
                                        if (i == 40) {
                                            ds.disconnected = false;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Succeeded);
        CHECK(r.doneBy == DoneBy::StopsSeeing);
        CHECK(r.elapsed == doctest::Approx(0.40));
        CHECK(r.ran == 40);
    }
    SUBCASE("R8 an OPTICAL that failed while empty, a piece fed; until it stops seeing: UNCONFIRMED") {
        const RResult r = powerCase(roller, true, true, Until::stopsSeeing(), 1.0, [&] { os.proximity = 0; },
                                    [&](int i) {
                                        if (i == 0) {
                                            os.disconnected = true;
                                            os.proximity = 255;
                                        }
                                    });
        CHECK(r.outcome == MechanismOutcome::Unconfirmed);
        CHECK_FALSE(r.counted);
        CHECK(r.ran == 100);
    }
    SUBCASE("WHY: R3 WITHOUT the read-fault channel is the false success -- 'stopped seeing' at 0.00 s, the motor never run") {
        const RResult r = powerCase(intake, false, false, Until::stopsSeeing(), 1.0, [&] {
            ds.disconnected = true;
            ds.distanceMm = 25;
        }, nothing);
        CHECK(r.outcome == MechanismOutcome::Succeeded);
        CHECK(r.elapsed == 0.0);
        CHECK(r.ran == 0);
    }
}

TEST_CASE("MECH-A2 evidence J: a distance held NEAR with confidence 0 (no read-fault channel at all) is UNKNOWN -- the eject ends UNCONFIRMED, not 'gone'") {
    hal::fake::FakeClock clock;
    hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    hal::fake::FakeMotor m;
    hal::IMotor* ms[1]{&m};
    hal::MotorMechanism group{std::span<hal::IMotor* const>{ms, 1}, hal::BrakeMode::Coast, "intake"};
    hal::fake::FakeDistance sensor;
    sensor.setDistance(Length{1.0});
    sensor.setConfidence(1.0);  // a piece at 1 in
    NoPace np;
    mech::MechanismService svc{np};
    const mech::MechanismSpec spec = rx::intake.spec();
    const mech::Devices d{.motors = &group, .pieceDistance = &sensor};
    svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults, sink);
    svc.power(0).run(Voltage{-6.0}, Until::stopsSeeing(), Time{2.0});
    for (int i = 0; i < 300 && svc.power(0).running(); ++i) {
        if (i == 30) {
            sensor.setConfidence(0.0);  // the adapter's PROS_ERR path: confidence 0, distance held at 1 in
        }
        svc.tickFromDriverLoop();
        clock.advance(Time{0.01});
    }
    CHECK(svc.power(0).lastStep().outcome == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(svc.power(0).lastStep().countedDone);
    CHECK(svc.power(0).lastStep().elapsed == doctest::Approx(2.0));
    CHECK(sink.last().message.find("stopped answering: no evidence on 171 of 201 reads") != std::string::npos);
}

TEST_CASE("MECH-A2 evidence: a sensor blind for PART of a step that then answers NO to the end is still UNCONFIRMED -- it may have missed the piece") {
    mech_rig::Rig r{lists::kRoller};
    r.devices.fakeOptical(9).setProximity(0.0);
    mech::PowerSlot& roller = r.svc.power(0);
    roller.run(12_volt, Until::sees(), 1_s);
    r.devices.fakeReadFaults(9).failing(true);
    r.pace(20);  // blind for 0.2 s -- a piece could have come and gone
    r.devices.fakeReadFaults(9).failing(false);
    r.paceUntilDone(0);
    CHECK(roller.lastStep().outcome == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(roller.lastStep().countedDone);
    CHECK(r.lastMech().find("stopped answering: no evidence on 20 of 101 reads") != std::string::npos);
    // ... while a sensor that answered NO on every sample is an honest TIMED_OUT
    roller.run(12_volt, Until::sees(), 1_s);
    r.paceUntilDone(0);
    CHECK(roller.lastStep().outcome == MechanismOutcome::TimedOut);
}

TEST_CASE("MECH-A2 evidence: ONE blind sample in a whole step is enough for UNCONFIRMED -- the boundary (the coordinator's MB4, 2026-10-15)") {
    // The piece could have passed on exactly the one tick the sensor could not look: one is the boundary
    // the rule is written at ("no evidence on SOME sample"), and the 20-sample case above cannot see it.
    mech_rig::Rig r{lists::kRoller};
    r.devices.fakeOptical(9).setProximity(0.0);
    mech::PowerSlot& roller = r.svc.power(0);
    roller.run(12_volt, Until::sees(), 1_s);
    r.devices.fakeReadFaults(9).failing(true);
    r.pace(1);
    r.devices.fakeReadFaults(9).failing(false);
    r.paceUntilDone(0);
    CHECK(roller.lastStep().outcome == MechanismOutcome::Unconfirmed);
    CHECK_FALSE(roller.lastStep().countedDone);
    CHECK(r.lastMech().find("stopped answering: no evidence on 1 of 101 reads") != std::string::npos);
}
