// The mechanism core's DEVICES (chunk MECH-A1, the MECH brief's §3.1 "the provider accessors", "the
// read-fault seam"): the five IDeviceProvider accessors in the on-robot provider (over a faked
// census and the host PROS shim) and the host one, hal::IReadFaults through the three real
// adapters, and hal/mechanism.hpp's two additions (MotorMechanism::meanPosition, the per-line
// PneumaticMechanism).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * THE KIND CHECK (mutation M3: distance() skips expectKind): EVERY kind the registry can report
//     is put on a port, and only DISTANCE serves distance() (only OPTICAL serves optical()); every
//     other refusal reads "port N: <what was found> (expected a distance sensor)". Ports 0 and 22
//     refused. The same port twice is the SAME object.
//   * ONE LINE, ONE DIRECTION: a 3-wire line made as an output and then asked for as an input — and
//     the reverse — is a precondition, in BOTH providers.
//   * THE READ-FAULT SEAM (mutation M41: ProsDistance loses IReadFaults): each adapter's count reached
//     THROUGH IReadFaults equals its own; readFaults(port) is the very adapter made there and nullptr
//     wherever none was made — and asking never MAKES one. And the reason the seam exists, shown: a
//     distance sensor dead from boot reads byte for byte the 9999 "no object" reading; only the count
//     tells them apart.
//   * The fake's counters move ONLY while failing, and only where a sensor was served.
//   * IDEMPOTENT means not re-made (mutations P1/P2 of MECH-A1's review: an adapter re-made in the same
//     storage keeps its address): a second request asks no registry and keeps the read-fault count.
//   * The per-line pneumatic: applySafeState() drives EACH line to ITS level (mutation M13 —
//     every line to line 0's level — is red here); the single-level form is untouched.
// NO HARDWARE CLAIM: the shim tests the adapters against our belief about PROS (HA-138, HA-141); the
// belief is the robot's to settle.

#include "doctest.h"

#include <array>
#include <span>
#include <string>

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_digital_out.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_read_faults.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/hal/pros/digital_in.hpp"
#include "shulib/hal/pros/distance.hpp"
#include "shulib/hal/pros/optical.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::DeviceKind;
using shulib::hal::IReadFaults;
using shulib::hal::fake::FakeDeviceProvider;
using shulib::hal::fake::FakeDeviceRegistry;
using shulib::hal::pros::ProsDeviceProvider;

namespace {

constexpr DeviceKind kEveryKind[] = {DeviceKind::None,     DeviceKind::Motor, DeviceKind::Imu,
                                     DeviceKind::Rotation, DeviceKind::Distance, DeviceKind::Optical,
                                     DeviceKind::Vision,   DeviceKind::Gps,   DeviceKind::Radio,
                                     DeviceKind::Adi,      DeviceKind::Other, DeviceKind::AiVision};

const char* foundWords(DeviceKind k) {
    switch (k) {
        case DeviceKind::None: return "nothing plugged in";
        case DeviceKind::Motor: return "a motor";
        case DeviceKind::Imu: return "an IMU";
        case DeviceKind::Rotation: return "a rotation sensor";
        case DeviceKind::Distance: return "a distance sensor";
        case DeviceKind::Optical: return "an optical sensor";
        case DeviceKind::Vision: return "a vision sensor";
        case DeviceKind::Gps: return "a GPS";
        case DeviceKind::Radio: return "the radio";
        case DeviceKind::Adi: return "an ADI expander";
        case DeviceKind::Other: return "an unrecognised device";
        case DeviceKind::AiVision: return "an AI Vision sensor";
    }
    return "?";
}

struct NullPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

/// A host provider with its four borrowed handles.
struct FakeWorld {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    NullPacer pacer;
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

}  // namespace

// ═══ 1. The on-robot provider ════════════════════════════════════════════════════════

TEST_CASE("MECH-A1 devices: ProsDeviceProvider::distance()/optical() serve ONLY their own kind, naming what was found") {
    for (const DeviceKind k : kEveryKind) {
        CAPTURE(static_cast<int>(k));
        pros::shim::resetAll();
        FakeDeviceRegistry reg;
        reg.setKind(5, k);
        {
            ProsDeviceProvider prov{reg};
            shulib::hal::IDistance* d = prov.distance(5);
            if (k == DeviceKind::Distance) {
                CHECK(d != nullptr);
                CHECK(prov.distance(5) == d);  // one port, ONE object
            } else {
                CHECK(d == nullptr);
                CHECK(std::string{prov.lastRefusal()}
                      == std::string{"port 5: "} + foundWords(k) + " (expected a distance sensor)");
                CHECK(prov.readFaults(5) == nullptr);  // a refused sensor has no count
            }
        }
        {
            ProsDeviceProvider prov{reg};
            shulib::hal::IOptical* o = prov.optical(5);
            if (k == DeviceKind::Optical) {
                CHECK(o != nullptr);
                CHECK(prov.optical(5) == o);
            } else {
                CHECK(o == nullptr);
                CHECK(std::string{prov.lastRefusal()}
                      == std::string{"port 5: "} + foundWords(k) + " (expected an optical sensor)");
            }
        }
    }
    pros::shim::resetAll();
    FakeDeviceRegistry reg;
    ProsDeviceProvider prov{reg};
    CHECK(prov.distance(0) == nullptr);
    CHECK(std::string{prov.lastRefusal()} == "port 0 is not a smart port (ports are 1..21)");
    CHECK(prov.optical(22) == nullptr);
    CHECK(std::string{prov.lastRefusal()} == "port 22 is not a smart port (ports are 1..21)");
}

TEST_CASE("MECH-A1 devices: ProsDeviceProvider::digitalIn() -- refused ONLY outside 1..8, never asks the registry, one direction per line") {
    pros::shim::resetAll();
    FakeDeviceRegistry reg;  // EMPTY: a switch is never the registry's to see (HA-158)
    ProsDeviceProvider prov{reg};
    const int queriesBefore = reg.queries();
    shulib::hal::IDigitalIn* in = prov.digitalIn(2);
    CHECK(in != nullptr);
    CHECK(prov.digitalIn(2) == in);
    CHECK(reg.queries() == queriesBefore);
    CHECK(pros::shim::adiLine(0, 2).writes.empty());  // making an input drives NOTHING
    CHECK(prov.digitalIn(0) == nullptr);
    CHECK(std::string{prov.lastRefusal()} == "3-wire port 0 is not a brain 3-wire port (1..8 are A..H)");
    CHECK(prov.digitalIn(9) == nullptr);
    CHECK(prov.digitalIn(8) != nullptr);
    // One line, ONE direction — both ways round.
    CHECK(prov.digitalOut(3, false) != nullptr);
    CHECK_THROWS_AS((void)prov.digitalIn(3), PreconditionError);
    CHECK_THROWS_AS((void)prov.digitalOut(2, false), PreconditionError);
}

TEST_CASE("MECH-A1 devices: the READ-FAULT seam through the REAL adapters -- and why it exists") {
    pros::shim::resetAll();
    FakeDeviceRegistry reg;
    reg.setKind(9, DeviceKind::Distance);
    reg.setKind(8, DeviceKind::Optical);
    ProsDeviceProvider prov{reg};
    // Asking for a count never MAKES a device.
    CHECK(prov.readFaults(9) == nullptr);
    CHECK(prov.readFaults(8) == nullptr);
    CHECK(prov.lineReadFaults(4) == nullptr);
    shulib::hal::IDistance* d = prov.distance(9);
    shulib::hal::IOptical* o = prov.optical(8);
    shulib::hal::IDigitalIn* sw = prov.digitalIn(4);
    REQUIRE(d != nullptr);
    REQUIRE(o != nullptr);
    REQUIRE(sw != nullptr);
    // readFaults(port) is THAT adapter, as an IReadFaults — the very object.
    const auto* pd = static_cast<const shulib::hal::pros::ProsDistance*>(d);
    const auto* po = static_cast<const shulib::hal::pros::ProsOptical*>(o);
    const auto* pi = static_cast<const shulib::hal::pros::ProsDigitalIn*>(sw);
    // REQUIRE, not CHECK: every line below dereferences these (a missing count must stop this case
    // cleanly, never crash the run and hide the cases after it).
    REQUIRE(prov.readFaults(9) == static_cast<const IReadFaults*>(pd));
    REQUIRE(prov.readFaults(8) == static_cast<const IReadFaults*>(po));
    REQUIRE(prov.lineReadFaults(4) == static_cast<const IReadFaults*>(pi));
    CHECK(prov.readFaults(7) == nullptr);
    CHECK(prov.readFaults(0) == nullptr);
    CHECK(prov.readFaults(22) == nullptr);
    CHECK(prov.lineReadFaults(0) == nullptr);
    CHECK(prov.lineReadFaults(9) == nullptr);
    CHECK(prov.lineReadFaults(5) == nullptr);
    // Healthy: no screen.
    (void)d->distance();
    (void)o->proximity();
    (void)sw->state();
    CHECK(prov.readFaults(9)->faultedReads() == 0);
    CHECK(prov.readFaults(8)->faultedReads() == 0);
    CHECK(prov.lineReadFaults(4)->faultedReads() == 0);
    // Each device fails; each read is screened and counted — and the count reached THROUGH the seam
    // equals the adapter's own.
    pros::shim::distanceState(9).disconnected = true;
    pros::shim::opticalState(8).disconnected = true;
    pros::shim::adiLine(0, 4).broken = true;
    (void)d->distance();
    (void)d->distance();
    (void)o->proximity();
    (void)sw->state();
    (void)sw->state();
    (void)sw->state();
    CHECK(prov.readFaults(9)->faultedReads() == 2);
    CHECK(prov.readFaults(9)->faultedReads() == pd->faultedReads());
    CHECK(prov.readFaults(8)->faultedReads() == 1);
    CHECK(prov.readFaults(8)->faultedReads() == po->faultedReads());
    CHECK(prov.lineReadFaults(4)->faultedReads() == 3);
    CHECK(prov.lineReadFaults(4)->faultedReads() == pi->faultedReads());
}

TEST_CASE("MECH-A1 devices: IDEMPOTENT is more than the same address -- a second request asks no registry and keeps the count") {
    // MECH-A1's review: `CHECK(prov.distance(5) == d)` cannot see an adapter RE-MADE in the same
    // storage (std::optional::emplace keeps the address) — which would reset its last-good value and
    // its read-fault count, the very thing the read-fault channel depends on. So: the registry is not
    // asked again, and a count taken before the second request survives it.
    pros::shim::resetAll();
    FakeDeviceRegistry reg;
    reg.setKind(9, DeviceKind::Distance);
    reg.setKind(8, DeviceKind::Optical);
    ProsDeviceProvider prov{reg};
    shulib::hal::IDistance* d = prov.distance(9);
    shulib::hal::IOptical* o = prov.optical(8);
    shulib::hal::IDigitalIn* sw = prov.digitalIn(4);
    REQUIRE(d != nullptr);
    REQUIRE(o != nullptr);
    REQUIRE(sw != nullptr);
    pros::shim::distanceState(9).disconnected = true;
    pros::shim::opticalState(8).disconnected = true;
    pros::shim::adiLine(0, 4).broken = true;
    (void)d->distance();
    (void)o->proximity();
    (void)sw->state();
    REQUIRE(prov.readFaults(9) != nullptr);
    REQUIRE(prov.readFaults(8) != nullptr);
    REQUIRE(prov.lineReadFaults(4) != nullptr);
    REQUIRE(prov.readFaults(9)->faultedReads() == 1);
    REQUIRE(prov.readFaults(8)->faultedReads() == 1);
    REQUIRE(prov.lineReadFaults(4)->faultedReads() == 1);
    const int queries = reg.queries();
    CHECK(prov.distance(9) == d);
    CHECK(prov.optical(8) == o);
    CHECK(prov.digitalIn(4) == sw);
    CHECK(reg.queries() == queries);                         // the registry was not asked again
    CHECK(prov.readFaults(9)->faultedReads() == 1);          // ... and no adapter was re-made:
    CHECK(prov.readFaults(8)->faultedReads() == 1);          //     each count kept what it had
    CHECK(prov.lineReadFaults(4)->faultedReads() == 1);
}

TEST_CASE("MECH-A1 devices: WHY the channel -- a distance sensor dead from boot reads exactly the 'no object' reading") {
    pros::shim::resetAll();
    pros::shim::distanceState(9) = {};  // 9999 mm: "can not detect an object", the honest far reading
    pros::shim::distanceState(10) = {};
    pros::shim::distanceState(10).disconnected = true;  // PROS_ERR from boot: the hold starts far
    FakeDeviceRegistry reg;
    reg.setKind(9, DeviceKind::Distance);
    reg.setKind(10, DeviceKind::Distance);
    ProsDeviceProvider prov{reg};
    shulib::hal::IDistance* alive = prov.distance(9);
    shulib::hal::IDistance* dead = prov.distance(10);
    REQUIRE(alive != nullptr);
    REQUIRE(dead != nullptr);
    CHECK(alive->distance().value() == dead->distance().value());      // byte for byte the same...
    CHECK(alive->confidence() == dead->confidence());                  // ... confidence too (0)
    REQUIRE(prov.readFaults(9) != nullptr);
    REQUIRE(prov.readFaults(10) != nullptr);
    CHECK(prov.readFaults(9)->faultedReads() == 0);                     // ... and only the count
    CHECK(prov.readFaults(10)->faultedReads() > 0);                     // tells them apart
}

// ═══ 2. The host provider ════════════════════════════════════════════════════════════

TEST_CASE("MECH-A1 devices: FakeDeviceProvider -- its own sensors, refusals naming the port, counts only where served") {
    FakeWorld w;
    FakeDeviceProvider& p = w.devices;
    CHECK(p.readFaults(9) == nullptr);  // nothing served yet: no count, as on the robot
    shulib::hal::IDistance* d = p.distance(9);
    CHECK(d == &p.fakeDistance(9));
    CHECK(p.distance(9) == d);
    CHECK(p.readFaults(9) == &p.fakeReadFaults(9));
    shulib::hal::IOptical* o = p.optical(8);
    CHECK(o == &p.fakeOptical(8));
    CHECK(p.readFaults(8) == &p.fakeReadFaults(8));
    CHECK(p.readFaults(7) == nullptr);
    p.refusePort(6);
    CHECK(p.distance(6) == nullptr);
    CHECK(std::string{p.lastRefusal()} == "port 6 has no distance sensor");
    CHECK(p.optical(6) == nullptr);
    CHECK(std::string{p.lastRefusal()} == "port 6 has no optical sensor");
    CHECK(p.readFaults(6) == nullptr);  // refused: no count
    CHECK(p.distance(0) == nullptr);
    CHECK(p.optical(22) == nullptr);
    CHECK(p.readFaults(0) == nullptr);
    CHECK(p.readFaults(22) == nullptr);
    // Lines.
    CHECK(p.lineReadFaults(3) == nullptr);
    shulib::hal::IDigitalIn* in = p.digitalIn(3);
    CHECK(in == &p.fakeDigitalIn(3));
    CHECK(p.digitalInMade(3));
    CHECK(p.lineReadFaults(3) == &p.fakeLineReadFaults(3));
    CHECK(p.digitalOutRequestCount(3) == 0);  // an input is not an output request
    CHECK_FALSE(p.digitalOutMade(3));
    CHECK(p.digitalIn(9) == nullptr);
    CHECK(std::string{p.lastRefusal()} == "3-wire port 9 is not a brain 3-wire port (1..8 are A..H)");
    p.refuseLine(5);
    CHECK(p.digitalIn(5) == nullptr);
    CHECK(std::string{p.lastRefusal()} == "3-wire port E (line 5) has no digital in");
    CHECK(p.lineReadFaults(5) == nullptr);
    // One line, one direction, both ways — as on the robot.
    CHECK(p.digitalOut(1, true) != nullptr);
    CHECK_THROWS_AS((void)p.digitalIn(1), PreconditionError);
    CHECK_THROWS_AS((void)p.digitalOut(3, false), PreconditionError);
}

TEST_CASE("MECH-A1 devices: FakeReadFaults moves ONLY while failing, and never goes back") {
    shulib::hal::fake::FakeReadFaults f;
    CHECK(f.faultedReads() == 0);
    CHECK(f.faultedReads() == 0);  // reading a healthy count moves nothing
    f.failing(true);
    CHECK(f.isFailing());
    CHECK(f.faultedReads() == 1);
    CHECK(f.faultedReads() == 2);
    f.failing(false);
    CHECK(f.faultedReads() == 2);
    CHECK(f.faultedReads() == 2);  // the total stays: monotonic
    const IReadFaults& seam = f;
    CHECK(seam.faultedReads() == 2);
}

// ═══ 3. hal/mechanism.hpp's additions ════════════════════════════════════════════════

TEST_CASE("MECH-A1 devices: MotorMechanism::meanPosition is the group's mean output-shaft angle") {
    shulib::hal::fake::FakeMotor a;
    shulib::hal::fake::FakeMotor b;
    a.setPosition(shulib::units::AngleDim{2.0});
    b.setPosition(shulib::units::AngleDim{4.0});  // a mirrored mate reads the same way: its adapter applied the sign
    shulib::hal::IMotor* m[2]{&a, &b};
    shulib::hal::MotorMechanism mm{m, shulib::hal::BrakeMode::Hold, "lift"};
    CHECK(mm.meanPosition().value() == doctest::Approx(3.0).epsilon(1e-15));
    b.setPosition(shulib::units::AngleDim{-10.0});  // cumulative and signed: never wrapped
    CHECK(mm.meanPosition().value() == doctest::Approx(-4.0).epsilon(1e-15));
    // One, three and four motors: the mean divides by the GROUP'S size (MECH-A1's review: a mean over
    // two only could not tell `/ size` from `/ 2`).
    shulib::hal::fake::FakeMotor c;
    shulib::hal::fake::FakeMotor e;
    a.setPosition(shulib::units::AngleDim{2.5});
    shulib::hal::IMotor* one[1]{&a};
    shulib::hal::MotorMechanism m1{one, shulib::hal::BrakeMode::Hold, "arm"};
    CHECK(m1.meanPosition().value() == doctest::Approx(2.5).epsilon(1e-15));
    a.setPosition(shulib::units::AngleDim{1.0});
    b.setPosition(shulib::units::AngleDim{2.0});
    c.setPosition(shulib::units::AngleDim{6.0});
    shulib::hal::IMotor* three[3]{&a, &b, &c};
    shulib::hal::MotorMechanism m3{three, shulib::hal::BrakeMode::Hold, "three"};
    CHECK(m3.meanPosition().value() == doctest::Approx(3.0).epsilon(1e-15));
    e.setPosition(shulib::units::AngleDim{-1.0});
    shulib::hal::IMotor* four[4]{&a, &b, &c, &e};
    shulib::hal::MotorMechanism m4{four, shulib::hal::BrakeMode::Hold, "four"};
    CHECK(m4.meanPosition().value() == doctest::Approx(2.0).epsilon(1e-15));
}

TEST_CASE("MECH-A1 devices: the PER-LINE PneumaticMechanism -- a declared safe POSITION, line by line (M13's net)") {
    shulib::hal::fake::FakeDigitalOut l1;
    shulib::hal::fake::FakeDigitalOut l2;
    shulib::hal::IDigitalOut* lines[2]{&l1, &l2};
    SUBCASE("per-line: safe = 'middle' = High on line 1, Low on line 2") {
        const bool safe[2]{true, false};
        shulib::hal::PneumaticMechanism hood{lines, safe, "hood"};
        CHECK(hood.perLine());
        CHECK(hood.safeLevel(0));
        CHECK_FALSE(hood.safeLevel(1));
        const bool up[2]{true, true};
        hood.setLevels(up);
        CHECK(l1.commanded());
        CHECK(l2.commanded());
        CHECK(hood.commandedLevel(0));
        CHECK(hood.commandedLevel(1));
        hood.applySafeState();
        CHECK(l1.commanded());         // each line to ITS level ...
        CHECK_FALSE(l2.commanded());   // ... never line 1's level on line 2
        CHECK(hood.commandedLevel(0));
        CHECK_FALSE(hood.commandedLevel(1));
        const bool wrongCount[1]{true};
        CHECK_THROWS_AS(hood.setLevels(wrongCount), PreconditionError);
        CHECK_THROWS_AS((void)hood.commandedLevel(2), PreconditionError);
        CHECK_THROWS_AS((void)hood.safeLevel(2), PreconditionError);
    }
    SUBCASE("the single-level form is unchanged: every line to its ONE safe level") {
        shulib::hal::PneumaticMechanism claw{lines, false, "claw"};
        CHECK_FALSE(claw.perLine());
        const bool mixed[2]{true, true};
        claw.setLevels(mixed);
        claw.applySafeState();
        CHECK_FALSE(l1.commanded());
        CHECK_FALSE(l2.commanded());
        CHECK_FALSE(claw.safeLevel(1));
        CHECK_FALSE(claw.safeCommand());
    }
    SUBCASE("the per-line form's preconditions") {
        const bool one[1]{true};
        CHECK_THROWS_AS((shulib::hal::PneumaticMechanism{lines, one, "x"}), PreconditionError);  // one level per line
        shulib::hal::IDigitalOut* withNull[2]{&l1, nullptr};
        const bool two[2]{true, false};
        CHECK_THROWS_AS((shulib::hal::PneumaticMechanism{withNull, two, "x"}), PreconditionError);
        CHECK_THROWS_AS((shulib::hal::PneumaticMechanism{lines, two, nullptr}), PreconditionError);
        std::array<shulib::hal::fake::FakeDigitalOut, 5> five{};
        std::array<shulib::hal::IDigitalOut*, 5> fiveLines{&five[0], &five[1], &five[2], &five[3], &five[4]};
        const bool fiveLevels[5]{true, true, true, true, true};
        CHECK_THROWS_AS((shulib::hal::PneumaticMechanism{fiveLines, fiveLevels, "x"}), PreconditionError);
    }
}
