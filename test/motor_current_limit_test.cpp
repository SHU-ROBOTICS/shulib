// hal::IMotorCurrentLimit through EVERY device provider — IDeviceProvider::currentLimit(port) (chunk
// MECH-S-A, 2026-11-05; the MECH-S brief's CA12). Say "host test plant", never "simulator": nothing here
// has run on a robot, and the PROS shim tests the adapter against our BELIEF about PROS (HA-219..HA-221).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * LOOKS UP, NEVER MAKES (readFaults()' rule): asking for a limit on a port no motor was made on
//     answers nullptr AND makes nothing — no motor() request is counted, no device is constructed, the
//     PROS registry is never asked; asking again after the motor is made answers it.
//   * THE SAME OBJECT: the limit answered is the motor made there, by address (a provider that answered
//     a SECOND adapter would cap a motor nobody commands, or read a limit nobody set).
//   * EVERY PORT EDGE: 0, 22, 255 — nullptr, never an out-of-bounds read; 1 and 21 served.
//   * A BOUND motor: a FakeMotor bound with bindMotor(port, FakeMotor&) is its own limit; a bare IMotor
//     bound with bindMotor(port, IMotor&) has none (nullptr — the service then says "this provider cannot
//     cap a motor"); rebinding replaces the limit with the motor.
//   * THE ON-ROBOT PROVIDER over the shim: the ProsMotor it made, writing mA to the DEVICE (the shim's
//     per-port state), a refused port, a port that holds something else.
//   * EVERY test provider that forwards (the calibrate rig's CountingProvider) answers the inner one's.

#include "doctest.h"

#include <cstdint>

#include "calibrate_program_rig.hpp"
#include "pros/shim_control.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/motor_current_limit.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace {
namespace hal = shulib::hal;
using hal::DeviceKind;
using hal::IMotorCurrentLimit;
using hal::MotorCartridge;
using hal::fake::FakeDeviceProvider;
using hal::fake::FakeDeviceRegistry;
using hal::fake::FakeMotor;
using hal::pros::ProsDeviceProvider;
using shulib::units::Current;

struct NoPacer final : shulib::motion::ITickPacer {
    void pace() override {}
};

struct FakeWorld {
    hal::fake::FakeClock clock;
    hal::fake::FakeBattery battery;
    hal::fake::FakeTelemetrySink sink;
    NoPacer pacer;
    FakeDeviceProvider devices{clock, battery, sink, pacer};
};

/// A bare IMotor: no current limit of its own (what a test binds when it has a motor of another kind).
class BareMotor final : public hal::IMotor {
public:
    void setVoltage(shulib::units::Voltage v) override { v_ = v; }
    [[nodiscard]] shulib::units::Voltage commandedVoltage() const override { return v_; }
    void setBrakeMode(hal::BrakeMode m) override { m_ = m; }
    [[nodiscard]] hal::BrakeMode brakeMode() const override { return m_; }
    [[nodiscard]] shulib::units::AngleDim position() const override { return shulib::units::AngleDim{0.0}; }
    [[nodiscard]] shulib::units::AngularVelocity velocity() const override {
        return shulib::units::AngularVelocity{0.0};
    }
    [[nodiscard]] Current current() const override { return Current{0.0}; }
    [[nodiscard]] double temperature() const override { return 20.0; }

private:
    shulib::units::Voltage v_{0.0};
    hal::BrakeMode m_ = hal::BrakeMode::Coast;
};
}  // namespace

TEST_CASE("MECH-S-A CA12: FakeDeviceProvider::currentLimit -- nullptr and NOTHING made before the motor; then that very motor") {
    FakeWorld w;
    // BUG CAUGHT: a lookup that MAKES — the next motor() would then not be the first request, and a
    // port nobody declared would hold a "served" motor.
    for (const std::uint8_t p : {std::uint8_t{1}, std::uint8_t{11}, std::uint8_t{21}}) {
        CAPTURE(static_cast<int>(p));
        CHECK(w.devices.currentLimit(p) == nullptr);
        CHECK(w.devices.motorRequestCount(p) == 0);
        CHECK_FALSE(w.devices.motorRequested(p));
    }
    hal::IMotor* m11 = w.devices.motor(-11, MotorCartridge::Red);  // reversed: the PHYSICAL port answers
    REQUIRE(m11 != nullptr);
    IMotorCurrentLimit* lim = w.devices.currentLimit(11);
    REQUIRE(lim != nullptr);
    // THE SAME OBJECT, by address: the fake the provider handed out as the motor
    CHECK(lim == static_cast<IMotorCurrentLimit*>(&w.devices.fakeMotor(11)));
    CHECK(static_cast<hal::IMotor*>(&w.devices.fakeMotor(11)) == m11);
    CHECK(w.devices.currentLimit(11) == lim);  // idempotent
    CHECK(w.devices.motorRequestCount(11) == 1);  // the lookups were never requests
    // the limit IS the motor's: written through the lookup, read on the fake
    CHECK(lim->setCurrentLimit(Current{1.2}));
    CHECK(w.devices.fakeMotor(11).currentLimit().value() == doctest::Approx(1.2));
    // a neighbouring port still has none
    CHECK(w.devices.currentLimit(12) == nullptr);
}

TEST_CASE("MECH-S-A CA12: FakeDeviceProvider::currentLimit -- every port edge answers nullptr without a read past the arrays") {
    FakeWorld w;
    for (const std::uint8_t p : {std::uint8_t{0}, std::uint8_t{22}, std::uint8_t{200}, std::uint8_t{255}}) {
        CAPTURE(static_cast<int>(p));
        CHECK(w.devices.currentLimit(p) == nullptr);
    }
    REQUIRE(w.devices.motor(1, MotorCartridge::Green) != nullptr);
    REQUIRE(w.devices.motor(21, MotorCartridge::Green) != nullptr);
    CHECK(w.devices.currentLimit(1) == &w.devices.fakeMotor(1));
    CHECK(w.devices.currentLimit(21) == &w.devices.fakeMotor(21));
    // a REFUSED port never served a motor: no limit there either
    w.devices.refusePort(5);
    CHECK(w.devices.motor(5, MotorCartridge::Green) == nullptr);
    CHECK(w.devices.currentLimit(5) == nullptr);
}

TEST_CASE("MECH-S-A CA12: a BOUND FakeMotor is its own limit; a bare IMotor has none; a rebind replaces it") {
    FakeWorld w;
    FakeMotor mine;
    BareMotor bare;
    w.devices.bindMotor(3, mine);   // the FakeMotor& overload
    w.devices.bindMotor(4, bare);   // the IMotor& overload
    CHECK(w.devices.currentLimit(3) == nullptr);  // bound, but not yet SERVED: still nothing
    REQUIRE(w.devices.motor(3, MotorCartridge::Red) == &mine);
    REQUIRE(w.devices.motor(4, MotorCartridge::Red) == &bare);
    CHECK(w.devices.currentLimit(3) == &mine);
    CHECK(w.devices.currentLimit(4) == nullptr);  // BUG CAUGHT: the provider's own unused fake answered
    // rebinding port 3 to a bare motor drops the old limit; rebinding port 4 to a FakeMotor gains one
    w.devices.bindMotor(3, bare);
    CHECK(w.devices.currentLimit(3) == nullptr);
    FakeMotor other;
    w.devices.bindMotor(4, other);
    CHECK(w.devices.currentLimit(4) == &other);
    // a FakeMotor bound through an IMotor& has no limit: the overload is what binds it, never a cast
    FakeMotor viaBase;
    w.devices.bindMotor(5, static_cast<hal::IMotor&>(viaBase));
    REQUIRE(w.devices.motor(5, MotorCartridge::Red) == &viaBase);
    CHECK(w.devices.currentLimit(5) == nullptr);
}

TEST_CASE("MECH-S-A CA12: ProsDeviceProvider::currentLimit -- the ProsMotor it made, writing the DEVICE in mA; never makes") {
    pros::shim::resetAll();
    FakeDeviceRegistry reg;
    reg.setKind(6, DeviceKind::Motor);
    reg.setKind(7, DeviceKind::Motor);
    reg.setKind(9, DeviceKind::Distance);
    ProsDeviceProvider prov{reg};
    CHECK(prov.currentLimit(6) == nullptr);  // nothing made yet
    CHECK(pros::shim::motorState(6).setCurrentLimitCalls == 0);
    CHECK(pros::shim::motorState(6).setGearingCalls == 0);  // BUG CAUGHT: a lookup that constructed a ProsMotor
    hal::IMotor* m6 = prov.motor(-6, MotorCartridge::Red);
    REQUIRE(m6 != nullptr);
    IMotorCurrentLimit* lim = prov.currentLimit(6);
    REQUIRE(lim != nullptr);
    CHECK(prov.currentLimit(6) == lim);
    // the SAME ProsMotor: its limit write lands on port 6's device, in mA (HA-219)
    CHECK(lim->setCurrentLimit(Current{1.2}));
    CHECK(pros::shim::motorState(6).currentLimitMa == 1200);
    CHECK(lim->currentLimit().value() == doctest::Approx(1.2));
    // nothing written to port 7 by any of it, and port 7 has no limit until its motor is made
    CHECK(prov.currentLimit(7) == nullptr);
    CHECK(pros::shim::motorState(7).setCurrentLimitCalls == 0);
    // a port carrying something else, an empty port, and the edges: nullptr
    REQUIRE(prov.distance(9) != nullptr);
    CHECK(prov.currentLimit(9) == nullptr);
    CHECK(prov.currentLimit(12) == nullptr);
    for (const std::uint8_t p : {std::uint8_t{0}, std::uint8_t{22}, std::uint8_t{255}}) {
        CAPTURE(static_cast<int>(p));
        CHECK(prov.currentLimit(p) == nullptr);
    }
    // a motor the registry REFUSED (an empty port) has none
    CHECK(prov.motor(12, MotorCartridge::Red) == nullptr);
    CHECK(prov.currentLimit(12) == nullptr);
}

TEST_CASE("MECH-S-A CA12: the calibrate rig's CountingProvider forwards currentLimit -- and counts no request for it") {
    FakeWorld w;
    cal_prog_rig::CountingProvider counting{w.devices};
    CHECK(counting.currentLimit(8) == nullptr);
    CHECK(counting.requests == 0);
    REQUIRE(counting.motor(8, MotorCartridge::Green) != nullptr);
    CHECK(counting.currentLimit(8) == &w.devices.fakeMotor(8));
    CHECK(counting.requests == 1);  // the motor() only
}
