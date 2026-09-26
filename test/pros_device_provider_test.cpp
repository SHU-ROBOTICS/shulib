// ProsDeviceProvider THROUGH THE HOST SHIM (chunk G1 Part 2a, brief §4 test 1).
//
// ═══ WHAT ONLY THIS FILE CAN PROVE ══════════════════════════════════════════════════
// That the on-robot provider refuses a port by asking the REGISTRY first. The audit that revised
// the G1 Part 2 brief measured the trap this file exists to keep shut: ProsImu's and ProsRotation's
// constructors cannot throw, ProsGps defers on an unreadable device, and in this shim an empty
// MOTOR port constructs cleanly too (the shim's motor device answers its read-back whether or not
// the registry lists it). So every "empty port" case below would come back as a LIVE adapter if
// the registry check were skipped — which is exactly the mutation the brief names, and why each
// case also asserts that the shim device was never touched (no gearing set, no reset() sent).
//
// And that everything around the check holds: one port, one object (the shim counts the
// configuration calls a second construction would repeat); one port, one configuration (a
// different sign, cartridge, lever arm or IMU/GPS port is a precondition); IMU calibration started
// exactly once; the motor read-back failure refused with the adapter's own words, in ASCII; and —
// the end-to-end half — a robot::Robot built over this provider, through the shim, with the sign
// of every drive port arriving at the device as stored and an ABSENT port never opened.
//
// NO HARDWARE CLAIM: the shim tests this class against our BELIEF about PROS (the registry's
// zero-indexing, HA-138; its KIND answers, HA-141); it cannot test the belief.

#include "doctest.h"

#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "pros/shim_control.hpp"

#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::DeviceKind;
using shulib::hal::MotorCartridge;
using shulib::hal::pros::ProsDeviceProvider;
using shulib::units::Length;
using shulib::units::Voltage;

namespace {

/// Every byte of `s` is 7-bit ASCII — what a brain screen's font can draw.
bool isAscii(const char* s) {
    for (std::size_t i = 0; s[i] != '\0'; ++i) {
        if (static_cast<unsigned char>(s[i]) >= 0x80U) {
            return false;
        }
    }
    return true;
}

bool contains(const char* haystack, const char* needle) {
    return std::string{haystack}.find(needle) != std::string::npos;
}

/// All thirteen transcribed v5_device_e_t values (the shim's apix.h), so a sweep can put EVERY
/// kind the brain can report on a port and require that only the right one is served.
constexpr pros::c::v5_device_e_t kEveryRawKind[] = {
    pros::c::E_DEVICE_NONE,     pros::c::E_DEVICE_MOTOR,  pros::c::E_DEVICE_ROTATION,
    pros::c::E_DEVICE_IMU,      pros::c::E_DEVICE_DISTANCE, pros::c::E_DEVICE_RADIO,
    pros::c::E_DEVICE_VISION,   pros::c::E_DEVICE_ADI,    pros::c::E_DEVICE_OPTICAL,
    pros::c::E_DEVICE_GPS,      pros::c::E_DEVICE_AIVISION, pros::c::E_DEVICE_SERIAL,
    pros::c::E_DEVICE_UNDEFINED};

}  // namespace

// ═══ 1a. One port, ONE object ═══════════════════════════════════════════════════════
TEST_CASE("ProsDeviceProvider 1a: the same port returns the SAME object, constructed ONCE") {
    // BUG CAUGHT (the brief's mutation "return a fresh device per request"): a provider that
    // made a second adapter on the second call. The graph holds these pointers for the life of
    // the robot, so half of it would read an object nobody commands. Asserted twice over: the
    // addresses match, AND the shim's configuration counters — which every ProsMotor
    // construction bumps — did not move on the repeat request.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(11, pros::c::E_DEVICE_MOTOR);
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    pros::shim::setPluggedAtPort(9, pros::c::E_DEVICE_GPS);
    pros::shim::setPluggedAtPort(6, pros::c::E_DEVICE_ROTATION);
    ProsDeviceProvider devices;

    shulib::hal::IMotor* m1 = devices.motor(-11, MotorCartridge::Blue);
    REQUIRE(m1 != nullptr);
    const int gearingCalls = pros::shim::motorState(11).setGearingCalls;
    const int unitsCalls = pros::shim::motorState(11).setEncoderUnitsCalls;
    CHECK(gearingCalls >= 1);
    shulib::hal::IMotor* m2 = devices.motor(-11, MotorCartridge::Blue);
    CHECK(m2 == m1);
    CHECK(pros::shim::motorState(11).setGearingCalls == gearingCalls);
    CHECK(pros::shim::motorState(11).setEncoderUnitsCalls == unitsCalls);
    // The adapter really is the one on port -11, configured as stated: blue, degrees, and the
    // SIGN applied by PROS (the shim negates a reversed port's command at the device).
    CHECK(pros::shim::motorState(11).gearing == pros::v5::MotorGears::blue);
    CHECK(pros::shim::motorState(11).units == pros::v5::MotorUnits::degrees);
    m1->setVoltage(Voltage{6.0});
    CHECK(pros::shim::motorState(11).lastVoltageMv == -6000);

    shulib::hal::IImu* i1 = devices.imu(2);
    REQUIRE(i1 != nullptr);
    CHECK(devices.imu(2) == i1);
    shulib::hal::IGps* g1 = devices.gps(9, Length{1.5}, Length{-2.0});
    REQUIRE(g1 != nullptr);
    CHECK(devices.gps(9, Length{1.5}, Length{-2.0}) == g1);
    shulib::hal::IRotation* r1 = devices.rotation(6);
    REQUIRE(r1 != nullptr);
    CHECK(devices.rotation(6) == r1);
    // Different ports are different objects (a provider that returned slot 0 for everything
    // would pass every identity check above).
    pros::shim::setPluggedAtPort(12, pros::c::E_DEVICE_MOTOR);
    shulib::hal::IMotor* other = devices.motor(12, MotorCartridge::Blue);
    REQUIRE(other != nullptr);
    CHECK(other != m1);
    // Nothing was refused, so there is nothing to explain.
    CHECK(std::strlen(devices.lastRefusal()) == 0);
}

// ═══ 1b. EMPTY ports are refused by the REGISTRY, before any adapter exists ════════
TEST_CASE("ProsDeviceProvider 1b: an EMPTY IMU, rotation, GPS or motor port is refused, naming "
          "the port — and no adapter is constructed") {
    // BUG CAUGHT (the brief's mutation "skip the registry check"): the audit's defect 2. Every
    // one of these four would come back LIVE without the check — ProsImu/ProsRotation cannot
    // throw, ProsGps reads a (0,0) offset from an unconfigured shim device and passes its own
    // boot check, and the shim's motor answers its read-back. The registry's EMPTY answer is the
    // only thing that can refuse them.
    pros::shim::resetAll();  // ADVERSARIAL DEFAULT: every registry index answers E_DEVICE_NONE
    ProsDeviceProvider devices;

    SUBCASE("IMU") {
        CHECK(devices.imu(2) == nullptr);
        CHECK(std::string{devices.lastRefusal()} == "port 2: nothing plugged in (expected an IMU)");
        CHECK(pros::shim::imuState(2).resetCalls == 0);  // never constructed, never calibrated
    }
    SUBCASE("rotation sensor") {
        CHECK(devices.rotation(7) == nullptr);
        CHECK(std::string{devices.lastRefusal()}
              == "port 7: nothing plugged in (expected a rotation sensor)");
    }
    SUBCASE("GPS") {
        CHECK(devices.gps(9, Length{0.0}, Length{0.0}) == nullptr);
        CHECK(std::string{devices.lastRefusal()} == "port 9: nothing plugged in (expected a GPS)");
        CHECK(pros::shim::gpsState(9).forbiddenConfigCalls == 0);
    }
    SUBCASE("motor") {
        CHECK(devices.motor(-13, MotorCartridge::Blue) == nullptr);
        CHECK(std::string{devices.lastRefusal()}
              == "port 13: nothing plugged in (expected a motor)");
        // The adapter's constructor never ran: it would have set the gearing on the device.
        CHECK(pros::shim::motorState(13).setGearingCalls == 0);
        CHECK(pros::shim::motorState(13).setEncoderUnitsCalls == 0);
    }
    // The physical port reached the registry as index port - 1 (HA-138, owned by the registry
    // adapter this provider asks — not re-derived here).
    CHECK(pros::shim::registryState().queries == 1);
}

// ═══ 1c. WRONG-KIND ports: only the expected kind is ever served ═══════════════════
TEST_CASE("ProsDeviceProvider 1c: a wrong-kind port is refused naming what was FOUND and what was "
          "EXPECTED — swept over every kind the brain can report") {
    // BUG CAUGHT: a check that only refused EMPTY ports (a rotation sensor plugged where the IMU
    // belongs would be opened as an IMU and read garbage-that-moves), and a check that compared
    // the wrong way round. Every raw kind is put on the port in turn and each of the four
    // requests must be served for exactly its own kind.
    struct Request {
        std::string name;
        pros::c::v5_device_e_t served;
    };
    const Request requests[] = {{"motor", pros::c::E_DEVICE_MOTOR},
                                {"imu", pros::c::E_DEVICE_IMU},
                                {"gps", pros::c::E_DEVICE_GPS},
                                {"rotation", pros::c::E_DEVICE_ROTATION}};
    for (const Request& req : requests) {
        for (const pros::c::v5_device_e_t raw : kEveryRawKind) {
            CAPTURE(req.name);
            CAPTURE(static_cast<int>(raw));
            pros::shim::resetAll();
            pros::shim::setPluggedAtPort(5, raw);
            ProsDeviceProvider devices;
            bool got = false;
            if (req.name == "motor") {
                got = devices.motor(5, MotorCartridge::Green) != nullptr;
            } else if (req.name == "imu") {
                got = devices.imu(5) != nullptr;
            } else if (req.name == "gps") {
                got = devices.gps(5, Length{0.0}, Length{0.0}) != nullptr;
            } else {
                got = devices.rotation(5) != nullptr;
            }
            CHECK(got == (raw == req.served));
            if (!got) {
                CHECK(contains(devices.lastRefusal(), "port 5: "));
                CHECK(contains(devices.lastRefusal(), "(expected "));
                CHECK(isAscii(devices.lastRefusal()));
            }
        }
    }

    // The brief's own example sentence, and three more, verbatim.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_ROTATION);
    pros::shim::setPluggedAtPort(4, pros::c::E_DEVICE_MOTOR);
    pros::shim::setPluggedAtPort(8, pros::c::E_DEVICE_IMU);
    pros::shim::setPluggedAtPort(1, pros::c::E_DEVICE_RADIO);
    ProsDeviceProvider devices;
    CHECK(devices.imu(2) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 2: a rotation sensor (expected an IMU)");
    CHECK(devices.rotation(4) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 4: a motor (expected a rotation sensor)");
    CHECK(devices.gps(8, Length{0.0}, Length{0.0}) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 8: an IMU (expected a GPS)");
    CHECK(devices.motor(-1, MotorCartridge::Blue) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 1: the radio (expected a motor)");
    CHECK(pros::shim::imuState(2).resetCalls == 0);
}

// ═══ 1d. A present motor that REJECTS its configuration ═════════════════════════════
TEST_CASE("ProsDeviceProvider 1d: a motor whose read-back fails is refused with the ADAPTER's "
          "message, naming the port, in plain ASCII") {
    // BUG CAUGHT: (a) an adapter throw escaping the provider — one bad motor would unwind the
    // whole boot instead of becoming a sentence; (b) the refusal losing the adapter's reason (the
    // registry said MOTOR, so "nothing plugged in" would be a lie); (c) the em dash in
    // ProsMotor's message reaching a screen whose fonts are ASCII-only.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(13, pros::c::E_DEVICE_MOTOR);
    pros::shim::motorState(13).disconnected = true;  // units read back `invalid`: HA-98 fires
    ProsDeviceProvider devices;

    shulib::hal::IMotor* m = nullptr;
    CHECK_NOTHROW(m = devices.motor(-13, MotorCartridge::Blue));
    CHECK(m == nullptr);
    const std::string why{devices.lastRefusal()};
    CAPTURE(why);
    CHECK(why.rfind("port 13: the motor rejected its configuration: ", 0) == 0);
    CHECK(why.find("ProsMotor: encoder units read-back disagrees") != std::string::npos);
    CHECK(why.find("did not accept degrees - wrong port") != std::string::npos);  // the dash
    CHECK(why.find("not a motor?)") != std::string::npos);  // the WHOLE reason fits
    CHECK(isAscii(devices.lastRefusal()));

    // Nothing was kept: when the port answers again, the next request constructs afresh and the
    // object works — a refusal is not cached as a permanent verdict.
    pros::shim::motorState(13).disconnected = false;
    m = devices.motor(-13, MotorCartridge::Blue);
    REQUIRE(m != nullptr);
    m->setVoltage(Voltage{3.0});
    CHECK(pros::shim::motorState(13).lastVoltageMv == -3000);
}

// ═══ 1e. One port, ONE configuration ════════════════════════════════════════════════
TEST_CASE("ProsDeviceProvider 1e: a second request with a different sign, cartridge, lever arm or "
          "IMU/GPS port is a PRECONDITION violation") {
    // BUG CAUGHT (the brief's mutation "drop the conflict check"): quietly returning the first
    // configuration. A drive side asked for once as -11 and once as +11 would be driven with
    // whichever sign came first, and nothing would say so — the single most dangerous number in
    // a drivetrain description, applied wrong without a word.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(11, pros::c::E_DEVICE_MOTOR);
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    pros::shim::setPluggedAtPort(3, pros::c::E_DEVICE_IMU);
    pros::shim::setPluggedAtPort(9, pros::c::E_DEVICE_GPS);
    pros::shim::setPluggedAtPort(10, pros::c::E_DEVICE_GPS);
    ProsDeviceProvider devices;

    shulib::hal::IMotor* m = devices.motor(-11, MotorCartridge::Blue);
    REQUIRE(m != nullptr);
    CHECK_THROWS_AS((void)devices.motor(11, MotorCartridge::Blue), PreconditionError);
    CHECK_THROWS_AS((void)devices.motor(-11, MotorCartridge::Green), PreconditionError);
    try {
        (void)devices.motor(11, MotorCartridge::Blue);
        FAIL("a sign conflict was accepted");
    } catch (const PreconditionError& e) {
        CHECK(contains(e.what(), "port 11"));
        CHECK(contains(e.what(), "one port, one configuration"));
    }
    // The original configuration is untouched and still served.
    CHECK(devices.motor(-11, MotorCartridge::Blue) == m);
    CHECK(pros::shim::motorState(11).gearing == pros::v5::MotorGears::blue);

    REQUIRE(devices.imu(2) != nullptr);
    CHECK_THROWS_AS((void)devices.imu(3), PreconditionError);
    REQUIRE(devices.gps(9, Length{1.0}, Length{2.0}) != nullptr);
    CHECK_THROWS_AS((void)devices.gps(9, Length{1.0}, Length{2.5}), PreconditionError);
    CHECK_THROWS_AS((void)devices.gps(9, Length{-1.0}, Length{2.0}), PreconditionError);
    CHECK_THROWS_AS((void)devices.gps(10, Length{1.0}, Length{2.0}), PreconditionError);
    CHECK(devices.gps(9, Length{1.0}, Length{2.0}) != nullptr);
}

// ═══ 1f. IMU calibration: started by the provider, exactly ONCE ══════════════════════
TEST_CASE("ProsDeviceProvider 1f: imu() starts calibration on construction and never again") {
    // BUG CAUGHT: calibration started on every request (a second reset() re-zeros the sensor
    // under a live boot heading — HA-05; ProsImu would throw on the second call, so a provider
    // that called calibrate() per request would turn the builder's second pass into a crash),
    // or never started (an uncalibrated IMU emits garbage-that-moves, HA-23, and isReady()
    // would stay false forever).
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    ProsDeviceProvider devices;
    shulib::hal::IImu* imu = devices.imu(2);
    REQUIRE(imu != nullptr);
    CHECK(pros::shim::imuState(2).resetCalls == 1);
    for (int i = 0; i < 5; ++i) {
        CHECK(devices.imu(2) == imu);
    }
    CHECK(pros::shim::imuState(2).resetCalls == 1);
    CHECK(pros::shim::imuState(2).tareFamilyCalls == 0);  // and nothing ever tares (HA-05)
    CHECK_FALSE(imu->isReady());                          // calibrating…
    pros::shim::imuState(2).calibrating = false;          // …the sensor finishes
    CHECK(imu->isReady());
}

// ═══ 1g. Not a port, not a gearset — refused without asking the platform ═══════════
TEST_CASE("ProsDeviceProvider 1g: a port outside 1..21 or a cartridge byte that is not a gearset is "
          "refused by number, and the registry is never asked") {
    pros::shim::resetAll();
    for (int p = 1; p <= 21; ++p) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    ProsDeviceProvider devices;
    CHECK(devices.motor(0, MotorCartridge::Blue) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port 0 is not a smart port (ports are 1..21)");
    CHECK(devices.motor(22, MotorCartridge::Blue) == nullptr);
    CHECK(devices.motor(-22, MotorCartridge::Blue) == nullptr);
    CHECK(std::string{devices.lastRefusal()} == "port -22 is not a smart port (ports are 1..21)");
    CHECK(devices.motor(-128, MotorCartridge::Blue) == nullptr);
    CHECK(devices.imu(0) == nullptr);
    CHECK(devices.imu(22) == nullptr);
    CHECK(devices.gps(0, Length{0.0}, Length{0.0}) == nullptr);
    CHECK(devices.rotation(255) == nullptr);
    CHECK(contains(devices.lastRefusal(), "port 255 is not a smart port"));
    CHECK(devices.motor(4, static_cast<MotorCartridge>(0)) == nullptr);
    CHECK(std::string{devices.lastRefusal()}
          == "port 4: cartridge byte 0 is not a V5 gearset (red, green or blue)");
    CHECK(pros::shim::motorState(4).setGearingCalls == 0);
    CHECK(pros::shim::registryState().queries == 0);  // none of these reached the platform
}

// ═══ 1h. The registry is asked when a device is MADE, not after ═════════════════════
TEST_CASE("ProsDeviceProvider 1h: a made device stays the answer if its port empties; a refused "
          "port is re-asked on the next request") {
    // BUG CAUGHT: re-checking the registry on every request — a cable jiggled between the
    // builder's two passes would make the SECOND pass (wireMotors) hit its "the provider refused
    // a motor validate() had obtained" precondition, i.e. a boot crash out of a flicker. And the
    // opposite: caching a refusal, so a port plugged in after the first look is never served.
    pros::shim::resetAll();
    pros::shim::setPluggedAtPort(12, pros::c::E_DEVICE_MOTOR);
    ProsDeviceProvider devices;
    shulib::hal::IMotor* m = devices.motor(12, MotorCartridge::Green);
    REQUIRE(m != nullptr);
    const int queries = pros::shim::registryState().queries;
    pros::shim::setPluggedAtPort(12, pros::c::E_DEVICE_NONE);  // the cable comes out
    CHECK(devices.motor(12, MotorCartridge::Green) == m);
    CHECK(pros::shim::registryState().queries == queries);  // not asked again

    CHECK(devices.rotation(7) == nullptr);                        // empty now…
    pros::shim::setPluggedAtPort(7, pros::c::E_DEVICE_ROTATION);  // …plugged in…
    CHECK(devices.rotation(7) != nullptr);                        // …served on the next ask
}

// ═══ 1i. The injected registry is the one asked ═════════════════════════════════════
TEST_CASE("ProsDeviceProvider 1i: a caller's registry replaces the brain's — and is the only one "
          "asked") {
    pros::shim::resetAll();  // the brain's registry says: nothing anywhere
    shulib::hal::fake::FakeDeviceRegistry census;
    census.setKind(4, DeviceKind::Motor);
    census.setKind(2, DeviceKind::Imu);
    ProsDeviceProvider devices{census};
    CHECK(devices.motor(4, MotorCartridge::Red) != nullptr);
    CHECK(devices.imu(2) != nullptr);
    CHECK(devices.rotation(6) == nullptr);
    CHECK(census.queries() == 3);
    CHECK(pros::shim::registryState().queries == 0);  // the owned one was never consulted
    CHECK(pros::shim::motorState(4).gearing == pros::v5::MotorGears::red);
}

// ═══ 1j. The port-less handles ═════════════════════════════════════════════════════
TEST_CASE("ProsDeviceProvider 1j: the clock, battery, sink and pacer are the V5 adapters, stable, "
          "never refused") {
    pros::shim::resetAll();
    ProsDeviceProvider devices;
    CHECK(&devices.clock() == &devices.clock());
    CHECK(&devices.battery() == &devices.battery());
    CHECK(&devices.telemetry() == &devices.telemetry());
    CHECK(&devices.pacer() == &devices.pacer());
    // The clock is the V5's (shim) time, in seconds.
    pros::shim::advanceUs(2'500'000);
    CHECK(devices.clock().now().value() == doctest::Approx(2.5));
    // The battery reads the brain's pack (the shim's 12600 mV).
    CHECK(devices.battery().voltage().value() == doctest::Approx(12.6));
    // The sink is the terminal formatter src/main.cpp's hand-wired graphs built.
    CHECK(dynamic_cast<shulib::diag::TermSink*>(&devices.telemetry()) != nullptr);
    // The pacer is the delay_until one, 10 ms.
    devices.pacer().pace();
    devices.pacer().pace();
    CHECK(pros::shim::timeState().delayUntilCalls == 2);
    CHECK(pros::shim::timeState().lastDelayUntilDelta == 10U);
    CHECK(std::strlen(devices.lastRefusal()) == 0);
}

// ═══ 1k. A whole robot, over the real provider, through the shim ════════════════════
namespace {

/// Robot two's shape (the 2026-09-10 measurement, signs included) — ports, signs, IMU 2 — with
/// invented scales: this case is about WIRING, not geometry.
shulib::config::RobotConfig shimTankConfig() {
    shulib::hal::DrivetrainDescription d{.cartridge = MotorCartridge::Blue,
                                         .imuPort = 2,
                                         .trackWidth = Length{12.5},
                                         .source = shulib::hal::DescriptionSource::RobotMeasured,
                                         .provenance = "G1 Part 2a shim fixture: never measured"};
    const std::int8_t left[5] = {-11, 12, -13, 14, -15};
    const std::int8_t right[5] = {20, -19, 18, -17, 16};
    for (std::size_t i = 0; i < 5; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 5;
    d.right.count = 5;
    d.left.inchesPerRadian = Length{1.375};
    d.right.inchesPerRadian = Length{1.375};
    return shulib::config::RobotConfig{
        .identity = {.name = "Shim Two", .team = "SHU"},
        .drivetrain = {.kind = shulib::config::DrivetrainKind::Tank, .tank = {.description = d}},
        .odometry = {.kind = shulib::config::OdometryKind::DriveEncoders}};
}

}  // namespace

TEST_CASE("ProsDeviceProvider 1k: robot::Robot builds over the REAL provider through the shim — "
          "every sign reaches the device as stored, and an absent port is never opened") {
    // The end-to-end half of test 1: the builder and this provider together, the composition
    // Part 2b will run. Landmine 5 (one sign, one place) is asserted at the DEVICE, which the
    // fake provider cannot do: a +6 V group command must reach a port stored NEGATIVE as -6000 mV
    // at the device and a port stored positive as +6000 mV — PROS applies the reversal, the group
    // and the builder never do.
    pros::shim::resetAll();
    const shulib::config::RobotConfig cfg = shimTankConfig();
    const int ports[10] = {11, 12, 13, 14, 15, 20, 19, 18, 17, 16};
    for (const int p : ports) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);

    SUBCASE("every port present") {
        ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(cfg, devices);
        CAPTURE(v.why);
        REQUIRE(v.ok);
        shulib::robot::Robot robot{cfg, devices};
        CHECK(pros::shim::imuState(2).resetCalls == 1);  // twice asked, once calibrated
        robot.context().driveMotors()[0]->setVoltage(Voltage{6.0});  // the LEFT group
        robot.context().driveMotors()[1]->setVoltage(Voltage{6.0});  // the RIGHT group
        const std::int8_t stored[10] = {-11, 12, -13, 14, -15, 20, -19, 18, -17, 16};
        for (const std::int8_t s : stored) {
            CAPTURE(static_cast<int>(s));
            CHECK(pros::shim::motorState(s).lastVoltageMv == (s < 0 ? -6000 : 6000));
            CHECK(pros::shim::motorState(s).gearing == pros::v5::MotorGears::blue);
        }
        char map[shulib::diag::kMaxPortMapBytes] = "";
        robot.portMap(map, sizeof map);
        CHECK(std::string{map}
              == "MEASURED L -11 +12 -13 +14 -15 R +20 -19 +18 -17 +16 IMU2 ipr 1.3750/1.3750 "
                 "TW12.50");
    }
    SUBCASE("ports 16 and 17 EMPTY: refused by name without the list, built with it") {
        pros::shim::setPluggedAtPort(16, pros::c::E_DEVICE_NONE);
        pros::shim::setPluggedAtPort(17, pros::c::E_DEVICE_NONE);
        {
            ProsDeviceProvider devices;
            const shulib::robot::BuildVerdict v = shulib::robot::validate(cfg, devices);
            CHECK_FALSE(v.ok);
            CHECK(std::string{v.why}
                  == "a RIGHT drive motor cannot be made: port 17: nothing plugged in (expected a "
                     "motor)");
        }
        const int absent[2] = {17, 16};
        const shulib::robot::BuildOptions options{.absentDrivePorts = absent};
        ProsDeviceProvider devices;
        const shulib::robot::BuildVerdict v = shulib::robot::validate(cfg, devices, options);
        CAPTURE(v.why);
        REQUIRE(v.ok);
        shulib::robot::Robot robot{cfg, devices, options};
        // The absent ports were never OPENED: their devices never saw a configuration call.
        CHECK(pros::shim::motorState(16).setGearingCalls == 0);
        CHECK(pros::shim::motorState(17).setGearingCalls == 0);
        const auto* right = dynamic_cast<const shulib::hal::MotorGroup*>(
            robot.context().driveMotors()[1]);
        REQUIRE(right != nullptr);
        CHECK(right->memberCount() == 5);
        CHECK(right->presentCount() == 3);
        robot.context().driveMotors()[1]->setVoltage(Voltage{4.0});
        CHECK(pros::shim::motorState(20).lastVoltageMv == 4000);
        CHECK(pros::shim::motorState(19).lastVoltageMv == -4000);
        CHECK(pros::shim::motorState(17).lastVoltageMv == 0);  // a hole is never commanded
        char banner[320] = "";
        robot.describe(banner, sizeof banner);
        CHECK(contains(banner, "DEGRADED: absent 17,16"));
    }
}

// ═══ 1l. The 3-wire lines (T3's addendum B2) ═════════════════════════════════════════
TEST_CASE("ProsDeviceProvider 1l: digitalOut() — the same line is the SAME object, DRIVEN once at "
          "the stated level when made; the registry is never asked (it cannot see a solenoid); a "
          "line outside 1..8 is refused by number and never driven") {
    // BUG CAUGHT: a fresh ProsDigitalOut per request (each construction DRIVES the line — the claw
    // would snap to its boot level every time anything asked); a registry check on a 3-wire port
    // (every line refused: the brain's registry has no kind for a solenoid, and with nothing
    // plugged into any smart port — this shim's default — it answers NONE everywhere); a letter
    // byte translated rather than refused; the stated level dropped for PROS's default LOW.
    pros::shim::resetAll();  // the registry says: nothing plugged in, anywhere
    ProsDeviceProvider devices;
    const auto& writesA = pros::shim::adiLine(0, 1).writes;

    shulib::hal::IDigitalOut* a = devices.digitalOut(1, true);
    REQUIRE(a != nullptr);
    CHECK(writesA == std::vector<std::int32_t>{1});  // made = driven HIGH, once
    CHECK(a->commanded());
    CHECK(devices.digitalOut(1, true) == a);
    CHECK(writesA.size() == 1U);  // not driven again
    a->set(false);
    CHECK(writesA == (std::vector<std::int32_t>{1, 0}));
    CHECK_FALSE(a->commanded());
    CHECK(pros::shim::registryState().queries == 0);  // HA-158: nothing to ask
    CHECK(std::strlen(devices.lastRefusal()) == 0);

    // One line, one configuration.
    try {
        (void)devices.digitalOut(1, false);
        FAIL("a second boot level for port A was accepted");
    } catch (const PreconditionError& e) {
        CHECK(contains(e.what(), "3-wire port A was made driven high; asked again as low"));
        CHECK(contains(e.what(), "one line, one configuration"));
    }
    CHECK(writesA.size() == 2U);

    // Another line, another object, made at ITS stated level — PROS's LOW default never decides.
    shulib::hal::IDigitalOut* h = devices.digitalOut(8, false);
    REQUIRE(h != nullptr);
    CHECK(h != a);
    CHECK(pros::shim::adiLine(0, 8).writes == std::vector<std::int32_t>{0});
    shulib::hal::IDigitalOut* c = devices.digitalOut(3, true);
    REQUIRE(c != nullptr);
    CHECK(pros::shim::adiLine(0, 3).writes == std::vector<std::int32_t>{1});

    // Outside 1..8 — 'A' (65) and 'a' (97) included: one spelling at the seam — refused by
    // number, and no line is driven (the shim would map 'A' onto line 1).
    for (const int bad : {0, 9, 65, 97, 255}) {
        CAPTURE(bad);
        CHECK(devices.digitalOut(static_cast<std::uint8_t>(bad), true) == nullptr);
        CHECK(std::string{devices.lastRefusal()}
              == "3-wire port " + std::to_string(bad)
                     + " is not a brain 3-wire port (1..8 are A..H)");
        CHECK(isAscii(devices.lastRefusal()));
    }
    CHECK(writesA.size() == 2U);
    CHECK(pros::shim::registryState().queries == 0);
}

TEST_CASE("ProsDeviceProvider 1m: robot::Robot builds a pneumatic claw over the REAL provider "
          "through the shim — validate() drives nothing; the line takes the START level once, "
          "when the Robot is built; the mechanism's commands reach the line") {
    // The end-to-end half of the addendum's provider item: the builder and this provider
    // together, as a program built on them will run. What only this case can show is the
    // PHYSICAL ACTION at the device: the one write the builder causes lands on the right line, at
    // the stated level, exactly once, and never during validate(). (Written whatever the brain's
    // state: whether a write made while the brain is DISABLED — a Robot built in initialize() on a
    // field — reaches the valve is A4 register HA-157's second half, beyond any host test.)
    pros::shim::resetAll();
    shulib::config::RobotConfig cfg = shimTankConfig();
    cfg.mechanisms[0] = {.name = "claw",
                         .pneumaticPorts = {1},
                         .pneumaticCount = 1,
                         .pneumaticSafe = shulib::config::PneumaticLevel::Low,
                         .pneumaticStart = shulib::config::PneumaticLevel::High};
    cfg.mechanismCount = 1;
    const int ports[10] = {11, 12, 13, 14, 15, 20, 19, 18, 17, 16};
    for (const int p : ports) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    const auto& writesA = pros::shim::adiLine(0, 1).writes;

    ProsDeviceProvider devices;
    const shulib::robot::BuildVerdict v = shulib::robot::validate(cfg, devices);
    CAPTURE(v.why);
    REQUIRE(v.ok);
    CHECK(writesA.empty());  // validate() never asks for a line: asking would drive it
    shulib::robot::Robot robot{cfg, devices};
    CHECK(writesA == std::vector<std::int32_t>{1});  // the START level, once
    shulib::hal::PneumaticMechanism* claw = robot.pneumaticMechanism("claw");
    REQUIRE(claw != nullptr);
    CHECK(claw->commanded());
    claw->set(false);
    CHECK(writesA == (std::vector<std::int32_t>{1, 0}));
    claw->set(true);
    claw->applySafeState();  // the SAFE level, Low
    CHECK(writesA == (std::vector<std::int32_t>{1, 0, 1, 0}));
    CHECK(robot.mechanisms().size() == 1U);
    // No other 3-wire line on the brain was touched.
    for (std::uint8_t line = 2; line <= 8; ++line) {
        CAPTURE(static_cast<int>(line));
        CHECK(pros::shim::adiLine(0, line).writes.empty());
    }
}
