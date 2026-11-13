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
#include <filesystem>
#include <fstream>
#include <iterator>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "pros/shim_control.hpp"
#include "source_pins.hpp"

#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/log_copy_sink.hpp"
#include "shulib/diag/run_summary.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/hal/pros/device_provider.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using source_pins::bodyOf;
using source_pins::readFile;
using source_pins::squash;
using source_pins::stripComments;
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

// ═══ 1n-1q. The optional log copy on the SD card (T4 Part 3; decisions N5 (a), A (a), B (a)) ══════
namespace {

namespace fs = std::filesystem;

/// A fresh scratch "card" (with the trailing '/' the adapter demands), the ProsBlockSink tests'
/// pattern: the provider's copy runs the SAME join+open+write path it runs against /usd/.
std::string copyRoot() {
    const fs::path dir = fs::temp_directory_path() / "shulib_t4p3_logcopy";
    fs::create_directories(dir);
    return dir.string() + "/";
}

std::string readText(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    REQUIRE_MESSAGE(in.good(), "cannot open ", p.string());
    return std::string((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
}

/// What the provider's sink is asked to carry in 1n-1p: two lines, ten per-tick records, a
/// summary, one more line — in that order.
void sayThings(shulib::hal::ITelemetrySink& sink) {
    sink.log(shulib::hal::LogLevel::Info, "DRV", "booted: shulib DRIVE");
    sink.log(shulib::hal::LogLevel::Warn, "TEL", "WARNING: port 18 (LEFT) went ABSENT at runtime");
    for (int k = 0; k < 10; ++k) {
        shulib::diag::DebugRecord r;
        r.t = shulib::units::Time{0.01 * k};
        sink.emit(r);
    }
    shulib::diag::RunSummary s{};
    s.motionsStarted = 1;
    s.motionsSettled = 1;
    sink.summarize(s);
    sink.log(shulib::hal::LogLevel::Error, "TEL", "REFUSED: more than 2 motors dead in total");
}

}  // namespace

TEST_CASE("ProsDeviceProvider 1n: given a bare file name, the provider copies every LOG line and the "
          "summary to the card, byte for byte what serial prints for them, and NO per-tick record; "
          "telemetry() is the copy sink over the serial TermSink") {
    // BUG CAUGHT (the brief's 4' test; decision A (a)): the copy taken at the character level — every
    // record line on the card, a block write and a flush every 10 ms; the copy file never opened, or
    // opened somewhere else; telemetry() still the bare serial sink, so the copy constructor's
    // argument did nothing; a copy whose lines differ from serial's (another clock, another format).
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = copyRoot();
    const fs::path file = fs::path(root) / "drive_log.txt";
    {
        std::ofstream old(file, std::ios::binary);
        old << "a previous boot's log\n";  // Truncate: one file per boot
    }
    pros::shim::advanceUs(3'250'000);
    ProsDeviceProvider devices{"drive_log.txt", shulib::hal::pros::ProsFileMode::Truncate,
                               root.c_str()};
    REQUIRE(devices.serialCopyOpen());
    CHECK(std::string{devices.serialCopyPath()} == file.string());
    auto* copy = dynamic_cast<shulib::diag::LogCopySink*>(&devices.telemetry());
    REQUIRE(copy != nullptr);
    CHECK(dynamic_cast<shulib::diag::TermSink*>(&copy->primary()) != nullptr);  // serial's
    CHECK(dynamic_cast<shulib::diag::TermSink*>(&copy->copy()) != nullptr);     // the card's
    CHECK(&copy->primary() != &copy->copy());
    CHECK(devices.telemetry().wantsRecord());  // records are still built — for serial
    sayThings(devices.telemetry());
    // The reference: what a TermSink over the provider's own clock prints for the LOG channel and
    // the summary alone — the serial line for each of them, with no record line.
    shulib::hal::fake::FakeCharSink expected;
    shulib::diag::TermSink reference{devices.clock(), expected};
    reference.log(shulib::hal::LogLevel::Info, "DRV", "booted: shulib DRIVE");
    reference.log(shulib::hal::LogLevel::Warn, "TEL",
                  "WARNING: port 18 (LEFT) went ABSENT at runtime");
    shulib::diag::RunSummary s{};
    s.motionsStarted = 1;
    s.motionsSettled = 1;
    reference.summarize(s);
    reference.log(shulib::hal::LogLevel::Error, "TEL", "REFUSED: more than 2 motors dead in total");
    const std::string onCard = readText(file);
    CHECK(onCard == expected.text());
    CHECK(onCard.find("a previous boot") == std::string::npos);
    CHECK(onCard.find("[t=   3.25] [WARN][TEL] WARNING: port 18") != std::string::npos);
    CHECK(onCard.find("[LOC]") == std::string::npos);  // no record line (an idle tick is [LOC])
    CHECK(devices.serialCopyDroppedLines() == 0U);
}

TEST_CASE("ProsDeviceProvider 1o: NO CARD — the copy refuses every line and COUNTS it, nothing is "
          "thrown, and serial keeps every line; with no file name there is no copy at all") {
    // BUG CAUGHT: a missing card stopping a robot (a throw at boot or mid-run), or dropping lines
    // without a count (the T5 rule: a drop is said, never silent); a provider whose default form
    // changed (the no-copy provider must stay today's, byte for byte).
    pros::shim::resetAll();  // the shim's default: no card (HA-122)
    ProsDeviceProvider devices{"drive_log.txt"};
    CHECK_FALSE(devices.serialCopyOpen());
    CHECK(std::string{devices.serialCopyPath()} == "/usd/drive_log.txt");
    CHECK_NOTHROW(sayThings(devices.telemetry()));
    // Three log lines and the summary's block, every one refused by the card and counted; the ten
    // records never reached the copy, so they are not among them.
    shulib::hal::fake::FakeCharSink summaryLines;
    shulib::hal::fake::FakeClock clock;
    shulib::diag::TermSink countSummary{clock, summaryLines};
    countSummary.summarize(shulib::diag::RunSummary{});
    std::uint32_t summaryWrites = 0;
    for (const char c : summaryLines.text()) {
        if (c == '\n') ++summaryWrites;
    }
    REQUIRE(summaryWrites >= 6U);
    CHECK(devices.serialCopyDroppedLines() == 3U + summaryWrites);
    auto* copy = dynamic_cast<shulib::diag::LogCopySink*>(&devices.telemetry());
    REQUIRE(copy != nullptr);
    CHECK(dynamic_cast<shulib::diag::TermSink*>(&copy->primary()) != nullptr);  // serial, unchanged
    // A null name is the default provider: no copy, telemetry() the serial TermSink (1j).
    ProsDeviceProvider plain{nullptr};
    CHECK(dynamic_cast<shulib::diag::TermSink*>(&plain.telemetry()) != nullptr);
    CHECK_FALSE(plain.serialCopyOpen());
    CHECK(std::string{plain.serialCopyPath()}.empty());
    CHECK(plain.serialCopyDroppedLines() == 0U);
    CHECK(plain.serialCopyAppendOpen() == shulib::hal::pros::AppendOpen::Refused);
}

TEST_CASE("ProsDeviceProvider 1p: APPEND (decision B (a), the competition program's) — an existing "
          "log keeps every byte and the new lines follow it; a missing one is created") {
    // BUG CAUGHT: the mode argument ignored (a reboot mid-match erasing that match's log — the case
    // the log exists for), or Append on a missing file refusing instead of creating it.
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = copyRoot();
    const fs::path file = fs::path(root) / "comp_log.txt";
    fs::remove(file);
    {
        ProsDeviceProvider first{"comp_log.txt", shulib::hal::pros::ProsFileMode::Append,
                                 root.c_str()};
        REQUIRE(first.serialCopyOpen());
        CHECK(first.serialCopyAppendOpen() == shulib::hal::pros::AppendOpen::Created);
        first.telemetry().log(shulib::hal::LogLevel::Info, "CMP", "boot one");
    }
    {
        ProsDeviceProvider second{"comp_log.txt", shulib::hal::pros::ProsFileMode::Append,
                                  root.c_str()};
        REQUIRE(second.serialCopyOpen());
        CHECK(second.serialCopyAppendOpen() == shulib::hal::pros::AppendOpen::Appended);
        second.telemetry().log(shulib::hal::LogLevel::Info, "CMP", "boot two");
    }
    const std::string onCard = readText(file);
    const std::size_t one = onCard.find("boot one");
    const std::size_t two = onCard.find("boot two");
    CHECK(one != std::string::npos);
    CHECK(two != std::string::npos);
    CHECK(one < two);
}

TEST_CASE("ProsDeviceProvider 1q: the copy's file name must be BARE — a leading '/' or an empty name "
          "is a precondition (ProsBlockSink's own rule, through the provider)") {
    // BUG CAUGHT: the double-prefix mistake ("/usd//usd/drive_log.txt") compiled out of sight.
    pros::shim::resetAll();
    CHECK_THROWS_AS(ProsDeviceProvider{"/usd/drive_log.txt"}, PreconditionError);
    CHECK_THROWS_AS(ProsDeviceProvider{""}, PreconditionError);
}

// ═══ 1r-1s. The copy never carries a line per tick, nor waits on another task (the T4-3 review) ═══
namespace {

/// A pacer whose every tick lasts `us` on the shim's clock — 20 ms is past LoopMonitor's 15 ms
/// budget, so every tick of a motion paced by it overruns.
struct ShimSlowPacer final : shulib::motion::ITickPacer {
    explicit ShimSlowPacer(std::uint64_t tickUs) noexcept : us{tickUs} {}
    void pace() override {
        ++paces;
        pros::shim::advanceUs(us);
    }
    std::uint64_t us;
    int paces = 0;
};

/// What one turnTo over the REAL provider (with the competition program's copy, Append) and the
/// REAL Robot left behind, at a tick of `tickUs`.
struct OverrunRun {
    int paces = 0;
    int overrunTicks = 0;        ///< the scheduler's LoopMonitor
    int overrunRaises = 0;       ///< the Robot's fault latch
    int overrunLinesOnCard = 0;  ///< `fault=LOOP_OVERRUN` lines in the copy
};

int countOf(const std::string& text, const std::string& what) {
    int n = 0;
    for (std::size_t p = text.find(what); p != std::string::npos; p = text.find(what, p + 1)) {
        ++n;
    }
    return n;
}

OverrunRun turnWithTicksOf(std::uint64_t tickUs) {
    pros::shim::resetAll();
    pros::shim::usdState().installed = true;
    const std::string root = copyRoot();
    const fs::path file = fs::path(root) / "comp_log.txt";
    fs::remove(file);
    for (const int p : {11, 12, 13, 14, 15, 20, 19, 18, 17, 16}) {
        pros::shim::setPluggedAtPort(p, pros::c::E_DEVICE_MOTOR);
    }
    pros::shim::setPluggedAtPort(2, pros::c::E_DEVICE_IMU);
    OverrunRun out;
    ShimSlowPacer pacer{tickUs};
    {
        ProsDeviceProvider devices{"comp_log.txt", shulib::hal::pros::ProsFileMode::Append,
                                   root.c_str()};
        REQUIRE(devices.serialCopyOpen());
        const shulib::config::RobotConfig cfg = shimTankConfig();
        const shulib::robot::BuildOptions options{.pacer = &pacer};
        const shulib::robot::BuildVerdict v = shulib::robot::validate(cfg, devices, options);
        CAPTURE(v.why);
        REQUIRE(v.ok);
        shulib::robot::Robot robot{cfg, devices, options};
        (void)robot.chassis().turnTo(shulib::math::Angle::degrees(90.0),
                                     {.timeout = shulib::units::Time{1.0}});
        out.overrunTicks = robot.chassis().scheduler().loopMonitor().overrunCount();
        out.overrunRaises = robot.faults().raiseCount(shulib::diag::FaultCode::LoopOverrun);
    }
    out.paces = pacer.paces;
    out.overrunLinesOnCard = countOf(readText(file), "fault=LOOP_OVERRUN");
    return out;
}

}  // namespace

TEST_CASE("ProsDeviceProvider 1r: an auton motion whose EVERY tick overruns puts ONE LOOP_OVERRUN "
          "line on the card copy, while every slow tick is still counted") {
    // BUG CAUGHT (the T4-3 review's major finding, its probe's shape): a slow loop logging a fault
    // line on every tick, which the copy turned into one card write and one flush inside every
    // slow tick (50 in a 1 s turn at 20 ms) — decision A's "never a line per tick" broken, and the
    // card's cost added to the very ticks that were already late. Counting must not change: the
    // monitor and the latch still see every slow tick.
    const OverrunRun slow = turnWithTicksOf(20'000);
    CAPTURE(slow.paces);
    REQUIRE(slow.paces >= 40);
    CHECK(slow.overrunTicks >= slow.paces - 2);  // every tick but the baselines
    CHECK(slow.overrunRaises == slow.overrunTicks);
    CHECK(slow.overrunLinesOnCard == 1);
    // The control: at 10 ms nothing overruns, and nothing is said.
    const OverrunRun healthy = turnWithTicksOf(10'000);
    CHECK(healthy.paces >= 40);
    CHECK(healthy.overrunTicks == 0);
    CHECK(healthy.overrunLinesOnCard == 0);
}

TEST_CASE("ProsDeviceProvider 1s: the lines the copy LOST are the card's refusals PLUS the lines "
          "another task's copy write kept off (diag::LogCopySink's guard)") {
    // BUG CAUGHT: a line left off the card because another task was mid-copy, never counted — the
    // boot line's "N line(s) lost" would under-say what the card is missing. Contention needs a
    // second task, which the host cannot schedule into the provider's own chain, so the sum is
    // pinned in the source (log_copy_sink_test.cpp drives the guard itself, re-entered).
    const std::string text = readFile("include/shulib/hal/pros/device_provider.hpp");
    CHECK(squash(bodyOf(stripComments(text),
                        "std::uint32_t serialCopyDroppedLines() const noexcept"))
          == "return copyChars_.has_value() && logCopy_.has_value() ? copyChars_->droppedWrites() "
             "+ logCopy_->contendedLines() : 0U;");
}
