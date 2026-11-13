// config::RobotConfig::valid() — the mechanism SENSOR roles and per-line levels (chunk MECH-A1, the
// MECH brief's §3.1 "valid()" and P4b).
//
// valid() is the RUN-TIME owner of every port collision: it sees the derived config beside the
// drivetrain the boot actually CHOSE — a calibration record's ports included, which no build-time
// check can see. So a record that measured a mechanism SENSOR's port as a drive motor must be refused
// here, naming both owners, before anything is asked for (mutation M5: valid() stops claiming sensor
// ports — the record-collision cases go red).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * Every owner a sensor port can collide with: the LEFT and RIGHT drive, a holonomic wheel, the
//     IMU, both tracking-wheel pods, the GPS, another mechanism's motor, another mechanism's sensor,
//     the mechanism's own other sensor — each refused naming BOTH.
//   * A distance AND an optical piece sensor on one mechanism, or either beside a limit-switch piece
//     sensor (MECH-A1's review); a sensor port outside 1..21.
//   * A limit switch on a line a pneumatic holds (both orders), on the mechanism's own other switch's
//     line, on line 9 — and line 0 meaning "none".
//   * Per-line levels are all-or-nothing: half-stated, a level past the lines, a byte outside the
//     enum, per-line levels on a mechanism with no lines — each refused; the whole set accepted.
//   * P4b's five cases, verbatim, over robot z (robot x + a wrist with an angle sensor + a claw pivot).

#include "doctest.h"

#include <cstdio>
#include <string>

#include "mechanism_fixtures.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace cfgns = shulib::config;
namespace mech = shulib::mechanism;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::MotorCartridge;
using shulib::units::Length;


namespace {

/// P4b's calibration record: a MEASURED tank, the right side fixed, the left side the case's.
DrivetrainDescription record(std::initializer_list<int> leftPorts) {
    DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 21;
    const int right[4] = {11, -12, 13, 15};
    std::size_t i = 0;
    for (const int p : leftPorts) {
        d.left.signedPorts[i++] = static_cast<std::int8_t>(p);
    }
    d.left.count = static_cast<std::uint8_t>(i);
    for (std::size_t k = 0; k < 4; ++k) {
        d.right.signedPorts[k] = static_cast<std::int8_t>(right[k]);
    }
    d.right.count = 4;
    d.left.inchesPerRadian = Length{1.25};
    d.right.inchesPerRadian = Length{1.36};
    d.trackWidth = Length{12.0};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "probe");
    return d;
}

cfgns::RobotConfig withList(const DrivetrainDescription& d, const mech::MechanismTable& t) {
    cfgns::RobotConfig c{};
    std::snprintf(c.identity.name, sizeof c.identity.name, "z");
    c.drivetrain.kind = cfgns::DrivetrainKind::Tank;
    c.drivetrain.tank.description = d;
    c.odometry.kind = cfgns::OdometryKind::DriveEncoders;
    for (std::size_t i = 0; i < t.count; ++i) {
        c.mechanisms[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    c.mechanismCount = t.count;
    return c;
}

std::string whyNot(const cfgns::RobotConfig& c) {
    char why[192] = "";
    const bool ok = c.valid(why, sizeof why);
    return ok ? std::string{"VALID"} : std::string{why};
}

/// The X-drive base of robot_config_test.cpp: wheels 1..4, IMU 10, pods 5 and 6, GPS 9.
cfgns::RobotConfig baseXDrive() {
    return cfgns::RobotConfig{
        .identity = {.name = "Robot One", .team = "1234B", .season = "Push Back"},
        .drivetrain = {.kind = cfgns::DrivetrainKind::XDrive,
                       .xdrive = {.ports = {1, 2, 3, 4},
                                  .cartridge = MotorCartridge::Green,
                                  .imuPort = 10,
                                  .driveRadius = Length{7.0}}},
        .odometry = {.kind = cfgns::OdometryKind::TrackingWheels,
                     .wheels = {.forward = {.port = 5, .wheelDiameter = Length{2.0}, .offset = Length{-3.0}},
                                .lateral = {.port = 6, .wheelDiameter = Length{2.0}, .offset = Length{-4.5}}}},
        .sensors = {.gps = {.present = true, .port = 9, .leverArmForward = Length{0.0}, .leverArmLeft = Length{0.0}}}};
}

/// A one-motor mechanism named `name` on `motorPort`.
cfgns::MechanismConfig motorMech(const char* name, std::int8_t motorPort) {
    cfgns::MechanismConfig m{};
    std::snprintf(m.name, sizeof m.name, "%s", name);
    m.motorPorts[0] = motorPort;
    m.motorCount = 1;
    m.cartridge = MotorCartridge::Red;
    m.safeMode = cfgns::SafeBrakeMode::Hold;
    return m;
}

/// A pneumatic mechanism named `name` on `lineCount` lines from `firstLine`.
cfgns::MechanismConfig pneumaticMech(const char* name, std::uint8_t firstLine, std::uint8_t lineCount) {
    cfgns::MechanismConfig m{};
    std::snprintf(m.name, sizeof m.name, "%s", name);
    for (std::uint8_t l = 0; l < lineCount; ++l) {
        m.pneumaticPorts[l] = static_cast<std::uint8_t>(firstLine + l);
    }
    m.pneumaticCount = lineCount;
    m.pneumaticSafe = cfgns::PneumaticLevel::Low;
    m.pneumaticStart = cfgns::PneumaticLevel::Low;
    return m;
}

cfgns::RobotConfig xdriveWith(std::initializer_list<cfgns::MechanismConfig> ms) {
    cfgns::RobotConfig c = baseXDrive();
    std::size_t i = 0;
    for (const auto& m : ms) {
        c.mechanisms[i++] = m;
    }
    c.mechanismCount = static_cast<std::uint8_t>(i);
    return c;
}

}  // namespace

TEST_CASE("MECH-A1 valid(): P4b's five cases, verbatim -- the runtime owner of every collision sees the RECORD") {
    namespace rz = mech_fixture::robot_z;
    CHECK(whyNot(withList(record({-16, 17, -18, -20}), rz::kRobotZMechanisms)) == "VALID");
    CHECK(whyNot(withList(record({-16, 17, -18, 5}), rz::kRobotZMechanisms))
          == "port 5 is claimed by the LEFT drive and by a mechanism motor");
    CHECK(whyNot(withList(record({-16, 17, -18, 9}), rz::kRobotZMechanisms))
          == "port 9 is claimed by the LEFT drive and by a mechanism sensor");
    CHECK(whyNot(withList(record({-16, 17, -18, 10}), rz::kRobotZMechanisms))
          == "port 10 is claimed by the LEFT drive and by a mechanism sensor");
    mech::MechanismTable handMade = rz::kRobotZMechanisms;
    handMade.wiring[1].homeSwitch = mech::Line::C;  // the hood's line, past the build check
    CHECK(whyNot(withList(record({-16, 17, -18, -20}), handMade))
          == "3-wire port C (line 3) is claimed by mechanism 'lift' and by mechanism 'hood'");
    // Every fixture robot's projection is valid beside a drive it does not touch.
    CHECK(whyNot(withList(record({-16, 17, -18, -20}), mech_fixture::robot_x::kRobotXMechanisms)) == "VALID");
    CHECK(whyNot(withList(record({-16, 17, -18, -20}), mech_fixture::robot_y::kRobotYMechanisms)) == "VALID");
    CHECK(whyNot(withList(record({-16, 17, -18, -20}), mech_fixture::robot_two::kRobotTwoMechanisms)) == "VALID");
}

TEST_CASE("MECH-A1 valid(): a SENSOR port colliding with each other owner is refused, naming both") {
    struct Collide {
        const char* what;
        std::uint8_t port;
        const char* expected;
    };
    for (const Collide& k : {Collide{"a holonomic wheel", 1, "port 1 is claimed by a drive wheel and by a mechanism sensor"},
                             Collide{"the IMU", 10, "port 10 is claimed by the IMU and by a mechanism sensor"},
                             Collide{"the forward pod", 5, "port 5 is claimed by the forward tracking wheel and by a mechanism sensor"},
                             Collide{"the lateral pod", 6, "port 6 is claimed by the lateral tracking wheel and by a mechanism sensor"},
                             Collide{"the GPS", 9, "port 9 is claimed by the GPS and by a mechanism sensor"}}) {
        CAPTURE(k.what);
        cfgns::MechanismConfig m = motorMech("arm", 11);
        m.rotationPort = k.port;
        CHECK(whyNot(xdriveWith({m})) == k.expected);
        m.rotationPort = 0;
        m.distancePort = k.port;
        CHECK(whyNot(xdriveWith({m})) == k.expected);
        m.distancePort = 0;
        m.opticalPort = k.port;
        CHECK(whyNot(xdriveWith({m})) == k.expected);
    }
    SUBCASE("the LEFT and the RIGHT drive of a tank") {
        cfgns::RobotConfig c = withList(record({-16, 17, -18, -20}), mech::mechanisms());
        cfgns::MechanismConfig m = motorMech("arm", 7);
        m.rotationPort = 17;
        c.mechanisms[0] = m;
        c.mechanismCount = 1;
        CHECK(whyNot(c) == "port 17 is claimed by the LEFT drive and by a mechanism sensor");
        c.mechanisms[0].rotationPort = 13;
        CHECK(whyNot(c) == "port 13 is claimed by the RIGHT drive and by a mechanism sensor");
    }
    SUBCASE("another mechanism's motor, either order") {
        cfgns::MechanismConfig a = motorMech("a", 11);
        cfgns::MechanismConfig b = motorMech("b", 12);
        b.distancePort = 11;
        CHECK(whyNot(xdriveWith({a, b})) == "port 11 is claimed by a mechanism motor and by a mechanism sensor");
        cfgns::MechanismConfig first = motorMech("first", 13);
        first.opticalPort = 14;
        cfgns::MechanismConfig second = motorMech("second", 14);
        CHECK(whyNot(xdriveWith({first, second})) == "port 14 is claimed by a mechanism sensor and by a mechanism motor");
    }
    SUBCASE("another mechanism's sensor, and its own other sensor") {
        cfgns::MechanismConfig a = motorMech("a", 11);
        a.rotationPort = 15;
        cfgns::MechanismConfig b = motorMech("b", 12);
        b.distancePort = 15;
        CHECK(whyNot(xdriveWith({a, b})) == "port 15 is claimed by a mechanism sensor and by a mechanism sensor");
        cfgns::MechanismConfig self = motorMech("self", 11);
        self.rotationPort = 16;
        self.distancePort = 16;
        CHECK(whyNot(xdriveWith({self})) == "port 16 is claimed by a mechanism sensor and by a mechanism sensor");
    }
    SUBCASE("its own motor port") {
        cfgns::MechanismConfig m = motorMech("arm", 11);
        m.rotationPort = 11;
        CHECK(whyNot(xdriveWith({m})) == "port 11 is claimed by a mechanism motor and by a mechanism sensor");
    }
}

TEST_CASE("MECH-A1 valid(): one piece sensor per mechanism; sensor ports in 1..21") {
    cfgns::MechanismConfig m = motorMech("intake", 11);
    m.distancePort = 12;
    m.opticalPort = 13;
    CHECK(whyNot(xdriveWith({m})) == "mechanism 'intake' has a distance AND an optical piece sensor -- a piece sensor is one device");
    // A limit-switch piece sensor beside either smart one (MECH-A1's review: accepted, its line
    // claimed, and the switch then never built) — and each alone is fine.
    m.opticalPort = 0;
    m.switchLines[1] = 5;
    CHECK(whyNot(xdriveWith({m})) == "mechanism 'intake' has a distance AND a limit-switch piece sensor -- a piece sensor is one device");
    m.distancePort = 0;
    m.opticalPort = 13;
    CHECK(whyNot(xdriveWith({m})) == "mechanism 'intake' has an optical AND a limit-switch piece sensor -- a piece sensor is one device");
    m.opticalPort = 0;
    CHECK(whyNot(xdriveWith({m})) == "VALID");  // the switch alone
    m.switchLines[1] = 0;
    m.distancePort = 22;
    CHECK(whyNot(xdriveWith({m})) == "mechanism 'intake' sensor port 22 is not a port in 1..21");
    m.distancePort = 0;
    m.rotationPort = 200;
    CHECK(whyNot(xdriveWith({m})) == "mechanism 'intake' sensor port 200 is not a port in 1..21");
    m.rotationPort = 21;
    CHECK(whyNot(xdriveWith({m})) == "VALID");
}

TEST_CASE("MECH-A1 valid(): a limit switch owns its 3-wire line like a pneumatic does") {
    SUBCASE("a pneumatic first, then a switch on its line") {
        cfgns::MechanismConfig hood = pneumaticMech("hood", 3, 2);
        cfgns::MechanismConfig lift = motorMech("lift", 11);
        lift.switchLines[0] = 4;
        CHECK(whyNot(xdriveWith({hood, lift}))
              == "3-wire port D (line 4) is claimed by mechanism 'hood' and by mechanism 'lift' (a limit switch)");
    }
    SUBCASE("a switch first, then a pneumatic on its line (P4b's order)") {
        cfgns::MechanismConfig lift = motorMech("lift", 11);
        lift.switchLines[0] = 3;
        cfgns::MechanismConfig hood = pneumaticMech("hood", 3, 2);
        CHECK(whyNot(xdriveWith({lift, hood})) == "3-wire port C (line 3) is claimed by mechanism 'lift' and by mechanism 'hood'");
    }
    SUBCASE("its home switch and its piece switch on one line") {
        cfgns::MechanismConfig cat = motorMech("catapult", 11);
        cat.switchLines[0] = 5;
        cat.switchLines[1] = 5;
        CHECK(whyNot(xdriveWith({cat})) == "mechanism 'catapult' lists 3-wire port E (line 5) twice (a limit switch)");
    }
    SUBCASE("a pneumatic's piece switch on its own line") {
        cfgns::MechanismConfig grip = pneumaticMech("grip", 1, 1);
        grip.switchLines[1] = 1;
        CHECK(whyNot(xdriveWith({grip})) == "mechanism 'grip' lists 3-wire port A (line 1) twice (a limit switch)");
    }
    SUBCASE("line 9 refused; line 0 is none") {
        cfgns::MechanismConfig m = motorMech("lift", 11);
        m.switchLines[1] = 9;
        CHECK(whyNot(xdriveWith({m})) == "mechanism 'lift' limit switch line 9 is not an ADI line in 1..8");
        m.switchLines[1] = 0;
        m.switchLines[0] = 8;
        CHECK(whyNot(xdriveWith({m})) == "VALID");
    }
}

TEST_CASE("MECH-A1 valid(): per-line levels are ALL OR NOTHING") {
    cfgns::MechanismConfig hood = pneumaticMech("hood", 3, 2);
    SUBCASE("none stated: the one-level form") {
        CHECK(whyNot(xdriveWith({hood})) == "VALID");
    }
    SUBCASE("every line stated") {
        hood.pneumaticSafeByLine[0] = cfgns::PneumaticLevel::High;
        hood.pneumaticSafeByLine[1] = cfgns::PneumaticLevel::Low;
        hood.pneumaticStartByLine[0] = cfgns::PneumaticLevel::Low;
        hood.pneumaticStartByLine[1] = cfgns::PneumaticLevel::Low;
        CHECK(whyNot(xdriveWith({hood})) == "VALID");
        hood.pneumaticStartByLine[1] = static_cast<cfgns::PneumaticLevel>(7);  // a byte outside the enum
        CHECK(whyNot(xdriveWith({hood}))
              == "mechanism 'hood' per-line levels: line 2 is not stated -- state a Low or High safe and start level "
                 "for every one of its 2 lines, or for none");
    }
    SUBCASE("half-stated: one line's safe level only") {
        hood.pneumaticSafeByLine[0] = cfgns::PneumaticLevel::High;
        CHECK(whyNot(xdriveWith({hood}))
              == "mechanism 'hood' per-line levels: line 1 is not stated -- state a Low or High safe and start level "
                 "for every one of its 2 lines, or for none");
    }
    SUBCASE("a level past its lines") {
        cfgns::MechanismConfig claw = pneumaticMech("claw", 1, 1);
        claw.pneumaticSafeByLine[0] = cfgns::PneumaticLevel::Low;
        claw.pneumaticStartByLine[0] = cfgns::PneumaticLevel::Low;
        claw.pneumaticSafeByLine[1] = cfgns::PneumaticLevel::High;
        CHECK(whyNot(xdriveWith({claw}))
              == "mechanism 'claw' per-line levels: line 2 is past its lines -- state a Low or High safe and start "
                 "level for every one of its 1 lines, or for none");
    }
    SUBCASE("per-line levels on a mechanism with no lines") {
        cfgns::MechanismConfig arm = motorMech("arm", 11);
        arm.pneumaticStartByLine[0] = cfgns::PneumaticLevel::Low;
        CHECK(whyNot(xdriveWith({arm}))
              == "mechanism 'arm' per-line levels: line 1 is past its lines -- state a Low or High safe and start "
                 "level for every one of its 0 lines, or for none");
    }
}

TEST_CASE("MECH-A1 valid(): the appended PortUse word, and a pre-MECH config is untouched by the new fields") {
    CHECK(std::string{cfgns::portUseWord(cfgns::PortUse::MechanismSensor)} == "a mechanism sensor");
    CHECK(std::string{cfgns::portUseWord(cfgns::PortUse::Mechanism)} == "a mechanism motor");
    const cfgns::MechanismConfig m{};
    CHECK(m.rotationPort == 0);
    CHECK(m.distancePort == 0);
    CHECK(m.opticalPort == 0);
    CHECK(m.switchLines[0] == 0);
    CHECK(m.switchLines[1] == 0);
    CHECK(m.pneumaticSafeByLine[0] == static_cast<cfgns::PneumaticLevel>(0));
    CHECK(m.pneumaticStartByLine[1] == static_cast<cfgns::PneumaticLevel>(0));
}
