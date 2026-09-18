// config::RobotConfig — the data a robot is described by, and the rules that say when that
// data is lying (chunk G1 Part 1, brief §4 test 1).
//
// The property under test is ONE sentence: every contradiction a typo, a mis-wire or a
// mis-generated file can produce is refused, and the refusal NAMES the thing. That matters more
// here than in most places, because this type is the front door for a team that cannot read C++:
// the only thing they will ever see when they get it wrong is the sentence valid() writes.
//
// Each case below is a SOLE-CAUSE case: it starts from a config that is valid, changes exactly
// one thing, and requires the refusal to name that thing. A test that built a config with three
// mistakes and checked that it was refused would pass with two of the three checks deleted.

#include "doctest.h"

#include <cstring>
#include <limits>
#include <string>

#include "shulib/config/robot_config.hpp"

namespace cfgns = shulib::config;
using shulib::hal::DescriptionSource;
using shulib::hal::DrivetrainDescription;
using shulib::hal::MotorCartridge;
using shulib::units::Length;

namespace {

/// A valid two-wheel-per-side tank: four drive motors, an IMU, drive-encoder odometry, no GPS,
/// no mechanisms. Every case below starts from this and breaks exactly one thing.
cfgns::RobotConfig baseTank() {
    return cfgns::RobotConfig{
        .identity = {.name = "Robot Two", .team = "1234A", .season = "Push Back"},
        .drivetrain = {.kind = cfgns::DrivetrainKind::Tank,
                       .tank = {.description =
                                    DrivetrainDescription{
                                        .cartridge = MotorCartridge::Blue,
                                        .imuPort = 10,
                                        .left = {.signedPorts = {1, -2},
                                                 .count = 2,
                                                 .inchesPerRadian = Length{1.5}},
                                        .right = {.signedPorts = {-3, 4},
                                                  .count = 2,
                                                  .inchesPerRadian = Length{1.5}},
                                        .trackWidth = Length{12.0},
                                        .source = DescriptionSource::TypedTable}}},
        .odometry = {.kind = cfgns::OdometryKind::DriveEncoders}};
}

/// A valid X-drive with tracking wheels and a GPS — the second shape, and the one that exercises
/// every field a tank does not have.
cfgns::RobotConfig baseXDrive() {
    return cfgns::RobotConfig{
        .identity = {.name = "Robot One", .team = "1234B", .season = "Push Back"},
        .drivetrain = {.kind = cfgns::DrivetrainKind::XDrive,
                       .xdrive = {.ports = {1, 2, 3, 4},
                                  .cartridge = MotorCartridge::Green,
                                  .imuPort = 10,
                                  .driveRadius = Length{7.0}}},
        .odometry = {.kind = cfgns::OdometryKind::TrackingWheels,
                     .wheels = {.forward = {.port = 5,
                                            .wheelDiameter = Length{2.0},
                                            .offset = Length{-3.0}},
                                .lateral = {.port = 6,
                                            .wheelDiameter = Length{2.0},
                                            .offset = Length{-4.5}}}},
        .sensors = {.gps = {.present = true,
                            .port = 9,
                            .leverArmForward = Length{0.0},
                            .leverArmLeft = Length{0.0}}}};
}

/// valid() with a message buffer, returned as a std::string so the checks can say `contains`.
std::string whyNot(const cfgns::RobotConfig& c) {
    char why[192] = "";
    const bool ok = c.valid(why, sizeof why);
    return ok ? std::string{} : std::string{why};
}

}  // namespace

// ═══ 1a. The two base configs are valid, and valid() is honest about saying nothing ═══
TEST_CASE("G1 config 1a: the two reference configs are valid and write no reason") {
    char why[192] = "sentinel";
    CHECK(baseTank().valid(why, sizeof why));
    CHECK(std::strlen(why) == 0);  // a passing check must not leave a stale sentence behind
    CHECK(baseXDrive().valid(why, sizeof why));
    CHECK(std::strlen(why) == 0);
    // The no-buffer form is the one a boot path uses when it only wants the bool.
    CHECK(baseTank().valid());
    CHECK(baseXDrive().valid());
}

// ═══ 1b. A port shared between two owners, named with BOTH owners ════════════════════
// Bug caught: a port-collision check that exists but says only "duplicate port 5" — which tells a
// person a number and not what to go and look at. Each case here changes ONE port.
TEST_CASE("G1 config 1b: a smart port claimed twice is refused, naming the port and both owners") {
    SUBCASE("a drive side and a tracking wheel") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.wheels.forward.port = 3;  // already an X-drive wheel
        const std::string why = whyNot(c);
        CHECK(why.find("port 3") != std::string::npos);
        CHECK(why.find("drive wheel") != std::string::npos);
        CHECK(why.find("forward tracking wheel") != std::string::npos);
    }
    SUBCASE("a mechanism motor and the IMU") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "intake", .motorPorts = {-10}, .motorCount = 1};
        c.mechanismCount = 1;
        const std::string why = whyNot(c);
        CHECK(why.find("port 10") != std::string::npos);
        CHECK(why.find("the IMU") != std::string::npos);
        CHECK(why.find("mechanism motor") != std::string::npos);
    }
    SUBCASE("a mechanism motor and a drive side — the SIGN does not hide it") {
        cfgns::RobotConfig c = baseTank();
        // The drive stores port 2 as `-2`; the mechanism states it as `+2`. A check that
        // compared the stored bytes rather than the magnitudes would miss this entirely.
        c.mechanisms[0] = {.name = "lift", .motorPorts = {2}, .motorCount = 1};
        c.mechanismCount = 1;
        const std::string why = whyNot(c);
        CHECK(why.find("port 2") != std::string::npos);
        CHECK(why.find("LEFT drive") != std::string::npos);
    }
    SUBCASE("the GPS and a tracking wheel") {
        cfgns::RobotConfig c = baseXDrive();
        c.sensors.gps.port = 6;  // already the lateral pod
        const std::string why = whyNot(c);
        CHECK(why.find("port 6") != std::string::npos);
        CHECK(why.find("GPS") != std::string::npos);
        CHECK(why.find("lateral tracking wheel") != std::string::npos);
    }
    SUBCASE("two tracking wheels on one port") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.wheels.lateral.port = c.odometry.wheels.forward.port;
        const std::string why = whyNot(c);
        CHECK(why.find("port 5") != std::string::npos);
        CHECK(why.find("forward tracking wheel") != std::string::npos);
        CHECK(why.find("lateral tracking wheel") != std::string::npos);
    }
}

// ═══ 1c. An X-drive with a repeated wheel port ═══════════════════════════════════════
// Bug caught: two kinematic wheels pointed at one motor — which drives, and drives WRONG, in a
// way no reading can detect (the two wheels' commands would fight inside one adapter).
TEST_CASE("G1 config 1c: an X-drive that lists the same wheel port twice is refused") {
    cfgns::RobotConfig c = baseXDrive();
    // Wheel 3 stated as `-2` while wheel 1 is `2`: the same physical motor, opposite signs. A
    // check that compared the stored BYTES rather than the magnitudes would let this through.
    c.drivetrain.xdrive.ports[3] = -2;
    const std::string why = whyNot(c);
    CHECK(why.find("port 2") != std::string::npos);
    CHECK(why.find("drive wheel") != std::string::npos);
}

// ═══ 1d. A tank's description refuses, and ITS message comes through VERBATIM ════════
// Bug caught: the config re-wording (or worse, re-deriving) the drivetrain's own rules. Ruling
// B1 is "one representation"; a second message for the same fault is the first sign of a second
// representation. The assertion is a string EQUALITY against what the description itself says.
TEST_CASE("G1 config 1d: a tank whose DrivetrainDescription is invalid surfaces its own message, "
          "unchanged") {
    struct Break {
        const char* what;
        void (*apply)(DrivetrainDescription&);
    };
    const Break breaks[] = {
        {"no left side", [](DrivetrainDescription& d) { d.left.count = 0; }},
        {"unset cartridge",
         [](DrivetrainDescription& d) { d.cartridge = static_cast<MotorCartridge>(0); }},
        {"unset IMU port", [](DrivetrainDescription& d) { d.imuPort = 0; }},
        {"a port on both sides", [](DrivetrainDescription& d) { d.right.signedPorts[0] = 1; }},
        {"unset track width", [](DrivetrainDescription& d) { d.trackWidth = Length{0.0}; }},
        {"unset inches per radian",
         [](DrivetrainDescription& d) { d.left.inchesPerRadian = Length{0.0}; }},
        {"the IMU doubling as a drive port",
         [](DrivetrainDescription& d) { d.left.signedPorts[0] = 10; }},
    };
    for (const Break& b : breaks) {
        CAPTURE(b.what);
        cfgns::RobotConfig c = baseTank();
        b.apply(c.drivetrain.tank.description);

        char fromDescription[192] = "";
        REQUIRE_FALSE(c.drivetrain.tank.description.valid(fromDescription,
                                                          sizeof fromDescription));
        char fromConfig[192] = "";
        REQUIRE_FALSE(c.valid(fromConfig, sizeof fromConfig));
        CHECK(std::string{fromConfig} == std::string{fromDescription});
    }
}

// ═══ 1e. The GPS's own rules ═════════════════════════════════════════════════════════
// Bug caught: a non-finite lever arm reaching the adapter, where it is subtracted from every fix
// forever — the fix stays "valid" and the position becomes NaN, which the gate then rejects, so
// the robot looks like it is off-strip rather than mis-configured.
TEST_CASE("G1 config 1e: a GPS declared present must have a port and a finite lever arm") {
    SUBCASE("a non-finite forward lever arm") {
        cfgns::RobotConfig c = baseXDrive();
        c.sensors.gps.leverArmForward = Length{std::numeric_limits<double>::quiet_NaN()};
        const std::string why = whyNot(c);
        CHECK(why.find("lever arm") != std::string::npos);
    }
    SUBCASE("a non-finite left lever arm") {
        cfgns::RobotConfig c = baseXDrive();
        c.sensors.gps.leverArmLeft = Length{std::numeric_limits<double>::infinity()};
        CHECK(whyNot(c).find("lever arm") != std::string::npos);
    }
    SUBCASE("present with an unset port") {
        cfgns::RobotConfig c = baseXDrive();
        c.sensors.gps.port = 0;
        const std::string why = whyNot(c);
        CHECK(why.find("GPS port 0") != std::string::npos);
    }
    SUBCASE("ABSENT — nothing else about it is read, garbage and all") {
        cfgns::RobotConfig c = baseXDrive();
        c.sensors.gps.present = false;
        c.sensors.gps.port = 99;  // nonsense …
        c.sensors.gps.leverArmForward = Length{std::numeric_limits<double>::quiet_NaN()};
        CHECK(c.valid());  // … and irrelevant: the robot has no GPS
    }
}

// ═══ 1f. Odometry: the one drivetrain/estimator cross-check ══════════════════════════
// Bug caught: DriveEncoderOdometry paired with a drivetrain that has no two sides. It would
// construct (the builder would have to invent two IMotors to give it) and then integrate
// nonsense. Refusing the PAIRING in the data is cheaper than a precondition three objects deep.
TEST_CASE("G1 config 1f: DriveEncoders is refused on a drivetrain with no two sides") {
    SUBCASE("X-drive") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.kind = cfgns::OdometryKind::DriveEncoders;
        const std::string why = whyNot(c);
        CHECK(why.find("DriveEncoders") != std::string::npos);
        CHECK(why.find("x-drive") != std::string::npos);
        CHECK(why.find("TrackingWheels") != std::string::npos);
    }
    SUBCASE("H-drive") {
        cfgns::RobotConfig c = baseTank();
        c.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
        c.drivetrain.hdrive = {.ports = {1, 2, 3},
                               .cartridge = MotorCartridge::Green,
                               .imuPort = 10,
                               .trackWidth = Length{11.0},
                               .strafeWheelOffset = Length{-4.0}};
        CHECK(whyNot(c).find("DriveEncoders") != std::string::npos);
    }
    SUBCASE("a tank with pods DECLARED but drive encoders CHOSEN is legal, and the pods are "
            "not even read") {
        // The brief's ruling: an over-complete config is not a contradiction. The pod fields are
        // not validated and — the part that actually matters — not port-claimed, so a pod
        // "declared" on a port the drive already uses cannot refuse a config that never builds it.
        cfgns::RobotConfig c = baseTank();
        c.odometry.wheels.forward = {.port = 1, .wheelDiameter = Length{-5.0}};
        c.odometry.wheels.lateral = {.port = 1, .wheelDiameter = Length{0.0}};
        CHECK(c.valid());
    }
}

// ═══ 1g. Tracking wheels' own rules ══════════════════════════════════════════════════
// Bug caught: a zero diameter reaching TrackingWheel, whose precondition would throw at
// construction from inside the builder — a loud failure, but one that names a library class
// instead of the field the person typed.
TEST_CASE("G1 config 1g: a tracking wheel needs a port, a positive diameter and a finite offset") {
    SUBCASE("unset port") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.wheels.forward.port = 0;
        CHECK(whyNot(c).find("forward tracking wheel port 0") != std::string::npos);
    }
    SUBCASE("zero diameter") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.wheels.lateral.wheelDiameter = Length{0.0};
        const std::string why = whyNot(c);
        CHECK(why.find("lateral tracking wheel diameter") != std::string::npos);
    }
    SUBCASE("non-finite offset") {
        cfgns::RobotConfig c = baseXDrive();
        c.odometry.wheels.forward.offset = Length{std::numeric_limits<double>::quiet_NaN()};
        CHECK(whyNot(c).find("forward tracking wheel offset") != std::string::npos);
    }
}

// ═══ 1h. The holonomic drivetrains' own rules ════════════════════════════════════════
TEST_CASE("G1 config 1h: a holonomic drivetrain's cartridge, IMU port, geometry and port range") {
    SUBCASE("unset cartridge") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.xdrive.cartridge = static_cast<MotorCartridge>(0);
        CHECK(whyNot(c).find("cartridge byte 0") != std::string::npos);
    }
    SUBCASE("IMU port out of range") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.xdrive.imuPort = 22;
        CHECK(whyNot(c).find("IMU port 22") != std::string::npos);
    }
    SUBCASE("a wheel port that is not a port at all") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.xdrive.ports[2] = 100;
        const std::string why = whyNot(c);
        CHECK(why.find("x-drive wheel 2") != std::string::npos);
    }
    SUBCASE("a zero drive radius") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.xdrive.driveRadius = Length{0.0};
        CHECK(whyNot(c).find("driveRadius") != std::string::npos);
    }
    SUBCASE("an H-drive with a zero track width") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
        c.drivetrain.hdrive = {.ports = {1, 2, 3},
                               .cartridge = MotorCartridge::Green,
                               .imuPort = 10,
                               .trackWidth = Length{0.0},
                               .strafeWheelOffset = Length{-4.0}};
        CHECK(whyNot(c).find("h-drive trackWidth") != std::string::npos);
    }
    SUBCASE("an H-drive with a non-finite strafe offset") {
        cfgns::RobotConfig c = baseXDrive();
        c.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
        c.drivetrain.hdrive = {.ports = {1, 2, 3},
                               .cartridge = MotorCartridge::Green,
                               .imuPort = 10,
                               .trackWidth = Length{11.0},
                               .strafeWheelOffset =
                                   Length{std::numeric_limits<double>::quiet_NaN()}};
        CHECK(whyNot(c).find("strafeWheelOffset") != std::string::npos);
    }
    SUBCASE("a kind byte nobody minted") {
        cfgns::RobotConfig c = baseTank();
        c.drivetrain.kind = static_cast<cfgns::DrivetrainKind>(7);
        CHECK(whyNot(c).find("drivetrain kind byte 7") != std::string::npos);
    }
    SUBCASE("an odometry kind byte nobody minted") {
        cfgns::RobotConfig c = baseTank();
        c.odometry.kind = static_cast<cfgns::OdometryKind>(9);
        CHECK(whyNot(c).find("odometry kind byte 9") != std::string::npos);
    }
}

// ═══ 1i. Mechanisms — types only, but their ports are real ═══════════════════════════
// Bug caught: mechanism ports left out of the collision check because nothing builds them yet.
// They are the ports most likely to collide (they are typed last and by a different person), and
// the collision is invisible until two adapters fight over one motor on the field.
TEST_CASE("G1 config 1i: a mechanism's shape and its ports are checked even though Part 1 builds "
          "none") {
    SUBCASE("more mechanisms than capacity") {
        cfgns::RobotConfig c = baseTank();
        c.mechanismCount = 9;
        CHECK(whyNot(c).find("mechanismCount 9") != std::string::npos);
    }
    SUBCASE("an empty name") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.motorPorts = {11}, .motorCount = 1};
        c.mechanismCount = 1;
        CHECK(whyNot(c).find("mechanism 0 has an empty name") != std::string::npos);
    }
    SUBCASE("too many motors") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "intake", .motorPorts = {11, 12, 13, 14}, .motorCount = 5};
        c.mechanismCount = 1;
        CHECK(whyNot(c).find("claims 5 motors") != std::string::npos);
    }
    SUBCASE("a motor port out of range") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "intake", .motorPorts = {99}, .motorCount = 1};
        c.mechanismCount = 1;
        CHECK(whyNot(c).find("motor entry 99") != std::string::npos);
    }
    SUBCASE("a pneumatic line outside 1..8") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "clamp", .pneumaticPorts = {9}, .pneumaticCount = 1};
        c.mechanismCount = 1;
        CHECK(whyNot(c).find("pneumatic entry 9") != std::string::npos);
    }
    SUBCASE("two mechanisms on one port") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "intake", .motorPorts = {11}, .motorCount = 1};
        c.mechanisms[1] = {.name = "lift", .motorPorts = {-11}, .motorCount = 1};
        c.mechanismCount = 2;
        const std::string why = whyNot(c);
        CHECK(why.find("port 11") != std::string::npos);
        CHECK(why.find("mechanism motor") != std::string::npos);
    }
    SUBCASE("a legal mechanism set does not disturb anything") {
        cfgns::RobotConfig c = baseTank();
        c.mechanisms[0] = {.name = "intake",
                           .motorPorts = {11, -12},
                           .motorCount = 2,
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1};
        c.mechanisms[1] = {.name = "lift", .motorPorts = {13}, .motorCount = 1};
        c.mechanismCount = 2;
        CHECK(c.valid());
        // Entries BEYOND the count are ignored entirely, garbage and all.
        c.mechanisms[7] = {.name = "", .motorPorts = {1, 1, 1, 1}, .motorCount = 9};
        CHECK(c.valid());
    }
}

// ═══ 1j. The derived counts, and what they mean for the F5 cross-check ═══════════════
// Bug caught: a wheel-count derivation that reads the wrong array, which would make
// robot::validate()'s cross-check pass on a drivetrain missing a motor.
TEST_CASE("G1 config 1j: wheelCount() and driveMotorCount() are what the F5 cross-check compares") {
    CHECK(baseTank().drivetrain.wheelCount() == 2);
    CHECK(baseTank().drivetrain.driveMotorCount() == 2);
    CHECK(baseXDrive().drivetrain.wheelCount() == 4);
    CHECK(baseXDrive().drivetrain.driveMotorCount() == 4);

    // A holonomic wheel with NO port is representable — and valid() lets it through on purpose,
    // because the refusal it deserves belongs to the one owner of the wheel-count rule
    // (robot::validate(); see RobotConfig::valid()'s documentation). That is what makes the
    // cross-check reachable instead of being dead code that can never fire.
    cfgns::RobotConfig c = baseXDrive();
    c.drivetrain.xdrive.ports[2] = 0;
    CHECK(c.valid());
    CHECK(c.drivetrain.wheelCount() == 4);
    CHECK(c.drivetrain.driveMotorCount() == 3);

    cfgns::RobotConfig h = baseXDrive();
    h.drivetrain.kind = cfgns::DrivetrainKind::HDrive;
    h.drivetrain.hdrive = {.ports = {1, 2, 0},
                           .cartridge = MotorCartridge::Green,
                           .imuPort = 10,
                           .trackWidth = Length{11.0},
                           .strafeWheelOffset = Length{-4.0}};
    CHECK(h.drivetrain.wheelCount() == 3);
    CHECK(h.drivetrain.driveMotorCount() == 2);
}

// ═══ 1k. describe(), and the buffer discipline every char* API here owes ═════════════
TEST_CASE("G1 config 1k: describe() names the robot, the drivetrain, the source and the GPS") {
    char buf[200] = "";
    baseTank().describe(buf, sizeof buf);
    const std::string tank{buf};
    CHECK(tank.find("Robot Two") != std::string::npos);
    CHECK(tank.find("1234A") != std::string::npos);
    CHECK(tank.find("tank") != std::string::npos);
    CHECK(tank.find("TYPED") != std::string::npos);
    CHECK(tank.find("drive-encoders") != std::string::npos);
    CHECK(tank.find("GPS ABSENT") != std::string::npos);

    baseXDrive().describe(buf, sizeof buf);
    const std::string x{buf};
    CHECK(x.find("x-drive") != std::string::npos);
    CHECK(x.find("tracking-wheels") != std::string::npos);
    CHECK(x.find("GPS present") != std::string::npos);

    // A MEASURED description says so, because "the robot measured this" and "a person typed this
    // off a tape" are not the same claim.
    cfgns::RobotConfig m = baseTank();
    m.drivetrain.tank.description.source = DescriptionSource::RobotMeasured;
    m.describe(buf, sizeof buf);
    CHECK(std::string{buf}.find("MEASURED") != std::string::npos);

    // Truncation, not overflow, and always a NUL — the same contract every char* API in this
    // tree carries. The canary past the end must survive.
    char small[24];
    std::memset(small, '\xAB', sizeof small);
    baseTank().describe(small, 16);
    CHECK(std::strlen(small) < 16);
    CHECK(small[16] == '\xAB');
    // cap == 0 and a null buffer are both no-ops rather than crashes.
    baseTank().describe(nullptr, 0);
    baseTank().describe(small, 0);
    CHECK(small[0] != '\0');  // untouched by the cap-0 call
}

// ═══ 1l. The seam ════════════════════════════════════════════════════════════════════
TEST_CASE("G1 config 1l: StaticRobotConfig holds its config BY VALUE") {
    cfgns::RobotConfig source = baseTank();
    const cfgns::StaticRobotConfig held{source};
    // Changing the original must not change what the source hands out — the whole point of the
    // copy, and the bug it prevents is a boot sequence that decodes into a local and returns a
    // reference to it.
    source.identity.name[0] = 'X';
    const cfgns::IRobotConfig& seam = held;
    CHECK(std::string{seam.config().identity.name} == "Robot Two");
    CHECK(seam.config().valid());
}
