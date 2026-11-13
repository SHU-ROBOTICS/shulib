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

#include <cstdint>
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

// ═══ 1i. Mechanisms — their shape and their ports (built since T3; see 1m) ════════════
// Bug caught: mechanism ports left out of the collision check because Part 1 built no mechanism.
// They are the ports most likely to collide (they are typed last and by a different person), and
// the collision is invisible until two adapters fight over one motor on the field.
TEST_CASE("G1 config 1i: a mechanism's shape and its ports are checked (Part 1 built none; T3 "
          "builds them)") {
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
        // Since T3 a mechanism with motors must STATE its cartridge and safe brake mode (1m
        // below); this set states them, so it stays the legal reference it always was. Since T3's
        // addendum B2 a mechanism may not have motors AND lines (1n below), so the intake's line
        // that used to ride on it is its own pneumatic mechanism, stating its two levels.
        c.mechanisms[0] = {.name = "intake",
                           .motorPorts = {11, -12},
                           .motorCount = 2,
                           .cartridge = MotorCartridge::Green,
                           .safeMode = cfgns::SafeBrakeMode::Coast};
        c.mechanisms[1] = {.name = "lift",
                           .motorPorts = {13},
                           .motorCount = 1,
                           .cartridge = MotorCartridge::Red,
                           .safeMode = cfgns::SafeBrakeMode::Hold};
        c.mechanisms[2] = {.name = "intake flap",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::Low,
                           .pneumaticStart = cfgns::PneumaticLevel::Low};
        c.mechanismCount = 3;
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

// ═══ 1m. What a BUILT mechanism needs (chunk T3, brief test 14's config half) ═════════
// Since T3, robot::Robot builds a hal::MotorMechanism from every mechanism with motors, asking the
// provider with the MECHANISM's cartridge and handing it the DECLARED safe brake mode. Both are
// therefore required, and "nobody wrote it" must never read as a value — least of all as Coast,
// which is where a value-initialised hal::BrakeMode would land and where a loaded claw drops.
namespace {

/// A valid tank with robot two's two claw pivots: separate single-motor mechanisms on 6 and 7,
/// RED, Hold. Every 1m case starts from this and breaks exactly one thing.
cfgns::RobotConfig tankWithClaws() {
    cfgns::RobotConfig c = baseTank();
    c.mechanisms[0] = {.name = "claw pivot 6",
                       .motorPorts = {6},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Hold};
    c.mechanisms[1] = {.name = "claw pivot 7",
                       .motorPorts = {-7},
                       .motorCount = 1,
                       .cartridge = MotorCartridge::Red,
                       .safeMode = cfgns::SafeBrakeMode::Hold};
    c.mechanismCount = 2;
    return c;
}

}  // namespace

TEST_CASE("T3 config 1m: the claw reference config is valid, and a MechanismConfig nobody filled "
          "in carries UNSET — never Coast") {
    CHECK(whyNot(tankWithClaws()).empty());
    const cfgns::MechanismConfig blank{};
    CHECK(static_cast<int>(static_cast<std::uint8_t>(blank.safeMode)) == 0);
    CHECK(static_cast<int>(static_cast<std::uint8_t>(blank.cartridge)) == 0);
    CHECK_FALSE(cfgns::isSafeBrakeMode(blank.safeMode));
    CHECK(std::string{cfgns::safeBrakeModeWord(blank.safeMode)} == "UNSET");
    // The three real modes convert to exactly the HAL mode of the same name…
    CHECK(cfgns::toBrakeMode(cfgns::SafeBrakeMode::Coast) == shulib::hal::BrakeMode::Coast);
    CHECK(cfgns::toBrakeMode(cfgns::SafeBrakeMode::Brake) == shulib::hal::BrakeMode::Brake);
    CHECK(cfgns::toBrakeMode(cfgns::SafeBrakeMode::Hold) == shulib::hal::BrakeMode::Hold);
    CHECK(std::string{cfgns::safeBrakeModeWord(cfgns::SafeBrakeMode::Hold)} == "hold");
    // …and a byte nobody minted never becomes Coast: the floor is Hold.
    for (const int b : {0, 4, 200}) {
        CAPTURE(b);
        const auto m = static_cast<cfgns::SafeBrakeMode>(b);
        CHECK_FALSE(cfgns::isSafeBrakeMode(m));
        CHECK(cfgns::toBrakeMode(m) == shulib::hal::BrakeMode::Hold);
    }
}

TEST_CASE("T3 config 1m': a mechanism WITH motors must state its cartridge and its safe brake mode "
          "— refused by name; a pneumatic-only one need not") {
    SUBCASE("an UNSET cartridge") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[1].cartridge = static_cast<MotorCartridge>(0);
        const std::string why = whyNot(c);
        CHECK(why.find("'claw pivot 7'") != std::string::npos);
        CHECK(why.find("cartridge byte 0") != std::string::npos);
        CHECK(why.find("UNSET") != std::string::npos);
    }
    SUBCASE("a cartridge byte nobody minted") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[0].cartridge = static_cast<MotorCartridge>(7);
        CHECK(whyNot(c).find("cartridge byte 7") != std::string::npos);
    }
    SUBCASE("an UNSET safe brake mode") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[0].safeMode = static_cast<cfgns::SafeBrakeMode>(0);
        const std::string why = whyNot(c);
        CHECK(why.find("'claw pivot 6'") != std::string::npos);
        CHECK(why.find("safe brake mode byte 0") != std::string::npos);
    }
    SUBCASE("a safe brake mode byte nobody minted") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[1].safeMode = static_cast<cfgns::SafeBrakeMode>(4);
        CHECK(whyNot(c).find("safe brake mode byte 4") != std::string::npos);
    }
    SUBCASE("each of the three real modes and cartridges is accepted") {
        for (const auto mode : {cfgns::SafeBrakeMode::Coast, cfgns::SafeBrakeMode::Brake,
                                cfgns::SafeBrakeMode::Hold}) {
            for (const auto cart :
                 {MotorCartridge::Red, MotorCartridge::Green, MotorCartridge::Blue}) {
                cfgns::RobotConfig c = tankWithClaws();
                c.mechanisms[0].safeMode = mode;
                c.mechanisms[0].cartridge = cart;
                CHECK(c.valid());
            }
        }
    }
    SUBCASE("a pneumatic-only mechanism is not asked for either (it states its LEVELS — 1n)") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[2] = {.name = "clamp",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::High,
                           .pneumaticStart = cfgns::PneumaticLevel::High};
        c.mechanismCount = 3;
        CHECK(whyNot(c).empty());
        CHECK(static_cast<int>(static_cast<std::uint8_t>(c.mechanisms[2].cartridge)) == 0);
    }
}

TEST_CASE("T3 config 1m'': the new refusals run AFTER every port claim — a collision keeps the "
          "message it always had, even when the same config also has an unset cartridge") {
    // BUG CAUGHT: the cartridge / safe-mode check placed inside the first mechanism loop, ahead of
    // the port claims — a mis-wired config would then be reported as "state the cartridge", and a
    // person would go and fix the wrong thing while the port collision stayed.
    SUBCASE("the unset mechanism is the one that collides") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[0].motorPorts[0] = -2;  // the LEFT drive's port 2
        c.mechanisms[0].cartridge = static_cast<MotorCartridge>(0);
        c.mechanisms[0].safeMode = static_cast<cfgns::SafeBrakeMode>(0);
        const std::string why = whyNot(c);
        CHECK(why.find("port 2 is claimed by the LEFT drive and by a mechanism motor")
              != std::string::npos);
    }
    SUBCASE("an EARLIER mechanism is unset and a LATER one collides") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[0].cartridge = static_cast<MotorCartridge>(0);
        c.mechanisms[1].motorPorts[0] = 10;  // the IMU
        const std::string why = whyNot(c);
        CHECK(why.find("port 10") != std::string::npos);
        CHECK(why.find("the IMU") != std::string::npos);
    }
    SUBCASE("a claw on a drive port is refused (the drive-port collision, with a claw's numbers)") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[1].motorPorts[0] = 4;  // RIGHT drive stores +4
        const std::string why = whyNot(c);
        CHECK(why.find("port 4") != std::string::npos);
        CHECK(why.find("RIGHT drive") != std::string::npos);
    }
}

TEST_CASE("T3 config 1m''': a mechanism name must be terminated inside its 24 bytes and unique — "
          "the Robot borrows it as a C string and looks mechanisms up by it") {
    SUBCASE("two mechanisms with one name") {
        cfgns::RobotConfig c = tankWithClaws();
        std::memcpy(c.mechanisms[1].name, "claw pivot 6", sizeof "claw pivot 6");
        const std::string why = whyNot(c);
        CHECK(why.find("mechanisms 0 and 1") != std::string::npos);
        CHECK(why.find("'claw pivot 6'") != std::string::npos);
    }
    SUBCASE("the same name with different garbage after its NUL is still the same name") {
        cfgns::RobotConfig c = tankWithClaws();
        std::memset(c.mechanisms[0].name, 'x', sizeof c.mechanisms[0].name);
        std::memset(c.mechanisms[1].name, 'y', sizeof c.mechanisms[1].name);
        std::memcpy(c.mechanisms[0].name, "claw", sizeof "claw");
        std::memcpy(c.mechanisms[1].name, "claw", sizeof "claw");
        CHECK(whyNot(c).find("both named 'claw'") != std::string::npos);
    }
    SUBCASE("a prefix is a different name") {
        cfgns::RobotConfig c = tankWithClaws();
        std::memcpy(c.mechanisms[1].name, "claw pivot 66", sizeof "claw pivot 66");
        CHECK(whyNot(c).empty());
    }
    SUBCASE("a pneumatic-only mechanism's name is checked too") {
        cfgns::RobotConfig c = tankWithClaws();
        c.mechanisms[2] = {.name = "claw pivot 7",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::Low,
                           .pneumaticStart = cfgns::PneumaticLevel::Low};
        c.mechanismCount = 3;
        CHECK(whyNot(c).find("mechanisms 1 and 2") != std::string::npos);
    }
    SUBCASE("24 characters and no NUL") {
        cfgns::RobotConfig c = tankWithClaws();
        std::memset(c.mechanisms[1].name, 'z', sizeof c.mechanisms[1].name);
        const std::string why = whyNot(c);
        CHECK(why.find("mechanism 1's name has no NUL") != std::string::npos);
        // 23 characters is the longest legal name.
        std::memset(c.mechanisms[1].name, 'z', sizeof c.mechanisms[1].name - 1);
        c.mechanisms[1].name[23] = '\0';
        CHECK(whyNot(c).empty());
    }
}

// ═══ 1n. What a BUILT pneumatic mechanism needs (T3's addendum B2) ════════════════════
// Since B2, robot::Robot builds a hal::PneumaticMechanism from every mechanism with 3-wire lines,
// making each line at its START level (on a brain, making a line DRIVES it) and declaring its SAFE
// level. Both are therefore required, and "nobody wrote it" must never read as a level — least of
// all as `false`, which is where a value-initialised bool would land, and which opens or closes a
// claw depending on plumbing only the team knows.
namespace {

/// Robot two's claw pivots plus its pneumatic claw on 3-wire port A, starting CLOSED — here the
/// line off, safe closed too. Every 1n case starts from this and breaks exactly one thing.
cfgns::RobotConfig tankWithPneumaticClaw() {
    cfgns::RobotConfig c = tankWithClaws();
    c.mechanisms[2] = {.name = "claw",
                       .pneumaticPorts = {1},
                       .pneumaticCount = 1,
                       .pneumaticSafe = cfgns::PneumaticLevel::Low,
                       .pneumaticStart = cfgns::PneumaticLevel::Low};
    c.mechanismCount = 3;
    return c;
}

}  // namespace

TEST_CASE("B2 config 1n: the pneumatic claw config is valid, and a MechanismConfig nobody filled "
          "in carries UNSET levels — never Low") {
    CHECK(whyNot(tankWithPneumaticClaw()).empty());
    const cfgns::MechanismConfig blank{};
    CHECK(static_cast<int>(static_cast<std::uint8_t>(blank.pneumaticSafe)) == 0);
    CHECK(static_cast<int>(static_cast<std::uint8_t>(blank.pneumaticStart)) == 0);
    CHECK_FALSE(cfgns::isPneumaticLevel(blank.pneumaticSafe));
    CHECK_FALSE(cfgns::isPneumaticLevel(blank.pneumaticStart));
    CHECK(std::string{cfgns::pneumaticLevelWord(blank.pneumaticStart)} == "UNSET");
    CHECK(std::string{cfgns::pneumaticLevelWord(cfgns::PneumaticLevel::Low)} == "low");
    CHECK(std::string{cfgns::pneumaticLevelWord(cfgns::PneumaticLevel::High)} == "high");
    CHECK(cfgns::isPneumaticLevel(cfgns::PneumaticLevel::Low));
    CHECK(cfgns::isPneumaticLevel(cfgns::PneumaticLevel::High));
    CHECK_FALSE(cfgns::isPneumaticLevel(static_cast<cfgns::PneumaticLevel>(3)));
    // Exactly High is the line on; Low is the line off.
    CHECK(cfgns::toLineValue(cfgns::PneumaticLevel::High));
    CHECK_FALSE(cfgns::toLineValue(cfgns::PneumaticLevel::Low));
}

TEST_CASE("B2 config 1n': a mechanism WITH 3-wire lines must state its safe and start levels — "
          "UNSET or unknown refused by name; all four pairs accepted") {
    SUBCASE("an UNSET safe level") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticSafe = static_cast<cfgns::PneumaticLevel>(0);
        const std::string why = whyNot(c);
        CHECK(why.find("mechanism 'claw' pneumatic safe level byte 0") != std::string::npos);
        CHECK(why.find("UNSET") != std::string::npos);
    }
    SUBCASE("an UNSET start level") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticStart = static_cast<cfgns::PneumaticLevel>(0);
        const std::string why = whyNot(c);
        CHECK(why.find("mechanism 'claw' pneumatic start level byte 0") != std::string::npos);
        CHECK(why.find("DRIVES the line") != std::string::npos);
    }
    SUBCASE("level bytes nobody minted") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticSafe = static_cast<cfgns::PneumaticLevel>(3);
        CHECK(whyNot(c).find("safe level byte 3") != std::string::npos);
        c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticStart = static_cast<cfgns::PneumaticLevel>(200);
        CHECK(whyNot(c).find("start level byte 200") != std::string::npos);
    }
    SUBCASE("a TWO-line mechanism is asked too") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticPorts[1] = 2;
        c.mechanisms[2].pneumaticCount = 2;
        CHECK(whyNot(c).empty());
        c.mechanisms[2].pneumaticStart = static_cast<cfgns::PneumaticLevel>(0);
        CHECK(whyNot(c).find("start level byte 0") != std::string::npos);
    }
    SUBCASE("each of the four real pairs is accepted") {
        for (const auto safe : {cfgns::PneumaticLevel::Low, cfgns::PneumaticLevel::High}) {
            for (const auto start : {cfgns::PneumaticLevel::Low, cfgns::PneumaticLevel::High}) {
                cfgns::RobotConfig c = tankWithPneumaticClaw();
                c.mechanisms[2].pneumaticSafe = safe;
                c.mechanisms[2].pneumaticStart = start;
                CHECK(c.valid());
            }
        }
    }
    SUBCASE("a mechanism with NO lines is not asked for levels (a motor one never was)") {
        cfgns::RobotConfig c = tankWithClaws();
        CHECK(static_cast<int>(static_cast<std::uint8_t>(c.mechanisms[0].pneumaticSafe)) == 0);
        CHECK(whyNot(c).empty());
    }
}

TEST_CASE("B2 config 1n'': a mechanism with motors AND 3-wire lines is refused by name — make them "
          "two mechanisms") {
    // BUG CAUGHT: a mixed entry accepted, which the Robot could only build as ONE of the two kinds
    // (the other half silently dropped) — a claw pivot whose air never fires, or the reverse.
    cfgns::RobotConfig c = tankWithClaws();
    c.mechanisms[0].pneumaticPorts[0] = 1;
    c.mechanisms[0].pneumaticCount = 1;
    c.mechanisms[0].pneumaticSafe = cfgns::PneumaticLevel::Low;
    c.mechanisms[0].pneumaticStart = cfgns::PneumaticLevel::Low;
    const std::string why = whyNot(c);
    CHECK(why.find("mechanism 'claw pivot 6' has 1 motors AND 1 3-wire lines") != std::string::npos);
    CHECK(why.find("two mechanisms") != std::string::npos);
    // Named for the MIX, not for anything else it lacks: with its levels unset as well, the mix
    // is still what is reported (it is the structural problem, and the first one the pass meets).
    c.mechanisms[0].pneumaticSafe = static_cast<cfgns::PneumaticLevel>(0);
    CHECK(whyNot(c).find("AND 1 3-wire lines") != std::string::npos);
}

TEST_CASE("B2 config 1n''': a 3-wire line listed twice — by two mechanisms or by one — is refused, "
          "naming the port letter; the line claims come before the level checks") {
    // BUG CAUGHT: two mechanisms fanning to ONE solenoid (the provider hands both the same object:
    // the claw opens when the other mechanism is told to), or a line typed twice in one entry.
    SUBCASE("two mechanisms on port A") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[3] = {.name = "flap",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::Low,
                           .pneumaticStart = cfgns::PneumaticLevel::Low};
        c.mechanismCount = 4;
        CHECK(whyNot(c) == "3-wire port A (line 1) is claimed by mechanism 'claw' and by mechanism "
                           "'flap'");
        c.mechanisms[3].pneumaticPorts[0] = 8;  // H: a different line is fine
        CHECK(whyNot(c).empty());
    }
    SUBCASE("one mechanism listing a line twice") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticPorts[1] = 1;
        c.mechanisms[2].pneumaticCount = 2;
        CHECK(whyNot(c) == "mechanism 'claw' lists 3-wire port A (line 1) twice");
    }
    SUBCASE("the collision is reported even when an EARLIER mechanism's levels are unset") {
        // BUG CAUGHT: the level checks placed in the claim pass, ahead of a later collision — a
        // mis-wired config would be reported as "state the level", and the person would go and
        // fix the wrong thing while two mechanisms still shared port A.
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[3] = {.name = "flap", .pneumaticPorts = {2}, .pneumaticCount = 1};  // unset
        c.mechanisms[4] = {.name = "latch",
                           .pneumaticPorts = {1},
                           .pneumaticCount = 1,
                           .pneumaticSafe = cfgns::PneumaticLevel::Low,
                           .pneumaticStart = cfgns::PneumaticLevel::Low};
        c.mechanismCount = 5;
        CHECK(whyNot(c) == "3-wire port A (line 1) is claimed by mechanism 'claw' and by mechanism "
                           "'latch'");
        c.mechanisms[4].pneumaticPorts[0] = 3;  // no collision left: now the unset level is named
        CHECK(whyNot(c).find("mechanism 'flap' pneumatic safe level byte 0") != std::string::npos);
    }
    SUBCASE("a line entry past the count is not read") {
        cfgns::RobotConfig c = tankWithPneumaticClaw();
        c.mechanisms[2].pneumaticPorts[1] = 1;  // garbage beyond pneumaticCount = 1
        CHECK(whyNot(c).empty());
    }
}
