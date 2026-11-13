// The robot file's SIDE DISTANCE SENSORS block, as data: config::WallSensorsConfig, wallMountProblem,
// wallLimitsProblem and RobotConfig::valid()'s wall checks (chunk L6-A2; the L6 brief's §1.4, D5-D7 and
// test T12).
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * ONE OWNER PER PORT: a wall port that is a drive port, the IMU's, a mechanism sensor's or the
//     camera's is refused NAMING BOTH ("port 4 is claimed by the LEFT drive and by a wall sensor") —
//     mutation M25 (the claim removed); the other side's port is refused naming the two sides; a port
//     past 21 is refused naming the side.
//   * An UNMEASURED mount is VALID — the robot boots and every reset refuses MOUNT UNSET (M26: an
//     unmeasured mount refused at boot) — even when its numbers are garbage (they are never read).
//   * Every bound of a MEASURED mount and of the limits, on its edge and just past it, each refused
//     with its own words — and the edge typed the way a robot file types it (`N * kWallDegToRad`)
//     accepted exactly.
//   * THE INVARIANT: every limits block valid() accepts, the reset accepts too — so a valid robot file
//     never meets BAD REQUEST from its own limits (20 000 seeded draws around every edge).
//   * The block is constexpr — robot two's sits in a constexpr table (L6-B) — and its default is "no
//     side declared", so every config written before it stays valid.

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>

#include "shulib/config/robot_config.hpp"
#include "shulib/config/wall_sensor_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/localization/override_walls.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"
#include "shulib/vision/override_elements.hpp"

namespace cfgns = shulib::config;
namespace hal = shulib::hal;
namespace loc = shulib::localization;
using shulib::units::Length;

namespace {

constexpr double kDeg = loc::kWallDegToRad;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

/// A measured mount that passes every check: the left sensor, centred, 10.5 in up.
constexpr loc::SideMount goodMount() {
    loc::SideMount m{};
    m.measured = true;
    m.left = Length{7.5};
    m.height = Length{10.5};
    m.facing = shulib::units::AngleDim{90.0 * kDeg};
    m.provenance = "tape, 2026-11-03";
    return m;
}

/// A two-motor-per-side tank on 11/-12 and -13/14, the IMU on 21; side sensors on 4 (left) and 5
/// (right), measured.
cfgns::RobotConfig tank() {
    cfgns::RobotConfig c{};
    c.drivetrain.kind = cfgns::DrivetrainKind::Tank;
    hal::DrivetrainDescription& d = c.drivetrain.tank.description;
    d.kind = hal::DrivetrainKind::Tank;
    d.cartridge = hal::MotorCartridge::Blue;
    d.imuPort = 21;
    d.left.signedPorts[0] = 11;
    d.left.signedPorts[1] = -12;
    d.left.count = 2;
    d.right.signedPorts[0] = -13;
    d.right.signedPorts[1] = 14;
    d.right.count = 2;
    d.left.inchesPerRadian = Length{1.5};
    d.right.inchesPerRadian = Length{1.5};
    d.trackWidth = Length{12.0};
    d.source = hal::DescriptionSource::TypedTable;
    c.odometry.kind = cfgns::OdometryKind::DriveEncoders;
    c.sensors.walls.left.port = 4;
    c.sensors.walls.left.mount = goodMount();
    c.sensors.walls.right.port = 5;
    c.sensors.walls.right.mount = goodMount();
    c.sensors.walls.right.mount.left = Length{-7.5};
    c.sensors.walls.right.mount.facing = shulib::units::AngleDim{-90.0 * kDeg};
    return c;
}

std::string validWhy(const cfgns::RobotConfig& c) {
    char why[240] = "";
    return c.valid(why, sizeof why) ? std::string{"(valid)"} : std::string{why};
}

std::string mountWhy(const loc::SideMount& m) {
    const char* p = cfgns::wallMountProblem(m);
    return p == nullptr ? std::string{"(ok)"} : std::string{p};
}

std::string limitsWhy(const loc::WallResetLimits& l) {
    const char* p = cfgns::wallLimitsProblem(l);
    return p == nullptr ? std::string{"(ok)"} : std::string{p};
}

// L6-B's shape: a robot file's block in a constexpr table — and the checks usable at compile time.
constexpr cfgns::WallSensorsConfig kConstexprBlock{
    .left = {.port = 4, .mount = goodMount()},
    .right = {.port = 5, .mount = {}},
    .limits = {},
    .note = "constexpr table"};
static_assert(kConstexprBlock.declared());
static_assert(cfgns::wallMountProblem(kConstexprBlock.left.mount) == nullptr);
static_assert(cfgns::wallMountProblem(kConstexprBlock.right.mount) == nullptr);  // unmeasured: none
static_assert(cfgns::wallLimitsProblem(kConstexprBlock.limits) == nullptr);       // the defaults pass
static_assert(!cfgns::WallSensorsConfig{}.declared());

}  // namespace

// ═══ Defaults, and the port words ══════════════════════════════════════════════════

TEST_CASE("L6 T12 config: no side declared by default -- every older config stays valid; the claim's word") {
    cfgns::RobotConfig c = tank();
    c.sensors.walls = {};
    CHECK(validWhy(c) == "(valid)");
    CHECK_FALSE(c.sensors.walls.declared());
    CHECK(std::string{cfgns::portUseWord(cfgns::PortUse::WallSensor)} == "a wall sensor");
    CHECK(validWhy(tank()) == "(valid)");
    // One side only is fine (a robot with a single sensor, or one taken off for repair).
    cfgns::RobotConfig left = tank();
    left.sensors.walls.right = {};
    CHECK(validWhy(left) == "(valid)");
    CHECK(left.sensors.walls.declared());
}

// ═══ One owner per port (M25) ══════════════════════════════════════════════════════

// Bug caught (M25 — the claim removed): a wall sensor typed on a drive motor's port boots, and the
// provider is asked for a distance sensor on a motor's port — or the reset reads a motor as a wall.
TEST_CASE("L6 T12 config: a wall port claimed by anything else is refused NAMING BOTH") {
    cfgns::RobotConfig c = tank();
    c.sensors.walls.left.port = 11;  // the LEFT drive's first motor
    CHECK(validWhy(c) == "port 11 is claimed by the LEFT drive and by a wall sensor");
    c = tank();
    c.sensors.walls.right.port = 13;  // the RIGHT drive's
    CHECK(validWhy(c) == "port 13 is claimed by the RIGHT drive and by a wall sensor");
    c = tank();
    c.sensors.walls.right.port = 21;  // the IMU's
    CHECK(validWhy(c) == "port 21 is claimed by the IMU and by a wall sensor");
    // A mechanism's piece sensor on port 4 — claimed first (an older claim keeps its own message).
    c = tank();
    c.mechanismCount = 1;
    std::snprintf(c.mechanisms[0].name, sizeof c.mechanisms[0].name, "%s", "claw");
    c.mechanisms[0].motorCount = 1;
    c.mechanisms[0].motorPorts[0] = 6;
    c.mechanisms[0].cartridge = hal::MotorCartridge::Red;
    c.mechanisms[0].safeMode = cfgns::SafeBrakeMode::Hold;
    c.mechanisms[0].distancePort = 4;
    CHECK(validWhy(c) == "port 4 is claimed by a mechanism sensor and by a wall sensor");
    // The camera's port.
    c = tank();
    c.sensors.camera.port = 5;
    c.sensors.camera.settings.objects = true;
    c.sensors.camera.settings.expectedClasses = shulib::vision::override2026::kOverrideClassNames;
    c.sensors.camera.settings.expectedClassCount = shulib::vision::override2026::kModelClassCount;
    CHECK(validWhy(c) == "port 5 is claimed by the AI Vision camera and by a wall sensor");
    // The other side's port: its own sentence, naming the two sides.
    c = tank();
    c.sensors.walls.right.port = 4;
    CHECK(validWhy(c) == "the LEFT and RIGHT wall sensors are both on port 4 -- one sensor per side, each on "
                         "its own port");
    // Past 21 — the side named.
    c = tank();
    c.sensors.walls.right.port = 22;
    CHECK(validWhy(c) == "the RIGHT wall sensor's port 22 is not a port in 1..21 (0 = no sensor)");
    c = tank();
    c.sensors.walls.left.port = 255;
    CHECK(validWhy(c) == "the LEFT wall sensor's port 255 is not a port in 1..21 (0 = no sensor)");
    // Ports 1 and 21 are ports (the edges), when nothing else has them.
    c = tank();
    c.drivetrain.tank.description.imuPort = 20;
    c.sensors.walls.left.port = 1;
    c.sensors.walls.right.port = 21;
    CHECK(validWhy(c) == "(valid)");
    // A collision AND a bad mount: the collision is the older rule and wins.
    c = tank();
    c.sensors.walls.left.port = 11;
    c.sensors.walls.left.mount.height = Length{30.0};
    CHECK(validWhy(c) == "port 11 is claimed by the LEFT drive and by a wall sensor");
}

// ═══ The mount (M26) ═══════════════════════════════════════════════════════════════

// Bug caught (M26 — an unmeasured mount refused at boot): a robot that will not boot until someone
// has measured a sensor with a tape — so the team unplugs it, or types a guess "so it boots".
TEST_CASE("L6 T12 config: an UNMEASURED mount is VALID, whatever its numbers -- the reset refuses, never the "
          "boot") {
    cfgns::RobotConfig c = tank();
    c.sensors.walls.left.mount = loc::SideMount{};  // measured = false, all zero, no provenance
    CHECK(validWhy(c) == "(valid)");
    c.sensors.walls.left.mount.height = Length{kNaN};
    c.sensors.walls.left.mount.forward = Length{1e9};
    c.sensors.walls.left.mount.positionTolerance = Length{-1.0};
    CHECK(validWhy(c) == "(valid)");  // never read: every reset with it refuses MOUNT UNSET
    CHECK(mountWhy(c.sensors.walls.left.mount) == "(ok)");
    // The same numbers, MEASURED, are refused — naming the side and the port.
    c.sensors.walls.left.mount.measured = true;
    CHECK(validWhy(c) == "the LEFT wall sensor's mount (port 4): a number in the mount is not finite");
    // A mount on a side with NO port is never judged (nothing is declared there).
    cfgns::RobotConfig n = tank();
    n.sensors.walls.right.port = 0;
    n.sensors.walls.right.mount.height = Length{99.0};
    CHECK(validWhy(n) == "(valid)");
}

TEST_CASE("L6 T12 config: every bound of a MEASURED mount -- on its edge accepted, just past it refused, with "
          "its words") {
    const auto with = [](auto change) {
        loc::SideMount m = goodMount();
        change(m);
        return mountWhy(m);
    };
    const std::string offset = "the lens is more than 24 in from the robot's centre (forward or left) -- a typo?";
    CHECK(with([](loc::SideMount& m) { m.forward = Length{24.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.forward = Length{-24.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.forward = Length{24.01}; }) == offset);
    CHECK(with([](loc::SideMount& m) { m.forward = Length{-24.01}; }) == offset);
    CHECK(with([](loc::SideMount& m) { m.left = Length{24.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.left = Length{-24.01}; }) == offset);
    const std::string height = "height outside (0, 11.5] in -- a lens above the wall's top reads past it";
    CHECK(with([](loc::SideMount& m) { m.height = Length{11.5}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.height = Length{11.51}; }) == height);
    CHECK(with([](loc::SideMount& m) { m.height = Length{0.01}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.height = Length{0.0}; }) == height);
    CHECK(with([](loc::SideMount& m) { m.height = Length{-1.0}; }) == height);
    const std::string cone = "coneHalfAngle must be 0 (unmeasured) or 3..30 degrees";
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{0.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{3.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{30.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{2.99 * kDeg}; }) == cone);
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{30.01 * kDeg}; }) == cone);
    CHECK(with([](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{-12.0 * kDeg}; }) == cone);
    const std::string pos = "positionTolerance must be above 0 and at most 2 in -- state how well it was measured";
    CHECK(with([](loc::SideMount& m) { m.positionTolerance = Length{2.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.positionTolerance = Length{2.01}; }) == pos);
    CHECK(with([](loc::SideMount& m) { m.positionTolerance = Length{0.0}; }) == pos);
    CHECK(with([](loc::SideMount& m) { m.positionTolerance = Length{0.001}; }) == "(ok)");
    const std::string face = "facingTolerance must be above 0 and at most 10 degrees -- state how square it was found";
    CHECK(with([](loc::SideMount& m) { m.facingTolerance = shulib::units::AngleDim{10.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.facingTolerance = shulib::units::AngleDim{10.01 * kDeg}; }) == face);
    CHECK(with([](loc::SideMount& m) { m.facingTolerance = shulib::units::AngleDim{0.0}; }) == face);
    const std::string bias = "rangeBias is more than 2 in either way";
    CHECK(with([](loc::SideMount& m) { m.rangeBias = Length{2.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.rangeBias = Length{-2.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.rangeBias = Length{2.01}; }) == bias);
    CHECK(with([](loc::SideMount& m) { m.rangeBias = Length{-2.01}; }) == bias);
    const std::string acc = "rangeAccuracy must be 0 (unmeasured) or 0.05..2 in";
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{0.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{0.05}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{2.0}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{0.049}; }) == acc);
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{2.01}; }) == acc);
    CHECK(with([](loc::SideMount& m) { m.rangeAccuracy = Length{-0.3}; }) == acc);
    // The facing (a review finding of 2026-11-05): `90.0` typed alone is 90 RADIANS (about 116.6 degrees
    // once wrapped) — every reset would refuse NOT SQUARE and nothing at boot would say why. The edges
    // are typed as a robot file types them, N * kWallDegToRad: 180 accepted, -180 refused.
    const std::string facing =
        "facing outside (-180, 180] degrees -- type it as N * kWallDegToRad (90.0 alone is 90 RADIANS)";
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{180.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{-179.99 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{-180.0 * kDeg}; }) == facing);
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{180.01 * kDeg}; }) == facing);
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{90.0}; }) == facing);   // degrees typed
    CHECK(with([](loc::SideMount& m) { m.facing = shulib::units::AngleDim{-90.0}; }) == facing);
    const std::string prov = "no provenance -- who measured the mount, how, when?";
    CHECK(with([](loc::SideMount& m) { m.provenance = ""; }) == prov);
    CHECK(with([](loc::SideMount& m) { m.provenance = nullptr; }) == prov);
    // Every number, non-finite, is refused by name — NaN and both infinities.
    const std::string nonFinite = "a number in the mount is not finite";
    for (const double bad : {kNaN, kInf, -kInf}) {
        CAPTURE(bad);
        CHECK(with([bad](loc::SideMount& m) { m.forward = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.left = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.height = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.facing = shulib::units::AngleDim{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.positionTolerance = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.facingTolerance = shulib::units::AngleDim{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.rangeBias = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.rangeAccuracy = Length{bad}; }) == nonFinite);
        CHECK(with([bad](loc::SideMount& m) { m.coneHalfAngle = shulib::units::AngleDim{bad}; }) == nonFinite);
    }
    // Through valid(): the side and the port in the sentence.
    cfgns::RobotConfig c = tank();
    c.sensors.walls.right.mount.height = Length{12.0};
    CHECK(validWhy(c) == "the RIGHT wall sensor's mount (port 5): " + height);
}

// Bug caught (a review finding of 2026-11-05): a mount block copied from the OTHER side passes valid()
// — a LEFT sensor typed with the right's -90 degrees refuses every reset NOT SQUARE with nothing at boot
// saying why; one typed with the right's -7.5 in offset (and its own +90) puts the answer ~15 in off,
// refused DISAGREES at a 3-in maxCorrection — and SEATED 15 in wrong under a generous one. The edges:
// a facing strictly inside (0, 180) / (-180, 0) degrees, an offset strictly on its own side; 0 refused.
TEST_CASE("L6 T12 config: a MEASURED mount must face out of, and sit on, its OWN side -- a block copied from "
          "the other side is refused by name") {
    const auto side = [](const loc::SideMount& m, loc::WallSide s) {
        const char* p = cfgns::wallSideProblem(m, s);
        return p == nullptr ? std::string{"(ok)"} : std::string{p};
    };
    const std::string leftFace = "a LEFT sensor faces out the left side: facing between 0 and 180 degrees "
                                 "(typically +90) -- copied from the RIGHT block?";
    const std::string leftSits = "a LEFT sensor sits left of the robot's centre: `left` above 0 -- copied from the "
                                 "RIGHT block?";
    const std::string rightFace = "a RIGHT sensor faces out the right side: facing between -180 and 0 degrees "
                                  "(typically -90) -- copied from the LEFT block?";
    const std::string rightSits = "a RIGHT sensor sits right of the robot's centre: `left` below 0 -- copied from "
                                  "the LEFT block?";
    loc::SideMount l = goodMount();  // +7.5 in, +90 degrees
    CHECK(side(l, loc::WallSide::Left) == "(ok)");
    CHECK(side(l, loc::WallSide::Right) == rightFace);
    const auto facingAt = [&](double deg, loc::WallSide s) {
        loc::SideMount m = goodMount();
        if (s == loc::WallSide::Right) {
            m.left = Length{-7.5};
        }
        m.facing = shulib::units::AngleDim{deg * kDeg};
        return side(m, s);
    };
    CHECK(facingAt(0.01, loc::WallSide::Left) == "(ok)");
    CHECK(facingAt(179.99, loc::WallSide::Left) == "(ok)");
    CHECK(facingAt(0.0, loc::WallSide::Left) == leftFace);
    CHECK(facingAt(180.0, loc::WallSide::Left) == leftFace);
    CHECK(facingAt(-90.0, loc::WallSide::Left) == leftFace);
    CHECK(facingAt(-0.01, loc::WallSide::Right) == "(ok)");
    CHECK(facingAt(-179.99, loc::WallSide::Right) == "(ok)");
    CHECK(facingAt(0.0, loc::WallSide::Right) == rightFace);
    CHECK(facingAt(-180.0, loc::WallSide::Right) == rightFace);
    CHECK(facingAt(90.0, loc::WallSide::Right) == rightFace);
    // The offset, facing right for its side.
    loc::SideMount lo = goodMount();
    lo.left = Length{0.01};
    CHECK(side(lo, loc::WallSide::Left) == "(ok)");
    lo.left = Length{0.0};
    CHECK(side(lo, loc::WallSide::Left) == leftSits);
    lo.left = Length{-7.5};
    CHECK(side(lo, loc::WallSide::Left) == leftSits);
    loc::SideMount ro = goodMount();
    ro.facing = shulib::units::AngleDim{-90.0 * kDeg};
    ro.left = Length{-0.01};
    CHECK(side(ro, loc::WallSide::Right) == "(ok)");
    ro.left = Length{0.0};
    CHECK(side(ro, loc::WallSide::Right) == rightSits);
    ro.left = Length{7.5};
    CHECK(side(ro, loc::WallSide::Right) == rightSits);
    // UNMEASURED: never judged (the reset refuses MOUNT UNSET).
    loc::SideMount unset{};
    unset.left = Length{-7.5};
    unset.facing = shulib::units::AngleDim{-90.0 * kDeg};
    CHECK(side(unset, loc::WallSide::Left) == "(ok)");

    // Through valid(), naming the side and the port: the RIGHT block copied whole onto the LEFT...
    cfgns::RobotConfig c = tank();
    c.sensors.walls.left.mount = c.sensors.walls.right.mount;
    CHECK(validWhy(c) == "the LEFT wall sensor's mount (port 4): " + leftFace);
    // ...or only the offset's sign wrong on the RIGHT.
    cfgns::RobotConfig d = tank();
    d.sensors.walls.right.mount.left = Length{7.5};
    CHECK(validWhy(d) == "the RIGHT wall sensor's mount (port 5): " + rightSits);
    // An unmeasured copy is still valid: the robot boots.
    cfgns::RobotConfig e = tank();
    e.sensors.walls.left.mount = e.sensors.walls.right.mount;
    e.sensors.walls.left.mount.measured = false;
    CHECK(validWhy(e) == "(valid)");
}

// ═══ The limits ════════════════════════════════════════════════════════════════════

TEST_CASE("L6 T12 config: every bound of the LIMITS -- on its edge accepted, just past it refused, with its "
          "words; checked whether or not a side is declared") {
    const auto with = [](auto change) {
        loc::WallResetLimits l{};
        change(l);
        return limitsWhy(l);
    };
    CHECK(limitsWhy(loc::WallResetLimits{}) == "(ok)");
    const std::string range = "rangeMin/rangeMax must satisfy 0.79 in (VEX's 20 mm) <= rangeMin < rangeMax, both finite";
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMin = Length{0.79}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMin = Length{0.78}; }) == range);
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMin = Length{16.0}; }) == range);   // == rangeMax
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMin = Length{15.99}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMax = Length{kInf}; }) == range);
    CHECK(with([](loc::WallResetLimits& l) { l.rangeMin = Length{kNaN}; }) == range);
    const std::string samples = "minSamples must be 3..64";
    CHECK(with([](loc::WallResetLimits& l) { l.minSamples = 3; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.minSamples = 2; }) == samples);
    CHECK(with([](loc::WallResetLimits& l) { l.minSamples = 64; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.minSamples = 65; }) == samples);
    const std::string window = "window/windowMax must satisfy 0 < window <= windowMax <= 1 s";
    CHECK(with([](loc::WallResetLimits& l) { l.window = shulib::units::Time{0.5}; }) == "(ok)");  // == windowMax
    CHECK(with([](loc::WallResetLimits& l) { l.window = shulib::units::Time{0.51}; }) == window);
    CHECK(with([](loc::WallResetLimits& l) { l.windowMax = shulib::units::Time{1.0}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.windowMax = shulib::units::Time{1.01}; }) == window);
    CHECK(with([](loc::WallResetLimits& l) { l.window = shulib::units::Time{0.0}; }) == window);
    const std::string buffer = "window must be at most 0.6 s (the 64-sample buffer spans 0.63 s at 10 ms)";
    CHECK(with([](loc::WallResetLimits& l) {
              l.windowMax = shulib::units::Time{1.0};
              l.window = shulib::units::Time{0.6};
          }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) {
              l.windowMax = shulib::units::Time{1.0};
              l.window = shulib::units::Time{0.61};
          }) == buffer);
    const std::string floor = "headingFloor must be 0.5..10 degrees";
    CHECK(with([](loc::WallResetLimits& l) { l.headingFloor = shulib::units::AngleDim{0.5 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.headingFloor = shulib::units::AngleDim{10.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.headingFloor = shulib::units::AngleDim{0.49 * kDeg}; }) == floor);
    CHECK(with([](loc::WallResetLimits& l) { l.headingFloor = shulib::units::AngleDim{10.01 * kDeg}; }) == floor);
    const std::string perTurn = "headingPerTurn must be 0..0.1";
    CHECK(with([](loc::WallResetLimits& l) { l.headingPerTurn = 0.0; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.headingPerTurn = 0.1; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.headingPerTurn = 0.1001; }) == perTurn);
    CHECK(with([](loc::WallResetLimits& l) { l.headingPerTurn = -0.0001; }) == perTurn);
    CHECK(with([](loc::WallResetLimits& l) { l.headingPerTurn = kNaN; }) == perTurn);
    const std::string drift = "alongDriftRate must be 0..0.5";
    CHECK(with([](loc::WallResetLimits& l) { l.alongDriftRate = 0.0; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.alongDriftRate = 0.5; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.alongDriftRate = 0.5001; }) == drift);
    CHECK(with([](loc::WallResetLimits& l) { l.alongDriftRate = -0.01; }) == drift);
    const std::string piece = "pieceTop/beamMargin must be finite and >= 0";
    CHECK(with([](loc::WallResetLimits& l) { l.pieceTop = Length{0.0}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.pieceTop = Length{-0.01}; }) == piece);
    CHECK(with([](loc::WallResetLimits& l) { l.beamMargin = Length{-0.01}; }) == piece);
    CHECK(with([](loc::WallResetLimits& l) { l.beamMargin = Length{kInf}; }) == piece);
    const std::string square = "squareWithin must be above 0 and at most 20 degrees";
    CHECK(with([](loc::WallResetLimits& l) { l.squareWithin = shulib::units::AngleDim{20.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.squareWithin = shulib::units::AngleDim{20.01 * kDeg}; }) == square);
    CHECK(with([](loc::WallResetLimits& l) { l.squareWithin = shulib::units::AngleDim{0.0}; }) == square);
    const std::string coneL = "coneHalfAngle must be 3..30 degrees";
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{3.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{2.99 * kDeg}; }) == coneL);
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{0.0}; }) == coneL);
    // The cone's UPPER edge (30 degrees, typed as the header computes it — the same constant, so the
    // check lands ON the edge, not near it) accepted, just past it refused.
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{30.0 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{30.01 * kDeg}; }) == coneL);
    CHECK(with([](loc::WallResetLimits& l) { l.coneHalfAngle = shulib::units::AngleDim{kNaN}; }) == coneL);
    CHECK(with([](loc::WallResetLimits& l) { l.squareWithin = shulib::units::AngleDim{0.01 * kDeg}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.squareWithin = shulib::units::AngleDim{kNaN}; }) == square);
    const std::string spread = "spreadMax must be finite and above 0";
    CHECK(with([](loc::WallResetLimits& l) { l.spreadMax = Length{0.0}; }) == spread);
    CHECK(with([](loc::WallResetLimits& l) { l.spreadMax = Length{0.001}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.spreadMax = Length{kInf}; }) == spread);
    const std::string conf = "minConfidence must be 0..1";
    CHECK(with([](loc::WallResetLimits& l) { l.minConfidence = 1.01; }) == conf);
    CHECK(with([](loc::WallResetLimits& l) { l.minConfidence = 1.0; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.minConfidence = 0.0; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.minConfidence = -0.01; }) == conf);
    CHECK(with([](loc::WallResetLimits& l) { l.minConfidence = kNaN; }) == conf);
    const std::string still = "stillSpeed/stillTurn must be finite and above 0";
    CHECK(with([](loc::WallResetLimits& l) { l.stillSpeed = shulib::units::Velocity{0.0}; }) == still);
    CHECK(with([](loc::WallResetLimits& l) { l.stillSpeed = shulib::units::Velocity{0.001}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.stillSpeed = shulib::units::Velocity{kInf}; }) == still);
    CHECK(with([](loc::WallResetLimits& l) { l.stillTurn = shulib::units::AngularVelocity{0.0}; }) == still);
    CHECK(with([](loc::WallResetLimits& l) { l.stillTurn = shulib::units::AngularVelocity{0.001}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.stillTurn = shulib::units::AngularVelocity{kInf}; }) == still);
    const std::string bound = "errorBoundMax must be finite and above 0";
    CHECK(with([](loc::WallResetLimits& l) { l.errorBoundMax = Length{0.0}; }) == bound);
    CHECK(with([](loc::WallResetLimits& l) { l.errorBoundMax = Length{0.001}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.errorBoundMax = Length{kInf}; }) == bound);
    const std::string margin = "keepOutMargin must be finite and >= 0";
    CHECK(with([](loc::WallResetLimits& l) { l.keepOutMargin = Length{-0.01}; }) == margin);
    CHECK(with([](loc::WallResetLimits& l) { l.keepOutMargin = Length{0.0}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.keepOutMargin = Length{kInf}; }) == margin);
    CHECK(with([](loc::WallResetLimits& l) { l.beamMargin = Length{0.0}; }) == "(ok)");
    CHECK(with([](loc::WallResetLimits& l) { l.pieceTop = Length{kInf}; }) == piece);
    CHECK(with([](loc::WallResetLimits& l) { l.alongDriftRate = kNaN; }) == drift);

    // Through valid(), with sides declared and with none: the limits are the robot's either way.
    cfgns::RobotConfig c = tank();
    c.sensors.walls.limits.minSamples = 2;
    CHECK(validWhy(c) == "the wall sensors' limits: " + samples);
    c.sensors.walls.left.port = 0;
    c.sensors.walls.right.port = 0;
    CHECK(validWhy(c) == "the wall sensors' limits: " + samples);
}

// Bug caught: a robot file that passes valid() and then refuses BAD REQUEST at every reset, from its
// own limits or its own mount — the boot said yes and the field says no, with nobody looking at the
// boot line. THE INVARIANT, over 20 000 seeded draws scattered around every edge of both checks.
TEST_CASE("L6 T12 config: every limits block and mount valid() accepts, the reset accepts too -- never BAD "
          "REQUEST from the robot file") {
    shulib::sim::Rng rng{20261103};
    // Each field keeps a good value 85 % of the time and is otherwise scattered across its edges, so a
    // draw usually carries one or a few edge values against a sane rest — the case a typo makes.
    const auto pick = [&rng](double good, double centre, double spread) {
        if (rng.nextUnit() < 0.85) {
            return good;
        }
        return centre + spread * (2.0 * rng.nextUnit() - 1.0);
    };
    loc::WallResetRequest req{};
    req.wall = loc::override2026::kLeftWall;
    req.sensor = loc::WallSide::Left;
    req.maxCorrection = Length{3.0};
    int accepted = 0;
    int refusedByConfig = 0;
    for (int i = 0; i < 20000; ++i) {
        loc::WallResetLimits l{};
        l.rangeMin = Length{pick(2.0, 1.0, 1.0)};
        l.rangeMax = Length{pick(16.0, 8.0, 9.0)};
        l.squareWithin = shulib::units::AngleDim{pick(5.0, 10.0, 12.0) * kDeg};
        l.coneHalfAngle = shulib::units::AngleDim{pick(18.0, 16.0, 15.0) * kDeg};
        l.spreadMax = Length{pick(0.5, 0.5, 0.6)};
        l.minSamples = static_cast<int>(std::floor(pick(5.0, 30.0, 40.0)));
        l.window = shulib::units::Time{pick(0.15, 0.5, 0.6)};
        l.windowMax = shulib::units::Time{pick(0.5, 0.6, 0.6)};
        l.minConfidence = pick(0.5, 0.5, 0.6);
        l.stillSpeed = shulib::units::Velocity{pick(1.0, 1.0, 1.2)};
        l.stillTurn = shulib::units::AngularVelocity{pick(3.0, 3.0, 3.5) * kDeg};
        l.errorBoundMax = Length{pick(1.0, 1.0, 1.2)};
        l.keepOutMargin = Length{pick(3.0, 1.0, 1.2)};
        l.alongDriftRate = pick(0.1, 0.25, 0.3);
        l.pieceTop = Length{pick(6.5, 3.0, 3.3)};
        l.beamMargin = Length{pick(0.5, 0.5, 0.6)};
        l.headingFloor = shulib::units::AngleDim{pick(2.0, 5.0, 5.6) * kDeg};
        l.headingPerTurn = pick(0.02, 0.05, 0.06);
        loc::SideMount m = goodMount();
        m.forward = Length{pick(0.0, 0.0, 26.0)};
        m.left = Length{pick(7.5, 0.0, 26.0)};
        m.height = Length{pick(10.5, 6.0, 6.5)};
        m.coneHalfAngle = shulib::units::AngleDim{pick(0.0, 16.0, 17.0) * kDeg};
        m.positionTolerance = Length{pick(0.25, 1.0, 1.2)};
        m.facingTolerance = shulib::units::AngleDim{pick(2.0, 5.0, 6.0) * kDeg};
        m.rangeBias = Length{pick(0.0, 0.0, 2.3)};
        m.rangeAccuracy = Length{pick(0.0, 1.0, 1.1)};
        if (cfgns::wallLimitsProblem(l) != nullptr || cfgns::wallMountProblem(m) != nullptr) {
            ++refusedByConfig;
            continue;
        }
        ++accepted;
        const char* reset = loc::wallResetProblem(m, req, l);
        if (reset != nullptr) {
            CAPTURE(i);
            CAPTURE(reset);
            FAIL("valid() accepted a robot file the reset refuses BAD REQUEST");
        }
    }
    // Both sides of the edge were really exercised (the draws are not all on one side).
    CHECK(accepted > 3000);
    CHECK(refusedByConfig > 3000);
}
