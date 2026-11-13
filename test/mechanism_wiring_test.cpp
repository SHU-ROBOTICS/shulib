// mechanism/wiring.hpp — THE BINDING, its run-time mirror, THE ONE PROJECTION and the boot line
// (chunk MECH-A1, the MECH brief's §3.1 "the wiring").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * ROBOT TWO'S PROJECTION IS IDENTICAL to what the tree builds today. The claws written in the new
//     words (test/mechanism_fixtures.hpp — MECH-A5's region, verbatim) go through
//     mechanism::toMechanismConfig; today's table (src/robots/mechanism_table.hpp) goes through
//     today's src/common/boot_source.hpp bootConfigFrom — the SAME function the programs boot
//     through, so the test fails if EITHER side moves (P9a). Every field, compared one by one.
//   * THE ONE PROJECTION, field by field: a Reversed role is a NEGATIVE port (mutation M2 drops it);
//     per-line levels appear ONLY when a position's levels differ across its lines; each sensor
//     role's port or line lands in its own field.
//   * THE MIRROR, both directions: wiringProblem() refuses, for a hand-built table, every wiring
//     mistake the build refuses in a robot file (test/compile_refusals/ holds the build-time half of
//     each pair) — and accepts every fixture robot, beside robot two's drive; and every entry BUILT BY
//     HAND that mechanisms() refuses at build (test/mechanism_hand_built.hpp, one per per-entry rule,
//     the SAME constants test/compile_refusals/hb_*.cpp compile) is named by entryProblem() and
//     refused by wiringProblem() (MECH-A1's review: hand-built entries used to walk around wire()).
//   * The far edges: a robot of exactly EIGHT mechanisms and a FOUR-motor mechanism are accepted (at
//     build and by the mirror); a collision through a piece SWITCH's line is seen.
//   * The boot line, VERBATIM in every state (P4's lines), and its bounds (truncation, null, cap 0).

#include "doctest.h"

#include <array>
#include <set>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_hand_built.hpp"
#include "common/boot_source.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/fake/fake_device_registry.hpp"
#include "shulib/mechanism/wiring.hpp"

namespace mech = shulib::mechanism;
namespace cfgns = shulib::config;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;
using mech::Line;

namespace {

/// Every field of the device-level config, compared — the identity test's definition of "the same".
void checkSameConfig(const cfgns::MechanismConfig& a, const cfgns::MechanismConfig& b) {
    CHECK(std::strncmp(a.name, b.name, sizeof a.name) == 0);
    CHECK(a.motorCount == b.motorCount);
    for (std::size_t i = 0; i < cfgns::kMaxMotorsPerMechanism; ++i) {
        CHECK(a.motorPorts[i] == b.motorPorts[i]);
    }
    CHECK(a.cartridge == b.cartridge);
    CHECK(a.safeMode == b.safeMode);
    CHECK(a.pneumaticCount == b.pneumaticCount);
    for (std::size_t i = 0; i < cfgns::kMaxPneumaticsPerMechanism; ++i) {
        CHECK(a.pneumaticPorts[i] == b.pneumaticPorts[i]);
        CHECK(a.pneumaticSafeByLine[i] == b.pneumaticSafeByLine[i]);
        CHECK(a.pneumaticStartByLine[i] == b.pneumaticStartByLine[i]);
    }
    CHECK(a.pneumaticSafe == b.pneumaticSafe);
    CHECK(a.pneumaticStart == b.pneumaticStart);
    CHECK(a.rotationPort == b.rotationPort);
    CHECK(a.distancePort == b.distancePort);
    CHECK(a.opticalPort == b.opticalPort);
    CHECK(a.switchLines[0] == b.switchLines[0]);
    CHECK(a.switchLines[1] == b.switchLines[1]);
}

/// Robot two's calibration record as boot_source_test.cpp rebuilds it from the 2026-09-25 capture
/// (its ports and scales; this test needs only a description the port check passes).
shulib::hal::DrivetrainDescription robotTwoRecord() {
    shulib::hal::DrivetrainDescription d{};
    d.kind = shulib::hal::DrivetrainKind::Tank;
    d.cartridge = shulib::hal::MotorCartridge::Blue;
    d.imuPort = 2;
    constexpr std::int8_t left[4] = {-16, 17, -18, -20};
    constexpr std::int8_t right[4] = {11, -12, 13, 15};
    for (std::size_t i = 0; i < 4; ++i) {
        d.left.signedPorts[i] = left[i];
        d.right.signedPorts[i] = right[i];
    }
    d.left.count = 4;
    d.right.count = 4;
    d.left.inchesPerRadian = shulib::units::Length{1.25442};
    d.right.inchesPerRadian = shulib::units::Length{1.36703};
    d.trackWidth = shulib::units::Length{12.006};
    d.source = shulib::hal::DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "%s", "mechanism wiring test");
    return d;
}

/// A table a test edits by hand — the run-time mirror's input.
struct Hand {
    std::vector<mech::MechanismSpec> specs;
    std::vector<mech::MechanismWiring> wiring;
    const char* problem(std::span<const std::uint8_t> drive = {}, std::uint8_t imu = 0) const {
        return mech::wiringProblem(specs, wiring, drive, imu);
    }
};

Hand handOf(const mech::MechanismTable& t) {
    Hand h;
    for (std::size_t i = 0; i < t.count; ++i) {
        h.specs.push_back(t.specs[i]);
        h.wiring.push_back(t.wiring[i]);
    }
    return h;
}

std::string text(const char* s) { return s != nullptr ? std::string{s} : std::string{"(null)"}; }

std::string line(const char* name, const char* role, std::uint8_t port, Line l, mech::RoleState st,
                 const char* refusal, bool degrade) {
    char out[240];
    mech::roleLine(out, sizeof out, name, role, port, l, st, refusal, degrade);
    return out;
}

constexpr std::uint8_t kRobotTwoDrive[] = {11, 12, 13, 15, 16, 17, 18, 20};

}  // namespace

TEST_CASE("MECH-A1 wiring: robot x's list -- every port where the robot file wrote it, every declaration found") {
    const mech::MechanismTable& t = rx::kRobotXMechanisms;
    REQUIRE(t.count == 5);
    CHECK(std::string{t.wiring[0].mechanism} == "arm");
    CHECK(t.wiring[0].motors[0] == 1);
    CHECK(t.wiring[1].motors[0] == 2);
    CHECK(t.wiring[1].motors[1] == 3);
    CHECK(t.wiring[1].homeSwitch == Line::B);
    CHECK(t.wiring[2].motors[0] == 4);
    CHECK(t.wiring[3].motors[0] == 5);
    CHECK(t.wiring[3].pieceSensorPort == 9);
    CHECK(t.wiring[4].lines[0] == Line::C);
    CHECK(t.wiring[4].lines[1] == Line::D);
    CHECK(t.specs[4].lineCount == 2);  // the WIRING said how many lines (lines('C', 'D'))
    CHECK(t.declared[0] == &rx::arm);
    CHECK(t.indexOf(&rx::lift) == 1);
    CHECK(t.indexOf(&rx::hood) == 4);
    CHECK(t.indexOf(&r2::claw) == -1);  // another robot's declaration is not on this list
    CHECK(t.specSpan().size() == 5U);
    CHECK(t.wiringSpan().size() == 5U);
    constexpr mech::MechanismTable none = mech::mechanisms();
    CHECK(none.count == 0);
    CHECK(none.specSpan().empty());
}

TEST_CASE("MECH-A1 wiring: ROBOT TWO's projection is IDENTICAL to today's bootConfigFrom, field by field (P9a)") {
    // The REAL side: src/robots/mechanism_table.hpp's list through the real boot conversion — until
    // MECH-A5 the OLD table's entries and bootConfigFrom's own conversion; since MECH-A5 the real list
    // in the core's words, which bootConfigFrom projects with toMechanismConfig — so this now pins the
    // FIXTURE (test/mechanism_fixtures.hpp's robot two) to the real file, field by field, and the
    // literal facts below pin both. It fails if either moves.
    shulib::hal::fake::FakeDeviceRegistry census;
    for (const int p : {11, 12, 13, 15, 16, 17, 18, 20, 6, 7}) {
        census.setKind(p, shulib::hal::DeviceKind::Motor);
    }
    census.setKind(2, shulib::hal::DeviceKind::Imu);
    const shulib::hal::DrivetrainDescription record = robotTwoRecord();
    const shulib::calibration::BootVerdict v = shulib::calibration::verifyAgainstRegistry(record, census);
    REQUIRE(v.usable());
    const shulib::bench::BootConfig today =
        shulib::bench::bootConfigFrom(record, v, "robot two", shulib::bench::robot_two::kRobotTwoMechanisms);
    const cfgns::RobotConfig& c = today.config;
    // The core's side: robot two in the new words, in its list's order (pivot 6, pivot 7, claw —
    // today's order: motors, then pneumatics).
    const mech::MechanismTable& t = r2::kRobotTwoMechanisms;
    REQUIRE(c.mechanismCount == 3);
    REQUIRE(t.count == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        checkSameConfig(mech::toMechanismConfig(t.specs[i], t.wiring[i]), c.mechanisms[i]);
    }
    // The literal facts, so "identical" cannot be identical-and-both-wrong: +6/+7 Red Hold; line A,
    // start and safe Low (closed; HA-159's guess is that High opens it).
    const cfgns::MechanismConfig p6 = mech::toMechanismConfig(t.specs[0], t.wiring[0]);
    CHECK(p6.motorPorts[0] == 6);
    CHECK(p6.cartridge == shulib::hal::MotorCartridge::Red);
    CHECK(p6.safeMode == cfgns::SafeBrakeMode::Hold);
    const cfgns::MechanismConfig claw = mech::toMechanismConfig(t.specs[2], t.wiring[2]);
    CHECK(claw.pneumaticCount == 1);
    CHECK(claw.pneumaticPorts[0] == 1);
    CHECK(claw.pneumaticStart == cfgns::PneumaticLevel::Low);
    CHECK(claw.pneumaticSafe == cfgns::PneumaticLevel::Low);
    CHECK(claw.pneumaticStartByLine[0] == static_cast<cfgns::PneumaticLevel>(0));  // the one-level form
}

TEST_CASE("MECH-A1 wiring: THE ONE PROJECTION, field by field -- signs, per-line levels, every sensor role") {
    SUBCASE("a Reversed role is a NEGATIVE port (M2's net); a Forward one positive") {
        const cfgns::MechanismConfig lift = mech::toMechanismConfig(rx::kRobotXMechanisms.specs[1], rx::kRobotXMechanisms.wiring[1]);
        CHECK(lift.motorCount == 2);
        CHECK(lift.motorPorts[0] == 2);
        CHECK(lift.motorPorts[1] == -3);
        CHECK(lift.switchLines[0] == 2);  // the home switch on B
        CHECK(lift.switchLines[1] == 0);
        CHECK(lift.cartridge == shulib::hal::MotorCartridge::Red);
        const cfgns::MechanismConfig dr4b = mech::toMechanismConfig(ry::kRobotYMechanisms.specs[2], ry::kRobotYMechanisms.wiring[2]);
        CHECK(dr4b.motorPorts[0] == 3);
        CHECK(dr4b.motorPorts[1] == -4);
    }
    SUBCASE("each sensor role in its own field") {
        const mech::MechanismTable& y = ry::kRobotYMechanisms;
        const cfgns::MechanismConfig wrist = mech::toMechanismConfig(y.specs[0], y.wiring[0]);
        CHECK(wrist.rotationPort == 10);
        CHECK(wrist.distancePort == 0);
        CHECK(wrist.opticalPort == 0);
        const cfgns::MechanismConfig roller = mech::toMechanismConfig(y.specs[4], y.wiring[4]);
        CHECK(roller.opticalPort == 9);
        CHECK(roller.distancePort == 0);
        const cfgns::MechanismConfig catapult = mech::toMechanismConfig(y.specs[5], y.wiring[5]);
        CHECK(catapult.switchLines[1] == 5);  // the piece switch on E
        CHECK(catapult.switchLines[0] == 0);
        const cfgns::MechanismConfig grip = mech::toMechanismConfig(y.specs[6], y.wiring[6]);
        CHECK(grip.distancePort == 8);
        CHECK(grip.pneumaticPorts[0] == 1);
        CHECK(grip.pneumaticStart == cfgns::PneumaticLevel::Low);   // "loose" = Low
        CHECK(grip.pneumaticSafe == cfgns::PneumaticLevel::High);   // "shut" = High
        const cfgns::MechanismConfig intake = mech::toMechanismConfig(rx::kRobotXMechanisms.specs[3], rx::kRobotXMechanisms.wiring[3]);
        CHECK(intake.distancePort == 9);
    }
    SUBCASE("per-line levels ONLY when a position's levels differ across its lines") {
        // Robot x's hood starts and is made safe "down" = Low/Low: uniform -> the one-level form.
        const cfgns::MechanismConfig hood = mech::toMechanismConfig(rx::kRobotXMechanisms.specs[4], rx::kRobotXMechanisms.wiring[4]);
        CHECK(hood.pneumaticCount == 2);
        CHECK(hood.pneumaticPorts[0] == 3);
        CHECK(hood.pneumaticPorts[1] == 4);
        CHECK(hood.pneumaticStart == cfgns::PneumaticLevel::Low);
        CHECK(hood.pneumaticSafe == cfgns::PneumaticLevel::Low);
        for (std::size_t l = 0; l < 2; ++l) {
            CHECK(hood.pneumaticStartByLine[l] == static_cast<cfgns::PneumaticLevel>(0));
            CHECK(hood.pneumaticSafeByLine[l] == static_cast<cfgns::PneumaticLevel>(0));
        }
        // The same hood STARTING "middle" (High/Low): per-line, for BOTH start and safe.
        using namespace shulib::mechanism::words;
        static constexpr Pneumatic midStart{.name = "hood",
                                     .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
                                     .start = "middle", .safe = "down", .moveTime = 0.3_s};
        static constexpr mech::MechanismTable t = mech::mechanisms(wire(midStart, lines('C', 'D')));
        const cfgns::MechanismConfig mid = mech::toMechanismConfig(t.specs[0], t.wiring[0]);
        CHECK(mid.pneumaticStartByLine[0] == cfgns::PneumaticLevel::High);
        CHECK(mid.pneumaticStartByLine[1] == cfgns::PneumaticLevel::Low);
        CHECK(mid.pneumaticSafeByLine[0] == cfgns::PneumaticLevel::Low);
        CHECK(mid.pneumaticSafeByLine[1] == cfgns::PneumaticLevel::Low);
        CHECK(mid.pneumaticStart == cfgns::PneumaticLevel::High);  // line 1's, kept for older readers
        // Uniform start but a NON-uniform safe ("up" = High/High is uniform; "middle" is not).
        static constexpr Pneumatic midSafe{.name = "hood",
                                    .positions = {{"down", {Low, Low}}, {"middle", {High, Low}}, {"up", {High, High}}},
                                    .start = "up", .safe = "middle", .moveTime = 0.3_s};
        static constexpr mech::MechanismTable t2 = mech::mechanisms(wire(midSafe, lines('C', 'D')));
        const cfgns::MechanismConfig ms = mech::toMechanismConfig(t2.specs[0], t2.wiring[0]);
        CHECK(ms.pneumaticSafeByLine[0] == cfgns::PneumaticLevel::High);
        CHECK(ms.pneumaticSafeByLine[1] == cfgns::PneumaticLevel::Low);
        CHECK(ms.pneumaticStartByLine[0] == cfgns::PneumaticLevel::High);
        CHECK(ms.pneumaticStartByLine[1] == cfgns::PneumaticLevel::High);
    }
}

TEST_CASE("MECH-A1 wiring: declaredMotorPorts is every bound motor port, physical, in list order") {
    std::array<int, 21> out{};
    CHECK(mech::declaredMotorPorts(rx::kRobotXMechanisms, out) == 5U);
    CHECK(out[0] == 1);
    CHECK(out[1] == 2);
    CHECK(out[2] == 3);  // the REVERSED lift motor: its PHYSICAL port, never -3
    CHECK(out[3] == 4);
    CHECK(out[4] == 5);
    CHECK(mech::declaredMotorPorts(r2::kRobotTwoMechanisms, out) == 2U);
    CHECK(out[0] == 6);
    CHECK(out[1] == 7);
    std::array<int, 2> small{};
    CHECK(mech::declaredMotorPorts(rx::kRobotXMechanisms, small) == 2U);  // bounded by the span
    CHECK(mech::declaredMotorPorts(mech::mechanisms(), out) == 0U);
}

TEST_CASE("MECH-A1 wiring: the DRIVE check -- clearOfDrive beside a typed chassis table") {
    constexpr std::uint8_t drive[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    CHECK(mech::clearOfDrive(rx::kRobotXMechanisms, drive, 21));
    CHECK(mech::clearOfDrive(r2::kRobotTwoMechanisms, kRobotTwoDrive, 2));
    constexpr std::uint8_t withFive[] = {11, 12, 13, 14, 5};
    CHECK_FALSE(mech::clearOfDrive(rx::kRobotXMechanisms, withFive, 21));  // the intake's motor port
    CHECK_FALSE(mech::clearOfDrive(rx::kRobotXMechanisms, drive, 9));      // the IMU on the intake's SENSOR port
    CHECK_FALSE(mech::clearOfDrive(ry::kRobotYMechanisms, drive, 10));     // the IMU on the wrist's ANGLE sensor
    CHECK(mech::clearOfDrive(ry::kRobotYMechanisms, drive, 21));
}

TEST_CASE("MECH-A1 wiring: wiringProblem -- the run-time MIRROR of what the build refuses, one mistake at a time") {
    // Every fixture robot is clean beside a drive that shares nothing with it.
    constexpr std::uint8_t drive[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    CHECK(handOf(rx::kRobotXMechanisms).problem(drive, 21) == nullptr);
    CHECK(handOf(ry::kRobotYMechanisms).problem(drive, 21) == nullptr);
    CHECK(handOf(r2::kRobotTwoMechanisms).problem(kRobotTwoDrive, 2) == nullptr);
    struct Mistake {
        const char* what;
        void (*edit)(Hand&);
        const char* expected;
    };
    const Mistake mistakes[] = {
        {"a smart port used twice (n10)", [](Hand& h) { h.wiring[3].motors[0] = 1; }, "a smart port is bound twice"},
        {"a SENSOR port on a motor's port", [](Hand& h) { h.wiring[3].pieceSensorPort = 4; }, "a smart port is bound twice"},
        {"a 3-wire line used twice (n28)", [](Hand& h) { h.wiring[1].homeSwitch = Line::C; }, "a 3-wire line is bound twice"},
        {"port 22 (n30)", [](Hand& h) { h.wiring[0].motors[0] = 22; }, "a smart-port role is unbound or not 1..21"},
        {"a motor role left unbound (n12)", [](Hand& h) { h.wiring[1].motors[1] = 0; }, "a smart-port role is unbound or not 1..21"},
        {"a home switch left unplugged (n13)", [](Hand& h) { h.wiring[1].homeSwitch = Line::None; },
         "a 3-wire role is unbound or not A..H"},
        {"an angle sensor the arm does not have", [](Hand& h) { h.wiring[0].angleSensor = 10; },
         "an angle sensor port is bound on a mechanism with no angle sensor role"},
        {"a motor port on a role the arm does not have", [](Hand& h) { h.wiring[0].motors[1] = 7; },
         "a motor port is bound to a role the mechanism does not have"},
        {"a line on a role the hood does not have", [](Hand& h) { h.specs[4].lineCount = 1; },
         "a 3-wire line is bound to a role the mechanism does not have"},
        {"a home switch on a mechanism with none", [](Hand& h) { h.wiring[0].homeSwitch = Line::E; },
         "a home switch line is bound on a mechanism with no home switch role"},
        {"a piece port on a mechanism with none", [](Hand& h) { h.wiring[2].pieceSensorPort = 8; },
         "a piece sensor port is bound on a mechanism with no distance/optical piece sensor"},
        {"a piece switch line on a mechanism with none", [](Hand& h) { h.wiring[2].pieceSwitch = Line::F; },
         "a piece switch line is bound on a mechanism with no switch piece sensor"},
        {"two mechanisms sharing a name (n38)",
         [](Hand& h) { std::strcpy(h.specs[1].name, "arm"); std::strcpy(h.wiring[1].mechanism, "arm"); },
         "two mechanisms share a name"},
        {"the wiring naming another mechanism at its place", [](Hand& h) { std::strcpy(h.wiring[2].mechanism, "lift"); },
         "the wiring names another mechanism at this place"},
        {"a mechanism with no wiring", [](Hand& h) { h.wiring.pop_back(); }, "the wiring does not list every mechanism, in order"},
        {"a position's levels vs the lines (U2, n42/n43)", [](Hand& h) { h.specs[4].pneumatic.positions[1].levelCount = 1; },
         "a position states a different number of levels than the wiring has lines"},
    };
    for (const Mistake& m : mistakes) {
        CAPTURE(m.what);
        Hand h = handOf(rx::kRobotXMechanisms);
        m.edit(h);
        CHECK(text(h.problem(drive, 21)) == m.expected);
    }
    SUBCASE("a drive port and the IMU's") {
        constexpr std::uint8_t withOne[] = {11, 1};
        CHECK(text(handOf(rx::kRobotXMechanisms).problem(withOne, 21)) == "a mechanism role is bound to a DRIVE port");
        CHECK(text(handOf(rx::kRobotXMechanisms).problem(drive, 9)) == "a mechanism role is bound to the IMU's port");
    }
    SUBCASE("more than eight mechanisms") {
        Hand h = handOf(rx::kRobotXMechanisms);
        Hand more = h;
        for (int k = 0; k < 4; ++k) {
            more.specs.push_back(h.specs[0]);
            more.wiring.push_back(h.wiring[0]);
        }
        CHECK(text(more.problem()) == "more mechanisms than a robot can have");
        Hand eight = h;  // exactly EIGHT is a full robot, not too many (robot x's five, three renamed copies)
        for (int k = 0; k < 3; ++k) {
            eight.specs.push_back(h.specs[3]);
            eight.wiring.push_back(h.wiring[3]);
        }
        for (std::size_t k = 5; k < 8; ++k) {
            const char name[] = {'i', 'n', static_cast<char>('0' + k), '\0'};
            std::memcpy(eight.specs[k].name, name, sizeof name);
            std::memcpy(eight.wiring[k].mechanism, name, sizeof name);
            eight.wiring[k].motors[0] = static_cast<std::uint8_t>(5 + k);       // 10, 11, 12
            eight.wiring[k].pieceSensorPort = static_cast<std::uint8_t>(10 + k);  // 15, 16, 17
        }
        REQUIRE(eight.specs.size() == 8U);
        CHECK(eight.problem() == nullptr);
    }
    SUBCASE("a collision through a piece SWITCH's line (robot y's catapult on the grip's line A)") {
        Hand y = handOf(ry::kRobotYMechanisms);
        REQUIRE(std::string{y.specs[5].name} == "catapult");
        y.wiring[5].pieceSwitch = Line::A;
        CHECK(text(y.problem(drive, 21)) == "a 3-wire line is bound twice");
    }
}

namespace eight_and_four {
using namespace shulib::mechanism::words;
inline constexpr Power p1{.name = "p1", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p2{.name = "p2", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p3{.name = "p3", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p4{.name = "p4", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p5{.name = "p5", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p6{.name = "p6", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p7{.name = "p7", .motors = {Forward}, .cartridge = Red, .safe = Coast};
inline constexpr Power p8{.name = "p8", .motors = {Forward}, .cartridge = Red, .safe = Coast};
/// A FOUR-motor mechanism, the most one may have.
inline constexpr Power quad{.name = "quad", .motors = {Forward, Reversed, Forward, Reversed}, .cartridge = Red, .safe = Coast};
inline constexpr mech::MechanismTable kEight = mechanisms(wire(p1, motors(1)), wire(p2, motors(2)), wire(p3, motors(3)),
                                                         wire(p4, motors(4)), wire(p5, motors(5)), wire(p6, motors(6)),
                                                         wire(p7, motors(7)), wire(p8, motors(8)));
inline constexpr mech::MechanismTable kQuad = mechanisms(wire(quad, motors(1, 2, 3, 4)), wire(p5, motors(5)));
}  // namespace eight_and_four

TEST_CASE("MECH-A1 wiring: the far edges ACCEPTED -- a robot of exactly eight mechanisms, a four-motor mechanism") {
    namespace e = eight_and_four;
    static_assert(e::kEight.count == 8);  // compiled: mechanisms() does not refuse a full robot
    constexpr std::uint8_t drive[] = {11, 12, 13, 14, 15, 16, 17, 18, 19, 20};
    CHECK(mech::clearOfDrive(e::kEight, drive, 21));
    CHECK(handOf(e::kEight).problem(drive, 21) == nullptr);
    static_assert(e::kQuad.count == 2);
    REQUIRE(e::kQuad.specs[0].motorCount == 4);
    CHECK(e::kQuad.wiring[0].motors[3] == 4);
    CHECK(handOf(e::kQuad).problem(drive, 21) == nullptr);
    const cfgns::MechanismConfig q = mech::toMechanismConfig(e::kQuad.specs[0], e::kQuad.wiring[0]);
    CHECK(q.motorCount == 4);
    CHECK(q.motorPorts[1] == -2);  // its mirrored motors: negative ports
    CHECK(q.motorPorts[3] == -4);
}

TEST_CASE("MECH-A1 wiring: THE MIRROR, second direction -- every hand-built entry mechanisms() refuses, entryProblem names and wiringProblem refuses") {
    namespace hb = mech_fixture::hand_built;
    using EP = mech::EntryProblem;
    struct Row {
        const char* what;
        const mech::Wired& entry;
        EP expected;
        const char* sentence;  // wiringProblem's
    };
    const Row rows[] = {
        {"the wiring names another mechanism", hb::kNameDiffers, EP::NameDiffers, "the wiring names another mechanism at this place"},
        {"a motor role on port 0", hb::kMotorUnbound, EP::MotorRoleUnbound, "a smart-port role is unbound or not 1..21"},
        {"a motor role on port 22", hb::kMotorPort22, EP::SmartPortPast21, "a smart-port role is unbound or not 1..21"},
        {"an angle sensor on port 22", hb::kAnglePort22, EP::SmartPortPast21, "a smart-port role is unbound or not 1..21"},
        {"a motor port on a role the arm lacks", hb::kMotorExtra, EP::MotorPortExtra,
         "a motor port is bound to a role the mechanism does not have"},
        {"a line role unbound", hb::kLineUnbound, EP::LineRoleUnbound, "a 3-wire role is unbound or not A..H"},
        {"a line past H", hb::kLine9, EP::LinePastH, "a 3-wire role is unbound or not A..H"},
        {"a home switch past H", hb::kHomeLine9, EP::LinePastH, "a 3-wire role is unbound or not A..H"},
        {"a line on a role the claw lacks", hb::kLineExtra, EP::LineExtra,
         "a 3-wire line is bound to a role the mechanism does not have"},
        {"a position's levels vs the lines (U2)", hb::kLevelsVsLines, EP::LevelsVsLines,
         "a position states a different number of levels than the wiring has lines"},
        {"the wrist's angle sensor unplugged", hb::kAngleUnplugged, EP::AngleSensorUnplugged,
         "a smart-port role is unbound or not 1..21"},
        {"an angle sensor on the arm", hb::kAngleExtra, EP::AngleSensorExtra,
         "an angle sensor port is bound on a mechanism with no angle sensor role"},
        {"the lift's home switch unplugged", hb::kHomeUnplugged, EP::HomeSwitchUnplugged, "a 3-wire role is unbound or not A..H"},
        {"a home switch on the arm", hb::kHomeExtra, EP::HomeSwitchExtra,
         "a home switch line is bound on a mechanism with no home switch role"},
        {"the intake's piece sensor unplugged", hb::kPieceUnplugged, EP::PieceSensorUnplugged,
         "a smart-port role is unbound or not 1..21"},
        {"a piece sensor port on the arm", hb::kPieceExtra, EP::PieceSensorExtra,
         "a piece sensor port is bound on a mechanism with no distance/optical piece sensor"},
        {"the catapult's piece switch unplugged", hb::kPieceSwitchUnplugged, EP::PieceSwitchUnplugged,
         "a 3-wire role is unbound or not A..H"},
        {"a piece switch line on the arm", hb::kPieceSwitchExtra, EP::PieceSwitchExtra,
         "a piece switch line is bound on a mechanism with no switch piece sensor"},
    };
    std::set<int> hit;
    for (const Row& r : rows) {
        CAPTURE(r.what);
        CHECK(mech::specRefusal(r.entry.spec) == mech::Refusal::None);  // ONE rule broken: the spec itself is valid
        CHECK(mech::entryProblem(r.entry.spec, r.entry.wiring) == r.expected);
        CHECK(text(mech::wiringProblem(std::span<const mech::MechanismSpec>{&r.entry.spec, 1},
                                       std::span<const mech::MechanismWiring>{&r.entry.wiring, 1}, {}, 0))
              == r.sentence);
        hit.insert(static_cast<int>(r.expected));
    }
    // EVERY per-entry rule has a hand-built entry here (and so a TU): a rule appended to entryProblem
    // with no row turns this red.
    for (int e = 1; e <= static_cast<int>(EP::PieceSwitchExtra); ++e) {
        CAPTURE(e);
        CHECK(hit.count(e) == 1U);
    }
    // Every entry a DECLARATION makes passes the per-entry rules (wire() enforced them first).
    for (const mech::MechanismTable* t : {&rx::kRobotXMechanisms, &ry::kRobotYMechanisms, &r2::kRobotTwoMechanisms,
                                          &mech_fixture::robot_z::kRobotZMechanisms}) {
        for (std::size_t i = 0; i < t->count; ++i) {
            CAPTURE(t->specs[i].name);
            CHECK(mech::entryProblem(t->specs[i], t->wiring[i]) == EP::None);
        }
    }
    CHECK(mech::entryProblemText(EP::None) == nullptr);
    CHECK(text(mech::entryProblemText(static_cast<EP>(200))) == "an unknown wiring problem");
    // POSITIVE: two DIFFERENT entries with no declaration behind them are two mechanisms (they used
    // to be refused as "the same mechanism wired twice": indexOf(nullptr) matched the first).
    static constexpr mech::MechanismTable two = mech::mechanisms(hb::kArmByHand, hb::kFlywheelByHand);
    static_assert(two.count == 2);
    CHECK(two.declared[0] == nullptr);
    CHECK(two.declared[1] == nullptr);
    CHECK(two.indexOf(nullptr) == -1);
    CHECK(rx::kRobotXMechanisms.indexOf(nullptr) == -1);
}

TEST_CASE("MECH-A1 wiring: ONE BOOT LINE per role, verbatim in every state (P4's lines)") {
    CHECK(line("arm", "motor 1", 1, Line::None, mech::RoleState::Present, "", false) == "arm: motor 1 -> port 1 ok");
    CHECK(line("lift", "home switch", 0, Line::B, mech::RoleState::NotCheckable, "", false)
          == "lift: home switch -> 3-wire B not checkable (the brain cannot see a 3-wire device)");
    CHECK(line("intake", "piece sensor", 9, Line::None, mech::RoleState::Refused,
               "port 9: a rotation sensor (expected a distance sensor)", true)
          == "intake: piece sensor -> port 9 MISSING, it runs WITHOUT it: port 9: a rotation sensor (expected a distance sensor)");
    CHECK(line("wrist", "angle sensor", 10, Line::None, mech::RoleState::Refused,
               "port 10: a distance sensor (expected a rotation sensor)", false)
          == "wrist: angle sensor -> port 10 ABSENT: port 10: a distance sensor (expected a rotation sensor)");
    CHECK(line("claw pivot 6", "motor 1", 7, Line::None, mech::RoleState::Refused,
               "port 7: nothing plugged in (expected a motor)", false)
          == "claw pivot 6: motor 1 -> port 7 ABSENT: port 7: nothing plugged in (expected a motor)");
    CHECK(line("wrist", "angle sensor", 10, Line::None, mech::RoleState::NotAsked, "", false)
          == "wrist: angle sensor -> port 10 not asked (the mechanism is already ABSENT)");
    // A 3-wire role names its LINE in every state (MECH-A1: never "not checkable" for a line it
    // never asked for).
    CHECK(line("lift", "home switch", 0, Line::B, mech::RoleState::NotAsked, "", false)
          == "lift: home switch -> 3-wire B not asked (the mechanism is already ABSENT)");
    CHECK(line("hood", "line 1", 0, Line::C, mech::RoleState::Refused, "3-wire port C (line 3) has no digital out", false)
          == "hood: line 1 -> 3-wire C ABSENT: 3-wire port C (line 3) has no digital out");
    SUBCASE("bounded: truncates, terminates, never writes past its cap") {
        char small[12];
        std::memset(small, 'Z', sizeof small);
        mech::roleLine(small, 8, "a very long mechanism", "motor 1", 1, Line::None, mech::RoleState::Present, "", false);
        CHECK(std::strlen(small) == 7U);
        CHECK(small[8] == 'Z');  // byte 8 onward untouched
        char none[4] = {'Q', 'Q', 'Q', 'Q'};
        mech::roleLine(none, 0, "arm", "motor 1", 1, Line::None, mech::RoleState::Present, "", false);
        CHECK(none[0] == 'Q');  // cap 0 writes nothing
        mech::roleLine(nullptr, 64, "arm", "motor 1", 1, Line::None, mech::RoleState::Present, "", false);  // no crash
        char nulls[64];
        mech::roleLine(nulls, sizeof nulls, nullptr, nullptr, 3, Line::None, mech::RoleState::Refused, nullptr, false);
        CHECK(std::string{nulls} == ":  -> port 3 ABSENT: ");
    }
    CHECK(mech::lineLetter(Line::None) == '-');
    CHECK(mech::lineLetter(Line::A) == 'A');
    CHECK(mech::lineLetter(Line::H) == 'H');
}
