// Adversarial tests for the chassis table's consistency checks (chunk R3b Part 0b, brief §4
// test 4): a signed table must have every entry a real port, no duplicates by absolute value
// on a side or across sides, both sides non-empty, the IMU port not a drive port; an unsigned
// table must carry no sign. A half-signed table is refused.
//
// The table lives in src/ (it is a per-robot composition fact, not library API), and it is
// PROS-free on purpose so this test can include it from src/ and break its checks
// with hand-built tables. The mutation the brief requires ("remove a check") was run against
// these cases and the observed red is in the R3b log.
//
// WHAT THIS TU SEES (since G1 Part 2c's final run, 2026-09-26): EVERY robot's REAL table, by its
// own name — kBenchBotChassis, kRobotTwoChassis, kNoChassis (the invented X-drive's, all UNSET) —
// and the selection chassisTableFor(), which the cases below call for every robot. kChassis, the
// build's selection, is the host's: the host sets no build switch, so src/build_selection.hpp's
// chain takes its #else and kChassis is the bench bot's, as in `make` with no ROBOT. Until that run
// robot two's table existed only under the tank define, so this file RE-TYPED its entries and a
// typo in the real table was invisible to the suite; the re-typed copy (robotTwo(), below) stays as
// the table the break-it cases mutate, and a case now pins it — and the real table — to the brief's
// literals, field by field.

#include "doctest.h"

#include <cstring>
#include <initializer_list>
#include <limits>
#include <string>
#include <string_view>

#include "robots/chassis_table.hpp"

using shulib::bench::absPort;
using shulib::bench::Cartridge;
using shulib::bench::chassisTableFor;
using shulib::bench::ChassisTable;
using shulib::bench::kBenchBotChassis;
using shulib::bench::kChassis;
using shulib::bench::kMaxPort;
using shulib::bench::kNoChassis;
using shulib::bench::kRobotTwoChassis;
using shulib::bench::RobotId;
using shulib::bench::portsToString;
using shulib::bench::tableConsistent;
using shulib::bench::tableCount;
using shulib::bench::tableHasPorts;
using shulib::bench::tableSideOf;
using shulib::bench::tableSignedPort;
using shulib::bench::tableSignOf;

namespace {

/// Robot two's table exactly as measured (brief §0): the signed ports, IMU 2, blue — typed by hand
/// with its own name and provenance, so the break-it cases can see their mutation in a message. The
/// case "robot two's REAL table" pins it to kRobotTwoChassis field by field.
[[nodiscard]] ChassisTable robotTwo() {
    return ChassisTable{
        .robot = "test copy of robot two",
        .left = {-11, 12, -13, 14, -15},
        .leftCount = 5,
        .right = {20, -19, 18, -17, 16},
        .rightCount = 5,
        .cartridge = Cartridge::Blue,
        .imuPort = 2,
        .measured = true,
        .signsMeasured = true,
        .provenance = "test",
        .wheelDiameterIn = 2.75,  // as committed: reported, the rest UNSET (R3b Parts 1-3)
        .trackWidthIn = 0.0,
        .externalRatio = 0.0,
        .geometryProvenance = "test",
    };
}

[[nodiscard]] bool consistent(const ChassisTable& t, char* why, std::size_t cap) {
    return tableConsistent(t, why, cap);
}

}  // namespace

TEST_CASE("chassis table: robot two's measured table is consistent, and the signs come back "
          "AS TYPED") {
    // BUG CAUGHT: the drive program handing ProsMotor the wrong sign — the whole reason the
    // signs moved into the table. Each measured value is a literal from the brief.
    const ChassisTable t = robotTwo();
    char why[96];
    CHECK(consistent(t, why, sizeof why));
    CHECK(why[0] == '\0');
    CHECK(tableHasPorts(t));
    CHECK(tableCount(t) == 10);
    CHECK(tableSignedPort(t, 11) == -11);
    CHECK(tableSignedPort(t, 12) == 12);
    CHECK(tableSignedPort(t, 13) == -13);
    CHECK(tableSignedPort(t, 14) == 14);
    CHECK(tableSignedPort(t, 15) == -15);
    CHECK(tableSignedPort(t, 20) == 20);
    CHECK(tableSignedPort(t, 19) == -19);
    CHECK(tableSignedPort(t, 18) == 18);
    CHECK(tableSignedPort(t, 17) == -17);
    CHECK(tableSignedPort(t, 16) == 16);
    CHECK(tableSignOf(t, 11) == -1);
    CHECK(tableSignOf(t, 12) == +1);
    CHECK(tableSignOf(t, 19) == -1);
    CHECK(tableSignOf(t, 20) == +1);
    // Sides by |port|: a signed entry and a physical port find each other.
    for (int p : {11, 12, 13, 14, 15}) CHECK(tableSideOf(t, p) == -1);
    for (int p : {16, 17, 18, 19, 20}) CHECK(tableSideOf(t, p) == +1);
    // Not in the table: the IMU, the radio, an empty port.
    CHECK(tableSideOf(t, 2) == 0);
    CHECK(tableSignedPort(t, 2) == 0);
    CHECK(tableSignOf(t, 1) == 0);
    CHECK(tableSideOf(t, 21) == 0);
    // The printed form carries the signs.
    char buf[80];
    portsToString(t.left, t.leftCount, true, buf, sizeof buf);
    CHECK(std::strcmp(buf, "-11 +12 -13 +14 -15") == 0);
    portsToString(t.right, t.rightCount, true, buf, sizeof buf);
    CHECK(std::strcmp(buf, "+20 -19 +18 -17 +16") == 0);
}

TEST_CASE("chassis table: the bench bot's table (the host's selection, as in `make` with no "
          "ROBOT) is consistent and UNSIGNED") {
    // BUG CAUGHT: the real table drifting into an inconsistent state, or growing signs it
    // never measured. On the host no build switch is set, so kChassis is the bench bot's.
    REQUIRE(&kChassis == &kBenchBotChassis);
    char why[96];
    CHECK(consistent(kChassis, why, sizeof why));
    CHECK_FALSE(kChassis.signsMeasured);
    CHECK(kChassis.leftCount == 4);
    CHECK(kChassis.rightCount == 4);
    CHECK(kChassis.imuPort == 4);
    for (std::size_t i = 0; i < kChassis.leftCount; ++i) CHECK(kChassis.left[i] > 0);
    for (std::size_t i = 0; i < kChassis.rightCount; ++i) CHECK(kChassis.right[i] > 0);
    CHECK(tableSignOf(kChassis, 15) == 0);  // unsigned table: no sign, ever
    CHECK(tableSignedPort(kChassis, 15) == 15);
    char buf[80];
    portsToString(kChassis.left, kChassis.leftCount, false, buf, sizeof buf);
    CHECK(std::strcmp(buf, "15 16 17 18") == 0);
}

TEST_CASE("chassis table: a HALF-SIGNED table is refused — a '-' with signsMeasured false") {
    // BUG CAUGHT: someone types one measured sign into the bench table without flipping the
    // flag; the other entries' '+' would then be indistinguishable from "never measured",
    // and a drive program would run on a table that is half a measurement.
    ChassisTable t = robotTwo();
    t.signsMeasured = false;
    char why[96];
    CHECK_FALSE(consistent(t, why, sizeof why));
    CHECK(std::strstr(why, "half-signed") != nullptr);
    CHECK(std::strstr(why, "-11") != nullptr);
    // The same table with every sign stripped IS a consistent unsigned table.
    for (std::size_t i = 0; i < t.leftCount; ++i) t.left[i] = static_cast<std::int8_t>(absPort(t.left[i]));
    for (std::size_t i = 0; i < t.rightCount; ++i) t.right[i] = static_cast<std::int8_t>(absPort(t.right[i]));
    CHECK(consistent(t, why, sizeof why));
}

TEST_CASE("chassis table: a port on BOTH sides is refused, judged by |port| — including "
          "+13 left and -13 right") {
    // BUG CAUGHT: the check the old table code carried was DEAD (it asked tableSideOf() of a
    // left port and could only ever get LEFT back), so a port typed on both sides sailed
    // through. A signed table makes it worse: +13 on one side and -13 on the other reads
    // as two different numbers to a naive comparison.
    ChassisTable t = robotTwo();
    t.right[2] = -13;  // 18 -> the left side's 13, opposite sign
    char why[96];
    CHECK_FALSE(consistent(t, why, sizeof why));
    CHECK(std::strstr(why, "BOTH sides") != nullptr);
    CHECK(std::strstr(why, "13") != nullptr);
    ChassisTable same = robotTwo();
    same.right[0] = -11;  // 20 -> 11 with the same sign as left's -11
    CHECK_FALSE(consistent(same, why, sizeof why));
    CHECK(std::strstr(why, "BOTH sides") != nullptr);
}

TEST_CASE("chassis table: a port listed twice on ONE side is refused, judged by |port|") {
    // BUG CAUGHT: a typo duplicating an entry (and thereby dropping the port it replaced);
    // with signs, `-12` and `+12` on the same side is the same typo.
    ChassisTable t = robotTwo();
    t.left[3] = -12;  // 14 -> 12 again, opposite sign
    char why[96];
    CHECK_FALSE(consistent(t, why, sizeof why));
    CHECK(std::strstr(why, "LEFT lists port 12 twice") != nullptr);
    ChassisTable r = robotTwo();
    r.right[4] = 20;  // 16 -> 20 again
    CHECK_FALSE(consistent(r, why, sizeof why));
    CHECK(std::strstr(why, "RIGHT lists port 20 twice") != nullptr);
}

TEST_CASE("chassis table: every entry must be a real port — 0, 22, -22 are refused") {
    // BUG CAUGHT: an entry of 0 (the "no port" value) counted as a drive port, or a port
    // beyond the brain's 21.
    char why[96];
    ChassisTable zero = robotTwo();
    zero.left[0] = 0;
    CHECK_FALSE(consistent(zero, why, sizeof why));
    CHECK(std::strstr(why, "not a port in 1..21") != nullptr);
    ChassisTable big = robotTwo();
    big.right[1] = 22;
    CHECK_FALSE(consistent(big, why, sizeof why));
    CHECK(std::strstr(why, "not a port in 1..21") != nullptr);
    ChassisTable negBig = robotTwo();
    negBig.right[1] = -22;
    CHECK_FALSE(consistent(negBig, why, sizeof why));
    // 21 and -21 are fine.
    ChassisTable edge = robotTwo();
    edge.right[1] = -21;
    CHECK(consistent(edge, why, sizeof why));
    CHECK(absPort(-21) == 21);
    CHECK(kMaxPort == 21);
}

TEST_CASE("chassis table: the IMU port may not be a drive port, on either side, either sign") {
    // BUG CAUGHT: the IMU typed onto a drive port (the adapter would then try to configure
    // an IMU as a motor, and the census's side label would lie).
    char why[96];
    ChassisTable l = robotTwo();
    l.imuPort = 13;  // the left side's -13
    CHECK_FALSE(consistent(l, why, sizeof why));
    CHECK(std::strstr(why, "IMU") != nullptr);
    ChassisTable r = robotTwo();
    r.imuPort = 18;
    CHECK_FALSE(consistent(r, why, sizeof why));
    CHECK(std::strstr(why, "IMU") != nullptr);
    // An UNSET IMU (0) never collides.
    ChassisTable none = robotTwo();
    none.imuPort = 0;
    CHECK(consistent(none, why, sizeof why));
}

TEST_CASE("chassis table: a SIGNED table must have BOTH sides non-empty") {
    // BUG CAUGHT: signs claimed for a side that has no ports — a claim about nothing, which a
    // drive program would read as "the left side is measured and empty" and drive one-sided.
    char why[96];
    ChassisTable t = robotTwo();
    t.rightCount = 0;
    CHECK_FALSE(consistent(t, why, sizeof why));
    CHECK(std::strstr(why, "no ports") != nullptr);
    ChassisTable u = robotTwo();
    u.leftCount = 0;
    CHECK_FALSE(consistent(u, why, sizeof why));
    // Whereas an UNSIGNED, UNSET table (both sides empty) is merely unset, not contradictory
    // — that is describeMissing()'s finding, not this one's.
    ChassisTable unset{};
    unset.robot = "unset";
    unset.provenance = "";
    CHECK(consistent(unset, why, sizeof why));
    CHECK_FALSE(tableHasPorts(unset));
}

// ═══ Test 18 (R3b Parts 1–3): the table's GEOMETRY — UNSET refuses; set values reach the
// kinematics and the odometry scale unchanged (bit-identical to a hand-built rig) ═══════
// Bug caught: a guessed track width or ratio driving the robot wrong in silence (the library
// graph must refuse while either is UNSET), or a helper that swaps two same-typed inches
// (wheel diameter for track width) between the table and its consumers.
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

TEST_CASE("chassis table 18: geometry UNSET refuses the library graph, naming what is missing") {
    using shulib::bench::describeMissingForLibrary;
    using shulib::bench::tableGeometrySet;
    char why[160];
    // Robot two as committed today: wheel reported, track width and ratio UNSET.
    ChassisTable t = robotTwo();
    t.wheelDiameterIn = 2.75;
    t.trackWidthIn = 0.0;
    t.externalRatio = 0.0;
    CHECK_FALSE(tableGeometrySet(t));
    CHECK(describeMissingForLibrary(t, why, sizeof why));
    CHECK(std::strstr(why, "track width") != nullptr);
    CHECK(std::strstr(why, "external ratio") != nullptr);
    CHECK(std::strstr(why, "wheel diameter") == nullptr);  // that one IS set
    CHECK(std::strstr(why, "IMU") == nullptr);             // and so is the IMU
    CHECK(consistent(t, why, sizeof why));                 // UNSET is legal; only the library refuses
    // Each number alone still refuses.
    t.trackWidthIn = 13.0;
    CHECK(describeMissingForLibrary(t, why, sizeof why));
    CHECK(std::strstr(why, "external ratio") != nullptr);
    CHECK(std::strstr(why, "track width") == nullptr);
    t.externalRatio = 1.0;
    CHECK_FALSE(describeMissingForLibrary(t, why, sizeof why));
    CHECK(tableGeometrySet(t));
    CHECK(why[0] == '\0');
    // The IMU is part of what the library needs (the tester's forDrive list excludes it).
    t.imuPort = 0;
    CHECK(describeMissingForLibrary(t, why, sizeof why));
    CHECK(std::strstr(why, "IMU port") != nullptr);
    // A negative or non-finite geometry number is a CONTRADICTION, refused by tableConsistent.
    ChassisTable bad = robotTwo();
    bad.trackWidthIn = -13.0;
    CHECK_FALSE(consistent(bad, why, sizeof why));
    CHECK(std::strstr(why, "track width") != nullptr);
    bad = robotTwo();
    bad.externalRatio = -1.0;
    CHECK_FALSE(consistent(bad, why, sizeof why));
    bad = robotTwo();
    bad.wheelDiameterIn = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(consistent(bad, why, sizeof why));
    // The committed table (the host's selection, the bench bot's) has legal geometry fields.
    CHECK(consistent(kChassis, why, sizeof why));
    CHECK(kChassis.wheelDiameterIn == 2.75);
    CHECK(kChassis.trackWidthIn == 0.0);   // UNSET, on purpose: never guessed
    CHECK(kChassis.externalRatio == 0.0);  // UNSET, on purpose: never guessed
    CHECK(std::strlen(kChassis.geometryProvenance) > 10);
}

TEST_CASE("chassis table 18: set geometry reaches TankKinematics and the odometry scale "
          "bit-identically to a hand-built rig") {
    using shulib::bench::tableDriveGeometry;
    using shulib::bench::tableTrackWidth;
    using shulib::hal::DriveGeometry;
    using shulib::kinematics::TankKinematics;
    using shulib::math::ChassisSpeeds;
    using shulib::units::AngularVelocity;
    using shulib::units::Length;
    using shulib::units::Velocity;
    ChassisTable t = robotTwo();
    t.wheelDiameterIn = 2.75;
    t.trackWidthIn = 13.25;   // a measured-looking, non-round value so a swap cannot hide
    t.externalRatio = 0.6;
    // Kinematics: the table's track width, bit for bit, on every wheel output.
    const TankKinematics fromTable{tableTrackWidth(t)};
    const TankKinematics byHand{Length{13.25}};
    const ChassisSpeeds cmds[] = {
        {Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}},
        {Velocity{15.0}, Velocity{0.0}, AngularVelocity{1.2}},
        {Velocity{0.0}, Velocity{0.0}, AngularVelocity{-1.5}},
        {Velocity{-18.5}, Velocity{3.0}, AngularVelocity{0.7}},
    };
    for (const ChassisSpeeds& c : cmds) {
        const auto a = fromTable.toWheels(c);
        const auto b = byHand.toWheels(c);
        REQUIRE(a.size() == 2);
        CHECK(a[0].value() == b[0].value());
        CHECK(a[1].value() == b[1].value());
    }
    // Odometry / stall-check scale: the table's geometry, bit for bit.
    const DriveGeometry g = tableDriveGeometry(t);
    const DriveGeometry h = DriveGeometry::fromDiameter(Length{2.75}, 0.6);
    CHECK(g.wheelRadius.value() == h.wheelRadius.value());
    CHECK(g.motorToWheelRatio == h.motorToWheelRatio);
    CHECK(g.inchesPerRadian().value() == h.inchesPerRadian().value());
    CHECK(g.inchesPerRadian().value() == 1.375 * 0.6);
    CHECK(g.valid());
    // A swap of the two inch values would be visible on every line above: the track width
    // is not the wheel diameter, and the helpers must never confuse them.
    CHECK(tableTrackWidth(t).value() == 13.25);
    CHECK(tableTrackWidth(t).value() != 2.0 * g.wheelRadius.value());
}

// ═══ Test 2 (R3d Part 1): the table is the SECOND SOURCE of the ONE representation ═══════
// Bug caught: the typed fallback and the robot's own calibration disagreeing about what a
// drivetrain IS — a second shape for the same facts, which is how two programs come to believe
// two different robots. And, more sharply: a sign dropped on the way across (every '-' entry
// silently made positive would fight five motors against four on the first command), or the
// wheel DIAMETER used where the radius belongs (every distance doubled, silently).
#include "shulib/hal/drivetrain_description.hpp"

TEST_CASE("chassis table 2: tableToDescription carries robot two's table into the one "
          "representation, signs and scale bit for bit") {
    using shulib::bench::describeMissingForDescription;
    using shulib::bench::tableDriveGeometry;
    using shulib::bench::tableToDescription;
    using shulib::hal::DescriptionSource;
    using shulib::hal::DrivetrainDescription;
    using shulib::hal::DrivetrainKind;
    using shulib::hal::MotorCartridge;
    using shulib::units::Length;

    ChassisTable t = robotTwo();
    t.wheelDiameterIn = 2.75;
    t.trackWidthIn = 13.25;  // non-round, so a swap with the wheel cannot hide
    t.externalRatio = 0.6;
    t.geometryProvenance = "tape + tooth counts, a host test";

    char missing[192];
    CHECK_FALSE(describeMissingForDescription(t, missing, sizeof missing));
    const DrivetrainDescription d = tableToDescription(t);
    char why[160];
    REQUIRE(d.valid(why, sizeof why));
    CHECK(why[0] == '\0');

    // The SIGNS, as typed, entry by entry — the one place a sign lives, carried not re-derived.
    REQUIRE(d.left.count == 5);
    REQUIRE(d.right.count == 5);
    const std::int8_t expectedLeft[5] = {-11, 12, -13, 14, -15};
    const std::int8_t expectedRight[5] = {20, -19, 18, -17, 16};
    for (std::size_t i = 0; i < 5; ++i) {
        CHECK(d.left.signedPorts[i] == expectedLeft[i]);
        CHECK(d.right.signedPorts[i] == expectedRight[i]);
    }
    CHECK(d.signedPortOf(11) == -11);
    CHECK(d.signedPortOf(20) == 20);
    CHECK(d.sideOf(15) == -1);
    CHECK(d.sideOf(16) == +1);
    // Ports beyond the count were never written: a description is writable to a card as it is.
    for (std::size_t i = 5; i < shulib::hal::kMaxSmartPorts; ++i) {
        CHECK(d.left.signedPorts[i] == 0);
        CHECK(d.right.signedPorts[i] == 0);
    }

    // The SCALE is the RADIUS x ratio, never the diameter: 2.75/2 x 0.6, bit for bit, on both
    // sides, and identical to what the odometry already gets from tableDriveGeometry().
    CHECK(d.left.inchesPerRadian.value() == 1.375 * 0.6);
    CHECK(d.right.inchesPerRadian.value() == 1.375 * 0.6);
    CHECK(d.left.inchesPerRadian.value() == tableDriveGeometry(t).inchesPerRadian().value());
    CHECK(d.leftGeometry().inchesPerRadian().value()
          == tableDriveGeometry(t).inchesPerRadian().value());
    CHECK(d.left.inchesPerRadian.value() != 2.75 * 0.6);  // the diameter, if anyone swapped it
    CHECK(d.trackWidth.value() == 13.25);

    // The stated facts and the provenance.
    CHECK(d.kind == DrivetrainKind::Tank);
    CHECK(d.cartridge == MotorCartridge::Blue);
    CHECK(d.imuPort == 2);
    CHECK(d.source == DescriptionSource::TypedTable);  // NOT RobotMeasured: a typed number
    CHECK(std::strstr(d.provenance, "TYPED TABLE") != nullptr);
    CHECK(std::strstr(d.provenance, "tape + tooth counts") != nullptr);
    CHECK(std::strstr(d.provenance, "test copy of robot two") != nullptr);
}

TEST_CASE("chassis table 2: an UNSET or UNSIGNED table cannot produce a VALID description, and "
          "names what is missing") {
    using shulib::bench::describeMissingForDescription;
    using shulib::bench::tableToDescription;
    using shulib::hal::DrivetrainDescription;
    using shulib::hal::MotorCartridge;

    char missing[192];
    char why[160];

    // (a) the table as COMMITTED for robot two: geometry UNSET. Refused, naming the numbers.
    ChassisTable unsetGeometry = robotTwo();  // wheel 2.75, width 0, ratio 0
    CHECK(describeMissingForDescription(unsetGeometry, missing, sizeof missing));
    CHECK(std::strstr(missing, "track width") != nullptr);
    CHECK(std::strstr(missing, "external ratio") != nullptr);
    const DrivetrainDescription a = tableToDescription(unsetGeometry);
    CHECK_FALSE(a.valid(why, sizeof why));
    CHECK(a.left.inchesPerRadian.value() == 0.0);  // never invented from the wheel alone
    CHECK(a.trackWidth.value() == 0.0);
    CHECK(std::strstr(why, "inches per radian") != nullptr);

    // (b) an UNSIGNED table — the dangerous one. Every entry is positive and a naive mapping
    // would produce a description that passes every check in valid() while half the motors
    // fought their side-mates on the first command. Both sides are left EMPTY instead.
    ChassisTable unsigned_ = robotTwo();
    unsigned_.wheelDiameterIn = 2.75;
    unsigned_.trackWidthIn = 13.25;
    unsigned_.externalRatio = 1.0;
    for (std::size_t i = 0; i < unsigned_.leftCount; ++i) {
        unsigned_.left[i] = static_cast<std::int8_t>(absPort(unsigned_.left[i]));
    }
    for (std::size_t i = 0; i < unsigned_.rightCount; ++i) {
        unsigned_.right[i] = static_cast<std::int8_t>(absPort(unsigned_.right[i]));
    }
    unsigned_.signsMeasured = false;
    CHECK(consistent(unsigned_, why, sizeof why));  // it is a legal table …
    CHECK(describeMissingForDescription(unsigned_, missing, sizeof missing));  // … not a
    CHECK(std::strstr(missing, "measured signs") != nullptr);                  //   describable one
    const DrivetrainDescription b = tableToDescription(unsigned_);
    CHECK_FALSE(b.valid(why, sizeof why));
    CHECK(b.left.count == 0);
    CHECK(b.right.count == 0);
    CHECK(std::strstr(why, "no ports") != nullptr);

    // (c) an UNSET cartridge leaves a byte outside the enum, and an UNSET IMU leaves port 0 —
    // each refused on its own, neither invented.
    ChassisTable noCartridge = robotTwo();
    noCartridge.wheelDiameterIn = 2.75;
    noCartridge.trackWidthIn = 13.25;
    noCartridge.externalRatio = 1.0;
    noCartridge.cartridge = Cartridge::Unset;
    CHECK(describeMissingForDescription(noCartridge, missing, sizeof missing));
    CHECK(std::strstr(missing, "cartridge") != nullptr);
    const DrivetrainDescription c = tableToDescription(noCartridge);
    CHECK(c.cartridge == static_cast<MotorCartridge>(0));
    CHECK_FALSE(c.valid(why, sizeof why));
    CHECK(std::strstr(why, "cartridge") != nullptr);

    ChassisTable noImu = robotTwo();
    noImu.wheelDiameterIn = 2.75;
    noImu.trackWidthIn = 13.25;
    noImu.externalRatio = 1.0;
    noImu.imuPort = 0;
    CHECK(describeMissingForDescription(noImu, missing, sizeof missing));
    CHECK(std::strstr(missing, "IMU port") != nullptr);
    CHECK_FALSE(tableToDescription(noImu).valid(why, sizeof why));
    CHECK(std::strstr(why, "IMU port") != nullptr);

    // (d) the host's selection — the bench bot's table — is UNSIGNED and geometry-less, so it too
    // refuses. Nothing in the tree can build a drivetrain from it today, by design.
    CHECK(describeMissingForDescription(kChassis, missing, sizeof missing));
    CHECK_FALSE(tableToDescription(kChassis).valid(why, sizeof why));
}

TEST_CASE("chassis table 3: tableToDescription()'s provenance is byte for byte the snprintf it "
          "replaced -- whole when it fits, cut ONE short of the 96-byte buffer when not") {
    // G1 Part 2b replaced `snprintf(d.provenance, 96, "TYPED TABLE (%s): %s", robot, geometry)`
    // with a hand-bounded copy (finding F-2b-1: -Wformat-truncation). Its comment promises the
    // same bytes; this pins them at every length that matters, INCLUDING the cut — the only path
    // the ROBOT=tank build's real table takes (its provenance is far longer than 95 bytes). The
    // expected text is built here with std::string, never by the code under test.
    using shulib::bench::tableToDescription;
    constexpr std::size_t kCap = sizeof(shulib::hal::DrivetrainDescription::provenance);
    static_assert(kCap == 96, "the description's provenance buffer changed size");
    const auto expected = [](const ChassisTable& t) {
        const std::string whole =
            std::string{"TYPED TABLE ("} + t.robot + "): " + t.geometryProvenance;
        return whole.substr(0, kCap - 1);  // what snprintf wrote: cut one short, terminated
    };
    ChassisTable t = robotTwo();
    const std::string prefix = std::string{"TYPED TABLE ("} + t.robot + "): ";
    REQUIRE(prefix.size() < kCap - 2);

    // (a) short: whole.
    t.geometryProvenance = "tape + tooth counts, a host test";
    CHECK(std::string_view{tableToDescription(t).provenance}
          == "TYPED TABLE (test copy of robot two): tape + tooth counts, a host test");
    // (b) exactly 95 bytes: whole, the NUL in the buffer's last byte.
    const std::string fits(kCap - 1 - prefix.size(), 'x');
    t.geometryProvenance = fits.c_str();
    {
        const shulib::hal::DrivetrainDescription d = tableToDescription(t);
        CHECK(std::strlen(d.provenance) == kCap - 1);
        CHECK(std::string_view{d.provenance} == prefix + fits);
    }
    // (c) 96 bytes: cut by exactly one — the boundary an off-by-one would write past.
    const std::string oneOver(kCap - prefix.size(), 'y');
    t.geometryProvenance = oneOver.c_str();
    {
        const shulib::hal::DrivetrainDescription d = tableToDescription(t);
        CHECK(std::strlen(d.provenance) == kCap - 1);
        CHECK(std::string_view{d.provenance} == expected(t));
        CHECK(d.provenance[kCap - 1] == '\0');
    }
    // (d) the host's selection (the bench bot's table), whose provenance is far longer.
    {
        const shulib::hal::DrivetrainDescription d = tableToDescription(kChassis);
        CHECK(std::strlen(d.provenance) == kCap - 1);
        CHECK(std::string_view{d.provenance} == expected(kChassis));
    }
}

// ═══ G1 Part 2c's final run: every robot's REAL table on the host, and the selection ═══════════
// The team lead's ruling "one chain picks both" (2026-09-26): src/build_selection.hpp's one chain
// sets kRobotId, and this table has no #if of its own — chassisTableFor(kRobotId) is the build's table.
// So the tables the robot builds carry are compiled HERE, and the mapping each robot build uses is
// a function the host can call for every robot.

TEST_CASE("chassis table: the selection -- chassisTableFor() gives every robot its OWN table, and "
          "the host's kChassis is the bench build's") {
    // BUG CAUGHT: a wrong robot in chassisTableFor() (robot two handed the bench bot's unsigned
    // table: every tank program refuses, or worse, a table with the right shape and the wrong
    // ports), or two robots sharing one table. The robot builds evaluate exactly these calls.
    CHECK(&chassisTableFor(RobotId::BenchBot) == &kBenchBotChassis);
    CHECK(&chassisTableFor(RobotId::RobotTwo) == &kRobotTwoChassis);
    CHECK(&chassisTableFor(RobotId::InventedXDrive) == &kNoChassis);
    CHECK(&kBenchBotChassis != &kRobotTwoChassis);
    CHECK(&kRobotTwoChassis != &kNoChassis);
    CHECK(&kNoChassis != &kBenchBotChassis);
    // Each table names its own robot, so a banner can never print one robot over another's ports.
    CHECK(std::string_view{chassisTableFor(RobotId::BenchBot).robot}.find("BENCH BOT")
          != std::string_view::npos);
    CHECK(std::string_view{chassisTableFor(RobotId::RobotTwo).robot}.find("robot two")
          != std::string_view::npos);
    CHECK(std::string_view{chassisTableFor(RobotId::InventedXDrive).robot}.find("INVENTED X-DRIVE")
          != std::string_view::npos);
    // The host (no build switch) is the bench build.
    CHECK(shulib::bench::kRobotId == RobotId::BenchBot);
    CHECK(&kChassis == &chassisTableFor(shulib::bench::kRobotId));
    CHECK(&kChassis == &kBenchBotChassis);
}

TEST_CASE("chassis table: robot two's REAL table (kRobotTwoChassis) is the measured one, field by "
          "field -- and the hand-typed copy the other cases break matches it") {
    // BUG CAUGHT: a typo in the table robot two's four builds carry — a dropped sign, a swapped
    // side, the IMU moved — which, until G1 Part 2c's final run, no host test could see (the table
    // existed only under the tank define, and this file tested a re-typed copy). Every literal below
    // is from the brief's §0 and the table's own provenance, never read from the header under test.
    const ChassisTable& t = kRobotTwoChassis;
    const std::int8_t left[5] = {-11, 12, -13, 14, -15};
    const std::int8_t right[5] = {20, -19, 18, -17, 16};
    REQUIRE(t.leftCount == 5);
    REQUIRE(t.rightCount == 5);
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(t.left[i] == left[i]);
        CHECK(t.right[i] == right[i]);
    }
    for (std::size_t i = 5; i < static_cast<std::size_t>(kMaxPort); ++i) {
        CHECK(t.left[i] == 0);
        CHECK(t.right[i] == 0);
    }
    CHECK(t.cartridge == Cartridge::Blue);
    CHECK(t.imuPort == 2);
    CHECK(t.measured);
    CHECK(t.signsMeasured);
    CHECK(t.wheelDiameterIn == 2.75);
    CHECK(t.trackWidthIn == 0.0);   // UNSET: tape owed, never guessed
    CHECK(t.externalRatio == 0.0);  // UNSET: tooth counts owed, never guessed
    char why[96];
    CHECK(consistent(t, why, sizeof why));
    // The two strings test/boot_source_test.cpp re-types as "what the ROBOT=tank build prints":
    // the robot's name and what its table cannot describe.
    CHECK(std::string_view{t.robot} == "2026 TANK CHASSIS (robot two)");
    char missing[192];
    CHECK(shulib::bench::describeMissingForDescription(t, missing, sizeof missing));
    CHECK(std::string_view{missing} == "track width, external ratio (motor->wheel)");
    // The hand-typed copy the break-it cases start from is the real table in every field a check
    // reads (its name and provenance are its own, on purpose).
    const ChassisTable copy = robotTwo();
    CHECK(copy.leftCount == t.leftCount);
    CHECK(copy.rightCount == t.rightCount);
    for (std::size_t i = 0; i < static_cast<std::size_t>(kMaxPort); ++i) {
        CAPTURE(i);
        CHECK(copy.left[i] == t.left[i]);
        CHECK(copy.right[i] == t.right[i]);
    }
    CHECK(copy.cartridge == t.cartridge);
    CHECK(copy.imuPort == t.imuPort);
    CHECK(copy.measured == t.measured);
    CHECK(copy.signsMeasured == t.signsMeasured);
    CHECK(copy.wheelDiameterIn == t.wheelDiameterIn);
    CHECK(copy.trackWidthIn == t.trackWidthIn);
    CHECK(copy.externalRatio == t.externalRatio);
}

TEST_CASE("chassis table: the invented X-drive's table is UNSET in every field -- consistent, and "
          "refused by everything that would drive from it") {
    // BUG CAUGHT: the xdrive build carrying ANOTHER robot's facts — until G1 Part 2c's final run it
    // carried the bench bot's table (the #else of this file's old #if), dormant only because no
    // program in that build reads a chassis table. An UNSET table makes any future reader refuse by
    // name instead of driving the bench bot's ports on an X-drive.
    const ChassisTable& t = kNoChassis;
    char why[96];
    CHECK(consistent(t, why, sizeof why));  // unset is legal, never contradictory
    CHECK_FALSE(tableHasPorts(t));
    CHECK(tableCount(t) == 0);
    CHECK(t.cartridge == Cartridge::Unset);
    CHECK(t.imuPort == 0);
    CHECK_FALSE(t.measured);
    CHECK_FALSE(t.signsMeasured);
    CHECK(t.wheelDiameterIn == 0.0);
    CHECK(t.trackWidthIn == 0.0);
    CHECK(t.externalRatio == 0.0);
    for (int p = 1; p <= kMaxPort; ++p) {
        CAPTURE(p);
        CHECK(tableSideOf(t, p) == 0);
        CHECK(tableSignedPort(t, p) == 0);
    }
    char missing[192];
    CHECK(shulib::bench::describeMissing(t, missing, sizeof missing, true));  // the drive refusal
    CHECK(std::string_view{missing} == "LEFT ports, RIGHT ports, cartridge");
    CHECK(shulib::bench::describeMissingForDescription(t, missing, sizeof missing));
    CHECK_FALSE(shulib::bench::tableToDescription(t).valid(why, sizeof why));
}
