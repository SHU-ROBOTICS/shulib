#pragma once
//
// THE CHASSIS TABLE — the ONE typed input about a drivetrain (R3b Session 2 brief §3.1;
// moved out of src/programs/bench_r3a.cpp at R3b Part 0b so the bench tester and the drive program
// shared ONE definition — two copies of a port table is how two programs come to disagree
// about which robot they are on). Since T4 Part 3 its readers are the bench tester and the boot
// source's fallback (src/common/boot_source.hpp, through tableToDescription()): the old drive
// program that built bare adapters from it was replaced by one that boots from the record.
//
// Per robot: which ports drive which side, WITH THEIR MEASURED SIGNS when those have been
// measured, the cartridge, the IMU port, and whether any of it was MEASURED. Every robot's table
// is defined in EVERY build under its own name (kBenchBotChassis, kRobotTwoChassis, kNoChassis),
// and the build's table, kChassis — the only name a program uses — is chassisTableFor(kRobotId):
// kRobotId is set by src/build_selection.hpp's ONE chain, in the same branch as the program and the
// beacon, so `make ROBOT=tank` and this table cannot disagree about which robot it is. A program
// that names a per-robot table or chassisTableFor() outside a static_assert FAILS the src build
// gate's step 0 (naming another robot's table is choosing a robot outside the chain, and nothing
// in the package would show it); the host tests under test/ read them by name. THIS FILE
// HAS NO #if OF ITS OWN (G1 Part 2c's final run, 2026-09-26, the team lead's ruling "one chain
// picks both"): it selected robot two's table on its own `#if defined(SHULIB_ROBOT_TANK_2026)`
// until then, and a break there gave a tank build the bench bot's table with every check green
// (measured at G1 Part 2c). The src build gate's step 0 fails any build switch named here.
//
// RULE: A VALUE IS NEVER INVENTED. An unset field is 0 / Unset, describeMissing() names it on
// screen, and every station or program that needs it refuses until it is filled. HA-111's
// invented port map was exactly this defect class -- every adapter constructor threw at boot
// -- and it is not repeated for robot two. When a fact arrives, THIS is the one place to
// edit, with the date and the source beside the numbers.
//
// SIGNS LIVE HERE AND NOWHERE ELSE (Part 0b, brief §2 and landmine 1): when `signsMeasured`
// is true, every port entry carries its sign -- `-11` is port 11 constructed with a NEGATIVE
// port number, which PROS reverses exactly once, in the adapter. The tester's DRIVE station (and
// the old drive program, until T4 Part 3) hands the entry to ProsMotor AS TYPED and never negates,
// and tableToDescription() carries it into a description unchanged; the tester's MOTOR WATCH cross-checks a push
// against these signs (AGREES / DISAGREES per port); its DRIVE station uses them and refuses
// if a fresh capture disagrees. A table that says `signsMeasured = false` carries POSITIVE
// entries only -- a '-' typed without flipping the flag is a half-signed table, and
// tableConsistent() refuses it. R3d makes the signs discovered and persisted; tonight they
// are typed, measured values with a date.
//
// PROS-FREE ON PURPOSE: this header is plain data and pure helpers over it, so the host test
// (test/chassis_table_test.cpp) can include it, break the consistency checks, and read EVERY
// robot's real table — robot two's included, which the host could not see while it was defined
// only under the tank define. It lives in src/, not include/shulib/, because it is a per-robot
// composition fact, not library API.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "build_selection.hpp"
#include "robots/robot_one_facts.hpp"
#include "robots/robot_two_facts.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// PHYSICAL smart ports are 1..21 -- how humans and every PROS API but the registry name them.
/// CITES hal::kMaxSmartPorts rather than repeating the number (R3d Part 1): the library owns the
/// brain's port count, and this table is one of its consumers.
constexpr int kMaxPort = static_cast<int>(hal::kMaxSmartPorts);

/// The cartridge in the drive motors. Unset = nobody has READ one off a motor yet.
enum class Cartridge { Unset, Red, Green, Blue };

/// One robot's drivetrain facts. Aggregate, so a host test can build a bad one by hand.
struct ChassisTable {
    const char* robot;            // the banner's name for this robot
    std::int8_t left[kMaxPort];   // LEFT-side drive ports (|p| in 1..21); SIGNED iff signsMeasured
    std::size_t leftCount;        // 0 = UNSET
    std::int8_t right[kMaxPort];  // RIGHT-side drive ports; SIGNED iff signsMeasured
    std::size_t rightCount;       // 0 = UNSET
    Cartridge cartridge;          // Unset = nobody has READ one off a motor yet
    std::uint8_t imuPort;         // 0 = UNSET (none mounted, or not reported)
    bool measured;                // true only when every SET field was read off the robot
    bool signsMeasured;           // true = every port entry above carries its MEASURED sign
    const char* provenance;       // who measured what, and when -- printed in every banner
    // ── DRIVE GEOMETRY (R3b Parts 1-3, 2026-09-14): what the LIBRARY graph needs and the
    //    tester does not. 0 = UNSET, and UNSET REFUSES a program that boots from the table
    //    (describeMissingForLibrary); a value is typed here only from a measurement, with its
    //    provenance beside it. NEVER guessed: TankKinematics needs the track width and the
    //    odometry needs the scale, and a guess in either drives the robot wrong in silence.
    double wheelDiameterIn;       // wheel diameter, inches (ruler across the tread); 0 = UNSET
    double trackWidthIn;          // contact line to contact line, inches (tape); 0 = UNSET
    double externalRatio;         // motor output shaft -> wheel: wheel revs per motor rev
                                  // (1.0 = direct drive; tooth counts otherwise); 0 = UNSET
    const char* geometryProvenance;  // who measured the three numbers above, how, and when
    double pulseYawFloorDeg = 0.0;         // Calibrate: the yaw one pulsed drive motor must turn the robot (0 = default)
    double restYawRateFloorDegPerS = 0.0;  // Calibrate: "still" means a yaw rate under this (0 = default)
};

// ROBOT TWO -- robot one's TWIN (chunk ROB1-BT, 2026-11-16; the team lead, 2026-11-13: "IT IS LITERALLY
// THE SAME ROBOT BUILD JUST A MIRRORED ROUTINE"). Its table is robot one's design (kRobotOneChassis,
// below): shulib Calibrate DISCOVERS the drive ports, their directions, the sides, the IMU, the scale and
// the track width. The one typed fact, the drive motors' CARTRIDGE, is UNSET until the builder reads it
// off robot two (robot one's is BLUE) -- Calibrate refuses by name until then. test/robot_twin_test.cpp
// holds the two tables equal but for the name, the cartridge and the provenance.
// RETIRED: the old robot two -- the tank chassis with claws -- had the project's one measured, SIGNED
// table (ports 11-20 with their signs, BLUE, the IMU on port 2; R3b-PROGRESS Session 2). It is kept,
// byte for byte, as the host-test fixture test/retired_robot_two.hpp; no build reads it.
// kChassis in the six ROBOT=tank builds (chassisTableFor(RobotId::RobotTwo)).
inline constexpr ChassisTable kRobotTwoChassis = {
    .robot = "2026 CASCADE ROBOT (robot two)",
    .left = {},
    .leftCount = 0,
    .right = {},
    .rightCount = 0,
    .cartridge = Cartridge::Unset,
    .imuPort = 0,
    .measured = false,
    .signsMeasured = false,
    .provenance = "nothing typed -- the cartridge is read off robot two; Calibrate discovers the rest",
    .wheelDiameterIn = 0.0,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "nothing typed -- Step 0 (docs/robot-facts-form.md)",
    .pulseYawFloorDeg = robot_two_facts::kCalibratePulseYawFloorDeg.value(),
    .restYawRateFloorDegPerS = robot_two_facts::kCalibrateRestYawRateDegPerS.value(),
};

// THE BENCH BOT -- measured. 2026-08-13 census, amended by R3a-PROGRESS §9.1 (port 13
// mechanically repaired => 8 motors, 4 per side, symmetric; §5.3's asymmetry ruling
// withdrawn) and the sides confirmed by the §15-§19 whole-robot pushes. IMU on port 4.
// Cartridge BLUE per the team, while the brain was found configured GREEN against it
// (§20.3, "the cartridge fix is still owed") -- so the value here is the team's, and the
// DRIVE station prints it as a belief before writing it; reading the insert colour off a
// motor is worksheet 3.3 and is what settles HA-15.
// SIGNS: NOT in this table (signsMeasured = false; every entry positive). The bench bot's
// signs were captured on the panel (R3a-PROGRESS §17.3/§18.2) but never typed into a
// table, so its DRIVE station keeps using MOTOR WATCH's per-power-cycle capture (the old
// drive program refused it: an unsigned table is not a drivable one).
// kChassis in the ROBOT=bench build — `make` with no ROBOT — and on the host, which sets no switch
// (chassisTableFor(RobotId::BenchBot)).
inline constexpr ChassisTable kBenchBotChassis = {
    .robot = "TANK BENCH BOT (measured) -- NOT the invented X-drive",
    .left = {15, 16, 17, 18},
    .leftCount = 4,
    .right = {11, 12, 13, 14},
    .rightCount = 4,
    .cartridge = Cartridge::Blue,
    .imuPort = 4,
    .measured = true,
    .signsMeasured = false,
    .provenance = "ports+IMU: 2026-08-13 census, sides by R3a-PROGRESS S15-S19 pushes; "
                  "cartridge BLUE per the team (brain was GREEN, S20.3) -- read the insert; "
                  "signs NOT in the table (captured per power cycle by MOTOR WATCH)",
    .wheelDiameterIn = 2.75,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "wheel 2.75 in measured on the bench bot (R3a); track width and ratio "
                          "UNSET -- no library graph is built for this robot",
};

// THE INVENTED X-DRIVE (A4 register HA-111) -- NO typed chassis table. Its drivetrain is data of
// another kind (src/robots/xdrive_config.hpp's kInventedXDrive, a config::RobotConfig), and no program in
// its build reads a chassis table. Every field is UNSET, so anything that ever did read this one
// would refuse by name (describeMissing) instead of driving another robot's ports. Until G1 Part
// 2c's final run the xdrive build carried the BENCH BOT's table here (the #else of this file's old
// #if) -- another robot's facts in a build, dormant only because nothing in that build read them.
// kChassis in the ROBOT=xdrive build (chassisTableFor(RobotId::InventedXDrive)).
inline constexpr ChassisTable kNoChassis = {
    .robot = "INVENTED X-DRIVE -- no typed chassis table (src/robots/xdrive_config.hpp, HA-111)",
    .left = {},
    .leftCount = 0,
    .right = {},
    .rightCount = 0,
    .cartridge = Cartridge::Unset,
    .imuPort = 0,
    .measured = false,
    .signsMeasured = false,
    .provenance = "none: the invented X-drive is src/robots/xdrive_config.hpp, not a typed table",
    .wheelDiameterIn = 0.0,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "none: no typed chassis table for this robot",
};

// ROBOT ONE -- the 2026 CASCADE robot (ROB1-A, 2026-11-11; the builder, 2026-11-05: "both robots
// identical, 15 in"; built by 2026-11-15). ONE drive fact is TYPED: the drive motors' CARTRIDGE, BLUE
// (the team's builder, 2026-11-15, read off the robot -- the facts form's A2; ROB1-C2) -- the one fact
// Calibrate cannot discover and refuses without. Every other field is UNSET by this table's own rule --
// NOT the facts ledger (src/robots/robot_one_facts.hpp holds its mechanisms' numbers): no ports (shulib
// Calibrate DISCOVERS which ports drive, their directions, the sides, the front, the scale and the track
// width), no IMU port (Calibrate DISCOVERS the IMU from the brain's registry and records it -- a port typed
// here would only be its cross-check, calibrate_program.hpp's `imuPortStated`; the library still cannot
// drive a tank WITHOUT an IMU on the robot -- HA-253), no geometry. Every station and program that needs
// one of these refuses BY NAME (describeMissing below) -- nothing is guessed. Robot one is believed a
// TANK like robot two ("identical") -- HA-252, the form's A1.
// kChassis in the six ROBOT=one builds (chassisTableFor(RobotId::RobotOne)).
inline constexpr ChassisTable kRobotOneChassis = {
    .robot = "2026 CASCADE ROBOT (robot one)",
    .left = {},
    .leftCount = 0,
    .right = {},
    .rightCount = 0,
    .cartridge = Cartridge::Blue,
    .imuPort = 0,
    .measured = false,
    .signsMeasured = false,
    .provenance = "cartridge BLUE: the team's builder, 2026-11-15, read off the robot; Calibrate discovers the rest",
    .wheelDiameterIn = 0.0,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "nothing typed -- Step 0 (docs/robot-facts-form.md)",
    // Calibrate's thresholds for its 10-motor drive: the facts ledger's (chosen, 2026-11-16).
    .pulseYawFloorDeg = robot_one_facts::kCalibratePulseYawFloorDeg.value(),
    .restYawRateFloorDegPerS = robot_one_facts::kCalibrateRestYawRateDegPerS.value(),
};

/// Robot `id`'s chassis table: THE ONE PLACE a robot becomes a table. A case per robot and no
/// default, so a robot added to RobotId (src/build_selection.hpp) without a table here is -Wswitch
/// (a src build gate FAILURE, a host compile error), and reaching the end in a constant expression
/// is a compile error. The host suite calls it for EVERY RobotId (test/chassis_table_test.cpp).
constexpr const ChassisTable& chassisTableFor(RobotId id) {
    switch (id) {
        case RobotId::BenchBot:
            return kBenchBotChassis;
        case RobotId::RobotTwo:
            return kRobotTwoChassis;
        case RobotId::InventedXDrive:
            return kNoChassis;
        case RobotId::RobotOne:
            return kRobotOneChassis;
    }
    __builtin_unreachable();
}

/// THE BUILD'S TABLE — the only name a program uses (the src build gate's step 0 fails one that
/// names a per-robot table): the table of the robot src/build_selection.hpp's one chain chose for
/// this build. No #if: a misspelt robot here is a compile error.
inline constexpr const ChassisTable& kChassis = chassisTableFor(kRobotId);

// THE SELECTION, restated as a check (the mechanism table's precedent): kChassis must be the table
// chassisTableFor() gives THIS build's robot -- so replacing kRobotId above with a fixed robot, or
// kChassis with a table by name, fails every build whose robot is not that one. A typo would have to
// be made twice, in two places, to get past it; the mapping itself is host-tested per robot.
static_assert(&kChassis == &chassisTableFor(kRobotId),
              "kChassis must be chassisTableFor(kRobotId): this build's robot, chosen by the one chain");
// ...and the mapping itself restated per robot, by NAME, so a wrong robot inside chassisTableFor()
// fails that robot's build too (the host suite calls it for every robot; the robot builds are the
// ones that carry the answer). (Until ROB1-BT robot two's table also had to be the SIGNED one; the twin's
// is robot one's design, every drive field UNSET, so it is restated by ADDRESS, like robot one's.)
static_assert(kRobotId != RobotId::RobotTwo || &kChassis == &kRobotTwoChassis,
              "ROBOT=tank must select robot two's table");
static_assert(kRobotId != RobotId::BenchBot || &kChassis == &kBenchBotChassis,
              "the bench build must select the bench bot's table");
static_assert(kRobotId != RobotId::InventedXDrive || &kChassis == &kNoChassis,
              "the X-drive build must select the UNSET table (its robot is src/robots/xdrive_config.hpp)");
static_assert(kRobotId != RobotId::RobotOne || &kChassis == &kRobotOneChassis,
              "ROBOT=one must select robot one's table");

/// |p| for a (possibly signed) table entry, as an int.
constexpr int absPort(std::int8_t p) { return p < 0 ? -static_cast<int>(p) : static_cast<int>(p); }

/// Both sides have ports. (The IMU and the cartridge are checked separately, because the
/// stations that need them differ: DRIVE needs ports + cartridge, the IMU test needs the
/// IMU port, the census needs nothing.)
constexpr bool tableHasPorts(const ChassisTable& t) { return t.leftCount > 0 && t.rightCount > 0; }
constexpr std::size_t tableCount(const ChassisTable& t) { return t.leftCount + t.rightCount; }

/// -1 LEFT, +1 RIGHT, 0 not a drive port in the table (or the table has no ports). Matches
/// on |port|, so a signed entry and a physical port number find each other.
constexpr int tableSideOf(const ChassisTable& t, int port) {
    for (std::size_t i = 0; i < t.leftCount; ++i) {
        if (absPort(t.left[i]) == port) return -1;
    }
    for (std::size_t i = 0; i < t.rightCount; ++i) {
        if (absPort(t.right[i]) == port) return +1;
    }
    return 0;
}

/// The table's entry for physical `port` AS TYPED -- signed when signsMeasured, positive
/// otherwise -- or 0 when the port is not in the table. THIS is the value the tester's DRIVE
/// station hands to ProsMotor: the one place a sign lives.
constexpr std::int8_t tableSignedPort(const ChassisTable& t, int port) {
    for (std::size_t i = 0; i < t.leftCount; ++i) {
        if (absPort(t.left[i]) == port) return t.left[i];
    }
    for (std::size_t i = 0; i < t.rightCount; ++i) {
        if (absPort(t.right[i]) == port) return t.right[i];
    }
    return 0;
}

/// The table's sign for physical `port`: +1 / -1 when signsMeasured and the port is listed,
/// 0 otherwise (unsigned table, or not a table port). MOTOR WATCH compares its capture to
/// this; the DRIVE station refuses when the two disagree.
constexpr int tableSignOf(const ChassisTable& t, int port) {
    if (!t.signsMeasured) return 0;
    const std::int8_t p = tableSignedPort(t, port);
    return p == 0 ? 0 : (p < 0 ? -1 : +1);
}

/// The cartridge as words, for banners.
inline const char* cartridgeWord(Cartridge c) {
    switch (c) {
        case Cartridge::Red:   return "RED 100 rpm";
        case Cartridge::Green: return "GREEN 200 rpm";
        case Cartridge::Blue:  return "BLUE 600 rpm";
        default:               return "UNSET";
    }
}

/// Names every field a caller needs that is UNSET, comma-separated, into `buf`.
/// Returns true when something is missing. `forDrive` = ports + cartridge; otherwise
/// every field, IMU included -- the banner uses the full list.
inline bool describeMissing(const ChassisTable& t, char* buf, std::size_t n, bool forDrive) {
    buf[0] = '\0';
    bool any = false;
    auto add = [&](const char* what) {
        if (any) std::strncat(buf, ", ", n - std::strlen(buf) - 1);
        std::strncat(buf, what, n - std::strlen(buf) - 1);
        any = true;
    };
    if (t.leftCount == 0) add("LEFT ports");
    if (t.rightCount == 0) add("RIGHT ports");
    if (t.cartridge == Cartridge::Unset) add("cartridge");
    if (!forDrive && t.imuPort == 0) add("IMU port");
    return any;
}

/// True when all three geometry numbers are SET (> 0). The library graph needs all three.
constexpr bool tableGeometrySet(const ChassisTable& t) {
    return t.wheelDiameterIn > 0.0 && t.trackWidthIn > 0.0 && t.externalRatio > 0.0;
}

/// The ONE drive-geometry object the library graph hands to BOTH the drive-encoder odometry
/// and the stall check (hal::DriveGeometry: wheel radius x motor->wheel ratio -> inches per
/// radian), built from the table's measured diameter and ratio. Precondition: geometry set
/// (tableGeometrySet) -- tableToDescription() calls it only then. Robot two is
/// believed symmetric, so one object serves both sides; the type represents asymmetry the
/// day a per-side measurement says otherwise.
inline hal::DriveGeometry tableDriveGeometry(const ChassisTable& t) {
    return hal::DriveGeometry::fromDiameter(units::Length{t.wheelDiameterIn}, t.externalRatio);
}

/// The table's track width as the typed Length TankKinematics and the odometry take.
inline units::Length tableTrackWidth(const ChassisTable& t) {
    return units::Length{t.trackWidthIn};
}

/// Names every field the LIBRARY graph needs that is UNSET: everything describeMissing()
/// checks with the IMU included, PLUS the three geometry numbers. Returns true when
/// something is missing; the boot source prints `buf` on serial (through
/// describeMissingForDescription(), in its `typed table: cannot describe a drivetrain -- UNSET:`
/// line) and a boot with no usable record refuses.
inline bool describeMissingForLibrary(const ChassisTable& t, char* buf, std::size_t n) {
    const bool any = describeMissing(t, buf, n, false);
    auto add = [&](const char* what) {
        if (buf[0] != '\0') std::strncat(buf, ", ", n - std::strlen(buf) - 1);
        std::strncat(buf, what, n - std::strlen(buf) - 1);
    };
    bool geometryMissing = false;
    if (t.wheelDiameterIn <= 0.0) { add("wheel diameter"); geometryMissing = true; }
    if (t.trackWidthIn <= 0.0) { add("track width"); geometryMissing = true; }
    if (t.externalRatio <= 0.0) { add("external ratio (motor->wheel)"); geometryMissing = true; }
    return any || geometryMissing;
}

/// Names every reason this table cannot produce a VALID hal::DrivetrainDescription: everything
/// describeMissingForLibrary() checks, PLUS the MEASURED SIGNS. Returns true when something is
/// missing. The signs are the addition that matters: an UNSIGNED table's entries are all
/// positive and would make a description that LOOKS complete — ten ports, two sides, a scale and
/// a width, every check in valid() passed — while half its motors would fight their side-mates on
/// the first command. An unsigned table is not a drivable one (the header's own rule), so it is
/// not a describable one either.
inline bool describeMissingForDescription(const ChassisTable& t, char* buf, std::size_t n) {
    const bool any = describeMissingForLibrary(t, buf, n);
    if (!t.signsMeasured) {
        if (buf[0] != '\0') std::strncat(buf, ", ", n - std::strlen(buf) - 1);
        std::strncat(buf, "measured signs", n - std::strlen(buf) - 1);
        return true;
    }
    return any;
}

/// THE SECOND SOURCE of hal::DrivetrainDescription (R3d Part 1, T9): the typed table, mapped
/// into the one representation the calibrator also produces, so a composition root has exactly
/// one input type with two sources.
///
/// What is carried over, unchanged: the SIGNED port entries bit for bit (the sign lives inside
/// the port number, here as everywhere), the cartridge, the IMU port, the track width, and the
/// geometry provenance string -- truncated to the description's fixed buffer, because a
/// description must be writable to a card as it stands. `source` is TypedTable, never
/// RobotMeasured: a number a person typed off a tape is not a number the robot measured, and the
/// banner must be able to say which it is looking at.
///
/// A table that cannot describe a drivetrain yields an INVALID description rather than a
/// plausible one (describeMissingForDescription names why): UNSET geometry leaves the scales and
/// the width at 0, an UNSET cartridge leaves a byte outside the enum, an UNSET IMU leaves port 0,
/// and an UNSIGNED table leaves BOTH SIDES EMPTY. Nothing here loosens the table's own rule that
/// a value is never invented -- valid() refuses each of those in turn, naming it.
[[nodiscard]] inline hal::DrivetrainDescription tableToDescription(const ChassisTable& t) {
    hal::DrivetrainDescription d{};
    d.kind = hal::DrivetrainKind::Tank;
    switch (t.cartridge) {
        case Cartridge::Red:   d.cartridge = hal::MotorCartridge::Red; break;
        case Cartridge::Green: d.cartridge = hal::MotorCartridge::Green; break;
        case Cartridge::Blue:  d.cartridge = hal::MotorCartridge::Blue; break;
        // UNSET stays the byte 0, which is not a member of the enum: valid() refuses it.
        case Cartridge::Unset: break;
    }
    d.imuPort = t.imuPort;
    d.source = hal::DescriptionSource::TypedTable;
    if (t.signsMeasured) {
        const std::size_t nLeft = t.leftCount < hal::kMaxSmartPorts ? t.leftCount
                                                                    : hal::kMaxSmartPorts;
        const std::size_t nRight = t.rightCount < hal::kMaxSmartPorts ? t.rightCount
                                                                      : hal::kMaxSmartPorts;
        for (std::size_t i = 0; i < nLeft; ++i) d.left.signedPorts[i] = t.left[i];
        for (std::size_t i = 0; i < nRight; ++i) d.right.signedPorts[i] = t.right[i];
        d.left.count = static_cast<std::uint8_t>(nLeft);
        d.right.count = static_cast<std::uint8_t>(nRight);
    }
    if (tableGeometrySet(t)) {
        // ONE object serves both sides: robot two is BELIEVED symmetric and the table has no
        // per-side measurement to say otherwise. The description represents asymmetry the day
        // one does -- and the calibrator's two independent scales already do.
        const units::Length ipr = tableDriveGeometry(t).inchesPerRadian();
        d.left.inchesPerRadian = ipr;
        d.right.inchesPerRadian = ipr;
        d.trackWidth = tableTrackWidth(t);
    }
    // "TYPED TABLE (<robot>): <geometry provenance>", cut to the description's fixed buffer.
    // Appended BY HAND rather than with snprintf (G1 Part 2b): with a constant table the compiler
    // can see the truncation is CERTAIN, and -Wformat-truncation — an error in the src build gate
    // — fired the first time this was inlined into a translation unit that every build compiles
    // (src/common/boot_source.cpp). The bytes are exactly what the snprintf wrote: the same text, cut
    // one short of the buffer, always terminated.
    std::size_t n = 0;
    const char* pieces[4] = {"TYPED TABLE (", t.robot != nullptr ? t.robot : "", "): ",
                             t.geometryProvenance != nullptr ? t.geometryProvenance : ""};
    for (const char* piece : pieces) {
        for (std::size_t k = 0; piece[k] != '\0' && n + 1 < sizeof d.provenance; ++k) {
            d.provenance[n++] = piece[k];
        }
    }
    d.provenance[n] = '\0';
    return d;
}

/// "15 16 17 18", or "-11 +12 -13 +14 -15" when `withSigns`, or "UNSET".
inline void portsToString(const std::int8_t* ports, std::size_t n, bool withSigns, char* buf,
                          std::size_t cap) {
    buf[0] = '\0';
    if (n == 0) {
        std::snprintf(buf, cap, "UNSET");
        return;
    }
    for (std::size_t i = 0; i < n; ++i) {
        char one[8];
        if (withSigns) {
            std::snprintf(one, sizeof one, "%s%+d", i ? " " : "", static_cast<int>(ports[i]));
        } else {
            std::snprintf(one, sizeof one, "%s%d", i ? " " : "", static_cast<int>(ports[i]));
        }
        std::strncat(buf, one, cap - std::strlen(buf) - 1);
    }
}

/// A table that contradicts itself is worse than an unset one. Printed at boot, refused by
/// DRIVE (and by the old drive program, until T4 Part 3). Returns true when the table is internally consistent;
/// otherwise `why` names the first contradiction. The checks, each the thing a typo would
/// produce:
///   1. every entry is a real port: |p| in 1..21 (0 is "no port", and cannot be an entry)
///   2. no port listed twice, on one side OR across both, judged by |p| -- a port cannot
///      drive both sides, and a signed table must not carry `+13` on one side and `-13` on
///      the other
///   3. the IMU port is not a drive port
///   4. a SIGNED table (signsMeasured) has BOTH sides non-empty -- signs for a side that has
///      no ports is a claim about nothing
///   5. an UNSIGNED table carries only POSITIVE entries -- a '-' typed without flipping
///      signsMeasured is a half-signed table: nobody can tell whether the other entries'
///      '+' is measured or default
///   6. (R3b Parts 1-3) every geometry number is FINITE and >= 0 -- 0 is UNSET and legal; a
///      negative or non-finite value is a typo, not a measurement, and is refused here rather
///      than reaching TankKinematics or the odometry as a precondition mid-boot
inline bool tableConsistent(const ChassisTable& t, char* why, std::size_t cap) {
    why[0] = '\0';
    const double geometry[3] = {t.wheelDiameterIn, t.trackWidthIn, t.externalRatio};
    const char* geometryNames[3] = {"wheel diameter", "track width", "external ratio"};
    for (int g = 0; g < 3; ++g) {
        if (!std::isfinite(geometry[g]) || geometry[g] < 0.0) {
            std::snprintf(why, cap, "%s is %g -- a geometry value must be >= 0 (0 = UNSET)",
                          geometryNames[g], geometry[g]);
            return false;
        }
    }
    const std::int8_t* sides[2] = {t.left, t.right};
    const std::size_t counts[2] = {t.leftCount, t.rightCount};
    const char* names[2] = {"LEFT", "RIGHT"};
    for (int s = 0; s < 2; ++s) {
        for (std::size_t i = 0; i < counts[s]; ++i) {
            const std::int8_t p = sides[s][i];
            const int a = absPort(p);
            if (a < 1 || a > kMaxPort) {
                std::snprintf(why, cap, "%s entry %d is not a port in 1..21", names[s],
                              static_cast<int>(p));
                return false;
            }
            if (!t.signsMeasured && p < 0) {
                std::snprintf(why, cap, "%s lists %d with a sign but signsMeasured is false "
                                        "(half-signed table)", names[s], static_cast<int>(p));
                return false;
            }
            if (t.imuPort != 0 && a == static_cast<int>(t.imuPort)) {
                std::snprintf(why, cap, "port %d is both %s and the IMU port", a, names[s]);
                return false;
            }
            for (std::size_t j = i + 1; j < counts[s]; ++j) {
                if (absPort(sides[s][j]) == a) {
                    std::snprintf(why, cap, "%s lists port %d twice", names[s], a);
                    return false;
                }
            }
            if (s == 0) {
                for (std::size_t j = 0; j < counts[1]; ++j) {
                    if (absPort(sides[1][j]) == a) {
                        std::snprintf(why, cap, "port %d appears on BOTH sides", a);
                        return false;
                    }
                }
            }
        }
    }
    if (t.signsMeasured && !tableHasPorts(t)) {
        std::snprintf(why, cap, "signsMeasured is true but a side has no ports");
        return false;
    }
    return true;
}

}  // namespace shulib::bench
