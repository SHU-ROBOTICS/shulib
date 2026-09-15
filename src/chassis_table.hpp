#pragma once
//
// THE CHASSIS TABLE — the ONE typed input about a drivetrain (R3b Session 2 brief §3.1;
// moved out of src/bench_r3a.cpp at R3b Part 0b so the bench tester and the drive program
// share ONE definition — two copies of a port table is how two programs come to disagree
// about which robot they are on).
//
// Per variant: which ports drive which side, WITH THEIR MEASURED SIGNS when those have been
// measured, the cartridge, the IMU port, and whether any of it was MEASURED. Selected by the
// SAME define the Makefile's ROBOT switch sets, so `make ROBOT=tank` and this table cannot
// disagree about which robot it is.
//
// RULE: A VALUE IS NEVER INVENTED. An unset field is 0 / Unset, describeMissing() names it on
// screen, and every station or program that needs it refuses until it is filled. HA-111's
// invented port map was exactly this defect class -- every adapter constructor threw at boot
// -- and it is not repeated for robot two. When a fact arrives, THIS is the one place to
// edit, with the date and the source beside the numbers.
//
// SIGNS LIVE HERE AND NOWHERE ELSE (Part 0b, brief §2 and landmine 1): when `signsMeasured`
// is true, every port entry carries its sign -- `-11` is port 11 constructed with a NEGATIVE
// port number, which PROS reverses exactly once, in the adapter. The drive program hands the
// entry to ProsMotor AS TYPED and never negates; the tester's MOTOR WATCH cross-checks a push
// against these signs (AGREES / DISAGREES per port); its DRIVE station uses them and refuses
// if a fresh capture disagrees. A table that says `signsMeasured = false` carries POSITIVE
// entries only -- a '-' typed without flipping the flag is a half-signed table, and
// tableConsistent() refuses it. R3d makes the signs discovered and persisted; tonight they
// are typed, measured values with a date.
//
// PROS-FREE ON PURPOSE: this header is plain data and pure helpers over it, so the host test
// (test/chassis_table_test.cpp) can include it and break the consistency checks. It lives in
// src/, not include/shulib/, because it is a per-robot composition fact, not library API.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "shulib/hal/drive_geometry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// PHYSICAL smart ports are 1..21 -- how humans and every PROS API but the registry name them.
constexpr int kMaxPort = 21;

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
    //    tester/drive program do not. 0 = UNSET, and UNSET REFUSES the library program at boot
    //    (describeMissingForLibrary); a value is typed here only from a measurement, with its
    //    provenance beside it. NEVER guessed: TankKinematics needs the track width and the
    //    odometry needs the scale, and a guess in either drives the robot wrong in silence.
    double wheelDiameterIn;       // wheel diameter, inches (ruler across the tread); 0 = UNSET
    double trackWidthIn;          // contact line to contact line, inches (tape); 0 = UNSET
    double externalRatio;         // motor output shaft -> wheel: wheel revs per motor rev
                                  // (1.0 = direct drive; tooth counts otherwise); 0 = UNSET
    const char* geometryProvenance;  // who measured the three numbers above, how, and when
};

#if defined(SHULIB_ROBOT_TANK_2026)
// ROBOT TWO -- the 2026 tank chassis (build team, stated 2026-09-10): five COUPLED motors
// per side driving four wheels per side.
// PORTS: reported by the team lead 2026-09-10, read off the robot while standing BEHIND
// it (the back of the robot against him, looking toward the front): LEFT 11 12 13 14 15
// and RIGHT 20 19 18 17 16, each listed BACK -> FRONT (11 and 20 are the rearmost motors,
// 15 and 16 the frontmost). THE FRONT is therefore the end where ports 15 and 16 sit --
// the end away from a person who reads the left side as 11..15. Order within a side is
// informational: the table is a set.
// SIGNS: MEASURED 2026-09-10 evening by the tester's MOTOR WATCH -- two whole-robot pushes,
// FRONT-first, identical both times (R3b-PROGRESS Session 2 §10.3):
//     LEFT -11 +12 -13 +14 -15  |  RIGHT +20 -19 +18 -17 +16
// A '-' entry is a port PROS reverses (negative port number in ProsMotor); the alternation
// within a side is the coupled-gear-train signature, not a wiring fault. Port 18 travelled
// ~20 % short of its side-mates on both pushes -- unexplained; watch it under power (A4
// register HA-130; the group's disagreement floor tolerates it, HA-133).
// CARTRIDGE: BLUE, read off a motor by the build team 2026-09-10 -- a reported measurement,
// so it may be set; it is still printed as a BELIEF wherever the adapter is about to WRITE
// it (the read-back cannot detect a wrong belief). IMU: PORT 2, mounted 2026-09-10 and seen
// by the census that evening; the radio is on port 1. Also reported, and deliberately NOT
// here because nothing here uses them yet: wheels 2.75 in, and "600 rpm" (reads as direct
// drive 1:1, UNCONFIRMED until tooth counts arrive -- odometry's business, R3b Part 2).
// `measured` flipped TRUE 2026-09-10 evening: station 1's census showed MOTOR on all ten
// table ports (after cables on 11, 15 and 19 were re-seated), the IMU on port 2 and the
// radio on port 1. The cartridge is still a belief read off a motor, not a measurement.
// GEOMETRY (R3b Parts 1-3, 2026-09-14; A4 register HA-124 wheel, HA-125 track width, HA-126
// ratio): wheel diameter 2.75 in REPORTED by the team lead
// ("standard wheels", the bench bot's measured size) -- a report, not yet a ruler on THIS
// robot; track width UNSET (tape measure, contact line to contact line, owed by the team
// lead); external ratio UNSET ("600 rpm" was reported, which reads as direct drive 1:1 but is
// a cartridge speed, not a tooth count -- the coordinator types 1.0 only when "direct drive"
// or the counts are stated). The library program REFUSES at boot while either is 0.
inline constexpr ChassisTable kChassis = {
    .robot = "2026 TANK CHASSIS (robot two)",
    .left = {-11, 12, -13, 14, -15},
    .leftCount = 5,
    .right = {20, -19, 18, -17, 16},
    .rightCount = 5,
    .cartridge = Cartridge::Blue,
    .imuPort = 2,
    .measured = true,
    .signsMeasured = true,
    .provenance = "ports + IMU 2 + radio 1: census 2026-09-10 evening, all ten motors; "
                  "SIGNS: MOTOR WATCH, two front-first pushes 2026-09-10 evening, identical; "
                  "front = the 15/16 end (team lead); cartridge BLUE off a motor",
    .wheelDiameterIn = 2.75,
    .trackWidthIn = 0.0,
    .externalRatio = 0.0,
    .geometryProvenance = "wheel 2.75 in REPORTED by the team lead 2026-09-14 (standard wheels; "
                          "not yet rulered on this robot); track width UNSET (tape owed); ratio "
                          "UNSET (\"600 rpm\" reported = a cartridge, not a tooth count)",
};
#else
// THE BENCH BOT -- measured. 2026-08-13 census, amended by R3a-PROGRESS §9.1 (port 13
// mechanically repaired => 8 motors, 4 per side, symmetric; §5.3's asymmetry ruling
// withdrawn) and the sides confirmed by the §15-§19 whole-robot pushes. IMU on port 4.
// Cartridge BLUE per the team, while the brain was found configured GREEN against it
// (§20.3, "the cartridge fix is still owed") -- so the value here is the team's, and the
// DRIVE station prints it as a belief before writing it; reading the insert colour off a
// motor is worksheet 3.3 and is what settles HA-15.
// SIGNS: NOT in this table (signsMeasured = false; every entry positive). The bench bot's
// signs were captured on the panel (R3a-PROGRESS §17.3/§18.2) but never typed into a
// table, so its DRIVE station keeps using MOTOR WATCH's per-power-cycle capture, and the
// drive program refuses it: an unsigned table is not a drivable one.
inline constexpr ChassisTable kChassis = {
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
#endif

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
/// otherwise -- or 0 when the port is not in the table. THIS is the value a drive program
/// hands to ProsMotor: the one place a sign lives.
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
/// (tableGeometrySet) -- the composition root refuses before calling this. Robot two is
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
/// something is missing; the composition root paints `buf` on the state screen and refuses.
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
/// DRIVE and by the drive program. Returns true when the table is internally consistent;
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
