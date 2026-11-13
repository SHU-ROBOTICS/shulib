// hal::DrivetrainDescription — the ONE representation of a drivetrain's discovered facts — and
// hal::DriveGeometry's calibrated constructor (chunk R3d Part 1, brief §5 tests 1 and 3).
//   1  validity: every contradiction a typo, a mis-wire or a corrupted record produces is
//      refused, each as the SOLE cause in its own case, each naming itself
//                                                        (mutation: remove any ONE check)
//   1b the accessors: sideOf / signedPortOf match on |port| and hand back the SIGN AS STORED;
//      describe() names the source; the geometry accessors carry the measured scale
//   3  fromInchesPerRadian(k).inchesPerRadian() == k EXACTLY, and equals fromDiameter(d, r)
//      bit for bit for k = d/2 x r                        (multiply by 2 anywhere in the chain)
//   +  the port-count constant has ONE home: kMaxSmartPorts, pinned equal to the two constants
//      that may not depend on it
//
// Nothing here has run on a robot. Every description in this file was built by hand.

#include "doctest.h"

#include <cstring>
#include <limits>

#include "robots/chassis_table.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/teleop/coupled_side_monitor.hpp"
#include "shulib/units/quantity.hpp"

using shulib::hal::cartridgeWord;
using shulib::hal::DescriptionSource;
using shulib::hal::DriveGeometry;
using shulib::hal::DrivetrainDescription;
using shulib::hal::DrivetrainKind;
using shulib::hal::kMaxSmartPorts;
using shulib::hal::MotorCartridge;
using shulib::units::Length;

// ── ONE HOME FOR THE PORT COUNT ─────────────────────────────────────────────────────
// hal::kMaxSmartPorts is the library's home for "a V5 brain has 21 smart ports".
// src/robots/chassis_table.hpp's kMaxPort CITES it (one definition, checked by the compiler).
// teleop::kMaxSideMembers deliberately does NOT: hal::MotorGroup already includes
// teleop/coupled_side_monitor.hpp, so a citation the other way would close a directory-level
// cycle and would make a pure fight detector depend on a drivetrain description. The equality is
// pinned HERE instead — a pin with no dependency, which is the same trade the brief's landmine 6
// offers for the median ("share it or pin equality").
static_assert(shulib::teleop::kMaxSideMembers == kMaxSmartPorts,
              "a side cannot have more members than the brain has ports");
static_assert(shulib::bench::kMaxPort == static_cast<int>(kMaxSmartPorts),
              "the chassis table's port bound must be the library's");

namespace {

/// Robot two's shape, as a description: five signed ports per side, IMU 2, blue, with a scale and
/// a width standing in for measurements nothing has taken yet (this is a HAND-BUILT description —
/// no robot has produced one).
[[nodiscard]] DrivetrainDescription robotTwoShaped() {
    DrivetrainDescription d{};
    d.kind = DrivetrainKind::Tank;
    d.cartridge = MotorCartridge::Blue;
    d.imuPort = 2;
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
    d.trackWidth = Length{13.25};
    d.source = DescriptionSource::RobotMeasured;
    std::snprintf(d.provenance, sizeof d.provenance, "hand-built in a host test");
    return d;
}

}  // namespace

// ═══ 1. Validity: each contradiction refused, alone, by name ════════════════════════
// Bug caught: a contradictory description reaching a constructor — two sides claiming one port,
// a scale of zero silently scaling every travel to nothing, an IMU port that is also a motor, a
// cartridge byte nobody set. Each case below is the VALID description with exactly ONE thing
// wrong, so a check that stops firing cannot hide behind another check.
TEST_CASE("DrivetrainDescription 1: every contradiction is refused as the sole cause, and named") {
    char why[160];
    const DrivetrainDescription good = robotTwoShaped();
    REQUIRE(good.valid(why, sizeof why));
    CHECK(why[0] == '\0');
    CHECK(good.valid());  // the no-buffer overload agrees

    SUBCASE("a port listed twice on ONE side") {
        DrivetrainDescription d = good;
        d.left.signedPorts[3] = -12;  // 14 -> 12 again, opposite sign: the same typo
        CHECK_FALSE(d.valid(why, sizeof why));
        CHECK(std::strstr(why, "LEFT lists port 12 twice") != nullptr);
    }
    SUBCASE("a port on BOTH sides, judged by |port|") {
        DrivetrainDescription d = good;
        d.right.signedPorts[2] = -13;  // 18 -> the left side's 13, opposite sign
        CHECK_FALSE(d.valid(why, sizeof why));
        CHECK(std::strstr(why, "BOTH sides") != nullptr);
        CHECK(std::strstr(why, "13") != nullptr);
    }
    SUBCASE("an entry outside 1..21, on either sign, and 0") {
        for (int bad : {0, 22, -22, 100, -100}) {
            CAPTURE(bad);
            DrivetrainDescription d = good;
            d.left.signedPorts[0] = static_cast<std::int8_t>(bad);
            CHECK_FALSE(d.valid(why, sizeof why));
            CHECK(std::strstr(why, "not a port in 1..21") != nullptr);
        }
        // The boundary from the legal side: 21 and -21 are ports.
        DrivetrainDescription edge = good;
        edge.left.signedPorts[0] = -21;
        CHECK(edge.valid(why, sizeof why));
        edge.left.signedPorts[0] = 21;
        CHECK(edge.valid(why, sizeof why));
    }
    SUBCASE("the IMU port doubling as a drive port, either side, either sign") {
        DrivetrainDescription l = good;
        l.imuPort = 13;  // the left side's -13
        CHECK_FALSE(l.valid(why, sizeof why));
        CHECK(std::strstr(why, "IMU") != nullptr);
        CHECK(std::strstr(why, "LEFT") != nullptr);
        DrivetrainDescription r = good;
        r.imuPort = 18;
        CHECK_FALSE(r.valid(why, sizeof why));
        CHECK(std::strstr(why, "RIGHT") != nullptr);
    }
    SUBCASE("an IMU port outside 1..21, including UNSET") {
        for (int bad : {0, 22, 255}) {
            CAPTURE(bad);
            DrivetrainDescription d = good;
            d.imuPort = static_cast<std::uint8_t>(bad);
            CHECK_FALSE(d.valid(why, sizeof why));
            CHECK(std::strstr(why, "IMU port") != nullptr);
        }
        DrivetrainDescription edge = good;
        edge.imuPort = 21;
        CHECK(edge.valid(why, sizeof why));  // 21 is a port; only 22 is not
    }
    SUBCASE("an empty side — either one") {
        DrivetrainDescription l = good;
        l.left.count = 0;
        CHECK_FALSE(l.valid(why, sizeof why));
        CHECK(std::strstr(why, "LEFT side has no ports") != nullptr);
        DrivetrainDescription r = good;
        r.right.count = 0;
        CHECK_FALSE(r.valid(why, sizeof why));
        CHECK(std::strstr(why, "RIGHT side has no ports") != nullptr);
    }
    SUBCASE("a side claiming more ports than the brain has") {
        DrivetrainDescription d = good;
        d.left.count = 22;
        CHECK_FALSE(d.valid(why, sizeof why));
        CHECK(std::strstr(why, "the brain has 21") != nullptr);
    }
    SUBCASE("inches per radian <= 0, NaN or infinite, on either side") {
        const double bad[] = {0.0, -1.375, std::numeric_limits<double>::quiet_NaN(),
                              std::numeric_limits<double>::infinity()};
        for (double v : bad) {
            CAPTURE(v);
            DrivetrainDescription l = good;
            l.left.inchesPerRadian = Length{v};
            CHECK_FALSE(l.valid(why, sizeof why));
            CHECK(std::strstr(why, "LEFT inches per radian") != nullptr);
            DrivetrainDescription r = good;
            r.right.inchesPerRadian = Length{v};
            CHECK_FALSE(r.valid(why, sizeof why));
            CHECK(std::strstr(why, "RIGHT inches per radian") != nullptr);
        }
        // Just over the line is fine: a tiny but positive scale is a measurement, not a typo.
        DrivetrainDescription tiny = good;
        tiny.left.inchesPerRadian = Length{std::numeric_limits<double>::denorm_min()};
        CHECK(tiny.valid(why, sizeof why));
    }
    SUBCASE("track width <= 0, NaN or infinite") {
        const double bad[] = {0.0, -13.25, std::numeric_limits<double>::quiet_NaN(),
                              -std::numeric_limits<double>::infinity()};
        for (double v : bad) {
            CAPTURE(v);
            DrivetrainDescription d = good;
            d.trackWidth = Length{v};
            CHECK_FALSE(d.valid(why, sizeof why));
            CHECK(std::strstr(why, "track width") != nullptr);
        }
    }
    SUBCASE("a kind that is not Tank") {
        DrivetrainDescription d = good;
        d.kind = static_cast<DrivetrainKind>(7);
        CHECK_FALSE(d.valid(why, sizeof why));
        CHECK(std::strstr(why, "not Tank") != nullptr);
    }
    SUBCASE("a cartridge byte nobody set, or one outside the enum") {
        for (int bad : {0, 4, 255}) {
            CAPTURE(bad);
            DrivetrainDescription d = good;
            d.cartridge = static_cast<MotorCartridge>(bad);
            CHECK_FALSE(d.valid(why, sizeof why));
            CHECK(std::strstr(why, "cartridge") != nullptr);
            CHECK(std::strcmp(cartridgeWord(d.cartridge), "UNSET") == 0);
        }
        for (MotorCartridge c : {MotorCartridge::Red, MotorCartridge::Green, MotorCartridge::Blue}) {
            DrivetrainDescription d = good;
            d.cartridge = c;
            CHECK(d.valid(why, sizeof why));
            CHECK(std::strcmp(cartridgeWord(c), "UNSET") != 0);
        }
    }
    SUBCASE("a DEFAULT-constructed description is invalid — never merely incomplete") {
        const DrivetrainDescription empty{};
        CHECK_FALSE(empty.valid(why, sizeof why));
        CHECK(why[0] != '\0');
        CHECK(empty.provenance[0] == '\0');
        // ... and the FIRST thing it is refused for is the unstated cartridge, not a
        // downstream consequence of it.
        CHECK(std::strstr(why, "cartridge") != nullptr);
    }
    SUBCASE("valid() never writes past the buffer it is given") {
        DrivetrainDescription d = good;
        d.left.count = 0;
        char tiny[8];
        std::memset(tiny, 'x', sizeof tiny);
        CHECK_FALSE(d.valid(tiny, 6));
        CHECK(tiny[5] == '\0');  // NUL inside the stated cap
        CHECK(tiny[6] == 'x');   // and nothing beyond it
        CHECK(tiny[7] == 'x');
        CHECK_FALSE(d.valid(nullptr, 0));  // the no-reason path does not dereference
    }
}

// ═══ 1b. The accessors ══════════════════════════════════════════════════════════════
// Bug caught: a consumer re-deriving a sign (the sign must come back AS STORED, once), a side
// lookup that only matches positive entries, or a banner that cannot tell a typed description
// from a measured one.
TEST_CASE("DrivetrainDescription 1b: sideOf / signedPortOf / describe / the geometry accessors") {
    const DrivetrainDescription d = robotTwoShaped();
    for (int p : {11, 12, 13, 14, 15}) {
        CHECK(d.sideOf(p) == -1);
    }
    for (int p : {16, 17, 18, 19, 20}) {
        CHECK(d.sideOf(p) == +1);
    }
    CHECK(d.sideOf(2) == 0);   // the IMU
    CHECK(d.sideOf(21) == 0);  // an empty port
    CHECK(d.sideOf(1) == 0);   // the radio
    // The SIGN comes back exactly as stored — this is the value handed to an adapter.
    CHECK(d.signedPortOf(11) == -11);
    CHECK(d.signedPortOf(12) == 12);
    CHECK(d.signedPortOf(19) == -19);
    CHECK(d.signedPortOf(20) == 20);
    CHECK(d.signedPortOf(2) == 0);
    CHECK(DrivetrainDescription::portMagnitude(-21) == 21);
    CHECK(DrivetrainDescription::portMagnitude(21) == 21);
    CHECK(DrivetrainDescription::portMagnitude(0) == 0);

    // The geometry accessors carry the measured scale, and nothing else.
    CHECK(d.leftGeometry().inchesPerRadian().value() == 1.375);
    CHECK(d.rightGeometry().inchesPerRadian().value() == 1.375);
    CHECK(d.leftGeometry().motorToWheelRatio == 1.0);
    CHECK(d.leftGeometry().valid());
    DrivetrainDescription asym = d;
    asym.right.inchesPerRadian = Length{1.2};
    CHECK(asym.leftGeometry().inchesPerRadian().value() == 1.375);
    CHECK(asym.rightGeometry().inchesPerRadian().value() == 1.2);  // asymmetry is representable

    // The banner says WHICH SOURCE it is looking at — a typed number and a measured one are
    // not the same claim, and the panel must not blur them.
    char line[160];
    d.describe(line, sizeof line);
    CHECK(std::strstr(line, "MEASURED") != nullptr);
    CHECK(std::strstr(line, "tank") != nullptr);
    CHECK(std::strstr(line, "BLUE") != nullptr);
    CHECK(std::strstr(line, "13.25") != nullptr);
    DrivetrainDescription typed = d;
    typed.source = DescriptionSource::TypedTable;
    typed.describe(line, sizeof line);
    CHECK(std::strstr(line, "TYPED") != nullptr);
    CHECK(std::strstr(line, "MEASURED") == nullptr);
    // Bounded: a short buffer truncates and is still NUL-terminated; cap 0 writes nothing.
    char small[10];
    std::memset(small, 'x', sizeof small);
    d.describe(small, 6);
    CHECK(small[5] == '\0');
    CHECK(small[6] == 'x');
    d.describe(nullptr, 0);  // must not dereference
}

// ═══ 3. The calibrated constructor is EXACT ═════════════════════════════════════════
// Bug caught: a factor of two (the field is called wheelRadius and a calibrated geometry stores
// the PRODUCT in it — exactly the place a halve-or-double slips in), or any rounding between the
// two ways of naming the same scale. Bits, never Approx.
TEST_CASE("DriveGeometry 3: fromInchesPerRadian is exact, and agrees with fromDiameter bit for "
          "bit") {
    const double scales[] = {1.375, 0.825, 3.25 / 2.0, 0.1, 12.345678901234, 1e-6, 1e3};
    for (double k : scales) {
        CAPTURE(k);
        const DriveGeometry g = DriveGeometry::fromInchesPerRadian(Length{k});
        CHECK(g.inchesPerRadian().value() == k);   // exactly, not Approx
        CHECK(g.wheelRadius.value() == k);         // the field carries the PRODUCT
        CHECK(g.motorToWheelRatio == 1.0);
        CHECK(g.valid());
    }
    // The two constructors name the same scale: k = d/2 x r, bit for bit.
    struct Pair {
        double diameter;
        double ratio;
    };
    const Pair pairs[] = {{2.75, 1.0}, {2.75, 0.6}, {3.25, 0.5}, {4.0, 1.0 / 3.0}, {2.75, 1.2345}};
    for (const Pair& p : pairs) {
        CAPTURE(p.diameter);
        CAPTURE(p.ratio);
        const DriveGeometry byHand = DriveGeometry::fromDiameter(Length{p.diameter}, p.ratio);
        const DriveGeometry byRobot =
            DriveGeometry::fromInchesPerRadian(byHand.inchesPerRadian());
        CHECK(byRobot.inchesPerRadian().value() == byHand.inchesPerRadian().value());
        CHECK(byRobot.wheelRadius.value() == 0.5 * p.diameter * p.ratio);
        // ... and it is NOT the radius unless the ratio was 1.
        CHECK((byRobot.wheelRadius.value() == byHand.wheelRadius.value())
              == (p.ratio == 1.0));
    }
    // UNSET and nonsense still refuse through the shared valid().
    CHECK_FALSE(DriveGeometry::fromInchesPerRadian(Length{0.0}).valid());
    CHECK_FALSE(DriveGeometry::fromInchesPerRadian(Length{-1.0}).valid());
    CHECK_FALSE(
        DriveGeometry::fromInchesPerRadian(Length{std::numeric_limits<double>::quiet_NaN()})
            .valid());
    // constexpr: a calibrated geometry can be a compile-time constant like a typed one.
    constexpr DriveGeometry kCompileTime = DriveGeometry::fromInchesPerRadian(Length{1.375});
    static_assert(kCompileTime.wheelRadius.value() == 1.375);
}
