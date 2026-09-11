// Adversarial tests for the chassis table's consistency checks (chunk R3b Part 0b, brief §4
// test 4): a signed table must have every entry a real port, no duplicates by absolute value
// on a side or across sides, both sides non-empty, the IMU port not a drive port; an unsigned
// table must carry no sign. A half-signed table is refused.
//
// The table lives in src/ (it is a per-robot composition fact, not library API), and it is
// PROS-free on purpose so this test can include it by relative path and break its checks
// with hand-built tables. The mutation the brief requires ("remove a check") was run against
// these cases and the observed red is in the R3b log. The REAL table this TU sees is the
// bench bot's (no variant define on the host). The tank table cannot be compiled into the
// same TU under its define, so robot two's measured entries are RE-TYPED here from the
// brief's §0 and checked as a table of their own; the header's copy is checked on every
// `make ROBOT=tank` by the drive program's boot-time tableConsistent() call, and the src
// build gate compiles that path.

#include "doctest.h"

#include <cstring>
#include <initializer_list>

#include "../src/chassis_table.hpp"

using shulib::bench::absPort;
using shulib::bench::Cartridge;
using shulib::bench::ChassisTable;
using shulib::bench::kChassis;
using shulib::bench::kMaxPort;
using shulib::bench::portsToString;
using shulib::bench::tableConsistent;
using shulib::bench::tableCount;
using shulib::bench::tableHasPorts;
using shulib::bench::tableSideOf;
using shulib::bench::tableSignedPort;
using shulib::bench::tableSignOf;

namespace {

/// Robot two's table exactly as measured (brief §0): the signed ports, IMU 2, blue.
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

TEST_CASE("chassis table: the bench bot's table (the one this TU compiles) is consistent and "
          "UNSIGNED") {
    // BUG CAUGHT: the real table drifting into an inconsistent state, or growing signs it
    // never measured. On the host neither variant define is set, so kChassis is the bench
    // bot's.
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
