// Adversarial tests for src/build_selection.hpp — the ONE place the build switches are read (G1 Part
// 2c's final run, 2026-09-26: the team lead's ruling "one chain picks both").
//
// WHAT THIS TU SEES. The host test build sets no build switch, so the header's one chain takes its
// #else: this TU is the bench build's selection — exactly what `make` with no ROBOT builds. So the
// first case pins that branch whole (program, robot, variant beacon, robot beacon). The other five
// branches are compiled only in the robot builds; what checks them there is the src build gate
// (tools/src_build_gate.py): step 0 reads every branch against the gate's own per-build table, and
// step 6 finds each build's two beacons — and no other build's — in its linked package.
//
// WHAT THIS TU CAN CHECK FOR EVERY ROBOT: robotBeacon(), which derives the ROBOT beacon from the enum.
// The gate matches beacons byte for byte with their NUL terminator, and its ROBOT_BEACONS table is
// checked against robotBeacon()'s cases by step 0; here the texts are pinned as literals, and the
// properties the gate relies on — one beacon per robot, none mistakable for another robot's or for a
// variant beacon — are tried against every pair.

#include "doctest.h"

#include <array>
#include <cstring>
#include <string_view>

#include "build_selection.hpp"

using shulib::bench::kProgram;
using shulib::bench::kRobotId;
using shulib::bench::kRobotBeacon;
using shulib::bench::kVariantBeacon;
using shulib::bench::Program;
using shulib::bench::robotBeacon;
using shulib::bench::RobotId;

namespace {

constexpr std::array<RobotId, 3> kEveryRobot = {RobotId::BenchBot, RobotId::RobotTwo,
                                                RobotId::InventedXDrive};

}  // namespace

// The host's branch, at compile time too: a host build that silently took another branch (a
// switch leaking into the test build's flags) fails to compile here, before any case runs.
static_assert(kProgram == Program::BenchTests && kRobotId == RobotId::BenchBot,
              "the host sets no build switch, so it must take the chain's #else (the bench build)");

TEST_CASE("build selection: the host takes the chain's #else -- the bench build's program, robot "
          "and both beacons, together") {
    // BUG CAUGHT: an #else that sets one of the three to another build's value (the bench build
    // running the tester over robot two's tables, say) — every branch must set all three, and the
    // #else is the one branch the host can see.
    CHECK(kProgram == Program::BenchTests);
    CHECK(kRobotId == RobotId::BenchBot);
    CHECK(std::string_view{kVariantBeacon} == "shulib-robot-variant=bench");
    CHECK(std::string_view{kRobotBeacon} == "shulib-robot=bench-bot");
    // kRobotBeacon IS robotBeacon(kRobotId) — derived, never typed beside it.
    CHECK(std::string_view{kRobotBeacon} == robotBeacon(kRobotId));
    // The beacon array carries exactly its text and the terminator the gate matches on.
    CHECK(sizeof kVariantBeacon == std::strlen("shulib-robot-variant=bench") + 1);
}

TEST_CASE("build selection: robotBeacon() names every robot, exactly as the gate's ROBOT_BEACONS "
          "table expects") {
    // BUG CAUGHT: a robot beacon renamed here and not in tools/src_build_gate.py (every build of
    // that robot would then fail the gate for a missing beacon; step 0 also compares the two), or
    // two robots given one beacon (a tank build carrying the bench bot's tables would then pass).
    CHECK(std::string_view{robotBeacon(RobotId::BenchBot)} == "shulib-robot=bench-bot");
    CHECK(std::string_view{robotBeacon(RobotId::RobotTwo)} == "shulib-robot=robot-two");
    CHECK(std::string_view{robotBeacon(RobotId::InventedXDrive)} == "shulib-robot=invented-xdrive");
}

TEST_CASE("build selection: no robot beacon can be mistaken for another robot's, or for a variant "
          "beacon") {
    // BUG CAUGHT: a beacon that the gate's NUL-terminated search would find inside another robot's
    // — it searches for `<beacon>\0`, which cannot match a PREFIX of a longer beacon but DOES match
    // an equal one or a SUFFIX of one — or a robot beacon spelled like a variant beacon
    // (`shulib-robot-variant=...`), which the gate would read as a build label.
    for (RobotId a : kEveryRobot) {
        const char* const beaconA = robotBeacon(a);
        CAPTURE(beaconA);
        const std::string_view sa{beaconA};
        CHECK(sa.rfind("shulib-robot=", 0) == 0);
        CHECK(sa.find("shulib-robot-variant=") == std::string_view::npos);
        CHECK(sa.size() > std::strlen("shulib-robot="));
        for (RobotId b : kEveryRobot) {
            if (a == b) continue;
            const char* const beaconB = robotBeacon(b);
            CAPTURE(beaconB);
            const std::string_view sb{beaconB};
            // sa + NUL inside sb + NUL <=> sa is a suffix of sb (equality included)
            CHECK_FALSE((sb.size() >= sa.size() && sb.substr(sb.size() - sa.size()) == sa));
        }
    }
}
