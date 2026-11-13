// Adversarial tests for src/build_selection.hpp — the ONE place the build switches are read (G1 Part
// 2c's final run, 2026-09-26: the team lead's ruling "one chain picks both").
//
// WHAT THIS TU SEES. The host test build sets no build switch, so the header's one chain takes its
// #else: this TU is the bench build's selection — exactly what `make` with no ROBOT builds. So the
// first case pins that branch whole (program, robot, variant beacon, robot beacon). The other seven
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
#include <set>
#include <sstream>
#include <string>
#include <string_view>

#include "build_selection.hpp"
#include "robots/mechanism_table.hpp"
#include "source_pins.hpp"

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

// ═══ "shulib Tune" (TUNE1-B1): the ROBOT-GENERIC row — the host half of the brief's TB12 ═══════════
// The Makefile's MECHANISM_ROBOTS (the ROBOT values PROGRAM=tune accepts) must be exactly the robots
// whose mechanism table is NOT empty — read here from the tables themselves, for every RobotId — and the
// tune row must name no robot. The src build gate's step 0 checks the same from the source text; its
// self-test runs `make ROBOT=bench PROGRAM=tune` and sees the $(error).

namespace {
/// Robot `id`'s ROBOT= value in the Makefile — a case per robot and no default (-Wswitch); null for an
/// integer that is no robot.
constexpr const char* makeValueOf(RobotId id) {
    switch (id) {
        case RobotId::BenchBot:
            return "bench";
        case RobotId::RobotTwo:
            return "tank";
        case RobotId::InventedXDrive:
            return "xdrive";
    }
    return nullptr;
}
}  // namespace

TEST_CASE("build selection: the Makefile's MECHANISM_ROBOTS is exactly the robots whose mechanism table is "
          "not empty, and its PROGRAM=tune row names no robot") {
    // BUG CAUGHT: a robot-two-only Tune row (robot one's plumbing would need a Tune row of its own — the
    // brief's re-check #10), or a robot listed whose table is empty (a Tune with nothing to tune).
    // EVERY RobotId (its enumerators are 0, 1, 2, ... — none states a value), each by its ROBOT= value:
    // a robot added to RobotId without a case below is -Wswitch, a host build ERROR (-Werror) — so a robot
    // that registers a table (G1's robot one) is read here with no list to keep in step by hand
    std::set<std::string> withMechanisms;
    int robotsSeen = 0;
    for (int k = 0; k < 64; ++k) {
        const char* makeValue = makeValueOf(static_cast<RobotId>(k));
        if (makeValue == nullptr) {
            continue;
        }
        ++robotsSeen;
        if (shulib::bench::mechanismTableFor(static_cast<RobotId>(k)).count > 0) {
            withMechanisms.insert(makeValue);
        }
    }
    CHECK(robotsSeen >= 3);
    const std::string makefile = source_pins::readFile("Makefile");
    const std::size_t at = makefile.find("\nMECHANISM_ROBOTS:=");
    REQUIRE(at != std::string::npos);
    CHECK(makefile.find("\nMECHANISM_ROBOTS:=", at + 1) == std::string::npos);  // set once
    const std::size_t end = makefile.find('\n', at + 1);
    std::istringstream listed{makefile.substr(at + 19, end - at - 19)};
    std::set<std::string> listedRobots;
    for (std::string w; listed >> w;) {
        listedRobots.insert(w);
    }
    CHECK(listedRobots == withMechanisms);
    CHECK(withMechanisms.count("tank") == 1);  // robot two's claws: the one robot with mechanisms today
    // the tune row: from its ifeq to its append, comments aside, no comparison of $(ROBOT) with a name
    const std::size_t row = makefile.find("else ifeq ($(PROGRAM),tune)");
    REQUIRE(row != std::string::npos);
    const std::size_t rowEnd = makefile.find("override EXTRA_CXXFLAGS+=-DSHULIB_PROGRAM_TUNE", row);
    REQUIRE(rowEnd != std::string::npos);
    std::istringstream lines{makefile.substr(row, rowEnd - row)};
    for (std::string line; std::getline(lines, line);) {
        if (line.rfind("#", 0) == 0) {
            continue;
        }
        CAPTURE(line);
        CHECK(line.find("${ROBOT}") == std::string::npos);
        if (line.find("$(error ") != std::string::npos) {
            continue;  // the refusal's own words name the ROBOT it refuses
        }
        // ANY other reading of $(ROBOT) but the one list filter names a robot: `ifneq ($(ROBOT),tank)`,
        // `ifeq ($(filter tank,$(ROBOT)),)`, `ifneq "$(ROBOT)" "tank"`
        std::string rest = line;
        const std::string allowed = "$(filter $(ROBOT),$(MECHANISM_ROBOTS))";
        if (const std::size_t f = rest.find(allowed); f != std::string::npos) {
            rest.erase(f, allowed.size());
        }
        CHECK(rest.find("$(ROBOT)") == std::string::npos);
    }
    CHECK(makefile.substr(row, rowEnd - row).find("$(filter $(ROBOT),$(MECHANISM_ROBOTS))") != std::string::npos);
}

TEST_CASE("build selection: Program::Tune is its own program, and the host's build is not it") {
    // BUG CAUGHT: the Tune enumerator aliased onto another program (a Tune upload running another
    // program's entry points), or the host's #else silently taking a Tune branch.
    CHECK(Program::Tune != Program::BenchTests);
    CHECK(Program::Tune != Program::Competition);
    CHECK(Program::Tune != Program::Drive);
    CHECK(Program::Tune != Program::AutonTester);
    CHECK(Program::Tune != Program::Calibrate);
    CHECK(Program::Tune != Program::InventedXDrive);
    CHECK(kProgram != Program::Tune);
    // the tune program file's backstop holds on the host (bench: not Tune, so nothing is asserted)
    CHECK((kProgram != Program::Tune || shulib::bench::kMechanisms.count > 0));
}
