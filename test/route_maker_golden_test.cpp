// route_maker_golden_test.cpp — THE ROUTE BUILDER'S EXPORT COMPILES AND RUNS (chunk RMP1-A1, 2026-11-14;
// the RMP1 brief §2.2 "The export compiles AND runs", GL0–GL19 as A.7 cuts them for A1, CV1, [R8]).
//
// Every golden in test/goldens/route_maker/ is a route (`<name>.route.json`, a project file) and the
// C++ the page's OWN exporter wrote for it (`<name>.cpp`, kept fresh by the page's node test
// export.test.mjs) — except gl0_empty.cpp, §6.3's contract text kept by hand, which the exporter is held
// to. Each export is compiled HERE with the suite's flags — `#include`d inside a namespace
// that makes its names resolve the way §6.3's skills-route file does (the library's namespaces, the unit
// literals, the BUILDING robot's declarations) — and RUN through competition::runAuton, its FIELD start
// seated, on the host test plant (route_maker_rig.hpp). Behaviour is asserted ONLY on robot x, whose
// numbers this test owns (R8); robots one and two are SMOKE runs (compiled, run, nothing thrown).
//
// Also here: the committed fixture descriptions (robots/x|y|z.json) and the curve golden (curves.txt)
// equal what the C++ makes now — the page's node tests read those files, so a library change that moves
// them turns THIS red until they are re-made (`shulib_route_maker describe` / `curves`).
//
// Host test plant only: nothing here has run on a robot.

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

#include "doctest.h"
#include "route_maker_curves.hpp"
#include "route_maker_rig.hpp"
#include "route_maker_robots.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"

// ── the exports, each in its own namespace (the shared file's names: §6.3 item 2) ─────────────────────
#define RM_FIXTURE_X                       \
    using namespace shulib;                \
    using namespace shulib::units::literals; \
    using namespace mech_fixture::robot_x;

namespace rm_gl0 { RM_FIXTURE_X
#include "goldens/route_maker/gl0_empty.cpp"
}
namespace rm_gl1 { RM_FIXTURE_X
#include "goldens/route_maker/gl1_drive_turn_first.cpp"
}
namespace rm_gl1b { RM_FIXTURE_X
#include "goldens/route_maker/gl1b_drive_off_line.cpp"
}
namespace rm_gl2 { RM_FIXTURE_X
#include "goldens/route_maker/gl2_reverse_and_turns.cpp"
}
namespace rm_gl3 { RM_FIXTURE_X
#include "goldens/route_maker/gl3_curves.cpp"
}
namespace rm_gl3b { RM_FIXTURE_X
#include "goldens/route_maker/gl3b_curve_16_points.cpp"
}
namespace rm_gl4 { RM_FIXTURE_X
#include "goldens/route_maker/gl4_mechanisms.cpp"
}
namespace rm_gl12s { RM_FIXTURE_X
#include "goldens/route_maker/gl12_fail_stop.cpp"
}
namespace rm_gl12c { RM_FIXTURE_X
#include "goldens/route_maker/gl12_fail_carry_on.cpp"
}
namespace rm_gl12k { RM_FIXTURE_X
#include "goldens/route_maker/gl12_fail_skip.cpp"
}
namespace rm_gl13 { RM_FIXTURE_X
#include "goldens/route_maker/gl13_wait_past_the_cut.cpp"
}
namespace rm_gl18a { RM_FIXTURE_X
#include "goldens/route_maker/gl18a_move_never_homed.cpp"
}
namespace rm_gl18b { RM_FIXTURE_X
#include "goldens/route_maker/gl18b_move_while_homing.cpp"
}
namespace rm_gl18c { RM_FIXTURE_X
#include "goldens/route_maker/gl18c_home_then_move.cpp"
}
namespace rm_gl18d { RM_FIXTURE_X
#include "goldens/route_maker/gl18d_home_while_driving.cpp"
}
namespace rm_gl19 { RM_FIXTURE_X
#include "goldens/route_maker/gl19_caps.cpp"
}
// Robot one's and robot two's: their REAL declarations (src/robots/mechanism_table.hpp), as the shared file
// sees them on each robot's build (robot one's names `facts` too: its ledger).
namespace rm_smoke_one {
using namespace shulib;
using namespace shulib::units::literals;
using namespace shulib::bench::robot_one;
#include "goldens/route_maker/smoke_robot_one.cpp"
}
namespace rm_smoke_two {
using namespace shulib;
using namespace shulib::units::literals;
using namespace shulib::bench::robot_two;
#include "goldens/route_maker/smoke_robot_two.cpp"
}
#undef RM_FIXTURE_X

namespace {

using shulib::competition::AutonEntry;
using shulib::competition::AutonRunOutcome;
using XStack = rm_rig::Stack<rm_rig::XFakes>;

constexpr double kPi = 3.14159265358979323846;

[[nodiscard]] std::string readFile(const std::string& path) {
    std::ifstream in{path, std::ios::binary};
    REQUIRE_MESSAGE(in.good(), "cannot open " << path);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}
[[nodiscard]] std::string golden(const char* rel) {
    return readFile(std::string{SHULIB_SOURCE_DIR} + "/test/goldens/route_maker/" + rel);
}

}  // namespace

TEST_CASE("RMP1 GL0: the EMPTY route region (§6.3, byte for byte) compiles and runs as a skills auton with no steps") {
    const AutonEntry e = rm_gl0::skillsRoute();
    CHECK(std::string{e.name} == "skills");
    CHECK(e.kind == shulib::competition::AutonKind::Skills);
    CHECK(e.start.mode() == shulib::competition::StartPose::Mode::Relative);
    CHECK(e.endActionAt.value() == 57.0);
    CHECK(e.hardStopAt.value() == 59.0);
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 0);
    CHECK(r.routine.ok);
    CHECK_FALSE(r.guard.scoringCut);
}

TEST_CASE("RMP1 GL1: a drive with 'turn to face it first' (R6) -- the face, then the leg, and the robot arrives") {
    const AutonEntry e = rm_gl1::skillsRoute();
    REQUIRE(e.start.mode() == shulib::competition::StartPose::Mode::Field);
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 2);  // r.face, r.driveTo
    CHECK(r.routine.completed == 2);
    CHECK(r.routine.ok);
    const auto t = s.truth();
    // the point the golden names: 24 in from (-36, -55.7) at a bearing of 45 deg
    CHECK(std::hypot(t.x().value() - -19.029, t.y().value() - -38.729) < 1.0);
}

TEST_CASE("RMP1 GL1b: the SAME leg 10 deg off its line, NOT turned first, stops short -- RC24's twin (P8)") {
    const AutonEntry e = rm_gl1b::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 1);
    CHECK_FALSE(r.routine.ok);
    CHECK(r.routine.stoppedAt == 1);
    CHECK(r.routine.cause == shulib::chassis::RoutineStopCause::MotionFailed);
    const auto t = s.truth();
    CHECK(std::hypot(t.x().value() - -12.365, t.y().value() - -51.532) > 0.3);
}

TEST_CASE("RMP1 GL2: reverse with its back turned first, a turn to a heading, face and face away") {
    const AutonEntry e = rm_gl2::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 5);  // faceAway, reverseTo, turnTo, face, faceAway
    CHECK(r.routine.completed == 5);
    CHECK(r.routine.ok);
    const auto t = s.truth();
    CHECK(std::hypot(t.x().value() - -48.0, t.y().value() - -40.0) < 1.0);
    // the last step: its BACK toward (0, -60) -- the heading is the bearing to it + 180 deg
    const double away = std::atan2(-60.0 - t.y().value(), 0.0 - t.x().value()) + kPi;
    CHECK(std::abs(std::remainder(t.heading().radians() - away, 2.0 * kPi)) < 2.0 * kPi / 180.0);
}

TEST_CASE("RMP1 GL3: curves -- forwards with an end heading, then BACKWARDS (the heading is the nose); and 16 points") {
    {
        const AutonEntry e = rm_gl3::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.steps == 2);
        CHECK(r.routine.ok);
        const auto t = s.truth();
        CHECK(std::hypot(t.x().value() - -34.0, t.y().value() - -48.0) < 1.0);
        CHECK(std::abs(std::remainder(t.heading().radians(), 2.0 * kPi)) < 3.0 * kPi / 180.0);
    }
    {
        const AutonEntry e = rm_gl3b::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.steps == 1);
        CHECK(r.routine.ok);
        const auto t = s.truth();
        CHECK(std::hypot(t.x().value() - 56.5, t.y().value() - (-50.0 + 3.0 * std::sin(15.0 / 4.0))) < 1.0);
    }
}

TEST_CASE("RMP1 GL4: every mechanism step A1 writes, on robot x's every kind (blocking and started, waitUntilDone, a home first)") {
    const AutonEntry e = rm_gl4::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 8);
    CHECK(r.routine.completed == 8);
    CHECK(r.routine.ok);
    CHECK(std::abs(s.fakes.liftInches() - 8.0) < 0.5);  // lift["low"] = 8 in
    CHECK(std::abs(s.fakes.armDeg() - 95.0) < 3.0);     // arm["score"] = 95 deg
    // hood "up" (High, High) was reached; after the run the guard safes it back to "down", so it is the
    // service's verdict that is read, not the lines' end state
    CHECK(s.count("MECH", "'hood': SUCCEEDED") == 1);
    CHECK(s.count("MECH", "'intake': SUCCEEDED") == 2);
}

TEST_CASE("RMP1 GL12: one forced failure under stop / carry on / skip -- exactly the counts each predicts, never a throw") {
    {
        const AutonEntry e = rm_gl12s::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK_FALSE(r.routine.ok);
        CHECK(r.routine.stoppedAt == 1);
        CHECK(r.routine.completed == 0);
        CHECK(r.routine.failedOn == 0);
        CHECK(r.routine.jumped == 0);
    }
    {
        const AutonEntry e = rm_gl12c::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.stoppedAt == 0);
        CHECK(r.routine.failedOn == 1);
        CHECK(r.routine.firstFailedAt == 1);
        CHECK(r.routine.completed == 2);  // the turn and the wait
        CHECK(r.routine.jumped == 0);
    }
    {
        const AutonEntry e = rm_gl12k::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.stoppedAt == 0);
        CHECK(r.routine.failedOn == 1);
        CHECK(r.routine.jumped == 1);     // the turn, passed over
        CHECK(r.routine.completed == 1);  // the labelled wait it landed on
        // the turn never ran: the robot still faces where it started (0 deg)
        CHECK(std::abs(std::remainder(s.truth().heading().radians(), 2.0 * kPi)) < 5.0 * kPi / 180.0);
    }
}

TEST_CASE("RMP1 GL13: a wait across the 57 s cut is the RUN GUARD's pause -- it ends AT the cut, the hard stop never fires") {
    const AutonEntry e = rm_gl13::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.guard.scoringCut);
    CHECK_FALSE(r.guard.floorFired);  // a frozen r.pause(60_s) would run past 59 s: the floor fires
    CHECK(r.guard.endActionRan);
    CHECK(r.routine.stoppedAt == 1);
}

TEST_CASE("RMP1 GL18: the home rule's twins (RC25) -- a move before a FINISHED home stops the route NOT_HOMED") {
    {   // no home at all, the lift off its switch at boot
        const AutonEntry e = rm_gl18a::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, true};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.stoppedAt == 1);
        CHECK(r.routine.mechanismVerdict == shulib::manipulation::MechanismOutcome::NotHomed);
    }
    {   // a startHome, then a move: the move REPLACES the running home
        const AutonEntry e = rm_gl18b::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, true};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.stoppedAt == 2);
        CHECK(r.routine.mechanismVerdict == shulib::manipulation::MechanismOutcome::NotHomed);
    }
    {   // the page's fix "home it first": a blocking home, then the move -- it gets there
        const AutonEntry e = rm_gl18c::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, true};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.ok);
        CHECK(r.routine.completed == 2);
        CHECK(std::abs(s.fakes.liftInches() - 8.0) < 0.5);
    }
    {   // the page's other fix "home it while the first leg drives": startHome, the leg, waitUntilDone, the move
        const AutonEntry e = rm_gl18d::skillsRoute();
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, true};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.ok);
        CHECK(r.routine.completed == 4);
        CHECK(std::abs(s.fakes.liftInches() - 8.0) < 0.5);
    }
}

TEST_CASE("RMP1 GL19: the caps' spellings (R10) -- N_in / 1_s and AngularVelocity{N * kPi / 180}, compiled, run, read back; and both tolerances") {
    // The drive carries BOTH caps (30 in/s, and 270 deg/s -- past 180, so a cap spelled through _deg would wrap to
    // -90 deg/s and be refused: RM26's oracle) and an arrival tolerance (2 in); the turn carries a 90 deg/s cap the
    // plant can show and a heading tolerance (5 deg) -- every motion option the inspector offers, compiled and run.
    using namespace shulib::units::literals;
    CHECK((30_in / 1_s).value() == 30.0);
    CHECK(shulib::units::AngularVelocity{270 * shulib::math::Angle::kPi / 180.0}.value() == doctest::Approx(4.712389).epsilon(1e-6));
    // never through _deg: an Angle wraps (P5)
    CHECK((270_deg).radians() == doctest::Approx(-kPi / 2.0));
    const AutonEntry e = rm_gl19::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.ok);
    CHECK(s.world.peakSpeed <= 30.0 * 1.1);                    // the drive's 30 in/s cap, read back from the plant
    CHECK(s.world.peakTurn <= 90.0 * kPi / 180.0 * 1.1);       // the turn's 90 deg/s cap
    CHECK(s.world.peakTurn >= 90.0 * kPi / 180.0 * 0.5);       // ...and it did turn
    // the tolerances are the export's `.positionTolerance = 2_in` and `.headingTolerance = 5_deg`: the robot settles
    // inside them (a settle is judged on the estimate; the plant's truth is within a hair of it here)
    const std::string code = golden("gl19_caps.cpp");
    CHECK(code.find(".positionTolerance = 2_in") != std::string::npos);
    CHECK(code.find(".headingTolerance = 5_deg") != std::string::npos);
    const auto t = s.truth();
    CHECK(std::hypot(t.x().value() - 0.0, t.y().value() - -55.7) < 2.0 + 0.5);
    CHECK(std::abs(std::remainder(t.heading().radians() - kPi / 2.0, 2.0 * kPi)) < (5.0 + 1.0) * kPi / 180.0);
}

TEST_CASE("RMP1 smoke: the exports on robot ONE's and robot TWO's real lists (the twins) compile and run without a throw") {
    {
        const AutonEntry e = rm_smoke_one::skillsRoute();
        rm_rig::Stack<rm_rig::OneFakes> s{rm_rig::startOf(e), shulib::bench::robot_one::kRobotOneMechanisms};
        const auto r = s.run(e);
        INFO(s.transcript());
        CHECK(r.outcome == AutonRunOutcome::RoutineRan);  // a verdict on placeholder facts is not asserted (R8)
    }
    {
        const AutonEntry e = rm_smoke_two::skillsRoute();
        rm_rig::Stack<rm_rig::TwoFakes> s{rm_rig::startOf(e), shulib::bench::robot_two::kRobotTwoMechanisms};
        const auto r = s.run(e);
        INFO(s.transcript());
        CHECK(r.outcome == AutonRunOutcome::RoutineRan);
    }
}

TEST_CASE("RMP1: the committed fixture descriptions and the curve golden equal what the C++ makes now") {
    for (const char* id : {"x", "y", "z"}) {
        const auto src = route_maker::robotSource(id);
        REQUIRE(src.has_value());
        INFO("test/goldens/route_maker/robots/" << id << ".json is stale: re-make it with "
             "`build/test/shulib_route_maker describe --robot " << id << "`, then read the diff");
        CHECK(golden((std::string{"robots/"} + id + ".json").c_str()) == route_maker::describe(*src));
    }
    INFO("test/goldens/route_maker/curves.txt is stale: re-make it with `build/test/shulib_route_maker curves`, then read the diff");
    CHECK(golden("curves.txt") == route_maker::curvesTable());
}

TEST_CASE("RMP1: robot one's description marks its cascade levels as its facts ledger's, and names every declaration") {
    const auto one = route_maker::robotSource("one");
    REQUIRE(one.has_value());
    const std::string j = route_maker::describe(*one);
    CHECK(j.find("\"ident\": \"cascade\"") != std::string::npos);
    CHECK(j.find("\"homing\": \"stall\"") != std::string::npos);
    // every NAMED level of the facts ledger is marked as the ledger's (none while the cascade is manual: levels NONE)
    for (std::size_t i = 0; i < shulib::mechanism::kMaxPresets; ++i) {
        if (shulib::bench::robot_one_facts::lv(i).name == nullptr) {
            break;
        }
        CHECK(j.find("\"ledger\": " + std::to_string(i)) != std::string::npos);
    }
    // an OWED ledger is described as owed (the page then shows no height): exactly while the fact is a placeholder
    CHECK((j.find("\"owed\": true") != std::string::npos) == !shulib::bench::robot_one_facts::kCascadeLevels.typed);
    const auto two = route_maker::robotSource("two");
    REQUIRE(two.has_value());
    const std::string t = route_maker::describe(*two);
    CHECK(t.find("\"ident\": \"cascade\"") != std::string::npos);  // robot one's twin (ROB1-BT)
    CHECK(t.find("\"ident\": \"leftToggle\"") != std::string::npos);
    // owed marks ride on NAMED levels only: robot two's placeholder is robot one's typed "none yet"
    CHECK((t.find("\"owed\": true") != std::string::npos) ==
          (!shulib::bench::robot_two_facts::kCascadeLevels.typed && shulib::bench::robot_two_facts::lv(0).name != nullptr));
    // RG1: no path in a description (the published page must not carry a laptop's directory)
    for (const std::string& d : {j, t}) {
        CHECK(d.find("/home/") == std::string::npos);
        CHECK(d.find(".hpp") == std::string::npos);
    }
    CHECK_FALSE(route_maker::robotSource("three").has_value());
}

TEST_CASE("RMP1: each robot's description reads ITS OWN facts ledger's levels and owed mark (robot two's own since ROB1-BT)") {
    // BUG CAUGHT: robot two described from robot one's ledger (or none) — the page would offer robot one's levels,
    // or none, on robot two, and say they are measured while robot two's are still owed.
    const auto one = route_maker::robotSource("one");
    const auto two = route_maker::robotSource("two");
    REQUIRE(one.has_value());
    REQUIRE(two.has_value());
    REQUIRE(one->ledgerLevel != nullptr);
    REQUIRE(two->ledgerLevel != nullptr);
    for (std::size_t i = 0; i < shulib::mechanism::kMaxPresets; ++i) {
        CHECK(one->ledgerLevel(i) == shulib::bench::robot_one_facts::lv(i).name);
        CHECK(two->ledgerLevel(i) == shulib::bench::robot_two_facts::lv(i).name);
    }
    CHECK(one->ledgerOwed == !shulib::bench::robot_one_facts::kCascadeLevels.typed);
    CHECK(two->ledgerOwed == !shulib::bench::robot_two_facts::kCascadeLevels.typed);
    CHECK(two->ledgerOwed);  // robot two's levels are owed until read off it
}

TEST_CASE("RMP1: a robot that builds the shared route and declares a cascade marks its levels as its facts ledger's (the tripwire for robot two)") {
    // The shared skills route moves the cascade to LEDGER levels (`cascade[facts::lv(i).name]`); the page lets it
    // only on a robot whose description marks them (RC18) — both robots since ROB1-BT (route_maker_robots.hpp),
    // never a silent "every level refused on robot two".
    for (const char* id : {"one", "two"}) {
        const auto src = route_maker::robotSource(id);
        REQUIRE(src.has_value());
        const std::string d = route_maker::describe(*src);
        const std::size_t at = d.find("\"ident\": \"cascade\"");
        if (at == std::string::npos) {
            continue;
        }
        const std::size_t presets = d.find("\"presets\"", at);
        if (d.compare(presets, std::string{"\"presets\": []"}.size(), "\"presets\": []") == 0) {
            continue;  // no levels named yet (a manual cascade): nothing to mark
        }
        const std::size_t end = d.find('}', presets);
        INFO("robot " << id << "'s cascade is described without its facts ledger: give robotSource(\"" << id
             << "\") its .ledgerLevel (test/route_maker_robots.hpp)");
        CHECK(d.substr(at, end - at).find("\"ledger\": 0") != std::string::npos);
    }
}
