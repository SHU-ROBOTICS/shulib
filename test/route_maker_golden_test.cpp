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

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "doctest.h"
#include "mirrored_pair_check.hpp"
#include "route_maker_curves.hpp"
#include "route_maker_rig.hpp"
#include "route_maker_robots.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/kinematics/tank.hpp"
#include "team/autons.hpp"
#include "team/robot_one_autons.hpp"
#include "team/skills_route.hpp"

// ── the exports, each in its own namespace (the shared file's names: §6.3 item 2) ─────────────────────
#define RM_FIXTURE_X                       \
    using namespace shulib;                \
    using namespace shulib::units::literals; \
    using namespace mech_fixture::robot_x; \
    using shulib::bench::TeamRoute;        \
    using shulib::bench::Mirror;           \
    using shulib::bench::Together;

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
namespace rm_wg8 { RM_FIXTURE_X
#include "goldens/route_maker/wg8_distinct_numbers.cpp"
}
namespace rm_gl20 { RM_FIXTURE_X
#include "goldens/route_maker/gl20_named_routes.cpp"
}
namespace rm_gl21 { RM_FIXTURE_X
#include "goldens/route_maker/gl21_lift_heights.cpp"
}
namespace rm_gl22 { RM_FIXTURE_X
#include "goldens/route_maker/gl22_small_moves.cpp"
}
namespace rm_gl23 { RM_FIXTURE_X
#include "goldens/route_maker/gl23_pair_near_the_diagonal.cpp"
}
namespace rm_gl24 { RM_FIXTURE_X
#include "goldens/route_maker/gl24_leave_gently.cpp"
}
// Robot one's and robot two's: their REAL declarations (src/robots/mechanism_table.hpp), as the shared file
// sees them on each robot's build (robot one's names `facts` too: its ledger).
namespace rm_smoke_one {
using namespace shulib;
using namespace shulib::units::literals;
using namespace shulib::bench::robot_one;
using shulib::bench::TeamRoute;
using shulib::bench::Mirror;
using shulib::bench::Together;
#include "goldens/route_maker/smoke_robot_one.cpp"
}
namespace rm_smoke_two {
using namespace shulib;
using namespace shulib::units::literals;
using namespace shulib::bench::robot_two;
using shulib::bench::TeamRoute;
using shulib::bench::Mirror;
using shulib::bench::Together;
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

TEST_CASE("RMP1 WG8: the golden whose every pair of numbers differs (the preview engine's WG1/WG2, RMP1-A2e) compiles and every step runs") {
    const AutonEntry e = rm_wg8::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 25);    // 22 page steps, three of them drives turned first (two calls each)
    CHECK(r.routine.stoppedAt == 0);  // every step carries on or skips forward: none is left unrun by a stop
    CHECK(r.routine.jumped == 0);
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
    CHECK_FALSE(two->ledgerOwed);  // robot two's levels: robot one's, the same build (2026-11-20)
}

namespace rmp1m {
/// One mechanism's line of a description (each mechanism is one line), from its "wiring" on.
std::string wiringOf(const std::string& d, const std::string& ident) {
    const std::size_t at = d.find("\"ident\": \"" + ident + "\"");
    REQUIRE(at != std::string::npos);
    const std::size_t w = d.find("\"wiring\": ", at);
    REQUIRE(w < d.find('\n', at));
    std::string s = d.substr(w, d.find('\n', w) - w);
    return s.ends_with(',') ? s.substr(0, s.size() - 1) : s;
}
/// One mechanism's whole line.
std::string lineOf(const std::string& d, const std::string& ident) {
    const std::size_t at = d.find("\"ident\": \"" + ident + "\"");
    REQUIRE(at != std::string::npos);
    const std::size_t from = d.rfind('\n', at) + 1;
    return d.substr(from, d.find('\n', at) - from);
}
template <class Facts>
std::string motorJson(int port, bool portOwed, const Facts& dir, bool first) {
    const auto& m = first ? dir.value().first : dir.value().second;
    return "{\"port\": " + (portOwed ? std::string{"null"} : std::to_string(port)) + ", \"reversed\": "
           + (!dir.typed ? "null" : m.reversed ? "true" : "false") + "}";
}
/// A one-line piston's wiring from its facts: its line, and its two positions' levels (`second` is the one the level fact names).
template <class Line, class Level>
std::string pistonJson(const Line& line, const Level& level, const char* first, const char* second) {
    const auto lv = [](bool high) { return std::string{high ? "[\"high\"]" : "[\"low\"]"}; };
    return "\"wiring\": {\"motors\": [], \"lines\": [" + (line.typed ? "\"" + std::string(1, line.value()) + "\"" : std::string{"null"}) + "], \"levels\": "
           + (level.typed ? "{\"" + std::string{first} + "\": " + lv(!level.value()) + ", \"" + second + "\": " + lv(level.value()) + "}" : std::string{"null"}) + "}";
}
}  // namespace rmp1m

TEST_CASE("RMP1-M: each mechanism's wiring is its robot's facts -- each motor's port and direction, each piston's line and levels; an owed one null") {
    // BUG CAUGHT (mutant): a card showing another mechanism's port, or a placeholder as a fact. Built here from the FACTS
    // (an independent path from describe()'s, which reads the table).
    auto check = [&](const char* id, const auto& f, int leftPort) {
        const std::string d = route_maker::describe(*route_maker::robotSource(id));
        INFO("robot " << std::string{id});
        CHECK(rmp1m::wiringOf(d, "cascade") == "\"wiring\": {\"motors\": ["
              + rmp1m::motorJson(f.kCascadePorts.value().first, !f.kCascadePorts.typed, f.kCascadeDirections, true) + ", "
              + rmp1m::motorJson(f.kCascadePorts.value().second, !f.kCascadePorts.typed, f.kCascadeDirections, false) + "], \"lines\": []}}");
        auto toggle = [&](int port, bool portOwed, const auto& dir) {
            return "\"wiring\": {\"motors\": [{\"port\": " + (portOwed ? std::string{"null"} : std::to_string(port)) + ", \"reversed\": "
                   + (!dir.typed ? "null" : dir.value().reversed ? "true" : "false") + "}], \"lines\": []}}";
        };
        CHECK(rmp1m::wiringOf(d, "rightToggle") == toggle(f.kRightTogglePort.value(), !f.kRightTogglePort.typed, f.kRightToggleDirection));
        CHECK(rmp1m::wiringOf(d, "leftToggle")
              == toggle(leftPort, !f.kRightTogglePort.typed || !f.kSideTogglePorts.typed, f.kLeftToggleDirection));
        CHECK(rmp1m::wiringOf(d, "claw") == rmp1m::pistonJson(f.kClawLine, f.kClawOpenLevel, "closed", "open") + "}");
        CHECK(rmp1m::wiringOf(d, "flipper") == rmp1m::pistonJson(f.kFlipperLine, f.kFlipperOpenLevel, "closed", "open") + "}");
        CHECK(rmp1m::wiringOf(d, "ramp") == rmp1m::pistonJson(f.kRampLine, f.kRampDownLevel, "up", "down") + "}");
        // a piston's start is its unpowered position: null while its levels are owed (never robot one's guess)
        CHECK((rmp1m::lineOf(d, "claw").find("\"start\": null") != std::string::npos) == !f.kClawOpenLevel.typed);
        return d;
    };
    namespace one = shulib::bench::robot_one_facts;
    namespace two = shulib::bench::robot_two_facts;
    struct F1 { decltype(one::kCascadePorts) kCascadePorts; decltype(one::kCascadeDirections) kCascadeDirections; decltype(one::kRightTogglePort) kRightTogglePort;
                decltype(one::kSideTogglePorts) kSideTogglePorts; decltype(one::kRightToggleDirection) kRightToggleDirection;
                decltype(one::kLeftToggleDirection) kLeftToggleDirection; decltype(one::kClawLine) kClawLine; decltype(one::kFlipperLine) kFlipperLine;
                decltype(one::kRampLine) kRampLine; decltype(one::kClawOpenLevel) kClawOpenLevel; decltype(one::kFlipperOpenLevel) kFlipperOpenLevel;
                decltype(one::kRampDownLevel) kRampDownLevel; };
    const std::string d1 = check("one", F1{one::kCascadePorts, one::kCascadeDirections, one::kRightTogglePort, one::kSideTogglePorts, one::kRightToggleDirection,
                                           one::kLeftToggleDirection, one::kClawLine, one::kFlipperLine, one::kRampLine, one::kClawOpenLevel,
                                           one::kFlipperOpenLevel, one::kRampDownLevel}, one::leftTogglePort());
    const std::string d2 = check("two", F1{two::kCascadePorts, two::kCascadeDirections, two::kRightTogglePort, two::kSideTogglePorts, two::kRightToggleDirection,
                                           two::kLeftToggleDirection, two::kClawLine, two::kFlipperLine, two::kRampLine, two::kClawOpenLevel,
                                           two::kFlipperOpenLevel, two::kRampDownLevel}, two::leftTogglePort());
    // robot one has every wiring fact; robot two's toggles' directions and pistons' levels are still OWED today (2026-11-20)
    CHECK(d1.find("null") == std::string::npos);
    CHECK((rmp1m::wiringOf(d2, "rightToggle").find("\"reversed\": null") != std::string::npos) == !two::kRightToggleDirection.typed);
    // a fixture robot's sensors ride along: robot x's lift is switch-homed (its line), its intake has a piece sensor
    const std::string x = route_maker::describe(*route_maker::robotSource("x"));
    CHECK(rmp1m::wiringOf(x, "lift").find("\"homeSwitch\": \"") != std::string::npos);
    CHECK(rmp1m::wiringOf(x, "intake").find("\"pieceSensor\": ") != std::string::npos);
}

TEST_CASE("RMP1-M: an OWED port, line or sensor is written null -- the placeholder's number or letter never reaches the page") {
    // BUG CAUGHT (mutant): portJson / lineJson ignoring `owed` — no team robot has an owed port today, so a fixture robot is
    // described with its lift, hood and intake (robot x) and its wrist and catapult (robot y) owed.
    using route_maker::WiringOwed;
    auto owedX = *route_maker::robotSource("x");
    owedX.wiringOwed = [](std::string_view m) { return m == "lift" || m == "hood" || m == "intake" ? WiringOwed{true, true} : WiringOwed{}; };
    const std::string x = route_maker::describe(owedX);
    CHECK(rmp1m::wiringOf(x, "lift") == "\"wiring\": {\"motors\": [{\"port\": null, \"reversed\": null}, {\"port\": null, \"reversed\": null}], \"lines\": [], \"homeSwitch\": null}}");
    CHECK(rmp1m::wiringOf(x, "hood") == "\"wiring\": {\"motors\": [], \"lines\": [null, null], \"levels\": null}}");
    CHECK(rmp1m::lineOf(x, "hood").find("\"start\": null") != std::string::npos);
    CHECK(rmp1m::wiringOf(x, "intake") == "\"wiring\": {\"motors\": [{\"port\": null, \"reversed\": null}], \"lines\": [], \"pieceSensor\": null}}");
    CHECK(rmp1m::wiringOf(x, "arm") == "\"wiring\": {\"motors\": [{\"port\": 1, \"reversed\": false}], \"lines\": []}}");  // not owed: as typed
    // ports only (directions typed): the direction stays, the number goes
    auto portsY = *route_maker::robotSource("y");
    portsY.wiringOwed = [](std::string_view m) { return m == "wrist" || m == "catapult" ? WiringOwed{true, false} : WiringOwed{}; };
    const std::string y = route_maker::describe(portsY);
    CHECK(rmp1m::wiringOf(y, "wrist") == "\"wiring\": {\"motors\": [{\"port\": null, \"reversed\": false}], \"lines\": [], \"angleSensor\": null}}");
    CHECK(rmp1m::wiringOf(y, "catapult") == "\"wiring\": {\"motors\": [{\"port\": null, \"reversed\": false}], \"lines\": [], \"pieceSensor\": null}}");
}

TEST_CASE("RMP1-M: which wiring facts each team mechanism waits for -- wiringOwedOf, flag by flag, and both robots' flags are their ledgers'") {
    // BUG CAUGHT (mutant): a typed flag read for the wrong mechanism, or never read (an owed port shown as a fact).
    using route_maker::WiringTyped;
    using route_maker::WiringOwed;
    const char* names[] = {"cascade", "right toggle", "left toggle", "claw", "flipper", "ramp"};
    // flag k owed alone -> exactly these (mechanism, ports|directions) owed
    const std::vector<std::vector<std::pair<int, bool>>> expect = {
        {{0, true}}, {{0, false}}, {{1, true}, {2, true}}, {{2, true}}, {{1, false}}, {{2, false}},
        {{3, true}}, {{4, true}}, {{5, true}}, {{3, false}}, {{4, false}}, {{5, false}}};
    for (std::size_t k = 0; k < expect.size(); ++k) {
        bool flags[12];
        std::fill(std::begin(flags), std::end(flags), true);
        flags[k] = false;
        const WiringTyped t{flags[0], flags[1], flags[2], flags[3], flags[4], flags[5], flags[6], flags[7], flags[8], flags[9], flags[10], flags[11]};
        for (int n = 0; n < 6; ++n) {
            const auto o = route_maker::wiringOwedOf(names[n], t);
            REQUIRE(o.has_value());
            bool ports = false;
            bool dirs = false;
            for (const auto& [mech, isPort] : expect[k]) {
                if (mech == n) {
                    (isPort ? ports : dirs) = true;
                }
            }
            INFO("flag " << k << " owed, " << names[n]);
            CHECK(o->ports == ports);
            CHECK(o->directions == dirs);
        }
    }
    CHECK_FALSE(route_maker::wiringOwedOf("arm", WiringTyped{}).has_value());  // a name it does not know: nothing (a static_assert guards the tables)
    namespace one = shulib::bench::robot_one_facts;
    namespace two = shulib::bench::robot_two_facts;
    const WiringTyped t1 = ROUTE_MAKER_WIRING_TYPED(one);
    const WiringTyped t2 = ROUTE_MAKER_WIRING_TYPED(two);
    CHECK(t1.cascadePorts == one::kCascadePorts.typed);
    CHECK(t2.rightToggleDirection == two::kRightToggleDirection.typed);
    CHECK(t2.leftToggleDirection == two::kLeftToggleDirection.typed);
    CHECK(t2.clawLevel == two::kClawOpenLevel.typed);
    CHECK(t2.flipperLevel == two::kFlipperOpenLevel.typed);
    CHECK(t2.rampLevel == two::kRampDownLevel.typed);
    CHECK(t2.clawLine == two::kClawLine.typed);
    CHECK(t2.sideTogglePorts == two::kSideTogglePorts.typed);
}

TEST_CASE("RMP1-M: each robot's description names the facts its Auton Tester still waits for -- a Test sent then would never build") {
    // BUG CAUGHT (review): robot two's Test sent while its Auton Tester REFUSES (five Stage-B facts owed): minutes of build, then nothing.
    namespace one = shulib::bench::robot_one_facts;
    namespace two = shulib::bench::robot_two_facts;
    using shulib::bench::Program;
    auto owedList = [](const std::string& d) {
        const std::size_t a = d.find("\"testerOwed\": [");
        REQUIRE(a != std::string::npos);
        return d.substr(a, d.find('\n', a) - a);
    };
    const std::string l1 = owedList(route_maker::describe(*route_maker::robotSource("one")));
    const std::string l2 = owedList(route_maker::describe(*route_maker::robotSource("two")));
    CHECK((l1 == "\"testerOwed\": [],") == (one::owedFor(Program::AutonTester) == 0));
    for (const auto& f : two::kRobotTwoFacts) {
        INFO(f.name);
        const bool named = l2.find("\"" + std::string{f.name} + "\"") != std::string::npos;
        CHECK(named == (!f.typed && one::needs(Program::AutonTester, f.needed)));
    }
    std::size_t quotes = static_cast<std::size_t>(std::count(l2.begin(), l2.end(), '"'));
    CHECK(quotes == 2 + 2 * static_cast<std::size_t>(two::owedFor(Program::AutonTester)));
    CHECK(owedList(route_maker::describe(*route_maker::robotSource("x"))) == "\"testerOwed\": [],");  // a fixture: no ledger
}

TEST_CASE("RMP1-O: each robot's claw spot is 15/2 - kClawGrabInsetIn ahead of its centre, on its centre line; a clawless robot has none") {
    // BUG CAUGHT: the inset ignored (the claw drawn at the front edge, 7.5 in: a pin placed there 1.232 in short);
    // the claw off the centre line; a clawless robot given one. Robot two's spot read from robot one's ledger turns
    // RED only once the two ledgers' values differ (today both 1.232: the same number either way).
    CHECK(shulib::bench::robot_one_facts::kClawGrabInsetIn.value() == 1.232);
    CHECK(shulib::bench::robot_two_facts::kClawGrabInsetIn.value() == shulib::bench::robot_one_facts::kClawGrabInsetIn.value());
    for (const char* id : {"one", "two"}) {
        CAPTURE(id);
        const std::string d = route_maker::describe(*route_maker::robotSource(id));
        CHECK(d.find("\n  \"clawSpot\": {\"forwardIn\": 6.268, \"sideIn\": 0},\n") != std::string::npos);
    }
    CHECK(route_maker::robotSource("two")->clawInsetIn == shulib::bench::robot_two_facts::kClawGrabInsetIn.value());
    for (const char* id : {"x", "y", "z"}) {
        CAPTURE(id);
        CHECK(route_maker::describe(*route_maker::robotSource(id)).find("clawSpot") == std::string::npos);
    }
}

TEST_CASE("RMP1-Q: each robot's open front bay is centred on its claw spot, with its ledger's two radii; a robot without one is solid") {
    // BUG CAUGHT: the bay centred on the robot's centre (or a second copy of the claw's inset drifting from it); the lip
    // and body radii swapped; robot two's radii read from robot one's ledger (RED only once the two differ); a fixture
    // robot given a bay (its square no longer solid: every node test of the goals would change).
    CHECK(shulib::bench::robot_one_facts::kFrontBayLipRadiusIn.value() == 3.22);
    CHECK(shulib::bench::robot_one_facts::kFrontBayBodyRadiusIn.value() == 2.80);
    CHECK(shulib::bench::robot_two_facts::kFrontBayLipRadiusIn.value() == shulib::bench::robot_one_facts::kFrontBayLipRadiusIn.value());
    CHECK(shulib::bench::robot_two_facts::kFrontBayBodyRadiusIn.value() == shulib::bench::robot_one_facts::kFrontBayBodyRadiusIn.value());
    for (const char* id : {"one", "two"}) {
        CAPTURE(id);
        const std::string d = route_maker::describe(*route_maker::robotSource(id));
        CHECK(d.find("\n  \"frontBay\": {\"centreForwardIn\": 6.268, \"lipRadiusIn\": 3.22, \"bodyRadiusIn\": 2.8},\n") != std::string::npos);
    }
    const auto two = route_maker::robotSource("two");
    CHECK((*two->frontBayIn)[0] == shulib::bench::robot_two_facts::kFrontBayLipRadiusIn.value());
    CHECK((*two->frontBayIn)[1] == shulib::bench::robot_two_facts::kFrontBayBodyRadiusIn.value());
    for (const char* id : {"x", "y", "z"}) {
        CAPTURE(id);
        CHECK(route_maker::describe(*route_maker::robotSource(id)).find("frontBay") == std::string::npos);
    }
}

TEST_CASE("RMP1: a robot that builds the shared route and declares a cascade marks its levels as its facts ledger's (the tripwire for robot two)") {
    // The shared routes move the cascade to LEDGER levels (`cascade[facts::lv(i).name]`); the page lets it
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

// ═══ RMP1-C: named routes, a lift's plain steps, small moves ═════════════════════════════════════════════════

TEST_CASE("RMP1-C GL20: named routes -- each its own steps and entry, ONE list; robot one registers each as written, robot two mirrors only the marked ones (the twin rule)") {
    // BUG CAUGHT: a route dropped from the list, the list out of the page's order, a route mirrored on robot ONE,
    // an unmarked route mirrored on robot two (or a marked one not), a name's C++ stem colliding.
    namespace sb = shulib::bench;
    const auto& routes = rm_gl20::kTeamRoutes;
    REQUIRE(std::size(routes) == 3);
    const char* names[] = {"skills", "practice 1", "40in test"};
    const sb::Mirror mirrors[] = {sb::Mirror::ForRobotTwo, sb::Mirror::None, sb::Mirror::ForRobotTwo};
    const sb::Together together[] = {sb::Together::Yes, sb::Together::No, sb::Together::No};
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        CHECK(std::string{routes[i].entry().name} == names[i]);
        CHECK(routes[i].mirror == mirrors[i]);
        CHECK(routes[i].together == together[i]);
    }
    CHECK(routes[1].entry == &rm_gl20::practice1Route);  // the stems: letters and digits, camelCase
    CHECK(routes[2].entry == &rm_gl20::route40inTestRoute);  // a leading digit gets "route"
    shulib::competition::AutonRegistry one{sb::kAutonPeriods};
    shulib::competition::AutonRegistry two{sb::kAutonPeriods};
    sb::registerRoutes(one, routes, sb::TeamRobot::One);
    sb::registerRoutes(two, routes, sb::TeamRobot::Two);
    REQUIRE(one.size() == 3);
    REQUIRE(two.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        CAPTURE(i);
        CHECK(std::string{one.at(i).name} == names[i]);
        CHECK(std::string{two.at(i).name} == names[i]);
        CHECK(one.at(i).routine == two.at(i).routine);
        CHECK(one.at(i).mirror == nullptr);
        CHECK(two.at(i).mirror == (mirrors[i] == sb::Mirror::ForRobotTwo ? &shulib::competition::override2026::kSkillsMirror : nullptr));
    }
    {
        const AutonEntry e = routes[1].entry();  // forward 6 then back 3, from (24, -48) facing 90
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.steps == 2);
        CHECK(r.routine.ok);
        const auto t = s.truth();
        CHECK(std::hypot(t.x().value() - 24.0, t.y().value() - -45.0) < 0.6);
    }
    {
        const AutonEntry e = routes[0].entry();  // forward 12 from (-36, -55.7) facing 0
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.ok);
        const auto t = s.truth();
        CHECK(std::hypot(t.x().value() - -24.0, t.y().value() - -55.7) < 0.6);
    }
}

TEST_CASE("RMP1-C GL21: a lift's plain steps -- home, up, up, down, to, back to the bottom -- and an arm's up, to, down, each the constant target the page tracked") {
    // BUG CAUGHT: a relative step written from the wrong height (the export's target is all the robot gets: the
    // library has no relative move), the bottom past the travel, an arm's target written with _deg.
    const AutonEntry e = rm_gl21::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 10);
    CHECK(r.routine.completed == 10);
    CHECK(r.routine.ok);
    CHECK(s.count("MECH", "'lift': SUCCEEDED") == 6);   // the home, up, up, down, to, the bottom
    CHECK(s.count("MECH", "'arm': SUCCEEDED") == 3);
    CHECK(std::abs(s.fakes.liftInches() - 0.0) < 0.5);  // back to the bottom
    // up 30, to 95.5, down 15.5: 80 (reached within 1.5 deg; the plant's arm settles ~3 deg off once the run is over —
    // never near 95.5 or 30, the targets a wrong height would have written)
    CHECK(std::abs(s.fakes.armDeg() - 80.0) < 4.0);
}

TEST_CASE("RMP1-C GL22: small moves -- drive forward / back up N in from the planned pose (1 in among them), after a turn and after a curve") {
    // BUG CAUGHT: a small move written from the wrong pose (the turn's heading, the curve's end), a 1 in move
    // refused or never ending.
    const AutonEntry e = rm_gl22::skillsRoute();
    XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
    const auto r = s.run(e);
    INFO(s.transcript());
    REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.routine.steps == 6);
    CHECK(r.routine.ok);
    const auto t = s.truth();
    CHECK(std::hypot(t.x().value() - -18.75, t.y().value() - -28.0) < 0.6);
}

TEST_CASE("RMP1-C GL23: the page and the suite agree on a route both robots run together near the diagonal -- the page's plan 9.0 in apart at the least, the host test plant's pair within 1 in of it") {
    // BUG CAUGHT: the page's too-close check (its own plan) and the suite's (both robots run on the host test plant)
    // drifting apart -- a route the page passes and the suite fails (the team lead's start, the other way round).
    namespace sb = shulib::bench;
    shulib::competition::AutonRegistry two{sb::kAutonPeriods};
    sb::registerRoutes(two, rm_gl23::kTeamRoutes, sb::TeamRobot::Two);
    REQUIRE(two.size() == 1);
    static const shulib::kinematics::TankKinematics tank{shulib::units::Length{12.0}};
    mirror_pair::PairOptions o;
    o.kin = &tank;
    o.period = two.periodFor(two.at(0));
    const mirror_pair::PairVerdict v = mirror_pair::checkMirroredPair(two.at(0), o);
    INFO(v.sentence);
    CHECK(v.checked);
    CHECK(v.ok);
    CHECK(std::abs(v.leastGapIn - 9.0) < 1.0);  // the page's least gap: 9.0 in (named_routes.test.mjs, GL23)
}

TEST_CASE("RMP1-P GL24: Leave gently -- the page's Wait, then the capped back-up, each a library step; its onFail before each") {
    // BUG CAUGHT: the wait left out (the robot backs straight off the drop), the cap left out (it backs out at full
    // speed: the team lead's stack knocked over; or at the step's faster typed 12 in/s), or the wait cut short. (A
    // carry-on lost after the wait: nothing here fails -- "RMP1-P runner: a gentle back-up that FAILS" catches it.)
    for (auto entry : {&rm_gl24::skillsRoute, &rm_gl24::gentlySlowRoute}) {
        const AutonEntry e = entry();
        CAPTURE(e.name);
        XStack s{rm_rig::startOf(e), mech_fixture::robot_x::kRobotXMechanisms, false};
        const auto r = s.run(e);
        INFO(s.transcript());
        REQUIRE(r.outcome == AutonRunOutcome::RoutineRan);
        CHECK(r.routine.ok);
        const bool skills = std::string{e.name} == "skills";
        CHECK(r.routine.steps == (skills ? 7 : 3));  // a gentle back-up is a wait and a drive; turned first, three calls
        CHECK(s.count("RTN", "wait 0.5 s") >= 1);
        CHECK(s.count("RTN", "wait 1 s") >= (skills ? 1 : 0));
        CHECK(s.world.peakSpeed <= (skills ? 8.0 : 5.0) * 1.1);  // the cap, read back from the plant
        CHECK(s.world.peakSpeed >= (skills ? 8.0 : 5.0) * 0.5);  // ...and it did drive
    }
    const std::string code = golden("gl24_leave_gently.cpp");
    CHECK(code.find("    r.then([&] { return ctx.guard.pause(0.5_s) == sequence::GuardedWaitResult::Satisfied; },\n"
                    "           \"wait 0.5 s\");\n"
                    "    r.reverseTo(-40_in, -55.7_in, {.maxLinearSpeed = 5_in / 1_s});\n") != std::string::npos);
}

TEST_CASE("RMP1-C: the other autons -- the names the page refuses for a route are exactly the autons both files register beside the routes") {
    // BUG CAUGHT: the 40 in renamed (or an auton added) in an autons file while the page keeps the old names: a route
    // the page passes named like one of them, which the brain's menu then drops ("the name is already registered").
    namespace sb = shulib::bench;
    const std::set<std::string> want(route_maker::kOtherAutonNames.begin(), route_maker::kOtherAutonNames.end());
    std::set<std::string> routes;
    for (const sb::TeamRoute& r : sb::teamRoutes()) {
        routes.insert(r.entry().name);
    }
    for (void (*reg)(shulib::competition::AutonRegistry&) : {&sb::registerRobotOneAutons, &sb::registerAutons}) {
        shulib::competition::AutonRegistry r{sb::kAutonPeriods};
        reg(r);
        std::set<std::string> others;
        for (std::size_t i = 0; i < r.size(); ++i) {
            if (routes.count(r.at(i).name) == 0) {
                others.insert(r.at(i).name);
            }
        }
        CHECK(others == want);
    }
    CHECK(route_maker::describe(*route_maker::robotSource("one")).find("\"otherAutons\": [\"40in forward\", \"TEST 24in turn back\"]")
          != std::string::npos);
}
