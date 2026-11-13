// mechanism/route_ids.hpp — EVERY MECHANISM'S ACTIONS AS ROUTE IDS (chunk MECH-A1; the team lead's
// ruling of 2026-10-08; the MECH brief's §1.16 and §3.1 "route ids").
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * THE RULE, exactly: robot x publishes P14's 17 ids IN ORDER with their arg counts, robot two its
//     6, robot y (every optional word) its 18 — each id's mechanism, action and preset index pinned.
//   * THE STEM: lower case, every run of other characters ONE '_' ("Claw  Pivot-6!" -> claw_pivot_6),
//     never a leading or trailing '_'.
//   * THE FOUR REFUSALS, each naming the offending id: exactly 23 characters accepted and 24
//     refused; a duplicate (a preset named "home" beside a homing method; two spellings of one stem);
//     a built-in (wait_ms, brake, hold_ms, log); the 33rd id. The FIRST problem wins and stops the
//     derivation (nothing after it is added).
//   * The capacity IS F8's: kMaxRouteIds == runner::kMaxCommands, so a change to either is a red test.
// The same four AT BUILD, at the robot's list, are test/compile_refusals/'s (n60–n63).

#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "shulib/config/route.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/runner/command_registry.hpp"

namespace mech = shulib::mechanism;
using mech::RouteAction;
using mech::RouteIdProblem;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
namespace r2 = mech_fixture::robot_two;

namespace {

struct Expected {
    const char* id;
    std::uint8_t mechanism;
    RouteAction action;
    std::uint8_t index;
    std::uint8_t args;
};

void checkIds(const mech::RouteIds& got, const std::vector<Expected>& want) {
    CHECK(got.problem == RouteIdProblem::None);
    REQUIRE(got.count == want.size());
    for (std::size_t i = 0; i < want.size(); ++i) {
        CAPTURE(i);
        CHECK(std::string{got.ids[i].view()} == want[i].id);
        CHECK(got.ids[i].mechanism == want[i].mechanism);
        CHECK(got.ids[i].action == want[i].action);
        CHECK(got.ids[i].index == want[i].index);
        CHECK(got.ids[i].argCount == want[i].args);
        CHECK(shulib::config::commandIdValid(got.ids[i].view()));  // F8's vocabulary, every one
    }
}

/// A one-mechanism spec list named `name`: a power mechanism (run, stop; + until_piece/gone with a
/// piece sensor) — enough to drive the stem rule and the length bound.
mech::MechanismSpec powerNamed(const char* name, bool withSensor = false) {
    mech::MechanismSpec s = r2::clawPivot6.spec();
    std::memset(s.name, 0, sizeof s.name);
    std::strncpy(s.name, name, sizeof s.name - 1);
    if (withSensor) {
        s.pieceSensor.kind = mech::SensorKind::Distance;
        s.pieceSensor.seesWithinIn = 2.0;
    }
    return s;
}

mech::RouteIds idsOf(std::initializer_list<mech::MechanismSpec> specs) {
    const std::vector<mech::MechanismSpec> v{specs};
    return mech::routeIdsOf(v);
}

}  // namespace

TEST_CASE("MECH-A1 route ids: robot x publishes exactly P14's 17, in its list's order, each with its args") {
    checkIds(mech::routeIdsOf(rx::kRobotXMechanisms.specSpan()),
             {{"arm_stow", 0, RouteAction::GoToPreset, 0, 1},
              {"arm_score", 0, RouteAction::GoToPreset, 1, 1},
              {"arm_high", 0, RouteAction::GoToPreset, 2, 1},
              {"arm_home", 0, RouteAction::Home, 0, 1},  // restsAtStop IS a homing method
              {"lift_floor", 1, RouteAction::GoToPreset, 0, 1},
              {"lift_low", 1, RouteAction::GoToPreset, 1, 1},
              {"lift_high", 1, RouteAction::GoToPreset, 2, 1},
              {"lift_home", 1, RouteAction::Home, 0, 1},
              {"flywheel_shoot", 2, RouteAction::SpinUp, 0, 1},
              {"flywheel_stop", 2, RouteAction::Stop, 0, 0},
              {"intake_run", 3, RouteAction::Run, 0, 2},
              {"intake_until_piece", 3, RouteAction::RunUntilPiece, 0, 2},
              {"intake_until_gone", 3, RouteAction::RunUntilGone, 0, 2},
              {"intake_stop", 3, RouteAction::Stop, 0, 0},
              {"hood_down", 4, RouteAction::SetPosition, 0, 0},
              {"hood_middle", 4, RouteAction::SetPosition, 1, 0},
              {"hood_up", 4, RouteAction::SetPosition, 2, 0}});
}

TEST_CASE("MECH-A1 route ids: robot two publishes its 6 -- and a power mechanism with NO sensor has no until_ ids") {
    checkIds(mech::routeIdsOf(r2::kRobotTwoMechanisms.specSpan()),
             {{"claw_pivot_6_run", 0, RouteAction::Run, 0, 2},
              {"claw_pivot_6_stop", 0, RouteAction::Stop, 0, 0},
              {"claw_pivot_7_run", 1, RouteAction::Run, 0, 2},
              {"claw_pivot_7_stop", 1, RouteAction::Stop, 0, 0},
              {"claw_closed", 2, RouteAction::SetPosition, 0, 0},
              {"claw_open", 2, RouteAction::SetPosition, 1, 0}});
}

TEST_CASE("MECH-A1 route ids: robot y (every optional word) -- a NEVER-homed lift has no _home id") {
    checkIds(mech::routeIdsOf(ry::kRobotYMechanisms.specSpan()),
             {{"wrist_flat", 0, RouteAction::GoToPreset, 0, 1},
              {"wrist_home", 0, RouteAction::Home, 0, 1},  // absolute: homes with no motion, still an id
              {"swing_down", 1, RouteAction::GoToPreset, 0, 1},
              {"swing_up", 1, RouteAction::GoToPreset, 1, 1},
              {"swing_home", 1, RouteAction::Home, 0, 1},
              {"dr4b_low", 2, RouteAction::GoToPreset, 0, 1},
              {"dr4b_home", 2, RouteAction::Home, 0, 1},
              {"tilt_mid", 3, RouteAction::GoToPreset, 0, 1},  // neverHomed: no tilt_home
              {"roller_run", 4, RouteAction::Run, 0, 2},
              {"roller_until_piece", 4, RouteAction::RunUntilPiece, 0, 2},
              {"roller_until_gone", 4, RouteAction::RunUntilGone, 0, 2},
              {"roller_stop", 4, RouteAction::Stop, 0, 0},
              {"catapult_run", 5, RouteAction::Run, 0, 2},
              {"catapult_until_piece", 5, RouteAction::RunUntilPiece, 0, 2},
              {"catapult_until_gone", 5, RouteAction::RunUntilGone, 0, 2},
              {"catapult_stop", 5, RouteAction::Stop, 0, 0},
              {"grip_shut", 6, RouteAction::SetPosition, 0, 0},
              {"grip_loose", 6, RouteAction::SetPosition, 1, 0}});
}

TEST_CASE("MECH-A1 route ids: the STEM -- lower case, every run of other characters one '_', none at either end") {
    CHECK(std::string{idsOf({powerNamed("Claw  Pivot-6!")}).ids[0].view()} == "claw_pivot_6_run");
    CHECK(std::string{idsOf({powerNamed("  arm")}).ids[0].view()} == "arm_run");
    CHECK(std::string{idsOf({powerNamed("ARM__2")}).ids[1].view()} == "arm_2_stop");
    CHECK(std::string{idsOf({powerNamed("a.b,c")}).ids[0].view()} == "a_b_c_run");
    CHECK(std::string{idsOf({powerNamed("x-")}).ids[0].view()} == "x_run");
}

TEST_CASE("MECH-A1 route ids: TOO LONG -- exactly 23 characters accepted, 24 refused, naming the id") {
    // "abcdefghijklmnopq_run" is 21; pad the stem so "<stem>_stop" lands exactly on 23 and 24.
    const mech::RouteIds ok = idsOf({powerNamed("abcdefghijklmnopqr")});  // _stop -> 23
    CHECK(ok.problem == RouteIdProblem::None);
    CHECK(std::string{ok.ids[1].view()} == "abcdefghijklmnopqr_stop");
    CHECK(ok.ids[1].view().size() == 23U);
    const mech::RouteIds no = idsOf({powerNamed("abcdefghijklmnopqrs")});  // _stop -> 24
    CHECK(no.problem == RouteIdProblem::TooLong);
    CHECK(std::string{no.offending} == "abcdefghijklmnopqrs_stop");
    CHECK(no.count == 1);  // its _run (23) was published before the problem, and nothing after it
    // n60's case: "conveyor belt one" + "_until_piece" is 29.
    const mech::RouteIds belt = idsOf({powerNamed("conveyor belt one", true)});
    CHECK(belt.problem == RouteIdProblem::TooLong);
    CHECK(std::string{belt.offending} == "conveyor_belt_one_until_piece");
    CHECK(std::string{mech::routeIdProblemText(RouteIdProblem::TooLong)}
          == "a route id does not fit 23 characters: shorten the mechanism's or the preset's name");
}

TEST_CASE("MECH-A1 route ids: TWO ALIKE -- a preset named 'home' beside a homing method; two spellings of one stem") {
    mech::MechanismSpec arm = rx::arm.spec();
    std::strcpy(arm.position.presets[1].name, "home");  // arm_home twice (n61)
    const mech::RouteIds d = idsOf({arm});
    CHECK(d.problem == RouteIdProblem::Duplicate);
    CHECK(std::string{d.offending} == "arm_home");
    const mech::RouteIds two = idsOf({powerNamed("claw pivot"), powerNamed("claw_pivot")});
    CHECK(two.problem == RouteIdProblem::Duplicate);
    CHECK(std::string{two.offending} == "claw_pivot_run");
    CHECK(two.count == 2);  // the first mechanism's two, then the problem
}

TEST_CASE("MECH-A1 route ids: A BUILT-IN -- wait_ms, brake, hold_ms and log are the library's own") {
    mech::MechanismSpec wait = r2::claw.spec(1);
    std::strcpy(wait.name, "wait");
    std::strcpy(wait.pneumatic.positions[0].name, "ms");  // wait_ms (n62)
    const mech::RouteIds w = idsOf({wait});
    CHECK(w.problem == RouteIdProblem::Builtin);
    CHECK(std::string{w.offending} == "wait_ms");
    mech::MechanismSpec hold = r2::claw.spec(1);
    std::strcpy(hold.name, "hold");
    std::strcpy(hold.pneumatic.positions[1].name, "ms");
    CHECK(idsOf({hold}).problem == RouteIdProblem::Builtin);
    // Every built-in the vocabulary has would be refused (the table is F8's, read — not copied).
    for (const auto& b : shulib::config::kBuiltinCommands) {
        CAPTURE(b.id);
        CHECK(shulib::config::findBuiltinCommand(b.id) != nullptr);
    }
}

TEST_CASE("MECH-A1 route ids: TOO MANY -- 32 published, the 33rd refused (n63), and the capacity IS F8's") {
    static_assert(mech::kMaxRouteIds == shulib::runner::kMaxCommands,
                  "a robot's route ids share a route registry's capacity");
    // Four 6-preset arms publish 4 x 7 = 28; a fifth's first four reach 32; its fifth is the 33rd.
    std::vector<mech::MechanismSpec> specs;
    for (int k = 0; k < 5; ++k) {
        mech::MechanismSpec a = rx::arm.spec();
        std::snprintf(a.name, sizeof a.name, "a%d", k);
        for (std::size_t p = 0; p < 6; ++p) {
            std::snprintf(a.position.presets[p].name, sizeof a.position.presets[p].name, "p%zu", p + 1);
            a.position.presets[p].value = static_cast<double>(p);
        }
        a.position.presetCount = 6;
        specs.push_back(a);
    }
    const mech::RouteIds r = mech::routeIdsOf(specs);
    CHECK(r.problem == RouteIdProblem::TooMany);
    CHECK(r.count == 32);
    CHECK(std::string{r.offending} == "a4_p5");
    // ... and exactly 32 is accepted.
    specs.back().position.presetCount = 3;  // a4: p1..p3 + home = 4 -> 28 + 4 = 32
    const mech::RouteIds full = mech::routeIdsOf(specs);
    CHECK(full.problem == RouteIdProblem::None);
    CHECK(full.count == 32);
}

TEST_CASE("MECH-A1 route ids: every problem has its sentence; a robot with none publishes none") {
    CHECK(std::string{mech::routeIdProblemText(RouteIdProblem::None)}.empty());
    CHECK(std::string{mech::routeIdProblemText(RouteIdProblem::Duplicate)}.find("rename") != std::string::npos);
    CHECK(std::string{mech::routeIdProblemText(RouteIdProblem::Builtin)}.find("wait_ms") != std::string::npos);
    CHECK(std::string{mech::routeIdProblemText(RouteIdProblem::TooMany)}.find("32") != std::string::npos);
    CHECK(std::string{mech::routeIdProblemText(static_cast<RouteIdProblem>(9))} == "an unknown route id problem");
    const mech::RouteIds none = mech::routeIdsOf(mech::mechanisms().specSpan());
    CHECK(none.count == 0);
    CHECK(none.problem == RouteIdProblem::None);
}
