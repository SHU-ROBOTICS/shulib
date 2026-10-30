// robot_two_claws_example_test.cpp — ROBOT TWO'S CLAWS IN AN AUTON and IN A ROUTE, through the
// Auton Tester's REAL run sequence with the mechanism service live (the mechanism core's MECH-A5,
// 2026-10-29; the MECH brief's re-check U3).
//
// ═══ WHAT THIS FILE PINS ════════════════════════════════════════════════════════════
// The team's autons are robot two's (src/team/autons.cpp says `using namespace robot_two;`), and the
// host suite runs every registered one with its mechanism steps live (test/autons_test.cpp's (1b),
// through test/auton_tester_rig.hpp — the program's chain, guard -> service -> watch, bound over robot
// two's REAL list). No auton in the team's file steps a claw yet (which claw does what in an auton is
// the team's to write — the project's G4 guardrail), so THIS file registers one, and a ROUTE whose
// markers fire robot two's derived route ids, and asserts EACH STEP'S VERDICT — never only "it ended
// safe": a step that can never be met, a sensorless grab that stops the chain, a marker with bad args,
// or a route id the robot does not have each fails `ctest` here, before any upload.
//
// It is an EXAMPLE test on purpose: guide chapter 16 quotes the auton below line for line (the
// documentation build's verbatim rule reads `test/*example*_test.cpp`), so the listing a student
// copies is one the suite compiles and runs on robot two's real declarations.
//
// NO HARDWARE CLAIM: the claws are fakes behind the real service; nothing here moved a claw.

#include "doctest.h"

#include <cstdio>
#include <string>
#include <string_view>
#include <vector>

#include "auton_tester_rig.hpp"
#include "programs/auton_tester_run.hpp"
#include "robots/mechanism_table.hpp"
#include "team/autons.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/route.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

namespace claws_example {
using namespace shulib::bench::robot_two;  // robot two's mechanisms, and the mechanism words

// ── guide chapter 16: a claws auton ─────────────────────────────────────────────────────
RoutineResult grabAndPivot(AutonContext& ctx) {
    Routine r{ctx, "grab-and-pivot"};
    r.set(claw["open"]);
    r.driveTo(20_in, 0_in);
    r.set(claw["closed"]);                     // no sensor: UNCONFIRMED, counted done
    r.run(clawPivot6, 12_volt, 0.4_s);         // pivot 6 forward for 0.4 s
    r.run(clawPivot7, -12_volt, 0.4_s);        // pivot 7 back for 0.4 s
    return r.result();
}
}  // namespace claws_example

namespace {

using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::competition::StartPose;
using shulib::units::Length;
using shulib::units::Time;

const shulib::kinematics::TankKinematics& tank() {
    static const shulib::kinematics::TankKinematics k{Length{12.0}};
    return k;
}

/// Every MECH line logged since `from`, in order.
std::vector<std::string> mechLines(const shulib::hal::fake::FakeTelemetrySink& sink, int from) {
    std::vector<std::string> out;
    for (int i = from; i < sink.size(); ++i) {
        if (sink.at(i).subsystem == "MECH") out.push_back(sink.at(i).message);
    }
    return out;
}

/// `line` begins with `prefix`.
bool startsWith(const std::string& line, std::string_view prefix) { return line.rfind(prefix, 0) == 0; }

/// A route marker.
shulib::config::Marker marker(const char* id, std::int32_t a0 = 0, std::int32_t a1 = 0, int args = 0) {
    shulib::config::Marker m{};
    std::snprintf(m.id, sizeof m.id, "%s", id);
    m.args[0] = a0;
    m.args[1] = a1;
    m.argCount = static_cast<std::uint8_t>(args);
    return m;
}

}  // namespace

TEST_CASE("MECH-A5 claws auton: robot two's claws in an auton through the Auton Tester's run, each "
          "step's VERDICT -- open SUCCEEDED, the sensorless grab UNCONFIRMED and counted done, each "
          "pivot SUCCEEDED by time, the routine OK and FINISHED") {
    // BUG CAUGHT: the tester running an auton with no mechanism service in its chain (every claw step
    // ABSENT, the chain stopped — the MECH brief's U3); the sensorless grab stopping the auton (the
    // team lead's Q2 (c): counted done); a pivot step that never commands its motor, the wrong motor,
    // or the wrong sign; the step's time ignored; the claw left open at the end (the guard's end of
    // run makes every mechanism safe — closed).
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    AutonEntry e{};
    e.name = "grab and pivot";
    e.kind = AutonKind::Match;
    e.routine = &claws_example::grabAndPivot;
    e.start = StartPose::relative();
    e.endActionAt = Time{27.0};
    e.hardStopAt = Time{29.0};
    reg.add(e);
    tester_rig::TesterStack s{tank()};
    double six = 0.0;
    double seven = 0.0;
    bool openWhileDriving = false;
    s.inputs.onRead = [&](double) {
        six = s.two.m6.commandedVoltage().value() != 0.0 ? s.two.m6.commandedVoltage().value() : six;
        seven = s.two.m7.commandedVoltage().value() != 0.0 ? s.two.m7.commandedVoltage().value() : seven;
        openWhileDriving = openWhileDriving || (s.two.lineA.commanded() && s.plant.distance > 5.0);
    };
    const int from = s.sink.size();
    const shulib::bench::TesterRunRecord& r = s.run(reg, 0);
    CHECK(std::string_view{shulib::bench::testerResultWord(r)} == "FINISHED");
    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK(r.run.routine.ok);
    CHECK(r.run.routine.steps == 5);
    CHECK(r.run.routine.completed == 5);
    CHECK(six == 12.0);     // pivot 6 forward
    CHECK(seven == -12.0);  // pivot 7 back
    CHECK(openWhileDriving);
    // Each step's verdict, in order, as the service logs it.
    const std::vector<std::string> lines = mechLines(s.sink, from);
    std::vector<std::string> verdicts;
    for (const std::string& l : lines) {
        if (startsWith(l, "'claw")) verdicts.push_back(l);
    }
    REQUIRE(verdicts.size() >= 4U);
    CHECK(startsWith(verdicts[0], "'claw': SUCCEEDED -- time in 0.5"));
    CHECK(startsWith(verdicts[1], "'claw': UNCONFIRMED -- time in 0.5"));
    CHECK(verdicts[1].find("(no piece sensor to confirm it; counted done)") != std::string::npos);
    CHECK(startsWith(verdicts[2], "'claw pivot 6': SUCCEEDED -- time in 0.4"));
    CHECK(startsWith(verdicts[3], "'claw pivot 7': SUCCEEDED -- time in 0.4"));
    CHECK(s.mechanismsSafe());  // the end of run: every mechanism safe, the claw CLOSED
}

TEST_CASE("MECH-A5 claws route: a ROUTE whose markers fire robot two's DERIVED route ids, through the "
          "tester with the program's registry -- each marker's verdict; a marker with no time limit "
          "REFUSED and the route stopped; an id the robot does not have counted unknown") {
    // BUG CAUGHT: a program that registers no route ids (every marker an unknown id — M47), a marker
    // that runs a step the auton's own line would not, the sensorless grab stopping the route, a
    // marker's args trusted (a pivot run with no time limit), or a route editor's typo run silently.
    const auto runRoute = [](const shulib::config::Route& route, tester_rig::TesterStack& s) {
        AutonRegistry reg{shulib::bench::kAutonPeriods};
        AutonEntry e{};
        e.name = "claws route";
        e.kind = AutonKind::Match;
        e.route = &route;
        e.endActionAt = Time{27.0};
        e.hardStopAt = Time{29.0};
        reg.add(e);
        return s.run(reg, 0);
    };
    {
        static shulib::config::Route route{.name = "claws_route"};
        route.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0),
                              .headingMode = shulib::config::HeadingMode::Fixed,
                              .motion = shulib::config::WaypointMotion::Start};
        route.waypoints[0].markers[0] = marker("claw_open");
        route.waypoints[0].markerCount = 1;
        route.waypoints[1] = {.x = Length{20.0}, .y = Length{0.0}, .headingMode = shulib::config::HeadingMode::Free,
                              .motion = shulib::config::WaypointMotion::DriveTo,
                              .constraints = {.timeout = Time{5.0}}};
        route.waypoints[1].markers[0] = marker("claw_closed");
        route.waypoints[1].markers[1] = marker("claw_pivot_6_run", 12, 400, 2);
        route.waypoints[1].markers[2] = marker("claw_pivot_7_run", -12, 400, 2);
        route.waypoints[1].markerCount = 3;
        route.count = 2;
        REQUIRE(route.valid());
        tester_rig::TesterStack s{tank()};
        const int from = s.sink.size();
        const shulib::bench::TesterRunRecord& r = runRoute(route, s);
        CHECK(std::string_view{shulib::bench::testerResultWord(r)} == "FINISHED");
        REQUIRE(r.run.outcome == AutonRunOutcome::RouteRan);
        CHECK(r.run.route.ok());
        CHECK(r.run.route.markersFired == 4);
        CHECK(r.run.route.unknownIds == 0);
        std::vector<std::string> verdicts;
        for (const std::string& l : mechLines(s.sink, from)) {
            if (startsWith(l, "'claw")) verdicts.push_back(l);
        }
        REQUIRE(verdicts.size() >= 4U);
        CHECK(startsWith(verdicts[0], "'claw': SUCCEEDED -- time in 0.5"));
        CHECK(startsWith(verdicts[1], "'claw': UNCONFIRMED -- time in 0.5"));
        CHECK(startsWith(verdicts[2], "'claw pivot 6': SUCCEEDED -- time in 0.4"));
        CHECK(startsWith(verdicts[3], "'claw pivot 7': SUCCEEDED -- time in 0.4"));
        CHECK(s.mechanismsSafe());
    }
    {
        // A pivot marker with NO time limit: refused, by name, nothing commanded, the route stopped.
        static shulib::config::Route bad{.name = "bad_route"};
        bad.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0),
                            .headingMode = shulib::config::HeadingMode::Fixed,
                            .motion = shulib::config::WaypointMotion::Start};
        bad.waypoints[0].markers[0] = marker("claw_pivot_6_run", 12, 0, 1);
        bad.waypoints[0].markers[1] = marker("claw_open");
        bad.waypoints[0].markerCount = 2;
        bad.count = 1;
        tester_rig::TesterStack s{tank()};
        bool pivotMoved = false;
        bool clawOpened = false;
        s.inputs.onRead = [&s, &pivotMoved, &clawOpened](double) {
            pivotMoved = pivotMoved || s.two.m6.commandedVoltage().value() != 0.0;
            clawOpened = clawOpened || s.two.lineA.commanded();
        };
        const int from = s.sink.size();
        const shulib::bench::TesterRunRecord& r = runRoute(bad, s);
        REQUIRE(r.run.outcome == AutonRunOutcome::RouteRan);
        CHECK_FALSE(r.run.route.ok());
        CHECK_FALSE(pivotMoved);  // nothing commanded
        CHECK_FALSE(clawOpened);  // ... and the route stopped there: the next marker never ran
        bool refusedLine = false;
        for (int i = from; i < s.sink.size(); ++i) {
            refusedLine = refusedLine || startsWith(s.sink.at(i).message, "route marker 'claw_pivot_6_run' REFUSED:");
        }
        CHECK(refusedLine);
    }
    {
        // An id the robot does not have (a route editor's typo): unknown, counted — never run.
        static shulib::config::Route typo{.name = "typo_route"};
        typo.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0),
                             .headingMode = shulib::config::HeadingMode::Fixed,
                             .motion = shulib::config::WaypointMotion::Start};
        typo.waypoints[0].markers[0] = marker("claw_opne");
        typo.waypoints[0].markerCount = 1;
        typo.count = 1;
        tester_rig::TesterStack s{tank()};
        bool clawOpened = false;
        s.inputs.onRead = [&s, &clawOpened](double) { clawOpened = clawOpened || s.two.lineA.commanded(); };
        const shulib::bench::TesterRunRecord& r = runRoute(typo, s);
        REQUIRE(r.run.outcome == AutonRunOutcome::RouteRan);
        CHECK(r.run.route.unknownIds == 1);
        CHECK_FALSE(clawOpened);  // the claw was never opened
    }
}
