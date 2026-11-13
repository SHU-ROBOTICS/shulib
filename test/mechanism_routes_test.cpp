// MECH-A4 — a robot's MECHANISM ROUTE IDS, bound and registered in ONE call (runner/mechanism_routes.hpp;
// the brief's §1.16, P14, N13 (a)), fired by the REAL PathRunner inside the REAL competition::runAuton
// under the REAL RunGuard, the service in the chassis's pacer chain, on the host test plant: robot x's
// route of generated markers; robot two's sensorless grab UNCONFIRMED, counted done, the route going on;
// a marker's bad args REFUSED (the route stopped, the drive safed, nothing commanded); a marker whose
// step fails; the manifest's lines byte-exact with their docs; the registry's doc (F8-additive). Every
// case names the bug it would catch. Nothing here has run on a robot.

#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <initializer_list>
#include <string>

#include "mechanism_auton_rig.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/manifest.hpp"
#include "shulib/runner/mechanism_routes.hpp"
#include "shulib/runner/path_runner.hpp"

using auton_rig::RobotTwo;
using auton_rig::RobotX;
using shulib::competition::AutonRunOutcome;
using shulib::competition::AutonRunResult;
using shulib::manipulation::MechanismOutcome;
using shulib::runner::CommandRegistry;
using shulib::runner::MechanismRoutes;
using shulib::runner::RunStopCause;
namespace cfg = shulib::config;
using namespace shulib::units::literals;

namespace {

cfg::Marker mk(const char* id, std::initializer_list<std::int32_t> args = {}) {
    cfg::Marker m{};
    std::snprintf(m.id, sizeof m.id, "%s", id);
    for (const std::int32_t a : args) {
        m.args[m.argCount++] = a;
    }
    return m;
}

/// A ROUTE entry (its Start waypoint is its start pose).
shulib::competition::AutonEntry routeEntry(const cfg::Route& r) {
    shulib::competition::AutonEntry e{};
    e.name = "route";
    e.kind = shulib::competition::AutonKind::Match;
    e.route = &r;
    e.endActionAt = shulib::units::Time{13.0};
    e.hardStopAt = shulib::units::Time{14.0};
    return e;
}

/// Robot x's route of the brief's P14: home the lift and start the intake at the start; drive 24 in;
/// the arm to 'score', the hood up, the flywheel to 'shoot'; a short hold; feed, then stop the wheel.
cfg::Route xRoute() {
    cfg::Route r{.name = "score_three_route"};
    r.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0), .headingMode = cfg::HeadingMode::Fixed,
                      .motion = cfg::WaypointMotion::Start};
    r.waypoints[0].markers[0] = mk("lift_home", {1500});
    r.waypoints[0].markers[1] = mk("intake_until_piece", {12, 3000});
    r.waypoints[0].markerCount = 2;
    r.waypoints[1] = {.x = shulib::units::Length{24.0}, .y = shulib::units::Length{0.0},
                      .headingMode = cfg::HeadingMode::Free, .motion = cfg::WaypointMotion::DriveTo,
                      .constraints = {.timeout = shulib::units::Time{5.0}}};
    r.waypoints[1].markers[0] = mk("arm_score", {1500});
    r.waypoints[1].markers[1] = mk("hood_up");
    r.waypoints[1].markers[2] = mk("flywheel_shoot", {2000});
    r.waypoints[1].markerCount = 3;
    r.waypoints[2] = {.headingMode = cfg::HeadingMode::Free, .motion = cfg::WaypointMotion::Hold,
                      .constraints = {.timeout = shulib::units::Time{0.3}}};
    r.waypoints[2].markers[0] = mk("intake_run", {12, 500});
    r.waypoints[2].markers[1] = mk("flywheel_stop");
    r.waypoints[2].markerCount = 2;
    r.count = 3;
    return r;
}

/// One marker, then hood_up, at the Start.
cfg::Route oneMarker(const cfg::Marker& m) {
    cfg::Route r{.name = "bad"};
    r.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0), .headingMode = cfg::HeadingMode::Fixed,
                      .motion = cfg::WaypointMotion::Start};
    r.waypoints[0].markers[0] = m;
    r.waypoints[0].markers[1] = mk("hood_up");
    r.waypoints[0].markerCount = 2;
    r.count = 1;
    return r;
}

const char* const kBuiltinLines =
    "{\"id\":\"wait_ms\",\"builtin\":true,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"wait, commanding nothing, for args[0] "
    "milliseconds (the drive keeps its last state)\"}\n"
    "{\"id\":\"brake\",\"builtin\":true,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"stop the drive (0 V + Brake) and block "
    "until the estimate certifies rest\"}\n"
    "{\"id\":\"hold_ms\",\"builtin\":true,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"actively hold the current pose for "
    "args[0] milliseconds, driving back disturbance\"}\n"
    "{\"id\":\"log\",\"builtin\":true,\"minArgs\":0,\"maxArgs\":2,\"doc\":\"write one Info line naming this marker and "
    "its args\"}\n";

}  // namespace

TEST_CASE("MECH-A4 P14: robot x's route of generated markers -- ROUTE_RAN, every marker SUCCEEDED, the end "
          "action succeeded") {
    // Bug caught: a derived id bound to the wrong action or mechanism, a marker that returns before its
    // step's verdict, a marker that runs a step the auton's own line would not, or a route the binder
    // stops for no reason.
    RobotX x;
    x.world.sensorSeesAt = 1.0;
    static CommandRegistry registry;
    registry = CommandRegistry{};
    MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    routes.registerInto(registry);
    CHECK(registry.count() == 17);
    const cfg::Route route = xRoute();
    REQUIRE(route.valid());
    const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry, x.sink,
                                                             x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
    CHECK(res.route.ok());
    CHECK(res.route.markersFired == 7);
    CHECK(res.route.unknownIds == 0);
    CHECK(res.guard.endActionRan);
    CHECK(res.guard.endActionSucceeded);
    CHECK(x.count("MECH", "'lift': SUCCEEDED") == 1);
    CHECK(x.count("MECH", "'intake': SUCCEEDED -- sees a piece") == 1);
    CHECK(x.count("MECH", "'arm': SUCCEEDED -- reached") == 1);
    CHECK(x.count("MECH", "'hood': SUCCEEDED -- time") == 1);
    CHECK(x.count("MECH", "'flywheel': SUCCEEDED -- reached") == 1);
    CHECK(x.count("MECH", "'intake': SUCCEEDED -- time") == 1);
    CHECK(x.count("MECH", "REFUSED") == 0);
    CHECK(x.allSafeAndReleased());  // the guard's end of run
}

TEST_CASE("MECH-A4 P14d (M44): robot two's route -- the sensorless grab UNCONFIRMED, COUNTED done, the "
          "route goes on") {
    // Bug caught (M44): the binder mapping a counted UNCONFIRMED to Unconfirmed — the PathRunner stops
    // on anything but Succeeded, so every robot-two route would end at its first grab.
    RobotTwo two;
    static CommandRegistry registry;
    registry = CommandRegistry{};
    MechanismRoutes routes{two.chassis, two.svc, mech_fixture::robot_two::kRobotTwoMechanisms, two.sink};
    routes.registerInto(registry);
    cfg::Route r{.name = "grab_route"};
    r.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0), .headingMode = cfg::HeadingMode::Fixed,
                      .motion = cfg::WaypointMotion::Start};
    r.waypoints[0].markers[0] = mk("claw_open");
    r.waypoints[0].markerCount = 1;
    r.waypoints[1] = {.x = shulib::units::Length{20.0}, .y = shulib::units::Length{0.0},
                      .headingMode = cfg::HeadingMode::Free, .motion = cfg::WaypointMotion::DriveTo,
                      .constraints = {.timeout = shulib::units::Time{5.0}}};
    r.waypoints[1].markers[0] = mk("claw_closed");
    r.waypoints[1].markers[1] = mk("claw_pivot_6_run", {12, 400});
    r.waypoints[1].markers[2] = mk("claw_pivot_7_run", {-12, 400});
    r.waypoints[1].markerCount = 3;
    r.count = 2;
    REQUIRE(r.valid());
    double v7 = 0.0;
    two.world.onPace = [&two, &v7] {
        if (two.p7.commandedVoltage().value() != 0.0) {
            v7 = two.p7.commandedVoltage().value();
        }
    };
    const AutonRunResult res =
        shulib::competition::runAuton(two.chassis, two.guard, routeEntry(r), &registry, two.sink, two.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
    CHECK(res.route.ok());
    CHECK(res.route.markersFired == 4);
    bool unconfirmed = false;
    for (const std::string& l : two.lines("MECH")) {
        unconfirmed = unconfirmed ||
                      l == "'claw': UNCONFIRMED -- time in 0.50s (no piece sensor to confirm it; counted done)" ||
                      l == "'claw': UNCONFIRMED -- time in 0.51s (no piece sensor to confirm it; counted done)";
    }
    CHECK(unconfirmed);
    CHECK(v7 == -12.0);  // a marker's negative volts reach the pivot
    CHECK_FALSE(two.lineA.commanded());  // closed = Low, kept through the run's end (its safe position)
}

TEST_CASE("MECH-A4 P14b/c (M45): a marker's bad args are REFUSED -- one line naming the id and what it needs; "
          "the route stops, the drive safed, nothing commanded") {
    // Bug caught (M45): the args check removed — `arm_score` with no args would run a 0 s step (its own
    // precondition throws out of the route) and `intake_run [13, 500]` a 13 V command; a refusal must be
    // DATA's answer (REFUSED, the route stopped), never a throw and never a motor command.
    struct Case {
        cfg::Marker m;
        const char* line;
    };
    const Case cases[] = {
        {mk("arm_score"), "route marker 'arm_score' REFUSED: it needs its time limit in ms, above 0 (args[0])"},
        {mk("arm_score", {0}), "route marker 'arm_score' REFUSED: it needs its time limit in ms, above 0 (args[0])"},
        {mk("lift_home", {-5}), "route marker 'lift_home' REFUSED: it needs its time limit in ms, above 0 (args[0])"},
        {mk("flywheel_shoot"),
         "route marker 'flywheel_shoot' REFUSED: it needs its time limit in ms, above 0 (args[0])"},
        {mk("intake_run", {13, 500}),
         "route marker 'intake_run' REFUSED: it needs its volts (args[0], -12..12, not 0) and its time in ms, above 0 "
         "(args[1])"},
        {mk("intake_run", {-13, 500}),
         "route marker 'intake_run' REFUSED: it needs its volts (args[0], -12..12, not 0) and its time in ms, above 0 "
         "(args[1])"},
        {mk("intake_run", {0, 500}),
         "route marker 'intake_run' REFUSED: it needs its volts (args[0], -12..12, not 0) and its time in ms, above 0 "
         "(args[1])"},
        {mk("intake_until_piece", {12, 0}),
         "route marker 'intake_until_piece' REFUSED: it needs its volts (args[0], -12..12, not 0) and its time in ms, "
         "above 0 (args[1])"},
        {mk("intake_until_gone", {12}),
         "route marker 'intake_until_gone' REFUSED: it needs its volts (args[0], -12..12, not 0) and its time in ms, "
         "above 0 (args[1])"},
    };
    for (const Case& c : cases) {
        CAPTURE(c.line);
        RobotX x;
        static CommandRegistry registry;
        registry = CommandRegistry{};
        MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
        routes.registerInto(registry);
        const cfg::Route route = oneMarker(c.m);
        const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry,
                                                                 x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
        CHECK(res.route.reason == RunStopCause::MarkerFailed);
        CHECK(res.route.stoppedAt == 0);
        CHECK(res.route.markersFired == 1);  // the refused one; hood_up after it never fired
        CHECK(x.count("MECH", c.line) == 1);
        CHECK(x.count("RUN", "returned REFUSED") == 1);
        CHECK(x.count("MECH", "'hood'") == 0);
        CHECK(x.armM.commandedVoltage().value() == 0.0);
        CHECK(x.intakeM.commandedVoltage().value() == 0.0);
        CHECK(x.flyM.commandedVoltage().value() == 0.0);
        for (const shulib::hal::IMotor* m : x.rig.h.context().driveMotors()) {
            CHECK(m->commandedVoltage().value() == 0.0);  // the drive safed
        }
    }
}

TEST_CASE("MECH-A4: a marker whose STEP fails stops the route with the step's verdict") {
    // Bug caught: the binder answering Succeeded (or Refused) for a step that TIMED_OUT — the route
    // would drive on with no piece, or blame the route author for the field.
    RobotX x;  // no piece ever comes
    static CommandRegistry registry;
    registry = CommandRegistry{};
    MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    routes.registerInto(registry);
    const cfg::Route route = oneMarker(mk("intake_until_piece", {12, 400}));
    const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry, x.sink,
                                                             x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
    CHECK(res.route.reason == RunStopCause::MarkerFailed);
    CHECK(x.count("RUN", "marker 'intake_until_piece' returned TIMED_OUT") == 1);
    CHECK(x.count("RTN", "routine 'intake_until_piece' STOPPED at step 1 (intake): mechanism step FAILED (TIMED_OUT)") ==
          1);
}

TEST_CASE("MECH-A4: every derived action runs its OWN step -- a lift's home is a lift's, a stop is a stop") {
    // Bug caught: the binder finding a LIFT through an Arm's lookup (units: the service never finds a lift
    // by an arm's declaration — every lift_home would be ABSENT), or a stop that does nothing.
    RobotX x{true, /*liftOnSwitchAtBoot=*/true};
    static CommandRegistry registry;
    registry = CommandRegistry{};
    MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    routes.registerInto(registry);
    cfg::Route r{.name = "every"};
    r.waypoints[0] = {.heading = shulib::math::Angle::degrees(0.0), .headingMode = cfg::HeadingMode::Fixed,
                      .motion = cfg::WaypointMotion::Start};
    r.waypoints[0].markers[0] = mk("lift_home", {1000});
    r.waypoints[0].markers[1] = mk("arm_home", {1000});
    r.waypoints[0].markers[2] = mk("lift_low", {3000});
    r.waypoints[0].markers[3] = mk("hood_middle");
    r.waypoints[0].markerCount = 4;  // kMaxMarkersPerWaypoint
    r.waypoints[1] = {.headingMode = cfg::HeadingMode::Free, .motion = cfg::WaypointMotion::Hold,
                      .constraints = {.timeout = shulib::units::Time{0.3}}};
    r.waypoints[1].markers[0] = mk("intake_run", {-6, 300});
    r.waypoints[1].markers[1] = mk("intake_stop");
    r.waypoints[1].markerCount = 2;
    r.count = 2;
    REQUIRE(r.valid());
    const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(r), &registry, x.sink,
                                                             x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
    CHECK(res.route.ok());
    CHECK(res.route.markersFired == 6);
    CHECK(x.count("MECH", "'lift': SUCCEEDED") == 2);  // its home, then 'low'
    CHECK(x.count("MECH", "'arm': SUCCEEDED") == 1);
    CHECK(x.count("MECH", "'hood': SUCCEEDED") == 1);
    CHECK(x.count("MECH", "'intake': SUCCEEDED -- time") == 1);
}

TEST_CASE("MECH-A4 P14: the MANIFESTS, byte for byte -- every derived id with its doc, both robots") {
    // Bug caught: a doc that drifts from what the id does, an arg count the picker would prompt wrongly
    // for, an id published out of the list's order, or a doc the manifest cannot carry.
    RobotX x;
    CommandRegistry rx;
    MechanismRoutes routesX{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    routesX.registerInto(rx);
    shulib::hal::fake::FakeCharSink mx;
    shulib::runner::writeManifest(rx, mx);
    const std::string expectX =
        std::string{"{\"manifestSchemaVersion\":1,\"builtins\":4,\"commands\":17}\n"} + kBuiltinLines +
        "{\"id\":\"arm_stow\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"arm: move to 'stow' and hold it; "
        "args[0] = time limit, ms\"}\n"
        "{\"id\":\"arm_score\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"arm: move to 'score' and hold "
        "it; args[0] = time limit, ms\"}\n"
        "{\"id\":\"arm_high\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"arm: move to 'high' and hold it; "
        "args[0] = time limit, ms\"}\n"
        "{\"id\":\"arm_home\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"arm: find zero; args[0] = time "
        "limit, ms\"}\n"
        "{\"id\":\"lift_floor\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"lift: move to 'floor' and hold "
        "it; args[0] = time limit, ms\"}\n"
        "{\"id\":\"lift_low\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"lift: move to 'low' and hold it; "
        "args[0] = time limit, ms\"}\n"
        "{\"id\":\"lift_high\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"lift: move to 'high' and hold "
        "it; args[0] = time limit, ms\"}\n"
        "{\"id\":\"lift_home\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"lift: find zero; args[0] = time "
        "limit, ms\"}\n"
        "{\"id\":\"flywheel_shoot\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"flywheel: spin up to "
        "'shoot' and keep spinning; args[0] = time limit, ms\"}\n"
        "{\"id\":\"flywheel_stop\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"flywheel: stop (its safe "
        "state)\"}\n"
        "{\"id\":\"intake_run\",\"builtin\":false,\"minArgs\":2,\"maxArgs\":2,\"doc\":\"intake: run; args[0] = volts "
        "(-12..12), args[1] = how long, ms\"}\n"
        "{\"id\":\"intake_until_piece\",\"builtin\":false,\"minArgs\":2,\"maxArgs\":2,\"doc\":\"intake: run until it "
        "sees a piece; args[0] = volts, args[1] = time limit, ms\"}\n"
        "{\"id\":\"intake_until_gone\",\"builtin\":false,\"minArgs\":2,\"maxArgs\":2,\"doc\":\"intake: run until the "
        "piece is gone; args[0] = volts, args[1] = time limit, ms\"}\n"
        "{\"id\":\"intake_stop\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"intake: stop (its safe "
        "state)\"}\n"
        "{\"id\":\"hood_down\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"hood: go to 'down'\"}\n"
        "{\"id\":\"hood_middle\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"hood: go to 'middle'\"}\n"
        "{\"id\":\"hood_up\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"hood: go to 'up'\"}\n";
    CHECK(mx.text() == expectX);

    RobotTwo two;
    CommandRegistry r2;
    MechanismRoutes routes2{two.chassis, two.svc, mech_fixture::robot_two::kRobotTwoMechanisms, two.sink};
    routes2.registerInto(r2);
    shulib::hal::fake::FakeCharSink m2;
    shulib::runner::writeManifest(r2, m2);
    const std::string expect2 =
        std::string{"{\"manifestSchemaVersion\":1,\"builtins\":4,\"commands\":6}\n"} + kBuiltinLines +
        "{\"id\":\"claw_pivot_6_run\",\"builtin\":false,\"minArgs\":2,\"maxArgs\":2,\"doc\":\"claw pivot 6: run; "
        "args[0] = volts (-12..12), args[1] = how long, ms\"}\n"
        "{\"id\":\"claw_pivot_6_stop\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"claw pivot 6: stop (its "
        "safe state)\"}\n"
        "{\"id\":\"claw_pivot_7_run\",\"builtin\":false,\"minArgs\":2,\"maxArgs\":2,\"doc\":\"claw pivot 7: run; "
        "args[0] = volts (-12..12), args[1] = how long, ms\"}\n"
        "{\"id\":\"claw_pivot_7_stop\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"claw pivot 7: stop (its "
        "safe state)\"}\n"
        "{\"id\":\"claw_closed\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"claw: go to 'closed'\"}\n"
        "{\"id\":\"claw_open\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"claw: go to 'open'\"}\n";
    CHECK(m2.text() == expect2);
    CHECK(std::string{routes2.doc(0)} == "claw pivot 6: run; args[0] = volts (-12..12), args[1] = how long, ms");
    CHECK_THROWS_AS((void)routes2.doc(6), shulib::PreconditionError);
}

TEST_CASE("MECH-A4: the registry's DOC -- published per entry, absent ones still \"\", a quote refused at "
          "registration; registerInto twice is a duplicate") {
    // Bug caught (F8-additive): a doc'd entry beside undoc'd ones shifting the bytes of the undoc'd
    // ones; a doc with a quote reaching the manifest (a file no picker can parse); a program that
    // registers its mechanism ids twice silently overwriting the first.
    auto fn = [] {};
    CommandRegistry r;
    r.on("intake_in", fn);
    shulib::runner::Handler h{[](void*, const cfg::Marker&) { return MechanismOutcome::Succeeded; }, nullptr};
    r.on("lift_to", h, 1, "lift: to a level; args[0] = level");
    r.on("bare", h, 0, nullptr);
    shulib::hal::fake::FakeCharSink m;
    shulib::runner::writeManifest(r, m);
    CHECK(m.text() == std::string{"{\"manifestSchemaVersion\":1,\"builtins\":4,\"commands\":3}\n"} + kBuiltinLines +
                          "{\"id\":\"intake_in\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"\"}\n"
                          "{\"id\":\"lift_to\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"lift: to a "
                          "level; args[0] = level\"}\n"
                          "{\"id\":\"bare\",\"builtin\":false,\"minArgs\":0,\"maxArgs\":0,\"doc\":\"\"}\n");
    CHECK(r.ids()[1].doc != nullptr);
    CHECK(r.ids()[0].doc == nullptr);
    CHECK_THROWS_AS(r.on("quoted", h, 0, "a \"quote\""), shulib::PreconditionError);
    CHECK_THROWS_AS(r.on("slashed", h, 0, "a \\ slash"), shulib::PreconditionError);
    CHECK(r.count() == 3);  // nothing registered by the refused calls
    CHECK_THROWS_AS(r.on("lift_to", h, 0, "again"), shulib::PreconditionError);  // the 3-arg form's duplicate

    RobotTwo two;
    CommandRegistry once;
    MechanismRoutes routes{two.chassis, two.svc, mech_fixture::robot_two::kRobotTwoMechanisms, two.sink};
    routes.registerInto(once);
    CHECK_THROWS_AS(routes.registerInto(once), shulib::PreconditionError);
    CHECK(once.count() == 6);
}

TEST_CASE("MECH-A4: the binder's ids are routeIdsOf()'s, in the list's order") {
    // Bug caught: a binder that derives its own ids (two rules that would drift) or registers them out
    // of the list's order (the manifest's order is part of its contract).
    RobotX x;
    MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    const shulib::mechanism::RouteIds want =
        shulib::mechanism::routeIdsOf(mech_fixture::robot_x::kRobotXMechanisms.specSpan());
    REQUIRE(routes.ids().count == want.count);
    for (std::size_t i = 0; i < want.count; ++i) {
        CHECK(routes.ids().ids[i].view() == want.ids[i].view());
    }
    CHECK(routes.ids().ids[0].view() == "arm_stow");
    CHECK(routes.ids().ids[16].view() == "hood_up");
}

// ═══ MECH-A4's review round (2026-10-27) ═══════════════════════════════════════════════════════

TEST_CASE("MECH-A4 (review): <power>_until_gone runs until the piece is GONE; a marker's ms are SECONDS "
          "of the step (intake_run [12, 500] = 0.50 s)") {
    // Bug caught: until_gone bound to "until it SEES a piece" (an eject marker that runs until a piece
    // arrives), or a marker's milliseconds converted wrongly (a 500 ms run lasting a second).
    {
        RobotX x;
        x.world.sensorSeesAt = 0.0;  // a piece is in the intake from the start …
        x.world.sensorGoneAt = 0.6;  // … and leaves 0.6 s in
        x.intakeSensor.setDistance(shulib::units::Length{1.0});  // seen before the first pace too
        static CommandRegistry registry;
        registry = CommandRegistry{};
        MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
        routes.registerInto(registry);
        const cfg::Route route = oneMarker(mk("intake_until_gone", {-6, 2000}));
        const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry,
                                                                 x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
        CHECK(res.route.ok());
        CHECK(x.count("MECH", "'intake': SUCCEEDED -- stopped seeing a piece") == 1);
        CHECK(x.svc.power(3).lastStep().doneBy == shulib::mechanism::DoneBy::StopsSeeing);
        CHECK(x.svc.power(3).lastStep().elapsed > 0.5);  // it ran until the piece LEFT, 0.6 s in
        CHECK(x.svc.power(3).lastStep().elapsed < 1.0);
    }
    {
        RobotX x;
        static CommandRegistry registry;
        registry = CommandRegistry{};
        MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
        routes.registerInto(registry);
        const cfg::Route route = oneMarker(mk("intake_run", {12, 500}));
        const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry,
                                                                 x.sink, x.svc, auton_rig::kPeriod);
        REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
        CHECK(res.route.ok());
        CHECK(x.count("MECH", "'intake': SUCCEEDED -- time in 0.50s") == 1);
        CHECK(x.svc.power(3).lastStep().elapsed >= 0.5);
        CHECK(x.svc.power(3).lastStep().elapsed < 0.52);
    }
}

TEST_CASE("MECH-A4 (review): a REFUSED marker's line is a WARN") {
    // Bug caught: the binder's "route marker '…' REFUSED: it needs …" logged below Warn — the one line
    // that says why the route stopped, dropped by a level filter on the brain.
    RobotX x;
    static CommandRegistry registry;
    registry = CommandRegistry{};
    MechanismRoutes routes{x.chassis, x.svc, mech_fixture::robot_x::kRobotXMechanisms, x.sink};
    routes.registerInto(registry);
    const cfg::Route route = oneMarker(mk("arm_score"));
    const AutonRunResult res = shulib::competition::runAuton(x.chassis, x.guard, routeEntry(route), &registry, x.sink,
                                                             x.svc, auton_rig::kPeriod);
    REQUIRE(res.outcome == AutonRunOutcome::RouteRan);
    CHECK(x.countAt(shulib::hal::LogLevel::Warn, "MECH",
                    "route marker 'arm_score' REFUSED: it needs its time limit in ms, above 0 (args[0])") == 1);
}
