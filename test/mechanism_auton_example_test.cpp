// MECH-A4 — the code the mechanism chapter's auton, driver and route sections and the cookbook's
// mechanism recipe quote (docs/guide/18-mechanisms.md, cases guide-18c..guide-18f; docs/cookbook/
// 07-mechanisms.md, case cookbook-07a): every quoted line is here VERBATIM, compiled, and run on the
// host test plant (the doc gate's check-examples finds each line in this file). Robot x is
// test/mechanism_fixtures.hpp's. Nothing here has run on a robot.

#include "doctest.h"

#include <algorithm>
#include <cmath>

#include "mechanism_auton_rig.hpp"
#include "mechanism_driver_fixtures.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/route.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/manifest.hpp"
#include "shulib/runner/mechanism_routes.hpp"

namespace runner = shulib::runner;

namespace guide18 {
using namespace mech_fixture::robot_x;  // the robot's own namespace: its declarations and the words

// ── guide-18c: an auton, one line per step ──────────────────────────────────────────────
RoutineResult scoreThree(AutonContext& ctx) {
    Routine r{ctx, "score-three"};
    r.startSpinUp(flywheel["shoot"], 2_s);         // the flywheel spins up while the robot drives
    r.startRun(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);  // until it sees a piece (or 2.5 s)
    r.driveTo(24_in, 0_in);
    r.waitUntilDone(intake, 1_s);
    r.move(arm["score"], 1.5_s);                    // the arm HOLDS 95 degrees from here on
    r.move(lift.to(18_in), 4_s);
    r.set(hood["up"]);
    r.waitUntilDone(flywheel, 1_s);
    r.run(intake, 12_volt, 0.5_s);                  // feed the piece into the flywheel
    r.stop(flywheel);
    return r.result();
}

// ── guide-18d: the driver file's controls ───────────────────────────────────────────────
inline constexpr DriverControls kControls = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 6_volt),  // up / down while held; let go: it holds there
    preset(arm["stow"], Button::Down, 1.5_s),
    preset(arm["score"], Button::Right, 1.5_s),      // the most recent button wins
    preset(arm["high"], Button::Up, 1.5_s),
    whileHeld(lift, Button::L1, Button::L2, 12_volt),
    homeAtDriverStart(lift, 1.5_s),                  // after a restart it may not be on its switch
    toggle(flywheel["shoot"], Button::X),            // on at "shoot" / off
    whileHeld(intake, Button::A, Button::B, 12_volt),
    autonOnly(hood));                                // the hood is the autons' alone

// ── cookbook-07a: an arm with presets, held while the robot drives ──────────────────────
RoutineResult scoreHigh(AutonContext& ctx) {
    Routine r{ctx, "score-high"};
    r.move(arm["score"], 1.5_s);  // the arm moves, then HOLDS 95 degrees ...
    r.driveTo(40_in, 0_in);       // ... while the robot drives
    r.move(arm["high"], 1.5_s);   // then on up, and it holds there
    return r.result();
}
}  // namespace guide18

TEST_CASE("guide-18c: the chapter's auton, one line per step, runs every step to SUCCEEDED") {
    auton_rig::RobotX x;
    x.world.sensorSeesAt = 1.5;
    const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("score-three", &guide18::scoreThree),
                                                   nullptr, x.sink, x.svc, auton_rig::kPeriod);
    CHECK(res.routine.ok);
    CHECK(res.routine.completed == 10);
}

TEST_CASE("guide-18d: the chapter's driver controls are checked at build and bind to the robot") {
    static_assert(guide18::kControls.count == 9);
    auton_rig::RobotX x;
    shulib::mechanism::DriverInputs inputs;
    inputs.bind(x.svc, guide18::kControls);
    CHECK(inputs.bound());
    CHECK(inputs.avoids(shulib::hal::ControllerButton::Y));
}

TEST_CASE("guide-18e: a robot's route ids, registered in ONE call, published with their docs") {
    using namespace mech_fixture::robot_x;
    auton_rig::RobotX x;
    shulib::chassis::Chassis& chassis = x.chassis;
    shulib::mechanism::MechanismService& service = x.svc;
    shulib::hal::ITelemetrySink& telemetry = x.sink;
    static runner::CommandRegistry registry;
    registry = runner::CommandRegistry{};
    static runner::MechanismRoutes routes{chassis, service, kRobotXMechanisms, telemetry};
    routes.registerInto(registry);
    CHECK(registry.count() == 17);
    shulib::hal::fake::FakeCharSink manifest;
    runner::writeManifest(registry, manifest);
    CHECK(manifest.text().find("{\"id\":\"arm_score\",\"builtin\":false,\"minArgs\":1,\"maxArgs\":1,\"doc\":\"arm: move to "
                               "'score' and hold it; args[0] = time limit, ms\"}") != std::string::npos);
}

TEST_CASE("cookbook-07a: an arm with presets, HELD while the robot drives") {
    auton_rig::RobotX x;
    double worst = 0.0;
    bool sampling = false;
    // sampled only while the 40 in drive runs with the arm holding "score" (not the end-of-run brake,
    // by when it holds "high")
    x.world.onPace = [&] {
        if (x.chassis.scheduler().hasActiveMotion() && x.svc.position(0).target() == 95.0) {
            sampling = true;
            worst = std::max(worst, std::abs(x.armPlant.trueDeg() - 95.0));
        }
    };
    const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("score-high", &guide18::scoreHigh),
                                                   nullptr, x.sink, x.svc, auton_rig::kPeriod);
    CHECK(res.routine.ok);
    CHECK(res.routine.completed == 3);
    CHECK(sampling);
    CHECK(worst < 1.5);  // the hold through the 40 in drive (the chapter's P1 numbers: 1.16 deg)
    CHECK(std::abs(x.armPlant.trueDeg() - 140.0) < 3.0);  // then "high" (140), measured before the run's end safed it
}
