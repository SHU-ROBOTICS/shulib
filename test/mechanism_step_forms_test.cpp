// MECH-A4 — THE POSITIVE TU of the compile-refusal harness (the MECH brief's §3.0: neg5/p01.cpp +
// probe4_forms.cpp + team5/counted_steps.cpp), compiled INTO the suite: every auton step form and every
// driver control at its exact limits, and every step and control the brief's §1.1 count table quotes,
// verbatim. A word that stopped compiling fails the suite's build here — the twin of every refusal TU
// that must keep failing. The counted autons then RUN on the host test plant.

#include "doctest.h"

#include "mechanism_auton_rig.hpp"
#include "mechanism_driver_fixtures.hpp"
#include "mechanism_fixtures.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/mechanism/driver_inputs.hpp"

namespace forms {

// ── p01: every step form, at the exact limits ───────────────────────────────────────────
namespace x {
using namespace mech_fixture::robot_x;
inline void everyForm(Routine& r, double computed) {
    r.move(arm["high"], 1_s).move(arm.to(160), 1_s).move(arm.to(0), 1_s).move(arm.clampedTo(computed), 1_s);
    r.move(lift["low"], 1_s).move(lift.to(24_in), 1_s).move(lift.to(0_in), 1_s).move(lift.clampedTo(18_in), 1_s);
    r.spinUp(flywheel.at(3600), 1_s).spinUp(flywheel.at(-3600), 1_s).spinUp(flywheel.clampedTo(computed), 1_s);
    r.set(hood["middle"]);
    r.run(intake, 12_volt, seesPiece, 1_s).run(intake, -12_volt, stopsSeeingPiece, 1_s);
    r.run(intake, 12_volt, seesPiece(intake), 1_s).run(intake, 12_volt, seesPiece.orAfter(0.5_s), 1_s);
    r.run(intake, 12_volt, 1_s).run(intake, 12_volt, 1_s, "named");
    r.startMove(arm["score"], 1_s).startHome(lift, 1_s).startSpinUp(flywheel["shoot"], 1_s);
    r.startRun(intake, 12_volt, seesPiece, 1_s).startRun(intake, 6_volt, 1_s);
    r.waitUntilDone(arm, 1_s).waitUntilDone(lift, 1_s).waitUntilDone(flywheel, 1_s).waitUntilDone(intake, 1_s);
    r.stop(intake).stop(flywheel).home(arm, 1_s).home(lift, 1_s);
    (void)r.lastMechanism();
}
// THE COUNTED STEPS (the brief's §1.1 table, verbatim).
RoutineResult counted(AutonContext& ctx) {
    Routine r{ctx, "counted"};
    r.move(arm["score"], 1.5_s);
    r.move(lift.to(18_in), 4_s);
    r.spinUp(flywheel["shoot"], 2_s);
    r.startSpinUp(flywheel["shoot"], 2_s);
    r.waitUntilDone(flywheel, 1_s);
    r.run(intake, 12_volt, seesPiece, 3_s);
    r.run(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);
    r.set(hood["up"]);
    return r.result();
}
// Every control at its limits: 12 V exactly, the least time limit a double can state above 0.
inline constexpr DriverControls kLimits = controls(kRobotXMechanisms,
    whileHeld(arm, Button::R1, Button::R2, 12_volt),
    preset(arm.to(160), Button::Up, 1e-6_s),
    preset(arm.to(0), Button::Down, 1_s),
    whileHeld(lift, Button::L1, Button::L2, 0.001_volt),
    homeOnPress(lift, Button::Left, 1_s),
    homeAtDriverStart(lift, 1_s),
    toggle(flywheel["shoot"], Button::X),
    preset(flywheel.at(-3600), Button::B, 1_s),
    whileHeld(intake, Button::A, Button::Right, 12_volt),
    preset(hood["down"], Button::Y));
static_assert(kLimits.count == 10);
}  // namespace x

namespace two {
using namespace mech_fixture::robot_two;
// THE COUNTED STEPS, robot two's (the brief's §1.1 table, verbatim).
RoutineResult counted(AutonContext& ctx) {
    Routine r{ctx, "counted-two"};
    r.run(clawPivot6, 12_volt, 0.4_s);
    r.run(clawPivot7, 12_volt, 0.4_s);
    r.set(claw["closed"]);
    return r.result();
}
}  // namespace two

// ── probe4_forms: EVERY optional word, robot y's (the brief's P5b) ───────────────────────
namespace y {
using namespace mech_fixture::robot_y;
RoutineResult everyStep(AutonContext& ctx) {
    Routine r{ctx, "every-step"};
    r.home(swing, 1.5_s);
    r.startHome(dr4b, 1.5_s).driveTo(12_in, 0_in).waitUntilDone(dr4b, 0.5_s);
    r.move(swing["up"], 1.5_s).move(dr4b.to(30_in), 3_s).move(dr4b.clampedTo(r.chassis().pose().x()), 1_s);
    r.run(roller, 12_volt, seesPiece, 2_s).run(roller, -6_volt, stopsSeeingPiece, 1_s);
    r.run(roller, 12_volt, seesPiece(grip), 2_s);         // until the GRIP's sensor sees one
    r.run(roller, 12_volt, seesPiece.orAfter(1_s), 2_s);  // ... or after 1 s
    r.run(catapult, 12_volt, seesPiece, 2_s);             // wind until its switch is pressed ("loaded")
    r.run(catapult, 6_volt, stalls, 1_s);                 // a squeeze: done when it stalls
    r.set(grip["shut"]).set(grip["loose"]);
    return r.result();
}
}  // namespace y

// ── U5's ACCEPTING side (MECH-A4's review round): a switch- or stall-homed mechanism with its
//    `.unhomedVolts` and NO homing control compiles when it is only HELD (its holds move it unhomed).
//    A preset beside it is refused (compile_refusals/n48d) — a jog never homes it. ───────────────
namespace u5 {
using namespace shulib::mechanism::words;
inline constexpr Lift slide{.name = "slide", .motors = {Forward}, .cartridge = Red, .gearing = direct,
                            .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s},
                            .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt),
                            .home = homeBySwitch(0_in, -3_volt), .unhomedVolts = 4_volt};
inline constexpr Arm flap{.name = "flap", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .safe = Brake,
                          .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 120},
                          .gravity = noGravity,
                          .home = homeByStall(0, -3_volt, {.amps = 0.45, .belowMotorRpm = 2, .lasting = 0.2_s}),
                          .unhomedVolts = 3_volt};
inline constexpr MechanismTable kHeldOnly = mechanisms(wire(slide, motors(1), homeSwitch('B')), wire(flap, motors(2)));
inline constexpr DriverControls kHeldOnlyControls =
    controls(kHeldOnly, whileHeld(slide, Button::L1, Button::L2, 12_volt), whileHeld(flap, Button::R1, Button::R2, 6_volt));
static_assert(kHeldOnlyControls.count == 2);
}  // namespace u5

}  // namespace forms

TEST_CASE("MECH-A4 forms: the counted autons, verbatim, run to SUCCEEDED on the host test plant") {
    // Bug caught: a counted line of the brief's §1.1 that compiles but does not run as it reads.
    {
        auton_rig::RobotX x;
        x.world.sensorSeesAt = 2.5;
        const auto res = shulib::competition::runAuton(x.chassis, x.guard, auton_rig::entryFor("counted", &forms::x::counted),
                                                       nullptr, x.sink, x.svc, auton_rig::kPeriod);
        CHECK(res.routine.ok);
        CHECK(res.routine.completed == 8);
    }
    {
        auton_rig::RobotTwo two;
        const auto res = shulib::competition::runAuton(two.chassis, two.guard,
                                                       auton_rig::entryFor("counted-two", &forms::two::counted), nullptr,
                                                       two.sink, two.svc, auton_rig::kPeriod);
        CHECK(res.routine.ok);
        CHECK(res.routine.completed == 3);
    }
    // the forms above are compiled — their functions exist; the step chains are the build's proof
    CHECK(&forms::x::everyForm != nullptr);
    CHECK(&forms::y::everyStep != nullptr);
}
