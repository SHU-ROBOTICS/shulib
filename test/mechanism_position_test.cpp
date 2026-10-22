// mechanism/service.hpp — the POSITION behaviour (an Arm, a Lift) on the host test plant (chunk
// MECH-A3, the MECH brief's §1.7 and §3.3). Say "host test plant", never "simulator": every plant
// constant is INVENTED (HA-183), and every controller number is the library's STARTING value (HA-182)
// unless a case says otherwise.
//
// ═══ WHAT THIS FILE TRIES TO BREAK ══════════════════════════════════════════════════
//   * THE MOVE (P2a): 144 moves over a deliberately MISMATCHED grid — 2 cartridges × 2 gearings ×
//     plant kV ×0.8/1/1.25 × plant kS 0.3/0.6 × plant kG 1.2/1.6/2.4 against the controller's 1.6 —
//     with the starting values, never tuned to the plant. Every SUCCEEDED verdict must be TRUE (the
//     plant within the tolerance at that moment); a profile the plant cannot follow must end
//     TIMED_OUT, never SUCCEEDED (finding 4).
//   * THE HOLD (P2b) under a piece's load, against a CONTROL nobody ticks (so the test can see a sag);
//     the hold's INTEGRAL earns its place (the ablation: without it, 3.2 deg of steady error under a
//     25 % gravity mismatch).
//   * A BLOCKED move (P2c): the stated signature STALLS it; one that cannot trip in time lets it TIME
//     OUT — and then it HOLDS WHERE IT STOPPED (N2), never pushing on at stall current.
//   * THE LIMITS (P2d; the re-check's D2, D4): a target AT a limit is reached; a COMPUTED one past it
//     is clamped and logged ONCE; a push past a limit is held AT it by the closed-loop hold, at both
//     limits, with a heavier plant than the controller believes — on a slow, a medium and a fast arm
//     at 4, 8 and 12 V, ONE soft stop per push however long it is held, never pushed toward the limit
//     harder than the push, taken over only where it could no longer stop (never far away, never while
//     it moves away); no hold target is ever past a limit; "reached" is judged in the mechanism's own
//     units at a table's end and a linkage's top, and a linkage's or a table's plan never asks the
//     motor for more than its share of the free speed.
//   * THE REVIEW'S HOLES (MECH-A3's review, 2026-10-22): the feedforward terms the starting values
//     leave at 0 (kS, kA) and a lift's constant gravity; both integral clamps and the speed integral's
//     freeze; "reached" for an UNBROKEN settle time; the re-home Warn's threshold at a table's end; every
//     reset a new command makes (each stall window, each step clock, the move loop); find()'s units and
//     preset values; a homing step's bound at its exact edge; the absolute turn above the middle and
//     the one-turn window's edge.
//   * EVERY HOMING MOMENT (P2e, P2h; the re-check's D10 and S): rests at its stop, an absolute sensor
//     (mounted either way, its reading wrapped through 0), a switch already pressed, a homing push by
//     switch and by stall — and a switch that is DEAD (HA-184 wrong: it reads "pressed" from boot) never
//     homes it; every (re-)home logs where its zero went.
//   * THE STALL WATCH ON EVERY BEHAVIOUR (N10): a hold after SUCCEEDED (case A), a driver's jog into an
//     obstacle, a homing push by switch — and the command number, so a held button is superseded.
// Thresholds are the measured number plus a stated margin, quoted beside each (the A3 probe,
// `scratch/mech-a3/probes/p2.txt` — the brief's P2 reproduced to the hundredth on this code).

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <span>
#include <string>
#include <vector>

#include "mechanism_fixtures.hpp"
#include "mechanism_position_rig.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_device_provider.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/mechanism_parts.hpp"

namespace {

namespace mech = shulib::mechanism;
namespace sim = shulib::sim;
namespace units = shulib::units;
namespace rx = mech_fixture::robot_x;
namespace ry = mech_fixture::robot_y;
using mech::MechanismOutcome;
using pos_rig::ArmRig;
using pos_rig::armPlant;
using pos_rig::Wiring;
using namespace shulib::mechanism::words;

// The arms the brief's probes declare (P2): only the cartridge and the gearing vary across the grid;
// every one rests at its stop at 0, holds 1.6 V level at 45 deg, and states the robot file's
// signature (2.0 A below 5 motor rpm for 0.5 s).
constexpr Arm kRed5{.name = "arm", .motors = {Forward, Forward}, .cartridge = Red, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0), .presets = {{"stow", 0}, {"score", 95}, {"high", 140}}};
constexpr Arm kRed7{.name = "arm", .motors = {Forward, Forward}, .cartridge = Red, .gearing = gears(12, 84), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0)};
constexpr Arm kGreen5{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0), .presets = {{"stow", 0}, {"score", 95}, {"high", 140}}};
constexpr Arm kGreen7{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 84), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0)};
// P2c's quicker signature, and the homing variants (Green 5:1)
constexpr Arm kStalls{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = Stall{.amps = 1.6, .belowMotorRpm = 3, .lasting = 0.15_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0)};
constexpr Arm kStallHome{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = homeByStall(0, -3_volt, Stall{.amps = 0.45, .belowMotorRpm = 3, .lasting = 0.1_s})};
constexpr Arm kSwitchHome{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = homeBySwitch(0, -3_volt)};
constexpr Arm kNeverHomed{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = neverHomed};
constexpr Arm kNeverHomed4V{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = neverHomed, .unhomedVolts = 4_volt};
constexpr Arm kAbsolute{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .angleSensor = rotationSensor(), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = absoluteSensor(0, 0)};
constexpr Arm kAbsoluteReversed{.name = "arm", .motors = {Forward, Forward}, .cartridge = Green, .gearing = gears(12, 60), .angleSensor = rotationSensor(Reversed), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = absoluteSensor(0, 0)};
// THE WRAP: the sensor reads 350 when the arm is at 0, so anywhere above 10 deg it reads past 360 —
// wrapped to 0.. at power-up (HA-186's belief, modelled by the plant's RotationMount)
constexpr Arm kAbsoluteWraps{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .angleSensor = rotationSensor(), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = absoluteSensor(0, 350)};
// the soft-stop arms of the re-check (adv_extra (1), adv_softstop): limits well inside the hard stops
constexpr Arm kInside{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {20, 140}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(20)};
// the lifts
constexpr Lift kLift{.name = "lift", .motors = {Forward, Reversed}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in), .presets = {{"floor", 0_in}, {"low", 8_in}, {"high", 22_in}}};
constexpr HeightAt kPairs[] = {{0_in, 0}, {6_in, 30}, {14_in, 70}, {20_in, 120}};
constexpr Lift kTable{.name = "tilt", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = measured(kPairs), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 20_in}, .gravity = noGravity, .home = restsAtStop(0_in), .presets = {{"top", 20_in}}};
constexpr Lift kDr4b{.name = "dr4b", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = linkage(14_in, 10_in, 2), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {-18_in, 38_in}, .gravity = noGravity, .home = restsAtStop(-18_in)};

constexpr double kPi = 3.14159265358979323846;

/// The message a call fails its precondition with, or "" when it does not.
template <class F>
std::string preconditionOf(F&& f) {
    try {
        f();
    } catch (const shulib::PreconditionError& e) {
        return e.what();
    }
    return {};
}

/// One move, watched: its verdict, its time, the worst overshoot past the target (in the move's
/// direction) through the verdict and 0.5 s after it, the final error, and the TRUE error the verdict
/// was judged on (the plant's state the service read on the verdict's tick).
struct Watched {
    MechanismOutcome outcome;
    double elapsed;
    double overshoot;
    double finalError;
    double errorAtVerdict;
};
Watched moveAndWatch(ArmRig& r, double target, double bound, bool computed = false) {
    r.slot().moveTo(target, units::Time{bound}, computed);
    const double dir = target >= r.trueDeg() ? 1.0 : -1.0;
    double over = 0.0;
    double atVerdict = r.trueDeg() - target;
    int n = 0;
    while (r.slot().running() && n < 2000) {
        // the state the service reads this tick is the plant's BEFORE the tick's world step (the
        // encoders read the truth here: no slip, no noise)
        atVerdict = r.trueDeg() - target;
        r.tick();
        ++n;
        over = std::max(over, dir * (r.trueDeg() - target));
    }
    for (int i = 0; i < 50; ++i) {
        r.tick();
        over = std::max(over, dir * (r.trueDeg() - target));
    }
    return {r.slot().lastStep().outcome, r.slot().lastStep().elapsed, over, r.trueDeg() - target, atVerdict};
}
/// The profile's own ideal time for `dist` (P2a's bound is 2.5 times it, at least 2 s).
double idealTime(const mech::PositionSpec& p, double dist) {
    const double v = p.maxVelocity;
    const double a = p.maxAcceleration;
    return dist > v * v / a ? dist / v + v / a : 2.0 * std::sqrt(dist / a);
}
/// A spool lift's plant: the joint is the spool, constant gravity, stops just past 0 and 24.5 in.
sim::ArmPlantConfig spoolPlant(double kG, double kvx, double startDeg = 0.0) {
    sim::ArmPlantConfig pc = armPlant(100, 1, kG, kvx, 0.3, startDeg);
    pc.constantGravity = true;
    pc.minStopDeg = -1.0;
    pc.maxStopDeg = 24.5 / (kPi * 1.5) * 360.0;
    return pc;
}

}  // namespace

TEST_CASE("MECH-A3 position: P2a -- 144 MISMATCHED moves with the starting values; every SUCCEEDED is TRUE") {
    // Measured (the A3 probe): 143/144 SUCCEEDED in 0.69..3.15 s; overshoot <= 7.22 deg (<= 3.59 with a
    // plant no faster than its cartridge — a real motor cannot beat it); final error <= 2.55 deg; the
    // one miss is a plant SLOWER than its own cartridge (Green 7:1, kS 0.6, kG 2.4 -> 90: TIMED_OUT).
    // Margins: overshoot 7.5 / 3.8, final 2.6 (the brief's §3.3).
    int total = 0;
    int ok = 0;
    double worstOver = 0.0;
    double worstOverPhysical = 0.0;
    double worstFinal = 0.0;
    std::vector<std::string> misses;
    for (const Arm* a : {&kRed5, &kRed7, &kGreen5, &kGreen7}) {
        const mech::MechanismSpec spec = a->spec();
        for (const double kvx : {0.8, 1.0, 1.25}) {
            for (const double kS : {0.3, 0.6}) {
                for (const double plantKg : {1.2, 1.6, 2.4}) {
                    ArmRig r{spec, armPlant(pos_rig::cartridgeRpm(spec.cartridge), spec.position.gearRatio, plantKg, kvx, kS)};
                    double from = 0.0;
                    for (const double target : {90.0, 150.0}) {
                        const double bound = std::max(2.0, 2.5 * idealTime(spec.position, std::abs(target - from)));
                        const Watched w = moveAndWatch(r, target, bound);
                        ++total;
                        CAPTURE(kvx);
                        CAPTURE(kS);
                        CAPTURE(plantKg);
                        CAPTURE(target);
                        if (kvx >= 1.0) {
                            worstOverPhysical = std::max(worstOverPhysical, w.overshoot);
                        }
                        if (w.outcome == MechanismOutcome::Succeeded) {
                            ++ok;
                            // a SUCCEEDED verdict is TRUE: the plant was inside the tolerance (1.5 deg) then
                            CHECK(std::abs(w.errorAtVerdict) <= spec.position.tolerance);
                            worstOver = std::max(worstOver, w.overshoot);
                            worstFinal = std::max(worstFinal, std::abs(w.finalError));
                        } else {
                            CHECK(w.outcome == MechanismOutcome::TimedOut);  // never STALLED, never anything else
                            misses.push_back(std::string{shulib::hal::cartridgeWord(spec.cartridge)} + " " +
                                             std::to_string(spec.position.gearRatio) + " kvx " + std::to_string(kvx) +
                                             " kS " + std::to_string(kS) + " kG " + std::to_string(plantKg) + " -> " +
                                             std::to_string(target));
                        }
                        from = target;
                    }
                }
            }
        }
    }
    CHECK(total == 144);
    CHECK(ok >= 143);
    CHECK(worstOver <= 7.5);
    CHECK(worstOverPhysical <= 3.8);
    CHECK(worstFinal <= 2.6);
    REQUIRE(misses.size() <= 1U);
    if (!misses.empty()) {  // the stated case, and only it: a plant slower than its cartridge, heavy, sticky
        CHECK(misses[0].find("GREEN") != std::string::npos);
        CHECK(misses[0].find("kvx 1.000000 kS 0.600000 kG 2.400000 -> 90") != std::string::npos);
    }
}

TEST_CASE("MECH-A3 position: a profile the plant CANNOT follow ends TIMED_OUT, never SUCCEEDED (finding 4)") {
    // A plant FOUR times slower than its cartridge (kVx 4): the profile is done long before the arm is
    // near. The step's bound is the profile's own time + 0.3 s: the verdict must be TIMED_OUT with the
    // arm truly far from 90 — "reached" waits for the plant, never for the plan.
    const mech::MechanismSpec spec = kGreen5.spec();
    ArmRig r{spec, armPlant(200, 5, 1.6, 4.0, 0.3)};
    const double bound = idealTime(spec.position, 90.0) + 0.3;
    const Watched w = moveAndWatch(r, 90.0, bound);
    CHECK(w.outcome == MechanismOutcome::TimedOut);
    CHECK(std::abs(w.errorAtVerdict) > spec.position.tolerance);
    CHECK(r.slot().lastStep().elapsed == doctest::Approx(bound).epsilon(0.01));
}

TEST_CASE("MECH-A3 position: REACHED waits for the smooth move to END -- even with a coarse tolerance and no settle time") {
    // A team's coarse "reached" (`within` 30 deg) and no settle time: the arm is inside 30 deg of 90 well
    // before its plan has finished. The verdict must still wait for the plan's end — the plan is the
    // move; the hold takes over only when it is done. Measured: SUCCEEDED at the plan's end + one tick.
    mech::MechanismSpec spec = kGreen5.spec();
    spec.position.tolerance = 30.0;
    spec.position.settleTime = 0.0;
    ArmRig r{spec, armPlant(200, 5, 1.6, 0.8, 0.3)};
    r.slot().moveTo(90.0, units::Time{3.0}, false);
    r.runStep();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.slot().lastStep().elapsed >= idealTime(spec.position, 90.0));
}

TEST_CASE("MECH-A3 position: a move after a LOADED hold starts its hold FRESH -- no integral carried in") {
    // A hold under a +3 V load winds its integral up to carry it; the load goes, a new move runs to 30.
    // Measured: after SUCCEEDED the arm stays within 0.1 deg of 30; with the integral CARRIED from the
    // loaded hold (both of the slot's resets removed — the run's own mutation) it rose to 37.6 deg.
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 1.6, 1.0, 0.3)};
    REQUIRE(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
    r.plant->addLoadVolts(3.0);
    r.tick(300);
    r.plant->addLoadVolts(0.0);
    r.slot().moveTo(30.0, units::Time{3.0}, false);
    r.runStep();
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    double worst = 0.0;
    for (int i = 0; i < 100; ++i) {
        r.tick();
        worst = std::max(worst, std::abs(r.trueDeg() - 30.0));
    }
    CHECK(worst <= 1.5);
}

TEST_CASE("MECH-A3 position: P2b -- the HOLD under a piece's load, against a control nobody ticks") {
    // Measured: worst 1.66 / 1.80 / 2.41 deg, -1.25 / -1.18 / -0.86 after 2 s, still holding; nobody
    // ticking (the plant's invented Hold-at-0-V stiffness alone, HA-183): -2.87 deg after 2 s.
    // Margins (the brief's §3.3): worst <= 2.6, within 1.4 after 2 s; the control sags >= 2.5.
    for (const double plantKg : {1.2, 1.6, 2.4}) {
        CAPTURE(plantKg);
        ArmRig r{kGreen5.spec(), armPlant(200, 5, plantKg, 1.0, 0.3)};
        REQUIRE(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
        r.plant->addLoadVolts(0.8);
        double worst = 0.0;
        for (int i = 0; i < 200; ++i) {
            r.tick();
            worst = std::max(worst, std::abs(r.trueDeg() - 90.0));
        }
        CHECK(worst <= 2.6);
        CHECK(std::abs(r.trueDeg() - 90.0) <= 1.4);
        CHECK(r.slot().holdsClaim());
        CHECK_FALSE(r.slot().running());  // the step is over; the HOLD goes on
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    }
    // THE CONTROL: the same load with nobody ticking the service — Hold at 0 V only.
    ArmRig c{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    REQUIRE(moveAndWatch(c, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
    c.mech->applySafeState();
    c.plant->addLoadVolts(0.8);
    for (int i = 0; i < 200; ++i) {
        c.world.pace();  // the world only
    }
    CHECK(90.0 - c.trueDeg() >= 2.5);
}

TEST_CASE("MECH-A3 position: the hold's INTEGRAL earns its place -- the ablation, and a FRESH hold loop") {
    // Plant kG 1.2 against the controller's 1.6 — the arm is 25 % lighter than the team believes — held
    // level (45 deg), plant kS 0.3. Measured: with the integral +0.001 deg after 4 s; WITHOUT it (kI 0)
    // +3.20 deg of steady error, the step TIMED_OUT (it never got within 1.5). Margins: <= 0.05 with
    // it; >= 2 without (the brief's §3.3).
    const auto settle = [](mech::MechanismSpec spec) {
        ArmRig r{spec, armPlant(200, 5, 1.2, 1.0, 0.3)};
        r.slot().moveTo(45.0, units::Time{3.0}, false);
        r.runStep();
        r.tick(400);
        return r.trueDeg() - 45.0;
    };
    mech::MechanismSpec withI = kGreen5.spec();
    mech::MechanismSpec withoutI = withI;
    withoutI.position.kI = 0.0;
    CHECK(std::abs(settle(withI)) <= 0.05);
    CHECK(std::abs(settle(withoutI)) >= 2.0);
}

TEST_CASE("MECH-A3 position: P2c -- a BLOCKED move STALLS on the stated signature; one that cannot trip in time TIMES OUT and HOLDS WHERE IT STOPPED (N2)") {
    // An obstacle at 60 deg, a move to 120 within 3 s (plant kG 2.0, Green 5:1).
    SUBCASE("the robot file's signature, 2.0 A below 5 motor rpm for 0.5 s: STALLED (measured 1.00 s; margin 1.2)") {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.plant->setObstacleDeg(60.0);
        r.slot().moveTo(120.0, units::Time{3.0}, false);
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
        CHECK(r.slot().lastStep().doneBy == mech::DoneBy::Stalled);
        CHECK(r.slot().lastStep().elapsed <= 1.2);
        CHECK(r.stallFaults() == 1);
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
        CHECK(r.trueDeg() == doctest::Approx(60.0).epsilon(0.01));
        CHECK(r.mechLines().back().find("'arm': STALLED -- stalled in") == 0);
        r.tick(100);  // nothing commands it after the verdict
        CHECK(r.volts() == 0.0);
        CHECK(r.stallFaults() == 1);
    }
    SUBCASE("a quicker one, 1.6 A below 3 rpm for 0.15 s: STALLED (measured 0.65 s; margin 0.8)") {
        ArmRig r{kStalls.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.plant->setObstacleDeg(60.0);
        r.slot().moveTo(120.0, units::Time{3.0}, false);
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
        CHECK(r.slot().lastStep().elapsed <= 0.8);
        CHECK(r.stallFaults() == 1);
        CHECK(r.volts() == 0.0);
    }
    SUBCASE("N2: a signature lasting 10 s cannot trip before the 3 s bound -- TIMED_OUT, then it HOLDS WHERE IT STOPPED") {
        // (The brief's case was a 2.6 A signature — the build now refuses one that can never trip, n58 —
        // so the signature here is a valid one that is merely too slow for this bound.) Measured: TIMED_OUT
        // at 3.01 s; over the next 2 s mean |V| 1.68, current <= 0.37 A, the arm at 59.7, its target 60.0.
        // Margins (§3.3): mean |V| < 2.5, current < 1 A.
        mech::MechanismSpec spec = kGreen5.spec();
        spec.position.stall.time = 10.0;
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.plant->setObstacleDeg(60.0);
        r.slot().moveTo(120.0, units::Time{3.0}, false);
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(r.mechLines().back().find("(it holds where it stopped)") != std::string::npos);
        CHECK(r.slot().target() == doctest::Approx(r.trueDeg()).epsilon(0.01));
        double sumV = 0.0;
        double maxA = 0.0;
        for (int i = 0; i < 200; ++i) {
            r.tick();
            sumV += std::abs(r.volts());
            maxA = std::max(maxA, r.amps());
        }
        CHECK(sumV / 200.0 < 2.5);
        CHECK(maxA < 1.0);
        CHECK(r.slot().holdsClaim());  // still holding, where it stopped
        CHECK(r.stallFaults() == 0);
    }
}

TEST_CASE("MECH-A3 position: P2d -- a target AT the limit is reached; a COMPUTED one past it is CLAMPED and logged ONCE") {
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    const Watched at = moveAndWatch(r, 160.0, 3.0);  // measured: SUCCEEDED, true 160.36, overshoot 2.79
    CHECK(at.outcome == MechanismOutcome::Succeeded);
    CHECK(r.slot().softStops() == 0);
    const std::size_t before = r.mechLines().size();
    const Watched clamped = moveAndWatch(r, 175.0, 3.0, /*computed=*/true);
    CHECK(clamped.outcome == MechanismOutcome::Succeeded);
    CHECK(r.slot().target() == 160.0);
    CHECK(r.slot().softStops() == 1);
    const std::vector<std::string> lines = r.mechLines();
    REQUIRE(lines.size() >= before + 1);
    CHECK(lines[before] == "'arm': target 175.00 clamped to the travel limit 160.00 (soft stop)");  // verbatim
    CHECK(r.mechCount("clamped to the travel limit") == 1);  // ONCE, never per tick
    // ... and below the bottom limit
    (void)moveAndWatch(r, -40.0, 3.0, true);
    CHECK(r.slot().target() == 0.0);
    CHECK(r.mechCount("'arm': target -40.00 clamped to the travel limit 0.00 (soft stop)") == 1);
    // a CONSTANT target past a limit cannot come from a declaration (the build refuses it): a precondition
    const std::string msg = preconditionOf([&] { r.slot().moveTo(161.0, units::Time{1.0}, false); });
    CHECK(msg.find("CONSTANT target outside the travel limits") != std::string::npos);
}

TEST_CASE("MECH-A3 position: P2d / D2 -- a push past EITHER limit is HELD AT THE LIMIT by the closed-loop hold, heavier plant or not") {
    // The re-check's adv_extra (1): limits 20..140 well inside the hard stops (-10 / 170), a jog ARRIVING
    // at 8 V for 3 s, plant kG 1x / 1.5x / 2x the controller's 1.6. The brief measured worst past the
    // limit 1.26 / 0.97, 2.06 / 1.76, 4.06 / 2.12 deg on the first soft stop; since the review's
    // (latched, the move loop on the way in, a fresh hold at the limit): 0.00 / 0.00, 1.49 / 0.00,
    // 3.67 / 0.04 deg. Margin (§3.3): the brief's measured 4.06 + 1. ONE soft stop per push — the
    // first version let go whenever the arm sat inside the limit, re-applied the full push for a tick
    // and counted another (up to 13 per push).
    for (const double mult : {1.0, 1.5, 2.0}) {
        for (const int dir : {-1, +1}) {
            CAPTURE(mult);
            CAPTURE(dir);
            sim::ArmPlantConfig pc = armPlant(200, 5, 1.6 * mult, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0);
            ArmRig r{kInside.spec(), pc};
            REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
            double worst = 0.0;
            for (int i = 0; i < 300; ++i) {
                r.slot().jog(units::Voltage{dir * 8.0});  // the driver keeps holding the button
                r.tick();
                worst = std::max(worst, dir < 0 ? 20.0 - r.trueDeg() : r.trueDeg() - 140.0);
            }
            CHECK(worst <= 4.06 + 1.0);
            CHECK(r.slot().softStops() == 1);  // one push, one soft stop
            CHECK(r.slot().target() == (dir < 0 ? 20.0 : 140.0));  // held AT the limit, not past it
            // the hard stop is never reached
            CHECK(r.trueDeg() > -10.0 + 1.0);
            CHECK(r.trueDeg() < 170.0 - 1.0);
            r.slot().holdHere();  // the release: HOLD HERE — never a target past the limit
            for (int i = 0; i < 200; ++i) {
                r.tick();
                CHECK(r.slot().target() >= 20.0);
                CHECK(r.slot().target() <= 140.0);
            }
            CHECK(r.stallFaults() == 0);
        }
    }
}

TEST_CASE("MECH-A3 position: the soft stop at the BOTTOM holds a heavier arm short of the drivetrain (adv_softstop)") {
    // Limit 20 deg, hard stop -10, a -6 V jog held 3 s from 60, plant kG 1x / 1.5x / 2x the controller's.
    // Measured (the brief): lowest 18.32 / 17.94 / 16.06 deg (was the hard stop under an open-loop cut).
    for (const double plantKg : {1.6, 2.4, 3.2}) {
        CAPTURE(plantKg);
        ArmRig r{kInside.spec(), armPlant(200, 5, plantKg, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
        REQUIRE(moveAndWatch(r, 60.0, 3.0).outcome == MechanismOutcome::Succeeded);
        r.slot().jog(units::Voltage{-6.0});
        double lowest = 1e9;
        for (int i = 0; i < 300; ++i) {
            r.tick();
            lowest = std::min(lowest, r.trueDeg());
        }
        CHECK(lowest >= 20.0 - 5.0);  // the brief measured worst 16.06 at 2x; margin 1 (since the review's
                                      // soft stop: 20.01 / 18.56 / 16.33)
        // the release: HOLD WHERE IT IS, clamped inside the travel — its target is exactly where the arm
        // read at the release, or the limit when it read past it
        const double atRelease = r.slot().position();
        r.slot().holdHere();
        CHECK(r.slot().target() == doctest::Approx(std::max(20.0, atRelease)).epsilon(1e-12));
        r.tick(100);
        CHECK(r.slot().target() >= 20.0);
    }
}

TEST_CASE("MECH-A3 position: the soft stop on a SLOW, a MEDIUM and a FAST arm pushed at 4, 8 and 12 V -- one soft stop per push, never harder than the push, never past the limit by more than measured") {
    // The review's finding: D2 measured ONE arm at 8 V. The same declaration (limits 20..140) on Red,
    // Green and Blue 5:1 — output free 120, 240 and 720 deg/s — plant kG 1x / 1.5x / 2x the
    // controller's, pushed at 4 / 8 / 12 V toward each limit for 3 s (54 pushes). Measured: worst past
    // a limit 8.32 deg (Blue, 2x heavy, 12 V DOWN: the heavier arm sags under a hold that believes it
    // lighter until the integral catches up), 2.50 deg with the gravity right (Blue, 12 V down) — the
    // first version 10.11 and 4.56. Margins: 1 deg. And the soft stop never pushes TOWARD the limit
    // harder than the push itself (the first version pushed the Blue arm at 8.70 V for an 8 V push:
    // its hold's P on the 40 deg left when it took over).
    static constexpr Arm kInsideRed{.name = "arm", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {20, 140}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(20)};
    static constexpr Arm kInsideBlue{.name = "arm", .motors = {Forward}, .cartridge = Blue, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {20, 140}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(20)};
    struct Cart {
        const Arm* arm;
        double rpm;
    };
    double worst = 0.0;
    double worstMatched = 0.0;
    for (const Cart c : {Cart{&kInsideRed, 100.0}, Cart{&kInside, 200.0}, Cart{&kInsideBlue, 600.0}}) {
        for (const double mult : {1.0, 1.5, 2.0}) {
            for (const int dir : {-1, +1}) {
                for (const double volts : {4.0, 8.0, 12.0}) {
                    CAPTURE(c.rpm);
                    CAPTURE(mult);
                    CAPTURE(dir);
                    CAPTURE(volts);
                    ArmRig r{c.arm->spec(), armPlant(c.rpm, 5, 1.6 * mult, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
                    REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
                    const double limit = dir < 0 ? 20.0 : 140.0;
                    double past = 0.0;
                    double hardest = 0.0;  // the most volts toward the limit on any tick
                    bool near = false;
                    for (int i = 0; i < 300; ++i) {
                        r.slot().jog(units::Voltage{dir * volts});
                        r.tick();
                        past = std::max(past, dir * (r.trueDeg() - limit));
                        hardest = std::max(hardest, dir * r.volts());
                        near = near || std::abs(r.trueDeg() - limit) <= 1.0;
                    }
                    CHECK(hardest <= volts + 1e-12);                    // never harder than the push
                    CHECK(r.slot().softStops() == (near ? 1 : 0));      // ONE per push (none: it never got there)
                    CHECK(r.stallFaults() == 0);
                    worst = std::max(worst, past);
                    worstMatched = mult == 1.0 ? std::max(worstMatched, past) : worstMatched;
                }
            }
        }
    }
    CHECK(worst <= 8.32 + 1.0);
    CHECK(worstMatched <= 2.50 + 1.0);
}

TEST_CASE("MECH-A3 position: a push held at a limit for 20 s stays ONE soft stop and CONVERGES there -- the hold keeps its integral") {
    // The review's finding: the first version let go whenever the hold brought the arm back inside the
    // limit, re-applied the full push for a tick, reset the hold's integral and counted another soft
    // stop (25 in 60 s at 2x, 8 V) — and a 2x heavier arm drifted +1.59 deg in 15 s instead of
    // settling. Measured now: worst 1.49 / 0.00 (1.5x), 3.67 / 0.04 (2x) deg; after 20 s within
    // 0.056 deg of the limit (the plant's static band: the integral breaks it slowly); one soft stop.
    // Margins: 5.06 (D2's), 0.1 deg.
    for (const double mult : {1.5, 2.0}) {
        for (const int dir : {-1, +1}) {
            CAPTURE(mult);
            CAPTURE(dir);
            ArmRig r{kInside.spec(), armPlant(200, 5, 1.6 * mult, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
            REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
            const double limit = dir < 0 ? 20.0 : 140.0;
            double past = 0.0;
            for (int i = 0; i < 2000; ++i) {
                r.slot().jog(units::Voltage{dir * 8.0});
                r.tick();
                past = std::max(past, dir * (r.trueDeg() - limit));
            }
            CHECK(r.slot().softStops() == 1);
            CHECK(past <= 4.06 + 1.0);
            CHECK(std::abs(r.trueDeg() - limit) <= 0.1);
            CHECK(r.slot().target() == limit);
            CHECK(r.stallFaults() == 0);
        }
    }
}

TEST_CASE("MECH-A3 position: the soft stop takes a push only where it could no longer stop -- never far from the limit, never while it moves AWAY") {
    SUBCASE("a spool lift pushed at 6 V from 12 in: the driver's own until near the limit, then held AT it") {
        // Measured: the soft stop took the push 2.71 s in going down and 3.89 s in going up (the lift
        // then ~0.2 in from the limit), and held it within 0.05 in of the limit. With the lift's
        // acceleration read in inches where degrees belong (the review's mutation) it took the push
        // 0.06 s in — the stopping distance 76 times too long.
        for (const int dir : {-1, +1}) {
            CAPTURE(dir);
            ArmRig r{kLift.spec(), spoolPlant(1.0, 1.0)};
            r.slot().moveTo(12.0, units::Time{6.0}, false);
            r.runStep();
            REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
            for (int i = 0; i < 100; ++i) {
                r.slot().jog(units::Voltage{dir * 6.0});
                r.tick();
                CHECK(r.volts() == dir * 6.0);  // the push, untouched
            }
            CHECK(r.slot().softStops() == 0);
            for (int i = 0; i < 500; ++i) {
                r.slot().jog(units::Voltage{dir * 6.0});
                r.tick();
            }
            CHECK(r.slot().softStops() == 1);
            CHECK(std::abs(r.slot().position() - (dir < 0 ? 0.0 : 24.0)) <= 0.05);
            CHECK(r.slot().target() == (dir < 0 ? 0.0 : 24.0));
        }
    }
    SUBCASE("an arm FALLING fast near its top when the push flips UP: the flip is the driver's own") {
        // Its speed AWAY from the top would stop it short of the top only past it (|v|^2 / 2a > the
        // degrees left) — a soft stop keyed on the SPEED alone, not its direction, would take the
        // flip over (measured: 1.83 V instead of 12, a soft stop counted).
        ArmRig r{kInside.spec(), armPlant(200, 5, 1.6, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
        REQUIRE(moveAndWatch(r, 135.0, 3.0).outcome == MechanismOutcome::Succeeded);
        const double accel = r.spec.position.maxAcceleration;  // an arm: output deg/s^2
        bool falling = false;
        for (int i = 0; i < 100 && !falling; ++i) {
            r.slot().jog(units::Voltage{-12.0});
            r.tick();
            const double down = -r.plant->trueSpeedDegS();
            falling = 140.0 - r.slot().jointDeg() <= down * down / (2.0 * accel);
        }
        REQUIRE(falling);
        r.slot().jog(units::Voltage{12.0});
        r.tick();
        CHECK(r.volts() == 12.0);
        CHECK(r.slot().softStops() == 0);
    }
    SUBCASE("a push held AT the top that turns DOWN is the driver's own at once -- and a push back up is a NEW soft stop") {
        ArmRig r{kInside.spec(), armPlant(200, 5, 1.6, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
        REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
        for (int i = 0; i < 200; ++i) {
            r.slot().jog(units::Voltage{8.0});
            r.tick();
        }
        REQUIRE(r.slot().softStops() == 1);
        REQUIRE(std::abs(r.trueDeg() - 140.0) <= 0.5);
        for (int i = 0; i < 30; ++i) {
            r.slot().jog(units::Voltage{-8.0});  // the same hold-to-run, turned away from the top
            r.tick();
            CHECK(r.volts() == -8.0);            // the driver's own: the soft stop let go at once
        }
        CHECK(r.trueDeg() < 140.0 - 5.0);
        for (int i = 0; i < 200; ++i) {
            r.slot().jog(units::Voltage{8.0});  // back up: taken over again, counted again
            r.tick();
        }
        CHECK(r.slot().softStops() == 2);
        CHECK(std::abs(r.trueDeg() - 140.0) <= 0.5);
    }
    SUBCASE("a push held at the top that ENDS -- by release(), by a move, by a STALLED -- leaves no soft stop behind") {
        // The soft stop holds a push "until it turns away from the limit or ends". After it ends, the
        // next push the same way is the driver's own until it nears the limit again: a NEW soft stop. (A
        // latch left set would aim that push at the old limit hold, capped at the push: measured 1.58 V
        // for an 8 V push from 80, held where it was, no soft stop counted.) A 0.5 A stall signature, so
        // a hold jammed short of the top STALLS; at 0.5 A nothing else here trips it.
        mech::MechanismSpec spec = kInside.spec();
        spec.position.stall.amps = 0.5;
        for (const int end : {0, 1, 2}) {
            CAPTURE(end);  // 0 release(), 1 a move, 2 a STALLED
            ArmRig r{spec, armPlant(200, 5, 1.6, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
            REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
            bool jammed = false;
            for (int i = 0; i < 200 && r.stallFaults() == 0; ++i) {
                r.slot().jog(units::Voltage{8.0});
                r.tick();
                if (end == 2 && !jammed && r.slot().softStops() == 1) {
                    r.plant->setJammed(true);  // caught short of the top: the soft stop's loops push into it
                    jammed = true;
                }
            }
            REQUIRE(r.slot().softStops() == 1);
            if (end == 0) {
                REQUIRE(std::abs(r.trueDeg() - 140.0) <= 0.5);
                r.slot().release();
            } else if (end == 2) {
                for (int i = 0; i < 300 && r.stallFaults() == 0; ++i) {
                    r.slot().jog(units::Voltage{8.0});
                    r.tick();
                }
                REQUIRE(r.stallFaults() == 1);
                REQUIRE(r.mechCount("a stated stall signature tripped while held to move") == 1);
                r.plant->setJammed(false);
            }
            REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);  // a move away
            r.slot().jog(units::Voltage{8.0});
            r.tick();
            CHECK(r.volts() == 8.0);  // the push's own volts, far from the limit
            for (int i = 0; i < 200; ++i) {
                r.slot().jog(units::Voltage{8.0});
                r.tick();
            }
            CHECK(r.slot().softStops() == 2);  // a NEW soft stop ...
            CHECK(std::abs(r.trueDeg() - 140.0) <= 0.5);  // ... and the push held at the limit again
        }
    }
}

TEST_CASE("MECH-A3 position: at a RESTS-AT-STOP end the limit IS the stop -- the soft stop brings a push onto it slowly") {
    // restsAtStop(20) needs its home inside the travel (the build refuses one outside), so the bottom
    // limit is where the stop is: here the plant's own stop is at 20. A push down at 6 / 8 / 12 V from
    // 80 (peak 146..252 deg/s) meets the stop at 0.0 .. 0.2 deg/s with matched gravity (at 6 V never: held
    // just above it), 12.9 .. 13.5 deg/s (1.5x heavy), 33.4 .. 35.3 deg/s (2x). Margin 5 deg/s. Never
    // STALLED, held there.
    for (const double mult : {1.0, 1.5, 2.0}) {
        for (const double volts : {6.0, 8.0, 12.0}) {
            CAPTURE(mult);
            CAPTURE(volts);
            ArmRig r{kInside.spec(), armPlant(200, 5, 1.6 * mult, 1.0, 0.3, 20.0, 45.0, 20.0, 170.0)};
            REQUIRE(moveAndWatch(r, 80.0, 3.0).outcome == MechanismOutcome::Succeeded);
            double peak = 0.0;
            double meets = 0.0;
            for (int i = 0; i < 300; ++i) {
                const double speed = std::abs(r.plant->trueSpeedDegS());
                r.slot().jog(units::Voltage{-volts});
                r.tick();
                peak = std::max(peak, speed);
                if (r.trueDeg() <= 20.0 + 1e-9) {
                    meets = std::max(meets, speed);
                }
            }
            CHECK(peak >= 140.0);           // it came fast ...
            CHECK(meets <= 35.3 + 5.0);     // ... and met its stop slowly
            CHECK(r.stallFaults() == 0);
            CHECK(r.trueDeg() == doctest::Approx(20.0).epsilon(0.001));
        }
    }
}

TEST_CASE("MECH-A3 position: a linkage's or a table's move plan never asks the motor for more than its starting share of the free speed") {
    // The plan runs in OUTPUT degrees, its speed the stated inches per second turned into degrees by the
    // mechanism's STEEPEST slope. The first version used the move's MEAN slope: near a DR4B's top it
    // asked 200..364 % of the free speed, and the move was a saturated push (measured: 30 -> 38 in, 30
    // ticks at 12 V, joint peak 97 % of free). Measured now: joint peak 74..77 % of free (the plan's
    // 75 % and its tracking), never a saturated tick. A measured table whose LAST segment is its
    // steepest: the starting speed is that segment's (16.07 in/s), its plan 77 % of free; the first
    // version's first-segment scale planned 46 %.
    static constexpr HeightAt kSteepLast[] = {{0_in, 0}, {4_in, 40}, {10_in, 80}, {20_in, 120}};
    static constexpr Lift kTilt2{.name = "tilt2", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = measured(kSteepLast), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 20_in}, .gravity = noGravity, .home = restsAtStop(0_in)};
    const double free = 100.0 * 6.0 / 7.0;  // Red 7:1, output deg/s
    CHECK(kTilt2.spec().position.maxVelocity == doctest::Approx(0.75 * free * 10.0 / 40.0));  // the steepest segment
    struct Move {
        const Lift* lift;
        double startDeg;
        double from;
        double to;
        bool cruises;  // long enough to reach the plan's cruise
    };
    for (const Move m : {Move{&kDr4b, -90.0, -18.0, 38.0, true}, Move{&kDr4b, -90.0, 30.0, 38.0, true},
                         Move{&kDr4b, -90.0, 36.0, 38.0, false}, Move{&kDr4b, -90.0, 38.0, 30.0, true},
                         Move{&kTable, 0.0, 0.0, 20.0, true}, Move{&kTable, 0.0, 14.0, 20.0, true},
                         Move{&kTilt2, 0.0, 0.0, 20.0, true}, Move{&kTilt2, 0.0, 10.0, 20.0, true}}) {
        CAPTURE(m.from);
        CAPTURE(m.to);
        CAPTURE(m.lift->name.text);
        const mech::MechanismSpec spec = m.lift->spec();
        ArmRig r{spec, armPlant(100, 7, 0.0, 1.0, 0.3, m.startDeg, 0.0, -400.0, 400.0)};
        if (m.from != spec.position.minPosition) {
            r.slot().moveTo(m.from, units::Time{8.0}, false);
            r.runStep(3000);
            r.tick(100);
        }
        r.slot().moveTo(m.to, units::Time{8.0}, false);
        double peak = 0.0;
        int saturated = 0;
        for (int n = 0; r.slot().running() && n < 3000; ++n) {
            r.tick();
            peak = std::max(peak, std::abs(r.plant->trueSpeedDegS()));
            saturated += std::abs(r.volts()) >= 12.0 ? 1 : 0;
        }
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(saturated == 0);
        CHECK(peak <= 0.80 * free);
        if (m.cruises) {
            CHECK(peak >= 0.70 * free);  // ... and not slower than its plan (the first-segment scale: 46 %)
        }
    }
}

TEST_CASE("MECH-A3 position: case C -- restsAtStop believed but truly at 30: a held DOWN stops at the BELIEVED zero") {
    // The arm was left up at 30 when the program started; restsAtStop(0) trusts the boot pose (the
    // guide says so). Measured (the brief): a -6 V jog for 3 s stops at true 29.20, never the real stop.
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
    REQUIRE(r.slot().homed());
    CHECK(r.slot().position() == doctest::Approx(0.0));
    r.slot().jog(units::Voltage{-6.0});
    r.tick(300);
    CHECK(r.trueDeg() >= 30.0 - 1.5);  // held at 'zero' (truly ~30), never driven to -2
    CHECK(r.slot().target() == 0.0);
}

TEST_CASE("MECH-A3 position: P2d -- NEVER HOMED: a jog runs at most unhomedVolts (0: nothing), a move is NOT_HOMED") {
    {
        ArmRig r{kNeverHomed.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        CHECK_FALSE(r.slot().homed());
        r.slot().jog(units::Voltage{8.0});
        r.tick(50);
        CHECK(r.volts() == 0.0);                 // nothing: no limits known, no power invented
        CHECK_FALSE(r.mech->claimed());          // ... and the safe state, not a held claim
        CHECK(r.trueDeg() <= 0.0);               // never raised (it settles under its own Hold brake)
        r.slot().moveTo(90.0, units::Time{1.0}, false);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::NotHomed);
        CHECK(r.mechLines().back() == "'arm': NOT_HOMED in 0.00s (it is declared neverHomed: a move needs a zero)");
        r.tick(20);
        CHECK(r.volts() == 0.0);
        CHECK(preconditionOf([&] { r.slot().home(units::Time{1.0}); }).find("neverHomed") !=
              std::string::npos);
    }
    {
        ArmRig r{kNeverHomed4V.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.slot().jog(units::Voltage{8.0});
        r.tick(10);
        CHECK(r.volts() == 4.0);  // capped at unhomedVolts
        r.slot().jog(units::Voltage{-9.0});
        r.tick(10);
        CHECK(r.volts() == -4.0);
        // a NOT_HOMED move stops the running jog (the most recent command wins): safe, not left at -4 V
        r.slot().moveTo(90.0, units::Time{1.0}, false);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::NotHomed);
        r.tick(5);
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
        // a release before homing: no position to hold — the safe state
        r.slot().jog(units::Voltage{3.0});
        r.tick(5);
        r.slot().holdHere();
        r.tick(5);
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
    }
}

TEST_CASE("MECH-A3 position: D4 -- REACHED is judged in the mechanism's OWN units at every geometry's ends") {
    // A degree tolerance converted at a measured table's end (or a linkage's top) is 0 deg — "reached"
    // could never be met there (the re-check's case G: TIMED_OUT before). Measured: tilt -> 'top' (20 in)
    // SUCCEEDED at 1.91 s.
    SUBCASE("a measured table at its TOP end and its BOTTOM end") {
        const mech::MechanismSpec t = kTable.spec();
        CHECK(std::abs(mech::positionToAngleDeg(t.position, 20.0 + t.position.tolerance) -
                       mech::positionToAngleDeg(t.position, 20.0)) == 0.0);  // why units, not degrees
        ArmRig r{t, armPlant(100, 7, 0.0, 1.0, 0.3, 0.0, 0.0, -1.0, 130.0)};
        r.slot().moveToPreset(0, units::Time{4.0});
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.slot().position() == doctest::Approx(20.0).epsilon(0.02));
        r.slot().moveTo(0.0, units::Time{4.0}, false);
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    }
    SUBCASE("a spool lift at both limits") {
        ArmRig r{kLift.spec(), spoolPlant(1.0, 1.0)};
        r.slot().moveTo(24.0, units::Time{6.0}, false);
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        r.slot().moveTo(0.0, units::Time{6.0}, false);
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    }
    SUBCASE("a DR4B linkage at its formula's maximum (38 in) and minimum") {
        const mech::MechanismSpec d = kDr4b.spec();
        CHECK(std::abs(mech::positionToAngleDeg(d.position, 38.0 + d.position.tolerance) -
                       mech::positionToAngleDeg(d.position, 38.0)) == 0.0);  // past the asin domain
        ArmRig r{d, armPlant(100, 7, 0.0, 1.0, 0.3, -90.0, 0.0, -92.0, 92.0)};
        REQUIRE(r.slot().position() == doctest::Approx(-18.0));
        r.slot().moveTo(38.0, units::Time{5.0}, false);
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        r.slot().moveTo(-18.0, units::Time{5.0}, false);
        r.runStep();
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    }
    SUBCASE("a rotary arm at both limits") {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        CHECK(moveAndWatch(r, 160.0, 3.0).outcome == MechanismOutcome::Succeeded);
        CHECK(moveAndWatch(r, 0.0, 3.0).outcome == MechanismOutcome::Succeeded);
    }
}

TEST_CASE("MECH-A3 position: P2f -- the two-motor LIFT in inches") {
    // Red direct, a 1.5 in spool, controller kG 1.0 against plant 0.8..1.4, kV x0.8/1.25. Measured: 18 in
    // SUCCEEDED in 3.42..3.58 s, within 0.05 in; under +0.5 V worst 0.091 in. Margins: 4 s, 0.1 in, 0.15 in.
    const mech::MechanismSpec ls = kLift.spec();
    CHECK(ls.position.maxVelocity == doctest::Approx(0.75 * 100.0 * 6.0 * kPi * 1.5 / 360.0));  // 5.89 in/s
    for (const double plantKg : {0.8, 1.0, 1.4}) {
        for (const double kvx : {0.8, 1.25}) {
            CAPTURE(plantKg);
            CAPTURE(kvx);
            ArmRig r{ls, spoolPlant(plantKg, kvx)};
            r.slot().moveTo(18.0, units::Time{6.0}, false);
            r.runStep();
            CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
            CHECK(r.slot().lastStep().elapsed <= 4.0);
            CHECK(mech::angleDegToPosition(ls.position, r.trueDeg()) == doctest::Approx(18.0).epsilon(0.1 / 18.0));
            r.plant->addLoadVolts(0.5);
            double worst = 0.0;
            for (int i = 0; i < 200; ++i) {
                r.tick();
                worst = std::max(worst, std::abs(mech::angleDegToPosition(ls.position, r.trueDeg()) - 18.0));
            }
            CHECK(worst <= 0.15);
        }
    }
}

TEST_CASE("MECH-A3 position: P2e -- EVERY HOMING MOMENT, each logging where its zero went (D10)") {
    SUBCASE("rests at its stop: homed at bind, with no motion, logged") {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        CHECK(r.slot().homed());
        CHECK(r.slot().position() == 0.0);
        CHECK(r.volts() == 0.0);
        REQUIRE(r.mechLines().size() == 1U);
        CHECK(r.mechLines()[0] == "'arm': homed -- +0.0 deg from where its encoders read zero at boot");
    }
    SUBCASE("before homing a move is NOT_HOMED; a STALL-home from an unknown 30 deg zeroes at the stop") {
        // measured: SUCCEEDED after 0.51 s, true -2.00 (the stop), reads 0.00; then to 90 SUCCEEDED
        ArmRig r{kStallHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        CHECK_FALSE(r.slot().homed());
        r.slot().moveTo(90.0, units::Time{1.0}, false);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::NotHomed);
        CHECK(r.mechLines().back().find("(it has not found its zero yet: home it first (homeByStall))") !=
              std::string::npos);
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.slot().lastStep().doneBy == mech::DoneBy::Reached);
        CHECK(r.slot().lastStep().elapsed <= 1.0);
        CHECK(r.slot().homed());
        CHECK(r.trueDeg() == doctest::Approx(-2.0));
        CHECK(r.slot().position() == doctest::Approx(0.0).epsilon(0.01));
        // the zero log: the stop is 32 deg below where the encoders read zero (the boot pose, 30)
        CHECK(r.mechCount("'arm': homed -- -32.0 deg from where its encoders read zero at boot") == 1);
        CHECK(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(88.0).epsilon(0.02));  // zero is the stop at -2
    }
    SUBCASE("a SWITCH-home from 40 deg: the switch at 0 (its fresh read) is its zero") {
        // measured (the brief): 0.48 s; here the plant's switch is pressed at or below 0 deg
        ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0),
                 Wiring{.homeSwitch = sim::SwitchMount{.atDeg = 0.0}}};
        CHECK_FALSE(r.slot().homed());
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.slot().lastStep().elapsed <= 1.0);
        CHECK(r.slot().homed());
        CHECK(r.slot().holdsClaim());  // it holds at home after the step
        CHECK(r.volts() > 0.0);        // ... from the verdict's own tick: the hold, never one more tick of push
        // zeroed on the first FRESH pressed read: the zero's true place is within one tick's travel of
        // the switch at true 0 (measured -0.23 deg: about 23 deg/s at -3 V). The push's momentum carries
        // it past its zero onto the stop at -2; the hold brings it back (measured -0.06 after 2.5 s).
        CHECK(std::abs(r.trueDeg() - r.slot().position()) <= 0.3);
        r.tick(300);
        CHECK(std::abs(r.slot().position()) <= 0.3);
    }
    SUBCASE("a switch ALREADY PRESSED at boot: homed at bind, nothing commanded") {
        ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3),
                 Wiring{.homeSwitch = sim::SwitchMount{.atDeg = 0.5}}};
        CHECK(r.slot().homed());
        CHECK(r.volts() == 0.0);
        CHECK(r.mechLines().size() == 1U);
    }
    SUBCASE("a switch NOT pressed at boot: not homed, and a switch that never presses TIMES OUT (the safe state)") {
        ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0), Wiring{.switchByHand = true}};
        CHECK_FALSE(r.slot().homed());
        r.slot().home(units::Time{2.0});
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(r.mechLines().back() == "'arm': TIMED_OUT in 2.00s (its home switch never read pressed)");
        CHECK_FALSE(r.slot().homed());
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
    }
}

TEST_CASE("MECH-A3 position: case S -- a home switch DEAD from boot (HA-184 wrong: it holds 'pressed') never homes it") {
    // The R probe's S: the switch's input failed from boot, and the PROS adapter holds its last level —
    // which, if HA-184 is wrong, can read PRESSED. Every read moves its read-fault count, so no read is
    // fresh: not homed at bind, not homed by a homing push either (it pushes, bounded: TIMED_OUT, safe).
    // The CONTROL: the same switch reading pressed WITHOUT its count wired homes it at bind wherever it
    // stands — the defect the read-fault channel closes.
    ArmRig dead{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 60.0),
                Wiring{.homeSwitchFaults = true, .switchByHand = true, .switchLevelAtBoot = true,
                       .switchFailingFromBoot = true}};
    CHECK_FALSE(dead.slot().homed());
    CHECK(dead.mechLines().empty());
    CHECK_FALSE(dead.slot().homeSwitchPressed());
    dead.slot().home(units::Time{1.0});
    dead.runStep(200);
    CHECK(dead.slot().lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK_FALSE(dead.slot().homed());
    // it answers again: the next tick's read is fresh, and pressed is pressed
    dead.homeFaults.failing(false);
    dead.tick();
    CHECK(dead.slot().homeSwitchPressed());

    ArmRig control{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 60.0),
                   Wiring{.switchByHand = true, .switchLevelAtBoot = true}};
    CHECK(control.slot().homed());  // homed at 60 deg — wherever it stood
}

TEST_CASE("MECH-A3 position: case F -- a home switch that reads PRESSED while it is not (HA-184 wrong): homed wherever it stands; the jam signature is the net") {
    // The re-check's case F: the switch's level says "pressed" at boot (a polarity HA-184 has wrong, or an
    // input that floats high) while the arm is truly at 60 — a FRESH read, so the service believes it.
    // Homed at bind, with no motion, at 60; a move to 140 is truly to 200, past the hard stop at 170: the
    // REQUIRED stall signature ends it there (the brief measured STALLED at 1.70 s). What no net sees: the
    // zero itself (the bench page settles HA-184).
    ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 60.0),
             Wiring{.switchByHand = true, .switchLevelAtBoot = true}};
    CHECK(r.slot().homed());
    CHECK(r.slot().position() == doctest::Approx(0.0));
    CHECK(r.volts() == 0.0);
    r.slot().moveTo(140.0, units::Time{3.0}, false);
    r.runStep();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().lastStep().elapsed <= 2.0);
    CHECK(r.trueDeg() == doctest::Approx(170.0));
    CHECK(r.volts() == 0.0);
}

TEST_CASE("MECH-A3 position: a switch that FAILS mid-homing is never read as a home -- only a fresh press homes it") {
    // Homing down from 40; the input fails at 20 deg holding its last level, then the plant's switch is
    // pressed at 0 — the read that FAILED cannot home it, the fresh one after it does.
    ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0),
             Wiring{.homeSwitchFaults = true, .switchByHand = true}};
    r.slot().home(units::Time{3.0});
    bool failedWhilePressed = false;
    for (int i = 0; i < 300 && r.slot().running(); ++i) {
        // the switch reads pressed from 30 deg down, but its input fails between 30 and 10 deg
        r.homeSwitch.setState(r.trueDeg() <= 30.0);
        r.homeFaults.failing(r.trueDeg() <= 30.0 && r.trueDeg() > 10.0);
        failedWhilePressed = failedWhilePressed || (r.homeFaults.isFailing() && r.trueDeg() <= 30.0);
        r.tick();
    }
    CHECK(failedWhilePressed);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.trueDeg() <= 10.0);  // homed on the first FRESH pressed read, never inside the failed window
}

TEST_CASE("MECH-A3 position: D10 -- homing into a WEDGED piece sets a zero 25 deg off, said; a re-home moves it 27 deg, a Warn") {
    // A stall-home from 60 with a piece wedged under the arm at 25 (true stop -2). The encoders read zero
    // at the true 0 here, so the log reads in true degrees (the brief's line).
    sim::ArmPlantConfig pc = armPlant(200, 5, 2.0, 1.0, 0.3, 60.0);
    pc.encoderZeroDeg = 0.0;
    ArmRig r{kStallHome.spec(), pc};
    r.plant->setWedgeDeg(25.0);
    r.slot().home(units::Time{3.0});
    r.runStep(400);
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.trueDeg() == doctest::Approx(25.0));
    CHECK(r.mechCount("'arm': homed -- +25.0 deg from where its encoders read zero at boot") == 1);
    // the piece is cleared and it is homed AGAIN: its zero moves -27 deg, to the true stop — a WARN
    r.plant->clearWedge();
    r.slot().home(units::Time{3.0});
    r.runStep(400);
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.trueDeg() == doctest::Approx(-2.0));
    bool warned = false;
    for (int i = 0; i < r.sink.size(); ++i) {
        if (r.sink.at(i).message ==
            "'arm': homed again -- its zero moved -27.0 deg (past its tolerance: was something in the way?)") {
            warned = r.sink.at(i).level == shulib::hal::LogLevel::Warn;
        }
    }
    CHECK(warned);
    // a re-home that finds the SAME zero is Info, not a Warn
    r.slot().home(units::Time{3.0});
    r.runStep(400);
    CHECK(r.mechLines()[r.mechLines().size() - 2] == "'arm': homed again -- its zero moved +0.0 deg");
}

TEST_CASE("MECH-A3 position: P2h -- an ABSOLUTE angle sensor: homed at bind, mounted either way, its reading wrapped through 0") {
    SUBCASE("the arm at 40 at power-up, the sensor reading 40: homed at 40, never 'zero where it stands' (M19)") {
        ArmRig r{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0), Wiring{.angle = sim::RotationMount{}}};
        CHECK(r.slot().homed());
        CHECK(r.slot().position() == doctest::Approx(40.0));
        CHECK(r.mechLines()[0] == "'arm': homed -- its angle sensor reads 40.0 deg, so it is at 40.00 deg");
        const Watched w = moveAndWatch(r, 90.0, 3.0);  // measured: SUCCEEDED, true 89.95
        CHECK(w.outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(90.0).epsilon(0.02));
    }
    SUBCASE("mounted REVERSED and declared rotationSensor(Reversed): the same") {
        ArmRig r{kAbsoluteReversed.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0),
                 Wiring{.angle = sim::RotationMount{.reversed = true}}};
        CHECK(r.slot().position() == doctest::Approx(40.0));
        CHECK(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(90.0).epsilon(0.02));
    }
    SUBCASE("the reading WRAPS through 0: the sensor reads 350 at the arm's 0, so at 20 it reads 10") {
        ArmRig r{kAbsoluteWraps.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 20.0),
                 Wiring{.angle = sim::RotationMount{.readsAtJointZeroDeg = 350.0}}};
        REQUIRE(r.rotation.position().value() * 180.0 / kPi == doctest::Approx(10.0));  // HA-186's power-up reading
        CHECK(r.slot().position() == doctest::Approx(20.0));  // not -340: the turn nearest its travel's middle
        CHECK(moveAndWatch(r, 150.0, 3.0).outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(150.0).epsilon(0.02));
    }
    SUBCASE("a REVERSED sensor reading 180 at the arm's 0 (robot y's wrist), the arm at -30") {
        const mech::MechanismSpec wrist = ry::wrist.spec();  // absoluteSensor(0, 180), rotationSensor(Reversed)
        ArmRig r{wrist, armPlant(200, 3, 0.0, 1.0, 0.3, -30.0, 0.0, -95.0, 95.0),
                 Wiring{.angle = sim::RotationMount{.reversed = true, .readsAtJointZeroDeg = 180.0}}};
        REQUIRE(r.rotation.position().value() * 180.0 / kPi == doctest::Approx(210.0));
        CHECK(r.slot().position() == doctest::Approx(-30.0));  // the overlay's formula read -390 here
        // the zero log prints what the SENSOR reads (its mounting applied) — never its negation
        CHECK(r.mechLines()[0] == "'wrist': homed -- its angle sensor reads 210.0 deg, so it is at -30.00 deg");
    }
    SUBCASE("ABOVE the middle of its travel: the arm at 150 (0..160) reads 150, never a turn below") {
        // The turn is chosen nearest the travel's middle (80): 150, not 150 - 360 = -210. Every subcase
        // above sits at or below its middle, where "nearest" and "the turn at or below" agree — the
        // review's mutation (floor for round) survived them; a move from a believed -210 would drive
        // some 300 deg toward a hard stop.
        ArmRig r{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 150.0), Wiring{.angle = sim::RotationMount{}}};
        CHECK(r.slot().position() == doctest::Approx(150.0));
        CHECK(r.mechLines()[0] == "'arm': homed -- its angle sensor reads 150.0 deg, so it is at 150.00 deg");
        CHECK(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(90.0).epsilon(0.02));
    }
    SUBCASE("robot y's wrist ABOVE its middle, at +60, mounted reversed: it reads 120 and is at +60") {
        const mech::MechanismSpec wrist = ry::wrist.spec();  // -90..90, absoluteSensor(0, 180), Reversed
        ArmRig r{wrist, armPlant(200, 3, 0.0, 1.0, 0.3, 60.0, 0.0, -95.0, 95.0),
                 Wiring{.angle = sim::RotationMount{.reversed = true, .readsAtJointZeroDeg = 180.0}}};
        REQUIRE(r.rotation.position().value() * 180.0 / kPi == doctest::Approx(120.0));
        CHECK(r.slot().position() == doctest::Approx(60.0));  // the turn at or below the middle: -300
        CHECK(r.mechLines()[0] == "'wrist': homed -- its angle sensor reads 120.0 deg, so it is at 60.00 deg");
    }
    SUBCASE("the ONE-TURN window: outside its travel by more than (360 - travel) / 2, the reading is a turn off") {
        // HA-186's sensor reads within ONE turn, and the turn is chosen nearest the travel's middle: an arm
        // 0..160 is read right from -100 to 260 (the middle +-180). Resting at -90 (past its bottom limit by
        // less than (360 - 160) / 2 = 100): read right. At -110 it is believed at 250 — a turn off — and a
        // move to 90 drives it DOWN into its stop: its jam signature is the net (the guide's rule: the
        // travel plus twice the farthest it can rest outside a limit must stay under one turn).
        ArmRig ok{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, -90.0, 45.0, -120.0, 170.0),
                  Wiring{.angle = sim::RotationMount{}}};
        CHECK(ok.slot().position() == doctest::Approx(-90.0));
        ArmRig off{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, -110.0, 45.0, -120.0, 170.0),
                   Wiring{.angle = sim::RotationMount{}}};
        CHECK(off.slot().position() == doctest::Approx(250.0));
        off.slot().moveTo(90.0, units::Time{3.0}, false);
        off.runStep();
        CHECK(off.slot().lastStep().outcome == MechanismOutcome::Stalled);
        CHECK(off.trueDeg() == doctest::Approx(-120.0).epsilon(0.002));  // stalled AT its stop (0.72 s in)
    }
    SUBCASE("a REVERSED sensor reading 100 at the arm's 0, the arm at -30: the mounting's sign matters (at 180 a whole turn hides it)") {
        mech::MechanismSpec wrist = ry::wrist.spec();
        wrist.position.absoluteSensorAtZeroDeg = 100.0;
        ArmRig r{wrist, armPlant(200, 3, 0.0, 1.0, 0.3, -30.0, 0.0, -95.0, 95.0),
                 Wiring{.angle = sim::RotationMount{.reversed = true, .readsAtJointZeroDeg = 100.0}}};
        REQUIRE(r.rotation.position().value() * 180.0 / kPi == doctest::Approx(130.0));
        CHECK(r.slot().position() == doctest::Approx(-30.0));  // without the sign: 130 deg
    }
    SUBCASE("declared Forward but mounted REVERSED: it reads -40 and drives into its hard stop -- STALLED there (N10)") {
        // measured (the brief): STALLED at 1.31 s, true 170 = the hard stop (TIMED_OUT at 3.01 s before N10)
        ArmRig r{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0),
                 Wiring{.angle = sim::RotationMount{.reversed = true}}};
        CHECK(r.slot().position() == doctest::Approx(-40.0));  // it reads 320 (-40, wrapped): the arm believes -40
        const Watched w = moveAndWatch(r, 90.0, 3.0);
        CHECK(w.outcome == MechanismOutcome::Stalled);
        CHECK(w.elapsed <= 2.0);
        CHECK(r.trueDeg() == doctest::Approx(170.0));
        CHECK(r.volts() == 0.0);
    }
    SUBCASE("home() on an absolute arm READS IT AGAIN (no motion) and logs a moved zero") {
        ArmRig r{kAbsolute.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0), Wiring{.angle = sim::RotationMount{}}};
        r.slot().home(units::Time{1.0});
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.mechCount("'arm': homed again -- its zero moved +0.0 deg") == 1);
        CHECK(r.slot().holdsClaim());  // holding where it is
    }
}

TEST_CASE("MECH-A3 position: home() on a mechanism that RESTS AT ITS STOP trusts the BOOT pose -- it never re-zeroes where it stands") {
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    REQUIRE(moveAndWatch(r, 95.0, 3.0).outcome == MechanismOutcome::Succeeded);
    const std::size_t lines = r.mechLines().size();
    r.slot().home(units::Time{1.0});
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.mechLines().size() == lines + 1);  // no zero log: the zero did not move
    CHECK(r.mechLines().back() == "'arm': SUCCEEDED -- reached in 0.00s (it rests at its stop at boot: its zero is kept)");
    CHECK(r.slot().position() == doctest::Approx(95.0).epsilon(0.02));  // still 95, not 0
    r.tick(100);
    CHECK(r.trueDeg() == doctest::Approx(95.0).epsilon(0.02));  // held where it was
}

TEST_CASE("MECH-A3 position: N10 case A -- a HOLD loaded past what it can carry after SUCCEEDED trips the stated signature, ONCE") {
    // The re-check's case A: a hold after SUCCEEDED, then a load far beyond 12 V. Before N10 it sat 30 s
    // at 11.94 V / 2.50 A, the verdict still SUCCEEDED. Measured (the brief): STALLED once the load had
    // dragged the arm to its stop, 1.7 s at >= 2.4 A, then 0 V. Margin: <= 2.5 s at >= 2.4 A of 30 s.
    ArmRig r{kStalls.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    REQUIRE(moveAndWatch(r, 95.0, 3.0).outcome == MechanismOutcome::Succeeded);
    const std::uint32_t seq = r.slot().commandSeq();
    r.plant->addLoadVolts(14.0);
    double atStall = 0.0;
    double maxAmps = 0.0;
    for (int i = 0; i < 3000; ++i) {
        r.tick();
        atStall += r.amps() >= 2.4 ? 0.01 : 0.0;
        maxAmps = std::max(maxAmps, r.amps());
    }
    CHECK(atStall <= 2.5);  // measured 1.66 s
    // WHY the build refuses a signature at 2.5 A or more (n58; the brief's adv_stallsig: a hand-built
    // 3.0 A one sat 29.9 s at stall current, never tripping): a motor limited at 2.5 A never reads more —
    // here, held against its stop at 12 V, exactly 2.5 — so such a signature can never trip. bind()
    // refuses it too (specRefusal), so no slot can carry one.
    CHECK(maxAmps <= 2.5);
    mech::MechanismSpec cannotTrip = kStalls.spec();
    cannotTrip.position.stall.amps = 3.0;
    CHECK(mech::specRefusal(cannotTrip) == mech::Refusal::AStallSignatureSAmpsMustBeBelowTheMotorS25ACurrentLimit);
    CHECK(r.stallFaults() == 1);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().commandSeq() == seq + 1);  // the hold ended AFTER its step: a held button is superseded
    CHECK(r.volts() == 0.0);
    CHECK_FALSE(r.mech->claimed());
    CHECK(r.mechCount("'arm': STALLED -- stalled in") == 1);
    CHECK(r.mechCount("(a stated stall signature tripped while holding)") == 1);
}

TEST_CASE("MECH-A3 position: N10 -- a driver's JOG into an obstacle trips the stated signature; the held button is superseded") {
    // adv_extra (2): +12 V held into an obstacle at 100 deg, the robot file's 2.0 A / 5 rpm / 0.5 s.
    // Measured: STALLED after 0.94 s held. Margin 1.5 s.
    ArmRig r{kInside.spec(), armPlant(200, 5, 1.6, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0)};
    r.plant->setObstacleDeg(100.0);
    std::uint32_t mine = 0;
    int t = 0;
    for (; t < 400; ++t) {
        r.slot().jog(units::Voltage{12.0});  // the driver keeps holding it
        mine = r.slot().commandSeq();
        r.tick();
        if (r.slot().lastStep().outcome == MechanismOutcome::Stalled) {
            break;
        }
    }
    CHECK(t * 0.01 <= 1.5);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().commandSeq() == mine + 1);  // bumped by the trip: the input that wrote `mine` is superseded
    CHECK(r.stallFaults() == 1);
    CHECK(r.volts() == 0.0);
    CHECK_FALSE(r.mech->claimed());
    CHECK(r.mechCount("(a stated stall signature tripped while held to move)") == 1);
}

TEST_CASE("MECH-A3 position: a homing push BY SWITCH is watched by the stated signature -- pushed into something, its switch never pressed") {
    // The switch never presses (an unplugged one reads released — HA-184) and the push is strong (12 V):
    // without the watch it sits at stall current against the stop for its whole bound.
    mech::MechanismSpec spec = kSwitchHome.spec();
    spec.position.homeVolts = -12.0;
    ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3, 40.0), Wiring{.switchByHand = true}};
    r.slot().home(units::Time{5.0});
    r.runStep(600);
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
    CHECK(r.slot().lastStep().elapsed < 2.0);  // the robot file's 0.5 s window, after it reached the stop
    CHECK(r.mechLines().back().find("its home switch never read pressed") != std::string::npos);
    CHECK_FALSE(r.slot().homed());
    CHECK(r.volts() == 0.0);
    CHECK(r.stallFaults() == 1);
}

TEST_CASE("MECH-A3 position: no HOLD target is ever past a limit -- a release and a TIMED_OUT move, the arm past its top") {
    // The re-check's latent defect (D2): a release held -1.30 deg on a 0..160 arm. Here the arm is driven
    // past its top limit by hand (the world alone; the hard stop is at 200), then the service holds it.
    // An exact binary tick (1/128 s) lands the 1/128 s bound exactly on the second tick.
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 0.0, 45.0, -2.0, 200.0), Wiring{.dt = 1.0 / 128.0}};
    const auto pastTheTop = [&r] {
        r.motors[0].setVoltage(units::Voltage{12.0});
        r.motors[1].setVoltage(units::Voltage{12.0});
        for (int i = 0; i < 1000 && r.trueDeg() < 175.0; ++i) {
            r.world.pace();  // the world only: nothing of the service runs
        }
        REQUIRE(r.trueDeg() >= 175.0);
    };
    pastTheTop();
    r.slot().holdHere();  // a release
    CHECK(r.slot().target() == 160.0);
    r.tick(384);
    CHECK(r.trueDeg() <= 161.0);  // pulled back to the limit and held there
    pastTheTop();
    r.slot().moveTo(150.0, units::Time{1.0 / 128.0}, false);
    r.tick(2);
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK(r.trueDeg() > 170.0);          // it stopped past the top...
    CHECK(r.slot().target() == 160.0);   // ... and holds AT it, never where it stopped (N2, clamped)
}

TEST_CASE("MECH-A3 position: the mode changes, the guard's reach and a released claim -- a HOLD is dropped to the safe state") {
    {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        REQUIRE(moveAndWatch(r, 95.0, 3.0).outcome == MechanismOutcome::Succeeded);
        REQUIRE(r.slot().holdsClaim());
        r.svc.enterDriverControl();  // finding 8: after an auton, driver entry drops a position HOLD
        CHECK_FALSE(r.mech->claimed());
        CHECK(r.volts() == 0.0);
        CHECK(r.motors[0].brakeMode() == shulib::hal::BrakeMode::Hold);  // its declared safe state
        r.tick(10);
        CHECK(r.volts() == 0.0);  // nothing re-commands it
    }
    {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.slot().moveTo(95.0, units::Time{3.0}, false);
        r.tick(20);
        REQUIRE(r.slot().running());
        r.slot().cancel();  // the guard's reach
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
        r.tick(10);
        CHECK(r.volts() == 0.0);
    }
    {
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        REQUIRE(moveAndWatch(r, 95.0, 3.0).outcome == MechanismOutcome::Succeeded);
        r.mech->releaseClaim();  // released outside the service (A2-D2): made safe at the next tick
        r.tick();
        CHECK(r.volts() == 0.0);
        r.svc.enterAutonomous();
        r.svc.leaveDriverControl();
        CHECK_FALSE(r.mech->claimed());
    }
    {
        // the most recent command wins: a jog mid-move CANCELS the move
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
        r.slot().moveTo(140.0, units::Time{3.0}, false);
        r.tick(10);
        r.slot().jog(units::Voltage{-4.0});
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(r.mechLines().back() == "'arm': CANCELLED in 0.10s (replaced by a newer command)");
        r.tick();
        CHECK(r.volts() == -4.0);
        r.slot().release();
        CHECK(r.volts() == 0.0);
        CHECK_FALSE(r.mech->claimed());
    }
}

TEST_CASE("MECH-A3 position: the preconditions -- checked BEFORE anything changes (a refused step commands nothing)") {
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    REQUIRE(moveAndWatch(r, 95.0, 3.0).outcome == MechanismOutcome::Succeeded);
    const std::uint32_t seq = r.slot().commandSeq();
    const std::uint32_t steps = r.slot().stepsBegun();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK(preconditionOf([&] { r.slot().moveTo(nan, units::Time{1.0}); }).find("finite") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().moveTo(90.0, units::Time{0.0}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().moveTo(90.0, units::Time{nan}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().moveToPreset(3, units::Time{1.0}); }).find("no such preset") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().home(units::Time{-1.0}); }).find("time limit") != std::string::npos);
    CHECK(preconditionOf([&] { r.slot().jog(units::Voltage{nan}); }).find("finite") != std::string::npos);
    CHECK(r.slot().commandSeq() == seq);  // nothing changed
    CHECK(r.slot().stepsBegun() == steps);
    CHECK(r.slot().holdsClaim());         // still holding 95
    r.tick(50);
    CHECK(r.trueDeg() == doctest::Approx(95.0).epsilon(0.02));
    // a preset by index moves to the BUILD-checked value
    r.slot().moveToPreset(2, units::Time{3.0});  // "high" = 140
    r.runStep();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(r.slot().target() == 140.0);
}

TEST_CASE("MECH-A3 position: an ABSENT arm or one ANOTHER owner holds -- every step says so, nothing moves") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    pos_rig::PlantWorld world;
    world.clock = &clock;
    const mech::MechanismSpec spec = kGreen5.spec();
    {
        mech::MechanismService svc{world};
        const mech::Devices absent{.absence = "port 1: nothing plugged in (expected a motor)"};
        svc.bind(std::span<const mech::MechanismSpec>{&spec, 1}, std::span<const mech::Devices>{&absent, 1}, clock,
                 faults, sink);
        mech::PositionSlot& a = svc.position(0);
        CHECK_FALSE(a.built());
        CHECK_FALSE(a.homed());  // nothing to home: no rests-at-stop zero logged for a mechanism not built
        a.moveTo(90.0, units::Time{1.0}, false);
        CHECK(a.lastStep().outcome == MechanismOutcome::Absent);
        a.home(units::Time{1.0});
        CHECK(a.lastStep().outcome == MechanismOutcome::Absent);
        a.jog(units::Voltage{6.0});
        a.holdHere();
        a.release();
        CHECK(a.position() == 0.0);
        CHECK(a.jointDeg() == 0.0);
        CHECK(svc.mechanisms().empty());
    }
    {
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3)};
        struct Other final : shulib::hal::ICancellable {
            int cancels = 0;
            void cancel() override { ++cancels; }
        } other;
        REQUIRE(r.mech->tryClaim(other));
        r.slot().moveTo(90.0, units::Time{1.0}, false);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Refused);
        r.slot().home(units::Time{1.0});
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Refused);
        r.motors[0].setVoltage(units::Voltage{3.0});  // the other owner's command
        r.slot().jog(units::Voltage{-6.0});
        r.slot().holdHere();
        r.slot().release();
        r.tick(5);
        CHECK(r.volts() == 3.0);  // untouched
        CHECK(other.cancels == 0);
        CHECK(r.mech->claimant() == &other);
    }
}

TEST_CASE("MECH-A3 position: robot x's ARM and LIFT through the REAL parts -- bind no longer refuses them (A2-D5)") {
    // Robot x's list booted over the fake provider exactly as a program does (MechanismParts), the
    // service bound over those devices — A2's bind() refused it ("their behaviours arrive with MECH-A3").
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeBattery battery;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    struct NullPacer final : shulib::motion::ITickPacer {
        void pace() override {}
    } providerPacer;
    shulib::hal::fake::FakeDeviceProvider devices{clock, battery, sink, providerPacer};
    std::array<shulib::config::MechanismConfig, mech::kMaxMechanisms> configs{};
    const mech::MechanismTable& t = rx::kRobotXMechanisms;
    for (std::size_t i = 0; i < t.count; ++i) {
        configs[i] = mech::toMechanismConfig(t.specs[i], t.wiring[i]);
    }
    shulib::robot::MechanismParts parts;
    parts.build(std::span<const shulib::config::MechanismConfig>{configs.data(), t.count}, devices);
    pos_rig::PlantWorld world;
    world.clock = &clock;
    mech::MechanismService svc{world};
    svc.bind(t, parts.devices(), clock, faults, sink);
    REQUIRE(svc.size() == 5U);
    CHECK(svc.mechanisms().size() == 5U);
    // the arm rests at its stop: homed at bind, logged; the lift is switch-homed, its switch released
    CHECK(svc.position(0).homed());
    CHECK_FALSE(svc.position(1).homed());
    CHECK(svc.find(rx::arm) == &svc.position(0));
    CHECK(svc.find(rx::lift) == &svc.position(1));
    CHECK(svc.find(rx::flywheel) == &svc.speed(2));
    CHECK(svc.find(rx::arm["score"]) == &svc.position(0));
    CHECK(svc.find(rx::lift.to(18_in)) == &svc.position(1));
    CHECK(svc.find(rx::flywheel["shoot"]) == &svc.speed(2));
    // the lift's home switch is the provider's input on 'B' (line 2): pressed, a homing step homes it
    svc.position(1).moveTo(8.0, units::Time{1.0}, false);
    CHECK(svc.position(1).lastStep().outcome == MechanismOutcome::NotHomed);
    devices.fakeDigitalIn(2).setState(true);
    svc.position(1).home(units::Time{1.0});
    svc.pace();
    CHECK(svc.position(1).lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(svc.position(1).homed());
    bool armLine = false;
    bool liftLine = false;
    for (int i = 0; i < sink.size(); ++i) {
        armLine = armLine || sink.at(i).message == "'arm': homed -- +0.0 deg from where its encoders read zero at boot";
        liftLine = liftLine || sink.at(i).message == "'lift': homed -- +0.0 deg from where its encoders read zero at boot";
    }
    CHECK(armLine);
    CHECK(liftLine);
    // the flywheel spins: its motor (port 4) is commanded
    svc.speed(2).spinUpPreset(0, units::Time{2.0});
    svc.pace();
    CHECK(devices.fakeMotor(4).commandedVoltage().value() > 0.0);
    CHECK(svc.speed(2).spinning());
}

TEST_CASE("MECH-A3 position: find() -- an ARM is never a same-named LIFT, and a GOAL's preset must be this robot's") {
    // a lift named "arm" on this robot; an Arm declaration named "arm" from somewhere else
    static constexpr Lift armLift{.name = "arm", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in), .presets = {{"floor", 0_in}}};
    const mech::MechanismSpec spec = armLift.spec();
    ArmRig r{spec, armPlant(100, 1, 1.0, 1.0, 0.3)};
    CHECK(r.svc.find(armLift) == &r.slot());
    CHECK(r.svc.find(kGreen5) == nullptr);              // an Arm named "arm": not this lift (degrees vs inches)
    CHECK(r.svc.find(armLift["floor"]) == &r.slot());
    CHECK(r.svc.find(kGreen5["score"]) == nullptr);     // a preset this robot's "arm" does not have
    CHECK(r.svc.find(armLift.clampedTo(3_in)) == &r.slot());  // a number: no preset to check
    CHECK(r.svc.find(rx::flywheel) == nullptr);
    CHECK(r.svc.find(rx::flywheel["shoot"]) == nullptr);
}

TEST_CASE("MECH-A3 position: bind's device preconditions for the new kinds -- each refused by name, the service left unbound") {
    shulib::hal::fake::FakeClock clock;
    shulib::hal::fake::FakeTelemetrySink sink;
    shulib::diag::FaultLatch faults{sink, clock};
    pos_rig::PlantWorld world;
    world.clock = &clock;
    shulib::hal::fake::FakeMotor m1;
    shulib::hal::fake::FakeMotor m2;
    std::array<shulib::hal::IMotor*, 1> one{&m1};
    std::array<shulib::hal::IMotor*, 2> two{&m1, &m2};
    shulib::hal::MotorMechanism g1{one, shulib::hal::BrakeMode::Hold, "x"};
    shulib::hal::MotorMechanism g2{two, shulib::hal::BrakeMode::Hold, "x"};
    shulib::hal::fake::FakeRotation rotation;
    shulib::hal::fake::FakeDigitalIn input;
    shulib::hal::fake::FakeReadFaults counter;
    const auto bindOne = [&](const mech::MechanismSpec& s, const mech::Devices& d) {
        mech::MechanismService svc{world};
        const std::string msg = preconditionOf([&] {
            svc.bind(std::span<const mech::MechanismSpec>{&s, 1}, std::span<const mech::Devices>{&d, 1}, clock, faults,
                     sink);
        });
        CHECK_FALSE(svc.bound());
        return msg;
    };
    const mech::MechanismSpec arm = rx::arm.spec();         // no angle sensor, rests at its stop
    const mech::MechanismSpec lift = rx::lift.spec();       // two motors, homed by its switch
    const mech::MechanismSpec wrist = ry::wrist.spec();     // an angle sensor
    const mech::MechanismSpec flywheel = rx::flywheel.spec();
    CHECK(bindOne(arm, mech::Devices{.motors = &g1, .angleSensor = &rotation}).find("angle sensor the declaration does not have") != std::string::npos);
    CHECK(bindOne(wrist, mech::Devices{.motors = &g1}).find("declares an angle sensor has none") != std::string::npos);
    CHECK(bindOne(lift, mech::Devices{.motors = &g2}).find("homed by its switch has none") != std::string::npos);
    CHECK(bindOne(arm, mech::Devices{.motors = &g1, .homeSwitch = &input}).find("home switch the declaration does not have") != std::string::npos);
    CHECK(bindOne(flywheel, mech::Devices{.motors = &g1, .homeSwitch = &input}).find("home switch the declaration does not have") != std::string::npos);
    CHECK(bindOne(arm, mech::Devices{.motors = &g1, .homeSwitchFaults = &counter}).find("read-fault count") != std::string::npos);
    CHECK(bindOne(lift, mech::Devices{.motors = &g1, .homeSwitch = &input}).find("motor count") != std::string::npos);
    CHECK(bindOne(arm, mech::Devices{}).find("needs its motors") != std::string::npos);
    // ABSENT: no devices at all — legal for every kind (a refused angle sensor or home switch made it so)
    mech::MechanismService ok{world};
    const std::array<mech::MechanismSpec, 3> specs{wrist, lift, flywheel};
    const std::array<mech::Devices, 3> absent{mech::Devices{.absence = "port 10: nothing"},
                                              mech::Devices{.absence = "3-wire B: refused"},
                                              mech::Devices{.absence = "port 4: nothing"}};
    ok.bind(specs, absent, clock, faults, sink);
    CHECK(ok.bound());
    CHECK(ok.mechanisms().empty());
}

TEST_CASE("MECH-A3 position: the starting values' BOOT LINE says which numbers are the library's (HA-182) and which the team's") {
    char line[320];
    mech::startingValuesLine(line, sizeof line, rx::arm.spec());
    CHECK(std::string{line} ==
          "arm: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 180.0 deg/s in 0.25 s, reached within "
          "1.50 deg for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides");
    mech::startingValuesLine(line, sizeof line, rx::lift.spec());
    CHECK(std::string{line} ==
          "lift: starting values kP 0.250 kI 0.300 kD 0.0040 kV 0.0200, cruise 5.9 in/s in 0.25 s, reached within "
          "0.25 in for 0.10 s -- PROVISIONAL (HA-182); .tuning overrides");
    mech::startingValuesLine(line, sizeof line, rx::flywheel.spec());
    CHECK(std::string{line} ==
          "flywheel: starting values kV 0.0318 kP 0.050 (V per output rad/s) kI 0.200 (V per output rad) -- "
          "PROVISIONAL (HA-182); .tuning overrides");  // kI per RAD: it multiplies the error's integral
    // robot y's wrist states kP 0.30 and within 2.00 — those two are the team's, the rest the library's
    mech::startingValuesLine(line, sizeof line, ry::wrist.spec());
    CHECK(std::string{line} ==
          "wrist: runs kP 0.300 (tuned) kI 0.300 kD 0.0040 kV 0.0300, cruise 300.0 deg/s in 0.25 s, reached within "
          "2.00 deg (tuned) for 0.10 s -- the rest are starting values, PROVISIONAL (HA-182)");
    // every number stated
    mech::MechanismSpec all = rx::arm.spec();
    all.position.kP = 0.4;
    all.position.kI = 0.5;
    all.position.kD = 0.01;
    all.position.kS = 0.2;
    all.position.kV = 0.06;
    all.position.kA = 0.001;
    all.position.maxVelocity = 100.0;
    all.position.maxAcceleration = 200.0;
    all.position.tolerance = 1.0;
    all.position.settleTime = 0.2;
    mech::startingValuesLine(line, sizeof line, all);
    CHECK(std::string{line}.find("-- every number stated by its .tuning") != std::string::npos);
    CHECK(std::string{line}.find(" kS 0.200 (tuned) kA 0.0010 (tuned)") != std::string::npos);
    CHECK(std::string{line}.find("cruise 100.0 deg/s (tuned) in 0.50 s (tuned)") != std::string::npos);
    // ONLY the speed stated: the ramp is still the starting 0.25 s (the acceleration follows the speed),
    // so the ramp is NOT marked and is counted a starting value (the review's finding: it read "(tuned)")
    mech::MechanismSpec speedOnly = rx::arm.spec();
    speedOnly.position.maxVelocity = 90.0;
    speedOnly.position.maxAcceleration = 90.0 / 0.25;  // what positionStart derives from a stated speed alone
    mech::startingValuesLine(line, sizeof line, speedOnly);
    CHECK(std::string{line} ==
          "arm: runs kP 0.250 kI 0.300 kD 0.0040 kV 0.0500, cruise 90.0 deg/s (tuned) in 0.25 s, reached within 1.50 "
          "deg for 0.10 s -- the rest are starting values, PROVISIONAL (HA-182)");
    // ... and only the acceleration stated: the ramp's TIME changed, so it is marked
    mech::MechanismSpec accelOnly = rx::arm.spec();
    accelOnly.position.maxAcceleration = 360.0;
    mech::startingValuesLine(line, sizeof line, accelOnly);
    CHECK(std::string{line}.find("cruise 180.0 deg/s in 0.50 s (tuned), reached") != std::string::npos);
    // the same through a real declaration's .tuning (positionStart derives the ramp from the stated speed)
    static constexpr Arm kSpeedTuned{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0), .tuning = {.maxSpeed = 90}};
    mech::startingValuesLine(line, sizeof line, kSpeedTuned.spec());
    CHECK(std::string{line}.find("cruise 90.0 deg/s (tuned) in 0.25 s, reached") != std::string::npos);
    // a power or pneumatic mechanism runs no loop: nothing
    mech::startingValuesLine(line, sizeof line, rx::intake.spec());
    CHECK(std::string{line}.empty());
    // bounded: a 40-byte buffer gets 39 characters and its NUL
    mech::startingValuesLine(line, 40, rx::arm.spec());
    CHECK(std::string{line}.size() == 39U);
    // the starting values are what positionStartingValues() / speedStartingValues() derive
    const mech::PositionSpec st = mech::positionStartingValues(rx::arm.spec());
    CHECK(st.kP == 0.25);
    CHECK(st.maxVelocity == doctest::Approx(180.0));
    CHECK(mech::speedStartingValues(rx::flywheel.spec()).kV == doctest::Approx(12.0 / (3600.0 * 2.0 * kPi / 60.0)));
}

TEST_CASE("MECH-A3 position: a tuned kS pushes WITH the move, a tuned kA with its acceleration -- exactly that much") {
    // Two arms identical but for ONE feedforward gain, absolute-homed at the same true 90 deg on identical
    // plants: their commanded volts differ by exactly that term. The plan's first sample is at rest
    // (speed 0: no kS yet) and accelerating (kA x the plan's acceleration, signed by the move); the
    // second is moving (kS signed by the move: a DOWNWARD move carries -kS). The starting values have
    // kS = kA = 0, so no other test reaches these terms (the review's mutations: kS always +, kA's sign
    // flipped — both survived).
    const mech::MechanismSpec base = kAbsolute.spec();
    for (const double target : {30.0, 150.0}) {
        const double dir = target < 90.0 ? -1.0 : 1.0;
        CAPTURE(target);
        mech::MechanismSpec withKs = base;
        withKs.position.kS = 0.5;
        mech::MechanismSpec withKa = base;
        withKa.position.kA = 0.001;
        ArmRig a{base, armPlant(200, 5, 2.0, 1.0, 0.3, 90.0), Wiring{.angle = sim::RotationMount{}}};
        ArmRig b{withKs, armPlant(200, 5, 2.0, 1.0, 0.3, 90.0), Wiring{.angle = sim::RotationMount{}}};
        ArmRig c{withKa, armPlant(200, 5, 2.0, 1.0, 0.3, 90.0), Wiring{.angle = sim::RotationMount{}}};
        for (ArmRig* r : {&a, &b, &c}) {
            r->slot().moveTo(target, units::Time{3.0}, false);
            r->tick();
        }
        CHECK(b.volts() == a.volts());  // at rest: no kS
        CHECK(c.volts() - a.volts() == doctest::Approx(dir * 0.001 * base.position.maxAcceleration).epsilon(1e-9));
        a.tick();
        b.tick();
        CHECK(b.volts() - a.volts() == doctest::Approx(dir * 0.5).epsilon(1e-9));  // moving: kS WITH the move
    }
}

TEST_CASE("MECH-A3 position: a LIFT's constant gravity term carries its load -- with no integral to cover for it") {
    // A heavy spool lift (the controller's kG 3 V, the plant's 3 V, no static friction) held at 12 in with
    // its hold's integral OFF: the gravity term alone carries the load, so it holds within 0.01 in.
    // Without the term (the review's mutation: Constant returns 0) the P term must carry 3 V — 12 deg of
    // its spool, 0.157 in of sag. (P2f's lift could not see it: its integral covered the 1 V.)
    mech::MechanismSpec spec = kLift.spec();
    spec.position.kG = 3.0;
    spec.position.kI = 0.0;
    sim::ArmPlantConfig pc = armPlant(100, 1, 3.0, 1.0, 0.0);
    pc.constantGravity = true;
    pc.minStopDeg = -1.0;
    pc.maxStopDeg = 24.5 / (kPi * 1.5) * 360.0;
    ArmRig r{spec, pc};
    r.slot().moveTo(12.0, units::Time{6.0}, false);
    r.runStep();
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    r.tick(200);
    CHECK(std::abs(r.slot().position() - 12.0) <= 0.01);
    CHECK(r.volts() == doctest::Approx(3.0).epsilon(0.05));  // the term itself, at rest
}

TEST_CASE("MECH-A3 position: a move BLOCKED short of its target for 5 s never winds its hold up to full power -- the 3 V clamp") {
    // An obstacle at 80 stops a move to 90 (bound 10 s). The plan ends; the hold's integral grows until
    // its clamp (3 V): measured at most 1.45 A — under the robot file's 2.0 A signature, so nothing
    // trips. The obstacle goes at 5 s: SUCCEEDED, overshoot 8.80 deg (the clamped integral unwinding).
    // Without the clamp the integral winds to full power and the signature trips (the review's
    // mutation: STALLED at 3.20 s). Margins: 1.9 A, 9.8 deg.
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 1.6, 1.0, 0.3)};
    r.plant->setObstacleDeg(80.0);
    r.slot().moveTo(90.0, units::Time{10.0}, false);
    double maxAmps = 0.0;
    for (int i = 0; i < 500; ++i) {
        r.tick();
        maxAmps = std::max(maxAmps, r.amps());
    }
    CHECK(r.slot().running());
    CHECK(maxAmps <= 1.9);
    CHECK(r.stallFaults() == 0);
    r.plant->clearObstacle();
    double over = 0.0;
    for (int i = 0; i < 600; ++i) {
        r.tick();
        over = std::max(over, r.trueDeg() - 90.0);
    }
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(over <= 8.80 + 1.0);
}

TEST_CASE("MECH-A3 position: REACHED needs the settle time UNBROKEN -- a ringing arm that leaves the tolerance starts its clock again") {
    // A ringing arm (its plant's spin-up 4 times the test plant's, kP 1.0 with no kD and no kI, a 0.25 s
    // settle): after its plan ends it passes in and out of the 1.5 deg tolerance (twice, measured). The
    // verdict must come only once it has been inside for 0.25 s WITHOUT a break: measured 1.85 s, the
    // last entry at 1.60 s. Without the clock's reset on leaving (the review's mutation) it said
    // SUCCEEDED at 1.32 s, the moment it re-entered.
    mech::MechanismSpec spec = kGreen5.spec();
    spec.position.kP = 1.0;
    spec.position.kD = 0.0;
    spec.position.kI = 0.0;
    spec.position.settleTime = 0.25;
    sim::ArmPlantConfig pc = armPlant(200, 5, 1.6, 1.0, 0.0);
    pc.kA = 0.2 * pc.kV;
    ArmRig r{spec, pc};
    r.slot().moveTo(90.0, units::Time{6.0}, false);
    const double planEnd = 90.0 / spec.position.maxVelocity + spec.position.maxVelocity / spec.position.maxAcceleration;
    double t = 0.0;
    double runStart = -1.0;
    bool wasIn = false;
    int exitsAfterPlan = 0;
    while (r.slot().running() && t < 6.5) {
        const bool in = std::abs(r.trueDeg() - 90.0) <= spec.position.tolerance;  // what the service reads this tick
        if (in && !wasIn) {
            runStart = t;
        }
        exitsAfterPlan += !in && wasIn && t > planEnd ? 1 : 0;
        wasIn = in;
        r.tick();
        t += 0.01;
    }
    REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
    CHECK(exitsAfterPlan >= 1);  // it rang: the case can see the rule
    CHECK(r.slot().lastStep().elapsed - runStart >= spec.position.settleTime - 0.005);
}

TEST_CASE("MECH-A3 position: a re-home warns past the tolerance -- 2 deg is a Warn, 1 deg is not; a table's end converts on its OTHER side") {
    SUBCASE("a stall-homed arm: a piece 2 deg above its stop moves the zero +2.0 (a Warn); 1 deg, +1.0 (Info)") {
        for (const double wedge : {0.0, -1.0}) {  // the stop is at -2
            CAPTURE(wedge);
            ArmRig r{kStallHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
            r.slot().home(units::Time{3.0});
            r.runStep(400);
            REQUIRE(r.trueDeg() == doctest::Approx(-2.0));
            REQUIRE(moveAndWatch(r, 30.0, 3.0).outcome == MechanismOutcome::Succeeded);
            r.plant->setWedgeDeg(wedge);  // a piece under the arm (a stop below it, from where it is now)
            r.slot().home(units::Time{3.0});
            r.runStep(400);
            REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
            const bool warn = wedge == 0.0;
            const std::string want = warn ? "'arm': homed again -- its zero moved +2.0 deg (past its tolerance: was something in the way?)"
                                          : "'arm': homed again -- its zero moved +1.0 deg";
            bool found = false;
            for (int i = 0; i < r.sink.size(); ++i) {
                if (r.sink.at(i).message == want) {
                    found = true;
                    CHECK(r.sink.at(i).level == (warn ? shulib::hal::LogLevel::Warn : shulib::hal::LogLevel::Info));
                }
            }
            CHECK(found);
        }
    }
    SUBCASE("a measured table stall-homed at its BOTTOM: its 0.25 in tolerance is 1.25 deg up and 0 down -- a 1 deg move is Info") {
        // At a table's end one side of the tolerance converts to 0 deg; the Warn's threshold is the larger
        // side (1.25 deg here). Measured from the one side below the end (the review's mutation) every
        // re-home there would be a Warn.
        static constexpr Lift kTableStall{.name = "tilt", .motors = {Forward}, .cartridge = Red, .gearing = gears(12, 84), .shape = measured(kPairs), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 20_in}, .gravity = noGravity, .home = homeByStall(0_in, -3_volt, Stall{.amps = 0.45, .belowMotorRpm = 3, .lasting = 0.1_s})};
        ArmRig r{kTableStall.spec(), armPlant(100, 7, 0.0, 1.0, 0.3, 30.0, 0.0, 0.0, 130.0)};
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        r.slot().moveTo(10.0, units::Time{4.0}, false);
        r.runStep();
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        r.plant->setWedgeDeg(1.0);
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        bool found = false;
        for (int i = 0; i < r.sink.size(); ++i) {
            if (r.sink.at(i).message == "'tilt': homed again -- its zero moved +1.0 deg") {
                found = true;
                CHECK(r.sink.at(i).level == shulib::hal::LogLevel::Info);
            }
        }
        CHECK(found);
    }
}

TEST_CASE("MECH-A3 position: every command RESTARTS the stall watch and the step clock -- no instant STALLED carried from the last one") {
    // Exact binary ticks (1/128 s): the signature's 0.5 s is 64 ticks after the window opens, so the
    // 65th stall-grade sample in a row trips it. Each subcase lets one behaviour show the service 64
    // such samples (one short of a trip), then gives the NEXT command just before the 65th — which must
    // get its own whole window (the review's mutations removed each reset: every one survived).
    constexpr double kTick = 1.0 / 128.0;
    // the rig's own reading of the signature on a tick, before the tick (what the service reads)
    const auto qualifies = [](ArmRig& r) {
        return r.amps() >= r.spec.position.stall.amps &&
               std::abs(r.motors[0].velocity().value()) <= r.spec.position.stall.speedRadS;
    };
    SUBCASE("a NEW jog after a release: its own window (and its STALLED counts from the jog)") {
        ArmRig r{kInside.spec(), armPlant(200, 5, 1.6, 1.0, 0.3, 20.0, 45.0, -10.0, 170.0), Wiring{.dt = kTick}};
        r.plant->setObstacleDeg(100.0);
        int seen = 0;  // stall-grade samples in a row the service has seen
        for (int i = 0; i < 2000; ++i) {
            if (qualifies(r) && seen == 64) {
                break;  // the next tick would be the 65th: the trip
            }
            seen = qualifies(r) ? seen + 1 : 0;
            r.slot().jog(units::Voltage{12.0});
            r.tick();
        }
        REQUIRE(seen == 64);
        REQUIRE(r.slot().lastStep().outcome != MechanismOutcome::Stalled);
        r.slot().release();
        const double since = r.clock.now().value();
        r.slot().jog(units::Voltage{12.0});  // a NEW hold-to-run, still pinned at the obstacle
        for (int i = 0; i < 200 && r.slot().lastStep().outcome != MechanismOutcome::Stalled; ++i) {
            r.tick();
        }
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
        CHECK(r.slot().lastStep().elapsed >= 0.5 - 1e-12);  // its own whole window
        CHECK(r.clock.now().value() - since >= 0.5);
        CHECK(r.slot().lastStep().elapsed <= 0.5 + 2 * kTick);  // ... counted from the jog, not from bind
    }
    SUBCASE("a move from a loaded HOLD (a heavy lift that holds at stall-grade current): its own window") {
        static constexpr Lift kHeavy{.name = "lift", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(10.0_volt), .home = restsAtStop(0_in)};
        sim::ArmPlantConfig pc = armPlant(100, 1, 10.0, 1.0, 0.3);
        pc.constantGravity = true;
        pc.minStopDeg = -1.0;
        pc.maxStopDeg = 24.5 / (kPi * 1.5) * 360.0;
        for (const bool viaHoldHere : {false, true}) {
            CAPTURE(viaHoldHere);
            ArmRig r{kHeavy.spec(), pc, Wiring{.dt = kTick}};
            r.slot().moveTo(1.0, units::Time{6.0}, false);  // 10 V of its 12 hold it up: a short move
            int seen = 0;  // counted from the move's start: its slow last ticks may already be stall-grade
            for (int i = 0; i < 2000; ++i) {  // then holding 10 V of gravity: 2.08 A, still
                if (qualifies(r) && seen == 64) {
                    break;
                }
                seen = qualifies(r) ? seen + 1 : 0;
                r.tick();
            }
            REQUIRE(seen == 64);
            REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
            if (viaHoldHere) {
                r.slot().holdHere();  // the release of a button: a NEW hold
                r.tick(66);
                // the hold squeezes at 2.08 A: it trips its signature — a WHOLE window after the release
                REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Stalled);
                CHECK(r.slot().lastStep().elapsed >= 0.5 - 1e-12);
                CHECK(r.slot().lastStep().elapsed <= 0.5 + 2 * kTick);  // counted from holdHere()
            } else {
                r.slot().moveTo(2.0, units::Time{6.0}, false);  // starts at rest, at 10 V: still stall-grade
                r.runStep(3000);
                CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);  // it moved before its own window ran out
            }
        }
    }
    SUBCASE("a SWITCH-homing push from a pinned jog: its own window") {
        mech::MechanismSpec spec = kSwitchHome.spec();
        spec.position.homeVolts = -12.0;  // a hard push: its first sample is stall-grade too
        spec.position.unhomedVolts = 12.0;  // before homing a push may run 12 V: pinned at an obstacle above
        ArmRig r{spec, armPlant(200, 5, 1.6, 1.0, 0.3, 40.0), Wiring{.homeSwitch = sim::SwitchMount{.atDeg = 0.0}, .dt = kTick}};
        r.plant->setObstacleDeg(40.5);
        int seen = 0;
        for (int i = 0; i < 2000; ++i) {
            if (qualifies(r) && seen == 64) {
                break;
            }
            seen = qualifies(r) ? seen + 1 : 0;
            r.slot().jog(units::Voltage{12.0});
            r.tick();
        }
        REQUIRE(seen == 64);
        REQUIRE(r.slot().lastStep().outcome != MechanismOutcome::Stalled);
        r.slot().home(units::Time{3.0});
        r.runStep(1000);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);  // pushed down to its switch, homed
        CHECK(r.slot().homed());
    }
    SUBCASE("a re-home by STALL from a loaded hold: the homing detector's own window -- the zero lands on the STOP") {
        // After a stall-home the homing detector's window stays open (it tripped); a later home() whose
        // first sample is stall-grade for it (a hold carrying a 3 V load, SETTLED: 1.08 A at rest, against
        // its 0.45 A and 3 motor rpm) must not trip on that sample — it would zero wherever it stands
        // (measured without the reset: "its zero moved +59.5 deg", the arm at 57.5 believed 0). The hold
        // must have SETTLED: 1 s after the load it still sags at 9.6 deg/s — not stall-grade, so a window
        // left open would close on that sample and the case could not see the reset (the review round's
        // first version of this case ticked 1 s and the mutation survived it).
        ArmRig r{kStallHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        REQUIRE(moveAndWatch(r, 62.0, 3.0).outcome == MechanismOutcome::Succeeded);
        r.plant->addLoadVolts(3.0);
        r.tick(400);
        // the homing detector's own signature reads this sample as a stall: the current AND the speed
        const mech::StallSpec hs = kStallHome.spec().position.homeStall;
        REQUIRE(r.amps() >= hs.amps);
        REQUIRE(std::abs(r.motors[0].velocity().value()) <= hs.speedRadS);
        r.plant->addLoadVolts(0.0);
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(-2.0));  // pushed to the STOP
        CHECK(r.slot().position() == doctest::Approx(0.0).epsilon(0.01));
        CHECK(r.mechCount("'arm': homed again -- its zero moved +0.0 deg") == 1);
    }
    SUBCASE("a move's FIRST tick commands its feedforward only -- no derivative carried from the last move") {
        // The move loop is reset at every move: its first update is P only (on a zero error). With kS = kA
        // = 0 the first tick is exactly the gravity hold at the arm's angle. Without the reset the last
        // move's measurement rides in: a D term on the 50 deg it moved since (measured +0.1 V).
        ArmRig r{kGreen5.spec(), armPlant(200, 5, 1.6, 1.0, 0.3)};
        REQUIRE(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
        for (int i = 0; i < 40; ++i) {
            r.slot().jog(units::Voltage{-6.0});
            r.tick();
        }
        r.slot().holdHere();
        r.tick(100);
        const double jd = r.slot().jointDeg();
        r.slot().moveTo(120.0, units::Time{3.0}, false);
        r.tick();
        const mech::PositionSpec& p = r.spec.position;
        CHECK(r.volts() == doctest::Approx(p.kG * std::cos((jd - p.horizontalAt) * kPi / 180.0)).epsilon(1e-12));
    }
}

TEST_CASE("MECH-A3 position: jog(0 V) HOLDS where it is -- the stick's zero is a release, never 0 V with the claim held") {
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    REQUIRE(moveAndWatch(r, 90.0, 3.0).outcome == MechanismOutcome::Succeeded);
    for (int i = 0; i < 10; ++i) {
        r.slot().jog(units::Voltage{6.0});
        r.tick();
    }
    const double at = r.slot().position();
    r.slot().jog(units::Voltage{0.0});
    CHECK(r.slot().target() == doctest::Approx(at).epsilon(1e-12));
    r.tick();
    CHECK(r.volts() > 0.5);  // the gravity hold, not 0 V
    r.tick(200);
    CHECK(std::abs(r.trueDeg() - r.slot().target()) <= 0.5);
    CHECK(r.slot().holdsClaim());
}

TEST_CASE("MECH-A3 position: a homing step's bound is met EXACTLY at its edge (1/128 s ticks: TIMED_OUT on the third)") {
    // home() at t = 0 with a 2/128 s bound: ticks at 0, 1/128 and 2/128 — the third is AT the bound,
    // and ">= the bound" is out of time there (the move's own edge, the same rule).
    ArmRig r{kSwitchHome.spec(), armPlant(200, 5, 2.0, 1.0, 0.3, 40.0), Wiring{.switchByHand = true, .dt = 1.0 / 128.0}};
    r.slot().home(units::Time{2.0 / 128.0});
    r.tick(2);
    CHECK(r.slot().running());
    r.tick();
    CHECK(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
    CHECK(r.slot().lastStep().elapsed == 2.0 / 128.0);
}

TEST_CASE("MECH-A3 position: a stall-home that NEVER trips says what it drew -- robot y's swing (a homing signature the push can reach) homes") {
    SUBCASE("the jam recipe's 2.0 A on a 3 V homing push: TIMED_OUT, and the line says it drew at most 0.62 A") {
        // A 3 V homing push stalled against its stop draws about 3/12 of the 2.5 A limit on the host test
        // plant (HA-93's shape, invented): a 2.0 A homing signature can never trip, so the step runs out
        // its time. The line says what the push DREW and what the signature needs.
        mech::MechanismSpec spec = kStallHome.spec();
        spec.position.homeStall.amps = 2.0;
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        r.slot().home(units::Time{3.0});
        r.runStep(400);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK_FALSE(r.slot().homed());
        CHECK(r.mechLines().back().find(
                  "(it never stalled against its stop: it drew at most 0.62 A when stopped, its homing signature needs "
                  "2.00 A)") != std::string::npos);
    }
    SUBCASE("robot y's swing: homeByStall(0, -3 V, 0.45 A) homes at its stop") {
        // Its homing signature said 1.5 A until the review — a stall a 3 V push can never draw (every
        // home() TIMED_OUT). Red 7:1, cosine gravity 1 V level at 30 deg, its stop at -2.
        ArmRig r{ry::swing.spec(), armPlant(100, 7, 1.0, 1.0, 0.3, 40.0, 30.0)};
        r.slot().home(units::Time{4.0});
        r.runStep(600);
        CHECK(r.slot().lastStep().outcome == MechanismOutcome::Succeeded);
        CHECK(r.trueDeg() == doctest::Approx(-2.0));
        CHECK(r.slot().position() == doctest::Approx(0.0).epsilon(0.01));
    }
}

TEST_CASE("MECH-A3 position: a stall-home's 'drew at most' is its OWN push's, when stopped -- never the behaviour before it, never a swing it brakes; each home() counts afresh") {
    // The number a homing signature is sized by: what the push draws STOPPED (no faster than the
    // signature's 3 motor rpm). On the host test plant a 3 V push stopped draws 0.625 A (HA-93's shape,
    // invented). A signature of 2.0 A never trips, so every home here runs out its time and says so.
    mech::MechanismSpec spec = kStallHome.spec();
    spec.position.homeStall.amps = 2.0;
    spec.position.unhomedVolts = 8.0;
    const std::string stopped =
        "(it never stalled against its stop: it drew at most 0.62 A when stopped, its homing signature needs 2.00 A)";
    SUBCASE("a jog PINNED at 8 V just before (1.67 A, still) is not the push's -- twice, so each home starts uncounted") {
        // The first homing tick's sample shows the behaviour BEFORE the push: it is never counted, on
        // the first home() and on the next (the second home's first sample is a pinned jog's again).
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        for (int k = 0; k < 2; ++k) {
            CAPTURE(k);
            r.plant->setObstacleDeg(40.0);
            for (int i = 0; i < 100; ++i) {
                r.slot().jog(units::Voltage{8.0});
                r.tick();
            }
            REQUIRE(r.trueDeg() == doctest::Approx(40.0));  // pinned against the obstacle ...
            REQUIRE(r.amps() > 1.6);                         // ... drawing far more than the push will
            r.plant->clearObstacle();
            r.slot().home(units::Time{3.0});
            r.runStep(400);
            REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
            CHECK(r.trueDeg() == doctest::Approx(-2.0));  // it did reach its stop
            CHECK(r.mechLines().back().find(stopped) != std::string::npos);
        }
    }
    SUBCASE("a home begun while the arm still swings UP: braking it at speed draws 1.29 A, not a stall's current") {
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        for (int i = 0; i < 10; ++i) {
            r.slot().jog(units::Voltage{8.0});
            r.tick();
        }
        REQUIRE(std::abs(r.motors[0].velocity().value()) > 5.0);  // rad/s at the motor: swinging up
        r.slot().home(units::Time{3.0});
        r.tick();
        REQUIRE(r.amps() > 1.2);  // the push's first sample: braking the swing, at speed
        r.runStep(400);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(r.mechLines().back().find(stopped) != std::string::npos);
    }
    SUBCASE("home #1 stopped on a wedged piece (0.62 A); home #2, too short to reach its stop, never slowed -- and says THAT") {
        ArmRig r{spec, armPlant(200, 5, 2.0, 1.0, 0.3, 30.0)};
        r.plant->setWedgeDeg(20.0);
        r.slot().home(units::Time{1.0});
        r.runStep(200);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        REQUIRE(r.trueDeg() == doctest::Approx(20.0));
        CHECK(r.mechLines().back().find(stopped) != std::string::npos);
        r.plant->clearWedge();
        r.slot().home(units::Time{0.15});  // 22 deg to its stop at about 15 joint rpm: it cannot get there
        r.runStep(100);
        REQUIRE(r.slot().lastStep().outcome == MechanismOutcome::TimedOut);
        CHECK(r.trueDeg() > 5.0);
        CHECK(r.mechLines().back().find("(it never stalled against its stop: it never slowed to its homing signature's "
                                        "speed -- did it reach its stop?)") != std::string::npos);
        CHECK(r.mechLines().back().find("drew at most") == std::string::npos);  // home #1's 0.62 A is not this push's
    }
}

TEST_CASE("MECH-A3 position: find() checks a goal's UNITS and its preset's VALUE -- a same-named stranger never moves this mechanism") {
    // this robot: an ARM named "arm" with "score" at 95 (kGreen5)
    static constexpr Lift liftNamedArm{.name = "arm", .motors = {Forward}, .cartridge = Red, .gearing = direct, .shape = spool(1.5_in), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0_in, 24_in}, .gravity = constantGravity(1.0_volt), .home = restsAtStop(0_in), .presets = {{"high", 22_in}, {"score", 18_in}}};
    static constexpr Arm otherArm{.name = "arm", .motors = {Forward}, .cartridge = Green, .gearing = gears(12, 60), .safe = Hold, .stall = {.amps = 2.0, .belowMotorRpm = 5, .lasting = 0.5_s}, .range = {0, 160}, .gravity = cosine(1.6_volt, 45), .home = restsAtStop(0), .presets = {{"score", 100}, {"high", 140}}};
    ArmRig r{kGreen5.spec(), armPlant(200, 5, 2.0, 1.0, 0.3)};
    CHECK(r.svc.find(kGreen5) == &r.slot());
    CHECK(r.svc.find(liftNamedArm) == nullptr);           // a LIFT named "arm" is not this arm
    CHECK(r.svc.find(liftNamedArm["high"]) == nullptr);   // ... nor any of its goals: inches, not degrees
    CHECK(r.svc.find(liftNamedArm["score"]) == nullptr);
    CHECK(r.svc.find(liftNamedArm.to(18_in)) == nullptr);
    CHECK(r.svc.find(liftNamedArm.clampedTo(3_in)) == nullptr);
    CHECK(r.svc.find(otherArm["score"]) == nullptr);      // a same-named arm's "score" at ANOTHER value (100, not 95)
    CHECK(r.svc.find(otherArm["high"]) == &r.slot());     // ... its "high" is this arm's, at the same 140
    CHECK(r.svc.find(otherArm.to(30)) == &r.slot());      // a number in degrees: no preset to check
    CHECK(r.svc.find(kGreen5["score"]) == &r.slot());
    CHECK(r.svc.find(kGreen5.clampedTo(175.0)) == &r.slot());   // a COMPUTED target, in degrees: found
    CHECK(r.svc.find(otherArm.clampedTo(175.0)) == &r.slot());  // ... whichever same-named arm made it
}
