// N1 — the run guard's latch CUTS a claimed mechanism (chunk MECH-A2; the MECH brief's N1 (a) with
// D5's wording, §1.8, §3.2, landmines 9 and 19). Against the full C4 stack on the host test plant:
// Chassis → RunGuard → MechanismService → the world, the chain a program builds (MECH-A5).
//
// THE DEFECT N1 CLOSES (the brief's P6, measured on the tree's guard): a mechanism step waits in a
// FROZEN wait (Chassis::waitUntil — what MECH-A4's Routine step uses), which the guard cannot cut, so
// an intake step still running at the deadline ran to its own bound: 1.01 s late with 12 V·s past the
// cut on a 2 s step; on a 5 s step 2.02 s late — the hard stop fired during scoring and the end
// action FAILED; after a disable, up to 2.52 s late. With the cut: one pace late, ~0 V·s.
//
// The step here is written the way MECH-A4's `r.run(intake, 12_volt, seesPiece, bound)` runs it —
// the slot's run() then a frozen wait on its verdict, a failed verdict stopping the chain — so this
// file pins the guard's half before the Routine exists.
//
// Every case tries to break one edge of WHERE the cut runs: the deadline cut NEVER inside the end
// action; an abandon cut EVERYWHERE, the end action included (B1); an UNCLAIMED mechanism never
// re-safed per pace (HA-144); an F1 operation with a registered claimant now stops at the deadline
// too (the behaviour change); a refused retry counted again; a throwing claimant not skipping the
// rest.

#include "doctest.h"

#include <array>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>

#include "mechanism_fixtures.hpp"
#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/fake/fake_distance.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/literals.hpp"

namespace {
namespace mech = shulib::mechanism;
namespace hal = shulib::hal;
namespace rx = mech_fixture::robot_x;
using mech::MechanismOutcome;
using mech::Until;
using shulib::control::ExitReason;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::sequence::RunGuardReport;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;
using namespace shulib::units::literals;

namespace lists {
using namespace shulib::mechanism::words;
inline constexpr MechanismTable kOnlyIntake = mechanisms(wire(rx::intake, motors(5), pieceSensor(9)));
inline constexpr Power roller{.name = "roller", .motors = {Forward}, .cartridge = Blue, .safe = Coast};
inline constexpr MechanismTable kTwoRollers = mechanisms(wire(rx::intake, motors(5), pieceSensor(9)), wire(roller, motors(6)));
}  // namespace lists

/// The world under the service: steps the drive plant (the harness's clock), requests a disable at
/// `abandonAt`, and integrates the intake's |V| after `cutAt` — the volt-seconds the cut leaves.
struct World final : shulib::motion::ITickPacer {
    motion_rig::MotionRig* rig = nullptr;
    hal::fake::FakeMotor* intakeMotor = nullptr;
    RunGuard* guard = nullptr;
    double abandonAt = 1e9;
    double cutAt = 1e9;
    double voltSecondsPast = 0.0;
    void pace() override {
        const double now = rig->h.clock().now().value();
        if (guard != nullptr && now >= abandonAt) {
            guard->requestAbandon();
        }
        if (now >= cutAt) {
            voltSecondsPast += std::abs(intakeMotor->commandedVoltage().value()) * 0.01;
        }
        rig->h.plant().step(Time{0.01});
    }
};

/// The whole chain over a hand-built intake (robot x's, a distance sensor that sees NO piece).
struct CutRig {
    shulib::kinematics::TankKinematics kin{Length{12.0}};
    hal::fake::FakeTelemetrySink sink;  // ONE sink: the chassis's (the guard logs there) and the service's
    motion_rig::MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    hal::fake::FakeMotor intakeMotor;
    std::array<hal::IMotor*, 1> ims{&intakeMotor};
    hal::MotorMechanism intake{ims, hal::BrakeMode::Coast, "intake"};
    hal::fake::FakeMotor rollerMotor;
    std::array<hal::IMotor*, 1> rms{&rollerMotor};
    hal::MotorMechanism roller{rms, hal::BrakeMode::Coast, "roller"};
    hal::fake::FakeDistance sensor;
    World world;
    mech::MechanismService svc{world};
    RunGuard guard{svc};
    shulib::chassis::Chassis chassis{rig.deps, guard, motion_rig::chassisConfig()};
    double t0 = 0.0;

    explicit CutRig(bool twoRollers = false) {
        sensor.setDistance(Length{393.66});  // the adapter's honest "no object"
        world.rig = &rig;
        world.intakeMotor = &intakeMotor;
        world.guard = &guard;
        if (twoRollers) {
            const std::array<mech::Devices, 2> d{mech::Devices{.motors = &intake, .pieceDistance = &sensor},
                                                 mech::Devices{.motors = &roller}};
            svc.bind(lists::kTwoRollers, d, rig.h.clock(), rig.latch, sink);
        } else {
            const mech::Devices d{.motors = &intake, .pieceDistance = &sensor};
            svc.bind(lists::kOnlyIntake, std::span<const mech::Devices>{&d, 1}, rig.h.clock(), rig.latch, sink);
        }
        chassis.setPose(Pose2d{});
        t0 = rig.h.clock().now().value();
    }
    [[nodiscard]] double since() { return rig.h.clock().now().value() - t0; }
    /// A mechanism STEP as MECH-A4's Routine runs it: start it, wait FROZEN on its verdict (bound +
    /// 0.25 s backstop); true when the chain may go on.
    bool runStep(mech::PowerSlot& s, double bound) {
        s.run(12_volt, Until::sees(), Time{bound});
        (void)chassis.waitUntil([&s] { return !s.running(); }, Time{bound + 0.25});
        return s.lastStep().outcome == MechanismOutcome::Succeeded;
    }
    [[nodiscard]] int seqLinesContaining(const std::string& needle) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "SEQ" && sink.at(i).message.find(needle) != std::string::npos) {
                ++n;
            }
        }
        return n;
    }
};

/// An IMechanism that counts its safe states (an unclaimed mechanism in the guard's span).
struct CountingMechanism final : hal::IMechanism {
    int safes = 0;
    void applySafeState() override { ++safes; }
    [[nodiscard]] const char* name() const noexcept override { return "counter"; }
};

/// A claimant whose cancel() throws (a broken adapter under it).
struct ThrowingClaimant final : hal::ICancellable {
    int calls = 0;
    void cancel() override {
        ++calls;
        throw std::runtime_error("broken claimant");
    }
};

}  // namespace

TEST_CASE("MECH-A2 N1 (P6): a step in a frozen wait at the DEADLINE is cut -- one pace late, ~0 V*s, CANCELLED; the end action runs and succeeds") {
    for (const double bound : {2.0, 5.0}) {
        CAPTURE(bound);
        CutRig g;
        g.world.cutAt = g.t0 + 1.0;
        double returnedAt = 0.0;
        bool driveRan = false;
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 3_s, .mechanisms = g.svc.mechanisms()},
            [&] {
                if (g.runStep(g.svc.power(0), bound)) {
                    driveRan = true;  // what the auton meant to do next
                    (void)g.chassis.moveTo(Pose2d{Length{20.0}, Length{0.0}, Angle{}});
                }
                returnedAt = g.since();
            },
            [&] { return g.chassis.brake() == ExitReason::Settled; });
        CHECK(returnedAt == doctest::Approx(1.01).epsilon(0.01));  // the tree's guard: T+2.01 / T+3.02
        CHECK(g.world.voltSecondsPast < 0.05);                     // the tree's guard: 12.0 / 24.1 V*s
        CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK_FALSE(driveRan);
        CHECK(rep.scoringCut);
        CHECK(rep.endActionRan);
        CHECK(rep.endActionSucceeded);  // the tree's guard, 5 s step: FAILED (the floor fired in scoring)
        CHECK_FALSE(rep.floorFired);
        CHECK(rep.mechanismCuts == 1);
        CHECK(g.intakeMotor.commandedVoltage().value() == 0.0);
        CHECK_FALSE(g.intake.claimed());
        CHECK(g.seqLinesContaining("mechanism 'intake' cut at the deadline") == 1);
    }
}

TEST_CASE("MECH-A2 N1 (P6): a DISABLE mid-step (the abandon) is cut the same -- TWO paces late (0.02 s), 0.12 V*s; the end action skipped") {
    for (const double bound : {2.0, 5.0}) {
        CAPTURE(bound);
        CutRig g;
        g.world.abandonAt = g.t0 + 0.5;
        g.world.cutAt = g.t0 + 0.5;
        double returnedAt = 0.0;
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 3_s, .mechanisms = g.svc.mechanisms()},
            [&] {
                (void)g.runStep(g.svc.power(0), bound);
                returnedAt = g.since();
            },
            [&] { return g.chassis.brake() == ExitReason::Settled; });
        // the disable at 0.50 s is READ at the guard's next pace (0.51 s), which cuts; the wait sees the
        // CANCELLED one pace later: 0.52 s — two paces, where the deadline costs one (the case above).
        // The tree's guard: T+2.01 / T+3.02 -- opcontrol() waited that long.
        CHECK(returnedAt == doctest::Approx(0.52).epsilon(0.005));
        CHECK(g.world.voltSecondsPast == doctest::Approx(0.12).epsilon(0.01));  // one pace at 12 V before the guard reads the request
        CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::Cancelled);
        CHECK(rep.abandoned);
        CHECK_FALSE(rep.endActionRan);
        CHECK(rep.mechanismCuts == 1);
        CHECK(g.seqLinesContaining("mechanism 'intake' cut after the abandon") == 1);
    }
}

TEST_CASE("MECH-A2 N1 (D5): the DEADLINE never cuts a step INSIDE the end action -- it runs to its own bound") {
    CutRig g;
    double endStarted = 0.0;
    double endReturned = 0.0;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 6_s, .mechanisms = g.svc.mechanisms()},
        [&] { (void)g.guard.pause(0.9_s); },
        [&] {
            endStarted = g.since();  // scoring returned at 0.9 s: the end action starts BEFORE the deadline ...
            const bool ok = g.runStep(g.svc.power(0), 3.0);
            endReturned = g.since();  // ... and runs through it (1 s) to its own bound
            return ok;
        });
    REQUIRE(endStarted < 1.0);
    CHECK(endReturned > 1.0);
    CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::TimedOut);  // its OWN bound: exempt
    // its own bound, to the tick: at least 3.0 s, at most one 10 ms pace past it (the plant's clock
    // accumulates 0.01 s steps, so 300 of them can land a hair short of 3.0)
    CHECK(g.svc.power(0).lastStep().elapsed >= 3.0 - 1e-9);
    CHECK(g.svc.power(0).lastStep().elapsed < 3.0 + 0.0101);
    CHECK(endReturned - endStarted == doctest::Approx(3.0).epsilon(0.01));
    CHECK(rep.mechanismCuts == 0);
    CHECK(g.seqLinesContaining("mechanism 'intake' cut") == 0);
    CHECK_FALSE(rep.floorFired);
}

TEST_CASE("MECH-A2 N1 (D5): an ABANDON cuts a step INSIDE the end action too (B1: it exempts nothing)") {
    CutRig g;
    g.world.abandonAt = g.t0 + 1.5;  // the field disables 0.5 s into the end action
    g.world.cutAt = g.t0 + 1.5;
    double endReturned = 0.0;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 6_s, .mechanisms = g.svc.mechanisms()},
        [&] { (void)g.guard.pause(0.9_s); },
        [&] {
            const bool ok = g.runStep(g.svc.power(0), 3.0);
            endReturned = g.since();
            return ok;
        });
    CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(endReturned < 1.535);  // the tree's guard: T+3.92, 28.9 V*s after the disable
    CHECK(g.world.voltSecondsPast < 0.13);
    CHECK(rep.abandoned);
    CHECK(rep.mechanismCuts == 1);
    CHECK_FALSE(rep.endActionSucceeded);
}

TEST_CASE("MECH-A2 N1: an UNCLAIMED mechanism is NOT re-safed every pace after the deadline (HA-144) -- only run()'s cancel-all safes it") {
    CutRig g;
    CountingMechanism counter;
    std::array<hal::IMechanism*, 2> mechs{g.svc.mechanisms()[0], &counter};
    int safesDuringScoring = -1;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = mechs},
        [&] {
            // a FROZEN wait 1 s past the deadline: 100 paces in the deadline branch
            (void)g.chassis.waitUntil([] { return false; }, 2_s);
            safesDuringScoring = counter.safes;
        },
        [] { return true; });
    CHECK(safesDuringScoring == 0);
    CHECK(counter.safes == 2);  // run()'s two cancel-alls: before and after the end action
    CHECK(rep.mechanismCuts == 0);
}

TEST_CASE("MECH-A2 N1 BEHAVIOUR CHANGE: an F1 operation with a registered claimant now stops at the DEADLINE, not at its own bound") {
    CutRig g;
    shulib::manipulation::MechanismDeps deps{.clock = &g.rig.h.clock(), .faults = &g.rig.latch, .telemetry = &g.sink};
    const shulib::manipulation::RunUntilConfirmedConfig cfg{
        .voltage = Voltage{9.0},
        .timeout = Time{4.0},
        .stall = shulib::manipulation::StallConfig{.currentAtLeast = shulib::units::Current{999.0},
                                                   .speedAtMost = shulib::units::AngularVelocity{0.0},
                                                   .persistence = Time{10.0}}};
    shulib::manipulation::RunUntilConfirmed op{g.roller, deps, cfg, [] { return false; }, "spin"};
    std::array<hal::IMechanism*, 1> mechs{&g.roller};
    double returnedAt = 0.0;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 6_s, .mechanisms = mechs},
        [&] {
            op.start();
            (void)g.chassis.waitUntil([&op] { return op.tick() != MechanismOutcome::Running; }, 5_s);
            returnedAt = g.since();
        },
        [] { return true; });
    CHECK(op.outcome() == MechanismOutcome::Cancelled);  // before N1: TIMED_OUT at its own 4 s
    CHECK(returnedAt == doctest::Approx(1.01).epsilon(0.01));
    CHECK(rep.mechanismCuts == 1);
    CHECK(g.rollerMotor.commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 N1: a refused RETRY is cut again and counted again; the first cut is said once PER RUN; the count resets per run") {
    CutRig g{true};
    int retries = 0;
    RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = g.svc.mechanisms()},
        [&] {
            g.svc.power(1).run(12_volt, Until::bound(), 3_s);  // the roller runs beside the intake
            while (retries < 3) {  // a stubborn caller retries a failed step (the latch refuses each)
                ++retries;
                (void)g.runStep(g.svc.power(0), 2.0);
            }
        },
        [] { return true; });
    CHECK(rep.mechanismCuts == 4);  // the intake's first step AND the roller at the deadline, then two retries
    CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(g.svc.power(1).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(g.seqLinesContaining(" cut at the deadline") == 1);
    // A SECOND run that cuts too: its count starts at 0 (one cut, not five), and ITS first cut is
    // said again — the "logged once" is per run, or every later run's guide-11 line would vanish.
    rep = g.guard.run(g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = g.svc.mechanisms()},
                      [&] { (void)g.runStep(g.svc.power(0), 2.0); }, [] { return true; });
    CHECK(rep.mechanismCuts == 1);
    CHECK(g.svc.power(0).lastStep().outcome == MechanismOutcome::Cancelled);
    CHECK(g.seqLinesContaining(" cut at the deadline") == 2);
    // ... and a third that cuts nothing counts nothing
    rep = g.guard.run(g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = g.svc.mechanisms()},
                      [] {}, [] { return true; });
    CHECK(rep.mechanismCuts == 0);
    CHECK(g.seqLinesContaining(" cut at the deadline") == 2);
}

TEST_CASE("MECH-A2 N1: a claimant whose cancel() THROWS does not skip the mechanisms after it; the first exception propagates") {
    CutRig g{true};
    ThrowingClaimant bad;
    REQUIRE(g.intake.tryClaim(bad));
    bool thrown = false;
    try {
        (void)g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = g.svc.mechanisms()},
            [&] {
                g.svc.power(1).run(12_volt, Until::bound(), 3_s);
                (void)g.chassis.waitUntil([] { return false; }, 2_s);
            },
            [] { return true; });
    } catch (const std::runtime_error& e) {
        thrown = std::string{e.what()} == "broken claimant";
    }
    CHECK(thrown);
    CHECK(bad.calls >= 1);
    CHECK(g.svc.power(1).lastStep().outcome == MechanismOutcome::Cancelled);  // reached despite the throw before it
    CHECK(g.rollerMotor.commandedVoltage().value() == 0.0);
}

TEST_CASE("MECH-A2 N1 (A2-D11): a claimant that throws on EVERY cut, its exception CAUGHT by scoring -- the mechanisms after it are still cut, while scoring is still running") {
    // The case above lets the exception escape scoring(), so run()'s own cancel-all (which has its own
    // catch-all) would cancel the roller anyway, and the cut's attempt-every-mechanism rule is never
    // what the test sees. Here a team's try/catch around a step catches the throw and keeps waiting:
    // only the CUT can reach the roller, at the deadline, before scoring returns.
    CutRig g{true};
    ThrowingClaimant bad;
    REQUIRE(g.intake.tryClaim(bad));  // listed FIRST: every cut meets the throw before the roller
    int caught = 0;
    double rollerVoltsAtFirstCatch = -1.0;
    bool rollerCancelledAtFirstCatch = false;
    double firstCatchAt = -1.0;
    bool runThrew = false;
    try {
        (void)g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = 1_s, .hardStopAt = 5_s, .mechanisms = g.svc.mechanisms()},
            [&] {
                g.svc.power(1).run(12_volt, Until::bound(), 3_s);  // the roller, beside it
                while (caught < 5) {
                    try {
                        (void)g.chassis.waitUntil([] { return false; }, 2_s);
                        break;
                    } catch (const std::runtime_error&) {
                        if (caught == 0) {
                            firstCatchAt = g.since();
                            rollerVoltsAtFirstCatch = g.rollerMotor.commandedVoltage().value();
                            rollerCancelledAtFirstCatch =
                                g.svc.power(1).lastStep().outcome == MechanismOutcome::Cancelled;
                        }
                        ++caught;
                    }
                }
            },
            [] { return true; });
    } catch (const std::runtime_error&) {
        runThrew = true;  // run()'s own cancel-all meets the broken claimant too: a loud error, rethrown
    }
    REQUIRE(caught >= 1);
    CHECK(firstCatchAt == doctest::Approx(1.0).epsilon(0.02));  // the deadline pace
    CHECK(rollerVoltsAtFirstCatch == 0.0);                        // cut on THAT pace, despite the throw before it
    CHECK(rollerCancelledAtFirstCatch);
    CHECK(bad.calls >= 1);
    CHECK(runThrew);
}
