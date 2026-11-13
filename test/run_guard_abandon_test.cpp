// RunGuard::requestAbandon — the third way a run ends (chunk COMP-a; the COMP brief's ruling C5 and
// amendments B1, B10, B22). Every case names the bug it would catch.
//
// The abandon arrives the way the competition program will deliver it (B7): from a pacer UNDER
// the guard — the "watch" — whose pace() runs inside the guard's pace() and calls
// guard.requestAbandon() when its (here: scripted) competition state says the robot was disabled.
// So the latency the tests see is the real one: the request lands during one pace and the guard
// acts on it at the next check.
//
// Graded on PLANT GROUND TRUTH (rig.h.truePose()) and on counts the test keeps itself, never on
// the guard's own account of what it did.

#include "doctest.h"

#include <array>
#include <cstdio>
#include <stdexcept>
#include <string>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/manipulation/mechanism_op.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

using motion_rig::MotionRig;
using motion_rig::PlantPacer;
using motion_rig::chassisConfig;
using motion_rig::posErr;
using shulib::PreconditionError;
using shulib::chassis::Chassis;
using shulib::chassis::Routine;
using shulib::chassis::RoutineStopCause;
using shulib::control::ExitReason;
using shulib::hal::BrakeMode;
using shulib::hal::IMechanism;
using shulib::hal::IMotor;
using shulib::hal::MotorMechanism;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::xDrive;
using shulib::manipulation::MechanismDeps;
using shulib::manipulation::MechanismOutcome;
using shulib::manipulation::RunUntilConfirmed;
using shulib::manipulation::RunUntilConfirmedConfig;
using shulib::manipulation::StallConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sequence::GuardedWaitResult;
using shulib::sequence::RunGuard;
using shulib::sequence::RunGuardConfig;
using shulib::sequence::RunGuardReport;
using shulib::units::AngularVelocity;
using shulib::units::Current;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Voltage;

namespace {

constexpr double kTick = 0.01;  // PlantPacer's dt

/// The competition program's watch, reduced to a script: a pacer UNDER the guard that asks the
/// guard to abandon once the clock reaches `disableAt` (B7's layering: guard -> watch -> plant).
/// It asks on EVERY pace from then on — level, not edge — exactly as run_guard.hpp's banner tells
/// a real watch to.
struct WatchPacer final : shulib::motion::ITickPacer {
    WatchPacer(PlantPacer& inner, shulib::hal::IClock& clock) : plant{&inner}, clk{&clock} {}
    void pace() override {
        if (guard != nullptr && guard->running() && clk->now().value() >= disableAt) {
            guard->requestAbandon();
            ++requests;
        }
        plant->pace();
    }
    PlantPacer* plant;
    shulib::hal::IClock* clk;
    RunGuard* guard = nullptr;  // set after the guard is constructed (B7: the same order)
    double disableAt = 1e9;
    int requests = 0;
};

/// A pacer ABOVE the guard (the chassis's pacer), forwarding to it, that sees each tick's command
/// BEFORE the guard's check — so it can count the ticks on which a motion actually commanded a
/// wheel (B22), which a pacer below the guard cannot see (the cut has already zeroed them).
struct ProbePacer final : shulib::motion::ITickPacer {
    ProbePacer(RunGuard& g, shulib::sim::SimHarness& harness) : guard{&g}, h{&harness} {}
    void pace() override {
        bool commanded = false;
        for (int i = 0; i < h->motorCount(); ++i) {
            if (h->motor(i).commandedVoltage().value() != 0.0) {
                commanded = true;
            }
        }
        if (commanded) {
            ++commandedTicks;
        }
        guard->pace();
    }
    RunGuard* guard;
    shulib::sim::SimHarness* h;
    int commandedTicks = 0;
};

/// plant -> watch -> guard -> chassis: the competition program's stack, on the host plant.
struct AbandonRig {
    MatrixKinematics kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    MotionRig rig;
    PlantPacer plant;
    WatchPacer watch;
    RunGuard guard;
    Chassis chassis;

    AbandonRig()
        : rig{kin, motion_rig::plantConfig(), &sink},
          plant{rig.h},
          watch{plant, rig.h.clock()},
          guard{watch},
          chassis{rig.deps, guard, chassisConfig()} {
        watch.guard = &guard;
        chassis.setPose(Pose2d{});
    }

    [[nodiscard]] double now() { return rig.h.clock().now().value(); }

    [[nodiscard]] bool seqLineContains(const std::string& needle) const {
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "SEQ" && sink.at(i).message.find(needle) != std::string::npos) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] int seqLineCount(const std::string& needle) const {
        int n = 0;
        for (int i = 0; i < sink.size(); ++i) {
            if (sink.at(i).subsystem == "SEQ" && sink.at(i).message.find(needle) != std::string::npos) {
                ++n;
            }
        }
        return n;
    }

    /// The T+ seconds printed on the `nth` (from 0) "run ABANDONED at T+" line — the abandon's
    /// own line, which carries WHEN it landed — or -1 when there is no such line.
    [[nodiscard]] double abandonedLineAt(int nth) const {
        static const std::string kPrefix = "run ABANDONED at T+";
        int seen = 0;
        for (int i = 0; i < sink.size(); ++i) {
            const std::string& msg = sink.at(i).message;
            const std::size_t at = msg.find(kPrefix);
            if (sink.at(i).subsystem != "SEQ" || at == std::string::npos) {
                continue;
            }
            if (seen++ == nth) {
                double t = -1.0;
                return std::sscanf(msg.c_str() + at + kPrefix.size(), "%lf", &t) == 1 ? t : -1.0;
            }
        }
        return -1.0;
    }

    void checkDriveSafe() {
        for (const IMotor* m : rig.h.context().driveMotors()) {
            CHECK(m->commandedVoltage().value() == 0.0);
            CHECK(m->brakeMode() == BrakeMode::Brake);
        }
    }
};

/// The %.2f rounding margin on a printed T+ time, below the instant the watch first asks.
constexpr double kPrinted = 0.006;

/// One mechanism with a declared HOLD safe state, and an operation config whose own watchdog would
/// outlive any test (so only the guard can end it).
struct Pivot {
    shulib::hal::fake::FakeMotor motor;
    std::array<IMotor*, 1> motors{&motor};
    MotorMechanism mech;
    MechanismDeps deps;
    RunUntilConfirmedConfig cfg{.voltage = Voltage{9.0},
                                .timeout = Time{60.0},
                                .stall = StallConfig{.currentAtLeast = Current{999.0},
                                                     .speedAtMost = AngularVelocity{0.0},
                                                     .persistence = Time{10.0}}};
    Pivot(AbandonRig& g, const char* name)
        : mech{motors, BrakeMode::Hold, name},
          deps{.clock = &g.rig.h.clock(), .faults = &g.rig.latch, .telemetry = &g.sink} {}
};

const Pose2d kFar{Length{300.0}, Length{0.0}, Angle{}};

}  // namespace

// ── B1: abandon during a MOTION ─────────────────────────────────────────────────────────

// Bug caught: an abandon that disarms (the listed mutation) or that waits for the deadline. The
// motion in flight must be cut at the next check after the request, the routine must unwind
// NORMALLY (its verb returns Cancelled, its chain stops), every later guard call must answer
// RunExpired WITHOUT throwing, a later motion start must be refused, the end action must be
// SKIPPED, and the guard must still be running until run() returns — then disarm, leaving the
// scheduler idle and callable.
TEST_CASE("B1 abandon mid-motion: cut at once, the routine unwinds, the end action is skipped") {
    AbandonRig g;
    g.watch.disableAt = 1.0;
    bool endActionCalled = false;
    bool runningAtEndOfScoring = false;
    GuardedWaitResult waitAfter = GuardedWaitResult::Satisfied;
    GuardedWaitResult pauseAfter = GuardedWaitResult::Satisfied;
    bool expiredAfter = false;
    double remainingAfter = -1.0;
    ExitReason firstVerdict = ExitReason::Running;
    ExitReason retryVerdict = ExitReason::Running;
    Pose2d truthAtCut{};
    double cutAt = 0.0;

    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{12.0}},
        [&] {
            firstVerdict = g.chassis.moveTo(kFar, {.timeout = Time{60.0}});
            cutAt = g.now();
            truthAtCut = g.rig.h.truePose();
            // every later guard call answers RunExpired, and none throws (a disarmed guard would)
            waitAfter = g.guard.waitFor([] { return true; }, Time{5.0});
            pauseAfter = g.guard.pause(Time{5.0});
            expiredAfter = g.guard.expired();
            remainingAfter = g.guard.remaining().value();
            retryVerdict = g.chassis.moveTo(kFar, {.timeout = Time{60.0}});
            runningAtEndOfScoring = g.guard.running();
        },
        [&] {
            endActionCalled = true;
            return true;
        });

    CHECK(firstVerdict == ExitReason::Cancelled);  // the honest verdict: it was cancelled
    CHECK(cutAt >= 1.0);
    CHECK(cutAt <= 1.0 + 3.0 * kTick);  // the request lands in one pace, acts at the next
    CHECK(waitAfter == GuardedWaitResult::RunExpired);
    CHECK(pauseAfter == GuardedWaitResult::RunExpired);
    CHECK(expiredAfter);
    CHECK(remainingAfter == 0.0);
    CHECK(retryVerdict == ExitReason::Cancelled);          // a later start is refused...
    CHECK(posErr(g.rig.h.truePose(), truthAtCut) < 0.01);  // ...and moves nothing
    CHECK(runningAtEndOfScoring);                          // the guard never disarmed early
    CHECK_FALSE(endActionCalled);                          // skipped: the robot is disabled
    CHECK(rep.abandoned);
    CHECK_FALSE(rep.endActionRan);
    CHECK_FALSE(rep.endActionSucceeded);
    CHECK_FALSE(rep.scoringCut);  // the DEADLINE never came: the run was abandoned, not cut
    CHECK(rep.pacesSeen > 0);
    CHECK(g.seqLineContains("ABANDONED"));
    CHECK(g.seqLineContains("skipped"));
    // The abandon's own line, with WHEN it landed — once, however many refusals followed it.
    CHECK(g.seqLineCount("run ABANDONED at T+") == 1);
    CHECK(g.abandonedLineAt(0) >= 1.0 - kPrinted);
    CHECK(g.abandonedLineAt(0) <= 1.0 + 3.0 * kTick);
    CHECK_FALSE(g.guard.running());  // disarmed when run() returned
    g.checkDriveSafe();
    // The scheduler is idle and callable: the next mode can drive at once.
    CHECK_FALSE(g.chassis.scheduler().hasActiveMotion());
    const Pose2d after{Length{g.rig.h.truePose().x().value() + 6.0}, Length{0.0}, Angle{}};
    CHECK(g.chassis.moveTo(after, {.timeout = Time{8.0}}) == ExitReason::Settled);
}

// Bug caught: the abandon not reaching a wait IN FLIGHT — the guard's waits are the waits the
// competition program tells students to use precisely so that a disable can end them.
TEST_CASE("B1 abandon inside guard.waitFor and guard.pause: both return RunExpired at once") {
    SUBCASE("waitFor") {
        AbandonRig g;
        g.watch.disableAt = 0.7;
        GuardedWaitResult w = GuardedWaitResult::Satisfied;
        double endedAt = 0.0;
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{12.0}},
            [&] {
                w = g.guard.waitFor([] { return false; }, Time{60.0});
                endedAt = g.now();
            },
            [] { return true; });
        CHECK(w == GuardedWaitResult::RunExpired);
        CHECK(endedAt >= 0.7);
        CHECK(endedAt <= 0.7 + 3.0 * kTick);
        CHECK(rep.abandoned);
        CHECK_FALSE(rep.endActionRan);
        CHECK_FALSE(g.guard.running());
        // The WAIT saw the abandon first (its predicate runs before the next pace), so the
        // abandon's line must come from the wait's side of the latch — no pace ever saw it.
        CHECK(g.abandonedLineAt(0) >= 0.7 - kPrinted);
        CHECK(g.abandonedLineAt(0) <= 0.7 + 3.0 * kTick);
    }
    SUBCASE("pause") {
        AbandonRig g;
        g.watch.disableAt = 0.7;
        GuardedWaitResult w = GuardedWaitResult::Satisfied;
        double endedAt = 0.0;
        const RunGuardReport rep = g.guard.run(
            g.chassis, RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{12.0}},
            [&] {
                w = g.guard.pause(Time{30.0});
                endedAt = g.now();
            },
            [] { return true; });
        CHECK(w == GuardedWaitResult::RunExpired);
        CHECK(endedAt <= 0.7 + 3.0 * kTick);  // not T+30
        CHECK(rep.abandoned);
        CHECK_FALSE(g.guard.running());
    }
}

// Bug caught: an end action treated as EXEMPT from an abandon the way it is exempt from the
// deadline — a park driven into a disable is a routine commanding a robot the field took back.
TEST_CASE("B1 abandon DURING the end action: its motion is cut and its verdict is void") {
    AbandonRig g;
    g.watch.disableAt = 1.0;
    ExitReason endVerdict = ExitReason::Running;
    double endReturnedAt = 0.0;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{0.5}, .hardStopAt = Time{12.0}},
        [] { /* scoring finishes at once */ },
        [&] {
            endVerdict = g.chassis.moveTo(kFar, {.timeout = Time{60.0}});
            endReturnedAt = g.now();
            return endVerdict;
        });
    CHECK(endVerdict == ExitReason::Cancelled);
    CHECK(endReturnedAt <= 1.0 + 3.0 * kTick);  // cut at the abandon, not at the floor (T+12)
    CHECK(rep.abandoned);
    CHECK(rep.endActionRan);             // it WAS invoked — honestly reported
    CHECK_FALSE(rep.endActionSucceeded);  // but no verdict survives an abandon
    CHECK_FALSE(rep.floorFired);
    CHECK(g.seqLineContains("during the end-of-run action"));
    g.checkDriveSafe();
    CHECK_FALSE(g.guard.running());
}

// Bug caught: an end action whose OWN verdict survives an abandon. This one reports success no
// matter what (a "park" that pauses for the partner and says true) — the abandon cuts its pause,
// and the report must still say the end action did not succeed: a success claimed about a park
// the field disabled halfway through is the verdict-honesty failure F2 exists to prevent.
TEST_CASE("B1 an abandon voids even an end action that reports success") {
    AbandonRig g;
    g.watch.disableAt = 1.0;
    GuardedWaitResult paused = GuardedWaitResult::Satisfied;
    const RunGuardReport rep = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{0.5}, .hardStopAt = Time{12.0}},
        [] {},
        [&] {
            paused = g.guard.pause(Time{5.0});
            return true;  // it CLAIMS success
        });
    CHECK(paused == GuardedWaitResult::RunExpired);  // the abandon cut the pause…
    CHECK(rep.endActionRan);
    CHECK(rep.abandoned);
    CHECK_FALSE(rep.endActionSucceeded);  // …and the claim does not survive it
    CHECK(rep.endActionEnded.value() <= 1.0 + 3.0 * kTick);
}

// Bug caught: a stale abandon. The flag stays readable after the abandoned run returns (the
// program reads it), but the NEXT run must start clean — or every run after one disable would be
// abandoned at T+0 and the robot would never drive an auton again on that boot.
TEST_CASE("B1 a request from the last run (or while idle) does not abandon the next run") {
    AbandonRig g;
    g.watch.disableAt = 0.5;
    const RunGuardReport first = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{5.0}, .hardStopAt = Time{6.0}},
        [&] { (void)g.chassis.moveTo(kFar, {.timeout = Time{60.0}}); }, [] { return true; });
    REQUIRE(first.abandoned);
    CHECK(g.guard.abandonRequested());  // still readable after run() returned
    g.guard.requestAbandon();           // and a request while idle...
    g.watch.disableAt = 1e9;            // ...but the field enables the robot again
    const Pose2d near{Length{g.rig.h.truePose().x().value() + 8.0}, Length{0.0}, Angle{}};
    ExitReason v = ExitReason::Running;
    const RunGuardReport second = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{20.0}, .hardStopAt = Time{25.0}},
        [&] { v = g.chassis.moveTo(near, {.timeout = Time{8.0}}); }, [] { return true; });
    CHECK(v == ExitReason::Settled);
    CHECK_FALSE(second.abandoned);
    CHECK(second.endActionRan);
    CHECK(second.endActionSucceeded);
}

// Bug caught (the COMP-a review's finding 5a): the abandon's own log line — the one line that
// says WHEN the field took the robot back, which COMP-b's practice-match station reads — lost, or
// logged for the first abandoned run of a boot and never again (a once-per-run latch not reset
// when the next run arms). Every other ABANDONED needle the suite uses is also satisfied by the
// verdict line run() prints at its end, so only this case can see the latch's line go. Two runs
// on ONE guard, abandoned at different times into each: each logs its own line, with its own T+
// measured from ITS start. Both routines only drive (no guard wait), so the line must come from
// the pace side of the latch.
TEST_CASE("B1 each abandoned run logs its own 'run ABANDONED at T+' line, at its own time") {
    AbandonRig g;
    g.watch.disableAt = 0.5;
    const RunGuardReport first = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{5.0}, .hardStopAt = Time{6.0}},
        [&] { (void)g.chassis.moveTo(kFar, {.timeout = Time{60.0}}); }, [] { return true; });
    REQUIRE(first.abandoned);
    REQUIRE(g.seqLineCount("run ABANDONED at T+") == 1);

    g.watch.disableAt = g.now() + 0.8;  // the second run is abandoned 0.8 s into IT
    const Pose2d back{Length{-300.0}, Length{0.0}, Angle{}};
    const RunGuardReport second = g.guard.run(
        g.chassis, RunGuardConfig{.endActionAt = Time{5.0}, .hardStopAt = Time{6.0}},
        [&] { (void)g.chassis.moveTo(back, {.timeout = Time{60.0}}); }, [] { return true; });
    REQUIRE(second.abandoned);

    CHECK(g.seqLineCount("run ABANDONED at T+") == 2);
    CHECK(g.abandonedLineAt(0) >= 0.5 - kPrinted);
    CHECK(g.abandonedLineAt(0) <= 0.5 + 3.0 * kTick);
    CHECK(g.abandonedLineAt(1) >= 0.8 - kPrinted);  // from the SECOND run's start, not the boot's
    CHECK(g.abandonedLineAt(1) <= 0.8 + 3.0 * kTick);
}

// ── B10: abandon's cancel-all is run()'s FULL cancel-all ────────────────────────────────

// Bug caught: an abandon that stops the drive but leaves a mechanism operation holding its claim —
// driver control's bindings never command a claimed mechanism (T3's rule), so the pivot would be
// dead for the rest of the match. After the abandon: the op is cancelled THROUGH ITS CLAIM, no
// mechanism is claimed, each sits in its DECLARED safe state, an anonymous claim is force-released,
// and the next user of the pivot can take it.
TEST_CASE("B10 an abandon during a mechanism operation leaves no mechanism claimed") {
    AbandonRig g;
    Pivot left{g, "left pivot"};
    Pivot right{g, "right pivot"};
    std::array<IMechanism*, 2> mechs{&left.mech, &right.mech};
    g.watch.disableAt = 0.8;
    bool confirmed = false;
    RunUntilConfirmed op{left.mech, left.deps, left.cfg, [&] { return confirmed; }, "grab"};
    REQUIRE(right.mech.tryClaim());  // an ANONYMOUS claim the guard cannot cancel, only release
    right.motor.setVoltage(Voltage{6.0});

    const RunGuardReport rep = g.guard.run(
        g.chassis,
        RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{12.0}, .mechanisms = mechs},
        [&] {
            op.start();
            (void)g.guard.waitFor([&] { return op.tick() != MechanismOutcome::Running; },
                                  Time{60.0});
        },
        [] { return true; });

    CHECK(rep.abandoned);
    CHECK(op.outcome() == MechanismOutcome::Cancelled);
    CHECK_FALSE(left.mech.claimed());
    CHECK_FALSE(right.mech.claimed());
    CHECK(rep.anonymousClaimsReleased == 1);
    CHECK(left.motor.commandedVoltage().value() == 0.0);
    CHECK(left.motor.brakeMode() == BrakeMode::Hold);
    CHECK(right.motor.commandedVoltage().value() == 0.0);
    CHECK(right.motor.brakeMode() == BrakeMode::Hold);
    // The next user of the pivot (driver control) can take it and drive it.
    REQUIRE(left.mech.tryClaim());
    left.mech.setVoltage(Voltage{5.0});
    CHECK(left.motor.commandedVoltage().value() == 5.0);
    left.mech.releaseClaim();
    g.checkDriveSafe();
}

// ── B22: the one-tick latch ─────────────────────────────────────────────────────────────

// Bug caught: the abandon checked AFTER the world advances (the step-then-check ordering the F2
// campaign measured at 10.79 in past a deadline). A routine that keeps starting motions after the
// request gets each one cut with AT MOST ONE commanded tick and ZERO plant travel: the probe above
// the guard counts the ticks that commanded a wheel, the plant below it moves not at all.
TEST_CASE("B22 after an abandon every refused start commands at most one tick and moves nothing") {
    MatrixKinematics kin = xDrive(Length{7.0});
    FakeTelemetrySink sink;
    MotionRig rig{kin, motion_rig::plantConfig(), &sink};
    PlantPacer plant{rig.h};
    WatchPacer watch{plant, rig.h.clock()};
    RunGuard guard{watch};
    ProbePacer probe{guard, rig.h};
    Chassis chassis{rig.deps, probe, chassisConfig()};  // chassis -> probe -> guard -> watch -> plant
    watch.guard = &guard;
    watch.disableAt = 1.0;
    chassis.setPose(Pose2d{});

    constexpr int kStarts = 25;
    Pose2d truthAfterCut{};
    int commandedBefore = 0;
    int refused = 0;
    const RunGuardReport rep = guard.run(
        chassis, RunGuardConfig{.endActionAt = Time{10.0}, .hardStopAt = Time{12.0}},
        [&] {
            (void)chassis.moveTo(kFar, {.timeout = Time{60.0}});  // the cut
            truthAfterCut = rig.h.truePose();
            commandedBefore = probe.commandedTicks;
            for (int i = 0; i < kStarts; ++i) {
                if (chassis.moveTo(kFar, {.timeout = Time{60.0}}) == ExitReason::Cancelled) {
                    ++refused;
                }
            }
        },
        [] { return true; });

    CHECK(rep.abandoned);
    CHECK(refused == kStarts);
    const int commandedByRetries = probe.commandedTicks - commandedBefore;
    CHECK(commandedByRetries <= kStarts);  // at most ONE commanded tick per refused start
    CHECK(commandedByRetries >= 1);        // (and the probe does see them: not vacuous)
    CHECK(posErr(rig.h.truePose(), truthAfterCut) < 1e-9);  // zero plant travel
    CHECK(rep.postExpiryCancels == kStarts + 1);           // the cut + every refusal, counted
}
