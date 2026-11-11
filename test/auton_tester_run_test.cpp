// auton_tester_run_test.cpp — ONE run of "shulib Auton Tester" on the host test plant
// (src/programs/auton_tester_run.hpp's runTesterOnce, stopAfterProgramError, testerResultWord and
// testerRunLines; chunk COMP-t, 2026-09-28; the brief's §5 tests (14)–(23); and the chunk review's of
// 2026-09-29: (15b), (18b), (18c), (20b), (21d), (21e), (22b), (23b)). Every case names the bug it
// would catch.
//
// The stack is the program's pacer chain with the plant in the provider's place
// (test/auton_tester_rig.hpp): Chassis -> RunGuard -> the REAL AbortWatch -> a probe layer (the
// painter's place) -> the plant. Graded on plant GROUND TRUTH — PlantPacer::distance, the plant's
// travel — and on the drive motors' commanded state, never on the estimate the motions read.
// Host test plant numbers (instant motors, a tank that never skids); nothing here has run on a
// robot (worksheet Station M).

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>

#include "auton_tester_rig.hpp"
#include "routine_golden_chains.hpp"
#include "motion_test_rig.hpp"
#include "programs/auton_abort_watch.hpp"
#include "programs/auton_tester_panel.hpp"
#include "programs/auton_tester_run.hpp"
#include "team/autons.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/fake/fake_mechanism.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/units/literals.hpp"
#include "shulib/units/quantity.hpp"

using shulib::bench::AbortCause;
using shulib::bench::kTesterCountdownSeconds;
using shulib::bench::kTesterLineBytes;
using shulib::bench::kTesterTickSeconds;
using shulib::bench::stopAfterProgramError;
using shulib::bench::TesterRunLines;
using shulib::bench::TesterRunRecord;
using shulib::bench::testerResultWord;
using shulib::bench::testerRunLines;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonRunOutcome;
using shulib::competition::StartPose;
using shulib::diag::FaultCode;
using shulib::hal::ControllerButton;
using shulib::kinematics::TankKinematics;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using tester_rig::TesterStack;

namespace {

const TankKinematics& tank() {
    static const TankKinematics k{Length{12.0}};
    return k;
}

AutonRegistry teamRegistry() {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::bench::registerAutons(reg);
    return reg;
}

bool says(std::string_view text, std::string_view needle) {
    return text.find(needle) != std::string_view::npos;
}

// The commanded speed's one tick: moveTo's cap (60 in/s) x 10 ms, plus rounding slack.
constexpr double kOneTickTravel = 60.0 * 0.01 + 1e-6;

/// A routine that drives 10 in, then trips a precondition (a programming error) with the drive
/// still energized by the last command.
RoutineResult driveThenThrow(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    SHULIB_PRECONDITION(false, "test: a routine tripped a precondition");
    return r.result();
}

AutonEntry routineEntry(const char* name, shulib::competition::AutonRoutine fn) {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Match;
    e.routine = fn;
    e.start = StartPose::relative();
    e.endActionAt = Time{27.0};
    e.hardStopAt = Time{29.0};
    return e;
}

/// A routine that drives 10 in, then throws a STANDARD exception that is not a precondition (a
/// student's std::out_of_range from an .at()) — runAuton does not catch, the guard rethrows any
/// type after safing, so it reaches runTesterOnce's std::exception handler.
RoutineResult driveThenThrowStd(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw-std"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    if (r.result().steps >= 0) {
        throw std::out_of_range("test: a student's std::out_of_range");
    }
    return r.result();
}

/// A routine that drives 10 in, then throws something that is not a std::exception at all — only
/// runTesterOnce's catch (...) can take it.
RoutineResult driveThenThrowInt(AutonContext& ctx) {
    Routine r{ctx.chassis, "drive-then-throw-int"};
    r.moveTo(Pose2d{Length{10.0}, Length{0.0}, Angle{}});
    if (r.result().steps >= 0) {
        throw 42;
    }
    return r.result();
}

/// True when a [SEQ] line saying the end-of-run action started has been logged since `from`.
bool endActionStarted(const shulib::hal::fake::FakeTelemetrySink& sink, int from) {
    for (int i = from; i < sink.size(); ++i) {
        if (sink.at(i).message.find("end-of-run action starting") != std::string::npos) {
            return true;
        }
    }
    return false;
}

/// A mechanism whose safe state THROWS (a broken adapter), counting its calls.
struct ThrowingMechanism final : shulib::hal::IMechanism {
    int calls = 0;
    void applySafeState() override {
        ++calls;
        throw std::runtime_error("test: a mechanism's safe state threw");
    }
    [[nodiscard]] const char* name() const noexcept override { return "thrower"; }
};

}  // namespace

// (14) Bug caught: a countdown shorter than 3 s (the operator has no time to step back), a run that
// starts moving in the countdown, and the quiet tick removed (mutation 28: the RUNNING page's full
// repaint would land in the run's first commanded tick). The loop ends on the first tick whose
// elapsed time is >= 3.0 s, and floating accumulation makes that tick 301 on the plant (3.0100 s) —
// so the bound is 3.0 <= countdown <= 3.0 + one tick (+1e-9). Then ONE quiet tick: the guard arms
// one tick after runStarted. Nothing is commanded until the guard is running, and the plant has not
// moved at runStarted NOR at the guard's first live pace.
TEST_CASE("run (14): a full 3 s countdown, one quiet tick, and nothing moves until the guard runs") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const TesterRunRecord& r = s.run(reg, 0);
    REQUIRE(s.obs.countdownCalls == 1);
    REQUIRE(s.obs.runCalls == 1);
    CHECK(s.obs.countdownAt == r.countdownStartedAt);
    const double countdown = s.obs.runAt - s.obs.countdownAt;
    CHECK(countdown >= kTesterCountdownSeconds);
    CHECK(countdown <= kTesterCountdownSeconds + kTesterTickSeconds + 1e-9);
    const double quiet = r.run.armedAt.value() - s.obs.runAt;
    CHECK(quiet >= kTesterTickSeconds - 1e-9);
    CHECK(quiet <= kTesterTickSeconds + 1e-9);
    CHECK(s.probe.energizedWhileIdle == 0);
    CHECK(s.obs.travelAtRun == 0.0);
    CHECK(s.probe.travelAtGuardArm == 0.0);
    CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    // The hand-off the log reports is B12's number: the quiet tick, and nothing else.
    const double handOff = r.run.armedAt.value() - (r.countdownStartedAt + kTesterCountdownSeconds);
    CHECK(handOff > 0.0);
    CHECK(handOff <= 2.0 * kTesterTickSeconds + 1e-9);
}

// (15) Bug caught: an abort in the countdown that still runs the auton (mutation 8: the post-loop
// check inverted), or a countdown that ignores its abort and runs out its 3 s anyway (mutation 7:
// the loop without its cause check — the post-loop check still refuses the run, so travel stays 0;
// only the countdown's LENGTH shows it, hence the return-time bound).
TEST_CASE("run (15): B 1.5 s into the countdown aborts it: runAuton never called, nothing moves") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const double start = s.now();
    double latchAt = -1.0;
    s.inputs.onRead = [&](double now) {
        if (now - start >= 1.5 && latchAt < 0.0) {
            s.inputs.pad.setPressed(ControllerButton::B, true);
            latchAt = now;
        }
    };
    const double before = s.plant.distance;
    const int logBefore = s.sink.size();
    const TesterRunRecord& r = s.run(reg, 0);
    const double returnedAt = s.now();

    CHECK(r.countdownAborted);
    CHECK(r.cause == AbortCause::Button);
    CHECK(r.buttons == (1u << 9));
    CHECK(s.obs.runCalls == 0);  // no runStarted: runAuton was never reached
    CHECK(r.run.guard.pacesSeen == 0);
    CHECK(r.run.outcome == AutonRunOutcome::Refused);  // the untouched default: never called
    CHECK(s.plant.distance - before == 0.0);
    CHECK(s.probe.energizedWhileIdle == 0);
    CHECK_FALSE(s.watch.armed());
    for (int i = logBefore; i < s.sink.size(); ++i) {
        CHECK_FALSE(says(s.sink.at(i).message, "arming"));  // no [COMP] line: nothing armed
        CHECK(s.sink.at(i).subsystem != "COMP");
    }
    REQUIRE(latchAt >= 0.0);
    CHECK(returnedAt - latchAt <= kTesterTickSeconds + 1e-9);  // within one tick of the latch
    CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
    const TesterRunLines lines = testerRunLines(r, reg.at(0));
    REQUIRE(lines.count == 1);
    CHECK(std::string_view{lines.line[0]}
          == "run 1: '40in forward' ABORTED in the countdown by a controller button B -- runAuton "
             "was never called; nothing moved");
}

// (15b) Bug caught (COMP-t's review, 2026-09-29; re-probed): a finger that lands AFTER the read whose
// release touched RUN but BEFORE the watch arms — the RUN line's SD write is in that gap on the
// brain — counted into the baseline and NEVER aborting, even held through the whole countdown, so
// the robot moves with a finger on the glass. runTesterOnce arms with RUN's own read's press count:
// the landed finger is an abort at the countdown's first tick, and nothing moves. (Before the fix
// the watch baselined its own later read: this run FINISHED, 40 in, with the finger down.)
TEST_CASE("run (15b): a finger landing between RUN's read and the arm aborts; nothing moves") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const std::int32_t runRead = s.inputs.touch.press_count;  // RUN's release came in on this read
    s.inputs.press();  // ...and a finger lands before the watch arms (and stays down)
    const double before = s.plant.distance;
    const double start = s.now();
    const TesterRunRecord& r = s.runFrom(runRead, reg, 0);
    CHECK(r.countdownAborted);
    CHECK(r.cause == AbortCause::Touch);
    CHECK(s.obs.runCalls == 0);
    CHECK(s.plant.distance - before == 0.0);
    CHECK(s.now() - start <= kTesterTickSeconds + 1e-9);  // at the countdown's first tick
    CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
    // The same run with RUN's read AT the current count (nothing landed since): no abort.
    TesterStack clean{tank()};
    const TesterRunRecord& c = clean.runFrom(clean.inputs.touch.press_count, reg, 0);
    CHECK_FALSE(c.countdownAborted);
    CHECK(std::string_view{testerResultWord(c)} == "FINISHED");
}

// (16) Bug caught: a button HELD when RUN is touched not aborting until it is released (a level
// read as an edge) — the countdown must end at its FIRST tick.
TEST_CASE("run (16): a button held at RUN aborts at the first countdown tick") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    s.inputs.pad.setPressed(ControllerButton::R2, true);
    const double start = s.now();
    const TesterRunRecord& r = s.run(reg, 0);
    CHECK(r.countdownAborted);
    CHECK(r.cause == AbortCause::Button);
    CHECK(r.buttons == (1u << 3));
    CHECK(s.now() - start <= kTesterTickSeconds + 1e-9);
    CHECK(s.obs.runCalls == 0);
}

// (17) Bug caught (mutation 2): the controller dropping in the countdown not aborting — the abort
// would have no hands for the run that follows.
TEST_CASE("run (17): the controller dropping in the countdown -> ControllerLost, nothing moves") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const double start = s.now();
    s.inputs.onRead = [&](double now) {
        if (now - start >= 0.7) {
            s.inputs.pad.setConnected(false);
        }
    };
    const double before = s.plant.distance;
    const TesterRunRecord& r = s.run(reg, 0);
    CHECK(r.countdownAborted);
    CHECK(r.cause == AbortCause::ControllerLost);
    CHECK(s.plant.distance - before == 0.0);
    const TesterRunLines lines = testerRunLines(r, reg.at(0));
    CHECK(says(lines.line[0], "ABORTED in the countdown by the controller disconnected -- "));
}

// (18) Bug caught: an abort mid-run that is not the guard's clean abandon — the end action run
// anyway (a brake is fine; a park driven after an abort is not), the drive left energized, the
// scheduler still holding a motion, the guard still armed; or more than ONE commanded tick after
// the latch (mutation 9: disarm() moved before runAuton, so the watch never latches in the run).
TEST_CASE("run (18): B mid-run -> abandoned within one tick, end action skipped, all safe") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const double start = s.now();
    s.inputs.onRead = [&](double now) {
        if (now - start >= kTesterCountdownSeconds + 0.45) {
            s.inputs.pad.setPressed(ControllerButton::B, true);
        }
    };
    const TesterRunRecord& r = s.run(reg, 0);
    REQUIRE(r.run.outcome == AutonRunOutcome::RoutineRan);
    CHECK_FALSE(r.countdownAborted);
    CHECK(r.cause == AbortCause::Button);
    CHECK(r.run.guard.abandoned);
    CHECK_FALSE(r.run.guard.endActionRan);
    CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
    CHECK(s.driveSafe());
    CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
    CHECK_FALSE(s.guard.running());
    CHECK_FALSE(s.watch.armed());
    REQUIRE(s.probe.travelAtLatch > 0.0);  // it was moving when B went down
    CHECK(s.plant.distance - s.probe.travelAtLatch <= kOneTickTravel);
    const TesterRunLines lines = testerRunLines(r, reg.at(0));
    CHECK(says(lines.line[0], "ABORTED -- ROUTINE_RAN; ABANDONED by a controller button B, end "
                              "action skipped"));
    CHECK(says(lines.line[1], "end action ended T+--,"));
}

// (18b) Bug caught (COMP-t's review, 2026-09-29; through the mechanism service since MECH-A5): the
// robot's PRESENT mechanisms not reaching the guard — runTesterOnce handing runAuton nothing to safe
// (mutation M1: `present.first(0)` until MECH-A5; now the span overload in place of the service's) —
// so the claws keep their last command at the end of a run and on an abort. And a step IN FLIGHT on a
// claw (its service slot, the registered claimant): the guard must CANCEL it — inert, safe, released —
// or it would command again at the next tick. An auton that starts pivot 6 for 20 s and then drives
// the 40 in, through runTesterOnce: a clean run safes every mechanism at its end and cancels the step;
// B mid-run safes every mechanism AND cancels the step at the abandon; B DURING the end action safes
// every mechanism; a countdown abort touches none (the guard never ran, the step never began).
namespace {
shulib::chassis::RoutineResult pivotWhileDriving(shulib::competition::AutonContext& ctx) {
    using namespace shulib::bench::robot_two;
    using namespace shulib::units::literals;
    shulib::chassis::Routine r{ctx, "pivot-while-driving"};
    r.startRun(clawPivot6, 6_volt, 20_s);  // a step in flight on a claw, beyond the run's own end
    r.moveTo(shulib::math::Pose2d{40_in, 0_in, 0_deg});
    return r.result();
}
AutonRegistry pivotRegistry() {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    shulib::competition::AutonEntry e{};
    e.name = "pivot while driving";
    e.kind = shulib::competition::AutonKind::Match;
    e.routine = &pivotWhileDriving;
    e.start = shulib::competition::StartPose::relative();
    e.endActionAt = Time{27.0};
    e.hardStopAt = Time{29.0};
    reg.add(e);
    return reg;
}
}  // namespace

TEST_CASE("run (18b): every PRESENT mechanism is safed at a run's end and on an abort, and a step in "
          "flight on a claw is CANCELLED") {
    const AutonRegistry reg = pivotRegistry();
    enum class Shape { Clean, MidRun, InEndAction, Countdown };
    for (const Shape shape : {Shape::Clean, Shape::MidRun, Shape::InEndAction, Shape::Countdown}) {
        CAPTURE(static_cast<int>(shape));
        TesterStack s{tank()};
        const int sixBefore = s.two.pivot6.safes;
        const int sevenBefore = s.two.pivot7.safes;
        const int clawBefore = s.two.claw.safes;
        bool pivotRanLive = false;
        const double start = s.now();
        const int from = s.sink.size();
        s.inputs.onRead = [&](double now) {
            pivotRanLive = pivotRanLive || s.two.m6.commandedVoltage().value() == 6.0;
            const bool press =
                (shape == Shape::MidRun && now - start >= kTesterCountdownSeconds + 0.45)
                || (shape == Shape::InEndAction && endActionStarted(s.sink, from))
                || (shape == Shape::Countdown && now - start >= 1.0);
            if (press) {
                s.inputs.pad.setPressed(ControllerButton::B, true);
            }
        };
        const TesterRunRecord& r = s.run(reg, 0, 1);
        if (shape == Shape::Countdown) {
            CHECK(r.countdownAborted);
            CHECK(s.two.pivot6.safes == sixBefore);  // nothing ran: nothing was touched
            CHECK(s.two.pivot7.safes == sevenBefore);
            CHECK(s.two.claw.safes == clawBefore);
            CHECK_FALSE(pivotRanLive);
            CHECK(s.mechanisms.slot(0).stepsBegun() == 0);
            continue;
        }
        CHECK(pivotRanLive);  // the step really drove the pivot while the robot drove
        CHECK(s.driveSafe());
        CHECK(s.two.pivot6.safes > sixBefore);
        CHECK(s.two.pivot7.safes > sevenBefore);
        CHECK(s.two.claw.safes > clawBefore);
        CHECK(s.mechanismsSafe());
        // The step in flight was CANCELLED — inert and released, so nothing commands it again.
        CHECK(s.mechanisms.slot(0).lastStep().outcome == shulib::manipulation::MechanismOutcome::Cancelled);
        CHECK_FALSE(s.two.pivot6.claimed());
        CHECK_FALSE(s.mechanisms.slot(0).running());
        CHECK_FALSE(s.guard.running());
        CHECK_FALSE(s.watch.armed());
        for (int i = 0; i < 5; ++i) {
            s.chassis.wait(Time{0.01});
            CHECK(s.two.m6.commandedVoltage().value() == 0.0);
        }
        if (shape == Shape::Clean) {
            CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
        } else {
            CHECK(r.run.guard.abandoned);
            CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
            CHECK(r.run.guard.endActionRan == (shape == Shape::InEndAction));
        }
    }
}

// (18c) Bug caught (COMP-t's review, 2026-09-29): an abort that lands WHILE the end action (the
// brake) runs reported as "end action FAILED" — the guard cuts the end action and voids its verdict
// ("run ABANDONED during the end-of-run action -- its verdict is void"), nothing is wrong with the
// brake, and FAILED would send the operator looking for a fault that does not exist. The log's first
// line says "CUT by the abort", RESULT says "end action CUT", and the drive is safe.
TEST_CASE("run (18c): B during the end action -> the end action CUT by the abort, never FAILED") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    const int from = s.sink.size();
    s.inputs.onRead = [&](double /*now*/) {
        if (endActionStarted(s.sink, from)) {
            s.inputs.pad.setPressed(ControllerButton::B, true);
        }
    };
    const TesterRunRecord& r = s.run(reg, 0);
    REQUIRE(r.run.guard.abandoned);
    REQUIRE(r.run.guard.endActionRan);
    CHECK_FALSE(r.run.guard.endActionSucceeded);  // the guard's own report: the verdict is void
    CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
    CHECK(s.driveSafe());
    const TesterRunLines lines = testerRunLines(r, reg.at(0));
    CHECK(std::string_view{lines.line[0]}
          == "run 1: '40in forward' ABORTED -- ROUTINE_RAN; ABANDONED by a controller button B, end "
             "action CUT by the abort");
    CHECK_FALSE(says(lines.line[0], "FAILED"));
    shulib::bench::TesterScreen screen{reg};
    shulib::bench::TesterFacts f;
    f.now = 100.0;
    screen.showResult(r, 0, false);
    const shulib::bench::TesterView v = screen.view(f);
    REQUIRE(v.lineCount == 6);
    CHECK(std::string_view{v.lines[0].text} == "aborted by: button B");
    CHECK(says(v.lines[1].text, "end action CUT; scoring T+"));
}

// (19) Bug caught: a touch or a disable mid-run not aborting, or recorded as the wrong cause.
TEST_CASE("run (19): a touch mid-run -> Touch; a disable mid-run -> Disabled") {
    const AutonRegistry reg = teamRegistry();
    {
        TesterStack s{tank()};
        const double start = s.now();
        bool pressed = false;
        s.inputs.onRead = [&](double now) {
            if (!pressed && now - start >= kTesterCountdownSeconds + 0.3) {
                s.inputs.press();
                pressed = true;
            }
        };
        const TesterRunRecord& r = s.run(reg, 0);
        CHECK(r.cause == AbortCause::Touch);
        CHECK(r.run.guard.abandoned);
        CHECK(s.plant.distance - s.probe.travelAtLatch <= kOneTickTravel);
        CHECK(s.driveSafe());
    }
    {
        TesterStack s{tank()};
        const double start = s.now();
        s.inputs.onRead = [&](double now) {
            if (now - start >= kTesterCountdownSeconds + 0.3) {
                s.inputs.disabled = true;
            }
        };
        const TesterRunRecord& r = s.run(reg, 0);
        CHECK(r.cause == AbortCause::Disabled);
        CHECK(r.run.guard.abandoned);
        CHECK(s.plant.distance - s.probe.travelAtLatch <= kOneTickTravel);
        CHECK(s.driveSafe());
    }
}

// (20) Bug caught (mutation 6): the second run after an abort inheriting the first one's latch or
// baseline (it would abort at T+0), or the guard carrying the stale abandon into the next run.
TEST_CASE("run (20): after an aborted run, a second run on the SAME objects is clean") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    double start = s.now();
    s.inputs.onRead = [&](double now) {
        if (now - start >= kTesterCountdownSeconds + 0.4 && now - start < 5.0) {
            s.inputs.pad.setPressed(ControllerButton::B, true);
        } else {
            s.inputs.pad.setPressed(ControllerButton::B, false);
        }
    };
    const TesterRunRecord& first = s.run(reg, 0, 1);
    REQUIRE(first.cause == AbortCause::Button);
    REQUIRE(first.run.guard.abandoned);
    s.inputs.onRead = nullptr;
    s.inputs.pad.setPressed(ControllerButton::B, false);
    s.inputs.press();  // the RESULT page was touched meanwhile: press count moved on
    s.inputs.release();
    start = s.now();
    const double before = s.plant.distance;
    const TesterRunRecord& second = s.run(reg, 0, 2);
    CHECK(second.number == 2);
    CHECK(second.cause == AbortCause::None);
    CHECK_FALSE(second.run.guard.abandoned);
    CHECK(std::string_view{testerResultWord(second)} == "FINISHED");
    CHECK(s.plant.distance - before > 39.0);  // RELATIVE: it drove its whole 40 in again
}

// (20b) Bug caught (COMP-t's review, 2026-09-29): an abort that never reaches the guard — the watch
// attached to no guard, or to another — reading as a green FINISHED. The watch still latches B, the
// run drives its whole 40 in through the button, and the record said nothing about the button. The
// boot's chain proof now refuses that wiring (test (13b)); and if it is ever reached anyway, the
// word is ABORT IGNORED, red, and the lines name the cause the guard never acted on.
TEST_CASE("run (20b): an abort the guard never acted on reads ABORT IGNORED, never FINISHED") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    shulib::sequence::RunGuard other{s.plant};  // the watch's link goes to ANOTHER guard
    s.watch.attachGuard(other);
    CHECK_FALSE(shulib::bench::pacerChainProven(s.chassis, s.guard, s.mechanisms, s.watch));  // boot refuses
    const double start = s.now();
    s.inputs.onRead = [&](double now) {
        if (now - start >= kTesterCountdownSeconds + 0.45) {
            s.inputs.pad.setPressed(ControllerButton::B, true);
        }
    };
    const TesterRunRecord& r = s.run(reg, 0);
    CHECK(r.cause == AbortCause::Button);
    CHECK_FALSE(r.run.guard.abandoned);  // the wiring fault: nobody abandoned the run
    CHECK(std::string_view{testerResultWord(r)} == "ABORT IGNORED");
    const TesterRunLines lines = testerRunLines(r, reg.at(0));
    CHECK(std::string_view{lines.line[0]}
          == "run 1: '40in forward' ABORT IGNORED -- ROUTINE_RAN; IGNORED the abort by a controller "
             "button B, end action succeeded");
    shulib::bench::TesterScreen screen{reg};
    shulib::bench::TesterFacts f;
    f.now = 100.0;
    screen.showResult(r, 0, false);
    const shulib::bench::TesterView v = screen.view(f);
    CHECK(v.bar == shulib::ui::Tone::Red);
    CHECK(std::string_view{v.big[0].text} == "ABORT IGNORED");
    CHECK(std::string_view{v.lines[0].text} == "abort IGNORED: button B");
}

// (21) Bug caught: a routine that trips a precondition leaving the tester broken — the throw
// escaping (the task would die), the drive left energized, the watch left armed — or the next run
// failing because of it. The record says PROGRAM ERROR with the sentence; a later run works.
TEST_CASE("run (21): a routine that throws -> PROGRAM ERROR, the drive safe, a later run clean") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    reg.add(routineEntry("throws after 10 in", &driveThenThrow));
    shulib::bench::registerAutons(reg);
    TesterStack s{tank()};
    const TesterRunRecord* r = nullptr;
    CHECK_NOTHROW(r = &s.run(reg, 0));
    REQUIRE(r != nullptr);
    CHECK(r->threw);
    CHECK(says(r->threwWhat, "test: a routine tripped a precondition"));
    CHECK(std::string_view{testerResultWord(*r)} == "PROGRAM ERROR");
    CHECK(s.driveSafe());
    CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
    CHECK_FALSE(s.guard.running());
    CHECK_FALSE(s.watch.armed());
    CHECK(s.plant.distance > 9.0);  // it did drive before it threw
    const double before = s.plant.distance;
    const TesterRunRecord& again = s.run(reg, 1, 2);  // the 40 in, on the same objects
    CHECK(std::string_view{testerResultWord(again)} == "FINISHED");
    CHECK(s.plant.distance - before > 39.0);
}

// (21b) Bug caught (the re-check's finding 6; mutation 29): a throw in the COUNTDOWN escaping
// runTesterOnce (its try narrowed back to runAuton alone) — the watch left armed, the task dead. A
// pacer layer below the watch throws 0.5 s into the countdown: recorded, the watch DISARMED,
// runStarted never called, the plant unmoved, PROGRAM ERROR, ONE line; a later run is clean.
TEST_CASE("run (21b): a throw in the countdown is recorded; nothing moved; a later run is clean") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    s.probe.throwAfter = s.probe.paces + 50;
    const double before = s.plant.distance;
    const TesterRunRecord* r = nullptr;
    CHECK_NOTHROW(r = &s.run(reg, 0));
    REQUIRE(r != nullptr);
    CHECK(r->threw);
    CHECK_FALSE(r->countdownAborted);
    CHECK_FALSE(s.watch.armed());
    CHECK(s.obs.runCalls == 0);
    CHECK(s.plant.distance - before == 0.0);
    CHECK(std::string_view{testerResultWord(*r)} == "PROGRAM ERROR");
    CHECK(s.driveSafe());
    const TesterRunLines lines = testerRunLines(*r, reg.at(0));
    REQUIRE(lines.count == 1);
    CHECK(std::string_view{lines.line[0]}
          == "run 1: '40in forward' PROGRAM ERROR -- it threw (a program error): probe: a pacer "
             "layer threw in the countdown; the drive was stopped");
    const TesterRunRecord& again = s.run(reg, 0, 2);
    CHECK_FALSE(again.threw);
    CHECK(std::string_view{testerResultWord(again)} == "FINISHED");
}

// (21c) Bug caught (mutation 37): the task's last-resort stop skipping a mechanism because an
// earlier one threw, throwing itself (a throw out of a PROS task function is std::terminate), or
// leaving the drive energized or the watch armed.
TEST_CASE("run (21c): stopAfterProgramError safes the drive and every mechanism, never throws") {
    for (const bool withMotion : {false, true}) {
        CAPTURE(withMotion);
        TesterStack s{tank()};
        s.watch.arm();
        shulib::motion::MoveToPose leg{s.chassis.deps(),
                                       Pose2d{Length{30.0}, Length{0.0}, Angle{}},
                                       motion_rig::motionConfig(), 5.0};
        if (withMotion) {
            s.chassis.scheduler().async(leg);
            s.chassis.wait(Time{0.1});  // the leg ticks: the drive is energized
            REQUIRE(s.chassis.scheduler().hasActiveMotion());
        } else {
            s.chassis.drive(shulib::math::ChassisSpeeds{shulib::units::Velocity{20.0},
                                                        shulib::units::Velocity{0.0},
                                                        shulib::units::AngularVelocity{0.0}},
                            shulib::math::Frame::Body);
        }
        REQUIRE_FALSE(s.driveSafe());
        shulib::hal::fake::FakeMechanism first{"first"};
        ThrowingMechanism middle;
        shulib::hal::fake::FakeMechanism last{"last"};
        shulib::hal::IMechanism* present[] = {&first, &middle, &last};
        CHECK_NOTHROW(stopAfterProgramError(
            s.chassis, std::span<shulib::hal::IMechanism* const>{present}, s.watch));
        CHECK(s.driveSafe());
        CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
        CHECK(first.safeStateApplications() == 1);
        CHECK(middle.calls == 1);
        CHECK(last.safeStateApplications() == 1);  // after the thrower: still safed
        CHECK_FALSE(s.watch.armed());
    }
}

// (21d) Bug caught (COMP-t's review, 2026-09-29): a routine that throws a STANDARD exception that is
// not a precondition (a student's std::out_of_range) — reachable: runAuton does not catch and the
// guard rethrows any type after safing. Without its handler the exception escaped runTesterOnce with
// the watch armed and the tester stopped for good (H2); with only catch (...) its words were lost
// ("an unknown exception"). Now: recorded WITH ITS TEXT, PROGRAM ERROR, the drive safe, the watch
// disarmed, and a later run clean. And a throw that is not a std::exception at all (catch (...)).
TEST_CASE("run (21d): a std::exception (its words kept) and a non-standard throw, both recorded") {
    AutonRegistry reg{shulib::bench::kAutonPeriods};
    reg.add(routineEntry("throws std after 10 in", &driveThenThrowStd));
    reg.add(routineEntry("throws int after 10 in", &driveThenThrowInt));
    shulib::bench::registerAutons(reg);
    for (const int index : {0, 1}) {
        CAPTURE(index);
        TesterStack s{tank()};
        const TesterRunRecord* r = nullptr;
        CHECK_NOTHROW(r = &s.run(reg, index));
        REQUIRE(r != nullptr);
        CHECK(r->threw);
        CHECK(std::string_view{r->threwWhat}
              == (index == 0 ? "test: a student's std::out_of_range" : "an unknown exception"));
        CHECK(std::string_view{testerResultWord(*r)} == "PROGRAM ERROR");
        CHECK(s.driveSafe());
        CHECK_FALSE(s.chassis.scheduler().hasActiveMotion());
        CHECK_FALSE(s.guard.running());
        CHECK_FALSE(s.watch.armed());
        CHECK(s.plant.distance > 9.0);  // it drove before it threw
        const TesterRunLines l = testerRunLines(*r, reg.at(static_cast<std::size_t>(index)));
        REQUIRE(l.count == 1);
        CHECK(says(l.line[0], index == 0 ? "it threw (a program error): test: a student's "
                                           "std::out_of_range; the drive was stopped"
                                         : "it threw (a program error): an unknown exception; the "
                                           "drive was stopped"));
        const double before = s.plant.distance;
        const TesterRunRecord& again = s.run(reg, 2, 2);  // the 40 in, on the same objects
        CHECK(std::string_view{testerResultWord(again)} == "FINISHED");
        CHECK(s.plant.distance - before > 39.0);
    }
}

// (21e) Bug caught (COMP-t's review, 2026-09-29): a handler that records a throw but does NOT
// panic-stop the drive (H1/H1b: `chassis.cancel()` deleted from a handler). Two halves, once per
// handler (a precondition, a std::exception, a non-standard throw):
// (a) a pacer layer that leaves every drive motor at 6 V and then throws in the countdown. The drive
//     ends safe -- but through the scheduler's own F2 unwind (WaitUnwindGuard cancels on any throw
//     through a wait), BEFORE the handler runs: this half pins the record and the recovery, and it
//     could NOT see a handler's stop deleted (measured by this chunk's resumed review pass: R26-R28
//     survived it).
// (b) the observer's runStarted() -- called OUTSIDE every wait, between the countdown and the quiet
//     tick -- leaves the drive at 6 V under Coast and throws. Nothing but the handler's own
//     chassis.cancel() stands between that and a live drive; no path in today's code leaves the drive
//     live there, so this half constructs one, to prove the handler's stop is real.
TEST_CASE("run (21e): each handler panic-stops a drive left live by the throw") {
    const AutonRegistry reg = teamRegistry();
    for (const tester_rig::ProbeThrow kind : {tester_rig::ProbeThrow::Precondition,
                                              tester_rig::ProbeThrow::Standard,
                                              tester_rig::ProbeThrow::Unknown}) {
        CAPTURE(static_cast<int>(kind));
        SUBCASE("(a) a pacer layer throws inside the countdown's wait") {
            TesterStack s{tank()};
            s.probe.throwAfter = s.probe.paces + 50;
            s.probe.throwKind = kind;
            s.probe.energizeBeforeThrow = true;
            const TesterRunRecord& r = s.run(reg, 0);
            CHECK(r.threw);
            CHECK(std::string_view{r.threwWhat}
                  == (kind == tester_rig::ProbeThrow::Precondition
                          ? "probe: a pacer layer threw in the countdown"
                          : kind == tester_rig::ProbeThrow::Standard
                                ? "probe: a std::runtime_error in the countdown"
                                : "an unknown exception"));
            CHECK(s.driveSafe());
            CHECK_FALSE(s.watch.armed());
            CHECK(s.obs.runCalls == 0);
            const TesterRunRecord& again = s.run(reg, 0, 2);
            CHECK(std::string_view{testerResultWord(again)} == "FINISHED");
        }
        SUBCASE("(b) the observer throws at runStarted, outside every wait, the drive live") {
            TesterStack s{tank()};
            s.obs.throwAtRun = true;
            s.obs.throwKind = kind;
            const TesterRunRecord& r = s.run(reg, 0);
            CHECK(r.threw);
            CHECK(std::string_view{r.threwWhat}
                  == (kind == tester_rig::ProbeThrow::Precondition
                          ? "probe: the observer threw at runStarted"
                          : kind == tester_rig::ProbeThrow::Standard
                                ? "probe: a std::runtime_error at runStarted"
                                : "an unknown exception"));
            CHECK(s.obs.runCalls == 1);
            CHECK(s.driveSafe());  // the handler's PANIC STOP: every motor 0 V under Brake
            CHECK_FALSE(s.watch.armed());
            CHECK_FALSE(s.guard.running());
            const TesterRunRecord& again = s.run(reg, 0, 2);
            CHECK(std::string_view{testerResultWord(again)} == "FINISHED");
        }
    }
}

// (22) Bug caught (B22; mutation 1): an abort latched BEFORE runAuton — bypassing the countdown's
// last check, which the tester's own flow never does — costs at most ONE commanded tick: the
// watch requests on its latched cause at the run's first pace, and the guard cuts at the next.
// Measured on the plant: 0.60 in at 60 in/s. An edge-only request would be dropped at the guard's
// arm and the whole 40 in would run.
TEST_CASE("run (22): a latch forced before runAuton costs at most one commanded tick") {
    const AutonRegistry reg = teamRegistry();
    TesterStack s{tank()};
    s.watch.arm();
    s.inputs.pad.setPressed(ControllerButton::B, true);
    s.chassis.wait(Time{0.01});
    REQUIRE(s.watch.cause() == AbortCause::Button);
    const double before = s.plant.distance;
    const auto res = shulib::competition::runAuton(s.chassis, s.guard, reg.at(0), nullptr, s.sink,
                                                   {}, reg.periodFor(reg.at(0)));
    s.watch.disarm();
    CHECK(res.guard.abandoned);
    CHECK_FALSE(res.guard.endActionRan);
    CHECK(s.plant.distance - before <= kOneTickTravel);
    CHECK(s.plant.distance - before > 0.0);  // exactly the one tick B22 documents
    CHECK(s.driveSafe());
}

// (22b) Bug caught (COMP-t's review, 2026-09-29: H6, `rec.endLive = true`): a record that claims a
// LIVE end estimate for a run whose estimate never went live — the NOT LIVE words existed only on
// made-up records. A real run through runTesterOnce on a plant whose IMU never comes ready: runAuton
// never starts it (NOT STARTED), the record says NOT LIVE in its fourth line and on RESULT, and the
// drive ends safe.
TEST_CASE("run (22b): an estimate that never goes live -> NOT STARTED, NOT LIVE, the drive safe") {
    struct NeverReadyImu final : shulib::sim::DegradationModel {
        [[nodiscard]] bool imuReady(bool /*ready*/, Time /*now*/) override { return false; }
    };
    const AutonRegistry reg = teamRegistry();
    NeverReadyImu imu;
    TesterStack s{tank(), 1, 1.0, motion_rig::plantConfig(), &imu, 5};
    const double before = s.plant.distance;
    const TesterRunRecord& r = s.run(reg, 0);
    CHECK(r.run.outcome == AutonRunOutcome::NotStarted);
    CHECK(std::string_view{testerResultWord(r)} == "NOT STARTED");
    CHECK_FALSE(r.endLive);
    CHECK(s.plant.distance - before == 0.0);
    CHECK(s.driveSafe());
    const TesterRunLines l = testerRunLines(r, reg.at(0));
    REQUIRE(l.count == 4);
    CHECK_FALSE(l.cut);
    CHECK(says(l.line[2], "the estimate never went live"));
    CHECK(says(l.line[3], "NOT LIVE -- measure the tape"));
    shulib::bench::TesterScreen screen{reg};
    shulib::bench::TesterFacts f;
    f.now = 100.0;
    screen.showResult(r, 0, false);
    const shulib::bench::TesterView v = screen.view(f);
    bool notLive = false;
    for (int i = 0; i < v.lineCount; ++i) {
        notLive = notLive || says(v.lines[i].text, " NOT LIVE");
    }
    CHECK(notLive);
}

// (23) Bug caught: the RESULT word and the log lines disagreeing with the record — a word that
// reads FINISHED for a cut, a stop or a refusal; a line that is cut, not ASCII, or over TermSink's
// 200 bytes at the worst case; an end action time printed for an end action that never ran; a
// ROUTE auton not refused by name (the tester passes no command registry: finding 5).
TEST_CASE("run (23): testerResultWord and testerRunLines for every record shape") {
    const AutonRegistry reg = teamRegistry();
    const AutonEntry& forty = reg.at(0);
    const auto allAscii = [](const TesterRunLines& l) {
        for (int i = 0; i < l.count; ++i) {
            for (const char* p = l.line[i]; *p != '\0'; ++p) {
                if (static_cast<unsigned char>(*p) < 0x20 || static_cast<unsigned char>(*p) > 0x7E) {
                    return false;
                }
            }
        }
        return true;
    };

    // The words, first match wins, each at most one LARGE line.
    TesterRunRecord w{};
    w.entry = 0;
    w.run.outcome = AutonRunOutcome::RoutineRan;
    w.run.routine.ok = true;
    w.run.guard.endActionSucceeded = true;
    CHECK(std::string_view{testerResultWord(w)} == "FINISHED");
    w.run.guard.endActionSucceeded = false;
    CHECK(std::string_view{testerResultWord(w)} == "ROUTINE STOPPED");
    w.run.guard.endActionSucceeded = true;
    w.run.routine.ok = false;
    CHECK(std::string_view{testerResultWord(w)} == "ROUTINE STOPPED");
    w.run.guard.scoringCut = true;
    CHECK(std::string_view{testerResultWord(w)} == "CUT BY DEADLINE");
    w.run.guard.floorFired = true;
    CHECK(std::string_view{testerResultWord(w)} == "HARD STOP");
    w.run.outcome = AutonRunOutcome::NotStarted;
    CHECK(std::string_view{testerResultWord(w)} == "NOT STARTED");
    w.cause = AbortCause::Button;  // latched, and the guard never abandoned: the review's word
    CHECK(std::string_view{testerResultWord(w)} == "ABORT IGNORED");
    w.run.guard.abandoned = true;
    CHECK(std::string_view{testerResultWord(w)} == "ABORTED");
    w.run.outcome = AutonRunOutcome::Refused;
    CHECK(std::string_view{testerResultWord(w)} == "REFUSED");
    w.threw = true;
    CHECK(std::string_view{testerResultWord(w)} == "PROGRAM ERROR");
    w.countdownAborted = true;
    CHECK(std::string_view{testerResultWord(w)} == "ABORTED");
    for (const char* word : {"FINISHED", "ROUTINE STOPPED", "CUT BY DEADLINE", "HARD STOP",
                             "NOT STARTED", "ABORTED", "ABORT IGNORED", "REFUSED",
                             "PROGRAM ERROR"}) {
        CHECK(std::strlen(word) <= shulib::ui::kLargeCols);
    }
    // A ROUTE's verdict is its RunReport's, never the (unused) routine's (H3: the branch ignored).
    TesterRunRecord byRoute{};
    byRoute.entry = 0;
    byRoute.run.outcome = AutonRunOutcome::RouteRan;
    byRoute.run.guard.endActionSucceeded = true;
    byRoute.run.routine.ok = true;  // stale: a route run never fills it
    byRoute.run.route.reason = shulib::runner::RunStopCause::MotionFailed;
    CHECK(std::string_view{testerResultWord(byRoute)} == "ROUTINE STOPPED");
    byRoute.run.routine.ok = false;
    byRoute.run.route.reason = shulib::runner::RunStopCause::None;
    CHECK(std::string_view{testerResultWord(byRoute)} == "FINISHED");
    byRoute.run.guard.endActionSucceeded = false;
    CHECK(std::string_view{testerResultWord(byRoute)} == "ROUTINE STOPPED");

    // A CLEAN run's lines, from a real run.
    {
        TesterStack s{tank()};
        const TesterRunRecord& r = s.run(reg, 0);
        const TesterRunLines l = testerRunLines(r, forty);
        REQUIRE(l.count == 4);
        CHECK_FALSE(l.cut);
        CHECK(allAscii(l));
        CHECK(std::string_view{l.line[0]}
              == "run 1: '40in forward' FINISHED -- ROUTINE_RAN; in time, end action succeeded");
        CHECK(std::string_view{l.line[1]}
              == "run 1: scoring ended T+1.80s, end action ended T+1.92s, 192 paces, 0 cancels "
                 "after the cut; hand-off 0.020s after the countdown, live after 0.00s");
        CHECK(std::string_view{l.line[2]}
              == "run 1: routine OK, 1 of 1 steps done; faults this run: none");
        CHECK(std::string_view{l.line[3]}
              == "run 1: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate "
                 "(39.77, 0.00) facing 0.0 deg LIVE -- measure the tape");
    }
    // The weak-side honest stop (x0.65), from a real run — the re-prediction's golden.
    {
        TesterStack s{tank(), 1, 0.65};
        const TesterRunRecord& r = s.run(reg, 0);
        const TesterRunLines l = testerRunLines(r, forty);
        REQUIRE(l.count == 4);
        CHECK_FALSE(l.cut);
        CHECK(std::string_view{l.line[0]}
              == "run 1: '40in forward' ROUTINE STOPPED -- ROUTINE_RAN; in time, end action "
                 "succeeded");
        // The WHOLE of each line (the review: its start alone would let the brake's end time or
        // the pace count move without a red) — the brief's re-predicted golden, word for word.
        CHECK(std::string_view{l.line[1]}
              == "run 1: scoring ended T+5.01s, end action ended T+5.13s, 513 paces, 0 cancels "
                 "after the cut; hand-off 0.020s after the countdown, live after 0.00s");
        CHECK(std::string_view{l.line[2]}
              == "run 1: routine STOPPED, 0 of 1 steps done, stopped at moveTo (motion TIMEOUT); "
                 "faults this run: 1, last MOTION_TIMEOUT -- see the [ERROR] line");
        CHECK(std::string_view{l.line[3]}
              == "run 1: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate "
                 "(39.38, -0.93) facing 0.6 deg LIVE -- measure the tape");
    }
    // A ROUTE entry RUNS (MECH-A5; it was REFUSED by name until then — the tester passed no command
    // registry, the COMP brief's B30): the tester hands runAuton the program's registry, which holds
    // robot two's derived mechanism route ids, so a route's markers fire them — here the claw opened
    // and pivot 6 run at the turn's waypoint; an id the robot does not have is an unknown id, counted.
    {
        static shulib::config::Route route{};  // a turn, no Start: RELATIVE is its start
        std::snprintf(route.name, sizeof route.name, "%s", "r1");
        route.waypoints[0].motion = shulib::config::WaypointMotion::TurnTo;
        route.waypoints[0].heading = Angle::degrees(90.0);
        std::snprintf(route.waypoints[0].markers[0].id, sizeof route.waypoints[0].markers[0].id, "%s", "claw_open");
        std::snprintf(route.waypoints[0].markers[1].id, sizeof route.waypoints[0].markers[1].id, "%s",
                      "claw_pivot_6_run");
        route.waypoints[0].markers[1].args[0] = 12;
        route.waypoints[0].markers[1].args[1] = 300;
        route.waypoints[0].markers[1].argCount = 2;
        route.waypoints[0].markerCount = 2;
        route.count = 1;
        REQUIRE(route.valid());
        AutonRegistry withRoute{shulib::bench::kAutonPeriods};
        AutonEntry e{};
        e.name = "a route auton";
        e.kind = AutonKind::Match;
        e.route = &route;
        e.start = StartPose::relative();
        e.endActionAt = Time{27.0};
        e.hardStopAt = Time{29.0};
        withRoute.add(e);
        TesterStack s{tank()};
        REQUIRE(s.commands.count() == 6U);  // robot two's six, registered as the program registers them
        bool pivotRan = false;
        s.inputs.onRead = [&s, &pivotRan](double) {
            pivotRan = pivotRan || s.two.m6.commandedVoltage().value() == 12.0;
        };
        const TesterRunRecord& r = s.run(withRoute, 0);
        CHECK(r.run.outcome == AutonRunOutcome::RouteRan);
        CHECK(r.run.route.ok());
        CHECK(r.run.route.markersFired == 2);
        CHECK(r.run.route.unknownIds == 0);  // M47: a program that forgot registerInto is 2 unknown ids
        CHECK(pivotRan);
        CHECK(s.two.lineA.setCount() >= 2);  // the claw was opened by its marker ...
        CHECK_FALSE(s.two.lineA.commanded());  // ... and the guard's end of run made it safe: closed
        CHECK(s.mechanismsSafe());
        CHECK(std::string_view{testerResultWord(r)} == "FINISHED");
    }

    // THE WORST CASE: a 29-character name, run 99999, extreme numbers, every button, the longest
    // words — every shape's every line under TermSink's cut, never cut, ASCII.
    AutonEntry longName = forty;
    longName.name = "abcdefghijklmnopqrstuvwxyz012";
    REQUIRE(std::strlen(longName.name) == 29);
    TesterRunRecord x{};
    x.number = 99999;
    x.entry = 0;
    x.buttons = 0x0FFF;
    x.cause = AbortCause::ControllerLost;
    x.run.outcome = AutonRunOutcome::NotStarted;
    x.run.guard.abandoned = true;
    x.run.guard.floorFired = true;
    x.run.guard.endActionRan = true;
    x.run.guard.scoringEnded = Time{-99999.99};
    x.run.guard.endActionEnded = Time{-99999.99};
    x.run.guard.pacesSeen = 99999999;
    x.run.guard.postExpiryCancels = 99999;
    x.run.armedAt = Time{-99999.99};
    x.countdownStartedAt = 99999.99;
    x.run.liveAfter = Time{-99999.99};
    x.run.seated = Pose2d{Length{-99999.99}, Length{-99999.99}, Angle::degrees(-179.9)};
    x.run.imuAtSeat = Angle::degrees(-179.9);
    x.endEstimate = Pose2d{Length{-99999.99}, Length{-99999.99}, Angle::degrees(-179.9)};
    x.faultsBefore = 0;
    x.faultsAfter = 99999;
    x.lastFault = FaultCode::MotorGroupDisagree;
    std::memset(x.run.why, 'w', sizeof x.run.why - 1);
    std::memset(x.threwWhat, 't', sizeof x.threwWhat - 1);
    const auto check = [&](const TesterRunRecord& rec, int expectLines) {
        const TesterRunLines l = testerRunLines(rec, longName);
        CHECK(l.count == expectLines);
        CHECK_FALSE(l.cut);
        CHECK(allAscii(l));
        for (int i = 0; i < l.count; ++i) {
            CHECK(std::strlen(l.line[i]) < kTesterLineBytes);
        }
    };
    check(x, 4);  // not started, abandoned by every button, the floor, faults
    x.run.outcome = AutonRunOutcome::RoutineRan;
    x.run.routine.ok = false;
    x.run.routine.stoppedAt = 999;
    x.run.routine.completed = 998;
    x.run.routine.steps = 999;
    x.run.routine.stoppedName = "a-step-name-that-is-long-too";
    x.run.routine.cause = shulib::chassis::RoutineStopCause::MechanismFailed;
    check(x, 4);
    // MECH-A4: the LONGEST stop word since API 2.5 ("mechanism step FAILED") fits the same lines.
    x.run.routine.cause = shulib::chassis::RoutineStopCause::MechanismStepFailed;
    check(x, 4);
    CHECK(std::string_view{shulib::bench::routineStopWord(shulib::chassis::RoutineStopCause::MechanismStepFailed,
                                                          shulib::control::ExitReason::Settled)} ==
          "mechanism step FAILED");
    x.run.routine.cause = shulib::chassis::RoutineStopCause::MechanismFailed;
    x.cause = AbortCause::Button;
    check(x, 4);
    x.run.guard.abandoned = false;  // ABORT IGNORED by every button: the longest prefix
    CHECK(std::string_view{testerResultWord(x)} == "ABORT IGNORED");
    check(x, 4);
    x.run.guard.abandoned = true;
    x.run.outcome = AutonRunOutcome::Refused;
    check(x, 4);
    x.faultsAfter = 0;
    check(x, 4);
    x.threw = true;
    check(x, 1);
    x.threw = false;
    x.countdownAborted = true;
    x.cause = AbortCause::ControllerLost;
    check(x, 1);
    x.cause = AbortCause::Button;
    check(x, 1);

    // THE GOLDENS for the shapes a real run on the plant does not produce here: an abort by two
    // buttons mid-run, the deadline cut, the hard floor, a run that never started, a throw inside
    // the run. Every word of every line, exactly.
    const auto base = [](int number) {
        TesterRunRecord r{};
        r.number = number;
        r.entry = 0;
        r.run.outcome = AutonRunOutcome::RoutineRan;
        r.countdownStartedAt = 10.0;
        r.run.armedAt = Time{13.02};
        r.endEstimate = Pose2d{Length{24.1}, Length{0.0}, Angle::degrees(0.0)};
        r.endLive = true;
        return r;
    };
    const auto lines = [&](const TesterRunRecord& r) {
        const TesterRunLines l = testerRunLines(r, forty);
        CHECK_FALSE(l.cut);
        std::string all;
        for (int i = 0; i < l.count; ++i) {
            all += l.line[i];
            all += '\n';
        }
        return all;
    };
    {
        TesterRunRecord r = base(2);  // two buttons mid-run
        r.cause = AbortCause::Button;
        r.buttons = (1u << 9) | (1u << 0);
        r.run.guard.abandoned = true;
        r.run.guard.scoringEnded = Time{0.45};
        r.run.guard.pacesSeen = 45;
        r.run.guard.postExpiryCancels = 1;
        r.run.routine.ok = false;
        r.run.routine.steps = 1;
        r.run.routine.stoppedAt = 1;
        r.run.routine.stoppedName = "moveTo";
        r.run.routine.cause = shulib::chassis::RoutineStopCause::MotionFailed;
        r.run.routine.exit = shulib::control::ExitReason::Cancelled;
        CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
        CHECK(lines(r)
              == "run 2: '40in forward' ABORTED -- ROUTINE_RAN; ABANDONED by a controller button "
                 "L1+B, end action skipped\n"
                 "run 2: scoring ended T+0.45s, end action ended T+--, 45 paces, 1 cancels after "
                 "the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 2: routine STOPPED, 0 of 1 steps done, stopped at moveTo (motion CANCELLED); "
                 "faults this run: none\n"
                 "run 2: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (24.10, "
                 "0.00) facing 0.0 deg LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(3);  // the deadline cut, the end action run
        r.run.guard.scoringCut = true;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = true;
        r.run.guard.scoringEnded = Time{27.0};
        r.run.guard.endActionEnded = Time{27.4};
        r.run.guard.pacesSeen = 2740;
        r.run.routine.ok = false;
        r.run.routine.steps = 2;
        r.run.routine.completed = 1;
        r.run.routine.stoppedAt = 2;
        r.run.routine.stoppedName = "moveTo";
        r.run.routine.cause = shulib::chassis::RoutineStopCause::MotionFailed;
        r.run.routine.exit = shulib::control::ExitReason::Cancelled;
        CHECK(std::string_view{testerResultWord(r)} == "CUT BY DEADLINE");
        CHECK(lines(r)
              == "run 3: '40in forward' CUT BY DEADLINE -- ROUTINE_RAN; cut at the deadline, end "
                 "action succeeded\n"
                 "run 3: scoring ended T+27.00s, end action ended T+27.40s, 2740 paces, 0 cancels "
                 "after the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 3: routine STOPPED, 1 of 2 steps done, stopped at moveTo (motion CANCELLED); "
                 "faults this run: none\n"
                 "run 3: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (24.10, "
                 "0.00) facing 0.0 deg LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(4);  // the hard floor: the end action cut too
        r.run.guard.scoringCut = true;
        r.run.guard.floorFired = true;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = false;
        r.run.guard.scoringEnded = Time{27.0};
        r.run.guard.endActionEnded = Time{29.0};
        r.run.guard.pacesSeen = 2900;
        r.run.guard.postExpiryCancels = 3;
        r.faultsBefore = 2;
        r.faultsAfter = 3;
        r.lastFault = FaultCode::MotionTimeout;
        r.run.routine.ok = false;
        r.run.routine.steps = 1;
        r.run.routine.stoppedAt = 1;
        r.run.routine.stoppedName = "moveTo";
        r.run.routine.cause = shulib::chassis::RoutineStopCause::MotionFailed;
        r.run.routine.exit = shulib::control::ExitReason::TimedOut;
        CHECK(std::string_view{testerResultWord(r)} == "HARD STOP");
        CHECK(lines(r)
              == "run 4: '40in forward' HARD STOP -- ROUTINE_RAN; cut at the deadline, end action "
                 "FAILED, HARD STOP fired\n"
                 "run 4: scoring ended T+27.00s, end action ended T+29.00s, 2900 paces, 3 cancels "
                 "after the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 4: routine STOPPED, 0 of 1 steps done, stopped at moveTo (motion TIMEOUT); "
                 "faults this run: 1, last MOTION_TIMEOUT -- see the [ERROR] line\n"
                 "run 4: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (24.10, "
                 "0.00) facing 0.0 deg LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(5);  // never started: the estimate never went live
        r.run.outcome = AutonRunOutcome::NotStarted;
        std::snprintf(r.run.why, sizeof r.run.why, "%s",
                      "auton '40in forward' never started: the estimate was not live by 27.00 s");
        r.run.guard.scoringCut = true;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = true;
        r.run.guard.scoringEnded = Time{27.0};
        r.run.guard.endActionEnded = Time{27.01};
        r.run.guard.pacesSeen = 2701;
        r.run.liveAfter = Time{0.0};
        r.endLive = false;
        r.endEstimate = Pose2d{};
        CHECK(std::string_view{testerResultWord(r)} == "NOT STARTED");
        CHECK(lines(r)
              == "run 5: '40in forward' NOT STARTED -- NOT_STARTED; cut at the deadline, end action "
                 "succeeded\n"
                 "run 5: scoring ended T+27.00s, end action ended T+27.01s, 2701 paces, 0 cancels "
                 "after the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 5: auton '40in forward' never started: the estimate was not live by 27.00 s; "
                 "faults this run: none\n"
                 "run 5: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (0.00, "
                 "0.00) facing 0.0 deg NOT LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(7);  // an abort DURING the end action: the guard cut it
        r.cause = AbortCause::Button;
        r.buttons = 1u << 9;
        r.run.guard.abandoned = true;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = false;  // void, per the guard's report
        r.run.guard.scoringEnded = Time{1.80};
        r.run.guard.endActionEnded = Time{1.82};
        r.run.guard.pacesSeen = 182;
        r.run.guard.postExpiryCancels = 1;
        r.run.routine.ok = true;
        r.run.routine.steps = 1;
        r.run.routine.completed = 1;
        CHECK(std::string_view{testerResultWord(r)} == "ABORTED");
        CHECK(lines(r)
              == "run 7: '40in forward' ABORTED -- ROUTINE_RAN; ABANDONED by a controller button "
                 "B, end action CUT by the abort\n"
                 "run 7: scoring ended T+1.80s, end action ended T+1.82s, 182 paces, 1 cancels after "
                 "the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 7: routine OK, 1 of 1 steps done; faults this run: none\n"
                 "run 7: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (24.10, "
                 "0.00) facing 0.0 deg LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(8);  // an abort latched and NEVER acted on (a wiring fault)
        r.cause = AbortCause::Touch;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = true;
        r.run.guard.scoringEnded = Time{1.80};
        r.run.guard.endActionEnded = Time{1.92};
        r.run.guard.pacesSeen = 192;
        r.run.routine.ok = true;
        r.run.routine.steps = 1;
        r.run.routine.completed = 1;
        CHECK(std::string_view{testerResultWord(r)} == "ABORT IGNORED");
        CHECK(lines(r)
              == "run 8: '40in forward' ABORT IGNORED -- ROUTINE_RAN; IGNORED the abort by a touch "
                 "of the screen, end action succeeded\n"
                 "run 8: scoring ended T+1.80s, end action ended T+1.92s, 192 paces, 0 cancels after "
                 "the cut; hand-off 0.020s after the countdown, live after 0.00s\n"
                 "run 8: routine OK, 1 of 1 steps done; faults this run: none\n"
                 "run 8: seated (0.00, 0.00) facing 0.0 deg (the IMU read 0.0); END estimate (24.10, "
                 "0.00) facing 0.0 deg LIVE -- measure the tape\n");
    }
    {
        TesterRunRecord r = base(6);  // a throw inside the run
        r.threw = true;
        std::snprintf(r.threwWhat, sizeof r.threwWhat, "%s", "test: a routine tripped a precondition");
        CHECK(std::string_view{testerResultWord(r)} == "PROGRAM ERROR");
        CHECK(lines(r)
              == "run 6: '40in forward' PROGRAM ERROR -- it threw (a program error): test: a routine "
                 "tripped a precondition; the drive was stopped\n");
    }

    // "--" for an end action that did not run.
    TesterRunRecord skipped{};
    skipped.entry = 0;
    skipped.number = 3;
    skipped.run.outcome = AutonRunOutcome::RoutineRan;
    skipped.run.guard.abandoned = true;
    skipped.cause = AbortCause::Touch;
    const TesterRunLines sl = testerRunLines(skipped, forty);
    CHECK(says(sl.line[0], "ABANDONED by a touch of the screen, end action skipped"));
    CHECK(says(sl.line[1], "end action ended T+--,"));
}

// (23b) Bug caught (COMP-t's review, 2026-09-29): the boot line cut at a real build hash. The brief's
// flow printed ONE line with the beacon unbounded and the hash at its full length; this repository's
// own `git describe` ("v0.1.1-354-gee2e399-dirty") made it 206 bytes, and TermSink's 199-byte cut
// took the SD path the worksheet's M.2 and M.7 read ("…/usd/shulib_auton_r"). The suite never saw it
// (its builds carry a short hash). Now THREE lines, every field bounded: each fits at the worst case
// of every field (a 47-byte hash — diag::kMaxHashBytes — or longer, the longest beacon and boot
// word, the most negative ints, a 60-character path), and the real values read as the worksheet says.
TEST_CASE("run (23b): the boot's three lines fit at every field's worst case, and read right") {
    using shulib::bench::TesterBootLines;
    using shulib::bench::TesterSdLog;
    using shulib::bench::testerBootLines;
    const std::string longHash(80, 'h');  // longer than any describe: cut at kMaxHashBytes
    // Longer than a whole line, so a field left unbounded (`%s`) cannot fit and says so (`cut`) —
    // a 64-byte beacon fitted even unbounded, and the resumed review pass's R33 (`%.40s` -> `%s`)
    // survived it.
    const std::string longBeacon(300, 'b');
    const std::string longBootWord(300, 'w');
    const std::string longPath(300, 'p');
    for (const TesterSdLog sd : {TesterSdLog::Off, TesterSdLog::Created, TesterSdLog::Appended}) {
        const TesterBootLines b =
            testerBootLines(longBeacon.c_str(), longHash, longBootWord.c_str(), -2147483647 - 1,
                            -2147483647 - 1, -2147483647 - 1, sd, longPath.c_str());
        CHECK_FALSE(b.cut);
        for (const auto& line : b.line) {
            CHECK(std::strlen(line) < kTesterLineBytes);
        }
        CHECK(std::string_view{b.line[0]}.find(std::string(shulib::diag::kMaxHashBytes, 'h'))
              != std::string_view::npos);
        CHECK(std::string_view{b.line[0]}.find(std::string(shulib::diag::kMaxHashBytes + 1, 'h'))
              == std::string_view::npos);
        CHECK(std::string_view{b.line[0]}.find(std::string(40, 'b')) != std::string_view::npos);
        CHECK(std::string_view{b.line[0]}.find(std::string(41, 'b')) == std::string_view::npos);
    }
    // The real boot, at this repository's own describe: the worksheet's words, the path whole.
    const TesterBootLines real =
        testerBootLines("shulib-robot-variant=tank-autontest", "v0.1.1-354-gee2e399-dirty", "RECORD",
                        2, 3, 3, TesterSdLog::Appended, "/usd/shulib_auton_runs.txt");
    CHECK_FALSE(real.cut);
    CHECK(std::string_view{real.line[0]}
          == "boot: shulib AUTON TESTER [shulib-robot-variant=tank-autontest] hash "
             "v0.1.1-354-gee2e399-dirty");
    CHECK(std::string_view{real.line[1]}
          == "boot source RECORD, 2 autons, 3 of 3 mechanisms present");
    CHECK(std::string_view{real.line[2]} == "SD run log ON, APPENDED to /usd/shulib_auton_runs.txt");
    const TesterBootLines created =
        testerBootLines("b", "", "RECORD", 2, 3, 3, TesterSdLog::Created,
                        "/usd/shulib_auton_runs.txt");
    CHECK(std::string_view{created.line[0]} == "boot: shulib AUTON TESTER [b] hash MISSING");
    CHECK(std::string_view{created.line[2]} == "SD run log ON, CREATED /usd/shulib_auton_runs.txt");
    const TesterBootLines off =
        testerBootLines("b", "abc", "RECORD", 2, 3, 3, TesterSdLog::Off, "/usd/never-printed");
    CHECK(std::string_view{off.line[2]} == "SD run log OFF (no card, or the log would not open)");
}

// (23c) STEP1-A (D5 (a)) Bug caught: a routine that CARRIED ON past failed steps to its end reading
// "ROUTINE STOPPED" (it did not stop) or "FINISHED" (it failed steps); the DONE word red (an unknown
// word is red); a DONE word too wide for the driver page's 15-character slot or the large line; a
// carried-on run's log naming no failed step (SM21); a pending skip read as DONE; a default run's
// lines changing count. SM11 (the word reading `ok` alone) reads FINISHED/ROUTINE STOPPED here.
TEST_CASE("run (23c) STEP1-A: DONE, n FAILED -- the word, its amber, and the routine's two lines") {
    const AutonRegistry reg = teamRegistry();
    const AutonEntry& forty = reg.at(0);
    const auto base = [] {
        TesterRunRecord r{};
        r.number = 7;
        r.entry = 0;
        r.run.outcome = AutonRunOutcome::RoutineRan;
        r.run.guard.endActionRan = true;
        r.run.guard.endActionSucceeded = true;
        r.run.routine.ok = false;
        r.run.routine.steps = 7;
        r.run.routine.completed = 5;
        r.run.routine.failedOn = 2;
        r.run.routine.firstFailedAt = 2;
        r.run.routine.firstFailedName = "miss";
        r.run.routine.firstFailedCause = shulib::chassis::RoutineStopCause::ActionFailed;
        return r;
    };
    // THE WORDS.
    TesterRunRecord w = base();
    CHECK(std::string_view{testerResultWord(w)} == "DONE, 2 FAILED");
    w.run.routine.failedOn = 1;
    CHECK(std::string_view{testerResultWord(w)} == "DONE, 1 FAILED");
    w.run.routine.failedOn = 9;
    CHECK(std::string_view{testerResultWord(w)} == "DONE, 9 FAILED");
    w.run.routine.failedOn = 10;
    CHECK(std::string_view{testerResultWord(w)} == "DONE, 9+ FAILED");
    w.run.routine.failedOn = 2147483647;
    CHECK(std::string_view{testerResultWord(w)} == "DONE, 9+ FAILED");
    w.run.routine.failedOn = 2;
    w.run.guard.endActionSucceeded = false;  // the park failed: today's word, unchanged (the brief's L12)
    CHECK(std::string_view{testerResultWord(w)} == "ROUTINE STOPPED");
    w.run.guard.endActionSucceeded = true;
    w.run.routine.skipPendingTo = "B";  // a skip that never landed: nothing ran after it
    CHECK(std::string_view{testerResultWord(w)} == "ROUTINE STOPPED");
    w.run.routine.skipPendingTo = "";
    w.run.routine.stoppedAt = 6;  // went on, then a later step stopped the chain
    CHECK(std::string_view{testerResultWord(w)} == "ROUTINE STOPPED");
    w.run.routine.stoppedAt = 0;
    w.run.guard.scoringCut = true;  // the deadline's word still comes first
    CHECK(std::string_view{testerResultWord(w)} == "CUT BY DEADLINE");
    w.run.guard.scoringCut = false;
    w.run.routine.failedOn = 0;
    w.run.routine.ok = true;
    CHECK(std::string_view{testerResultWord(w)} == "FINISHED");
    for (int n = -1; n <= 12; ++n) {
        CAPTURE(n);
        const std::string_view d{shulib::bench::testerDoneWord(n)};
        CHECK(d.size() <= 15);  // the driver page's "auton %.15s" slot (competition_screen.hpp)
        CHECK(d.size() <= shulib::ui::kLargeCols);
        // review RM9/RM10: the exact word for EVERY n (R25 pins the competition program's twin to the
        // same rule, so the two tables cannot drift): below 1 reads as 1; 9 is the last single figure.
        const std::string want = n > 9 ? "DONE, 9+ FAILED" : "DONE, " + std::to_string(n < 1 ? 1 : n) + " FAILED";
        CHECK(d == want);
    }
    // ITS COLOUR, on the RESULT page: amber (an unknown word would be red) -- for EVERY count, so the
    // panel's rule is the prefix, not one word (review RM11).
    for (int n = 1; n <= 12; ++n) {
        CAPTURE(n);
        shulib::bench::TesterScreen screen{reg};
        shulib::bench::TesterFacts f;
        f.now = 100.0;
        TesterRunRecord done = base();
        done.run.routine.failedOn = n;
        screen.showResult(done, 0, false);
        const shulib::bench::TesterView v = screen.view(f);
        CHECK(v.bar == shulib::ui::Tone::Amber);
        CHECK(std::string_view{v.big[0].text} == std::string_view{shulib::bench::testerDoneWord(n)});
    }
    // THE LINES. Went on to the end: line 3 the summary, line 3b the first failure; five lines.
    const auto all = [&](const TesterRunRecord& r, int expect) {
        const TesterRunLines l = testerRunLines(r, forty);
        CHECK(l.count == expect);
        CHECK_FALSE(l.cut);
        return l;
    };
    {
        const TesterRunLines l = all(base(), 5);
        CHECK(std::string_view{l.line[0]}.find("run 7: '40in forward' DONE, 2 FAILED -- ROUTINE_RAN; in time, end "
                                               "action succeeded") == 0);
        CHECK(std::string_view{l.line[2]} == "run 7: routine DONE, 5 of 7 steps done, 2 failed, 0 jumped; faults "
                                             "this run: none");
        CHECK(std::string_view{l.line[3]} == "run 7: first failed: step 2 miss (action FAILED); the routine went on");
        CHECK(std::string_view{l.line[4]}.find("run 7: seated (") == 0);
    }
    {
        TesterRunRecord r = base();  // a skip that never landed
        r.run.routine.failedOn = 1;
        r.run.routine.jumped = 3;
        r.run.routine.skipPendingTo = "third piece";
        const TesterRunLines l = all(r, 5);
        CHECK(std::string_view{l.line[2]} == "run 7: routine STOPPED, 5 of 7 steps done, 1 failed, 3 jumped; faults "
                                             "this run: none");
        CHECK(std::string_view{l.line[3]} == "run 7: first failed: step 2 miss (action FAILED); a skip to 'third "
                                             "piece' never landed");
    }
    {
        TesterRunRecord r = base();  // went on, then stopped: line 3 is today's STOPPED form
        r.run.routine.stoppedAt = 6;
        r.run.routine.skipped = 1;
        r.run.routine.stoppedName = "driveTo";
        r.run.routine.cause = shulib::chassis::RoutineStopCause::MechanismNotStarted;
        const TesterRunLines l = all(r, 5);
        CHECK(std::string_view{l.line[2]} == "run 7: routine STOPPED, 5 of 7 steps done, stopped at driveTo "
                                             "(mechanism NOT STARTED); faults this run: none");
        CHECK(std::string_view{l.line[3]} == "run 7: first failed: step 2 miss (action FAILED); the routine went on, "
                                             "then stopped");
    }
    {
        // A carried-on MOTION failure (a drive that timed out): line 3b says "motion failed" -- the
        // result keeps no exit for its first failed step (the six fields are the brief's; the step's own
        // RTN Warn says "motion TIMEOUT"). Pinned so a change to it is a decision, not a drift (review).
        TesterRunRecord r = base();
        r.run.routine.firstFailedName = "driveTo";
        r.run.routine.firstFailedCause = shulib::chassis::RoutineStopCause::MotionFailed;
        const TesterRunLines l = all(r, 5);
        CHECK(std::string_view{l.line[3]} == "run 7: first failed: step 2 driveTo (motion failed); the routine went "
                                             "on");
    }
    {
        TesterRunRecord r = base();  // the default rule: four lines, as ever
        r.run.routine.failedOn = 0;
        r.run.routine.firstFailedAt = 0;
        r.run.routine.ok = true;
        r.run.routine.completed = 7;
        all(r, 4);
    }
    // A REAL carried-on chain on the host test plant gives the record its result.
    {
        golden::Tank t;
        Routine r{t.ch, "real"};
        r.onFail(shulib::chassis::carryOn).then([] { return false; }, "miss").then([] { return true; }, "next");
        TesterRunRecord rec = base();
        rec.run.routine = r.result();
        CHECK(std::string_view{testerResultWord(rec)} == "DONE, 1 FAILED");
        const TesterRunLines l = all(rec, 5);
        CHECK(std::string_view{l.line[2]} == "run 7: routine DONE, 1 of 2 steps done, 1 failed, 0 jumped; faults "
                                             "this run: none");
        CHECK(std::string_view{l.line[3]} == "run 7: first failed: step 1 miss (action FAILED); the routine went on");
    }
    // THE WORST CASE of each new line: run 99999, 4-digit counts, a name past 24 characters, the
    // longest cause word (21), the faults tail at its cap — then every count at its int extreme.
    TesterRunRecord x = base();
    x.number = 99999;
    x.faultsBefore = 0;
    x.faultsAfter = 2147483647;
    x.lastFault = FaultCode::MotorGroupDisagree;
    x.run.routine.firstFailedName = "a-step-name-that-is-longer-than-24";
    x.run.routine.firstFailedCause = shulib::chassis::RoutineStopCause::MechanismNotStarted;
    x.run.routine.skipPendingTo = "a-label-that-is-longer-than-24";
    for (const int n : {9999, 2147483647, -2147483647 - 1}) {
        CAPTURE(n);
        x.run.routine.steps = n;
        x.run.routine.completed = n;
        // failedOn stays > 0 (a negative count is never produced, and it is what writes line 3b);
        // every other count takes the widest %d there is.
        x.run.routine.failedOn = n < 0 ? 2147483647 : n;
        x.run.routine.jumped = n;
        x.run.routine.firstFailedAt = n;
        for (const bool pending : {false, true}) {
            x.run.routine.skipPendingTo = pending ? "a-label-that-is-longer-than-24" : "";
            const TesterRunLines l = all(x, 5);
            for (int i = 0; i < l.count; ++i) {
                CAPTURE(l.line[i]);
                CHECK(std::strlen(l.line[i]) <= kTesterLineBytes - 1);
            }
        }
    }
    CHECK(std::string_view{shulib::bench::routineStopWord(shulib::chassis::RoutineStopCause::MechanismNotStarted,
                                                          shulib::control::ExitReason::Running)} ==
          "mechanism NOT STARTED");
}
