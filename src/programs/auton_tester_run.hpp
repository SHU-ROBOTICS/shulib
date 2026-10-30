#pragma once
//
// auton_tester_run.hpp -- ONE run of "shulib Auton Tester", exactly as the program performs it
// (COMP-t, 2026-09-28): arm the abort watch, the 3 s countdown (one-tick waits: nothing is
// commanded), ONE quiet tick, then competition::runAuton -- the competition program's call -- and
// the record of what happened. Plus the task's last-resort stop, the words the RESULT page and the
// log lines say, and the boot's log lines. The boot's chain proof and the generic half of the
// last-resort stop moved to src/common/guarded_run.hpp at COMP-b's first step (2026-09-30, the
// COMP brief's ND3 (b)), because the competition program needs both with its own watch; the
// tester calls them from there, unchanged, and keeps the thin wrapper below that also disarms
// ITS watch.
//
// WHY PROS-FREE: this is the sequence that decides whether the robot moves. The program's TU cannot
// be compiled on the host (pros/screen.h), so the sequence lives here, where the host suite runs it
// on the A2 plant with the REAL AbortWatch between the plant's pacer and the guard
// (test/auton_tester_run_test.cpp, test/autons_test.cpp). The program only calls it, logs its lines
// and shows its result.
//
// ONE TRY FROM THE ARM TO THE END (the brief's R6 and R14). Whatever throws between arming the
// watch and the run's end -- a precondition in the countdown's wait, in the quiet tick, or inside
// runAuton -- is RECORDED (the RESULT word PROGRAM ERROR), the drive is panic-stopped (0 V + Brake;
// after a runAuton throw the guard has already safed the robot on the unwind), the watch is
// disarmed, and the tester's task lives on to say so. A throw that escaped would leave the watch
// armed and, on the robot, reach std::terminate from a PROS task with the drive in whatever state
// it was last given.
//
// THE FAULTS ARE THIS RUN'S. The fault latch counts for the whole boot; the record reads its count
// at RUN and at the end, and the last fault ONLY when the count rose -- a fault latched before RUN
// is never reported as this run's (test (3c)).
//
// THE MECHANISM SERVICE AND THE ROUTE REGISTRY (the mechanism core's MECH-A5, 2026-10-29): the run
// goes through competition::runAuton's MECHANISM-SERVICE overload, exactly as the competition
// program's does — the service handed over (enterAutonomous), the guard safing its built mechanisms,
// the routine's context carrying it — with the program's route registry, which holds the robot's
// derived mechanism route ids (a ROUTE auton's markers fire them).
//
// NOTHING HERE HAS RUN ON A ROBOT (worksheet Station M).

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <span>
#include <string_view>

#include "common/auton_run_log.hpp"
#include "common/guarded_run.hpp"
#include "programs/auton_abort_watch.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/match_runner.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/session_info.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The countdown between RUN and the first motion: time to step back (the team lead's "3 s").
inline constexpr double kTesterCountdownSeconds = 3.0;
/// One countdown wait: the motion tick (the program static_asserts it equals ProsTickPacer's).
inline constexpr double kTesterTickSeconds = 0.01;
static_assert(kTesterTickSeconds == kGuardedTickSeconds,
              "the countdown's tick and the chain proof's idle tick are the same motion tick");

/// One run, as the tester saw it -- the RESULT page and the log lines are built from it.
struct TesterRunRecord {
    int number = 0;                       ///< run N of this boot (1-based)
    int entry = -1;                       ///< the registry index run
    bool countdownAborted = false;        ///< aborted before anything moved: runAuton never called
    AbortCause cause = AbortCause::None;  ///< the watch's latch at the end (None = nobody aborted)
    std::uint16_t buttons = 0;            ///< the buttons down when a Button cause latched
    bool threw = false;                   ///< something threw between arm and end (a program error)
    char threwWhat[160]{};                ///< what it said
    competition::AutonRunResult run{};    ///< valid unless countdownAborted or threw
    math::Pose2d endEstimate{};           ///< the estimate when the run ended
    bool endLive = false;                 ///< ... and whether it was live then
    double countdownStartedAt = 0.0;      ///< the robot clock at RUN (s)
    int faultsBefore = 0;                 ///< the fault latch's count at RUN
    int faultsAfter = 0;                  ///< ... and when the run ended
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< the last fault DURING the run, or None
};

/// What the screen is told as a run moves on (the program shows the page; a host test records).
class ITesterRunObserver {
public:
    virtual ~ITesterRunObserver() = default;
    ITesterRunObserver() = default;
    ITesterRunObserver(const ITesterRunObserver&) = default;
    ITesterRunObserver(ITesterRunObserver&&) = default;
    ITesterRunObserver& operator=(const ITesterRunObserver&) = default;
    ITesterRunObserver& operator=(ITesterRunObserver&&) = default;
    /// The countdown started at `now` (s).
    virtual void countdownStarted(double now) = 0;
    /// The countdown ended untouched at `now` (s): the quiet tick paints RUNNING, then runAuton.
    virtual void runStarted(double now) = 0;
};

/// ONE run (file banner). `chassis` MUST be the Chassis built with `guard` as its pacer, the guard's
/// inner pacer MUST be `mechanisms` and the service's `watch` (the COMP brief's B7 layering, MECH-A5;
/// pacerChainMissing() checks it at boot). `mechanisms` is the robot's mechanism service, bound at
/// boot: the guard safes its built mechanisms — the Robot's PRESENT ones (B9). `commands` is the route
/// registry the robot's mechanism route ids were registered into (nullptr: a ROUTE auton is refused
/// by name). `runPressCount` is the press count
/// of the good touch read whose release touched RUN: the abort's touch baseline, so a finger that
/// lands after that read aborts however soon (AbortWatch::arm(pressBaseline)). Fills and returns
/// `rec` (the caller's storage -- never a task stack). Everything from the arm to the end is inside
/// ONE try: whatever throws there (a precondition -- a programming error -- or any other exception)
/// is recorded with its text, the drive is put in its safe state, the watch is disarmed, and the
/// tester's task lives on to show it.
inline TesterRunRecord& runTesterOnce(TesterRunRecord& rec, int number, chassis::Chassis& chassis,
                                      sequence::RunGuard& guard, AbortWatch& watch,
                                      const competition::AutonRegistry& registry, int index,
                                      hal::ITelemetrySink& telemetry,
                                      mechanism::MechanismService& mechanisms,
                                      const runner::CommandRegistry* commands,
                                      ITesterRunObserver& observer, std::int32_t runPressCount) {
    SHULIB_PRECONDITION(index >= 0 && static_cast<std::size_t>(index) < registry.size(),
                        "runTesterOnce: no such auton");
    const competition::AutonEntry& entry = registry.at(static_cast<std::size_t>(index));
    hal::IClock& clock = chassis.deps().ctx->clock();
    localization::Localizer& localizer = *chassis.deps().localizer;
    const diag::FaultLatch& faults = *chassis.deps().faults;
    rec = TesterRunRecord{};
    rec.number = number;
    rec.entry = index;
    rec.faultsBefore = faults.faultCount();

    try {
        // THE COUNTDOWN: the watch ARMED, one-tick waits that command nothing. Any abort ends it
        // here, and runAuton is never called: nothing moves. The touch baseline is RUN's own read,
        // so a finger that lands between it and this line is an abort too.
        watch.arm(runPressCount);
        rec.countdownStartedAt = clock.now().value();
        observer.countdownStarted(rec.countdownStartedAt);
        while (clock.now().value() - rec.countdownStartedAt < kTesterCountdownSeconds
               && watch.cause() == AbortCause::None) {
            chassis.wait(units::Time{kTesterTickSeconds});
        }
        if (watch.cause() == AbortCause::None) {
            // ONE QUIET TICK: the RUNNING page is painted whole on a tick that commands nothing,
            // so the run's first tick carries no full repaint, and the watch reads once more.
            observer.runStarted(clock.now().value());
            chassis.wait(units::Time{kTesterTickSeconds});
        }
        if (watch.cause() != AbortCause::None) {
            rec.countdownAborted = true;
        } else {
            // THE RUN: the competition program's call, on the same entry, guard, schedule,
            // mechanism service and route registry (MECH-A5; until then no registry, and a ROUTE
            // auton was refused by name — the COMP brief's B30).
            rec.run = competition::runAuton(chassis, guard, entry, commands, telemetry, mechanisms,
                                            registry.periodFor(entry));
        }
    } catch (const PreconditionError& e) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%.159s", e.what());
        chassis.cancel();  // the PANIC STOP: 0 V + Brake (runAuton's guard already safed on unwind)
    } catch (const std::exception& e) {
        // Any other standard exception (a student's std::out_of_range, a broken adapter's
        // std::runtime_error): its OWN words are the record, never "an unknown exception".
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%.159s", e.what());
        chassis.cancel();
    } catch (...) {
        rec.threw = true;
        std::snprintf(rec.threwWhat, sizeof rec.threwWhat, "%s", "an unknown exception");
        chassis.cancel();
    }
    watch.disarm();
    rec.cause = watch.cause();
    rec.buttons = watch.buttonsAtLatch();
    rec.endEstimate = localizer.pose();
    rec.endLive = localizer.qualityClass() != localization::Localizer::Quality::Uninitialized;
    rec.faultsAfter = faults.faultCount();
    rec.lastFault = rec.faultsAfter > rec.faultsBefore ? faults.lastFault() : diag::FaultCode::None;
    return rec;
}

// THE BOOT'S CHAIN PROOF is src/common/guarded_run.hpp's pacerChainMissing() since COMP-b's first
// step (2026-09-30; the mechanism service its third layer since MECH-A5): a template over any watch
// with paces() and attachedTo() -- the tester's AbortWatch is one. The program refuses its boot,
// naming the gap, unless it is ChainGap::None (a Chassis built without the guard, a service outside
// the chain or unbound, or a watch never attached, could not be aborted or would run no mechanism).

/// A throw reached the tester's task (a programming error): the drive at 0 V + Brake (the panic
/// stop), every PRESENT mechanism to its safe state and its mechanism-service slot cancelled --
/// src/common/guarded_run.hpp's program-error stop, each step tried on its own -- then the watch
/// disarmed. Never throws.
inline void stopAfterProgramError(chassis::Chassis& chassis,
                                  std::span<hal::IMechanism* const> present,
                                  AbortWatch& watch) noexcept {
    stopAfterProgramError(chassis, present);
    watch.disarm();
}

/// True when the watch latched an abort during a run the guard did NOT abandon. The watch asks
/// for the abandon on every pace while latched and the guard runs, and runAuton paces only inside
/// the guard's run -- so this is never a late button: it is the abort not reaching the guard (a
/// wiring fault the boot's chain proof exists to refuse), and it must never read as a pass.
[[nodiscard]] inline bool abortIgnored(const TesterRunRecord& r) noexcept {
    return r.cause != AbortCause::None && !r.countdownAborted && !r.threw
           && r.run.outcome != competition::AutonRunOutcome::Refused && !r.run.guard.abandoned;
}

/// The word a finished run earns -- the RESULT page's big line and the first log line say the same
/// (one source). First match wins: a countdown abort, a throw, a refusal, an abort, an abort the
/// guard never acted on (ABORT IGNORED, red), never started, the hard floor, the deadline cut, then
/// FINISHED (the routine -- or route -- OK and the end action succeeded) or ROUTINE STOPPED. At
/// most ui::kLargeCols characters (test (23)).
[[nodiscard]] inline const char* testerResultWord(const TesterRunRecord& r) noexcept {
    if (r.countdownAborted) {
        return "ABORTED";
    }
    if (r.threw) {
        return "PROGRAM ERROR";
    }
    if (r.run.outcome == competition::AutonRunOutcome::Refused) {
        return "REFUSED";
    }
    if (r.run.guard.abandoned) {
        return "ABORTED";
    }
    if (abortIgnored(r)) {
        return "ABORT IGNORED";
    }
    if (r.run.outcome == competition::AutonRunOutcome::NotStarted) {
        return "NOT STARTED";
    }
    if (r.run.guard.floorFired) {
        return "HARD STOP";
    }
    if (r.run.guard.scoringCut) {
        return "CUT BY DEADLINE";
    }
    const bool ranOk = r.run.outcome == competition::AutonRunOutcome::RouteRan
                           ? r.run.route.ok()
                           : r.run.routine.ok;
    return ranOk && r.run.guard.endActionSucceeded ? "FINISHED" : "ROUTINE STOPPED";
}

/// The names of the buttons in `mask` ("B", "L1+R2"), bit = hal::ControllerButton's value. Always
/// terminated; stops at a name that would not fit (all twelve take 38 characters).
inline void buttonNamesOf(std::uint16_t mask, char* out, std::size_t cap) noexcept {
    static constexpr const char* kNames[12] = {"L1", "L2", "R1", "R2", "UP", "DOWN",
                                               "LEFT", "RIGHT", "X", "B", "Y", "A"};
    if (out == nullptr || cap == 0) {
        return;
    }
    out[0] = '\0';
    std::size_t n = 0;
    for (int b = 0; b < 12; ++b) {
        if ((mask & (1u << b)) == 0) {
            continue;
        }
        const int w = std::snprintf(out + n, cap - n, "%s%s", n == 0 ? "" : "+", kNames[b]);
        if (w < 0 || static_cast<std::size_t>(w) >= cap - n) {
            out[n] = '\0';  // never a half name
            return;
        }
        n += static_cast<std::size_t>(w);
    }
}

/// The routine's stop, in the words the Routine's own stop line uses.
[[nodiscard]] constexpr const char* routineStopWord(chassis::RoutineStopCause cause,
                                                    control::ExitReason exit) noexcept {
    switch (cause) {
        case chassis::RoutineStopCause::None: return "no stop";
        case chassis::RoutineStopCause::MotionFailed:
            return exit == control::ExitReason::TimedOut    ? "motion TIMEOUT"
                   : exit == control::ExitReason::Cancelled ? "motion CANCELLED"
                                                            : "motion failed";
        case chassis::RoutineStopCause::WaitTimedOut: return "wait TIMEOUT";
        case chassis::RoutineStopCause::ActionFailed: return "action FAILED";
        case chassis::RoutineStopCause::MechanismFailed: return "mechanism FAILED";
        case chassis::RoutineStopCause::MechanismStepFailed: return "mechanism step FAILED";
    }
    return "stopped";
}

/// The most bytes one log line may take: TermSink cuts a message at 200 (diag/term_sink.hpp).
inline constexpr std::size_t kTesterLineBytes = 200;

/// The lines one finished run leaves in the log -- serial and the SD card carry the same words.
struct TesterRunLines {
    char line[4][kTesterLineBytes]{};  ///< each <= kTesterLineBytes - 1 characters, ASCII
    int count = 0;                     ///< how many are real
    bool cut = false;                  ///< a line did not fit and was cut (a test failure, never
                                       ///< expected: every field below is precision-bounded)
};

namespace detail {
/// Record one snprintf's verdict: a line that did not fit sets `cut`.
inline void noteLine(TesterRunLines& out, int written) noexcept {
    if (written < 0 || static_cast<std::size_t>(written) >= kTesterLineBytes) {
        out.cut = true;
    }
}
}  // namespace detail

/// The run's lines (golden-tested, test (23)): what happened, the guard's report, the routine's
/// verdict with THIS run's faults, and the seat against the END estimate -- the numbers to compare
/// with the tape. A countdown abort and a throw are ONE line each. Every variable string carries a
/// precision, so -Wformat-truncation can see each line's real bound (the brief's landmine 16).
[[nodiscard]] inline TesterRunLines testerRunLines(const TesterRunRecord& r,
                                                   const competition::AutonEntry& e) {
    TesterRunLines out;
    char buttons[40] = "";
    buttonNamesOf(r.buttons, buttons, sizeof buttons);
    const char* word = testerResultWord(r);
    const bool byButton = r.cause == AbortCause::Button;
    if (r.countdownAborted) {
        detail::noteLine(
            out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                               "run %d: '%.29s' %s in the countdown by %s%s%.38s -- runAuton was "
                               "never called; nothing moved",
                               r.number, e.name, word, abortCauseWord(r.cause),
                               byButton ? " " : "", byButton ? buttons : ""));
        return out;
    }
    if (r.threw) {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: '%.29s' %s -- it threw (a program error): "
                                            "%.88s; the drive was stopped",
                                            r.number, e.name, word, r.threwWhat));
        return out;
    }
    const sequence::RunGuardReport& g = r.run.guard;
    const bool refused = r.run.outcome == competition::AutonRunOutcome::Refused;

    // 1. the word, the outcome, how it ended, the end action. An abort that lands while the end
    //    action runs CUTS it (the guard voids its verdict): "CUT by the abort", never "FAILED" --
    //    nothing is wrong with the brake. An abort the guard never acted on names its cause.
    const bool ignored = abortIgnored(r);
    const bool named = g.abandoned || ignored;
    if (refused) {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: '%.29s' %s -- runAuton refused it before the "
                                            "guard armed; nothing moved",
                                            r.number, e.name, word));
    } else {
        detail::noteLine(
            out,
            std::snprintf(out.line[out.count++], kTesterLineBytes,
                          "run %d: '%.29s' %s -- %s; %s%.27s%s%.38s, end action %s%s", r.number,
                          e.name, word, competition::autonRunOutcomeWord(r.run.outcome),
                          g.abandoned ? "ABANDONED by "
                          : ignored   ? "IGNORED the abort by "
                                      : (g.scoringCut ? "cut at the deadline" : "in time"),
                          named ? abortCauseWord(r.cause) : "", named && byButton ? " " : "",
                          named && byButton ? buttons : "",
                          !g.endActionRan       ? "skipped"
                          : g.abandoned         ? "CUT by the abort"
                          : g.endActionSucceeded ? "succeeded"
                                                 : "FAILED",
                          g.floorFired ? ", HARD STOP fired" : ""));
    }

    // 2. the guard's clock: when scoring and the end action ended, the paces, the refused retries,
    //    the hand-off latency (B12's number) and how long the estimate took to be live.
    if (refused) {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: the guard never armed", r.number));
    } else {
        char endedAt[24] = "--";  // "--" when the end action did not run
        if (g.endActionRan) {
            std::snprintf(endedAt, sizeof endedAt, "%.2fs", g.endActionEnded.value());
        }
        detail::noteLine(
            out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                               "run %d: scoring ended T+%.2fs, end action ended T+%.12s, %d paces, "
                               "%d cancels after the cut; hand-off %.3fs after the countdown, live "
                               "after %.2fs",
                               r.number, g.scoringEnded.value(), endedAt, g.pacesSeen,
                               g.postExpiryCancels,
                               r.run.armedAt.value()
                                   - (r.countdownStartedAt + kTesterCountdownSeconds),
                               r.run.liveAfter.value()));
    }

    // 3. the routine's verdict (or runAuton's sentence for REFUSED / NOT STARTED), and THIS run's
    //    faults -- never the boot's (the record's counts; file banner).
    const int faultsThisRun = r.faultsAfter - r.faultsBefore;
    char faults[80] = "faults this run: none";
    if (faultsThisRun != 0) {
        std::snprintf(faults, sizeof faults,
                      "faults this run: %d, last %.20s -- see the [ERROR] line", faultsThisRun,
                      diag::faultCodeName(r.lastFault));
    }
    if (r.run.outcome == competition::AutonRunOutcome::RoutineRan) {
        const chassis::RoutineResult& rr = r.run.routine;
        if (rr.stoppedAt > 0) {
            detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                                "run %d: routine STOPPED, %d of %d steps done, "
                                                "stopped at %.24s (%s); %.79s",
                                                r.number, rr.completed, rr.steps, rr.stoppedName,
                                                routineStopWord(rr.cause, rr.exit), faults));
        } else {
            detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                                "run %d: routine %s, %d of %d steps done; %.79s",
                                                r.number, rr.ok ? "OK" : "STOPPED", rr.completed,
                                                rr.steps, faults));
        }
    } else if (r.run.outcome == competition::AutonRunOutcome::RouteRan) {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: route %s, %d legs run, %d skipped; %.79s",
                                            r.number, r.run.route.ok() ? "OK" : "STOPPED",
                                            r.run.route.legsRun, r.run.route.legsSkipped, faults));
    } else if (faultsThisRun == 0) {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: %.150s; %.21s", r.number, r.run.why, faults));
    } else {
        detail::noteLine(out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                                            "run %d: %.100s; %.79s", r.number, r.run.why, faults));
    }

    // 4. where the run was seated, and the END estimate -- the numbers to put against the tape.
    detail::noteLine(
        out, std::snprintf(out.line[out.count++], kTesterLineBytes,
                           "run %d: seated (%.2f, %.2f) facing %.1f deg (the IMU read %.1f); END "
                           "estimate (%.2f, %.2f) facing %.1f deg %s -- measure the tape",
                           r.number, r.run.seated.x().value(), r.run.seated.y().value(),
                           r.run.seated.heading().degrees(), r.run.imuAtSeat.degrees(),
                           r.endEstimate.x().value(), r.endEstimate.y().value(),
                           r.endEstimate.heading().degrees(), r.endLive ? "LIVE" : "NOT LIVE"));
    return out;
}

/// The SD run log's state at boot, for the boot lines: src/common/auton_run_log.hpp's AutonRunLog
/// since COMP-b (2026-09-30) -- the competition program appends to the same file and says the same
/// words about it, so both are stated once there. The tester's name for it stays.
using TesterSdLog = AutonRunLog;

/// The boot's log lines (serial and the SD card), THREE of them: what booted and which build; how
/// it booted and with what; the SD run log. The brief's flow printed them as ONE line, and at a
/// real build hash that line passed TermSink's 199 bytes and lost its SD path -- 206 bytes at this
/// repository's own `git describe` ("v0.1.1-354-gee2e399-dirty"), and a hash may be up to
/// diag::kMaxHashBytes (COMP-t's review, 2026-09-29). Split, every field bounded, each line fits
/// at the worst case of every field (test (23b)).
struct TesterBootLines {
    char line[3][kTesterLineBytes]{};  ///< each <= kTesterLineBytes - 1 characters
    bool cut = false;                  ///< a line did not fit (a test failure, never expected)
};

/// The boot lines (TesterBootLines). `buildHash` empty reads MISSING (the session header's rule);
/// longer than diag::kMaxHashBytes it is cut there. `sdPath` is printed only when the log is on.
[[nodiscard]] inline TesterBootLines testerBootLines(const char* variantBeacon,
                                                     std::string_view buildHash,
                                                     const char* bootSource, int autons,
                                                     int present, int mechanisms, TesterSdLog sd,
                                                     const char* sdPath) {
    TesterBootLines out;
    const auto note = [&out](int written) {
        if (written < 0 || static_cast<std::size_t>(written) >= kTesterLineBytes) {
            out.cut = true;
        }
    };
    const std::string_view hash = buildHash.empty() ? std::string_view{"MISSING"} : buildHash;
    const int hashLen = static_cast<int>(hash.size() < diag::kMaxHashBytes ? hash.size()
                                                                            : diag::kMaxHashBytes);
    note(std::snprintf(out.line[0], kTesterLineBytes, "boot: shulib AUTON TESTER [%.40s] hash %.*s",
                       variantBeacon != nullptr ? variantBeacon : "", hashLen, hash.data()));
    note(std::snprintf(out.line[1], kTesterLineBytes,
                       "boot source %.11s, %d autons, %d of %d mechanisms present",
                       bootSource != nullptr ? bootSource : "", autons, present, mechanisms));
    note(std::snprintf(out.line[2], kTesterLineBytes, "SD run log %s%.60s", autonRunLogWords(sd),
                       sd != TesterSdLog::Off && sdPath != nullptr ? sdPath : ""));
    return out;
}

}  // namespace shulib::bench
