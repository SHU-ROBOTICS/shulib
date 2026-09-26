#pragma once
//
// competition::runAuton — run ONE registered auton under a RunGuard, from its start pose to its
// guaranteed end (chunk COMP-a, 2026-09-26; the COMP brief's rulings C2, C5, C6 and amendments
// B9, B10, B12, B14, B16).
//
// ── The order of events, and why each is where it is ────────────────────────────────
//   1. REFUSE, with a sentence, anything that cannot run — BEFORE the guard arms: an entry that
//      fails the registry's own check against the period it is about to run in (an unset
//      schedule, a hard stop past the period — C6; the guard's run() would THROW on the first),
//      a route entry with no command registry, a null in the mechanisms span (B9: the span is the
//      Robot's PRESENT mechanisms; an absent one is left out and logged by name where its name is
//      known — the program). A refused auton moves nothing and returns `Refused`.
//   2. ARM THE GUARD FIRST (B12). The run's clock starts as close to autonomous() as the hand-off
//      allows, so the schedule the students wrote is measured from the start the field measures.
//   3. Inside the guarded run, WAIT FOR A LIVE ESTIMATE with the guard's own wait (B12) — the one
//      wait the deadline and a mid-auton disable can cut. An auton started while the IMU is still
//      calibrating is slow, not wrong: it spends its first seconds here, motionless.
//   4. RE-SEAT position AND heading on ONE tick (C2, B14): Chassis::setPose then
//      Localizer::reseatHeading, with no update between. Only now — a re-seat during calibration
//      would compute the heading bias from calibration garbage — and only with no motion active.
//   5. Run the routine (or the route through a runner::PathRunner), with an AutonContext.
//   6. The guard does the rest: the cut at endActionAt, the end action (the entry's, or
//      `chassis.brake()`), the hard floor, the full cancel-all, and — if the program's watch
//      called requestAbandon() — the skipped end action (run_guard.hpp's ABANDON banner).
//      A run that never STARTED (step 3 ran out) gets `chassis.brake()` as its end action
//      WHATEVER the entry says: an entry's park is written for the frame step 4 seats, and that
//      frame was never seated. Were the IMU to come live during the end action's runway, a
//      field-frame park would otherwise drive to its pose in the boot frame — the IMU's boot 0
//      for a heading and wherever the estimate began for a position (COMP-a review finding).
// The result is TAGGED (B16): the RunGuardReport always, plus EITHER the routine's RoutineResult
// OR the route's RunReport, never a mix a reader has to guess about.
//
// ── What it does not do ─────────────────────────────────────────────────────────────
// It owns no task and reads no competition state: the program's auton task calls it, and the
// program's watch (a pacer under the guard) abandons it. It does not catch: a routine that THROWS
// (a precondition — a programming error) is safed by the guard on the unwind and the exception
// reaches the caller, whose task body catches it and says what broke (COMP-b). It never falls back
// to a default auton, a default schedule or a default start.
//
// PROS-free, allocation-free. NOTHING HERE HAS RUN ON A ROBOT.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/path_runner.hpp"
#include "shulib/sequence/run_guard.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::competition {

/// How a runAuton call ended — the TAG on AutonRunResult (B16). Read it before any other field.
enum class AutonRunOutcome : std::uint8_t {
    /// Refused before the guard armed; `why` says what. Nothing moved; `guard` is empty.
    Refused,
    /// The guard armed but the auton never started: the estimate did not go live before the run
    /// ended — the deadline, or an abandon (`guard.abandoned` says which). Nothing was re-seated;
    /// the guard's cancel-all still ran, and — unless abandoned — so did the end-of-run slot,
    /// filled by `chassis.brake()` and NEVER by the entry's own end action (that park is written
    /// for a start pose this run never seated; file banner, step 6). `why` says what.
    NotStarted,
    /// The routine ran; `routine` holds its verdict.
    RoutineRan,
    /// The route ran; `route` holds its report.
    RouteRan,
};

/// The outcome as a word ("REFUSED", "NOT_STARTED", "ROUTINE_RAN", "ROUTE_RAN").
[[nodiscard]] constexpr const char* autonRunOutcomeWord(AutonRunOutcome o) noexcept {
    switch (o) {
        case AutonRunOutcome::Refused: return "REFUSED";
        case AutonRunOutcome::NotStarted: return "NOT_STARTED";
        case AutonRunOutcome::RoutineRan: return "ROUTINE_RAN";
        case AutonRunOutcome::RouteRan: return "ROUTE_RAN";
    }
    return "UNKNOWN";
}

/// What one runAuton call did (B16's tagged result).
struct AutonRunResult {
    /// The tag.
    AutonRunOutcome outcome = AutonRunOutcome::Refused;
    /// For Refused and NotStarted: the sentence, naming the auton. Empty otherwise.
    char why[192]{};
    /// The guard's own account — valid unless Refused. `guard.pacesSeen == 0` after a run that
    /// did work means the Chassis was not built with this guard as its pacer.
    sequence::RunGuardReport guard{};
    /// The routine's verdict — valid only when `outcome == RoutineRan`.
    chassis::RoutineResult routine{};
    /// The route's report — valid only when `outcome == RouteRan`.
    runner::RunReport route{};
    /// The clock reading at which the guard ARMED (valid unless Refused) — the instant the
    /// auton's schedule is measured from. The program subtracts its own autonomous()-entry
    /// reading from this and logs the hand-off latency (B12: arm as close to autonomous() as the
    /// hand-off allows, and say how close that was).
    units::Time armedAt{0.0};
    /// Seconds from the guard arming to the estimate being live — 0 only when it was live
    /// already. "Live" is `qualityClass() != Uninitialized`, which a Localizer never reports on
    /// its first update() (localizer.hpp, Quality), so one nothing has ticked yet takes two
    /// ticks here even with an IMU that was ready all along.
    units::Time liveAfter{0.0};
    /// The pose the re-seat set (the entry's effective start) — valid for RoutineRan / RouteRan.
    math::Pose2d seated{};
    /// The IMU's raw heading at the re-seat — what the start heading REPLACED (valid as `seated`).
    math::Angle imuAtSeat{};

    /// True when the auton's routine or route actually started.
    [[nodiscard]] bool started() const noexcept {
        return outcome == AutonRunOutcome::RoutineRan || outcome == AutonRunOutcome::RouteRan;
    }
};

/// Run `entry` under `guard` (file banner: the order of events).
///
///  * `chassis` MUST be the Chassis built with `guard` as its pacer (robot::BuildOptions::pacer) —
///    the guard counts paces and the result says so if it never saw one.
///  * `commands` is the command registry a ROUTE entry's markers fire through; required for a
///    route entry (refused without one), ignored for a routine entry, and may be nullptr then.
///  * `mechanisms` is what the guard safes at the cut and at the end — the Robot's PRESENT
///    mechanisms (B9). No nullptr (refused, before arming).
///  * `period` is the period this run is in — the PROGRAM's constant for the entry's kind (C6).
///
/// PRECONDITIONS (a programming error, thrown): the guard is not already running, and no motion is
/// active on the chassis (the re-seat must not move a heading under a live motion — B14). Every
/// other problem is a REFUSAL with a sentence, returned without arming.
[[nodiscard]] inline AutonRunResult runAuton(chassis::Chassis& chassis, sequence::RunGuard& guard,
                                             const AutonEntry& entry,
                                             const runner::CommandRegistry* commands,
                                             hal::ITelemetrySink& telemetry,
                                             std::span<hal::IMechanism* const> mechanisms,
                                             units::Time period) {
    SHULIB_PRECONDITION(!guard.running(), "runAuton: the guard is already running a run");
    SHULIB_PRECONDITION(!chassis.scheduler().hasActiveMotion(),
                        "runAuton: a motion is still active -- the start pose cannot be re-seated "
                        "under it");
    AutonRunResult result{};

    // ── 1. refusals, before anything arms (C6, B9, B16) ───────────────────────────────────
    const AutonVerdict verdict = checkAutonEntry(entry, period);
    if (!verdict.ok) {
        std::snprintf(result.why, sizeof result.why, "%s", verdict.why);
    } else if (entry.route != nullptr && commands == nullptr) {
        std::snprintf(result.why, sizeof result.why,
                      "auton '%s': a route auton needs a command registry to fire its markers",
                      entry.name);
    } else {
        for (std::size_t i = 0; i < mechanisms.size(); ++i) {
            if (mechanisms[i] == nullptr) {
                std::snprintf(result.why, sizeof result.why,
                              "auton '%s': mechanism slot %d is null -- pass the Robot's PRESENT "
                              "mechanisms, leaving an absent one out",
                              entry.name, static_cast<int>(i));
                break;
            }
        }
    }
    if (result.why[0] != '\0') {
        result.outcome = AutonRunOutcome::Refused;
        telemetry.log(hal::LogLevel::Warn, "COMP", result.why);
        return result;
    }

    const StartPose start = entry.effectiveStart();
    localization::Localizer& localizer = *chassis.deps().localizer;
    hal::IClock& clock = chassis.deps().ctx->clock();
    AutonContext ctx{chassis, guard, telemetry, mechanisms, entry};
    {
        char line[160];
        std::snprintf(line, sizeof line,
                      "auton '%s' (%s, period %.1fs, park T+%.2fs, stop T+%.2fs, start %s): arming",
                      entry.name, autonKindWord(entry.kind), period.value(),
                      entry.endActionAt.value(), entry.hardStopAt.value(),
                      startModeWord(start.mode()));
        telemetry.log(hal::LogLevel::Info, "COMP", line);
    }

    // ── 2. arm FIRST; 3-5 happen inside the guarded run (B12) ────────────────────────────
    const sequence::RunGuardConfig runCfg{
        .endActionAt = entry.endActionAt, .hardStopAt = entry.hardStopAt, .mechanisms = mechanisms};
    result.guard = guard.run(
        chassis, runCfg,
        [&] {
            const double armedAt = clock.now().value();
            result.armedAt = units::Time{armedAt};
            // 3. the guard's own wait — cut by the deadline and by an abandon (B12).
            const sequence::GuardedWaitResult live = guard.waitFor(
                [&localizer] {
                    return localizer.qualityClass()
                           != localization::Localizer::Quality::Uninitialized;
                },
                entry.hardStopAt);
            result.liveAfter = units::Time{clock.now().value() - armedAt};
            if (live != sequence::GuardedWaitResult::Satisfied) {
                result.outcome = AutonRunOutcome::NotStarted;
                std::snprintf(result.why, sizeof result.why,
                              "auton '%s': the estimate never went live (IMU still booting?) "
                              "before the run %s -- nothing ran",
                              entry.name,
                              guard.abandonRequested() ? "was abandoned" : "ran out of time");
                telemetry.log(hal::LogLevel::Warn, "COMP", result.why);
                return;
            }
            // 4. the re-seat: position AND heading, on ONE tick (C2, B14).
            result.seated = start.pose();
            result.imuAtSeat = chassis.deps().ctx->imu().heading();
            chassis.setPose(result.seated);
            localizer.reseatHeading(result.seated.heading());
            {
                char line[160];
                std::snprintf(line, sizeof line,
                              "auton '%s': re-seated %s (%.2f, %.2f) in facing %.1f deg; the IMU "
                              "read %.1f deg, live after %.2fs",
                              entry.name, startModeWord(start.mode()),
                              result.seated.x().value(), result.seated.y().value(),
                              result.seated.heading().degrees(), result.imuAtSeat.degrees(),
                              result.liveAfter.value());
                telemetry.log(hal::LogLevel::Info, "COMP", line);
            }
            // 5. the auton.
            if (entry.routine != nullptr) {
                result.routine = entry.routine(ctx);
                result.outcome = AutonRunOutcome::RoutineRan;
            } else {
                runner::PathRunner runner{chassis, *commands, telemetry};
                result.route = runner.run(*entry.route);
                result.outcome = AutonRunOutcome::RouteRan;
            }
        },
        [&]() -> bool {
            // 6. the end action: the entry's park, or brake (the brief's default). A run that
            //    never started gets brake whatever the entry says (file banner, step 6).
            if (result.outcome == AutonRunOutcome::NotStarted) {
                if (entry.endAction != nullptr) {
                    char line[192];
                    std::snprintf(line, sizeof line,
                                  "auton '%s': never started -- its end action is NOT run (it "
                                  "is written for a start pose that was never seated); braking "
                                  "instead",
                                  entry.name);
                    telemetry.log(hal::LogLevel::Warn, "COMP", line);
                }
                return chassis.brake() == control::ExitReason::Settled;
            }
            if (entry.endAction != nullptr) {
                return entry.endAction(ctx);
            }
            return chassis.brake() == control::ExitReason::Settled;
        });

    {
        char line[160];
        std::snprintf(line, sizeof line,
                      "auton '%s': %s; %s%s%s, %d paces",
                      entry.name, autonRunOutcomeWord(result.outcome),
                      result.guard.abandoned
                          ? "ABANDONED"
                          : (result.guard.scoringCut ? "cut at the deadline" : "finished in time"),
                      result.guard.endActionRan
                          ? (result.guard.endActionSucceeded ? ", end action succeeded"
                                                             : ", end action FAILED")
                          : ", end action skipped",
                      result.guard.floorFired ? ", HARD STOP fired" : "",
                      result.guard.pacesSeen);
        telemetry.log(result.guard.abandoned || result.guard.floorFired ? hal::LogLevel::Warn
                                                                        : hal::LogLevel::Info,
                      "COMP", line);
    }
    return result;
}

}  // namespace shulib::competition
