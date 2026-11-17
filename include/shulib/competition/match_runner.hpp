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
// ── With the robot's MECHANISM SERVICE (chunk MECH-A4, API 2.5) ─────────────────────
// The second overload takes the program's mechanism::MechanismService instead of a span. Three
// things differ, nothing above moves (both overloads run ONE body, detail::runAutonImpl):
//   0. AUTON ENTRY, right after the preconditions and before step 1: the service's
//      enterAutonomous(). PROS deletes the driver-control task on a mode change, so driver
//      control's own exit may never have run — a driver's intake left at 12 V would otherwise run
//      through the whole auton (the MECH brief's D1, measured on the host test plant: 269 of 269
//      auton paces at 12 V without the call, 0 with it). It clears the service's re-entrancy flag,
//      releases a claim another owner left (said, by name), stops every motor mechanism's
//      behaviour (its safe state) and leaves a pneumatic's level alone. It runs even when the
//      entry is then refused: whatever driver control left running stops either way.
//   1. The guard's span is the service's mechanisms() — exactly the Robot's built ones (B9).
//   5. The routine's AutonContext carries the service (`mechanismService`), so `Routine r{ctx,
//      "name"}` reaches every mechanism by its declaration; a route's mechanism markers reach it
//      through the program's registry (runner/mechanism_routes.hpp).
// The service must be IN the chassis's pacer chain, under the guard (Chassis -> RunGuard ->
// MechanismService -> the program's watch -> the tick pacer): a routine whose mechanism step
// waits on a service that never ticks stops, saying so (chassis/routine.hpp).
//
// ── A MIRRORED entry (chunk MIRROR1-A, 2026-11-13; competition/field_mirror.hpp) ─────
// An entry `competition::mirrored(...)` marked runs robot one's route on robot two, reflected. Three
// things differ, nothing above moves:
//   4. The re-seat writes the REFLECTED start (field_mirror.hpp's seatedStart: a FIELD start through
//      the entry's FieldMirror, a RELATIVE one through the robot-frame flip) — position AND heading
//      on one tick, as ever — so `seated` is robot two's REAL start, where the drivers must place it.
//      Its line's mode word reads "FIELD MIRRORED" / "RELATIVE MIRRORED", and a second line names the
//      mirror and robot one's start as written (both ASCII, both under the 160-byte buffer and the
//      terminal's 200-byte cut at their worst case — test/mirrored_auton_test.cpp measures them).
//   5-6. ONE chassis::MirroredRun is opened after the re-seat and closed after the end action, around
//      the routine (or route) AND the park, so the park runs mirrored too; it closes on every way out
//      (a normal end, the deadline's cut, an abandon, a throw unwinding through runAuton) — before
//      runAuton returns, so everything read afterwards (the program's END estimate) is the real
//      field. The arming line's start word says MIRRORED as well, so a mirrored run that never
//      starts is still named one.
// An UNMIRRORED entry runs byte for byte as before (test/mirror_unchanged_runs.hpp's hash, measured
// at the merge base, pinned). runAuton refuses (a precondition) a Chassis whose mirror is still on —
// a leaked run would reflect an unmirrored auton.
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
#include <optional>
#include <span>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/competition/field_mirror.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/mechanism/service.hpp"
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
    /// MIRROR1-A: true when the entry was a MIRRORED one — `seated` is then robot two's REAL start (the
    /// reflection of the entry's own), and the screens' START line says MIR.
    bool mirrored = false;

    /// True when the auton's routine or route actually started.
    [[nodiscard]] bool started() const noexcept {
        return outcome == AutonRunOutcome::RoutineRan || outcome == AutonRunOutcome::RouteRan;
    }
};

namespace detail {
/// THE ONE BODY of both runAuton overloads (file banner: the order of events, and the service's
/// three differences). `service` is nullptr for COMP-a's overload.
[[nodiscard]] inline AutonRunResult runAutonImpl(chassis::Chassis& chassis, sequence::RunGuard& guard,
                                                 const AutonEntry& entry,
                                                 const runner::CommandRegistry* commands,
                                                 hal::ITelemetrySink& telemetry,
                                                 std::span<hal::IMechanism* const> mechanisms,
                                                 mechanism::MechanismService* service,
                                                 units::Time period) {
    SHULIB_PRECONDITION(!guard.running(), "runAuton: the guard is already running a run");
    SHULIB_PRECONDITION(!chassis.scheduler().hasActiveMotion(),
                        "runAuton: a motion is still active -- the start pose cannot be re-seated "
                        "under it");
    SHULIB_PRECONDITION(chassis.reflection().isIdentity(),
                        "runAuton: the chassis is still in a mirrored run -- a chassis::MirroredRun outlived "
                        "its auton");
    AutonRunResult result{};

    // ── 0. auton entry: whatever driver control left running stops HERE (MECH-A4; banner) ──
    if (service != nullptr) {
        (void)service->enterAutonomous();  // a released leftover claim is said, by name, by the service
    }

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
    AutonContext ctx{chassis, guard, telemetry, mechanisms, entry, service};
    // MIRROR1-A: the run's reflection (the identity for an entry as written) and the scope that applies
    // it — opened after the re-seat, closed after the end action or by the unwind (file banner).
    const math::Reflection mirror = runReflection(entry);
    const bool isMirroredRun = !mirror.isIdentity();
    result.mirrored = isMirroredRun;
    std::optional<chassis::MirroredRun> mirroredRun;
    {
        char line[160];
        std::snprintf(line, sizeof line,
                      "auton '%s' (%s, period %.1fs, park T+%.2fs, stop T+%.2fs, start %s%s): arming",
                      entry.name, autonKindWord(entry.kind), period.value(),
                      entry.endActionAt.value(), entry.hardStopAt.value(),
                      startModeWord(start.mode()), isMirroredRun ? " MIRRORED" : "");
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
            // 4. the re-seat: position AND heading, on ONE tick (C2, B14) — for a mirrored entry, the
            //    REFLECTED start (robot two's real one), written before the mirrored run opens.
            result.seated = isMirroredRun ? mirror.apply(start.pose()) : start.pose();
            result.imuAtSeat = chassis.deps().ctx->imu().heading();
            chassis.setPose(result.seated);
            localizer.reseatHeading(result.seated.heading());
            {
                char line[160];
                std::snprintf(line, sizeof line,
                              "auton '%s': re-seated %s%s (%.2f, %.2f) in facing %.1f deg; the IMU "
                              "read %.1f deg, live after %.2fs",
                              entry.name, startModeWord(start.mode()), isMirroredRun ? " MIRRORED" : "",
                              result.seated.x().value(), result.seated.y().value(),
                              result.seated.heading().degrees(), result.imuAtSeat.degrees(),
                              result.liveAfter.value());
                telemetry.log(hal::LogLevel::Info, "COMP", line);
            }
            if (isMirroredRun) {
                // The second line: which mirror, and robot one's start AS WRITTEN (ASCII; file banner).
                const math::Pose2d asWritten = start.pose();
                char line[160];
                const bool field = start.mode() == StartPose::Mode::Field;
                std::snprintf(line, sizeof line,
                              "auton '%s': MIRRORED %s%.32s -- robot one's start (%.2f, %.2f) facing %.1f deg",
                              entry.name, field ? "across " : "in the robot's own frame",
                              field ? entry.mirror->name : "", asWritten.x().value(), asWritten.y().value(),
                              asWritten.heading().degrees());
                telemetry.log(hal::LogLevel::Info, "COMP", line);
                mirroredRun.emplace(chassis, mirror);
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
    mirroredRun.reset();  // MIRROR1-A: the real field from here on (file banner)

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
}  // namespace detail


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
    return detail::runAutonImpl(chassis, guard, entry, commands, telemetry, mechanisms, nullptr, period);
}

/// Run `entry` under `guard` WITH the robot's MECHANISM SERVICE (chunk MECH-A4, API 2.5; the file
/// banner's three differences): the service's enterAutonomous() first, the guard's span is
/// `mechanisms.mechanisms()`, and the routine's AutonContext carries the service, so `Routine
/// r{ctx, "name"}` reaches every mechanism by its declaration. Every rule of the overload above
/// holds unchanged. `mechanisms` belongs in the chassis's pacer chain under the guard, bound at
/// boot before any auton can run; an UNBOUND one has no mechanisms (an empty span for the guard),
/// and every mechanism step then ends ABSENT, saying the service is not bound — the drive steps
/// still run.
[[nodiscard]] inline AutonRunResult runAuton(chassis::Chassis& chassis, sequence::RunGuard& guard,
                                             const AutonEntry& entry,
                                             const runner::CommandRegistry* commands,
                                             hal::ITelemetrySink& telemetry,
                                             mechanism::MechanismService& mechanisms,
                                             units::Time period) {
    return detail::runAutonImpl(chassis, guard, entry, commands, telemetry, mechanisms.mechanisms(),
                                &mechanisms, period);
}

}  // namespace shulib::competition
