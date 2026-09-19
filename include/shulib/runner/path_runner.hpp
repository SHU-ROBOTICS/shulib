#pragma once
//
// PathRunner — the engine that EXECUTES a Route (chunk G2, 2026-09-18; master plan §16.3).
//
// ── What this file finally closes ───────────────────────────────────────────────────
// This team has authored three parallel data representations of an autonomous routine and
// executed none of them (docs/legacy-command-vocabulary.md §1). What was missing every time
// was not the format — it was this: a thing that walks the waypoints, calls the drivetrain
// verbs, and fires the markers. G1 built the data; this builds the runner; G3 fills the data
// from a `.vexbot` file. A Route built by hand in C++ and a Route parsed from a file are the
// same object and run through the same code, which is what makes the file path testable
// before the file format exists.
//
// ── It drives the FACADE, never the recipe chain (ruling D1, restated at G2 R1) ─────
// `chassis::Routine` is the Tier-2 recipe layer and this runner is NOT one of its clients.
// The chain is EAGER and fluent — built for a human writing steps in program order — and its
// verdict type, its step numbering and its log vocabulary all describe an authored chain, not
// a table of waypoints. So the runner calls `Chassis` directly.
//
// That leaves one thing the two layers must agree on EXACTLY: the bearing. `Routine::driveTo`
// computes "face the point" as `atan2` of the deltas from the live pose estimate, and if this
// file computed it any other way — a different wrap, a different pose read, `atan2(dx, dy)` —
// a route and a recipe aiming at the same point would disagree by an ulp today and by a
// degree after the first refactor. `bearingTo` below is `Routine::bearingTo`'s expression,
// character for character, and a test on identical rigs pins the two runs BIT-IDENTICAL.
// (It is duplicated rather than shared because `Routine::bearingTo` is private and
// `routine.hpp` is FROZEN — F10's pins fail the build if it is touched.)
//
// ── The error policy is D1's, in meaning, verbatim ──────────────────────────────────
// A route that keeps driving after a failed leg is chasing the field from a position it is
// not at. So on the FIRST leg that exits non-Settled, the FIRST handler that reports failure,
// or the FIRST marker outcome other than `Succeeded`:
//   1. the route STOPS,
//   2. the drive is put in the defined safe state — `Chassis::cancel()` (0 V + Brake,
//      idempotent), NOT `brake()`: `brake()` is a blocking MOTION that can itself time out,
//      and the safe state must not be able to fail,
//   3. every later waypoint is SKIPPED, counted, and visible in the report,
//   4. one Warn names the route, the waypoint and the cause.
// PRECONDITION THROWS ARE NOT POLICY: a NaN or a nonsense duration throws out of the facade
// exactly as it would from any other caller. A programming error stays loud.
//
// ── Markers fire on Settled, and only on Settled (ruling R2) ────────────────────────
// A waypoint's markers run IN ORDER after its motion settles. A leg that TIMED OUT fires
// none of them, because a marker that fires at the wrong place is a robot doing the wrong
// thing in the wrong spot — strictly worse than a marker that does not fire at all. (A
// `Hold` whose window simply elapses IS settled: `Chassis::hold` returns Settled when it held
// within tolerance to the end, which is a hold ending by design.)
//
// ── An unknown id is DATA, not a defect ─────────────────────────────────────────────
// `vexbuilder-integration.md` §4.2 promises VexBuilder, in writing: "an unknown id logs a
// warning and is skipped; it never crashes." So an id no handler and no built-in matches
// costs exactly one `LogLevel::Warn` line naming the id and the waypoint index, one increment
// of `RunReport::unknownIds`, and nothing else — no latched fault, no stop. A typo in a
// picker must not end a match. (An id outside the CHARSET is a different thing and never
// reaches here: `Route::valid()` refuses it, because no registry could ever hold it.)
//
// ── Blocking, single-task, and the guard is the pacer ───────────────────────────────
// `run()` blocks exactly as the facade verbs it calls do, on the caller's task. It owns no
// loop and no deadline: a route that must end with the match is wrapped in a
// `sequence::RunGuard`, whose pacer cuts the active motion and whose end action parks the
// robot — the runner needs nothing special for that to work, because the Chassis was
// constructed with the guard as its pacer.
//
// NO HARDWARE CLAIM. A Route has executed on the A2 host plant only. The library's motion
// stack has never driven a physical robot.
//
// FROZEN: F8 (docs/roadmap.md), pinned by test/f8_signature_pin_test.cpp.
//
// PROS-free, sim-free, allocation-free; no std::string, no std::function.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string_view>

#include "shulib/chassis/chassis.hpp"
#include "shulib/config/route.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::runner {

/// Why a route stopped early. `None` = it ran to the end. Append-only, like every verdict
/// vocabulary in this project: a new stop cause arrives as a new enumerator, never as a
/// re-meaning of an existing one, because log lines carry these spellings.
enum class RunStopCause : std::uint8_t {
    /// Every waypoint ran and every marker succeeded.
    None = 0,
    /// A leg exited non-Settled (`RunReport::lastExit` carries which).
    MotionFailed = 1,
    /// A marker's handler — registered or built-in — returned an outcome other than
    /// `MechanismOutcome::Succeeded`.
    MarkerFailed = 2,
};

/// The stop cause as a word, for banners and reports. "UNKNOWN" for a byte outside the enum.
[[nodiscard]] inline const char* runStopCauseWord(RunStopCause c) noexcept {
    switch (c) {
        case RunStopCause::None: return "None";
        case RunStopCause::MotionFailed: return "MotionFailed";
        case RunStopCause::MarkerFailed: return "MarkerFailed";
    }
    return "UNKNOWN";
}

/// What one route run did — the whole-run verdict.
///
/// A bare `ExitReason` would lose WHERE the route broke and HOW MUCH of it never ran, which is
/// exactly `RoutineResult`'s and `TrajectoryResult`'s argument one tier up. The counts are here
/// so that "the auton just did not score" has an answer other than a guess.
struct RunReport {
    /// The 0-BASED index of the waypoint the route stopped at; **-1 when nothing stopped it**.
    /// Zero-based with a -1 sentinel, deliberately unlike `RoutineResult::stoppedAt` (1-based,
    /// 0 = none): waypoint 0 is a real, addressable waypoint here, and the number must match the
    /// index printed in this run's own log lines and drawn in the path editor. Re-basing it
    /// would make a report and the transcript describing it disagree.
    int stoppedAt = -1;
    /// What kind of thing stopped the route. Read this BEFORE `lastExit` — only `MotionFailed`
    /// puts a failing motion verdict there.
    RunStopCause reason = RunStopCause::None;
    /// Waypoints whose verb actually executed, including the one that failed.
    int legsRun = 0;
    /// Waypoints never attempted because the route stopped before them.
    int legsSkipped = 0;
    /// Markers whose handler RAN — including the one that failed, and including the built-ins.
    /// An unknown id is not counted here (nothing ran); see `unknownIds`.
    int markersFired = 0;
    /// Markers whose id matched no built-in and no registration: warned, skipped, counted. A
    /// non-zero count on a route that "worked" is the single most useful number in this struct.
    int unknownIds = 0;
    /// The most recent MOTION leg's verdict. `Running` means no motion leg has run yet (a route
    /// of a lone `Start`, or an empty one) — the project's "no verdict here" convention, the
    /// same one `Routine::lastTrajectory()` uses.
    control::ExitReason lastExit = control::ExitReason::Running;

    /// True when the route ran to the end with every marker succeeding.
    [[nodiscard]] bool ok() const noexcept { return reason == RunStopCause::None; }
};

/// The engine that executes a `config::Route` through the frozen `Chassis` verbs, firing each
/// waypoint's markers through a `CommandRegistry` once the leg settles (file banner).
///
///     runner::CommandRegistry registry;
///     registry.on("intake_in", intakeIn);
///     runner::PathRunner runner{chassis, registry, telemetry};
///     const runner::RunReport report = runner.run(*routes.route("left_side_rush"));
///
/// Borrows all three (they must outlive it) and owns nothing. Non-copyable and non-movable for
/// the same reason `Routine` is: it holds references, and a second handle would be a second
/// opinion about a run that has one.
class PathRunner {
public:
    /// Borrows `chassis` (the verbs), `registry` (the id → action table) and `telemetry` (where
    /// the run's narrative goes). All three must outlive this object. The registry is `const`:
    /// running a route never registers anything.
    explicit PathRunner(chassis::Chassis& chassis, const CommandRegistry& registry,
                        hal::ITelemetrySink& telemetry) noexcept
        : chassis_{&chassis}, registry_{&registry}, telemetry_{&telemetry} {}

    /// Non-copyable, non-movable (class comment).
    PathRunner(const PathRunner&) = delete;
    PathRunner& operator=(const PathRunner&) = delete;
    PathRunner(PathRunner&&) = delete;
    PathRunner& operator=(PathRunner&&) = delete;
    ~PathRunner() = default;

    /// Execute `route`, blocking until it finishes or stops, and report what happened.
    ///
    /// Precondition: `route.valid()` — the refusal message is `valid()`'s own, so a bad route
    /// names the waypoint and the field rather than failing somewhere downstream. Validating at
    /// the door rather than per leg is `followTrajectory`'s rule: a contradiction at waypoint 7
    /// must not let waypoints 0–6 drive first.
    ///
    /// The mapping from waypoint to verb, in full:
    ///   * `Start`      — `setPose(x, y, heading)`; no motion, no verdict. Legal at index 0 only.
    ///   * `MoveToPose` — `moveTo({x, y, heading})`. With `HeadingMode::Free` the commanded
    ///     heading is the ESTIMATE'S heading read at the leg's start ("keep what you have";
    ///     ruling R6) — `moveTo` always commands a heading, so there is no "don't care" to pass.
    ///   * `DriveTo`    — `moveTo({x, y, bearingTo(x, y)})`, arriving facing the point, exactly
    ///     `Routine::driveTo`. `reverse` adds π, so the robot arrives BACKWARDS onto the point.
    ///   * `TurnTo`     — `turnTo(heading)`; `x` and `y` are not read.
    ///   * `StrafeTo`   — `strafeTo(x, y)`. Nothing here special-cases the drivetrain: a tank
    ///     reports strafe authority 0 and the facade's own honest fallback applies.
    ///   * `Hold`       — `hold(constraints.timeout)`; for that verb the timeout IS the duration.
    /// Each waypoint's `Constraints` become the leg's `MotionOptions` field for field (0 = the
    /// robot's configured default, the same sentinel both types already use).
    [[nodiscard]] RunReport run(const config::Route& route) {
        char why[192] = "";
        if (!route.valid(why, sizeof why)) {
            char msg[256];
            std::snprintf(msg, sizeof msg, "PathRunner::run: %s", why);
            SHULIB_PRECONDITION(false, msg);
        }
        RunReport report;
        for (std::size_t i = 0; i < route.count; ++i) {
            const config::Waypoint& w = route.waypoints[i];
            const control::ExitReason exit = runLeg(w, report);
            ++report.legsRun;
            if (exit != control::ExitReason::Settled) {
                stop(route, i, RunStopCause::MotionFailed, report, exitWord(exit));
                return report;
            }
            if (!fireMarkers(route, i, w, report)) {
                return report;
            }
        }
        logFinished(route, report);
        return report;
    }

private:
    /// The per-leg dispatch (the table in `run()`'s comment). Returns the leg's verdict;
    /// a `Start` has none and reports `Settled`, since nothing about it can fail.
    [[nodiscard]] control::ExitReason runLeg(const config::Waypoint& w, RunReport& report) {
        const chassis::MotionOptions options = toOptions(w.constraints);
        control::ExitReason exit = control::ExitReason::Settled;
        switch (w.motion) {
            case config::WaypointMotion::Start:
                chassis_->setPose(math::Pose2d{w.x, w.y, w.heading});
                return exit;  // no motion ran: `report.lastExit` must not move
            case config::WaypointMotion::MoveToPose: {
                const math::Angle heading = (w.headingMode == config::HeadingMode::Free)
                                                ? chassis_->pose().heading()
                                                : w.heading;
                exit = chassis_->moveTo(math::Pose2d{w.x, w.y, heading}, options);
                break;
            }
            case config::WaypointMotion::DriveTo: {
                math::Angle bearing = bearingTo(w.x, w.y);
                if (w.reverse) {
                    // Arrive BACKWARDS onto the point: the approach line is the same, the robot
                    // is turned around on it. Angle re-wraps, so +π is the whole operation.
                    bearing = bearing + math::Angle::radians(math::Angle::kPi);
                }
                exit = chassis_->moveTo(math::Pose2d{w.x, w.y, bearing}, options);
                break;
            }
            case config::WaypointMotion::TurnTo:
                exit = chassis_->turnTo(w.heading, options);
                break;
            case config::WaypointMotion::StrafeTo:
                exit = chassis_->strafeTo(w.x, w.y, options);
                break;
            case config::WaypointMotion::Hold:
                exit = chassis_->hold(w.constraints.timeout, options);
                break;
        }
        report.lastExit = exit;
        return exit;
    }

    /// Fire one waypoint's markers in order, after the leg settled (ruling R2). Returns false
    /// when a marker stopped the route (the report is already complete then).
    [[nodiscard]] bool fireMarkers(const config::Route& route, std::size_t index,
                                   const config::Waypoint& w, RunReport& report) {
        for (std::size_t m = 0; m < w.markerCount; ++m) {
            const config::Marker& marker = w.markers[m];
            const std::string_view id = config::markerIdView(marker);
            manipulation::MechanismOutcome outcome = manipulation::MechanismOutcome::Succeeded;
            if (const config::BuiltinCommand* builtin = config::findBuiltinCommand(id)) {
                outcome = runBuiltin(*builtin, route, index, marker);
            } else if (const Handler* handler = registry_->find(id)) {
                outcome = handler->fn(handler->ctx, marker);
            } else {
                warnUnknownId(route, index, id);
                ++report.unknownIds;
                continue;  // the guarantee: warned, skipped, the route CONTINUES
            }
            ++report.markersFired;
            if (outcome != manipulation::MechanismOutcome::Succeeded) {
                stop(route, index, RunStopCause::MarkerFailed, report,
                     manipulation::mechanismOutcomeName(outcome), id);
                return false;
            }
        }
        return true;
    }

    /// The four library-owned ids (`config::kBuiltinCommands`), performed by the runner itself.
    /// Their arg counts and their durations were already checked by `Route::valid()`, which is
    /// why this reads them without re-guarding: the data was refused at the door or it is sound.
    [[nodiscard]] manipulation::MechanismOutcome runBuiltin(const config::BuiltinCommand& builtin,
                                                            const config::Route& route,
                                                            std::size_t index,
                                                            const config::Marker& marker) {
        const std::string_view id{builtin.id};
        if (id == "wait_ms") {
            chassis_->wait(millis(marker.args[0]));
            return manipulation::MechanismOutcome::Succeeded;
        }
        if (id == "hold_ms") {
            return outcomeOf(chassis_->hold(millis(marker.args[0])));
        }
        if (id == "brake") {
            return outcomeOf(chassis_->brake());
        }
        // "log" — the non-coder's print statement, and the one built-in that cannot fail.
        diag::lineformat::Line line;
        appendWhere(line, route, index);
        line.appendLiteral(": log");
        for (std::size_t a = 0; a < marker.argCount; ++a) {
            line.appendLiteral(" ");
            appendSigned(line, marker.args[a]);
        }
        telemetry_->log(hal::LogLevel::Info, kSubsystem, line.view());
        return manipulation::MechanismOutcome::Succeeded;
    }

    /// The stop half of the error policy (file banner): record, SAFE THE DRIVE, count what will
    /// never run, Warn once. `cancel()` and not `brake()` — the safe state must not be able to
    /// fail, and `brake()` is a motion with a watchdog.
    void stop(const config::Route& route, std::size_t index, RunStopCause cause,
              RunReport& report, const char* detail, std::string_view markerId = {}) {
        report.stoppedAt = static_cast<int>(index);
        report.reason = cause;
        report.legsSkipped = static_cast<int>(route.count) - static_cast<int>(index) - 1;
        chassis_->cancel();  // 0 V + Brake — the route parks where it stopped
        diag::lineformat::Line line;
        appendWhere(line, route, index);
        line.appendLiteral(": STOPPED — ");
        if (cause == RunStopCause::MarkerFailed) {
            line.appendLiteral("marker '");
            line.appendSanitized(markerId, config::kMaxMarkerIdBytes);
            line.appendLiteral("' returned ");
        } else {
            line.appendLiteral("the leg exited ");
        }
        line.appendLiteral(detail);
        line.appendLiteral("; skipping the remaining ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(report.legsSkipped));
        line.appendLiteral(" waypoint(s); drive safed");
        telemetry_->log(hal::LogLevel::Warn, kSubsystem, line.view());
    }

    /// The unknown-id line — ONE Warn, naming the id and the waypoint, and nothing else.
    /// Deliberately not a latched fault: `diag::FaultCode` is for a robot that is UNWELL, and a
    /// route naming a capability this robot was not given is a strategy fact, not a pathology.
    void warnUnknownId(const config::Route& route, std::size_t index, std::string_view id) {
        diag::lineformat::Line line;
        appendWhere(line, route, index);
        line.appendLiteral(": no handler for command id '");
        line.appendSanitized(id, config::kMaxMarkerIdBytes);
        line.appendLiteral("' — skipped; the route continues");
        telemetry_->log(hal::LogLevel::Warn, kSubsystem, line.view());
    }

    /// The end-of-run Info line: what ran, what fired, what nobody knew.
    void logFinished(const config::Route& route, const RunReport& report) {
        diag::lineformat::Line line;
        line.appendLiteral("route '");
        line.appendSanitized(routeName(route), sizeof route.name);
        line.appendLiteral("' finished: ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(report.legsRun));
        line.appendLiteral(" waypoint(s), ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(report.markersFired));
        line.appendLiteral(" marker(s) fired, ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(report.unknownIds));
        line.appendLiteral(" unknown id(s)");
        telemetry_->log(hal::LogLevel::Info, kSubsystem, line.view());
    }

    /// "route 'name' waypoint N" — the prefix every line in this file shares, so a transcript
    /// can be grepped by route or by index.
    static void appendWhere(diag::lineformat::Line& line, const config::Route& route,
                            std::size_t index) {
        line.appendLiteral("route '");
        line.appendSanitized(routeName(route), sizeof route.name);
        line.appendLiteral("' waypoint ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(index));
    }

    /// The route's name, bounded by its buffer (a generator may fill the last byte).
    [[nodiscard]] static std::string_view routeName(const config::Route& route) noexcept {
        return config::commandIdView(route.name, sizeof route.name);
    }

    /// A signed typed argument, for the `log` built-in's line.
    static void appendSigned(diag::lineformat::Line& line, std::int32_t v) {
        char tmp[16];
        const int len = std::snprintf(tmp, sizeof tmp, "%d", static_cast<int>(v));
        if (len > 0) {
            line.appendRaw(tmp, static_cast<std::size_t>(len));
        }
    }

    /// Bearing from the live pose estimate to field point (x, y) — `Routine::bearingTo`'s
    /// expression, reproduced character for character and PINNED equal to it by test (file
    /// banner). Rejects the degenerate "bearing to the point I am already at" on the same
    /// 1e-9 in threshold, for the same reason: that is numerical degeneracy, not intent.
    [[nodiscard]] math::Angle bearingTo(units::Length x, units::Length y) const {
        const math::Pose2d here = chassis_->pose();
        const double dx = (x - here.x()).value();
        const double dy = (y - here.y()).value();
        SHULIB_PRECONDITION(std::hypot(dx, dy) > 1e-9,
                            "PathRunner: a DriveTo waypoint is (numerically) where the robot "
                            "already is — no defined bearing");
        return math::Angle::radians(std::atan2(dy, dx));
    }

    /// A waypoint's budgets as the facade's options — field for field, because both types use
    /// 0 for "the robot's configured default" (that agreement is G1's, and it is why no
    /// translation table exists here to get wrong).
    [[nodiscard]] static chassis::MotionOptions toOptions(
        const config::Constraints& constraints) noexcept {
        return chassis::MotionOptions{.timeout = constraints.timeout,
                                      .maxLinearSpeed = constraints.maxLinear,
                                      .maxAngularSpeed = constraints.maxAngular};
    }

    /// Integer milliseconds as typed time — the unit the marker args carry (`kBuiltinCommands`
    /// documents it, and the manifest publishes it).
    [[nodiscard]] static units::Time millis(std::int32_t ms) noexcept {
        return units::Time{static_cast<double>(ms) / 1000.0};
    }

    /// A motion verdict as a mechanism outcome, for the two built-ins that run a verb. Settled is
    /// the only success; `Running` cannot be returned by a blocking verb and maps to the honest
    /// "it finished and the thing did not happen".
    [[nodiscard]] static manipulation::MechanismOutcome outcomeOf(
        control::ExitReason exit) noexcept {
        switch (exit) {
            case control::ExitReason::Settled: return manipulation::MechanismOutcome::Succeeded;
            case control::ExitReason::TimedOut: return manipulation::MechanismOutcome::TimedOut;
            case control::ExitReason::Cancelled: return manipulation::MechanismOutcome::Cancelled;
            case control::ExitReason::Running: break;
        }
        return manipulation::MechanismOutcome::Unconfirmed;
    }

    /// An exit reason as a word for the stop line. Local rather than shared: `control::` has no
    /// public speller, and minting one would widen a surface this chunk does not own.
    [[nodiscard]] static const char* exitWord(control::ExitReason exit) noexcept {
        switch (exit) {
            case control::ExitReason::Running: return "RUNNING";
            case control::ExitReason::Settled: return "SETTLED";
            case control::ExitReason::TimedOut: return "TIMED_OUT";
            case control::ExitReason::Cancelled: return "CANCELLED";
        }
        return "UNKNOWN";
    }

    /// The §18.3 subsystem tag every line from this file carries.
    static constexpr const char* kSubsystem = "RUN";

    chassis::Chassis* chassis_;
    const CommandRegistry* registry_;
    hal::ITelemetrySink* telemetry_;
};

}  // namespace shulib::runner
