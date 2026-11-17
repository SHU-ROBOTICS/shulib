#pragma once
//
// chassis::resetFromWall — the wall reset's EXPLICIT call (chunk L6-A1, 2026-10-29): re-seat ONE axis
// of the estimate against a field wall from a side distance sensor, over sensor handles the caller
// passes. The form for a hand-wired Tier-3 robot and for the host tests; an auton uses the everyday
// form over its context (`competition::resetFromWall(ctx, ...)`, competition/auton_wall_reset.hpp — L6-A2),
// which runs the same steps through the same sampler below.
//
// ── Why it sits HERE, at the facade's layer ──────────────────────────────────────────────────────
// It needs the Chassis (the estimate, the scheduler's wait, the telemetry), and a localization/
// header may not include upward (the master plan's §5 — and doing so closes an include cycle through
// motion/motion.hpp, which includes the Localizer). So the pure gates live in
// localization/wall_reset.hpp and this call sits over chassis.hpp, as chassis/routine.hpp does.
//
// ── What one call does ───────────────────────────────────────────────────────────────────────────
//   1. Setup, reading NOTHING: the sensor exists (else SensorAbsent, with its absence sentence), its
//      mount was measured (MountUnset), the estimate is live (NotLive), NO motion is armed
//      (MotionActive — the sampling wait keeps an armed motion ticking, and a seat under it is the
//      master plan's risk 5, a pose jump mid-motion), the request is usable (BadRequest, naming the
//      field); then rules 2-4 from the estimate alone (NotSquare, KeepOut, BeamLow). A refusal here
//      costs no time and reads no sensor.
//   2. The window: samples inside `Chassis::waitUntil`, so the Localizer and every health check keep
//      ticking. The PREDICATE ends the wait — at the first refusing tick, once the window has enough,
//      or at `windowMax` by the clock — so the wait never reaches its own timeout and never writes
//      the scheduler's "waitUntil timed out" line.
//   3. The verdict (rules 5-9, localization::evaluateWallReset); when SEATED, ONE axis is seated
//      through `Localizer::seatAxis` — never setPose, never the heading.
//   4. ONE log line, subsystem "WALL": Info when SEATED, Warn when REFUSED.
// It commands NOTHING to the drive: a robot still coasting refuses MOVING. A refusal leaves the
// estimate bit-identical; what the auton does next is its `required` (D3) through `then()`.
//
// ── The frozen wait ──────────────────────────────────────────────────────────────────────────────
// `Chassis::waitUntil` is a wait the run guard cannot cut, so a deadline or a disable that lands
// mid-window is acted on at most `windowMax` (0.5 s by default, never more than 1 s) late. The
// everyday form (competition/auton_wall_reset.hpp) samples through the guard's deadline-aware wait instead
// and refuses CUT.
//
// ── A seat is a deliberate jump (the scheduler's pose guard) ────────────────────────────────────
// The scheduler's plausibility guard raises an advisory IMPLAUSIBLE when one tick's pose change
// exceeds 150 in/s x 1.5 x dt — 2.25 in at the 10-ms tick — and forgives a jump only on a dt-0 tick,
// which every verb and wait gives on its first tick. So a seat over 2.25 in followed by a Routine
// step raises nothing; a Tier-3 loop that ticks the scheduler itself (`scheduler().tick()`) sees ONE
// advisory IMPLAUSIBLE per such seat. Never "fix" that by clamping the seat (the report would then lie
// about what it applied) or by silencing the guard.

#include <utility>

#include "shulib/chassis/chassis.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/localization/axis.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/wall_reset.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::chassis {

/// Slack past `windowMax` given to the sampling wait's own timeout (seconds). Unreachable by
/// construction — the predicate ends the window at `windowMax` by the clock — so it is code-level
/// belt-and-braces, not an HA entry (Chassis's wait() backstop is the same idea).
inline constexpr double kWallResetWaitBackstopSeconds = 1.0;

namespace detail {

/// THE ONE SAMPLER both forms run (the explicit call below, and the everyday form in
/// competition/auton_wall_reset.hpp): the setup and pre-checks, the window through `wait`, the verdict, the
/// seat, the log line. `wait` is called once as `wait(pred, timeout)` with a predicate that returns
/// true when the window is done and a `units::Time` timeout; it returns true when the wait ENDED (the
/// predicate held), false when it was CUT (the everyday form's run guard: a deadline or a disable) — a
/// cut window refuses CUT and seats nothing. `log` (may be null) is filed with the outcome. `sink`
/// (may be null) is where the ONE line goes — the everyday form passes the auton's own
/// (`AutonContext::telemetry`); null means the chassis context's telemetry. `sideNote` (may be null:
/// none) goes inside the line's sensor parenthesis — the everyday form's MIRRORED note (MIRROR1-A;
/// localization::appendWallResetLine).
template <typename Wait>
[[nodiscard]] localization::WallResetReport runWallReset(Chassis& chassis,
                                                         const localization::WallSensor& sensor,
                                                         const localization::WallResetRequest& request,
                                                         const localization::WallResetLimits& limits,
                                                         Wait&& wait,
                                                         localization::WallResetLog* log = nullptr,
                                                         hal::ITelemetrySink* sink = nullptr,
                                                         const char* sideNote = nullptr) {
    using localization::WallResetReport;
    using localization::WallResetVerdict;
    const motion::MotionDeps& deps = chassis.deps();
    localization::Localizer& loc = *deps.localizer;

    WallResetReport rep{};
    rep.axis = request.wall.axis;
    rep.side = request.sensor;
    rep.required = request.required;
    rep.maxCorrection = request.maxCorrection;
    rep.faultChannel = sensor.faults != nullptr;
    rep.before = units::Length{localization::detail::poseOn(loc.pose(), request.wall.axis)};

    // Rule 1 — the setup, reading nothing.
    if (sensor.sensor == nullptr) {
        rep.verdict = WallResetVerdict::SensorAbsent;
        rep.detail = sensor.absence != nullptr ? sensor.absence : "";
    } else if (!sensor.mount.measured) {
        rep.verdict = WallResetVerdict::MountUnset;
    } else if (loc.qualityClass() == localization::Localizer::Quality::Uninitialized) {
        rep.verdict = WallResetVerdict::NotLive;
    } else if (chassis.scheduler().hasActiveMotion()) {
        rep.verdict = WallResetVerdict::MotionActive;
    } else {
        // Rules 1 (the request) - 4, from the estimate alone.
        const math::Pose2d start = loc.pose();
        const units::AngleDim headingTol =
            localization::headingTolerance(loc.turnedSinceHeadingSeed(), limits);
        const units::Length alongUncertainty{
            limits.alongDriftRate
            * loc.travelSinceSeat(localization::alongAxis(request.wall.axis)).value()};
        const WallResetVerdict pre =
            localization::preCheck(start, sensor.mount, request, limits, alongUncertainty);
        if (pre != WallResetVerdict::Seated) {
            rep = localization::describeWallReset(start, sensor.mount, request, limits, headingTol,
                                                  alongUncertainty);
            rep.verdict = pre;
            rep.faultChannel = sensor.faults != nullptr;
        } else {
            // The window (rules 5-6 per tick), through the caller's wait.
            hal::IClock& clock = deps.ctx->clock();
            localization::WallSampler sampler{sensor, limits};
            const auto done = [&sampler, &clock, &loc] {
                return sampler.step(clock.now(), loc.pose(), loc.twist().omega());
            };
            const bool ended = std::forward<Wait>(wait)(
                done, units::Time{limits.windowMax.value() + kWallResetWaitBackstopSeconds});
            if (!ended) {
                rep = localization::describeWallReset(loc.pose(), sensor.mount, request, limits,
                                                      headingTol, alongUncertainty);
                rep.verdict = WallResetVerdict::Cut;
                rep.detail = "";
                rep.samples = static_cast<int>(sampler.samples().size());
                rep.faultChannel = sensor.faults != nullptr;
            } else {
                // Rules 5-9, against the estimate as it stands after the window.
                rep = localization::evaluateWallReset(sampler.samples(), loc.pose(), sampler.maxTwist(),
                                                      sensor.mount, request, limits, headingTol,
                                                      alongUncertainty);
                rep.faultChannel = sensor.faults != nullptr;
                if (rep.verdict == WallResetVerdict::Seated) {
                    loc.seatAxis(request.wall.axis, rep.measured);  // ONE axis, nothing else
                }
            }
        }
    }

    diag::lineformat::Line line;
    localization::appendWallResetLine(line, rep, request, sensor.mount, limits, sideNote);
    hal::ITelemetrySink& out = sink != nullptr ? *sink : deps.ctx->telemetry();
    out.log(rep.seated() ? hal::LogLevel::Info : hal::LogLevel::Warn, "WALL", line.view());
    if (log != nullptr) {
        log->record(rep);
    }
    return rep;
}

}  // namespace detail

/// THE EXPLICIT CALL (L6-A1 — a hand-wired Tier-3 robot's form, and the host tests'): re-seat ONE axis
/// of `chassis`'s estimate against `request.wall` from `sensor`, inside `limits` — blocking, at most
/// `limits.windowMax` plus a tick; the setup and pre-checks first (no time, nothing read), then a
/// window of samples inside `chassis.waitUntil` (its predicate ENDS the wait — no "timed out" line),
/// the verdict, and, when SEATED, `Localizer::seatAxis` on that axis only. ONE log line either way
/// (subsystem "WALL"). Commands nothing to the drive. The heading tolerance and the along-wall drift
/// allowance come from the Localizer's counters (`turnedSinceHeadingSeed`, `travelSinceSeat`). The
/// window is a FROZEN wait: a deadline landing in it is acted on up to `windowMax` late (the
/// everyday form, `competition::resetFromWall(ctx, ...)`, is cut instead). Returns the report — a VALUE; `continues()` is what a
/// `then()` returns (D3). PRECONDITION (MIRROR1-A, the MIRROR1 brief's D5): no mirrored run is open on
/// `chassis` — this form is handed one SENSOR, and a mirrored auton needs the OTHER side's; the
/// everyday form swaps the wall and the side itself.
[[nodiscard]] inline localization::WallResetReport resetFromWall(Chassis& chassis,
                                                                 const localization::WallSensor& sensor,
                                                                 const localization::WallResetRequest& request,
                                                                 const localization::WallResetLimits& limits) {
    SHULIB_PRECONDITION(chassis.reflection().isIdentity(),
                        "chassis::resetFromWall: a mirrored run is open -- this form is handed one sensor and a "
                        "mirrored auton needs the other side's: call competition::resetFromWall(ctx, ...), which "
                        "swaps the wall and the sensor side");
    return detail::runWallReset(chassis, sensor, request, limits,
                                [&chassis](const auto& pred, units::Time timeout) {
                                    (void)chassis.waitUntil(pred, timeout);
                                    return true;  // the explicit form's wait is never cut
                                });
}

}  // namespace shulib::chassis
