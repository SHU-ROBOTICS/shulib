#pragma once
//
// MotionResult — the per-motion result line, as data + one formatter (§18.3/§18.4;
// WS13, chunk C5). The second idea salvaged from the legacy "logging extreme" code,
// rebuilt the §18 way: STRUCTURED FIELDS a formatter renders, never an essay
// emitted from a motion loop.
//
// Target shape (§18.3, byte-pinned by test/motion_result_test.cpp):
//
//   [t=  12.50] [MOT] MoveToPose#7 ✓SETTLED final(  24.1,  36.0,  90.1°) over  0.20" drift  0.1°   1.16s
//
// The line rides log() as [MOT]-tagged Info text (the FaultLatch precedent), so it
// lands in the tick stream exactly where the motion ended, in exact §18.3 shape.
//
// ── The outcome vocabulary is §18.4's, verbatim ─────────────────────────────────────
// §18.4 names the motion exit-reason codes: SETTLED / TIMEOUT / CANCELLED /
// FAULT_ABORT / SUPERSEDED. control::ExitReason deliberately carries only the first
// three verbs plus Running — the scheduler's BOUNDARY knows more than the motion
// does (a Cancelled exit whose cause was the fault policy is FAULT_ABORT; one whose
// cause was pre-emption is SUPERSEDED). This enum is that boundary vocabulary,
// wire-stable like FaultCode's (explicit values, append-only, pinned by test).
// diag/ cannot name motion-layer types (dependency leaf), which is exactly why the
// RESULT vocabulary — not the motion vocabulary — lives here.
//
// ── Numbers must be TRUE (the brief's constraint 3) ─────────────────────────────────
// hasPathData says whether the record stream actually flowed for this motion (it
// does not with NullSink, and a motion cancelled in the boot window never had a
// live estimate). When false, over/drift render "n/a" — the line NEVER fabricates
// a 0.00 it has no data behind. finalPose is always real (the scheduler reads the
// estimate at the boundary). The reported values are the ESTIMATE's story; the C5
// tests bound estimate-vs-truth divergence across all three drivetrains so "what
// the motion believed" provably tracks "what the motion did".
//
// Semantics of the two derived quantities (computed by the scheduler's stats sink):
//   * over (inches): how far the robot pushed PAST its target — max projection of
//     (measured − target) onto the start→target direction, floored at 0. For a
//     stationary-target motion (turn/hold: |target − start| < ~0.1"), the direction
//     is undefined, so it degrades to max |measured − target| (worst wander from
//     the point) — the honest analogue.
//   * drift (degrees): |final heading error| — how far the heading ended from the
//     target heading (the §18.3 sample's "drift 0.1°").

#include <cstdint>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// §18.4's motion exit-reason codes, at the BOUNDARY level (header note).
/// WIRE-STABLE: explicit values, append-only, pinned by test.
enum class MotionOutcome : std::uint8_t {
    Settled = 0,     ///< arrived within tolerances — the ✓ case
    TimedOut = 1,    ///< the watchdog fired first
    Cancelled = 2,   ///< stopped by the caller (user cancel / panic stop)
    FaultAbort = 3,  ///< the scheduler's fault policy forced the stop (causal code attached)
    Superseded = 4,  ///< pre-empted: a newer motion took the slot
};

/// §18.4 spelling for the line. Never null; out-of-range renders, never crashes.
[[nodiscard]] constexpr const char* motionOutcomeName(MotionOutcome outcome) noexcept {
    switch (outcome) {
        case MotionOutcome::Settled: return "SETTLED";
        case MotionOutcome::TimedOut: return "TIMEOUT";
        case MotionOutcome::Cancelled: return "CANCELLED";
        case MotionOutcome::FaultAbort: return "FAULT_ABORT";
        case MotionOutcome::Superseded: return "SUPERSEDED";
    }
    return "UNKNOWN";
}

/// One finished motion's result, as the boundary saw it (a value type; the
/// motion-layer glue builds it from CompletedMotion — motion/run_reporter.hpp).
struct MotionResult {
    std::uint32_t id = 0;            ///< the command id it ran under
    std::string_view name{};         ///< IMotion::name() (stable literal)
    /// How the motion ended. Drives the glanceable pass/fail column — only Settled renders ✓ —
    /// and decides whether `abortFault` is meaningful (it is rendered iff this is FaultAbort).
    /// NOTE the default: an unpopulated record reads as a success.
    MotionOutcome outcome = MotionOutcome::Settled;
    FaultCode abortFault = FaultCode::None;  ///< causal code iff FaultAbort
    units::Time duration{};          ///< end − start
    bool hasPathData = false;        ///< record stream flowed (header note)
    math::Pose2d finalPose{};        ///< estimate at the boundary (always real)
    units::Length overshoot{};       ///< see header; valid iff hasPathData
    units::AngleDim drift{};         ///< |final heading error|; valid iff hasPathData
};

/// Format + log the §18.3 result line (one [MOT] Info line; byte shape pinned by
/// test). ✓ marks SETTLED; every other outcome is ✗ — a glanceable pass/fail
/// column. FAULT_ABORT carries its causal code: "✗FAULT_ABORT=ODO_STUCK".
inline void emitResultLine(hal::ITelemetrySink& sink, const MotionResult& r) {
    constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
    constexpr std::size_t kMaxNameBytes = 24;  // stable literals; bound enforced anyway
    lineformat::Line line;
    line.appendSanitized(r.name, kMaxNameBytes);
    line.appendLiteral("#");
    lineformat::appendUnsigned(line, r.id);
    line.appendLiteral(r.outcome == MotionOutcome::Settled ? " ✓" : " ✗");
    line.appendLiteral(motionOutcomeName(r.outcome));
    if (r.outcome == MotionOutcome::FaultAbort) {
        line.appendLiteral("=");
        line.appendLiteral(faultCodeName(r.abortFault));
    }
    line.appendLiteral(" final(");
    lineformat::appendNum(line, r.finalPose.x().value(), 6, 1);
    line.appendLiteral(",");
    lineformat::appendNum(line, r.finalPose.y().value(), 6, 1);
    line.appendLiteral(",");
    lineformat::appendNum(line, r.finalPose.heading().degrees(), 6, 1);
    line.appendLiteral("°) over ");
    if (r.hasPathData) {
        lineformat::appendNum(line, r.overshoot.value(), 5, 2);
        line.appendLiteral("\" drift ");
        lineformat::appendNum(line, r.drift.value() * kRadToDeg, 4, 1);
        line.appendLiteral("°");
    } else {
        lineformat::appendPadded(line, "n/a", 5);
        line.appendLiteral("  drift ");
        lineformat::appendPadded(line, "n/a", 4);
        line.appendLiteral(" ");
    }
    line.appendLiteral(" ");
    lineformat::appendNum(line, r.duration.value(), 6, 2);
    line.appendLiteral("s");
    sink.log(hal::LogLevel::Info, "MOT", line.view());
}

}  // namespace shulib::diag
