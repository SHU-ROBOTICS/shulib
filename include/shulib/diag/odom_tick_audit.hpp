#pragma once
//
// OdomTickAudit — the odometry trust gate's account of ONE tick: which of its checks tripped
// and the values it tripped on (chunk DIAG1, 2026-09-26).
//
// ── Why this exists ─────────────────────────────────────────────────────────────────
// On 2026-09-25 robot two raised `fault=ODO_STUCK n=1 FIRST implausible odometry tick`
// twice in one start, and the line named neither the check that tripped nor a single value,
// so the cause could not be told from the log. That matters more than one bad line:
// ODO_STUCK is the scheduler's DEFAULT abort fault (motion_scheduler.hpp,
// MotionSchedulerConfig::abortFaultMask), so a repeat during an autonomous ENDS the drive.
// It has to be explainable from the log before the first auton, not reproduced on the bench.
//
// ── Why it lives in diag/, as a plain value ─────────────────────────────────────────
// The gate is computed inside each IOdometry implementation (localization/), and the ONLY
// place ODO_STUCK is raised is HealthMonitor (this directory). diag/ is a dependency LEAF —
// localization may include diag, never the reverse (debug_record.hpp, health_monitor.hpp) —
// so the monitor cannot name an odometry type. The account therefore travels the same way
// the verdict always did, as an OBSERVABLE: the odometry fills this struct every update(),
// the Localizer forwards it, motion::tickHealthObservables copies it into
// HealthMonitor::Observations, and the monitor renders it on the rising edge only.
//
// ── What it records, and what it does NOT decide ────────────────────────────────────
// The three flags are the gate's three sub-expressions, COPIED AS THEY STAND in each
// implementation — never re-derived — and each implementation then sets its verdict to
// tripped(). So the verdict (lastDeltaImplausible()) and this account cannot disagree: they
// are the same bits, by construction, and the existing trust-gate tests are the pin that the
// verdict did not move. One consequence of copying rather than re-deriving is documented,
// not "fixed": `travel` is `!(|a| <= max && |b| <= max)`, and a comparison with NaN is
// false, so a NaN side reports NONFINITE+TRAVEL together (its travel is not "within the
// bound", which is true). This struct decides nothing — HealthMonitor's trigger is still the
// verdict, never tripped() — so a bug here can garble a line but cannot raise or hide a
// fault.
//
// When is NONFINITE the ONLY check that trips? Written down so nobody re-derives it — DIAG1's
// first mutation run got it wrong (it called dropping `!finite` from the verdict an
// "equivalent change no test can see"), and review round 1 found the configurations that
// break that claim; the suite now pins them (DIAG1 B10, C9). The heading change is always
// finite (an Angle cannot hold NaN, and |dTheta| <= pi), and arcStep's chord factor and its
// rotation are at most 1. So once every travel value is finite and within maxTickTravel (M),
// the field delta is non-finite only through an OVERFLOW on the way to it:
//  * drive encoders: the centre is 0.5 * (dL + dR), and the SUM overflows before the
//    halving when M > DBL_MAX / 2 (both sides at 1e308 under M = DBL_MAX: +Inf, then arcStep
//    gives dx = +Inf, dy = Inf * 0 = NaN);
//  * tracking wheels: arcStep rotates (fwd, lat), whose length can reach sqrt(2) * M, so an
//    M from about DBL_MAX / sqrt(2) up overflows at a 45-degree heading; and the offset
//    correction dTheta * offset overflows once |offset| passes DBL_MAX / |dTheta| (so from
//    DBL_MAX / pi up, on a big enough turn), and is non-finite on EVERY tick for a
//    non-finite offset (0 * Inf is NaN) — TrackingWheel checks neither, only the diameter.
// With M at most DBL_MAX / 2 and offsets whose pi-multiple plus M stays under DBL_MAX / 2, a
// NONFINITE tick is always also a TRAVEL tick (a NaN or Inf travel value fails `<= M`). Every
// real robot is some 300 orders of magnitude inside that (36 in; offsets of inches), but the
// config checks ask only for a finite M > 0, so the breaking configurations are legal — and in
// them NONFINITE appears ALONE (`why=NONFINITE`), the only flag that says position FROZE.
//
// ── The rendering (the text a person reads) ─────────────────────────────────────────
//   ` why=ROT+TRAVEL dth=<n>rad h0=<n> h1=<n> L=<n>in R=<n>in`   (drive-encoder sides)
//   ` why=TRAVEL ... fwd=<n>in lat=<n>in`                         (tracking wheels)
// Tokens in the fixed order NONFINITE, ROT, TRAVEL, joined by '+'. dth, h0 and h1 are
// RADIANS (1.571 = 90 deg; the default rotation bound is pi/2 = 1.571, the travel bound
// 36 in); h0/h1 are the IMU headings the tick started and ended on. An audit with no flag
// set renders ` why=UNREPORTED` and NO values: the verdict said implausible but the source
// did not say why (a test double, or a caller that never filled the field), and printing
// the values of an account nobody gave would invent one. Every number goes through
// lineformat::appendNum at precision 3 and width 0 — the ONE formatter (NaN, +Inf and -Inf
// as fixed tokens, anything longer than 10 bytes compacted to %.3g) — never `snprintf %f`,
// which prints libc's own NaN spellings and a 300-digit value that would blow the size
// bound below. ASCII only.
//
// ── The size bound ──────────────────────────────────────────────────────────────────
// FaultLatch::raise() formats into `char buf[176]` and TRUNCATES SILENTLY (fault.hpp). Its
// prefix for this code is at most 35 bytes — `fault=ODO_STUCK` (15) + ` n=` (3) + a 10-digit
// count + ` FIRST` (6) + the separating space (1) — so the whole DETAIL may be at most
// 175 - 35 = 140 bytes: kOdomAuditDetailMaxBytes. Each <n> is at most 10 bytes (a %.3f
// rendering longer than that is compacted, and %.3g's widest is `-1.23e+308`), so the worst
// detail HealthMonitor can build — `implausible odometry tick` (25) + all three tokens (25)
// + five 10-byte numbers with their labels and units (80, tracking-wheel labels) + ` +STALL`
// (7) — is 137 bytes. The test suite pins it against adversarial values, not this sum.
//
// Stateless free function plus a plain aggregate; nothing allocates. Single-task by contract,
// like the rest of diag/.

#include <cstddef>
#include <cstdint>

#include "shulib/diag/line_format.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// Which pair of labels a rendering gives its two travel values: the drive-encoder
/// odometry's two SIDES (`L=`, `R=`) or the tracking-wheel odometry's two PODS (`fwd=`,
/// `lat=`). The same two numbers mean different things on the two robots, so the line says
/// which — a "check the tracking wheels" reading of a drive-encoder robot's line is exactly
/// the misdiagnosis this prevents.
enum class OdomTravelLabels : std::uint8_t {
    DriveSides,      ///< DriveEncoderOdometry: travelA = left side, travelB = right side
    TrackingWheels,  ///< PilonsOdometry: travelA = forward pod (RAW), travelB = lateral pod
};

/// The trust gate's account of the LAST update(): which checks tripped and the values they
/// were computed from — every tick, not sticky (a clean tick clears every flag). The flags
/// are the gate's own sub-expressions (header), so `tripped()` IS the verdict
/// lastDeltaImplausible() reports. Default-constructed = "no account": nothing tripped, every
/// value zero — what an odometry reports before its first update().
struct OdomTickAudit {
    /// The integrated field delta came out non-finite (NaN or +/-Inf), so position FROZE this
    /// tick (heading still advanced).
    bool nonFinite = false;
    /// |dTheta| exceeded maxTickRotation (strict: equality does not trip).
    bool rotation = false;
    /// A travel value's magnitude was NOT within maxTickTravel — true for an oversized value
    /// AND for a NaN one (the comparison fails; header), never at exact equality.
    bool travel = false;
    /// The tick's heading change, radians: shortest signed, in (-pi, pi].
    units::AngleDim dTheta{};
    /// The IMU heading the tick started from, radians in (-pi, pi].
    units::AngleDim headingBefore{};
    /// The IMU heading the tick ended on, radians in (-pi, pi].
    units::AngleDim headingAfter{};
    /// First travel value the gate bounded, inches: the LEFT side, or the forward pod's RAW
    /// travel (before offset correction — what the gate actually compares).
    units::Length travelA{};
    /// Second travel value the gate bounded, inches: the RIGHT side, or the lateral pod's.
    units::Length travelB{};
    /// Which labels the two travel values render under.
    OdomTravelLabels labels = OdomTravelLabels::DriveSides;

    /// True iff any check tripped — by construction the verdict lastDeltaImplausible()
    /// reports for the same tick (each implementation sets its verdict FROM this).
    [[nodiscard]] constexpr bool tripped() const noexcept {
        return nonFinite || rotation || travel;
    }
};

/// The most bytes an ODO_STUCK detail may carry and still reach the log whole: FaultLatch's
/// 176-byte line buffer, less its NUL and the 35-byte worst-case prefix (header).
inline constexpr std::size_t kOdomAuditDetailMaxBytes = 140;

/// Decimal places every audit number renders with — enough to tell 1.571 (pi/2, the rotation
/// bound) from 1.570, and 36.000 in (the travel bound) from 36.001.
inline constexpr int kOdomAuditPrecision = 3;

/// Append the audit's rendering to `line` (header: the exact text): ` why=` and the tripped
/// checks' tokens, then the three angles and the two travel values under `audit.labels` — or
/// ` why=UNREPORTED` alone when nothing tripped. Numbers via lineformat::appendNum only.
inline void appendOdomTickAudit(lineformat::Line& line, const OdomTickAudit& audit) {
    line.appendLiteral(" why=");
    if (!audit.tripped()) {
        line.appendLiteral("UNREPORTED");
        return;
    }
    bool first = true;
    const auto token = [&](bool set, const char* name) {
        if (!set) {
            return;
        }
        if (!first) {
            line.appendLiteral("+");
        }
        line.appendLiteral(name);
        first = false;
    };
    token(audit.nonFinite, "NONFINITE");
    token(audit.rotation, "ROT");
    token(audit.travel, "TRAVEL");

    line.appendLiteral(" dth=");
    lineformat::appendNum(line, audit.dTheta.value(), 0, kOdomAuditPrecision);
    line.appendLiteral("rad h0=");
    lineformat::appendNum(line, audit.headingBefore.value(), 0, kOdomAuditPrecision);
    line.appendLiteral(" h1=");
    lineformat::appendNum(line, audit.headingAfter.value(), 0, kOdomAuditPrecision);

    const bool sides = audit.labels == OdomTravelLabels::DriveSides;
    line.appendLiteral(sides ? " L=" : " fwd=");
    lineformat::appendNum(line, audit.travelA.value(), 0, kOdomAuditPrecision);
    line.appendLiteral(sides ? "in R=" : "in lat=");
    lineformat::appendNum(line, audit.travelB.value(), 0, kOdomAuditPrecision);
    line.appendLiteral("in");
}

}  // namespace shulib::diag
