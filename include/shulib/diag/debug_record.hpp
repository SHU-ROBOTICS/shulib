#pragma once
//
// DebugRecord — the per-tick snapshot schema (master plan §18.2; WS13, chunk A1).
//
// ONE record, MANY sinks (§18.1): this struct is the single source of truth every sink
// merely FORMATS — TermSink pretty-prints it (A1), SdSink writes it to the blackbox (E1),
// Shul2Sink serializes it onto the wire (H1). Because every sink shares this one schema,
// bench / terminal / field / sim traces are directly comparable.
//
// ── FREEZE REGISTER NOTE (read before touching a field) ─────────────────────────────
// F9 (frozen at chunk H1) is the wire serialization of EXACTLY this record. From A1
// onward, every field change carries the cost of reshaping that wire, every sink, the
// SdSink blackbox layout, and the VexBuilder overlay AT ONCE. That is why the COMPLETE
// §18.2 field set is defined NOW — including fields for systems that do not exist yet
// (gating residuals at E2/E3, covariance trace at E4, strafe fallback at C3). Later
// chunks POPULATE fields; they must never RESHAPE them. Additions are append-only.
// ────────────────────────────────────────────────────────────────────────────────────
//
// Populated-when: each field notes the chunk that first writes it. Until then it holds
// its documented default — a default-constructed record is a valid "quiet" record (no
// command, no fault, zero quality), pinned by test/debug_record_test.cpp.
//
// Typed units by design (F3): every field with a dimension is a units::Quantity /
// math value type, so a milliseconds-vs-seconds or degrees-vs-radians confusion cannot
// enter the record. Angular ERRORS/deltas are units::AngleDim (radians, non-wrapping —
// a difference is not a heading); absolute headings are math::Angle (wrapping).
//
// Fixed capacity, no heap: the per-wheel arrays reuse kinematics::WheelSpeeds::kMaxWheels
// so the two capacities can never diverge; the record is a flat value type produced every
// ~10ms tick, so it must never allocate (see hal::emitRecord for how a NullSink build
// skips even POPULATING it).

#include <array>
#include <cstdint>

#include "shulib/diag/fault.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// Why the fusion gate accepted/rejected this tick's correction (§18.2 "gating reason").
/// WIRE-STABLE: explicit values, append-only, pinned by test (these go on the F9 wire).
/// A1 defines the vocabulary; E2/E3 (correctors) and E4 (EKF) populate it.
enum class GateReason : std::uint8_t {
    None = 0,                 ///< no correction proposal this tick (pure dead-reckoning)
    Accepted = 1,             ///< a proposal passed the gate and was (nudge-)applied
    RejectedInnovation = 2,   ///< outside the innovation bound (complementary tier)
    RejectedMahalanobis = 3,  ///< failed the Mahalanobis gate (EKF tier, E4)
    RejectedNoFix = 4,        ///< source had no usable fix (off-strip GPS / no tag, E2/E3)
    RejectedHighYawRate = 5,  ///< spinning too fast to trust the fix (E2)
    /// Failed the complementary tier's NORMALIZED-INNOVATION gate: |residual| exceeded
    /// `gateSigma` times the fix's own 1σ (measurement σ from the device's reported error,
    /// widened by how far the estimate has dead-reckoned since its last fix). Appended at
    /// E2 rather than reusing `RejectedMahalanobis`, which needs a filter-estimated
    /// covariance the complementary tier does not have — see gps_corrector.hpp. (E2)
    RejectedNormalizedInnovation = 6,
    /// The source re-reported a sample it has already folded, so there is no new
    /// information this tick. The V5 GPS camera produces a fix every ~50 ms while the
    /// control loop runs at ~100 Hz, so a corrector that folds every read counts one
    /// measurement five times. (E2)
    RejectedStaleFix = 7,
    /// The source claims a fix but reports a self-error too large to be worth folding —
    /// a sensor saying "I can see, badly" rather than "I cannot see". (E2) — reused at E3
    /// for a tag detection below the confidence floor, which is the same statement.
    RejectedSensorQuality = 8,
    /// A tag was SEEN but the tag map does not know where it is, so no absolute pose can be
    /// derived from it. Distinct from RejectedNoFix on purpose: this is a CONFIGURATION error
    /// the team can fix (an id missing from the map, or an empty map), not the field being the
    /// field, and it is the one worth shouting about. (E3)
    RejectedNoTagMapEntry = 9,
    /// Every visible tag was outside the corrector's trusted range band — too close to fit in
    /// the frame, or far enough that planar-PnP's heading ambiguity makes the orientation
    /// untrustworthy (localization/apriltag_corrector.hpp). (E3)
    RejectedTagRange = 10,
    /// The newest vision frame is older than the corrector's freshness horizon: the vision task
    /// has stalled, died, or was never started. Distinct from RejectedStaleFix (a frame already
    /// folded — the normal steady state) and from RejectedNoFix (looked, saw nothing), because
    /// "the camera stopped talking" calls for a different response than either. (E3)
    RejectedObservationAge = 11,
    /// The EKF tier gave up on its own confidence and re-initialised its covariance: N
    /// consecutive Mahalanobis rejections with a persistently large innovation means the
    /// filter's belief about how wrong it might be is itself wrong. **The estimate is NOT
    /// moved** — only the uncertainty is reset — so §13 #4's never-snap bound still holds on
    /// this tick and every tick after it (E4's T2 ruling, localization/ekf_fusion.hpp).
    /// It is a WORD in the record rather than an inference because an estimator that quietly
    /// changes its mind about how much to trust the world is the hardest kind of run to debug;
    /// `covarianceTrace` jumping on the same tick is the independent numeric witness. (E4)
    CovarianceReinit = 12,
};

/// Index vocabulary for DebugRecord::tickPhase — WHO consumed the loop budget this
/// tick (diagnostics-plan D-3: LoopMonitor detects an overrun but cannot attribute
/// it; these slots turn "the loop is slow" into a name).
/// WIRE-STABLE: explicit values, append-only, pinned by test (F9 serializes the
/// array these index). Defined at C5 — BEFORE the H1 freeze — precisely so the
/// slots exist even where the producer does not yet (the one genuinely
/// time-sensitive act in diagnostics-plan.md).
enum class TickPhase : std::uint8_t {
    Localization = 0,  ///< Localizer::update() — producer: C5 (MotionScheduler)
    Motion = 1,        ///< the active motion's tick / the idle work — producer: C5
    Health = 2,        ///< health observables, where separable — RESERVED (E1+)
    Telemetry = 3,     ///< sink formatting/IO, where separable — RESERVED (E1+)
    Scheduler = 4,     ///< scheduler bookkeeping, where separable — RESERVED (E1+)
    User = 5,          ///< caller-owned work (G2 markers, mechanisms) — RESERVED, still
                       ///< no producer. F1 RULED why it stayed empty rather than filling
                       ///< it: the only place caller work is visible today is a waitUntil
                       ///< predicate, which runs OUTSIDE the attribution bracket, and
                       ///< crediting it would break the pinned sum contract (attributed
                       ///< phases never exceed the tick total — tick_attribution.hpp).
                       ///< The named producer is F2's sequencer loop / G2's marker
                       ///< dispatch, which own a loop and can bracket user work properly.
};

/// Capacity of DebugRecord::tickPhase. STRICTLY GREATER than the defined phases on
/// purpose: slots 6..7 are spare, reserved before the F9 freeze so a new phase is
/// a vocabulary append, not a wire reshape. Pinned by test.
inline constexpr int kTickPhaseSlots = 8;

/// The per-tick snapshot (§18.2), captured each control tick and rate-budgeted by the
/// producer. Plain struct on purpose: it is a snapshot, not an invariant-bearing type —
/// the invariants live in the systems that populate it.
struct DebugRecord {
    /// Per-wheel capacity, tied to the kinematics contract so they can never diverge.
    static constexpr int kMaxWheels = kinematics::WheelSpeeds::kMaxWheels;

    // ── timing ──────────────────────────────────────────────────────────────────────
    units::Time t{};   ///< seconds since the run epoch (the [t=…] stamp) — producer: C1
    units::Time dt{};  ///< this tick's measured dt — producer: C1 (via LoopMonitor)

    // ── pose & control (field frame, F1) ────────────────────────────────────────────
    math::Pose2d targetPose{};    ///< where the active motion wants the robot — C1
    math::Pose2d measuredPose{};  ///< the fused estimate (Localizer::pose()) — C1
    units::Length errorX{};       ///< target − measured, field x — C1
    units::Length errorY{};       ///< target − measured, field y — C1
    units::AngleDim errorHeading{};  ///< shortest signed heading error (radians) — C1
    math::ChassisSpeeds commanded{}; ///< commanded (vx, vy, ω) this tick — C1

    // ── per-wheel electricals ───────────────────────────────────────────────────────
    int wheelCount = 0;  ///< valid entries in the arrays below, [0, kMaxWheels] — C1
    std::array<units::Voltage, static_cast<std::size_t>(kMaxWheels)> wheelVoltage{};  ///< — C1
    std::array<units::Current, static_cast<std::size_t>(kMaxWheels)> wheelCurrent{};  ///< — C1

    // ── inertial ────────────────────────────────────────────────────────────────────
    math::Angle imuYaw{};               ///< canonical IMU heading — C1
    units::AngularVelocity imuYawRate{};  ///< canonical yaw rate — C1

    // ── active command ──────────────────────────────────────────────────────────────
    /// 0 = no active command. Ids are assigned by the motion scheduler (C2); the value
    /// is wire-stable as a plain integer regardless of what the ids come to mean.
    std::uint32_t activeCommandId = 0;
    /// Motion-layer state (run/settling/…). 0 = idle. The VOCABULARY is owned by the
    /// motion layer (C1/C2); once assigned, values are wire-stable like FaultCode's.
    std::uint8_t activeCommandState = 0;

    // ── estimator health ────────────────────────────────────────────────────────────
    bool deadReckoning = false;  ///< Localizer::isDeadReckoning() — C1
    /// Categorical quality, mirroring localization::Localizer::Quality numerically
    /// (0=Uninitialized 1=DeadReckon 2=Corrected 3=Degraded — the mapping is pinned by
    /// test so a reorder of either enum is caught). Kept as a raw byte so diag/ stays a
    /// leaf that localization/ may depend on, never the reverse. — C1
    std::uint8_t qualityClass = 0;
    double quality = 0.0;  ///< the [0,1] scalar (Localizer::quality()) — C1
    /// EKF covariance trace once E4 lands; the complementary tier may surface its scalar
    /// trust weight here until then. One slot, semantics per active fusion policy (§18.2
    /// "covariance trace / filter trust weights"). — E4
    double covarianceTrace = 0.0;

    // ── fusion gating (audits every accept/reject decision) ─────────────────────────
    units::Length gateResidualX{};          ///< innovation, field x — E2/E3
    units::Length gateResidualY{};          ///< innovation, field y — E2/E3
    units::AngleDim gateResidualHeading{};  ///< innovation, heading (radians) — E3
    double gateMahalanobis = 0.0;           ///< Mahalanobis distance of the fix — E4
    GateReason gateReason = GateReason::None;  ///< why accepted/rejected — E2/E3/E4

    // ── applied correction (audits the NEVER-SNAP invariant, §13 #4) ────────────────
    units::Length correctionDx{};           ///< net position nudge applied this tick — C1
    units::Length correctionDy{};           ///< — C1
    units::AngleDim correctionDTheta{};     ///< heading nudge (0 at M2: heading is IMU-owned) — E3
    bool clampedThisTick = false;  ///< the per-tick nudge budget bound the correction — C1
    bool strafeFallbackActive = false;  ///< H-drive turn-then-drive fallback engaged (§13 #5) — C3

    // ── fault & power ───────────────────────────────────────────────────────────────
    FaultCode fault = FaultCode::None;  ///< fault raised THIS tick (the latch keeps the first)
    units::Voltage batteryVoltage{};    ///< — C1
    units::Current batteryCurrent{};    ///< — C1

    // ── observability self-diagnostics (appended at C5, BEFORE the F9 freeze) ───────
    // D-2: throttling must never be silent. CUMULATIVE counts of what rate limiting
    // dropped since run start, stamped by RateLimitedSink onto every record it
    // FORWARDS — so a gap in the stream always carries its own explanation, on the
    // wire itself. 0 = nothing dropped. — C5 (diag/rate_limit_sink.hpp)
    std::uint32_t droppedRecords = 0;  ///< emit()-channel records dropped so far — C5
    std::uint32_t droppedLines = 0;    ///< log()-channel lines dropped so far — C5
    /// D-3: per-subsystem tick-time attribution in canonical seconds, indexed by TickPhase
    /// (top of file). Slots for phases marked RESERVED hold 0 until their producer exists;
    /// slots 6..7 are spare capacity (kTickPhaseSlots note). The values describe the most
    /// recently COMPLETED tick (the stamping sink cannot know the current tick's total
    /// mid-tick; one-tick lag, documented at the producer). All-zero when attribution is off,
    /// which is indistinguishable here from a tick that spent no time anywhere — read it with
    /// that in mind. Do NOT audit the sum against `dt`: tick_attribution.hpp's "attributed
    /// never exceeds the total" contract is qualified ON THE SAME CLOCK, and the attribution
    /// clock is injected separately from the loop clock `dt` is measured on — this record
    /// carries no attribution total of its own to compare against. Even on that one clock the
    /// relation is soft enough that TickAttribution floors its own remainder at 0, because a
    /// clock that jumps mid-phase can push the phases past the total. So read a shortfall as
    /// un-instrumented work rather than a missing phase, and read a sum above `dt` as a
    /// statement about two clocks rather than a broken record. — C5 (scheduler)
    std::array<units::Time, static_cast<std::size_t>(kTickPhaseSlots)> tickPhase{};
};

}  // namespace shulib::diag
