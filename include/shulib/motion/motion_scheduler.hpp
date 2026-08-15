#pragma once
//
// MotionScheduler — the thing that actually runs a routine (chunk C2, WS6/M2).
//
// C1 made the library able to execute ONE motion; every routine test hand-rolled
// the loop. This class is that loop, formalized: exactly one active motion at a
// time, started without blocking (async), waited on (waitUntilSettled /
// waitUntil), stopped into a defined safe state (cancel), with the fault policy
// C1 explicitly deferred ("a motion raises but does not self-abort — the
// scheduler owns cancellation"). C4's Chassis facade wraps these verbs; F6
// freezes their shape at D2 — everything here is what that facade will inherit.
//
// ── Who owns the loop (the A2/C1 controller-first shape, formalized) ────────────────
// One scheduler tick is EXACTLY the loop C1's rig documented:
//
//     localizer.update();          // the estimate advances FIRST (sees time t)
//     loopMonitor.tick();          // timing pathology → LOOP_OVERRUN, visibly
//     active ? active->tick()      // the motion reads the world and commands
//            : idle work;          // no motion: HealthMonitor + an idle record
//     <the world advances to t+dt> // via the injected ITickPacer (below)
//
// The scheduler NEVER owns time. Advancing the world is the pacer's job — in
// host sim that is stepping the A2 plant; on the robot it is delaying to the
// next tick boundary. This is the same inversion as IClock: the scheduler is
// deterministic because it can only observe time, never make it.
//
// ── One active motion — structural, in two layers ───────────────────────────────────
// (1) The scheduler has ONE active slot and no queue. Starting a motion while
//     one is active PRE-EMPTS: the old motion is cancel()led — which puts the
//     drivetrain in the cancel safe state AND renders the old object inert
//     (post-cancel ticks are contractual no-ops that touch no motor) — and only
//     then is the new motion armed. There is no tick on which both command.
//     * Rejected REJECT-while-active: a routine that forgot one
//       waitUntilSettled() would silently SKIP a move — the robot does the
//       wrong thing quietly, the worst failure class.
//     * Rejected QUEUE: a queue is F2's Sequence combinator in disguise
//       (explicitly out of scope), and a latent queued motion firing seconds
//       later is scarier than last-command-wins. Pre-empt matches how a robot
//       is commanded: a NEW order supersedes the old one, safely.
// (2) The pre-empted object itself cannot re-command even if a stale caller
//     still ticks it — enforcement lives in the MOTION (motion.hpp cancel
//     contract), not in scheduler bookkeeping. Direct IMotion use that never
//     touches a scheduler remains possible at Tier 3 by design; C4's facade is
//     what closes that door for library users.
//
// ── cancel(): the defined safe state ────────────────────────────────────────────────
// Every cancel path lands in applyCancelSafeState() (motion.hpp): zero volts +
// BrakeMode::Brake, commanded SYNCHRONOUSLY inside the call — no further tick
// is required for the drivetrain to be safe (a cancel that depends on someone
// continuing to tick is a cancel that can leave motors energized). Scheduler
// cancel() with NO active motion still applies the safe state: the panic stop
// must always work. Rationale for brake-not-coast/hold: motion.hpp. Real-world
// braking efficacy is a registered hardware claim (A4: HA-53).
//
// ── The fault policy (C1's named deferral, decided here) ────────────────────────────
// After each Running tick the scheduler checks whether any fault in
// `abortFaultMask` was raised SINCE THIS MOTION STARTED (per-code raiseCount
// snapshots — a since-clear bitmask cannot see a re-raise, and a dead encoder
// that faulted in motion 1 must still abort motion 2). On a hit: the motion is
// cancelled into the safe state, one Warn line names the causal code, and the
// boundary records it (CompletedMotion.abortFault). The RUN continues — faults
// log and recover, they never crash (fault.hpp); what the next motion does
// about a fault-aborted predecessor is the caller's strategy.
//
// Default mask = ODO_STUCK only. The reasoning, per code:
//   * ODO_STUCK  → ABORT. The spin-vs-motion cross-check says the estimate is
//     LYING (dead encoder) or the robot is physically stalled. Continuing to
//     servo against a lying estimate is worse than stopping: the controller
//     integrates toward a phantom target at full authority. Brake and hand the
//     decision back to the routine.
//   * IMU_LOST   → CONTINUE. C1 pinned that Degraded does NOT gate: encoders
//     are still good, and freezing mid-run strands the robot mid-field (the
//     Localizer's own D8 choice). The watchdog + tolerances bound the damage.
//   * BROWNOUT   → CONTINUE. A power collapse zeroes effective volts at the
//     firmware; the ESTIMATE is still honest. If the pack bounces back,
//     finishing the motion is the right outcome; if not, the motion's watchdog
//     times it out. Aborting would convert a transient sag into a dead run.
//   * GPS_GATE_REJECT → CONTINUE. The gate already did its job — the lying fix
//     was rejected and the estimate is protected. Aborting on a defended attack
//     would let a flaky sensor end routines.
//   * MOTOR_OVER_TEMP / LOOP_OVERRUN → CONTINUE. Degraded authority / timing,
//     not a lying estimate; both are visible, bounded, and watchdog-contained.
//   * MOTION_TIMEOUT never needs the mask: the motion already exited TimedOut.
// The mask is configurable because this is policy, not physics — a team may
// legitimately choose abort-on-brownout for a strategy that prefers parking.
//
// ── The task-boundary catch (check.hpp's contract, discharged here) ─────────────────
// core/check.hpp promises: on-robot, a PreconditionError thrown mid-motion is
// "caught by the motion scheduler at the task boundary and converted to a
// FAULT_ABORT exit + a safe drivetrain state". This tick loop is that boundary.
// The catch is TIGHT — PreconditionError only, wrapped around active->tick()
// only (swallowing arbitrary exceptions would hide real bugs; a Localizer
// breach has no motion boundary to unwind to and propagates). If the installed
// handler already raised (the on-robot policy), no second raise happens — the
// faultCount snapshot disambiguates. One bad reading degrades one motion to a
// fault code; it never aborts the auton.
//
// ── Nothing may hang ────────────────────────────────────────────────────────────────
//   * waitUntilSettled() is bounded by the MOTION's watchdog — C1 proved (and
//     mutation-guarded) that no motion can fail to exit. No second timeout.
//   * waitUntil(pred, timeout) REQUIRES an explicit finite timeout — every
//     bound is caller-visible, and there is no invented default constant. The
//     return distinguishes Satisfied from TimedOut; a timeout logs one Warn
//     line but raises NO fault (a timed-out wait is a legitimate strategy
//     branch — "wait for the ring, else move on" — not a pathology).
//   * The predicate is checked BEFORE the first tick: true-on-entry returns
//     Satisfied having ticked zero times; timeout 0 is an honest poll.
//   * A broken pacer that never advances the clock would hang EVERY bound —
//     watchdogs read the same frozen clock — so the scheduler counts
//     consecutive non-advancing paces and fails the kMaxStalledPaces
//     precondition loudly instead of spinning forever.
//
// ── Re-entrancy (decided and pinned) ────────────────────────────────────────────────
//   * async()/cancel() from inside a waitUntil predicate: ALLOWED. Pre-emption
//     applies normally; the swap happens between ticks, so the single-command
//     invariant holds, and the wait simply continues over the new state.
//   * cancel() from inside the pacer's pace(): ALLOWED, and pinned since F2 —
//     pace() runs between ticks (inTick_/inBoundary_ both false), so the
//     precondition set always admitted it; it was undocumented and untested
//     until F2's end-of-run guard came to RELY on it (the deadline cut: a
//     cancel from pace() unwinds waitUntilSettled on the same iteration, with
//     zero latency — measured). Pinned by test so a later chunk cannot break
//     the guard by tightening this precondition. async() from pace() is
//     equally precondition-legal and measured to WORK — and is REJECTED as a
//     pattern: the hijacked wait keeps looping over the new motion and returns
//     ITS verdict as the original caller's (a moveTo that was cut reports
//     Settled describing a motion the caller never issued, with zero log
//     lines). The guard never uses it; nothing should.
//   * A BLOCKING verb (waitUntil / waitUntilSettled / tick) from inside a
//     predicate — or from pace(): REJECTED by precondition — disguised
//     recursion whose depth is user-data-dependent. Nothing in the G2 marker
//     use case needs it; relaxing later is additive, un-forbidding is not.
//   * async()/cancel() from inside a motion's tick(): REJECTED by precondition
//     — mutating the active slot while active->tick() is on the stack.
//
// ── Unwind safety of the blocking waits (F2, closing C4's known gap at its root) ───
// A throw through a blocking wait (stalled-pace precondition, a Localizer
// breach, a throwing predicate) used to leave the active motion ARMED and the
// motors at their last command: C4 documented the consequence for
// waitUntilSettled (a verb's stack-owned motion dangling in the slot) and
// patched it in the FACADE with runBlocking's DetachGuard — but the same hole
// was open for direct Tier-3 waits and for waitUntil, where F2 measured the
// worst state in the campaign: 11.4 V under Coast with the slot pointing at a
// dying stack object (Chassis::waitUntil is a bare pass-through). The fix now
// lives HERE, in the loop owner: both waits cancel on unwind (safe state +
// boundary recorded + slot cleared) before the exception propagates. The
// facade's DetachGuard stays — redundant cancels are idempotent, and belt
// plus braces is the right dress code for the panic path.
//
// ── Observability (what C5 will need; A1's cost contract respected) ─────────────────
// The scheduler assigns activeCommandId (debug_record.hpp: "ids are assigned by
// the motion scheduler") by interposing CommandIdStampSink between the motions
// and the real sink: every record emitted while a motion is active is stamped
// with its id — unforgettably, for every motion type, including future ones.
// Construct motions from scheduler.deps() so their records route through the
// stamp (the C4 facade will make this plumbing automatic; flagged for F6). The
// stamper forwards wantsRecord() — the pair rule — so a NullSink run still
// skips record population entirely. Idle ticks emit a quiet record (no
// invented target) so the stream stays continuous between motions; motion
// boundaries surface as CompletedMotion + per-exit counters, NOT as formatted
// result lines — that formatting is C5's, deliberately not built here.
//
// Single-task by contract, like everything it composes. Not copyable/movable:
// it holds a self-referential context (the stamped telemetry route).

#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/loop_monitor.hpp"
#include "shulib/diag/plausibility_guard.hpp"
#include "shulib/diag/tick_attribution.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// The seam through which the WORLD advances between scheduler ticks (header:
/// "who owns the loop"). Host sim: step the A2 plant by the tick dt. Robot:
/// delay to the next tick boundary. pace() MUST eventually advance
/// IClock::now() — every bounded wait depends on time actually passing; a
/// pacer that never advances the clock trips the scheduler's stalled-pace
/// precondition (loudly) rather than hanging.
class ITickPacer {
public:
    /// Interface boilerplate: a public virtual destructor, with the copy/move set
    /// defaulted back in because declaring a destructor suppresses the implicit MOVE
    /// constructor and move assignment (the implicit copies survive, merely deprecated —
    /// spelling all five keeps the intent explicit rather than inherited). The scheduler
    /// holds a pacer by REFERENCE and never copies, moves or destroys one — the pacer is
    /// caller-owned and must outlive the scheduler.
    virtual ~ITickPacer() = default;
    ITickPacer() = default;
    ITickPacer(const ITickPacer&) = default;
    ITickPacer(ITickPacer&&) = default;
    ITickPacer& operator=(const ITickPacer&) = default;
    ITickPacer& operator=(ITickPacer&&) = default;

    /// Advance the world to the next control-tick instant.
    virtual void pace() = 0;
};

/// The outcome of waitUntil — a DISTINCT vocabulary from ExitReason on
/// purpose: a predicate satisfying is not a motion settling, and conflating
/// them would let "the wait timed out" read as "the motion timed out".
enum class WaitResult {
    Satisfied,  ///< the predicate became true (possibly true on entry)
    TimedOut,   ///< the timeout elapsed first — the predicate never held
};

/// One bit per FaultCode value, for MotionSchedulerConfig::abortFaultMask.
[[nodiscard]] constexpr std::uint32_t faultBit(diag::FaultCode code) noexcept {
    return 1U << static_cast<unsigned>(code);
}

/// Scheduler policy, COPIED at construction — mutating the caller's struct afterwards
/// changes nothing about a live scheduler. The defaults are the competition posture:
/// abort a motion only when the estimate is lying (ODO_STUCK), tick-time attribution
/// OFF (nullptr = zero clock calls, zero cost), and a generous advisory plausibility
/// envelope that never rewrites a pose. Every pointer here must outlive the scheduler.
struct MotionSchedulerConfig {
    /// Faults that ABORT the active motion when raised during it (header:
    /// "the fault policy"). Default: ODO_STUCK only — the one code that means
    /// the estimate is lying. Policy, not physics: configurable by design.
    std::uint32_t abortFaultMask = faultBit(diag::FaultCode::OdoStuck);

    /// Scheduler-owned loop timing watchdog (LOOP_OVERRUN). The budget must be
    /// strictly greater than the nominal tick period (loop_monitor.hpp).
    diag::LoopMonitorConfig loopMonitor{};

    /// D-3 tick-time attribution clock (chunk C5). nullptr = attribution OFF —
    /// zero clock calls, zero cost (the A1 contract, structurally). When set, it
    /// must be a clock that advances DURING a tick (tick_attribution.hpp says
    /// which: real time on the robot — R1 wires it; a scripted fake in tests —
    /// the SIM clock only advances between ticks and would attribute all zeros).
    /// Must outlive the scheduler.
    hal::IClock* attributionClock = nullptr;

    /// D-5 pose-delta plausibility envelope (chunk C5): per-tick estimate motion
    /// beyond maxSpeed/maxYawRate × margin × dt raises IMPLAUSIBLE (advisory,
    /// episode-gated — plausibility_guard.hpp). Defaults are generous physical
    /// upper bounds (PROVISIONAL, A4: HA-56).
    diag::PlausibilityConfig plausibility{};
};

/// ITelemetrySink decorator that stamps DebugRecord.activeCommandId with the
/// scheduler's current id (0 between motions). Stamping at the SINK makes id
/// assignment unforgettable for every record producer — no motion type has to
/// remember to do it. The overwrite is unconditional: this scheduler is THE
/// id assigner (debug_record.hpp), so an incoming nonzero id would be a bug,
/// not information. wantsRecord() forwards to the inner sink — the A1 pair
/// rule — so record population stays skipped when nothing consumes it; the
/// one-record copy in emit() is paid only when a real sink is attached.
///
/// Since C5 it also stamps the D-3 tickPhase slots: the scheduler sets the LAST
/// COMPLETED tick's attribution after each tick (records are emitted mid-tick,
/// before this tick's total is knowable — the one-tick lag documented on the
/// schema field). With attribution off the stamp is the quiet all-zeros default.
/// One decorator, one record copy, both stamps.
///
/// ── Since E1 it also stamps the ESTIMATOR fields, and the tick's fault ──────────
/// Two holes were found while wiring the blackbox, and both are fixed HERE because
/// this is the layer that owns record population:
///   * Only MoveToPose stamped `correctionDx/Dy/clampedThisTick`; TurnTo, StrafeTo,
///     DriveBrake, HoldPose and the idle record left them at zero, so what the fusion
///     gate did was invisible for most of a run. The §18.2 gating slots
///     (`gateResidual*`, `gateMahalanobis`, `gateReason`, `covarianceTrace`) had no
///     producer at all.
///   * `DebugRecord::fault` — "the fault raised THIS tick" — had NO producer anywhere
///     in the tree. TermSink has rendered ` flt=NAME` since A1 and it could never
///     appear on a real run; the SdSink flight recorder's whole trigger is that field.
/// Both are now stamped from the ONE place every record already passes through, which
/// is the same reasoning that put the command id here. The fault stamp is deliberately
/// CONDITIONAL (unlike the id): a producer that already knows its own fault keeps it.
/// Honest scope: the stamped fault is the most recent fault raised during this tick
/// BEFORE this record was emitted — a fault raised later in the same tick lands on the
/// next record. The FaultLatch remains the authority on the first-fault root cause.
class CommandIdStampSink final : public hal::ITelemetrySink {
public:
    /// `faults` (optional) supplies the per-tick fault stamp; nullptr disables it.
    explicit CommandIdStampSink(hal::ITelemetrySink& inner,
                                const diag::FaultLatch* faults = nullptr) noexcept
        : inner_{&inner}, faults_{faults} {}

    /// Pass-through, unstamped: every stamp this decorator applies rides the RECORD
    /// channel, so a log line never carries a command id.
    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        inner_->log(level, subsystem, message);
    }

    /// Forwards the inner sink's answer — the A1 pair rule. A NullSink run therefore
    /// still skips record population entirely, and this decorator costs one bool query.
    [[nodiscard]] bool wantsRecord() const noexcept override { return inner_->wantsRecord(); }

    /// Stamp one record and forward it: the command id (UNCONDITIONALLY — this scheduler
    /// is the id assigner, so an incoming nonzero id is a bug, not information), the last
    /// completed tick's phase breakdown, the estimator's gate audit, and — only if the
    /// producer left it None — the fault raised so far this tick. Costs one DebugRecord
    /// copy, paid only when a sink downstream actually wants records.
    void emit(const diag::DebugRecord& record) override {
        diag::DebugRecord stamped = record;
        stamped.activeCommandId = id_;
        stamped.tickPhase = phases_;
        stamped.correctionDx = audit_.dx;
        stamped.correctionDy = audit_.dy;
        stamped.correctionDTheta = audit_.dtheta;  // E3: the §18.2 slot A1 reserved for the
                                                   // heading nudge, filled now that one exists
        stamped.clampedThisTick = audit_.clamped;
        stamped.gateResidualX = audit_.audit.residualX;
        stamped.gateResidualY = audit_.audit.residualY;
        stamped.gateResidualHeading = audit_.audit.residualHeading;
        stamped.gateMahalanobis = audit_.audit.mahalanobis;
        stamped.covarianceTrace = audit_.audit.covarianceTrace;
        stamped.gateReason = audit_.audit.reason;
        if (stamped.fault == diag::FaultCode::None) {
            stamped.fault = tickFault();
        }
        inner_->emit(stamped);
    }

    /// C5 decorator rule (telemetry_sink.hpp): forward, or the summary dies here.
    void summarize(const diag::RunSummary& summary) override { inner_->summarize(summary); }

    /// The id every subsequent record is stamped with; 0 means "between motions". The
    /// scheduler calls this when it arms a motion and again at its boundary — nothing
    /// else should, or records will be attributed to a motion that never emitted them.
    void setActiveId(std::uint32_t id) noexcept { id_ = id; }
    /// Whatever setActiveId() last received; 0 between motions.
    [[nodiscard]] std::uint32_t activeId() const noexcept { return id_; }

    /// Install the per-TickPhase time breakdown stamped onto subsequent records. The
    /// scheduler passes the LAST COMPLETED tick's numbers, because a record emitted
    /// mid-tick cannot know its own tick's total — that is the one-tick lag documented
    /// on DebugRecord::tickPhase. All zeros while attribution is off.
    void setTickPhases(
        const std::array<units::Time, static_cast<std::size_t>(diag::kTickPhaseSlots)>&
            phases) noexcept {
        phases_ = phases;
    }

    /// The estimator's account of the tick just localized (E1). The scheduler calls
    /// this right after Localizer::update(), so every record emitted during the tick —
    /// motion or idle — carries the same, consistent gate audit.
    void setEstimatorAudit(const localization::AppliedCorrection& audit) noexcept {
        audit_ = audit;
    }

    /// Open a new tick for the fault stamp: everything raised from here on belongs to
    /// this tick. Cheap (one counter read) and a no-op without a latch.
    void beginTick() noexcept {
        faultsAtTickStart_ = faults_ != nullptr ? faults_->faultCount() : 0;
    }

private:
    /// The fault raised during this tick so far, or None (header note).
    [[nodiscard]] diag::FaultCode tickFault() const noexcept {
        if (faults_ == nullptr || faults_->faultCount() <= faultsAtTickStart_) {
            return diag::FaultCode::None;
        }
        return faults_->lastFault();
    }

    hal::ITelemetrySink* inner_;
    const diag::FaultLatch* faults_;
    std::uint32_t id_ = 0;
    std::array<units::Time, static_cast<std::size_t>(diag::kTickPhaseSlots)> phases_{};
    localization::AppliedCorrection audit_{};
    int faultsAtTickStart_ = 0;
};

/// ITelemetrySink decorator that AGGREGATES the active motion's record stream into
/// the C5 result-line quantities (motion_result.hpp carries their definitions):
/// start pose, target, worst excursion past the target, final heading error. Sits
/// AFTER the id stamp in the scheduler's chain (it discriminates on the stamped
/// id) and forwards everything untouched — a pure observer.
///
/// Why derive these from the RECORD STREAM rather than ask the motion: the
/// boundary (CompletedMotion) must not re-derive what the motion already
/// published per tick (brief rule 7), overshoot is inherently a per-tick MAX no
/// boundary snapshot can recover, and the stream is the one place every motion
/// type — including future Tier-3 ones — already reports target/measured/error
/// uniformly. Consequence, stated honestly: with NullSink no records flow
/// (wantsRecord false ⇒ never even built), so hasData() is false and the result
/// line renders "n/a" for the derived fields — you cannot have free result
/// numbers AND zero-cost ticks; the always-real fields (final pose, duration,
/// outcome) come from the boundary itself.
///
/// Aggregation rules (each load-bearing, pinned by test):
///   * only records with a nonzero stamped id (idle/teleop records are not the
///     motion's story);
///   * only Running-state ticks and — once Running was seen — the exit-state
///     record (waiting-for-estimate records carry deliberately-zero errors and,
///     for capture-at-live motions, a not-yet-real target: aggregating them
///     would fabricate numbers, the exact lie the brief bans);
///   * target is re-sampled per record (capture-at-live motions publish it from
///     the first live tick; TurnTo/DriveBrake publish a here-anchored target).
class MotionStatsSink final : public hal::ITelemetrySink {
public:
    /// `inner` is NON-OWNING and must outlive this sink; every call is forwarded to it.
    /// One of these serves a whole scheduler, not one motion — beginMotion() is what
    /// clears the aggregates between motions.
    explicit MotionStatsSink(hal::ITelemetrySink& inner) noexcept : inner_{&inner} {}

    /// Pass-through: only the record channel carries the quantities this sink derives.
    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        inner_->log(level, subsystem, message);
    }

    /// Forwards the inner sink's answer, which is also the honest limit of this sink:
    /// behind a sink that wants no records, nothing is ever aggregated, hasData() stays
    /// false, and the derived result-line fields render "n/a" rather than a made-up 0.
    [[nodiscard]] bool wantsRecord() const noexcept override { return inner_->wantsRecord(); }

    /// Aggregate, then forward the record UNMODIFIED — a pure observer that stamps
    /// nothing, so it may sit anywhere after the id stamp it discriminates on.
    void emit(const diag::DebugRecord& record) override {
        aggregate(record);
        inner_->emit(record);
    }

    /// Pass-through, per the decorator rule (telemetry_sink.hpp): a decorator that keeps
    /// the default no-op body silently eats the run summary.
    void summarize(const diag::RunSummary& summary) override { inner_->summarize(summary); }

    /// New motion armed: forget the previous motion's story.
    void beginMotion() noexcept {
        sawRunning_ = false;
        maxProj_ = 0.0;
        maxDist_ = 0.0;
        lastAbsHeadErr_ = 0.0;
    }

    /// True iff at least one live (Running) record was aggregated.
    [[nodiscard]] bool hasData() const noexcept { return sawRunning_; }
    /// The motion's published target, RE-SAMPLED from the most recent aggregated record:
    /// a capture-at-live motion has no real target until its first live tick, so this is
    /// the last target it published, not the one it was constructed with. Read it ONLY
    /// when hasData(): beginMotion() does NOT clear it, so between motions it still holds
    /// the PREVIOUS motion's target — the scheduler substitutes a default Pose2d itself.
    [[nodiscard]] const math::Pose2d& targetPose() const noexcept { return target_; }

    /// Overshoot per motion_result.hpp: projection past the target along the
    /// start→target direction when the motion HAD a direction; worst wander from
    /// the point when it did not (|target − start| < kHoldEpsilonIn).
    [[nodiscard]] units::Length overshoot() const noexcept {
        const double axx = target_.x().value() - startPose_.x().value();
        const double axy = target_.y().value() - startPose_.y().value();
        if (std::hypot(axx, axy) < kHoldEpsilonIn) {
            return units::Length{maxDist_};
        }
        return units::Length{maxProj_ > 0.0 ? maxProj_ : 0.0};
    }

    /// |final heading error| — the last aggregated record's errorHeading.
    [[nodiscard]] units::AngleDim drift() const noexcept {
        return units::AngleDim{lastAbsHeadErr_};
    }

private:
    /// Below this start→target distance a motion is "stationary-target" (turn /
    /// hold / brake) and overshoot degrades to worst-wander (header). Well under
    /// any deliberate translation, well over settle chatter. Logic constant.
    static constexpr double kHoldEpsilonIn = 0.1;

    void aggregate(const diag::DebugRecord& r) {
        if (r.activeCommandId == 0) {
            return;  // idle/teleop record — not a motion's story
        }
        const auto st = static_cast<MotionState>(r.activeCommandState);
        const bool running = (st == MotionState::Running);
        const bool exited = (st == MotionState::Settled || st == MotionState::TimedOut
                             || st == MotionState::Cancelled);
        if (!running && !(exited && sawRunning_)) {
            return;  // waiting records, and boot-window exits that never went live
        }
        if (!sawRunning_) {
            sawRunning_ = true;
            startPose_ = r.measuredPose;
        }
        target_ = r.targetPose;
        const double dx = r.measuredPose.x().value() - target_.x().value();
        const double dy = r.measuredPose.y().value() - target_.y().value();
        const double axx = target_.x().value() - startPose_.x().value();
        const double axy = target_.y().value() - startPose_.y().value();
        const double alen = std::hypot(axx, axy);
        if (alen >= kHoldEpsilonIn) {
            const double proj = (dx * axx + dy * axy) / alen;
            if (proj > maxProj_) {
                maxProj_ = proj;
            }
        }
        const double dist = std::hypot(dx, dy);
        if (dist > maxDist_) {
            maxDist_ = dist;
        }
        lastAbsHeadErr_ = std::abs(r.errorHeading.value());
    }

    hal::ITelemetrySink* inner_;
    math::Pose2d startPose_{};
    math::Pose2d target_{};
    double maxProj_ = 0.0;
    double maxDist_ = 0.0;
    double lastAbsHeadErr_ = 0.0;
    bool sawRunning_ = false;
};

/// One finished motion, as the scheduler saw it — the raw material for the C5
/// per-motion result line (motion/run_reporter.hpp formats it; this type only
/// records). The C5 fields were ADDED here rather than shadowed in a parallel
/// struct (brief rule 7: CompletedMotion is the one motion-boundary record).
struct CompletedMotion {
    std::uint32_t id = 0;         ///< the activeCommandId it ran under
    const char* name = "";        ///< IMotion::name() (stable literal)
    control::ExitReason exit = control::ExitReason::Running;  ///< Running ⇒ "none yet"
    /// None for a settle/timeout/user-cancel; the causal FaultCode when the
    /// scheduler's fault policy (or the task-boundary catch) forced the abort.
    diag::FaultCode abortFault = diag::FaultCode::None;
    units::Time startTime{};      ///< clock at async()
    units::Time endTime{};        ///< clock at the exit/cancel boundary

    // ── C5 additions (result-line data; motion_result.hpp defines the semantics) ────
    /// True iff this Cancelled boundary was a PRE-EMPTION (a newer motion took
    /// the slot) — §18.4's SUPERSEDED, distinct from a user cancel.
    bool preempted = false;
    /// The estimate at the boundary — ALWAYS real (read from the Localizer at
    /// finalize, independent of the record stream).
    math::Pose2d finalPose{};
    /// True iff the record stream flowed for a live tick of this motion; the
    /// three fields below are only meaningful when it did (MotionStatsSink's
    /// honest-scope note — with NullSink they render "n/a", never a lie).
    bool hasPathData = false;
    math::Pose2d targetPose{};    ///< the motion's published target (last sampled)
    units::Length overshoot{};    ///< worst excursion past the target (see semantics)
    units::AngleDim drift{};      ///< |final heading error|
};

/// Boundary-observer seam (chunk C5): the scheduler calls this SYNCHRONOUSLY at
/// every motion boundary — exit, fault abort, user cancel, pre-empt — right
/// after CompletedMotion is fully recorded. This is what makes the per-motion
/// result line STRUCTURAL (RunReporter implements it): a routine cannot forget
/// to report a boundary, the A1 emitRecord lesson one layer up.
/// Contract: the callback may log through the sinks; it must NOT call any
/// scheduler verb (async/cancel/tick/waits — enforced by precondition: the
/// boundary is not a place to re-plan a routine from). It must not throw.
class IMotionObserver {
public:
    /// Interface boilerplate: a public virtual destructor, with the copy/move set
    /// defaulted back in because declaring a destructor suppresses the implicit MOVE
    /// constructor and move assignment (the implicit copies survive, merely deprecated —
    /// spelling all five keeps the intent explicit rather than inherited).
    /// Observers attach by RAW POINTER through setBoundaryObserver(); the scheduler
    /// never owns one, so an observer must outlive it or be detached first.
    virtual ~IMotionObserver() = default;
    IMotionObserver() = default;
    IMotionObserver(const IMotionObserver&) = default;
    IMotionObserver(IMotionObserver&&) = default;
    IMotionObserver& operator=(const IMotionObserver&) = default;
    IMotionObserver& operator=(IMotionObserver&&) = default;

    /// One finished motion, observed at its boundary.
    virtual void onMotionComplete(const CompletedMotion& completed) = 0;
};

/// The loop that actually runs a routine. Exactly ONE active motion and no queue:
/// starting another PRE-EMPTS the first into the cancel safe state (0 V + Brake, applied
/// synchronously), so there is no tick on which two motions command. It never owns time —
/// the injected ITickPacer advances the world, which is what lets the same scheduler be
/// deterministic in host sim and real on the robot. The verbs are async() to arm, tick()
/// or a blocking wait to make progress, cancel() to stop; cancel() with nothing active is
/// still the panic stop, because a cancel that can be too late is one nobody can rely on.
/// Nothing here can hang: waitUntilSettled() is bounded by the motion's own watchdog,
/// waitUntil() by a required explicit timeout, and a pacer that stops advancing the clock
/// fails loudly rather than spinning. Faults in abortFaultMask abort the MOTION, never
/// the run. Single-task by contract, like everything it composes.
class MotionScheduler {
public:
    /// `deps` is the same bundle every motion takes (validated non-null); all
    /// pointees — and `pacer` — must outlive the scheduler.
    MotionScheduler(const MotionDeps& deps, ITickPacer& pacer,
                    const MotionSchedulerConfig& config = {})
        : pacer_{pacer},
          cfg_{config},
          statsHolder_{deps.validatedClock(), deps.ctx->telemetry()},
          stamperSink_{statsHolder_.sink, deps.faults},
          shadowCtx_{chassis::RobotContextConfig{.clock = &deps.ctx->clock(),
                                                 .driveMotors = deps.ctx->driveMotors(),
                                                 .imu = &deps.ctx->imu(),
                                                 .gps = &deps.ctx->gps(),
                                                 .battery = &deps.ctx->battery(),
                                                 .telemetry = &stamperSink_,
                                                 .tags = &deps.ctx->tags(),
                                                 .vision = &deps.ctx->vision()}},
          schedDeps_{MotionDeps{.ctx = &shadowCtx_,
                                .localizer = deps.localizer,
                                .kinematics = deps.kinematics,
                                .faults = deps.faults,
                                .health = deps.health}},
          loopMonitor_{deps.ctx->clock(), *deps.faults, config.loopMonitor},
          poseGuard_{config.plausibility} {
        if (cfg_.attributionClock != nullptr) {
            att_.emplace(*cfg_.attributionClock);  // absent = off = zero cost (D-3)
        }
    }

    /// Neither copyable nor movable, and not by taste: the context this scheduler hands
    /// to motions points at the scheduler's OWN telemetry decorator, so a copy or a move
    /// would leave that route aimed at the original object. Construct one where it will
    /// live and pass it by reference. Destruction is DEFAULTED and does not cancel — a
    /// scheduler destroyed with a motion still armed leaves the drive at its last
    /// command, so call cancel() before letting one go out of scope.
    MotionScheduler(const MotionScheduler&) = delete;
    MotionScheduler(MotionScheduler&&) = delete;
    MotionScheduler& operator=(const MotionScheduler&) = delete;
    MotionScheduler& operator=(MotionScheduler&&) = delete;
    ~MotionScheduler() = default;

    /// The MotionDeps to construct scheduled motions FROM: identical to the
    /// caller's deps except telemetry routes through the id stamp (header:
    /// observability). A motion built with raw deps still schedules correctly —
    /// its records merely carry id 0. Flagged for F6: the C4 facade must build
    /// motions from THIS so the stamping is structural, not remembered.
    [[nodiscard]] const MotionDeps& deps() const noexcept { return schedDeps_; }

    /// Start `motion` without blocking: arm it and return — it progresses on
    /// subsequent ticks (tick() / the blocking waits). If a motion is active,
    /// PRE-EMPT per the pinned semantics (header): the old motion is cancelled
    /// into the safe state first; there is no tick on which both command.
    /// async(active motion) is a well-defined RESTART (cancel + re-arm).
    /// `motion` must outlive its scheduled run. Callable from a waitUntil
    /// predicate; NOT from inside a motion tick.
    void async(IMotion& motion) {
        SHULIB_PRECONDITION(!inTick_,
                            "MotionScheduler::async: cannot start a motion from inside a tick");
        SHULIB_PRECONDITION(!inBoundary_,
                            "MotionScheduler::async: cannot start a motion from a boundary "
                            "observer");
        if (active_ != nullptr) {
            active_->cancel();  // pre-empt: safe state now, old object inert
            // The C5 boundary vocabulary: a pre-empt is SUPERSEDED, not a user
            // cancel — the result line must not blame the routine's author for a
            // stop the scheduler's last-command-wins semantics performed.
            finalize(control::ExitReason::Cancelled, diag::FaultCode::None,
                     /*preempted=*/true);
        }
        active_ = &motion;
        currentId_ = ++idCounter_;
        stamperSink_.setActiveId(currentId_);
        statsHolder_.sink.beginMotion();  // fresh result-line aggregates (C5)
        activeStart_ = schedDeps_.ctx->clock().now();
        snapshotFaultCounts();
        // User code may have run since the last tick — a deliberate gap, not a
        // loop overrun (loop_monitor.hpp reset semantics).
        loopMonitor_.reset();
        ++startedCount_;
        motion.start();
    }

    /// One scheduler tick (header: "who owns the loop") — for callers running
    /// their own paced loop (the facade's non-blocking mode; teleop polling).
    /// Does NOT pace: the caller owns cadence here. Returns whether a motion
    /// is still active after the tick. Not callable re-entrantly or from a
    /// blocking wait (the wait already owns the loop).
    bool tick() {
        SHULIB_PRECONDITION(!inTick_, "MotionScheduler::tick: re-entrant tick");
        SHULIB_PRECONDITION(!inWait_,
                            "MotionScheduler::tick: a blocking wait already owns the loop");
        SHULIB_PRECONDITION(!inBoundary_,
                            "MotionScheduler::tick: cannot tick from a boundary observer");
        tickImpl();
        return active_ != nullptr;
    }

    /// Block until the active motion exits; returns its ExitReason (Settled /
    /// TimedOut / Cancelled — never Running). Bounded WITHOUT a parameter: the
    /// motion's own watchdog guarantees exit (C1, mutation-proven), and the
    /// stalled-pace guard converts a broken pacer into a loud failure. With no
    /// active motion the wait is VACUOUSLY over and returns lastExitReason()
    /// immediately (Settled on a virgin scheduler — completedCount() tells a
    /// caller nothing actually ran).
    [[nodiscard]] control::ExitReason waitUntilSettled() {
        SHULIB_PRECONDITION(!inWait_,
                            "MotionScheduler::waitUntilSettled: blocking waits are not re-entrant");
        SHULIB_PRECONDITION(!inTick_,
                            "MotionScheduler::waitUntilSettled: cannot block from inside a tick");
        SHULIB_PRECONDITION(!inBoundary_,
                            "MotionScheduler::waitUntilSettled: cannot block from a boundary "
                            "observer");
        FlagScope wait{inWait_};
        WaitUnwindGuard unwind{*this};  // F2: a throw must not strand an armed motion
        loopMonitor_.reset();
        stalledPaces_ = 0;
        while (active_ != nullptr) {
            tickImpl();
            if (active_ != nullptr) {
                pace();  // no trailing pace after the exit tick (the C1 rig shape)
            }
        }
        unwind.disarm();
        return lastExit_;
    }

    /// Block until `pred()` holds (checked BEFORE the first tick — true on
    /// entry returns immediately) or `timeoutSeconds` elapses, whichever is
    /// first; the return says which. The active motion (if any) keeps ticking
    /// throughout — this is the marker/callback primitive (G2's PathRunner).
    /// timeout is REQUIRED, finite and >= 0 (0 = an honest poll); a timeout
    /// logs one Warn line and raises NO fault (header: nothing may hang).
    /// `pred` may call async()/cancel() (pre-emption applies); it must not
    /// call a blocking verb (precondition).
    template <typename Pred>
    [[nodiscard]] WaitResult waitUntil(Pred&& pred, double timeoutSeconds) {
        SHULIB_PRECONDITION(!inWait_,
                            "MotionScheduler::waitUntil: blocking waits are not re-entrant");
        SHULIB_PRECONDITION(!inTick_,
                            "MotionScheduler::waitUntil: cannot block from inside a tick");
        SHULIB_PRECONDITION(!inBoundary_,
                            "MotionScheduler::waitUntil: cannot block from a boundary observer");
        SHULIB_PRECONDITION(std::isfinite(timeoutSeconds) && timeoutSeconds >= 0.0,
                            "MotionScheduler::waitUntil: timeout must be finite and >= 0");
        FlagScope wait{inWait_};
        WaitUnwindGuard unwind{*this};  // F2: a throw must not strand an armed motion
        loopMonitor_.reset();
        stalledPaces_ = 0;
        const double deadline = schedDeps_.ctx->clock().now().value() + timeoutSeconds;
        while (true) {
            if (pred()) {
                unwind.disarm();
                return WaitResult::Satisfied;
            }
            if (schedDeps_.ctx->clock().now().value() >= deadline) {
                char buf[80];
                std::snprintf(buf, sizeof buf, "waitUntil timed out after %.2fs",
                              timeoutSeconds);
                schedDeps_.ctx->telemetry().log(hal::LogLevel::Warn, "SCH", buf);
                unwind.disarm();
                return WaitResult::TimedOut;
            }
            tickImpl();
            pace();
        }
    }

    /// Stop the active motion into the defined safe state (0 V + Brake —
    /// motion.hpp), record the Cancelled boundary, and idle the scheduler.
    /// With NO active motion this is the PANIC STOP: the safe state is applied
    /// to the drive anyway (a cancel that can be "too late" to do anything is
    /// a cancel nobody can rely on). Idempotent; callable from a waitUntil
    /// predicate AND from a pacer's pace() (the F2 deadline cut — pinned in
    /// the re-entrancy banner); NOT from inside a motion tick.
    void cancel() {
        SHULIB_PRECONDITION(!inTick_,
                            "MotionScheduler::cancel: cannot cancel from inside a tick");
        SHULIB_PRECONDITION(!inBoundary_,
                            "MotionScheduler::cancel: cannot cancel from a boundary observer");
        if (active_ != nullptr) {
            active_->cancel();
            finalize(control::ExitReason::Cancelled, diag::FaultCode::None);
            return;
        }
        applyCancelSafeState(*schedDeps_.ctx);
    }

    // ── observability (C5's raw material; header note) ─────────────────────────────
    /// True between async() and that motion's boundary — equivalently activeCommandId()
    /// != 0. False again the instant a motion settles, times out, is cancelled or is
    /// pre-empted, on the same tick, before any wait returns.
    [[nodiscard]] bool hasActiveMotion() const noexcept { return active_ != nullptr; }
    /// The active motion's command id; 0 when none. Ids are 1-based and
    /// monotonically increasing for the scheduler's lifetime.
    [[nodiscard]] std::uint32_t activeCommandId() const noexcept { return currentId_; }
    /// Exit reason of the most recently finished motion. Settled before any
    /// motion has finished (the vacuous-wait default — see waitUntilSettled).
    [[nodiscard]] control::ExitReason lastExitReason() const noexcept { return lastExit_; }
    /// The most recent motion boundary in full, overwritten at each one. Default-
    /// constructed until a motion finishes, and IN THAT VIRGIN STATE ONLY it disagrees
    /// with lastExitReason(): this reads Running ("none yet") where that reads Settled
    /// (the vacuous-wait default). Once any motion has reached a boundary the two always
    /// agree — finalize() writes both from the same exit reason. completedCount() is what
    /// actually says whether anything ran.
    [[nodiscard]] const CompletedMotion& lastCompleted() const noexcept { return last_; }
    /// async() calls over the scheduler's lifetime — restarts and pre-empting starts
    /// included, so this counts STARTS, not distinct motion objects. It equals
    /// completedCount() plus one while a motion is active, and equals it exactly when idle.
    [[nodiscard]] int motionsStarted() const noexcept { return startedCount_; }
    /// Motions that reached their exit group and stopped there — the only success
    /// verdict of the four; the counters around it are all the ways a motion did not
    /// finish the job it was given.
    [[nodiscard]] int motionsSettled() const noexcept { return settledCount_; }
    /// Motions the MOTION's own watchdog ended. A waitUntil() timeout is not counted
    /// here and raises no fault — that is a wait giving up, not a motion failing.
    [[nodiscard]] int motionsTimedOut() const noexcept { return timedOutCount_; }
    /// User/pre-empt cancellations (abortFault == None).
    [[nodiscard]] int motionsCancelled() const noexcept { return cancelledCount_; }
    /// Fault-policy + task-boundary aborts (abortFault != None).
    [[nodiscard]] int motionsAborted() const noexcept { return abortedCount_; }
    /// Every motion that reached a boundary: settled + timed out + cancelled + aborted,
    /// a partition with no double counting. This is the number that tells a caller
    /// whether anything actually ran, which lastExitReason() cannot — it reads Settled
    /// on a scheduler that has never been given a motion.
    [[nodiscard]] int completedCount() const noexcept {
        return settledCount_ + timedOutCount_ + cancelledCount_ + abortedCount_;
    }
    /// The scheduler's own tick-timing watchdog, for worstDt() / overrunCount() after a
    /// run. The scheduler ticks it once per tick and RE-BASELINES it at every async() and
    /// at the top of each blocking wait — that drops only the previous tick's timestamp,
    /// so a deliberate gap in which the caller's own code ran between motions is not
    /// reported as an overrun. Nothing here ever clears the statistics: worstDt() and
    /// overrunCount() are WHOLE-RUN totals, not per-motion ones. A gap between two of the
    /// caller's own tick() calls is NOT re-baselined and does count as an overrun.
    [[nodiscard]] const diag::LoopMonitor& loopMonitor() const noexcept { return loopMonitor_; }

    // ── C5 observability additions ─────────────────────────────────────────────────

    /// Attach/replace the boundary observer (nullptr detaches). One observer:
    /// the C5 reporter is the intended consumer; fan-out belongs to a composite
    /// the caller writes if ever needed. Contract in IMotionObserver.
    void setBoundaryObserver(IMotionObserver* observer) noexcept { observer_ = observer; }
    /// The attached observer, or nullptr. NON-OWNING: the scheduler neither deletes it
    /// nor extends its lifetime, so detach before the observer dies.
    [[nodiscard]] IMotionObserver* boundaryObserver() const noexcept { return observer_; }

    /// The run's heading story for the §18.3 summary: max / final of the
    /// PER-MOTION BOUNDARY drifts (|final heading error| of each motion that
    /// produced path data). Deliberately not mid-tick transients: a 90° turn
    /// passes through 90° of "error" by design, and a summary that reported it
    /// would bury the real story — how headings LANDED.
    [[nodiscard]] bool runHasHeadingData() const noexcept { return runHasHeadingData_; }
    /// The largest |final heading error|, in RADIANS, over every motion boundary that
    /// produced path data; 0 while runHasHeadingData() is false. BOUNDARY values only —
    /// a 90° turn passes through 90° of error by design, and counting that would bury
    /// the story this reports. Never reset: one scheduler is one run.
    [[nodiscard]] units::AngleDim runMaxHeadingDrift() const noexcept {
        return units::AngleDim{runMaxDriftRad_};
    }
    /// |final heading error|, in RADIANS, at the LAST boundary that produced path data —
    /// where the run's heading actually LANDED, as opposed to its worst moment. 0 while
    /// runHasHeadingData() is false, which is not the same as a run that landed square.
    [[nodiscard]] units::AngleDim runFinalHeadingDrift() const noexcept {
        return units::AngleDim{runFinalDriftRad_};
    }

    /// The D-3 attribution instrument, when enabled (nullptr when off).
    [[nodiscard]] const diag::TickAttribution* attribution() const noexcept {
        return att_.has_value() ? &*att_ : nullptr;
    }

    /// Consecutive pace() calls that may fail to advance the clock before the
    /// scheduler declares the pacer broken (header: nothing may hang). Pure
    /// logic constant — no hardware claim, hence no register entry.
    static constexpr int kMaxStalledPaces = 100;

private:
    /// F2 (banner: unwind safety): cancels the active motion when an exception
    /// unwinds a blocking wait — safe state applied, boundary recorded, slot
    /// cleared, all BEFORE a stack-owned motion object can die under the
    /// scheduler. Mirrors the facade's DetachGuard shape (disarm() on the
    /// normal path); calls the full cancel() so the boundary accounting stays
    /// truthful — with no active motion it is the panic stop, which is the
    /// right response to an exception mid-wait either way. Declared before the
    /// wait loops use it; constructed AFTER FlagScope so it destructs while
    /// inWait_ is still up (cancel() does not check inWait_ — the same window
    /// the F2 pacer cut uses, pinned in the re-entrancy list).
    class WaitUnwindGuard {
    public:
        explicit WaitUnwindGuard(MotionScheduler& sched) noexcept : sched_{&sched} {}
        ~WaitUnwindGuard() {
            if (sched_ != nullptr) {
                sched_->cancel();
            }
        }
        WaitUnwindGuard(const WaitUnwindGuard&) = delete;
        WaitUnwindGuard& operator=(const WaitUnwindGuard&) = delete;
        void disarm() noexcept { sched_ = nullptr; }

    private:
        MotionScheduler* sched_;
    };

    /// Sets a flag for a scope, exception-safely (the tick body can throw
    /// through — e.g. a Localizer precondition — and the flag must not stick).
    class FlagScope {
    public:
        explicit FlagScope(bool& flag) noexcept : flag_{flag} { flag_ = true; }
        ~FlagScope() { flag_ = false; }
        FlagScope(const FlagScope&) = delete;
        FlagScope& operator=(const FlagScope&) = delete;

    private:
        bool& flag_;
    };

    /// Brackets one tick for D-3 attribution, exception-safely: complete() closes
    /// the tick normally; a throw through the body abandons the half-measured
    /// tick instead (its numbers never completed, so they are discarded, not
    /// reported — and the instrument is re-armed for the next tick).
    class AttributionTickGuard {
    public:
        explicit AttributionTickGuard(diag::TickAttribution* att) : att_{att} {
            if (att_ != nullptr) {
                att_->beginTick();
            }
        }
        ~AttributionTickGuard() {
            if (att_ != nullptr && !completed_) {
                att_->abandonTick();
            }
        }
        AttributionTickGuard(const AttributionTickGuard&) = delete;
        AttributionTickGuard& operator=(const AttributionTickGuard&) = delete;

        void complete() {
            if (att_ != nullptr) {
                att_->endTick();
            }
            completed_ = true;
        }

    private:
        diag::TickAttribution* att_;
        bool completed_ = false;
    };

    /// A phase scope when attribution is on; empty (free) when off. Guaranteed
    /// copy elision constructs the non-movable scope in place.
    [[nodiscard]] std::optional<diag::TickAttribution::PhaseScope> phase(diag::TickPhase p) {
        if (att_.has_value()) {
            return std::optional<diag::TickAttribution::PhaseScope>{std::in_place, *att_, p};
        }
        return std::nullopt;
    }

    /// Stats sink + a validated-before-use hook: validatedClock() runs the deps
    /// validation before any member construction dereferences deps.ctx. (The
    /// chain is producer → id stamp → stats → caller sink: the stats sink
    /// discriminates on the STAMPED id, so it sits after the stamp; this holder
    /// is merely the innermost link and therefore carries the validation hook.)
    struct StatsHolder {
        StatsHolder(hal::IClock& /*validated*/, hal::ITelemetrySink& inner) noexcept
            : sink{inner} {}
        MotionStatsSink sink;
    };

    void tickImpl() {
        FlagScope scope{inTick_};
        // D-3: the attribution tick brackets the whole body; a throw through the
        // body (Localizer precondition — deliberately propagated) abandons the
        // half-measured tick rather than wedging the instrument.
        AttributionTickGuard attGuard{att_.has_value() ? &*att_ : nullptr};
        tickBody();
        attGuard.complete();
        if (att_.has_value()) {
            // Records ride the NEXT emissions with this (completed) breakdown —
            // the one-tick lag documented on the schema field.
            stamperSink_.setTickPhases(att_->lastPhases());
        }
    }

    void tickBody() {
        // E1: open the tick for the fault stamp BEFORE anything can raise, so a fault
        // raised by localization itself still lands on this tick's records.
        stamperSink_.beginTick();
        {
            const auto phaseScope = phase(diag::TickPhase::Localization);
            schedDeps_.localizer->update();
        }
        // E1: the estimator's account of the tick just localized, stamped onto every
        // record this tick emits (motion or idle) — see CommandIdStampSink's header.
        stamperSink_.setEstimatorAudit(schedDeps_.localizer->lastCorrection());
        const units::Time dt = loopMonitor_.tick();
        // D-3 payoff: when this tick's dt says the PREVIOUS tick overran
        // (LoopMonitor just raised), name who consumed it — the last completed
        // attribution IS that tick (tick_attribution.hpp's lag note).
        if (dt.value() >= cfg_.loopMonitor.budget.value() && att_.has_value()
            && att_->hasCompletedTick()) {
            char buf[112];
            std::snprintf(buf, sizeof buf,
                          "overrun attribution: loc %.1fms · mot %.1fms · other %.1fms "
                          "(worst %s)",
                          att_->lastPhases()[0].value() * 1000.0,
                          att_->lastPhases()[1].value() * 1000.0,
                          att_->lastOther().value() * 1000.0,
                          diag::tickPhaseName(att_->lastWorstPhase()));
            schedDeps_.ctx->telemetry().log(hal::LogLevel::Warn, "SCH", buf);
        }
        // D-5 invariant 1: the estimate must move like a robot, not a glitch.
        // Advisory (never rewrites the pose), episode-gated, dt-scaled. NOT
        // judged during the boot window: while Uninitialized the published
        // estimate is definitionally not a physical trajectory (boot garbage is
        // held out of the fold, heading follows a still-calibrating IMU), and
        // the moment it GOES live the pose materializes — a legitimate jump.
        // reset() through boot makes the first live tick a fresh baseline, so
        // neither the window nor the transition can raise a false IMPLAUSIBLE
        // (found immediately by the C2/C4 boot suites when the guard first
        // judged them — boot is normal, not a fault).
        if (schedDeps_.localizer->qualityClass()
            == localization::Localizer::Quality::Uninitialized) {
            poseGuard_.reset();
        } else {
            (void)poseGuard_.check(schedDeps_.localizer->pose(), dt, *schedDeps_.faults);
        }
        if (active_ == nullptr) {
            const auto phaseScope = phase(diag::TickPhase::Motion);
            tickIdleHealth();
            emitIdleRecord(schedDeps_.ctx->clock().now(), dt);
            return;
        }
        const int preCount = schedDeps_.faults->faultCount();
        control::ExitReason reason = control::ExitReason::Running;
        try {
            const auto phaseScope = phase(diag::TickPhase::Motion);
            reason = active_->tick();
        } catch (const PreconditionError& e) {
            // The task-boundary conversion check.hpp promises (header note).
            if (schedDeps_.faults->faultCount() == preCount) {
                // Host policy: the throwing handler raised nothing — raise here
                // so the abort is visible. (On-robot the handler already did.)
                schedDeps_.faults->raise(diag::FaultCode::Precondition, "SCH", e.what());
            }
            active_->cancel();
            finalize(control::ExitReason::Cancelled, diag::FaultCode::Precondition);
            return;
        }
        if (reason != control::ExitReason::Running) {
            finalize(reason, diag::FaultCode::None);
            return;
        }
        const diag::FaultCode abortCause = newlyRaisedAbortFault();
        if (abortCause != diag::FaultCode::None) {
            char buf[80];
            std::snprintf(buf, sizeof buf, "fault abort: %s — cancelling %s",
                          diag::faultCodeName(abortCause), active_->name());
            schedDeps_.ctx->telemetry().log(hal::LogLevel::Warn, "SCH", buf);
            active_->cancel();
            finalize(control::ExitReason::Cancelled, abortCause);
        }
    }

    void finalize(control::ExitReason exit, diag::FaultCode abortFault,
                  bool preempted = false) {
        const MotionStatsSink& stats = statsHolder_.sink;
        const bool hasData = stats.hasData();
        last_ = CompletedMotion{.id = currentId_,
                                .name = active_->name(),
                                .exit = exit,
                                .abortFault = abortFault,
                                .startTime = activeStart_,
                                .endTime = schedDeps_.ctx->clock().now(),
                                .preempted = preempted,
                                // ALWAYS real: read at the boundary, independent
                                // of whether records flowed (C5).
                                .finalPose = schedDeps_.localizer->pose(),
                                .hasPathData = hasData,
                                .targetPose = hasData ? stats.targetPose() : math::Pose2d{},
                                .overshoot = hasData ? stats.overshoot() : units::Length{0.0},
                                .drift = hasData ? stats.drift() : units::AngleDim{0.0}};
        lastExit_ = exit;
        switch (exit) {
            case control::ExitReason::Settled: ++settledCount_; break;
            case control::ExitReason::TimedOut: ++timedOutCount_; break;
            case control::ExitReason::Cancelled:
                if (abortFault == diag::FaultCode::None) {
                    ++cancelledCount_;
                } else {
                    ++abortedCount_;
                }
                break;
            case control::ExitReason::Running: break;  // unreachable: exits only
        }
        if (hasData) {
            runHasHeadingData_ = true;
            runFinalDriftRad_ = std::abs(last_.drift.value());
            if (runFinalDriftRad_ > runMaxDriftRad_) {
                runMaxDriftRad_ = runFinalDriftRad_;
            }
        }
        active_ = nullptr;
        currentId_ = 0;
        stamperSink_.setActiveId(0);
        if (observer_ != nullptr) {
            // The C5 boundary callback: state is fully consistent (slot cleared,
            // last_ recorded). The observer may log; scheduler verbs are
            // precondition-blocked while this flag is up (IMotionObserver).
            FlagScope boundary{inBoundary_};
            observer_->onMotionComplete(last_);
        }
    }

    void snapshotFaultCounts() noexcept {
        for (std::size_t i = 0; i < faultCountsAtStart_.size(); ++i) {
            faultCountsAtStart_[i] =
                schedDeps_.faults->raiseCount(static_cast<diag::FaultCode>(i));
        }
    }

    /// First abort-mask code raised since the active motion started, else None.
    [[nodiscard]] diag::FaultCode newlyRaisedAbortFault() const noexcept {
        std::uint32_t mask = cfg_.abortFaultMask;
        while (mask != 0U) {
            const auto bitIdx = static_cast<unsigned>(std::countr_zero(mask));
            mask &= mask - 1U;
            if (bitIdx >= faultCountsAtStart_.size()) {
                break;  // beyond the tally capacity: nothing to compare against
            }
            const auto code = static_cast<diag::FaultCode>(bitIdx);
            if (schedDeps_.faults->raiseCount(code) > faultCountsAtStart_[bitIdx]) {
                return code;
            }
        }
        return diag::FaultCode::None;
    }

    /// Between motions the scheduler owns the HealthMonitor (C1's named
    /// handoff): every observable it can reach, with odomStalled = false —
    /// nothing is commanded, so there is no spin to cross-check (the same
    /// reasoning as DriveBrake's exemption). One shared definition since C4
    /// (motion.hpp tickHealthObservables).
    void tickIdleHealth() { tickHealthObservables(schedDeps_, false); }

    /// The idle record: pose/quality/power continuity between motions, with NO
    /// invented target or command (fields stay their quiet defaults). Renders
    /// as "[LOC] idle" (id 0, state 0). Lazy via emitRecord — A1 cost contract.
    void emitIdleRecord(units::Time now, units::Time dt) {
        chassis::RobotContext& ctx = *schedDeps_.ctx;
        const localization::Localizer& loc = *schedDeps_.localizer;
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r;
            r.t = now;
            r.dt = dt;
            r.measuredPose = loc.pose();
            r.wheelCount = schedDeps_.kinematics->wheelCount();
            const auto motors = ctx.driveMotors();
            for (std::size_t i = 0;
                 i < motors.size()
                 && i < static_cast<std::size_t>(diag::DebugRecord::kMaxWheels);
                 ++i) {
                r.wheelVoltage[i] = motors[i]->commandedVoltage();
                r.wheelCurrent[i] = motors[i]->current();
            }
            r.imuYaw = ctx.imu().heading();
            r.imuYawRate = ctx.imu().yawRate();
            r.deadReckoning = loc.isDeadReckoning();
            r.qualityClass = static_cast<std::uint8_t>(loc.qualityClass());
            r.quality = loc.quality();
            r.batteryVoltage = ctx.battery().voltage();
            return r;
        });
    }

    /// pace() + the stalled-clock guard (header: nothing may hang).
    void pace() {
        const double before = schedDeps_.ctx->clock().now().value();
        pacer_.pace();
        if (schedDeps_.ctx->clock().now().value() > before) {
            stalledPaces_ = 0;
            return;
        }
        ++stalledPaces_;
        SHULIB_PRECONDITION(stalledPaces_ < kMaxStalledPaces,
                            "MotionScheduler: pacer never advances the clock — a bounded wait "
                            "would hang (is the pacer stepping the plant / delaying?)");
    }

    ITickPacer& pacer_;
    MotionSchedulerConfig cfg_;
    StatsHolder statsHolder_;          // innermost link (validation hook) — C5 stats
    CommandIdStampSink stamperSink_;   // outer link: stamps id + tick phases
    chassis::RobotContext shadowCtx_;  // = caller's ctx, telemetry re-routed
    MotionDeps schedDeps_;
    diag::LoopMonitor loopMonitor_;
    diag::PoseDeltaGuard poseGuard_;             // D-5 invariant 1 (C5)
    std::optional<diag::TickAttribution> att_{};  // D-3; empty = off = zero cost (C5)

    IMotion* active_ = nullptr;
    IMotionObserver* observer_ = nullptr;  // C5 boundary seam (RunReporter)
    std::uint32_t idCounter_ = 0;
    std::uint32_t currentId_ = 0;
    units::Time activeStart_{};
    std::array<int, 32> faultCountsAtStart_{};

    control::ExitReason lastExit_ = control::ExitReason::Settled;  // vacuous default
    CompletedMotion last_{};
    int startedCount_ = 0;
    int settledCount_ = 0;
    int timedOutCount_ = 0;
    int cancelledCount_ = 0;
    int abortedCount_ = 0;
    // Run-level heading story for the §18.3 summary (per-motion boundary drift —
    // the max/final of the drifts, NOT mid-turn transients; run_reporter.hpp).
    double runMaxDriftRad_ = 0.0;
    double runFinalDriftRad_ = 0.0;
    bool runHasHeadingData_ = false;

    bool inTick_ = false;
    bool inWait_ = false;
    bool inBoundary_ = false;  // observer callback in progress (re-entrancy guard)
    int stalledPaces_ = 0;
};

}  // namespace shulib::motion
