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
//   * A BLOCKING verb (waitUntil / waitUntilSettled / tick) from inside a
//     predicate: REJECTED by precondition — disguised recursion whose depth is
//     user-data-dependent. Nothing in the G2 marker use case needs it; relaxing
//     later is additive, un-forbidding is not.
//   * async()/cancel() from inside a motion's tick(): REJECTED by precondition
//     — mutating the active slot while active->tick() is on the stack.
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

#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/loop_monitor.hpp"
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

struct MotionSchedulerConfig {
    /// Faults that ABORT the active motion when raised during it (header:
    /// "the fault policy"). Default: ODO_STUCK only — the one code that means
    /// the estimate is lying. Policy, not physics: configurable by design.
    std::uint32_t abortFaultMask = faultBit(diag::FaultCode::OdoStuck);

    /// Scheduler-owned loop timing watchdog (LOOP_OVERRUN). The budget must be
    /// strictly greater than the nominal tick period (loop_monitor.hpp).
    diag::LoopMonitorConfig loopMonitor{};
};

/// ITelemetrySink decorator that stamps DebugRecord.activeCommandId with the
/// scheduler's current id (0 between motions). Stamping at the SINK makes id
/// assignment unforgettable for every record producer — no motion type has to
/// remember to do it. The overwrite is unconditional: this scheduler is THE
/// id assigner (debug_record.hpp), so an incoming nonzero id would be a bug,
/// not information. wantsRecord() forwards to the inner sink — the A1 pair
/// rule — so record population stays skipped when nothing consumes it; the
/// one-record copy in emit() is paid only when a real sink is attached.
class CommandIdStampSink final : public hal::ITelemetrySink {
public:
    explicit CommandIdStampSink(hal::ITelemetrySink& inner) noexcept : inner_{&inner} {}

    void log(hal::LogLevel level, std::string_view subsystem,
             std::string_view message) override {
        inner_->log(level, subsystem, message);
    }

    [[nodiscard]] bool wantsRecord() const noexcept override { return inner_->wantsRecord(); }

    void emit(const diag::DebugRecord& record) override {
        diag::DebugRecord stamped = record;
        stamped.activeCommandId = id_;
        inner_->emit(stamped);
    }

    void setActiveId(std::uint32_t id) noexcept { id_ = id; }
    [[nodiscard]] std::uint32_t activeId() const noexcept { return id_; }

private:
    hal::ITelemetrySink* inner_;
    std::uint32_t id_ = 0;
};

/// One finished motion, as the scheduler saw it — the raw material for C5's
/// per-motion result line (C5 formats; this chunk only records).
struct CompletedMotion {
    std::uint32_t id = 0;         ///< the activeCommandId it ran under
    const char* name = "";        ///< IMotion::name() (stable literal)
    control::ExitReason exit = control::ExitReason::Running;  ///< Running ⇒ "none yet"
    /// None for a settle/timeout/user-cancel; the causal FaultCode when the
    /// scheduler's fault policy (or the task-boundary catch) forced the abort.
    diag::FaultCode abortFault = diag::FaultCode::None;
    units::Time startTime{};      ///< clock at async()
    units::Time endTime{};        ///< clock at the exit/cancel boundary
};

class MotionScheduler {
public:
    /// `deps` is the same bundle every motion takes (validated non-null); all
    /// pointees — and `pacer` — must outlive the scheduler.
    MotionScheduler(const MotionDeps& deps, ITickPacer& pacer,
                    const MotionSchedulerConfig& config = {})
        : pacer_{pacer},
          cfg_{config},
          stamper_{deps.validatedClock(), deps.ctx->telemetry()},
          shadowCtx_{chassis::RobotContextConfig{.clock = &deps.ctx->clock(),
                                                 .driveMotors = deps.ctx->driveMotors(),
                                                 .imu = &deps.ctx->imu(),
                                                 .gps = &deps.ctx->gps(),
                                                 .battery = &deps.ctx->battery(),
                                                 .telemetry = &stamper_.sink,
                                                 .tags = &deps.ctx->tags(),
                                                 .vision = &deps.ctx->vision()}},
          schedDeps_{MotionDeps{.ctx = &shadowCtx_,
                                .localizer = deps.localizer,
                                .kinematics = deps.kinematics,
                                .faults = deps.faults,
                                .health = deps.health}},
          loopMonitor_{deps.ctx->clock(), *deps.faults, config.loopMonitor} {}

    // Self-referential (shadowCtx_ points at stamper_): pinned in place.
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
        if (active_ != nullptr) {
            active_->cancel();  // pre-empt: safe state now, old object inert
            finalize(control::ExitReason::Cancelled, diag::FaultCode::None);
        }
        active_ = &motion;
        currentId_ = ++idCounter_;
        stamper_.sink.setActiveId(currentId_);
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
        FlagScope wait{inWait_};
        loopMonitor_.reset();
        stalledPaces_ = 0;
        while (active_ != nullptr) {
            tickImpl();
            if (active_ != nullptr) {
                pace();  // no trailing pace after the exit tick (the C1 rig shape)
            }
        }
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
        SHULIB_PRECONDITION(std::isfinite(timeoutSeconds) && timeoutSeconds >= 0.0,
                            "MotionScheduler::waitUntil: timeout must be finite and >= 0");
        FlagScope wait{inWait_};
        loopMonitor_.reset();
        stalledPaces_ = 0;
        const double deadline = schedDeps_.ctx->clock().now().value() + timeoutSeconds;
        while (true) {
            if (pred()) {
                return WaitResult::Satisfied;
            }
            if (schedDeps_.ctx->clock().now().value() >= deadline) {
                char buf[80];
                std::snprintf(buf, sizeof buf, "waitUntil timed out after %.2fs",
                              timeoutSeconds);
                schedDeps_.ctx->telemetry().log(hal::LogLevel::Warn, "SCH", buf);
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
    /// predicate; NOT from inside a motion tick.
    void cancel() {
        SHULIB_PRECONDITION(!inTick_,
                            "MotionScheduler::cancel: cannot cancel from inside a tick");
        if (active_ != nullptr) {
            active_->cancel();
            finalize(control::ExitReason::Cancelled, diag::FaultCode::None);
            return;
        }
        applyCancelSafeState(*schedDeps_.ctx);
    }

    // ── observability (C5's raw material; header note) ─────────────────────────────
    [[nodiscard]] bool hasActiveMotion() const noexcept { return active_ != nullptr; }
    /// The active motion's command id; 0 when none. Ids are 1-based and
    /// monotonically increasing for the scheduler's lifetime.
    [[nodiscard]] std::uint32_t activeCommandId() const noexcept { return currentId_; }
    /// Exit reason of the most recently finished motion. Settled before any
    /// motion has finished (the vacuous-wait default — see waitUntilSettled).
    [[nodiscard]] control::ExitReason lastExitReason() const noexcept { return lastExit_; }
    [[nodiscard]] const CompletedMotion& lastCompleted() const noexcept { return last_; }
    [[nodiscard]] int motionsStarted() const noexcept { return startedCount_; }
    [[nodiscard]] int motionsSettled() const noexcept { return settledCount_; }
    [[nodiscard]] int motionsTimedOut() const noexcept { return timedOutCount_; }
    /// User/pre-empt cancellations (abortFault == None).
    [[nodiscard]] int motionsCancelled() const noexcept { return cancelledCount_; }
    /// Fault-policy + task-boundary aborts (abortFault != None).
    [[nodiscard]] int motionsAborted() const noexcept { return abortedCount_; }
    [[nodiscard]] int completedCount() const noexcept {
        return settledCount_ + timedOutCount_ + cancelledCount_ + abortedCount_;
    }
    [[nodiscard]] const diag::LoopMonitor& loopMonitor() const noexcept { return loopMonitor_; }

    /// Consecutive pace() calls that may fail to advance the clock before the
    /// scheduler declares the pacer broken (header: nothing may hang). Pure
    /// logic constant — no hardware claim, hence no register entry.
    static constexpr int kMaxStalledPaces = 100;

private:
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

    /// Stamper + a validated-before-use hook: validatedClock() runs the deps
    /// validation before any member construction dereferences deps.ctx.
    struct StampHolder {
        StampHolder(hal::IClock& /*validated*/, hal::ITelemetrySink& inner) noexcept
            : sink{inner} {}
        CommandIdStampSink sink;
    };

    void tickImpl() {
        FlagScope scope{inTick_};
        schedDeps_.localizer->update();
        const units::Time dt = loopMonitor_.tick();
        if (active_ == nullptr) {
            tickIdleHealth();
            emitIdleRecord(schedDeps_.ctx->clock().now(), dt);
            return;
        }
        const int preCount = schedDeps_.faults->faultCount();
        control::ExitReason reason = control::ExitReason::Running;
        try {
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

    void finalize(control::ExitReason exit, diag::FaultCode abortFault) {
        last_ = CompletedMotion{.id = currentId_,
                                .name = active_->name(),
                                .exit = exit,
                                .abortFault = abortFault,
                                .startTime = activeStart_,
                                .endTime = schedDeps_.ctx->clock().now()};
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
        active_ = nullptr;
        currentId_ = 0;
        stamper_.sink.setActiveId(0);
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
    StampHolder stamper_;
    chassis::RobotContext shadowCtx_;  // = caller's ctx, telemetry re-routed
    MotionDeps schedDeps_;
    diag::LoopMonitor loopMonitor_;

    IMotion* active_ = nullptr;
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

    bool inTick_ = false;
    bool inWait_ = false;
    int stalledPaces_ = 0;
};

}  // namespace shulib::motion
