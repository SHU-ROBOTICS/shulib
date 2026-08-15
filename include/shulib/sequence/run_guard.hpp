#pragma once
//
// RunGuard — the run-scoped deadline owner and the guaranteed END-OF-RUN ACTION
// (chunk F2, WS8/M4; master plan §14's non-negotiable, D-8's discharge).
//
// Before this file, every bound in the tree was scoped to ONE motion, ONE wait,
// or ONE mechanism operation. Nothing bounded a whole routine and nothing knew
// when the match ends: eleven steps at the 5 s default timeout is a legal
// 200-second routine inside a 15-second match, behaving exactly as designed.
// RunGuard is the owner that outlives a motion: it holds a run-scoped start
// time, two caller-supplied deadlines, a latch, and one caller-supplied action
// that fires when scoring time is up — D-8 (the routine-level watchdog) and
// the end-of-run action as ONE primitive with two policies, not two features.
//
// ── What "guaranteed" means here, and what it does not (read before quoting) ────────
// F2 proves a SCHEDULING property, against the host plant: a deliberately
// stalled scoring loop still ends with the caller's end action performed, with
// the clock driven to the match limit. It CANNOT claim the timing margin is
// right on a real brain — real loop rate under load and PROS call latency are
// unmeasured until R4 — and NOTHING PREEMPTS PURE USER CODE: there are no
// background tasks, so code that never lets a finished shulib call end its
// loop (an unconditional retry `while` that ignores guard.expired()) keeps
// control forever and the end action runs only when it returns. The guard
// makes every shulib call after the deadline finish quickly and refuses to let
// new ones make progress; it cannot take the CPU from you. Two lateness holes
// are structural and documented at the waits section below.
//
// ── The library refuses to know your strategy ───────────────────────────────────────
// No field coordinate, no park pose, no default lead time, no default match
// length lives here or anywhere in shulib. The library knows only that SOME
// caller-supplied action fires at SOME caller-supplied instant: a team ending
// tucked against a goal supplies that pose; a team raising a lift under a
// height limit supplies that; §14's own robots supply a park AND a Toggle
// re-verify (the action is a callable, so it composes). A default lead time
// would be an invented number governing whether the robot scores — HA-51's
// invented 5 s default is the cautionary tale, cited on purpose. Both
// instants are REQUIRED, validated, and relative to run() start.
//
// ── Two instants, because two different things want to happen (T2) ──────────────────
//   * endActionAt — STOP SCORING, early enough to still reach the end
//     position: the scoring latch fires, the active motion is cut, mechanisms
//     are cancelled into their declared safe states (cancel-all STRICTLY
//     precedes act: a stalled operation's unreleased claim would otherwise
//     make the end action's own operation throw at start() — measured), and
//     the end action runs with the remaining runway.
//   * hardStopAt — BE SAFE, unconditionally: every device is forced safe and
//     everything, the end action included, is refused from here on. Fires
//     even if the end action is still running — safety is not negotiable,
//     going somewhere is. Safing is the library's to own once a deadline
//     exists at all; where to GO is strategy and stays the caller's.
//
// ── How the deadline actually reaches running code (T1 — the measured design) ───────
// The ONLY seam that regains control mid-motion is the tick pacer, so RunGuard
// IS an ITickPacer: construct it around the real pacer and hand it to the
// Chassis constructor. Every pace() while a run is live checks the deadlines
// BEFORE advancing the world — that ordering is load-bearing and pinned by
// test: check-then-step measured 0.0000 in of post-deadline travel,
// step-then-check measured 10.79 in. At the cut, the guard calls
// scheduler.cancel() from pace() — legal, and since F2 pinned in C2's
// re-entrancy list — which unwinds waitUntilSettled on the same iteration:
// the blocking verb returns Cancelled (its honest verdict: the motion WAS
// cancelled), a Routine records the stop and skips the rest, and control
// returns to the caller, who is now standing after run()'s scoring call.
// run() then performs cancel-all and the end action — in the caller's own
// call context, through the frozen facade's ordinary blocking verbs, so F2
// adds NO second loop owner and never re-implements C2's loop.
//   Rejected: a supervisory scheduler.tick() loop (a second loop owner
// duplicating what the existing waits already do); async()-from-pace() to
// hijack the caller's wait into driving the end action (measured to WORK and
// to LIE — the caller's moveTo returned Settled describing the park, zero log
// lines; the worst outcome measured in the campaign); a THROWING pacer
// (measured: 11.4 V under Coast); cancel-only expiry with no latch (measured
// INERT — cancel/restart resets the deceleration and the run arrives sooner).
//
// ── The latch (T7): after the deadline, motions are refused, not raced ──────────────
// Once endActionAt passes, any motion outside the end action is cancelled at
// the next pace() — one commanded tick, zero plant travel (the ordering pin
// above), every occurrence counted, the first Warn-logged. A retrying caller
// gets each retry cut the same way and the world does not move. The end
// action is EXEMPT (the guard knows when it is running it); the hard floor
// exempts nothing. After run() returns the guard is inert again — the robot
// belongs to the caller (post-run code, driver control) and a guard that kept
// refusing forever would fight the next mode.
//
// ── The waits (T4): what is deadline-aware and what CANNOT be ───────────────────────
// waitFor()/pause() here are deadline-aware: they return RunExpired at the
// live deadline with zero latency (implemented as a composite predicate over
// C2's own waitUntil — reusing its guards, not its shape) and RunExpired WINS
// a tie with Satisfied, because a chain that reads "satisfied" keeps scoring
// past the buzzer (measured: a deadline folded into a plain predicate returns
// Satisfied, which Routine::waitFor maps to success). The FROZEN F10/F6 waits
// (Routine::pause / Routine::waitFor / Chassis::wait / Chassis::waitUntil)
// CANNOT be deadline-aware without a breaking change: a scheduler-level wait
// checks its predicate and its own timeout and nothing else — 2801 cancels
// were fired into one and all were invisible. The lateness bound for frozen
// code is: THE UNEXPIRED REMAINDER OF THE WAIT'S OWN TIMEOUT at the instant
// the deadline fires, summed over every wait/pause step executed after that
// instant (a Routine usually pays one term: its first post-deadline motion is
// refused and stops the chain; consecutive pauses each pay). Budget waits
// tightly or use the guard's own.
//
// ── Reaching the mechanisms (T6) ────────────────────────────────────────────────────
// cancel-all walks the caller-supplied span<hal::IMechanism*>: a registered
// claimant (mechanism.hpp's F2 hook) is cancelled — inert, safe, claim
// released; an ANONYMOUS claim is force-released with a Warn (the guard
// cannot render an unknown operation inert — register a claimant); then
// applySafeState() lands the declared state regardless. applySafeState alone
// is NOT enough and the guard never relies on it alone: a live operation
// re-commands its voltage on its next tick, restoring voltage but not brake
// mode — the half-safe `brake=Hold, V=9.0` that passes any mode-only check.
//
// ── Verdict honesty (T5) ────────────────────────────────────────────────────────────
// GuardedWaitResult is a sequence-layer vocabulary, minted because no existing
// one can say "the RUN's budget expired" distinctly from "this wait's own
// timeout elapsed" — WaitResult::Satisfied does not mean success and
// RoutineStopCause is Tier-2's. Map in, never re-mean. The end action's
// verdict is reported in the RunGuardReport and logged — never as the
// caller's motion verdict, and never silently.
//
// FREEZES NOTHING. Single consumer today (F4's student-authored routines are
// hardware-gated; D1 ruled G2 out) — the register says "open by design" out
// loud, and nothing here may freeze on one consumer's evidence.

#include <cmath>
#include <cstdio>
#include <functional>
#include <span>
#include <type_traits>
#include <utility>

#include "shulib/chassis/chassis.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sequence {

/// The guard's wait verdict (banner: verdict honesty). DISTINCT from
/// WaitResult on purpose: RunExpired is a fact about the RUN, not the wait,
/// and it must be impossible to read as success.
enum class GuardedWaitResult {
    Satisfied,   ///< the predicate became true before any deadline
    TimedOut,    ///< the WAIT's own timeout elapsed first (run still live)
    RunExpired,  ///< the RUN's deadline passed — stop scoring; wins ties with
                 ///< Satisfied (a satisfied-but-expired wait must still halt
                 ///< the chain — the measured predicate-folding trap)
};

/// One run's schedule + reach. Everything is REQUIRED and caller-supplied:
/// there is deliberately no default here to invent (banner).
struct RunGuardConfig {
    /// When scoring stops and the end action starts, measured from run()
    /// start. The caller computes the lead ("park takes ~6 s") — the library
    /// has no number to offer that would not be an invented one.
    units::Time endActionAt{0.0};
    /// The unconditional safe floor, measured from run() start. At this
    /// instant every device is forced safe and everything — the end action
    /// included — is refused. Must be >= endActionAt; the gap is the end
    /// action's runway (equal instants = zero runway: legal, and the end
    /// action's motions will all be refused — supply distinct instants if it
    /// must MOVE).
    units::Time hardStopAt{0.0};
    /// Every mechanism the run touches (may be empty). cancel-all reaches
    /// operations through the claim's registered claimant (mechanism.hpp);
    /// list a mechanism here or the guard cannot see it at the deadline.
    std::span<hal::IMechanism* const> mechanisms{};

    /// Reject a schedule that could not mean anything, before a run arms: both
    /// instants finite, endActionAt > 0, hardStopAt >= endActionAt, and no null in
    /// `mechanisms`. run() calls it at the door, so a bad number is a loud error at
    /// the call site instead of a deadline that silently never arrives.
    void validate() const {
        SHULIB_PRECONDITION(std::isfinite(endActionAt.value()) && endActionAt.value() > 0.0,
                            "RunGuardConfig: endActionAt must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(hardStopAt.value())
                                && hardStopAt.value() >= endActionAt.value(),
                            "RunGuardConfig: hardStopAt must be finite and >= endActionAt");
        for (const hal::IMechanism* m : mechanisms) {
            SHULIB_PRECONDITION(m != nullptr, "RunGuardConfig: a mechanism is null");
        }
    }
};

/// What one guarded run did — the guard's own account, kept SEPARATE from
/// every motion verdict the caller's code saw (banner: verdict honesty).
struct RunGuardReport {
    /// True iff the deadline latched scoring off (false: scoring returned on
    /// its own and the end action started early — the caller was done).
    bool scoringCut = false;
    bool endActionRan = false;        ///< the callable was invoked (always, unless a throw unwound run())
    bool endActionSucceeded = false;  ///< its verdict, per the four accepted return types
    bool floorFired = false;          ///< hardStopAt arrived during the run
    /// Scheduler cancels the guard performed after the latch (the first is
    /// the cut; the rest are refused retries). Zero plant travel either way.
    int postExpiryCancels = 0;
    /// Anonymous claims force-released at cancel-all (should be zero —
    /// register claimants).
    int anonymousClaimsReleased = 0;
    /// pace() calls observed while the run was live. ZERO after a run whose
    /// scoring did real work means the Chassis was NOT constructed with this
    /// guard as its pacer — the guard was never in the loop and its
    /// guarantee never applied (Warn-logged).
    int pacesSeen = 0;
    units::Time scoringEnded{0.0};    ///< clock at scoring()'s return, from run start
    units::Time endActionEnded{0.0};  ///< clock at the end action's return, from run start
};

/// The run-scoped deadline owner (file banner). Construct it around the real
/// pacer, give the Chassis the guard AS its pacer, then wrap the whole auton
/// in run(). Inert by construction: until run() is live, pace() is a pure
/// pass-through — zero clock reads, zero behavior change (the D3 §2.1
/// instruction: a deadline must be opt-in and inert by default; wiring the
/// guard in must not change an existing routine by one tick).
///
///     motion::ITickPacer& real = ...;             // plant pacer / R1's delay
///     sequence::RunGuard guard{real};
///     chassis::Chassis chassis{deps, guard, cfg}; // the guard IS the pacer
///     ...
///     const sequence::RunGuardReport rep = guard.run(chassis, runCfg,
///         [&] { /* scoring: Routine chain, verbs, guard.waitFor(...) */ },
///         [&] { /* end action: YOUR pose, YOUR re-verify */ return true; });
///
/// Not copyable/movable: the Chassis holds a reference to it as the pacer.
class RunGuard final : public motion::ITickPacer {
public:
    /// `inner` advances the real world (host: step the plant; robot: delay to
    /// the tick boundary) and must outlive the guard.
    explicit RunGuard(motion::ITickPacer& inner) noexcept : inner_{&inner} {}

    /// Pinned where it is constructed: the Chassis holds this object BY REFERENCE
    /// as its pacer, so a copy would be paced by nobody and a move would leave the
    /// Chassis pacing a corpse. The destructor releases nothing — the guard owns no
    /// device and holds only non-owning pointers to the inner pacer and, while a
    /// run is live, the chassis's scheduler, clock and telemetry.
    RunGuard(const RunGuard&) = delete;
    RunGuard(RunGuard&&) = delete;
    RunGuard& operator=(const RunGuard&) = delete;
    RunGuard& operator=(RunGuard&&) = delete;
    ~RunGuard() override = default;

    /// The pacer seam (banner: how the deadline reaches running code). The
    /// deadline checks run BEFORE the world advances — the ordering is
    /// load-bearing (0.0000 in vs 10.79 in of post-deadline travel, measured)
    /// and pinned by test. Inert pass-through when no run is live.
    void pace() override {
        if (running_) {
            ++pacesSeen_;
            const double now = clock_->now().value();
            if (now >= floorDeadline_) {
                fireFloor(now);
            } else if (now >= actDeadline_ && !inEndAction_) {
                noteExpired(now);
                cutActiveMotion();
            }
        }
        inner_->pace();  // the world advances AFTER the checks (the ordering pin)
    }

    // ── live observers (callable from scoring / the end action, during run()) ──────

    /// True once the CURRENT phase's deadline has passed: endActionAt during
    /// scoring, hardStopAt during the end action. The retry-loop idiom:
    /// `while (!guard.expired() && ...) { ... }` — an unconditional retry
    /// loop is the one stall the guard cannot end (banner, honesty section).
    [[nodiscard]] bool expired() const {
        SHULIB_PRECONDITION(running_, "RunGuard::expired: no run is live");
        return clock_->now().value() >= phaseDeadline();
    }

    /// Time left before the current phase's deadline (never negative).
    /// During the end action this counts down to the hard stop — the
    /// "hold position until the buzzer" budget.
    [[nodiscard]] units::Time remaining() const {
        SHULIB_PRECONDITION(running_, "RunGuard::remaining: no run is live");
        const double left = phaseDeadline() - clock_->now().value();
        return units::Time{left > 0.0 ? left : 0.0};
    }

    /// True only while run() is executing — scoring OR the end action. That window
    /// is exactly when expired(), remaining(), waitFor() and pause() may be called
    /// at all (outside it they trip a precondition) and exactly when pace() checks
    /// deadlines rather than passing straight through. False before the first run
    /// and again the moment run() returns: the robot belongs to the caller then.
    [[nodiscard]] bool running() const noexcept { return running_; }

    // ── the deadline-aware waits (banner: T4) ──────────────────────────────────────

    /// Block until `pred` holds, the wait's own `timeout` elapses, or the
    /// run's live deadline passes — the return says which, and RunExpired
    /// wins a tie with Satisfied (banner: verdict honesty). Implemented over
    /// C2's waitUntil with a composite predicate, so every C2 guard (finite
    /// timeout, stalled-pace loudness, no blocking verbs in `pred`) applies
    /// unchanged; at the deadline it returns with zero latency and `pred` is
    /// not called again — a scoring predicate that ticks an operation stops
    /// being ticked the instant scoring time is over (the latch, applied to
    /// waits). The active motion keeps ticking throughout, exactly as C2's
    /// wait — until the pace-side latch cuts it.
    template <typename Pred>
    [[nodiscard]] GuardedWaitResult waitFor(Pred&& pred, units::Time timeout) {
        SHULIB_PRECONDITION(running_, "RunGuard::waitFor: no run is live");
        if (expiredNow()) {
            return GuardedWaitResult::RunExpired;  // zero ticks, zero latency
        }
        const motion::WaitResult w = sched_->waitUntil(
            [this, &pred] { return expiredNow() || pred(); }, timeout.value());
        if (expiredNow()) {
            return GuardedWaitResult::RunExpired;  // wins the tie (measured trap)
        }
        return w == motion::WaitResult::Satisfied ? GuardedWaitResult::Satisfied
                                                  : GuardedWaitResult::TimedOut;
    }

    /// Sleep `duration`, or less if the run's live deadline arrives first —
    /// Satisfied means the full duration was slept, RunExpired means the run
    /// cut it short (TimedOut is unreachable: the sleep IS the timeout).
    /// The deadline-aware twin of Chassis::wait / Routine::pause, which
    /// cannot be cut (banner: T4) — the "wait for the alliance partner, but
    /// never past the budget" beat. `duration` must be finite and > 0.
    [[nodiscard]] GuardedWaitResult pause(units::Time duration) {
        SHULIB_PRECONDITION(running_, "RunGuard::pause: no run is live");
        SHULIB_PRECONDITION(std::isfinite(duration.value()) && duration.value() > 0.0,
                            "RunGuard::pause: duration must be finite and > 0");
        const double wake = clock_->now().value() + duration.value();
        // The composite predicate is time-monotone, so the timeout backstop
        // below is unreachable slack, never a reachable timeout — the same
        // Warn-free construction as Chassis::wait, same code-level constant
        // reasoning (any value clearing one tick behaves identically; no HA
        // entry, nothing about hardware is assumed).
        return waitFor([this, wake] { return clock_->now().value() >= wake; },
                       units::Time{duration.value() + kPauseBackstopSeconds});
    }

    // ── the run itself ─────────────────────────────────────────────────────────────

    /// Execute one guarded run (file banner carries the whole design):
    ///   1. arm — capture the run start from the chassis clock; deadlines
    ///      become absolute instants; the pacer checks go live;
    ///   2. `scoring()` — your auton, written against the ordinary frozen
    ///      surface (Routine chains, blocking verbs, guard.waitFor). It ends
    ///      when it returns — early because it finished, or because the
    ///      deadline cut its motions/waits and its chain stopped;
    ///   3. cancel-all — active motion cancelled, every listed mechanism's
    ///      claimant cancelled, claims cleared, declared safe states applied.
    ///      STRICTLY before step 4 (a stalled operation's unreleased claim
    ///      would make the end action's own operation throw at start());
    ///   4. `endAction()` — YOUR final act, running in your own call context
    ///      through the same public verbs, bounded by the hard floor. Return
    ///      void (always "performed"), bool, ExitReason (Settled = success)
    ///      or MechanismOutcome (Succeeded = success) — then()'s exact
    ///      convention. Its verdict lands in the report and the log, never
    ///      in any motion verdict your scoring code saw;
    ///   5. final cancel-all + disarm — the guard hands the robot back safe
    ///      and goes inert.
    /// If scoring() or endAction() THROWS (a precondition — a programming
    /// error), the guard cancels-all and safes on the unwind and RETHROWS: a
    /// broken program stays loud, and the guard does not drive to a pose on
    /// its behalf (converting a throw into a park would hide the bug).
    ///
    /// `chassis` MUST be the one constructed with THIS guard as its pacer —
    /// the guard has no way to verify that wiring, so it counts: a finished
    /// run that saw zero pace() calls Warn-logs that the guarantee never
    /// applied (RunGuardReport::pacesSeen).
    template <typename Scoring, typename EndAction>
    RunGuardReport run(chassis::Chassis& chassis, const RunGuardConfig& config,
                       Scoring&& scoring, EndAction&& endAction) {
        SHULIB_PRECONDITION(!running_, "RunGuard::run: a run is already live");
        config.validate();
        arm(chassis, config);
        RunGuardReport report{};
        {
            RunUnwindGuard unwind{*this};  // throw ⇒ cancel-all + disarm + rethrow
            std::forward<Scoring>(scoring)();
            report.scoringCut = expired_;
            report.scoringEnded = sinceStart();
            cancelAll();  // strictly precedes the act (banner: T2)
            logActStart();
            inEndAction_ = true;
            report.endActionRan = true;
            report.endActionSucceeded = invokeEndAction(std::forward<EndAction>(endAction));
            inEndAction_ = false;
            report.endActionEnded = sinceStart();
            cancelAll();  // hand the robot back safe, whatever the action left
            unwind.disarm();
        }
        logVerdict(report);
        report.floorFired = floorFired_;
        report.postExpiryCancels = postExpiryCancels_;
        report.anonymousClaimsReleased = anonymousReleases_;
        report.pacesSeen = pacesSeen_;
        disarm();
        return report;
    }

private:
    /// pause()'s backstop slack over its wake deadline (seconds) — see the
    /// pause() comment; the Chassis::wait precedent, deliberately not an HA
    /// entry (no plant, gain or field property depends on it).
    static constexpr double kPauseBackstopSeconds = 1.0;

    /// Cancel-all + disarm on an exception unwinding run() (member comment
    /// on run()). Mirrors the facade's DetachGuard shape.
    class RunUnwindGuard {
    public:
        explicit RunUnwindGuard(RunGuard& guard) noexcept : guard_{&guard} {}
        ~RunUnwindGuard() {
            if (guard_ != nullptr) {
                guard_->inEndAction_ = false;
                guard_->cancelAll();
                guard_->disarm();
            }
        }
        RunUnwindGuard(const RunUnwindGuard&) = delete;
        RunUnwindGuard& operator=(const RunUnwindGuard&) = delete;
        void disarm() noexcept { guard_ = nullptr; }

    private:
        RunGuard* guard_;
    };

    void arm(chassis::Chassis& chassis, const RunGuardConfig& config) {
        sched_ = &chassis.scheduler();
        clock_ = &chassis.deps().ctx->clock();
        telemetry_ = &chassis.deps().ctx->telemetry();
        mechanisms_ = config.mechanisms;
        startTime_ = clock_->now().value();
        actDeadline_ = startTime_ + config.endActionAt.value();
        floorDeadline_ = startTime_ + config.hardStopAt.value();
        expired_ = false;
        floorFired_ = false;
        inEndAction_ = false;
        warnedRefusal_ = false;
        postExpiryCancels_ = 0;
        anonymousReleases_ = 0;
        pacesSeen_ = 0;
        running_ = true;
        char buf[96];
        std::snprintf(buf, sizeof buf,
                      "run guard armed: end action at T+%.2fs, hard stop at T+%.2fs",
                      config.endActionAt.value(), config.hardStopAt.value());
        telemetry_->log(hal::LogLevel::Info, "SEQ", buf);
    }

    void disarm() noexcept { running_ = false; }

    [[nodiscard]] double phaseDeadline() const noexcept {
        return inEndAction_ ? floorDeadline_ : actDeadline_;
    }

    /// True once the CURRENT phase's deadline has passed — and, like pace(),
    /// this FIRES the floor rather than merely reporting it.
    ///
    /// REVIEWER FIX (independent probe, after the chunk's own campaign): the
    /// floor could silently not fire. C2's waitUntil evaluates `pred` BEFORE
    /// pace(), so when the end action ticked an operation through waitFor, this
    /// predicate observed the floor and returned — ending the wait correctly —
    /// while pace() never ran fireFloor(). Measured at the time: the intake sat
    /// at 9.000 V past the floor, `report.floorFired` read false, and the
    /// "all devices safed unconditionally" Warn was never logged. Devices were
    /// still safed when run() returned, so it was never a runaway — but a run
    /// where the floor MATTERED was indistinguishable from one where it never
    /// came up, which is the observability failure E1 spent a chunk on.
    ///
    /// The floor is checked FIRST and unconditionally, mirroring pace() exactly
    /// (that ordering is why the floor outranks the scoring deadline in both
    /// entry points). Calling fireFloor() from a predicate is legal: C2 permits
    /// cancel() from a waitUntil predicate, and fireFloor() is idempotent.
    [[nodiscard]] bool expiredNow() {
        if (floorFired_) {
            return true;
        }
        const double now = clock_->now().value();
        if (now >= floorDeadline_) {
            fireFloor(now);  // a wait can reach the floor before any pace does
            return true;
        }
        if (!inEndAction_ && now >= actDeadline_) {
            noteExpired(now);  // a wait can see expiry before any pace does
            return true;
        }
        return now >= phaseDeadline();
    }

    [[nodiscard]] units::Time sinceStart() const {
        return units::Time{clock_->now().value() - startTime_};
    }

    void noteExpired(double now) {
        if (expired_) {
            return;
        }
        expired_ = true;
        char buf[80];
        std::snprintf(buf, sizeof buf, "run budget expired at T+%.2fs — scoring latched off",
                      now - startTime_);
        telemetry_->log(hal::LogLevel::Warn, "SEQ", buf);
    }

    /// The latch's enforcement: any motion live after the deadline (outside
    /// the end action) is cancelled — the first is the cut that unwinds the
    /// caller's blocking verb, the rest are refused retries. Counted; the
    /// refusals Warn once (a retry storm would flood the transcript — 300
    /// cancel boundaries were measured buying nothing but telemetry).
    void cutActiveMotion() {
        if (!sched_->hasActiveMotion()) {
            return;
        }
        sched_->cancel();  // legal from pace(); pinned in C2's re-entrancy list
        ++postExpiryCancels_;
        if (postExpiryCancels_ >= 2 && !warnedRefusal_) {
            warnedRefusal_ = true;
            telemetry_->log(hal::LogLevel::Warn, "SEQ",
                            "post-deadline motion refused (further refusals are "
                            "counted, not logged)");
        }
    }

    /// The hard floor (banner: T2): every device safe, everything refused,
    /// every pace from now on — idempotent and cheap, and repeating it means
    /// nothing a predicate re-commands can outlive one pace boundary.
    void fireFloor(double now) {
        if (!floorFired_) {
            floorFired_ = true;
            noteExpired(now);
            char buf[80];
            std::snprintf(buf, sizeof buf,
                          "hard stop at T+%.2fs — all devices safed unconditionally",
                          now - startTime_);
            telemetry_->log(hal::LogLevel::Warn, "SEQ", buf);
        }
        if (sched_->hasActiveMotion()) {
            sched_->cancel();
            ++postExpiryCancels_;
        } else {
            sched_->cancel();  // panic stop: the drive lands safe even between motions
        }
        safeMechanisms();
    }

    /// cancel-all's mechanism half (banner: T6): claimant cancelled (inert +
    /// safe + released), anonymous claims force-released with a Warn, then
    /// the declared safe state regardless — order pinned by test, because
    /// applySafeState() without the cancel lasts exactly one tick.
    void safeMechanisms() {
        for (hal::IMechanism* m : mechanisms_) {
            hal::ICancellable* claimant = m->claimant();
            if (claimant != nullptr) {
                claimant->cancel();
            } else if (m->claimed()) {
                m->releaseClaim();
                ++anonymousReleases_;
                char buf[112];
                std::snprintf(buf, sizeof buf,
                              "mechanism '%s': anonymous claim force-released — register "
                              "a claimant so the guard can cancel the operation",
                              m->name());
                telemetry_->log(hal::LogLevel::Warn, "SEQ", buf);
            }
            m->applySafeState();
        }
    }

    /// The full cancel-all: drive first (the big rolling mass), then the
    /// mechanisms. Idempotent; used at the act boundary, at run() exit, and
    /// on the unwind path.
    void cancelAll() {
        sched_->cancel();  // active motion → safe state; none → panic stop
        safeMechanisms();
    }

    void logActStart() {
        char buf[96];
        std::snprintf(buf, sizeof buf,
                      "end-of-run action starting at T+%.2fs (%.2fs to the hard stop)",
                      sinceStart().value(), floorDeadline_ - clock_->now().value());
        telemetry_->log(hal::LogLevel::Info, "SEQ", buf);
    }

    void logVerdict(const RunGuardReport& report) {
        if (report.endActionSucceeded) {
            telemetry_->log(hal::LogLevel::Info, "SEQ", "end-of-run action succeeded");
        } else {
            telemetry_->log(hal::LogLevel::Warn, "SEQ",
                            "end-of-run action FAILED — devices safed");
        }
        if (pacesSeen_ == 0) {
            telemetry_->log(hal::LogLevel::Warn, "SEQ",
                            "guard pacer never ran — was this Chassis constructed with "
                            "the guard as its pacer? the deadline guarantee never applied");
        }
    }

    /// then()'s exact return convention (routine.hpp), applied to the end
    /// action: void / bool / ExitReason / MechanismOutcome.
    template <typename Action>
    [[nodiscard]] bool invokeEndAction(Action&& action) {
        using R = std::invoke_result_t<Action&>;
        if constexpr (std::is_void_v<R>) {
            std::invoke(action);
            return true;
        } else if constexpr (std::is_same_v<R, control::ExitReason>) {
            return std::invoke(action) == control::ExitReason::Settled;
        } else if constexpr (std::is_same_v<R, manipulation::MechanismOutcome>) {
            return std::invoke(action) == manipulation::MechanismOutcome::Succeeded;
        } else {
            static_assert(std::is_constructible_v<bool, R>,
                          "RunGuard: the end action must return void, bool, "
                          "control::ExitReason, or manipulation::MechanismOutcome");
            return static_cast<bool>(std::invoke(action));
        }
    }

    motion::ITickPacer* inner_;
    motion::MotionScheduler* sched_ = nullptr;
    hal::IClock* clock_ = nullptr;
    hal::ITelemetrySink* telemetry_ = nullptr;
    std::span<hal::IMechanism* const> mechanisms_{};
    double startTime_ = 0.0;
    double actDeadline_ = 0.0;
    double floorDeadline_ = 0.0;
    bool running_ = false;
    bool expired_ = false;
    bool floorFired_ = false;
    bool inEndAction_ = false;
    bool warnedRefusal_ = false;
    int postExpiryCancels_ = 0;
    int anonymousReleases_ = 0;
    int pacesSeen_ = 0;
};

}  // namespace shulib::sequence
