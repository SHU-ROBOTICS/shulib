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
//   * hardStopAt — BE SAFE, unconditionally: every device is forced safe, and
//     every MOTION and every WAIT is refused from here on — including the end
//     action's own. What is NOT refused is the end action's INVOCATION: run()
//     has no floor-fired guard and reports endActionRan = true even when the
//     floor already fired during scoring, which is reachable exactly in the
//     case this banner's own honesty section describes (scoring code that keeps
//     the CPU past both deadlines). The guard cannot preempt caller code, so
//     what it can refuse is what the action ASKS FOR, not that it runs; the
//     field doc on endActionRan states the same thing. Fires
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
//   The latch reaches CLAIMED MECHANISMS too (the mechanism core, MECH-A2, the
// team lead's N1 (a), 2026-10-08) — see "Reaching the mechanisms" below.
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
// A mechanism that THROWS there (a broken adapter) skips nothing: every listed
// mechanism is attempted, then the first exception propagates; the unwind path's
// own cancel-all drops its repeat, so a throwing safe state is a loud error the
// caller catches, never std::terminate (COMP-t's review, 2026-09-29, measured).
//   THE LATCH CUTS CLAIMED MECHANISMS (MECH-A2; N1 (a), the team lead's "accept
// all", 2026-10-08). A mechanism step waits through Chassis::waitUntil — a FROZEN
// wait this guard cannot cut (the waits section above) — so before this, an intake
// step still running at endActionAt ran on to its own bound: measured 1.01 s late
// with 12.0 V·s past the cut on a 2 s step, and on a 5 s one 2.02 s late — the
// hard stop fired during scoring and the end action (the park) FAILED; after a
// disable, up to 2.52 s late, which COMP-b's opcontrol() waits out. Now, in the
// two branches of pace() that cut the motion, the latch also calls cancel() on
// every listed mechanism whose claim has a REGISTERED claimant (a mechanism-service
// slot, an F1 operation): inert, safe, released — the step ends CANCELLED at the
// next pace, its frozen wait sees it, and the routine unwinds (measured: 0.01 s
// late, 0.0 V·s; the end action ran and succeeded). WHERE it runs, exactly:
//   * the DEADLINE cut never runs inside the end action (`!inEndAction_`, as the
//     motion cut) — the end action's own steps are exempt from the deadline;
//   * an ABANDON (a disable) cuts claimed mechanisms EVERYWHERE, the end action
//     included — B1's "exempts nothing" (measured: a step inside the end action,
//     cut 0.02 s after the disable, 0.1 V·s).
// An UNCLAIMED mechanism is NOT re-safed every pace: a repeated brake under Hold
// re-captures its point and lets a loaded shaft creep (HA-144) — run()'s
// cancel-all still makes every listed mechanism safe, as before. Each cut is
// counted (RunGuardReport::mechanismCuts; a refused retry counts again) and the
// run's first cut is logged once. A BEHAVIOUR CHANGE for F1 operations too: one
// with a registered claimant now stops at the deadline, not at its own bound.
//
// ── Verdict honesty (T5) ────────────────────────────────────────────────────────────
// GuardedWaitResult is a sequence-layer vocabulary, minted because no existing
// one can say "the RUN's budget expired" distinctly from "this wait's own
// timeout elapsed" — WaitResult::Satisfied does not mean success and
// RoutineStopCause is Tier-2's. Map in, never re-mean. The end action's
// verdict is reported in the RunGuardReport and logged — never as the
// caller's motion verdict, and never silently.
//
// ── ABANDON: a third way for a run to end, and it is a LATCH (COMP-a, ruling B1) ────
// A competition program must be able to end a run that is NOT out of time: the
// field disabled the robot, or the match left autonomous, while the routine was
// mid-motion. requestAbandon() is that door, and what it does is deliberately
// small — ONE atomic store, callable from any task and from inside a pacer (the
// competition program's watch calls it from ITS pace(), which runs inside this
// guard's pace(); a store cannot re-enter anything). Everything else happens on
// the run's own task, through machinery the deadline already proved:
//   * from the next check on, the run behaves EXACTLY as after endActionAt, with
//     no exemption for the end action: pace() cuts any active motion BEFORE the
//     world advances (one commanded tick per refused start, zero plant travel —
//     the ordering pin, B22), and waitFor()/pause() return RunExpired and
//     expired() reads true, without throwing;
//   * run() — never requestAbandon() — sees the flag when scoring returns: it
//     performs the FULL cancel-all (drive, then every listed mechanism: claimant
//     cancelled, anonymous claim force-released, declared safe state applied),
//     SKIPS the end action (a disabled robot's motors are off; the park cannot
//     run, and an end action started into a disable is a routine commanding a
//     robot the field has taken back), and disarms at its normal exit, with
//     RunGuardReport::abandoned set.
// Rejected: an abandon that DISARMS. The routine is still on the stack, inside a
// blocking verb whose motion lives in the scheduler's slot; a disarmed guard
// would pass every later pace() straight through, so the routine's next verb
// would drive the robot the field just disabled, and every guard call after it
// would throw "no run is live" out of the routine instead of unwinding it. The
// latch lets the routine unwind NORMALLY, on its own stack, so nothing dangles
// (ruling C5 of the COMP brief: why PROS killing the task was rejected).
// A request made while no run is live is dropped when the next run arms (a stale
// request from the last run must not abandon the next one at T+0) — so a
// program's watch requests on the competition STATE, every tick a run is live,
// not once on its edge.
//
// FREEZES NOTHING. TWO consumers since COMP-a: the host suites, and
// competition::runAuton (match_runner.hpp), the competition program's auton
// runner — which is why requestAbandon() could be added without a version bump.
// pacesTotal() (COMP-t, 2026-09-28) is additive the same way: a counter every pace
// increments, read by the Auton Tester's boot to prove its pacer chain. The
// mechanism cut and RunGuardReport::mechanismCuts (MECH-A2, 2026-10-15) change no
// signature — but they change BEHAVIOUR (an operation with a registered claimant
// now stops at the deadline), so the changelog says so.
// The freeze itself is the team lead's call (Freeze Register row F12), and one
// program that has not run on a robot is not yet the evidence a freeze wants.

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <exception>
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
    /// The callable was invoked — always, unless a throw unwound run() or the run was
    /// ABANDONED before scoring returned (then it is skipped, and this stays false).
    bool endActionRan = false;
    /// Its verdict, per the four accepted return types — and FALSE whenever the run was
    /// abandoned, even if an end action already under way returned success: a verdict
    /// about a park the field disabled halfway through is not a success.
    bool endActionSucceeded = false;
    bool floorFired = false;          ///< hardStopAt arrived during the run
    /// requestAbandon() latched during this run (file banner: ABANDON). The cancel-all
    /// ran, the end action was skipped if it had not started (`endActionRan` false), and
    /// the guard disarmed at run()'s normal exit.
    bool abandoned = false;
    /// Scheduler cancels the guard performed after the latch — the deadline's or an
    /// abandon's (the first is the cut; the rest are refused retries). Zero plant travel
    /// either way.
    int postExpiryCancels = 0;
    /// Anonymous claims force-released at cancel-all (should be zero —
    /// register claimants).
    int anonymousClaimsReleased = 0;
    /// pace() calls observed while the run was live. ZERO after a run whose
    /// scoring did real work means the Chassis was NOT constructed with this
    /// guard as its pacer — the guard was never in the loop and its
    /// guarantee never applied (Warn-logged).
    int pacesSeen = 0;
    /// Claimed mechanisms the latch CANCELLED after the deadline or an abandon (MECH-A2, the file
    /// banner's mechanism cut): a mechanism step cut mid-wait counts one; a refused retry counts
    /// again. The hard floor's sweep (every mechanism, claimed or not) is `floorFired`'s, not counted
    /// here; nor is run()'s own cancel-all.
    int mechanismCuts = 0;
    units::Time scoringEnded{0.0};    ///< clock at scoring()'s return, from run start
    /// Clock at the end action's return, from run start — 0 when the end action never ran
    /// (an abandoned run skips it; read `endActionRan` before this).
    units::Time endActionEnded{0.0};
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
        ++pacesTotal_;  // every pace, run or not: a program proves its pacer chain with it
        if (running_) {
            ++pacesSeen_;
            const double now = clock_->now().value();
            if (now >= floorDeadline_) {
                fireFloor(now);
            } else if (abandonRequested()) {
                // B1: exactly the deadline latch, and it exempts NOTHING — not even the
                // end action (a park driven into a disable is the thing being refused).
                // The mechanism cut runs here too, the end action included (N1, D5).
                noteAbandoned(now);
                cutActiveMotion();
                cutClaimedMechanisms();
            } else if (now >= actDeadline_ && !inEndAction_) {
                // The DEADLINE: never inside the end action, for the motion and the
                // mechanisms alike (N1, D5).
                noteExpired(now);
                cutActiveMotion();
                cutClaimedMechanisms();
            }
        }
        inner_->pace();  // the world advances AFTER the checks (the ordering pin)
    }

    // ── abandon (file banner: ABANDON — a latch, never a disarm) ───────────────────

    /// Ask the live run to END NOW, cleanly: one atomic store and nothing else, so it is
    /// safe from any task and from inside a pacer's pace() (the competition program's
    /// watch). From the next check on, every motion is cut before the world advances,
    /// every guard wait returns RunExpired and expired() reads true; when scoring
    /// returns, run() performs the full cancel-all, SKIPS the end action and disarms at
    /// its normal exit, reporting `abandoned`. It never disarms the guard itself.
    /// With no run live it is harmless: the request is dropped when the next run arms.
    void requestAbandon() noexcept { abandon_.store(true); }

    /// True once requestAbandon() has been called since the current (or, after run()
    /// returned, the most recent) run armed. Readable from any task.
    [[nodiscard]] bool abandonRequested() const noexcept { return abandon_.load(); }

    /// Every pace() since construction, whether or not a run was live (RunGuardReport's
    /// `pacesSeen` counts only a run's, so nothing in the report can say anything about the
    /// wiring BEFORE a run). A program reads it before and after one idle tick to PROVE its
    /// Chassis is paced through this guard -- the wiring run() itself cannot verify (it only
    /// warns, after a run, when `pacesSeen` is 0). The Auton Tester refuses its boot when that
    /// proof fails (COMP-t, 2026-09-28: additive). The pacing task only.
    [[nodiscard]] std::uint64_t pacesTotal() const noexcept { return pacesTotal_; }

    // ── live observers (callable from scoring / the end action, during run()) ──────

    /// True once the CURRENT phase's deadline has passed: endActionAt during
    /// scoring, hardStopAt during the end action — or at once, in either phase, once
    /// the run has been abandoned. The retry-loop idiom:
    /// `while (!guard.expired() && ...) { ... }` — an unconditional retry
    /// loop is the one stall the guard cannot end (banner, honesty section).
    [[nodiscard]] bool expired() const {
        SHULIB_PRECONDITION(running_, "RunGuard::expired: no run is live");
        return abandonRequested() || clock_->now().value() >= phaseDeadline();
    }

    /// Time left before the current phase's deadline (never negative).
    /// During the end action this counts down to the hard stop — the
    /// "hold position until the buzzer" budget. Zero once the run is abandoned.
    [[nodiscard]] units::Time remaining() const {
        SHULIB_PRECONDITION(running_, "RunGuard::remaining: no run is live");
        if (abandonRequested()) {
            return units::Time{0.0};
        }
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
    /// If the run was ABANDONED (requestAbandon()) by the time scoring returns,
    /// step 4 is SKIPPED: the cancel-all of step 3 is the last act, and the guard
    /// disarms here, at run()'s normal exit, with RunGuardReport::abandoned set.
    /// An abandon that arrives DURING the end action cuts its motions like any
    /// other (the latch exempts nothing) and voids its verdict.
    /// If scoring() or endAction() THROWS (a precondition — a programming
    /// error), the guard cancels-all and safes on the unwind and RETHROWS: a
    /// broken program stays loud, and the guard does not drive to a pose on
    /// its behalf (converting a throw into a park would hide the bug). A
    /// mechanism whose claimant or safe state throws does not stop the sweep:
    /// every listed mechanism is still attempted, then the first such exception
    /// propagates like any other throw — and the unwind never throws a second
    /// one (that would be std::terminate).
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
            cancelAll();  // strictly precedes the act (banner: T2); an abandon's full cancel-all
            if (!abandonRequested()) {
                logActStart();
                inEndAction_ = true;
                report.endActionRan = true;
                report.endActionSucceeded = invokeEndAction(std::forward<EndAction>(endAction));
                inEndAction_ = false;
                report.endActionEnded = sinceStart();
                cancelAll();  // hand the robot back safe, whatever the action left
            }
            unwind.disarm();
        }
        report.abandoned = abandonRequested();
        if (report.abandoned) {
            report.endActionSucceeded = false;  // no verdict survives an abandon (field doc)
        }
        logVerdict(report);
        report.floorFired = floorFired_;
        report.postExpiryCancels = postExpiryCancels_;
        report.anonymousClaimsReleased = anonymousReleases_;
        report.pacesSeen = pacesSeen_;
        report.mechanismCuts = mechanismCuts_;
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
    ///
    /// NEVER THROWS (COMP-t's review, 2026-09-29). It runs while one exception is
    /// already in flight, and a second one leaving a destructor is std::terminate —
    /// measured: a mechanism whose applySafeState() threw made the NORMAL path's
    /// cancel-all throw, and this destructor's cancel-all threw again (exit 134),
    /// so the program died instead of reaching its handler. The cancel-all here is
    /// still complete (safeMechanisms() attempts every mechanism whatever one of
    /// them throws); anything it throws is dropped, because the exception already
    /// unwinding is the one the caller must see.
    class RunUnwindGuard {
    public:
        explicit RunUnwindGuard(RunGuard& guard) noexcept : guard_{&guard} {}
        ~RunUnwindGuard() {
            if (guard_ != nullptr) {
                guard_->inEndAction_ = false;
                try {
                    guard_->cancelAll();
                } catch (...) {  // NOLINT(bugprone-empty-catch): see the class comment
                }
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
        // FIRST: a request left over from the last run (or made while idle) must not
        // abandon this one at T+0 (banner: ABANDON). A request made from here on is kept.
        abandon_.store(false);
        abandonNoted_ = false;
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
        mechanismCuts_ = 0;
        cutNoted_ = false;
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
        if (abandonRequested()) {
            noteAbandoned(now);  // a wait can see the abandon before any pace does
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

    /// The abandon latch's one log line (banner: ABANDON), once per run. ASCII only:
    /// it is new text, and the competition program may copy SEQ lines to its screen.
    void noteAbandoned(double now) {
        if (abandonNoted_) {
            return;
        }
        abandonNoted_ = true;
        char buf[112];
        std::snprintf(buf, sizeof buf,
                      "run ABANDONED at T+%.2fs -- motions refused from here, %s",
                      now - startTime_,
                      inEndAction_ ? "the end action is cut" : "the end action will be skipped");
        telemetry_->log(hal::LogLevel::Warn, "SEQ", buf);
    }

    /// The latch's enforcement: any motion live after the deadline (outside
    /// the end action) or after an abandon (anywhere) is cancelled — the first is the
    /// cut that unwinds the caller's blocking verb, the rest are refused retries.
    /// Counted; the refusals Warn once (a retry storm would flood the transcript — 300
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
                            abandonRequested()
                                ? "motion refused after the abandon (further refusals are "
                                  "counted, not logged)"
                                : "post-deadline motion refused (further refusals are "
                                  "counted, not logged)");
        }
    }

    /// The latch reaches MECHANISMS too (banner: the mechanism cut; N1 (a)). Every listed
    /// mechanism whose claim has a REGISTERED claimant — a mechanism-service slot, an F1
    /// operation — is cancelled now: inert, safe, released, so a mechanism step blocked in a
    /// FROZEN wait ends CANCELLED at this pace instead of running on to its own bound. An
    /// unclaimed mechanism is NOT re-safed here (a repeated Hold re-captures its point,
    /// HA-144); an anonymous claim is left for run()'s cancel-all, which force-releases it with
    /// its Warn. Counted (mechanismCuts); the run's first cut is logged once. EVERY mechanism is
    /// attempted whatever one claimant throws, then the first exception propagates — the
    /// safeMechanisms() rule (COMP-t's review).
    void cutClaimedMechanisms() {
        std::exception_ptr first;
        for (hal::IMechanism* m : mechanisms_) {
            try {
                hal::ICancellable* claimant = m->claimant();
                if (claimant == nullptr) {
                    continue;
                }
                claimant->cancel();
                ++mechanismCuts_;
                if (!cutNoted_) {
                    cutNoted_ = true;
                    char buf[160];
                    std::snprintf(buf, sizeof buf,
                                  "mechanism '%.40s' cut %s -- its operation cancelled into its safe "
                                  "state (later cuts are counted, not logged)",
                                  m->name(), abandonRequested() ? "after the abandon" : "at the deadline");
                    telemetry_->log(hal::LogLevel::Warn, "SEQ", buf);
                }
            } catch (...) {
                if (!first) {
                    first = std::current_exception();
                }
            }
        }
        if (first) {
            std::rethrow_exception(first);
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
    ///
    /// EVERY mechanism is attempted, whatever one of them throws (COMP-t's review,
    /// 2026-09-29): a throwing claimant or safe state no longer skips the mechanisms
    /// after it. The FIRST exception is rethrown once the sweep is done, so a broken
    /// adapter stays loud (run()'s unwind path then drops its repeat — RunUnwindGuard).
    void safeMechanisms() {
        std::exception_ptr first;
        for (hal::IMechanism* m : mechanisms_) {
            try {
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
            } catch (...) {
                if (!first) {
                    first = std::current_exception();
                }
            }
        }
        if (first) {
            std::rethrow_exception(first);
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
        if (report.abandoned) {
            telemetry_->log(hal::LogLevel::Warn, "SEQ",
                            report.endActionRan
                                ? "run ABANDONED during the end-of-run action -- its verdict "
                                  "is void, devices safed"
                                : "run ABANDONED -- end-of-run action skipped, devices safed");
        } else if (report.endActionSucceeded) {
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
    bool abandonNoted_ = false;  // the abandon Warn is logged once per run
    int postExpiryCancels_ = 0;
    int anonymousReleases_ = 0;
    int pacesSeen_ = 0;
    int mechanismCuts_ = 0;      // claimed mechanisms the latch cancelled (cutClaimedMechanisms)
    bool cutNoted_ = false;      // the first cut's Warn is logged once per run
    std::uint64_t pacesTotal_ = 0;  // every pace since construction (pacesTotal())
    // The ONE member another task may write (banner: ABANDON). Everything else here
    // belongs to the run's own task, as before.
    std::atomic<bool> abandon_{false};
};

}  // namespace shulib::sequence
