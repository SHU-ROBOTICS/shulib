#pragma once
//
// Routine — the Tier-2 recipe layer (chunk D1, WS12/M7 pulled forward; master
// plan §17).
//
// ═══ STATUS: FROZEN — F10, LOCKED 2026-08-12 (chunk D3, API 2.0) ══════════════════
// This surface is FROZEN, as its own register row (F10) rather than as part of
// F6: the facade it delegates to is a different tier and can version
// independently. Frozen: the constructor (both spellings, noexcept),
// non-copyable/non-movable, ELEVEN steps — startAt / moveTo / driveTo /
// strafeTo / turnTo / face / followTrajectory (span + brace) / brake / hold /
// pause / waitFor — the four observers ok / result / lastTrajectory / chassis,
// the types RoutineResult (all eight fields) and RoutineStopCause (append-only,
// existing values fixed), the documented error policy below, and typed time as
// a SEMANTIC (hold(0.3) must not compile). Enforced structurally:
// test/routine_signature_pin_test.cpp fails the build, naming F10, if any of
// those drifts. Changes only with a major API-version bump plus a migration
// note (include/shulib/version.hpp); ADDITIVE growth — new steps, new
// observers, new RoutineResult fields, appended RoutineStopCause enumerators —
// stays legal and is the intended path.
//
// NOT frozen, deliberately: **then()** — its accepted return types and its
// `name` default were left out of F10 because they were chosen before
// mechanisms existed; chunk F1 then filled the seam (a fourth accepted return
// type, manipulation::MechanismOutcome, and the MechanismFailed stop cause).
// then() STAYS unfrozen until the seam has a second real consumer (F2's
// combinators / F3's primitives) — the same build → second consumer → freeze
// path every other surface here took. Also unfrozen: the exact WORDING of the
// stop/skip log lines (the behaviour — one Warn naming routine and step, one
// Info per skipped step — is frozen; the sentence is not).
// Stated out loud because silence in a freeze reads as "frozen too" (D2 ruling
// A2's lesson). The freeze waited for a second independent consumer: D3's
// recipe cookbook (docs/cookbook/), which wrote fourteen recipes against this
// surface and needed zero changes to it. Its critique — every awkwardness
// found, with a recommendation — is the D3 completion record's centrepiece
// (development log, shulib-v2 branch).
//
// ═══ What this class is ══════════════════════════════════════════════════════════
// A fluent, EAGER chain over the Chassis facade, so a complete autonomous
// routine is ~10 readable lines:
//
//     Routine r{chassis, "left-side"};
//     r.startAt(Pose2d{-48_in, -24_in, 90_deg})
//      .moveTo(Pose2d{-24_in, 0_in, 45_deg})
//      .then([&] { intake.in(); }, "intake")  // your mechanism code (ch. 13)
//      .strafeTo(-24_in, 24_in)
//      .face(0_in, 48_in)
//      .driveTo(0_in, 48_in)
//      .hold(300_ms)
//      .brake();
//     if (!r.ok()) { /* strategy branch on r.result() */ }
//
// EAGER means each step runs (blocking) the moment it is chained — the routine
// runs exactly in the order it reads, top to bottom, and a first-year can say
// what the robot will do by reading it. The deliberate alternative (a DEFERRED
// chain that accumulates steps and executes on a terminal .run()) was rejected:
// it re-opens the read-order/run-order split when recipe and facade calls mix,
// it needs step storage (heap or an arbitrary capacity) where this class needs
// none, and it adds the worst misuse door of all — a chain built but never
// run, which compiles and does nothing. The full analysis is in the D1
// completion record (development log, `shulib-v2` branch).
//
// ═══ Tier discipline: this layer DELEGATES ═══════════════════════════════════════
// Every step is exactly one Chassis call (§17: tiers are strict supersets — a
// recipe is never the only way to do something). There is no motion logic
// here: no gains, no kinematics, no exit criteria, no fault policy. The two
// "field vocabulary" steps, face(x, y) and driveTo(x, y), compute ONE argument
// (the bearing from the live pose estimate to a field point — pure trig) and
// then delegate to turnTo / moveTo; that keeps C1's D12 intact on tank drives
// (the AUTHOR still writes the turn — face() IS the author's turn, said in
// field words), and a hand-written turnTo(atan2(...)) remains bit-identical
// to it by construction. Dropping from Tier 2 to Tier 3 is deleting the
// Routine object and keeping every verb.
//
// ═══ The error policy (decided, documented, tested — design constraint 4) ═════════
// A fluent chain makes it EASY to swallow an ExitReason, so the chain acts on
// failure whether or not anyone reads the result:
//
//   * A step FAILS when its motion exits non-Settled, a trajectory does not
//     complete every leg, a waitFor() times out, or a then()-action reports
//     failure. On the first failure the chain STOPS:
//       1. the drive is put in the defined safe state (Chassis::cancel — 0 V
//          + Brake; idempotent, HA-53),
//       2. every later step is SKIPPED (counted and visible, never executed —
//          a routine that keeps driving after a failed move is chasing the
//          field from a position it is not at),
//       3. one Warn names the failing step; each skipped step logs at Info,
//       4. ok() goes false and result() carries step index, name, cause, and
//          the failing motion's ExitReason.
//   * Skipping is not silent recovery: the transcript shows the stop, the
//     fault latch and C5 result lines below carry the pathology, and the
//     robot is parked — the failure is behaviourally loud even if the author
//     never branches on it.
//   * PRECONDITION THROWS ARE NOT POLICY. Nonsense input (NaN pose, negative
//     timeout) throws out of the step exactly as the facade throws — a
//     programming error must stay loud and early, never be converted into a
//     polite "the chain stopped". A throwing step leaves the chain's counters
//     untouched (the bad call never ran).
//
// A routine that must CONTINUE past a failure (a sweep that shrugs off one
// missed goal) branches on the verbs directly — that is one step down the
// tier ladder, not a rewrite, and mixing the two styles in one routine is
// supported and tested (steps and direct facade calls interleave freely,
// because eager execution keeps program order = field order).
//
// ═══ then(): the mechanism seam (filled at F1; still unfrozen — see above) ═══════
// then() accepts any callable returning void (always succeeds), bool, or
// ExitReason (failure stops the chain like any step) — and, since F1, a
// mechanism operation's MechanismOutcome, where ONLY Succeeded continues and
// an Unconfirmed grab can never read as success (the T2 guarantee; the member
// comment carries the idiom). One honest spelling note: the action is a
// CALLABLE, so a member function is passed as `then([&] { intake.in(); })` —
// `then(intake.in)` is valid C++ only for a callable data member, and the
// documentation spells the lambda form everywhere for that reason (T5).
//
// ═══ What a recipe deliberately CANNOT do (the documented gaps) ══════════════════
//   * drive(speeds, Frame) — a recipe is a SEQUENCE; drive() is a per-
//     iteration loop primitive (teleop / expert escape hatch). A routine that
//     wants it calls the facade directly, mid-chain if it likes.
//   * cancel() — the panic stop belongs to whoever supervises the routine,
//     not to a step inside it (a step that cancels itself is just a shorter
//     chain). The chain uses it internally on failure.
//   * Branch on pose — recipes are position-blind between steps by design;
//     `chassis.pose()` mid-chain is the supported mixed-tier idiom.
//
// Single-task, like everything it wraps. Borrows the Chassis (must outlive
// this object). Copying is disabled: two handles sharing stop-state counters
// would let a stopped chain's twin keep driving.

#include <cmath>
#include <functional>
#include <initializer_list>
#include <span>
#include <type_traits>
#include <utility>

#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/robot_context.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::chassis {

/// Why a Routine stopped early. `None` = it never stopped. Append-only: F1/F3
/// mechanism failures arrive as new enumerators, never as re-meanings.
enum class RoutineStopCause {
    None,          ///< every executed step succeeded (chain still running or finished)
    MotionFailed,  ///< a motion step exited non-Settled (see RoutineResult::exit)
    WaitTimedOut,  ///< a waitFor() deadline passed with the condition still false
    ActionFailed,  ///< a then()-action reported failure (bool false / non-Settled)
    MechanismFailed,  ///< a then()-action returned a mechanism verdict other than
                      ///< Succeeded — Unconfirmed / Stalled / TimedOut / Cancelled;
                      ///< the stop log line names which, and the operation object
                      ///< itself remains the authority on the exact outcome
                      ///< (APPENDED at chunk F1, per the append-only rule above)
};

/// What a Routine did — the whole-chain verdict, readable at any point (it is
/// a snapshot; ask again after more steps). ExitReason alone would lose WHERE
/// the routine broke and WHY (a wait and a watchdog are different strategy
/// facts), exactly TrajectoryResult's argument one layer up.
struct RoutineResult {
    bool ok = true;         ///< no step has failed (and none skipped)
    int steps = 0;          ///< steps encountered so far, including skipped ones
    int completed = 0;      ///< steps that ran and succeeded
    int skipped = 0;        ///< steps skipped after the stop
    int stoppedAt = 0;      ///< 1-based index of the failing step; 0 = none
    const char* stoppedName = "";  ///< the failing step's verb/name ("" = none)
    /// WHAT kind of thing stopped the chain — read this BEFORE `exit`, because
    /// only `MotionFailed` puts a real motion verdict in `exit`.
    RoutineStopCause cause = RoutineStopCause::None;
    /// The failing MOTION step's verdict (TimedOut / Cancelled). `Running`
    /// means "no motion verdict here" — the stop was a wait or an action, or
    /// nothing stopped (the same "none yet" convention as CompletedMotion).
    control::ExitReason exit = control::ExitReason::Running;
};

/// The Tier-2 recipe chain: a complete autonomous routine as a sequence of
/// named steps, each delegating to exactly one Chassis verb, executed EAGERLY
/// (a step runs the moment it is chained) with one built-in failure policy —
/// stop, safe the drive, skip the rest, report. The file banner above explains
/// every one of those choices and is meant to be read.
class Routine {
public:
    /// Borrows `chassis` (must outlive the Routine). `name` appears in the
    /// stop/skip log lines so a transcript names WHICH routine stopped; it
    /// must be a stable literal (it is stored, not copied).
    explicit Routine(Chassis& chassis, const char* name = "routine") noexcept
        : chassis_{&chassis}, name_{name} {}

    /// Neither copyable nor movable, and there is no reset(): one chain is one
    /// run. Two handles sharing the stop-state counters would let a stopped
    /// chain's twin keep driving — the failure the whole error policy exists to
    /// prevent. Pass a `Routine&` to helpers (that is how you factor a routine
    /// into reusable steps); construct a new one for a new run.
    Routine(const Routine&) = delete;
    Routine& operator=(const Routine&) = delete;
    Routine(Routine&&) = delete;
    Routine& operator=(Routine&&) = delete;
    ~Routine() = default;

    // ── the steps (each: exactly one Chassis call; header semantics) ───────────────

    /// Seed the pose estimate with the measured starting pose — every auton's
    /// first line (heading stays IMU-owned, exactly Chassis::setPose).
    Routine& startAt(const math::Pose2d& pose) {
        if (skipIfStopped("startAt")) {
            return *this;
        }
        chassis_->setPose(pose);
        recordSuccess();
        return *this;
    }

    /// Drive to a FIELD pose — translation and rotation simultaneous (C1).
    Routine& moveTo(const math::Pose2d& target, const MotionOptions& options = {}) {
        return motionStep("moveTo", [&] { return chassis_->moveTo(target, options); });
    }

    /// Drive to the FIELD point (x, y), arriving FACING it: one moveTo whose
    /// target heading is the bearing from the live pose estimate to (x, y),
    /// computed when this step RUNS (after the steps before it). On tank,
    /// face(x, y) first so the approach is a line the drivetrain can follow
    /// (C1's honesty: tank verbs never plan turns — face() is YOUR turn).
    Routine& driveTo(units::Length x, units::Length y, const MotionOptions& options = {}) {
        if (skipIfStopped("driveTo")) {
            return *this;
        }
        const math::Angle bearing = bearingTo(x, y);
        return runMotion("driveTo",
                         chassis_->moveTo(math::Pose2d{x, y, bearing}, options));
    }

    /// Translate to FIELD (x, y) holding the current heading (Chassis::strafeTo;
    /// on tank an off-line target honestly exits TimedOut).
    Routine& strafeTo(units::Length x, units::Length y, const MotionOptions& options = {}) {
        return motionStep("strafeTo",
                          [&] { return chassis_->strafeTo(x, y, options); });
    }

    /// Rotate in place to a FIELD heading, always the short way (Chassis::turnTo).
    Routine& turnTo(math::Angle heading, const MotionOptions& options = {}) {
        return motionStep("turnTo", [&] { return chassis_->turnTo(heading, options); });
    }

    /// Rotate in place to FACE the field point (x, y): one turnTo whose target
    /// is the bearing from the live pose estimate to (x, y), computed when
    /// this step runs. "Face the goal" in field words; turnTo(atan2(...)) by
    /// hand is bit-identical.
    Routine& face(units::Length x, units::Length y, const MotionOptions& options = {}) {
        if (skipIfStopped("face")) {
            return *this;
        }
        const math::Angle bearing = bearingTo(x, y);
        return runMotion("face", chassis_->turnTo(bearing, options));
    }

    /// Chain waypoints as sequential moveTo legs (Chassis::followTrajectory —
    /// options apply PER LEG). The full TrajectoryResult stays readable via
    /// lastTrajectory(); an incomplete trajectory fails the step.
    Routine& followTrajectory(std::span<const math::Pose2d> waypoints,
                              const MotionOptions& options = {}) {
        if (skipIfStopped("followTrajectory")) {
            return *this;
        }
        lastTrajectory_ = chassis_->followTrajectory(waypoints, options);
        if (lastTrajectory_.succeeded()) {
            recordSuccess();
        } else {
            recordStop("followTrajectory", RoutineStopCause::MotionFailed,
                       lastTrajectory_.exit);
        }
        return *this;
    }

    /// Brace-list convenience: followTrajectory({a, b, c}).
    Routine& followTrajectory(std::initializer_list<math::Pose2d> waypoints,
                              const MotionOptions& options = {}) {
        return followTrajectory(std::span<const math::Pose2d>{waypoints.begin(),
                                                              waypoints.size()},
                                options);
    }

    /// Controlled stop: 0 V under Brake until the estimate certifies rest
    /// (Chassis::brake — a C4 candidate adopted into F6 at D2).
    Routine& brake(const MotionOptions& options = {}) {
        return motionStep("brake", [&] { return chassis_->brake(options); });
    }

    /// Actively hold the current pose for `duration` (Chassis::hold). Typed
    /// time (D2): hold(300_ms) — hold(0.3) does not compile.
    Routine& hold(units::Time duration, const MotionOptions& options = {}) {
        return motionStep("hold", [&] { return chassis_->hold(duration, options); });
    }

    /// Wait, doing nothing, for `duration` — the alliance-partner beat every
    /// real auton has. Motors keep their last state (after a settled motion:
    /// stopped); the world keeps advancing. Distinct from hold(): pause()
    /// does not energize the drive. A pure delegation to Chassis::wait (D2 —
    /// before the wait verb existed, this step carried its own Tier-3
    /// clock-deadline plumbing; that implementation moved down to the facade
    /// where both tiers get it). A pause cannot fail, logs nothing, and
    /// never stops the chain; nonsense input (NaN, <= 0) throws through
    /// untouched, exactly like every step.
    Routine& pause(units::Time duration) {
        if (skipIfStopped("pause")) {
            return *this;
        }
        chassis_->wait(duration);  // throws on nonsense; counters untouched then
        recordSuccess();
        return *this;
    }

    /// Wait until `pred()` holds, up to `timeout` (required and finite —
    /// C2's no-hang discipline). In a recipe the condition MATTERS: if the
    /// deadline passes with it still false, continuing the script would act
    /// on a state the field never reached, so the chain stops (WaitTimedOut).
    /// A wait whose timeout is a legitimate strategy branch belongs one tier
    /// down: `chassis.waitUntil(...)` directly, branching on the WaitResult.
    template <typename Pred>
    Routine& waitFor(Pred&& pred, units::Time timeout, const char* name = "waitFor") {
        if (skipIfStopped(name)) {
            return *this;
        }
        const motion::WaitResult w =
            chassis_->waitUntil(std::forward<Pred>(pred), timeout);
        if (w == motion::WaitResult::Satisfied) {
            recordSuccess();
        } else {
            recordStop(name, RoutineStopCause::WaitTimedOut, control::ExitReason::Running);
        }
        return *this;
    }

    /// Run an action between motions — THE MECHANISM SEAM (the one member
    /// deliberately outside F10, filled in at chunk F1). `action` is any
    /// callable taking nothing and returning
    ///   * void        — the action always succeeds,
    ///   * bool        — false fails the step and stops the chain,
    ///   * ExitReason  — non-Settled fails the step (so an action may wrap a
    ///                   facade verb and have its verdict honored),
    ///   * manipulation::MechanismOutcome — a mechanism operation's verdict:
    ///     ONLY Succeeded continues the chain; Unconfirmed / Stalled /
    ///     TimedOut / Cancelled stop it as MechanismFailed with the outcome
    ///     named in the stop line. MechanismOutcome has no bool conversion, so
    ///     an Unconfirmed can never be truthy by accident — the T2 guarantee
    ///     that a failed grab cannot read as success at this layer.
    /// The mechanism idiom (contract in manipulation/mechanism_op.hpp):
    ///
    ///     r.then([&] {
    ///         grab.start();
    ///         (void)chassis.waitUntil([&] { return grab.tick() != Running; },
    ///                                 Time{2.0});
    ///         return grab.outcome();
    ///     }, "grab");
    ///
    /// RETURN THE OUTCOME. A void lambda that runs an operation and drops its
    /// verdict "succeeds" whatever happened — the same sharp edge as dropping
    /// a direct facade call's ExitReason (guide chapter 9), owned the same way.
    /// `name` labels the step in stop/skip log lines (stable literal).
    template <typename Action>
    Routine& then(Action&& action, const char* name = "action") {
        if (skipIfStopped(name)) {
            return *this;
        }
        using R = std::invoke_result_t<Action&>;
        bool succeeded = true;
        control::ExitReason exit = control::ExitReason::Running;
        const char* detail = nullptr;
        RoutineStopCause failCause = RoutineStopCause::ActionFailed;
        if constexpr (std::is_void_v<R>) {
            std::invoke(action);
        } else if constexpr (std::is_same_v<R, control::ExitReason>) {
            exit = std::invoke(action);
            succeeded = (exit == control::ExitReason::Settled);
        } else if constexpr (std::is_same_v<R, manipulation::MechanismOutcome>) {
            const manipulation::MechanismOutcome mo = std::invoke(action);
            succeeded = (mo == manipulation::MechanismOutcome::Succeeded);
            failCause = RoutineStopCause::MechanismFailed;
            detail = manipulation::mechanismOutcomeName(mo);
        } else {
            static_assert(std::is_constructible_v<bool, R>,
                          "Routine::then: the action must return void, bool, "
                          "control::ExitReason, or manipulation::MechanismOutcome");
            succeeded = static_cast<bool>(std::invoke(action));
        }
        if (succeeded) {
            recordSuccess();
        } else {
            recordStop(name, failCause, exit, detail);
        }
        return *this;
    }

    // ── the whole-chain verdict ────────────────────────────────────────────────────

    /// True while no step has failed (and none was skipped).
    [[nodiscard]] bool ok() const noexcept { return stoppedAt_ == 0; }

    /// The chain verdict so far (a snapshot — see RoutineResult).
    [[nodiscard]] RoutineResult result() const noexcept {
        RoutineResult r;
        r.ok = ok();
        r.steps = steps_;
        r.completed = completed_;
        r.skipped = skipped_;
        r.stoppedAt = stoppedAt_;
        r.stoppedName = stoppedName_;
        r.cause = cause_;
        r.exit = exit_;
        return r;
    }

    /// The most recent followTrajectory step's full result — completedLegs is
    /// strategy-relevant and must not be flattened away by the chain. Before
    /// any trajectory has run it reads `exit = Running`, the project's "no
    /// verdict here yet" convention (RoutineResult::exit, CompletedMotion), so
    /// succeeded() is honestly FALSE on a virgin routine. (D3: a plain
    /// value-initialized TrajectoryResult reports `Settled` with 0 of 0 legs,
    /// which succeeded() calls SUCCESS — correct for the facade, whose verb
    /// requires at least one waypoint, and a lie here, where the member exists
    /// before any trajectory does.)
    [[nodiscard]] const TrajectoryResult& lastTrajectory() const noexcept {
        return lastTrajectory_;
    }

    /// The chassis this routine drives — the mixed-tier seam, spelled out.
    /// (You can equally keep your own reference; this exists so a routine
    /// passed across a function boundary still reaches Tier 3.)
    [[nodiscard]] Chassis& chassis() noexcept { return *chassis_; }

private:
    /// Bearing from the live pose estimate to field point (x, y). The ONLY
    /// arithmetic in this layer: argument computation (author intent → verb
    /// vocabulary), not motion logic. Rejects the degenerate "bearing to the
    /// point I am already at" — 1e-9 in is numerical-degeneracy territory, not
    /// an intent check (a 0.1 in nudge is legitimate and passes).
    [[nodiscard]] math::Angle bearingTo(units::Length x, units::Length y) const {
        const math::Pose2d here = chassis_->pose();
        const double dx = (x - here.x()).value();
        const double dy = (y - here.y()).value();
        SHULIB_PRECONDITION(std::hypot(dx, dy) > 1e-9,
                            "Routine: face/driveTo target is (numerically) where the "
                            "robot already is — no defined bearing");
        return math::Angle::radians(std::atan2(dy, dx));
    }

    /// One motion-verb step: run `f` (which may throw the facade's
    /// preconditions — deliberately uncaught, and the counters stay untouched
    /// because nothing is recorded until `f` returns), then record.
    template <typename F>
    Routine& motionStep(const char* name, F&& f) {
        if (skipIfStopped(name)) {
            return *this;
        }
        return runMotion(name, f());
    }

    Routine& runMotion(const char* name, control::ExitReason exit) {
        if (exit == control::ExitReason::Settled) {
            recordSuccess();
        } else {
            recordStop(name, RoutineStopCause::MotionFailed, exit);
        }
        return *this;
    }

    /// True (and the step recorded as skipped) when the chain has stopped.
    [[nodiscard]] bool skipIfStopped(const char* name) {
        if (stoppedAt_ == 0) {
            return false;
        }
        ++steps_;
        ++skipped_;
        diag::lineformat::Line line;
        line.appendLiteral("routine '");
        line.appendLiteral(name_);
        line.appendLiteral("': step ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(steps_));
        line.appendLiteral(" (");
        line.appendLiteral(name);
        line.appendLiteral(") skipped — stopped at step ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(stoppedAt_));
        chassis_->deps().ctx->telemetry().log(hal::LogLevel::Info, "RTN", line.view());
        return true;
    }

    void recordSuccess() noexcept {
        ++steps_;
        ++completed_;
    }

    /// The stop half of the error policy: record, SAFE the drive (cancel is
    /// idempotent and defined-safe with or without an active motion), Warn.
    /// `detail` (optional) names the specific verdict inside the cause — F1's
    /// mechanism outcomes use it, so a transcript says WHICH way a grab failed.
    /// Note the layering: a mechanism operation has already safed its OWN
    /// mechanism on every exit path (mechanism_op.hpp); this stop parks the
    /// DRIVE on top of that.
    void recordStop(const char* name, RoutineStopCause cause, control::ExitReason exit,
                    const char* detail = nullptr) {
        ++steps_;
        stoppedAt_ = steps_;
        stoppedName_ = name;
        cause_ = cause;
        exit_ = exit;
        chassis_->cancel();  // 0 V + Brake — the routine parks where it stopped
        diag::lineformat::Line line;
        line.appendLiteral("routine '");
        line.appendLiteral(name_);
        line.appendLiteral("' STOPPED at step ");
        diag::lineformat::appendUnsigned(line, static_cast<unsigned long>(stoppedAt_));
        line.appendLiteral(" (");
        line.appendLiteral(name);
        line.appendLiteral("): ");
        line.appendLiteral(stopCauseText(cause, exit));
        if (detail != nullptr) {
            line.appendLiteral(" (");
            line.appendLiteral(detail);
            line.appendLiteral(")");
        }
        line.appendLiteral(" — skipping the rest; drive safed");
        chassis_->deps().ctx->telemetry().log(hal::LogLevel::Warn, "RTN", line.view());
    }

    [[nodiscard]] static const char* stopCauseText(RoutineStopCause cause,
                                                   control::ExitReason exit) noexcept {
        switch (cause) {
            case RoutineStopCause::None:
                return "no stop";  // unreachable from recordStop; total switch anyway
            case RoutineStopCause::MotionFailed:
                switch (exit) {
                    case control::ExitReason::TimedOut: return "motion TIMEOUT";
                    case control::ExitReason::Cancelled: return "motion CANCELLED";
                    case control::ExitReason::Running:
                    case control::ExitReason::Settled: break;
                }
                return "motion failed";
            case RoutineStopCause::WaitTimedOut:
                return "wait TIMEOUT";
            case RoutineStopCause::ActionFailed:
                return "action FAILED";
            case RoutineStopCause::MechanismFailed:
                return "mechanism FAILED";
        }
        return "stopped";
    }

    Chassis* chassis_;
    const char* name_;
    /// `Running` = "no trajectory has run yet" (see lastTrajectory()). NOT a
    /// value-initialized TrajectoryResult: that one claims success.
    TrajectoryResult lastTrajectory_{.exit = control::ExitReason::Running};
    int steps_ = 0;
    int completed_ = 0;
    int skipped_ = 0;
    int stoppedAt_ = 0;  // 1-based; 0 = chain still clean
    const char* stoppedName_ = "";
    RoutineStopCause cause_ = RoutineStopCause::None;
    control::ExitReason exit_ = control::ExitReason::Running;
};

}  // namespace shulib::chassis
