#pragma once
//
// Routine — the Tier-2 recipe layer (chunk D1, WS12/M7 pulled forward; master
// plan §17).
//
// ═══ STATUS: FROZEN — F10, LOCKED 2026-08-12 (chunk D3, API 2.0) ══════════════════
// This surface is FROZEN, as its own register row (F10) rather than as part of
// F6: the facade it delegates to is a different tier and can version
// independently. Frozen: the constructor (both spellings, noexcept),
// non-copyable/non-movable, THIRTEEN steps — startAt / moveTo / driveTo /
// strafeTo / turnTo / face / followTrajectory (span + brace) / brake / hold /
// pause / waitFor, and reverseTo / faceAway, the two APPENDED at chunk L9
// (API 2.4) by exactly the additive path below and pinned the day they
// landed — the four observers ok / result / lastTrajectory / chassis,
// the types RoutineResult (all NINE fields — mechanismVerdict appended at
// MECH-A4, API 2.5, its type pinned) and RoutineStopCause (append-only,
// existing values fixed), the documented error policy below, and typed time as
// a SEMANTIC (hold(0.3) must not compile). Enforced structurally:
// test/routine_signature_pin_test.cpp fails the build, naming F10, if any of
// those drifts. Changes only with a major API-version bump plus a migration
// note (include/shulib/version.hpp); ADDITIVE growth — new steps, new
// observers, new RoutineResult fields, appended RoutineStopCause enumerators —
// stays legal and is the intended path (F1 used it for the stop cause, L9 for
// two steps: no existing member moved either time).
//
// The MECHANISM STEPS (the mechanism core, chunk MECH-A4, API 2.5) took the same
// path, and what they added is in ONE delimited block below ("MECH-A4: the
// mechanism steps"): the appended stop cause MechanismStepFailed (its value
// pinned, like every enumerator's), the appended RoutineResult field
// mechanismVerdict, a SECOND constructor `Routine r{ctx, "name"}` (a new
// overload — the frozen `Routine(Chassis&, const char*)` is untouched), the
// steps move / startMove / home / startHome / run / startRun / set / spinUp /
// startSpinUp / stop / waitUntilDone, the observer lastMechanism(), and four
// refusal templates that name the right step when a team writes the wrong one.
// NOT pinned, and NOT YET RULED: whether the mechanism steps, the context
// constructor and lastMechanism() join this freeze is an OPEN decision (the
// Freeze Register's F10 row says so). Proposed at MECH-A4: not yet — every one
// of them takes the mechanism seam's types (register row F11, NOT frozen), as
// then() below does, and then() stays out of F10 for exactly that reason. The
// other reading is the additive path L9 took (its two steps pinned the day they
// landed). Until it is ruled they are unpinned: pinning them later is additive,
// un-freezing them would not be. Stated out loud for the reason the then()
// paragraph gives: silence in a freeze reads as "frozen too".
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
// here: no gains, no kinematics, no exit criteria, no fault policy. The four
// "field vocabulary" steps — face(x, y) and driveTo(x, y), and their reversed
// twins faceAway(x, y) and reverseTo(x, y) (L9, API 2.4) — each compute ONE
// argument (the bearing from the live pose estimate to a field point — pure
// trig — plus π for the reversed pair) and then delegate to turnTo / moveTo;
// that keeps C1's D12 intact on tank drives (the AUTHOR still writes the turn
// — face() IS the author's turn, said in field words, and faceAway() is the
// same turn for backing into a point), and a hand-written turnTo(atan2(...))
// — or turnTo(atan2(...) + π) — remains bit-identical to it by construction.
// Since TANK1 (API 2.3) face() is also what makes a
// tank's driveTo leg steerable: a tank moveTo steers back onto its line only
// for a leg that starts within 15° of its target heading, and a leg right
// after face() starts within about a degree of it — the library still plans
// no turn (chassis.hpp: TANK STEERING). faceAway() does the same for a
// reverseTo leg, which starts as far off its heading as the robot's BACK is
// turned from the point: 180° from a robot facing the point (not steered),
// about 0 after driving straight out from it (steered, no faceAway() needed).
// Dropping from Tier 2 to Tier 3 is deleting the Routine object and keeping
// every verb.
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
// Since MECH-A4 a robot's DECLARED mechanisms need no then(): each has its own
// steps — `r.move(arm["score"], 1.5_s)`, `r.run(intake, 12_volt, seesPiece,
// 3_s)` — run by the mechanism service, with their own verdict policy (the
// MECH-A4 block below). then() stays the seam for anything else.
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
#include <concepts>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <initializer_list>
#include <span>
#include <string_view>
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
#include "shulib/mechanism/service.hpp"  // MECH-A4: the mechanism steps run through the service
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::chassis {

namespace detail {
/// MECH-A4: what the context constructor reads — a `chassis` (a Chassis&) and a `mechanismService`
/// (a MechanismService*): competition::AutonContext, or a route binder's own two-member context.
template <class Context>
concept RoutineContext = requires(Context& c) {
    { c.chassis } -> std::convertible_to<Chassis&>;
    { c.mechanismService } -> std::convertible_to<mechanism::MechanismService*>;
};
/// MECH-A4: a mechanism DECLARATION (an Arm, a Lift, a Flywheel, a Power, a Pneumatic) — it has a
/// `.name`.
template <class Declared>
concept MechanismDeclaration = requires(const Declared& d) { d.name.text; };
}  // namespace detail

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
    MechanismStepFailed,  ///< a mechanism STEP (move, run, set, spinUp, home, stop, waitUntilDone —
                          ///< and their start forms' starts) ended other than SUCCEEDED or an
                          ///< UNCONFIRMED counted done; the verdict is RoutineResult::mechanismVerdict
                          ///< and the stop line names it. MechanismFailed keeps its F1 meaning (a
                          ///< then()-action's verdict). (APPENDED at chunk MECH-A4, API 2.5.)
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
    /// The failing MECHANISM step's verdict — TimedOut / Stalled / Absent / NotHomed / Refused /
    /// Cancelled / an Unconfirmed NOT counted done. `Running` means "no mechanism verdict here":
    /// read `cause` first — only `MechanismStepFailed` puts one here. (APPENDED at chunk MECH-A4,
    /// API 2.5.)
    manipulation::MechanismOutcome mechanismVerdict = manipulation::MechanismOutcome::Running;
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

    /// MECH-A4 (API 2.5; a NEW overload — the frozen constructor above is untouched): `Routine
    /// r{ctx, "score-three"}` in an auton — the chassis AND the robot's mechanism service, from the
    /// auton's context (competition::AutonContext; any type with these two members). Borrows both
    /// (each must outlive the Routine). A program that runs autons without a service leaves
    /// `mechanismService` null: the chassis steps run as ever, and a mechanism step is a loud
    /// precondition naming the fix. Not frozen (the file header).
    template <detail::RoutineContext Context>
    explicit Routine(Context& ctx, const char* name = "routine") noexcept
        : chassis_{&ctx.chassis}, name_{name}, mechs_{ctx.mechanismService} {}

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
    /// (C1's honesty: tank verbs never plan turns — face() is YOUR turn); it is
    /// also what makes the leg steerable — a tank moveTo steers back onto its
    /// line only when the leg starts within 15° of its target heading, and logs
    /// "turn first: face()" when it does not (chassis.hpp: TANK STEERING).
    Routine& driveTo(units::Length x, units::Length y, const MotionOptions& options = {}) {
        if (skipIfStopped("driveTo")) {
            return *this;
        }
        const math::Angle bearing = bearingTo(x, y);
        return runMotion("driveTo",
                         chassis_->moveTo(math::Pose2d{x, y, bearing}, options));
    }

    /// Drive to the FIELD point (x, y) and arrive with the robot's BACK toward it: one moveTo
    /// whose target heading is the bearing from the live pose estimate to (x, y) plus π, computed
    /// when this step RUNS — driveTo turned around, and exactly a route's DriveTo waypoint with
    /// `reverse` set (runner/path_runner.hpp spells the `+ π` the same way, so a route and a recipe
    /// that name the same leg stay bit-identical). The caller's options pass through whole.
    /// The leg starts as far off its target heading as the robot's back is turned from the point,
    /// so after faceAway(x, y) it drives BACKWARDS the whole way, on any drive. Without it: a
    /// holonomic drive rotates in flight — it still arrives turned round, but its nose leads early
    /// in the leg. A tank never plans the turn (faceAway() is YOUR turn) and steers a leg back onto
    /// its line only when it starts within 15° of its target heading: from a robot FACING the
    /// point it starts 180° off, is not steered, logs "turn first: face()" — for a backwards leg
    /// that turn is faceAway() — and times out beside the point (chassis.hpp: TANK STEERING); a
    /// tank that just drove straight out from the point already has its back to it and needs no
    /// faceAway(). No robot has run a reverseTo yet (robot two has reversed only under driver
    /// control): whether a tank backs up as far as its calibration says is register entry HA-167.
    /// (Appended at chunk L9, API 2.4.)
    Routine& reverseTo(units::Length x, units::Length y, const MotionOptions& options = {}) {
        if (skipIfStopped("reverseTo")) {
            return *this;
        }
        // Angle re-wraps, so + π is the whole operation (path_runner.hpp's DriveTo, verbatim).
        const math::Angle away = bearingTo(x, y) + math::Angle::radians(math::Angle::kPi);
        return runMotion("reverseTo", chassis_->moveTo(math::Pose2d{x, y, away}, options));
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

    /// Rotate in place so the robot's BACK points at the FIELD point (x, y): one turnTo whose
    /// target is the bearing from the live pose estimate to (x, y) plus π, computed when this
    /// step runs — face() turned around, and the turn that goes before a reverseTo(x, y) on a
    /// tank, or on any drive whose back must lead the whole way ("back into the goal" in field
    /// words). The caller's options pass through whole (a turn in place reads the timeout and
    /// the yaw budget); turnTo(atan2(...) + π) by hand is bit-identical. (Appended at chunk L9,
    /// API 2.4.)
    Routine& faceAway(units::Length x, units::Length y, const MotionOptions& options = {}) {
        if (skipIfStopped("faceAway")) {
            return *this;
        }
        const math::Angle away = bearingTo(x, y) + math::Angle::radians(math::Angle::kPi);
        return runMotion("faceAway", chassis_->turnTo(away, options));
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

    // ═══ MECH-A4: the mechanism steps — BEGIN (API 2.5; not frozen — the file header) ═════════
    // ONE eager call per step, each naming its mechanism by its DECLARATION (or a goal the
    // declaration made — a preset, a constant, a computed target), each run by the robot's
    // mechanism service (mechanism/service.hpp), which this Routine reaches through the context
    // constructor above. A BLOCKING step commands, then waits for the step's verdict within its
    // `bound` — the step's own time limit, always stated; the service's deadline ends the step,
    // and this wait's backstop is the bound plus kMechanismWaitSlackSeconds. A START form commands
    // and returns at once (the next drive runs while the service ticks the step);
    // waitUntilDone() collects it later.
    //
    //   r.move(arm["score"], 1.5_s);                       // a preset (checked at BUILD)
    //   r.move(arm.to(120), 1.5_s);                        // a constant (past a limit: no build)
    //   r.move(lift.clampedTo(h), 2_s);                    // a computed one (clamped, logged)
    //   r.run(intake, 12_volt, seesPiece, 3_s);            // until its sensor sees a piece
    //   r.run(intake, 12_volt, seesPiece.orAfter(2.5_s), 3_s);  // ... or after 2.5 s
    //   r.run(clawPivot6, 12_volt, 0.4_s);                 // for 0.4 s
    //   r.set(claw["closed"]);                             // a pneumatic position
    //   r.spinUp(flywheel["shoot"], 2_s);                  // until "at speed"; it keeps spinning
    //   r.home(lift, 1.5_s);  r.stop(flywheel);
    //   r.startRun(intake, 12_volt, seesPiece, 3_s).driveTo(40_in, 0_in).waitUntilDone(intake, 1_s);
    //
    // THE VERDICT POLICY (each rule pinned by a test): SUCCEEDED continues; UNCONFIRMED COUNTED
    // DONE (no sensor exists to confirm it — the team lead's Q2 (c): robot two's sensorless claw)
    // continues; EVERY other verdict stops the chain as RoutineStopCause::MechanismStepFailed, with
    // the verdict in the stop line, in RoutineResult::mechanismVerdict and in lastMechanism(). A
    // START form stops the chain only when the start itself is ABSENT, REFUSED or NOT_HOMED. A step
    // on a mechanism this robot's list does not have is ABSENT, said, and stops the chain. A
    // blocking wait during which the service NEVER ticked stops the chain TIMED_OUT, saying the
    // service is not in the chassis pacer's chain (the program's wiring fault, found on the first
    // blocking step). After its step a mechanism keeps its behaviour — an arm HOLDS, a flywheel
    // keeps spinning — until another command, the run guard's end of run, or driver control.
    //
    // THE HONEST LIMIT: the service ticks only inside a shulib motion, wait or pause (and the
    // driver loop). An auton's own busy loop — `pros::delay`, a then() action that blocks — STARVES
    // it, and a starved service leaves every mechanism at its LAST command: not held, not limited,
    // not safe. In an auton, wait with r.pause() / r.waitFor().
    //
    // PRECONDITIONS (thrown, the error policy above: the bad call never ran, the counters are
    // untouched): the Routine was built from a context with a mechanism service; a bound finite and
    // > 0; and every precondition of the service's own command (a time condition longer than its
    // bound, a sensor condition on a mechanism that declares no piece sensor, `stalls` with no
    // stall signature, homing a mechanism declared neverHomed, ...).

    /// Move an arm or a lift to `goal` within `bound`, then HOLD it there (`arm["score"]`,
    /// `arm.to(120)`, `lift.to(18_in)`, `lift.clampedTo(h)`). Out of time: TIMED_OUT, and it holds
    /// where it stopped.
    Routine& move(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr) {
        return positionStep(goal, bound, name, /*wait=*/true);
    }
    /// move()'s START form: commands the move and returns at once; waitUntilDone() collects it.
    Routine& startMove(mechanism::PositionGoal goal, units::Time bound, const char* name = nullptr) {
        return positionStep(goal, bound, name, /*wait=*/false);
    }
    /// Find an ARM's zero by its declaration's homing method, within `bound` (then it holds there).
    Routine& home(const mechanism::Arm& m, units::Time bound, const char* name = nullptr) {
        return homeStep(m, bound, name, /*wait=*/true);
    }
    /// Find a LIFT's zero by its declaration's homing method, within `bound`.
    Routine& home(const mechanism::Lift& m, units::Time bound, const char* name = nullptr) {
        return homeStep(m, bound, name, /*wait=*/true);
    }
    /// home()'s START form, for an arm (homing while the robot drives).
    Routine& startHome(const mechanism::Arm& m, units::Time bound, const char* name = nullptr) {
        return homeStep(m, bound, name, /*wait=*/false);
    }
    /// home()'s START form, for a lift.
    Routine& startHome(const mechanism::Lift& m, units::Time bound, const char* name = nullptr) {
        return homeStep(m, bound, name, /*wait=*/false);
    }
    /// Run a POWER mechanism at `volts` until `until` holds (seesPiece, seesPiece(grip),
    /// stopsSeeingPiece, stalls, `seesPiece.orAfter(2.5_s)`), within `bound`; its declared safe
    /// state at the end, however it ends.
    Routine& run(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound,
                 const char* name = nullptr) {
        return powerStep(m, volts, until, bound, name, /*wait=*/true);
    }
    /// Run a POWER mechanism at `volts` for `time`: SUCCEEDED when the time is up.
    Routine& run(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr) {
        return powerStep(m, volts, mechanism::Until::time(time), time, name, /*wait=*/true);
    }
    /// run()'s START form, with a done-condition.
    Routine& startRun(const mechanism::Power& m, units::Voltage volts, mechanism::Until until, units::Time bound,
                      const char* name = nullptr) {
        return powerStep(m, volts, until, bound, name, /*wait=*/false);
    }
    /// run()'s START form, for a time.
    Routine& startRun(const mechanism::Power& m, units::Voltage volts, units::Time time, const char* name = nullptr) {
        return powerStep(m, volts, mechanism::Until::time(time), time, name, /*wait=*/false);
    }
    /// Put a PNEUMATIC mechanism in a named position (`claw["closed"]`). Its time is the robot
    /// file's `.moveTime` (plus `.confirmWithin` when a sensor confirms it), so the step states no
    /// bound. A `grabs` position with no sensor ends UNCONFIRMED, counted done: the chain goes on.
    Routine& set(mechanism::PneumaticGoal goal, const char* name = nullptr) {
        const char* step = name != nullptr ? name : goal.mechanismName;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::PneumaticSlot* m = mechs().find(goal);
        if (m == nullptr) {
            return notOnThisRobot(goal.mechanismName, step);
        }
        m->set(goal.position);
        noteStarted(*m);
        return awaitMechanism(*m, units::Time{m->lastStepBound()}, step);
    }
    /// Spin a FLYWHEEL up (`flywheel["shoot"]`, `flywheel.at(3000)`, `flywheel.clampedTo(x)`): the
    /// step SUCCEEDS at speed and the wheel KEEPS spinning until stop(), the guard or driver control.
    Routine& spinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr) {
        return speedStep(goal, bound, name, /*wait=*/true);
    }
    /// spinUp()'s START form (the wheel spins up while the robot drives).
    Routine& startSpinUp(mechanism::SpeedGoal goal, units::Time bound, const char* name = nullptr) {
        return speedStep(goal, bound, name, /*wait=*/false);
    }
    /// Stop a POWER mechanism now — its declared safe state (a running step on it ends CANCELLED).
    /// SUCCEEDED, unless it is ABSENT this boot or ANOTHER owner holds its claim (REFUSED: the stop
    /// did nothing).
    Routine& stop(const mechanism::Power& m, const char* name = nullptr) {
        const char* step = name != nullptr ? name : m.name.text;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::PowerSlot* s = mechs().find(m);
        if (s == nullptr) {
            return notOnThisRobot(m.name.text, step);
        }
        const std::uint32_t before = s->commandSeq();
        s->stop();
        return stoppedMechanism(*s, before, step);
    }
    /// Stop a FLYWHEEL now — its declared safe state. As stop(const Power&).
    Routine& stop(const mechanism::Flywheel& m, const char* name = nullptr) {
        const char* step = name != nullptr ? name : m.name.text;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::SpeedSlot* s = mechs().find(m);
        if (s == nullptr) {
            return notOnThisRobot(m.name.text, step);
        }
        const std::uint32_t before = s->commandSeq();
        s->stop();
        return stoppedMechanism(*s, before, step);
    }
    /// Wait (at most `bound` more) for the verdict of the step THIS routine last began on `m` — a
    /// START form's, usually; a step that already ended answers at once. A step that something
    /// OUTSIDE this routine replaced meanwhile ended CANCELLED, and so does this one.
    /// PRECONDITIONS: `bound` finite and > 0; this routine began a step on `m`.
    template <detail::MechanismDeclaration Declared>
    Routine& waitUntilDone(const Declared& m, units::Time bound, const char* name = nullptr) {
        const char* step = name != nullptr ? name : m.name.text;
        if (skipIfStopped(step)) {
            return *this;
        }
        SHULIB_PRECONDITION(std::isfinite(bound.value()) && bound.value() > 0.0,
                            "r.waitUntilDone(...): the time limit must be finite and above 0");
        auto* s = mechs().find(m);
        if (s == nullptr) {
            return notOnThisRobot(m.name.text, step);
        }
        const std::uint32_t mine = startedSeq_[s->index()];
        SHULIB_PRECONDITION(mine != 0,
                            "r.waitUntilDone(...): this routine started no step on that mechanism -- start one "
                            "first (r.startRun / r.startMove / r.startSpinUp / r.startHome)");
        if (s->stepsBegun() != mine) {
            // Something outside this routine began a newer step on it: the one this routine started
            // was replaced (CANCELLED by the service, said) and has no verdict of its own to wait for.
            lastMech_ = mechanism::StepReport{.outcome = manipulation::MechanismOutcome::Cancelled};
            mechStop_ = manipulation::MechanismOutcome::Cancelled;
            recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                       "CANCELLED: a newer command replaced the step this routine started");
            return *this;
        }
        return awaitMechanism(*s, bound, step);
    }

    // The WRONG STEP FOR THE KIND, refused BY NAME where the team calls it: each is a template only
    // so its static_assert fires at the team's line ("required from here"), never as a list of the
    // overloads that did not match.

    /// `r.run(arm, ...)`: refused — run() is a power mechanism's step.
    template <class D, class... Rest>
        requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> ||
                 std::same_as<D, mechanism::Flywheel> || std::same_as<D, mechanism::Pneumatic>)
    Routine& run(const D&, Rest&&...) {
        static_assert(sizeof(D) == 0,
                      "r.run(...) is for a POWER mechanism -- move an arm or a lift with r.move(arm[\"score\"], "
                      "1.5_s), spin a flywheel up with r.spinUp(flywheel[\"shoot\"], 2_s), set a pneumatic with "
                      "r.set(claw[\"open\"])");
        return *this;
    }
    /// `r.run(intake, 12, 1_s)`: refused — volts carry their unit.
    template <class V, class... Rest>
        requires std::is_arithmetic_v<V>
    Routine& run(const mechanism::Power&, V, Rest&&...) {
        static_assert(sizeof(V) == 0, "volts are written with their unit: 12_volt, not 12");
        return *this;
    }
    /// `r.move(intake, ...)` / `r.move(arm, ...)`: refused — move() takes a TARGET.
    template <class D, class... Rest>
        requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> ||
                 std::same_as<D, mechanism::Power> || std::same_as<D, mechanism::Flywheel> ||
                 std::same_as<D, mechanism::Pneumatic>)
    Routine& move(const D&, Rest&&...) {
        static_assert(sizeof(D) == 0,
                      "r.move(...) takes a TARGET: arm[\"score\"], arm.to(95), lift.to(18_in) (a power mechanism "
                      "runs: r.run(intake, 12_volt, 1_s); a pneumatic is set: r.set(claw[\"open\"]))");
        return *this;
    }
    /// `r.stop(arm)`: refused — an arm or a lift holds, a pneumatic keeps its position.
    template <class D, class... Rest>
        requires(std::same_as<D, mechanism::Arm> || std::same_as<D, mechanism::Lift> ||
                 std::same_as<D, mechanism::Pneumatic>)
    Routine& stop(const D&, Rest&&...) {
        static_assert(sizeof(D) == 0,
                      "r.stop(...) is for a power mechanism or a flywheel -- an arm or a lift HOLDS where its "
                      "last step left it, and a pneumatic keeps its position (r.set(claw[\"open\"]))");
        return *this;
    }

    /// The most recent mechanism step's report — its verdict, what ended it, whether an UNCONFIRMED
    /// was counted done, how long it took. Before any mechanism step: the default report (`Running`).
    [[nodiscard]] const mechanism::StepReport& lastMechanism() const noexcept { return lastMech_; }
    // ═══ MECH-A4: the mechanism steps — END ═══════════════════════════════════════════════════

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
        r.mechanismVerdict = mechStop_;  // MECH-A4
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
                            "Routine: face/driveTo/faceAway/reverseTo target is (numerically) "
                            "where the robot already is — no defined bearing");
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
            case RoutineStopCause::MechanismStepFailed:
                return "mechanism step FAILED";
        }
        return "stopped";
    }

    // ═══ MECH-A4: the mechanism steps' machinery — BEGIN ══════════════════════════════════════
    /// The wait's backstop past a step's own bound (the service's deadline ends the step first).
    static constexpr double kMechanismWaitSlackSeconds = 0.25;

    /// The service, or a loud precondition naming the fix (a Routine built from a bare Chassis, or
    /// from a context whose program passed no service).
    [[nodiscard]] mechanism::MechanismService& mechs() const {
        SHULIB_PRECONDITION(mechs_ != nullptr,
                            "Routine: a mechanism step needs the robot's mechanism service -- construct it as "
                            "Routine r{ctx, \"name\"} in an auton, in a program that passes its mechanism service "
                            "to runAuton");
        return *mechs_;
    }
    /// Remember that THIS routine began the slot's current step (waitUntilDone() collects only it).
    void noteStarted(const mechanism::Slot& s) noexcept {
        if (s.index() < mechanism::kMaxMechanisms) {
            startedSeq_[s.index()] = s.stepsBegun();
        }
    }
    /// Wait for `m`'s step, then record it under the verdict policy (the block's header).
    Routine& awaitMechanism(mechanism::Slot& m, units::Time bound, const char* step) {
        const long ticks0 = mechs().ticks();
        if (m.running()) {
            (void)chassis_->waitUntil([&m] { return !m.running(); },
                                      units::Time{bound.value() + kMechanismWaitSlackSeconds});
        }
        lastMech_ = m.lastStep();
        if (m.running()) {
            // The WAIT ran out before the step's verdict: the chain stops TIMED_OUT, said. The step
            // itself runs on to its own bound (the guard's cancel-all ends it with the run).
            mechStop_ = manipulation::MechanismOutcome::TimedOut;
            lastMech_.outcome = manipulation::MechanismOutcome::TimedOut;
            recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                       mechs().ticks() == ticks0
                           ? "TIMED_OUT: the mechanism service never ticked -- it is not in the chassis pacer's chain"
                           : "TIMED_OUT: no verdict within the wait's bound; the step runs on to its own");
            return *this;
        }
        const manipulation::MechanismOutcome o = lastMech_.outcome;
        if (o == manipulation::MechanismOutcome::Succeeded ||
            (o == manipulation::MechanismOutcome::Unconfirmed && lastMech_.countedDone)) {
            recordSuccess();
        } else {
            mechStop_ = o;
            recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                       manipulation::mechanismOutcomeName(o));
        }
        return *this;
    }
    /// A START form's record: the chain stops only when the start itself was ABSENT, REFUSED or
    /// NOT_HOMED (nothing runs); anything else — still running, or already done — continues.
    Routine& startedMechanism(const mechanism::Slot& m, const char* step) {
        lastMech_ = m.lastStep();
        const manipulation::MechanismOutcome o = lastMech_.outcome;
        if (o == manipulation::MechanismOutcome::Absent || o == manipulation::MechanismOutcome::Refused ||
            o == manipulation::MechanismOutcome::NotHomed) {
            mechStop_ = o;
            recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                       manipulation::mechanismOutcomeName(o));
        } else {
            recordSuccess();
        }
        return *this;
    }
    /// A stop step's record. The slot's stop() bumps the command number exactly when it acted; when
    /// it did not, the mechanism is ABSENT this boot or ANOTHER owner holds its claim (REFUSED) —
    /// either way nothing was stopped, said in the step's MECH line format, and the chain stops.
    Routine& stoppedMechanism(const mechanism::Slot& s, std::uint32_t seqBefore, const char* step) {
        if (s.commandSeq() != seqBefore) {
            lastMech_ = mechanism::StepReport{.outcome = manipulation::MechanismOutcome::Succeeded};
            recordSuccess();
            return *this;
        }
        const manipulation::MechanismOutcome o =
            s.built() ? manipulation::MechanismOutcome::Refused : manipulation::MechanismOutcome::Absent;
        char line[200];
        if (s.built()) {
            std::snprintf(line, sizeof line, "'%.23s': REFUSED in 0.00s (another owner holds its claim: the stop did "
                          "nothing)", s.name());
        } else {
            std::snprintf(line, sizeof line, "'%.23s': ABSENT in 0.00s (not built at boot: %.120s)", s.name(),
                          s.absence());
        }
        chassis_->deps().ctx->telemetry().log(hal::LogLevel::Warn, "MECH", line);
        lastMech_ = mechanism::StepReport{.outcome = o};
        mechStop_ = o;
        recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                   manipulation::mechanismOutcomeName(o));
        return *this;
    }
    /// A step whose mechanism this robot's service cannot find: ABSENT, said, and the chain stops —
    /// nothing was commanded. Three sentences: the service is not bound at all, no mechanism of that
    /// NAME on this robot, or one of that name that is not what the step names (another robot's
    /// declaration — a different kind, units, preset or position; autons are per robot).
    Routine& notOnThisRobot(const char* mechanismName, const char* step) {
        const char* n = mechanismName != nullptr ? mechanismName : "?";
        char line[220];
        if (!mechs().bound()) {
            std::snprintf(line, sizeof line,
                          "'%.23s': the mechanism service is not bound (the program bound no robot list): the step "
                          "did nothing",
                          n);
        } else if (mechs().slotFor(mechanismName) == nullptr) {
            std::snprintf(line, sizeof line,
                          "'%.23s' is not on this robot (its wiring list does not have it): the step did nothing", n);
        } else {
            std::snprintf(line, sizeof line,
                          "'%.23s' on this robot is not the mechanism this step names (another robot's declaration: "
                          "a different kind, units, preset or position): the step did nothing",
                          n);
        }
        chassis_->deps().ctx->telemetry().log(hal::LogLevel::Warn, "MECH", line);
        lastMech_ = mechanism::StepReport{.outcome = manipulation::MechanismOutcome::Absent};
        mechStop_ = manipulation::MechanismOutcome::Absent;
        recordStop(step, RoutineStopCause::MechanismStepFailed, control::ExitReason::Running,
                   manipulation::mechanismOutcomeName(manipulation::MechanismOutcome::Absent));
        return *this;
    }
    Routine& positionStep(const mechanism::PositionGoal& goal, units::Time bound, const char* name, bool wait) {
        const char* step = name != nullptr ? name : goal.mechanismName;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::PositionSlot* m = mechs().find(goal);
        if (m == nullptr) {
            return notOnThisRobot(goal.mechanismName, step);
        }
        m->moveTo(goal.value, bound, goal.computed);
        noteStarted(*m);
        return wait ? awaitMechanism(*m, bound, step) : startedMechanism(*m, step);
    }
    template <class Declared>
    Routine& homeStep(const Declared& d, units::Time bound, const char* name, bool wait) {
        const char* step = name != nullptr ? name : d.name.text;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::PositionSlot* m = mechs().find(d);
        if (m == nullptr) {
            return notOnThisRobot(d.name.text, step);
        }
        m->home(bound);
        noteStarted(*m);
        return wait ? awaitMechanism(*m, bound, step) : startedMechanism(*m, step);
    }
    Routine& powerStep(const mechanism::Power& d, units::Voltage volts, mechanism::Until until, units::Time bound,
                       const char* name, bool wait) {
        const char* step = name != nullptr ? name : d.name.text;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::PowerSlot* m = mechs().find(d);
        if (m == nullptr) {
            return notOnThisRobot(d.name.text, step);
        }
        m->run(volts, until, bound);
        noteStarted(*m);
        return wait ? awaitMechanism(*m, bound, step) : startedMechanism(*m, step);
    }
    Routine& speedStep(const mechanism::SpeedGoal& goal, units::Time bound, const char* name, bool wait) {
        const char* step = name != nullptr ? name : goal.mechanismName;
        if (skipIfStopped(step)) {
            return *this;
        }
        mechanism::SpeedSlot* m = mechs().find(goal);
        if (m == nullptr) {
            return notOnThisRobot(goal.mechanismName, step);
        }
        m->spinUp(goal.rpm, bound, goal.computed);
        noteStarted(*m);
        return wait ? awaitMechanism(*m, bound, step) : startedMechanism(*m, step);
    }
    // ═══ MECH-A4: the mechanism steps' machinery — END ════════════════════════════════════════

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
    // ═══ MECH-A4 (declared LAST, so the frozen constructor's initializer order is untouched) ═══
    mechanism::MechanismService* mechs_ = nullptr;  ///< the robot's service; null = a chassis-only routine
    mechanism::StepReport lastMech_{};              ///< lastMechanism()
    /// The failing mechanism step's verdict (RoutineResult::mechanismVerdict); Running = none.
    manipulation::MechanismOutcome mechStop_ = manipulation::MechanismOutcome::Running;
    /// Per slot index: the slot's stepsBegun() right after THIS routine began a step on it (0 = none).
    std::uint32_t startedSeq_[mechanism::kMaxMechanisms]{};
};

}  // namespace shulib::chassis

// MECH-A4: an AUTON's words, beside the robot file's mechanism words (mechanism/kinds.hpp), so a
// team's auton reads `RoutineResult scoreThree(AutonContext& ctx) { Routine r{ctx, "score-three"};
// ... }` with no library namespace in it — `using namespace robot_two;` brings them, because the
// robot's namespace says `using namespace mechanism::words;`. (AutonContext joins them in
// competition/auton_registry.hpp.)
namespace shulib::mechanism::words {
/// An auton's step chain: `Routine r{ctx, "name"}`.
using chassis::Routine;
/// What an auton returns: `return r.result();`.
using chassis::RoutineResult;
}  // namespace shulib::mechanism::words
