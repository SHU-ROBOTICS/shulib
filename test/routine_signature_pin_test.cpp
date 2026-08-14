// F10 SIGNATURE PIN — the Routine freeze, enforced structurally (chunk D3).
//
// Bug this file catches: an accidental reshape of the frozen public `Routine`
// API. The Freeze Register (docs/roadmap.md, row F10) promises the recipe
// surface changes only with a major API bump plus a migration note; a promise
// with no mechanism is a comment, so every frozen member's EXACT type is
// asserted here at compile time. If any frozen signature changes, the build
// fails in this file with a message naming F10.
//
// This is D2's F6 pin, applied one tier up, and it inherits both of that
// chunk's hard-won lessons:
//
//   * Every member pin routes through a CONCEPT TEMPLATED ON THE CLASS. In a
//     non-dependent context an invalid static_cast inside a requires-expression
//     is a HARD error and the named static_assert never gets to speak; in a
//     dependent context the same failure is a substitution failure, the concept
//     evaluates false, and the message below is what the reader sees. The bar
//     is "fails the build NAMING the freeze", and "fails the build" is not that.
//   * Every noexcept-carrying pin pairs the static_cast with a COMPOUND
//     REQUIREMENT `{ call } noexcept`. D2's campaign hole #1: for a
//     NON-overloaded member the compiler accepts a static_cast that ADDS
//     noexcept, so the cast alone cannot see noexcept being DROPPED. Since
//     version.hpp lists a noexcept change as BREAKING, an exact-cast-only pin
//     would have been decoration on exactly the observability surface a routine
//     author reads. Do not re-open that hole.
//
// ═══ WHAT IS DELIBERATELY NOT PINNED HERE: `then()` ══════════════════════════════
// `Routine::then` is EXCLUDED from F10 and its absence below is deliberate, not
// an oversight. It is the mechanism seam, and mechanisms do not exist yet
// (F1/F3 build them): its accepted return types (void / bool / ExitReason) and
// its `name` parameter's default are a placeholder shape chosen before there
// was anything real to plug in. Freezing it would commit the project to a
// guess. The register row F10 states the exclusion in the same words, because
// silence in a freeze reads as "frozen too" (D2 ruling A2's lesson).
//
// IF THIS FILE JUST FAILED YOUR BUILD: either the change is accidental
// (revert it) or it is an intended breaking change — then read
// include/shulib/version.hpp: bump kApiMajor, write the migration note,
// update the Freeze Register row F10, and update these pins LAST.

#include "doctest.h"

#include <cstdio>
#include <initializer_list>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

#include "shulib/chassis/routine.hpp"
#include "shulib/version.hpp"

namespace {

using shulib::chassis::Chassis;
using shulib::chassis::MotionOptions;
using shulib::chassis::Routine;
using shulib::chassis::RoutineResult;
using shulib::chassis::RoutineStopCause;
using shulib::chassis::TrajectoryResult;
using shulib::control::ExitReason;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

// One uniform failure prefix so a tripped pin is unmistakable in a build log.
#define SHULIB_F10_PIN(cond, member)                                                 \
    static_assert(cond, "F10 FREEZE VIOLATION: the frozen signature of " member      \
                        " changed (Routine, locked 2026-08-12, chunk D3). "          \
                        "Accidental? Revert. Intended? That is a BREAKING change: "  \
                        "see include/shulib/version.hpp — bump kApiMajor, write "    \
                        "the migration note, update Freeze Register row F10, THEN "  \
                        "this pin.")

// ── construction (borrows the Chassis; name defaulted; noexcept) ──────────────────
SHULIB_F10_PIN((std::is_constructible_v<Routine, Chassis&, const char*>),
               "Routine(Chassis&, const char*)");
SHULIB_F10_PIN((std::is_constructible_v<Routine, Chassis&>),
               "Routine(chassis) — the defaulted-name spelling");
SHULIB_F10_PIN((std::is_nothrow_constructible_v<Routine, Chassis&, const char*>),
               "Routine's constructor staying noexcept");
SHULIB_F10_PIN(!std::is_copy_constructible_v<Routine> && !std::is_copy_assignable_v<Routine>,
               "Routine non-copyability (two handles would fork the stop state)");
SHULIB_F10_PIN(!std::is_move_constructible_v<Routine> && !std::is_move_assignable_v<Routine>,
               "Routine non-movability");

// ── the steps: every one returns Routine& so the chain composes ───────────────────
template <typename R>
concept F10StartAt = requires { static_cast<R& (R::*)(const Pose2d&)>(&R::startAt); };
SHULIB_F10_PIN(F10StartAt<Routine>, "Routine::startAt(const Pose2d&) -> Routine&");

template <typename R>
concept F10MoveTo = requires {
    static_cast<R& (R::*)(const Pose2d&, const MotionOptions&)>(&R::moveTo);
};
SHULIB_F10_PIN(F10MoveTo<Routine>,
               "Routine::moveTo(const Pose2d&, const MotionOptions&) -> Routine&");

template <typename R>
concept F10DriveTo = requires {
    static_cast<R& (R::*)(Length, Length, const MotionOptions&)>(&R::driveTo);
};
SHULIB_F10_PIN(F10DriveTo<Routine>,
               "Routine::driveTo(Length, Length, const MotionOptions&) -> Routine&");

template <typename R>
concept F10StrafeTo = requires {
    static_cast<R& (R::*)(Length, Length, const MotionOptions&)>(&R::strafeTo);
};
SHULIB_F10_PIN(F10StrafeTo<Routine>,
               "Routine::strafeTo(Length, Length, const MotionOptions&) -> Routine&");

template <typename R>
concept F10TurnTo = requires {
    static_cast<R& (R::*)(Angle, const MotionOptions&)>(&R::turnTo);
};
SHULIB_F10_PIN(F10TurnTo<Routine>,
               "Routine::turnTo(Angle, const MotionOptions&) -> Routine&");

template <typename R>
concept F10Face = requires {
    static_cast<R& (R::*)(Length, Length, const MotionOptions&)>(&R::face);
};
SHULIB_F10_PIN(F10Face<Routine>,
               "Routine::face(Length, Length, const MotionOptions&) -> Routine&");

template <typename R>
concept F10FollowSpan = requires {
    static_cast<R& (R::*)(std::span<const Pose2d>, const MotionOptions&)>(
        &R::followTrajectory);
};
SHULIB_F10_PIN(F10FollowSpan<Routine>,
               "Routine::followTrajectory(span<const Pose2d>, options) -> Routine&");

template <typename R>
concept F10FollowBraces = requires {
    static_cast<R& (R::*)(std::initializer_list<Pose2d>, const MotionOptions&)>(
        &R::followTrajectory);
};
SHULIB_F10_PIN(F10FollowBraces<Routine>,
               "Routine::followTrajectory({...}, options) -> Routine& (brace form)");

template <typename R>
concept F10Brake = requires { static_cast<R& (R::*)(const MotionOptions&)>(&R::brake); };
SHULIB_F10_PIN(F10Brake<Routine>, "Routine::brake(const MotionOptions&) -> Routine&");

template <typename R>
concept F10Hold = requires {
    static_cast<R& (R::*)(Time, const MotionOptions&)>(&R::hold);
};
SHULIB_F10_PIN(F10Hold<Routine>,
               "Routine::hold(units::Time, const MotionOptions&) -> Routine& (TYPED time)");

template <typename R>
concept F10Pause = requires { static_cast<R& (R::*)(Time)>(&R::pause); };
SHULIB_F10_PIN(F10Pause<Routine>, "Routine::pause(units::Time) -> Routine&");

// The template step, pinned through an exact instantiation — the deduced
// signature including the typed timeout and the defaulted step name.
using PredPtr = bool (*)();
template <typename R>
concept F10WaitFor = requires {
    static_cast<R& (R::*)(PredPtr&&, Time, const char*)>(&R::template waitFor<PredPtr>);
};
SHULIB_F10_PIN(F10WaitFor<Routine>,
               "Routine::waitFor(Pred&&, units::Time, const char*) -> Routine&");

// ── the OTHER half of the noexcept shape (added by the reviewer at F1 verification) ─
// D2 closed hole #1 — a static_cast that ADDS noexcept is accepted, so the cast
// alone cannot see noexcept being DROPPED — with compound `{ call } noexcept`
// requirements on the four observers below. The INVERSE was left open: the eleven
// STEPS are not noexcept, nothing asserted they stay that way, and the same cast
// permissiveness makes an ADDED noexcept invisible to their pins.
//
// version.hpp calls a change to a frozen signature's "noexcept shape" BREAKING —
// in EITHER direction. It changes the member's type (a user taking &Routine::pause
// into a typed pointer breaks), and it converts a precondition throw into
// std::terminate.
//
// MEASURED before this was written, by adding noexcept to Routine::pause: the F10
// pin never fired. The build did fail — but on the api-doc FRESHNESS gate, naming
// the wrong problem; and after regenerating the reference exactly as that gate
// instructs, the only remaining signals were a doc-fidelity test with a hardcoded
// signature and a SIGABRT (pause calls wait(), which throws on nonsense input, and
// a throw from a noexcept function terminates). A reader would have debugged a
// crash in a test about precondition handling and never seen the words F10 FREEZE
// VIOLATION. For a frozen member that is neither in the generated reference nor
// calls anything throwing, the change would be entirely SILENT.
#define SHULIB_F10_NOT_NOEXCEPT(expr, member)                                        \
    static_assert(!noexcept(expr),                                                   \
                  "F10 FREEZE VIOLATION: " member " GAINED noexcept. A change to a " \
                  "frozen signature's noexcept shape is BREAKING IN EITHER "         \
                  "DIRECTION (include/shulib/version.hpp): it changes the member's " \
                  "type, and it turns a precondition throw into std::terminate. "    \
                  "Accidental? Revert. Intended? Bump kApiMajor, write the "         \
                  "migration note, update Freeze Register row F10, THEN this pin.")

SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().startAt(std::declval<const Pose2d&>()),
                        "Routine::startAt");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().moveTo(std::declval<const Pose2d&>(),
                                                        std::declval<const MotionOptions&>()),
                        "Routine::moveTo");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().driveTo(std::declval<Length>(),
                                                         std::declval<Length>(),
                                                         std::declval<const MotionOptions&>()),
                        "Routine::driveTo");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().strafeTo(std::declval<Length>(),
                                                          std::declval<Length>(),
                                                          std::declval<const MotionOptions&>()),
                        "Routine::strafeTo");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().turnTo(std::declval<Angle>(),
                                                        std::declval<const MotionOptions&>()),
                        "Routine::turnTo");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().face(std::declval<Length>(),
                                                      std::declval<Length>(),
                                                      std::declval<const MotionOptions&>()),
                        "Routine::face");
SHULIB_F10_NOT_NOEXCEPT(
    std::declval<Routine&>().followTrajectory(std::declval<std::span<const Pose2d>>(),
                                              std::declval<const MotionOptions&>()),
    "Routine::followTrajectory(span)");
SHULIB_F10_NOT_NOEXCEPT(
    std::declval<Routine&>().followTrajectory(std::declval<std::initializer_list<Pose2d>>(),
                                              std::declval<const MotionOptions&>()),
    "Routine::followTrajectory(braces)");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().brake(std::declval<const MotionOptions&>()),
                        "Routine::brake");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().hold(std::declval<Time>(),
                                                      std::declval<const MotionOptions&>()),
                        "Routine::hold");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().pause(std::declval<Time>()),
                        "Routine::pause");
SHULIB_F10_NOT_NOEXCEPT(std::declval<Routine&>().waitFor(std::declval<PredPtr>(),
                                                         std::declval<Time>(),
                                                         std::declval<const char*>()),
                        "Routine::waitFor");

// ── the whole-chain verdict (every one noexcept — see the header note) ────────────
template <typename R>
concept F10Ok = requires(const R& r) {
    static_cast<bool (R::*)() const noexcept>(&R::ok);
    { r.ok() } noexcept -> std::same_as<bool>;  // D2 hole #1: the cast alone is blind
};
SHULIB_F10_PIN(F10Ok<Routine>, "Routine::ok() const noexcept -> bool");

template <typename R>
concept F10Result = requires(const R& r) {
    static_cast<RoutineResult (R::*)() const noexcept>(&R::result);
    { r.result() } noexcept -> std::same_as<RoutineResult>;
};
SHULIB_F10_PIN(F10Result<Routine>, "Routine::result() const noexcept -> RoutineResult");

template <typename R>
concept F10LastTrajectory = requires(const R& r) {
    static_cast<const TrajectoryResult& (R::*)() const noexcept>(&R::lastTrajectory);
    { r.lastTrajectory() } noexcept -> std::same_as<const TrajectoryResult&>;
};
SHULIB_F10_PIN(F10LastTrajectory<Routine>,
               "Routine::lastTrajectory() const noexcept -> const TrajectoryResult&");

template <typename R>
concept F10ChassisSeam = requires(R& r) {
    static_cast<Chassis& (R::*)() noexcept>(&R::chassis);
    { r.chassis() } noexcept -> std::same_as<Chassis&>;
};
SHULIB_F10_PIN(F10ChassisSeam<Routine>,
               "Routine::chassis() noexcept -> Chassis& (the mixed-tier seam)");

// ── RoutineResult: a frozen return type whose fields are the frozen facts ─────────
template <typename T>
concept F10ResOk = requires { requires std::is_same_v<decltype(T::ok), bool>; };
SHULIB_F10_PIN(F10ResOk<RoutineResult>, "RoutineResult::ok : bool");

template <typename T>
concept F10ResSteps = requires { requires std::is_same_v<decltype(T::steps), int>; };
SHULIB_F10_PIN(F10ResSteps<RoutineResult>, "RoutineResult::steps : int");

template <typename T>
concept F10ResCompleted = requires { requires std::is_same_v<decltype(T::completed), int>; };
SHULIB_F10_PIN(F10ResCompleted<RoutineResult>, "RoutineResult::completed : int");

template <typename T>
concept F10ResSkipped = requires { requires std::is_same_v<decltype(T::skipped), int>; };
SHULIB_F10_PIN(F10ResSkipped<RoutineResult>, "RoutineResult::skipped : int");

template <typename T>
concept F10ResStoppedAt = requires { requires std::is_same_v<decltype(T::stoppedAt), int>; };
SHULIB_F10_PIN(F10ResStoppedAt<RoutineResult>, "RoutineResult::stoppedAt : int");

template <typename T>
concept F10ResStoppedName = requires {
    requires std::is_same_v<decltype(T::stoppedName), const char*>;
};
SHULIB_F10_PIN(F10ResStoppedName<RoutineResult>, "RoutineResult::stoppedName : const char*");

template <typename T>
concept F10ResCause = requires {
    requires std::is_same_v<decltype(T::cause), RoutineStopCause>;
};
SHULIB_F10_PIN(F10ResCause<RoutineResult>, "RoutineResult::cause : RoutineStopCause");

template <typename T>
concept F10ResExit = requires { requires std::is_same_v<decltype(T::exit), ExitReason>; };
SHULIB_F10_PIN(F10ResExit<RoutineResult>, "RoutineResult::exit : control::ExitReason");

// ── RoutineStopCause: APPEND-ONLY, so the existing values are frozen ──────────────
// Renumbering or re-meaning an enumerator is BREAKING (version.hpp), and a
// re-meaning is invisible at every call site — which is exactly why the
// numeric values are pinned rather than merely the names.
SHULIB_F10_PIN(static_cast<int>(RoutineStopCause::None) == 0,
               "RoutineStopCause::None == 0 (append-only enum)");
SHULIB_F10_PIN(static_cast<int>(RoutineStopCause::MotionFailed) == 1,
               "RoutineStopCause::MotionFailed == 1 (append-only enum)");
SHULIB_F10_PIN(static_cast<int>(RoutineStopCause::WaitTimedOut) == 2,
               "RoutineStopCause::WaitTimedOut == 2 (append-only enum)");
SHULIB_F10_PIN(static_cast<int>(RoutineStopCause::ActionFailed) == 3,
               "RoutineStopCause::ActionFailed == 3 (append-only enum)");
SHULIB_F10_PIN(static_cast<int>(RoutineStopCause::MechanismFailed) == 4,
               "RoutineStopCause::MechanismFailed == 4 (append-only enum; APPENDED at F1 "
               "via the sanctioned additive path — a re-meaning of it is what this pin "
               "would catch)");

// ── the defaulted call spellings (default args are not part of a function type) ───
template <typename R>
concept F10TerseSpellings = requires(R& r, const Pose2d& p, Length l, Angle a, Time t,
                                     PredPtr pred) {
    { r.startAt(p) } -> std::same_as<R&>;
    { r.moveTo(p) } -> std::same_as<R&>;
    { r.driveTo(l, l) } -> std::same_as<R&>;
    { r.strafeTo(l, l) } -> std::same_as<R&>;
    { r.turnTo(a) } -> std::same_as<R&>;
    { r.face(l, l) } -> std::same_as<R&>;
    { r.followTrajectory({p, p}) } -> std::same_as<R&>;
    { r.brake() } -> std::same_as<R&>;
    { r.hold(t) } -> std::same_as<R&>;
    { r.pause(t) } -> std::same_as<R&>;
    { r.waitFor(pred, t) } -> std::same_as<R&>;  // the defaulted step name
};
SHULIB_F10_PIN(F10TerseSpellings<Routine>,
               "the defaulted spellings (moveTo(pose), waitFor(pred, t), ...)");

// ── NEGATIVE pins: typed time is a FROZEN SEMANTIC, not just a signature ──────────
// A bare double where a duration belongs must stay uncompilable. Adding an
// implicit conversion would break no member-pointer pin above (the types would
// not move) yet would reopen the exact bug D2's retype exists to kill:
// `hold(300)` from someone thinking in milliseconds, holding pose for 300 s of
// a 15 s match.
template <typename R>
concept F10HoldBareDouble = requires(R& r) { r.hold(0.3); };
SHULIB_F10_PIN(!F10HoldBareDouble<Routine>,
               "Routine::hold(0.3) staying UNCOMPILABLE — durations are typed");

template <typename R>
concept F10PauseBareDouble = requires(R& r) { r.pause(0.2); };
SHULIB_F10_PIN(!F10PauseBareDouble<Routine>,
               "Routine::pause(0.2) staying UNCOMPILABLE — durations are typed");

// A step must never be reachable on a `const Routine&`: every step mutates the
// chain's counters, and a const-callable step would be a silent lie about it.
template <typename R>
concept F10ConstStep = requires(const R& r, const Pose2d& p) { r.moveTo(p); };
SHULIB_F10_PIN(!F10ConstStep<Routine>,
               "steps staying non-const (a const chain cannot secretly advance)");

#undef SHULIB_F10_PIN

// The pins above are the test; this case exists so the file registers in the
// runner and records what the freeze is hung off.
// Bug caught: F10 flips to LOCKED while the version mechanism the register
// promises does not exist or no longer says major version 2.
// HISTORY (fixed at F1): originally asserted `kApiMinor == 0` — the same
// conflation as the F6 twin (see f6_signature_pin_test.cpp): it made the
// documented ADDITIVE path (appending a RoutineStopCause enumerator, which
// routine.hpp:145 explicitly anticipates for F1/F3) fail the pin that exists
// to guard the FROZEN surface. Additive growth bumps kApiMinor and must never
// fail this pin; a major bump still does, loudly, as intended.
// SECOND FIX (reviewer, at F1 verification) — the same repair as the F6 twin,
// and for the same reason: F1's fix traded a too-strict check for a too-weak
// one. `kApiMinor >= 0` cannot fail (int), and a one-character check on
// kApiVersionString cannot see the token drifting from the numbers. Measured:
// kApiMinor = 1 with the string left at "2.0" passed all three former lines.
//
// Bug now caught: the printable version and the numeric version disagree.
// That token is what a session header prints (main.cpp TODO(R1)), so a desync
// mislabels every on-robot transcript and blackbox file.
//
// kApiMinor is deliberately unpinned — additive growth is legal. Saying so is
// the point; a vacuous assertion says it in a way that looks like a test.
TEST_CASE("F10 pin: the frozen Routine surface is API major 2 and the mechanism exists") {
    CHECK(shulib::kApiMajor == 2);

    char expected[16];
    std::snprintf(expected, sizeof expected, "%d.%d", shulib::kApiMajor, shulib::kApiMinor);
    CHECK(std::string_view{shulib::kApiVersionString} == std::string_view{expected});
}

}  // namespace
