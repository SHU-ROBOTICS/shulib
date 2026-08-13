// F6 SIGNATURE PIN — the freeze, enforced structurally (chunk D2, locked 2026-08-12).
//
// Bug this file catches: an accidental reshape of the frozen public `Chassis`
// API. The Freeze Register (docs/roadmap.md, row F6) promises the surface
// changes only with a major API bump plus a migration note; a promise with no
// mechanism is a comment, so every frozen member's EXACT type is asserted
// here at compile time. If any frozen signature changes, the build fails in
// this file with a message naming F6 — instead of the change slipping through
// review as "just a refactor". (Same move C4 used to close C2's stamping gap:
// guarantees are structural, not conventional.)
//
// HOW THE PINS WORK, and why this exact shape:
//   * Each member is pinned by `static_cast<ExactType>(&C::member)` inside a
//     CONCEPT TEMPLATED ON THE CLASS. A static_cast to a member-pointer type
//     succeeds ONLY for an overload whose type matches EXACTLY — parameter
//     types, return type, const, and noexcept all participate — so any
//     reshape renders the concept false and fires the named assert.
//   * The concept indirection is LOAD-BEARING, not style (D2 proof #1's
//     lesson): in a non-template context an invalid static_cast inside a
//     requires-expression is a HARD error — the build breaks at the pin site
//     with a raw "invalid static_cast" diagnostic and the F6 message never
//     appears. Templated on the class, the same failure is a substitution
//     failure in a dependent context: the concept quietly evaluates false and
//     the static_assert fires with the message below. The freeze's bar is
//     "fails the build NAMING F6", so every member pin routes through a
//     dependent context. (Field pins use `decltype(T::field)` inside a
//     concept for the same reason: a renamed field must fire the named
//     message, not an undeclared-identifier error.)
//   * Deliberately NOT `decltype(&Chassis::member)`: that spelling breaks
//     (ambiguous address) the moment a future ADDITIVE overload appears. The
//     freeze's whole design is that additive extension stays legal (see
//     include/shulib/version.hpp); the pin must enforce "the frozen shape
//     still exists", not "no new shapes may ever exist".
//   * Default arguments are not part of a C++ function type, so the
//     one-argument call spellings (`moveTo(pose)` etc.) are pinned separately
//     by invocability — dropping a default would break every terse call site.
//   * The template verb `waitUntil` cannot be pinned as a plain member
//     pointer; it is pinned through an explicit instantiation
//     (&C::template waitUntil<PredPtr>), which checks the exact deduced
//     signature including the typed `units::Time` timeout.
//
// IF THIS FILE JUST FAILED YOUR BUILD: either the change is accidental
// (revert it) or it is an intended breaking change — then read
// include/shulib/version.hpp: bump kApiMajor, write the migration note,
// update the Freeze Register row F6, and update these pins LAST.
//
// The pin proved itself at D2 by mutation: every frozen member was reshaped
// in turn and each reshape failed the build in this file with the named F6
// message (D2 completion record §mutations). A pin that never caught
// anything is decoration.

#include "doctest.h"

#include <cstdio>
#include <initializer_list>
#include <span>
#include <string_view>
#include <type_traits>

#include "shulib/chassis/chassis.hpp"
#include "shulib/version.hpp"

namespace {

using shulib::chassis::Chassis;
using shulib::chassis::ChassisConfig;
using shulib::chassis::MotionOptions;
using shulib::chassis::TrajectoryResult;
using shulib::control::ExitReason;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::motion::CompletedMotion;
using shulib::motion::ITickPacer;
using shulib::motion::MotionConfig;
using shulib::motion::MotionDeps;
using shulib::motion::MotionScheduler;
using shulib::motion::WaitResult;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

// One uniform failure prefix so a tripped pin is unmistakable in a build log.
#define SHULIB_F6_PIN(cond, member)                                                  \
    static_assert(cond, "F6 FREEZE VIOLATION: the frozen signature of " member       \
                        " changed (locked 2026-08-12, chunk D2). Accidental? "       \
                        "Revert. Intended? That is a BREAKING change: see "          \
                        "include/shulib/version.hpp — bump kApiMajor, write the "    \
                        "migration note, update the register row, THEN this pin.")

// ── construction (borrow deps + pacer; config defaulted) ──────────────────────────
// (The is_constructible/is_copy traits are already SFINAE-safe — they evaluate
// false rather than hard-erroring — so they need no concept indirection.)
SHULIB_F6_PIN((std::is_constructible_v<Chassis, const MotionDeps&, ITickPacer&,
                                       const ChassisConfig&>),
              "Chassis(const MotionDeps&, ITickPacer&, const ChassisConfig&)");
SHULIB_F6_PIN((std::is_constructible_v<Chassis, const MotionDeps&, ITickPacer&>),
              "Chassis(deps, pacer) — the defaulted-config spelling");
SHULIB_F6_PIN(!std::is_copy_constructible_v<Chassis> && !std::is_copy_assignable_v<Chassis>,
              "Chassis non-copyability (the scheduler is pinned by its stamp)");
SHULIB_F6_PIN(!std::is_move_constructible_v<Chassis> && !std::is_move_assignable_v<Chassis>,
              "Chassis non-movability");

// ── the blocking verbs ────────────────────────────────────────────────────────────
template <typename C>
concept F6MoveTo = requires {
    static_cast<ExitReason (C::*)(const Pose2d&, const MotionOptions&)>(&C::moveTo);
};
SHULIB_F6_PIN(F6MoveTo<Chassis>,
              "Chassis::moveTo(const Pose2d&, const MotionOptions&) -> ExitReason");

template <typename C>
concept F6StrafeTo = requires {
    static_cast<ExitReason (C::*)(Length, Length, const MotionOptions&)>(&C::strafeTo);
};
SHULIB_F6_PIN(F6StrafeTo<Chassis>,
              "Chassis::strafeTo(Length, Length, const MotionOptions&) -> ExitReason");

template <typename C>
concept F6TurnTo = requires {
    static_cast<ExitReason (C::*)(Angle, const MotionOptions&)>(&C::turnTo);
};
SHULIB_F6_PIN(F6TurnTo<Chassis>,
              "Chassis::turnTo(Angle, const MotionOptions&) -> ExitReason");

template <typename C>
concept F6FollowSpan = requires {
    static_cast<TrajectoryResult (C::*)(std::span<const Pose2d>, const MotionOptions&)>(
        &C::followTrajectory);
};
SHULIB_F6_PIN(F6FollowSpan<Chassis>,
              "Chassis::followTrajectory(span<const Pose2d>, options) -> TrajectoryResult");

template <typename C>
concept F6FollowBraces = requires {
    static_cast<TrajectoryResult (C::*)(std::initializer_list<Pose2d>, const MotionOptions&)>(
        &C::followTrajectory);
};
SHULIB_F6_PIN(F6FollowBraces<Chassis>,
              "Chassis::followTrajectory({...}, options) -> TrajectoryResult (brace form)");

template <typename C>
concept F6Brake = requires {
    static_cast<ExitReason (C::*)(const MotionOptions&)>(&C::brake);
};
SHULIB_F6_PIN(F6Brake<Chassis>, "Chassis::brake(const MotionOptions&) -> ExitReason");

template <typename C>
concept F6Hold = requires {
    static_cast<ExitReason (C::*)(Time, const MotionOptions&)>(&C::hold);
};
SHULIB_F6_PIN(F6Hold<Chassis>,
              "Chassis::hold(units::Time, const MotionOptions&) -> ExitReason (TYPED time, D2)");

template <typename C>
concept F6Wait = requires { static_cast<void (C::*)(Time)>(&C::wait); };
SHULIB_F6_PIN(F6Wait<Chassis>, "Chassis::wait(units::Time) -> void (adopted at D2)");

// ── the manual verb + control ─────────────────────────────────────────────────────
template <typename C>
concept F6Drive = requires {
    static_cast<void (C::*)(const ChassisSpeeds&, Frame)>(&C::drive);
};
SHULIB_F6_PIN(F6Drive<Chassis>,
              "Chassis::drive(const ChassisSpeeds&, Frame) — Frame REQUIRED, no default");

template <typename C>
concept F6Cancel = requires { static_cast<void (C::*)()>(&C::cancel); };
SHULIB_F6_PIN(F6Cancel<Chassis>, "Chassis::cancel() — the panic stop");

// A NEGATIVE pin: drive(speeds) without a Frame must NOT compile. Adding a
// default Frame would not move the member-pointer pin above (the type is
// unchanged — additive spellings are normally legal), but "the caller always
// names the frame" is a FROZEN SEMANTIC: silent frame assumption is the bug
// class this rebuild exists to prevent, so its impossibility is part of F6.
template <typename C>
concept F6DriveFrameless = requires(C& c, const ChassisSpeeds& s) { c.drive(s); };
SHULIB_F6_PIN(!F6DriveFrameless<Chassis>,
              "drive(speeds) staying UNCOMPILABLE — Frame is required, no default "
              "(frozen semantic, not just a frozen signature)");

// The template verb, pinned through an exact instantiation (header note above).
using PredPtr = bool (*)();
template <typename C>
concept F6WaitUntil = requires {
    static_cast<WaitResult (C::*)(PredPtr&&, Time)>(&C::template waitUntil<PredPtr>);
};
SHULIB_F6_PIN(F6WaitUntil<Chassis>,
              "Chassis::waitUntil(Pred&&, units::Time) -> WaitResult (TYPED time, D2)");

// ── state / observability ─────────────────────────────────────────────────────────
template <typename C>
concept F6Pose = requires { static_cast<Pose2d (C::*)() const>(&C::pose); };
SHULIB_F6_PIN(F6Pose<Chassis>, "Chassis::pose() const -> Pose2d");

template <typename C>
concept F6SetPose = requires { static_cast<void (C::*)(const Pose2d&)>(&C::setPose); };
SHULIB_F6_PIN(F6SetPose<Chassis>, "Chassis::setPose(const Pose2d&)");

template <typename C>
concept F6StrafeAuthority = requires {
    static_cast<double (C::*)() const>(&C::strafeAuthority);
};
SHULIB_F6_PIN(F6StrafeAuthority<Chassis>, "Chassis::strafeAuthority() const -> double");

// The noexcept-carrying pins pair the exact static_cast with a compound
// requirement `{ call } noexcept` — CAMPAIGN FIND (D2 mutations A16/A31, the
// chunk's green hole): for a NON-overloaded member, the static_cast that ADDS
// noexcept is accepted, so the cast alone does not notice noexcept being
// DROPPED (an overloaded member like scheduler() is caught anyway, because
// target-type matching over an overload set refuses a potentially-throwing
// candidate). The compound requirement observes the actual call's
// noexcept-ness and cannot be fooled; version.hpp lists a noexcept change as
// BREAKING, so the pin must see it.
template <typename C>
concept F6LastExitReason = requires(const C& c) {
    static_cast<ExitReason (C::*)() const noexcept>(&C::lastExitReason);
    { c.lastExitReason() } noexcept -> std::same_as<ExitReason>;
};
SHULIB_F6_PIN(F6LastExitReason<Chassis>,
              "Chassis::lastExitReason() const noexcept -> ExitReason");

template <typename C>
concept F6LastCompleted = requires(const C& c) {
    static_cast<const CompletedMotion& (C::*)() const noexcept>(&C::lastCompleted);
    { c.lastCompleted() } noexcept -> std::same_as<const CompletedMotion&>;
};
SHULIB_F6_PIN(F6LastCompleted<Chassis>,
              "Chassis::lastCompleted() const noexcept -> const CompletedMotion&");

template <typename C>
concept F6MotionConfig = requires(const C& c) {
    static_cast<const MotionConfig& (C::*)() const noexcept>(&C::motionConfig);
    { c.motionConfig() } noexcept -> std::same_as<const MotionConfig&>;
};
SHULIB_F6_PIN(F6MotionConfig<Chassis>,
              "Chassis::motionConfig() const noexcept -> const MotionConfig&");

// ── the Tier-3 seam (the no-ceiling guarantee is itself frozen) ───────────────────
template <typename C>
concept F6Deps = requires(const C& c) {
    static_cast<const MotionDeps& (C::*)() const noexcept>(&C::deps);
    { c.deps() } noexcept -> std::same_as<const MotionDeps&>;
};
SHULIB_F6_PIN(F6Deps<Chassis>,
              "Chassis::deps() const noexcept -> const MotionDeps& (the STAMPED bundle)");

template <typename C>
concept F6Scheduler = requires(C& c) {
    static_cast<MotionScheduler& (C::*)() noexcept>(&C::scheduler);
    { c.scheduler() } noexcept -> std::same_as<MotionScheduler&>;
};
SHULIB_F6_PIN(F6Scheduler<Chassis>, "Chassis::scheduler() noexcept -> MotionScheduler&");

template <typename C>
concept F6SchedulerConst = requires(const C& c) {
    static_cast<const MotionScheduler& (C::*)() const noexcept>(&C::scheduler);
    { c.scheduler() } noexcept -> std::same_as<const MotionScheduler&>;
};
SHULIB_F6_PIN(F6SchedulerConst<Chassis>,
              "Chassis::scheduler() const noexcept -> const MotionScheduler& (const overload)");

// ── the three frozen public types (a frozen signature over an unfrozen type
//    freezes nothing — existing fields are pinned; the SET stays additive-open) ────
template <typename T>
concept F6OptionsTimeout = requires { requires std::is_same_v<decltype(T::timeout), Time>; };
SHULIB_F6_PIN(F6OptionsTimeout<MotionOptions>,
              "MotionOptions::timeout : units::Time (TYPED time, D2 — was timeoutSeconds)");

template <typename T>
concept F6OptionsMaxLinear = requires {
    requires std::is_same_v<decltype(T::maxLinearSpeed), Velocity>;
};
SHULIB_F6_PIN(F6OptionsMaxLinear<MotionOptions>,
              "MotionOptions::maxLinearSpeed : units::Velocity");

template <typename T>
concept F6OptionsMaxAngular = requires {
    requires std::is_same_v<decltype(T::maxAngularSpeed), AngularVelocity>;
};
SHULIB_F6_PIN(F6OptionsMaxAngular<MotionOptions>,
              "MotionOptions::maxAngularSpeed : units::AngularVelocity");

template <typename T>
concept F6OptionsValidate = requires {
    static_cast<void (T::*)() const>(&T::validate);
};
SHULIB_F6_PIN(F6OptionsValidate<MotionOptions>, "MotionOptions::validate() const");

template <typename T>
concept F6ConfigMotion = requires {
    requires std::is_same_v<decltype(T::motion), MotionConfig>;
};
SHULIB_F6_PIN(F6ConfigMotion<ChassisConfig>,
              "ChassisConfig::motion : MotionConfig (passed through WHOLE — the additive path)");

template <typename T>
concept F6ConfigScheduler = requires {
    requires std::is_same_v<decltype(T::scheduler), shulib::motion::MotionSchedulerConfig>;
};
SHULIB_F6_PIN(F6ConfigScheduler<ChassisConfig>,
              "ChassisConfig::scheduler : MotionSchedulerConfig (passed through WHOLE)");

template <typename T>
concept F6TrajExit = requires { requires std::is_same_v<decltype(T::exit), ExitReason>; };
SHULIB_F6_PIN(F6TrajExit<TrajectoryResult>, "TrajectoryResult::exit : ExitReason");

template <typename T>
concept F6TrajCompleted = requires {
    requires std::is_same_v<decltype(T::completedLegs), int>;
};
SHULIB_F6_PIN(F6TrajCompleted<TrajectoryResult>, "TrajectoryResult::completedLegs : int");

template <typename T>
concept F6TrajTotal = requires { requires std::is_same_v<decltype(T::totalLegs), int>; };
SHULIB_F6_PIN(F6TrajTotal<TrajectoryResult>, "TrajectoryResult::totalLegs : int");

template <typename T>
concept F6TrajSucceeded = requires(const T& t) {
    static_cast<bool (T::*)() const noexcept>(&T::succeeded);
    { t.succeeded() } noexcept -> std::same_as<bool>;  // campaign find A31 (header note)
};
SHULIB_F6_PIN(F6TrajSucceeded<TrajectoryResult>,
              "TrajectoryResult::succeeded() const noexcept -> bool");

// ── the defaulted call spellings (default args are not part of a function type;
//    dropping one would break every terse call site without moving the pins above) ─
template <typename C>
concept TerseSpellingsStillCompile = requires(C& c, const Pose2d& p, Length l, Angle a, Time t) {
    { c.moveTo(p) } -> std::same_as<ExitReason>;
    { c.strafeTo(l, l) } -> std::same_as<ExitReason>;
    { c.turnTo(a) } -> std::same_as<ExitReason>;
    { c.followTrajectory({p, p}) } -> std::same_as<TrajectoryResult>;
    { c.brake() } -> std::same_as<ExitReason>;
    { c.hold(t) } -> std::same_as<ExitReason>;
};
SHULIB_F6_PIN(TerseSpellingsStillCompile<Chassis>,
              "the defaulted MotionOptions spellings (moveTo(pose) etc.)");

#undef SHULIB_F6_PIN

// The pins above are the test; this case exists so the file registers in the
// runner and pins the version constants the freeze policy hangs off.
// Bug caught: the F6 freeze flips to LOCKED while the version mechanism the
// register promises ("changes only with an API-version bump") doesn't exist
// or no longer says major version 2.
// HISTORY (fixed at F1): this pin originally asserted `kApiMinor == 0`, which
// conflated "F6 is API 2.x and the mechanism exists" (what the freeze means)
// with "the version is exactly 2.0" (what was written) — and thereby turned
// version.hpp's OWN documented additive path ("bump kApiMinor ... the intended
// growth path of every frozen surface") into a red pin. The first legal
// additive change (F1: appended enumerators + then() growth, 2.0 → 2.1) hit
// it. A minor bump is additive by definition and must never fail this pin.
// SECOND FIX (reviewer, at F1 verification): the F1 repair above swapped a
// too-strict check for a too-weak one. `kApiMinor >= 0` is VACUOUS — an int is
// always >= 0, so that line could never fail, and a check that cannot fail
// reads as coverage without being it. And `kApiVersionString[0] == '2'`
// inspects ONE character, so the printable token could drift from the numbers
// it claims to print. Measured, not argued: with kApiMinor = 1 and the string
// left at "2.0", all three former assertions PASSED.
//
// Bug now caught: kApiMinor is bumped for an additive change and
// kApiVersionString is not updated with it (or vice versa). That token is the
// one printable version — main.cpp carries a TODO(R1) to emit it in the §18.5
// session header — so a desync would label every on-robot transcript and every
// blackbox file with an API version the build is not. Provenance that is
// silently wrong is worse than absent, because it looks authoritative.
//
// kApiMinor is deliberately NOT pinned to any value: additive growth is legal
// and expected (see the HISTORY note above). Its ABSENCE is stated here rather
// than implied by a check that cannot fail.
TEST_CASE("F6 pin: the frozen surface is API major 2 and the version mechanism exists") {
    CHECK(shulib::kApiMajor == 2);

    char expected[16];
    std::snprintf(expected, sizeof expected, "%d.%d", shulib::kApiMajor, shulib::kApiMinor);
    CHECK(std::string_view{shulib::kApiVersionString} == std::string_view{expected});
}

}  // namespace
