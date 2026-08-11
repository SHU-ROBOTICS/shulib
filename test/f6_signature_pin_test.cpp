// F6 SIGNATURE PIN — the freeze, enforced structurally (chunk D2, 2026-08-11).
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
//   * Each member is pinned by `static_cast<ExactType>(&Chassis::member)`
//     inside a requires-expression. A static_cast to a member-pointer type
//     succeeds ONLY for an overload whose type matches EXACTLY — parameter
//     types, return type, const, and noexcept all participate — so any
//     reshape renders the requires-expression false and fires the assert.
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
//     (&Chassis::waitUntil<PredPtr>), which checks the exact deduced
//     signature including the typed `units::Time` timeout.
//
// IF THIS FILE JUST FAILED YOUR BUILD: either the change is accidental
// (revert it) or it is an intended breaking change — then read
// include/shulib/version.hpp: bump kApiMajor, write the migration note,
// update the Freeze Register row F6, and update these pins LAST.
//
// The pin proved itself at D2 by mutation: every frozen member was reshaped
// in turn and each reshape failed the build in this file (D2 completion
// record §mutations). A pin that never caught anything is decoration.

#include "doctest.h"

#include <initializer_list>
#include <span>
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
                        " changed (locked 2026-08-11, chunk D2). Accidental? "       \
                        "Revert. Intended? That is a BREAKING change: see "          \
                        "include/shulib/version.hpp — bump kApiMajor, write the "    \
                        "migration note, update the register row, THEN this pin.")

// ── construction (borrow deps + pacer; config defaulted) ──────────────────────────
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
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)(const Pose2d&, const MotionOptions&)>(
                      &Chassis::moveTo);
              }),
              "Chassis::moveTo(const Pose2d&, const MotionOptions&) -> ExitReason");
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)(Length, Length, const MotionOptions&)>(
                      &Chassis::strafeTo);
              }),
              "Chassis::strafeTo(Length, Length, const MotionOptions&) -> ExitReason");
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)(Angle, const MotionOptions&)>(
                      &Chassis::turnTo);
              }),
              "Chassis::turnTo(Angle, const MotionOptions&) -> ExitReason");
SHULIB_F6_PIN((requires {
                  static_cast<TrajectoryResult (Chassis::*)(std::span<const Pose2d>,
                                                            const MotionOptions&)>(
                      &Chassis::followTrajectory);
              }),
              "Chassis::followTrajectory(span<const Pose2d>, options) -> TrajectoryResult");
SHULIB_F6_PIN((requires {
                  static_cast<TrajectoryResult (Chassis::*)(std::initializer_list<Pose2d>,
                                                            const MotionOptions&)>(
                      &Chassis::followTrajectory);
              }),
              "Chassis::followTrajectory({...}, options) -> TrajectoryResult (brace form)");
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)(const MotionOptions&)>(&Chassis::brake);
              }),
              "Chassis::brake(const MotionOptions&) -> ExitReason");
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)(Time, const MotionOptions&)>(
                      &Chassis::hold);
              }),
              "Chassis::hold(units::Time, const MotionOptions&) -> ExitReason (TYPED time, D2)");
SHULIB_F6_PIN((requires { static_cast<void (Chassis::*)(Time)>(&Chassis::wait); }),
              "Chassis::wait(units::Time) -> void (adopted at D2)");

// ── the manual verb + control ─────────────────────────────────────────────────────
SHULIB_F6_PIN((requires {
                  static_cast<void (Chassis::*)(const ChassisSpeeds&, Frame)>(&Chassis::drive);
              }),
              "Chassis::drive(const ChassisSpeeds&, Frame) — Frame REQUIRED, no default");
SHULIB_F6_PIN((requires { static_cast<void (Chassis::*)()>(&Chassis::cancel); }),
              "Chassis::cancel() — the panic stop");
// The template verb, pinned through an exact instantiation (header note above).
using PredPtr = bool (*)();
SHULIB_F6_PIN((requires {
                  static_cast<WaitResult (Chassis::*)(PredPtr&&, Time)>(
                      &Chassis::waitUntil<PredPtr>);
              }),
              "Chassis::waitUntil(Pred&&, units::Time) -> WaitResult (TYPED time, D2)");

// ── state / observability ─────────────────────────────────────────────────────────
SHULIB_F6_PIN((requires { static_cast<Pose2d (Chassis::*)() const>(&Chassis::pose); }),
              "Chassis::pose() const -> Pose2d");
SHULIB_F6_PIN((requires { static_cast<void (Chassis::*)(const Pose2d&)>(&Chassis::setPose); }),
              "Chassis::setPose(const Pose2d&)");
SHULIB_F6_PIN((requires {
                  static_cast<double (Chassis::*)() const>(&Chassis::strafeAuthority);
              }),
              "Chassis::strafeAuthority() const -> double");
SHULIB_F6_PIN((requires {
                  static_cast<ExitReason (Chassis::*)() const noexcept>(&Chassis::lastExitReason);
              }),
              "Chassis::lastExitReason() const noexcept -> ExitReason");
SHULIB_F6_PIN((requires {
                  static_cast<const CompletedMotion& (Chassis::*)() const noexcept>(
                      &Chassis::lastCompleted);
              }),
              "Chassis::lastCompleted() const noexcept -> const CompletedMotion&");
SHULIB_F6_PIN((requires {
                  static_cast<const MotionConfig& (Chassis::*)() const noexcept>(
                      &Chassis::motionConfig);
              }),
              "Chassis::motionConfig() const noexcept -> const MotionConfig&");

// ── the Tier-3 seam (the no-ceiling guarantee is itself frozen) ───────────────────
SHULIB_F6_PIN((requires {
                  static_cast<const MotionDeps& (Chassis::*)() const noexcept>(&Chassis::deps);
              }),
              "Chassis::deps() const noexcept -> const MotionDeps& (the STAMPED bundle)");
SHULIB_F6_PIN((requires {
                  static_cast<MotionScheduler& (Chassis::*)() noexcept>(&Chassis::scheduler);
              }),
              "Chassis::scheduler() noexcept -> MotionScheduler&");
SHULIB_F6_PIN((requires {
                  static_cast<const MotionScheduler& (Chassis::*)() const noexcept>(
                      &Chassis::scheduler);
              }),
              "Chassis::scheduler() const noexcept -> const MotionScheduler& (const overload)");

// ── the three frozen public types (a frozen signature over an unfrozen type
//    freezes nothing — existing fields are pinned; the SET stays additive-open) ────
SHULIB_F6_PIN((std::is_same_v<decltype(MotionOptions::timeout), Time>),
              "MotionOptions::timeout : units::Time (TYPED time, D2 — was timeoutSeconds)");
SHULIB_F6_PIN((std::is_same_v<decltype(MotionOptions::maxLinearSpeed), Velocity>),
              "MotionOptions::maxLinearSpeed : units::Velocity");
SHULIB_F6_PIN((std::is_same_v<decltype(MotionOptions::maxAngularSpeed), AngularVelocity>),
              "MotionOptions::maxAngularSpeed : units::AngularVelocity");
SHULIB_F6_PIN((requires {
                  static_cast<void (MotionOptions::*)() const>(&MotionOptions::validate);
              }),
              "MotionOptions::validate() const");
SHULIB_F6_PIN((std::is_same_v<decltype(ChassisConfig::motion), MotionConfig>),
              "ChassisConfig::motion : MotionConfig (passed through WHOLE — the additive path)");
SHULIB_F6_PIN((std::is_same_v<decltype(ChassisConfig::scheduler),
                              shulib::motion::MotionSchedulerConfig>),
              "ChassisConfig::scheduler : MotionSchedulerConfig (passed through WHOLE)");
SHULIB_F6_PIN((std::is_same_v<decltype(TrajectoryResult::exit), ExitReason>),
              "TrajectoryResult::exit : ExitReason");
SHULIB_F6_PIN((std::is_same_v<decltype(TrajectoryResult::completedLegs), int>),
              "TrajectoryResult::completedLegs : int");
SHULIB_F6_PIN((std::is_same_v<decltype(TrajectoryResult::totalLegs), int>),
              "TrajectoryResult::totalLegs : int");
SHULIB_F6_PIN((requires {
                  static_cast<bool (TrajectoryResult::*)() const noexcept>(
                      &TrajectoryResult::succeeded);
              }),
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
// or doesn't say 2.0.
TEST_CASE("F6 pin: the frozen surface is API 2.0 and the version mechanism exists") {
    CHECK(shulib::kApiMajor == 2);
    CHECK(shulib::kApiMinor == 0);
    CHECK(shulib::kApiVersionString[0] == '2');
}

}  // namespace
