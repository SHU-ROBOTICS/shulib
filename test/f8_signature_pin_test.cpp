// F8 SIGNATURE PIN — the path/command-vocabulary freeze, enforced structurally (chunk G2).
//
// Bug this file catches: an accidental reshape of the frozen `paths[]` vocabulary — the
// in-memory types a route is made of (`config::Route`/`Waypoint`/`Marker`/`Constraints`), the
// enum VALUES a generator persists as literals, the capacity constants, the registry's public
// surface, `PathRunner::run`, `RunReport`'s fields, and the built-in id strings. The Freeze
// Register (docs/roadmap.md, row F8) promises that surface changes only with a major API bump
// plus a migration note; a promise with no mechanism is a comment.
//
// This is D2's F6 pin and D3's F10 pin, applied to Phase G, and it inherits both of their
// hard-won lessons verbatim:
//
//   * Every member pin routes through a CONCEPT TEMPLATED ON THE CLASS. In a non-dependent
//     context an invalid static_cast inside a requires-expression is a HARD error and the named
//     static_assert never gets to speak; in a dependent context the same failure is a
//     substitution failure, the concept evaluates false, and the message below is what the
//     reader sees. The bar is "fails the build NAMING the freeze", and "fails the build" is not
//     that.
//   * Every noexcept-carrying pin pairs the static_cast with a COMPOUND REQUIREMENT
//     `{ call } noexcept`. For a NON-overloaded member the compiler accepts a static_cast that
//     ADDS noexcept, so the cast alone cannot see noexcept being DROPPED — and version.hpp calls
//     a noexcept change BREAKING. Do not re-open that hole.
//
// ═══ WHAT F8 FREEZES, AND WHAT STAYS ADDITIVE ════════════════════════════════════════
// FROZEN: the field names and types above; the enumerator VALUES of `WaypointMotion` and
// `HeadingMode` (a generator emits them as integer literals, so a renumbering is invisible at
// every call site and catastrophic in a file written last season); the capacity constants; the
// four built-in id STRINGS; `CommandRegistry`'s members; `PathRunner::run`'s signature;
// `RunReport`'s fields; and the manifest's field names and record order (pinned in
// manifest_test.cpp's golden, which is the artifact's own pin).
//
// ADDITIVE and expressly legal under `kApiMinor`: new enumerators AT THE END, new built-in ids
// AT THE END of `config::kBuiltinCommands`, new `RunReport` fields, new registry members, new
// optional fields in the file schema and in a manifest line.
//
// ═══ WHAT IS DELIBERATELY NOT PINNED HERE ════════════════════════════════════════════
// The exact WORDING of the runner's log lines (the behaviour — one Warn per unknown id naming
// the id and the index, one Warn on a stop, one Info per `log` marker — is frozen; the sentence
// is not), and `Route::valid()`'s refusal MESSAGES (same reasoning: the rules are frozen, their
// prose is not). Said out loud, because silence in a freeze reads as "frozen too" (D2 ruling
// A2's lesson).
//
// IF THIS FILE JUST FAILED YOUR BUILD: either the change is accidental (revert it) or it is an
// intended breaking change — then read include/shulib/version.hpp: bump kApiMajor, write the
// migration note, update Freeze Register row F8, and update these pins LAST.

#include "doctest.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>
#include <type_traits>
#include <utility>

#include "shulib/config/route.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/runner/manifest.hpp"
#include "shulib/runner/path_runner.hpp"
#include "shulib/version.hpp"

namespace {

using shulib::config::BuiltinCommand;
using shulib::config::Constraints;
using shulib::config::HeadingMode;
using shulib::config::Marker;
using shulib::config::Route;
using shulib::config::Waypoint;
using shulib::config::WaypointMotion;
using shulib::control::ExitReason;
using shulib::manipulation::MechanismOutcome;
using shulib::math::Angle;
using shulib::runner::CommandEntry;
using shulib::runner::CommandRegistry;
using shulib::runner::Handler;
using shulib::runner::PathRunner;
using shulib::runner::RunReport;
using shulib::runner::RunStopCause;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

// One uniform failure prefix so a tripped pin is unmistakable in a build log.
#define SHULIB_F8_PIN(cond, member)                                                  \
    static_assert(cond, "F8 FREEZE VIOLATION: the frozen shape of " member           \
                        " changed (paths[] + the command-id vocabulary, locked "     \
                        "2026-09-18, chunk G2). Accidental? Revert. Intended? That " \
                        "is a BREAKING change: see include/shulib/version.hpp — "    \
                        "bump kApiMajor, write the migration note, update Freeze "   \
                        "Register row F8, THEN this pin.")

// ── the capacities (a generator sizes its output by these; G3 reads them) ─────────────
SHULIB_F8_PIN(shulib::config::kMaxWaypoints == 32, "config::kMaxWaypoints == 32");
SHULIB_F8_PIN(shulib::config::kMaxMarkersPerWaypoint == 4,
              "config::kMaxMarkersPerWaypoint == 4");
SHULIB_F8_PIN(shulib::config::kMaxMarkerArgs == 2, "config::kMaxMarkerArgs == 2");
SHULIB_F8_PIN(shulib::config::kMaxMarkerIdBytes == 24, "config::kMaxMarkerIdBytes == 24");
SHULIB_F8_PIN(shulib::runner::kMaxCommands == 32, "runner::kMaxCommands == 32");
SHULIB_F8_PIN(shulib::runner::kManifestSchemaVersion == 1,
              "runner::kManifestSchemaVersion == 1");

// ── the enum VALUES, because a generator persists them as integer literals ────────────
// Renumbering or re-meaning one is BREAKING and INVISIBLE at every call site: a `.vexbot`
// exported last season carries the NUMBER, not the name.
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::Start) == 1, "WaypointMotion::Start == 1");
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::MoveToPose) == 2,
              "WaypointMotion::MoveToPose == 2");
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::DriveTo) == 3, "WaypointMotion::DriveTo == 3");
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::TurnTo) == 4, "WaypointMotion::TurnTo == 4");
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::StrafeTo) == 5, "WaypointMotion::StrafeTo == 5");
SHULIB_F8_PIN(static_cast<int>(WaypointMotion::Hold) == 6, "WaypointMotion::Hold == 6");
SHULIB_F8_PIN((std::is_same_v<std::underlying_type_t<WaypointMotion>, std::uint8_t>),
              "WaypointMotion's underlying type (std::uint8_t — one byte on the wire)");

SHULIB_F8_PIN(static_cast<int>(HeadingMode::Fixed) == 1, "HeadingMode::Fixed == 1");
SHULIB_F8_PIN(static_cast<int>(HeadingMode::Free) == 2, "HeadingMode::Free == 2");
SHULIB_F8_PIN((std::is_same_v<std::underlying_type_t<HeadingMode>, std::uint8_t>),
              "HeadingMode's underlying type (std::uint8_t)");

SHULIB_F8_PIN(static_cast<int>(RunStopCause::None) == 0, "RunStopCause::None == 0");
SHULIB_F8_PIN(static_cast<int>(RunStopCause::MotionFailed) == 1,
              "RunStopCause::MotionFailed == 1 (append-only enum)");
SHULIB_F8_PIN(static_cast<int>(RunStopCause::MarkerFailed) == 2,
              "RunStopCause::MarkerFailed == 2 (append-only enum)");

// ── config::Constraints ───────────────────────────────────────────────────────────────
template <typename T>
concept F8ConstraintsMaxLinear = requires {
    requires std::is_same_v<decltype(T::maxLinear), Velocity>;
};
SHULIB_F8_PIN(F8ConstraintsMaxLinear<Constraints>, "Constraints::maxLinear : units::Velocity");

template <typename T>
concept F8ConstraintsMaxAngular = requires {
    requires std::is_same_v<decltype(T::maxAngular), AngularVelocity>;
};
SHULIB_F8_PIN(F8ConstraintsMaxAngular<Constraints>,
              "Constraints::maxAngular : units::AngularVelocity");

template <typename T>
concept F8ConstraintsTimeout = requires { requires std::is_same_v<decltype(T::timeout), Time>; };
SHULIB_F8_PIN(F8ConstraintsTimeout<Constraints>, "Constraints::timeout : units::Time");

// ── config::Marker ────────────────────────────────────────────────────────────────────
template <typename T>
concept F8MarkerId = requires {
    requires std::is_same_v<decltype(T::id), char[shulib::config::kMaxMarkerIdBytes]>;
};
SHULIB_F8_PIN(F8MarkerId<Marker>, "Marker::id : char[kMaxMarkerIdBytes]");

template <typename T>
concept F8MarkerArgs = requires {
    requires std::is_same_v<decltype(T::args),
                            std::int32_t[shulib::config::kMaxMarkerArgs]>;
};
SHULIB_F8_PIN(F8MarkerArgs<Marker>,
              "Marker::args : std::int32_t[kMaxMarkerArgs] (typed integers, never a variant)");

template <typename T>
concept F8MarkerArgCount = requires {
    requires std::is_same_v<decltype(T::argCount), std::uint8_t>;
};
SHULIB_F8_PIN(F8MarkerArgCount<Marker>, "Marker::argCount : std::uint8_t");

// ── config::Waypoint ──────────────────────────────────────────────────────────────────
template <typename T>
concept F8WaypointX = requires { requires std::is_same_v<decltype(T::x), Length>; };
SHULIB_F8_PIN(F8WaypointX<Waypoint>, "Waypoint::x : units::Length");

template <typename T>
concept F8WaypointY = requires { requires std::is_same_v<decltype(T::y), Length>; };
SHULIB_F8_PIN(F8WaypointY<Waypoint>, "Waypoint::y : units::Length");

template <typename T>
concept F8WaypointHeading = requires { requires std::is_same_v<decltype(T::heading), Angle>; };
SHULIB_F8_PIN(F8WaypointHeading<Waypoint>, "Waypoint::heading : math::Angle");

template <typename T>
concept F8WaypointHeadingMode = requires {
    requires std::is_same_v<decltype(T::headingMode), HeadingMode>;
};
SHULIB_F8_PIN(F8WaypointHeadingMode<Waypoint>, "Waypoint::headingMode : HeadingMode");

template <typename T>
concept F8WaypointMotionField = requires {
    requires std::is_same_v<decltype(T::motion), WaypointMotion>;
};
SHULIB_F8_PIN(F8WaypointMotionField<Waypoint>, "Waypoint::motion : WaypointMotion");

template <typename T>
concept F8WaypointReverse = requires { requires std::is_same_v<decltype(T::reverse), bool>; };
SHULIB_F8_PIN(F8WaypointReverse<Waypoint>, "Waypoint::reverse : bool");

template <typename T>
concept F8WaypointConstraints = requires {
    requires std::is_same_v<decltype(T::constraints), Constraints>;
};
SHULIB_F8_PIN(F8WaypointConstraints<Waypoint>, "Waypoint::constraints : Constraints");

template <typename T>
concept F8WaypointMarkers = requires {
    requires std::is_same_v<decltype(T::markers),
                            Marker[shulib::config::kMaxMarkersPerWaypoint]>;
};
SHULIB_F8_PIN(F8WaypointMarkers<Waypoint>, "Waypoint::markers : Marker[kMaxMarkersPerWaypoint]");

template <typename T>
concept F8WaypointMarkerCount = requires {
    requires std::is_same_v<decltype(T::markerCount), std::uint8_t>;
};
SHULIB_F8_PIN(F8WaypointMarkerCount<Waypoint>, "Waypoint::markerCount : std::uint8_t");

// ── config::Route ─────────────────────────────────────────────────────────────────────
template <typename T>
concept F8RouteName = requires { requires std::is_same_v<decltype(T::name), char[32]>; };
SHULIB_F8_PIN(F8RouteName<Route>, "Route::name : char[32]");

template <typename T>
concept F8RouteWaypoints = requires {
    requires std::is_same_v<decltype(T::waypoints), Waypoint[shulib::config::kMaxWaypoints]>;
};
SHULIB_F8_PIN(F8RouteWaypoints<Route>, "Route::waypoints : Waypoint[kMaxWaypoints]");

template <typename T>
concept F8RouteCount = requires { requires std::is_same_v<decltype(T::count), std::uint8_t>; };
SHULIB_F8_PIN(F8RouteCount<Route>, "Route::count : std::uint8_t");

template <typename T>
concept F8RouteValid = requires(const T& r) {
    static_cast<bool (T::*)(char*, std::size_t) const noexcept>(&T::valid);
    { r.valid(nullptr, 0) } noexcept -> std::same_as<bool>;  // D2 hole #1: the cast alone is blind
    { r.valid() } -> std::same_as<bool>;                     // the defaulted spelling
};
SHULIB_F8_PIN(F8RouteValid<Route>, "Route::valid(char*, size_t) const noexcept -> bool");

// A Route must stay a LITERAL TYPE: G3's codegen emits `inline constexpr Route kRoutes[]`, and a
// field that quietly stopped being constexpr-constructible would break the generator long after
// this header was last read.
SHULIB_F8_PIN(std::is_trivially_copyable_v<Route> && std::is_trivially_copyable_v<Waypoint>
                  && std::is_trivially_copyable_v<Marker>,
              "Route/Waypoint/Marker staying trivially copyable (a route is written to a card "
              "and emitted as a constant expression)");

// ── the command-id vocabulary rules ───────────────────────────────────────────────────
SHULIB_F8_PIN(shulib::config::commandIdValid("intake_in"),
              "commandIdValid accepting lower-snake ids");
SHULIB_F8_PIN(!shulib::config::commandIdValid(""), "commandIdValid refusing the empty id");
SHULIB_F8_PIN(!shulib::config::commandIdValid("Intake_In"),
              "commandIdValid refusing uppercase (the no-escaping-ever rule)");
SHULIB_F8_PIN(!shulib::config::commandIdValid("intake in"), "commandIdValid refusing a space");
SHULIB_F8_PIN(!shulib::config::commandIdValid("intake-in"), "commandIdValid refusing a hyphen");
SHULIB_F8_PIN(shulib::config::commandIdValid("aaaaaaaaaaaaaaaaaaaaaaa"),
              "commandIdValid accepting 23 bytes (the buffer minus its terminator)");
SHULIB_F8_PIN(!shulib::config::commandIdValid("aaaaaaaaaaaaaaaaaaaaaaaa"),
              "commandIdValid refusing 24 bytes");

// ── the built-in ids: the STRINGS and their arities are the frozen part ───────────────
SHULIB_F8_PIN(shulib::config::kBuiltinCommandCount == 4,
              "config::kBuiltinCommandCount == 4 (new built-ins APPEND; they never replace)");
SHULIB_F8_PIN(std::string_view{shulib::config::kBuiltinCommands[0].id} == "wait_ms",
              "built-in 0 is \"wait_ms\" (the manifest's order is part of its golden)");
SHULIB_F8_PIN(std::string_view{shulib::config::kBuiltinCommands[1].id} == "brake",
              "built-in 1 is \"brake\"");
SHULIB_F8_PIN(std::string_view{shulib::config::kBuiltinCommands[2].id} == "hold_ms",
              "built-in 2 is \"hold_ms\"");
SHULIB_F8_PIN(std::string_view{shulib::config::kBuiltinCommands[3].id} == "log",
              "built-in 3 is \"log\"");
SHULIB_F8_PIN(shulib::config::kBuiltinCommands[0].minArgs == 1
                  && shulib::config::kBuiltinCommands[0].maxArgs == 1,
              "wait_ms taking exactly one arg");
SHULIB_F8_PIN(shulib::config::kBuiltinCommands[1].maxArgs == 0, "brake taking no args");
SHULIB_F8_PIN(shulib::config::kBuiltinCommands[2].minArgs == 1
                  && shulib::config::kBuiltinCommands[2].maxArgs == 1,
              "hold_ms taking exactly one arg");
SHULIB_F8_PIN(shulib::config::kBuiltinCommands[3].maxArgs == 2, "log taking up to two args");
SHULIB_F8_PIN(shulib::config::findBuiltinCommand("wait_ms") != nullptr
                  && shulib::config::findBuiltinCommand("intake_in") == nullptr,
              "findBuiltinCommand answering exactly the built-in set");

// `NONE` is NOT an id — the answer to vexbuilder-integration.md's open question. A waypoint IS a
// segment, so the legacy planner's boundary rows have no successor in this vocabulary, and G4's
// importer folds them into waypoints. A future built-in called "none" would silently give the
// legacy id a meaning it was ruled out of having.
SHULIB_F8_PIN(shulib::config::findBuiltinCommand("none") == nullptr,
              "\"none\" NOT being a command id (a waypoint IS a segment — the NONE ruling)");

template <typename T>
concept F8BuiltinFields = requires {
    requires std::is_same_v<decltype(T::id), const char*>;
    requires std::is_same_v<decltype(T::minArgs), std::uint8_t>;
    requires std::is_same_v<decltype(T::maxArgs), std::uint8_t>;
    requires std::is_same_v<decltype(T::firstArgIsDurationMs), bool>;
    requires std::is_same_v<decltype(T::doc), const char*>;
};
SHULIB_F8_PIN(F8BuiltinFields<BuiltinCommand>,
              "BuiltinCommand's five fields (id / minArgs / maxArgs / firstArgIsDurationMs / doc)");

// ── runner::Handler and runner::CommandEntry ─────────────────────────────────────────
template <typename T>
concept F8HandlerFn = requires {
    requires std::is_same_v<decltype(T::fn),
                            MechanismOutcome (*)(void*, const Marker&)>;
};
SHULIB_F8_PIN(F8HandlerFn<Handler>,
              "Handler::fn : MechanismOutcome (*)(void*, const config::Marker&) — a plain "
              "function pointer, never std::function");

template <typename T>
concept F8HandlerCtx = requires { requires std::is_same_v<decltype(T::ctx), void*>; };
SHULIB_F8_PIN(F8HandlerCtx<Handler>, "Handler::ctx : void* (non-owning)");

template <typename T>
concept F8EntryId = requires {
    requires std::is_same_v<decltype(T::id), char[shulib::config::kMaxMarkerIdBytes]>;
};
SHULIB_F8_PIN(F8EntryId<CommandEntry>, "CommandEntry::id : char[kMaxMarkerIdBytes]");

template <typename T>
concept F8EntryArgCount = requires {
    requires std::is_same_v<decltype(T::argCount), std::uint8_t>;
};
SHULIB_F8_PIN(F8EntryArgCount<CommandEntry>, "CommandEntry::argCount : std::uint8_t");

template <typename T>
concept F8EntryHandler = requires { requires std::is_same_v<decltype(T::handler), Handler>; };
SHULIB_F8_PIN(F8EntryHandler<CommandEntry>, "CommandEntry::handler : Handler");

template <typename T>
concept F8EntryIdView = requires(const T& e) {
    static_cast<std::string_view (T::*)() const noexcept>(&T::idView);
    { e.idView() } noexcept -> std::same_as<std::string_view>;
};
SHULIB_F8_PIN(F8EntryIdView<CommandEntry>,
              "CommandEntry::idView() const noexcept -> std::string_view");

// ── runner::CommandRegistry ───────────────────────────────────────────────────────────
using VoidFn = void (*)();

template <typename R>
concept F8RegistryOnHandler = requires {
    static_cast<void (R::*)(std::string_view, Handler, std::uint8_t)>(&R::on);
};
SHULIB_F8_PIN(F8RegistryOnHandler<CommandRegistry>,
              "CommandRegistry::on(string_view, Handler, uint8_t) -> void");

template <typename R>
concept F8RegistryOnCallable = requires {
    static_cast<void (R::*)(std::string_view, VoidFn&, std::uint8_t)>(
        &R::template on<VoidFn>);
};
SHULIB_F8_PIN(F8RegistryOnCallable<CommandRegistry>,
              "CommandRegistry::on(string_view, Callable&, uint8_t) — the NON-OWNING template "
              "overload (a `Callable&`, so a temporary lambda cannot be registered)");

template <typename R>
concept F8RegistryFind = requires(const R& r) {
    static_cast<const Handler* (R::*)(std::string_view) const noexcept>(&R::find);
    { r.find("x") } noexcept -> std::same_as<const Handler*>;
};
SHULIB_F8_PIN(F8RegistryFind<CommandRegistry>,
              "CommandRegistry::find(string_view) const noexcept -> const Handler*");

template <typename R>
concept F8RegistryCount = requires(const R& r) {
    static_cast<std::size_t (R::*)() const noexcept>(&R::count);
    { r.count() } noexcept -> std::same_as<std::size_t>;
};
SHULIB_F8_PIN(F8RegistryCount<CommandRegistry>,
              "CommandRegistry::count() const noexcept -> std::size_t");

template <typename R>
concept F8RegistryIds = requires(const R& r) {
    static_cast<std::span<const CommandEntry> (R::*)() const noexcept>(&R::ids);
    { r.ids() } noexcept -> std::same_as<std::span<const CommandEntry>>;
};
SHULIB_F8_PIN(F8RegistryIds<CommandRegistry>,
              "CommandRegistry::ids() const noexcept -> std::span<const CommandEntry>");

// The defaulted argCount spelling: default arguments are not part of a function type, so dropping
// one would break `registry.on("intake_in", fn)` at every call site without moving a pin above.
template <typename R>
concept F8RegistryTerseSpellings = requires(R& r, VoidFn& fn, Handler h) {
    r.on("x", fn);
    r.on("x", h);
};
SHULIB_F8_PIN(F8RegistryTerseSpellings<CommandRegistry>,
              "the defaulted argCount spellings on(id, callable) / on(id, handler)");

// ── runner::RunReport ─────────────────────────────────────────────────────────────────
template <typename T>
concept F8ReportStoppedAt = requires { requires std::is_same_v<decltype(T::stoppedAt), int>; };
SHULIB_F8_PIN(F8ReportStoppedAt<RunReport>,
              "RunReport::stoppedAt : int (0-BASED, -1 = nothing stopped the route)");

template <typename T>
concept F8ReportReason = requires { requires std::is_same_v<decltype(T::reason), RunStopCause>; };
SHULIB_F8_PIN(F8ReportReason<RunReport>, "RunReport::reason : RunStopCause");

template <typename T>
concept F8ReportLegsRun = requires { requires std::is_same_v<decltype(T::legsRun), int>; };
SHULIB_F8_PIN(F8ReportLegsRun<RunReport>, "RunReport::legsRun : int");

template <typename T>
concept F8ReportLegsSkipped = requires {
    requires std::is_same_v<decltype(T::legsSkipped), int>;
};
SHULIB_F8_PIN(F8ReportLegsSkipped<RunReport>, "RunReport::legsSkipped : int");

template <typename T>
concept F8ReportMarkersFired = requires {
    requires std::is_same_v<decltype(T::markersFired), int>;
};
SHULIB_F8_PIN(F8ReportMarkersFired<RunReport>, "RunReport::markersFired : int");

template <typename T>
concept F8ReportUnknownIds = requires { requires std::is_same_v<decltype(T::unknownIds), int>; };
SHULIB_F8_PIN(F8ReportUnknownIds<RunReport>, "RunReport::unknownIds : int");

template <typename T>
concept F8ReportLastExit = requires { requires std::is_same_v<decltype(T::lastExit), ExitReason>; };
SHULIB_F8_PIN(F8ReportLastExit<RunReport>, "RunReport::lastExit : control::ExitReason");

template <typename T>
concept F8ReportOk = requires(const T& r) {
    static_cast<bool (T::*)() const noexcept>(&T::ok);
    { r.ok() } noexcept -> std::same_as<bool>;
};
SHULIB_F8_PIN(F8ReportOk<RunReport>, "RunReport::ok() const noexcept -> bool");

// ── runner::PathRunner ────────────────────────────────────────────────────────────────
SHULIB_F8_PIN((std::is_constructible_v<PathRunner, shulib::chassis::Chassis&,
                                       const CommandRegistry&, shulib::hal::ITelemetrySink&>),
              "PathRunner(Chassis&, const CommandRegistry&, ITelemetrySink&)");
SHULIB_F8_PIN((std::is_nothrow_constructible_v<PathRunner, shulib::chassis::Chassis&,
                                               const CommandRegistry&,
                                               shulib::hal::ITelemetrySink&>),
              "PathRunner's constructor staying noexcept");
SHULIB_F8_PIN(!std::is_copy_constructible_v<PathRunner> && !std::is_copy_assignable_v<PathRunner>,
              "PathRunner non-copyability (it holds references; a second handle would be a "
              "second opinion about one run)");
SHULIB_F8_PIN(!std::is_move_constructible_v<PathRunner> && !std::is_move_assignable_v<PathRunner>,
              "PathRunner non-movability");

template <typename P>
concept F8RunnerRun = requires {
    static_cast<RunReport (P::*)(const Route&)>(&P::run);
};
SHULIB_F8_PIN(F8RunnerRun<PathRunner>, "PathRunner::run(const config::Route&) -> RunReport");

// The INVERSE noexcept pin (the F6/F10 lesson): `run()` throws the facade's preconditions
// through, so gaining noexcept would convert a loud programming error into std::terminate — and
// a static_cast that ADDS noexcept is accepted, so the cast above cannot see it.
static_assert(!noexcept(std::declval<PathRunner&>().run(std::declval<const Route&>())),
              "F8 FREEZE VIOLATION: PathRunner::run GAINED noexcept. A change to a frozen "
              "signature's noexcept shape is BREAKING IN EITHER DIRECTION "
              "(include/shulib/version.hpp): it changes the member's type, and it turns a "
              "precondition throw into std::terminate. Accidental? Revert. Intended? Bump "
              "kApiMajor, write the migration note, update Freeze Register row F8, THEN this pin.");

// ── runner::writeManifest ─────────────────────────────────────────────────────────────
SHULIB_F8_PIN((std::is_same_v<decltype(&shulib::runner::writeManifest),
                              void (*)(const CommandRegistry&, shulib::hal::ICharSink&)>),
              "writeManifest(const CommandRegistry&, hal::ICharSink&) -> void");

#undef SHULIB_F8_PIN

// The pins above are the test; this case exists so the file registers in the runner and pins the
// version constants the freeze policy hangs off — the F6/F10 twins' case, verbatim in intent.
//
// kApiMinor is deliberately NOT pinned to a value: additive growth (a new built-in, an appended
// enumerator, a new RunReport field) is the intended path and must never fail this pin. Its
// ABSENCE is stated here rather than implied by a check that cannot fail — the vacuous
// `kApiMinor >= 0` that the F1 reviewer found in both twins.
//
// Bug caught: F8 flips to LOCKED while the version mechanism the register promises does not say
// major version 2, or the printable token drifts from the numbers it claims to print (that token
// labels every on-robot transcript and every blackbox file).
TEST_CASE("F8 pin: the frozen path vocabulary is API major 2 and the version mechanism exists") {
    CHECK(shulib::kApiMajor == 2);

    char expected[16];
    std::snprintf(expected, sizeof expected, "%d.%d", shulib::kApiMajor, shulib::kApiMinor);
    CHECK(std::string_view{shulib::kApiVersionString} == std::string_view{expected});

    // F8 is an ADDITIVE growth of the 2.x surface, so the minor must have moved past F1's 1.
    CHECK(shulib::kApiMinor >= 2);
}

}  // namespace
