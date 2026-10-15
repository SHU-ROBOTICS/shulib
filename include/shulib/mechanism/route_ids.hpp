#pragma once
//
// mechanism/route_ids.hpp — EVERY MECHANISM'S ACTIONS AS ROUTE IDS, automatically (the mechanism
// core, chunk MECH-A1, 2026-10-08; the team lead's ruling of 2026-10-08, MECH-PANEL-requirements §4
// item 7b: "an arm's presets as arm_stow, arm_score; an intake's intake_run, intake_until_piece ...
// a coder no longer registers each id by hand").
//
// ── The ids are DERIVED, never typed ────────────────────────────────────────────────
// From the robot's mechanism list, at BUILD time — the same declarations the autons and the driver
// file name, so an id can never drift from the mechanism it moves. The STEM is the mechanism's
// `.name` in route-id words: lower case, every run of other characters one '_' ("claw pivot 6" ->
// `claw_pivot_6`); a name with no letter or digit in it contributes nothing. The rule, one per kind:
//
//   an ARM or a LIFT   <stem>_<preset> for each preset (move there, hold)       args [time limit ms]
//                      <stem>_home when it has a homing method                  args [time limit ms]
//   a FLYWHEEL         <stem>_<preset> for each preset (spin up, keep spinning) args [time limit ms]
//                      <stem>_stop                                              no args
//   a POWER mechanism  <stem>_run                                               args [volts, ms]
//                      <stem>_until_piece, <stem>_until_gone (a piece sensor declared)
//                                                                               args [volts, time limit ms]
//                      <stem>_stop                                              no args
//   a PNEUMATIC        <stem>_<position> for each position                     no args (its move time)
//
// F8's vocabulary rule holds for every id (config::commandIdValid: 1..23 bytes of [a-z0-9_]). An id
// that would break it, two ids that would be the same (a preset named "home" beside a homing
// method; "claw pivot" and "claw_pivot"), an id that is one of the library's four built-ins
// (`wait_ms`, `brake`, `hold_ms`, `log`), or more ids than a route registry holds (32) FAILS THE
// BUILD at the robot's list — mechanism/wiring.hpp's mechanisms() calls refuseRouteIds() — never a
// silently missing id. routeIdsOf() is constexpr, so a host test or a tool can derive any robot's
// ids with no brain, and the robot panel (G3a Part R) mirrors the rule and its four refusals.
//
// What RUNS an id (the route binder, runner/mechanism_routes.hpp, and the registry's per-id doc) is
// MECH-A4's; registering a robot's ids in a program is MECH-A5's. This file is the rule only.
//
// PROS-free, allocation-free, constexpr.

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "shulib/config/route.hpp"
#include "shulib/mechanism/spec.hpp"

namespace shulib::mechanism {

/// What a route id does.
enum class RouteAction : std::uint8_t {
    GoToPreset = 1,  ///< an arm's or a lift's preset: move there and hold — args [time limit ms]
    Home,            ///< an arm or a lift: find zero by its method — args [time limit ms]
    SpinUp,          ///< a flywheel's preset: spin up, "at speed", keep spinning — args [time limit ms]
    Run,             ///< a power mechanism: run at args[0] volts for args[1] ms
    RunUntilPiece,   ///< a power mechanism: run at args[0] volts until its sensor sees a piece (limit args[1] ms)
    RunUntilGone,    ///< a power mechanism: run at args[0] volts until its sensor stops seeing one (limit args[1] ms)
    Stop,            ///< a power mechanism or a flywheel: its safe state
    SetPosition,     ///< a pneumatic's position: its lines now, its verdict after its move time
};

/// The most route ids one robot's mechanisms may publish: a route registry's capacity — F8's design
/// constant `runner::kMaxCommands` (32), which a host test pins equal (the runner sits above this
/// layer, so it is not included here). The team's own registrations share it.
inline constexpr std::size_t kMaxRouteIds = 32;

/// One published id: its text, whose mechanism (its index in the robot's list), what it does, which
/// preset or position (`index`), and how many args a marker carrying it must give.
struct RouteId {
    /// The id text, NUL-terminated (at most 23 characters — F8's rule).
    char id[config::kMaxMarkerIdBytes]{};
    /// The mechanism's index in the robot's list.
    std::uint8_t mechanism = 0;
    /// What the id does.
    RouteAction action = RouteAction::Stop;
    /// GoToPreset / SpinUp: the preset's index; SetPosition: the position's; 0 otherwise.
    std::uint8_t index = 0;
    /// How many args a marker carrying this id gives (the rule's table, header).
    std::uint8_t argCount = 0;
    /// The id text as a view, bounded by its buffer.
    [[nodiscard]] constexpr std::string_view view() const noexcept {
        return config::commandIdView(id, config::kMaxMarkerIdBytes);
    }
};

/// Why a robot's route ids cannot be published (None = they can).
enum class RouteIdProblem : std::uint8_t {
    None = 0,   ///< every id is publishable
    TooLong,    ///< an id would run past 23 characters (or be empty)
    Duplicate,  ///< two ids would be the same
    Builtin,    ///< an id would be one of the library's own (wait_ms, brake, hold_ms, log)
    TooMany,    ///< more ids than a route registry holds (kMaxRouteIds)
};

/// A robot's route ids, in its list's order — or, at the FIRST problem, what it was and which id.
struct RouteIds {
    /// The ids, in order (every one before the problem, if there is one).
    RouteId ids[kMaxRouteIds]{};
    /// How many of `ids` are real.
    std::uint8_t count = 0;
    /// The first problem, or None.
    RouteIdProblem problem = RouteIdProblem::None;
    /// The id that broke the rule, as far as it was built (47 characters at most) — the panel's
    /// and a test's words for WHICH id.
    char offending[48]{};
};

namespace detail {
/// Append `text` in route-id words (lower case; any other run of characters one '_' joining two
/// words). Returns the new length; past `cap` it keeps counting, so a caller sees the id did not fit.
constexpr std::size_t appendRouteWords(char* out, std::size_t len, std::size_t cap, const char* text) noexcept {
    bool sep = len > 0;  // a '_' joins this part to what came before
    bool wrote = false;
    for (const char* p = text; p != nullptr && *p != '\0'; ++p) {
        char c = *p;
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
        const bool word = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9');
        if (!word) {
            sep = wrote || len > 0;
            continue;
        }
        if (sep && (wrote || len > 0)) {
            if (len < cap) {
                out[len] = '_';
            }
            ++len;
        }
        sep = false;
        if (len < cap) {
            out[len] = c;
        }
        ++len;
        wrote = true;
    }
    return len;
}
/// Copy what was built of a refused id into `r.offending` (bounded) and record the problem.
constexpr void refuseRouteId(RouteIds& r, const char* buf, std::size_t len, RouteIdProblem p) noexcept {
    const std::size_t shown = len < sizeof r.offending - 1 ? len : sizeof r.offending - 1;
    for (std::size_t i = 0; i < shown; ++i) {
        r.offending[i] = buf[i];
    }
    r.problem = p;
}
/// Add `<stem>_<suffix>` for mechanism `mech`, or record the first problem with it (and stop adding).
constexpr void addRouteId(RouteIds& r, std::uint8_t mech, const char* stem, const char* suffix, RouteAction a,
                   std::uint8_t index, std::uint8_t args) noexcept {
    if (r.problem != RouteIdProblem::None) {
        return;
    }
    char buf[64]{};
    std::size_t len = appendRouteWords(buf, 0, sizeof buf - 1, stem);
    len = appendRouteWords(buf, len, sizeof buf - 1, suffix);
    const std::string_view v{buf, len < sizeof buf - 1 ? len : sizeof buf - 1};
    if (len > config::kMaxMarkerIdBytes - 1 || !config::commandIdValid(v)) {
        refuseRouteId(r, buf, len, RouteIdProblem::TooLong);
        return;
    }
    if (config::findBuiltinCommand(v) != nullptr) {
        refuseRouteId(r, buf, len, RouteIdProblem::Builtin);
        return;
    }
    for (std::size_t i = 0; i < r.count; ++i) {
        if (r.ids[i].view() == v) {
            refuseRouteId(r, buf, len, RouteIdProblem::Duplicate);
            return;
        }
    }
    if (r.count == kMaxRouteIds) {
        refuseRouteId(r, buf, len, RouteIdProblem::TooMany);
        return;
    }
    RouteId& id = r.ids[r.count++];
    for (std::size_t i = 0; i < len; ++i) {
        id.id[i] = buf[i];
    }
    id.mechanism = mech;
    id.action = a;
    id.index = index;
    id.argCount = args;
}
}  // namespace detail

/// THE RULE (header): every id a robot's mechanisms publish, in the list's order — or the first
/// problem. A spec of no known kind publishes nothing (specRefusal refuses it first).
[[nodiscard]] constexpr RouteIds routeIdsOf(std::span<const MechanismSpec> specs) noexcept {
    RouteIds r{};
    for (std::size_t m = 0; m < specs.size() && m < kMaxMechanisms; ++m) {
        const MechanismSpec& s = specs[m];
        const auto mi = static_cast<std::uint8_t>(m);
        switch (s.kind) {
            case Kind::Position:
                for (std::size_t i = 0; i < s.position.presetCount && i < kMaxPresets; ++i) {
                    detail::addRouteId(r, mi, s.name, s.position.presets[i].name, RouteAction::GoToPreset,
                                      static_cast<std::uint8_t>(i), 1);
                }
                if (s.position.home != HomeMethod::None) {
                    detail::addRouteId(r, mi, s.name, "home", RouteAction::Home, 0, 1);
                }
                break;
            case Kind::Speed:
                for (std::size_t i = 0; i < s.speed.presetCount && i < kMaxPresets; ++i) {
                    detail::addRouteId(r, mi, s.name, s.speed.presets[i].name, RouteAction::SpinUp,
                                      static_cast<std::uint8_t>(i), 1);
                }
                detail::addRouteId(r, mi, s.name, "stop", RouteAction::Stop, 0, 0);
                break;
            case Kind::Power:
                detail::addRouteId(r, mi, s.name, "run", RouteAction::Run, 0, 2);
                if (s.pieceSensor.kind != SensorKind::None) {
                    detail::addRouteId(r, mi, s.name, "until piece", RouteAction::RunUntilPiece, 0, 2);
                    detail::addRouteId(r, mi, s.name, "until gone", RouteAction::RunUntilGone, 0, 2);
                }
                detail::addRouteId(r, mi, s.name, "stop", RouteAction::Stop, 0, 0);
                break;
            case Kind::Pneumatic:
                for (std::size_t i = 0; i < s.pneumatic.positionCount && i < kMaxPneumaticPositions; ++i) {
                    detail::addRouteId(r, mi, s.name, s.pneumatic.positions[i].name, RouteAction::SetPosition,
                                      static_cast<std::uint8_t>(i), 0);
                }
                break;
        }
    }
    return r;
}

/// The sentence for a problem — the boot's and the panel's words (G3a's validator mirrors it); ""
/// for None.
[[nodiscard]] constexpr const char* routeIdProblemText(RouteIdProblem p) noexcept {
    switch (p) {
        case RouteIdProblem::None:
            return "";
        case RouteIdProblem::TooLong:
            return "a route id does not fit 23 characters: shorten the mechanism's or the preset's name";
        case RouteIdProblem::Duplicate:
            return "two route ids are the same: rename a mechanism, a preset or a position";
        case RouteIdProblem::Builtin:
            return "a route id is one of the library's own (wait_ms, brake, hold_ms, log): rename it";
        case RouteIdProblem::TooMany:
            return "more route ids than a route registry holds (32): fewer presets or positions";
    }
    return "an unknown route id problem";
}

/// The route-id problems as NAMED FUNCTIONS — deliberately not constexpr (spec.hpp's idiom): GCC
/// names the reason at the robot's list.
namespace refused {
/// RouteIdProblem::TooLong's name at the robot's list.
inline void a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name() {}
/// RouteIdProblem::Duplicate's name at the robot's list.
inline void two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position() {}
/// RouteIdProblem::Builtin's name at the robot's list.
inline void a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log() {}
/// RouteIdProblem::TooMany's name at the robot's list.
inline void more_route_ids_than_a_route_registry_holds_32() {}
}  // namespace refused

/// Stop the build with `p`'s named function (consteval contexts only); None does nothing.
consteval void refuseRouteIds(RouteIdProblem p) {
    switch (p) {
        case RouteIdProblem::None:
            return;
        case RouteIdProblem::TooLong:
            refused::a_route_id_does_not_fit_23_characters_shorten_the_mechanism_or_preset_name();
            return;
        case RouteIdProblem::Duplicate:
            refused::two_route_ids_are_the_same_rename_a_mechanism_a_preset_or_a_position();
            return;
        case RouteIdProblem::Builtin:
            refused::a_route_id_is_one_of_the_librarys_own_wait_ms_brake_hold_ms_log();
            return;
        case RouteIdProblem::TooMany:
            refused::more_route_ids_than_a_route_registry_holds_32();
            return;
    }
}

}  // namespace shulib::mechanism
