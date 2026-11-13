#pragma once
//
// Route — an autonomous routine as DATA (chunk G1 Part 1, 2026-09-18).
//
// ── Why this type exists, and why it carries no code ────────────────────────────────
// The team has tried to make autons data three times (a generated C array, a CSV, a
// hard-coded vector — docs/legacy-command-vocabulary.md §1) and what was missing every
// time was the RUNNER, not the format. G2 builds the runner. This header builds the thing
// it runs: a fixed-capacity, allocation-free, constexpr-constructible value that says
// where the robot goes and what it does on the way.
//
// The legacy planner embedded C++ snippets in its rows (`code_template:"mech.intakeIn();"`).
// That is rejected outright (ruling B5, master plan §16.3): a marker here is an ID plus
// TYPED ARGUMENTS, and binding an id to an action is the student's one line in the command
// registry. Data that can execute is not data — it is a second, unreviewable source file,
// and it is what makes "a non-coder authors the strategy" impossible to keep honest.
//
// ── The vocabulary is the facade's, plus Start and Hold ─────────────────────────────
// Every Motion below maps onto a verb chassis.hpp already has (ruling B5). A waypoint that
// needs a verb the library does not have is a request for a LIBRARY FEATURE, filed as
// such — never a silent no-op. That is why an unknown Motion byte is INVALID at valid()
// while an unknown MARKER id is not: G2 warns and skips an unknown id (§16.3), because
// marker ids are an open vocabulary a team extends, whereas the motion set is the library's
// own contract and a robot cannot guess what a verb it has never heard of should do.
//
// ── EXECUTION IS NOT HERE ───────────────────────────────────────────────────────────
// Nothing in this file runs a route. G2's PathRunner walks the waypoints, calls the facade
// verbs and fires the markers through the registry. Part 1 pins the SHAPE and the rules
// that say when the data is lying; that is deliberately all.
//
// NO HARDWARE CLAIM. No Route has ever been executed at all, on a plant or on a robot.
//
// PROS-free, sim-free, allocation-free; no std::string, no std::function, no pointers in
// the data types.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <string_view>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::config {

/// The most waypoints one Route may carry. A DESIGN constant (ruling B6): a route longer than
/// this is two routes, which is also how a person should be thinking about it. Not a hardware
/// belief — nothing measured says 32.
inline constexpr std::size_t kMaxWaypoints = 32;

/// The most markers one waypoint may fire. Design constant (B6).
inline constexpr std::size_t kMaxMarkersPerWaypoint = 4;

/// The most typed arguments one marker may carry. Design constant (B6): two covers
/// `{id:"lift_to_level", args:{3}}` and every parametric primitive sketched so far, and a marker
/// that needs a third is a marker that wants a different id.
inline constexpr std::size_t kMaxMarkerArgs = 2;

/// Bytes of storage one command id occupies, NUL INCLUDED — so the longest id TEXT is 23 bytes.
/// G1 wrote `char id[24]` and never named the number; G2 needs the same bound in three more
/// places (the registry's key buffer, the manifest writer, the id-validity rule), and a capacity
/// spelled as a literal in four files is a capacity that will disagree with itself. Design
/// constant (B6), not a hardware belief: 23 characters is longer than every id anyone has
/// proposed, and short enough that a whole registry of 32 is under a kilobyte.
inline constexpr std::size_t kMaxMarkerIdBytes = 24;

/// The text of a fixed-width, possibly UNTERMINATED character buffer (a command id, a route
/// name), bounded by the BUFFER rather than by `strlen` — the same hazard `StaticRoutes::route()`
/// guards against, since a generator may fill the last byte. `capacity` is the buffer's size in
/// bytes.
[[nodiscard]] constexpr std::string_view commandIdView(const char* id,
                                                       std::size_t capacity) noexcept {
    std::size_t n = 0;
    while (n < capacity && id[n] != '\0') {
        ++n;
    }
    return std::string_view{id, n};
}

/// Is `id` a legal command id? Non-empty, at most `kMaxMarkerIdBytes - 1` characters, and drawn
/// from `[a-z0-9_]` only.
///
/// The charset is deliberately narrow (G2's REJECTED list): an id is the SAME STRING in a C++
/// source line, in a manifest line, in a `.vexbot` file and on a picker's button, and a vocabulary
/// with no uppercase, no spaces, no punctuation and no non-ASCII is one that never needs escaping
/// on any of those four paths. The length bound is what makes the id storable in the fixed buffer
/// WITH a terminator — a 24-byte id that filled its buffer could be compared but never printed.
[[nodiscard]] constexpr bool commandIdValid(std::string_view id) noexcept {
    if (id.empty() || id.size() > kMaxMarkerIdBytes - 1) {
        return false;
    }
    for (const char c : id) {
        const bool ok = (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_';
        if (!ok) {
            return false;
        }
    }
    return true;
}

/// One LIBRARY-OWNED command id: a marker a team never registers because the runner itself
/// performs it (G2 ruling R3). The table below is the whole set; a team that registered `brake`
/// to mean something else would break every route another team exported, so `CommandRegistry`
/// refuses one as a precondition violation.
///
/// The SPEC lives here, at the data layer, rather than in `runner/`, because an id, its arity and
/// its meaning are facts about the ROUTE VOCABULARY — `Route::valid()` checks them, the manifest
/// publishes them and F8 freezes them; only the DOING of them belongs to the runner. The reverse
/// arrangement would make `config/` depend on `runner/`, which is upside down.
struct BuiltinCommand {
    /// The id text, e.g. "wait_ms". Always `commandIdValid()`.
    const char* id;
    /// The fewest typed args the marker must carry.
    std::uint8_t minArgs;
    /// The most typed args the marker may carry (<= kMaxMarkerArgs).
    std::uint8_t maxArgs;
    /// True when `args[0]` is a DURATION IN MILLISECONDS and must therefore be > 0 — the facade
    /// verbs behind these ids refuse a non-positive duration at the door, so a route carrying one
    /// is a routine that throws mid-auton instead of one that is refused before it runs.
    bool firstArgIsDurationMs;
    /// What it does, in one line — published verbatim in the manifest a picker reads.
    const char* doc;
};

/// Every built-in id, in the order the manifest publishes them. **APPEND-ONLY**: the order is part
/// of the manifest's byte-exact contract (G2 landmine 5) and the id strings are frozen by F8.
inline constexpr BuiltinCommand kBuiltinCommands[] = {
    {"wait_ms", 1, 1, true,
     "wait, commanding nothing, for args[0] milliseconds (the drive keeps its last state)"},
    {"brake", 0, 0, false,
     "stop the drive (0 V + Brake) and block until the estimate certifies rest"},
    {"hold_ms", 1, 1, true,
     "actively hold the current pose for args[0] milliseconds, driving back disturbance"},
    {"log", 0, 2, false, "write one Info line naming this marker and its args"},
};

/// How many built-in ids there are — `kBuiltinCommands`'s length, named so a manifest header and a
/// freeze pin can state it without re-deriving it.
inline constexpr std::size_t kBuiltinCommandCount = sizeof kBuiltinCommands / sizeof(BuiltinCommand);

/// The built-in whose id is exactly `id`, or nullptr when `id` is not one. Exact match only:
/// ids are compared whole, never by prefix (`StaticRoutes::route()`'s lesson).
[[nodiscard]] constexpr const BuiltinCommand* findBuiltinCommand(std::string_view id) noexcept {
    for (const BuiltinCommand& b : kBuiltinCommands) {
        if (std::string_view{b.id} == id) {
            return &b;
        }
    }
    return nullptr;
}

/// What the robot does to get to a waypoint. Every value maps onto a verb the facade has TODAY
/// (ruling B5); the numeric values are explicit and APPEND-ONLY, because a route may be generated
/// by one version of the tools and read by another.
enum class WaypointMotion : std::uint8_t {
    /// Not a motion: "the robot starts HERE". Legal only at index 0 (see Route::valid()); the
    /// runner seeds the pose from it rather than driving to it.
    Start = 1,
    /// `Chassis::moveTo(pose)` — translate and rotate simultaneously to the full pose. With
    /// `HeadingMode::Free` the commanded heading is the estimate's own, read when the leg starts
    /// (G2 ruling R6): "go there, keep facing where I am facing".
    MoveToPose = 2,
    /// `Routine::driveTo(x, y)` — move to the point ARRIVING FACING IT; `heading` is ignored and
    /// computed from the approach, which is why headingMode must be Free. With `reverse`, it is
    /// `Routine::reverseTo(x, y)`: the same point, arriving BACKWARDS.
    DriveTo = 3,
    /// `Chassis::turnTo(heading)` — rotate in place; `x` and `y` are ignored.
    TurnTo = 4,
    /// `Chassis::strafeTo(x, y)` — translate holding the current heading. Honest on tank: an
    /// off-line target exits TimedOut rather than pretending (C1's drivetrain honesty).
    StrafeTo = 5,
    /// `Chassis::hold(duration)` — actively hold the pose for `constraints.timeout`, which for
    /// this motion is the DURATION and must therefore be > 0.
    Hold = 6,
};

/// The motion as a word, for banners and refusal messages. "UNKNOWN" for a byte outside the enum.
[[nodiscard]] inline const char* waypointMotionWord(WaypointMotion m) noexcept {
    switch (m) {
        case WaypointMotion::Start: return "Start";
        case WaypointMotion::MoveToPose: return "MoveToPose";
        case WaypointMotion::DriveTo: return "DriveTo";
        case WaypointMotion::TurnTo: return "TurnTo";
        case WaypointMotion::StrafeTo: return "StrafeTo";
        case WaypointMotion::Hold: return "Hold";
    }
    return "UNKNOWN";
}

/// Whether the waypoint's `heading` field means anything.
enum class HeadingMode : std::uint8_t {
    /// The robot must arrive at `heading`. The only mode `TurnTo` accepts, and the mode
    /// `MoveToPose` takes when the author has a heading in mind.
    Fixed = 1,
    /// The heading is the RUNNER's to choose, and `heading` is not read. What it chooses depends
    /// on the verb, and each choice is the only honest one that verb has (G2 ruling R6):
    ///   * `DriveTo` — the approach bearing to (x, y), so the robot arrives facing the point.
    ///     This is the only mode `DriveTo` accepts.
    ///   * `MoveToPose` — whatever heading the estimate already has when the leg STARTS
    ///     ("translate, keep facing where I am facing"). `moveTo` always commands a heading, so
    ///     there is no "don't care" to pass down and the current one is the one that asks the
    ///     drivetrain to do the least.
    ///   * `StrafeTo`, `Hold`, `Start` — the runner reads neither mode; either is accepted. On a
    ///     `Start` a competition auton entry DOES read it (competition/auton_registry.hpp, since
    ///     COMP-a): a `Fixed` Start's `heading` is the heading the robot is seated to when the
    ///     auton starts, and a `Free` Start states none, so the entry must declare one.
    ///   * `TurnTo` — REFUSED: a turn to no heading is not a motion.
    Free = 2,
};

/// The heading mode as a word, for messages. "UNKNOWN" for a byte outside the enum.
[[nodiscard]] inline const char* headingModeWord(HeadingMode m) noexcept {
    switch (m) {
        case HeadingMode::Fixed: return "Fixed";
        case HeadingMode::Free: return "Free";
    }
    return "UNKNOWN";
}

/// Per-waypoint speed and time budgets. 0 means "use the robot's configured default" — the same
/// sentinel `chassis::MotionOptions` already uses, so a route's constraints map onto it field for
/// field with no translation table.
struct Constraints {
    /// Field-frame linear speed budget for this leg, in/s. Finite and >= 0; 0 = the default.
    units::Velocity maxLinear{0.0};
    /// Yaw-rate budget for this leg, rad/s. Finite and >= 0; 0 = the default.
    units::AngularVelocity maxAngular{0.0};
    /// Watchdog bound for this leg, seconds. Finite and >= 0; 0 = the default — EXCEPT on a
    /// `Hold`, where this is the hold DURATION and must be > 0 (there is no sensible default
    /// duration for "stay here", and `Chassis::hold` refuses a non-positive one anyway).
    units::Time timeout{0.0};
};

/// One thing to do at a waypoint, named by an ID the student's registry binds to an action.
///
/// The id is a fixed-width NUL-terminated buffer rather than a pointer, so a whole Route can be
/// written to a card or emitted by a generator as one object. The arguments are `std::int32_t`
/// and deliberately NOT a variant or a double: every parametric primitive anyone has asked for
/// takes a small integer (a lift level, a count, a side), and a typed integer is the one form
/// that survives serialization, `constexpr` emission and a human reading it in a diff.
struct Marker {
    /// The command id, e.g. "intake_in". Non-empty and `commandIdValid()` for any marker inside
    /// `Waypoint::markerCount`. An id G2's registry does not know logs a WARN and is SKIPPED
    /// (§16.3) — it is not a route validity error, because marker ids are an open vocabulary a
    /// team extends. An id outside the CHARSET is a different thing and IS refused: no registry
    /// can ever hold it, so it could only ever warn.
    char id[kMaxMarkerIdBytes]{};
    /// The typed arguments, e.g. `{3}` for "lift_to_level". Entries beyond `argCount` are 0 and
    /// are not read.
    std::int32_t args[kMaxMarkerArgs]{};
    /// How many entries of `args` are real. 0 is the common case.
    std::uint8_t argCount = 0;
};

/// A marker's id as text, bounded by the buffer (`commandIdView`'s note). This is the ONE spelling
/// the registry, the runner and the manifest all use, so no consumer can invent a different
/// reading of the same bytes.
[[nodiscard]] constexpr std::string_view markerIdView(const Marker& marker) noexcept {
    return commandIdView(marker.id, kMaxMarkerIdBytes);
}

/// One step of a route: where, how, how fast, and what to do there.
struct Waypoint {
    /// Field x, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.
    units::Length x{0.0};
    /// Field y, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.
    units::Length y{0.0};
    /// Field heading. Must be finite. Read only when `headingMode == Fixed` — on a `Start`, as
    /// the heading a competition auton is seated to (HeadingMode's note).
    math::Angle heading{};
    /// Whether `heading` means anything at this waypoint.
    HeadingMode headingMode = HeadingMode::Fixed;
    /// Which verb takes the robot here.
    WaypointMotion motion = WaypointMotion::MoveToPose;
    /// Back into this leg's point. Only a `DriveTo` has an approach to reverse (`valid()` refuses
    /// it on every other verb): the runner adds π to the bearing, so the robot arrives with its
    /// back to the point — exactly `Routine::reverseTo` (L9, API 2.4), pinned bit-identical to it.
    /// On a tank — or on any drive whose back must lead the whole leg, since a holonomic drive
    /// otherwise turns round in flight — an authored `TurnTo` that puts the robot's back to the
    /// point goes first, as `Routine::faceAway` does in a recipe. (Until L9 this comment called the
    /// flag advisory and read by no verb; the runner has read it since G2.) Carried, too, because
    /// the legacy planner's rows have it and a route that loses it on import has silently changed
    /// what a person drew.
    bool reverse = false;
    /// This leg's budgets.
    Constraints constraints{};
    /// What to fire here. Entries beyond `markerCount` are ignored entirely.
    Marker markers[kMaxMarkersPerWaypoint]{};
    /// How many entries of `markers` are real. Must be <= kMaxMarkersPerWaypoint.
    std::uint8_t markerCount = 0;
};

/// A whole autonomous routine, as one value.
///
/// Plain data with designated initializers at the call site. Like `RobotConfig`, it does not
/// validate itself on construction — a caller builds one and asks `valid()`, which is what lets a
/// test build a deliberately contradictory route and watch it be refused by name.
struct Route {
    /// The route's name — the key `IRouteSource::route()` looks up. Must be non-empty: a route
    /// nobody can name is a route nobody can run.
    char name[32]{};
    /// The steps, in order.
    Waypoint waypoints[kMaxWaypoints]{};
    /// How many entries of `waypoints` are real. Must be >= 1 and <= kMaxWaypoints.
    std::uint8_t count = 0;

    /// True when nothing in the route contradicts itself. `why` (optional; pass nullptr and 0 to
    /// skip) receives the FIRST contradiction found, naming the waypoint index and the field.
    ///
    /// The checks are exactly what a mis-drawn path, a bad import or a mis-generated file
    /// produces: an empty route name; no waypoints at all, or more than the capacity; a `Start`
    /// anywhere but index 0; an unknown motion byte or an unknown heading-mode byte; a NaN or an
    /// infinity in x, y or heading; a `TurnTo` whose headingMode is Free (a turn to no heading is
    /// not a motion) or a `DriveTo` whose headingMode is Fixed (the verb computes the heading, so
    /// a stated one would be silently ignored); a negative or non-finite budget; a `Hold` whose
    /// timeout is not > 0 (that field IS the hold duration); more markers than capacity; and an
    /// empty marker id.
    ///
    /// **Changed at G2 (ruling R6), the one RELAXATION on this list:** a `MoveToPose` with
    /// `HeadingMode::Free` used to be refused alongside `TurnTo`, on the reading that both verbs
    /// "need a heading". They do — but a `MoveToPose` has an honest one to fall back on that a
    /// `TurnTo` does not: the heading the robot already has. "Translate there, keep facing where
    /// I am facing" is a thing path authors ask for constantly and it had no spelling in the
    /// vocabulary; the runner supplies the estimate's heading at the leg's start. Relaxing a
    /// validity rule is the ADDITIVE direction (every route that was valid before is still
    /// valid), which is why it is a thing G2 may do at all and why doing it at the F8 freeze
    /// rather than after it costs nothing. `TurnTo` keeps the refusal, unchanged.
    ///
    /// **Added at G2**, once a runner existed to say what the data can actually ask for — the same
    /// principle, applied to four more ways a route can contradict itself:
    ///   * `reverse` on anything but a `DriveTo`. `DriveTo` is the only verb with an approach
    ///     direction to reverse (the runner drives to the point facing away from it); on every
    ///     other verb the flag is READ BY NOTHING, and a stated value that is silently ignored is
    ///     the quieter of the two bugs — exactly the reasoning that already refuses a `DriveTo`
    ///     with a `Fixed` heading.
    ///   * A marker id outside the command-id charset (`commandIdValid`): non-empty,
    ///     `[a-z0-9_]`, at most `kMaxMarkerIdBytes - 1` bytes. An unknown id is legal data; an
    ///     UNREGISTRABLE one is not, because `CommandRegistry` refuses it and the runner could
    ///     therefore only ever warn about it.
    ///   * A BUILT-IN id with the wrong number of args (`kBuiltinCommands`): `wait_ms` without its
    ///     duration is not a command the runner can guess a value for.
    ///   * A built-in whose `args[0]` is a duration and is not > 0: `Chassis::wait`/`hold` refuse
    ///     a non-positive duration at the door, so this is the `Hold`-timeout rule applied to the
    ///     two markers that carry the same quantity.
    ///
    /// Index 0 is NOT required to be a `Start`: a route that begins with a motion from wherever
    /// the robot already is, is a legitimate thing to author (it is what every hand-written auton
    /// in this repo's history does), and requiring a Start would force every importer to invent
    /// a pose it does not have.
    ///
    /// noexcept and total: it allocates nothing, throws nothing, and returns for every input.
    [[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept {
        if (why != nullptr && cap > 0) {
            why[0] = '\0';
        }
        if (name[0] == '\0') {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "route name is empty");
            }
            return false;
        }
        if (count == 0) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "route '%s' has no waypoints", name);
            }
            return false;
        }
        if (count > static_cast<std::uint8_t>(kMaxWaypoints)) {
            if (why != nullptr && cap > 0) {
                std::snprintf(why, cap, "route '%s' claims %d waypoints; the capacity is %d", name,
                              static_cast<int>(count), static_cast<int>(kMaxWaypoints));
            }
            return false;
        }
        for (std::size_t i = 0; i < count; ++i) {
            const Waypoint& w = waypoints[i];
            const int idx = static_cast<int>(i);
            switch (w.motion) {
                case WaypointMotion::Start:
                    if (i != 0) {
                        if (why != nullptr && cap > 0) {
                            std::snprintf(why, cap,
                                          "route '%s' waypoint %d is a Start; only waypoint 0 may "
                                          "be one",
                                          name, idx);
                        }
                        return false;
                    }
                    break;
                case WaypointMotion::MoveToPose:
                case WaypointMotion::DriveTo:
                case WaypointMotion::TurnTo:
                case WaypointMotion::StrafeTo:
                case WaypointMotion::Hold:
                    break;
                default:
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "route '%s' waypoint %d has motion byte %d, which is not a "
                                      "verb this library has",
                                      name, idx,
                                      static_cast<int>(static_cast<std::uint8_t>(w.motion)));
                    }
                    return false;
            }
            if (w.headingMode != HeadingMode::Fixed && w.headingMode != HeadingMode::Free) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d has headingMode byte %d, which is "
                                  "neither Fixed nor Free",
                                  name, idx,
                                  static_cast<int>(static_cast<std::uint8_t>(w.headingMode)));
                }
                return false;
            }
            if (!std::isfinite(w.x.value()) || !std::isfinite(w.y.value())
                || !std::isfinite(w.heading.radians())) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d pose (%g, %g, %g rad) must be finite",
                                  name, idx, w.x.value(), w.y.value(), w.heading.radians());
                }
                return false;
            }
            // The verb/heading-mode pairing. `TurnTo` servos TO a heading and does nothing else,
            // so Free leaves it with no target at all and no other content — there is no motion
            // left to perform. `DriveTo` COMPUTES the heading from the approach, so a stated one
            // would be accepted and then silently ignored — the quieter of the two bugs and
            // therefore the one worth refusing out loud. `MoveToPose` is the case in between and
            // is handled by ruling R6 below, not here.
            if (w.motion == WaypointMotion::TurnTo && w.headingMode != HeadingMode::Fixed) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d is a %s with headingMode %s; that verb "
                                  "needs a heading",
                                  name, idx, waypointMotionWord(w.motion),
                                  headingModeWord(w.headingMode));
                }
                return false;
            }
            if (w.motion == WaypointMotion::DriveTo && w.headingMode != HeadingMode::Free) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d is a DriveTo with headingMode %s; that "
                                  "verb computes the heading from the approach",
                                  name, idx, headingModeWord(w.headingMode));
                }
                return false;
            }
            const double lin = w.constraints.maxLinear.value();
            const double ang = w.constraints.maxAngular.value();
            const double t = w.constraints.timeout.value();
            if (!std::isfinite(lin) || lin < 0.0 || !std::isfinite(ang) || ang < 0.0
                || !std::isfinite(t) || t < 0.0) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d constraints (%g in/s, %g rad/s, %g s) "
                                  "must be finite and >= 0 (0 = the robot's default)",
                                  name, idx, lin, ang, t);
                }
                return false;
            }
            // `reverse` is DriveTo's alone (the G2 note above). Refusing it everywhere else is the
            // wide rule on purpose: relaxing a validity rule later is additive, tightening one is
            // breaking, so the honest width is the one to start at.
            if (w.reverse && w.motion != WaypointMotion::DriveTo) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d is a %s with reverse set; only a "
                                  "DriveTo has an approach direction to reverse",
                                  name, idx, waypointMotionWord(w.motion));
                }
                return false;
            }
            if (w.motion == WaypointMotion::Hold && !(t > 0.0)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d is a Hold with timeout %g; for a Hold "
                                  "that field IS the duration and must be > 0",
                                  name, idx, t);
                }
                return false;
            }
            if (w.markerCount > static_cast<std::uint8_t>(kMaxMarkersPerWaypoint)) {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "route '%s' waypoint %d claims %d markers; the capacity is %d",
                                  name, idx, static_cast<int>(w.markerCount),
                                  static_cast<int>(kMaxMarkersPerWaypoint));
                }
                return false;
            }
            for (std::size_t m = 0; m < w.markerCount; ++m) {
                if (w.markers[m].id[0] == '\0') {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap, "route '%s' waypoint %d marker %d has an empty id",
                                      name, idx, static_cast<int>(m));
                    }
                    return false;
                }
                if (w.markers[m].argCount > static_cast<std::uint8_t>(kMaxMarkerArgs)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "route '%s' waypoint %d marker '%s' claims %d args; the "
                                      "capacity is %d",
                                      name, idx, w.markers[m].id,
                                      static_cast<int>(w.markers[m].argCount),
                                      static_cast<int>(kMaxMarkerArgs));
                    }
                    return false;
                }
                // ── the G2 rules (see the member comment) ──────────────────────────────
                const std::string_view id = markerIdView(w.markers[m]);
                if (!commandIdValid(id)) {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "route '%s' waypoint %d marker %d has id '%.*s', which is "
                                      "not a legal command id (1..%d bytes of [a-z0-9_])",
                                      name, idx, static_cast<int>(m), static_cast<int>(id.size()),
                                      id.data(), static_cast<int>(kMaxMarkerIdBytes - 1));
                    }
                    return false;
                }
                const BuiltinCommand* const builtin = findBuiltinCommand(id);
                if (builtin != nullptr) {
                    const std::uint8_t n = w.markers[m].argCount;
                    if (n < builtin->minArgs || n > builtin->maxArgs) {
                        if (why != nullptr && cap > 0) {
                            std::snprintf(why, cap,
                                          "route '%s' waypoint %d marker '%s' carries %d args; "
                                          "that built-in id takes %d..%d",
                                          name, idx, builtin->id, static_cast<int>(n),
                                          static_cast<int>(builtin->minArgs),
                                          static_cast<int>(builtin->maxArgs));
                        }
                        return false;
                    }
                    if (builtin->firstArgIsDurationMs && !(w.markers[m].args[0] > 0)) {
                        if (why != nullptr && cap > 0) {
                            std::snprintf(why, cap,
                                          "route '%s' waypoint %d marker '%s' has args[0] = %d ms; "
                                          "that argument is a DURATION and must be > 0",
                                          name, idx, builtin->id,
                                          static_cast<int>(w.markers[m].args[0]));
                        }
                        return false;
                    }
                }
            }
        }
        return true;
    }
};

/// The route seam (master plan decision #10): where a robot's routines come from. A generated
/// header, a card read at boot and an array typed by hand all look the same through it.
class IRouteSource {
public:
    /// Polymorphic-base boilerplate: a virtual destructor so a source held as `IRouteSource&`/`*`
    /// destroys correctly, with the copy/move set re-defaulted because declaring the destructor
    /// suppresses them. The base carries no state.
    virtual ~IRouteSource() = default;
    /// Default-constructible: the base has nothing to initialize.
    IRouteSource() = default;
    /// Copyable: the base holds no state, so a derived type decides what copying means.
    IRouteSource(const IRouteSource&) = default;
    /// Movable, same reasoning as the copy constructor.
    IRouteSource(IRouteSource&&) = default;
    /// Copy-assignable, same reasoning.
    IRouteSource& operator=(const IRouteSource&) = default;
    /// Move-assignable, same reasoning.
    IRouteSource& operator=(IRouteSource&&) = default;

    /// The route called `name`, or NULLPTR when this source has no such route. A null return is
    /// the whole error channel on purpose: "the route you asked for is not here" is a normal thing
    /// for a program to ask (an auton selector offers what exists), not an exception.
    [[nodiscard]] virtual const Route* route(std::string_view name) const noexcept = 0;

    /// How many routes this source carries — for an auton selector that lists them.
    [[nodiscard]] virtual std::size_t count() const noexcept = 0;

    /// The route at `index`, which must be < count(). Preconditioned, not bounds-returned: an
    /// out-of-range index here is a programming error, unlike a missing NAME.
    [[nodiscard]] virtual const Route& at(std::size_t index) const = 0;
};

/// An `IRouteSource` over a caller's array of routes — the implementation a generated header or a
/// hand-typed table uses.
///
/// NON-OWNING: it holds a `std::span`, so the array must outlive the source. That is the opposite
/// choice from `StaticRobotConfig` (which copies) and the reason is size: one config is ~400
/// bytes and one route is ~5 KB, so copying a table of them into a second buffer would double the
/// largest static object on a brain with 128 KB of user RAM, to protect against a lifetime bug
/// that a `constexpr` table at namespace scope — the only way either producer emits one — cannot
/// have.
class StaticRoutes final : public IRouteSource {
public:
    /// View `routes`. The array must outlive this source. An empty span is legal: a robot with no
    /// routes yet is a robot, and `route()` will answer nullptr for everything.
    explicit StaticRoutes(std::span<const Route> routes) noexcept : routes_{routes} {}

    /// The route whose `name` matches exactly, or nullptr. Order is preserved and the FIRST match
    /// wins — duplicate names are not refused here, because a source is a view of someone else's
    /// table and refusing at lookup time would hide the duplicate rather than report it.
    [[nodiscard]] const Route* route(std::string_view name) const noexcept override {
        for (const Route& r : routes_) {
            // The stored name is a fixed buffer that may not be NUL-terminated if a generator
            // filled all 32 bytes, so the comparison is bounded by the BUFFER, not by strlen.
            // Bounded by hand rather than with strnlen(), which is POSIX and not guaranteed by
            // the freestanding-ish newlib the V5 cross-compiler links against.
            std::size_t n = 0;
            while (n < sizeof r.name && r.name[n] != '\0') {
                ++n;
            }
            if (std::string_view{r.name, n} == name) {
                return &r;
            }
        }
        return nullptr;
    }

    /// How many routes this source views.
    [[nodiscard]] std::size_t count() const noexcept override { return routes_.size(); }

    /// The route at `index`, in the caller's array order. Precondition: index < count().
    [[nodiscard]] const Route& at(std::size_t index) const override {
        SHULIB_PRECONDITION(index < routes_.size(), "StaticRoutes::at: index out of range");
        return routes_[index];
    }

private:
    std::span<const Route> routes_;
};

}  // namespace shulib::config
