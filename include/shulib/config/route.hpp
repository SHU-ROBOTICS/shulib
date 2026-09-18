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

/// What the robot does to get to a waypoint. Every value maps onto a verb the facade has TODAY
/// (ruling B5); the numeric values are explicit and APPEND-ONLY, because a route may be generated
/// by one version of the tools and read by another.
enum class WaypointMotion : std::uint8_t {
    /// Not a motion: "the robot starts HERE". Legal only at index 0 (see Route::valid()); the
    /// runner seeds the pose from it rather than driving to it.
    Start = 1,
    /// `Chassis::moveTo(pose)` — translate and rotate simultaneously to the full pose.
    MoveToPose = 2,
    /// `Routine::driveTo(x, y)` — move to the point ARRIVING FACING IT; `heading` is ignored and
    /// computed from the approach, which is why headingMode must be Free.
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
    /// The robot must arrive at `heading`. The only mode `MoveToPose` and `TurnTo` accept.
    Fixed = 1,
    /// The heading is the runner's to choose (the approach bearing, or whatever the robot
    /// already has). The only mode `DriveTo` accepts; `heading` is not read.
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
    /// The command id, e.g. "intake_in". Non-empty for any marker inside `Waypoint::markerCount`.
    /// An id G2's registry does not know logs a WARN and is SKIPPED (§16.3) — it is not a route
    /// validity error, because marker ids are an open vocabulary a team extends.
    char id[24]{};
    /// The typed arguments, e.g. `{3}` for "lift_to_level". Entries beyond `argCount` are 0 and
    /// are not read.
    std::int32_t args[kMaxMarkerArgs]{};
    /// How many entries of `args` are real. 0 is the common case.
    std::uint8_t argCount = 0;
};

/// One step of a route: where, how, how fast, and what to do there.
struct Waypoint {
    /// Field x, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.
    units::Length x{0.0};
    /// Field y, inches. Must be finite. Not read when `motion` is `TurnTo` or `Hold`.
    units::Length y{0.0};
    /// Field heading. Must be finite. Read only when `headingMode == Fixed`.
    math::Angle heading{};
    /// Whether `heading` means anything at this waypoint.
    HeadingMode headingMode = HeadingMode::Fixed;
    /// Which verb takes the robot here.
    WaypointMotion motion = WaypointMotion::MoveToPose;
    /// Drive this leg backwards where the verb supports it. Advisory data for G2's runner; the
    /// library's verbs are holonomic and do not read it today, and it is carried rather than
    /// dropped because the legacy planner's rows have it and a route that loses it on import has
    /// silently changed what a person drew.
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
    /// infinity in x, y or heading; a `MoveToPose` or `TurnTo` whose headingMode is Free (the
    /// verb needs a heading and Free says there is not one) or a `DriveTo` whose headingMode is
    /// Fixed (the verb computes the heading, so a stated one would be silently ignored); a
    /// negative or non-finite budget; a `Hold` whose timeout is not > 0 (that field IS the hold
    /// duration); more markers than capacity; and an empty marker id.
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
            // The verb/heading-mode pairing. MoveToPose and TurnTo servo TO a heading, so Free
            // leaves them without a target; DriveTo COMPUTES the heading from the approach, so a
            // stated one would be accepted and then silently ignored — the quieter of the two
            // bugs and therefore the one worth refusing out loud.
            if ((w.motion == WaypointMotion::MoveToPose || w.motion == WaypointMotion::TurnTo)
                && w.headingMode != HeadingMode::Fixed) {
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
