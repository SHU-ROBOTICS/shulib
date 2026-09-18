// config::Route — an auton as data, and the rules that say when the data is lying
// (chunk G1 Part 1, brief §4 tests 2 and 3).
//
// Nothing executes a Route yet (G2's PathRunner does). What is under test here is the SHAPE and
// its validity rules, and the reason that is worth a test file of its own before the runner
// exists is in docs/legacy-command-vocabulary.md §1: this team has authored three parallel data
// representations of a routine and executed none of them. The format outlives the runner. A
// format that accepts a route it cannot mean — a Start in the middle, a Hold with no duration, a
// marker with no id — hands the runner a decision the runner has no information to make, and
// "warn and skip" at RUN time is a much worse place to discover it than "refused" at build time.
//
// Sole-cause discipline, as in robot_config_test.cpp: one valid route, one thing broken per case.

#include "doctest.h"

#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>
#include <type_traits>

#include "shulib/config/route.hpp"

namespace cfgns = shulib::config;
using shulib::math::Angle;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

/// A valid four-step route: start, a pose move that fires a marker, a free-heading drive, and a
/// hold with a duration. Every case below starts from this and breaks exactly one thing.
cfgns::Route baseRoute() {
    return cfgns::Route{
        .name = "left_side_rush",
        .waypoints =
            {
                {.x = Length{-48.0},
                 .y = Length{-24.0},
                 .heading = Angle::degrees(90.0),
                 .headingMode = cfgns::HeadingMode::Fixed,
                 .motion = cfgns::WaypointMotion::Start},
                {.x = Length{-24.0},
                 .y = Length{0.0},
                 .heading = Angle::degrees(45.0),
                 .headingMode = cfgns::HeadingMode::Fixed,
                 .motion = cfgns::WaypointMotion::MoveToPose,
                 .constraints = {.maxLinear = Velocity{30.0}, .timeout = Time{3.0}},
                 .markers = {{.id = "intake_in"},
                             {.id = "lift_to_level", .args = {3}, .argCount = 1}},
                 .markerCount = 2},
                {.x = Length{0.0},
                 .y = Length{24.0},
                 .headingMode = cfgns::HeadingMode::Free,
                 .motion = cfgns::WaypointMotion::DriveTo,
                 .reverse = true},
                {.headingMode = cfgns::HeadingMode::Free,
                 .motion = cfgns::WaypointMotion::Hold,
                 .constraints = {.timeout = Time{0.5}}},
            },
        .count = 4};
}

/// valid() with a message buffer, as a std::string so the checks can say `contains`.
std::string whyNot(const cfgns::Route& r) {
    char why[192] = "";
    const bool ok = r.valid(why, sizeof why);
    return ok ? std::string{} : std::string{why};
}

}  // namespace

// ═══ 2a. The reference route is valid, and constexpr-constructible ═══════════════════
// The constexpr pin is not decoration: G3's codegen emits `inline constexpr Route kRoutes[]`,
// and a field that quietly stopped being a literal type would break the generator long after
// this header was last read.
TEST_CASE("G1 route 2a: the reference route is valid and leaves no stale reason") {
    char why[192] = "sentinel";
    CHECK(baseRoute().valid(why, sizeof why));
    CHECK(std::strlen(why) == 0);
    CHECK(baseRoute().valid());
}

namespace {
/// The generator's spelling, compiled: a whole route as a constant expression.
inline constexpr cfgns::Route kGenerated{
    .name = "generated",
    .waypoints = {{.x = Length{12.0},
                   .y = Length{0.0},
                   .heading = Angle{},
                   .motion = cfgns::WaypointMotion::MoveToPose,
                   .markers = {{.id = "score", .args = {1, 2}, .argCount = 2}},
                   .markerCount = 1}},
    .count = 1};
static_assert(kGenerated.count == 1, "a Route must be usable as a constant expression");
static_assert(kGenerated.waypoints[0].markers[0].args[1] == 2);
}  // namespace

// ═══ 2b. A Start anywhere but index 0 ════════════════════════════════════════════════
// Bug caught: a route that re-seeds the pose mid-run. The runner would teleport the estimate to
// a waypoint the robot has not reached — every later leg then drives from a lie, and nothing in
// the telemetry says so, because a setPose() is a legitimate operation that leaves no fault.
TEST_CASE("G1 route 2b: a Start is legal only at index 0") {
    cfgns::Route r = baseRoute();
    r.waypoints[2].motion = cfgns::WaypointMotion::Start;
    const std::string why = whyNot(r);
    CHECK(why.find("waypoint 2") != std::string::npos);
    CHECK(why.find("Start") != std::string::npos);

    // …and index 0 is NOT REQUIRED to be a Start: a route that begins from wherever the robot
    // already is, is a legitimate thing to author (and is what every hand-written auton in this
    // repo's history has done).
    cfgns::Route noStart = baseRoute();
    noStart.waypoints[0] = noStart.waypoints[1];
    CHECK(noStart.valid());
}

// ═══ 2c. A Hold with no duration ═════════════════════════════════════════════════════
// Bug caught: `timeout = 0` meaning "use the default" everywhere EXCEPT on a Hold, where there
// is no sensible default for "stay here" — Chassis::hold() refuses a non-positive duration at
// the door, so a route carrying one is a routine that throws mid-auton instead of one that is
// refused before it runs.
TEST_CASE("G1 route 2c: a Hold's timeout IS its duration and must be > 0") {
    cfgns::Route r = baseRoute();
    r.waypoints[3].constraints.timeout = Time{0.0};
    const std::string why = whyNot(r);
    CHECK(why.find("waypoint 3") != std::string::npos);
    CHECK(why.find("Hold") != std::string::npos);
    CHECK(why.find("duration") != std::string::npos);

    // Every OTHER motion is happy with 0 — that is the "use the robot's default" sentinel.
    cfgns::Route ok = baseRoute();
    ok.waypoints[1].constraints.timeout = Time{0.0};
    CHECK(ok.valid());
}

// ═══ 2d. An empty marker id ══════════════════════════════════════════════════════════
// Bug caught: a marker that cannot be bound to anything. G2 warns and SKIPS an id its registry
// does not know — which is right for an id a team has not registered YET, and wrong for an id
// that does not exist at all: the second is a data defect, and skipping it silently is how a
// missing intake command becomes "the auton just did not score".
TEST_CASE("G1 route 2d: a marker with no id is refused; an UNRECOGNIZED id is not this type's "
          "business") {
    cfgns::Route r = baseRoute();
    r.waypoints[1].markers[1].id[0] = '\0';
    const std::string why = whyNot(r);
    CHECK(why.find("waypoint 1") != std::string::npos);
    CHECK(why.find("marker 1") != std::string::npos);
    CHECK(why.find("empty id") != std::string::npos);

    // An id nobody has ever registered is still VALID DATA: marker ids are an open vocabulary a
    // team extends, and the library does not hold the list. (Ruling B5's other half: an unknown
    // MOTION is invalid, because the motion set IS the library's own contract.)
    cfgns::Route exotic = baseRoute();
    std::strncpy(exotic.waypoints[1].markers[0].id, "wobble_the_thing",
                 sizeof exotic.waypoints[1].markers[0].id - 1);
    CHECK(exotic.valid());

    // Markers BEYOND the count are ignored entirely, empty ids and all.
    cfgns::Route beyond = baseRoute();
    beyond.waypoints[1].markerCount = 1;
    beyond.waypoints[1].markers[1].id[0] = '\0';
    CHECK(beyond.valid());
}

// ═══ 2e. NaN anywhere in a pose ══════════════════════════════════════════════════════
// Bug caught: a NaN reaching Chassis::moveTo, whose precondition throws — mid-routine, after the
// earlier legs have already driven. followTrajectory hoists that check to before the first leg
// for exactly this reason; a Route must not be able to carry one at all.
TEST_CASE("G1 route 2e: a non-finite x, y or heading is refused, naming the waypoint") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    SUBCASE("x") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].x = Length{nan};
        CHECK(whyNot(r).find("waypoint 1") != std::string::npos);
    }
    SUBCASE("y") {
        cfgns::Route r = baseRoute();
        r.waypoints[2].y = Length{inf};
        CHECK(whyNot(r).find("waypoint 2") != std::string::npos);
    }
    SUBCASE("heading — reached the way a CARD reaches it, because Angle itself refuses one") {
        // math::Angle::radians() rejects a non-finite input, so a NaN heading cannot be TYPED.
        // It can still ARRIVE: a Route is laid out to be written to an SD card byte for byte
        // (R3d Part 2 already does exactly that with a DrivetrainDescription), and a flipped bit
        // or a truncated read lands a non-finite double in the storage without ever passing
        // through a constructor. That is the path this check exists for, so that is the path the
        // test takes — Angle is a single trivially-copyable double, which is what makes both the
        // card and this bit_cast possible.
        cfgns::Route r = baseRoute();
        static_assert(std::is_trivially_copyable_v<Angle>);
        static_assert(sizeof(Angle) == sizeof(double));
        r.waypoints[1].heading = std::bit_cast<Angle>(nan);
        REQUIRE_FALSE(std::isfinite(r.waypoints[1].heading.radians()));
        const std::string why = whyNot(r);
        CHECK(why.find("waypoint 1") != std::string::npos);
        CHECK(why.find("finite") != std::string::npos);
    }
    SUBCASE("a constraint") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].constraints.maxAngular = AngularVelocity{nan};
        CHECK(whyNot(r).find("constraints") != std::string::npos);
    }
    SUBCASE("a NEGATIVE constraint — finite, and still nonsense") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].constraints.maxLinear = Velocity{-5.0};
        CHECK(whyNot(r).find("constraints") != std::string::npos);
    }
}

// ═══ 2f. Capacities ══════════════════════════════════════════════════════════════════
TEST_CASE("G1 route 2f: counts past capacity, and the empty route") {
    SUBCASE("more markers than capacity") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].markerCount = 5;
        const std::string why = whyNot(r);
        CHECK(why.find("claims 5 markers") != std::string::npos);
    }
    SUBCASE("more args than capacity") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].markers[1].argCount = 3;
        CHECK(whyNot(r).find("claims 3 args") != std::string::npos);
    }
    SUBCASE("more waypoints than capacity") {
        cfgns::Route r = baseRoute();
        r.count = 33;
        CHECK(whyNot(r).find("claims 33 waypoints") != std::string::npos);
    }
    SUBCASE("a route with no waypoints at all") {
        cfgns::Route r = baseRoute();
        r.count = 0;
        CHECK(whyNot(r).find("no waypoints") != std::string::npos);
    }
    SUBCASE("a route with no name — nothing could ever look it up") {
        cfgns::Route r = baseRoute();
        r.name[0] = '\0';
        CHECK(whyNot(r).find("name is empty") != std::string::npos);
    }
    SUBCASE("exactly at capacity is fine") {
        cfgns::Route r{.name = "full"};
        for (std::size_t i = 0; i < cfgns::kMaxWaypoints; ++i) {
            r.waypoints[i] = {.x = Length{static_cast<double>(i)},
                              .y = Length{0.0},
                              .heading = Angle{},
                              .motion = cfgns::WaypointMotion::MoveToPose};
        }
        r.count = static_cast<std::uint8_t>(cfgns::kMaxWaypoints);
        CHECK(r.valid());
    }
}

// ═══ 2g. The verb vocabulary is the library's own, and a byte outside it is invalid ══
// Ruling B5: a waypoint that asks for a verb this library does not have is a REQUEST FOR A
// LIBRARY FEATURE, and the honest answer is a refusal, not a silent skip. This is the case that
// distinguishes an unknown MOTION (invalid) from an unknown MARKER (fine — see 2d).
TEST_CASE("G1 route 2g: a motion byte this library has no verb for is refused") {
    cfgns::Route r = baseRoute();
    r.waypoints[2].motion = static_cast<cfgns::WaypointMotion>(99);
    const std::string why = whyNot(r);
    CHECK(why.find("waypoint 2") != std::string::npos);
    CHECK(why.find("motion byte 99") != std::string::npos);

    cfgns::Route h = baseRoute();
    h.waypoints[1].headingMode = static_cast<cfgns::HeadingMode>(4);
    CHECK(whyNot(h).find("headingMode byte 4") != std::string::npos);

    // Every minted verb is accepted in its own right.
    const cfgns::WaypointMotion verbs[] = {
        cfgns::WaypointMotion::MoveToPose, cfgns::WaypointMotion::DriveTo,
        cfgns::WaypointMotion::TurnTo,     cfgns::WaypointMotion::StrafeTo,
        cfgns::WaypointMotion::Hold};
    for (const cfgns::WaypointMotion v : verbs) {
        CAPTURE(cfgns::waypointMotionWord(v));
        cfgns::Route one{.name = "one"};
        one.waypoints[0] = {.x = Length{10.0},
                            .y = Length{2.0},
                            .heading = Angle{},
                            .headingMode = v == cfgns::WaypointMotion::DriveTo
                                               ? cfgns::HeadingMode::Free
                                               : cfgns::HeadingMode::Fixed,
                            .motion = v,
                            .constraints = {.timeout = Time{1.0}}};
        one.count = 1;
        CHECK(one.valid());
    }
}

// ═══ 2h. The verb/heading-mode pairing ═══════════════════════════════════════════════
// Bug caught: a stated heading that is silently ignored (DriveTo computes its own from the
// approach — C1's honesty about tank verbs never planning turns), and a heading-servoing verb
// with no heading to servo to. The first is the quieter bug and therefore the one worth naming.
TEST_CASE("G1 route 2h: a verb that needs a heading refuses Free; DriveTo refuses Fixed") {
    SUBCASE("MoveToPose with Free") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].headingMode = cfgns::HeadingMode::Free;
        const std::string why = whyNot(r);
        CHECK(why.find("MoveToPose") != std::string::npos);
        CHECK(why.find("needs a heading") != std::string::npos);
    }
    SUBCASE("TurnTo with Free") {
        cfgns::Route r = baseRoute();
        r.waypoints[1].motion = cfgns::WaypointMotion::TurnTo;
        r.waypoints[1].headingMode = cfgns::HeadingMode::Free;
        CHECK(whyNot(r).find("TurnTo") != std::string::npos);
    }
    SUBCASE("DriveTo with Fixed") {
        cfgns::Route r = baseRoute();
        r.waypoints[2].headingMode = cfgns::HeadingMode::Fixed;
        const std::string why = whyNot(r);
        CHECK(why.find("DriveTo") != std::string::npos);
        CHECK(why.find("computes the heading") != std::string::npos);
    }
    SUBCASE("StrafeTo and Hold accept either — neither reads the heading field") {
        for (const cfgns::HeadingMode mode :
             {cfgns::HeadingMode::Fixed, cfgns::HeadingMode::Free}) {
            cfgns::Route r = baseRoute();
            r.waypoints[1].motion = cfgns::WaypointMotion::StrafeTo;
            r.waypoints[1].headingMode = mode;
            r.waypoints[3].headingMode = mode;
            CHECK(r.valid());
        }
    }
}

// ═══ 3. StaticRoutes: the seam ═══════════════════════════════════════════════════════
// Bug caught: a lookup that matches a PREFIX (strncmp over the buffer would say "left" finds
// "left_side_rush"), and an `at()` that silently returns garbage rather than tripping.
TEST_CASE("G1 route 3: StaticRoutes looks a route up by name, answers nullptr for a missing one, "
          "and preserves order") {
    const cfgns::Route table[] = {baseRoute(), cfgns::Route{.name = "right_side",
                                                            .waypoints = {{.x = Length{0.0},
                                                                           .y = Length{0.0},
                                                                           .heading = Angle{},
                                                                           .motion =
                                                                               cfgns::WaypointMotion::
                                                                                   MoveToPose}},
                                                            .count = 1}};
    const cfgns::StaticRoutes routes{table};
    const cfgns::IRouteSource& seam = routes;

    CHECK(seam.count() == 2);
    REQUIRE(seam.route("left_side_rush") != nullptr);
    CHECK(seam.route("left_side_rush") == &table[0]);
    REQUIRE(seam.route("right_side") != nullptr);
    CHECK(seam.route("right_side") == &table[1]);

    // Missing, and the near-misses that a sloppy comparison would accept.
    CHECK(seam.route("nope") == nullptr);
    CHECK(seam.route("left") == nullptr);              // a prefix is not a name
    CHECK(seam.route("left_side_rush_2") == nullptr);  // nor is a superstring
    CHECK(seam.route("") == nullptr);
    CHECK(seam.route("LEFT_SIDE_RUSH") == nullptr);    // case matters

    // Order is the caller's array order — an auton selector lists them in the order they were
    // authored, not in whatever order a lookup happened to visit.
    CHECK(&seam.at(0) == &table[0]);
    CHECK(&seam.at(1) == &table[1]);
    CHECK_THROWS_AS((void)seam.at(2), shulib::PreconditionError);

    // An empty source is legal: a robot with no routes yet is still a robot.
    const cfgns::StaticRoutes empty{std::span<const cfgns::Route>{}};
    CHECK(empty.count() == 0);
    CHECK(empty.route("anything") == nullptr);
}

// ═══ 3b. A name that fills the whole buffer ══════════════════════════════════════════
// Bug caught: a lookup that runs off the end of a name a generator filled to the last byte. The
// stored name is a fixed buffer with no guaranteed NUL, and `std::string_view{r.name}` would
// read past it.
TEST_CASE("G1 route 3b: a 32-byte name with no NUL is compared by the BUFFER, not by strlen") {
    cfgns::Route full = baseRoute();
    std::memset(full.name, 'a', sizeof full.name);  // no terminator anywhere
    const cfgns::Route table[] = {full};
    const cfgns::StaticRoutes routes{table};
    const std::string exact(sizeof full.name, 'a');
    CHECK(routes.route(exact) == &table[0]);
    CHECK(routes.route(exact + "a") == nullptr);
    CHECK(routes.route(std::string(sizeof full.name - 1, 'a')) == nullptr);
    // valid() is happy with it: a full buffer is a name, not a defect.
    CHECK(full.valid());
}
