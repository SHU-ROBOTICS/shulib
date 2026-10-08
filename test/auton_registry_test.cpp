// competition::AutonRegistry — registration refuses, BY NAME, every entry that could not run
// (chunk COMP-a; the COMP brief's rulings C3, C6 and amendments B15, B16, B25). Every case names
// the bug it would catch.
//
// A refusal here is the difference between a program that says "auton 'left rush': hardStopAt
// 35.00 s is past its MATCH period of 30.00 s" at boot, in the pits, and one that discovers it on
// the field. So each case checks the sentence names the auton and the problem, not merely that
// something threw.

#include "doctest.h"

#include <cmath>
#include <cstdio>
#include <string>

#include "shulib/chassis/routine.hpp"
#include "shulib/competition/acceptance_run.hpp"
#include "shulib/competition/auton_registry.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::chassis::RoutineResult;
using shulib::competition::AutonContext;
using shulib::competition::AutonEntry;
using shulib::competition::AutonKind;
using shulib::competition::AutonPeriods;
using shulib::competition::AutonRegistry;
using shulib::competition::AutonVerdict;
using shulib::competition::checkAutonEntry;
using shulib::competition::kMaxAutonNameChars;
using shulib::competition::kMaxAutons;
using shulib::competition::StartPose;
using shulib::config::Route;
using shulib::config::Waypoint;
using shulib::config::WaypointMotion;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

namespace {

RoutineResult noop(AutonContext& /*ctx*/) { return RoutineResult{}; }

/// The program's two periods (the COMP brief's C6: match 30 s, skills 60 s).
AutonPeriods periods() { return AutonPeriods{.match = Time{30.0}, .skills = Time{60.0}}; }

/// A valid MATCH routine entry, to be broken one field at a time.
AutonEntry good(const char* name = "left rush") {
    AutonEntry e{};
    e.name = name;
    e.kind = AutonKind::Match;
    e.routine = &noop;
    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(90.0));
    e.endActionAt = Time{26.0};
    e.hardStopAt = Time{29.0};
    return e;
}

/// A two-waypoint route, with or without a Start at index 0.
Route route(bool withStart, Pose2d start = Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(90.0)}) {
    Route r{};
    std::snprintf(r.name, sizeof r.name, "%s", "r1");
    if (withStart) {
        r.waypoints[0].motion = WaypointMotion::Start;
        r.waypoints[0].x = start.x();
        r.waypoints[0].y = start.y();
        r.waypoints[0].heading = start.heading();
        r.waypoints[1].motion = WaypointMotion::TurnTo;
        r.waypoints[1].heading = Angle::degrees(0.0);
        r.count = 2;
    } else {
        r.waypoints[0].motion = WaypointMotion::TurnTo;
        r.waypoints[0].heading = Angle::degrees(0.0);
        r.count = 1;
    }
    return r;
}

bool says(const AutonVerdict& v, const std::string& needle) {
    return std::string{v.why}.find(needle) != std::string::npos;
}

}  // namespace

// Bug caught: a registry that accepts a good entry only by accident — the baseline every refusal
// below breaks one field of.
TEST_CASE("B15 a complete entry registers; find/at/size see it; the sentence is empty") {
    AutonRegistry reg{periods()};
    const AutonVerdict v = reg.check(good());
    CHECK(v.ok);
    CHECK(std::string{v.why}.empty());
    reg.add(good());
    CHECK(reg.size() == 1);
    REQUIRE(reg.find("left rush") != nullptr);
    CHECK(reg.find("left rush") == &reg.at(0));
    CHECK(reg.find("right rush") == nullptr);
    CHECK(reg.periodFor(reg.at(0)).value() == 30.0);
}

// Bug caught: the library inventing a match length. The periods are the PROGRAM's constants; a
// registry built without them is a loud error, not a 15-second default.
TEST_CASE("C6 the periods are required: an unstated one refuses the registry itself") {
    CHECK_THROWS_AS(AutonRegistry{AutonPeriods{}}, PreconditionError);
    CHECK_THROWS_AS((AutonRegistry{AutonPeriods{.match = Time{30.0}}}), PreconditionError);
    CHECK_THROWS_AS((AutonRegistry{AutonPeriods{.match = Time{30.0}, .skills = Time{-1.0}}}),
                    PreconditionError);
}

// Bug caught: an unset or nonsensical schedule reaching a match (RunGuard::run would throw there,
// on the field), and a hard stop past the period — the field disables the robot first, so the
// guarantee the schedule promises could never be kept (C6). Each refusal names the auton.
TEST_CASE("B15 the schedule: unset, nonsense and past-the-period are refused at registration") {
    AutonRegistry reg{periods()};
    SUBCASE("unset") {
        AutonEntry e = good();
        e.endActionAt = Time{0.0};
        e.hardStopAt = Time{0.0};
        const AutonVerdict v = reg.check(e);
        CHECK_FALSE(v.ok);
        CHECK(says(v, "auton 'left rush'"));
        CHECK(says(v, "UNSET"));
    }
    SUBCASE("hardStopAt before endActionAt") {
        AutonEntry e = good();
        e.hardStopAt = Time{20.0};
        CHECK(says(reg.check(e), "hardStopAt must be finite and >= endActionAt"));
    }
    SUBCASE("non-finite") {
        AutonEntry e = good();
        e.endActionAt = Time{std::nan("")};
        CHECK_FALSE(reg.check(e).ok);
    }
    SUBCASE("past the MATCH period") {
        AutonEntry e = good();
        e.hardStopAt = Time{35.0};
        const AutonVerdict v = reg.check(e);
        CHECK_FALSE(v.ok);
        CHECK(says(v, "past its MATCH period of 30.00 s"));
    }
    SUBCASE("the same schedule is legal for SKILLS (its period is 60 s)") {
        AutonEntry e = good();
        e.kind = AutonKind::Skills;
        e.endActionAt = Time{50.0};
        e.hardStopAt = Time{35.0 + 20.0};
        CHECK(reg.check(e).ok);
    }
    SUBCASE("exactly ON the period is accepted (the guide recommends a margin, the rule is <=)") {
        AutonEntry e = good();
        e.hardStopAt = Time{30.0};
        CHECK(reg.check(e).ok);
    }
    SUBCASE("add() throws the SAME sentence") {
        AutonEntry e = good();
        e.hardStopAt = Time{35.0};
        try {
            reg.add(e);
            FAIL("add() accepted a hard stop past the period");
        } catch (const PreconditionError& err) {
            CHECK(std::string{err.what()}.find("auton 'left rush'") != std::string::npos);
        }
        CHECK(reg.empty());
    }
}

// Bug caught: a KIND left at its zero value quietly running under one of the two periods.
TEST_CASE("B15 an entry must say MATCH or SKILLS") {
    AutonRegistry reg{periods()};
    AutonEntry e = good();
    e.kind = AutonKind::Unset;
    CHECK(says(reg.check(e), "kind is not set"));
    e.kind = static_cast<AutonKind>(7);
    CHECK_FALSE(reg.check(e).ok);
}

// Bug caught: names the screen cannot show (B25 — "AUTON: <name>" must be one medium line, ASCII,
// with no '%' that PROS would re-read as a format), duplicates the selector could not tell apart,
// and "NONE", which is the selector's own choice.
TEST_CASE("B25 names: 1..29 printable ASCII, no percent, unique, never NONE") {
    AutonRegistry reg{periods()};
    const std::string longest(kMaxAutonNameChars, 'a');
    const std::string tooLong(kMaxAutonNameChars + 1, 'a');
    CHECK(reg.check(good(longest.c_str())).ok);
    CHECK_FALSE(reg.check(good(tooLong.c_str())).ok);
    CHECK_FALSE(reg.check(good("")).ok);
    CHECK_FALSE(reg.check(good(nullptr)).ok);
    CHECK_FALSE(reg.check(good("50% rush")).ok);
    CHECK_FALSE(reg.check(good("tab\there")).ok);
    CHECK_FALSE(reg.check(good("caf\xc3\xa9")).ok);  // UTF-8: the V5 fonts are ASCII only
    CHECK(says(reg.check(good("NONE")), "NONE is the selector's own choice"));
    // The sentence never carries a bad name back to the screen.
    CHECK_FALSE(says(reg.check(good("50% rush")), "50%"));
    reg.add(good("left rush"));
    const AutonVerdict dup = reg.check(good("left rush"));
    CHECK_FALSE(dup.ok);
    CHECK(says(dup, "already registered"));
}

// Bug caught: an entry that is both, or neither — the runner would have to guess which to run.
TEST_CASE("C3 exactly one of routine and route") {
    AutonRegistry reg{periods()};
    const Route r = route(true);
    AutonEntry both = good();
    both.route = &r;
    CHECK(says(reg.check(both), "BOTH"));
    AutonEntry neither = good();
    neither.routine = nullptr;
    CHECK(says(reg.check(neither), "neither"));
}

// Bug caught: a routine with no declared start — the runner would re-seat to an invented (0, 0, 0)
// field pose (C2: each auton DECLARES field or relative).
TEST_CASE("C2 a routine entry must declare its start (FIELD or RELATIVE)") {
    AutonRegistry reg{periods()};
    AutonEntry e = good();
    e.start = StartPose{};
    CHECK(says(reg.check(e), "no start pose"));
    e.start = StartPose::relative();
    CHECK(reg.check(e).ok);
    e.start = StartPose::field(Length{std::nan("")}, Length{0.0}, Angle{});
    CHECK(says(reg.check(e), "not finite"));
}

// Bug caught (B16): a route entry whose declared start DISAGREES with its route's Start waypoint —
// the runner would re-seat to one pose and the route's Start leg would setPose to the other a tick
// later. Unset takes the route's; equal is fine; different (or relative) is refused. A route with
// no Start takes the entry's, which must then be set. And an invalid route is refused with the
// route's own sentence.
TEST_CASE("B16 route entries: the route's Start IS the start pose") {
    AutonRegistry reg{periods()};
    const Route withStart = route(true);
    const Route noStart = route(false);
    AutonEntry e = good();
    e.routine = nullptr;
    e.route = &withStart;

    e.start = StartPose{};
    CHECK(reg.check(e).ok);  // takes the route's
    CHECK(e.effectiveStart().mode() == StartPose::Mode::Field);
    CHECK(e.effectiveStart().pose().x().value() == -48.0);
    CHECK(e.effectiveStart().pose().heading().approxEqual(Angle::degrees(90.0)));

    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(90.0));
    CHECK(reg.check(e).ok);  // the same pose, stated twice

    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(0.0));
    CHECK(says(reg.check(e), "different from its route's Start"));
    e.start = StartPose::relative();
    CHECK(says(reg.check(e), "different from its route's Start"));

    e.route = &noStart;
    e.start = StartPose{};
    CHECK(says(reg.check(e), "no Start waypoint"));
    e.start = StartPose::relative();
    CHECK(reg.check(e).ok);

    Route broken = route(true);
    broken.count = 0;
    e.route = &broken;
    CHECK(says(reg.check(e), "its route is invalid: route 'r1' has no waypoints"));
}

// Bug caught (the COMP-a review's F8 consistency finding): a Start waypoint marked FREE — "the
// heading is not read", in route.hpp's own words — whose `heading` field (a default 0, or any
// leftover number) is seated as the robot's start heading anyway. Built so the leftover number is
// NOT what the robot faces: the Start says Free with heading 0 while the entry, where it states
// one, says 90. Unset is refused with a sentence that says what to do; a FIELD start at the
// Start's (x, y) supplies the heading and is taken (any heading — the Start states none); a
// FIELD start somewhere else, or RELATIVE, is still refused as different.
TEST_CASE("B16 a Free Start states no heading: the entry must, or it is refused") {
    AutonRegistry reg{periods()};
    Route freeStart = route(true, Pose2d{Length{-48.0}, Length{-24.0}, Angle::degrees(0.0)});
    freeStart.waypoints[0].headingMode = shulib::config::HeadingMode::Free;
    REQUIRE(freeStart.valid());  // F8's Route::valid accepts either mode on a Start — unchanged
    AutonEntry e = good();
    e.routine = nullptr;
    e.route = &freeStart;

    e.start = StartPose{};
    CHECK_FALSE(reg.check(e).ok);
    CHECK(says(reg.check(e), "auton 'left rush': its route's Start states no heading (Free)"));
    CHECK_FALSE(e.effectiveStart().isSet());  // the seat never reads the Free Start's heading

    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(90.0));
    CHECK(reg.check(e).ok);
    CHECK(e.effectiveStart().pose().heading().approxEqual(Angle::degrees(90.0), 1e-12));
    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(-135.0));
    CHECK(reg.check(e).ok);  // any heading: the Start states none to disagree with

    e.start = StartPose::field(Length{-47.0}, Length{-24.0}, Angle::degrees(90.0));
    CHECK(says(reg.check(e), "different from its route's Start"));
    e.start = StartPose::relative();
    CHECK(says(reg.check(e), "different from its route's Start"));

    // And the same Start marked Fixed states its heading (0 here): the entry's 90 now DISAGREES.
    freeStart.waypoints[0].headingMode = shulib::config::HeadingMode::Fixed;
    e.start = StartPose::field(Length{-48.0}, Length{-24.0}, Angle::degrees(90.0));
    CHECK(says(reg.check(e), "different from its route's Start"));
    e.start = StartPose{};
    CHECK(reg.check(e).ok);
    CHECK(e.effectiveStart().pose().heading().approxEqual(Angle::degrees(0.0), 1e-12));
}

// Bug caught: a registry that silently drops the seventeenth auton, or overwrites the first.
TEST_CASE("C3 capacity: the registry refuses past kMaxAutons, by name") {
    AutonRegistry reg{periods()};
    char names[kMaxAutons + 1][8]{};
    for (std::size_t i = 0; i < kMaxAutons; ++i) {
        std::snprintf(names[i], sizeof names[i], "a%d", static_cast<int>(i));
        reg.add(good(names[i]));
    }
    std::snprintf(names[kMaxAutons], sizeof names[kMaxAutons], "extra");
    const AutonVerdict v = reg.check(good(names[kMaxAutons]));
    CHECK_FALSE(v.ok);
    CHECK(says(v, "auton 'extra': the registry is full"));
    CHECK_THROWS_AS(reg.add(good(names[kMaxAutons])), PreconditionError);
    CHECK(reg.size() == kMaxAutons);
    CHECK(std::string{reg.at(0).name} == "a0");
}

// Bug caught: the library's own test routine failing the registry's rules (it is registered from
// the students' file like any entry — B18).
TEST_CASE("B18 the acceptance run's entry registers, RELATIVE, with the schedule it is given") {
    AutonRegistry reg{periods()};
    const AutonEntry e =
        shulib::competition::acceptanceRunEntry(AutonKind::Match, Time{25.0}, Time{28.0});
    CHECK(e.start.mode() == StartPose::Mode::Relative);
    CHECK(reg.check(e).ok);
    reg.add(e);
    CHECK(reg.find(shulib::competition::kAcceptanceRunName) != nullptr);
    // …and its schedule is the caller's: none is invented for it.
    const AutonEntry unset =
        shulib::competition::acceptanceRunEntry(AutonKind::Match, Time{0.0}, Time{0.0});
    CHECK(says(checkAutonEntry(unset, Time{30.0}), "UNSET"));
}

// ── OnRefusal::DropEntry (T4 Part 3, the team lead's ruling of 2026-10-02 on COMP-b's C5: "drop
//    just that auton") ──────────────────────────────────────────────────────────────────────────

namespace {
/// A students'-file-shaped registration: three add() calls in a row, the FIRST one refused (its
/// hard stop is past the 30 s match period). The real file (src/team/autons.cpp) has the same
/// shape: a list of add() calls, nothing between them that could catch a throw.
void registerThree(AutonRegistry& reg) {
    AutonEntry late = good("late stop");
    late.hardStopAt = Time{35.0};  // past the MATCH period: check() refuses it
    reg.add(late);
    reg.add(good("left rush"));
    reg.add(good("right rush"));
}
}  // namespace

// Bug caught (C5): one refused entry costing EVERY auton — and, in a program, the boot and its
// driver control. Under DropEntry the refused entry is registered as if it had never been written:
// add() returns, the NEXT entries register in order, the dropped one is findable by nobody, and its
// sentence is kept, naming it. The default is unchanged: the same registration THROWS, by name, with
// nothing half-registered — every caller that did not ask for a drop keeps B15.
TEST_CASE("C5 DropEntry: a refused entry is dropped, named and counted; the rest register") {
    AutonRegistry drop{periods(), shulib::competition::OnRefusal::DropEntry};
    CHECK(drop.onRefusal() == shulib::competition::OnRefusal::DropEntry);
    CHECK_NOTHROW(registerThree(drop));
    CHECK(drop.size() == 2);
    CHECK(std::string{drop.at(0).name} == "left rush");
    CHECK(std::string{drop.at(1).name} == "right rush");
    CHECK(drop.find("late stop") == nullptr);
    CHECK(drop.refusedCount() == 1);
    CHECK(std::string{drop.refusal(0)}.rfind("auton 'late stop': ", 0) == 0);  // names it FIRST
    AutonEntry late = good("late stop");
    late.hardStopAt = Time{35.0};
    CHECK(std::string{drop.refusal(0)} == std::string{drop.check(late).why});  // check()'s own words
    CHECK(says(drop.check(late), "hardStopAt"));
    CHECK(std::string{drop.refusal(1)}.empty());   // past the count: "", never a stale sentence
    CHECK(std::string{drop.refusal(99)}.empty());

    AutonRegistry strict{periods()};  // the default: B15, unchanged
    CHECK(strict.onRefusal() == shulib::competition::OnRefusal::Throw);
    bool threw = false;
    try {
        registerThree(strict);
    } catch (const PreconditionError& e) {
        threw = true;
        CHECK(std::string{e.what()}.find("auton 'late stop'") != std::string::npos);
    }
    CHECK(threw);
    CHECK(strict.size() == 0);
    CHECK(strict.refusedCount() == 0);  // a throw is not a drop
}

// Bug caught: a drop that keeps the WRONG sentence (another entry's, or a stale one) or loses the
// order, a drop that still takes a capacity slot, and a registry that stops counting past the
// sentences it keeps (the program says how many more there were).
TEST_CASE("C5 DropEntry: each dropped sentence in order; a drop takes no slot; counted past the cap") {
    AutonRegistry reg{periods(), shulib::competition::OnRefusal::DropEntry};
    AutonEntry noKind = good("no kind");
    noKind.kind = AutonKind{};  // neither MATCH nor SKILLS
    reg.add(noKind);
    reg.add(good("left rush"));
    reg.add(good("left rush"));  // a duplicate name: dropped too
    AutonEntry unset = good("unset");
    unset.endActionAt = Time{0.0};
    unset.hardStopAt = Time{0.0};
    reg.add(unset);
    CHECK(reg.size() == 1);
    REQUIRE(reg.refusedCount() == 3);
    CHECK(std::string{reg.refusal(0)}.rfind("auton 'no kind': ", 0) == 0);
    CHECK(std::string{reg.refusal(1)}.find("already registered") != std::string::npos);
    CHECK(std::string{reg.refusal(2)}.rfind("auton 'unset': ", 0) == 0);
    // A drop takes no slot: kMaxAutons - 1 more good entries still fit.
    char names[kMaxAutons][8]{};
    for (std::size_t i = 0; i + 1 < kMaxAutons; ++i) {
        std::snprintf(names[i], sizeof names[i], "g%d", static_cast<int>(i));
        reg.add(good(names[i]));
    }
    CHECK(reg.size() == kMaxAutons);
    // Past the kept sentences: still counted, the kept ones untouched.
    AutonRegistry many{periods(), shulib::competition::OnRefusal::DropEntry};
    for (std::size_t i = 0; i < shulib::competition::kMaxAutonRefusals + 3; ++i) {
        AutonEntry bad = good("same");
        bad.hardStopAt = Time{99.0};
        many.add(bad);
    }
    CHECK(many.size() == 0);
    CHECK(many.refusedCount() == shulib::competition::kMaxAutonRefusals + 3);
    CHECK(std::string{many.refusal(shulib::competition::kMaxAutonRefusals - 1)}.rfind("auton 'same'", 0)
          == 0);
    CHECK(std::string{many.refusal(shulib::competition::kMaxAutonRefusals)}.empty());
    // The cap by ADDRESS, not by content (the T4-3 review's RV1: without the cap half of the guard,
    // refusal(kMaxAutonRefusals) read one slot past the array -- the next member's bytes, which
    // happened to start with a zero, so the emptiness check above passed on undefined behaviour).
    // Past the cap the answer is the guard's own "" -- the same pointer an index past the count
    // gets -- and never a pointer into the registry.
    const char* pastCap = many.refusal(shulib::competition::kMaxAutonRefusals);
    const char* pastCount = many.refusal(99);
    CHECK(pastCap == pastCount);
    const char* begin = reinterpret_cast<const char*>(&many);
    CHECK((pastCap < begin || pastCap >= begin + sizeof many));
}
