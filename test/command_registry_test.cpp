// runner::CommandRegistry — the id → action table, and the adaptation it shares with
// Routine::then() (chunk G2, brief §4 tests 1 and 2).
//
// The two things worth testing here are the two things that go wrong in a table like this:
//   * the FENCE. Registration is code, so every way a registration can be wrong is a
//     precondition — a duplicate, a built-in re-bound, an id outside the vocabulary, a null
//     function, a table already full. A fence with a hole is not a fence, so each one is
//     asserted separately and each is the SOLE cause of its case.
//   * the ADAPTATION. Two layers now map a user callable's return value onto "did it work?" —
//     Routine::then() and this. If they ever disagree, the same lambda means one thing in a
//     hand-written recipe and another in a data-driven route, which is the worst kind of bug to
//     debug because both halves look right on their own. Test 2 runs the SAME callables through
//     both and requires the verdicts to agree.
//
// No hardware claim: the Routine arm of test 2 drives the A2 host plant.

#include "doctest.h"

#include <cstdio>
#include <cstring>
#include <string>
#include <string_view>

#include "motion_test_rig.hpp"
#include "shulib/chassis/routine.hpp"
#include "shulib/config/route.hpp"
#include "shulib/core/check.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/manipulation/mechanism_outcome.hpp"
#include "shulib/runner/command_registry.hpp"

namespace cfgns = shulib::config;
using shulib::PreconditionError;
using shulib::chassis::Routine;
using shulib::manipulation::MechanismOutcome;
using shulib::runner::CommandRegistry;
using shulib::runner::Handler;
using shulib::runner::kMaxCommands;

namespace {

/// A marker with just an id — what most handlers are called with.
cfgns::Marker marker(const char* id) {
    cfgns::Marker m;
    std::strncpy(m.id, id, sizeof m.id - 1);
    return m;
}

/// A marker carrying one typed argument.
cfgns::Marker marker(const char* id, std::int32_t a0) {
    cfgns::Marker m = marker(id);
    m.args[0] = a0;
    m.argCount = 1;
    return m;
}

/// A free function with the Handler signature, for the explicit-Handler overload's cases.
MechanismOutcome noop(void*, const cfgns::Marker&) { return MechanismOutcome::Succeeded; }

/// Run a registered id and return its outcome. REQUIREs the id to be registered, so a missing
/// registration is a red test rather than a null dereference.
MechanismOutcome fire(const CommandRegistry& r, const cfgns::Marker& m) {
    const Handler* h = r.find(cfgns::markerIdView(m));
    REQUIRE(h != nullptr);
    return h->fn(h->ctx, m);
}

}  // namespace

// ═══ 1. The table's rules ════════════════════════════════════════════════════════════
// Bug caught: any of the five registration fences being absent. Each subcase breaks exactly one
// thing about an otherwise-legal registration.
TEST_CASE("G2 registry 1: on/find/count/ids, and every way a registration is refused") {
    int intakeCalls = 0;
    auto intakeIn = [&] { ++intakeCalls; };
    auto liftTo = [&](const cfgns::Marker& m) { intakeCalls += m.args[0]; };

    CommandRegistry r;
    CHECK(r.count() == 0);
    CHECK(r.ids().empty());
    CHECK(r.find("intake_in") == nullptr);

    r.on("intake_in", intakeIn);
    r.on("lift_to_level", liftTo, 1);

    CHECK(r.count() == 2);
    REQUIRE(r.ids().size() == 2);
    // Registration ORDER, not sorted order — the manifest publishes this sequence.
    CHECK(r.ids()[0].idView() == "intake_in");
    CHECK(r.ids()[1].idView() == "lift_to_level");
    CHECK(r.ids()[0].argCount == 0);
    CHECK(r.ids()[1].argCount == 1);
    REQUIRE(r.find("intake_in") != nullptr);
    REQUIRE(r.find("lift_to_level") != nullptr);

    // Lookup is EXACT: not a prefix, not a superstring, not case-insensitive (StaticRoutes'
    // lesson, which is the same lesson).
    CHECK(r.find("intake") == nullptr);
    CHECK(r.find("intake_in_2") == nullptr);
    CHECK(r.find("INTAKE_IN") == nullptr);
    CHECK(r.find("") == nullptr);

    // The handlers really are bound to the right callables, and the marker reaches the one that
    // asked for it.
    CHECK(fire(r, marker("intake_in")) == MechanismOutcome::Succeeded);
    CHECK(intakeCalls == 1);
    CHECK(fire(r, marker("lift_to_level", 3)) == MechanismOutcome::Succeeded);
    CHECK(intakeCalls == 4);

    SUBCASE("a duplicate id is a typo, not a merge") {
        CHECK_THROWS_AS(r.on("intake_in", intakeIn), PreconditionError);
        // …including through the explicit-Handler overload, which is a different code path to
        // the same fence.
        CHECK_THROWS_AS(r.on("intake_in", Handler{&noop, nullptr}), PreconditionError);
        CHECK(r.count() == 2);  // and nothing was half-written
    }

    SUBCASE("a built-in id is library-owned and cannot be re-bound (ruling R3)") {
        for (const cfgns::BuiltinCommand& b : cfgns::kBuiltinCommands) {
            CAPTURE(b.id);
            CHECK_THROWS_AS(r.on(b.id, intakeIn), PreconditionError);
        }
        CHECK(r.count() == 2);
    }

    SUBCASE("an id outside the vocabulary is refused — charset, length, emptiness") {
        CHECK_THROWS_AS(r.on("", intakeIn), PreconditionError);
        CHECK_THROWS_AS(r.on("Intake_In", intakeIn), PreconditionError);   // uppercase
        CHECK_THROWS_AS(r.on("intake in", intakeIn), PreconditionError);   // space
        CHECK_THROWS_AS(r.on("intake-in", intakeIn), PreconditionError);   // hyphen
        CHECK_THROWS_AS(r.on("intake.in", intakeIn), PreconditionError);   // dot
        CHECK_THROWS_AS(r.on("intake/in", intakeIn), PreconditionError);   // slash
        CHECK_THROWS_AS(r.on("\303\251clair", intakeIn), PreconditionError);  // non-ASCII

        // The length bound is the BUFFER minus its terminator: 23 fits, 24 does not.
        const std::string longest(cfgns::kMaxMarkerIdBytes - 1, 'a');
        r.on(longest, intakeIn);
        CHECK(r.find(longest) != nullptr);
        CHECK_THROWS_AS(r.on(longest + "a", intakeIn), PreconditionError);
        // …and a 23-byte id round-trips through the fixed buffer WITH its terminator, which is
        // the whole reason the bound is 23 and not 24.
        CHECK(r.ids()[2].idView() == longest);
    }

    SUBCASE("a null function pointer would crash at fire time, so it is refused at bind time") {
        CHECK_THROWS_AS(r.on("clamp", Handler{nullptr, nullptr}), PreconditionError);
    }

    SUBCASE("an argCount past the marker's capacity is refused") {
        CHECK_THROWS_AS(r.on("clamp", liftTo, static_cast<std::uint8_t>(cfgns::kMaxMarkerArgs + 1)),
                        PreconditionError);
    }

    SUBCASE("capacity: exactly kMaxCommands fit, and the next is refused") {
        CommandRegistry full;
        char id[cfgns::kMaxMarkerIdBytes];
        for (std::size_t i = 0; i < kMaxCommands; ++i) {
            std::snprintf(id, sizeof id, "cmd_%02zu", i);
            full.on(id, intakeIn);
        }
        CHECK(full.count() == kMaxCommands);
        CHECK(full.ids().size() == kMaxCommands);
        CHECK(full.find("cmd_00") != nullptr);
        CHECK(full.find("cmd_31") != nullptr);
        CHECK_THROWS_AS(full.on("one_too_many", intakeIn), PreconditionError);
        CHECK(full.count() == kMaxCommands);
    }
}

// ═══ 2. The adaptation, pinned against Routine::then() ═══════════════════════════════
// Bug caught: the registry and the recipe layer disagreeing about what a callable's return value
// MEANS. The same lambda must succeed in both or fail in both; anything else makes "does my
// handler work?" depend on which tier called it.
//
// The pin is behavioural, not structural: for each callable, "the registry's outcome is
// Succeeded" must equal "Routine::then() left the chain ok()". Both are computed here, from the
// same lambda object, on a live chassis.
TEST_CASE("G2 registry 2: void/bool/MechanismOutcome adapt exactly as Routine::then() does") {
    const shulib::kinematics::MatrixKinematics kin =
        shulib::kinematics::xDrive(shulib::units::Length{7.0});

    int calls = 0;
    auto voidFn = [&] { ++calls; };
    auto trueFn = [&] { ++calls; return true; };
    auto falseFn = [&] { ++calls; return false; };

    SUBCASE("void always succeeds; true succeeds; false fails") {
        CommandRegistry r;
        r.on("v", voidFn);
        r.on("t", trueFn);
        r.on("f", falseFn);

        CHECK(fire(r, marker("v")) == MechanismOutcome::Succeeded);
        CHECK(fire(r, marker("t")) == MechanismOutcome::Succeeded);
        // false maps to Unconfirmed — "the handler ran and the task did not happen" — never to
        // TimedOut or Stalled, which would claim a pathology the bool never stated.
        CHECK(fire(r, marker("f")) == MechanismOutcome::Unconfirmed);
        CHECK(calls == 3);

        // The same three callables through Routine::then(), on a live chassis.
        motion_rig::ChassisRig cr{kin};
        for (int k = 0; k < 3; ++k) {
            Routine routine{cr.chassis, "adaptation"};
            CommandRegistry one;
            switch (k) {
                case 0: routine.then(voidFn); one.on("x", voidFn); break;
                case 1: routine.then(trueFn); one.on("x", trueFn); break;
                default: routine.then(falseFn); one.on("x", falseFn); break;
            }
            CAPTURE(k);
            const bool registrySucceeded = fire(one, marker("x")) == MechanismOutcome::Succeeded;
            CHECK(registrySucceeded == routine.ok());
        }
    }

    SUBCASE("a MechanismOutcome passes through unchanged, and agrees with then() on all six") {
        motion_rig::ChassisRig cr{kin};
        const MechanismOutcome all[] = {
            MechanismOutcome::Running,  MechanismOutcome::Succeeded, MechanismOutcome::Unconfirmed,
            MechanismOutcome::TimedOut, MechanismOutcome::Cancelled, MechanismOutcome::Stalled};
        for (const MechanismOutcome o : all) {
            CAPTURE(std::string{shulib::manipulation::mechanismOutcomeName(o)});
            auto fn = [o] { return o; };
            CommandRegistry r;
            r.on("x", fn);
            CHECK(fire(r, marker("x")) == o);  // passed through, not re-meant

            Routine routine{cr.chassis, "outcomes"};
            routine.then(fn);
            CHECK((o == MechanismOutcome::Succeeded) == routine.ok());
            CHECK((fire(r, marker("x")) == MechanismOutcome::Succeeded) == routine.ok());
        }
    }

    SUBCASE("a handler may take the marker, and reads its own id and args") {
        std::string sawId;
        std::int32_t sawArg = 0;
        auto withMarker = [&](const cfgns::Marker& m) {
            sawId = std::string{cfgns::markerIdView(m)};
            sawArg = m.args[0];
            return MechanismOutcome::Succeeded;
        };
        CommandRegistry r;
        r.on("lift_to_level", withMarker, 1);
        CHECK(fire(r, marker("lift_to_level", 7)) == MechanismOutcome::Succeeded);
        CHECK(sawId == "lift_to_level");
        CHECK(sawArg == 7);
    }

    SUBCASE("the stored pointer is NON-OWNING: the caller's object is the one that runs") {
        int owned = 0;
        auto counter = [&owned] { ++owned; };
        CommandRegistry r;
        r.on("count", counter);
        (void)fire(r, marker("count"));
        (void)fire(r, marker("count"));
        CHECK(owned == 2);  // the caller's `owned`, through the caller's lambda — no copy ran
    }
}
