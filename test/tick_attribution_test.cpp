// Tests for diag/tick_attribution.hpp — D-3, tick-time attribution. What each targets:
//  * ACCOUNTING EXACTNESS: attributed + other must equal the tick total on the same
//    clock — attribution that doesn't sum is attribution nobody can trust.
//  * THE NAME: the whole point is turning "the loop is slow" into a subsystem name;
//    lastWorstPhase must finger the right one.
//  * LAG DISCIPLINE: consumers read the LAST COMPLETED tick; an open tick must never
//    leak half-measured numbers.
//  * EXCEPTION SAFETY: a throw through a tick abandons it — the instrument re-arms
//    instead of wedging (or reporting a tick that never finished).

#include "doctest.h"

#include <type_traits>

#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/tick_attribution.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::TickAttribution;
using shulib::diag::TickPhase;
using shulib::diag::tickPhaseName;
using shulib::hal::fake::FakeClock;
using shulib::units::Time;

// Bug caught: phase durations mis-bracketed (crediting the wrong span) or the sum
// contract broken — the exact-arithmetic case, advance-by-hand on a fake clock.
TEST_CASE("D-3: attributed phases sum with 'other' to exactly the measured total") {
    FakeClock clock;
    TickAttribution att{clock};

    att.beginTick();
    clock.advance(Time{0.001});  // 1 ms pre-phase overhead → "other"
    {
        const auto scope = att.phase(TickPhase::Localization);
        clock.advance(Time{0.004});  // 4 ms of localization
    }
    clock.advance(Time{0.002});  // 2 ms between phases → "other"
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.002});  // 2 ms of motion
    }
    clock.advance(Time{0.001});  // 1 ms post-phase → "other"
    att.endTick();

    REQUIRE(att.hasCompletedTick());
    CHECK(att.lastPhases()[0].value() == doctest::Approx(0.004));
    CHECK(att.lastPhases()[1].value() == doctest::Approx(0.002));
    CHECK(att.lastAttributed().value() == doctest::Approx(0.006));
    CHECK(att.lastTotal().value() == doctest::Approx(0.010));
    CHECK(att.lastOther().value() == doctest::Approx(0.004));
    // attributed + other == total, exactly the accounting identity D-3 promises.
    CHECK(att.lastAttributed().value() + att.lastOther().value()
          == doctest::Approx(att.lastTotal().value()));
}

// Bug caught: the deliberately slow subsystem NOT being the one named — the
// requirement in one line. Also pins deterministic tie-breaking.
TEST_CASE("D-3: a deliberately slow subsystem is correctly named; ties resolve low") {
    FakeClock clock;
    TickAttribution att{clock};

    att.beginTick();
    {
        const auto scope = att.phase(TickPhase::Localization);
        clock.advance(Time{0.001});
    }
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.007});  // the hog
    }
    att.endTick();
    CHECK(att.lastWorstPhase() == TickPhase::Motion);
    CHECK(tickPhaseName(att.lastWorstPhase()) == std::string_view{"mot"});

    att.beginTick();  // exact tie → lower index (deterministic, pinned)
    {
        const auto scope = att.phase(TickPhase::Localization);
        clock.advance(Time{0.003});
    }
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.003});
    }
    att.endTick();
    CHECK(att.lastWorstPhase() == TickPhase::Localization);
}

// Bug caught: repeated phases within one tick overwriting instead of accumulating
// (a motion phase entered twice would report only its second half).
TEST_CASE("D-3: a phase entered twice in one tick accumulates") {
    FakeClock clock;
    TickAttribution att{clock};
    att.beginTick();
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.002});
    }
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.003});
    }
    att.endTick();
    CHECK(att.lastPhases()[1].value() == doctest::Approx(0.005));
}

// Bug caught: an open tick leaking half-measured numbers into consumers (records
// are stamped mid-tick — they must carry the last COMPLETED story, unchanged).
TEST_CASE("D-3: the last completed tick is stable while the next tick is open") {
    FakeClock clock;
    TickAttribution att{clock};
    att.beginTick();
    {
        const auto scope = att.phase(TickPhase::Localization);
        clock.advance(Time{0.004});
    }
    att.endTick();

    att.beginTick();  // next tick opens…
    {
        const auto scope = att.phase(TickPhase::Localization);
        clock.advance(Time{0.009});  // …and is much slower
    }
    // Mid-tick consumers still see the completed 4 ms story, not 9.
    CHECK(att.lastPhases()[0].value() == doctest::Approx(0.004));
    att.endTick();
    CHECK(att.lastPhases()[0].value() == doctest::Approx(0.009));
}

// Bug caught: an exception path wedging the instrument (next beginTick blows) or
// an abandoned half-tick being REPORTED as if it completed.
TEST_CASE("D-3: abandonTick discards the half-measured tick, keeps the last completed "
          "one, and re-arms") {
    FakeClock clock;
    TickAttribution att{clock};
    att.beginTick();
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.002});
    }
    att.endTick();

    att.beginTick();
    clock.advance(Time{0.030});  // a horrible half-tick that will "throw"
    att.abandonTick();
    CHECK(att.lastPhases()[1].value() == doctest::Approx(0.002));  // story unchanged

    att.beginTick();  // re-armed: no precondition trip
    att.endTick();
    CHECK(att.hasCompletedTick());
}

// Bug caught: bracket misuse compiling into nonsense numbers instead of failing
// loudly (phase timing outside a tick has no total to reconcile against).
TEST_CASE("D-3: bracket misuse is loud — phase/endTick need an open tick; double "
          "begin is caught") {
    FakeClock clock;
    TickAttribution att{clock};
    CHECK_THROWS_AS((void)att.phase(TickPhase::Motion), PreconditionError);
    CHECK_THROWS_AS(att.endTick(), PreconditionError);
    att.beginTick();
    CHECK_THROWS_AS(att.beginTick(), PreconditionError);
    att.endTick();

    att.reset();
    CHECK_FALSE(att.hasCompletedTick());
    CHECK(att.lastTotal().value() == 0.0);
}

// Bug caught: the display vocabulary drifting from the wire vocabulary (an
// overrun line naming the wrong subsystem via a stale switch).
TEST_CASE("D-3: tickPhaseName covers every defined phase; spares render as reserved") {
    CHECK(tickPhaseName(TickPhase::Localization) == std::string_view{"loc"});
    CHECK(tickPhaseName(TickPhase::Motion) == std::string_view{"mot"});
    CHECK(tickPhaseName(TickPhase::Health) == std::string_view{"hlt"});
    CHECK(tickPhaseName(TickPhase::Telemetry) == std::string_view{"tel"});
    CHECK(tickPhaseName(TickPhase::Scheduler) == std::string_view{"sch"});
    CHECK(tickPhaseName(TickPhase::User) == std::string_view{"usr"});
    CHECK(tickPhaseName(static_cast<TickPhase>(6)) == std::string_view{"rsv"});
    CHECK(tickPhaseName(static_cast<TickPhase>(7)) == std::string_view{"rsv"});
}

// Bug caught (DEFECTS1 item A8): PhaseScope's constructor was public, so phase()'s tick-open
// precondition was advisory — `TickAttribution::PhaseScope s{att, p};` compiled with no tick
// open and its destructor still wrote into current_, crediting the interval to whatever tick
// happened to be open when it finally closed. The constructor now needs a passkey only
// TickAttribution can produce, so the checked factories are the only way in. Compile-time
// facts, asserted as such: a runtime test cannot check that something does NOT compile.
TEST_CASE("A8: a phase scope cannot be opened around the tick-open check") {
    static_assert(!std::is_constructible_v<TickAttribution::PhaseScope, TickAttribution&,
                                           TickPhase>,
                  "PhaseScope must not be constructible without a passkey");
    static_assert(!std::is_default_constructible_v<TickAttribution::PhaseScope::Key>,
                  "the passkey must not be forgeable by a caller");

    // And the checked path enforces what the bypass skipped, at run time.
    FakeClock clock;
    TickAttribution att{clock};
    CHECK_THROWS_AS((void)att.phase(TickPhase::Motion), PreconditionError);
    CHECK_THROWS_AS((void)att.phaseInPlace(TickPhase::Motion), PreconditionError);

    // NEGATIVE CONTROL: with a tick open, both factories work and both charge the phase.
    att.beginTick();
    {
        const auto scope = att.phase(TickPhase::Motion);
        clock.advance(Time{0.004});
    }
    {
        const auto scope = att.phaseInPlace(TickPhase::Localization);
        clock.advance(Time{0.002});
    }
    att.endTick();
    const auto& phases = att.lastPhases();
    CHECK(phases[static_cast<std::size_t>(TickPhase::Motion)].value()
          == doctest::Approx(0.004));
    CHECK(phases[static_cast<std::size_t>(TickPhase::Localization)].value()
          == doctest::Approx(0.002));
}
