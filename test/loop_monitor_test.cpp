// Tests for diag/loop_monitor.hpp — loop-overrun detection (§18.4). What each targets:
//  * THE BOUNDARY: overrun is dt >= budget, INCLUSIVE. The dt == budget case is pinned
//    with float-exact values (0.25/0.5 are exact binaries) so the test can distinguish
//    >= from > — a loosened comparison ('>') must turn the edge case red.
//  * BASELINE SEMANTICS: the first tick (and the first after reset()) can never fault,
//    or a run boundary / deliberate pause would masquerade as a timing pathology.
//  * The fault goes through the LATCH (cascade counted, first retained) with a
//    structured, pinned message — not prose.

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/loop_monitor.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::LoopMonitor;
using shulib::diag::LoopMonitorConfig;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::units::Time;

namespace {
struct Rig {
    FakeTelemetrySink sink;
    FakeClock clock;
    FaultLatch latch{sink, clock};
};
}  // namespace

TEST_CASE("LoopMonitor: the first tick only baselines — it can never fault") {
    Rig rig;
    rig.clock.set(Time{1000.0});  // a huge absolute time must not read as a huge dt
    LoopMonitor mon{rig.clock, rig.latch, LoopMonitorConfig{Time{0.5}}};

    const Time dt = mon.tick();
    CHECK(dt.value() == doctest::Approx(0.0));
    CHECK_FALSE(rig.latch.hasFault());
    CHECK(mon.overrunCount() == 0);
    CHECK(mon.worstDt().value() == doctest::Approx(0.0));  // baseline doesn't count
}

TEST_CASE("LoopMonitor: healthy cadence below budget never faults; worstDt tracks the max") {
    Rig rig;
    LoopMonitor mon{rig.clock, rig.latch, LoopMonitorConfig{Time{0.5}}};

    (void)mon.tick();  // baseline at t=0
    rig.clock.advance(Time{0.125});
    CHECK(mon.tick().value() == doctest::Approx(0.125));
    rig.clock.advance(Time{0.25});
    CHECK(mon.tick().value() == doctest::Approx(0.25));
    rig.clock.advance(Time{0.0625});
    (void)mon.tick();

    CHECK_FALSE(rig.latch.hasFault());
    CHECK(mon.overrunCount() == 0);
    CHECK(mon.worstDt().value() == doctest::Approx(0.25));  // the max, not the last
}

TEST_CASE("LoopMonitor: the exact boundary — dt just below budget passes, dt == budget "
          "FAULTS (>= is inclusive; '>' would miss this)") {
    Rig rig;
    LoopMonitor mon{rig.clock, rig.latch, LoopMonitorConfig{Time{0.5}}};

    // Just below: 0.499999 < 0.5.
    (void)mon.tick();  // baseline at 0
    rig.clock.set(Time{0.499999});
    (void)mon.tick();
    CHECK_FALSE(rig.latch.hasFault());
    CHECK(mon.overrunCount() == 0);

    // Exactly AT the budget: 1.0 → 1.5 is a float-exact dt of 0.5. This is THE edge.
    // (Re-baseline first: the clock JUMP to 1.0 is not a tick under test.)
    mon.reset();
    rig.clock.set(Time{1.0});
    (void)mon.tick();
    rig.clock.set(Time{1.5});
    const Time dt = mon.tick();
    CHECK(dt.value() == 0.5);  // exact by construction — the boundary is really hit
    CHECK(rig.latch.hasFault());
    CHECK(rig.latch.firstFault() == FaultCode::LoopOverrun);
    CHECK(mon.overrunCount() == 1);
}

TEST_CASE("LoopMonitor: an overrun raises a structured LOOP_OVERRUN with the measured dt") {
    Rig rig;
    LoopMonitor mon{rig.clock, rig.latch, LoopMonitorConfig{Time{0.5}}};

    (void)mon.tick();
    rig.clock.advance(Time{0.75});
    (void)mon.tick();

    REQUIRE(rig.sink.size() == 1);
    CHECK(rig.sink.at(0).subsystem == "DIAG");
    CHECK(rig.sink.at(0).message == "fault=LOOP_OVERRUN n=1 FIRST dt=0.7500 budget=0.5000");
}

TEST_CASE("LoopMonitor: repeated overruns cascade into the latch; the first fault stays first") {
    Rig rig;
    FaultLatch& latch = rig.latch;
    LoopMonitor mon{rig.clock, latch, LoopMonitorConfig{Time{0.5}}};

    latch.raise(FaultCode::NanPose, "LOC", "");  // a pre-existing root cause
    (void)mon.tick();
    rig.clock.advance(Time{1.0});
    (void)mon.tick();
    rig.clock.advance(Time{2.0});
    (void)mon.tick();

    CHECK(mon.overrunCount() == 2);
    CHECK(latch.faultCount() == 3);
    CHECK(latch.firstFault() == FaultCode::NanPose);  // the overruns did not usurp it
    CHECK(mon.worstDt().value() == doctest::Approx(2.0));
}

TEST_CASE("LoopMonitor: reset() re-baselines — a deliberate gap is not an overrun") {
    Rig rig;
    LoopMonitor mon{rig.clock, rig.latch, LoopMonitorConfig{Time{0.5}}};

    (void)mon.tick();
    rig.clock.advance(Time{0.25});
    (void)mon.tick();

    mon.reset();
    rig.clock.advance(Time{500.0});  // e.g. sitting between runs
    const Time dt = mon.tick();      // baseline again — must NOT fault
    CHECK(dt.value() == doctest::Approx(0.0));
    CHECK_FALSE(rig.latch.hasFault());
    CHECK(mon.worstDt().value() == doctest::Approx(0.25));  // history survives reset

    rig.clock.advance(Time{0.75});  // but the NEXT gap after re-baselining counts again
    (void)mon.tick();
    CHECK(rig.latch.firstFault() == FaultCode::LoopOverrun);
}

TEST_CASE("LoopMonitor: a non-positive budget is rejected at construction") {
    Rig rig;
    CHECK_THROWS_AS((LoopMonitor{rig.clock, rig.latch, LoopMonitorConfig{Time{0.0}}}),
                    PreconditionError);
    CHECK_THROWS_AS((LoopMonitor{rig.clock, rig.latch, LoopMonitorConfig{Time{-0.01}}}),
                    PreconditionError);
}
