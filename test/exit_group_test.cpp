// Adversarial tests for Watchdog + ExitGroup. Watchdog: never expires before start or
// before the timeout, expires at/after it. ExitGroup: Running until something fires, Settled
// on a held-in-band error, TimedOut when the error never settles, settled-takes-priority, and
// re-arm via start().

#include "doctest.h"

#include "shulib/control/exit_group.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::ExitGroup;
using shulib::control::ExitReason;
using shulib::control::Watchdog;
using shulib::hal::fake::FakeClock;
using shulib::units::Time;

TEST_CASE("Watchdog: never expires before start") {
    FakeClock clk;
    Watchdog w{1.0, clk};
    CHECK_FALSE(w.expired());      // not started
    CHECK_FALSE(w.started());
}

TEST_CASE("Watchdog: expires only at/after the timeout") {
    FakeClock clk;
    Watchdog w{1.0, clk};
    w.start();
    CHECK_FALSE(w.expired());
    clk.advance(Time{0.9});
    CHECK_FALSE(w.expired());
    CHECK(w.elapsed() == doctest::Approx(0.9));
    clk.advance(Time{0.1});
    CHECK(w.expired());            // exactly at 1.0
    clk.advance(Time{5.0});
    CHECK(w.expired());            // stays expired
}

TEST_CASE("Watchdog: reset disarms it; re-start re-arms from now") {
    FakeClock clk;
    Watchdog w{1.0, clk};
    w.start();
    clk.advance(Time{2.0});
    CHECK(w.expired());
    w.reset();
    CHECK_FALSE(w.expired());      // disarmed
    clk.advance(Time{10.0});
    w.start();                     // re-arm from t=12
    CHECK_FALSE(w.expired());
}

TEST_CASE("Watchdog: rejects a non-positive timeout; elapsed() before start throws") {
    FakeClock clk;
    CHECK_THROWS_AS((Watchdog{0.0, clk}), PreconditionError);
    Watchdog w{1.0, clk};
    CHECK_THROWS_AS((void)w.elapsed(), PreconditionError);
}

TEST_CASE("ExitGroup: Settled when the error holds in band before the timeout") {
    FakeClock clk;
    ExitGroup g{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, /*timeout=*/5.0, clk};
    g.start();
    CHECK(g.check(0.2) == ExitReason::Running);  // first tick: no rate yet
    clk.advance(Time{0.1});
    CHECK(g.check(0.2) == ExitReason::Settled);  // small error, zero rate, settleTime 0
}

TEST_CASE("ExitGroup: TimedOut when the error never settles") {
    FakeClock clk;
    ExitGroup g{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, /*timeout=*/0.5, clk};
    g.start();
    CHECK(g.check(100.0) == ExitReason::Running);  // way off target
    clk.advance(Time{0.4});
    CHECK(g.check(100.0) == ExitReason::Running);
    clk.advance(Time{0.2});                         // t=0.6 > 0.5
    CHECK(g.check(100.0) == ExitReason::TimedOut);
}

TEST_CASE("ExitGroup: Settled takes priority over a simultaneous timeout") {
    FakeClock clk;
    ExitGroup g{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, /*timeout=*/0.1, clk};
    g.start();
    (void)g.check(0.2);             // prime
    clk.advance(Time{0.2});         // both: settled-able AND past the 0.1 timeout
    CHECK(g.check(0.2) == ExitReason::Settled);  // success wins
}

TEST_CASE("ExitGroup: start re-arms (a fresh motion isn't pre-timed-out)") {
    FakeClock clk;
    ExitGroup g{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, /*timeout=*/0.5, clk};
    g.start();
    clk.advance(Time{1.0});
    CHECK(g.check(100.0) == ExitReason::TimedOut);
    g.start();                      // new motion
    CHECK(g.check(100.0) == ExitReason::Running);  // not pre-expired
}
