// Adversarial tests for SettledUtil. Targets: the THREE conditions are all required —
// small error alone isn't enough if it's still moving (rate), and a single in-window tick
// isn't enough (held time) — plus window reset on a break, immediate settle, and reset.

#include "doctest.h"

#include "shulib/control/settled_util.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::SettledUtil;
using shulib::hal::fake::FakeClock;
using shulib::units::Time;

TEST_CASE("SettledUtil: a large error never settles") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 100.0, .settleTime = 0.0}, clk};
    CHECK_FALSE(s.update(50.0));
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(50.0));  // |error| way over maxError
}

TEST_CASE("SettledUtil: small error but high rate does NOT settle (still moving)") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 2.0, .settleTime = 0.0}, clk};
    (void)s.update(0.0);     // prime
    clk.advance(Time{0.1});
    // error 0 → 0.5 in 0.1s = rate 5 (> maxErrorRate 2): inside the position band but moving fast
    CHECK_FALSE(s.update(0.5));
}

TEST_CASE("SettledUtil: settles only after error AND rate hold for settleTime") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.3}, clk};
    (void)s.update(0.5);         // prime @ t=0 (no rate yet)
    // hold error 0.5 (rate 0). The window can only open on the first tick with a valid rate
    // (t=0.1), so settling needs 0.3 s FROM THERE → t=0.4, not t=0.3.
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(0.5));  // t=0.1: window opens, 0 s held
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(0.5));  // t=0.2: 0.1 s held < 0.3
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(0.5));  // t=0.3: 0.2 s held < 0.3
    clk.advance(Time{0.1});
    CHECK(s.update(0.5));        // t=0.4: 0.3 s held ≥ 0.3 → settled
}

TEST_CASE("SettledUtil: a condition break resets the held window") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 100.0, .settleTime = 0.3}, clk};
    (void)s.update(0.5);  // prime
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(0.5));  // window open 0.1s
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(5.0));  // error jumps out → window resets
    clk.advance(Time{0.1});
    CHECK_FALSE(s.update(0.5));  // back in, but window restarted → only 0.1s held
}

TEST_CASE("SettledUtil: settleTime 0 settles on the first in-window tick (with a valid rate)") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, clk};
    CHECK_FALSE(s.update(0.2));  // first call: no rate yet
    clk.advance(Time{0.1});
    CHECK(s.update(0.2));        // error small, rate 0, settleTime 0 → settled now
}

TEST_CASE("SettledUtil: reset clears the settled state") {
    FakeClock clk;
    SettledUtil s{{.maxError = 1.0, .maxErrorRate = 1.0, .settleTime = 0.0}, clk};
    (void)s.update(0.2);
    clk.advance(Time{0.1});
    CHECK(s.update(0.2));
    CHECK(s.isSettled());

    s.reset();
    CHECK_FALSE(s.isSettled());
    CHECK_FALSE(s.update(0.2));  // first call again after reset
}

TEST_CASE("SettledUtil: construction rejects negative thresholds") {
    FakeClock clk;
    CHECK_THROWS_AS((SettledUtil{{.maxError = -1.0}, clk}), PreconditionError);
    CHECK_THROWS_AS((SettledUtil{{.maxErrorRate = -1.0}, clk}), PreconditionError);
    CHECK_THROWS_AS((SettledUtil{{.settleTime = -1.0}, clk}), PreconditionError);
}
