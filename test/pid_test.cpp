// Adversarial tests for Pid. Each targets one design promise: P scaling, derivative
// ON MEASUREMENT (no setpoint kick), the derivative responding to real motion, integral
// accumulation, anti-windup, output clamp, the first-call / dt≤0 P-only behavior, reset,
// and the construction preconditions.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/control/pid.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::Pid;
using shulib::control::PidConfig;
using shulib::hal::fake::FakeClock;
using shulib::units::Time;

TEST_CASE("Pid: pure proportional scales the error") {
    FakeClock clk;
    Pid pid{{.kP = 2.0}, clk};
    CHECK(pid.update(10.0, 4.0) == doctest::Approx(12.0));  // 2·(10−4)
    clk.advance(Time{0.01});
    CHECK(pid.update(10.0, 4.0) == doctest::Approx(12.0));  // steady
}

TEST_CASE("Pid: derivative is on MEASUREMENT — a setpoint step produces no kick") {
    FakeClock clk;
    Pid pid{{.kP = 1.0, .kD = 0.5}, clk};
    (void)pid.update(0.0, 5.0);  // prime (first call, sets prevMeasurement = 5)
    clk.advance(Time{0.1});
    // setpoint jumps 0 → 100 while the measurement is unchanged at 5:
    const double out = pid.update(100.0, 5.0);
    CHECK(out == doctest::Approx(95.0));  // P only (1·95); D = −kD·(5−5)/dt = 0, NO kick
}

TEST_CASE("Pid: derivative responds to a measurement change") {
    FakeClock clk;
    Pid pid{{.kD = 0.5}, clk};  // kP = 0
    (void)pid.update(0.0, 0.0);       // prime
    clk.advance(Time{0.1});
    const double out = pid.update(0.0, 2.0);   // measurement 0 → 2 over 0.1 s
    CHECK(out == doctest::Approx(-10.0));      // −0.5·(2−0)/0.1
}

TEST_CASE("Pid: integral accumulates the error over time") {
    FakeClock clk;
    Pid pid{{.kI = 1.0}, clk};  // kP = kD = 0
    (void)pid.update(10.0, 0.0);      // prime (no integral on the first call)
    clk.advance(Time{0.1});
    CHECK(pid.update(10.0, 0.0) == doctest::Approx(1.0));  // ∫ += 10·0.1
    clk.advance(Time{0.1});
    CHECK(pid.update(10.0, 0.0) == doctest::Approx(2.0));  // ∫ += 10·0.1 again
}

TEST_CASE("Pid: integral anti-windup clamps the I-term") {
    FakeClock clk;
    Pid pid{{.kI = 1.0, .integralLimit = 1.5}, clk};
    (void)pid.update(10.0, 0.0);  // prime
    for (int i = 0; i < 10; ++i) {
        clk.advance(Time{0.1});
        (void)pid.update(10.0, 0.0);  // would reach 10 without the clamp
    }
    clk.advance(Time{0.1});
    CHECK(pid.update(10.0, 0.0) == doctest::Approx(1.5));  // I-term pinned at the limit
}

TEST_CASE("Pid: output is clamped to [outputMin, outputMax]") {
    FakeClock clk;
    Pid pid{{.kP = 100.0, .outputMin = -12.0, .outputMax = 12.0}, clk};
    CHECK(pid.update(10.0, 0.0) == doctest::Approx(12.0));    // 1000 → +12
    CHECK(pid.update(-10.0, 0.0) == doctest::Approx(-12.0));  // −1000 → −12
}

TEST_CASE("Pid: the first call and any dt≤0 tick apply P only (no NaN, no integral step)") {
    FakeClock clk;
    Pid pid{{.kP = 1.0, .kI = 1.0, .kD = 1.0}, clk};
    CHECK(pid.update(5.0, 0.0) == doctest::Approx(5.0));  // first call: P only

    // clock NOT advanced → dt = 0: still P only, no divide-by-zero in D, no ∫ step
    const double out = pid.update(5.0, 2.0);
    CHECK(out == doctest::Approx(3.0));
    CHECK(std::isfinite(out));
    CHECK(pid.integralAccumulator() == doctest::Approx(0.0));
}

TEST_CASE("Pid: reset clears integral and derivative history") {
    FakeClock clk;
    Pid pid{{.kI = 1.0}, clk};
    (void)pid.update(10.0, 0.0);
    clk.advance(Time{0.1});
    (void)pid.update(10.0, 0.0);
    CHECK(pid.integralAccumulator() == doctest::Approx(1.0));

    pid.reset();
    CHECK(pid.integralAccumulator() == doctest::Approx(0.0));
    clk.advance(Time{0.1});
    CHECK(pid.update(10.0, 0.0) == doctest::Approx(0.0));  // post-reset: like a first call (P=0 here)
}

TEST_CASE("Pid: construction rejects bad config") {
    FakeClock clk;
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((Pid{{.outputMin = 1.0, .outputMax = -1.0}, clk}), PreconditionError);
    CHECK_THROWS_AS((Pid{{.integralLimit = -1.0}, clk}), PreconditionError);
    CHECK_THROWS_AS((Pid{{.kP = nan}, clk}), PreconditionError);
}
