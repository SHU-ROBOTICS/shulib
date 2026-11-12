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

// ── setGains (chunk TUNE1-A, 2026-11-10 — live tuning's BUMPLESS rule) ──────────────────────
// A gain changed mid-loop must make no jump of its own: the integral TERM (kI·∫e, the volts it adds)
// survives, not the raw accumulator. Each case runs a TWIN that keeps its gains and pins the difference
// at the next update to exactly what the new gains account for.

TEST_CASE("Pid::setGains: a new kI keeps the integral TERM -- the next update differs from its twin's by only the new gain's share of this tick") {
    FakeClock clk;
    Pid pid{{.kP = 0.5, .kI = 0.4, .kD = 0.05}, clk};
    Pid twin{{.kP = 0.5, .kI = 0.4, .kD = 0.05}, clk};
    for (int i = 0; i < 50; ++i) {  // wind the integral: a steady error of 3 for 0.5 s
        (void)pid.update(10.0, 7.0);
        (void)twin.update(10.0, 7.0);
        clk.advance(Time{0.01});
    }
    const double term = 0.4 * pid.integralAccumulator();
    REQUIRE(term > 0.5);  // ~0.6 V wound in: a kept accumulator would show
    pid.setGains(0.5, 0.8, 0.05);
    CHECK(0.8 * pid.integralAccumulator() == doctest::Approx(term).epsilon(1e-12));  // the TERM, not the accumulator
    CHECK(pid.config().kI == 0.8);
    const double a = pid.update(10.0, 7.0);
    const double b = twin.update(10.0, 7.0);
    CHECK(std::abs((a - b) - 0.4 * 3.0 * 0.01) < 1e-12);  // (0.8 − 0.4)·e·dt: nothing beyond it
    CHECK(std::abs(a - b) < 0.05);                         // a kept accumulator jumps by ~0.6 V
}

TEST_CASE("Pid::setGains: a new kP changes the next output by exactly ΔkP x the error; the derivative's baseline is kept (no first-call reset)") {
    FakeClock clk;
    Pid pid{{.kP = 0.5, .kD = 0.2}, clk};
    Pid twin{{.kP = 0.5, .kD = 0.2}, clk};
    (void)pid.update(10.0, 4.0);
    (void)twin.update(10.0, 4.0);
    clk.advance(Time{0.1});
    pid.setGains(2.0, 0.0, 0.2);
    const double a = pid.update(10.0, 5.0);  // the measurement moved 1 in 0.1 s: D = −0.2·10 = −2 on BOTH
    const double b = twin.update(10.0, 5.0);
    CHECK(b == doctest::Approx(0.5 * 5.0 - 2.0));
    CHECK(a == doctest::Approx(2.0 * 5.0 - 2.0));  // D still applied: the baseline was kept
    CHECK(std::abs((a - b) - 1.5 * 5.0) < 1e-12);
}

TEST_CASE("Pid::setGains: an UNCHANGED kI leaves the accumulator bit for bit; a new kI of 0 clears it; from 0 it starts at 0; limits kept") {
    FakeClock clk;
    Pid pid{{.kP = 1.0, .kI = 0.3, .integralLimit = 2.0, .outputMin = -5.0, .outputMax = 5.0}, clk};
    (void)pid.update(4.0, 1.0);
    clk.advance(Time{0.02});
    (void)pid.update(4.0, 1.0);
    const double acc = pid.integralAccumulator();
    REQUIRE(acc != 0.0);
    pid.setGains(2.0, 0.3, 0.1);
    // bit for bit. (TUNE1-A's review: an UNGUARDED rescale would pass this too, and no input can tell them
    // apart — every accumulator a Pid can hold is fl(w / kI) (update()'s back-calculation, setGains'
    // rescale, or 0), and that is a fixed point of fl(fl(kI·x) / kI): 0 of 20,000,000 random (w, kI) pairs
    // moved. So setGains' `kI != cfg_.kI` guard states the intent; its mutant is EQUIVALENT — logged.)
    CHECK(pid.integralAccumulator() == acc);
    CHECK(pid.config().integralLimit == 2.0);
    CHECK(pid.config().outputMax == 5.0);
    pid.setGains(2.0, 0.0, 0.1);
    CHECK(pid.integralAccumulator() == 0.0);
    pid.setGains(2.0, 0.6, 0.1);
    CHECK(pid.integralAccumulator() == 0.0);
}

TEST_CASE("Pid::setGains: a gain that is not finite is a precondition (the constructor's rule), and nothing changes") {
    FakeClock clk;
    Pid pid{{.kP = 1.0, .kI = 0.5}, clk};
    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS(pid.setGains(nan, 0.5, 0.0), PreconditionError);
    CHECK_THROWS_AS(pid.setGains(1.0, std::numeric_limits<double>::infinity(), 0.0), PreconditionError);
    CHECK(pid.config().kP == 1.0);
    CHECK(pid.config().kI == 0.5);
}
