// Adversarial tests for the controller→canonical conversion (chunk R1a).
// Every expected value is a HAND-COMPUTED LITERAL (E2's lesson).

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/controller_conversion.hpp"

using shulib::PreconditionError;
using shulib::hal::controllerAxisToCanonical;

TEST_CASE("controllerAxisToCanonical: ÷127 maps both rails exactly to ±1 (HA-103)") {
    // BUG CAUGHT: the ÷127 dropped (raw counts passed through) — the smallest
    // stick wiggle saturates the speed budget and the robot lurches at full
    // speed; or ÷128 (the plausible power-of-two typo), which makes full
    // deflection 0.992 and the robot can never reach its top speed.
    CHECK(controllerAxisToCanonical(127.0) == doctest::Approx(1.0));
    CHECK(controllerAxisToCanonical(-127.0) == doctest::Approx(-1.0));
    CHECK(controllerAxisToCanonical(0.0) == doctest::Approx(0.0));
    // Hand-computed: 64/127 = 0.5039370078740157; 1/127 = 0.007874015748031496.
    CHECK(controllerAxisToCanonical(64.0) == doctest::Approx(0.5039370078740157));
    CHECK(controllerAxisToCanonical(1.0) == doctest::Approx(0.007874015748031496));
}

TEST_CASE("controllerAxisToCanonical: out-of-range raw clamps — the contract holds anyway") {
    // BUG CAUGHT: an undocumented −128 (int8 min) leaking through as −1.008 —
    // downstream code that trusts |axis| ≤ 1 (speed budget scaling) would
    // command 0.8% over budget, invisibly. The canonical contract must be
    // unconditionally true, not true-if-the-doc-is-right.
    CHECK(controllerAxisToCanonical(-128.0) == doctest::Approx(-1.0));
    CHECK(controllerAxisToCanonical(200.0) == doctest::Approx(1.0));
}

TEST_CASE("controllerAxisToCanonical: symmetric — no sign bias between the rails") {
    // BUG CAUGHT: an asymmetric mapping (e.g. offsetting before scaling) —
    // the robot drifts under a centred stick in one direction only, which a
    // driver reports as "it pulls left" and nobody can reproduce on the bench.
    for (double raw = 0.0; raw <= 127.0; raw += 17.0) {
        CHECK(controllerAxisToCanonical(raw) == doctest::Approx(-controllerAxisToCanonical(-raw)));
    }
}

TEST_CASE("controllerAxisToCanonical: non-finite is rejected") {
    // BUG CAUGHT: a NaN reaching the speed-budget multiply and poisoning the
    // commanded twist — the teleop loop would command NaN wheel volts.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)controllerAxisToCanonical(nan), PreconditionError);
    CHECK_THROWS_AS((void)controllerAxisToCanonical(inf), PreconditionError);
}
