// Adversarial tests for Feedforward + compensateForBattery. Targets: each gain term in
// isolation, kS following the SIGN of velocity (and being zero at rest), the combined sum,
// and the battery ceiling clamp + its saturation flag.

#include "doctest.h"

#include <limits>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::compensateForBattery;
using shulib::control::Feedforward;
using shulib::units::Acceleration;
using shulib::units::Velocity;
using shulib::units::Voltage;

TEST_CASE("Feedforward: each term scales its input") {
    const Feedforward ff{{.kS = 1.0, .kV = 0.5, .kA = 0.1}};
    // v = 10, a = 4 → kS(+1) + kV·10(5) + kA·4(0.4)
    CHECK(ff.calculate(Velocity{10.0}, Acceleration{4.0}).value() == doctest::Approx(6.4));
    // velocity-only overload (a = 0)
    CHECK(ff.calculate(Velocity{10.0}).value() == doctest::Approx(1.0 + 5.0));
}

TEST_CASE("Feedforward: kS follows the sign of velocity and is zero at rest") {
    const Feedforward ff{{.kS = 1.0, .kV = 0.5}};
    CHECK(ff.calculate(Velocity{4.0}).value() == doctest::Approx(1.0 + 2.0));    // +kS
    CHECK(ff.calculate(Velocity{-4.0}).value() == doctest::Approx(-1.0 - 2.0));  // −kS, opposite
    CHECK(ff.calculate(Velocity{0.0}).value() == doctest::Approx(0.0));          // no kS at rest
}

TEST_CASE("Feedforward: pure kA term (at rest, accelerating)") {
    const Feedforward ff{{.kA = 0.3}};
    CHECK(ff.calculate(Velocity{0.0}, Acceleration{2.0}).value() == doctest::Approx(0.6));
}

TEST_CASE("Feedforward: non-finite gains are rejected") {
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((Feedforward{{.kV = inf}}), PreconditionError);
}

TEST_CASE("compensateForBattery: a desired within budget passes through unflagged") {
    const auto r = compensateForBattery(Voltage{6.0}, Voltage{12.0});
    CHECK(r.voltage.value() == doctest::Approx(6.0));
    CHECK_FALSE(r.brownoutLimited);
}

TEST_CASE("compensateForBattery: an over-budget desired is clamped to ±battery and flagged") {
    const auto hi = compensateForBattery(Voltage{13.0}, Voltage{12.0});
    CHECK(hi.voltage.value() == doctest::Approx(12.0));
    CHECK(hi.brownoutLimited);

    const auto lo = compensateForBattery(Voltage{-13.0}, Voltage{12.0});
    CHECK(lo.voltage.value() == doctest::Approx(-12.0));
    CHECK(lo.brownoutLimited);

    // a sagging battery lowers the ceiling
    const auto sag = compensateForBattery(Voltage{6.0}, Voltage{5.0});
    CHECK(sag.voltage.value() == doctest::Approx(5.0));
    CHECK(sag.brownoutLimited);
}

TEST_CASE("compensateForBattery: a negative battery voltage is a contract violation") {
    CHECK_THROWS_AS((void)compensateForBattery(Voltage{1.0}, Voltage{-1.0}), PreconditionError);
}
