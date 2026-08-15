// Adversarial tests for Feedforward + compensateForBattery. Targets: each gain term in
// isolation, kS following the SIGN of velocity (and being zero at rest), the combined sum,
// and the battery ceiling clamp + its saturation flag.

#include "doctest.h"

#include <limits>

#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::control::CompensatedVoltage;
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

// Bug caught (DEFECTS1 items A2 + I2): the saturation flag lied about a NaN, and the struct
// could hold an indeterminate one.
//
// A2 — the flag was computed as `|d| > b`. Both comparisons against NaN are false, so a NaN
// request came back reported CLEAN: the one function whose job is to bound a commanded
// voltage asserting that a non-value is inside the battery envelope. Written `!(|d| <= b)`
// it lands on the true side. The NaN itself is still passed through DELIBERATELY — the volt
// path recovers at the motor edge (diag::recoverWheelVoltage, plausibility_guard.hpp
// invariant 3), so swallowing it here would lose the Implausible fault, and throwing here
// would turn a hostile-sensor pathology into an aborted motion.
//
// I2 — brownoutLimited had no default member initializer while its sibling self-initializes
// through Quantity, so `CompensatedVoltage c;` left a safety flag holding whatever was on
// the stack.
TEST_CASE("compensateForBattery: a non-finite desired is FLAGGED, not reported clean (A2)") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    const auto n = compensateForBattery(Voltage{nan}, Voltage{12.0});
    CHECK(n.brownoutLimited);                       // was FALSE — the defect
    CHECK(std::isnan(n.voltage.value()));           // still passed on, on purpose

    const auto i = compensateForBattery(Voltage{inf}, Voltage{12.0});
    CHECK(i.brownoutLimited);
    CHECK(i.voltage.value() == doctest::Approx(12.0));  // inf DOES clamp

    // NEGATIVE CONTROL: an ordinary in-budget request must still read clean, or the flag
    // would be true for everything and the checks above would prove nothing.
    const auto ok = compensateForBattery(Voltage{6.0}, Voltage{12.0});
    CHECK_FALSE(ok.brownoutLimited);
    CHECK(ok.voltage.value() == doctest::Approx(6.0));
}

TEST_CASE("CompensatedVoltage: a default-constructed one holds a determinate flag (I2)") {
    const CompensatedVoltage c{};
    CHECK_FALSE(c.brownoutLimited);
    CHECK(c.voltage.value() == 0.0);
}
