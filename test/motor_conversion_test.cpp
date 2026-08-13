// Adversarial tests for the motor→canonical conversions (chunk R1a). Every
// expected value is a HAND-COMPUTED LITERAL — never the header's own constant
// re-imported — because E2 found the pre-E2 GPS test asserting kMetersToInches
// against kMetersToInches, where a wrong constant satisfies both sides of the
// ==. A wrong scale here must FAIL against an independently derived number.

#include "doctest.h"

#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor_conversion.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::motorMilliampsToCanonical;
using shulib::hal::motorPositionDegToCanonical;
using shulib::hal::motorRpmToCanonical;
using shulib::hal::motorVoltageApplied;
using shulib::hal::motorVoltageToMillivolts;
using shulib::units::Voltage;

TEST_CASE("motorVoltageToMillivolts: volts scale x1000 to the wire (HA-94)") {
    // BUG CAUGHT: the mV scale dropped (volts sent as millivolts) — every
    // command becomes 1/1000 of the intended torque; the robot hums in place
    // and "a command was sent" style checks stay green. Mutation M1's target.
    CHECK(motorVoltageToMillivolts(Voltage{6.0}) == 6000);
    CHECK(motorVoltageToMillivolts(Voltage{-6.0}) == -6000);
    CHECK(motorVoltageToMillivolts(Voltage{12.0}) == 12000);
    CHECK(motorVoltageToMillivolts(Voltage{-12.0}) == -12000);
    CHECK(motorVoltageToMillivolts(Voltage{0.0}) == 0);
    // 3.3333 V → 3333.3 mV → nearest integer millivolt.
    CHECK(motorVoltageToMillivolts(Voltage{3.3333}) == 3333);
}

TEST_CASE("motorVoltageToMillivolts: the ±12 V hardware clamp holds at the wire") {
    // BUG CAUGHT: a >12 V request reaching move_voltage() — PROS clamps or
    // rejects unpredictably, and commandedVoltage() telemetry would disagree
    // with the wire. The clamp must happen in OUR conversion, visibly.
    CHECK(motorVoltageToMillivolts(Voltage{13.5}) == 12000);
    CHECK(motorVoltageToMillivolts(Voltage{-13.5}) == -12000);
    CHECK(motorVoltageToMillivolts(Voltage{1000.0}) == 12000);
    CHECK(motorVoltageToMillivolts(Voltage{-1000.0}) == -12000);
}

TEST_CASE("motorVoltageToMillivolts: rounds to the nearest millivolt, not truncates") {
    // BUG CAUGHT: truncation-toward-zero — a persistent sub-millivolt bias on
    // every command, asymmetric between + and −, which a feedforward sweep
    // would read as asymmetric kS. lround rounds half away from zero.
    CHECK(motorVoltageToMillivolts(Voltage{0.0015}) == 2);
    CHECK(motorVoltageToMillivolts(Voltage{-0.0015}) == -2);
    CHECK(motorVoltageToMillivolts(Voltage{0.0004}) == 0);
    CHECK(motorVoltageToMillivolts(Voltage{5.9996}) == 6000);
}

TEST_CASE("motorVoltageApplied: reports the post-clamp value the wire really got") {
    // BUG CAUGHT: commandedVoltage() reporting the REQUEST instead of the
    // applied value — telemetry showing an impossible 15 V while the motor got
    // 12, which poisons every battery-compensation calculation reading it.
    CHECK(motorVoltageApplied(Voltage{13.5}).value() == doctest::Approx(12.0));
    CHECK(motorVoltageApplied(Voltage{-20.0}).value() == doctest::Approx(-12.0));
    CHECK(motorVoltageApplied(Voltage{5.25}).value() == doctest::Approx(5.25));
}

TEST_CASE("motorVoltageToMillivolts/Applied: non-finite is rejected, never coerced (L4)") {
    // BUG CAUGHT: the measurement prototype's own mistake — silently coercing
    // NaN to 0.0, which converts a programming error into a robot that coasts
    // mid-run with green tests. Mutation M9's target.
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)motorVoltageToMillivolts(Voltage{nan}), PreconditionError);
    CHECK_THROWS_AS((void)motorVoltageToMillivolts(Voltage{inf}), PreconditionError);
    CHECK_THROWS_AS((void)motorVoltageApplied(Voltage{-inf}), PreconditionError);
}

TEST_CASE("motorPositionDegToCanonical: degrees to radians, CUMULATIVE (never wrapped)") {
    // BUG CAUGHT: routing position through the wrapping math::Angle — at the
    // first ±180° crossing odometry's travel delta jumps by 2π and the pose
    // teleports. 720° must stay 720°'s radians, not wrap to 0.
    // Hand-computed: 180° = 3.14159265358979312 rad; 720° = 12.566370614359172.
    CHECK(motorPositionDegToCanonical(180.0).value() == doctest::Approx(3.14159265358979312));
    CHECK(motorPositionDegToCanonical(90.0).value() == doctest::Approx(1.5707963267948966));
    CHECK(motorPositionDegToCanonical(720.0).value() == doctest::Approx(12.566370614359172));
    CHECK(motorPositionDegToCanonical(-450.0).value() == doctest::Approx(-7.853981633974483));
    CHECK(motorPositionDegToCanonical(0.0).value() == doctest::Approx(0.0));
}

TEST_CASE("motorRpmToCanonical: RPM to rad/s via 2π/60 (HA-96)") {
    // BUG CAUGHT: mutation M14's shape — RPM passed through as if rad/s.
    // 60 RPM is exactly 1 rev/s = 2π rad/s; a pass-through would report 60,
    // an error of ~9.5×, and the feedforward would command ~1/9.5 the speed.
    // Hand-computed: 60 RPM → 6.283185307179586; 200 RPM → 20.943951023931955.
    CHECK(motorRpmToCanonical(60.0).value() == doctest::Approx(6.283185307179586));
    CHECK(motorRpmToCanonical(200.0).value() == doctest::Approx(20.943951023931955));
    CHECK(motorRpmToCanonical(-100.0).value() == doctest::Approx(-10.471975511965978));
    CHECK(motorRpmToCanonical(0.0).value() == doctest::Approx(0.0));
}

TEST_CASE("motorMilliampsToCanonical: mA to A via ÷1000 (HA-97)") {
    // BUG CAUGHT: mutation M2's target — mA read as A. 2500 mA would read as
    // 2500 A: stall detection (thresholds ~2.5 A) fires on a free-spinning
    // motor and every sensor-confirm operation reports a phantom grab.
    CHECK(motorMilliampsToCanonical(2500.0).value() == doctest::Approx(2.5));
    CHECK(motorMilliampsToCanonical(50.0).value() == doctest::Approx(0.05));
    CHECK(motorMilliampsToCanonical(-300.0).value() == doctest::Approx(-0.3));
}

TEST_CASE("motor conversions: non-finite readings are rejected by the backstop") {
    // BUG CAUGHT: a PROS_ERR_F (= +inf) slipping past the adapter's screen and
    // INTO a conversion — the backstop must throw (fail-loud), never return an
    // infinite canonical value for the estimators to integrate.
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_THROWS_AS((void)motorPositionDegToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)motorRpmToCanonical(inf), PreconditionError);
    CHECK_THROWS_AS((void)motorMilliampsToCanonical(inf), PreconditionError);
}
