// Adapter tests for ProsImu THROUGH THE HOST SHIM (chunk R1a). The shim tests
// the adapter against our belief about PROS (HA-02..05, HA-108/109); hardware
// tests the belief. The shim is deliberately ADVERSARIAL about
// get_rotation()/get_heading(): they agree only inside the first positive
// revolution, so the forbidden binding cannot hide.

#include "doctest.h"

#include "pros/shim_control.hpp"

#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/pros/imu.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::fake::FakeClock;
using shulib::hal::pros::ProsImu;
using shulib::hal::pros::YawRateSource;
using shulib::math::Angle;
using shulib::units::Time;

namespace {
/// A calibrated-and-ready ProsImu on `port` (the shim starts non-calibrating,
/// so only the calibrate() bookkeeping is needed for isReady()).
ProsImu readyImu(int port, const FakeClock& clock, Angle boot = Angle::degrees(0.0),
                 YawRateSource src = YawRateSource::DifferentiateRotation) {
    ProsImu imu{static_cast<std::uint8_t>(port), boot, clock, src};
    imu.calibrate();
    pros::shim::imuState(port).calibrating = false;  // calibration finishes
    return imu;
}
}  // namespace

TEST_CASE("ProsImu: heading binds get_rotation(), NOT get_heading() — continuity past 360° (HA-03)") {
    // BUG CAUGHT (mutation M4): binding the bounded [0,360) get_heading().
    // The shim wraps get_heading() while get_rotation() accumulates, so at
    // 450° cumulative CW the two DISAGREE (450 vs 90): only the cumulative
    // binding converts to the same canonical heading as 90° + a full recorded
    // revolution — and the conversion of 450 vs 90 agrees in wrapped canonical
    // space. The discriminating read: a heading-bound adapter at rotation
    // -90 reads 270 and converts to canonical +90... also equal wrapped.
    // The REAL discriminator is the differentiated yaw rate: across the 360°
    // seam a heading-bound differentiation sees a -360° step in one tick — a
    // huge phantom rate — while the rotation-bound one sees the true 2°.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(10, clock);

    pros::shim::imuState(10).rotationDegCw = 359.0;
    clock.advance(Time{0.01});
    (void)imu.yawRate();  // primes the differentiator at 359°
    pros::shim::imuState(10).rotationDegCw = 361.0;  // crosses the wrap seam
    clock.advance(Time{0.01});
    const double rate = imu.yawRate().value();
    // True rate: +2° CW over 10 ms = 200 deg/s CW → canonical −3.490658503988659 rad/s.
    // A get_heading()-bound adapter reads 359→1: −358°/10ms → +624.8 rad/s. Night and day.
    CHECK(rate == doctest::Approx(-3.490658503988659));

    // And the canonical heading at 450° cumulative equals boot − 450° wrapped:
    // 0 − 450 → wrap(−450°) = −90° = −1.5707963267948966 rad.
    pros::shim::imuState(10).rotationDegCw = 450.0;
    CHECK(imu.heading().radians() == doctest::Approx(-1.5707963267948966));
}

TEST_CASE("ProsImu: heading applies bootHeading exactly ONCE, at the edge (HA-05)") {
    // BUG CAUGHT: the boot offset dropped (robot thinks it started at 0°) or
    // applied twice (every downstream consumer re-adding it). Boot at +90°,
    // IMU turned 30° CW → canonical must be exactly 90 − 30 = 60° =
    // 1.0471975511965976 rad.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(11, clock, Angle::degrees(90.0));
    pros::shim::imuState(11).rotationDegCw = 30.0;
    CHECK(imu.heading().radians() == doctest::Approx(1.0471975511965976));
}

TEST_CASE("ProsImu: isReady is POSITIVE polarity and false until calibrate() ran (HA-23)") {
    // BUG CAUGHT: binding is_calibrating() without negation (ready reads
    // backwards), or ready-by-default before anyone calibrated — an
    // uncalibrated IMU emits garbage-that-moves and the localizer would trust
    // it from tick zero.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu{12, Angle::degrees(0.0), clock};
    CHECK_FALSE(imu.isReady());  // never calibrated → not ready, even though not calibrating
    imu.calibrate();
    CHECK_FALSE(imu.isReady());  // calibrating now
    CHECK(pros::shim::imuState(12).resetCalls == 1);
    pros::shim::imuState(12).calibrating = false;
    CHECK(imu.isReady());
}

TEST_CASE("ProsImu: calibrate() twice is a precondition violation (HA-05's re-zero hazard)") {
    // BUG CAUGHT: a second reset() mid-run re-zeroing the sensor under a live
    // bootHeading — a silent constant heading offset from that instant on,
    // unexplainable from telemetry (the exact HA-05 failure).
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu{13, Angle::degrees(0.0), clock};
    imu.calibrate();
    CHECK_THROWS_AS(imu.calibrate(), PreconditionError);
    CHECK(pros::shim::imuState(13).resetCalls == 1);  // the second call never reached the device
}

TEST_CASE("ProsImu: the adapter NEVER calls the tare family (HA-05)") {
    // BUG CAUGHT: any code path sneaking in a tare/set_rotation/set_heading —
    // the shim counts them; after construction + calibration + a full read of
    // every quantity, the count must be zero.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(14, clock);
    pros::shim::imuState(14).rotationDegCw = 123.0;
    (void)imu.heading();
    (void)imu.yawRate();
    (void)imu.pitch();
    (void)imu.roll();
    (void)imu.isReady();
    CHECK(pros::shim::imuState(14).tareFamilyCalls == 0);
}

TEST_CASE("ProsImu: differentiation branch — d/dt of get_rotation, CW→CCW negated") {
    // BUG CAUGHT: the differentiator using wall-instead-of-clock time, missing
    // the CW→CCW negate, or updating state without computing (rate stuck at 0).
    // 90° CW over 0.5 s = 180 deg/s CW → canonical −π = −3.14159265358979312.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(15, clock);
    (void)imu.yawRate();  // prime at rotation 0, t=0
    pros::shim::imuState(15).rotationDegCw = 90.0;
    clock.advance(Time{0.5});
    CHECK(imu.yawRate().value() == doctest::Approx(-3.14159265358979312));
    // Same tick again (dt = 0): no new information — the rate must HOLD, not
    // divide by zero.
    CHECK(imu.yawRate().value() == doctest::Approx(-3.14159265358979312));
}

TEST_CASE("ProsImu: gyro branch — get_gyro_rate().z through the same canonical negate (T4)") {
    // BUG CAUGHT: the flag shipping an untested second branch (an untaken
    // branch on a robot is where bugs hide). z = +45 deg/s CW must read
    // canonical −0.7853981633974483 rad/s.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(16, clock, Angle::degrees(0.0), YawRateSource::GyroRateZ);
    pros::shim::imuState(16).gyroZDegPerSecCw = 45.0;
    CHECK(imu.yawRate().value() == doctest::Approx(-0.7853981633974483));
    pros::shim::imuState(16).gyroZDegPerSecCw = -90.0;
    CHECK(imu.yawRate().value() == doctest::Approx(1.5707963267948966));
}

TEST_CASE("ProsImu: sentinel reads hold last-good; heading before any good read is bootHeading (T7)") {
    // BUG CAUGHT (mutation M8): PROS_ERR_F reaching imuHeadingToCanonical —
    // which THROWS by design (the backstop), so an unscreened adapter turns a
    // calibration window into a crash; or a screened read returning 0 heading
    // (a phantom snap to +X mid-run).
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu{17, Angle::degrees(45.0), clock};
    imu.calibrate();  // shim now calibrating → reads are PROS_ERR_F
    // During calibration: heading is the bootHeading (the robot has not moved),
    // and nothing throws.
    CHECK(imu.heading().radians() == doctest::Approx(0.7853981633974483));
    CHECK(imu.faultedReads() > 0);

    pros::shim::imuState(17).calibrating = false;
    pros::shim::imuState(17).rotationDegCw = 45.0;
    CHECK(imu.heading().radians() == doctest::Approx(0.0));  // 45 boot − 45 turned

    // Mid-run drop: hold the last good heading, not boot, not zero.
    pros::shim::imuState(17).disconnected = true;
    CHECK(imu.heading().radians() == doctest::Approx(0.0));
}

TEST_CASE("ProsImu: pitch/roll pass through as canonical Angles (sign is HA-110's bench item)") {
    // BUG CAUGHT: pitch/roll swapped, or converted through the yaw handedness
    // negate (tip detection would see a climb as a dive). Passed through
    // unnegated BY DESIGN until the bench settles HA-110.
    pros::shim::resetAll();
    FakeClock clock;
    ProsImu imu = readyImu(18, clock);
    pros::shim::imuState(18).pitchDeg = 10.0;
    pros::shim::imuState(18).rollDeg = -5.0;
    CHECK(imu.pitch().degrees() == doctest::Approx(10.0));
    CHECK(imu.roll().degrees() == doctest::Approx(-5.0));
}
