#pragma once
//
// ProsRotation — IRotation over pros::Rotation (chunk R1a): the tracking-wheel
// pods behind the HAL.
//
// BINDS:
//  * get_position() [int32 centidegrees, cumulative] → position()
//    (rotationCentidegToCanonical — HA-11/HA-16)
//  * get_velocity() [int32 centidegrees/s] → velocity()
//    (rotationCentidegPerSecToCanonical — HA-105)
//
// REVERSAL — "exactly once" (rotation.hpp:4-6): a NEGATIVE ctor port reverses
// the sensor, applied by PROS itself (vendored rotation.hpp:46-47). This
// adapter therefore does NOT negate, ever — sign lives in the port number the
// robot config states, one owner, one application.
//
// SENTINELS (T7): IRotation has no validity channel. PROS_ERR (= INT32_MAX)
// is IN-BAND for an int32 centidegree reading — 2147483647 centideg is ~59652
// revolutions, unreachable in a match (HA-11's arithmetic), so it is screened
// as a sentinel HERE: hold the last good value, never propagate, never zero
// (a zeroed tracking wheel reads as "the robot stopped" — the exact
// dead-encoder runaway the loop's ODO_STUCK cross-check exists to catch; a
// frozen last-good value is what that cross-check is DESIGNED to see).
// faultedReads() exposes the screen count; raising a fault stays with the
// loop layer (health_monitor.hpp: raising is policy, hal/ is below diag/).
//
// DELIBERATELY NOT here: no reset()/set_position() calls — odometry owns its
// own zero (TrackingWheel reads deltas); re-zeroing the device mid-run would
// step every consumer at once.
//
// HA register: HA-11, HA-16, HA-105 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/error.h"
#include "pros/rotation.hpp"
#pragma GCC diagnostic pop

#include <cstdint>

#include "shulib/hal/rotation.hpp"
#include "shulib/hal/rotation_conversion.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

class ProsRotation final : public IRotation {
public:
    /// `port`: 1..21, NEGATIVE to reverse (PROS applies the reversal — once).
    explicit ProsRotation(std::int8_t port) : sensor_{port} {}

    /// Cumulative radians (never wrapped). Sentinel-screened to last-good (T7).
    [[nodiscard]] units::AngleDim position() const override {
        const std::int32_t centideg = sensor_.get_position();
        if (centideg == PROS_ERR) {
            faultedReads_ += 1;
            return lastPosition_;
        }
        lastPosition_ = rotationCentidegToCanonical(static_cast<double>(centideg));
        return lastPosition_;
    }

    [[nodiscard]] units::AngularVelocity velocity() const override {
        const std::int32_t rate = sensor_.get_velocity();
        if (rate == PROS_ERR) {
            faultedReads_ += 1;
            return lastVelocity_;
        }
        lastVelocity_ = rotationCentidegPerSecToCanonical(static_cast<double>(rate));
        return lastVelocity_;
    }

    /// How many reads were screened to last-good (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    ::pros::v5::Rotation sensor_;
    // `mutable`: the readers are const but the T7 hold-last-good screen must
    // remember state — the caches are part of the read, not commanded state.
    mutable units::AngleDim lastPosition_{0.0};
    mutable units::AngularVelocity lastVelocity_{0.0};
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
