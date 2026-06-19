#pragma once
//
// IGps — the VEX GPS behind the HAL, reporting in shulib's CANONICAL frame (the VEX
// meters / clockwise-from-North convention is converted away in the hal/pros adapter
// via gps_conversion.hpp). pose() is the robot-CENTER pose (lever-arm corrected).
//
// rmsError() and hasFix() drive the fusion (§13 #4): the GPS feeds a gated nudge,
// never a snap; and Driving Skills has NO GPS strip, so when the sensor is off the
// strip (or error is high / disconnected) hasFix() is false and the estimator runs
// dead-reckon only. rmsError() sets the corrector's measurement noise R.

#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IGps {
public:
    virtual ~IGps() = default;
    IGps() = default;
    IGps(const IGps&) = default;
    IGps(IGps&&) = default;
    IGps& operator=(const IGps&) = default;
    IGps& operator=(IGps&&) = default;

    /// Canonical robot-CENTER pose (position + heading), lever-arm and frame corrected.
    /// When hasFix()==false the value is UNSPECIFIED but MUST be finite (no NaN/Inf) and
    /// MUST NOT throw — read it only as a stale last-known estimate; the fuser ignores it
    /// and dead-reckons (§13 #4). Callers MUST check hasFix() before trusting pose().
    [[nodiscard]] virtual math::Pose2d pose() const = 0;

    /// RMS position error (canonical Length) — drives the corrector's R; large when off-strip.
    [[nodiscard]] virtual units::Length rmsError() const = 0;

    /// True when the GPS currently has a usable fix (on the strip, error bounded, connected).
    /// False → the estimator must dead-reckon (ignore the GPS this tick).
    [[nodiscard]] virtual bool hasFix() const = 0;
};

}  // namespace shulib::hal
