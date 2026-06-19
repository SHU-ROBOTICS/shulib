#pragma once
//
// IImu — the inertial sensor behind the HAL, reporting in shulib's CANONICAL frame
// (the V5's clockwise/degrees convention is converted away in the hal/pros adapter
// via imu_conversion.hpp, so everything above this line is CCW-positive radians).
//
// heading() is the load-bearing < 1° quantity. yawRate() feeds the fused estimate.
// isCalibrating() gates trust at boot (readings are garbage until calibration ends).
// pitch()/roll() exist for tip detection (master plan M2).

#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IImu {
public:
    virtual ~IImu() = default;
    IImu() = default;
    IImu(const IImu&) = default;
    IImu(IImu&&) = default;
    IImu& operator=(const IImu&) = default;
    IImu& operator=(IImu&&) = default;

    /// Canonical field heading: CCW-positive, +X = 0, wrapped to (-π, π].
    [[nodiscard]] virtual math::Angle heading() const = 0;

    /// Canonical yaw rate (CCW-positive).
    [[nodiscard]] virtual units::AngularVelocity yawRate() const = 0;

    /// True while the IMU is performing boot calibration — readings are not yet trustworthy.
    [[nodiscard]] virtual bool isCalibrating() const = 0;

    /// Chassis pitch and roll (canonical, for tip detection).
    [[nodiscard]] virtual math::Angle pitch() const = 0;
    [[nodiscard]] virtual math::Angle roll() const = 0;
};

}  // namespace shulib::hal
