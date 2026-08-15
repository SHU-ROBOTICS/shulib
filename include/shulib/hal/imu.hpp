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

/// The inertial sensor seam, reporting in shulib's CANONICAL frame: CCW-positive radians with
/// +X = 0. The V5's clockwise-degrees convention is converted away once, inside the PROS adapter,
/// so no code above this line ever converts an angle again. heading() is the load-bearing
/// sub-degree quantity the whole accuracy claim rests on; every reading is untrustworthy until
/// isReady() returns true. Implementations must not throw and must return finite values.
class IImu {
public:
    /// All defaulted, and the fact worth carrying away is a lifetime one rather than a language
    /// one: an implementation is REFERENCED and never owned anywhere in this tree — RobotContext
    /// keeps a non-owning `IImu*`, and Localizer, PilonsOdometry and both correctors each hold an
    /// `IImu&` — so the adapter you construct must outlive every one of them. The destructor is
    /// virtual only so that owning one through an `IImu*` would still be well-defined; declaring
    /// it is what forces the copy and move members to be re-defaulted here.
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

    /// True once the IMU is calibrated and its readings are trustworthy (false during boot
    /// calibration). POSITIVE polarity by convention — every HAL health predicate reads
    /// true = usable (cf. IGps::hasFix), so `if (imu.isReady())` can never read backwards.
    [[nodiscard]] virtual bool isReady() const = 0;

    /// Chassis pitch and roll (canonical, for tip detection).
    [[nodiscard]] virtual math::Angle pitch() const = 0;
    /// Chassis roll, as a wrapped math::Angle. Unlike heading(), the SIGN is NOT yet a settled
    /// convention: the PROS adapter passes the sensor's as-mounted sign through unnegated
    /// (open hardware assumption HA-110), so consume the MAGNITUDE until a bench measurement
    /// fixes it. That is enough for the tip detection this exists for.
    [[nodiscard]] virtual math::Angle roll() const = 0;
};

}  // namespace shulib::hal
