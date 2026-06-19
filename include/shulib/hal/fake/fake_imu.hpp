#pragma once
//
// FakeImu — a deterministic IImu for host tests. It stores CANONICAL values (the
// conversion from the V5 frame is the adapter's job, tested separately in
// imu_conversion_test) and lets a test drive heading/yaw-rate/calibration/tilt —
// including the ±180° seam and a simulated drift — to exercise the layers above.

#include "shulib/hal/imu.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeImu final : public IImu {
public:
    [[nodiscard]] math::Angle heading() const override { return heading_; }
    [[nodiscard]] units::AngularVelocity yawRate() const override { return yawRate_; }
    [[nodiscard]] bool isCalibrating() const override { return calibrating_; }
    [[nodiscard]] math::Angle pitch() const override { return pitch_; }
    [[nodiscard]] math::Angle roll() const override { return roll_; }

    // --- test injection ---
    void setHeading(math::Angle h) { heading_ = h; }
    void setYawRate(units::AngularVelocity r) { yawRate_ = r; }
    void setCalibrating(bool c) { calibrating_ = c; }
    void setPitch(math::Angle p) { pitch_ = p; }
    void setRoll(math::Angle r) { roll_ = r; }

private:
    math::Angle heading_{};
    units::AngularVelocity yawRate_{};
    bool calibrating_ = false;
    math::Angle pitch_{};
    math::Angle roll_{};
};

}  // namespace shulib::hal::fake
