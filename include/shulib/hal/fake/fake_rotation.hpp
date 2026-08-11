#pragma once
//
// FakeRotation — a deterministic IRotation for host tests. Injectable cumulative
// position + velocity, so tracking-wheel odometry can be driven with exact numbers.

#include "shulib/hal/rotation.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeRotation final : public IRotation {
public:
    [[nodiscard]] units::AngleDim position() const override { return position_; }
    [[nodiscard]] units::AngularVelocity velocity() const override { return velocity_; }

    void setPosition(units::AngleDim p) { position_ = p; }
    void setVelocity(units::AngularVelocity v) { velocity_ = v; }

private:
    units::AngleDim position_{};
    units::AngularVelocity velocity_{};
};

}  // namespace shulib::hal::fake
