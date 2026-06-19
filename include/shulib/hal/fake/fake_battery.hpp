#pragma once
//
// FakeBattery — a deterministic IBattery for host tests. Injectable voltage +
// capacity, so brownout-compensation and graceful-end behavior can be driven by
// simulating a sagging battery.

#include "shulib/hal/battery.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeBattery final : public IBattery {
public:
    [[nodiscard]] units::Voltage voltage() const override { return voltage_; }
    [[nodiscard]] double capacity() const override { return capacity_; }

    void setVoltage(units::Voltage v) { voltage_ = v; }
    void setCapacity(double c) { capacity_ = c; }

private:
    units::Voltage voltage_{};
    double capacity_ = 0.0;
};

}  // namespace shulib::hal::fake
