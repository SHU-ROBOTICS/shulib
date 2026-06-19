#pragma once
//
// IBattery — the V5 battery (pros::battery) behind the HAL. voltage() in canonical
// volts; capacity() in [0, 1]. Feeds voltage / brownout compensation (master plan
// §M2): the control layer scales motor commands by the measured battery voltage so a
// routine behaves the same on a full or a sagging battery, and the guaranteed
// end-of-run park still fires as the battery collapses.

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IBattery {
public:
    virtual ~IBattery() = default;
    IBattery() = default;
    IBattery(const IBattery&) = default;
    IBattery(IBattery&&) = default;
    IBattery& operator=(const IBattery&) = default;
    IBattery& operator=(IBattery&&) = default;

    /// Present battery voltage (canonical volts).
    [[nodiscard]] virtual units::Voltage voltage() const = 0;

    /// Present current draw (canonical amperes) — the I half of the DebugRecord battery V/I
    /// (§18.2); voltage × current is dimensionally Power.
    [[nodiscard]] virtual units::Current current() const = 0;

    /// Remaining capacity in [0, 1].
    [[nodiscard]] virtual double capacity() const = 0;
};

}  // namespace shulib::hal
