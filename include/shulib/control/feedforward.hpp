#pragma once
//
// Feedforward — the kS/kV/kA motor feedforward (master plan §M2): the open-loop voltage to
// achieve a target velocity + acceleration, so the PID only has to correct the residual.
//
//   V = kS·sign(v) + kV·v + kA·a
//
// kS [volts] overcomes static friction in the direction of motion; kV [volt·s/in] is the
// back-EMF / velocity term; kA [volt·s²/in] is the inertia / acceleration term. Gains are
// bare doubles (characterized offline by sysid); inputs are TYPED (Velocity, Acceleration)
// and the output is a Voltage — typed at the boundary, like Pid.
//
// Voltage / brownout compensation: compensateForBattery() limits a desired voltage to what
// the battery can actually deliver (±battery) and flags when it saturated — so the motion
// layer knows it is voltage-starved and the guaranteed end-of-run park still fires as the
// battery collapses (§M2, §18). We command actual voltage (IMotor::setVoltage), so the only
// battery effect is this ceiling; the kV/kS/kA themselves are battery-independent.

#include <algorithm>
#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::control {

struct FeedforwardGains {
    double kS = 0.0;  // volts        (static friction)
    double kV = 0.0;  // volt·s/in    (velocity)
    double kA = 0.0;  // volt·s²/in   (acceleration)
};

class Feedforward {
public:
    explicit Feedforward(const FeedforwardGains& gains) : g_{gains} {
        SHULIB_PRECONDITION(std::isfinite(g_.kS) && std::isfinite(g_.kV) && std::isfinite(g_.kA),
                            "Feedforward: gains must be finite");
    }

    [[nodiscard]] units::Voltage calculate(units::Velocity velocity,
                                           units::Acceleration acceleration) const {
        const double v = velocity.value();
        const double a = acceleration.value();
        const double s = (v > 0.0) ? g_.kS : (v < 0.0) ? -g_.kS : 0.0;  // kS in the direction of motion
        return units::Voltage{s + g_.kV * v + g_.kA * a};
    }

    [[nodiscard]] units::Voltage calculate(units::Velocity velocity) const {
        return calculate(velocity, units::Acceleration{0.0});
    }

private:
    FeedforwardGains g_;
};

struct CompensatedVoltage {
    units::Voltage voltage;
    bool brownoutLimited;  // true when |desired| exceeded the battery and was clamped
};

/// Limit `desired` to what `battery` can deliver (±battery), flagging saturation.
[[nodiscard]] inline CompensatedVoltage compensateForBattery(units::Voltage desired,
                                                             units::Voltage battery) {
    SHULIB_PRECONDITION(battery.value() >= 0.0, "compensateForBattery: battery voltage must be >= 0");
    const double b = battery.value();
    const double d = desired.value();
    return CompensatedVoltage{units::Voltage{std::clamp(d, -b, b)}, std::abs(d) > b};
}

}  // namespace shulib::control
