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

/// The three constants of V = kS·sign(v) + kV·v + kA·a, characterized OFFLINE by sysid — nothing
/// in this library tunes them. They describe ONE wheel, against that wheel's surface speed in
/// in/s. All-zero (the default) is legal and yields 0 V for every request, so an uncharacterized
/// drivetrain goes dead rather than wrong.
struct FeedforwardGains {
    /// Volts to break static friction. Applied at FULL magnitude with the sign of the commanded
    /// velocity and not at all at exactly v == 0, so the law steps by ±kS across zero.
    double kS = 0.0;
    /// Volt·s/in — volts per in/s of wheel surface speed (the back-EMF term; roughly the rail
    /// voltage divided by free speed).
    double kV = 0.0;
    /// Volt·s²/in — the inertia term. 0 (the default) leaves a steady-state-only feedforward:
    /// correct while cruising, and behind by the whole acceleration term on every ramp.
    double kA = 0.0;
};

/// The open-loop half of the control law: the voltage that should ALREADY hold a wheel at the
/// requested speed, so the PID beside it only has to correct the residual. Stateless and
/// clock-free — the same arguments always give the same volts, and nothing accumulates between
/// calls. One instance serves a whole drivetrain: the command pipeline applies it in turn to each
/// wheel's desaturated speed, because the gains describe a wheel, not a particular motor.
class Feedforward {
public:
    /// Copies `gains`; there is no setter, so re-characterizing means constructing a new
    /// Feedforward. Rejects a non-finite gain here, at setup, rather than letting a NaN reach a
    /// motor command later.
    explicit Feedforward(const FeedforwardGains& gains) : g_{gains} {
        SHULIB_PRECONDITION(std::isfinite(g_.kS) && std::isfinite(g_.kV) && std::isfinite(g_.kA),
                            "Feedforward: gains must be finite");
    }

    /// V = kS·sign(v) + kV·v + kA·a for ONE wheel: `velocity` is that wheel's surface speed
    /// (in/s), `acceleration` its surface acceleration (in/s²), and the result is volts. Only the
    /// kS term follows sign(v) — the SUM need not, and is not meant to: on a hard deceleration
    /// kA·a outweighs kS + kV·v and the law asks for voltage AGAINST the direction of travel,
    /// which is the braking authority kA exists to supply (kS = 1, kV = 0.5, kA = 0.1 at
    /// v = +10 in/s, a = −100 in/s² gives −4 V). Deliberately UNBOUNDED — the result routinely
    /// exceeds the rail on an aggressive request, and compensateForBattery() makes it commandable.
    [[nodiscard]] units::Voltage calculate(units::Velocity velocity,
                                           units::Acceleration acceleration) const {
        const double v = velocity.value();
        const double a = acceleration.value();
        const double s = (v > 0.0) ? g_.kS : (v < 0.0) ? -g_.kS : 0.0;  // kS in the direction of motion
        return units::Voltage{s + g_.kV * v + g_.kA * a};
    }

    /// Cruise form: the same law with acceleration = 0, i.e. the steady-state voltage that HOLDS
    /// the wheel at `velocity`. This is the overload the command pipeline uses, because
    /// WheelSpeeds carries speeds only — there is no per-wheel acceleration channel to pass.
    [[nodiscard]] units::Voltage calculate(units::Velocity velocity) const {
        return calculate(velocity, units::Acceleration{0.0});
    }

private:
    FeedforwardGains g_;
};

/// What compensateForBattery() returns: the voltage that may actually be commanded, plus whether
/// getting it there cost anything. The pair travels together on purpose — a clamped voltage that
/// arrives without its flag is indistinguishable from a request that simply was not very big.
struct CompensatedVoltage {
    units::Voltage voltage;  ///< `desired` clamped into ±battery; safe to hand to IMotor
    /// True when the request could NOT be delivered as asked: |desired| exceeded the battery and
    /// was cut down — the drive is voltage-starved, not merely slow — or `desired` was non-finite,
    /// in which case `voltage` is non-finite too and nothing about it is trustworthy. The test is
    /// written `!(|d| <= b)` rather than `|d| > b` precisely so NaN lands on the true side: it
    /// used to read CLEAN for a NaN, which is this struct claiming a value is inside the battery
    /// envelope when it is not a value at all. Nothing in the library acts on this today (the
    /// command pipeline reads only `voltage`, and screens it at the motor edge through
    /// diag::recoverWheelVoltage); it is the channel a caller reads to tell those cases apart.
    /// Defaulted false, so a default-constructed CompensatedVoltage does not hold an
    /// indeterminate safety flag.
    bool brownoutLimited = false;
};

/// Limit `desired` to what `battery` can deliver (±battery), flagging saturation.
[[nodiscard]] inline CompensatedVoltage compensateForBattery(units::Voltage desired,
                                                             units::Voltage battery) {
    SHULIB_PRECONDITION(battery.value() >= 0.0, "compensateForBattery: battery voltage must be >= 0");
    const double b = battery.value();
    const double d = desired.value();
    // `!(|d| <= b)`, NOT `|d| > b`: both comparisons are false for NaN, so the second spelling
    // reports a NaN as unsaturated. std::clamp likewise returns NaN unchanged, and that is
    // deliberate — the non-finite value must reach diag::recoverWheelVoltage at the motor edge,
    // which is where this library recovers (zero it, raise Implausible) rather than here. A
    // throwing precondition would convert a hostile-sensor pathology into an aborted motion.
    return CompensatedVoltage{units::Voltage{std::clamp(d, -b, b)}, !(std::abs(d) <= b)};
}

}  // namespace shulib::control
