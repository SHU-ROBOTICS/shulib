#pragma once
//
// TrapezoidProfile — a trapezoidal motion profile (master plan §M2). For a move of signed
// `distance` under (maxVelocity, maxAcceleration), it gives the (position, velocity,
// acceleration) target at any time: ramp up at aMax to the cruise speed, cruise, ramp down
// to rest. If the move is too short to reach maxVelocity it degrades to a TRIANGLE (peak
// speed < maxVelocity, no cruise). The velocity target feeds Feedforward; the position
// target feeds the per-axis Pid. (S-curve is a later sibling.)
//
// Bare doubles, like the rest of control: the motion layer instantiates one per axis with
// matching units (distance, unit/s, unit/s²). sample(t) clamps t to [0, duration].

#include <cmath>

#include "shulib/core/check.hpp"

namespace shulib::control {

struct ProfileConstraints {
    double maxVelocity = 0.0;      // > 0
    double maxAcceleration = 0.0;  // > 0
};

struct ProfileState {
    double position = 0.0;
    double velocity = 0.0;
    double acceleration = 0.0;
};

class TrapezoidProfile {
public:
    TrapezoidProfile(double distance, const ProfileConstraints& c) {
        SHULIB_PRECONDITION(c.maxVelocity > 0.0, "TrapezoidProfile: maxVelocity must be > 0");
        SHULIB_PRECONDITION(c.maxAcceleration > 0.0, "TrapezoidProfile: maxAcceleration must be > 0");
        SHULIB_PRECONDITION(std::isfinite(distance), "TrapezoidProfile: distance must be finite");

        sign_ = (distance < 0.0) ? -1.0 : 1.0;
        distance_ = std::abs(distance);
        aMax_ = c.maxAcceleration;

        const double dAccelFull = c.maxVelocity * c.maxVelocity / (2.0 * c.maxAcceleration);
        if (distance_ >= 2.0 * dAccelFull) {  // long enough to cruise → trapezoid
            vPeak_ = c.maxVelocity;
            tAccel_ = c.maxVelocity / c.maxAcceleration;
            dAccel_ = dAccelFull;
            tCruise_ = (distance_ - 2.0 * dAccelFull) / c.maxVelocity;
        } else {  // too short → triangle (never reaches maxVelocity)
            vPeak_ = std::sqrt(distance_ * c.maxAcceleration);
            tAccel_ = vPeak_ / c.maxAcceleration;
            dAccel_ = distance_ / 2.0;
            tCruise_ = 0.0;
        }
        duration_ = 2.0 * tAccel_ + tCruise_;
    }

    [[nodiscard]] ProfileState sample(double t) const {
        if (t <= 0.0) {
            return scaled(0.0, 0.0, (duration_ > 0.0) ? aMax_ : 0.0);  // about to accelerate
        }
        if (t >= duration_) {
            return scaled(distance_, 0.0, 0.0);  // arrived, at rest
        }
        if (t < tAccel_) {  // accelerate
            return scaled(0.5 * aMax_ * t * t, aMax_ * t, aMax_);
        }
        if (t < tAccel_ + tCruise_) {  // cruise
            const double tc = t - tAccel_;
            return scaled(dAccel_ + vPeak_ * tc, vPeak_, 0.0);
        }
        // decelerate
        const double td = t - tAccel_ - tCruise_;
        const double pos = dAccel_ + vPeak_ * tCruise_ + (vPeak_ * td - 0.5 * aMax_ * td * td);
        return scaled(pos, vPeak_ - aMax_ * td, -aMax_);
    }

    [[nodiscard]] double duration() const noexcept { return duration_; }
    [[nodiscard]] bool isDone(double t) const noexcept { return t >= duration_; }

private:
    [[nodiscard]] ProfileState scaled(double p, double v, double a) const noexcept {
        return ProfileState{sign_ * p, sign_ * v, sign_ * a};
    }

    double sign_ = 1.0;
    double distance_ = 0.0;
    double aMax_ = 0.0;
    double vPeak_ = 0.0;
    double tAccel_ = 0.0;
    double tCruise_ = 0.0;
    double dAccel_ = 0.0;
    double duration_ = 0.0;
};

}  // namespace shulib::control
