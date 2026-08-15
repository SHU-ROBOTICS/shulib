#pragma once
//
// TrapezoidProfile — a trapezoidal motion profile (master plan §M2). For a move of signed
// `distance` under (maxVelocity, maxAcceleration), it gives the (position, velocity,
// acceleration) target at any time: ramp up at aMax to the cruise speed, cruise, ramp down
// to rest. If the move is too short to reach maxVelocity it degrades to a TRIANGLE (peak
// speed < maxVelocity, no cruise). (S-curve is a later sibling.)
//
// NOTHING IN THE TREE INSTANTIATES ONE YET, and this note is here because the header used
// to say the opposite. It claimed "the velocity target feeds Feedforward; the position
// target feeds the per-axis Pid" and "the motion layer instantiates one per axis" — both
// describe an INTENDED wiring, not a real one. The C1 motions run per-axis Pid plus
// Feedforward directly against the live error, with the speed cap applied in the command
// pipeline; no profile is generated anywhere, and the only consumer of this class in the
// repository is its own test. That is a gap, not a bug — profiled motion is future work —
// but a header that describes an integration it does not have is a lie a generated
// reference then publishes, so it says the true thing now.
//
// Bare doubles, like the rest of control: the CALLER picks the distance unit and supplies
// matching unit/s and unit/s² (inches and radians are what a motion layer would use).
// sample(t) clamps t to [0, duration] — see its own note for what NaN does.

#include <cmath>

#include "shulib/core/check.hpp"

namespace shulib::control {

/// The envelope a profile must stay inside. Bare doubles by design: the CALLER picks the
/// distance unit and these are that unit per second and per second² — inches and radians
/// are what a motion layer would use, though none instantiates one today (header note).
struct ProfileConstraints {
    double maxVelocity = 0.0;      ///< Cruise-speed cap; must be > 0 (the 0 default is unusable)
    double maxAcceleration = 0.0;  ///< Ramp rate, used for BOTH ramps (accel == decel); must be > 0
};

/// The profile's command at one instant. All three fields carry the SIGN of the move — a
/// negative `distance` mirrors every one of them.
struct ProfileState {
    /// Displacement FROM THE START of the move, not a field coordinate: 0 at t <= 0 and
    /// exactly `distance` at t >= duration(). The caller adds its own origin.
    double position = 0.0;
    double velocity = 0.0;      ///< Signed speed; 0 at both ends, never exceeds maxVelocity
    double acceleration = 0.0;  ///< +aMax on the up-ramp, 0 while cruising, -aMax on the down-ramp
};

/// A one-axis motion plan: ramp up at maxAcceleration, cruise, ramp down to rest exactly
/// on target — degrading to a triangle when the move is too short to reach cruise speed.
/// Built once per move and then IMMUTABLE: sample(t) is a pure function of t, so the same
/// t always returns the same state, re-sampling is free, and nothing advances a baseline.
/// **It has no consumer in this library yet** — the shipped motions servo the live error
/// with Pid + Feedforward and generate no profile. Shipped, tested, and waiting for the
/// profiled-motion work; the header note says why that is written down rather than implied.
class TrapezoidProfile {
public:
    /// Plan a move of SIGNED `distance` under `c`. `distance` and both constraints must be
    /// FINITE, and the constraints strictly positive; a violation trips SHULIB_PRECONDITION
    /// being clamped, because a silently corrected limit is a plan nobody asked for. The
    /// finiteness of the constraints used to be unchecked, and `> 0.0` is satisfied by
    /// infinity: `maxAcceleration = inf` was stored raw as aMax_ and handed straight back out
    /// of sample() as a non-finite acceleration target. If
    /// the move is too short to reach c.maxVelocity the plan degrades to a TRIANGLE (peak
    /// speed sqrt(|distance| * maxAcceleration), no cruise phase). A zero distance is legal
    /// and yields duration() == 0 — an already-finished plan, not an error.
    TrapezoidProfile(double distance, const ProfileConstraints& c) {
        SHULIB_PRECONDITION(std::isfinite(c.maxVelocity) && c.maxVelocity > 0.0,
                            "TrapezoidProfile: maxVelocity must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(c.maxAcceleration) && c.maxAcceleration > 0.0,
                            "TrapezoidProfile: maxAcceleration must be finite and > 0");
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

    /// The target state at `t` SECONDS AFTER THE MOVE STARTED — the caller owns the clock
    /// and the elapsed-time subtraction. `t` is CLAMPED, never rejected: t <= 0 returns
    /// rest at the start with acceleration already at ±aMax (the next instant is the
    /// up-ramp; 0 for a zero-distance move), and t >= duration() returns rest exactly on
    /// target, forever. Const and side-effect-free.
    /// A NON-FINITE `t` is REJECTED, not clamped — the one input that is a caller bug rather
    /// than a position on the plan's timeline. It used to fall through every comparison (each
    /// is false against NaN) into the decelerate branch and return a PARTIALLY finite state:
    /// position and velocity NaN, but acceleration a perfectly finite -aMax. A caller
    /// screening only `acceleration` passed it and forwarded a plausible-looking down-ramp
    /// downstream, which is the "plausible instead of visible" failure this library rejects
    /// everywhere else.
    [[nodiscard]] ProfileState sample(double t) const {
        SHULIB_PRECONDITION(std::isfinite(t), "TrapezoidProfile::sample: t must be finite");
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

    /// Total planned time in seconds, both ramps included (0 for a zero-distance move).
    /// This is the PLAN's time, not a promise the drivetrain tracks it — a follower's
    /// timeout must allow slack beyond this, not equal it.
    [[nodiscard]] double duration() const noexcept { return duration_; }

    /// True once `t` has reached duration(), inclusive — i.e. sample(t) has stopped
    /// changing. True at t == 0 for a zero-distance move. A statement about the PLAN's
    /// clock only: it says nothing about whether the robot actually arrived, which is
    /// SettledUtil's question, measured against the real estimate.
    ///
    /// A non-finite `t` is rejected here too, on the same rule as sample(). It used to
    /// return FALSE (every NaN comparison is false), so a follower loop terminating on
    /// isDone() would spin forever on a NaN clock instead of failing fast. **This member
    /// is deliberately NOT noexcept**, because the precondition handler throws and a
    /// noexcept frame would turn a caller bug into std::terminate; the drop is a breaking
    /// signature change by version.hpp's rule, taken because this class has no consumer in
    /// the tree but its own test and a hang is the worse failure.
    [[nodiscard]] bool isDone(double t) const {
        SHULIB_PRECONDITION(std::isfinite(t), "TrapezoidProfile::isDone: t must be finite");
        return t >= duration_;
    }

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
