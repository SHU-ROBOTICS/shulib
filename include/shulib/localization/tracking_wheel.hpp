#pragma once
//
// TrackingWheel — one unpowered odometry wheel: an `IRotation` sensor + the wheel's diameter
// + its mounting offset from the tracking center. It turns cumulative shaft rotation into
// LINEAR travel and hands `PilonsOdometry` the two things it needs per tick: the travel delta
// since the last read, and the wheel's signed offset. (The PROS-free analogue of the legacy
// `OdomUnit`, driven by the HAL so it is host-testable with `FakeRotation`.)
//
// Travel = shaft angle (radians) × wheel radius. `IRotation::position()` is CUMULATIVE and
// unwrapped (it is `AngleDim`, not the wrapping `Angle`), so the running difference is the true
// signed distance the wheel has rolled — including direction reversals — with no seam to handle.
// (Binding contract: the hal/pros adapter MUST source this from the cumulative reading, e.g.
// `pros::Rotation::get_position`, NOT a wrapping 0–360 angle — the same get_rotation-vs-get_heading
// distinction the IMU has. The int32 centidegree range is ~6×10⁴ revolutions ≈ miles of travel,
// far beyond a match, so no wrap is seen in practice. See master plan §7.)
//
// ROLE + OFFSET. The offset's reference axis DIFFERS by role, so a wheel is built through a named
// factory that stamps the role and documents the axis — you cannot set the wrong axis or pass the
// wheels to PilonsOdometry in the wrong order (it checks the role):
//   * forward(): a FORWARD-rolling wheel (+X body); its offset is the +Y (LEFT) coordinate.
//   * lateral(): a LATERAL-rolling wheel (+Y body); its offset is the +X (FORWARD) coordinate.
// This is the perpendicular-to-rolling distance — the only offset that matters, because rolling
// along the wheel's own axis is what a turn-in-place sweeps. (Derivation + signs live in
// PilonsOdometry; verified by /tmp/odom_oracle.py and the pure-rotation tests.)
//
// Finiteness: TrackingWheel does no sentinel screening — it trusts the HAL finiteness contract
// (§7: the hal/pros adapter clamps PROS_ERR/NaN at the edge, so the core never sees non-finite).
// PilonsOdometry adds a last-resort guard so a contract breach can't poison the persistent pose.

#include "shulib/core/check.hpp"
#include "shulib/hal/rotation.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

class TrackingWheel {
public:
    enum class Role { Forward, Lateral };

    /// A FORWARD-rolling wheel (+X body). `leftOffset` is its +Y (LEFT) coordinate from center.
    [[nodiscard]] static TrackingWheel forward(hal::IRotation& sensor, units::Length wheelDiameter,
                                               units::Length leftOffset) {
        return TrackingWheel{sensor, wheelDiameter, leftOffset, Role::Forward};
    }

    /// A LATERAL-rolling wheel (+Y body). `forwardOffset` is its +X (FORWARD) coordinate.
    [[nodiscard]] static TrackingWheel lateral(hal::IRotation& sensor, units::Length wheelDiameter,
                                               units::Length forwardOffset) {
        return TrackingWheel{sensor, wheelDiameter, forwardOffset, Role::Lateral};
    }

    /// Linear travel (inches) since the previous call. STATEFUL: it advances the baseline, so
    /// successive calls return successive deltas, never a cumulative total.
    [[nodiscard]] units::Length travelDelta() {
        const double shaft = sensor_.position().value();            // cumulative radians
        const double travel = (shaft - lastShaft_) * radiusInches_;  // arc length = Δθ·r
        lastShaft_ = shaft;
        return units::Length{travel};
    }

    /// The wheel's signed offset from the tracking center (perpendicular to its rolling axis).
    [[nodiscard]] units::Length offset() const noexcept { return offset_; }

    /// Whether this wheel rolls forward (+X body) or laterally (+Y body) — checked by PilonsOdometry.
    [[nodiscard]] Role role() const noexcept { return role_; }

    /// Resync the baseline to the current reading, so the next travelDelta() starts from zero
    /// (used when odometry is (re)initialized, so a pre-existing shaft total isn't counted).
    void reset() { lastShaft_ = sensor_.position().value(); }

private:
    TrackingWheel(hal::IRotation& sensor, units::Length wheelDiameter, units::Length offset, Role role)
        : sensor_{sensor}, radiusInches_{0.5 * wheelDiameter.value()}, offset_{offset}, role_{role} {
        SHULIB_PRECONDITION(wheelDiameter.value() > 0.0,
                            "TrackingWheel: wheel diameter must be > 0");
        lastShaft_ = sensor_.position().value();  // baseline = current reading at construction
    }

    hal::IRotation& sensor_;
    double radiusInches_;
    units::Length offset_;
    Role role_;
    double lastShaft_ = 0.0;  // cumulative shaft radians at the last travelDelta()/reset()
};

}  // namespace shulib::localization
