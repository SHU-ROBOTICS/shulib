#pragma once
//
// WheelSpeeds — per-wheel linear *surface* speeds (in/s), in a drivetrain-defined
// wheel order (each IKinematics impl documents its own order).
//
// Why a fixed inline capacity (no heap):
//   * cheap and allocation-free on the V5 (this is produced every ~10ms tick),
//   * trivially value-typed for host tests.
// kMaxWheels is deliberately generous: a FROZEN contract (F5) should never need a
// version bump merely to gain a wheel. Current drives fit easily — tank=2, H=3,
// X/mecanum=4, swerve drive-speeds=4.
//
// The element type is units::Velocity, not a bare double, so the units wall (F3)
// reaches all the way to the motor edge: you cannot accidentally feed a length or
// a voltage in where a wheel speed belongs.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

/// A drivetrain's per-wheel linear SURFACE speeds in in/s (units::Velocity, never a bare
/// double — the units wall reaches the motor edge), indexed in that drivetrain's own
/// wheel order. Each IKinematics implementation documents its order; this type does not
/// know which one it is holding, so a set from one drivetrain means nothing to another.
/// A plain value type with fixed inline capacity and no heap, because one of these is
/// produced every control tick. The wheel COUNT is fixed at construction — there is no
/// push or resize, so you build a set of size n and set() into it.
class WheelSpeeds {
public:
    /// Hard upper bound on wheel count (see header note). Generous on purpose.
    static constexpr int kMaxWheels = 8;

    /// An empty set (size 0). Used as a default / accumulator seed.
    WheelSpeeds() = default;

    /// A zero-initialized set of `count` wheels. `count` must be in [0, kMaxWheels].
    explicit WheelSpeeds(int count) : n_{count} {
        SHULIB_PRECONDITION(count >= 0 && count <= kMaxWheels,
                            "WheelSpeeds: count must be in [0, kMaxWheels]");
    }

    /// Wheels in the set, fixed at construction: this IS the valid index range for
    /// operator[] and set(), and 0 for a default-constructed set. For anything
    /// IKinematics::toWheels() produced it equals that drivetrain's wheelCount().
    [[nodiscard]] int size() const noexcept { return n_; }

    /// The i-th wheel speed. Precondition: 0 <= i < size().
    [[nodiscard]] units::Velocity operator[](int i) const {
        SHULIB_PRECONDITION(i >= 0 && i < n_, "WheelSpeeds: index out of range");
        return v_[static_cast<std::size_t>(i)];
    }

    /// Set the i-th wheel speed. Precondition: 0 <= i < size().
    void set(int i, units::Velocity speed) {
        SHULIB_PRECONDITION(i >= 0 && i < n_, "WheelSpeeds: index out of range");
        v_[static_cast<std::size_t>(i)] = speed;
    }

    /// Largest |wheel speed| across the set (0 for an empty set). The quantity a
    /// uniform desaturation scales against.
    [[nodiscard]] units::Velocity maxMagnitude() const noexcept {
        double m = 0.0;
        for (int i = 0; i < n_; ++i) {
            m = std::max(m, std::abs(v_[static_cast<std::size_t>(i)].value()));
        }
        return units::Velocity{m};
    }

    /// Element-wise comparison within `tol`, which is an ABSOLUTE tolerance in in/s —
    /// not relative, and not a norm over the set: every wheel must agree on its own.
    /// Differing sizes compare UNEQUAL rather than tripping a precondition, so it is
    /// safe to call across drivetrains. Signs matter: a reversed wheel is not
    /// approximately the forward one. A comparison for tests and assertions, not an
    /// equivalence relation — tolerance comparison is not transitive.
    [[nodiscard]] bool approxEqual(const WheelSpeeds& o, double tol = 1e-9) const noexcept {
        if (n_ != o.n_) {
            return false;
        }
        for (int i = 0; i < n_; ++i) {
            const auto d = v_[static_cast<std::size_t>(i)].value()
                         - o.v_[static_cast<std::size_t>(i)].value();
            if (std::abs(d) > tol) {
                return false;
            }
        }
        return true;
    }

private:
    std::array<units::Velocity, static_cast<std::size_t>(kMaxWheels)> v_{};
    int n_ = 0;
};

}  // namespace shulib::kinematics
