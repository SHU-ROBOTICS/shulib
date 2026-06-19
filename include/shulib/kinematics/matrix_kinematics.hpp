#pragma once
//
// MatrixKinematics — the coefficient-matrix engine for FULLY-HOLONOMIC LINEAR
// drives (the hybrid backend, §13 #15). A drivetrain becomes pure data: each
// wheel is a row [h, v, turnInches], and
//
//     wheel_i surface speed = h_i·vx + v_i·vy + turnInches_i·ω
//
// where the last term is the ONE sanctioned radian-drop (ω[rad/s]·lever[in] →
// in/s). h and v are dimensionless projection factors; turnInches is the wheel's
// yaw lever arm in inches.
//
// forward() (wheels → body twist, for odometry) is the ORTHOGONAL-COLUMN
// least-squares inverse:
//
//     vx = (Σ h_i w_i)/Σh²,  vy = (Σ v_i w_i)/Σv²,  ω = (Σ turn_i w_i)/Σturn²
//
// This is EXACT only when the three coefficient columns are mutually orthogonal —
// which holds for the symmetric holonomic drives (X-drive, symmetric mecanum).
// Construction REJECTS a non-orthogonal or rank-deficient table with a precondition
// (red-on-failure) rather than silently mis-inverting.
//
// TODO(M2 — H-drive): generalize forward() to the full (AᵀA)⁻¹Aᵀ pseudo-inverse so
// an off-center strafe wheel (a non-orthogonal column) is supported. That change
// only RELAXES the orthogonality precondition below — toWheels(), the F5
// signatures, and every existing drive's numbers are unchanged. (Decision-checked
// 2026-06-19: nothing past or future breaks.)
//
// Tank is NOT a MatrixKinematics: it is rank-2 (cannot strafe), so a column is
// all-zero and the rank precondition correctly rejects it. Tank lives in its own
// dedicated TankKinematics.

#include <array>
#include <cmath>
#include <cstddef>
#include <initializer_list>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/desaturate.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::kinematics {

class MatrixKinematics final : public IKinematics {
public:
    /// One wheel's contribution row. h, v are dimensionless; turnInches is the
    /// yaw lever arm in inches (signed). See the header formula.
    struct Wheel {
        double h;           // body +X (forward) coefficient, dimensionless
        double v;           // body +Y (left/strafe) coefficient, dimensionless
        double turnInches;  // yaw lever arm (inches)
    };

    /// Build from a per-wheel coefficient table + the drive's strafe authority.
    /// Preconditions (all red-on-failure): 1..kMaxWheels wheels; strafeAuthority ≥ 0;
    /// the table is rank-3 (each column non-degenerate) and has orthogonal columns.
    MatrixKinematics(std::initializer_list<Wheel> wheels, double strafeAuthority) {
        SHULIB_PRECONDITION(
            wheels.size() >= 1u && wheels.size() <= static_cast<std::size_t>(WheelSpeeds::kMaxWheels),
            "MatrixKinematics: wheel count must be in [1, kMaxWheels]");
        SHULIB_PRECONDITION(strafeAuthority >= 0.0, "MatrixKinematics: strafeAuthority must be >= 0");

        n_ = static_cast<int>(wheels.size());
        {
            int i = 0;
            for (const Wheel& row : wheels) {
                wheels_[static_cast<std::size_t>(i)] = row;
                ++i;
            }
        }

        // Column inner products: the diagonal (sumH2/sumV2/sumT2) drives forward();
        // the off-diagonals (hv/ht/vt) must vanish for the projection to be exact.
        double hv = 0.0, ht = 0.0, vt = 0.0;
        for (int i = 0; i < n_; ++i) {
            const Wheel& row = wheels_[static_cast<std::size_t>(i)];
            sumH2_ += row.h * row.h;
            sumV2_ += row.v * row.v;
            sumT2_ += row.turnInches * row.turnInches;
            hv += row.h * row.v;
            ht += row.h * row.turnInches;
            vt += row.v * row.turnInches;
        }

        SHULIB_PRECONDITION(
            sumH2_ > kRankEps && sumV2_ > kRankEps && sumT2_ > kRankEps,
            "MatrixKinematics: table is rank-deficient (a column is all-zero) -- not a "
            "fully-holonomic drive (tank belongs in TankKinematics)");
        SHULIB_PRECONDITION(
            std::abs(hv) <= kOrthoTol * std::sqrt(sumH2_ * sumV2_)
                && std::abs(ht) <= kOrthoTol * std::sqrt(sumH2_ * sumT2_)
                && std::abs(vt) <= kOrthoTol * std::sqrt(sumV2_ * sumT2_),
            "MatrixKinematics: coefficient columns are not orthogonal (forward() would "
            "mis-invert) -- non-orthogonal drives await the M2 pseudo-inverse");

        strafeAuthority_ = strafeAuthority;
    }

    [[nodiscard]] WheelSpeeds toWheels(const math::ChassisSpeeds& body) const override {
        const double vx = body.vx().value();
        const double vy = body.vy().value();
        const double omega = body.omega().value();  // rad/s — radian dropped at ·turnInches below
        WheelSpeeds out{n_};
        for (int i = 0; i < n_; ++i) {
            const Wheel& row = wheels_[static_cast<std::size_t>(i)];
            out.set(i, units::Velocity{row.h * vx + row.v * vy + row.turnInches * omega});
        }
        return out;  // NO clamping here (§13 #5)
    }

    [[nodiscard]] math::Twist2d forward(const WheelSpeeds& wheels) const override {
        SHULIB_PRECONDITION(wheels.size() == n_, "MatrixKinematics::forward: wheel-count mismatch");
        double gh = 0.0, gv = 0.0, gt = 0.0;
        for (int i = 0; i < n_; ++i) {
            const double s = wheels[i].value();
            const Wheel& row = wheels_[static_cast<std::size_t>(i)];
            gh += row.h * s;
            gv += row.v * s;
            gt += row.turnInches * s;
        }
        return math::Twist2d{units::Velocity{gh / sumH2_},
                             units::Velocity{gv / sumV2_},
                             units::AngularVelocity{gt / sumT2_}};  // radian re-attached
    }

    [[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels,
                                         units::Velocity maxWheelSpeed) const override {
        return desaturateUniform(wheels, maxWheelSpeed);
    }

    [[nodiscard]] double strafeAuthority() const override { return strafeAuthority_; }
    [[nodiscard]] int wheelCount() const override { return n_; }

private:
    static constexpr double kRankEps = 1e-9;
    static constexpr double kOrthoTol = 1e-9;

    std::array<Wheel, static_cast<std::size_t>(WheelSpeeds::kMaxWheels)> wheels_{};
    int n_ = 0;
    double sumH2_ = 0.0;
    double sumV2_ = 0.0;
    double sumT2_ = 0.0;
    double strafeAuthority_ = 0.0;
};

}  // namespace shulib::kinematics
