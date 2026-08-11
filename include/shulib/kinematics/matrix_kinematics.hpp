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
// forward() (wheels → body twist, for odometry) is the FULL LEAST-SQUARES
// pseudo-inverse  t = (AᵀA)⁻¹Aᵀ·w  (chunk C3, discharging the M1 deferral so the
// H-drive's OFF-CENTRE strafe wheel — a non-orthogonal column — is supported).
// Because AᵀA is 3×3 symmetric it is inverted once, in closed form, at
// construction; forward() is then two small matrix multiplies per call.
//
// ── The strict-generalization guarantee (the C3 no-regression contract) ─────────────
// When the columns are mutually orthogonal (X-drive, symmetric mecanum — every
// table this class accepted before C3), AᵀA is diagonal and the pseudo-inverse
// REDUCES to the historical per-column projection
//
//     vx = (Σ h_i w_i)/Σh²,  vy = (Σ v_i w_i)/Σv²,  ω = (Σ turn_i w_i)/Σturn²
//
// forward() detects that case ONCE at construction — using the EXACT predicate
// the pre-C3 precondition used to accept tables — and runs the historical
// computation VERBATIM for it, so every previously-accepted drive is
// BIT-IDENTICAL to its pre-C3 numbers (pinned by XOR-of-bit-pattern checksums in
// test/matrix_kinematics_test.cpp, captured from the pre-C3 build). The general
// path serves only tables the old code REJECTED: a relaxed precondition, nothing
// more (F5-safe — signatures, toWheels(), desaturate(), strafeAuthority()
// untouched; decision-checked 2026-06-19, discharged 2026-08-06).
//
// ── Conditioning guard (the silent-garbage defence) ─────────────────────────────────
// Once orthogonality is no longer required, the per-column rank check below is
// NOT sufficient: three individually-nonzero columns can still be linearly
// dependent (e.g. two parallel wheel directions), making AᵀA singular — and a
// NEAR-degenerate table would pass any exact-singularity test yet amplify wheel
// noise by an unbounded factor in forward(). Construction therefore computes the
// RELATIVE GRAM DETERMINANT
//
//     relDet = det(AᵀA) / (Σh²·Σv²·Σturn²)   ∈ [0, 1]   (Hadamard's inequality;
//              1 ⟺ orthogonal columns, 0 ⟺ rank-deficient)
//
// and REJECTS relDet ≤ kMinRelativeDeterminant with a red-on-failure
// precondition rather than silently mis-inverting — the worst possible failure
// mode here is plausible-looking garbage odometry. relDet is scale-free (column
// units cancel), so a huge turn-lever column cannot mask a genuine geometric
// degeneracy. See kMinRelativeDeterminant below for the threshold's derivation.
//
// Tank is NOT a MatrixKinematics: it is rank-2 (cannot strafe), so a column is
// all-zero and the rank precondition correctly rejects it. Tank lives in its own
// dedicated TankKinematics. (This is unchanged by the pseudo-inverse: rank-3 is
// still required — the generalization admits non-ORTHOGONAL tables, never
// rank-DEFICIENT ones.)

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
    /// the table is genuinely rank-3 (each column non-degenerate AND the columns
    /// jointly well-conditioned — relDet > kMinRelativeDeterminant, header note).
    /// Orthogonal columns are NO LONGER required (C3's pseudo-inverse); they remain
    /// the well-trodden fast path.
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

        // Column inner products: AᵀA's diagonal (sumH2/sumV2/sumT2) and its
        // off-diagonals (hv/ht/vt). Together they are the whole 3×3 Gram matrix.
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

        // Diagnostic-specificity check: a dead column gets THIS actionable message
        // (name the tank escape hatch) instead of the generic conditioning one.
        // Honest status, established by mutation M10 at C3: the conditioning guard
        // below SUBSUMES this rejection (a zero column drives det to 0 — or relDet
        // to 0/0 = NaN, which also rejects), so this check is defence-in-depth
        // kept for its message, not the load-bearing rejection.
        SHULIB_PRECONDITION(
            sumH2_ > kRankEps && sumV2_ > kRankEps && sumT2_ > kRankEps,
            "MatrixKinematics: table is rank-deficient (a column is all-zero) -- not a "
            "fully-holonomic drive (tank belongs in TankKinematics)");

        // The pre-C3 acceptance predicate, VERBATIM: every table it accepts ran the
        // historical per-column projection before C3 and still does (the
        // bit-identity half of the strict-generalization guarantee, header note).
        orthogonal_ = std::abs(hv) <= kOrthoTol * std::sqrt(sumH2_ * sumV2_)
                      && std::abs(ht) <= kOrthoTol * std::sqrt(sumH2_ * sumT2_)
                      && std::abs(vt) <= kOrthoTol * std::sqrt(sumV2_ * sumT2_);

        // Gram determinant (symmetric 3×3, closed form) and the conditioning guard
        // (header note). Applies to EVERY table — for orthogonal ones relDet ≈ 1 and
        // the guard is trivially clear; for the rest it is what stands between an
        // ill-posed geometry and silently-garbage odometry. NaN inputs fail the
        // comparison and reject (the guard cannot be evaded by a poisoned table).
        const double det = sumH2_ * (sumV2_ * sumT2_ - vt * vt)
                           - hv * (hv * sumT2_ - vt * ht)
                           + ht * (hv * vt - sumV2_ * ht);
        const double relDet = det / (sumH2_ * sumV2_ * sumT2_);
        SHULIB_PRECONDITION(
            relDet > kMinRelativeDeterminant,
            "MatrixKinematics: coefficient columns are (near-)linearly dependent -- "
            "forward() would amplify wheel noise unboundedly; fix the wheel geometry "
            "(two wheel directions are indistinguishable to this table)");

        if (!orthogonal_) {
            // (AᵀA)⁻¹ via the adjugate — computed ONCE; forward() then costs two
            // small multiplies. Symmetric, so six unique entries.
            g00_ = (sumV2_ * sumT2_ - vt * vt) / det;
            g01_ = (ht * vt - hv * sumT2_) / det;
            g02_ = (hv * vt - ht * sumV2_) / det;
            g11_ = (sumH2_ * sumT2_ - ht * ht) / det;
            g12_ = (hv * ht - sumH2_ * vt) / det;
            g22_ = (sumH2_ * sumV2_ - hv * hv) / det;
        }

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
        double gh = 0.0, gv = 0.0, gt = 0.0;  // Aᵀ·w, common to both paths
        for (int i = 0; i < n_; ++i) {
            const double s = wheels[i].value();
            const Wheel& row = wheels_[static_cast<std::size_t>(i)];
            gh += row.h * s;
            gv += row.v * s;
            gt += row.turnInches * s;
        }
        if (orthogonal_) {
            // Historical per-column projection, VERBATIM — for an orthogonal table
            // (AᵀA)⁻¹ is exactly diag(1/Σh², 1/Σv², 1/Σturn²), and running the
            // pre-C3 expression (a division, not a reciprocal-multiply) keeps every
            // previously-accepted drive BIT-IDENTICAL to its pre-C3 numbers
            // (header: the strict-generalization guarantee; pinned by checksum).
            return math::Twist2d{units::Velocity{gh / sumH2_},
                                 units::Velocity{gv / sumV2_},
                                 units::AngularVelocity{gt / sumT2_}};  // radian re-attached
        }
        // General least squares: t = (AᵀA)⁻¹·(Aᵀw). For a square full-rank table
        // (the 3-wheel H-drive) this is exactly A⁻¹w; for redundant non-orthogonal
        // tables it is the unique minimizer of ‖A·t − w‖ (normal-equation
        // certificate pinned by test).
        return math::Twist2d{units::Velocity{g00_ * gh + g01_ * gv + g02_ * gt},
                             units::Velocity{g01_ * gh + g11_ * gv + g12_ * gt},
                             units::AngularVelocity{g02_ * gh + g12_ * gv + g22_ * gt}};
    }

    [[nodiscard]] WheelSpeeds desaturate(const WheelSpeeds& wheels,
                                         units::Velocity maxWheelSpeed) const override {
        return desaturateUniform(wheels, maxWheelSpeed);
    }

    [[nodiscard]] double strafeAuthority() const override { return strafeAuthority_; }
    [[nodiscard]] int wheelCount() const override { return n_; }

private:
    static constexpr double kRankEps = 1e-9;
    /// The pre-C3 orthogonality tolerance, kept VERBATIM as the fast-path
    /// predicate (bit-identity for every table the old precondition accepted).
    static constexpr double kOrthoTol = 1e-9;
    /// Conditioning floor on the relative Gram determinant (header note).
    /// Derivation: relDet is det of the column-NORMALIZED Gram matrix Ĝ (unit
    /// diagonal, eigenvalues λ₁≥λ₂≥λ₃>0 with Σλ=3, so λ₁λ₂ ≤ 9/4); relDet ≥ 1e-6
    /// forces λ₃ ≥ relDet/(λ₁λ₂) ≥ 4.4e-7, i.e. κ(Ĝ) = λ₁/λ₃ ≤ ~7e6 — forward()
    /// keeps ≥ ~9-10 significant digits, ample for in/s odometry. Every physical
    /// drive sits FAR above it (X-drive: 1.0; the H-bot: ~0.94); a table that
    /// trips this has two wheel directions the geometry cannot distinguish, which
    /// is a DESIGN error, not a tuning matter. Host-decidable pure-numerics
    /// constant — deliberately NOT an A4 register entry (register rule 1: the
    /// register is for hardware claims; same reasoning as C2's kMaxStalledPaces).
    static constexpr double kMinRelativeDeterminant = 1e-6;

    std::array<Wheel, static_cast<std::size_t>(WheelSpeeds::kMaxWheels)> wheels_{};
    int n_ = 0;
    double sumH2_ = 0.0;
    double sumV2_ = 0.0;
    double sumT2_ = 0.0;
    double strafeAuthority_ = 0.0;
    bool orthogonal_ = true;
    // (AᵀA)⁻¹, symmetric — populated only for non-orthogonal tables (general path).
    double g00_ = 0.0, g01_ = 0.0, g02_ = 0.0;
    double g11_ = 0.0, g12_ = 0.0, g22_ = 0.0;
};

}  // namespace shulib::kinematics
