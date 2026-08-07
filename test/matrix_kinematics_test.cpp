// Engine-level tests for MatrixKinematics, using SYNTHETIC tables with
// hand-computable numbers — deliberately isolated from any real drivetrain
// geometry so an engine bug and a geometry bug can never hide inside each other.
// (The X-drive preset and its physical properties are tested separately; the
// H-drive preset — the drive the pseudo-inverse exists for — in h_drive_test.)
//
// The table used for the round-trip is intentionally ASYMMETRIC (Σh²=4, Σv²=16,
// Σturn²=4) so that a forward() that divides by the wrong column sum is caught —
// a symmetric table would mask that bug.
//
// ── The C3 pseudo-inverse block (bottom half of this file) ──────────────────────────
// forward() generalized to (AᵀA)⁻¹Aᵀ. The proof obligations, each its own case:
//   * STRICT GENERALIZATION — every previously-accepted (orthogonal) table is
//     BIT-IDENTICAL to its pre-C3 numbers, pinned by XOR-of-bit-pattern
//     checksums captured from the pristine pre-C3 build (commit 7fcb3d4);
//   * CORRECTNESS of the new path — exact inverse for square tables, and the
//     least-squares NORMAL-EQUATION certificate Aᵀ(A·t − w) = 0 for redundant
//     tables fed deliberately INCONSISTENT wheel speeds (uniquely characterizes
//     the pseudo-inverse — any wrong (AᵀA)⁻¹ fails it);
//   * CONDITIONING — near-degenerate geometry is REJECTED at construction, on
//     both sides of the documented relDet boundary, and the guard is scale-free
//     (a big turn-lever column is not degeneracy).

#include "doctest.h"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <initializer_list>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::kinematics::MatrixKinematics;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::WheelSpeeds;
using shulib::kinematics::xDrive;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Velocity;

namespace {

// A valid, orthogonal, rank-3, asymmetric synthetic drive (NOT physical).
MatrixKinematics synthetic() {
    return MatrixKinematics({{-1.0, +2.0, 1.0},
                             {-1.0, -2.0, 1.0},
                             {+1.0, -2.0, 1.0},
                             {+1.0, +2.0, 1.0}},
                            0.75);
}

WheelSpeeds wheels(std::initializer_list<double> vs) {
    WheelSpeeds w{static_cast<int>(vs.size())};
    int i = 0;
    for (double v : vs) {
        w.set(i++, Velocity{v});
    }
    return w;
}

}  // namespace

TEST_CASE("MatrixKinematics: a valid table constructs and reports its shape") {
    const MatrixKinematics k = synthetic();
    CHECK(k.wheelCount() == 4);
    CHECK(k.strafeAuthority() == doctest::Approx(0.75));
}

TEST_CASE("MatrixKinematics: toWheels applies the coefficient rows exactly") {
    const MatrixKinematics k = synthetic();
    // vx=2, vy=0.5, ω=-1  →  per row  h·vx + v·vy + turn·ω
    const WheelSpeeds w = k.toWheels(ChassisSpeeds{Velocity{2.0}, Velocity{0.5}, AngularVelocity{-1.0}});
    REQUIRE(w.size() == 4);
    CHECK(w[0].value() == doctest::Approx(-2.0));  // -2 +1 -1
    CHECK(w[1].value() == doctest::Approx(-4.0));  // -2 -1 -1
    CHECK(w[2].value() == doctest::Approx(0.0));   //  2 -1 -1
    CHECK(w[3].value() == doctest::Approx(2.0));   //  2 +1 -1
}

TEST_CASE("MatrixKinematics: forward inverts the coefficient rows exactly") {
    const MatrixKinematics k = synthetic();
    const Twist2d t = k.forward(wheels({-2.0, -4.0, 0.0, 2.0}));
    CHECK(t.vx().value() == doctest::Approx(2.0));
    CHECK(t.vy().value() == doctest::Approx(0.5));
    CHECK(t.omega().value() == doctest::Approx(-1.0));
}

TEST_CASE("MatrixKinematics: forward∘toWheels is identity across a swept grid") {
    const MatrixKinematics k = synthetic();
    for (int a = -3; a <= 3; ++a) {
        for (int b = -3; b <= 3; ++b) {
            for (int c = -3; c <= 3; ++c) {
                const ChassisSpeeds cmd{Velocity{static_cast<double>(a)},
                                        Velocity{static_cast<double>(b)},
                                        AngularVelocity{static_cast<double>(c)}};
                const Twist2d back = k.forward(k.toWheels(cmd));
                const Twist2d want{Velocity{static_cast<double>(a)},
                                   Velocity{static_cast<double>(b)},
                                   AngularVelocity{static_cast<double>(c)}};
                CHECK(back.approxEqual(want, 1e-9));
            }
        }
    }
}

TEST_CASE("MatrixKinematics: desaturate delegates to the uniform scale") {
    const MatrixKinematics k = synthetic();
    const WheelSpeeds out = k.desaturate(wheels({3.0, -12.0, 6.0, 0.0}), Velocity{6.0});
    CHECK(out.maxMagnitude().value() == doctest::Approx(6.0));  // peak 12 → scaled onto 6
    CHECK(out[0].value() == doctest::Approx(1.5));
}

// --- construction preconditions: malformed tables must throw, not mis-behave ---

TEST_CASE("MatrixKinematics: an empty table is rejected") {
    CHECK_THROWS_AS(MatrixKinematics({}, 1.0), PreconditionError);
}

TEST_CASE("MatrixKinematics: a rank-deficient table (dead column) is rejected") {
    // v column all-zero → cannot strafe → not fully holonomic. (This is a tank.)
    CHECK_THROWS_AS(MatrixKinematics({{-1.0, 0.0, 1.0}, {+1.0, 0.0, 1.0}}, 0.0),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: a rank-deficient-in-disguise table is rejected "
          "(2 wheels cannot span 3 twist axes)") {
    // Pre-C3 this threw on the ORTHOGONALITY precondition; post-C3 (pseudo-inverse)
    // non-orthogonality is legal, but this table is still rejected — and must be:
    // two wheel rows can never be rank-3 (det(AᵀA) = 0 exactly), and the
    // per-column check alone cannot see it (every column is nonzero). The
    // conditioning guard is what catches it. Bug caught: relaxing orthogonality
    // WITHOUT adding true-rank protection — the exact hole the relaxation opens.
    CHECK_THROWS_AS(MatrixKinematics({{1.0, 1.0, 0.0}, {1.0, 0.0, 1.0}}, 1.0),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: a negative strafeAuthority is rejected") {
    CHECK_THROWS_AS(MatrixKinematics({{-1.0, +2.0, 1.0},
                                      {-1.0, -2.0, 1.0},
                                      {+1.0, -2.0, 1.0},
                                      {+1.0, +2.0, 1.0}},
                                     -0.5),
                    PreconditionError);
}

TEST_CASE("MatrixKinematics: forward with the wrong wheel count is rejected") {
    const MatrixKinematics k = synthetic();  // expects 4
    CHECK_THROWS_AS((void)k.forward(wheels({1.0, 2.0, 3.0})), PreconditionError);
}

// ═══════════════════════════════════════════════════════════════════════════════════
// The C3 pseudo-inverse block (see file header for the proof-obligation map).
// ═══════════════════════════════════════════════════════════════════════════════════

namespace {

// The exact input sweep the pre-C3 goldens were captured over (scratch capture
// program, commit-7fcb3d4 headers, verified -O0 ≡ -O2). All values are exact
// binary fractions so no decimal-parsing ambiguity can enter the comparison.
constexpr double kSweepVals[] = {-37.25, -8.5, -1.0, 0.0, 0.5, 3.75, 12.0, 55.5};
constexpr int kSweepN = 8;

std::uint64_t bitsOf(double d) {
    std::uint64_t u = 0;
    std::memcpy(&u, &d, sizeof u);
    return u;
}

struct BitChecksum {
    std::uint64_t xvx = 0, xvy = 0, xw = 0;  // XOR of raw bit patterns: any
                                             // 1-ulp change anywhere flips it
    long count = 0;

    void fold(const Twist2d& t) {
        xvx ^= bitsOf(t.vx().value());
        xvy ^= bitsOf(t.vy().value());
        xw ^= bitsOf(t.omega().value());
        ++count;
    }
};

BitChecksum sweepForward4(const shulib::kinematics::IKinematics& k) {
    BitChecksum s;
    for (int a = 0; a < kSweepN; ++a) {
        for (int b = 0; b < kSweepN; ++b) {
            for (int c = 0; c < kSweepN; ++c) {
                for (int d = 0; d < kSweepN; ++d) {
                    WheelSpeeds w{4};
                    w.set(0, Velocity{kSweepVals[a]});
                    w.set(1, Velocity{kSweepVals[b]});
                    w.set(2, Velocity{kSweepVals[c]});
                    w.set(3, Velocity{kSweepVals[d]});
                    s.fold(k.forward(w));
                }
            }
        }
    }
    return s;
}

}  // namespace

// ── STRICT GENERALIZATION: previously-accepted tables are BIT-identical ────────────
// Bug caught: ANY numerical change to forward() on a pre-C3-accepted table — a
// 1-ulp drift from re-deriving the diagonal case through the general inverse, or
// the fast-path predicate accidentally routing an orthogonal table down the
// general path. Either would silently invalidate every recorded C1/C2 accuracy
// digit (the plant's truth flows through forward()). The golden checksums below
// were captured from the PRISTINE pre-C3 build over the identical sweep.
TEST_CASE("pseudo-inverse regression: forward() is BIT-IDENTICAL to pre-C3 on every "
          "previously-accepted table") {
    {
        const BitChecksum s = sweepForward4(xDrive(Length{7.0}));  // the motion-suite geometry
        REQUIRE(s.count == 4096);
        CHECK(s.xvx == 0x00000000000003e9ULL);
        CHECK(s.xvy == 0x00a68000000003d0ULL);
        CHECK(s.xw == 0xbf93db6db6db6db5ULL);
    }
    {
        const BitChecksum s = sweepForward4(xDrive(Length{10.0}));  // the unit-test geometry
        REQUIRE(s.count == 4096);
        CHECK(s.xvx == 0x00000000000003e9ULL);
        CHECK(s.xvy == 0x00a68000000003d0ULL);
        CHECK(s.xw == 0xbfcbfffffffffffeULL);
    }
    {
        const BitChecksum s = sweepForward4(synthetic());  // the engine-test table
        REQUIRE(s.count == 4096);
        CHECK(s.xvx == 0x0000000000000000ULL);
        CHECK(s.xvy == 0x0000000000000000ULL);
        CHECK(s.xw == 0xc01e600000000000ULL);
    }
    {
        // Tank shares no MatrixKinematics code, but the brief's no-regression
        // claim names it — pinned so an accidental edit cannot hide either.
        const TankKinematics tank{Length{12.0}};
        BitChecksum s;
        for (int a = 0; a < kSweepN; ++a) {
            for (int b = 0; b < kSweepN; ++b) {
                WheelSpeeds w{2};
                w.set(0, Velocity{kSweepVals[a]});
                w.set(1, Velocity{kSweepVals[b]});
                s.fold(tank.forward(w));
            }
        }
        REQUIRE(s.count == 64);
        CHECK(s.xvx == 0xc01e600000000000ULL);
        CHECK(s.xvy == 0x0000000000000000ULL);
        CHECK(s.xw == 0x0000000000000000ULL);
    }
    // One spot value for failure localization (exact hexfloat, same capture):
    {
        const auto k = xDrive(Length{7.0});
        const Twist2d t = k.forward(wheels({-37.25, 0.5, 12.0, 55.5}));
        CHECK(t.vx().value() == 0x1.26dd1027aa034p+5);
        CHECK(t.vy().value() == 0x1.04371d9ab72ffp+1);
        CHECK(t.omega().value() == 0x1.1924924924925p+0);
    }
}

// ── The relaxation is REAL: a non-orthogonal table constructs and inverts ──────────
// Bug caught: the precondition not actually relaxed, or a general path that is
// not the true inverse (a square rank-3 table has exactly one).
TEST_CASE("pseudo-inverse: a non-orthogonal square table constructs and round-trips "
          "exactly") {
    // v·turn = 4 ≠ 0 (the H-drive's own non-orthogonality shape, synthetic numbers).
    const MatrixKinematics k{{{1.0, 0.0, -5.5}, {1.0, 0.0, +5.5}, {0.0, 1.0, 4.0}}, 0.5};
    CHECK(k.wheelCount() == 3);
    for (int a = -3; a <= 3; ++a) {
        for (int b = -3; b <= 3; ++b) {
            for (int c = -3; c <= 3; ++c) {
                const ChassisSpeeds cmd{Velocity{static_cast<double>(a) * 7.0},
                                        Velocity{static_cast<double>(b) * 5.0},
                                        AngularVelocity{static_cast<double>(c) * 0.75}};
                const Twist2d back = k.forward(k.toWheels(cmd));
                CHECK(back.approxEqual(Twist2d{cmd.vx(), cmd.vy(), cmd.omega()}, 1e-9));
            }
        }
    }
}

// ── The least-squares certificate on INCONSISTENT inputs ───────────────────────────
// A redundant (4-wheel) non-orthogonal table fed wheel speeds OFF its column
// space (real encoders disagree — slip, noise). The pseudo-inverse is the UNIQUE
// t minimizing ‖A·t − w‖, characterized exactly by the normal equations
// Aᵀ(A·t − w) = 0. Bug caught: any wrong (AᵀA)⁻¹ — a transposed cofactor, a sign
// slip, a mis-derived adjugate entry — every one leaves a nonzero residual
// projection. (Round-trip tests alone CANNOT see these: they only exercise
// consistent inputs.)
TEST_CASE("pseudo-inverse: normal-equation residual is zero for inconsistent wheel "
          "speeds on a redundant non-orthogonal table") {
    // 4 wheels, deliberately lopsided: every Gram off-diagonal nonzero.
    const double rows[4][3] = {{1.0, 0.2, -6.0}, {1.0, -0.1, +5.0}, {0.3, 1.0, 3.5}, {0.1, 0.9, -2.0}};
    const MatrixKinematics k{{{rows[0][0], rows[0][1], rows[0][2]},
                              {rows[1][0], rows[1][1], rows[1][2]},
                              {rows[2][0], rows[2][1], rows[2][2]},
                              {rows[3][0], rows[3][1], rows[3][2]}},
                             0.4};
    shulib::sim::Rng rng{20260806ULL};
    for (int trial = 0; trial < 200; ++trial) {
        CAPTURE(trial);
        WheelSpeeds w{4};
        for (int i = 0; i < 4; ++i) {
            w.set(i, Velocity{rng.uniform(-60.0, 60.0)});  // arbitrary — almost surely
                                                           // off the column space
        }
        const Twist2d t = k.forward(w);
        // residual r_i = (A·t)_i − w_i, then project back through Aᵀ:
        double ph = 0.0, pv = 0.0, pt = 0.0;
        for (int i = 0; i < 4; ++i) {
            const double ri = rows[i][0] * t.vx().value() + rows[i][1] * t.vy().value()
                              + rows[i][2] * t.omega().value() - w[i].value();
            ph += rows[i][0] * ri;
            pv += rows[i][1] * ri;
            pt += rows[i][2] * ri;
        }
        REQUIRE(std::abs(ph) < 1e-9);
        REQUIRE(std::abs(pv) < 1e-9);
        REQUIRE(std::abs(pt) < 1e-9);
    }
}

// ── Seeded random-table sweep: construct + round-trip across the config space ──────
// Bug caught: a general-path failure that only appears for some table shape the
// hand-picked cases missed (the sweep is the net under the named cases).
TEST_CASE("pseudo-inverse: seeded random well-conditioned tables all round-trip") {
    shulib::sim::Rng rng{424242ULL};
    int accepted = 0;
    for (int trial = 0; trial < 400; ++trial) {
        CAPTURE(trial);
        const int n = 3 + static_cast<int>(rng.uniform(0.0, 2.999));  // 3..5 wheels
        double rows[5][3];
        for (int i = 0; i < n; ++i) {
            rows[i][0] = rng.uniform(-1.5, 1.5);
            rows[i][1] = rng.uniform(-1.5, 1.5);
            rows[i][2] = rng.uniform(-8.0, 8.0);
        }
        // Construction may legitimately reject a randomly-degenerate draw; the
        // sweep only asserts about tables the guard ACCEPTS (and counts them, so
        // the case cannot go vacuous by rejecting everything).
        bool built = false;
        try {
            MatrixKinematics k = [&] {
                switch (n) {
                    case 3:
                        return MatrixKinematics{{{rows[0][0], rows[0][1], rows[0][2]},
                                                 {rows[1][0], rows[1][1], rows[1][2]},
                                                 {rows[2][0], rows[2][1], rows[2][2]}},
                                                0.5};
                    case 4:
                        return MatrixKinematics{{{rows[0][0], rows[0][1], rows[0][2]},
                                                 {rows[1][0], rows[1][1], rows[1][2]},
                                                 {rows[2][0], rows[2][1], rows[2][2]},
                                                 {rows[3][0], rows[3][1], rows[3][2]}},
                                                0.5};
                    default:
                        return MatrixKinematics{{{rows[0][0], rows[0][1], rows[0][2]},
                                                 {rows[1][0], rows[1][1], rows[1][2]},
                                                 {rows[2][0], rows[2][1], rows[2][2]},
                                                 {rows[3][0], rows[3][1], rows[3][2]},
                                                 {rows[4][0], rows[4][1], rows[4][2]}},
                                                0.5};
                }
            }();
            built = true;
            const ChassisSpeeds cmd{Velocity{rng.uniform(-50.0, 50.0)},
                                    Velocity{rng.uniform(-50.0, 50.0)},
                                    AngularVelocity{rng.uniform(-5.0, 5.0)}};
            const Twist2d back = k.forward(k.toWheels(cmd));
            // Tolerance scales with the conditioning the guard admits (~1e-10
            // relative at the floor); 1e-6 absolute on ≤50 in/s is conservative.
            REQUIRE(back.approxEqual(Twist2d{cmd.vx(), cmd.vy(), cmd.omega()}, 1e-6));
        } catch (const PreconditionError&) {
            REQUIRE_FALSE(built);  // only construction may throw, never forward()
        }
        accepted += built ? 1 : 0;
    }
    REQUIRE(accepted > 300);  // the sweep is not vacuous (random tables are
                              // overwhelmingly well-conditioned)
}

// ── Conditioning: both sides of the documented boundary ────────────────────────────
// The near-parallel family rows {1, 1+ε, 0}, {1, 1, 1}, {0, ε, 1} has
// relDet ≈ 1.25e-3·ε² (measured: 1.24e-5 at ε=1e-2, 1.25e-7 at ε=1e-3), so the
// 1e-6 floor separates the two decades. Bug caught: a guard that rejects
// everything, accepts everything, or sits at the wrong magnitude — and a general
// path that silently returns garbage NEAR the boundary instead of being accurate
// right up to it.
TEST_CASE("pseudo-inverse conditioning: near-degenerate tables are REJECTED, "
          "boundary-accurate ones still invert") {
    // Exactly parallel h/v columns: det(AᵀA) = 0. Must throw, never mis-invert.
    CHECK_THROWS_AS(MatrixKinematics({{1.0, 2.0, 0.0}, {2.0, 4.0, 1.0}, {0.5, 1.0, 1.0}}, 1.0),
                    PreconditionError);
    // ε = 1e-3 → relDet ≈ 1.25e-7 < 1e-6: rejected.
    {
        const double e = 1e-3;
        CHECK_THROWS_AS(MatrixKinematics({{1.0, 1.0 + e, 0.0}, {1.0, 1.0, 1.0}, {0.0, e, 1.0}}, 1.0),
                        PreconditionError);
    }
    // ε = 1e-2 → relDet ≈ 1.24e-5 > 1e-6: accepted AND still accurate.
    {
        const double e = 1e-2;
        const MatrixKinematics k{{{1.0, 1.0 + e, 0.0}, {1.0, 1.0, 1.0}, {0.0, e, 1.0}}, 1.0};
        const ChassisSpeeds cmd{Velocity{12.0}, Velocity{-30.0}, AngularVelocity{2.5}};
        const Twist2d back = k.forward(k.toWheels(cmd));
        CHECK(back.approxEqual(Twist2d{cmd.vx(), cmd.vy(), cmd.omega()}, 1e-6));
    }
    // A NaN coefficient cannot slip past the guards (NaN fails every comparison
    // the right way round: the preconditions REJECT).
    CHECK_THROWS_AS(MatrixKinematics({{1.0, 0.0, -5.0},
                                      {1.0, std::numeric_limits<double>::quiet_NaN(), 5.0},
                                      {0.0, 1.0, 2.0}},
                                     0.5),
                    PreconditionError);
}

// ── Conditioning guard is SCALE-FREE ───────────────────────────────────────────────
// Bug caught: a guard on the RAW determinant (or raw eigenvalues), which a
// legitimately large turn-lever column (inches vs dimensionless h/v) would
// dominate — rejecting good geometry or masking bad. relDet normalizes per
// column, so only the GEOMETRY (angles between columns) matters.
TEST_CASE("pseudo-inverse conditioning: column scale disparity is not degeneracy") {
    // Same geometry as the accepted H-shape table, turn levers ×40 (a 220 in
    // "track" — absurd but numerically legitimate): must construct + invert.
    const MatrixKinematics k{{{1.0, 0.0, -220.0}, {1.0, 0.0, +220.0}, {0.0, 1.0, 160.0}}, 0.5};
    const ChassisSpeeds cmd{Velocity{20.0}, Velocity{10.0}, AngularVelocity{0.25}};
    const Twist2d back = k.forward(k.toWheels(cmd));
    CHECK(back.approxEqual(Twist2d{cmd.vx(), cmd.vy(), cmd.omega()}, 1e-8));
}
