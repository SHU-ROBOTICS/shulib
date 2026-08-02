// Adversarial tests for sim::advanceTruth — the plant's ground-truth integrator — and
// the two-sided arcStep-vs-truth comparison that A2's constraint 2 exists to enable.
//
// THE POINT OF THIS FILE (read truth_integrator.hpp's header first): truth is RK4,
// arcStep is the half-angle closed form, and they are algebraically independent. That
// independence is what makes "arcStep agrees with truth" a REAL test of arcStep — and
// what makes every Phase E odometry-vs-truth measurement meaningful.
//
// The oracle used below is the direct-integral SE(2) endpoint, coded HERE in the test
// (Δ = R(θ0)·[sinΔθ, 1−cosΔθ; …]·v/ω — the raw integral of the rotated body velocity,
// NOT arcStep's chord-times-average-heading form and NOT the test subject's RK4). Three
// implementations, three derivations; any pairwise disagreement is a real defect.
//
// THE INDEPENDENCE TRIPWIRE: arcStep is mathematically EXACT for a constant twist, so
// no agreement test can catch a truth integrator that secretly reuses it — they would
// agree to machine precision. The case that CAN catch it is a per-tick sweep beyond π:
// arcStep receives wrapped Angles, so 270° aliases to −90° before it even runs, while
// the unwrapped RK4 truth handles it exactly. Mutation check #1 (truth → arcStep) must
// turn "beyond arcStep's wrap horizon" red.

#include "doctest.h"

#include <cmath>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/localization/arc_step.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/truth_integrator.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::localization::arcStep;
using shulib::localization::FieldDelta;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Twist2d;
using shulib::math::robotToField;
using shulib::sim::advanceTruth;
using shulib::sim::TruthState;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
constexpr double kPi = Angle::kPi;

[[nodiscard]] Twist2d twist(double vx, double vy, double w) {
    return Twist2d{Velocity{vx}, Velocity{vy}, AngularVelocity{w}};
}

/// The direct-integral SE(2) endpoint for a constant body twist over dt, from
/// (0, 0, th0) — valid for ANY Δθ, including beyond ±π (dTheta is unwrapped).
/// Independent derivation: ∫R(θ0+ωt)·v dt evaluated termwise; ω = 0 falls back to
/// the straight-line limit. This is the oracle, so it is deliberately the "dumb"
/// integral form, not the chord form and not RK4. CONDITIONING CAVEAT (measured):
/// the (1−cos ω)/ω term cancels catastrophically as ω→0 (~1.5e-7 floor at ω≈1e-8),
/// so this oracle is only consulted at LARGE ω; small-ω agreement is checked
/// truth-vs-arcStep directly, where both forms are well-conditioned.
struct Endpoint {
    double dx, dy;
};
[[nodiscard]] Endpoint oracle(double vx, double vy, double w, double th0, double dt) {
    double bx, by;  // displacement in the θ0 frame
    if (w == 0.0) {
        bx = vx * dt;
        by = vy * dt;
    } else {
        const double dTheta = w * dt;
        const double s = std::sin(dTheta) / w;
        const double c = (1.0 - std::cos(dTheta)) / w;
        bx = vx * s - vy * c;
        by = vx * c + vy * s;
    }
    return Endpoint{bx * std::cos(th0) - by * std::sin(th0),
                    bx * std::sin(th0) + by * std::cos(th0)};
}
}  // namespace

// ── Straight lines: analytic distance, swept across headings (catches a handedness
// or axis swap in the RK4 integrand) ──
TEST_CASE("sim truth: straight-line travel lands at heading-rotated v*t, swept") {
    for (double deg : {0.0, 30.0, 90.0, 135.0, 200.0, 300.0, -45.0}) {
        const double th = deg * kPi / 180.0;
        const TruthState end = advanceTruth(TruthState{0.0, 0.0, th}, twist(20.0, -8.0, 0.0),
                                            Time{0.5}, 32);
        // Independent expectation: the F1 body→field rotation applied to (10, −4).
        const ChassisSpeeds field = robotToField(
            ChassisSpeeds{Velocity{20.0}, Velocity{-8.0}, AngularVelocity{0.0}},
            Angle::radians(th));
        CHECK(end.x == doctest::Approx(field.vx().value() * 0.5).epsilon(1e-12));
        CHECK(end.y == doctest::Approx(field.vy().value() * 0.5).epsilon(1e-12));
        CHECK(end.theta == th);  // no rotation commanded, none introduced
    }
}

// ── Pure rotation: position frozen, theta advances UNWRAPPED (can pass π) ──
TEST_CASE("sim truth: pure rotation leaves position untouched and does not wrap theta") {
    const TruthState end =
        advanceTruth(TruthState{3.0, -2.0, 1.0}, twist(0.0, 0.0, 2.0), Time{4.0}, 64);
    CHECK(end.x == doctest::Approx(3.0).epsilon(1e-12));
    CHECK(end.y == doctest::Approx(-2.0).epsilon(1e-12));
    CHECK(end.theta == doctest::Approx(1.0 + 8.0));  // 9 rad — far past π, NOT wrapped
    // ...but the Pose2d view wraps correctly.
    CHECK(end.pose().heading().approxEqual(Angle::radians(9.0), 1e-12));
}

// ── The classic quarter circle, from circle GEOMETRY (radius = v/ω): the same
// keystone the arcStep suite pins, now demanded of the independent truth ──
TEST_CASE("sim truth: quarter-circle CCW lands at (rho, rho) from circle geometry") {
    const double rho = 10.0;
    const double w = kPi / 2.0;              // 90° in 1 s
    const double v = rho * w;                // arc speed for that radius
    const TruthState end = advanceTruth(TruthState{}, twist(v, 0.0, w), Time{1.0}, 128);
    CHECK(end.x == doctest::Approx(rho).epsilon(1e-9));
    CHECK(end.y == doctest::Approx(rho).epsilon(1e-9));
    CHECK(end.theta == doctest::Approx(kPi / 2.0));
}

// ── RK4 order verification: halving h must shrink the endpooint error ~16× against
// the analytic oracle. Proves the documented accuracy budget is real, not assumed. ──
TEST_CASE("sim truth: RK4 converges at 4th order against the analytic endpoint") {
    // A deliberately hard step: 3 rad of rotation with holonomic velocity in one tick.
    const double vx = 40.0, vy = -15.0, w = 3.0, dt = 1.0;
    const Endpoint exact = oracle(vx, vy, w, 0.3, dt);
    auto errorWith = [&](int substeps) {
        const TruthState end =
            advanceTruth(TruthState{0.0, 0.0, 0.3}, twist(vx, vy, w), Time{dt}, substeps);
        return std::hypot(end.x - exact.dx, end.y - exact.dy);
    };
    const double e4 = errorWith(4);
    const double e8 = errorWith(8);
    REQUIRE(e4 > 0.0);  // the coarse error must be measurable, or the ratio proves nothing
    CHECK(e8 < e4 / 10.0);          // ~16× for a genuine 4th-order scheme; >10× required
    CHECK(errorWith(256) < 1e-9);   // the tolerance regime the agreement sweep relies on
}

// ── THE TWO-SIDED CHECK (constraint 2 / DoD): arcStep vs independent truth, swept
// across twists including near-zero ω (both sides of arcStep's 1e-9 guard), pure
// rotation, pure strafe, holonomic combinations, and large per-tick deltas.
// Tolerance: 1e-9 ABSOLUTE. Measured budget (probe, logged in A2-PROGRESS): RK4@256
// worst self-error across this exact sweep is 9.2e-11 — a 10× margin — so a miss
// here is arcStep's error, not the truth integrator's. (The naive integral oracle is
// NOT used here: its (1−cos ω)/ω form loses ~7 digits near ω→0 by cancellation —
// the very ill-conditioning arcStep's half-angle form exists to avoid.) ──
TEST_CASE("sim truth: arcStep agrees with the independent truth across a swept input space") {
    const double dt = 1.0;
    int compared = 0;
    for (double th0 : {0.0, 0.7, kPi / 2.0, 2.5, -2.0}) {
        for (double vx : {0.0, -12.0, 25.0}) {
            for (double vy : {0.0, 9.0, -18.0}) {
                for (double w : {0.0, 1e-12, 1e-10, 1e-8, 1e-6, 0.05, -0.4, 1.5, -2.7}) {
                    const TruthState end = advanceTruth(TruthState{0.0, 0.0, th0},
                                                       twist(vx, vy, w), Time{dt}, 256);
                    const FieldDelta d = arcStep(
                        {.forward = Length{vx * dt}, .lateral = Length{vy * dt}},
                        Angle::radians(th0), Angle::radians(th0 + w * dt));
                    CHECK(d.dx.value() == doctest::Approx(end.x).epsilon(1e-9).scale(1.0));
                    CHECK(d.dy.value() == doctest::Approx(end.y).epsilon(1e-9).scale(1.0));
                    ++compared;
                }
            }
        }
    }
    CHECK(compared == 5 * 3 * 3 * 9);  // the sweep actually ran (405 points, 810 asserts)
}

// ── THE INDEPENDENCE TRIPWIRE (mutation check #1's red): beyond arcStep's wrap
// horizon. A 270°-in-one-step arc has a well-defined truth endpoint; a truth
// integrator that reused arcStep would alias it to −90° and land somewhere else. ──
TEST_CASE("sim truth: handles a per-tick rotation beyond arcStep's wrap horizon exactly") {
    const double rho = 8.0;
    const double w = 3.0 * kPi / 2.0;  // 270° in 1 s — unrepresentable per-tick for arcStep
    const double v = rho * w;
    const TruthState end = advanceTruth(TruthState{}, twist(v, 0.0, w), Time{1.0}, 256);
    // Circle geometry: after 270° CCW from heading 0, the center-(0,ρ) circle puts the
    // robot at (−ρ, ρ), heading 3π/2 (unwrapped).
    CHECK(end.x == doctest::Approx(-rho).epsilon(1e-9));
    CHECK(end.y == doctest::Approx(rho).epsilon(1e-9));
    CHECK(end.theta == doctest::Approx(3.0 * kPi / 2.0));
    // And the aliased arcStep answer is genuinely ELSEWHERE — the demonstration that
    // this case separates the two integrators (what makes mutation #1 detectable).
    const FieldDelta aliased = arcStep({.forward = Length{v}, .lateral = Length{0.0}},
                                       Angle{}, Angle::radians(w));  // wraps to −90°
    CHECK(std::hypot(aliased.dx.value() - end.x, aliased.dy.value() - end.y) > 1.0);
}

// ── A full 2π in one step returns home with theta = 2π, not 0 (unwrapped state) ──
TEST_CASE("sim truth: a full-circle tick returns to the start with unwrapped theta 2*pi") {
    const TruthState end =
        advanceTruth(TruthState{}, twist(31.4, 0.0, 2.0 * kPi), Time{1.0}, 512);
    CHECK(end.x == doctest::Approx(0.0).scale(1.0).epsilon(1e-8));
    CHECK(end.y == doctest::Approx(0.0).scale(1.0).epsilon(1e-8));
    CHECK(end.theta == doctest::Approx(2.0 * kPi));
}

// ── Degenerates: zero twist, zero dt, contract rejections ──
TEST_CASE("sim truth: zero twist and zero dt change nothing; bad inputs are rejected") {
    const TruthState s{1.0, 2.0, 3.0};
    const TruthState zt = advanceTruth(s, twist(0.0, 0.0, 0.0), Time{5.0}, 32);
    CHECK(zt.x == 1.0);
    CHECK(zt.y == 2.0);
    CHECK(zt.theta == 3.0);
    const TruthState zdt = advanceTruth(s, twist(9.0, 9.0, 9.0), Time{0.0}, 32);
    CHECK(zdt.x == 1.0);
    CHECK(zdt.y == 2.0);
    CHECK(zdt.theta == 3.0);

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_THROWS_AS((void)advanceTruth(s, twist(1.0, 0.0, 0.0), Time{-0.1}, 32),
                    PreconditionError);
    CHECK_THROWS_AS((void)advanceTruth(s, twist(1.0, 0.0, 0.0), Time{nan}, 32),
                    PreconditionError);
    CHECK_THROWS_AS((void)advanceTruth(s, twist(1.0, 0.0, 0.0), Time{0.1}, 0),
                    PreconditionError);
}
