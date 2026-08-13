// Adversarial tests for EkfFusion — the 5-state SE(2) EKF fusion policy (chunk E4).
//
// ── THE TRAP THIS FILE IS BUILT AROUND ────────────────────────────────────────────────
// If the truth trajectory and the filter's prediction share a motion model, an error in
// that model cancels and a sweep proves nothing. That failure has now bitten C1, C3, C4,
// E2 and E3. So `Truth` below integrates the body twist EXACTLY, in closed form, as a
// circular arc:
//     Δp_body = [ sin(ωh)/ω · vx − (1−cos(ωh))/ω · vy ,
//                 (1−cos(ωh))/ω · vx + sin(ωh)/ω · vy ]      then rotated by R(θ₀)
// which is the exact integral of ṗ = R(θ₀+ωt)·v over the tick. The FILTER takes a
// first-order Euler step at the END heading. They agree only in the limit; at ω = 4 rad/s
// and h = 10 ms they differ in the fourth decimal every tick, which is precisely what
// makes the odometry stream real evidence rather than a rearrangement of the filter's own
// arithmetic. Nothing in this file computes a truth pose using anything the filter uses.
//
// ── THE SEAM DRIVER ───────────────────────────────────────────────────────────────────
// `SeamDriver` reproduces the three Localizer steps a fusion policy actually sees — the
// odometry delta re-expressed under the learned heading bias (STEP 2), the `fuse()` call
// (STEP 4), and the bias accumulation before the IMU re-stamp (STEP 5) — so these tests
// can script sensor streams directly. It is a MIRROR of `localizer.hpp`, not a substitute
// for it: `ekf_fusion_seam_test.cpp` drives the real Localizer, and if the two ever
// disagree that file is the one that is right.

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <span>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/ekf_fusion.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::diag::GateReason;
using shulib::localization::CorrectionProposal;
using shulib::localization::EkfFusion;
using shulib::localization::EkfFusionConfig;
using shulib::localization::FusionResult;
using shulib::localization::IFusionPolicy;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Acceleration;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kDeg = Angle::kPi / 180.0;
constexpr std::size_t kN = EkfFusion::kN;

// ── an independent truth integrator (see the file header) ────────────────────────────
struct Truth {
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    /// Exact closed-form arc for a constant body twist over `h` seconds.
    void advance(double vx, double vy, double w, double h) {
        double sw = 0.0;
        double cw = 0.0;
        if (std::abs(w) < 1e-12) {
            sw = h;        // lim ω→0 sin(ωh)/ω
            cw = 0.0;      // lim ω→0 (1−cos(ωh))/ω
        } else {
            sw = std::sin(w * h) / w;
            cw = (1.0 - std::cos(w * h)) / w;
        }
        const double bx = sw * vx - cw * vy;
        const double by = cw * vx + sw * vy;
        const double c = std::cos(th);
        const double s = std::sin(th);
        x += bx * c - by * s;
        y += bx * s + by * c;
        th = Angle::radians(th + w * h).radians();
    }
};

/// A tiny seeded generator, written here so the tests own their randomness and no sim
/// header is involved. splitmix64 + Box–Muller.
struct Rng {
    std::uint64_t s;
    explicit Rng(std::uint64_t seed) : s{seed * 0x9E3779B97F4A7C15ULL + 1} {}
    std::uint64_t next() {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    double uniform() {
        return static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0);
    }
    double gauss() {
        const double u1 = std::max(uniform(), 1e-12);
        const double u2 = uniform();
        return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * Angle::kPi * u2);
    }
};

/// Mirrors Localizer STEP 2 / STEP 4 / STEP 5 (file header).
struct SeamDriver {
    IFusionPolicy* policy = nullptr;
    double fx = 0.0;
    double fy = 0.0;
    double bias = 0.0;
    double predX = 0.0;
    double predY = 0.0;

    FusionResult step(double odx, double ody, double imuHeading,
                      std::span<const CorrectionProposal> ps, double h) {
        if (bias != 0.0) {  // STEP 2: re-express the odom delta under the learned bias (E3)
            const double cb = std::cos(bias);
            const double sb = std::sin(bias);
            const double rx = odx * cb - ody * sb;
            const double ry = odx * sb + ody * cb;
            odx = rx;
            ody = ry;
        }
        predX = fx + odx;
        predY = fy + ody;
        const Pose2d predicted{Length{predX}, Length{predY},
                               Angle::radians(imuHeading) + Angle::radians(bias)};
        const FusionResult fr = policy->fuse(predicted, ps, Time{h});
        if (fr.headingApplied && std::isfinite(fr.headingNudge.value())) {  // STEP 5
            bias += fr.headingNudge.value();
        }
        fx = fr.x.value();
        fy = fr.y.value();
        return fr;
    }
    [[nodiscard]] double publishedHeading(double imuHeading) const {
        return (Angle::radians(imuHeading) + Angle::radians(bias)).radians();
    }
};

[[nodiscard]] CorrectionProposal fix(double x, double y, double sigma, double conf = 0.8) {
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{x}, Length{y}, Angle{}};
    p.confidence = conf;
    p.positionStdDev = Length{sigma};
    return p;
}

[[nodiscard]] CorrectionProposal headingFix(double x, double y, double sigma, double headingRad,
                                            double conf = 0.8) {
    CorrectionProposal p = fix(x, y, sigma, conf);
    p.fieldPose = Pose2d{Length{x}, Length{y}, Angle::radians(headingRad)};
    p.providesHeading = true;
    return p;
}

/// An INDEPENDENT positive-definiteness test: a Cholesky factorization written here, not
/// exposed by the filter. A covariance that has stopped being positive-definite is broken
/// even when the pose it reports still looks sensible — every gate decision after that
/// point is made against a matrix that is no longer a distribution.
[[nodiscard]] bool isPositiveDefinite(const EkfFusion& f) {
    std::array<double, kN * kN> L{};
    for (std::size_t i = 0; i < kN; ++i) {
        for (std::size_t j = 0; j <= i; ++j) {
            double sum = f.covariance(i, j);
            for (std::size_t k = 0; k < j; ++k) {
                sum -= L[i * kN + k] * L[j * kN + k];
            }
            if (i == j) {
                if (!(sum > 0.0) || !std::isfinite(sum)) {
                    return false;
                }
                L[i * kN + i] = std::sqrt(sum);
            } else {
                L[i * kN + j] = sum / L[j * kN + j];
            }
        }
    }
    return true;
}

/// Exact symmetry, compared with `==` on doubles. The filter symmetrizes explicitly, so
/// exact equality is the correct bar: the Joseph products are NOT bit-symmetric on their
/// own, which is what makes this assertion able to see the symmetrization disappear.
[[nodiscard]] bool isExactlySymmetric(const EkfFusion& f) {
    for (std::size_t i = 0; i < kN; ++i) {
        for (std::size_t j = i + 1; j < kN; ++j) {
            if (f.covariance(i, j) != f.covariance(j, i)) {
                return false;
            }
        }
    }
    return true;
}

/// One of three scripted body twists. Deliberately different in character: a straight
/// drive (ω = 0, where the arc and the Euler step agree and the θ column of F is idle), a
/// hard spin-and-strafe (where they do not), and a stop-start profile (which is where a
/// velocity-lagging filter would show up).
[[nodiscard]] std::array<double, 3> scriptTwist(int which, int tick) {
    const int phase = (tick / 120) % 5;
    switch (which) {
        case 0:
            return {30.0, 0.0, 0.0};
        case 1:
            switch (phase) {
                case 1: return {0.0, 0.0, 4.0};
                case 2: return {18.0, -12.0, -1.5};
                case 3: return {-14.0, 6.0, 2.2};
                default: return {26.0, 4.0, 0.6};
            }
        default:
            switch (phase) {
                case 0: return {0.0, 0.0, 0.0};
                case 1: return {40.0, 0.0, 0.0};
                case 2: return {0.0, 0.0, 0.0};
                case 3: return {-40.0, 10.0, 1.0};
                default: return {8.0, -8.0, -0.4};
            }
    }
}

constexpr double kH = 0.01;

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a config value that makes the algebra degenerate slipping through — a zero
// gateSigma (nothing is ever accepted), a zero sigma anywhere (a division by zero inside
// the update), a negative process noise (a covariance that shrinks while dead-reckoning,
// which is the single most dangerous state this filter can be in).
TEST_CASE("EkfFusion: rejects an out-of-range config") {
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.gateSigma = 0.0}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.posNoisePerInch = -0.1}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.posNoiseRate = Velocity{0.0}}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.velNoise = Acceleration{0.0}}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.odomStdDev = Length{0.0}}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.headingStdDev = AngleDim{0.0}}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.initialPosStdDev = Length{0.0}}}),
                    PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.maxNudgeRate = Velocity{-1.0}}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.reinitRejectCount = 0}}), PreconditionError);
    CHECK_THROWS_AS((EkfFusion{EkfFusionConfig{.maxDt = 0.0}}), PreconditionError);
    CHECK_NOTHROW((EkfFusion{EkfFusionConfig{}}));
}

// Would catch: the filter treating its very first tick as a prediction. On the first call
// there is no previous answer to difference against, so `u` would be the whole absolute
// position read as one tick's travel — a robot starting at (100, 50) would be modelled as
// having teleported there at 10,000 in/s, and the velocity state would be poisoned for
// seconds afterwards.
TEST_CASE("EkfFusion: the first tick initialises and returns the prediction untouched") {
    EkfFusion f{};
    const Pose2d start{Length{100.0}, Length{50.0}, Angle::degrees(37.0)};
    const FusionResult r = f.fuse(start, {}, Time{kH});
    CHECK(r.x.value() == 100.0);  // exactly: nothing was applied
    CHECK(r.y.value() == 50.0);
    CHECK_FALSE(r.applied);
    CHECK_FALSE(r.gated);
    CHECK(f.state(EkfFusion::kVx) == 0.0);
    CHECK(f.state(EkfFusion::kVy) == 0.0);
    CHECK(f.resyncCount() == 0);
    CHECK(isPositiveDefinite(f));
}

// Would catch: a teleport being integrated as motion. `Localizer::setPose()` clears
// `hasLast_`, so the tick after a teleport arrives with dt == 0. If the filter treated
// that as a prediction interval it would divide the jump by zero; if it treated it as a
// normal tick it would believe the robot crossed the field in 10 ms and would spend the
// next second unwinding a fictitious velocity.
TEST_CASE("EkfFusion: a dt<=0 tick re-bases instead of predicting, and applies nothing") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 50; ++i) {
        d.step(0.2, 0.0, 0.0, {}, kH);
    }
    const double before = f.resyncCount();
    // the teleport: the Localizer would move fusedX_ and then hand dt == 0
    d.fx = 90.0;
    d.fy = -40.0;
    const std::array<CorrectionProposal, 1> ps{fix(90.0, -40.0, 1.0)};
    const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, 0.0);
    CHECK(r.x.value() == 90.0);  // untouched
    CHECK(r.y.value() == -40.0);
    CHECK_FALSE(r.applied);  // a zero-budget tick applies nothing, exactly as the other tier
    CHECK(f.resyncCount() == before + 1);
    CHECK(f.state(EkfFusion::kVx) == 0.0);
    CHECK(isPositiveDefinite(f));
    CHECK(isExactlySymmetric(f));
}

// Would catch: a loop stall being integrated as one enormous tick. dt above maxDt is the
// Localizer's own definition of an untrustworthy interval, and u/dt on a 400 ms tick is a
// velocity nobody measured.
TEST_CASE("EkfFusion: a dt above maxDt re-bases rather than integrating a stall") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 50; ++i) {
        d.step(0.2, 0.0, 0.0, {}, kH);
    }
    const std::uint32_t before = f.resyncCount();
    const FusionResult r = d.step(9.0, 0.0, 0.0, {}, 0.4);
    CHECK(f.resyncCount() == before + 1);
    CHECK(r.x.value() == doctest::Approx(d.predX));  // the odometry's answer is taken as-is
    CHECK_FALSE(r.applied);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// The structural invariants, across a parameter sweep
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: ANY loss of the covariance's defining properties — the symmetrization
// removed, the Joseph form replaced by a shortcut that is only valid at the optimal gain,
// a sign error in a Jacobian, a negative variance sneaking in through a subtraction. A
// filter whose P is no longer a covariance is broken even while its POSE still looks
// right, and the pose is the only thing anybody normally looks at.
//
// The sweep is over 3 trajectories x 4 seeds x 3 noise levels x 600 ticks, with fixes
// arriving at 20 Hz — so the gate, the clamp, the accept path and the reject path are all
// exercised inside it.
TEST_CASE("[sweep] EkfFusion: the covariance stays symmetric, positive-definite and bounded") {
    int checkedTicks = 0;
    double worstTrace = 0.0;
    for (int traj = 0; traj < 3; ++traj) {
        for (std::uint64_t seed = 1; seed <= 4; ++seed) {
            for (int level = 0; level < 3; ++level) {
                const double odomSigma = 0.002 * (1 << level);  // 0.002 / 0.004 / 0.008 in
                const double fixSigma = 0.5 * static_cast<double>(1 << level);
                Rng rng{seed * 977 + static_cast<std::uint64_t>(traj * 31 + level)};
                EkfFusion f{};
                SeamDriver d{&f};
                Truth t{};
                double imuDrift = 0.0;
                bool symmetric = true;
                bool definite = true;
                bool finite = true;
                for (int tick = 0; tick < 600; ++tick) {
                    const auto tw = scriptTwist(traj, tick);
                    const double px = t.x;
                    const double py = t.y;
                    t.advance(tw[0], tw[1], tw[2], kH);
                    imuDrift += 0.0002 * kH;  // a slow, one-sided IMU error
                    const double odx = (t.x - px) + odomSigma * rng.gauss();
                    const double ody = (t.y - py) + odomSigma * rng.gauss();
                    std::array<CorrectionProposal, 1> ps{
                        fix(t.x + fixSigma * rng.gauss(), t.y + fixSigma * rng.gauss(), fixSigma)};
                    const bool haveFix = (tick % 5) == 0;
                    d.step(odx, ody, t.th + imuDrift,
                           haveFix ? std::span<const CorrectionProposal>{ps}
                                   : std::span<const CorrectionProposal>{},
                           kH);
                    symmetric = symmetric && isExactlySymmetric(f);
                    definite = definite && isPositiveDefinite(f);
                    const double tr = f.positionCovarianceTrace();
                    finite = finite && std::isfinite(tr) && tr >= 0.0 && tr < 1.0e4;
                    worstTrace = std::max(worstTrace, tr);
                    ++checkedTicks;
                }
                CAPTURE(traj);
                CAPTURE(seed);
                CAPTURE(level);
                CHECK(symmetric);
                CHECK(definite);
                CHECK(finite);
                CHECK(f.numericGuardTrips() == 0);
            }
        }
    }
    MESSAGE("covariance sweep: ", checkedTicks, " ticks checked, worst position trace ",
            worstTrace, " in^2");
    CHECK(checkedTicks == 3 * 4 * 3 * 600);
}

// Would catch: a covariance that never grows while dead-reckoning. This is the property
// that E2 had to hand-build inside a corrector for want of a filter (its D2 anti-lockout
// widening), and it is the reason a fix that is rejected while the estimate is fresh must
// be ACCEPTED after a long blind stretch. A Q that ignores travel makes the estimator
// permanently as confident as it was at its last fix, and the correction locks out exactly
// when it is worth the most.
TEST_CASE("EkfFusion: process noise scales with TRAVEL, not with time alone") {
    EkfFusion still{};
    SeamDriver ds{&still};
    ds.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 600; ++i) {
        ds.step(0.0, 0.0, 0.0, {}, kH);
    }
    EkfFusion driving{};
    SeamDriver dd{&driving};
    dd.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 600; ++i) {
        dd.step(0.3, 0.0, 0.0, {}, kH);  // 30 in/s for 6 s = 180 inches
    }
    MESSAGE("trace after 6 s: standing still ", still.positionCovarianceTrace(), " in^2, after 180 in ",
            driving.positionCovarianceTrace(), " in^2");
    CHECK(driving.positionCovarianceTrace() > 4.0 * still.positionCovarianceTrace());
    CHECK(still.positionCovarianceTrace() > 0.0);  // …but it never stops growing entirely
}

// Would catch: THE GATE LOCKOUT, stated as behaviour rather than as a number. The same fix,
// at the same distance, must be rejected by a filter that has just been corrected and
// accepted by one that has driven a long way blind. This is E2's finding 2 and D2 in one
// assertion, and it is what a covariance-driven gate buys over a fixed 12-inch bound.
TEST_CASE("EkfFusion: the SAME distant fix is rejected when confident and accepted when lost") {
    const auto run = [](int blindTicks) {
        EkfFusion f{};
        SeamDriver d{&f};
        d.step(0.0, 0.0, 0.0, {}, kH);
        // settle: 200 ticks of tight fixes at the origin, standing still
        for (int i = 0; i < 200; ++i) {
            const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
            d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        }
        // then drive blind
        for (int i = 0; i < blindTicks; ++i) {
            d.step(0.3, 0.0, 0.0, {}, kH);
        }
        // and offer one fix 20 inches away from where the filter thinks it is
        const std::array<CorrectionProposal, 1> ps{
            fix(d.fx + 20.0, d.fy, 1.0)};
        const FusionResult r = d.step(0.3, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        return r;
    };
    const FusionResult fresh = run(0);
    const FusionResult lost = run(1200);  // 360 inches of dead-reckoning
    CHECK(fresh.gated);
    CHECK_FALSE(fresh.applied);
    CHECK(fresh.audit.reason == GateReason::RejectedMahalanobis);
    CHECK(lost.applied);
    CHECK(lost.audit.reason == GateReason::Accepted);
    MESSAGE("same 20 in fix — fresh: nu = ", fresh.audit.mahalanobis,
            " (rejected); after 360 in blind: nu = ", lost.audit.mahalanobis, " (accepted)");
}

// ─────────────────────────────────────────────────────────────────────────────────────
// T5 — the Mahalanobis distance and the covariance trace are real numbers
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: `gateMahalanobis` filled with a look-alike. The value is re-derived here
// from the filter's own exposed covariance and the proposal's own sigma, by an
// implementation of nu = sqrt(r' S^-1 r) written in this file — so a scalar ratio, a
// residual magnitude, or a normalisation by a constant would all disagree.
TEST_CASE("EkfFusion: gateMahalanobis is nu = sqrt(r' (HPH' + R)^-1 r), re-derived here") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 40; ++i) {
        d.step(0.15, 0.05, 0.2, {}, kH);
    }
    // capture P BEFORE the fix arrives — the gate is tested against the prior
    const double p00 = f.covariance(EkfFusion::kPx, EkfFusion::kPx);
    const double p01 = f.covariance(EkfFusion::kPx, EkfFusion::kPy);
    const double p11 = f.covariance(EkfFusion::kPy, EkfFusion::kPy);
    const double sigma = 1.5;

    const double targetX = d.fx + 3.0;
    const double targetY = d.fy - 2.0;
    const std::array<CorrectionProposal, 1> ps{fix(targetX, targetY, sigma)};
    const FusionResult r = d.step(0.15, 0.05, 0.2, std::span<const CorrectionProposal>{ps}, kH);

    // The prior P the gate actually saw is one more tick of Q and one more propagation on
    // from what we captured, so this oracle reproduces the innovation and the FORM of the
    // normalisation, and asserts the filter's number matches it closely rather than to the
    // bit. A tier that reported |r|, or |r|/sigma, or a constant, misses by a wide margin.
    const double rx = r.audit.residualX.value();
    const double ry = r.audit.residualY.value();
    const double s00 = p00 + sigma * sigma;
    const double s01 = p01;
    const double s11 = p11 + sigma * sigma;
    const double det = s00 * s11 - s01 * s01;
    const double nu2 = (rx * (s11 * rx - s01 * ry) + ry * (-s01 * rx + s00 * ry)) / det;
    const double nu = std::sqrt(nu2);
    CHECK(r.audit.mahalanobis == doctest::Approx(nu).epsilon(0.02));
    CHECK(r.audit.mahalanobis > 0.0);
    // …and it is NOT any of the look-alikes E2 refused to write.
    const double residual = std::hypot(rx, ry);
    CHECK(r.audit.mahalanobis != doctest::Approx(residual));
    CHECK(r.audit.mahalanobis != doctest::Approx(residual / sigma));
}

// Would catch: `covarianceTrace` reporting something other than the POSITION block — a
// full 5-state trace would add square inches to square radians to square inches per second
// squared and produce a number with no unit, which a reader could not turn into the 1-sigma
// radius the field exists to give them.
TEST_CASE("EkfFusion: covarianceTrace is the POSITION covariance trace, in square inches") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    const FusionResult r = d.step(0.2, 0.1, 0.05, {}, kH);
    const double expected = f.covariance(EkfFusion::kPx, EkfFusion::kPx) +
                            f.covariance(EkfFusion::kPy, EkfFusion::kPy);
    CHECK(r.audit.covarianceTrace == expected);
    CHECK(r.audit.covarianceTrace > 0.0);
    // it excludes the heading and velocity variances, which are large at this point
    CHECK(f.covariance(EkfFusion::kVx, EkfFusion::kVx) > 0.0);
    CHECK(r.audit.covarianceTrace < f.covariance(EkfFusion::kPx, EkfFusion::kPx) +
                                        f.covariance(EkfFusion::kPy, EkfFusion::kPy) +
                                        f.covariance(EkfFusion::kVx, EkfFusion::kVx));
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Outlier rejection
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: an outlier inflating the state, which is the failure a gate exists to
// prevent and the one that is easiest to get subtly wrong — a filter that REJECTS the fix
// but still folds it into the covariance, or one that lets each rejection widen P a little
// until the next lie walks in. Two thousand consecutive lies, all confidently 40 inches
// wrong, must move the estimate by NOTHING and must not open the door for the next one.
TEST_CASE("EkfFusion: repeated confident outliers move nothing and do not inflate the state") {
    EkfFusionConfig cfg;
    cfg.reinitRejectCount = 100000;  // re-init OFF for this case; it is tested on its own
    EkfFusion f{cfg};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 200; ++i) {  // become confident about the origin
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.4)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    const double traceBefore = f.positionCovarianceTrace();
    double worst = 0.0;
    for (int i = 0; i < 2000; ++i) {
        const std::array<CorrectionProposal, 1> ps{fix(40.0, -30.0, 0.4, 1.0)};
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        CHECK_FALSE(r.applied);
        worst = std::max(worst, std::hypot(d.fx, d.fy));
    }
    MESSAGE("2000 confident 50-inch lies moved the estimate ", worst, " inches");
    CHECK(worst < 0.05);  // the estimate did not move
    CHECK(f.rejectedFixes() == 2000);
    CHECK(f.reinitCount() == 0);
    // The covariance still grows (time passes) but nowhere near enough to let the lie in.
    CHECK(f.positionCovarianceTrace() > traceBefore);
    // …and nowhere near the ~278 in^2 it would take for a 50-inch lie to pass a 3-sigma gate.
    CHECK(f.positionCovarianceTrace() < 50.0);
    CHECK(isPositiveDefinite(f));
}

// Would catch: a NaN or an Inf in a proposal poisoning five states and twenty-five
// covariance entries permanently. A single NaN in P never washes out and is completely
// silent — the pose keeps being published, and every gate decision afterwards is nonsense.
TEST_CASE("EkfFusion: non-finite proposals are rejected and never enter the state") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    const std::array<CorrectionProposal, 3> ps{fix(nan, 0.0, 1.0), fix(0.0, inf, 1.0),
                                               fix(1.0, 1.0, nan)};
    const FusionResult r = d.step(0.1, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    CHECK(std::isfinite(r.x.value()));
    CHECK(std::isfinite(r.y.value()));
    CHECK_FALSE(r.applied);
    CHECK(r.gated);
    CHECK(r.audit.reason == GateReason::RejectedMahalanobis);
    for (std::size_t i = 0; i < kN; ++i) {
        CHECK(std::isfinite(f.state(i)));
        for (std::size_t j = 0; j < kN; ++j) {
            CHECK(std::isfinite(f.covariance(i, j)));
        }
    }
    CHECK(isPositiveDefinite(f));
}

// Would catch: a proposal with a zero or negative stated sigma dividing by zero inside the
// update. The Localizer screens for it, but a policy that relied on being called only by a
// well-behaved Localizer would be a landmine for the next caller.
TEST_CASE("EkfFusion: a zero-sigma proposal is rejected rather than dividing by zero") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    const std::array<CorrectionProposal, 1> ps{fix(2.0, 2.0, 0.0)};
    const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    CHECK_FALSE(r.applied);
    CHECK(r.gated);
    CHECK(std::isfinite(r.x.value()));
    CHECK(isPositiveDefinite(f));
}

// ─────────────────────────────────────────────────────────────────────────────────────
// ARBITRATION — the capability that justifies the whole chunk
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: two disagreeing sources being handled by picking one, by averaging them
// blindly, or by ignoring the second entirely. THE analytic case: with an essentially
// uninformative prior, folding two independent position measurements must land on the
// inverse-variance weighted mean
//     x* = (z_A/sigma_A^2 + z_B/sigma_B^2) / (1/sigma_A^2 + 1/sigma_B^2)
// which is computed here from the sigmas alone and shares nothing with the filter. A tier
// that ignored the second proposal lands on z_A; one that averaged blindly lands on the
// midpoint; one that picked the more confident lands on z_A. All three miss by inches.
TEST_CASE("EkfFusion: two disagreeing fixes land on the inverse-variance weighted mean") {
    EkfFusionConfig cfg;
    cfg.initialPosStdDev = Length{1.0e4};   // an uninformative prior: the analytic regime
    cfg.gateSigma = 1.0e6;                  // nothing is gated in this case
    cfg.maxNudgeRate = Velocity{1.0e6};     // …and nothing is clamped, so the algebra shows
    EkfFusion f{cfg};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);

    const double sa = 1.0;
    const double sb = 5.0;
    const double zax = 10.0;
    const double zbx = 30.0;
    const double zay = -4.0;
    const double zby = 6.0;
    const std::array<CorrectionProposal, 2> ps{
        fix(zbx, zby, sb, /*conf=*/0.99),   // deliberately the MORE confident of the two,
        fix(zax, zay, sa, /*conf=*/0.10)};  // and deliberately listed first
    const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);

    const double wa = 1.0 / (sa * sa);
    const double wb = 1.0 / (sb * sb);
    const double expectX = (zax * wa + zbx * wb) / (wa + wb);
    const double expectY = (zay * wa + zby * wb) / (wa + wb);
    MESSAGE("A(sigma=1) at ", zax, ", B(sigma=5) at ", zbx, " -> fused ", r.x.value(),
            " (inverse-variance mean ", expectX, ")");
    CHECK(r.applied);
    CHECK(r.x.value() == doctest::Approx(expectX).epsilon(0.002));
    CHECK(r.y.value() == doctest::Approx(expectY).epsilon(0.002));
    // …and it is emphatically NOT any of the wrong answers.
    CHECK(std::abs(r.x.value() - zax) > 0.3);                    // not "trust the first"
    CHECK(std::abs(r.x.value() - zbx) > 10.0);                   // not "trust the confident"
    CHECK(std::abs(r.x.value() - 0.5 * (zax + zbx)) > 5.0);      // not a blind average
}

// Would catch: the weighting being right by accident on one sigma ratio. Swapping which
// source is the precise one must swap which one the answer sits near, and equal sigmas
// must land exactly in the middle — a symmetry that no amount of tuning can fake.
TEST_CASE("EkfFusion: arbitration is symmetric under swapping the two sigmas") {
    const auto fuseTwo = [](double sa, double sb) {
        EkfFusionConfig cfg;
        cfg.initialPosStdDev = Length{1.0e4};
        cfg.gateSigma = 1.0e6;
        cfg.maxNudgeRate = Velocity{1.0e6};
        EkfFusion f{cfg};
        SeamDriver d{&f};
        d.step(0.0, 0.0, 0.0, {}, kH);
        const std::array<CorrectionProposal, 2> ps{fix(0.0, 0.0, sa), fix(20.0, 0.0, sb)};
        return d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH).x.value();
    };
    CHECK(fuseTwo(1.0, 1.0) == doctest::Approx(10.0).epsilon(0.002));      // exact midpoint
    CHECK(fuseTwo(1.0, 5.0) == doctest::Approx(20.0 - fuseTwo(5.0, 1.0)).epsilon(0.002));
    // monotone: as B gets more precise, the answer moves toward B, every step of the way
    double previous = fuseTwo(1.0, 20.0);
    for (const double sb : {10.0, 5.0, 2.0, 1.0, 0.5, 0.25}) {
        const double now = fuseTwo(1.0, sb);
        CAPTURE(sb);
        CHECK(now > previous);
        previous = now;
    }
}

// Would catch: arbitration collapsing the moment one of the two sources is lying. The
// sigma-weighted blend must not be reachable by a fix that fails the gate — a precise but
// WRONG source is exactly the attack, and the answer must sit on the honest source alone.
TEST_CASE("EkfFusion: a gated proposal contributes nothing to the arbitration") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 300; ++i) {
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    double worst = 0.0;
    for (int i = 0; i < 500; ++i) {
        const std::array<CorrectionProposal, 2> ps{fix(0.05, 0.0, 0.5),      // honest
                                                   fix(60.0, 60.0, 0.2)};    // precise, lying
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        CHECK(r.applied);  // the honest one still lands
        CHECK(r.gated);    // …and the liar is still refused
        worst = std::max(worst, std::hypot(d.fx, d.fy));
    }
    CHECK(worst < 0.5);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// T2 — re-init, and the shove it exists for
// ─────────────────────────────────────────────────────────────────────────────────────

namespace {

/// The scenario re-init exists for, and the reason it is not an academic case in VEX: an
/// opponent SHOVES the robot. The wheels do not turn, so the odometry reports no travel
/// and the process noise stays tiny — the estimate is 30 inches wrong and CERTAIN, which
/// is the only state from which a covariance gate cannot recover on its own.
struct ShoveRun {
    std::uint32_t reinits = 0;
    int ticksToRecover = -1;
    double worstCorrection = 0.0;
    double maxJumpPerTick = 0.0;
    GateReason reinitTickReason = GateReason::None;
    double finalError = 0.0;
    double traceBeforeReinit = 0.0;
    double traceAfterReinit = 0.0;
};

[[nodiscard]] ShoveRun runShove(const EkfFusionConfig& cfg, double shoveInches, int ticks) {
    EkfFusion f{cfg};
    SeamDriver d{&f};
    ShoveRun out;
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 300; ++i) {  // become confident about the origin
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    double lastX = d.fx;
    double lastY = d.fy;
    for (int i = 0; i < ticks; ++i) {
        const std::uint32_t before = f.reinitCount();
        const double traceBefore = f.positionCovarianceTrace();
        const std::array<CorrectionProposal, 1> ps{fix(shoveInches, 0.0, 0.5)};
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        if (f.reinitCount() > before && out.reinits == 0) {
            out.reinitTickReason = r.audit.reason;
            out.traceBeforeReinit = traceBefore;
            out.traceAfterReinit = r.audit.covarianceTrace;
        }
        out.reinits = f.reinitCount();
        out.worstCorrection = std::max(out.worstCorrection, f.lastCorrectionMagnitude().value());
        out.maxJumpPerTick = std::max(out.maxJumpPerTick, std::hypot(d.fx - lastX, d.fy - lastY));
        lastX = d.fx;
        lastY = d.fy;
        const double err = std::hypot(d.fx - shoveInches, d.fy);
        if (out.ticksToRecover < 0 && err < 0.5) {
            out.ticksToRecover = i;
        }
        out.finalError = err;
    }
    return out;
}

}  // namespace

// Would catch: THE FAILURE E2 RECORDED AS FINDING 2, left unfixed. An estimator that is
// confidently wrong rejects every honest fix forever, and the complementary tier's fixed
// 12-inch gate makes that permanent. This case is the negative control — with re-init
// disabled, a perfect fix in view the whole time still never repairs a 30-inch error.
TEST_CASE("EkfFusion: without re-init, a shoved estimate never recovers (E2's finding 2)") {
    EkfFusionConfig cfg;
    cfg.reinitRejectCount = 1000000;
    const ShoveRun r = runShove(cfg, 30.0, 3000);
    CHECK(r.reinits == 0);
    CHECK(r.finalError > 29.0);       // thirty seconds later, still thirty inches out
    CHECK(r.ticksToRecover == -1);
}

// Would catch: a re-init that does not happen, one that teleports, or one that is silent.
// The T2 ruling in three assertions — the estimate RECOVERS, it recovers WITHOUT A SNAP
// (every tick's move is inside the per-tick budget, asserted on the tick the re-init fires
// as well as on all the others), and the event is a WORD in the record rather than
// something a reader has to infer from a discontinuity.
TEST_CASE("EkfFusion: re-init recovers a shoved estimate, visibly, and without ever snapping") {
    EkfFusionConfig cfg;
    const ShoveRun r = runShove(cfg, 30.0, 3000);
    const double budget = cfg.maxNudgeRate.value() * kH;
    MESSAGE("shove 30 in: re-inits ", r.reinits, ", recovered after ", r.ticksToRecover,
            " ticks, worst per-tick correction ", r.worstCorrection, " in (budget ", budget,
            "), trace ", r.traceBeforeReinit, " -> ", r.traceAfterReinit, " in^2");
    CHECK(r.reinits >= 1);
    CHECK(r.ticksToRecover > 0);
    CHECK(r.finalError < 0.5);
    // NEVER-SNAP, on the re-init path: no tick moved the estimate more than the budget.
    CHECK(r.worstCorrection <= budget + 1e-9);
    // The raw per-tick move of the published estimate is the correction PLUS the velocity
    // filtering residual from steps B/C, which is not a correction and is not budgeted. It is
    // asserted here at 1% over the budget so a regression that let it grow would still show.
    CHECK(r.maxJumpPerTick <= budget * 1.01);
    // …and the recovery took real time rather than happening in one tick, which is the
    // numeric signature of a nudge rather than a teleport: 30 inches at 12 in/s is 2.5 s.
    CHECK(r.ticksToRecover > 200);
    // DECLARED: the tick carries its own word, and the trace jumps as the second witness.
    CHECK(r.reinitTickReason == GateReason::CovarianceReinit);
    CHECK(r.traceAfterReinit > 100.0 * r.traceBeforeReinit);
}

// Would catch: a re-init that fires on ordinary noise. The bar is N CONSECUTIVE rejections
// AND a persistently large innovation, so a filter that is merely being fed a slightly
// noisy sensor must never declare itself lost — an estimator that re-inits during a normal
// run has thrown away the confidence that makes its gate worth having.
TEST_CASE("EkfFusion: ordinary noise never triggers a re-init") {
    EkfFusion f{};
    SeamDriver d{&f};
    Rng rng{4242};
    Truth t{};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 6000; ++i) {
        const auto tw = scriptTwist(1, i);
        const double px = t.x;
        const double py = t.y;
        t.advance(tw[0], tw[1], tw[2], kH);
        const std::array<CorrectionProposal, 1> ps{
            fix(t.x + 0.7 * rng.gauss(), t.y + 0.7 * rng.gauss(), 0.7)};
        d.step(t.x - px + 0.004 * rng.gauss(), t.y - py + 0.004 * rng.gauss(), t.th,
               (i % 5) == 0 ? std::span<const CorrectionProposal>{ps}
                            : std::span<const CorrectionProposal>{},
               kH);
    }
    CHECK(f.reinitCount() == 0);
    CHECK_FALSE(f.everReinit());
    CHECK(std::hypot(d.fx - t.x, d.fy - t.y) < 2.0);
}

// Would catch: the rate limit missing. A filter that re-inits every time it is unhappy has
// no confidence left to lose, and a re-init storm in the record is indistinguishable from a
// filter that is working. Asserted as the property itself: however many times the estimator
// is hijacked, no two declarations may be closer together than the cooldown.
TEST_CASE("EkfFusion: re-init is rate-limited by its cooldown") {
    EkfFusionConfig cfg;
    EkfFusion f{cfg};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 300; ++i) {
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    // Hijack the estimate over and over, alternating sides every 2 s, for half a minute — a
    // deliberately hostile stream that a filter with no rate limit would re-init on constantly.
    std::vector<double> declaredAt;
    std::uint32_t seen = 0;
    for (int i = 0; i < 3000; ++i) {
        const double target = ((i / 200) % 2 == 0) ? 30.0 : -30.0;
        const std::array<CorrectionProposal, 1> ps{fix(target, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        if (f.reinitCount() > seen) {
            seen = f.reinitCount();
            declaredAt.push_back(static_cast<double>(i) * kH);
        }
    }
    MESSAGE("30 s of alternating hijacks produced ", declaredAt.size(), " re-init declarations");
    CHECK(declaredAt.size() >= 2);   // the case is not vacuous
    CHECK(declaredAt.size() <= 7);   // …and it is nothing like once per opportunity
    for (std::size_t i = 1; i < declaredAt.size(); ++i) {
        CAPTURE(i);
        CHECK(declaredAt[i] - declaredAt[i - 1] >= cfg.reinitCooldown.value() - 1e-9);
    }
    CHECK(f.everReinit());  // latched, and it never clears
}

// Would catch: a Kalman gain near 1 doing what a Kalman gain near 1 does. The whole reason
// decision #4 exists is that an optimal filter WILL snap to a good measurement when it is
// uncertain, and a snapped estimate teleports the pose the motion controller is steering
// off. The bound must hold on every tick, at every gain, including the ticks right after a
// re-init where the gain is closest to 1.
TEST_CASE("EkfFusion: never-snap holds on every tick, position and heading") {
    EkfFusionConfig cfg;
    EkfFusion f{cfg};
    SeamDriver d{&f};
    const double posBudget = cfg.maxNudgeRate.value() * kH;
    const double headBudget = cfg.maxHeadingNudgeRate.value() * kH;
    d.step(0.0, 0.0, 0.0, {}, kH);
    double worstPos = 0.0;
    double worstHead = 0.0;
    int applied = 0;
    for (int i = 0; i < 4000; ++i) {
        // a fix that is always a long way from wherever the filter currently is, so the
        // optimal step is always far bigger than the budget
        const std::array<CorrectionProposal, 1> ps{
            headingFix(d.fx + 11.0, d.fy - 9.0, 0.3, d.publishedHeading(0.0) + 25.0 * kDeg, 1.0)};
        const FusionResult r = d.step(0.05, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        REQUIRE(f.lastCorrectionMagnitude().value() <= posBudget + 1e-9);
        REQUIRE(std::abs(r.headingNudge.value()) <= headBudget + 1e-9);
        worstPos = std::max(worstPos, f.lastCorrectionMagnitude().value());
        worstHead = std::max(worstHead, std::abs(r.headingNudge.value()));
        applied += r.applied ? 1 : 0;
    }
    MESSAGE("never-snap: worst per-tick correction ", worstPos, " in (budget ", posBudget,
            "), worst heading nudge ", worstHead / kDeg, " deg (budget ", headBudget / kDeg, ")");
    MESSAGE("...applied on ", applied, " of 4000 ticks");
    CHECK(applied > 500);        // the bound is not vacuous: corrections really were applied
    CHECK(worstPos > 0.0);
    CHECK(worstHead > 0.0);
    CHECK(worstPos <= posBudget + 1e-9);
    CHECK(worstHead <= headBudget + 1e-9);
}

// Would catch: the per-tick budget being spent once PER PROPOSAL rather than once per
// tick, so four correctors could move the estimate four budgets in one tick. The
// complementary tier clamps the sum for exactly this reason; the EKF has to do the same
// through its sequential updates.
TEST_CASE("EkfFusion: four simultaneous proposals cannot out-vote the per-tick budget") {
    EkfFusionConfig cfg;
    cfg.gateSigma = 1.0e6;  // the SUBJECT here is the budget; the gate is tested on its own
    EkfFusion f{cfg};
    SeamDriver d{&f};
    const double budget = cfg.maxNudgeRate.value() * kH;
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 500; ++i) {
        const std::array<CorrectionProposal, 4> ps{fix(d.fx + 9.0, d.fy, 0.3),
                                                   fix(d.fx + 9.0, d.fy, 0.4),
                                                   fix(d.fx + 9.0, d.fy, 0.5),
                                                   fix(d.fx + 9.0, d.fy, 0.6)};
        const double before = d.fx;
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        REQUIRE(r.applied);
        REQUIRE(f.lastCorrectionMagnitude().value() <= budget + 1e-9);
        // The raw position difference also carries the velocity-filtering residual from
        // steps B/C, which is not a correction and is not budgeted; 0.1% covers it here,
        // where the robot is stationary. The dead-reckon case measures that term on its own.
        REQUIRE(std::abs(d.fx - before) <= budget * 1.001);
    }
}

// ─────────────────────────────────────────────────────────────────────────────────────
// The four holes the mutation harness found, each closed by a case that fails alone
// ─────────────────────────────────────────────────────────────────────────────────────

// HOLE 1. Would catch: the per-tick HEADING budget being spent once per proposal instead of
// once per tick. Every other heading case in this chunk uses ONE heading-providing proposal,
// so the budget can never be spent twice and the accounting is invisible. E3 explicitly
// anticipates two tag sources (an AI Vision camera and a Pi), and the moment there are two the
// bound is load-bearing: three sources each pulling a full budget would move the robot's idea
// of which way it faces three times faster than the documented rate, and a yaw snap poisons
// every field-relative command after it.
TEST_CASE("EkfFusion: three heading proposals in one tick cannot out-vote the heading budget") {
    EkfFusionConfig cfg;
    EkfFusion f{cfg};
    SeamDriver d{&f};
    const double headBudget = cfg.maxHeadingNudgeRate.value() * kH;
    d.step(0.0, 0.0, 0.0, {}, kH);
    double worst = 0.0;
    int applied = 0;
    for (int i = 0; i < 1500; ++i) {
        // three sources, all claiming the robot is 20 degrees off in the SAME direction, so
        // their pulls add rather than cancelling
        const double target = d.publishedHeading(0.0) + 20.0 * kDeg;
        const std::array<CorrectionProposal, 3> ps{headingFix(0.0, 0.0, 0.4, target),
                                                   headingFix(0.0, 0.0, 0.6, target),
                                                   headingFix(0.0, 0.0, 0.9, target)};
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        REQUIRE(std::abs(r.headingNudge.value()) <= headBudget + 1e-9);
        worst = std::max(worst, std::abs(r.headingNudge.value()));
        applied += r.headingApplied ? 1 : 0;
    }
    MESSAGE("three heading sources at once: worst nudge ", worst / kDeg, " deg (budget ",
            headBudget / kDeg, "), applied on ", applied, " of 1500 ticks");
    CHECK(applied > 200);                         // not vacuous
    CHECK(worst > 0.5 * headBudget);              // the budget really was the binding constraint
    CHECK(worst <= headBudget + 1e-9);
}

// HOLE 2. Would catch: THE MOST DANGEROUS STATE THIS FILTER CAN BE IN. The odometry increment
// is a RELATIVE measurement; folding it as though it told you where you ARE shrinks the
// position covariance on every tick, and a position covariance that shrinks while
// dead-reckoning is a filter that becomes CERTAIN as it becomes WRONG. After that no absolute
// fix can ever pass the gate again — E2's D2 gate-lockout failure, arrived at from the other
// side.
//
// The existing covariance-growth cases do NOT see it: process noise and the heading coupling
// still dominate the total, so the trace still grows and the coarse assertion still passes.
// The sharp statement is the one that had to be written: while dead-reckoning with no absolute
// fix, the position covariance is MONOTONICALLY NON-DECREASING, tick over tick, always.
TEST_CASE("EkfFusion: dead-reckoning never buys position confidence it did not earn") {
    // TWO statements, because the obvious one is not universally true and finding that out is
    // part of the content. On a STRAIGHT blind drive the position covariance is monotonically
    // non-decreasing. On a trajectory with hard reversals and spins it is NOT, legitimately:
    // `F P Fᵀ` carries a position–heading cross-covariance that can be negative, so driving a
    // curve can genuinely cancel some of the position uncertainty a previous curve created.
    // Measured here: drops of up to 12 in² on the spinning and stop-start scripts, with the
    // correct code. So the universal statement is the second one — the covariance can never end
    // up BELOW what the process-noise model alone put into it, because nothing between fixes
    // measures position.
    {
        EkfFusion f{};
        SeamDriver d{&f};
        d.step(0.0, 0.0, 0.0, {}, kH);
        double previous = f.positionCovarianceTrace();
        bool monotone = true;
        for (int i = 0; i < 3000; ++i) {
            d.step(0.3, 0.0, 0.0, {}, kH);  // straight, 900 inches, no proposals
            const double now = f.positionCovarianceTrace();
            monotone = monotone && (now >= previous);
            previous = now;
        }
        CHECK(monotone);
    }
    for (int traj = 0; traj < 3; ++traj) {
        EkfFusion f{};
        SeamDriver d{&f};
        Truth t{};
        d.step(0.0, 0.0, 0.0, {}, kH);
        const double start = f.positionCovarianceTrace();
        double distance = 0.0;
        for (int i = 0; i < 3000; ++i) {
            const auto tw = scriptTwist(traj, i);
            const double px = t.x;
            const double py = t.y;
            t.advance(tw[0], tw[1], tw[2], kH);
            distance += std::hypot(t.x - px, t.y - py);
            d.step(t.x - px, t.y - py, t.th, {}, kH);  // NO proposals: pure dead-reckoning
        }
        // The lower bound the documented model guarantees: sigma = posNoisePerInch * distance
        // on each axis, so the trace is twice its square. Computed here from the config, not
        // from the filter.
        const double k = EkfFusionConfig{}.posNoisePerInch;
        const double floorTrace = 2.0 * (k * distance) * (k * distance);
        CAPTURE(traj);
        MESSAGE("traj ", traj, ": ", distance, " inches blind -> trace ",
                f.positionCovarianceTrace(), " in^2 (process-noise floor ", floorTrace, ")");
        CHECK(f.positionCovarianceTrace() >= floorTrace);
        CHECK(f.positionCovarianceTrace() > start);
    }
}

// HOLE 3. Would catch: an accepted fix failing to reset the consecutive-rejection run, so a
// perfectly healthy estimator accumulates its way to a re-init over a long match. This is the
// realistic two-corrector case — one source healthy, one source persistently wrong (a GPS
// reflecting off a wall, a tag map entry that is 40 inches out) — and it must NOT be read as
// "the estimator is lost". It is read as "one of the two sources is lying", which is exactly
// what the gate is for.
TEST_CASE("EkfFusion: a healthy source resets the reject run, so a liar cannot force a re-init") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 200; ++i) {
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    for (int i = 0; i < 6000; ++i) {  // a full minute of one honest source and one liar
        const std::array<CorrectionProposal, 2> ps{fix(0.0, 0.0, 0.5),        // honest
                                                   fix(45.0, -20.0, 0.7)};    // 49 inches out
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        REQUIRE(r.applied);  // the honest source keeps landing…
        REQUIRE(r.gated);    // …and the liar keeps being refused
        REQUIRE(f.consecutiveRejects() == 0);  // …and the run never accumulates
    }
    CHECK(f.reinitCount() == 0);
    CHECK_FALSE(f.everReinit());
    CHECK(f.rejectedFixes() == 6000);          // the liar really was rejected 6000 times
    CHECK(std::hypot(d.fx, d.fy) < 0.2);       // and it moved the estimate nowhere
}

// HOLE 5 (found on the harness re-run, once the pattern compiled). Would catch: the
// mean-innovation bar dropping out of the re-init trigger, so a long run of SMALL rejections
// makes the estimator declare itself lost.
//
// The realistic case is a corrector with a small systematic bias — a lever arm entered half an
// inch off, a tag map entry two inches out, a GPS whose frame convention is slightly wrong. It
// sits just outside the gate forever, and the rejections pile up indefinitely. **That is a
// calibration problem, not a lost robot**, and an estimator that responds by throwing away its
// confidence would make a small persistent error into a large intermittent one — it would
// re-init, accept the biased fix wholesale, drift back, and re-init again. The count alone must
// not be enough; the disagreement has to be large as well.
TEST_CASE("EkfFusion: a small persistent disagreement is a calibration problem, not a re-init") {
    EkfFusionConfig cfg;
    EkfFusion f{cfg};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 300; ++i) {  // become confident about the origin
        const std::array<CorrectionProposal, 1> ps{fix(0.0, 0.0, 0.5)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    // Now the source develops a 2-inch bias: far enough outside the gate to be refused every
    // tick (nu ~ 4), nowhere near the 6-inch bar that means "lost".
    int rejected = 0;
    for (int i = 0; i < 4000; ++i) {
        const std::array<CorrectionProposal, 1> ps{fix(2.0, 0.0, 0.5)};
        const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
        rejected += r.gated ? 1 : 0;
        REQUIRE(f.reinitCount() == 0);
    }
    MESSAGE("a 2-inch systematic bias produced ", rejected,
            " consecutive rejections and ", f.reinitCount(), " re-inits (mean innovation 2 in, "
            "bar ", cfg.reinitInnovation.value(), " in); estimate ended ",
            std::hypot(d.fx, d.fy), " in from the origin");
    // The rejections really did pile up, far past the COUNT threshold on its own…
    CHECK(rejected > 10 * cfg.reinitRejectCount);
    // …and the estimator never declared itself lost over two inches.
    CHECK(f.reinitCount() == 0);
    CHECK_FALSE(f.everReinit());
    // What it did instead is the right answer, and worth pinning: as time passed with nothing
    // folded, its own uncertainty grew until two inches was no longer an outlier, and it then
    // accepted the fix and converged on it — gradually, at the per-tick rate. A small persistent
    // disagreement resolves itself by the estimator becoming appropriately less sure, which is
    // exactly what the covariance is for. No re-init was needed, and none happened.
    CHECK(std::hypot(d.fx, d.fy) == doctest::Approx(2.0).epsilon(0.05));
}

// HOLE 4. Would catch: proposals folded in ARRIVAL order rather than most-trusted-first.
//
// The reason the strongest arbitration case in this file cannot see this is worth writing down:
// it uses a deliberately uninformative prior, and WITH AN UNINFORMATIVE PRIOR SEQUENTIAL KALMAN
// UPDATES ARE EXACTLY ORDER-INDEPENDENT — they are the information form, which commutes. Order
// only matters once GATING is in play. A tight fix folded first shrinks P, and a doubtful fix
// is then judged against the tighter P and REFUSED where it would otherwise have walked in.
//
// The arithmetic here is chosen so the verdict flips, and it can be checked by hand:
//   prior P = 2^2 = 4 in^2 on each axis.
//   most-trusted-first: honest (sigma 0.2) at the prediction, nu = 0, accepted; P -> 4*0.04/4.04
//                       = 0.0396. Then the liar at 8 in, sigma 2: S = 0.0396 + 4 = 4.04,
//                       nu = 8/2.010 = 3.98 > 3  =>  REFUSED.
//   arrival order:      the liar first: S = 4 + 4 = 8, nu = 8/2.828 = 2.83 < 3  =>  ACCEPTED.
TEST_CASE("EkfFusion: most-trusted-first ordering is what refuses a fix that arrival order lets in") {
    EkfFusionConfig cfg;
    cfg.initialPosStdDev = Length{2.0};    // a known prior, so the arithmetic above is exact
    cfg.maxNudgeRate = Velocity{1.0e6};    // the clamp is not the subject here
    EkfFusion f{cfg};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    const std::array<CorrectionProposal, 2> ps{fix(8.0, 0.0, 2.0),    // the liar, listed FIRST
                                               fix(0.0, 0.0, 0.2)};   // the honest, tight fix
    const FusionResult r = d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    MESSAGE("liar-first arrival order vs sigma order: applied=", r.applied, " gated=", r.gated,
            " x=", r.x.value(), " nu(audited)=", r.audit.mahalanobis);
    CHECK(r.applied);                       // the honest fix landed
    CHECK(r.gated);                         // and the liar did NOT
    CHECK(f.rejectedFixes() == 1);
    CHECK(f.acceptedFixes() == 1);
    CHECK(r.x.value() == doctest::Approx(0.0).epsilon(0.02));  // nowhere near 4 or 8
}

// The system property the unreachable posterior-finiteness guard stands for. Would catch: any
// combination of hostile config and hostile proposal driving the state or the covariance
// non-finite. The guard itself could not be mutated red — every route to a non-finite posterior
// I could construct is caught earlier, by the screen on the proposal or by the determinant test
// inside the update — so this pins the property rather than the branch, and the guard stays as
// defence in depth. Recorded as an unclosed mutation in E4-COMPLETED rather than pretended away.
TEST_CASE("EkfFusion: hostile configs and hostile proposals never produce a non-finite state") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double sigmas[] = {1e-6, 1.0, 1e6, 1e150};
    const double priors[] = {1e-3, 24.0, 1e6, 1e120};
    for (const double sg : sigmas) {
        for (const double pr : priors) {
            EkfFusionConfig cfg;
            cfg.initialPosStdDev = Length{pr};
            cfg.initialVelStdDev = Velocity{pr};
            cfg.headingStdDev = AngleDim{sg};
            EkfFusion f{cfg};
            SeamDriver d{&f};
            d.step(0.0, 0.0, 0.0, {}, kH);
            Rng rng{static_cast<std::uint64_t>(sg * 7.0 + pr)};
            for (int i = 0; i < 200; ++i) {
                const double wild = (i % 7 == 0) ? inf : ((i % 11 == 0) ? nan : 1e9 * rng.gauss());
                const std::array<CorrectionProposal, 3> ps{
                    fix(wild, wild, sg), headingFix(1e8 * rng.gauss(), 0.0, sg, 3.0),
                    fix(rng.gauss(), rng.gauss(), 1e-9)};
                const FusionResult r =
                    d.step(0.3 * rng.gauss(), 0.3 * rng.gauss(), 0.1 * rng.gauss(),
                           std::span<const CorrectionProposal>{ps}, kH);
                REQUIRE(std::isfinite(r.x.value()));
                REQUIRE(std::isfinite(r.y.value()));
                REQUIRE(std::isfinite(r.headingNudge.value()));
                for (std::size_t a = 0; a < kN; ++a) {
                    REQUIRE(std::isfinite(f.state(a)));
                    for (std::size_t b = 0; b < kN; ++b) {
                        REQUIRE(std::isfinite(f.covariance(a, b)));
                    }
                }
            }
            CAPTURE(sg);
            CAPTURE(pr);
            CHECK(isExactlySymmetric(f));
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────────────
// T1 — heading
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: the filter quietly acquiring ownership of heading. With no proposal that
// CLAIMS to measure heading, the published heading must be the raw IMU reading bit for
// bit — the same property E3 proved for the complementary tier, and the thing that would
// break first if a position fix were allowed to rotate the robot through a cross-covariance
// term. Compared with `==` on doubles, deliberately.
TEST_CASE("EkfFusion: with no heading-providing proposal, heading is the raw IMU, bit for bit") {
    EkfFusion f{};
    SeamDriver d{&f};
    Truth t{};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 1500; ++i) {
        const auto tw = scriptTwist(1, i);
        const double px = t.x;
        const double py = t.y;
        t.advance(tw[0], tw[1], tw[2], kH);
        // position fixes only — deliberately a long way off, so any cross-covariance leak
        // into heading would be large enough to see
        const std::array<CorrectionProposal, 1> ps{fix(t.x + 4.0, t.y - 3.0, 0.8)};
        d.step(t.x - px, t.y - py, t.th, std::span<const CorrectionProposal>{ps}, kH);
        REQUIRE(d.bias == 0.0);  // exactly zero: not "small"
        REQUIRE(d.publishedHeading(t.th) == t.th);
    }
    CHECK(f.acceptedFixes() > 1000);  // the property is not vacuous — fixes really landed
}

// Would catch: a heading correction that does not accumulate — the M2 red team's
// corrections-not-accumulating failure, which E3 built the persistent bias to prevent. A
// nudge that only decorates this tick's published heading is discarded on the next one.
TEST_CASE("EkfFusion: an absolute heading fix moves the bias, monotonically, and converges") {
    EkfFusion f{};
    SeamDriver d{&f};
    const double imu = 0.0;
    const double truthHeading = 4.0 * kDeg;  // the IMU is 4 degrees behind the truth
    d.step(0.0, 0.0, imu, {}, kH);
    double previous = 0.0;
    bool monotone = true;
    bool noOvershoot = true;
    for (int i = 0; i < 1500; ++i) {
        const std::array<CorrectionProposal, 1> ps{headingFix(0.0, 0.0, 1.0, truthHeading)};
        d.step(0.0, 0.0, imu, std::span<const CorrectionProposal>{ps}, kH);
        monotone = monotone && (d.bias >= previous - 1e-15);
        noOvershoot = noOvershoot && (d.bias <= truthHeading + 1e-9);
        previous = d.bias;
    }
    MESSAGE("heading bias after 15 s: ", d.bias / kDeg, " deg (truth 4 deg)");
    CHECK(monotone);
    CHECK(noOvershoot);
    CHECK(d.bias / kDeg == doctest::Approx(4.0).epsilon(0.02));
}

// Would catch: `providesHeading` not actually being load-bearing — E3 found exactly this
// hole and closed it by keeping the lie INSIDE the gate, where only the flag can stop it.
// The same trap applies here: a 5-degree wrong heading is well inside any plausible gate,
// so if the flag is ignored the bias moves.
TEST_CASE("EkfFusion: a source that does not claim heading cannot move it, even inside the gate") {
    EkfFusion f{};
    SeamDriver d{&f};
    d.step(0.0, 0.0, 0.0, {}, kH);
    for (int i = 0; i < 800; ++i) {
        CorrectionProposal p = fix(0.0, 0.0, 0.5);
        // a heading 5 degrees wrong, carried on the pose but NOT claimed
        p.fieldPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(5.0)};
        const std::array<CorrectionProposal, 1> ps{p};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    CHECK(d.bias == 0.0);
    // …and the same corrector moves it the moment it does claim to measure heading.
    for (int i = 0; i < 800; ++i) {
        const std::array<CorrectionProposal, 1> ps{headingFix(0.0, 0.0, 0.5, 5.0 * kDeg)};
        d.step(0.0, 0.0, 0.0, std::span<const CorrectionProposal>{ps}, kH);
    }
    CHECK(d.bias / kDeg == doctest::Approx(5.0).epsilon(0.05));
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Dead-reckoning fidelity — the cost of putting a filter between the wheels and the pose
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: the velocity filtering costing more than it should. Where the complementary
// tier returns the odometry's prediction untouched when nothing is proposed, this tier
// returns its own one-tick-filtered version. The header claims that difference is bounded
// by a fraction of ONE tick's travel and is transient rather than cumulative — if it were
// cumulative, swapping tiers would quietly make dead-reckoning worse, which is the one
// regression a swap must not hide.
TEST_CASE("EkfFusion: dead-reckoning tracks the odometry within a bounded, non-cumulative gap") {
    const auto measure = [](int ticks) {
        EkfFusion f{};
        SeamDriver d{&f};
        Truth t{};
        d.step(0.0, 0.0, 0.0, {}, kH);
        double odomX = 0.0;
        double odomY = 0.0;
        double worstGap = 0.0;
        double worstTravel = 0.0;
        for (int i = 0; i < ticks; ++i) {
            const auto tw = scriptTwist(2, i);  // the stop-start profile: hardest for a lag
            const double px = t.x;
            const double py = t.y;
            t.advance(tw[0], tw[1], tw[2], kH);
            const double odx = t.x - px;
            const double ody = t.y - py;
            odomX += odx;
            odomY += ody;
            d.step(odx, ody, t.th, {}, kH);
            worstGap = std::max(worstGap, std::hypot(d.fx - odomX, d.fy - odomY));
            worstTravel = std::max(worstTravel, std::hypot(odx, ody));
        }
        return std::array<double, 3>{worstGap, std::hypot(d.fx - odomX, d.fy - odomY),
                                     worstTravel};
    };
    const auto shortRun = measure(3000);    // 30 s
    const auto longRun = measure(12000);    // 120 s — four times the driving
    MESSAGE("dead-reckon gap vs raw odometry: 30 s worst ", shortRun[0], " in / final ",
            shortRun[1], " in; 120 s worst ", longRun[0], " in / final ", longRun[1],
            " in; one tick's travel at top speed ", shortRun[2], " in");
    // BOUNDED by a small multiple of ONE tick's travel — a shape, not a tuned constant.
    CHECK(shortRun[0] < 2.0 * shortRun[2]);
    CHECK(longRun[0] < 2.0 * longRun[2]);
    // NOT CUMULATIVE, which is the property that actually matters: four times the driving
    // does not make the gap any worse. A lag that accumulated would show up here as a gap
    // that grew with the run, and swapping tiers would then quietly degrade dead-reckoning.
    CHECK(longRun[0] < shortRun[0] * 1.05);
    CHECK(shortRun[0] > 0.0);  // the filter really is doing something, so the bound is real
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Recovery of a known trajectory from corrupted streams, swept
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a filter that is busy and useless. Corrupt the odometry with a systematic
// scale error (the classic wheel-diameter mismeasurement) plus noise, drift the IMU, and
// add sensor noise to the fixes — then require the estimate to track truth, and to beat the
// same filter with the fixes withheld. Swept over trajectories, seeds and noise levels so a
// single friendly scenario cannot carry it.
TEST_CASE("[sweep] EkfFusion: recovers a known trajectory from corrupted streams") {
    int runs = 0;
    double worstCorrected = 0.0;
    double meanCorrected = 0.0;
    double meanBlind = 0.0;
    int wins = 0;
    for (int traj = 0; traj < 3; ++traj) {
        for (std::uint64_t seed = 1; seed <= 4; ++seed) {
            for (int level = 0; level < 2; ++level) {
                const double odomScale = 1.0 + 0.01 * static_cast<double>(level + 1);
                const double odomSigma = 0.003 * static_cast<double>(level + 1);
                const double fixSigma = 0.6 * static_cast<double>(level + 1);
                const auto run = [&](bool useFixes) {
                    Rng rng{seed * 101 + static_cast<std::uint64_t>(traj * 7 + level)};
                    EkfFusion f{};
                    SeamDriver d{&f};
                    Truth t{};
                    double drift = 0.0;
                    d.step(0.0, 0.0, 0.0, {}, kH);
                    for (int tick = 0; tick < 3000; ++tick) {
                        const auto tw = scriptTwist(traj, tick);
                        const double px = t.x;
                        const double py = t.y;
                        t.advance(tw[0], tw[1], tw[2], kH);
                        drift += (1.0 / 60.0) * kDeg * kH;  // HA-20's 1 deg/min, one-sided
                        const double odx = (t.x - px) * odomScale + odomSigma * rng.gauss();
                        const double ody = (t.y - py) * odomScale + odomSigma * rng.gauss();
                        const std::array<CorrectionProposal, 1> ps{
                            fix(t.x + fixSigma * rng.gauss(), t.y + fixSigma * rng.gauss(),
                                fixSigma)};
                        d.step(odx, ody, t.th + drift,
                               (useFixes && (tick % 5) == 0)
                                   ? std::span<const CorrectionProposal>{ps}
                                   : std::span<const CorrectionProposal>{},
                               kH);
                    }
                    return std::hypot(d.fx - t.x, d.fy - t.y);
                };
                const double corrected = run(true);
                const double blind = run(false);
                CAPTURE(traj);
                CAPTURE(seed);
                CAPTURE(level);
                CHECK(corrected < 3.0 * fixSigma);  // tracks truth to the sensor's own scale
                CHECK(corrected < blind);
                wins += (corrected < blind) ? 1 : 0;
                worstCorrected = std::max(worstCorrected, corrected);
                meanCorrected += corrected;
                meanBlind += blind;
                ++runs;
            }
        }
    }
    MESSAGE("recovery sweep over ", runs, " runs: mean final error corrected ",
            meanCorrected / runs, " in vs blind ", meanBlind / runs, " in; worst corrected ",
            worstCorrected, " in; wins ", wins, "/", runs);
    CHECK(runs == 24);
    CHECK(wins == runs);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// A from-scratch algebra oracle
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: the implementation drifting from the algorithm its own header documents —
// a transposed index, Q added in the wrong place, the Joseph form quietly replaced by
// (I−KH)P, a gain row that should be blocked and is not, the rate clamp applied to the
// state instead of to the gain.
//
// WHAT THIS ORACLE DOES AND DOES NOT PROVE. It re-implements the documented tick from
// scratch, in this file, with plain loops and no shared code — so it catches transcription
// and structural errors. It cannot validate the EQUATIONS themselves, because it uses the
// same ones; that job belongs to the invariants above and the plant comparison in
// ekf_fusion_accuracy_test.cpp. Both halves are needed and neither substitutes.
namespace {

struct Oracle {
    std::array<double, 5> x{};
    std::array<double, 25> P{};

    void set(std::size_t i, std::size_t j, double v) { P[i * 5 + j] = v; }
    [[nodiscard]] double get(std::size_t i, std::size_t j) const { return P[i * 5 + j]; }

    void symmetrize() {
        for (std::size_t i = 0; i < 5; ++i) {
            for (std::size_t j = i + 1; j < 5; ++j) {
                const double a = 0.5 * (get(i, j) + get(j, i));
                set(i, j, a);
                set(j, i, a);
            }
        }
    }

    /// P <- A P A' + K R K', written out longhand.
    void joseph(const std::array<double, 25>& A, const std::array<double, 10>& K,
                const std::array<double, 4>& R, std::size_t m) {
        std::array<double, 25> AP{};
        for (std::size_t i = 0; i < 5; ++i) {
            for (std::size_t j = 0; j < 5; ++j) {
                double s = 0.0;
                for (std::size_t k = 0; k < 5; ++k) {
                    s += A[i * 5 + k] * P[k * 5 + j];
                }
                AP[i * 5 + j] = s;
            }
        }
        std::array<double, 25> next{};
        for (std::size_t i = 0; i < 5; ++i) {
            for (std::size_t j = 0; j < 5; ++j) {
                double s = 0.0;
                for (std::size_t k = 0; k < 5; ++k) {
                    s += AP[i * 5 + k] * A[j * 5 + k];
                }
                for (std::size_t a = 0; a < m; ++a) {
                    for (std::size_t b = 0; b < m; ++b) {
                        s += K[i * 2 + a] * R[a * m + b] * K[j * 2 + b];
                    }
                }
                next[i * 5 + j] = s;
            }
        }
        P = next;
        symmetrize();
    }

    /// One measurement update, with the same blocking and rate-clamp rules the header
    /// documents. Returns the Mahalanobis distance.
    double update(const std::array<double, 10>& H, std::size_t m, const std::array<double, 2>& r,
                  const std::array<double, 4>& R, bool mayMoveHeading, bool mayMovePosition,
                  double gate, double posBudget, double headBudget) {
        std::array<double, 10> PHt{};
        for (std::size_t i = 0; i < 5; ++i) {
            for (std::size_t a = 0; a < m; ++a) {
                double s = 0.0;
                for (std::size_t j = 0; j < 5; ++j) {
                    s += get(i, j) * H[a * 5 + j];
                }
                PHt[i * 2 + a] = s;
            }
        }
        std::array<double, 4> S{};
        for (std::size_t a = 0; a < m; ++a) {
            for (std::size_t b = 0; b < m; ++b) {
                double s = 0.0;
                for (std::size_t j = 0; j < 5; ++j) {
                    s += H[a * 5 + j] * PHt[j * 2 + b];
                }
                S[a * 2 + b] = s + R[a * m + b];
            }
        }
        std::array<double, 4> Si{};
        if (m == 1) {
            Si[0] = 1.0 / S[0];
        } else {
            const double det = S[0] * S[3] - S[1] * S[2];
            Si[0] = S[3] / det;
            Si[1] = -S[1] / det;
            Si[2] = -S[2] / det;
            Si[3] = S[0] / det;
        }
        double d2 = 0.0;
        for (std::size_t a = 0; a < m; ++a) {
            for (std::size_t b = 0; b < m; ++b) {
                d2 += r[a] * Si[a * 2 + b] * r[b];
            }
        }
        if (d2 > gate * gate) {
            return std::sqrt(d2);  // rejected: nothing touched
        }
        std::array<double, 10> K{};
        for (std::size_t i = 0; i < 5; ++i) {
            const bool blocked = (i == 2 && !mayMoveHeading) ||
                                 ((i == 0 || i == 1) && !mayMovePosition);
            for (std::size_t a = 0; a < m; ++a) {
                if (blocked) {
                    continue;
                }
                double s = 0.0;
                for (std::size_t b = 0; b < m; ++b) {
                    s += PHt[i * 2 + b] * Si[b * 2 + a];
                }
                K[i * 2 + a] = s;
            }
        }
        std::array<double, 5> d{};
        for (std::size_t i = 0; i < 5; ++i) {
            double s = 0.0;
            for (std::size_t a = 0; a < m; ++a) {
                s += K[i * 2 + a] * r[a];
            }
            d[i] = s;
        }
        double scale = 1.0;
        const double dp = std::hypot(d[0], d[1]);
        const double dh = std::abs(d[2]);
        if (dp > posBudget && dp > 0.0) {
            scale = std::min(scale, posBudget / dp);
        }
        if (dh > headBudget && dh > 0.0) {
            scale = std::min(scale, headBudget / dh);
        }
        for (std::size_t i = 0; i < 10; ++i) {
            K[i] *= scale;
        }
        std::array<double, 25> A{};
        for (std::size_t i = 0; i < 5; ++i) {
            for (std::size_t j = 0; j < 5; ++j) {
                double s = (i == j) ? 1.0 : 0.0;
                for (std::size_t a = 0; a < m; ++a) {
                    s -= K[i * 2 + a] * H[a * 5 + j];
                }
                A[i * 5 + j] = s;
            }
        }
        joseph(A, K, R, m);
        for (std::size_t i = 0; i < 5; ++i) {
            x[i] += d[i] * scale;
        }
        x[2] = Angle::radians(x[2]).radians();
        return std::sqrt(d2);
    }
};

}  // namespace

TEST_CASE("EkfFusion: one full tick matches an independently coded implementation of the header") {
    EkfFusionConfig cfg;
    cfg.maxNudgeRate = Velocity{1.0e6};  // no clamp in the first subcase
    cfg.maxHeadingNudgeRate = AngularVelocity{1.0e6};

    const double h = kH;
    const double h0 = 0.30;    // heading at the first (initialising) tick
    const double h1 = 0.32;    // …and at the second
    const double ux = 0.20;
    const double uy = 0.10;
    const double zx = 5.0;
    const double zy = -3.0;
    const double zsigma = 2.0;

    double clampBudget = 1.0e6;
    SUBCASE("unclamped") { clampBudget = 1.0e6; }
    SUBCASE("the rate clamp binds, and the covariance is updated with the REDUCED gain") {
        clampBudget = 0.05;
    }
    cfg.maxNudgeRate = Velocity{clampBudget / h};

    EkfFusion f{cfg};
    (void)f.fuse(Pose2d{Length{0.0}, Length{0.0}, Angle::radians(h0)}, {}, Time{h});
    const std::array<CorrectionProposal, 1> ps{fix(zx, zy, zsigma)};
    const FusionResult r =
        f.fuse(Pose2d{Length{ux}, Length{uy}, Angle::radians(h1)},
               std::span<const CorrectionProposal>{ps}, Time{h});

    // ── the oracle, from the header's own description of the tick ──
    Oracle o;
    o.x = {0.0, 0.0, h0, 0.0, 0.0};
    const double sp0 = cfg.initialPosStdDev.value();
    const double sh0 = cfg.initialHeadingStdDev.value();
    const double sv0 = cfg.initialVelStdDev.value();
    o.set(0, 0, sp0 * sp0);
    o.set(1, 1, sp0 * sp0);
    o.set(2, 2, sh0 * sh0);
    o.set(3, 3, sv0 * sv0);
    o.set(4, 4, sv0 * sv0);

    // STEP A — re-base theta, add Q for the interval
    o.x[2] = h1;
    const double travel = std::hypot(ux, uy);
    const double rot = std::abs(Angle::radians(h0).errorTo(Angle::radians(h1)));
    const double sq = cfg.posNoisePerInch * travel;                   // systematic, linear
    const double sqFloor = cfg.posNoiseRate.value() * h;              // random walk
    const double dPosVar = sq * sq + sqFloor * sqFloor;
    const double sqh = cfg.headingNoisePerRad * rot + cfg.headingDriftRate.value() * h;
    const double sqv = cfg.velNoise.value() * h;
    o.set(0, 0, o.get(0, 0) + dPosVar);
    o.set(1, 1, o.get(1, 1) + dPosVar);
    o.set(2, 2, o.get(2, 2) + sqh * sqh);
    o.set(3, 3, o.get(3, 3) + sqv * sqv);
    o.set(4, 4, o.get(4, 4) + sqv * sqv);

    // STEP B — the odometry velocity update, in the BODY frame: velocity states only
    {
        const double c = std::cos(o.x[2]);
        const double s = std::sin(o.x[2]);
        std::array<double, 10> H{};
        H[0 * 5 + 3] = 1.0;
        H[1 * 5 + 4] = 1.0;
        const std::array<double, 2> res{(ux * c + uy * s) / h - o.x[3],
                                        (-ux * s + uy * c) / h - o.x[4]};
        const double su = cfg.odomStdDev.value() + cfg.odomStdDevPerInch * travel;
        const double rv = (su / h) * (su / h);
        const std::array<double, 4> R{rv, 0.0, 0.0, rv};
        o.update(H, 2, res, R, false, false, 1.0e300, 1.0e300, 1.0e300);  // ungated
    }

    // STEP C — propagate position with the posterior velocity
    {
        const double c = std::cos(o.x[2]);
        const double s = std::sin(o.x[2]);
        const double vx = o.x[3];
        const double vy = o.x[4];
        std::array<double, 25> F{};
        for (std::size_t i = 0; i < 5; ++i) {
            F[i * 5 + i] = 1.0;
        }
        F[0 * 5 + 2] = -(vx * s + vy * c) * h;
        F[0 * 5 + 3] = c * h;
        F[0 * 5 + 4] = -s * h;
        F[1 * 5 + 2] = (vx * c - vy * s) * h;
        F[1 * 5 + 3] = s * h;
        F[1 * 5 + 4] = c * h;
        o.x[0] += (vx * c - vy * s) * h;
        o.x[1] += (vx * s + vy * c) * h;
        const std::array<double, 10> noK{};
        const std::array<double, 4> noR{};
        o.joseph(F, noK, noR, 0);
    }

    // STEP D — the position fix. The per-tick budget is charged first for however far steps
    // B and C already moved the position away from the handed prediction (header).
    {
        std::array<double, 10> H{};
        H[0 * 5 + 0] = 1.0;
        H[1 * 5 + 1] = 1.0;
        const std::array<double, 2> res{zx - o.x[0], zy - o.x[1]};
        const std::array<double, 4> R{zsigma * zsigma, 0.0, 0.0, zsigma * zsigma};
        const double already = std::hypot(o.x[0] - ux, o.x[1] - uy);
        const double posBudget = std::max(0.0, cfg.maxNudgeRate.value() * h - already);
        o.update(H, 2, res, R, false, true, cfg.gateSigma, posBudget,
                 cfg.maxHeadingNudgeRate.value() * h);
    }

    CHECK(r.applied);
    for (std::size_t i = 0; i < 5; ++i) {
        CAPTURE(i);
        CHECK(f.state(i) == doctest::Approx(o.x[i]).epsilon(1e-12));
        for (std::size_t j = 0; j < 5; ++j) {
            CAPTURE(j);
            CHECK(f.covariance(i, j) == doctest::Approx(o.get(i, j)).epsilon(1e-10));
        }
    }
    CHECK(r.x.value() == doctest::Approx(o.x[0]).epsilon(1e-12));
    CHECK(r.y.value() == doctest::Approx(o.x[1]).epsilon(1e-12));
    // heading was NOT moved by a position-only fix, in either implementation
    CHECK(o.x[2] == doctest::Approx(h1));
    CHECK(r.headingNudge.value() == 0.0);
}
