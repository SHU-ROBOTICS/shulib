// Adversarial tests for ComplementaryFusion — the gated-nudge fusion policy. Built to fail if the
// "never snap, always bounded" contract (decision #4) breaks: a far fix must be REJECTED, a close
// fix must move the pose by at most the per-tick budget (never to the measurement), confidence
// scales the pull, and a non-finite proposal must never poison the result.

#include <array>
#include <cmath>
#include <limits>
#include <span>

#include "doctest.h"

#include "shulib/core/check.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
using shulib::localization::CorrectionProposal;
using shulib::localization::FusionResult;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {
CorrectionProposal prop(double x, double y, double conf) {
    CorrectionProposal p{};
    p.valid = true;
    p.fieldPose = Pose2d{Length{x}, Length{y}, Angle{}};
    p.confidence = conf;
    p.positionStdDev = Length{0.5};
    return p;
}
const Pose2d kOrigin{};  // (0,0,0)
}  // namespace

TEST_CASE("ComplementaryFusion: no proposals → position unchanged (dead-reckon)") {
    ComplementaryFusion f{};
    const FusionResult r = f.fuse(kOrigin, {}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.0));
    CHECK(r.y.value() == doctest::Approx(0.0));
    CHECK_FALSE(r.applied);  // nothing contributed
    CHECK_FALSE(r.gated);
    CHECK_FALSE(r.clamped);
}

// THE never-snap gate: a fix farther than innovationGate is rejected outright.
TEST_CASE("ComplementaryFusion: a fix beyond the innovation gate is rejected, pose held") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{100.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 1> ps{prop(100.0, 0.0, 1.0)};  // 100" off ≫ 12" gate
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.0));  // not pulled at all
    CHECK(r.y.value() == doctest::Approx(0.0));
    CHECK(r.gated);
    CHECK_FALSE(r.applied);  // the only proposal was rejected → nothing applied
}

// THE never-snap clamp: an in-gate fix moves the pose by at most maxNudgeRate·dt, never to it.
TEST_CASE("ComplementaryFusion: an in-gate fix nudges by the per-tick budget, never snaps") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{10.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 1> ps{prop(10.0, 0.0, 1.0)};  // wants 0.15·10 = 1.5"
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.5));  // clamped to 10·0.05 = 0.5", NOT 1.5 and NOT 10
    CHECK(r.y.value() == doctest::Approx(0.0));  // direction preserved
    CHECK(r.clamped);
    CHECK(r.applied);  // an in-gate fix did contribute
}

TEST_CASE("ComplementaryFusion: confidence scales the pull (no clamp regime)") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{100.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 1> ps{prop(10.0, 0.0, 0.5)};  // weight 0.075 → 0.75"
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.75));  // 0.15·0.5·10, well under the 5" budget
    CHECK_FALSE(r.clamped);
    CHECK_FALSE(r.gated);
}

TEST_CASE("ComplementaryFusion: zero-confidence proposal exerts no pull") {
    ComplementaryFusion f{};
    const std::array<CorrectionProposal, 1> ps{prop(10.0, 0.0, 0.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.0));
}

TEST_CASE("ComplementaryFusion: a non-finite proposal is rejected, result stays finite") {
    ComplementaryFusion f{};
    const std::array<CorrectionProposal, 1> ps{
        prop(std::numeric_limits<double>::quiet_NaN(), 0.0, 1.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(std::isfinite(r.x.value()));
    CHECK(r.x.value() == doctest::Approx(0.0));  // not poisoned
    CHECK(r.gated);
}

TEST_CASE("ComplementaryFusion: two in-budget proposals sum") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{100.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 2> ps{prop(10.0, 0.0, 1.0), prop(0.0, 10.0, 1.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(1.5));  // each pulls 1.5", well under the 5" budget
    CHECK(r.y.value() == doctest::Approx(1.5));
    CHECK_FALSE(r.clamped);
}

// Two proposals of DIFFERENT magnitude: the big one must be clamped PER-PROPOSAL before summing,
// else it dominates the combined direction. Distinguishes the per-proposal clamp from the sum clamp.
TEST_CASE("ComplementaryFusion: each proposal is clamped before summing (big one can't dominate)") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{10.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    // prop1 wants 1.5" (clamped per-proposal to 0.5"); prop2 wants 0.15" (under budget, untouched).
    const std::array<CorrectionProposal, 2> ps{prop(10.0, 0.0, 1.0), prop(0.0, 1.0, 1.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(0.478913));  // sum (0.5,0.15) then clamped to 0.5 mag
    CHECK(r.y.value() == doctest::Approx(0.143674));  // (without per-proposal clamp → 0.498,0.050)
    CHECK(r.clamped);
}

TEST_CASE("ComplementaryFusion: the SUM of proposals is clamped so they can't out-vote the budget") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{40.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 2> ps{prop(10.0, 0.0, 1.0), prop(0.0, 10.0, 1.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    // each nudge 1.5" is under the 2" budget, but their sum (mag 2.121) is clamped back to 2".
    CHECK(std::hypot(r.x.value(), r.y.value()) == doctest::Approx(2.0));
    CHECK(r.x.value() == doctest::Approx(1.414214));  // direction (45°) preserved
    CHECK(r.clamped);
}

// confidence > 1 must not amplify the gain beyond maxGain (a corrector can't buy extra pull).
TEST_CASE("ComplementaryFusion: confidence is clamped to [0,1] (no gain amplification)") {
    ComplementaryFusion f{ComplementaryFusionConfig{.maxNudgeRate = Velocity{100.0},
                                                    .innovationGate = Length{12.0}, .maxGain = 0.15}};
    const std::array<CorrectionProposal, 1> ps{prop(10.0, 0.0, 5.0)};  // confidence 5 ⇒ clamped to 1
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(r.x.value() == doctest::Approx(1.5));  // 0.15·1·10, NOT 0.15·5·10 = 7.5
}

// A non-finite confidence (finite position) must be rejected, not poison the result with NaN.
TEST_CASE("ComplementaryFusion: a non-finite confidence is rejected, result stays finite") {
    ComplementaryFusion f{};
    const std::array<CorrectionProposal, 1> ps{prop(2.0, 0.0, std::numeric_limits<double>::infinity())};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.05});
    CHECK(std::isfinite(r.x.value()));
    CHECK(r.x.value() == doctest::Approx(0.0));
    CHECK(r.gated);
    CHECK_FALSE(r.applied);
}

// dt == 0 ⇒ per-tick budget is 0 ⇒ nothing can be applied, even for an in-gate fix (not "corrected").
TEST_CASE("ComplementaryFusion: a zero-budget (dt==0) tick applies nothing") {
    ComplementaryFusion f{};
    const std::array<CorrectionProposal, 1> ps{prop(2.0, 0.0, 1.0)};
    const FusionResult r = f.fuse(kOrigin, std::span<const CorrectionProposal>{ps}, Time{0.0});
    CHECK(r.x.value() == doctest::Approx(0.0));  // no movement
    CHECK_FALSE(r.applied);                       // and not reported as an applied correction
}

TEST_CASE("ComplementaryFusion: rejects an out-of-range config") {
    CHECK_THROWS_AS((ComplementaryFusion{ComplementaryFusionConfig{.maxGain = 0.0}}), PreconditionError);
    CHECK_THROWS_AS((ComplementaryFusion{ComplementaryFusionConfig{.maxGain = 1.5}}), PreconditionError);
    CHECK_THROWS_AS((ComplementaryFusion{ComplementaryFusionConfig{.innovationGate = Length{0.0}}}),
                    PreconditionError);
    CHECK_THROWS_AS((ComplementaryFusion{ComplementaryFusionConfig{.maxNudgeRate = Velocity{-1.0}}}),
                    PreconditionError);
}
