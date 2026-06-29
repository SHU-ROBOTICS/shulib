// Seam-level tests for the localization fusion interfaces (CorrectionProposal / ICorrector /
// IPoseSource / IFusionPolicy). Thin by design — the heavy adversarial work lives in the
// ComplementaryFusion and Localizer tests; here we just pin the value-type defaults and the two
// stub/double correctors so the seam contract is nailed before anything is built on it.

#include "doctest.h"

#include "shulib/localization/correction.hpp"
#include "shulib/localization/fake/fake_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::localization::CorrectionProposal;
using shulib::localization::NullCorrector;
using shulib::localization::fake::FakeCorrector;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::Length;
using shulib::units::Time;

TEST_CASE("CorrectionProposal: a default-constructed proposal is invalid (no accidental pull)") {
    CorrectionProposal p{};
    CHECK_FALSE(p.valid);
    CHECK(p.confidence == doctest::Approx(0.0));
    CHECK_FALSE(p.providesHeading);
}

TEST_CASE("NullCorrector: always proposes invalid and has a stable name") {
    NullCorrector c;
    const CorrectionProposal p = c.propose(Pose2d{}, Time{0.01});
    CHECK_FALSE(p.valid);
    CHECK(std::string{c.name()} == "null");
}

TEST_CASE("FakeCorrector: returns the injected proposal and records what it was asked") {
    FakeCorrector c{"gps"};
    CorrectionProposal in{};
    in.valid = true;
    in.fieldPose = Pose2d{Length{3.0}, Length{4.0}, Angle::degrees(10.0)};
    in.confidence = 0.8;
    in.positionStdDev = Length{0.5};
    c.setProposal(in);

    const Pose2d predicted{Length{1.0}, Length{2.0}, Angle::degrees(5.0)};
    const CorrectionProposal out = c.propose(predicted, Time{0.02});

    CHECK(out.valid);
    CHECK(out.fieldPose.x().value() == doctest::Approx(3.0));
    CHECK(out.confidence == doctest::Approx(0.8));
    CHECK(c.calls() == 1);
    CHECK(c.lastPredicted().x().value() == doctest::Approx(1.0));   // got the PREDICTED pose
    CHECK(c.lastDt().value() == doctest::Approx(0.02));
    CHECK(std::string{c.name()} == "gps");
}
