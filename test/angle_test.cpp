// Adversarial tests for shulib::math::Angle.
//
// Each case targets a SPECIFIC way the wrap/shortest-error logic could be wrong
// (sign of fmod, the (-π,π] boundary tie-break, naive subtraction for shortest
// path, NaN propagation), and the sweeps assert invariants across the whole
// input space so edge cases surface themselves. See the roadmap's Testing
// discipline. None of these is a "1+1==2" confirmation.

#include <cmath>
#include <limits>
#include <random>

#include "doctest.h"
#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"

using shulib::math::Angle;

namespace {
constexpr double kPi = Angle::kPi;
constexpr double kTol = 1e-9;          // fp slack for exact-arithmetic cases
constexpr double kDeg = kPi / 180.0;   // degrees -> radians
}  // namespace

TEST_CASE("wrap: reductions a naive fmod-based impl gets wrong") {
    CHECK(Angle::degrees(270).degrees()  == doctest::Approx(-90.0));   // fmod keeps the sign
    CHECK(Angle::degrees(-270).degrees() == doctest::Approx(90.0));
    CHECK(Angle::degrees(360).degrees()  == doctest::Approx(0.0));
    CHECK(Angle::degrees(540).degrees()  == doctest::Approx(180.0));   // 3π -> π
    CHECK(Angle::degrees(720).degrees()  == doctest::Approx(0.0));
}

TEST_CASE("wrap: the (-π, π] boundary tie-break is +180, never -180") {
    CHECK(Angle::degrees(180).degrees()  == doctest::Approx(180.0));
    CHECK(Angle::degrees(-180).degrees() == doctest::Approx(180.0));   // -π folds up to +π
    CHECK(Angle::radians(-kPi).radians() == doctest::Approx(kPi));     // nothing ever stores -π
}

TEST_CASE("wrap: range invariant over a wide sweep — result always in (-π, π]") {
    std::mt19937 rng(12345u);
    std::uniform_real_distribution<double> dist(-10.0 * kPi, 10.0 * kPi);
    for (int i = 0; i < 100000; ++i) {
        const double w = Angle::radians(dist(rng)).radians();
        CHECK(w <=  kPi + kTol);
        CHECK(w >  -kPi - kTol);
    }
}

TEST_CASE("wrap: idempotent — re-wrapping an Angle is a no-op") {
    std::mt19937 rng(777u);
    std::uniform_real_distribution<double> dist(-10.0 * kPi, 10.0 * kPi);
    for (int i = 0; i < 10000; ++i) {
        const Angle a = Angle::radians(dist(rng));
        CHECK(Angle::radians(a.radians()).approxEqual(a, kTol));
    }
}

TEST_CASE("wrap: periodic — adding any k·2π is the same heading") {
    std::mt19937 rng(2024u);
    std::uniform_real_distribution<double> dist(-kPi, kPi);
    for (int i = 0; i < 10000; ++i) {
        const double base = dist(rng);
        const Angle a = Angle::radians(base);
        for (int k = -3; k <= 3; ++k) {
            const Angle b = Angle::radians(base + static_cast<double>(k) * 2.0 * kPi);
            CHECK(a.approxEqual(b, 1e-6));
        }
    }
}

TEST_CASE("errorTo: shortest signed path, not naive subtraction") {
    CHECK(Angle::degrees(170).errorTo(Angle::degrees(-170)) == doctest::Approx(20.0 * kDeg));
    CHECK(Angle::degrees(-170).errorTo(Angle::degrees(170)) == doctest::Approx(-20.0 * kDeg));
    CHECK(Angle::degrees(10).errorTo(Angle::degrees(40))    == doctest::Approx(30.0 * kDeg));
}

TEST_CASE("errorTo: exact-180 antipode resolves deterministically to +π both ways") {
    CHECK(Angle::degrees(0).errorTo(Angle::degrees(180))  == doctest::Approx(kPi));
    CHECK(Angle::degrees(90).errorTo(Angle::degrees(-90)) == doctest::Approx(kPi));
    CHECK(Angle::degrees(-90).errorTo(Angle::degrees(90)) == doctest::Approx(kPi));
}

TEST_CASE("errorTo: anti-symmetric except at the antipode (a documented consequence of the tie-break)") {
    std::mt19937 rng(99u);
    std::uniform_real_distribution<double> dist(-kPi, kPi);
    for (int i = 0; i < 10000; ++i) {
        const Angle a = Angle::radians(dist(rng));
        const Angle b = Angle::radians(dist(rng));
        const double e = a.errorTo(b);
        CHECK(std::abs(e) <= kPi + kTol);                       // range
        if (std::abs(std::abs(e) - kPi) < 1e-6) {
            continue;                                            // antipodal: both directions give +π
        }
        CHECK(e == doctest::Approx(-b.errorTo(a)));
    }
}

TEST_CASE("deg<->rad round-trips inside the canonical interval") {
    for (int d = -179; d <= 180; ++d) {
        CHECK(Angle::degrees(static_cast<double>(d)).degrees()
              == doctest::Approx(static_cast<double>(d)));
    }
    CHECK(Angle::radians(1.0).radians() == doctest::Approx(1.0));
}

TEST_CASE("non-finite input is rejected, never silently propagated") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    // (void)-cast: we are testing that construction THROWS, so discarding the
    // (never-produced) [[nodiscard]] result is intentional.
    CHECK_THROWS_AS((void)Angle::degrees(nan), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Angle::radians(nan), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Angle::radians(inf), shulib::PreconditionError);
    CHECK_THROWS_AS((void)Angle::degrees(-inf), shulib::PreconditionError);
}
