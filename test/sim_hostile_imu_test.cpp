// Adversarial tests for sim/hostile/imu_hostility.hpp — the direct attack on the
// < 1° budget. Each case pins one SHAPE the model claims (window, drift, noise,
// dropout, consistency, identity-when-zeroed) so a silently dead pathology cannot
// ship (the A3 liveness rule), and the drift case REPORTS the measured 60 s heading
// error as a number — the quantity the M2 acceptance stub and the A4 register need.

#include "doctest.h"

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::Rng;
using shulib::units::AngularVelocity;
using shulib::units::Time;

namespace {
constexpr double kDegToRad = Angle::kPi / 180.0;

/// A config with ONLY the named pathology live — the independent-injection idiom.
[[nodiscard]] ImuHostileConfig quietConfig() {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};
    cfg.rateBiasMax = AngularVelocity{0.0};
    cfg.headingNoiseSigmaRad = 0.0;
    cfg.yawRateNoiseSigmaRadPerS = 0.0;
    return cfg;
}
}  // namespace

TEST_CASE("hostile imu: an all-zero config is the exact identity") {
    ImuHostileModel m{quietConfig()};
    Rng rng{1};
    const Angle h = Angle::degrees(37.0);
    for (int i = 0; i < 50; ++i) {
        const Time t{0.01 * i};
        CHECK(m.imuHeading(h, t, rng).radians() == h.radians());  // exact, not approx
        CHECK(m.imuYawRate(AngularVelocity{1.25}, t, rng).value() == 1.25);
        CHECK(m.imuReady(true, t));
    }
}

TEST_CASE("hostile imu: calibration window — not ready, and the readings are garbage that MOVES") {
    ImuHostileConfig cfg = quietConfig();
    cfg.calibrationEnd = Time{2.0};
    ImuHostileModel m{cfg};
    Rng rng{42};
    const Angle truth = Angle::degrees(90.0);

    bool headingMoved = false;
    Angle prev = m.imuHeading(truth, Time{0.0}, rng);
    double worstGarbageErr = 0.0;
    for (int i = 1; i < 200; ++i) {
        const Time t{0.01 * i};
        CHECK_FALSE(m.imuReady(true, t));
        const Angle h = m.imuHeading(truth, t, rng);
        headingMoved = headingMoved || std::abs(prev.errorTo(h)) > 1e-6;
        worstGarbageErr = std::max(worstGarbageErr, std::abs(h.errorTo(truth)));
        prev = h;
    }
    CHECK(headingMoved);                       // garbage JUMPS — not a frozen zero
    CHECK(worstGarbageErr > 30.0 * kDegToRad);  // and it is nowhere near the truth
    // the instant the window ends, ready flips and readings snap to truth (quiet cfg)
    CHECK(m.imuReady(true, Time{2.0}));
    CHECK(m.imuHeading(truth, Time{2.0}, rng).approxEqual(truth, 1e-12));
}

TEST_CASE("hostile imu: per-boot rate bias drifts heading EXACTLY as bias*(t-calEnd), consistent with the reported rate") {
    ImuHostileConfig cfg = quietConfig();
    cfg.calibrationEnd = Time{1.0};
    cfg.rateBiasMax = AngularVelocity{kDegToRad / 60.0};  // ±1°/min
    ImuHostileModel m{cfg};
    Rng rng{7};
    const Angle truth = Angle::degrees(-20.0);

    (void)m.imuHeading(truth, Time{1.0}, rng);  // first call draws the boot bias
    const double bias = m.rateBiasRadPerS();
    CHECK(std::abs(bias) <= kDegToRad / 60.0);
    CHECK(bias != 0.0);  // a drawn bias of exactly 0 would mean the draw is dead

    for (double t : {1.0, 10.0, 30.0, 61.0}) {
        const Angle h = m.imuHeading(truth, Time{t}, rng);
        const double expected = bias * (t - 1.0);
        CHECK(truth.errorTo(h) == doctest::Approx(expected).epsilon(1e-12));
        // consistency: reported rate = true rate + the SAME bias (noise off)
        const auto r = m.imuYawRate(AngularVelocity{0.5}, Time{t}, rng);
        CHECK(r.value() == doctest::Approx(0.5 + bias).epsilon(1e-12));
    }
}

TEST_CASE("hostile imu: the 60 s drift number, measured and reported (worst over 8 boots)") {
    // The quantity F2's <1° budget lives or dies on. Bias is uniform in ±1°/min
    // (PROVISIONAL — A4 register HA-20), so the worst boot approaches 1° at 60 s — measured here, with
    // noise live, exactly as the acceptance test will see it.
    ImuHostileConfig cfg;  // FULL defaults: window + drift + noise
    double worstEndErrDeg = 0.0;
    double worstBiasDegPerMin = 0.0;
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        ImuHostileModel m{cfg};
        Rng rng{seed};
        const Angle truth = Angle::degrees(0.0);
        double endErr = 0.0;
        for (int i = 0; i <= 6200; ++i) {  // 62 s at 10 ms, spanning calibration + 60 s
            const Time t{0.01 * i};
            const Angle h = m.imuHeading(truth, t, rng);
            if (i == 6200) {
                endErr = std::abs(truth.errorTo(h));
            }
        }
        worstEndErrDeg = std::max(worstEndErrDeg, endErr / kDegToRad);
        worstBiasDegPerMin =
            std::max(worstBiasDegPerMin, std::abs(m.rateBiasRadPerS()) / kDegToRad * 60.0);
    }
    MESSAGE("60s drift, worst over 8 boots: ", worstEndErrDeg,
            " deg (worst |bias| drawn: ", worstBiasDegPerMin, " deg/min)");
    // The SHAPE claims: drift is live (well above the noise floor) and bounded by
    // the configured ±1°/min budget plus noise. NOT an F2 verdict — that is the
    // acceptance test's job against the full stack.
    CHECK(worstEndErrDeg > 0.2);   // a dead drift term cannot produce this
    CHECK(worstEndErrDeg < 1.2);   // |bias|·60s ≤ 1° plus noise margin
}

TEST_CASE("hostile imu: heading noise has the configured sigma (drift off)") {
    ImuHostileConfig cfg = quietConfig();
    cfg.headingNoiseSigmaRad = 0.05 * kDegToRad;
    ImuHostileModel m{cfg};
    Rng rng{2026};
    const Angle truth = Angle::degrees(45.0);
    double sum = 0.0;
    double sumSq = 0.0;
    constexpr int kN = 20000;
    for (int i = 0; i < kN; ++i) {
        const double e = truth.errorTo(m.imuHeading(truth, Time{0.01 * i}, rng));
        sum += e;
        sumSq += e * e;
    }
    const double mean = sum / kN;
    const double sigma = std::sqrt(sumSq / kN - mean * mean);
    CHECK(std::abs(mean) < 0.01 * kDegToRad);                       // unbiased
    CHECK(sigma == doctest::Approx(0.05 * kDegToRad).epsilon(0.05));  // the right sigma
}

TEST_CASE("hostile imu: mid-run dropout freezes the last emitted values and drops ready, forever") {
    ImuHostileConfig cfg = quietConfig();
    cfg.dropoutAt = Time{3.0};
    ImuHostileModel m{cfg};
    Rng rng{5};

    // healthy: tracks a moving truth
    const Angle before = m.imuHeading(Angle::degrees(10.0), Time{2.99}, rng);
    CHECK(before.approxEqual(Angle::degrees(10.0), 1e-12));
    (void)m.imuYawRate(AngularVelocity{0.8}, Time{2.99}, rng);
    CHECK(m.imuReady(true, Time{2.99}));

    // dropped: truth keeps moving, the sensor does not
    for (double t : {3.0, 3.5, 10.0, 60.0}) {
        CHECK_FALSE(m.imuReady(true, Time{t}));
        const Angle h = m.imuHeading(Angle::degrees(170.0), Time{t}, rng);
        CHECK(h.approxEqual(Angle::degrees(10.0), 1e-12));  // frozen at the LAST emitted
        CHECK(m.imuYawRate(AngularVelocity{-2.0}, Time{t}, rng).value()
              == doctest::Approx(0.8));  // frozen rate too — and finite, never NaN
    }
}

TEST_CASE("hostile imu: seeded determinism — same seed identical, different seed different bias") {
    ImuHostileConfig cfg;  // full defaults
    ImuHostileModel a{cfg};
    ImuHostileModel b{cfg};
    Rng ra{99};
    Rng rb{99};
    for (int i = 0; i < 500; ++i) {
        const Time t{0.01 * i};
        CHECK(a.imuHeading(Angle::degrees(30.0), t, ra).radians()
              == b.imuHeading(Angle::degrees(30.0), t, rb).radians());
        CHECK(a.imuYawRate(AngularVelocity{0.1}, t, ra).value()
              == b.imuYawRate(AngularVelocity{0.1}, t, rb).value());
    }
    CHECK(a.rateBiasRadPerS() == b.rateBiasRadPerS());

    ImuHostileModel c{cfg};
    Rng rc{100};
    (void)c.imuHeading(Angle::degrees(30.0), Time{0.0}, rc);
    CHECK(c.rateBiasRadPerS() != a.rateBiasRadPerS());  // a different boot
}

TEST_CASE("hostile imu: rejects an out-of-range config") {
    ImuHostileConfig bad;
    bad.rateBiasMax = AngularVelocity{-1.0};
    CHECK_THROWS_AS((ImuHostileModel{bad}), shulib::PreconditionError);
    ImuHostileConfig bad2;
    bad2.headingNoiseSigmaRad = -0.1;
    CHECK_THROWS_AS((ImuHostileModel{bad2}), shulib::PreconditionError);
}
