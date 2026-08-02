// Adversarial tests for sim/hostile/gps_hostility.hpp. Each case pins one claimed
// SHAPE: decimation (the double-counting hazard), noise vs the DECOUPLED reported
// rms, the three no-fix flavours (off-strip / window / dropout) with the IGps
// finiteness contract held throughout, the bad-fix confident lie, and seeded
// determinism. The origin-until-first-fix stale pose is pinned on purpose: it is
// the value that DRAGS code which trusts a no-fix pose, so it must stay wrong.

#include "doctest.h"

#include <cmath>

#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/gps_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::sim::GpsBadFixWindow;
using shulib::sim::GpsHostileConfig;
using shulib::sim::GpsHostileModel;
using shulib::sim::GpsNoFixWindow;
using shulib::sim::GpsTruth;
using shulib::sim::Rng;
using shulib::units::Length;
using shulib::units::Time;

namespace {
[[nodiscard]] GpsHostileConfig quietConfig() {
    GpsHostileConfig cfg;
    cfg.noiseSigmaIn = 0.0;
    cfg.headingNoiseSigmaRad = 0.0;
    cfg.updatePeriod = Time{0.0};
    return cfg;
}

[[nodiscard]] GpsTruth truthAt(double x, double y, double headingDeg = 0.0) {
    return GpsTruth{Pose2d{Length{x}, Length{y}, Angle::degrees(headingDeg)}, Length{0.1}, true};
}

[[nodiscard]] bool finitePose(const Pose2d& p) {
    return std::isfinite(p.x().value()) && std::isfinite(p.y().value());
}
}  // namespace

TEST_CASE("hostile gps: a quiet config passes the truth through exactly (fresh sample every call)") {
    GpsHostileModel m{quietConfig()};
    Rng rng{1};
    for (int i = 0; i < 20; ++i) {
        const GpsTruth out = m.gps(truthAt(3.0 + i, -2.0, 45.0), Time{0.01 * i}, rng);
        CHECK(out.pose.x().value() == 3.0 + i);
        CHECK(out.pose.y().value() == -2.0);
        CHECK(out.hasFix);
        CHECK(out.rmsError.value() == doctest::Approx(1.0));  // the sensor's CLAIM
    }
}

TEST_CASE("hostile gps: decimation re-reports the SAME fix until the camera cadence elapses") {
    GpsHostileConfig cfg = quietConfig();
    cfg.updatePeriod = Time{0.05};
    GpsHostileModel m{cfg};
    Rng rng{1};

    // truth moves every 10 ms; the sensor must hold each sample for 5 ticks
    const GpsTruth first = m.gps(truthAt(0.0, 0.0), Time{0.00}, rng);
    for (int i = 1; i < 5; ++i) {
        const GpsTruth held = m.gps(truthAt(1.0 * i, 0.0), Time{0.01 * i}, rng);
        CHECK(held.pose.x().value() == first.pose.x().value());  // identical, not close
    }
    const GpsTruth fresh = m.gps(truthAt(5.0, 0.0), Time{0.05}, rng);
    CHECK(fresh.pose.x().value() == 5.0);  // a new sample of the CURRENT truth
}

TEST_CASE("hostile gps: fresh samples carry sigma-shaped noise; the reported rms stays the config's claim") {
    GpsHostileConfig cfg = quietConfig();
    cfg.noiseSigmaIn = 0.7;
    GpsHostileModel m{cfg};
    Rng rng{2026};
    double sum = 0.0;
    double sumSq = 0.0;
    constexpr int kN = 20000;
    for (int i = 0; i < kN; ++i) {
        const GpsTruth out = m.gps(truthAt(10.0, -4.0), Time{0.01 * i}, rng);
        const double ex = out.pose.x().value() - 10.0;
        sum += ex;
        sumSq += ex * ex;
        CHECK(out.rmsError.value() == doctest::Approx(1.0));  // claim ≠ actual sigma
    }
    const double mean = sum / kN;
    const double sigma = std::sqrt(sumSq / kN - mean * mean);
    CHECK(std::abs(mean) < 0.02);
    CHECK(sigma == doctest::Approx(0.7).epsilon(0.05));
}

TEST_CASE("hostile gps: off-strip (Driving Skills) — never a fix, inflated rms, finite stale pose") {
    GpsHostileConfig cfg = quietConfig();
    cfg.offStrip = true;
    GpsHostileModel m{cfg};
    Rng rng{1};
    for (int i = 0; i < 100; ++i) {
        const GpsTruth out = m.gps(truthAt(20.0, 30.0), Time{0.01 * i}, rng);
        CHECK_FALSE(out.hasFix);
        CHECK(out.rmsError.value() == doctest::Approx(99.0));
        REQUIRE(finitePose(out.pose));            // the IGps contract
        CHECK(out.pose.x().value() == 0.0);       // origin: deliberately WRONG,
        CHECK(out.pose.y().value() == 0.0);       // so trusting it gets caught
    }
}

TEST_CASE("hostile gps: a no-fix window drops the fix, holds the last sample finite, then recovers") {
    GpsHostileConfig cfg = quietConfig();
    cfg.noFixWindows = {GpsNoFixWindow{Time{1.0}, Time{2.0}}};
    GpsHostileModel m{cfg};
    Rng rng{1};

    const GpsTruth before = m.gps(truthAt(5.0, 5.0), Time{0.99}, rng);
    CHECK(before.hasFix);

    for (double t : {1.0, 1.5, 1.99}) {
        const GpsTruth in = m.gps(truthAt(50.0, 50.0), Time{t}, rng);
        CHECK_FALSE(in.hasFix);
        CHECK(in.rmsError.value() == doctest::Approx(99.0));
        REQUIRE(finitePose(in.pose));
        CHECK(in.pose.x().value() == 5.0);  // the STALE held sample, not the moving truth
    }
    const GpsTruth after = m.gps(truthAt(60.0, 60.0), Time{2.0}, rng);
    CHECK(after.hasFix);
    CHECK(after.pose.x().value() == 60.0);
}

TEST_CASE("hostile gps: dropout is a permanent no-fix from that instant") {
    GpsHostileConfig cfg = quietConfig();
    cfg.dropoutAt = Time{3.0};
    GpsHostileModel m{cfg};
    Rng rng{1};
    CHECK(m.gps(truthAt(1.0, 1.0), Time{2.9}, rng).hasFix);
    for (double t : {3.0, 10.0, 600.0}) {
        const GpsTruth out = m.gps(truthAt(2.0, 2.0), Time{t}, rng);
        CHECK_FALSE(out.hasFix);
        REQUIRE(finitePose(out.pose));
    }
}

TEST_CASE("hostile gps: a bad-fix window is a CONFIDENT lie — truth+offset, normal rms, hasFix true") {
    GpsHostileConfig cfg = quietConfig();
    cfg.badFixWindows = {GpsBadFixWindow{Time{1.0}, Time{2.0}, Length{8.0}, Length{-6.0}}};
    GpsHostileModel m{cfg};
    Rng rng{1};

    const GpsTruth honest = m.gps(truthAt(10.0, 10.0), Time{0.5}, rng);
    CHECK(honest.pose.x().value() == 10.0);

    const GpsTruth lie = m.gps(truthAt(10.0, 10.0), Time{1.5}, rng);
    CHECK(lie.hasFix);                                       // it CLAIMS to be fine
    CHECK(lie.rmsError.value() == doctest::Approx(1.0));     // with normal confidence
    CHECK(lie.pose.x().value() == doctest::Approx(18.0));    // and it is 10 inches off
    CHECK(lie.pose.y().value() == doctest::Approx(4.0));

    const GpsTruth recovered = m.gps(truthAt(10.0, 10.0), Time{2.5}, rng);
    CHECK(recovered.pose.x().value() == doctest::Approx(10.0));
}

TEST_CASE("hostile gps: seeded determinism — same seed identical stream") {
    GpsHostileConfig cfg;  // full defaults (noise + decimation live)
    GpsHostileModel a{cfg};
    GpsHostileModel b{cfg};
    Rng ra{55};
    Rng rb{55};
    for (int i = 0; i < 300; ++i) {
        const GpsTruth oa = a.gps(truthAt(0.1 * i, -0.2 * i), Time{0.01 * i}, ra);
        const GpsTruth ob = b.gps(truthAt(0.1 * i, -0.2 * i), Time{0.01 * i}, rb);
        CHECK(oa.pose.x().value() == ob.pose.x().value());
        CHECK(oa.pose.y().value() == ob.pose.y().value());
        CHECK(oa.hasFix == ob.hasFix);
    }
}

TEST_CASE("hostile gps: rejects an out-of-range config") {
    GpsHostileConfig bad;
    bad.noiseSigmaIn = -1.0;
    CHECK_THROWS_AS((GpsHostileModel{bad}), shulib::PreconditionError);
    GpsHostileConfig bad2;
    bad2.noFixWindows = {GpsNoFixWindow{Time{2.0}, Time{1.0}}};
    CHECK_THROWS_AS((GpsHostileModel{bad2}), shulib::PreconditionError);
}
