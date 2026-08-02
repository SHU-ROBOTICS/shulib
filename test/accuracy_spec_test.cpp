// F2 accuracy-spec encoding (master plan §7 / Freeze F2).
//
// The pinned target values, the internal-consistency invariants, and the
// acceptance tests. Status per tier:
//  * [acceptance][M2] went LIVE at chunk A3: the dead-reckon stack now runs against
//    MODELED IMU drift/noise (sim/hostile/), so the <1° assertion finally claims
//    something (against perfect sensors it was vacuous — the A2 record explains
//    why it stayed skipped until hostility existed).
//  * [acceptance][M3] stubs stay skipped — no correctors/docking exist yet.

#include "doctest.h"

#include <algorithm>
#include <cmath>

#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/spec/accuracy.hpp"
#include "shulib/units/quantity.hpp"

using namespace shulib::spec;

TEST_CASE("F2 spec guard: the locked target values (a tripwire on the Freeze)") {
    CHECK(kHeadingErrorMaxDeg            == doctest::Approx(1.0));
    CHECK(kPositionErrorEndOfRun.value() == doctest::Approx(1.0));
    CHECK(kRepeatability.value()         == doctest::Approx(0.75));
    CHECK(kDockedPositionError.value()   == doctest::Approx(0.25));
}

TEST_CASE("F2 spec: internal-consistency invariants the target set must satisfy") {
    CHECK(kDockedPositionError < kPositionErrorEndOfRun);   // docking is tighter than dead-reckon
    CHECK(kRepeatability       < kPositionErrorEndOfRun);   // repeatability tighter than absolute
    CHECK(kDockedHeadingTypicalDeg < kHeadingErrorMaxDeg);  // the aspiration beats the hard cap
    CHECK(kHeadingErrorMaxDeg  > 0.0);
}

// ----- Acceptance tests — [M2] live since A3; [M3] stubs stay skipped -----

TEST_CASE("[acceptance][M2] dead-reckon heading holds over a 60s straight-line test") {
    // LIVE SINCE A3, and what it now honestly claims: the full M2 stack
    // (PilonsOdometry + Localizer, heading IMU-owned) holds |heading error| <
    // kHeadingErrorMaxDeg over 60 s of straight-line driving UNDER THE MODELED
    // hostile world (FullHostility defaults: 2 s calibration window, per-boot drift
    // uniform in ±1°/min, heading noise sigma 0.05°, latency, quantization, sag).
    //
    // Scope, stated honestly:
    //  * The drift bound is PROVISIONAL (A4 register; R4 measures the real rate).
    //    At exactly the ±1°/min bound the F2 margin is ZERO by construction
    //    (1°/min × 60 s = 1°) — heading quality on the REAL IMU is the ceiling on
    //    this target (build-order R4), and Phase E's correctors are what buy
    //    margin. This test proves the STACK adds no error of its own on top of
    //    the sensor's, across a fair sweep of boots.
    //  * Ten seeded boots sweep the per-boot drift distribution, INCLUDING the
    //    near-bound draw (seed 10 draws −0.93°/min — verified against the pinned
    //    SplitMix64 first-draw mapping). Seeds are representative, not curated:
    //    1..10, no gaps.
    //  * "Straight-line" = alternating straight legs, no commanded rotation (the
    //    field-honest shape of the bench test the spec names).
    const auto kin = shulib::kinematics::xDrive(shulib::units::Length{7.0});
    constexpr double kDt = 0.01;
    double worstEndErrDeg = 0.0;
    double worstRunErrDeg = 0.0;

    for (std::uint64_t seed = 1; seed <= 10; ++seed) {
        shulib::sim::FullHostility full{};
        shulib::sim::SimHarnessConfig cfg;
        cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
        cfg.plant.seed = seed;
        shulib::sim::SimHarness h{kin, cfg, nullptr, &full.model()};
        shulib::localization::PilonsOdometry odom{
            h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
        shulib::localization::ComplementaryFusion fusion{};
        shulib::localization::Localizer loc{h.clock(), h.imu(), odom, fusion};

        double runWorstDeg = 0.0;
        bool live = false;
        int liveTicks = 0;
        // 2.5 s boot hold (calibration + settle: the C1 wait-for-live contract),
        // then 60.0 s of straight legs, 5 s each, alternating direction.
        h.runTicks(6250, shulib::units::Time{kDt}, [&](int) {
            loc.update();
            live = live || loc.qualityClass()
                               != shulib::localization::Localizer::Quality::Uninitialized;
            if (!live) {
                return;
            }
            ++liveTicks;
            const double errDeg =
                std::abs(h.truePose().heading().errorTo(loc.pose().heading()))
                / (shulib::math::Angle::kPi / 180.0);
            runWorstDeg = std::max(runWorstDeg, errDeg);
            const double vx = ((liveTicks / 500) % 2 == 0) ? 15.0 : -15.0;
            h.commandBodyTwist(shulib::math::ChassisSpeeds{shulib::units::Velocity{vx},
                                                           shulib::units::Velocity{0.0},
                                                           shulib::units::AngularVelocity{0.0}});
        });
        REQUIRE(live);
        REQUIRE(liveTicks >= 6000);  // the claim really spans 60 s of driving
        const double endErrDeg =
            std::abs(h.truePose().heading().errorTo(loc.pose().heading()))
            / (shulib::math::Angle::kPi / 180.0);

        // THE F2 CLAIM, per boot: end-of-60s heading error under the cap.
        CHECK(endErrDeg < kHeadingErrorMaxDeg);
        worstEndErrDeg = std::max(worstEndErrDeg, endErrDeg);
        worstRunErrDeg = std::max(worstRunErrDeg, runWorstDeg);
    }
    MESSAGE("[acceptance][M2] 60s dead-reckon heading: worst end-of-run error ",
            worstEndErrDeg, " deg, worst instantaneous ", worstRunErrDeg,
            " deg across 10 boots (cap ", kHeadingErrorMaxDeg, " deg)");
}

TEST_CASE("[acceptance][M3] end-of-60s fused pose within F2 targets"
          * doctest::skip()) {
    // TODO(M3): full fused run (GPS + AprilTag) with contact + spins;
    // assert position <= kPositionErrorEndOfRun AND |heading| < kHeadingErrorMaxDeg.
}

TEST_CASE("[acceptance][M3] vision docking nests a 1.6in pin within kDockedPositionError"
          * doctest::skip()) {
    // TODO(M3): DockToGoal closed-loop visual-servo;
    // assert final alignment <= kDockedPositionError AND |heading| < kHeadingErrorMaxDeg.
}
