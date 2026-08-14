// GpsCorrector — the first REAL corrector (chunk E2). Adversarial unit tests.
//
// Every case names, in its comment, the bug it would catch. The organising idea is that
// each of the corrector's decisions is a place a robot can be lost differently, so each
// gets its own case rather than being folded into one "it works" test:
//
//   * adaptive R          — the sensor's claimed error becoming the fix's sigma
//   * no fix / off-strip  — the Driving-Skills path, which must be VISIBLE, not quiet
//   * staleness           — one measurement folded once, not five times
//   * sensor quality      — a source claiming to see badly
//   * high yaw rate       — the geometry's worst moment
//   * latency             — a fix describes where the robot WAS
//   * the gate            — normalized innovation, and its anti-lockout widening
//   * heading             — the GPS's is never allowed out (tension T3)
//   * never-snap          — the corrector proposes; it cannot write a pose
//
// UNITS NOTE: this file works entirely in CANONICAL units (inches, seconds, radians),
// because IGps::pose()/rmsError() are canonical by contract — the metres→inches and
// East/North→field conversions belong to the HAL edge and are pinned independently in
// gps_conversion_test.cpp (chunk tension T4). Nothing here re-implements them.

#include "doctest.h"

#include <cmath>
#include <limits>
#include <span>
#include <vector>

#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

using shulib::diag::GateReason;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeGps;
using shulib::hal::fake::FakeImu;
using shulib::localization::CorrectionProposal;
using shulib::localization::GpsCorrector;
using shulib::localization::GpsCorrectorConfig;
using shulib::math::Angle;
using shulib::math::Pose2d;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;

namespace {

/// Clock + fakes + corrector, with the tick bookkeeping every case repeats.
struct Rig {
    FakeClock clock;
    FakeGps gps;
    FakeImu imu;
    GpsCorrector corrector;

    explicit Rig(const GpsCorrectorConfig& cfg = {}) : corrector{clock, gps, imu, cfg} {
        imu.setReady(true);
        imu.setYawRate(AngularVelocity{0.0});
    }

    /// Advance time, then ask for a proposal at `predicted`. The order matches the
    /// Localizer's: the world moves, then the estimator asks.
    CorrectionProposal tick(double px, double py, double dt = 0.01, double headingRad = 0.0) {
        clock.advance(Time{dt});
        return corrector.propose(
            Pose2d{Length{px}, Length{py}, Angle::radians(headingRad)}, Time{dt});
    }

    void setFix(double x, double y, double rms = 1.0) {
        gps.setPose(Pose2d{Length{x}, Length{y}, Angle::radians(0.0)});
        gps.setRmsError(Length{rms});
        gps.setHasFix(true);
    }
};

}  // namespace

// ─────────────────────────────────────────────────────────────────────────────────────
// Adaptive R
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: positionStdDev echoing the device's raw claim instead of the trusted,
// floored, dead-reckon-widened sigma the gate and the fuser actually use — or the
// rmsTrustFactor being dropped, which would make R optimistic exactly when HA-29 says
// the device's self-estimate is least trustworthy.
//
// Hand-computed, start to finish. Config: rmsTrustFactor 2, minPositionStdDev 0.5,
// postFixStdDev 1.0, driftStdDevPerInch 0 (so sigma_dr is exactly 1.0 and the arithmetic
// stays checkable on paper). The robot has not moved, so travel is 0.
//   rms 1.5  → sigma_meas = max(2·1.5, 0.5) = 3.0
//              sigma_eff  = hypot(3.0, 1.0) = sqrt(10) = 3.1622776601683795
//              confidence = 1²/(1² + 3²)   = 0.1
TEST_CASE("GpsCorrector: adaptive R — sigma and confidence, computed by hand") {
    GpsCorrectorConfig cfg;
    cfg.rmsTrustFactor = 2.0;
    cfg.minPositionStdDev = Length{0.5};
    cfg.postFixStdDev = Length{1.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.latency = Time{0.0};
    Rig rig{cfg};

    rig.setFix(0.0, 0.0, 1.5);
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    REQUIRE(p.valid);
    CHECK(p.positionStdDev.value() == doctest::Approx(3.1622776601683795).epsilon(1e-12));
    CHECK(p.confidence == doctest::Approx(0.1).epsilon(1e-12));
}

// Would catch: the sigma FLOOR missing. A device claiming ~0 error would otherwise
// produce sigma_meas ~0, a gate a few thousandths of an inch wide, and a corrector that
// rejects every fix it ever sees including truthful ones — GPS silently dead.
//   rms 0.001 → sigma_meas = max(0.002, 0.5) = 0.5 (the floor)
//               sigma_eff  = hypot(0.5, 1.0) = 1.118033988749895
TEST_CASE("GpsCorrector: adaptive R — the sigma floor holds against an absurdly good claim") {
    GpsCorrectorConfig cfg;
    cfg.rmsTrustFactor = 2.0;
    cfg.minPositionStdDev = Length{0.5};
    cfg.postFixStdDev = Length{1.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.latency = Time{0.0};
    Rig rig{cfg};

    rig.setFix(0.0, 0.0, 0.001);
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    REQUIRE(p.valid);
    CHECK(p.positionStdDev.value() == doctest::Approx(1.118033988749895).epsilon(1e-12));
}

// Would catch: R not tracking the device's claim at all (a constant sigma). A worse
// claim must produce a wider sigma and a WEAKER pull — that is the whole content of the
// word "adaptive".
TEST_CASE("GpsCorrector: adaptive R — a worse claim widens sigma and weakens the pull") {
    GpsCorrectorConfig cfg;
    cfg.driftStdDevPerInch = 0.0;
    cfg.latency = Time{0.0};
    Rig good{cfg};
    Rig bad{cfg};

    good.setFix(0.0, 0.0, 0.5);
    bad.setFix(0.0, 0.0, 2.5);
    const CorrectionProposal pg = good.tick(0.0, 0.0);
    const CorrectionProposal pb = bad.tick(0.0, 0.0);
    REQUIRE(pg.valid);
    REQUIRE(pb.valid);
    CHECK(pb.positionStdDev.value() > pg.positionStdDev.value());
    CHECK(pb.confidence < pg.confidence);
    CHECK(pb.confidence > 0.0);  // still a proposal, just a weak one
}

// ─────────────────────────────────────────────────────────────────────────────────────
// No fix — the Driving-Skills path
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: THE BUG THAT LOSES SKILLS MATCHES. A corrector that returns a
// low-confidence proposal when the strip is absent instead of nothing at all. The model
// serves the ORIGIN while it has no fix (HA-31, deliberately adversarial), so a
// zero-confidence pull is a pull toward (0,0) — and the Localizer would report the tick
// as "Corrected" rather than dead-reckoning.
TEST_CASE("GpsCorrector: no fix yields NO proposal — never a low-confidence pull") {
    Rig rig;
    rig.gps.setHasFix(false);
    rig.gps.setPose(Pose2d{});             // the origin, as HA-31's model serves it
    rig.gps.setRmsError(Length{99.0});     // and the no-fix error report

    for (int i = 0; i < 5; ++i) {
        const CorrectionProposal p = rig.tick(24.0, 36.0);
        CHECK_FALSE(p.valid);
        CHECK(p.confidence == 0.0);
        CHECK(p.positionStdDev.value() == 0.0);
        CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);
    }
    CHECK(rig.corrector.noFixTicks() == 5);
    CHECK(rig.corrector.acceptedFixes() == 0);
    CHECK(rig.corrector.lastVerdict() == GateReason::RejectedNoFix);
}

// Would catch: FakeGps's own default being anything other than "no fix". The safe
// default is load-bearing — a corrector wired to a GPS nobody configured must
// dead-reckon, not invent an anchor at the origin.
TEST_CASE("GpsCorrector: an unconfigured GPS produces no proposal (safe default off-strip)") {
    Rig rig;  // FakeGps default: hasFix == false
    const CorrectionProposal p = rig.tick(10.0, 10.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);
}

// Would catch: a sentinel or NaN pose reaching the estimate. The R1 adapter is supposed
// to screen PROS_ERR_F (== +INF) to hasFix()==false (HA-08); if the screen ever fails,
// this must be a no-fix, not an infinite innovation. propose() must also NOT THROW here
// — ICorrector's contract is non-throwing, and a throw inside the control loop is worse
// than a bad fix.
TEST_CASE("GpsCorrector: a non-finite read is a no-fix, not a throw and not a pose") {
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    Rig a;
    a.gps.setHasFix(true);
    a.gps.setPose(Pose2d{Length{inf}, Length{0.0}, Angle::radians(0.0)});
    a.gps.setRmsError(Length{1.0});
    CorrectionProposal p;
    CHECK_NOTHROW(p = a.tick(0.0, 0.0));
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);

    Rig b;
    b.gps.setHasFix(true);
    b.gps.setPose(Pose2d{Length{0.0}, Length{0.0}, Angle::radians(0.0)});
    b.gps.setRmsError(Length{nan});
    CHECK_NOTHROW(p = b.tick(0.0, 0.0));
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNoFix);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Staleness — the double-count guard
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: folding one measurement once per LOOP TICK instead of once per FIX. The
// V5 camera produces a fix every ~50 ms (HA-28) against a ~100 Hz loop, so a naive
// corrector treats a single observation as five independent ones — and, worse, keeps
// re-applying a measurement that describes an ever-older moment while the robot moves.
TEST_CASE("GpsCorrector: a re-reported sample is folded once, then declined as stale") {
    Rig rig;
    rig.setFix(5.0, 0.0, 1.0);

    const CorrectionProposal first = rig.tick(0.0, 0.0);
    CHECK(first.valid);

    for (int i = 0; i < 4; ++i) {  // the same sample, four more loop ticks
        const CorrectionProposal again = rig.tick(0.0, 0.0);
        CHECK_FALSE(again.valid);
        CHECK(again.selfAudit.reason == GateReason::RejectedStaleFix);
    }
    CHECK(rig.corrector.acceptedFixes() == 1);
    CHECK(rig.corrector.staleTicks() == 4);

    rig.setFix(5.01, 0.0, 1.0);  // the camera produces a new sample
    CHECK(rig.tick(0.0, 0.0).valid);
    CHECK(rig.corrector.acceptedFixes() == 2);
}

// Would catch: freshness keyed on position alone. A device that reports the same
// position with a changed error estimate has told us something new about how much to
// trust it, and a device whose position changes by a hair has genuinely re-measured.
TEST_CASE("GpsCorrector: freshness watches the reported error too, not just position") {
    Rig rig;
    rig.setFix(1.0, 2.0, 1.0);
    CHECK(rig.tick(0.0, 0.0).valid);
    CHECK_FALSE(rig.tick(0.0, 0.0).valid);   // unchanged → stale
    rig.setFix(1.0, 2.0, 1.25);              // same place, new confidence in it
    CHECK(rig.tick(0.0, 0.0).valid);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Sensor quality
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a source that claims a fix while reporting 99 inches of error being
// folded anyway. It would barely move the estimate (confidence ~0.0001), which is
// exactly why it is dangerous: the Localizer would still see `applied`, report quality
// class "Corrected", and a run with no usable anchor would look anchored.
TEST_CASE("GpsCorrector: a fix whose claimed error is huge is declined, visibly") {
    GpsCorrectorConfig cfg;
    cfg.maxReportedRms = Length{6.0};
    Rig rig{cfg};

    rig.setFix(0.0, 0.0, 99.0);  // "I have a fix", says the sensor, "give or take 8 feet"
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedSensorQuality);
    CHECK(rig.corrector.qualityRejects() == 1);

    rig.setFix(0.0, 0.0, 5.9);  // just inside the ceiling: still usable, weakly
    CHECK(rig.tick(0.0, 0.0).valid);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// High yaw rate
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: fixes folded mid-spin. The lever-arm removal at the HAL edge is at its
// most wrong during a fast rotation (the sensor is swinging through an arc), and no
// latency compensation built from position history can recover a rotation it did not
// see. A sign error in the lever arm shows up as a circle traced by stationary-spin
// fixes — this is the gate that keeps that out of the estimate.
TEST_CASE("GpsCorrector: fixes taken during a fast spin are declined") {
    GpsCorrectorConfig cfg;
    cfg.maxYawRate = AngularVelocity{3.0};
    Rig rig{cfg};

    rig.imu.setYawRate(AngularVelocity{5.0});  // ~286 deg/s
    rig.setFix(1.0, 0.0, 1.0);
    CorrectionProposal p = rig.tick(0.0, 0.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedHighYawRate);

    // The sign of the rotation must not matter.
    rig.imu.setYawRate(AngularVelocity{-5.0});
    rig.setFix(1.1, 0.0, 1.0);
    p = rig.tick(0.0, 0.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedHighYawRate);
    CHECK(rig.corrector.yawRateRejects() == 2);

    // Slow enough, and the same class of fix is fine.
    rig.imu.setYawRate(AngularVelocity{0.5});
    rig.setFix(1.2, 0.0, 1.0);
    CHECK(rig.tick(0.0, 0.0).valid);
}

// Would catch: a sample captured mid-spin being folded a tick later, once the spin has
// stopped. The rejection must CONSUME the sample — it describes a moment already judged
// untrustworthy, and "wait for the robot to stop and then use it" is worse than
// dropping it.
TEST_CASE("GpsCorrector: a fix rejected mid-spin is not folded after the spin ends") {
    Rig rig;
    rig.imu.setYawRate(AngularVelocity{5.0});
    rig.setFix(3.0, 0.0, 1.0);
    CHECK_FALSE(rig.tick(0.0, 0.0).valid);

    rig.imu.setYawRate(AngularVelocity{0.0});  // spin over, sample unchanged
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedStaleFix);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Latency
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a fix applied as if it described NOW. At 40 in/s with 50 ms of latency
// (HA-30) that is a systematic 2-inch lag along the direction of travel — larger than
// the sensor's own noise, and biased rather than random, so it never averages out.
//
// The setup is arranged so the answer is exact: the robot moves +1 inch per 10 ms tick
// (100 in/s) in a straight line along +X, and the GPS reports the TRUE position of five
// ticks ago (50 ms of latency). A corrector that ignores latency proposes 5 inches
// behind the prediction; one that compensates proposes the prediction itself, so the
// residual is 0.
TEST_CASE("GpsCorrector: latency compensation removes the lag of a moving robot") {
    GpsCorrectorConfig cfg;
    cfg.latency = Time{0.05};
    Rig rig{cfg};

    std::vector<double> path;
    for (int i = 0; i <= 20; ++i) {
        path.push_back(static_cast<double>(i));
    }

    CorrectionProposal last;
    for (std::size_t i = 0; i < path.size(); ++i) {
        // The fix that becomes readable at tick i describes the truth at tick i-5.
        if (i >= 5) {
            rig.setFix(path[i - 5], 0.0, 1.0);
        }
        last = rig.tick(path[i], 0.0, 0.01);
    }
    REQUIRE(last.valid);
    // The measurement said 15.0; the prediction is 20.0; the compensated proposal must
    // be back at 20.0, not 15.0.
    CHECK(last.fieldPose.x().value() == doctest::Approx(20.0).epsilon(1e-9));
}

// Would catch: latency compensation applied with the WRONG SIGN — pushing the fix
// further back instead of forward. That doubles the lag instead of removing it, and a
// test that only checks "the number changed" would pass.
TEST_CASE("GpsCorrector: latency compensation moves the fix FORWARD along travel") {
    GpsCorrectorConfig cfg;
    cfg.latency = Time{0.05};
    Rig rig{cfg};

    for (int i = 0; i < 10; ++i) {  // build history: moving +1 in per tick along +X
        rig.tick(static_cast<double>(i), 0.0, 0.01);
    }
    rig.setFix(5.0, 0.0, 1.0);  // a fix reading 5.0 while the prediction is at 10.0
    const CorrectionProposal p = rig.tick(10.0, 0.0, 0.01);
    REQUIRE(p.valid);
    CHECK(p.fieldPose.x().value() > 5.0);   // carried forward, not left where it was
    CHECK(p.fieldPose.x().value() < 11.0);  // and not overshot past the prediction
}

// Would catch: latency compensation inventing motion for a STATIONARY robot. With no
// travel there is nothing to compensate, and a corrector that added a velocity estimate
// anyway would bias every fix taken at rest — the condition most auton routines are in
// when they care most about position.
TEST_CASE("GpsCorrector: a stationary robot's fix is not moved by latency compensation") {
    GpsCorrectorConfig cfg;
    cfg.latency = Time{0.05};
    Rig rig{cfg};

    for (int i = 0; i < 10; ++i) {
        rig.tick(7.0, -3.0, 0.01);
    }
    rig.setFix(7.5, -3.25, 1.0);
    const CorrectionProposal p = rig.tick(7.0, -3.0, 0.01);
    REQUIRE(p.valid);
    CHECK(p.fieldPose.x().value() == doctest::Approx(7.5).epsilon(1e-12));
    CHECK(p.fieldPose.y().value() == doctest::Approx(-3.25).epsilon(1e-12));
}

// ─────────────────────────────────────────────────────────────────────────────────────
// The gate (tension T1)
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: the gate accepting a confident lie. A3's bad-fix window models exactly
// this — truth plus a constant offset, still claiming a normal rms — and it is the
// attack the innovation gate exists for. Hand-computed threshold: rms 1.0,
// rmsTrustFactor 2 → sigma_meas 2.0; travel 0 with driftStdDevPerInch 0 → sigma_dr 1.0;
// sigma_eff = sqrt(5) = 2.2360679…; gateSigma 4 → the boundary is 8.944271909999159.
TEST_CASE("GpsCorrector: the normalized-innovation gate rejects a confident lie") {
    GpsCorrectorConfig cfg;
    cfg.rmsTrustFactor = 2.0;
    cfg.postFixStdDev = Length{1.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.gateSigma = 4.0;
    cfg.latency = Time{0.0};
    cfg.maxReportedRms = Length{50.0};
    Rig rig{cfg};

    rig.setFix(8.9, 0.0, 1.0);  // just inside 8.944…
    CorrectionProposal p = rig.tick(0.0, 0.0);
    CHECK(p.valid);

    rig.setFix(9.0, 0.0, 1.0);  // just outside
    p = rig.tick(0.0, 0.0);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNormalizedInnovation);
    CHECK(rig.corrector.innovationRejects() == 1);
}

// Would catch: the gate recording nothing, so a rejection is unexplainable after the
// fact. The residual it was rendered on AND the sigma it was normalized by must both
// come out, because the verdict is `|residual| > gateSigma · sigma` and neither number
// alone lets anyone check it. THIS is what makes E1's "reconstructable from the file
// alone" true for a real gate rather than a synthetic one.
TEST_CASE("GpsCorrector: a gate rejection carries the numbers it was decided on") {
    GpsCorrectorConfig cfg;
    cfg.rmsTrustFactor = 2.0;
    cfg.postFixStdDev = Length{1.0};
    cfg.driftStdDevPerInch = 0.0;
    cfg.gateSigma = 4.0;
    cfg.latency = Time{0.0};
    Rig rig{cfg};

    rig.setFix(20.0, -15.0, 1.0);
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    REQUIRE_FALSE(p.valid);
    CHECK(p.selfAudit.residualX.value() == doctest::Approx(20.0).epsilon(1e-12));
    CHECK(p.selfAudit.residualY.value() == doctest::Approx(-15.0).epsilon(1e-12));
    CHECK(p.selfAudit.covarianceTrace == doctest::Approx(2.2360679774997896).epsilon(1e-12));
    // …and the ratio a reader recomputes from those two numbers is the verdict:
    const double nu = std::hypot(p.selfAudit.residualX.value(), p.selfAudit.residualY.value()) /
                      p.selfAudit.covarianceTrace;
    CHECK(nu > cfg.gateSigma);
    // The Mahalanobis slot stays EMPTY. This tier has no filter-estimated covariance,
    // and a plausible-looking number there would be indistinguishable from E4's real one.
    CHECK(p.selfAudit.mahalanobis == 0.0);
    CHECK(p.selfAudit.reason != GateReason::RejectedMahalanobis);
}

// Would catch: GATE LOCKOUT — the failure where a corrector goes permanently silent
// after the estimate drifts. Without the dead-reckoning widening, a fix that disagrees
// by 30 inches is rejected no matter how long the robot has been navigating blind, and
// since nothing else can fix the estimate, every subsequent fix is rejected too. The GPS
// dies exactly when it is worth the most.
//
// The two rigs differ ONLY in driftStdDevPerInch.
TEST_CASE("GpsCorrector: the gate widens with dead-reckoned travel (anti-lockout)") {
    GpsCorrectorConfig fixed;
    fixed.driftStdDevPerInch = 0.0;
    fixed.latency = Time{0.0};
    GpsCorrectorConfig widening = fixed;
    widening.driftStdDevPerInch = 0.05;

    Rig rigFixed{fixed};
    Rig rigWide{widening};

    // Both robots drive 200 inches with no GPS at all, so travel accumulates.
    rigFixed.gps.setHasFix(false);
    rigWide.gps.setHasFix(false);
    for (int i = 1; i <= 200; ++i) {
        const double x = static_cast<double>(i);
        rigFixed.tick(x, 0.0, 0.01);
        rigWide.tick(x, 0.0, 0.01);
    }
    CHECK(rigWide.corrector.travelSinceFix().value() == doctest::Approx(199.0).epsilon(1e-9));

    // Now the strip appears, and says the estimate is 30 inches off — which, after 200
    // inches of dead reckoning, is entirely believable.
    rigFixed.setFix(170.0, 0.0, 1.0);
    rigWide.setFix(170.0, 0.0, 1.0);
    const CorrectionProposal pf = rigFixed.tick(200.0, 0.0, 0.01);
    const CorrectionProposal pw = rigWide.tick(200.0, 0.0, 0.01);

    CHECK_FALSE(pf.valid);  // locked out
    CHECK(pf.selfAudit.reason == GateReason::RejectedNormalizedInnovation);
    CHECK(pw.valid);        // recovered
    // …and the recovery is not just "accept everything": the widened sigma also raises
    // the confidence, because a long dead-reckon means the estimate is the uncertain one.
    CHECK(pw.confidence > 0.5);
}

// Would catch: the widening never RESETTING, which would leave the gate permanently
// wide open after one long blind stretch — the opposite failure, where a lie is accepted
// because the corrector still thinks it is lost.
TEST_CASE("GpsCorrector: accepting a fix resets the dead-reckon widening") {
    GpsCorrectorConfig cfg;
    cfg.driftStdDevPerInch = 0.05;
    cfg.latency = Time{0.0};
    Rig rig{cfg};

    rig.gps.setHasFix(false);
    for (int i = 1; i <= 100; ++i) {
        rig.tick(static_cast<double>(i), 0.0, 0.01);
    }
    CHECK(rig.corrector.travelSinceFix().value() > 90.0);

    rig.setFix(100.0, 0.0, 1.0);
    REQUIRE(rig.tick(100.0, 0.0, 0.01).valid);
    CHECK(rig.corrector.travelSinceFix().value() == 0.0);

    // The very next lie, told from a standstill, is rejected again.
    rig.setFix(140.0, 0.0, 1.0);
    const CorrectionProposal p = rig.tick(100.0, 0.0, 0.01);
    CHECK_FALSE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::RejectedNormalizedInnovation);
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Heading (tension T3)
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: the GPS's heading escaping the corrector. GPS heading noise is ~1°
// (HA-27) against an IMU an order of magnitude better, heading is IMU-owned by decision
// #4, and providesHeading is RESERVED at M2. The proposal must carry the PREDICTED
// heading, so that even a future policy that read fieldPose.heading() would read the
// IMU's answer rather than the camera's.
TEST_CASE("GpsCorrector: the proposal carries the IMU heading, never the GPS's") {
    Rig rig;
    // A GPS that is confidently, wildly wrong about which way the robot faces.
    rig.gps.setPose(Pose2d{Length{1.0}, Length{2.0}, Angle::degrees(137.0)});
    rig.gps.setRmsError(Length{1.0});
    rig.gps.setHasFix(true);

    const double predictedHeading = Angle::degrees(-42.0).radians();
    const CorrectionProposal p = rig.tick(0.0, 0.0, 0.01, predictedHeading);
    REQUIRE(p.valid);
    CHECK(p.fieldPose.heading().approxEqual(Angle::degrees(-42.0)));
    CHECK_FALSE(p.fieldPose.heading().approxEqual(Angle::degrees(137.0)));
    CHECK_FALSE(p.providesHeading);  // RESERVED, and E2 does not start using it
}

// ─────────────────────────────────────────────────────────────────────────────────────
// Never-snap, structurally
// ─────────────────────────────────────────────────────────────────────────────────────

// Would catch: a corrector that tries to SET the pose. ICorrector has no channel to do
// so — it returns a value — but the property worth pinning is that even a maximally
// disagreeing accepted fix moves the fused estimate by no more than the per-tick budget.
// This runs the corrector through the real ComplementaryFusion rather than asserting
// about the proposal in isolation.
TEST_CASE("GpsCorrector: even a huge accepted innovation is a bounded nudge, never a snap") {
    GpsCorrectorConfig cfg;
    cfg.driftStdDevPerInch = 1.0;  // maximally lost, so the gate accepts a wild fix
    cfg.latency = Time{0.0};
    Rig rig{cfg};

    shulib::localization::ComplementaryFusion fusion{
        {.maxNudgeRate = shulib::units::Velocity{12.0},
         .innovationGate = Length{200.0},
         .maxGain = 0.15}};

    rig.gps.setHasFix(false);
    for (int i = 1; i <= 100; ++i) {  // accumulate travel so the gate is wide
        rig.tick(static_cast<double>(i), 0.0, 0.01);
    }
    rig.setFix(0.0, 0.0, 1.0);  // "you are 100 inches from where you think you are"
    const Pose2d predicted{Length{100.0}, Length{0.0}, Angle::radians(0.0)};
    const CorrectionProposal p = rig.tick(100.0, 0.0, 0.01);
    REQUIRE(p.valid);

    const CorrectionProposal one[] = {p};
    const auto fr = fusion.fuse(predicted, std::span<const CorrectionProposal>{one}, Time{0.01});
    const double moved = std::hypot(fr.x.value() - 100.0, fr.y.value() - 0.0);
    CHECK(moved <= 12.0 * 0.01 + 1e-12);  // the per-tick budget, and not one inch more
    CHECK(fr.clamped);                    // and it says so
    CHECK(fr.applied);
}

// Would catch: the corrector's own `selfAudit` claiming Accepted on a tick where the
// proposal was screened out and nothing was applied. The Localizer substitutes a
// corrector's self-audit whenever the policy has no verdict, so a corrector that filled
// in "Accepted" for its own proposals could manufacture a false Accepted in the record.
TEST_CASE("GpsCorrector: an accepted proposal leaves selfAudit EMPTY for the policy to fill") {
    Rig rig;
    rig.setFix(1.0, 1.0, 1.0);
    const CorrectionProposal p = rig.tick(0.0, 0.0);
    REQUIRE(p.valid);
    CHECK(p.selfAudit.reason == GateReason::None);
}

// Would catch: the corrector's name not reaching telemetry, which is what per-source
// dead-reckon accounting is keyed on when more than one corrector exists (E3).
TEST_CASE("GpsCorrector: name() is the stable telemetry id") {
    FakeClock clock;
    FakeGps gps;
    FakeImu imu;
    GpsCorrector defaultName{clock, gps, imu};
    GpsCorrector custom{clock, gps, imu, {}, "gps-rear"};
    CHECK(std::string_view{defaultName.name()} == "gps");
    CHECK(std::string_view{custom.name()} == "gps-rear");
}
