// GpsCorrector accuracy evidence (chunk E2, tension T2) — does the first real corrector
// actually help, and in what precise sense?
//
// ── THE METRIC, STATED BEFORE THE TEST ─────────────────────────────────────────────
// The E2 brief's DoD says the corrector "reduces pose error versus dead-reckoning alone
// and never increases it". Read tick-by-tick that is unachievable: any corrector fed a
// noisy fix will occasionally make one tick worse, and a test asserting otherwise would
// be either vacuous or unfairly specified. So the claim is defined here, and only these
// four things are claimed:
//
//   1. RECOVERY — the property that actually separates a corrected estimator from a
//      dead-reckoning one. Give both a known 6-inch position error and drive: the
//      corrected estimate converges back toward truth, the dead-reckoned one carries the
//      error to the end of the run. Odometry cannot remove an error it has absorbed;
//      that is the whole reason correctors exist, and it does not depend on any invented
//      magnitude.
//   2. AGGREGATE ERROR — over 8 seeds of a 60-second skills-length run, mean final and
//      mean worst-case position error are lower with the corrector than without.
//   3. BUDGET HONESTY — no tick moves the fused position further from the odometry-only
//      prediction than maxNudgeRate * dt. Never-snap, asserted every tick of every seed.
//   4. DEGRADATION — off-strip, the corrector contributes exactly nothing (bit-identical
//      to no corrector) and a confident lie mid-run does bounded damage.
//
// ── WHAT IS *NOT* CLAIMED, AND WHY — READ THIS BEFORE QUOTING ANY NUMBER ───────────
//  * NOT that every tick improves. It does not, and a fair test cannot ask for that.
//  * NOT that the corrector wins on EVERY seed. It does not. Measured over 8 seeds of the
//    60 s run below: final error is better on 7 of 8, worst-case on 6 of 8. On seed 3 the
//    corrected FINAL error is worse than dead-reckoning's, though its worst case is less
//    than half. The reason is worth understanding rather than tuning away: in this
//    simulation dead-reckoning is already sub-two-inch over 1246 inches of driving,
//    because A3's slip model degrades the DRIVEN wheels while the unpowered tracking
//    wheels read true body travel — so the only error sources left are IMU heading drift
//    (HA-20) and encoder quantization. The modeled GPS noise (0.7"/axis, HA-26) is the
//    SAME ORDER as the drift it is correcting, so folding it injects roughly as much
//    noise as it removes.
//  * Which means: WHETHER A GPS CORRECTOR IS WORTH FOLDING AT ALL depends on the ratio of
//    sensor noise to dead-reckoning drift, and BOTH numbers are invented (HA-26, HA-20).
//    R4 measures them. Nothing in this file is evidence about a robot.
//  * NOT any absolute accuracy number. E2 proves LOGIC; R4 measures constants.
//  * NOTHING about heading. Heading is IMU-owned and untouched by this chunk, so the
//    F2 < 1 deg budget is exactly where A3 left it.
//  * NOT recovery from a GROSSLY wrong estimate. `ComplementaryFusion` applies a FIXED
//    12-inch innovation gate after this corrector's own, so an estimate more than 12
//    inches from truth is rejected one layer down no matter how wide E2's gate opens.
//    Observed during this chunk (see E2-PROGRESS): an estimate 29 inches out never
//    recovered with a perfectly good GPS in view. That constant belongs to the fusion
//    policy, which E4 replaces.
//
// ── THE A/B IS EXACT ────────────────────────────────────────────────────────────────
// Both estimators run on ONE plant, reading the SAME sensor stream, tick for tick: two
// PilonsOdometry instances over the same tracking wheels, two Localizers, one with the
// corrector wired and one without. There is no run-to-run variance to argue about — the
// only difference between the two numbers is the corrector.

#include "doctest.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#include <vector>

#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/gps_corrector.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::kinematics::xDrive;
using shulib::localization::ComplementaryFusion;
using shulib::localization::ComplementaryFusionConfig;
using shulib::localization::GpsCorrector;
using shulib::localization::GpsCorrectorConfig;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::sim::FullHostility;
using shulib::sim::FullHostilityConfig;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kDt = 0.01;
/// The IMU calibration window is 2 s by default and the Localizer holds the fold for a
/// further settle period; real autons wait it out, so the robot holds still until then
/// and measurement starts after. Comparing estimators across a window where NEITHER has
/// a live estimate would compare nothing.
constexpr int kSettleTicks = 300;
/// 60 s — a skills-run length, which is the horizon F2's accuracy spec is written about.
constexpr int kDriveTicks = 6000;

[[nodiscard]] SimHarnessConfig plantConfig(std::uint64_t seed) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    cfg.plant.seed = seed;
    return cfg;
}

/// A mostly-outbound skills-shaped path: keep driving, weave, and spin hard once per
/// ten seconds. Outbound rather than a closed loop on purpose — on a loop that returns
/// to its start, heading-drift errors partly cancel and dead-reckoning looks better than
/// it is. The hard spin (4 rad/s, above the corrector's default threshold) is what
/// exercises the yaw-rate rejection on a real trajectory rather than in isolation.
[[nodiscard]] ChassisSpeeds scriptedTwist(int tick) {
    switch ((tick / 100) % 10) {
        case 3: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{-4.0}};  // hard spin
        case 6: return {Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.35}};
        case 8: return {Velocity{20.0}, Velocity{0.0}, AngularVelocity{-0.35}};
        default: return {Velocity{24.0}, Velocity{0.0}, AngularVelocity{0.0}};
    }
}

[[nodiscard]] double posErr(const Pose2d& a, const Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// Per-seed outcome of one A/B run.
struct Outcome {
    double finalGps = 0.0;
    double finalDr = 0.0;
    double worstGps = 0.0;
    double worstDr = 0.0;
    double firstHalfWorstGps = 0.0;
    double secondHalfWorstGps = 0.0;
    double firstHalfWorstDr = 0.0;
    double secondHalfWorstDr = 0.0;
    double worstNudge = 0.0;
    std::uint32_t accepted = 0;
    std::uint32_t noFix = 0;
    std::uint32_t yawRejects = 0;
    double pathLength = 0.0;
};

/// One seeded run of the exact A/B described in the file header. `perturbIn`, when
/// non-zero, displaces BOTH estimators by that many inches in +x at the moment the fold
/// opens — a known, identical wound, so what is measured afterwards is which estimator
/// can heal it.
[[nodiscard]] Outcome runSeed(std::uint64_t seed, const FullHostilityConfig& hostileCfg,
                              const GpsCorrectorConfig& gpsCfg = {}, double perturbIn = 0.0,
                              int driveTicks = kDriveTicks) {
    const auto kin = xDrive(Length{7.0});
    FullHostility hostile{hostileCfg};
    SimHarness h{kin, plantConfig(seed), nullptr, &hostile.model()};

    PilonsOdometry odomGps{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    PilonsOdometry odomDr{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    const ComplementaryFusionConfig fusionCfg{};
    ComplementaryFusion fusionGps{fusionCfg};
    ComplementaryFusion fusionDr{fusionCfg};
    GpsCorrector corrector{h.clock(), h.gps(), h.imu(), gpsCfg};
    std::array<ICorrector*, 1> correctors{&corrector};
    Localizer locGps{h.clock(), h.imu(), odomGps, fusionGps,
                     std::span<ICorrector* const>{correctors}};
    Localizer locDr{h.clock(), h.imu(), odomDr, fusionDr};

    Outcome out;
    Pose2d lastTruth = h.truePose();
    const double budget = fusionCfg.maxNudgeRate.value() * kDt;

    h.runTicks(kSettleTicks + driveTicks, Time{kDt}, [&](int tick) {
        locGps.update();
        locDr.update();

        if (tick == kSettleTicks && perturbIn != 0.0) {
            // The identical wound, applied to both. setPose moves POSITION only (heading
            // stays IMU-owned) and re-baselines the twist, so neither estimator gets a
            // phantom velocity out of the jump.
            const auto wound = [&](Localizer& l) {
                const Pose2d p = l.pose();
                l.setPose(Pose2d{Length{p.x().value() + perturbIn}, p.y(), p.heading()});
            };
            wound(locGps);
            wound(locDr);
        }

        if (tick >= kSettleTicks) {
            const Pose2d truth = h.truePose();
            const double eGps = posErr(locGps.pose(), truth);
            const double eDr = posErr(locDr.pose(), truth);
            out.finalGps = eGps;
            out.finalDr = eDr;
            out.worstGps = std::max(out.worstGps, eGps);
            out.worstDr = std::max(out.worstDr, eDr);
            const bool secondHalf = tick >= kSettleTicks + driveTicks / 2;
            if (secondHalf) {
                out.secondHalfWorstGps = std::max(out.secondHalfWorstGps, eGps);
                out.secondHalfWorstDr = std::max(out.secondHalfWorstDr, eDr);
            } else {
                out.firstHalfWorstGps = std::max(out.firstHalfWorstGps, eGps);
                out.firstHalfWorstDr = std::max(out.firstHalfWorstDr, eDr);
            }
            out.pathLength += posErr(truth, lastTruth);
            lastTruth = truth;

            // CLAIM 4, asserted here rather than collected: the applied nudge is the
            // fused position minus the odometry-only prediction, and the never-snap
            // invariant says it can never exceed the per-tick budget.
            const double nudge = std::hypot(locGps.lastCorrection().dx.value(),
                                            locGps.lastCorrection().dy.value());
            out.worstNudge = std::max(out.worstNudge, nudge);
            REQUIRE(nudge <= budget + 1e-9);
        }
        // Hold still through IMU calibration. This is not test convenience: the
        // Localizer's boot guard folds NOTHING before the IMU has ever been ready, so
        // motion commanded during calibration is unaccounted by construction
        // (localizer.hpp states it as the consumer contract, and real autons wait it
        // out). Driving through it leaves the estimate tens of inches wrong before the
        // corrector has ever been consulted, which measures the boot guard, not the
        // corrector.
        h.commandBodyTwist(tick < kSettleTicks ? ChassisSpeeds{}
                                               : scriptedTwist(tick - kSettleTicks));
    });

    out.accepted = corrector.acceptedFixes();
    out.noFix = corrector.noFixTicks();
    out.yawRejects = corrector.yawRateRejects();
    return out;
}

}  // namespace

// CLAIM 1 — the one that does not depend on a single invented magnitude.
//
// Would catch: a corrector that smooths but does not ANCHOR. Odometry has no mechanism
// to notice, let alone remove, an error it absorbed a thousand ticks ago — that sentence
// is guide chapter 3's, and it is the reason correctors exist. Give both estimators the
// same 6-inch wound and drive: if the corrected one does not heal while the
// dead-reckoned one carries the wound to the end, the corrector is not doing the one job
// no amount of better odometry can do.
//
// 6 inches is chosen to sit inside BOTH gates (the corrector's ~8.9 sigma-derived bound
// and ComplementaryFusion's fixed 12") so the test measures convergence, not gating.
TEST_CASE("[accuracy] a known position error is HEALED, which dead-reckoning can never do") {
    FullHostilityConfig cfg;
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        CAPTURE(seed);
        const Outcome o = runSeed(seed, cfg, {}, /*perturbIn=*/6.0, /*driveTicks=*/1500);
        REQUIRE(o.accepted > 100);
        // The wound was real for both — each estimator started 6 inches out.
        CHECK(o.worstGps > 4.0);
        CHECK(o.worstDr > 4.0);
        // Fifteen seconds later: one healed, the other did not.
        CHECK(o.finalGps < 1.5);
        CHECK(o.finalDr > 5.0);
    }
}

// CLAIM 2 — the aggregate accuracy claim, scoped exactly as the file header scopes it.
//
// Would catch: a corrector that is wired up, busy and useless — or worse, one that makes
// the estimate less accurate on average than not having it at all. Asserted on the MEAN
// across seeds rather than per seed, because per-seed superiority is NOT what the
// measurements support (see the header: 7 of 8 on final, 6 of 8 on worst-case) and a
// per-seed assertion here would have to be bought by shopping for a friendlier scenario.
TEST_CASE("[accuracy] over 8 seeds of a 60 s run, mean error is lower with the corrector") {
    FullHostilityConfig cfg;  // full hostility: GPS noise + decimation, IMU drift, slip,
                              // encoder quantization, latency, battery sag
    double sumFinalGps = 0.0;
    double sumFinalDr = 0.0;
    double sumWorstGps = 0.0;
    double sumWorstDr = 0.0;
    int finalWins = 0;
    int worstWins = 0;
    constexpr int kSeeds = 8;
    for (std::uint64_t seed = 1; seed <= kSeeds; ++seed) {
        CAPTURE(seed);
        const Outcome o = runSeed(seed, cfg);
        // The run has to have been a real one, or every comparison below is vacuous.
        REQUIRE(o.pathLength > 800.0);
        REQUIRE(o.accepted > 500);
        sumFinalGps += o.finalGps;
        sumFinalDr += o.finalDr;
        sumWorstGps += o.worstGps;
        sumWorstDr += o.worstDr;
        finalWins += (o.finalGps <= o.finalDr) ? 1 : 0;
        worstWins += (o.worstGps <= o.worstDr) ? 1 : 0;
    }
    MESSAGE("60 s, 8 seeds — mean final: corrected ", sumFinalGps / kSeeds, " vs dead-reckoned ",
            sumFinalDr / kSeeds, "; mean worst: ", sumWorstGps / kSeeds, " vs ",
            sumWorstDr / kSeeds, "; per-seed wins ", finalWins, "/8 final, ", worstWins,
            "/8 worst");
    CHECK(sumFinalGps < sumFinalDr);
    CHECK(sumWorstGps < sumWorstDr);
    // …and the win is not one seed carrying seven losses.
    CHECK(finalWins >= 6);
    CHECK(worstWins >= 5);
}

// Would catch: a "correction" that snaps. §13 #4 is not negotiable — a hard pose reset
// mid-routine teleports the estimate the motion controller is steering off, and the
// robot visibly twitches. The bound is asserted on EVERY tick of every seed inside
// runSeed(); this case exists to state it as its own claim and to prove the assertion is
// not vacuous (a nudge of exactly zero all run would satisfy any upper bound).
TEST_CASE("[accuracy] never-snap: every applied nudge is inside the per-tick budget") {
    FullHostilityConfig cfg;
    const Outcome o = runSeed(3, cfg);
    const double budget = ComplementaryFusionConfig{}.maxNudgeRate.value() * kDt;
    CHECK(o.worstNudge <= budget + 1e-9);
    CHECK(o.worstNudge > 0.0);  // corrections really were applied
}

// Would catch: THE SKILLS FAILURE, end to end and under full hostility. With the strip
// absent for the entire run (Driving Skills), the corrector must contribute NOTHING —
// not a weak pull toward the stale origin the model serves — and must say so. The
// estimator degrades to dead-reckoning rather than drifting toward a phantom fix.
TEST_CASE("[accuracy] off-strip for a whole run: dead-reckon only, and no phantom pull") {
    FullHostilityConfig cfg;
    cfg.gps.offStrip = true;  // Driving Skills: there is no strip
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        CAPTURE(seed);
        const Outcome o = runSeed(seed, cfg);
        CHECK(o.accepted == 0);
        CHECK(o.noFix > 3000);
        CHECK(o.worstNudge == 0.0);        // nothing was ever applied
        CHECK(o.finalGps == o.finalDr);    // …so the two estimates are identical
        CHECK(o.worstGps == o.worstDr);
    }
}

// Would catch: a confident lie surviving the gate and corrupting the estimate. A3's
// bad-fix window reports truth-plus-a-constant-offset while still claiming a normal rms
// — a strip misread or reflection — and it is the attack the innovation gate exists for.
// Damage must be BOUNDED by the gate, not merely "usually fine".
TEST_CASE("[accuracy] a confident lie mid-run does not corrupt the estimate") {
    FullHostilityConfig clean;
    FullHostilityConfig lying;
    lying.gps.badFixWindows = {{.start = Time{13.0},
                                .end = Time{18.0},
                                .dx = Length{40.0},
                                .dy = Length{-30.0}}};
    for (std::uint64_t seed = 1; seed <= 4; ++seed) {
        CAPTURE(seed);
        const Outcome honest = runSeed(seed, clean);
        const Outcome lied = runSeed(seed, lying);
        // The lie is 50 inches off. If it were accepted, the estimate would be dragged
        // tens of inches; the gate must keep the damage to a small multiple of the
        // honest run's worst error.
        CHECK(lied.worstGps < honest.worstGps + 6.0);
        CHECK(lied.finalGps < honest.finalGps + 3.0);
        // …and the damage stays inside the gate's own scale rather than the lie's: a
        // 50-inch lie must not move the estimate by anything like 50 inches.
        CHECK(lied.worstGps < 10.0);
    }
}

// Would catch: the yaw-rate rejection being dead code in practice. The scripted path
// includes a 4 rad/s spin every ten seconds, which is above the default threshold, so a
// live rejection path must fire — otherwise the gate is untested where it matters.
TEST_CASE("[accuracy] the high-yaw-rate rejection actually fires on a real trajectory") {
    FullHostilityConfig cfg;
    const Outcome o = runSeed(1, cfg);
    CHECK(o.yawRejects > 0);
    CHECK(o.accepted > o.yawRejects);  // …but it is not rejecting everything
}
