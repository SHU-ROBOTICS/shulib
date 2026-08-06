// C1 SWEEP + UNIVERSAL INVARIANTS — seeded random start poses × targets ×
// headings across the field, both drivetrains. Hand-picked points prove
// hand-picked things; the sweep asserts the properties that must hold over the
// WHOLE input space, per tick:
//
//   * TERMINATION   — no trial ends Running (the watchdog is universal)
//   * VOLTAGE BUDGET— |commanded volts| ≤ 12 AND ≤ battery (compensation live)
//   * WHEEL BUDGET  — the FF-implied wheel speed never exceeds maxWheelSpeed
//                     (desaturate() is actually in the path — its removal reds)
//   * AUTHORITY     — body-frame |vy| of the achievable command ≤
//                     strafeAuthority()·maxLinearSpeed (audited from records)
//   * WRAP          — the estimate's heading stays in (-π, π]
//   * PHYSICALITY   — per-tick TRUE translation ≤ maxLinearSpeed·dt·(1+slack)
//   * FINITENESS    — no NaN/Inf in pose or volts, ever
//   * PROGRESS      — aggregate monotone: distance-to-target never exceeds the
//                     initial distance + a small transient allowance, and ends
//                     inside tolerance
//
// Every trial is seeded and reproducible; failures print their seed.

#include "doctest.h"

#include <algorithm>
#include <cmath>

#include "motion_test_rig.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/kinematics/matrix_kinematics.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::diag::isFinitePose;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::motion::MoveToPose;
using shulib::sim::Rng;
using shulib::units::Time;

namespace {

Pose2d randomPose(Rng& rng, double range) {
    return Pose2d{Length{rng.uniform(-range, range)}, Length{rng.uniform(-range, range)},
                  Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
}

struct TrialStats {
    double truthErr = 0.0;
    double headingErr = 0.0;
};

/// One swept trial with EVERY universal invariant asserted per tick.
TrialStats runInvariantTrial(const shulib::kinematics::IKinematics& kin, const Pose2d& start,
                             const Pose2d& target) {
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = start;
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, pcfg, &records};
    const auto mcfg = motionConfig();
    MoveToPose m{rig.deps, target, mcfg, 8.0};
    m.start();

    const double initialDist = posErr(start, target);
    const double maxLin = mcfg.maxLinearSpeed.value();
    const double maxWheel = mcfg.maxWheelSpeed.value();
    const double kS = mcfg.wheelFf.kS;
    const double kV = mcfg.wheelFf.kV;
    const double dt = 0.01;

    auto reason = ExitReason::Running;
    Pose2d prevTruth = rig.h.truePose();
    for (int i = 0; i < 1200 && reason == ExitReason::Running; ++i) {
        rig.loc.update();

        // WRAP + FINITENESS (estimate side)
        const double h = rig.loc.pose().heading().radians();
        REQUIRE(h > -Angle::kPi);
        REQUIRE(h <= Angle::kPi);
        REQUIRE(isFinitePose(rig.loc.pose()));

        reason = m.tick();

        // VOLTAGE + WHEEL BUDGET (what actually reached the motors)
        const double battery = rig.h.battery().voltage().value();
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            const double v = rig.h.motor(w).commandedVoltage().value();
            REQUIRE(std::isfinite(v));
            REQUIRE(std::abs(v) <= 12.0 + 1e-9);
            REQUIRE(std::abs(v) <= battery + 1e-9);
            if (std::abs(v) > kS) {  // FF-implied wheel speed vs the budget
                REQUIRE((std::abs(v) - kS) / kV <= maxWheel + 1e-6);
            }
        }

        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{dt});
        }

        // PHYSICALITY + PROGRESS + FINITENESS (truth side)
        const Pose2d truth = rig.h.truePose();
        REQUIRE(isFinitePose(truth));
        REQUIRE(posErr(truth, prevTruth) <= maxLin * dt * 1.25 + 1e-9);
        REQUIRE(posErr(truth, target) <= initialDist + 5.0);
        prevTruth = truth;
    }
    REQUIRE(reason == ExitReason::Settled);  // TERMINATION (and arrival)

    // AUTHORITY — audited from the motion's own record stream.
    const double vyLimit = kin.strafeAuthority() * maxLin;
    int audited = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        if (rec.activeCommandState == 0) {
            continue;
        }
        const ChassisSpeeds body =
            shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
        REQUIRE(std::abs(body.vy().value()) <= vyLimit + 1e-9);
        ++audited;
    }
    REQUIRE(audited > 0);

    return TrialStats{posErr(rig.h.truePose(), target), headErr(rig.h.truePose(), target)};
}

}  // namespace

TEST_CASE("C1 sweep: X-drive — 24 seeded random start->target trials hold every invariant") {
    const auto kin = xDrive(Length{7.0});
    double worstPos = 0.0;
    double worstHead = 0.0;
    for (std::uint64_t seed = 1; seed <= 24; ++seed) {
        CAPTURE(seed);
        Rng rng{seed * 7919};
        const Pose2d start = randomPose(rng, 50.0);
        const Pose2d target = randomPose(rng, 50.0);
        const TrialStats s = runInvariantTrial(kin, start, target);
        REQUIRE(s.truthErr < 0.6);
        REQUIRE(s.headingErr < 0.025);
        worstPos = std::max(worstPos, s.truthErr);
        worstHead = std::max(worstHead, s.headingErr);
    }
    MESSAGE("clean-plant sweep worst truth-vs-target: pos=", worstPos,
            " in, heading=", worstHead * 180.0 / Angle::kPi, " deg");
}

TEST_CASE("C1 sweep: tank — 12 seeded along-axis trials (fwd AND reverse) hold every invariant") {
    const TankKinematics kin{Length{12.0}};
    for (std::uint64_t seed = 1; seed <= 12; ++seed) {
        CAPTURE(seed);
        Rng rng{seed * 104729};
        const Pose2d start = randomPose(rng, 40.0);
        const double d = rng.uniform(-40.0, 40.0);  // signed: reverse trials included
        const double th = start.heading().radians();
        const Pose2d target{Length{start.x().value() + d * std::cos(th)},
                            Length{start.y().value() + d * std::sin(th)}, start.heading()};
        const TrialStats s = runInvariantTrial(kin, start, target);
        REQUIRE(s.truthErr < 0.6);
        REQUIRE(s.headingErr < 0.025);
    }
}

// ── Saturation: MAX translation demand + MAX rotation demand simultaneously. ──
// Bug caught: desaturation missing (per-wheel clamping distorts the direction —
// the landing misses) or mis-scaled (budget invariant reds). The wheel budget
// must actually BIND during this trial, proving desaturate() is live in the
// path, not vacuously satisfied.
TEST_CASE("C1 sweep: simultaneous max translation + max rotation saturates, desaturates, lands") {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{Length{-45.0}, Length{-45.0}, Angle::degrees(179.0)};
    MotionRig rig{kin, pcfg};
    const auto mcfg = motionConfig();
    const Pose2d target{Length{45.0}, Length{45.0}, Angle::degrees(-1.0)};
    MoveToPose m{rig.deps, target, mcfg, 8.0};
    m.start();

    bool budgetBound = false;  // some tick must RIDE the wheel budget
    auto reason = ExitReason::Running;
    for (int i = 0; i < 1200 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        double maxImplied = 0.0;
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            const double v = std::abs(rig.h.motor(w).commandedVoltage().value());
            if (v > mcfg.wheelFf.kS) {
                maxImplied = std::max(maxImplied, (v - mcfg.wheelFf.kS) / mcfg.wheelFf.kV);
            }
            REQUIRE(maxImplied <= mcfg.maxWheelSpeed.value() + 1e-6);
        }
        if (maxImplied > mcfg.maxWheelSpeed.value() - 0.5) {
            budgetBound = true;
        }
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(budgetBound);
    CHECK(posErr(rig.h.truePose(), target) < 0.6);
    CHECK(headErr(rig.h.truePose(), target) < 0.025);
}

// ── The strafe-authority clamp, made OBSERVABLE (mutation #5's home). ──
// Bug caught: the upstream authority clamp (§13 #5 — the motion layer's half
// of the F5 choreography) silently dropped. On tank the clamp is structurally
// unobservable (toWheels discards vy) and on the X-drive it is vacuous
// (authority 1.0) — so this test builds the drive that actually CONSUMES it:
// X-drive geometry with a DECLARED authority of 0.35, the exact contract shape
// C3's H-drive arrives with. A lateral move must ride the 0.35·maxLin body-vy
// limit (the clamp visibly BINDS) and never exceed it.
TEST_CASE("C1 sweep: a fractional-authority drive has its body vy clamped to authority*maxLin") {
    // X geometry, but the drive DECLARES it can only sustain 35% lateral —
    // the read-only query the motion layer must clamp against (F5).
    const double c = std::sqrt(2.0) / 2.0;
    const shulib::kinematics::MatrixKinematics kin{{{-c, +c, 7.0},
                                                    {-c, -c, 7.0},
                                                    {+c, -c, 7.0},
                                                    {+c, +c, 7.0}},
                                                   0.35};
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    const auto mcfg = motionConfig();
    // A nearly-pure-lateral move at heading 0: raw body-vy demand ≈ 60 in/s,
    // far above the 0.35·60 = 21 in/s authority limit.
    MoveToPose m{rig.deps, Pose2d{Length{2.0}, Length{30.0}, Angle{}}, mcfg, 8.0};
    REQUIRE(rig.run(m, 1200) == ExitReason::Settled);

    const double vyLimit = 0.35 * mcfg.maxLinearSpeed.value();
    double maxBodyVy = 0.0;
    int audited = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        if (rec.activeCommandState == 0) {
            continue;
        }
        const ChassisSpeeds body =
            shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
        REQUIRE(std::abs(body.vy().value()) <= vyLimit + 1e-9);  // never exceeded
        maxBodyVy = std::max(maxBodyVy, std::abs(body.vy().value()));
        ++audited;
    }
    REQUIRE(audited > 50);
    CHECK(maxBodyVy > 0.9 * vyLimit);  // the clamp genuinely BOUND (not vacuous)
}

// ── Battery compensation, made OBSERVABLE (mutation #11's home). ──
// Bug caught: compensateForBattery() dropped from the output stage. On a
// nominal pack it never binds (demands cap ≈ 11.4 V < 12.6 V), so this test
// runs on a WEAK 8 V pack: every commanded wheel voltage must respect the
// battery ceiling — voltage-starved, flagged-not-faked — and the motion must
// still complete (slower), which is exactly the §M2 brownout-aware contract.
TEST_CASE("C1 sweep: on a weak pack every commanded voltage respects the battery ceiling") {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.batteryVoltage = shulib::units::Voltage{8.0};
    MotionRig rig{kin, pcfg};
    MoveToPose m{rig.deps, Pose2d{Length{30.0}, Length{0.0}, Angle{}}, motionConfig(), 8.0};
    m.start();
    bool ceilingBound = false;
    auto reason = ExitReason::Running;
    for (int i = 0; i < 1200 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        const double battery = rig.h.battery().voltage().value();
        for (int w = 0; w < rig.h.motorCount(); ++w) {
            const double v = std::abs(rig.h.motor(w).commandedVoltage().value());
            REQUIRE(v <= battery + 1e-9);  // the compensation ceiling, every tick
            if (v > battery - 0.01) {
                ceilingBound = true;
            }
        }
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);  // starved, not stranded
    CHECK(ceilingBound);                     // the clamp genuinely engaged
}

// ── Mid-motion retarget by abandonment (the C2 cancel shape, by hand). ──
// Bug caught: motion-B startup transients poisoned by motion-A state (stale
// PID history, stale watchdog) — B must behave as if freshly commanded.
TEST_CASE("C1 sweep: abandoning motion A mid-flight and starting B lands on B") {
    const auto kin = xDrive(Length{7.0});
    MotionRig rig{kin};
    MoveToPose a{rig.deps, Pose2d{Length{40.0}, Length{0.0}, Angle{}}, motionConfig()};
    a.start();
    for (int i = 0; i < 50; ++i) {  // 0.5 s toward +x, then abandon mid-cruise
        rig.loc.update();
        REQUIRE(a.tick() == ExitReason::Running);
        rig.h.plant().step(Time{0.01});
    }
    REQUIRE(rig.h.truePose().x().value() > 10.0);
    MoveToPose b{rig.deps, Pose2d{Length{-20.0}, Length{10.0}, Angle::degrees(90.0)},
                 motionConfig()};
    REQUIRE(rig.run(b, 900) == ExitReason::Settled);
    CHECK(posErr(rig.h.truePose(), b.target()) < 0.6);
    CHECK(headErr(rig.h.truePose(), b.target()) < 0.025);
}
