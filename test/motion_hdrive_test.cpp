// C3 MOTION-LAYER PROOF: the 15″ H-bot runs the SAME motion code as the X-bot,
// UNMODIFIED — the M2 Definition of Done ("the same auton runs the H-bot"),
// exercised primitive by primitive on the HA-55 stand-in geometry, plus the
// C3 contract items the brief names:
//
//   * BEYOND-AUTHORITY: commanded strafe past authority·maxLinearSpeed is
//     clamped by the MOTION layer (C1's clamp, confirmed doing the right thing
//     at authority < 1), the fallback ENGAGES, and it is VISIBLE — record-level
//     (strafeFallbackActive) and terminal-level (TermSink " SFB"). A SILENT
//     FALLBACK IS A FAILING TEST, in exactly those words: every beyond-authority
//     case REQUIREs flagged ticks > 0.
//   * THE FALLBACK'S SHAPE: authority-limited translation with rotation FREE —
//     turn-WHILE-drive, never a sequenced turn-then-drive (C1's landmine stands;
//     StrafeTo's held heading proves the fallback cannot rotate what it must
//     not). When the routine's target heading aligns with the displacement, the
//     closed loop demonstrably migrates translation from the weak strafe axis to
//     the strong drive axis as heading converges — and is measurably FASTER than
//     the pure crab (the A/B case), which is what makes the fallback CORRECT
//     rather than merely bounded.
//   * X-DRIVE NEVER FALLS BACK (structural: authority 1.0 + the norm cap), and
//     tank's undeliverable strafe is now telemetry-visible instead of silent.
//   * The H-drive survives A3's composed hostility and runs a full scheduler
//     chain — the same engine, zero drivetrain branches anywhere in motion/.

#include "doctest.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "motion_test_rig.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/diag/term_sink.hpp"
#include "shulib/hal/fake/fake_char_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/rng.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::diag::isFinitePose;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::motion::DriveBrake;
using shulib::motion::HoldPose;
using shulib::motion::MoveToPose;
using shulib::motion::StrafeTo;
using shulib::motion::TurnTo;
using shulib::sim::FullHostility;
using shulib::sim::Rng;
using shulib::units::Time;

namespace {

// H-drive legs can be strafe-limited (21 in/s lateral at the HA-54 default), so
// the per-motion budget is wider than the X suites' 8 s. Still watchdog-bounded.
constexpr double kHTimeout = 14.0;
constexpr int kHMaxTicks = 2200;

struct SfbAudit {
    int motionRecords = 0;   // records with a non-idle command state
    int flagged = 0;         // …with strafeFallbackActive set
    double maxBodyVy = 0.0;  // recovered |body vy| across motion records
    bool everExceeded = false;  // any record beyond vyLimit (+eps) — must stay false
    bool flaggedOffLimit = false;  // SFB set while |body vy| NOT riding the limit
};

/// Audit a record stream against the authority contract: |body vy| of the FINAL
/// achievable command never exceeds authority·maxLin, and every SFB-flagged tick
/// is genuinely RIDING the limit (the flag may never fire on an unclamped tick).
SfbAudit auditRecords(const shulib::hal::fake::FakeTelemetrySink& records, double vyLimit) {
    SfbAudit a;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        if (rec.activeCommandState == 0) {
            continue;
        }
        ++a.motionRecords;
        const ChassisSpeeds body =
            shulib::math::fieldToRobot(rec.commanded, rec.measuredPose.heading());
        const double vy = std::abs(body.vy().value());
        a.maxBodyVy = std::max(a.maxBodyVy, vy);
        if (vy > vyLimit + 1e-9) {
            a.everExceeded = true;
        }
        if (rec.strafeFallbackActive) {
            ++a.flagged;
            if (std::abs(vy - vyLimit) > 1e-6) {
                a.flaggedOffLimit = true;  // flagged but not clamped ⇒ the flag lies
            }
        }
    }
    return a;
}

}  // namespace

// ═══ C1's primitives, UNMODIFIED, on the H-drive ═══════════════════════════════════

// Bug caught: any hidden X-drive assumption in the motion layer — a primitive
// that needs a drivetrain branch to converge is the abstraction failing, which
// this chunk exists to detect before F6 freezes the facade.
TEST_CASE("H-drive: MoveToPose reaches seeded random field poses — every universal "
          "invariant, authority audited from records") {
    const auto kin = hBotKinematics();
    const auto mcfg = motionConfig();
    const double vyLimit = kin.strafeAuthority() * mcfg.maxLinearSpeed.value();
    double worstPos = 0.0;
    double worstHead = 0.0;
    int trialsWithFallback = 0;
    for (std::uint64_t seed = 1; seed <= 12; ++seed) {
        CAPTURE(seed);
        Rng rng{seed * 15485863ULL};
        const Pose2d start{Length{rng.uniform(-45.0, 45.0)}, Length{rng.uniform(-45.0, 45.0)},
                           Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
        const Pose2d target{Length{rng.uniform(-45.0, 45.0)}, Length{rng.uniform(-45.0, 45.0)},
                            Angle::radians(rng.uniform(-Angle::kPi, Angle::kPi))};
        auto pcfg = plantConfig();
        pcfg.plant.initialPose = start;
        shulib::hal::fake::FakeTelemetrySink records;
        MotionRig rig{kin, pcfg, &records};
        MoveToPose m{rig.deps, target, mcfg, kHTimeout};
        m.start();

        const double initialDist = posErr(start, target);
        const double maxLin = mcfg.maxLinearSpeed.value();
        auto reason = ExitReason::Running;
        Pose2d prevTruth = rig.h.truePose();
        for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
            rig.loc.update();
            const double h = rig.loc.pose().heading().radians();
            REQUIRE(h > -Angle::kPi);
            REQUIRE(h <= Angle::kPi);
            REQUIRE(isFinitePose(rig.loc.pose()));
            reason = m.tick();
            const double battery = rig.h.battery().voltage().value();
            for (int w = 0; w < rig.h.motorCount(); ++w) {
                const double v = rig.h.motor(w).commandedVoltage().value();
                REQUIRE(std::isfinite(v));
                REQUIRE(std::abs(v) <= 12.0 + 1e-9);
                REQUIRE(std::abs(v) <= battery + 1e-9);
            }
            if (reason == ExitReason::Running) {
                rig.h.plant().step(Time{0.01});
            }
            const Pose2d truth = rig.h.truePose();
            REQUIRE(isFinitePose(truth));
            // H truth speed can reach hypot(maxLin, vyLimit) ≈ 1.06·maxLin:
            REQUIRE(posErr(truth, prevTruth) <= maxLin * 0.01 * 1.25 + 1e-9);
            REQUIRE(posErr(truth, target) <= initialDist + 5.0);
            prevTruth = truth;
        }
        REQUIRE(reason == ExitReason::Settled);
        worstPos = std::max(worstPos, posErr(rig.h.truePose(), target));
        worstHead = std::max(worstHead, headErr(rig.h.truePose(), target));
        REQUIRE(posErr(rig.h.truePose(), target) < 0.6);
        REQUIRE(headErr(rig.h.truePose(), target) < 0.025);

        const SfbAudit a = auditRecords(records, vyLimit);
        REQUIRE(a.motionRecords > 0);
        REQUIRE_FALSE(a.everExceeded);     // the C1 clamp does the right thing at 0.35
        REQUIRE_FALSE(a.flaggedOffLimit);  // and the flag never lies
        trialsWithFallback += (a.flagged > 0) ? 1 : 0;
    }
    MESSAGE("H-drive sweep worst truth-vs-target: pos=", worstPos,
            " in, heading=", worstHead * 180.0 / Angle::kPi,
            " deg; trials that entered fallback: ", trialsWithFallback, "/12");
    // Random field poses are lateral-heavy often enough that the fallback must
    // have been exercised — otherwise this sweep proves less than it claims:
    REQUIRE(trialsWithFallback >= 4);
}

TEST_CASE("H-drive: TurnTo rotates in place — no translation, no fallback ever") {
    const auto kin = hBotKinematics();
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    TurnTo m{rig.deps, Angle::degrees(117.0), motionConfig(), kHTimeout};
    REQUIRE(rig.run(m, kHMaxTicks) == ExitReason::Settled);
    // In place: the off-centre strafe wheel MUST counter-roll during rotation
    // (h_drive_test pins a·ω ≠ 0) — if the kinematics dropped that coupling the
    // rotation would translate. Truth must stay put:
    CHECK(posErr(rig.h.truePose(), Pose2d{}) < 0.35);
    CHECK(std::abs(rig.h.truePose().heading().errorTo(Angle::degrees(117.0))) < 0.025);
    // A pure rotation demands no vy: the fallback flag must never appear.
    for (int i = 0; i < records.recordCount(); ++i) {
        REQUIRE_FALSE(records.recordAt(i).strafeFallbackActive);
    }
}

TEST_CASE("H-drive: DriveBrake stops and stays; HoldPose recovers from a LATERAL shove "
          "through the weak axis") {
    const auto kin = hBotKinematics();
    MotionRig rig{kin, plantConfig()};
    // Get moving diagonally, then brake:
    MoveToPose cruise{rig.deps, Pose2d{Length{30.0}, Length{10.0}, Angle{}}, motionConfig(),
                      kHTimeout};
    cruise.start();
    for (int i = 0; i < 40; ++i) {
        rig.loc.update();
        REQUIRE(cruise.tick() == ExitReason::Running);
        rig.h.plant().step(Time{0.01});
    }
    DriveBrake brake{rig.deps, motionConfig(), 5.0};
    REQUIRE(rig.run(brake, 800) == ExitReason::Settled);
    const Pose2d rest = rig.h.truePose();
    for (int i = 0; i < 50; ++i) {  // stays at rest
        rig.h.plant().step(Time{0.01});
    }
    CHECK(posErr(rig.h.truePose(), rest) < 0.05);

    // HoldPose: displace the truth LATERALLY (the axis the H-drive is weakest
    // on) and require active recovery — the disturbance-rejection loop must
    // work through the authority-limited channel. Bug caught: a hold loop that
    // only recovers along the strong axis.
    HoldPose hold{rig.deps, 5.0, motionConfig()};
    hold.start();
    for (int i = 0; i < 30; ++i) {
        rig.loc.update();
        REQUIRE(hold.tick() == ExitReason::Running);
        rig.h.plant().step(Time{0.01});
    }
    const Pose2d held = rig.h.truePose();
    // Shove: command pure lateral through the harness for 0.3 s, then let the
    // hold fight it back.
    for (int i = 0; i < 30; ++i) {
        rig.loc.update();
        (void)hold.tick();
        rig.h.commandBodyTwist(ChassisSpeeds{shulib::units::Velocity{0.0},
                                             shulib::units::Velocity{25.0},
                                             shulib::units::AngularVelocity{0.0}});
        rig.h.plant().step(Time{0.01});
    }
    REQUIRE(posErr(rig.h.truePose(), held) > 1.0);  // the shove genuinely moved it
    auto reason = ExitReason::Running;
    for (int i = 0; i < 800 && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = hold.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    CHECK(posErr(rig.h.truePose(), held) < 0.6);
}

// ═══ Beyond authority: the clamp, the fallback, the VISIBILITY ═════════════════════

// Bug caught (severally): the C1 clamp mis-sized at authority < 1 (the D11
// confirmation — truth may never crab faster than authority·maxLin); a SILENT
// fallback (zero flagged records while the clamp binds = FAIL); a flag that
// fires without the clamp riding the limit; a fallback that stays engaged at
// settle (the exit record must be quiet).
TEST_CASE("H-drive: StrafeTo beyond authority — clamped to authority*maxLin, heading "
          "held, fallback VISIBLE, never silent") {
    const auto kin = hBotKinematics();
    const auto mcfg = motionConfig();
    const double vyLimit = kin.strafeAuthority() * mcfg.maxLinearSpeed.value();  // 21 in/s
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    // 30 in pure lateral: raw demand ≈ 60 in/s ≫ 21 — deep beyond authority.
    StrafeTo m{rig.deps, Length{0.0}, Length{30.0}, mcfg, kHTimeout};
    m.start();
    auto reason = ExitReason::Running;
    double maxTrueVy = 0.0;
    double maxTrueHeadErr = 0.0;
    for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
        maxTrueVy = std::max(maxTrueVy, std::abs(rig.h.trueBodyTwist().vy().value()));
        maxTrueHeadErr =
            std::max(maxTrueHeadErr, std::abs(rig.h.truePose().heading().errorTo(Angle{})));
    }
    REQUIRE(reason == ExitReason::Settled);  // slower, not stranded
    CHECK(posErr(rig.h.truePose(), Pose2d{Length{0.0}, Length{30.0}, Angle{}}) < 0.6);

    // PHYSICS of the clamp, from ground truth (not from the motion's own claim):
    // the robot genuinely crabs at ≈ the authority limit and never meaningfully
    // above it (kS deadband + matched FF keep truth at/below the command).
    MESSAGE("beyond-authority crab: true |vy| peak = ", maxTrueVy, " in/s (limit ", vyLimit,
            "), heading held within ", maxTrueHeadErr * 180.0 / Angle::kPi, " deg");
    REQUIRE(maxTrueVy <= vyLimit * 1.05 + 1e-9);
    REQUIRE(maxTrueVy > 0.85 * vyLimit);  // the limit genuinely RIDES (not vacuous)
    // The fallback must NOT rotate: StrafeTo's held heading is the proof the
    // engine never sequences a turn it was not asked for.
    REQUIRE(maxTrueHeadErr < 2.0 * Angle::kPi / 180.0);

    // VISIBILITY — a silent fallback is a failing test:
    const SfbAudit a = auditRecords(records, vyLimit);
    REQUIRE(a.flagged > 50);            // engaged for the whole cruise, visibly
    REQUIRE_FALSE(a.everExceeded);
    REQUIRE_FALSE(a.flaggedOffLimit);
    // The EXIT record is quiet (motors stopped, no clamp, no flag):
    const auto& last = records.recordAt(records.recordCount() - 1);
    CHECK_FALSE(last.strafeFallbackActive);
}

// Bug caught: the whole visibility PATH broken anywhere producer→record→sink —
// the record flag could be set yet never reach a human. This is the end-to-end
// legibility pin: the terminal stream itself must carry " SFB" while the H-bot
// runs beyond authority.
TEST_CASE("H-drive: the fallback is legible on the TERMINAL — TermSink lines carry SFB") {
    const auto kin = hBotKinematics();
    shulib::hal::fake::FakeCharSink out;
    MotionRig rig{kin, plantConfig()};
    shulib::diag::TermSink term{rig.h.clock(), out};
    // Route the motion's telemetry through the TermSink via a deps copy over a
    // shadow context — the rig wiring stays untouched (RobotContext's sink is
    // fixed at construction; this is the same shadow-context shape C2 uses).
    auto deps = rig.deps;
    shulib::chassis::RobotContext ctx2{shulib::chassis::RobotContextConfig{
        .clock = &rig.h.clock(),
        .driveMotors = rig.h.context().driveMotors(),
        .imu = &rig.h.imu(),
        .gps = &rig.h.gps(),
        .battery = &rig.h.battery(),
        .telemetry = &term,
        .tags = &rig.h.context().tags(),
        .vision = &rig.h.context().vision()}};
    deps.ctx = &ctx2;
    StrafeTo m{deps, Length{0.0}, Length{25.0}, motionConfig(), kHTimeout};
    m.start();
    auto reason = ExitReason::Running;
    for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
        rig.loc.update();
        reason = m.tick();
        if (reason == ExitReason::Running) {
            rig.h.plant().step(Time{0.01});
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    REQUIRE(out.text().find(" SFB") != std::string::npos);
}

// Bug caught: the flag misfiring on a full-authority drive. On the X-drive the
// clamp is STRUCTURALLY unreachable (authority 1.0 ⇒ vyLimit == the norm cap ≥
// any post-rotation |vy|) — so ANY flagged record is a defect. Pure-lateral
// moves are included precisely because they maximize the vy demand.
TEST_CASE("X-drive: strafeFallbackActive is NEVER set — the X-bot never falls back") {
    const auto kin = xDrive(Length{7.0});
    for (const auto& target : {Pose2d{Length{0.0}, Length{40.0}, Angle{}},
                               Pose2d{Length{-25.0}, Length{25.0}, Angle::degrees(90.0)},
                               Pose2d{Length{30.0}, Length{-35.0}, Angle::degrees(-135.0)}}) {
        shulib::hal::fake::FakeTelemetrySink records;
        MotionRig rig{kin, plantConfig(), &records};
        MoveToPose m{rig.deps, target, motionConfig(), 8.0};
        REQUIRE(rig.run(m, 1200) == ExitReason::Settled);
        int motionRecords = 0;
        for (int i = 0; i < records.recordCount(); ++i) {
            const auto& rec = records.recordAt(i);
            if (rec.activeCommandState != 0) {
                ++motionRecords;
            }
            REQUIRE_FALSE(rec.strafeFallbackActive);
        }
        REQUIRE(motionRecords > 0);  // the audit is not vacuous
    }
}

// Bug caught: tank's undeliverable strafe staying SILENT. C1 D12 pinned the
// honest TimedOut; C3 adds the WHY to the telemetry — the whole attempt is
// flagged as fallback (vyLimit = 0: every real lateral demand is undeliverable).
TEST_CASE("tank: an undeliverable StrafeTo times out honestly AND visibly (SFB through "
          "the attempt)") {
    const TankKinematics kin{Length{12.0}};
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    StrafeTo m{rig.deps, Length{0.0}, Length{24.0}, motionConfig(), 2.0};
    REQUIRE(rig.run(m, 400) == ExitReason::TimedOut);
    int flagged = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        if (records.recordAt(i).strafeFallbackActive) {
            ++flagged;
        }
    }
    REQUIRE(flagged > 50);  // the drive TOLD you it cannot do this
}

// Bug caught: the SFB legibility floor removed or mis-sized. A feasible tank
// move with a DELIBERATE hair of lateral offset (0.1 in ⇒ lateral demand
// ≈ kP·0.1 = 0.3 in/s, above zero but under the 1%-of-maxLin floor) generates
// real-but-negligible clamped demand every tick; flagging it would light SFB for
// entire healthy tank runs — a permanently-on flag is as undebuggable as a
// silent one. This is the floor's designed detector (its removal turns exactly
// this case red, nothing else).
TEST_CASE("tank: sub-perceptible lateral demand does NOT light the fallback flag "
          "(the legibility floor)") {
    const TankKinematics kin{Length{12.0}};
    shulib::hal::fake::FakeTelemetrySink records;
    MotionRig rig{kin, plantConfig(), &records};
    // Along-heading move with a 0.1 in lateral offset — feasible for tank (the
    // offset is far inside the 0.5 in settle tolerance).
    MoveToPose m{rig.deps, Pose2d{Length{30.0}, Length{0.1}, Angle{}}, motionConfig(), 8.0};
    REQUIRE(rig.run(m, 1200) == ExitReason::Settled);
    int motionRecords = 0;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        if (rec.activeCommandState != 0) {
            ++motionRecords;
        }
        REQUIRE_FALSE(rec.strafeFallbackActive);
    }
    REQUIRE(motionRecords > 0);
}

// ═══ The fallback's SHAPE: turn-WHILE-drive, and why it is correct ═════════════════

// Bug caught: (a) a sequenced turn-then-drive hiding anywhere in the engine —
// rotation and translation must overlap (the C1 simultaneity discipline, now
// exercised at authority 0.35); (b) a fallback that never disengages — once
// heading converges the displacement is body-FORWARD and the clamp must go
// quiet; (c) the fallback being WORSE than the pure crab it exists to beat.
TEST_CASE("H-drive: with displacement-aligned target heading the loop migrates "
          "translation to the drive axis — fallback engages, then clears, and beats "
          "the pure crab") {
    const auto kin = hBotKinematics();
    const auto mcfg = motionConfig();
    const double vyLimit = kin.strafeAuthority() * mcfg.maxLinearSpeed.value();

    // A: the auton asks "40 in to the left, END FACING the direction of travel".
    shulib::hal::fake::FakeTelemetrySink recA;
    MotionRig rigA{kin, plantConfig(), &recA};
    MoveToPose a{rigA.deps, Pose2d{Length{0.0}, Length{40.0}, Angle::degrees(90.0)}, mcfg,
                 kHTimeout};
    a.start();
    auto reason = ExitReason::Running;
    bool rotationAndTranslationOverlapped = false;
    for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
        rigA.loc.update();
        reason = a.tick();
        if (reason == ExitReason::Running) {
            rigA.h.plant().step(Time{0.01});
        }
        const auto tw = rigA.h.trueBodyTwist();
        if (std::abs(tw.omega().value()) > 0.5
            && std::hypot(tw.vx().value(), tw.vy().value()) > 10.0) {
            rotationAndTranslationOverlapped = true;  // turn-WHILE-drive, observed
        }
    }
    REQUIRE(reason == ExitReason::Settled);
    const double tAligned = rigA.h.clock().now().value();
    REQUIRE(rotationAndTranslationOverlapped);  // never a sequenced decomposition

    // The flag TRAJECTORY: engaged early (heading 0 ⇒ displacement is body-left,
    // demand ≫ 21), clear at the end (displacement is body-forward once heading
    // ≈ 90°). Assert both phases exist and the LAST flagged record precedes the
    // last quarter of the run — i.e. the fallback disengages as the drive axis
    // takes over.
    int firstFlagged = -1;
    int lastFlagged = -1;
    int motionRecords = 0;
    for (int i = 0; i < recA.recordCount(); ++i) {
        const auto& rec = recA.recordAt(i);
        if (rec.activeCommandState == 0) {
            continue;
        }
        ++motionRecords;
        if (rec.strafeFallbackActive) {
            if (firstFlagged < 0) {
                firstFlagged = motionRecords;
            }
            lastFlagged = motionRecords;
        }
    }
    REQUIRE(firstFlagged > 0);                        // it engaged (visibly)
    REQUIRE(firstFlagged <= 5);                       // …from the start of the leg
    REQUIRE(lastFlagged < (motionRecords * 3) / 4);   // …and let go before the end

    // B: the pure crab of the same displacement (StrafeTo holds heading 0).
    MotionRig rigB{kin, plantConfig()};
    StrafeTo b{rigB.deps, Length{0.0}, Length{40.0}, mcfg, kHTimeout};
    REQUIRE(rigB.run(b, kHMaxTicks) == ExitReason::Settled);
    const double tCrab = rigB.h.clock().now().value();

    MESSAGE("aligned-heading MoveToPose: ", tAligned, " s vs pure crab StrafeTo: ", tCrab,
            " s (authority ", kin.strafeAuthority(), ", vyLimit ", vyLimit, " in/s)");
    // The fallback mode must BEAT the crab it falls back from — that is what
    // makes turn-while-drive the CORRECT degraded behaviour for this drive:
    REQUIRE(tAligned < tCrab);
}

// ═══ A3 composed hostility on the H-drive ══════════════════════════════════════════

// Bug caught: an H-drive-specific divergence under the hostile world — the
// authority clamp interacting badly with hostile estimates (e.g. noise-driven
// lateral demand parked at the clamp forever), or any finiteness escape. Bounds
// are looser than X's (slower legs ⇒ more drift seconds) but derived the same
// way: observed + margin, inside the drift-physics ceiling.
TEST_CASE("H-drive: MoveToPose and StrafeTo settle under FULL composed hostility, "
          "bounded and finite every tick") {
    const auto kin = hBotKinematics();
    for (const std::uint64_t seed : {11ULL, 22ULL, 33ULL}) {
        CAPTURE(seed);
        FullHostility world{};
        auto pcfg = plantConfig();
        pcfg.plant.seed = seed;
        MotionRig rig{kin, pcfg, nullptr, &world.model()};

        const Pose2d t1{Length{24.0}, Length{18.0}, Angle::degrees(35.0)};
        MoveToPose m{rig.deps, t1, motionConfig(), kHTimeout};
        m.start();
        auto reason = ExitReason::Running;
        for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
            rig.loc.update();
            REQUIRE(isFinitePose(rig.loc.pose()));
            reason = m.tick();
            if (reason == ExitReason::Running) {
                rig.h.plant().step(Time{0.01});
            }
            REQUIRE(isFinitePose(rig.h.truePose()));
        }
        REQUIRE(reason == ExitReason::Settled);
        CHECK(posErr(rig.h.truePose(), t1) < 2.5);

        StrafeTo s{rig.deps, Length{10.0}, Length{34.0}, motionConfig(), kHTimeout};
        reason = ExitReason::Running;
        s.start();
        for (int i = 0; i < kHMaxTicks && reason == ExitReason::Running; ++i) {
            rig.loc.update();
            reason = s.tick();
            if (reason == ExitReason::Running) {
                rig.h.plant().step(Time{0.01});
            }
            REQUIRE(isFinitePose(rig.h.truePose()));
        }
        REQUIRE(reason == ExitReason::Settled);
        CHECK(posErr(rig.h.truePose(), Pose2d{Length{10.0}, Length{34.0}, Angle{}}) < 2.5);
    }
}

// ═══ C2's scheduler, unmodified, on the H-drive ════════════════════════════════════

// Bug caught: any drivetrain awareness leaking into the scheduler (it reads only
// wheelCount for records — C2's own note), or engine bookkeeping diverging from
// physics on the second robot.
TEST_CASE("H-drive: the C2 scheduler runs a move→turn→strafe chain — graded on truth, "
          "bookkeeping consistent, ids stamped") {
    const auto kin = hBotKinematics();
    shulib::hal::fake::FakeTelemetrySink records;
    SchedulerRig sr{kin, plantConfig(), &records};

    MoveToPose m1{sr.sched.deps(), Pose2d{Length{20.0}, Length{25.0}, Angle::degrees(45.0)},
                  motionConfig(), kHTimeout};
    TurnTo m2{sr.sched.deps(), Angle::degrees(-90.0), motionConfig(), kHTimeout};
    // At heading −90° (facing field −Y), body +Y (left) is field +X — so a
    // field +X displacement is a PURE LATERAL leg: genuinely beyond authority.
    StrafeTo m3{sr.sched.deps(), Length{40.0}, Length{25.0}, motionConfig(), kHTimeout};

    REQUIRE(sr.run(m1) == ExitReason::Settled);
    REQUIRE(sr.run(m2) == ExitReason::Settled);
    REQUIRE(sr.run(m3) == ExitReason::Settled);

    CHECK(posErr(sr.rig.h.truePose(), Pose2d{Length{40.0}, Length{25.0}, Angle::degrees(-90.0)})
          < 0.8);
    CHECK(std::abs(sr.rig.h.truePose().heading().errorTo(Angle::degrees(-90.0))) < 0.03);

    CHECK(sr.sched.motionsStarted() == 3);
    CHECK(sr.sched.motionsSettled() == 3);
    CHECK(sr.sched.motionsCancelled() == 0);
    CHECK(sr.sched.motionsAborted() == 0);

    // Ids stamped 1/2/3 across the stream; the strafe leg (id 3, beyond
    // authority at 20 in lateral) carries SFB on stamped records:
    bool sawId3Sfb = false;
    for (int i = 0; i < records.recordCount(); ++i) {
        const auto& rec = records.recordAt(i);
        REQUIRE(rec.activeCommandId <= 3);
        if (rec.activeCommandId == 3 && rec.strafeFallbackActive) {
            sawId3Sfb = true;
        }
    }
    REQUIRE(sawId3Sfb);  // fallback visibility survives the scheduler's id stamp
}
