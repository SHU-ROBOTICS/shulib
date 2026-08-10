// C4 drive(ChassisSpeeds, Frame) — the frame-explicit manual verb. Frame
// correctness is THE bug class this rebuild exists to prevent, so it is pinned
// the way frame bugs actually surface: swept over headings (a frame bug is a
// heading-DEPENDENT error, invisible at heading 0 where the frames coincide).

#include "doctest.h"

#include <cmath>

#include "motion_test_rig.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"

using namespace motion_rig;
using shulib::control::ExitReason;
using shulib::hal::LogLevel;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::TankKinematics;
using shulib::kinematics::xDrive;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Frame;
using shulib::math::Pose2d;
using shulib::motion::MotionState;
using shulib::motion::MoveToPose;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

/// Drive at `cmd` in `frame` for `ticks` loop iterations (the documented
/// teleop shape: drive() then pace) from a start heading of `h0`, returning
/// the truth displacement direction (radians) and magnitude (inches).
struct DriveOutcome {
    double direction;
    double magnitude;
};

DriveOutcome driveFor(double h0Deg, const ChassisSpeeds& cmd, Frame frame, int ticks = 100) {
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(h0Deg)};
    ChassisRig c{kin, pcfg};
    const Pose2d start = c.rig.h.truePose();
    for (int i = 0; i < ticks; ++i) {
        c.chassis.drive(cmd, frame);
        c.pacer.pace();
    }
    const Pose2d end = c.rig.h.truePose();
    const double dx = (end.x() - start.x()).value();
    const double dy = (end.y() - start.y()).value();
    return DriveOutcome{.direction = std::atan2(dy, dx), .magnitude = std::hypot(dx, dy)};
}

/// |shortest angular difference| in degrees.
double angDiffDeg(double aRad, double bRad) {
    return std::abs(Angle::radians(aRad).errorTo(Angle::radians(bRad))) * 180.0 / Angle::kPi;
}

int warnCount(const FakeTelemetrySink& sink) {
    int n = 0;
    for (int i = 0; i < sink.size(); ++i) {
        if (sink.at(i).level == LogLevel::Warn && sink.at(i).subsystem == "CHS") {
            ++n;
        }
    }
    return n;
}

}  // namespace

// ═══ Frame correctness, swept over headings ════════════════════════════════════════

// Bug caught: ANY frame defect in the manual verb — inverse rotation
// (robotToField for fieldToRobot), sign flip, degrees into trig, Body treated
// as Field. Each shows as a heading-dependent direction error; sweeping
// headings (including the seam-adjacent 179°) leaves it nowhere to hide. At
// h0 = 0 the frames coincide, which is exactly why 0 alone would be vacuous.
TEST_CASE("C4 drive frames: FIELD command moves along the FIELD axis at every heading") {
    const ChassisSpeeds plusX{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};
    for (const double h0 : {0.0, 30.0, 90.0, 135.0, 179.0, -60.0, -120.0}) {
        CAPTURE(h0);
        const DriveOutcome out = driveFor(h0, plusX, Frame::Field);
        CHECK(out.magnitude > 15.0);                    // it genuinely drove (~20 in)
        CHECK(angDiffDeg(out.direction, 0.0) < 4.0);    // along FIELD +X, heading-independent
    }
}

// Bug caught: the mirror-image family — a BODY command leaking the field
// rotation (or none at all: at h0 = 0 both bugs vanish, hence the sweep).
TEST_CASE("C4 drive frames: BODY command moves along the ROBOT's forward at every heading") {
    const ChassisSpeeds forward{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};
    for (const double h0 : {0.0, 30.0, 90.0, 135.0, 179.0, -60.0, -120.0}) {
        CAPTURE(h0);
        const DriveOutcome out = driveFor(h0, forward, Frame::Body);
        CHECK(out.magnitude > 15.0);
        CHECK(angDiffDeg(out.direction, h0 * Angle::kPi / 180.0) < 4.0);  // along BODY +X
    }
}

// Bug caught: a lateral-axis sign error that the +X sweeps cannot see (the
// classic transposed-rotation bug maps +X onto +X at some headings while
// mirroring +Y). Field +Y must land +Y; body +Y (robot's LEFT) must land at
// heading + 90°.
TEST_CASE("C4 drive frames: the LATERAL axis lands correctly in both frames (sign pin)") {
    const ChassisSpeeds plusY{Velocity{0.0}, Velocity{20.0}, AngularVelocity{0.0}};
    for (const double h0 : {0.0, 45.0, -135.0}) {
        CAPTURE(h0);
        const DriveOutcome field = driveFor(h0, plusY, Frame::Field);
        CHECK(angDiffDeg(field.direction, Angle::kPi / 2.0) < 4.0);
        const DriveOutcome body = driveFor(h0, plusY, Frame::Body);
        CHECK(angDiffDeg(body.direction, (h0 + 90.0) * Angle::kPi / 180.0) < 4.0);
    }
}

// Bug caught: the two paths diverging where they must agree — at heading 0
// the frames coincide, so Field and Body runs of the same command must land
// within estimator noise of each other.
TEST_CASE("C4 drive frames: Field == Body at heading zero (coincidence pin)") {
    const ChassisSpeeds cmd{Velocity{15.0}, Velocity{10.0}, AngularVelocity{0.0}};
    const DriveOutcome field = driveFor(0.0, cmd, Frame::Field);
    const DriveOutcome body = driveFor(0.0, cmd, Frame::Body);
    CHECK(std::abs(field.magnitude - body.magnitude) < 0.3);
    CHECK(angDiffDeg(field.direction, body.direction) < 1.0);
}

// ═══ The shared pipeline is really in the path ═════════════════════════════════════

// Bug caught: drive() bypassing the strafe-authority clamp (commanding the
// strafe wheel past its physics) or dropping the C3 visibility contract —
// the H-drive's fallback flag must ride drive() records exactly as it rides
// motion records; tank must zero an impossible lateral demand.
TEST_CASE("C4 drive pipeline: authority clamp + SFB flag hold on H-drive; tank zeroes "
          "lateral demand") {
    // H-drive: demand 30 in/s lateral; authority 0.35 × 60 = 21 in/s.
    FakeTelemetrySink sink;
    const auto hKin = hBotKinematics();
    ChassisRig h{hKin, plantConfig(), &sink};
    const int before = sink.recordCount();
    h.chassis.drive(ChassisSpeeds{Velocity{0.0}, Velocity{30.0}, AngularVelocity{0.0}},
                    Frame::Body);
    REQUIRE(sink.recordCount() > before);
    const auto& r = sink.lastRecord();
    CHECK(r.strafeFallbackActive);                       // the clamp bound — visibly
    // The strafe wheel (index 2) carries ff(21 in/s), NOT ff(30 in/s):
    const double v2 = h.rig.h.motor(2).commandedVoltage().value();
    const double expected21 = 1.2 + 0.17 * 21.0;
    const double naive30 = 1.2 + 0.17 * 30.0;
    CHECK(std::abs(v2 - expected21) < 0.05);
    CHECK(std::abs(v2 - naive30) > 1.0);
    // Below the limit: no clamp, no flag (the flag must not cry wolf).
    h.chassis.drive(ChassisSpeeds{Velocity{0.0}, Velocity{5.0}, AngularVelocity{0.0}},
                    Frame::Body);
    CHECK_FALSE(sink.lastRecord().strafeFallbackActive);

    // Tank: a pure lateral demand is undeliverable — every wheel commands 0 V
    // and the robot does not move.
    const TankKinematics tKin{Length{12.0}};
    ChassisRig t{tKin};
    for (int i = 0; i < 30; ++i) {
        t.chassis.drive(ChassisSpeeds{Velocity{0.0}, Velocity{30.0}, AngularVelocity{0.0}},
                        Frame::Body);
        t.pacer.pace();
    }
    for (int w = 0; w < t.rig.h.motorCount(); ++w) {
        CHECK(t.rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
    CHECK(posErr(t.rig.h.truePose(), Pose2d{}) < 1e-6);
}

// Bug caught: the record schema breaking for the manual verb — `commanded`
// must be the FIELD-frame final achievable command (§18.2) no matter which
// frame the caller used.
TEST_CASE("C4 drive record: commanded is FIELD-frame regardless of input frame") {
    FakeTelemetrySink sink;
    const auto kin = xDrive(Length{7.0});
    auto pcfg = plantConfig();
    pcfg.plant.initialPose = Pose2d{Length{0.0}, Length{0.0}, Angle::degrees(90.0)};
    ChassisRig c{kin, pcfg, &sink};
    // Let the estimate see the true 90° heading before auditing the record.
    for (int i = 0; i < 5; ++i) {
        c.chassis.drive(ChassisSpeeds{}, Frame::Body);
        c.pacer.pace();
    }
    // BODY +X at heading 90° is FIELD +Y:
    c.chassis.drive(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}},
                    Frame::Body);
    const auto& r = sink.lastRecord();
    CHECK(std::abs(r.commanded.vx().value()) < 0.5);
    CHECK(std::abs(r.commanded.vy().value() - 10.0) < 0.5);
}

// ═══ Pre-empt: a manual command supersedes a motion, safely ════════════════════════

// Bug caught: drive() and an active motion commanding on the same tick (the
// double-commander bug — the exact failure one-active-motion exists to
// prevent), or drive() silently ignored while a motion runs.
TEST_CASE("C4 drive pre-empt: drive() during an active motion cancels it first, then "
          "commands") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    MoveToPose m{c.chassis.deps(), Pose2d{Length{40.0}, Length{0.0}, Angle{}},
                 motionConfig(), 8.0};
    c.chassis.scheduler().async(m);
    for (int i = 0; i < 30; ++i) {  // genuinely mid-flight
        (void)c.chassis.scheduler().tick();
        c.pacer.pace();
    }
    REQUIRE(c.chassis.scheduler().hasActiveMotion());

    c.chassis.drive(ChassisSpeeds{Velocity{0.0}, Velocity{10.0}, AngularVelocity{0.0}},
                    Frame::Body);
    CHECK_FALSE(c.chassis.scheduler().hasActiveMotion());     // pre-empted
    CHECK(m.state() == MotionState::Cancelled);               // old object inert
    CHECK(c.chassis.lastCompleted().exit == ExitReason::Cancelled);
    CHECK(c.chassis.scheduler().motionsCancelled() == 1);
    // The drive command LANDED (motors carry the lateral demand, not zero and
    // not the motion's forward demand): front-left of the X layout sees -vy·c.
    bool anyNonZero = false;
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        anyNonZero = anyNonZero || c.rig.h.motor(w).commandedVoltage().value() != 0.0;
    }
    CHECK(anyNonZero);
    // A stray tick of the cancelled motion must not re-command (C1 contract):
    const double v0 = c.rig.h.motor(0).commandedVoltage().value();
    CHECK(m.tick() == ExitReason::Cancelled);
    CHECK(c.rig.h.motor(0).commandedVoltage().value() == v0);
}

// ═══ The boot window ═══════════════════════════════════════════════════════════════

// Bug caught: a field-frame drive() during IMU calibration rotating by a
// garbage heading (moving the robot in a garbage direction), or the warn
// spamming every call, or body-frame driving being wrongly gated too.
TEST_CASE("C4 drive boot: FIELD commands zero + one warn per window; BODY drives") {
    FakeTelemetrySink sink;
    ImuHostileModel imu{ImuHostileConfig{}};  // 2 s calibration
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), &sink, &imu};

    // Field-frame during boot: zero volts, robot stays put, exactly one warn.
    for (int i = 0; i < 50; ++i) {
        c.chassis.drive(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}},
                        Frame::Field);
        c.pacer.pace();
    }
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        CHECK(c.rig.h.motor(w).commandedVoltage().value() == 0.0);
    }
    CHECK(posErr(c.rig.h.truePose(), Pose2d{}) < 1e-6);
    CHECK(warnCount(sink) == 1);  // once per window, not per call

    // Body-frame during the SAME boot: not gated — the driver's stick works.
    c.chassis.drive(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}},
                    Frame::Body);
    bool anyNonZero = false;
    for (int w = 0; w < c.rig.h.motorCount(); ++w) {
        anyNonZero = anyNonZero || c.rig.h.motor(w).commandedVoltage().value() != 0.0;
    }
    CHECK(anyNonZero);

    // After calibration ends, field-frame drives normally (same chassis).
    for (int i = 0; i < 220; ++i) {  // ride out the 2 s calibration
        c.chassis.drive(ChassisSpeeds{}, Frame::Body);
        c.pacer.pace();
    }
    const Pose2d before = c.rig.h.truePose();
    for (int i = 0; i < 50; ++i) {
        c.chassis.drive(ChassisSpeeds{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}},
                        Frame::Field);
        c.pacer.pace();
    }
    CHECK(posErr(c.rig.h.truePose(), before) > 5.0);  // it drives now
    CHECK(warnCount(sink) == 1);                      // and no new warn
}

// Bug caught: drive() silently dropping the health-observable tick — found as
// mutation M20, which stayed GREEN against the original suite: nothing pinned
// that IMU_LOST / BROWNOUT / OVER_TEMP stay live while a teleop loop runs on
// drive() alone (no scheduler ticking for it). A season's driver practice
// would run with fault detection dark. This case closes the hole: an IMU
// dropout mid-teleop must latch ImuLost with no motion involved.
TEST_CASE("C4 drive health: fault observables stay live in a drive()-only teleop loop") {
    ImuHostileConfig cfg;
    cfg.calibrationEnd = Time{0.0};  // live immediately; isolate the dropout
    cfg.dropoutAt = Time{0.5};
    ImuHostileModel imu{cfg};
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin, plantConfig(), nullptr, &imu};
    for (int i = 0; i < 150; ++i) {  // 1.5 s of pure teleop, through the dropout
        c.chassis.drive(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}},
                        Frame::Body);
        c.pacer.pace();
    }
    CHECK(c.rig.latch.raiseCount(shulib::diag::FaultCode::ImuLost) == 1);  // seen, once
    CHECK(c.chassis.scheduler().motionsStarted() == 0);  // no motion did the seeing
}

// ═══ Back-to-back conflicting commands ═════════════════════════════════════════════

// Bug caught: state bleeding between rapid conflicting commands — a stale
// active slot, a stuck brake mode, or drive() leaving residue that corrupts
// the next blocking verb.
TEST_CASE("C4 conflict: verb → opposite drive() → verb, back to back, stays coherent") {
    const auto kin = xDrive(Length{7.0});
    ChassisRig c{kin};
    REQUIRE(c.chassis.moveTo(Pose2d{Length{15.0}, Length{0.0}, Angle{}},
                             {.timeoutSeconds = 8.0})
            == ExitReason::Settled);
    // Immediately command the OPPOSITE direction manually for 0.3 s…
    for (int i = 0; i < 30; ++i) {
        c.chassis.drive(ChassisSpeeds{Velocity{-30.0}, Velocity{0.0}, AngularVelocity{0.0}},
                        Frame::Field);
        c.pacer.pace();
    }
    CHECK(c.rig.h.truePose().x().value() < 14.0);  // the manual verb genuinely acted
    // …then a conflicting blocking verb: it must win cleanly.
    const Pose2d target{Length{30.0}, Length{5.0}, Angle::degrees(-90.0)};
    REQUIRE(c.chassis.moveTo(target, {.timeoutSeconds = 8.0}) == ExitReason::Settled);
    CHECK(posErr(c.rig.h.truePose(), target) < 1.0);
    CHECK(headErr(c.rig.h.truePose(), target) < 0.03);
    CHECK(c.chassis.scheduler().motionsSettled() == 2);
    CHECK(c.chassis.scheduler().motionsCancelled() == 0);  // nothing was pre-empted
}
