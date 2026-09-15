// DriveEncoderOdometry, the IOdometry seam, and the ONE drive geometry (chunk R3b Part 2).
// Each case names the bug it catches and the mutation that must turn it red (Session-2
// brief §7 numbering; test 8 — "the entire existing suite unchanged" — is the suite itself):
//   8b the seam is real: a Localizer over a SCRIPTED IOdometry folds its deltas
//   9  DriveEncoderOdometry vs the A2 truth integrator: straight, arc, in-place spin (zero
//      translation), reverse — every tick                    (swap ΔL/ΔR; drop the /2; lateral ≠ 0)
//  10  asymmetric per-side scale: a 2 % right-side error produces the PREDICTED drift;
//      correct scales produce none                            (use the left scale for both sides)
//  11  the heading cross-check fires under injected slip, silent otherwise, with the
//      predicted sign and magnitude                           (cross-check computed with the wrong sign)
//  12  trust-gate parity with Pilons: oversized Δθ flagged-but-integrated; oversized side
//      travel flagged; a non-finite tick freezes position while heading advances — and
//      (better than parity) the next finite read recovers the travel   (remove either half)
//  13  the stall check and the odometry agree on implied travel from ONE geometry object,
//      symmetric and asymmetric                               (change one consumer's conversion)
//  17  the §2 ruling: a check with no independent motion source NEVER reports a stall, says so,
//      still computes its observables, and the boot note names it     (the check re-wired)

#include "doctest.h"

#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <span>
#include <string_view>

#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/tank.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/drive_encoder_odometry.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::PreconditionError;
using shulib::hal::DriveGeometry;
using shulib::hal::IMotor;
using shulib::hal::fake::FakeClock;
using shulib::hal::fake::FakeImu;
using shulib::hal::fake::FakeMotor;
using shulib::kinematics::TankKinematics;
using shulib::localization::DriveEncoderOdometry;
using shulib::localization::DriveEncoderOdometryConfig;
using shulib::localization::IOdometry;
using shulib::localization::Localizer;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::math::Pose2d;
using shulib::motion::OdoStallCheck;
using shulib::motion::OdoStallCheckConfig;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;

namespace {

constexpr double kDt = 0.01;
constexpr double kBoundIn = 1e-6;      // the perfect-sensor tracking bound (sim_odometry_truth_test)
constexpr double kTrackWidth = 12.0;   // the tank rig's, inches
constexpr double kExact = 1e-9;

[[nodiscard]] SimHarnessConfig instantConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    return cfg;
}

/// The plant's own drive geometry: its driveWheelDiameter at the 1:1 gearing it bakes in
/// (drive_plant.hpp HA-13/14; the team lead has reported no ratio other than 1:1, so the
/// plant stays at 1:1 — R3b Parts 1–3 log).
[[nodiscard]] DriveGeometry plantGeometry(const SimHarnessConfig& cfg) {
    return DriveGeometry::fromDiameter(cfg.plant.driveWheelDiameter, 1.0);
}

[[nodiscard]] double posError(const Pose2d& a, const Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// A tank script: forward, CCW arc, in-place CW spin, reverse, CW arc, brake — 1 s each.
[[nodiscard]] ChassisSpeeds tankScript(int tick) {
    switch (tick / 100) {
        case 0: return {Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};
        case 1: return {Velocity{15.0}, Velocity{0.0}, AngularVelocity{1.2}};
        case 2: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{-1.5}};
        case 3: return {Velocity{-18.0}, Velocity{0.0}, AngularVelocity{0.0}};
        case 4: return {Velocity{-9.0}, Velocity{0.0}, AngularVelocity{-0.9}};
        default: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};
    }
}

/// One wheel slips: what moves the body is `factor` × what the encoder counts.
struct OneWheelSlip final : shulib::sim::DegradationModel {
    int wheel;
    double factor;
    OneWheelSlip(int w, double f) : wheel{w}, factor{f} {}
    [[nodiscard]] shulib::units::Velocity wheelMotionVelocity(int w, shulib::units::Velocity spin,
                                                              Time, shulib::sim::Rng&) override {
        return w == wheel ? shulib::units::Velocity{spin.value() * factor} : spin;
    }
};

/// A scripted IOdometry: advances +1 in along x per update(), reports what it is told.
class ScriptedOdometry final : public IOdometry {
public:
    void update() override {
        ++updates;
        pose_ = Pose2d{Length{pose_.x().value() + 1.0}, pose_.y(), pose_.heading()};
    }
    [[nodiscard]] Pose2d pose() const noexcept override { return pose_; }
    void setPose(const Pose2d& p) override { pose_ = p; }
    [[nodiscard]] bool lastDeltaImplausible() const noexcept override { return implausible; }
    int updates = 0;
    bool implausible = false;

private:
    Pose2d pose_{};
};

}  // namespace

// ═══ 8b. The seam is what the Localizer depends on ═══════════════════════════════════
// Bug caught: a Localizer that still reaches past the interface (a cast, a concrete
// member) — a scripted odometry must be folded exactly like a real one.
TEST_CASE("IOdometry seam: the Localizer folds a scripted odometry's deltas and reads its gate") {
    FakeClock clk{Time{3.0}};
    FakeImu imu;  // ready from construction: no boot hold
    ScriptedOdometry odom;
    shulib::localization::ComplementaryFusion fusion;
    Localizer loc{clk, imu, odom, fusion};
    for (int i = 0; i < 50; ++i) {
        clk.advance(Time{kDt});
        loc.update();
    }
    CHECK(odom.updates == 50);
    CHECK(loc.pose().x().value() == doctest::Approx(50.0).epsilon(1e-12));  // 1:1 dead reckon
    CHECK_FALSE(loc.lastOdomDeltaImplausible());
    odom.implausible = true;
    clk.advance(Time{kDt});
    loc.update();
    CHECK(loc.lastOdomDeltaImplausible());   // the gate is read THROUGH the seam
    loc.setPose(Pose2d{Length{-7.0}, Length{2.0}, Angle{}});
    CHECK(odom.pose().x().value() == -7.0);  // setPose forwards through the seam
}

// ═══ 9. Against ground truth, every tick ════════════════════════════════════════════
// Bug caught: an integration or sign error — a swapped side, a dropped /2, a lateral term
// that a tank cannot have, a wrong chord — any of which walks the estimate off truth during
// the arc or the reverse segments; the spin segment pins zero translation exactly.
TEST_CASE("DriveEncoderOdometry 9: tracks the A2 truth every tick — straight, arc, spin, reverse") {
    const TankKinematics kin{Length{kTrackWidth}};
    const SimHarnessConfig cfg = instantConfig();
    const DriveGeometry g = plantGeometry(cfg);
    double worst = 0.0;
    // Tick by tick so the check is EVERY tick, not the endpoint.
    SimHarness h2{kin, cfg};
    DriveEncoderOdometry odom2{h2.imu(), h2.motor(0), h2.motor(1), g, g, Length{kTrackWidth}};
    for (int tick = 0; tick < 600; ++tick) {
        h2.commandBodyTwist(tankScript(tick));
        h2.plant().step(Time{kDt});
        odom2.update();
        const double err = posError(odom2.pose(), h2.truePose());
        worst = std::max(worst, err);
        REQUIRE(err < kBoundIn);
        REQUIRE(std::abs(odom2.pose().heading().errorTo(h2.truePose().heading())) < kExact);
        REQUIRE_FALSE(odom2.lastDeltaImplausible());
        REQUIRE(std::abs(odom2.lastHeadingDisagreement()) < kExact);  // ideal coupling
        if (tick >= 200 && tick < 300) {  // the in-place spin: ZERO translation, exactly
            REQUIRE(std::abs(odom2.lastSideTravel().left.value()
                             + odom2.lastSideTravel().right.value()) < kExact);
        }
    }
    MESSAGE("drive-encoder odometry worst error vs truth over 6 s: ", worst, " in");
    CHECK(posError(h2.truePose(), Pose2d{}) > 5.0);  // it went somewhere
}

// ═══ 10. Per-side scale — asymmetric geometry is representable and matters ══════════
// Bug caught: one scalar for both sides (the A29 shape): a right-side ratio error would be
// undetectable, and a real asymmetric drivetrain unrepresentable.
TEST_CASE("DriveEncoderOdometry 10: a 2 % right-side scale error drifts by the predicted amount; "
          "correct per-side scales do not") {
    const TankKinematics kin{Length{kTrackWidth}};
    const SimHarnessConfig cfg = instantConfig();
    const DriveGeometry g = plantGeometry(cfg);
    const DriveGeometry gRightWrong{Length{g.wheelRadius.value() * 1.02}, 1.0};

    SimHarness good{kin, cfg};
    DriveEncoderOdometry odomGood{good.imu(), good.motor(0), good.motor(1), g, g, Length{kTrackWidth}};
    SimHarness bad{kin, cfg};
    DriveEncoderOdometry odomBad{bad.imu(), bad.motor(0), bad.motor(1), g, gRightWrong, Length{kTrackWidth}};

    const ChassisSpeeds straight{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};
    for (int tick = 0; tick < 300; ++tick) {  // 3 s straight: 60 in
        good.commandBodyTwist(straight);
        good.plant().step(Time{kDt});
        odomGood.update();
        bad.commandBodyTwist(straight);
        bad.plant().step(Time{kDt});
        odomBad.update();
    }
    const double travelled = good.truePose().x().value();
    REQUIRE(travelled > 50.0);
    CHECK(posError(odomGood.pose(), good.truePose()) < kBoundIn);
    // Predicted: centre forward = (dL + 1.02·dR)/2 = 1.01 × true, so the drift is 1 %.
    const double drift = odomBad.pose().x().value() - bad.truePose().x().value();
    CHECK(drift == doctest::Approx(0.01 * travelled).epsilon(1e-6));
    CHECK(std::abs(odomBad.pose().y().value()) < 1e-3);  // heading is the IMU's: no curve
    // The asymmetry shows in the cross-check too: the wheels imply a turn the IMU denies.
    CHECK(odomBad.lastHeadingDisagreement() > 0.0);
    CHECK(std::abs(odomGood.lastHeadingDisagreement()) < kExact);
}

// ═══ 11. The heading cross-check ════════════════════════════════════════════════════
// Bug caught: a slipping side going unnoticed — the observable must be silent under clean
// rolling (arcs included) and fire with the predicted SIGN and magnitude under slip.
TEST_CASE("DriveEncoderOdometry 11: the heading cross-check fires under injected slip with the "
          "predicted sign and magnitude, and is silent otherwise") {
    const TankKinematics kin{Length{kTrackWidth}};
    const SimHarnessConfig cfg = instantConfig();
    const DriveGeometry g = plantGeometry(cfg);
    const ChassisSpeeds straight{Velocity{20.0}, Velocity{0.0}, AngularVelocity{0.0}};

    // Right wheel (index 1) slips: the body gets 90 % of its spin.
    OneWheelSlip slip{1, 0.9};
    SimHarness h{kin, cfg, nullptr, &slip};
    DriveEncoderOdometry odom{h.imu(), h.motor(0), h.motor(1), g, g, Length{kTrackWidth}};
    for (int tick = 0; tick < 100; ++tick) {
        h.commandBodyTwist(straight);
        h.plant().step(Time{kDt});
        odom.update();
    }
    // The encoders say straight (Δθ_enc = 0); the IMU says the robot turned CW (the right
    // side moved less): Δθ_imu = (0.9·v − v)·dt / tw < 0. Disagreement = 0 − Δθ_imu > 0.
    const double predicted = 0.1 * 20.0 * kDt / kTrackWidth;
    CHECK(odom.lastHeadingDisagreement() == doctest::Approx(predicted).epsilon(1e-6));
    CHECK(odom.lastHeadingDisagreement() > 0.0);
    // And the odometry is now WRONG by exactly what the slip stole — which is the point:
    // the cross-check is the only thing that saw it.
    CHECK(posError(odom.pose(), h.truePose()) > 0.5);

    // Left wheel slipping instead: the sign flips.
    OneWheelSlip slipL{0, 0.9};
    SimHarness hl{kin, cfg, nullptr, &slipL};
    DriveEncoderOdometry odomL{hl.imu(), hl.motor(0), hl.motor(1), g, g, Length{kTrackWidth}};
    for (int tick = 0; tick < 100; ++tick) {
        hl.commandBodyTwist(straight);
        hl.plant().step(Time{kDt});
        odomL.update();
    }
    CHECK(odomL.lastHeadingDisagreement() == doctest::Approx(-predicted).epsilon(1e-6));

    // Silent under clean rolling through the whole script (arcs and spins included).
    SimHarness clean{kin, cfg};
    DriveEncoderOdometry odomC{clean.imu(), clean.motor(0), clean.motor(1), g, g, Length{kTrackWidth}};
    for (int tick = 0; tick < 600; ++tick) {
        clean.commandBodyTwist(tankScript(tick));
        clean.plant().step(Time{kDt});
        odomC.update();
        REQUIRE(std::abs(odomC.lastHeadingDisagreement()) < kExact);
    }
}

// ═══ 12. Trust-gate parity with Pilons ══════════════════════════════════════════════
// Bug caught: a gate half not carried over — an oversized heading step passing as
// trustworthy, a phantom side travel passing, or a non-finite tick poisoning the pose.
TEST_CASE("DriveEncoderOdometry 12: trust gate — oversized dtheta flagged but integrated; oversized "
          "side travel flagged; a non-finite tick freezes position and advances heading") {
    FakeImu imu;
    FakeMotor left, right;
    const DriveGeometry g{Length{1.0}, 1.0};  // 1 in per rad: radians are inches
    DriveEncoderOdometry odom{imu, left, right, g, g, Length{10.0}};

    // A plausible tick is not flagged (heading held at 0: a straight inch is exactly +1 x).
    left.setPosition(AngleDim{1.0});
    right.setPosition(AngleDim{1.0});
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK(odom.pose().x().value() == 1.0);

    // Rotation half: 120° in one tick exceeds the π/2 default — flagged, STILL integrated.
    left.setPosition(AngleDim{2.0});
    right.setPosition(AngleDim{2.0});
    imu.setHeading(Angle::degrees(121.0));
    odom.update();
    CHECK(odom.lastDeltaImplausible());
    CHECK(posError(odom.pose(), Pose2d{Length{1.0}, Length{0.0}, Angle{}}) > 0.5);  // moved
    CHECK(odom.pose().heading().radians() == doctest::Approx(Angle::degrees(121.0).radians()));

    // Travel half: one side jumps 1000 rad (a late-enumerating port) — flagged, integrated.
    left.setPosition(AngleDim{1002.0});
    odom.update();
    CHECK(odom.lastDeltaImplausible());
    // Exactly at the bound is not flagged (36 in on one side).
    left.setPosition(AngleDim{1002.0 + 36.0});
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());

    // Non-finite half: a NaN position freezes POSITION, heading still advances, flagged.
    const Pose2d before = odom.pose();
    left.setPosition(AngleDim{std::numeric_limits<double>::quiet_NaN()});
    imu.setHeading(Angle::degrees(130.0));
    odom.update();
    CHECK(odom.lastDeltaImplausible());
    CHECK(odom.pose().x().value() == before.x().value());
    CHECK(odom.pose().y().value() == before.y().value());
    CHECK(odom.pose().heading().radians() == doctest::Approx(Angle::degrees(130.0).radians()));
    // Better than parity: the next FINITE read recovers — the baseline was not poisoned,
    // so the travel across the bad tick is integrated on the next one, not lost.
    const double lastGoodLeft = 1002.0 + 36.0;
    left.setPosition(AngleDim{lastGoodLeft + 2.0});
    right.setPosition(AngleDim{2.0 + 2.0});
    imu.setHeading(Angle::degrees(130.0));
    odom.update();
    CHECK_FALSE(odom.lastDeltaImplausible());
    CHECK(odom.lastSideTravel().left.value() == doctest::Approx(2.0).epsilon(1e-9));
    CHECK(odom.lastSideTravel().right.value() == doctest::Approx(2.0).epsilon(1e-9));
    CHECK(std::isfinite(odom.pose().x().value()));

    // Construction baselines: a pre-existing shaft total is not travel.
    FakeMotor l2, r2;
    l2.setPosition(AngleDim{500.0});
    r2.setPosition(AngleDim{500.0});
    FakeImu imu2;
    DriveEncoderOdometry fresh{imu2, l2, r2, g, g, Length{10.0}};
    fresh.update();
    CHECK(posError(fresh.pose(), Pose2d{}) < kExact);
    // setPose teleports position only; the heading stays the IMU's; no phantom rotation.
    imu2.setHeading(Angle::degrees(45.0));
    fresh.setPose(Pose2d{Length{3.0}, Length{4.0}, Angle::degrees(-90.0)});
    CHECK(fresh.pose().heading().radians() == doctest::Approx(Angle::degrees(45.0).radians()));
    fresh.update();
    CHECK_FALSE(fresh.lastDeltaImplausible());
    CHECK(posError(fresh.pose(), Pose2d{Length{3.0}, Length{4.0}, Angle{}}) < kExact);

    // Refusals: UNSET geometry, bad track width, bad gate knobs.
    const DriveGeometry unset{Length{0.0}, 0.0};
    CHECK_THROWS_AS((DriveEncoderOdometry{imu, left, right, unset, g, Length{10.0}}),
                    PreconditionError);
    CHECK_THROWS_AS((DriveEncoderOdometry{imu, left, right, g, unset, Length{10.0}}),
                    PreconditionError);
    CHECK_THROWS_AS((DriveEncoderOdometry{imu, left, right, g, g, Length{0.0}}),
                    PreconditionError);
    DriveEncoderOdometryConfig badCfg;
    badCfg.maxTickTravel = Length{0.0};
    CHECK_THROWS_AS((DriveEncoderOdometry{imu, left, right, g, g, Length{10.0}, {}, badCfg}),
                    PreconditionError);
}

// ═══ 13. One geometry, two consumers ═════════════════════════════════════════════════
// Bug caught: two sources of truth for the ratio — the stall check converting shaft
// radians with a different scale than the odometry for the SAME encoder delta.
TEST_CASE("DriveEncoderOdometry 13: the stall check and the odometry agree on implied travel from "
          "ONE geometry object, symmetric and asymmetric") {
    struct Case {
        DriveGeometry left;
        DriveGeometry right;
    };
    const Case cases[] = {
        {DriveGeometry::fromDiameter(Length{2.75}), DriveGeometry::fromDiameter(Length{2.75})},
        {DriveGeometry::fromDiameter(Length{2.75}, 0.6), DriveGeometry::fromDiameter(Length{2.75}, 0.6)},
        {DriveGeometry::fromDiameter(Length{2.75}, 0.6), DriveGeometry::fromDiameter(Length{3.25}, 0.5)},
    };
    for (const Case& c : cases) {
        CAPTURE(c.left.motorToWheelRatio);
        CAPTURE(c.right.wheelRadius.value());
        FakeImu imu;
        FakeMotor left, right;
        DriveEncoderOdometry odom{imu, left, right, c.left, c.right, Length{12.0}};
        OdoStallCheckConfig sc;
        sc.wheels[0] = c.left;   // the SAME objects
        sc.wheels[1] = c.right;
        OdoStallCheck check{sc};
        std::array<IMotor*, 2> motors{&left, &right};
        const Pose2d still{};
        (void)check.update(Time{0.0}, motors, still);  // baseline
        const double dRad = 7.5;
        left.setPosition(AngleDim{dRad});
        right.setPosition(AngleDim{dRad});
        odom.update();
        (void)check.update(Time{0.31}, motors, still);  // window closed
        const double odomForward = 0.5 * (odom.lastSideTravel().left.value()
                                          + odom.lastSideTravel().right.value());
        CHECK(check.lastSpinTravel().value() == odomForward);  // bits, not Approx
        CHECK(odom.lastSideTravel().left.value() == dRad * c.left.inchesPerRadian().value());
        CHECK(odom.lastSideTravel().right.value() == dRad * c.right.inchesPerRadian().value());
        CHECK(odom.leftInchesPerRadian().value() == c.left.inchesPerRadian().value());
        CHECK(odom.rightInchesPerRadian().value() == c.right.inchesPerRadian().value());
    }
    // The geometry type itself: diameter → radius; ratio multiplies; UNSET is invalid.
    const DriveGeometry g = DriveGeometry::fromDiameter(Length{2.75}, 0.6);
    CHECK(g.wheelRadius.value() == 1.375);
    CHECK(g.inchesPerRadian().value() == 1.375 * 0.6);
    CHECK(g.valid());
    CHECK_FALSE(DriveGeometry{Length{0.0}, 1.0}.valid());
    CHECK_FALSE(DriveGeometry{Length{1.0}, 0.0}.valid());
    CHECK_FALSE(DriveGeometry{Length{-1.0}, 1.0}.valid());
    CHECK_FALSE(DriveGeometry{Length{std::numeric_limits<double>::infinity()}, 1.0}.valid());
    // The stall check refuses an UNSET slot, and setAllWheels fills every slot.
    OdoStallCheckConfig bad;
    bad.wheels[3] = DriveGeometry{Length{0.0}, 1.0};
    CHECK_THROWS_AS((OdoStallCheck{bad}), PreconditionError);
    OdoStallCheckConfig sym;
    sym.setAllWheels(g);
    for (const DriveGeometry& w : sym.wheels) {
        CHECK(w.inchesPerRadian().value() == g.inchesPerRadian().value());
    }
    CHECK_NOTHROW((OdoStallCheck{sym}));
}

// ═══ 17. The §2 ruling: no independent motion source, no verdict ════════════════════
// Bug caught: the check left wired on a drive-encoder robot — reporting "not stalled"
// forever from a tautology, so the health monitor lies. The same feed that trips a wired
// check must never trip an unwired one, and the unwired one must SAY it cannot see.
TEST_CASE("OdoStallCheck 17: with no independent motion source the check never reports a stall, "
          "says so, keeps its observables, and the boot note names the fact") {
    FakeMotor left, right;
    std::array<IMotor*, 2> motors{&left, &right};
    const Pose2d still{};
    auto feedStall = [&](OdoStallCheck& check) {  // wheels spin 10 rad, pose never moves
        left.setPosition(AngleDim{0.0});
        right.setPosition(AngleDim{0.0});
        (void)check.update(Time{0.0}, motors, still);
        left.setPosition(AngleDim{10.0});
        right.setPosition(AngleDim{10.0});
        return check.update(Time{0.31}, motors, still);
    };
    // The CONTROL: a wired check trips on this feed (so the case below proves something).
    OdoStallCheckConfig wired;
    OdoStallCheck control{wired};
    CHECK(feedStall(control));
    CHECK(control.stalled());
    CHECK(control.canDetectStall());

    // The ruling: the same feed, no independent source — never a verdict, observables intact.
    OdoStallCheckConfig drive;
    drive.independentMotionSource = false;
    OdoStallCheck unwired{drive};
    CHECK_FALSE(unwired.canDetectStall());
    CHECK_FALSE(feedStall(unwired));
    CHECK_FALSE(unwired.stalled());
    CHECK(unwired.lastSpinTravel().value() == control.lastSpinTravel().value());  // still computed
    CHECK(unwired.lastSpinTravel().value() == 10.0 * (3.25 / 2.0));
    CHECK(unwired.lastObservedMotion().value() == 0.0);
    // Repeated windows: still never.
    for (int w = 2; w < 20; ++w) {
        left.setPosition(AngleDim{10.0 * w});
        right.setPosition(AngleDim{10.0 * w});
        REQUIRE_FALSE(unwired.update(Time{0.31 * w}, motors, still));
    }

    // Through the health monitor: the unwired verdict raises nothing; the wired one raises
    // ODO_STUCK — the difference the ruling is about.
    shulib::hal::fake::FakeTelemetrySink sink;
    FakeClock clk;
    shulib::diag::FaultLatch latch{sink, clk};
    shulib::diag::HealthMonitor health{latch};
    health.tick({.odomStalled = unwired.stalled()});
    CHECK(latch.raiseCount(shulib::diag::FaultCode::OdoStuck) == 0);
    health.tick({.odomStalled = control.stalled()});
    CHECK(latch.raiseCount(shulib::diag::FaultCode::OdoStuck) == 1);

    // The boot note exists, and says the thing the composition root must say once.
    const std::string_view note{shulib::motion::kNoIndependentStallSourceNote};
    CHECK(note.find("NO INDEPENDENT") != std::string_view::npos);
    CHECK(note.find("drive encoders") != std::string_view::npos);
    CHECK(note.find("heading cross-check") != std::string_view::npos);
}
