// THE A3 SURVIVAL MATRIX — every pathology, attacked through the REAL stack
// (PilonsOdometry + Localizer + ComplementaryFusion + HealthMonitor + LoopMonitor,
// reading only the F4 fakes), graded against ground truth. The contract each case
// enforces is the brief's non-negotiable: a FAULT CODE is raised, the pose stays
// FINITE every tick, the damage is BOUNDED, and the run CONTINUES — never a crash,
// never a NaN in the pose, never a silent wrong answer.
//
// The GPS cases drive the fusion screening through a TEST-LOCAL naive corrector
// (reads FakeGps, proposes every fix at face value). It is a harness probe in the
// spirit of A2's TrapSink — deliberately gullible so the LOCALIZER's screening is
// what gets graded — and is NOT E2's GpsCorrector (adaptive R, lever arm, latency
// compensation, yaw-rate rejection land there).

#include "doctest.h"

#include <cmath>
#include <vector>

#include "shulib/control/pid.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/diag/loop_monitor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/hostile/composed.hpp"
#include "shulib/sim/scenario.hpp"
#include "shulib/units/quantity.hpp"

using shulib::control::Pid;
using shulib::control::PidConfig;
using shulib::diag::FaultCode;
using shulib::diag::FaultLatch;
using shulib::diag::HealthMonitor;
using shulib::diag::isFinitePose;
using shulib::diag::LoopMonitor;
using shulib::diag::LoopMonitorConfig;
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::xDrive;
using shulib::localization::ComplementaryFusion;
using shulib::localization::CorrectionProposal;
using shulib::localization::ICorrector;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::math::ChassisSpeeds;
using shulib::sim::EncoderHostileConfig;
using shulib::sim::EncoderHostileModel;
using shulib::sim::GpsBadFixWindow;
using shulib::sim::GpsHostileConfig;
using shulib::sim::GpsHostileModel;
using shulib::sim::ImuHostileConfig;
using shulib::sim::ImuHostileModel;
using shulib::sim::JitterSchedule;
using shulib::sim::JitterScheduleConfig;
using shulib::sim::PowerHostileConfig;
using shulib::sim::PowerHostileModel;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
[[nodiscard]] SimHarnessConfig instantConfig() {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    return cfg;
}

[[nodiscard]] double posErr(const shulib::math::Pose2d& a, const shulib::math::Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// The fault-discipline rig every case wires: A1's latch + A3's monitor, on the
/// harness clock so fault timestamps are run time.
struct Watch {
    FakeTelemetrySink sink;
    FaultLatch latch;
    HealthMonitor monitor;
    explicit Watch(shulib::hal::IClock& clock) : latch{sink, clock}, monitor{latch} {}
};

/// The deliberately gullible GPS corrector (file header note).
class NaiveGpsCorrector final : public ICorrector {
public:
    explicit NaiveGpsCorrector(shulib::hal::IGps& gps) : gps_{gps} {}
    [[nodiscard]] CorrectionProposal propose(const shulib::math::Pose2d&, Time) override {
        if (!gps_.hasFix()) {
            return {};  // no fix → no proposal (never a zero-confidence pull)
        }
        CorrectionProposal p;
        p.valid = true;
        p.fieldPose = gps_.pose();
        p.confidence = 0.6;
        p.positionStdDev = gps_.rmsError();
        return p;
    }
    [[nodiscard]] const char* name() const noexcept override { return "test-gps"; }

private:
    shulib::hal::IGps& gps_;
};
}  // namespace

// ── ATTACK 1: the boot calibration window. A stationary robot, a garbage IMU.
// The offset-correction math turns garbage heading swings into phantom
// translation, so the Localizer must NOT integrate odometry deltas before the IMU
// has ever been ready. (This attack found the boot-poisoning defect — see
// A3-PROGRESS / A3-COMPLETED for the observed pre-fix magnitude.) ──
TEST_CASE("survival: IMU calibration garbage cannot poison the fused pose (boot guard)") {
    ImuHostileConfig imuCfg;  // defaults: 2 s window, drift, noise
    ImuHostileModel imu{imuCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &imu};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    Watch w{h.clock()};

    // Phase 1: STATIONARY through the calibration window (the realistic boot).
    // Monitor wiring note (the C1 loop shape): odometry health is only OBSERVED
    // once the IMU is ready — before that the whole estimator is un-trusted by
    // quality 0, and boot garbage tripping ODO_STUCK would bury real root causes.
    double fusedDriftDuringBoot = 0.0;
    double rawOdomDriftDuringBoot = 0.0;
    h.runTicks(200, Time{0.01}, [&](int) {
        loc.update();
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .odomImplausible = h.imu().isReady() && odom.lastDeltaImplausible(),
                        .batteryVolts = h.battery().voltage()});
        REQUIRE(isFinitePose(loc.pose()));
        fusedDriftDuringBoot = std::max(fusedDriftDuringBoot, posErr(loc.pose(), h.truePose()));
        rawOdomDriftDuringBoot = std::max(rawOdomDriftDuringBoot, posErr(odom.pose(), h.truePose()));
        CHECK(loc.qualityClass() == Localizer::Quality::Uninitialized);
        CHECK(loc.quality() == 0.0);
    });

    // The attack was REAL: the raw odometry (which trusts its inputs by contract —
    // gating on readiness is the policy layer's job) got dragged by the garbage…
    CHECK(rawOdomDriftDuringBoot > 1.0);
    // …but the published fused estimate held its ground.
    CHECK(fusedDriftDuringBoot < 0.05);
    // Boot is NORMAL: never-ready must not have raised ImuLost.
    CHECK_FALSE(w.latch.hasFault());

    // Phase 2: calibration ends; the stack runs a real script and stays sane.
    // The C1 contract the settle window imposes (localizer.hpp header): motion
    // before the estimate is LIVE is unaccounted — so the loop, like a real auton,
    // holds still until qualityClass leaves Uninitialized, then drives.
    double worstErr = 0.0;
    bool live = false;
    int liveTicks = 0;
    h.runTicks(600, Time{0.01}, [&](int) {
        loc.update();
        REQUIRE(isFinitePose(loc.pose()));
        live = live || loc.qualityClass() != Localizer::Quality::Uninitialized;
        if (live) {
            ++liveTicks;
            worstErr = std::max(worstErr, posErr(loc.pose(), h.truePose()));
            const double vx = (liveTicks < 300) ? 15.0 : -10.0;
            h.commandBodyTwist(ChassisSpeeds{Velocity{vx}, Velocity{2.0}, AngularVelocity{0.4}});
        }
    });
    CHECK(live);                    // the estimate came alive after settling
    CHECK(liveTicks > 550);         // and the settle hold was ~0.1 s, not forever
    // Bounded by drift/noise-induced heading error over ~6 s of travel — inches
    // would mean the boot garbage leaked through after all.
    CHECK(worstErr < 1.0);
}

// ── ATTACK 2: mid-run IMU disconnect. ImuLost must be raised (once), the quality
// must say Degraded — a robot that WAS localized and lost its heading authority is
// not "uninitialized", and a skills gate must be able to tell the difference —
// and the estimate must stay finite with bounded damage. ──
TEST_CASE("survival: mid-run IMU dropout raises IMU_LOST, degrades quality, stays finite") {
    ImuHostileConfig imuCfg;
    imuCfg.calibrationEnd = Time{0.0};  // isolate the dropout
    imuCfg.rateBiasMax = AngularVelocity{0.0};
    imuCfg.headingNoiseSigmaRad = 0.0;
    imuCfg.yawRateNoiseSigmaRadPerS = 0.0;
    imuCfg.dropoutAt = Time{3.0};
    ImuHostileModel imu{imuCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &imu};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    Watch w{h.clock()};

    double travelAfterDrop = 0.0;
    shulib::math::Pose2d truthAtDrop{};
    h.runTicks(600, Time{0.01}, [&](int tick) {
        loc.update();
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .odomImplausible = odom.lastDeltaImplausible(),
                        .batteryVolts = h.battery().voltage()});
        REQUIRE(isFinitePose(loc.pose()));
        if (tick == 300) {
            truthAtDrop = h.truePose();
        }
        // healthy 3 s: gentle arc; after the drop: keep turning + driving, so the
        // frozen heading genuinely costs accuracy (a straight run would hide it)
        h.commandBodyTwist(ChassisSpeeds{Velocity{12.0}, Velocity{0.0}, AngularVelocity{0.5}});
    });

    CHECK(w.latch.firstFault() == FaultCode::ImuLost);  // raised, and the ROOT cause
    CHECK(w.monitor.imuLost());
    CHECK(loc.qualityClass() == Localizer::Quality::Degraded);  // NOT Uninitialized
    CHECK(loc.quality() == 0.0);                                // zero trust, honestly

    // Bounded damage: with a frozen heading the estimate can be at most 2×travel
    // wrong (opposite direction), and the 90°-ish of missed rotation must have
    // cost REAL error — a "survival" with no cost would mean the attack was dead.
    travelAfterDrop = posErr(h.truePose(), truthAtDrop);  // chord, understates path
    const double err = posErr(loc.pose(), h.truePose());
    CHECK(err > 0.5);
    CHECK(err < 2.0 * (12.0 * 3.0));  // ≤ 2 × the post-drop path length, structurally
    CHECK(std::isfinite(travelAfterDrop));
}

// ── ATTACK 3: the sentinel breach — PROS_ERR_F (+∞) leaks from a tracking
// encoder for 50 ms. PilonsOdometry's last-resort finite guard must freeze the
// position (never a NaN in the pose), flag the ticks, recover afterward; the
// monitor turns the flag into ODO_STUCK. ──
TEST_CASE("survival: a +inf sentinel breach freezes, flags, faults ODO_STUCK, and recovers") {
    EncoderHostileConfig encCfg;  // quantization on; breach on tracking wheel 0
    encCfg.sentinelAt = Time{2.0};
    encCfg.sentinelFor = Time{0.05};
    EncoderHostileModel enc{encCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &enc};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    Watch w{h.clock()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{15.0}, Velocity{0.0}, AngularVelocity{0.0}});
    bool sawImplausible = false;
    double errAt3s = 0.0;
    double errAt6s = 0.0;
    h.runTicks(600, Time{0.01}, [&](int tick) {
        loc.update();
        sawImplausible = sawImplausible || odom.lastDeltaImplausible();
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .odomImplausible = odom.lastDeltaImplausible(),
                        .batteryVolts = h.battery().voltage()});
        REQUIRE(isFinitePose(loc.pose()));   // the one absolute guarantee
        REQUIRE(isFinitePose(odom.pose()));
        if (tick == 300) {
            errAt3s = posErr(loc.pose(), h.truePose());
        }
        if (tick == 599) {
            errAt6s = posErr(loc.pose(), h.truePose());
        }
    });

    CHECK(sawImplausible);                              // the guard saw the breach
    CHECK(w.latch.firstFault() == FaultCode::OdoStuck);  // and it became a FAULT
    // Bounded damage: position froze for ~7 ticks (5 breach + 2 poisoned deltas)
    // at 15 in/s ⇒ ~1.05 in lost, permanently (dead-reckoning cannot refund it) —
    // and NOT growing afterward.
    CHECK(errAt3s > 0.5);
    CHECK(errAt3s < 2.0);
    CHECK(errAt6s == doctest::Approx(errAt3s).epsilon(0.05));
}

// ── ATTACK 4: a tracking encoder freezes mid-run (disconnect). RECORDED FINDING:
// this pathology is INVISIBLE to the M2 estimator — zero travel is a plausible
// reading, so lastDeltaImplausible() stays false and the estimate walks away from
// truth with no estimator-side alarm. Containment is the LOOP's cross-check
// (wheels commanded+spinning vs no reported motion → odomStalled → ODO_STUCK),
// exercised here exactly as C1 will own it; the estimator-side detector is
// E-phase work (fault.hpp assigns OdoStuck to the C/E layers). ──
TEST_CASE("survival: a frozen tracking encoder is caught by the loop cross-check, not the estimator") {
    EncoderHostileConfig encCfg;
    encCfg.trackingFreezeAt = Time{2.0};
    encCfg.trackingFreezeIndex = 0;  // the forward wheel
    EncoderHostileModel enc{encCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &enc};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    Watch w{h.clock()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{15.0}, Velocity{0.0}, AngularVelocity{0.0}});
    bool estimatorEverFlagged = false;
    double lastFusedX = 0.0;
    double driveShaftAtWindowStart = 0.0;
    h.runTicks(500, Time{0.01}, [&](int tick) {
        loc.update();
        REQUIRE(isFinitePose(loc.pose()));
        estimatorEverFlagged = estimatorEverFlagged || odom.lastDeltaImplausible();

        // The C1-shaped cross-check, over a 30-tick window: the drive encoders say
        // the wheels are rolling, the estimate says the robot is not moving.
        bool stalled = false;
        if (tick % 30 == 0) {
            const double driveShaft = h.motor(2).position().value();
            const double driveTravel = (driveShaft - driveShaftAtWindowStart) * (3.25 / 2.0);
            const double fusedTravel = std::abs(loc.pose().x().value() - lastFusedX);
            stalled = tick > 0 && driveTravel > 1.0 && fusedTravel < 0.05;
            driveShaftAtWindowStart = driveShaft;
            lastFusedX = loc.pose().x().value();
        }
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .odomImplausible = odom.lastDeltaImplausible(),
                        .odomStalled = stalled,
                        .batteryVolts = h.battery().voltage()});
    });

    CHECK_FALSE(estimatorEverFlagged);                   // the honest gap, recorded
    CHECK(w.latch.firstFault() == FaultCode::OdoStuck);  // the cross-check caught it
    // The error is exactly the truth's travel since the freeze (the estimate
    // stopped; the robot did not) — undetected ≠ unbounded-mystery: it is
    // precisely characterized, which is what makes the E-phase detector testable.
    const double err = posErr(loc.pose(), h.truePose());
    const double truthTravelSinceFreeze = h.truePose().x().value() - 15.0 * 2.0;
    CHECK(err == doctest::Approx(truthTravelSinceFreeze).epsilon(0.01));
}

// ── ATTACK 5: GPS off-strip (Driving Skills). No fix ever exists; the naive
// corrector must never fire; the fused output must be BIT-EQUAL to a twin run
// with no corrector at all — "no garbage trusted" as an equality, not a bound. ──
TEST_CASE("survival: GPS off-strip degrades to exact dead-reckoning (no garbage trusted)") {
    const auto kin = xDrive(Length{7.0});

    GpsHostileConfig gpsCfg;
    gpsCfg.offStrip = true;
    gpsCfg.noiseSigmaIn = 0.0;
    gpsCfg.headingNoiseSigmaRad = 0.0;

    auto runOne = [&](bool withCorrector) {
        GpsHostileModel gps{gpsCfg};
        SimHarness h{kin, instantConfig(), nullptr, &gps};
        PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
        ComplementaryFusion fusion{};
        NaiveGpsCorrector corrector{h.gps()};
        std::vector<ICorrector*> list;
        if (withCorrector) {
            list.push_back(&corrector);
        }
        Localizer loc{h.clock(), h.imu(), odom, fusion,
                      std::span<ICorrector* const>{list.data(), list.size()}};
        bool alwaysDeadReckon = true;
        h.runTicks(500, Time{0.01}, [&](int tick) {
            loc.update();
            alwaysDeadReckon = alwaysDeadReckon && loc.isDeadReckoning();
            const double vx = (tick < 250) ? 18.0 : -9.0;
            h.commandBodyTwist(ChassisSpeeds{Velocity{vx}, Velocity{4.0}, AngularVelocity{0.6}});
        });
        CHECK(alwaysDeadReckon);
        return loc.pose();
    };

    const auto with = runOne(true);
    const auto without = runOne(false);
    CHECK(with.x().value() == without.x().value());  // EXACT equality — bit-for-bit
    CHECK(with.y().value() == without.y().value());
}

// ── ATTACK 6a: an out-of-gate GPS lie (30 in off, claiming confidence). The
// innovation gate must reject every fix, the monitor must raise GPS_GATE_REJECT,
// and the fused pose must equal exact dead-reckoning. ──
TEST_CASE("survival: an out-of-gate GPS lie is rejected wholesale and faulted") {
    GpsHostileConfig gpsCfg;
    gpsCfg.noiseSigmaIn = 0.0;
    gpsCfg.headingNoiseSigmaRad = 0.0;
    gpsCfg.updatePeriod = Time{0.0};
    gpsCfg.badFixWindows = {GpsBadFixWindow{Time{1.0}, Time{4.0}, Length{30.0}, Length{0.0}}};
    GpsHostileModel gps{gpsCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &gps};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    NaiveGpsCorrector corrector{h.gps()};
    std::array<ICorrector*, 1> list{&corrector};
    Localizer loc{h.clock(), h.imu(), odom, fusion, list};
    Watch w{h.clock()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}});
    double worstErr = 0.0;
    bool sawGated = false;
    h.runTicks(500, Time{0.01}, [&](int) {
        loc.update();
        REQUIRE(isFinitePose(loc.pose()));
        sawGated = sawGated || loc.lastCorrection().gated;
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .odomImplausible = odom.lastDeltaImplausible(),
                        .fixGated = loc.lastCorrection().gated,
                        .batteryVolts = h.battery().voltage()});
        worstErr = std::max(worstErr, posErr(loc.pose(), h.truePose()));
    });

    CHECK(sawGated);
    CHECK(w.latch.firstFault() == FaultCode::GpsGateReject);
    // In-gate honest fixes (before 1 s and after 4 s) may nudge within noise-free
    // exactness; the 30-inch lie must never move the pose measurably.
    CHECK(worstErr < 0.01);
}

// ── ATTACK 6b: an IN-gate GPS lie (8 in — plausible). The nudge gets pulled, but
// the damage is BOUNDED by the lie/gate geometry, and honest fixes heal it. ──
TEST_CASE("survival: an in-gate GPS lie does bounded damage and honest fixes heal it") {
    GpsHostileConfig gpsCfg;
    gpsCfg.noiseSigmaIn = 0.0;
    gpsCfg.headingNoiseSigmaRad = 0.0;
    gpsCfg.updatePeriod = Time{0.0};
    gpsCfg.badFixWindows = {GpsBadFixWindow{Time{1.0}, Time{3.0}, Length{8.0}, Length{0.0}}};
    GpsHostileModel gps{gpsCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &gps};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    NaiveGpsCorrector corrector{h.gps()};
    std::array<ICorrector*, 1> list{&corrector};
    Localizer loc{h.clock(), h.imu(), odom, fusion, list};

    double worstErr = 0.0;
    double errAtLieEnd = 0.0;
    h.runTicks(800, Time{0.01}, [&](int tick) {
        loc.update();
        REQUIRE(isFinitePose(loc.pose()));
        worstErr = std::max(worstErr, posErr(loc.pose(), h.truePose()));
        if (tick == 300) {
            errAtLieEnd = posErr(loc.pose(), h.truePose());
        }
    });  // stationary: isolates the corrector's pull

    CHECK(errAtLieEnd > 4.0);   // the lie genuinely dragged the estimate…
    CHECK(worstErr < 8.5);      // …but never past the lie itself (< gate by design)
    const double errAtEnd = posErr(loc.pose(), h.truePose());
    CHECK(errAtEnd < 0.5);      // honest fixes pulled it back — the wound heals
}

// ── ATTACK 7: brownout. A weak pack + full-throttle load collapses below the
// cutoff: motors get zero volts (the fallback), BROWNOUT is raised and LATCHED,
// and the run keeps going — the exact precondition for F2's guaranteed park. ──
TEST_CASE("survival: brownout collapse raises BROWNOUT, kills drive, and the run continues") {
    PowerHostileConfig pwrCfg;  // provisional defaults; the pack itself is the weakness
    PowerHostileModel power{pwrCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig cfg = instantConfig();
    cfg.plant.batteryVoltage = Voltage{11.2};  // a nearly-dead pack on the cart
    SimHarness h{kin, cfg, nullptr, &power};
    Watch w{h.clock()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{100.0}, Velocity{0.0}, AngularVelocity{0.0}});
    bool sawZeroVelocityWhileCommanded = false;
    h.runTicks(300, Time{0.01}, [&](int) {
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .batteryVolts = h.battery().voltage()});
        sawZeroVelocityWhileCommanded =
            sawZeroVelocityWhileCommanded
            || (std::abs(h.trueBodyTwist().vx().value()) < 1e-9
                && h.motor(2).commandedVoltage().value() > 11.0);
    });

    CHECK(w.latch.firstFault() == FaultCode::Brownout);
    CHECK(w.monitor.brownedOut());                       // the latched E1-style marker
    CHECK(h.battery().voltage().value() < 10.5);         // the collapse is visible
    CHECK(sawZeroVelocityWhileCommanded);                // motors dead, commands ignored
    CHECK(h.clock().now().value() == doctest::Approx(3.0));  // the run COMPLETED
}

// ── ATTACK 8: loop jitter. The hostile dt schedule's spikes must each be caught
// by LoopMonitor (exact count — determinism makes it countable), LOOP_OVERRUN
// must latch, and a Pid position loop must converge and hold through the jitter. ──
TEST_CASE("survival: jittered timing fires LOOP_OVERRUN per spike; the controller rides it out") {
    const JitterScheduleConfig jitterCfg{};  // nominal 10 ms, ±20%, 2% spikes ×5
    constexpr std::uint64_t kSeed = 77;
    constexpr int kTicks = 600;

    // The monitor observes dt_i at tick i+1, so the last draw is never judged.
    JitterSchedule preview{kSeed, jitterCfg};
    int expectedOverruns = 0;
    for (int i = 0; i < kTicks - 1; ++i) {
        if (preview(i).value() >= 0.015) {
            ++expectedOverruns;
        }
    }
    REQUIRE(expectedOverruns > 0);  // the seed must actually spike (77 does)

    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.051};  // lag: jitter has teeth
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, cfg};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    Pid pid{PidConfig{.kP = 3.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};
    Watch w{h.clock()};
    LoopMonitor loopMon{h.clock(), w.latch, LoopMonitorConfig{.budget = Time{0.015}}};

    JitterSchedule schedule{kSeed, jitterCfg};
    const double target = 24.0;
    h.runTicksVariable(
        kTicks, [&](int i) { return schedule(i); },
        [&](int) {
            (void)loopMon.tick();
            odom.update();
            const double cmd = pid.update(target, odom.pose().x().value());
            h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        });

    CHECK(loopMon.overrunCount() == expectedOverruns);        // every spike, exactly
    CHECK(w.latch.firstFault() == FaultCode::LoopOverrun);    // and it latched
    CHECK(loopMon.worstDt().value() == doctest::Approx(0.05));  // the 5× stall was seen
    CHECK(std::abs(h.truePose().x().value() - target) < 0.1);   // still converged
}

// ── ATTACK 9: thermal droop. Accelerated heating crosses the 55 °C step mid-run:
// MOTOR_OVER_TEMP must be raised (temperatures wired the way C1 will wire them),
// and the robot must demonstrably slow while still completing the run. ──
TEST_CASE("survival: thermal droop raises MOTOR_OVER_TEMP and visibly slows the drive") {
    PowerHostileConfig pwrCfg;
    pwrCfg.sagPerCommandedVolt = 0.0;  // isolate thermal
    pwrCfg.dischargeRatePerS = 0.0;
    pwrCfg.heatRatePerV2 = 0.02;       // accelerated bench thermal mass
    pwrCfg.coolRatePerS = 0.02;
    PowerHostileModel power{pwrCfg};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &power};
    Watch w{h.clock()};

    h.commandBodyTwist(ChassisSpeeds{Velocity{100.0}, Velocity{0.0}, AngularVelocity{0.0}});
    double speedBefore = 0.0;
    double speedAfter = 0.0;
    h.runTicks(2000, Time{0.01}, [&](int tick) {
        // wire the modeled temperatures into the fakes + monitor (C1's shape)
        for (int i = 0; i < h.motorCount(); ++i) {
            h.motor(i).setTemperature(power.temperatureC(i));
        }
        w.monitor.tick({.imuReady = h.imu().isReady(),
                        .batteryVolts = h.battery().voltage(),
                        .maxMotorTempC = power.maxTemperatureC()});
        if (tick == 500) {
            speedBefore = h.trueBodyTwist().vx().value();
        }
        if (tick == 1999) {
            speedAfter = h.trueBodyTwist().vx().value();
        }
    });

    CHECK(w.latch.firstFault() == FaultCode::MotorOverTemp);
    CHECK(h.motor(0).temperature() >= 55.0);      // visible through the F4 fake
    CHECK(speedAfter < 0.6 * speedBefore);        // the droop is not cosmetic
    CHECK(speedAfter > 0.0);                      // degraded, not dead
}
