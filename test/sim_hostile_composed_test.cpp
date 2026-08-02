// The composition layer under test: ChainedDegradation semantics, FullHostility's
// canonical "everything hostile at once" world, byte-identical determinism UNDER
// hostility (the constraint that keeps a hostile failure reproducible instead of
// folklore), ablation-based reducibility (a composed failure attributes to its
// family by removing exactly one model), and the catastrophic tier: every event
// pathology armed at once, and the stack still completes with faults latched and
// a finite pose on every single tick.

#include "doctest.h"

#include <cmath>
#include <cstring>
#include <vector>

#include "shulib/control/pid.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/finite_guard.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/hal/fake/fake_telemetry_sink.hpp"
#include "shulib/kinematics/x_drive.hpp"
#include "shulib/localization/complementary_fusion.hpp"
#include "shulib/localization/localizer.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
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
using shulib::hal::fake::FakeTelemetrySink;
using shulib::kinematics::xDrive;
using shulib::localization::ComplementaryFusion;
using shulib::localization::Localizer;
using shulib::localization::PilonsOdometry;
using shulib::math::Angle;
using shulib::math::ChassisSpeeds;
using shulib::sim::ChainedDegradation;
using shulib::sim::DegradationModel;
using shulib::sim::FullHostility;
using shulib::sim::FullHostilityConfig;
using shulib::sim::Rng;
using shulib::sim::SimHarness;
using shulib::sim::SimHarnessConfig;
using shulib::sim::TruthSample;
using shulib::units::AngleDim;
using shulib::units::AngularVelocity;
using shulib::units::Length;
using shulib::units::Time;
using shulib::units::Velocity;
using shulib::units::Voltage;

namespace {
constexpr double kDegToRad = Angle::kPi / 180.0;

[[nodiscard]] SimHarnessConfig instantConfig(std::uint64_t seed = 1) {
    SimHarnessConfig cfg;
    cfg.plant.wheelFf = {.kS = 1.2, .kV = 0.17, .kA = 0.0};
    cfg.plant.seed = seed;
    return cfg;
}

[[nodiscard]] double posErr(const shulib::math::Pose2d& a, const shulib::math::Pose2d& b) {
    return std::hypot((a.x() - b.x()).value(), (a.y() - b.y()).value());
}

/// The composed scenario every determinism/bounded-error case runs: 2.5 s
/// stationary boot (the 2 s calibration window + the Localizer's 0.1 s settle
/// window + margin — the C1 wait-for-live-estimate contract), then a scripted mix
/// of motion classes.
[[nodiscard]] ChassisSpeeds composedScript(int tick) {
    if (tick < 250) {
        return {};  // wait out calibration + settle, like a real auton start
    }
    const int phase = (tick - 250) / 200;  // 2 s per segment
    switch (phase % 5) {
        case 0: return {Velocity{15.0}, Velocity{0.0}, AngularVelocity{0.0}};
        case 1: return {Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.5}};
        case 2: return {Velocity{0.0}, Velocity{12.0}, AngularVelocity{0.0}};
        case 3: return {Velocity{-8.0}, Velocity{4.0}, AngularVelocity{-0.6}};
        default: return {Velocity{0.0}, Velocity{0.0}, AngularVelocity{0.0}};
    }
}
}  // namespace

TEST_CASE("composed: an empty chain and a chain of identity bases are the exact identity") {
    ChainedDegradation empty{{}};
    DegradationModel id1;
    DegradationModel id2;
    ChainedDegradation ids{{&id1, &id2}};
    Rng rng{1};
    for (ChainedDegradation* chain : {&empty, &ids}) {
        CHECK(chain->effectiveVoltage(0, Voltage{7.5}, Time{1.0}, rng).value() == 7.5);
        CHECK(chain->driveEncoderPosition(2, AngleDim{3.25}, Time{1.0}, rng).value() == 3.25);
        CHECK(chain->imuHeading(Angle::degrees(12.0), Time{1.0}, rng).radians()
              == Angle::degrees(12.0).radians());
        CHECK(chain->imuReady(true, Time{1.0}));
        CHECK(chain->batteryVoltage(Voltage{12.6}, Time{1.0}, rng).value() == 12.6);
    }
}

TEST_CASE("composed: the chain folds LEFT to RIGHT (order is semantics, pinned)") {
    struct PlusOne final : DegradationModel {
        AngleDim driveEncoderPosition(int, AngleDim v, Time, Rng&) override {
            return AngleDim{v.value() + 1.0};
        }
    } plusOne;
    struct TimesTwo final : DegradationModel {
        AngleDim driveEncoderPosition(int, AngleDim v, Time, Rng&) override {
            return AngleDim{v.value() * 2.0};
        }
    } timesTwo;
    Rng rng{1};
    ChainedDegradation ab{{&plusOne, &timesTwo}};
    ChainedDegradation ba{{&timesTwo, &plusOne}};
    CHECK(ab.driveEncoderPosition(0, AngleDim{3.0}, Time{0.0}, rng).value() == 8.0);  // (3+1)·2
    CHECK(ba.driveEncoderPosition(0, AngleDim{3.0}, Time{0.0}, rng).value() == 7.0);  // 3·2+1
    CHECK_THROWS_AS((ChainedDegradation{{&plusOne, nullptr}}), shulib::PreconditionError);
}

// ── Liveness of the DEFAULT composed world: every family's most direct observable
// must visibly change, or a dead model shipped inside FullHostility. ──
TEST_CASE("composed: a default FullHostility degrades every seam family observably") {
    FullHostility full{};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &full.model()};

    // IMU family: the calibration window is live from the very first sensor write.
    CHECK_FALSE(h.imu().isReady());

    // stationary boot, then a hard launch + steady cruise
    h.runTicks(200, Time{0.01});
    CHECK(h.imu().isReady());  // window over

    h.commandBodyTwist(ChassisSpeeds{Velocity{25.0}, Velocity{0.0}, AngularVelocity{0.0}});
    // With the kA = 0 (instant) motor model, spin reaches steady state in ONE tick,
    // so the accel-slip verdict lives on exactly the FIRST step after the command
    // (the 200 boot ticks already baselined each wheel's spin history at zero).
    h.plant().step(Time{0.01});
    // SLIP family: the body trails the spinning wheels during the launch tick.
    const double spin = std::abs(h.plant().trueWheelSpin()[0].value()) * std::sqrt(2.0);
    CHECK(h.trueBodyTwist().vx().value() < spin * 0.75);

    double gpsRepeats = 0.0;
    double gpsLagSum = 0.0;
    int gpsTicks = 0;
    double prevGpsX = h.gps().pose().x().value();
    h.runTicks(200, Time{0.01}, [&](int) {
        const double gx = h.gps().pose().x().value();
        if (gx == prevGpsX) {
            gpsRepeats += 1.0;  // decimation: the same fix re-reported
        }
        gpsLagSum += h.truePose().x().value() - gx;
        ++gpsTicks;
        prevGpsX = gx;
    });
    // GPS family: decimation holds each fix for several ticks…
    CHECK(gpsRepeats > 100.0);
    // …and latency + decimation put the reported position measurably BEHIND a
    // moving robot (mean lag ≈ v·(latency + period/2) ≈ 1.5 in ≫ the noise mean).
    CHECK(gpsLagSum / gpsTicks > 0.5);

    // ENCODER family: the tracking reading sits on the centidegree grid even
    // after the latency stage (a delayed sample of an on-grid value is on-grid).
    const double step = 2.0 * Angle::kPi / 36000.0;
    const double reading = h.forwardEncoder().position().value();
    CHECK(std::abs(reading / step - std::round(reading / step)) < 1e-6);

    // IMU family again: heading is NOT the truth (drift + noise are live).
    CHECK(std::abs(h.truePose().heading().errorTo(h.imu().heading())) > 1e-6);

    // POWER family: the pack sagged below nominal under load + discharge.
    CHECK(h.battery().voltage().value() < 12.6 - 0.05);
}

// ── THE DoD run: the full production stack (odom + Localizer, no correctors — the
// honest M2 configuration) completes a 14 s run under the composed world with
// BOUNDED, REPORTED error and honest quality. ──
TEST_CASE("composed: the full stack completes a 14 s hostile run with bounded, reported error") {
    FullHostility full{};
    const auto kin = xDrive(Length{7.0});
    SimHarness h{kin, instantConfig(), nullptr, &full.model()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};

    double worstErr = 0.0;
    bool everUninitAfterBoot = false;
    h.runTicks(1400, Time{0.01}, [&](int tick) {
        loc.update();
        REQUIRE(isFinitePose(loc.pose()));
        if (tick > 250) {
            worstErr = std::max(worstErr, posErr(loc.pose(), h.truePose()));
            everUninitAfterBoot =
                everUninitAfterBoot || loc.qualityClass() == Localizer::Quality::Uninitialized;
        }
        h.commandBodyTwist(composedScript(tick));
    });
    const double endErr = posErr(loc.pose(), h.truePose());
    const double endHeadingErrDeg =
        std::abs(h.truePose().heading().errorTo(loc.pose().heading())) / kDegToRad;

    MESSAGE("composed 14 s run: worst position error ", worstErr, " in, end ", endErr,
            " in, end heading error ", endHeadingErrDeg, " deg");
    // REPORTED and BOUNDED: drift/noise/latency-scale error, fractions of an inch
    // to low inches, not feet — and NOT zero, or the hostility died. (The tight
    // sub-inch bounds belong to the perfect-sensor logic proofs in
    // sim_odometry_truth_test.cpp; anything over ~2 in here would mean a boundary
    // leak of the boot-poisoning class survived.)
    CHECK(worstErr > 0.02);
    CHECK(worstErr < 2.0);
    CHECK(endErr < 2.0);
    CHECK(endHeadingErrDeg < 1.0);  // the F2-scale quantity, riding drift + noise
    CHECK_FALSE(everUninitAfterBoot);  // quality stayed alive after calibration
    CHECK(loc.qualityClass() == Localizer::Quality::Degraded);  // and HONEST: ~90 in
    CHECK(loc.isDeadReckoning());                               // dead-reckoned, no fix
}

// ── Determinism UNDER hostility (constraint: an unreproducible hostile failure is
// folklore): same seed → byte-identical truth AND bit-identical estimator trace;
// different seed → genuinely different PHYSICS. The run is CLOSED-LOOP on purpose:
// under an open-loop script the seed only perturbs the sensor lies, never the
// motion (all truth-side hostility — sag, slip, thermal — is deliberately
// deterministic; recorded honestly in A3-COMPLETED), so only a controller in the
// loop lets seeded noise reach the trajectory and prove end-to-end divergence. ──
TEST_CASE("composed: same seed replays byte-identically under full hostility; different seed diverges") {
    const auto kin = xDrive(Length{7.0});

    struct Trace {
        std::vector<TruthSample> truth;
        std::vector<double> fused;  // x, y per tick — the estimator's own trace
    };
    auto runOne = [&](std::uint64_t seed) {
        FullHostility full{};
        SimHarness h{kin, instantConfig(seed), nullptr, &full.model()};
        PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
        ComplementaryFusion fusion{};
        Localizer loc{h.clock(), h.imu(), odom, fusion};
        Pid pid{PidConfig{.kP = 3.0, .outputMin = -60.0, .outputMax = 60.0}, h.clock()};
        Trace t;
        h.runTicks(800, Time{0.01}, [&](int tick) {
            loc.update();
            t.truth.push_back(h.sample());
            t.fused.push_back(loc.pose().x().value());
            t.fused.push_back(loc.pose().y().value());
            if (tick < 250) {
                return;  // hold through calibration + settle (the C1 contract)
            }
            const double cmd = pid.update(24.0, loc.pose().x().value());
            h.commandBodyTwist(ChassisSpeeds{Velocity{cmd}, Velocity{0.0}, AngularVelocity{0.0}});
        });
        return t;
    };

    const Trace a = runOne(2026);
    const Trace b = runOne(2026);
    REQUIRE(a.truth.size() == b.truth.size());
    CHECK(std::memcmp(a.truth.data(), b.truth.data(),
                      a.truth.size() * sizeof(TruthSample)) == 0);
    REQUIRE(a.fused.size() == b.fused.size());
    CHECK(std::memcmp(a.fused.data(), b.fused.data(), a.fused.size() * sizeof(double)) == 0);

    const Trace c = runOne(2027);
    CHECK(std::memcmp(a.truth.data(), c.truth.data(),
                      a.truth.size() * sizeof(TruthSample)) != 0);  // the PHYSICS differ
    CHECK(std::abs(a.fused.back() - c.fused.back()) > 1e-9);        // and so does the estimate
}

// ── Reducibility by ablation: removing exactly one family's model removes exactly
// that family's signature. Two DRAW-INDEPENDENT attributions (slip, power) plus
// the drift attribution on a seed whose drawn bias stands above the noise floor. ──
TEST_CASE("composed: a composed-run signature is attributable by removing one model") {
    const auto kin = xDrive(Length{7.0});

    SUBCASE("slip: the launch gap vanishes when (only) the slip model is removed") {
        auto launchGap = [&](bool withSlip) {
            FullHostilityConfig cfg;
            if (!withSlip) {
                cfg.slip.accelThresholdInPerS2 = 1e18;  // ablated (never triggers)
            }
            FullHostility full{cfg};
            SimHarness h{kin, instantConfig(), nullptr, &full.model()};
            h.runTicks(200, Time{0.01});  // boot (baselines the slip history at zero)
            h.commandBodyTwist(ChassisSpeeds{Velocity{25.0}, Velocity{0.0}, AngularVelocity{0.0}});
            h.plant().step(Time{0.01});  // instant model: the launch IS this one tick
            const double spin = std::abs(h.plant().trueWheelSpin()[0].value()) * std::sqrt(2.0);
            return spin - h.trueBodyTwist().vx().value();
        };
        CHECK(launchGap(true) > 1.0);       // composed: wheels outrun the body
        CHECK(launchGap(false) < 1e-9);     // minus slip: the gap is exactly gone
    }

    SUBCASE("power: the battery sag vanishes when (only) the power model is removed") {
        auto endBattery = [&](bool withPower) {
            FullHostilityConfig cfg;
            if (!withPower) {
                cfg.power.sagPerCommandedVolt = 0.0;
                cfg.power.dischargeRatePerS = 0.0;
            }
            FullHostility full{cfg};
            SimHarness h{kin, instantConfig(), nullptr, &full.model()};
            h.commandBodyTwist(ChassisSpeeds{Velocity{30.0}, Velocity{0.0}, AngularVelocity{0.0}});
            h.runTicks(300, Time{0.01});
            return h.battery().voltage().value();
        };
        CHECK(endBattery(true) < 12.6 - 0.1);
        CHECK(endBattery(false) == doctest::Approx(12.6));
    }

    SUBCASE("imu: the end-of-run heading error collapses when (only) drift+noise are removed") {
        // Swept over three boots chosen to SPAN the per-boot bias distribution —
        // seeds 1/3/10 draw bias fractions +0.13/−0.77/−0.93 of the ±1°/min bound
        // (computed from the pinned SplitMix64 first draw; representative range,
        // not picked to make a number pass). The attribution claims: with the IMU
        // model removed the error is ~0 on EVERY boot, and with it live the
        // worst-of-boots error carries the drift signature the bound predicts.
        auto endHeadingErr = [&](bool withImuLies, std::uint64_t seed) {
            FullHostilityConfig cfg;
            cfg.imu.calibrationEnd = Time{0.0};  // same boot either way
            if (!withImuLies) {
                cfg.imu.rateBiasMax = AngularVelocity{0.0};
                cfg.imu.headingNoiseSigmaRad = 0.0;
                cfg.imu.yawRateNoiseSigmaRadPerS = 0.0;
            }
            FullHostility full{cfg};
            SimHarness h{kin, instantConfig(seed), nullptr, &full.model()};
            h.commandBodyTwist(ChassisSpeeds{Velocity{10.0}, Velocity{0.0}, AngularVelocity{0.0}});
            h.runTicks(3000, Time{0.01});  // 30 s straight
            h.stopAllMotors();
            h.runTicks(10, Time{0.01});    // drain the latency window
            return std::abs(h.truePose().heading().errorTo(h.imu().heading()));
        };
        double worstWith = 0.0;
        for (std::uint64_t seed : {1ULL, 3ULL, 10ULL}) {
            worstWith = std::max(worstWith, endHeadingErr(true, seed));
            CHECK(endHeadingErr(false, seed) < 1e-9);  // ablated: exactly gone, every boot
        }
        // 0.773°/min × 0.5 min ≈ 0.39° on seed 3 alone; noise σ0.05° cannot mask it.
        CHECK(worstWith > 0.3 * kDegToRad);
    }
}

// ── The CATASTROPHIC tier: every event pathology armed in one run — brownout,
// sentinel breach, IMU disconnect, GPS disconnect, encoder freeze. The stack must
// complete the run with a finite pose on EVERY tick and the faults latched in
// cause order. This is "no crash, no NaN, no silent wrong answer" at full volume. ──
TEST_CASE("composed catastrophic: every event pathology at once — finite, faulted, completed") {
    FullHostilityConfig cfg;
    cfg.imu.dropoutAt = Time{8.0};
    cfg.gps.dropoutAt = Time{5.0};
    cfg.encoders.sentinelAt = Time{4.5};   // +∞ on tracking wheel 0 for 50 ms
    cfg.encoders.trackingFreezeAt = Time{9.0};
    cfg.encoders.trackingFreezeIndex = 1;
    FullHostility full{cfg};

    const auto kin = xDrive(Length{7.0});
    SimHarnessConfig hCfg = instantConfig();
    hCfg.plant.batteryVoltage = Voltage{11.2};  // a weak pack: full throttle browns out
    SimHarness h{kin, hCfg, nullptr, &full.model()};
    PilonsOdometry odom{h.imu(), h.makeForwardTrackingWheel(), h.makeLateralTrackingWheel()};
    ComplementaryFusion fusion{};
    Localizer loc{h.clock(), h.imu(), odom, fusion};
    FakeTelemetrySink sink;
    FaultLatch latch{sink, h.clock()};
    HealthMonitor monitor{latch};

    h.runTicks(1000, Time{0.01}, [&](int tick) {
        loc.update();
        monitor.tick({.imuReady = h.imu().isReady(),
                      .odomImplausible = h.imu().isReady() && odom.lastDeltaImplausible(),
                      .batteryVolts = h.battery().voltage()});
        REQUIRE(isFinitePose(loc.pose()));   // the absolute guarantee, all 1000 ticks
        REQUIRE(isFinitePose(odom.pose()));
        // full throttle in [2,4) s browns the weak pack out; moderate elsewhere
        const bool fullThrottle = tick >= 200 && tick < 400;
        const double vx = fullThrottle ? 100.0 : 12.0;
        h.commandBodyTwist(ChassisSpeeds{Velocity{vx}, Velocity{0.0}, AngularVelocity{0.3}});
    });

    CHECK(h.clock().now().value() == doctest::Approx(10.0));  // the run COMPLETED
    CHECK(latch.firstFault() == FaultCode::Brownout);          // first cause, kept
    CHECK(latch.faultCount() >= 3);                            // the cascade is real
    CHECK(monitor.brownedOut());
    CHECK(monitor.imuLost());
    CHECK(loc.qualityClass() == Localizer::Quality::Degraded);
    CHECK(loc.quality() == 0.0);
}
