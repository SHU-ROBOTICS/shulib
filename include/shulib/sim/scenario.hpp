#pragma once
//
// sim::SimHarness — the deterministic scenario runner (chunk A2, scope items 3–4):
// one object that wires a complete fake robot around a DrivePlant and drives
// seeded, bit-reproducible, replayable closed-loop runs.
//
// ── What it gives a test ────────────────────────────────────────────────────────────
//   * every F4 fake, constructed and plumbed (clock, motors, IMU, GPS, battery,
//     two tracking-wheel encoders, tags, vision, telemetry sink);
//   * a chassis::RobotContext over those fakes — so code under test reads hardware
//     EXACTLY as it will on the robot (constraint 1: the plant sits BEHIND the
//     fakes; nothing under test can tell the difference);
//   * the DrivePlant, stepped in lockstep with the injected FakeClock;
//   * ground-truth accessors and a memcmp-able per-tick TruthSample for the
//     determinism/replay proofs (constraint 4) and Phase E error measurement;
//   * tracking-wheel factories whose geometry MATCHES the plant's synthesis specs —
//     the honest wiring. A test that wants to prove the harness can DETECT estimator
//     error deliberately builds its own mis-calibrated wheels instead (see
//     test/sim_odometry_truth_test.cpp).
//
// ── The scenario contract (what "replayable" means precisely) ───────────────────────
// A scenario is: the harness config (including the seed) + the command sequence +
// the dt schedule. Two harnesses built from equal configs and driven by the same
// calls produce BYTE-IDENTICAL TruthSample streams, run to run — pinned by test.
// The seed's consumers are the plant's Rng (A3 degradation draws — none at A2) and
// whatever the scenario itself draws via rng() (e.g. randomBodyTwist below), so a
// different seed genuinely changes the run. No wall-clock exists anywhere; time is
// the FakeClock the plant advances. (Cross-libm caveat recorded in drive_plant.hpp.)
//
// ── Loop shape (documented so every consumer agrees on sensor timing) ───────────────
//     harness.runTicks(n, dt, [&](int tick) {  <controller: read fakes, set volts>  });
// The controller runs FIRST each tick and sees the world at time t; step(dt) then
// advances physics and clock to t + dt and refreshes every sensor. A Pid reading
// context().clock() therefore measures exactly one dt between calls — the same
// shape C1's motion loop will have on the robot.
//
// The VARIABLE-dt overload is the A3 LOOP-JITTER SEAM (degradation.hpp's note):
// hostile timing is injected by handing it a dt schedule, not by changing the plant.
//
// Host-test infrastructure: allocation and virtuals are fine here; this never runs
// on the V5. Single-task by contract.

#include <array>
#include <cstddef>
#include <span>

#include "shulib/chassis/robot_context.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/fake/fake_tag_source.hpp"
#include "shulib/hal/fake/fake_vision.hpp"
#include "shulib/hal/null_sink.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/drive_plant.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// One tick of ground truth, as raw doubles: trivially copyable so two runs can be
/// compared BYTE-FOR-BYTE (memcmp) — the determinism proof compares representations,
/// not tolerances. Harness/assertion currency only; never an estimator input.
struct TruthSample {
    double t = 0.0;      ///< clock time (s)
    double x = 0.0;      ///< true field x (in)
    double y = 0.0;      ///< true field y (in)
    double theta = 0.0;  ///< true heading, UNWRAPPED (rad)
    double vx = 0.0;     ///< true body vx (in/s)
    double vy = 0.0;     ///< true body vy (in/s)
    double omega = 0.0;  ///< true yaw rate (rad/s)
};
static_assert(sizeof(TruthSample) == 7 * sizeof(double),
              "TruthSample must stay packed doubles (memcmp-comparable)");

/// A seeded random body twist for sweep scenarios — |vx|,|vy| <= vMax (in/s),
/// |omega| <= omegaMax (rad/s). Draw order is part of the scenario contract
/// (vx, vy, omega), so the same Rng state always yields the same twist.
[[nodiscard]] inline math::ChassisSpeeds randomBodyTwist(Rng& rng, double vMax, double omegaMax) {
    const double vx = rng.uniform(-vMax, vMax);
    const double vy = rng.uniform(-vMax, vMax);
    const double omega = rng.uniform(-omegaMax, omegaMax);
    return math::ChassisSpeeds{units::Velocity{vx}, units::Velocity{vy},
                               units::AngularVelocity{omega}};
}

struct SimHarnessConfig {
    DrivePlantConfig plant{};
    /// Tracking-wheel geometry the PLANT synthesizes with. Non-zero offsets by
    /// default ON PURPOSE: the default harness exercises the offset-correction
    /// math every run instead of hiding it behind zeros (adversarial by default).
    /// The VALUES are invented stand-ins — the real robot's mounting geometry does
    /// not exist yet; R3 measures it (A4 register HA-12 offsets/signs, HA-13 diameter).
    units::Length trackingWheelDiameter{2.0};
    units::Length forwardWheelLeftOffset{-3.0};    ///< forward wheel's +LEFT coordinate
    units::Length lateralWheelForwardOffset{-4.5};  ///< lateral wheel's +FORWARD coordinate
};

class SimHarness {
public:
    /// `kinematics` (and `sink`/`degradation` when given) must outlive the harness.
    /// Defaults: telemetry → an owned NullSink (a run costs nothing unless a test
    /// attaches a sink); degradation → the owned identity model (the perfect robot).
    explicit SimHarness(const kinematics::IKinematics& kinematics,
                        const SimHarnessConfig& config = {},
                        hal::ITelemetrySink* sink = nullptr,
                        DegradationModel* degradation = nullptr)
        : cfg_{config},
          n_{kinematics.wheelCount()},
          kin_{kinematics},
          sink_{sink != nullptr ? sink : &ownNullSink_},
          fakeMotorPtrs_{makePtrs<hal::fake::FakeMotor>(motorStorage_)},
          iMotorPtrs_{makePtrs<hal::IMotor>(motorStorage_)},
          trackingSpecs_{TrackingWheelSpec{&forwardEncoder_, TrackingAxis::Forward,
                                           config.forwardWheelLeftOffset,
                                           config.trackingWheelDiameter},
                         TrackingWheelSpec{&lateralEncoder_, TrackingAxis::Lateral,
                                           config.lateralWheelForwardOffset,
                                           config.trackingWheelDiameter}},
          ff_{config.plant.wheelFf},
          plant_{kinematics,
                 std::span<hal::fake::FakeMotor* const>{fakeMotorPtrs_.data(),
                                                        static_cast<std::size_t>(n_)},
                 clock_,
                 imu_,
                 gps_,
                 battery_,
                 std::span<const TrackingWheelSpec>{trackingSpecs_.data(), trackingSpecs_.size()},
                 degradation != nullptr ? *degradation : ownIdentityDegradation_,
                 *sink_,
                 config.plant},
          context_{chassis::RobotContextConfig{
              .clock = &clock_,
              .driveMotors = std::span<hal::IMotor* const>{iMotorPtrs_.data(),
                                                           static_cast<std::size_t>(n_)},
              .imu = &imu_,
              .gps = &gps_,
              .battery = &battery_,
              .telemetry = sink_,
              .tags = &tags_,
              .vision = &vision_}} {}

    // ── the world as the code under test sees it (constraint 1) ────────────────────
    [[nodiscard]] chassis::RobotContext& context() noexcept { return context_; }
    [[nodiscard]] hal::fake::FakeClock& clock() noexcept { return clock_; }
    [[nodiscard]] hal::fake::FakeImu& imu() noexcept { return imu_; }
    [[nodiscard]] hal::fake::FakeGps& gps() noexcept { return gps_; }
    [[nodiscard]] hal::fake::FakeBattery& battery() noexcept { return battery_; }
    [[nodiscard]] hal::fake::FakeMotor& motor(int i) {
        SHULIB_PRECONDITION(i >= 0 && i < n_, "SimHarness::motor: index out of range");
        return motorStorage_[static_cast<std::size_t>(i)];
    }
    [[nodiscard]] int motorCount() const noexcept { return n_; }
    [[nodiscard]] hal::fake::FakeRotation& forwardEncoder() noexcept { return forwardEncoder_; }
    [[nodiscard]] hal::fake::FakeRotation& lateralEncoder() noexcept { return lateralEncoder_; }

    /// Tracking wheels wired with the SAME geometry the plant synthesizes with —
    /// the honest configuration for an odometry under test. (For a deliberate
    /// mis-calibration experiment, construct TrackingWheels with different numbers.)
    [[nodiscard]] localization::TrackingWheel makeForwardTrackingWheel() {
        return localization::TrackingWheel::forward(forwardEncoder_, cfg_.trackingWheelDiameter,
                                                    cfg_.forwardWheelLeftOffset);
    }
    [[nodiscard]] localization::TrackingWheel makeLateralTrackingWheel() {
        return localization::TrackingWheel::lateral(lateralEncoder_, cfg_.trackingWheelDiameter,
                                                    cfg_.lateralWheelForwardOffset);
    }

    // ── the plant / truth side (harness + assertions only, constraint 3) ───────────
    [[nodiscard]] DrivePlant& plant() noexcept { return plant_; }
    [[nodiscard]] math::Pose2d truePose() const { return plant_.truePose(); }
    [[nodiscard]] math::Twist2d trueBodyTwist() const noexcept { return plant_.trueBodyTwist(); }
    [[nodiscard]] Rng& rng() noexcept { return plant_.rng(); }

    /// This instant's ground truth as a memcmp-able sample (see TruthSample).
    [[nodiscard]] TruthSample sample() const {
        const TruthState& s = plant_.truthState();
        const math::Twist2d tw = plant_.trueBodyTwist();
        return TruthSample{clock_.now().value(), s.x,           s.y,
                           s.theta,              tw.vx().value(), tw.vy().value(),
                           tw.omega().value()};
    }

    // ── commanding (mirrors what C1's motion loop will do) ─────────────────────────
    /// BODY-frame twist → per-wheel voltages via toWheels + the configured
    /// feedforward. No desaturation and no field rotation here on purpose: those are
    /// the motion layer's jobs (§13 #5 / F1); a scenario that wants them applies
    /// them before calling this.
    void commandBodyTwist(const math::ChassisSpeeds& body) {
        const kinematics::WheelSpeeds target = kin_.toWheels(body);
        for (int i = 0; i < n_; ++i) {
            motorStorage_[static_cast<std::size_t>(i)].setVoltage(ff_.calculate(target[i]));
        }
    }

    /// Zero every drive voltage (coast — the F4 setVoltage(0) semantics).
    void stopAllMotors() {
        for (int i = 0; i < n_; ++i) {
            motorStorage_[static_cast<std::size_t>(i)].setVoltage(units::Voltage{0.0});
        }
    }

    // ── the run loop (see "Loop shape" in the header) ──────────────────────────────
    /// Fixed dt: perTick(i) runs first (sees time t_i), then the plant advances.
    template <typename PerTickFn>
    void runTicks(int ticks, units::Time dt, PerTickFn&& perTick) {
        SHULIB_PRECONDITION(ticks >= 0, "SimHarness::runTicks: ticks must be >= 0");
        for (int i = 0; i < ticks; ++i) {
            perTick(i);
            plant_.step(dt);
        }
    }

    /// Open-loop convenience: hold whatever is commanded for `ticks` steps.
    void runTicks(int ticks, units::Time dt) {
        runTicks(ticks, dt, [](int) {});
    }

    /// VARIABLE dt — the A3 loop-jitter seam: `dtFor(i)` supplies each tick's dt
    /// (hostile schedules are injected here, never inside the plant).
    template <typename DtFn, typename PerTickFn>
    void runTicksVariable(int ticks, DtFn&& dtFor, PerTickFn&& perTick) {
        SHULIB_PRECONDITION(ticks >= 0, "SimHarness::runTicksVariable: ticks must be >= 0");
        for (int i = 0; i < ticks; ++i) {
            perTick(i);
            plant_.step(dtFor(i));
        }
    }

private:
    template <typename PtrT>
    static std::array<PtrT*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
    makePtrs(std::array<hal::fake::FakeMotor,
                        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>& storage) {
        std::array<PtrT*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)> out{};
        for (std::size_t i = 0; i < storage.size(); ++i) {
            out[i] = &storage[i];
        }
        return out;
    }

    SimHarnessConfig cfg_;
    int n_;
    const kinematics::IKinematics& kin_;

    hal::fake::FakeClock clock_{};
    hal::fake::FakeImu imu_{};
    hal::fake::FakeGps gps_{};
    hal::fake::FakeBattery battery_{};
    hal::fake::FakeTagSource tags_{};
    hal::fake::FakeVision vision_{};
    hal::NullSink ownNullSink_{};
    DegradationModel ownIdentityDegradation_{};
    hal::ITelemetrySink* sink_;

    std::array<hal::fake::FakeMotor, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        motorStorage_{};
    std::array<hal::fake::FakeMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        fakeMotorPtrs_;
    std::array<hal::IMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        iMotorPtrs_;
    std::array<TrackingWheelSpec, 2> trackingSpecs_;
    hal::fake::FakeRotation forwardEncoder_{};
    hal::fake::FakeRotation lateralEncoder_{};

    control::Feedforward ff_;
    DrivePlant plant_;
    chassis::RobotContext context_;
};

}  // namespace shulib::sim
