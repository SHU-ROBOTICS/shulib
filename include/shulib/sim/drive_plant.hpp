#pragma once
//
// sim::DrivePlant — the host plant that closes the loop (chunk A2, the missing
// prerequisite): commanded voltage → wheel velocity → body twist → TRUE pose →
// synthesized sensor readings pushed into the existing F4 fakes.
//
// With no robot, this object is the only means by which any closed-loop behaviour in
// shulib can be validated at all; every chunk in Phases C, D, E and F is proven or
// disproven against it. Its one advantage over a physical field: it possesses GROUND
// TRUTH — the exact pose at every tick — which is what makes Phase E's estimator
// error MEASURABLE rather than inferred.
//
// ── The tick pipeline (step(dt)) ────────────────────────────────────────────────────
//   1. read each FakeMotor's commandedVoltage()   (post ±12 V clamp — the F4 contract)
//   2. DegradationModel::effectiveVoltage()       (A3 seam; identity today)
//   3. MotorModel::advance() per wheel            (the inverted-feedforward "dynamics")
//   4. DegradationModel::wheelMotionVelocity()    (A3 slip seam; identity today)
//   5. IKinematics::forward(motion wheels)        → the TRUE body twist
//   6. advanceTruth() (RK4 sub-stepping)          → the TRUE pose   [NEVER arcStep —
//        see truth_integrator.hpp; that independence is constraint 2, the one that
//        keeps every Phase E localization test meaningful]
//   7. clock.advance(dt)                          (the plant is the ONE time authority,
//        so physics and time can never skew; controllers read the same FakeClock)
//   8. synthesize sensors FROM TRUTH → the fakes  (each through its A3 seam)
//   9. emitRecord() — one DebugRecord per tick    (A1's lazy-build contract: a
//        NullSink run never even populates the record)
//
// DISCRETE-TIME SEMANTICS (zero-order hold, a documented contract): each tick, the
// wheel velocities advance to their END-of-tick values (steps 3–4) and THAT twist is
// held constant across the tick for the pose integral (step 6). For kA = 0 configs —
// what most logic tests use — this is EXACT (velocity is constant within a tick
// anyway). Under a first-order lag it carries an O(dt) transient bias vs. the
// continuous model, bounded and pinned by test (sim_plant_test.cpp's discrete-sum
// case). Chosen over an exact-average-velocity integral because "constant twist per
// tick" is the same per-tick model the truth integrator AND the odometry derivation
// assume — one semantics everywhere beats a slightly smoother transient.
//
// ── Which kinematics, and why sharing it is correct here (unlike arcStep) ───────────
// Step 5 uses the SAME IKinematics contract the motion layer uses. That is
// deliberate, and different in kind from the arcStep prohibition:
//   * F5 froze this contract and its implementations are independently oracle-tested
//     (closed-form geometry in x_drive/tank/matrix tests) — the plant must agree with
//     what motion code computes BY CONTRACT, and a privately re-derived second
//     kinematics could silently diverge from it;
//   * no estimator consumes kinematics::forward() at M2 — sensor synthesis (step 8)
//     derives from the TRUE POSE, not from kinematics — so no Phase E
//     estimator-vs-truth comparison can cancel through it;
//   * arcStep, by contrast, IS the odometry's integrator: sharing it would null every
//     localization measurement. Shared: the frozen F5 contract. Independent: the
//     pose integrator. Both on purpose.
//
// ── Sensor synthesis (step 8) — everything derives from truth ───────────────────────
//   * IMU: heading = wrapped true θ; yawRate = true ω          (→ FakeImu)
//   * Drive encoders: shaft ← ∫ SPIN/r                          (→ FakeMotor)
//       — the SPIN (pre-slip) velocity, because a real encoder measures the wheel,
//         not the floor; under A3 slip the encoder overcounts exactly as hardware does.
//   * Tracking wheels: unpowered, they measure ACTUAL MOTION at their mount point.
//       Rigid-body kinematics: v_point = v_body + ω × r, so a Forward wheel (offset
//       b = +LEFT coordinate) rolls at vx − ω·b and a Lateral wheel (offset a =
//       +FORWARD coordinate) rolls at vy + ω·a. This is derived HERE from ω × r,
//       independently of PilonsOdometry's inverse (which removes the same component);
//       both sides are separately pinned (its pure-rotation tests, our truth tests),
//       so a sign error cannot cancel end-to-end.  (→ FakeRotation, cumulative shaft)
//   * GPS: the true robot-center pose + configured rms/fix      (→ FakeGps)
//   * Battery: the configured nominal voltage                   (→ FakeBattery)
//
// ── Ground truth is the product — and must never leak (constraint 3) ────────────────
// truePose()/trueBodyTwist()/truthState() exist for the HARNESS and ASSERTIONS only.
// Structurally: no F4 interface exposes them, RobotContext cannot reach this object,
// and the CI layering guard forbids any core header from including shulib/sim/ — so
// code under test can only ever see the synthesized FAKES, exactly as on a robot.
//
// Determinism (constraint 4): given the same construction parameters, command
// sequence and dt schedule, every quantity here is a pure function of the seed — the
// only random source is the injected Rng (see rng.hpp), time comes from FakeClock,
// and iteration order is fixed arrays. Byte-identical replay is pinned by test.
// (Honest caveat, recorded: run-to-run and same-toolchain identity is guaranteed;
// IDENTITY ACROSS DIFFERENT libm implementations is not, since cos/sin/exp may differ
// in the last ulp between C libraries. The suite pins run-to-run identity.)
//
// NOT modeled here, on purpose: mass, inertia, torque curves, friction, slip physics
// (see motor_model.hpp's honesty boundary — this plant proves LOGIC, not CONSTANTS);
// hostile behaviours (A3 populates the DegradationModel seams); motion primitives
// (C1). Single-task by contract, like everything it drives.

#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/fake/fake_battery.hpp"
#include "shulib/hal/fake/fake_clock.hpp"
#include "shulib/hal/fake/fake_gps.hpp"
#include "shulib/hal/fake/fake_imu.hpp"
#include "shulib/hal/fake/fake_motor.hpp"
#include "shulib/hal/fake/fake_rotation.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/motor_model.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/sim/truth_integrator.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// Which body axis an unpowered tracking wheel rolls along. Mirrors
/// localization::TrackingWheel's role vocabulary (Forward: offset = +LEFT
/// coordinate; Lateral: offset = +FORWARD coordinate) without depending on it —
/// the plant must stay derivable from rigid-body kinematics alone (header note).
enum class TrackingAxis { Forward, Lateral };

/// One synthesized tracking-wheel channel: where the reading goes, and the
/// geometry truth uses to generate it. The geometry an odometry-under-test is
/// configured with lives with THAT test — deliberately separate, so a test can
/// mis-calibrate the estimator against the plant and watch the error grow.
struct TrackingWheelSpec {
    hal::fake::FakeRotation* sensor = nullptr;
    TrackingAxis axis = TrackingAxis::Forward;
    units::Length offset{};    ///< Forward: +LEFT coord; Lateral: +FORWARD coord (inches)
    units::Length diameter{};  ///< physical wheel diameter (inches), > 0
};

struct DrivePlantConfig {
    /// Per-wheel surface-speed feedforward gains (see motor_model.hpp). The defaults
    /// are PLACEHOLDERS in the right order of magnitude for a V5 drive (≈70 in/s free
    /// speed at 12 V with a little static offset) — NOT measurements. R5 measures the
    /// real ones; R6 feeds them back here. (A4 register HA-45.)
    control::FeedforwardGains wheelFf{.kS = 1.0, .kV = 12.0 / 70.0, .kA = 0.0};
    /// For drive-encoder synthesis (inches). Diameter AND the implied 1:1 wheel↔shaft
    /// gearing are unmeasured guesses until the drivetrain exists (A4 register HA-13/HA-14).
    units::Length driveWheelDiameter{3.25};
    int truthSubsteps = 32;                  ///< RK4 substeps per tick (see truth_integrator.hpp)
    math::Pose2d initialPose{};              ///< truth starts here; sensors seeded to match
    units::Voltage batteryVoltage{12.6};     ///< nominal pack voltage (A3 sags it via the seam; A4 register HA-46)
    units::Length gpsRmsError{1.0};          ///< reported GPS rms (A3 inflates via the seam)
    bool gpsHasFix = true;                   ///< A3 drops it via the seam
    std::uint64_t seed = 1;                  ///< the run's ONE random seed (rng.hpp)
};

class DrivePlant {
public:
    static constexpr int kMaxTrackingWheels = 4;

    /// All references/pointees must outlive the plant. `motors` must match
    /// kinematics.wheelCount() and be in the drivetrain's canonical wheel order.
    DrivePlant(const kinematics::IKinematics& kinematics,
               std::span<hal::fake::FakeMotor* const> motors, hal::fake::FakeClock& clock,
               hal::fake::FakeImu& imu, hal::fake::FakeGps& gps, hal::fake::FakeBattery& battery,
               std::span<const TrackingWheelSpec> trackingWheels, DegradationModel& degradation,
               hal::ITelemetrySink& sink, const DrivePlantConfig& config = {})
        : kin_{kinematics},
          clock_{clock},
          imu_{imu},
          gps_{gps},
          battery_{battery},
          degradation_{degradation},
          sink_{sink},
          cfg_{config},
          model_{config.wheelFf},
          rng_{config.seed},
          truth_{config.initialPose.x().value(), config.initialPose.y().value(),
                 config.initialPose.heading().radians()} {
        SHULIB_PRECONDITION(static_cast<int>(motors.size()) == kin_.wheelCount(),
                            "DrivePlant: motor count must match kinematics.wheelCount()");
        SHULIB_PRECONDITION(motors.size() >= 1u
                                && motors.size()
                                       <= static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels),
                            "DrivePlant: motor count must be in [1, kMaxWheels]");
        n_ = static_cast<int>(motors.size());
        for (std::size_t i = 0; i < motors.size(); ++i) {
            SHULIB_PRECONDITION(motors[i] != nullptr, "DrivePlant: a motor is null");
            motors_[i] = motors[i];
        }
        SHULIB_PRECONDITION(trackingWheels.size()
                                <= static_cast<std::size_t>(kMaxTrackingWheels),
                            "DrivePlant: too many tracking wheels");
        nTracking_ = static_cast<int>(trackingWheels.size());
        for (std::size_t i = 0; i < trackingWheels.size(); ++i) {
            SHULIB_PRECONDITION(trackingWheels[i].sensor != nullptr,
                                "DrivePlant: a tracking-wheel sensor is null");
            SHULIB_PRECONDITION(trackingWheels[i].diameter.value() > 0.0,
                                "DrivePlant: tracking-wheel diameter must be > 0");
            tracking_[i] = trackingWheels[i];
        }
        SHULIB_PRECONDITION(cfg_.driveWheelDiameter.value() > 0.0,
                            "DrivePlant: driveWheelDiameter must be > 0");
        SHULIB_PRECONDITION(cfg_.truthSubsteps >= 1, "DrivePlant: truthSubsteps must be >= 1");
        SHULIB_PRECONDITION(cfg_.batteryVoltage.value() >= 0.0,
                            "DrivePlant: batteryVoltage must be >= 0");

        // Seed every sensor from the INITIAL truth before any controller runs, so the
        // first tick reads a consistent world (odometry seeded from imu.heading() sees
        // the true start heading, not a default zero).
        synthesizeSensors();
    }

    /// One plant tick (the 9-step pipeline in the header). dt finite, >= 0;
    /// dt == 0 is an explicit no-op (no time passed: state, clock and sensors are
    /// untouched, and no record is emitted).
    void step(units::Time dt) {
        SHULIB_PRECONDITION(std::isfinite(dt.value()) && dt.value() >= 0.0,
                            "DrivePlant::step: dt must be finite and >= 0");
        if (dt.value() == 0.0) {
            return;
        }
        const units::Time now = clock_.now();

        // 1–4: voltage → spin → motion, per wheel, through the A3 seams.
        kinematics::WheelSpeeds motion{n_};
        for (int i = 0; i < n_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            const units::Voltage commanded = motors_[idx]->commandedVoltage();
            const units::Voltage effective =
                degradation_.effectiveVoltage(i, commanded, now, rng_);
            wheelSpin_[idx] = model_.advance(wheelSpin_[idx], effective, dt);
            motion.set(i, degradation_.wheelMotionVelocity(i, wheelSpin_[idx], now, rng_));
        }

        // 5: the TRUE body twist, via the frozen F5 contract (header: shared on purpose).
        bodyTwist_ = kin_.forward(motion);

        // 6: the TRUE pose, via RK4 — NEVER arcStep (constraint 2; truth_integrator.hpp).
        truth_ = advanceTruth(truth_, bodyTwist_, dt, cfg_.truthSubsteps);

        // 7: time. The plant is the single time authority (header).
        clock_.advance(dt);

        // 8: sensors, from truth, through the seams.
        integrateEncoders(dt);
        synthesizeSensors();

        // 9: one per-tick record — A1's lazy-build contract, so NullSink pays nothing.
        hal::emitRecord(sink_, [&] {
            diag::DebugRecord r;
            r.t = clock_.now();
            r.dt = dt;
            r.wheelCount = n_;
            for (int i = 0; i < n_; ++i) {
                const auto idx = static_cast<std::size_t>(i);
                r.wheelVoltage[idx] = motors_[idx]->commandedVoltage();
            }
            r.imuYaw = imu_.heading();        // what the (possibly degraded) sensor reports
            r.imuYawRate = imu_.yawRate();
            r.batteryVoltage = battery_.voltage();
            return r;
        });
    }

    // ── Ground truth — for the harness and assertions ONLY (constraint 3) ──────────
    [[nodiscard]] math::Pose2d truePose() const { return truth_.pose(); }
    [[nodiscard]] const TruthState& truthState() const noexcept { return truth_; }
    [[nodiscard]] math::Twist2d trueBodyTwist() const noexcept { return bodyTwist_; }
    /// True per-wheel SPIN surface speeds (pre-slip — what the encoders integrate).
    [[nodiscard]] kinematics::WheelSpeeds trueWheelSpin() const {
        kinematics::WheelSpeeds w{n_};
        for (int i = 0; i < n_; ++i) {
            w.set(i, wheelSpin_[static_cast<std::size_t>(i)]);
        }
        return w;
    }

    /// The run's one seeded random source (scenario generation + A3 degradation draws).
    [[nodiscard]] Rng& rng() noexcept { return rng_; }

private:
    /// Advance the cumulative encoder shafts by this tick's travel (constant twist
    /// over the tick ⇒ travel = velocity·dt exactly; no quadrature needed because
    /// BODY-frame rates are constant even when the field path curves).
    void integrateEncoders(units::Time dt) {
        const double r = 0.5 * cfg_.driveWheelDiameter.value();
        for (int i = 0; i < n_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            driveShaft_[idx] += wheelSpin_[idx].value() * dt.value() / r;  // spin, not motion
        }
        const double vx = bodyTwist_.vx().value();
        const double vy = bodyTwist_.vy().value();
        const double w = bodyTwist_.omega().value();
        for (int i = 0; i < nTracking_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            const TrackingWheelSpec& tw = tracking_[idx];
            // v_point = v_body + ω × r  (header derivation): Forward wheel reads the
            // x-component vx − ω·b; Lateral wheel reads the y-component vy + ω·a.
            const double pointVel = (tw.axis == TrackingAxis::Forward)
                                        ? (vx - w * tw.offset.value())
                                        : (vy + w * tw.offset.value());
            trackingShaft_[idx] += pointVel * dt.value() / (0.5 * tw.diameter.value());
        }
    }

    /// Push the current truth into every fake, each value through its A3 seam.
    void synthesizeSensors() {
        const units::Time now = clock_.now();

        imu_.setHeading(
            degradation_.imuHeading(math::Angle::radians(truth_.theta), now, rng_));
        imu_.setYawRate(degradation_.imuYawRate(bodyTwist_.omega(), now, rng_));
        imu_.setReady(degradation_.imuReady(true, now));

        const double r = 0.5 * cfg_.driveWheelDiameter.value();
        for (int i = 0; i < n_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            motors_[idx]->setPosition(degradation_.driveEncoderPosition(
                i, units::AngleDim{driveShaft_[idx]}, now, rng_));
            motors_[idx]->setVelocity(units::AngularVelocity{wheelSpin_[idx].value() / r});
        }
        for (int i = 0; i < nTracking_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            tracking_[idx].sensor->setPosition(degradation_.trackingEncoderPosition(
                i, units::AngleDim{trackingShaft_[idx]}, now, rng_));
        }

        const GpsTruth g = degradation_.gps(
            GpsTruth{truth_.pose(), cfg_.gpsRmsError, cfg_.gpsHasFix}, now, rng_);
        gps_.setPose(g.pose);
        gps_.setRmsError(g.rmsError);
        gps_.setHasFix(g.hasFix);

        battery_.setVoltage(degradation_.batteryVoltage(cfg_.batteryVoltage, now, rng_));
    }

    const kinematics::IKinematics& kin_;
    hal::fake::FakeClock& clock_;
    hal::fake::FakeImu& imu_;
    hal::fake::FakeGps& gps_;
    hal::fake::FakeBattery& battery_;
    DegradationModel& degradation_;
    hal::ITelemetrySink& sink_;
    DrivePlantConfig cfg_;
    MotorModel model_;
    Rng rng_;

    std::array<hal::fake::FakeMotor*, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        motors_{};
    std::array<TrackingWheelSpec, static_cast<std::size_t>(kMaxTrackingWheels)> tracking_{};
    int n_ = 0;
    int nTracking_ = 0;

    TruthState truth_{};
    math::Twist2d bodyTwist_{};
    std::array<units::Velocity, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        wheelSpin_{};
    std::array<double, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        driveShaft_{};  // cumulative radians
    std::array<double, static_cast<std::size_t>(kMaxTrackingWheels)> trackingShaft_{};  // radians
};

}  // namespace shulib::sim
