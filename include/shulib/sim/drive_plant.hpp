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
//   1. read each WHEEL's command                  (post ±12 V clamp — the F4 contract):
//        the primary's commandedVoltage() when every member of the wheel was commanded
//        the same volts, else the members' mean — wheelCommand(), the coupled-train
//        model added at R3d Part 1 (T6b); a one-member wheel is the original line
//   2. DegradationModel::effectiveVoltage()       (A3 seam; identity today)
//   3. MotorModel::advance() per wheel            (the inverted-feedforward "dynamics")
//   3b. COAST-FOLLOW (R3d Part 2F-1; OFF by default — DrivePlantConfig::coastFollow = 0):
//        a wheel whose every coupled member is at 0 V AND in BrakeMode::Coast turns at
//        coastFollow × the driven wheel's spin instead of sitting pinned — see the config
//        field; with the default the original loop runs, bit for bit
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
    /// COAST-FOLLOW, κ in [0, 1] (R3d Part 2F-1). **0 — the default — is the plant every suite
    /// before 2F-1 ran on, bit for bit:** a wheel commanded 0 V spins at 0, i.e. an unpowered wheel
    /// is PINNED, an implicit perfect brake. That made the calibrator's one-sided pulse a PIVOT about
    /// the idle side on the host, always — and on robot two the idle side COASTS, so a host test
    /// could never fail the way the robot did (the Part 2F brief, §0).
    ///
    /// With κ > 0, a wheel whose EVERY coupled member is commanded exactly 0 V AND reports
    /// BrakeMode::Coast (the plant reads the brake mode for this and for nothing else) turns at
    /// κ × the mean spin of the wheels that are NOT coasting, instead of 0. On a tank, one side
    /// driven at v with the other coasting gives ω = (1 − κ)·v / W and vx = (1 + κ)·v / 2: κ = 0 is
    /// the pinned pivot, κ = 1 is pure translation — the one-sided push ROLLS the robot and turns it
    /// not at all. A Brake/Hold side at 0 V stays pinned whatever κ is.
    ///
    /// **It is a KINEMATIC stand-in, and INVENTED (A4 register HA-137).** The plant models no mass,
    /// no inertia and no scrub (motor_model.hpp's honesty boundary), so κ is not derived from a wheel
    /// type: lateral scrub (traction wheels) pushes a real robot toward κ → 1, while a rigid body
    /// with a truly free-rolling, scrub-free idle side turns MORE than a pinned pivot (its idle side
    /// runs slightly backward — a negative κ, which this field does not allow). What κ buys the host
    /// is the FAILING regime: it lets a test drive the calibrator into "the robot rolled instead of
    /// turning", which the pinned plant could never produce. Precondition: 0 ≤ κ ≤ 1, and κ > 0
    /// only on a two-wheel (tank) kinematics — "the driven wheel's spin" means nothing on four
    /// wheels pointing four ways.
    double coastFollow = 0.0;
    /// The fraction of its train's shaft travel a MemberFault::Decoupled member's encoder still
    /// sees, in [0, 1] (R3d Part 2F-1). 0 — the default — is the fault as the Part 2F brief defines
    /// it: the member's reading is its OWN free spin and nothing else. Above 0 it also carries that
    /// fraction of the train's travel — a slipping gear that still drags its motor along when the
    /// WHEEL turns it (a hand push), while its motor cannot turn the wheel: robot two's port 18,
    /// which travels 22–33 % short on a push (A4 register HA-130). INVENTED, a sim shape only.
    double decoupledDrag = 0.0;
};

/// How one COUPLED member of a wheel misbehaves (R3b Part 1 — the hal::MotorGroup
/// detector tests). The plant writes the same synthesized shaft state into every member
/// of a wheel; a faulted member is what that state looks like through a broken port.
enum class MemberFault {
    None,         ///< healthy: the wheel's true shaft position and spin, like every mate
    Frozen,       ///< a dead port: HOLDS whatever it last reported (position AND velocity —
                  ///< ProsMotor's last-good screen), from the tick the fault is set on
    SignFlipped,  ///< a member wired the wrong way round on the READ side only: reports the
                  ///< NEGATED shaft position and spin of its mates while its command still
                  ///< reaches the train the right way (what a fight looks like to a detector).
                  ///< Its semantics are pinned by motor_group_plant_test 3–4 and the monitor
                  ///< tests and must not change — Reversed below is the NEW, separate member.
    Reversed,     ///< a motor physically WIRED BACKWARDS through a POSITIVE port (chunk R3d
                  ///< Part 1, T6a): its readings are negated exactly like SignFlipped AND its
                  ///< commanded volts are negated before the motor model, so +V drives the
                  ///< train the wrong way. This is the fault a sign-capture calibration has to
                  ///< discover; SignFlipped could not model it, because there the command
                  ///< still goes the right way.
    Decoupled,    ///< a motor whose gear SLIPS (R3d Part 2F-1 — robot two's port 18): its command
                  ///< does NOT reach the train (it is left out of wheelCommand()'s mean), and its
                  ///< encoder reports its OWN free spin under its own command — integrated through
                  ///< the plant's motor model from the tick the fault is set — plus
                  ///< DrivePlantConfig::decoupledDrag of the train's travel since then. Not passed
                  ///< through the encoder seam: a fault model, not a sensor model. Appended, like
                  ///< Reversed, so no existing value moves.
};

class DrivePlant {
public:
    static constexpr int kMaxTrackingWheels = 4;
    /// Extra coupled members a wheel may carry beyond its primary FakeMotor (robot two has
    /// five per side; a V5 brain has 21 ports).
    static constexpr int kMaxExtraMembersPerWheel = 20;

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
        SHULIB_PRECONDITION(cfg_.coastFollow >= 0.0 && cfg_.coastFollow <= 1.0,
                            "DrivePlant: coastFollow must be in [0, 1]");
        SHULIB_PRECONDITION(cfg_.coastFollow == 0.0 || n_ == 2,
                            "DrivePlant: coastFollow > 0 is defined for a two-wheel (tank) "
                            "kinematics only");
        SHULIB_PRECONDITION(cfg_.decoupledDrag >= 0.0 && cfg_.decoupledDrag <= 1.0,
                            "DrivePlant: decoupledDrag must be in [0, 1]");

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
        if (cfg_.coastFollow == 0.0) {
            // THE ORIGINAL LOOP, untouched: with coast-follow off (the default) every existing
            // scenario draws its degradation seams in exactly the order it always did.
            for (int i = 0; i < n_; ++i) {
                const auto idx = static_cast<std::size_t>(i);
                const units::Voltage commanded = wheelCommand(idx);  // the coupled-train model (T6b)
                const units::Voltage effective =
                    degradation_.effectiveVoltage(i, commanded, now, rng_);
                wheelSpin_[idx] = model_.advance(wheelSpin_[idx], effective, dt);
                motion.set(i, degradation_.wheelMotionVelocity(i, wheelSpin_[idx], now, rng_));
            }
        } else {
            stepWheelsCoastFollow(now, dt, motion);  // 3b — only when configured (header)
        }

        // 5: the TRUE body twist, via the frozen F5 contract (header: shared on purpose).
        bodyTwist_ = kin_.forward(motion);

        // 6: the TRUE pose, via RK4 — NEVER arcStep (constraint 2; truth_integrator.hpp).
        truth_ = advanceTruth(truth_, bodyTwist_, dt, cfg_.truthSubsteps);

        // 7: time. The plant is the single time authority (header).
        clock_.advance(dt);

        // 8: sensors, from truth, through the seams.
        integrateEncoders(dt);
        if (anyDecoupled_) {
            advanceDecoupled(dt);  // a slipping member's own free spin (MemberFault::Decoupled)
        }
        synthesizeSensors();

        // 9: one per-tick record — A1's lazy-build contract, so NullSink pays nothing.
        hal::emitRecord(sink_, [&] {
            diag::DebugRecord r;
            r.t = clock_.now();
            r.dt = dt;
            r.wheelCount = n_;
            for (int i = 0; i < n_; ++i) {
                const auto idx = static_cast<std::size_t>(i);
                // The COMMAND a controller issued, not wheelCommand()'s train model: the
                // record is telemetry of what the stack asked for, and under a MotorGroup's
                // fan-out (the only way the library commands a coupled side) the two are the
                // same value anyway.
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

    // ── Coupled members (R3b Part 1) ──────────────────────────────────────────────
    /// Attach EXTRA FakeMotors to wheel `wheel`: from now on every sensor synthesis writes the
    /// SAME shaft position and spin into each of them as into the wheel's primary motor —
    /// N motors on one gear train, which is physical truth for a coupled side. Since R3d
    /// Part 1 the plant reads the wheel's COMMAND from all of them through wheelCommand():
    /// the primary's value when a hal::MotorGroup has fanned one voltage to every member
    /// (the original path, bit for bit), the members' mean when they differ — so pulsing a
    /// single member of a train moves the train. One member per wheel — no call — is the default
    /// and is bit-identical to a plant that never had this method (the whole suite is the
    /// pin). Non-owning: the fakes and the array must outlive the plant. Precondition:
    /// wheel in range, count <= kMaxExtraMembersPerWheel, all non-null; may be called once
    /// per wheel.
    void attachCoupledMembers(int wheel, std::span<hal::fake::FakeMotor* const> extra) {
        SHULIB_PRECONDITION(wheel >= 0 && wheel < n_,
                            "DrivePlant::attachCoupledMembers: wheel out of range");
        SHULIB_PRECONDITION(extra.size() <= static_cast<std::size_t>(kMaxExtraMembersPerWheel),
                            "DrivePlant::attachCoupledMembers: too many members");
        const auto w = static_cast<std::size_t>(wheel);
        SHULIB_PRECONDITION(extraCount_[w] == 0,
                            "DrivePlant::attachCoupledMembers: wheel already has members");
        for (std::size_t i = 0; i < extra.size(); ++i) {
            SHULIB_PRECONDITION(extra[i] != nullptr,
                                "DrivePlant::attachCoupledMembers: a member is null");
            extra_[w][i] = extra[i];
        }
        extraCount_[w] = static_cast<int>(extra.size());
        // Seed the new members from the CURRENT truth, exactly as the constructor seeds the
        // primary, so a group formed after construction reads one consistent shaft.
        writeCoupledMembers(w, lastPosition_[w], lastVelocity_[w]);
    }

    /// Degrade one member of `wheel` from the NEXT synthesis on: `member` 0 is the primary
    /// FakeMotor handed to the constructor, 1.. are the attached members in order. A Frozen
    /// member keeps the values it holds NOW (freeze it at rest and it reports 0 rad/s while
    /// its mates turn — the frozen-encoder signature the group's median must ignore and its
    /// monitor must count); a SignFlipped member reports its mates' state negated. A Decoupled
    /// member starts from the reading it holds NOW, and from the next step reports that plus its
    /// own free spin plus decoupledDrag of the train's travel (MemberFault::Decoupled).
    void setMemberFault(int wheel, int member, MemberFault fault) {
        SHULIB_PRECONDITION(wheel >= 0 && wheel < n_, "DrivePlant::setMemberFault: wheel out of range");
        const auto w = static_cast<std::size_t>(wheel);
        SHULIB_PRECONDITION(member >= 0 && member <= extraCount_[w],
                            "DrivePlant::setMemberFault: member out of range");
        const auto m = static_cast<std::size_t>(member);
        if (fault == MemberFault::Decoupled && memberFault_[w][m] != MemberFault::Decoupled) {
            // Anchor the slip where the member stands: its reading does not jump when its gear
            // lets go, it simply stops following the train from here on.
            const hal::fake::FakeMotor& fm = (m == 0) ? *motors_[w] : *extra_[w][m - 1];
            decoupledAnchorPos_[w][m] = fm.position().value();
            decoupledAnchorTrain_[w][m] = driveShaft_[w];
            decoupledOwnShaft_[w][m] = 0.0;
            decoupledOwnSpin_[w][m] = 0.0;
            anyDecoupled_ = true;
        }
        memberFault_[w][m] = fault;
    }

private:
    /// Write one member's reading under its fault (header of MemberFault).
    static void writeMember(hal::fake::FakeMotor& m, MemberFault fault, units::AngleDim pos,
                            units::AngularVelocity vel) {
        switch (fault) {
            case MemberFault::None:
                m.setPosition(pos);
                m.setVelocity(vel);
                return;
            case MemberFault::Frozen:
                return;  // holds its last reading, like a dead port through ProsMotor
            case MemberFault::SignFlipped:
            case MemberFault::Reversed:
                // Both report the negated shaft: from the READ side a motor wired
                // backwards and a member fighting its mates look exactly alike. They
                // differ only on the COMMAND side (effectiveMemberVolts).
                m.setPosition(units::AngleDim{-pos.value()});
                m.setVelocity(units::AngularVelocity{-vel.value()});
                return;
            case MemberFault::Decoupled:
                // Never reached: a Decoupled member's reading needs the plant's own state and
                // is written by writeDecoupled(); every caller routes it there first.
                return;
        }
    }

    /// Write member `m` of wheel `w` under MemberFault::Decoupled: where it stood when its gear
    /// let go, plus decoupledDrag of the train's travel since then, plus its own free spin.
    /// `trainVel` is the wheel's synthesized shaft rate this tick.
    void writeDecoupled(std::size_t w, std::size_t m, units::AngularVelocity trainVel) {
        hal::fake::FakeMotor& fm = (m == 0) ? *motors_[w] : *extra_[w][m - 1];
        const double r = 0.5 * cfg_.driveWheelDiameter.value();
        fm.setPosition(units::AngleDim{decoupledAnchorPos_[w][m]
                                       + cfg_.decoupledDrag
                                             * (driveShaft_[w] - decoupledAnchorTrain_[w][m])
                                       + decoupledOwnShaft_[w][m]});
        fm.setVelocity(units::AngularVelocity{cfg_.decoupledDrag * trainVel.value()
                                              + decoupledOwnSpin_[w][m] / r});
    }

    /// True when member `m` of wheel `w` is Decoupled.
    [[nodiscard]] bool isDecoupled(std::size_t w, std::size_t m) const noexcept {
        return memberFault_[w][m] == MemberFault::Decoupled;
    }

    /// Advance every Decoupled member's OWN free spin by one step, from its own command, through
    /// the plant's motor model (the same inverted feedforward a wheel turns under — torque-free,
    /// like everything here; a motor spinning free of its train is not claimed to be faster).
    void advanceDecoupled(units::Time dt) {
        const double r = 0.5 * cfg_.driveWheelDiameter.value();
        for (int i = 0; i < n_; ++i) {
            const auto w = static_cast<std::size_t>(i);
            for (int k = 0; k <= extraCount_[w]; ++k) {
                const auto m = static_cast<std::size_t>(k);
                if (!isDecoupled(w, m)) {
                    continue;
                }
                const hal::fake::FakeMotor& fm = (m == 0) ? *motors_[w] : *extra_[w][m - 1];
                decoupledOwnSpin_[w][m] = model_
                                              .advance(units::Velocity{decoupledOwnSpin_[w][m]},
                                                       fm.commandedVoltage(), dt)
                                              .value();
                decoupledOwnShaft_[w][m] += decoupledOwnSpin_[w][m] * dt.value() / r;
            }
        }
    }

    /// True when wheel `w` is COASTING in the coast-follow sense: every member whose command
    /// reaches the train is at exactly 0 V and reports BrakeMode::Coast. A wheel with no coupled
    /// member left (every one Decoupled) is unpowered too, and coasts if they all coast.
    [[nodiscard]] bool wheelCoasting(std::size_t w) const {
        for (int k = 0; k <= extraCount_[w]; ++k) {
            const auto m = static_cast<std::size_t>(k);
            if (isDecoupled(w, m)) {
                continue;  // its command never reaches the train (MemberFault::Decoupled)
            }
            const hal::fake::FakeMotor& fm = (m == 0) ? *motors_[w] : *extra_[w][m - 1];
            if (fm.commandedVoltage().value() != 0.0 || fm.brakeMode() != hal::BrakeMode::Coast) {
                return false;
            }
        }
        return true;
    }

    /// Steps 1–4 with coast-follow on (DrivePlantConfig::coastFollow > 0). Three passes instead
    /// of the original interleaved loop, because a coasting wheel's spin depends on the OTHER
    /// wheel's: every wheel's command → spin first, then the coasting wheels follow, then the
    /// motion seam. The seams are therefore drawn in a different ORDER from the original loop —
    /// which is why this is a separate path that only a κ > 0 configuration ever takes.
    void stepWheelsCoastFollow(units::Time now, units::Time dt, kinematics::WheelSpeeds& motion) {
        bool coasting[static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)]{};
        double drivenSum = 0.0;
        int driven = 0;
        for (int i = 0; i < n_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            const units::Voltage commanded = wheelCommand(idx);
            const units::Voltage effective =
                degradation_.effectiveVoltage(i, commanded, now, rng_);
            wheelSpin_[idx] = model_.advance(wheelSpin_[idx], effective, dt);
            coasting[idx] = wheelCoasting(idx);
            if (!coasting[idx]) {
                drivenSum += wheelSpin_[idx].value();
                ++driven;
            }
        }
        if (driven > 0) {
            const double follow = cfg_.coastFollow * drivenSum / static_cast<double>(driven);
            for (int i = 0; i < n_; ++i) {
                const auto idx = static_cast<std::size_t>(i);
                if (coasting[idx]) {
                    wheelSpin_[idx] = units::Velocity{follow};
                }
            }
        }
        for (int i = 0; i < n_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            motion.set(i, degradation_.wheelMotionVelocity(i, wheelSpin_[idx], now, rng_));
        }
    }

    /// The volts a member's command actually delivers to the train: a Reversed member is
    /// wired backwards, so its commanded volts turn the gear train the other way. Applied to
    /// EVERY member wherever its command is read (the primary on the equal-command path, and
    /// every member inside the mean) — the negation belongs to the member's wiring, not to
    /// its position in the array. Identity for every other fault, which is what keeps the
    /// pre-existing suite bit-identical.
    [[nodiscard]] static double effectiveMemberVolts(double commanded, MemberFault fault) noexcept {
        return fault == MemberFault::Reversed ? -commanded : commanded;
    }

    /// The volts wheel `w` actually turns under — the COUPLED-TRAIN COMMAND MODEL (R3d
    /// Part 1, T6b). Until R3d the plant read a wheel's command from its PRIMARY member
    /// alone, so a calibrator pulsing ONE extra member of a five-motor train would have
    /// moved nothing at all and its test would have passed for the wrong reason.
    ///
    /// The rule, and why it is shaped this way:
    ///   * if every member of the wheel is commanded the SAME volts — EXACT comparison, which
    ///     is what a hal::MotorGroup's fan-out always produces — the wheel's command is the
    ///     PRIMARY's value: the pre-R3d path, bit for bit, which is why
    ///     motor_group_plant_test 7's five-member equivalence pin survives unchanged;
    ///   * otherwise the wheel turns at the MEAN of its members' effective volts — a
    ///     velocity-driven model of a train pulled by some of its motors while the rest
    ///     free-wheel. It is TORQUE-FREE like everything else here (motor_model.hpp's
    ///     honesty boundary): it does not claim what force five motors exert on one shaft,
    ///     only that the train follows the average of what it is being asked for. That makes it
    ///     OPTIMISTIC — a real motor back-driving four gearboxes reaches less than 1/N — so it is
    ///     a registered SIM belief, settled by R6's back-fit (A4 register HA-137).
    ///
    /// A MemberFault::Decoupled member's command does not reach the train (R3d Part 2F-1): when
    /// a wheel has one, its command is taken over the COUPLED members only — their common value
    /// when they agree, else their mean — and 0 V when none is left. A wheel with no Decoupled
    /// member never enters that branch, so every pre-2F-1 path is the one below, unchanged.
    [[nodiscard]] units::Voltage wheelCommand(std::size_t w) const {
        if (anyDecoupled_) {
            for (int k = 0; k <= extraCount_[w]; ++k) {
                if (isDecoupled(w, static_cast<std::size_t>(k))) {
                    return coupledOnlyCommand(w);
                }
            }
        }
        const double primary = motors_[w]->commandedVoltage().value();
        const int extras = extraCount_[w];
        bool allSame = true;
        for (int k = 0; k < extras; ++k) {
            if (extra_[w][static_cast<std::size_t>(k)]->commandedVoltage().value() != primary) {
                allSame = false;  // exact compare, on purpose (header)
                break;
            }
        }
        if (allSame) {
            return units::Voltage{effectiveMemberVolts(primary, memberFault_[w][0])};
        }
        double sum = effectiveMemberVolts(primary, memberFault_[w][0]);
        for (int k = 0; k < extras; ++k) {
            const auto kIdx = static_cast<std::size_t>(k);
            sum += effectiveMemberVolts(extra_[w][kIdx]->commandedVoltage().value(),
                                        memberFault_[w][kIdx + 1]);
        }
        return units::Voltage{sum / static_cast<double>(extras + 1)};
    }

    /// wheelCommand() for a wheel that has at least one Decoupled member: the same rule over the
    /// members whose command still reaches the train.
    [[nodiscard]] units::Voltage coupledOnlyCommand(std::size_t w) const {
        double first = 0.0;
        double sum = 0.0;
        int count = 0;
        bool allSame = true;
        for (int k = 0; k <= extraCount_[w]; ++k) {
            const auto m = static_cast<std::size_t>(k);
            if (isDecoupled(w, m)) {
                continue;
            }
            const hal::fake::FakeMotor& fm = (m == 0) ? *motors_[w] : *extra_[w][m - 1];
            const double v = effectiveMemberVolts(fm.commandedVoltage().value(), memberFault_[w][m]);
            if (count == 0) {
                first = v;
            } else if (v != first) {
                allSame = false;  // exact compare, as wheelCommand()
            }
            sum += v;
            ++count;
        }
        if (count == 0) {
            return units::Voltage{0.0};  // no member drives this train at all
        }
        return units::Voltage{allSame ? first : sum / static_cast<double>(count)};
    }

    /// The attached members of wheel `w` (never the primary — its write is the original
    /// synthesis line, kept as it was for bit-identity).
    void writeCoupledMembers(std::size_t w, units::AngleDim pos, units::AngularVelocity vel) {
        for (int i = 0; i < extraCount_[w]; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            if (isDecoupled(w, idx + 1)) {
                writeDecoupled(w, idx + 1, vel);
                continue;
            }
            writeMember(*extra_[w][idx], memberFault_[w][idx + 1], pos, vel);
        }
    }
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
            const units::AngleDim pos = degradation_.driveEncoderPosition(
                i, units::AngleDim{driveShaft_[idx]}, now, rng_);
            const units::AngularVelocity vel{wheelSpin_[idx].value() / r};
            // The primary: the ORIGINAL two writes (bit-identical when unfaulted), then the
            // coupled members, each under its own fault (R3b Part 1).
            if (memberFault_[idx][0] == MemberFault::None) {
                motors_[idx]->setPosition(pos);
                motors_[idx]->setVelocity(vel);
            } else if (memberFault_[idx][0] == MemberFault::Decoupled) {
                writeDecoupled(idx, 0, vel);
            } else {
                writeMember(*motors_[idx], memberFault_[idx][0], pos, vel);
            }
            lastPosition_[idx] = pos;
            lastVelocity_[idx] = vel;
            writeCoupledMembers(idx, pos, vel);
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

    // Coupled members per wheel (R3b Part 1): the attached fakes, how many, each member's
    // fault (index 0 = the primary), and the last synthesized reading so a late attach seeds
    // its members consistently.
    std::array<std::array<hal::fake::FakeMotor*, static_cast<std::size_t>(kMaxExtraMembersPerWheel)>,
               static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        extra_{};
    std::array<int, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)> extraCount_{};
    std::array<std::array<MemberFault, static_cast<std::size_t>(kMaxExtraMembersPerWheel) + 1>,
               static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        memberFault_{};
    std::array<units::AngleDim, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        lastPosition_{};
    std::array<units::AngularVelocity, static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>
        lastVelocity_{};

    // MemberFault::Decoupled state (R3d Part 2F-1), per wheel per member (index 0 = primary):
    // the reading and the train's raw shaft when the gear let go, and the member's own free
    // spin since (surface in/s, and its integral in shaft radians). `anyDecoupled_` keeps every
    // plant that never sets the fault on the original code paths.
    using PerMember = std::array<std::array<double, static_cast<std::size_t>(kMaxExtraMembersPerWheel) + 1>,
                                 static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels)>;
    PerMember decoupledAnchorPos_{};
    PerMember decoupledAnchorTrain_{};
    PerMember decoupledOwnShaft_{};
    PerMember decoupledOwnSpin_{};
    bool anyDecoupled_ = false;
};

}  // namespace shulib::sim
