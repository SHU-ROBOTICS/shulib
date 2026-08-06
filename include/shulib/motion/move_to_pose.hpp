#pragma once
//
// MoveToPose — decoupled per-axis field-pose motion (chunk C1). THE HOLONOMIC
// THESIS, as code: three INDEPENDENT controllers — field-x, field-y, heading —
// each closing its own loop every tick, combined into one ChassisSpeeds. The
// robot translates and rotates SIMULTANEOUSLY and INDEPENDENTLY; there is no
// turn-then-drive sequencing anywhere in this file, and the simultaneity test
// pins that a diagonal move with a heading change happens as ONE motion. (That
// sequencing is precisely LemLib's tank-coupled behaviour this project exists
// to beat; C3's H-drive strafe FALLBACK is the only sanctioned exception, and
// it is telemetry-visible by contract.)
//
// ── The per-tick pipeline (frames and clamps annotated — the F1/F5 choreography) ────
//   1. pose  = Localizer::pose()                                  [FIELD]
//   2. errors: ex = tx−px, ey = ty−py (in); eh = heading.errorTo(target)
//      — eh is F3's SHORTEST signed error, so the ±180° seam is absorbed
//      BEFORE any controller sees a number.                       [FIELD/rad]
//   3. three decoupled PIDs → (vx, vy) in/s, ω rad/s              [FIELD]
//      heading PID is fed (setpoint 0, measurement −eh): the error the PID
//      differentiates is CONTINUOUS near the target (no wrap in its input),
//      and D-on-measurement stays live for a future kD.
//   4. |ω| clamp; (vx,vy) NORM-capped uniformly (direction preserved).
//   5. fieldToRobot(field, heading)          ← THE one F1 rotation [BODY]
//   6. body |vy| clamped to strafeAuthority()·maxLinearSpeed — the UPSTREAM
//      clamp §13 #5 assigns to THIS layer; strafeAuthority() is read-only and
//      toWheels() never clamps (F5).
//   7. toWheels → desaturate(maxWheelSpeed)   — the DOWNSTREAM uniform scale.
//   8. Feedforward → compensateForBattery → IMotor::setVoltage    [volts]
//   9. OdoStallCheck + HealthMonitor::tick — the A3 containment wiring.
//  10. settle/watchdog verdict; one lazy DebugRecord (A1 contract).
//
// ── Exit logic ──────────────────────────────────────────────────────────────────────
// Arrival needs BOTH a translation criterion (‖(ex,ey)‖) and a heading
// criterion (|eh|) — so this class composes TWO SettledUtils + ONE Watchdog
// rather than the 1-scalar ExitGroup (same tested primitives, 2+1 composition;
// ExitGroup's Settled-beats-simultaneous-TimedOut priority is preserved).
// TurnTo / DriveBrake, whose exit is one scalar, use ExitGroup unchanged.
// On the exit tick the motors are stopped BEFORE the record is emitted, so the
// record stream shows the true final state (zero command, exit state).
//
// ── Sibling shaping (PoseMotionOptions) ─────────────────────────────────────────────
// StrafeTo and HoldPose are this same engine with capture/exit options — the
// three share one pipeline so a fix lands once. Capture-at-first-LIVE-tick is
// part of the wait-for-live contract (motion.hpp): an estimate-derived target
// must never be read during the boot window.
//
// Gains/tolerances: MotionConfig — every default provisional until R5 (HA-50/51/52).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "shulib/control/feedforward.hpp"
#include "shulib/control/pid.hpp"
#include "shulib/control/settled_util.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// Internal shaping knobs for the sibling primitives (StrafeTo / HoldPose).
/// Not part of MoveToPose's public construction surface.
struct PoseMotionOptions {
    bool captureHeadingAtLive = false;  ///< StrafeTo: hold the first-live heading
    bool capturePoseAtLive = false;     ///< HoldPose: hold the first-live pose
    double holdFor = 0.0;               ///< > 0 ⇒ hold-mode exit (HoldPose)
};

class MoveToPose : public IMotion {
public:
    /// Drive to `target` (FIELD frame). `timeout` seconds bounds the whole
    /// motion INCLUDING any boot wait; 0 selects config.defaultTimeout.
    MoveToPose(const MotionDeps& deps, const math::Pose2d& target,
               const MotionConfig& config = {}, double timeout = 0.0)
        : MoveToPose(deps, target, config, timeout, PoseMotionOptions{}) {}

    void start() override {
        pidX_.reset();
        pidY_.reset();
        pidH_.reset();
        settledTrans_.reset();
        settledHead_.reset();
        stall_.reset();
        watchdog_.start();
        reason_ = control::ExitReason::Running;
        state_ = MotionState::WaitingForEstimate;
        everLive_ = false;
        captured_ = !(opts_.captureHeadingAtLive || opts_.capturePoseAtLive);
        holdStart_ = 0.0;
        hasTick_ = false;
        lastTickTime_ = 0.0;
    }

    [[nodiscard]] control::ExitReason tick() override {
        SHULIB_PRECONDITION(state_ != MotionState::Idle, "MoveToPose::tick: start() not called");
        if (reason_ != control::ExitReason::Running) {
            return reason_;  // finished — a safe no-op (motors already stopped)
        }
        chassis::RobotContext& ctx = *deps_.ctx;
        localization::Localizer& loc = *deps_.localizer;
        const units::Time now = ctx.clock().now();
        const units::Time dt = measuredDt(now);

        // ── the wait-for-live gate (motion.hpp contract) ──────────────────────
        const bool live =
            loc.qualityClass() != localization::Localizer::Quality::Uninitialized;
        if (!live && !everLive_) {
            state_ = MotionState::WaitingForEstimate;
            stopMotors();  // acts on NO estimate: zero output, every wait tick
            if (watchdog_.expired()) {
                return exitTimedOut("timed out waiting for a live estimate");
            }
            emitWaitingRecord(now, dt);
            return control::ExitReason::Running;
        }
        if (!everLive_) {  // ── the first LIVE tick: capture estimate-derived targets
            everLive_ = true;
            holdStart_ = now.value();
            if (!captured_) {
                const math::Pose2d p = loc.pose();
                if (opts_.capturePoseAtLive) {
                    target_ = p;
                } else if (opts_.captureHeadingAtLive) {
                    target_ = math::Pose2d{target_.x(), target_.y(), p.heading()};
                }
                captured_ = true;
            }
        }
        state_ = MotionState::Running;

        // ── errors (FIELD frame; heading via F3's shortest signed error) ──────
        const math::Pose2d pose = loc.pose();
        const double errX = target_.x().value() - pose.x().value();       // in
        const double errY = target_.y().value() - pose.y().value();       // in
        const double errH = pose.heading().errorTo(target_.heading());    // rad

        // ── exit verdict FIRST (Settled beats a simultaneous TimedOut) ────────
        const bool transSettled = settledTrans_.update(std::hypot(errX, errY));
        const bool headSettled = settledHead_.update(errH);
        if (opts_.holdFor > 0.0) {
            if ((now.value() - holdStart_) >= opts_.holdFor) {
                // Held the clock out — but success still requires actually being
                // on target NOW (a hold that ends 10 in off must not report it).
                if (transSettled && headSettled) {
                    return exitSettled(now, dt, pose, errX, errY, errH);
                }
                return exitTimedOut("hold expired off-target");
            }
        } else {
            if (transSettled && headSettled) {
                return exitSettled(now, dt, pose, errX, errY, errH);
            }
            if (watchdog_.expired()) {
                return exitTimedOut("target not reached");
            }
        }

        // ── the three DECOUPLED per-axis controllers (FIELD frame) ────────────
        double vxF = pidX_.update(target_.x().value(), pose.x().value());  // in/s
        double vyF = pidY_.update(target_.y().value(), pose.y().value());  // in/s
        double w = pidH_.update(0.0, -errH);                               // rad/s

        // ── saturation policy (MotionConfig header) ───────────────────────────
        const double maxLin = cfg_.maxLinearSpeed.value();
        w = std::clamp(w, -cfg_.maxAngularSpeed.value(), cfg_.maxAngularSpeed.value());
        const double norm = std::hypot(vxF, vyF);
        if (norm > maxLin) {
            const double s = maxLin / norm;  // uniform: direction preserved
            vxF *= s;
            vyF *= s;
        }

        // ── FIELD → BODY: the ONE frame rotation (F1) ─────────────────────────
        const math::ChassisSpeeds fieldCmd{units::Velocity{vxF}, units::Velocity{vyF},
                                           units::AngularVelocity{w}};
        const math::ChassisSpeeds body = math::fieldToRobot(fieldCmd, pose.heading());

        // ── strafe-authority clamp: THIS layer clamps; kinematics never (F5) ──
        const double vyLimit = deps_.kinematics->strafeAuthority() * maxLin;
        const math::ChassisSpeeds bodyClamped{
            body.vx(),
            units::Velocity{std::clamp(body.vy().value(), -vyLimit, vyLimit)},
            body.omega()};

        // ── wheels: unclamped inverse kinematics, then the uniform desaturate ─
        kinematics::WheelSpeeds wheels = deps_.kinematics->toWheels(bodyClamped);
        wheels = deps_.kinematics->desaturate(wheels, cfg_.maxWheelSpeed);

        // ── volts: feedforward, then the battery ceiling, per wheel ───────────
        const units::Voltage vb = ctx.battery().voltage();
        const auto motors = ctx.driveMotors();
        for (int i = 0; i < wheels.size(); ++i) {
            const control::CompensatedVoltage cv =
                control::compensateForBattery(ff_.calculate(wheels[i]), vb);
            motors[static_cast<std::size_t>(i)]->setVoltage(cv.voltage);
        }

        // ── A3 containment: stall cross-check + health observables ────────────
        const bool stalled = stall_.update(now, motors, pose);
        tickHealth(ctx, loc, stalled, vb);

        // ── one lazy record (A1): commanded = FINAL achievable cmd, FIELD frame
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r = baseRecord(ctx, loc, now, dt, pose, errX, errY, errH);
            r.commanded = math::robotToField(bodyClamped, pose.heading());
            for (int i = 0; i < wheels.size(); ++i) {
                const auto idx = static_cast<std::size_t>(i);
                r.wheelVoltage[idx] = motors[idx]->commandedVoltage();
                r.wheelCurrent[idx] = motors[idx]->current();
            }
            return r;
        });
        return control::ExitReason::Running;
    }

    /// The cancel contract (motion.hpp): safe state whenever started, verdict
    /// only if still running, Idle untouched, idempotent, never raises.
    void cancel() override {
        if (state_ == MotionState::Idle) {
            return;  // never started: no relationship to the drivetrain yet
        }
        applyCancelSafeState(*deps_.ctx);
        if (reason_ != control::ExitReason::Running) {
            return;  // already exited: verdict preserved (motion.hpp rationale)
        }
        const bool wasWaiting = (state_ == MotionState::WaitingForEstimate);
        reason_ = control::ExitReason::Cancelled;
        state_ = MotionState::Cancelled;
        const units::Time now = deps_.ctx->clock().now();
        const math::Pose2d pose = deps_.localizer->pose();
        if (wasWaiting) {
            // Cancelled during the boot window: there was never a live estimate
            // and (for capture-at-live siblings) possibly no captured target —
            // like the waiting record, the exit record must not invent errors.
            emitExitRecord(now, units::Time{0.0}, pose, 0.0, 0.0, 0.0);
            return;
        }
        const double errX = target_.x().value() - pose.x().value();
        const double errY = target_.y().value() - pose.y().value();
        const double errH = pose.heading().errorTo(target_.heading());
        emitExitRecord(now, units::Time{0.0}, pose, errX, errY, errH);
    }

    [[nodiscard]] control::ExitReason exitReason() const noexcept override { return reason_; }
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    [[nodiscard]] const char* name() const noexcept override { return "MoveToPose"; }

    /// The FIELD-frame target (after any first-live-tick capture).
    [[nodiscard]] const math::Pose2d& target() const noexcept { return target_; }

    /// Retarget BEFORE start() (rebuilding a motion for a new waypoint).
    /// Precondition: not currently running.
    void setTarget(const math::Pose2d& target) {
        SHULIB_PRECONDITION(reason_ != control::ExitReason::Running
                                || state_ == MotionState::Idle,
                            "MoveToPose::setTarget: motion is running");
        target_ = target;
    }

protected:
    /// The shaping constructor the siblings use (see PoseMotionOptions).
    MoveToPose(const MotionDeps& deps, const math::Pose2d& target, const MotionConfig& config,
               double timeout, const PoseMotionOptions& options)
        : deps_{deps},
          cfg_{config},
          opts_{options},
          target_{target},
          pidX_{pidConfig(config.translation), deps.validatedClock()},
          pidY_{pidConfig(config.translation), deps.ctx->clock()},
          pidH_{pidConfig(config.heading), deps.ctx->clock()},
          ff_{config.wheelFf},
          settledTrans_{config.translationSettle, deps.ctx->clock()},
          settledHead_{config.headingSettle, deps.ctx->clock()},
          watchdog_{options.holdFor > 0.0
                        ? options.holdFor + kHoldSlack
                        : (timeout > 0.0 ? timeout : config.defaultTimeout),
                    deps.ctx->clock()},
          stall_{config.stall} {
        cfg_.validate();
        SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0,
                            "MoveToPose: timeout must be finite and >= 0");
    }

    /// The hold watchdog only backstops a clock pathology; hold-mode's own
    /// deadline (holdFor) is the real exit.
    static constexpr double kHoldSlack = 1.0;

    [[nodiscard]] static control::PidConfig pidConfig(const AxisGains& g) noexcept {
        // Output saturation deliberately unlimited here: the layer's norm/ω caps
        // own it (a per-axis clamp before the norm cap would distort direction).
        return control::PidConfig{.kP = g.kP, .kI = g.kI, .kD = g.kD,
                                  .integralLimit = g.integralLimit};
    }

    [[nodiscard]] units::Time measuredDt(units::Time now) noexcept {
        const double dt = hasTick_ ? (now.value() - lastTickTime_) : 0.0;
        lastTickTime_ = now.value();
        hasTick_ = true;
        return units::Time{dt};
    }

    void stopMotors() {
        for (hal::IMotor* m : deps_.ctx->driveMotors()) {
            m->setVoltage(units::Voltage{0.0});
        }
    }

    void tickHealth(chassis::RobotContext& ctx, localization::Localizer& loc, bool stalled,
                    units::Voltage batteryVolts) {
        double maxTemp = 0.0;
        for (const hal::IMotor* m : ctx.driveMotors()) {
            maxTemp = std::max(maxTemp, m->temperature());
        }
        deps_.health->tick({.imuReady = ctx.imu().isReady(),
                            .odomImplausible = loc.lastOdomDeltaImplausible(),
                            .odomStalled = stalled,
                            .fixGated = loc.lastCorrection().gated,
                            .batteryVolts = batteryVolts,
                            .maxMotorTempC = maxTemp});
    }

    [[nodiscard]] diag::DebugRecord baseRecord(chassis::RobotContext& ctx,
                                               const localization::Localizer& loc,
                                               units::Time now, units::Time dt,
                                               const math::Pose2d& pose, double errX,
                                               double errY, double errH) const {
        diag::DebugRecord r;
        r.t = now;
        r.dt = dt;
        r.targetPose = target_;
        r.measuredPose = pose;
        r.errorX = units::Length{errX};
        r.errorY = units::Length{errY};
        r.errorHeading = units::AngleDim{errH};
        r.wheelCount = deps_.kinematics->wheelCount();
        r.imuYaw = ctx.imu().heading();
        r.imuYawRate = ctx.imu().yawRate();
        r.activeCommandState = static_cast<std::uint8_t>(state_);
        r.deadReckoning = loc.isDeadReckoning();
        r.qualityClass = static_cast<std::uint8_t>(loc.qualityClass());
        r.quality = loc.quality();
        r.correctionDx = loc.lastCorrection().dx;
        r.correctionDy = loc.lastCorrection().dy;
        r.clampedThisTick = loc.lastCorrection().clamped;
        r.batteryVoltage = ctx.battery().voltage();
        return r;
    }

    void emitWaitingRecord(units::Time now, units::Time dt) {
        chassis::RobotContext& ctx = *deps_.ctx;
        const localization::Localizer& loc = *deps_.localizer;
        hal::emitRecord(ctx.telemetry(), [&] {
            const math::Pose2d pose = loc.pose();
            // Errors/commands deliberately zero: there IS no estimate to err
            // against, and nothing was commanded — the record must not invent.
            diag::DebugRecord r = baseRecord(ctx, loc, now, dt, pose, 0.0, 0.0, 0.0);
            r.commanded = math::ChassisSpeeds{};
            return r;
        });
    }

    control::ExitReason exitSettled(units::Time now, units::Time dt, const math::Pose2d& pose,
                                    double errX, double errY, double errH) {
        stopMotors();
        reason_ = control::ExitReason::Settled;
        state_ = MotionState::Settled;
        emitExitRecord(now, dt, pose, errX, errY, errH);
        return reason_;
    }

    control::ExitReason exitTimedOut(const char* why) {
        stopMotors();
        reason_ = control::ExitReason::TimedOut;
        state_ = MotionState::TimedOut;
        deps_.faults->raise(diag::FaultCode::MotionTimeout, name(), why);
        const units::Time now = deps_.ctx->clock().now();
        const math::Pose2d pose = deps_.localizer->pose();
        const double errX = target_.x().value() - pose.x().value();
        const double errY = target_.y().value() - pose.y().value();
        const double errH = pose.heading().errorTo(target_.heading());
        emitExitRecord(now, units::Time{0.0}, pose, errX, errY, errH);
        return reason_;
    }

    void emitExitRecord(units::Time now, units::Time dt, const math::Pose2d& pose, double errX,
                        double errY, double errH) {
        chassis::RobotContext& ctx = *deps_.ctx;
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r =
                baseRecord(ctx, *deps_.localizer, now, dt, pose, errX, errY, errH);
            r.commanded = math::ChassisSpeeds{};  // motors are stopped on exit
            return r;
        });
    }

    MotionDeps deps_;
    MotionConfig cfg_;
    PoseMotionOptions opts_;
    math::Pose2d target_;
    control::Pid pidX_;  // FIELD x: inches → in/s
    control::Pid pidY_;  // FIELD y: inches → in/s
    control::Pid pidH_;  // heading: radians → rad/s
    control::Feedforward ff_;
    control::SettledUtil settledTrans_;
    control::SettledUtil settledHead_;
    control::Watchdog watchdog_;
    OdoStallCheck stall_;
    control::ExitReason reason_ = control::ExitReason::Running;
    MotionState state_ = MotionState::Idle;
    bool everLive_ = false;
    bool captured_ = true;
    double holdStart_ = 0.0;
    double lastTickTime_ = 0.0;
    bool hasTick_ = false;
};

}  // namespace shulib::motion
