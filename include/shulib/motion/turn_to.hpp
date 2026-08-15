#pragma once
//
// TurnTo — rotate in place to a FIELD heading (chunk C1).
//
// A pure heading loop: ω from the heading PID, body vx = vy = 0. The error fed
// to the controller AND to the exit logic is math::Angle::errorTo — F3's
// shortest signed rotation in (-π, π], with the exact-antipodal case resolving
// to +π deterministically. That single choice is what makes the ±180° seam
// safe: a target 350° "away" is an error of −10°, a target at exactly ±180°
// turns CCW (+π) every time, and no raw θ difference ever reaches a gain.
//
// Exit is the unmodified 1-scalar ExitGroup (SettledUtil on |errH| + Watchdog)
// — constraint 6's plumbing, reused as-is. Everything else (wait-for-live gate,
// stall/health wiring, record emission, F5 output pipeline) matches
// MoveToPose's documented pipeline; the two share idioms deliberately so the
// C4 facade inherits one shape.
//
// TANK NOTE: rotation is achievable on every supported drive (tank included),
// so TurnTo is the one C1 translation-free primitive with no authority caveat.
// The stall cross-check's rotation term (odo_stall_check.hpp) makes a pure turn
// immune to false ODO_STUCK — pinned by test.

#include <cmath>

#include "shulib/control/exit_group.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/control/pid.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/command_pipeline.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// Rotate IN PLACE to a FIELD heading: ω from the heading PID, body vx = vy = 0. Both the
/// controller and the exit test are fed math::Angle::errorTo — the shortest signed rotation in
/// (-π, π], with an exact antipode resolving to +π every time — so a target 350° "away" is a 10°
/// error the other way and no raw θ difference ever reaches a gain. The one translation-free
/// primitive with NO drivetrain-authority caveat: every supported drive can rotate, tank included,
/// and the stall cross-check's rotation term keeps a pure turn from reading as ODO_STUCK.
class TurnTo final : public IMotion {
public:
    /// Rotate to `target` (FIELD heading). `timeout` (s) bounds the whole
    /// motion including any boot wait; 0 selects config.defaultTimeout.
    TurnTo(const MotionDeps& deps, math::Angle target, const MotionConfig& config = {},
           double timeout = 0.0)
        : deps_{deps},
          cfg_{config},
          target_{target},
          pidH_{control::PidConfig{.kP = config.heading.kP, .kI = config.heading.kI,
                                   .kD = config.heading.kD,
                                   .integralLimit = config.heading.integralLimit},
                deps.validatedClock()},
          ff_{config.wheelFf},
          exit_{config.headingSettle, timeout > 0.0 ? timeout : config.defaultTimeout,
                deps.ctx->clock()},
          stall_{config.stall} {
        cfg_.validate();
        SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0,
                            "TurnTo: timeout must be finite and >= 0");
    }

    /// Arm, or fully re-arm, the turn: PID, stall detector, settle state and watchdog all reset,
    /// and the motion re-enters the boot wait. A finished TurnTo is reusable this way — but it is
    /// never re-AIMED, since target() is fixed at construction and start() reads nothing from the
    /// estimator.
    void start() override {
        pidH_.reset();
        stall_.reset();
        exit_.start();
        reason_ = control::ExitReason::Running;
        state_ = MotionState::WaitingForEstimate;
        everLive_ = false;
        hasTick_ = false;
        lastTickTime_ = 0.0;
    }

    /// One control tick, emitting one DebugRecord. Precondition: start() was called, and the
    /// caller must have updated the Localizer FIRST — this reads the estimate, it does not advance
    /// it. While quality is still Uninitialized it commands zero volts and makes no settle
    /// progress, but the WATCHDOG RUNS THROUGH THAT WAIT, so a never-live estimate exits TimedOut
    /// (raising MOTION_TIMEOUT) rather than hanging. Settled beats a simultaneous timeout. Once a
    /// non-Running verdict is returned the motion is finished: further calls are no-ops that
    /// return the cached verdict and leave the motors stopped.
    [[nodiscard]] control::ExitReason tick() override {
        SHULIB_PRECONDITION(state_ != MotionState::Idle, "TurnTo::tick: start() not called");
        if (reason_ != control::ExitReason::Running) {
            return reason_;
        }
        chassis::RobotContext& ctx = *deps_.ctx;
        localization::Localizer& loc = *deps_.localizer;
        const units::Time now = ctx.clock().now();
        const double dtRaw = hasTick_ ? (now.value() - lastTickTime_) : 0.0;
        lastTickTime_ = now.value();
        hasTick_ = true;
        const units::Time dt{dtRaw};

        // wait-for-live gate (motion.hpp contract; watchdog runs through it)
        const bool live =
            loc.qualityClass() != localization::Localizer::Quality::Uninitialized;
        if (!live && !everLive_) {
            stopMotors();
            if (exit_.watchdog().expired()) {
                return exitTimedOut("timed out waiting for a live estimate");
            }
            emitRecordFor(now, dt, loc.pose(), 0.0, math::ChassisSpeeds{});
            return control::ExitReason::Running;
        }
        everLive_ = true;
        state_ = MotionState::Running;

        const math::Pose2d pose = loc.pose();
        const double errH = pose.heading().errorTo(target_);  // shortest signed (F3)

        // exit verdict first (ExitGroup: Settled beats a simultaneous TimedOut)
        switch (exit_.check(errH)) {
            case control::ExitReason::Settled: {
                stopMotors();
                reason_ = control::ExitReason::Settled;
                state_ = MotionState::Settled;
                emitRecordFor(now, dt, pose, errH, math::ChassisSpeeds{});
                return reason_;
            }
            case control::ExitReason::TimedOut:
                return exitTimedOut("target heading not reached");
            case control::ExitReason::Cancelled:  // unreachable: check() renders only
                break;                            // Settled/TimedOut/Running (exit_group.hpp)
            case control::ExitReason::Running:
                break;
        }

        // heading PID (same continuous-near-zero encoding as MoveToPose), then
        // the ONE shared choreography (command_pipeline.hpp, since C4 — this
        // file's previous inline copy was the degenerate ω-only case of it).
        // Frame::Body: a pure ω command is identical in both frames, so the
        // pipeline's Body path — no F1 rotation — commands exactly what the
        // C1 inline code commanded, bit for bit.
        const double w = pidH_.update(0.0, -errH);  // rad/s
        const CommandOutcome cmd = applyCommandPipeline(
            deps_, cfg_, ff_,
            math::ChassisSpeeds{units::Velocity{0.0}, units::Velocity{0.0},
                                units::AngularVelocity{w}},
            math::Frame::Body, pose.heading());

        // A3 containment wiring
        const auto motors = ctx.driveMotors();
        const bool stalled = stall_.update(now, motors, pose);
        tickHealthObservables(deps_, stalled);

        emitRecordFor(now, dt, pose, errH, cmd.body);  // body == field for pure ω
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
        const math::Pose2d pose = deps_.localizer->pose();
        // A boot-window cancel emits zero error (no live estimate to err against).
        emitRecordFor(deps_.ctx->clock().now(), units::Time{0.0}, pose,
                      wasWaiting ? 0.0 : pose.heading().errorTo(target_),
                      math::ChassisSpeeds{});
    }

    /// The latched verdict: Running until an exit, then Settled, TimedOut or Cancelled. Once set
    /// it is never rewritten — a later cancel() still applies the safe state but preserves this,
    /// because a turn that settled really did settle.
    [[nodiscard]] control::ExitReason exitReason() const noexcept override { return reason_; }
    /// The motion-layer state, and the value stamped into DebugRecord.activeCommandState: Idle
    /// before start(), WaitingForEstimate through the boot wait, Running once an estimate is live,
    /// then whichever exit state matches exitReason().
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    /// The stable telemetry and result-line id — always the literal "TurnTo", a static string with
    /// no lifetime for the caller to manage.
    [[nodiscard]] const char* name() const noexcept override { return "TurnTo"; }
    /// The FIELD heading this instance was built to reach. Fixed for the object's lifetime: a
    /// TurnTo is re-armed by start(), never re-aimed, so a new heading means a new TurnTo.
    [[nodiscard]] math::Angle target() const noexcept { return target_; }

private:
    void stopMotors() {
        for (hal::IMotor* m : deps_.ctx->driveMotors()) {
            m->setVoltage(units::Voltage{0.0});
        }
    }

    control::ExitReason exitTimedOut(const char* why) {
        stopMotors();
        reason_ = control::ExitReason::TimedOut;
        state_ = MotionState::TimedOut;
        deps_.faults->raise(diag::FaultCode::MotionTimeout, name(), why);
        const math::Pose2d pose = deps_.localizer->pose();
        emitRecordFor(deps_.ctx->clock().now(), units::Time{0.0}, pose,
                      pose.heading().errorTo(target_), math::ChassisSpeeds{});
        return reason_;
    }

    void emitRecordFor(units::Time now, units::Time dt, const math::Pose2d& pose, double errH,
                       const math::ChassisSpeeds& commanded) {
        chassis::RobotContext& ctx = *deps_.ctx;
        const localization::Localizer& loc = *deps_.localizer;
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r;
            r.t = now;
            r.dt = dt;
            r.targetPose = math::Pose2d{pose.x(), pose.y(), target_};  // turn in place
            r.measuredPose = pose;
            r.errorHeading = units::AngleDim{errH};
            r.commanded = commanded;
            r.wheelCount = deps_.kinematics->wheelCount();
            const auto motors = ctx.driveMotors();
            for (std::size_t i = 0; i < motors.size()
                                    && i < static_cast<std::size_t>(diag::DebugRecord::kMaxWheels);
                 ++i) {
                r.wheelVoltage[i] = motors[i]->commandedVoltage();
                r.wheelCurrent[i] = motors[i]->current();
            }
            r.imuYaw = ctx.imu().heading();
            r.imuYawRate = ctx.imu().yawRate();
            r.activeCommandState = static_cast<std::uint8_t>(state_);
            r.deadReckoning = loc.isDeadReckoning();
            r.qualityClass = static_cast<std::uint8_t>(loc.qualityClass());
            r.quality = loc.quality();
            r.batteryVoltage = ctx.battery().voltage();
            return r;
        });
    }

    MotionDeps deps_;
    MotionConfig cfg_;
    math::Angle target_;
    control::Pid pidH_;
    control::Feedforward ff_;
    control::ExitGroup exit_;
    OdoStallCheck stall_;
    control::ExitReason reason_ = control::ExitReason::Running;
    MotionState state_ = MotionState::Idle;
    bool everLive_ = false;
    bool hasTick_ = false;
    double lastTickTime_ = 0.0;
};

}  // namespace shulib::motion
