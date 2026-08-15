#pragma once
//
// DriveBrake — stop the drivetrain and confirm it stopped (chunk C1).
//
// Commands ZERO volts to every drive motor each tick (with BrakeMode::Brake so
// real hardware resists rather than coasts — the A2 plant does not model brake
// modes, documented limitation) and exits Settled once the ESTIMATED speed
// norm |v| + rotationRadius·|ω| has stayed inside brakeSettle for its held
// time; the watchdog bounds the wait (a robot that never reads stopped — e.g.
// a dead estimator — exits TimedOut rather than hanging).
//
// ── The estimator's twist noise floor (measured at C1, why the averaging) ───────────
// The M2 Localizer's twist is a RAW finite difference of the fused position; at
// a physical dead stop under A3's composed hostility it reads 0.5–1.5 in/s of
// noise (heading noise ≈ HA-21 through the tracking-offset correction path,
// differentiated at 100 Hz). A settle threshold below that floor can never
// certify "stopped". So the speed norm is built from a kTwistAvgTicks-tick
// VECTOR average (≈√n noise reduction, ~50 ms verdict delay), and the
// brakeSettle default sits above the averaged floor (HA-51). TRUE stopped-ness
// is pinned against plant ground truth by test; the verdict here is what the
// ESTIMATE can honestly certify at M2.
//
// Boot-window verdict deferral, documented: during IMU calibration the yaw-rate
// stream is garbage (±10 rad/s class), so a brake issued mid-boot holds its
// zero-volt command IMMEDIATELY but cannot certify "stopped" until the sensor
// goes live (~calibration end). Action now, verdict when provable.
//
// ── EXEMPT from the wait-for-live gate, BY DESIGN (motion.hpp) ──────────────────────
// Zero output is the SAFE action in every state, including the boot window —
// gating "stop" behind an estimate would be absurd (and the guaranteed
// end-of-run park must be able to kill the drive unconditionally). The cost,
// documented honestly: during Uninitialized the settle verdict reads the
// Localizer's boot-frozen twist (reports ~0), so a robot being PUSHED during
// calibration can read "stopped" while moving. With zero volts commanded there
// is no better observable at M2; the verdict is best-available, not an
// accuracy claim.
//
// No stall/health wiring here: nothing is commanded, so the spin-vs-motion
// cross-check has no spin to compare (and a false ODO_STUCK from a coasting
// wheel would be noise). The record stream still runs (A1 contract).

#include <algorithm>
#include <array>
#include <cmath>

#include "shulib/control/exit_group.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// The OPEN-LOOP stop: 0 V under BrakeMode::Brake on every drive motor, every tick, plus a
/// verdict that the drivetrain actually came to rest. The ONE primitive exempt from the
/// wait-for-live-estimate gate (motion.hpp) — zero output is the safe action in every state,
/// boot window included — so the command lands immediately and only the SETTLE verdict
/// depends on the estimator. Settled therefore means "the estimate can certify rest", which
/// during the boot window can read stopped for a robot being pushed (header). Nothing is
/// commanded, so no stall/health cross-check runs here. HoldPose is the closed-loop
/// counterpart: it recovers a pose, this one only stops.
class DriveBrake final : public IMotion {
public:
    /// Twist-averaging window, ticks (header note). At 100 Hz: 50 ms.
    static constexpr int kTwistAvgTicks = 5;
    /// `timeout` (s) bounds the whole stop; 0 selects config.defaultTimeout.
    DriveBrake(const MotionDeps& deps, const MotionConfig& config = {}, double timeout = 0.0)
        : deps_{deps},
          cfg_{config},
          exit_{config.brakeSettle, timeout > 0.0 ? timeout : config.defaultTimeout,
                deps.validatedClock()} {
        cfg_.validate();
        SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0,
                            "DriveBrake: timeout must be finite and >= 0");
    }

    /// Arm: reset the settle window, restart the watchdog, and empty the twist-averaging
    /// ring so a re-run never averages across the previous stop. Goes straight to
    /// MotionState::Running — this is the one primitive that never reports
    /// WaitingForEstimate. Re-callable: a finished brake re-arms completely.
    void start() override {
        exit_.start();
        reason_ = control::ExitReason::Running;
        state_ = MotionState::Running;  // no live gate: braking starts immediately
        hasTick_ = false;
        lastTickTime_ = 0.0;
        ringCount_ = 0;
        ringNext_ = 0;
    }

    /// One tick. Re-commands Brake + 0 V (idempotent), then judges the estimated speed norm
    /// |v| + rotationRadius·|ω| — in/s, folding rotation into linear currency so a spinning
    /// robot is not "stopped" — against config.brakeSettle. The norm is built from the
    /// kTwistAvgTicks-tick VECTOR average of the localizer's twist, not the raw one, so the
    /// first few ticks after start() average over fewer samples. TimedOut also raises
    /// FaultCode::MotionTimeout. Precondition: start() was called. Once a verdict is
    /// reached, later calls return it and command nothing further, and emit no record —
    /// the motors were already left at 0 V.
    [[nodiscard]] control::ExitReason tick() override {
        SHULIB_PRECONDITION(state_ != MotionState::Idle, "DriveBrake::tick: start() not called");
        if (reason_ != control::ExitReason::Running) {
            return reason_;
        }
        chassis::RobotContext& ctx = *deps_.ctx;
        const units::Time now = ctx.clock().now();
        const double dtRaw = hasTick_ ? (now.value() - lastTickTime_) : 0.0;
        lastTickTime_ = now.value();
        hasTick_ = true;

        // the stop command, every tick (idempotent; Brake mode for hardware)
        for (hal::IMotor* m : ctx.driveMotors()) {
            m->setBrakeMode(hal::BrakeMode::Brake);
            m->setVoltage(units::Voltage{0.0});
        }

        // estimated speed norm (in/s): |v| + R·|ω| folds rotation into the same
        // currency so a spinning robot is not "stopped". Built from the
        // VECTOR-averaged twist (header note: the raw finite-difference twist's
        // noise floor would defeat any honest threshold).
        const math::Twist2d tw = deps_.localizer->twist();
        const auto slot = static_cast<std::size_t>(ringNext_);
        ringVx_[slot] = tw.vx().value();
        ringVy_[slot] = tw.vy().value();
        ringW_[slot] = tw.omega().value();
        ringNext_ = (ringNext_ + 1) % kTwistAvgTicks;
        ringCount_ = std::min(ringCount_ + 1, kTwistAvgTicks);
        double vx = 0.0;
        double vy = 0.0;
        double w = 0.0;
        for (int i = 0; i < ringCount_; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            vx += ringVx_[idx];
            vy += ringVy_[idx];
            w += ringW_[idx];
        }
        const double n = static_cast<double>(ringCount_);
        const double speed =
            std::hypot(vx / n, vy / n) + cfg_.rotationRadius.value() * std::abs(w / n);

        control::ExitReason verdict = exit_.check(speed);
        if (verdict == control::ExitReason::TimedOut) {
            deps_.faults->raise(diag::FaultCode::MotionTimeout, name(), "drive never read stopped");
            state_ = MotionState::TimedOut;
            reason_ = verdict;
        } else if (verdict == control::ExitReason::Settled) {
            state_ = MotionState::Settled;
            reason_ = verdict;
        }

        emitRecordFor(now, units::Time{dtRaw}, speed);
        return verdict;
    }

    /// The cancel contract (motion.hpp). Cancelling a brake changes nothing
    /// physical — the safe state (0 V + Brake) is exactly what every tick was
    /// already commanding — but the verdict discipline is identical to every
    /// other motion: a running brake ends Cancelled (it never CERTIFIED the
    /// stop), a settled/timed-out one keeps its verdict.
    void cancel() override {
        if (state_ == MotionState::Idle) {
            return;  // never started: no relationship to the drivetrain yet
        }
        applyCancelSafeState(*deps_.ctx);
        if (reason_ != control::ExitReason::Running) {
            return;  // already exited: verdict preserved (motion.hpp rationale)
        }
        reason_ = control::ExitReason::Cancelled;
        state_ = MotionState::Cancelled;
        emitRecordFor(deps_.ctx->clock().now(), units::Time{0.0}, 0.0);
    }

    /// The verdict of the last tick(): Running until the AVERAGED speed norm settles, then
    /// Settled / TimedOut / Cancelled, held unchanged from then on. Running before the first
    /// tick(). Settled is a claim about what the estimate can certify, not about ground-truth
    /// stillness (header).
    [[nodiscard]] control::ExitReason exitReason() const noexcept override { return reason_; }

    /// Idle before start(); Running from start() onward — WaitingForEstimate is the one
    /// MotionState this primitive never reports — then Settled / TimedOut / Cancelled. This
    /// is the value the record stream carries in DebugRecord.activeCommandState.
    [[nodiscard]] MotionState state() const noexcept override { return state_; }

    /// The literal "DriveBrake" — what result lines and the MOTION_TIMEOUT fault name this
    /// motion, so a stop that never certified is distinguishable from a move that failed.
    [[nodiscard]] const char* name() const noexcept override { return "DriveBrake"; }

private:
    void emitRecordFor(units::Time now, units::Time dt, double speed) {
        chassis::RobotContext& ctx = *deps_.ctx;
        const localization::Localizer& loc = *deps_.localizer;
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r;
            r.t = now;
            r.dt = dt;
            r.measuredPose = loc.pose();
            r.targetPose = r.measuredPose;  // a brake targets "here, at rest";
            (void)speed;                    // the speed norm is exit logic, not a
                                            // record field — error fields stay 0
            r.commanded = math::ChassisSpeeds{};
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
    control::ExitGroup exit_;
    control::ExitReason reason_ = control::ExitReason::Running;
    MotionState state_ = MotionState::Idle;
    bool hasTick_ = false;
    double lastTickTime_ = 0.0;
    std::array<double, static_cast<std::size_t>(kTwistAvgTicks)> ringVx_{};
    std::array<double, static_cast<std::size_t>(kTwistAvgTicks)> ringVy_{};
    std::array<double, static_cast<std::size_t>(kTwistAvgTicks)> ringW_{};
    int ringCount_ = 0;
    int ringNext_ = 0;
};

}  // namespace shulib::motion
