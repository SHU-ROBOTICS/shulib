#pragma once
//
// IMotion — the contract every motion primitive implements (chunk C1, WS6/M2).
//
// This is the layer that makes the library able to DRIVE: everything below it
// (control, kinematics, localization, diagnostics) is the parts of a car; a
// motion is the driver. C1 ships MoveToPose / TurnTo / StrafeTo / HoldPose /
// DriveBrake; C2's MotionScheduler runs them asynchronously; C4's Chassis facade
// wraps them in the public verbs (F6 freezes at D2 — the shapes here are what
// that facade will inherit).
//
// ── The tick contract (who owns the loop) ───────────────────────────────────────────
// A motion does NOT own a loop, a task, or the estimator. The loop owner (a test
// harness lambda today, C2's scheduler tomorrow) each tick:
//
//     localizer.update();                 // the estimate advances FIRST
//     const auto reason = motion.tick();  // then the motion reads it and commands
//
// This is the controller-first shape A2 froze into the harness (scenario.hpp
// "Loop shape") — the motion sees the world at time t, the plant/robot then
// advances to t+dt. tick() reads sensors through RobotContext + the Localizer,
// computes ONE ChassisSpeeds command, pushes it through kinematics to the motors,
// evaluates its exit criteria, and returns the verdict:
//
//   * Running   — still working; call tick() again next loop iteration.
//   * Settled   — arrived (per SettledUtil criteria); motors have been stopped.
//   * TimedOut  — the watchdog fired first; motors have been stopped and
//                 FaultCode::MotionTimeout raised. A motion can NEVER hang: the
//                 watchdog is armed in start() and no code path disarms it.
//
// After a non-Running verdict the motion is FINISHED: further tick() calls are
// safe no-ops that return the cached verdict and leave the motors stopped.
// start() fully re-arms (a motion object is reusable, e.g. retrying a move).
//
// ── The wait-for-live-estimate contract (A3 handoff #1 — the boot window) ───────────
// A3 proved (localizer.hpp header) that motion commanded before qualityClass()
// leaves Uninitialized acts on a pose that DOES NOT EXIST YET (boot garbage is
// held out of the fold; the published pose is frozen). The C1 contract, enforced
// by every primitive except DriveBrake:
//
//   * While qualityClass() == Uninitialized, tick() commands ZERO volts, makes NO
//     settle progress, reports state() == WaitingForEstimate, and captures no
//     estimate-derived target.
//   * The WATCHDOG RUNS THROUGH THE WAIT: a never-live estimate exits TimedOut
//     (with MOTION_TIMEOUT raised) rather than hanging — "no motion can hang" is
//     absolute and includes the wait. Callers budget timeouts to cover boot.
//   * Targets derived from the current estimate (StrafeTo's held heading,
//     HoldPose's captured pose) are captured at the FIRST LIVE tick, never during
//     the boot window — capturing at start() would target calibration garbage.
//   * Quality::Degraded does NOT gate. A robot that HAD an estimate and lost
//     heading authority mid-run keeps driving on the stale estimate (the
//     Localizer's own D8 choice: encoders are still good; freezing mid-run
//     strands the robot mid-field). The watchdog + tolerance bound the damage.
//   * DriveBrake is EXEMPT by design: zero output IS the safe boot action and
//     must never be gated behind an estimate.
//
//   Rejected alternatives: refuse-at-start (an auton legitimately starts while
//   the IMU is still calibrating — the A3 survival suite's own loop waits, then
//   drives); fault-immediately (boot is NORMAL, not a fault — HealthMonitor's
//   boot-window-is-not-a-loss rule says exactly this).
//
// ── Faults & health (A3 handoff #2 lives here) ──────────────────────────────────────
// Every motion carries the fault-discipline wiring so the loop-level containment
// A3 assigned to C1 cannot be forgotten by a caller:
//   * TimedOut raises FaultCode::MotionTimeout on the shared FaultLatch.
//   * Each active tick runs the OdoStallCheck (spin-vs-motion cross-check — the
//     ONLY defence against a frozen/dead encoder until Phase E) and ticks the
//     shared HealthMonitor with every observable reachable from the deps
//     (imuReady, odomStalled, odomImplausible, fixGated, batteryVolts,
//     maxMotorTempC) — so ODO_STUCK / IMU_LOST / BROWNOUT / GPS_GATE_REJECT /
//     MOTOR_OVER_TEMP surface during motion with no extra caller wiring.
//   The active motion IS the loop at C1; when no motion is active, the loop
//   owner ticks the monitor itself (C2 formalizes this ownership).
//
// ── Observability (A1's contracts) ──────────────────────────────────────────────────
// Every tick emits one DebugRecord through hal::emitRecord() (lazy build — a
// NullSink run never populates it). The record's `commanded` field carries the
// FINAL ACHIEVABLE command expressed in the FIELD frame (the record's pose &
// control section is field-frame by schema): post strafe-authority clamp, so the
// clamping this layer owns is auditable from the record stream.
// MotionState (below) is the wire vocabulary for DebugRecord.activeCommandState.
//
// ── Units ───────────────────────────────────────────────────────────────────────────
// Pid / SettledUtil / TrapezoidProfile are bare-double BY DESIGN (their headers
// say so). THIS layer owns unit consistency: every PID instance here is
// dedicated to one axis with documented units (translation: inches → in/s;
// heading: radians → rad/s), typed quantities at every boundary, and the only
// place a frame rotation happens is math::fieldToRobot / robotToField (F1).

#include <cstdint>

#include "shulib/chassis/robot_context.hpp"
#include "shulib/control/exit_group.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/diag/health_monitor.hpp"
#include "shulib/kinematics/kinematics.hpp"
#include "shulib/localization/localizer.hpp"

namespace shulib::motion {

/// Motion-layer state, the wire vocabulary for DebugRecord.activeCommandState
/// (§18.2 — "the VOCABULARY is owned by the motion layer; once assigned, values
/// are wire-stable like FaultCode's"). Explicit values, append-only.
enum class MotionState : std::uint8_t {
    Idle = 0,                ///< constructed / reset; start() not yet called
    WaitingForEstimate = 1,  ///< started, but qualityClass() is still Uninitialized
    Running = 2,             ///< actively controlling toward the target
    Settled = 3,             ///< exited: arrived within tolerances
    TimedOut = 4,            ///< exited: watchdog fired (MOTION_TIMEOUT raised)
};

/// The dependencies every motion shares, as NAMED pointers (designated
/// initializers at the call site), validated non-null by validate(). All
/// pointees must outlive the motion. This bundle is deliberately the same set
/// the C4 Chassis facade will own — a motion is constructible from a facade's
/// internals with no reshaping (flagged for F6).
struct MotionDeps {
    chassis::RobotContext* ctx = nullptr;             ///< clock, motors, imu, battery, telemetry
    localization::Localizer* localizer = nullptr;     ///< the fused estimate + categorical quality
    const kinematics::IKinematics* kinematics = nullptr;  ///< the F5 drivetrain contract
    diag::FaultLatch* faults = nullptr;               ///< run-scoped latch (MotionTimeout, …)
    diag::HealthMonitor* health = nullptr;            ///< the A3 pathology→fault policy

    void validate() const {
        SHULIB_PRECONDITION(ctx != nullptr, "MotionDeps: ctx is null");
        SHULIB_PRECONDITION(localizer != nullptr, "MotionDeps: localizer is null");
        SHULIB_PRECONDITION(kinematics != nullptr, "MotionDeps: kinematics is null");
        SHULIB_PRECONDITION(faults != nullptr, "MotionDeps: faults is null");
        SHULIB_PRECONDITION(health != nullptr, "MotionDeps: health is null");
    }

    /// validate(), then hand out the clock — for a member-initializer list's
    /// FIRST dependency use, so a null pointer trips the precondition rather
    /// than being dereferenced.
    [[nodiscard]] hal::IClock& validatedClock() const {
        validate();
        return ctx->clock();
    }
};

class IMotion {
public:
    virtual ~IMotion() = default;
    IMotion() = default;
    IMotion(const IMotion&) = default;
    IMotion(IMotion&&) = default;
    IMotion& operator=(const IMotion&) = default;
    IMotion& operator=(IMotion&&) = default;

    /// Arm the motion: reset controllers/settle state, start the watchdog.
    /// Re-callable — a finished motion re-arms completely.
    virtual void start() = 0;

    /// One control tick (see the tick contract above). Precondition: start()
    /// has been called. The loop must update the Localizer BEFORE calling this.
    [[nodiscard]] virtual control::ExitReason tick() = 0;

    /// The verdict of the most recent tick() (Running before the first tick).
    [[nodiscard]] virtual control::ExitReason exitReason() const noexcept = 0;

    /// The motion-layer state (the activeCommandState vocabulary).
    [[nodiscard]] virtual MotionState state() const noexcept = 0;

    /// Stable short name for logs / result lines (e.g. "MoveToPose").
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

}  // namespace shulib::motion
