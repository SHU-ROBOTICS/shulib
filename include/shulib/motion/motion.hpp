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
//   * Cancelled — stopped from outside via cancel() (added at C2 through the
//                 documented additive path; see the cancel contract below).
//
// After a non-Running verdict the motion is FINISHED: further tick() calls are
// safe no-ops that return the cached verdict and leave the motors stopped.
// start() fully re-arms (a motion object is reusable, e.g. retrying a move).
//
// ── The cancel contract (chunk C2 — the scheduler's structural guarantee) ───────────
// cancel() is how a motion is stopped from OUTSIDE its own exit criteria: user
// abort, scheduler pre-emption (a new motion supersedes this one), or the C2
// fault policy (e.g. ODO_STUCK — the estimate is lying, and continuing to servo
// against a lie is worse than stopping). Semantics, identical in every
// primitive and pinned by test:
//
//   * If the motion is RUNNING (incl. WaitingForEstimate — a cancel during the
//     boot window is legal and common): the drivetrain is put in the CANCEL
//     SAFE STATE (below) synchronously, the verdict becomes Cancelled, one
//     final exit record is emitted, and the motion is FINISHED — subsequent
//     tick() calls are the same safe no-ops as any other exit. This is what
//     makes one-active-motion STRUCTURAL for the scheduler: a pre-empted
//     motion object is inert at the object level, not merely unreferenced.
//   * If the motion already EXITED (Settled / TimedOut / Cancelled): the safe
//     state is still applied (cancel() means "make the drivetrain safe NOW",
//     and it must be idempotent), but the verdict is PRESERVED — a motion that
//     settled really did settle; rewriting history would lie to the C5 result
//     line. Back-to-back cancels are therefore harmless no-ops after the first.
//   * If the motion was never started (Idle): complete no-op. An unstarted
//     motion has no relationship to the drivetrain yet; commanding motors from
//     it would be the surprise, not the safety.
//   * cancel() raises NO fault. Cancellation is a commanded, normal act; when
//     the CAUSE is a fault, that fault is already latched by whoever detected
//     it (the scheduler records the causal code alongside the boundary).
//
// The CANCEL SAFE STATE is defined once, in applyCancelSafeState() below:
// zero volts + BrakeMode::Brake on every drive motor. Why brake and not the
// alternatives: COAST lets a robot at speed keep rolling (a cancel that leaves
// the robot coasting into a wall fails the whole point); a closed-loop HOLD
// keeps servoing against the estimate — and the highest-priority cancel cause
// (ODO_STUCK) is precisely "the estimate is lying", so holding would reproduce
// the failure cancel exists to stop. Brake is estimate-independent (valid
// during the boot window, valid with dead encoders), passive, and immediate.
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

#include <algorithm>
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
    Cancelled = 5,           ///< exited: cancel() — stopped from outside (APPENDED at
                             ///< chunk C2 per the append-only rule; wire-stable)
};

/// The CANCEL SAFE STATE, defined in ONE place so every cancel path — each
/// primitive's cancel(), the scheduler's pre-emption, its fault-policy abort,
/// and its no-active-motion panic stop — commands the identical thing: zero
/// volts under BrakeMode::Brake on every drive motor (rationale in the cancel
/// contract above). Brake mode is set BEFORE the zero-volt command so the stop
/// lands under braking semantics, never a momentary coast.
///
/// HARDWARE CLAIM, honest scope: the A2 plant does not model brake modes, so
/// host tests prove the 0 V dynamics reach rest and pin the Brake command by
/// state inspection — how hard a real V5 drivetrain brakes from speed is
/// unverifiable until hardware. PROVISIONAL (A4: HA-53).
inline void applyCancelSafeState(chassis::RobotContext& ctx) {
    for (hal::IMotor* m : ctx.driveMotors()) {
        m->setBrakeMode(hal::BrakeMode::Brake);
        m->setVoltage(units::Voltage{0.0});
    }
}

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

    /// Trip SHULIB_PRECONDITION on the FIRST null pointer, naming which one. Every motion
    /// calls this from its constructor (through validatedClock()), so a dependency the
    /// designated-initializer call site forgot is a loud contract breach at construction
    /// rather than a null dereference three ticks into an auton.
    void validate() const {
        SHULIB_PRECONDITION(ctx != nullptr, "MotionDeps: ctx is null");
        SHULIB_PRECONDITION(localizer != nullptr, "MotionDeps: localizer is null");
        SHULIB_PRECONDITION(kinematics != nullptr, "MotionDeps: kinematics is null");
        SHULIB_PRECONDITION(faults != nullptr, "MotionDeps: faults is null");
        SHULIB_PRECONDITION(health != nullptr, "MotionDeps: health is null");
        // The one cross-check no single component can make. RobotContext validates that
        // driveMotors is non-empty and all-non-null; IKinematics knows how many wheels it
        // has; NOTHING compared them, and applyCommandPipeline indexes the motor span by
        // WHEEL index with std::span::operator[], which is unchecked. A context built with
        // three motors and an XDrive installed therefore read one past the end of the span
        // on EVERY tick — undefined behaviour with no diagnostic, on the hot path. This
        // bundle is the one place that holds both, so the check lives here.
        SHULIB_PRECONDITION(
            ctx->driveMotors().size() >= static_cast<std::size_t>(kinematics->wheelCount()),
            "MotionDeps: fewer drive motors than the kinematics has wheels");
    }

    /// validate(), then hand out the clock — for a member-initializer list's
    /// FIRST dependency use, so a null pointer trips the precondition rather
    /// than being dereferenced.
    [[nodiscard]] hal::IClock& validatedClock() const {
        validate();
        return ctx->clock();
    }
};

/// Tick the shared HealthMonitor with every observable reachable from the
/// deps — the A3 containment wiring in ONE place (chunk C4; three copies had
/// grown by then: MoveToPose, TurnTo, and the scheduler's idle tick, and the
/// facade's drive() would have been a fourth). `odomStalled` stays a
/// parameter because it is the one observable with a per-caller story: the
/// active motion feeds its OdoStallCheck verdict; idle/teleop callers pass
/// false — nothing (or nothing closed-loop) is commanded, so there is no
/// spin to cross-check (the DriveBrake-exemption reasoning).
inline void tickHealthObservables(const MotionDeps& deps, bool odomStalled) {
    chassis::RobotContext& ctx = *deps.ctx;
    localization::Localizer& loc = *deps.localizer;
    double maxTemp = 0.0;
    for (const hal::IMotor* m : ctx.driveMotors()) {
        maxTemp = std::max(maxTemp, m->temperature());
    }
    deps.health->tick({.imuReady = ctx.imu().isReady(),
                       .odomImplausible = loc.lastOdomDeltaImplausible(),
                       .odomStalled = odomStalled,
                       .fixGated = loc.lastCorrection().gated,
                       .batteryVolts = ctx.battery().voltage(),
                       .maxMotorTempC = maxTemp});
}

/// The contract every motion primitive implements: one target, one tick() that reads the
/// world and issues ONE drivetrain command, one verdict. A motion owns no loop, no task
/// and no estimator — the loop owner advances the Localizer first, then calls tick() (the
/// tick contract above). Implementers owe the whole of it, not just the signatures: an
/// exit leaves the motors stopped and every later tick() is a no-op returning the cached
/// verdict, start() fully re-arms a finished object, and cancel() works at any time and
/// is idempotent. No motion may hang — the watchdog runs even while waiting for a live
/// estimate.
class IMotion {
public:
    /// Interface plumbing, spelled out because declaring the destructor demands all six:
    /// motions are held and destroyed through this base, and copy/move are defaulted
    /// because IMotion itself holds no state — every motion's state is in the concrete
    /// type, which is also why the scheduler passes motions by pointer, not by value.
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

    /// Stop the motion from outside (see the cancel contract above). PURE
    /// virtual ON PURPOSE — a motion type without a cancellation story is the
    /// forgettable-safety-step failure mode (A1's emitRecord lesson); every
    /// implementer must state one. Idempotent; never raises; applies the
    /// cancel safe state whenever the motion has been started.
    virtual void cancel() = 0;

    /// The verdict of the most recent tick() (Running before the first tick).
    [[nodiscard]] virtual control::ExitReason exitReason() const noexcept = 0;

    /// The motion-layer state (the activeCommandState vocabulary).
    [[nodiscard]] virtual MotionState state() const noexcept = 0;

    /// Stable short name for logs / result lines (e.g. "MoveToPose").
    [[nodiscard]] virtual const char* name() const noexcept = 0;
};

}  // namespace shulib::motion
