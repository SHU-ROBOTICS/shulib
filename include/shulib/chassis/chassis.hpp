#pragma once
//
// Chassis — the public facade every auton is written against (chunk C4, WS6/M2).
//
// ═══ STATUS: BUILT, DELIBERATELY NOT FROZEN ═══════════════════════════════════════
// This surface is the F6 CANDIDATE. It freezes at D2, after D1's recipe API has
// exercised it as a second independent consumer — a contract exercised once has
// been exercised only by its author. Until D2, shapes here may still change
// freely; after D2 they change only by version bump plus migration. The exact
// candidate signatures and the reasoning for each are recorded in the C4
// completion record (development log: docs/internal/chunks/C4-COMPLETED.md on
// the shulib-v2 branch — the section D1 stresses and D2 reads).
//
// ═══ What this class is ══════════════════════════════════════════════════════════
// The composition root of the MOTION stack: it owns the MotionScheduler and
// wraps C1's primitives in blocking verbs. It DELEGATES everywhere — motion
// logic is C1's, scheduling/fault policy C2's, kinematics C3's, the command
// choreography command_pipeline.hpp's. If a behaviour looks like it lives
// here, its test and its fix belong in the layer that owns it.
//
//   verbs      moveTo · strafeTo · turnTo · followTrajectory · drive(speeds, Frame)
//   (candidate additions, D2 decides)        brake · hold
//   control    cancel (panic stop) · waitUntil(pred, timeout)
//   state      pose · setPose · strafeAuthority · lastExitReason · lastCompleted
//   Tier 3     scheduler() · deps() — the no-ceiling seam
//
// ═══ ONE deps source (C2's structural handoff, closed here) ══════════════════════
// The facade constructs the scheduler from the caller's MotionDeps and EVERY
// motion from scheduler.deps() — the stamped bundle whose telemetry routes
// through the command-id stamp. At C2 that plumbing was a convention a caller
// had to remember (its one named gap, §5 D10); through the facade there is no
// unstamped path: a verb CANNOT build a motion from raw deps, so every record
// of every facade motion carries its command id structurally. Tier-3 callers
// composing their own IMotion get the same guarantee by building from
// chassis.deps().
//
// ═══ Construction: the standalone promise (locked principle, master plan §16.2) ═══
// A code-fluent team builds a working Chassis in PLAIN C++ — value-construct a
// kinematics preset (xDrive / hDrive / TankKinematics: drivetrain is config
// data), wire the HAL + Localizer + FaultLatch + HealthMonitor into a
// MotionDeps, pick a pacer, done. No .vexbot file, no VexBuilder, no config
// file of any kind is EVER required; G1's RobotBuilder.from(profile) is an
// additional on-ramp, never the only path. (Pinned by the file-free
// construction test.)
//
// The facade BORROWS the deps (all pointees + the pacer must outlive it) and
// OWNS the scheduler + its MotionConfig. It deliberately does NOT own the
// Localizer or the HAL: wiring those is the builder's job (G1) or the
// caller's, and owning them here would weld the facade to one localization
// stack — Tier 3 must be able to swap estimators without losing the facade.
//
// ═══ API semantics the lower layers guarantee (carried, not re-implemented) ═══════
// These are facade-level API behaviour — user code WILL depend on them:
//
//  * BLOCKING VERBS: each of moveTo/strafeTo/turnTo/followTrajectory/brake/
//    hold is async() + waitUntilSettled() on the owned scheduler — it returns
//    only when the motion exits, and CANNOT hang: the motion's own watchdog
//    bounds it (C1, mutation-proven), including any boot wait; a pacer that
//    stops advancing the clock trips a loud precondition (C2). The returned
//    ExitReason is the motion's honest verdict (Settled / TimedOut /
//    Cancelled — never Running). Deliberately NOT [[nodiscard]]: discarding
//    it is a legitimate auton style (the fault latch + C5 result lines carry
//    the pathology), and forcing (void) casts on every routine line would
//    punish the common case.
//  * WAIT-FOR-LIVE (C1): a verb issued during the boot window (estimate still
//    Uninitialized) WAITS, motionless, watchdog running — a never-live
//    estimate exits TimedOut rather than hanging. Budget timeouts to cover
//    IMU calibration (~2 s).
//  * PRE-EMPT (C2): starting a verb while a motion is active (possible via
//    the Tier-3 seam or a waitUntil predicate) cancels the old motion into
//    the safe state first; there is never a tick on which two motions
//    command. drive() pre-empts identically — a manual command supersedes.
//  * CANCEL SAFE STATE (C1/C2, HA-53): cancel() puts every drive motor at
//    0 V + BrakeMode::Brake synchronously; with no active motion it is the
//    PANIC STOP and still applies the safe state.
//  * FAULT POLICY (C2): a fault in the configured abortFaultMask raised
//    during a motion aborts it into the safe state; the verb returns
//    Cancelled and lastCompleted().abortFault names the cause. Default mask:
//    ODO_STUCK only (the estimate is lying). The run continues — faults log
//    and recover, they never crash.
//  * TURN-WHILE-DRIVE ON LIMITED-STRAFE DRIVES (C3): on an H-drive, a
//    lateral-dominant leg runs authority-limited — translation proceeds at
//    the achievable |vy| while vx and ω stay at full authority; rotation is
//    never sequenced before translation. The mode is telemetry-visible
//    (record strafeFallbackActive → TermSink " SFB"), never silent. There is
//    deliberately NO polling getter for it: a live-polled bool invites
//    control-flow coupling to a telemetry concept (C3 §11's recommendation,
//    adopted). On tank, laterally-offset targets honestly exit TimedOut.
//
// ═══ drive(ChassisSpeeds, Frame) — the frame-explicit manual verb ═════════════════
// The verb a driver-control loop calls every iteration (field-centric or
// body-frame driving), and the escape hatch for direct velocity control in
// auton. The Frame parameter has NO default: the caller must say which frame
// the command is in, so silent frame confusion — the classic bug class this
// rebuild exists to prevent — is a compile error.
//
//  * It pre-empts any active motion (above), then runs ONE loop iteration it
//    owns: localizer.update() FIRST (a teleop loop that never advanced the
//    estimate would rot the field rotation into exactly the frame bug),
//    then the shared command pipeline, health observables, one record.
//    Intended use: call it at your loop cadence; between calls nothing else
//    needs ticking.
//  * Frame::Field during the boot window commands ZERO volts (+ one Warn per
//    window): a field-relative command needs a heading, and the boot estimate
//    does not have one — rotating by garbage would move the robot in a
//    garbage direction. Frame::Body works during boot (no estimate needed).
//  * No stall cross-check: the driver is the supervisor in teleop, and there
//    is no target to cross-check against. Health observables still tick, so
//    IMU_LOST / BROWNOUT / OVER_TEMP stay live. Records carry command id 0
//    (no scheduled motion) — the honest attribution.
//
// ═══ followTrajectory — the shape is F6, the body is deliberately minimal ═════════
// Chains the waypoints as sequential MoveToPose legs through the scheduler,
// settling at each (stop-and-settle is v1's documented motion model; blending
// is a measured Frontier item). Stops at the FIRST non-Settled leg and
// reports it — a robot that timed out mid-trajectory is lost, and chasing
// later waypoints on a lie compounds blindly.
//     G2 BOUNDARY, stated honestly: no marker callbacks, no command ids on
// waypoints, no .vexbot ingestion, no profiled/curved segments — those are
// G2's PathRunner, built on this same scheduler's waitUntil primitive. This
// verb exists now because F6 freezes the VERB SET; a richer Trajectory type
// arrives as an ADDITIVE overload, never a reshape of this one.
//
// ═══ Options (per-call, additive-extensible) ══════════════════════════════════════
// MotionOptions carries the per-call knobs every real auton needs (a slow
// precise approach leg is table stakes). 0 means "use the config default".
// An options STRUCT (not positional parameters) is deliberate: post-freeze,
// new knobs are added fields — no signature change, no migration.
//
// Single-task by contract, like everything it composes. Not copyable/movable
// (it owns the scheduler, which is pinned by its self-referential stamp).

#include <cmath>
#include <initializer_list>
#include <span>
#include <utility>

#include "shulib/control/exit_group.hpp"
#include "shulib/control/feedforward.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/command_pipeline.hpp"
#include "shulib/motion/drive_brake.hpp"
#include "shulib/motion/hold_pose.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/strafe_to.hpp"
#include "shulib/motion/turn_to.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::chassis {

/// Everything configurable about a Chassis, in one place. Both members are
/// the lower layers' own config types passed through WHOLE — so an additive
/// field there (e.g. a future per-wheel speed budget in MotionConfig, the
/// C3 §11 flag) flows through this surface with no reshape.
struct ChassisConfig {
    motion::MotionConfig motion{};             ///< gains/budgets/tolerances (HA-50/51/52)
    motion::MotionSchedulerConfig scheduler{}; ///< fault policy mask + loop monitor
};

/// Per-call knobs for the blocking verbs. 0 (the default) = "use the
/// ChassisConfig value". Validated finite and >= 0 at each call.
struct MotionOptions {
    /// Watchdog bound for this motion, seconds, INCLUDING any boot wait.
    double timeoutSeconds = 0.0;
    /// Field-frame linear speed budget for this motion (in/s) — the norm cap
    /// AND the base of the strafe-authority clamp, exactly as in MotionConfig.
    /// The per-wheel budget (maxWheelSpeed) is deliberately NOT scaled with
    /// it: that is a hardware envelope, not a per-leg intent.
    units::Velocity maxLinearSpeed{0.0};
    /// Yaw-rate budget for this motion (rad/s).
    units::AngularVelocity maxAngularSpeed{0.0};

    void validate() const {
        SHULIB_PRECONDITION(std::isfinite(timeoutSeconds) && timeoutSeconds >= 0.0,
                            "MotionOptions: timeoutSeconds must be finite and >= 0");
        SHULIB_PRECONDITION(std::isfinite(maxLinearSpeed.value())
                                && maxLinearSpeed.value() >= 0.0,
                            "MotionOptions: maxLinearSpeed must be finite and >= 0");
        SHULIB_PRECONDITION(std::isfinite(maxAngularSpeed.value())
                                && maxAngularSpeed.value() >= 0.0,
                            "MotionOptions: maxAngularSpeed must be finite and >= 0");
    }
};

/// What followTrajectory did — which leg count it completed and how the last
/// attempted leg exited. (ExitReason alone would lose WHERE the chain broke;
/// the next thing a routine does after a failed trajectory legitimately
/// depends on how far it got.)
struct TrajectoryResult {
    control::ExitReason exit = control::ExitReason::Settled; ///< last attempted leg's verdict
    int completedLegs = 0;  ///< legs that SETTLED (== totalLegs on success)
    int totalLegs = 0;      ///< waypoints given
    [[nodiscard]] bool succeeded() const noexcept {
        return exit == control::ExitReason::Settled && completedLegs == totalLegs;
    }
};

class Chassis {
public:
    /// `deps` is the same validated bundle every motion takes; `pacer` is the
    /// seam through which the world advances during blocking verbs (host sim:
    /// step the plant; robot: delay to the tick boundary — R1/R3 build that
    /// one). All deps pointees AND the pacer must outlive the Chassis; the
    /// facade borrows, it does not own (header: construction).
    explicit Chassis(const motion::MotionDeps& deps, motion::ITickPacer& pacer,
                     const ChassisConfig& config = {})
        : sched_{deps, pacer, config.scheduler}, cfg_{config.motion}, ff_{config.motion.wheelFf} {
        cfg_.validate();
    }

    // Owns the scheduler (pinned in place by its self-referential stamp).
    Chassis(const Chassis&) = delete;
    Chassis(Chassis&&) = delete;
    Chassis& operator=(const Chassis&) = delete;
    Chassis& operator=(Chassis&&) = delete;
    ~Chassis() = default;

    // ── the blocking verbs (each: async + waitUntilSettled; header semantics) ──────

    /// Drive to `target` (FIELD pose): the decoupled holonomic engine —
    /// translation and rotation simultaneous and independent (C1's thesis).
    control::ExitReason moveTo(const math::Pose2d& target, const MotionOptions& options = {}) {
        options.validate();
        motion::MoveToPose m{sched_.deps(), target, effectiveConfig(options),
                             options.timeoutSeconds};
        return runBlocking(m);
    }

    /// Translate to FIELD (x, y) while actively HOLDING the heading the robot
    /// has at its first live tick. On tank (authority 0) an off-line target
    /// honestly exits TimedOut (C1's drivetrain honesty).
    control::ExitReason strafeTo(units::Length x, units::Length y,
                                 const MotionOptions& options = {}) {
        options.validate();
        motion::StrafeTo m{sched_.deps(), x, y, effectiveConfig(options),
                           options.timeoutSeconds};
        return runBlocking(m);
    }

    /// Rotate in place to a FIELD heading, always the short way (F3's
    /// shortest signed error; exact ±180° resolves CCW, deterministically).
    control::ExitReason turnTo(math::Angle heading, const MotionOptions& options = {}) {
        options.validate();
        motion::TurnTo m{sched_.deps(), heading, effectiveConfig(options),
                         options.timeoutSeconds};
        return runBlocking(m);
    }

    /// Chain `waypoints` as sequential moveTo legs, settling at each; stop at
    /// the first non-Settled leg (header: followTrajectory). `options` apply
    /// PER LEG (each leg is one scheduled motion with its own watchdog).
    /// Precondition: at least one waypoint. G2 boundary in the header.
    TrajectoryResult followTrajectory(std::span<const math::Pose2d> waypoints,
                                      const MotionOptions& options = {}) {
        SHULIB_PRECONDITION(!waypoints.empty(),
                            "Chassis::followTrajectory: waypoints must be non-empty");
        options.validate();
        // Validate EVERY waypoint before the first leg runs: a NaN at leg k
        // must not let legs 1..k-1 drive and then throw mid-routine — input
        // validation is atomic at this verb (per-leg finiteness is also
        // MoveToPose's own precondition; this hoists it to before any motion).
        for (const math::Pose2d& wp : waypoints) {
            SHULIB_PRECONDITION(std::isfinite(wp.x().value()) && std::isfinite(wp.y().value()),
                                "Chassis::followTrajectory: waypoint positions must be finite");
        }
        const motion::MotionConfig legCfg = effectiveConfig(options);
        TrajectoryResult result{.exit = control::ExitReason::Settled,
                                .completedLegs = 0,
                                .totalLegs = static_cast<int>(waypoints.size())};
        for (const math::Pose2d& wp : waypoints) {
            motion::MoveToPose leg{sched_.deps(), wp, legCfg, options.timeoutSeconds};
            result.exit = runBlocking(leg);
            if (result.exit != control::ExitReason::Settled) {
                return result;  // lost mid-chain: do not chase later waypoints blind
            }
            ++result.completedLegs;
        }
        return result;
    }

    /// Brace-list convenience: followTrajectory({a, b, c}).
    TrajectoryResult followTrajectory(std::initializer_list<math::Pose2d> waypoints,
                                      const MotionOptions& options = {}) {
        return followTrajectory(std::span<const math::Pose2d>{waypoints.begin(),
                                                              waypoints.size()},
                                options);
    }

    // ── candidate verbs beyond the roadmap's five (D2 decides; header note) ────────

    /// Stop the drivetrain (0 V under Brake) and block until the ESTIMATE
    /// certifies rest (or the watchdog fires). The controlled end-of-motion
    /// stop; cancel() is the uncontrolled one.
    control::ExitReason brake(const MotionOptions& options = {}) {
        options.validate();
        motion::DriveBrake m{sched_.deps(), effectiveConfig(options), options.timeoutSeconds};
        return runBlocking(m);
    }

    /// Actively hold the pose the robot has at its first live tick for
    /// `seconds`, driving back any disturbance with full holonomic authority;
    /// Settled iff still within tolerance when the window ends. `seconds`
    /// must be finite and > 0 (HoldPose's precondition).
    control::ExitReason hold(double seconds, const MotionOptions& options = {}) {
        options.validate();
        motion::HoldPose m{sched_.deps(), seconds, effectiveConfig(options)};
        return runBlocking(m);
    }

    // ── the manual verb ────────────────────────────────────────────────────────────

    /// Command a chassis velocity directly, in the frame the CALLER names
    /// (no default — header: drive). Pre-empts any active motion; owns one
    /// loop iteration (estimate update → shared pipeline → health → record).
    /// Precondition: all three components finite.
    void drive(const math::ChassisSpeeds& speeds, math::Frame frame) {
        SHULIB_PRECONDITION(std::isfinite(speeds.vx().value())
                                && std::isfinite(speeds.vy().value())
                                && std::isfinite(speeds.omega().value()),
                            "Chassis::drive: speeds must be finite");
        if (sched_.hasActiveMotion()) {
            sched_.cancel();  // pre-empt: a manual command supersedes, safely (C2)
        }
        const motion::MotionDeps& d = sched_.deps();
        d.localizer->update();  // the estimate advances FIRST (the loop shape)
        const units::Time now = d.ctx->clock().now();
        const units::Time dt = measuredDriveDt(now);
        const math::Pose2d pose = d.localizer->pose();

        const bool uninit = d.localizer->qualityClass()
                            == localization::Localizer::Quality::Uninitialized;
        if (frame == math::Frame::Field && uninit) {
            // A field command needs a heading; the boot estimate has none.
            for (hal::IMotor* m : d.ctx->driveMotors()) {
                m->setVoltage(units::Voltage{0.0});
            }
            if (!warnedFieldDriveUninit_) {
                d.ctx->telemetry().log(hal::LogLevel::Warn, "CHS",
                                       "field-frame drive() before the estimate is live: "
                                       "commanding zero until it is");
                warnedFieldDriveUninit_ = true;  // once per boot window, not per call
            }
            motion::tickHealthObservables(d, false);
            emitDriveRecord(now, dt, pose, math::ChassisSpeeds{}, false);
            return;
        }
        warnedFieldDriveUninit_ = false;  // live again: re-arm the once-per-window warn

        const motion::CommandOutcome out =
            motion::applyCommandPipeline(d, cfg_, ff_, speeds, frame, pose.heading());
        motion::tickHealthObservables(d, false);
        emitDriveRecord(now, dt, pose, math::robotToField(out.body, pose.heading()),
                        out.strafeFallback);
    }

    // ── control ────────────────────────────────────────────────────────────────────

    /// Stop the active motion into the defined safe state (0 V + Brake); with
    /// no active motion this is the PANIC STOP and still safes the drive.
    void cancel() { sched_.cancel(); }

    /// Block until `pred()` holds or `timeoutSeconds` elapses (required,
    /// finite, >= 0; 0 = an honest poll) — the return says which. The active
    /// motion (if any) keeps ticking throughout; the world keeps advancing.
    /// Timing out logs one Warn and raises NO fault (a timed-out wait is a
    /// strategy branch, not a pathology). C2's verb, re-exported unchanged.
    template <typename Pred>
    [[nodiscard]] motion::WaitResult waitUntil(Pred&& pred, double timeoutSeconds) {
        return sched_.waitUntil(std::forward<Pred>(pred), timeoutSeconds);
    }

    // ── state / observability ──────────────────────────────────────────────────────

    /// The current fused FIELD pose estimate.
    [[nodiscard]] math::Pose2d pose() const { return sched_.deps().localizer->pose(); }

    /// Seed / teleport the estimated POSITION (x, y) — heading stays
    /// IMU-owned (the Localizer's structural choice). Call at auton start
    /// with the measured starting pose.
    void setPose(const math::Pose2d& pose) { sched_.deps().localizer->setPose(pose); }

    /// Read-only passthrough of the drivetrain's sustainable lateral
    /// authority (fraction of the linear budget; F5). Routine authors
    /// budgeting lateral legs legitimately want it — the difference between a
    /// 2 s and a 3 s leg on the H-bot (C3 §11 #2, adopted).
    [[nodiscard]] double strafeAuthority() const {
        return sched_.deps().kinematics->strafeAuthority();
    }

    /// Exit reason of the most recently finished motion (Settled on a virgin
    /// chassis — completedCount() via scheduler() says whether anything ran).
    [[nodiscard]] control::ExitReason lastExitReason() const noexcept {
        return sched_.lastExitReason();
    }

    /// The most recent motion boundary — id/name/exit/abortFault/times (C5's
    /// raw material; abortFault names a fault-policy cause).
    [[nodiscard]] const motion::CompletedMotion& lastCompleted() const noexcept {
        return sched_.lastCompleted();
    }

    /// The config the verbs run under (per-call options override per motion).
    [[nodiscard]] const motion::MotionConfig& motionConfig() const noexcept { return cfg_; }

    // ── the Tier-3 seam (no ceiling; header note) ──────────────────────────────────

    /// The STAMPED deps bundle — build custom IMotions from THIS and their
    /// records carry command ids like the built-in verbs' do.
    [[nodiscard]] const motion::MotionDeps& deps() const noexcept { return sched_.deps(); }

    /// The owned scheduler, for async composition / caller-paced tick() /
    /// counters. It is the SAME single motion slot the verbs use: async()
    /// here pre-empts a facade verb's motion and vice versa (one-active-
    /// motion is structural, never relaxed).
    [[nodiscard]] motion::MotionScheduler& scheduler() noexcept { return sched_; }
    [[nodiscard]] const motion::MotionScheduler& scheduler() const noexcept { return sched_; }

private:
    /// If a blocking wait throws (stalled pacer, an estimator precondition
    /// with no motion boundary to convert it), the stack-owned motion in
    /// runBlocking would otherwise DANGLE in the scheduler's active slot —
    /// a later verb would pre-empt-cancel a dead object. This guard cancels
    /// on unwind: the drivetrain lands in the safe state and the slot is
    /// cleared BEFORE the motion object dies. (Declared after the motion in
    /// runBlocking, so it destructs first.)
    class DetachGuard {
    public:
        explicit DetachGuard(motion::MotionScheduler& sched) noexcept : sched_{&sched} {}
        ~DetachGuard() {
            if (sched_ != nullptr) {
                sched_->cancel();
            }
        }
        DetachGuard(const DetachGuard&) = delete;
        DetachGuard& operator=(const DetachGuard&) = delete;
        void disarm() noexcept { sched_ = nullptr; }

    private:
        motion::MotionScheduler* sched_;
    };

    /// async + waitUntilSettled, exception-safe (DetachGuard above). The
    /// motion lives on the CALLER's stack for exactly the blocking window —
    /// safe because waitUntilSettled cannot return while it is active.
    control::ExitReason runBlocking(motion::IMotion& m) {
        sched_.async(m);
        DetachGuard guard{sched_};
        const control::ExitReason reason = sched_.waitUntilSettled();
        guard.disarm();
        return reason;
    }

    /// The per-call config: the chassis config with any nonzero option
    /// overrides applied (0 = keep the default — MotionConfig::validate
    /// forbids 0 budgets, so 0 is unambiguous as a sentinel).
    [[nodiscard]] motion::MotionConfig effectiveConfig(const MotionOptions& options) const {
        motion::MotionConfig cfg = cfg_;
        if (options.maxLinearSpeed.value() > 0.0) {
            cfg.maxLinearSpeed = options.maxLinearSpeed;
        }
        if (options.maxAngularSpeed.value() > 0.0) {
            cfg.maxAngularSpeed = options.maxAngularSpeed;
        }
        return cfg;
    }

    /// dt between drive() calls (0 on the first / after a gap is fine — the
    /// record's dt is observability, not control input).
    [[nodiscard]] units::Time measuredDriveDt(units::Time now) noexcept {
        const double dt = hasDriveTick_ ? (now.value() - lastDriveTime_) : 0.0;
        lastDriveTime_ = now.value();
        hasDriveTick_ = true;
        return units::Time{dt};
    }

    /// drive()'s record: pose/quality/power continuity plus the FINAL
    /// achievable command (FIELD frame, per the record schema) and the C3
    /// fallback flag. No target — nothing closed-loop to err against; the
    /// fields stay their quiet defaults (the record must not invent). Lazy
    /// via emitRecord (A1 cost contract); rides the stamped sink with id 0.
    void emitDriveRecord(units::Time now, units::Time dt, const math::Pose2d& pose,
                         const math::ChassisSpeeds& commandedField, bool strafeFallback) {
        const motion::MotionDeps& d = sched_.deps();
        RobotContext& ctx = *d.ctx;
        const localization::Localizer& loc = *d.localizer;
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r;
            r.t = now;
            r.dt = dt;
            r.measuredPose = pose;
            r.commanded = commandedField;
            r.strafeFallbackActive = strafeFallback;
            r.wheelCount = d.kinematics->wheelCount();
            const auto motors = ctx.driveMotors();
            for (std::size_t i = 0;
                 i < motors.size()
                 && i < static_cast<std::size_t>(diag::DebugRecord::kMaxWheels);
                 ++i) {
                r.wheelVoltage[i] = motors[i]->commandedVoltage();
                r.wheelCurrent[i] = motors[i]->current();
            }
            r.imuYaw = ctx.imu().heading();
            r.imuYawRate = ctx.imu().yawRate();
            r.deadReckoning = loc.isDeadReckoning();
            r.qualityClass = static_cast<std::uint8_t>(loc.qualityClass());
            r.quality = loc.quality();
            r.batteryVoltage = ctx.battery().voltage();
            return r;
        });
    }

    motion::MotionScheduler sched_;
    motion::MotionConfig cfg_;
    control::Feedforward ff_;
    bool warnedFieldDriveUninit_ = false;
    bool hasDriveTick_ = false;
    double lastDriveTime_ = 0.0;
};

}  // namespace shulib::chassis
