#pragma once
//
// MoveToPose — decoupled per-axis field-pose motion (chunk C1). THE HOLONOMIC
// THESIS, as code: three INDEPENDENT controllers — field-x, field-y, heading —
// each closing its own loop every tick, combined into one ChassisSpeeds. The
// robot translates and rotates SIMULTANEOUSLY and INDEPENDENTLY; there is no
// turn-then-drive sequencing anywhere in this file, and the simultaneity test
// pins that a diagonal move with a heading change happens as ONE motion. (That
// sequencing is precisely LemLib's tank-coupled behaviour this project exists
// to beat.) Two sanctioned exceptions, each telemetry-visible by contract:
//   * C3's strafe FALLBACK — on a drive with limited strafe authority, body |vy|
//     is clamped while vx and ω keep full authority (record flag SFB);
//   * TANK1's tank steering — on a drive with NO strafe authority (a tank;
//     strafeAuthority() exactly 0), a plain MoveToPose whose leg STARTS within
//     tankSteer.maxSteer (15°) of its target heading bends its heading setpoint
//     toward the target's LINE (through the target point, along the target
//     heading) by at most maxSteer, faded to 0 at the target — steering on
//     where the robot will be off that line tankSteer.leadTime ahead, so the
//     bend eases while it closes on the line — and cancels the
//     measured yaw it did not ask for (a weaker side's), bounded by
//     yawCorrectionLimit (record flag TSTR). What it is NOT: a planned turn.
//     Nothing in it aims at the target POINT (tank_steer.hpp), and a leg that
//     starts farther off its heading is not steered at all — it runs exactly as
//     before TANK1 and logs one Warn ("turn first: face()"). StrafeTo and
//     HoldPose never steer (PoseMotionOptions::tankSteer stays false).
//
// ── The per-tick pipeline (frames and clamps annotated — the F1/F5 choreography) ────
//   1. pose  = Localizer::pose()                                  [FIELD]
//   2. errors: ex = tx−px, ey = ty−py (in); eh = heading.errorTo(target)
//      — eh is F3's SHORTEST signed error, so the ±180° seam is absorbed
//      BEFORE any controller sees a number.                       [FIELD/rad]
//   3. three decoupled PIDs → (vx, vy) in/s, ω rad/s              [FIELD]
//      heading PID is fed (setpoint 0, measurement −eh): the error the PID
//      differentiates is CONTINUOUS near the target (no wrap in its input),
//      and D-on-measurement stays live for a future kD. On an ENGAGED tank
//      leg (TANK1, above) it is fed the error to the BENT setpoint instead
//      (eh_s = heading.errorTo(target heading + bend), the bend taken from the
//      cross-track error predicted leadTime ahead — tank_steer.hpp), and the
//      yaw correction is subtracted from its output; every other leg is fed
//      −eh exactly.
//   4–8. applyCommandPipeline (command_pipeline.hpp — extracted from here at
//      C4 so the Chassis facade's drive() shares ONE choreography): |ω| clamp;
//      uniform norm cap; fieldToRobot (THE one F1 rotation); body |vy| clamped
//      to strafeAuthority()·maxLinearSpeed (the UPSTREAM clamp §13 #5 assigns
//      to THIS layer — kinematics never clamps, F5; a meaningful bind flags
//      strafeFallbackActive in the record, C3's never-silent contract);
//      toWheels → desaturate → Feedforward → compensateForBattery → volts.
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
#include "shulib/diag/line_format.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/command_pipeline.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/motion/tank_steer.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// Internal shaping knobs for the sibling primitives (StrafeTo / HoldPose).
/// Not part of MoveToPose's public construction surface.
struct PoseMotionOptions {
    bool captureHeadingAtLive = false;  ///< StrafeTo: hold the first-live heading
    bool capturePoseAtLive = false;     ///< HoldPose: hold the first-live pose
    double holdFor = 0.0;               ///< > 0 ⇒ hold-mode exit (HoldPose)
    /// TANK1: this motion may steer a tank back onto its target's line. Only the public
    /// MoveToPose constructor sets it; StrafeTo (a translation that HOLDS heading) and
    /// HoldPose (no line to steer along) leave it false, so on a tank they move exactly as
    /// before. (Their result lines are another matter: the scheduler gives a tank's StrafeTo,
    /// and a HoldPose of a spot away from the robot, TANK1 Part B's ` xt` column — a report.)
    bool tankSteer = false;
};

/// Drive to a FIELD-frame pose with three INDEPENDENT controllers — field-x, field-y and
/// heading — each closing its own loop every tick and combining into one ChassisSpeeds. The
/// robot therefore translates and rotates simultaneously; nothing in this class sequences a
/// turn before a drive. Arrival needs BOTH criteria at once (translation distance AND heading
/// error), so it composes two SettledUtils and one Watchdog rather than one scalar exit.
/// StrafeTo and HoldPose are this same engine with different capture/exit options.
///
/// On a TANK (strafeAuthority() exactly 0) a leg that starts within config.tankSteer.maxSteer of
/// its target heading also steers back onto the target's LINE while it drives (TANK1 — the file
/// header; MotionConfig::tankSteer; tankSteerStatus()), and a leg that starts farther off runs
/// unsteered and logs one Warn. It never plans a turn: face() the point first.
///
/// A MoveToPose owns no loop and no thread: the caller ticks it, having updated the
/// Localizer first, until tick() returns something other than Running.
class MoveToPose : public IMotion {
public:
    /// Drive to `target` (FIELD frame). `timeout` seconds bounds the whole
    /// motion INCLUDING any boot wait; 0 selects config.defaultTimeout.
    MoveToPose(const MotionDeps& deps, const math::Pose2d& target,
               const MotionConfig& config = {}, double timeout = 0.0)
        : MoveToPose(deps, target, config, timeout, PoseMotionOptions{.tankSteer = true}) {}

    /// Arm, or fully re-arm: the three PIDs, both settle detectors and the stall check are
    /// reset, the watchdog clock restarts, and the state drops back to WaitingForEstimate.
    /// Commands no motors. A capture-at-first-live target (StrafeTo's heading, HoldPose's
    /// pose) is re-armed too, so a re-started motion captures again from the CURRENT estimate
    /// rather than reusing the previous run's. A plain MoveToPose keeps its explicit target.
    /// TANK1's state is re-armed too: the engage decision is made again at the next first live
    /// tick, and the yaw correction starts from 0.
    void start() override {
        pidX_.reset();
        pidY_.reset();
        pidH_.reset();
        tankDecided_ = false;
        tankEngaged_ = false;
        yawDisturbance_ = 0.0;
        lastOmegaCmd_ = 0.0;
        lastVxCmd_ = 0.0;
        hasOmegaCmd_ = false;
        lastCross_ = 0.0;
        lastCrossAhead_ = 0.0;
        lastBend_ = 0.0;
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

    /// One control tick, and the only member here that commands a DRIVING voltage — cancel()
    /// commands the motors too, into the shared safe state, and is in fact the only member that
    /// ever changes a brake mode (this one's stops just write 0 V). Precondition:
    /// start() has been called; the loop owner must have advanced the Localizer FIRST, since
    /// this reads the estimate as the world at time t. While the estimate is still
    /// Uninitialized it commands zero volts and makes no settle progress — but the watchdog
    /// keeps running through that wait, so a never-live estimate exits TimedOut instead of
    /// hanging. Returns Running until both criteria settle (Settled) or the watchdog fires
    /// (TimedOut, MotionTimeout raised); motors are stopped BEFORE the exit record is emitted,
    /// so the record stream ends on the true final state. After any non-Running verdict this
    /// is a no-op that returns the cached verdict. Emits AT MOST one DebugRecord per call: that
    /// cached-verdict path emits nothing, and no path emits unless the sink answers
    /// wantsRecord() — the record is built inside hal::emitRecord's lambda, so against a
    /// NullSink or any log-only sink it is never populated at all. When one is emitted its
    /// `commanded` field is the FINAL achievable command in the FIELD frame — post-clamp, so
    /// this layer's clamping is auditable from the stream.
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

        // ── TANK1: engage tank steering or not — decided ONCE, at the first live tick ──
        // Never at start() (the boot window has no heading — the wait-for-live contract), and
        // never per tick (a leg the heading loop turned into the window mid-run would then be
        // steered: the library making the author's turn).
        if (!tankDecided_) {
            tankDecided_ = true;
            tankEngaged_ = tankEligible_ && tankLegEngages(errH, cfg_.tankSteer);
            if (tankEligible_ && !tankEngaged_) {
                warnNotSteered(errH);  // never silent: say why this leg will not be steered
            }
        }

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
        const double vxF = pidX_.update(target_.x().value(), pose.x().value());  // in/s
        const double vyF = pidY_.update(target_.y().value(), pose.y().value());  // in/s
        // TANK1 (tank only, engaged legs only): the heading loop servoes to the target
        // heading BENT toward the target's line, minus the measured yaw the loop did not ask
        // for (a weaker side's). Everywhere else this is C1's line, unchanged. Every constant
        // this path reads is MotionConfig::tankSteer's, PROVISIONAL (A4: HA-179) — tuned on the
        // host test plant, never on a robot. The path also depends on the SIGN of the
        // Localizer's yaw rate agreeing with the heading's (HA-04; HA-179's blast radius).
        double w = 0.0;                                                           // rad/s
        bool tankActive = false;
        if (tankEngaged_) {
            // The bend steers on the cross-track error leadTime ahead (tank_steer.hpp: the lead),
            // predicted from this tick's heading and the forward speed commanded LAST tick — the
            // same command the observer below compares against; 0 before the first command.
            // Nothing here reads a sensor (the pose is the Localizer's, already this tick's).
            const TankLineError line = tankLineError(pose, target_);
            const TankLineError ahead{
                .along = line.along,
                .cross = tankLeadCross(line, -errH, lastVxCmd_, cfg_.tankSteer)};
            const double delta = tankSteerAngle(ahead, cfg_.tankSteer);
            const double errHs =
                pose.heading().errorTo(target_.heading() + math::Angle::radians(delta));
            // The yaw observer: the DIFFERENCE between the yaw rate the robot made and the yaw
            // rate this loop asked for last tick (after the omega clamp, before desaturation),
            // low-passed and cancelled. That difference is a weaker side, a scrubbing wheel or a
            // push -- and also desaturation at full speed, motor lag, or rotation that is blocked.
            // Because it learns the difference and not the heading error, a turn the robot DOES
            // follow is not accumulated (a leg's own opening turn -- where an integral of the
            // heading error winds up, motion_tank_steer_test.cpp P5); while the robot cannot
            // follow, it integrates like one, bounded by yawCorrectionLimit, and decays within
            // about yawObserverTime once the robot rotates again (P8). The rate is the one the
            // Localizer read THIS tick (loc.twist(), finite-guarded there): the value the estimate
            // used, pinned by P14. (Before IMU1 a second ctx.imu().yawRate() would have been ~0
            // or a spike -- derived, never measured; ProsImu answers once per tick -- its header.)
            // It holds its estimate while the IMU is not ready.
            if (hasOmegaCmd_ && dt.value() > 0.0 && ctx.imu().isReady()) {
                const double unexplained = loc.twist().omega().value() - lastOmegaCmd_;
                const double a = dt.value() / (cfg_.tankSteer.yawObserverTime + dt.value());
                yawDisturbance_ = std::clamp(yawDisturbance_ + a * (unexplained - yawDisturbance_),
                                             -cfg_.tankSteer.yawCorrectionLimit,
                                             cfg_.tankSteer.yawCorrectionLimit);
            }
            w = pidH_.update(0.0, -errHs) - yawDisturbance_;
            // What TANK1 changed this tick: the bend's P-share plus the correction. Above the
            // legibility floor it lights the record's flag (TermSink " TSTR").
            const double contribution =
                cfg_.heading.kP * math::Angle::radians(errHs - errH).radians() - yawDisturbance_;
            tankActive = std::abs(contribution)
                         > kTankSteerNoiseFraction * cfg_.maxAngularSpeed.value();
            lastCross_ = line.cross;
            lastCrossAhead_ = ahead.cross;
            lastBend_ = delta;
        } else {
            w = pidH_.update(0.0, -errH);  // C1's line — every holonomic and un-engaged leg
        }

        // ── clamps → F1 rotation → authority clamp → wheels → volts: the ONE
        // shared choreography (command_pipeline.hpp — extracted from this file
        // at C4, arithmetic and order unchanged, so the Chassis facade's
        // drive() REUSES the pipeline instead of re-deriving it; the C2
        // bit-identity suites pin the extraction). The C3 strafe FALLBACK flag
        // rides back on the outcome: this engine's authority-limited
        // turn-WHILE-drive mode — translation proceeds at the achievable |vy|
        // while vx and ω stay at full authority; rotation is NEVER sequenced
        // before translation (that would be the LemLib behaviour this engine
        // exists to beat, C1's landmine). On the X-drive the flag is
        // structurally unreachable (authority 1.0 + the norm cap ⇒ |body vy| ≤
        // vyLimit, pinned by test); on the H-drive it engages exactly when a
        // leg out-demands the strafe wheel; on tank it marks any real lateral
        // demand as undeliverable — and on an engaged tank leg that demand is
        // what TANK1's bend (above) is steering out, so SFB lights less there,
        // beside TSTR. The flag observes the clamp; it never alters it (TANK1
        // changes only ω, never the translation command).
        const CommandOutcome cmd = applyCommandPipeline(
            deps_, cfg_, ff_,
            math::ChassisSpeeds{units::Velocity{vxF}, units::Velocity{vyF},
                                units::AngularVelocity{w}},
            math::Frame::Field, pose.heading());
        if (tankEngaged_) {
            // TANK1's observer compares NEXT tick's measured rate against THIS: the body ω after
            // the ω clamp and before desaturation (cmd.body). The clamp's own shortfall is
            // therefore never read as a disturbance; desaturation's is (bounded, as above).
            // The bend's lead predicts from this tick's body forward speed, from the same cmd.
            lastOmegaCmd_ = cmd.body.omega().value();
            lastVxCmd_ = cmd.body.vx().value();
            hasOmegaCmd_ = true;
        }

        // ── A3 containment: stall cross-check + health observables ────────────
        const auto motors = ctx.driveMotors();
        const bool stalled = stall_.update(now, motors, pose);
        tickHealthObservables(deps_, stalled);

        // ── one lazy record (A1): commanded = FINAL achievable cmd, FIELD frame
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r = baseRecord(ctx, loc, now, dt, pose, errX, errY, errH);
            r.commanded = math::robotToField(cmd.body, pose.heading());
            r.strafeFallbackActive = cmd.strafeFallback;  // C3 — never silent (TermSink "SFB")
            r.tankSteerActive = tankActive;               // TANK1 — never silent (TermSink "TSTR")
            for (std::size_t i = 0; i < motors.size()
                                    && i < static_cast<std::size_t>(diag::DebugRecord::kMaxWheels);
                 ++i) {
                r.wheelVoltage[i] = motors[i]->commandedVoltage();
                r.wheelCurrent[i] = motors[i]->current();
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

    /// The verdict cached by the last tick() or cancel() — Running until the first exit, then
    /// that exit reason for good. Reading it never recomputes anything and never advances the
    /// motion; only start() clears it back to Running.
    [[nodiscard]] control::ExitReason exitReason() const noexcept override { return reason_; }

    /// The motion-layer state, which is also written into DebugRecord.activeCommandState every
    /// tick: Idle before start(), WaitingForEstimate through the boot window, Running while
    /// controlling, then the state matching the verdict. Finer-grained than exitReason(),
    /// which cannot tell Idle from Running.
    [[nodiscard]] MotionState state() const noexcept override { return state_; }

    /// Always the literal "MoveToPose" — the string that identifies this motion in
    /// MotionTimeout fault text and in run result lines. The siblings override it with their
    /// own names, so a StrafeTo never reports as its base class.
    [[nodiscard]] const char* name() const noexcept override { return "MoveToPose"; }

    /// TANK1's account of the most recent tick that computed a command (all zero / false off a
    /// tank, and before the first live tick; the exit tick commands nothing, so after it this
    /// still describes the tick before): whether this motion may steer, whether this run does,
    /// and that tick's cross-track error, the predicted one the bend steered on, the bend and
    /// the yaw correction.
    [[nodiscard]] TankSteerStatus tankSteerStatus() const noexcept {
        return TankSteerStatus{.eligible = tankEligible_,
                               .engaged = tankEngaged_,
                               .cross = lastCross_,
                               .crossAhead = lastCrossAhead_,
                               .bend = lastBend_,
                               .yawCorrection = yawDisturbance_};
    }

    /// The FIELD-frame target (after any first-live-tick capture).
    [[nodiscard]] const math::Pose2d& target() const noexcept { return target_; }

    /// Retarget BEFORE start() (rebuilding a motion for a new waypoint).
    /// Precondition: not currently running.
    void setTarget(const math::Pose2d& target) {
        SHULIB_PRECONDITION(reason_ != control::ExitReason::Running
                                || state_ == MotionState::Idle,
                            "MoveToPose::setTarget: motion is running");
        SHULIB_PRECONDITION(std::isfinite(target.x().value())
                                && std::isfinite(target.y().value()),
                            "MoveToPose::setTarget: target position must be finite");
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
                        ? std::max(options.holdFor + kHoldSlack,
                                   timeout > 0.0 ? timeout : config.defaultTimeout)
                        : (timeout > 0.0 ? timeout : config.defaultTimeout),
                    deps.ctx->clock()},
          stall_{config.stall} {
        cfg_.validate();
        SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0,
                            "MoveToPose: timeout must be finite and >= 0");
        // A non-finite target would servo NaN volts for a full watchdog window
        // before the TimedOut exit — nonsense input, rejected loudly at
        // construction (added at C4; heading is an Angle, finite by its own
        // contract). Accepts strictly less than before: precondition-safe.
        SHULIB_PRECONDITION(std::isfinite(target.x().value())
                                && std::isfinite(target.y().value()),
                            "MoveToPose: target position must be finite");
        // TANK1's gate: the drivetrain has NO lateral authority, this motion is a plain
        // MoveToPose, and the feature is not switched off. Exactly == 0.0, not a type check:
        // "no lateral authority at all" is the property that makes a sideways error
        // uncorrectable by strafing, and it is the one honest signal — an H-drive whose strafe
        // wheel is declared dead (strafeTractionDerate 0) reports 0 too and IS a tank for this
        // purpose, which a TankKinematics check would miss; a type check would also bind this
        // layer to a concrete kinematics class.
        tankEligible_ = options.tankSteer && deps.kinematics->strafeAuthority() == 0.0
                        && cfg_.tankSteer.maxSteer > 0.0;
    }

    /// TANK1's one log line, at the first live tick of a tank leg that starts too far off its
    /// heading to be steered: the leg then runs exactly as before TANK1 and will most likely
    /// time out, and the author deserves to know why before reading the TIMEOUT. Built with
    /// the ONE §18.3 formatter (line_format.hpp); ASCII only.
    void warnNotSteered(double errH) {
        constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
        diag::lineformat::Line line;
        line.appendLiteral("tank moveTo not steered: it starts ");
        diag::lineformat::appendNum(line, std::abs(errH) * kRadToDeg, 1, 1);
        line.appendLiteral(" deg off its target heading (steers within ");
        diag::lineformat::appendNum(line, cfg_.tankSteer.maxSteer * kRadToDeg, 1, 1);
        line.appendLiteral(" deg) -- turn first: face()");
        deps_.ctx->telemetry().log(hal::LogLevel::Warn, "MOT", line.view());
    }

    /// Slack added to holdFor so a hold cannot be cut short by clock granularity. It is NOT
    /// the whole hold-mode budget: the watchdog is armed with max(holdFor + kHoldSlack, the
    /// effective timeout), because that same watchdog is the ONLY bound on the wait-for-live
    /// boot window, and holdFor + 1 s is not a boot budget. HoldPose(deps, holdFor = 0.5) used
    /// to have a total budget of 1.5 s against a ~2 s V5 IMU calibration, so it exited
    /// TimedOut before its hold window ever began — and HoldPose exposes no timeout knob, so
    /// motion.hpp's "callers budget timeouts to cover boot" was not something this caller
    /// could do.
    ///
    /// On the LIVE path in hold mode the watchdog is deliberately not consulted: holdStart_ +
    /// holdFor is the exit, and it is reached by the same clock. So the honest description is
    /// "the boot bound, plus a floor under holdFor" — not the clock-pathology backstop this
    /// comment used to claim, which was a live backstop the code never read.
    /// (kStrafeFallbackNoiseFraction moved to command_pipeline.hpp at C4,
    /// unchanged — the flag is computed where the clamp is applied.)
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
    bool tankEligible_ = false;    // TANK1: tank + plain MoveToPose + not switched off
    bool tankDecided_ = false;     // TANK1: the engage decision has been made this run
    bool tankEngaged_ = false;     // TANK1: this run steers (decided at the first live tick)
    double yawDisturbance_ = 0.0;  // TANK1: estimated unexplained yaw rate (rad/s), cancelled
    double lastOmegaCmd_ = 0.0;    // TANK1: last tick's commanded body yaw rate (rad/s)
    double lastVxCmd_ = 0.0;       // TANK1: last tick's commanded body forward speed (in/s)
    bool hasOmegaCmd_ = false;     // TANK1: lastOmegaCmd_ holds a real command
    double lastCross_ = 0.0;       // TANK1: last tick's cross-track error (in) — status only
    double lastCrossAhead_ = 0.0;  // TANK1: last tick's predicted cross (in) — status only
    double lastBend_ = 0.0;        // TANK1: last tick's bend (rad) — status only
};

}  // namespace shulib::motion
