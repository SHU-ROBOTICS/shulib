#pragma once
//
// CurveFollow — the curve verbs' motion (chunk CURVE1, 2026-11): the robot drives a smooth path
// through the author's points WITHOUT stopping at them — pure pursuit on the path curve_path.hpp
// builds, a speed plan that slows before each bend, two guards that stop a curve the robot has
// lost, and an END that is an ordinary MoveToPose. Chassis::curveTo / reverseCurveTo and the
// Routine steps of the same names build one; a Tier-3 caller may too (from chassis.deps()).
//
// ── What C1's D12 says, and the one exception this file is ─────────────────────────────
// Every other motion keeps C1's D12 (amended by TANK1): the library never plans a turn toward a
// target and never aims at a point. By the team lead's ruling of 2026-10-30 (G3a's D16 (c)) this
// motion — and only it — steers toward a point: a point a look-ahead ahead of the robot ON THE PATH
// THE AUTHOR DREW, or, in the last look-ahead, on the end's own line extended past the end. That is
// pure pursuit, LemLib's `follow` — the behaviour D12 kept out of the core, admitted inside the two
// curve verbs only. It never aims at the END point itself, never plans a turn the path does not
// contain (a first point more than 90 deg off the nose, and a path that doubles back, are refused
// before the robot moves), and never turns round to drive backwards (reverse is the author's verb).
//
// ── The tick (the order is load-bearing: watchdog, end phase, guards, command) ──────────────
//   wait-for-live  zero volts while the estimate is Uninitialized; the watchdog runs through the
//                  wait (motion.hpp). At the FIRST live tick the path is BUILT from the live pose
//                  (never at construction: the boot window has no heading): the near-start drop
//                  (one Info line), checkCurve() — Invalid / StartBehind exit at once, TimedOut +
//                  MotionTimeout + one Warn naming the point ("curve not driven: ..."), the robot
//                  never moves; TightBend logs one Warn and drives — then ONE Info line ("curve: 4
//                  points, 143.4 in, tightest bend 7.4 in radius, bends at <= 21 in/s").
//   1. watchdog    FIRST, every live tick: a curve that runs out of time times out as CurveTo,
//                  never as its end's MoveToPose (whose own watchdog has the curve's FULL budget,
//                  armed later, so it can never fire first). The one consequence, stated: on the
//                  very tick the budget runs out, the curve exits TimedOut even if its end would
//                  have settled on that tick — the reverse of MoveToPose's own order (Settled
//                  beats a simultaneous TimedOut). One tick's difference; pinned (CV13).
//   2. end phase   once handed off, the end MoveToPose ticks and its verdict is the curve's.
//                  The hand-off fires on the FIRST of (path left = length - closest point):
//                    (1) path left <= handoffDistance AND heading within handoffHeading of the end
//                        heading AND within handoffCross of the end's line AND inside TANK1's
//                        window (below) — the clean hand-off;
//                    (2) path left <= handoffDistance x handoffLast AND tankLegEngages(heading
//                        error) — the SAME function MoveToPose decides its steering with, on the
//                        same pose, so the end is always steered (TANK1's 15 deg window). With
//                        TANK1 switched off (tankSteer.maxSteer = 0) the end MoveToPose never
//                        steers and never warns, so there is no window to wait for: (1) and (2)
//                        drop the window, (3) never warns;
//                    (3) the closest point reaches the end of the path (path left <= 0.125 in)
//                        still outside that window: handed off anyway, to a MoveToPose built with
//                        tankSteer.maxSteer = 0 (TANK1 off — the same unsteered line it would have
//                        driven, without its "turn first: face()" Warn, which would blame the
//                        author for a turn they never had to make), and ONE Warn in the curve's
//                        own words ("curve end: 21.1 deg off the end heading at the end of the
//                        path (steers within 15.0) -- unsteered; more run-up, a gentler last bend
//                        or a lower speed into it"; on a tank only — a holonomic end strafes).
//                  The end MoveToPose is built at the hand-off (std::optional, no heap) with the
//                  curve's deps, its MotionConfig copy and its FULL timeout, and ticked on the same
//                  tick, against the same pose. Its yaw observer starts from 0, like any moveTo's
//                  (carrying the curve's estimate in was probed on the host test plant and made it
//                  WORSE). So a curve ENDS like a moveTo: the same stop, the same arrival check, the
//                  same per-step arrival tolerance (a step's positionTolerance reaches it through
//                  the config, never as an option), and the same log.
//   3. guards      (pursuit only) the direction of travel more than lostHeading (60 deg) off the
//                  path's direction for lostTime (0.25 s), or less than 1 in of progress along the
//                  path in stuckTime (0.75 s) while more than handoffDistance (12 in) of path is
//                  left: stop (0 V), TimedOut, MotionTimeout, one Warn naming the guard ("curve
//                  lost: ..."). So the HEADING guard runs on every pursuit tick, the last 12 in
//                  included, until the hand-off; only the no-progress guard is off in the end zone
//                  (the end is slow on purpose). From the hand-off on, only the time budget stops
//                  the curve. NOT covered, stated: a side dying inside the end zone (the curve hands
//                  off, and the step's time budget ends it — travel stays bounded), and a jam on a
//                  robot whose odometry is its drive encoders (spinning wheels read as progress).
//                  A side dying EARLY can keep making progress while it pivots: on the host test
//                  plant (7 routes, either side, dead from 0.1-0.5 s) a guard stopped every such run
//                  within 2.7 s of the death and 9.5 in of travel after it (CV5).
//   4. command     s = the closest point, searched FORWARD only over searchWindow of path; the
//                  look-ahead point lookAhead further on; in the frame of the direction of travel
//                  (the robot's back's frame reversing) the arc through it has curvature
//                  k = 2y / (x^2 + y^2); v = min(the speed plan at s, endSpeedGain x translation.kP
//                  x path left, the last commanded speed + accel x dt); omega = v k minus TANK1's
//                  yaw observer's estimate (measured minus commanded yaw rate, low-passed — the same
//                  step as move_to_pose.hpp's, written once more here: curveYawObserverStep, pinned
//                  equal by test; held at 0 when TANK1 is switched off, tankSteer.maxSteer = 0). The command goes through applyCommandPipeline in the BODY frame
//                  with vy = 0 (the omega clamp, the uniform desaturate that keeps the arc), then
//                  OdoStallCheck and the health observables, exactly as MoveToPose.
//
// ── Records (A1: at most one per tick, lazy; motors stopped before an exit record) ────────
// targetPose = the END pose on every live record (so the result line's ` xt` is "off the end's
// line", and `ov` is along the start-to-end chord — read ` xt` and the end error for a curve),
// errors to the end pose, `commanded` the final achievable command in the FIELD frame,
// tankSteerActive when the yaw correction is above TANK1's legibility floor. From the hand-off on,
// the end MoveToPose emits the tick's record. The record schema does not change.
//
// Every tunable this reads is CurveConfig's (motion_config.hpp), PROVISIONAL (A4: HA-246,
// HA-248), and TankSteerConfig's maxSteer / yawObserverTime / yawCorrectionLimit (HA-179). Its own
// constants are logic constants, not hardware beliefs: kEndOfPath (0.125 in), kProgressStep (1 in)
// and kFirstTickDt (0.01 s). It reads the pose only through the Localizer: on a robot whose odometry is its drive
// encoders, how true that pose stays in a skid-steer bend is A4 register HA-249. Proven on the host
// test plant only: nothing of it has run on a robot.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <optional>
#include <span>

#include "shulib/control/feedforward.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/diag/line_format.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/command_pipeline.hpp"
#include "shulib/motion/curve_path.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/motion/move_to_pose.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/motion/tank_steer.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// TANK1's yaw observer step (move_to_pose.hpp, the engaged-leg branch), written once more for the
/// curve: the estimate moves toward the unexplained yaw rate (measured minus the rate commanded
/// last tick, after the omega clamp, before desaturation) by dt / (yawObserverTime + dt), clamped to
/// +-yawCorrectionLimit. The caller applies the same gate MoveToPose does (a command exists, dt > 0,
/// the IMU is ready) — otherwise the estimate is held. Identical arithmetic, pinned bit for bit by
/// test (motion_curve_test.cpp CV14), so a change to one is seen in the other.
[[nodiscard]] inline double curveYawObserverStep(double estimate, double measuredRate,
                                                 double commandedRate, double dt,
                                                 const TankSteerConfig& cfg) noexcept {
    const double unexplained = measuredRate - commandedRate;
    const double a = dt / (cfg.yawObserverTime + dt);
    return std::clamp(estimate + a * (unexplained - estimate), -cfg.yawCorrectionLimit,
                      cfg.yawCorrectionLimit);
}

/// Where a curve is.
enum class CurvePhase : std::uint8_t {
    Idle = 0,     ///< constructed, not started
    Waiting = 1,  ///< started, the estimate not yet live
    Pursuing = 2,  ///< following the path
    Ending = 3,   ///< handed off: the end MoveToPose is running
    Done = 4,     ///< exited (any verdict)
};

/// Which guard stopped a curve.
enum class CurveGuard : std::uint8_t {
    None = 0,      ///< none fired
    Heading = 1,   ///< more than lostHeading off the path for lostTime
    Progress = 2,  ///< under 1 in of progress in stuckTime, outside the end zone
};

/// Which hand-off started the end (the file header's (1)-(3)).
enum class CurveHandoff : std::uint8_t {
    None = 0,        ///< not handed off
    Clean = 1,       ///< (1) in the end zone, on the end's line, lined up
    LastResort = 2,  ///< (2) inside TANK1's steering window, handoffLast of the end zone left
    EndOfPath = 3,   ///< (3) the end of the path, outside the window (unsteered end, one Warn)
};

/// What a CurveFollow did on its most recent tick — for tests and Tier-3 tooling (the record
/// stream carries only the standard fields).
struct CurveStatus {
    CurvePhase phase = CurvePhase::Idle;  ///< where it is
    double s = 0.0;                ///< the closest point's arc length (in)
    double length = 0.0;           ///< the path's length (in); 0 before it is built
    double lookX = 0.0;            ///< the look-ahead point, field x (in)
    double lookY = 0.0;            ///< the look-ahead point, field y (in)
    double curvature = 0.0;        ///< the commanded arc's curvature (1/in)
    double speedLimit = 0.0;       ///< the speed plan at s (in/s)
    double commandedSpeed = 0.0;   ///< the forward speed asked for (in/s, along the travel)
    double yawCorrection = 0.0;    ///< the yaw observer's estimate (rad/s), subtracted
    double headingOffPath = 0.0;   ///< |direction of travel - path direction at s| (rad)
    CurveGuard guard = CurveGuard::None;      ///< the guard that fired, if any
    CurveCheck startCheck{};                  ///< checkCurve()'s verdict at the first live tick
    CurveHandoff handoff = CurveHandoff::None;  ///< which hand-off started the end
    double handoffHeadingError = 0.0;  ///< |heading error to the end heading| at the hand-off (rad)
};

/// Drive a smooth path through `points` (curve_path.hpp) and end with an ordinary MoveToPose
/// (the file header). On any drivetrain it drives like a tank — the nose (the back, reversing)
/// along the path, no sideways command. Owns no loop: tick it after updating the Localizer.
class CurveFollow : public IMotion {
public:
    /// The author's points (1..kMaxCurvePoints, finite — preconditions, checked here, before
    /// anything moves), the motion config (copied and validated), the timeout (seconds, the
    /// WHOLE curve including any boot wait and its end; 0 = config.defaultTimeout) and the
    /// direction (`reverse`: the robot's back leads; every heading is still the NOSE's).
    CurveFollow(const MotionDeps& deps, std::span<const CurvePoint> points,
                const MotionConfig& config = {}, double timeout = 0.0, bool reverse = false)
        : deps_{deps},
          cfg_{validatedConfig(config, timeout, "CurveFollow")},
          ff_{config.wheelFf},
          timeout_{timeout > 0.0 ? timeout : config.defaultTimeout},
          watchdog_{timeout_, deps.validatedClock()},
          stall_{config.stall},
          path_{points, reverse},
          reverse_{reverse} {
        end_ = path_.lastPoint();
    }

    /// Arm, or fully re-arm: the path is re-built at the next first live tick (from the pose then),
    /// the end MoveToPose is discarded, the guards, the observer and the stall check reset, the
    /// watchdog restarts. Commands no motors.
    void start() override {
        member_.reset();
        stall_.reset();
        watchdog_.start();
        reason_ = control::ExitReason::Running;
        state_ = MotionState::WaitingForEstimate;
        everLive_ = false;
        hasTick_ = false;
        lastTickTime_ = 0.0;
        end_ = path_.lastPoint();
        status_ = CurveStatus{};
        status_.phase = CurvePhase::Waiting;
        s_ = 0.0;
        yaw_ = 0.0;
        lastOmegaCmd_ = 0.0;
        lastSpeed_ = 0.0;
        hasOmegaCmd_ = false;
        lostSince_ = -1.0;
        bestS_ = 0.0;
        bestSAt_ = 0.0;
    }

    /// One control tick (the file header: watchdog, end phase, guards, command). Returns Running
    /// until the end MoveToPose settles (Settled), the watchdog fires, a guard fires or the path
    /// is refused (TimedOut, MotionTimeout raised naming this motion). After any non-Running verdict
    /// a no-op returning it. At most one DebugRecord per call.
    [[nodiscard]] control::ExitReason tick() override {
        SHULIB_PRECONDITION(state_ != MotionState::Idle, "CurveFollow::tick: start() not called");
        if (reason_ != control::ExitReason::Running) {
            return reason_;
        }
        chassis::RobotContext& ctx = *deps_.ctx;
        localization::Localizer& loc = *deps_.localizer;
        const units::Time now = ctx.clock().now();
        const units::Time dt = measuredDt(now);

        // ── the wait-for-live gate (motion.hpp contract) ──────────────────────────────────
        const bool live = loc.qualityClass() != localization::Localizer::Quality::Uninitialized;
        if (!live && !everLive_) {
            state_ = MotionState::WaitingForEstimate;
            stopMotors();
            if (watchdog_.expired()) {
                return exitTimedOut("timed out waiting for a live estimate");
            }
            emitWaitingRecord(now, dt);
            return control::ExitReason::Running;
        }
        if (!everLive_) {
            everLive_ = true;
            if (!buildAtFirstLiveTick(loc.pose())) {
                return exitRefused();
            }
        }
        state_ = MotionState::Running;

        // ── 1. the watchdog FIRST: the deadline belongs to the curve ──────────────────────
        if (watchdog_.expired()) {
            return exitTimedOut("curve end not reached");
        }

        // ── 2. the end phase ─────────────────────────────────────────────────────────────
        if (member_.has_value()) {
            return fromMember(member_->tick());
        }
        const math::Pose2d pose = loc.pose();
        s_ = path_.project(pose, s_, cfg_.curve.searchWindow);
        const double remaining = std::max(path_.length() - s_, 0.0);
        const double errH = pose.heading().errorTo(end_.heading());
        const double cross = std::abs(tankLineError(pose, end_).cross);
        const CurveConfig& cc = cfg_.curve;
        if (remaining <= cc.handoffDistance && std::abs(errH) <= cc.handoffHeading
            && cross <= cc.handoffCross && endSteerable(errH)) {
            return handOff(CurveHandoff::Clean, errH);
        }
        if (remaining <= cc.handoffDistance * cc.handoffLast && endSteerable(errH)) {
            return handOff(CurveHandoff::LastResort, errH);
        }
        if (remaining <= kEndOfPath) {
            return handOff(CurveHandoff::EndOfPath, errH);
        }

        // ── 3. the guards (pursuit only) ──────────────────────────────────────────────────
        const double travel = pose.heading().radians() + (reverse_ ? math::Angle::kPi : 0.0);
        const CurveSample here = path_.at(s_);
        const double off = std::abs(math::Angle::radians(travel).errorTo(
            math::Angle::radians(here.direction)));
        status_.headingOffPath = off;
        if (off > cc.lostHeading) {
            if (lostSince_ < 0.0) {
                lostSince_ = now.value();
            }
            if (now.value() - lostSince_ >= cc.lostTime) {
                return exitLost(CurveGuard::Heading, off, now.value() - lostSince_);
            }
        } else {
            lostSince_ = -1.0;
        }
        if (remaining > cc.handoffDistance) {
            if (s_ > bestS_ + kProgressStep) {
                bestS_ = s_;
                bestSAt_ = now.value();
            }
            if (now.value() - bestSAt_ > cc.stuckTime) {
                return exitLost(CurveGuard::Progress, s_ - bestS_, now.value() - bestSAt_);
            }
        }

        // ── 4. the command ────────────────────────────────────────────────────────────────
        const CurveSample aim = path_.lookAheadPoint(s_, cc.lookAhead);
        const double c = std::cos(travel);
        const double sn = std::sin(travel);
        const double dx = aim.x - pose.x().value();
        const double dy = aim.y - pose.y().value();
        const double bx = dx * c + dy * sn;    // ahead, in the frame of travel
        const double by = -dx * sn + dy * c;   // left, in the frame of travel
        const double l2 = bx * bx + by * by;
        const double kappa = l2 > 1e-9 ? 2.0 * by / l2 : 0.0;
        const double limit = path_.speedLimitAt(s_);
        const double rampDt = dt.value() > 0.0 ? dt.value() : kFirstTickDt;
        double v = std::min({limit, cc.endSpeedGain * cfg_.translation.kP * remaining,
                             lastSpeed_ + cc.accel * rampDt});
        v = std::max(v, 0.0);
        // TANK1's yaw observer, on the curve (the same gate and step as MoveToPose's). TANK1
        // switched off (tankSteer.maxSteer = 0) switches it off here too: the estimate stays 0.
        if (cfg_.tankSteer.maxSteer > 0.0 && hasOmegaCmd_ && dt.value() > 0.0
            && ctx.imu().isReady()) {
            yaw_ = curveYawObserverStep(yaw_, loc.twist().omega().value(), lastOmegaCmd_,
                                        dt.value(), cfg_.tankSteer);
        }
        const double w = v * kappa - yaw_;
        const double vx = reverse_ ? -v : v;
        const CommandOutcome cmd = applyCommandPipeline(
            deps_, cfg_, ff_,
            math::ChassisSpeeds{units::Velocity{vx}, units::Velocity{0.0},
                                units::AngularVelocity{w}},
            math::Frame::Body, pose.heading());
        lastOmegaCmd_ = cmd.body.omega().value();
        lastSpeed_ = std::abs(cmd.body.vx().value());
        hasOmegaCmd_ = true;
        status_.phase = CurvePhase::Pursuing;
        status_.s = s_;
        status_.lookX = aim.x;
        status_.lookY = aim.y;
        status_.curvature = kappa;
        status_.speedLimit = limit;
        status_.commandedSpeed = v;
        status_.yawCorrection = yaw_;

        const auto motors = ctx.driveMotors();
        const bool stalled = stall_.update(now, motors, pose);
        tickHealthObservables(deps_, stalled);

        const bool tankActive =
            std::abs(yaw_) > kTankSteerNoiseFraction * cfg_.maxAngularSpeed.value();
        hal::emitRecord(ctx.telemetry(), [&] {
            diag::DebugRecord r = baseRecord(now, dt, pose);
            r.commanded = math::robotToField(cmd.body, pose.heading());
            r.strafeFallbackActive = cmd.strafeFallback;
            r.tankSteerActive = tankActive;
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

    /// The cancel contract (motion.hpp): safe state whenever started, verdict only if still
    /// running, Idle untouched, idempotent, never raises. In the end phase the end MoveToPose is
    /// cancelled too — it applies the safe state and emits the exit record.
    void cancel() override {
        if (state_ == MotionState::Idle) {
            return;
        }
        if (reason_ != control::ExitReason::Running) {
            applyCancelSafeState(*deps_.ctx);
            return;
        }
        if (member_.has_value()) {
            member_->cancel();
        } else {
            applyCancelSafeState(*deps_.ctx);
            const units::Time now = deps_.ctx->clock().now();
            if (state_ == MotionState::WaitingForEstimate) {
                emitZeroErrorExitRecord(now);
            } else {
                emitExitRecord(now, units::Time{0.0}, deps_.localizer->pose());
            }
        }
        reason_ = control::ExitReason::Cancelled;
        state_ = MotionState::Cancelled;
        status_.phase = CurvePhase::Done;
    }

    /// The cached verdict (Running until the first exit).
    [[nodiscard]] control::ExitReason exitReason() const noexcept override { return reason_; }
    /// The motion-layer state (Running through the end phase too).
    [[nodiscard]] MotionState state() const noexcept override { return state_; }
    /// "CurveTo", or "ReverseCurveTo" — the name the result line and the MotionTimeout text use.
    [[nodiscard]] const char* name() const noexcept override {
        return reverse_ ? "ReverseCurveTo" : "CurveTo";
    }

    /// The most recent tick's account (phase, progress, the look-ahead, the guard, the start
    /// check, the hand-off).
    [[nodiscard]] const CurveStatus& curveStatus() const noexcept { return status_; }
    /// The path (built at the first live tick; before it, built() is false).
    [[nodiscard]] const CurvePath& path() const noexcept { return path_; }
    /// The END pose (the robot's nose): known once the path is built; before it, the last point
    /// with its heading (0 if it has none).
    [[nodiscard]] const math::Pose2d& endPose() const noexcept { return end_; }
    /// The end MoveToPose once handed off; null before.
    [[nodiscard]] const MoveToPose* endMotion() const noexcept {
        return member_.has_value() ? &*member_ : nullptr;
    }

private:
    /// Path left (in) at which the closest point counts as the end of the path (hand-off (3)):
    /// half the 0.25 in grid the host-test-plant probe measured on — an exact 0 could be approached
    /// for ever while the speed fades with the path left. A logic constant, not a hardware belief.
    static constexpr double kEndOfPath = 0.125;
    /// Progress (in) that counts as progress for the no-progress guard.
    static constexpr double kProgressStep = 1.0;
    /// The speed ramp's step on a first command tick that has no measured dt yet (s): the 100 Hz
    /// loop's period, so the first command is one tick of acceleration, not zero.
    static constexpr double kFirstTickDt = 0.01;

    /// The end MoveToPose would steer from heading error `errH` — inside TANK1's window, judged by
    /// tankLegEngages(), the function MoveToPose itself decides with — or TANK1 is switched off
    /// (tankSteer.maxSteer = 0), when MoveToPose never steers and never warns, so there is no
    /// window to wait for.
    [[nodiscard]] bool endSteerable(double errH) const noexcept {
        return cfg_.tankSteer.maxSteer == 0.0 || tankLegEngages(errH, cfg_.tankSteer);
    }

    /// The first live tick: build the path from the live pose, log the drop and the plan, decide.
    /// False when the path is refused (the caller exits).
    bool buildAtFirstLiveTick(const math::Pose2d& pose) {
        path_.build(pose, cfg_.curve, cfg_.maxLinearSpeed.value(), cfg_.maxAngularSpeed.value());
        end_ = path_.endPose();
        bestS_ = 0.0;
        bestSAt_ = deps_.ctx->clock().now().value();
        status_.length = path_.length();
        namespace lf = diag::lineformat;
        if (path_.droppedFirstDistance() >= 0.0) {
            lf::Line line;
            line.appendLiteral("curve: point 1 is ");
            lf::appendNum(line, path_.droppedFirstDistance(), 1, 1);
            line.appendLiteral(" in from the robot -- dropped; do not list the start");
            deps_.ctx->telemetry().log(hal::LogLevel::Info, "MOT", line.view());
        }
        const CurveCheck check = checkCurve(path_, cfg_.curve);
        status_.startCheck = check;
        if (check.verdict == CurveVerdict::Invalid || check.verdict == CurveVerdict::StartBehind) {
            return false;
        }
        if (check.verdict == CurveVerdict::TightBend) {
            lf::Line line;
            appendCurveCheckWords(line, check, cfg_.curve);
            deps_.ctx->telemetry().log(hal::LogLevel::Warn, "MOT", line.view());
        }
        lf::Line line;
        line.appendLiteral("curve: ");
        lf::appendUnsigned(line, static_cast<unsigned long>(path_.segments()));
        line.appendLiteral(path_.segments() == 1 ? " point, " : " points, ");
        lf::appendNum(line, path_.length(), 1, 1);
        line.appendLiteral(" in, ");
        if (std::isfinite(path_.tightestRadius())) {
            line.appendLiteral("tightest bend ");
            lf::appendNum(line, path_.tightestRadius(), 1, 1);
            line.appendLiteral(" in radius, bends at <= ");
            lf::appendNum(line, path_.speedLimitAt(path_.tightestAt()), 1, 0);
            line.appendLiteral(" in/s");
        } else {
            line.appendLiteral("no bend");
        }
        deps_.ctx->telemetry().log(hal::LogLevel::Info, "MOT", line.view());
        status_.phase = CurvePhase::Pursuing;
        return true;
    }

    /// Hand the last inches to an ordinary MoveToPose (the file header's (1)-(3)), and tick it now.
    control::ExitReason handOff(CurveHandoff kind, double errH) {
        MotionConfig endCfg = cfg_;
        if (kind == CurveHandoff::EndOfPath && !endSteerable(errH)) {
            endCfg.tankSteer.maxSteer = 0.0;  // TANK1 off: the same unsteered line, minus its Warn
            if (deps_.kinematics->strafeAuthority() == 0.0) {
                warnEndUnsteered(errH);
            }
        }
        member_.emplace(deps_, end_, endCfg, timeout_);
        member_->start();
        status_.phase = CurvePhase::Ending;
        status_.handoff = kind;
        status_.handoffHeadingError = std::abs(errH);
        status_.s = s_;
        return fromMember(member_->tick());
    }

    /// The end MoveToPose's verdict is the curve's (it has already stopped the motors and emitted
    /// the tick's record).
    control::ExitReason fromMember(control::ExitReason r) {
        if (r == control::ExitReason::Running) {
            return r;
        }
        reason_ = r;
        state_ = r == control::ExitReason::Settled    ? MotionState::Settled
                 : r == control::ExitReason::Cancelled ? MotionState::Cancelled
                                                       : MotionState::TimedOut;
        status_.phase = CurvePhase::Done;
        return reason_;
    }

    void warnEndUnsteered(double errH) {
        constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
        diag::lineformat::Line line;
        line.appendLiteral("curve end: ");
        diag::lineformat::appendNum(line, std::abs(errH) * kRadToDeg, 1, 1);
        line.appendLiteral(" deg off the end heading at the end of the path (steers within ");
        diag::lineformat::appendNum(line, cfg_.tankSteer.maxSteer * kRadToDeg, 1, 1);
        line.appendLiteral(") -- unsteered; more run-up, a gentler last bend or a lower speed into it");
        deps_.ctx->telemetry().log(hal::LogLevel::Warn, "MOT", line.view());
    }

    control::ExitReason exitRefused() {
        diag::lineformat::Line line;
        appendCurveCheckWords(line, status_.startCheck, cfg_.curve);
        deps_.ctx->telemetry().log(hal::LogLevel::Warn, "MOT", line.view());
        return exitWith(control::ExitReason::TimedOut, line);
    }

    control::ExitReason exitLost(CurveGuard guard, double value, double held) {
        constexpr double kRadToDeg = 180.0 / math::Angle::kPi;
        status_.guard = guard;
        diag::lineformat::Line line;
        if (guard == CurveGuard::Heading) {
            line.appendLiteral("curve lost: heading ");
            diag::lineformat::appendNum(line, value * kRadToDeg, 1, 0);
            line.appendLiteral(" deg off the path for ");
            diag::lineformat::appendNum(line, held, 1, 2);
            line.appendLiteral(" s");
        } else {
            line.appendLiteral("curve lost: ");
            diag::lineformat::appendNum(line, std::max(value, 0.0), 1, 1);
            line.appendLiteral(" in of progress in ");
            diag::lineformat::appendNum(line, held, 1, 2);
            line.appendLiteral(" s");
        }
        deps_.ctx->telemetry().log(hal::LogLevel::Warn, "MOT", line.view());
        return exitWith(control::ExitReason::TimedOut, line);
    }

    control::ExitReason exitTimedOut(const char* why) {
        diag::lineformat::Line line;
        line.appendLiteral(why);
        return exitWith(control::ExitReason::TimedOut, line);
    }

    /// Stop, record the verdict, raise MotionTimeout (naming this motion) with `why`, and emit
    /// the exit record — motors stopped BEFORE it.
    control::ExitReason exitWith(control::ExitReason r, const diag::lineformat::Line& why) {
        stopMotors();
        reason_ = r;
        state_ = MotionState::TimedOut;
        status_.phase = CurvePhase::Done;
        deps_.faults->raise(diag::FaultCode::MotionTimeout, name(), why.view());
        emitExitRecord(deps_.ctx->clock().now(), units::Time{0.0}, deps_.localizer->pose());
        return reason_;
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

    [[nodiscard]] diag::DebugRecord baseRecord(units::Time now, units::Time dt,
                                               const math::Pose2d& pose) const {
        chassis::RobotContext& ctx = *deps_.ctx;
        const localization::Localizer& loc = *deps_.localizer;
        diag::DebugRecord r;
        r.t = now;
        r.dt = dt;
        r.targetPose = end_;
        r.measuredPose = pose;
        r.errorX = units::Length{end_.x().value() - pose.x().value()};
        r.errorY = units::Length{end_.y().value() - pose.y().value()};
        r.errorHeading = units::AngleDim{pose.heading().errorTo(end_.heading())};
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
        hal::emitRecord(deps_.ctx->telemetry(), [&] {
            diag::DebugRecord r = baseRecord(now, dt, deps_.localizer->pose());
            // No estimate to err against, nothing commanded: the record must not invent.
            r.errorX = units::Length{0.0};
            r.errorY = units::Length{0.0};
            r.errorHeading = units::AngleDim{0.0};
            r.commanded = math::ChassisSpeeds{};
            return r;
        });
    }

    void emitZeroErrorExitRecord(units::Time now) {
        hal::emitRecord(deps_.ctx->telemetry(), [&] {
            diag::DebugRecord r = baseRecord(now, units::Time{0.0}, deps_.localizer->pose());
            r.errorX = units::Length{0.0};
            r.errorY = units::Length{0.0};
            r.errorHeading = units::AngleDim{0.0};
            r.commanded = math::ChassisSpeeds{};
            return r;
        });
    }

    void emitExitRecord(units::Time now, units::Time dt, const math::Pose2d& pose) {
        hal::emitRecord(deps_.ctx->telemetry(), [&] {
            diag::DebugRecord r = baseRecord(now, dt, pose);
            r.commanded = math::ChassisSpeeds{};  // motors are stopped on exit
            return r;
        });
    }

    MotionDeps deps_;
    MotionConfig cfg_;
    control::Feedforward ff_;
    double timeout_;
    control::Watchdog watchdog_;
    OdoStallCheck stall_;
    CurvePath path_;
    std::optional<MoveToPose> member_;
    math::Pose2d end_{};
    CurveStatus status_{};
    control::ExitReason reason_ = control::ExitReason::Running;
    MotionState state_ = MotionState::Idle;
    double s_ = 0.0;
    double yaw_ = 0.0;           // TANK1's observer on the curve: unexplained yaw rate (rad/s)
    double lastOmegaCmd_ = 0.0;  // last tick's body yaw rate after the omega clamp (rad/s)
    double lastSpeed_ = 0.0;     // last tick's |body forward speed| after the pipeline (in/s)
    double lostSince_ = -1.0;    // when the heading guard's condition began (s); < 0 = not now
    double bestS_ = 0.0;         // the progress guard's last 1 in step (in) ...
    double bestSAt_ = 0.0;       // ... and when it was made (s)
    double lastTickTime_ = 0.0;
    bool hasTick_ = false;
    bool hasOmegaCmd_ = false;
    bool everLive_ = false;
    bool reverse_ = false;
};

/// The size rule (HA-250): a blocking curve verb's motion lives on the calling task's stack (the
/// auton task's 32 KiB). Counted: the path ~3.1 KB + the optional end MoveToPose ~1.5 KB + this
/// motion's MotionConfig copy ~0.5 KB + state — about 5.4 KB on the host. The ARM -fstack-usage
/// frame of a 16-point curveTo is the real gate (logged by the chunk; flagged above 8 KB).
static_assert(sizeof(CurveFollow) <= 6144, "CurveFollow outgrew its stated stack budget (HA-250)");

}  // namespace shulib::motion
