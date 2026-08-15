#pragma once
//
// StrafeTo — translate to a FIELD (x, y) while HOLDING heading (chunk C1).
//
// The same decoupled three-axis engine as MoveToPose (one pipeline, one set of
// fixes) with the heading target CAPTURED AT THE FIRST LIVE TICK — the heading
// the robot actually has once the estimate exists, per the wait-for-live
// contract (motion.hpp): capturing at start() during the boot window would lock
// onto calibration garbage. The heading loop then actively HOLDS that capture
// throughout the translation (a pure strafe that lets heading wander is a
// decoupling failure, pinned by test).
//
// Drivetrain honesty: on a drive with strafeAuthority() == 0 (tank) the lateral
// component of the body command is clamped to zero by the engine's authority
// clamp — the motion physically cannot reach a laterally-offset target and
// exits TimedOut via the watchdog rather than hanging or lying. On the X-drive
// (authority 1.0) it strafes freely; C3's H-drive lands between.

#include "shulib/motion/move_to_pose.hpp"

namespace shulib::motion {

/// Translate to a FIELD (x, y) while actively HOLDING heading. The heading loop is not
/// switched off — it is given a target: the heading the robot actually has at the FIRST
/// LIVE tick, captured then rather than at start() so it can never lock onto a boot-window
/// estimate. So "hold" here means a closed loop that fights disturbance, not an absence of
/// rotation command, and a heading that wanders during the translation is a test failure.
/// The target heading is therefore NOT knowable before the motion runs: read target()
/// after the first live tick, and note that the heading component of the inherited
/// setTarget() is discarded by the same capture. On a drivetrain with no strafe authority
/// (tank) the body-lateral command is clamped to zero, so a laterally-offset target is
/// physically unreachable and the motion exits TimedOut rather than hanging or claiming a
/// success it never had.
class StrafeTo final : public MoveToPose {
public:
    /// Translate to FIELD (x, y), holding the first-live heading. `timeout` (s)
    /// bounds the whole motion including any boot wait; 0 selects
    /// config.defaultTimeout.
    StrafeTo(const MotionDeps& deps, units::Length x, units::Length y,
             const MotionConfig& config = {}, double timeout = 0.0)
        : MoveToPose(deps, math::Pose2d{x, y, math::Angle{}}, config, timeout,
                     PoseMotionOptions{.captureHeadingAtLive = true}) {}

    /// "StrafeTo". Overridden so the two places this string surfaces name THIS motion
    /// rather than the MoveToPose it is built from: the detail of the MotionTimeout fault
    /// the base raises, and the scheduler's CompletedMotion / run result line. Without it a
    /// strafe failure would be reported under the wrong primitive. It does NOT reach the
    /// DebugRecord stream — a record identifies its motion only by the scheduler-assigned
    /// activeCommandId, so filtering a blackbox for "StrafeTo" finds nothing.
    [[nodiscard]] const char* name() const noexcept override { return "StrafeTo"; }
};

}  // namespace shulib::motion
