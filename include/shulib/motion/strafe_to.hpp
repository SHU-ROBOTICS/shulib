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

class StrafeTo final : public MoveToPose {
public:
    /// Translate to FIELD (x, y), holding the first-live heading. `timeout` (s)
    /// bounds the whole motion including any boot wait; 0 selects
    /// config.defaultTimeout.
    StrafeTo(const MotionDeps& deps, units::Length x, units::Length y,
             const MotionConfig& config = {}, double timeout = 0.0)
        : MoveToPose(deps, math::Pose2d{x, y, math::Angle{}}, config, timeout,
                     PoseMotionOptions{.captureHeadingAtLive = true}) {}

    [[nodiscard]] const char* name() const noexcept override { return "StrafeTo"; }
};

}  // namespace shulib::motion
