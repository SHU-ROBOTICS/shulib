#pragma once
//
// HoldPose — actively hold a FIELD pose against disturbance (chunk C1).
//
// The same decoupled three-axis engine as MoveToPose in HOLD mode: it never
// settle-exits early — it keeps closing all three loops for `holdFor` seconds,
// driving back any disturbance (a shove, a defender, field contact), then
// reports:
//   * Settled  — the hold window ended AND the robot is currently within the
//                settle tolerances (it held its ground);
//   * TimedOut — the window ended OFF target (pushed away and not recovered) —
//                "held the clock out while 10 inches off" must never read as
//                success. MOTION_TIMEOUT is raised, same as any timeout.
//
// Two constructions:
//   * capture-current (the common one): the pose captured at the FIRST LIVE
//     tick (wait-for-live contract, motion.hpp) — "stay where you are".
//   * explicit pose: "hold THIS spot" (e.g. a scoring alignment).
//
// This is closed-loop position holding, distinct from IMotor BrakeMode::Hold
// (a per-motor firmware hold): HoldPose recovers a POSE, using the full
// holonomic authority of the drive. DriveBrake is the open-loop stop.

#include "shulib/motion/move_to_pose.hpp"

namespace shulib::motion {

/// Actively hold a FIELD pose against disturbance — a shove, a defender, field contact — by
/// running MoveToPose's three decoupled loops in HOLD mode. It NEVER exits early: the hold
/// window always runs to the end, and only then does being inside the settle tolerances
/// decide the verdict — Settled if the robot is on target AT THAT MOMENT, TimedOut (with
/// MOTION_TIMEOUT raised) if it was pushed away and never recovered, because holding the
/// clock out while 10 inches off is not success. The window is measured from the FIRST LIVE
/// tick, not from start(). Distinct from IMotor BrakeMode::Hold, a per-motor firmware hold
/// with no pose in it; DriveBrake is the open-loop stop.
class HoldPose final : public MoveToPose {
public:
    /// Hold the pose the robot has at the first live tick, for `holdFor` seconds.
    HoldPose(const MotionDeps& deps, double holdFor, const MotionConfig& config = {})
        : MoveToPose(deps, math::Pose2d{}, config, 0.0,
                     PoseMotionOptions{.capturePoseAtLive = true, .holdFor = holdFor}) {
        SHULIB_PRECONDITION(std::isfinite(holdFor) && holdFor > 0.0,
                            "HoldPose: holdFor must be finite and > 0");
    }

    /// Hold an explicit FIELD pose for `holdFor` seconds.
    HoldPose(const MotionDeps& deps, const math::Pose2d& pose, double holdFor,
             const MotionConfig& config = {})
        : MoveToPose(deps, pose, config, 0.0, PoseMotionOptions{.holdFor = holdFor}) {
        SHULIB_PRECONDITION(std::isfinite(holdFor) && holdFor > 0.0,
                            "HoldPose: holdFor must be finite and > 0");
    }

    /// The literal "HoldPose" — overriding the inherited "MoveToPose" so a result line or a
    /// MOTION_TIMEOUT fault names the hold that failed, not the engine it borrows.
    [[nodiscard]] const char* name() const noexcept override { return "HoldPose"; }
};

}  // namespace shulib::motion
