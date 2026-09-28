#pragma once
//
// tank_steer.hpp — the pure geometry of TANK1: where a tank is relative to its target's LINE,
// and how far MoveToPose bends the heading setpoint to steer it back (chunk TANK1, 2026-09).
//
// ── The line, and the two numbers ────────────────────────────────────────────────────
// The line runs through the target POINT along the target HEADING. In that line's frame:
//   along = the robot's signed distance along the target heading, measured FROM the target
//           (negative before the target on a forward approach, positive past it);
//   cross = the robot's signed distance off the line, + to the LEFT of the target heading.
// Neither uses a bearing to the target point: nothing here ever aims the robot AT the point.
// That is the difference from a pursuit or boomerang controller, and it is why TANK1 cannot
// plan a turn (C1's D12, amended by TANK1: bounded steering toward the author's LINE is
// allowed and telemetry-visible; turning toward the POINT is not).
//
// ── The bend ─────────────────────────────────────────────────────────────────────────
//   delta = −clamp(−along / fadeDistance, −1, 1) · clamp(crossGain · cross, ±maxSteer)
// The first factor is the fade AND the direction: +1 while the target is ahead along its
// heading (a forward approach), −1 while it is behind (a reverse approach — backing up, the
// nose must turn the OTHER way to bring the robot back), and continuous through 0 at arrival,
// so the bend vanishes exactly where the heading criterion is judged. There is no speed in any
// denominator: the classic Stanley singularity (gain ∝ 1/v as v → 0) cannot occur.
//
// ── The lead: steer on where the robot is GOING to be off the line ───────────────────
// The cross fed to the bend is not the error now but the error leadTime seconds ahead, if the
// robot kept its heading and the forward speed it was last commanded:
//   crossAhead = cross + leadTime · v · sin(heading − target heading)
// A tank moves along its heading, so v·sin(θ − θt) is exactly how fast it closes on (or leaves)
// the line; signed v makes the same formula right backing up. Why it is there (the Part A
// review, 2026-09-28, measured on the host test plant): the heading loop turns the robot with a
// lag (kP 4 ≈ 0.25 s, ~15 in at full speed), so a bend on the present error alone is an
// UNDER-DAMPED line loop — a leg that starts pointed at its line (10–15° off its target heading,
// its point on the arc the heading loop drives) ran at the line, crossed it near the target, and
// the fade then left 0.5–0.7 in nobody could remove: legs that finished before TANK1 timed out.
// The same root left holes in the lateral reach (a faced leg 50 in long settled a 3 in and a
// 7 in offset but timed out on 4–6 in). Steering on the predicted error damps it: nothing is
// computed from the target POINT, the bend keeps its cap, and leadTime 0 is the bend without it.
//
// ── The settle bound ─────────────────────────────────────────────────────────────────
// When a leg is within the translation tolerance (|along| ≤ 0.5 in and |cross| ≤ 0.5 in),
// |delta| ≤ (0.5/fadeDistance)·crossGain·|crossAhead|, which at the PROVISIONAL defaults is
// 0.0042 rad for |crossAhead| ≤ 0.5 in — about a fifth of the 0.02 rad heading tolerance — so the
// bend can never hold a settle open. The lead adds nothing there that matters: a leg in its
// tolerance is commanded at most translation.kP × 0.5 in = 1.5 in/s, and with the heading inside
// its own 0.02 rad the lead is at most leadTime × 1.5 × 0.02 = 0.002 in. Pinned over a grid by
// test/tank_steer_test.cpp.
//
// What uses this: MoveToPose (move_to_pose.hpp) on a drivetrain whose strafeAuthority() is
// exactly 0, for a leg that starts within maxSteer of its target heading — and nothing else.
// PROS-free, allocation-free, no state: every function here is pure.

#include <algorithm>
#include <cmath>

#include "shulib/math/pose2d.hpp"
#include "shulib/motion/motion_config.hpp"

namespace shulib::motion {

/// The robot's position in its target's line frame (header): inches.
struct TankLineError {
    double along = 0.0;  ///< along the target heading, from the target (− before it going forward)
    double cross = 0.0;  ///< off the line, + to the LEFT of the target heading
};

/// Where `pose` sits relative to the line through `target` along `target.heading()`.
[[nodiscard]] inline TankLineError tankLineError(const math::Pose2d& pose,
                                                 const math::Pose2d& target) noexcept {
    const double th = target.heading().radians();
    const double c = std::cos(th);
    const double s = std::sin(th);
    const double dx = pose.x().value() - target.x().value();
    const double dy = pose.y().value() - target.y().value();
    return TankLineError{.along = dx * c + dy * s, .cross = -dx * s + dy * c};
}

/// The cross-track error (in) `cfg.leadTime` seconds ahead (header: the lead): the error now
/// plus how far the robot closes on or leaves the line in that time if it keeps its heading and
/// `forwardSpeed` (in/s along its own heading, + forward, − backing up — MoveToPose passes the
/// body forward speed it commanded the tick before). `headingFromTarget` is the robot's heading
/// minus the target heading (rad; any wrap — only its sine is used). leadTime 0 returns e.cross.
[[nodiscard]] inline double tankLeadCross(const TankLineError& e, double headingFromTarget,
                                          double forwardSpeed,
                                          const TankSteerConfig& cfg) noexcept {
    return e.cross + cfg.leadTime * forwardSpeed * std::sin(headingFromTarget);
}

/// The bend (rad) TANK1 adds to the target heading for this line error (header formula) —
/// MoveToPose passes the error with its cross replaced by tankLeadCross(). Bounded by
/// cfg.maxSteer; zero at along == 0 and at cross == 0; odd in cross; its sign flips with the
/// direction of approach.
[[nodiscard]] inline double tankSteerAngle(const TankLineError& e,
                                           const TankSteerConfig& cfg) noexcept {
    const double direction = std::clamp(-e.along / cfg.fadeDistance, -1.0, 1.0);
    const double bend = std::clamp(cfg.crossGain * e.cross, -cfg.maxSteer, cfg.maxSteer);
    return -direction * bend;
}

/// Whether a tank leg whose FIRST LIVE heading error (target heading minus heading, shortest,
/// rad) is `startHeadingError` is steered at all: only when TANK1 is on (maxSteer > 0) and the
/// leg starts within maxSteer of its heading — inclusive, so a leg exactly at the window is
/// steered. A leg outside it is the author's turn (face()), and runs exactly as before TANK1.
[[nodiscard]] inline bool tankLegEngages(double startHeadingError,
                                         const TankSteerConfig& cfg) noexcept {
    return cfg.maxSteer > 0.0 && std::abs(startHeadingError) <= cfg.maxSteer;
}

/// What TANK1 did on a MoveToPose's most recent tick — for tests and Tier-3 tooling; the
/// record stream carries only the flag (DebugRecord::tankSteerActive).
struct TankSteerStatus {
    bool eligible = false;       ///< tank + plain MoveToPose + TANK1 on
    bool engaged = false;        ///< this run steers (decided at the first live tick)
    double cross = 0.0;          ///< last tick's cross-track error (in), + left of the line
    double crossAhead = 0.0;     ///< last tick's tankLeadCross() — the error the bend steered on
    double bend = 0.0;           ///< last tick's heading-setpoint bend (rad)
    double yawCorrection = 0.0;  ///< the yaw rate (rad/s) TANK1 subtracted last tick
};

/// tankSteerActive's legibility floor, as a fraction of maxAngularSpeed: TANK1 must have
/// changed the tick's yaw command (its bend's P-share plus its yaw correction) by more than this
/// before the tick is flagged. 2 % of the HA-50 budget is 0.12 rad/s; the floor in rad/s moves
/// with maxAngularSpeed. Measured on the host test plant (test/motion_tank_steer_test.cpp, P7, at
/// 6 rad/s): under composed hostility on an EVEN drive, over ten 40 in legs, the flag lights on 20
/// of 2 125 ticks (0.9 %; the worst single leg 2.9 %), where with no floor IMU rate noise reaching
/// the yaw observer lights it on 2 109 of them — nearly every tick; a 5 % weaker side lights it on
/// 81 of a leg's 186 ticks. (Re-measured 2026-09-28 with the bend's lead: 14 of 2 125 before
/// it.) The same kind of telemetry-legibility constant as kStrafeFallbackNoiseFraction —
/// host-decidable, not a register entry (A4 register rule 1).
inline constexpr double kTankSteerNoiseFraction = 0.02;

}  // namespace shulib::motion
