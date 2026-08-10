#pragma once
//
// applyCommandPipeline — the ONE command path from a chassis-speeds demand to
// energized motors (chunk C4; extracted verbatim from MoveToPose's tick, where
// chunk C1 first assembled it).
//
// ── Why this is one function in one place ───────────────────────────────────────────
// C1's Freeze-Register note named the saturation choreography "the de-facto
// command path the facade's drive(ChassisSpeeds, Frame) verb must reuse, not
// re-derive — one pipeline, one place." Before C4 that choreography lived
// inside MoveToPose::tick() (with a degenerate copy in TurnTo); the facade
// would have needed a THIRD copy, and three copies of a clamp order is how a
// clamp order silently diverges. So C4 extracted it here — the motions and the
// facade now share one definition, and a bug fixed in the pipeline is fixed
// for every caller at once. (The C2 bit-identity suites — scheduled ==
// hand-chained, clean and hostile — pin that this extraction changed nothing:
// the arithmetic and its order are exactly C1's.)
//
// ── The choreography (MotionConfig header's saturation policy, F1/F5) ───────────────
//   1. |ω| clamped to maxAngularSpeed (scalar; a rate is frame-invariant).
//   2. (vx, vy) NORM-capped to maxLinearSpeed — uniformly, so the commanded
//      direction is preserved. The Euclidean norm is rotation-invariant, so
//      capping before or after the frame rotation is the same operation; it
//      happens here, once, for both input frames.
//   3. Frame::Field input only: math::fieldToRobot — THE one F1 rotation.
//      Frame::Body input skips it (already body-frame; rotating it would BE
//      the frame bug this parameter exists to prevent).
//   4. body |vy| clamped to strafeAuthority()·maxLinearSpeed — the upstream
//      clamp §13 #5 assigns to the MOTION layer; kinematics never clamps (F5).
//      The strafeFallback flag reports when the clamp BOUND meaningfully
//      (> kStrafeFallbackNoiseFraction·maxLinearSpeed removed — the C3
//      telemetry-visibility contract; rationale at the constant below).
//   5. IKinematics::toWheels — pure, unclamped inverse kinematics.
//   6. IKinematics::desaturate(maxWheelSpeed) — the downstream uniform scale.
//   7. Feedforward → compensateForBattery → IMotor::setVoltage, per wheel.
//
// The function COMMANDS THE MOTORS (step 7) — it is the pipeline, not a
// planner — and returns what it commanded so the caller can record it (the
// DebugRecord `commanded` field carries the final achievable command in the
// FIELD frame; callers convert via robotToField). It does NOT emit records,
// tick health, or evaluate exits: those stay with the caller, because the
// motion primitives and the facade's drive() legitimately differ there.
//
// Preconditions are the callers' (MotionConfig::validate, finite inputs):
// this function is the hot path and adds none of its own.

#include <algorithm>
#include <cmath>

#include "shulib/control/feedforward.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/math/frame.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/motion/motion.hpp"
#include "shulib/motion/motion_config.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// strafeFallbackActive's legibility floor, as a fraction of maxLinearSpeed:
/// the authority clamp must be removing more than this much lateral speed
/// before a tick is flagged as fallback. The floor exists so sub-perceptible
/// PID chatter near settle (or on tank, where the limit is 0) cannot light
/// the flag on every tick — a permanently-on flag is as undebuggable as a
/// silent one. At the HA-50 default budget this is 0.6 in/s — far below any
/// deliberate strafe, far above near-settle chatter. Telemetry-legibility
/// constant, host-decidable — not an A4 register entry (register rule 1).
/// (Moved here from MoveToPose at C4, unchanged, when the pipeline was
/// extracted — the flag is computed where the clamp is applied.)
inline constexpr double kStrafeFallbackNoiseFraction = 0.01;

/// What the pipeline commanded, for the caller's record.
struct CommandOutcome {
    /// The final achievable command in the BODY frame (post every clamp) —
    /// exactly what went into toWheels(). Record it via robotToField().
    math::ChassisSpeeds body{};
    /// True iff the strafe-authority clamp bound meaningfully this call (the
    /// C3 fallback contract — telemetry-visible, never silent).
    bool strafeFallback = false;
};

/// Run the full choreography above and command the motors. `command` is
/// expressed in `frame`; `heading` is the robot's current estimated heading
/// (used only for the Field→Body rotation — pass the pose the caller already
/// read this tick, so the whole tick acts on ONE snapshot).
[[nodiscard]] inline CommandOutcome applyCommandPipeline(const MotionDeps& deps,
                                                         const MotionConfig& cfg,
                                                         const control::Feedforward& ff,
                                                         const math::ChassisSpeeds& command,
                                                         math::Frame frame,
                                                         math::Angle heading) {
    // 1. ω clamp (frame-invariant).
    const double w = std::clamp(command.omega().value(), -cfg.maxAngularSpeed.value(),
                                cfg.maxAngularSpeed.value());

    // 2. uniform norm cap (rotation-invariant: same in either frame).
    const double maxLin = cfg.maxLinearSpeed.value();
    double vx = command.vx().value();
    double vy = command.vy().value();
    const double norm = std::hypot(vx, vy);
    if (norm > maxLin) {
        const double s = maxLin / norm;  // uniform: direction preserved
        vx *= s;
        vy *= s;
    }

    // 3. FIELD → BODY: the ONE frame rotation (F1) — Field input only.
    const math::ChassisSpeeds capped{units::Velocity{vx}, units::Velocity{vy},
                                     units::AngularVelocity{w}};
    const math::ChassisSpeeds body =
        (frame == math::Frame::Field) ? math::fieldToRobot(capped, heading) : capped;

    // 4. strafe-authority clamp: THIS layer clamps; kinematics never (F5).
    const double vyLimit = deps.kinematics->strafeAuthority() * maxLin;
    const math::ChassisSpeeds bodyClamped{
        body.vx(), units::Velocity{std::clamp(body.vy().value(), -vyLimit, vyLimit)},
        body.omega()};
    const bool strafeFallback =
        std::abs(body.vy().value()) - vyLimit > kStrafeFallbackNoiseFraction * maxLin;

    // 5–6. wheels: unclamped inverse kinematics, then the uniform desaturate.
    kinematics::WheelSpeeds wheels = deps.kinematics->toWheels(bodyClamped);
    wheels = deps.kinematics->desaturate(wheels, cfg.maxWheelSpeed);

    // 7. volts: feedforward, then the battery ceiling, per wheel.
    const units::Voltage vb = deps.ctx->battery().voltage();
    const auto motors = deps.ctx->driveMotors();
    for (int i = 0; i < wheels.size(); ++i) {
        const control::CompensatedVoltage cv =
            control::compensateForBattery(ff.calculate(wheels[i]), vb);
        motors[static_cast<std::size_t>(i)]->setVoltage(cv.voltage);
    }

    return CommandOutcome{.body = bodyClamped, .strafeFallback = strafeFallback};
}

}  // namespace shulib::motion
