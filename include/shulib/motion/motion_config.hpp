#pragma once
//
// MotionConfig — the shared knobs of the C1 motion primitives.
//
// ── EVERY DEFAULT HERE IS PROVISIONAL (Phase C's opening rule) ──────────────────────
// The A2 plant proves control LOGIC, not CONSTANTS. Gains below converge on the
// plant's placeholder dynamics; R5 measures real kS/kV/kA and re-tunes, R6
// back-fits the plant. Register entries: HA-50 (gains + speed budget), HA-51
// (settle tolerances), HA-52 (stall cross-check + stand-in radii). Do not read
// any number here as a measurement.
//
// ── Unit discipline (constraint: this layer owns unit consistency) ──────────────────
// Pid is bare-double by design. Each axis's gains are documented WITH their
// units and never shared across dimensions:
//   * translation: error INCHES → command IN/S      (kP in 1/s)
//   * heading:     error RADIANS → command RAD/S    (kP in 1/s)
// One translation gain set serves BOTH field axes deliberately: the x/y
// controllers act on FIELD coordinates, and unequal gains would make the
// closed-loop behaviour depend on which way the FIELD is oriented — breaking
// the rotational equivariance the frame tests pin. (An axis-asymmetric robot is
// a BODY-frame property; it belongs in kinematics/feedforward, not here.)
//
// ── Saturation policy (who clamps what — F5 choreography) ───────────────────────────
//   1. |ω| clamped to maxAngularSpeed (scalar).
//   2. (vx, vy) FIELD demand norm-capped to maxLinearSpeed — UNIFORMLY, so the
//      commanded direction is preserved (per-axis clamps would curve diagonals).
//   3. After fieldToRobot: BODY |vy| clamped to strafeAuthority()·maxLinearSpeed
//      (the upstream clamp §13 #5 assigns to the motion layer; authority is a
//      READ-ONLY query — kinematics itself never clamps).
//   4. IKinematics::desaturate(…, maxWheelSpeed) — the downstream uniform scale.
//   5. compensateForBattery() — the final per-wheel hardware ceiling.
// maxWheelSpeed's default keeps Feedforward(maxWheelSpeed) inside the 12 V rail
// with margin, so step 5 engages only under genuine sag (per-wheel clamping
// distorts direction, so routine operation must not rely on it).
//
// Settle-rate floors vs sensor noise (why headingSettle.maxErrorRate = 0.3):
// hostile IMU heading noise σ ≈ 0.05° (HA-21) differentiates to ≈ 0.12 rad/s of
// MEASURED rate noise at 100 Hz — a tighter rate bound would flap the settle
// window under hostility without the robot moving at all. 0.3 rad/s clears the
// noise floor ~2.5σ while still rejecting real residual rotation.

#include <cmath>
#include <limits>

#include "shulib/control/feedforward.hpp"
#include "shulib/control/settled_util.hpp"
#include "shulib/core/check.hpp"
#include "shulib/motion/odo_stall_check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// Per-axis PID gains (units documented at each use site). Output saturation is
/// deliberately NOT here — the motion layer's norm/ω caps own it (header note).
struct AxisGains {
    double kP = 0.0;  ///< Proportional gain, 1/s on both axes: in→in/s, rad→rad/s.
    double kI = 0.0;  ///< Integral gain, 1/s². 0 (the default) makes the axis pure-P.
    double kD = 0.0;  ///< Derivative gain (dimensionless), on the MEASUREMENT — no setpoint kick.
    /// Symmetric ± clamp on the I-TERM (kI·∫e dt) in command units, with the accumulator
    /// back-calculated so it cannot wind up past the clamp. Infinity means unclamped,
    /// which is only safe while kI is 0 — the default pairing. Must be ≥ 0.
    double integralLimit = std::numeric_limits<double>::infinity();
};

/// Every knob the C1 motion primitives share. A motion COPIES it at construction and
/// validate()s the copy, so later edits to the object you built from never reach a live
/// motion — build a fresh config, then a fresh motion. Units are canonical throughout
/// (inches, radians, seconds), but only the speed and geometry budgets carry theirs in the
/// TYPE (units::Velocity / AngularVelocity / Length); the gains, defaultTimeout and every
/// SettleConfig / OdoStallCheckConfig field are bare doubles whose units live only in the
/// comment beside them. Nor are the gains dimensionless — kP is 1/s and kI 1/s², kD alone
/// is dimensionless — what the axis they are handed to supplies is WHICH quantity they act
/// on (inches for translation, radians for heading), not their dimension.
struct MotionConfig {
    /// Wheel feedforward — MUST match the drivetrain's characterization (R5).
    /// Default mirrors the plant's placeholder (≈70 in/s free speed at 12 V).
    /// PROVISIONAL (A4: HA-45/HA-50).
    control::FeedforwardGains wheelFf{.kS = 1.0, .kV = 12.0 / 70.0, .kA = 0.0};

    /// Translation: inches of field-axis error → in/s of field-axis velocity
    /// command. Applied identically to x AND y (header note). PROVISIONAL (HA-50).
    AxisGains translation{.kP = 3.0};
    /// Heading: radians of shortest-path error → rad/s. PROVISIONAL (HA-50).
    AxisGains heading{.kP = 4.0};

    /// Field-frame linear speed budget (in/s) — the norm cap AND the base of the
    /// strafe-authority clamp. PROVISIONAL (HA-50).
    units::Velocity maxLinearSpeed{60.0};
    /// Yaw-rate budget (rad/s). PROVISIONAL (HA-50).
    units::AngularVelocity maxAngularSpeed{6.0};
    /// Per-wheel surface-speed budget for desaturate() (in/s). PROVISIONAL (HA-50).
    units::Velocity maxWheelSpeed{60.0};

    /// Translation settle: |pos error| (in), |d error/dt| (in/s), held (s).
    /// PROVISIONAL (A4: HA-51).
    control::SettleConfig translationSettle{.maxError = 0.5, .maxErrorRate = 1.0,
                                            .settleTime = 0.1};
    /// Heading settle: |shortest error| (rad ≈ 1.15°), rate (rad/s — noise floor
    /// note in header), held (s). PROVISIONAL (A4: HA-51).
    control::SettleConfig headingSettle{.maxError = 0.02, .maxErrorRate = 0.30,
                                        .settleTime = 0.1};
    /// DriveBrake settle on the AVERAGED speed norm |v| + rotationRadius·|ω|
    /// (in/s), its rate (in/s²), held (s). The threshold sits deliberately
    /// ABOVE the M2 estimator's averaged twist-noise floor (~0.3–0.9 in/s at a
    /// physical dead stop under composed hostility — drive_brake.hpp header);
    /// tighter would never settle on a hostile field. PROVISIONAL (A4: HA-51).
    control::SettleConfig brakeSettle{.maxError = 1.2, .maxErrorRate = 100.0,
                                      .settleTime = 0.1};

    /// Watchdog default when a motion is constructed without an explicit
    /// timeout (seconds). PROVISIONAL (A4: HA-51).
    double defaultTimeout = 5.0;

    /// Center-to-wheel distance (in) — converts |ω| to an equivalent linear
    /// speed in DriveBrake's norm. Stand-in geometry (A4: HA-17/HA-52).
    units::Length rotationRadius{7.0};

    /// The spin-vs-motion cross-check thresholds (A4: HA-52).
    OdoStallCheckConfig stall{};

    /// Re-check the invariants the motions rely on and RAISE on the first violation:
    /// feedforward and PID gains finite, integral limits non-negative, and all FIVE speed /
    /// timeout / geometry scalars strictly positive (maxLinearSpeed, maxAngularSpeed,
    /// maxWheelSpeed, defaultTimeout, rotationRadius — 0 is rejected, never read as
    /// "unset"). Every C1 motion calls this from its own constructor, so it is a backstop
    /// rather than a step you can forget — call it yourself only when validating a config
    /// you have not yet handed to a motion.
    /// It deliberately does NOT descend into the SettleConfig or OdoStallCheckConfig
    /// members: those are checked by SettledUtil and OdoStallCheck when the motion builds
    /// them, which is the only place their own invariants are known.
    void validate() const {
        auto finiteGains = [](const AxisGains& g) {
            return std::isfinite(g.kP) && std::isfinite(g.kI) && std::isfinite(g.kD)
                   && g.integralLimit >= 0.0;
        };
        SHULIB_PRECONDITION(std::isfinite(wheelFf.kS) && std::isfinite(wheelFf.kV)
                                && std::isfinite(wheelFf.kA),
                            "MotionConfig: wheelFf gains must be finite");
        SHULIB_PRECONDITION(finiteGains(translation), "MotionConfig: translation gains invalid");
        SHULIB_PRECONDITION(finiteGains(heading), "MotionConfig: heading gains invalid");
        SHULIB_PRECONDITION(maxLinearSpeed.value() > 0.0,
                            "MotionConfig: maxLinearSpeed must be > 0");
        SHULIB_PRECONDITION(maxAngularSpeed.value() > 0.0,
                            "MotionConfig: maxAngularSpeed must be > 0");
        SHULIB_PRECONDITION(maxWheelSpeed.value() > 0.0,
                            "MotionConfig: maxWheelSpeed must be > 0");
        SHULIB_PRECONDITION(defaultTimeout > 0.0, "MotionConfig: defaultTimeout must be > 0");
        SHULIB_PRECONDITION(rotationRadius.value() > 0.0,
                            "MotionConfig: rotationRadius must be > 0");
        // SettleConfig / OdoStallCheckConfig fields are validated by their owners
        // (SettledUtil / OdoStallCheck) at construction.
    }
};

}  // namespace shulib::motion
