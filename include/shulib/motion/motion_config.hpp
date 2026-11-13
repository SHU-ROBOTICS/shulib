#pragma once
//
// MotionConfig — the shared knobs of the C1 motion primitives.
//
// ── EVERY DEFAULT HERE IS PROVISIONAL (Phase C's opening rule) ──────────────────────
// The A2 plant proves control LOGIC, not CONSTANTS. Gains below converge on the
// plant's placeholder dynamics; R5 measures real kS/kV/kA and re-tunes, R6
// back-fits the plant. Register entries: HA-50 (gains + speed budget), HA-51
// (settle tolerances), HA-52 (stall cross-check + stand-in radii), HA-179 (TANK1's
// tank steering, TankSteerConfig). Do not read any number here as a measurement.
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

/// How MoveToPose steers a TANK (a drivetrain whose strafeAuthority() is exactly 0) back onto
/// its target's LINE while it drives — chunk TANK1. The line runs through the target point along
/// the target heading (for Routine::driveTo that is the start-to-target line). On any drivetrain
/// with strafe authority this struct is never read, so X-drive and H-drive motion is unchanged
/// bit for bit. Every value is PROVISIONAL (A4: HA-179) — host-test-plant numbers, not a robot's.
///
/// What it does, in one sentence per knob: the heading MoveToPose servoes to is the target
/// heading bent by crossGain × (inches off the line, as predicted leadTime seconds ahead), never
/// by more than maxSteer, faded to nothing over the last fadeDistance inches of the leg; and
/// while it steers, it MEASURES the
/// difference between the yaw rate the robot made (the Localizer's reading of the IMU, this
/// tick) and the yaw rate the heading loop asked for the tick before (after the omega clamp,
/// before desaturation), low-passed over yawObserverTime, and commands the opposite, never more
/// than yawCorrectionLimit — which is what cancels the steady turn of a weaker drive side. That
/// difference also contains desaturation at full speed, motor lag and blocked rotation: while
/// the robot cannot follow a yaw command, the correction integrates like a heading integral, up
/// to yawCorrectionLimit, and decays within about yawObserverTime once the robot turns again
/// (HA-179's blast radius). What it never does: plan a turn, or aim at the target POINT. A leg
/// that STARTS more than maxSteer off its target heading is not steered at all (it runs exactly
/// as it did before TANK1, and one Warn line says why) — face() stays your turn.
struct TankSteerConfig {
    /// Heading bend per inch of cross-track error (rad/in). PROVISIONAL (A4: HA-179).
    double crossGain = 0.1;
    /// The cap on the bend AND the engage window (rad, 15 degrees): the heading setpoint never
    /// leaves target heading ± maxSteer, and a leg whose first live heading is farther than this
    /// from its target heading is not steered. The two are ONE number on purpose: a leg that
    /// starts within the cap is one whose heading error the bend itself could have commanded;
    /// beyond it, steering would mean the heading loop first making the author's turn. So
    /// raising it to let the bend remove larger offsets also widens which un-faced legs are
    /// steered — on the host test plant un-faced legs arrive up to about 30 degrees off and time
    /// out from about 40, a cliff the author cannot see, so keep it well under 30 (HA-179).
    /// 0 turns the steering off entirely: a tank's moveTo then moves, records and ends exactly as
    /// it did before TANK1, bit for bit (its result line keeps TANK1 Part B's ` xt` column, a
    /// report kept whatever maxSteer is); a curve (CURVE1) then hands off without waiting for a
    /// window and holds its own yaw correction at 0. Must be < pi/2 (a bend of 90 degrees or more would reverse which
    /// way the robot drives along the line). PROVISIONAL (A4: HA-179).
    double maxSteer = 0.2617993877991494;
    /// Along-track distance over which the bend fades to zero at arrival (in, > 0): the bend is
    /// scaled by clamp(along-track distance to go / fadeDistance, -1, 1), which is continuous
    /// through arrival and carries the sign that makes a REVERSE approach steer the right way.
    /// PROVISIONAL (A4: HA-179).
    double fadeDistance = 6.0;
    /// How far ahead (s) the bend looks: it steers on the cross-track error predicted this far
    /// ahead from the robot's heading and the forward speed it was commanded the tick before
    /// (tank_steer.hpp, tankLeadCross), not on the error now. It damps the line loop: without it
    /// a leg that starts pointed at its line can overshoot the line near the target, where the
    /// fade leaves the rest unremovable — a leg that finished before TANK1 then timed out. On the
    /// host test plant (2026-09-28, the Part A review's grid of 24 563 legs) no lead from 0.05 to
    /// 0.10 s left any leg that settles with TANK1 off timing out with it on, at motor time
    /// constants from 0 to about 0.18 s; with no lead 178 did (0.04 s: 4; 0.25 s under lag:
    /// 72–102). 0.06 sits inside that band and keeps a 6 in offset over 40 in removable on the
    /// lag-free plant. >= 0 (0 = steer on the error now). PROVISIONAL (A4: HA-179).
    double leadTime = 0.06;
    /// Time constant (s) of the low-pass that turns "measured yaw rate − commanded yaw rate" into
    /// the estimated unexplained yaw. It must be long against IMU noise and short against the
    /// leg; it assumes the drivetrain follows a yaw command much faster than this (a heavy
    /// robot's lag reads as unexplained yaw — HA-179 records the host-test-plant margin).
    /// > 0. PROVISIONAL (A4: HA-179).
    double yawObserverTime = 0.1;
    /// The most yaw rate (rad/s) the correction may add, either way — bounding how large a side
    /// imbalance TANK1 will cancel, how hard it will fight a robot being turned by something
    /// else, and how far it integrates while the robot cannot follow (pinned, lagging). 0
    /// disables the correction (the bend alone). >= 0. PROVISIONAL (A4: HA-179).
    double yawCorrectionLimit = 1.5;

    /// Every field finite; crossGain, leadTime and yawCorrectionLimit >= 0; maxSteer in
    /// [0, pi/2); fadeDistance and yawObserverTime > 0. MotionConfig::validate() refuses a
    /// config whose tankSteer fails this.
    [[nodiscard]] bool valid() const noexcept {
        auto fin = [](double v) { return std::isfinite(v); };
        return fin(crossGain) && fin(maxSteer) && fin(fadeDistance) && fin(leadTime)
               && fin(yawObserverTime) && fin(yawCorrectionLimit) && crossGain >= 0.0
               && maxSteer >= 0.0 && maxSteer < 1.5707963267948966 && fadeDistance > 0.0
               && leadTime >= 0.0 && yawObserverTime > 0.0 && yawCorrectionLimit >= 0.0;
    }
};

/// How the curve verbs (Chassis::curveTo / reverseCurveTo — chunk CURVE1, motion/curve_follow.hpp)
/// follow their path: pure pursuit on a smoothed path, a speed plan that slows BEFORE each bend,
/// guards that stop a curve the robot has lost, and the hand-off to an ordinary MoveToPose for the
/// last inches. Read by CurveFollow (and the pure path builder, motion/curve_path.hpp) only — no
/// other motion reads a field of it, so every other verb is unchanged bit for bit. Every value is
/// PROVISIONAL (A4: HA-246; the three guard values HA-248) — chosen for ROBUSTNESS on the host test
/// plant (a weaker side to 15 %, motor lag to 0.18 s), never measured on a robot. Station C tunes
/// lookAhead and bendAccel on the robot (the team's worksheet).
///
/// The hand-off's heading window is deliberately NOT a field here: it is
/// MotionConfig::tankSteer.maxSteer, read through tankLegEngages() — the function the end's
/// MoveToPose decides its own steering with — so the two can never disagree. With maxSteer = 0
/// (TANK1 off) there is no window: the end never steers and never warns.
struct CurveConfig {
    /// How far ahead along the path (in) the robot aims: each tick it steers along the arc through
    /// the path point this far past its closest point. Shorter tracks tighter bends and wobbles more
    /// under motor lag; longer is steadier and cuts tight bends. A bend tighter than lookAhead / 2 is
    /// driven but warned about. PROVISIONAL (A4: HA-246).
    double lookAhead = 8.0;
    /// The sideways "grip" (in/s^2) a bend may use: the speed limit in a bend of curvature k is
    /// sqrt(bendAccel / k) (60 in/s^2 is about 0.16 g, gentle). PROVISIONAL (A4: HA-246; the
    /// tyres' grip itself: HA-247).
    double bendAccel = 60.0;
    /// How fast the commanded forward speed may RISE (in/s^2), per tick. PROVISIONAL (A4: HA-246).
    double accel = 200.0;
    /// How fast the speed plan slows (in/s^2) — into each bend and to the end of the path (the
    /// plan's backward pass). PROVISIONAL (A4: HA-246).
    double decel = 120.0;
    /// How far ahead along the path (in) the closest-point search looks each tick, from where it
    /// was the tick before. FORWARD ONLY, and bounded: a path that comes back near itself must never
    /// make the robot jump to another leg of it. PROVISIONAL (A4: HA-246).
    double searchWindow = 12.0;
    /// The end zone (in of path left): inside it the curve may hand the last inches to an ordinary
    /// MoveToPose, and the no-progress guard is off (the end is slow on purpose). PROVISIONAL
    /// (A4: HA-246).
    double handoffDistance = 12.0;
    /// The clean hand-off's heading window (rad, 5 degrees): within this of the end heading.
    /// PROVISIONAL (A4: HA-246).
    double handoffHeading = 0.08726646259971647;
    /// The clean hand-off's line window (in): within this of the end's line. PROVISIONAL
    /// (A4: HA-246).
    double handoffCross = 0.3;
    /// The last-resort hand-off, as a fraction of handoffDistance (1/3: 4 in of path left), taken
    /// only once the heading is inside the end MoveToPose's own steering window. In (0, 1].
    /// PROVISIONAL (A4: HA-246).
    double handoffLast = 1.0 / 3.0;
    /// The approach speed toward the end, as a fraction of translation.kP x the path left (a
    /// gentler P than the end MoveToPose's own, so the hand-off is not a speed step). PROVISIONAL
    /// (A4: HA-246).
    double endSpeedGain = 0.6;
    /// Guard: the robot's direction of travel more than this (rad, 60 degrees) off the path's
    /// direction for lostTime stops the curve ("curve lost"). PROVISIONAL (A4: HA-248).
    double lostHeading = 1.0471975511965976;
    /// Guard: how long (s) the heading must stay past lostHeading. PROVISIONAL (A4: HA-248).
    double lostTime = 0.25;
    /// Guard: less than 1 in of progress along the path in this long (s), outside the end zone,
    /// stops the curve. PROVISIONAL (A4: HA-248).
    double stuckTime = 0.75;
    /// The start rule, and the "doubles back" rule at every point (rad, 90 degrees): a curve
    /// whose first point is more than this off the robot's nose (its back, reversing) is refused,
    /// and so is any point whose direction is more than this from the chord arriving at it or
    /// leaving it. PROVISIONAL (A4: HA-246).
    double maxStartOff = 1.5707963267948966;

    /// Every field finite; the lengths (lookAhead, searchWindow, handoffDistance, handoffCross),
    /// the accelerations and the times > 0; endSpeedGain > 0; handoffHeading, lostHeading and
    /// maxStartOff in (0, pi); handoffLast in (0, 1]. MotionConfig::validate() refuses a config
    /// whose curve fails this. lookAhead < handoffDistance is deliberately NOT required: the
    /// look-ahead runs on past the path's end along the end's line (curve_path.hpp), so a long
    /// look-ahead with a short end zone is a legal (if loose) tuning, not a contradiction.
    [[nodiscard]] bool valid() const noexcept {
        auto pos = [](double v) { return std::isfinite(v) && v > 0.0; };
        auto angle = [](double v) {
            return std::isfinite(v) && v > 0.0 && v < 3.141592653589793;
        };
        return pos(lookAhead) && pos(bendAccel) && pos(accel) && pos(decel) && pos(searchWindow)
               && pos(handoffDistance) && angle(handoffHeading) && pos(handoffCross)
               && pos(handoffLast) && handoffLast <= 1.0 && pos(endSpeedGain)
               && angle(lostHeading) && pos(lostTime) && pos(stuckTime) && angle(maxStartOff);
    }
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

    /// The spin-vs-motion cross-check thresholds (A4: HA-52) and, since R3b Part 2, its
    /// per-wheel drive geometry (`stall.wheels`, hal::DriveGeometry — the same objects the
    /// drive-encoder odometry converts with; `stall.setAllWheels()` for a symmetric drive) and
    /// `stall.independentMotionSource` (false on a drivetrain whose odometry IS the drive
    /// encoders: the check then never reports a verdict — odo_stall_check.hpp's §2 ruling).
    OdoStallCheckConfig stall{};

    /// Tank steering (chunk TANK1): read by MoveToPose, only when the drivetrain's
    /// strafeAuthority() is exactly 0, and — since CURVE1 — by CurveFollow (the curve verbs) on
    /// every drivetrain: maxSteer as the end's steering window, yawObserverTime and
    /// yawCorrectionLimit for the curve's own yaw observer. `tankSteer.maxSteer = 0` switches it
    /// off for every leg built from this config, curves included (no window to wait for, the
    /// curve's yaw correction held at 0). Appended last so every designated initializer of this struct keeps
    /// compiling. PROVISIONAL (A4: HA-179).
    TankSteerConfig tankSteer{};

    /// The curve verbs' knobs (chunk CURVE1): read by CurveFollow only (curveTo /
    /// reverseCurveTo), on any drivetrain. Appended last, after tankSteer, so every designated
    /// initializer of this struct keeps compiling. PROVISIONAL (A4: HA-246, HA-248).
    CurveConfig curve{};

    /// Re-check the invariants the motions rely on and RAISE on the first violation:
    /// feedforward and PID gains finite, integral limits non-negative, all FIVE speed /
    /// timeout / geometry scalars strictly positive (maxLinearSpeed, maxAngularSpeed,
    /// maxWheelSpeed, defaultTimeout, rotationRadius — 0 is rejected, never read as
    /// "unset"), tankSteer.valid() and curve.valid(). Every C1 motion calls this from its own constructor,
    /// so it is a backstop rather than a step you can forget — call it yourself only when
    /// validating a config you have not yet handed to a motion.
    /// It deliberately does NOT descend into the SettleConfig or OdoStallCheckConfig
    /// members: those are checked by SettledUtil and OdoStallCheck when the motion builds
    /// them, which is the only place their own invariants are known. It DOES descend into
    /// tankSteer (since TANK1): no other object owns that struct, and MoveToPose reads it on
    /// every tank tick, so this is the one place a bad value can be refused before it steers.
    /// It descends into curve (since CURVE1) for the same reason: nothing else owns that struct.
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
        // FINITE and > 0, not merely > 0: infinity satisfies `> 0.0`, and an infinite
        // defaultTimeout builds a Watchdog that can never expire — which defeats the one
        // guarantee the watchdog exists to provide ("a motion can never hang", watchdog.hpp).
        // The caller-supplied timeout was already screened for finiteness at every motion's
        // constructor; `timeout = 0` selects THIS field, which was the unscreened way in.
        SHULIB_PRECONDITION(std::isfinite(maxLinearSpeed.value()) && maxLinearSpeed.value() > 0.0,
                            "MotionConfig: maxLinearSpeed must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(maxAngularSpeed.value())
                                && maxAngularSpeed.value() > 0.0,
                            "MotionConfig: maxAngularSpeed must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(maxWheelSpeed.value()) && maxWheelSpeed.value() > 0.0,
                            "MotionConfig: maxWheelSpeed must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(defaultTimeout) && defaultTimeout > 0.0,
                            "MotionConfig: defaultTimeout must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(rotationRadius.value()) && rotationRadius.value() > 0.0,
                            "MotionConfig: rotationRadius must be finite and > 0");
        SHULIB_PRECONDITION(tankSteer.valid(), "MotionConfig: tankSteer invalid");
        SHULIB_PRECONDITION(curve.valid(), "MotionConfig: curve invalid");
        // SettleConfig / OdoStallCheckConfig fields are validated by their owners
        // (SettledUtil / OdoStallCheck) at construction.
    }
};

/// STEP1-A — an ARRIVAL TOLERANCE applied to one of the robot's settle configs (`robot`, e.g.
/// MotionConfig::translationSettle), as chassis::MotionOptions' positionTolerance /
/// headingTolerance apply it. With `e` = robot.maxError, `r` = robot.maxErrorRate and `t` =
/// `tolerance` (in the config's own error units — inches, or radians):
///   * `t <= 0` (or NaN): `robot`, unchanged — 0 means "the robot's own numbers", bit for bit;
///   * otherwise maxError = t, and maxErrorRate = r·t/e when `e > 0 && t > e` (LOOSER: the speed
///     bound loosens in proportion — on a drive leg it is the speed bound, not the distance, that
///     decides when the leg ends, so a distance alone would change nothing), else r (TIGHTER, or a
///     robot whose own distance is 0: the rate is never lowered — a smaller rate bound can sit
///     inside the IMU's noise and never settle, this header's top note — and never divided by 0:
///     r·t/0 would be inf, which silently drops the speed bound, or NaN, which SettledUtil refuses).
/// settleTime is never changed. Pure and total; validating `t` is the caller's (MotionOptions::
/// validate refuses a non-finite or negative tolerance before this runs).
[[nodiscard]] inline control::SettleConfig loosenedSettle(const control::SettleConfig& robot,
                                                         double tolerance) noexcept {
    if (!(tolerance > 0.0)) {
        return robot;
    }
    control::SettleConfig s = robot;
    s.maxError = tolerance;
    s.maxErrorRate = (robot.maxError > 0.0 && tolerance > robot.maxError)
                         ? robot.maxErrorRate * tolerance / robot.maxError
                         : robot.maxErrorRate;
    return s;
}

/// Validate `config` (and a caller-supplied `timeout`) and hand the config straight back, so a
/// motion can write `cfg_{validatedConfig(config, timeout, "TurnTo")}` as the FIRST member in
/// its initializer list and have the check run before any component is built from these fields.
/// The counterpart to MotionDeps::validatedClock(), which exists for exactly the same reason on
/// the pointer half: "a null pointer trips the precondition rather than being dereferenced."
/// Without it the first component constructed from a bad config reports the failure in ITS
/// vocabulary, naming a class the caller never named.
[[nodiscard]] inline const MotionConfig& validatedConfig(const MotionConfig& config,
                                                         double timeout, const char* who) {
    config.validate();
    (void)who;  // reserved for a per-motion message once SHULIB_PRECONDITION takes one
    SHULIB_PRECONDITION(std::isfinite(timeout) && timeout >= 0.0,
                        "motion: timeout must be finite and >= 0");
    return config;
}

}  // namespace shulib::motion
