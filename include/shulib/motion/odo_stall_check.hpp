#pragma once
//
// OdoStallCheck — the spin-vs-motion cross-check (chunk C1; A3 handoff #2).
//
// ── Why this exists (the A3 finding, verbatim consequence) ──────────────────────────
// A frozen tracking encoder is INVISIBLE to the M2 estimator: zero travel is a
// perfectly plausible reading, so PilonsOdometry::lastDeltaImplausible() never
// fires and the fused estimate walks away from truth at exactly the truth's
// speed (measured during the hostile-fakes campaign, and asserted by test).
// fault.hpp assigns OdoStuck to "the C/E layers"; the estimator-side detector
// is E-phase work. Until then,
// THIS windowed cross-check — owned by every C1 motion's tick — is the only
// defence against a dead encoder: the drive encoders say the wheels are rolling,
// the fused estimate says the robot is not moving. Sustained disagreement ⇒ the
// odometry is stuck ⇒ HealthMonitor::Observations::odomStalled ⇒ ODO_STUCK.
//
// ── The verdict (per evaluation window) ─────────────────────────────────────────────
//     spinTravel     = mean_i |Δ driveShaft_i| · wheelRadius          (inches)
//     observedMotion = hypot(Δx, Δy) + rotationRadius · |Δheading|    (inches)
//     stalled        = spinTravel ≥ minSpinTravel
//                      AND observedMotion < motionRatio · spinTravel
//
// Design points, each load-bearing:
//   * The ROTATION TERM (rotationRadius·|Δheading|, shortest-path Δ so the ±180°
//     seam cannot inflate it) is what makes a pure TurnTo immune to false
//     positives: spinning in place the wheels travel ≈ R·Δθ each while the
//     position stands still — without the term every turn would fault. It also
//     means a frozen tracking encoder does NOT false-fault a pure turn: heading
//     is IMU-owned, so a turn genuinely progresses and reports its motion even
//     with dead position odometry. That is correct, not a miss — dead position
//     odometry cannot hurt a pure turn, and any translation attempt still trips.
//   * The RATIO (not an absolute floor) keeps the check speed-independent, with
//     margin: A3's slip model still propels ≈70% of spin (HA-40), far above the
//     25% default — slip degrades, a stuck estimate STOPS. A physically blocked
//     robot with spinning wheels also trips; that is the same fault family
//     (fault.hpp: "odometry implausible / WHEEL STUCK"), and on purpose.
//   * MEAN |Δshaft| over all drive wheels: a single dead DRIVE encoder leaves
//     (n-1)/n of the mean rather than zeroing it (still trips) — 1/2 on a
//     2-wheel tank, 3/4 on the X-drive named below, 2/3 on the C3 H-bot. This
//     bullet used to say "halves", which is the n=2 answer stated as if it were
//     general, in the header of a check whose flagship consumers have 3 and 4
//     wheels. The conclusion ("still trips") holds and is in fact STRONGER than
//     the wrong figure implied. An X-drive strafe (all four wheels spinning)
//     reads full spin travel.
//   * WINDOWED (default 0.3 s), not per-tick: per-tick deltas are quantization-
//     noise-dominated; a window integrates real travel. The verdict HOLDS until
//     the next window closes, so HealthMonitor's edge-per-episode logic sees one
//     sustained episode, not chatter.
//
// Thresholds are PROVISIONAL (A4: HA-52) — window, minSpinTravel, motionRatio
// and the radii are invented/stand-in numbers until R3 measures geometry and R4
// measures noise floors. The register carries the falsifiable claims.
//
// Owned per-motion, reset() at start(): the window baseline must not straddle a
// motion boundary (a teleport/setPose between motions would look like motion).

#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::motion {

/// The five knobs of the spin-vs-motion cross-check, taken BY VALUE at construction (editing the
/// struct afterwards does nothing to a live check) and every one of them validated by that
/// constructor. Every default is PROVISIONAL: the two radii are stand-in geometry and the three
/// thresholds are invented numbers — none has yet been measured against a real drivetrain or a
/// real noise floor, so treat a default as a placeholder that compiles, not as a tuning.
struct OdoStallCheckConfig {
    /// Evaluation window (seconds). PROVISIONAL (A4: HA-52).
    double window = 0.3;
    /// Mean wheel-implied travel that counts as "the wheels are spinning"
    /// (inches per window). PROVISIONAL (A4: HA-52).
    units::Length minSpinTravel{1.0};
    /// observedMotion / spinTravel below this ⇒ stalled. PROVISIONAL (A4: HA-52).
    double motionRatio = 0.25;
    /// Drive wheel RADIUS (inches) — converts shaft radians to surface travel.
    /// Stand-in geometry (3.25″ wheel, 1:1 gearing — A4: HA-14).
    units::Length wheelRadius{3.25 / 2.0};
    /// Converts |Δheading| to equivalent wheel travel (≈ center-to-wheel
    /// distance). Stand-in geometry (A4: HA-17/HA-52).
    units::Length rotationRadius{7.0};
};

/// The windowed spin-vs-motion cross-check: the drive encoders say the wheels rolled, the fused
/// estimate says the robot did not move, and sustained disagreement means the odometry is stuck.
/// It is the only defence against a FROZEN tracking encoder, which the estimator itself cannot
/// see — zero travel is a perfectly plausible reading, so no plausibility guard fires while the
/// fused pose walks away from truth at exactly truth's speed. Owned per-motion and reset() at
/// start(), because a window straddling a motion boundary would read a setPose as motion. The
/// verdict HOLDS between window closes, so a consumer sees one sustained episode, not chatter.
class OdoStallCheck {
public:
    /// Fixed capacity of the per-wheel shaft baseline, mirroring kinematics::WheelSpeeds so the
    /// hot path never allocates. update() rejects a larger span outright rather than truncating.
    static constexpr int kMaxWheels = 8;  // mirrors kinematics::WheelSpeeds::kMaxWheels

    /// Copies `config` and validates every field: window finite and > 0, minSpinTravel > 0, both
    /// radii > 0, and motionRatio strictly inside (0, 1) — at 0 nothing could ever trip, at 1 any
    /// slip at all would read as a stall. A violation trips the precondition handler; nothing is
    /// clamped. The check starts with no baseline, so the first update() only baselines and no
    /// verdict can be true until a full `window` has elapsed.
    explicit OdoStallCheck(const OdoStallCheckConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.window) && cfg_.window > 0.0,
                            "OdoStallCheck: window must be finite and > 0");
        SHULIB_PRECONDITION(cfg_.minSpinTravel.value() > 0.0,
                            "OdoStallCheck: minSpinTravel must be > 0");
        SHULIB_PRECONDITION(cfg_.motionRatio > 0.0 && cfg_.motionRatio < 1.0,
                            "OdoStallCheck: motionRatio must be in (0, 1)");
        SHULIB_PRECONDITION(cfg_.wheelRadius.value() > 0.0,
                            "OdoStallCheck: wheelRadius must be > 0");
        SHULIB_PRECONDITION(cfg_.rotationRadius.value() > 0.0,
                            "OdoStallCheck: rotationRadius must be > 0");
    }

    /// Feed one tick's observables; returns the current (window-held) verdict.
    /// `motors` are the drive motors in kinematic order (size constant per run), NON-EMPTY
    /// and all non-null — the same discipline every other span-taking fan-out in the tree
    /// keeps (MotorMechanism, PneumaticMechanism, RobotContext, Localizer). Both checks were
    /// missing, and the empty case was the dangerous one: with no motors the mean shaft
    /// delta is 0, so spinTravel is 0, so `spinTravel >= minSpinTravel` is never true and
    /// the check reports "healthy" forever. A misconfiguration that silently disables a
    /// safety cross-check is exactly what this library's precondition discipline exists to
    /// turn into a loud failure. The in-tree path (ctx.driveMotors()) was already safe; a
    /// direct caller — which the generated reference invites — was not.
    [[nodiscard]] bool update(units::Time now, std::span<hal::IMotor* const> motors,
                              const math::Pose2d& fusedPose) {
        SHULIB_PRECONDITION(motors.size() <= static_cast<std::size_t>(kMaxWheels),
                            "OdoStallCheck: too many motors");
        SHULIB_PRECONDITION(!motors.empty(), "OdoStallCheck: motors is empty");
        for (const hal::IMotor* m : motors) {
            SHULIB_PRECONDITION(m != nullptr, "OdoStallCheck: a motor is null");
        }
        if (!hasBaseline_) {
            baseline(now, motors, fusedPose);
            return stalled_;
        }
        if ((now.value() - windowStart_) < cfg_.window) {
            return stalled_;  // window still open: hold the previous verdict
        }
        // Window closed: evaluate, then re-baseline.
        double sumAbsShaftDelta = 0.0;
        const auto n = motors.size();
        for (std::size_t i = 0; i < n; ++i) {
            sumAbsShaftDelta += std::abs(motors[i]->position().value() - shaftBase_[i]);
        }
        const double meanShaftDelta = (n > 0) ? sumAbsShaftDelta / static_cast<double>(n) : 0.0;
        const double spinTravel = meanShaftDelta * cfg_.wheelRadius.value();
        const double dx = fusedPose.x().value() - xBase_;
        const double dy = fusedPose.y().value() - yBase_;
        // Shortest-path heading delta: the ±180° seam must not fabricate motion.
        const double dHeading = std::abs(headingBase_.errorTo(fusedPose.heading()));
        const double observed = std::hypot(dx, dy) + cfg_.rotationRadius.value() * dHeading;

        stalled_ = (spinTravel >= cfg_.minSpinTravel.value())
                   && (observed < cfg_.motionRatio * spinTravel);
        baseline(now, motors, fusedPose);
        return stalled_;
    }

    /// The latest window verdict (held between window closes).
    [[nodiscard]] bool stalled() const noexcept { return stalled_; }

    /// Forget the window baseline AND the verdict (motion start / after setPose).
    void reset() noexcept {
        hasBaseline_ = false;
        stalled_ = false;
    }

private:
    void baseline(units::Time now, std::span<hal::IMotor* const> motors,
                  const math::Pose2d& fusedPose) {
        windowStart_ = now.value();
        for (std::size_t i = 0; i < motors.size(); ++i) {
            shaftBase_[i] = motors[i]->position().value();
        }
        xBase_ = fusedPose.x().value();
        yBase_ = fusedPose.y().value();
        headingBase_ = fusedPose.heading();
        hasBaseline_ = true;
    }

    OdoStallCheckConfig cfg_;
    std::array<double, static_cast<std::size_t>(kMaxWheels)> shaftBase_{};
    double windowStart_ = 0.0;
    double xBase_ = 0.0;
    double yBase_ = 0.0;
    math::Angle headingBase_{};
    bool hasBaseline_ = false;
    bool stalled_ = false;
};

}  // namespace shulib::motion
