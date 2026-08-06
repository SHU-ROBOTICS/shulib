#pragma once
//
// PilonsOdometry — tracking-wheel dead-reckoning (master plan §8; WS5). Each tick it folds the
// two perpendicular tracking wheels + the IMU heading into one `arcStep` and accumulates the
// field-frame `Pose2d`. This is the high-rate (~100 Hz) prediction backbone the fused
// `Localizer` corrects with absolute fixes (GPS/AprilTag) at M3.
//
// Two responsibilities live here (and NOT in arc_step, which stays pure geometry):
//
// 1. OFFSET CORRECTION (wheel travel → tracking-CENTER travel). A wheel offset from the center
//    reads extra/less travel during a turn. Removing the rotation-induced component (derived
//    from the planar rigid-body velocity v_point = (vf − ω·b, vl + ω·a); verified end-to-end by
//    /tmp/odom_oracle.py):
//        centerForward = forwardWheel.travel + Δθ · forwardWheel.offset   (offset = +LEFT coord)
//        centerLateral = lateralWheel.travel − Δθ · lateralWheel.offset   (offset = +FORWARD coord)
//    The opposite signs are not arbitrary: a pure in-place rotation (either direction) must yield
//    ZERO center travel (else the robot "drifts" while turning — a classic odom bug). The
//    pure-rotation tests (CW and CCW) pin both signs.
//
// 2. HEADING is IMU-OWNED. The pose heading is set EQUAL to the IMU heading every tick (absolute,
//    never integrated from the wheels — wheel-difference heading is the H-bot's cross-check only,
//    decision #3), and from construction onward (the seeded pose's heading is informational; the
//    IMU is the authority, so there is no construction→first-update window where they disagree).
//    Δθ for the arc comes from the two IMU samples via Angle::errorTo (shortest signed, wrap-correct).
//
// TRUST GATE. `lastDeltaImplausible()` is true when a tick is NOT trustworthy, for either reason:
//   * |Δθ| > maxTickRotation — a heading change too large for one tick. NOTE this is a magnitude
//     threshold on the ALREADY-WRAPPED Δθ; it catches an aliased/stalled sample only when the
//     wrapped magnitude lands in (maxTickRotation, π]. A real rotation > π that aliases to a SMALL
//     Δθ is invisible here — it is excluded by the ~100 Hz loop-rate assumption (arc_step's
//     PRECONDITION; A4 register HA-32 — the sustained loop rate on a loaded V5 is unmeasured),
//     NOT by this gate. So `false` means "Δθ is within the per-tick bound," not
//     "the rotation is certainly real."
//   * the integration came out non-finite — a breach of the HAL finiteness contract (§7). A
//     non-finite tick FREEZES the position at its last good value (it never writes NaN into the
//     persistent pose) rather than poisoning the run; an oversized-but-finite tick is flagged but
//     STILL integrated (the motion may be real). Recovery policy beyond this is the fusion layer's.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/localization/arc_step.hpp"
#include "shulib/localization/tracking_wheel.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

struct PilonsOdometryConfig {
    /// |Δθ| (radians) above which a tick's heading change is treated as implausible. Default π/2
    /// sits far above any real ~100 Hz tick (≪ 1 rad) yet below the π wrap cliff. Tighten it for a
    /// known loop rate / max yaw rate. (See the trust-gate note in the header for what it can detect.)
    double maxTickRotation = 0.5 * math::Angle::kPi;
};

class PilonsOdometry {
public:
    /// `forward` must be a TrackingWheel::forward(), `lateral` a TrackingWheel::lateral() — the
    /// roles are checked so a swapped pair throws at construction. `initial` seeds the position;
    /// its heading is informational (the IMU owns heading from the first reading — see header).
    PilonsOdometry(hal::IImu& imu, TrackingWheel forward, TrackingWheel lateral,
                   const math::Pose2d& initial = {}, const PilonsOdometryConfig& config = {})
        : imu_{imu},
          forward_{forward},
          lateral_{lateral},
          pose_{initial.x(), initial.y(), imu.heading()},  // heading is IMU-owned from t=0
          prevHeading_{imu.heading()},
          maxTickRotation_{config.maxTickRotation} {
        SHULIB_PRECONDITION(forward.role() == TrackingWheel::Role::Forward,
                            "PilonsOdometry: first wheel must be TrackingWheel::forward()");
        SHULIB_PRECONDITION(lateral.role() == TrackingWheel::Role::Lateral,
                            "PilonsOdometry: second wheel must be TrackingWheel::lateral()");
        SHULIB_PRECONDITION(config.maxTickRotation > 0.0,
                            "PilonsOdometry: maxTickRotation must be > 0");
        forward_.reset();  // baseline the wheels to NOW, so a pre-existing shaft total isn't
        lateral_.reset();  // counted as travel on the first update()
    }

    /// One integration tick: read the IMU + wheels, offset-correct, arcStep, accumulate.
    void update() {
        const math::Angle h0 = prevHeading_;
        const math::Angle h1 = imu_.heading();
        const double dTheta = h0.errorTo(h1);  // shortest signed (wrap-correct)

        // wheel travel → tracking-center travel (remove the rotation-induced component)
        const double centerFwd = forward_.travelDelta().value() + dTheta * forward_.offset().value();
        const double centerLat = lateral_.travelDelta().value() - dTheta * lateral_.offset().value();
        const FieldDelta d =
            arcStep({units::Length{centerFwd}, units::Length{centerLat}}, h0, h1);

        const bool finite = std::isfinite(d.dx.value()) && std::isfinite(d.dy.value());
        implausible_ = !finite || std::abs(dTheta) > maxTickRotation_;

        // Heading is IMU-owned and always advances. Position only accumulates a FINITE delta, so a
        // non-finite tick freezes position at its last good value instead of poisoning the pose.
        pose_ = finite ? math::Pose2d{pose_.x() + d.dx, pose_.y() + d.dy, h1}
                       : math::Pose2d{pose_.x(), pose_.y(), h1};
        prevHeading_ = h1;
    }

    [[nodiscard]] math::Pose2d pose() const noexcept { return pose_; }

    /// Teleport the POSITION (x, y); heading stays IMU-owned. Re-baselines the heading reference
    /// so the teleport itself injects no phantom rotation on the next tick. Wheel baselines are
    /// left intact (a teleport doesn't change what the wheels have rolled).
    void setPose(const math::Pose2d& p) {
        pose_ = math::Pose2d{p.x(), p.y(), imu_.heading()};
        prevHeading_ = imu_.heading();
    }

    /// True iff the last update() was untrustworthy (oversized Δθ OR non-finite integration).
    [[nodiscard]] bool lastDeltaImplausible() const noexcept { return implausible_; }

private:
    hal::IImu& imu_;
    TrackingWheel forward_;
    TrackingWheel lateral_;
    math::Pose2d pose_;
    math::Angle prevHeading_;
    double maxTickRotation_;
    bool implausible_ = false;
};

}  // namespace shulib::localization
