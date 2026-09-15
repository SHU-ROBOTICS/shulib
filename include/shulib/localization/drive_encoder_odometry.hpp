#pragma once
//
// DriveEncoderOdometry — dead reckoning from the DRIVE motors' own encoders and the IMU,
// for a tank drive with no tracking wheels (chunk R3b Part 2, 2026-09-14). The second
// IOdometry implementation, beside PilonsOdometry.
//
// ── What it integrates, and the decision it borrows ─────────────────────────────────
// Per tick: ΔL and ΔR are each side's cumulative IMotor::position() delta (output-shaft
// radians, the MEDIAN of a hal::MotorGroup on a real robot) times that side's inches per
// radian (hal::DriveGeometry — the ONE representation of the per-side ratio, shared with
// the stall check). The centre's forward travel is (ΔL + ΔR)/2 and its LATERAL travel is
// EXACTLY 0: kinematics/tank.hpp's forward() already states, in writing, that "this
// drivetrain cannot observe lateral motion, so a real skid sideways is reported as no
// motion at all". This class is the odometry counterpart of that decision, not a new one
// — it inherits the justification rather than inventing a second. Then the SAME arcStep
// as PilonsOdometry (the one constant-curvature integration step; the localizer's
// accuracy-critical line, exhaustively tested on its own).
//
// HEADING IS IMU-OWNED — the identical policy, in the identical words, as PilonsOdometry's
// decision #3: the pose heading is set EQUAL to the IMU heading every tick (absolute,
// never integrated from the wheels — wheel-difference heading is the cross-check only,
// never the authority), and from construction onward (the seeded pose's heading is
// informational; the IMU is the authority, so there is no construction→first-update
// window where they disagree). Δθ for the arc comes from the two IMU samples via
// Angle::errorTo (shortest signed, wrap-correct).
//
// ── The trust gate — carried over, not re-derived ───────────────────────────────────
// Both halves of PilonsOdometry's gate, with the same two knobs, the same names, the same
// defaults and the same reasoning (read PilonsOdometryConfig's field comments; they are
// not repeated here): |Δθ| above maxTickRotation, or either side's |Δtravel| above
// maxTickTravel, or a non-finite integration ⇒ lastDeltaImplausible(). An implausible-but-
// finite tick is REPORTED, NOT WITHHELD; only a non-finite tick FREEZES position (heading
// still advances). The travel half matters more here than for tracking wheels: a drive
// port that enumerates late, or whose adapter holds a last-good value and then wakes,
// produces exactly the one-tick phantom the gate exists to flag.
//
// ── The heading cross-check — NEW, and deliberately an observable, not a fault ──────
// The encoders imply their own heading change, Δθ_enc = (ΔR − ΔL) / trackWidth. Against
// the IMU's Δθ it is the wheel-difference heading the Pilons header calls "the cross-check
// only, never the authority": lastHeadingDisagreement() = Δθ_enc − Δθ_imu (radians,
// signed) is what a SLIPPING side looks like, and it is the measurement R3d's track-width
// calibration is built from. It is NOT promoted to a fault here: a straight-line stall
// (both sides slipping equally) does not show in it, so it cannot stand in for the
// stall check. For THIS drivetrain the stall check has no independent motion source at
// all (the odometry IS the wheels — motion/odo_stall_check.hpp's
// `independentMotionSource`), and a future stuck/slip detector for the chassis will be
// built on this observable; the composition root says so once at boot.
//
// Baselines both sides at construction (the TrackingWheel::reset() precedent,
// pilons_odometry.hpp:119), so a pre-existing shaft total is not counted as travel on the
// first update(). Holds every reference; the IMU and both motors must outlive this object.
// Owns no loop. PROS-free; host-tested against the A2 plant's ground truth.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/drive_geometry.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/localization/arc_step.hpp"
#include "shulib/localization/odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// The trust gate's two knobs — the SAME two as PilonsOdometryConfig, same names, same
/// defaults, same reasoning (cited, not restated: read that struct's field comments).
struct DriveEncoderOdometryConfig {
    /// |Δθ| (radians) above which a tick's heading change is implausible. Default π/2 —
    /// PilonsOdometryConfig::maxTickRotation, verbatim.
    units::AngleDim maxTickRotation{0.5 * math::Angle::kPi};
    /// Largest believable |Δtravel| of ONE side in one tick (inches) before the delta is
    /// implausible. Default 36 in — PilonsOdometryConfig::maxTickTravel, verbatim, incl. its
    /// dt-blindness caveat. PROVISIONAL (A4: HA-123).
    units::Length maxTickTravel{36.0};
};

/// The two sides' travel over the last tick (inches), for the panel and the stall-check
/// agreement test.
struct SideTravel {
    units::Length left{};   ///< left side's wheel travel this tick (+ = forward)
    units::Length right{};  ///< right side's wheel travel this tick (+ = forward)
};

/// Dead reckoning for a tank drive without tracking wheels: each side's drive-motor
/// encoder delta (radians × that side's hal::DriveGeometry) gives the side's travel; the
/// centre moves forward by their mean and sideways by exactly zero (tank.hpp's written
/// decision); the IMU owns heading; the same arcStep and the same trust gate as
/// PilonsOdometry. Exposes the encoder-vs-IMU heading cross-check as an observable — never
/// a fault. Holds the IMU and both motors by reference; owns no loop.
class DriveEncoderOdometry final : public IOdometry {
public:
    /// `left` / `right`: the side's IMotor (a hal::MotorGroup on a real robot) in the
    /// kinematics' wheel order; `leftGeometry` / `rightGeometry`: each side's geometry — one
    /// object per side, the SAME objects the stall check is configured with; `trackWidth`:
    /// contact line to contact line, > 0 (TankKinematics's own value); `initial` seeds the
    /// position (heading informational — IMU-owned from the first reading).
    DriveEncoderOdometry(hal::IImu& imu, hal::IMotor& left, hal::IMotor& right,
                         hal::DriveGeometry leftGeometry, hal::DriveGeometry rightGeometry,
                         units::Length trackWidth, const math::Pose2d& initial = {},
                         const DriveEncoderOdometryConfig& config = {})
        : imu_{imu},
          left_{left},
          right_{right},
          leftIpr_{leftGeometry.inchesPerRadian().value()},
          rightIpr_{rightGeometry.inchesPerRadian().value()},
          trackWidth_{trackWidth.value()},
          pose_{initial.x(), initial.y(), imu.heading()},  // heading is IMU-owned from t=0
          prevHeading_{imu.heading()},
          maxTickRotation_{config.maxTickRotation.value()},
          maxTickTravel_{config.maxTickTravel.value()} {
        SHULIB_PRECONDITION(leftGeometry.valid(),
                            "DriveEncoderOdometry: left geometry must be finite and > 0 "
                            "(UNSET geometry is refused, never scaled by zero)");
        SHULIB_PRECONDITION(rightGeometry.valid(),
                            "DriveEncoderOdometry: right geometry must be finite and > 0 "
                            "(UNSET geometry is refused, never scaled by zero)");
        SHULIB_PRECONDITION(std::isfinite(trackWidth_) && trackWidth_ > 0.0,
                            "DriveEncoderOdometry: trackWidth must be finite and > 0");
        SHULIB_PRECONDITION(config.maxTickRotation.value() > 0.0,
                            "DriveEncoderOdometry: maxTickRotation must be > 0");
        SHULIB_PRECONDITION(std::isfinite(config.maxTickTravel.value())
                                && config.maxTickTravel.value() > 0.0,
                            "DriveEncoderOdometry: maxTickTravel must be finite and > 0");
        baseLeft_ = left_.position().value();    // baseline NOW, so a pre-existing shaft
        baseRight_ = right_.position().value();  // total isn't counted as travel on tick one
    }

    /// One integration tick: read the IMU and both sides, convert, arcStep, accumulate,
    /// evaluate the trust gate and the heading cross-check.
    void update() override {
        const math::Angle h0 = prevHeading_;
        const math::Angle h1 = imu_.heading();
        const double dTheta = h0.errorTo(h1);  // shortest signed (wrap-correct)

        // Each side's travel this tick: cumulative shaft delta × inches per radian. The
        // baselines re-arm every tick (a CONSUMING read, like TrackingWheel::travelDelta) —
        // but ONLY on a finite read: a non-finite reading (an F4 contract breach the fakes
        // model as hostility) must not poison the baseline, or every later tick would read
        // NaN too. Kept finite, the NEXT finite read integrates the travel across the bad
        // tick instead of losing it — one step better than the tracking-wheel path, which
        // freezes for good after a NaN. The bad tick itself still freezes position (below).
        const double pL = left_.position().value();
        const double pR = right_.position().value();
        const double dL = (pL - baseLeft_) * leftIpr_;
        const double dR = (pR - baseRight_) * rightIpr_;
        if (std::isfinite(pL)) baseLeft_ = pL;
        if (std::isfinite(pR)) baseRight_ = pR;
        lastTravel_ = SideTravel{units::Length{dL}, units::Length{dR}};

        // Centre travel: forward is the mean of the sides; LATERAL IS EXACTLY 0 — the
        // odometry counterpart of TankKinematics::forward()'s written decision (header).
        const double centerFwd = 0.5 * (dL + dR);
        const FieldDelta d = arcStep({units::Length{centerFwd}, units::Length{0.0}}, h0, h1);

        // The cross-check: what the wheels say the heading did, against what the IMU said.
        // Signed, radians; an observable for the panel and for R3d — never the authority.
        headingDisagreement_ = (dR - dL) / trackWidth_ - dTheta;

        // Both halves of the trust gate, Pilons semantics: report, never withhold; freeze
        // position only on a non-finite tick.
        const bool finite = std::isfinite(d.dx.value()) && std::isfinite(d.dy.value());
        const bool travelSane = std::abs(dL) <= maxTickTravel_ && std::abs(dR) <= maxTickTravel_;
        implausible_ = !finite || std::abs(dTheta) > maxTickRotation_ || !travelSane;

        pose_ = finite ? math::Pose2d{pose_.x() + d.dx, pose_.y() + d.dy, h1}
                       : math::Pose2d{pose_.x(), pose_.y(), h1};
        prevHeading_ = h1;
    }

    /// The accumulated field-frame estimate (heading the IMU's). A pure read.
    [[nodiscard]] math::Pose2d pose() const noexcept override { return pose_; }

    /// Teleport the POSITION (x, y); heading stays IMU-owned. Re-baselines the heading
    /// reference so the teleport injects no phantom rotation; the encoder baselines are
    /// left intact (a teleport doesn't change what the wheels have rolled).
    void setPose(const math::Pose2d& p) override {
        pose_ = math::Pose2d{p.x(), p.y(), imu_.heading()};
        prevHeading_ = imu_.heading();
    }

    /// True iff the last update() was untrustworthy (oversized Δθ, oversized side travel,
    /// or a non-finite integration).
    [[nodiscard]] bool lastDeltaImplausible() const noexcept override { return implausible_; }

    /// The heading cross-check of the last tick: encoder-implied Δθ − IMU Δθ (radians,
    /// signed). ≈ 0 under clean rolling; a slipping or stalled side shows as a bias with the
    /// sign of the side that under-travelled the IMU's turn. An observable, not a fault.
    [[nodiscard]] double lastHeadingDisagreement() const noexcept { return headingDisagreement_; }

    /// Each side's wheel travel over the last tick (inches) — the panel's two numbers and
    /// the value the stall check must agree with for the same encoder delta.
    [[nodiscard]] SideTravel lastSideTravel() const noexcept { return lastTravel_; }

    /// Inches per radian in use for the left / right side (the geometry, resolved).
    [[nodiscard]] units::Length leftInchesPerRadian() const noexcept { return units::Length{leftIpr_}; }
    /// Inches per radian in use for the right side.
    [[nodiscard]] units::Length rightInchesPerRadian() const noexcept { return units::Length{rightIpr_}; }
    /// The track width in use (inches).
    [[nodiscard]] units::Length trackWidth() const noexcept { return units::Length{trackWidth_}; }

private:
    hal::IImu& imu_;
    hal::IMotor& left_;
    hal::IMotor& right_;
    double leftIpr_;
    double rightIpr_;
    double trackWidth_;
    math::Pose2d pose_;
    math::Angle prevHeading_;
    double maxTickRotation_;
    double maxTickTravel_;
    double baseLeft_ = 0.0;
    double baseRight_ = 0.0;
    SideTravel lastTravel_{};
    double headingDisagreement_ = 0.0;
    bool implausible_ = false;
};

}  // namespace shulib::localization
