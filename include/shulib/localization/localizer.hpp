#pragma once
//
// Localizer — the fused field-frame estimate (master plan §5/§6/§8; WS5). It is a thin, deterministic
// orchestrator over three already-tested pieces: PilonsOdometry (high-rate prediction), the IMU
// (heading authority), and a list of correctors (absolute fixes) behind an IFusionPolicy. It owns
// the four jobs the pieces below cannot, in a fixed five-step update():
//   1. dt — source it from the injected IClock and turn the per-tick position change into a Twist2d.
//   2. predict — advance PilonsOdometry; its pose is the dead-reckon prediction.
//   3. fuse — ask each corrector for an absolute proposal and fold the valid ones in as an
//      innovation-bounded, per-tick-clamped GATED NUDGE (position only), via the IFusionPolicy.
//   4. heading — re-stamp the fused heading from the IMU as the LAST write, so heading is provably
//      IMU-owned through fusion: no corrector or policy can ever rotate the robot (decision #4).
//   5. publish — recompute the quality scalar + categorical flags from observable inputs.
//
// Swapping the complementary filter for an EKF later is a one-argument change (IFusionPolicy); the
// IPoseSource read seam and the ICorrector write seam never move. At M2 the corrector list is empty
// or holds only NullCorrector, so the default-tested path is dead-reckoning (odom + IMU).

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <span>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/localization/i_fusion_policy.hpp"
#include "shulib/localization/i_pose_source.hpp"
#include "shulib/localization/pilons_odometry.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/math/twist2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

struct LocalizerConfig {
    /// Above this tick dt (s), the linear-velocity finite-difference is not trusted (first tick after
    /// construction/teleport, or a loop stall) → zero linear velocity for that tick + a flagged tick.
    double maxDt = 0.1;
    /// Below this tick dt (s), the finite-difference is likewise not trusted (a near-zero interval
    /// would otherwise blow up into an unphysical velocity spike).
    double minDt = 1e-4;
    /// distanceSinceCorrection at which the quality scalar decays to qFloor (drift erodes trust as we
    /// dead-reckon farther — process noise scales with travel). Default ~ one foot.
    units::Length driftHorizon{12.0};
    /// Quality floor while dead-reckoning far from a fix, in [0,1).
    double qFloor = 0.2;
};

class Localizer final : public IPoseSource {
public:
    /// Categorical health for motion/skills gating (distinct from the [0,1] scalar).
    enum class Quality { Uninitialized, DeadReckon, Corrected, Degraded };

    /// At most this many correctors (GPS + AI-Vision tag + Pi tag + LIDAR today) — the valid-proposal
    /// buffer is fixed-capacity so the hot path never heap-allocates.
    static constexpr std::size_t kMaxCorrectors = 4;

    /// `correctors` is a NON-OWNING view: the backing array (and the correctors it points to) must
    /// outlive the Localizer. Empty at M2 (dead-reckon). All references are validated non-null.
    Localizer(hal::IClock& clock, hal::IImu& imu, PilonsOdometry& odom, IFusionPolicy& fusion,
              std::span<ICorrector* const> correctors = {}, const LocalizerConfig& config = {})
        : clock_{clock}, imu_{imu}, odom_{odom}, fusion_{fusion}, correctors_{correctors},
          config_{config}, pose_{odom.pose()} {
        SHULIB_PRECONDITION(config.maxDt > 0.0, "Localizer: maxDt must be > 0");
        SHULIB_PRECONDITION(config.driftHorizon.value() > 0.0, "Localizer: driftHorizon must be > 0");
        SHULIB_PRECONDITION(config.qFloor >= 0.0 && config.qFloor < 1.0,
                            "Localizer: qFloor must be in [0, 1)");
        SHULIB_PRECONDITION(correctors.size() <= kMaxCorrectors, "Localizer: too many correctors");
        for (ICorrector* c : correctors_) {
            SHULIB_PRECONDITION(c != nullptr, "Localizer: a corrector is null");
        }
        fusedX_ = pose_.x().value();
        fusedY_ = pose_.y().value();
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odom_.pose().x().value();
        lastOdomY_ = odom_.pose().y().value();
    }

    /// One fused tick (the five steps above).
    void update() {
        // STEP 1 — dt from the injected clock.
        const double now = clock_.now().value();
        const double dt = hasLast_ ? (now - lastNow_) : 0.0;
        const bool dtHealthy = hasLast_ && dt >= config_.minDt && dt <= config_.maxDt;

        // STEP 2 — predict: advance the PERSISTENT fused position by the per-tick odom field-frame
        // DELTA (NOT a reset to absolute odom). This is what lets a correction accumulate and persist
        // after a corrector goes quiet — the never-snap nudge can now actually converge & bound drift.
        odom_.update();
        const math::Pose2d odomNow = odom_.pose();
        const double odx = odomNow.x().value() - lastOdomX_;
        const double ody = odomNow.y().value() - lastOdomY_;
        const math::Angle heading = imu_.heading();
        const math::Pose2d predicted{units::Length{fusedX_ + odx}, units::Length{fusedY_ + ody}, heading};

        // STEP 3 — gather VALID proposals (screened, incl. FINITE confidence so an Inf can't sail through).
        std::array<CorrectionProposal, kMaxCorrectors> buf{};
        std::array<const char*, kMaxCorrectors> names{};
        std::size_t n = 0;
        for (ICorrector* c : correctors_) {
            const CorrectionProposal p = c->propose(predicted, units::Time{dt});
            if (p.valid && std::isfinite(p.confidence) && p.confidence > 0.0 &&
                p.positionStdDev.value() > 0.0 &&
                std::isfinite(p.fieldPose.x().value()) && std::isfinite(p.fieldPose.y().value()) &&
                n < buf.size()) {
                names[n] = c->name();
                buf[n++] = p;
            }
        }

        // STEP 4 — fuse (position only): gated nudge against the PREDICTED fused pose (so the
        // innovation shrinks as the fused state converges).
        const FusionResult fr =
            fusion_.fuse(predicted, std::span<const CorrectionProposal>{buf.data(), n}, units::Time{dt});

        // STEP 5 — heading re-stamp (IMU-owned, the LAST write) + publish.
        fusedX_ = fr.x.value();
        fusedY_ = fr.y.value();
        const math::Pose2d newPose{units::Length{fusedX_}, units::Length{fusedY_}, heading};

        // twist: linear from the fused-pose finite-difference (dt-guarded), omega from the IMU (finite).
        const double rawOmega = imu_.yawRate().value();
        const double omega = std::isfinite(rawOmega) ? rawOmega : 0.0;
        if (hasLast_ && dt > 0.0) {
            const double vx = dtHealthy ? (fusedX_ - lastFusedX_) / dt : 0.0;  // tiny/huge dt ⇒ 0
            const double vy = dtHealthy ? (fusedY_ - lastFusedY_) / dt : 0.0;
            twist_ = math::Twist2d{units::Velocity{vx}, units::Velocity{vy}, units::AngularVelocity{omega}};
        } else {
            // dt <= 0 (or first tick): keep last linear velocity, refresh omega from the sensor.
            twist_ = math::Twist2d{twist_.vx(), twist_.vy(), units::AngularVelocity{omega}};
        }

        // drift accumulator: an applied fix CLEARS it in proportion to the fix's confidence (a strong
        // fix zeroes drift uncertainty; a weak one barely dents it — so quality() can't spring to 1.0
        // on a microscopic-confidence fix). Otherwise it grows by the odom-driven travel.
        if (fr.applied) {
            const double retain = 1.0 - std::clamp(fr.appliedConfidence, 0.0, 1.0);
            distanceSinceCorrection_ = units::Length{distanceSinceCorrection_.value() * retain};
        } else {
            distanceSinceCorrection_ =
                units::Length{distanceSinceCorrection_.value() + std::hypot(odx, ody)};
        }

        deadReckoning_ = !fr.applied;
        lastCorrection_ = AppliedCorrection{units::Length{fusedX_ - predicted.x().value()},
                                            units::Length{fusedY_ - predicted.y().value()},
                                            fr.gated, fr.clamped, (fr.applied && n > 0) ? names[0] : "none"};
        refreshQuality(dtHealthy);

        pose_ = newPose;
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odomNow.x().value();
        lastOdomY_ = odomNow.y().value();
        lastNow_ = now;
        hasLast_ = true;
        everUpdated_ = true;
    }

    // --- IPoseSource ---
    [[nodiscard]] math::Pose2d pose() const noexcept override { return pose_; }
    [[nodiscard]] math::Twist2d twist() const noexcept override { return twist_; }
    [[nodiscard]] double quality() const noexcept override { return quality_; }
    [[nodiscard]] bool isDeadReckoning() const noexcept override { return deadReckoning_; }

    // --- extra observability (telemetry / motion gating) ---
    [[nodiscard]] Quality qualityClass() const noexcept { return qualityClass_; }
    [[nodiscard]] units::Length distanceSinceCorrection() const noexcept { return distanceSinceCorrection_; }
    [[nodiscard]] const AppliedCorrection& lastCorrection() const noexcept { return lastCorrection_; }

    /// Teleport the POSITION (x, y); heading stays IMU-owned. Forwards to PilonsOdometry::setPose so
    /// the predictor and the fused belief never diverge, and re-baselines twist + dt so the teleport
    /// injects no phantom velocity next tick.
    void setPose(const math::Pose2d& p) {
        odom_.setPose(p);
        fusedX_ = p.x().value();
        fusedY_ = p.y().value();
        pose_ = math::Pose2d{p.x(), p.y(), imu_.heading()};
        lastFusedX_ = fusedX_;
        lastFusedY_ = fusedY_;
        lastOdomX_ = odom_.pose().x().value();  // re-baseline the odom delta so the jump isn't counted
        lastOdomY_ = odom_.pose().y().value();
        twist_ = math::Twist2d{};
        distanceSinceCorrection_ = units::Length{0.0};
        hasLast_ = false;  // next tick re-establishes dt (no phantom velocity from the jump)
    }

private:
    void refreshQuality(bool dtHealthy) {
        const bool ready = imu_.isReady();
        const bool implausible = odom_.lastDeltaImplausible();
        if (!everUpdated_ || !ready) {
            qualityClass_ = Quality::Uninitialized;
        } else if (implausible || !dtHealthy) {
            qualityClass_ = Quality::Degraded;
        } else if (!deadReckoning_) {
            qualityClass_ = Quality::Corrected;
        } else if (distanceSinceCorrection_.value() > config_.driftHorizon.value()) {
            qualityClass_ = Quality::Degraded;
        } else {
            qualityClass_ = Quality::DeadReckon;
        }

        // Scalar, kept CONSISTENT with the class: 0 until ready+initialized (no false confidence at
        // boot), then driftTerm decayed by dt-health and odom implausibility so it can't read 1.0 on
        // a Degraded tick. (Graded confidence-weighted trust per fix is the M3 EKF's job; M2 resets
        // the drift term on any accepted in-gate fix.)
        if (!everUpdated_ || !ready) {
            quality_ = 0.0;
            return;
        }
        const double driftFrac = distanceSinceCorrection_.value() / config_.driftHorizon.value();
        const double driftTerm = std::clamp(1.0 - driftFrac, config_.qFloor, 1.0);
        const double dtTerm = dtHealthy ? 1.0 : 0.5;
        const double implausTerm = implausible ? 0.5 : 1.0;
        quality_ = driftTerm * dtTerm * implausTerm;
    }

    hal::IClock& clock_;
    hal::IImu& imu_;
    PilonsOdometry& odom_;
    IFusionPolicy& fusion_;
    std::span<ICorrector* const> correctors_;
    LocalizerConfig config_;

    math::Pose2d pose_{};
    math::Twist2d twist_{};
    units::Length distanceSinceCorrection_{0.0};
    AppliedCorrection lastCorrection_{};
    Quality qualityClass_ = Quality::Uninitialized;
    double quality_ = 0.0;
    bool deadReckoning_ = true;

    double fusedX_ = 0.0;     // PERSISTENT fused position — advanced by odom deltas + retained nudges
    double fusedY_ = 0.0;
    double lastOdomX_ = 0.0;  // odom pose at the previous tick (to take the per-tick odom delta)
    double lastOdomY_ = 0.0;
    double lastNow_ = 0.0;
    double lastFusedX_ = 0.0;  // fused position at the previous tick (for the twist finite-difference)
    double lastFusedY_ = 0.0;
    bool hasLast_ = false;
    bool everUpdated_ = false;
};

}  // namespace shulib::localization
