#pragma once
//
// GpsCorrector — the FIRST REAL corrector (master plan §6/§8; WS5, chunk E2). Everything
// before this chunk dead-reckoned: odometry and the IMU counted the robot's own motion and
// nothing in the library could ever tell the estimate it was wrong. This is the code that can.
//
// It implements ICorrector behind the exact signature the seam has had since M2 — PULL, not
// push: the Localizer calls propose(predicted, dt) each tick with the odometry-predicted pose,
// and this class answers with an absolute field position and how much to trust it, or with
// nothing at all.
//
// ── WHAT IT DOES, AND IN WHAT ORDER ────────────────────────────────────────────────────────
//   1. record the predicted position in a short history ring (needed for latency, below);
//   2. no fix?              → decline, RejectedNoFix        (the Driving-Skills path)
//   3. non-finite read?     → decline, RejectedNoFix        (F4 backstop; never trust a NaN)
//   4. sample unchanged?    → decline, RejectedStaleFix     (the double-count guard)
//   5. claimed error huge?  → decline, RejectedSensorQuality
//   6. spinning fast?       → decline, RejectedHighYawRate
//   7. compensate for sensor latency using the odometry history;
//   8. normalized-innovation gate → decline, RejectedNormalizedInnovation
//   9. otherwise propose, with an adaptive σ and a confidence derived from it.
// Every decline carries its reason out on CorrectionProposal::selfAudit, which is how an
// off-strip run becomes visible in the blackbox instead of looking like an idle estimator.
//
// ── WHAT IT DELIBERATELY DOES NOT DO ───────────────────────────────────────────────────────
// * NO FRAME CONVERSION and NO LEVER-ARM REMOVAL. Both belong to the HAL edge and are already
//   done by the time a pose reaches here: hal/gps.hpp documents IGps::pose() as "the
//   robot-CENTER pose (lever-arm corrected)", and hal/gps_conversion.hpp calls itself "the ONE
//   place the VEX GPS frame becomes shulib's canonical frame" and says ONE owner for the lever
//   arm, naming double-subtraction as the failure. Doing either here would be invisible in host
//   tests (FakeGps stores a centre pose, so a second removal would just be wrong by the arm)
//   and silently wrong on the robot, where the R1 adapter is contractually obliged to have done
//   it already. E2's obligation to those two conversions is PROOF — independent oracles at
//   several headings in test/gps_conversion_test.cpp — not a second implementation.
// * NO HEADING. The V5 GPS reports one; this class never lets it out. CorrectionProposal::
//   fieldPose carries the PREDICTED (IMU) heading, so even a future policy that read
//   fieldPose.heading() would read the IMU's answer, and providesHeading stays false. Heading
//   is IMU-owned by decision #4 and the Localizer re-stamps it as the last write; a GPS-heading
//   path is E3/E4's deliberate additive decision (a headingNudge on FusionResult), not a side
//   effect of E2.
// * NO SNAP, ever. This class only ever PROPOSES an absolute pose; how far the estimate moves
//   toward it is the fusion policy's bounded, per-tick-clamped nudge (§13 #4). There is no code
//   path here that writes a pose.
//
// ── THE GATE IS A NORMALIZED INNOVATION, NOT A MAHALANOBIS DISTANCE (chunk tension T1) ──────
// The gate computes
//        ν = |z − predicted| / σ_eff,      σ_eff = hypot(σ_meas, σ_dr)
// and declines when ν > gateSigma. A Mahalanobis distance normalises by the innovation
// covariance S = H·P·Hᵀ + R with P ESTIMATED BY A FILTER; the complementary tier estimates no P
// and σ_dr below is a hand-written heuristic over an invented growth rate. So this class never
// raises GateReason::RejectedMahalanobis and never writes GateAudit::mahalanobis — E4's EKF
// earns that number, and one field holding both an earned and an asserted quantity would make
// the difference between them invisible. GateReason::RejectedNormalizedInnovation says what
// actually happened.
//
// WHY σ_dr EXISTS (and why a fixed inch threshold was rejected). σ_meas alone gives a gate a
// couple of inches wide. After twenty feet of dead-reckoning the estimate can be further off
// than that, so a truthful fix would look outrageous and be rejected — and every subsequent
// one too, because nothing else can fix the estimate. That is gate lockout: the GPS goes
// silently dead exactly when it is worth the most. Widening the gate with the distance travelled
// since this source last had a fix is what prevents it, and it is tested directly (a fix that a
// zero-growth build rejects is accepted after a long dead-reckon).
//
// ── ADAPTIVE R, AND WHY THE DEVICE'S CLAIM IS NOT TAKEN AT FACE VALUE ───────────────────────
//   σ_meas     = max(rmsTrustFactor · rmsError(), minPositionStdDev)
//   confidence = σ_dr² / (σ_dr² + σ_meas²)
// The confidence is the scalar Kalman gain of a one-dimensional update, which is the right
// shape for the complementary tier's pull weight: trust the fix more when the estimate is
// uncertain, less when the sensor says it is uncertain. rmsTrustFactor exists because A4
// register HA-29 records that the device's self-estimate and its real error are different
// numbers; minPositionStdDev exists so a device claiming ~0 error cannot produce an arbitrarily
// tight gate. Both are provisional (see the register entries on each field) — E2 proves the
// LOGIC; R4 measures the constants.
//
// ── THE DOUBLE-COUNT GUARD ─────────────────────────────────────────────────────────────────
// The GPS camera produces a new fix every ~50 ms (HA-28) while the control loop runs at ~100 Hz,
// so a corrector that folds every read treats one measurement as five independent ones — the
// hazard sim/hostile/gps_hostility.hpp names in its header. This class folds a sample once: a
// read whose position and reported error are unchanged is declined as RejectedStaleFix.
// HONEST LIMITATION: with a NOISELESS gps (the identity degradation model, or a FakeGps whose
// pose is set once) every read after the first is byte-identical, so correction happens once and
// then stops. That is the contract behaving as written — an unchanged number carries no new
// information — but it surprises anyone who expects continuous correction from a static fake.
// A real device jitters (HA-26) and a hostile-model run produces a fresh sample every cadence.
//
// ── LATENCY ────────────────────────────────────────────────────────────────────────────────
// A fix describes where the robot WAS, roughly 50 ms ago (HA-30). Applied as if it described
// where the robot IS, it drags the estimate backwards along the direction of travel — at 40 in/s
// that is a systematic 2-inch lag, larger than the sensor's own noise. So the fix is carried
// forward by the odometry travelled since it was captured: z = gpsPose + (P(now) − P(capture)),
// read out of the history ring. Odometry is excellent over 50 ms even when it is poor over 50
// seconds, which is exactly what this needs.
//
// Pure w.r.t. its injected handles (clock, gps, imu) and PROS-free: it is built against the HAL
// seam, so the same code runs against FakeGps on the host and the R1 pros::Gps adapter on the
// robot. propose() never throws and never allocates.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "shulib/core/check.hpp"
#include "shulib/diag/debug_record.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/hal/gps.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/localization/correction.hpp"
#include "shulib/localization/i_corrector.hpp"
#include "shulib/math/pose2d.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::localization {

/// Tuning for GpsCorrector. Every default is PROVISIONAL — there is no robot yet, and each one
/// carries its A4 Hardware Assumptions Register entry. E2 proves the corrector's LOGIC against
/// A3's hostile GPS; the magnitudes in that model are themselves guesses, and R4 measures both.
struct GpsCorrectorConfig {
    /// End-to-end delay between the moment a fix describes and the moment it can be read
    /// (camera exposure + solve + transport). PROVISIONAL (A4: HA-30) — invented, ≈50 ms.
    units::Time latency{0.05};
    /// Multiplier applied to the device's self-reported rms to get the 1σ actually used.
    /// > 1 because A4 register HA-29 records that a sensor's self-estimate and its real error
    /// are different numbers and the gap must be survived. PROVISIONAL (A4: HA-61).
    double rmsTrustFactor = 2.0;
    /// Floor on the measurement 1σ. Without it, a device reporting ~0 error produces an
    /// arbitrarily tight gate that rejects everything including itself. PROVISIONAL (A4: HA-62).
    units::Length minPositionStdDev{0.5};
    /// Decline a fix whose REPORTED rms exceeds this — the sensor saying "I can see, badly".
    /// Without it a fix claiming 99" of error is still folded: the gate widens to accept it and
    /// the confidence shrinks to almost nothing, so the estimate barely moves, but the Localizer
    /// still reports quality class "Corrected" and the run looks anchored when it is not.
    /// PROVISIONAL (A4: HA-63).
    units::Length maxReportedRms{6.0};
    /// Decline any fix taken while the yaw rate exceeds this. During a fast spin the lever-arm
    /// removal done at the HAL edge is at its most wrong (the sensor is swinging through an arc
    /// at ω·r, and its heading and position are sampled at slightly different instants), and the
    /// latency compensation cannot recover a rotation it did not see. PROVISIONAL (A4: HA-64).
    units::AngularVelocity maxYawRate{3.0};
    /// Gate width in units of σ_eff. PROVISIONAL (A4: HA-65).
    double gateSigma = 4.0;
    /// The estimate's position 1σ immediately after this source's fix is folded — the floor of
    /// σ_dr, so confidence is never 0 (a 0-confidence proposal is screened out by the Localizer
    /// and would read as "no proposal at all"). PROVISIONAL (A4: HA-66).
    units::Length postFixStdDev{1.0};
    /// Growth of the dead-reckoning 1σ per inch travelled since this source's last fix. This is
    /// the anti-lockout term (header note). PROVISIONAL (A4: HA-67).
    double driftStdDevPerInch = 0.02;
};

class GpsCorrector final : public ICorrector {
public:
    /// Ticks of predicted-position history kept for latency compensation. 64 ticks is ~0.64 s at
    /// 100 Hz against a ~50 ms latency — deep enough that a stalled loop or a slower control
    /// rate still finds the capture instant inside the ring. Fixed capacity: the hot path never
    /// allocates.
    static constexpr std::size_t kHistory = 64;

    /// `clock`, `gps` and `imu` are non-owning references that must outlive this corrector.
    /// `name` is the stable telemetry id reported by name() and stamped into
    /// AppliedCorrection::source, so per-source dead-reckon accounting can say WHICH source
    /// went quiet.
    GpsCorrector(hal::IClock& clock, hal::IGps& gps, hal::IImu& imu,
                 const GpsCorrectorConfig& config = {}, const char* name = "gps")
        : clock_{clock}, gps_{gps}, imu_{imu}, config_{config}, name_{name} {
        SHULIB_PRECONDITION(config.latency.value() >= 0.0, "GpsCorrector: latency must be >= 0");
        SHULIB_PRECONDITION(config.rmsTrustFactor > 0.0,
                            "GpsCorrector: rmsTrustFactor must be > 0");
        SHULIB_PRECONDITION(config.minPositionStdDev.value() > 0.0,
                            "GpsCorrector: minPositionStdDev must be > 0");
        SHULIB_PRECONDITION(config.maxReportedRms.value() > 0.0,
                            "GpsCorrector: maxReportedRms must be > 0");
        SHULIB_PRECONDITION(config.maxYawRate.value() > 0.0,
                            "GpsCorrector: maxYawRate must be > 0");
        SHULIB_PRECONDITION(config.gateSigma > 0.0, "GpsCorrector: gateSigma must be > 0");
        SHULIB_PRECONDITION(config.postFixStdDev.value() > 0.0,
                            "GpsCorrector: postFixStdDev must be > 0");
        SHULIB_PRECONDITION(config.driftStdDevPerInch >= 0.0,
                            "GpsCorrector: driftStdDevPerInch must be >= 0");
        SHULIB_PRECONDITION(name != nullptr, "GpsCorrector: name must not be null");
    }

    /// One tick of the sequence in the header note. Never throws, never allocates; `dt` is
    /// unused because this corrector timestamps from the injected clock, which is authoritative
    /// and monotonic where a per-tick dt is a difference the Localizer already took.
    [[nodiscard]] CorrectionProposal propose(const math::Pose2d& predicted,
                                             units::Time /*dt*/) override {
        const double now = clock_.now().value();
        const double px = predicted.x().value();
        const double py = predicted.y().value();
        if (!std::isfinite(now) || !std::isfinite(px) || !std::isfinite(py)) {
            return decline(diag::GateReason::RejectedNoFix);  // nothing sane to reason from
        }

        // (1) history + dead-reckon travel accounting. Both advance on EVERY tick, including
        // ticks with no fix — the off-strip case is precisely when σ_dr must keep growing.
        if (havePrev_) {
            travelSinceFix_ += std::hypot(px - prevX_, py - prevY_);
        }
        prevX_ = px;
        prevY_ = py;
        havePrev_ = true;
        push(now, px, py);

        // (2) the Driving-Skills path: no strip, no fix, no proposal — and NEVER a
        // low-confidence pull, which would drag the estimate toward whatever stale pose the
        // device happens to be serving (A4 register HA-31 makes that the origin in the model,
        // on purpose, so code that trusts it gets caught).
        if (!gps_.hasFix()) {
            ++noFixTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (3) F4 finiteness backstop. The adapter is supposed to screen PROS_ERR_F before a
        // sentinel ever gets this far (HA-08); if one does, it is a no-fix, not a pose.
        const math::Pose2d fix = gps_.pose();
        const double zx = fix.x().value();
        const double zy = fix.y().value();
        const double rms = gps_.rmsError().value();
        if (!std::isfinite(zx) || !std::isfinite(zy) || !std::isfinite(rms) || rms < 0.0) {
            ++noFixTicks_;
            return decline(diag::GateReason::RejectedNoFix);
        }

        // (4) freshness. Consume the sample HERE, before any later rejection, so a sample taken
        // during a fast spin is skipped rather than folded once the spin ends — it describes a
        // moment we already decided not to trust.
        if (haveSample_ && zx == sampleX_ && zy == sampleY_ && rms == sampleRms_) {
            ++staleTicks_;
            return decline(diag::GateReason::RejectedStaleFix);
        }
        sampleX_ = zx;
        sampleY_ = zy;
        sampleRms_ = rms;
        sampleObservedAt_ = now;
        haveSample_ = true;

        // (5) the sensor's own verdict on itself.
        if (rms > config_.maxReportedRms.value()) {
            ++qualityRejects_;
            return decline(diag::GateReason::RejectedSensorQuality);
        }

        // (6) spinning too fast to trust the geometry (header note).
        const double yawRate = imu_.yawRate().value();
        if (!std::isfinite(yawRate) || std::abs(yawRate) > config_.maxYawRate.value()) {
            ++yawRateRejects_;
            return decline(diag::GateReason::RejectedHighYawRate);
        }

        // (7) latency: carry the fix forward by the odometry travelled since it was captured.
        // The sample became readable at sampleObservedAt_ having been captured `latency` before
        // that; the (now − observed) term is zero while freshness suppression is in force and
        // stays correct if it is ever relaxed.
        const double captureTime = sampleObservedAt_ - config_.latency.value();
        double baseX = px;
        double baseY = py;
        positionAt(captureTime, baseX, baseY);
        const double zxc = zx + (px - baseX);
        const double zyc = zy + (py - baseY);

        // (8) the normalized-innovation gate (header note, tension T1).
        const double sigmaMeas =
            std::max(config_.rmsTrustFactor * rms, config_.minPositionStdDev.value());
        const double sigmaDr =
            std::hypot(config_.postFixStdDev.value(), config_.driftStdDevPerInch * travelSinceFix_);
        const double sigmaEff = std::hypot(sigmaMeas, sigmaDr);
        const double residualX = zxc - px;
        const double residualY = zyc - py;
        const double residual = std::hypot(residualX, residualY);
        if (!std::isfinite(residual) || !std::isfinite(sigmaEff) || sigmaEff <= 0.0 ||
            residual > config_.gateSigma * sigmaEff) {
            ++innovationRejects_;
            return decline(diag::GateReason::RejectedNormalizedInnovation, residualX, residualY,
                           sigmaEff);
        }

        // (9) propose. The confidence is the scalar Kalman gain (header note); positionStdDev is
        // σ_eff, which is what a fuser means by R for THIS proposal. `selfAudit` stays None on
        // purpose: the fusion policy owns the audit for a proposal that reaches it, and the
        // Localizer's substitution rule must never be able to manufacture an "Accepted" for a
        // tick where nothing was applied (see correction.hpp).
        const double sdr2 = sigmaDr * sigmaDr;
        const double confidence = sdr2 / (sdr2 + sigmaMeas * sigmaMeas);
        travelSinceFix_ = 0.0;
        ++accepted_;
        lastVerdict_ = diag::GateReason::Accepted;

        CorrectionProposal p;
        p.valid = true;
        // T3: the PREDICTED (IMU) heading rides out, never the GPS's.
        p.fieldPose = math::Pose2d{units::Length{zxc}, units::Length{zyc}, predicted.heading()};
        p.confidence = confidence;
        p.positionStdDev = units::Length{sigmaEff};
        p.providesHeading = false;
        return p;
    }

    /// Stable telemetry id — also what AppliedCorrection::source reports when this corrector is
    /// the reason the tick dead-reckoned.
    [[nodiscard]] const char* name() const noexcept override { return name_; }

    // ── per-source accounting (the "visible off-strip" requirement) ─────────────────────────

    /// What this corrector decided on the most recent propose() call.
    [[nodiscard]] diag::GateReason lastVerdict() const noexcept { return lastVerdict_; }
    /// Fixes proposed to the fusion policy since construction.
    [[nodiscard]] std::uint32_t acceptedFixes() const noexcept { return accepted_; }
    /// Ticks the source had no usable fix at all — off the strip, disconnected, or serving a
    /// non-finite read. This is the number that says "Driving Skills" out loud.
    [[nodiscard]] std::uint32_t noFixTicks() const noexcept { return noFixTicks_; }
    /// Ticks that re-read a sample already folded (the ~50 ms camera cadence against a ~100 Hz
    /// loop, so a healthy run spends MOST of its ticks here).
    [[nodiscard]] std::uint32_t staleTicks() const noexcept { return staleTicks_; }
    /// Fresh fixes declined because the device's own reported error was too large.
    [[nodiscard]] std::uint32_t qualityRejects() const noexcept { return qualityRejects_; }
    /// Fresh fixes declined because the robot was spinning too fast to trust them.
    [[nodiscard]] std::uint32_t yawRateRejects() const noexcept { return yawRateRejects_; }
    /// Fresh fixes declined by the normalized-innovation gate.
    [[nodiscard]] std::uint32_t innovationRejects() const noexcept { return innovationRejects_; }
    /// Distance the prediction has travelled since this source last proposed a fix — the input
    /// to the anti-lockout term, exposed so a test can prove the widening is real.
    [[nodiscard]] units::Length travelSinceFix() const noexcept {
        return units::Length{travelSinceFix_};
    }

private:
    /// Build a declined proposal carrying its reason (and, for the gate, the numbers the verdict
    /// was rendered on). `covarianceTrace` carries σ_eff here: on a declined tick nothing was
    /// trusted, so the fusion policy's trust-weight meaning for that slot is genuinely vacant,
    /// and σ_eff plus the residual is exactly what a reader needs to recompute ν and check the
    /// verdict from the blackbox alone. `reason` disambiguates which producer wrote it.
    [[nodiscard]] CorrectionProposal decline(diag::GateReason reason, double residualX = 0.0,
                                             double residualY = 0.0,
                                             double sigmaEff = 0.0) noexcept {
        lastVerdict_ = reason;
        CorrectionProposal p;  // valid == false
        p.selfAudit.reason = reason;
        p.selfAudit.residualX = units::Length{residualX};
        p.selfAudit.residualY = units::Length{residualY};
        p.selfAudit.covarianceTrace = sigmaEff;
        return p;
    }

    void push(double t, double x, double y) noexcept {
        hist_[head_] = Sample{t, x, y};
        head_ = (head_ + 1) % kHistory;
        if (count_ < kHistory) {
            ++count_;
        }
    }

    /// Predicted position at time `t`, by linear interpolation between the two bracketing ring
    /// samples. Clamps to the oldest sample when the ring does not reach that far back (early
    /// ticks, or a latency longer than the history): partial compensation, never extrapolation
    /// off the end of what was actually observed.
    void positionAt(double t, double& x, double& y) const noexcept {
        if (count_ == 0) {
            return;
        }
        const std::size_t oldest = (head_ + kHistory - count_) % kHistory;
        if (t <= hist_[oldest].t) {
            x = hist_[oldest].x;
            y = hist_[oldest].y;
            return;
        }
        for (std::size_t k = count_; k-- > 1;) {  // newest→oldest, looking for the bracket
            const Sample& newer = hist_[(oldest + k) % kHistory];
            const Sample& older = hist_[(oldest + k - 1) % kHistory];
            if (t >= older.t && t <= newer.t) {
                const double span = newer.t - older.t;
                const double f = span > 0.0 ? (t - older.t) / span : 1.0;
                x = older.x + f * (newer.x - older.x);
                y = older.y + f * (newer.y - older.y);
                return;
            }
        }
        const Sample& newest = hist_[(head_ + kHistory - 1) % kHistory];
        x = newest.x;  // t is at or after the newest sample: no travel to add
        y = newest.y;
    }

    struct Sample {
        double t = 0.0;
        double x = 0.0;
        double y = 0.0;
    };

    hal::IClock& clock_;
    hal::IGps& gps_;
    hal::IImu& imu_;
    GpsCorrectorConfig config_;
    const char* name_;

    std::array<Sample, kHistory> hist_{};
    std::size_t head_ = 0;
    std::size_t count_ = 0;

    double prevX_ = 0.0;
    double prevY_ = 0.0;
    bool havePrev_ = false;
    double travelSinceFix_ = 0.0;

    double sampleX_ = 0.0;
    double sampleY_ = 0.0;
    double sampleRms_ = 0.0;
    double sampleObservedAt_ = 0.0;
    bool haveSample_ = false;

    diag::GateReason lastVerdict_ = diag::GateReason::None;
    std::uint32_t accepted_ = 0;
    std::uint32_t noFixTicks_ = 0;
    std::uint32_t staleTicks_ = 0;
    std::uint32_t qualityRejects_ = 0;
    std::uint32_t yawRateRejects_ = 0;
    std::uint32_t innovationRejects_ = 0;
};

}  // namespace shulib::localization
