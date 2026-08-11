#pragma once
//
// sim::ImuHostileModel — how a V5 IMU actually lies (chunk A3, scope item 3).
// Populates the imuHeading / imuYawRate / imuReady seams of degradation.hpp.
//
// ── The failure SHAPES modeled (confident), and their MAGNITUDES (provisional) ──────
// 1. BOOT CALIBRATION WINDOW: isReady() is false until `calibrationEnd`, and the
//    readings during the window are GARBAGE THAT MOVES (uniform random heading each
//    sample) — matching how a calibrating V5 IMU jumps around. This is the pathology
//    that exposed the Localizer boot-poisoning defect (see A3-COMPLETED): garbage
//    heading swings through the odometry offset-correction become phantom
//    translation on a stationary robot.
// 2. PER-BOOT RATE BIAS → DRIFT: one yaw-rate offset is drawn per boot (uniform in
//    ±rateBiasMax, drawn lazily on the first hook call so it comes from the run's
//    seeded Rng) and heading error grows as rateBias·(t − calibrationEnd). Rate bias
//    and heading drift are ONE phenomenon, modeled consistently: imuYawRate() reports
//    trueRate + rateBias, so d(reported heading)/dt == reported rate — which is what
//    lets a Phase E filter OBSERVE the bias instead of fighting an inconsistent pair.
//    Drift references calibrationEnd because a real IMU re-zeros its frame when
//    calibration completes: at that instant its error is ~0 by construction.
// 3. WHITE NOISE on heading and rate (Gaussian, per sample).
// 4. MID-RUN DROPOUT (event, default OFF): at `dropoutAt` the device disconnects —
//    isReady() goes false and heading/rate FREEZE at their last emitted values
//    forever. Frozen-but-finite, per the F4 finiteness contract (degradation.hpp:
//    the hal/pros adapter screens PROS_ERR at the edge; what the core sees from a
//    dead device is a stale cached value, not a NaN — and math::Angle could not
//    carry a NaN anyway, by construction).
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4 measures) ─────────
// Register: docs/hardware-assumptions.md — HA-20..HA-23.
//   * rateBiasMax = 1°/min       — the pessimistic community bound for a calibrated
//                                  V5 IMU (typical reports: 0.1–0.5°/min). NOTE: at
//                                  exactly 1°/min, F2's <1° @ 60 s has ZERO margin —
//                                  the honest A3 headline; R4's measurement decides. (HA-20)
//   * headingNoiseSigma = 0.05°  — short-term heading jitter guess. (HA-21)
//   * yawRateNoiseSigma = 0.5°/s — rate noise guess. (HA-22)
//   * calibrationEnd = 2 s       — V5 IMU calibrate takes ~2–3 s. (HA-23)
// The SHAPES above are asserted by test; none of these NUMBERS are evidence of
// anything until R4 replaces them with measurements.
//
// ── Defaults are HOSTILE (the A3 convention, decided in the A3 log) ─────────────────
// Continuous pathologies (window, drift, noise) are ON by default at the provisional
// magnitudes — a default-constructed hostile model that does nothing would be a dead
// seam waiting to ship (liveness-tested). Event pathologies (dropout) default OFF and
// are enabled per scenario. Zeroing a field disables exactly that pathology; zeroing
// all of them makes the model the identity (pinned by test).
//
// Draw discipline (determinism contract): normal phase draws exactly one Gaussian per
// heading call and one per rate call (even at sigma == 0, so the stream position never
// depends on config values); the calibration phase draws exactly one uniform per
// heading/rate call; dropout draws nothing. All draws come from the plant's seeded Rng.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

namespace hostile_detail {
inline constexpr double kDegToRad = math::Angle::kPi / 180.0;
}  // namespace hostile_detail

struct ImuHostileConfig {
    /// isReady() false and garbage readings until this time. 0 disables the window.
    units::Time calibrationEnd{2.0};  // PROVISIONAL (A4: HA-23)
    /// Per-boot yaw-rate bias drawn uniform in ±this. 0 disables drift.
    units::AngularVelocity rateBiasMax{hostile_detail::kDegToRad / 60.0};  // 1°/min PROVISIONAL (A4: HA-20)
    /// White heading noise, 1σ radians. 0 disables.
    double headingNoiseSigmaRad = 0.05 * hostile_detail::kDegToRad;  // PROVISIONAL (A4: HA-21)
    /// White yaw-rate noise, 1σ rad/s. 0 disables.
    double yawRateNoiseSigmaRadPerS = 0.5 * hostile_detail::kDegToRad;  // PROVISIONAL (A4: HA-22)
    /// EVENT (default off): device disconnects at this time — ready false, values frozen.
    units::Time dropoutAt{1e18};
    /// Garbage yaw-rate magnitude bound during calibration (uniform ±this).
    units::AngularVelocity calibrationGarbageRate{10.0};  // shape only; PROVISIONAL (A4: HA-23)
};

class ImuHostileModel final : public DegradationModel {
public:
    explicit ImuHostileModel(const ImuHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.calibrationEnd.value() >= 0.0,
                            "ImuHostileModel: calibrationEnd must be >= 0");
        SHULIB_PRECONDITION(cfg_.rateBiasMax.value() >= 0.0,
                            "ImuHostileModel: rateBiasMax must be >= 0");
        SHULIB_PRECONDITION(cfg_.headingNoiseSigmaRad >= 0.0,
                            "ImuHostileModel: headingNoiseSigmaRad must be >= 0");
        SHULIB_PRECONDITION(cfg_.yawRateNoiseSigmaRadPerS >= 0.0,
                            "ImuHostileModel: yawRateNoiseSigmaRadPerS must be >= 0");
    }

    [[nodiscard]] math::Angle imuHeading(math::Angle trueHeading, units::Time now,
                                         Rng& rng) override {
        ensureBoot(rng);
        if (now.value() >= cfg_.dropoutAt.value()) {
            if (!hasLastHeading_) {  // dropout before any sample: freeze on the truth
                lastHeading_ = trueHeading;
                hasLastHeading_ = true;
            }
            return lastHeading_;  // frozen forever; no draws
        }
        math::Angle out{};
        if (now.value() < cfg_.calibrationEnd.value()) {
            // calibration garbage: a fresh uniform heading each sample (1 draw)
            out = math::Angle::radians(rng.uniform(-math::Angle::kPi, math::Angle::kPi));
        } else {
            // drift (re-zeroed at calibration end) + noise (1 Gaussian draw, always)
            const double driftRad =
                rateBias_ * (now.value() - cfg_.calibrationEnd.value());
            const double noiseRad = cfg_.headingNoiseSigmaRad * rng.nextGaussian();
            out = trueHeading + math::Angle::radians(driftRad + noiseRad);
        }
        lastHeading_ = out;
        hasLastHeading_ = true;
        return out;
    }

    [[nodiscard]] units::AngularVelocity imuYawRate(units::AngularVelocity trueRate,
                                                    units::Time now, Rng& rng) override {
        ensureBoot(rng);
        if (now.value() >= cfg_.dropoutAt.value()) {
            return lastRate_;  // frozen (default 0 if dropout precedes any sample)
        }
        units::AngularVelocity out{};
        if (now.value() < cfg_.calibrationEnd.value()) {
            const double g = cfg_.calibrationGarbageRate.value();
            out = units::AngularVelocity{rng.uniform(-g, g)};
        } else {
            // consistent with the heading model: reported = true + bias (+ noise)
            out = units::AngularVelocity{trueRate.value() + rateBias_
                                         + cfg_.yawRateNoiseSigmaRadPerS * rng.nextGaussian()};
        }
        lastRate_ = out;
        return out;
    }

    [[nodiscard]] bool imuReady(bool trueReady, units::Time now) override {
        if (now.value() >= cfg_.dropoutAt.value()) {
            return false;  // disconnected
        }
        if (now.value() < cfg_.calibrationEnd.value()) {
            return false;  // still calibrating
        }
        return trueReady;
    }

    /// The per-boot rate bias actually drawn (rad/s) — for assertions and for
    /// reporting the measured drift a run was subjected to. 0 until the first draw.
    [[nodiscard]] double rateBiasRadPerS() const noexcept { return rateBias_; }

private:
    void ensureBoot(Rng& rng) {
        if (!booted_) {
            booted_ = true;
            const double m = cfg_.rateBiasMax.value();
            rateBias_ = rng.uniform(-m, m);  // one draw per boot, always (stream stability)
        }
    }

    ImuHostileConfig cfg_;
    bool booted_ = false;
    double rateBias_ = 0.0;
    math::Angle lastHeading_{};
    bool hasLastHeading_ = false;
    units::AngularVelocity lastRate_{};
};

}  // namespace shulib::sim
