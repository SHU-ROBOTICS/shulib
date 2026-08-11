#pragma once
//
// sim::ChainedDegradation + FullHostility + JitterSchedule — composition (chunk A3,
// scope item 9): every degradation independently injectable AND stackable, with a
// canonical "everything hostile at once" model.
//
// ── ChainedDegradation: composition = function composition ──────────────────────────
// Each hook folds left→right: model[1] receives model[0]'s output as its "truth".
// That is the whole design — one pathology per model class, and a composed world is
// a list. The property that makes composed failures DEBUGGABLE (the brief's
// reducibility requirement): removing one model from the list removes exactly that
// pathology, so a composed-run failure is attributed by ablation, not archaeology.
// (Removing a model does shift which Rng draws the others see — two configurations
// are two scenarios, each individually byte-reproducible from the seed; determinism
// is per-configuration, not across configurations.)
//
// ── FullHostility: the canonical composed model ─────────────────────────────────────
// Owns one of each family model and chains them in the DOCUMENTED physical order:
//
//     power → slip → imu → gps → encoders → latency
//
// Why this order: power corrupts what reaches the motors (physics side, before
// sensors exist); slip corrupts what the spinning wheel gives the floor; the sensor
// models corrupt each device AT ITS SOURCE (bias/noise/garbage, then the encoder
// model's quantize-at-the-device); and latency is LAST because the smart-port bus
// delays whatever the device actually produced — a delayed reading of a quantized,
// noisy value, which is what real hardware hands you. (power and slip act on
// disjoint seams from the sensor models, so only the sensor-side order is
// load-bearing; it is pinned by test anyway so it cannot silently change.)
//
// Its DEFAULTS are the continuous, realistic pathologies (calibration window, IMU
// drift + noise, GPS decimation + noise, encoder quantization, battery sag +
// discharge, accel-triggered slip, IMU/GPS latency) — the world every stack test
// SHOULD be run against once it claims robustness. Event pathologies (dropouts,
// sentinel breach, bad fix, slip windows, brownout-grade sag) are enabled per
// scenario through the config. A default-constructed FullHostility measurably
// degrades every seam family (liveness-pinned) — a dead composed model cannot ship.
//
// ── JitterSchedule: hostile TIMING lives outside the plant ──────────────────────────
// Loop jitter is the runner's dt-schedule seam (degradation.hpp's note; the
// SimHarness::runTicksVariable overload) — this is the canonical hostile schedule:
// dt = nominal·(1 ± jitterFrac uniform), with probability spikeProb replaced by
// nominal·spikeFactor (a blocked task / contended scheduler stall). It owns a
// PRIVATE Rng (seeded at construction) so the dt sequence is reproducible
// independently of how many draws the degradation models consume — timing hostility
// and sensor hostility compose without entangling their streams. Draw discipline:
// exactly two draws per tick (spike roll, jitter), regardless of outcome.
//   * jitterFrac = 0.2, spikeProb = 0.02, spikeFactor = 5 — PROVISIONAL (A4: HA-34):
//     PROS task-contention statistics are unmeasured until R-phase telemetry.
//     Register: docs/hardware-assumptions.md — HA-34 (jitter), HA-32 (loop rate).

#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/math/angle.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/hostile/encoder_hostility.hpp"
#include "shulib/sim/hostile/gps_hostility.hpp"
#include "shulib/sim/hostile/imu_hostility.hpp"
#include "shulib/sim/hostile/latency_hostility.hpp"
#include "shulib/sim/hostile/power_hostility.hpp"
#include "shulib/sim/hostile/slip_hostility.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

class ChainedDegradation final : public DegradationModel {
public:
    /// Non-owning: every pointed-to model must outlive the chain. An empty chain is
    /// the identity (pinned by test).
    explicit ChainedDegradation(std::vector<DegradationModel*> models)
        : models_{std::move(models)} {
        for (DegradationModel* m : models_) {
            SHULIB_PRECONDITION(m != nullptr, "ChainedDegradation: a model is null");
        }
    }

    [[nodiscard]] units::Voltage effectiveVoltage(int wheel, units::Voltage commanded,
                                                  units::Time now, Rng& rng) override {
        units::Voltage v = commanded;
        for (DegradationModel* m : models_) {
            v = m->effectiveVoltage(wheel, v, now, rng);
        }
        return v;
    }

    [[nodiscard]] units::Velocity wheelMotionVelocity(int wheel, units::Velocity spin,
                                                      units::Time now, Rng& rng) override {
        units::Velocity v = spin;
        for (DegradationModel* m : models_) {
            v = m->wheelMotionVelocity(wheel, v, now, rng);
        }
        return v;
    }

    [[nodiscard]] units::AngleDim driveEncoderPosition(int wheel, units::AngleDim trueShaft,
                                                       units::Time now, Rng& rng) override {
        units::AngleDim v = trueShaft;
        for (DegradationModel* m : models_) {
            v = m->driveEncoderPosition(wheel, v, now, rng);
        }
        return v;
    }

    [[nodiscard]] units::AngleDim trackingEncoderPosition(int wheelIndex, units::AngleDim trueShaft,
                                                          units::Time now, Rng& rng) override {
        units::AngleDim v = trueShaft;
        for (DegradationModel* m : models_) {
            v = m->trackingEncoderPosition(wheelIndex, v, now, rng);
        }
        return v;
    }

    [[nodiscard]] math::Angle imuHeading(math::Angle trueHeading, units::Time now,
                                         Rng& rng) override {
        math::Angle v = trueHeading;
        for (DegradationModel* m : models_) {
            v = m->imuHeading(v, now, rng);
        }
        return v;
    }

    [[nodiscard]] units::AngularVelocity imuYawRate(units::AngularVelocity trueRate,
                                                    units::Time now, Rng& rng) override {
        units::AngularVelocity v = trueRate;
        for (DegradationModel* m : models_) {
            v = m->imuYawRate(v, now, rng);
        }
        return v;
    }

    [[nodiscard]] bool imuReady(bool trueReady, units::Time now) override {
        bool v = trueReady;
        for (DegradationModel* m : models_) {
            v = m->imuReady(v, now);
        }
        return v;
    }

    [[nodiscard]] GpsTruth gps(const GpsTruth& truth, units::Time now, Rng& rng) override {
        GpsTruth v = truth;
        for (DegradationModel* m : models_) {
            v = m->gps(v, now, rng);
        }
        return v;
    }

    [[nodiscard]] units::Voltage batteryVoltage(units::Voltage nominal, units::Time now,
                                                Rng& rng) override {
        units::Voltage v = nominal;
        for (DegradationModel* m : models_) {
            v = m->batteryVoltage(v, now, rng);
        }
        return v;
    }

private:
    std::vector<DegradationModel*> models_;
};

struct FullHostilityConfig {
    PowerHostileConfig power{};
    SlipHostileConfig slip{};
    ImuHostileConfig imu{};
    GpsHostileConfig gps{};
    EncoderHostileConfig encoders{};
    LatencyHostileConfig latency{};
};

/// The canonical "everything hostile at once" world (header note). Hand
/// `&full.model()` to the SimHarness; the sub-model accessors exist for assertions
/// (drawn drift rate, modeled motor temperatures).
class FullHostility {
public:
    explicit FullHostility(const FullHostilityConfig& config = {})
        : power_{config.power},
          slip_{config.slip},
          imu_{config.imu},
          gps_{config.gps},
          encoders_{config.encoders},
          latency_{config.latency},
          chain_{{&power_, &slip_, &imu_, &gps_, &encoders_, &latency_}} {}

    [[nodiscard]] DegradationModel& model() noexcept { return chain_; }
    [[nodiscard]] PowerHostileModel& power() noexcept { return power_; }
    [[nodiscard]] SlipHostileModel& slip() noexcept { return slip_; }
    [[nodiscard]] ImuHostileModel& imu() noexcept { return imu_; }
    [[nodiscard]] GpsHostileModel& gps() noexcept { return gps_; }
    [[nodiscard]] EncoderHostileModel& encoders() noexcept { return encoders_; }
    [[nodiscard]] LatencyHostileModel& latency() noexcept { return latency_; }

private:
    PowerHostileModel power_;
    SlipHostileModel slip_;
    ImuHostileModel imu_;
    GpsHostileModel gps_;
    EncoderHostileModel encoders_;
    LatencyHostileModel latency_;
    ChainedDegradation chain_;
};

struct JitterScheduleConfig {
    units::Time nominal{0.01};
    double jitterFrac = 0.2;   // PROVISIONAL (A4: HA-34)
    double spikeProb = 0.02;   // PROVISIONAL (A4: HA-34)
    double spikeFactor = 5.0;  // PROVISIONAL (A4: HA-34)
};

/// The canonical hostile dt schedule for SimHarness::runTicksVariable (header note).
/// Deterministic from its own seed; exactly two draws per call.
class JitterSchedule {
public:
    explicit JitterSchedule(std::uint64_t seed, const JitterScheduleConfig& config = {})
        : cfg_{config}, rng_{seed} {
        SHULIB_PRECONDITION(cfg_.nominal.value() > 0.0, "JitterSchedule: nominal must be > 0");
        SHULIB_PRECONDITION(cfg_.jitterFrac >= 0.0 && cfg_.jitterFrac < 1.0,
                            "JitterSchedule: jitterFrac must be in [0, 1)");
        SHULIB_PRECONDITION(cfg_.spikeProb >= 0.0 && cfg_.spikeProb <= 1.0,
                            "JitterSchedule: spikeProb must be in [0, 1]");
        SHULIB_PRECONDITION(cfg_.spikeFactor >= 1.0, "JitterSchedule: spikeFactor must be >= 1");
    }

    [[nodiscard]] units::Time operator()(int /*tick*/) {
        const double spikeRoll = rng_.nextUnit();                              // draw 1
        const double jitter = rng_.uniform(-cfg_.jitterFrac, cfg_.jitterFrac);  // draw 2
        const double n = cfg_.nominal.value();
        if (spikeRoll < cfg_.spikeProb) {
            return units::Time{n * cfg_.spikeFactor};
        }
        return units::Time{n * (1.0 + jitter)};
    }

private:
    JitterScheduleConfig cfg_;
    Rng rng_;
};

}  // namespace shulib::sim
