#pragma once
//
// sim::PowerHostileModel — battery sag, brownout collapse and motor thermal droop
// (chunk A3, scope items 6/8). Populates the effectiveVoltage / batteryVoltage seams.
//
// ── The failure SHAPES modeled (confident), and their MAGNITUDES (provisional) ──────
// 1. SAG UNDER LOAD (continuous, ON): pack voltage = nominal − discharge·t −
//    sagPerCommandedVolt·(last tick's Σ|commanded|). Commanded voltage is the load
//    PROXY — the honesty boundary (motor_model.hpp) forbids a real current model, and
//    demand is the one load signal the plant truthfully has. Consequence a test can
//    pin: driving hard reads a lower battery than idling, and releasing the sticks
//    recovers the reading — the exact shape drivers see on the brain screen.
// 2. PACK CEILING: a motor can never receive more than the sagged pack voltage —
//    effective = clamp(commanded, ±pack). On a fresh pack (>12 V) this never engages
//    below the F4 ±12 V clamp; on a sagging pack it is the "robot got slower at the
//    end of the match" effect.
// 3. BROWNOUT COLLAPSE: at/below `cutoffVolts` the brain cuts motor power —
//    effective = 0 for every wheel while the condition holds. The run itself
//    CONTINUES (the CPU survives on the supercap; motors die first) — which is
//    precisely why the guaranteed-park logic (F2 chunk) can exist at all. That
//    motors-die-first/CPU-survives premise is itself unverified: A4 register HA-19.
// 4. THERMAL DROOP (continuous accumulation, droop only when hot): per-wheel
//    temperature integrates heatRate·V²·dt and cools toward ambient; at the V5's
//    documented throttle steps the effective voltage is scaled:
//        T ≥ 55 °C → ×0.50,  T ≥ 60 °C → ×0.25,  T ≥ 65 °C → ×0.125.
//    The STEP SHAPE is VEX-documented behaviour (current limiting to 50/25/12.5%);
//    representing a current limit as a voltage scale is a PROXY — our kinematic
//    plant has no torque, so "half the current" is modeled as "half the drive".
//    Recorded honestly: CURRENT LIMITING per se (the 2.5 A cap under stall) is NOT
//    modeled — it needs the load model the honesty boundary forbids; R6 back-fits
//    (A4 register HA-49).
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4/R5 measure) ───────
// Register: docs/planning/hardware-assumptions.md — HA-40..HA-44 (+HA-19 brownout
// survivability, HA-49 unmodeled current limiting).
//   * sagPerCommandedVolt = 0.02 V/V  — ≈1 V sag with 4 motors at full 12 V. (HA-40)
//   * dischargeRate = 0.005 V/s       — ≈0.3 V over a 60 s run. (HA-41)
//   * cutoffVolts = 10.5 V            — where the brain starts cutting motor power
//                                       (align with HealthMonitorConfig.brownoutVolts). (HA-42)
//   * heatRatePerV2 = 0.0023 °C/(V²s) — full-drive reaches ~55 °C in ~90 s from 25 °C. (HA-43)
//   * coolRate = 0.01 /s              — cooling time constant ~100 s. (HA-43)
//   * throttle steps 55/60/65 °C      — VEX-documented shape, onset unmeasured on
//                                       our motors until R4. (HA-44)
//
// The plant does NOT synthesize motor temperature (no seam exists — an A2 decision,
// kept); tests that want FakeMotor::temperature() to agree with this model push
// temperatureC(i) into the fake themselves and feed it to the HealthMonitor.
//
// Determinism: no rng draws — sag/thermal are pure functions of the command history
// and time. Load accounting relies on the plant's documented call order (all
// effectiveVoltage calls for one tick share one `now`; batteryVoltage is called
// after the clock advanced), so per-wheel caps use the PREVIOUS tick's completed
// load — order-independent across wheels — while the reported battery uses the
// just-completed tick's load. One tick of skew, documented, far inside the fidelity
// of the provisional constants.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

struct PowerHostileConfig {
    double sagPerCommandedVolt = 0.02;  // PROVISIONAL (A4: HA-40)
    double dischargeRatePerS = 0.005;   // PROVISIONAL (A4: HA-41)
    units::Voltage cutoffVolts{10.5};   // PROVISIONAL (A4: HA-42)
    double ambientC = 25.0;             // PROVISIONAL (A4: HA-43)
    double heatRatePerV2 = 0.0023;      // PROVISIONAL (A4: HA-43)
    double coolRatePerS = 0.01;         // PROVISIONAL (A4: HA-43)
    double throttleTempC = 55.0;        // PROVISIONAL (A4: HA-44): first VEX throttle step
    /// Fallback pack nominal if effectiveVoltage is ever consulted before the
    /// plant's construction-time batteryVoltage call captures the real nominal.
    units::Voltage fallbackNominal{12.6};
};

class PowerHostileModel final : public DegradationModel {
public:
    explicit PowerHostileModel(const PowerHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.sagPerCommandedVolt >= 0.0,
                            "PowerHostileModel: sagPerCommandedVolt must be >= 0");
        SHULIB_PRECONDITION(cfg_.dischargeRatePerS >= 0.0,
                            "PowerHostileModel: dischargeRatePerS must be >= 0");
        SHULIB_PRECONDITION(cfg_.cutoffVolts.value() >= 0.0,
                            "PowerHostileModel: cutoffVolts must be >= 0");
        SHULIB_PRECONDITION(cfg_.heatRatePerV2 >= 0.0 && cfg_.coolRatePerS >= 0.0,
                            "PowerHostileModel: thermal rates must be >= 0");
        SHULIB_PRECONDITION(cfg_.throttleTempC > cfg_.ambientC,
                            "PowerHostileModel: throttleTempC must exceed ambientC");
        temps_.fill(cfg_.ambientC);
    }

    [[nodiscard]] units::Voltage effectiveVoltage(int wheel, units::Voltage commanded,
                                                  units::Time now, Rng& /*rng*/) override {
        const auto idx = static_cast<std::size_t>(wheel);

        // Load accounting (header note): roll the accumulator on a new tick time.
        if (!hasLoadTick_ || now.value() != loadTickNow_) {
            prevTickLoad_ = hasLoadTick_ ? tickLoad_ : 0.0;
            tickLoad_ = 0.0;
            loadTickNow_ = now.value();
            hasLoadTick_ = true;
        }
        tickLoad_ += std::abs(commanded.value());

        // The sagged pack this wheel sees (previous completed tick's load).
        const double pack = packVoltsAt(now.value(), prevTickLoad_);
        if (pack <= cfg_.cutoffVolts.value()) {
            return units::Voltage{0.0};  // brownout: the brain cut motor power
        }

        // Thermal state advances on this wheel's own clock (dt from consecutive calls).
        double& temp = temps_[idx];
        const double dt = wheelHasLast_[idx] ? (now.value() - wheelLastNow_[idx]) : 0.0;
        wheelLastNow_[idx] = now.value();
        wheelHasLast_[idx] = true;
        const double heatV = std::min(std::abs(commanded.value()), pack);
        if (dt > 0.0) {
            temp += cfg_.heatRatePerV2 * heatV * heatV * dt
                    - cfg_.coolRatePerS * (temp - cfg_.ambientC) * dt;
            temp = std::max(temp, cfg_.ambientC);
        }

        const double capped = std::clamp(commanded.value(), -pack, pack);
        return units::Voltage{capped * droopFactor(temp)};
    }

    [[nodiscard]] units::Voltage batteryVoltage(units::Voltage nominal, units::Time now,
                                                Rng& /*rng*/) override {
        nominal_ = nominal.value();  // capture the plant's configured pack nominal
        hasNominal_ = true;
        // Reported battery: the just-completed tick's load (header note on the skew).
        const double load = (hasLoadTick_ && now.value() != loadTickNow_) ? tickLoad_
                                                                          : prevTickLoad_;
        return units::Voltage{std::max(packVoltsAt(now.value(), load), 0.0)};
    }

    /// This wheel's modeled temperature (°C) — for pushing into FakeMotor and the
    /// HealthMonitor (the plant has no temperature seam; header note).
    [[nodiscard]] double temperatureC(int wheel) const {
        SHULIB_PRECONDITION(wheel >= 0 && wheel < static_cast<int>(temps_.size()),
                            "PowerHostileModel::temperatureC: wheel out of range");
        return temps_[static_cast<std::size_t>(wheel)];
    }

    /// The hottest modeled wheel (°C) — the HealthMonitor observable.
    [[nodiscard]] double maxTemperatureC() const noexcept {
        double m = cfg_.ambientC;
        for (double t : temps_) {
            m = std::max(m, t);
        }
        return m;
    }

private:
    [[nodiscard]] double packVoltsAt(double t, double load) const noexcept {
        const double v0 = hasNominal_ ? nominal_ : cfg_.fallbackNominal.value();
        return v0 - cfg_.dischargeRatePerS * t - cfg_.sagPerCommandedVolt * load;
    }

    /// The VEX throttle steps, offset from the configured first step (55 → 50%,
    /// +5 → 25%, +10 → 12.5%).
    [[nodiscard]] double droopFactor(double tempC) const noexcept {
        if (tempC >= cfg_.throttleTempC + 10.0) {
            return 0.125;
        }
        if (tempC >= cfg_.throttleTempC + 5.0) {
            return 0.25;
        }
        if (tempC >= cfg_.throttleTempC) {
            return 0.5;
        }
        return 1.0;
    }

    static constexpr std::size_t kMaxWheels =
        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels);

    PowerHostileConfig cfg_;
    std::array<double, kMaxWheels> temps_{};
    std::array<double, kMaxWheels> wheelLastNow_{};
    std::array<bool, kMaxWheels> wheelHasLast_{};
    double nominal_ = 0.0;
    bool hasNominal_ = false;
    double tickLoad_ = 0.0;
    double prevTickLoad_ = 0.0;
    double loadTickNow_ = 0.0;
    bool hasLoadTick_ = false;
};

}  // namespace shulib::sim
