#pragma once
//
// sim::SlipHostileModel — tracking loss between wheel and floor (chunk A3, scope
// item 5). Populates the wheelMotionVelocity seam of degradation.hpp.
//
// ── Where this seam sits (the A2 D9 architecture, exploited on purpose) ─────────────
// The plant reads drive ENCODERS from the wheel's SPIN (before this hook) and moves
// the BODY from this hook's output — so anything removed here reproduces exactly how
// real slip lies: drive encoders overcount while the robot undershoots, and the
// unpowered tracking wheels keep telling the truth. The A3 tests pin BOTH halves:
// the lie (encoder-implied travel > true travel, in the predicted direction) and the
// architecture's answer to it (tracking-wheel odometry rides through drive slip).
//
// ── The failure SHAPES modeled (confident), and their MAGNITUDES (provisional) ──────
// 1. ACCELERATION-TRIGGERED SLIP (continuous, ON): when a wheel's spin is changing
//    faster than `accelThreshold`, traction breaks and only `slipRetain` of the spin
//    reaches the floor. This is the "wheels spin up faster than the robot can
//    follow" launch behaviour; with a lagged motor model it engages for exactly the
//    early part of every hard ramp (while |v_ss − v| > threshold·τ) and releases on
//    its own — a shape a test can derive and pin.
// 2. SLIP WINDOWS (events, default OFF): during [start, end) the selected wheels
//    retain only `retain` of their spin — sustained traction loss (pushing a wall,
//    carpet transition, defense contact). What a window is NOT: a contact model.
//    The honesty boundary forbids inventing contact physics; a window is a declared
//    scenario fact, not an emergent one.
//
// ── PROVISIONAL MAGNITUDES (A4 Hardware Assumptions Register; R4 measures) ─────────
//   * accelThreshold = 80 in/s² — where traction breaks on our wheels/foam, unknown.
//   * slipRetain = 0.7          — how much of the spin still propels during a slip.
//
// Determinism: no rng draws — slip is a pure function of the spin history the plant
// feeds it. State is per-wheel (lastSpin/lastNow) so wheels slip independently,
// exactly as a real drivetrain's inside/outside wheels do in a hard turn.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

#include "shulib/core/check.hpp"
#include "shulib/kinematics/wheel_speeds.hpp"
#include "shulib/sim/degradation.hpp"
#include "shulib/sim/rng.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::sim {

/// Sustained traction loss for [start, end): wheels in `wheelMask` (bit i = wheel i;
/// 0 means ALL wheels) retain only `retain` of their spin.
struct SlipWindow {
    units::Time start{};
    units::Time end{};
    double retain = 0.5;
    unsigned wheelMask = 0;  ///< 0 = every wheel
};

struct SlipHostileConfig {
    double accelThresholdInPerS2 = 80.0;  // PROVISIONAL (A4)
    double slipRetain = 0.7;              // PROVISIONAL (A4)
    std::vector<SlipWindow> windows{};    ///< events, default none
};

class SlipHostileModel final : public DegradationModel {
public:
    explicit SlipHostileModel(const SlipHostileConfig& config = {}) : cfg_{config} {
        SHULIB_PRECONDITION(cfg_.accelThresholdInPerS2 > 0.0,
                            "SlipHostileModel: accelThreshold must be > 0");
        SHULIB_PRECONDITION(cfg_.slipRetain >= 0.0 && cfg_.slipRetain <= 1.0,
                            "SlipHostileModel: slipRetain must be in [0, 1]");
        for (const SlipWindow& w : cfg_.windows) {
            SHULIB_PRECONDITION(w.end.value() >= w.start.value(),
                                "SlipHostileModel: a slip window ends before it starts");
            SHULIB_PRECONDITION(w.retain >= 0.0 && w.retain <= 1.0,
                                "SlipHostileModel: window retain must be in [0, 1]");
        }
    }

    [[nodiscard]] units::Velocity wheelMotionVelocity(int wheel, units::Velocity spin,
                                                      units::Time now, Rng& /*rng*/) override {
        const auto idx = static_cast<std::size_t>(wheel);
        double retain = 1.0;

        // 1. acceleration-triggered slip, from this wheel's own spin history
        const double dt = hasLast_[idx] ? (now.value() - lastNow_[idx]) : 0.0;
        if (dt > 0.0) {
            const double accel = (spin.value() - lastSpin_[idx]) / dt;
            if (std::abs(accel) > cfg_.accelThresholdInPerS2) {
                retain = cfg_.slipRetain;
            }
        }
        lastSpin_[idx] = spin.value();
        lastNow_[idx] = now.value();
        hasLast_[idx] = true;

        // 2. sustained slip windows (worst active window wins)
        for (const SlipWindow& w : cfg_.windows) {
            const bool timeHit = now.value() >= w.start.value() && now.value() < w.end.value();
            const bool wheelHit = w.wheelMask == 0u || ((w.wheelMask >> idx) & 1u) != 0u;
            if (timeHit && wheelHit) {
                retain = std::min(retain, w.retain);
            }
        }
        return units::Velocity{spin.value() * retain};
    }

private:
    static constexpr std::size_t kMaxWheels =
        static_cast<std::size_t>(kinematics::WheelSpeeds::kMaxWheels);

    SlipHostileConfig cfg_;
    std::array<double, kMaxWheels> lastSpin_{};
    std::array<double, kMaxWheels> lastNow_{};
    std::array<bool, kMaxWheels> hasLast_{};
};

}  // namespace shulib::sim
