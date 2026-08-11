#pragma once
//
// SettledUtil — the motion exit check (master plan §13 #9, OkapiLib-style). A motion is
// SETTLED only when ALL THREE hold continuously for a settle time:
//   * |error| ≤ maxError            (close enough), AND
//   * |d(error)/dt| ≤ maxErrorRate  (not still moving — no settling mid-overshoot), AND
//   * both have held for ≥ settleTime.
//
// Requiring the RATE (not just position) is what stops a motion from declaring victory while
// flying through the target; requiring the HELD time stops single-tick flukes. dt comes from
// an injected clock, so it is deterministic and host-testable. The first call (no rate yet)
// and any dt ≤ 0 tick never report settled.
//
// Bare double error, like Pid: the motion layer feeds per-axis error in matching units.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

struct SettleConfig {
    double maxError = 0.0;      // |error| must be within this
    double maxErrorRate = 0.0;  // |d(error)/dt| must be within this (error-units per second)
    double settleTime = 0.0;    // both must hold continuously for this long (seconds)
};

class SettledUtil {
public:
    SettledUtil(const SettleConfig& config, hal::IClock& clock) : cfg_{config}, clock_{clock} {
        SHULIB_PRECONDITION(cfg_.maxError >= 0.0, "SettledUtil: maxError must be >= 0");
        SHULIB_PRECONDITION(cfg_.maxErrorRate >= 0.0, "SettledUtil: maxErrorRate must be >= 0");
        SHULIB_PRECONDITION(cfg_.settleTime >= 0.0, "SettledUtil: settleTime must be >= 0");
    }

    /// Feed the current error; returns true once settled (and stays true while it remains so).
    [[nodiscard]] bool update(double error) {
        const double now = clock_.now().value();
        if (!hasPrev_) {  // first call: no rate yet
            prevError_ = error;
            prevTime_ = now;
            hasPrev_ = true;
            return false;
        }
        const double dt = now - prevTime_;
        if (dt <= 0.0) {  // can't re-evaluate the rate; hold the prior verdict
            return settled_;
        }
        const double rate = std::abs((error - prevError_) / dt);
        prevError_ = error;
        prevTime_ = now;

        const bool within = std::abs(error) <= cfg_.maxError && rate <= cfg_.maxErrorRate;
        if (within) {
            if (!inWindow_) {
                inWindow_ = true;
                windowStart_ = now;
            }
            settled_ = (now - windowStart_) >= cfg_.settleTime;
        } else {
            inWindow_ = false;
            settled_ = false;
        }
        return settled_;
    }

    void reset() {
        hasPrev_ = false;
        inWindow_ = false;
        settled_ = false;
    }

    [[nodiscard]] bool isSettled() const noexcept { return settled_; }

private:
    SettleConfig cfg_;
    hal::IClock& clock_;
    double prevError_ = 0.0;
    double prevTime_ = 0.0;
    double windowStart_ = 0.0;
    bool hasPrev_ = false;
    bool inWindow_ = false;
    bool settled_ = false;
};

}  // namespace shulib::control
