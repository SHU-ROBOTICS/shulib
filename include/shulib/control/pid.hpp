#pragma once
//
// Pid — a single-axis PID controller (master plan §M2 control). The design choices that
// matter, each pinned by a test:
//  * DERIVATIVE ON MEASUREMENT (not on error): D differentiates the measurement, so a
//    setpoint step produces NO derivative kick — only real motion drives D.
//  * INTEGRAL ANTI-WINDUP: the I-term is clamped to ±integralLimit and the accumulator is
//    back-calculated, so it can never wind up past the clamp.
//  * OUTPUT CLAMP to [outputMin, outputMax].
//  * INJECTED CLOCK for dt — deterministic and host-testable via FakeClock. The first
//    update (and any dt ≤ 0 tick) applies P ONLY: no derivative divide-by-zero, no integral
//    step on a zero interval.
//
// Units are bare `double` BY DESIGN: a PID is a generic numeric law; the caller (the motion
// layer) applies it per-axis with matching units (error in inches/radians → output in
// in/s·rad/s or volts) and owns unit consistency. Inputs are assumed finite (guaranteed by
// the HAL finiteness convention, §7).

#include <algorithm>
#include <cmath>
#include <limits>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

struct PidConfig {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
    double integralLimit = std::numeric_limits<double>::infinity();  // symmetric ± clamp on the I-term
    double outputMin = -std::numeric_limits<double>::infinity();
    double outputMax = std::numeric_limits<double>::infinity();
};

class Pid {
public:
    Pid(const PidConfig& config, hal::IClock& clock) : cfg_{config}, clock_{clock} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.kP) && std::isfinite(cfg_.kI) && std::isfinite(cfg_.kD),
                            "Pid: gains must be finite");
        SHULIB_PRECONDITION(cfg_.integralLimit >= 0.0, "Pid: integralLimit must be >= 0");
        SHULIB_PRECONDITION(cfg_.outputMin <= cfg_.outputMax, "Pid: outputMin must be <= outputMax");
    }

    /// One control step: returns the clamped control output for (setpoint − measurement).
    [[nodiscard]] double update(double setpoint, double measurement) {
        const double now = clock_.now().value();
        const double err = setpoint - measurement;
        lastError_ = err;
        double out = cfg_.kP * err;  // P always applies

        if (hasPrev_) {
            const double dt = now - lastTime_;
            if (dt > 0.0) {
                if (cfg_.kI != 0.0) {
                    integral_ += err * dt;
                    const double iTerm =
                        std::clamp(cfg_.kI * integral_, -cfg_.integralLimit, cfg_.integralLimit);
                    integral_ = iTerm / cfg_.kI;  // anti-windup back-calculation
                    out += iTerm;
                }
                // derivative ON MEASUREMENT (negated): a setpoint step does NOT kick D
                out -= cfg_.kD * (measurement - prevMeasurement_) / dt;
            }
        }

        hasPrev_ = true;
        lastTime_ = now;
        prevMeasurement_ = measurement;
        return std::clamp(out, cfg_.outputMin, cfg_.outputMax);
    }

    /// Clear integral + derivative history (e.g. between motions). Gains/limits unchanged.
    void reset() {
        integral_ = 0.0;
        prevMeasurement_ = 0.0;
        lastError_ = 0.0;
        hasPrev_ = false;
    }

    [[nodiscard]] double lastError() const noexcept { return lastError_; }
    [[nodiscard]] double integralAccumulator() const noexcept { return integral_; }

private:
    PidConfig cfg_;
    hal::IClock& clock_;
    double integral_ = 0.0;
    double prevMeasurement_ = 0.0;
    double lastTime_ = 0.0;
    double lastError_ = 0.0;
    bool hasPrev_ = false;
};

}  // namespace shulib::control
