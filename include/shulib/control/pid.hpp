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

/// Gains and the two bounds, every one of them in the CALLER's units (header note): kP multiplies
/// whatever error unit is fed in, and the three terms must sum to the unit the caller wants back.
/// All-default is a controller that returns 0 for every error, with no clamping anywhere.
struct PidConfig {
    /// Output per unit of error. The only term that ever applies on the first tick.
    double kP = 0.0;
    /// Output per unit of accumulated error·seconds. Exactly 0 skips integration entirely — the
    /// accumulator is not even advanced, so integralAccumulator() stays 0 for a P/PD controller.
    double kI = 0.0;
    /// Output per unit of error rate, realized as MINUS kD times the measurement's rate of change:
    /// a measurement climbing at 1 unit/s with kD = 2 SUBTRACTS 2 from the output. Differentiating
    /// the measurement instead of the error is what keeps a setpoint step from kicking D; while
    /// the setpoint is held the two agree, because then d(error)/dt = −d(measurement)/dt.
    double kD = 0.0;
    /// Symmetric ± clamp on the I-TERM (kI·∫e·dt), not on the raw accumulator — the accumulator is
    /// then back-calculated to match, which is what makes windup past this bound impossible rather
    /// than merely invisible. Must be ≥ 0; the default, infinity, is no anti-windup limit at all.
    double integralLimit = std::numeric_limits<double>::infinity();
    /// Lower clamp on the returned output, applied after P + I + D are summed. Must be ≤ outputMax
    /// (checked at construction). Default −infinity: unclamped.
    double outputMin = -std::numeric_limits<double>::infinity();
    /// Upper clamp on the returned output. Default +infinity: unclamped.
    double outputMax = std::numeric_limits<double>::infinity();
};

/// One axis of PID, distinguished from the textbook loop by three properties the header argues
/// for and the suite pins: derivative on measurement, back-calculated anti-windup, and dt taken
/// from an INJECTED clock instead of read from the OS.
///
/// STATEFUL: every update() overwrites the dt baseline and the remembered measurement, and (only
/// when kI != 0) advances the integral, so the output depends on the call history and not on this
/// tick's arguments alone. The first update() after construction or reset() has no baseline and
/// applies P only. A repeat call is NOT automatically a different number, though: with kI == 0 and
/// an unchanged measurement both I and D contribute nothing, so a P or PD controller returns the
/// same output twice. Use one instance per axis, and reset() between motions — otherwise the
/// previous motion's integral rides into the next one.
class Pid {
public:
    /// `config` is copied (later edits to the caller's struct do nothing); `clock` is a NON-OWNING
    /// reference that must outlive this controller and is the sole source of dt. Rejects non-finite
    /// gains, a negative integralLimit and outputMin > outputMax — all at construction, so a
    /// controller that cannot be trusted never reaches a match.
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

    /// setpoint − measurement as of the most recent update(), for telemetry — the law never reads
    /// it back. 0 before the first update() and after reset(); recorded on every tick, including
    /// the dt ≤ 0 ticks that contribute only P.
    [[nodiscard]] double lastError() const noexcept { return lastError_; }

    /// The raw ∫e·dt in error·seconds, AFTER the anti-windup back-calculation — multiply by kI to
    /// recover the I-term that was actually added. Stays exactly 0 when kI == 0 (nothing
    /// accumulates) and is zeroed by reset(). Exposed so a test can prove the clamp bounds the
    /// accumulator itself and not just the output.
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
