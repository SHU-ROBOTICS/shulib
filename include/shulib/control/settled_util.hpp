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

/// The three-part settle criterion, as one value. Bare doubles like PidConfig: the UNITS
/// are the caller's error units and this struct never learns them — MotionConfig's three
/// instances are where they are pinned (inches for translation, radians for heading, in/s
/// for DriveBrake's averaged speed norm). Every default is 0, which is deliberately NOT a
/// working configuration: it demands an exactly-zero error at an exactly-zero rate, so a
/// default-constructed SettleConfig effectively never settles. Set all three.
struct SettleConfig {
    double maxError = 0.0;      ///< |error| ≤ this is close enough (INCLUSIVE bound).
    double maxErrorRate = 0.0;  ///< |d(error)/dt| ≤ this (units/s) — no settling mid-overshoot.
    double settleTime = 0.0;    ///< Both must hold continuously for this long (s); 0 ⇒ one tick.
};

/// The exit check as a stateful object: feed it one error per tick, it answers "settled?".
/// It measures the error RATE itself by differencing consecutive update() calls against the
/// injected clock, so it is not interchangeable with a stateless predicate: the answer
/// depends on the history since the last reset(), not on this tick's error alone. Calling
/// update() twice in one tick is SAFE but lossy — the second call sees dt == 0, mutates
/// nothing and repeats the previous verdict, so the error it was handed is DISCARDED and
/// never becomes the rate baseline. One instance per criterion, owned by the motion and
/// baseline carried across a motion boundary would differentiate a jump that never happened.
class SettledUtil {
public:
    /// All three bounds must be ≥ 0 or this raises. `clock` is BORROWED — it must outlive
    /// this object, and it is the same clock the rest of the motion reads, which is what
    /// makes the settle verdict reproducible under a fake clock. Construction opens no
    /// window: the first update() only establishes the rate baseline.
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

    /// Drop the rate baseline, the open window AND the verdict. The next update() ALWAYS
    /// answers false — it only re-establishes the baseline, so there is no rate yet — and
    /// the one after it can answer true only when settleTime is 0, because the window it
    /// opens starts at that same instant; with settleTime > 0 the earliest true arrives
    /// settleTime later, however close to target the robot already is. Call it at every
    /// motion start(): re-using a baseline across motions differentiates the gap between
    /// two unrelated errors. The config and the clock reference are untouched.
    void reset() {
        hasPrev_ = false;
        inWindow_ = false;
        settled_ = false;
    }

    /// The verdict the last update() computed — a pure read that touches neither the clock
    /// nor the window, so it is safe to call repeatedly per tick (telemetry, exit logic).
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
