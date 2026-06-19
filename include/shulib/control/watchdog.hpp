#pragma once
//
// Watchdog — a hard timeout primitive (master plan §M2: "a motion can never hang"). Clock-
// driven: start() records the clock time; expired() becomes true once `timeout` seconds have
// elapsed. The motion layer arms one per motion, so even a stalled control loop still exits
// (→ TimedOut) and the guaranteed end-of-run park can fire. Reusable for any bounded wait.

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

class Watchdog {
public:
    Watchdog(double timeout, hal::IClock& clock) : timeout_{timeout}, clock_{clock} {
        SHULIB_PRECONDITION(timeout > 0.0, "Watchdog: timeout must be > 0");
    }

    /// (Re)arm: start counting from now.
    void start() {
        startTime_ = clock_.now().value();
        started_ = true;
    }

    /// Seconds since start(). Precondition: started.
    [[nodiscard]] double elapsed() const {
        SHULIB_PRECONDITION(started_, "Watchdog::elapsed: not started");
        return clock_.now().value() - startTime_;
    }

    /// True once `timeout` has elapsed since start(); always false before start().
    [[nodiscard]] bool expired() const {
        return started_ && (clock_.now().value() - startTime_) >= timeout_;
    }

    [[nodiscard]] bool started() const noexcept { return started_; }
    void reset() noexcept { started_ = false; }

private:
    double timeout_;
    hal::IClock& clock_;
    double startTime_ = 0.0;
    bool started_ = false;
};

}  // namespace shulib::control
