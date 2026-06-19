#pragma once
//
// ExitReason / ExitGroup — the motion-exit decision (master plan §M2; §18.4 exit-reason
// codes). A motion exits when it SETTLES (success) or the WATCHDOG fires (TimedOut — the
// hang guard). check() reports WHICH fired, so every IMotion can log a motion exit-reason.
// Settled takes priority over a simultaneous timeout (a motion that settled right at the
// deadline still counts as a success). start() arms both (resets settling, starts the timer).
//
// More exit conditions (e.g. stall via motor current) are additive later — they slot in as
// extra branches without changing this contract.

#include "shulib/control/settled_util.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

enum class ExitReason { Running, Settled, TimedOut };

class ExitGroup {
public:
    ExitGroup(const SettleConfig& settle, double timeout, hal::IClock& clock)
        : settled_{settle, clock}, watchdog_{timeout, clock} {}

    /// Arm the group at the start of a motion.
    void start() {
        settled_.reset();
        watchdog_.start();
    }

    /// One tick: feed the current error, get the exit verdict.
    [[nodiscard]] ExitReason check(double error) {
        if (settled_.update(error)) {
            return ExitReason::Settled;
        }
        if (watchdog_.expired()) {
            return ExitReason::TimedOut;
        }
        return ExitReason::Running;
    }

    [[nodiscard]] const SettledUtil& settled() const noexcept { return settled_; }
    [[nodiscard]] const Watchdog& watchdog() const noexcept { return watchdog_; }

private:
    SettledUtil settled_;
    Watchdog watchdog_;
};

}  // namespace shulib::control
