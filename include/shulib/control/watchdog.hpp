#pragma once
//
// Watchdog — a hard timeout primitive (master plan §M2: "a motion can never hang"). Clock-
// driven: start() records the clock time; expired() becomes true once `timeout` seconds have
// elapsed. The motion layer arms one per motion, so even a stalled control loop still exits
// (→ TimedOut) and the guaranteed end-of-run park can fire. Reusable for any bounded wait.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

/// A hard timeout: arm it, then ask whether the deadline has passed (§M2, "a motion can never
/// hang"). CLOCK-driven rather than tick-counting, so a control loop that runs SLOW still times
/// out after the same interval on the injected clock's timeline instead of after some fixed
/// number of ticks — which is what lets a crawling motion exit as TimedOut and its guaranteed
/// end-of-run park fire.
///
/// It is a POLLED predicate, not a timer. Nothing here owns a task, a callback or an alarm:
/// expired() computes clock-now minus start-time only when someone asks, and the only callers are
/// the motions' own tick(). A caller that stops asking — a tick() blocked on a deadlocked mutex,
/// a control task that died — is NOT rescued by this class, so the §M2 guarantee reaches exactly
/// as far as the polling does.
///
/// The motion layer arms one per motion, but nothing here is motion-specific: it bounds any wait.
///
/// Constructed DISARMED — expired() is false and elapsed() is a precondition failure until
/// start(). The clock is held by non-owning reference and must outlive the Watchdog.
class Watchdog {
public:
    /// `timeout` is in SECONDS and must be FINITE and > 0; a zero or negative deadline is a
    /// caller bug, not a request to fire immediately, and an infinite one is a watchdog that
    /// can never expire — the single thing this class exists to make impossible. Finiteness
    /// was unchecked until DEFECTS1: `> 0.0` is satisfied by infinity, so `cfg.defaultTimeout
    /// = inf` built a motion that ran forever with no TimedOut exit. `clock` is stored by
    /// reference, never copied — pass the same IClock the surrounding loop reads, so the
    /// deadline lives on one timeline (and in a test, on the fake clock the test advances).
    /// Does NOT begin counting: call start().
    Watchdog(double timeout, hal::IClock& clock) : timeout_{timeout}, clock_{clock} {
        SHULIB_PRECONDITION(std::isfinite(timeout) && timeout > 0.0,
                            "Watchdog: timeout must be finite and > 0");
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

    /// Whether it is armed — i.e. start() has been called and reset() has not. Says nothing about
    /// whether the deadline has passed; that is expired().
    [[nodiscard]] bool started() const noexcept { return started_; }
    /// DISARM, not "restart". expired() reverts to false, elapsed() becomes a precondition
    /// failure again, and no deadline is running until the next start(). To begin the countdown
    /// again from now, call start() — it re-arms on its own and does not need this first.
    void reset() noexcept { started_ = false; }

private:
    double timeout_;
    hal::IClock& clock_;
    double startTime_ = 0.0;
    bool started_ = false;
};

}  // namespace shulib::control
