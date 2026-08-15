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
//
// `Cancelled` was appended at chunk C2 via exactly that additive path: it is an exit a
// MOTION reports after IMotion::cancel() (scheduler pre-emption, user cancel, or a
// fault-policy abort) — ExitGroup::check() itself can never return it, because settling
// and timing out are the only verdicts the group's own criteria can render. Cancellation
// is imposed from outside; the enum carries it so every consumer of "why did this motion
// end?" has one vocabulary.

#include "shulib/control/settled_util.hpp"
#include "shulib/control/watchdog.hpp"
#include "shulib/hal/clock.hpp"

namespace shulib::control {

/// Why a motion stopped — the ONE vocabulary shared by IMotion::exitReason(), the
/// scheduler, the fault path and every logged result line (§18.4 exit-reason codes).
enum class ExitReason {
    Running,   ///< no exit condition has fired yet: tick() again next loop iteration
    Settled,   ///< the settle criteria (error AND its rate) held for their full settle time
    TimedOut,  ///< the watchdog deadline passed first — the hang guard, not a tuning knob
    Cancelled,  ///< stopped from outside via IMotion::cancel() (chunk C2; never
                ///< returned by ExitGroup::check() — see header note)
};

/// Settling (success) and the watchdog (hang guard) as ONE verdict per tick. Settled WINS
/// a tie — a motion that settles on the very tick the deadline passes is a success, not a
/// timeout. The group can only ever return Running / Settled / TimedOut; Cancelled is
/// imposed from outside and never originates here. STATEFUL: check() advances the settle
/// window from the injected clock, so call it exactly once per tick, in order.
class ExitGroup {
public:
    /// `settle` is applied to whatever error check() is later fed — the motion owns the
    /// units. `timeout` is the watchdog deadline in SECONDS and must be > 0 (Watchdog's
    /// precondition). `clock` is held BY REFERENCE by both halves and must outlive the
    /// group; it is the only time source either uses. Construction arms nothing — start() does.
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

    /// The settle half, exposed for telemetry only. isSettled() here is a pure read of the
    /// verdict the last check() computed, so it is true EXACTLY when that check() returned
    /// Settled — it is not a separate "was it close?" measure, and after a TimedOut exit it
    /// reads false by construction (settling is tested first, and losing that test is what
    /// let the watchdog branch run at all). Const on purpose: check() is the one way to feed
    /// it, so a caller cannot advance the settle window behind the group's back.
    [[nodiscard]] const SettledUtil& settled() const noexcept { return settled_; }

    /// The timer half, for inspection only — elapsed() is seconds since start(), which is
    /// how long the motion has been running. Const on purpose: start() is the only legal
    /// way to (re)arm it.
    [[nodiscard]] const Watchdog& watchdog() const noexcept { return watchdog_; }

private:
    SettledUtil settled_;
    Watchdog watchdog_;
};

}  // namespace shulib::control
