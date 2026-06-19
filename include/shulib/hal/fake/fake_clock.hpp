#pragma once
//
// FakeClock — a deterministic IClock for host tests. Time advances ONLY when the
// test moves it, so timed logic (dt, profiles, watchdog, settling) is reproducible
// to the tick instead of depending on wall-clock timing.
//
// It also ENFORCES the IClock monotonicity contract: advancing by a negative dt,
// or setting time backward, is a precondition violation (red-on-failure) — so a
// test that would silently rely on time going backward fails loudly instead.

#include "shulib/core/check.hpp"
#include "shulib/hal/clock.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::fake {

class FakeClock final : public IClock {
public:
    explicit FakeClock(units::Time start = units::Time{0.0}) : t_{start} {}

    [[nodiscard]] units::Time now() const override { return t_; }

    /// Advance the clock by a NON-NEGATIVE delta (monotonicity guard).
    void advance(units::Time dt) {
        SHULIB_PRECONDITION(dt.value() >= 0.0,
                            "FakeClock::advance: dt must be >= 0 (the clock is monotonic)");
        t_ += dt;
    }

    /// Jump to an absolute time at or after the current time (never backward).
    void set(units::Time t) {
        SHULIB_PRECONDITION(t.value() >= t_.value(),
                            "FakeClock::set: time cannot move backward (the clock is monotonic)");
        t_ = t;
    }

private:
    units::Time t_;
};

}  // namespace shulib::hal::fake
