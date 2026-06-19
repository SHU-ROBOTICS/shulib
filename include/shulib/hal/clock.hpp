#pragma once
//
// IClock — the single source of "now" for the whole stack. Injecting time (rather
// than calling pros::millis() / the OS directly) is what makes every timed
// behavior — PID dt, motion profiles, the motion watchdog, settling — DETERMINISTIC
// and host-testable. The real V5 clock and a deterministic test clock are just two
// implementations of this one interface.
//
// Canonical time unit is SECONDS (§7, F3). The V5's milliseconds are converted to
// seconds exactly once, in the hal/pros adapter — ms never leak upward.
//
// Contract: now() is MONOTONIC (never decreases) within a run.

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IClock {
public:
    virtual ~IClock() = default;
    IClock() = default;
    IClock(const IClock&) = default;
    IClock(IClock&&) = default;
    IClock& operator=(const IClock&) = default;
    IClock& operator=(IClock&&) = default;

    /// Seconds elapsed since a fixed per-run epoch. Monotonic non-decreasing.
    [[nodiscard]] virtual units::Time now() const = 0;
};

}  // namespace shulib::hal
