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

/// The single source of "now" for the whole stack, in SECONDS (F3). PID dt, motion profiles, the
/// motion watchdog and settling all read time through this one seam, which is what makes every
/// timed behaviour reproducible on the host instead of dependent on how fast the machine ran. The
/// V5 clock — whose milliseconds are converted to seconds exactly once, in the PROS adapter — and
/// the deterministic test clock are just two implementations of it.
class IClock {
public:
    /// The seam is READ-ONLY by construction: now() is its only member, so nothing that holds an
    /// IClock& can reset it, set it, or sleep on it. Time moves only through an implementation's
    /// OWN type — a host test holds the concrete FakeClock, calls advance() on that, and passes
    /// the reference down — so "what moved time" has exactly one answer per run. The virtual
    /// destructor makes destruction through an IClock* well-defined; the defaulted copy/move set
    /// restores the move operations that declaring a destructor suppresses, so a concrete clock
    /// stays movable. The interface is abstract and stateless — there is no IClock value to copy.
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
