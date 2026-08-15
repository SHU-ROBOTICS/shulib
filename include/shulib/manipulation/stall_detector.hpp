#pragma once
//
// StallDetector — the jam/stall decision for motor mechanisms (chunk F1).
// The physical signature of a stalled or jammed shaft is HIGH CURRENT with the
// shaft NOT TURNING, held for long enough to rule out a start-up transient
// (spin-up draws stall-grade current for the first few ticks with the shaft
// still accelerating from rest — a persistence window is what keeps that from
// reading as a jam). motor.hpp names current() "the PRIMARY capture/stall
// signal for manipulation sensor-confirm"; this class is that sentence made
// executable.
//
// The thresholds are REQUIRED constructor parameters with NO defaults, on
// C2's waitUntil precedent (an explicit finite timeout, "no invented default
// constant"): a 5.5 W and an 11 W motor stall at different currents, a lift and
// an intake tolerate different windows, and a library default would be wrong
// for half of them silently. F3's concrete primitives choose values per
// mechanism and register them; R4 measures them.
//
// Honest limits, stated: the detector can only know what the sensors say. A
// motor whose encoder lies "spinning" under a true stall passes right through
// it — which is why the operation layer's watchdog is the backstop and needs no
// sensor honesty at all (mechanism_op.hpp), and why the F1 hostile suite
// includes exactly that lying motor.

#include <cmath>

#include "shulib/core/check.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::manipulation {

/// The three numbers that define "stalled" for ONE mechanism. There are NO defaults, deliberately:
/// a 5.5 W and an 11 W motor stall at different currents, and a lift and an intake tolerate
/// different windows, so any library-chosen value would be silently wrong for half of them.
/// Concrete mechanisms pick their own and register them; hardware measurement replaces the
/// estimates later. Validation of all three happens in `StallDetector`'s constructor, not here.
struct StallConfig {
    /// Current at or above this (amps, > 0) counts as stall-grade.
    units::Current currentAtLeast;
    /// |shaft velocity| at or below this (rad/s, >= 0) counts as not turning.
    units::AngularVelocity speedAtMost;
    /// Both conditions must hold CONTINUOUSLY for this long (seconds, >= 0;
    /// 0 = trip on the first qualifying sample — legitimate for a mechanism
    /// with no spin-up, wrong for most).
    units::Time persistence;
};

/// The jam/stall decision for a motor mechanism: HIGH CURRENT with the shaft NOT TURNING, held
/// continuously for a persistence window. That window is the whole difference between a jam and a
/// spin-up — accelerating from rest draws stall-grade current for the first few ticks with the
/// shaft still nearly stopped, and without a window every start reads as a jam.
///
/// Both signals are compared by MAGNITUDE, so direction is irrelevant: a mechanism jammed while
/// running in reverse trips exactly the same way.
///
/// STATEFUL — it remembers when the current window opened, so what update() answers depends on
/// the samples that came before it and not on this one alone. Re-feeding an IDENTICAL sample is
/// harmless: `windowStart_` moves only when a CLOSED window opens, so a duplicate read within one
/// tick returns the same answer and leaves the window where it was. What the sequence does buy is
/// the other direction — one healthy sample in the middle closes the window, and the persistence
/// count starts over from the next qualifying sample. It holds NO clock: the caller supplies
/// `now`, which is what makes it loop-rate independent and testable without one.
///
/// HONEST LIMIT, stated rather than discovered: it knows only what the sensors say. A motor whose
/// encoder reports "spinning" under a true stall passes straight through it. That is why the
/// operation layer's watchdog is the backstop — the watchdog needs no sensor honesty at all.
class StallDetector {
public:
    /// Thresholds are required and CHECKED, not clamped: `currentAtLeast` must be finite and > 0,
    /// `speedAtMost` and `persistence` finite and >= 0; anything else is a precondition failure.
    /// The config is COPIED here, so mutating the caller's StallConfig afterward changes nothing.
    /// No window is open yet — the first update() carrying the signature is what starts the clock.
    explicit StallDetector(const StallConfig& config) : cfg_{config} {
        SHULIB_PRECONDITION(std::isfinite(cfg_.currentAtLeast.value()) &&
                                cfg_.currentAtLeast.value() > 0.0,
                            "StallDetector: currentAtLeast must be finite and > 0");
        SHULIB_PRECONDITION(std::isfinite(cfg_.speedAtMost.value()) &&
                                cfg_.speedAtMost.value() >= 0.0,
                            "StallDetector: speedAtMost must be finite and >= 0");
        SHULIB_PRECONDITION(std::isfinite(cfg_.persistence.value()) &&
                                cfg_.persistence.value() >= 0.0,
                            "StallDetector: persistence must be finite and >= 0");
    }

    /// Re-arm (forget any partial window) — an operation's start() calls this.
    void reset() noexcept { windowOpen_ = false; }

    /// Feed one sample; true once the stall signature has held for the full
    /// persistence window. The window RESETS on any healthy sample — a jam that
    /// intermittently slips is a chatter problem for R4's thresholds, not a
    /// reason to remember stale evidence.
    [[nodiscard]] bool update(units::Time now, units::Current current,
                              units::AngularVelocity velocity) {
        const bool signature = std::abs(current.value()) >= cfg_.currentAtLeast.value() &&
                               std::abs(velocity.value()) <= cfg_.speedAtMost.value();
        if (!signature) {
            windowOpen_ = false;
            return false;
        }
        if (!windowOpen_) {
            windowOpen_ = true;
            windowStart_ = now;
        }
        return (now - windowStart_).value() >= cfg_.persistence.value();
    }

    /// The thresholds this detector is using — the constructor's copy, so it is the authority on
    /// what was actually accepted. Read-only: there is no setter, and re-tuning means constructing
    /// a new detector. Useful for telemetry that wants to report the threshold a trip fired at.
    [[nodiscard]] const StallConfig& config() const noexcept { return cfg_; }

private:
    StallConfig cfg_;
    bool windowOpen_ = false;
    units::Time windowStart_{0.0};
};

}  // namespace shulib::manipulation
