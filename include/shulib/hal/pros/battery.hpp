#pragma once
//
// ProsBattery — IBattery over the pros::battery namespace (chunk R1a): the
// brownout-compensation input behind the HAL.
//
// BINDS: battery::get_voltage(), battery::get_current(), battery::get_capacity().
//
// ═══ THE UNIT BELIEF HERE IS THE WEAKEST IN R1a — read this before trusting
// a number ═══════════════════════════════════════════════════════════════════
// The vendored misc.h documents get_voltage()/get_current() as int32 with NO
// UNIT AT ALL ("the current voltage of the battery", misc.h:718-750). The
// mV / mA belief below comes from PROS's WEBSITE, not from the source this
// tree vendors — a strictly weaker provenance than every other conversion in
// this chunk, and it is registered as such (HA-99). The stakes: a 1000× error
// here silently destroys brownout compensation, which BOUNDS every DRIVE
// command and scales nothing — control::compensateForBattery() clamps each
// wheel's desired volts to ±the measured pack, in the per-wheel drive pipeline
// only. Mechanism motors never reach it: their volts are bounded by IMotor's
// fixed ±12 V clamp alone. So the bench runbook's battery step runs before any
// driving step, and checks the raw integer is ~12600, not ~12.6.
//
// CONVERTS: mV→V and mA→A (÷1000, once, here); capacity percent→[0,1]
// (÷100, HA-100), clamped so a device quirk can never hand the core 1.02.
//
// SENTINELS (T7): IBattery has no validity channel. PROS_ERR / PROS_ERR_F
// reads hold the last good value — never zero: a 0 V battery reading would
// read as the deepest possible brownout and floor every motor command.
// Defaults before any good read are a healthy fresh pack (12.6 V, HA-46) for
// the same reason. faultedReads() exposes the screen count.
//
// HA register: HA-99, HA-100, HA-46.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/error.h"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "shulib/hal/battery.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// IBattery over the pros::battery free functions: millivolts and milliamps divided by 1000
/// here, once, and capacity percent scaled to [0, 1] and clamped so a device quirk cannot hand
/// the core a 1.02.
///
/// READ THE HEADER BANNER BEFORE TRUSTING A NUMBER FROM THIS CLASS. The mV/mA belief is the
/// weakest conversion provenance in the PROS adapter set — it comes from PROS's website, while
/// the misc.h this tree vendors documents these as int32 with no unit at all. A 1000x error
/// would silently destroy brownout compensation, which bounds every DRIVE command — the ±battery
/// clamp lives in the per-wheel drive pipeline, and mechanism motors bypass it entirely, keeping
/// only IMotor's fixed ±12 V clamp. So the bench procedure checks the raw integer reads ~12600
/// rather than ~12.6 before anything drives.
///
/// IBattery has no validity channel, so a sentinel read HOLDS the last good value and is
/// counted in faultedReads() — never zero, because 0 V would read as the deepest possible
/// brownout and floor every command. Pre-first-read defaults are a healthy fresh pack for the
/// same reason: this class is never silent, but it is also never alarming by accident.
class ProsBattery final : public IBattery {
public:
    /// Canonical volts (mV ÷ 1000 — HA-99, the website-only belief).
    [[nodiscard]] units::Voltage voltage() const override {
        const std::int32_t mv = ::pros::battery::get_voltage();
        if (mv == PROS_ERR) {
            faultedReads_ += 1;
            return lastVoltage_;
        }
        lastVoltage_ = units::Voltage{static_cast<double>(mv) / 1000.0};
        return lastVoltage_;
    }

    /// Canonical amperes (mA ÷ 1000 — HA-99).
    [[nodiscard]] units::Current current() const override {
        const std::int32_t ma = ::pros::battery::get_current();
        if (ma == PROS_ERR) {
            faultedReads_ += 1;
            return lastCurrent_;
        }
        lastCurrent_ = units::Current{static_cast<double>(ma) / 1000.0};
        return lastCurrent_;
    }

    /// [0, 1] (percent ÷ 100 — HA-100), clamped to the contract's range.
    [[nodiscard]] double capacity() const override {
        const double percent = ::pros::battery::get_capacity();
        if (!std::isfinite(percent)) {
            faultedReads_ += 1;
            return lastCapacity_;
        }
        lastCapacity_ = std::clamp(percent / 100.0, 0.0, 1.0);
        return lastCapacity_;
    }

    /// How many reads were screened to last-good (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    // `mutable`: const readers + T7 hold-last-good (state of the read).
    // Pre-first-read defaults are a healthy pack (header: why never zero).
    mutable units::Voltage lastVoltage_{12.6};  // HA-46 fresh-pack nominal
    mutable units::Current lastCurrent_{0.0};
    mutable double lastCapacity_ = 1.0;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
