#pragma once
//
// ControllerFaultDisplay — the D-4 controller-screen content (WS13, chunk C5).
//
// The use case (diagnostics-plan D-4): a student at the field, no laptop, robot
// stopped. The V5 controller's three-row LCD can say WHY — the latched first
// fault and a one-word state — which converts "it just stopped" into a fault
// name someone can act on. Disproportionately useful on competition day; nearly
// free to build.
//
// This class is the CONTENT side of the seam split (brief constraint 4): it
// renders rows and pushes them through hal::ILineDisplay; the PROS controller
// adapter is R1's glue. Content (each row pinned by test):
//
//     row 0:  "OK    t   12.3s"      one-word state + run clock
//             "FAULT t   12.3s"      …the word flips when anything has latched
//     row 1:  "flt none"             …or the FIRST fault: "flt ODO_STUCK"
//     row 2:  "batt 12.4V n 0"      battery + total fault count
//
// The longest fault spellings (GPS_GATE_REJECT, MOTOR_OVER_TEMP: 15 chars) fit
// row 1's 19 columns beside "flt " exactly — checked by static math here, pinned
// by test, and the seam truncates (never wraps) if a future code outgrows it.
//
// ── The write discipline (why update() diffs) ──────────────────────────────────────
// V5 controller text writes are SLOW and firmware-rate-limited (~50 ms per line
// class of slow). A per-tick repaint would both starve the display and waste the
// loop budget it shares. So update() rewrites ONLY rows whose content CHANGED —
// steady state costs three string compares and zero device writes; the battery
// row quantizes to 0.1 V so millivolt jitter cannot repaint it. The run clock
// quantizes to 0.1 s: ~10 row-0 writes/second, inside any sane device budget,
// and the seconds display is exactly why a student trusts the screen is LIVE
// (a frozen screen and a crashed program must not look identical).
//
// Reads the FaultLatch and battery it is given; owns nothing; raises nothing;
// never throws. Single-task by contract, like the rest of diag/.

#include <cstdio>
#include <cstring>

#include "shulib/diag/fault.hpp"
#include "shulib/hal/battery.hpp"
#include "shulib/hal/line_display.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

/// The three rows the V5 controller's LCD shows when a run stops: a one-word state plus the run
/// clock, the FIRST latched fault BY NAME, then battery and total fault count. Built for the
/// student standing at the field with no laptop and a robot that just stopped — it converts "it
/// died" into a fault name someone can act on. update() DIFFS: only rows whose text changed reach
/// the device, because V5 text writes are slow and firmware-rate-limited, and the clock and
/// battery quantize (0.1 s, 0.1 V) so jitter alone cannot force a repaint. The clock still ticks
/// visibly, on purpose — a frozen screen and a crashed program must not look identical.
/// Reads the latch and battery it is given; owns nothing, raises nothing, never throws.
class ControllerFaultDisplay {
public:
    /// All three must outlive the display.
    ControllerFaultDisplay(hal::ILineDisplay& display, const FaultLatch& faults,
                          const hal::IBattery& battery) noexcept
        : display_{display}, faults_{faults}, battery_{battery} {}

    /// Refresh the screen from current state; call at any convenient cadence
    /// (every loop tick is fine — unchanged rows cost no device writes).
    void update(units::Time now) {
        char buf[kBuf];
        // row 0 — one-word state + run clock (0.1 s quantum; header note).
        std::snprintf(buf, sizeof buf, "%-5s t %7.1fs", faults_.hasFault() ? "FAULT" : "OK",
                      quantize(now.value(), 0.1));
        setIfChanged(0, buf);
        // row 1 — the FIRST fault (the root cause), by name.
        if (faults_.hasFault()) {
            std::snprintf(buf, sizeof buf, "flt %s", faultCodeName(faults_.firstFault()));
        } else {
            std::snprintf(buf, sizeof buf, "flt none");
        }
        setIfChanged(1, buf);
        // row 2 — battery (0.1 V quantum) + total fault count.
        std::snprintf(buf, sizeof buf, "batt %4.1fV n %d",
                      quantize(battery_.voltage().value(), 0.1), faults_.faultCount());
        setIfChanged(2, buf);
    }

private:
    static constexpr std::size_t kBuf = 40;  ///< pre-truncation scratch (seam truncates at kCols)

    [[nodiscard]] static double quantize(double v, double quantum) noexcept {
        // Truncate toward zero: 12.49 V displays 12.4 stably rather than
        // flickering between 12.4/12.5 at the rounding boundary.
        return static_cast<double>(static_cast<long long>(v / quantum))
               * quantum;
    }

    void setIfChanged(int row, const char* text) {
        char* cache = cache_[static_cast<std::size_t>(row)];
        if (std::strncmp(cache, text, kBuf) == 0) {
            return;  // steady state: no device write (the discipline under test)
        }
        std::snprintf(cache, kBuf, "%s", text);
        display_.setLine(row, text);
    }

    hal::ILineDisplay& display_;
    const FaultLatch& faults_;
    const hal::IBattery& battery_;
    char cache_[static_cast<std::size_t>(hal::ILineDisplay::kRows)][kBuf] = {{0}, {0}, {0}};
};

}  // namespace shulib::diag
