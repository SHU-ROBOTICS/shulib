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
// Row 1's budget for a fault name is kCols - 4 ("flt ") = 15 characters, and the
// longest spelling does NOT fit: MECHANISM_STALLED is 17, so it renders as
// "flt MECHANISM_STALL" — the seam truncates rather than wraps, which is defined
// behaviour, not a bug. This banner used to claim the longest were the two 15-char
// codes and that the fit was "checked by static math here". There was no such
// check, and the arithmetic went stale the day chunk F1 appended MECHANISM_STALLED.
// The static math now EXISTS (kNameBudget below) and asserts the property that
// actually matters: every fault name must stay DISTINGUISHABLE inside the budget,
// so a truncated row still names exactly one code. A future code that collides in
// its first 15 characters is a compile error here, at the line that renders it.
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

#include <algorithm>
#include <cstdio>
#include <string_view>
#include <cstring>

#include "shulib/diag/fault.hpp"
#include "shulib/hal/battery.hpp"
#include "shulib/hal/line_display.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::diag {

namespace detail {

/// Characters row 1 leaves for a fault name after the "flt " prefix. The static math the
/// banner promises, finally written: not "every name fits" — MECHANISM_STALLED does not,
/// and shortening a §18.4 spelling to make it fit would change the text of every TermSink
/// line and run summary that carries it — but the property the display actually needs,
/// which is that a TRUNCATED row still identifies exactly one code. Iterating the code
/// slots rather than a hand-written list is what keeps this true for a code appended
/// later: unused slots render "UNKNOWN", which is 7 characters and collides with nothing.
inline constexpr int kNameBudget = hal::ILineDisplay::kCols - 4;

/// How many enumerator values the check below scans. Not a hand-maintained count of the
/// enum — the assert underneath proves the scan REACHES PAST the last named code, so
/// appending a code beyond this bound is itself a compile error rather than a silent
/// hole. That is the D3 lesson ("a gate's exclusion list is where its holes live") applied
/// to a loop bound.
inline constexpr std::size_t kScannedCodes = 32;
static_assert(std::string_view{faultCodeName(static_cast<FaultCode>(kScannedCodes))}
                  == "UNKNOWN",
              "FaultCode now has a named enumerator at or beyond kScannedCodes, so the "
              "distinctness scan below no longer covers the whole enum. Raise it.");

constexpr bool faultNamesDistinctWithin(int budget) noexcept {
    for (std::size_t i = 0; i < kScannedCodes; ++i) {
        for (std::size_t j = i + 1; j < kScannedCodes; ++j) {
            const std::string_view a{faultCodeName(static_cast<FaultCode>(i))};
            const std::string_view b{faultCodeName(static_cast<FaultCode>(j))};
            if (a == b) {
                continue;  // both are the "UNKNOWN" filler for unused slots
            }
            const std::size_t cap = static_cast<std::size_t>(budget);
            if (a.substr(0, std::min(cap, a.size()))
                == b.substr(0, std::min(cap, b.size()))) {
                return false;
            }
        }
    }
    return true;
}
static_assert(detail::faultNamesDistinctWithin(detail::kNameBudget),
              "Two FaultCode spellings share their first kCols-4 characters, so row 1 of "
              "the controller display would truncate them to the same text and name the "
              "wrong fault. Shorten one, or widen the row.");

}  // namespace detail

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
