#pragma once
//
// ProsLineDisplay — ILineDisplay over the V5 controller's LCD (chunk R1a):
// where the D-4 status rows physically go.
//
// BINDS: pros::Controller::set_text(row, 0, text). The adapter owns its own
// pros::Controller handle — that class is a thin wrapper over the C API keyed
// by controller id, so a second handle on the same id (the IController
// adapter's) is two views of one device, not two devices.
//
// GEOMETRY — TRUNCATE AT kCols, NEVER WRAP (line_display.hpp:20-21): a
// wrapped status row would corrupt the row below it. kCols = 19 is HA-57,
// PROVISIONAL — and R1a found a CONFLICT while reading the vendored source:
// misc.hpp:322 documents set_text's col parameter as [0-14], which implies a
// 15-column grid, not 19. Community practice says 19 visible characters;
// the vendored doc says 15; neither is a measurement. Registered as HA-107;
// the bench runbook writes a ruler string ("0123456789ABCDEFGHIJ") and
// counts what the physical LCD shows. Until then this adapter truncates at
// ILineDisplay::kCols exactly as the seam contract states.
//
// PADDING TO kCols, deliberately: set_text writes from column 0 and leaves
// whatever was beyond the new text's end — a shorter line would show the
// tail of the previous one ("ARM OK" over "ARMED FAULT" reads "ARM OKFAULT").
// Padding with spaces makes setLine() a true OVERWRITE, which is the verb
// the seam promises ("Overwrite row i").
//
// RATE: the firmware rate-limits controller writes (misc.hpp:312-313 "text
// setting is currently in beta … continuous fast updates will not work
// well"). Pacing lives ABOVE the seam (ControllerFaultDisplay already
// repaints only changed rows — line_display.hpp header); this adapter stays
// a dumb device write, one call = one set_text.
//
// MUST NOT THROW (contract): set_text returns an error code; a failed write
// (controller unplugged) is DROPPED — a status row has no fallback channel,
// and the telemetry log already carries the same information.
//
// HA register: HA-57, HA-107.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include <array>
#include <cstdint>
#include <string_view>

#include "shulib/hal/line_display.hpp"

namespace shulib::hal::pros {

/// Which physical controller's LCD this display writes (mirrors the
/// IController adapter's id — see controller.hpp).
enum class DisplayController {
    Master,   ///< The driver's own controller (::pros::E_CONTROLLER_MASTER).
    Partner,  ///< The second controller tethered to it (::pros::E_CONTROLLER_PARTNER).
};

/// Where the status rows actually land: ILineDisplay driven by pros::Controller::set_text.
/// One setLine() is exactly one device write — the adapter does no pacing, no batching and
/// no change-detection, because the firmware's write rate limit is a CONTENT-layer problem
/// (ControllerFaultDisplay repaints only rows that changed) and a device adapter that
/// silently withheld writes would be unfalsifiable. What it does add is the two things the
/// seam promises and set_text does not give for free: truncation at ILineDisplay::kCols
/// (never a wrap, which would corrupt the row below) and space-padding out to kCols, so a
/// short line is a true overwrite instead of leaving the previous row's tail visible. A
/// failed write — controller unplugged — is dropped, never thrown; the telemetry log
/// already carries the same text. kCols itself is still unverified against real firmware
/// (see the header: the vendored SDK doc and community practice disagree, 15 vs 19).
class ProsLineDisplay final : public ILineDisplay {
public:
    /// Opens this adapter's OWN pros::Controller handle on `which` (master by default).
    /// That handle is a thin wrapper keyed by controller id, so holding one here while the
    /// IController adapter holds another on the same id is two views of ONE device, not a
    /// second device — no coordination is required and none is performed. Constructing
    /// touches no hardware and cannot fail: an absent controller shows up as dropped
    /// writes in setLine(), not as an error here.
    explicit ProsLineDisplay(DisplayController which = DisplayController::Master)
        : controller_{which == DisplayController::Master ? ::pros::E_CONTROLLER_MASTER
                                                         : ::pros::E_CONTROLLER_PARTNER} {}

    /// Overwrite `row` with `text`: truncated at kCols (NEVER wrapped), padded
    /// with spaces to kCols (a true overwrite — header). MUST NOT throw.
    void setLine(int row, std::string_view text) override {
        if (row < 0 || row >= kRows) {
            return;  // caller's precondition (line_display.hpp:27); never throw here
        }
        std::array<char, static_cast<std::size_t>(kCols) + 1> buf{};
        const std::size_t n =
            text.size() < static_cast<std::size_t>(kCols) ? text.size()
                                                          : static_cast<std::size_t>(kCols);
        for (std::size_t i = 0; i < n; ++i) {
            buf[i] = text[i];
        }
        for (std::size_t i = n; i < static_cast<std::size_t>(kCols); ++i) {
            buf[i] = ' ';
        }
        buf[static_cast<std::size_t>(kCols)] = '\0';
        controller_.set_text(static_cast<std::uint8_t>(row), 0, buf.data());
    }

private:
    mutable ::pros::v5::Controller controller_;  // set_text is non-const in PROS
};

}  // namespace shulib::hal::pros
