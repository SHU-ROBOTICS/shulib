#pragma once
//
// driver_panel.hpp — THE DRIVER-CONTROL SCREEN'S SHARED PURE PIECES: the controller row a program's
// brain page shows, and the controller's own three lines with their ONE chooser (chunk COMP-b,
// 2026-09-30; the COMP brief's B11 as T4 Part 1a replaced it, and the T4 brief's D4).
//
// ── Why here ────────────────────────────────────────────────────────────────────────
// Two programs run driver control on robot two now — "shulib Teleop" and the competition program
// (COMP-b, which runs the team's driver file) — and T4 Part 3's new "shulib Drive" will be a third.
// The COMP brief's rule is that the controller screen's ONE chooser MOVES to one shared header and
// is never copied (B11): so the three lines "shulib Teleop" wrote in its panel's refreshLcd() moved
// here, WORD FOR WORD and in the same order of urgency, and Teleop now calls
// controllerScreenLines() — what its controller shows did not change (host-tested below the
// program, where it could not be before: test/driver_panel_test.cpp walks every pair of states).
// T4 Part 2 adds its PROTECTION row and its REFUSED / CUT words here, and SAFE-SAVE B2 its BACKUP
// word, each at its place in the chooser (the T4 brief's D4); nothing else in either program may
// choose a controller word.
//
// ── The chooser, in order of urgency (the controller holds 3 lines of 19 characters, HA-57) ──
//   line 0  `LIB 12V B12.9V`   the library's drive and its voltage cap, and the battery;
//   line 1  `hdg +123.4 deg`   the fused heading;
//   line 2  the FIRST that holds:
//           `AUTON ENDING`         the competition program still waits for its auton task (B2) —
//                                  the sticks do nothing yet, and the driver must know why;
//           the last fault by name (`MOTION_TIMEOUT` ...);
//           `no controller 0V`     (a disconnected controller reads 0 on every stick);
//           `DEGRADED see brain`   the boot's port check found a dead drive port;
//           `CLAW ABSENT`          a declared mechanism was not built;
//           `ok`.
// A state in force outranks a latched history, so AUTON ENDING (which ends the moment the auton
// task hands the robot back) sits above the last fault.
//
// ── How the lines are written (COMP-b's review, 2026-10-01) ─────────────────────────────────
// ONE line per write, at most one write per 0.1 s, line 2 first whenever it changed
// (ControllerLineWriter). PROS's doc says faster text updates are not applied, and both programs
// used to write every changed line in one burst — so after a burst two lines stayed stale while
// the program believed them written. Both programs write through the writer.
//
// ── The claw word, interim ──────────────────────────────────────────────────────────
// `CLAW ABSENT` is src/programs/teleop_claws.hpp's kLcdClawAbsent, with the claw and fault rows the
// competition program's page shows (clawRowText, faultRowText): they stay in that header until T4
// Part 3 moves them (the COMP brief's ND3), so this header includes it as an INTERIM step — said in
// both files.
//
// PROS-free, header-only. NOTHING HERE HAS RUN ON A ROBOT.

#include <cstddef>
#include <cstdio>
#include <string_view>

#include "programs/teleop_claws.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/ui/touch_panel.hpp"

namespace shulib::bench {

/// One controller line holds 19 characters (A4 register HA-57); a program keeps each in 24 bytes.
inline constexpr std::size_t kControllerCols = 19;
/// The storage of one controller line (Teleop's size since G1 Part 2b).
inline constexpr std::size_t kControllerLineBytes = 24;

/// What the controller's lines are chosen from — read by the program (it owns the robot), never
/// here.
struct ControllerFacts {
    double batteryVolts = 0.0;    ///< the battery (line 0)
    double headingDegrees = 0.0;  ///< the fused heading, degrees (line 1)
    bool autonEnding = false;     ///< the competition program still waits for its auton task (B2)
    bool hasFault = false;        ///< the fault latch holds a fault
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< ... and its last one
    bool connected = false;       ///< the controller is connected
    bool degraded = false;        ///< the boot's port check said DEGRADED
    bool clawAbsent = false;      ///< a declared mechanism was not built (anyClawAbsent)
};

/// The controller's three lines, each NUL-terminated within kControllerLineBytes.
struct ControllerScreen {
    char line[3][kControllerLineBytes]{};  ///< lines 0, 1, 2 (file banner)
};

/// Line 2's word — THE ONE CHOOSER (file banner), in order of urgency.
[[nodiscard]] inline const char* controllerWord(const ControllerFacts& f) noexcept {
    return f.autonEnding  ? "AUTON ENDING"
           : f.hasFault   ? diag::faultCodeName(f.lastFault)
           : !f.connected ? "no controller 0V"
           : f.degraded   ? "DEGRADED see brain"
           : f.clawAbsent ? kLcdClawAbsent.data()
                          : "ok";
}

/// The controller's three lines (file banner). Line 2 is cut at 19 characters.
[[nodiscard]] inline ControllerScreen controllerScreenLines(const ControllerFacts& f) noexcept {
    ControllerScreen s;
    std::snprintf(s.line[0], sizeof s.line[0], "LIB %2.0fV B%4.1fV", hal::kMaxMotorVoltage.value(),
                  f.batteryVolts);
    std::snprintf(s.line[1], sizeof s.line[1], "hdg %+6.1f deg", f.headingDegrees);
    std::snprintf(s.line[2], sizeof s.line[2], "%.19s", controllerWord(f));
    return s;
}

/// At most one controller line is written per this many seconds (ControllerLineWriter). PROS's own
/// doc of controller_set_text (the vendored pros/misc.h) says text updates faster than 10 ms on a
/// wired connection, or 50 ms over VEXnet, "will not be applied" -- and the adapter drops a failed
/// write without a word (hal/pros/line_display.hpp) -- so this is twice the VEXnet figure. A
/// documented rate, not a measurement on robot two.
inline constexpr double kControllerWriteGapSeconds = 0.1;

/// HOW the controller's lines are written, for every program that shows them (COMP-b's review,
/// 2026-10-01): ONE line per write, at most one write per kControllerWriteGapSeconds, line 2 (the
/// word: a fault, AUTON ENDING, no controller) first whenever it changed, the other two in turn.
/// The two programs wrote every changed line in the same tick: a burst of three, of which PROS
/// applies the first -- and the two dropped lines were remembered as written, so they stayed stale
/// until their text changed (line 2's "ok" never does). PURE; the program owns the device write.
class ControllerLineWriter {
public:
    /// The line to write NOW, or -1 (nothing changed, or the last write was too recent). A line it
    /// returns is remembered as written at `now` (seconds): the caller writes it at once.
    [[nodiscard]] int next(const ControllerScreen& want, double now) noexcept {
        if (written_ && now - lastAt_ < kControllerWriteGapSeconds - 1e-6) {  // a clock's rounding
            return -1;
        }
        const int order[3] = {2, last_ == 0 ? 1 : 0, last_ == 0 ? 0 : 1};
        for (const int row : order) {
            if (std::string_view{want.line[row]} != std::string_view{shown_.line[row]}) {
                std::snprintf(shown_.line[row], sizeof shown_.line[row], "%s", want.line[row]);
                written_ = true;
                lastAt_ = now;
                if (row != 2) {
                    last_ = row;
                }
                return row;
            }
        }
        return -1;
    }
    /// A line written OUTSIDE the writer (the competition program's AUTON ENDING): remembered, and
    /// the next write waits out the gap from it.
    void wrote(int row, const char* text, double now) noexcept {
        if (row < 0 || row > 2) {
            return;
        }
        std::snprintf(shown_.line[row], sizeof shown_.line[row], "%s", text != nullptr ? text : "");
        written_ = true;
        lastAt_ = now;
    }
    /// Forget what the controller shows (a new driver period writes every line again), keeping the
    /// time of the last write: the gap still runs from it.
    void forget() noexcept { shown_ = ControllerScreen{}; }
    /// What the writer believes the controller shows on `row` ("" before it wrote one).
    [[nodiscard]] const char* shown(int row) const noexcept {
        return row >= 0 && row <= 2 ? shown_.line[row] : "";
    }

private:
    ControllerScreen shown_{};
    bool written_ = false;
    double lastAt_ = 0.0;
    int last_ = 1;  ///< the last of lines 0 and 1 written (so line 0 goes first)
};

/// One brain-panel row: at most ui::kSmallCols characters, with the tone the program paints it in.
struct PanelRow {
    char text[ui::kSmallCols + 1]{};  ///< the row, screen-safe
    ui::Tone tone = ui::Tone::White;  ///< its colour, by meaning
};

/// The controller row a brain page shows: connected (white), or NO CONTROLLER (red) — a
/// disconnected controller reads 0 on every stick, so the drive gets 0 V (HA-103).
inline constexpr std::string_view kControllerRowOk = "controller: CONNECTED";
/// The controller row's words when it is gone.
inline constexpr std::string_view kControllerRowLost = "NO CONTROLLER -- the drive gets 0 V";
static_assert(ui::screenText(kControllerRowOk, ui::kSmallCols)
                  && ui::screenText(kControllerRowLost, ui::kSmallCols),
              "a controller row does not fit the SMALL budget");

/// The controller row (above).
[[nodiscard]] inline PanelRow controllerRowText(bool connected) noexcept {
    PanelRow r;
    const std::string_view text = connected ? kControllerRowOk : kControllerRowLost;
    ui::copyScreenText(r.text, sizeof r.text, text, ui::kSmallCols);
    r.tone = connected ? ui::Tone::White : ui::Tone::Red;
    return r;
}

}  // namespace shulib::bench
