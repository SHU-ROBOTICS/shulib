#pragma once
//
// driver_panel.hpp — THE DRIVER-CONTROL SCREEN'S SHARED PURE PIECES: every word a driver-control
// program on robot two shows or prints about the drive and the claws — the PROTECTION row, the
// controller row, the CLAW row, the ONE fault row, the claw boot lines — and the controller's own
// three lines with their ONE chooser and the writer that sends them (chunk COMP-b, 2026-09-30, the
// COMP brief's B11 as T4 Part 1a replaced it; T4 Part 3, 2026-10-04: the claw words moved in, and
// Part 2b's PROTECTION row and chooser words were added — the T4 brief's D4).
//
// ── Why here ────────────────────────────────────────────────────────────────────────
// Two programs hand robot two to a driver: "shulib Drive" (src/programs/drive_program.cpp, slot
// 1) and the competition program's opcontrol() (src/programs/competition_program.cpp, slot 6).
// Both run the team's driver file (src/team/robot_two_driver.hpp), and the rule for a piece two
// programs need is that it lives in ONE shared place and is never copied (the COMP brief's B11).
// So everything either program says about the drive and the claws is made HERE, by pure functions
// from plain facts the program reads — and the host suite pins every word (test/driver_panel_test.cpp
// for the chooser, the writer and the PROTECTION row; test/driver_panel_claws_test.cpp for the
// claw words, through the REAL table, the REAL driver file and the PROS shim). A program only
// reads the robot, hands the facts in, and paints or prints what comes back. Nothing else in
// either program may choose a controller word. Being pure is also what lets a later robot console
// stream the same rows (the team lead's R7).
//
// History: the chooser and the controller lines were "shulib Teleop"'s refreshLcd() (moved here
// word for word at COMP-b); the claw words — the boot lines, the claw row, the merged fault row,
// `CLAW ABSENT` — were src/programs/teleop_claws.hpp's (T3 Part 2, 2026-09-26, the team lead's
// ruling R3: "the panel is full — merge the two fault rows and give the freed row to the claws"),
// and moved here when T4 Part 3 retired "shulib Teleop" (its ruling R5) and deleted that file.
//
// ── The chooser, in order of urgency (the controller holds 3 lines of 19 characters, HA-57) ──
//   line 0  `LIB 12V B12.9V`   the library's drive and its voltage cap, and the battery;
//   line 1  `dead 18 13`       every dead drive port (a boot hole or a runtime mark), when any —
//           `hdg +123.4 deg`   otherwise the fused heading;
//   line 2  the FIRST that holds:
//           `AUTON ENDING`         the competition program still waits for its auton task (B2) —
//                                  the sticks do nothing yet, and the driver must know why;
//           `REFUSED see brain`    the drive protections stopped the drive for the rest of the
//                                  run (more than two drive motors dead);
//           `CUT see brain`        a 1 s protection cut is in force (a fight, or an over-current);
//           the last fault by name (`MOTION_TIMEOUT` ...);
//           `no controller 0V`     (a disconnected controller reads 0 on every stick);
//           `DEGRADED see brain`   a drive port is dead — at boot, or marked mid-run;
//           `CLAW ABSENT`          a declared mechanism was not built;
//           `ok`.
// A state in force outranks a latched history, so AUTON ENDING, REFUSED and CUT (each ends, or
// holds, by itself) sit above the last fault: a cut is never hidden behind an old fault. (SAFE-SAVE
// B2 adds its `BACKUP see brain` after CLAW ABSENT, here and nowhere else — the T4 brief's D4.)
//
// ── How the lines are written (COMP-b's review, 2026-10-01) ─────────────────────────────────
// ONE line per write, at most one write per 0.1 s, line 2 first whenever it changed
// (ControllerLineWriter). PROS's doc says faster text updates are not applied, and both programs
// used to write every changed line in one burst — so after a burst two lines stayed stale while
// the program believed them written. Both programs write through the writer.
//
// ── An ABSENT mechanism ─────────────────────────────────────────────────────────────
// A mechanism the provider refused (robot::Robot left it ABSENT with the provider's sentence) gets
// no binding from the driver file's binder: its buttons do nothing, its boot line says ABSENT with
// that sentence, the claw row says ABSENT, the controller says `CLAW ABSENT` when a line has room —
// and the drive runs.
//
// Every sentence is ASCII, and bounded: each goes through snprintf, with a precision on every %s
// that carries a name, a reason or a state's text (a bare %s prints a literal of this file, or a
// line this file already bounded), so -Wformat-truncation (an error in src/) can see it fit.
// PROS-free, header-only. NOTHING HERE HAS RUN ON A ROBOT: host-tested only (the PROS shim and the
// fake provider); no driver has pressed a claw button or read the controller's lines.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <string_view>

#include "common/boot_source.hpp"
#include "common/driver_controls.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

// ═══ THE CLAW WORDS (moved from src/programs/teleop_claws.hpp at T4 Part 3, unchanged) ═══════════
// Every sentence below is ASCII, and bounded: each goes through snprintf with a precision on every
// %s, so -Wformat-truncation (an error in src/) can see it fit.

/// The cartridge as the boot line names it.
constexpr const char* clawCartridgeWord(Cartridge c) noexcept {
    switch (c) {
        case Cartridge::Red: return "RED";
        case Cartridge::Green: return "GREEN";
        case Cartridge::Blue: return "BLUE";
        case Cartridge::Unset: break;
    }
    return "UNSET";
}

/// A brake mode as the panel and the boot line name a stopped pivot: `HOLD`, `BRAKE`, `COAST`.
constexpr const char* clawStopWord(hal::BrakeMode m) noexcept {
    switch (m) {
        case hal::BrakeMode::Hold: return "HOLD";
        case hal::BrakeMode::Brake: return "BRAKE";
        case hal::BrakeMode::Coast: return "COAST";
    }
    return "?";
}

/// A 3-wire line 1..8 as its letter on the brain, 'A'..'H' ('?' outside).
constexpr char adiLetter(std::uint8_t line) noexcept {
    return line >= 1 && line <= kMaxAdiLine ? static_cast<char>('A' + (line - 1)) : '?';
}

/// One boot line, at most kClawLineBytes - 1 characters (logWrapped() splits it for TermSink).
inline constexpr std::size_t kClawLineBytes = 256;
struct ClawLine {
    char text[kClawLineBytes]{};
};

/// The hold control in `holds` that names `name`, or nullptr — found BY NAME, as the driver file's
/// binder finds a mechanism (src/common/driver_controls.hpp). A driver file whose controls miss an
/// entry does not compile (controlsCoverTable), so a null here is a hand-built test's.
constexpr const HoldControl* holdControlFor(std::span<const HoldControl> holds,
                                            const char* name) noexcept {
    for (const HoldControl& h : holds) {
        if (controlNameIs(h.mechanism, name)) return &h;
    }
    return nullptr;
}

/// The toggle control in `toggles` that names `name`, or nullptr (as holdControlFor()).
constexpr const ToggleControl* toggleControlFor(std::span<const ToggleControl> toggles,
                                                const char* name) noexcept {
    for (const ToggleControl& c : toggles) {
        if (controlNameIs(c.mechanism, name)) return &c;
    }
    return nullptr;
}

/// Motor entry `i`'s boot line: BUILT (its port, sign, cartridge, its control's buttons and volts,
/// its stop) or ABSENT with the provider's own sentence (`robot.mechanismAbsence(i)`). The buttons
/// and the volts come from `holds` — the driver file's controls — found BY NAME (T4 Part 3: they
/// left the table); an entry no control names prints `?` for its buttons and 0 V. A complete entry
/// of a table the robot was built from (config index i).
inline ClawLine motorClawBootLine(const MechanismTable& t, std::span<const HoldControl> holds,
                                  const robot::Robot& robot, std::size_t i) {
    ClawLine out;
    const MotorMechanismEntry& e = t.motors[i];
    const HoldControl* c = holdControlFor(holds, e.name);
    const char* fwd = c != nullptr ? teleop::controllerButtonWord(c->forward) : "?";
    const char* rev = c != nullptr ? teleop::controllerButtonWord(c->back) : "?";
    const double volts = c != nullptr ? c->volts.value() : 0.0;
    if (robot.builtMechanismAt(i) != nullptr) {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: BUILT -- port %d (typed %+d), %.5s, %.5s forward / %.5s reverse at "
                      "%d V, %.5s on release -- buttons: src/team/robot_two_driver.hpp",
                      e.name, absPort(e.signedPort), static_cast<int>(e.signedPort),
                      clawCartridgeWord(e.cartridge), fwd, rev,
                      static_cast<int>(std::lround(volts)), clawStopWord(*e.safeMode));
    } else {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: ABSENT -- %.159s -- %.5s/%.5s do nothing; the drive still runs",
                      e.name, robot.mechanismAbsence(i), fwd, rev);
    }
    return out;
}

/// Pneumatic entry `j`'s boot line: BUILT (its 3-wire port, its start position and the line level
/// that is, the polarity's status, its control's button, the disable rule) or ABSENT with the
/// provider's sentence. The button comes from `toggles` — the driver file's controls — found BY
/// NAME (`?` when no control names it). A complete entry of a table the robot was built from.
inline ClawLine pneumaticClawBootLine(const MechanismTable& t,
                                      std::span<const ToggleControl> toggles,
                                      const robot::Robot& robot, std::size_t j) {
    ClawLine out;
    const PneumaticMechanismEntry& e = t.pneumatics[j];
    const std::size_t k = mechanismConfigIndexOfPneumatic(t, j);
    const ToggleControl* c = toggleControlFor(toggles, e.name);
    const char* button = c != nullptr ? teleop::controllerButtonWord(c->button) : "?";
    if (robot.builtMechanismAt(k) != nullptr) {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: BUILT -- 3-wire port %c, %.6s at the start (line %.4s; line %.4s "
                      "opens it -- polarity UNVERIFIED, HA-159); %.5s toggles it; a disable "
                      "leaves it where it is",
                      e.name, adiLetter(e.adiLine),
                      *e.start == PneumaticPosition::Closed ? "CLOSED" : "OPEN",
                      pneumaticStartCommand(e) ? "high" : "low", *e.openCommand ? "high" : "low",
                      button);
    } else {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: ABSENT -- %.159s -- %.5s does nothing; the drive still runs", e.name,
                      robot.mechanismAbsence(k), button);
    }
    return out;
}

// ═══ THE PANEL'S CLAW ROW ═══════════════════════════════════════════════════════════
// `CLAW 6 HOLD 34C | 7 FWD 35C | CLAW CLOSED` — each pivot by its port: the word is what its motor
// was last COMMANDED (FWD above 0 V, REV below, else its declared stop — HOLD — so an operation
// driving it would show too), then its temperature in whole degrees C; ABSENT when it was not
// built. Then each pneumatic mechanism by its name in capitals: OPEN / CLOSED (its commanded line
// against its polarity) or ABSENT. ` FREED` at the end while a claim released at this driver-control
// entry is remembered (the serial `[WARN][TEL]` line names which). The panel paints it amber then.

/// The temperature as the row prints it: whole degrees, clamped to -99..999 (a width the budget
/// check below can bound); "--" when the read is not a number.
///
/// WHAT THE NUMBER IS ON THE BRAIN (review round 1, finding 6): the row reads the pivot through
/// hal::IMotor, which has no validity channel, and hal::pros::ProsMotor never returns a
/// non-finite temperature — a failed read returns the LAST GOOD value (and counts it in
/// ProsMotor::faultedReads(), which the IMotor seam does not expose), and before any successful
/// read it returns its 20 C seed. So on robot two "--" never appears: a pivot that stops answering
/// after boot shows its last temperature, frozen and unmarked, and one that never answered shows
/// 20C. Worksheet Station J (J.3.3, J.8) and A4 register HA-154 say so. Showing "--" there would
/// need the seam to carry a read's validity — a library change outside T3, flagged for the team
/// lead (T3-PROGRESS P2.11). The "--" branch stays for an IMotor that does return NaN.
inline void clawTemperatureText(double celsius, char (&out)[8]) {
    if (!std::isfinite(celsius)) {
        std::snprintf(out, sizeof out, "--");
        return;
    }
    const double clamped = celsius < -99.0 ? -99.0 : (celsius > 999.0 ? 999.0 : celsius);
    std::snprintf(out, sizeof out, "%d", static_cast<int>(std::lround(clamped)));
}

/// The row when the table declares nothing (every build but ROBOT=tank).
inline constexpr std::string_view kNoClawsRow = "claws: none declared for this robot";

/// The row's widest possible text for `t`, computed from the words it can print — the budget the
/// panel's static_assert holds against calibration::kSmallCols (constexpr).
constexpr std::size_t clawRowWorstLength(const MechanismTable& t) noexcept {
    if (motorEntryCount(t) + pneumaticEntryCount(t) == 0) {
        return kNoClawsRow.size();
    }
    const auto len = [](const char* s) {
        std::size_t n = 0;
        while (s[n] != '\0') ++n;
        return n;
    };
    std::size_t total = 4;  // "CLAW"
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        const int port = absPort(t.motors[i].signedPort);
        const std::size_t digits = port >= 10 ? 2 : 1;
        std::size_t word = len(clawStopWord(*t.motors[i].safeMode));
        word = word < 3 ? 3 : word;                 // FWD / REV
        std::size_t body = word + 1 + 4;            // "<word> 999C"
        body = body < 6 ? 6 : body;                 // ABSENT
        total += (i == 0 ? 1 : 3) + digits + 1 + body;
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(t); ++j) {
        total += 3 + len(t.pneumatics[j].name) + 1 + 6;  // " | NAME " + CLOSED / ABSENT
    }
    return total + 6;  // " FREED"
}
static_assert(clawRowWorstLength(kRobotTwoMechanisms) <= calibration::kSmallCols,
              "robot two's claw row could run past the SMALL budget");
static_assert(clawRowWorstLength(kMechanisms) <= calibration::kSmallCols,
              "this build's claw row could run past the SMALL budget");

/// One panel row: at most calibration::kSmallCols characters.
struct ClawRow {
    char text[calibration::kSmallCols + 1]{};
};

/// The claw row for `t` over `robot`, with ` FREED` when `freed` — a claim released at this
/// driver-control entry is remembered by whatever holds the bindings (a driver file's
/// DriverBindings: `anyReleasedLeftoverClaim()`). The row reads the robot and the table, and takes
/// the one fact it needs from the bindings as a bool (since T4 Part 1a), so every program that
/// binds through a driver file prints the same row, byte for byte.
inline ClawRow clawRowText(const MechanismTable& t, const robot::Robot& robot, bool freed) {
    ClawRow out;
    if (motorEntryCount(t) + pneumaticEntryCount(t) == 0) {
        std::snprintf(out.text, sizeof out.text, "%.50s", kNoClawsRow.data());
        return out;
    }
    char row[128] = "CLAW";
    std::size_t n = 4;
    for (std::size_t i = 0; i < motorEntryCount(t) && n < sizeof row; ++i) {
        const MotorMechanismEntry& e = t.motors[i];
        const char* sep = i == 0 ? " " : " | ";
        const hal::MotorMechanism* m = robot.mechanismAt(i);
        if (m == nullptr) {
            n += static_cast<std::size_t>(std::snprintf(row + n, sizeof row - n, "%s%d ABSENT", sep,
                                                        absPort(e.signedPort)));
            continue;
        }
        const double v = m->commandedVoltage().value();
        const char* word = v > 0.0 ? "FWD" : (v < 0.0 ? "REV" : clawStopWord(m->safeBrakeMode()));
        char temp[8];
        clawTemperatureText(m->motors().front()->temperature(), temp);
        n += static_cast<std::size_t>(std::snprintf(row + n, sizeof row - n, "%s%d %.5s %.4sC", sep,
                                                    absPort(e.signedPort), word, temp));
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(t) && n < sizeof row; ++j) {
        const PneumaticMechanismEntry& e = t.pneumatics[j];
        char upper[24] = "";
        std::size_t k = 0;
        for (; e.name[k] != '\0' && k + 1 < sizeof upper; ++k) {
            const char c = e.name[k];
            upper[k] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
        }
        upper[k] = '\0';
        const hal::PneumaticMechanism* p =
            robot.pneumaticMechanismAt(mechanismConfigIndexOfPneumatic(t, j));
        const char* word = p == nullptr ? "ABSENT"
                           : p->commanded() == pneumaticCommandFor(e, PneumaticPosition::Open)
                               ? "OPEN"
                               : "CLOSED";
        n += static_cast<std::size_t>(
            std::snprintf(row + n, sizeof row - n, " | %.23s %.6s", upper, word));
    }
    if (freed && n < sizeof row) {
        std::snprintf(row + n, sizeof row - n, " FREED");
    }
    std::snprintf(out.text, sizeof out.text, "%.50s", row);
    return out;
}

/// True when any mechanism the robot declares was NOT built — what the controller's third line
/// says (`CLAW ABSENT`) when nothing more urgent is there.
inline bool anyClawAbsent(const robot::Robot& robot) noexcept {
    for (std::size_t i = 0; i < robot.mechanismCount(); ++i) {
        if (robot.builtMechanismAt(i) == nullptr) return true;
    }
    return false;
}

/// The controller's word for an absent claw (at most 19 characters, HA-57's line) — the ONE
/// chooser below says it (controllerWord()).
inline constexpr std::string_view kLcdClawAbsent = "CLAW ABSENT";
static_assert(kLcdClawAbsent.size() <= 19, "the controller line holds 19 characters");

// ═══ THE MERGED FAULT ROW (ruling R3) ══════════════════════════════════════════════
/// `faults: none` — or `FAULT n=<count> first <A> last <B>` when that fits the SMALL budget, else
/// `FAULT n=<count> last <B>` (the latest is the one to act on; the serial log has every fault).
/// The fallback cannot exceed the budget: 8 + 11 (any int) + 6 + 25 (the name's precision) = 50.
inline ClawRow faultRowText(int count, diag::FaultCode first, diag::FaultCode last) {
    ClawRow out;
    if (count <= 0) {
        std::snprintf(out.text, sizeof out.text, "faults: none");
        return out;
    }
    char full[96];
    const int n = std::snprintf(full, sizeof full, "FAULT n=%d first %.24s last %.24s", count,
                                diag::faultCodeName(first), diag::faultCodeName(last));
    if (n >= 0 && static_cast<std::size_t>(n) <= calibration::kSmallCols) {
        std::snprintf(out.text, sizeof out.text, "%.50s", full);
    } else {
        std::snprintf(out.text, sizeof out.text, "FAULT n=%d last %.25s", count,
                      diag::faultCodeName(last));
    }
    return out;
}

// ═══ THE CONTROLLER'S THREE LINES (COMP-b; Part 2b's words added at T4 Part 3) ═══════════════

/// One controller line holds 19 characters (A4 register HA-57); a program keeps each in 24 bytes.
inline constexpr std::size_t kControllerCols = 19;
/// The storage of one controller line (Teleop's size since G1 Part 2b).
inline constexpr std::size_t kControllerLineBytes = 24;

/// What the controller's lines are chosen from — read by the program (it owns the robot), never
/// here.
struct ControllerFacts {
    double batteryVolts = 0.0;    ///< the battery (line 0)
    double headingDegrees = 0.0;  ///< the fused heading, degrees (line 1, while no port is dead)
    bool autonEnding = false;     ///< the competition program still waits for its auton task (B2)
    bool refused = false;         ///< the drive protections REFUSE the drive for the rest of the run
    bool cut = false;             ///< a protection cut is in force (both sides at 0 V)
    bool hasFault = false;        ///< the fault latch holds a fault
    diag::FaultCode lastFault = diag::FaultCode::None;  ///< ... and its last one
    bool connected = false;       ///< the controller is connected
    bool degraded = false;        ///< a drive port is dead: the boot's port check, or a runtime mark
    bool clawAbsent = false;      ///< a declared mechanism was not built (anyClawAbsent)
    /// Every dead drive port by label ("18 13"; the protections' DriveProtectionState::dead), or
    /// "" — line 1 says them in place of the heading. Never null.
    const char* deadPorts = "";
};

/// The drive protections' state as controller facts (the T4 brief's D4): REFUSED and CUT while in
/// force, DEGRADED while the mid-run verdict says so (a boot hole counts — the protections judge
/// from each group's present members at entry), and the dead ports for line 1. A program with a
/// four-argument perTick calls it with the state the driver loop hands it, after it filled the
/// rest; it only ever SETS a word, so the boot's own DEGRADED (`degraded`) is never cleared.
/// `s` must outlive the facts (they point at its dead list).
inline void addProtectionFacts(ControllerFacts& f, const teleop::DriveProtectionState& s) noexcept {
    f.refused = f.refused || s.mode == teleop::ProtectionMode::Refused;
    f.cut = f.cut || s.mode == teleop::ProtectionMode::Cut;
    f.degraded = f.degraded || s.verdict.verdict == teleop::DriveVerdict::DriveDegraded;
    if (s.dead[0] != '\0') {
        f.deadPorts = s.dead;
    }
}

/// The controller's three lines, each NUL-terminated within kControllerLineBytes.
struct ControllerScreen {
    char line[3][kControllerLineBytes]{};  ///< lines 0, 1, 2 (file banner)
};

/// Line 2's word — THE ONE CHOOSER (file banner), in order of urgency.
[[nodiscard]] inline const char* controllerWord(const ControllerFacts& f) noexcept {
    return f.autonEnding  ? "AUTON ENDING"
           : f.refused    ? "REFUSED see brain"
           : f.cut        ? "CUT see brain"
           : f.hasFault   ? diag::faultCodeName(f.lastFault)
           : !f.connected ? "no controller 0V"
           : f.degraded   ? "DEGRADED see brain"
           : f.clawAbsent ? kLcdClawAbsent.data()
                          : "ok";
}

/// The controller's three lines (file banner). Lines 1 and 2 are cut at 19 characters.
[[nodiscard]] inline ControllerScreen controllerScreenLines(const ControllerFacts& f) noexcept {
    ControllerScreen s;
    std::snprintf(s.line[0], sizeof s.line[0], "LIB %2.0fV B%4.1fV", hal::kMaxMotorVoltage.value(),
                  f.batteryVolts);
    if (f.deadPorts != nullptr && f.deadPorts[0] != '\0') {
        std::snprintf(s.line[1], sizeof s.line[1], "dead %.14s", f.deadPorts);
    } else {
        std::snprintf(s.line[1], sizeof s.line[1], "hdg %+6.1f deg", f.headingDegrees);
    }
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

// ═══ THE PROTECTION ROW (T4 Part 2b, built in Part 3's late step; the T4 brief's D4) ══════════
// What the library's drive protections (teleop/drive_protection.hpp, applied inside
// teleop::runDriverControl for every program) are doing to the drive, from the state the driver
// loop hands a four-argument perTick. FIRST on "shulib Drive"'s MAIN page and on the competition
// program's driver page: it is the row that explains a drive that stopped.
//   `drive OK`                                          green: every drive motor answers, no cut;
//   `drive OK -- frozen 13`                             amber: a reading froze this tick — on its
//                                                       way to dead in 250 ms, or a live train
//                                                       stopping (HA-175);
//   `drive OK -- 2 over-current warnings`               amber: over-current cuts are OFF (N1);
//   `DEGRADED dead 18 13`                               amber: drive ports dead, the stick still
//                                                       drives (and ` frozen 14` while one freezes);
//   `CUT 1 s #2: FIGHT on LEFT -- port(s) 18`           red: both sides at 0 V, re-arms by itself;
//   `REFUSED: 3 drive motors dead -- 0 V for the run`   red: until the program is started again.

/// The PROTECTION row's word while every drive motor answers and nothing is cut.
inline constexpr std::string_view kProtectionRowOk = "drive OK";
static_assert(ui::screenText(kProtectionRowOk, ui::kSmallCols) && kProtectionRowOk.size() <= 8,
              "the PROTECTION row's OK word does not fit the SMALL budget (or its %.8s)");

/// The PROTECTION row (above) for `s`. `cutSeconds` is how long a cut holds — the config's
/// cutTicks times the tick — handed in by the program from the config its loop runs with, never
/// retyped. The cut's own words are the log's (DriveProtectionState::lastCut, without its
/// "CUT: "); a row longer than the SMALL budget is cut there (the serial log and "shulib Drive"'s
/// MORE page carry the whole of it).
[[nodiscard]] inline PanelRow protectionRowText(const teleop::DriveProtectionState& s,
                                                double cutSeconds) noexcept {
    PanelRow r;
    char text[2 * teleop::kProtectionTextBytes]{};
    switch (s.mode) {
        case teleop::ProtectionMode::Refused:
            std::snprintf(text, sizeof text, "REFUSED: %d drive motors dead -- 0 V for the run",
                          s.verdict.totalDead);
            r.tone = ui::Tone::Red;
            break;
        case teleop::ProtectionMode::Cut: {
            const char* words = s.lastCut;
            if (std::strncmp(words, "CUT: ", 5) == 0) {
                words += 5;
            }
            std::snprintf(text, sizeof text, "CUT %.3g s #%d: %.90s",
                          std::isfinite(cutSeconds) ? cutSeconds : 0.0, s.cutCount, words);
            r.tone = ui::Tone::Red;
            break;
        }
        case teleop::ProtectionMode::Degraded:
            std::snprintf(text, sizeof text, "DEGRADED dead %.40s%.8s%.40s", s.dead,
                          s.frozen[0] != '\0' ? " frozen " : "", s.frozen);
            r.tone = ui::Tone::Amber;
            break;
        case teleop::ProtectionMode::Driving:
            if (s.frozen[0] != '\0') {
                std::snprintf(text, sizeof text, "%.8s -- frozen %.40s", kProtectionRowOk.data(),
                              s.frozen);
                r.tone = ui::Tone::Amber;
            } else if (s.overCurrentWarnings > 0) {
                std::snprintf(text, sizeof text, "%.8s -- %d over-current warning%.1s",
                              kProtectionRowOk.data(), s.overCurrentWarnings,
                              s.overCurrentWarnings == 1 ? "" : "s");
                r.tone = ui::Tone::Amber;
            } else {
                std::snprintf(text, sizeof text, "%.8s", kProtectionRowOk.data());
                r.tone = ui::Tone::Green;
            }
            break;
    }
    ui::copyScreenText(r.text, sizeof r.text, text, ui::kSmallCols);
    return r;
}

}  // namespace shulib::bench
