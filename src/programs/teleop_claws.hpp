#pragma once
//
// teleop_claws.hpp — robot two's CLAWS in "shulib Teleop" (T3 Part 2, 2026-09-26): the driver's
// bindings built from src/robots/mechanism_table.hpp over the robot::Robot the boot built, and every word
// the program says about them — the boot lines, the banner's claw sentence, the panel's claw row,
// the controller's absent word — plus the MERGED fault row that made room for the claw row (the
// team lead's ruling R3: "the panel is full — merge the two fault rows and give the freed row to
// the claws"). Since T4 Part 1a (2026-09-27) it also carries the INTERIM TIE between these
// bindings' buttons and robot two's driver file (controlsMatchTable, below), and the claw row
// takes its ` FREED` as a bool, so a program binding through the driver file prints the same row;
// T4 Part 1b moves the buttons into the driver file alone and retires TeleopClaws.
//
// ── Why its own file, PROS-free ─────────────────────────────────────────────────────
// src/programs/teleop_program.cpp cannot be compiled on the host (it paints through pros/screen.h, which
// the host PROS shim does not have). Everything here is logic or text the worksheet quotes WORD
// FOR WORD (Station J), so it lives where the host suite can run it: test/teleop_claws_test.cpp
// drives the REAL table through hal::pros::ProsDeviceProvider and the PROS shim, and pins every
// sentence. teleop_program.cpp only calls it and prints what it returns.
//
// ── The rulings it carries (the team lead, 2026-09-26) ──────────────────────────────
//   * R1/R2 drive port 6 and L1/L2 port 7 (the table's buttons), full power (the table's 12 V),
//     HOLD on release (the table's safe mode); both buttons of one pivot held => hold (the
//     binding's rule R2);
//   * A toggles the pneumatic claw, which starts CLOSED — the toggle's initial level is
//     pneumaticStartCommand(entry), the SAME function that gives the config its pneumaticStart
//     (src/common/boot_source.hpp), so the level the builder drove the line to at boot and the level the
//     toggle sends at the first driver-control entry are one value. THE RULE (corrected at T4 Part
//     1a, 2026-09-27; it said "the claw cannot move when driver control starts" until then): the
//     FIRST driver-control entry in a run SENDS the start level; nothing moves unless something
//     commanded the line in between; after an autonomous that left the claw OPEN, the entry closes
//     it — the team lead's ruling (2026-09-26): "starts closed — whatever autonomous left it at".
//     A disable, and every later entry, leave it where it is (the library's toggle);
//   * the re-zero button is Y (src/programs/teleop_program.cpp's kRezeroButton, tied to the table's
//     kTeleopRezeroButton by a static_assert): bind() validates the whole set against it.
//
// ── An ABSENT mechanism has no binding ──────────────────────────────────────────────
// A mechanism the provider refused (robot::Robot left it ABSENT with the provider's sentence) gets
// NO binding: its buttons do nothing, the boot line says ABSENT with that sentence, the claw row
// says ABSENT, the controller says `CLAW ABSENT` when a line has room — and the drive runs.
//
// ── Storage ─────────────────────────────────────────────────────────────────────────
// A TeleopClaws lives in "shulib Teleop"'s function-local static TeleopState, never on a stack:
// PROS deletes the opcontrol task on a field disable and runs no destructor, and the loop holds
// these bindings by address. Its default constructor cannot throw (empty optionals); bind() is
// called ONCE, inside teleopInitialize()'s try — a binding's constructor may throw a precondition,
// and nothing that can throw runs after driver control starts.
//
// NO HARDWARE CLAIM: host-tested only (the PROS shim and the fake provider). No driver has pressed
// a claw button, and nothing here has run on robot two.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
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
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

/// The driver's bindings for one table's mechanisms, in static storage (header).
///
/// Non-copyable and non-movable: the bindings inside are, and the loop holds them by address.
class TeleopClaws {
public:
    TeleopClaws() = default;
    /// Non-copyable: two copies would be two state machines on one mechanism.
    TeleopClaws(const TeleopClaws&) = delete;
    /// Non-movable: the loop holds the bindings by address.
    TeleopClaws(TeleopClaws&&) = delete;
    /// Non-copy-assignable, same reasoning.
    TeleopClaws& operator=(const TeleopClaws&) = delete;
    /// Non-move-assignable, same reasoning.
    TeleopClaws& operator=(TeleopClaws&&) = delete;
    ~TeleopClaws() = default;

    /// Bind every mechanism `table` declares that `robot` BUILT — motor entry i to
    /// `robot.mechanismAt(i)` with its forward / reverse buttons and its volts, pneumatic entry j
    /// to `robot.pneumaticMechanismAt(mechanismConfigIndexOfPneumatic(table, j))` with its toggle
    /// button and `pneumaticStartCommand(entry)` as its initial level — then check the whole set
    /// against the loop's re-zero button (`teleop::validBindings`). An ABSENT mechanism gets no
    /// binding. A robot not built from this table in its order — a different mechanism count, a
    /// built mechanism of the other kind or with another name at an entry's index — is refused,
    /// by name. Returns true when the set is valid; false
    /// with the first problem in `why` (bounded, NUL-terminated), and then NO binding is left.
    /// PRECONDITIONS: `mechanismTableProblem(table) == nullptr` (every declared field is set);
    /// a binding's own constructor may throw a PreconditionError (a volts outside (0, 12]) — call
    /// this inside the boot boundary's try. Replaces whatever an earlier call bound.
    bool bind(const MechanismTable& table, robot::Robot& robot, hal::ControllerButton rezero,
              char* why, std::size_t cap) {
        unbind();
        SHULIB_PRECONDITION(mechanismTableProblem(table) == nullptr,
                            "TeleopClaws::bind: the mechanism table has an unset field");
        // The robot must have been built from THIS table, in its order (src/common/boot_source.hpp's
        // CONFIG ORDER): otherwise an index would bind a button to another mechanism, or a count
        // mismatch would read a declared mechanism as absent. Checked, never assumed.
        if (robot.mechanismCount() != motorEntryCount(table) + pneumaticEntryCount(table)) {
            return refuse(why, cap, "(the table's count)", "(the robot's count)");
        }
        for (std::size_t i = 0; i < motorEntryCount(table); ++i) {
            const MotorMechanismEntry& e = table.motors[i];
            const hal::IMechanism* built = robot.builtMechanismAt(i);
            if (built == nullptr) {
                continue;  // ABSENT: no binding, its buttons do nothing
            }
            hal::MotorMechanism* m = robot.mechanismAt(i);
            if (m == nullptr || std::strcmp(built->name(), e.name) != 0) {
                return refuse(why, cap, e.name, built->name());
            }
            hold_[i].emplace(*m, *e.forward, *e.reverse, units::Voltage{e.volts});
            holdPtrs_[holdCount_++] = &*hold_[i];
        }
        for (std::size_t j = 0; j < pneumaticEntryCount(table); ++j) {
            const PneumaticMechanismEntry& e = table.pneumatics[j];
            const std::size_t k = mechanismConfigIndexOfPneumatic(table, j);
            const hal::IMechanism* built = robot.builtMechanismAt(k);
            if (built == nullptr) {
                continue;  // ABSENT: no toggle, its button does nothing
            }
            hal::PneumaticMechanism* p = robot.pneumaticMechanismAt(k);
            if (p == nullptr || std::strcmp(built->name(), e.name) != 0) {
                return refuse(why, cap, e.name, built->name());
            }
            toggle_[j].emplace(*p, *e.toggle, pneumaticStartCommand(e));
            togglePtrs_[toggleCount_++] = &*toggle_[j];
        }
        if (!teleop::validBindings(holds(), toggles(), rezero, why, cap)) {
            unbind();
            return false;
        }
        return true;
    }

    /// Forget every binding (no mechanism is commanded). bind() starts with it; a program whose
    /// boot boundary caught a throw from bind() calls it before it tears the robot down, so no
    /// pointer into a destroyed mechanism is left behind.
    void unbind() noexcept {
        for (auto& h : hold_) h.reset();
        for (auto& t : toggle_) t.reset();
        holdCount_ = 0;
        toggleCount_ = 0;
    }

    /// Every hold binding bound (built pivots only), in table order — the loop's first span.
    [[nodiscard]] std::span<teleop::MechanismBinding* const> holds() const noexcept {
        return std::span<teleop::MechanismBinding* const>{holdPtrs_, holdCount_};
    }
    /// Every toggle bound (built pneumatic mechanisms only), in table order — the second span.
    [[nodiscard]] std::span<teleop::ToggleBinding* const> toggles() const noexcept {
        return std::span<teleop::ToggleBinding* const>{togglePtrs_, toggleCount_};
    }
    /// Motor entry `i`'s binding, or nullptr when it is ABSENT (or `i` is past the table).
    [[nodiscard]] const teleop::MechanismBinding* holdFor(std::size_t i) const noexcept {
        return i < kMaxMechanisms && hold_[i].has_value() ? &*hold_[i] : nullptr;
    }
    /// Pneumatic entry `j`'s toggle, or nullptr when it is ABSENT (or `j` is past the table).
    [[nodiscard]] const teleop::ToggleBinding* toggleFor(std::size_t j) const noexcept {
        return j < kMaxPneumatics && toggle_[j].has_value() ? &*toggle_[j] : nullptr;
    }
    /// True when the LAST loop entry released a claim an operation had left on any bound
    /// mechanism (the team lead's claim ruling; the loop logged which) — the panel's `FREED`.
    [[nodiscard]] bool anyReleasedLeftoverClaim() const noexcept {
        for (const teleop::MechanismBinding* b : holds()) {
            if (b->releasedLeftoverClaim()) return true;
        }
        for (const teleop::ToggleBinding* t : toggles()) {
            if (t->releasedLeftoverClaim()) return true;
        }
        return false;
    }

private:
    bool refuse(char* why, std::size_t cap, const char* want, const char* got) noexcept {
        unbind();
        if (why != nullptr && cap > 0) {
            std::snprintf(why, cap,
                          "the robot has '%.23s' where the table declares '%.23s' -- its config "
                          "was not converted from this table, in its order",
                          got, want);
        }
        return false;
    }

    std::optional<teleop::MechanismBinding> hold_[kMaxMechanisms];
    std::optional<teleop::ToggleBinding> toggle_[kMaxPneumatics];
    teleop::MechanismBinding* holdPtrs_[kMaxMechanisms]{};
    std::size_t holdCount_ = 0;
    teleop::ToggleBinding* togglePtrs_[kMaxPneumatics]{};
    std::size_t toggleCount_ = 0;
};

// ═══ THE INTERIM TIE: the driver file and this table, until T4 Part 1b ════════════════
// Since T4 Part 1a robot two's buttons are stated TWICE: in the driver file
// (src/team/robot_two_driver.hpp, kHolds / kToggles — the team's copy, which the competition
// program will run) and in this program's table entries (forward / reverse / volts / toggle), which
// TeleopClaws binds. T4 Part 1b removes the table's copy and makes "shulib Teleop" run the driver
// file; until then src/programs/teleop_program.cpp static_asserts this predicate over the two, so a
// button changed in one and not the other fails the build. It retires in 1b, with TeleopClaws.

/// True when every control's buttons and volts equal those of the `t` entry of the SAME NAME: a
/// hold's forward, back and volts are its motor entry's forward, reverse and volts, and a toggle's
/// button is its pneumatic entry's toggle. A control whose name `t` does not declare (of its kind)
/// does not match. Which entries have a control at all is the driver file's own check
/// (controlsCoverTable), not this one's. Complete entries only (every table a build compiles).
constexpr bool controlsMatchTable(std::span<const HoldControl> holds,
                                  std::span<const ToggleControl> toggles,
                                  const MechanismTable& t) noexcept {
    for (const HoldControl& h : holds) {
        bool matched = false;
        for (std::size_t k = 0; k < motorEntryCount(t); ++k) {
            const MotorMechanismEntry& e = t.motors[k];
            if (controlNameIs(e.name, h.mechanism)) {
                matched = e.forward.has_value() && e.reverse.has_value() && *e.forward == h.forward
                          && *e.reverse == h.back && e.volts == h.volts.value();
            }
        }
        if (!matched) {
            return false;
        }
    }
    for (const ToggleControl& c : toggles) {
        bool matched = false;
        for (std::size_t k = 0; k < pneumaticEntryCount(t); ++k) {
            const PneumaticMechanismEntry& e = t.pneumatics[k];
            if (controlNameIs(e.name, c.mechanism)) {
                matched = e.toggle.has_value() && *e.toggle == c.button;
            }
        }
        if (!matched) {
            return false;
        }
    }
    return true;
}

// ═══ THE WORDS ══════════════════════════════════════════════════════════════════════
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

/// Motor entry `i`'s boot line: BUILT (its port, sign, cartridge, buttons, volts, stop) or ABSENT
/// with the provider's own sentence (`robot.mechanismAbsence(i)`). A complete entry of a table the
/// robot was built from (config index i). The BUILT line ends `-- in shulib Teleop and shulib
/// Competition`: both programs bind these buttons and print this line (it read `-- in shulib Teleop
/// only` until COMP-b, 2026-09-30, when the competition program began binding the claws through
/// the driver file, whose buttons are tied to this table's until T4 Part 3 -- controlsMatchTable).
inline ClawLine motorClawBootLine(const MechanismTable& t, const robot::Robot& robot,
                                  std::size_t i) {
    ClawLine out;
    const MotorMechanismEntry& e = t.motors[i];
    const char* fwd = teleop::controllerButtonWord(*e.forward);
    const char* rev = teleop::controllerButtonWord(*e.reverse);
    if (robot.builtMechanismAt(i) != nullptr) {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: BUILT -- port %d (typed %+d), %.5s, %.5s forward / %.5s reverse at "
                      "%d V, %.5s on release -- in shulib Teleop and shulib Competition",
                      e.name, absPort(e.signedPort), static_cast<int>(e.signedPort),
                      clawCartridgeWord(e.cartridge), fwd, rev,
                      static_cast<int>(std::lround(e.volts)), clawStopWord(*e.safeMode));
    } else {
        std::snprintf(out.text, sizeof out.text,
                      "%.23s: ABSENT -- %.159s -- %.5s/%.5s do nothing; the drive still runs",
                      e.name, robot.mechanismAbsence(i), fwd, rev);
    }
    return out;
}

/// Pneumatic entry `j`'s boot line: BUILT (its 3-wire port, its start position and the line level
/// that is, the polarity's status, its button, the disable rule) or ABSENT with the provider's
/// sentence. A complete entry of a table the robot was built from.
inline ClawLine pneumaticClawBootLine(const MechanismTable& t, const robot::Robot& robot,
                                      std::size_t j) {
    ClawLine out;
    const PneumaticMechanismEntry& e = t.pneumatics[j];
    const std::size_t k = mechanismConfigIndexOfPneumatic(t, j);
    const char* button = teleop::controllerButtonWord(*e.toggle);
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

/// The banner's claw sentence: which buttons move what, that the claws live in THIS program and
/// not in "shulib Drive", and the re-zero button — generated from the table, so it cannot drift.
inline ClawLine clawBannerLine(const MechanismTable& t, hal::ControllerButton rezero) {
    ClawLine out;
    if (motorEntryCount(t) + pneumaticEntryCount(t) == 0) {
        std::snprintf(out.text, sizeof out.text,
                      "no mechanisms are declared for this robot (src/robots/mechanism_table.hpp)");
        return out;
    }
    std::size_t n = static_cast<std::size_t>(std::snprintf(
        out.text, sizeof out.text,
        "THE CLAWS are in THIS program (slot 2), NOT shulib Drive (slot 1):"));
    for (std::size_t i = 0; i < motorEntryCount(t) && n < sizeof out.text; ++i) {
        const MotorMechanismEntry& e = t.motors[i];
        n += static_cast<std::size_t>(std::snprintf(
            out.text + n, sizeof out.text - n, "%s %.5s/%.5s port %d", i == 0 ? "" : ",",
            teleop::controllerButtonWord(*e.forward), teleop::controllerButtonWord(*e.reverse),
            absPort(e.signedPort)));
    }
    for (std::size_t j = 0; j < pneumaticEntryCount(t) && n < sizeof out.text; ++j) {
        const PneumaticMechanismEntry& e = t.pneumatics[j];
        n += static_cast<std::size_t>(std::snprintf(
            out.text + n, sizeof out.text - n, "%s %.5s the %.23s on 3-wire %c",
            (j == 0 && motorEntryCount(t) == 0) ? "" : ",",
            teleop::controllerButtonWord(*e.toggle), e.name, adiLetter(e.adiLine)));
    }
    if (n < sizeof out.text) {
        std::snprintf(out.text + n, sizeof out.text - n, "; re-zero %.5s",
                      teleop::controllerButtonWord(rezero));
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
/// driver-control entry is remembered by whatever holds the bindings (TeleopClaws, or a driver
/// file's DriverBindings: `anyReleasedLeftoverClaim()`). THE SHARED FORM since T4 Part 1a: the
/// row reads the robot and the table, and takes the one fact it needs from the bindings as a
/// bool, so a program that binds through a driver file (the competition program, chunk COMP-b)
/// prints the same row, byte for byte, without a TeleopClaws.
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

/// The claw row with ` FREED` read from `claws` — "shulib Teleop"'s call, unchanged since T3 Part
/// 2. A one-line forwarder to the shared form above (T4 Part 1a), kept until T4 Part 1b retires
/// TeleopClaws, so no call site and no test pin changes.
inline ClawRow clawRowText(const MechanismTable& t, const robot::Robot& robot,
                           const TeleopClaws& claws) {
    return clawRowText(t, robot, claws.anyReleasedLeftoverClaim());
}

/// True when any mechanism the robot declares was NOT built — what the controller's third line
/// says (`CLAW ABSENT`) when nothing more urgent is there.
inline bool anyClawAbsent(const robot::Robot& robot) noexcept {
    for (std::size_t i = 0; i < robot.mechanismCount(); ++i) {
        if (robot.builtMechanismAt(i) == nullptr) return true;
    }
    return false;
}

/// The controller's word for an absent claw (at most 19 characters, HA-57's line). Since COMP-b
/// (2026-09-30) the ONE controller chooser that says it is src/common/driver_panel.hpp's, which
/// includes this header for it as an INTERIM step (the COMP brief's ND3: the claw words stay here
/// until T4 Part 3 moves them) -- as the competition program includes it for its claw and fault
/// rows.
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

}  // namespace shulib::bench
