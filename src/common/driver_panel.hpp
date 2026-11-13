#pragma once
//
// driver_panel.hpp — THE DRIVER-CONTROL SCREEN'S SHARED PURE PIECES: every word a driver-control
// program on robot two shows or prints about the drive and the mechanisms — the PROTECTION row, the
// controller row, the CLAW row, the ONE fault row, the mechanism boot lines — and the controller's own
// three lines with their ONE chooser and the writer that sends them (chunk COMP-b, 2026-09-30, the
// COMP brief's B11 as T4 Part 1a replaced it; T4 Part 3, 2026-10-04: the claw words moved in, and
// Part 2b's PROTECTION row and chooser words were added — the T4 brief's D4; the mechanism core's
// MECH-A5, 2026-10-29: the claw boot lines became the mechanism boot lines, read from the robot's
// list, its built roles and its mechanism service, and the claw row reads the service).
//
// ── Why here ────────────────────────────────────────────────────────────────────────
// Two programs hand robot two to a driver: "shulib Drive" (src/programs/drive_program.cpp, slot
// 1) and the competition program's opcontrol() (src/programs/competition_program.cpp, slot 6).
// Both run the team's driver file (src/team/robot_two_driver.hpp), and the rule for a piece two
// programs need is that it lives in ONE shared place and is never copied (the COMP brief's B11).
// So everything either program says about the drive and the claws is made HERE, by pure functions
// from plain facts the program reads — and the host suite pins every word (test/driver_panel_test.cpp
// for the chooser, the writer and the PROTECTION row; test/driver_panel_claws_test.cpp for the
// mechanism words, through the REAL list, the REAL driver file and the PROS shim). A program only
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
// A mechanism the provider refused (robot::Robot left it ABSENT with the provider's sentence) is a
// mechanism-service slot that answers ABSENT: its buttons do nothing, its role's boot line says
// ABSENT with that sentence (and its control lines say so), the claw row says ABSENT, the controller
// says `CLAW ABSENT` when a line has room — and the drive runs.
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
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/diag/fault.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/telemetry_sink.hpp"
#include "shulib/mechanism/driver_inputs.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/service.hpp"
#include "shulib/mechanism/wiring.hpp"
#include "shulib/robot/robot.hpp"
#include "shulib/runner/command_registry.hpp"
#include "shulib/teleop/drive_protection.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"
#include "shulib/teleop/mechanism_binding.hpp"
#include "shulib/ui/touch_panel.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::bench {

// ═══ THE MECHANISM BOOT LINES (the mechanism core's MECH-A5, 2026-10-29) ═════════════════════════
// Every program that boots robot two ("shulib Drive", the Auton Tester, the competition program)
// prints, AFTER its mechanism service is bound, one block per mechanism in the list's order — each
// line ASCII, at most kMechanismLineBytes - 1 = 199 characters (one TermSink line: no line is ever
// cut), logged with the tag MECH:
//   1. WHAT IT IS — its kind, its cartridge and its motors' directions (or a pneumatic's positions and
//      their levels), and what it does when let go or made safe:
//        `claw pivot 6: POWER -- RED, motor 1 forward; let go: HOLD`
//        `claw: PNEUMATIC -- starts 'closed', safe 'closed'; closed = A low, open = A high (levels
//         UNVERIFIED until a bench station sees them); a disable leaves it where it is`
//      (a line that would run past 199 characters — 23-character names, three 15-character
//      positions on two lines — is CUT there, never overflowed);
//   2. ONE PER ROLE, in the library's words (mechanism::roleLine over robot::MechanismParts' facts):
//        `claw pivot 6: motor 1 -> port 6 ok`, `claw: line 1 -> 3-wire A not checkable (...)`, or
//        `claw pivot 7: motor 1 -> port 7 ABSENT: port 7: nothing plugged in (expected a motor)` (Warn);
//   3. ONE PER DRIVER CONTROL naming it (the driver file's — a program with none, the Auton Tester,
//      prints none): `claw pivot 6: hold-to-run R1 / R2, 12 V`, `claw: toggle on A ('closed' /
//      'open')`, `arm: preset 'score' on RIGHT (1.50 s)`; on an ABSENT mechanism the line says its
//      buttons do nothing and the drive still runs (Warn);
//   4. a POSITION mechanism's homed state (`lift: NOT HOMED -- its buttons do nothing until it is
//      homed (homeAtDriverStart)`, the MECH brief's U5); a POWER mechanism with no stall signature
//      (`'claw pivot 6': no stall signature -- held against its stop it is cut only by the driver
//      letting go (HA-154)`, N10); a position or speed mechanism's starting values
//      (mechanism::startingValuesLine, N7 — HA-182).
// (Until MECH-A5 two lines per robot-two claw said most of this — motorClawBootLine() /
// pneumaticClawBootLine(), from the old table entries and T4's HoldControl / ToggleControl; the
// worksheet's Stations J and K quote these lines now.)

/// One mechanism boot line's storage (199 characters and its NUL: one TermSink line, never cut).
inline constexpr std::size_t kMechanismLineBytes = 200;

/// A cartridge as a boot line names it: `RED`, `GREEN`, `BLUE` (`UNSET` for any other byte).
constexpr const char* mechanismCartridgeWord(hal::MotorCartridge c) noexcept {
    switch (c) {
        case hal::MotorCartridge::Red: return "RED";
        case hal::MotorCartridge::Green: return "GREEN";
        case hal::MotorCartridge::Blue: return "BLUE";
    }
    return "UNSET";
}

/// A declared safe brake mode as the boot line and the claw row name a stopped mechanism: `HOLD`,
/// `BRAKE`, `COAST` (`?` for any other byte).
constexpr const char* mechanismStopWord(config::SafeBrakeMode m) noexcept {
    switch (m) {
        case config::SafeBrakeMode::Hold: return "HOLD";
        case config::SafeBrakeMode::Brake: return "BRAKE";
        case config::SafeBrakeMode::Coast: return "COAST";
    }
    return "?";
}

/// A declaration's kind as its boot line names it: `POWER`, `ARM`, `LIFT`, `FLYWHEEL`, `PNEUMATIC`.
constexpr const char* mechanismKindWord(const mechanism::MechanismSpec& s) noexcept {
    switch (s.kind) {
        case mechanism::Kind::Power: return "POWER";
        case mechanism::Kind::Position: return s.position.units == mechanism::Units::Inches ? "LIFT" : "ARM";
        case mechanism::Kind::Speed: return "FLYWHEEL";
        case mechanism::Kind::Pneumatic: return "PNEUMATIC";
    }
    return "UNSET";
}

/// One mechanism boot line.
struct MechanismLine {
    hal::LogLevel level = hal::LogLevel::Info;  ///< Warn for an ABSENT role or an ABSENT control
    char text[kMechanismLineBytes]{};           ///< NUL-terminated, at most 199 characters
};

namespace detail {
/// Append `text` to `out` at `n`, bounded (never past kMechanismLineBytes - 1).
inline void appendTo(char (&out)[kMechanismLineBytes], std::size_t& n, const char* text) noexcept {
    for (std::size_t i = 0; text[i] != '\0' && n + 1 < kMechanismLineBytes; ++i) {
        out[n++] = text[i];
    }
    out[n] = '\0';
}
/// A position's levels as `A low` / `C high D low` (the wiring's letters, line 1 first).
inline void appendLevels(char (&out)[kMechanismLineBytes], std::size_t& n, const mechanism::MechanismSpec& s,
                         const mechanism::MechanismWiring& w, std::size_t p) noexcept {
    for (std::size_t l = 0; l < s.lineCount && l < mechanism::kMaxLineRoles; ++l) {
        char one[16];
        std::snprintf(one, sizeof one, "%s%c %s", l == 0 ? "" : " ", mechanism::lineLetter(w.lines[l]),
                      s.pneumatic.positions[p].levels[l] ? "high" : "low");
        appendTo(out, n, one);
    }
}
}  // namespace detail

/// Line 1 for mechanism `i` of `list` (block header): WHAT IT IS.
inline MechanismLine mechanismDeclarationLine(const MechanismTable& list, std::size_t i) {
    MechanismLine out;
    const mechanism::MechanismSpec& s = list.specs[i];
    std::size_t n = 0;
    char head[64];
    std::snprintf(head, sizeof head, "%.23s: %s -- ", s.name, mechanismKindWord(s));
    detail::appendTo(out.text, n, head);
    if (s.kind != mechanism::Kind::Pneumatic) {
        detail::appendTo(out.text, n, mechanismCartridgeWord(s.cartridge));
        for (std::size_t m = 0; m < s.motorCount && m < mechanism::kMaxMotorRoles; ++m) {
            char one[24];
            std::snprintf(one, sizeof one, ", motor %d %s", static_cast<int>(m + 1),
                          s.motors[m].reversed ? "reversed" : "forward");
            detail::appendTo(out.text, n, one);
        }
        detail::appendTo(out.text, n, s.kind == mechanism::Kind::Power ? "; let go: " : "; made safe: ");
        detail::appendTo(out.text, n, mechanismStopWord(s.safeMode));
        return out;
    }
    const mechanism::PneumaticSpec& q = s.pneumatic;
    const mechanism::MechanismWiring& w = list.wiring[i];
    char start[56];  // "starts '" 8 + 15 + "', safe '" 9 + 15 + "';" 2 = 49, and the NUL
    std::snprintf(start, sizeof start, "starts '%.15s', safe '%.15s';", q.positions[q.start].name,
                  q.positions[q.safe].name);
    detail::appendTo(out.text, n, start);
    for (std::size_t p = 0; p < q.positionCount && p < mechanism::kMaxPneumaticPositions; ++p) {
        char name[24];
        std::snprintf(name, sizeof name, "%s%.15s = ", p == 0 ? " " : ", ", q.positions[p].name);
        detail::appendTo(out.text, n, name);
        detail::appendLevels(out.text, n, s, w, p);
    }
    detail::appendTo(out.text, n,
                     " (levels UNVERIFIED until a bench station sees them); a disable leaves it where it is");
    return out;
}

/// Line 2: role `r` of mechanism `i`, in the library's words (mechanism::roleLine) — Warn when the
/// provider refused it.
inline MechanismLine mechanismRoleLine(const MechanismTable& list, const robot::Robot& robot, std::size_t i,
                                       std::size_t r) {
    MechanismLine out;
    const mechanism::RoleFact& f = robot.mechanismParts().roles(i)[r];
    mechanism::roleLine(out.text, sizeof out.text, list.specs[i].name, f.role, f.port,
                        static_cast<mechanism::Line>(f.line), f.state, f.refusal, f.degradeOnly);
    out.level = f.state == mechanism::RoleState::Refused ? hal::LogLevel::Warn : hal::LogLevel::Info;
    return out;
}

/// Line 3: driver control `c` (it names mechanism `i` of `list`); `built` is whether the boot built
/// it. On an ABSENT mechanism the line says its buttons do nothing (Warn).
inline MechanismLine mechanismControlLine(const MechanismTable& list, std::size_t i, const mechanism::Control& c,
                                          bool built) {
    MechanismLine out;
    const mechanism::MechanismSpec& s = list.specs[i];
    const char* name = s.name;
    switch (c.kind) {
        case mechanism::ControlKind::Hold:
            std::snprintf(out.text, sizeof out.text, "%.23s: hold-to-run %s / %s, %d V", name,
                          teleop::controllerButtonWord(c.button), teleop::controllerButtonWord(c.back),
                          static_cast<int>(std::lround(c.volts)));
            break;
        case mechanism::ControlKind::Toggle:
            if (c.mechanismKind == mechanism::Kind::Pneumatic) {
                std::snprintf(out.text, sizeof out.text, "%.23s: toggle on %s ('%.15s' / '%.15s')", name,
                              teleop::controllerButtonWord(c.button), s.pneumatic.positions[0].name,
                              s.pneumatic.positions[1].name);
            } else {
                std::snprintf(out.text, sizeof out.text, "%.23s: toggle on %s (spin to '%.15s' / stop)", name,
                              teleop::controllerButtonWord(c.button), c.presetName != nullptr ? c.presetName : "");
            }
            break;
        case mechanism::ControlKind::Preset:
            if (c.mechanismKind == mechanism::Kind::Pneumatic) {
                std::snprintf(out.text, sizeof out.text, "%.23s: preset '%.15s' on %s", name,
                              s.pneumatic.positions[c.position].name, teleop::controllerButtonWord(c.button));
            } else if (c.presetName != nullptr) {
                std::snprintf(out.text, sizeof out.text, "%.23s: preset '%.15s' on %s (%.2f s)", name, c.presetName,
                              teleop::controllerButtonWord(c.button), c.bound);
            } else {
                std::snprintf(out.text, sizeof out.text, "%.23s: preset %.1f on %s (%.2f s)", name, c.value,
                              teleop::controllerButtonWord(c.button), c.bound);
            }
            break;
        case mechanism::ControlKind::HomeOnPress:
            std::snprintf(out.text, sizeof out.text, "%.23s: home on %s (%.2f s)", name,
                          teleop::controllerButtonWord(c.button), c.bound);
            break;
        case mechanism::ControlKind::HomeAtDriverStart:
            std::snprintf(out.text, sizeof out.text,
                          "%.23s: homed when driver control starts, if nothing has homed it (%.2f s)", name, c.bound);
            break;
        case mechanism::ControlKind::AutonOnly:
            std::snprintf(out.text, sizeof out.text, "%.23s: no driver control (autonOnly: the autons' alone)", name);
            break;
    }
    if (!built && c.kind != mechanism::ControlKind::AutonOnly) {
        std::size_t n = std::strlen(out.text);
        detail::appendTo(out.text, n, " -- ABSENT: does nothing; the drive still runs");
        out.level = hal::LogLevel::Warn;
    }
    return out;
}

/// Lines 4 for mechanism `i` (zero to two of them): a position mechanism's homed state, a power
/// mechanism with no stall signature, a position or speed mechanism's starting values. `homed` is the
/// service's word for a position mechanism (PositionSlot::homed()); ignored otherwise.
template <class Emit>
void mechanismStateLines(const MechanismTable& list, std::size_t i, bool built, bool homed, Emit&& emit) {
    const mechanism::MechanismSpec& s = list.specs[i];
    MechanismLine out;
    if (s.kind == mechanism::Kind::Position && built) {
        if (homed) {
            std::snprintf(out.text, sizeof out.text, "%.23s: homed at boot", s.name);
        } else {
            std::snprintf(out.text, sizeof out.text,
                          "%.23s: NOT HOMED -- its buttons do nothing until it is homed (homeAtDriverStart, "
                          "homeOnPress, or an auton's r.home)",
                          s.name);
            out.level = hal::LogLevel::Warn;
        }
        emit(out);
        out = MechanismLine{};
    }
    if (s.kind == mechanism::Kind::Power && !s.power.stall.stated()) {
        std::snprintf(out.text, sizeof out.text,
                      "'%.23s': no stall signature -- held against its stop it is cut only by the driver letting "
                      "go (HA-154)",
                      s.name);
        emit(out);
        out = MechanismLine{};
    }
    if (s.kind == mechanism::Kind::Position || s.kind == mechanism::Kind::Speed) {
        mechanism::startingValuesLine(out.text, sizeof out.text, s);
        emit(out);
    }
}

/// EVERY mechanism boot line of `list` (the section banner), in order, handed to `emit(const
/// MechanismLine&)` one at a time. `robot` was built from `list` (config index i IS list entry i);
/// `service` is bound over it; `controls` is the driver file's (nullptr: a program with no driver
/// control prints no control line).
template <class Emit>
void mechanismBootLines(const MechanismTable& list, const robot::Robot& robot,
                        const mechanism::MechanismService& service, const mechanism::DriverControls* controls,
                        Emit&& emit) {
    for (std::size_t i = 0; i < list.count && i < robot.mechanismCount(); ++i) {
        const bool built = robot.builtMechanismAt(i) != nullptr;
        emit(mechanismDeclarationLine(list, i));
        const std::size_t roles = robot.mechanismParts().roles(i).size();
        for (std::size_t r = 0; r < roles; ++r) {
            emit(mechanismRoleLine(list, robot, i, r));
        }
        if (controls != nullptr) {
            for (const mechanism::Control& c : controls->all()) {
                if (list.indexOf(c.mechanism) == static_cast<int>(i)) {
                    emit(mechanismControlLine(list, i, c, built));
                }
            }
        }
        const mechanism::Slot* slot = service.slotPtr(i);
        const bool homed = slot != nullptr && slot->kind() == mechanism::Kind::Position &&
                           static_cast<const mechanism::PositionSlot*>(slot)->homed();
        mechanismStateLines(list, i, built, homed, emit);
    }
}

/// The ROUTES boot line's storage (one TermSink line).
struct RoutesLine {
    char text[kMechanismLineBytes]{};  ///< NUL-terminated, at most 199 characters
};

/// THE ROUTES BOOT LINE (the two auton programs; the team lead's ruling of 2026-10-08 — every
/// mechanism publishes its actions as route ids): how many mechanism route ids the robot's list
/// derived and registered (runner::MechanismRoutes::ids()), the first and the last by name, and how
/// many of the route registry's slots are left for the team's own ids:
///     `ROUTES: 6 mechanism route ids published (claw_pivot_6_run .. claw_open); 26 of the registry's
///      32 left for the team's own`
/// and, for a robot that declares no mechanism, `ROUTES: no mechanism route ids (this robot declares no
/// mechanism); 32 of the registry's 32 left for the team's own`.
inline RoutesLine routesBootLine(const mechanism::RouteIds& ids) {
    RoutesLine out;
    const std::size_t n = ids.count < mechanism::kMaxRouteIds ? ids.count : mechanism::kMaxRouteIds;
    const int left = static_cast<int>(runner::kMaxCommands) - static_cast<int>(n);
    if (n == 0) {
        std::snprintf(out.text, sizeof out.text,
                      "ROUTES: no mechanism route ids (this robot declares no mechanism); %d of the registry's %d "
                      "left for the team's own",
                      left, static_cast<int>(runner::kMaxCommands));
    } else if (n == 1) {
        std::snprintf(out.text, sizeof out.text,
                      "ROUTES: 1 mechanism route id published (%.23s); %d of the registry's %d left for the team's "
                      "own",
                      ids.ids[0].id, left, static_cast<int>(runner::kMaxCommands));
    } else {
        std::snprintf(out.text, sizeof out.text,
                      "ROUTES: %d mechanism route ids published (%.23s .. %.23s); %d of the registry's %d left for "
                      "the team's own",
                      static_cast<int>(n), ids.ids[0].id, ids.ids[n - 1].id, left,
                      static_cast<int>(runner::kMaxCommands));
    }
    return out;
}

// ═══ THE PANEL'S CLAW ROW ═══════════════════════════════════════════════════════════
// `CLAW 6 HOLD 34C | 7 FWD 35C | CLAW CLOSED` — each MOTOR mechanism by its first motor's port: the
// word is what its motors were last COMMANDED (FWD above 0 V, REV below, else its declared stop —
// HOLD — so a step or the guard driving it shows too), then its temperature in whole degrees C;
// ABSENT when it was not built. Then each PNEUMATIC mechanism by its name in capitals and the position
// its lines are at, from the MECHANISM SERVICE (PneumaticSlot::commandedPosition() — the declaration's
// own levels; CLOSED / OPEN for robot two's claw), `?` when they match none, or ABSENT. ` FREED` at the
// end while a claim another owner left was released at THIS driver-control entry (the driver file's
// releasedLeftoverAtEntry(); the serial `[WARN][MECH]` line names which). The panel paints it amber
// then. (Since MECH-A5; until then the row read the old table's polarity constant for the claw.)

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

/// The row when the list declares nothing (every build but ROBOT=tank).
inline constexpr std::string_view kNoClawsRow = "claws: none declared for this robot";

/// The row's widest possible text for `list`, computed from the words it can print — the budget the
/// panel's static_assert holds against calibration::kSmallCols (constexpr).
constexpr std::size_t clawRowWorstLength(const MechanismTable& list) noexcept {
    if (list.count == 0) {
        return kNoClawsRow.size();
    }
    const auto len = [](const char* s) {
        std::size_t n = 0;
        while (s[n] != '\0') ++n;
        return n;
    };
    std::size_t total = 4;  // "CLAW"
    bool firstMotor = true;
    for (std::size_t i = 0; i < list.count && i < mechanism::kMaxMechanisms; ++i) {
        const mechanism::MechanismSpec& s = list.specs[i];
        if (s.kind == mechanism::Kind::Pneumatic) {
            std::size_t word = 6;  // ABSENT
            for (std::size_t p = 0; p < s.pneumatic.positionCount && p < mechanism::kMaxPneumaticPositions; ++p) {
                const std::size_t w = len(s.pneumatic.positions[p].name);
                word = w > word ? w : word;
            }
            total += 3 + len(s.name) + 1 + word;  // " | NAME " + its widest word
            continue;
        }
        const std::size_t digits = list.wiring[i].motors[0] >= 10 ? 2 : 1;
        std::size_t word = len(mechanismStopWord(s.safeMode));
        word = word < 3 ? 3 : word;                 // FWD / REV
        std::size_t body = word + 1 + 4;            // "<word> 999C"
        body = body < 6 ? 6 : body;                 // ABSENT
        total += (firstMotor ? 1 : 3) + digits + 1 + body;
        firstMotor = false;
    }
    return total + 6;  // " FREED"
}
static_assert(clawRowWorstLength(robot_two::kRobotTwoMechanisms) <= calibration::kSmallCols,
              "robot two's claw row could run past the SMALL budget");
static_assert(clawRowWorstLength(kMechanisms) <= calibration::kSmallCols,
              "this build's claw row could run past the SMALL budget");

/// One panel row: at most calibration::kSmallCols characters.
struct ClawRow {
    char text[calibration::kSmallCols + 1]{};
};

/// The claw row for `list` over `robot` and its bound `service` (section banner), with ` FREED` when
/// `freed` — the driver file's releasedLeftoverAtEntry(service) — so every program that runs a driver
/// file prints the same row, byte for byte.
inline ClawRow clawRowText(const MechanismTable& list, const robot::Robot& robot,
                           const mechanism::MechanismService& service, bool freed) {
    ClawRow out;
    if (list.count == 0) {
        std::snprintf(out.text, sizeof out.text, "%.50s", kNoClawsRow.data());
        return out;
    }
    char row[128] = "CLAW";
    std::size_t n = 4;
    bool firstMotor = true;
    for (std::size_t i = 0; i < list.count && i < robot.mechanismCount() && n < sizeof row; ++i) {
        const mechanism::MechanismSpec& s = list.specs[i];
        if (s.kind == mechanism::Kind::Pneumatic) {
            continue;  // the pneumatics after every motor mechanism (below)
        }
        const char* sep = firstMotor ? " " : " | ";
        firstMotor = false;
        const int port = list.wiring[i].motors[0];
        const hal::MotorMechanism* m = robot.mechanismAt(i);
        if (m == nullptr) {
            n += static_cast<std::size_t>(std::snprintf(row + n, sizeof row - n, "%s%d ABSENT", sep, port));
            continue;
        }
        const double v = m->commandedVoltage().value();
        const char* word = v > 0.0 ? "FWD" : (v < 0.0 ? "REV" : mechanismStopWord(s.safeMode));
        char temp[8];
        clawTemperatureText(m->motors().front()->temperature(), temp);
        n += static_cast<std::size_t>(std::snprintf(row + n, sizeof row - n, "%s%d %.5s %.4sC", sep, port, word, temp));
    }
    for (std::size_t i = 0; i < list.count && i < robot.mechanismCount() && n < sizeof row; ++i) {
        const mechanism::MechanismSpec& s = list.specs[i];
        if (s.kind != mechanism::Kind::Pneumatic) {
            continue;
        }
        char upper[24] = "";
        std::size_t k = 0;
        for (; s.name[k] != '\0' && k + 1 < sizeof upper; ++k) {
            const char c = s.name[k];
            upper[k] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
        }
        upper[k] = '\0';
        char word[16] = "ABSENT";
        const mechanism::Slot* slot = service.slotPtr(i);
        if (slot != nullptr && slot->built()) {
            const std::uint8_t p = static_cast<const mechanism::PneumaticSlot*>(slot)->commandedPosition();
            if (p < s.pneumatic.positionCount) {
                std::size_t w = 0;
                for (; s.pneumatic.positions[p].name[w] != '\0' && w + 1 < sizeof word; ++w) {
                    const char c = s.pneumatic.positions[p].name[w];
                    word[w] = (c >= 'a' && c <= 'z') ? static_cast<char>(c - 'a' + 'A') : c;
                }
                word[w] = '\0';
            } else {
                std::snprintf(word, sizeof word, "?");
            }
        }
        n += static_cast<std::size_t>(std::snprintf(row + n, sizeof row - n, " | %.23s %.15s", upper, word));
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
