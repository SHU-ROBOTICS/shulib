#pragma once
//
// mechanism/wiring.hpp — THE BINDING (the mechanism core, chunk MECH-A1, 2026-10-08): which port
// each ROLE is plugged into, a block of the robot file SEPARATE from the declarations, compiled in
// (P2; the team lead's Q1 (a): the binding lives only in the robot file).
//
//     inline constexpr MechanismTable kRobotXMechanisms = mechanisms(
//         wire(arm, motors(1)),
//         wire(lift, motors(2, 3), homeSwitch('B')),
//         wire(flywheel, motors(4)),
//         wire(intake, motors(5), pieceSensor(9)),
//         wire(hood, lines('C', 'D')));
//
// One line per mechanism, in the order the boot lists them; this list IS the robot's mechanisms (a
// robot names its list `k<Robot>Mechanisms` — `MechanismTable` is the library's one type name, and
// `kMechanisms` is the BUILD's selection, never a robot's). A smart port is a NUMBER (1..21), a
// 3-wire port a LETTER in quotes ('A'..'H') — `Line` is its own type, so smart port 6 and line F
// cannot be confused (an early census probe read "claw line = 6" as 3-wire F).
//
// ── Checked AT BUILD, by name ────────────────────────────────────────────────────────
// Each wire() checks ITS mechanism: the declaration itself (spec.hpp's specRefusal), every role
// bound and nothing extra, no sensor role written twice, a port in range and of the right KIND (a
// number where a smart device goes, a letter where a 3-wire one does), the motor count (at most
// four) and the line count (one or two), and — before the spec's own checks — that every pneumatic
// position states one level per wired line (U2). mechanisms() checks the ROBOT: each entry again on
// its own (its spec, and its wiring beside it — entryProblem(), the rules wiringProblem() shares —
// so an entry built by hand is held to wire()'s rules), then no smart port or 3-wire port used
// twice — NAMING it (`smart_port_4_is_used_twice`, `three_wire_port_C_is_used_twice`) —, the same
// declaration twice, two declarations sharing a `.name`, more than eight, and every route id the
// list would publish (route_ids.hpp). Each refusal stops the build in a consteval function whose
// NAME is the reason (spec.hpp's `refused::` idiom).
// Where GCC puts it, measured (the brief's neg5 set): a refusal between entries points at the
// list's first line and names the port; a declaration's own refusal points at its wire(...) line.
//
// ── The drive check, and the run-time owner of every collision ───────────────────────
// The drive's ports and the IMU's are checked against the robot's own chassis table beside its list
// (clearOfDrive, wiringProblem — the robot table file's job, MECH-A5) and, at RUN time, by
// config::RobotConfig::valid() over the derived config (toMechanismConfig) beside the drivetrain the
// boot actually chose — a calibration record's ports included. valid() is the owner; the build-time
// checks mirror it for the typed drive, and the host suite proves the two agree.
//
// ── At every boot ───────────────────────────────────────────────────────────────────
// robot::MechanismParts asks the registry-checked device provider for every role's device (the
// provider IS the kind check, HA-141). A missing or wrong-kind ACTUATOR or ANGLE sensor makes that
// mechanism ABSENT (its controls and steps do nothing; the drive runs); a missing PIECE sensor leaves
// it running WITHOUT one. roleLine() words one boot line per role; a 3-wire role reads "not
// checkable" (the brain cannot see a 3-wire device, HA-158). Two cables of the SAME kind swapped
// between mechanisms look fine at boot — the bench page's direction check is for that (MECH-A6).
//
// PROS-free, allocation-free; the list is constexpr (flash on the brain).

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <source_location>
#include <span>
#include <string_view>

#include "shulib/config/robot_config.hpp"
#include "shulib/mechanism/devices.hpp"
#include "shulib/mechanism/kinds.hpp"
#include "shulib/mechanism/route_ids.hpp"
#include "shulib/mechanism/spec.hpp"

namespace shulib::mechanism {

/// A brain 3-wire port, A..H. None = unbound. Its own type, so a smart port number cannot be
/// written where a line belongs (header). The value is the config's line number (A = 1).
enum class Line : std::uint8_t {
    None = 0,  ///< unbound
    A = 1,     ///< 3-wire port A
    B,         ///< 3-wire port B
    C,         ///< 3-wire port C
    D,         ///< 3-wire port D
    E,         ///< 3-wire port E
    F,         ///< 3-wire port F
    G,         ///< 3-wire port G
    H,         ///< 3-wire port H
};

/// A line as its letter ('A'..'H'), '-' for None.
[[nodiscard]] constexpr char lineLetter(Line l) noexcept {
    return l == Line::None ? '-' : static_cast<char>('A' + static_cast<int>(l) - 1);
}

/// ONE mechanism's wiring: its name (checked against the spec's) and a port or line per role.
struct MechanismWiring {
    /// The spec's name, in the specs' order (checked).
    char mechanism[kNameBytes]{};
    /// Smart ports 1..21, one per motor role, in the declaration's `.motors` order (physical; the
    /// direction is the declaration's).
    std::uint8_t motors[kMaxMotorRoles]{};
    /// 3-wire lines, one per line role, in the positions' level order.
    Line lines[kMaxLineRoles]{};
    /// The angle sensor's smart port (a Rotation role); 0 = none.
    std::uint8_t angleSensor = 0;
    /// The home switch's line; None = none.
    Line homeSwitch = Line::None;
    /// A distance or optical piece sensor's smart port; 0 = none.
    std::uint8_t pieceSensorPort = 0;
    /// A limit-switch piece sensor's line; None = none.
    Line pieceSwitch = Line::None;
    /// Where this mechanism's `wire(...)` line is (captured where motors(...) / lines(...) is
    /// written): the direction check's "a cable of this mechanism is on another's port: check the
    /// wire(...) line HERE".
    const char* file = nullptr;
    /// That line.
    std::uint32_t line = 0;
};

/// ONE ROBOT's mechanisms: the declarations as data and their wiring, in the same order — the ONE
/// team-facing type for a robot's list (N11; the `src` build gate's word too).
struct MechanismTable {
    /// The mechanisms as data, in boot order.
    MechanismSpec specs[kMaxMechanisms]{};
    /// Their wiring, in the same order.
    MechanismWiring wiring[kMaxMechanisms]{};
    /// How many are real.
    std::uint8_t count = 0;
    /// The team's declarations (kinds.hpp), in the same order — what a driver control's build check
    /// compares (by address); nullptr for a hand-built table. The run-time lookup is by NAME.
    const void* declared[kMaxMechanisms]{};
    /// The index of declaration `d` in this list, or -1 when this robot does not have it. nullptr is
    /// no declaration, so it is never found — even beside hand-built entries, whose `declared` is
    /// nullptr (MECH-A1's review: two hand-built entries were taken for "the same mechanism").
    [[nodiscard]] constexpr int indexOf(const void* d) const noexcept {
        if (d == nullptr) {
            return -1;
        }
        for (std::size_t i = 0; i < count && i < kMaxMechanisms; ++i) {
            if (declared[i] == d) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }
    /// The specs as a span (bounded by the capacity).
    [[nodiscard]] constexpr std::span<const MechanismSpec> specSpan() const noexcept {
        return std::span<const MechanismSpec>{specs, count < kMaxMechanisms ? count : kMaxMechanisms};
    }
    /// The wiring as a span (bounded by the capacity).
    [[nodiscard]] constexpr std::span<const MechanismWiring> wiringSpan() const noexcept {
        return std::span<const MechanismWiring>{wiring, count < kMaxMechanisms ? count : kMaxMechanisms};
    }
};

/// What can be wrong with ONE mechanism's wiring beside its own spec, before any other mechanism or
/// the drive is looked at (header) — the per-entry rules wire() enforces as it builds an entry, as
/// data, in the order entryProblem() checks them. mechanisms() holds EVERY entry to them, a
/// hand-built one included (it never passed through wire()), and names each (refused::);
/// wiringProblem() checks a tool's table with the same function and words each
/// (entryProblemText). One rule set, two readers (MECH-A1's review: a hand-built entry with an
/// unbound motor, a sensor its spec lacks or another mechanism's name used to compile).
enum class EntryProblem : std::uint8_t {
    None = 0,              ///< nothing is wrong with the entry on its own
    NameDiffers,           ///< the wiring names another mechanism than its spec
    MotorRoleUnbound,      ///< a motor role the mechanism has is on port 0
    SmartPortPast21,       ///< a bound smart port (a motor's or a sensor's) past 21
    MotorPortExtra,        ///< a motor port on a role past the mechanism's motor count
    LineRoleUnbound,       ///< a line role the mechanism has is unbound
    LinePastH,             ///< a bound 3-wire line past H
    LineExtra,             ///< a 3-wire line on a role past the mechanism's line count
    LevelsVsLines,         ///< a pneumatic position states a different number of levels than the lines (U2)
    AngleSensorUnplugged,  ///< the angle sensor the mechanism has is unbound
    AngleSensorExtra,      ///< an angle sensor port on a mechanism with none
    HomeSwitchUnplugged,   ///< the home switch the mechanism has is unbound
    HomeSwitchExtra,       ///< a home switch line on a mechanism with none
    PieceSensorUnplugged,  ///< the distance or optical piece sensor the mechanism has is unbound
    PieceSensorExtra,      ///< a piece sensor port on a mechanism with no distance or optical one
    PieceSwitchUnplugged,  ///< the limit-switch piece sensor the mechanism has is unbound
    PieceSwitchExtra,      ///< a piece switch line on a mechanism with no switch piece sensor
};

/// THE PER-ENTRY WIRING RULES (header): the FIRST thing wrong with entry `w` beside its spec `s` —
/// its name, then its motor roles, its line roles, a pneumatic's levels against its lines, its angle
/// sensor, its home switch, its piece sensor — or EntryProblem::None. Every bound port is checked in
/// range here, so a caller may index by it afterwards. It does not check the spec (specRefusal does).
[[nodiscard]] constexpr EntryProblem entryProblem(const MechanismSpec& s, const MechanismWiring& w) noexcept {
    std::size_t n = 0;
    while (n < kNameBytes && w.mechanism[n] != '\0') {
        ++n;
    }
    if (std::string_view{w.mechanism, n} != nameOf(s)) {
        return EntryProblem::NameDiffers;
    }
    const auto smart = [](std::uint8_t p, EntryProblem unbound) {
        return p == 0 ? unbound : (p > 21 ? EntryProblem::SmartPortPast21 : EntryProblem::None);
    };
    const auto line = [](Line l, EntryProblem unbound) {
        return l == Line::None ? unbound
                               : (static_cast<std::uint8_t>(l) > 8 ? EntryProblem::LinePastH : EntryProblem::None);
    };
    for (std::size_t m = 0; m < kMaxMotorRoles; ++m) {
        if (m < s.motorCount) {
            if (const EntryProblem e = smart(w.motors[m], EntryProblem::MotorRoleUnbound); e != EntryProblem::None) {
                return e;
            }
        } else if (w.motors[m] != 0) {
            return EntryProblem::MotorPortExtra;
        }
    }
    for (std::size_t l = 0; l < kMaxLineRoles; ++l) {
        if (l < s.lineCount) {
            if (const EntryProblem e = line(w.lines[l], EntryProblem::LineRoleUnbound); e != EntryProblem::None) {
                return e;
            }
        } else if (w.lines[l] != Line::None) {
            return EntryProblem::LineExtra;
        }
    }
    if (s.kind == Kind::Pneumatic) {
        for (std::size_t p = 0; p < s.pneumatic.positionCount && p < kMaxPneumaticPositions; ++p) {
            if (s.pneumatic.positions[p].levelCount != s.lineCount) {
                return EntryProblem::LevelsVsLines;
            }
        }
    }
    if (s.angleSensor.kind != SensorKind::None) {
        if (const EntryProblem e = smart(w.angleSensor, EntryProblem::AngleSensorUnplugged); e != EntryProblem::None) {
            return e;
        }
    } else if (w.angleSensor != 0) {
        return EntryProblem::AngleSensorExtra;
    }
    if (s.homeSwitch.kind != SensorKind::None) {
        if (const EntryProblem e = line(w.homeSwitch, EntryProblem::HomeSwitchUnplugged); e != EntryProblem::None) {
            return e;
        }
    } else if (w.homeSwitch != Line::None) {
        return EntryProblem::HomeSwitchExtra;
    }
    if (s.pieceSensor.kind == SensorKind::Distance || s.pieceSensor.kind == SensorKind::Optical) {
        if (const EntryProblem e = smart(w.pieceSensorPort, EntryProblem::PieceSensorUnplugged); e != EntryProblem::None) {
            return e;
        }
    } else if (w.pieceSensorPort != 0) {
        return EntryProblem::PieceSensorExtra;
    }
    if (s.pieceSensor.kind == SensorKind::Switch) {
        if (const EntryProblem e = line(w.pieceSwitch, EntryProblem::PieceSwitchUnplugged); e != EntryProblem::None) {
            return e;
        }
    } else if (w.pieceSwitch != Line::None) {
        return EntryProblem::PieceSwitchExtra;
    }
    return EntryProblem::None;
}

/// An entry problem as wiringProblem()'s SENTENCE (nullptr for None). An unbound or out-of-range
/// role reads alike whichever role it is: "a smart-port role is unbound or not 1..21", "a 3-wire role
/// is unbound or not A..H".
[[nodiscard]] constexpr const char* entryProblemText(EntryProblem e) noexcept {
    switch (e) {
        case EntryProblem::None:
            return nullptr;
        case EntryProblem::NameDiffers:
            return "the wiring names another mechanism at this place";
        case EntryProblem::MotorRoleUnbound:
        case EntryProblem::SmartPortPast21:
        case EntryProblem::AngleSensorUnplugged:
        case EntryProblem::PieceSensorUnplugged:
            return "a smart-port role is unbound or not 1..21";
        case EntryProblem::MotorPortExtra:
            return "a motor port is bound to a role the mechanism does not have";
        case EntryProblem::LineRoleUnbound:
        case EntryProblem::LinePastH:
        case EntryProblem::HomeSwitchUnplugged:
        case EntryProblem::PieceSwitchUnplugged:
            return "a 3-wire role is unbound or not A..H";
        case EntryProblem::LineExtra:
            return "a 3-wire line is bound to a role the mechanism does not have";
        case EntryProblem::LevelsVsLines:
            return "a position states a different number of levels than the wiring has lines";
        case EntryProblem::AngleSensorExtra:
            return "an angle sensor port is bound on a mechanism with no angle sensor role";
        case EntryProblem::HomeSwitchExtra:
            return "a home switch line is bound on a mechanism with no home switch role";
        case EntryProblem::PieceSensorExtra:
            return "a piece sensor port is bound on a mechanism with no distance/optical piece sensor";
        case EntryProblem::PieceSwitchExtra:
            return "a piece switch line is bound on a mechanism with no switch piece sensor";
    }
    return "an unknown wiring problem";
}

/// THE WIRING CHECK at run time — the mirror of what wire() and mechanisms() refuse at build, for a
/// table a TOOL or a test built by hand, plus the drive's ports (header): the first problem, or
/// nullptr. Each entry is checked on its own first (entryProblem — the SAME rules mechanisms()
/// holds a hand-built entry to), then against the others (a name or a port or line used twice) and
/// the drive. `drivePorts` are the chassis table's port magnitudes, `imuPort` its IMU (0 = none). It
/// checks the wiring, not the declarations (specProblem() does those), and not the route ids
/// (routeIdsOf() does).
[[nodiscard]] constexpr const char* wiringProblem(std::span<const MechanismSpec> specs,
                                                  std::span<const MechanismWiring> wiring,
                                                  std::span<const std::uint8_t> drivePorts,
                                                  std::uint8_t imuPort) noexcept {
    if (specs.size() != wiring.size()) {
        return "the wiring does not list every mechanism, in order";
    }
    if (specs.size() > kMaxMechanisms) {
        return "more mechanisms than a robot can have";
    }
    bool smart[22]{};
    bool line[9]{};
    // entryProblem() has range-checked every bound role before these run; the bounds below keep the
    // arrays' indices safe whatever the order.
    const auto takeSmart = [&](std::uint8_t p) -> const char* {
        if (p < 1 || p > 21) {
            return "a smart-port role is unbound or not 1..21";
        }
        for (const std::uint8_t d : drivePorts) {
            if (d == p) {
                return "a mechanism role is bound to a DRIVE port";
            }
        }
        if (p == imuPort) {
            return "a mechanism role is bound to the IMU's port";
        }
        if (smart[p]) {
            return "a smart port is bound twice";
        }
        smart[p] = true;
        return nullptr;
    };
    const auto takeLine = [&](Line l) -> const char* {
        if (l == Line::None || static_cast<std::uint8_t>(l) > 8) {
            return "a 3-wire role is unbound or not A..H";
        }
        if (line[static_cast<std::uint8_t>(l)]) {
            return "a 3-wire line is bound twice";
        }
        line[static_cast<std::uint8_t>(l)] = true;
        return nullptr;
    };
    for (std::size_t i = 0; i < specs.size(); ++i) {
        const MechanismSpec& s = specs[i];
        const MechanismWiring& w = wiring[i];
        if (const char* e = entryProblemText(entryProblem(s, w))) {
            return e;
        }
        for (std::size_t j = 0; j < i; ++j) {
            if (nameOf(specs[j]) == nameOf(s)) {
                return "two mechanisms share a name";
            }
        }
        for (std::size_t m = 0; m < s.motorCount && m < kMaxMotorRoles; ++m) {
            if (const char* e = takeSmart(w.motors[m])) {
                return e;
            }
        }
        for (std::size_t l = 0; l < s.lineCount && l < kMaxLineRoles; ++l) {
            if (const char* e = takeLine(w.lines[l])) {
                return e;
            }
        }
        if (s.angleSensor.kind != SensorKind::None) {
            if (const char* e = takeSmart(w.angleSensor)) {
                return e;
            }
        }
        if (s.homeSwitch.kind != SensorKind::None) {
            if (const char* e = takeLine(w.homeSwitch)) {
                return e;
            }
        }
        if (s.pieceSensor.kind == SensorKind::Distance || s.pieceSensor.kind == SensorKind::Optical) {
            if (const char* e = takeSmart(w.pieceSensorPort)) {
                return e;
            }
        }
        if (s.pieceSensor.kind == SensorKind::Switch) {
            if (const char* e = takeLine(w.pieceSwitch)) {
                return e;
            }
        }
    }
    return nullptr;
}

/// THE ONE PROJECTION: one mechanism as the device-level config robot::Robot (and the bench page)
/// builds from — signed motor ports (a Reversed role is a NEGATIVE port, reversed once by its
/// adapter), its lines, its sensor roles' ports and lines, and the start and safe LEVELS from its
/// START and SAFE positions: one level for every line when each position is uniform across its
/// lines (today's form — robot two's claw, byte for byte), per-line levels otherwise.
/// PRECONDITION: specProblem(s) == nullptr and the wiring passed wiringProblem() (a mechanisms()
/// list always has).
[[nodiscard]] constexpr config::MechanismConfig toMechanismConfig(const MechanismSpec& s,
                                                                  const MechanismWiring& w) noexcept {
    config::MechanismConfig c{};
    for (std::size_t i = 0; i < kNameBytes; ++i) {
        c.name[i] = s.name[i];
    }
    if (s.kind != Kind::Pneumatic) {
        c.motorCount = s.motorCount;
        for (std::size_t m = 0; m < s.motorCount && m < kMaxMotorRoles; ++m) {
            const auto p = static_cast<std::int8_t>(w.motors[m]);
            c.motorPorts[m] = s.motors[m].reversed ? static_cast<std::int8_t>(-p) : p;
        }
        c.cartridge = s.cartridge;
        c.safeMode = s.safeMode;
    } else {
        c.pneumaticCount = s.lineCount;
        const PneumaticSpec& q = s.pneumatic;
        const auto level = [](bool high) { return high ? config::PneumaticLevel::High : config::PneumaticLevel::Low; };
        bool sameAcross = true;
        for (std::size_t l = 0; l < s.lineCount && l < kMaxLineRoles; ++l) {
            c.pneumaticPorts[l] = static_cast<std::uint8_t>(w.lines[l]);
            sameAcross = sameAcross && q.positions[q.start].levels[l] == q.positions[q.start].levels[0] &&
                         q.positions[q.safe].levels[l] == q.positions[q.safe].levels[0];
        }
        c.pneumaticStart = level(q.positions[q.start].levels[0]);
        c.pneumaticSafe = level(q.positions[q.safe].levels[0]);
        if (!sameAcross) {
            for (std::size_t l = 0; l < s.lineCount && l < kMaxLineRoles; ++l) {
                c.pneumaticStartByLine[l] = level(q.positions[q.start].levels[l]);
                c.pneumaticSafeByLine[l] = level(q.positions[q.safe].levels[l]);
            }
        }
    }
    if (s.angleSensor.kind == SensorKind::Rotation) {
        c.rotationPort = w.angleSensor;
    }
    if (s.pieceSensor.kind == SensorKind::Distance) {
        c.distancePort = w.pieceSensorPort;
    }
    if (s.pieceSensor.kind == SensorKind::Optical) {
        c.opticalPort = w.pieceSensorPort;
    }
    if (s.homeSwitch.kind == SensorKind::Switch) {
        c.switchLines[0] = static_cast<std::uint8_t>(w.homeSwitch);
    }
    if (s.pieceSensor.kind == SensorKind::Switch) {
        c.switchLines[1] = static_cast<std::uint8_t>(w.pieceSwitch);
    }
    return c;
}

// ═══ THE WIRING, AS A TEAM WRITES IT (header) ══════════════════════════════════════════

/// The motor roles' smart ports, as motors(...) wrote them.
struct MotorPorts {
    /// The ports, in the declaration's `.motors` order.
    std::uint8_t port[kMaxMotorRoles]{};
    /// How many were written.
    std::uint8_t count = 0;
    /// A number outside 1..21 was written.
    bool bad = false;
    /// The file of the wire(...) line (captured where motors(...) is written).
    const char* file = nullptr;
    /// That line.
    std::uint32_t line = 0;
};
/// A pneumatic's 3-wire lines, as lines(...) wrote them.
struct LinePorts {
    /// The lines, in the positions' level order.
    Line line[kMaxLineRoles]{};
    /// How many were written.
    std::uint8_t count = 0;
    /// Not a letter 'A'..'H' was written.
    bool bad = false;
    /// The file of the wire(...) line (captured where lines(...) is written).
    const char* file = nullptr;
    /// That line.
    std::uint32_t lineNo = 0;
};

/// The wiring's refusals as NAMED FUNCTIONS — deliberately not constexpr (spec.hpp's idiom): GCC
/// prints the reason as the function's name.
namespace refused {
/// motors('A'): a motor port is a number.
inline void a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port() {}
/// lines(6, 7) / homeSwitch(66): a 3-wire port is a letter.
inline void a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port() {}
/// angleSensor('B'): an angle sensor is a smart sensor.
inline void an_angle_sensor_is_a_smart_sensor_give_it_a_port_number() {}
/// A smart port outside 1..21.
inline void a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors() {}
/// A 3-wire port that is not 'A'..'H'.
inline void a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines() {}
/// motors(...) gives a different count than the declaration's `.motors`.
inline void the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has() {}
/// A sensor the declaration has, unplugged in the wiring.
inline void the_mechanism_has_a_sensor_the_wiring_does_not_plug_in() {}
/// A sensor plugged in that the declaration does not have.
inline void the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have() {}
/// pieceSensor('C') for a distance or optical sensor: it is a smart sensor.
inline void a_piece_sensor_is_a_smart_sensor_give_it_a_port_number() {}
/// pieceSensor(9) for a limit switch: it is a 3-wire device.
inline void a_limit_switch_is_a_three_wire_device_give_it_a_letter() {}
/// A smart port used twice — the fallback of the named list below; UNREACHABLE from a list
/// (mechanisms()' per-entry check refuses a port outside 1..21 before it counts one), kept as the
/// switch's default.
inline void a_smart_port_is_used_twice() {}
/// A 3-wire port used twice — the fallback of the named list below; UNREACHABLE from a list
/// (mechanisms() skips an unbound line, and its per-entry check refuses one past H before it counts
/// one).
inline void a_three_wire_port_is_used_twice() {}
/// `wire(intake, motors(5), pieceSensor(9), pieceSensor(10))`: one sensor role written twice in a
/// wire(...) line — the last would win and the first port would never be checked, counted or built
/// (MECH-A1's review: a re-cabled sensor with its old line left in).
inline void the_wiring_plugs_in_the_same_sensor_twice() {}
/// A HAND-BUILT entry whose wiring names another mechanism than its declaration (wire() copies the
/// name, so only an entry built by hand can).
inline void the_wiring_names_another_mechanism_than_its_declaration() {}
/// A HAND-BUILT entry with a line role unbound, or a line on a role its declaration does not have
/// (wire() makes the spec's line count from lines(...), so only an entry built by hand can).
inline void the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has() {}
/// More than kMaxMechanisms in one list.
inline void more_mechanisms_than_a_robot_can_have() {}
/// The same declaration wired twice.
inline void the_same_mechanism_is_wired_twice() {}
/// Two declarations with one `.name` (the run-time lookup is by name).
inline void two_mechanisms_share_a_name() {}
/// A position whose level count is not the wiring's line count (U2).
inline void a_position_states_a_different_number_of_levels_than_the_wiring_has_lines() {}
/// wire(pneumatic, motors(...)).
inline void a_pneumatic_is_wired_with_lines_like_lines_A_not_motors() {}
/// wire(arm, lines(...)).
inline void a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines() {}
/// Smart port 1 used twice.
inline void smart_port_1_is_used_twice() {}
/// Smart port 2 used twice.
inline void smart_port_2_is_used_twice() {}
/// Smart port 3 used twice.
inline void smart_port_3_is_used_twice() {}
/// Smart port 4 used twice.
inline void smart_port_4_is_used_twice() {}
/// Smart port 5 used twice.
inline void smart_port_5_is_used_twice() {}
/// Smart port 6 used twice.
inline void smart_port_6_is_used_twice() {}
/// Smart port 7 used twice.
inline void smart_port_7_is_used_twice() {}
/// Smart port 8 used twice.
inline void smart_port_8_is_used_twice() {}
/// Smart port 9 used twice.
inline void smart_port_9_is_used_twice() {}
/// Smart port 10 used twice.
inline void smart_port_10_is_used_twice() {}
/// Smart port 11 used twice.
inline void smart_port_11_is_used_twice() {}
/// Smart port 12 used twice.
inline void smart_port_12_is_used_twice() {}
/// Smart port 13 used twice.
inline void smart_port_13_is_used_twice() {}
/// Smart port 14 used twice.
inline void smart_port_14_is_used_twice() {}
/// Smart port 15 used twice.
inline void smart_port_15_is_used_twice() {}
/// Smart port 16 used twice.
inline void smart_port_16_is_used_twice() {}
/// Smart port 17 used twice.
inline void smart_port_17_is_used_twice() {}
/// Smart port 18 used twice.
inline void smart_port_18_is_used_twice() {}
/// Smart port 19 used twice.
inline void smart_port_19_is_used_twice() {}
/// Smart port 20 used twice.
inline void smart_port_20_is_used_twice() {}
/// Smart port 21 used twice.
inline void smart_port_21_is_used_twice() {}
/// 3-wire port A used twice.
inline void three_wire_port_A_is_used_twice() {}
/// 3-wire port B used twice.
inline void three_wire_port_B_is_used_twice() {}
/// 3-wire port C used twice.
inline void three_wire_port_C_is_used_twice() {}
/// 3-wire port D used twice.
inline void three_wire_port_D_is_used_twice() {}
/// 3-wire port E used twice.
inline void three_wire_port_E_is_used_twice() {}
/// 3-wire port F used twice.
inline void three_wire_port_F_is_used_twice() {}
/// 3-wire port G used twice.
inline void three_wire_port_G_is_used_twice() {}
/// 3-wire port H used twice.
inline void three_wire_port_H_is_used_twice() {}
}  // namespace refused

namespace detail {
/// A smart port written as a NUMBER; anything else is refused by name; outside 1..21 sets `bad`.
template <class P>
consteval std::uint8_t smartPort(P p, bool& bad) {
    if constexpr (!std::same_as<P, int>) {
        refused::a_motor_port_is_a_number_like_6_a_letter_in_quotes_is_a_three_wire_port();
        return 0;
    } else {
        if (p < 1 || p > 21) {
            bad = true;
            return 0;
        }
        return static_cast<std::uint8_t>(p);
    }
}
/// A 3-wire letter ('A'..'H', or lower case); anything else sets `bad`.
consteval Line letterOf(char c, bool& bad) noexcept {
    if (c >= 'A' && c <= 'H') {
        return static_cast<Line>(c - 'A' + 1);
    }
    if (c >= 'a' && c <= 'h') {
        return static_cast<Line>(c - 'a' + 1);
    }
    bad = true;
    return Line::None;
}
/// A 3-wire port written as a LETTER in quotes; a number is refused by name.
template <class C>
consteval Line lineOf(C c, bool& bad) {
    if constexpr (!std::same_as<C, char>) {
        refused::a_three_wire_port_is_a_letter_in_quotes_like_A_a_number_is_a_smart_port();
        return Line::None;
    } else {
        return letterOf(c, bad);
    }
}
}  // namespace detail

/// One mechanism's motor ports: `motors(6)` — a NUMBER per motor role, in its `.motors` order (one
/// to four); `motors('A')` is refused by name. Records the line it is written on (the wire line).
template <class A>
[[nodiscard]] consteval MotorPorts motors(A a, std::source_location at = std::source_location::current()) {
    MotorPorts m{.file = at.file_name(), .line = at.line()};
    m.port[m.count++] = detail::smartPort(a, m.bad);
    return m;
}
/// Two motor ports: `motors(2, 3)`.
template <class A, class B>
[[nodiscard]] consteval MotorPorts motors(A a, B b, std::source_location at = std::source_location::current()) {
    MotorPorts m{.file = at.file_name(), .line = at.line()};
    m.port[m.count++] = detail::smartPort(a, m.bad);
    m.port[m.count++] = detail::smartPort(b, m.bad);
    return m;
}
/// Three motor ports.
template <class A, class B, class C>
[[nodiscard]] consteval MotorPorts motors(A a, B b, C c, std::source_location at = std::source_location::current()) {
    MotorPorts m{.file = at.file_name(), .line = at.line()};
    m.port[m.count++] = detail::smartPort(a, m.bad);
    m.port[m.count++] = detail::smartPort(b, m.bad);
    m.port[m.count++] = detail::smartPort(c, m.bad);
    return m;
}
/// Four motor ports (the most).
template <class A, class B, class C, class D>
[[nodiscard]] consteval MotorPorts motors(A a, B b, C c, D d, std::source_location at = std::source_location::current()) {
    MotorPorts m{.file = at.file_name(), .line = at.line()};
    m.port[m.count++] = detail::smartPort(a, m.bad);
    m.port[m.count++] = detail::smartPort(b, m.bad);
    m.port[m.count++] = detail::smartPort(c, m.bad);
    m.port[m.count++] = detail::smartPort(d, m.bad);
    return m;
}
/// FIVE or more motor ports: refused BY NAME — a mechanism has at most four motor roles (MECH-A1's
/// review: without this overload it was a plain "no matching function" error).
template <class A, class B, class C, class D, class E, class... More>
consteval MotorPorts motors(A, B, C, D, E, More...) {
    refused::a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors();
    return MotorPorts{};
}
/// NO motor port, `motors()`: wire() then refuses it BY NAME, "the wiring gives a different number of
/// motors than the mechanism has" (MECH-A1's review: no longer a plain C++ error).
[[nodiscard]] consteval MotorPorts motors(std::source_location at = std::source_location::current()) {
    return MotorPorts{.file = at.file_name(), .line = at.line()};
}
/// A pneumatic mechanism's 3-wire line: `lines('A')` — a LETTER in quotes; `lines(6)` is refused by
/// name. Records the line it is written on.
template <class A>
[[nodiscard]] consteval LinePorts lines(A a, std::source_location at = std::source_location::current()) {
    LinePorts l{.file = at.file_name(), .lineNo = at.line()};
    l.line[l.count++] = detail::lineOf(a, l.bad);
    return l;
}
/// Two lines, in its positions' level order: `lines('C', 'D')`.
template <class A, class B>
[[nodiscard]] consteval LinePorts lines(A a, B b, std::source_location at = std::source_location::current()) {
    LinePorts l{.file = at.file_name(), .lineNo = at.line()};
    l.line[l.count++] = detail::lineOf(a, l.bad);
    l.line[l.count++] = detail::lineOf(b, l.bad);
    return l;
}
/// THREE or more lines: refused BY NAME — a pneumatic mechanism has at most two line roles (MECH-A1's
/// review: without this overload it was a plain "no matching function" error).
template <class A, class B, class C, class... More>
consteval LinePorts lines(A, B, C, More...) {
    refused::a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines();
    return LinePorts{};
}
/// NO line, `lines()`: wire() then refuses it BY NAME, "a pneumatic mechanism needs one or two line
/// roles" (MECH-A1's review: no longer a plain C++ error).
[[nodiscard]] consteval LinePorts lines(std::source_location at = std::source_location::current()) {
    return LinePorts{.file = at.file_name(), .lineNo = at.line()};
}

/// A SENSOR role's port, as angleSensor(10), pieceSensor(9) / pieceSensor('E'), homeSwitch('B')
/// wrote it.
struct RolePort {
    /// Which role it is.
    enum class Role : std::uint8_t {
        Angle,       ///< the angle sensor (a smart port)
        Piece,       ///< the piece sensor (a smart port, or a line for a limit switch)
        HomeSwitch,  ///< the home switch (a line)
    } role;
    /// Its smart port; 0 for a line.
    std::uint8_t port = 0;
    /// Its line; None for a smart port.
    Line line = Line::None;
    /// Written as a LETTER (so a bad value is a bad 3-wire port, not a bad smart port).
    bool letter = false;
    /// Out of range.
    bool bad = false;
};
/// The angle sensor's smart port: `angleSensor(10)` — a NUMBER; `angleSensor('B')` is refused by name.
template <class P>
[[nodiscard]] consteval RolePort angleSensor(P port) {
    if constexpr (!std::same_as<P, int>) {
        refused::an_angle_sensor_is_a_smart_sensor_give_it_a_port_number();
        return RolePort{RolePort::Role::Angle, 0, Line::None, false, true};
    } else {
        bool bad = false;
        const std::uint8_t p = detail::smartPort(port, bad);
        return RolePort{RolePort::Role::Angle, p, Line::None, false, bad};
    }
}
/// The piece sensor's port: `pieceSensor(9)` for a distance or optical sensor (a number),
/// `pieceSensor('E')` for a limit switch (a letter) — the declaration says which, and wire() refuses
/// the wrong form by name.
template <class P>
[[nodiscard]] consteval RolePort pieceSensor(P port) {
    if constexpr (std::same_as<P, char>) {
        bool bad = false;
        const Line l = detail::letterOf(port, bad);
        return RolePort{RolePort::Role::Piece, 0, l, true, bad};
    } else {
        bool bad = false;
        const std::uint8_t p = detail::smartPort(port, bad);
        return RolePort{RolePort::Role::Piece, p, Line::None, false, bad};
    }
}
/// The home switch's line: `homeSwitch('B')` — a LETTER; `homeSwitch(66)` is refused by name.
template <class C>
[[nodiscard]] consteval RolePort homeSwitch(C line) {
    bool bad = false;
    const Line l = detail::lineOf(line, bad);
    return RolePort{RolePort::Role::HomeSwitch, 0, l, true, bad};
}

namespace detail {
/// Name the port used twice (refused::smart_port_N_is_used_twice).
constexpr void refuseSmartPortTwice(std::uint8_t p) {
    switch (p) {
        case 1: refused::smart_port_1_is_used_twice(); return;
        case 2: refused::smart_port_2_is_used_twice(); return;
        case 3: refused::smart_port_3_is_used_twice(); return;
        case 4: refused::smart_port_4_is_used_twice(); return;
        case 5: refused::smart_port_5_is_used_twice(); return;
        case 6: refused::smart_port_6_is_used_twice(); return;
        case 7: refused::smart_port_7_is_used_twice(); return;
        case 8: refused::smart_port_8_is_used_twice(); return;
        case 9: refused::smart_port_9_is_used_twice(); return;
        case 10: refused::smart_port_10_is_used_twice(); return;
        case 11: refused::smart_port_11_is_used_twice(); return;
        case 12: refused::smart_port_12_is_used_twice(); return;
        case 13: refused::smart_port_13_is_used_twice(); return;
        case 14: refused::smart_port_14_is_used_twice(); return;
        case 15: refused::smart_port_15_is_used_twice(); return;
        case 16: refused::smart_port_16_is_used_twice(); return;
        case 17: refused::smart_port_17_is_used_twice(); return;
        case 18: refused::smart_port_18_is_used_twice(); return;
        case 19: refused::smart_port_19_is_used_twice(); return;
        case 20: refused::smart_port_20_is_used_twice(); return;
        case 21: refused::smart_port_21_is_used_twice(); return;
        default: refused::a_smart_port_is_used_twice(); return;
    }
}
/// Name the 3-wire port used twice (refused::three_wire_port_X_is_used_twice).
constexpr void refuseLineTwice(Line l) {
    switch (l) {
        case Line::A: refused::three_wire_port_A_is_used_twice(); return;
        case Line::B: refused::three_wire_port_B_is_used_twice(); return;
        case Line::C: refused::three_wire_port_C_is_used_twice(); return;
        case Line::D: refused::three_wire_port_D_is_used_twice(); return;
        case Line::E: refused::three_wire_port_E_is_used_twice(); return;
        case Line::F: refused::three_wire_port_F_is_used_twice(); return;
        case Line::G: refused::three_wire_port_G_is_used_twice(); return;
        case Line::H: refused::three_wire_port_H_is_used_twice(); return;
        case Line::None: refused::a_three_wire_port_is_used_twice(); return;
    }
}
/// Name an entry's problem (entryProblem) — what mechanisms() refuses a HAND-BUILT entry for, by the
/// same name wire() refuses the mistake by when a declaration makes it.
constexpr void refuseEntry(EntryProblem e) {
    switch (e) {
        case EntryProblem::None:
            return;
        case EntryProblem::NameDiffers:
            refused::the_wiring_names_another_mechanism_than_its_declaration();
            return;
        case EntryProblem::MotorRoleUnbound:
        case EntryProblem::MotorPortExtra:
            refused::the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has();
            return;
        case EntryProblem::SmartPortPast21:
            refused::a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors();
            return;
        case EntryProblem::LineRoleUnbound:
        case EntryProblem::LineExtra:
            refused::the_wiring_gives_a_different_number_of_lines_than_the_mechanism_has();
            return;
        case EntryProblem::LinePastH:
            refused::a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines();
            return;
        case EntryProblem::LevelsVsLines:
            refused::a_position_states_a_different_number_of_levels_than_the_wiring_has_lines();
            return;
        case EntryProblem::AngleSensorUnplugged:
        case EntryProblem::HomeSwitchUnplugged:
        case EntryProblem::PieceSensorUnplugged:
        case EntryProblem::PieceSwitchUnplugged:
            refused::the_mechanism_has_a_sensor_the_wiring_does_not_plug_in();
            return;
        case EntryProblem::AngleSensorExtra:
        case EntryProblem::HomeSwitchExtra:
        case EntryProblem::PieceSensorExtra:
        case EntryProblem::PieceSwitchExtra:
            refused::the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have();
            return;
    }
}
}  // namespace detail

/// One mechanism, checked and wired: its spec, its ports, and its declaration's address — what one
/// wire(...) line makes.
struct Wired {
    /// The declaration as data.
    MechanismSpec spec{};
    /// Its ports.
    MechanismWiring wiring{};
    /// The declaration's address.
    const void* declared = nullptr;
};

namespace detail {
/// Bind each sensor role's port, refusing a bad one, one written twice, one the declaration lacks,
/// or one left unplugged.
template <class... R>
consteval void sensors(const MechanismSpec& s, MechanismWiring& w, R... roles) {
    bool angle = false;
    bool piece = false;
    bool home = false;
    for (const RolePort& r : std::array<RolePort, sizeof...(R)>{roles...}) {
        if (r.bad && r.letter) {
            refused::a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines();
        }
        if (r.bad && !r.letter) {
            refused::a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors();
        }
        switch (r.role) {
            case RolePort::Role::Angle:
                if (angle) {
                    refused::the_wiring_plugs_in_the_same_sensor_twice();
                }
                if (s.angleSensor.kind == SensorKind::None) {
                    refused::the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have();
                }
                w.angleSensor = r.port;
                angle = true;
                break;
            case RolePort::Role::Piece:
                if (piece) {
                    refused::the_wiring_plugs_in_the_same_sensor_twice();
                }
                if (s.pieceSensor.kind == SensorKind::None) {
                    refused::the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have();
                }
                if (s.pieceSensor.kind == SensorKind::Switch) {
                    if (!r.letter) {
                        refused::a_limit_switch_is_a_three_wire_device_give_it_a_letter();
                    }
                    w.pieceSwitch = r.line;
                } else {
                    if (r.letter) {
                        refused::a_piece_sensor_is_a_smart_sensor_give_it_a_port_number();
                    }
                    w.pieceSensorPort = r.port;
                }
                piece = true;
                break;
            case RolePort::Role::HomeSwitch:
                if (home) {
                    refused::the_wiring_plugs_in_the_same_sensor_twice();
                }
                if (s.homeSwitch.kind == SensorKind::None) {
                    refused::the_wiring_plugs_in_a_sensor_the_mechanism_does_not_have();
                }
                w.homeSwitch = r.line;
                home = true;
                break;
        }
    }
    if ((s.angleSensor.kind != SensorKind::None && !angle) || (s.pieceSensor.kind != SensorKind::None && !piece) ||
        (s.homeSwitch.kind != SensorKind::None && !home)) {
        refused::the_mechanism_has_a_sensor_the_wiring_does_not_plug_in();
    }
}
/// wire() for every motor kind: the declaration checked, then its ports and sensors.
template <class D, class... R>
consteval Wired wireMotors(const D& d, MotorPorts m, R... roles) {
    Wired x{.spec = d.spec(), .declared = &d};
    refuse(specRefusal(x.spec));
    if (m.bad) {
        refused::a_smart_port_is_not_a_number_from_1_to_21_or_more_than_four_motors();
    }
    if (m.count != x.spec.motorCount) {
        refused::the_wiring_gives_a_different_number_of_motors_than_the_mechanism_has();
    }
    copyText(x.wiring.mechanism, x.spec.name);
    for (std::size_t i = 0; i < m.count && i < kMaxMotorRoles; ++i) {
        x.wiring.motors[i] = m.port[i];
    }
    x.wiring.file = m.file;
    x.wiring.line = m.line;
    sensors(x.spec, x.wiring, roles...);
    return x;
}
}  // namespace detail

/// `wire(intake, motors(5), pieceSensor(9))`: one power mechanism's wiring, checked AT BUILD.
template <class... R>
[[nodiscard]] consteval Wired wire(const Power& d, MotorPorts m, R... r) {
    return detail::wireMotors(d, m, r...);
}
/// `wire(arm, motors(1))`: one arm's wiring, checked AT BUILD.
template <class... R>
[[nodiscard]] consteval Wired wire(const Arm& d, MotorPorts m, R... r) {
    return detail::wireMotors(d, m, r...);
}
/// `wire(lift, motors(2, 3), homeSwitch('B'))`: one lift's wiring, checked AT BUILD.
template <class... R>
[[nodiscard]] consteval Wired wire(const Lift& d, MotorPorts m, R... r) {
    return detail::wireMotors(d, m, r...);
}
/// `wire(flywheel, motors(4))`: one flywheel's wiring, checked AT BUILD.
template <class... R>
[[nodiscard]] consteval Wired wire(const Flywheel& d, MotorPorts m, R... r) {
    return detail::wireMotors(d, m, r...);
}
/// `wire(hood, lines('C', 'D'))`: one pneumatic's wiring, checked AT BUILD — every position's level
/// count against the lines FIRST (U2: one level on two lines would leave the second cylinder dead in
/// every position; checked before the spec, so the slip is named as itself).
template <class... R>
[[nodiscard]] consteval Wired wire(const Pneumatic& d, LinePorts l, R... roles) {
    if (l.bad) {
        refused::a_three_wire_port_is_not_a_letter_from_A_to_H_or_more_than_two_lines();
    }
    if (l.count == 0) {
        refused::a_pneumatic_mechanism_needs_one_or_two_line_roles();  // `lines()` (spec.hpp's name)
    }
    Wired x{.spec = d.spec(l.count), .declared = &d};
    for (std::size_t i = 0; i < x.spec.pneumatic.positionCount && i < kMaxPneumaticPositions; ++i) {
        if (x.spec.pneumatic.positions[i].levelCount != l.count) {
            refused::a_position_states_a_different_number_of_levels_than_the_wiring_has_lines();
        }
    }
    refuse(specRefusal(x.spec));
    detail::copyText(x.wiring.mechanism, x.spec.name);
    for (std::size_t i = 0; i < l.count && i < kMaxLineRoles; ++i) {
        x.wiring.lines[i] = l.line[i];
    }
    x.wiring.file = l.file;
    x.wiring.line = l.lineNo;
    detail::sensors(x.spec, x.wiring, roles...);
    return x;
}
/// `wire(hood, motors(...))`: refused by name — a pneumatic is wired with lines.
template <class... R>
consteval Wired wire(const Pneumatic&, MotorPorts, R...) {
    refused::a_pneumatic_is_wired_with_lines_like_lines_A_not_motors();
    return Wired{};
}
/// `wire(arm, lines(...))`: refused by name — a motor kind is wired with motors.
template <class D, class... R>
    requires(!std::same_as<D, Pneumatic>)
consteval Wired wire(const D&, LinePorts, R...) {
    refused::a_motor_mechanism_is_wired_with_motors_like_motors_6_not_lines();
    return Wired{};
}

/// THE ROBOT'S MECHANISMS: every wire(...) line, in boot order — checked AT BUILD across mechanisms
/// (header): a port or a line used twice (named), the same declaration twice, two `.name`s alike,
/// more than kMaxMechanisms, and every route id the list publishes (route_ids.hpp). Each entry is
/// checked AGAIN on its own here — its spec (specRefusal) and its wiring beside it (entryProblem),
/// by the names wire() uses — before any port is counted: wire() checks what a declaration makes,
/// and an entry built by hand must not walk around it (MECH-A1's review found that it could).
template <class... W>
[[nodiscard]] consteval MechanismTable mechanisms(const W&... wired) {
    MechanismTable r{};
    for (const Wired& w : {Wired{wired}...}) {
        if (r.count == kMaxMechanisms) {
            refused::more_mechanisms_than_a_robot_can_have();
            break;
        }
        refuse(specRefusal(w.spec));                        // a Wired built by hand is held to wire()'s
        detail::refuseEntry(entryProblem(w.spec, w.wiring));  // rules too: its spec, then its wiring
        if (r.indexOf(w.declared) >= 0) {                   // never matches nullptr (a hand-built entry)
            refused::the_same_mechanism_is_wired_twice();
        }
        for (std::size_t j = 0; j < r.count; ++j) {
            if (nameOf(r.specs[j]) == nameOf(w.spec)) {
                refused::two_mechanisms_share_a_name();
            }
        }
        r.specs[r.count] = w.spec;
        r.wiring[r.count] = w.wiring;
        r.declared[r.count] = w.declared;
        ++r.count;
    }
    // Every port and line used twice, NAMED. entryProblem() above has refused every port past 21 and
    // every line past H — and every one on a role its mechanism lacks — so each index below is in
    // bounds. (This function is consteval: an index out of bounds would stop the build, never read
    // past the array. A range check repeated here would refuse by the SAME name as entryProblem's and
    // hide either one from every test — MECH-A1's review's call-site sweep found exactly that.)
    bool smart[22]{};
    bool line[9]{};
    for (std::size_t i = 0; i < r.count; ++i) {
        const MechanismWiring& w = r.wiring[i];
        for (const std::uint8_t p : {w.motors[0], w.motors[1], w.motors[2], w.motors[3], w.angleSensor, w.pieceSensorPort}) {
            if (p == 0) {
                continue;
            }
            if (smart[p]) {
                detail::refuseSmartPortTwice(p);
            }
            smart[p] = true;
        }
        for (const Line l : {w.lines[0], w.lines[1], w.homeSwitch, w.pieceSwitch}) {
            if (l == Line::None) {
                continue;
            }
            const auto k = static_cast<std::size_t>(l);
            if (line[k]) {
                detail::refuseLineTwice(l);
            }
            line[k] = true;
        }
    }
    // The team lead's ruling of 2026-10-08: every mechanism publishes its actions as ROUTE IDS; an
    // id that cannot be published fails the build HERE, by name (route_ids.hpp).
    refuseRouteIds(routeIdsOf(r.specSpan()).problem);
    return r;
}
/// A robot with no mechanisms.
[[nodiscard]] consteval MechanismTable mechanisms() { return MechanismTable{}; }

/// THE DRIVE CHECK (the robot's own typed chassis table, beside its list — MECH-A5's
/// `mechanismsClearOf`): no mechanism role on a drive port or the IMU's — the build-time mirror of
/// RobotConfig::valid(), one of the places a later PTO changes (the brief's §1.15).
[[nodiscard]] constexpr bool clearOfDrive(const MechanismTable& r, std::span<const std::uint8_t> drivePorts,
                                          std::uint8_t imuPort) noexcept {
    return wiringProblem(r.specSpan(), r.wiringSpan(), drivePorts, imuPort) == nullptr;
}

/// Every bound mechanism MOTOR port, physical, in list order — what the drive's boot check is told
/// as its declared motor ports (calibration/boot_check.hpp `DeclaredMotorPorts`). Writes at most
/// `out.size()`; returns the count written.
[[nodiscard]] constexpr std::size_t declaredMotorPorts(const MechanismTable& r, std::span<int> out) noexcept {
    std::size_t k = 0;
    for (std::size_t i = 0; i < r.count && i < kMaxMechanisms; ++i) {
        for (std::size_t m = 0; m < r.specs[i].motorCount && m < kMaxMotorRoles; ++m) {
            if (k < out.size()) {
                out[k++] = r.wiring[i].motors[m];
            }
        }
    }
    return k;
}

/// ONE BOOT LINE per role (tag MECH; the caller logs it), from what robot::MechanismParts recorded:
///   "<mechanism>: <role> -> port <N> ok"
///   "<mechanism>: <role> -> port <N> ABSENT: <the provider's sentence>"
///   "<mechanism>: <role> -> port <N> MISSING, it runs WITHOUT it: <the provider's sentence>"
///   "<mechanism>: <role> -> 3-wire <X> not checkable (the brain cannot see a 3-wire device)"
///   "<mechanism>: <role> -> port <N> not asked (the mechanism is already ABSENT)"
/// A 3-wire role names its line ("3-wire B") in every form. Truncates, never overflows; writes at
/// least a NUL whenever `cap > 0`.
inline void roleLine(char* out, std::size_t cap, const char* mechanismName, const char* role, std::uint8_t port,
                     Line line, RoleState st, const char* refusal, bool degradeOnly) {
    if (out == nullptr || cap == 0) {
        return;
    }
    char where[16]{};
    if (line != Line::None) {
        std::snprintf(where, sizeof where, "3-wire %c", lineLetter(line));
    } else {
        std::snprintf(where, sizeof where, "port %d", static_cast<int>(port));
    }
    const char* name = mechanismName != nullptr ? mechanismName : "";
    const char* what = role != nullptr ? role : "";
    switch (st) {
        case RoleState::Present:
            std::snprintf(out, cap, "%.23s: %s -> %s ok", name, what, where);
            return;
        case RoleState::Refused:
            std::snprintf(out, cap, "%.23s: %s -> %s %s: %.110s", name, what, where,
                          degradeOnly ? "MISSING, it runs WITHOUT it" : "ABSENT", refusal != nullptr ? refusal : "");
            return;
        case RoleState::NotCheckable:
            std::snprintf(out, cap, "%.23s: %s -> %s not checkable (the brain cannot see a 3-wire device)", name,
                          what, where);
            return;
        case RoleState::NotAsked:
            std::snprintf(out, cap, "%.23s: %s -> %s not asked (the mechanism is already ABSENT)", name, what,
                          where);
            return;
    }
    out[0] = '\0';
}

/// The wiring's words, added to `mechanism::words` (kinds.hpp has the rest).
namespace words {
/// A robot's list of mechanisms: `inline constexpr MechanismTable kRobotXMechanisms = mechanisms(...)`.
using mechanism::MechanismTable;
/// The list itself.
using mechanism::mechanisms;
/// One mechanism's line of it.
using mechanism::wire;
/// Its motors' smart ports.
using mechanism::motors;
/// Its 3-wire lines.
using mechanism::lines;
/// Its angle sensor's smart port.
using mechanism::angleSensor;
/// Its piece sensor's port (a number) or line (a letter).
using mechanism::pieceSensor;
/// Its home switch's line.
using mechanism::homeSwitch;
}  // namespace words

}  // namespace shulib::mechanism
