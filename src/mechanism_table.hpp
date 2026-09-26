#pragma once
//
// THE MECHANISM TABLE — the ONE typed input about a robot's MECHANISMS: every motor that is not
// the drivetrain, and every pneumatic line (chunk T3, 2026-09-26; the pneumatic claw added by T3's
// addendum, E2, the same day). The PROS-free sibling of src/chassis_table.hpp, which stays "the ONE
// typed input about a drivetrain"; selected by the SAME define the Makefile's ROBOT switch sets
// (SHULIB_ROBOT_TANK_2026), so `make ROBOT=tank` and this table cannot disagree about which robot
// it is.
//
// WHAT IT DECLARES. Robot two (the 2026 tank chassis) has two SEPARATE claw PIVOT motors, stated
// by the team lead on 2026-09-26: port 6 drives one pivot forward while R1 is held and backward
// while R2 is held; port 7 drives the other forward on L1 and backward on L2; RED cartridge each,
// full power (12 V) while held, and on release each HOLDS its angle. Every other robot declares
// none. Two pivots are two ENTRIES — one entry per motor — never one entry with two ports: a single
// mechanism fans one voltage to all its motors, so one two-motor entry would move both claws on R1
// (T3 brief, landmine 1).
// Robot two also has a PNEUMATIC CLAW, stated by the team lead the same day (T3 addendum §0): its
// solenoid is on the brain's 3-wire port A; pressing A opens the claw and pressing A again closes
// it; it is CLOSED at program start; and when the robot is disabled it KEEPS its state (it must
// not drop what it holds). Whether its valve is single or double, and which line level opens it,
// are NOT KNOWN (A4 register HA-159) — see "THE POLARITY" and "ONE LINE" below.
//
// TWO KINDS, TWO ENTRY TYPES. A motor mechanism (MotorMechanismEntry: a signed SMART port, a
// cartridge, a brake mode, two hold-to-run buttons, a voltage) and a pneumatic one
// (PneumaticMechanismEntry: a 3-WIRE line, a polarity, a start and a safe POSITION, one toggle
// button) share nothing but a name and a provenance, so they are separate types in separate arrays
// — mirroring the library's own hal::MotorMechanism / hal::PneumaticMechanism and
// config::MechanismConfig's motorCount / pneumaticCount — not one struct with a kind tag. With a
// tag, every check would have to branch on it, and a field of the other kind (a voltage on a
// solenoid, a line on a motor) would sit in every entry, silently ignored or silently read. Here a
// pneumatic entry CANNOT be read as a motor: the compiler has no `.signedPort` on it. And the one
// confusion that would matter most is impossible: 3-wire line 1 ('A') is not smart port 1, and
// isDeclaredMechanismPort() — "shulib Calibrate"'s drop — reads the MOTOR entries only.
//
// RULE (chassis_table.hpp's, adopted whole): A VALUE IS NEVER INVENTED. Each field has an UNSET
// value — port 0, line 0, Cartridge::Unset, std::nullopt, volts 0, a null or empty string — and an
// entry is DECLARED only when every field is stated: the completeness check below refuses a
// partial entry at COMPILE time. When a fact arrives, THIS is the one place to edit, with the date
// and the source beside it.
//
// WHY std::optional FOR THE MODES, THE BUTTONS, THE POLARITY AND THE POSITIONS. A field left out
// of a designated initializer is value-initialised, and a value-initialised enum is its FIRST
// enumerator: hal::BrakeMode's is Coast (the invented default that DROPS a loaded pivot — T3 brief,
// landmine 7), hal::ControllerButton's is L1 (the port-7 pivot's forward button, AND
// teleop::DriverLoopConfig's default re-zero button), and PneumaticPosition's is Closed; a
// value-initialised bool is `false` — an invented polarity. A plain field would turn "nobody
// stated it" into a plausible real value; an optional makes it a value of its own, which the
// completeness check refuses. The cartridge reuses chassis_table.hpp's Cartridge, whose first
// enumerator already IS Unset.
//
// SIGNS LIVE HERE AND NOWHERE ELSE, as in the chassis table: `signedPort` is handed to the motor
// adapter AS TYPED, and PROS reverses a negative port exactly once, in the adapter. Robot two's
// entries are +6 and +7 because nobody has seen which way either pivot turns under a positive
// command: "forward" is the team lead's word for R1 / L1, and which PHYSICAL way that moves each
// claw is UNMEASURED (worksheet Station J, A4 register HA-153). A pivot that moves the wrong way
// is fixed by flipping ITS sign here — never by swapping its buttons.
//
// THE POLARITY LIVES HERE, IN ONE CONSTANT: kClawOpenCommand (below), the 3-wire line level that
// OPENS the claw. Its value is a GUESS ("ON = open", the addendum's starting point), UNVERIFIED:
// worksheet Station J.9 settles it (the claw must be CLOSED after the start, and the first press
// of A must OPEN it). The claw's start and safe states are stated as POSITIONS (Closed), never as
// levels, and pneumaticCommandFor() is the one place a position becomes a level — so flipping
// kClawOpenCommand moves the start command and the safe command together, and no other CODE is
// edited. Its VALUE is recorded the way a pivot's sign is: the host test pins it, and A4 register
// HA-159 and Station J.9 quote it — a flip is a measurement, and all three change with it in the
// same change. A claw that opens on the wrong press is fixed by flipping that constant — never by
// changing its positions or its button.
//
// ONE LINE, because one is what was reported (port A). Whether the valve is single or double is
// NOT KNOWN; Station J looks at how many 3-wire cables run from the valve into the brain. If it is
// two, this entry does NOT quietly gain a second line: the second line's letter is read off the
// brain first, and its level per position is a fact of its own — the library's
// hal::PneumaticMechanism commands EVERY line with the SAME level (mechanism.hpp, set()), so a
// second line that must be driven the opposite way cannot be expressed by adding it to the same
// mechanism. That design is made when the fact arrives (the library half, B2, says how
// config::MechanismConfig's second pneumatic line would carry it); nothing here builds it now.
//
// THE HOST SUITE TESTS THESE VERY BYTES. chassis_table.hpp defines robot two's table only under
// its define, so its host test has to RE-TYPE the numbers (test/chassis_table_test.cpp's header
// says so) and a typo in the real table is invisible to the suite. Here robot two's table is
// defined in EVERY build under its own name, kRobotTwoMechanisms, and only the selector kMechanisms
// depends on the define — so test/mechanism_table_test.cpp reads the real entries, and a swapped
// button in this file turns the suite red. A program uses kMechanisms, never the per-robot name.
// (The alternative — one name defined differently per variant, as the chassis table does — would
// also force a host test that defined the tank macro into an ODR violation against every other TU
// that includes the chassis table.)
//
// WHO READS IT, as of this commit:
//   * "shulib Calibrate" (src/calibrate_program.cpp): the census's motor ports go through
//     dropDeclaredMechanismPorts() BEFORE any ProsMotor is built, so a declared claw PIVOT is
//     never a drive candidate — no gearing written, no Coast, no pulse. It logs one line per
//     dropped port, `port 6: claw pivot (declared) -- not a candidate, not touched`; the words
//     "claw pivot" there are the kind of every MOTOR entry declared today, and a table that
//     declares another kind of motor mechanism changes that line with it. The pneumatic claw is
//     not in the census at all (a 3-wire port is not a smart port), and Calibrate never drives a
//     3-wire line.
//   * "shulib Teleop": NOT YET. The pivots' bindings, boot lines and panel row, and the claw's
//     toggle, are wired when T3's library half (the hold-to-run binding, the driver loop's
//     bindings overload, mechanisms built from config::MechanismConfig) and the addendum's (B2: the
//     toggle binding, the provider's 3-wire output, pneumatic mechanisms from config) are merged.
//     Nothing here depends on those library pieces, on purpose — they were being written in
//     parallel; the entries convert to them at that merge.
//
// COMPILE-TIME CHECKS (the static_asserts at the end of this file) — each the thing a typo would
// produce, each a named predicate the host test breaks with hand-built tables:
//   1. no more entries of either kind than its array holds;
//   2. every declared entry COMPLETE, of either kind (the rule above);
//   3. every motor port a real SMART port, |p| in 1..21, and no port declared twice;
//   4. forward != reverse on every motor entry;
//   5. every binding button unique across the WHOLE table — the pivots' forward and reverse
//      buttons and the claw's toggle (one button, one mechanism);
//   6. no binding button, of either kind, is kTeleopRezeroButton (below — Y);
//   7. 0 < volts <= 12 (hal::kMaxMotorVoltage) on every motor entry, which also refuses NaN and
//      infinity;
//   8. no MOTOR port is a drive port of the SAME variant's typed chassis table, nor its IMU port
//      (checked for the build's kChassis / kMechanisms pair; the same predicate runs in the host
//      suite against robot two's drive ports). A 3-wire line is a different connector: the
//      chassis table names no 3-wire port, so there is nothing to collide with;
//   9. every pneumatic line a real 3-WIRE port, 1..8 ('A'..'H'), and no line declared twice.
// Checks 1-7 and 9 run on robot two's table in EVERY build, the host's included; check 8 runs on
// the build's own pair. What no compiler can check — that a motor is really on the port, which
// way it turns, and which level opens the claw — is Station J's.
//
// PROS-FREE ON PURPOSE: plain data and pure helpers, so the host test can include it by relative
// path. It lives in src/, not include/shulib/, because it is a per-robot composition fact, not
// library API.

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "chassis_table.hpp"
#include "shulib/hal/controller.hpp"
#include "shulib/hal/motor.hpp"

namespace shulib::bench {

/// THE BUTTON NO MECHANISM MAY TAKE: "shulib Teleop"'s field-centric RE-ZERO button — **Y**, the
/// team lead's ruling of 2026-09-26 (T3 addendum §0): A became the pneumatic claw's toggle, so the
/// re-zero moved to Y. (It was A, restated from G1 Part 2b's Teleop; the library's own
/// DriverLoopConfig::rezeroButton default is L1, which robot two's port-7 pivot uses.)
/// NOT YET TIED TO TELEOP — the coordinator's step at merge: Teleop's own constant,
/// `kRezeroButton` in src/teleop_program.cpp (committed in the main tree at 7f9b9a1, :141, still
/// `hal::ControllerButton::A`), is not in the tree this table was written against (b622d9b). At
/// the merge the coordinator moves it A -> Y (the addendum's one-line Teleop change) and ties the
/// two with `static_assert(kRezeroButton == bench::kTeleopRezeroButton)` (or sets one from the
/// other), so they can never drift. Until then the two DISAGREE, and the claw must not be wired
/// into a Teleop whose re-zero is still A. (This paragraph describes the tree BEFORE that merge;
/// the merge that ties the two rewrites it to name the static_assert and where it sits.) The
/// library's bindings also refuse, at construction, a button equal to the loop's re-zero button —
/// that is the runtime check; this is the compile-time one.
inline constexpr hal::ControllerButton kTeleopRezeroButton = hal::ControllerButton::Y;

/// THE CLAW'S OPEN POLARITY — **UNVERIFIED**. The 3-wire line level that OPENS robot two's
/// pneumatic claw: `true` (line HIGH = open) is the addendum's starting GUESS, nothing more — the
/// team lead reported the port, not the polarity (A4 register HA-159). **Station J.9 settles it:**
/// the claw must be CLOSED after the start and OPEN on the first press of A. This constant is
/// BACKWARDS only when the claw is open after the start AND that first press CLOSES it, with the
/// valve's one cable in A (J.9.0); a claw that does not move on the press says nothing about the
/// polarity (the port, the air or the valve — J.9.3). **THE ONLY CODE EDIT IS HERE** — the claw's
/// entry reads it, its start and safe states are POSITIONS, and pneumaticCommandFor() turns every
/// position into a level through it, so the start command and the safe command move with the flip.
/// **The flip is a measurement, recorded where the guess is written:** the value is pinned in
/// test/mechanism_table_test.cpp, as the pivots' signs are, and quoted by HA-159 and Station J.9 —
/// the pin and both quotes change with it, in the same change.
inline constexpr bool kClawOpenCommand = true;

/// One mechanism MOTOR driven hold-to-run from two buttons. Aggregate, so a host test can build a
/// bad one by hand. Every field has an UNSET value (the header's rule), and a declared entry must
/// state them all.
struct MotorMechanismEntry {
    const char* name;          // "claw pivot 6": boot lines, panel, log. UNSET = null or ""
    std::int8_t signedPort;    // |p| in 1..21, the SIGN is the flip (header). 0 = UNSET
    Cartridge cartridge;       // RED / GREEN / BLUE. Unset = nobody has READ one off the motor
    std::optional<hal::BrakeMode> safeMode;        // the state on release. nullopt = UNSET
    std::optional<hal::ControllerButton> forward;  // held => +volts. nullopt = UNSET
    std::optional<hal::ControllerButton> reverse;  // held => -volts. nullopt = UNSET
    double volts;              // the magnitude while a button is held, in (0, 12]. 0 = UNSET
    const char* provenance;    // who stated what, and when. UNSET = null or ""
};

/// The two positions of a pneumatic mechanism, in the robot's words: for the claw they mean what
/// they say; in general, Open is the position the entry's `openCommand` level drives it to and
/// Closed the other. A POSITION, never a line level — the level is pneumaticCommandFor()'s, through
/// the entry's polarity (header, "THE POLARITY").
enum class PneumaticPosition { Closed, Open };

/// The brain's 3-wire ports: lines 1..8 = 'A'..'H' — the same numbering config::MechanismConfig's
/// pneumaticPorts and hal::pros::ProsDigitalOut use.
inline constexpr int kMaxAdiLine = 8;

/// One PNEUMATIC mechanism on ONE 3-wire line, toggled by one button (header, "ONE LINE"). Its own
/// type, never a MotorMechanismEntry with a kind tag (header, "TWO KINDS"). Aggregate; every field
/// has an UNSET value, and a declared entry must state them all.
struct PneumaticMechanismEntry {
    const char* name;           // "claw": boot lines, panel, log. UNSET = null or ""
    std::uint8_t adiLine;       // the 3-wire port, 1..8 = 'A'..'H'. 0 = UNSET
    std::optional<bool> openCommand;  // the line level that OPENS it (header). nullopt = UNSET
    std::optional<PneumaticPosition> start;  // commanded at program start. nullopt = UNSET
    /// The position a LIBRARY-FORCED safe state commands (hal::PneumaticMechanism's `safe`: COMP's
    /// end-of-run guard, later). NOT what a disable does: on a disable or a disconnect the toggle
    /// commands NOTHING and the mechanism keeps its state (T3 addendum §0). nullopt = UNSET
    std::optional<PneumaticPosition> safe;
    std::optional<hal::ControllerButton> toggle;  // each press flips it. nullopt = UNSET
    const char* provenance;     // who stated what, and when. UNSET = null or ""
};

/// The most MOTOR entries a table can hold. The controller has twelve driver-usable buttons, one
/// is reserved (kTeleopRezeroButton), each motor entry takes two distinct ones and each pneumatic
/// entry one — so 2 x motors + pneumatics <= 11, and FIVE motor entries is the most any valid
/// table can declare (the eleventh button then left for one pneumatic). The array holds six, so a
/// sixth motor entry is refused by the button checks (it would need the reserved button) rather
/// than by the array's size.
inline constexpr std::size_t kMaxMechanisms = 6;

/// The most PNEUMATIC entries a table can hold: one per 3-wire port (check 9 refuses a line
/// declared twice), so the array can hold every valid table.
inline constexpr std::size_t kMaxPneumatics = static_cast<std::size_t>(kMaxAdiLine);

/// One robot's mechanisms. `motorCount` = `pneumaticCount` = 0: this robot declares none.
struct MechanismTable {
    MotorMechanismEntry motors[kMaxMechanisms];
    std::size_t motorCount;
    PneumaticMechanismEntry pneumatics[kMaxPneumatics];
    std::size_t pneumaticCount;
};

/// ROBOT TWO — the 2026 tank chassis. Defined in every build (header: the host suite tests these
/// bytes); selected by kMechanisms only under SHULIB_ROBOT_TANK_2026.
/// THE PIVOTS' PORTS, CARTRIDGE, BUTTONS, VOLTS, SAFE STATE: REPORTED by the team lead 2026-09-26
/// (T3's request, verbatim in its brief). NOT SEEN: the 2026-09-24/25 captures list motors only on
/// 11-18 and 20, so no census has shown a motor on 6 or 7 yet (A4 register HA-153). SIGNS: + on
/// both, i.e. each motor's own forward; which physical way that moves each claw is UNMEASURED.
/// SAFE STATE Hold: the team lead's "holds its angle" -- a Hold at 0 V reaches the motor only after
/// DEFECTS2, and holding a LOADED pivot is unmeasured (A4 register HA-156). VOLTS 12: full power
/// passes straight through while a button is held (a stall cut-off is out of T3), so a pivot held
/// against its stop heats at the motor's DEFAULT current limit, 2500 mA, and nothing in shulib
/// watches a mechanism motor's temperature yet (A4 register HA-154). THE PORTS are all the brain
/// can check: a claw cable swapped with a drive cable is invisible to it (A4 register HA-155).
/// THE PNEUMATIC CLAW: REPORTED by the team lead 2026-09-26 (T3 addendum §0) — 3-wire port A
/// (line 1), toggled by A, CLOSED at program start, keeps its state when disabled; single or
/// double, and the polarity, NOT KNOWN (A4 register HA-159): one line declared, the polarity
/// kClawOpenCommand's guess. Its SAFE position, CLOSED, is the addendum's statement — a closed claw
/// keeps what it holds — not the team lead's words. Nothing has seen it: shulib's census reads
/// the 21 smart ports only, and no program in this tree drives a 3-wire line.
inline constexpr MechanismTable kRobotTwoMechanisms = {
    .motors =
        {
            {
                .name = "claw pivot 6",
                .signedPort = +6,
                .cartridge = Cartridge::Red,
                .safeMode = hal::BrakeMode::Hold,
                .forward = hal::ControllerButton::R1,
                .reverse = hal::ControllerButton::R2,
                .volts = 12.0,
                .provenance = "team lead 2026-09-26: port 6, RED, R1 forward / R2 back, 12 V while "
                              "held, holds on release; REPORTED, not yet seen by a census; sign + "
                              "= the motor's own forward (way UNMEASURED)",
            },
            {
                .name = "claw pivot 7",
                .signedPort = +7,
                .cartridge = Cartridge::Red,
                .safeMode = hal::BrakeMode::Hold,
                .forward = hal::ControllerButton::L1,
                .reverse = hal::ControllerButton::L2,
                .volts = 12.0,
                .provenance = "team lead 2026-09-26: port 7, RED, L1 forward / L2 back, 12 V while "
                              "held, holds on release; REPORTED, not yet seen by a census; sign + "
                              "= the motor's own forward (way UNMEASURED)",
            },
        },
    .motorCount = 2,
    .pneumatics =
        {
            {
                .name = "claw",
                .adiLine = 1,  // 3-wire port A
                .openCommand = kClawOpenCommand,  // UNVERIFIED: flip the CONSTANT, never this line
                .start = PneumaticPosition::Closed,
                .safe = PneumaticPosition::Closed,
                .toggle = hal::ControllerButton::A,
                .provenance = "team lead 2026-09-26 (port A; polarity unknown): A opens, A again "
                              "closes, CLOSED at program start, keeps its state when disabled; "
                              "single or double solenoid NOT KNOWN, one line declared; safe = "
                              "CLOSED is the T3 addendum's (a closed claw keeps what it holds)",
            },
        },
    .pneumaticCount = 1,
};

/// Every other robot (the bench bot, the invented X-drive): no mechanism declared.
inline constexpr MechanismTable kNoMechanisms = {
    .motors = {}, .motorCount = 0, .pneumatics = {}, .pneumaticCount = 0};

#if defined(SHULIB_ROBOT_TANK_2026)
/// THE BUILD'S TABLE — the only name a program uses. Robot two's.
inline constexpr const MechanismTable& kMechanisms = kRobotTwoMechanisms;
#else
/// THE BUILD'S TABLE — the only name a program uses. None declared for this robot.
inline constexpr const MechanismTable& kMechanisms = kNoMechanisms;
#endif

/// How many MOTOR entries every check and lookup reads: `motorCount`, clamped to the array, so a
/// hand-built table with an impossible count is refused by mechanismCountValid() instead of read
/// past its end.
constexpr std::size_t motorEntryCount(const MechanismTable& t) {
    return t.motorCount < kMaxMechanisms ? t.motorCount : kMaxMechanisms;
}

/// How many PNEUMATIC entries every check reads: `pneumaticCount`, clamped the same way.
constexpr std::size_t pneumaticEntryCount(const MechanismTable& t) {
    return t.pneumaticCount < kMaxPneumatics ? t.pneumaticCount : kMaxPneumatics;
}

/// Check 1: both counts fit their arrays.
constexpr bool mechanismCountValid(const MechanismTable& t) {
    return t.motorCount <= kMaxMechanisms && t.pneumaticCount <= kMaxPneumatics;
}

/// A text field is STATED when it is non-null and non-empty.
constexpr bool mechanismTextStated(const char* s) { return s != nullptr && s[0] != '\0'; }

/// Check 2, for one motor entry: every field stated (the header's rule). Ranges are checked
/// separately.
constexpr bool motorEntryComplete(const MotorMechanismEntry& e) {
    return mechanismTextStated(e.name) && mechanismTextStated(e.provenance) && e.signedPort != 0
           && e.cartridge != Cartridge::Unset && e.safeMode.has_value() && e.forward.has_value()
           && e.reverse.has_value() && e.volts != 0.0;
}

/// Check 2, for one pneumatic entry: every field stated — the line, the polarity, both positions
/// and the button included (none may default: header, "WHY std::optional"). The line's RANGE is
/// check 9's.
constexpr bool pneumaticEntryComplete(const PneumaticMechanismEntry& e) {
    return mechanismTextStated(e.name) && mechanismTextStated(e.provenance) && e.adiLine != 0
           && e.openCommand.has_value() && e.start.has_value() && e.safe.has_value()
           && e.toggle.has_value();
}

/// Check 2: every declared entry, of either kind, complete.
constexpr bool mechanismTableComplete(const MechanismTable& t) {
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        if (!motorEntryComplete(t.motors[i])) return false;
    }
    for (std::size_t i = 0; i < pneumaticEntryCount(t); ++i) {
        if (!pneumaticEntryComplete(t.pneumatics[i])) return false;
    }
    return true;
}

/// Check 3: every motor |port| in 1..21, none declared twice (by |port|, so +6 and -6 collide).
constexpr bool mechanismPortsValid(const MechanismTable& t) {
    const std::size_t n = motorEntryCount(t);
    for (std::size_t i = 0; i < n; ++i) {
        const int a = absPort(t.motors[i].signedPort);
        if (a < 1 || a > kMaxPort) return false;
        for (std::size_t j = i + 1; j < n; ++j) {
            if (absPort(t.motors[j].signedPort) == a) return false;
        }
    }
    return true;
}

/// Check 9: every pneumatic line in 1..8 ('A'..'H'), none declared twice.
constexpr bool pneumaticLinesValid(const MechanismTable& t) {
    const std::size_t n = pneumaticEntryCount(t);
    for (std::size_t i = 0; i < n; ++i) {
        const int line = t.pneumatics[i].adiLine;
        if (line < 1 || line > kMaxAdiLine) return false;
        for (std::size_t j = i + 1; j < n; ++j) {
            if (t.pneumatics[j].adiLine == line) return false;
        }
    }
    return true;
}

/// Two STATED buttons that are the same button. An unset button is nobody's button: it collides
/// with nothing here, and check 2 (completeness) is the one that refuses it — each check tests one
/// property, so a failing static_assert names the real fault.
constexpr bool mechanismSameButton(const std::optional<hal::ControllerButton>& a,
                                   const std::optional<hal::ControllerButton>& b) {
    return a.has_value() && b.has_value() && *a == *b;
}

/// True when motor entry `e` binds `b` (as its forward or its reverse button).
constexpr bool motorEntryBinds(const MotorMechanismEntry& e,
                               const std::optional<hal::ControllerButton>& b) {
    return mechanismSameButton(e.forward, b) || mechanismSameButton(e.reverse, b);
}

/// Check 4: no motor entry's forward button is its own reverse button — one button cannot mean
/// both ways.
constexpr bool mechanismForwardReverseDistinct(const MechanismTable& t) {
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        const MotorMechanismEntry& e = t.motors[i];
        if (mechanismSameButton(e.forward, e.reverse)) return false;
    }
    return true;
}

/// Check 5: no button is bound by two entries, of either kind: motor vs motor (any pairing of
/// forward/reverse), pneumatic toggle vs every motor button, and toggle vs toggle. (An entry's own
/// forward vs its own reverse is check 4's.)
constexpr bool mechanismButtonsUnique(const MechanismTable& t) {
    const std::size_t n = motorEntryCount(t);
    const std::size_t k = pneumaticEntryCount(t);
    for (std::size_t i = 0; i < n; ++i) {
        const MotorMechanismEntry& e = t.motors[i];
        for (std::size_t j = i + 1; j < n; ++j) {
            const MotorMechanismEntry& o = t.motors[j];
            if (motorEntryBinds(o, e.forward) || motorEntryBinds(o, e.reverse)) return false;
        }
    }
    for (std::size_t i = 0; i < k; ++i) {
        const std::optional<hal::ControllerButton>& toggle = t.pneumatics[i].toggle;
        for (std::size_t j = 0; j < n; ++j) {
            if (motorEntryBinds(t.motors[j], toggle)) return false;
        }
        for (std::size_t j = i + 1; j < k; ++j) {
            if (mechanismSameButton(t.pneumatics[j].toggle, toggle)) return false;
        }
    }
    return true;
}

/// Check 6: no entry, of either kind, binds `reserved` (the build passes kTeleopRezeroButton).
constexpr bool mechanismButtonsAvoid(const MechanismTable& t, hal::ControllerButton reserved) {
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        const MotorMechanismEntry& e = t.motors[i];
        if (e.forward == reserved || e.reverse == reserved) return false;
    }
    for (std::size_t i = 0; i < pneumaticEntryCount(t); ++i) {
        if (t.pneumatics[i].toggle == reserved) return false;
    }
    return true;
}

/// Check 7: every motor entry's volts in (0, kMaxMotorVoltage]. Written as a negated conjunction
/// so a NaN fails it (every comparison with NaN is false).
constexpr bool mechanismVoltsValid(const MechanismTable& t) {
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        const double v = t.motors[i].volts;
        if (!(v > 0.0 && v <= hal::kMaxMotorVoltage.value())) return false;
    }
    return true;
}

/// Check 8: no MOTOR port is a drive port of `c` (either side, by |port|) or `c`'s IMU port. The
/// pneumatic lines are 3-wire ports, a different connector the chassis table never names.
constexpr bool mechanismsDisjointFromChassis(const MechanismTable& m, const ChassisTable& c) {
    for (std::size_t i = 0; i < motorEntryCount(m); ++i) {
        const int a = absPort(m.motors[i].signedPort);
        if (tableSideOf(c, a) != 0) return false;
        if (c.imuPort != 0 && a == static_cast<int>(c.imuPort)) return false;
    }
    return true;
}

/// The 3-wire line level that puts pneumatic entry `e` at `position`: its `openCommand` for Open,
/// the other level for Closed. THE one place a position becomes a level (header, "THE POLARITY"),
/// so flipping an entry's polarity moves its start command and its safe command together. For a
/// COMPLETE entry only (check 2 — every table a build compiles): an unset polarity has no level,
/// and reading one is refused in a constant expression.
constexpr bool pneumaticCommandFor(const PneumaticMechanismEntry& e, PneumaticPosition position) {
    return position == PneumaticPosition::Open ? *e.openCommand : !*e.openCommand;
}

/// The level `e` is commanded to at program start (its `start` position): what the coordinator
/// hands the library as the mechanism's start value at merge. A COMPLETE entry only.
constexpr bool pneumaticStartCommand(const PneumaticMechanismEntry& e) {
    return pneumaticCommandFor(e, *e.start);
}

/// The level a LIBRARY-FORCED safe state commands (its `safe` position): the `safe` argument of
/// hal::PneumaticMechanism at merge. Never what a disable does (the entry's `safe` field). A
/// COMPLETE entry only.
constexpr bool pneumaticSafeCommand(const PneumaticMechanismEntry& e) {
    return pneumaticCommandFor(e, *e.safe);
}

/// True when PHYSICAL smart port `port` (1..21) carries a declared mechanism MOTOR in `t`. Matched
/// on |entry|, so a flipped pivot (-6) is still port 6; a port outside 1..21 is never one. The
/// pneumatic entries are NEVER read here: a 3-wire line is not a smart port (line 1, 'A', is not
/// port 1), and this is the predicate "shulib Calibrate" drops census ports by.
constexpr bool isDeclaredMechanismPort(const MechanismTable& t, int port) {
    if (port < 1 || port > kMaxPort) return false;
    for (std::size_t i = 0; i < motorEntryCount(t); ++i) {
        if (absPort(t.motors[i].signedPort) == port) return true;
    }
    return false;
}

/// The declared mechanism ports "shulib Calibrate" took out of its candidate list, in the order the
/// census listed them: the ports its `claw pivot (declared)` log lines name.
struct DroppedMechanismPorts {
    std::int8_t ports[kMaxPort];
    std::size_t count;
};

/// "shulib Calibrate"'s candidate filter, run IN PLACE on the census's own port list, so Calibrate
/// keeps ONE list and has no second copy to keep in step. `ports` holds the census's motor ports
/// (physical, positive). Every declared mechanism port of `t` leaves it and is appended to
/// `dropped` (which is reset first); the ports that stay DRIVE CANDIDATES move to the front, in
/// the census's order. Every slot past them is set to 0 (never a port), so a caller that went on
/// reading the old length would find no port there, not a stale duplicate. Returns how many
/// stayed: Calibrate's new candidate count. Calibrate calls it ONCE, after the census and BEFORE
/// any ProsMotor is built, and step 4 builds adapters only from `ports[0 .. returned)`.
///
/// WHY IN PLACE, and why the count comes back as the RESULT: re-listing the survivors and setting
/// the new count is logic, and logic belongs where the host suite can RUN it. Calibrate's own part
/// is one assignment and one log loop, whose text test/mechanism_table_test.cpp pins (the program
/// itself cannot be compiled on the host). `[[nodiscard]]`: a call whose count is thrown away
/// would leave Calibrate's old count over a list that now holds fewer ports.
///
/// The brain has 21 ports, so no census lists more, and `dropped` (21 slots) can never fill. If it
/// somehow did, a declared port would still LEAVE the candidate list and only be missing from the
/// log: a claw is never handed back to Calibrate.
[[nodiscard]] constexpr std::size_t dropDeclaredMechanismPorts(const MechanismTable& t,
                                                               std::span<std::int8_t> ports,
                                                               DroppedMechanismPorts& dropped) {
    constexpr std::size_t cap = static_cast<std::size_t>(kMaxPort);
    dropped = DroppedMechanismPorts{};
    std::size_t kept = 0;
    for (std::size_t i = 0; i < ports.size(); ++i) {
        const std::int8_t p = ports[i];  // read before any write: kept <= i, always
        if (isDeclaredMechanismPort(t, p)) {
            if (dropped.count < cap) dropped.ports[dropped.count++] = p;
        } else {
            ports[kept++] = p;
        }
    }
    for (std::size_t i = kept; i < ports.size(); ++i) ports[i] = 0;
    return kept;
}

// ═══ THE COMPILE-TIME CHECKS (header, "COMPILE-TIME CHECKS") ══════════════════════════
// Robot two's table, in EVERY build (checks 1-7 and 9):
static_assert(mechanismCountValid(kRobotTwoMechanisms), "robot two: more mechanisms than fit");
static_assert(mechanismTableComplete(kRobotTwoMechanisms),
              "robot two: a declared mechanism has an UNSET field (a value is never invented)");
static_assert(mechanismPortsValid(kRobotTwoMechanisms),
              "robot two: a mechanism port is outside 1..21 or declared twice");
static_assert(pneumaticLinesValid(kRobotTwoMechanisms),
              "robot two: a pneumatic line is outside 1..8 (3-wire A..H) or declared twice");
static_assert(mechanismForwardReverseDistinct(kRobotTwoMechanisms),
              "robot two: a mechanism's forward and reverse buttons are the same button");
static_assert(mechanismButtonsUnique(kRobotTwoMechanisms),
              "robot two: one button is bound to two mechanisms (pivots and claw together)");
static_assert(mechanismButtonsAvoid(kRobotTwoMechanisms, kTeleopRezeroButton),
              "robot two: a mechanism takes shulib Teleop's re-zero button (kTeleopRezeroButton)");
static_assert(mechanismVoltsValid(kRobotTwoMechanisms),
              "robot two: a mechanism's volts are not in (0, 12]");
// The BUILD's table (all nine; the same eight again when it is robot two's, and check 8 against
// the SAME variant's chassis table — the only pairing that means anything):
static_assert(mechanismCountValid(kMechanisms) && mechanismTableComplete(kMechanisms)
                  && mechanismPortsValid(kMechanisms) && pneumaticLinesValid(kMechanisms)
                  && mechanismForwardReverseDistinct(kMechanisms)
                  && mechanismButtonsUnique(kMechanisms)
                  && mechanismButtonsAvoid(kMechanisms, kTeleopRezeroButton)
                  && mechanismVoltsValid(kMechanisms),
              "this build's mechanism table fails a consistency check");
static_assert(mechanismsDisjointFromChassis(kMechanisms, kChassis),
              "a mechanism port is a DRIVE port (or the IMU port) of this build's chassis table");
// THE SELECTION, restated as a check: a misspelt macro in the #if above would compile cleanly and
// hand robot two an EMPTY table, and Calibrate would then take its claws as drive candidates again
// — HA-111's defect class (a build carrying another robot's facts). A typo would have to be made
// twice, in two places, to get past this.
#if defined(SHULIB_ROBOT_TANK_2026)
static_assert(&kMechanisms == &kRobotTwoMechanisms && kMechanisms.motorCount == 2
                  && kMechanisms.pneumaticCount == 1,
              "ROBOT=tank must select robot two's mechanism table (the claw pivots on 6 and 7, "
              "the pneumatic claw on 3-wire A)");
#else
static_assert(&kMechanisms == &kNoMechanisms, "only robot two declares mechanisms");
#endif

}  // namespace shulib::bench
