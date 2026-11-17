#pragma once
//
// boot_source.hpp — WHICH DESCRIPTION DRIVES THIS ROBOT, decided once per boot, and the
// robot::Robot config it becomes (G1 Part 2b, 2026-09-26; the decisions are R3d Part 2's, moved
// out of src/main.cpp unchanged).
//
// ── Why this is its own file ────────────────────────────────────────────────────────
// Until G1 Part 2b the boot source was a 77-line block inside src/main.cpp's initialize(), and it
// wrote six program globals and painted the screen on the way. Every program that boots robot two's
// graph needs it: "shulib Drive" (src/programs/drive_program.cpp, since T4 Part 3 — "shulib Teleop"
// until then), "shulib Auton Tester" (COMP-t) and "shulib Competition"
// (src/programs/competition_program.cpp, COMP-b); the invented X-drive's program builds its own
// graph without it. So the decisions live here ONCE, and each program paints its own refusal and
// logs with its own prefix. Nothing here writes a
// program global or a screen.
//
// ── The decisions (R3d Part 2, rulings P2/P3 — moved, not changed) ──────────────────
// TWO sources, one representation (hal::DrivetrainDescription), a fixed order:
//   1. the CALIBRATION RECORD on the SD card (/usd/shulib_drivetrain.bin), written by "shulib
//      Calibrate" after the robot measured itself, VERIFIED against the brain's port registry on
//      the same boot, before any adapter exists (calibration/boot_check.hpp);
//   2. else src/robots/chassis_table.hpp's typed table, through tableToDescription() — the ONLY path
//      from the table into a graph — with a WARNING, and registry-verified too;
//   3. else refuse, naming BOTH reasons.
// A record ALWAYS beats a table; a record that fails its port check NEVER falls back to the table
// (calibration::chooseBootSource is that table of rules, host-tested). The dead-port POLICY is
// applied once, inside calibration::verifyAgainstRegistry: its MISSING ports become the absent
// list below, and robot::Robot builds them as ABSENT group members (G1 Part 2a, ruling M1).
//
// ── Everything decided here is PROS-free; only the card and the registry are not ────
//  * THE PURE CORE: bootConfigFrom() — description + verdict + the mechanism list → the
//    config::RobotConfig robot::Robot is built from (the drivetrain AND, since T3 Part 2, every
//    mechanism src/robots/mechanism_table.hpp declares — through the library's one projection,
//    mechanism::toMechanismConfig, since MECH-A5), plus the absent-port list.
//  * THE DECISION: readRecord() (the card's bytes → a decoded record, or why not) and
//    decideBootSourceFrom() (record + registry + the table's description → which source, the
//    port check of the one chosen, the cross-check, the refusal, the config) — each decision
//    line handed to the caller's printer AS IT IS DECIDED (the brief's A9, design (a): today's
//    text and order, before the session header).
//  * THE SHIM (src/common/boot_source.cpp, PROS-bound, a dozen lines): decideBootSource() opens the
//    card, makes the registry, turns this build's typed table into a description, and prints
//    each line under the caller's tag as ONE serial write() per piece (hal::pros::serialWrapped,
//    SER1 — a line can run to 511 characters, so it wraps, never cut).
// All of the first two is inline here so the host test (test/boot_source_test.cpp, the brief's
// test 6) runs EVERY decision against the IBlockSource / IDeviceRegistry fakes, the way
// test/chassis_table_test.cpp includes src/robots/chassis_table.hpp: the host build compiles
// test/*.cpp and, of src/, only the team's PROS-free auton file (src/team/autons.cpp, since COMP-t)
// — never this header's .cpp, which calls PROS. (Until the Part 2b review's second round the
// decision flow lived in the .cpp beside the PROS calls, where no test could reach it.)
//
// ── The absent ports cross ONE door ─────────────────────────────────────────────────
// The list is private storage with a length. It leaves only as `BootConfig::absent()` — a span
// bounded to the ports that ARE absent — or as `BootConfig::buildOptions()`, the
// robot::BuildOptions built from that span, and a program spells them through the outcome's
// `built` member (`boot.built.buildOptions()`): the one door, and the one the test goes through.
// A span over the whole fixed array would hand validate() the unused zeros, and it refuses
// "absent port 0 is not one of this drivetrain's drive ports": the G1 Part 2b brief spells that
// hazard out, and keeping the array private is the fix at the root rather than a warning at each
// call site. (It cannot stop a program that builds a longer span from `absent().data()` on
// purpose; nothing in the tree does.)
//
// NO HARDWARE CLAIM for this file's arrangement. The decisions themselves ran on robot two on
// 2026-09-25 inside the previous "shulib Teleop" (eight boots from the RECORD, R3d-PROGRESS
// "2026-09-25 evening"); this file's version of them has not run on hardware.

#include <array>
#include <cstddef>
#include <cstdio>
#include <span>
#include <string_view>

#include "common/robot_words.hpp"
#include "robots/mechanism_table.hpp"
#include "shulib/calibration/boot_check.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/hal/block_source.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/robot/robot.hpp"

namespace shulib::bench {

// ═══ THE MECHANISMS, INTO THE CONFIG (T3 Part 2, 2026-09-26; the mechanism core's words since MECH-A5) ═══
// src/robots/mechanism_table.hpp is the ONE typed input about a robot's mechanisms — since MECH-A5 its
// DECLARATIONS and their wiring (mechanism::MechanismTable); config::MechanismConfig is what
// robot::Robot builds them from. The projection is the LIBRARY's ONE (mechanism::toMechanismConfig,
// mechanism/wiring.hpp), called here in the pure core so the host suite runs it against the REAL list
// (test/boot_source_test.cpp, test/driver_panel_claws_test.cpp). CONFIG ORDER IS THE LIST'S ORDER —
// robot two's is today's: pivot 6, pivot 7, then the claw — so config index i IS the list's entry i,
// for the Robot's accessors, the mechanism service and every boot line. Nothing is converted by hand
// and nothing defaulted: the signed motor ports (a Reversed motor is a NEGATIVE port, reversed once by
// its adapter), the cartridge, the safe state, each 3-wire line, the sensor roles' ports, and a
// pneumatic's START and SAFE as LEVELS — the levels its declaration states for its start and safe
// POSITIONS. The START level is what the builder drives each line to when it makes it (A4 register
// HA-119: a 3-wire output is driven the moment it is constructed); the program's FIRST driver-control
// entry SENDS it again (the mechanism service's enterDriverControl()), so an autonomous that left the
// claw OPEN closes it there — the team lead's ruling (2026-09-26): "starts closed — whatever
// autonomous left it at". The buttons and the volts are not config fields: they are the driver
// file's. (Until MECH-A5 this section converted the old entry structs itself, refused a table that
// would not convert — mechanismTableProblem() — and kept the motors before the pneumatics; the list's
// own build checks now refuse what that refused, and its order is the config's.)

/// The words the boot check uses for a declared mechanism MOTOR port that carries a motor the
/// description does not list — Calibrate's words for them (its census line, "claw pivot 6 (declared)"
/// since CAL1 Part B; "claw pivot (declared)" before it), because
/// every MOTOR role robot two declares is a claw pivot; a list that declares another kind of motor
/// mechanism changes this with it (L9 Part B's boot wording makes it the program's).
/// (ROB1-A, 2026-11-11: robot TWO's label. decideBootSourceFrom() reads its `Words`' label —
/// src/common/robot_words.hpp, robot one's "mechanism motors (...)" — and this constant stays robot two's,
/// byte for byte; the static_assert below holds the two the same.)
inline constexpr const char* kDeclaredMotorLabel = "claw pivots (declared, not drive motors)";
static_assert(std::string_view{kDeclaredMotorLabel} == std::string_view{kRobotTwoWords.declaredMotorLabel},
              "robot two's boot label is kDeclaredMotorLabel, verbatim");

/// The most mechanism MOTOR ports a list can declare: kMaxMechanisms mechanisms of four motors each.
inline constexpr std::size_t kMaxDeclaredMotorPorts = mechanism::kMaxMechanisms * mechanism::kMaxMotorRoles;
static_assert(mechanism::kMaxMechanisms == config::kMaxMechanisms,
              "a mechanism list must fit a RobotConfig whole (bootConfigFrom converts every entry)");

class BootConfig;

/// THE PURE CORE: the config robot::Robot is built from, for the description the boot chose and
/// the port check it passed — and the ports that check found MISSING.
///
///  * `identity.name` ← `robotName` (bounded, NUL-terminated; the team and season stay empty —
///    nothing on the robot knows them), because a banner that says which robot it is looking at
///    is worth one line and the per-robot name lives in src/robots/chassis_table.hpp's `robot` field;
///  * the drivetrain is a TANK whose description is `description`, COPIED AS IT IS — signed ports,
///    both inches-per-radian, the track width, the cartridge, the IMU port, the SOURCE and the
///    provenance, byte for byte;
///  * the odometry is the drive encoders (the fall robots have no tracking wheels);
///  * no GPS (robot two has none) — robot::Robot then installs hal::AbsentGps and no corrector;
///  * every mechanism `mechanisms` declares, in the LIST's order, each through the library's one
///    projection, mechanism::toMechanismConfig(spec, wiring) (the header's "THE MECHANISMS, INTO THE
///    CONFIG"). `kNoMechanisms` gives none;
///  * `tuning` is `chassis::ChassisConfig{}`, the library's PROVISIONAL defaults, UNTOUCHED: the
///    one thing a tank's tuning needs that a person should not type — the stall check's geometry
///    and "no independent motion source" — is applied by robot::Robot::effectiveTuning(), which is
///    exactly what src/main.cpp's tankChassisConfig() did by hand. Writing it here as well would
///    be two owners for one rule;
///  * the absent list: every drive port of `description`, LEFT side then RIGHT in the stored
///    order, that `verdict.isMissing()` — as the port MAGNITUDE the builder takes.
///
/// PRECONDITION: `verdict.usable()` (Ok or Degraded) — a refused verdict describes a robot that
/// must not be built, and no caller in the tree builds one. (A mechanisms(...) list always converts:
/// its own build checks refused every name past 23 characters, every list past eight mechanisms.)
[[nodiscard]] inline BootConfig bootConfigFrom(const hal::DrivetrainDescription& description,
                                               const calibration::BootVerdict& verdict,
                                               std::string_view robotName,
                                               const MechanismTable& mechanisms);

/// The pure core's answer: the config, and the absent drive ports behind one door (header).
class BootConfig {
public:
    /// The whole robot, as data — what `robot::validate()` and `robot::Robot` take.
    config::RobotConfig config{};

    /// The drive ports the boot check found MISSING, as port magnitudes 1..21, LEFT then RIGHT in
    /// the description's order — exactly as many as there are, never the unused tail of the
    /// storage. The span points into THIS object, so it is valid while this object is.
    [[nodiscard]] std::span<const int> absent() const noexcept {
        return std::span<const int>{absentPorts_.data(), absentCount_};
    }

    /// The builder's options for this boot: `{.absentDrivePorts = absent()}`, and nothing else.
    /// The only spelling of the options a program should use (header: one door). Borrows this
    /// object's storage, so it must not outlive it — robot::Robot reads the PORTS during
    /// construction only. A program may add a pacer before building (`opts.pacer = &guard;`,
    /// the Auton Tester since COMP-t and the competition program after it): that pacer is
    /// BORROWED for the Robot's whole life (robot.hpp, BuildOptions::pacer), so it must outlive
    /// the Robot — declare it first.
    [[nodiscard]] robot::BuildOptions buildOptions() const noexcept {
        return robot::BuildOptions{.absentDrivePorts = absent()};
    }

private:
    friend BootConfig bootConfigFrom(const hal::DrivetrainDescription&,
                                     const calibration::BootVerdict&, std::string_view,
                                     const MechanismTable&);

    std::array<int, hal::kMaxSmartPorts> absentPorts_{};
    std::size_t absentCount_ = 0;
};

inline BootConfig bootConfigFrom(const hal::DrivetrainDescription& description,
                                 const calibration::BootVerdict& verdict,
                                 std::string_view robotName, const MechanismTable& mechanisms) {
    SHULIB_PRECONDITION(verdict.usable(),
                        "bootConfigFrom: the port check REFUSED this description -- nothing may "
                        "be built from it");
    BootConfig out;
    config::RobotConfig& c = out.config;

    const std::size_t nameCap = sizeof c.identity.name - 1;
    std::size_t n = 0;
    for (; n < robotName.size() && n < nameCap; ++n) {
        c.identity.name[n] = robotName[n];
    }
    c.identity.name[n] = '\0';

    c.drivetrain.kind = config::DrivetrainKind::Tank;
    c.drivetrain.tank.description = description;
    c.odometry.kind = config::OdometryKind::DriveEncoders;
    c.sensors.gps.present = false;
    // The mechanisms, in the LIST's order (the header's CONFIG ORDER), through the library's one
    // projection. A list holds at most mechanism::kMaxMechanisms = config::kMaxMechanisms (static_assert).
    std::size_t k = 0;
    for (std::size_t i = 0; i < mechanisms.count && i < config::kMaxMechanisms; ++i) {
        c.mechanisms[k++] = mechanism::toMechanismConfig(mechanisms.specs[i], mechanisms.wiring[i]);
    }
    c.mechanismCount = static_cast<std::uint8_t>(k);
    // c.tuning stays chassis::ChassisConfig{} ON PURPOSE (see the declaration above).

    const hal::DriveSideDescription* sides[2] = {&description.left, &description.right};
    for (const hal::DriveSideDescription* side : sides) {
        for (std::size_t i = 0; i < side->count; ++i) {
            const int port = hal::DrivetrainDescription::portMagnitude(side->signedPorts[i]);
            if (verdict.isMissing(port) && out.absentCount_ < out.absentPorts_.size()) {
                out.absentPorts_[out.absentCount_++] = port;
            }
        }
    }
    return out;
}

/// How many physical ports 1..21 the two descriptions disagree about — a port one lists and the
/// other does not, or a port both list with different signs. The boot's CROSS-CHECK between the
/// card's record and the typed table (a disagreement is not an error: the record measured THIS
/// robot, and the table was typed from a build that may no longer exist — but a person should be
/// told before driving). On robot two on 2026-09-25 it read 10: the record's front is the table's
/// back, which flips every sign, plus ports 14 and 19, which only the table lists.
[[nodiscard]] inline int countPortDisagreements(const hal::DrivetrainDescription& a,
                                                const hal::DrivetrainDescription& b) noexcept {
    int differing = 0;
    for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
        if (a.signedPortOf(p) != b.signedPortOf(p)) {
            ++differing;
        }
    }
    return differing;
}

/// What one boot decided: usable or refused (with the refusal's headline and reason, for the
/// program to paint), which source and why, the port check's verdict, the cross-check count, and
/// — when usable — the config and the absent ports, in `built`.
///
/// It has no accessors of its own on purpose: a program reads the config and the builder's
/// options through `built` (`boot.built.config`, `boot.built.buildOptions()`), the same door the
/// host test goes through. (Until the Part 2b review's second round it carried forwarders to
/// them that no test called — a second, untested door to the one hazard the header warns about.)
struct BootOutcome {
    /// True when a description was chosen and passed its port check: the program may build.
    bool ok = false;
    /// The refusal's short headline (screen and serial), empty when `ok`.
    char headline[64]{};
    /// The refusal's reason, naming the port or the field, empty when `ok`.
    char why[192]{};
    /// Which source was chosen (UseRecord / UseTable), or Refuse.
    calibration::BootSource source = calibration::BootSource::Refuse;
    /// The `boot source:` sentence — always a string literal from chooseBootSource().
    const char* sourceWhy = "";
    /// The port check of the CHOSEN description (the table's, when the table was chosen). Its
    /// `reason` is what the panel's source row and the `port check:` line print, and what the
    /// worksheet's Station G quotes.
    calibration::BootVerdict verdict{};
    /// How many ports the card's record and the typed table disagree about; 0 unless the record
    /// was chosen (the cross-check runs only then).
    int crossCheckDiffering = 0;
    /// The config and the absent ports (meaningful only when `ok`) — the one door (header).
    BootConfig built{};
};

/// What the card held, read ONCE at boot: the decoded description, or why there is none.
struct RecordRead {
    /// The record's description (meaningful only when `decoded`).
    hal::DrivetrainDescription description{};
    /// True when the card held a record that passed every format check.
    bool decoded = false;
    /// Why there is no usable record — no card or no file, an EMPTY file, or the decoder's own
    /// refusal naming the check that failed; empty when `decoded`.
    char why[192] = "";
};

/// Read the calibration record through `card` and decode it. `path` is only what the messages call
/// the file (on the brain, `/usd/shulib_drivetrain.bin`). A card that is not open, an open but
/// EMPTY file and a record the decoder refuses are three different sentences and one decision:
/// not decoded.
[[nodiscard]] inline RecordRead readRecord(hal::IBlockSource& card, const char* path) {
    RecordRead out{};
    const char* p = path != nullptr ? path : "";
    if (!card.isOpen()) {
        std::snprintf(out.why, sizeof out.why,
                      "no calibration record: %s could not be opened (no SD card, or the robot "
                      "has never been calibrated)",
                      p);
        return out;
    }
    // Read into a buffer LARGER than a record on purpose: a longer file must show up as a LENGTH
    // refusal, not as "the first 180 bytes fitted".
    std::byte raw[calibration::kRecordBytes + 32]{};
    const std::size_t got = card.readAll(raw);
    if (got == 0) {
        // The one message the move ADDED (G1 Part 2b, ruling G1-2b-R7): an open but EMPTY file
        // used to print "calibration record: NOT USED -- " with nothing after the dash. The
        // decision was and is the same (not decoded); only the sentence is new.
        std::snprintf(out.why, sizeof out.why,
                      "the calibration record %s is EMPTY (0 bytes read) -- run shulib Calibrate", p);
        return out;
    }
    out.decoded = calibration::decodeRecord(std::span<const std::byte>{raw, got}, out.description,
                                            out.why, sizeof out.why);
    return out;
}

/// The source as the `boot source:` line's word.
[[nodiscard]] inline const char* bootSourceWord(calibration::BootSource s) noexcept {
    switch (s) {
        case calibration::BootSource::UseRecord: return "RECORD";
        case calibration::BootSource::UseTable: return "TYPED TABLE";
        case calibration::BootSource::Refuse: return "REFUSE";
    }
    return "REFUSE";
}

namespace detail {
/// Fill a refused outcome: the headline and the reason, for the caller to paint and print.
inline void refuseWith(BootOutcome& out, const char* headline, const char* why) {
    out.ok = false;
    // Precisions one short of each buffer, so -Wformat-truncation can see the bound (a verdict's
    // reason is up to 319 bytes; the refusal's `why` keeps its first 191).
    std::snprintf(out.headline, sizeof out.headline, "%.63s", headline);
    std::snprintf(out.why, sizeof out.why, "%.191s", why);
}
}  // namespace detail

/// The longest decision line decideBootSourceFrom() hands its printer, with room to spare: the
/// TYPED TABLE warning is 117 bytes of fixed text, a decision word of at most 8 and a verdict
/// reason of up to 319 — 444.
inline constexpr std::size_t kBootLineBytes = 512;

/// THE BOOT DECISION, PROS-free (header): verify the card's record against the registry, fall back
/// to the typed table only when there is no usable record, and refuse when neither will do — the
/// decisions of R3d Part 2 (rulings P2/P3), moved from src/main.cpp at G1 Part 2b with every
/// decision unchanged. Each decision line is handed to `printLine(const char*)` as it is decided,
/// in today's text and order: the record line, the table line when the table cannot describe a
/// drivetrain, `boot source:`, the TYPED TABLE warning, the cross-check. Returns the outcome;
/// paints nothing and sets no global.
///
/// `fromTable` is this build's typed table as a description (tableToDescription(kChassis) — the
/// ONLY path from the table into a graph) and `tableMissing` the words naming what it lacks
/// (describeMissingForDescription), printed only when the table cannot describe a drivetrain.
/// `mechanisms` is this build's list (kMechanisms; T3 Part 2, the mechanism core's words since
/// MECH-A5): its MOTOR ports are the port check's declared ports (mechanism::declaredMotorPorts(),
/// named `kDeclaredMotorLabel`, never as NEW motors), and after the source is chosen one more refusal
/// can end the boot — a declared mechanism SMART port (a motor's since T3 Part 2; an angle sensor's or
/// a distance or optical piece sensor's since MECH-A5) that the CHOSEN description lists as a DRIVE
/// port (`port N is both a claw (<name>) and a drive motor in the record -- recalibrate ...`, or
/// `port N is both <name>'s piece sensor and ...`), printed as it is decided. `robotName` becomes the
/// config's identity (bootConfigFrom).
///
/// `Words` (LAST, defaulted to this build's robot — chunk ROB1-A, re-check R7): the robot's words for the
/// port check's label and step 5's MOTOR refusal — robot two's "claw" words verbatim, robot one's
/// "mechanism" words (src/common/robot_words.hpp). A host test passes `WordsOf<RobotId::RobotOne>{}`;
/// every other call is unchanged.
template <class PrintLine, class Words = ThisRobotWords>
[[nodiscard]] BootOutcome decideBootSourceFrom(const RecordRead& record,
                                               const hal::IDeviceRegistry& registry,
                                               const hal::DrivetrainDescription& fromTable,
                                               const char* tableMissing,
                                               const MechanismTable& mechanisms,
                                               std::string_view robotName, PrintLine&& printLine,
                                               Words = {}) {
    BootOutcome out{};
    char line[kBootLineBytes];

    // The declared mechanism MOTOR ports, as the port check takes them: a motor there that the
    // description does not list is a claw pivot, not a NEW motor (T3 Part 2). Physical ports: the
    // wiring holds them unsigned (a Reversed motor's direction is its declaration's).
    int declaredPorts[kMaxDeclaredMotorPorts]{};
    const std::size_t declaredCount = mechanism::declaredMotorPorts(mechanisms, declaredPorts);
    const calibration::DeclaredMotorPorts declared{
        .ports = std::span<const int>{declaredPorts, declaredCount}, .label = Words::value.declaredMotorLabel};

    // 1. the record, verified against the ports on THIS boot, before anything is constructed.
    if (record.decoded) {
        out.verdict = calibration::verifyAgainstRegistry(record.description, registry, declared);
        std::snprintf(line, sizeof line, "calibration record: DECODED, port check %s -- %s",
                      calibration::bootDecisionWord(out.verdict.decision), out.verdict.reason);
    } else {
        std::snprintf(line, sizeof line, "calibration record: NOT USED -- %s", record.why);
    }
    printLine(static_cast<const char*>(line));

    // 2. the table as the second source.
    char tableWhy[192] = "";
    const bool tableValid = fromTable.valid(tableWhy, sizeof tableWhy);
    if (!tableValid) {
        std::snprintf(line, sizeof line, "typed table: cannot describe a drivetrain -- UNSET: %s (%s)",
                      tableMissing != nullptr ? tableMissing : "", tableWhy);
        printLine(static_cast<const char*>(line));
    }

    // 3. the decision, as a host-tested table (calibration::chooseBootSource) rather than an
    //    if-chain here.
    const calibration::BootSourceChoice choice =
        calibration::chooseBootSource(record.decoded, out.verdict.decision, tableValid);
    out.source = choice.source;
    out.sourceWhy = choice.why;
    std::snprintf(line, sizeof line, "boot source: %s -- %s", bootSourceWord(choice.source),
                  choice.why);
    printLine(static_cast<const char*>(line));

    const hal::DrivetrainDescription* chosen = &record.description;
    if (choice.source == calibration::BootSource::UseTable) {
        chosen = &fromTable;
        out.verdict = calibration::verifyAgainstRegistry(fromTable, registry, declared);
        std::snprintf(line, sizeof line,
                      "WARNING: driving from the TYPED TABLE -- unverified since the rebuild; run "
                      "shulib Calibrate (slot 4). Port check %s -- %s",
                      calibration::bootDecisionWord(out.verdict.decision), out.verdict.reason);
        printLine(static_cast<const char*>(line));
        if (out.verdict.decision == calibration::BootDecision::Refuse) {
            detail::refuseWith(out, "the TYPED TABLE does not match the ports", out.verdict.reason);
            return out;
        }
    } else if (choice.source == calibration::BootSource::Refuse) {
        char both[192];
        std::snprintf(both, sizeof both, "%.80s | %.100s", record.why,
                      record.decoded ? out.verdict.reason : tableWhy);
        detail::refuseWith(out, "NO USABLE DRIVETRAIN DESCRIPTION", both);
        return out;
    }

    // 4. the record/table CROSS-CHECK — the only other place a boot may read the table: if the
    //    card's record and the typed table disagree about which ports drive, say so. Not an error
    //    (the record measured THIS robot), but exactly what a person should be told before
    //    driving. Printed here, as it is decided, since G1 Part 2b (it used to be a TermSink WARN
    //    line after the session header — the brief's A9).
    if (choice.source == calibration::BootSource::UseRecord) {
        out.crossCheckDiffering = countPortDisagreements(record.description, fromTable);
        if (out.crossCheckDiffering > 0) {
            std::snprintf(line, sizeof line,
                          "CROSS-CHECK: the MEASURED record and the typed chassis table disagree "
                          "about %d port(s). The record wins (it moved THIS robot); the table is "
                          "stale and should be updated or deleted.",
                          out.crossCheckDiffering);
            printLine(static_cast<const char*>(line));
        }
    }

    // 5. the MECHANISMS (T3 Part 2; every smart ROLE since MECH-A5): no declared mechanism port — a
    //    motor's, an angle sensor's, a distance or optical piece sensor's — may be a drive port of the
    //    description just chosen. A record taken before Calibrate skipped the claws could list one,
    //    and robot two would then drive a claw as a wheel. (RobotConfig::valid() would refuse that
    //    too, but its words name a collision, not the fix.) A MOTOR role keeps T3 Part 2's words
    //    byte for byte ("a claw" — every motor robot two declares is a claw pivot; L9 Part B makes
    //    the noun the program's); a SENSOR role is named by its mechanism and role, and its fix is the
    //    robot file's wire(...) line first (the record measured a MOTOR on that port).
    const bool fromRecord = choice.source == calibration::BootSource::UseRecord;
    constexpr const char* kMotorRoleWords[mechanism::kMaxMotorRoles] = {"motor 1", "motor 2", "motor 3",
                                                                         "motor 4"};
    for (std::size_t i = 0; i < mechanisms.count && i < mechanism::kMaxMechanisms; ++i) {
        const mechanism::MechanismSpec& spec = mechanisms.specs[i];
        const mechanism::MechanismWiring& wiring = mechanisms.wiring[i];
        // Its smart roles in boot-line order (MechanismParts'): the motors, the angle sensor, a
        // distance or optical piece sensor. A 3-wire role is not a smart port.
        struct SmartRole {
            std::uint8_t port;
            const char* word;
            bool motor;
        };
        SmartRole roles[mechanism::kMaxMotorRoles + 2]{};
        std::size_t n = 0;
        for (std::size_t m = 0; m < spec.motorCount && m < mechanism::kMaxMotorRoles; ++m) {
            roles[n++] = SmartRole{wiring.motors[m], kMotorRoleWords[m], true};
        }
        if (spec.angleSensor.kind == mechanism::SensorKind::Rotation) {
            roles[n++] = SmartRole{wiring.angleSensor, "angle sensor", false};
        }
        if (spec.pieceSensor.kind == mechanism::SensorKind::Distance ||
            spec.pieceSensor.kind == mechanism::SensorKind::Optical) {
            roles[n++] = SmartRole{wiring.pieceSensorPort, "piece sensor", false};
        }
        for (std::size_t r = 0; r < n; ++r) {
            const int port = roles[r].port;
            if (port == 0 || chosen->sideOf(port) == 0) {
                continue;
            }
            char why[192];
            if (roles[r].motor) {
                // Robot two's words verbatim ("a claw"); robot one's "a mechanism" (ROB1-A, R7) — two
                // literals, chosen at compile time (src/common/robot_words.hpp).
                std::snprintf(why, sizeof why,
                              Words::value.claw ? "port %d is both a claw (%.23s) and a drive motor in the %s -- %s"
                                                : "port %d is both a mechanism (%.23s) and a drive motor in the %s -- %s",
                              port, spec.name, fromRecord ? "record" : "typed table",
                              fromRecord ? (Words::value.claw ? "recalibrate: shulib Calibrate (slot 4) never takes a "
                                                                "declared claw port"
                                                              : "recalibrate: shulib Calibrate (slot 4) never takes a "
                                                                "declared mechanism port")
                                         : "src/robots/chassis_table.hpp and src/robots/mechanism_table.hpp "
                                           "disagree");
            } else {
                std::snprintf(why, sizeof why, "port %d is both %.23s's %.12s and a drive motor in the %s -- %s",
                              port, spec.name, roles[r].word, fromRecord ? "record" : "typed table",
                              fromRecord ? "check its wire(...) line in src/robots/mechanism_table.hpp; "
                                           "then recalibrate (slot 4)"
                                         : "src/robots/chassis_table.hpp and src/robots/mechanism_table.hpp "
                                           "disagree");
            }
            std::snprintf(line, sizeof line, "mechanisms: REFUSED -- %s", why);
            printLine(static_cast<const char*>(line));
            const char* headline = roles[r].motor
                                       ? (Words::value.claw
                                              ? (fromRecord ? "a CLAW port is a DRIVE port in the record"
                                                            : "a CLAW port is a DRIVE port in the typed table")
                                              : (fromRecord ? "a MECHANISM MOTOR port is a DRIVE port in the record"
                                                            : "a MECHANISM MOTOR port is a DRIVE port in the typed table"))
                                       : (fromRecord ? "a MECHANISM SENSOR port is a DRIVE port in the record"
                                                     : "a MECHANISM SENSOR port is a DRIVE port in the typed table");
            detail::refuseWith(out, headline, why);
            return out;
        }
    }

    out.built = bootConfigFrom(*chosen, out.verdict, robotName, mechanisms);
    out.ok = true;
    return out;
}

/// THE BOOT (src/common/boot_source.cpp; the PROS shim): open the card's record and read it, make the
/// brain's registry, turn this build's typed table into a description, and run
/// decideBootSourceFrom() with this build's mechanism list (kMechanisms) — printing each decision
/// line as `[<tag>] …` through hal::pros::serialWrapped() (one serial write() per piece; SER1),
/// BEFORE any adapter exists. Returns the outcome; paints nothing and sets no global.
[[nodiscard]] BootOutcome decideBootSource(const char* tag);

}  // namespace shulib::bench
