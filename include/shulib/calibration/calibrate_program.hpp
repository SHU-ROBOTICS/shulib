#pragma once
//
// calibrate_program — THE STANDARD "shulib Calibrate" (chunk CAL1, Part A2a, 2026-10-18): the robot
// measures its own drivetrain, for ANY team, from the few facts only a person can state.
//
// ── What a team states, and nothing else ────────────────────────────────────────────
// CalibrateFacts: the drive KIND (config::DrivetrainKind — only a tank is measured; anything else
// is refused by name), the drive motors' CARTRIDGE (no reading can tell it: the rotor turns the same
// for every insert), and the port of EVERY motor that is NOT a drive motor (its MECHANISMS — the
// registry reports a motor's kind, never its job). Everything else is discovered on the robot: which
// ports drive, which way each is wired, which side, each side's inches per radian, the effective
// track width, the IMU's port — and THE FRONT, which is the end that LEADS the push (the team lead,
// 2026-10-06). A motor the facts do not list is a drive CANDIDATE: one that turns with the push is
// POWERED (3, then 6, then 9 V) like a drive motor. The program says so wherever it can (its banner
// when nothing is declared, a line per candidate that did not turn, the LEFT OUT and pulse-refusal
// lines), because the facts are the only guard.
//
// ── Where it comes from ─────────────────────────────────────────────────────────────
// This is "shulib Calibrate" — src/programs/calibrate_program.cpp (R3d Part 2F-2, 2026-09-24; SER1's
// one-call serial lines, 2026-10-15) — MOVED: the census, the declared-port drop, the refusals, the
// calibrator, the touch, the serial record and RUN AGAIN are its code, in its order. What changed is
// the CAL1 brief's §1.3 list and nothing else: the words that named robot two are the facts'; the
// front is the push's (the program answers the calibrator's WHICH END question itself, the tick DONE
// is accepted, so that screen is never painted — CAL1 Part B deletes the question); the save is
// SAFE-SAVE B1's (calibration/record_store.hpp); and three things are new — the FRONT CHECK, the
// WATCH of the declared ports, and the two HELD screens. That program stays robot two's until CAL1
// Part B moves robot two onto this one.
//
// ── It owns NO pixel and NO PROS call ───────────────────────────────────────────────
// It paints through calibrate_screen.hpp onto an hal::IBrainScreen, builds its motors and its IMU
// through an hal::IDeviceProvider (the provider asks the registry before it constructs, and catches
// an adapter's throw per device), reads the census from an hal::IDeviceRegistry, reads and saves the
// record through an hal::IFileStore, and reaches the rest — a serial line, the competition status, a
// plain wait, and the watch's read of a declared motor — through CalibrateIo, four calls. So the
// whole program runs on the host test plant (hal::fake), and hal/pros/calibrate.hpp is its thin robot
// shell.
//
// ── The FRONT CHECK, before anything is saved ───────────────────────────────────────
// At DONE (after the MEASURED lines, before any write) the card's record is read the way the boot
// reads it (loadRecord(): the record file, else its backup) and compared with this run's port by port
// (front_check.hpp's compareFronts()). No card: no check (nothing can be saved). A card holding no
// record (neither file opened): the FIRST record, saved with a NOTE (painted only over a save that
// SUCCEEDED: under NOT SAVED it would name a record the card does not hold). Files that opened and do
// not decode: NOT DONE, a line per file naming it and its reason, saved with a NOTE. SAME FRONT: saved.
// CANNOT TELL: saved with a NOTE. FRONT FLIPPED: the save WAITS on the FRONT FLIPPED screen — USE NEW
// (save as pushed: right after a LEFT/RIGHT cable swap or a deliberate change of front) or RUN AGAIN
// (save nothing: right after a wrong push). The records cannot tell those apart, so the screen states
// what the RECORDS say and never what the operator did, and nothing is ever turned to another front:
// every saved record's front is the end that led its push.
// THE TYPED DRIVE PORTS (CAL1 Part A2b, 2026-10-30 — the team lead's D6 (a), 2026-10-08): a team
// whose drive program already TYPES its drive ports may state them (CalibrateFacts::typedDrive,
// optional). With a usable record on the card they change nothing — not a line, not a draw, not a
// byte. With NO usable record on a present card (neither file opened, or files that opened and do
// not decode: the boot treats a record that does not decode as absent, and drives from a valid typed
// table) they are the reference instead: the same verdicts in their own words, FRONT FLIPPED held the
// same way (USE NEW, then fix the typed ports; or RUN AGAIN), and SAME noted as an EXACT match
// (every port in either list agrees) or a partial one (the serial log names each port that differs:
// a typed port with a wrong sign fights its side-mates in anything that drives from them). They are
// COMPARED, never copied into a record: Calibrate never calibrates from them. With no card there is
// no check, typed or not. The words that say what a discarded run leaves follow the CARD's contents,
// never the reference.
// THE LIMIT: the check and the save read the card separately (one program, one task: no race). A
// card that misreads a GOOD record as absent at the check makes this run a FIRST record, saved with
// no flip check — SAFE-SAVE's two-fault limit (HA-139, the belief that a read returns what the card
// holds; record_store.hpp, THE LIMIT). No third read is added to hide it: a third read can misread
// too. So a FIRST RECORD note means "the check READ no record", never "the card held none". (With
// typed drive ports in the facts, such a run is checked against THEM instead — still not against the
// record the card holds.)
//
// ── The WATCH, and the second hold ──────────────────────────────────────────────────
// Every declared mechanism port is READ when the push begins and again at DONE — never commanded,
// never configured: CalibrateIo::readShaftDegrees(), which on the brain is PROS's C reads by port and
// no motor object (hal/pros/calibrate.hpp; its belief is HA-187 in docs/hardware-assumptions.md). One
// that turned as far as the push's own movedThreshold — the calibrator's own test for a drive port,
// so no new threshold — HOLDS the save on MECHANISM PORT TURNED: SAVE (a mechanism the push turns:
// keep it declared) or RUN AGAIN (a drive motor declared by mistake: fix the facts). After a USE NEW
// it follows FRONT FLIPPED: one question per screen. EVERY declared port that turned is its own
// question, on its own screen, in the facts' order — each with ITS turn and whether the card's record
// (or, when they were the reference, the typed drive ports) drives with IT, its boxes greyed again as
// it arrives — and nothing is saved until every one is
// answered SAVE: SAVE is a judgment about one port, so one port's evidence never stands for another's
// (an intake the push legitimately turns must not hide a drive motor declared by mistake). Only an
// ACCEPTED push is judged — a push the calibrator refuses at DONE saves nothing, so the watch says
// nothing about it.
//
// ── Nothing is written while HELD, and every exit leaves the motors at 0 V + Coast ──
// A hold is a SaveOutcome, not a calibrator phase: nothing is powered, and RUN AGAIN, the end of
// driver control or a power-off ends it with nothing written — the words then say what the card
// holds (its record, no usable record, or no card at all), and what ended it (the field's disable,
// or autonomous starting). An RAII guard runs the all-stop on every C++ exit; the competition status
// is polled every tick, and EITHER bit — disabled, or autonomous — exits the loop cleanly (PROS
// deletes the opcontrol task at either change, and a deleted task runs no destructors); a
// PreconditionError inside the loop is caught once at its edge (PROGRAM FAULT: everything stopped,
// said, idled until driver control ends the same way).
//
// NO HARDWARE CLAIM: this program has not run on a robot in this form. The calibrator inside it ran
// on robot two in its R3d Part 2F form on 2026-09-24 (one run stopped, one finished unrecorded).
// Every threshold is INVENTED (A4 register HA-136). PROS-free, sim-free; allocation-free.

#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "shulib/calibration/calibrate_panel.hpp"
#include "shulib/calibration/calibrate_screen.hpp"
#include "shulib/calibration/calibration_record.hpp"
#include "shulib/calibration/drivetrain_calibrator.hpp"
#include "shulib/calibration/front_check.hpp"
#include "shulib/calibration/record_store.hpp"
#include "shulib/config/robot_config.hpp"
#include "shulib/core/check.hpp"
#include "shulib/diag/build_info.hpp"
#include "shulib/hal/brain_screen.hpp"
#include "shulib/hal/device_provider.hpp"
#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/hal/file_store.hpp"
#include "shulib/hal/imu.hpp"
#include "shulib/hal/motor.hpp"
#include "shulib/hal/motor_group.hpp"
#include "shulib/motion/motion_scheduler.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::calibration {

// ═══ WHAT A TEAM STATES ══════════════════════════════════════════════════════════════

/// A motor Calibrate must never power: a mechanism's smart port, and what to call it in the log.
/// LIST EVERY MOTOR THAT IS NOT A DRIVE MOTOR: a motor not listed is a drive CANDIDATE, and one that
/// turns with the push is POWERED (3, then 6, then 9 V) like a drive motor. A 3-wire solenoid is not
/// a motor and is never a candidate.
struct MechanismPort {
    std::uint8_t port = 0;       ///< 1..21; 0 = an unused entry (only if `name` is null too)
    const char* name = nullptr;  ///< what the log and the screen call it; null prints "a mechanism"
};

/// The most mechanism motors a team can declare: every smart port, so a robot is never refused for
/// counting.
inline constexpr std::size_t kMaxMechanismPorts = hal::kMaxSmartPorts;

/// The drive ports a team ALREADY TYPES for its drive program (CAL1 D6 (a), the team lead
/// 2026-10-08) — OPTIONAL: leave them out and nothing changes. When the card holds no usable record,
/// the front check compares this run with THESE instead of with nothing (header). The spelling is
/// the one a typed table already uses (src/robots/chassis_table.hpp's
/// `.left = {-11, 12, -13, 14, -15}`, carried unchanged into hal::DriveSideDescription::signedPorts
/// by the boot's tableToDescription()): a SIGNED port per motor on the side it drives, `-11` = port
/// 11 reversed once in the adapter. So "SAME FRONT as the typed ports" means exactly "a drive built
/// from them turns every motor the way this push measured". 0 = an unused slot (skipped, the order
/// kept). They are compared, never calibrated from.
struct TypedDrivePorts {
    std::int8_t left[hal::kMaxSmartPorts]{};   ///< the LEFT side's drive ports, signed; 0 = unused
    std::int8_t right[hal::kMaxSmartPorts]{};  ///< the RIGHT side's drive ports, signed; 0 = unused
};

/// The facts only a person can state (header). The optional members are UNSET until stated: a
/// value is never invented, and an unset kind or cartridge REFUSES before anything is made or
/// powered. Every member has a default, so a team names only what it states (designated
/// initializers, in this order — `typedDrive` is LAST, so a file that names the others still
/// compiles).
struct CalibrateFacts {
    std::optional<config::DrivetrainKind> drive{};   ///< the kind the team HAS; only Tank is measured
    std::optional<hal::MotorCartridge> cartridge{};  ///< the DRIVE motors' cartridge (insert colour)
    MechanismPort mechanisms[kMaxMechanismPorts]{};  ///< EVERY motor that is not a drive motor
    const char* robot = nullptr;     ///< optional: the robot's name, for the log
    std::uint8_t imuPortStated = 0;  ///< optional cross-check (the registry decides); 0 = none
    /// optional (D6 (a)): the drive ports the team types — the front check's reference when the
    /// card holds no usable record; empty = none
    TypedDrivePorts typedDrive{};
};

/// Which list of CalibrateFacts a FactsProblem's `entry` indexes.
enum class FactsList : std::uint8_t {
    Mechanisms,  ///< CalibrateFacts::mechanisms
    TypedLeft,   ///< CalibrateFacts::typedDrive.left
    TypedRight,  ///< CalibrateFacts::typedDrive.right
};

/// Why a team's facts cannot be used: `why` is null when they can; `entry` is the index of the
/// entry it names, in the list `list` names.
struct FactsProblem {
    const char* why = nullptr;               ///< null = usable
    std::size_t entry = 0;                   ///< the entry it is about
    FactsList list = FactsList::Mechanisms;  ///< the list `entry` indexes
};

/// True when `f` states typed drive ports: any non-zero entry on EITHER side (a list with one side
/// only is then refused by factsProblem(), never run as untyped).
[[nodiscard]] constexpr bool hasTypedDrive(const CalibrateFacts& f) noexcept {
    for (std::size_t i = 0; i < hal::kMaxSmartPorts; ++i) {
        if (f.typedDrive.left[i] != 0 || f.typedDrive.right[i] != 0) {
            return true;
        }
    }
    return false;
}

/// The first reason `f` cannot be used. The mechanism list first: an entry NAMED but with no port (a
/// placeholder `{0, "intake"}` would silently leave that motor a drive candidate), a port outside
/// 1..21, or a port listed twice (the SECOND entry is named). Then, when it states typed drive ports
/// (D6 (a)): a port outside 1..21, a port listed twice — on one side, or once on each, by |port|
/// (the description's own rule) — or only ONE side typed. constexpr, so a build can static_assert
/// its own facts.
[[nodiscard]] constexpr FactsProblem factsProblem(const CalibrateFacts& f) noexcept {
    for (std::size_t i = 0; i < kMaxMechanismPorts; ++i) {
        const int p = f.mechanisms[i].port;
        if (p == 0) {
            if (f.mechanisms[i].name != nullptr) {
                return {"a mechanism is named but its port is 0 -- state its port", i};
            }
            continue;
        }
        if (p > static_cast<int>(hal::kMaxSmartPorts)) {
            return {"a mechanism port is not a smart port (1..21)", i};
        }
        for (std::size_t j = i + 1; j < kMaxMechanismPorts; ++j) {
            if (f.mechanisms[j].port == f.mechanisms[i].port) {
                return {"a mechanism port is listed twice", j};
            }
        }
    }
    if (!hasTypedDrive(f)) {
        return {};
    }
    const std::int8_t* sides[2] = {f.typedDrive.left, f.typedDrive.right};
    constexpr FactsList kLists[2] = {FactsList::TypedLeft, FactsList::TypedRight};
    bool any[2] = {false, false};
    for (std::size_t s = 0; s < 2; ++s) {
        for (std::size_t i = 0; i < hal::kMaxSmartPorts; ++i) {
            const int a = hal::DrivetrainDescription::portMagnitude(sides[s][i]);
            if (a == 0) {
                continue;
            }
            any[s] = true;
            if (a > static_cast<int>(hal::kMaxSmartPorts)) {
                return {"a typed drive port is not a smart port (1..21)", i, kLists[s]};
            }
            for (std::size_t t = s; t < 2; ++t) {
                for (std::size_t j = t == s ? i + 1 : 0; j < hal::kMaxSmartPorts; ++j) {
                    if (hal::DrivetrainDescription::portMagnitude(sides[t][j]) == a) {
                        return {"a typed drive port is listed twice (one side, or both)", j,
                                kLists[t]};
                    }
                }
            }
        }
    }
    if (!any[0]) {
        return {"typed drive ports must list BOTH sides -- the LEFT is empty", 0,
                FactsList::TypedLeft};
    }
    if (!any[1]) {
        return {"typed drive ports must list BOTH sides -- the RIGHT is empty", 0,
                FactsList::TypedRight};
    }
    return {};
}

/// The typed drive ports as a description's two sides — the reference compareFronts() reads when
/// the card holds no usable record (header). Only the two sides are filled (the check reads nothing
/// else); zero slots are skipped and the order kept. Assumes factsProblem(f) passed.
[[nodiscard]] constexpr hal::DrivetrainDescription typedDriveDescription(
    const CalibrateFacts& f) noexcept {
    hal::DrivetrainDescription d{};
    for (const std::int8_t p : f.typedDrive.left) {
        if (p != 0) {
            d.left.signedPorts[d.left.count++] = p;
        }
    }
    for (const std::int8_t p : f.typedDrive.right) {
        if (p != 0) {
            d.right.signedPorts[d.right.count++] = p;
        }
    }
    return d;
}

/// A description's two sides as typed drive ports — for a team whose drive program is a
/// config::RobotConfig: `.typedDrive = typedDrivePortsFrom(kRobot.drivetrain.tank.description)`
/// (the inverse of typedDriveDescription(); the `kRobot` must be a constant expression when the
/// facts are `constexpr`).
[[nodiscard]] constexpr TypedDrivePorts typedDrivePortsFrom(
    const hal::DrivetrainDescription& d) noexcept {
    TypedDrivePorts t{};
    for (std::size_t i = 0; i < d.left.count && i < hal::kMaxSmartPorts; ++i) {
        t.left[i] = d.left.signedPorts[i];
    }
    for (std::size_t i = 0; i < d.right.count && i < hal::kMaxSmartPorts; ++i) {
        t.right[i] = d.right.signedPorts[i];
    }
    return t;
}

/// The declared mechanism entry for physical `port`, or null.
[[nodiscard]] constexpr const MechanismPort* declaredMechanism(const CalibrateFacts& f,
                                                               int port) noexcept {
    for (const MechanismPort& m : f.mechanisms) {
        if (m.port != 0 && static_cast<int>(m.port) == port) {
            return &m;
        }
    }
    return nullptr;
}

// ═══ WHAT THE PROGRAM NEEDS FROM ITS PLATFORM ════════════════════════════════════════

/// The competition status bit "disabled" (PROS's competition_get_status(); hal/pros/calibrate.hpp
/// checks the three against PROS's own values at compile time). Written in hex: the API reference
/// tool reads `<<` as a bracket.
inline constexpr std::uint8_t kCompetitionDisabled = 0x01;
/// The competition status bit "autonomous" — see kCompetitionDisabled.
inline constexpr std::uint8_t kCompetitionAutonomous = 0x02;
/// The competition status bit "field or switch connected" — see kCompetitionDisabled.
inline constexpr std::uint8_t kCompetitionConnected = 0x04;

/// Everything the program reaches outside the library's seams — four calls. On the brain:
/// hal::pros::ProsCalibrateIo (hal/pros/calibrate.hpp); on the host: a test's fake.
class CalibrateIo {
public:
    /// Polymorphic-base boilerplate: a virtual destructor, the copy/move set re-defaulted (the base
    /// holds no state).
    virtual ~CalibrateIo() = default;
    /// Default-constructible.
    CalibrateIo() = default;
    /// Copyable at the base.
    CalibrateIo(const CalibrateIo&) = default;
    /// Movable at the base.
    CalibrateIo(CalibrateIo&&) = default;
    /// Copy-assignable at the base.
    CalibrateIo& operator=(const CalibrateIo&) = default;
    /// Move-assignable at the base.
    CalibrateIo& operator=(CalibrateIo&&) = default;
    /// One serial line, WITHOUT its "[R3D] " tag and newline (the platform adds them). On the
    /// brain it reaches the USB serial as ONE write() per line (SER1).
    virtual void line(const char* text) = 0;
    /// The competition status byte (the kCompetition* bits).
    [[nodiscard]] virtual std::uint8_t competitionStatus() = 0;
    /// Wait `ms` milliseconds — a plain wait (the touch baseline's retry, the fault's idle).
    virtual void delayMs(std::uint32_t ms) = 0;
    /// THE WATCH'S READ: the DECLARED mechanism motor on `port`'s shaft position, degrees, read
    /// WITHOUT configuring or commanding it. False when it cannot be read (then nothing is judged).
    [[nodiscard]] virtual bool readShaftDegrees(int port, double& degrees) = 0;
};

/// The status byte in words, into `buf`: "0x04 field=yes disabled=no auton=no". Returns `buf`.
inline const char* competitionWords(std::uint8_t s, char* buf, std::size_t n) {
    std::snprintf(buf, n, "0x%02x field=%s disabled=%s auton=%s", static_cast<unsigned>(s),
                  (s & kCompetitionConnected) != 0 ? "yes" : "no",
                  (s & kCompetitionDisabled) != 0 ? "yes" : "no",
                  (s & kCompetitionAutonomous) != 0 ? "yes" : "no");
    return buf;
}

/// The devices the program runs on, all BORROWED: each must outlive the program.
struct CalibrateDevices {
    hal::IDeviceProvider& devices;   ///< motors and the IMU by port; the clock; the pacer
    hal::IDeviceRegistry& registry;  ///< what is plugged in, read without opening anything
    hal::IFileStore& card;           ///< the SD card (the record and its backup)
    hal::IBrainScreen& screen;       ///< the brain's screen and touch
    CalibrateIo& io;                 ///< the serial line, the competition status, a wait, the watch
};

namespace detail {

/// The run has ended (Done or Aborted).
[[nodiscard]] inline bool programRunFinished(Phase p) noexcept {
    return p == Phase::Done || p == Phase::Aborted;
}

/// A phase as the serial `phase:` line names it.
[[nodiscard]] inline const char* programPhaseWord(Phase p) noexcept {
    switch (p) {
        case Phase::Idle: return "IDLE";
        case Phase::WaitImu: return "IMU";
        case Phase::PushCapture: return "PUSH";
        case Phase::SideAssign: return "PULSES";
        case Phase::SpinTest: return "SPIN";
        case Phase::Done: return "DONE";
        case Phase::Aborted: return "STOPPED";
    }
    return "?";
}

/// A button as the serial log names it: its LABEL, the words on the screen — so a held screen's
/// left box (today's FRONT FIRST id, relabelled) logs as USE NEW or SAVE.
[[nodiscard]] inline const char* programButtonWord(const PanelButton& b) noexcept {
    return b.label != nullptr ? b.label : "?";
}

/// An answer as the serial `touch:` line names it.
[[nodiscard]] inline const char* programAnswerWord(Answer a) noexcept {
    switch (a) {
        case Answer::Done: return "DONE";
        case Answer::FrontFirst: return "FRONT FIRST";
        case Answer::BackFirst: return "BACK FIRST";
        case Answer::Abort: return "ABORT";
    }
    return "?";
}

/// What the program has printed and decided about ONE run. RUN AGAIN replaces it with a
/// value-initialised one, so a field added here later is reset by construction.
struct ProgramRun {
    int number = 1;
    Phase lastPhase = Phase::Idle;
    std::uint8_t pulsesPrinted = 0;
    std::uint8_t spinsPrinted = 0;
    std::uint8_t excludedPrinted = 0;
    bool outcomePrinted = false;
    SaveOutcome save = SaveOutcome::NotAttempted;
    FrontChoice front = FrontChoice::None;
    bool haveLayout = false;
    std::uint32_t lastLayout = 0;
    HoldReason lastLayoutHold = HoldReason::None;
    int lastLayoutHoldPort = 0;
    // the push's live count, on change
    int lastMoved = -1;
    int lastReady = -1;
    int lastFar = -1;
    double lastPushLogAt = -1.0e9;
    // the loop's timing, while the run is under way
    double lastTickAt = -1.0;
    double worstGap = 0.0;
    std::uint32_t ticks = 0;
    // the watch: each declared port's position when the push began
    bool watchArmed = false;
    double watchBase[kMaxMechanismPorts]{};
    bool watchRead[kMaxMechanismPorts]{};
    // the watch's verdict at DONE: EVERY declared port that turned, in the facts' order, and how far
    // — one MECHANISM PORT TURNED question each
    std::size_t turnedCount = 0;
    int turnedPort[kMaxMechanismPorts]{};
    double turnedDeg[kMaxMechanismPorts]{};
    char note[kPanelLineCap] = "";  // the finished screen's NOTE row (empty = none)
};

/// "PORT 21 KEPT AS A MECHANISM -- see the log" — the widest NOTE a SAVE on ONE MECHANISM PORT
/// TURNED question writes (two digits of port); checked against the SMALL budget below.
inline constexpr std::string_view kProgramWidestKeptNote = "PORT 21 KEPT AS A MECHANISM -- see the log";
static_assert(screenText(kProgramWidestKeptNote, kSmallCols),
              "calibrate_program: the PORT N KEPT NOTE does not fit a SMALL line");
/// "21 PORTS KEPT AS MECHANISMS -- see the log" — the widest NOTE after SAVE on SEVERAL questions
/// (every declared port that turned; the serial log names each); checked against the SMALL budget.
inline constexpr std::string_view kProgramWidestKeptManyNote =
    "21 PORTS KEPT AS MECHANISMS -- see the log";
static_assert(screenText(kProgramWidestKeptManyNote, kSmallCols),
              "calibrate_program: the N PORTS KEPT NOTE does not fit a SMALL line");

/// The push's `push live:` serial line is due: a CHANGED count once at least `minSeconds` have
/// passed since the last line, and an unchanged one once `keepaliveSeconds` have — BOTH edges
/// inclusive (exactly the interval is due). The program passes CalibrateProgram::kPushLogMinSeconds
/// and kPushLogKeepaliveSeconds; a function so its two edges can be landed on exactly in a test.
[[nodiscard]] constexpr bool programPushLogDue(bool changed, double sinceLast, double minSeconds,
                                               double keepaliveSeconds) noexcept {
    return (changed && sinceLast >= minSeconds) || sinceLast >= keepaliveSeconds;
}

/// Driver control has ended: the field disabled the robot, or autonomous started — EITHER bit. The
/// one test the loop and the PROGRAM FAULT idle both use (PROS deletes the opcontrol task at either).
[[nodiscard]] constexpr bool programDriverControlEnded(std::uint8_t status) noexcept {
    return (status & kCompetitionDisabled) != 0 || (status & kCompetitionAutonomous) != 0;
}

/// What ended driver control, in words: "the field disabled the robot" when the disabled bit is
/// set (the brief's words), else "autonomous started" — never a disable that did not happen.
[[nodiscard]] constexpr const char* programDriverControlEndWords(std::uint8_t status) noexcept {
    return (status & kCompetitionDisabled) != 0 ? "the field disabled the robot"
                                                : "autonomous started";
}

}  // namespace detail

/// THE STANDARD CALIBRATE (header). Construct it over the robot's devices and the team's facts and
/// call run() from opcontrol(): it owns its own 10 ms loop, and returns when the field disables the
/// robot or starts autonomous (cleanly, every motor at 0 V + Coast) or when no run can start (a
/// refusal painted and printed before anything is made). Not copyable; one per program.
class CalibrateProgram {
public:
    /// The least time between two `push live:` serial lines while the count changes, seconds.
    static constexpr double kPushLogMinSeconds = 0.25;
    /// The longest time between two `push live:` lines while nothing changes, seconds.
    static constexpr double kPushLogKeepaliveSeconds = 5.0;

    /// `buildStamp` is "<date> <time>" of the build (the caller's __DATE__ " " __TIME__): the
    /// banner prints it whole and the screen's header its time. `facts` is copied; `d` and the
    /// stamp are borrowed and must outlive the program.
    CalibrateProgram(const CalibrateFacts& facts, CalibrateDevices d, const char* buildStamp)
        : facts_{facts}, d_{d}, stamp_{buildStamp != nullptr ? buildStamp : ""},
          buildTime_{}, painter_{d.screen, fillBuildTime(buildTime_, stamp_)} {}

    /// Not copyable: it borrows the robot's devices, and a copy would be a second program
    /// commanding the same motors.
    CalibrateProgram(const CalibrateProgram&) = delete;
    /// Not copy-assignable, for the same reason.
    CalibrateProgram& operator=(const CalibrateProgram&) = delete;

    /// Run calibrations until driver control ends — the field's disable, or autonomous starting —
    /// or return at once when no run can start (a refusal, painted and printed) (header).
    void run() {
        Candidate candidates[hal::kMaxSmartPorts];
        std::size_t n = 0;
        AllStopGuard guard{candidates, &n};  // every C++ exit from here on: 0 V, coast

        // ── 1. the banner: what this is, and what it is NOT ────────────────────────────
        emit("");
        emit("################################################################");
        emit("#  shulib CALIBRATE -- the robot MEASURES ITS OWN DRIVETRAIN");
        emit("#  which ports drive, which way each is wired, which side, inches per radian");
        emit("#  per side, and the EFFECTIVE track width -- then writes them to the SD card.");
        emit("#  It POWERS THE MOTORS. The end that LEADS the push is the FRONT.");
        emit("#  Every threshold in it is INVENTED for a first run (A4 register HA-136).");
        emit("################################################################");
        emitf("BUILD STAMP: %s   <-- must match the build you just uploaded", stamp_);
        {
            // compiledBuildHash() is a string_view: NEVER hand it to %s (the 2026-08-18 data abort).
            const std::string_view hash = diag::compiledBuildHash();
            if (hash.empty()) {
                emit("build hash : [ERROR] MISSING -- the build injected no hash (S18.5)");
            } else {
                emitf("build hash : %.*s", static_cast<int>(hash.size()), hash.data());
            }
        }
        emitf("robot      : %s", facts_.robot != nullptr ? facts_.robot : "(not stated)");
        emitf("record file: /usd/%s (written only after a COMPLETE run, a verified backup %s "
              "first)",
              kRecordFileName, kRecordBackupFileName);
        emitf("opcontrol entry; competition status 0x%02x",
              static_cast<unsigned>(d_.io.competitionStatus()));
        painter_.paintStarting();
        cardLine();

        // ── 2. the census: the brain's own registry, not a typed list ──────────────────
        std::int8_t ports[hal::kMaxSmartPorts]{};
        int imuPort = 0;
        int imuCount = 0;
        emit("STAGE 0  DEVICE CENSUS (registry only; nothing is constructed yet)");
        emit("  PORT  device      (no typed port list is consulted for this)");
        for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
            const hal::DeviceKind kind = d_.registry.pluggedKind(p);
            if (kind == hal::DeviceKind::None) {
                continue;
            }
            emitf("  %4d  %s", p, hal::deviceKindWord(kind));
            if (kind == hal::DeviceKind::Imu) {
                ++imuCount;
                imuPort = p;
            }
            if (kind == hal::DeviceKind::Motor && n < hal::kMaxSmartPorts) {
                ports[n] = static_cast<std::int8_t>(p);  // POSITIVE: the calibrator applies the sign
                ++n;
            }
        }
        emitf("  %u motor(s), %d IMU(s)", static_cast<unsigned>(n), imuCount);

        // ── 3. the stated facts, and the refusals that protect them ────────────────────
        // Every refusal here comes before ANY device is asked for: nothing is made, nothing is
        // powered. The mechanism list is checked BEFORE the drop reads it: the drop's lines are
        // only true of a usable list (a port listed twice would print its second entry as "the
        // census found NO MOTOR there", over a port that HAS one).
        if (const FactsProblem fp = factsProblem(facts_); fp.why != nullptr) {
            char why[192];
            if (fp.list == FactsList::Mechanisms) {
                const MechanismPort& m = facts_.mechanisms[fp.entry];
                std::snprintf(why, sizeof why, "%s: mechanisms[%u] = {%u, \"%.60s\"}", fp.why,
                              static_cast<unsigned>(fp.entry), static_cast<unsigned>(m.port),
                              m.name != nullptr ? m.name : "");
            } else {
                const bool left = fp.list == FactsList::TypedLeft;
                std::snprintf(why, sizeof why, "%s: typedDrive.%s[%u] = %d", fp.why,
                              left ? "left" : "right", static_cast<unsigned>(fp.entry),
                              static_cast<int>(left ? facts_.typedDrive.left[fp.entry]
                                                    : facts_.typedDrive.right[fp.entry]));
            }
            refusal(screen_text::kFactsBad, why, "correct the robot's stated facts and rebuild");
            return;
        }
        const std::size_t droppedDeclared = dropDeclared(ports, n);
        if (!facts_.drive.has_value()) {
            refusal(screen_text::kDriveUnset, "the drive kind is not stated",
                    "state it in the robot's facts (Tank), rebuild, upload");
            return;
        }
        if (*facts_.drive != config::DrivetrainKind::Tank) {
            char why[120];
            std::snprintf(why, sizeof why,
                          "the facts say %s; this procedure measures a TANK drivetrain only",
                          config::drivetrainKindWord(*facts_.drive));
            refusal(screen_text::kNotTank, why,
                    "a holonomic drive's calibration is not built yet -- do not run this on it");
            return;
        }
        if (!facts_.cartridge.has_value()) {
            refusal(screen_text::kCartridgeUnset,
                    "the cartridge cannot be measured from inside; read the insert colour off a "
                    "motor",
                    "state it in the robot's facts, rebuild, upload");
            return;
        }
        const hal::MotorCartridge cartridge = *facts_.cartridge;
        if (cartridge != hal::MotorCartridge::Red && cartridge != hal::MotorCartridge::Green
            && cartridge != hal::MotorCartridge::Blue) {
            // A byte outside the enum is no cartridge: refused here, by name, rather than as the
            // calibrator's precondition — which would throw out of run(), past every screen.
            char why[120];
            std::snprintf(why, sizeof why,
                          "the stated cartridge (byte %d) is not RED, GREEN or BLUE",
                          static_cast<int>(cartridge));
            refusal(screen_text::kCartridgeUnset, why,
                    "state RED, GREEN or BLUE in the robot's facts, rebuild, upload");
            return;
        }
        emitf("CARTRIDGE BELIEF: %s -- stated by a human and UNVERIFIABLE from inside (every "
              "cartridge spins the rotor the same)",
              hal::cartridgeWord(cartridge));
        if (imuCount != 1) {
            char why[140];
            std::snprintf(why, sizeof why,
                          "the registry reports %d IMUs; this procedure needs exactly one",
                          imuCount);
            refusal(screen_text::kImuCount, why, "plug in exactly one inertial sensor and run again");
            return;
        }
        if (facts_.imuPortStated != 0 && static_cast<int>(facts_.imuPortStated) != imuPort) {
            // A WARNING, not a refusal (ruling P1): a re-plugged IMU is not an error, and the
            // calibration is taken against wherever the IMU IS.
            emitf("WARNING: the robot's facts say IMU port %u, the registry says port %d -- using "
                  "the REGISTRY (a stated port is a cross-check, not the truth)",
                  static_cast<unsigned>(facts_.imuPortStated), imuPort);
        }
        if (facts_.imuPortStated != 0) {
            emitf("IMU        : port %d, DISCOVERED from the registry (stated %u)", imuPort,
                  static_cast<unsigned>(facts_.imuPortStated));
        } else {
            emitf("IMU        : port %d, DISCOVERED from the registry", imuPort);
        }
        if (n < 2) {
            char why[140];
            std::snprintf(why, sizeof why,
                          "the registry found %u motor(s) besides the declared mechanisms; a "
                          "drivetrain needs at least two",
                          static_cast<unsigned>(n));
            char fix[160] = "re-seat the drive cables; the census is on the serial log";
            if (droppedDeclared > 0) {
                declaredPortsFix(fix, sizeof fix);
            }
            refusal(screen_text::kTooFewMotors, why, fix);
            return;
        }

        // ── 4. one motor per port, POSITIVE, through the provider ──────────────────────
        // The provider asks the registry before it constructs and catches an adapter's refusal per
        // device; a port that refuses is dropped with a WARNING naming it (a calibration of nine
        // motors is still a calibration — and a silently missing motor is how a side comes up
        // short).
        n = buildMotors(ports, n, cartridge, candidates);
        if (n < 2) {
            refusal(screen_text::kMotorsRefused,
                    "fewer than two motor adapters would construct; see the WARNING lines",
                    "re-seat the drive cables; the census is on the serial log");
            return;
        }
        stopAll(candidates, n);  // 0 V + Coast is the DECLARED starting state, not an inherited one

        hal::IMotor* motorPtrs[hal::kMaxSmartPorts]{};
        for (std::size_t i = 0; i < n; ++i) {
            motorPtrs[i] = candidates[i].motor;
        }

        // ── 5. the IMU (its calibration starts as the provider makes it), the calibrator ──
        hal::IImu* imu = d_.devices.imu(static_cast<std::uint8_t>(imuPort));
        if (imu == nullptr) {
            refusal(screen_text::kImuRefused, d_.devices.lastRefusal(),
                    "check the sensor and its cable");
            return;
        }
        emit("IMU calibrating (HA-23 says ~2 s) -- keep the robot STILL until the screen says to "
             "push it");
        hal::IClock& clock = d_.devices.clock();
        char provenance[96];
        std::snprintf(provenance, sizeof provenance, "shulib Calibrate %s", stamp_);
        HumanFacts facts{};
        facts.kind = hal::DrivetrainKind::Tank;
        facts.cartridge = cartridge;
        facts.imuPort = static_cast<std::uint8_t>(imuPort);
        facts.provenance = provenance;

        const std::span<hal::IMotor* const> motorSpan{motorPtrs, n};
        const std::span<const std::int8_t> portSpan{ports, n};
        std::optional<DrivetrainCalibrator> cal;
        startRun(cal, motorSpan, portSpan, *imu, clock, facts);
        emitf("calibrator started: %u candidate(s), kind Tank, cartridge %s, IMU port %d, "
              "provenance \"%s\"",
              static_cast<unsigned>(n), hal::cartridgeWord(cartridge), imuPort, provenance);
        emitConfig(cal->config());
        emit("ONE PUSH, 48 in, FRONT END FIRST, along a tape -- the end that leads is the FRONT;");
        emit("then STAND CLEAR: one motor at a time jerks (3, then 6, then 9 V), then it spins");
        emit("both ways. INVENTED numbers.");
        watchIntro();

        // ── 6. the touch baseline, from a GOOD read ───────────────────────────────────
        std::int32_t touchBase = 0;
        bool touchBaseKnown = false;
        for (int i = 0; i < 20 && !touchBaseKnown; ++i) {
            const TouchSample s = readTouch();
            if (!s.error) {
                touchBase = s.releaseCount;
                touchBaseKnown = true;
            } else {
                d_.io.delayMs(5);
            }
        }
        if (!touchBaseKnown) {
            emit("WARNING: every touch read at start was an ERROR read -- the first good read will "
                 "set the baseline and is never acted on");
        }
        std::optional<TouchGate> gate;
        gate.emplace(touchBase);

        // ── 7. the loop ──────────────────────────────────────────────────────────────
        motion::ITickPacer& pacer = d_.devices.pacer();
        detail::ProgramRun run{};
        int runNumber = 1;
        bool stoppedByField = false;
        held_ = {};
        emit("===== RUN 1 =====");
        try {
            while (true) {
                // A field DISABLE or the start of AUTONOMOUS exits this loop cleanly so the guard
                // runs before PROS kills the task — either bit, with or without the "connected"
                // bit a field or switch adds. Polled every tick: 10 ms is the most the robot can
                // drive past it. A finished run is not aborted (abort() from Done would discard
                // the result, and nothing is powered) — the loop simply ends. The words name what
                // happened: a disable, or autonomous (never a disable that did not happen).
                const std::uint8_t status = d_.io.competitionStatus();
                if (detail::programDriverControlEnded(status)) {
                    if (!detail::programRunFinished(cal->phase())) {
                        cal->abort("the field disabled the robot, or autonomous started");
                        emitf("===== RUN %d ENDED: STOPPED =====", run.number);
                        finishStopped(*cal, run);
                    } else if (held_.held) {
                        emitf("%s on the held %s screen -- NOTHING SAVED: %s",
                              detail::programDriverControlEndWords(status),
                              held_.reason == HoldReason::FrontFlipped ? "FRONT FLIPPED"
                                                                       : "MECHANISM PORT TURNED",
                              cardKeeps());
                    } else {
                        emitf("%s after the run had finished -- nothing was powered",
                              detail::programDriverControlEndWords(status));
                    }
                    stoppedByField = true;
                    break;
                }

                const double now = clock.now().value();
                if (!detail::programRunFinished(cal->phase())) {
                    if (run.lastTickAt >= 0.0 && now - run.lastTickAt > run.worstGap) {
                        run.worstGap = now - run.lastTickAt;
                    }
                    run.lastTickAt = now;
                    ++run.ticks;
                }
                cal->tick();
                record(*cal, run, now);  // the serial record — and, at Done, the check and the save

                // ── paint the view, then read the touch against THAT view ──
                PanelView drawn = gate->view(*cal, clock.now().value(), run.save, FrontRule::ByPush,
                                             finishFacts(run));
                painter_.render(drawn, now);
                recordLayout(drawn, run);

                const TouchSample sample = readTouch();
                if (!touchBaseKnown) {
                    if (!sample.error) {
                        gate.emplace(sample.releaseCount);  // the baseline; this read is not acted on
                        touchBaseKnown = true;
                    }
                } else {
                    const TouchResult r = gate->touch(sample, drawn, *cal);
                    if (r.newRelease) {
                        logTouch(drawn, sample, r);
                    }
                    if (r.action == TouchAction::Answer) {
                        cal->answer(r.answer);
                        if (r.answer == Answer::Done) {
                            // The end that LED the push IS the front (the team lead, 2026-10-06):
                            // the calibrator still asks WHICH END after DONE, so the program
                            // answers it here, before any view is painted. CAL1 Part B deletes the
                            // question, and this answer with it.
                            if (cal->accepts(Answer::FrontFirst)) {
                                cal->answer(Answer::FrontFirst);
                            }
                            emitPushTable(*cal);
                            if (cal->phase() == Phase::SideAssign) {  // the push was accepted
                                emitStillCandidates(*cal);
                                watchReport(*cal, run);
                                emitFront(*cal);
                            }
                        }
                        record(*cal, run, now);  // a refusal at DONE, or the abort: its outcome now
                        // The step changed under the operator's finger: the new view goes up NOW,
                        // not on the next tick — its new buttons greyed for the arming delay.
                        drawn = gate->view(*cal, clock.now().value(), run.save, FrontRule::ByPush,
                                           finishFacts(run));
                        painter_.render(drawn, now);
                        recordLayout(drawn, run);
                    } else if (r.action == TouchAction::SaveHeld) {
                        resolveHeld(run);
                        drawn = gate->view(*cal, clock.now().value(), run.save, FrontRule::ByPush,
                                           finishFacts(run));
                        painter_.render(drawn, now);
                        recordLayout(drawn, run);
                    } else if (r.action == TouchAction::RunAgain) {
                        // RUN AGAIN: a NEW calibrator in place, a NEW gate based on the release
                        // that asked for it, a NEW per-run record. Nothing of the last run
                        // survives; a held run is discarded unsaved. The IMU is already
                        // calibrated, so the IMU step passes at once.
                        if (held_.held) {
                            emitf("RUN AGAIN on a held screen -- NOTHING SAVED: this run is "
                                  "discarded, %s",
                                  cardKeeps());
                        }
                        ++runNumber;
                        emit("");
                        emitf("===== RUN %d -- RUN AGAIN: a NEW calibrator, nothing carried over "
                              "=====",
                              runNumber);
                        stopAll(candidates, n);
                        startRun(cal, motorSpan, portSpan, *imu, clock, facts);
                        gate.emplace(sample.releaseCount);
                        run = detail::ProgramRun{};
                        run.number = runNumber;
                        held_ = {};
                        painter_.invalidate();
                    }
                }
                pacer.pace();
            }
        } catch (const PreconditionError& e) {
            // A broken contract inside the loop is a program bug. Stop everything, say so on the
            // screen and the log, and idle — never a frozen screen over a running motor.
            stopAll(candidates, n);
            emitf("PROGRAM FAULT (a precondition broke inside the loop): %s", e.what());
            emit("every motor is at 0 V, coasting. This is a bug in the program, not the robot.");
            painter_.paintCannot(screen_text::kHeaderStopped, PanelTone::Red,
                                 screen_text::kProgramFault, e.what(),
                                 "a bug in the program, not the robot: keep the serial log",
                                 screen_text::kFaultIdle);
            // Idle until driver control ends — by the loop's own test, so a fault returns on
            // autonomous as the loop does, not only on a disable.
            while (!detail::programDriverControlEnded(d_.io.competitionStatus())) {
                d_.io.delayMs(250);
            }
            return;
        }

        // ── 8. the field ended it ────────────────────────────────────────────────────
        stopAll(candidates, n);
        if (stoppedByField) {
            char bits[64];
            competitionWords(d_.io.competitionStatus(), bits, sizeof bits);
            char line[96];
            std::snprintf(line, sizeof line, "status bits %s", bits);
            emitf("competition state changed: %s -- calibration runs ONLY in driver control, "
                  "enabled",
                  line);
            painter_.paintNotDriverControl(line);
            emit("STOPPED by the field/competition state -- every motor at 0 V, coast.");
        }
    }

private:
    /// What kind of run the program starts (CAL1 §1.6): today only the drivetrain's. R5a Part C
    /// appends GAINS and REST — append-only, and run() switches on it where a run starts and where
    /// its outcome is printed, so a new kind is a compile error until both are written.
    enum class RunKind : std::uint8_t { Drivetrain = 1 };

    /// One candidate: the physical port the registry reported, and the motor the provider made on
    /// it with a POSITIVE port number (the calibrator applies the sign — it is what the run
    /// MEASURES).
    struct Candidate {
        int port = 0;
        hal::IMotor* motor = nullptr;
    };

    /// The all-stop on every C++ exit path (header: a task kill is not one of them).
    struct AllStopGuard {
        Candidate* c;
        const std::size_t* n;
        ~AllStopGuard() { stopAll(c, *n); }
    };

    /// A run whose save waits for the operator: FRONT FLIPPED, then ONE MECHANISM PORT TURNED
    /// question per declared port that turned (the facts' order) — nothing is saved until the last
    /// is answered SAVE.
    struct Held {
        bool held = false;
        HoldReason reason = HoldReason::None;  // the screen showing now
        std::size_t mechCount = 0;             // MECHANISM PORT TURNED questions in all
        std::size_t mechAsked = 0;             // asked so far: the one showing is mechAsked - 1
        bool onCardOf[kMaxMechanismPorts]{};   // per question: the REFERENCE drives with its port
        int port = 0;                          // the declared port the question SHOWING is about
        bool onCard = false;                   // the reference drives with THAT port
        bool typedRef = false;                 // D6 (a): the reference was the TYPED drive ports
        bool cardHadRecord = false;            // the card held a usable record at DONE
        bool cardPresent = true;               // a card was in the brain at DONE
        double driveDeg = 0.0;                 // the drive's median push travel, degrees
        char line1[64] = "";
        char line2[64] = "";
        hal::DrivetrainDescription measured{};
    };

    /// 0 V + Coast on every made candidate. The calibrator does this itself on every refusal and
    /// on abort(); this is the program's own copy, for the exits that never reach the calibrator.
    static void stopAll(Candidate* c, std::size_t n) {
        for (std::size_t i = 0; i < n; ++i) {
            if (c[i].motor != nullptr) {
                c[i].motor->setVoltage(units::Voltage{0.0});
                c[i].motor->setBrakeMode(hal::BrakeMode::Coast);
            }
        }
    }

    /// "BUILD hh:mm:ss" from a "<date> <time>" stamp (its last eight characters), into the
    /// program's own buffer — never a static shared between programs. Returns `out`.
    static const char* fillBuildTime(char (&out)[24], const char* stamp) {
        const std::size_t len = stamp != nullptr ? std::strlen(stamp) : 0;
        std::snprintf(out, sizeof out, "BUILD %s", len >= 8 ? stamp + len - 8 : "");
        return out;
    }

    void emit(const char* s) { d_.io.line(s); }
    [[gnu::format(printf, 2, 3)]] void emitf(const char* fmt, ...) {
        char buf[240];
        va_list args;
        va_start(args, fmt);
        std::vsnprintf(buf, sizeof buf, fmt, args);
        va_end(args);
        d_.io.line(buf);
    }

    TouchSample readTouch() {
        const hal::ScreenTouch t = d_.screen.touch();
        TouchSample s;
        s.error = t.error;
        s.releaseCount = t.releaseCount;
        s.x = t.x;
        s.y = t.y;
        return s;
    }

    /// Refused before any run could start: the screen, and the same words on serial.
    void refusal(std::string_view headline, const char* why, const char* fix) {
        painter_.paintCannot(screen_text::kHeaderCannot, PanelTone::Red, headline, why, fix,
                             screen_text::kNothingPowered);
        emitf("REFUSED: %.*s -- %s", static_cast<int>(headline.size()), headline.data(), why);
        emitf("  %s", fix);
    }

    /// A DECLARED MECHANISM PORT IS NEVER A DRIVE CANDIDATE: dropped from `ports` (in place, the
    /// census order kept) before any device is asked for — never geared, never set to Coast (a
    /// loaded claw would drop), never pulsed. One line per declared port: dropped, or not found.
    /// It drops PORTS: a mechanism cable swapped with a drive cable at the brain is invisible to
    /// the registry, and to this — the WATCH is the catch for that (header). Returns how many were
    /// dropped.
    std::size_t dropDeclared(std::int8_t* ports, std::size_t& n) {
        bool seen[kMaxMechanismPorts]{};
        std::size_t kept = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const std::int8_t p = ports[i];
            if (const MechanismPort* m = declaredMechanism(facts_, p); m != nullptr) {
                emitf("port %d: %s (declared) -- not a candidate, not touched", static_cast<int>(p),
                      m->name != nullptr ? m->name : "a mechanism");
                seen[static_cast<std::size_t>(m - facts_.mechanisms)] = true;
            } else {
                ports[kept++] = p;
            }
        }
        for (std::size_t i = kept; i < n; ++i) {
            ports[i] = 0;
        }
        const std::size_t dropped = n - kept;
        n = kept;
        for (std::size_t k = 0; k < kMaxMechanismPorts; ++k) {
            const MechanismPort& m = facts_.mechanisms[k];
            if (m.port != 0 && !seen[k]) {
                emitf("port %d: %s (declared) -- the census found NO MOTOR there",
                      static_cast<int>(m.port), m.name != nullptr ? m.name : "a mechanism");
            }
        }
        return dropped;
    }

    /// TOO FEW MOTORS' fix when declared ports were dropped: name them — a drive motor declared as
    /// a mechanism is the other way a robot comes up short.
    void declaredPortsFix(char* fix, std::size_t cap) {
        int w = std::snprintf(fix, cap, "declared as mechanisms:");
        std::size_t used = w > 0 ? static_cast<std::size_t>(w) : 0;
        for (const MechanismPort& m : facts_.mechanisms) {
            if (m.port == 0 || d_.registry.pluggedKind(m.port) != hal::DeviceKind::Motor) {
                continue;
            }
            w = std::snprintf(fix + used, cap - used, " %u", static_cast<unsigned>(m.port));
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < cap ? used : cap - 1;
        }
        std::snprintf(fix + used, cap - used,
                      " -- is any a DRIVE motor? else re-seat the drive cables");
    }

    /// One motor per port from `ports` (POSITIVE), with `cartridge`, through the provider — THE
    /// motor construction (R5a Part C calls it with a record's ports). A port the provider refuses
    /// is dropped with a WARNING naming it; `ports` is compacted to the made ones. Returns how many
    /// were made.
    std::size_t buildMotors(std::int8_t* ports, std::size_t n, hal::MotorCartridge cartridge,
                            Candidate* out) {
        std::size_t built = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const int port = ports[i];
            hal::IMotor* m = d_.devices.motor(static_cast<std::int8_t>(port), cartridge);
            if (m == nullptr) {
                emitf("WARNING: port %d answered MOTOR in the registry but could not be made: %s "
                      "-- dropped from the candidates",
                      port, d_.devices.lastRefusal());
                continue;
            }
            out[built].port = port;
            out[built].motor = m;
            ports[built] = static_cast<std::int8_t>(port);
            ++built;
        }
        return built;
    }

    /// A new calibrator, started: the first run, and every RUN AGAIN (there is no reset()).
    void startRun(std::optional<DrivetrainCalibrator>& cal, std::span<hal::IMotor* const> motors,
                  std::span<const std::int8_t> ports, hal::IImu& imu, hal::IClock& clock,
                  const HumanFacts& facts) {
        switch (kind_) {
            case RunKind::Drivetrain:
                cal.emplace(motors, ports, imu, clock);
                cal->start(facts);
                return;
        }
    }

    /// What a discarded held run leaves on the card, in words: "the card keeps its record" only
    /// when it held one (C2) — and with no card, that there is none (C3).
    [[nodiscard]] const char* cardKeeps() const noexcept {
        if (!held_.cardPresent) {
            return "there is no SD card in the brain";
        }
        return held_.cardHadRecord ? "the card keeps its record"
                                   : "the card is unchanged (it holds no usable record)";
    }

    /// The card's record before any run, one line — so the operator knows before pushing whether a
    /// front check will run (the check re-reads it at DONE). No card is not "no record".
    void cardLine() {
        if (!d_.card.cardPresent()) {
            emit("card       : NO SD CARD in the brain -- nothing can be saved, and the FRONT is not "
                 "checked");
            return;
        }
        const LoadedRecord rec = loadRecord(d_.card);
        if (rec.from == Which::None) {
            if (hasTypedDrive(facts_)) {  // D6 (a): the typed drive ports stand in
                emit("card       : no usable record yet -- this run's FRONT is checked against "
                     "your typed drive ports");
                return;
            }
            emit("card       : no usable record yet -- this run's push decides the FRONT");
            return;
        }
        char described[128];
        rec.description.describe(described, sizeof described);
        emitf("card       : %s holds %s -- this run's FRONT is checked against it",
              rec.from == Which::Primary ? kRecordFileName : kRecordBackupFileName, described);
    }

    /// One line per record file that OPENED and did not decode, naming it: never an empty reason (a
    /// backup-only card, an EMPTY file) and never a long one cut inside another line.
    void emitUndecodedFiles(const LoadedRecord& r) {
        const RecordFileRead* files[2] = {&r.primary, &r.backup};
        const char* names[2] = {kRecordFileName, kRecordBackupFileName};
        for (std::size_t i = 0; i < 2; ++i) {
            const RecordFileRead& f = *files[i];
            if (!f.opened || f.decoded) {
                continue;
            }
            if (f.count == 0) {
                emitf("  %s: EMPTY (0 bytes read)", names[i]);
            } else {
                emitf("  %s: %s", names[i], f.why[0] != '\0' ? f.why : "did not decode");
            }
        }
    }

    /// SAME against the TYPED ports, but not exactly (the CAL1 check of D6 (a), C4): every port where
    /// they and this push's record differ, one line each, so a team can fix the typed port that is
    /// wrong — a typed port with a wrong sign fights its side-mates in anything that still drives
    /// from them, so "matches" alone would hide it.
    void emitTypedDifferences(const hal::DrivetrainDescription& typed,
                              const hal::DrivetrainDescription& d, unsigned differing) {
        emitf("front check: SAME FRONT, but your typed drive ports and this push differ at %u "
              "port(s) -- check each before anything drives from the typed ports:",
              differing);
        const auto sideWord = [](int side) { return side < 0 ? "LEFT" : "RIGHT"; };
        for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
            const int st = typed.sideOf(p);
            const int sn = d.sideOf(p);
            if (st == 0 && sn == 0) {
                continue;
            }
            const int vt = typed.signedPortOf(p);
            const int vn = d.signedPortOf(p);
            if (sn == 0) {
                emitf("  port %d: typed %s %+d, not in this push's record (LEFT OUT, declared, or "
                      "did not turn)",
                      p, sideWord(st), vt);
            } else if (st == 0) {
                emitf("  port %d: this push %s %+d, not in your typed drive ports", p, sideWord(sn),
                      vn);
            } else if (st == sn && (vt < 0) != (vn < 0)) {
                emitf("  port %d: typed %s %+d, this push %s %+d -- the SIGN differs", p,
                      sideWord(st), vt, sideWord(sn), vn);
            } else if (st != sn && (vt < 0) == (vn < 0)) {
                emitf("  port %d: typed %s %+d, this push %s %+d -- the SIDE differs", p,
                      sideWord(st), vt, sideWord(sn), vn);
            } else if (st != sn) {
                emitf("  port %d: typed %s %+d, this push %s %+d -- the side AND the sign differ", p,
                      sideWord(st), vt, sideWord(sn), vn);
            }
        }
    }

    void watchIntro() {
        bool any = false;
        for (const MechanismPort& m : facts_.mechanisms) {
            any = any || m.port != 0;
        }
        if (!any) {
            emit("watch      : no mechanism port declared -- nothing to watch, and EVERY motor is a");
            emit("             drive candidate: one that turns with the push is POWERED (3, 6, 9 V).");
            emit("             Declare every mechanism motor in the robot's facts, or unplug it first.");
            return;
        }
        emit("watch      : the declared mechanism ports are READ during the push, never commanded;");
        emit("             one that turns with the push is named at DONE, and the save waits");
    }

    /// The watch's baseline, when the push begins: every declared port read once, nothing written.
    void watchArm(detail::ProgramRun& run) {
        run.watchArmed = true;
        for (std::size_t k = 0; k < kMaxMechanismPorts; ++k) {
            const MechanismPort& m = facts_.mechanisms[k];
            run.watchRead[k] = m.port != 0 && d_.io.readShaftDegrees(m.port, run.watchBase[k]);
        }
    }

    /// The drive motors' median push travel, degrees of shaft (the watch's comparison).
    static double driveDeg(const DrivetrainCalibrator& cal) {
        return cal.status().pushTravelRad / kDegreesToRadians;
    }

    /// The watch's verdict, at DONE: a declared port that turned as far as the push's own
    /// threshold is named, with the drive's travel beside it — the operator judges; no ratio is
    /// invented. "Turned" is the calibrator's own test for a drive port (movedOnPush(): a FINITE
    /// travel whose size REACHES movedThreshold), so a reading that is not a number is no reading,
    /// and exactly the threshold counts — never a second rule beside the first. EVERY port that
    /// turned is kept, in the facts' order: each becomes its own question.
    void watchReport(const DrivetrainCalibrator& cal, detail::ProgramRun& run) {
        if (!run.watchArmed) {
            return;
        }
        run.watchArmed = false;  // once per push
        const double threshold = cal.config().movedThreshold.value() / kDegreesToRadians;
        for (std::size_t k = 0; k < kMaxMechanismPorts; ++k) {
            const MechanismPort& m = facts_.mechanisms[k];
            if (m.port == 0) {
                continue;
            }
            const char* name = m.name != nullptr ? m.name : "a mechanism";
            double now = 0.0;
            const bool read = run.watchRead[k] && d_.io.readShaftDegrees(m.port, now);
            const double moved = now - run.watchBase[k];
            if (!read || !hal::isUsableReading(moved)) {
                emitf("watch: port %d (%s) could not be read -- not judged",
                      static_cast<int>(m.port), name);
                continue;
            }
            if (std::abs(moved) >= threshold) {
                emitf("WARNING: port %d is declared as %s but TURNED WITH THE PUSH (%+.1f deg; the "
                      "drive turned %.0f deg) -- the save waits for you",
                      static_cast<int>(m.port), name, moved, driveDeg(cal));
                run.turnedPort[run.turnedCount] = m.port;  // at most one entry per facts slot
                run.turnedDeg[run.turnedCount] = moved;
                ++run.turnedCount;
            } else {
                emitf("watch: port %d (%s) did not turn with the push (%+.1f deg)",
                      static_cast<int>(m.port), name, moved);
            }
        }
    }

    /// The config the run uses, once — so a capture can be read against the thresholds that
    /// produced it (every one INVENTED, HA-136).
    void emitConfig(const CalibratorConfig& c) {
        constexpr double kDeg = kDegreesToRadians;
        emitf("config: push %.0f in along a tape; a port MOVED past %.1f deg of shaft; the push "
              "straight within %.1f deg and at least %.2f rad (one wheel turn) of median travel",
              c.pushDistance.value(), c.movedThreshold.value() / kDeg,
              c.straightBound.value() / kDeg, c.minPushRad.value());
        char levels[48] = "";
        std::size_t used = 0;
        for (std::size_t k = 0; k < c.pulseLevelCount && k < kMaxPulseLevels; ++k) {
            const int w = std::snprintf(levels + used, sizeof levels - used, "%s%.1f",
                                        k > 0 ? "/" : "", c.pulseLevels[k].value());
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < sizeof levels ? used : sizeof levels - 1;
        }
        emitf("config: pulses ramp %s V for %.2f s each, a side needs %.1f deg of yaw; a port "
              "whose own shaft turns %.0f deg with nothing else moving is LEFT OUT; rest = motors "
              "under %.2f rad/s AND yaw rate under %.1f deg/s for %.1f s",
              levels, c.pulseTime.value(), c.pulseYawFloor.value() / kDeg,
              c.decoupledOwnMin.value() / kDeg, c.restFloor.value(),
              c.restYawRateFloor.value() / kDeg, c.restSettleTime.value());
        emitf("config: spin %.0f turns each way, levels %.1f then %.1f V; must turn %.1f deg "
              "within %.1f s per level; STOPS at %.1f deg the wrong way; %.0f s timeout; the two "
              "ways agree within %.1f pct",
              c.spinTurns, c.spinLevels[0].value(),
              c.spinLevels[c.spinLevelCount > 1 ? 1 : 0].value(), c.spinStartYaw.value() / kDeg,
              c.spinStartTime.value(), c.spinWrongWay.value() / kDeg, c.spinTimeout.value(),
              100.0 * c.spinAgreement);
    }

    /// The PUSH TABLE, right after DONE — whatever the calibrator made of it (accepted, or refused
    /// as too few / curved / too short: the table is then the diagnosis).
    void emitPushTable(const DrivetrainCalibrator& cal) {
        const CalibrationReport& rep = cal.report();
        const CalibratorConfig& cfg = cal.config();
        emitf("PUSH TABLE at DONE (travel AS READ; the end that led the push is the FRONT; MOVED "
              "= past %.1f deg):",
              cfg.movedThreshold.value() / kDegreesToRadians);
        for (std::size_t i = 0; i < rep.portCount; ++i) {
            char line[96];
            formatPushRecord(rep.ports[i], line, sizeof line);
            emitf("  %s", line);
        }
        emitf("push: %u of %u candidates moved, drift %+.2f deg (bound %.2f), median travel %.2f "
              "rad (need %.2f)",
              static_cast<unsigned>(rep.drivePortCount), static_cast<unsigned>(rep.portCount),
              rep.pushStraightnessRad / kDegreesToRadians + 0.0,
              cfg.straightBound.value() / kDegreesToRadians, cal.status().pushTravelRad,
              cfg.minPushRad.value());
    }

    /// At an accepted DONE: every candidate that did NOT turn with the push is named — it is not a
    /// drive motor and is never powered; if it is a mechanism, it belongs in the facts.
    void emitStillCandidates(const DrivetrainCalibrator& cal) {
        const CalibrationReport& rep = cal.report();
        for (std::size_t i = 0; i < rep.portCount; ++i) {
            if (rep.ports[i].sign != 0) {
                continue;
            }
            emitf("port %d did not turn with the push: not a drive motor, never powered -- if it is "
                  "a mechanism, declare it in the robot's facts",
                  static_cast<int>(rep.ports[i].port));
        }
    }

    /// The front, and every drive port with the sign it carries.
    void emitFront(const DrivetrainCalibrator& cal) {
        const CalibrationReport& rep = cal.report();
        emit("front: the end that LED the push is the FRONT -- the signs stand as captured");
        char ports[hal::kMaxSmartPorts * 5 + 1] = "";
        std::size_t used = 0;
        for (std::size_t i = 0; i < rep.portCount; ++i) {
            if (rep.ports[i].sign == 0) {
                continue;
            }
            const int w = std::snprintf(ports + used, sizeof ports - used, " %+d",
                                        static_cast<int>(rep.ports[i].sign * rep.ports[i].port));
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < sizeof ports ? used : sizeof ports - 1;
        }
        emitf("drive ports, signed:%s  (%u to pulse, one at a time)", ports,
              static_cast<unsigned>(rep.drivePortCount));
    }

    /// One side of a description as a signed list: " -11 +12 -13".
    static void signedList(const hal::DriveSideDescription& s, char* out, std::size_t cap) {
        out[0] = '\0';
        std::size_t used = 0;
        for (std::size_t i = 0; i < s.count; ++i) {
            const int w = std::snprintf(out + used, cap - used, " %+d",
                                        static_cast<int>(s.signedPorts[i]));
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < cap ? used : cap - 1;
        }
    }

    void emitLoopStats(const detail::ProgramRun& run) {
        emitf("loop: %lu ticks, worst gap between two ticks %.0f ms (a powered step stops past "
              "100 ms); %lu screen prints so far",
              static_cast<unsigned long>(run.ticks), 1000.0 * run.worstGap,
              static_cast<unsigned long>(d_.screen.prints()));
    }

    /// DONE: the MEASURED lines, the FRONT CHECK, and — unless the run must wait for the operator
    /// (FRONT FLIPPED, then MECHANISM PORT TURNED) — the SAFE SAVE. Called once per run, before the
    /// finished screen is first painted, so it never shows MEASURED and then changes its mind.
    SaveOutcome finishDone(const DrivetrainCalibrator& cal, detail::ProgramRun& run) {
        const hal::DrivetrainDescription& d = cal.result();
        const CalibrationReport& rep = cal.report();
        char described[128];
        d.describe(described, sizeof described);
        emit(described);  // "MEASURED tank BLUE 600 rpm: L5/R5 ports, ipr L... R..., W ... in"
        emitf("provenance: %s", d.provenance);
        char list[hal::kMaxSmartPorts * 5 + 1];
        signedList(d.left, list, sizeof list);
        emitf("  LEFT  (signed, AS MEASURED):%s  (%u ports)", list,
              static_cast<unsigned>(d.left.count));
        signedList(d.right, list, sizeof list);
        emitf("  RIGHT (signed, AS MEASURED):%s  (%u ports)", list,
              static_cast<unsigned>(d.right.count));
        if (rep.excludedCount == 0) {
            emit("  LEFT OUT: none");
        } else {
            char out[hal::kMaxSmartPorts * 4 + 1] = "";
            std::size_t used = 0;
            for (std::size_t k = 0; k < rep.excludedCount; ++k) {
                const int w = std::snprintf(out + used, sizeof out - used, " %d",
                                            static_cast<int>(rep.excludedPorts[k]));
                used += w > 0 ? static_cast<std::size_t>(w) : 0;
                used = used < sizeof out ? used : sizeof out - 1;
            }
            emitf("  LEFT OUT:%s -- in no side of the description (%s)", out, cal.warning());
        }
        emitf("  inches per radian: LEFT %.6f  RIGHT %.6f  (L/R %.4f)",
              d.left.inchesPerRadian.value(), d.right.inchesPerRadian.value(),
              d.right.inchesPerRadian.value() > 0.0
                  ? d.left.inchesPerRadian.value() / d.right.inchesPerRadian.value()
                  : 0.0);
        emitf("  effective track width: %.4f in (LARGER than a tape's -- a turning tank scrubs; "
              "HA-135)",
              d.trackWidth.value());
        emitf("  push: %.0f in, drift %+.2f deg, median travel LEFT %.4f rad RIGHT %.4f rad, spread "
              "LEFT %.4f rad RIGHT %.4f rad (a short member shows as spread -- HA-130)",
              rep.pushDistanceIn, rep.pushStraightnessRad / kDegreesToRadians + 0.0,
              rep.leftMedianRad, rep.rightMedianRad, rep.leftSpreadRad, rep.rightSpreadRad);
        emitf("  spin widths: CCW %.4f in, CW %.4f in, disagreement %.2f pct", rep.widthCcwIn,
              rep.widthCwIn, 100.0 * rep.widthDisagreement);

        // ── THE FRONT CHECK (the team lead's ruling, 2026-10-06) ────────────────────────
        // The reference is the card's record as the boot reads it (the record file, else its
        // backup); with NO usable record on a present card — no file, or none that decodes — the
        // drive ports the team TYPED, when its facts carry them (D6 (a), 2026-10-08); else none.
        // No card: no check, typed or not — nothing can be saved, so there is no front to protect.
        const bool cardPresent = d_.card.cardPresent();
        const LoadedRecord earlier = loadRecord(d_.card);
        run.note[0] = '\0';
        bool flipped = false;
        bool firstRecord = false;  // the FIRST RECORD note was set: it stands only over a save
        bool typedRef = false;     // the typed drive ports were the reference
        const hal::DrivetrainDescription typed = typedDriveDescription(facts_);
        if (!cardPresent) {
            emit("front check: NOT DONE -- no SD card in the brain (nothing can be saved either)");
        } else if (earlier.from == Which::None && hasTypedDrive(facts_)) {
            // D6 (a): no usable record — the typed ports are the front the robot drives with now
            // (the boot drives from a valid typed table when no record decodes).
            typedRef = true;
            if (!earlier.primary.opened && !earlier.backup.opened) {
                emit("front check: the card holds no record yet -- checking against your typed "
                     "drive ports instead");
            } else {
                emit("front check: the card's record does not decode -- checking against your "
                     "typed drive ports instead");
                emitUndecodedFiles(earlier);
            }
            char l[hal::kMaxSmartPorts * 5 + 1];
            char r[hal::kMaxSmartPorts * 5 + 1];
            signedList(typed.left, l, sizeof l);
            signedList(typed.right, r, sizeof r);
            emitf("typed drive ports: LEFT%s | RIGHT%s", l, r);
            const FrontCheck fc = compareFronts(typed, d);
            emitf("front check against your typed drive ports: %u drive port(s) in both -- %u "
                  "agree, %u flipped, %u sign only, %u side only; %u in the typed ports only, %u in "
                  "this run's only: %s",
                  static_cast<unsigned>(fc.shared), static_cast<unsigned>(fc.agree),
                  static_cast<unsigned>(fc.flipped), static_cast<unsigned>(fc.signOnly),
                  static_cast<unsigned>(fc.sideOnly), static_cast<unsigned>(fc.onlyEarlier),
                  static_cast<unsigned>(fc.onlyNow), frontVerdictWord(fc.verdict));
            if (fc.verdict == FrontVerdict::Undetermined) {
                emit("front check: CANNOT TELL (no drive port in common, or no majority) -- saved "
                     "as measured; check the robot drives the right way before an auton");
                setNote(run, panel_text::kNoteNotChecked);
            } else if (fc.verdict == FrontVerdict::Same) {
                // EXACTLY the same only when every port in either list agrees (C4): any flipped,
                // sign-only or side-only port, or a port in one list only, is named on its own line.
                const unsigned differing =
                    static_cast<unsigned>(fc.shared) - static_cast<unsigned>(fc.agree)
                    + static_cast<unsigned>(fc.onlyEarlier) + static_cast<unsigned>(fc.onlyNow);
                if (differing == 0) {
                    setNote(run, panel_text::kNoteTypedSame);
                } else {
                    emitTypedDifferences(typed, d, differing);
                    setNote(run, panel_text::kNoteTypedPartial);
                }
            }
            flipped = fc.verdict == FrontVerdict::Flipped;
        } else if (earlier.from == Which::None && !earlier.primary.opened
                   && !earlier.backup.opened) {
            // "No record yet" means NEITHER file opened: a backup alone, or an EMPTY record file,
            // is a record that does not decode — the branch below.
            emit("front check: the card holds no record yet -- this is the FIRST: the end that led "
                 "the push is the FRONT");
            setNote(run, panel_text::kNoteFirst);
            firstRecord = true;
        } else if (earlier.from == Which::None) {
            emit("front check: NOT DONE -- the card's record does not decode; the end that led the "
                 "push is the FRONT -- check the robot drives the right way before an auton");
            emitUndecodedFiles(earlier);
            setNote(run, panel_text::kNoteNotChecked);
        } else {
            const FrontCheck fc = compareFronts(earlier.description, d);
            emitf("front check against %s: %u drive port(s) in both -- %u agree, %u flipped, %u "
                  "sign only, %u side only; %u in the card's only, %u in this run's only: %s",
                  earlier.from == Which::Primary ? kRecordFileName : kRecordBackupFileName,
                  static_cast<unsigned>(fc.shared), static_cast<unsigned>(fc.agree),
                  static_cast<unsigned>(fc.flipped), static_cast<unsigned>(fc.signOnly),
                  static_cast<unsigned>(fc.sideOnly), static_cast<unsigned>(fc.onlyEarlier),
                  static_cast<unsigned>(fc.onlyNow), frontVerdictWord(fc.verdict));
            if (fc.verdict == FrontVerdict::Undetermined) {
                emit("front check: CANNOT TELL (no drive port in common, or no majority) -- saved "
                     "as measured; check the robot drives the right way before an auton");
                setNote(run, panel_text::kNoteNotChecked);
            }
            flipped = fc.verdict == FrontVerdict::Flipped;
        }

        // ── THE HOLDS: nothing is written until the operator answers ────────────────────
        held_ = {};
        held_.measured = d;
        held_.typedRef = typedRef;
        held_.cardHadRecord = earlier.from != Which::None;
        held_.cardPresent = cardPresent;
        held_.driveDeg = driveDeg(cal);
        held_.mechCount = run.turnedCount;
        for (std::size_t k = 0; k < run.turnedCount; ++k) {
            // Judged for EACH port that turned, against the reference the check read — the card's
            // record, else the typed ports (D6 (a)): whether it drives with that port is that
            // port's evidence, never another's.
            held_.onCardOf[k] = typedRef ? typed.sideOf(run.turnedPort[k]) != 0
                                         : earlier.from != Which::None
                                               && earlier.description.sideOf(run.turnedPort[k])
                                                      != 0;
        }
        if (flipped && typedRef) {
            held_.held = true;
            held_.reason = HoldReason::FrontFlipped;
            emit("FRONT FLIPPED -- your typed drive ports call the end that LED this push its "
                 "BACK.");
            emit("Either the BACK led by mistake (touch RUN AGAIN: nothing is saved), or the FRONT "
                 "led");
            emit("and the typed ports are wrong, or the drive cables were swapped since they were "
                 "typed");
            emit("(touch USE NEW: saved as pushed -- then fix the typed ports). The push cannot "
                 "tell");
            emit("these apart. USE NEW reverses every auton tuned on the typed ports. NOTHING IS "
                 "SAVED YET.");
            emitLoopStats(run);
            return SaveOutcome::Held;
        }
        if (flipped) {
            held_.held = true;
            held_.reason = HoldReason::FrontFlipped;
            emit("FRONT FLIPPED -- the card's record calls the end that LED this push its BACK.");
            emit("Either the BACK led by mistake (touch RUN AGAIN: nothing is saved), or the FRONT "
                 "led");
            emit("and the drive cables were swapped between the sides, or the front was changed "
                 "on");
            emit("purpose (touch USE NEW: saved as pushed). The records cannot tell these apart.");
            emit("USE NEW after a wrong push reverses every auton. NOTHING IS SAVED YET.");
            emitLoopStats(run);
            return SaveOutcome::Held;
        }
        if (held_.mechCount > 0) {
            holdMechanism(run);
            emitLoopStats(run);
            return SaveOutcome::Held;
        }
        const SaveOutcome o = save(d);
        if (o != SaveOutcome::Saved && firstRecord) {
            // FIRST RECORD names a record this run MADE: under a NOT SAVED header it would claim one
            // the card does not hold (the serial says the save's own words, and the next boot's).
            run.note[0] = '\0';
        }
        emitLoopStats(run);
        return o;
    }

    /// The NOTE row's text, `text` (a panel literal or a formatted one) — at most one row.
    static void setNote(detail::ProgramRun& run, std::string_view text) {
        std::snprintf(run.note, sizeof run.note, "%.*s", static_cast<int>(text.size()),
                      text.data());
    }

    /// MECHANISM PORT TURNED: hold the save on the NEXT declared port that turned — its own
    /// question, with its own turn and whether the reference (the card's record, else the typed
    /// drive ports) drives with it. With several,
    /// the first says how many, and each says which of them it is.
    void holdMechanism(const detail::ProgramRun& run) {
        const std::size_t k = held_.mechAsked++;
        held_.held = true;
        held_.reason = HoldReason::MechanismTurned;
        held_.port = run.turnedPort[k];
        held_.onCard = held_.onCardOf[k];
        std::snprintf(held_.line1, sizeof held_.line1, "port %d: %.0f deg, drive %.0f deg",
                      held_.port, run.turnedDeg[k], held_.driveDeg);
        std::snprintf(held_.line2, sizeof held_.line2,
                      held_.typedRef ? "your typed ports DRIVE with port %d"
                                     : "the card's record DRIVES with port %d",
                      held_.port);
        if (held_.mechCount > 1) {
            if (k == 0) {
                emitf("MECHANISM PORTS TURNED -- %u declared ports turned with the push: each is "
                      "asked on its own screen, one at a time. NOTHING is saved until every one is "
                      "answered SAVE; RUN AGAIN on any of them saves nothing.",
                      static_cast<unsigned>(held_.mechCount));
            }
            emitf("MECHANISM PORT TURNED (%u of %u) -- port %d is declared a mechanism but turned "
                  "with the push (%s).",
                  static_cast<unsigned>(k + 1), static_cast<unsigned>(held_.mechCount), held_.port,
                  held_.line1);
        } else {
            emitf("MECHANISM PORT TURNED -- port %d is declared a mechanism but turned with the "
                  "push (%s).",
                  held_.port, held_.line1);
        }
        if (held_.onCard && held_.typedRef) {
            emitf("Your typed drive ports DRIVE with port %d: saving now makes a record with NO "
                  "port %d.",
                  held_.port, held_.port);
        } else if (held_.onCard) {
            emitf("The card's record DRIVES with port %d: saving now replaces that record with one "
                  "that has NO port %d.",
                  held_.port, held_.port);
        }
        emit("If it is a mechanism the push turns (an intake on the floor), touch SAVE and keep it");
        emit("declared. If it is a DRIVE motor, touch RUN AGAIN (nothing is saved), then stop: fix "
             "the");
        emit("robot's facts or the wiring, rebuild, upload. NOTHING IS SAVED YET.");
    }

    /// A held screen's left box asks the NEXT question rather than saving: FRONT FLIPPED with a
    /// MECHANISM PORT TURNED question to come, or a MECHANISM PORT TURNED that is not the last.
    [[nodiscard]] bool heldAsksNext() const noexcept {
        if (!held_.held) {
            return false;
        }
        return held_.reason == HoldReason::FrontFlipped ? held_.mechCount > 0
                                                        : held_.mechAsked < held_.mechCount;
    }

    /// The operator touched the held screen's left box: USE NEW (FRONT FLIPPED) or SAVE
    /// (MECHANISM PORT TURNED). Each MECHANISM PORT TURNED question still unasked is asked next,
    /// one per screen; only when nothing is left to ask is the run saved — AS MEASURED.
    void resolveHeld(detail::ProgramRun& run) {
        if (!held_.held) {
            return;
        }
        if (held_.reason == HoldReason::FrontFlipped) {
            emit("front: USE NEW -- the end pushed first THIS time is the FRONT (saved as pushed)");
            run.front = FrontChoice::New;
            if (held_.typedRef) {  // D6 (a): the typed ports now call the saved front the back
                emit("front: now FIX YOUR TYPED DRIVE PORTS -- they call this FRONT the BACK, and "
                     "anything that still drives from them drives this robot backwards");
                setNote(run, panel_text::kNoteFixTyped);
            }
        } else {
            emitf("watch: SAVE -- port %d stays a declared mechanism; this record has no port %d",
                  held_.port, held_.port);
        }
        if (held_.mechAsked < held_.mechCount) {
            holdMechanism(run);  // the next question: the screen changes, its boxes re-arm
            return;
        }
        if (held_.mechCount == 1) {
            std::snprintf(run.note, sizeof run.note, "PORT %d KEPT AS A MECHANISM -- see the log",
                          held_.port);
        } else if (held_.mechCount > 1) {
            std::snprintf(run.note, sizeof run.note, "%u PORTS KEPT AS MECHANISMS -- see the log",
                          static_cast<unsigned>(held_.mechCount));
        }
        const hal::DrivetrainDescription d = held_.measured;
        held_ = {};
        run.save = save(d);
    }

    /// The finished and held screens' facts for this run (the panel cannot know them).
    [[nodiscard]] FinishFacts finishFacts(const detail::ProgramRun& run) const {
        FinishFacts f{};
        f.front = run.front;
        f.hold = held_.held ? held_.reason : HoldReason::None;
        f.holdPort =
            held_.held && held_.reason == HoldReason::MechanismTurned ? held_.port : 0;
        f.holdLine1 = held_.line1;
        f.holdLine2 = held_.onCard ? held_.line2 : nullptr;
        f.note = run.note[0] != '\0' ? run.note : nullptr;
        f.typedPorts = held_.held && held_.typedRef;
        return f;
    }

    /// SAFE-SAVE B1: the safe save over the program's card, the panel told Saved iff the save
    /// MEASURED it so, and the words from that measured report (the boot of this build does not
    /// read the backup until SAFE-SAVE B2: BootReadsBackup::No).
    SaveOutcome save(const hal::DrivetrainDescription& d) {
        const SaveReport report = saveRecordSafely(d_.card, d);
        formatSaveReport(report, BootReadsBackup::No, [this](const char* l) { emit(l); });
        if (report.outcome == SaveResult::Saved) {
            emit("COPY THE 'MEASURED tank ...' LINE ABOVE DOWN. Then touch RUN AGAIN and check");
            emit("the two runs agree -- a number that does not reproduce is not a measurement");
            emit("(the tolerances, 2 pct for a scale and 3 pct for the width, are HA-136, "
                 "invented).");
            return SaveOutcome::Saved;
        }
        emit("THE RESULT IS ON THE SCREEN AND IN THIS LOG ONLY. Copy it down NOW.");
        return SaveOutcome::NotSaved;
    }

    /// Aborted: the refusal's code, its port, and its sentence.
    void finishStopped(const DrivetrainCalibrator& cal, const detail::ProgramRun& run) {
        if (cal.refusalPort() > 0) {
            emitf("STOPPED [%s port %d]: %s", refusalName(cal.refusal()), cal.refusalPort(),
                  cal.reason());
        } else {
            emitf("STOPPED [%s]: %s", refusalName(cal.refusal()), cal.reason());
        }
        if (cal.refusalPort() > 0
            && (cal.refusal() == Refusal::PulseRolled || cal.refusal() == Refusal::PulseStill)) {
            emitf("if port %d is a MECHANISM motor, declare it in the robot's facts: a motor that "
                  "is not declared is a drive candidate",
                  cal.refusalPort());
        }
        emit("Every motor is at 0 V and coasting. Nothing was written to the card by this run.");
        emitLoopStats(run);
    }

    /// Everything the serial record owes after a tick or an answer: a phase change (and the
    /// watch's baseline when the push begins), each new pulse, each port newly LEFT OUT, each
    /// finished spin direction, the push's live count, and — once — the outcome (the check and the
    /// save included).
    void record(const DrivetrainCalibrator& cal, detail::ProgramRun& run, double now) {
        const CalibrationReport& rep = cal.report();
        const CalibratorStatus& st = cal.status();
        while (run.pulsesPrinted < rep.pulseCount) {
            char line[160];
            formatPulseRecord(rep.pulses[run.pulsesPrinted],
                              static_cast<unsigned>(rep.drivePortCount), line, sizeof line);
            emit(line);
            ++run.pulsesPrinted;
        }
        while (run.excludedPrinted < rep.excludedCount) {
            emitf("LEFT OUT: port %d turns but does not move the robot (a slipping gear? or a "
                  "MECHANISM motor -- declare it in the robot's facts) -- it is in no side and is "
                  "never commanded again",
                  static_cast<int>(rep.excludedPorts[run.excludedPrinted]));
            ++run.excludedPrinted;
        }
        while (run.spinsPrinted < rep.spinCount) {
            char line[160];
            formatSpinRecord(rep.spins[run.spinsPrinted], line, sizeof line);
            emit(line);
            ++run.spinsPrinted;
        }
        if (cal.phase() != run.lastPhase) {
            run.lastPhase = cal.phase();
            if (cal.phase() == Phase::PushCapture) {
                watchArm(run);
            }
            if (!detail::programRunFinished(cal.phase())) {  // an ending is the outcome's to say
                emitf("phase: %s | %s", detail::programPhaseWord(cal.phase()), st.progress);
            }
        }
        if (cal.phase() == Phase::PushCapture && cal.accepts(Answer::Done)) {
            const int moved = st.movedNow;
            const int ready = cal.pushReady() ? 1 : 0;
            const int far = st.pushFarEnough ? 1 : 0;
            const bool changed =
                moved != run.lastMoved || ready != run.lastReady || far != run.lastFar;
            if (detail::programPushLogDue(changed, now - run.lastPushLogAt, kPushLogMinSeconds,
                                          kPushLogKeepaliveSeconds)) {
                run.lastMoved = moved;
                run.lastReady = ready;
                run.lastFar = far;
                run.lastPushLogAt = now;
                emitf("push live: %s", st.progress);
            }
        }
        if (detail::programRunFinished(cal.phase()) && !run.outcomePrinted) {
            run.outcomePrinted = true;
            emitf("===== RUN %d ENDED: %s =====", run.number,
                  cal.phase() == Phase::Done ? "DONE" : "STOPPED");
            if (cal.phase() == Phase::Done) {
                switch (kind_) {  // R5a Part C's GAINS and REST print their own outcome here
                    case RunKind::Drivetrain: run.save = finishDone(cal, run); break;
                }
            } else {
                finishStopped(cal, run);
            }
        }
    }

    /// A `screen:` line whenever the LAYOUT changes — the header word, the first three lines and
    /// the buttons — or a held screen's question does (its screens share one layout: FRONT FLIPPED
    /// and MECHANISM PORT TURNED, and one MECHANISM PORT TURNED per port), so a capture says what
    /// the operator was looking at when each touch landed.
    void recordLayout(const PanelView& v, detail::ProgramRun& run) {
        const std::uint32_t key = layoutKey(v);
        const HoldReason hold = v.screen == PanelScreen::Held ? held_.reason : HoldReason::None;
        const int holdPort = hold == HoldReason::MechanismTurned ? held_.port : 0;
        if (run.haveLayout && key == run.lastLayout && hold == run.lastLayoutHold
            && holdPort == run.lastLayoutHoldPort) {
            return;
        }
        run.haveLayout = true;
        run.lastLayout = key;
        run.lastLayoutHold = hold;
        run.lastLayoutHoldPort = holdPort;
        char words[200] = "";
        std::size_t used = 0;
        for (std::size_t i = 0; i < v.lineCount && i < 3; ++i) {
            const int w = std::snprintf(words + used, sizeof words - used, "%s%s",
                                        i > 0 ? " / " : "", v.lines[i].text);
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < sizeof words ? used : sizeof words - 1;
        }
        char buttons[64] = "";
        used = 0;
        for (std::size_t b = 0; b < v.buttonCount; ++b) {
            const int w = std::snprintf(buttons + used, sizeof buttons - used, " [%s]",
                                        detail::programButtonWord(v.buttons[b]));
            used += w > 0 ? static_cast<std::size_t>(w) : 0;
            used = used < sizeof buttons ? used : sizeof buttons - 1;
        }
        emitf("screen: %s: %s |%s", v.headerWord, words,
              buttons[0] != '\0' ? buttons : " no buttons");
    }

    /// One `touch:` line for a new release: the button it landed on and what it did — or why it
    /// did nothing (greyed, no longer accepted, or no button there).
    void logTouch(const PanelView& drawn, const TouchSample& sample, const TouchResult& r) {
        const char* on = r.hit >= 0
                             ? detail::programButtonWord(drawn.buttons[static_cast<std::size_t>(r.hit)])
                             : "nothing";
        const int x = sample.x;
        const int y = sample.y;
        const long release = sample.releaseCount;
        if (r.action == TouchAction::Answer) {
            emitf("touch: x=%d y=%d release=%ld on %s -> ANSWER %s", x, y, release, on,
                  detail::programAnswerWord(r.answer));
        } else if (r.action == TouchAction::RunAgain) {
            emitf("touch: x=%d y=%d release=%ld on %s -> RUN AGAIN", x, y, release, on);
        } else if (r.action == TouchAction::SaveHeld) {
            // Worded by what the touch DOES: it saves only when no question is left to ask — a USE
            // NEW with a MECHANISM question pending, or any SAVE but the last, saves nothing yet.
            if (heldAsksNext()) {
                emitf("touch: x=%d y=%d release=%ld on %s -> %s -- the next question follows; nothing "
                      "is saved yet",
                      x, y, release, on, on);
            } else {
                emitf("touch: x=%d y=%d release=%ld on %s -> SAVE AS PUSHED", x, y, release, on);
            }
        } else {
            const char* why = "ignored -- no button there";
            if (r.hit >= 0) {
                why = "ignored -- the calibrator no longer accepts it";
            } else {
                for (std::size_t b = 0; b < drawn.buttonCount; ++b) {
                    if (!drawn.buttons[b].armed
                        && contains(hitBox(drawn.buttons[b].rect), sample.x, sample.y)) {
                        why = "ignored -- the button was still GREYED (arming)";
                    }
                }
            }
            emitf("touch: x=%d y=%d release=%ld on %s -> %s", x, y, release, on, why);
        }
    }

    CalibrateFacts facts_;
    CalibrateDevices d_;
    const char* stamp_;
    char buildTime_[24];  // before painter_: the painter is constructed over it
    CalibratePainter painter_;
    Held held_{};
    RunKind kind_ = RunKind::Drivetrain;
};

/// initialize()'s banner for the standard Calibrate, through `io`: `label` (a build's beacon, or
/// null), the competition status at boot, the build stamp. Constructs nothing and powers nothing —
/// the motors are found when driver control starts.
inline void calibrateBootBanner(CalibrateIo& io, const char* label, std::uint8_t status,
                                const char* buildStamp) {
    char bits[64];
    competitionWords(status, bits, sizeof bits);
    char line[240];
    std::snprintf(line, sizeof line, "booted: shulib CALIBRATE build [%.150s].",
                  label != nullptr ? label : "no label");
    io.line(line);
    io.line("the robot MEASURES ITS OWN DRIVETRAIN -- which ports drive, which way each");
    io.line("is wired, which side each is on, inches per radian per side, and the");
    std::snprintf(line, sizeof line,
                  "EFFECTIVE track width -- then writes them to /usd/%s (a verified backup %s "
                  "first).",
                  kRecordFileName, kRecordBackupFileName);
    io.line(line);
    io.line("It POWERS THE MOTORS. The end that LEADS the push is the FRONT. Every threshold");
    io.line("in it is INVENTED (A4 register HA-136).");
    io.line("nothing is constructed at boot: the motors are found when driver control starts.");
    std::snprintf(line, sizeof line, "BUILD STAMP %.100s", buildStamp != nullptr ? buildStamp : "");
    io.line(line);
    std::snprintf(line, sizeof line,
                  "competition status at boot: %s -- calibration runs only in DRIVER CONTROL, "
                  "enabled; any other state stops the run and says so.",
                  bits);
    io.line(line);
}

/// disabled() / competition_initialize() / autonomous() for the standard Calibrate: its
/// competition-state screen on `screen` (it runs ONLY in driver control) and one serial line through
/// `io`. Pass calibrate_screen.hpp's words — screen_text::kStateName* for `state` and kStateWhy*
/// for `why` — which are checked at compile time to fit; any other text is cut, and the screen says
/// so. A free function: those entry points run on their own PROS tasks, before and after run()'s
/// program exists.
inline void calibrateStateScreen(hal::IBrainScreen& screen, CalibrateIo& io, const char* state,
                                 const char* why) {
    char bits[64];
    competitionWords(io.competitionStatus(), bits, sizeof bits);
    char line[240];
    std::snprintf(line, sizeof line, "competition state: %.40s (%.40s) -- %.130s",
                  state != nullptr ? state : "", bits, why != nullptr ? why : "");
    io.line(line);
    char status[96];
    std::snprintf(status, sizeof status, "status bits %s", bits);
    CalibratePainter painter{screen, ""};
    painter.paintStateScreen(state, why, status);
}

}  // namespace shulib::calibration
