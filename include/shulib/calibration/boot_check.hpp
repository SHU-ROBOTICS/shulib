#pragma once
//
// THE BOOT CHECK — is this description still true of THIS robot? (chunk R3d Part 2)
//
// ── Why a description is not enough ─────────────────────────────────────────────────
// A calibration record is a claim about a robot made on some earlier day. Between then and
// now: a cable can be re-seated into the next port, a motor can die, a card can be carried
// from robot one to robot two, a sensor can be swapped for a different kind. None of those
// is visible from inside the record — the CRC will be perfect, `valid()` will pass, and the
// description will describe a robot that no longer exists. So before "shulib Teleop"
// constructs a single adapter, it asks the brain what is actually plugged in and compares.
//
// BEFORE ANY ADAPTER EXISTS, deliberately (ruling P2). Constructing ten ProsMotors to find
// out which ones answer is how HA-111's invented port map turned into ten throwing
// constructors at boot; the registry read is free and happens first.
//
// ── The four classes, and what each one means ───────────────────────────────────────
//   PRESENT  the port carries a MOTOR, as the description says. Nothing to report.
//   MISSING  the port is EMPTY. A dead cable, a dead motor, a port bank that browned out.
//            This is TOLERATED by policy, not by this file: teleop::evaluateDegradation
//            (the team lead's rule — "1–2 dead ports shouldn't stop driving") decides, so
//            the drive program and the library program cannot disagree about it.
//   SWAPPED  the port carries a DIFFERENT KIND of device. This always REFUSES (ruling P2).
//            A port that lost its motor may be a dead cable; a port that now carries a
//            rotation sensor is a REWIRED ROBOT, and a rewired robot needs a new
//            calibration, not a best effort.
//   NEW      a motor the registry sees on a port the description does not list. Always a
//            WARNING, never a refusal: an extra motor is not a drivetrain fault (an
//            intake, a lift), but a drive motor added since the calibration would sit
//            uncommanded and fighting its side-mates, so nobody gets to not be told.
// The IMU port must carry an IMU. Missing or wrong kind ⇒ REFUSE: every number in the
// description was measured against an IMU, and without one nothing can be re-checked.
//
// ═══ THE HONEST LIMIT — READ THIS, AND SAY IT ON THE SCREEN ═════════════════════════
// TWO DRIVE CABLES SWAPPED WITH EACH OTHER ARE INVISIBLE HERE. Both ports still carry
// motors, so both classify PRESENT, and the verdict is Ok. The description then drives port
// 12's motor with port 13's sign — two members of one side fighting each other on the first
// command. The same is true of a RE-GEARING that keeps the ports: the registry cannot see a
// gear.
//
// That is not a hole to be patched here; it is the boundary of what a port census can know.
// What catches it is the FIRST DRIVE: teleop::CoupledSideMonitor sees a member opposite its
// side's command and raises MOTOR_GROUP_DISAGREE within 250 ms, and the panel names the
// port. A limit nobody has pinned is a limit somebody will forget, so
// test/boot_check_test.cpp asserts it — two drive ports exchanged ⇒ Ok — and the refusal
// screen's wording tells the operator what the check did NOT check.
//
// ── Pure, allocation-free, testable against every row ──────────────────────────────
// Numbers and an interface in, a verdict out; no clock, no state, no adapters. Everything
// here is exercised through hal::fake::FakeDeviceRegistry, which is how "a port that now
// carries a rotation sensor" becomes a test case instead of an argument.
//
// NO HARDWARE CLAIM: no verdict in this file has ever been computed on a robot.

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "shulib/hal/device_registry.hpp"
#include "shulib/hal/drivetrain_description.hpp"
#include "shulib/teleop/drivetrain_degradation.hpp"

namespace shulib::calibration {

/// What the registry says about ONE port the description lists as a drive port.
enum class PortClass : std::uint8_t {
    Present,  ///< a MOTOR is there, as the description says
    Missing,  ///< the port is EMPTY — a dead cable or a dead motor (policy decides)
    Swapped,  ///< the port carries a DIFFERENT KIND of device — a rewired robot (always refuses)
};

/// The class as one word, for the panel and the refusal line.
[[nodiscard]] inline const char* portClassWord(PortClass c) noexcept {
    switch (c) {
        case PortClass::Present: return "PRESENT";
        case PortClass::Missing: return "MISSING";
        case PortClass::Swapped: return "SWAPPED";
    }
    return "?";
}

/// What the boot may do with this description on this robot.
enum class BootDecision : std::uint8_t {
    Ok,        ///< every listed port carries its motor and the IMU is where it should be
    Degraded,  ///< some ports are MISSING, but teleop::evaluateDegradation still says drive
    Refuse,    ///< a SWAPPED port, a bad IMU port, an unusable census, or the policy says no
};

/// The decision as one word, for a banner.
[[nodiscard]] inline const char* bootDecisionWord(BootDecision d) noexcept {
    switch (d) {
        case BootDecision::Ok: return "OK";
        case BootDecision::Degraded: return "DEGRADED";
        case BootDecision::Refuse: return "REFUSE";
    }
    return "?";
}

/// One row of the comparison: what the description said about a port, and what the brain says.
struct BootPortStatus {
    /// The description's entry AS STORED — signed, because that is what the adapter gets.
    std::int8_t signedPort = 0;
    /// |signedPort| — the physical port a person reads off the brain.
    std::uint8_t port = 0;
    /// −1 LEFT, +1 RIGHT (the description's sides).
    std::int8_t side = 0;
    /// What the census made of it.
    PortClass state = PortClass::Present;
    /// What the registry actually answered — so a SWAPPED row can say WHAT is there now.
    hal::DeviceKind found = hal::DeviceKind::None;
};

/// The whole verdict: the decision, every port's row, the new motors, the IMU's own row, the
/// degradation verdict the decision was taken from, and one bounded line of prose naming every
/// port in every class. Fixed width and allocation-free — a composition root builds one on the
/// stack before any adapter exists.
struct BootVerdict {
    /// What the boot may do. Defaults to Refuse: a verdict nobody computed must never read "Ok".
    BootDecision decision = BootDecision::Refuse;
    /// One row per drive port the description lists, LEFT side first then RIGHT, in the
    /// description's own order.
    BootPortStatus ports[hal::kMaxSmartPorts]{};
    /// How many rows of `ports` are real.
    std::uint8_t portCount = 0;
    /// How many of them are MISSING.
    std::uint8_t missingCount = 0;
    /// How many of them are SWAPPED.
    std::uint8_t swappedCount = 0;
    /// Physical ports carrying a MOTOR that the description does not list — always a warning.
    std::uint8_t newPorts[hal::kMaxSmartPorts]{};
    /// How many entries of `newPorts` are real.
    std::uint8_t newCount = 0;
    /// The description's IMU port.
    std::uint8_t imuPort = 0;
    /// What the registry says is on the IMU port.
    hal::DeviceKind imuFound = hal::DeviceKind::None;
    /// True when the IMU port carries an IMU.
    bool imuOk = false;
    /// The dead-port policy's own verdict over the PRESENT/MISSING counts — one policy, two
    /// programs (teleop/drivetrain_degradation.hpp).
    teleop::DegradationVerdict degradation{};
    /// One line naming every port in every class, for the screen and the log. Bounded and
    /// NUL-terminated; truncates rather than overflowing. 320 bytes because the Ok line
    /// carries the honest limit as well as the counts, and the host build compiles this header
    /// under -Wformat-truncation: a buffer that COULD truncate a literal is a compile error
    /// here, which is the right place to find out rather than on a brain's screen.
    /// ASCII only, deliberately -- the V5's screen font and the USB serial log both render a
    /// UTF-8 em dash as mojibake, and every other on-screen string in src/ uses "--".
    char reason[320]{};

    /// True when this verdict permits building a drivetrain at all (Ok or Degraded).
    [[nodiscard]] bool usable() const noexcept { return decision != BootDecision::Refuse; }

    /// True when physical `port` is one this verdict classed MISSING — the question the
    /// composition root asks per port when it decides which group members are ABSENT.
    [[nodiscard]] bool isMissing(int port) const noexcept {
        for (std::size_t i = 0; i < portCount; ++i) {
            if (static_cast<int>(ports[i].port) == port) {
                return ports[i].state == PortClass::Missing;
            }
        }
        return false;
    }
};

/// Compare `desc` against what `registry` says is plugged in, WITHOUT constructing anything.
///
/// The decision, in this order (each earlier one wins, and `reason` names it):
///  1. the description is not `valid()` — Refuse (defensive: every caller checks first, and a
///     refusal here is still better than a graph built from a contradictory description);
///  2. any SWAPPED port — Refuse, naming each one and what is there instead (ruling P2);
///  3. the IMU port missing or carrying something else — Refuse, naming what is there;
///  4. whatever `teleop::evaluateDegradation` says about the PRESENT counts — Refuse,
///     Degraded (listing the missing ports) or Ok. One policy, shared with "shulib Drive".
/// NEW motors are always listed, and never change the decision.
///
/// The honest limit is in the header and is repeated in `reason` on every non-Ok verdict: two
/// drive cables swapped WITH EACH OTHER are invisible to a port census and are caught at the
/// first drive by the coupled-side monitor.
[[nodiscard]] inline BootVerdict verifyAgainstRegistry(const hal::DrivetrainDescription& desc,
                                                       const hal::IDeviceRegistry& registry) {
    BootVerdict v{};
    char why[160];
    if (!desc.valid(why, sizeof why)) {
        std::snprintf(v.reason, sizeof v.reason,
                      "REFUSE: the description contradicts itself before any port was read: %s",
                      why);
        return v;
    }

    // ── one row per listed drive port, LEFT then RIGHT, in the description's order ──
    const hal::DriveSideDescription* sides[2] = {&desc.left, &desc.right};
    const std::int8_t sideSign[2] = {-1, +1};
    for (int s = 0; s < 2; ++s) {
        const hal::DriveSideDescription& side = *sides[s];
        for (std::size_t i = 0; i < side.count; ++i) {
            BootPortStatus& row = v.ports[v.portCount++];
            row.signedPort = side.signedPorts[i];
            row.port = static_cast<std::uint8_t>(
                hal::DrivetrainDescription::portMagnitude(side.signedPorts[i]));
            row.side = sideSign[s];
            row.found = registry.pluggedKind(static_cast<int>(row.port));
            if (row.found == hal::DeviceKind::Motor) {
                row.state = PortClass::Present;
            } else if (row.found == hal::DeviceKind::None) {
                row.state = PortClass::Missing;
                ++v.missingCount;
            } else {
                row.state = PortClass::Swapped;
                ++v.swappedCount;
            }
        }
    }

    // ── the IMU, and the motors nobody told the description about ──
    v.imuPort = desc.imuPort;
    v.imuFound = registry.pluggedKind(static_cast<int>(desc.imuPort));
    v.imuOk = v.imuFound == hal::DeviceKind::Imu;
    for (int p = 1; p <= static_cast<int>(hal::kMaxSmartPorts); ++p) {
        if (registry.pluggedKind(p) != hal::DeviceKind::Motor) {
            continue;
        }
        if (desc.sideOf(p) != 0) {
            continue;  // a listed drive port; it already has a row
        }
        v.newPorts[v.newCount++] = static_cast<std::uint8_t>(p);
    }

    // ── the degradation policy, over PRESENT counts (one policy, two programs) ──
    teleop::SideCount left{};
    teleop::SideCount right{};
    for (std::size_t i = 0; i < v.portCount; ++i) {
        teleop::SideCount& c = v.ports[i].side < 0 ? left : right;
        ++c.expected;
        if (v.ports[i].state == PortClass::Present) {
            ++c.answering;
        }
    }
    v.degradation = teleop::evaluateDegradation(left, right);

    // ── the prose, built once, naming every port in every class ──
    // Every snprintf below uses a LITERAL format string: src/ compiles under
    // -Wformat-nonliteral, and a shared helper taking `const char* format` would defeat the
    // compiler's argument checking exactly where a wrong specifier would corrupt the one
    // message a person gets when the robot refuses to boot (Part 1's note).
    char listSwapped[96] = "";
    char listMissing[64] = "";
    char listNew[64] = "";
    for (std::size_t i = 0; i < v.portCount; ++i) {
        const BootPortStatus& row = v.ports[i];
        if (row.state == PortClass::Swapped) {
            char one[32];
            std::snprintf(one, sizeof one, "%s%d(%s)", listSwapped[0] == '\0' ? "" : " ",
                          static_cast<int>(row.port), hal::deviceKindWord(row.found));
            std::snprintf(listSwapped + std::strlen(listSwapped),
                          sizeof listSwapped - std::strlen(listSwapped), "%s", one);
        } else if (row.state == PortClass::Missing) {
            char one[16];
            std::snprintf(one, sizeof one, "%s%d", listMissing[0] == '\0' ? "" : " ",
                          static_cast<int>(row.port));
            std::snprintf(listMissing + std::strlen(listMissing),
                          sizeof listMissing - std::strlen(listMissing), "%s", one);
        }
    }
    for (std::size_t i = 0; i < v.newCount; ++i) {
        char one[16];
        std::snprintf(one, sizeof one, "%s%d", listNew[0] == '\0' ? "" : " ",
                      static_cast<int>(v.newPorts[i]));
        std::snprintf(listNew + std::strlen(listNew), sizeof listNew - std::strlen(listNew),
                      "%s", one);
    }

    if (v.swappedCount > 0) {
        v.decision = BootDecision::Refuse;
        std::snprintf(v.reason, sizeof v.reason,
                      "REFUSE: port(s) %s carry a DIFFERENT device than the calibration "
                      "recorded -- this robot was rewired; run shulib Calibrate",
                      listSwapped);
        return v;
    }
    if (!v.imuOk) {
        v.decision = BootDecision::Refuse;
        std::snprintf(v.reason, sizeof v.reason,
                      "REFUSE: the calibration was taken against an IMU on port %d and that "
                      "port now reads %s; run shulib Calibrate",
                      static_cast<int>(v.imuPort), hal::deviceKindWord(v.imuFound));
        return v;
    }
    if (v.degradation.verdict == teleop::DriveVerdict::Refuse) {
        v.decision = BootDecision::Refuse;
        std::snprintf(v.reason, sizeof v.reason,
                      "REFUSE: %s -- missing port(s) %s (left dead %d, right dead %d)",
                      v.degradation.reason, listMissing[0] == '\0' ? "none" : listMissing,
                      v.degradation.leftDead, v.degradation.rightDead);
        return v;
    }
    if (v.degradation.verdict == teleop::DriveVerdict::DriveDegraded) {
        v.decision = BootDecision::Degraded;
        std::snprintf(v.reason, sizeof v.reason,
                      "DEGRADED: dead %s -- driving without them (left dead %d, right dead "
                      "%d). NEW motors not in the calibration: %s",
                      listMissing, v.degradation.leftDead, v.degradation.rightDead,
                      listNew[0] == '\0' ? "none" : listNew);
        return v;
    }
    v.decision = BootDecision::Ok;
    std::snprintf(v.reason, sizeof v.reason,
                  "OK: all %d recorded drive ports carry motors, IMU on port %d. NEW motors "
                  "not in the calibration: %s. NOT CHECKED: two drive cables swapped WITH EACH "
                  "OTHER look identical here -- the first drive catches that",
                  static_cast<int>(v.portCount), static_cast<int>(v.imuPort),
                  listNew[0] == '\0' ? "none" : listNew);
    return v;
}

/// Which of the two description sources a boot should use.
enum class BootSource : std::uint8_t {
    UseRecord,  ///< the card's calibration record — measured on THIS robot
    UseTable,   ///< src/chassis_table.hpp's typed table — a human's tape measure
    Refuse,     ///< neither can be trusted; nothing is constructed
};

/// The chosen source with the sentence that explains it. `why` is always a string literal, so
/// the struct is copyable, allocation-free and safe to keep.
struct BootSourceChoice {
    /// What to build the drivetrain from.
    BootSource source = BootSource::Refuse;
    /// One sentence for the banner and the refusal screen — never empty.
    const char* why = "";
};

/// THE BOOT DECISION TABLE, as data rather than as an `if` chain nobody tested (ruling P3).
///
/// `recordDecoded` — a record was on the card and passed every format check.
/// `recordVerdict` — what verifyAgainstRegistry() made of it (meaningless when !recordDecoded).
/// `tableValid`    — `tableToDescription(kChassis).valid()`.
///
/// | record   | verdict  | table   | → |
/// |----------|----------|---------|---|
/// | decoded  | Ok       | any     | UseRecord |
/// | decoded  | Degraded | any     | UseRecord (with dead ports named by the verdict) |
/// | decoded  | Refuse   | any     | **Refuse** — NO fallback to the table |
/// | absent   | —        | valid   | UseTable, with a warning that it is unverified |
/// | absent   | —        | invalid | Refuse, naming BOTH reasons |
///
/// The row that carries the ruling is the third. A record that FAILED its port check means the
/// robot was rewired since the calibration — and the typed table is OLDER than the record, so
/// falling back to it would answer "this robot changed" with "then use the more out-of-date
/// description". Rejected alternatives: falling back to the table on a refused record (above);
/// preferring a VALID table over a DEGRADED record (a table is typed from a build that may no
/// longer exist; a degraded record was measured on this robot and merely lost a motor); a
/// screen chooser (the team lead's ruling against choosers inside programs, R3b Part 0b).
[[nodiscard]] inline constexpr BootSourceChoice chooseBootSource(bool recordDecoded,
                                                                 BootDecision recordVerdict,
                                                                 bool tableValid) noexcept {
    if (recordDecoded) {
        switch (recordVerdict) {
            case BootDecision::Ok:
                return {BootSource::UseRecord,
                        "MEASURED calibration record on the card, verified against the ports"};
            case BootDecision::Degraded:
                return {BootSource::UseRecord,
                        "MEASURED calibration record on the card; some ports are dead and the "
                        "drivetrain is built without them"};
            case BootDecision::Refuse:
                break;
        }
        return {BootSource::Refuse,
                "the calibration record does not match this robot's ports -- it was rewired, or "
                "the card came from another robot; run shulib Calibrate"};
    }
    if (tableValid) {
        return {BootSource::UseTable,
                "no calibration record on the card -- falling back to the TYPED TABLE, which is "
                "unverified since the rebuild; run shulib Calibrate"};
    }
    return {BootSource::Refuse,
            "no calibration record on the card, and the typed table cannot describe a "
            "drivetrain either; run shulib Calibrate"};
}

}  // namespace shulib::calibration
