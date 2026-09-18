#pragma once
//
// DrivetrainDescription — the ONE representation of a drivetrain's discovered facts
// (chunk R3d Part 1, 2026-09-17).
//
// ── Why this type exists ────────────────────────────────────────────────────────────
// Almost every number a human is asked to type about a drivetrain is discoverable on the
// robot: which ports drive, which way each one is wired, which side each belongs to, how
// far the chassis travels per shaft radian, how wide the track effectively is. Only three
// facts must be STATED — the cartridge (physically unobservable: the rotor turns the same
// for every insert and the firmware already divides by whatever it was told), the
// drivetrain KIND, and which end is the FRONT. Everything else is a measurement, and this
// struct is where a measurement lands.
//
// It has more than one producer on purpose:
//   * calibration::DrivetrainCalibrator fills it from a push, a set of pulses, a push
//     along a tape and a powered spin      (source = RobotMeasured);
//   * src/chassis_table.hpp's tableToDescription() fills it from the typed table, which
//     stays as the fallback                (source = TypedTable).
// One representation, two sources — so a composition root has exactly ONE input type.
//
// ── Laid out to be SERIALIZED, before anything serializes it ────────────────────────
// Fixed-width fields, bounded arrays, no pointers, no std::string, no allocation. That is
// not tidiness: a calibration record is read ONCE per boot, and a flipped bit in a sign or
// a scale drives the robot wrong SILENTLY — so the record that carries this will be
// written byte by byte under a CRC-32 and refused on mismatch. This header only defines
// the shape that makes that possible; nothing here writes a byte.
//
// ── One number lives here: kMaxSmartPorts ───────────────────────────────────────────
// A V5 brain has 21 smart ports. src/chassis_table.hpp's kMaxPort now CITES the constant
// below rather than repeating it. teleop::kMaxSideMembers deliberately keeps its own copy:
// hal::MotorGroup already depends on teleop/coupled_side_monitor.hpp, so making the monitor
// depend on this header would close a directory-level cycle, and a pure fight detector has
// no business knowing what a drivetrain description is. The two constants are pinned EQUAL
// by a static_assert in the host test instead — a pin with no dependency.
//
// NO HARDWARE CLAIM IS MADE ANYWHERE IN THIS FILE. No description has ever been produced
// on a robot; the calibrator that fills one has never moved a physical drivetrain. Every
// property here is proven against the A2 host plant and nothing else.
//
// PROS-free, sim-free, allocation-free; safe to include from any layer that may include
// hal/.

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "shulib/hal/drive_geometry.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal {

/// The most smart ports a V5 brain has, and therefore the most drive motors any side or any
/// description can carry. Physical ports are numbered 1..kMaxSmartPorts — 0 is "no port".
inline constexpr std::size_t kMaxSmartPorts = 21;

/// Which drivetrain the description describes. APPEND-ONLY: the numeric values are written
/// into a persisted record, so an existing name's value may never change. Holonomic kinds
/// arrive with the X-drive in spring 2027 (R3c), and the calibrator refuses anything but Tank
/// because a strafe scale needs measurements it does not yet take.
enum class DrivetrainKind : std::uint8_t {
    Tank = 1,  ///< two sides, no lateral authority — kinematics/tank.hpp
};

/// The cartridge in the drive motors. HUMAN-STATED and unverifiable from inside: every
/// cartridge spins the rotor at the same speed and the firmware divides by the gearset it was
/// configured with, so a wrong belief is invisible to any reading the robot can take. Append-only
/// values, for the same persisted-record reason as DrivetrainKind. There is deliberately no
/// `Unset` member: a description carrying an unstated cartridge must be INVALID, not merely
/// incomplete, and valid() refuses any byte outside this set.
enum class MotorCartridge : std::uint8_t {
    Red = 1,    ///< 100 rpm
    Green = 2,  ///< 200 rpm
    Blue = 3,   ///< 600 rpm
};

/// The cartridge as words, for banners — "UNSET" for any byte outside the enum, which is what a
/// default-constructed or corrupted description carries.
[[nodiscard]] inline const char* cartridgeWord(MotorCartridge c) noexcept {
    switch (c) {
        case MotorCartridge::Red: return "RED 100 rpm";
        case MotorCartridge::Green: return "GREEN 200 rpm";
        case MotorCartridge::Blue: return "BLUE 600 rpm";
    }
    return "UNSET";
}

/// Where a description's numbers came from — printed in every banner, because "typed by a
/// human from a tape measure" and "measured by the robot" are not the same claim.
enum class DescriptionSource : std::uint8_t {
    TypedTable = 1,     ///< src/chassis_table.hpp, via tableToDescription()
    RobotMeasured = 2,  ///< calibration::DrivetrainCalibrator, on a robot that moved
};

/// One side of a tank drivetrain: its drive ports WITH THEIR SIGNS, and the one number that
/// converts that side's shaft radians into inches of travel.
struct DriveSideDescription {
    /// The side's drive ports, |p| in 1..kMaxSmartPorts, SIGN = the PROS reversal: `-11` is
    /// port 11 constructed with a negative port number, which PROS reverses exactly once, in
    /// the adapter. THE SIGN LIVES HERE AND NOWHERE ELSE — a consumer hands the entry to its
    /// adapter AS STORED and never negates again (hal/pros/motor.hpp; hal/motor_group.hpp's
    /// "one place for a sign, or the sign gets applied twice"). Order is fixed but
    /// informational; a side is a set. Entries beyond `count` are 0 and are not read.
    std::int8_t signedPorts[kMaxSmartPorts]{};
    /// How many entries of `signedPorts` are real. 0 = NO SIDE, which is invalid: a drivetrain
    /// with a side nobody found is not a drivetrain.
    std::uint8_t count = 0;
    /// Inches of wheel travel per radian of this side's motor output-shaft rotation — the whole
    /// conversion, as one measured number (hal::DriveGeometry::fromInchesPerRadian). Must be
    /// finite and > 0; 0 is UNSET and refuses rather than silently scaling travel by nothing.
    units::Length inchesPerRadian{0.0};
};

/// Everything a composition root needs to build a drivetrain's object graph, from whichever of
/// the two sources produced it. Plain data (designated initializers at the call site), fixed
/// width, allocation-free, serializable byte by byte. Nothing validates itself on construction —
/// a caller builds one and then asks valid(), which is what lets a host test build a
/// contradictory one by hand and watch it be refused.
struct DrivetrainDescription {
    /// Which drivetrain this is. HUMAN-STATED.
    DrivetrainKind kind = DrivetrainKind::Tank;
    /// The cartridge in the drive motors. HUMAN-STATED (see MotorCartridge). Default-constructed
    /// as the byte 0, which is NOT a member of the enum and which valid() refuses on purpose: a
    /// description nobody filled in must never look filled in.
    MotorCartridge cartridge = static_cast<MotorCartridge>(0);
    /// The IMU's smart port, 1..kMaxSmartPorts. 0 = UNSET, and invalid: every measurement this
    /// type carries was taken against an IMU, and a description without one cannot be re-checked.
    std::uint8_t imuPort = 0;
    /// The LEFT side — kinematic wheel 0 (kinematics/tank.hpp's wheel order).
    DriveSideDescription left{};
    /// The RIGHT side — kinematic wheel 1.
    DriveSideDescription right{};
    /// Contact line to contact line, inches, > 0. From a tape (TypedTable) or EFFECTIVE, from a
    /// powered spin compared against the IMU (RobotMeasured; A4 register HA-135) — and those are
    /// not the same number:
    /// a turning tank scrubs on foam, so the encoders over-count the turn and the measured value
    /// comes out LARGER than the tape's. The effective one is what TankKinematics needs to command
    /// a turn that lands, and what the odometry's heading cross-check reads as zero.
    units::Length trackWidth{0.0};
    /// Which producer filled this in.
    DescriptionSource source = DescriptionSource::TypedTable;
    /// Who measured what, how, and when — printed in every banner. A fixed-width, NUL-terminated
    /// buffer rather than a pointer, so the whole struct can be written to a card as it stands.
    char provenance[96]{};

    /// True when nothing in the description contradicts itself. `why` (optional; pass nullptr and
    /// 0 to skip) receives the FIRST contradiction found, naming the field or the port. Each check
    /// writes its own reason with a LITERAL format string rather than through a shared helper: the
    /// on-robot build gate compiles src/ under -Wformat-nonliteral, and a helper taking `const
    /// char* format` defeats the compiler's argument checking exactly where a wrong specifier
    /// would corrupt the one message a person gets when the robot refuses to boot. The
    /// checks are exactly the mistakes a typo, a mis-wire or a corrupted record produce:
    /// an unknown kind or cartridge byte; an IMU port outside 1..21; a side with no ports or with
    /// more than the brain has; an entry that is not a port in 1..21; a port listed twice on one
    /// side or once on each; the IMU port doubling as a drive port; a non-finite or non-positive
    /// inches-per-radian on either side; a non-finite or non-positive track width.
    [[nodiscard]] bool valid(char* why = nullptr, std::size_t cap = 0) const noexcept {
        if (why != nullptr && cap > 0) {
            why[0] = '\0';
        }
        if (kind != DrivetrainKind::Tank) {
            {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "drivetrain kind %d is not Tank",
                                  static_cast<int>(static_cast<std::uint8_t>(kind)));
                }
                return false;
            }
        }
        if (cartridge != MotorCartridge::Red && cartridge != MotorCartridge::Green
            && cartridge != MotorCartridge::Blue) {
            {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "cartridge byte %d is not Red, Green or Blue (UNSET is 0)",
                                  static_cast<int>(static_cast<std::uint8_t>(cartridge)));
                }
                return false;
            }
        }
        if (imuPort < 1 || imuPort > static_cast<std::uint8_t>(kMaxSmartPorts)) {
            {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "IMU port %d is not a port in 1..21 (0 = UNSET)",
                                  static_cast<int>(imuPort));
                }
                return false;
            }
        }
        const DriveSideDescription* sides[2] = {&left, &right};
        const char* names[2] = {"LEFT", "RIGHT"};
        for (int s = 0; s < 2; ++s) {
            const DriveSideDescription& side = *sides[s];
            if (side.count == 0) {
                {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap, "%s side has no ports", names[s]);
                    }
                    return false;
                }
            }
            if (side.count > static_cast<std::uint8_t>(kMaxSmartPorts)) {
                {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "%s side claims %d ports; the brain has 21",
                                      names[s], static_cast<int>(side.count));
                    }
                    return false;
                }
            }
            for (std::size_t i = 0; i < side.count; ++i) {
                const int a = portMagnitude(side.signedPorts[i]);
                if (a < 1 || a > static_cast<int>(kMaxSmartPorts)) {
                    {
                        if (why != nullptr && cap > 0) {
                            std::snprintf(why, cap,
                                          "%s entry %d is not a port in 1..21",
                                          names[s], static_cast<int>(side.signedPorts[i]));
                        }
                        return false;
                    }
                }
                if (a == static_cast<int>(imuPort)) {
                    {
                        if (why != nullptr && cap > 0) {
                            std::snprintf(why, cap,
                                          "port %d is both %s and the IMU port",
                                          a, names[s]);
                        }
                        return false;
                    }
                }
                for (std::size_t j = i + 1; j < side.count; ++j) {
                    if (portMagnitude(side.signedPorts[j]) == a) {
                        {
                            if (why != nullptr && cap > 0) {
                                std::snprintf(why, cap, "%s lists port %d twice", names[s], a);
                            }
                            return false;
                        }
                    }
                }
                if (s == 0) {
                    for (std::size_t j = 0; j < right.count; ++j) {
                        if (portMagnitude(right.signedPorts[j]) == a) {
                            {
                                if (why != nullptr && cap > 0) {
                                    std::snprintf(why, cap, "port %d appears on BOTH sides", a);
                                }
                                return false;
                            }
                        }
                    }
                }
            }
            const double ipr = side.inchesPerRadian.value();
            if (!std::isfinite(ipr) || ipr <= 0.0) {
                {
                    if (why != nullptr && cap > 0) {
                        std::snprintf(why, cap,
                                      "%s inches per radian is %g — must be finite and > 0 (0 = "
                                      "UNSET)",
                                      names[s], ipr);
                    }
                    return false;
                }
            }
        }
        const double width = trackWidth.value();
        if (!std::isfinite(width) || width <= 0.0) {
            {
                if (why != nullptr && cap > 0) {
                    std::snprintf(why, cap,
                                  "track width is %g — must be finite and > 0 (0 = UNSET)",
                                  width);
                }
                return false;
            }
        }
        return true;
    }

    /// The LEFT side's geometry for the odometry and the stall check — one object, built from
    /// the one measured number (DriveGeometry::fromInchesPerRadian, so `wheelRadius` carries the
    /// product). Only meaningful on a valid() description.
    [[nodiscard]] constexpr DriveGeometry leftGeometry() const noexcept {
        return DriveGeometry::fromInchesPerRadian(left.inchesPerRadian);
    }

    /// The RIGHT side's geometry — a separate object, because an asymmetric drivetrain is
    /// representable and has been seen.
    [[nodiscard]] constexpr DriveGeometry rightGeometry() const noexcept {
        return DriveGeometry::fromInchesPerRadian(right.inchesPerRadian);
    }

    /// −1 LEFT, +1 RIGHT, 0 = not a drive port in this description. Matches on |port|, so a
    /// signed entry and a physical port number find each other.
    [[nodiscard]] constexpr int sideOf(int port) const noexcept {
        for (std::size_t i = 0; i < left.count; ++i) {
            if (portMagnitude(left.signedPorts[i]) == port) {
                return -1;
            }
        }
        for (std::size_t i = 0; i < right.count; ++i) {
            if (portMagnitude(right.signedPorts[i]) == port) {
                return +1;
            }
        }
        return 0;
    }

    /// This description's entry for physical `port` AS STORED — signed — or 0 when the port is
    /// not a drive port here. THIS is the value a composition root hands to its motor adapter:
    /// the one place a sign lives.
    [[nodiscard]] constexpr std::int8_t signedPortOf(int port) const noexcept {
        for (std::size_t i = 0; i < left.count; ++i) {
            if (portMagnitude(left.signedPorts[i]) == port) {
                return left.signedPorts[i];
            }
        }
        for (std::size_t i = 0; i < right.count; ++i) {
            if (portMagnitude(right.signedPorts[i]) == port) {
                return right.signedPorts[i];
            }
        }
        return 0;
    }

    /// The one-line panel text: source, kind, cartridge, port counts, both scales, the width.
    /// Truncates rather than overflowing; writes at least a NUL whenever cap > 0.
    void describe(char* buf, std::size_t cap) const {
        if (buf == nullptr || cap == 0) {
            return;
        }
        std::snprintf(buf, cap, "%s %s %s: L%d/R%d ports, ipr L%.5f R%.5f, W %.3f in",
                      source == DescriptionSource::RobotMeasured ? "MEASURED" : "TYPED",
                      kind == DrivetrainKind::Tank ? "tank" : "?", cartridgeWord(cartridge),
                      static_cast<int>(left.count), static_cast<int>(right.count),
                      left.inchesPerRadian.value(), right.inchesPerRadian.value(),
                      trackWidth.value());
    }

    /// |p| for a (possibly signed) port entry, as an int.
    [[nodiscard]] static constexpr int portMagnitude(std::int8_t p) noexcept {
        return p < 0 ? -static_cast<int>(p) : static_cast<int>(p);
    }

};

}  // namespace shulib::hal
