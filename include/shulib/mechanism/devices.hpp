#pragma once
//
// mechanism/devices.hpp — the DEVICES one mechanism was built over, and what the build found for
// each of its ROLES (the mechanism core, chunk MECH-A1, 2026-10-08).
//
// robot::MechanismParts (robot/mechanism_parts.hpp) builds every role a mechanism declares — its
// motors or lines, its angle sensor, its home switch, its piece sensor — and hands the result to
// the mechanism service (MECH-A2) as one `Devices` per mechanism, and to the boot lines as one
// `RoleFact` per role. HAL POINTERS ONLY, and no other mechanism/ header included, so robot/ may
// produce these without depending on the vocabulary above it (mechanism/kinds.hpp, wiring.hpp).
//
// nullptr in a `Devices` = that role is not on this mechanism, or the mechanism is ABSENT, or (a
// piece sensor) it was refused and the mechanism runs WITHOUT one. The read-fault pointers are the
// provider's IDeviceProvider::readFaults / lineReadFaults for that sensor (hal/read_faults.hpp):
// the service reads a sample taken while one moved as HELD, never as evidence (MECH-A2's rule).
//
// PROS-free, allocation-free.

#include <cstdint>

#include "shulib/hal/digital_in.hpp"
#include "shulib/hal/distance.hpp"
#include "shulib/hal/mechanism.hpp"
#include "shulib/hal/optical.hpp"
#include "shulib/hal/read_faults.hpp"
#include "shulib/hal/rotation.hpp"

namespace shulib::mechanism {

/// The devices one mechanism was built over (header).
struct Devices {
    /// A motor kind's actuator (power, position, speed); nullptr for a pneumatic or when ABSENT.
    hal::MotorMechanism* motors = nullptr;
    /// A pneumatic kind's actuator; nullptr for a motor kind or when ABSENT.
    hal::PneumaticMechanism* pneumatic = nullptr;
    /// Its angle sensor (a Rotation role).
    hal::IRotation* angleSensor = nullptr;
    /// Its home switch (a limit switch on a 3-wire line).
    hal::IDigitalIn* homeSwitch = nullptr;
    /// Its piece sensor when it is a distance sensor (at most one of the three piece pointers).
    hal::IDistance* pieceDistance = nullptr;
    /// Its piece sensor when it is an optical sensor.
    hal::IOptical* pieceOptical = nullptr;
    /// Its piece sensor when it is a limit switch.
    hal::IDigitalIn* pieceSwitch = nullptr;
    /// The piece sensor's READ-FAULT count (the provider's readFaults(port) / lineReadFaults(line)):
    /// a sample taken while it moved was a HELD value, never evidence. nullptr = no piece sensor, or
    /// one that never screens a read.
    const hal::IReadFaults* pieceFaults = nullptr;
    /// The home switch's read-fault count (lineReadFaults(line)).
    const hal::IReadFaults* homeSwitchFaults = nullptr;
    /// Why the mechanism was NOT built — the provider's sentence naming the port; "" when it was.
    const char* absence = "";
};

/// What the build found for ONE ROLE — the facts its boot line words (mechanism::roleLine, in
/// mechanism/wiring.hpp).
enum class RoleState : std::uint8_t {
    Present,       ///< a smart device of the expected kind answered on its port
    Refused,       ///< the provider refused it (its sentence names the port and what was found)
    NotCheckable,  ///< a 3-wire role: made, but the brain cannot see a 3-wire device (HA-158)
    NotAsked,      ///< not requested: the mechanism was already ABSENT
};

/// One role of one mechanism, as the build left it — what a boot line needs.
struct RoleFact {
    /// The role in the boot line's words: "motor 1".."motor 4", "line 1", "line 2", "angle sensor",
    /// "home switch", "piece sensor".
    const char* role = "";
    /// Its smart port, physical (1..21); 0 for a 3-wire role.
    std::uint8_t port = 0;
    /// Its 3-wire line, 1..8 = A..H; 0 for a smart role.
    std::uint8_t line = 0;
    /// What the build found.
    RoleState state = RoleState::NotAsked;
    /// A refusal that leaves the mechanism RUNNING WITHOUT this role (a piece sensor), rather than
    /// ABSENT.
    bool degradeOnly = false;
    /// The provider's refusal sentence when `state` is Refused (stable for the parts' lifetime);
    /// "" otherwise.
    const char* refusal = "";
};

}  // namespace shulib::mechanism
