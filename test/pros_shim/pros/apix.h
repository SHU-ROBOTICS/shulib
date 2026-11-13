#pragma once
//
// HOST SHIM for <pros/apix.h> — the SMART-PORT REGISTRY only (chunk R3d Part 2).
//
// The real apix.h is the Extended API: RTOS primitives, serial, file IO, the whole of
// LVGL's C surface. This shim carries the ONE thing the library binds from it —
// `registry_get_plugged_type` and the `v5_device_e_t` enum it returns — and nothing else,
// deliberately: a shim that models an API nobody calls is a second source of beliefs that
// no test can check.
//
// Beliefs modeled (registered in docs/hardware-assumptions.md):
//  * `v5_device_e_t`'s VALUES, transcribed verbatim from the vendored pros/device.h:48-63
//    (NONE 0, MOTOR 2, ROTATION 4, IMU 6, DISTANCE 7, RADIO 8, VISION 11, ADI 12,
//    OPTICAL 16, GPS 20, AIVISION 29, SERIAL 129, UNDEFINED 255). The gaps are real; the
//    values are not consecutive, and an adapter that assumed they were would map every
//    device wrong.
//  * `registry_get_plugged_type(uint8_t port)` is ZERO-INDEXED: "Returns the type of the
//    device plugged into the zero-indexed port … The V5 port number from 0-20"
//    (vendored apix.h:640-660). HA-138. THE SHIM IS INDEXED THE SAME WAY, so the adapter's
//    physical→index conversion is the thing under test rather than a convention the shim
//    quietly shares with it: `shim::registryState().kinds[i]` is what index i answers, and
//    the adapter test asserts that PHYSICAL port 1 reads slot 0 and PHYSICAL port 21 reads
//    slot 20.
//  * ADVERSARIAL DEFAULT: every index answers E_DEVICE_NONE. A test that forgets to
//    populate the registry sees an empty robot and gets a refusal, never a lucky pass.
//    Reading past index 20 answers E_DEVICE_NONE too — the real call sets ENXIO there, and
//    2026-08-18 showed what believing index 21 costs (HA-120).
//
// HONEST LIMIT (shim_control.hpp carries it in full): this tests the ADAPTER against our
// BELIEF about PROS; it cannot test the belief. Hardware tests the belief — the worksheet's
// Station G census beside the brain's own Devices screen.

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstddef>
#include <cstdint>

namespace pros::c {

/// Transcribed VERBATIM from the vendored pros/device.h:48-63 — the values are not
/// consecutive and the gaps are real.
typedef enum v5_device_e {
    E_DEVICE_NONE = 0,       ///< No device is plugged into the port
    E_DEVICE_MOTOR = 2,      ///< A motor is plugged into the port
    E_DEVICE_ROTATION = 4,   ///< A rotation sensor is plugged into the port
    E_DEVICE_IMU = 6,        ///< An inertial sensor is plugged into the port
    E_DEVICE_DISTANCE = 7,   ///< A distance sensor is plugged into the port
    E_DEVICE_RADIO = 8,      ///< A radio is plugged into the port
    E_DEVICE_VISION = 11,    ///< A vision sensor is plugged into the port
    E_DEVICE_ADI = 12,       ///< This port is an ADI expander
    E_DEVICE_OPTICAL = 16,   ///< An optical sensor is plugged into the port
    E_DEVICE_GPS = 20,       ///< A GPS sensor is plugged into the port
    E_DEVICE_AIVISION = 29,  ///< An AI Vision sensor is plugged into the port
    E_DEVICE_SERIAL = 129,   ///< A serial device is plugged into the port
    E_DEVICE_UNDEFINED = 255 ///< The device type is not defined, or is not a valid device
} v5_device_e_t;

}  // namespace pros::c

namespace pros::shim {

/// The registry as the brain would report it: ZERO-INDEXED, index i = PHYSICAL PORT i + 1,
/// exactly as the vendored apix.h documents. 21 slots, indices 0..20.
struct RegistryState {
    /// What each registry INDEX answers. ADVERSARIAL DEFAULT: all empty.
    std::array<::pros::c::v5_device_e_t, 21> kinds{};

    /// Every index that has been read, in order, so a test can assert that an out-of-range
    /// PHYSICAL port never reached the platform at all.
    int queries = 0;
    /// The last index the adapter actually asked for (-1 = none) — the direct evidence that
    /// physical port p became index p − 1.
    int lastIndex = -1;
};

inline RegistryState& registryState() {
    static RegistryState s;
    return s;
}

inline void resetRegistry() { registryState() = RegistryState{}; }

/// Put a device on a PHYSICAL port (1..21), converting to the registry's index HERE so a
/// test never has to do the arithmetic the adapter is being tested on. Out of range is a
/// no-op: the shim cannot represent a port the brain does not have.
inline void setPluggedAtPort(int physicalPort, ::pros::c::v5_device_e_t kind) {
    if (physicalPort < 1 || physicalPort > 21) {
        return;
    }
    registryState().kinds[static_cast<std::size_t>(physicalPort - 1)] = kind;
}

}  // namespace pros::shim

namespace pros::c {

/// Vendored contract (apix.h:640-660): "Returns the type of the device plugged into the
/// ZERO-INDEXED port … The V5 port number from 0-20." Out of range answers E_DEVICE_NONE
/// here (the real call sets ENXIO and returns an unspecified value; believing index 21 is
/// what produced the phantom ADI expander on 2026-08-18 — HA-120).
inline v5_device_e_t registry_get_plugged_type(std::uint8_t port) {
    ::pros::shim::RegistryState& s = ::pros::shim::registryState();
    ++s.queries;
    s.lastIndex = static_cast<int>(port);
    if (port >= s.kinds.size()) {
        return E_DEVICE_NONE;
    }
    return s.kinds[static_cast<std::size_t>(port)];
}

}  // namespace pros::c
