#pragma once
//
// IDeviceRegistry — WHAT IS ACTUALLY PLUGGED INTO EACH SMART PORT (chunk R3d Part 2).
//
// ── Why the library needs this at all ───────────────────────────────────────────────
// Every other HAL seam asks a device a question. This one asks the BRAIN a question the
// devices cannot answer: which ports have something in them, and what kind of something.
// Two things in R3d need exactly that and nothing more:
//   * "shulib Calibrate" builds its candidate list from the ports that carry a MOTOR,
//     because a calibration that starts from a typed list can only ever confirm the typing;
//   * "shulib Teleop" checks a calibration record against the ports BEFORE it constructs a
//     single adapter, so a card carried over from another robot, or a robot re-plugged
//     since the calibration, is refused instead of driven (calibration/boot_check.hpp).
// Neither wants to open a device to find out. Constructing an adapter to discover whether
// a device exists is how HA-111's invented port map turned into ten throwing constructors
// at boot: the probe must be cheaper than the thing it is probing for.
//
// ── PHYSICAL PORTS, 1..21 — and the trap this seam exists to contain ────────────────
// PROS's `registry_get_plugged_type()` is ZERO-INDEXED and nothing else in PROS is
// (apix.h: "the zero-indexed port … The V5 port number from 0-20"). Scanning 1..21 as if it
// were physical shifts every reading by one port and invents a device at index 21 — which
// is exactly what happened on 2026-08-18 and produced a confident, completely wrong port
// map, and what HA-120 had already predicted. THIS INTERFACE IS STATED IN PHYSICAL PORTS,
// 1..kMaxSmartPorts, like every other port number a human or any other PROS API uses; the
// conversion lives in ONE line, in hal/pros/device_registry.hpp, and is tested from both
// ends. A port outside 1..21 answers None WITHOUT asking the platform — so a loop that
// walks 0..21 by mistake cannot reach past the end.
//
// ── DeviceKind is OURS, not PROS's ──────────────────────────────────────────────────
// A small enum of the kinds this tree has adapters for, plus Other for everything else.
// Deliberately not a re-export of `v5_device_e_t`: that would put a PROS type in the
// library's vocabulary (the PROS-free rule) and would make every consumer switch over
// values it has no opinion about. The mapping is one function in the adapter, and the
// adapter's test walks EVERY v5_device_e_t value so a new PROS device kind cannot silently
// become a motor.
//
// PROS-free, allocation-free, const — a registry is asked, never told.

#include <cstdint>

#include "shulib/hal/drivetrain_description.hpp"

namespace shulib::hal {

/// What kind of device a smart port carries. Ours, not PROS's (header): the kinds this tree
/// has adapters for, plus Other for a device it knows nothing about. `None` is an EMPTY port
/// — no device, or a port outside 1..kMaxSmartPorts, which answers None without asking the
/// platform.
enum class DeviceKind : std::uint8_t {
    None,      ///< nothing plugged in (or the port number is out of range)
    Motor,     ///< a V5 smart motor — hal/pros/motor.hpp
    Imu,       ///< an inertial sensor — hal/pros/imu.hpp
    Rotation,  ///< a rotation sensor — hal/pros/rotation.hpp
    Distance,  ///< a distance sensor — hal/pros/distance.hpp
    Optical,   ///< an optical sensor — hal/pros/optical.hpp
    Vision,    ///< a vision sensor (including AI Vision) — no adapter yet; R2 owns the camera
    Gps,       ///< a GPS sensor — hal/pros/gps.hpp
    Radio,     ///< the VEXnet radio
    Adi,       ///< an ADI (three-wire) expander — hal/pros/digital_in.hpp's pair form
    Other,     ///< a device this library has no name for (serial, undefined, anything new)
};

/// The kind as one short word, for a census line or a refusal message. Never invents:
/// an unknown byte reads "?".
[[nodiscard]] inline const char* deviceKindWord(DeviceKind k) noexcept {
    switch (k) {
        case DeviceKind::None: return "EMPTY";
        case DeviceKind::Motor: return "MOTOR";
        case DeviceKind::Imu: return "IMU";
        case DeviceKind::Rotation: return "ROTATION";
        case DeviceKind::Distance: return "DISTANCE";
        case DeviceKind::Optical: return "OPTICAL";
        case DeviceKind::Vision: return "VISION";
        case DeviceKind::Gps: return "GPS";
        case DeviceKind::Radio: return "RADIO";
        case DeviceKind::Adi: return "ADI";
        case DeviceKind::Other: return "OTHER";
    }
    return "?";
}

/// The seam that answers "what is plugged into this port?" without opening anything. Stated
/// in PHYSICAL port numbers, 1..kMaxSmartPorts — the zero-indexed PROS call it is implemented
/// over is the adapter's business and nobody else's (header). Non-owning and const: a
/// registry is asked, never told.
class IDeviceRegistry {
public:
    /// The polymorphic-base special members: a virtual destructor so a registry may be owned
    /// and destroyed through this interface, and defaulted copy/move because the interface
    /// itself holds no state.
    virtual ~IDeviceRegistry() = default;
    IDeviceRegistry() = default;
    IDeviceRegistry(const IDeviceRegistry&) = default;
    IDeviceRegistry(IDeviceRegistry&&) = default;
    IDeviceRegistry& operator=(const IDeviceRegistry&) = default;
    IDeviceRegistry& operator=(IDeviceRegistry&&) = default;

    /// What PHYSICAL port `port` (1..kMaxSmartPorts) currently carries. Out of range answers
    /// None without asking the platform — a loop that walks 0..21, or 1..22, gets an honest
    /// "empty" rather than a shifted reading or a phantom device (header, and HA-120's
    /// 2026-08-18 finding). MUST NOT throw.
    [[nodiscard]] virtual DeviceKind pluggedKind(int port) const noexcept = 0;
};

/// True when `port` is a physical smart port number a registry may be asked about. The ONE
/// place the 1..kMaxSmartPorts range is spelled for this seam, so the fake, the adapter and
/// every caller agree about the boundary.
[[nodiscard]] inline constexpr bool isSmartPort(int port) noexcept {
    return port >= 1 && port <= static_cast<int>(kMaxSmartPorts);
}

}  // namespace shulib::hal
