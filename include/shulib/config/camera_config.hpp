#pragma once
//
// CameraConfig — the robot file's AI VISION CAMERA block (chunk VIS1-A1, 2026-10-18).
//
// ── DATA the robot file carries, so it lives in config/ ─────────────────────────────
// The robot file is the one place a robot's facts live (MECH's Q1 (a)); the camera's facts are its
// port, what the library writes to it (the colour descriptors the team measured, the AI model's
// OBJECTS on or off, the class names the model must report — hal::AiVisionSettings), and where it sits
// (CameraPlacement, measured). config/robot_config.hpp carries only the hooks: `SensorsConfig::camera`,
// `PortUse::Camera`, and the camera's checks in `RobotConfig::valid()`. G3a's robot panel writes this
// block like every other.
//
// ── The placement is measured from the robot's TRACKING CENTRE ──────────────────────
// The point the robot's pose names (guide chapter 2, "Where does the robot's position actually
// point?"): every consumer turns the camera's bearing and range into a FIELD position by adding them to
// that pose, through this placement. On a tank with drive-encoder odometry and no tracking pods it is
// the point the robot turns about in place.
//
// ── UNSET is not an error ───────────────────────────────────────────────────────────
// `CameraPlacement::measured == false` (the default) is UNSET: the camera is still built and read (its
// colours and objects need no mount), and every GEOMETRY consumer refuses to construct, naming it
// (vision::cameraModelProblem). So the team can bring a camera up a piece at a time — the objects
// first, the colours once they are measured, the mount once it is measured — and no value is ever
// invented to make a block "complete" (HA-111's lesson).
//
// PROS-free, sim-free, allocation-free, constexpr-constructible.

#include <cstdint>

#include "shulib/hal/vision_frames.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::config {

/// Where the camera's LENS sits and where its optical axis points, in the body frame (+X forward, +Y
/// LEFT, +Z up from the floor) — MEASURED by the team, typed once per robot with how well each number
/// was measured. `measured == false` (the default) is UNSET (header).
struct CameraPlacement {
    bool measured = false;              ///< true only when every field below was measured
    units::Length forward{};            ///< the lens FORWARD of the tracking centre
    units::Length left{};               ///< the lens LEFT of the tracking centre
    units::Length height{};             ///< the lens above the FLOOR (the field tiles), to the lens centre
    double yawDeg = 0.0;                ///< the optical axis, CCW from the robot's +X (0 = straight ahead), degrees
    double pitchDeg = 0.0;              ///< the optical axis above the horizontal (+ = tilted UP), degrees
    units::Length positionTolerance{};  ///< +- on forward and left (a tape: about 0.25 in)
    units::Length heightTolerance{};    ///< +- on height
    double yawToleranceDeg = 0.0;       ///< +- on yaw, degrees
    double pitchToleranceDeg = 0.0;     ///< +- on pitch, degrees (a phone level on the camera's back face: about 1;
                                        ///< the geometry refuses a pitch it takes past 55: vision::kMaxPitchSpanDeg)
    const char* provenance = "";        ///< who measured, how, when — and the firmware versions — required when measured
};

/// True when `v` is a finite number (constexpr: neither NaN nor an infinity).
[[nodiscard]] constexpr bool finiteNumber(double v) noexcept {
    return v == v && (v - v) == 0.0;  // NaN fails the first; an infinity gives inf - inf = NaN
}

/// The first problem with a placement, or nullptr: UNSET; a position that is not a number; a height
/// outside (0, 36) in; a pitch outside (-45, 45) degrees; a yaw outside (-180, 180]; a tolerance that is
/// zero, negative, not a number or infinite (nothing is measured perfectly — a zero would make every
/// interval a point and every refusal impossible); no provenance.
[[nodiscard]] constexpr const char* cameraPlacementProblem(const CameraPlacement& p) noexcept {
    if (!p.measured) {
        return "the camera's mount is UNSET -- measure it from the robot's tracking centre and type it";
    }
    if (!finiteNumber(p.forward.value()) || !finiteNumber(p.left.value())) {
        return "the camera's forward or left offset is not a number";
    }
    if (!(p.height.value() > 0.0 && p.height.value() < 36.0)) {
        return "camera height outside (0, 36) in";
    }
    if (!(p.pitchDeg > -45.0 && p.pitchDeg < 45.0)) {
        return "pitch outside +-45 degrees";
    }
    if (!(p.yawDeg > -180.0 && p.yawDeg <= 180.0)) {
        return "yaw outside (-180, 180] degrees";
    }
    const double tolerances[] = {p.positionTolerance.value(), p.heightTolerance.value(),
                                 p.yawToleranceDeg, p.pitchToleranceDeg};
    for (const double t : tolerances) {
        if (!(t > 0.0) || !finiteNumber(t)) {
            return "a mount tolerance is zero, UNSET or not a number -- state how well it was measured";
        }
    }
    if (p.provenance == nullptr || p.provenance[0] == '\0') {
        return "no provenance -- who measured the mount, how, when?";
    }
    return nullptr;
}

/// The robot's AI Vision camera: its port, what the adapter writes to it, and where it is (header).
struct CameraConfig {
    std::uint8_t port = 0;                ///< 1..21; 0 = this robot has no camera
    hal::AiVisionSettings settings{};     ///< what the adapter writes (colours, objects, the class table)
    CameraPlacement placement{};          ///< UNSET until it is measured
    const char* note = "";                ///< e.g. "front, centred" — for a boot line and the panel
};

}  // namespace shulib::config
