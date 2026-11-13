#pragma once
//
// WallSensorsConfig — the robot file's SIDE DISTANCE SENSORS block (chunk L6-A2, 2026-11-03; the L6
// brief's D5, D6 and D7 — the team lead's answers of 2026-10-22).
//
// ── DATA the robot file carries, so it lives in config/ ─────────────────────────────
// The robot file is the one place a robot's facts live (VIS1's camera block is the template): one
// V5 Distance Sensor on each SIDE of the robot, facing straight out, each with its port and its
// MEASURED mount (localization::SideMount), and the robot's wall-reset gate limits
// (localization::WallResetLimits — the library's PROVISIONAL defaults unless the team typed its own).
// config/robot_config.hpp carries only the hooks: `SensorsConfig::walls`, `PortUse::WallSensor`, and
// these checks in `RobotConfig::valid()`. robot::Robot builds each declared side through the device
// provider; G3a's robot panel is to write this block like every other (it writes none yet).
//
// ── UNSET is not an error ───────────────────────────────────────────────────────────
// `SideMount::measured == false` (the default) is UNSET: the sensor is still built, and every wall
// reset with it refuses `MOUNT UNSET` — never the robot. So a robot boots and drives the day its
// sensors are plugged in, and the reset starts working the day someone measures the mount with a
// tape. No value is ever invented to make a block "complete" (HA-111's lesson): `valid()` judges a
// mount's numbers only once it says `measured = true`.
//
// ── What valid() refuses, and why each bound ────────────────────────────────────────
// A MEASURED mount: a number that is not finite; a `facing` outside (-180, 180] degrees (the likeliest
// typo: `90.0` typed alone is 90 RADIANS — every angle here is `N * kWallDegToRad`); a sensor that does
// not face out of, or sit on, its OWN side (`wallSideProblem`: the LEFT one's facing between 0 and 180
// degrees and its `left` above 0, the RIGHT one's mirrored — a block copied from the other side would
// refuse every reset NOT SQUARE, or put the answer about twice the offset off); the lens more than 24 in from the robot's centre
// forward or sideways (bigger than any robot this season may be — a typo, not a robot); a height of 0
// or less, or above 11.5 in (the field wall's top, A4 register HA-208: a lens above it reads past the
// wall, never the wall); a measured beam half-angle outside 3–30 degrees (0 = unmeasured; A4 register
// HA-205); a position tolerance of 0 or less or above 2 in, a facing tolerance of 0 or less or above
// 10 degrees (nothing is measured perfectly, and a tolerance that loose is not a measurement); a range
// bias above 2 in either way; a measured accuracy outside 0.05–2 in (0 = unmeasured: VEX's published
// figures, A4 register HA-204); no provenance (who measured it, how, when).
// THE LIMITS, always: everything the wall reset itself would refuse as a bad request (so a valid robot
// file never meets BAD REQUEST from its own limits), and tighter, so a typed limit stays inside the
// span its derivation covers (A4 register HA-211): a range floor under 0.79 in (VEX's 20 mm) or a
// ceiling not above it; fewer than 3 or more than 64 samples; a window longer than its own maximum, a
// maximum past 1 s, a window past 0.6 s; a heading floor outside 0.5–10 degrees; a heading growth
// outside 0–0.1 per radian turned; an along-wall drift rate outside 0–0.5; a piece top or a beam
// margin below 0; and the rest's own ranges.
//
// The angle edges are compared in RADIANS against `N * kWallDegToRad` — the very value a robot file
// types — so a number typed exactly on an edge is accepted exactly (localization/wall_reset.hpp's
// wallResetProblem does the same, for the same reason).
//
// PROS-free, sim-free, allocation-free, constexpr-constructible (robot two's block sits in a
// constexpr table, L6-B).

#include <cstdint>

#include "shulib/config/camera_config.hpp"  // config::finiteNumber — one constexpr finiteness check for config/
#include "shulib/localization/wall_reset.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::config {

/// The farthest a side sensor's lens may sit from the robot's centre, forward or sideways (inches):
/// a VEX U robot is at most 24 in across, so more is a typo, not a mount.
inline constexpr double kWallMountMaxOffsetInches = 24.0;
/// The highest a side sensor's lens may sit (inches): the field wall's top — manual drawing A14's
/// 11.50 in (the metal perimeter; A13's portable one is 11.54). A lens above it reads past the wall
/// (A4 register HA-208).
inline constexpr double kWallMountMaxHeightInches = 11.5;
/// The widest position tolerance a measured mount may state (inches).
inline constexpr double kWallMountMaxPositionToleranceInches = 2.0;
/// The widest facing tolerance a measured mount may state (degrees).
inline constexpr double kWallMountMaxFacingToleranceDeg = 10.0;
/// The largest range bias a measured mount may state, either way (inches).
inline constexpr double kWallMountMaxBiasInches = 2.0;
/// The narrowest and widest measured accuracy a mount may state (inches); 0 = unmeasured.
inline constexpr double kWallMountMinAccuracyInches = 0.05;
/// See kWallMountMinAccuracyInches.
inline constexpr double kWallMountMaxAccuracyInches = 2.0;
/// The shortest `WallResetLimits::rangeMin` a robot file may type (inches): VEX's published 20-mm
/// floor (0.787 in), rounded up (A4 register HA-204).
inline constexpr double kWallRangeFloorInches = 0.79;
/// The fewest `WallResetLimits::minSamples` a robot file may type: a median of fewer than three
/// samples cannot outvote one outlier.
inline constexpr int kWallMinSamplesFloor = 3;

/// One side's distance sensor, as the robot file declares it: its port, and its MEASURED mount.
struct WallSensorConfig {
    /// The sensor's smart port, 1..21; 0 = no sensor on this side (the default).
    std::uint8_t port = 0;
    /// Where the sensor sits on the robot and how well that was measured — UNSET until measured
    /// (header): the sensor is built, and every reset with it refuses MOUNT UNSET.
    localization::SideMount mount{};
};

/// The robot's side distance sensors (header): one per side, and the wall reset's limits.
struct WallSensorsConfig {
    /// The LEFT side's sensor (its beam out the robot's left; `facing` typically
    /// `90.0 * localization::kWallDegToRad`).
    WallSensorConfig left{};
    /// The RIGHT side's sensor (`facing` typically `-90.0 * localization::kWallDegToRad`).
    WallSensorConfig right{};
    /// The wall reset's gate limits for this robot — PROVISIONAL library defaults unless typed (A4
    /// register HA-211).
    localization::WallResetLimits limits{};
    /// A note for a boot line and the panel, e.g. "both centred, 10.5 in up" — a stable literal.
    const char* note = "";

    /// True when either side declares a sensor (a port that is not 0).
    [[nodiscard]] constexpr bool declared() const noexcept { return left.port != 0 || right.port != 0; }
};

namespace detail {

/// |v|, constexpr.
[[nodiscard]] constexpr double wallAbs(double v) noexcept { return v < 0.0 ? -v : v; }

}  // namespace detail

/// The first problem with a side sensor's MEASURED mount (header), or nullptr. An UNMEASURED mount
/// (`measured == false`) has none: it is not a config error — every reset with it refuses MOUNT UNSET.
[[nodiscard]] constexpr const char* wallMountProblem(const localization::SideMount& m) noexcept {
    using detail::wallAbs;
    constexpr double deg = localization::kWallDegToRad;
    if (!m.measured) {
        return nullptr;
    }
    if (!finiteNumber(m.forward.value()) || !finiteNumber(m.left.value()) || !finiteNumber(m.height.value())
        || !finiteNumber(m.facing.value()) || !finiteNumber(m.positionTolerance.value())
        || !finiteNumber(m.facingTolerance.value()) || !finiteNumber(m.rangeBias.value())
        || !finiteNumber(m.rangeAccuracy.value()) || !finiteNumber(m.coneHalfAngle.value())) {
        return "a number in the mount is not finite";
    }
    if (!(m.facing.value() > -180.0 * deg && m.facing.value() <= 180.0 * deg)) {
        return "facing outside (-180, 180] degrees -- type it as N * kWallDegToRad (90.0 alone is 90 RADIANS)";
    }
    if (wallAbs(m.forward.value()) > kWallMountMaxOffsetInches
        || wallAbs(m.left.value()) > kWallMountMaxOffsetInches) {
        return "the lens is more than 24 in from the robot's centre (forward or left) -- a typo?";
    }
    if (!(m.height.value() > 0.0) || m.height.value() > kWallMountMaxHeightInches) {
        return "height outside (0, 11.5] in -- a lens above the wall's top reads past it";
    }
    const double cone = m.coneHalfAngle.value();
    if (cone != 0.0 && !(cone >= 3.0 * deg && cone <= 30.0 * deg)) {
        return "coneHalfAngle must be 0 (unmeasured) or 3..30 degrees";
    }
    if (!(m.positionTolerance.value() > 0.0)
        || m.positionTolerance.value() > kWallMountMaxPositionToleranceInches) {
        return "positionTolerance must be above 0 and at most 2 in -- state how well it was measured";
    }
    if (!(m.facingTolerance.value() > 0.0)
        || m.facingTolerance.value() > kWallMountMaxFacingToleranceDeg * deg) {
        return "facingTolerance must be above 0 and at most 10 degrees -- state how square it was found";
    }
    if (wallAbs(m.rangeBias.value()) > kWallMountMaxBiasInches) {
        return "rangeBias is more than 2 in either way";
    }
    const double acc = m.rangeAccuracy.value();
    if (acc != 0.0 && !(acc >= kWallMountMinAccuracyInches && acc <= kWallMountMaxAccuracyInches)) {
        return "rangeAccuracy must be 0 (unmeasured) or 0.05..2 in";
    }
    if (m.provenance == nullptr || m.provenance[0] == '\0') {
        return "no provenance -- who measured the mount, how, when?";
    }
    return nullptr;
}

/// The first problem with a MEASURED mount for the side it is declared on (header), or nullptr — run
/// after wallMountProblem accepted it. A sensor on a side faces out of that side and sits on that side
/// of the robot's centre line: the LEFT one's `facing` strictly between 0 and 180 degrees (typically
/// +90) and its `left` above 0; the RIGHT one's `facing` strictly between -180 and 0 (typically -90) and
/// its `left` below 0. The usual cause is a block copied from the other side: a facing the wrong way
/// refuses every reset NOT SQUARE, and an offset the wrong way puts the answer about twice the offset
/// off — refused DISAGREES, or SEATED that far wrong under a generous `maxCorrection`. An UNMEASURED
/// mount has none.
[[nodiscard]] constexpr const char* wallSideProblem(const localization::SideMount& m,
                                                    localization::WallSide side) noexcept {
    if (!m.measured) {
        return nullptr;
    }
    constexpr double half = 180.0 * localization::kWallDegToRad;  // the edge as a robot file types it
    const double facing = m.facing.value();
    if (side == localization::WallSide::Left) {
        if (!(facing > 0.0 && facing < half)) {
            return "a LEFT sensor faces out the left side: facing between 0 and 180 degrees (typically +90) -- "
                   "copied from the RIGHT block?";
        }
        if (!(m.left.value() > 0.0)) {
            return "a LEFT sensor sits left of the robot's centre: `left` above 0 -- copied from the RIGHT block?";
        }
        return nullptr;
    }
    if (!(facing < 0.0 && facing > -half)) {
        return "a RIGHT sensor faces out the right side: facing between -180 and 0 degrees (typically -90) -- "
               "copied from the LEFT block?";
    }
    if (!(m.left.value() < 0.0)) {
        return "a RIGHT sensor sits right of the robot's centre: `left` below 0 -- copied from the LEFT block?";
    }
    return nullptr;
}

/// The first problem with a robot's wall-reset LIMITS (header), or nullptr. Every limits block this
/// accepts is one localization::wallResetProblem accepts too, so a valid robot file never meets BAD
/// REQUEST from its own limits.
[[nodiscard]] constexpr const char* wallLimitsProblem(const localization::WallResetLimits& l) noexcept {
    constexpr double deg = localization::kWallDegToRad;
    if (!finiteNumber(l.rangeMin.value()) || !finiteNumber(l.rangeMax.value())
        || l.rangeMin.value() < kWallRangeFloorInches || !(l.rangeMax.value() > l.rangeMin.value())) {
        return "rangeMin/rangeMax must satisfy 0.79 in (VEX's 20 mm) <= rangeMin < rangeMax, both finite";
    }
    if (!(l.squareWithin.value() > 0.0 && l.squareWithin.value() <= 20.0 * deg)) {
        return "squareWithin must be above 0 and at most 20 degrees";
    }
    if (!(l.coneHalfAngle.value() >= 3.0 * deg && l.coneHalfAngle.value() <= 30.0 * deg)) {
        return "coneHalfAngle must be 3..30 degrees";
    }
    if (!(l.spreadMax.value() > 0.0) || !finiteNumber(l.spreadMax.value())) {
        return "spreadMax must be finite and above 0";
    }
    if (l.minSamples < kWallMinSamplesFloor
        || l.minSamples > static_cast<int>(localization::kMaxWallSamples)) {
        return "minSamples must be 3..64";
    }
    if (!(l.window.value() > 0.0) || !(l.windowMax.value() >= l.window.value())
        || !(l.windowMax.value() <= 1.0)) {
        return "window/windowMax must satisfy 0 < window <= windowMax <= 1 s";
    }
    if (!(l.window.value() <= localization::kMaxWallWindowSeconds)) {
        return "window must be at most 0.6 s (the 64-sample buffer spans 0.63 s at 10 ms)";
    }
    if (!(l.minConfidence >= 0.0 && l.minConfidence <= 1.0)) {
        return "minConfidence must be 0..1";
    }
    if (!(l.stillSpeed.value() > 0.0) || !finiteNumber(l.stillSpeed.value())
        || !(l.stillTurn.value() > 0.0) || !finiteNumber(l.stillTurn.value())) {
        return "stillSpeed/stillTurn must be finite and above 0";
    }
    if (!(l.errorBoundMax.value() > 0.0) || !finiteNumber(l.errorBoundMax.value())) {
        return "errorBoundMax must be finite and above 0";
    }
    if (!(l.keepOutMargin.value() >= 0.0) || !finiteNumber(l.keepOutMargin.value())) {
        return "keepOutMargin must be finite and >= 0";
    }
    if (!(l.alongDriftRate >= 0.0 && l.alongDriftRate <= 0.5)) {
        return "alongDriftRate must be 0..0.5";
    }
    if (!(l.pieceTop.value() >= 0.0) || !finiteNumber(l.pieceTop.value())
        || !(l.beamMargin.value() >= 0.0) || !finiteNumber(l.beamMargin.value())) {
        return "pieceTop/beamMargin must be finite and >= 0";
    }
    if (!(l.headingFloor.value() >= 0.5 * deg && l.headingFloor.value() <= 10.0 * deg)) {
        return "headingFloor must be 0.5..10 degrees";
    }
    if (!(l.headingPerTurn >= 0.0 && l.headingPerTurn <= 0.1)) {
        return "headingPerTurn must be 0..0.1";
    }
    return nullptr;
}

}  // namespace shulib::config
