#pragma once
//
// Axis — the FIELD axis a wall measures (chunk L6-A1, the wall reset, 2026-10-29).
//
// A LEAF header on purpose. Two headers need this one enum: the Localizer (its one-axis seat,
// `Localizer::seatAxis`, and its per-axis travel counter) and the wall reset's pure gates
// (`localization/wall_reset.hpp`). The wall reset's EXPLICIT call needs the Chassis, and the Chassis
// reaches the Localizer through motion/motion.hpp — so if `Axis` lived beside the call, the Localizer
// would have to include upward through the facade and the includes would close a cycle
// (chassis.hpp -> motion/command_pipeline.hpp -> motion/motion.hpp -> localizer.hpp -> ... ->
// chassis.hpp). The master plan's §5 layering rule forbids that include anyway: no layer includes a
// header from a layer above it. So the enum lives here, alone, below both.
//
// PROS-free, allocation-free, no includes beyond <cstdint>.

#include <cstdint>

namespace shulib::localization {

/// A field axis (the F1 frame: +X to the right as seen from the red alliance station, +Y away from
/// it). A field wall is a line `axis = constant`, so a range measured square to it says where the
/// robot is along THAT axis and nothing about the other one — which is why the wall reset re-seats
/// one axis at a time (`Localizer::seatAxis`).
enum class Axis : std::uint8_t {
    /// The x axis — what the LEFT and RIGHT walls (x = -70.2 / +70.2 in) measure.
    X,
    /// The y axis — what the RED and BLUE walls (y = -70.2 / +70.2 in) measure.
    Y
};

/// The axis's letter for a log line: "x" or "y". Exhaustive, no default — a new axis without a
/// letter does not compile under the project's -Wswitch -Werror.
[[nodiscard]] constexpr const char* axisLetter(Axis axis) noexcept {
    switch (axis) {
        case Axis::X:
            return "x";
        case Axis::Y:
            return "y";
    }
    return "?";
}

/// The OTHER axis: the one a wall on `axis` lies ALONG (the left wall measures x and runs along y).
/// The wall reset's keep-outs are tested along it, which is why it widens them by how far the
/// estimate may have drifted on it (`Localizer::travelSinceSeat`).
[[nodiscard]] constexpr Axis alongAxis(Axis axis) noexcept {
    return axis == Axis::X ? Axis::Y : Axis::X;
}

}  // namespace shulib::localization
