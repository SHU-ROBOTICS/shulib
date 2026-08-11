#pragma once
//
// IOptical — a color / optical sensor (pros::Optical) behind the HAL. Used for
// game-object color confirmation (master plan §M4: orientToScoringHalf, the Toggle
// color confirm, intake capture).
//
// hue() is a COLOR hue in degrees [0, 360) — cyclic, but NOT a heading, so it is a
// plain double, never a math::Angle (mixing a color with a spatial angle would be a
// bug). saturation()/brightness()/proximity() are normalized to [0, 1] by the
// hal/pros adapter (proximity ≈ 1 means an object is close).

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IOptical {
public:
    virtual ~IOptical() = default;
    IOptical() = default;
    IOptical(const IOptical&) = default;
    IOptical(IOptical&&) = default;
    IOptical& operator=(const IOptical&) = default;
    IOptical& operator=(IOptical&&) = default;

    /// Detected color hue in degrees, [0, 360). A color, not a heading.
    [[nodiscard]] virtual double hue() const = 0;

    /// Color saturation in [0, 1].
    [[nodiscard]] virtual double saturation() const = 0;

    /// Brightness in [0, 1].
    [[nodiscard]] virtual double brightness() const = 0;

    /// Proximity in [0, 1] (≈1 = object close to the sensor).
    [[nodiscard]] virtual double proximity() const = 0;
};

}  // namespace shulib::hal
