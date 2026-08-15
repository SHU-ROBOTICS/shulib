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

/// A color / optical sensor behind the HAL — the seam over pros::Optical that game-object
/// color confirmation reads (the M4 scoring-half orient, the Toggle color confirm, intake
/// capture). Every reader's SCALE is fixed by this seam, not by the device: hue in degrees
/// [0, 360), the other three in [0, 1]. An implementation owns whatever vendor rescaling
/// that takes, so no caller ever divides by 255 or wonders which scale it got. Read-only —
/// the vendor's LED, gesture and integration-time controls are not part of this seam.
class IOptical {
public:
    /// The polymorphic-base special members, and the exact rule they follow. The virtual
    /// destructor is what makes deleting a derived sensor through an `IOptical*` legal, and
    /// user-declaring it costs the two MOVE operations: the move constructor and move assignment
    /// are then never implicitly declared, while the COPY pair still is (merely deprecated). So
    /// only the two `&&` lines below restore anything. Declaring constructors is in turn what
    /// suppresses the implicit default constructor, which is why `IOptical() = default;` has to
    /// be spelled out — without it no derived sensor could be default-constructed. None of this
    /// is an invitation to copy a live device: implementations are held by pointer, and copying
    /// one would duplicate a port, not a reading.
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
