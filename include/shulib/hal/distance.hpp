#pragma once
//
// IDistance — a distance / time-of-flight sensor (pros::Distance) behind the HAL.
// distance() in canonical inches; confidence() in [0, 1]. The hal/pros adapter
// converts mm→inches and normalizes the raw confidence (0–63 → 0–1) exactly once.
//
// Callers threshold confidence() for "object present" — never trust distance() when
// confidence() is ~0 (out of range / no return). Used for manipulation sensor-confirm
// and the docking distance-fallback (master plan §M3/§M4).

#include "shulib/units/quantity.hpp"

namespace shulib::hal {

class IDistance {
public:
    virtual ~IDistance() = default;
    IDistance() = default;
    IDistance(const IDistance&) = default;
    IDistance(IDistance&&) = default;
    IDistance& operator=(const IDistance&) = default;
    IDistance& operator=(IDistance&&) = default;

    /// Measured distance to the nearest object (canonical inches). Only meaningful when
    /// confidence() is above the caller's threshold.
    [[nodiscard]] virtual units::Length distance() const = 0;

    /// Reading confidence in [0, 1]; ~0 means no usable return (treat as "no object").
    [[nodiscard]] virtual double confidence() const = 0;
};

}  // namespace shulib::hal
