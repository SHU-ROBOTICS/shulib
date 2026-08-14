#pragma once
//
// Optical-sensor canonical conversions — the ONE place the V5 optical sensor's
// raw channels become shulib's canonical ranges (§7: "convert exactly once, at
// the edge"). Pure, PROS-free, host- and mutation-testable in isolation; the
// hal/pros IOptical adapter is thin glue that CALLS these.
//
// V5 optical sensor convention (vendored pros/optical.hpp:76-170):
//  * get_hue() is a double with "a range of 0 to 359.999" (optical.hpp:79-80;
//    HA-116) — already the seam's degree unit, so the conversion is a clamp
//    that makes the [0, 360) contract unconditional rather than
//    true-if-the-doc-is-right.
//  * get_saturation() / get_brightness() are doubles with "a range of 0 to
//    1.0" (optical.hpp:103-104,127-128; HA-116) — identity plus the same
//    defensive clamp.
//  * get_proximity() is int32 with "a range of 0 to 255" (optical.hpp:151-152;
//    HA-117) — and the vendored doc documents ONLY the range: the belief that
//    LARGER means CLOSER is community knowledge, not vendored text. HA-117
//    flags it weak, and the bench measures it before any capture threshold is
//    written against proximity().
//  * Sentinels (HA-118): the double channels return PROS_ERR_F (INFINITY) on
//    failure, proximity returns PROS_ERR — the adapter screens BEFORE calling
//    these (T7); the finiteness preconditions here are the backstop.
//
// HA register: HA-116, HA-117, HA-118 (docs/hardware-assumptions.md).

#include <algorithm>
#include <cmath>

#include "shulib/core/check.hpp"

namespace shulib::hal {

/// Raw hue (get_hue(), 0–359.999, HA-116) → canonical degrees [0, 360).
/// Clamped to the doc's own stated range so the seam contract holds even
/// against an out-of-range raw value (a raw 360.0 must not leak through the
/// half-open interval). A COLOR hue, never a heading — the seam's own rule
/// (optical.hpp header).
[[nodiscard]] inline double opticalHueToCanonical(double raw) {
    SHULIB_PRECONDITION(std::isfinite(raw),
                        "opticalHueToCanonical: hue must be finite (screen PROS_ERR_F first)");
    return std::clamp(raw, 0.0, 359.999);
}

/// Raw saturation or brightness (0–1.0, HA-116) → canonical [0, 1]. Identity
/// plus the defensive clamp — one function for both channels because the
/// vendored doc gives them the same range and meaning shape.
[[nodiscard]] inline double opticalUnitIntervalToCanonical(double raw) {
    SHULIB_PRECONDITION(std::isfinite(raw),
                        "opticalUnitIntervalToCanonical: value must be finite (screen PROS_ERR_F first)");
    return std::clamp(raw, 0.0, 1.0);
}

/// Raw proximity (get_proximity(), 0–255, HA-117) → canonical [0, 1] with ≈1
/// meaning close (the seam's contract — resting on HA-117's UNMEASURED
/// larger-is-closer polarity). Drop the ÷255 and "proximity > 0.8" is true
/// the moment anything reflects at all — capture-confirm fires on an empty
/// intake.
[[nodiscard]] inline double opticalProximityToCanonical(double raw) {
    SHULIB_PRECONDITION(std::isfinite(raw),
                        "opticalProximityToCanonical: proximity must be finite (screen PROS_ERR first)");
    return std::clamp(raw / 255.0, 0.0, 1.0);
}

}  // namespace shulib::hal
