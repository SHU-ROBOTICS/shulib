#pragma once
//
// Controller canonical conversions — the ONE place the V5 controller's raw
// stick range becomes shulib's canonical [-1, 1] (§7: "convert exactly once,
// at the edge"). Pure, PROS-free, host- and mutation-testable in isolation;
// the hal/pros IController adapter is thin glue that CALLS this.
//
// V5 controller convention (vendored pros/misc.hpp:85: "The current reading of
// the analog channel: [-127, 127]. If the controller was not connected, then 0
// is returned" — HA-103). shulib canonical: [-1, 1], full deflection = ±1.
//
// Note the asymmetry this hides: −127…+127 is symmetric, so ÷127 maps both
// rails exactly to ±1. A hypothetical −128 (int8 min, never documented to
// occur) would map to −1.008 — the clamp below makes the canonical contract
// ("axis() is in [-1, 1]") unconditionally true rather than true-if-the-doc-
// is-right. Clamping is NOT deadband: deadband/curves/slew are driver-feel
// POLICY and belong to the teleop layer (chunk T2), never to a conversion.

#include <algorithm>
#include <cmath>

#include "shulib/core/check.hpp"

namespace shulib::hal {

/// Raw analog reading (get_analog(), [-127, 127], HA-103) → canonical [-1, 1].
/// ÷127 so full deflection is exactly ±1; clamped so the contract holds even
/// against an out-of-range raw value. Drop the ÷127 and every stick input
/// saturates the speed budget at the slightest touch — full-speed lurch on a
/// 1-count wiggle.
[[nodiscard]] inline double controllerAxisToCanonical(double raw) {
    SHULIB_PRECONDITION(std::isfinite(raw),
                        "controllerAxisToCanonical: raw axis must be finite");
    return std::clamp(raw / 127.0, -1.0, 1.0);
}

}  // namespace shulib::hal
