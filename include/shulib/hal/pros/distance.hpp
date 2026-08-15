#pragma once
//
// ProsDistance — IDistance over pros::Distance (chunk R1b): the
// capture/dock-confirm rangefinder behind the HAL.
//
// BINDS:
//  * get_distance() [int32 mm; HA-113] → distance()  (distanceMmToCanonical)
//  * get_confidence() [int32 0–63; HA-115] → confidence()
//    (distanceConfidenceToCanonical)
//
// THE 9999 RULE (T4 — the most important lines in chunk R1b): PROS reports
// "can not detect an object" as a PLAIN IN-BAND 9999, not PROS_ERR (vendored
// distance.hpp:71,98; HA-114). 9999 mm converts to 393.66 inches — a
// perfectly plausible-looking wall 33 feet away. This adapter maps raw 9999
// to confidence() == 0.0, IDistance's existing contract for "no usable
// return" (distance.hpp:28), which callers already threshold on. It is a
// READING, not a fault: an empty intake is a normal state, and raising a
// fault would cry wolf every tick — so no fault is raised and faultedReads()
// does NOT count it. distance() stays finite (F4 finiteness): it reports the
// honest conversion of 9999 — far, and failing every proximity threshold —
// never a held stale object (a stale "object present" is the dangerous
// direction for capture-confirm).
//
// THE CLOSE-RANGE RULE: get_confidence() is "only available when distance is
// > 200mm" (vendored distance.hpp:133-135; HA-115) — what it returns at or
// below 200 mm is UNDOCUMENTED. An object at 100 mm is exactly when a
// mechanism cares most, and passing the undefined raw value through could
// read "object touching the sensor, zero confidence". So: a valid reading at
// or below 200 mm reports confidence 1.0 — the returned distance IS the
// detection; the raw channel is not consulted where PROS says it does not
// exist.
//
// SENTINELS (T7): PROS_ERR (device failure — unplugged, wrong port) is a
// DIFFERENT state from 9999 and is screened separately: distance() holds the
// last good FINITE value, confidence() reports 0.0, faultedReads() counts it.
// The hold's INITIAL value is the far no-object distance, not 0.0 — a sensor
// dead from boot must not read "object touching the sensor". Raising a fault
// stays with the loop layer (hal/ is below diag/, the R1a T7 refinement).
//
// DELIBERATELY NOT BOUND: get_object_size() (0–400, -1 in-band for
// "undeterminable") and get_object_velocity() (m/s) — IDistance's seam does
// not carry them, no consumer asks for them, and each would import another
// in-band sentinel to rule on. If a consumer appears, they enter through a
// seam amendment, not through this adapter quietly growing.
//
// HA register: HA-113, HA-114, HA-115 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/distance.hpp"
#include "pros/error.h"
#pragma GCC diagnostic pop

#include <cstdint>

#include "shulib/hal/distance.hpp"
#include "shulib/hal/distance_conversion.hpp"
#include "shulib/units/quantity.hpp"

namespace shulib::hal::pros {

/// IDistance over pros::Distance: millimetres become canonical inches and the raw 0–63
/// confidence becomes [0, 1], each converted exactly once. The value of this adapter is
/// the three screens it puts in front of PROS's sentinels. (1) A raw 9999 — PROS's plain
/// IN-BAND "cannot detect an object", which converts to a perfectly plausible 393.66 in —
/// reports confidence 0.0 and raises nothing: an empty intake is a normal state, and a
/// fault every tick would cry wolf. (2) At or below 200 mm, where get_confidence() is
/// documented as unavailable and its value undefined, confidence is 1.0, because there the
/// returned distance IS the detection. (3) PROS_ERR is a DEVICE failure — unplugged, wrong
/// port — and is different: distance() holds the last good finite value (initially the far
/// no-object distance, never 0.0, so a sensor dead from boot cannot read "object touching
/// the sensor"), confidence() is 0.0, and faultedReads() counts it. Raising a fault is the
/// loop layer's job, not this one's. Reads are LIVE: each accessor hits the device, so
/// distance() and confidence() are two samples, not one atomic snapshot.
class ProsDistance final : public IDistance {
public:
    /// `port`: 1..21.
    explicit ProsDistance(std::uint8_t port) : sensor_{port} {}

    /// Canonical inches. 9999 (no object) converts honestly — far and finite —
    /// with confidence() reporting 0.0 (the T4 rule); PROS_ERR holds the last
    /// good value (T7).
    [[nodiscard]] units::Length distance() const override {
        const std::int32_t mm = sensor_.get_distance();
        if (mm == PROS_ERR) {
            faultedReads_ += 1;
            return lastDistance_;
        }
        lastDistance_ = distanceMmToCanonical(static_cast<double>(mm));
        return lastDistance_;
    }

    /// [0, 1]; 0.0 = no usable return (no object, or device failure). Full
    /// 1.0 at or below 200 mm, where a returned distance IS the detection
    /// (the close-range rule).
    [[nodiscard]] double confidence() const override {
        const std::int32_t mm = sensor_.get_distance();
        if (mm == PROS_ERR) {
            faultedReads_ += 1;
            return 0.0;
        }
        if (mm == kDistanceNoObjectMm) {
            return 0.0;  // THE 9999 RULE (T4) — a normal state, not a fault
        }
        if (mm <= kDistanceConfidenceAvailableAboveMm) {
            return 1.0;  // the close-range rule — raw confidence undefined here
        }
        const std::int32_t raw = sensor_.get_confidence();
        if (raw == PROS_ERR) {
            faultedReads_ += 1;
            return 0.0;
        }
        return distanceConfidenceToCanonical(static_cast<double>(raw));
    }

    /// How many DEVICE-FAILURE reads were screened (T7 observability). The
    /// 9999 no-object state is NOT counted — it is a reading, not a fault.
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    mutable ::pros::v5::Distance sensor_;  // PROS's readers are non-const
    // `mutable`: readers are const but the T7 hold-last-good screen remembers
    // state. Initial hold = the far no-object value, NEVER 0.0 (header note).
    mutable units::Length lastDistance_{9999.0 / 25.4};
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
