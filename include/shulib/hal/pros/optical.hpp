#pragma once
//
// ProsOptical — IOptical over pros::Optical (chunk R1b): the game-object
// color/proximity confirmation sensor behind the HAL.
//
// BINDS:
//  * get_hue()        [double 0–359.999; HA-116] → hue()
//    (opticalHueToCanonical — clamp only; a COLOR, never a heading)
//  * get_saturation() [double 0–1.0; HA-116] → saturation()
//  * get_brightness() [double 0–1.0; HA-116] → brightness()
//    (both via opticalUnitIntervalToCanonical)
//  * get_proximity()  [int32 0–255; HA-117] → proximity()
//    (opticalProximityToCanonical — ÷255; the LARGER-IS-CLOSER polarity is a
//    belief the vendored doc does NOT state; HA-117 flags it and the bench
//    measures it before any capture threshold trusts proximity())
//
// SENTINELS (T7): IOptical has NO validity channel — unlike IDistance there
// is no confidence() to absorb a failure. The double channels return
// PROS_ERR_F (INFINITY) and proximity returns PROS_ERR on device failure
// (HA-118): each reader screens its own sentinel, holds its last good value,
// and counts it in faultedReads(). Never propagate (INFINITY through hue()
// breaks the F4 finiteness contract), never zero (hue 0.0 IS a color — red —
// so a zeroed failure would read as a confident wrong answer to "what color
// is this game piece"). Raising a fault stays with the loop layer (hal/ is
// below diag/).
//
// DELIBERATELY NOT here: set_led_pwm()/get_led_pwm(), gestures, raw RGBC —
// the seam carries color-confirm channels only; lighting policy is a
// mechanism decision (which owns knowing whether ITS sensor needs the LED),
// and gestures have no consumer anywhere in the plan.
//
// HA register: HA-116, HA-117, HA-118 (docs/hardware-assumptions.md).

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/error.h"
#include "pros/optical.hpp"
#pragma GCC diagnostic pop

#include <cmath>
#include <cstdint>

#include "shulib/hal/optical.hpp"
#include "shulib/hal/optical_conversion.hpp"

namespace shulib::hal::pros {

/// IOptical over pros::Optical — the game-object colour/proximity confirm sensor behind the HAL.
/// Every reader SCREENS the device's failure sentinel and returns the last good value instead of
/// the failure: never INFINITY, which would break the F4 finiteness contract downstream, and never
/// a 0 written OVER a good reading, because hue 0.0 IS a colour (red) — a zeroed failure would
/// answer "what colour is this game piece" confidently and wrongly. The cache itself STARTS at
/// 0.0, so a sensor that has never once read successfully (unplugged at boot) does serve
/// hue() == 0.0 on every call; faultedReads(), which counts every screen, is the only channel that
/// separates that from a sensor staring at a steady red object, because this seam has no
/// confidence() to say so (T7).
/// The readers are const but update that held-value cache, so const here does not mean safe to
/// poll from two tasks at once. Raising a fault is the loop layer's job, not this seam's.
class ProsOptical final : public IOptical {
public:
    /// `port`: 1..21.
    explicit ProsOptical(std::uint8_t port) : sensor_{port} {}

    /// Color hue [0, 360) — sentinel-screened to last good (T7).
    [[nodiscard]] double hue() const override {
        const double raw = sensor_.get_hue();
        if (!std::isfinite(raw)) {  // PROS_ERR_F (HA-118)
            faultedReads_ += 1;
            return lastHue_;
        }
        lastHue_ = opticalHueToCanonical(raw);
        return lastHue_;
    }

    /// Colour purity in [0, 1]: how strongly coloured the reading is, independent of how bright.
    /// A low saturation means hue() is describing something near grey and is not a colour call
    /// worth acting on. Sentinel-screened to last good (T7); 0.0 until the first successful read.
    [[nodiscard]] double saturation() const override {
        const double raw = sensor_.get_saturation();
        if (!std::isfinite(raw)) {
            faultedReads_ += 1;
            return lastSaturation_;
        }
        lastSaturation_ = opticalUnitIntervalToCanonical(raw);
        return lastSaturation_;
    }

    /// The light level the sensor sees, in [0, 1] — a brightness, not a distance; proximity() is
    /// the distance channel. Sentinel-screened to last good (T7); 0.0 until the first good read.
    [[nodiscard]] double brightness() const override {
        const double raw = sensor_.get_brightness();
        if (!std::isfinite(raw)) {
            faultedReads_ += 1;
            return lastBrightness_;
        }
        lastBrightness_ = opticalUnitIntervalToCanonical(raw);
        return lastBrightness_;
    }

    /// [0, 1], ≈1 = close (HA-117's unmeasured polarity — header note).
    [[nodiscard]] double proximity() const override {
        const std::int32_t raw = sensor_.get_proximity();
        if (raw == PROS_ERR) {  // int32 channel: PROS_ERR, not PROS_ERR_F
            faultedReads_ += 1;
            return lastProximity_;
        }
        lastProximity_ = opticalProximityToCanonical(static_cast<double>(raw));
        return lastProximity_;
    }

    /// How many reads were screened to last-good (T7 observability).
    [[nodiscard]] int faultedReads() const noexcept { return faultedReads_; }

private:
    mutable ::pros::v5::Optical sensor_;  // PROS's readers are non-const
    // `mutable`: readers are const but the T7 hold-last-good screen must
    // remember state — the caches are part of the read, not commanded state.
    mutable double lastHue_ = 0.0;
    mutable double lastSaturation_ = 0.0;
    mutable double lastBrightness_ = 0.0;
    mutable double lastProximity_ = 0.0;
    mutable int faultedReads_ = 0;
};

}  // namespace shulib::hal::pros
