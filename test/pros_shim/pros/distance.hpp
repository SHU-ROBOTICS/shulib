#pragma once
//
// HOST SHIM for <pros/distance.hpp> — a programmable pros::Distance.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * get_distance() is int32 MILLIMETERS (vendored distance.hpp:63,89;
//    HA-113)
//  * "Will return 9999 if the sensor can not detect an object" (vendored
//    distance.hpp:71,98) — an IN-BAND plain integer, NOT PROS_ERR (HA-114).
//    ADVERSARIAL DEFAULT: a fresh shim sensor reports exactly that 9999 with
//    a HIGH raw confidence, because that is what a real sensor over an empty
//    intake hands an adapter — an adapter that passes it through reads a
//    plausible 393.66-inch object and fails its test.
//  * get_confidence() is int32 "a range of 0 to 63. 63 means high
//    confidence", "only available when distance is > 200mm" (vendored
//    distance.hpp:133-135; HA-115). What it returns at or below 200 mm is
//    UNDOCUMENTED — the shim models that honestly with a poisoned
//    below-200mm value the test can set to garbage.
//  * failed reads return PROS_ERR (vendored distance.hpp:70,97,142; error.h)
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstdint>

#include "pros/error.h"

namespace pros {

namespace shim {
struct DistancePortState {
    /// Raw mm. ADVERSARIAL DEFAULT: 9999 — "can not detect an object", the
    /// in-band trap made real (HA-114). A test must PLACE an object.
    std::int32_t distanceMm = 9999;
    /// Raw 0–63. ADVERSARIAL DEFAULT: 63 (high) — paired with the 9999
    /// default so an adapter that trusts confidence without checking the
    /// distance reads "393 inches, fully confident".
    std::int32_t confidenceRaw = 63;
    /// What get_confidence() hands back when distanceMm <= 200 — the range
    /// where the vendored doc says confidence is NOT available (HA-115).
    /// Defaults to 0 so an adapter that passes raw confidence through at
    /// close range reads "object touching the sensor, zero confidence" and
    /// fails its test.
    std::int32_t confidenceBelow200Raw = 0;
    bool disconnected = false;  ///< true → every read returns PROS_ERR
};
inline std::array<DistancePortState, 22>& distancePorts() {
    static std::array<DistancePortState, 22> ports{};
    return ports;
}
inline DistancePortState& distanceState(int port) {
    return distancePorts()[static_cast<std::size_t>(port)];
}
inline void resetDistances() { distancePorts() = {}; }
}  // namespace shim

inline namespace v5 {

class Distance {
public:
    explicit Distance(const std::uint8_t port) : port_{port} {}

    /// mm; 9999 = no object (HA-113/114); PROS_ERR on failure.
    std::int32_t get_distance() {
        const auto& s = shim::distanceState(port_);
        return s.disconnected ? PROS_ERR : s.distanceMm;
    }

    /// Identical to get_distance() (vendored distance.hpp:90: "This function
    /// is identical to get()").
    std::int32_t get() { return get_distance(); }

    /// 0–63 above 200 mm; UNDOCUMENTED at/below 200 mm (modeled via
    /// confidenceBelow200Raw); PROS_ERR on failure (HA-115).
    std::int32_t get_confidence() {
        const auto& s = shim::distanceState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        return s.distanceMm <= 200 ? s.confidenceBelow200Raw : s.confidenceRaw;
    }

private:
    std::uint8_t port_;
};

}  // namespace v5
}  // namespace pros
