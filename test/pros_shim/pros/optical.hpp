#pragma once
//
// HOST SHIM for <pros/optical.hpp> — a programmable pros::Optical.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * get_hue() is a double, "a range of 0 to 359.999" (vendored
//    optical.hpp:79-80; HA-116)
//  * get_saturation()/get_brightness() are doubles, "a range of 0 to 1.0"
//    (optical.hpp:103-104,127-128; HA-116)
//  * get_proximity() is int32, "a range of 0 to 255" (optical.hpp:151-152;
//    HA-117) — the vendored doc states ONLY the range; larger-means-closer is
//    an UNMEASURED belief the shim inherits (it cannot do otherwise: a shared
//    model cannot test its own polarity — the bench does, HA-117)
//  * failed double reads return PROS_ERR_F, failed proximity reads return
//    PROS_ERR (optical.hpp:87-88,111-112,135-136,159-160; HA-118)
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
struct OpticalPortState {
    double hue = 0.0;            ///< raw 0–359.999
    double saturation = 0.0;     ///< raw 0–1.0
    double brightness = 0.0;     ///< raw 0–1.0
    std::int32_t proximity = 0;  ///< raw 0–255 (default: nothing near)
    bool disconnected = false;   ///< true → PROS_ERR_F / PROS_ERR per channel
};
inline std::array<OpticalPortState, 22>& opticalPorts() {
    static std::array<OpticalPortState, 22> ports{};
    return ports;
}
inline OpticalPortState& opticalState(int port) {
    return opticalPorts()[static_cast<std::size_t>(port)];
}
inline void resetOpticals() { opticalPorts() = {}; }
}  // namespace shim

inline namespace v5 {

class Optical {
public:
    explicit Optical(const std::uint8_t port) : port_{port} {}

    double get_hue() {
        const auto& s = shim::opticalState(port_);
        return s.disconnected ? static_cast<double>(PROS_ERR_F) : s.hue;
    }

    double get_saturation() {
        const auto& s = shim::opticalState(port_);
        return s.disconnected ? static_cast<double>(PROS_ERR_F) : s.saturation;
    }

    double get_brightness() {
        const auto& s = shim::opticalState(port_);
        return s.disconnected ? static_cast<double>(PROS_ERR_F) : s.brightness;
    }

    std::int32_t get_proximity() {
        const auto& s = shim::opticalState(port_);
        return s.disconnected ? PROS_ERR : s.proximity;
    }

private:
    std::uint8_t port_;
};

}  // namespace v5
}  // namespace pros
