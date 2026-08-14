#pragma once
//
// HOST SHIM for <pros/rotation.hpp> — a programmable pros::Rotation.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * get_position() is int32 CUMULATIVE CENTIDEGREES (vendored
//    rotation.hpp:195-216; HA-11/HA-16)
//  * get_velocity() is int32 centidegrees/second (rotation.hpp:219-242; HA-105)
//  * a NEGATIVE ctor port reverses the sensor, applied by PROS itself exactly
//    once (rotation.hpp:46-47) — the shim negates so a double-negating adapter
//    reads backwards and fails its test
//  * failed reads return PROS_ERR (error.h)
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstdint>
#include <cstdlib>

#include "pros/error.h"

namespace pros {

namespace shim {
struct RotationPortState {
    std::int32_t positionCentideg = 0;  ///< truth, un-reversed cumulative centidegrees
    std::int32_t velocityCentidegPerSec = 0;
    bool disconnected = false;
};
inline std::array<RotationPortState, 22>& rotationPorts() {
    static std::array<RotationPortState, 22> ports{};
    return ports;
}
inline RotationPortState& rotationState(int port) {
    return rotationPorts()[static_cast<std::size_t>(std::abs(port))];
}
inline void resetRotations() { rotationPorts() = {}; }
}  // namespace shim

inline namespace v5 {

class Rotation {
public:
    explicit Rotation(const std::int8_t port) : port_{port} {}

    std::int32_t get_position() const {
        const auto& s = shim::rotationState(port_);
        return s.disconnected ? PROS_ERR : sign() * s.positionCentideg;
    }

    std::int32_t get_velocity() const {
        const auto& s = shim::rotationState(port_);
        return s.disconnected ? PROS_ERR : sign() * s.velocityCentidegPerSec;
    }

private:
    [[nodiscard]] std::int32_t sign() const { return port_ < 0 ? -1 : 1; }
    std::int8_t port_;
};

}  // namespace v5
}  // namespace pros
