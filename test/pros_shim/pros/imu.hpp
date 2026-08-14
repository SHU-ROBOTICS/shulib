#pragma once
//
// HOST SHIM for <pros/imu.hpp> — a programmable pros::Imu that is ADVERSARIAL
// about the get_rotation()/get_heading() distinction: get_rotation() returns
// the cumulative CW degrees a test sets; get_heading() returns that value
// WRAPPED to [0,360). The two agree only inside the first positive revolution —
// so an adapter mis-bound to get_heading() (HA-03's forbidden binding) reads
// differently the moment a test spins past 360° or negative, and its
// continuity test goes red instead of green.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * get_rotation(): cumulative CW-positive degrees, "theoretically unbounded"
//    (vendored imu.hpp:210-214; HA-03)
//  * get_heading(): CW-positive degrees bounded [0,360) (imu.hpp:245-250)
//  * is_calibrating(): true during boot calibration (imu.hpp:1014-1027; HA-23)
//  * reset(): STARTS calibration (imu.hpp:141-145; HA-108) — counted, so a
//    test can pin "the adapter calibrates at most once and never tares"
//  * get_gyro_rate(): raw body-axis struct, z assumed yaw in deg/s with an
//    UNDOCUMENTED sign — modeled CW-positive (HA-109 units, HA-04 sign)
//  * get_pitch()/get_roll(): degrees bounded (-180,180) (imu.hpp:346-406)
//  * failed reads return PROS_ERR_F (error.h)
//  * tare/tare_rotation/set_rotation/tare_heading/set_heading are COUNTED —
//    the HA-05 contract says the adapter must never call them; a nonzero count
//    in a test is a red flag, literally
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "pros/error.h"

namespace pros {

struct imu_raw_s {
    double x;
    double y;
    double z;
};
typedef imu_raw_s imu_gyro_s_t;

namespace shim {
struct ImuPortState {
    double rotationDegCw = 0.0;  ///< cumulative CW degrees (truth)
    double gyroZDegPerSecCw = 0.0;
    double pitchDeg = 0.0;
    double rollDeg = 0.0;
    bool calibrating = false;
    bool disconnected = false;
    int resetCalls = 0;
    int tareFamilyCalls = 0;  ///< tare/tare_rotation/set_rotation/tare_heading/set_heading
};
inline std::array<ImuPortState, 22>& imuPorts() {
    static std::array<ImuPortState, 22> ports{};
    return ports;
}
inline ImuPortState& imuState(int port) {
    return imuPorts()[static_cast<std::size_t>(std::abs(port))];
}
inline void resetImus() { imuPorts() = {}; }
}  // namespace shim

inline namespace v5 {

class Imu {
public:
    explicit Imu(const std::uint8_t port) : port_{static_cast<std::int8_t>(port)} {}

    double get_rotation() const {
        const auto& s = shim::imuState(port_);
        return (s.disconnected || s.calibrating) ? static_cast<double>(PROS_ERR_F) : s.rotationDegCw;
    }

    /// Wrapped [0,360) — deliberately DIFFERENT from get_rotation() outside the
    /// first revolution (see header: the mis-binding trap).
    double get_heading() const {
        const auto& s = shim::imuState(port_);
        if (s.disconnected || s.calibrating) {
            return PROS_ERR_F;
        }
        double wrapped = std::fmod(s.rotationDegCw, 360.0);
        if (wrapped < 0.0) {
            wrapped += 360.0;
        }
        return wrapped;
    }

    bool is_calibrating() const { return shim::imuState(port_).calibrating; }

    imu_gyro_s_t get_gyro_rate() const {
        const auto& s = shim::imuState(port_);
        if (s.disconnected || s.calibrating) {
            return imu_gyro_s_t{PROS_ERR_F, PROS_ERR_F, PROS_ERR_F};
        }
        return imu_gyro_s_t{0.0, 0.0, s.gyroZDegPerSecCw};
    }

    double get_pitch() const {
        const auto& s = shim::imuState(port_);
        return (s.disconnected || s.calibrating) ? static_cast<double>(PROS_ERR_F) : s.pitchDeg;
    }

    double get_roll() const {
        const auto& s = shim::imuState(port_);
        return (s.disconnected || s.calibrating) ? static_cast<double>(PROS_ERR_F) : s.rollDeg;
    }

    std::int32_t reset(bool /*blocking*/ = false) const {
        auto& s = shim::imuState(port_);
        s.resetCalls += 1;
        s.calibrating = true;  // a test clears this to model calibration finishing
        return 1;
    }

    // The HA-05 forbidden family — counted, never modeled further: the adapter
    // must simply never call them.
    std::int32_t tare() const { return countTare(); }
    std::int32_t tare_rotation() const { return countTare(); }
    std::int32_t set_rotation(double) const { return countTare(); }
    std::int32_t tare_heading() const { return countTare(); }
    std::int32_t set_heading(double) const { return countTare(); }

private:
    std::int32_t countTare() const {
        shim::imuState(port_).tareFamilyCalls += 1;
        return 1;
    }
    std::int8_t port_;
};

}  // namespace v5
}  // namespace pros
