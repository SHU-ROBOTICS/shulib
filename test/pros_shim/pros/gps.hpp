#pragma once
//
// HOST SHIM for <pros/gps.hpp> — a programmable pros::Gps that models the
// HA-06 double-subtraction hazard: the shim carries a firmware offset a test
// can PRELOAD (simulating a device another program configured), get_offset()
// reports it, and the forbidden configuration paths (offset-taking ctors,
// set_offset(), initialize_full()) are COUNTED so a test can pin that the
// adapter never touches them.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * gps_status_s_t carries x/y in METERS (vendored gps.h:53-64) and yaw as
//    the CW-from-North heading in DEGREES (HA-106 — the struct documents the
//    field name, not its convention; this is the weaker belief)
//  * get_error() returns RMS position error in METERS (gps.hpp:347-362; HA-07)
//  * the port-only ctor leaves the firmware offset alone; unset = (0,0)
//    (HA-06)
//  * failed/off-strip/calibrating reads return PROS_ERR_F in every struct
//    member (gps.hpp:374-376; HA-08)
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

struct gps_position_s_t {
    double x;
    double y;
};

struct gps_status_s_t {
    double x;
    double y;
    double pitch;
    double roll;
    double yaw;
};

namespace shim {
struct GpsPortState {
    double offsetX = 0.0;  ///< firmware offset (meters) — preload to model a configured device
    double offsetY = 0.0;
    double xMeters = 0.0;  ///< current reading (meters, VEX frame)
    double yMeters = 0.0;
    double yawDegCwFromNorth = 0.0;
    double errorMeters = 0.01;
    bool noFix = false;  ///< true → every read returns PROS_ERR_F (off-strip / calibrating)
    int forbiddenConfigCalls = 0;  ///< offset ctors + set_offset + initialize_full (HA-06)
};
inline std::array<GpsPortState, 22>& gpsPorts() {
    static std::array<GpsPortState, 22> ports{};
    return ports;
}
inline GpsPortState& gpsState(int port) {
    return gpsPorts()[static_cast<std::size_t>(std::abs(port))];
}
inline void resetGps() { gpsPorts() = {}; }
}  // namespace shim

inline namespace v5 {

class Gps {
public:
    /// The PORT-ONLY ctor — the ONLY one the binding contract sanctions (HA-06).
    explicit Gps(const std::uint8_t port) : port_{static_cast<std::int8_t>(port)} {}

    /// Forbidden ctors (gps_conversion.hpp:28-34) — counted, and they really do
    /// set the firmware offset, so an adapter that uses one fails its
    /// boot-check test on the double-subtraction it would cause.
    explicit Gps(const std::uint8_t port, double xInitial, double yInitial, double headingInitial)
        : port_{static_cast<std::int8_t>(port)} {
        auto& s = shim::gpsState(port_);
        s.forbiddenConfigCalls += 1;
        s.xMeters = xInitial;
        s.yMeters = yInitial;
        s.yawDegCwFromNorth = headingInitial;
    }
    explicit Gps(const std::uint8_t port, double xOffset, double yOffset)
        : port_{static_cast<std::int8_t>(port)} {
        auto& s = shim::gpsState(port_);
        s.forbiddenConfigCalls += 1;
        s.offsetX = xOffset;
        s.offsetY = yOffset;
    }

    std::int32_t set_offset(double xOffset, double yOffset) const {
        auto& s = shim::gpsState(port_);
        s.forbiddenConfigCalls += 1;
        s.offsetX = xOffset;
        s.offsetY = yOffset;
        return 1;
    }

    std::int32_t initialize_full(double, double, double, double xOffset, double yOffset) const {
        auto& s = shim::gpsState(port_);
        s.forbiddenConfigCalls += 1;
        s.offsetX = xOffset;
        s.offsetY = yOffset;
        return 1;
    }

    gps_position_s_t get_offset() const {
        const auto& s = shim::gpsState(port_);
        if (s.noFix) {
            // A device that cannot be read cannot report its offset either.
            return gps_position_s_t{PROS_ERR_F, PROS_ERR_F};
        }
        return gps_position_s_t{s.offsetX, s.offsetY};
    }

    double get_error() const {
        const auto& s = shim::gpsState(port_);
        return s.noFix ? static_cast<double>(PROS_ERR_F) : s.errorMeters;
    }

    gps_status_s_t get_position_and_orientation() const {
        const auto& s = shim::gpsState(port_);
        if (s.noFix) {
            return gps_status_s_t{PROS_ERR_F, PROS_ERR_F, PROS_ERR_F, PROS_ERR_F, PROS_ERR_F};
        }
        return gps_status_s_t{s.xMeters, s.yMeters, 0.0, 0.0, s.yawDegCwFromNorth};
    }

private:
    std::int8_t port_;
};

}  // namespace v5
}  // namespace pros
