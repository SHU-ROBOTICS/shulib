#pragma once
//
// HOST SHIM for three READS of <pros/motors.h>'s C API (chunk CAL1, Part A1, 2026-10-08):
// motor_get_position, motor_get_encoder_units and motor_get_gearing — the reads by port that the
// standard Calibrate's WATCH of a declared mechanism motor makes (CAL1 Part A2), with no motor
// object of any kind. They read the SAME per-port device state as this shim's pros::Motor
// (motors.hpp's pros::shim::motorState) and WRITE NOTHING: no field of that state changes, and
// every write counter there (gearing, units, voltage, brake mode, brake) stays where it was. They
// count their own calls instead (pros::shim::motorCReads()), so a test can see a read happened.
// The belief behind "a read writes nothing" on the brain is CAL1's (A2 registers it): the vendored
// libpros.a's three functions call only the firmware's *Get functions.
//
// Names and values transcribed from the vendored include/pros/motors.h: the two enums at :875-899
// (in namespace pros), the three functions at :735, :1294 and :1318 (in pros::c). motor_get_position
// returns the position "in its encoder units", and "a negative port value negates the return
// value" (:709-711); PROS_ERR_F on a port that does not answer.
//
// ONE DIFFERENCE FROM motors.hpp, on purpose: in COUNTS, this read scales by the gearing the
// device reports — 1800 counts per output turn for red (36:1), 900 for green (18:1), 300 for blue
// (6:1), the register's HA-15 — where pros::Motor::get_position() in motors.hpp uses green's 900
// whatever the gearing (its own comment: there the exact count is not load-bearing). Here it is:
// the watch turns counts into degrees by the gearing it reads, so a read that ignored the gearing
// would let a conversion bug pass. A count read with no valid gearing returns PROS_ERR_F.
//
// Reset with pros::shim::resetMotorCReads(): pros::shim::resetAll() (shim_control.hpp) resets the
// device state through resetMotors() but does not reach this file's counters, which are newer.
//
// HONEST LIMIT: this shim tests a program against OUR BELIEF about PROS; it cannot test the
// belief. Hardware tests the belief (shim_control.hpp says it in full).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdlib>

#include "pros/error.h"
#include "pros/motors.hpp"

namespace pros {

typedef enum motor_encoder_units_e {
    E_MOTOR_ENCODER_DEGREES = 0,
    E_MOTOR_ENCODER_ROTATIONS = 1,
    E_MOTOR_ENCODER_COUNTS = 2,
    E_MOTOR_ENCODER_INVALID = INT32_MAX
} motor_encoder_units_e_t;

typedef enum motor_gearset_e {
    E_MOTOR_GEARSET_36 = 0,
    E_MOTOR_GEAR_RED = E_MOTOR_GEARSET_36,
    E_MOTOR_GEAR_100 = E_MOTOR_GEARSET_36,
    E_MOTOR_GEARSET_18 = 1,
    E_MOTOR_GEAR_GREEN = E_MOTOR_GEARSET_18,
    E_MOTOR_GEAR_200 = E_MOTOR_GEARSET_18,
    E_MOTOR_GEARSET_06 = 2,
    E_MOTOR_GEAR_BLUE = E_MOTOR_GEARSET_06,
    E_MOTOR_GEAR_600 = E_MOTOR_GEARSET_06,
    E_MOTOR_GEARSET_INVALID = INT32_MAX,
} motor_gearset_e_t;

namespace shim {

/// How many times each of the three C reads was called on one port.
struct MotorCReadCounts {
    int position = 0;  ///< motor_get_position()
    int units = 0;     ///< motor_get_encoder_units()
    int gearing = 0;   ///< motor_get_gearing()
};
/// The counters, by |port| (index 0 unused, as motorPorts()).
inline std::array<MotorCReadCounts, 22>& motorCReads() {
    static std::array<MotorCReadCounts, 22> n{};
    return n;
}
inline void resetMotorCReads() { motorCReads() = {}; }

/// Encoder counts per OUTPUT-shaft turn for a gearing (HA-15: red 1800, green 900, blue 300), or 0
/// when the gearing is not one of the three.
inline double countsPerTurn(v5::MotorGears g) {
    switch (g) {
        case v5::MotorGears::red: return 1800.0;
        case v5::MotorGears::green: return 900.0;
        case v5::MotorGears::blue: return 300.0;
        case v5::MotorGears::invalid: break;
    }
    return 0.0;
}

}  // namespace shim

namespace c {

inline double motor_get_position(std::int8_t port) {
    ::pros::shim::motorCReads()[static_cast<std::size_t>(std::abs(port))].position += 1;
    const auto& s = ::pros::shim::motorState(port);
    if (s.disconnected) {
        return PROS_ERR_F;
    }
    const double outDeg = (port < 0 ? -1.0 : 1.0) * s.positionOutputDeg;
    switch (s.units) {
        case v5::MotorUnits::degrees: return outDeg;
        case v5::MotorUnits::rotations: return outDeg / 360.0;
        case v5::MotorUnits::counts: {
            const double perTurn = ::pros::shim::countsPerTurn(s.gearing);
            if (perTurn <= 0.0) {
                return PROS_ERR_F;
            }
            return outDeg * perTurn / 360.0;
        }
        case v5::MotorUnits::invalid: break;
    }
    return PROS_ERR_F;
}

inline motor_encoder_units_e_t motor_get_encoder_units(std::int8_t port) {
    ::pros::shim::motorCReads()[static_cast<std::size_t>(std::abs(port))].units += 1;
    const auto& s = ::pros::shim::motorState(port);
    if (s.disconnected) {
        return E_MOTOR_ENCODER_INVALID;
    }
    return static_cast<motor_encoder_units_e_t>(static_cast<int>(s.units));
}

inline motor_gearset_e_t motor_get_gearing(std::int8_t port) {
    ::pros::shim::motorCReads()[static_cast<std::size_t>(std::abs(port))].gearing += 1;
    const auto& s = ::pros::shim::motorState(port);
    if (s.disconnected) {
        return E_MOTOR_GEARSET_INVALID;
    }
    return static_cast<motor_gearset_e_t>(static_cast<int>(s.gearing));
}

}  // namespace c
}  // namespace pros
