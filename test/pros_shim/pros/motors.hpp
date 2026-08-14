#pragma once
//
// HOST SHIM for <pros/motors.hpp> — a programmable pros::Motor with PER-PORT
// DEVICE STATE, so the trap-A semantics are real in tests: the smart motor's
// encoder units and gearing live in the DEVICE, survive across Motor object
// constructions, and default to whatever "a previous program" left — this shim
// deliberately leaves them at rotations/red (NOT the degrees/green shulib
// wants), so an adapter that fails to set-and-read-back reads positions 1/360
// off and tests go red instead of green.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * ctor defaults MotorGears::invalid / MotorUnits::invalid = "leave the
//    device as it is" (vendored motors.hpp:74-75; HA-98)
//  * move_voltage() takes MILLIVOLTS in [-12000, 12000] (motors.hpp:234; HA-94)
//  * get_position() returns "the absolute position in ITS encoder units"
//    (motors.hpp:628; HA-95) — degrees only if degrees were set
//  * get_actual_velocity() returns RPM (motors.hpp:404; HA-96)
//  * get_current_draw() returns mA (motors.hpp:426; HA-97)
//  * get_temperature() returns °C (motors.hpp:745)
//  * a NEGATIVE port reverses the motor: commands and readings negate, applied
//    by PROS itself, exactly once
//  * failed reads return PROS_ERR / PROS_ERR_F (error.h)
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
inline namespace v5 {

enum class MotorBrake {
    coast = 0,
    brake = 1,
    hold = 2,
    invalid = INT32_MAX,
};

enum class MotorEncoderUnits {
    degrees = 0,
    deg = 0,
    rotations = 1,
    counts = 2,
    invalid = INT32_MAX,
};
using MotorUnits = MotorEncoderUnits;

enum class MotorGears {
    ratio_36_to_1 = 0,
    red = ratio_36_to_1,
    rpm_100 = ratio_36_to_1,
    ratio_18_to_1 = 1,
    green = ratio_18_to_1,
    rpm_200 = ratio_18_to_1,
    ratio_6_to_1 = 2,
    blue = ratio_6_to_1,
    rpm_600 = ratio_6_to_1,
    invalid = INT32_MAX,
};

}  // namespace v5

namespace shim {

/// The DEVICE-side state of one motor port. Adversarial defaults on purpose
/// (rotations/red): the state a hostile "different program on a different day"
/// could have left, which is exactly what pros::Motor's leave-as-is ctor
/// defaults expose an adapter to.
struct MotorPortState {
    v5::MotorUnits units = v5::MotorUnits::rotations;
    v5::MotorGears gearing = v5::MotorGears::red;
    v5::MotorBrake brake = v5::MotorBrake::coast;
    double positionOutputDeg = 0.0;  ///< truth, output-shaft degrees (test-set)
    double velocityRpm = 0.0;
    std::int32_t currentMa = 0;
    double temperatureC = 20.0;
    std::int32_t lastVoltageMv = 0;
    int moveVoltageCalls = 0;
    int setEncoderUnitsCalls = 0;
    int setGearingCalls = 0;
    int setBrakeModeCalls = 0;
    bool disconnected = false;  ///< true → every read returns the PROS sentinel
};

inline std::array<MotorPortState, 22>& motorPorts() {
    static std::array<MotorPortState, 22> ports{};
    return ports;
}
inline MotorPortState& motorState(int port) {
    return motorPorts()[static_cast<std::size_t>(std::abs(port))];
}
inline void resetMotors() { motorPorts() = {}; }

}  // namespace shim

inline namespace v5 {

class Motor {
public:
    explicit Motor(const std::int8_t port, const MotorGears gearset = MotorGears::invalid,
                   const MotorUnits encoder_units = MotorUnits::invalid)
        : port_{port} {
        auto& s = shim::motorState(port_);
        // "invalid" = LEAVE THE DEVICE AS IT IS (vendored motors.hpp:74-75).
        if (gearset != MotorGears::invalid) {
            s.gearing = gearset;
            s.setGearingCalls += 1;
        }
        if (encoder_units != MotorUnits::invalid) {
            s.units = encoder_units;
            s.setEncoderUnitsCalls += 1;
        }
    }

    std::int32_t move_voltage(const std::int32_t voltage) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.moveVoltageCalls += 1;
        s.lastVoltageMv = static_cast<std::int32_t>(sign()) * voltage;
        return 1;
    }

    double get_position(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR_F;
        }
        const double outDeg = sign() * s.positionOutputDeg;
        switch (s.units) {
            case MotorUnits::degrees:
                return outDeg;
            case MotorUnits::rotations:
                return outDeg / 360.0;
            case MotorUnits::counts:
                // green cartridge: 900 ticks/rev at the output (HA-15); the
                // exact tick count is not load-bearing here — what matters is
                // that "not degrees" reads WRONG for a degrees-assuming adapter.
                return outDeg * (900.0 / 360.0);
            case MotorUnits::invalid:
                break;
        }
        return PROS_ERR_F;
    }

    double get_actual_velocity(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? static_cast<double>(PROS_ERR_F) : sign() * s.velocityRpm;
    }

    std::int32_t get_current_draw(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? PROS_ERR : s.currentMa;
    }

    double get_temperature(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? static_cast<double>(PROS_ERR_F) : s.temperatureC;
    }

    std::int32_t set_brake_mode(const MotorBrake mode, const std::uint8_t /*index*/ = 0) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.brake = mode;
        s.setBrakeModeCalls += 1;
        return 1;
    }

    MotorBrake get_brake_mode(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? MotorBrake::invalid : s.brake;
    }

    std::int32_t set_encoder_units(const MotorUnits units, const std::uint8_t /*index*/ = 0) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.units = units;
        s.setEncoderUnitsCalls += 1;
        return 1;
    }

    MotorUnits get_encoder_units(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? MotorUnits::invalid : s.units;
    }

    std::int32_t set_gearing(const MotorGears gearset, const std::uint8_t /*index*/ = 0) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.gearing = gearset;
        s.setGearingCalls += 1;
        return 1;
    }

    MotorGears get_gearing(const std::uint8_t /*index*/ = 0) const {
        const auto& s = shim::motorState(port_);
        return s.disconnected ? MotorGears::invalid : s.gearing;
    }

private:
    [[nodiscard]] double sign() const { return port_ < 0 ? -1.0 : 1.0; }
    std::int8_t port_;
};

}  // namespace v5
}  // namespace pros
