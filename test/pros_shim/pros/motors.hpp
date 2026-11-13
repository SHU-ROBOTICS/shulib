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
//  * THE CURRENT LIMIT lives in the DEVICE, in mA, 2500 by default
//    (set_current_limit / get_current_limit — vendored motors.hpp:974, :1237;
//    motors.h "The default value is 2500 mA"; chunk MECH-S-A, A4 register
//    HA-219..HA-221). The shim REMEMBERS it and enforces nothing (the current
//    reading is whatever the test writes); knobs make the device refuse the
//    write (PROS_ERR), keep a DIFFERENT number than was asked (VEXos's own
//    share — HA-221), or fail the read.
//  * THE BRAKE MODE IS APPLIED ONLY BY brake() (chunk DEFECTS2, 2026-09-26;
//    HA-142). move_voltage() "will not respect brake modes, and simply sets
//    the voltage" (vendored motors.h:253-254; the same note on motor_move,
//    :52); brake() "stops the motor using the currently configured brake
//    mode ... sets motor velocity to zero, which will cause it to act
//    according to the set brake mode" (motors.hpp:260-284). So each port
//    carries a CONTROL STATE — how the motor is being driven right now:
//      MotorControl::Voltage — the last movement call was move_voltage(); at
//        0 mV the brake mode is IGNORED (what the motor then physically does
//        is HA-143: unknown);
//      MotorControl::Stopped — the last movement call was brake(); the motor
//        acts per the mode it was stopped under (`stoppedUnder`, captured at
//        the call).
//    Before DEFECTS2 this shim had no brake() at all and `brake` was a field
//    unrelated to how the motor was driven — so a test could not tell "0 V,
//    mode ignored" from "stopped, mode applied", and the library's
//    "mode + 0 V" stops passed every test while holding nothing on a robot.
//
// WHAT THIS SHIM STILL CANNOT MODEL (each an HA entry, each needs a robot):
//  * what a motor does PHYSICALLY at move_voltage(0) — free-wheel or not
//    (HA-143); this shim only records that the mode was ignored;
//  * whether a set_brake_mode() sent while Stopped changes the stop already
//    in effect — the shim just stores the new mode and leaves `stoppedUnder`
//    alone. For a CHANGED mode the adapter does not rely on the answer (it
//    re-issues brake()); for the SAME mode re-sent to a stopped motor —
//    DriveBrake every tick, the driver loop's re-assert under a Brake/Hold
//    release mode — it relies on the write changing nothing (HA-144);
//  * whether a REPEATED brake() under hold re-captures the held position
//    (HA-144) — the shim re-captures `stoppedUnder` and counts, nothing more;
//  * what a VEXos disable does to a Stopped motor (HA-145);
//  * how fast a non-zero command takes a Stopped motor back (HA-146) — here,
//    at once.
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

/// How a port's motor is being DRIVEN right now (DEFECTS2 — the header's
/// "brake mode is applied only by brake()"). Not a PROS type: PROS exposes no
/// such read-back; this is the shim's record of which movement call came last.
enum class MotorControl {
    Voltage,  ///< last movement call: move_voltage() — the brake mode is IGNORED (HA-142)
    Stopped,  ///< last movement call: brake() — the motor acts per `stoppedUnder`
};

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
    /// How the motor is driven now (header). A fresh port is under voltage
    /// control at 0 mV — the state the library's stops USED to leave, which is
    /// the state a stop test must prove it left.
    MotorControl control = MotorControl::Voltage;
    /// The mode brake() was issued under — the `brake` field AT THE CALL.
    /// Meaningful only while control == Stopped; `invalid` until the first
    /// brake(). A later set_brake_mode() changes `brake`, NOT this (header:
    /// what the shim cannot model), so a test can see whether the adapter
    /// re-issued the stop under the new mode.
    v5::MotorBrake stoppedUnder = v5::MotorBrake::invalid;
    int brakeCalls = 0;  ///< successful brake() calls on this port
    /// The next N set_brake_mode() calls answer PROS_ERR and change nothing,
    /// while every other call goes through — a dropout that ends between two
    /// calls (ENODEV, "the port cannot be configured as a motor", is the only
    /// failure PROS documents for it). Lets a test refuse the mode write alone,
    /// which `disconnected` (every call refused) cannot (DEFECTS2 review).
    int refuseSetBrakeMode = 0;
    bool disconnected = false;  ///< true → every read returns the PROS sentinel
    /// The next N calls of ONE of the three C reads (motors.h: motor_get_position(),
    /// motor_get_encoder_units(), motor_get_gearing()) answer the PROS sentinel while the other two
    /// go through — a dropout that lands between two reads of one port (CAL1 Part A2a: the watch's
    /// read makes three). `disconnected` fails all three at once, so it reaches only the first of
    /// the shell's guards; these reach each guard alone. 0 = never.
    int failPositionReads = 0;
    int failUnitsReads = 0;    ///< as failPositionReads, for motor_get_encoder_units()
    int failGearingReads = 0;  ///< as failPositionReads, for motor_get_gearing()
    /// THE CURRENT LIMIT the device holds, mA (MECH-S-A; header). 2500 = the V5 default.
    std::int32_t currentLimitMa = 2500;
    /// set_current_limit() calls that reached the device (accepted or refused by the knob below).
    int setCurrentLimitCalls = 0;
    /// The next N set_current_limit() calls answer PROS_ERR and change nothing.
    int refuseSetCurrentLimit = 0;
    /// An accepted set_current_limit() leaves THIS many mA in the device instead of the asked
    /// value (the device's own answer — HA-221); 0 = take what is asked.
    std::int32_t currentLimitTakesInsteadMa = 0;
    /// The next N get_current_limit() calls answer PROS_ERR (a dropout between two reads).
    int failCurrentLimitReads = 0;
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
        s.control = shim::MotorControl::Voltage;  // leaves any stop; the mode is now IGNORED
        return 1;
    }

    /// Belief (vendored motors.hpp:260-284; HA-142): stops the motor under the
    /// CURRENTLY configured brake mode — the one call that applies the mode.
    /// A port that does not answer returns PROS_ERR and changes nothing, like
    /// every other command here.
    std::int32_t brake() const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.brakeCalls += 1;
        s.control = shim::MotorControl::Stopped;
        s.stoppedUnder = s.brake;
        // `lastVoltageMv` is deliberately left alone: it stays "the last move_voltage()
        // argument", and while Stopped it is NOT in force — read `control` first.
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
        if (s.refuseSetBrakeMode > 0) {
            s.refuseSetBrakeMode -= 1;
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

    /// Belief (vendored motors.hpp:1237; motors.h motor_set_current_limit): the
    /// limit in mA, kept by the DEVICE; 1 on success, PROS_ERR when the port
    /// does not answer (header: the knobs).
    std::int32_t set_current_limit(const std::int32_t limit, const std::uint8_t /*index*/ = 0) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        s.setCurrentLimitCalls += 1;
        if (s.refuseSetCurrentLimit > 0) {
            s.refuseSetCurrentLimit -= 1;
            return PROS_ERR;
        }
        s.currentLimitMa = s.currentLimitTakesInsteadMa > 0 ? s.currentLimitTakesInsteadMa : limit;
        return 1;
    }

    /// Belief (vendored motors.hpp:974; motors.h "The default value is 2500
    /// mA"): the limit the device holds, mA; PROS_ERR when it does not answer.
    std::int32_t get_current_limit(const std::uint8_t /*index*/ = 0) const {
        auto& s = shim::motorState(port_);
        if (s.disconnected) {
            return PROS_ERR;
        }
        if (s.failCurrentLimitReads > 0) {
            s.failCurrentLimitReads -= 1;
            return PROS_ERR;
        }
        return s.currentLimitMa;
    }

private:
    [[nodiscard]] double sign() const { return port_ < 0 ? -1.0 : 1.0; }
    std::int8_t port_;
};

}  // namespace v5
}  // namespace pros
