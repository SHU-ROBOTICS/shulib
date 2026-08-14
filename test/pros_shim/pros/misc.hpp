#pragma once
//
// HOST SHIM for <pros/misc.hpp> — a programmable pros::Controller and the
// pros::battery namespace.
//
// The controller shim models the ONE semantic that shapes the IController
// design: get_digital_new_press() CONSUMES the press on read (per-Controller-
// object edge state), so with two consumers one silently loses. The shim
// counts new_press calls AND faithfully consumes — an adapter mis-bound to it
// fails the two-consumer test on real behaviour, not on a counter alone.
//
// Beliefs modeled (each registered in docs/hardware-assumptions.md):
//  * get_analog() returns [-127, 127]; 0 when disconnected (vendored
//    misc.hpp:85-86; HA-103)
//  * get_digital() is a level; new_press consumes (misc.hpp:173-212; HA-104)
//  * battery_get_voltage()/get_current() return int32 with NO unit documented
//    in the vendored source — the mV/mA belief is from PROS's website and is
//    the WEAKEST assumption in this shim (HA-99; bench measures it first)
//  * battery get_capacity() returns PERCENT 0–100 (HA-100)
//  * controller set_text(line, col, str) writes the LCD row; the vendored doc
//    says line [0-2], col [0-14] — which CONFLICTS with HA-57's 19-column
//    claim; the shim records the exact payload and lets the test pin the
//    adapter's 19-column truncation, while HA-107 records the conflict for
//    the bench to settle
//  * (chunk R1b) pros::usd::is_installed() returns 1 with a card, 0 without
//    (vendored misc.hpp:555-568; HA-122). ADVERSARIAL DEFAULT: NO CARD —
//    the trap is a blackbox that reports success with nowhere to write, so a
//    test must opt IN to having a card, and an adapter that skips the check
//    fails the no-card test on real behaviour.
//
// HONEST LIMIT: this shim tests the adapter against OUR BELIEF about PROS; it
// cannot test the belief. Hardware tests the belief (bench runbook).

#ifndef SHULIB_HOST_PROS_SHIM
#error "test/pros_shim/ is the HOST TEST shim for PROS — it must NEVER reach a robot build. \
Only test/CMakeLists.txt defines SHULIB_HOST_PROS_SHIM."
#endif

#include <array>
#include <cstdint>
#include <string>

#include "pros/error.h"

namespace pros {

typedef enum {
    E_CONTROLLER_MASTER = 0,
    E_CONTROLLER_PARTNER,
} controller_id_e_t;

typedef enum {
    E_CONTROLLER_ANALOG_LEFT_X = 0,
    E_CONTROLLER_ANALOG_LEFT_Y,
    E_CONTROLLER_ANALOG_RIGHT_X,
    E_CONTROLLER_ANALOG_RIGHT_Y,
} controller_analog_e_t;

typedef enum {
    E_CONTROLLER_DIGITAL_L1 = 6,
    E_CONTROLLER_DIGITAL_L2,
    E_CONTROLLER_DIGITAL_R1,
    E_CONTROLLER_DIGITAL_R2,
    E_CONTROLLER_DIGITAL_UP,
    E_CONTROLLER_DIGITAL_DOWN,
    E_CONTROLLER_DIGITAL_LEFT,
    E_CONTROLLER_DIGITAL_RIGHT,
    E_CONTROLLER_DIGITAL_X,
    E_CONTROLLER_DIGITAL_B,
    E_CONTROLLER_DIGITAL_Y,
    E_CONTROLLER_DIGITAL_A,
    E_CONTROLLER_DIGITAL_POWER,
} controller_digital_e_t;

namespace shim {
struct ControllerState {
    std::array<std::int32_t, 4> analog{};   ///< raw [-127,127], indexed by channel
    std::array<bool, 19> digital{};         ///< indexed by controller_digital_e_t value
    std::array<bool, 19> newPressPrev{};    ///< per-device edge state (consume model)
    bool connected = true;
    int newPressCalls = 0;  ///< the forbidden binding, counted (HA-104)
    std::array<std::string, 3> lcdLines{};
    int setTextCalls = 0;
};
inline std::array<ControllerState, 2>& controllers() {
    static std::array<ControllerState, 2> state{};
    return state;
}
inline ControllerState& controllerState(controller_id_e_t id) {
    return controllers()[static_cast<std::size_t>(id)];
}
inline void resetControllers() { controllers() = {}; }

struct BatteryState {
    std::int32_t voltageMv = 12600;
    std::int32_t currentMa = 500;
    double capacityPercent = 100.0;
    bool errored = false;  ///< true → reads return the PROS sentinels
};
inline BatteryState& batteryState() {
    static BatteryState s;
    return s;
}
inline void resetBattery() { batteryState() = BatteryState{}; }

struct UsdState {
    /// ADVERSARIAL DEFAULT: no card (HA-122) — a test must opt in to one.
    bool installed = false;
};
inline UsdState& usdState() {
    static UsdState s;
    return s;
}
inline void resetUsd() { usdState() = UsdState{}; }
}  // namespace shim

inline namespace v5 {

class Controller {
public:
    explicit Controller(controller_id_e_t id) : id_{id} {}

    std::int32_t is_connected() { return shim::controllerState(id_).connected ? 1 : 0; }

    std::int32_t get_analog(controller_analog_e_t channel) {
        const auto& s = shim::controllerState(id_);
        // Belief (misc.hpp:86): "If the controller was not connected, then 0 is
        // returned" — NOT a sentinel. HA-103.
        return s.connected ? s.analog[static_cast<std::size_t>(channel)] : 0;
    }

    std::int32_t get_digital(controller_digital_e_t button) {
        const auto& s = shim::controllerState(id_);
        return (s.connected && s.digital[static_cast<std::size_t>(button)]) ? 1 : 0;
    }

    /// The CONSUMING edge read (HA-104): returns 1 only for the first caller
    /// after a false→true transition — the second consumer gets 0. Counted so
    /// tests can also pin "the adapter never calls this at all".
    std::int32_t get_digital_new_press(controller_digital_e_t button) {
        auto& s = shim::controllerState(id_);
        s.newPressCalls += 1;
        const auto i = static_cast<std::size_t>(button);
        const bool now = s.connected && s.digital[i];
        const bool rising = now && !s.newPressPrev[i];
        s.newPressPrev[i] = now;
        return rising ? 1 : 0;
    }

    std::int32_t set_text(std::uint8_t line, std::uint8_t /*col*/, const char* str) {
        return set_text(line, 0, std::string{str});
    }
    std::int32_t set_text(std::uint8_t line, std::uint8_t /*col*/, const std::string& str) {
        auto& s = shim::controllerState(id_);
        if (!s.connected || line > 2) {
            return PROS_ERR;
        }
        s.lcdLines[static_cast<std::size_t>(line)] = str;
        s.setTextCalls += 1;
        return 1;
    }

    std::int32_t clear_line(std::uint8_t line) { return set_text(line, 0, ""); }

private:
    controller_id_e_t id_;
};

namespace battery {

/// int32, unit NOT documented in the vendored source — modeled as mV (HA-99).
inline std::int32_t get_voltage() {
    const auto& s = shim::batteryState();
    return s.errored ? PROS_ERR : s.voltageMv;
}

/// int32, unit NOT documented in the vendored source — modeled as mA (HA-99).
inline std::int32_t get_current() {
    const auto& s = shim::batteryState();
    return s.errored ? PROS_ERR : s.currentMa;
}

/// Modeled as percent 0–100 (HA-100).
inline double get_capacity() {
    const auto& s = shim::batteryState();
    return s.errored ? static_cast<double>(PROS_ERR_F) : s.capacityPercent;
}

}  // namespace battery

}  // namespace v5

namespace usd {

/// 1 if the SD card is installed, 0 otherwise (vendored misc.hpp:555-568;
/// HA-122).
inline std::int32_t is_installed() { return shim::usdState().installed ? 1 : 0; }

}  // namespace usd
}  // namespace pros
