#pragma once
//
// ProsController — IController over pros::Controller (chunk R1a): the driver's
// hands behind the HAL.
//
// BINDS:
//  * get_analog()  → axis()      (controllerAxisToCanonical — ÷127, HA-103)
//  * get_digital() → pressed()   — a LEVEL, deliberately. This adapter NEVER
//    calls get_digital_new_press(): PROS's edge detection is stateful on the
//    device object and CONSUMES the press when read, so with two consumers
//    one silently loses (HA-104, trap C). Edge detection belongs to each
//    consumer, above the seam, via hal::ButtonEdge (controller.hpp) — N
//    consumers all see the same press. The fence guard test greps this file
//    to keep the forbidden binding structurally absent.
//  * is_connected() → isConnected() — POSITIVE validity (imu.hpp:31-33
//    polarity). Load-bearing because a disconnected controller reads 0 on
//    every channel (HA-103): without this signal, "driver unplugged" and
//    "sticks centred" are the same number.
//
// PARTNER SUPPORT: construct one adapter per physical controller
// (ControllerId::Master / ControllerId::Partner) — "which controller" is a
// construction fact, not an interface question (controller.hpp header).
//
// SENTINELS: none to screen — the controller API reports disconnection
// through is_connected() and benign zeros, not PROS_ERR (HA-103), so the
// conversion's finiteness backstop is the only guard needed.
//
// DELIBERATELY NOT here: deadband, curves, slew, rumble — driver-feel policy
// is chunk T2's layer. This seam reports what the hands are doing, verbatim
// (normalized), and nothing else.
//
// HA register: HA-103, HA-104.

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wsign-conversion"
#include "pros/misc.hpp"
#pragma GCC diagnostic pop

#include "shulib/hal/controller.hpp"
#include "shulib/hal/controller_conversion.hpp"

namespace shulib::hal::pros {

/// Which physical controller this adapter reads.
enum class ControllerId {
    Master,
    Partner,
};

class ProsController final : public IController {
public:
    explicit ProsController(ControllerId id)
        : controller_{id == ControllerId::Master ? ::pros::E_CONTROLLER_MASTER
                                                 : ::pros::E_CONTROLLER_PARTNER} {}

    /// Normalized [-1, 1] via controllerAxisToCanonical (the ONE ÷127).
    [[nodiscard]] double axis(ControllerAxis axis) const override {
        return controllerAxisToCanonical(
            static_cast<double>(controller_.get_analog(toProsAxis(axis))));
    }

    /// Level read via get_digital() — NEVER get_digital_new_press() (header).
    [[nodiscard]] bool pressed(ControllerButton button) const override {
        return controller_.get_digital(toProsButton(button)) != 0;
    }

    [[nodiscard]] bool isConnected() const override { return controller_.is_connected() != 0; }

private:
    [[nodiscard]] static ::pros::controller_analog_e_t toProsAxis(ControllerAxis a) {
        switch (a) {
            case ControllerAxis::LeftX: return ::pros::E_CONTROLLER_ANALOG_LEFT_X;
            case ControllerAxis::LeftY: return ::pros::E_CONTROLLER_ANALOG_LEFT_Y;
            case ControllerAxis::RightX: return ::pros::E_CONTROLLER_ANALOG_RIGHT_X;
            case ControllerAxis::RightY: return ::pros::E_CONTROLLER_ANALOG_RIGHT_Y;
        }
        return ::pros::E_CONTROLLER_ANALOG_LEFT_X;  // unreachable; switch is exhaustive
    }

    [[nodiscard]] static ::pros::controller_digital_e_t toProsButton(ControllerButton b) {
        switch (b) {
            case ControllerButton::L1: return ::pros::E_CONTROLLER_DIGITAL_L1;
            case ControllerButton::L2: return ::pros::E_CONTROLLER_DIGITAL_L2;
            case ControllerButton::R1: return ::pros::E_CONTROLLER_DIGITAL_R1;
            case ControllerButton::R2: return ::pros::E_CONTROLLER_DIGITAL_R2;
            case ControllerButton::Up: return ::pros::E_CONTROLLER_DIGITAL_UP;
            case ControllerButton::Down: return ::pros::E_CONTROLLER_DIGITAL_DOWN;
            case ControllerButton::Left: return ::pros::E_CONTROLLER_DIGITAL_LEFT;
            case ControllerButton::Right: return ::pros::E_CONTROLLER_DIGITAL_RIGHT;
            case ControllerButton::X: return ::pros::E_CONTROLLER_DIGITAL_X;
            case ControllerButton::B: return ::pros::E_CONTROLLER_DIGITAL_B;
            case ControllerButton::Y: return ::pros::E_CONTROLLER_DIGITAL_Y;
            case ControllerButton::A: return ::pros::E_CONTROLLER_DIGITAL_A;
        }
        return ::pros::E_CONTROLLER_DIGITAL_A;  // unreachable; switch is exhaustive
    }

    mutable ::pros::v5::Controller controller_;  // PROS's readers are non-const
};

}  // namespace shulib::hal::pros
