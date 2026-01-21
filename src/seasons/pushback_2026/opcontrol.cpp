#include "shulib/seasons/pushback_2026/opcontrol.hpp"
#include "shulib/seasons/pushback_2026/mechanisms.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace shulib::seasons::pushback::opcontrol {

void run(Chassis& chassis, const RobotConfig& config) {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    Mechanisms mech(config.mechanisms);

    while (true) {
        // ─────────────────────────────────────────────────────────
        // Drivetrain Control (arcade/tank style)
        // ─────────────────────────────────────────────────────────
        int horizontal = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int vertical = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        chassis.drive(horizontal, vertical, turn);

        // ─────────────────────────────────────────────────────────
        // Intake + Conveyor (R1 = in, L1 = out)
        // ─────────────────────────────────────────────────────────
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            mech.intakeAndConveyorIn();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            mech.intakeAndConveyorOut();
        } else {
            mech.intakeStop();
            mech.conveyorStop();
        }

        // ─────────────────────────────────────────────────────────
        // Releaser (R2 = forward, L2 = backward)
        // ─────────────────────────────────────────────────────────
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            mech.releaserForward();
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            mech.releaserBackward();
        } else {
            mech.releaserStop();
        }

        // ─────────────────────────────────────────────────────────
        // Pneumatics (Y = arm toggle, LEFT = lever toggle)
        // ─────────────────────────────────────────────────────────
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            mech.toggleArm();
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            mech.toggleLever();
        }

        pros::delay(20);
    }
}

}  // namespace shulib::seasons::pushback::opcontrol