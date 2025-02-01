#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/logger.hpp"
#include "shulib/chassis/drivetrain/tankdrive.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup pooksterLeft({-2, -3, 16, 14, -12, -13});
MotorGroup pooksterRight({-17, -18, 11, 19, -15, -20});

// IMU imu(10);

pros::Rotation left(-5);
pros::Rotation right(4);
// pros::Rotation back(7);
// set these to nullptrs instead

shulib::OdomUnit leftOdom(&left, 2.75, -2.625);
shulib::OdomUnit rightOdom(&right, 2.75, 2.625);
shulib::OdomUnit backOdom(nullptr, 2.75, 0);

shulib::TankDrive drivetrain(pooksterLeft, pooksterRight, 15.5, 3.25, 2);

shulib::OdomSensors sensors(&leftOdom,  // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom,  // horizontal odom unit
                            nullptr     // inertial sensor
);
shulib::Chassis chassis(drivetrain, sensors);

/* shulib::XDrive fifteenDriveTrain(frontLeft, frontRight, backLeft,
backRight, 2.25, 200, 2);

shulib::OdomSensors fifteenSensors(&fifteenLeftOdom, &fifteenRightOdom,
&fifteenBackOdom, nullptr);
*/

void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");
  
  logger().init();

  chassis.calibrate();
  chassis.setPose({36, -60, 0});
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

bool wallStakeMode = false;
pros::adi::Pneumatics grabber('A', true);
pros::Motor intake(9);
pros::Motor conveyor(10);
pros::MotorGroup wallStakeLift({6,-7}, pros::v5::MotorGears::red, pros::v5::MotorEncoderUnits::degrees);


void pooksterControls()
{
    if (master.get_digital(DIGITAL_X))
    {
        conveyor.move(-127);
        intake.move(127);
    }
    else
    {
        if (master.get_digital(DIGITAL_R1))
        {
            conveyor.move(-127);
        }
        else if (master.get_digital(DIGITAL_R2))
        {
            conveyor.move(127);
        }
        else
        {
            conveyor.move(0);
        }
        if (master.get_digital(DIGITAL_L1))
        {
            intake.move(127);
        }
        else if (master.get_digital(DIGITAL_L2))
        {
            intake.move(-127);
        }
        else
        {
            intake.move(0);
        }
    }

    if (master.get_digital_new_press(DIGITAL_A))
    {
        wallStakeMode = !wallStakeMode;
    }

    if (wallStakeMode)
    {
        wallStakeLift.move_absolute(180, 60);
    }
    else
    {
        wallStakeLift.move_absolute(0, 60);
    }



    if (master.get_digital_new_press(DIGITAL_RIGHT))
    {
        grabber.toggle();
    }
}

void opcontrol()
{
    wallStakeLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    while (true)
    {
        chassis.drive(master.get_analog(ANALOG_LEFT_X),
                       master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));
        logger().updateTelemetry("conveyor_voltage", conveyor.get_voltage());
        logger().updateTelemetry("wallStakeVoltage", wallStakeLift.get_voltage());
        pooksterControls();


        // static uint32_t stuckStartTime = 0;
        // int voltage = wallStakeLift.get_voltage();
        // int absVoltage = abs(voltage);
        
        // if (absVoltage > 3000 && absVoltage < 7000) {
        //     if (stuckStartTime == 0) {
        //         stuckStartTime = pros::millis();
        //     } else if (pros::millis() - stuckStartTime >= 200) {
        //         double position = wallStakeLift.get_position();
        //         wallStakeLift.set_zero_position(voltage > 0 ? position + 180 : position - 180);
        //         pros::delay(250);
        //         stuckStartTime = 0;
        //     }

        // } else {
        //     stuckStartTime = 0;
        // }



        // if conveyor voltage gets stuck between 3000 and 7000 (or -7000 and -3000) for 500ms, reverse direction for 500ms
        // static uint32_t stuckStartTime = 0;
        // int voltage = conveyor.get_voltage();
        // int absVoltage = abs(voltage);
        
        // if (absVoltage > 3000 && absVoltage < 7000) {
        //     if (stuckStartTime == 0) {
        //         stuckStartTime = pros::millis();
        //     } else if (pros::millis() - stuckStartTime >= 200) {
        //         // Reverse the direction based on current voltage sign
        //         conveyor.move(voltage > 0 ? -127 : 127);
        //         pros::delay(250);
        //         conveyor.move(0);
        //         stuckStartTime = 0;
        //     }
        // } else {
        //     stuckStartTime = 0;
        // }


        pros::delay(20);
    }
}