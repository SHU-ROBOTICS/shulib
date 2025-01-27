#include "main.h"
#include "pros/adi.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
#include "shulib/logger.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup frontLeft({9});
MotorGroup frontRight({-10});
MotorGroup backLeft({-1});
MotorGroup backRight({3});

// IMU imu(10);

pros::Rotation left(-20);
pros::Rotation right(-11);
pros::Rotation back(7);
// set these to nullptrs instead

shulib::OdomUnit leftOdom(nullptr, 2.75, -5.875);
shulib::OdomUnit rightOdom(nullptr, 2.75, 5.875);
shulib::OdomUnit backOdom(nullptr, 2.75, 4);

shulib::XDrive drivetrain(frontLeft, frontRight, backLeft, backRight, 2.25, 200,
                          2);

shulib::OdomSensors sensors(&leftOdom,   // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom,   // horizontal odom unit
                            nullptr       // inertial sensor
);

shulib::Chassis chassis(drivetrain, sensors);

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

void opcontrol() {
  while (true) {
    chassis.drive(master.get_analog(ANALOG_LEFT_X),
                  master.get_analog(ANALOG_LEFT_Y),
                  master.get_analog(ANALOG_RIGHT_X));

    pros::delay(20);
  }
}