#include "main.h"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup frontLeft({-17, -18});
MotorGroup frontRight({19, 20});
MotorGroup backLeft({-13, -14});
MotorGroup backRight({11, 12});

IMU imu(10);

pros::Rotation left(-8);
pros::Rotation right(9);
pros::Rotation back(7);

shulib::OdomUnit leftOdom(&left, 2.75, 0);
shulib::OdomUnit rightOdom(&right, 2.75, 0);
shulib::OdomUnit backOdom(&back, 2.75, 0);

shulib::XDrive drivetrain(frontLeft, frontRight, backLeft, backRight, 2.25, 200,
                          2);

shulib::OdomSensors sensors(&leftOdom,  // vertical tracking wheel
                            &rightOdom, // vertical tracking wheel 2, set to
                                        // nullptr as we don't have a second one
                            &backOdom,  // horizontal tracking wheel
                            &imu        // inertial sensor
);

shulib::Chassis chassis(drivetrain, sensors);

void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");

  chassis.calibrate();

  Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      lcd::print(0, "X: %f", chassis.getPose().x);
      pros::lcd::print(1, "Y: %f", chassis.getPose().y);
      lcd::print(2, "Theta: %f", chassis.getPose().theta);

      delay(50);
    }
  });
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