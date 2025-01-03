#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/tankdrive.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup frontLeft({-10, -9});
MotorGroup frontRight({8, 7});
MotorGroup backLeft({-1, -4});
MotorGroup backRight({6, 3});

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
shulib::Chassis pookster(drivetrain, sensors);

/* shulib::XDrive fifteenDriveTrain(frontLeft, frontRight, backLeft,
backRight, 2.25, 200, 2);

shulib::OdomSensors fifteenSensors(&fifteenLeftOdom, &fifteenRightOdom,
&fifteenBackOdom, nullptr);
*/

void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");

  pookster.calibrate();
  pookster.setPose({36, -60, 0});

  Task screenTask([&]() {
    while (true) {
      // print robot location to the brain screen
      lcd::print(0, "X: %f", pookster.getPose().x);
      lcd::print(1, "Y: %f", pookster.getPose().y);
      lcd::print(2, "Theta: %f", pookster.getPose().theta);

      delay(50);
    }
  });
}

void disabled() {}
void competition_initialize() {}

void autonomous() { pookster.moveToLocalPose(Pose(12, 12, 0)); }



bool wallStakeMode = false;
pros::adi::Pneumatics grabber('A', true);
pros::Motor intake(9);
pros::Motor conveyor(10);

void pooksterControls(){
  if(master.get_digital(DIGITAL_UP)){
    conveyor.move(-127);
  }
  else if(master.get_digital(DIGITAL_DOWN)){
    conveyor.move(127);
  }
  else{
    conveyor.move(0);
  }

  if(master.get_digital_new_press(DIGITAL_RIGHT)){
    grabber.toggle();
  }
  if(master.get_digital(DIGITAL_L1)){
    intake.move(127);
  }
  else if(master.get_digital(DIGITAL_L2)){
    intake.move(-127);
  }
  else{
    intake.move(0);
  }
}


void opcontrol() {
  while (true) {
    pookster.drive(master.get_analog(ANALOG_LEFT_X),
                  master.get_analog(ANALOG_LEFT_Y),
                  master.get_analog(ANALOG_RIGHT_X));

    pooksterControls();

    pros::delay(20);
  }
}