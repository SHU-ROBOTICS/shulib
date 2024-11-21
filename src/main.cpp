#include "main.h"
#include "pros/adi.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup frontLeft({-10, -9});
MotorGroup frontRight({8, 7});
MotorGroup backLeft({-1, -4});
MotorGroup backRight({6, 3});

// IMU imu(10);

// pros::Rotation left(-6);
// pros::Rotation right(9);
// pros::Rotation back(7);
// set these to nullptrs instead

shulib::OdomUnit leftOdom(nullptr, 2.75, -5.875);
shulib::OdomUnit rightOdom(nullptr, 2.75, 5.875);
shulib::OdomUnit backOdom(nullptr, 2.75, 0);

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

  chassis.calibrate();
  chassis.setPose({36, -60, 0});

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


pros::adi::Pneumatics doinker('H', true);
pros::adi::Pneumatics grabber('G', true);
pros::Motor intake(15);
pros::Motor lift(17);
pros::MotorGroup conveyor({16, -18});

bool wallStakeMode = false;

void fifteen() {
  // right: pneumatics
  if (master.get_digital_new_press(DIGITAL_RIGHT)) {
    grabber.toggle();
  }
  // y : pneumatics #2
  if (master.get_digital_new_press(DIGITAL_Y)) {
    doinker.toggle();
  }
  // r1 : wall stake mode
  if (master.get_digital_new_press(DIGITAL_R1)) {
    wallStakeMode = !wallStakeMode;
  }
  // r2 : wapow
  if (master.get_digital(DIGITAL_R2)) {
  }
  // l2: intake, l1: outtake
  if (master.get_digital(DIGITAL_L2)) {
    intake.move(127);
    conveyor.move(-127);
  } else if (master.get_digital(DIGITAL_L1)) {
    intake.move(-127);
    conveyor.move(127);
  } else {
    intake.move(0);
    conveyor.move(0);
  }
  // up: conveyor up, down: conveyor down
  if (master.get_digital(DIGITAL_UP)) {
    conveyor.move(127);
  } else if (master.get_digital(DIGITAL_DOWN)) {
    conveyor.move(-127);
  } else {
    conveyor.move(0);
  }
}

void opcontrol() {
  Motor lift(1);
  Motor intake(3);

  while (true) {
    chassis.drive(master.get_analog(ANALOG_LEFT_X),
                  master.get_analog(ANALOG_LEFT_Y),
                  master.get_analog(ANALOG_RIGHT_X));

    fifteen();

    pros::delay(20);
  }
}