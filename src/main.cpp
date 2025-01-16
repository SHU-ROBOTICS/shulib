#include "main.h"
#include "pros/adi.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/xdrive.hpp"
// #include "shulib/GUI/gui.c"

Controller master(CONTROLLER_MASTER);

MotorGroup frontLeft({-18, -19});
MotorGroup frontRight({12, 13});
MotorGroup backLeft({-16, -17});
MotorGroup backRight({15, 14});

// IMU imu(10);

pros::Rotation left(-20);
pros::Rotation right(11);
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

shulib::ControllerSettings lateralSettings(10, 0, 3, 3, 1, 100, 3, 500, 20);
shulib::ControllerSettings angularSettings(2, 0, 1, 1, 1, 100, 3, 500, 20);

shulib::Chassis chassis(drivetrain, sensors, lateralSettings, angularSettings);

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


pros::adi::Pneumatics doinker('H', false);
pros::adi::Pneumatics grabber('G', false);
pros::Motor intake(2);
pros::Motor lift(9);
pros::MotorGroup conveyor({1, -10});
pros::MotorGroup wallStake({3, -8}, pros::v5::MotorGears::red, pros::v5::MotorUnits::degrees);

double conveyorSpeed = .8;
double intakeSpeed = 1;

void fifteen() {
  // right: pneumatics
  if (master.get_digital_new_press(DIGITAL_RIGHT)) {
    grabber.toggle();
  }
  // y : pneumatics #2
  if (master.get_digital_new_press(DIGITAL_Y)) {
    doinker.toggle();
  }
  // r1 : wall stake setup
  if (master.get_digital_new_press(DIGITAL_R1)) {
    if (abs(wallStake.get_position()) < 1) {
      wallStake.move_absolute(37, 50);
    } else {
      wallStake.move_absolute(0, 20);
    }
  }
  // r2 : wall stake lift
  if (master.get_digital_new_press(DIGITAL_R2)) {
    // check if wall stake is at 37 degrees with a tolerance of 1 degree
    if (abs(wallStake.get_position() - 37) < 2) {
      wallStake.move_absolute(140, 30);
    } else {
      wallStake.move_absolute(37, 30);
    }
  }
  // l2: intake, l1: outtake
  if (master.get_digital(DIGITAL_L2)) {
    intake.move(127 * intakeSpeed);
    conveyor.move(-127 * conveyorSpeed);
  } else if (master.get_digital(DIGITAL_L1)) {
    intake.move(-127 * intakeSpeed);
    conveyor.move(127 * conveyorSpeed);
  } else if (master.get_digital(DIGITAL_UP)) { // up: conveyor up, down: conveyor down
    conveyor.move(127 * conveyorSpeed);
  } else if (master.get_digital(DIGITAL_DOWN)) {
    conveyor.move(-127 * conveyorSpeed);
  } else {
    intake.move(0);
    conveyor.move(0);
  }
}

void opcontrol() {
  wallStake.move_absolute(0, 10);

  while (true) {
    chassis.drive(master.get_analog(ANALOG_LEFT_X),
                  master.get_analog(ANALOG_LEFT_Y),
                  master.get_analog(ANALOG_RIGHT_X));

    fifteen();
    printf("wallStake: %f\n", wallStake.get_position());

    pros::delay(20);
  }
}