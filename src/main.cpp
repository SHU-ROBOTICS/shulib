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

  shulib::logger().updateTelemetry("test", master.get_digital(DIGITAL_B));

  if (master.get_digital(DIGITAL_X)) {
    chassis.setPose(0, 0, 0);
    shulib::logger().updateTelemetry("test", true);
  }

  if (master.get_digital(DIGITAL_LEFT)) {
    printf("chassis pose: %f, %f, %f\n", chassis.getPose().x, chassis.getPose().y, chassis.getPose().theta);
  }

void autonomous() { pookster.moveToLocalPose(Pose(12, 12, 0)); }

bool wallStakeMode = false;
pros::adi::Pneumatics grabber('A', true);
pros::Motor intake(9);
pros::Motor conveyor(10);

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

    if (master.get_digital_new_press(DIGITAL_RIGHT))
    {
        grabber.toggle();
    }
}

void opcontrol()
{
    while (true)
    {
        pookster.drive(master.get_analog(ANALOG_LEFT_X),
                       master.get_analog(ANALOG_LEFT_Y),
                       master.get_analog(ANALOG_RIGHT_X));

        pooksterControls();

        // if conveyor voltage spikes, reverse for 500ms
        // if (conveyor.get_voltage() > 1000)
        // {
        //     conveyor.move(-127);
        //     pros::delay(500);
        //     conveyor.move(0);
        // }


        pros::delay(20);
    }
}