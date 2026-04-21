#include "main.h"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"

pros::MotorGroup left({11, -12, 13, -14}, pros::MotorGearset::blue);
pros::MotorGroup right({-16, 17, -18, 19}, pros::MotorGearset::blue);

lemlib::Drivetrain tachyon(&left, &right, 15, 3, 450 /*placeholder*/, 2);

pros::Imu imu(6);
pros::Rotation horizontal(20);
pros::Rotation vertical(-15);

pros::MotorGroup intake({1, -2});
pros::MotorGroup conveyor({3, -4});

pros::adi::Pneumatics column('A', false);
pros::adi::Pneumatics releaser('B', false);
pros::adi::Pneumatics unloader('C', false);
pros::adi::Pneumatics descore('D', false);

lemlib::TrackingWheel horizontalTracking(&horizontal, 1.5, -4);
lemlib::TrackingWheel verticalTracking(&vertical, 1.5, 0);

lemlib::OdomSensors odoms(&verticalTracking, nullptr, &horizontalTracking, nullptr, &imu);
lemlib::ControllerSettings translational(6, 
  0, 
  3, 
  0, 
  1, 
  100, 
  3, 
  500, 
  0);

lemlib::ControllerSettings rotational(1.25, 
  0, 
  2, 
  0, 
  1, 
  100, 
  3, 
  500, 
  0);

lemlib::Chassis chassis(tachyon, translational, rotational, odoms);

void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");
}

void disabled() {}

void competition_initialize() {}

void tempMovement(int time, int backwards){
  chassis.arcade(100 * backwards, 0);
  pros::delay(time);
  chassis.arcade(0,0);
}

void tempTurn(int time, int backwards){
  chassis.arcade(0, 100 * backwards);
  pros::delay(time);
  chassis.arcade(0,0);
}

struct intakeParams{
  int time;
  int mode;
  int releasePower;
};

void limitedIntake(void* params){
  intakeParams* args = static_cast<intakeParams*>(params);

  int time = args->time;
  int mode = args->mode;
  int releasePower = args->releasePower;

  intake.move(-90);
  conveyor.move(90);

  pros::delay(time);

  intake.move(0);
  conveyor.move(0);
}

struct tubeParams{
  int time;
  int power;
};

void tubeFunction(void* params){
  tubeParams* args = static_cast<tubeParams*>(params);

  int time = args->time;
  int power = args->power;

  for(int i = 0; i < 8; i++){
    chassis.arcade(power, 0);
    
    pros::delay(time);

    chassis.arcade(-20, 0);

    pros::delay(time);
  }

  chassis.arcade(0,0);

  pros::delay(100);
}

void autonomous() {

  tempMovement(640, 1);
  tempTurn(300, -1);
  tempMovement(180, 1);

  pros::delay(100);
  unloader.toggle();
  pros::delay(100);

  tubeParams tubeParamsOne{127, 200};
  intakeParams intakeParamsOne{2400, 0, 115};
  pros::Task unloadTask(tubeFunction, &tubeParamsOne, "troha");
  limitedIntake(&intakeParamsOne);

  tempMovement(80, -1);
  pros::delay(100);
  tempMovement(540, -1);
  pros::delay(100);

  pros::delay(100);
  unloader.toggle();
  pros::delay(100);

  intakeParams* intakeParamsTwo = new intakeParams{2000, 1, 90};
  limitedIntake(intakeParamsTwo);
  pros::delay(100);

  tempMovement(350, 1);
  pros::delay(100);
  tempTurn(300, -1);
  pros::delay(100);
  tempMovement(500, 1);
  pros::delay(100);
}

pros::Controller c(pros::E_CONTROLLER_MASTER);

void externalControls(){
  if (c.get_digital(DIGITAL_R1)) {
    intake.move(-127);
    conveyor.move(127);
  } else {
    if (c.get_digital(DIGITAL_R2)) {
      intake.move(127);
      conveyor.move(-127);
    } else {
		  intake.move(0);
      conveyor.move(0);
    }
  }

  if(c.get_digital_new_press(DIGITAL_RIGHT)){
    unloader.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_Y)){
    releaser.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_L1)){
	descore.toggle();
  }

  if(c.get_digital_new_press(DIGITAL_L2)){
	column.toggle();
  }
}

void opcontrol() {
	while (true) {
		double vert = c.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		double rotate = c.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		chassis.arcade(vert, rotate);

		externalControls();

    	pros::lcd::print(1, "x %f", chassis.getPose().x);
    	pros::lcd::print(2, "y %f", chassis.getPose().y);
    	pros::lcd::print(3, "theta %f", chassis.getPose().theta);

		pros::delay(10);
	}
}