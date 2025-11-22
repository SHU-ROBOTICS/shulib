#include "main.h" 
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "shulib/api.hpp" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/tankdrive.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/logger.hpp"
#include "shulib/pid.hpp"
#include "shulib/util.hpp"
#include <iostream>
#include <fstream>  // <-- Add this line
#include <sstream>
#include <vector>
#include <string> 


// #include "shulib/GUI/gui.c"


Controller master(CONTROLLER_MASTER);

MotorGroup pooksterLeft({-11,12,-13,14,-15});
MotorGroup pooksterRight({-16,17,-18,19,-20});

// IMU imu(10);

pros::Rotation left(-8);
pros::Rotation right(10);
pros::Rotation back(9);
// set these to nullptrs instead

shulib::OdomUnit leftOdom(&left, 2.75, -7);
shulib::OdomUnit rightOdom(&right,2.75, 7);
shulib::OdomUnit backOdom(&back, 2.75, 3.0);

shulib::TankDrive drivetrain(pooksterLeft, pooksterRight, 15.25, 3.25, 400); //trackwidth, wheeldiameter, rpm

shulib::OdomSensors sensors(&leftOdom,  // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom, // back odom unit
                            nullptr // back odom unit
);
shulib::Chassis chassis(drivetrain, sensors);

pros::IMU imu(8);

/* shulib::XDrive fifteenDriveTrain(frontLeft, frontRight, backLeft,
backRight, 2.25, 200, 2);

shulib::OdomSensors fifteenSensors(&fifteenLeftOdom, &fifteenRightOdom,
&fifteenBackOdom, nullptr);
*/

bool wallStakeMode = false;
pros::adi::Pneumatics arm('B', false);
pros::adi::Pneumatics lever('A', false);

pros::MotorGroup intake{-1,2};
pros::MotorGroup lowerConveyor{-3, 4};
pros::Motor upperConveyor{-5};
pros::Motor releaser(21);

int toggleCount = 0;

void timer(int time){
  pros::delay(time);
}

//pros::MotorGroup conveyor({17, -12});
//pros::MotorGroup wallStakeLift({-15, 16}, pros::v5::MotorGears::red,pros::v5::MotorEncoderUnits::degrees);

void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");

  logger().init();

  imu.reset();

  logger().log("IMU not calibrated, calibrating...");
  while (imu.is_calibrating()) {
    pros::delay(100);
  }

  shulib::setXCorrectionFactor(1);
  shulib::setYCorrectionFactor(1);
  shulib::setThetaCorrectionFactor(0.95);

  logger().log("IMU calibrated!");
  logger().log("IMU pitch: " + std::to_string(imu.get_pitch()));
  logger().log("IMU yaw: " + std::to_string(imu.get_yaw()));
  logger().log("IMU roll: " + std::to_string(imu.get_roll()));

  chassis.calibrate();
  chassis.setPose({36, -60, -180});
}

void disabled() {}
void competition_initialize() {}
void test_min_output() {
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  int power = 0;
  double start_y = chassis.getPose().y;
  double start_theta = chassis.getPose().theta;

  while (true) {
    power += 1;
    chassis.drive(0, power, 0);
    pros::delay(500);

    if (chassis.getPose().y > start_y + .5) {
      logger().log("Min output Y: " + std::to_string(power));
      chassis.drive(0, 0, 0);
      break;
    }
  }
  power = 0;
  while (true) {
    power += 1;
    chassis.drive(0, 0, power);
    pros::delay(500);

    if (chassis.getPose().theta > start_theta + .5) {
      logger().log("Min output theta: " + std::to_string(power));
      chassis.drive(0, 0, 0);
      break;
    }
  }
}

void rotation_calibration() {
  logger().log("Starting autonomous");
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  // Reset IMU and chassis pose
  imu.tare();
  chassis.setPose(0, 0, 0);
  pros::delay(500); // Give time for reset

  double correctionFactor = shulib::getThetaCorrectionFactor();
  const double TARGET_ANGLE = 180.0;
  const double TOLERANCE = 1.0;
  const int MAX_ITERATIONS = 3;
  int iterations = 0;

  while (iterations < MAX_ITERATIONS) {
    double start_yaw = imu.get_yaw();
    double start_theta = chassis.getPose().theta;

    logger().log("Starting rotation..");
    logger().log("Start yaw: " + std::to_string(start_yaw));
    logger().log("Start theta: " + std::to_string(start_theta));

    chassis.drive(0, 0, 40);
    pros::delay(500);
    // Rotate clockwise
    while (true) {
      double current_yaw = imu.get_yaw();
      // Calculate total rotation considering wraparound
      double total_rotation = current_yaw > start_yaw
                                  ? current_yaw - start_yaw
                                  : TARGET_ANGLE - (start_yaw - current_yaw);

      logger().updateTelemetry("total_rotation", total_rotation);
      logger().updateTelemetry("imu_yaw", imu.get_yaw());
      logger().updateTelemetry("chassis_theta", chassis.getPose().theta);

      if (total_rotation >= TARGET_ANGLE - TOLERANCE)
        break;

      chassis.drive(0, 0, 40); // Increased power for more consistent rotation
      pros::delay(10);
    }

    chassis.drive(0, 0, 0); // Stop rotation
    logger().log("Rotation finished!");
    logger().log("Settling...");
    pros::delay(2000); // Longer settle time

    // Calculate rotation amounts
    double chassis_rotation = std::abs(chassis.getPose().theta - start_theta);
    double imu_rotation = std::abs(imu.get_yaw() - start_yaw);

    logger().log("End yaw: " + std::to_string(imu.get_yaw()));
    logger().log("End theta: " + std::to_string(chassis.getPose().theta));
    logger().log("");
    logger().log("Chassis rotation: " + std::to_string(chassis_rotation));
    logger().log("IMU rotation: " + std::to_string(imu_rotation));
    logger().log("");

    // Update correction factor
    if (chassis_rotation > 1.0) { // Prevent division by very small numbers
      double new_correction = imu_rotation / chassis_rotation;
      correctionFactor *= new_correction;
      shulib::setThetaCorrectionFactor(correctionFactor);
      logger().log("Correction factor: " + std::to_string(correctionFactor));
      logger().log("");

      logger().updateTelemetry("chassis_rotation", chassis_rotation);
      logger().updateTelemetry("imu_rotation", imu_rotation);
      logger().updateTelemetry("iteration_correction", new_correction);
      logger().updateTelemetry("cumulative_correction", correctionFactor);
    }

    // Reset for next iteration
    imu.reset();
    while (imu.is_calibrating()) {
      pros::delay(10);
    }
    chassis.setPose(0, 0, 0);
    pros::delay(100);
    iterations++;
  }

  logger().updateTelemetry("final_correction", correctionFactor);
  logger().log("Correction factor: " + std::to_string(correctionFactor));
}

void limitedIntake(int n, int reverse) {
  intake.move(-127 * reverse);
  pros::delay(n);
  intake.move(0);
}

void limitedConveyor(int n){
  lowerConveyor.move(127);
  upperConveyor.move(127);
  pros::delay(n);
  lowerConveyor.move(0);
  upperConveyor.move(0);
}

void limitedComboFull(int n, int reverse){
  lowerConveyor.move(127 * reverse);
  upperConveyor.move(127 * reverse);
  releaser.move(127 * reverse);
  intake.move(-127 * reverse);
  pros::delay(n);
  lowerConveyor.move(0);
  upperConveyor.move(0);
  releaser.move(0);
  intake.move(0);
}

void limitedCombo(void* n){
  lowerConveyor.move(127);
  upperConveyor.move(127);
  intake.move(-127);
  pros::delay((int)n);
  lowerConveyor.move(0);
  upperConveyor.move(0);
  intake.move(0);
}


void rotate_to(double target_angle) {
  Pose startPose = chassis.getPose();
  logger().log("Starting rotation from " + std::to_string(startPose.theta) +
               " to " + std::to_string(target_angle) + " degrees");
  double desiredTheta = target_angle;

  logger().log("Start pose - X: " + std::to_string(startPose.x) +
               " Y: " + std::to_string(startPose.y) +
               " Theta: " + std::to_string(startPose.theta));

  const double MIN_ROTATION = 25.0; // Minimum rotation power
  const double MAX_ROTATION = 70.0; // Maximum rotation power
  const double ACCEL_RATE = 2.0;    // How fast to ramp up rotation speed
  double DECEL_ANGLE = fabs(target_angle) - 22.5;  // Start slowing down when within this angle

  if(target_angle < 0){
    DECEL_ANGLE *= -1;
  }

  double error = target_angle - chassis.getPose().theta;
  // Normalize error to [-180, 180]
  while (error > 181)
    error -= 360;
  while (error < -181)
    error += 360;

  int stuckCounter = 0;
  double lastError = error;
  double currentMaxSpeed = MAX_ROTATION; // Start at minimum speed

  // Two-phase control with separate PIDs
  if (fabs(error) > 1.0) {
    // Coarse control phase
    logger().log("Starting coarse rotation (target error < 1.0)");

    PID rotationPID(2,0.3,0.1);

    while (fabs(error) > 1.0) {
      Pose currentPose = chassis.getPose();
      error = target_angle - currentPose.theta;
      while (error > 181)
        error -= 360;
      while (error < -181)
        error += 360;

      double rotationOutput = rotationPID.update(error, 0.005);

      // Ramp up speed gradually
      /*if (currentMaxSpeed < MAX_ROTATION) {
        currentMaxSpeed += ACCEL_RATE;
        if (currentMaxSpeed > MAX_ROTATION)
          currentMaxSpeed = MAX_ROTATION;
      }*/

      // Calculate deceleration factor based on angle to target
      /*double decelFactor = 1.0;
      if (fabs(error) < DECEL_ANGLE) {
        decelFactor = fabs(error) / DECEL_ANGLE; // Linear ramp down
        // Ensure we don't go below minimum output
        decelFactor =
            decelFactor * (currentMaxSpeed - MIN_ROTATION) / currentMaxSpeed +
            MIN_ROTATION / currentMaxSpeed;
      }*/

      // Apply speed limits and deceleration
      rotationOutput =
          std::clamp(rotationOutput, -currentMaxSpeed, currentMaxSpeed);
      //rotationOutput *= decelFactor;

      // Ensure minimum power to overcome friction
      if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1) {
        rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
      }

      // Check if we're stuck
      if (fabs(error - lastError) < 0.001) {
        stuckCounter++;
        if (stuckCounter > 100) {
          logger().log("WARNING: Possibly stuck - minimal progress detected");
          logger().log("Current Theta: " + std::to_string(currentPose.theta) +
                       " Error: " + std::to_string(error));
          rotationOutput *= 1.5;
        }
      } else {
        stuckCounter = 0;
      }
      lastError = error;

      if (stuckCounter % 50 == 0) {
        logger().log("Coarse Phase - Error: " + std::to_string(error) +
                     " Output: " + std::to_string(rotationOutput) +
                     " Speed: " + std::to_string(currentMaxSpeed) +
                     " Theta: " + std::to_string(chassis.getPose().theta));
      }

      chassis.drive(0, 0, rotationOutput);
      pros::delay(5);
    }

    logger().log("Coarse rotation complete. Starting fine rotation");
    chassis.drive(0, 0, 0);
    pros::delay(100);
  }
  /*
  // Fine control phase
  logger().log("Starting fine rotation (target error < 0.5)");
  PID fineRotationPID(1, 0.005,
                      0.08); // More conservative gains for fine control

  // Reset for fine control
  stuckCounter = 0;
  lastError = error;
  currentMaxSpeed = MIN_ROTATION; // Reset speed for fine control

  while (fabs(error) > 0.5) {
    Pose currentPose = chassis.getPose();
    error = target_angle - currentPose.theta;
    while (error > 180)
      error -= 360;
    while (error < -180)
      error += 360;

    double rotationOutput = fineRotationPID.update(error);

    // In fine control, we keep speed limited
    rotationOutput =
        std::clamp(rotationOutput, -MIN_ROTATION * 1.5, MIN_ROTATION * 1.5);

    // Ensure minimum power
    if (fabs(rotationOutput) < MIN_ROTATION && fabs(rotationOutput) > 0.1) {
      rotationOutput = (rotationOutput > 0) ? MIN_ROTATION : -MIN_ROTATION;
    }

    if (fabs(error - lastError) < 0.0005) {
      stuckCounter++;
      if (stuckCounter > 100) {
        logger().log("WARNING: Possibly stuck in fine control - minimal "
                     "progress detected");
        logger().log("Current Theta: " + std::to_string(currentPose.theta) +
                     " Error: " + std::to_string(error));
        rotationOutput *= 1.5;
      }
    } else {
      stuckCounter = 0;
    }
    lastError = error;

    if (stuckCounter % 50 == 0) {
      logger().log("Fine Phase - Error: " + std::to_string(error) +
                   " Output: " + std::to_string(rotationOutput));
    }

    chassis.drive(0, 0, rotationOutput);
    pros::delay(5);
  }

  chassis.drive(0, 0, 0);
  logger().log("Rotation complete. Final pose - X: " +
               std::to_string(chassis.getPose().x) +
               " Y: " + std::to_string(chassis.getPose().y) +
               " Theta: " + std::to_string(chassis.getPose().theta));
  */
}

// Struct for storing parsed command data
struct CommandData {
  std::string command;
  double x;
  double y;
  double heading;
  double speed;
};

// **🚀 Hardcoded List of Commands (Instead of File)**
const std::vector<CommandData> autonomousCommands = {
  {"NONE", 25.1, 21.16, 90.59, 0},
  {"MOVE_WITH_HEADING", 25.09, 22.17, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.08, 23.18, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.07, 24.18, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.06, 25.19, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.05, 26.2, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.04, 27.21, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.03, 28.21, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.02, 29.22, 90.59, 50},
  {"MOVE_WITH_HEADING", 25.01, 30.23, 90.59, 50},
  {"MOVE_WITH_HEADING", 24.99, 31.24, 90.59, 50}
};


void move_to_pose(Pose target_pose, bool reverse, bool intaking, bool conv) {
  logger().log(
      "Starting move to pose - Target X: " + std::to_string(target_pose.x) +
      " Y: " + std::to_string(target_pose.y) +
      " Theta: " + std::to_string(target_pose.theta));
  logger().updateTelemetry("target", target_pose);

  Pose current_pose = chassis.getPose();
  double distance = current_pose.distance(target_pose);
  double angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
  
  angle = std::fmod(angle + 360, 360);  // Normalize to 0-360 range
  logger().log("Angle to target: " + std::to_string(angle));

  double angle_error = angle - current_pose.theta;
  logger().log("Angle error: " + std::to_string(angle_error));

  if (fabs(angle_error) > 1) {
      logger().log("Rotating to angle: " + std::to_string(angle));
      rotate_to(angle);
  }

  const double MIN_OUTPUT = 20.0;
  const double MAX_OUTPUT = 70.0;
  const double MAX_ROTATION = 30.0;
  const double ACCEL_RATE = 6.0;
  const double DECEL_ZONE = 6.0;

  double currentMaxSpeed = MIN_OUTPUT;
  PID linearPID(12, 0.03, 0);
  PID headingPID(10, 0.005, 0.25);

  int log_counter = 0;
  while (distance > 1) {
      current_pose = chassis.getPose();
      distance = current_pose.distance(target_pose);

      angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
      angle = std::fmod(angle + 360, 360);
      angle_error = angle - current_pose.theta;

      double forwardOutput = linearPID.update(distance, 5);

      // Dynamic acceleration and deceleration
      if (currentMaxSpeed < MAX_OUTPUT) {
          currentMaxSpeed = std::min(currentMaxSpeed + ACCEL_RATE, MAX_OUTPUT);
      }

      double decelFactor = (distance < DECEL_ZONE) ? (distance / DECEL_ZONE) : 1.0;
      forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed) * decelFactor;

      double rotationOutput = headingPID.update(angle_error, 0.005);
      rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

      chassis.drive(0, forwardOutput, 0);

      if (intaking) intake.move(127);
      log_counter++;
      if (log_counter % 25 == 0) {
          logger().log("error_rotation: " + std::to_string(angle_error) +
                       " error_distance: " + std::to_string(distance));
          logger().log("rotation_output: " + std::to_string(rotationOutput) +
                       " forward_output: " + std::to_string(forwardOutput));
      }

      pros::delay(5);
  }

  chassis.drive(0, 0, 0);
  if (intaking) limitedIntake(500, 1);

  logger().log("Move to pose complete");
}


void move_vertical(double distance_inches, bool intaking, bool conv) {
  logger().log("Starting vertical move - Distance: " +
               std::to_string(distance_inches) + " inches");

  Pose start_pose = chassis.getPose();
  double initial_theta = start_pose.theta;
  double total_distance_traveled = 0;
  double target_distance = std::abs(distance_inches);

  const double MIN_OUTPUT = 20.0;
  const double MAX_OUTPUT = 60.0;
  const double MAX_ROTATION = 10.0;
  const double ACCEL_RATE = 2.0;
  const double DECEL_ZONE = 5.0;

  double currentMaxSpeed = MAX_OUTPUT;
  double last_y = start_pose.y;
  double last_x = start_pose.x;

  PID linearPID(10, 2.5, 0.3);
  PID headingPID(0, 0, 0);

  int log_counter = 0;
  while (total_distance_traveled < target_distance) {
    Pose current_pose = chassis.getPose();

    // Calculate incremental distance traveled
    double dy = std::abs(current_pose.y - last_y);
    last_y = current_pose.y;

    double dx = std::abs(current_pose.x - last_x);
    last_x = current_pose.x;

    total_distance_traveled += sqrt(pow(dx, 2) + pow(dy, 2));

    double remaining_distance = target_distance - total_distance_traveled;

    // Calculate heading error relative to initial rotation
    double heading_error = initial_theta - current_pose.theta;
    while (heading_error > 180)
      heading_error -= 360;
    while (heading_error < -180)
      heading_error += 360;

    double forwardOutput = linearPID.update(remaining_distance, 0.005);
    // Invert output if moving backwards
    if (distance_inches < 0)
      forwardOutput = -forwardOutput;

    /*if (currentMaxSpeed < MAX_OUTPUT) {
      currentMaxSpeed += ACCEL_RATE;
      if (currentMaxSpeed > MAX_OUTPUT)
        currentMaxSpeed = MAX_OUTPUT;
    }

    double decelFactor = 1.0;
    if (remaining_distance < DECEL_ZONE) {
      decelFactor = remaining_distance / DECEL_ZONE;
      decelFactor =
          decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed +
          MIN_OUTPUT / currentMaxSpeed;
    }*/
    forwardOutput =
        std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
    //forwardOutput *= decelFactor;

    double rotationOutput = headingPID.update(heading_error, 0.005);
    rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

    chassis.drive(0, forwardOutput,0);

     if(intaking){
        intake.move(-127);
     }

     if(conv){
        lowerConveyor.move(-127);
        upperConveyor.move(-127);
     }

    log_counter++;
    if (log_counter % 25 == 0) {
      logger().log(
          "error_heading: " + std::to_string(heading_error) +
          " distance_traveled: " + std::to_string(total_distance_traveled) +
          " remaining: " + std::to_string(remaining_distance));
      logger().log("rotation_output: " + std::to_string(rotationOutput) +
                   " forward_output: " + std::to_string(forwardOutput));
    }
    pros::delay(5);
  }

  chassis.drive(0, 0, 0);

  if (intaking) {
    intake.move(0);
  }

  if(conv){
    lowerConveyor.move(-127);
    upperConveyor.move(-127);
  }


  logger().log("Vertical move complete - Total distance traveled: " +
               std::to_string(total_distance_traveled));
}

void positionReset(){
  pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(100);
}

void oscillation(void* cycles){
  for(int i = 0; i < (int)cycles; i++){
    move_vertical(-7, false, false);
    pros::delay(50);
    move_vertical(7, false, false);
    pros::delay(50);   
  }
}

void autonomous() {
  // test_min_output();
  // MIN_OUTPUT_Y 20
  // MIN_OUTPUT_THETA 25
  // rotation_calibration();
  // moveVertical();


  //MOVEMENT ROUTINE

  chassis.setPose(0, 0, -180);
  pros::delay(100);

  lever.extend();
  pros::delay(50);

  move_vertical(-20, false, false);
  pros::delay(100);

  lever.retract();
  pros::delay(50);

  move_vertical(20, true, true);
  pros::delay(100);

  move_vertical(-5, false, false);
  pros::delay(100);

  rotate_to(-90);
  pros::delay(100);
  chassis.setPose(0, 0, -90);
  pros::delay(100);

  move_vertical(56, true, true);
  pros::delay(750);

  move_vertical(-14, true, true);
  pros::delay(100);

  chassis.setPose(0, 0, -90);
  pros::delay(100);
  rotate_to(0);
  pros::delay(100);

  move_vertical(10, false, false);
  pros::delay(50);

  limitedComboFull(750, 1);

  move_vertical(-10, false, false);
  pros::delay(100);

  chassis.setPose(0,0,0);
  rotate_to(90);
  pros::delay(50);
  chassis.setPose(0, 0, 90);
  pros::delay(50);
  rotate_to(180);
  pros::delay(50);
  chassis.setPose(0,0,180);

  lever.extend();

  move_vertical(12, false, false);
  pros::delay(750);

  pros::Task(oscillation, (void*)6);
  pros::Task(limitedCombo,(void*)700);

  move_vertical(-12, false, false);
  pros::delay(100);

  lever.retract();

  chassis.setPose(0, 0, 180);
  pros::delay(50);
  rotate_to(90);
  pros::delay(50);
  chassis.setPose(0, 0, 90);
  pros::delay(50);
  rotate_to(0);

  move_vertical(11, false, false);
  pros::delay(50);

  limitedComboFull(750, 1);

  move_vertical(-15, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 0);
  pros::delay(100);
  rotate_to(47.5);
  pros::delay(100);
  chassis.setPose(0, 0, 47.5);
  pros::delay(100);

  move_vertical(30, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 47.5);
  pros::delay(100);
  rotate_to(0);
  pros::delay(100);
  chassis.setPose(0, 0, 0);
  pros::delay(100); 

  move_vertical(22, true, true);
  pros::delay(100);

  chassis.setPose(0, 0, 0);
  pros::delay(100);
  rotate_to(-90);
  pros::delay(100);
  chassis.setPose(0, 0, -90);
  pros::delay(100);  

  move_vertical(16, true, true);
  pros::delay(100);

  move_vertical(-8, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, -90);
  pros::delay(50);
  rotate_to(0);
  pros::delay(50);
  chassis.setPose(0, 0, 0);
  pros::delay(50);
  rotate_to(45);
  pros::delay(50);

  move_vertical(15, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 45);
  pros::delay(100);
  rotate_to(135);
  pros::delay(100);
  chassis.setPose(0, 0, 135);
  pros::delay(100);

  move_vertical(10, false, false);
  pros::delay(50);

  limitedComboFull(700, -1);

  move_vertical(-12, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 135);
  pros::delay(100);
  rotate_to(-37.5);
  pros::delay(100);
  chassis.setPose(0, 0, -45);
  pros::delay(100);

  move_vertical(24, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, -45);
  pros::delay(100);
  rotate_to(0);
  pros::delay(100);
  chassis.setPose(0, 0, 0);
  pros::delay(100);

  lever.extend();

  move_vertical(15, false, false);
  pros::delay(100);

  pros::Task(oscillation, (void*)6);
  pros::Task(limitedCombo,(void*)700);

  lever.retract();

  move_vertical(-15, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 0);
  pros::delay(50);
  rotate_to(90);
  pros::delay(50);
  chassis.setPose(0, 0, 90);
  pros::delay(50);
  rotate_to(180);

  move_vertical(12, false, false);
  pros::delay(50);

  limitedComboFull(700, 1);

  move_vertical(-15, false, false);
  pros::delay(100);

}

void pooksterControls() {

  if (master.get_digital(DIGITAL_R1)) {
    intake.move(127);
    lowerConveyor.move(127);
  } else {
    if (master.get_digital(DIGITAL_R2)) {
      intake.move(-127);
      lowerConveyor.move(-127);
    } else {
      intake.move(0);
      lowerConveyor.move(0);
    }
  }

  if (master.get_digital(DIGITAL_L1)) {
    upperConveyor.move(127);
  } else {
    if (master.get_digital(DIGITAL_L2)) {
      upperConveyor.move(-127);
    } else {
      upperConveyor.move(0);
    }
  }

  if (master.get_digital(DIGITAL_RIGHT)) {
    releaser.move(127);
  } else {
    if (master.get_digital(DIGITAL_RIGHT)) {
      releaser.move(-127);
    } else {
      releaser.move(0);
    }
  }
 
  if(master.get_digital_new_press(DIGITAL_Y)){
    if(lever.is_extended()){
      lever.retract();
    } else {
      lever.extend();
    }
  }

  if(master.get_digital_new_press(DIGITAL_LEFT)){
    if(arm.is_extended()){
      arm.retract();
    } else {
      arm.extend();
    }
  }
 
}
 
void opcontrol() {
 
  while (true) {
    chassis.drive(master.get_analog(ANALOG_LEFT_X),
                  master.get_analog(ANALOG_LEFT_Y),
                  master.get_analog(ANALOG_RIGHT_X));
    pooksterControls();
 
    // static uint32_t stuckStartTime = 0;
    // int voltage = wallStakeLift.get_voltage();
    // int absVoltage = abs(voltage);
 
    // if (absVoltage > 3000 && absVoltage < 7000) {
    //     if (stuckStartTime == 0) {
    //         stuckStartTime = pros::millis();
    //     } else if (pros::millis() - stuckStartTime >= 200) {
    //         double position = wallStakeLift.get_position();
    //         wallStakeLift.set_zero_position(voltage > 0 ? position + 180 :
    //         position - 180); pros::delay(250); stuckStartTime = 0;
    //     }
 
    // } else {
    //     stuckStartTime = 0;
    // }
 
    // if conveyor voltage gets stuck between 3000 and 7000 (or -7000 and -3000)
    // for 500ms, reverse direction for 500ms static uint32_t stuckStartTime =
    // 0; int voltage = conveyor.get_voltage(); int absVoltage = abs(voltage);
 
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
