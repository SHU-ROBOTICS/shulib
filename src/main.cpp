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
#include <string>
#include <fstream>
#include <iostream>
// #include "shulib/GUI/gui.c"


Controller master(CONTROLLER_MASTER);

MotorGroup pooksterLeft({-2, -4, 5, 11, -13, -14});
MotorGroup pooksterRight({-6, -7, -8, -9, 19, 20});

// IMU imu(10);

pros::Rotation left(1);
pros::Rotation right(18);
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

pros::IMU imu(8);

/* shulib::XDrive fifteenDriveTrain(frontLeft, frontRight, backLeft,
backRight, 2.25, 200, 2);

shulib::OdomSensors fifteenSensors(&fifteenLeftOdom, &fifteenRightOdom,
&fifteenBackOdom, nullptr);
*/
bool wallStakeMode = false;
pros::adi::Pneumatics grabber('A', true);
pros::Motor intake(-10);
pros::MotorGroup conveyor({17, -12});
pros::MotorGroup wallStakeLift({-15, 16}, pros::v5::MotorGears::red,pros::v5::MotorEncoderUnits::degrees);

// Global variable to store the robot's initial odometry position
Pose globalInitialPose = {0, 0, 0};  // Default to (0,0,0)

void initialize() {
  lcd::initialize();
  lcd::set_text(0, "Hello, PROS User!");

  logger().init();

  std::cout << "[IMU] Resetting IMU...\n";
  imu.reset();  
  pros::delay(500);  

  std::cout << "[IMU] Waiting for calibration...\n";
  while (imu.is_calibrating()) {
      std::cout << "[IMU] Calibrating...\n";
      pros::delay(100);
  }

  std::cout << "[IMU] Calibration complete. IMU yaw: " << imu.get_yaw() << std::endl;

  // 🔹 Capture the initial odometry and store it in `globalInitialPose`
  globalInitialPose = chassis.getPose();

  std::cout << "[DEBUG] Initial Odometry - X: " << globalInitialPose.x 
            << " inches | Y: " << globalInitialPose.y 
            << " inches | Theta: " << globalInitialPose.theta << " degrees\n";
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
  std::cout << "[DEBUG] Starting rotation calibration...\n";
  
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  // 🔹 Capture Initial Odometry
  Pose startPose = chassis.getPose();
  double initialTheta = startPose.theta;

  // 🔹 Convert Encoder Ticks to Inches
  const double wheel_diameter = 2.75;
  const double wheel_circumference = wheel_diameter * 3.1415926535;
  const double encoder_ticks_per_revolution = 360.0;
  const double inches_per_tick = wheel_circumference / encoder_ticks_per_revolution;
  const double chassis_width = 15.5;  

  double correctionFactor = shulib::getThetaCorrectionFactor();
  const double TARGET_ANGLE = 180.0;
  const double TOLERANCE = 1.0;
  const int MAX_ITERATIONS = 3;
  int iterations = 0;

  while (iterations < MAX_ITERATIONS) {
      // 🔹 Capture Initial Encoder Positions
      double start_left = left.get_position();
      double start_right = right.get_position();
      double startTheta = chassis.getPose().theta;

      std::cout << "[DEBUG] Starting rotation...\n";
      std::cout << "[DEBUG] Start Theta: " << startTheta << " degrees\n";

      chassis.drive(0, 0, 40);
      pros::delay(500); // Give time for motors to start moving

      // 🔹 Rotate Until Target Angle is Reached
      while (true) {
          double leftOdo = left.get_position() * inches_per_tick;
          double rightOdo = right.get_position() * inches_per_tick;
          double estimatedTheta = (rightOdo - leftOdo) / chassis_width;

          double total_rotation = estimatedTheta - startTheta;

          std::cout << "[DEBUG] Total Rotation: " << total_rotation 
                    << " degrees | Target: " << TARGET_ANGLE - TOLERANCE << " degrees\n";

          if (fabs(total_rotation) >= TARGET_ANGLE - TOLERANCE) break;

          chassis.drive(0, 0, 40);
          pros::delay(10);
      }

      chassis.drive(0, 0, 0);
      std::cout << "[DEBUG] Rotation finished!\n";
      pros::delay(2000); // Settling time

      // 🔹 Calculate Rotation Differences
      double final_left = left.get_position() * inches_per_tick;
      double final_right = right.get_position() * inches_per_tick;
      double finalTheta = chassis.getPose().theta;

      double odometry_rotation = fabs(final_right - final_left) / chassis_width;
      double chassis_rotation = fabs(finalTheta - startTheta);

      std::cout << "[DEBUG] Final Odometry Rotation: " << odometry_rotation << " degrees\n";
      std::cout << "[DEBUG] Final Chassis Rotation: " << chassis_rotation << " degrees\n";

      // 🔹 Update Correction Factor
      if (chassis_rotation > 1.0) {
          double new_correction = odometry_rotation / chassis_rotation;
          correctionFactor *= new_correction;
          shulib::setThetaCorrectionFactor(correctionFactor);
          std::cout << "[DEBUG] Updated Correction Factor: " << correctionFactor << "\n";
      }

      // 🔹 Reset for Next Iteration
      chassis.setPose(0, 0, 0);
      pros::delay(100);
      iterations++;
  }

  std::cout << "[DEBUG] Final Correction Factor: " << correctionFactor << "\n";
}


void limitedIntake(int n){
  intake.move(127);
  pros::delay(n);
  intake.move(0);
}

void limitedConveyor(int n){
  conveyor.move(127);
  pros::delay(n);
  conveyor.move(0);
}

void rotate_to(double target_angle) {
  std::cout << "\n\n[DEBUG] Starting rotation to " << target_angle << " degrees\n";

  // Use only odometry—IMU is ignored
  Pose startPose = chassis.getPose();
  double initialOdoTheta = startPose.theta;

  std::cout << "[DEBUG] Initial Odometry - X: " << startPose.x 
            << " inches | Y: " << startPose.y 
            << " inches | Theta: " << initialOdoTheta << " degrees\n";

  // === Convert Encoder Ticks to Inches ===
  const double wheel_diameter = 2.75;  // Diameter of tracking wheels (in inches)
  const double wheel_circumference = wheel_diameter * 3.1415926535;
  const double encoder_ticks_per_revolution = 360.0;
  const double inches_per_tick = wheel_circumference / encoder_ticks_per_revolution;
  const double chassis_width = 18; // Distance between left and right wheels

  // Capture Initial Encoder Positions
  double leftStart = left.get_position();
  double rightStart = right.get_position();

  std::cout << "[DEBUG] Initial Encoders - Left: " << leftStart 
            << " ticks | Right: " << rightStart << " ticks\n";

  double error = target_angle - initialOdoTheta;
  while (error > 180) error -= 360;
  while (error < -180) error += 360;

  int logCount = 0;
  int iterationCount = 0;
  double lastError = error;
  double rotationPower = 25.0;  // Fixed power, no dynamic changes

  while (fabs(error) > 3.0) {  
      iterationCount++;
      if (iterationCount > 500) {
          std::cout << "[ERROR] Rotation timeout! Exiting to prevent infinite spin.\n";
          break;
      }

      // Read Current Encoder Positions
      double leftOdoTicks = left.get_position();
      double rightOdoTicks = right.get_position();

      // Convert Ticks to Inches
      double leftOdo = (leftOdoTicks - leftStart) * inches_per_tick;
      double rightOdo = (rightOdoTicks - rightStart) * inches_per_tick;

      // Compute Rotation in Degrees Based on Wheel Movement
      double estimatedTheta = (rightOdo - leftOdo) / chassis_width;
      error = target_angle - estimatedTheta;

      while (error > 180) error -= 360;
      while (error < -180) error += 360;

      // Apply Fixed Rotation Power
      double rotationOutput = (error > 0) ? rotationPower : -rotationPower;

      // Debug Every Few Iterations
      if (logCount % 5 == 0) {  
          std::cout << "[ODO ROTATION] Left: " << leftOdo << " inches | Right: " << rightOdo
                    << " inches | Estimated Theta: " << estimatedTheta
                    << " degrees | Error: " << error
                    << " degrees | Output: " << rotationOutput << " power\n";
      }

      // Drive Rotation
      chassis.drive(0, 0, rotationOutput);
      logCount++;
      pros::delay(10);
  }

  // Stop Motors
  chassis.drive(0, 0, 0);
  std::cout << "[DEBUG] Rotation complete. Final Theta: " 
            << ((right.get_position() - left.get_position()) / chassis_width) << " degrees\n\n";
}

void move_to_pose(Pose target_pose, bool intaking, bool reverse, bool conv)
{
    logger().log("Starting move to pose - Target X: " + std::to_string(target_pose.x) + 

                 " Y: " + std::to_string(target_pose.y) + 
                 " Theta: " + std::to_string(target_pose.theta));
    logger().updateTelemetry("target", target_pose);

    Pose current_pose = chassis.getPose();
    double distance = current_pose.distance(target_pose);
    double angle = -shulib::radToDeg(current_pose.angle(target_pose))-270;
    while (angle > 360) angle -= 360;
    while (angle < 0) angle += 360;

    logger().log("Angle to target: " + std::to_string(angle));
    double angle_error = angle - current_pose.theta;
    logger().log("Angle error: " + std::to_string(angle_error));


    if (fabs(angle_error) > 1) {
        logger().log("Rotating to angle: " + std::to_string(angle));
        rotate_to(angle);
    }
    pros::delay(1000);
    logger().log("Moving forward");

    const double MIN_OUTPUT = 20.0;
    const double MAX_OUTPUT = 70.0;
    const double MAX_ROTATION = 30.0;
    const double MIN_ROTATION = 20.0;
    const double ACCEL_RATE = 4.0;
    const double DECEL_ZONE = 6.0;

    double currentMaxSpeed = MIN_OUTPUT;
    
    PID linearPID(12, 0.01, 0);
    PID headingPID(8, 0.02, 0);
    
    int log_counter = 0;
    while (distance > 1) {
        current_pose = chassis.getPose();
        
        distance = current_pose.distance(target_pose);

        angle = -shulib::radToDeg(current_pose.angle(target_pose))-270;
        while (angle > 360) angle -= 360;
        while (angle < 0) angle += 360;
        angle_error = angle - current_pose.theta;
        
        double forwardOutput = linearPID.update(distance);

        if (currentMaxSpeed < MAX_OUTPUT) {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT) currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (distance < DECEL_ZONE) {
            decelFactor = distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;

        double rotationOutput = headingPID.update(angle_error);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        if(reverse){
          chassis.drive(0, -forwardOutput, rotationOutput);
        } else {
          chassis.drive(0, forwardOutput, rotationOutput);
        }

        if(intaking){
          intake.move(127);
        }

        if(conv){
          conveyor.move(127);
        }
        

        log_counter++;
        if (log_counter % 25 == 0) {
            logger().log("error_rotation: " + std::to_string(angle_error) + " error_distance: " + std::to_string(distance));
            logger().log("rotation_output: " + std::to_string(rotationOutput) + " forward_output: " + std::to_string(forwardOutput));
        }
        pros::delay(10);


    }

    chassis.drive(0, 0, 0);

    if(intaking){
      limitedIntake(500);
    }

    logger().log("Move to pose complete");
}


void move_vertical(double distance_inches, bool intaking, bool conv)
{
    logger().log("Starting vertical move - Distance: " + std::to_string(distance_inches) + " inches");
    
    Pose start_pose = chassis.getPose();
    double initial_theta = start_pose.theta;
    double total_distance_traveled = 0;
    double target_distance = std::abs(distance_inches);

    const double MIN_OUTPUT = 20.0;
    const double MAX_OUTPUT = 40.0;
    const double MAX_ROTATION = 10.0;
    const double ACCEL_RATE = 2.0;
    const double DECEL_ZONE = 3.0;

    double currentMaxSpeed = MIN_OUTPUT;
    double last_y = start_pose.y;
    
    PID linearPID(15, 0.02, 0);
    PID headingPID(10, 0.01, 0.2);
    
    int log_counter = 0;
    while (total_distance_traveled < target_distance) {
        Pose current_pose = chassis.getPose();
        
        // Calculate incremental distance traveled
        double dy = std::abs(current_pose.y - last_y);
        total_distance_traveled += dy;
        last_y = current_pose.y;

        double remaining_distance = target_distance - total_distance_traveled;

        // Calculate heading error relative to initial rotation
        double heading_error = initial_theta - current_pose.theta;
        while (heading_error > 180) heading_error -= 360;
        while (heading_error < -180) heading_error += 360;
        
        double forwardOutput = linearPID.update(remaining_distance);
        // Invert output if moving backwards
        if (distance_inches < 0) forwardOutput = -forwardOutput;

        if (currentMaxSpeed < MAX_OUTPUT) {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT) currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (remaining_distance < DECEL_ZONE) {
            decelFactor = remaining_distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;

        double rotationOutput = headingPID.update(heading_error);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        chassis.drive(0, forwardOutput, 0);

       // if(intaking){
        //  intake.move(127);
       // }

       // if(conv){
        //  conveyor.move(127);
       // }

        log_counter++;
        if (log_counter % 25 == 0) {
            logger().log("error_heading: " + std::to_string(heading_error) + 
                        " distance_traveled: " + std::to_string(total_distance_traveled) +
                        " remaining: " + std::to_string(remaining_distance));
            logger().log("rotation_output: " + std::to_string(rotationOutput) + 
                        " forward_output: " + std::to_string(forwardOutput));
        }
        pros::delay(10);
    }
    
    chassis.drive(0, 0, 0);

   // if(intaking){
    //  limitedIntake(500);
   // }

    logger().log("Vertical move complete - Total distance traveled: " + std::to_string(total_distance_traveled));
}

void curve_to_pose(Pose target_pose, bool intaking, bool reverse, bool conv)
{
    logger().log("Starting move to pose - Target X: " + std::to_string(target_pose.x) + 

                 " Y: " + std::to_string(target_pose.y) + 
                 " Theta: " + std::to_string(target_pose.theta));
    logger().updateTelemetry("target", target_pose);

    Pose current_pose = chassis.getPose();
    double distance = current_pose.distance(target_pose);
    double angle = -shulib::radToDeg(current_pose.angle(target_pose))-270;
    while (angle > 360) angle -= 360;
    while (angle < 0) angle += 360;

    logger().log("Angle to target: " + std::to_string(angle));
    double angle_error = angle - current_pose.theta;
    logger().log("Angle error: " + std::to_string(angle_error));


    if (fabs(angle_error) > 1) {
        logger().log("Rotating to angle: " + std::to_string(angle));
        rotate_to(angle);
    }
    pros::delay(1000);
    logger().log("Moving forward");

    const double MIN_OUTPUT = 20.0;
    const double MAX_OUTPUT = 70.0;
    const double MAX_ROTATION = 30.0;
    const double MIN_ROTATION = 20.0;
    const double ACCEL_RATE = 4.0;
    const double DECEL_ZONE = 6.0;
    const double POWVEL_CONV_FACTOR = 0.12943587531;
    const double ARC_HEIGHT = 5.0;


    double currentMaxSpeed = MIN_OUTPUT;

    Pose startPos = chassis.getPose();
    Pose startLeftWheels = Pose(startPos.x + 12.0, startPos.y, startPos.theta);
    Pose startRightWheels = Pose(startPos.x - 12.0, startPos.y, startPos.theta);

    Pose endLeftWheels = Pose(target_pose.x + 12.0, target_pose.y, target_pose.theta);
    Pose endRightWheels = Pose(target_pose.x - 12.0, target_pose.y, target_pose.theta);

    double distLeft = startLeftWheels.distance(endLeftWheels);
    double distRight = startRightWheels.distance(endRightWheels);

    double leftMidPoint = (fabs(endLeftWheels.x - startLeftWheels.x)) / 2;
    double rightMidPoint = (fabs(endRightWheels.x - startRightWheels.x)) / 2;

    double deltaTheta = fabs(target_pose.theta - startPos.theta);

    double leftArcHeight = -1 * ((leftMidPoint - startLeftWheels.x) * (leftMidPoint - endLeftWheels.x)) * (1/deltaTheta);
    double rightArcHeight = -1 * ((rightMidPoint - startRightWheels.x) * (rightMidPoint - endRightWheels.x)) * (1/deltaTheta);

    double arcLeft = ((pow(distLeft, 2) + pow(leftArcHeight, 2)) * asin((2 * distLeft * leftArcHeight) 
      / (pow(distLeft, 2) + pow(leftArcHeight, 2)))) / (2 * leftArcHeight);
    double arcRight = ((pow(distRight, 2) + pow(rightArcHeight, 2)) * asin((2 * distRight * rightArcHeight) 
      / (pow(distRight, 2) + pow(rightArcHeight, 2)))) / (2 * rightArcHeight);

      
    float differential = arcLeft / arcRight;
    

    PID linearPID(12, 0.01, 0);
    PID headingPID(10, 0.01, 0.1);
    
    int log_counter = 0;
    while (distance > 1) {
        current_pose = chassis.getPose();
        
        distance = current_pose.distance(target_pose);

        angle = -shulib::radToDeg(current_pose.angle(target_pose))-270;
        while (angle > 360) angle -= 360;
        while (angle < 0) angle += 360;
        angle_error = angle - current_pose.theta;
        
        double forwardOutput = linearPID.update(distance);

        if (currentMaxSpeed < MAX_OUTPUT) {
            currentMaxSpeed += ACCEL_RATE;
            if (currentMaxSpeed > MAX_OUTPUT) currentMaxSpeed = MAX_OUTPUT;
        }

        double decelFactor = 1.0;
        if (distance < DECEL_ZONE) {
            decelFactor = distance / DECEL_ZONE;
            decelFactor = decelFactor * (currentMaxSpeed - MIN_OUTPUT) / currentMaxSpeed + MIN_OUTPUT / currentMaxSpeed;
        }
        forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);
        forwardOutput *= decelFactor;

        double rotationOutput = headingPID.update(angle_error);
        rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

        if(!reverse){
         // chassis.driveCurve(0, forwardOutput, rotationOutput, differential);
        } else {
         // chassis.driveCurve(0, -forwardOutput, rotationOutput, (1/differential));
        }

        if(intaking){
          intake.move(127);
        }

        if(conv){
          conveyor.move(127);
        }

        log_counter++;
        if (log_counter % 25 == 0) {
            logger().log("error_rotation: " + std::to_string(angle_error) + " error_distance: " + std::to_string(distance));
        }
        pros::delay(10);


    }
    chassis.drive(0, 0, 0);

    if(intaking){
      limitedIntake(500);
    }


    logger().log("Move to pose complete");
}

void autonomous() {
  // test_min_output();
  // MIN_OUTPUT_Y 20
  // MIN_OUTPUT_THETA 25
  // rotation_calibration();
  // moveVertical();
  chassis.setPose(-66, 0, 0);

  //move_to_pose(Pose(-60, 0, 90), false, false, false );

  rotate_to(90);
  move_vertical(24, false, false);
  rotate_to(0);
  pros::delay(500);
}

int actionCount = 1;

void pooksterControls() {
  if (master.get_digital(DIGITAL_L2)) {
    intake.move(127);
  } else {
    if (master.get_digital(DIGITAL_L1)) {
      intake.move(-127);
    } else {
      intake.move(0);
    }
  } 

  if (master.get_digital(DIGITAL_R2)) {
    conveyor.move(127);
  } else {
    if (master.get_digital(DIGITAL_R1)) {
      conveyor.move(-127);
    } else {
      conveyor.move(0);
    }
  } 

 /* if (master.get_digital_new_press(DIGITAL_A)) {
    if (wallStakeMode == false) {
        wallStakeLift.move_absolute(27, 50);
    } else {
        wallStakeLift.move_absolute(0, 20);
    }
    wallStakeMode = !wallStakeMode;
  }
  
  // r2 : wall stake lift
  if (master.get_digital_new_press(DIGITAL_Y)) {
    // check if wall stake is at 30 degrees with a tolerance of 1 degree
    if (fabs(wallStakeLift.get_position() - 27) < 2) {
      wallStakeLift.move_absolute(140, 30);
    } else {
      wallStakeLift.move_absolute(27, 30);
    }
  } */


  if(master.get_digital_new_press(DIGITAL_Y)){
    switch(actionCount % 4){
      case 0:
        wallStakeLift.move_absolute(0, 20);
      case 1:
        wallStakeLift.move_absolute(30, 50);
      case 2:
        wallStakeLift.move_absolute(140, 30);
      case 3:
        wallStakeLift.move_absolute(30, 50);
    }
    actionCount++;
  }

  if(master.get_digital_new_press(DIGITAL_RIGHT)){
    if(grabber.is_extended()){
      grabber.retract();
    } else {
      grabber.extend();
    }
  }

}

void opcontrol() {
  wallStakeLift.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

  while (true) {
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

