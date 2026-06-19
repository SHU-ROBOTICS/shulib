// src/main.cpp
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
#include <fstream>
#include <sstream>
#include <vector>
#include <string> 


// #include "shulib/GUI/gui.c"


Controller master(CONTROLLER_MASTER);

MotorGroup pooksterRight({16,-17,18,-19,20});
MotorGroup pooksterLeft({11,-12,13,-14,-15});
// IMU imu(10);

pros::Rotation left(-8);
pros::Rotation right(10);
pros::Rotation back(9);
// set these to nullptrs instead

shulib::OdomUnit leftOdom(&left, 2.75, -6.5);
shulib::OdomUnit rightOdom(&right,2.75, 6.5);
shulib::OdomUnit backOdom(&back, 2.75, 2.5);

shulib::TankDrive drivetrain(pooksterLeft, pooksterRight, 15, 3.25, 400); //trackwidth, wheeldiameter, rpm

shulib::OdomSensors sensors(&leftOdom,  // left odom unit
                            &rightOdom, // right odom unit
                            &backOdom, // back odom unit
                            nullptr // back odom unit
);
shulib::Chassis chassis(drivetrain, sensors);

pros::IMU imu(6);

/* shulib::XDrive fifteenDriveTrain(frontLeft, frontRight, backLeft,
backRight, 2.25, 200, 2);

shulib::OdomSensors fifteenSensors(&fifteenLeftOdom, &fifteenRightOdom,
&fifteenBackOdom, nullptr);
*/

bool wallStakeMode = false;
pros::adi::Pneumatics arm('B', false);
pros::adi::Pneumatics lever('C', false);

pros::MotorGroup intake{2, -3};
pros::MotorGroup conveyor{4, -5};
pros::MotorGroup releaser{-6, 7};

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
  shulib::setThetaCorrectionFactor(1);

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

/*void limitedIntake(int n, int reverse) {
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
  intake.move(127 * reverse);
  pros::delay(n);
  lowerConveyor.move(0);
  upperConveyor.move(0);
  releaser.move(0);
  intake.move(0);
}

void limitedCombo(void* n){
  lowerConveyor.move(127);
  upperConveyor.move(127);
  intake.move(127);
  pros::delay((int)n);
  lowerConveyor.move(0);
  upperConveyor.move(0);
  intake.move(0);
}*/


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

    PID rotationPID(1,0.4,0);

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

  // Log final state
  Pose finalPose = chassis.getPose();
  double finalError = target_angle - finalPose.theta;
  while (finalError > 181) finalError -= 360;
  while (finalError < -181) finalError += 360;
  logger().log("[ROTATE] final_theta=" + std::to_string(finalPose.theta) +
               " | final_error=" + std::to_string(finalError) +
               " | final_pose=(" + std::to_string(finalPose.x) +
               ", " + std::to_string(finalPose.y) + ")");

  // Warn if position drifted during rotation (should be ~zero for pure rotation)
  double positionDrift = startPose.distance(finalPose);
  if (positionDrift > 1.0) {
    logger().warning("[ROTATE] position drifted " + std::to_string(positionDrift) +
                    "in during rotation (expected ~0)");
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

// Hardcoded List of Commands (Instead of File)
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

      //if (intaking) intake.move(127);
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
  //if (intaking) limitedIntake(500, 1);

  logger().log("Move to pose complete");
}


// ============================================================
//  MOVE_VERTICAL
//  Uses displacement-based remaining distance (not accumulated).
//  Includes veering diagnostics for hardware debugging.
// ============================================================
void move_vertical(double distance_inches, bool intaking, bool conv) {
  logger().log("======== MOVE_VERTICAL START ========");

  Pose start_pose = chassis.getPose();
  double initial_theta = start_pose.theta;
  double target_distance = std::abs(distance_inches);
  double remaining_distance = target_distance;

  std::string dirLabel = (distance_inches >= 0) ? "FORWARD" : "REVERSE";
  logger().log("[VERT] target=" + std::to_string(distance_inches) + "in" +
               " | direction=" + dirLabel);
  logger().log("[VERT] start_pose=(" + std::to_string(start_pose.x) +
               ", " + std::to_string(start_pose.y) +
               ", " + std::to_string(start_pose.theta) + ")");

  const double MIN_OUTPUT = 20.0;
  const double MAX_OUTPUT = 60.0;
  const double MAX_ROTATION = 10.0;
  const double ACCEL_RATE = 2.0;
  const double DECEL_ZONE = 5.0;

  double currentMaxSpeed = MAX_OUTPUT;

  PID linearPID(10, 2.5, 0.3);
  PID headingPID(0, 0, 0);

  logger().log("[VERT] linear PID: kP=10, kI=2.5, kD=0.3");
  logger().warning("[VERT] NOTE: heading PID gains are ALL ZERO -- no heading correction active");

  int loopCount = 0;
  uint32_t startTime = pros::millis();

  // ── Veering / Motor Imbalance Tracking ─────────────────
  double cumulativeLeftVel = 0;
  double cumulativeRightVel = 0;
  int velocitySamples = 0;
  double maxVelDifference = 0;
  double maxLeftTemp = 0;
  double maxRightTemp = 0;

  while (std::abs(remaining_distance) >= 0.1) {
    Pose current_pose = chassis.getPose();

    // ── FIX: Displacement-based remaining distance (not accumulated) ──
    // Previously this used accumulated step-by-step distance which inflated
    // from noise, jitter, and lateral drift. Now we measure straight-line
    // displacement from start each cycle. Noise in one cycle does not carry
    // over to the next.
    double actual_displacement = start_pose.distance(current_pose);
    remaining_distance = target_distance - actual_displacement;

    // Calculate heading error relative to initial rotation
    double heading_error = initial_theta - current_pose.theta;
    while (heading_error > 180)
      heading_error -= 360;
    while (heading_error < -180)
      heading_error += 360;

    double forwardOutput = linearPID.update(remaining_distance, 0.005);

    // Invert output if moving backwards
    if (distance_inches < 0) {
      forwardOutput = -forwardOutput;
    }

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

    chassis.drive(0, forwardOutput, 0);

    if(intaking){
      // intake.move(-127);
    }

    if(conv){
      //lowerConveyor.move(-127);
      //upperConveyor.move(-127);
    }

    // ── Per-cycle motor imbalance tracking ─────────────────
    loopCount++;
    {
      double leftVelNow = pooksterLeft.get_actual_velocity();
      double rightVelNow = pooksterRight.get_actual_velocity();
      if (fabs(forwardOutput) > 5) {
        cumulativeLeftVel += fabs(leftVelNow);
        cumulativeRightVel += fabs(rightVelNow);
        velocitySamples++;
        double diff = fabs(leftVelNow) - fabs(rightVelNow);
        if (fabs(diff) > fabs(maxVelDifference)) {
          maxVelDifference = diff;
        }
      }
    }

    // Periodic detailed status every 50 loops (~250ms)
    if (loopCount % 50 == 0) {
      uint32_t elapsed = pros::millis() - startTime;
      double leftVel = pooksterLeft.get_actual_velocity();
      double rightVel = pooksterRight.get_actual_velocity();
      double velDiff = fabs(leftVel) - fabs(rightVel);

      double leftTemp = pooksterLeft.get_temperature();
      double rightTemp = pooksterRight.get_temperature();
      if (leftTemp > maxLeftTemp) maxLeftTemp = leftTemp;
      if (rightTemp > maxRightTemp) maxRightTemp = rightTemp;

      logger().log("[VERT] t=" + std::to_string(elapsed) + "ms" +
                   " | remaining=" + std::to_string(remaining_distance) +
                   " | actual_disp=" + std::to_string(actual_displacement) +
                   " | fwd_out=" + std::to_string(forwardOutput));
      logger().log("[VERT]   heading_err=" + std::to_string(heading_error) +
                   " | pose=(" + std::to_string(current_pose.x) +
                   ", " + std::to_string(current_pose.y) +
                   ", " + std::to_string(current_pose.theta) + ")");

      // ── VEERING DIAGNOSTICS ──────────────────────────────
      logger().log("[VEER] L_vel=" + std::to_string(leftVel) +
                   " | R_vel=" + std::to_string(rightVel) +
                   " | diff=" + std::to_string(velDiff) +
                   " | " + std::string(velDiff > 5 ? "LEFT FASTER->veers RIGHT"
                                     : velDiff < -5 ? "RIGHT FASTER->veers LEFT"
                                     : "balanced"));
      logger().log("[VEER] L_temp=" + std::to_string(leftTemp) + "C" +
                   " | R_temp=" + std::to_string(rightTemp) + "C" +
                   " | " + std::string(leftTemp > rightTemp + 5 ? "LEFT HOTTER (possible tight gearbox LEFT)"
                                     : rightTemp > leftTemp + 5 ? "RIGHT HOTTER (possible tight gearbox RIGHT)"
                                     : "temps balanced"));

      if (velocitySamples > 0) {
        double avgLeft = cumulativeLeftVel / velocitySamples;
        double avgRight = cumulativeRightVel / velocitySamples;
        double pctImbalance = (avgLeft + avgRight > 0)
            ? ((avgLeft - avgRight) / ((avgLeft + avgRight) / 2.0)) * 100.0
            : 0;
        logger().log("[VEER] running_avg: L=" + std::to_string(avgLeft) +
                     " R=" + std::to_string(avgRight) +
                     " | imbalance=" + std::to_string(pctImbalance) + "%");

        if (fabs(pctImbalance) > 15) {
          std::string diagnosis = pctImbalance > 0
              ? "LEFT motors consistently faster -- robot will veer RIGHT. Check RIGHT side gearbox/friction."
              : "RIGHT motors consistently faster -- robot will veer LEFT. Check LEFT side gearbox/friction.";
          logger().warning("[VEER] SIGNIFICANT MOTOR IMBALANCE: " +
                          std::to_string(pctImbalance) + "% | " + diagnosis);
        }
      }
    }

    pros::delay(5);
  }

  chassis.drive(0, 0, 0);

  // ---- FINAL SUMMARY ----
  uint32_t totalTime = pros::millis() - startTime;
  Pose finalPose = chassis.getPose();
  double actual_final_displacement = start_pose.distance(finalPose);
  double overshoot = actual_final_displacement - target_distance;
  double heading_drift = finalPose.theta - initial_theta;

  logger().log("-------- MOVE_VERTICAL SUMMARY --------");
  logger().log("[VERT] target=" + std::to_string(distance_inches) + "in" +
               " | actual_displacement=" + std::to_string(actual_final_displacement) + "in" +
               " | overshoot=" + std::to_string(overshoot) + "in");
  logger().log("[VERT] heading_drift=" + std::to_string(heading_drift) + "deg" +
               " | start_theta=" + std::to_string(initial_theta) +
               " | end_theta=" + std::to_string(finalPose.theta));
  logger().log("[VERT] final_pose=(" + std::to_string(finalPose.x) +
               ", " + std::to_string(finalPose.y) +
               ", " + std::to_string(finalPose.theta) + ")");
  logger().log("[VERT] time=" + std::to_string(totalTime) + "ms" +
               " | loops=" + std::to_string(loopCount));

  // ── VEERING SUMMARY ──────────────────────────────────────
  logger().log("-------- VEERING ANALYSIS (" + dirLabel + " " +
               std::to_string(fabs(distance_inches)) + "in) --------");
  if (velocitySamples > 0) {
    double avgLeft = cumulativeLeftVel / velocitySamples;
    double avgRight = cumulativeRightVel / velocitySamples;
    double pctImbalance = (avgLeft + avgRight > 0)
        ? ((avgLeft - avgRight) / ((avgLeft + avgRight) / 2.0)) * 100.0
        : 0;

    logger().log("[VEER] FINAL: avg_L_vel=" + std::to_string(avgLeft) +
                 " | avg_R_vel=" + std::to_string(avgRight) +
                 " | imbalance=" + std::to_string(pctImbalance) + "%" +
                 " | max_single_diff=" + std::to_string(maxVelDifference) +
                 " | samples=" + std::to_string(velocitySamples));
    logger().log("[VEER] FINAL: max_L_temp=" + std::to_string(maxLeftTemp) + "C" +
                 " | max_R_temp=" + std::to_string(maxRightTemp) + "C");
    logger().log("[VEER] heading_drift=" + std::to_string(heading_drift) + "deg" +
                 " | lateral_offset=" + std::to_string(finalPose.x - start_pose.x) + "in");

    // ── DIAGNOSIS ──────────────────────────────────────────
    if (fabs(heading_drift) > 3.0) {
      std::string veering_direction = heading_drift > 0 ? "LEFT" : "RIGHT";
      logger().warning("[VEER] ROBOT VEERED " + veering_direction + " by " +
                      std::to_string(fabs(heading_drift)) + " degrees during straight move");

      if (fabs(pctImbalance) > 10) {
        std::string slow_side = pctImbalance > 0 ? "RIGHT" : "LEFT";
        std::string hot_side = maxLeftTemp > maxRightTemp + 3 ? "LEFT"
                             : maxRightTemp > maxLeftTemp + 3 ? "RIGHT"
                             : "NEITHER";

        logger().warning("[VEER] DIAGNOSIS: " + slow_side + " motors are slower by " +
                        std::to_string(fabs(pctImbalance)) + "%");

        if (hot_side == slow_side) {
          logger().error("[VEER] LIKELY CAUSE: " + slow_side +
                        std::string(" side gearbox is tight or has excessive friction.") +
                        " That side is BOTH slower AND hotter.");
        } else if (hot_side == "NEITHER") {
          logger().warning(std::string("[VEER] Temps are similar. Could be motor quality difference,") +
                          " weight distribution, or wheel friction. Try swapping motor groups" +
                          " left<->right to isolate if it follows the motors or the chassis.");
        } else {
          logger().warning("[VEER] Unusual: " + slow_side + " side is slower but " + hot_side +
                          " side is hotter. May be multiple issues. Check both gearboxes.");
        }
      } else {
        logger().warning(std::string("[VEER] Motors are balanced but robot still veered.") +
                        " Possible causes: weight distribution, one side has more" +
                        " wheel friction, uneven field surface, or odom wheel issue" +
                        " causing false heading readings.");
        logger().warning(std::string("[VEER] Check odom WHEEL HEALTH logs above --") +
                        " if L/R odom imbalance is large, an odom wheel" +
                        " may have different contact than the other.");
      }
    } else {
      logger().success("[VEER] heading drift under 3 degrees -- straight tracking OK");
    }
  } else {
    logger().warning("[VEER] No velocity samples collected (robot may not have moved)");
  }
  logger().log("-------- END VEERING ANALYSIS --------");

  if (fabs(overshoot) > 2.0) {
    logger().error("[VERT] LARGE ERROR: off by " + std::to_string(overshoot) + "in from target!");
  } else if (fabs(overshoot) > 1.0) {
    logger().warning("[VERT] MODERATE ERROR: off by " + std::to_string(overshoot) + "in from target");
  } else {
    logger().success("[VERT] movement within 1in tolerance");
  }

  if (intaking) {
    //intake.move(0);
  }

  if(conv){
    //lowerConveyor.move(-127);
    //upperConveyor.move(-127);
  }

  logger().log("======== MOVE_VERTICAL END ==========");
}

void positionReset(){
  pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(100);
}

void oscillation(void* cycles){
  int i = 0;
  while(i < int(cycles)){
    move_vertical(-7, false, false);
    pros::delay(50);
    move_vertical(7, false, false);
    pros::delay(50);   

    i++;
  }
}

void autonomous() {
  // test_min_output();
  // MIN_OUTPUT_Y 20
  // MIN_OUTPUT_THETA 25
  // rotation_calibration();
  // moveVertical();

  // ── FIX: Set brake mode to HOLD for predictable stopping ──
  // Without this, motors default to COAST mode and the robot drifts
  // after move_vertical sets power to zero.
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);

  chassis.setPose(0,0,0);

  rotate_to(90);
  pros::delay(100);


  //MOVEMENT ROUTINE

  /*chassis.setPose(0, 0, -180);
  pros::delay(100);

  lever.extend();
  pros::delay(2000);

  move_vertical(-20, false, false);
  pros::delay(100);

  lever.retract();
  pros::delay(100);

  move_vertical(20, true, true); //look into this later
  pros::delay(100);

  move_vertical(-5, false, false);
  pros::delay(100);

  chassis.setPose(0,0,-180);
  rotate_to(-90);
  pros::delay(100);
  chassis.setPose(0, 0, -90);
  pros::delay(100);

  move_vertical(59, true, true);
  pros::delay(100);

  limitedCombo((void*)1000);

  move_vertical(-14, true, true);
  pros::delay(100);

  chassis.setPose(0, 0, -90);
  pros::delay(100);
  rotate_to(0);
  pros::delay(100);

  move_vertical(15, false, false);
  pros::delay(50);

  limitedComboFull(750, 1);

  move_vertical(-15, false, false);
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

  move_vertical(14, false, false);
  pros::delay(750);

  pros::Task(limitedCombo,(void*)1000);
  pros::Task(oscillation, (void*)6);

  move_vertical(-14, false, false);
  pros::delay(100);

  lever.retract();

  chassis.setPose(0, 0, 180);
  pros::delay(50);
  rotate_to(90);
  pros::delay(50);
  chassis.setPose(0, 0, 90);
  pros::delay(50);
  rotate_to(0);

  move_vertical(12, false, false);
  pros::delay(50);

  limitedComboFull(750, 1);

  move_vertical(-15, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 0);
  pros::delay(100);
  rotate_to(45);
  pros::delay(100);
  chassis.setPose(0, 0, 45);
  pros::delay(100);

  move_vertical(29, false, false);
  pros::delay(100);

  chassis.setPose(0, 0, 45);
  pros::delay(100);
  rotate_to(0);
  pros::delay(100);
  chassis.setPose(0, 0, 0);
  pros::delay(100); 

  move_vertical(44, true, true);
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

  limitedComboFull(750, -1);

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
  pros::delay(100);*/

}

void pooksterControls() {

  if (master.get_digital(DIGITAL_R1)) {
    intake.move(127);
  } else {
    if (master.get_digital(DIGITAL_R2)) {
      intake.move(-127);
    } else {
      intake.move(0);
    }
  }

  if (master.get_digital(DIGITAL_L1)) {
    conveyor.move(127);
  } else {
    if (master.get_digital(DIGITAL_L2)) {
      conveyor.move(-127);
    } else {
      conveyor.move(0);
    }
  }

  if (master.get_digital_new_press(DIGITAL_DOWN)) {
    if(toggleCount == 0){
      toggleCount++;
      //test.move(127);
    } else {
      toggleCount = 0;
      //test.move(0);
    }
  } 

  if (master.get_digital(DIGITAL_Y)) {
    releaser.move(127);
  } else {
    releaser.move(0);
  }
 
}
 
void opcontrol() {
  logger().log("======== OPCONTROL START ========");

  // ── Opcontrol Drive Diagnostics ──────────────────────────
  int opLoopCount = 0;

  // Per-direction accumulators (forward vs backward)
  double fwdLeftVelSum = 0, fwdRightVelSum = 0;
  int fwdSamples = 0;
  double revLeftVelSum = 0, revRightVelSum = 0;
  int revSamples = 0;

  // Overall accumulators
  double totalLeftVelSum = 0, totalRightVelSum = 0;
  int totalSamples = 0;
  double maxLeftTemp = 0, maxRightTemp = 0;

  // Track when we last printed a diagnostic (don't spam)
  uint32_t lastDiagTime = pros::millis();
  const uint32_t DIAG_INTERVAL_MS = 2000;  // Full report every 2 seconds

  while (true) {
    int stickY = master.get_analog(ANALOG_LEFT_Y);
    int stickX = master.get_analog(ANALOG_LEFT_X);
    int stickRot = master.get_analog(ANALOG_RIGHT_X);

    chassis.drive(stickX, stickY, stickRot);
    pooksterControls();

    // ── Gather motor data every cycle ────────────────────
    opLoopCount++;
    double leftVel = pooksterLeft.get_actual_velocity();
    double rightVel = pooksterRight.get_actual_velocity();

    // Only track when the driver is actually commanding forward/backward
    // (not turning in place, not stationary)
    bool drivingStraightish = (abs(stickY) > 20 && abs(stickRot) < 30);

    if (drivingStraightish) {
      double absL = fabs(leftVel);
      double absR = fabs(rightVel);
      totalLeftVelSum += absL;
      totalRightVelSum += absR;
      totalSamples++;

      if (stickY > 0) {
        // Forward
        fwdLeftVelSum += absL;
        fwdRightVelSum += absR;
        fwdSamples++;
      } else {
        // Reverse
        revLeftVelSum += absL;
        revRightVelSum += absR;
        revSamples++;
      }
    }

    // Track temperatures
    double leftTemp = pooksterLeft.get_temperature();
    double rightTemp = pooksterRight.get_temperature();
    if (leftTemp > maxLeftTemp) maxLeftTemp = leftTemp;
    if (rightTemp > maxRightTemp) maxRightTemp = rightTemp;

    // ── Periodic diagnostic report ───────────────────────
    uint32_t now = pros::millis();
    if (now - lastDiagTime >= DIAG_INTERVAL_MS) {
      lastDiagTime = now;

      logger().log("-------- OPCONTROL DRIVE DIAGNOSTICS --------");
      logger().log("[OPCTL] temps: L=" + std::to_string(leftTemp) + "C" +
                   " R=" + std::to_string(rightTemp) + "C" +
                   " | max: L=" + std::to_string(maxLeftTemp) + "C" +
                   " R=" + std::to_string(maxRightTemp) + "C");

      if (totalSamples > 10) {
        double avgL = totalLeftVelSum / totalSamples;
        double avgR = totalRightVelSum / totalSamples;
        double pctImbalance = (avgL + avgR > 0)
            ? ((avgL - avgR) / ((avgL + avgR) / 2.0)) * 100.0
            : 0;

        logger().log("[OPCTL] overall: avg_L=" + std::to_string(avgL) +
                     " avg_R=" + std::to_string(avgR) +
                     " | imbalance=" + std::to_string(pctImbalance) + "%" +
                     " | samples=" + std::to_string(totalSamples));

        if (fabs(pctImbalance) > 10) {
          std::string slowSide = pctImbalance > 0 ? "RIGHT" : "LEFT";
          std::string fastSide = pctImbalance > 0 ? "LEFT" : "RIGHT";
          logger().warning("[OPCTL] " + slowSide + " side is slower by " +
                          std::to_string(fabs(pctImbalance)) + "% -> robot veers toward " +
                          slowSide);

          if (maxLeftTemp > maxRightTemp + 3 && slowSide == "LEFT") {
            logger().error("[OPCTL] LEFT side is BOTH slower AND hotter." +
                          std::string(" Likely LEFT gearbox friction / tight bearing."));
          } else if (maxRightTemp > maxLeftTemp + 3 && slowSide == "RIGHT") {
            logger().error("[OPCTL] RIGHT side is BOTH slower AND hotter." +
                          std::string(" Likely RIGHT gearbox friction / tight bearing."));
          } else {
            logger().warning("[OPCTL] Temps don't clearly point to one side." +
                            std::string(" Could be motor quality, weight dist, or wheel grip."));
          }
        }
      }

      // ── Forward vs backward comparison ─────────────────
      if (fwdSamples > 10) {
        double fwdL = fwdLeftVelSum / fwdSamples;
        double fwdR = fwdRightVelSum / fwdSamples;
        double fwdImb = (fwdL + fwdR > 0)
            ? ((fwdL - fwdR) / ((fwdL + fwdR) / 2.0)) * 100.0
            : 0;
        logger().log("[OPCTL] FORWARD:  avg_L=" + std::to_string(fwdL) +
                     " avg_R=" + std::to_string(fwdR) +
                     " | imbalance=" + std::to_string(fwdImb) + "%" +
                     " | samples=" + std::to_string(fwdSamples));
      }

      if (revSamples > 10) {
        double revL = revLeftVelSum / revSamples;
        double revR = revRightVelSum / revSamples;
        double revImb = (revL + revR > 0)
            ? ((revL - revR) / ((revL + revR) / 2.0)) * 100.0
            : 0;
        logger().log("[OPCTL] REVERSE:  avg_L=" + std::to_string(revL) +
                     " avg_R=" + std::to_string(revR) +
                     " | imbalance=" + std::to_string(revImb) + "%" +
                     " | samples=" + std::to_string(revSamples));
      }

      // ── Direction comparison diagnosis ─────────────────
      if (fwdSamples > 10 && revSamples > 10) {
        double fwdImb = ((fwdLeftVelSum/fwdSamples) - (fwdRightVelSum/fwdSamples));
        double revImb = ((revLeftVelSum/revSamples) - (revRightVelSum/revSamples));

        if (fabs(fwdImb) > fabs(revImb) + 5) {
          logger().warning("[OPCTL] DIRECTION CLUE: Imbalance is WORSE going forward." +
              std::string(" This suggests the problem is direction-dependent.") +
              " Possible causes: weight distribution shifts forward," +
              " front wheel contact changes, or gear engagement" +
              " differs by direction.");
        } else if (fabs(revImb) > fabs(fwdImb) + 5) {
          logger().warning("[OPCTL] DIRECTION CLUE: Imbalance is WORSE going backward.");
        } else {
          logger().log("[OPCTL] Imbalance is similar in both directions." +
              std::string(" Problem is NOT direction-dependent.") +
              " Points to: tight gearbox, odom wheel issue, or" +
              " motor quality difference.");
        }
      }

      // ── Instantaneous snapshot ─────────────────────────
      logger().log("[OPCTL] RIGHT NOW: stick_Y=" + std::to_string(stickY) +
                   " stick_rot=" + std::to_string(stickRot) +
                   " | L_vel=" + std::to_string(leftVel) +
                   " R_vel=" + std::to_string(rightVel) +
                   " | heading=" + std::to_string(chassis.getPose().theta) + "deg");
      logger().log("-------- END OPCONTROL DIAGNOSTICS --------");
    }

    pros::delay(20);
  }
}