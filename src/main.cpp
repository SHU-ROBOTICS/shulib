#include "main.h" 
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "pros/apix.h" // IWYU pragma: keep
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/drivetrain/tankdrive.hpp"
#include "shulib/chassis/odometry.hpp"
#include "shulib/logger.hpp"
#include "shulib/pid.hpp"
#include "shulib/util.hpp"
#include <vector>
#include <string> 


Controller master(CONTROLLER_MASTER);

MotorGroup pooksterRight({11, -13, 15, -17, 19});
MotorGroup pooksterLeft({12, -14, 16, -18, 20});

pros::Rotation right(-8);
pros::Rotation left(10);
pros::Rotation back(9);

shulib::OdomUnit leftOdom(&left, 1.5, -6.5);
shulib::OdomUnit rightOdom(&right,1.5, 6.5);
shulib::OdomUnit backOdom(&back, 1.5, -4.0);

shulib::TankDrive drivetrain(pooksterLeft, pooksterRight, 15, 3.00, 400);

shulib::OdomSensors sensors(&leftOdom,
                            &rightOdom,
                            &backOdom,
                            nullptr
);
shulib::Chassis chassis(drivetrain, sensors);

pros::IMU imu(6);

bool wallStakeMode = false;
pros::adi::Pneumatics arm('B', false);
pros::adi::Pneumatics lever('C', false);
pros::adi::Pneumatics solenoid('D', false);

pros::MotorGroup intake{-6, 7};
pros::MotorGroup conveyor{2, -3, -4, 5};
pros::Motor releaser(1);

int toggleCount = 0;

void timer(int time){
  pros::delay(time);
}

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
  shulib::setThetaCorrectionFactor(1.275);

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
  imu.tare();
  chassis.setPose(0, 0, 0);
  pros::delay(500);

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
    while (true) {
      double current_yaw = imu.get_yaw();
      double total_rotation = current_yaw > start_yaw
                                  ? current_yaw - start_yaw
                                  : TARGET_ANGLE - (start_yaw - current_yaw);

      logger().updateTelemetry("total_rotation", total_rotation);
      logger().updateTelemetry("imu_yaw", imu.get_yaw());
      logger().updateTelemetry("chassis_theta", chassis.getPose().theta);

      if (total_rotation >= TARGET_ANGLE - TOLERANCE)
        break;

      chassis.drive(0, 0, 40);
      pros::delay(10);
    }

    chassis.drive(0, 0, 0);
    logger().log("Rotation finished!");
    logger().log("Settling...");
    pros::delay(2000);

    double chassis_rotation = std::abs(chassis.getPose().theta - start_theta);
    double imu_rotation = std::abs(imu.get_yaw() - start_yaw);

    logger().log("End yaw: " + std::to_string(imu.get_yaw()));
    logger().log("End theta: " + std::to_string(chassis.getPose().theta));
    logger().log("Chassis rotation: " + std::to_string(chassis_rotation));
    logger().log("IMU rotation: " + std::to_string(imu_rotation));

    if (chassis_rotation > 1.0) {
      double new_correction = imu_rotation / chassis_rotation;
      correctionFactor *= new_correction;
      shulib::setThetaCorrectionFactor(correctionFactor);
      logger().log("Correction factor: " + std::to_string(correctionFactor));

      logger().updateTelemetry("chassis_rotation", chassis_rotation);
      logger().updateTelemetry("imu_rotation", imu_rotation);
      logger().updateTelemetry("iteration_correction", new_correction);
      logger().updateTelemetry("cumulative_correction", correctionFactor);
    }

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

void limitedIntake(int n, int reverse, int releaserMode, int releasePower){
  if(reverse == 1){
    intake.move(-127);
  } else {
    intake.move(-40 * reverse);
  }
  conveyor.move(-127 * reverse);
  releaser.move(releasePower * releaserMode);
  pros::delay(n);
  intake.move(0);
  conveyor.move(0);
  releaser.move(0);
}


// ============================================================
//  ROTATE_TO - with diagnostic logging
// ============================================================
void rotate_to(double target_angle) {
  Pose startPose = chassis.getPose();

  logger().log("-------- ROTATE_TO START --------");
  logger().log("[ROTATE] target=" + std::to_string(target_angle) +
               "deg | start_theta=" + std::to_string(startPose.theta) +
               "deg | start_pose=(" + std::to_string(startPose.x) +
               ", " + std::to_string(startPose.y) + ")");

  const double MAX_ROTATION = 100.0;
  const double ACCEL_RATE = 2.0;
  double DECEL_ANGLE = fabs(target_angle) - 45;

  if(target_angle < 0){
    DECEL_ANGLE *= -1;
  }

  double error = target_angle - chassis.getPose().theta;
  while (error > 181) error -= 360;
  while (error < -181) error += 360;

  logger().log("[ROTATE] initial_error=" + std::to_string(error) + "deg");

  int stuckCounter = 0;
  double lastError = error;
  double currentMaxSpeed = MAX_ROTATION;

  if (fabs(error) > 0.1) {
    PID rotationPID(0.5, 0, 0.015, 25);

    logger().log("[ROTATE] PID gains: kP=0.5, kI=0, kD=0.015, kC=25");
    logger().log("[ROTATE] entering coarse phase (exit when error < 1.0deg)");

    int loopCount = 0;
    int signChanges = 0;
    double prevErrorSign = (error > 0) ? 1.0 : -1.0;
    uint32_t startTime = pros::millis();

    while (fabs(error) > 1.0) {
      Pose currentPose = chassis.getPose();
      error = target_angle - currentPose.theta;
      while (error > 181) error -= 360;
      while (error < -181) error += 360;

      double rotationOutput = rotationPID.update(error, 0.001);

      rotationOutput = std::clamp(rotationOutput, -currentMaxSpeed, currentMaxSpeed);

      // Detect oscillation (error sign changes)
      double currentSign = (error > 0) ? 1.0 : -1.0;
      if (currentSign != prevErrorSign) {
        signChanges++;
        if (signChanges >= 3) {
          logger().warning("[ROTATE] OSCILLATION detected - " + std::to_string(signChanges) +
                          " sign changes | error=" + std::to_string(error) +
                          " | output=" + std::to_string(rotationOutput));
        }
        prevErrorSign = currentSign;
      }

      // Stuck detection
      if (fabs(error - lastError) < 0.001) {
        stuckCounter++;
        if (stuckCounter > 100) {
          logger().warning("[ROTATE] STUCK - no progress for " + std::to_string(stuckCounter) +
                          " cycles | theta=" + std::to_string(currentPose.theta) +
                          " | error=" + std::to_string(error));
          rotationOutput *= 1.5;
        }
      } else {
        stuckCounter = 0;
      }
      lastError = error;

      // Periodic status log every 50 iterations (~250ms)
      loopCount++;
      if (loopCount % 50 == 0) {
        uint32_t elapsed = pros::millis() - startTime;
        logger().log("[ROTATE] t=" + std::to_string(elapsed) + "ms" +
                     " | error=" + std::to_string(error) +
                     " | output=" + std::to_string(rotationOutput) +
                     " | theta=" + std::to_string(currentPose.theta) +
                     " | loops=" + std::to_string(loopCount));
      }

      chassis.drive(0, 0, rotationOutput);
      pros::delay(5);
    }

    uint32_t totalTime = pros::millis() - startTime;
    chassis.drive(0, 0, 0);
    pros::delay(100);

    Pose finalPose = chassis.getPose();
    double finalError = target_angle - finalPose.theta;
    while (finalError > 181) finalError -= 360;
    while (finalError < -181) finalError += 360;

    logger().log("[ROTATE] coarse phase done in " + std::to_string(totalTime) + "ms" +
                 " | " + std::to_string(loopCount) + " loops" +
                 " | " + std::to_string(signChanges) + " oscillations");
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
  } else {
    logger().log("[ROTATE] error < 0.1deg, skipping rotation");
  }

  logger().log("-------- ROTATE_TO END ----------");
}

// Struct for storing parsed command data
struct CommandData {
  std::string command;
  double x;
  double y;
  double heading;
  double speed;
};

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


// ============================================================
//  MOVE_TO_POSE - with diagnostic logging
// ============================================================
void move_to_pose(Pose target_pose, bool reverse, bool intaking, bool conv) {
  logger().log("-------- MOVE_TO_POSE START --------");

  Pose current_pose = chassis.getPose();
  double distance = current_pose.distance(target_pose);
  double angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
  angle = std::fmod(angle + 360, 360);
  double angle_error = angle - current_pose.theta;

  logger().log("[M2P] target=(" + std::to_string(target_pose.x) +
               ", " + std::to_string(target_pose.y) +
               ", " + std::to_string(target_pose.theta) + ")");
  logger().log("[M2P] start=(" + std::to_string(current_pose.x) +
               ", " + std::to_string(current_pose.y) +
               ", " + std::to_string(current_pose.theta) + ")");
  logger().log("[M2P] distance=" + std::to_string(distance) +
               "in | angle_to_target=" + std::to_string(angle) +
               "deg | angle_error=" + std::to_string(angle_error) + "deg");

  logger().updateTelemetry("target", target_pose);

  if (fabs(angle_error) > 1) {
    logger().log("[M2P] heading error > 1deg, rotating first");
    rotate_to(angle);
  } else {
    logger().log("[M2P] heading error < 1deg, skipping initial rotation");
  }

  const double MIN_OUTPUT = 20.0;
  const double MAX_OUTPUT = 70.0;
  const double MAX_ROTATION = 30.0;
  const double ACCEL_RATE = 6.0;
  const double DECEL_ZONE = 6.0;

  double currentMaxSpeed = MIN_OUTPUT;
  PID linearPID(12, 0.03, 0, 3.5);
  PID headingPID(10, 0.005, 0.25, 3.5);

  logger().log("[M2P] linear PID: kP=12, kI=0.03, kD=0, kC=3.5");
  logger().log("[M2P] heading PID: kP=10, kI=0.005, kD=0.25, kC=3.5");
  logger().log("[M2P] MAX_OUTPUT=" + std::to_string(MAX_OUTPUT) +
               " | DECEL_ZONE=" + std::to_string(DECEL_ZONE) + "in");

  int loopCount = 0;
  uint32_t startTime = pros::millis();
  double prevDistance = distance;

  while (distance > 1) {
    current_pose = chassis.getPose();
    distance = current_pose.distance(target_pose);

    angle = -shulib::radToDeg(current_pose.angle(target_pose)) - 270;
    angle = std::fmod(angle + 360, 360);
    angle_error = angle - current_pose.theta;

    double forwardOutput = linearPID.update(distance, 5);

    if (currentMaxSpeed < MAX_OUTPUT) {
      currentMaxSpeed = std::min(currentMaxSpeed + ACCEL_RATE, MAX_OUTPUT);
    }

    double decelFactor = (distance < DECEL_ZONE) ? (distance / DECEL_ZONE) : 1.0;
    double rawForward = forwardOutput;
    forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed) * decelFactor;

    double rotationOutput = headingPID.update(angle_error, 0.005);
    double rawRotation = rotationOutput;
    rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

    chassis.drive(0, forwardOutput, 0);

    // Log clamping events
    if (fabs(rawForward) > currentMaxSpeed) {
      if (loopCount % 50 == 0) {
        logger().warning("[M2P] forward output CLAMPED: " + std::to_string(rawForward) +
                        " -> " + std::to_string(forwardOutput));
      }
    }

    // Warn if distance is increasing (moving away from target)
    if (distance > prevDistance + 0.5) {
      logger().warning("[M2P] MOVING AWAY from target! distance increased " +
                      std::to_string(prevDistance) + " -> " + std::to_string(distance));
    }
    prevDistance = distance;

    // Periodic status every 50 loops (~250ms)
    loopCount++;
    if (loopCount % 50 == 0) {
      uint32_t elapsed = pros::millis() - startTime;
      logger().log("[M2P] t=" + std::to_string(elapsed) + "ms" +
                   " | dist=" + std::to_string(distance) +
                   " | fwd=" + std::to_string(forwardOutput) +
                   " | heading_err=" + std::to_string(angle_error) +
                   " | rot=" + std::to_string(rotationOutput) +
                   " | pose=(" + std::to_string(current_pose.x) +
                   ", " + std::to_string(current_pose.y) +
                   ", " + std::to_string(current_pose.theta) + ")");
    }

    pros::delay(5);
  }

  chassis.drive(0, 0, 0);

  uint32_t totalTime = pros::millis() - startTime;
  Pose finalPose = chassis.getPose();
  double finalError = finalPose.distance(target_pose);

  logger().log("[M2P] complete in " + std::to_string(totalTime) + "ms" +
               " | " + std::to_string(loopCount) + " loops");
  logger().log("[M2P] final_pose=(" + std::to_string(finalPose.x) +
               ", " + std::to_string(finalPose.y) +
               ", " + std::to_string(finalPose.theta) +
               ") | final_error=" + std::to_string(finalError) + "in");
  logger().log("-------- MOVE_TO_POSE END ----------");
}


// ============================================================
//  MOVE_VERTICAL - with extensive diagnostic logging
//  This is the primary suspect for forward movement issues.
//  Extra diagnostics compare accumulated distance vs actual
//  displacement to detect odometry drift.
// ============================================================
void move_vertical(double distance_inches, bool intaking, bool conv) {
  logger().log("======== MOVE_VERTICAL START ========");

  Pose start_pose = chassis.getPose();
  double initial_theta = start_pose.theta;
  double total_distance_traveled = 0;
  double target_distance = std::abs(distance_inches);
  double remaining_distance = target_distance;

  logger().log("[VERT] target=" + std::to_string(distance_inches) + "in" +
               " (abs=" + std::to_string(target_distance) + "in)" +
               " | direction=" + std::string(distance_inches >= 0 ? "FORWARD" : "REVERSE"));
  logger().log("[VERT] start_pose=(" + std::to_string(start_pose.x) +
               ", " + std::to_string(start_pose.y) +
               ", " + std::to_string(start_pose.theta) + ")");

  const double MAX_OUTPUT = 60.0;
  const double MAX_ROTATION = 10.0;
  const double ACCEL_RATE = 2.0;
  const double DECEL_ZONE = 5.0;

  logger().log("[VERT] constants: MAX_OUTPUT=" + std::to_string(MAX_OUTPUT) +
               " | MAX_ROTATION=" + std::to_string(MAX_ROTATION));

  double currentMaxSpeed = MAX_OUTPUT;
  double last_y = start_pose.y;
  double last_x = start_pose.x;

  PID linearPID(5, 0, 0.1, 25);
  PID headingPID(0, 0, 0, 0);

  logger().log("[VERT] linear PID: kP=5, kI=0, kD=0.1, kC=25");
  logger().warning("[VERT] NOTE: heading PID gains are ALL ZERO - no heading correction active");

  double currentOutput = (pooksterLeft.get_actual_velocity() + pooksterRight.get_actual_velocity()) / 2;
  double prevOutput;
  bool stopped = false;

  int loopCount = 0;
  int clampCount = 0;
  int signChangeCount = 0;
  double prevRemainingSign = 1.0;
  double maxForwardOutput = 0;
  double maxPositionJump = 0;
  uint32_t startTime = pros::millis();

  // ── Veering / Motor Imbalance Tracking ─────────────────
  double cumulativeLeftVel = 0;
  double cumulativeRightVel = 0;
  int velocitySamples = 0;
  double maxVelDifference = 0;  // Largest single-sample L-R difference
  double maxLeftTemp = 0;
  double maxRightTemp = 0;

  while (std::abs(remaining_distance) >= 0.1) {
    Pose current_pose = chassis.getPose();

    // Calculate incremental distance traveled
    double dy = std::abs(current_pose.y - last_y);
    double dx = std::abs(current_pose.x - last_x);
    double stepDist = sqrt(pow(dx, 2) + pow(dy, 2));

    // Track largest single-step jump (possible sensor glitch)
    if (stepDist > maxPositionJump) {
      maxPositionJump = stepDist;
    }
    if (stepDist > 2.0) {
      logger().warning("[VERT] LARGE POSITION JUMP: " + std::to_string(stepDist) +
                      "in in one cycle! dx=" + std::to_string(dx) +
                      " dy=" + std::to_string(dy) +
                      " | possible sensor glitch or lag");
    }

    last_y = current_pose.y;
    last_x = current_pose.x;

    total_distance_traveled += stepDist;
    remaining_distance = target_distance - total_distance_traveled;

    // DIAGNOSTIC: Compare accumulated distance vs actual displacement from start
    // If these diverge, the accumulation method is adding phantom distance
    double actual_displacement = start_pose.distance(current_pose);
    double accumulation_error = total_distance_traveled - actual_displacement;

    if (fabs(accumulation_error) > 1.0 && loopCount % 50 == 0) {
      logger().warning("[VERT] ACCUMULATION DRIFT: accumulated=" + std::to_string(total_distance_traveled) +
                      " vs actual_displacement=" + std::to_string(actual_displacement) +
                      " | drift=" + std::to_string(accumulation_error) + "in" +
                      " | This means odometry noise is adding phantom distance");
    }

    // Calculate heading error relative to initial rotation
    double heading_error = initial_theta - current_pose.theta;
    while (heading_error > 180) heading_error -= 360;
    while (heading_error < -180) heading_error += 360;

    double forwardOutput = linearPID.update(remaining_distance, 0.001);
    if (distance_inches < 0) forwardOutput = -forwardOutput;

    double rawForward = forwardOutput;
    forwardOutput = std::clamp(forwardOutput, -currentMaxSpeed, currentMaxSpeed);

    if (fabs(rawForward) > currentMaxSpeed) {
      clampCount++;
    }
    if (fabs(forwardOutput) > maxForwardOutput) {
      maxForwardOutput = fabs(forwardOutput);
    }

    double rotationOutput = headingPID.update(heading_error, 0.001);
    rotationOutput = std::clamp(rotationOutput, -MAX_ROTATION, MAX_ROTATION);

    // Detect if remaining_distance changed sign (overshot target)
    double currentRemainingSign = (remaining_distance >= 0) ? 1.0 : -1.0;
    if (currentRemainingSign != prevRemainingSign && loopCount > 0) {
      signChangeCount++;
      logger().warning("[VERT] TARGET CROSSED - remaining went from " +
                      std::string(prevRemainingSign > 0 ? "positive" : "negative") +
                      " to " + std::string(currentRemainingSign > 0 ? "positive" : "negative") +
                      " | remaining=" + std::to_string(remaining_distance) +
                      " | This means the robot overshot and is correcting");
    }
    prevRemainingSign = currentRemainingSign;

    chassis.drive(0, forwardOutput, 0);

    prevOutput = currentOutput;
    currentOutput = (pooksterLeft.get_actual_velocity() + pooksterRight.get_actual_velocity()) / 2;

    if(prevOutput - currentOutput >= 25){
      stopped = true;
      logger().warning("[VERT] SUDDEN VELOCITY DROP: " + std::to_string(prevOutput) +
                      " -> " + std::to_string(currentOutput) +
                      " | possible collision or stall");
    }

    if(intaking){
      intake.move(-127);
    }

    if(conv){
      conveyor.move(-127);
    }

    // Periodic detailed status every 50 loops (~250ms)
    loopCount++;

    // ── Per-cycle motor imbalance tracking ─────────────────
    {
      double leftVelNow = pooksterLeft.get_actual_velocity();
      double rightVelNow = pooksterRight.get_actual_velocity();
      // Only accumulate when robot is actually trying to move (not starting/stopping)
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

    if (loopCount % 50 == 0) {
      uint32_t elapsed = pros::millis() - startTime;
      double leftVel = pooksterLeft.get_actual_velocity();
      double rightVel = pooksterRight.get_actual_velocity();
      double velDiff = fabs(leftVel) - fabs(rightVel);

      // Get motor temperatures per side
      double leftTemp = pooksterLeft.get_temperature();
      double rightTemp = pooksterRight.get_temperature();
      if (leftTemp > maxLeftTemp) maxLeftTemp = leftTemp;
      if (rightTemp > maxRightTemp) maxRightTemp = rightTemp;

      logger().log("[VERT] t=" + std::to_string(elapsed) + "ms" +
                   " | remaining=" + std::to_string(remaining_distance) +
                   " | accumulated=" + std::to_string(total_distance_traveled) +
                   " | actual_disp=" + std::to_string(actual_displacement) +
                   " | drift=" + std::to_string(accumulation_error));
      logger().log("[VERT]   fwd_out=" + std::to_string(forwardOutput) +
                   " | heading_err=" + std::to_string(heading_error) +
                   " | pose=(" + std::to_string(current_pose.x) +
                   ", " + std::to_string(current_pose.y) +
                   ", " + std::to_string(current_pose.theta) + ")");

      // ── VEERING DIAGNOSTICS ──────────────────────────────
      // This is the key block for diagnosing why the robot veers.
      // velDiff > 0 means LEFT side is faster -> robot veers RIGHT
      // velDiff < 0 means RIGHT side is faster -> robot veers LEFT
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

      // Running average imbalance
      if (velocitySamples > 0) {
        double avgLeft = cumulativeLeftVel / velocitySamples;
        double avgRight = cumulativeRightVel / velocitySamples;
        double avgDiff = avgLeft - avgRight;
        double pctImbalance = (avgLeft + avgRight > 0)
            ? ((avgLeft - avgRight) / ((avgLeft + avgRight) / 2.0)) * 100.0
            : 0;
        logger().log("[VEER] running_avg: L=" + std::to_string(avgLeft) +
                     " R=" + std::to_string(avgRight) +
                     " | avg_diff=" + std::to_string(avgDiff) +
                     " | imbalance=" + std::to_string(pctImbalance) + "%");

        if (fabs(pctImbalance) > 15) {
          std::string diagnosis = pctImbalance > 0
              ? "LEFT motors consistently faster - robot will veer RIGHT. Check RIGHT side gearbox/friction."
              : "RIGHT motors consistently faster - robot will veer LEFT. Check LEFT side gearbox/friction.";
          logger().warning("[VEER] SIGNIFICANT MOTOR IMBALANCE: " +
                          std::to_string(pctImbalance) + "% | " + diagnosis);
        }
      }

      logger().log("[VERT]   clamps=" + std::to_string(clampCount) +
                   " | overshoots=" + std::to_string(signChangeCount));
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
  logger().log("[VERT] accumulated_distance=" + std::to_string(total_distance_traveled) +
               " | accumulation_drift=" + std::to_string(total_distance_traveled - actual_final_displacement) + "in");
  logger().log("[VERT] heading_drift=" + std::to_string(heading_drift) + "deg" +
               " | start_theta=" + std::to_string(initial_theta) +
               " | end_theta=" + std::to_string(finalPose.theta));
  logger().log("[VERT] final_pose=(" + std::to_string(finalPose.x) +
               ", " + std::to_string(finalPose.y) +
               ", " + std::to_string(finalPose.theta) + ")");
  logger().log("[VERT] time=" + std::to_string(totalTime) + "ms" +
               " | loops=" + std::to_string(loopCount) +
               " | output_clamps=" + std::to_string(clampCount) +
               " | overshoots=" + std::to_string(signChangeCount) +
               " | max_fwd=" + std::to_string(maxForwardOutput) +
               " | max_jump=" + std::to_string(maxPositionJump) + "in");

  // ── VEERING SUMMARY ──────────────────────────────────────
  std::string dirLabel = (distance_inches >= 0) ? "FORWARD" : "REVERSE";
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
        // Motors are imbalanced AND robot veered
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
        // Robot veered but motors are balanced - odom or external factor
        logger().warning(std::string("[VEER] Motors are balanced but robot still veered.") +
                        " Possible causes: weight distribution, one side has more" +
                        " wheel friction, uneven field surface, or odom wheel issue" +
                        " causing false heading readings.");
        logger().warning(std::string("[VEER] Check odom WHEEL HEALTH logs above -") +
                        " if L/R odom imbalance is large, an odom wheel" +
                        " may have different contact than the other.");
      }
    } else {
      logger().success("[VEER] heading drift under 3 degrees - straight tracking OK");
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

  if (total_distance_traveled - actual_final_displacement > 1.0) {
    logger().warning("[VERT] DIAGNOSTIC: accumulated " +
                    std::to_string(total_distance_traveled - actual_final_displacement) +
                    "in more than actual displacement. Suggests odometry noise or oscillation " +
                    "is inflating the distance counter. Consider using displacement from start instead.");
  }

  logger().log("======== MOVE_VERTICAL END ==========");
}

struct tubeParams{
  int time;
  int power;
};

void tubeFunction(void* params){
  tubeParams* args = static_cast<tubeParams*>(params);

  int time = args->time;
  int power = args->power;
  for(int i = 0; i < 6; i++){
    pooksterLeft.move(power);
    pooksterRight.move(power * -1);
    
    pros::delay(time);

    pooksterLeft.move(-10);
    pooksterRight.move(-10);

    pros::delay(time);
  }
}

void positionReset(){
  pros::delay(100);
  chassis.setPose(0,0,0);
  pros::delay(100);
}

void tempMovement(int time, int backwards){
  chassis.drive(0, 100 * backwards, 0);
  pros::delay(100);
}

void tempTurn(int time, int backwards){
  pooksterLeft.move(100 * backwards);
  pooksterRight.move(100 * backwards);
  pros::delay(time);
}

void readout(){
    pros::screen::print(TEXT_LARGE, 1, "Power: %d", (int)pooksterLeft.get_actual_velocity());
    pros::delay(10);
    pros::screen::erase();
    pros::delay(1);
}


void autonomous() {
  chassis.setPose(0,0,90);
  arm.toggle();
  pros::delay(50);

  move_vertical(36, false, false);
  pros::delay(100);

  chassis.setPose(0,0,90);
  pros::delay(100);
  pros::delay(100);

  rotate_to(180);
  arm.toggle(); 
  pros::delay(300);

  move_vertical(6, false, false);
  pros::delay(100);

  tubeParams* paramsOne = new tubeParams {300, 127 };
  
  pros::Task tubeIntakeTask(tubeFunction, paramsOne, "Oscillation");
  limitedIntake(2500, 1, -1, 115);
  pros::delay(100);

  move_vertical(-4, false, false);
  pros::delay(100);
  arm.toggle();
  pros::delay(100);

  chassis.setPose(0,0,180);
  pros::delay(100);

  rotate_to(181);
  pros::delay(100);

  move_vertical(-26, false,false);
  pros::delay(100);

  limitedIntake(2400, 1, 1, 117);
  pros::delay(100);
}

void pooksterControls() {
  if (master.get_digital(DIGITAL_L1)) {
    intake.move(45);
    conveyor.move(127);
    releaser.move(127);
  } else {
    if (master.get_digital(DIGITAL_L2)) {
      releaser.move(127);
    } else {
      if(master.get_digital(DIGITAL_R1)){
        intake.move(-127);
        conveyor.move(-127);
        releaser.move(-127);
      } else {
        if(master.get_digital(DIGITAL_R2)){
          intake.move(-127);
          conveyor.move(-127);
          releaser.move(127);
        } else {
          intake.move(0);
          conveyor.move(0);
          releaser.move(0);
        }
      }
    }
  }

  if (master.get_digital_new_press(DIGITAL_RIGHT)) {
    solenoid.toggle();
  } 
 
  if(master.get_digital_new_press(DIGITAL_LEFT)){
    if(lever.is_extended()){
      lever.retract();
    } else {
      lever.extend();
    }
  }

  if(master.get_digital_new_press(DIGITAL_Y)){
    if(arm.is_extended()){
      arm.retract();
    } else {
      arm.extend();
    }
  }
}
 
void opcontrol() {
  logger().log("======== OPCONTROL START ========");

  // ── Opcontrol Drive Diagnostics ──────────────────────────
  // These track motor behavior over time during driver control
  // to diagnose veering, stuck wheels, and gearbox issues.
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

          // Temperature correlation
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