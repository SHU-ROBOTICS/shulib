// src/shulib/chassis/odometry.cpp
// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include "shulib/chassis/odometry.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/util.hpp"
#include "shulib/logger.hpp"
#include <math.h>
#include <cmath>

// tracking thread
pros::Task *trackingTask = nullptr;

// telemetry thread
pros::Task *telemetryTask = nullptr;

// global variables
pros::Controller controller(pros::E_CONTROLLER_MASTER);

shulib::OdomSensors odomSensors(nullptr, nullptr, nullptr,
                                nullptr); // the sensors to be used for odometry
shulib::Drivetrain drive(0, 0, 0);    // the drivetrain to be used for odometry
shulib::Pose odomPose(0, 0, 0);       // the pose of the robot
shulib::Pose odomSpeed(0, 0, 0);      // the speed of the robot
shulib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot

double xCorrectionFactor = 1;
double yCorrectionFactor = 1;
double thetaCorrectionFactor = 1;

float prevVertical = 0;
float prevLeft = 0;
float prevRight = 0;
float prevHorizontal = 0;
float prevBack = 0;
float prevImu = 0;

int telemetryDelay = 200;

// ── Wheel Health Monitoring State ────────────────────────────
// These track consecutive zero-delta readings for each wheel.
// If a wheel reads zero while the others are moving, it is likely stuck.
static int leftZeroCount = 0;
static int rightZeroCount = 0;
static int backZeroCount = 0;

// Total travel per wheel since monitoring started (for long-term comparison)
static double leftTotalTravel = 0;
static double rightTotalTravel = 0;
static double backTotalTravel = 0;

// Track how many update cycles have run (for periodic logging)
static int odomCycleCount = 0;

// Threshold: if a wheel reports zero for this many consecutive cycles
// while another wheel is moving, flag it as stuck.
// 50 cycles * 10ms = 500ms of zero readings while others move.
static const int STUCK_THRESHOLD = 50;

// Minimum delta on OTHER wheels to consider "the robot is moving"
// If all wheels read near-zero, the robot is just stationary -- not stuck.
static const double MOVING_THRESHOLD = 0.005; // inches per cycle

// Track if we've already warned about each wheel (don't spam)
static bool leftStuckWarned = false;
static bool rightStuckWarned = false;
static bool backStuckWarned = false;


void shulib::setSensors(shulib::OdomSensors sensors,
                        shulib::Drivetrain drivetrain) {
  odomSensors = sensors;
  drive = drivetrain;
}

shulib::Pose shulib::getPose(bool radians) {
  if (radians)
    return odomPose;
  else
    return shulib::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.theta));
}

void shulib::setPose(shulib::Pose pose, bool radians) {
  if (radians)
    odomPose = pose;
  else
    odomPose = shulib::Pose(pose.x, pose.y, degToRad(pose.theta));
}

shulib::Pose shulib::getSpeed(bool radians) {
  if (radians)
    return odomSpeed;
  else
    return shulib::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.theta));
}

shulib::Pose shulib::getLocalSpeed(bool radians) {
  if (radians)
    return odomLocalSpeed;
  else
    return shulib::Pose(odomLocalSpeed.x, odomLocalSpeed.y,
                        radToDeg(odomLocalSpeed.theta));
}

shulib::Pose shulib::estimatePose(float time, bool radians) {
  // get current position and speed
  Pose curPose = getPose(true);
  Pose localSpeed = getLocalSpeed(true);
  // calculate the change in local position
  Pose deltaLocalPose = localSpeed * time;

  // calculate the future pose
  float avgHeading = curPose.theta + deltaLocalPose.theta / 2;
  Pose futurePose = curPose;
  futurePose.x += deltaLocalPose.y * sin(avgHeading);
  futurePose.y += deltaLocalPose.y * cos(avgHeading);
  futurePose.x += deltaLocalPose.x * -cos(avgHeading);
  futurePose.y += deltaLocalPose.x * sin(avgHeading);
  if (!radians)
    futurePose.theta = radToDeg(futurePose.theta);

  return futurePose;
}

void shulib::update() {
  float sL = odomSensors.left->get_offset();
  float sR = odomSensors.right->get_offset();
  float sS = odomSensors.back->get_offset();

  float dL = odomSensors.left->get_travel_delta();
  float dR = odomSensors.right->get_travel_delta();
  float dS = odomSensors.back->get_travel_delta();

  odomCycleCount++;

  // ── Wheel Health Monitoring ──────────────────────────────
  // Track total travel per wheel
  leftTotalTravel += std::fabs(dL);
  rightTotalTravel += std::fabs(dR);
  backTotalTravel += std::fabs(dS);

  // Check if each wheel is reading zero
  bool leftZero = (std::fabs(dL) < 0.0001);
  bool rightZero = (std::fabs(dR) < 0.0001);
  bool backZero = (std::fabs(dS) < 0.0001);

  // Check if the robot is actually moving (other wheels have significant deltas)
  bool otherWheelsMovingForLeft = (std::fabs(dR) > MOVING_THRESHOLD || std::fabs(dS) > MOVING_THRESHOLD);
  bool otherWheelsMovingForRight = (std::fabs(dL) > MOVING_THRESHOLD || std::fabs(dS) > MOVING_THRESHOLD);
  bool otherWheelsMovingForBack = (std::fabs(dL) > MOVING_THRESHOLD || std::fabs(dR) > MOVING_THRESHOLD);

  // Left wheel stuck detection (rotation sensor port -8, offset -6.5")
  if (leftZero && otherWheelsMovingForLeft) {
    leftZeroCount++;
    if (leftZeroCount >= STUCK_THRESHOLD && !leftStuckWarned) {
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      shulib::logger().error("[STUCK] LEFT ODOM WHEEL (port 8, left side, offset -6.5\")");
      shulib::logger().error("[STUCK] Zero readings for " +
                             std::to_string(leftZeroCount * 10) + "ms while robot is moving");
      shulib::logger().error("[STUCK] Right wheel reads " + std::to_string(dR) +
                             " | Back wheel reads " + std::to_string(dS) +
                             " | Left reads NOTHING");
      shulib::logger().error("[STUCK] CHECK: Is the left odom wheel physically spinning?");
      shulib::logger().error("[STUCK] CHECK: Is the left odom wheel pressed too hard against the ground?");
      shulib::logger().error("[STUCK] CHECK: Is the rotation sensor insert fully seated in port 8?");
      shulib::logger().error("[STUCK] CHECK: Is anything blocking or binding the left odom wheel?");
      shulib::logger().error("[STUCK] IMPACT: Odometry heading will be WRONG. Robot will miscalculate turns.");
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      leftStuckWarned = true;
    }
    // Repeat a shorter reminder every 2 seconds while still stuck
    if (leftStuckWarned && leftZeroCount % 200 == 0) {
      shulib::logger().error("[STUCK] LEFT ODOM WHEEL still stuck (" +
                             std::to_string(leftZeroCount * 10) + "ms total)");
    }
  } else {
    if (leftStuckWarned && !leftZero) {
      shulib::logger().success("[STUCK] LEFT ODOM WHEEL recovered -- reading again");
      leftStuckWarned = false;
    }
    leftZeroCount = 0;
  }

  // Right wheel stuck detection (rotation sensor port 10, offset 6.5")
  if (rightZero && otherWheelsMovingForRight) {
    rightZeroCount++;
    if (rightZeroCount >= STUCK_THRESHOLD && !rightStuckWarned) {
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      shulib::logger().error("[STUCK] RIGHT ODOM WHEEL (port 10, right side, offset 6.5\")");
      shulib::logger().error("[STUCK] Zero readings for " +
                             std::to_string(rightZeroCount * 10) + "ms while robot is moving");
      shulib::logger().error("[STUCK] Left wheel reads " + std::to_string(dL) +
                             " | Back wheel reads " + std::to_string(dS) +
                             " | Right reads NOTHING");
      shulib::logger().error("[STUCK] CHECK: Is the right odom wheel physically spinning?");
      shulib::logger().error("[STUCK] CHECK: Is the right odom wheel pressed too hard against the ground?");
      shulib::logger().error("[STUCK] CHECK: Is the rotation sensor insert fully seated in port 10?");
      shulib::logger().error("[STUCK] CHECK: Is anything blocking or binding the right odom wheel?");
      shulib::logger().error("[STUCK] IMPACT: Odometry heading will be WRONG. Robot will miscalculate turns.");
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      rightStuckWarned = true;
    }
    if (rightStuckWarned && rightZeroCount % 200 == 0) {
      shulib::logger().error("[STUCK] RIGHT ODOM WHEEL still stuck (" +
                             std::to_string(rightZeroCount * 10) + "ms total)");
    }
  } else {
    if (rightStuckWarned && !rightZero) {
      shulib::logger().success("[STUCK] RIGHT ODOM WHEEL recovered -- reading again");
      rightStuckWarned = false;
    }
    rightZeroCount = 0;
  }

  // Back wheel stuck detection (rotation sensor port 9, offset 2.5")
  if (backZero && otherWheelsMovingForBack) {
    backZeroCount++;
    if (backZeroCount >= STUCK_THRESHOLD && !backStuckWarned) {
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      shulib::logger().error("[STUCK] BACK ODOM WHEEL (port 9, rear, offset 2.5\")");
      shulib::logger().error("[STUCK] Zero readings for " +
                             std::to_string(backZeroCount * 10) + "ms while robot is moving");
      shulib::logger().error("[STUCK] Left wheel reads " + std::to_string(dL) +
                             " | Right wheel reads " + std::to_string(dR) +
                             " | Back reads NOTHING");
      shulib::logger().error("[STUCK] CHECK: Is the back odom wheel physically spinning?");
      shulib::logger().error("[STUCK] CHECK: Is the back odom wheel pressed too hard against the ground?");
      shulib::logger().error("[STUCK] CHECK: Is the rotation sensor insert fully seated in port 9?");
      shulib::logger().error("[STUCK] CHECK: Is anything blocking or binding the back odom wheel?");
      shulib::logger().error("[STUCK] IMPACT: Lateral (strafing) position will be WRONG.");
      shulib::logger().error("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
      backStuckWarned = true;
    }
    if (backStuckWarned && backZeroCount % 200 == 0) {
      shulib::logger().error("[STUCK] BACK ODOM WHEEL still stuck (" +
                             std::to_string(backZeroCount * 10) + "ms total)");
    }
  } else {
    if (backStuckWarned && !backZero) {
      shulib::logger().success("[STUCK] BACK ODOM WHEEL recovered -- reading again");
      backStuckWarned = false;
    }
    backZeroCount = 0;
  }

  // ── Intermittent Stuck Detection ────────────────────────
  // A wheel might not be fully stuck but could be skipping or
  // losing contact intermittently. If one wheel's total travel
  // is significantly less than the others over time, flag it.
  if (odomCycleCount % 100 == 0 && odomCycleCount > 200) {
    double maxTravel = std::max({leftTotalTravel, rightTotalTravel, backTotalTravel});
    if (maxTravel > 2.0) {
      // Left and right should be very close for straight movement
      // Back is independent (measures lateral), so only compare L/R to each other
      if (leftTotalTravel > 1.0 || rightTotalTravel > 1.0) {
        double lrMax = std::max(leftTotalTravel, rightTotalTravel);
        if (leftTotalTravel < lrMax * 0.5 && leftTotalTravel < rightTotalTravel) {
          shulib::logger().warning("[ODOM] LEFT wheel total travel is less than HALF of right." +
              std::string(" L=") + std::to_string(leftTotalTravel) +
              " R=" + std::to_string(rightTotalTravel) +
              " | Left wheel may be slipping, skipping, or have intermittent contact.");
        }
        if (rightTotalTravel < lrMax * 0.5 && rightTotalTravel < leftTotalTravel) {
          shulib::logger().warning("[ODOM] RIGHT wheel total travel is less than HALF of left." +
              std::string(" L=") + std::to_string(leftTotalTravel) +
              " R=" + std::to_string(rightTotalTravel) +
              " | Right wheel may be slipping, skipping, or have intermittent contact.");
        }
      }
    }
  }

  // ── Left/Right Imbalance Detection ───────────────────────
  // If both left and right wheels are reading data but one is consistently
  // reading more than the other during what should be straight-line movement,
  // either the robot is actually curving (hardware issue) or one wheel has
  // a different effective diameter or contact.
  //
  // Log every 100 cycles (1 second) with cumulative totals so you can see
  // if one side is consistently reading more than the other over time.
  if (odomCycleCount % 100 == 0) {
    double imbalance = 0;
    if (leftTotalTravel + rightTotalTravel > 1.0) {
      // Percentage difference: positive means right reads more, negative means left reads more
      imbalance = ((rightTotalTravel - leftTotalTravel) / ((leftTotalTravel + rightTotalTravel) / 2.0)) * 100.0;
    }

    shulib::logger().log("[ODOM] WHEEL HEALTH t=" + std::to_string(odomCycleCount / 100) + "s" +
                         " | L_total=" + std::to_string(leftTotalTravel) +
                         " R_total=" + std::to_string(rightTotalTravel) +
                         " B_total=" + std::to_string(backTotalTravel) +
                         " | L/R_imbalance=" + std::to_string(imbalance) + "%");

    if (std::fabs(imbalance) > 10.0 && leftTotalTravel + rightTotalTravel > 5.0) {
      std::string moreWheel = imbalance > 0 ? "RIGHT" : "LEFT";
      std::string lessWheel = imbalance > 0 ? "LEFT" : "RIGHT";
      shulib::logger().warning("[ODOM] L/R ODOM IMBALANCE: " + std::to_string(imbalance) + "%");
      shulib::logger().warning("[ODOM] " + moreWheel + " odom wheel reading MORE than " + lessWheel);
      shulib::logger().warning("[ODOM] This means EITHER:");
      shulib::logger().warning("[ODOM]   1) Robot is physically curving (" + moreWheel +
          " side travels further) -> check drive motors/gearbox");
      shulib::logger().warning("[ODOM]   2) " + lessWheel +
          " odom wheel has too much pressure / friction -> reads less than actual travel");
      shulib::logger().warning("[ODOM]   3) " + moreWheel +
          " odom wheel has too little contact / bouncing -> reads more (noisy)");
      shulib::logger().warning("[ODOM] COMPARE with [OPCTL] or [VEER] motor velocity logs." +
          std::string(" If motors are balanced but odom is not, it's an odom wheel issue.") +
          " If motors are also imbalanced, it's a drive issue.");
    }
  }

  // ── Periodic Raw Delta Log ───────────────────────────────
  // Every 50 cycles (500ms), log the instantaneous deltas.
  // Useful for spotting intermittent issues (wheel skipping, bouncing).
  if (odomCycleCount % 50 == 0) {
    // Guard against divide-by-zero if sL == sR (would mean offsets are identical, which is a config error)
    double headingChange = (sL != sR) ? ((dR - dL) / (sL - sR)) : 0;
    shulib::logger().log("[ODOM] DELTAS dL=" + std::to_string(dL) +
                         " dR=" + std::to_string(dR) +
                         " dS=" + std::to_string(dS) +
                         " | heading_change=" + std::to_string(headingChange));
  }

  // ── Standard Odometry Math (unchanged) ───────────────────
  Pose localPose(0, 0, 0);
  localPose.theta = (dR - dL) / (sL - sR) * thetaCorrectionFactor;

  float deltaX = 0;
  float deltaY = 0;
  float rC = 0;

  odomPose.theta += localPose.theta;
  if (abs(localPose.theta) < 0.0001) {  // Check for very small angles
    deltaY = (dL + dR) / 2;
    deltaX = dS;
  } else {
    rC = (dR / localPose.theta) + sR;
    deltaY = 2 * sin(localPose.theta / 2) * rC;

    rC = (dS / localPose.theta) + sS;
    deltaX = 2 * sin(localPose.theta / 2) * rC;
  }

  // set odomPose
  odomPose.y += deltaY * cos(odomPose.theta) * yCorrectionFactor;
  odomPose.x += deltaY * sin(odomPose.theta) * yCorrectionFactor;

  odomPose.y += deltaX * sin(odomPose.theta) * xCorrectionFactor;
  odomPose.x += deltaX * -cos(odomPose.theta) * xCorrectionFactor;
}

void shulib::init_odometry() {
  shulib::logger().log("Initializing odometry...");

  // Reset wheel health monitoring
  leftZeroCount = 0;
  rightZeroCount = 0;
  backZeroCount = 0;
  leftTotalTravel = 0;
  rightTotalTravel = 0;
  backTotalTravel = 0;
  odomCycleCount = 0;
  leftStuckWarned = false;
  rightStuckWarned = false;
  backStuckWarned = false;

  if (trackingTask == nullptr) {
    trackingTask = new pros::Task{[=] {
      shulib::Pose lastLoggedPose(0, 0, 0);
      while (true) {
        update();
        if (abs(odomPose.x - lastLoggedPose.x) > 0.1 ||
            abs(odomPose.y - lastLoggedPose.y) > 0.1 ||
            abs(odomPose.theta - lastLoggedPose.theta) > 0.1) {
          shulib::logger().updateTelemetry("odometry", odomPose);
          lastLoggedPose = odomPose;
        }
        shulib::logger().updateTelemetry("temps", drive.getTemps());
        std::string batteryTelemetry = "{\"voltage\":" + std::to_string(pros::battery::get_voltage()) +
        ", \"current\":" + std::to_string(pros::battery::get_current()) +
        ", \"temperature\":" + std::to_string(pros::battery::get_temperature()) +
        ", \"capacity\":" + std::to_string(pros::battery::get_capacity()) +
        "}";
        shulib::logger().updateTelemetry("battery", batteryTelemetry);
        std::string controllerTelemetry = "{\"capacity\":" + std::to_string(controller.get_battery_capacity()) +
        ", \"level\":" + std::to_string(controller.get_battery_level()) +
        "}";
        shulib::logger().updateTelemetry("controller", controllerTelemetry);
        pros::delay(10);
      }
    }};
    shulib::logger().success("Odometry initialized!");
  }
}

void shulib::setXCorrectionFactor(double factor) {
    xCorrectionFactor = factor;
    logger().log("Set x correction factor to: " + std::to_string(factor));
}

void shulib::setYCorrectionFactor(double factor) {
    yCorrectionFactor = factor;
    logger().log("Set y correction factor to: " + std::to_string(factor));
}

void shulib::setThetaCorrectionFactor(double factor) {
    thetaCorrectionFactor = factor;
    logger().log("Set theta correction factor to: " + std::to_string(factor));
}

double shulib::getThetaCorrectionFactor() {
    return thetaCorrectionFactor;
}