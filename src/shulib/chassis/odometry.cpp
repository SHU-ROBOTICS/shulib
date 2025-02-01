// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include "shulib/chassis/odometry.hpp"
#include "pros/rtos.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/util.hpp"
#include "shulib/logger.hpp"
#include <math.h>

// tracking thread
pros::Task *trackingTask = nullptr;

// telemetry thread
pros::Task *telemetryTask = nullptr;

// global variables
shulib::OdomSensors odomSensors(nullptr, nullptr, nullptr,
                                nullptr); // the sensors to be used for odometry
shulib::Drivetrain drive(0, 0, 0);    // the drivetrain to be used for odometry
shulib::Pose odomPose(0, 0, 0);       // the pose of the robot
shulib::Pose odomSpeed(0, 0, 0);      // the speed of the robot
shulib::Pose odomLocalSpeed(0, 0, 0); // the local speed of the robot
double thetaCorrectionFactor = 1.0;    // correction factor for theta calculations

float prevVertical = 0;
float prevLeft = 0;
float prevRight = 0;
float prevHorizontal = 0;
float prevBack = 0;
float prevImu = 0;

int telemetryDelay = 200;

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

  Pose localPose(0,0,0);
  localPose.theta = ((dR - dL) / (sL - sR)) * thetaCorrectionFactor;

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
  odomPose.y += deltaY * cos(odomPose.theta);
  odomPose.x += deltaY * sin(odomPose.theta);

  odomPose.y += deltaX * sin(odomPose.theta);
  odomPose.x += deltaX * -cos(odomPose.theta);
}

void shulib::init_odometry() {
  shulib::logger().log("Initializing odometry...");
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
        pros::delay(10);
      }
    }};
    shulib::logger().success("Odometry initialized!");
  }
}

void shulib::setThetaCorrectionFactor(double factor) {
    thetaCorrectionFactor = factor;
    logger().log("Set theta correction factor to: " + std::to_string(factor));
}

double shulib::getThetaCorrectionFactor() {
    return thetaCorrectionFactor;
}
  
