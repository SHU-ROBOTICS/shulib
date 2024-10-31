// The implementation below is mostly based off of
// the document written by 5225A (Pilons)
// Here is a link to the original document
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf

#include "shulib/chassis/odometry.hpp"
#include "pros/rtos.hpp"
#include "shulib/chassis/chassis.hpp"
#include "shulib/chassis/odomUnit.hpp"
#include "shulib/util.hpp"
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

  float deltaX = 0;
  float deltaY = 0;
  float deltaTheta = (dR - dL) / (sL - sR);
  float rC = 0;

  if (deltaTheta == 0) {
    deltaX = (dL + dR) / 2;
    deltaY = dS;
  } else {
    rC = (dR / deltaTheta) + sR;
    deltaX = 2 * sin(deltaTheta / 2) * rC;

    rC = (dS / deltaTheta) + sS;
    deltaY = 2 * sin(deltaTheta / 2) * rC;
  }

  // set odomPose
  odomPose.theta -= deltaTheta;
  odomPose.y += deltaX;
  odomPose.x += deltaY;
}

void shulib::init() {
  if (trackingTask == nullptr) {
    trackingTask = new pros::Task{[=] {
      while (true) {
        update();
        pros::delay(10);
      }
    }};
  }
  if (telemetryTask == nullptr && telemetryDelay > 0) {
    telemetryTask = new pros::Task{[=] {
      while (true) {
        std::string odomData = odomPose;
        printf("{'odometry':%s}\n", odomData.c_str());
        pros::delay(telemetryDelay);
      }
    }};
  }
}